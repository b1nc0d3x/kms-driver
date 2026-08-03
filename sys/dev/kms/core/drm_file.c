/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/fcntl.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/poll.h>
#include <sys/queue.h>
#include <sys/selinfo.h>
#include <sys/sx.h>
#include <sys/uio.h>

#include <vm/vm.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>

#include <kms/drm_crtc.h>
#include <kms/drm_device.h>
#include <kms/drm_file.h>
#include <kms/drm_gem.h>
#include <kms/drm_mode_config.h>

#include "kms_internal.h"

static void
kms_file_dtor(void *data)
{
	struct drm_file *file = data;
	struct drm_device *dev;
	struct drm_mode_config *mc;
	struct drm_mode_object *obj;
	struct drm_crtc *crtc;
	struct kms_pending_vblank_event *pe, *pe_next;

	if (file == NULL)
		return;
	dev = file->dev;

	sx_xlock(&dev->dev_lock);
	TAILQ_REMOVE(&dev->files, file, link);
	if (dev->open_count > 0)
		dev->open_count--;
	sx_xunlock(&dev->dev_lock);

	/*
	 * Scrub every reference the mode_config still holds to this
	 * drm_file BEFORE we free it.  A vblank IRQ that fires between
	 * here and the free would otherwise dereference a dangling
	 * pointer: `crtc->pending_flip_file` and every
	 * `pending_vblank_events` entry can point at us.
	 *
	 * Under mc->mutex so the vblank handler (drm_events.c:kms_vblank_
	 * handler) sees a consistent view — either the entry is still
	 * queued (before we take the lock) or it's gone (after).
	 */
	mc = &dev->mode_config;
	sx_xlock(&mc->mutex);
	TAILQ_FOREACH(obj, &mc->crtcs, link) {
		crtc = __containerof(obj, struct drm_crtc, base);
		if (crtc->pending_flip_file == file) {
			crtc->pending_flip_file = NULL;
			crtc->pending_flip_user_data = 0;
		}
		TAILQ_FOREACH_SAFE(pe, &crtc->pending_vblank_events,
		    link, pe_next) {
			if (pe->file != file)
				continue;
			TAILQ_REMOVE(&crtc->pending_vblank_events, pe, link);
			free(pe, M_KMS);
		}
	}
	sx_xunlock(&mc->mutex);

	/*
	 * Walk the handle table and drop the ref each one holds.  Done
	 * outside dev_lock since gem_object_put may take dev->gem_lock
	 * and we never want a lock ordering edge from dev_lock down to
	 * gem_lock.
	 */
	kms_gem_release_all(file);
	sx_destroy(&file->handle_lock);
	kms_syncobj_release_all(file);
	sx_destroy(&file->syncobj_lock);
	kms_event_queue_drain(file);

	free(file, M_KMS);
	/*
	 * Drop the device reference acquired in open.  If we're the last
	 * holder (registry already released its initial ref via
	 * dev_unregister), this frees the device storage.
	 */
	kms_device_release(dev);
}

static int
kms_open(struct cdev *cdev, int oflags __unused, int devtype __unused,
    struct thread *td __unused)
{
	struct drm_device *dev;
	struct drm_file *file;
	int error;

	dev = cdev->si_drv1;
	if (dev == NULL)
		return (ENXIO);

	/*
	 * Pin the device for the lifetime of this open.  The matching
	 * release runs in kms_file_dtor when the last reference to
	 * the file descriptor goes away.  Acquired before any state is
	 * published so the cleanup path is symmetric.
	 */
	kms_device_acquire(dev);

	file = malloc(sizeof(*file), M_KMS, M_WAITOK | M_ZERO);
	file->dev = dev;
	file->authenticated = false;
	file->is_master = false;
	/*
	 * Tag opens of /dev/dri/renderD<128+N> so kms_ioctl can reject the
	 * KMS-only ioctls per Linux render-node ABI.  Compare the cdev we
	 * were opened on to the device's render_cdev pointer rather than
	 * sniff the minor — render_cdev may be NULL when the render node
	 * wasn't created (kms_dev_register continues without it).
	 */
	file->is_render_node = (cdev == dev->render_cdev);
	file->magic = 0;
	sx_init(&file->handle_lock, "drmgem");
	TAILQ_INIT(&file->handles);
	file->next_handle = 0;
	sx_init(&file->syncobj_lock, "drmsync");
	TAILQ_INIT(&file->syncobjs);
	file->next_syncobj_handle = 0;
	kms_event_queue_init(file);

	sx_xlock(&dev->dev_lock);
	TAILQ_INSERT_TAIL(&dev->files, file, link);
	dev->open_count++;
	/*
	 * First opener becomes implicit master so simple userspace
	 * (single-Xorg / single-Wayland) doesn't need SET_MASTER.
	 */
	if (dev->open_count == 1)
		file->is_master = true;
	sx_xunlock(&dev->dev_lock);

	error = devfs_set_cdevpriv(file, kms_file_dtor);
	if (error != 0) {
		/*
		 * dtor drops the device ref; do not double-release.
		 */
		kms_file_dtor(file);
		return (error);
	}
	return (0);
}

/*
 * Userspace mmap on the cdev — page-at-a-time variant.  Returns the
 * physical address that backs a specific byte offset within a GEM
 * object's MAP_DUMB region, with an explicit memattr so the resulting
 * PTE gets the right cache-attribute bits.
 *
 * We take this path (instead of d_mmap_single with a cdev_pager) so
 * we control the memattr per PTE.  On arm64 the cdev_pager MGTDEVICE
 * path silently drops per-page memattr for FICTITIOUS pages, which
 * left Xorg's writes stranded in a cached alias that the VOP DMA
 * never observed.  Direct d_mmap forces UNCACHEABLE PTEs so user
 * writes land in DRAM in time for scan-out.
 */
static int
kms_mmap(struct cdev *cdev, vm_ooffset_t offset, vm_paddr_t *paddr,
    int nprot __unused, vm_memattr_t *memattr)
{
	struct drm_device *dev;
	struct drm_gem_object *obj;
	uint64_t base;
	uint32_t page_idx;

	dev = cdev->si_drv1;
	if (dev == NULL)
		return (ENXIO);

	obj = kms_gem_object_lookup_offset_containing(dev, (uint64_t)offset,
	    &base);
	if (obj == NULL)
		return (EINVAL);
	page_idx = (uint32_t)((offset - base) / PAGE_SIZE);
	if (page_idx >= obj->npages) {
		kms_gem_object_put(obj);
		return (EINVAL);
	}
	*paddr = VM_PAGE_TO_PHYS(obj->pages[page_idx]);
	if (memattr != NULL) {
#ifdef __aarch64__
		*memattr = VM_MEMATTR_UNCACHEABLE;
#else
		*memattr = VM_MEMATTR_DEFAULT;
#endif
	}
	kms_gem_object_put(obj);
	return (0);
}

/*
 * Drain the file's event queue into a user uio.  Returns whole events
 * only — if the next event doesn't fit in the remaining buffer, stop
 * (matches Linux DRM semantics).  Blocks if the queue is empty unless
 * O_NONBLOCK is set on the fd.
 */
static int
kms_read(struct cdev *cdev __unused, struct uio *uio, int ioflag)
{
	struct drm_file *file;
	struct drm_pending_event *ev;
	int error;

	error = devfs_get_cdevpriv((void **)&file);
	if (error != 0)
		return (error);

	mtx_lock(&file->event_mtx);
	for (;;) {
		ev = TAILQ_FIRST(&file->events);
		if (ev != NULL)
			break;
		if (ioflag & O_NONBLOCK) {
			mtx_unlock(&file->event_mtx);
			return (EAGAIN);
		}
		error = msleep(&file->events, &file->event_mtx, PCATCH,
		    "drmrd", 0);
		if (error != 0) {
			mtx_unlock(&file->event_mtx);
			return (error);
		}
	}

	while (ev != NULL && (size_t)uio->uio_resid >= ev->length) {
		TAILQ_REMOVE(&file->events, ev, link);
		file->events_bytes -= ev->length;
		mtx_unlock(&file->event_mtx);
		error = uiomove(ev->data, ev->length, uio);
		if (error != 0) {
			/*
			 * uiomove failed (typically EFAULT — user buffer
			 * moved out from under us).  Re-queue the event at
			 * the head so a well-formed retry from userspace
			 * can consume it; freeing here would silently drop
			 * a pageflip / vblank event and stall the
			 * compositor.  Match Linux drm_read on -EFAULT.
			 */
			mtx_lock(&file->event_mtx);
			TAILQ_INSERT_HEAD(&file->events, ev, link);
			file->events_bytes += ev->length;
			mtx_unlock(&file->event_mtx);
			return (error);
		}
		free(ev->data, M_KMS);
		free(ev, M_KMS);
		mtx_lock(&file->event_mtx);
		ev = TAILQ_FIRST(&file->events);
	}
	mtx_unlock(&file->event_mtx);
	return (0);
}

static int
kms_poll(struct cdev *cdev __unused, int events, struct thread *td)
{
	struct drm_file *file;
	int revents = 0;

	if (devfs_get_cdevpriv((void **)&file) != 0)
		return (POLLERR);
	if (events & (POLLIN | POLLRDNORM)) {
		mtx_lock(&file->event_mtx);
		if (!TAILQ_EMPTY(&file->events))
			revents |= events & (POLLIN | POLLRDNORM);
		else
			selrecord(td, &file->event_select);
		mtx_unlock(&file->event_mtx);
	}
	return (revents);
}

/*
 * kqueue EVFILT_READ support (M2).  wlroots / Weston / kwin on
 * FreeBSD reach the DRM fd through libepoll-shim which is kqueue
 * underneath — without a d_kqfilter, EVFILT_READ registration on
 * /dev/dri/cardN returns EINVAL and the compositor can't wait on
 * page-flip events.
 *
 * Wired to file->event_select.si_note; selwakeup() in kms_send_event
 * triggers KNOTE fanout so we don't need a separate wake path.
 */
static void
kms_kqrdetach(struct knote *kn)
{
	struct drm_file *file = kn->kn_hook;

	knlist_remove(&file->event_select.si_note, kn, 0);
}

static int
kms_kqrevent(struct knote *kn, long hint __unused)
{
	struct drm_file *file = kn->kn_hook;

	return (TAILQ_EMPTY(&file->events) ? 0 : 1);
}

static const struct filterops kms_read_filtops = {
	.f_isfd		= 1,
	.f_detach	= kms_kqrdetach,
	.f_event	= kms_kqrevent,
};

static int
kms_kqfilter(struct cdev *cdev __unused, struct knote *kn)
{
	struct drm_file *file;

	if (devfs_get_cdevpriv((void **)&file) != 0 || file == NULL)
		return (ENXIO);
	if (kn->kn_filter != EVFILT_READ)
		return (EINVAL);
	kn->kn_fop = &kms_read_filtops;
	kn->kn_hook = file;
	knlist_add(&file->event_select.si_note, kn, 0);
	return (0);
}

struct cdevsw kms_cdevsw = {
	.d_version =		D_VERSION,
	.d_name =		"kms",
	.d_open =		kms_open,
	.d_read =		kms_read,
	.d_ioctl =		kms_ioctl,
	.d_poll =		kms_poll,
	.d_kqfilter =		kms_kqfilter,
	.d_mmap =		kms_mmap,
};
