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
#include <sys/refcount.h>
#include <sys/selinfo.h>
#include <sys/sx.h>
#include <sys/uio.h>

#include <vm/vm.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>

#include <kms/drm_crtc.h>
#include <kms/drm_device.h>
#include <kms/drm_drv.h>
#include <kms/drm_file.h>
#include <kms/drm_framebuffer.h>
#include <kms/drm_gem.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_property.h>

#include "kms_internal.h"

/*
 * H4 drm_file refcount helpers.  IRQ paths in drm_events.c hold a ref
 * across the "detach event under mc->mutex → send it after dropping the
 * lock" window so a concurrent close() can't free file out from under
 * the delivery.
 */
void
kms_file_get(struct drm_file *file)
{
	if (file != NULL)
		refcount_acquire(&file->refs);
}

static void kms_file_free(struct drm_file *file);

void
kms_file_put(struct drm_file *file)
{
	if (file != NULL && refcount_release(&file->refs))
		kms_file_free(file);
}

/*
 * Actually free the drm_file's storage + drop the device ref.  Runs
 * only when the last reference goes away — either from kms_file_dtor
 * (nothing else was holding a ref) or from an IRQ handler completing
 * the last in-flight event delivery after close() already ran.
 */
static void
kms_file_free(struct drm_file *file)
{
	struct drm_device *dev;

	if (file == NULL)
		return;
	dev = file->dev;
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
	 * drm_file that HASN'T already been detached to an IRQ-side
	 * ready list.  Entries still queued here get their pe->file ref
	 * released as we free them.  Entries the vblank handler has
	 * already moved to its local ready list keep an extra ref via
	 * kms_file_get() so their kms_send_event runs safely after we
	 * drop this file's d_open ref below.
	 *
	 * Under mc->mutex so the vblank handler sees a consistent view.
	 */
	mc = &dev->mode_config;
	/*
	 * M2 (2026-08-03 review): orphan any property blobs the closing
	 * file created, so no other client can destroy them via
	 * DESTROYPROPBLOB after we drop below.  Blobs stay reachable by
	 * id; only the master can then destroy them.
	 */
	kms_property_blob_orphan_owner(dev, file);
	sx_xlock(&mc->mutex);
	TAILQ_FOREACH(obj, &mc->crtcs, link) {
		crtc = __containerof(obj, struct drm_crtc, base);
		if (crtc->pending_flip_file == file) {
			crtc->pending_flip_file = NULL;
			crtc->pending_flip_user_data = 0;
			/* Drop the ref taken at PAGE_FLIP arm-time. */
			refcount_release(&file->refs);
			if (crtc->pending_flip_old_fb != NULL) {
				kms_mode_object_put(
				    &crtc->pending_flip_old_fb->base);
				crtc->pending_flip_old_fb = NULL;
			}
		}
		TAILQ_FOREACH_SAFE(pe, &crtc->pending_vblank_events,
		    link, pe_next) {
			if (pe->file != file)
				continue;
			TAILQ_REMOVE(&crtc->pending_vblank_events, pe, link);
			/* Drop the ref taken at WAIT_VBLANK queue-time. */
			refcount_release(&file->refs);
			free(pe, M_KMS);
		}
	}
	sx_xunlock(&mc->mutex);

	/*
	 * H5 (2026-08-03 review): give the driver a chance to release
	 * per-file state stashed in file->driver_priv — most importantly
	 * igen's softpin GGTT bindings that would otherwise outlive the
	 * fd and keep PTEs pointing at freed pages.  Called after we
	 * detached from mc->pending lists (so IRQs won't reference this
	 * file's driver state) but before kms_file_put releases the
	 * storage (so the driver still has a valid `file`).
	 *
	 * 3rd-review L-3: snapshot dev->driver with atomic_load_ptr so a
	 * concurrent kms_dev_unregister setting it to NULL under dev_lock
	 * is observed atomically here.  Without the snapshot we'd race a
	 * torn 8-byte load on some architectures and could deref garbage.
	 */
	{
		const struct drm_driver *drv =
		    (const struct drm_driver *)atomic_load_ptr(&dev->driver);

		if (drv != NULL && drv->file_free != NULL) {
			drv->file_free(file);
			file->driver_priv = NULL;
		}
	}

	/* Drop the d_open reference.  Frees storage if refs hits 0. */
	kms_file_put(file);
	(void)dev;
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
	refcount_init(&file->refs, 1);	/* d_open ref, dropped in dtor */
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
 * Userspace mmap on the cdev.  MAP_DUMB returned an mmap_offset; the
 * userspace mmap() syscall carries that offset to us and we map it
 * back to the GEM object that owns it, then hand back the pre-built
 * cdev_pager.  vm_object_reference bumps the pager's refcount so it
 * outlives the GEM object's handle-table ref (lets userspace keep
 * a live mapping past DESTROY_DUMB, matching Linux semantics).
 *
 * Reverted from d_mmap (page-at-a-time) after H1 review flagged a
 * page UAF: d_mmap never held a pager ref, so GEM_CLOSE freed the
 * pages while userspace still had valid PTEs to them.  The pager
 * already has VM_MEMATTR_UNCACHEABLE (via vm_object_set_memattr in
 * kms_gem_object_create on arm64) + per-page pmap_page_set_memattr,
 * so the original reason for d_mmap (memattr control) is now handled
 * by the pager itself.  d_mmap_single lifetime semantics are correct.
 */
static int
kms_mmap_single(struct cdev *cdev, vm_ooffset_t *offset, vm_size_t size,
    vm_object_t *object, int prot __unused)
{
	struct drm_device *dev;
	struct drm_file *file;
	struct drm_gem_object *obj;

	dev = cdev->si_drv1;
	if (dev == NULL)
		return (ENXIO);

	/*
	 * H2 ownership check: mmap offsets are device-global, so any
	 * open of /dev/dri/cardN could otherwise walk offsets and map a
	 * different fd's BOs (including the compositor's scanout fb).
	 * Require the calling drm_file to hold at least one handle
	 * that references the same GEM object.  Mirrors Linux's
	 * drm_vma_node_allow() gate.
	 */
	if (devfs_get_cdevpriv((void **)&file) != 0 || file == NULL)
		return (EACCES);

	obj = kms_gem_object_lookup_offset(dev, (uint64_t)*offset);
	if (obj == NULL)
		return (EINVAL);
	if (size > obj->size) {
		kms_gem_object_put(obj);
		return (EINVAL);
	}
	if (!kms_file_owns_gem(file, obj)) {
		kms_gem_object_put(obj);
		return (EACCES);
	}

	/*
	 * Bump the pager's reference for the user mapping.  The lookup
	 * already pinned the GEM object via its own refcount; the put
	 * below drops that and leaves only the pager ref to keep pages
	 * alive while the mapping exists.
	 */
	vm_object_reference(obj->pager);
	*object = obj->pager;
	*offset = 0;	/* page index within the returned vm_object */

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
 * Wired to file->event_select.si_note.  Wake path is a
 * KNOTE_UNLOCKED() called explicitly next to selwakeup() in
 * kms_send_event — selwakeup itself only walks si_tdlist (poll/select)
 * and never fans out to si_note.  Missing that KNOTE call was M1 in
 * the 2026-08-03 review.
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
	.d_mmap_single =	kms_mmap_single,
};
