/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/malloc.h>
#include <sys/queue.h>
#include <sys/sx.h>

#include <vm/vm.h>
#include <vm/vm_object.h>

#include <kms/drm_device.h>
#include <kms/drm_file.h>
#include <kms/drm_gem.h>

#include "kms_internal.h"

static void
kms_file_dtor(void *data)
{
	struct drm_file *file = data;
	struct drm_device *dev;

	if (file == NULL)
		return;
	dev = file->dev;

	sx_xlock(&dev->dev_lock);
	TAILQ_REMOVE(&dev->files, file, link);
	if (dev->open_count > 0)
		dev->open_count--;
	sx_xunlock(&dev->dev_lock);

	/*
	 * Walk the handle table and drop the ref each one holds.  Done
	 * outside dev_lock since gem_object_put may take dev->gem_lock
	 * and we never want a lock ordering edge from dev_lock down to
	 * gem_lock.
	 */
	kms_gem_release_all(file);
	sx_destroy(&file->handle_lock);

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
	file->magic = 0;
	sx_init(&file->handle_lock, "drmgem");
	TAILQ_INIT(&file->handles);
	file->next_handle = 0;

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
 */
static int
kms_mmap_single(struct cdev *cdev, vm_ooffset_t *offset, vm_size_t size,
    vm_object_t *object, int prot __unused)
{
	struct drm_device *dev;
	struct drm_gem_object *obj;

	dev = cdev->si_drv1;
	if (dev == NULL)
		return (ENXIO);

	obj = kms_gem_object_lookup_offset(dev, (uint64_t)*offset);
	if (obj == NULL)
		return (EINVAL);
	if (size > obj->size) {
		kms_gem_object_put(obj);
		return (EINVAL);
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

struct cdevsw kms_cdevsw = {
	.d_version =		D_VERSION,
	.d_name =		"kms",
	.d_open =		kms_open,
	.d_ioctl =		kms_ioctl,
	.d_mmap_single =	kms_mmap_single,
};
