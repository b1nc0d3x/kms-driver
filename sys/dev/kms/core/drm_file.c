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

#include <kms/drm_device.h>
#include <kms/drm_file.h>

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

struct cdevsw kms_cdevsw = {
	.d_version =	D_VERSION,
	.d_name =	"kms",
	.d_open =	kms_open,
	.d_ioctl =	kms_ioctl,
};
