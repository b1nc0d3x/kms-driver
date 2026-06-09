/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/queue.h>
#include <sys/refcount.h>
#include <sys/sx.h>

#include <kms/drm_device.h>
#include <kms/drm_drv.h>
#include <kms/drm_mode_config.h>

#include "kms_internal.h"

static void	kms_device_destroy(struct drm_device *dev);

MALLOC_DEFINE(M_KMS, "kms", "DRM compatibility framework");

static struct sx	kms_registry_lock;
static TAILQ_HEAD(, drm_device) kms_devices =
    TAILQ_HEAD_INITIALIZER(kms_devices);
static int		kms_next_minor = 0;

int
kms_dev_register(const struct drm_driver *driver, void *driver_priv,
    struct drm_device **out_dev)
{
	struct drm_device *dev;
	struct make_dev_args args;
	int error;

	if (driver == NULL || driver->name == NULL || out_dev == NULL)
		return (EINVAL);

	dev = malloc(sizeof(*dev), M_KMS, M_WAITOK | M_ZERO);
	sx_init(&dev->dev_lock, "drmdev");
	sx_init(&dev->gem_lock, "drmgem_dev");
	dev->driver = driver;
	dev->driver_priv = driver_priv;
	TAILQ_INIT(&dev->files);
	TAILQ_INIT(&dev->gem_objects);
	dev->mmap_offset_counter = PAGE_SIZE;	/* keep 0 reserved */
	kms_mode_config_init(&dev->mode_config);
	refcount_init(&dev->refs, 1);	/* initial: held by the registry */
	kms_mode_config_standard_properties_init(dev);

	make_dev_args_init(&args);
	args.mda_flags = MAKEDEV_CHECKNAME;	/* EEXIST instead of panic */
	args.mda_devsw = &kms_cdevsw;
	args.mda_uid = UID_ROOT;
	args.mda_gid = GID_VIDEO;
	args.mda_mode = 0660;
	args.mda_si_drv1 = dev;

	/*
	 * Walk forward through dri/cardN until make_dev_s finds an unused
	 * slot.  Coexistence requirement: drm2 (or another DRM stack) may
	 * already own card0..cardN — claim the first free minor instead
	 * of failing.  Registry lock is held across the loop so concurrent
	 * registrations don't both race on the same number.
	 */
	sx_xlock(&kms_registry_lock);
	for (;;) {
		if (kms_next_minor >= 256) {
			sx_xunlock(&kms_registry_lock);
			kms_device_destroy(dev);
			return (ENOSPC);
		}
		dev->minor = kms_next_minor++;
		error = make_dev_s(&args, &dev->cdev, "dri/card%d",
		    dev->minor);
		if (error == 0)
			break;
		if (error != EEXIST) {
			sx_xunlock(&kms_registry_lock);
			kms_device_destroy(dev);
			return (error);
		}
	}
	TAILQ_INSERT_TAIL(&kms_devices, dev, link);
	sx_xunlock(&kms_registry_lock);

	*out_dev = dev;
	printf("kms: registered /dev/dri/card%d driver=%s\n",
	    dev->minor, driver->name);
	return (0);
}

/*
 * Tear down a drm_device's storage.  Called only when the refcount
 * reaches zero — either from kms_dev_unregister (no outstanding
 * opens) or from the last kms_file_dtor after a deferred
 * unregister.  Cdev destruction happens earlier, in
 * kms_dev_unregister, so no new opens can race here.
 */
static void
kms_device_destroy(struct drm_device *dev)
{
	kms_mode_config_cleanup(&dev->mode_config);
	sx_destroy(&dev->gem_lock);
	sx_destroy(&dev->dev_lock);
	free(dev, M_KMS);
}

void
kms_device_release(struct drm_device *dev)
{
	if (dev == NULL)
		return;
	if (refcount_release(&dev->refs))
		kms_device_destroy(dev);
}

void
kms_device_acquire(struct drm_device *dev)
{
	refcount_acquire(&dev->refs);
}

void
kms_dev_unregister(struct drm_device *dev)
{
	if (dev == NULL)
		return;

	sx_xlock(&kms_registry_lock);
	TAILQ_REMOVE(&kms_devices, dev, link);
	sx_xunlock(&kms_registry_lock);

	/*
	 * Drop the cdev first so no new open() can grab a fresh
	 * reference, then release the initial registry ref.  If any
	 * open fds remain, they each hold a ref and the final free
	 * happens when the last one runs its file dtor.
	 */
	if (dev->cdev != NULL) {
		destroy_dev(dev->cdev);
		dev->cdev = NULL;
	}
	kms_device_release(dev);
}

static int
kms_modevent(module_t mod __unused, int what, void *arg __unused)
{
	switch (what) {
	case MOD_LOAD:
		sx_init(&kms_registry_lock, "kms_reg");
		printf("kms: loaded\n");
		return (0);
	case MOD_UNLOAD:
		sx_xlock(&kms_registry_lock);
		if (!TAILQ_EMPTY(&kms_devices)) {
			sx_xunlock(&kms_registry_lock);
			return (EBUSY);
		}
		sx_xunlock(&kms_registry_lock);
		sx_destroy(&kms_registry_lock);
		printf("kms: unloaded\n");
		return (0);
	}
	return (EOPNOTSUPP);
}

static moduledata_t kms_mod = {
	"kms",
	kms_modevent,
	NULL,
};
DECLARE_MODULE(kms, kms_mod, SI_SUB_DRIVERS, SI_ORDER_ANY);
MODULE_VERSION(kms, 1);
