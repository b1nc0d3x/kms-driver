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
#include <sys/sx.h>

#include <kms/drm_device.h>
#include <kms/drm_drv.h>
#include <kms/drm_mode_config.h>

#include "kms_internal.h"

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
	dev->driver = driver;
	dev->driver_priv = driver_priv;
	TAILQ_INIT(&dev->files);
	drm_mode_config_init(&dev->mode_config);

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
			drm_mode_config_cleanup(&dev->mode_config);
			sx_destroy(&dev->dev_lock);
			free(dev, M_KMS);
			return (ENOSPC);
		}
		dev->minor = kms_next_minor++;
		error = make_dev_s(&args, &dev->cdev, "dri/card%d",
		    dev->minor);
		if (error == 0)
			break;
		if (error != EEXIST) {
			sx_xunlock(&kms_registry_lock);
			drm_mode_config_cleanup(&dev->mode_config);
			sx_destroy(&dev->dev_lock);
			free(dev, M_KMS);
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

void
kms_dev_unregister(struct drm_device *dev)
{
	if (dev == NULL)
		return;

	sx_xlock(&kms_registry_lock);
	TAILQ_REMOVE(&kms_devices, dev, link);
	sx_xunlock(&kms_registry_lock);

	if (dev->cdev != NULL)
		destroy_dev(dev->cdev);
	drm_mode_config_cleanup(&dev->mode_config);
	sx_destroy(&dev->dev_lock);
	free(dev, M_KMS);
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
