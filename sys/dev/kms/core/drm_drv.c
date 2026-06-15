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
#include <sys/sysctl.h>

#include <kms/drm_device.h>
#include <kms/drm_drv.h>
#include <kms/drm_mode_config.h>

#include "kms_internal.h"

/*
 * Root hw.dri node.  Created on the first call to kms_set_busid_pci so
 * we don't allocate sysctl state we never need.  Reference-counted —
 * destroyed when the last drm_device with a busid is unregistered.
 *
 * libdrm's get_sysctl_pci_bus_info expects this exact layout:
 *   hw.dri.<minor>.busid = "pci:DDDD:BB:SS.F"
 */
static struct sysctl_ctx_list	kms_hw_dri_ctx;
static struct sysctl_oid	*kms_hw_dri_oid = NULL;
static int			 kms_hw_dri_refs = 0;
static struct sx		 kms_hw_dri_lock;

static void
kms_hw_dri_acquire(void)
{
	sx_xlock(&kms_hw_dri_lock);
	if (kms_hw_dri_oid == NULL) {
		sysctl_ctx_init(&kms_hw_dri_ctx);
		kms_hw_dri_oid = SYSCTL_ADD_NODE(&kms_hw_dri_ctx,
		    SYSCTL_STATIC_CHILDREN(_hw), OID_AUTO, "dri",
		    CTLFLAG_RD, NULL, "DRI devices (libdrm compat)");
	}
	kms_hw_dri_refs++;
	sx_xunlock(&kms_hw_dri_lock);
}

static void
kms_hw_dri_release(void)
{
	sx_xlock(&kms_hw_dri_lock);
	kms_hw_dri_refs--;
	if (kms_hw_dri_refs == 0 && kms_hw_dri_oid != NULL) {
		sysctl_ctx_free(&kms_hw_dri_ctx);
		kms_hw_dri_oid = NULL;
	}
	sx_xunlock(&kms_hw_dri_lock);
}

void
kms_set_busid_pci(struct drm_device *dev, uint32_t domain, uint32_t bus,
    uint32_t slot, uint32_t func)
{
	struct sysctl_oid *card;
	char card_name[8];

	if (dev == NULL || dev->busid_set)
		return;

	snprintf(dev->busid, sizeof(dev->busid),
	    "pci:%04x:%02x:%02x.%u", domain, bus, slot, func);

	kms_hw_dri_acquire();

	sysctl_ctx_init(&dev->busid_sysctl_ctx);
	snprintf(card_name, sizeof(card_name), "%d", dev->minor);
	card = SYSCTL_ADD_NODE(&dev->busid_sysctl_ctx,
	    SYSCTL_CHILDREN(kms_hw_dri_oid), OID_AUTO, card_name,
	    CTLFLAG_RD, NULL, "DRI card");
	SYSCTL_ADD_STRING(&dev->busid_sysctl_ctx, SYSCTL_CHILDREN(card),
	    OID_AUTO, "busid", CTLFLAG_RD, dev->busid, 0,
	    "PCI bus-id for libdrm drmParsePciBusInfo()");
	dev->busid_set = true;
}

static void	kms_device_destroy(struct drm_device *dev);

MALLOC_DEFINE(M_KMS, "kms", "DRM compatibility framework");

static struct sx	kms_registry_lock;
static TAILQ_HEAD(, drm_device) kms_devices =
    TAILQ_HEAD_INITIALIZER(kms_devices);
/*
 * kms_dev_register walks dri/cardN from 0 looking for the first free
 * slot — no monotonic counter, so a kld unload/reload cycle that frees
 * the prior minor will reuse it instead of skipping forward.  The
 * registry lock serialises the search.
 */

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
	for (int try = 0; ; try++) {
		if (try >= 256) {
			sx_xunlock(&kms_registry_lock);
			kms_device_destroy(dev);
			return (ENOSPC);
		}
		dev->minor = try;
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
	/*
	 * Render node.  Linux numbers these as minor 128 + N so libdrm /
	 * Mesa / GBM can find them at /dev/dri/renderD<128+N>.  Same cdevsw,
	 * same si_drv1 (drm_device pointer), so kms_open ties opens of either
	 * node to the same drm_device.  Without this, Mesa falls back to
	 * opening /dev/dri/cardN multiple times for render allocation,
	 * which produces multiple distinct drm_file structs whose event
	 * queues don't share — page-flip events go to one fd, polls happen
	 * on another, and Wayland compositors wedge on the first frame.
	 *
	 * Walk forward exactly the same way as cardN, but starting at 128.
	 * No permission split yet: render node is 0660 / video group like
	 * the card node.  Mesa's render-node-only ioctl restrictions
	 * (no DRM_MASTER, no SET_VERSION) belong in a later commit when
	 * we route ioctls per cdev.
	 */
	for (int try = 128; ; try++) {
		if (try >= 256) {
			TAILQ_INSERT_TAIL(&kms_devices, dev, link);
			sx_xunlock(&kms_registry_lock);
			printf("kms: registered /dev/dri/card%d driver=%s"
			    " (no render node)\n", dev->minor, driver->name);
			*out_dev = dev;
			return (0);
		}
		dev->render_minor = try;
		error = make_dev_s(&args, &dev->render_cdev,
		    "dri/renderD%d", dev->render_minor);
		if (error == 0)
			break;
		if (error != EEXIST) {
			/*
			 * Don't fail the whole registration — the card node
			 * is already alive and KMS is usable, render node is
			 * a Mesa convenience.  Continue without it.
			 */
			dev->render_cdev = NULL;
			dev->render_minor = -1;
			break;
		}
	}

	TAILQ_INSERT_TAIL(&kms_devices, dev, link);
	sx_xunlock(&kms_registry_lock);

	*out_dev = dev;
	if (dev->render_cdev != NULL)
		printf("kms: registered /dev/dri/card%d + renderD%d driver=%s\n",
		    dev->minor, dev->render_minor, driver->name);
	else
		printf("kms: registered /dev/dri/card%d driver=%s"
		    " (render node unavailable)\n", dev->minor, driver->name);
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
	if (dev->render_cdev != NULL) {
		destroy_dev(dev->render_cdev);
		dev->render_cdev = NULL;
	}
	if (dev->busid_set) {
		sysctl_ctx_free(&dev->busid_sysctl_ctx);
		dev->busid_set = false;
		kms_hw_dri_release();
	}
	kms_device_release(dev);
}

static int
kms_modevent(module_t mod __unused, int what, void *arg __unused)
{
	switch (what) {
	case MOD_LOAD:
		sx_init(&kms_registry_lock, "kms_reg");
		sx_init(&kms_hw_dri_lock, "kms_hwdri");
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
		sx_destroy(&kms_hw_dri_lock);
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
