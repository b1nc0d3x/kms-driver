/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#ifndef _KMS_DRM_DRV_H_
#define _KMS_DRM_DRV_H_

#include <sys/types.h>

#include <kms/drm_device.h>

struct drm_file;

/*
 * Driver descriptor.  Filled in by the hardware driver and passed to
 * kms_dev_register().  Mirrors the relevant fields of Linux's
 * struct drm_driver so porting rk_drm stays mechanical.
 */
struct drm_driver {
	const char	*name;
	const char	*desc;
	const char	*date;
	uint32_t	 major;
	uint32_t	 minor;
	uint32_t	 patchlevel;
	uint32_t	 driver_features;
	/*
	 * Per-driver ioctl fallback.  Called by kms_ioctl after the core
	 * dispatch matches nothing — typically for driver-specific cmd
	 * codes in the DRM_COMMAND_BASE+ range (e.g. I915_* / RADEON_*
	 * ioctls iris and amdgpu issue at userspace init).  Return 0 on
	 * success or a positive errno; the framework returns ENOTTY
	 * unchanged if this hook is NULL.
	 */
	int		(*ioctl)(struct drm_file *file, u_long cmd, void *data);
};

/*
 * Register a card.  Allocates struct drm_device, makes /dev/dri/cardN
 * with N = next free minor.  *out_dev gets the allocated device on
 * success.  Returns 0 or errno.
 *
 * driver_priv is opaque, retrievable from drm_device->driver_priv by
 * the registering driver.
 *
 * DO NOT call kms_dev_register_versioned directly — use kms_dev_register
 * (the macro).  Passing the caller's sizeof(struct drm_device) lets kms
 * catch an ABI-mismatched consumer (rk_kms.ko compiled against an older
 * layout than the loaded kms.ko) and refuse the registration cleanly
 * instead of letting the driver deref shifted fields and panic — the
 * exact failure mode of the 2026-08-03 rk_kms_set_config crash after a
 * next_magic field was removed without a matched-set rebuild.
 */
int	kms_dev_register_versioned(const struct drm_driver *driver,
	    void *driver_priv, size_t caller_devsize,
	    struct drm_device **out_dev);

#define	kms_dev_register(drv, priv, out) \
	kms_dev_register_versioned((drv), (priv), \
	    sizeof(struct drm_device), (out))

/*
 * Unregister.  Removes the cdev so no new open()s succeed; existing
 * opens are allowed to drain.  Caller must ensure no outstanding
 * references after this returns.
 */
void	kms_dev_unregister(struct drm_device *dev);

/*
 * Register a PCI bus-id under hw.dri.<minor>.busid as a string
 * "pci:DDDD:BB:SS.F".  libdrm's drmGetDeviceFromDevId path on
 * FreeBSD reads this sysctl to map a dri/cardN device to a PCI
 * vendor/device pair via drmParsePciDeviceInfo; without it,
 * Wayland compositors (kwin, weston, plasma) cannot enumerate
 * the GPU and wedge on their first frame.
 *
 * Call after kms_dev_register from a PCI-attached driver, before
 * making the cdev visible to userspace.  String storage is owned
 * by the framework and freed on kms_dev_unregister.
 */
void	kms_set_busid_pci(struct drm_device *dev, uint32_t domain,
	    uint32_t bus, uint32_t slot, uint32_t func);

#endif /* _KMS_DRM_DRV_H_ */
