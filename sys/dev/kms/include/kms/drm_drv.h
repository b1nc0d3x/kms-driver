/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#ifndef _KMS_DRM_DRV_H_
#define _KMS_DRM_DRV_H_

#include <sys/types.h>

struct drm_device;

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
};

/*
 * Register a card.  Allocates struct drm_device, makes /dev/dri/cardN
 * with N = next free minor.  *out_dev gets the allocated device on
 * success.  Returns 0 or errno.
 *
 * driver_priv is opaque, retrievable from drm_device->driver_priv by
 * the registering driver.
 */
int	kms_dev_register(const struct drm_driver *driver,
	    void *driver_priv, struct drm_device **out_dev);

/*
 * Unregister.  Removes the cdev so no new open()s succeed; existing
 * opens are allowed to drain.  Caller must ensure no outstanding
 * references after this returns.
 */
void	kms_dev_unregister(struct drm_device *dev);

#endif /* _KMS_DRM_DRV_H_ */
