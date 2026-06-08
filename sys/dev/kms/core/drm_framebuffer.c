/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#include <sys/param.h>
#include <sys/systm.h>

#include <kms/drm_device.h>
#include <kms/drm_framebuffer.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_mode_object.h>

#include "kms_internal.h"

int
kms_framebuffer_init(struct drm_device *dev, struct drm_framebuffer *fb,
    const struct drm_framebuffer_funcs *funcs)
{
	if (dev == NULL || fb == NULL)
		return (EINVAL);

	fb->dev = dev;
	fb->funcs = funcs;

	return (kms_mode_object_register(dev, &fb->base, DRM_MODE_OBJECT_FB));
}

void
kms_framebuffer_cleanup(struct drm_framebuffer *fb)
{
	if (fb == NULL || fb->dev == NULL)
		return;
	kms_mode_object_unregister(fb->dev, &fb->base);
	if (fb->funcs != NULL && fb->funcs->destroy != NULL)
		fb->funcs->destroy(fb);
}
