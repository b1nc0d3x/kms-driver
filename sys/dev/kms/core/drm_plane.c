/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#include <sys/param.h>
#include <sys/systm.h>

#include <kms/drm_device.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_mode_object.h>
#include <kms/drm_plane.h>

#include "kms_internal.h"

int
kms_plane_init(struct drm_device *dev, struct drm_plane *plane,
    const struct drm_plane_funcs *funcs, enum drm_plane_type type,
    uint32_t possible_crtcs)
{
	if (dev == NULL || plane == NULL)
		return (EINVAL);

	plane->dev = dev;
	plane->funcs = funcs;
	plane->type = type;
	plane->possible_crtcs = possible_crtcs;
	plane->crtc = NULL;
	plane->fb = NULL;

	return (kms_mode_object_register(dev, &plane->base,
	    DRM_MODE_OBJECT_PLANE));
}

void
kms_plane_cleanup(struct drm_plane *plane)
{
	if (plane == NULL || plane->dev == NULL)
		return;
	kms_mode_object_unregister(plane->dev, &plane->base);
	if (plane->funcs != NULL && plane->funcs->destroy != NULL)
		plane->funcs->destroy(plane);
}
