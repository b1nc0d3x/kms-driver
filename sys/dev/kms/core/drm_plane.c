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
#include <kms/drm_property.h>

#include "kms_internal.h"

static void
drm_plane_attach_standard_properties(struct drm_plane *plane)
{
	struct drm_mode_config *mc = &plane->dev->mode_config;
	struct drm_mode_object *o = &plane->base;

	if (mc->prop_plane_type != NULL)
		drm_object_attach_property(o, mc->prop_plane_type,
		    (uint64_t)plane->type);
	if (mc->prop_plane_fb_id != NULL)
		drm_object_attach_property(o, mc->prop_plane_fb_id, 0);
	if (mc->prop_plane_crtc_id != NULL)
		drm_object_attach_property(o, mc->prop_plane_crtc_id, 0);
	if (mc->prop_plane_crtc_x != NULL)
		drm_object_attach_property(o, mc->prop_plane_crtc_x, 0);
	if (mc->prop_plane_crtc_y != NULL)
		drm_object_attach_property(o, mc->prop_plane_crtc_y, 0);
	if (mc->prop_plane_crtc_w != NULL)
		drm_object_attach_property(o, mc->prop_plane_crtc_w, 0);
	if (mc->prop_plane_crtc_h != NULL)
		drm_object_attach_property(o, mc->prop_plane_crtc_h, 0);
	if (mc->prop_plane_src_x != NULL)
		drm_object_attach_property(o, mc->prop_plane_src_x, 0);
	if (mc->prop_plane_src_y != NULL)
		drm_object_attach_property(o, mc->prop_plane_src_y, 0);
	if (mc->prop_plane_src_w != NULL)
		drm_object_attach_property(o, mc->prop_plane_src_w, 0);
	if (mc->prop_plane_src_h != NULL)
		drm_object_attach_property(o, mc->prop_plane_src_h, 0);
}

int
kms_plane_init(struct drm_device *dev, struct drm_plane *plane,
    const struct drm_plane_funcs *funcs, enum drm_plane_type type,
    uint32_t possible_crtcs, const uint32_t *format_types,
    uint32_t format_count)
{
	int error;

	if (dev == NULL || plane == NULL)
		return (EINVAL);
	if (format_count > 0 && format_types == NULL)
		return (EINVAL);

	plane->dev = dev;
	plane->funcs = funcs;
	plane->type = type;
	plane->possible_crtcs = possible_crtcs;
	plane->crtc = NULL;
	plane->fb = NULL;
	plane->format_types = format_types;
	plane->format_count = format_count;

	error = kms_mode_object_register(dev, &plane->base,
	    DRM_MODE_OBJECT_PLANE);
	if (error != 0)
		return (error);
	drm_plane_attach_standard_properties(plane);
	return (0);
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
