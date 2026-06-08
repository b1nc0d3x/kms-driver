/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#ifndef _KMS_DRM_PLANE_H_
#define _KMS_DRM_PLANE_H_

#include <sys/types.h>

#include <kms/drm_mode_object.h>

struct drm_device;
struct drm_plane;
struct drm_crtc;
struct drm_framebuffer;

enum drm_plane_type {
	DRM_PLANE_TYPE_OVERLAY	= 0,
	DRM_PLANE_TYPE_PRIMARY	= 1,
	DRM_PLANE_TYPE_CURSOR	= 2,
};

struct drm_plane_funcs {
	void	(*destroy)(struct drm_plane *plane);
};

struct drm_plane {
	struct drm_mode_object		 base;
	struct drm_device		*dev;
	const struct drm_plane_funcs	*funcs;
	enum drm_plane_type		 type;
	uint32_t			 possible_crtcs;
	struct drm_crtc			*crtc;	/* current binding */
	struct drm_framebuffer		*fb;
};

int	kms_plane_init(struct drm_device *dev, struct drm_plane *plane,
	    const struct drm_plane_funcs *funcs, enum drm_plane_type type,
	    uint32_t possible_crtcs);
void	kms_plane_cleanup(struct drm_plane *plane);

#endif /* _KMS_DRM_PLANE_H_ */
