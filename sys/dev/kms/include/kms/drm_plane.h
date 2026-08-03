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
struct drm_plane_state;
struct drm_crtc;
struct drm_framebuffer;

enum drm_plane_type {
	DRM_PLANE_TYPE_OVERLAY	= 0,
	DRM_PLANE_TYPE_PRIMARY	= 1,
	DRM_PLANE_TYPE_CURSOR	= 2,
};

struct drm_plane_funcs {
	void	(*destroy)(struct drm_plane *plane);
	/*
	 * Per-plane validation.  Called by kms_atomic_check_planes for
	 * every plane touched in the batch after property resolution and
	 * before any device-wide atomic_check.  Return 0 to accept the
	 * proposed state, an errno on rejection (typically EINVAL for
	 * unsupported format/scaling, ENOSPC for out-of-bounds).  Optional
	 * — a NULL hook means the framework accepts any proposed state
	 * for this plane (current behaviour, back-compat).
	 */
	int	(*atomic_check)(struct drm_plane *plane,
		    struct drm_plane_state *new_state);
};

struct drm_plane {
	struct drm_mode_object		 base;
	struct drm_device		*dev;
	const struct drm_plane_funcs	*funcs;
	enum drm_plane_type		 type;
	uint32_t			 possible_crtcs;
	uint32_t			 index;	/* assigned at init; indexes
					   the parent drm_device's plane
					   array.  Used by drm_atomic
					   state slot lookup. */
	struct drm_crtc			*crtc;	/* current binding */
	struct drm_framebuffer		*fb;
	/*
	 * Format whitelist exposed via GETPLANE.  Pointer is to
	 * driver-owned storage that must outlive the plane.  Phase 4 just
	 * forwards what the driver passed in; Phase 8 (atomic modeset)
	 * will validate framebuffers against this list on commit.
	 */
	const uint32_t			*format_types;
	uint32_t			 format_count;
};

int	kms_plane_init(struct drm_device *dev, struct drm_plane *plane,
	    const struct drm_plane_funcs *funcs, enum drm_plane_type type,
	    uint32_t possible_crtcs, const uint32_t *format_types,
	    uint32_t format_count);
void	kms_plane_cleanup(struct drm_plane *plane);

#endif /* _KMS_DRM_PLANE_H_ */
