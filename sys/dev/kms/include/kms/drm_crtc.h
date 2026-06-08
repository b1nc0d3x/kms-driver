/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#ifndef _KMS_DRM_CRTC_H_
#define _KMS_DRM_CRTC_H_

#include <sys/types.h>

#include <kms/drm_mode_object.h>

struct drm_device;
struct drm_crtc;
struct drm_framebuffer;
struct drm_plane;

/*
 * Driver-supplied operations.  destroy is the only hook required in
 * Phase 4 — it lets the driver free the wrapper struct that embeds
 * drm_crtc as its first member.  May be NULL if the driver manages
 * storage out of band (the stub does this).  More ops join here in
 * Phase 7 (set_config, page_flip) and Phase 8 (atomic_*).
 */
struct drm_crtc_funcs {
	void	(*destroy)(struct drm_crtc *crtc);
};

struct drm_crtc {
	struct drm_mode_object		 base;	/* must be first */
	struct drm_device		*dev;
	const struct drm_crtc_funcs	*funcs;
	uint32_t			 index;	/* assigned at init; bit
						 * position in
						 * possible_crtcs masks */
	bool				 enabled;
	struct drm_framebuffer		*primary_fb;
	struct drm_plane		*primary_plane;
};

int	kms_crtc_init(struct drm_device *dev, struct drm_crtc *crtc,
	    const struct drm_crtc_funcs *funcs);
void	kms_crtc_cleanup(struct drm_crtc *crtc);

#endif /* _KMS_DRM_CRTC_H_ */
