/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#ifndef _KMS_DRM_CRTC_H_
#define _KMS_DRM_CRTC_H_

#include <sys/types.h>

#include <kms/drm_mode_object.h>
#include <kms/drm_modes.h>

struct drm_device;
struct drm_crtc;
struct drm_framebuffer;
struct drm_plane;

struct drm_connector;
struct drm_display_mode;

/*
 * Argument bundle passed to set_config.  Mirrors the SETCRTC ioctl
 * payload after the framework has resolved ids to kernel pointers
 * and dropped the lookup refs: drivers see a single coherent view
 * of what userspace asked for.  fb is NULL on a "blank this CRTC"
 * request; connectors is NULL when count_connectors is zero.
 */
struct drm_mode_set {
	struct drm_crtc			*crtc;
	struct drm_framebuffer		*fb;
	const struct drm_display_mode	*mode;	/* NULL on blank */
	uint32_t			 x;
	uint32_t			 y;
	uint32_t			 count_connectors;
	struct drm_connector		**connectors;	/* count_connectors */
};

/*
 * Driver-supplied operations.  Phase 4 only required .destroy; Phase
 * 7 adds .set_config (SETCRTC) and .page_flip (PAGE_FLIP).  All hooks
 * may be NULL on the stub; the framework records the new state on
 * drm_crtc unconditionally so userspace GETCRTC sees the result even
 * without a driver-side implementation.  Phase 8 wires atomic_*.
 */
struct drm_crtc_funcs {
	void	(*destroy)(struct drm_crtc *crtc);
	int	(*set_config)(struct drm_mode_set *set);
	int	(*page_flip)(struct drm_crtc *crtc, struct drm_framebuffer *fb,
		    uint32_t flags, uint64_t user_data);
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
	/*
	 * Cached modeset state.  SETCRTC populates these; GETCRTC reports
	 * them back.  mode_valid is 0 when the CRTC is blanked.  The
	 * primary_fb ref above and any future plane refs are released
	 * when the CRTC is re-armed or destroyed.
	 */
	struct drm_display_mode		 mode;
	uint32_t			 x;
	uint32_t			 y;
	uint8_t				 mode_valid;
};

int	kms_crtc_init(struct drm_device *dev, struct drm_crtc *crtc,
	    const struct drm_crtc_funcs *funcs);
void	kms_crtc_cleanup(struct drm_crtc *crtc);

#endif /* _KMS_DRM_CRTC_H_ */
