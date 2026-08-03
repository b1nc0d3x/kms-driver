/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#ifndef _KMS_DRM_FRAMEBUFFER_H_
#define _KMS_DRM_FRAMEBUFFER_H_

#include <sys/types.h>

#include <kms/drm_mode_object.h>

struct drm_device;
struct drm_file;
struct drm_framebuffer;
struct drm_gem_object;
struct drm_mode_fb_cmd2;

#define	DRM_FORMAT_MAX_PLANES	4

struct drm_framebuffer_funcs {
	void	(*destroy)(struct drm_framebuffer *fb);
};

struct drm_framebuffer {
	struct drm_mode_object		 base;
	struct drm_device		*dev;
	const struct drm_framebuffer_funcs *funcs;
	uint32_t			 width;
	uint32_t			 height;
	uint32_t			 format;	/* DRM_FORMAT_* fourcc */
	uint32_t			 pitches[DRM_FORMAT_MAX_PLANES];
	uint32_t			 offsets[DRM_FORMAT_MAX_PLANES];
	uint32_t			 handles[DRM_FORMAT_MAX_PLANES];
	/*
	 * Per-plane DRM format modifier (AFBC, tiled, etc.) as passed
	 * in via ADDFB2's DRM_MODE_FB_MODIFIERS flag.  Zero on the
	 * legacy path (linear).  Drivers inspect at scan-out time and
	 * accept/reject via their plane->funcs->atomic_check hook.
	 * Storing per-plane (not scalar) so future NV12+AFBC works.
	 */
	uint64_t			 modifiers[DRM_FORMAT_MAX_PLANES];
	/*
	 * GEM object references held while the framebuffer is alive.
	 * Phase 6 buffers are single-plane, but the slots are sized so a
	 * future NV12-class FB can use 2 entries without changing the
	 * struct layout.  Each ref is paired with an entry in handles[]
	 * and dropped in kms_framebuffer_cleanup.
	 */
	struct drm_gem_object		*gem_objs[DRM_FORMAT_MAX_PLANES];
};

int	kms_framebuffer_init(struct drm_device *dev,
	    struct drm_framebuffer *fb,
	    const struct drm_framebuffer_funcs *funcs);
void	kms_framebuffer_cleanup(struct drm_framebuffer *fb);

/*
 * Helper used by the ADDFB2 ioctl path.  Resolves each non-zero handle
 * in fb_cmd against the caller's GEM handle table, pins the GEM
 * object, allocates a drm_framebuffer, copies the geometry / format /
 * pitches / offsets, and registers it as a DRM_MODE_OBJECT_FB.  On
 * success *out_fb receives the new framebuffer with an initial
 * refcount of 1; on error any GEM refs taken are released and the
 * partial framebuffer is destroyed.
 */
int	kms_framebuffer_create(struct drm_file *file,
	    const struct drm_mode_fb_cmd2 *fb_cmd,
	    struct drm_framebuffer **out_fb);

#endif /* _KMS_DRM_FRAMEBUFFER_H_ */
