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
struct drm_framebuffer;

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
	uint64_t			 modifier;
};

int	kms_framebuffer_init(struct drm_device *dev,
	    struct drm_framebuffer *fb,
	    const struct drm_framebuffer_funcs *funcs);
void	kms_framebuffer_cleanup(struct drm_framebuffer *fb);

#endif /* _KMS_DRM_FRAMEBUFFER_H_ */
