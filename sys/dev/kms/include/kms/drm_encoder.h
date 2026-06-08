/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#ifndef _KMS_DRM_ENCODER_H_
#define _KMS_DRM_ENCODER_H_

#include <sys/types.h>

#include <kms/drm_mode_object.h>

struct drm_device;
struct drm_encoder;
struct drm_crtc;

struct drm_encoder_funcs {
	void	(*destroy)(struct drm_encoder *encoder);
};

struct drm_encoder {
	struct drm_mode_object		 base;
	struct drm_device		*dev;
	const struct drm_encoder_funcs	*funcs;
	uint32_t			 encoder_type;	/* DRM_MODE_ENCODER_* */
	uint32_t			 possible_crtcs;
	uint32_t			 possible_clones;
	struct drm_crtc			*crtc;		/* current binding */
};

int	kms_encoder_init(struct drm_device *dev, struct drm_encoder *encoder,
	    const struct drm_encoder_funcs *funcs, uint32_t encoder_type);
void	kms_encoder_cleanup(struct drm_encoder *encoder);

#endif /* _KMS_DRM_ENCODER_H_ */
