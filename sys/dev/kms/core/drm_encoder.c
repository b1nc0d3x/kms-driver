/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#include <sys/param.h>
#include <sys/systm.h>

#include <kms/drm_device.h>
#include <kms/drm_encoder.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_mode_object.h>

#include "kms_internal.h"

int
kms_encoder_init(struct drm_device *dev, struct drm_encoder *encoder,
    const struct drm_encoder_funcs *funcs, uint32_t encoder_type)
{
	if (dev == NULL || encoder == NULL)
		return (EINVAL);

	encoder->dev = dev;
	encoder->funcs = funcs;
	encoder->encoder_type = encoder_type;
	encoder->possible_crtcs = 0;
	encoder->possible_clones = 0;
	encoder->crtc = NULL;

	return (kms_mode_object_register(dev, &encoder->base,
	    DRM_MODE_OBJECT_ENCODER));
}

void
kms_encoder_cleanup(struct drm_encoder *encoder)
{
	if (encoder == NULL || encoder->dev == NULL)
		return;
	kms_mode_object_unregister(encoder->dev, &encoder->base);
	if (encoder->funcs != NULL && encoder->funcs->destroy != NULL)
		encoder->funcs->destroy(encoder);
}
