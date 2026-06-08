/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#include <sys/param.h>
#include <sys/systm.h>

#include <kms/drm_crtc.h>
#include <kms/drm_device.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_mode_object.h>

#include "kms_internal.h"

int
kms_crtc_init(struct drm_device *dev, struct drm_crtc *crtc,
    const struct drm_crtc_funcs *funcs)
{
	int error;

	if (dev == NULL || crtc == NULL)
		return (EINVAL);

	crtc->dev = dev;
	crtc->funcs = funcs;
	crtc->enabled = false;
	crtc->primary_fb = NULL;
	crtc->primary_plane = NULL;
	/*
	 * Index is the CRTC's bit position in possible_crtcs masks on
	 * encoders/planes.  Assigned in registration order from the
	 * pre-increment count snapshot — the lock inside register
	 * commits the count, so reading num_crtc here is stable.
	 */
	crtc->index = dev->mode_config.num_crtc;

	error = kms_mode_object_register(dev, &crtc->base,
	    DRM_MODE_OBJECT_CRTC);
	if (error != 0)
		return (error);
	return (0);
}

void
kms_crtc_cleanup(struct drm_crtc *crtc)
{
	if (crtc == NULL || crtc->dev == NULL)
		return;
	kms_mode_object_unregister(crtc->dev, &crtc->base);
	if (crtc->funcs != NULL && crtc->funcs->destroy != NULL)
		crtc->funcs->destroy(crtc);
}
