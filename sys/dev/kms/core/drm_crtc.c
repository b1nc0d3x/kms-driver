/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#include <sys/param.h>
#include <sys/systm.h>

#include <kms/drm_crtc.h>
#include <kms/drm_device.h>
#include <kms/drm_framebuffer.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_mode_object.h>
#include <kms/drm_property.h>

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

	TAILQ_INIT(&crtc->pending_vblank_events);

	error = kms_mode_object_register(dev, &crtc->base,
	    DRM_MODE_OBJECT_CRTC);
	if (error != 0)
		return (error);

	/*
	 * Attach the universal CRTC properties so atomic userspace can
	 * discover them via OBJ_GETPROPERTIES and drive them via the
	 * ATOMIC ioctl.  Defaults: ACTIVE=0 (blanked), MODE_ID=0 (no
	 * blob bound).
	 */
	if (dev->mode_config.prop_crtc_active != NULL)
		kms_object_attach_property(&crtc->base,
		    dev->mode_config.prop_crtc_active, 0);
	if (dev->mode_config.prop_crtc_mode_id != NULL)
		kms_object_attach_property(&crtc->base,
		    dev->mode_config.prop_crtc_mode_id, 0);
	return (0);
}

void
kms_crtc_cleanup(struct drm_crtc *crtc)
{
	struct kms_pending_vblank_event *pe;

	if (crtc == NULL || crtc->dev == NULL)
		return;
	/*
	 * Drop the ref that SETCRTC / PAGE_FLIP acquired on primary_fb
	 * so tearing down the CRTC doesn't leak the framebuffer's
	 * backing GEM allocation.
	 */
	if (crtc->primary_fb != NULL) {
		kms_mode_object_put(&crtc->primary_fb->base);
		crtc->primary_fb = NULL;
	}
	while ((pe = TAILQ_FIRST(&crtc->pending_vblank_events)) != NULL) {
		TAILQ_REMOVE(&crtc->pending_vblank_events, pe, link);
		free(pe, M_KMS);
	}
	kms_mode_object_unregister(crtc->dev, &crtc->base);
	if (crtc->funcs != NULL && crtc->funcs->destroy != NULL)
		crtc->funcs->destroy(crtc);
}
