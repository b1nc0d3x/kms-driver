/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Per-object query ioctls: GETCRTC / GETENCODER / GETCONNECTOR.
 * GETRESOURCES lives in drm_mode_config.c because it touches the
 * config root directly; the per-object getters are split out so
 * they stay close to their object-type code.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/sx.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>
#include <kms/drm_connector.h>
#include <kms/drm_crtc.h>
#include <kms/drm_device.h>
#include <kms/drm_encoder.h>
#include <kms/drm_file.h>
#include <kms/drm_framebuffer.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_mode_object.h>

#include "kms_internal.h"

int
kms_ioctl_mode_getcrtc(struct drm_file *file, struct drm_mode_crtc *r)
{
	struct drm_mode_object *obj;
	struct drm_crtc *crtc;
	struct drm_framebuffer *fb;

	obj = kms_mode_object_find(file->dev, r->crtc_id, DRM_MODE_OBJECT_CRTC);
	if (obj == NULL)
		return (ENOENT);
	crtc = __containerof(obj, struct drm_crtc, base);

	fb = crtc->primary_fb;
	r->fb_id = (fb != NULL) ? fb->base.id : 0;
	r->x = 0;
	r->y = 0;
	r->gamma_size = 0;
	r->mode_valid = 0;
	memset(&r->mode, 0, sizeof(r->mode));
	/*
	 * set_connectors_ptr / count_connectors are input-only fields
	 * for SETCRTC; GETCRTC leaves them untouched per the Linux
	 * uapi contract.
	 */

	kms_mode_object_put(obj);
	return (0);
}

int
kms_ioctl_mode_getencoder(struct drm_file *file,
    struct drm_mode_get_encoder *r)
{
	struct drm_mode_object *obj;
	struct drm_encoder *encoder;

	obj = kms_mode_object_find(file->dev, r->encoder_id,
	    DRM_MODE_OBJECT_ENCODER);
	if (obj == NULL)
		return (ENOENT);
	encoder = __containerof(obj, struct drm_encoder, base);

	r->encoder_type = encoder->encoder_type;
	r->crtc_id = (encoder->crtc != NULL) ? encoder->crtc->base.id : 0;
	r->possible_crtcs = encoder->possible_crtcs;
	r->possible_clones = encoder->possible_clones;

	kms_mode_object_put(obj);
	return (0);
}

int
kms_ioctl_mode_getconnector(struct drm_file *file,
    struct drm_mode_get_connector *r)
{
	struct drm_mode_object *obj;
	struct drm_connector *connector;
	uint32_t *enc_ids = NULL;
	uint32_t enc_n = 0;
	uint32_t i;
	int error = 0;
	/*
	 * Snapshot the connector's local state under its dev_lock-free
	 * find ref, then drop the ref before copyout so we don't hold
	 * a refcount across a sleepable copy.  Phase 5 will add modes
	 * + props here; Phase 4 only reports the encoder list.
	 */
	uint32_t out_encoder_type, out_encoder_id, out_connector_type;
	uint32_t out_connector_type_id, out_status, out_mm_w, out_mm_h;
	uint32_t out_subpixel;

	obj = kms_mode_object_find(file->dev, r->connector_id,
	    DRM_MODE_OBJECT_CONNECTOR);
	if (obj == NULL)
		return (ENOENT);
	connector = __containerof(obj, struct drm_connector, base);

	out_encoder_type = 0;
	out_encoder_id = (connector->encoder != NULL) ?
	    connector->encoder->base.id : 0;
	out_connector_type = connector->connector_type;
	out_connector_type_id = connector->connector_type_id;
	out_status = (uint32_t)connector->status;
	out_mm_w = connector->mm_width;
	out_mm_h = connector->mm_height;
	out_subpixel = connector->subpixel_order;

	enc_n = connector->encoder_count;
	if (enc_n > 0) {
		enc_ids = malloc((size_t)enc_n * sizeof(uint32_t), M_KMS,
		    M_WAITOK);
		for (i = 0; i < enc_n; i++)
			enc_ids[i] = connector->encoder_ids[i];
	}

	kms_mode_object_put(obj);

	if (r->count_encoders > 0 && r->encoders_ptr != 0 && enc_n > 0) {
		uint32_t to_copy;

		to_copy = (r->count_encoders < enc_n) ?
		    r->count_encoders : enc_n;
		error = copyout(enc_ids, (void *)(uintptr_t)r->encoders_ptr,
		    to_copy * sizeof(uint32_t));
		if (error != 0)
			goto out;
	}

	/*
	 * count_modes and count_props stay 0 until Phase 5 (EDID +
	 * properties).  Report total encoders so userspace can size
	 * the array correctly on retry.
	 */
	r->count_encoders = enc_n;
	r->count_modes = 0;
	r->count_props = 0;
	r->encoder_id = out_encoder_id;
	r->connector_type = out_connector_type;
	r->connector_type_id = out_connector_type_id;
	r->connection = out_status;
	r->mm_width = out_mm_w;
	r->mm_height = out_mm_h;
	r->subpixel = out_subpixel;
	r->pad = 0;
	(void)out_encoder_type;

out:
	free(enc_ids, M_KMS);
	return (error);
}
