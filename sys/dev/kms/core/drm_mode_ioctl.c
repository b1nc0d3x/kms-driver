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
#include <kms/drm_modes.h>
#include <kms/drm_plane.h>

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
	struct drm_mode_config *mc = &file->dev->mode_config;
	struct drm_mode_object *obj;
	struct drm_connector *connector;
	struct drm_display_mode *m;
	struct drm_mode_modeinfo *modeinfos = NULL;
	uint32_t *enc_ids = NULL;
	uint32_t enc_n = 0;
	uint32_t mode_n = 0;
	uint32_t i;
	int error = 0;
	/*
	 * Snapshot the connector's local state under its dev_lock-free
	 * find ref, then drop the ref before copyout so we don't hold
	 * a refcount across a sleepable copy (security rule 1).
	 */
	uint32_t out_encoder_id, out_connector_type;
	uint32_t out_connector_type_id, out_status, out_mm_w, out_mm_h;
	uint32_t out_subpixel;

	obj = kms_mode_object_find(file->dev, r->connector_id,
	    DRM_MODE_OBJECT_CONNECTOR);
	if (obj == NULL)
		return (ENOENT);
	connector = __containerof(obj, struct drm_connector, base);

	/*
	 * Lazy mode probe: first GETCONNECTOR triggers get_modes, which
	 * populates the connector's mode list via add_mode.  Subsequent
	 * calls reuse the cached list.  Phase 7+ will add an explicit
	 * rescan path for hotplug.  Race window: two concurrent first
	 * calls could both invoke get_modes; drivers are expected to
	 * make get_modes idempotent (clear-then-add or check mode_count).
	 */
	if (connector->mode_count == 0 && connector->funcs != NULL &&
	    connector->funcs->get_modes != NULL)
		(void)connector->funcs->get_modes(connector);

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

	/*
	 * Snapshot the mode list under the config lock.  Cap at
	 * DRM_MODE_GETRES_MAX (rule 4) before sizing the alloc.  We
	 * copy into kernel-owned modeinfo storage so the sleepable
	 * copyout below runs outside the lock.
	 */
	sx_slock(&mc->mutex);
	mode_n = connector->mode_count;
	if (mode_n > DRM_MODE_GETRES_MAX)
		mode_n = DRM_MODE_GETRES_MAX;
	if (mode_n > 0) {
		modeinfos = malloc((size_t)mode_n * sizeof(*modeinfos),
		    M_KMS, M_WAITOK | M_ZERO);
		i = 0;
		TAILQ_FOREACH(m, &connector->modes, link) {
			if (i >= mode_n)
				break;
			drm_display_mode_to_modeinfo(m, &modeinfos[i++]);
		}
		mode_n = i;
	}
	sx_sunlock(&mc->mutex);

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
	if (r->count_modes > 0 && r->modes_ptr != 0 && mode_n > 0) {
		uint32_t to_copy;

		to_copy = (r->count_modes < mode_n) ?
		    r->count_modes : mode_n;
		error = copyout(modeinfos, (void *)(uintptr_t)r->modes_ptr,
		    to_copy * sizeof(struct drm_mode_modeinfo));
		if (error != 0)
			goto out;
	}

	/*
	 * Report totals so userspace can resize and retry; userspace
	 * keys off these to allocate the right array sizes.  count_props
	 * stays 0 until properties land (Phase 7+).
	 */
	r->count_encoders = enc_n;
	r->count_modes = mode_n;
	r->count_props = 0;
	r->encoder_id = out_encoder_id;
	r->connector_type = out_connector_type;
	r->connector_type_id = out_connector_type_id;
	r->connection = out_status;
	r->mm_width = out_mm_w;
	r->mm_height = out_mm_h;
	r->subpixel = out_subpixel;
	r->pad = 0;

out:
	free(enc_ids, M_KMS);
	free(modeinfos, M_KMS);
	return (error);
}

int
kms_ioctl_mode_getplane_resources(struct drm_file *file,
    struct drm_mode_get_plane_res *r)
{
	struct drm_mode_config *mc = &file->dev->mode_config;
	struct drm_mode_object *obj;
	uint32_t *ids = NULL;
	uint32_t total, written;
	uint32_t user_count;
	int error = 0;

	/*
	 * Snapshot under shared lock, copyout afterward (rule 1:
	 * sleepable copyout never runs while holding the config lock).
	 */
	user_count = r->count_planes;
	written = 0;

	sx_slock(&mc->mutex);
	total = mc->num_plane;
	if (user_count > 0) {
		uint32_t cap;

		cap = user_count;
		if (cap > DRM_MODE_GETRES_MAX)
			cap = DRM_MODE_GETRES_MAX;
		ids = malloc((size_t)cap * sizeof(uint32_t), M_KMS,
		    M_WAITOK | M_ZERO);
		TAILQ_FOREACH(obj, &mc->planes, link) {
			if (written >= cap)
				break;
			ids[written++] = obj->id;
		}
	}
	sx_sunlock(&mc->mutex);

	if (written > 0 && r->plane_id_ptr != 0) {
		error = copyout(ids, (void *)(uintptr_t)r->plane_id_ptr,
		    written * sizeof(uint32_t));
	}
	r->count_planes = total;
	free(ids, M_KMS);
	return (error);
}

#define	DRM_MODE_PLANE_FORMATS_MAX	4096

int
kms_ioctl_mode_getplane(struct drm_file *file, struct drm_mode_get_plane *r)
{
	struct drm_mode_object *obj;
	struct drm_plane *plane;
	uint32_t out_crtc_id, out_fb_id, out_possible_crtcs;
	uint32_t out_format_count;
	uint32_t *formats = NULL;
	uint32_t to_copy = 0;
	int error = 0;

	obj = kms_mode_object_find(file->dev, r->plane_id,
	    DRM_MODE_OBJECT_PLANE);
	if (obj == NULL)
		return (ENOENT);
	plane = __containerof(obj, struct drm_plane, base);

	out_crtc_id = (plane->crtc != NULL) ? plane->crtc->base.id : 0;
	out_fb_id = (plane->fb != NULL) ? plane->fb->base.id : 0;
	out_possible_crtcs = plane->possible_crtcs;
	out_format_count = plane->format_count;
	if (out_format_count > DRM_MODE_PLANE_FORMATS_MAX)
		out_format_count = DRM_MODE_PLANE_FORMATS_MAX;

	/*
	 * Snapshot the format list to a kernel buffer while holding the
	 * object ref, then drop the ref before copyout to keep the
	 * sleepable copy outside any kept lock (rule 1).
	 */
	if (out_format_count > 0 && r->count_format_types > 0 &&
	    r->format_type_ptr != 0) {
		uint32_t cap;
		uint32_t i;

		cap = r->count_format_types;
		if (cap > out_format_count)
			cap = out_format_count;
		formats = malloc((size_t)cap * sizeof(uint32_t), M_KMS,
		    M_WAITOK);
		for (i = 0; i < cap; i++)
			formats[i] = plane->format_types[i];
		to_copy = cap;
	}

	kms_mode_object_put(obj);

	if (to_copy > 0) {
		error = copyout(formats,
		    (void *)(uintptr_t)r->format_type_ptr,
		    to_copy * sizeof(uint32_t));
		if (error != 0)
			goto out;
	}
	r->crtc_id = out_crtc_id;
	r->fb_id = out_fb_id;
	r->possible_crtcs = out_possible_crtcs;
	r->gamma_size = 0;
	r->count_format_types = out_format_count;

out:
	free(formats, M_KMS);
	return (error);
}
