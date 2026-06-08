/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/sx.h>

#include <drm/drm.h>
#include <kms/drm_device.h>
#include <kms/drm_file.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_mode_object.h>

#include "kms_internal.h"

/*
 * Upper bound on per-array entries we'll snapshot under the lock and
 * copy out.  Bigger than any realistic CRTC/connector count by orders
 * of magnitude; small enough that a hostile user-supplied count_*
 * field can't drive a multi-megabyte kernel alloc (see
 * feedback_kms_security_rules.md rule 4).
 */
#define	DRM_MODE_GETRES_MAX	4096

void
drm_mode_config_init(struct drm_mode_config *mc)
{
	sx_init(&mc->mutex, "drmmode");
	TAILQ_INIT(&mc->crtcs);
	TAILQ_INIT(&mc->connectors);
	TAILQ_INIT(&mc->encoders);
	TAILQ_INIT(&mc->fbs);
	TAILQ_INIT(&mc->objects);
	mc->next_object_id = 0;
	mc->num_crtc = 0;
	mc->num_connector = 0;
	mc->num_encoder = 0;
	mc->num_fb = 0;
	/*
	 * Defaults.  A real hardware driver overrides these in its
	 * probe path before registering connectors.  Match Linux
	 * drm_mode_config_init defaults so a mechanically-ported
	 * driver doesn't see different limits.
	 */
	mc->min_width = 0;
	mc->max_width = 16384;
	mc->min_height = 0;
	mc->max_height = 16384;
}

void
drm_mode_config_cleanup(struct drm_mode_config *mc)
{
	/*
	 * Objects must already be unregistered by the driver (Phase 4+
	 * will assert this).  For Phase 3 the lists are always empty
	 * but we still tear the lock down.
	 */
	sx_destroy(&mc->mutex);
}

/*
 * Snapshot the IDs from one per-type list into a kernel-side array
 * sized by the user's count.  Returns the malloc'd array (caller
 * frees) and the number of IDs actually written.  Cap is applied
 * before allocation.  Must be called with mc->mutex held shared.
 */
static uint32_t *
drm_mode_object_snapshot_ids(struct drm_mode_object_list *list,
    uint32_t user_count, uint32_t *out_written)
{
	struct drm_mode_object *obj;
	uint32_t *ids;
	uint32_t written;

	*out_written = 0;
	if (user_count == 0)
		return (NULL);
	if (user_count > DRM_MODE_GETRES_MAX)
		user_count = DRM_MODE_GETRES_MAX;
	ids = malloc((size_t)user_count * sizeof(uint32_t), M_KMS,
	    M_WAITOK | M_ZERO);
	written = 0;
	TAILQ_FOREACH(obj, list, link) {
		if (written >= user_count)
			break;
		ids[written++] = obj->id;
	}
	*out_written = written;
	return (ids);
}

int
kms_ioctl_mode_getresources(struct drm_file *file,
    struct drm_mode_card_res *r)
{
	struct drm_mode_config *mc = &file->dev->mode_config;
	uint32_t *fb_ids = NULL;
	uint32_t *crtc_ids = NULL;
	uint32_t *conn_ids = NULL;
	uint32_t *enc_ids = NULL;
	uint32_t fb_n = 0, crtc_n = 0, conn_n = 0, enc_n = 0;
	uint32_t total_fb, total_crtc, total_conn, total_enc;
	uint32_t min_w, max_w, min_h, max_h;
	int error = 0;

	sx_slock(&mc->mutex);
	fb_ids = drm_mode_object_snapshot_ids(&mc->fbs, r->count_fbs, &fb_n);
	crtc_ids = drm_mode_object_snapshot_ids(&mc->crtcs, r->count_crtcs,
	    &crtc_n);
	conn_ids = drm_mode_object_snapshot_ids(&mc->connectors,
	    r->count_connectors, &conn_n);
	enc_ids = drm_mode_object_snapshot_ids(&mc->encoders,
	    r->count_encoders, &enc_n);
	total_fb = mc->num_fb;
	total_crtc = mc->num_crtc;
	total_conn = mc->num_connector;
	total_enc = mc->num_encoder;
	min_w = mc->min_width;
	max_w = mc->max_width;
	min_h = mc->min_height;
	max_h = mc->max_height;
	sx_sunlock(&mc->mutex);

	if (fb_n > 0 && r->fb_id_ptr != 0) {
		error = copyout(fb_ids, (void *)(uintptr_t)r->fb_id_ptr,
		    fb_n * sizeof(uint32_t));
		if (error != 0)
			goto out;
	}
	if (crtc_n > 0 && r->crtc_id_ptr != 0) {
		error = copyout(crtc_ids, (void *)(uintptr_t)r->crtc_id_ptr,
		    crtc_n * sizeof(uint32_t));
		if (error != 0)
			goto out;
	}
	if (conn_n > 0 && r->connector_id_ptr != 0) {
		error = copyout(conn_ids,
		    (void *)(uintptr_t)r->connector_id_ptr,
		    conn_n * sizeof(uint32_t));
		if (error != 0)
			goto out;
	}
	if (enc_n > 0 && r->encoder_id_ptr != 0) {
		error = copyout(enc_ids, (void *)(uintptr_t)r->encoder_id_ptr,
		    enc_n * sizeof(uint32_t));
		if (error != 0)
			goto out;
	}

	/*
	 * Report the totals so userspace can resize and retry; reporting
	 * the truncated written count would loop forever if there are
	 * more objects than the user asked for.
	 */
	r->count_fbs = total_fb;
	r->count_crtcs = total_crtc;
	r->count_connectors = total_conn;
	r->count_encoders = total_enc;
	r->min_width = min_w;
	r->max_width = max_w;
	r->min_height = min_h;
	r->max_height = max_h;

out:
	free(fb_ids, M_KMS);
	free(crtc_ids, M_KMS);
	free(conn_ids, M_KMS);
	free(enc_ids, M_KMS);
	return (error);
}
