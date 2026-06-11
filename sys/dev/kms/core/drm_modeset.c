/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Legacy modeset ioctls: ADDFB2 / RMFB / SETCRTC / PAGE_FLIP.
 *
 * The framework resolves user-supplied ids to kernel pointers
 * (releasing the lookup refs once the driver hook returns), records
 * the new CRTC state on drm_crtc, and forwards a struct drm_mode_set
 * to driver-provided set_config / page_flip hooks.  Hooks may be NULL
 * — the stub uses that path, exercising the ioctl plumbing without
 * touching real hardware.
 *
 * Atomic equivalents land in Phase 8.  GETFB / GETFB2 land alongside
 * rk_drm in Phase 9 since modesetting DDX never calls them.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/sx.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>
#include <drm/drm_fourcc.h>

#include <kms/drm_connector.h>
#include <kms/drm_crtc.h>
#include <kms/drm_device.h>
#include <kms/drm_file.h>
#include <kms/drm_framebuffer.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_mode_object.h>
#include <kms/drm_modes.h>

#include "kms_internal.h"

#define	DRM_SETCRTC_MAX_CONNECTORS	16

/*
 * Convert a uapi drm_mode_modeinfo into the kernel drm_display_mode
 * SETCRTC will hand to set_config.  Pure data shuffle; flags / type
 * pass through unchanged because the kernel-side constants match the
 * uapi values for the bits Phase 7 cares about.
 */
static void
drm_modeinfo_to_display_mode(const struct drm_mode_modeinfo *info,
    struct drm_display_mode *mode)
{
	size_t n;

	memset(mode, 0, sizeof(*mode));
	mode->clock = info->clock;
	mode->hdisplay = info->hdisplay;
	mode->hsync_start = info->hsync_start;
	mode->hsync_end = info->hsync_end;
	mode->htotal = info->htotal;
	mode->hskew = info->hskew;
	mode->vdisplay = info->vdisplay;
	mode->vsync_start = info->vsync_start;
	mode->vsync_end = info->vsync_end;
	mode->vtotal = info->vtotal;
	mode->vscan = info->vscan;
	mode->vrefresh = info->vrefresh;
	mode->flags = info->flags;
	mode->type = info->type;
	n = strnlen(info->name, sizeof(info->name));
	if (n >= sizeof(mode->name))
		n = sizeof(mode->name) - 1;
	memcpy(mode->name, info->name, n);
	mode->name[n] = '\0';
}

int
kms_ioctl_mode_addfb2(struct drm_file *file, struct drm_mode_fb_cmd2 *cmd)
{
	struct drm_framebuffer *fb;
	int error;

	error = kms_framebuffer_create(file, cmd, &fb);
	if (error != 0)
		return (error);
	cmd->fb_id = fb->base.id;
	return (0);
}

/*
 * Translate the legacy DRM_IOCTL_MODE_ADDFB (single-handle, bpp+depth-coded
 * format) into a drm_mode_fb_cmd2 and forward.  Xorg's modesetting driver
 * still issues the legacy ioctl for scanout buffers on FreeBSD-arm64;
 * without this handler we return ENOTTY and the screen stays black.
 *
 * Format mapping mirrors Linux's drm_mode_legacy_fb_format(): bpp+depth
 * picks a fourcc.  Stick to the four combinations Xorg's
 * modesetting driver actually emits; anything else returns EINVAL.
 */
int
kms_ioctl_mode_addfb(struct drm_file *file, struct drm_mode_fb_cmd *cmd)
{
	struct drm_mode_fb_cmd2 cmd2;
	struct drm_framebuffer *fb;
	uint32_t fourcc;
	int error;

	if (file == NULL || cmd == NULL)
		return (EINVAL);

	switch (cmd->bpp) {
	case 16:
		if (cmd->depth == 16)
			fourcc = DRM_FORMAT_RGB565;
		else
			return (EINVAL);
		break;
	case 24:
		if (cmd->depth == 24)
			fourcc = DRM_FORMAT_RGB888;
		else
			return (EINVAL);
		break;
	case 32:
		if (cmd->depth == 24)
			fourcc = DRM_FORMAT_XRGB8888;
		else if (cmd->depth == 32)
			fourcc = DRM_FORMAT_ARGB8888;
		else
			return (EINVAL);
		break;
	default:
		return (EINVAL);
	}

	memset(&cmd2, 0, sizeof(cmd2));
	cmd2.width = cmd->width;
	cmd2.height = cmd->height;
	cmd2.pixel_format = fourcc;
	cmd2.handles[0] = cmd->handle;
	cmd2.pitches[0] = cmd->pitch;

	error = kms_framebuffer_create(file, &cmd2, &fb);
	if (error != 0)
		return (error);
	cmd->fb_id = fb->base.id;
	return (0);
}

int
kms_ioctl_mode_rmfb(struct drm_file *file, uint32_t *fb_id)
{
	struct drm_mode_object *obj;
	struct drm_framebuffer *fb;

	if (file == NULL || fb_id == NULL)
		return (EINVAL);
	obj = kms_mode_object_find(file->dev, *fb_id, DRM_MODE_OBJECT_FB);
	if (obj == NULL)
		return (ENOENT);
	fb = __containerof(obj, struct drm_framebuffer, base);
	/*
	 * Drop the find ref before cleanup, then cleanup.  cleanup
	 * unregisters and drops the framework's creation ref + the
	 * GEM refs taken at create time.  After unregister no new
	 * lookups can find this fb.
	 */
	kms_mode_object_put(obj);
	kms_framebuffer_cleanup(fb);
	return (0);
}

int
kms_ioctl_mode_setcrtc(struct drm_file *file, struct drm_mode_crtc *r)
{
	struct drm_mode_config *mc = &file->dev->mode_config;
	struct drm_mode_object *crtc_obj = NULL;
	struct drm_mode_object *fb_obj = NULL;
	struct drm_crtc *crtc;
	struct drm_framebuffer *fb = NULL;
	struct drm_connector **conn_arr = NULL;
	uint32_t *conn_ids = NULL;
	struct drm_mode_set set;
	struct drm_display_mode requested_mode;
	uint32_t i, count;
	int error = 0;

	if (file == NULL || r == NULL)
		return (EINVAL);
	if (r->count_connectors > DRM_SETCRTC_MAX_CONNECTORS)
		return (EINVAL);

	crtc_obj = kms_mode_object_find(file->dev, r->crtc_id,
	    DRM_MODE_OBJECT_CRTC);
	if (crtc_obj == NULL)
		return (ENOENT);
	crtc = __containerof(crtc_obj, struct drm_crtc, base);

	if (r->fb_id != 0) {
		fb_obj = kms_mode_object_find(file->dev, r->fb_id,
		    DRM_MODE_OBJECT_FB);
		if (fb_obj == NULL) {
			error = ENOENT;
			goto out;
		}
		fb = __containerof(fb_obj, struct drm_framebuffer, base);
	}

	count = r->count_connectors;
	if (count > 0) {
		conn_ids = malloc((size_t)count * sizeof(uint32_t),
		    M_KMS, M_WAITOK);
		conn_arr = malloc((size_t)count *
		    sizeof(struct drm_connector *), M_KMS,
		    M_WAITOK | M_ZERO);
		error = copyin((const void *)(uintptr_t)r->set_connectors_ptr,
		    conn_ids, count * sizeof(uint32_t));
		if (error != 0)
			goto out;
		for (i = 0; i < count; i++) {
			struct drm_mode_object *cobj;

			cobj = kms_mode_object_find(file->dev,
			    conn_ids[i], DRM_MODE_OBJECT_CONNECTOR);
			if (cobj == NULL) {
				error = ENOENT;
				goto out;
			}
			conn_arr[i] = __containerof(cobj, struct drm_connector,
			    base);
		}
	}

	/*
	 * Update the cached CRTC state under mode_config.mutex so a
	 * concurrent GETCRTC sees one consistent snapshot — either the
	 * old state or the new one, never a half-applied mix.
	 */
	if (r->mode_valid)
		drm_modeinfo_to_display_mode(&r->mode, &requested_mode);

	set.crtc = crtc;
	set.fb = fb;
	set.mode = r->mode_valid ? &requested_mode : NULL;
	set.x = r->x;
	set.y = r->y;
	set.count_connectors = count;
	set.connectors = conn_arr;

	if (crtc->funcs != NULL && crtc->funcs->set_config != NULL) {
		error = crtc->funcs->set_config(&set);
		if (error != 0)
			goto out;
	}

	sx_xlock(&mc->mutex);
	crtc->primary_fb = fb;
	crtc->x = r->x;
	crtc->y = r->y;
	crtc->mode_valid = r->mode_valid;
	if (r->mode_valid) {
		crtc->mode = requested_mode;
		crtc->enabled = true;
	} else {
		memset(&crtc->mode, 0, sizeof(crtc->mode));
		crtc->enabled = false;
	}
	sx_xunlock(&mc->mutex);

out:
	if (conn_arr != NULL) {
		for (i = 0; i < count; i++)
			if (conn_arr[i] != NULL)
				kms_mode_object_put(&conn_arr[i]->base);
		free(conn_arr, M_KMS);
	}
	free(conn_ids, M_KMS);
	if (fb_obj != NULL)
		kms_mode_object_put(fb_obj);
	kms_mode_object_put(crtc_obj);
	return (error);
}

int
kms_ioctl_mode_page_flip(struct drm_file *file,
    struct drm_mode_crtc_page_flip *r)
{
	struct drm_mode_object *crtc_obj, *fb_obj;
	struct drm_crtc *crtc;
	struct drm_framebuffer *fb;
	int error = 0;

	if (file == NULL || r == NULL)
		return (EINVAL);

	crtc_obj = kms_mode_object_find(file->dev, r->crtc_id,
	    DRM_MODE_OBJECT_CRTC);
	if (crtc_obj == NULL)
		return (ENOENT);
	crtc = __containerof(crtc_obj, struct drm_crtc, base);

	fb_obj = kms_mode_object_find(file->dev, r->fb_id,
	    DRM_MODE_OBJECT_FB);
	if (fb_obj == NULL) {
		kms_mode_object_put(crtc_obj);
		return (ENOENT);
	}
	fb = __containerof(fb_obj, struct drm_framebuffer, base);

	if (crtc->funcs != NULL && crtc->funcs->page_flip != NULL)
		error = crtc->funcs->page_flip(crtc, fb, r->flags, r->user_data);

	if (error == 0) {
		sx_xlock(&file->dev->mode_config.mutex);
		crtc->primary_fb = fb;
		/*
		 * If PAGE_FLIP_EVENT was requested, stash the requesting
		 * file + user cookie so the next vblank IRQ emits a
		 * FLIP_COMPLETE event.  Stub drivers without an IRQ chain
		 * simply leak the stash — no harm, the cookie storage is
		 * one pointer.
		 */
		if (r->flags & DRM_MODE_PAGE_FLIP_EVENT) {
			crtc->pending_flip_file = file;
			crtc->pending_flip_user_data = r->user_data;
		}
		sx_xunlock(&file->dev->mode_config.mutex);
	}

	kms_mode_object_put(fb_obj);
	kms_mode_object_put(crtc_obj);
	return (error);
}
