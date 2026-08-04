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
void
kms_modeinfo_to_display_mode(const struct drm_mode_modeinfo *info,
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

/*
 * CLOSEFB is the "modern" version of RMFB — same lifecycle-drop, but
 * per uapi it must not disable planes that still reference the fb.  We
 * already have that semantic: cleanup unregisters + drops the creation
 * ref; live plane bindings keep the fb alive via their own refs until
 * they're replaced or torn down.  So the implementation is literally
 * rmfb with a padding check.
 */
int
kms_ioctl_mode_closefb(struct drm_file *file, struct drm_mode_closefb *arg)
{
	if (file == NULL || arg == NULL)
		return (EINVAL);
	if (arg->pad != 0)
		return (EINVAL);
	return (kms_ioctl_mode_rmfb(file, &arg->fb_id));
}

/*
 * FourCC → (bpp, depth) for the packed single-plane formats GETFB
 * legacy expects.  Returns 0 on success, EINVAL for anything the
 * legacy struct can't describe (planar / paletted / >32 bpp).  depth
 * is the X11-style visible-bit count (24 for XRGB8888, 30 for
 * XRGB2101010, 16 for RGB565, 15 for XRGB1555).
 */
static int
kms_fb_format_bpp_depth(uint32_t fourcc, uint32_t *bpp, uint32_t *depth)
{
	switch (fourcc) {
	case DRM_FORMAT_RGB565:
	case DRM_FORMAT_BGR565:
		*bpp = 16; *depth = 16; return (0);
	case DRM_FORMAT_XRGB1555:
	case DRM_FORMAT_XBGR1555:
		*bpp = 16; *depth = 15; return (0);
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_ABGR1555:
		*bpp = 16; *depth = 16; return (0);
	case DRM_FORMAT_RGB888:
	case DRM_FORMAT_BGR888:
		*bpp = 24; *depth = 24; return (0);
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_BGRX8888:
		*bpp = 32; *depth = 24; return (0);
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_BGRA8888:
		*bpp = 32; *depth = 32; return (0);
	case DRM_FORMAT_XRGB2101010:
	case DRM_FORMAT_XBGR2101010:
		*bpp = 32; *depth = 30; return (0);
	case DRM_FORMAT_ARGB2101010:
	case DRM_FORMAT_ABGR2101010:
		*bpp = 32; *depth = 30; return (0);
	}
	return (EINVAL);
}

/*
 * GETFB: fill drm_mode_fb_cmd from a live fb.  Handle exposure follows
 * Linux — only masters see it, since a raw GEM handle number lets a
 * non-master compositor peek at another fd's scanout buffer.  Planar
 * formats can't be described by this legacy struct, so we reject them
 * with EINVAL and expect the caller to fall back to GETFB2.
 */
int
kms_ioctl_mode_getfb(struct drm_file *file, struct drm_mode_fb_cmd *r)
{
	struct drm_mode_object *obj;
	struct drm_framebuffer *fb;
	uint32_t bpp, depth;
	int error;

	if (file == NULL || r == NULL)
		return (EINVAL);
	obj = kms_mode_object_find(file->dev, r->fb_id, DRM_MODE_OBJECT_FB);
	if (obj == NULL)
		return (ENOENT);
	fb = __containerof(obj, struct drm_framebuffer, base);
	error = kms_fb_format_bpp_depth(fb->format, &bpp, &depth);
	if (error != 0) {
		kms_mode_object_put(obj);
		return (error);
	}
	r->width  = fb->width;
	r->height = fb->height;
	r->pitch  = fb->pitches[0];
	r->bpp    = bpp;
	r->depth  = depth;
	r->handle = file->is_master ? fb->handles[0] : 0;
	kms_mode_object_put(obj);
	return (0);
}

/*
 * GETFB2: fill drm_mode_fb_cmd2 from a live fb, including per-plane
 * pitches/offsets/handles/modifier and the source pixel_format.  Same
 * master-only rule for handles as GETFB.  Unlike GETFB this accepts
 * any format we recognized on ADDFB2, since the struct can carry
 * planar layouts.
 */
int
kms_ioctl_mode_getfb2(struct drm_file *file, struct drm_mode_fb_cmd2 *r)
{
	struct drm_mode_object *obj;
	struct drm_framebuffer *fb;
	int i;

	if (file == NULL || r == NULL)
		return (EINVAL);
	obj = kms_mode_object_find(file->dev, r->fb_id, DRM_MODE_OBJECT_FB);
	if (obj == NULL)
		return (ENOENT);
	fb = __containerof(obj, struct drm_framebuffer, base);
	r->width        = fb->width;
	r->height       = fb->height;
	r->pixel_format = fb->format;
	r->flags        = 0;
	for (i = 0; i < DRM_FORMAT_MAX_PLANES; i++) {
		r->pitches[i]  = fb->pitches[i];
		r->offsets[i]  = fb->offsets[i];
		r->handles[i]  = file->is_master ? fb->handles[i] : 0;
		r->modifier[i] = fb->modifiers[i];
		if (fb->modifiers[i] != 0)
			r->flags |= DRM_MODE_FB_MODIFIERS;
	}
	kms_mode_object_put(obj);
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
		kms_modeinfo_to_display_mode(&r->mode, &requested_mode);

	/*
	 * H6 fb-vs-mode geometry check: with a real fb + a real mode,
	 * (x + hdisplay) and (y + vdisplay) must fit inside the fb's
	 * dimensions.  Otherwise VOP DMA would scan out past the end
	 * of the BO — an OOB DMA-read of adjacent kernel/user memory.
	 * Cast to uint64_t to defeat uint32 wrap on huge x/y values.
	 */
	if (r->mode_valid && fb != NULL) {
		if ((uint64_t)r->x + requested_mode.hdisplay > fb->width ||
		    (uint64_t)r->y + requested_mode.vdisplay > fb->height) {
			error = EINVAL;
			goto out;
		}
	}

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

	{
		struct drm_framebuffer *old_fb;

		/*
		 * Swap primary_fb under the mode-config lock so the crtc's
		 * fb pointer is never observed dangling.  Take a fresh ref
		 * on the new fb before assignment; drop the old one after
		 * releasing the lock (put may reach the free callback).
		 */
		sx_xlock(&mc->mutex);
		old_fb = crtc->primary_fb;
		if (fb != NULL)
			kms_mode_object_get(&fb->base);
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
		if (old_fb != NULL)
			kms_mode_object_put(&old_fb->base);
	}

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

	/*
	 * Reject unknown flag bits per Linux drm_mode_page_flip_ioctl —
	 * silently ignoring bits userspace didn't understand hides driver
	 * / uapi mismatches until an unrelated flip regresses in a way
	 * that's hard to trace.  ASYNC is rejected here because our
	 * commit path always waits for the next vblank to fire
	 * FLIP_COMPLETE; asserting it would lie about semantics.
	 */
	if ((r->flags & ~(uint32_t)DRM_MODE_PAGE_FLIP_EVENT) != 0)
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

	/*
	 * H6 PAGE_FLIP geometry check: the new fb must match the
	 * currently-scanned fb in width, height and format.  Linux
	 * enforces this in drm_mode_page_flip_ioctl — a mismatch
	 * would cause VOP to scan OOB or corrupted pixels for at
	 * least one frame.  Also refuses flip against a CRTC with
	 * no current scanout fb (nothing to flip away from).
	 */
	{
		struct drm_mode_config *mc = &file->dev->mode_config;
		struct drm_framebuffer *cur;
		int mismatch = 0;

		sx_slock(&mc->mutex);
		cur = crtc->primary_fb;
		if (cur == NULL)
			mismatch = 1;
		else if (fb->width != cur->width ||
		    fb->height != cur->height ||
		    fb->format != cur->format)
			mismatch = 2;
		sx_sunlock(&mc->mutex);
		if (mismatch != 0) {
			kms_mode_object_put(fb_obj);
			kms_mode_object_put(crtc_obj);
			return (EINVAL);
		}
	}

	{
		struct drm_framebuffer *old_fb;
		bool have_event = (r->flags & DRM_MODE_PAGE_FLIP_EVENT) != 0;

		/*
		 * M3 (2026-08-03 review): arm pending_flip_file BEFORE calling
		 * funcs->page_flip — a driver whose vblank IRQ fires between
		 * hardware programming and our post-call stash would find
		 * pending_flip_file NULL and drop the FLIP_COMPLETE event on
		 * the floor.  If page_flip returns error we roll back the arm
		 * under the same mutex.  Matches Linux drm_atomic_helper_
		 * page_flip's arm-then-commit shape.
		 */
		sx_xlock(&file->dev->mode_config.mutex);
		old_fb = crtc->primary_fb;
		/*
		 * Take a persistent ref before swapping primary_fb — the
		 * next RMFB on the outgoing fb must not free storage that
		 * the CRTC still points at.  Old fb's ref is dropped either
		 * inline (event-less flip: nothing scans it after this call
		 * returns from the driver, keeping current behaviour) or
		 * deferred to the vblank handler (event flip: H3, VOP is
		 * still scanning old_fb until latch).
		 */
		kms_mode_object_get(&fb->base);
		crtc->primary_fb = fb;
		if (have_event) {
			crtc->pending_flip_file = file;
			crtc->pending_flip_user_data = r->user_data;
			crtc->pending_flip_old_fb = old_fb;
			kms_file_get(file);
		}
		sx_xunlock(&file->dev->mode_config.mutex);

		if (crtc->funcs != NULL && crtc->funcs->page_flip != NULL)
			error = crtc->funcs->page_flip(crtc, fb, r->flags,
			    r->user_data);

		if (error != 0) {
			/*
			 * Roll back the arm.  Under mode_config.mutex so a vblank
			 * handler racing us either sees the armed state (and fires
			 * FLIP_COMPLETE, spurious but harmless — HW didn't actually
			 * flip since page_flip failed) or the cleared state.  Rare
			 * path; simpler to leave the mode_config commit in place
			 * than to also revert primary_fb.
			 */
			sx_xlock(&file->dev->mode_config.mutex);
			if (have_event && crtc->pending_flip_file == file) {
				crtc->pending_flip_file = NULL;
				crtc->pending_flip_old_fb = NULL;
				kms_file_put(file);
				have_event = false;
			}
			/* undo primary_fb swap */
			kms_mode_object_put(&fb->base);
			crtc->primary_fb = old_fb;
			sx_xunlock(&file->dev->mode_config.mutex);
			old_fb = NULL;	/* still installed */
		} else if (have_event) {
			old_fb = NULL;	/* ownership handed to vblank handler */
		}
		if (old_fb != NULL)
			kms_mode_object_put(&old_fb->base);
	}

	kms_mode_object_put(fb_obj);
	kms_mode_object_put(crtc_obj);
	return (error);
}

static int
kms_ioctl_mode_cursor_common(struct drm_file *file, uint32_t crtc_id,
    uint32_t flags, int32_t x, int32_t y, uint32_t width, uint32_t height,
    uint32_t handle, int32_t hot_x, int32_t hot_y)
{
	struct drm_mode_object *crtc_obj;
	struct drm_crtc *crtc;
	int error = 0;

	/*
	 * Only BO + MOVE bits are meaningful.  Reject anything else so a
	 * misaligned userspace stops failing silently.  Matches Linux
	 * drm_mode_cursor_common's DRM_MODE_CURSOR_FLAGS mask.
	 */
	if ((flags & ~(uint32_t)(DRM_MODE_CURSOR_BO |
	    DRM_MODE_CURSOR_MOVE)) != 0)
		return (EINVAL);

	crtc_obj = kms_mode_object_find(file->dev, crtc_id,
	    DRM_MODE_OBJECT_CRTC);
	if (crtc_obj == NULL)
		return (ENOENT);
	crtc = __containerof(crtc_obj, struct drm_crtc, base);

	if (crtc->funcs == NULL || (crtc->funcs->cursor_set == NULL &&
	    crtc->funcs->cursor_move == NULL)) {
		kms_mode_object_put(crtc_obj);
		return (ENOTTY);
	}

	if ((flags & DRM_MODE_CURSOR_BO) != 0 &&
	    crtc->funcs->cursor_set != NULL) {
		error = crtc->funcs->cursor_set(crtc, file, handle, width,
		    height, hot_x, hot_y);
	}
	if (error == 0 && (flags & DRM_MODE_CURSOR_MOVE) != 0 &&
	    crtc->funcs->cursor_move != NULL) {
		error = crtc->funcs->cursor_move(crtc, x, y);
	}

	kms_mode_object_put(crtc_obj);
	return (error);
}

int
kms_ioctl_mode_cursor(struct drm_file *file, struct drm_mode_cursor *r)
{
	return (kms_ioctl_mode_cursor_common(file, r->crtc_id, r->flags,
	    r->x, r->y, r->width, r->height, r->handle, 0, 0));
}

int
kms_ioctl_mode_cursor2(struct drm_file *file, struct drm_mode_cursor2 *r)
{
	return (kms_ioctl_mode_cursor_common(file, r->crtc_id, r->flags,
	    r->x, r->y, r->width, r->height, r->handle, r->hot_x, r->hot_y));
}
