/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>

#include <drm/drm_fourcc.h>
#include <drm/drm_mode.h>

#include <kms/drm_device.h>
#include <kms/drm_file.h>
#include <kms/drm_framebuffer.h>
#include <kms/drm_gem.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_mode_object.h>

#include "kms_internal.h"

/*
 * Bytes-per-pixel for the packed single-plane formats we accept for
 * scanout.  Returns 0 for anything else — planar/multi-plane / paletted
 * layouts fall through to be rejected by kms_framebuffer_check_geometry,
 * matching Xorg's modesetting driver which only emits these.
 */
static unsigned int
kms_fb_format_cpp(uint32_t fourcc)
{
	switch (fourcc) {
	case DRM_FORMAT_RGB565:
	case DRM_FORMAT_BGR565:
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_XRGB1555:
	case DRM_FORMAT_ABGR1555:
	case DRM_FORMAT_XBGR1555:
	case DRM_FORMAT_ARGB4444:
	case DRM_FORMAT_XRGB4444:
		return (2);
	case DRM_FORMAT_RGB888:
	case DRM_FORMAT_BGR888:
		return (3);
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_BGRA8888:
	case DRM_FORMAT_BGRX8888:
	case DRM_FORMAT_ARGB2101010:
	case DRM_FORMAT_XRGB2101010:
	case DRM_FORMAT_ABGR2101010:
	case DRM_FORMAT_XBGR2101010:
		return (4);
	default:
		return (0);
	}
}

/*
 * Reject framebuffers whose backing BO can't hold the addressed pixels.
 * A scanout engine walks `offsets[0] + pitches[0]*(height-1) +
 * width*cpp` bytes when reading the last row; if the BO is shorter, the
 * DMA engine crosses into whatever follows in the object table — OOB
 * read / info leak (CVE-2026-46209 pattern).  Cross-checks:
 *   - format is one we recognize as packed single-plane (cpp != 0);
 *   - width * cpp doesn't overflow 32 bits;
 *   - pitch >= width * cpp (rules out UB rounding + backed-buffer
 *     shrinkage);
 *   - offset + pitch * (height-1) + width * cpp <= gem_obj->size,
 *     computed in size_t with explicit overflow checks so a hostile
 *     pitch/height combo can't wrap round the size to a value that
 *     passes.
 */
static int
kms_framebuffer_check_geometry(const struct drm_mode_fb_cmd2 *fb_cmd,
    const struct drm_gem_object *obj)
{
	unsigned int cpp;
	uint64_t line_bytes, tail_bytes, min_size;

	cpp = kms_fb_format_cpp(fb_cmd->pixel_format);
	if (cpp == 0)
		return (EINVAL);

	line_bytes = (uint64_t)fb_cmd->width * cpp;
	if (line_bytes > UINT32_MAX)
		return (EINVAL);
	if (fb_cmd->pitches[0] < line_bytes)
		return (EINVAL);

	/*
	 * Compute min_size in 64-bit to catch pitch*height overflow.
	 * Rows 0..height-2 are full pitch bytes; the last row only needs
	 * width*cpp so we don't reject BOs that are exactly stride-fit.
	 */
	tail_bytes = (uint64_t)fb_cmd->pitches[0] *
	    ((uint64_t)fb_cmd->height - 1);
	min_size = (uint64_t)fb_cmd->offsets[0] + tail_bytes + line_bytes;
	if (min_size > obj->size)
		return (EINVAL);
	return (0);
}

static void
kms_framebuffer_destroy_default(struct drm_framebuffer *fb)
{
	free(fb, M_KMS);
}

static const struct drm_framebuffer_funcs drm_framebuffer_default_funcs = {
	.destroy = kms_framebuffer_destroy_default,
};

int
kms_framebuffer_create(struct drm_file *file,
    const struct drm_mode_fb_cmd2 *fb_cmd, struct drm_framebuffer **out_fb)
{
	struct drm_framebuffer *fb;
	struct drm_gem_object *obj;
	int i, error;
	int taken = 0;

	if (file == NULL || fb_cmd == NULL || out_fb == NULL)
		return (EINVAL);
	if (fb_cmd->width == 0 || fb_cmd->height == 0)
		return (EINVAL);
	if (fb_cmd->handles[0] == 0)
		return (EINVAL);

	fb = malloc(sizeof(*fb), M_KMS, M_WAITOK | M_ZERO);
	fb->width = fb_cmd->width;
	fb->height = fb_cmd->height;
	fb->format = fb_cmd->pixel_format;
	fb->modifier = (fb_cmd->flags & DRM_MODE_FB_MODIFIERS) ?
	    fb_cmd->modifier[0] : 0;

	/*
	 * Resolve every non-zero handle and pin the GEM object.  We
	 * stop at the first zero handle — Linux uapi rule: planes are
	 * dense from index 0 and a zero handle terminates the list
	 * unless DRM_MODE_FB_MODIFIERS is set, in which case the
	 * sentinel is a zero pitch.  Phase 7 covers the common single-
	 * plane case correctly; multi-plane planar formats land when
	 * a driver needs them.
	 */
	for (i = 0; i < DRM_FORMAT_MAX_PLANES; i++) {
		if (fb_cmd->handles[i] == 0)
			break;
		obj = kms_gem_handle_lookup(file, fb_cmd->handles[i]);
		if (obj == NULL) {
			error = ENOENT;
			goto fail;
		}
		fb->gem_objs[i] = obj;
		fb->handles[i] = fb_cmd->handles[i];
		fb->pitches[i] = fb_cmd->pitches[i];
		fb->offsets[i] = fb_cmd->offsets[i];
		taken++;
		/*
		 * Validate the plane 0 geometry against its BO before we
		 * hand the fb to the framework — later planes are only
		 * accepted for packed formats where cpp == 0 for i > 0
		 * would have already tripped the format check.
		 */
		if (i == 0) {
			error = kms_framebuffer_check_geometry(fb_cmd, obj);
			if (error != 0)
				goto fail;
		}
	}

	error = kms_framebuffer_init(file->dev, fb,
	    &drm_framebuffer_default_funcs);
	if (error != 0)
		goto fail;

	*out_fb = fb;
	return (0);

fail:
	for (i = 0; i < taken; i++)
		kms_gem_object_put(fb->gem_objs[i]);
	free(fb, M_KMS);
	return (error);
}

/*
 * Final-release callback wired into fb->base.free.  Runs when the last
 * reference drops (whether that's RMFB's cleanup, a SETCRTC that
 * replaced the primary_fb, or a completed page flip whose event has
 * been consumed).  Drops the GEM plane refs and hands the storage back
 * to the driver-supplied destroy hook.
 */
static void
kms_framebuffer_free_obj(struct drm_mode_object *base)
{
	struct drm_framebuffer *fb =
	    __containerof(base, struct drm_framebuffer, base);
	int i;

	for (i = 0; i < DRM_FORMAT_MAX_PLANES; i++) {
		if (fb->gem_objs[i] != NULL) {
			kms_gem_object_put(fb->gem_objs[i]);
			fb->gem_objs[i] = NULL;
		}
	}
	if (fb->funcs != NULL && fb->funcs->destroy != NULL)
		fb->funcs->destroy(fb);
}

int
kms_framebuffer_init(struct drm_device *dev, struct drm_framebuffer *fb,
    const struct drm_framebuffer_funcs *funcs)
{
	if (dev == NULL || fb == NULL)
		return (EINVAL);

	fb->dev = dev;
	fb->funcs = funcs;
	/*
	 * Wire the free callback BEFORE register so the very first put
	 * (in the failure path of a caller that immediately RMFBs) can
	 * find it.
	 */
	fb->base.free = kms_framebuffer_free_obj;

	return (kms_mode_object_register(dev, &fb->base, DRM_MODE_OBJECT_FB));
}

/*
 * "Please destroy this framebuffer when no more refs exist."  Idempotent
 * — concurrent RMFB from two threads both go through unregister, and
 * unregister no-ops if the object is already off the id table.  The
 * actual destruction (GEM refs + funcs->destroy) fires in the free
 * callback registered above, whenever the last ref drops.
 */
void
kms_framebuffer_cleanup(struct drm_framebuffer *fb)
{
	if (fb == NULL || fb->dev == NULL)
		return;
	kms_mode_object_unregister(fb->dev, &fb->base);
}
