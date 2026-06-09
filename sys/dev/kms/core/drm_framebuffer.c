/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>

#include <drm/drm_mode.h>

#include <kms/drm_device.h>
#include <kms/drm_file.h>
#include <kms/drm_framebuffer.h>
#include <kms/drm_gem.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_mode_object.h>

#include "kms_internal.h"

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

int
kms_framebuffer_init(struct drm_device *dev, struct drm_framebuffer *fb,
    const struct drm_framebuffer_funcs *funcs)
{
	if (dev == NULL || fb == NULL)
		return (EINVAL);

	fb->dev = dev;
	fb->funcs = funcs;

	return (kms_mode_object_register(dev, &fb->base, DRM_MODE_OBJECT_FB));
}

void
kms_framebuffer_cleanup(struct drm_framebuffer *fb)
{
	int i;

	if (fb == NULL || fb->dev == NULL)
		return;
	kms_mode_object_unregister(fb->dev, &fb->base);
	/*
	 * Drop GEM refs taken at create time.  If a userspace mapping
	 * is still alive against any of these BOs the GEM pager keeps
	 * the pages around; the BO storage itself is released here.
	 */
	for (i = 0; i < DRM_FORMAT_MAX_PLANES; i++) {
		if (fb->gem_objs[i] != NULL) {
			kms_gem_object_put(fb->gem_objs[i]);
			fb->gem_objs[i] = NULL;
		}
	}
	if (fb->funcs != NULL && fb->funcs->destroy != NULL)
		fb->funcs->destroy(fb);
}
