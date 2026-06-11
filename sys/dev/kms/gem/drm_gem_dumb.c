/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * DRM_IOCTL_MODE_CREATE_DUMB / MAP_DUMB / DESTROY_DUMB.
 *
 * Phase 6 implements dumb buffers as the framework's *only* buffer
 * type — sufficient for X server scanout via the modesetting DDX,
 * and the only thing rk_drm needs from the GEM layer.  No GEM_OPEN /
 * GEM_CLOSE / PRIME yet; those add when a real GEM-aware client
 * (Mesa userspace, Vulkan WSI) appears.
 */

#include <sys/param.h>
#include <sys/systm.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>

#include <kms/drm_device.h>
#include <kms/drm_file.h>
#include <kms/drm_gem.h>

#include "kms_internal.h"

/*
 * Hard cap on a single dumb buffer.  256 MiB is enough for 8K@32bpp
 * with overhead and far less than what a hostile width*height*bpp
 * multiplication would compute.  Anything bigger almost certainly
 * indicates an integer-overflow attempt (security rule 4) — reject.
 */
#define	DRM_DUMB_MAX_BYTES	(256ULL << 20)

int
kms_ioctl_mode_create_dumb(struct drm_file *file,
    struct drm_mode_create_dumb *args)
{
	struct drm_gem_object *obj;
	uint64_t size, pitch;
	uint32_t handle;
	int error;

	if (file == NULL || args == NULL)
		return (EINVAL);
	/*
	 * Xorg's modesetting driver calls CREATE_DUMB with width=0,
	 * height=0, bpp=32 as part of its CRTC-shadow probe and segfaults
	 * if we return EINVAL.  Linux's drm core also returns EINVAL for
	 * 0x0 but modesetting must dodge this on real Linux drivers some
	 * other way (probably via a feature flag).  Clamp to 1x1 so
	 * userspace gets a valid handle and can free it again; the BO
	 * costs one page and pins no further state.
	 */
	if (args->width == 0)
		args->width = 1;
	if (args->height == 0)
		args->height = 1;
	if (args->bpp == 0)
		args->bpp = 32;
	if (args->bpp > 64 || (args->bpp & 7) != 0)
		return (EINVAL);
	if (args->flags != 0)
		return (EINVAL);

	/*
	 * Overflow-checked geometry math (security rule 4).  Pitch is
	 * (width * bpp/8) rounded to a 64-byte cache line; size is
	 * pitch * height.  All in 64-bit space; reject if size exceeds
	 * the per-BO cap or overflows the user's __u64 return.
	 */
	pitch = (uint64_t)args->width * (args->bpp / 8);
	if (pitch > UINT32_MAX || pitch == 0)
		return (EINVAL);
	pitch = (pitch + 63ULL) & ~63ULL;
	if (pitch > UINT32_MAX)
		return (EINVAL);
	if (args->height > (DRM_DUMB_MAX_BYTES / pitch))
		return (EINVAL);
	size = pitch * (uint64_t)args->height;
	if (size > DRM_DUMB_MAX_BYTES)
		return (EINVAL);

	obj = kms_gem_object_create(file->dev, (size_t)size);
	if (obj == NULL)
		return (ENOMEM);

	error = kms_gem_handle_create(file, obj, &handle);
	if (error != 0) {
		kms_gem_object_put(obj);
		return (error);
	}

	args->handle = handle;
	args->pitch = (uint32_t)pitch;
	args->size = obj->size;	/* page-rounded */

	/*
	 * Handle now owns its own ref; drop ours so the BO will be
	 * freed when userspace deletes the handle (or the file dtor
	 * runs).
	 */
	kms_gem_object_put(obj);
	return (0);
}

int
kms_ioctl_mode_map_dumb(struct drm_file *file,
    struct drm_mode_map_dumb *args)
{
	struct drm_gem_object *obj;

	if (file == NULL || args == NULL || args->handle == 0)
		return (EINVAL);
	obj = kms_gem_handle_lookup(file, args->handle);
	if (obj == NULL)
		return (ENOENT);
	args->offset = obj->mmap_offset;
	kms_gem_object_put(obj);
	return (0);
}

int
kms_ioctl_mode_destroy_dumb(struct drm_file *file,
    struct drm_mode_destroy_dumb *args)
{
	if (file == NULL || args == NULL)
		return (EINVAL);
	return (kms_gem_handle_delete(file, args->handle));
}
