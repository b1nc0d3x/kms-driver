/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/errno.h>

#include <drm/drm.h>
#include <kms/drm_device.h>
#include <kms/drm_drv.h>
#include <kms/drm_file.h>

#include "kms_internal.h"

/*
 * Copy a NUL-terminated kernel string out to a user buffer described
 * by (uptr, *ulen).  *ulen on entry is the buffer capacity; on return
 * holds the source length (not including NUL).  Truncates without
 * error if the buffer is too small, matching Linux behavior.  uptr
 * may be NULL when userspace is probing for the required length.
 */
static int
drm_copy_name_out(const char *src, char *uptr, size_t *ulen)
{
	size_t srclen, n;
	int error;

	srclen = (src != NULL) ? strlen(src) : 0;
	if (uptr != NULL && *ulen > 0) {
		n = MIN(*ulen, srclen);
		error = copyout(src, uptr, n);
		if (error != 0)
			return (error);
	}
	*ulen = srclen;
	return (0);
}

static int
drm_ioctl_version(struct drm_file *file, struct drm_version *v)
{
	const struct drm_driver *drv;
	int error;

	drv = file->dev->driver;
	v->version_major = drv->major;
	v->version_minor = drv->minor;
	v->version_patchlevel = drv->patchlevel;
	error = drm_copy_name_out(drv->name, v->name, &v->name_len);
	if (error != 0)
		return (error);
	error = drm_copy_name_out(drv->date, v->date, &v->date_len);
	if (error != 0)
		return (error);
	return (drm_copy_name_out(drv->desc, v->desc, &v->desc_len));
}

static int
drm_ioctl_get_unique(struct drm_file *file, struct drm_unique *u)
{
	char busid[32];

	snprintf(busid, sizeof(busid), "kms:%d", file->dev->minor);
	return (drm_copy_name_out(busid, u->unique, &u->unique_len));
}

static int
drm_ioctl_set_version(struct drm_file *file __unused,
    struct drm_set_version *v)
{
	/*
	 * Userspace requests a DRM-interface version; we report ours
	 * back and accept whatever they asked for as long as the major
	 * matches.  drm_interface_version is 1.4 (legacy DRI) for now;
	 * later phases bump to advertise DRI3 / atomic / etc.
	 */
	if (v->drm_di_major != -1 && v->drm_di_major != 1)
		return (EINVAL);
	v->drm_di_major = 1;
	v->drm_di_minor = 4;
	v->drm_dd_major = file->dev->driver->major;
	v->drm_dd_minor = file->dev->driver->minor;
	return (0);
}

static int
drm_ioctl_get_cap(struct drm_file *file __unused, struct drm_get_cap *c)
{
	/*
	 * Known capabilities — we recognize them, return 0 to mean
	 * "supported but disabled / no value yet."  This is the
	 * distinction libdrm draws between "driver says no" (return 0,
	 * value 0) and "driver doesn't know this cap" (EINVAL).  Returning
	 * 0 for unknown ids would mislead libdrm into thinking we
	 * explicitly disabled features it never asked us about.
	 *
	 * Caps with non-zero values (cursor dimensions, prime flags, etc.)
	 * get populated as the phases that implement them land.
	 */
	switch (c->capability) {
	case DRM_CAP_DUMB_BUFFER:
	case DRM_CAP_VBLANK_HIGH_CRTC:
	case DRM_CAP_DUMB_PREFERRED_DEPTH:
	case DRM_CAP_DUMB_PREFER_SHADOW:
	case DRM_CAP_PRIME:
	case DRM_CAP_TIMESTAMP_MONOTONIC:
	case DRM_CAP_ASYNC_PAGE_FLIP:
	case DRM_CAP_CURSOR_WIDTH:
	case DRM_CAP_CURSOR_HEIGHT:
	case DRM_CAP_ADDFB2_MODIFIERS:
	case DRM_CAP_PAGE_FLIP_TARGET:
	case DRM_CAP_CRTC_IN_VBLANK_EVENT:
	case DRM_CAP_SYNCOBJ:
	case DRM_CAP_SYNCOBJ_TIMELINE:
	case DRM_CAP_ATOMIC_ASYNC_PAGE_FLIP:
		c->value = 0;
		return (0);
	}
	return (EINVAL);
}

int
kms_ioctl(struct cdev *cdev __unused, u_long cmd, caddr_t data,
    int fflag __unused, struct thread *td __unused)
{
	struct drm_file *file;
	int error;

	error = devfs_get_cdevpriv((void **)&file);
	if (error != 0)
		return (error);

	switch (cmd) {
	case DRM_IOCTL_VERSION:
		return (drm_ioctl_version(file, (struct drm_version *)data));
	case DRM_IOCTL_GET_UNIQUE:
		return (drm_ioctl_get_unique(file,
		    (struct drm_unique *)data));
	case DRM_IOCTL_SET_VERSION:
		return (drm_ioctl_set_version(file,
		    (struct drm_set_version *)data));
	case DRM_IOCTL_GET_CAP:
		return (drm_ioctl_get_cap(file, (struct drm_get_cap *)data));
	case DRM_IOCTL_MODE_GETRESOURCES:
		return (kms_ioctl_mode_getresources(file,
		    (struct drm_mode_card_res *)data));
	case DRM_IOCTL_MODE_GETCRTC:
		return (kms_ioctl_mode_getcrtc(file,
		    (struct drm_mode_crtc *)data));
	case DRM_IOCTL_MODE_GETENCODER:
		return (kms_ioctl_mode_getencoder(file,
		    (struct drm_mode_get_encoder *)data));
	case DRM_IOCTL_MODE_GETCONNECTOR:
		return (kms_ioctl_mode_getconnector(file,
		    (struct drm_mode_get_connector *)data));
	case DRM_IOCTL_MODE_GETPLANERESOURCES:
		return (kms_ioctl_mode_getplane_resources(file,
		    (struct drm_mode_get_plane_res *)data));
	case DRM_IOCTL_MODE_GETPLANE:
		return (kms_ioctl_mode_getplane(file,
		    (struct drm_mode_get_plane *)data));
	case DRM_IOCTL_MODE_CREATE_DUMB:
		return (kms_ioctl_mode_create_dumb(file,
		    (struct drm_mode_create_dumb *)data));
	case DRM_IOCTL_MODE_MAP_DUMB:
		return (kms_ioctl_mode_map_dumb(file,
		    (struct drm_mode_map_dumb *)data));
	case DRM_IOCTL_MODE_DESTROY_DUMB:
		return (kms_ioctl_mode_destroy_dumb(file,
		    (struct drm_mode_destroy_dumb *)data));
	case DRM_IOCTL_MODE_ADDFB2:
		return (kms_ioctl_mode_addfb2(file,
		    (struct drm_mode_fb_cmd2 *)data));
	case DRM_IOCTL_MODE_RMFB:
		return (kms_ioctl_mode_rmfb(file, (uint32_t *)data));
	case DRM_IOCTL_MODE_SETCRTC:
		return (kms_ioctl_mode_setcrtc(file,
		    (struct drm_mode_crtc *)data));
	case DRM_IOCTL_MODE_PAGE_FLIP:
		return (kms_ioctl_mode_page_flip(file,
		    (struct drm_mode_crtc_page_flip *)data));
	case DRM_IOCTL_SET_CLIENT_CAP:
		return (kms_ioctl_set_client_cap(file,
		    (struct drm_set_client_cap *)data));
	case DRM_IOCTL_MODE_GETPROPERTY:
		return (kms_ioctl_mode_getproperty(file,
		    (struct drm_mode_get_property *)data));
	case DRM_IOCTL_MODE_OBJ_GETPROPERTIES:
		return (kms_ioctl_mode_obj_getproperties(file,
		    (struct drm_mode_obj_get_properties *)data));
	case DRM_IOCTL_MODE_OBJ_SETPROPERTY:
		return (kms_ioctl_mode_obj_setproperty(file,
		    (struct drm_mode_obj_set_property *)data));
	case DRM_IOCTL_MODE_CREATEPROPBLOB:
		return (kms_ioctl_mode_createpropblob(file,
		    (struct drm_mode_create_blob *)data));
	case DRM_IOCTL_MODE_DESTROYPROPBLOB:
		return (kms_ioctl_mode_destroypropblob(file,
		    (struct drm_mode_destroy_blob *)data));
	case DRM_IOCTL_MODE_GETPROPBLOB:
		return (kms_ioctl_mode_getpropblob(file,
		    (struct drm_mode_get_blob *)data));
	case DRM_IOCTL_MODE_ATOMIC:
		return (kms_ioctl_mode_atomic(file,
		    (struct drm_mode_atomic *)data));
	}
	return (ENOTTY);
}
