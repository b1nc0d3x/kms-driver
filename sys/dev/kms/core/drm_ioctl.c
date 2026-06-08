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
	 * Phase 2: report nothing yet.  Capability bits get wired up as
	 * dumb buffers (DRM_CAP_DUMB_BUFFER), atomic (CAP_ATOMIC), etc.
	 * land in later phases.  Returning 0/value=0 is the documented
	 * "unsupported capability" response, which libdrm handles.
	 */
	c->value = 0;
	return (0);
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
	}
	return (ENOTTY);
}
