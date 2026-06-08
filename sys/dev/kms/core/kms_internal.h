/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Symbols shared between kms's own translation units.  Not part
 * of the public driver-facing API (which lives in include/kms/).
 */

#ifndef _KMS_INTERNAL_H_
#define _KMS_INTERNAL_H_

#include <sys/types.h>
#include <sys/malloc.h>

struct cdev;
struct cdevsw;
struct thread;
struct drm_device;
struct drm_file;
struct drm_mode_card_res;
struct drm_mode_crtc;
struct drm_mode_get_encoder;
struct drm_mode_get_connector;
struct drm_mode_get_plane;
struct drm_mode_get_plane_res;
struct drm_mode_create_dumb;
struct drm_mode_map_dumb;
struct drm_mode_destroy_dumb;

MALLOC_DECLARE(M_KMS);

/*
 * Cap applied to user-supplied count_* fields in GETRESOURCES /
 * GETPLANERESOURCES before we malloc.  Bigger than any realistic KMS
 * inventory; small enough that a hostile count_ field can't drive a
 * multi-megabyte alloc.  See feedback_kms_security_rules.md rule
 * 4 (overflow-checked size validation).
 */
#define	DRM_MODE_GETRES_MAX	4096

extern struct cdevsw	kms_cdevsw;

int	kms_ioctl(struct cdev *dev, u_long cmd, caddr_t data,
	    int fflag, struct thread *td);

void	kms_device_acquire(struct drm_device *dev);
void	kms_device_release(struct drm_device *dev);

int	kms_ioctl_mode_getresources(struct drm_file *file,
	    struct drm_mode_card_res *r);
int	kms_ioctl_mode_getcrtc(struct drm_file *file,
	    struct drm_mode_crtc *r);
int	kms_ioctl_mode_getencoder(struct drm_file *file,
	    struct drm_mode_get_encoder *r);
int	kms_ioctl_mode_getconnector(struct drm_file *file,
	    struct drm_mode_get_connector *r);
int	kms_ioctl_mode_getplane_resources(struct drm_file *file,
	    struct drm_mode_get_plane_res *r);
int	kms_ioctl_mode_getplane(struct drm_file *file,
	    struct drm_mode_get_plane *r);
int	kms_ioctl_mode_create_dumb(struct drm_file *file,
	    struct drm_mode_create_dumb *args);
int	kms_ioctl_mode_map_dumb(struct drm_file *file,
	    struct drm_mode_map_dumb *args);
int	kms_ioctl_mode_destroy_dumb(struct drm_file *file,
	    struct drm_mode_destroy_dumb *args);

#endif /* _KMS_INTERNAL_H_ */
