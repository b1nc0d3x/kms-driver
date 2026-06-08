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
struct drm_file;
struct drm_mode_card_res;
struct drm_mode_crtc;
struct drm_mode_get_encoder;
struct drm_mode_get_connector;

MALLOC_DECLARE(M_KMS);

extern struct cdevsw	kms_cdevsw;

int	kms_ioctl(struct cdev *dev, u_long cmd, caddr_t data,
	    int fflag, struct thread *td);

int	kms_ioctl_mode_getresources(struct drm_file *file,
	    struct drm_mode_card_res *r);
int	kms_ioctl_mode_getcrtc(struct drm_file *file,
	    struct drm_mode_crtc *r);
int	kms_ioctl_mode_getencoder(struct drm_file *file,
	    struct drm_mode_get_encoder *r);
int	kms_ioctl_mode_getconnector(struct drm_file *file,
	    struct drm_mode_get_connector *r);

#endif /* _KMS_INTERNAL_H_ */
