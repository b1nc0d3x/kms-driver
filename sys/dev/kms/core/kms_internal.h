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
struct drm_mode_fb_cmd2;
struct drm_mode_fb_cmd;
struct drm_mode_closefb;
struct drm_mode_crtc_page_flip;
struct drm_mode_cursor;
struct drm_mode_cursor2;
struct drm_set_client_cap;
struct drm_mode_get_property;
struct drm_mode_obj_get_properties;
struct drm_mode_obj_set_property;
struct drm_mode_create_blob;
struct drm_mode_destroy_blob;
struct drm_mode_get_blob;
struct drm_mode_atomic;
struct drm_syncobj_create;
struct drm_syncobj_destroy;
struct drm_syncobj_wait;
struct drm_syncobj_array;
struct drm_syncobj_handle;

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
int	kms_ioctl_mode_addfb2(struct drm_file *file,
	    struct drm_mode_fb_cmd2 *cmd);
int	kms_ioctl_mode_addfb(struct drm_file *file,
	    struct drm_mode_fb_cmd *cmd);
int	kms_ioctl_mode_rmfb(struct drm_file *file, uint32_t *fb_id);
int	kms_ioctl_mode_closefb(struct drm_file *file,
	    struct drm_mode_closefb *arg);
int	kms_ioctl_mode_getfb(struct drm_file *file,
	    struct drm_mode_fb_cmd *r);
int	kms_ioctl_mode_getfb2(struct drm_file *file,
	    struct drm_mode_fb_cmd2 *r);
int	kms_ioctl_mode_setcrtc(struct drm_file *file,
	    struct drm_mode_crtc *r);
int	kms_ioctl_mode_page_flip(struct drm_file *file,
	    struct drm_mode_crtc_page_flip *r);
int	kms_ioctl_mode_cursor(struct drm_file *file,
	    struct drm_mode_cursor *r);
int	kms_ioctl_mode_cursor2(struct drm_file *file,
	    struct drm_mode_cursor2 *r);
int	kms_ioctl_set_client_cap(struct drm_file *file,
	    struct drm_set_client_cap *cap);
int	kms_ioctl_mode_getproperty(struct drm_file *file,
	    struct drm_mode_get_property *r);
int	kms_ioctl_mode_obj_getproperties(struct drm_file *file,
	    struct drm_mode_obj_get_properties *r);
int	kms_ioctl_mode_obj_setproperty(struct drm_file *file,
	    struct drm_mode_obj_set_property *r);
int	kms_ioctl_mode_createpropblob(struct drm_file *file,
	    struct drm_mode_create_blob *r);
int	kms_ioctl_mode_destroypropblob(struct drm_file *file,
	    struct drm_mode_destroy_blob *r);
int	kms_ioctl_mode_getpropblob(struct drm_file *file,
	    struct drm_mode_get_blob *r);
int	kms_ioctl_mode_atomic(struct drm_file *file,
	    struct drm_mode_atomic *r);

int	kms_ioctl_syncobj_create(struct drm_file *file,
	    struct drm_syncobj_create *args);
int	kms_ioctl_syncobj_destroy(struct drm_file *file,
	    struct drm_syncobj_destroy *args);
int	kms_ioctl_syncobj_wait(struct drm_file *file,
	    struct drm_syncobj_wait *args);
int	kms_ioctl_syncobj_reset(struct drm_file *file,
	    struct drm_syncobj_array *args);
int	kms_ioctl_syncobj_signal(struct drm_file *file,
	    struct drm_syncobj_array *args);
int	kms_ioctl_syncobj_handle_to_fd(struct thread *td,
	    struct drm_file *file, struct drm_syncobj_handle *args);
void	kms_syncobj_release_all(struct drm_file *file);

struct drm_crtc;
union drm_wait_vblank;

void	kms_event_queue_init(struct drm_file *file);
void	kms_event_queue_drain(struct drm_file *file);
int	kms_send_event(struct drm_file *file, const void *data,
	    size_t length);
int	kms_send_vblank_event(struct drm_file *file,
	    struct drm_crtc *crtc, uint32_t type, uint64_t user_data);
void	kms_vblank_handler(struct drm_crtc *crtc);
int	kms_ioctl_wait_vblank(struct drm_file *file,
	    union drm_wait_vblank *arg);

#endif /* _KMS_INTERNAL_H_ */
