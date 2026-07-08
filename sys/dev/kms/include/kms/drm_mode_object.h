/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#ifndef _KMS_DRM_MODE_OBJECT_H_
#define _KMS_DRM_MODE_OBJECT_H_

#include <sys/types.h>
#include <sys/queue.h>

struct drm_device;

/*
 * Mode-object type identifiers.  Values match Linux DRM uapi so a
 * mechanically-ported driver doesn't have to translate them.
 */
#define	DRM_MODE_OBJECT_CRTC		0xcccccccc
#define	DRM_MODE_OBJECT_CONNECTOR	0xc0c0c0c0
#define	DRM_MODE_OBJECT_ENCODER		0xe0e0e0e0
#define	DRM_MODE_OBJECT_MODE		0xdededede
#define	DRM_MODE_OBJECT_PROPERTY	0xb0b0b0b0
#define	DRM_MODE_OBJECT_FB		0xfbfbfbfb
#define	DRM_MODE_OBJECT_BLOB		0xbbbbbbbb
#define	DRM_MODE_OBJECT_PLANE		0xeeeeeeee
#define	DRM_MODE_OBJECT_ANY		0

struct drm_object_property;

/*
 * Embedded base type for every KMS object.  Driver structs (drm_crtc,
 * drm_connector, ...) place this as their first member so a
 * drm_mode_object * can be cast to the concrete type.  ref starts at 1
 * after register and is incremented by kms_mode_object_find.
 *
 * Property table (Phase 8): TAILQ of drm_object_property entries.
 * Mutations and reads happen under drm_device->mode_config.mutex.
 */
struct drm_mode_object {
	uint32_t			 id;
	uint32_t			 type;
	u_int				 refs;
	TAILQ_ENTRY(drm_mode_object)	 link;	/* per-type list */
	TAILQ_ENTRY(drm_mode_object)	 reg;	/* master id->object list */
	TAILQ_HEAD(, drm_object_property) properties;
	uint32_t			 prop_count;
	/*
	 * Optional final-release callback.  Populated by the concrete
	 * object's init helper (e.g. kms_framebuffer_init); fired by
	 * kms_mode_object_put on the last refcount release so lifetime
	 * is owned by refs rather than by whoever calls _cleanup.  NULL
	 * for embedded-in-driver objects that never get freed
	 * individually (CRTCs, encoders, planes).
	 */
	void				(*free)(struct drm_mode_object *);
};

TAILQ_HEAD(drm_mode_object_list, drm_mode_object);

/*
 * Assign an ID, insert into the per-type and master lists.  Returns 0
 * on success.  Caller-owned storage; we do not allocate the object
 * itself.  Refcount is initialized to 1.
 */
int	kms_mode_object_register(struct drm_device *dev,
	    struct drm_mode_object *obj, uint32_t type);

/*
 * Same as kms_mode_object_register but caller already holds
 * dev->mode_config.mutex exclusive.  Used by paths that need to do
 * additional per-type bookkeeping (e.g. connector_type_id allocation)
 * atomically with the register itself.
 */
int	kms_mode_object_register_locked(struct drm_device *dev,
	    struct drm_mode_object *obj, uint32_t type);

/*
 * Remove from both lists.  Caller must guarantee no concurrent
 * kms_mode_object_find for the same id from this point on; outstanding
 * references prevent the storage being reclaimed by the caller.
 */
void	kms_mode_object_unregister(struct drm_device *dev,
	    struct drm_mode_object *obj);

/*
 * Refcounted lookup.  Returns NULL if no object with that id exists,
 * or if type is non-zero and doesn't match.  On success the returned
 * object has its refcount incremented; caller must drop the reference
 * with kms_mode_object_put.
 */
struct drm_mode_object *kms_mode_object_find(struct drm_device *dev,
	    uint32_t id, uint32_t type);

/*
 * Take an additional reference on an object the caller already holds
 * a ref to (via kms_mode_object_find or by being embedded in driver
 * storage that outlives this ref).  Used by SETCRTC / PAGE_FLIP to
 * pin the framebuffer for the lifetime of the crtc->primary_fb
 * assignment.  Never call this on an object whose ref could be zero.
 */
void	kms_mode_object_get(struct drm_mode_object *obj);

/*
 * Drop a reference acquired via kms_mode_object_find or _get.  Fires
 * the object's free callback on the final release.
 */
void	kms_mode_object_put(struct drm_mode_object *obj);

#endif /* _KMS_DRM_MODE_OBJECT_H_ */
