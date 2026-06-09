/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Property + blob framework for atomic modeset.
 *
 * Every KMS mode object (CRTC, plane, connector) carries a property
 * table — a list of (drm_property *, value) pairs.  Userspace sees the
 * properties via OBJ_GETPROPERTIES and writes them in batch via the
 * ATOMIC ioctl.  Property metadata (name, value range, ENUM entries)
 * is per-property and shared across all objects that attach it.
 *
 * Blobs are an immutable byte string indexed by a 32-bit id allocated
 * from the same id space as other mode objects (so the ATOMIC ioctl
 * can pass blob ids as property values transparently).  The canonical
 * use case is CRTC.MODE_ID, which carries a drm_mode_modeinfo blob.
 */

#ifndef _KMS_DRM_PROPERTY_H_
#define _KMS_DRM_PROPERTY_H_

#include <sys/types.h>
#include <sys/queue.h>

#include <kms/drm_mode_object.h>

struct drm_device;
struct drm_mode_object;

#define	KMS_PROP_NAME_LEN	32

/*
 * Property flag bits.  Values match Linux uapi DRM_MODE_PROP_* so the
 * kernel-side constants can be copied straight to GETPROPERTY's
 * flags field without translation.
 */
#define	KMS_PROP_PENDING		(1u << 0)
#define	KMS_PROP_RANGE		(1u << 1)
#define	KMS_PROP_IMMUTABLE	(1u << 2)
#define	KMS_PROP_ENUM		(1u << 3)
#define	KMS_PROP_BLOB		(1u << 4)
#define	KMS_PROP_BITMASK		(1u << 5)
#define	KMS_PROP_OBJECT		(1u << 24)
#define	KMS_PROP_SIGNED_RANGE	(1u << 25)
#define	KMS_PROP_ATOMIC		(1u << 31)

/*
 * ENUM entry stored on an ENUM/BITMASK property.  Values match the
 * uapi drm_mode_property_enum struct so GETPROPERTY's enum_blob_ptr
 * fill can copy directly.
 */
struct drm_property_enum {
	uint64_t	value;
	char		name[KMS_PROP_NAME_LEN];
	TAILQ_ENTRY(drm_property_enum)	link;
};

/*
 * One property.  Registered as DRM_MODE_OBJECT_PROPERTY in the device
 * id space so userspace can refer to it by id alongside CRTCs etc.
 * For RANGE / SIGNED_RANGE the (min, max) tuple lives in values[];
 * for ENUM the entries list holds (value, name) pairs; for OBJECT
 * the values[0] field holds the object type id (e.g.
 * DRM_MODE_OBJECT_FB) so set-property can validate the target.
 */
struct drm_property {
	struct drm_mode_object		 base;
	struct drm_device		*dev;
	uint32_t			 flags;
	char				 name[KMS_PROP_NAME_LEN];
	uint64_t			 values[2];
	TAILQ_HEAD(, drm_property_enum)	 entries;
	uint32_t			 num_values;
	uint32_t			 num_entries;
};

/*
 * Property blob.  Immutable byte array indexed by id.  Allocated from
 * drm_property_blob_create, freed when its refcount hits zero
 * (released by destroy ioctl, or when no longer referenced as a
 * property value — Phase 8 conservative model is "blobs are
 * userspace-managed, destroyed when userspace says so").
 */
struct drm_property_blob {
	struct drm_mode_object		 base;
	struct drm_device		*dev;
	size_t				 length;
	uint8_t				*data;
};

/*
 * Per-object property entry.  Each mode object holds a list of these.
 * value is interpreted by the property's flags: RANGE/SIGNED_RANGE
 * = scalar, ENUM = enum value, BLOB = blob id, OBJECT = mode object id.
 */
struct drm_object_property {
	struct drm_property		*property;
	uint64_t			 value;
	TAILQ_ENTRY(drm_object_property) link;
};

TAILQ_HEAD(drm_object_property_list, drm_object_property);

/* --- property registration --- */

struct drm_property *drm_property_create_range(struct drm_device *dev,
	    uint32_t flags, const char *name, uint64_t min, uint64_t max);
struct drm_property *drm_property_create_object(struct drm_device *dev,
	    uint32_t flags, const char *name, uint32_t obj_type);
struct drm_property *drm_property_create_enum(struct drm_device *dev,
	    uint32_t flags, const char *name);
int	drm_property_add_enum(struct drm_property *prop, uint64_t value,
	    const char *name);
struct drm_property *drm_property_create_blob_prop(struct drm_device *dev,
	    uint32_t flags, const char *name);
void	drm_property_destroy(struct drm_property *prop);

/* --- per-object attachment + lookup --- */

int	drm_object_attach_property(struct drm_mode_object *obj,
	    struct drm_property *prop, uint64_t default_value);
int	drm_object_property_set_value(struct drm_mode_object *obj,
	    struct drm_property *prop, uint64_t value);
int	drm_object_property_get_value(struct drm_mode_object *obj,
	    struct drm_property *prop, uint64_t *value_out);
void	drm_object_properties_cleanup(struct drm_mode_object *obj);

/* --- blob lifecycle --- */

struct drm_property_blob *drm_property_blob_create(struct drm_device *dev,
	    const void *data, size_t length);
struct drm_property_blob *drm_property_blob_find(struct drm_device *dev,
	    uint32_t id);
void	drm_property_blob_destroy(struct drm_property_blob *blob);

#endif /* _KMS_DRM_PROPERTY_H_ */
