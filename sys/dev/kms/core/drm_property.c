/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Property + blob framework.  See drm_property.h for the design.
 *
 * Properties are themselves registered as DRM_MODE_OBJECT_PROPERTY
 * mode objects, so they share the device's 32-bit id namespace with
 * CRTCs / planes / connectors.  Same for blobs (DRM_MODE_OBJECT_BLOB).
 * Lookups via kms_mode_object_find work transparently.
 *
 * Locking model:
 *   - property + blob registration / destroy: mode_config.mutex
 *     exclusive.
 *   - per-object attach / set / get: mode_config.mutex exclusive
 *     (object_properties cleanup runs under it during unregister).
 *   - find paths take a shared lookup ref via mode_object_find which
 *     internally takes a brief shared lock — no deadlock with the
 *     callers that hold the mutex.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/queue.h>
#include <sys/sx.h>

#include <kms/drm_device.h>
#include <kms/drm_file.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_mode_object.h>
#include <kms/drm_property.h>

#include "kms_internal.h"

static struct drm_property *
drm_property_alloc(struct drm_device *dev, uint32_t flags, const char *name)
{
	struct drm_property *prop;
	size_t n;

	prop = malloc(sizeof(*prop), M_KMS, M_WAITOK | M_ZERO);
	prop->dev = dev;
	prop->flags = flags;
	TAILQ_INIT(&prop->entries);
	n = strnlen(name, sizeof(prop->name) - 1);
	memcpy(prop->name, name, n);
	prop->name[n] = '\0';
	return (prop);
}

struct drm_property *
kms_property_create_range(struct drm_device *dev, uint32_t flags,
    const char *name, uint64_t min, uint64_t max)
{
	struct drm_property *prop;
	int error;

	/*
	 * SIGNED_RANGE is an extended-type id that occupies bits 6-15 of
	 * the flags field; it is mutually exclusive with the legacy RANGE
	 * bit (bit 1).  Linux DRM checks RANGE first and treats the value
	 * pair as uint64 when set, so setting both at once silently
	 * downgrades a signed range to unsigned and any int32_min-style
	 * lower bound looks like a huge positive number to userspace.
	 */
	if ((flags & KMS_PROP_SIGNED_RANGE) == 0)
		flags |= KMS_PROP_RANGE;
	prop = drm_property_alloc(dev, flags, name);
	prop->values[0] = min;
	prop->values[1] = max;
	prop->num_values = 2;
	error = kms_mode_object_register(dev, &prop->base,
	    DRM_MODE_OBJECT_PROPERTY);
	if (error != 0) {
		free(prop, M_KMS);
		return (NULL);
	}
	return (prop);
}

struct drm_property *
kms_property_create_object(struct drm_device *dev, uint32_t flags,
    const char *name, uint32_t obj_type)
{
	struct drm_property *prop;
	int error;

	prop = drm_property_alloc(dev, flags | KMS_PROP_OBJECT, name);
	prop->values[0] = obj_type;
	prop->num_values = 1;
	error = kms_mode_object_register(dev, &prop->base,
	    DRM_MODE_OBJECT_PROPERTY);
	if (error != 0) {
		free(prop, M_KMS);
		return (NULL);
	}
	return (prop);
}

struct drm_property *
kms_property_create_enum(struct drm_device *dev, uint32_t flags,
    const char *name)
{
	struct drm_property *prop;
	int error;

	prop = drm_property_alloc(dev, flags | KMS_PROP_ENUM, name);
	error = kms_mode_object_register(dev, &prop->base,
	    DRM_MODE_OBJECT_PROPERTY);
	if (error != 0) {
		free(prop, M_KMS);
		return (NULL);
	}
	return (prop);
}

int
kms_property_add_enum(struct drm_property *prop, uint64_t value,
    const char *name)
{
	struct drm_property_enum *e;
	size_t n;

	if (prop == NULL ||
	    (prop->flags & (KMS_PROP_ENUM | KMS_PROP_BITMASK)) == 0)
		return (EINVAL);
	e = malloc(sizeof(*e), M_KMS, M_WAITOK | M_ZERO);
	e->value = value;
	n = strnlen(name, sizeof(e->name) - 1);
	memcpy(e->name, name, n);
	e->name[n] = '\0';
	sx_xlock(&prop->dev->mode_config.mutex);
	TAILQ_INSERT_TAIL(&prop->entries, e, link);
	prop->num_entries++;
	sx_xunlock(&prop->dev->mode_config.mutex);
	return (0);
}

struct drm_property *
kms_property_create_blob_prop(struct drm_device *dev, uint32_t flags,
    const char *name)
{
	struct drm_property *prop;
	int error;

	prop = drm_property_alloc(dev, flags | KMS_PROP_BLOB, name);
	error = kms_mode_object_register(dev, &prop->base,
	    DRM_MODE_OBJECT_PROPERTY);
	if (error != 0) {
		free(prop, M_KMS);
		return (NULL);
	}
	return (prop);
}

void
kms_property_destroy(struct drm_property *prop)
{
	struct drm_property_enum *e;

	if (prop == NULL)
		return;
	kms_mode_object_unregister(prop->dev, &prop->base);
	while ((e = TAILQ_FIRST(&prop->entries)) != NULL) {
		TAILQ_REMOVE(&prop->entries, e, link);
		free(e, M_KMS);
	}
	free(prop, M_KMS);
}

/* --- per-object attach + set + get --- */

int
kms_object_attach_property(struct drm_mode_object *obj,
    struct drm_property *prop, uint64_t default_value)
{
	struct drm_object_property *op;
	struct drm_object_property *existing;

	if (obj == NULL || prop == NULL)
		return (EINVAL);
	TAILQ_FOREACH(existing, &obj->properties, link) {
		if (existing->property == prop) {
			/*
			 * Idempotent: drivers that re-register the same
			 * property on the same object just get the existing
			 * entry's value updated.  Matches Linux behavior.
			 */
			existing->value = default_value;
			return (0);
		}
	}
	op = malloc(sizeof(*op), M_KMS, M_WAITOK | M_ZERO);
	op->property = prop;
	op->value = default_value;
	TAILQ_INSERT_TAIL(&obj->properties, op, link);
	obj->prop_count++;
	return (0);
}

int
kms_object_property_set_value(struct drm_mode_object *obj,
    struct drm_property *prop, uint64_t value)
{
	struct drm_object_property *op;

	TAILQ_FOREACH(op, &obj->properties, link) {
		if (op->property == prop) {
			op->value = value;
			return (0);
		}
	}
	return (ENOENT);
}

int
kms_object_property_get_value(struct drm_mode_object *obj,
    struct drm_property *prop, uint64_t *value_out)
{
	struct drm_object_property *op;

	TAILQ_FOREACH(op, &obj->properties, link) {
		if (op->property == prop) {
			*value_out = op->value;
			return (0);
		}
	}
	return (ENOENT);
}

void
kms_object_properties_cleanup(struct drm_mode_object *obj)
{
	struct drm_object_property *op;

	while ((op = TAILQ_FIRST(&obj->properties)) != NULL) {
		TAILQ_REMOVE(&obj->properties, op, link);
		free(op, M_KMS);
	}
	obj->prop_count = 0;
}

/* --- blobs --- */

void
kms_property_blob_orphan_owner(struct drm_device *dev, struct drm_file *file)
{
	struct drm_mode_config *mc;
	struct drm_mode_object *obj;
	struct drm_property_blob *blob;

	if (dev == NULL || file == NULL)
		return;
	mc = &dev->mode_config;
	sx_xlock(&mc->mutex);
	TAILQ_FOREACH(obj, &mc->objects, reg) {
		if (obj->type != DRM_MODE_OBJECT_BLOB)
			continue;
		blob = __containerof(obj, struct drm_property_blob, base);
		if (blob->owner == file)
			blob->owner = NULL;
	}
	sx_xunlock(&mc->mutex);
}

struct drm_property_blob *
kms_property_blob_create(struct drm_device *dev, struct drm_file *owner,
    const void *data, size_t length)
{
	struct drm_property_blob *blob;
	int error;

	if (dev == NULL || length == 0)
		return (NULL);
	if (length > (16ULL << 20))	/* 16 MiB sanity cap, rule 4 */
		return (NULL);

	/*
	 * M2 (2026-08-03 review): cap blobs per file so a client can't
	 * exhaust kernel memory with 1 MiB blobs.  Only enforced when we
	 * have an owner (compositor path — kernel-internal blobs from
	 * modeset construction bypass this).
	 */
	if (owner != NULL && owner->blob_count >= KMS_MAX_BLOBS_PER_FILE)
		return (NULL);

	blob = malloc(sizeof(*blob), M_KMS, M_WAITOK | M_ZERO);
	blob->dev = dev;
	blob->owner = owner;
	blob->length = length;
	blob->data = malloc(length, M_KMS, M_WAITOK);
	if (data != NULL)
		memcpy(blob->data, data, length);
	else
		memset(blob->data, 0, length);

	error = kms_mode_object_register(dev, &blob->base,
	    DRM_MODE_OBJECT_BLOB);
	if (error != 0) {
		free(blob->data, M_KMS);
		free(blob, M_KMS);
		return (NULL);
	}
	if (owner != NULL)
		owner->blob_count++;
	return (blob);
}

struct drm_property_blob *
kms_property_blob_find(struct drm_device *dev, uint32_t id)
{
	struct drm_mode_object *obj;

	obj = kms_mode_object_find(dev, id, DRM_MODE_OBJECT_BLOB);
	if (obj == NULL)
		return (NULL);
	return (__containerof(obj, struct drm_property_blob, base));
}

void
kms_property_blob_destroy(struct drm_property_blob *blob)
{
	struct drm_file *owner;

	if (blob == NULL)
		return;
	/*
	 * M2 (2026-08-03 review): decrement the owning file's blob count
	 * so a well-behaved client that creates+destroys blobs at frame
	 * rate isn't spuriously capped.  Owner may be NULL for orphaned
	 * blobs (see kms_property_blob_orphan_owner) or kernel-internal
	 * blobs — skip decrement in that case.
	 */
	owner = blob->owner;
	if (owner != NULL && owner->blob_count > 0)
		owner->blob_count--;
	kms_mode_object_unregister(blob->dev, &blob->base);
	free(blob->data, M_KMS);
	free(blob, M_KMS);
}
