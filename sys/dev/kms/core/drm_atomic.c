/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Atomic-modeset ioctl glue plus the property metadata / blob ioctls
 * required to make ATOMIC discoverable by userspace:
 *
 *   - SET_CLIENT_CAP            : opt into atomic / universal planes
 *   - MODE_GETPROPERTY          : describe one property's metadata
 *   - MODE_OBJ_GETPROPERTIES    : enumerate an object's (prop, value)
 *                                 pairs
 *   - MODE_OBJ_SETPROPERTY      : single-property write (legacy
 *                                 fallback for atomic-unaware userspace)
 *   - MODE_GETPROPBLOB / CREATEPROPBLOB / DESTROYPROPBLOB
 *   - MODE_ATOMIC               : batch property write across N objects
 *
 * The ATOMIC ioctl applies the proposed values directly to the
 * per-object property table.  Driver hooks (atomic_check /
 * atomic_commit) are NULL on the stub — the framework records what
 * userspace asked for so a follow-up OBJ_GETPROPERTIES round-trips.
 * Real hardware drivers (Phase 9 rk_drm port) install hooks that
 * walk the proposed state and program the SoC.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/sx.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>

#include <kms/drm_device.h>
#include <kms/drm_file.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_mode_object.h>
#include <kms/drm_property.h>

#include "kms_internal.h"

/*
 * Sanity caps on user-supplied counts so a hostile process can't push
 * us into a multi-GiB alloc (security rule 4).  Both numbers are
 * generous: ATOMIC over 256 objects with 256 props each per call is
 * far past any realistic compositor.
 */
#define	DRM_ATOMIC_MAX_OBJS		256
#define	DRM_ATOMIC_MAX_PROPS_PER_OBJ	256
#define	DRM_ATOMIC_MAX_TOTAL_PROPS	4096
#define	DRM_PROP_NAME_LEN		32

/* --- SET_CLIENT_CAP --- */

int
kms_ioctl_set_client_cap(struct drm_file *file,
    struct drm_set_client_cap *cap)
{
	if (file == NULL || cap == NULL)
		return (EINVAL);
	/*
	 * Known client capabilities all map to a single bit in
	 * drm_file->client_caps.  We don't gate any framework behavior
	 * on stereo_3d / aspect_ratio yet, but accepting them keeps
	 * libdrm clients happy.  Unknown caps return EINVAL (matches
	 * Linux).
	 */
	switch (cap->capability) {
	case DRM_CLIENT_CAP_STEREO_3D:
	case DRM_CLIENT_CAP_UNIVERSAL_PLANES:
	case DRM_CLIENT_CAP_ATOMIC:
	case DRM_CLIENT_CAP_ASPECT_RATIO:
	case DRM_CLIENT_CAP_WRITEBACK_CONNECTORS:
		if (cap->value > 1)
			return (EINVAL);
		if (cap->value)
			file->client_caps |= (1u << cap->capability);
		else
			file->client_caps &= ~(1u << cap->capability);
		return (0);
	}
	return (EINVAL);
}

/* --- MODE_GETPROPERTY --- */

int
kms_ioctl_mode_getproperty(struct drm_file *file,
    struct drm_mode_get_property *r)
{
	struct drm_mode_object *obj;
	struct drm_property *prop;
	struct drm_property_enum *e;
	size_t n;
	uint32_t i;
	int error = 0;

	if (file == NULL || r == NULL)
		return (EINVAL);
	obj = kms_mode_object_find(file->dev, r->prop_id,
	    DRM_MODE_OBJECT_PROPERTY);
	if (obj == NULL)
		return (ENOENT);
	prop = __containerof(obj, struct drm_property, base);

	r->flags = prop->flags;
	n = strnlen(prop->name, sizeof(prop->name));
	if (n >= DRM_PROP_NAME_LEN)
		n = DRM_PROP_NAME_LEN - 1;
	memset(r->name, 0, sizeof(r->name));
	memcpy(r->name, prop->name, n);

	if (r->count_values >= prop->num_values && r->values_ptr != 0 &&
	    prop->num_values > 0) {
		error = copyout(prop->values,
		    (void *)(uintptr_t)r->values_ptr,
		    prop->num_values * sizeof(uint64_t));
		if (error != 0)
			goto out;
	}
	r->count_values = prop->num_values;

	if (prop->flags & (KMS_PROP_ENUM | KMS_PROP_BITMASK)) {
		if (r->count_enum_blobs >= prop->num_entries &&
		    r->enum_blob_ptr != 0 && prop->num_entries > 0) {
			struct {
				uint64_t value;
				char	 name[DRM_PROP_NAME_LEN];
			} u;
			void *dst = (void *)(uintptr_t)r->enum_blob_ptr;

			i = 0;
			TAILQ_FOREACH(e, &prop->entries, link) {
				memset(&u, 0, sizeof(u));
				u.value = e->value;
				n = strnlen(e->name, sizeof(e->name));
				if (n >= DRM_PROP_NAME_LEN)
					n = DRM_PROP_NAME_LEN - 1;
				memcpy(u.name, e->name, n);
				error = copyout(&u, (uint8_t *)dst +
				    i * sizeof(u), sizeof(u));
				if (error != 0)
					goto out;
				i++;
			}
		}
		r->count_enum_blobs = prop->num_entries;
	} else {
		r->count_enum_blobs = 0;
	}

out:
	kms_mode_object_put(obj);
	return (error);
}

/* --- MODE_OBJ_GETPROPERTIES --- */

int
kms_ioctl_mode_obj_getproperties(struct drm_file *file,
    struct drm_mode_obj_get_properties *r)
{
	struct drm_mode_config *mc = &file->dev->mode_config;
	struct drm_mode_object *obj;
	struct drm_object_property *op;
	uint32_t *props = NULL;
	uint64_t *values = NULL;
	uint32_t n_props, want, written, i;
	int error = 0;

	if (file == NULL || r == NULL)
		return (EINVAL);
	obj = kms_mode_object_find(file->dev, r->obj_id, r->obj_type);
	if (obj == NULL)
		return (ENOENT);

	want = r->count_props;
	sx_slock(&mc->mutex);
	n_props = obj->prop_count;
	if (n_props > DRM_MODE_GETRES_MAX)
		n_props = DRM_MODE_GETRES_MAX;
	written = 0;
	if (want > 0 && n_props > 0) {
		uint32_t cap;

		cap = (want < n_props) ? want : n_props;
		props = malloc((size_t)cap * sizeof(uint32_t), M_KMS,
		    M_WAITOK);
		values = malloc((size_t)cap * sizeof(uint64_t), M_KMS,
		    M_WAITOK);
		i = 0;
		TAILQ_FOREACH(op, &obj->properties, link) {
			if (i >= cap)
				break;
			props[i] = op->property->base.id;
			values[i] = op->value;
			i++;
		}
		written = i;
	}
	sx_sunlock(&mc->mutex);

	if (written > 0) {
		if (r->props_ptr != 0) {
			error = copyout(props, (void *)(uintptr_t)r->props_ptr,
			    written * sizeof(uint32_t));
			if (error != 0)
				goto out;
		}
		if (r->prop_values_ptr != 0) {
			error = copyout(values,
			    (void *)(uintptr_t)r->prop_values_ptr,
			    written * sizeof(uint64_t));
			if (error != 0)
				goto out;
		}
	}
	r->count_props = n_props;

out:
	free(props, M_KMS);
	free(values, M_KMS);
	kms_mode_object_put(obj);
	return (error);
}

/* --- MODE_OBJ_SETPROPERTY (single-property write) --- */

int
kms_ioctl_mode_obj_setproperty(struct drm_file *file,
    struct drm_mode_obj_set_property *r)
{
	struct drm_mode_config *mc = &file->dev->mode_config;
	struct drm_mode_object *obj, *prop_obj;
	struct drm_property *prop;
	int error;

	if (file == NULL || r == NULL)
		return (EINVAL);
	obj = kms_mode_object_find(file->dev, r->obj_id, r->obj_type);
	if (obj == NULL)
		return (ENOENT);
	prop_obj = kms_mode_object_find(file->dev, r->prop_id,
	    DRM_MODE_OBJECT_PROPERTY);
	if (prop_obj == NULL) {
		kms_mode_object_put(obj);
		return (ENOENT);
	}
	prop = __containerof(prop_obj, struct drm_property, base);
	if (prop->flags & KMS_PROP_IMMUTABLE) {
		kms_mode_object_put(prop_obj);
		kms_mode_object_put(obj);
		return (EINVAL);
	}

	sx_xlock(&mc->mutex);
	error = kms_object_property_set_value(obj, prop, r->value);
	sx_xunlock(&mc->mutex);

	kms_mode_object_put(prop_obj);
	kms_mode_object_put(obj);
	return (error);
}

/* --- BLOB ioctls --- */

int
kms_ioctl_mode_createpropblob(struct drm_file *file,
    struct drm_mode_create_blob *r)
{
	struct drm_property_blob *blob;
	void *buf;
	int error;

	if (file == NULL || r == NULL)
		return (EINVAL);
	if (r->length == 0 || r->length > (1u << 20))
		return (EINVAL);
	buf = malloc(r->length, M_KMS, M_WAITOK);
	error = copyin((const void *)(uintptr_t)r->data, buf, r->length);
	if (error != 0) {
		free(buf, M_KMS);
		return (error);
	}
	blob = kms_property_blob_create(file->dev, buf, r->length);
	free(buf, M_KMS);
	if (blob == NULL)
		return (ENOMEM);
	r->blob_id = blob->base.id;
	return (0);
}

int
kms_ioctl_mode_destroypropblob(struct drm_file *file,
    struct drm_mode_destroy_blob *r)
{
	struct drm_mode_object *obj;
	struct drm_property_blob *blob;

	if (file == NULL || r == NULL)
		return (EINVAL);
	obj = kms_mode_object_find(file->dev, r->blob_id,
	    DRM_MODE_OBJECT_BLOB);
	if (obj == NULL)
		return (ENOENT);
	blob = __containerof(obj, struct drm_property_blob, base);
	kms_mode_object_put(obj);
	kms_property_blob_destroy(blob);
	return (0);
}

int
kms_ioctl_mode_getpropblob(struct drm_file *file,
    struct drm_mode_get_blob *r)
{
	struct drm_mode_object *obj;
	struct drm_property_blob *blob;
	uint32_t to_copy;
	int error = 0;

	if (file == NULL || r == NULL)
		return (EINVAL);
	obj = kms_mode_object_find(file->dev, r->blob_id,
	    DRM_MODE_OBJECT_BLOB);
	if (obj == NULL)
		return (ENOENT);
	blob = __containerof(obj, struct drm_property_blob, base);
	if (r->length > 0 && r->data != 0) {
		to_copy = ((uint32_t)blob->length < r->length) ?
		    (uint32_t)blob->length : r->length;
		error = copyout(blob->data, (void *)(uintptr_t)r->data,
		    to_copy);
	}
	r->length = (uint32_t)blob->length;
	kms_mode_object_put(obj);
	return (error);
}

/* --- MODE_ATOMIC --- */

int
kms_ioctl_mode_atomic(struct drm_file *file, struct drm_mode_atomic *r)
{
	struct drm_mode_config *mc = &file->dev->mode_config;
	uint32_t *objs = NULL;
	uint32_t *counts = NULL;
	uint32_t *props = NULL;
	uint64_t *values = NULL;
	struct drm_mode_object **resolved = NULL;
	struct drm_property **prop_resolved = NULL;
	uint32_t total_props = 0;
	uint32_t i, j, off;
	int error = 0;
	bool test_only;

	if (file == NULL || r == NULL)
		return (EINVAL);
	if ((file->client_caps & (1u << DRM_CLIENT_CAP_ATOMIC)) == 0)
		return (EINVAL);
	if (r->count_objs == 0 || r->count_objs > DRM_ATOMIC_MAX_OBJS)
		return (EINVAL);

	objs = malloc((size_t)r->count_objs * sizeof(uint32_t), M_KMS,
	    M_WAITOK);
	counts = malloc((size_t)r->count_objs * sizeof(uint32_t), M_KMS,
	    M_WAITOK);
	error = copyin((const void *)(uintptr_t)r->objs_ptr, objs,
	    r->count_objs * sizeof(uint32_t));
	if (error != 0)
		goto out;
	error = copyin((const void *)(uintptr_t)r->count_props_ptr, counts,
	    r->count_objs * sizeof(uint32_t));
	if (error != 0)
		goto out;

	/* Sum + overflow-checked total prop count (security rule 4). */
	for (i = 0; i < r->count_objs; i++) {
		if (counts[i] > DRM_ATOMIC_MAX_PROPS_PER_OBJ) {
			error = EINVAL;
			goto out;
		}
		if (total_props + counts[i] < total_props ||
		    total_props + counts[i] > DRM_ATOMIC_MAX_TOTAL_PROPS) {
			error = EINVAL;
			goto out;
		}
		total_props += counts[i];
	}

	if (total_props > 0) {
		props = malloc((size_t)total_props * sizeof(uint32_t),
		    M_KMS, M_WAITOK);
		values = malloc((size_t)total_props * sizeof(uint64_t),
		    M_KMS, M_WAITOK);
		error = copyin((const void *)(uintptr_t)r->props_ptr,
		    props, total_props * sizeof(uint32_t));
		if (error != 0)
			goto out;
		error = copyin((const void *)(uintptr_t)r->prop_values_ptr,
		    values, total_props * sizeof(uint64_t));
		if (error != 0)
			goto out;
	}

	/*
	 * Resolve every (object, property) pair before mutating
	 * anything.  Refs are released in the cleanup pass whether or
	 * not the commit succeeds, so a failed validation leaves zero
	 * state behind.
	 */
	resolved = malloc((size_t)r->count_objs *
	    sizeof(struct drm_mode_object *), M_KMS, M_WAITOK | M_ZERO);
	prop_resolved = malloc((size_t)total_props *
	    sizeof(struct drm_property *), M_KMS, M_WAITOK | M_ZERO);

	off = 0;
	for (i = 0; i < r->count_objs; i++) {
		resolved[i] = kms_mode_object_find(file->dev,
		    objs[i], DRM_MODE_OBJECT_ANY);
		if (resolved[i] == NULL) {
			error = ENOENT;
			goto out;
		}
		for (j = 0; j < counts[i]; j++) {
			struct drm_mode_object *pobj;

			pobj = kms_mode_object_find(file->dev,
			    props[off + j], DRM_MODE_OBJECT_PROPERTY);
			if (pobj == NULL) {
				error = ENOENT;
				goto out;
			}
			prop_resolved[off + j] = __containerof(pobj,
			    struct drm_property, base);
			if (prop_resolved[off + j]->flags &
			    KMS_PROP_IMMUTABLE) {
				error = EINVAL;
				goto out;
			}
		}
		off += counts[i];
	}

	test_only = (r->flags & DRM_MODE_ATOMIC_TEST_ONLY) != 0;
	if (!test_only) {
		sx_xlock(&mc->mutex);
		off = 0;
		for (i = 0; i < r->count_objs; i++) {
			for (j = 0; j < counts[i]; j++) {
				(void)kms_object_property_set_value(
				    resolved[i], prop_resolved[off + j],
				    values[off + j]);
			}
			off += counts[i];
		}
		sx_xunlock(&mc->mutex);
	}

out:
	if (resolved != NULL) {
		for (i = 0; i < r->count_objs; i++)
			if (resolved[i] != NULL)
				kms_mode_object_put(resolved[i]);
		free(resolved, M_KMS);
	}
	if (prop_resolved != NULL) {
		for (i = 0; i < total_props; i++)
			if (prop_resolved[i] != NULL)
				kms_mode_object_put(
				    &prop_resolved[i]->base);
		free(prop_resolved, M_KMS);
	}
	free(objs, M_KMS);
	free(counts, M_KMS);
	free(props, M_KMS);
	free(values, M_KMS);
	return (error);
}
