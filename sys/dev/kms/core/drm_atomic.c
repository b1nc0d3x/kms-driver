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

#include <kms/drm_atomic.h>
#include <kms/drm_connector.h>
#include <kms/drm_crtc.h>
#include <kms/drm_device.h>
#include <kms/drm_file.h>
#include <kms/drm_framebuffer.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_mode_object.h>
#include <kms/drm_modes.h>
#include <kms/drm_plane.h>
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

static void kms_atomic_state_release_refs(struct drm_atomic_state *state);

/* --- atomic_state lifecycle --- */

struct drm_atomic_state *
kms_atomic_state_alloc(struct drm_device *dev)
{
	struct drm_mode_config *mc = &dev->mode_config;
	struct drm_atomic_state *state;

	state = malloc(sizeof(*state), M_KMS, M_WAITOK | M_ZERO);
	state->dev = dev;
	state->num_crtc = mc->num_crtc;
	state->num_plane = mc->num_plane;
	state->num_connector = mc->num_connector;
	if (state->num_crtc > 0)
		state->crtc_states = malloc(state->num_crtc *
		    sizeof(*state->crtc_states), M_KMS, M_WAITOK | M_ZERO);
	if (state->num_plane > 0)
		state->plane_states = malloc(state->num_plane *
		    sizeof(*state->plane_states), M_KMS, M_WAITOK | M_ZERO);
	if (state->num_connector > 0)
		state->connector_states = malloc(state->num_connector *
		    sizeof(*state->connector_states), M_KMS,
		    M_WAITOK | M_ZERO);
	return (state);
}

void
kms_atomic_state_free(struct drm_atomic_state *state)
{
	uint32_t i;

	if (state == NULL)
		return;
	kms_atomic_state_release_refs(state);
	if (state->crtc_states != NULL) {
		for (i = 0; i < state->num_crtc; i++)
			free(state->crtc_states[i], M_KMS);
		free(state->crtc_states, M_KMS);
	}
	if (state->plane_states != NULL) {
		for (i = 0; i < state->num_plane; i++)
			free(state->plane_states[i], M_KMS);
		free(state->plane_states, M_KMS);
	}
	if (state->connector_states != NULL) {
		for (i = 0; i < state->num_connector; i++)
			free(state->connector_states[i], M_KMS);
		free(state->connector_states, M_KMS);
	}
	free(state, M_KMS);
}

/*
 * Lazy per-object state accessors.  index maps the object to the slot
 * in the parent state's array — set at object init time and stable
 * for the object's lifetime.  Returns NULL only on bounds-mismatch
 * (object belongs to a different device than the state).
 */
struct drm_crtc_state *
kms_atomic_get_crtc_state(struct drm_atomic_state *state,
    struct drm_crtc *crtc)
{
	if (crtc == NULL || crtc->index >= state->num_crtc)
		return (NULL);
	if (state->crtc_states[crtc->index] == NULL) {
		struct drm_crtc_state *cs;

		cs = malloc(sizeof(*cs), M_KMS, M_WAITOK | M_ZERO);
		cs->crtc = crtc;
		state->crtc_states[crtc->index] = cs;
	}
	return (state->crtc_states[crtc->index]);
}

struct drm_plane_state *
kms_atomic_get_plane_state(struct drm_atomic_state *state,
    struct drm_plane *plane)
{
	if (plane == NULL || plane->index >= state->num_plane)
		return (NULL);
	if (state->plane_states[plane->index] == NULL) {
		struct drm_plane_state *ps;

		ps = malloc(sizeof(*ps), M_KMS, M_WAITOK | M_ZERO);
		ps->plane = plane;
		state->plane_states[plane->index] = ps;
	}
	return (state->plane_states[plane->index]);
}

struct drm_connector_state *
kms_atomic_get_connector_state(struct drm_atomic_state *state,
    struct drm_connector *connector)
{
	if (connector == NULL || connector->index >= state->num_connector)
		return (NULL);
	if (state->connector_states[connector->index] == NULL) {
		struct drm_connector_state *cs;

		cs = malloc(sizeof(*cs), M_KMS, M_WAITOK | M_ZERO);
		cs->connector = connector;
		state->connector_states[connector->index] = cs;
	}
	return (state->connector_states[connector->index]);
}

/* --- property → state-field resolver --- */

/*
 * Look an ID-typed property value up against the mode-object registry.
 * Returns a new ref to the object (caller must release) or NULL on a
 * mismatch.  Value of 0 is the well-known "no object" sentinel and
 * returns NULL without setting *errp.
 */
static struct drm_mode_object *
kms_atomic_resolve_id(struct drm_device *dev, uint64_t value,
    uint32_t expected_type, int *errp)
{
	struct drm_mode_object *obj;

	*errp = 0;
	if (value == 0)
		return (NULL);
	if (value > UINT32_MAX) {
		*errp = EINVAL;
		return (NULL);
	}
	obj = kms_mode_object_find(dev, (uint32_t)value, expected_type);
	if (obj == NULL) {
		*errp = ENOENT;
		return (NULL);
	}
	return (obj);
}

/*
 * Decode a MODE_ID blob value into crtc_state->mode.  value=0 is
 * "disable" — clears the mode and flags the CRTC as disabled.
 * Otherwise the value is a blob id whose payload is one drm_mode_modeinfo.
 */
static int
kms_atomic_apply_mode_id(struct drm_atomic_state *state,
    struct drm_crtc_state *cs, uint64_t value)
{
	struct drm_property_blob *blob;

	if (value == 0) {
		memset(&cs->mode, 0, sizeof(cs->mode));
		cs->enable = false;
		cs->mode_changed = true;
		return (0);
	}
	if (value > UINT32_MAX)
		return (EINVAL);
	blob = kms_property_blob_find(state->dev, (uint32_t)value);
	if (blob == NULL)
		return (ENOENT);
	if (blob->length != sizeof(struct drm_mode_modeinfo)) {
		kms_mode_object_put(&blob->base);
		return (EINVAL);
	}
	kms_modeinfo_to_display_mode(
	    (const struct drm_mode_modeinfo *)blob->data, &cs->mode);
	cs->enable = true;
	cs->mode_changed = true;
	kms_mode_object_put(&blob->base);
	return (0);
}

int
kms_atomic_state_set_property(struct drm_atomic_state *state,
    struct drm_mode_object *obj, struct drm_property *prop, uint64_t value)
{
	struct drm_mode_config *mc;
	int error;

	if (state == NULL || obj == NULL || prop == NULL)
		return (EINVAL);
	mc = &state->dev->mode_config;

	switch (obj->type) {
	case DRM_MODE_OBJECT_CRTC: {
		struct drm_crtc *crtc =
		    __containerof(obj, struct drm_crtc, base);
		struct drm_crtc_state *cs;

		cs = kms_atomic_get_crtc_state(state, crtc);
		if (cs == NULL)
			return (EINVAL);
		if (prop == mc->prop_crtc_active) {
			bool active = (value != 0);

			/*
			 * ACTIVE flips the dpms bit; it does NOT itself
			 * count as a mode change.  mode_changed is set
			 * only by MODE_ID writes below — keeps drivers'
			 * check hooks from rejecting an enable/disable
			 * transition that doesn't carry a fresh mode.
			 */
			cs->active = active;
			cs->enable = active;
			return (0);
		}
		if (prop == mc->prop_crtc_mode_id)
			return (kms_atomic_apply_mode_id(state, cs, value));
		break;
	}
	case DRM_MODE_OBJECT_PLANE: {
		struct drm_plane *plane =
		    __containerof(obj, struct drm_plane, base);
		struct drm_plane_state *ps;
		struct drm_mode_object *o;

		ps = kms_atomic_get_plane_state(state, plane);
		if (ps == NULL)
			return (EINVAL);
		if (prop == mc->prop_plane_fb_id) {
			o = kms_atomic_resolve_id(state->dev, value,
			    DRM_MODE_OBJECT_FB, &error);
			if (error != 0)
				return (error);
			ps->fb = (o != NULL) ?
			    __containerof(o, struct drm_framebuffer, base) :
			    NULL;
			return (0);
		}
		if (prop == mc->prop_plane_crtc_id) {
			o = kms_atomic_resolve_id(state->dev, value,
			    DRM_MODE_OBJECT_CRTC, &error);
			if (error != 0)
				return (error);
			ps->crtc = (o != NULL) ?
			    __containerof(o, struct drm_crtc, base) : NULL;
			if (ps->crtc != NULL) {
				struct drm_crtc_state *cs =
				    kms_atomic_get_crtc_state(state, ps->crtc);
				if (cs != NULL) {
					cs->plane_mask |= (1u << plane->index);
					cs->planes_changed = true;
				}
			}
			return (0);
		}
		if (prop == mc->prop_plane_crtc_x) {
			ps->crtc_x = (int32_t)value;
			return (0);
		}
		if (prop == mc->prop_plane_crtc_y) {
			ps->crtc_y = (int32_t)value;
			return (0);
		}
		if (prop == mc->prop_plane_crtc_w) {
			ps->crtc_w = (uint32_t)value;
			return (0);
		}
		if (prop == mc->prop_plane_crtc_h) {
			ps->crtc_h = (uint32_t)value;
			return (0);
		}
		if (prop == mc->prop_plane_src_x) {
			ps->src_x = (uint32_t)value;
			return (0);
		}
		if (prop == mc->prop_plane_src_y) {
			ps->src_y = (uint32_t)value;
			return (0);
		}
		if (prop == mc->prop_plane_src_w) {
			ps->src_w = (uint32_t)value;
			return (0);
		}
		if (prop == mc->prop_plane_src_h) {
			ps->src_h = (uint32_t)value;
			return (0);
		}
		break;
	}
	case DRM_MODE_OBJECT_CONNECTOR: {
		struct drm_connector *connector =
		    __containerof(obj, struct drm_connector, base);
		struct drm_connector_state *cs;
		struct drm_mode_object *o;

		cs = kms_atomic_get_connector_state(state, connector);
		if (cs == NULL)
			return (EINVAL);
		if (prop == mc->prop_connector_crtc_id) {
			o = kms_atomic_resolve_id(state->dev, value,
			    DRM_MODE_OBJECT_CRTC, &error);
			if (error != 0)
				return (error);
			cs->crtc = (o != NULL) ?
			    __containerof(o, struct drm_crtc, base) : NULL;
			if (cs->crtc != NULL) {
				struct drm_crtc_state *cstate =
				    kms_atomic_get_crtc_state(state, cs->crtc);
				if (cstate != NULL) {
					cstate->connector_mask |=
					    (1u << connector->index);
					cstate->connectors_changed = true;
				}
			}
			return (0);
		}
		break;
	}
	default:
		break;
	}

	/*
	 * Not a well-known atomic property — record on the legacy per-
	 * object property table so OBJ_GETPROPERTIES round-trips.  Driver-
	 * specific properties (gamma LUT IDs, vendor controls) flow through
	 * here.
	 */
	(void)kms_object_property_set_value(obj, prop, value);
	return (0);
}

/*
 * Walk an in-flight atomic_state and release every object ref the
 * resolver took.  Called from kms_atomic_state_free.
 */
static void
kms_atomic_state_release_refs(struct drm_atomic_state *state)
{
	uint32_t i;

	if (state->plane_states != NULL) {
		for (i = 0; i < state->num_plane; i++) {
			struct drm_plane_state *ps = state->plane_states[i];

			if (ps == NULL)
				continue;
			if (ps->fb != NULL) {
				kms_mode_object_put(&ps->fb->base);
				ps->fb = NULL;
			}
			if (ps->crtc != NULL) {
				kms_mode_object_put(&ps->crtc->base);
				ps->crtc = NULL;
			}
		}
	}
	if (state->connector_states != NULL) {
		for (i = 0; i < state->num_connector; i++) {
			struct drm_connector_state *cs =
			    state->connector_states[i];

			if (cs == NULL || cs->crtc == NULL)
				continue;
			kms_mode_object_put(&cs->crtc->base);
			cs->crtc = NULL;
		}
	}
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

	/*
	 * Atomic dispatch.  When the driver has installed both hooks the
	 * framework builds a drm_atomic_state from the batch and walks
	 * the (obj, prop, value) tuples to populate per-object touched
	 * slots, then hands off to atomic_check / atomic_commit.  Until
	 * the property→state resolver lands (Phase 8 step 2) the state
	 * carries only the "touched" markers — drivers that need
	 * specific property values still read them out of the per-object
	 * property table during their commit walk.  Drivers that haven't
	 * wired the hooks (or wired only one) fall back to the legacy
	 * property-table-write path so atomic-unaware userspace keeps
	 * working unchanged.
	 */
	if (mc->funcs != NULL && mc->funcs->atomic_check != NULL &&
	    mc->funcs->atomic_commit != NULL) {
		struct drm_atomic_state *astate;
		bool nonblock;

		astate = kms_atomic_state_alloc(file->dev);
		astate->flags = r->flags;

		/*
		 * Resolve every (object, property, value) tuple into the
		 * appropriate per-object state field.  Stop at the first
		 * error so the driver never sees a half-populated state.
		 */
		off = 0;
		for (i = 0; i < r->count_objs && error == 0; i++) {
			for (j = 0; j < counts[i] && error == 0; j++) {
				error = kms_atomic_state_set_property(
				    astate, resolved[i],
				    prop_resolved[off + j],
				    values[off + j]);
			}
			off += counts[i];
		}

		if (error == 0)
			error = mc->funcs->atomic_check(file->dev, astate);
		if (error == 0 && !test_only) {
			nonblock = (r->flags & DRM_MODE_ATOMIC_NONBLOCK) != 0;
			error = mc->funcs->atomic_commit(file->dev, astate,
			    nonblock);
		}

		/*
		 * If PAGE_FLIP_EVENT was set, arm pending_flip_file on every
		 * active CRTC the batch touched.  The framework owns the file
		 * + user_data handoff so drivers don't have to: the next
		 * vblank IRQ runs kms_vblank_handler which dispatches
		 * DRM_EVENT_FLIP_COMPLETE on the stashed file.  Without this,
		 * Wayland compositors (kwin_wayland, weston) wedge on the
		 * first frame waiting for an event that never arrives —
		 * atomic_commit succeeds but the event side is silent.
		 */
		if (error == 0 && !test_only &&
		    (r->flags & DRM_MODE_PAGE_FLIP_EVENT) != 0) {
			sx_xlock(&mc->mutex);
			for (uint32_t k = 0; k < astate->num_crtc; k++) {
				struct drm_crtc_state *cs =
				    astate->crtc_states[k];

				if (cs == NULL || cs->crtc == NULL ||
				    !cs->active)
					continue;
				cs->crtc->pending_flip_file = file;
				cs->crtc->pending_flip_user_data =
				    r->user_data;
			}
			sx_xunlock(&mc->mutex);
		}

		/*
		 * The driver may queue the state for deferred completion
		 * by stashing the pointer and returning -EAGAIN today; that
		 * lands with the state-swap work in Phase 8b.  For now the
		 * framework always owns the lifetime — driver must not stash.
		 */
		kms_atomic_state_free(astate);
	} else if (!test_only) {
		/* Legacy fallback: write straight to the property table. */
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
