/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/refcount.h>
#include <sys/sx.h>

#include <kms/drm_device.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_mode_object.h>

#include "kms_internal.h"

/*
 * Pick the per-type list (if any) for an object kind.  Returns NULL
 * for types that aren't yet tracked separately (MODE/PROPERTY/BLOB
 * land in later phases).
 */
static struct drm_mode_object_list *
drm_mode_object_list_for_type(struct drm_mode_config *mc, uint32_t type)
{
	switch (type) {
	case DRM_MODE_OBJECT_CRTC:
		return (&mc->crtcs);
	case DRM_MODE_OBJECT_CONNECTOR:
		return (&mc->connectors);
	case DRM_MODE_OBJECT_ENCODER:
		return (&mc->encoders);
	case DRM_MODE_OBJECT_FB:
		return (&mc->fbs);
	}
	return (NULL);
}

static uint32_t *
drm_mode_object_count_for_type(struct drm_mode_config *mc, uint32_t type)
{
	switch (type) {
	case DRM_MODE_OBJECT_CRTC:
		return (&mc->num_crtc);
	case DRM_MODE_OBJECT_CONNECTOR:
		return (&mc->num_connector);
	case DRM_MODE_OBJECT_ENCODER:
		return (&mc->num_encoder);
	case DRM_MODE_OBJECT_FB:
		return (&mc->num_fb);
	}
	return (NULL);
}

int
kms_mode_object_register(struct drm_device *dev, struct drm_mode_object *obj,
    uint32_t type)
{
	struct drm_mode_config *mc;
	struct drm_mode_object_list *list;
	uint32_t *count;

	if (dev == NULL || obj == NULL || type == DRM_MODE_OBJECT_ANY)
		return (EINVAL);

	mc = &dev->mode_config;
	obj->type = type;
	refcount_init(&obj->refs, 1);

	sx_xlock(&mc->mutex);
	/*
	 * Monotonic ID allocation.  0 is reserved as invalid; wrap is
	 * impossible in any realistic session (4G register events would
	 * exhaust the 32-bit space).  If we ever care, reuse via a
	 * free-list goes here.
	 */
	if (mc->next_object_id == UINT32_MAX) {
		sx_xunlock(&mc->mutex);
		return (ENOSPC);
	}
	obj->id = ++mc->next_object_id;

	TAILQ_INSERT_TAIL(&mc->objects, obj, reg);
	list = drm_mode_object_list_for_type(mc, type);
	count = drm_mode_object_count_for_type(mc, type);
	if (list != NULL) {
		TAILQ_INSERT_TAIL(list, obj, link);
		(*count)++;
	}
	sx_xunlock(&mc->mutex);
	return (0);
}

void
kms_mode_object_unregister(struct drm_device *dev, struct drm_mode_object *obj)
{
	struct drm_mode_config *mc;
	struct drm_mode_object_list *list;
	uint32_t *count;

	if (dev == NULL || obj == NULL || obj->id == 0)
		return;

	mc = &dev->mode_config;

	sx_xlock(&mc->mutex);
	TAILQ_REMOVE(&mc->objects, obj, reg);
	list = drm_mode_object_list_for_type(mc, obj->type);
	count = drm_mode_object_count_for_type(mc, obj->type);
	if (list != NULL) {
		TAILQ_REMOVE(list, obj, link);
		(*count)--;
	}
	obj->id = 0;
	sx_xunlock(&mc->mutex);

	kms_mode_object_put(obj);
}

struct drm_mode_object *
kms_mode_object_find(struct drm_device *dev, uint32_t id, uint32_t type)
{
	struct drm_mode_config *mc;
	struct drm_mode_object *obj;

	if (dev == NULL || id == 0)
		return (NULL);

	mc = &dev->mode_config;
	sx_slock(&mc->mutex);
	TAILQ_FOREACH(obj, &mc->objects, reg) {
		if (obj->id != id)
			continue;
		if (type != DRM_MODE_OBJECT_ANY && obj->type != type) {
			obj = NULL;
			break;
		}
		refcount_acquire(&obj->refs);
		break;
	}
	sx_sunlock(&mc->mutex);
	return (obj);
}

void
kms_mode_object_put(struct drm_mode_object *obj)
{
	if (obj == NULL)
		return;
	/*
	 * Final release is the caller's responsibility — the object is
	 * embedded in driver storage, so we have no allocation to free.
	 * refcount_release returning true is the signal that whoever
	 * owns the storage may reclaim it.
	 */
	(void)refcount_release(&obj->refs);
}
