/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#include <sys/param.h>
#include <sys/systm.h>

#include <sys/sx.h>

#include <kms/drm_connector.h>
#include <kms/drm_device.h>
#include <kms/drm_encoder.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_mode_object.h>
#include <kms/drm_modes.h>
#include <kms/drm_property.h>

#include "kms_internal.h"

/*
 * Per-(connector_type, dev) counter for connector_type_id.  Two
 * HDMI connectors get type_ids 1 and 2 inside the same device.
 * Linux maintains this; userspace observes it in xrandr output.
 */
static uint32_t
drm_connector_next_type_id(struct drm_device *dev, uint32_t type)
{
	struct drm_mode_object *obj;
	struct drm_connector *c;
	uint32_t max_id;

	max_id = 0;
	TAILQ_FOREACH(obj, &dev->mode_config.connectors, link) {
		c = __containerof(obj, struct drm_connector, base);
		if (c->connector_type == type &&
		    c->connector_type_id > max_id)
			max_id = c->connector_type_id;
	}
	return (max_id + 1);
}

int
kms_connector_init(struct drm_device *dev, struct drm_connector *connector,
    const struct drm_connector_funcs *funcs, uint32_t connector_type)
{
	int error;

	if (dev == NULL || connector == NULL)
		return (EINVAL);

	connector->dev = dev;
	connector->funcs = funcs;
	connector->connector_type = connector_type;
	/*
	 * Index is the connector's slot in drm_atomic_state->
	 * connector_states.  Assigned in registration order from the
	 * pre-increment count snapshot — the lock around the
	 * connector_type_id scan + register_locked below covers this
	 * read too, so the snapshot is stable.
	 */
	connector->index = dev->mode_config.num_connector;
	connector->status = connector_status_unknown;
	connector->mm_width = 0;
	connector->mm_height = 0;
	connector->subpixel_order = 0;
	connector->encoder_count = 0;
	connector->encoder = NULL;
	TAILQ_INIT(&connector->modes);
	connector->mode_count = 0;

	/*
	 * Hold mode_config.mutex across both the type_id scan AND the
	 * register call so a second concurrent connector_init for the
	 * same connector_type can't compute the same type_id.  The
	 * _locked variant of register skips the inner lock acquisition.
	 */
	sx_xlock(&dev->mode_config.mutex);
	connector->connector_type_id =
	    drm_connector_next_type_id(dev, connector_type);
	error = kms_mode_object_register_locked(dev, &connector->base,
	    DRM_MODE_OBJECT_CONNECTOR);
	sx_xunlock(&dev->mode_config.mutex);
	if (error != 0)
		return (error);
	if (dev->mode_config.prop_connector_crtc_id != NULL)
		kms_object_attach_property(&connector->base,
		    dev->mode_config.prop_connector_crtc_id, 0);
	return (0);
}

void
kms_connector_cleanup(struct drm_connector *connector)
{
	if (connector == NULL || connector->dev == NULL)
		return;
	kms_connector_modes_clear(connector);
	kms_mode_object_unregister(connector->dev, &connector->base);
	if (connector->funcs != NULL && connector->funcs->destroy != NULL)
		connector->funcs->destroy(connector);
}

int
kms_connector_attach_encoder(struct drm_connector *connector,
    struct drm_encoder *encoder)
{
	uint8_t i;

	if (connector == NULL || encoder == NULL ||
	    encoder->base.id == 0)
		return (EINVAL);

	for (i = 0; i < connector->encoder_count; i++) {
		if (connector->encoder_ids[i] == encoder->base.id)
			return (0);	/* idempotent */
	}
	if (connector->encoder_count >= DRM_CONNECTOR_MAX_ENCODER)
		return (ENOSPC);
	connector->encoder_ids[connector->encoder_count++] =
	    encoder->base.id;
	return (0);
}

void
kms_connector_add_mode(struct drm_connector *connector,
    struct drm_display_mode *mode)
{
	if (connector == NULL || mode == NULL)
		return;
	kms_mode_set_name(mode);
	sx_xlock(&connector->dev->mode_config.mutex);
	TAILQ_INSERT_TAIL(&connector->modes, mode, link);
	connector->mode_count++;
	sx_xunlock(&connector->dev->mode_config.mutex);
}

void
kms_connector_modes_clear(struct drm_connector *connector)
{
	struct drm_display_mode *m;

	if (connector == NULL)
		return;
	sx_xlock(&connector->dev->mode_config.mutex);
	while ((m = TAILQ_FIRST(&connector->modes)) != NULL) {
		TAILQ_REMOVE(&connector->modes, m, link);
		kms_mode_destroy(m);
	}
	connector->mode_count = 0;
	sx_xunlock(&connector->dev->mode_config.mutex);
}
