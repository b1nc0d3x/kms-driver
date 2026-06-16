/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#ifndef _KMS_DRM_CONNECTOR_H_
#define _KMS_DRM_CONNECTOR_H_

#include <sys/types.h>

#include <kms/drm_mode_object.h>
#include <kms/drm_modes.h>

struct drm_device;
struct drm_connector;
struct drm_encoder;

/*
 * Maximum number of encoders a connector can advertise as candidates.
 * Linux's DRM_CONNECTOR_MAX_ENCODER is 3; bumping is a uapi-visible
 * choice via GETCONNECTOR's bounded array — keep parity.
 */
#define	DRM_CONNECTOR_MAX_ENCODER	3

enum drm_connector_status {
	connector_status_connected	= 1,
	connector_status_disconnected	= 2,
	connector_status_unknown	= 3,
};

struct drm_connector_funcs {
	void	(*destroy)(struct drm_connector *connector);
	/*
	 * Called from GETCONNECTOR (and any future "rescan") path when
	 * the framework wants the connector's mode list refreshed.  The
	 * hook builds mode entries with kms_mode_create() and adds them
	 * via kms_connector_add_mode().  Returns the number of modes
	 * added (>= 0) or a negative errno.  If NULL, the connector keeps
	 * whatever modes the driver populated at init time (the stub
	 * case).
	 */
	int	(*get_modes)(struct drm_connector *connector);
};

struct drm_connector {
	struct drm_mode_object		 base;
	struct drm_device		*dev;
	const struct drm_connector_funcs *funcs;
	uint32_t			 connector_type;
	uint32_t			 connector_type_id;
	uint32_t			 index;	/* assigned at init; indexes
					   the parent drm_device's connector
					   array.  Used by drm_atomic
					   state slot lookup. */
	enum drm_connector_status	 status;
	uint32_t			 mm_width;
	uint32_t			 mm_height;
	uint32_t			 subpixel_order;
	uint8_t				 encoder_count;
	uint32_t			 encoder_ids[DRM_CONNECTOR_MAX_ENCODER];
	struct drm_encoder		*encoder;	/* current binding */

	/*
	 * Available display modes.  Populated by the driver's get_modes
	 * hook (called from GETCONNECTOR) or by hand at init.  Storage is
	 * malloc'd via kms_mode_create and owned by the connector — freed
	 * in cleanup.  Protected by drm_device->mode_config.mutex.
	 */
	struct drm_display_mode_list	 modes;
	uint32_t			 mode_count;
};

int	kms_connector_init(struct drm_device *dev,
	    struct drm_connector *connector,
	    const struct drm_connector_funcs *funcs, uint32_t connector_type);
void	kms_connector_cleanup(struct drm_connector *connector);

/*
 * Add a candidate encoder to the connector's list.  Returns 0 on
 * success, ENOSPC if the array is already full.  Idempotent: adding
 * the same encoder twice is a no-op success.
 */
int	kms_connector_attach_encoder(struct drm_connector *connector,
	    struct drm_encoder *encoder);

/*
 * Driver-facing hotplug notification.  A hardware driver calls this
 * when its HPD-detection path observes a sink plug / unplug or any
 * other event that changes the connector's detected status.  The
 * framework updates connector->status atomically, then delivers a
 * DRM_EVENT_CONNECTOR_HOTPLUG to every drm_file currently open on
 * the device and broadcasts a devd-readable
 *   system=kms subsystem=cardN type=hotplug
 *   data="connector=ID status=N"
 * notification.
 *
 * Lock context: must be called from sleep-safe context (a callout or
 * worker scheduled from the actual HPD IRQ handler).  Takes
 * dev->dev_lock shared to walk the per-device file list, then each
 * file's event_mtx briefly to queue the per-fd event.  Do not call
 * from a hard-interrupt thread.
 */
void	kms_connector_hotplug(struct drm_connector *connector,
	    enum drm_connector_status new_status);

/*
 * Append a mode to the connector's mode list.  Connector takes
 * ownership of the storage — the mode is freed during
 * kms_connector_cleanup or kms_connector_modes_clear.
 * If the mode has no name, kms_mode_set_name is called to generate one.
 */
void	kms_connector_add_mode(struct drm_connector *connector,
	    struct drm_display_mode *mode);

/*
 * Free every mode currently on the connector's list.  Used by
 * cleanup and by get_modes implementations that fully replace
 * (rather than augment) the previous mode set on rescan.
 */
void	kms_connector_modes_clear(struct drm_connector *connector);

/*
 * Publish a new EDID byte string on the connector.  Allocates a new
 * property blob, points the connector's EDID property at it, and
 * destroys the previously-published blob (if any).  Pass (NULL, 0) to
 * clear the property on hotplug-disconnect.  Caller may discard the
 * source buffer after return; the framework copies it.
 */
int	kms_connector_update_edid(struct drm_connector *connector,
	    const void *data, size_t length);

#endif /* _KMS_DRM_CONNECTOR_H_ */
