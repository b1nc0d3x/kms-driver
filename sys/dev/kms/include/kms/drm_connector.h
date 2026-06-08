/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#ifndef _KMS_DRM_CONNECTOR_H_
#define _KMS_DRM_CONNECTOR_H_

#include <sys/types.h>

#include <kms/drm_mode_object.h>

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
};

struct drm_connector {
	struct drm_mode_object		 base;
	struct drm_device		*dev;
	const struct drm_connector_funcs *funcs;
	uint32_t			 connector_type;
	uint32_t			 connector_type_id;
	enum drm_connector_status	 status;
	uint32_t			 mm_width;
	uint32_t			 mm_height;
	uint32_t			 subpixel_order;
	uint8_t				 encoder_count;
	uint32_t			 encoder_ids[DRM_CONNECTOR_MAX_ENCODER];
	struct drm_encoder		*encoder;	/* current binding */
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

#endif /* _KMS_DRM_CONNECTOR_H_ */
