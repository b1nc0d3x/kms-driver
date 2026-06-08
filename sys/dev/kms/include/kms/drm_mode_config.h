/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#ifndef _KMS_DRM_MODE_CONFIG_H_
#define _KMS_DRM_MODE_CONFIG_H_

#include <sys/types.h>
#include <sys/queue.h>
#include <sys/sx.h>

#include <kms/drm_mode_object.h>

/*
 * Root of a drm_device's KMS state.  One per drm_device, embedded.
 * mutex covers everything inside (object lists, counts, dimensions).
 * Held shared by readers (GETRESOURCES), exclusive by mutators
 * (object register/unregister, future SETCRTC etc.).
 */
struct drm_mode_config {
	struct sx			 mutex;

	/*
	 * Display surface dimensions advertised to userspace via
	 * GETRESOURCES.  Hardware driver overrides defaults before
	 * registering any connector.
	 */
	uint32_t			 min_width;
	uint32_t			 max_width;
	uint32_t			 min_height;
	uint32_t			 max_height;

	/* Per-type counts updated in lockstep with the lists below. */
	uint32_t			 num_crtc;
	uint32_t			 num_connector;
	uint32_t			 num_encoder;
	uint32_t			 num_fb;
	uint32_t			 num_plane;

	/* Monotonic ID counter; 0 reserved as "invalid id". */
	uint32_t			 next_object_id;

	/*
	 * Per-type lists for fast GETRESOURCES iteration.  objects is
	 * the master id->object list used by kms_mode_object_find.
	 */
	struct drm_mode_object_list	 crtcs;
	struct drm_mode_object_list	 connectors;
	struct drm_mode_object_list	 encoders;
	struct drm_mode_object_list	 fbs;
	struct drm_mode_object_list	 planes;
	struct drm_mode_object_list	 objects;
};

void	drm_mode_config_init(struct drm_mode_config *mc);
void	drm_mode_config_cleanup(struct drm_mode_config *mc);

#endif /* _KMS_DRM_MODE_CONFIG_H_ */
