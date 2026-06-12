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

struct drm_atomic_state;
struct drm_device;

/*
 * Per-device driver hooks.  All optional.
 *
 *   atomic_check   — validate a proposed atomic_state.  Returns 0 if
 *                    every requested value is implementable, errno
 *                    otherwise.  Must not mutate hardware.  Required
 *                    for MODE_ATOMIC TEST_ONLY commits to mean
 *                    anything beyond resolver-level sanity.
 *   atomic_commit  — program the proposed state into the silicon.
 *                    May be sync or deferred; `nonblock` is a hint.
 *                    On success the framework swaps the new per-object
 *                    state pointers in (Phase 8b).
 *
 * When either is NULL kms_ioctl_mode_atomic falls back to the legacy
 * property-table-write path, so atomic-unaware userspace (one property
 * at a time via OBJ_SETPROPERTY) still works against driver builds
 * that have not yet wired the atomic surface.
 */
struct drm_mode_config_funcs {
	int (*atomic_check)(struct drm_device *dev,
	    struct drm_atomic_state *state);
	int (*atomic_commit)(struct drm_device *dev,
	    struct drm_atomic_state *state, bool nonblock);
};

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

	/*
	 * Standard properties registered at mode_config_init time and
	 * attached to objects on their init paths.  Drivers may add more
	 * properties via drm_property_create_* and attach via
	 * kms_object_attach_property.  Phase 8 surface is the minimum
	 * an atomic-aware userspace (modesetting DDX, xf86-video-amdgpu-
	 * style ports) actually queries.
	 */
	struct drm_property		*prop_crtc_active;
	struct drm_property		*prop_crtc_mode_id;
	struct drm_property		*prop_plane_type;
	struct drm_property		*prop_plane_fb_id;
	struct drm_property		*prop_plane_crtc_id;
	struct drm_property		*prop_plane_crtc_x;
	struct drm_property		*prop_plane_crtc_y;
	struct drm_property		*prop_plane_crtc_w;
	struct drm_property		*prop_plane_crtc_h;
	struct drm_property		*prop_plane_src_x;
	struct drm_property		*prop_plane_src_y;
	struct drm_property		*prop_plane_src_w;
	struct drm_property		*prop_plane_src_h;
	struct drm_property		*prop_connector_crtc_id;

	/*
	 * Driver atomic hooks.  See struct drm_mode_config_funcs above.
	 * Initialized to (NULL, NULL) by kms_mode_config_init; drivers
	 * install hooks before kms_dev_register, after which a NULL
	 * pair is treated as "use legacy property-write fallback."
	 */
	const struct drm_mode_config_funcs *funcs;
};

struct drm_device;

void	kms_mode_config_init(struct drm_mode_config *mc);
void	kms_mode_config_cleanup(struct drm_mode_config *mc);
void	kms_mode_config_standard_properties_init(struct drm_device *dev);

#endif /* _KMS_DRM_MODE_CONFIG_H_ */
