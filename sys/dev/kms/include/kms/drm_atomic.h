/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Atomic modeset state objects.  An MODE_ATOMIC ioctl call builds one
 * struct drm_atomic_state that aggregates the proposed per-object
 * state across every CRTC / plane / connector touched by the batch.
 * The driver receives that object in two phases:
 *
 *   atomic_check  (drm_device *, drm_atomic_state *)
 *       Inspect the proposed state.  Return 0 if every value is
 *       implementable on real hardware (mode in the driver's pool,
 *       plane fb has GEM backing, connector→crtc routing matches
 *       the encoder topology, …).  Return an errno otherwise.  Must
 *       not mutate hardware.  Required for TEST_ONLY commits.
 *
 *   atomic_commit (drm_device *, drm_atomic_state *, bool nonblock)
 *       Program the requested state into the silicon.  When 0 is
 *       returned the framework takes ownership of swapping the new
 *       per-object state pointers in (the swap itself lands in a
 *       follow-up — Phase 8b).  `nonblock` lets the driver defer to
 *       a worker / vblank IRQ; v1 implementations may treat it as a
 *       hint and run synchronously.
 *
 * Both hooks are optional.  If either is NULL the framework falls
 * back to the legacy property-table-write path — atomic-unaware
 * userspace (which writes one property at a time via
 * MODE_OBJ_SETPROPERTY) still works.
 */

#ifndef _KMS_DRM_ATOMIC_H_
#define _KMS_DRM_ATOMIC_H_

#include <sys/types.h>

#include <kms/drm_modes.h>

struct drm_atomic_state;
struct drm_connector;
struct drm_crtc;
struct drm_device;
struct drm_event;
struct drm_framebuffer;
struct drm_plane;

/*
 * Per-CRTC proposed state.  `mode` is the requested display timing;
 * `enable` is the legacy bit ("CRTC is logically powered") while
 * `active` is the dpms / active-state bit (CRTC is actually scanning).
 * The Linux drm distinction lets a compositor stage a config (enable=1
 * active=0) without lighting the panel.  `mode_changed`, `planes_changed`,
 * and `connectors_changed` are derived flags the framework sets so
 * drivers can skip expensive PLL / WIN0 / encoder reprogramming when
 * only an attribute below their granularity changed.
 *
 * `connector_mask` and `plane_mask` are bit sets indexed by the
 * connector / plane's per-device numeric index (assigned at init).
 * They let the driver iterate the connectors / planes routed to this
 * CRTC without walking the full atomic_state.
 *
 * `event` is non-NULL when userspace passed DRM_MODE_PAGE_FLIP_EVENT
 * in the atomic flags — the driver fires it on commit completion.
 */
struct drm_crtc_state {
	struct drm_crtc			*crtc;
	bool				 enable;
	bool				 active;
	bool				 mode_changed;
	bool				 planes_changed;
	bool				 connectors_changed;
	struct drm_display_mode		 mode;
	uint32_t			 connector_mask;
	uint32_t			 plane_mask;
	struct drm_event		*event;
};

/*
 * Per-plane proposed state.  `crtc` is the CRTC this plane is routed to
 * (NULL to disable the plane).  `fb` is the framebuffer to scan out;
 * NULL implies the plane is disabled.  Source rectangle is in 16.16
 * fixed-point (Linux convention so subpixel positioning works); dest
 * rectangle is in integer pixels.
 */
struct drm_plane_state {
	struct drm_plane		*plane;
	struct drm_crtc			*crtc;
	struct drm_framebuffer		*fb;
	uint32_t			 src_x;
	uint32_t			 src_y;
	uint32_t			 src_w;
	uint32_t			 src_h;
	int32_t				 crtc_x;
	int32_t				 crtc_y;
	uint32_t			 crtc_w;
	uint32_t			 crtc_h;
	uint16_t			 alpha;
};

/*
 * Per-connector proposed state.  `crtc` is the CRTC driving this
 * connector (NULL to detach).  `max_bpc` lets userspace request a
 * specific output bit depth (8 / 10 / 12); drivers may clip down.
 */
struct drm_connector_state {
	struct drm_connector		*connector;
	struct drm_crtc			*crtc;
	uint8_t				 max_bpc;
};

/*
 * The full atomic transaction.  Arrays are sized at allocation time
 * to the number of objects in the parent drm_device (cheap upper
 * bound — typically < 16 each).  An entry is "touched" by the batch
 * iff the corresponding slot is non-NULL; the rest stay NULL and the
 * driver treats them as "no change."
 *
 * `dev` and `acquire_ctx` are bookkeeping the framework owns; drivers
 * read but do not mutate them.  `flags` carries the DRM_MODE_ATOMIC_*
 * bits from userspace (TEST_ONLY / NONBLOCK / ALLOW_MODESET / PAGE_FLIP_
 * EVENT) so drivers can branch on them.
 */
struct drm_atomic_state {
	struct drm_device		*dev;
	uint32_t			 flags;

	uint32_t			 num_crtc;
	uint32_t			 num_plane;
	uint32_t			 num_connector;

	struct drm_crtc_state		**crtc_states;
	struct drm_plane_state		**plane_states;
	struct drm_connector_state	**connector_states;
};

/*
 * Lifecycle.  Allocate, populate via the framework's
 * property-translation helpers, hand to the driver, then free.  Free
 * walks every non-NULL slot and releases per-object state plus the
 * top-level state.
 */
struct drm_atomic_state	*kms_atomic_state_alloc(struct drm_device *dev);
void			 kms_atomic_state_free(struct drm_atomic_state *state);

/*
 * Per-object lazy state accessors.  Return the existing entry in
 * `state` for the given object, or allocate + zero a new entry on
 * first touch.  Returns NULL only on OOM.  Used by the property
 * resolver (Phase 8 step 2) and directly by tests.
 */
struct drm_crtc_state		*kms_atomic_get_crtc_state(
				    struct drm_atomic_state *state,
				    struct drm_crtc *crtc);
struct drm_plane_state		*kms_atomic_get_plane_state(
				    struct drm_atomic_state *state,
				    struct drm_plane *plane);
struct drm_connector_state	*kms_atomic_get_connector_state(
				    struct drm_atomic_state *state,
				    struct drm_connector *connector);

/*
 * Property → state-field resolver.  Looks `prop` up in the parent
 * device's well-known-property table, identifies which field on the
 * appropriate per-object state slot it maps to, allocates the slot if
 * needed, and stores the value.  For ID-typed properties (fb_id,
 * crtc_id, mode_id-blob) the resolver dereferences the ID against
 * the mode-object registry and stores the resulting pointer; the ref
 * is held until kms_atomic_state_free runs.
 *
 * Returns 0 on a known prop (regardless of whether the value
 * round-trips to a real object), ENOENT when a referenced ID doesn't
 * resolve, EINVAL when the (object-type, property) pair is invalid,
 * and 0 silently when the property isn't a well-known atomic prop —
 * the framework records it on the per-object property table as a
 * fall-back so OBJ_GETPROPERTIES still round-trips.
 */
struct drm_mode_object;
struct drm_property;

int	kms_atomic_state_set_property(struct drm_atomic_state *state,
	    struct drm_mode_object *obj, struct drm_property *prop,
	    uint64_t value);

/*
 * Fan out per-plane validation.  Walks every non-NULL slot in
 * state->plane_states and calls that plane's funcs->atomic_check hook
 * (if set).  Returns the first non-zero errno; on success every touched
 * plane has been accepted by its driver.  Drivers can call this from
 * mode_config.funcs->atomic_check to keep per-plane rules where they
 * belong instead of switch-casing on plane->type in a monolithic hook.
 */
int	kms_atomic_check_planes(struct drm_atomic_state *state);

#endif /* _KMS_DRM_ATOMIC_H_ */
