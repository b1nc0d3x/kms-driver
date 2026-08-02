/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#ifndef _KMS_DRM_CRTC_H_
#define _KMS_DRM_CRTC_H_

#include <sys/types.h>
#include <sys/queue.h>

#include <kms/drm_mode_object.h>
#include <kms/drm_modes.h>

struct drm_device;
struct drm_crtc;
struct drm_framebuffer;
struct drm_plane;

struct drm_connector;
struct drm_display_mode;

/*
 * Argument bundle passed to set_config.  Mirrors the SETCRTC ioctl
 * payload after the framework has resolved ids to kernel pointers
 * and dropped the lookup refs: drivers see a single coherent view
 * of what userspace asked for.  fb is NULL on a "blank this CRTC"
 * request; connectors is NULL when count_connectors is zero.
 */
struct drm_mode_set {
	struct drm_crtc			*crtc;
	struct drm_framebuffer		*fb;
	const struct drm_display_mode	*mode;	/* NULL on blank */
	uint32_t			 x;
	uint32_t			 y;
	uint32_t			 count_connectors;
	struct drm_connector		**connectors;	/* count_connectors */
};

/*
 * Driver-supplied operations.  Phase 4 only required .destroy; Phase
 * 7 adds .set_config (SETCRTC) and .page_flip (PAGE_FLIP).  All hooks
 * may be NULL on the stub; the framework records the new state on
 * drm_crtc unconditionally so userspace GETCRTC sees the result even
 * without a driver-side implementation.  Phase 8 wires atomic_*.
 */
struct drm_file;

struct drm_crtc_funcs {
	void	(*destroy)(struct drm_crtc *crtc);
	int	(*set_config)(struct drm_mode_set *set);
	int	(*page_flip)(struct drm_crtc *crtc, struct drm_framebuffer *fb,
		    uint32_t flags, uint64_t user_data);
	/*
	 * MODE_CURSOR / MODE_CURSOR2 backing.  cursor_set switches the
	 * cursor BO; cursor_move repositions it.  handle == 0 (or
	 * width/height == 0) disables the cursor.  hot_x/hot_y describe
	 * the cursor bitmap's reference point so the driver can convert
	 * the userspace-coordinate (x, y) into a top-left position on
	 * subsequent moves.  Hooks may be NULL — the framework then
	 * returns ENOTTY to userspace which falls back to a software
	 * cursor composited into the primary plane (with the visible
	 * flicker that implies).
	 */
	int	(*cursor_set)(struct drm_crtc *crtc, struct drm_file *file,
		    uint32_t handle, uint32_t width, uint32_t height,
		    int32_t hot_x, int32_t hot_y);
	int	(*cursor_move)(struct drm_crtc *crtc, int32_t x, int32_t y);

	/*
	 * SETGAMMA: install per-channel 16-bit gamma correction ramps of
	 * `size` entries each.  If NULL, the framework returns 0 (no-op
	 * as before) so callers that don't need real gamma still see
	 * successful set — matches Linux behaviour for stub drivers.
	 * red / green / blue are kernel-side copies already sanitized by
	 * the framework; callers must not touch userspace pointers.
	 */
	int	(*gamma_set)(struct drm_crtc *crtc, uint32_t size,
		    const uint16_t *red, const uint16_t *green,
		    const uint16_t *blue);
};

struct drm_crtc {
	struct drm_mode_object		 base;	/* must be first */
	struct drm_device		*dev;
	const struct drm_crtc_funcs	*funcs;
	uint32_t			 index;	/* assigned at init; bit
						 * position in
						 * possible_crtcs masks */
	bool				 enabled;
	struct drm_framebuffer		*primary_fb;
	struct drm_plane		*primary_plane;
	/*
	 * Cached modeset state.  SETCRTC populates these; GETCRTC reports
	 * them back.  mode_valid is 0 when the CRTC is blanked.  The
	 * primary_fb ref above and any future plane refs are released
	 * when the CRTC is re-armed or destroyed.
	 */
	struct drm_display_mode		 mode;
	uint32_t			 x;
	uint32_t			 y;
	uint8_t				 mode_valid;

	/*
	 * VBLANK accounting (Phase 9g).  sequence advances on every
	 * driver vblank IRQ via kms_vblank_handler; pending_flip
	 * is the file that armed a PAGE_FLIP_EVENT and the user_data
	 * cookie that page-flip-event delivery echoes back.  Both are
	 * protected by drm_device->mode_config.mutex held exclusive
	 * for mutation, shared for reads.  WAIT_VBLANK sleeps tsleep()
	 * on &crtc->sequence.
	 */
	uint32_t			 sequence;
	struct drm_file			*pending_flip_file;
	uint64_t			 pending_flip_user_data;
	/*
	 * H3: outgoing fb held from PAGE_FLIP arm-time until the vblank
	 * handler fires FLIP_COMPLETE.  Prevents the VOP scan-out engine
	 * from reading freed memory for up to a frame after the flip is
	 * armed but before it latches.  NULL when no flip is armed.
	 */
	struct drm_framebuffer		*pending_flip_old_fb;
	/*
	 * WAIT_VBLANK + _DRM_VBLANK_EVENT outstanding requests.  Each
	 * carries the target sequence; kms_vblank_handler dispatches +
	 * frees the entry when sequence >= target.  Multiple outstanding
	 * (queued renderer frames, multiple clients) so this is a list,
	 * not a single slot.
	 */
	TAILQ_HEAD(kms_pending_vblank_event_head, kms_pending_vblank_event)
					 pending_vblank_events;
};

struct kms_pending_vblank_event {
	TAILQ_ENTRY(kms_pending_vblank_event)	 link;
	struct drm_file				*file;
	uint32_t				 target_seq;
	uint64_t				 user_data;
};

int	kms_crtc_init(struct drm_device *dev, struct drm_crtc *crtc,
	    const struct drm_crtc_funcs *funcs);
void	kms_crtc_cleanup(struct drm_crtc *crtc);

#endif /* _KMS_DRM_CRTC_H_ */
