/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Driver-facing vblank API.
 *
 * Drivers call kms_vblank_handler from their vblank source
 * (real IRQ, software timer, whatever) on every refresh.  The
 * framework advances the per-CRTC sequence counter, wakes
 * WAIT_VBLANK sleepers, and delivers a FLIP_COMPLETE event for any
 * page-flip that was armed with DRM_MODE_PAGE_FLIP_EVENT.
 */

#ifndef _KMS_DRM_VBLANK_H_
#define _KMS_DRM_VBLANK_H_

struct drm_crtc;

void	kms_vblank_handler(struct drm_crtc *crtc);

#endif /* _KMS_DRM_VBLANK_H_ */
