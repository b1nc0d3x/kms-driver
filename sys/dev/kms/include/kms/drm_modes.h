/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#ifndef _KMS_DRM_MODES_H_
#define _KMS_DRM_MODES_H_

#include <sys/types.h>
#include <sys/queue.h>

struct drm_mode_modeinfo;

/*
 * Length of the mode name buffer.  Matches the Linux uapi
 * DRM_DISPLAY_MODE_LEN so the kernel-side struct can be copied to
 * userspace drm_mode_modeinfo without truncation surprises.
 */
#define	KMS_DISPLAY_MODE_LEN	32

/*
 * Sync polarity / flag bits matching Linux uapi DRM_MODE_FLAG_*.  Only
 * the subset relevant to connector get_modes paths is enumerated here;
 * full list lives in uapi/drm/drm_mode.h.
 */
#define	KMS_MODE_FLAG_PHSYNC	(1 << 0)
#define	KMS_MODE_FLAG_NHSYNC	(1 << 1)
#define	KMS_MODE_FLAG_PVSYNC	(1 << 2)
#define	KMS_MODE_FLAG_NVSYNC	(1 << 3)
#define	KMS_MODE_FLAG_INTERLACE	(1 << 4)
#define	KMS_MODE_FLAG_DBLSCAN	(1 << 5)

/*
 * Mode type bits.  Match Linux uapi DRM_MODE_TYPE_* so the value can
 * be copied straight to drm_mode_modeinfo.type.
 */
#define	KMS_MODE_TYPE_PREFERRED	(1 << 3)
#define	KMS_MODE_TYPE_DEFAULT	(1 << 4)
#define	KMS_MODE_TYPE_DRIVER	(1 << 6)

/*
 * Kernel-side display mode.  This is the rich form: drivers manipulate
 * these and the framework converts to drm_mode_modeinfo for userspace
 * on GETCONNECTOR.  Linked into a connector's modes list via the link
 * field; the connector owns the storage (allocated via
 * drm_mode_create_from_modeinfo or hand-filled by a get_modes hook).
 */
struct drm_display_mode {
	TAILQ_ENTRY(drm_display_mode)	 link;

	uint32_t	clock;		/* pixel clock in kHz */
	uint16_t	hdisplay;
	uint16_t	hsync_start;
	uint16_t	hsync_end;
	uint16_t	htotal;
	uint16_t	hskew;
	uint16_t	vdisplay;
	uint16_t	vsync_start;
	uint16_t	vsync_end;
	uint16_t	vtotal;
	uint16_t	vscan;

	uint32_t	vrefresh;	/* Hz, computed if zero on register */
	uint32_t	flags;		/* KMS_MODE_FLAG_* */
	uint32_t	type;		/* KMS_MODE_TYPE_* */

	uint32_t	width_mm;
	uint32_t	height_mm;

	char		name[KMS_DISPLAY_MODE_LEN];
};

TAILQ_HEAD(drm_display_mode_list, drm_display_mode);

/*
 * Allocate a zeroed display mode in M_KMS.  Caller is the framework
 * (connector-side helpers) or driver code.  Free with drm_mode_destroy.
 */
struct drm_display_mode *drm_mode_create(void);

/*
 * Free a mode allocated by drm_mode_create.  The mode must not be on
 * any connector's modes list when this is called.
 */
void	drm_mode_destroy(struct drm_display_mode *mode);

/*
 * Compute a refresh rate (Hz) from the timing fields.  Used when a
 * driver provides hdisplay/htotal/vdisplay/vtotal/clock but doesn't
 * fill vrefresh.
 */
uint32_t drm_mode_vrefresh(const struct drm_display_mode *mode);

/*
 * Auto-generate a name like "1920x1080" if the mode has no name set,
 * with an "i" suffix for interlaced.  Idempotent: called multiple
 * times produces the same string.
 */
void	drm_mode_set_name(struct drm_display_mode *mode);

/*
 * Convert a kernel drm_display_mode into the uapi drm_mode_modeinfo
 * userspace receives in GETCONNECTOR.  Fills *info in place.
 */
void	drm_display_mode_to_modeinfo(const struct drm_display_mode *mode,
	    struct drm_mode_modeinfo *info);

#endif /* _KMS_DRM_MODES_H_ */
