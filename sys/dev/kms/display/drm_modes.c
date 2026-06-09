/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Display-mode helpers: allocate/destroy, refresh-rate compute, name
 * generation, and the kernel→uapi conversion used by GETCONNECTOR.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>

#include <drm/drm_mode.h>
#include <kms/drm_modes.h>

#include "kms_internal.h"

struct drm_display_mode *
kms_mode_create(void)
{
	return (malloc(sizeof(struct drm_display_mode), M_KMS,
	    M_WAITOK | M_ZERO));
}

void
kms_mode_destroy(struct drm_display_mode *mode)
{
	free(mode, M_KMS);
}

uint32_t
kms_mode_vrefresh(const struct drm_display_mode *mode)
{
	uint64_t num, den;

	if (mode->vrefresh != 0)
		return (mode->vrefresh);
	if (mode->htotal == 0 || mode->vtotal == 0 || mode->clock == 0)
		return (0);

	/*
	 * Refresh in Hz = clock * 1000 / (htotal * vtotal), with interlace
	 * doubling and dblscan halving.  Compute in 64-bit to avoid the
	 * intermediate overflow at high pixel clocks (e.g. 4K@60Hz =
	 * 594 MHz × 1000 > 2^32).
	 */
	num = (uint64_t)mode->clock * 1000ULL;
	den = (uint64_t)mode->htotal * (uint64_t)mode->vtotal;
	if (mode->flags & KMS_MODE_FLAG_INTERLACE)
		num *= 2;
	if (mode->flags & KMS_MODE_FLAG_DBLSCAN)
		den *= 2;
	if (mode->vscan > 1)
		den *= mode->vscan;

	return ((uint32_t)((num + (den / 2)) / den));
}

void
kms_mode_set_name(struct drm_display_mode *mode)
{
	bool interlaced;

	if (mode->name[0] != '\0')
		return;
	interlaced = (mode->flags & KMS_MODE_FLAG_INTERLACE) != 0;
	snprintf(mode->name, sizeof(mode->name), "%dx%d%s",
	    mode->hdisplay, mode->vdisplay, interlaced ? "i" : "");
}

void
kms_display_mode_to_modeinfo(const struct drm_display_mode *mode,
    struct drm_mode_modeinfo *info)
{
	size_t n;

	memset(info, 0, sizeof(*info));
	info->clock = mode->clock;
	info->hdisplay = mode->hdisplay;
	info->hsync_start = mode->hsync_start;
	info->hsync_end = mode->hsync_end;
	info->htotal = mode->htotal;
	info->hskew = mode->hskew;
	info->vdisplay = mode->vdisplay;
	info->vsync_start = mode->vsync_start;
	info->vsync_end = mode->vsync_end;
	info->vtotal = mode->vtotal;
	info->vscan = mode->vscan;
	info->vrefresh = kms_mode_vrefresh(mode);
	info->flags = mode->flags;
	info->type = mode->type;
	n = strnlen(mode->name, sizeof(mode->name));
	if (n >= sizeof(info->name))
		n = sizeof(info->name) - 1;
	memcpy(info->name, mode->name, n);
	info->name[n] = '\0';
}
