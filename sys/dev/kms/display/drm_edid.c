/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * E-EDID base block parser.  Validates the 128-byte block and walks
 * its four Detailed Timing Descriptors, turning each non-empty DTD
 * into a drm_display_mode appended to the supplied connector.
 *
 * The DTD layout is from VESA EDID 1.4 §3.10.  Each DTD is 18 bytes;
 * bytes 0-1 are the pixel clock in units of 10 kHz (little-endian).
 * A pixel-clock value of zero marks the slot as one of the alternate
 * descriptor types (monitor name, range limits, etc.) which Phase 5
 * skips.  Higher-order bits for timing fields are packed into shared
 * bytes — see the EDID_DTD_* macros below.
 */

#include <sys/param.h>
#include <sys/systm.h>

#include <kms/drm_connector.h>
#include <kms/drm_edid.h>
#include <kms/drm_modes.h>

#include "kms_internal.h"

static const uint8_t drm_edid_header[8] = KMS_EDID_MAGIC;

/*
 * Offsets inside one 18-byte DTD.
 */
#define	EDID_DTD_PIXCLK_LO	0
#define	EDID_DTD_PIXCLK_HI	1
#define	EDID_DTD_HACTIVE_LO	2
#define	EDID_DTD_HBLANK_LO	3
#define	EDID_DTD_HMSB		4	/* bits 7-4 hactive, 3-0 hblank */
#define	EDID_DTD_VACTIVE_LO	5
#define	EDID_DTD_VBLANK_LO	6
#define	EDID_DTD_VMSB		7	/* bits 7-4 vactive, 3-0 vblank */
#define	EDID_DTD_HSYNC_OFF_LO	8
#define	EDID_DTD_HSYNC_WID_LO	9
#define	EDID_DTD_VSYNC_OFFWID	10	/* bits 7-4 voff, 3-0 vwid */
#define	EDID_DTD_SYNC_MSB	11	/* hoff[7-6] hwid[5-4]
					 * voff[3-2] vwid[1-0] */
#define	EDID_DTD_HIMG_LO	12
#define	EDID_DTD_VIMG_LO	13
#define	EDID_DTD_IMG_MSB	14	/* bits 7-4 himg, 3-0 vimg */
#define	EDID_DTD_HBORDER	15
#define	EDID_DTD_VBORDER	16
#define	EDID_DTD_FLAGS		17

#define	EDID_DTD_FLAG_INTERLACE	(1 << 7)
#define	EDID_DTD_FLAG_SYNC_MASK	0x18	/* bits 4-3 */
#define	EDID_DTD_FLAG_SYNC_SEP	0x18	/* digital separate sync */
#define	EDID_DTD_FLAG_VSYNC_POS	(1 << 2)
#define	EDID_DTD_FLAG_HSYNC_POS	(1 << 1)

/*
 * Base-block constants.
 */
#define	EDID_OFFSET_DTD0	54
#define	EDID_DTD_SIZE		18
#define	EDID_NUM_DTDS		4
#define	EDID_OFFSET_MFG		8
#define	EDID_OFFSET_PRODUCT	10
#define	EDID_OFFSET_SERIAL	12
#define	EDID_OFFSET_WEEK	16
#define	EDID_OFFSET_YEAR	17
#define	EDID_OFFSET_HIMG_CM	21
#define	EDID_OFFSET_VIMG_CM	22

bool
drm_edid_checksum(const uint8_t *data)
{
	uint8_t sum = 0;
	size_t i;

	for (i = 0; i < KMS_EDID_BLOCK_SIZE; i++)
		sum = (uint8_t)(sum + data[i]);
	return (sum == 0);
}

/*
 * Decode the 5+5+5-bit manufacturer code stored big-endian at offset
 * 8.  Each 5-bit field is a letter index (1='A' ... 26='Z').  Writes
 * three characters plus NUL into out[0..3].  Invalid letters become
 * '?'.
 */
static void
drm_edid_decode_pnp(uint16_t mfg, char out[4])
{
	static const char alpha[27] = "@ABCDEFGHIJKLMNOPQRSTUVWXYZ";
	uint8_t a, b, c;

	a = (mfg >> 10) & 0x1f;
	b = (mfg >> 5) & 0x1f;
	c = mfg & 0x1f;
	out[0] = (a >= 1 && a <= 26) ? alpha[a] : '?';
	out[1] = (b >= 1 && b <= 26) ? alpha[b] : '?';
	out[2] = (c >= 1 && c <= 26) ? alpha[c] : '?';
	out[3] = '\0';
}

int
drm_edid_parse(const uint8_t *data, size_t len, struct drm_edid_info *info)
{
	uint16_t mfg_be;
	uint8_t week, year_raw;

	if (data == NULL || len < KMS_EDID_BLOCK_SIZE)
		return (EINVAL);
	if (memcmp(data, drm_edid_header, sizeof(drm_edid_header)) != 0)
		return (EINVAL);
	if (!drm_edid_checksum(data))
		return (EINVAL);
	if (info == NULL)
		return (0);

	memset(info, 0, sizeof(*info));
	mfg_be = ((uint16_t)data[EDID_OFFSET_MFG] << 8) |
	    data[EDID_OFFSET_MFG + 1];
	drm_edid_decode_pnp(mfg_be, info->pnp_id);

	info->product_code = (uint16_t)data[EDID_OFFSET_PRODUCT] |
	    ((uint16_t)data[EDID_OFFSET_PRODUCT + 1] << 8);
	info->serial = (uint32_t)data[EDID_OFFSET_SERIAL] |
	    ((uint32_t)data[EDID_OFFSET_SERIAL + 1] << 8) |
	    ((uint32_t)data[EDID_OFFSET_SERIAL + 2] << 16) |
	    ((uint32_t)data[EDID_OFFSET_SERIAL + 3] << 24);
	week = data[EDID_OFFSET_WEEK];
	year_raw = data[EDID_OFFSET_YEAR];
	info->week = week;
	info->year = (uint16_t)year_raw + 1990;

	info->width_mm = (uint16_t)data[EDID_OFFSET_HIMG_CM] * 10;
	info->height_mm = (uint16_t)data[EDID_OFFSET_VIMG_CM] * 10;
	return (0);
}

/*
 * Decode one 18-byte DTD into a fresh drm_display_mode.  Returns the
 * malloc'd mode on success, NULL if the slot is an alternate
 * descriptor (pixel_clock == 0) or if the timing fails sanity
 * (htotal/vtotal compute to zero).  Caller owns the storage.
 */
static struct drm_display_mode *
drm_edid_dtd_to_mode(const uint8_t *dtd)
{
	struct drm_display_mode *mode;
	uint32_t pixel_clock;
	uint16_t hactive, hblank, hsync_off, hsync_wid;
	uint16_t vactive, vblank, vsync_off, vsync_wid;
	uint16_t himg, vimg;
	uint8_t flags;

	pixel_clock = (uint32_t)dtd[EDID_DTD_PIXCLK_LO] |
	    ((uint32_t)dtd[EDID_DTD_PIXCLK_HI] << 8);
	if (pixel_clock == 0)
		return (NULL);

	hactive = dtd[EDID_DTD_HACTIVE_LO] |
	    ((uint16_t)(dtd[EDID_DTD_HMSB] & 0xf0) << 4);
	hblank = dtd[EDID_DTD_HBLANK_LO] |
	    ((uint16_t)(dtd[EDID_DTD_HMSB] & 0x0f) << 8);
	vactive = dtd[EDID_DTD_VACTIVE_LO] |
	    ((uint16_t)(dtd[EDID_DTD_VMSB] & 0xf0) << 4);
	vblank = dtd[EDID_DTD_VBLANK_LO] |
	    ((uint16_t)(dtd[EDID_DTD_VMSB] & 0x0f) << 8);
	if (hactive == 0 || vactive == 0 ||
	    hblank == 0 || vblank == 0)
		return (NULL);

	hsync_off = dtd[EDID_DTD_HSYNC_OFF_LO] |
	    ((uint16_t)(dtd[EDID_DTD_SYNC_MSB] & 0xc0) << 2);
	hsync_wid = dtd[EDID_DTD_HSYNC_WID_LO] |
	    ((uint16_t)(dtd[EDID_DTD_SYNC_MSB] & 0x30) << 4);
	vsync_off = ((dtd[EDID_DTD_VSYNC_OFFWID] >> 4) & 0x0f) |
	    ((uint16_t)(dtd[EDID_DTD_SYNC_MSB] & 0x0c) << 2);
	vsync_wid = (dtd[EDID_DTD_VSYNC_OFFWID] & 0x0f) |
	    ((uint16_t)(dtd[EDID_DTD_SYNC_MSB] & 0x03) << 4);

	himg = dtd[EDID_DTD_HIMG_LO] |
	    ((uint16_t)(dtd[EDID_DTD_IMG_MSB] & 0xf0) << 4);
	vimg = dtd[EDID_DTD_VIMG_LO] |
	    ((uint16_t)(dtd[EDID_DTD_IMG_MSB] & 0x0f) << 8);

	mode = drm_mode_create();
	mode->clock = pixel_clock * 10;	/* DTD stores 10-kHz units */
	mode->hdisplay = hactive;
	mode->hsync_start = hactive + hsync_off;
	mode->hsync_end = hactive + hsync_off + hsync_wid;
	mode->htotal = hactive + hblank;
	mode->vdisplay = vactive;
	mode->vsync_start = vactive + vsync_off;
	mode->vsync_end = vactive + vsync_off + vsync_wid;
	mode->vtotal = vactive + vblank;
	mode->width_mm = himg;
	mode->height_mm = vimg;

	flags = dtd[EDID_DTD_FLAGS];
	if (flags & EDID_DTD_FLAG_INTERLACE)
		mode->flags |= KMS_MODE_FLAG_INTERLACE;
	/*
	 * Digital separate sync — the only case modern DP/HDMI panels
	 * use.  Composite/bipolar sync would set different polarity
	 * bits and isn't reachable through cdn_dp/dw-hdmi paths.
	 */
	if ((flags & EDID_DTD_FLAG_SYNC_MASK) == EDID_DTD_FLAG_SYNC_SEP) {
		mode->flags |= (flags & EDID_DTD_FLAG_HSYNC_POS) ?
		    KMS_MODE_FLAG_PHSYNC :
		    KMS_MODE_FLAG_NHSYNC;
		mode->flags |= (flags & EDID_DTD_FLAG_VSYNC_POS) ?
		    KMS_MODE_FLAG_PVSYNC :
		    KMS_MODE_FLAG_NVSYNC;
	}
	return (mode);
}

int
drm_edid_add_modes(struct drm_connector *connector, const uint8_t *data,
    size_t len)
{
	struct drm_edid_info info;
	struct drm_display_mode *mode;
	int added = 0;
	int error;
	int i;

	error = drm_edid_parse(data, len, &info);
	if (error != 0)
		return (-error);

	if (connector->mm_width == 0)
		connector->mm_width = info.width_mm;
	if (connector->mm_height == 0)
		connector->mm_height = info.height_mm;

	for (i = 0; i < EDID_NUM_DTDS; i++) {
		const uint8_t *dtd;

		dtd = data + EDID_OFFSET_DTD0 + (i * EDID_DTD_SIZE);
		mode = drm_edid_dtd_to_mode(dtd);
		if (mode == NULL)
			continue;
		/*
		 * The first DTD is the panel's preferred timing per
		 * EDID 1.4 §3.10.3.  Tag it so userspace can pick it
		 * as the default mode without consulting modeline
		 * priorities.
		 */
		if (i == 0)
			mode->type |= KMS_MODE_TYPE_PREFERRED;
		mode->type |= KMS_MODE_TYPE_DRIVER;
		kms_connector_add_mode(connector, mode);
		added++;
	}
	return (added);
}
