/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * EDID parser API.  Validates a 128-byte E-EDID block, extracts
 * monitor metadata, and converts the four Detailed Timing Descriptors
 * into drm_display_mode entries appended to a connector's mode list.
 *
 * Phase 5 scope:
 *   - One 128-byte base block (no extension blocks).
 *   - The four DTDs at offsets 54, 72, 90, 108.
 *   - Manufacturer / product / week / year decode.
 *
 * CTA-861 extension blocks (audio data, additional DTDs, HDMI VSDB)
 * land alongside the rk_cdn_dp / rk_hdmi ports in Phase 9.
 */

#ifndef _KMS_DRM_EDID_H_
#define _KMS_DRM_EDID_H_

#include <sys/types.h>

struct drm_connector;

#define	KMS_EDID_BLOCK_SIZE	128
#define	KMS_EDID_MAGIC		{ 0x00, 0xff, 0xff, 0xff, \
					  0xff, 0xff, 0xff, 0x00 }

/*
 * Decoded monitor identification.  Populated by kms_edid_parse from
 * the vendor block of a validated EDID.  pnp_id is a 3-character
 * Plug-and-Play vendor code (e.g. "SAM" for Samsung); the buffer
 * always has a NUL terminator at index 3.
 */
struct drm_edid_info {
	char		pnp_id[4];
	uint16_t	product_code;
	uint32_t	serial;
	uint8_t		week;		/* 0-54, or 0xFF = model-year flag */
	uint16_t	year;		/* if week<255: year of manufacture;
					 * else model year, both relative to
					 * 1990 + raw byte */
	uint16_t	width_mm;	/* maximum image size from the
					 * basic display parameters block */
	uint16_t	height_mm;
};

/*
 * Verify the EDID block's checksum: every byte summed mod 256 must
 * equal 0.  Returns true on a good checksum.  Does not validate the
 * magic header — kms_edid_parse does that.
 */
bool	kms_edid_checksum(const uint8_t *data);

/*
 * Validate + decode an EDID block.  data must be at least 128 bytes.
 * Returns 0 on success and fills *info; returns EINVAL on bad magic
 * or bad checksum.  info may be NULL when only validation is wanted.
 */
int	kms_edid_parse(const uint8_t *data, size_t len,
	    struct drm_edid_info *info);

/*
 * Validate the EDID block, then convert each non-empty Detailed
 * Timing Descriptor into a drm_display_mode and append it to
 * connector->modes via kms_connector_add_mode.  Sets
 * connector->mm_width / mm_height from the basic display block if
 * they weren't already set.  Returns the number of modes added on
 * success or a negative errno on failure.  On error nothing is added.
 */
int	kms_edid_add_modes(struct drm_connector *connector,
	    const uint8_t *data, size_t len);

#endif /* _KMS_DRM_EDID_H_ */
