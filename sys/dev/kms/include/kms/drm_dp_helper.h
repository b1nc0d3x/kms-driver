/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * DisplayPort AUX channel helpers.  Provides the transport abstraction
 * a DP source driver (rk_cdn_dp, future i.MX HDP, …) registers, plus
 * convenience wrappers for native-DPCD read / write at fixed offsets.
 *
 * Phase 5 scope:
 *   - Native AUX request types: read, write, address-only.
 *   - Per-request transfer callback supplied by the driver.
 *   - Retry on AUX_NACK and AUX_DEFER per the DP 1.4 §2.7.5 rules.
 *   - DPCD register read/write at typical offsets (rev, link rate,
 *     lane count, status registers).
 *
 * I2C-over-AUX (for EDID reads on DP) lands when rk_cdn_dp needs it
 * in Phase 9 — the .transfer callback already carries the I2C request
 * type bits so the wire format is forward-compatible.
 */

#ifndef _KMS_DRM_DP_HELPER_H_
#define _KMS_DRM_DP_HELPER_H_

#include <sys/types.h>

struct drm_dp_aux;

/*
 * AUX request types.  Values are the wire-level "command" field used
 * by every DP source IP (DP 1.4 §2.7.5.1, Table 2-100).  Drivers
 * receiving a struct drm_dp_aux_msg pass these straight through to
 * their transmit register.
 */
#define	DP_AUX_NATIVE_WRITE	0x8
#define	DP_AUX_NATIVE_READ	0x9
#define	DP_AUX_I2C_WRITE	0x0
#define	DP_AUX_I2C_READ		0x1
#define	DP_AUX_I2C_WRITE_STATUS	0x2
#define	DP_AUX_I2C_MOT		0x4	/* OR'd with I2C ops */

/*
 * Reply status returned by the sink in the first byte of the reply
 * buffer.  ACK indicates the transfer succeeded; the rest signal
 * retry-on-defer or hard NACK.
 */
#define	DP_AUX_NATIVE_REPLY_ACK		0x00
#define	DP_AUX_NATIVE_REPLY_NACK	0x01
#define	DP_AUX_NATIVE_REPLY_DEFER	0x02

#define	DP_AUX_I2C_REPLY_ACK		0x00
#define	DP_AUX_I2C_REPLY_NACK		0x04
#define	DP_AUX_I2C_REPLY_DEFER		0x08

/*
 * Common DPCD register offsets the link-training and capability
 * paths use.  More land as Phase 7+ implements modeset.  Values
 * match the DP 1.4 spec (and Linux's <drm/dp/drm_dp_helper.h>) so
 * a mechanically-ported driver doesn't translate constants.
 */
#define	DP_DPCD_REV			0x000
#define	DP_MAX_LINK_RATE		0x001
#define	DP_MAX_LANE_COUNT		0x002
#define	DP_MAX_DOWNSPREAD		0x003
#define	DP_TRAINING_PATTERN_SET		0x102
#define	DP_TRAINING_LANE0_SET		0x103
#define	DP_LANE0_1_STATUS		0x202
#define	DP_LANE2_3_STATUS		0x203
#define	DP_LANE_ALIGN_STATUS_UPDATED	0x204
#define	DP_SINK_STATUS			0x205

/*
 * One AUX transaction.  Drivers populate the wire fields and pass to
 * their own .transfer callback (or to drm_dp_dpcd_read/write which
 * does it for them).  size carries the requested byte count; the
 * transfer's return value is the number of bytes actually exchanged
 * (which may be smaller on a partial reply — see DP 1.4 §2.7.5.5).
 */
struct drm_dp_aux_msg {
	uint32_t	 address;	/* DPCD or I2C sub-address */
	uint8_t		 request;	/* DP_AUX_* */
	uint8_t		 reply;		/* set by driver/transfer */
	void		*buffer;
	size_t		 size;
};

/*
 * Per-connector AUX channel.  Driver allocates, fills name +
 * .transfer, registers via drm_dp_aux_init.  drm_dp_aux is reused
 * (no allocation) across multiple link-training cycles for the
 * same DP source.
 */
struct drm_dp_aux {
	const char	*name;		/* for kprintf identification */
	ssize_t		(*transfer)(struct drm_dp_aux *aux,
			    struct drm_dp_aux_msg *msg);
	void		*priv;		/* driver-owned context */
};

/*
 * Initialize a drm_dp_aux struct in place.  Driver supplies its
 * transfer callback before calling.  No allocation.
 */
void	drm_dp_aux_init(struct drm_dp_aux *aux);

/*
 * Submit a single native-AUX request.  Retries on DEFER per DP
 * 1.4 §2.7.5.5 (up to 7 attempts, monotonic backoff).  Returns the
 * byte count exchanged on success, or a negative errno on hard NACK,
 * transport failure, or exhausted retries.
 */
ssize_t	drm_dp_aux_transfer(struct drm_dp_aux *aux,
	    struct drm_dp_aux_msg *msg);

/*
 * Convenience: read or write `size` bytes from/to the DPCD at
 * `offset`.  Wraps drm_dp_aux_transfer and handles the size argument.
 * Returns bytes actually read/written, or a negative errno.
 */
ssize_t	drm_dp_dpcd_read(struct drm_dp_aux *aux, uint32_t offset,
	    void *buffer, size_t size);
ssize_t	drm_dp_dpcd_write(struct drm_dp_aux *aux, uint32_t offset,
	    const void *buffer, size_t size);

#endif /* _KMS_DRM_DP_HELPER_H_ */
