/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * DisplayPort AUX channel helpers.  Provides defer-retry on top of
 * the driver-supplied transfer callback and convenience read/write
 * wrappers at typical DPCD offsets.  See drm_dp_helper.h for the API
 * surface and DP 1.4 §2.7.5 for the wire-level reference.
 */

#include <sys/param.h>
#include <sys/systm.h>

#include <kms/drm_dp_helper.h>

#include "kms_internal.h"

/*
 * Defer-retry budget per DP 1.4 §2.7.5.5.  After the 7th retry the
 * transport is considered wedged and the helper returns -ETIMEDOUT.
 */
#define	DP_AUX_MAX_RETRIES	7

void
kms_dp_aux_init(struct drm_dp_aux *aux)
{
	/*
	 * No allocation; .transfer + .name + .priv are caller-filled
	 * before this is invoked.  The init is a hook for future
	 * per-channel state (mutex, log ring) without changing the
	 * driver-facing API.
	 */
	(void)aux;
}

ssize_t
kms_dp_aux_transfer(struct drm_dp_aux *aux, struct drm_dp_aux_msg *msg)
{
	ssize_t ret;
	int retries;

	if (aux == NULL || aux->transfer == NULL || msg == NULL)
		return (-EINVAL);
	if (msg->size > 0 && msg->buffer == NULL)
		return (-EINVAL);

	for (retries = 0; retries < DP_AUX_MAX_RETRIES; retries++) {
		ret = aux->transfer(aux, msg);
		if (ret < 0)
			return (ret);
		switch (msg->reply & 0x3) {
		case DP_AUX_NATIVE_REPLY_ACK:
			/*
			 * Short replies are allowed: the sink may
			 * deliver fewer bytes than requested.  Caller
			 * gets the actual count.
			 */
			return (ret);
		case DP_AUX_NATIVE_REPLY_NACK:
			return (-EIO);
		case DP_AUX_NATIVE_REPLY_DEFER:
			/*
			 * Yield briefly between retries — most DP sinks
			 * defer when they're still EDID-fetching or
			 * power-stabilizing.  400-500 µs is the
			 * canonical wait (DP 1.4 §2.7.5.5).
			 */
			DELAY(500);
			continue;
		}
	}
	return (-ETIMEDOUT);
}

ssize_t
kms_dp_dpcd_read(struct drm_dp_aux *aux, uint32_t offset, void *buffer,
    size_t size)
{
	struct drm_dp_aux_msg msg;

	memset(&msg, 0, sizeof(msg));
	msg.address = offset;
	msg.request = DP_AUX_NATIVE_READ;
	msg.buffer = buffer;
	msg.size = size;
	return (kms_dp_aux_transfer(aux, &msg));
}

ssize_t
kms_dp_dpcd_write(struct drm_dp_aux *aux, uint32_t offset, const void *buffer,
    size_t size)
{
	struct drm_dp_aux_msg msg;

	memset(&msg, 0, sizeof(msg));
	msg.address = offset;
	msg.request = DP_AUX_NATIVE_WRITE;
	/*
	 * AUX transfer is one-way per request — the underlying chip
	 * doesn't distinguish const vs non-const buffers, only direction
	 * via msg.request.  Cast is safe; helper functions cannot
	 * propagate constness through the void * field without forking
	 * the message struct.
	 */
	msg.buffer = __DECONST(void *, buffer);
	msg.size = size;
	return (kms_dp_aux_transfer(aux, &msg));
}
