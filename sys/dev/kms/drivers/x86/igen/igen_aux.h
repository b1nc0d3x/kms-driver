/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Per-DDI AUX channel context.  Bridges the kms framework's
 * drm_dp_aux (driver-agnostic transport) to the HSW-specific
 * DDI_AUX_CTL/DATA register block in igen_aux.c.  One of these is
 * allocated per port the driver intends to talk DPCD on (typically
 * just DDI A for the eDP panel, plus DDI B/C/D/E as external DP/DP++
 * sinks are detected).
 */

#ifndef _IGEN_AUX_H_
#define _IGEN_AUX_H_

#include <sys/types.h>

#include <kms/drm_dp_helper.h>

struct igen_softc;

struct igen_aux_channel {
	struct drm_dp_aux	 aux;	/* framework-facing handle */
	struct igen_softc	*sc;	/* back-pointer for MMIO */
	int			 ddi;	/* 0=A..4=E */
};

ssize_t	igen_aux_transfer(struct drm_dp_aux *aux, struct drm_dp_aux_msg *msg);
int	igen_aux_i2c_read_block(struct drm_dp_aux *aux, uint8_t i2c_addr,
	    uint8_t offset, uint8_t *buf, size_t size);
void	igen_aux_init(struct igen_softc *sc, struct igen_aux_channel *ch,
	    int ddi, const char *name);

#endif /* _IGEN_AUX_H_ */
