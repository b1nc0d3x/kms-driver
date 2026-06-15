/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * igen live hot-plug detect decoder.  Split out of igen.c for topical
 * cohesion: this file owns the three HPD oracles (SFUSE_STRAP for
 * which DDIs the silicon fused on, SHOTPLUG_CTL_DDI for the per-port
 * enable + pulse history, SDEISR for the live HPD bits) and the sysctl
 * that dumps them in human-readable form.
 *
 * The capability table (cap_dump) in igen.c also touches SFUSE_STRAP /
 * SDEISR, so those register addresses live in igen_internal.h rather
 * than here.
 *
 * Exported entry point:
 *   void igen_hpd_register_sysctls(sc);
 *
 * Sysctl registered here:
 *   dev.igen.<n>.re.hpd_dump   =1 dumps SFUSE_STRAP, SHOTPLUG_CTL_DDI,
 *                              SDEISR with per-DDI decode.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/sysctl.h>

#include "igen_internal.h"

/*
 * SKL+ HPD register layout summary:
 *   - SFUSE_STRAP (0xc2014): bits[2:0] set per DDI present on package
 *     (fuse-set at boot; not real-time, just "this PORT exists")
 *   - SHOTPLUG_CTL_DDI (0xc4030): 4 bits per port [A,B,C,D,E]:
 *       bit hpd_pin*4+0: short-pulse seen
 *       bit hpd_pin*4+1: long-pulse seen (plug/unplug edge)
 *       bit hpd_pin*4+4: HPD irq enable
 *   - SDEISR (0xc4000): PCH interrupt status; DDI HPD live bits here too
 */

static int
igen_sysctl_hpd_dump(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	uint32_t sfuse, hot, sde;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	sfuse = igen_r32(sc, SFUSE_STRAP);
	hot   = igen_r32(sc, SHOTPLUG_CTL_DDI);
	sde   = igen_r32(sc, SDEISR);

	device_printf(sc->dev,
	    "hpd: SFUSE_STRAP=0x%08x  SHOTPLUG_CTL_DDI=0x%08x  SDEISR=0x%08x\n",
	    sfuse, hot, sde);

	device_printf(sc->dev,
	    "  SFUSE_STRAP DDI present: B=%d C=%d D=%d\n",
	    (sfuse >> 2) & 1, (sfuse >> 1) & 1, (sfuse >> 0) & 1);

	/* SHOTPLUG_CTL_DDI: 4 bits per port; bit0..3=port A, 4..7=B, ... */
	for (int p = 0; p < 5; p++) {
		uint32_t f = (hot >> (p * 4)) & 0xf;
		device_printf(sc->dev,
		    "  SHOTPLUG_CTL DDI_%c: en=%d  short_pulse=%d  long_pulse=%d  raw=0x%x\n",
		    'A' + p, (f >> 4) & 1, f & 1, (f >> 1) & 1, f);
	}

	/*
	 * SDEISR HPD live bits (SKL+ desktop):
	 *   bit 21 = DDI_B HPD live
	 *   bit 22 = DDI_C
	 *   bit 23 = DDI_D
	 *   bit 24 = DDI_E
	 */
	device_printf(sc->dev,
	    "  SDEISR HPD live: B=%d C=%d D=%d E=%d\n",
	    (sde >> 21) & 1, (sde >> 22) & 1,
	    (sde >> 23) & 1, (sde >> 24) & 1);
	return (0);
}

void
igen_hpd_register_sysctls(struct igen_softc *sc)
{
	struct sysctl_oid_list *children;

	children = SYSCTL_CHILDREN(sc->re_sysctl_tree);

	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "hpd_dump",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_hpd_dump, "I",
	    "write 1 to dump SFUSE_STRAP / SHOTPLUG_CTL_DDI / SDEISR live HPD");
}
