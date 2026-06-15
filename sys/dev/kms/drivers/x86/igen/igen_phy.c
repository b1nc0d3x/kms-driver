/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * igen SKL display PHY / PCS / Common Lane RE substrate.
 *
 * The DDI_BUF_CTL register only controls the top-of-stack digital
 * enable.  Below it sit:
 *
 *   * a per-PHY common-lane block (CL_DWx) — reference clock + power
 *     gates that arm the entire PHY
 *   * a per-PHY reference-clock block (REF_DWx) — synthesizer trim
 *   * per-port physical-coding-sublayer registers (PCS_DWx) — bit
 *     ordering, scrambling, training pattern
 *   * per-lane analog transmitter registers (TX_DWx) — voltage swing,
 *     pre-emphasis, slew, clock recovery
 *
 * On SKL/KBL/CFL the silicon has two PHYs:
 *
 *   * PHY_A  — serves DDI_A (eDP) and DDI_E (DP-E).  Base 0x162000.
 *   * PHY_BC — serves DDI_B, DDI_C, DDI_D.  Base 0x6c000.
 *
 * Within PHY_BC the per-port channels live at:
 *
 *   * DDI_B channel: ~0x6c400..0x6c5ff   (PCS) + 0x6c500..0x6c5ff (TX)
 *   * DDI_C channel: ~0x6c600..0x6c7ff   (PCS) + 0x6c700..0x6c7ff (TX)
 *   * DDI_D channel: ~0x6c800..0x6c9ff   (PCS) + 0x6c900..0x6c9ff (TX)
 *
 * This file is RE substrate — the macros and sysctls here are for
 * snapshotting and decoding the live PHY state so we can diff it
 * against firmware's working configuration.  Programming paths
 * (when we have a reproducible recipe) belong in igen_dpll.c.
 *
 * Exported entry point:
 *   void igen_phy_register_sysctls(sc);
 *
 * Sysctls registered here:
 *   dev.igen.<n>.re.phy_dump_bc   =1 decode CL+REF+PCS+TX for DDI_B
 *                                 =2 same for DDI_C
 *                                 =3 same for DDI_D
 *   dev.igen.<n>.re.phy_scan_bc   =1 brute-force walk 0x6c000..0x6cfff
 *                                  printing every nonzero dword
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/sysctl.h>

#include "igen_internal.h"

/* ---- Common Lane (per-PHY shared block) -------------------------------- */
#define	PORT_CL_DW10_BC		0x0006c028
#define	PORT_CL_DW12_BC		0x0006c030
#define	PORT_CL_DW10_A		0x00162028
#define	PORT_CL_DW12_A		0x00162030

/* ---- Reference-clock / synthesizer trim -------------------------------- */
#define	PORT_REF_DW3_BC		0x0006c18c
#define	PORT_REF_DW6_BC		0x0006c198
#define	PORT_REF_DW8_BC		0x0006c1a0
#define	PORT_REF_DW3_A		0x0016218c
#define	PORT_REF_DW6_A		0x00162198
#define	PORT_REF_DW8_A		0x001621a0

/*
 * Per-port channel base within PHY_BC.  Lane and PCS register addresses
 * are this base + a per-DW offset.  i915 calls these "DDI port" indices
 * for the BC PHY: DDI_B=0, DDI_C=1, DDI_D=2.
 */
#define	BC_CHAN_BASE(p)		(0x0006c000 + 0x200 * ((p) + 2))
/*
 *   DDI_B (p=0): 0x6c400
 *   DDI_C (p=1): 0x6c600
 *   DDI_D (p=2): 0x6c800
 *
 * PCS regs live in the low half (0x000..0x0ff inside the channel);
 * per-lane TX regs in the high half (0x100..0x1ff).  Four lanes per port,
 * 0x80 apart in the original i915 macros — see i915_reg.h SKL_PHY block.
 */
#define	PORT_PCS_DW10_LN0(p)	(BC_CHAN_BASE(p) + 0x028)
#define	PORT_PCS_DW12_LN0(p)	(BC_CHAN_BASE(p) + 0x030)
#define	PORT_TX_DW2_LN0(p)	(BC_CHAN_BASE(p) + 0x108)
#define	PORT_TX_DW2_LN1(p)	(BC_CHAN_BASE(p) + 0x188)
#define	PORT_TX_DW4_LN0(p)	(BC_CHAN_BASE(p) + 0x110)
#define	PORT_TX_DW5_LN0(p)	(BC_CHAN_BASE(p) + 0x114)
#define	PORT_TX_DW14_LN0(p)	(BC_CHAN_BASE(p) + 0x138)

/*
 * Decode a single port (DDI_B=0, DDI_C=1, DDI_D=2).  Read-only; no MMIO
 * writes.  Useful to compare firmware-driven analog state against our
 * post-modeset state.
 */
static void
igen_phy_dump_bc_port(struct igen_softc *sc, int port)
{
	char ddi = 'B' + port;

	device_printf(sc->dev,
	    "phy DDI_%c (channel base 0x%05x):\n",
	    ddi, BC_CHAN_BASE(port));
	device_printf(sc->dev,
	    "  CL_DW10  =0x%08x  CL_DW12  =0x%08x\n",
	    igen_r32(sc, PORT_CL_DW10_BC),
	    igen_r32(sc, PORT_CL_DW12_BC));
	device_printf(sc->dev,
	    "  REF_DW3  =0x%08x  REF_DW6  =0x%08x  REF_DW8  =0x%08x\n",
	    igen_r32(sc, PORT_REF_DW3_BC),
	    igen_r32(sc, PORT_REF_DW6_BC),
	    igen_r32(sc, PORT_REF_DW8_BC));
	device_printf(sc->dev,
	    "  PCS_DW10 =0x%08x  PCS_DW12 =0x%08x\n",
	    igen_r32(sc, PORT_PCS_DW10_LN0(port)),
	    igen_r32(sc, PORT_PCS_DW12_LN0(port)));
	device_printf(sc->dev,
	    "  TX_DW2_LN0=0x%08x  TX_DW2_LN1=0x%08x\n",
	    igen_r32(sc, PORT_TX_DW2_LN0(port)),
	    igen_r32(sc, PORT_TX_DW2_LN1(port)));
	device_printf(sc->dev,
	    "  TX_DW4_LN0=0x%08x  TX_DW5_LN0=0x%08x  TX_DW14_LN0=0x%08x\n",
	    igen_r32(sc, PORT_TX_DW4_LN0(port)),
	    igen_r32(sc, PORT_TX_DW5_LN0(port)),
	    igen_r32(sc, PORT_TX_DW14_LN0(port)));
}

static int
igen_sysctl_phy_dump_bc(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	int port = 0;
	int error = sysctl_handle_int(oidp, &port, 0, req);

	if (error || req->newptr == NULL)
		return (error);
	if (port < 1 || port > 3) {
		device_printf(sc->dev,
		    "phy_dump_bc: write 1=DDI_B, 2=DDI_C, 3=DDI_D\n");
		return (0);
	}
	igen_phy_dump_bc_port(sc, port - 1);
	return (0);
}

/*
 * Brute-force walk 0x6c000..0x6cfff printing nonzero dwords.  This is the
 * companion to mmio_snapshot_save/diff — gives a single-shot view of every
 * programmed PHY_BC register, without needing to know the symbolic names.
 * Useful as a cold-boot baseline capture.
 *
 * SAFETY GATE: refuses to run if any pipe has PIPE_CONF ENABLE set.  Reading
 * the 1024-dword PHY window in tight succession while the display is live
 * scanout-of-DDI_B has been observed to stall the display-engine bus and
 * wedge the iGPU until reboot.  Write 2 to override (use only when
 * deliberately debugging on a hung pipe and a reset is acceptable).
 */
static int
igen_sysctl_phy_scan_bc(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	uint32_t nonzero = 0;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	if (trigger != 2) {
		for (int p = 0; p < 3; p++) {
			uint32_t conf = igen_r32(sc, PIPE_CONF(p));

			if (conf & PIPE_CONF_ENABLE) {
				device_printf(sc->dev,
				    "phy_scan_bc: REFUSE: pipe %c is active"
				    " (PIPE_CONF=0x%08x).  PHY register reads"
				    " during live scanout can wedge the iGPU."
				    "  Write 2 to override.\n",
				    'A' + p, conf);
				return (EBUSY);
			}
		}
	}

	device_printf(sc->dev,
	    "phy_scan_bc: walking 0x6c000..0x6cfff (4 KiB)%s:\n",
	    trigger == 2 ? " [FORCED, pipe may be active]" : "");
	for (uint32_t a = 0x0006c000; a < 0x0006d000; a += 4) {
		uint32_t v = igen_r32(sc, a);
		if (v != 0) {
			device_printf(sc->dev,
			    "  0x%05x = 0x%08x\n", a, v);
			nonzero++;
		}
		/*
		 * Throttle slightly to keep the read storm from monopolising
		 * the MMIO bus even in the idle-pipe path; cheap insurance.
		 */
		if ((a & 0xff) == 0)
			DELAY(2);
	}
	device_printf(sc->dev,
	    "phy_scan_bc: %u nonzero dwords of 1024\n", nonzero);
	return (0);
}

void
igen_phy_register_sysctls(struct igen_softc *sc)
{
	struct sysctl_oid_list *children;

	children = SYSCTL_CHILDREN(sc->re_sysctl_tree);

	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "phy_dump_bc",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_phy_dump_bc, "I",
	    "write 1=DDI_B, 2=DDI_C, 3=DDI_D to decode PHY_BC channel");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "phy_scan_bc",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_phy_scan_bc, "I",
	    "write 1 to brute-force scan 0x6c000..0x6cfff for nonzero dwords");
}
