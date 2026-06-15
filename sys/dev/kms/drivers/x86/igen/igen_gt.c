/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * igen iGPU "GT" (graphics tile) RE substrate — Phase 1.
 *
 * "GT" is Intel's term for the render-engine subsystem that lives next
 * to display on the same die: the Render Command Streamer (RCS), the
 * Blitter (BCS), Video Decode (VCS), Video Enhance (VECS), plus the
 * GuC and HuC microcontrollers that schedule them.  Display has been
 * the focus so far because pixels are what users see; GT is what makes
 * Mesa's GL/Vulkan/compute path run on the iGPU rather than the CPU
 * (llvmpipe).
 *
 * Forcewake gotcha (SKL+): render-engine MMIO sits in a power domain
 * gated by FORCEWAKE_RENDER.  Reads to the engine register range return
 * 0 (or wedge the silicon) unless we first request forcewake and poll
 * the ACK.  Display MMIO has no such requirement, which is why every
 * RE sysctl up to now could just bus_read_4 directly.
 *
 * This file is Phase 1: just dump state.  Phase 2 will be minimal
 * execlist enable (write to ELSP, observe the engine pull from RING
 * buffer).  Phase 3 first batch submission ("no-op + MI_BATCH_BUFFER_
 * END").  Phase 4 GEM-backed batch + integration with the existing KMS
 * BO surface.  Multi-month project; this file is the scaffold.
 *
 * Exported entry point:
 *   void igen_gt_register_sysctls(sc);
 *
 * Sysctls registered here:
 *   dev.igen.<n>.re.gt_status   =1 take forcewake, dump RCS + GuC
 *                                  state, release forcewake.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/sysctl.h>

#include "igen_internal.h"

/* ---- Forcewake (SKL+ multi-domain layout) ------------------------------ */

/*
 * SKL/KBL/CFL gen9 forcewake.  Writing 1 to a request bit gates the
 * corresponding power domain ON; the ACK bit reflects the silicon's
 * acknowledgement (poll until it asserts before reading engine MMIO).
 * The "MT" (multi-threaded) variant uses paired SET / CLEAR masks in
 * the high 16 bits so multiple agents can request the same domain
 * without trampling each other.
 */
#define	FORCEWAKE_RENDER_GEN9		0x0000a188
#define	FORCEWAKE_RENDER_GEN9_ACK	0x0013d84
#define	FORCEWAKE_MEDIA_GEN9		0x0000a270
#define	FORCEWAKE_MEDIA_GEN9_ACK	0x0000d88
#define	FORCEWAKE_BLITTER_GEN9		0x0000a188
#define	  FW_REQ_SET(bit)		(((1u << (bit)) << 16) | (1u << (bit)))
#define	  FW_REQ_CLR(bit)		((1u << (bit)) << 16)

/* ---- Render Command Streamer (RCS, engine 0) --------------------------- */

#define	RING_BASE_RCS			0x00002000
#define	RING_BUFFER_TAIL(b)		((b) + 0x30)
#define	RING_BUFFER_HEAD(b)		((b) + 0x34)
#define	RING_BUFFER_START(b)		((b) + 0x38)
#define	RING_BUFFER_CTL(b)		((b) + 0x3c)
#define	RING_HWS_PGA(b)			((b) + 0x80)
#define	RING_MODE(b)			((b) + 0x29c)
#define	RING_HWSTAM(b)			((b) + 0xe8)

/*
 * ExecList Submission Port — 4×32-bit writes form a context descriptor
 * pair (priority + LRC ptr).  Phase 2 will compose context descriptors
 * and pipe them through here; Phase 1 just reads the status side.
 */
#define	ELSP_RCS			0x00002230
#define	ELSP_STATUS_RCS			0x00002234
#define	EXECLIST_STATUS_RCS		0x00002234
#define	EXECLIST_SQ_CONTENTS_RCS	0x00002510
#define	CSFE_CHICKEN1_RCS		0x000022a0

/* ---- GuC microcontroller -------------------------------------------- */

#define	GUC_STATUS			0x0000c000
#define	  GUC_STATUS_BOOTROM(v)		(((v) >> 0) & 0x7f)
#define	  GUC_STATUS_UCODE(v)		(((v) >> 8) & 0xfff)
#define	  GUC_STATUS_MSG(v)		(((v) >> 24) & 0xff)
#define	GUC_WOPCM_SIZE			0x0000c050
#define	DMA_GUC_WOPCM_OFFSET		0x0000c340
#define	GUC_SHIM_CONTROL		0x0000c064

/*
 * Take forcewake on the render domain.  Returns 0 on ACK, EAGAIN on
 * timeout.  Caller MUST release with igen_gt_fw_render_release on the
 * matched-pair path so the silicon can power-gate again.
 *
 * Per BSpec the request bit for the render domain on gen9 is bit 0 of
 * FORCEWAKE_RENDER_GEN9; ACK polls bit 0 of FORCEWAKE_RENDER_GEN9_ACK.
 */
static int
igen_gt_fw_render_take(struct igen_softc *sc)
{
	igen_w32(sc, FORCEWAKE_RENDER_GEN9, FW_REQ_SET(0));
	for (int i = 0; i < 1000; i++) {
		uint32_t ack = igen_r32(sc, FORCEWAKE_RENDER_GEN9_ACK);

		if (ack & 1u)
			return (0);
		DELAY(10);
	}
	device_printf(sc->dev, "gt: forcewake RENDER ACK timeout\n");
	return (EAGAIN);
}

static void
igen_gt_fw_render_release(struct igen_softc *sc)
{
	igen_w32(sc, FORCEWAKE_RENDER_GEN9, FW_REQ_CLR(0));
}

/* ---- gt_status sysctl --------------------------------------------- */

/*
 * Refuse the dump if pipe A is actively scanning out — phy_scan_bc
 * proved that reading the wrong MMIO range during live scanout can
 * wedge the iGPU's MMIO bus.  Render-engine MMIO probably has the
 * same hazard.  Write 2 to override (used only when you're already
 * planning to power-cycle anyway).
 */
static int
igen_sysctl_gt_status(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	uint32_t fw, guc, ring_head, ring_tail, ring_start, ring_ctl;
	uint32_t exec_status, ring_mode;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	if (trigger != 2) {
		for (int p = 0; p < 3; p++) {
			uint32_t conf = igen_r32(sc, PIPE_CONF(p));

			if (conf & PIPE_CONF_ENABLE) {
				device_printf(sc->dev,
				    "gt_status: REFUSE: pipe %c is active"
				    " (PIPE_CONF=0x%08x).  Render-engine MMIO"
				    " reads during live scanout may wedge the"
				    " iGPU.  Write 2 to override.\n",
				    'A' + p, conf);
				return (EBUSY);
			}
		}
	}

	error = igen_gt_fw_render_take(sc);
	if (error != 0) {
		device_printf(sc->dev,
		    "gt_status: forcewake render take failed: %d\n", error);
		return (error);
	}

	fw          = igen_r32(sc, FORCEWAKE_RENDER_GEN9_ACK);
	guc         = igen_r32(sc, GUC_STATUS);
	ring_head   = igen_r32(sc, RING_BUFFER_HEAD(RING_BASE_RCS));
	ring_tail   = igen_r32(sc, RING_BUFFER_TAIL(RING_BASE_RCS));
	ring_start  = igen_r32(sc, RING_BUFFER_START(RING_BASE_RCS));
	ring_ctl    = igen_r32(sc, RING_BUFFER_CTL(RING_BASE_RCS));
	ring_mode   = igen_r32(sc, RING_MODE(RING_BASE_RCS));
	exec_status = igen_r32(sc, EXECLIST_STATUS_RCS);

	igen_gt_fw_render_release(sc);

	device_printf(sc->dev,
	    "gt: FW_RENDER_ACK=0x%08x  GUC_STATUS=0x%08x"
	    "  bootrom=0x%02x ucode=0x%03x msg=0x%02x\n",
	    fw, guc,
	    GUC_STATUS_BOOTROM(guc), GUC_STATUS_UCODE(guc),
	    GUC_STATUS_MSG(guc));
	device_printf(sc->dev,
	    "gt: RCS ring HEAD=0x%08x TAIL=0x%08x START=0x%08x CTL=0x%08x\n",
	    ring_head, ring_tail, ring_start, ring_ctl);
	device_printf(sc->dev,
	    "gt: RCS MODE=0x%08x  EXECLIST_STATUS=0x%08x\n",
	    ring_mode, exec_status);
	device_printf(sc->dev,
	    "gt: ring buffer %s,  CTL_LENGTH=%u pages\n",
	    (ring_ctl & 1u) ? "ENABLED" : "disabled",
	    (ring_ctl >> 12) & 0x1ffu);
	return (0);
}

void
igen_gt_register_sysctls(struct igen_softc *sc)
{
	struct sysctl_oid_list *children;

	children = SYSCTL_CHILDREN(sc->re_sysctl_tree);

	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "gt_status",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_gt_status, "I",
	    "write 1 to take forcewake render, dump RCS+GuC state, release."
	    " 2 to force-override pipe-active refusal");
}
