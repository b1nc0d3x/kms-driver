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
#include <sys/malloc.h>
#include <sys/sysctl.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_param.h>

#include "igen_internal.h"

MALLOC_DECLARE(M_KMS);

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
#define	FORCEWAKE_RENDER_GEN9_ACK	0x00000d84
#define	FORCEWAKE_MEDIA_GEN9		0x0000a270
#define	FORCEWAKE_MEDIA_GEN9_ACK	0x00000d88
#define	FORCEWAKE_BLITTER_GEN9		0x0000a188	/* shared with render */
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

/* ============================================================ *
 *  Phase 2 — execlist submission scaffold (DRY-RUN by default)
 * ============================================================
 *
 * The shape of "make the engine execute something" on gen 9 RCS is:
 *
 *   1) Allocate two contiguous, page-aligned regions in kernel memory:
 *        - LRC (Logical Ring Context) — 1 page of register save state
 *          that the silicon DMAs in/out on context switch.
 *        - RING — 4 KiB ring buffer the engine reads command DWORDs
 *          from.  Address is GGTT-mapped, but for Phase 2 we install
 *          a 1:1 GGTT mapping of the physical page so the GPU sees
 *          the same address the CPU does (Phase 4 will switch to
 *          PPGTT and per-process address spaces).
 *
 *   2) Install GGTT page-table entries pointing each region's physical
 *          page at a chosen GTT slot we own.  PFN | VALID | WRITEABLE.
 *
 *   3) Compose the LRC: i915 calls this "default LRC" image — bytes
 *          0x00..0xff are MI_LOAD_REGISTER_IMM headers and (offset,
 *          value) pairs that restore engine state on switch-in.  The
 *          mandatory ones for RCS are RING_CTL / RING_HEAD / RING_TAIL
 *          / RING_START / RING_BB_PER_CTX_PTR / RING_BBSTATE / 3D mode
 *          / CS_CHICKEN_BITS / CTX_TIMESTAMP.
 *
 *   4) Compose the batch we want the engine to execute.  Phase 3's
 *          minimum is two DWORDs: MI_NOOP followed by MI_BATCH_BUFFER_
 *          END.  Write into the ring buffer at offset 0; update LRC's
 *          RING_TAIL field to 8 so the engine knows where to stop.
 *
 *   5) Compose the context descriptor: 64-bit field {LRC_GGTT_addr |
 *          GEN8_CTX_VALID | engine_class | privilege}, written as
 *          two 32-bit halves to ELSP_RCS in (low, high) order.  After
 *          two contexts (or one context + 0 padding) the engine pops
 *          them onto its 2-deep submission queue.
 *
 *   6) Poll EXECLIST_STATUS.  The "completed" bit goes high once the
 *          context retires.  Read RING_HEAD; it should equal the new
 *          TAIL value if the engine ran the no-op batch.
 *
 * The DRY-RUN sysctl `gt_first_batch_dry` performs steps (1)-(4) plus
 * a printout of the composed bytes — allocation succeeds or fails, the
 * LRC image is computed, the ring contents are computed, but NOTHING
 * is written to ELSP and the GGTT is NOT touched.  That gives us a
 * way to validate the byte-level setup against BSpec / a known-good
 * Linux capture before risking a wedge.  `gt_first_batch_submit`
 * (write 2) is the live path; refused if any pipe is active.
 */

/*
 * GTT slot reservations for the GT scaffold.  Well above scanout (the
 * user-FB cache tops out at 0xa0000 - 0xa0fff and the test_fb at
 * 0x80000 - 0x807ff).  Ring is 2 pages (i915 minimum); LRC is 1 page.
 */
#define	IGT_GTT_FIRST_RCS_RING		0x80800
#define	IGT_RCS_RING_PAGES		2
#define	IGT_GTT_FIRST_RCS_LRC		(IGT_GTT_FIRST_RCS_RING + \
					    IGT_RCS_RING_PAGES)
#define	IGT_RCS_LRC_PAGES		IGT_LRC_TOTAL_PAGES

/*
 * Context descriptor field layout for gen 9 (i915 v4.19 intel_lrc.h):
 *   bit 63..32: bits 31..0 of LRCA (LRC GGTT address, page-aligned)
 *   bit 31..16: reserved
 *   bit 15..12: engine_class (RCS = 0)
 *   bit 11:     PRIVILEGE
 *   bit 10:     IRQ_DISABLE
 *   bit 9..8:   addressing_mode (0 = ADDRESSING_MODE_LEGACY_64B)
 *   bit 1:      FAULT_DISABLE
 *   bit 0:      VALID
 */
#define	GEN8_CTX_VALID			(1ull << 0)
#define	GEN8_CTX_PRIVILEGE		(1ull << 11)
#define	GEN8_CTX_ADDR_MODE_LEGACY_64B	(0ull << 8)
#define	GEN8_CTX_ENGINE_CLASS_RCS	(0ull << 12)

static uint64_t
igen_gt_context_desc(uint64_t lrca_ggtt)
{
	return (GEN8_CTX_VALID |
	    GEN8_CTX_PRIVILEGE |
	    GEN8_CTX_ADDR_MODE_LEGACY_64B |
	    GEN8_CTX_ENGINE_CLASS_RCS |
	    ((lrca_ggtt & 0xfffff000ull) << 32));
}

/* MI commands (gen 9 instruction set). */
#define	MI_NOOP				0x00000000u
#define	MI_BATCH_BUFFER_END		0x05000000u
#define	MI_LOAD_REGISTER_IMM(n)		(0x11000000u | (2u * (n) - 1))
#define	MI_LRI_FORCE_POSTED		(1u << 12)

/*
 * Absolute MMIO offsets (RING_BASE_RCS added).  MI_LOAD_REGISTER_IMM
 * writes at whatever absolute offset the LRI stores — the CS restore
 * machinery does NOT add mmio_base for you (that was the "engine-local"
 * misreading of populate_lr_context that shipped in the first pass).
 * Sourced from i915 v4.19 intel_ringbuffer.h RING_*(base) macros.
 */
#define	RCS_MMIO_CONTEXT_CONTROL	(RING_BASE_RCS + 0x244)
#define	RCS_MMIO_RING_HEAD		(RING_BASE_RCS + 0x034)
#define	RCS_MMIO_RING_TAIL		(RING_BASE_RCS + 0x030)
#define	RCS_MMIO_RING_START		(RING_BASE_RCS + 0x038)
#define	RCS_MMIO_RING_CTL		(RING_BASE_RCS + 0x03c)
#define	RCS_MMIO_BB_HEAD_U		(RING_BASE_RCS + 0x168)
#define	RCS_MMIO_BB_HEAD_L		(RING_BASE_RCS + 0x140)
#define	RCS_MMIO_BB_STATE		(RING_BASE_RCS + 0x110)
#define	RCS_MMIO_SBB_HEAD_U		(RING_BASE_RCS + 0x16c)
#define	RCS_MMIO_SBB_HEAD_L		(RING_BASE_RCS + 0x144)
#define	RCS_MMIO_SBB_STATE		(RING_BASE_RCS + 0x114)
#define	RCS_MMIO_BB_PER_CTX_PTR		(RING_BASE_RCS + 0x1c0)
#define	RCS_MMIO_INDIRECT_CTX		(RING_BASE_RCS + 0x1bc)
#define	RCS_MMIO_INDIRECT_CTX_OFFSET	(RING_BASE_RCS + 0x1c8)
#define	RCS_MMIO_CTX_TIMESTAMP		(RING_BASE_RCS + 0x3a8)
#define	RCS_MMIO_PDP_LDW(n)		(RING_BASE_RCS + 0x270 + (n) * 8)
#define	RCS_MMIO_PDP_UDW(n)		(RING_BASE_RCS + 0x270 + (n) * 8 + 4)
#define	GEN8_R_PWR_CLK_STATE		0x0000a09c

/*
 * CONTEXT_CONTROL value per i915 v4.19 execlists_init_reg_state:
 *   _MASKED_BIT_DISABLE(CTX_CTRL_ENGINE_CTX_RESTORE_INHIBIT |
 *                       CTX_CTRL_ENGINE_CTX_SAVE_INHIBIT)      = 0x00050000
 *   _MASKED_BIT_ENABLE (CTX_CTRL_INHIBIT_SYN_CTX_SWITCH |
 *                       CTX_CTRL_RS_CTX_ENABLE)                = 0x000a000a
 * OR                                                          = 0x000f000a
 * Bits used (see intel_lrc.h): RESTORE_INHIBIT bit 0, RS_CTX_ENABLE
 * bit 1, SAVE_INHIBIT bit 2, INHIBIT_SYN_CTX_SWITCH bit 3.
 */
#define	RCS_CONTEXT_CONTROL_VALUE	0x000f000au
#define	RING_BB_PPGTT			(1u << 5)	/* BB_STATE bit */
#define	RING_VALID			0x1u

/*
 * Positions inside the state page, DWORD-indexed.  Match Linux v4.19
 * intel_lrc_reg.h exactly — the CS-restore machinery + subsequent
 * mid-flight updates (Linux's regs[CTX_RING_TAIL + 1] = tail; etc.)
 * assume these packed positions.
 *
 * regs[0] is left as MI_NOOP (zero-init) so LRI_HEADER_0 lands at
 * regs[1], not regs[0].  The gaps 0x1e..0x20 and 0x34..0x40 are NOOP
 * padding — the engine walks them without side effects but the fixed
 * positions of LRI_HEADER_1 (0x21) and LRI_HEADER_2 (0x41) are what
 * the HW expects.
 */
#define	CTX_LRI_HEADER_0		0x01
#define	CTX_CONTEXT_CONTROL		0x02
#define	CTX_RING_HEAD			0x04
#define	CTX_RING_TAIL			0x06
#define	CTX_RING_BUFFER_START		0x08
#define	CTX_RING_BUFFER_CONTROL		0x0a
#define	CTX_BB_HEAD_U			0x0c
#define	CTX_BB_HEAD_L			0x0e
#define	CTX_BB_STATE			0x10
#define	CTX_SECOND_BB_HEAD_U		0x12
#define	CTX_SECOND_BB_HEAD_L		0x14
#define	CTX_SECOND_BB_STATE		0x16
#define	CTX_BB_PER_CTX_PTR		0x18
#define	CTX_RCS_INDIRECT_CTX		0x1a
#define	CTX_RCS_INDIRECT_CTX_OFFSET	0x1c
#define	CTX_LRI_HEADER_1		0x21
#define	CTX_CTX_TIMESTAMP		0x22
#define	CTX_PDP3_UDW			0x24
#define	CTX_PDP3_LDW			0x26
#define	CTX_PDP2_UDW			0x28
#define	CTX_PDP2_LDW			0x2a
#define	CTX_PDP1_UDW			0x2c
#define	CTX_PDP1_LDW			0x2e
#define	CTX_PDP0_UDW			0x30
#define	CTX_PDP0_LDW			0x32
#define	CTX_LRI_HEADER_2		0x41
#define	CTX_R_PWR_CLK_STATE		0x42
#define	CTX_LRC_STATE_END		0x44	/* one past last populated DWORD */

/* LRC layout — page 0 is reserved header, page 1+ is register state. */
#define	IGT_LRC_HEADER_PAGES		1
#define	IGT_LRC_STATE_PAGES		1
#define	IGT_LRC_TOTAL_PAGES		(IGT_LRC_HEADER_PAGES + \
					    IGT_LRC_STATE_PAGES + 2)
#define	IGT_LRC_STATE_OFFSET		(IGT_LRC_HEADER_PAGES * PAGE_SIZE)

/*
 * Place a (register_mmio_offset, value) pair at the specified DWORD
 * position inside the LRC state image.  Mirrors Linux CTX_REG.
 */
#define	CTX_REG(regs, pos, mmio, val)	do {				\
	(regs)[(pos) + 0] = (mmio);					\
	(regs)[(pos) + 1] = (val);					\
} while (0)

/*
 * Compose the canonical SKL/KBL RCS Logical Ring Context image, matching
 * Linux v4.19 execlists_init_reg_state byte-for-byte.
 *
 * The LRC is laid out as:
 *   page 0 (offset 0x0000)        : engine header (DMA scratch — zero)
 *   page 1 (offset 0x1000)        : register state image (this routine)
 *   page 2+                       : URB / scratch / per-process state
 *                                   (zero is fine for a pure-MI no-op)
 *
 * The register state image consists of three MI_LOAD_REGISTER_IMM blocks
 * at fixed DWORD positions inside page 1.  Positions between them are
 * left as MI_NOOP (zero-init) — the CS-restore machinery walks them
 * without side effects.
 *
 * `lrc_base` is the start of the LRC buffer (offset 0).  The register
 * image starts at lrc_base + IGT_LRC_STATE_OFFSET = 0x1000.
 *
 * Returns the byte length of the register image (i.e. the number of
 * bytes after IGT_LRC_STATE_OFFSET this routine populated, up to and
 * including the last populated DWORD, so callers can dump exactly the
 * meaningful range).
 */
static size_t
igen_gt_compose_lrc(uint8_t *lrc_base, uint32_t ring_ggtt,
    uint32_t ring_len_bytes, uint32_t ring_tail)
{
	uint32_t *regs = (uint32_t *)(lrc_base + IGT_LRC_STATE_OFFSET);
	uint32_t ring_ctl_val =
	    (((ring_len_bytes / PAGE_SIZE) - 1u) << 12) | RING_VALID;

	/* Block 0 — 14 pairs starting at CTX_LRI_HEADER_0 (DWORD 1). */
	regs[CTX_LRI_HEADER_0] = MI_LOAD_REGISTER_IMM(14) |
	    MI_LRI_FORCE_POSTED;
	CTX_REG(regs, CTX_CONTEXT_CONTROL,     RCS_MMIO_CONTEXT_CONTROL,
	    RCS_CONTEXT_CONTROL_VALUE);
	CTX_REG(regs, CTX_RING_HEAD,           RCS_MMIO_RING_HEAD,      0);
	CTX_REG(regs, CTX_RING_TAIL,           RCS_MMIO_RING_TAIL,      ring_tail);
	CTX_REG(regs, CTX_RING_BUFFER_START,   RCS_MMIO_RING_START,     ring_ggtt);
	CTX_REG(regs, CTX_RING_BUFFER_CONTROL, RCS_MMIO_RING_CTL,       ring_ctl_val);
	CTX_REG(regs, CTX_BB_HEAD_U,           RCS_MMIO_BB_HEAD_U,      0);
	CTX_REG(regs, CTX_BB_HEAD_L,           RCS_MMIO_BB_HEAD_L,      0);
	CTX_REG(regs, CTX_BB_STATE,            RCS_MMIO_BB_STATE,       RING_BB_PPGTT);
	CTX_REG(regs, CTX_SECOND_BB_HEAD_U,    RCS_MMIO_SBB_HEAD_U,     0);
	CTX_REG(regs, CTX_SECOND_BB_HEAD_L,    RCS_MMIO_SBB_HEAD_L,     0);
	CTX_REG(regs, CTX_SECOND_BB_STATE,     RCS_MMIO_SBB_STATE,      0);
	CTX_REG(regs, CTX_BB_PER_CTX_PTR,      RCS_MMIO_BB_PER_CTX_PTR, 0);
	CTX_REG(regs, CTX_RCS_INDIRECT_CTX,    RCS_MMIO_INDIRECT_CTX,   0);
	CTX_REG(regs, CTX_RCS_INDIRECT_CTX_OFFSET,
	                                       RCS_MMIO_INDIRECT_CTX_OFFSET, 0);

	/* Gap 0x1e..0x20 stays MI_NOOP. */

	/* Block 1 — CTX_TIMESTAMP + 4 PDP{UDW,LDW} pairs = 9 pairs. */
	regs[CTX_LRI_HEADER_1] = MI_LOAD_REGISTER_IMM(9) |
	    MI_LRI_FORCE_POSTED;
	CTX_REG(regs, CTX_CTX_TIMESTAMP,       RCS_MMIO_CTX_TIMESTAMP,  0);
	CTX_REG(regs, CTX_PDP3_UDW,            RCS_MMIO_PDP_UDW(3),     0);
	CTX_REG(regs, CTX_PDP3_LDW,            RCS_MMIO_PDP_LDW(3),     0);
	CTX_REG(regs, CTX_PDP2_UDW,            RCS_MMIO_PDP_UDW(2),     0);
	CTX_REG(regs, CTX_PDP2_LDW,            RCS_MMIO_PDP_LDW(2),     0);
	CTX_REG(regs, CTX_PDP1_UDW,            RCS_MMIO_PDP_UDW(1),     0);
	CTX_REG(regs, CTX_PDP1_LDW,            RCS_MMIO_PDP_LDW(1),     0);
	CTX_REG(regs, CTX_PDP0_UDW,            RCS_MMIO_PDP_UDW(0),     0);
	CTX_REG(regs, CTX_PDP0_LDW,            RCS_MMIO_PDP_LDW(0),     0);

	/* Gap 0x34..0x40 stays MI_NOOP. */

	/*
	 * Block 2 — R_PWR_CLK_STATE (RPCS).  Leave 0 — engine uses its
	 * boot-time slice/subslice/EU defaults.  Real Mesa workloads will
	 * need make_rpcs() but a no-op MI batch doesn't care.  Note the
	 * absence of MI_LRI_FORCE_POSTED here matches Linux v4.19 (block
	 * 2 was reworked in later kernels but v4.19 leaves it off).
	 */
	regs[CTX_LRI_HEADER_2] = MI_LOAD_REGISTER_IMM(1);
	CTX_REG(regs, CTX_R_PWR_CLK_STATE,     GEN8_R_PWR_CLK_STATE,    0);

	return (CTX_LRC_STATE_END * sizeof(uint32_t));
}

static size_t
igen_gt_compose_batch_noop(uint32_t *ring)
{
	uint32_t *p = ring;

	*p++ = MI_NOOP;
	*p++ = MI_BATCH_BUFFER_END;
	return ((p - ring) * sizeof(uint32_t));
}

/*
 * Allocate `n_pages` of contig kernel memory and bind n consecutive
 * GGTT entries starting at gtt_first_idx pointing at the n physical
 * pages.  Returns the GGTT byte offset of page 0 on success, 0 on
 * failure.  Caller owns the VA + must call igen_gt_free_pages_gtt.
 */
static uint32_t
igen_gt_alloc_pages_gtt(struct igen_softc *sc, uint32_t gtt_first_idx,
    uint32_t n_pages, void **out_va, vm_paddr_t *out_pa)
{
	void *va;
	vm_paddr_t pa;
	size_t len = (size_t)n_pages * PAGE_SIZE;

	va = contigmalloc(len, M_KMS, M_WAITOK | M_ZERO,
	    0, ~(vm_paddr_t)0, PAGE_SIZE, 0);
	if (va == NULL)
		return (0);
	pa = pmap_kextract((vm_offset_t)va);
	for (uint32_t i = 0; i < n_pages; i++) {
		uint64_t pte = ((pa + (uint64_t)i * PAGE_SIZE) & ~0xfffULL) |
		    0x1ULL | 0x2ULL;	/* VALID | WRITEABLE */
		igen_gtt_write(sc, gtt_first_idx + i, pte);
	}
	*out_va = va;
	*out_pa = pa;
	return (gtt_first_idx * PAGE_SIZE);
}

static void
igen_gt_free_pages_gtt(struct igen_softc *sc, uint32_t gtt_first_idx,
    uint32_t n_pages, void *va)
{
	for (uint32_t i = 0; i < n_pages; i++)
		igen_gtt_write(sc, gtt_first_idx + i, 0);
	if (va != NULL)
		contigfree(va, (size_t)n_pages * PAGE_SIZE, M_KMS);
}

/*
 * Dry-run path.  Allocates the same buffers a real submit would use,
 * fills in LRC and ring contents, prints everything, frees, returns.
 * Touches the GGTT for the duration (we don't have a "compute PTE
 * without writing" helper) but writes zeros back before freeing.
 */
static int
igen_sysctl_gt_first_batch_dry(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	void *ring_va = NULL, *lrc_va = NULL;
	vm_paddr_t ring_pa = 0, lrc_pa = 0;
	uint32_t ring_ggtt = 0, lrc_ggtt = 0;
	size_t batch_len, lrc_len;
	uint64_t ctx_desc;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	ring_ggtt = igen_gt_alloc_pages_gtt(sc, IGT_GTT_FIRST_RCS_RING,
	    IGT_RCS_RING_PAGES, &ring_va, &ring_pa);
	if (ring_ggtt == 0) {
		device_printf(sc->dev,
		    "gt_first_batch_dry: ring alloc failed\n");
		return (ENOMEM);
	}
	lrc_ggtt = igen_gt_alloc_pages_gtt(sc, IGT_GTT_FIRST_RCS_LRC,
	    IGT_RCS_LRC_PAGES, &lrc_va, &lrc_pa);
	if (lrc_ggtt == 0) {
		igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_RING,
		    IGT_RCS_RING_PAGES, ring_va);
		device_printf(sc->dev,
		    "gt_first_batch_dry: lrc alloc failed\n");
		return (ENOMEM);
	}

	batch_len = igen_gt_compose_batch_noop(ring_va);
	lrc_len   = igen_gt_compose_lrc((uint8_t *)lrc_va, ring_ggtt,
	    IGT_RCS_RING_PAGES * PAGE_SIZE, batch_len);
	ctx_desc  = igen_gt_context_desc(lrc_ggtt);

	device_printf(sc->dev,
	    "gt: ring  GGTT=0x%08x  PA=0x%llx  VA=%p  batch_len=%zu\n",
	    ring_ggtt, (unsigned long long)ring_pa, ring_va, batch_len);
	device_printf(sc->dev,
	    "gt: ring[0..1] = 0x%08x 0x%08x\n",
	    ((uint32_t *)ring_va)[0], ((uint32_t *)ring_va)[1]);
	device_printf(sc->dev,
	    "gt: lrc   GGTT=0x%08x  PA=0x%llx  VA=%p  total=%u pages"
	    "  state_image_len=%zu\n",
	    lrc_ggtt, (unsigned long long)lrc_pa, lrc_va,
	    IGT_RCS_LRC_PAGES, lrc_len);
	/*
	 * Register state image lives at lrc + IGT_LRC_STATE_OFFSET (0x1000).
	 * Dump only the state image; the header page (offset 0..0xfff) is
	 * uniformly zero.
	 */
	{
		uint32_t *state = (uint32_t *)((uint8_t *)lrc_va +
		    IGT_LRC_STATE_OFFSET);

		for (size_t i = 0; i < lrc_len / sizeof(uint32_t); i += 4) {
			device_printf(sc->dev,
			    "gt: lrc+0x%04zx %08x %08x %08x %08x\n",
			    IGT_LRC_STATE_OFFSET + i * 4,
			    state[i + 0], state[i + 1],
			    state[i + 2], state[i + 3]);
		}
	}
	device_printf(sc->dev,
	    "gt: ctx_desc = 0x%016llx (LRCA=0x%08x | VALID | PRIVILEGE"
	    " | RCS)\n",
	    (unsigned long long)ctx_desc, lrc_ggtt);
	device_printf(sc->dev,
	    "gt: DRY-RUN — nothing written to ELSP, LRC not handed to HW\n");

	igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_LRC, IGT_RCS_LRC_PAGES,
	    lrc_va);
	igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_RING, IGT_RCS_RING_PAGES,
	    ring_va);
	return (0);
}

/*
 * Live submission.  Refuses if any pipe is active (same scanout-wedge
 * hazard as gt_status / phy_scan_bc); write 2 to override.  Sequence
 * mirrors the dry-run, then takes forcewake, writes the context
 * descriptor to ELSP, polls EXECLIST_STATUS for the completed bit,
 * reads back RING_HEAD to confirm advance, releases forcewake, frees.
 */
static int
igen_sysctl_gt_first_batch_submit(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	void *ring_va = NULL, *lrc_va = NULL;
	vm_paddr_t ring_pa = 0, lrc_pa = 0;
	uint32_t ring_ggtt = 0, lrc_ggtt = 0;
	uint64_t ctx_desc;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	if (trigger != 2) {
		for (int p = 0; p < 3; p++) {
			uint32_t conf = igen_r32(sc, PIPE_CONF(p));

			if (conf & PIPE_CONF_ENABLE) {
				device_printf(sc->dev,
				    "gt_first_batch_submit: REFUSE: pipe %c"
				    " active (PIPE_CONF=0x%08x).  Wedge risk."
				    "  Write 2 to override.\n",
				    'A' + p, conf);
				return (EBUSY);
			}
		}
	}

	ring_ggtt = igen_gt_alloc_pages_gtt(sc, IGT_GTT_FIRST_RCS_RING,
	    IGT_RCS_RING_PAGES, &ring_va, &ring_pa);
	if (ring_ggtt == 0)
		return (ENOMEM);
	lrc_ggtt = igen_gt_alloc_pages_gtt(sc, IGT_GTT_FIRST_RCS_LRC,
	    IGT_RCS_LRC_PAGES, &lrc_va, &lrc_pa);
	if (lrc_ggtt == 0) {
		igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_RING,
		    IGT_RCS_RING_PAGES, ring_va);
		return (ENOMEM);
	}

	{
		size_t blen = igen_gt_compose_batch_noop(ring_va);
		(void)igen_gt_compose_lrc((uint8_t *)lrc_va, ring_ggtt,
		    IGT_RCS_RING_PAGES * PAGE_SIZE, blen);
	}
	ctx_desc = igen_gt_context_desc(lrc_ggtt);

	error = igen_gt_fw_render_take(sc);
	if (error != 0) {
		igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_LRC,
		    IGT_RCS_LRC_PAGES, lrc_va);
		igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_RING,
		    IGT_RCS_RING_PAGES, ring_va);
		return (error);
	}

	uint32_t head_before = igen_r32(sc, RING_BUFFER_HEAD(RING_BASE_RCS));
	uint32_t status_before = igen_r32(sc, EXECLIST_STATUS_RCS);
	device_printf(sc->dev,
	    "gt: pre-submit HEAD=0x%08x STATUS=0x%08x\n",
	    head_before, status_before);

	/*
	 * Write descriptor pair to ELSP.  Order matters per BSpec:
	 * the four 32-bit writes form a (ctx1_hi, ctx1_lo, ctx0_hi,
	 * ctx0_lo) pair queued into the engine's two-deep slot.  We
	 * only need ctx0 — pad ctx1 with zeros.
	 */
	igen_w32(sc, ELSP_RCS, 0);
	igen_w32(sc, ELSP_RCS, 0);
	igen_w32(sc, ELSP_RCS, (uint32_t)(ctx_desc >> 32));
	igen_w32(sc, ELSP_RCS, (uint32_t)(ctx_desc & 0xffffffffu));

	/* Poll EXECLIST_STATUS bit 7 (completed) or active bits going low. */
	uint32_t status = 0;
	int spin;
	for (spin = 0; spin < 10000; spin++) {
		status = igen_r32(sc, EXECLIST_STATUS_RCS);
		if (status & (1u << 7))
			break;
		DELAY(10);
	}

	uint32_t head_after = igen_r32(sc, RING_BUFFER_HEAD(RING_BASE_RCS));

	device_printf(sc->dev,
	    "gt: post-submit HEAD=0x%08x STATUS=0x%08x after %d us\n",
	    head_after, status, spin * 10);
	if (head_after != head_before)
		device_printf(sc->dev,
		    "gt: HEAD advanced by %u dwords — engine executed.\n",
		    (head_after - head_before) / 4);
	else
		device_printf(sc->dev,
		    "gt: HEAD did not advance — engine did not run.  "
		    "LRC layout almost certainly needs BSpec verification.\n");

	igen_gt_fw_render_release(sc);
	igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_LRC, IGT_RCS_LRC_PAGES,
	    lrc_va);
	igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_RING, IGT_RCS_RING_PAGES,
	    ring_va);
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
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "gt_first_batch_dry",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_gt_first_batch_dry, "I",
	    "write 1 to allocate + compose LRC/ring/batch + context desc,"
	    " print bytes, free.  No ELSP write.  Safe with pipe active");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "gt_first_batch_submit",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_gt_first_batch_submit, "I",
	    "write 1 to live-submit first no-op batch to RCS via ELSP."
	    " 2 to force-override pipe-active refusal (wedge risk)");
}
