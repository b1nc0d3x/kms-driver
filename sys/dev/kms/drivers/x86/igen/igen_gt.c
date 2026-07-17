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
/*
 * gen9 forcewake domain register map (BSpec / Linux i915 canonical):
 *   RENDER  request 0xa278, ack 0x0d84
 *   MEDIA   request 0xa270, ack 0x0d88
 *   BLITTER request 0xa188, ack 0x130044
 * Prior version of this table had RENDER at 0xa188 (that's actually
 * BLITTER) — the ack poll happened to succeed spuriously but the
 * render domain stayed asleep, so any subsequent ELSP write into the
 * render MMIO range (0x2230+) stalled the MMIO bus.  Live-verified
 * 4× wedge on Kabylake fbsdx86 2026-07-16 before this fix.
 */
#define	FORCEWAKE_RENDER_GEN9		0x0000a278
#define	FORCEWAKE_RENDER_GEN9_ACK	0x00000d84
#define	FORCEWAKE_MEDIA_GEN9		0x0000a270
#define	FORCEWAKE_MEDIA_GEN9_ACK	0x00000d88
#define	FORCEWAKE_BLITTER_GEN9		0x0000a188
#define	FORCEWAKE_BLITTER_GEN9_ACK	0x00130044
#define	  FW_REQ_SET(bit)		(((1u << (bit)) << 16) | (1u << (bit)))
#define	  FW_REQ_CLR(bit)		((1u << (bit)) << 16)

/* ---- Render Command Streamer (RCS, engine 0) --------------------------- */

#define	RING_BASE_RCS			0x00002000
#define	RING_BUFFER_TAIL(b)		((b) + 0x30)
#define	RING_BUFFER_HEAD(b)		((b) + 0x34)
#define	RING_BUFFER_START(b)		((b) + 0x38)
#define	RING_BUFFER_CTL(b)		((b) + 0x3c)
#define	RING_HWS_PGA(b)			((b) + 0x80)
#define	RING_MI_MODE(b)			((b) + 0x9c)
#define	RING_HWSTAM(b)			((b) + 0x98)
#define	RING_MODE(b)			((b) + 0x29c)

#define	STOP_RING			(1u << 8)
#define	GFX_RUN_LIST_ENABLE		(1u << 15)

/*
 * "Masked" writes: high 16 bits are a write-enable mask over the low 16.
 * Used for RING_MODE_GEN7 / RING_MI_MODE / RING_CONTEXT_CONTROL and other
 * registers where individual bits can be toggled without a read-modify-
 * write.  Matches Linux i915 _MASKED_BIT_ENABLE / _MASKED_BIT_DISABLE.
 */
#define	_MASKED_BIT_ENABLE(v)		((((v) & 0xffffu) << 16) | ((v) & 0xffffu))
#define	_MASKED_BIT_DISABLE(v)		(((v) & 0xffffu) << 16)

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
/* LRC layout — page 0 is reserved header, page 1+ is register state. */
#define	IGT_LRC_HEADER_PAGES		1
#define	IGT_LRC_STATE_PAGES		1
#define	IGT_LRC_TOTAL_PAGES		(IGT_LRC_HEADER_PAGES + \
					    IGT_LRC_STATE_PAGES + 2)
#define	IGT_LRC_STATE_OFFSET		(IGT_LRC_HEADER_PAGES * PAGE_SIZE)

#define	IGT_GTT_FIRST_RCS_RING		0x80800
#define	IGT_RCS_RING_PAGES		2
#define	IGT_GTT_FIRST_RCS_LRC		(IGT_GTT_FIRST_RCS_RING + \
					    IGT_RCS_RING_PAGES)
#define	IGT_RCS_LRC_PAGES		IGT_LRC_TOTAL_PAGES
#define	IGT_GTT_FIRST_RCS_HWSP		(IGT_GTT_FIRST_RCS_LRC + \
					    IGT_RCS_LRC_PAGES)
#define	IGT_RCS_HWSP_PAGES		1

/*
 * HWSP (Hardware Status Page) layout — intel_ringbuffer.h v4.19:
 *   dword index 0x10..0x1b : Context Status Buffer (CSB), 6 entries of
 *                            two dwords each: (status, ctx_id).
 *   dword index 0x1f       : CSB write pointer (bits 0..7 = write ptr,
 *                            bits 8..15 = read ptr the CS observed).
 */
#define	I915_HWS_CSB_BUF0_INDEX		0x10
#define	I915_HWS_CSB_WRITE_INDEX	0x1f
#define	GEN8_CSB_ENTRIES		6

/*
 * gen8+ context-switch buffer status flags (bits 0-7 of the CSB status
 * dword; upper bits carry seqno/timestamp fields).  Names match Linux
 * i915 intel_engine_types.h.
 */
#define	GEN8_CTX_STATUS_IDLE_ACTIVE	(1u << 0)
#define	GEN8_CTX_STATUS_PREEMPTED	(1u << 1)
#define	GEN8_CTX_STATUS_ELEMENT_SWITCH	(1u << 2)
#define	GEN8_CTX_STATUS_ACTIVE_IDLE	(1u << 3)
#define	GEN8_CTX_STATUS_COMPLETE	(1u << 4)
#define	GEN8_CTX_STATUS_LITE_RESTORE	(1u << 15)

/*
 * Context descriptor field layout, gen 8/9 (i915 v4.19 intel_lrc.c comment
 * lines 200-207, i915_reg.h GEN8_CTX_*, i915_gem_context.c
 * default_desc_template):
 *
 *   bits 63..53:  group ID (0)
 *   bits 52..32:  HW context ID (GEN8_CTX_ID_WIDTH = 21)
 *   bits 31..12:  LRCA — GGTT byte address of the LRC *state* page,
 *                 page-aligned so low 12 bits are naturally 0
 *   bits 11..0:   control flags (GEN8_CTX_*)
 *
 * The LRCA points at the STATE page (buffer_start + LRC_HEADER_PAGES *
 * PAGE_SIZE), NOT the buffer start — the CS-restore machinery reads
 * (offset, value) pairs directly from that page.
 */
#define	GEN8_CTX_VALID			(1u <<  0)
#define	GEN8_CTX_FORCE_PD_RESTORE	(1u <<  1)
#define	GEN8_CTX_FORCE_RESTORE		(1u <<  2)
#define	GEN8_CTX_L3LLC_COHERENT		(1u <<  5)	/* gen8 only */
#define	GEN8_CTX_PRIVILEGE		(1u <<  8)
#define	GEN8_CTX_ADDRESSING_MODE_SHIFT	3
#define	INTEL_LEGACY_32B_CONTEXT	0u
#define	INTEL_LEGACY_64B_CONTEXT	3u
#define	GEN8_CTX_ID_SHIFT		32

static uint64_t
igen_gt_context_desc(uint32_t lrc_buffer_ggtt, uint32_t hw_id)
{
	/*
	 * gen9 execlists require INTEL_LEGACY_64B_CONTEXT (addressing mode
	 * value 3, sets bits 3 and 4 of ctx_desc).  LEGACY_32B (value 0) is
	 * a BDW/BXT-only mode; gen9 silently mis-loads the LRC when it sees
	 * addressing mode 0 and emits an IDLE_TO_ACTIVE CSB with ctx_id=0
	 * (null context) instead of running our LRC.  Live-verified
	 * 2026-07-16 on Kabylake fbsdx86 pre/post-fix.
	 * Linux i915 hardcodes LEGACY_64B for all gen8..gen10.
	 */
	uint32_t flags = GEN8_CTX_VALID | GEN8_CTX_PRIVILEGE |
	    (INTEL_LEGACY_64B_CONTEXT << GEN8_CTX_ADDRESSING_MODE_SHIFT);
	uint32_t lrca = (lrc_buffer_ggtt + IGT_LRC_STATE_OFFSET) & 0xfffff000u;

	return ((uint64_t)flags) |
	    ((uint64_t)lrca) |
	    (((uint64_t)hw_id) << GEN8_CTX_ID_SHIFT);
}

/* MI commands (gen 9 instruction set). */
#define	MI_NOOP				0x00000000u
#define	MI_BATCH_BUFFER_END		0x05000000u
#define	MI_LOAD_REGISTER_IMM(n)		(0x11000000u | (2u * (n) - 1))
#define	MI_LRI_FORCE_POSTED		(1u << 12)
/*
 * MI_BATCH_BUFFER_START — gen 8+ 48-bit-address form:
 *   dword 0: 0x18800001  (op 0x31 << 23 | length 1)
 *            bit 8 (0x100) = ADDRESS_SPACE_PPGTT (else GGTT)
 *   dword 1: address bits 31:0
 *   dword 2: address bits 63:32
 * We stay in GGTT for now (aliasing PPGTT model): bit 8 clear.
 */
/*
 * MI_BATCH_BUFFER_START (gen8+).  Header format:
 *   [31:29] = 0 (MI command type)
 *   [28:23] = 0x31 (opcode)
 *   [22]    = 0 (first level; 1 = second-level chained)
 *   [8]     = Address Space Indicator: 0=PPGTT, 1=GGTT
 *   [7:0]   = DWORD_LENGTH (= dwords - 2) = 1 -> 3 total dwords
 *
 * Without bit 8 set, the engine tries to translate `batch_ggtt`
 * through PPGTT.  Our privileged context has no PPGTT tables loaded
 * (all PDPs are zero in the LRC), so the translation returns scratch
 * and the batch silently no-ops.  Live-verified 2026-07-16 on KBL:
 * CSB reports COMPLETE, but MI_STORE_DWORD_IMM inside the target BO
 * never fires and target[0x100] stays at its userspace-poisoned value.
 * Setting bit 8 makes the engine treat `batch_ggtt` as GGTT (which is
 * where our softpin bind wrote real PTEs).
 */
#define	MI_BATCH_BUFFER_START_GEN8	(0x18800001u | (1u << 8))

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
/*
 * GEN8_R_PWR_CLK_STATE MMIO offset: 0x20C8 per Linux i915 v4.19
 * `intel_lrc_reg.h`.  Prior value 0xa09c was a completely different
 * register (a global GT PM reg), so the LRC's LRI to CTX_R_PWR_CLK_STATE
 * was writing RPCS bits into the wrong MMIO -- render engine's slice/EU
 * enable state never got applied.  Live-verified 2026-07-16 as one of
 * several stacked bugs preventing MI_STORE_DWORD_IMM from ever firing
 * despite CSB reporting COMPLETE.
 */
#define	GEN8_R_PWR_CLK_STATE		0x000020c8

/*
 * CONTEXT_CONTROL value: first-submit init.  Bit meanings (RING_CONTEXT_
 * CONTROL, see intel_lrc.h):
 *   bit 0 = ENGINE_CTX_RESTORE_INHIBIT
 *   bit 1 = RS_CTX_ENABLE (resource streamer — gen>=8, HAS_RESOURCE_STREAMER)
 *   bit 2 = ENGINE_CTX_SAVE_INHIBIT
 *   bit 3 = INHIBIT_SYN_CTX_SWITCH
 *
 * Per Linux v4.19 `populate_lr_context` for a first-submit LRC WITHOUT
 * a golden default state (i.e., fresh alloc, no prior save area):
 *   _MASKED_BIT_DISABLE(RESTORE_INHIBIT | SAVE_INHIBIT) = 0x00050000
 *   _MASKED_BIT_ENABLE (INHIBIT_SYN_CTX_SWITCH)         = 0x00080008
 *   _MASKED_BIT_ENABLE (RESTORE_INHIBIT)  ← for no-golden path
 *                                                       = 0x00010001
 * OR                                                     = 0x000D0009
 *
 * KBL gen9 has NO resource streamer (HAS_RESOURCE_STREAMER returns
 * false), so bit 1 (RS_CTX_ENABLE) is NOT masked and NOT set.
 * Prior value 0x000f000b had bit 1 set, which is a Broadwell/HSW mask
 * that gen9 doesn't honor and might cause the engine to reject the
 * context load.  Live-verified 2026-07-16 as one of several bugs.
 */
#define	RCS_CONTEXT_CONTROL_VALUE	0x000D0009u
#define	RING_BB_PPGTT			(1u << 5)	/* BB_STATE bit */
#define	RING_VALID			0x1u

/*
 * GEN9 R_PWR_CLK_STATE (RPCS) — 0xa09c.
 *
 * A zero write leaves slices/subslices/EUs power-gated from RC6 idle
 * (or from whatever state PM firmware left them in), and the CS engine
 * cannot execute even a single MI_NOOP because the execution units for
 * this context are gated off.  Live-verified 2026-07-16: with RPCS=0
 * in the LRC, ELSP submit lands, engine STATUS transitions 0x1 -> 0x302
 * (idle after failed load), CSB write_ptr stays 0, HEAD never advances.
 *
 * Fix: encode a valid (slice=1, subslice=3, EU=8/8) config with the
 * ENABLE + SS_CNT_ENABLE bits set.  Matches Linux `intel_sseu_make_rpcs`
 * for a KBL GT2 (HD 630) part.  For SKL GT2 the value is the same;
 * BDW/HSW predate the LRC RPCS field (Block 2 didn't exist yet).
 *
 * Bit layout (BSpec / Linux `intel_sseu_regs.h`):
 *   [31]     ENABLE            — apply this RPCS write
 *   [25..27] SLICE_CNT         — slice count (1..N), enabled by bit 18
 *   [18]     S_CNT_ENABLE      — set slice_cnt field
 *   [16..18] SUBSLICE_CNT      — subslice count (1..N), enabled by bit 11
 *   [11]     SS_CNT_ENABLE     — set subslice_cnt field
 *   [4..7]   EU_MAX_PER_SS     — max EUs per subslice (all 8 = 0x8)
 *   [0..3]   EU_MIN_PER_SS     — min EUs per subslice (all 8 = 0x8)
 *
 * Only S_CNT_ENABLE is skipped for gen9 GT2 parts (has_slice_pg=false
 * per Linux — slices are permanently on, can't gate).
 */
#define	  RPCS_ENABLE			(1u << 31)
#define	  RPCS_EU_MAX_SHIFT		4
#define	  RPCS_EU_MIN_SHIFT		0
/*
 * Per Linux `intel_lrc.c:make_rpcs` on KBL GT2:
 *   has_slice_pg    = false  (only 1 slice, can't power-gate)
 *   has_subslice_pg = false  (KBL is not GEN9_LP)
 *   has_eu_pg       = true   (8 EUs/subslice > 2)
 *
 * So make_rpcs takes ONLY the EU-pg branch: min_eus << MIN_SHIFT |
 * max_eus << MAX_SHIFT | RPCS_ENABLE.  The SS_CNT_ENABLE +
 * SLICE_CNT_ENABLE bits stay CLEAR.  Prior 0x80030888 spuriously set
 * SS_CNT_ENABLE + subslice count — engine may treat this as invalid
 * request and reject the LRC.  Live-verified 2026-07-16.
 */
#define	GEN9_KBL_GT2_RPCS_VALUE						\
	(RPCS_ENABLE |							\
	 (8u << RPCS_EU_MAX_SHIFT) |					\
	 (8u << RPCS_EU_MIN_SHIFT))
/* Evaluates to 0x80000088. */

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

/*
 * LRC layout defs live upstream (near GTT slot reservations) because
 * igen_gt_context_desc needs IGT_LRC_STATE_OFFSET when composing LRCA.
 */

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
	/*
	 * BB_STATE — Linux v4.19 sets `RING_BB_PPGTT` (bit 5) unconditionally.
	 * Prior version cleared this on the theory that we don't have PPGTT
	 * PDPs set up, but that's wrong: bit 5 is a "batch-buffer uses
	 * PPGTT-format addressing" indicator, not "PPGTT walker enabled" —
	 * and gen9 execlists REQUIRE it in the LRC image regardless of
	 * PPGTT setup.  Live-verified 2026-07-16 as one of several stacked
	 * bugs preventing batch execution.
	 */
	CTX_REG(regs, CTX_BB_STATE,            RCS_MMIO_BB_STATE,
	    RING_BB_PPGTT);
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
	 * Block 2 — R_PWR_CLK_STATE (RPCS).  Writing 0 leaves the context's
	 * slices/subslices/EUs power-gated, so even a no-op batch can't run
	 * (live-verified 2026-07-16 on KBL GT2: STATUS 0x1 -> 0x302, HEAD
	 * stayed 0, CSB never advanced).  Encode the KBL GT2 topology
	 * (1 slice, 3 subslices, 8 EUs each) via the RPCS_ENABLE-gated
	 * masked-field format defined above.
	 */
	regs[CTX_LRI_HEADER_2] = MI_LOAD_REGISTER_IMM(1);
	CTX_REG(regs, CTX_R_PWR_CLK_STATE,     GEN8_R_PWR_CLK_STATE,
	    GEN9_KBL_GT2_RPCS_VALUE);

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
 * Compose a ring that jumps to `batch_ggtt` (a batch buffer already in
 * GGTT) and terminates.  Used by EXECBUFFER2 to dispatch userspace
 * batches through the shared RCS ring.
 */
static size_t
igen_gt_compose_batch_bb_start(uint32_t *ring, uint64_t batch_ggtt)
{
	uint32_t *p = ring;

	*p++ = MI_BATCH_BUFFER_START_GEN8;
	*p++ = (uint32_t)(batch_ggtt & 0xffffffffu);
	*p++ = (uint32_t)(batch_ggtt >> 32);
	*p++ = MI_BATCH_BUFFER_END;
	return ((p - ring) * sizeof(uint32_t));
}

/*
 * Allocate `n_pages` of contig kernel memory and bind n consecutive
 * GGTT entries starting at gtt_first_idx pointing at the n physical
 * pages.  Returns the GGTT byte offset of page 0 on success, 0 on
 * failure.  Caller owns the VA + must call igen_gt_free_pages_gtt.
 */
/*
 * gen8+ Private Page Attribute Table (PPAT) setup.
 *
 * GGTT PTE bits 3, 4, 7 index into an 8-entry PPAT that defines the
 * cache mode for that mapping.  Hardware defaults are documented
 * (BSpec) but firmware may leave them at implementation-specific
 * values, so Linux i915 always reprograms PPAT to a known set at
 * driver init.  We do the same.
 *
 * Entry 0 = WB | LLC (default cached, CPU-coherent).  This is what
 * every engine page (ring / LRC / HWSP) with PTE bits 3=4=7=0 uses.
 *
 * If PPAT[0] were left at an UC value by firmware, GPU CS reads via
 * our engine PTEs would bypass CPU cache — the CS instruction
 * prefetcher would see uninitialized or stale data even though we
 * wrote the ring content from CPU-side (M_ZERO'd via contigmalloc,
 * then filled by compose_batch_bb_start).  That is a plausible
 * explanation for the "engine loads context, CSB reports COMPLETE,
 * but MI_STORE_DWORD_IMM never fires" pattern.
 */
#define	GEN8_PRIVATE_PAT_LO		0x000040e0
#define	GEN8_PRIVATE_PAT_HI		0x000040e4
#define	GEN8_PPAT_WB			(3u << 0)
#define	GEN8_PPAT_WT			(2u << 0)
#define	GEN8_PPAT_WC			(1u << 0)
#define	GEN8_PPAT_UC			(0u << 0)
#define	GEN8_PPAT_LLC			(1u << 2)
#define	GEN8_PPAT_LLCELLC		(2u << 2)
#define	GEN8_PPAT_AGE(x)		((uint64_t)(x) << 4)
#define	GEN8_PPAT(i, x)			(((uint64_t)(x)) << ((i) * 8))

static void
igen_gt_init_ppat(struct igen_softc *sc)
{
	uint64_t pat =
	    GEN8_PPAT(0, GEN8_PPAT_WB | GEN8_PPAT_LLC) |
	    GEN8_PPAT(1, GEN8_PPAT_WC | GEN8_PPAT_LLCELLC) |
	    GEN8_PPAT(2, GEN8_PPAT_WT | GEN8_PPAT_LLCELLC) |
	    GEN8_PPAT(3, GEN8_PPAT_UC) |
	    GEN8_PPAT(4, GEN8_PPAT_WB | GEN8_PPAT_LLCELLC | GEN8_PPAT_AGE(0)) |
	    GEN8_PPAT(5, GEN8_PPAT_WB | GEN8_PPAT_LLCELLC | GEN8_PPAT_AGE(1)) |
	    GEN8_PPAT(6, GEN8_PPAT_WB | GEN8_PPAT_LLCELLC | GEN8_PPAT_AGE(2)) |
	    GEN8_PPAT(7, GEN8_PPAT_WB | GEN8_PPAT_LLCELLC | GEN8_PPAT_AGE(3));

	igen_w32(sc, GEN8_PRIVATE_PAT_LO, (uint32_t)pat);
	igen_w32(sc, GEN8_PRIVATE_PAT_HI, (uint32_t)(pat >> 32));
	(void)igen_r32(sc, GEN8_PRIVATE_PAT_LO);	/* posting read */
	device_printf(sc->dev,
	    "gt: PPAT programmed LO=0x%08x HI=0x%08x\n",
	    (uint32_t)pat, (uint32_t)(pat >> 32));
}

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
		/*
		 * PTE bits: 0 = VALID, 1 = WRITEABLE, 7 = PPAT index bit 2.
		 * Setting bit 7 selects PPAT entry 4 which we program above
		 * to WB | LLC+eLLC | Age 0 — coherent with CPU cache.
		 * Explicitly using index 4 (vs default index 0) makes cache
		 * behavior independent of whatever firmware left PPAT[0] as.
		 */
		uint64_t pte = ((pa + (uint64_t)i * PAGE_SIZE) & ~0xfffULL) |
		    0x1ULL | 0x2ULL | 0x80ULL;
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
 * Enable execlist submission on the RCS engine.  Must run before any
 * ELSP write — without RING_MODE.GFX_RUN_LIST_ENABLE set the engine's
 * execlist state machine is idle and ELSP writes silently drop.
 *
 * Mirrors i915 v4.19 intel_lrc.c:enable_execlists (lines 1734-1761).
 * Caller must be holding forcewake on the render domain.
 */
/*
 * gen9 engine-init workaround MMIO offsets.  All are per Linux
 * v4.19 `gen9_gt_workarounds_apply` + `kbl_gt_workarounds_apply`
 * (drivers/gpu/drm/i915/intel_workarounds.c).  Ranked by likelihood
 * of gating ELSP execution on Kabylake GT2.
 */
#define	GEN7_FF_SLICE_CS_CHICKEN1	0x000020e0
#define	  GEN9_FFSC_PERCTX_PREEMPT_CTRL	(1u << 14)
#define	GEN9_CSFE_CHICKEN1_RCS		0x000020d4
#define	  GEN9_PREEMPT_GPGPU_SYNC_SWITCH_DISABLE	(1u << 2)
#define	GEN8_L3SQCREG4			0x0000b118
#define	  GEN8_LQSC_FLUSH_COHERENT_LINES	(1u << 21)
#define	GAM_ECOCHK			0x00004090
#define	  BDW_DISABLE_HDC_INVALIDATION	(1u << 25)
#define	  ECOCHK_DIS_TLB		(1u << 8)

/*
 * Apply the load-bearing gen9 workaround MMIO writes.  Per Linux
 * source study + agent analysis:
 *
 * 1. FF_SLICE_CS_CHICKEN1.PERCTX_PREEMPT_CTRL — highest signal.
 *    Without this, engine accepts the context, sits in an ambiguous
 *    preempt state, and complete-notifies without advancing HEAD.
 *    Exactly our symptom.
 * 2. CSFE_CHICKEN1_RCS.GPGPU_SYNC_SWITCH_DISABLE — CS front-end
 *    can drop the first batch after context switch without this.
 * 3. L3SQCREG4.FLUSH_COHERENT_LINES (RMW) — L3 SQC won't propagate
 *    stores from render pipe to LLC without this bit.  Directly
 *    explains "store executes in EU but never reaches memory".
 * 4. GAM_ECOCHK (RMW) — GT-side address translation quirks that
 *    can silently kill batch fetches.
 *
 * All are single-write masked-bit-enable except 3 and 4 which are
 * RMW.  Idempotent — safe to run before every submit.
 */
static void
igen_gt_apply_gen9_workarounds(struct igen_softc *sc)
{
	uint32_t v;

	igen_w32(sc, GEN7_FF_SLICE_CS_CHICKEN1,
	    _MASKED_BIT_ENABLE(GEN9_FFSC_PERCTX_PREEMPT_CTRL));
	igen_w32(sc, GEN9_CSFE_CHICKEN1_RCS,
	    _MASKED_BIT_ENABLE(GEN9_PREEMPT_GPGPU_SYNC_SWITCH_DISABLE));

	v = igen_r32(sc, GEN8_L3SQCREG4);
	igen_w32(sc, GEN8_L3SQCREG4, v | GEN8_LQSC_FLUSH_COHERENT_LINES);

	v = igen_r32(sc, GAM_ECOCHK);
	igen_w32(sc, GAM_ECOCHK,
	    v | BDW_DISABLE_HDC_INVALIDATION | ECOCHK_DIS_TLB);

	(void)igen_r32(sc, GAM_ECOCHK);	/* posting read */

	device_printf(sc->dev, "gt: gen9 workarounds applied\n");
}

/*
 * Per-engine reset for RCS on gen8+.  Mirrors Linux i915 v4.19
 * gen8_engine_reset_prepare + gen6_hw_domain_reset + engine complete
 * (drivers/gpu/drm/i915/intel_uncore.c).  Sequence:
 *   1. RING_RESET_CTL <- REQUEST_RESET (masked bit 0).
 *   2. Poll RING_RESET_CTL for READY_TO_RESET (bit 1).
 *   3. GEN6_GDRST <- GRDOM_RENDER (bit 1).
 *   4. Poll GEN6_GDRST bit 1 to CLEAR (auto-clears when reset done).
 *   5. RING_RESET_CTL <- clear REQUEST_RESET (masked bit 0).
 *
 * After this, engine state is reset (HEAD, TAIL, MODE, HWSTAM all
 * zero).  Caller MUST re-run enable_execlists to reprogram them.
 * Forcewake must be held throughout.
 *
 * Rationale: engine may be inheriting a stuck state from firmware /
 * BIOS handoff that lets ELSP loads report COMPLETE without actually
 * walking the ring.  A per-engine reset gives us a known-clean
 * starting point before every submit.  Linux does this on hang or
 * on driver load; we do it on every submit as a diagnostic.
 */
#define	RING_RESET_CTL(base)		((base) + 0xd0)
#define	  RESET_CTL_REQUEST_RESET	(1u << 0)
#define	  RESET_CTL_READY_TO_RESET	(1u << 1)
#define	GEN6_GDRST			0x0000941c
#define	  GEN6_GRDOM_RENDER		(1u << 1)

static void
igen_gt_reset_rcs(struct igen_softc *sc)
{
	int spin;

	/* Step 1: request per-engine reset. */
	igen_w32(sc, RING_RESET_CTL(RING_BASE_RCS),
	    _MASKED_BIT_ENABLE(RESET_CTL_REQUEST_RESET));

	/* Step 2: wait for READY_TO_RESET (bit 1).  Linux caps at 700us. */
	for (spin = 0; spin < 700; spin++) {
		uint32_t v = igen_r32(sc, RING_RESET_CTL(RING_BASE_RCS));
		if (v & RESET_CTL_READY_TO_RESET)
			break;
		DELAY(1);
	}

	/* Step 3: trigger the reset via GDRST. */
	igen_w32(sc, GEN6_GDRST, GEN6_GRDOM_RENDER);

	/* Step 4: wait for GDRST bit 1 to auto-clear.  Linux caps at 500us. */
	for (spin = 0; spin < 500; spin++) {
		uint32_t v = igen_r32(sc, GEN6_GDRST);
		if ((v & GEN6_GRDOM_RENDER) == 0)
			break;
		DELAY(1);
	}

	/* Step 5: release the reset request. */
	igen_w32(sc, RING_RESET_CTL(RING_BASE_RCS),
	    _MASKED_BIT_DISABLE(RESET_CTL_REQUEST_RESET));
	(void)igen_r32(sc, RING_RESET_CTL(RING_BASE_RCS));	/* posting */

	device_printf(sc->dev,
	    "gt: RCS engine reset done (spin=%d/500)\n", spin);
}

static void
igen_gt_enable_execlists(struct igen_softc *sc, uint32_t hwsp_ggtt)
{
	/*
	 * Per-engine reset before every submit.  Gives us a known-clean
	 * starting state regardless of what firmware / prior submits left
	 * behind.  Must run under forcewake.  After reset, RING_MODE,
	 * RING_HWSTAM, RING_HWS_PGA are all cleared — the writes below
	 * restore them.
	 */
	igen_gt_reset_rcs(sc);

	/*
	 * PPAT setup — must run before ELSP so the CS's PTE lookups for
	 * ring / LRC / HWSP resolve to a well-defined LLC-coherent cache
	 * mode.  Safe to re-run; PRIVATE_PAT MMIO writes are idempotent.
	 */
	igen_gt_init_ppat(sc);

	/*
	 * gen9 workaround MMIO writes — Linux does these at engine
	 * init before the first ELSP submit.  Some (like the CS
	 * chicken bits) are load-bearing for the engine to actually
	 * walk HEAD..TAIL instead of context-load-then-idle.
	 */
	igen_gt_apply_gen9_workarounds(sc);

	igen_w32(sc, RING_HWSTAM(RING_BASE_RCS), 0xffffffffu);
	igen_w32(sc, RING_MODE(RING_BASE_RCS),
	    _MASKED_BIT_ENABLE(GFX_RUN_LIST_ENABLE));
	igen_w32(sc, RING_MI_MODE(RING_BASE_RCS),
	    _MASKED_BIT_DISABLE(STOP_RING));
	igen_w32(sc, RING_HWS_PGA(RING_BASE_RCS), hwsp_ggtt);
	(void)igen_r32(sc, RING_HWS_PGA(RING_BASE_RCS));	/* posting read */
}

/*
 * Core ELSP submit: engine already programmed and forcewake held.
 * Writes the context descriptor into ELSP for RCS port[0], polls
 * RING_HEAD until it reaches `tail`, dumps CSB.  Returns 0 if the
 * engine consumed the ring (HEAD==TAIL) or ETIMEDOUT otherwise.
 *
 * Caller owns everything: forcewake, ring/LRC/HWSP allocation, and
 * the enable_execlists call.  Cache coherency of ring / LRC / HWSP is
 * a shared caller responsibility — see wbinvd() call in the sysctl /
 * EXECBUFFER2 entry points.  See PTE-caching comment on
 * igen_gt_alloc_pages_gtt for why this matters on gen9.
 */
static int
igen_gt_submit_lrc(struct igen_softc *sc, uint64_t ctx_desc, uint32_t tail,
    void *hwsp_va)
{
	uint32_t head_before, status_before, mode_before, mi_mode;
	uint32_t head_after, status_after;
	int spin;

	head_before   = igen_r32(sc, RING_BUFFER_HEAD(RING_BASE_RCS));
	status_before = igen_r32(sc, EXECLIST_STATUS_RCS);
	mode_before   = igen_r32(sc, RING_MODE(RING_BASE_RCS));
	mi_mode       = igen_r32(sc, RING_MI_MODE(RING_BASE_RCS));

	device_printf(sc->dev,
	    "gt: pre-submit HEAD=0x%08x STATUS=0x%08x MODE=0x%08x"
	    " MI_MODE=0x%08x ctx_desc=0x%016llx\n",
	    head_before, status_before, mode_before, mi_mode,
	    (unsigned long long)ctx_desc);

	/*
	 * Push every dirty CPU cache line to RAM before the engine reads
	 * ring / LRC / HWSP / user-batch pages via GGTT.  Our GGTT PTEs
	 * have no caching-mode bits set, which on gen9 = Uncached, so the
	 * engine bypasses CPU cache entirely.  Without wbinvd, dirty
	 * cache lines that hold our composed BB_START are NOT visible to
	 * the engine — it reads zeros (the M_ZERO'd contigmalloc page)
	 * and executes them as MI_NOOPs to TAIL.
	 *
	 * TODO: replace with per-page pmap_invalidate_cache_range once
	 * the caller path threads through the ring/LRC/user-BO KVAs.  For
	 * now, blanket wbinvd is correct if expensive.
	 */
	wbinvd();

	/*
	 * ELSP write sequence: for n=num_ports; n--:
	 *     writel(upper_32(port[n].desc), submit_reg);
	 *     writel(lower_32(port[n].desc), submit_reg);
	 * num_ports=2; port[1] empty (both dwords 0), then port[0].
	 */
	igen_w32(sc, ELSP_RCS, 0);				/* port[1] hi */
	igen_w32(sc, ELSP_RCS, 0);				/* port[1] lo */
	igen_w32(sc, ELSP_RCS, (uint32_t)(ctx_desc >> 32));	/* port[0] hi */
	igen_w32(sc, ELSP_RCS,
	    (uint32_t)(ctx_desc & 0xffffffffu));		/* port[0] lo */

	/* Poll RING_HEAD until it reaches TAIL.  Timeout 100 ms. */
	head_after = head_before;
	for (spin = 0; spin < 10000; spin++) {
		head_after = igen_r32(sc, RING_BUFFER_HEAD(RING_BASE_RCS));
		if ((head_after & 0x1fffffu) == tail)
			break;
		/*
		 * Alternative completion signal: after the engine switches
		 * OUR context out and goes idle, MMIO RING_HEAD does NOT
		 * necessarily hold the current pointer — the ring pointer is
		 * saved back into the LRC's RING_HEAD field.  Trust CSB
		 * instead: if any entry shows (ACTIVE_IDLE | COMPLETE) with
		 * our hw_id, the batch executed even if HEAD MMIO reads 0.
		 */
		{
			volatile uint32_t *hws = (volatile uint32_t *)hwsp_va;
			uint32_t hw_id = (uint32_t)(ctx_desc >>
			    GEN8_CTX_ID_SHIFT);
			bool done = false;
			for (int i = 0; i < GEN8_CSB_ENTRIES; i++) {
				uint32_t s = hws[I915_HWS_CSB_BUF0_INDEX +
				    i * 2];
				uint32_t c = hws[I915_HWS_CSB_BUF0_INDEX +
				    i * 2 + 1];
				if (c == hw_id && (s & (GEN8_CTX_STATUS_ACTIVE_IDLE |
				    GEN8_CTX_STATUS_COMPLETE)) ==
				    (GEN8_CTX_STATUS_ACTIVE_IDLE |
				     GEN8_CTX_STATUS_COMPLETE)) {
					done = true;
					break;
				}
			}
			if (done)
				break;
		}
		DELAY(10);
	}
	status_after = igen_r32(sc, EXECLIST_STATUS_RCS);

	device_printf(sc->dev,
	    "gt: post-submit HEAD=0x%08x TAIL=%u STATUS=0x%08x after %d us\n",
	    head_after, tail, status_after, spin * 10);

	bool head_ok = ((head_after & 0x1fffffu) == tail);
	bool csb_ok = false;
	uint32_t hw_id = (uint32_t)(ctx_desc >> GEN8_CTX_ID_SHIFT);
	{
		volatile uint32_t *hws = (volatile uint32_t *)hwsp_va;
		uint32_t wptr = hws[I915_HWS_CSB_WRITE_INDEX];

		for (int i = 0; i < GEN8_CSB_ENTRIES; i++) {
			uint32_t s = hws[I915_HWS_CSB_BUF0_INDEX + i * 2];
			uint32_t c = hws[I915_HWS_CSB_BUF0_INDEX + i * 2 + 1];
			if (c == hw_id && (s & (GEN8_CTX_STATUS_ACTIVE_IDLE |
			    GEN8_CTX_STATUS_COMPLETE)) ==
			    (GEN8_CTX_STATUS_ACTIVE_IDLE |
			     GEN8_CTX_STATUS_COMPLETE)) {
				csb_ok = true;
				break;
			}
		}

		if (head_ok)
			device_printf(sc->dev,
			    "gt: HEAD == TAIL — engine executed the ring.\n");
		else if (csb_ok)
			device_printf(sc->dev,
			    "gt: CSB shows ACTIVE_IDLE|COMPLETE for hw_id=%u —"
			    " engine executed the ring (HEAD MMIO stale).\n",
			    hw_id);
		else if (head_after != head_before)
			device_printf(sc->dev,
			    "gt: HEAD advanced from 0x%08x to 0x%08x but didn't"
			    " reach TAIL=%u — batch partially consumed.\n",
			    head_before, head_after, tail);
		else
			device_printf(sc->dev,
			    "gt: HEAD did not advance and CSB has no completion"
			    " for hw_id=%u — engine did not run.\n", hw_id);

		device_printf(sc->dev,
		    "gt: CSB write_ptr=0x%08x  (bits 0..7 = wptr,"
		    " 8..15 = engine-observed rptr)\n", wptr);
		for (int i = 0; i < GEN8_CSB_ENTRIES; i++) {
			uint32_t sts = hws[I915_HWS_CSB_BUF0_INDEX + i * 2];
			uint32_t cid = hws[I915_HWS_CSB_BUF0_INDEX + i * 2 + 1];

			device_printf(sc->dev,
			    "gt: CSB[%d] status=0x%08x ctx_id=0x%08x\n",
			    i, sts, cid);
		}
	}

	return ((head_ok || csb_ok) ? 0 : ETIMEDOUT);
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
	ctx_desc  = igen_gt_context_desc(lrc_ggtt, /* hw_id */ 1);

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
	    "gt: ctx_desc = 0x%016llx"
	    "  (LRCA=0x%08x | hw_id=1 | VALID | PRIVILEGE | LEGACY_32B)\n",
	    (unsigned long long)ctx_desc,
	    lrc_ggtt + (uint32_t)IGT_LRC_STATE_OFFSET);
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
 * hazard as gt_status / phy_scan_bc); write 2 to override.
 *
 * Sequence per i915 v4.19 (intel_lrc.c enable_execlists +
 * execlists_submit_ports):
 *
 *   1. Allocate ring (2p) + LRC (4p) + HWSP (1p).  Compose batch and
 *      register-state image.
 *   2. Take forcewake render.
 *   3. Program engine: RING_HWSTAM = ~0, RING_MODE = GFX_RUN_LIST_ENABLE
 *      masked-set, RING_MI_MODE = STOP_RING masked-clear, RING_HWS_PGA =
 *      HWSP GGTT.  Without RING_MODE.GFX_RUN_LIST_ENABLE the engine's
 *      execlist state machine is idle and ELSP writes drop silently.
 *   4. Write descriptor pair to ELSP: (port[1]_hi=0, port[1]_lo=0,
 *      port[0]_hi=ctx_desc>>32, port[0]_lo=ctx_desc&0xffffffff).  This
 *      matches v4.19 write_desc() — high dword first, low dword second,
 *      per port, loop from port[num-1] down to port[0].
 *   5. Poll RING_HEAD until it equals the ring TAIL we composed into the
 *      LRC.  The CS advances HEAD as it retires commands.
 *      (The BSpec "completed" signal is a CSB entry in HWSP with
 *      GEN8_CTX_STATUS_COMPLETED_MASK set; HEAD==TAIL is a sufficient
 *      first-fire smoke check that doesn't require a CSB parser yet.)
 *   6. Dump the raw CSB slots + write pointer for post-mortem.
 *   7. Release forcewake, free everything.
 */
static int
igen_sysctl_gt_first_batch_submit(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	void *ring_va = NULL, *lrc_va = NULL, *hwsp_va = NULL;
	vm_paddr_t ring_pa = 0, lrc_pa = 0, hwsp_pa = 0;
	uint32_t ring_ggtt = 0, lrc_ggtt = 0, hwsp_ggtt = 0;
	uint64_t ctx_desc;
	size_t batch_len;
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
	hwsp_ggtt = igen_gt_alloc_pages_gtt(sc, IGT_GTT_FIRST_RCS_HWSP,
	    IGT_RCS_HWSP_PAGES, &hwsp_va, &hwsp_pa);
	if (hwsp_ggtt == 0) {
		igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_LRC,
		    IGT_RCS_LRC_PAGES, lrc_va);
		igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_RING,
		    IGT_RCS_RING_PAGES, ring_va);
		return (ENOMEM);
	}

	batch_len = igen_gt_compose_batch_noop(ring_va);
	(void)igen_gt_compose_lrc((uint8_t *)lrc_va, ring_ggtt,
	    IGT_RCS_RING_PAGES * PAGE_SIZE, batch_len);
	ctx_desc = igen_gt_context_desc(lrc_ggtt, /* hw_id */ 1);

	error = igen_gt_fw_render_take(sc);
	if (error != 0) {
		igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_HWSP,
		    IGT_RCS_HWSP_PAGES, hwsp_va);
		igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_LRC,
		    IGT_RCS_LRC_PAGES, lrc_va);
		igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_RING,
		    IGT_RCS_RING_PAGES, ring_va);
		return (error);
	}

	igen_gt_enable_execlists(sc, hwsp_ggtt);
	(void)igen_gt_submit_lrc(sc, ctx_desc, (uint32_t)batch_len, hwsp_va);

	igen_gt_fw_render_release(sc);
	igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_HWSP, IGT_RCS_HWSP_PAGES,
	    hwsp_va);
	igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_LRC, IGT_RCS_LRC_PAGES,
	    lrc_va);
	igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_RING, IGT_RCS_RING_PAGES,
	    ring_va);
	return (0);
}

/*
 * Public entry point for EXECBUFFER2.  Allocates a private ring +
 * LRC + HWSP, composes a ring that JUMPs to the caller-supplied
 * `batch_ggtt` and returns, then live-submits via ELSP.  Refuses if
 * any pipe is scanning (wedge risk shared with the sysctl smoke
 * test) unless force==true.  Returns 0 on HEAD==TAIL, error otherwise.
 *
 * The caller is responsible for binding the user's batch BO pages
 * into GGTT at `batch_ggtt` BEFORE calling this — softpin address
 * management lives above us.
 */
int
igen_gt_submit_user_batch(struct igen_softc *sc, uint64_t batch_ggtt,
    bool force)
{
	void *ring_va = NULL, *lrc_va = NULL, *hwsp_va = NULL;
	vm_paddr_t ring_pa = 0, lrc_pa = 0, hwsp_pa = 0;
	uint32_t ring_ggtt = 0, lrc_ggtt = 0, hwsp_ggtt = 0;
	uint64_t ctx_desc;
	size_t batch_len;
	int error;

	if (!force) {
		for (int p = 0; p < 3; p++) {
			uint32_t conf = igen_r32(sc, PIPE_CONF(p));

			if (conf & PIPE_CONF_ENABLE)
				return (EBUSY);
		}
	}

	ring_ggtt = igen_gt_alloc_pages_gtt(sc, IGT_GTT_FIRST_RCS_RING,
	    IGT_RCS_RING_PAGES, &ring_va, &ring_pa);
	if (ring_ggtt == 0)
		return (ENOMEM);
	lrc_ggtt = igen_gt_alloc_pages_gtt(sc, IGT_GTT_FIRST_RCS_LRC,
	    IGT_RCS_LRC_PAGES, &lrc_va, &lrc_pa);
	if (lrc_ggtt == 0) {
		error = ENOMEM;
		goto out_ring;
	}
	hwsp_ggtt = igen_gt_alloc_pages_gtt(sc, IGT_GTT_FIRST_RCS_HWSP,
	    IGT_RCS_HWSP_PAGES, &hwsp_va, &hwsp_pa);
	if (hwsp_ggtt == 0) {
		error = ENOMEM;
		goto out_lrc;
	}

	batch_len = igen_gt_compose_batch_bb_start((uint32_t *)ring_va,
	    batch_ggtt);
	(void)igen_gt_compose_lrc((uint8_t *)lrc_va, ring_ggtt,
	    IGT_RCS_RING_PAGES * PAGE_SIZE, batch_len);
	ctx_desc = igen_gt_context_desc(lrc_ggtt, /* hw_id */ 1);

	error = igen_gt_fw_render_take(sc);
	if (error != 0)
		goto out_hwsp;

	igen_gt_enable_execlists(sc, hwsp_ggtt);
	error = igen_gt_submit_lrc(sc, ctx_desc, (uint32_t)batch_len, hwsp_va);

	igen_gt_fw_render_release(sc);

out_hwsp:
	igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_HWSP,
	    IGT_RCS_HWSP_PAGES, hwsp_va);
out_lrc:
	igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_LRC,
	    IGT_RCS_LRC_PAGES, lrc_va);
out_ring:
	igen_gt_free_pages_gtt(sc, IGT_GTT_FIRST_RCS_RING,
	    IGT_RCS_RING_PAGES, ring_va);
	return (error);
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
