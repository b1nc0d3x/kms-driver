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

/* GTT entry decoded bits — see igen_gtt.c for the full PTE format. */
#define	IGT_GTT_FIRST_RCS_RING		0x80800	/* well above scanout slots */
#define	IGT_GTT_FIRST_RCS_LRC		0x80808

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

/*
 * Compose the minimum-viable LRC for RCS on gen 9.  The "default LRC"
 * i915 ships is ~1 KiB of register save image starting at offset 0x150
 * (after a fixed header), but for our first execution we can get away
 * with a much shorter image that only sets RING_CTL / RING_HEAD /
 * RING_TAIL / RING_START / BB_HEAD / BB_TAIL.  The engine reads from
 * RING_START + RING_HEAD and executes until HEAD == TAIL, so as long
 * as those four registers carry sane values the rest defaults to zero
 * and the engine completes the no-op batch.
 *
 * Returns the byte length written.  Caller owns the buffer.
 */
static size_t
igen_gt_compose_lrc(uint32_t *lrc, uint32_t ring_ggtt, uint32_t ring_len_bytes)
{
	uint32_t *p = lrc;

	/*
	 * MI_LOAD_REGISTER_IMM with count=6 — six (offset, value) pairs.
	 * The offsets are RCS-relative (added to RING_BASE_RCS = 0x2000
	 * by the engine; we encode the engine-local offset).
	 */
	*p++ = MI_LOAD_REGISTER_IMM(6);
	*p++ = 0x244;	/* CONTEXT_CONTROL: inhibit_syn=1, save=1 */
	*p++ = (1u << 0) | (1u << 18);
	*p++ = 0x34;	/* RING_BUFFER_HEAD */
	*p++ = 0;
	*p++ = 0x30;	/* RING_BUFFER_TAIL */
	*p++ = 0;
	*p++ = 0x38;	/* RING_BUFFER_START */
	*p++ = ring_ggtt;
	*p++ = 0x3c;	/* RING_BUFFER_CTL */
	*p++ = ((ring_len_bytes - PAGE_SIZE) & 0x1ff000u) | 1u;
	*p++ = 0x2c;	/* BB_HEAD_U (zero high bits of batch start) */
	*p++ = 0;
	*p++ = MI_NOOP;

	return ((p - lrc) * sizeof(uint32_t));
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
 * Allocate one page of contig kernel memory and bind it into the
 * driver's GGTT at the chosen index.  Returns the GGTT byte offset
 * (== the address the engine will see) on success, 0 on failure.
 */
static uint32_t
igen_gt_alloc_page_gtt(struct igen_softc *sc, uint32_t gtt_idx,
    void **out_va, vm_paddr_t *out_pa)
{
	void *va;
	vm_paddr_t pa;
	uint64_t pte;

	va = contigmalloc(PAGE_SIZE, M_KMS, M_WAITOK | M_ZERO,
	    0, ~(vm_paddr_t)0, PAGE_SIZE, 0);
	if (va == NULL)
		return (0);
	pa = pmap_kextract((vm_offset_t)va);
	pte = (pa & ~0xfffULL) | 0x1ULL | 0x2ULL; /* VALID | WRITEABLE */
	igen_gtt_write(sc, gtt_idx, pte);
	*out_va = va;
	*out_pa = pa;
	return (gtt_idx * PAGE_SIZE);
}

static void
igen_gt_free_page_gtt(struct igen_softc *sc, uint32_t gtt_idx, void *va)
{
	igen_gtt_write(sc, gtt_idx, 0);
	if (va != NULL)
		contigfree(va, PAGE_SIZE, M_KMS);
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

	ring_ggtt = igen_gt_alloc_page_gtt(sc, IGT_GTT_FIRST_RCS_RING,
	    &ring_va, &ring_pa);
	if (ring_ggtt == 0) {
		device_printf(sc->dev,
		    "gt_first_batch_dry: ring alloc failed\n");
		return (ENOMEM);
	}
	lrc_ggtt = igen_gt_alloc_page_gtt(sc, IGT_GTT_FIRST_RCS_LRC,
	    &lrc_va, &lrc_pa);
	if (lrc_ggtt == 0) {
		igen_gt_free_page_gtt(sc, IGT_GTT_FIRST_RCS_RING, ring_va);
		device_printf(sc->dev,
		    "gt_first_batch_dry: lrc alloc failed\n");
		return (ENOMEM);
	}

	batch_len = igen_gt_compose_batch_noop(ring_va);
	lrc_len   = igen_gt_compose_lrc(lrc_va, ring_ggtt, PAGE_SIZE);
	ctx_desc  = igen_gt_context_desc(lrc_ggtt);

	device_printf(sc->dev,
	    "gt: ring  GGTT=0x%08x  PA=0x%llx  VA=%p  batch_len=%zu\n",
	    ring_ggtt, (unsigned long long)ring_pa, ring_va, batch_len);
	device_printf(sc->dev,
	    "gt: ring[0..1] = 0x%08x 0x%08x\n",
	    ((uint32_t *)ring_va)[0], ((uint32_t *)ring_va)[1]);
	device_printf(sc->dev,
	    "gt: lrc   GGTT=0x%08x  PA=0x%llx  VA=%p  image_len=%zu\n",
	    lrc_ggtt, (unsigned long long)lrc_pa, lrc_va, lrc_len);
	for (size_t i = 0; i < lrc_len / sizeof(uint32_t); i += 4) {
		device_printf(sc->dev,
		    "gt: lrc[%2zu] 0x%08x 0x%08x 0x%08x 0x%08x\n",
		    i,
		    ((uint32_t *)lrc_va)[i + 0],
		    ((uint32_t *)lrc_va)[i + 1],
		    ((uint32_t *)lrc_va)[i + 2],
		    ((uint32_t *)lrc_va)[i + 3]);
	}
	device_printf(sc->dev,
	    "gt: ctx_desc = 0x%016llx (LRCA=0x%08x | VALID | PRIVILEGE"
	    " | RCS)\n",
	    (unsigned long long)ctx_desc, lrc_ggtt);
	device_printf(sc->dev,
	    "gt: DRY-RUN — nothing written to ELSP, LRC not handed to HW\n");

	igen_gt_free_page_gtt(sc, IGT_GTT_FIRST_RCS_LRC, lrc_va);
	igen_gt_free_page_gtt(sc, IGT_GTT_FIRST_RCS_RING, ring_va);
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

	ring_ggtt = igen_gt_alloc_page_gtt(sc, IGT_GTT_FIRST_RCS_RING,
	    &ring_va, &ring_pa);
	if (ring_ggtt == 0)
		return (ENOMEM);
	lrc_ggtt = igen_gt_alloc_page_gtt(sc, IGT_GTT_FIRST_RCS_LRC,
	    &lrc_va, &lrc_pa);
	if (lrc_ggtt == 0) {
		igen_gt_free_page_gtt(sc, IGT_GTT_FIRST_RCS_RING, ring_va);
		return (ENOMEM);
	}

	(void)igen_gt_compose_batch_noop(ring_va);
	(void)igen_gt_compose_lrc(lrc_va, ring_ggtt, PAGE_SIZE);
	ctx_desc = igen_gt_context_desc(lrc_ggtt);

	error = igen_gt_fw_render_take(sc);
	if (error != 0) {
		igen_gt_free_page_gtt(sc, IGT_GTT_FIRST_RCS_LRC, lrc_va);
		igen_gt_free_page_gtt(sc, IGT_GTT_FIRST_RCS_RING, ring_va);
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
	igen_gt_free_page_gtt(sc, IGT_GTT_FIRST_RCS_LRC, lrc_va);
	igen_gt_free_page_gtt(sc, IGT_GTT_FIRST_RCS_RING, ring_va);
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
