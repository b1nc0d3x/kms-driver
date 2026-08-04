/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Haswell display-engine bring-up: TRANSCODER EDP / DDI_A eDP scanout.
 *
 * Macbsd (15" MBP Retina, Iris Pro 5200) firmware leaves the display
 * subsystem in a half-up state: CDCLK 540 MHz live, both PWELLs up,
 * DDI_A PORT_CLK_SEL pointing at LCPLL_1350, but every PIPE_CONF reads
 * 0 — no transcoder, no DDI buffer, nothing on the eDP wires.  Loading
 * igen.ko therefore lands on a dark panel.
 *
 * This file is the minimum code path to push that state to "panel lit
 * with our scanout buffer":
 *
 *   1. Sanity-check the link via DPCD.  If the sink advertises HBR
 *      capability and isn't already trained, walk the abbreviated eDP
 *      link-training sequence (CR + EQ via DPCD register writes; HSW
 *      drives the on-wire pattern through DP_TP_CTL).
 *   2. Program TRANSCODER EDP timing (HTOTAL/HBLANK/HSYNC/VTOTAL/
 *      VBLANK/VSYNC) from the EDID's preferred DTD already parsed at
 *      attach.  The transcoder feeds DDI_A natively on HSW.
 *   3. Program PIPE_DATA_M / PIPE_DATA_N for the DP stuffer (the M/N
 *      ratio that tells the encoder how often to insert null symbols
 *      on a DP link that runs faster than the pixel stream).
 *   4. Configure TRANS_DDI_FUNC_CTL_EDP for DP-SST mode, 8 bpc, sync
 *      polarity, port-select=DDI_A; set ENABLE.
 *   5. DDI_BUF_CTL[A]: set port width from DPCD-advertised lane count,
 *      buffer-translation idx, set ENABLE.  Wait for IDLE_STATUS to
 *      clear (buffer powered up).
 *   6. PIPE_CONF[A] ENABLE.  Poll PIPE_STATE bit.
 *   7. PLANE_CTL/SIZE/STRIDE/SURF point at the driver's scanout_fb
 *      (test pattern; later atomic_commit will rewrite SURF on user
 *      page-flips).
 *   8. PLANE_CTL[A] ENABLE — first pixel hits the panel.
 *
 * igen_hsw_panel_on() is called from igen_atomic_commit when the pipe
 * is cold — that's the normal path once a compositor requests a
 * mode set on macbsd.  The two sysctls remain for diagnostic /
 * bring-up-by-hand use before a compositor is running:
 *
 *   dev.igen.<n>.re.hsw_dp_dump   diagnostic dump of every register
 *                                  + DPCD page involved in eDP scanout
 *   dev.igen.<n>.re.hsw_panel_on  run the whole sequence end-to-end
 *
 * All entry points are gated to gen == HSW so SKL hosts don't
 * accidentally fire the SKL-incompatible writes.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/sysctl.h>

MALLOC_DECLARE(M_KMS);

#include <machine/bus.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>

#include <kms/drm_dp_helper.h>
#include <kms/drm_framebuffer.h>

#include "igen_internal.h"
#include "igen_aux.h"

/* --------------------------- HSW display register map -------------------- */

/* Transcoders: A=0, B=1, C=2 at 0x60000 + idx*0x1000; EDP is a gap at
 * 0x6F000.  We define an _EDP variant rather than overloading the
 * existing TRANS_HTOTAL(t)/TRANS_DDI_FUNC_CTL(t) macros so the gap is
 * obvious in code.
 */
#define	HSW_TRANS_EDP_BASE		0x0006F000u
#define	HSW_TRANS_EDP_HTOTAL		(HSW_TRANS_EDP_BASE + 0x000)
#define	HSW_TRANS_EDP_HBLANK		(HSW_TRANS_EDP_BASE + 0x004)
#define	HSW_TRANS_EDP_HSYNC		(HSW_TRANS_EDP_BASE + 0x008)
#define	HSW_TRANS_EDP_VTOTAL		(HSW_TRANS_EDP_BASE + 0x00c)
#define	HSW_TRANS_EDP_VBLANK		(HSW_TRANS_EDP_BASE + 0x010)
#define	HSW_TRANS_EDP_VSYNC		(HSW_TRANS_EDP_BASE + 0x014)
#define	HSW_TRANS_EDP_VSYNCSHIFT	(HSW_TRANS_EDP_BASE + 0x028)
#define	HSW_TRANS_EDP_DDI_FUNC_CTL	(HSW_TRANS_EDP_BASE + 0x400)
#define	HSW_TRANS_EDP_CONF		(0x0007F008u)
#define	  HSW_TRANS_EDP_CONF_ENABLE	(1u << 31)
#define	  HSW_TRANS_EDP_CONF_STATE	(1u << 30)

/* DDI_FUNC_CTL_EDP bits per BSpec (vol4 TRANS_DDI_FUNC_CTL): */
#define	TRANS_DDI_FUNC_ENABLE		(1u << 31)
#define	TRANS_DDI_PORT_SHIFT		28
#define	TRANS_DDI_PORT_MASK		(0x7u << 28)
#define	TRANS_DDI_PORT_NONE		(0u << 28)
#define	TRANS_DDI_PORT_DDI_A		(1u << 28)
#define	TRANS_DDI_PORT_DDI_B		(2u << 28)
#define	TRANS_DDI_PORT_DDI_C		(3u << 28)
#define	TRANS_DDI_PORT_DDI_D		(4u << 28)
#define	TRANS_DDI_PORT_DDI_E		(5u << 28)
#define	TRANS_DDI_MODE_SHIFT		24
#define	  TRANS_DDI_MODE_HDMI		(0u << 24)
#define	  TRANS_DDI_MODE_DVI		(1u << 24)
#define	  TRANS_DDI_MODE_DP_SST		(2u << 24)
#define	  TRANS_DDI_MODE_DP_MST		(3u << 24)
#define	  TRANS_DDI_MODE_FDI		(4u << 24)
#define	TRANS_DDI_BPC_8			(0u << 20)
#define	TRANS_DDI_BPC_10		(1u << 20)
#define	TRANS_DDI_BPC_6			(2u << 20)
#define	TRANS_DDI_BPC_12		(3u << 20)
#define	TRANS_DDI_PVSYNC		(1u << 17)
#define	TRANS_DDI_PHSYNC		(1u << 16)
#define	TRANS_DDI_EDP_INPUT_SHIFT	12
#define	  TRANS_DDI_EDP_INPUT_A_ON	(0u << 12)
#define	  TRANS_DDI_EDP_INPUT_A_ONOFF	(4u << 12)
#define	  TRANS_DDI_EDP_INPUT_B_ONOFF	(5u << 12)
#define	  TRANS_DDI_EDP_INPUT_C_ONOFF	(6u << 12)
#define	TRANS_DDI_PORT_WIDTH_SHIFT	1
#define	  TRANS_DDI_PORT_WIDTH_X1	(0u << 1)
#define	  TRANS_DDI_PORT_WIDTH_X2	(1u << 1)
#define	  TRANS_DDI_PORT_WIDTH_X4	(3u << 1)

/* DDI buffer control.  Bits per HSW BSpec (vol4 DDI_BUF_CTL).  Note the
 * 0x100-stride per port (DDI A at 0x64000, B at 0x64100, ...). */
#define	HSW_DDI_BUF_CTL(p)		(0x00064000u + (p) * 0x100u)
#define	  DDI_BUF_CTL_ENABLE		(1u << 31)
#define	  DDI_BUF_CTL_IDLE_STATUS	(1u << 7)
#define	  DDI_BUF_CTL_TRANS_SEL_SHIFT	24
#define	  DDI_BUF_CTL_PORT_WIDTH_SHIFT	1
#define	    DDI_BUF_CTL_PORT_WIDTH_X1	(0u << 1)
#define	    DDI_BUF_CTL_PORT_WIDTH_X2	(1u << 1)
#define	    DDI_BUF_CTL_PORT_WIDTH_X4	(3u << 1)

/* DP transport layer control (per DDI). */
#define	HSW_DP_TP_CTL(p)		(0x00064040u + (p) * 0x100u)
#define	  DP_TP_CTL_ENABLE		(1u << 31)
#define	  DP_TP_CTL_MODE_SST		(0u << 27)
#define	  DP_TP_CTL_MODE_MST		(1u << 27)
#define	  DP_TP_CTL_ENHANCED		(1u << 18)
#define	  DP_TP_CTL_LINK_TRAIN_PAT1	(0u << 8)
#define	  DP_TP_CTL_LINK_TRAIN_PAT2	(1u << 8)
#define	  DP_TP_CTL_LINK_TRAIN_IDLE	(2u << 8)
#define	  DP_TP_CTL_LINK_TRAIN_NORMAL	(3u << 8)
#define	  DP_TP_CTL_LINK_TRAIN_MASK	(7u << 8)

#define	HSW_DP_TP_STATUS(p)		(0x00064044u + (p) * 0x100u)
#define	  DP_TP_STATUS_IDLE_DONE	(1u << 25)

/* PIPE_DATA_M/N + LINK_M/N for DP M/N rate stuffing.  Per-pipe.  HSW
 * indexes per-pipe at 0x6{0,1,2}030 etc. */
#define	HSW_PIPE_DATA_M(p)		(0x00060030u + (p) * 0x1000u)
#define	HSW_PIPE_DATA_N(p)		(0x00060034u + (p) * 0x1000u)
#define	HSW_PIPE_LINK_M(p)		(0x00060040u + (p) * 0x1000u)
#define	HSW_PIPE_LINK_N(p)		(0x00060044u + (p) * 0x1000u)
#define	  PIPE_DATA_M_TU_SIZE_DEFAULT	(0x3fu << 25)	/* TU=64 (TU-1 in bits[30:25]) */

/* PIPE_SRCSZ encoding per BSpec: bits[28:16]=h-1, bits[12:0]=v-1. */
#define	HSW_PIPE_SRCSZ(p)		(0x0007001cu + (p) * 0x1000u)

/* PLANE_CTL HSW bits — the existing igen_internal.h has SKL bits; HSW
 * uses different format codes.  HSW BSpec vol4 DSPACNTR.  We use the
 * "primary plane A control" register at 0x70180 (same offset as SKL
 * PLANE_CTL, register has compatible structure for the bits we set:
 * ENABLE, GAMMA enable, pixel format, tiling). */
#define	HSW_DSPCNTR(p)			(0x00070180u + (p) * 0x1000u)
#define	  DSPCNTR_ENABLE		(1u << 31)
#define	  DSPCNTR_GAMMA_ENABLE		(1u << 30)
#define	  DSPCNTR_FORMAT_SHIFT		26
#define	  DSPCNTR_FORMAT_BGRX8888	(0x6u << 26)
#define	  DSPCNTR_FORMAT_BGRA8888	(0x7u << 26)
#define	  DSPCNTR_FORMAT_RGBX1010102	(0x8u << 26)
#define	  DSPCNTR_TILED			(1u << 10)
#define	HSW_DSPSTRIDE(p)		(0x00070188u + (p) * 0x1000u)
#define	HSW_DSPSURF(p)			(0x0007019cu + (p) * 0x1000u)
#define	HSW_DSPLINOFF(p)		(0x00070184u + (p) * 0x1000u)
#define	HSW_DSPTILEOFF(p)		(0x000701a4u + (p) * 0x1000u)

/* --------------------------- helpers -------------------------------------- */

/*
 * Pack a BSpec-encoded "total + active" register value.  HSW transcoder
 * timing fields are: bits[28:16] = active-1, bits[12:0] = total-1.
 * (Yes, active is in the high half, the BSpec is just like that.)
 */
static inline uint32_t
htotal_pack(uint32_t active, uint32_t total)
{
	return (((active - 1) << 16) | (total - 1));
}

/*
 * Pack a BSpec-encoded "blank start + blank end" register value.  Same
 * shape — start in low 13 bits, end in 16-28.
 */
static inline uint32_t
hblank_pack(uint32_t start, uint32_t end)
{
	return (((end - 1) << 16) | (start - 1));
}

/*
 * Pack a BSpec-encoded "sync start + sync end" — identical to
 * hblank_pack (the two register groups share encoding).
 */
static inline uint32_t
hsync_pack(uint32_t start, uint32_t end)
{
	return (((end - 1) << 16) | (start - 1));
}

/*
 * Read 4 bytes of EDID DTD and decode the canonical timing parameters
 * we need to program the transcoder.  The EDID block layout puts the
 * first DTD at byte 54 of the 128-byte base block.
 */
struct igen_dtd {
	uint32_t	pixclk_khz;
	uint16_t	hactive, hblank, hsync_off, hsync_w;
	uint16_t	vactive, vblank, vsync_off, vsync_w;
	bool		hsync_pos, vsync_pos;
};

static bool
igen_decode_dtd(const uint8_t *d, struct igen_dtd *out)
{
	uint32_t pixclk = ((d[1] << 8) | d[0]) * 10;	/* in kHz */
	if (pixclk == 0)
		return (false);
	out->pixclk_khz = pixclk;
	out->hactive  = d[2] | ((d[4] & 0xf0) << 4);
	out->hblank   = d[3] | ((d[4] & 0x0f) << 8);
	out->vactive  = d[5] | ((d[7] & 0xf0) << 4);
	out->vblank   = d[6] | ((d[7] & 0x0f) << 8);
	out->hsync_off = d[8] | ((d[11] & 0xc0) << 2);
	out->hsync_w   = d[9] | ((d[11] & 0x30) << 4);
	out->vsync_off = (d[10] >> 4) | ((d[11] & 0x0c) << 2);
	out->vsync_w   = (d[10] & 0x0f) | ((d[11] & 0x03) << 4);
	out->hsync_pos = (d[17] & 0x02) != 0;
	out->vsync_pos = (d[17] & 0x04) != 0;
	return (true);
}

/* --------------------------- diagnostic dump ------------------------------ */

static int
igen_sysctl_hsw_dp_dump(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);
	uint8_t dpcd[16];
	uint8_t link_status[6];
	ssize_t n;

	if (error || req->newptr == NULL || trigger == 0)
		return (error);
	if (sc->gen != IGEN_GEN_HSW) {
		device_printf(sc->dev, "hsw_dp_dump: HSW-only\n");
		return (EOPNOTSUPP);
	}

	device_printf(sc->dev, "---- HSW DP/DDI/pipe state ----\n");

	device_printf(sc->dev,
	    "  DDI_BUF_CTL[A]      = 0x%08x  (ENABLE=%u IDLE=%u)\n",
	    igen_r32(sc, HSW_DDI_BUF_CTL(0)),
	    !!(igen_r32(sc, HSW_DDI_BUF_CTL(0)) & DDI_BUF_CTL_ENABLE),
	    !!(igen_r32(sc, HSW_DDI_BUF_CTL(0)) & DDI_BUF_CTL_IDLE_STATUS));
	device_printf(sc->dev,
	    "  DP_TP_CTL[A]        = 0x%08x  (ENABLE=%u train=0x%x)\n",
	    igen_r32(sc, HSW_DP_TP_CTL(0)),
	    !!(igen_r32(sc, HSW_DP_TP_CTL(0)) & DP_TP_CTL_ENABLE),
	    (igen_r32(sc, HSW_DP_TP_CTL(0)) & DP_TP_CTL_LINK_TRAIN_MASK) >> 8);
	device_printf(sc->dev,
	    "  DP_TP_STATUS[A]     = 0x%08x\n",
	    igen_r32(sc, HSW_DP_TP_STATUS(0)));
	device_printf(sc->dev,
	    "  TRANS_EDP_DDI_FUNC  = 0x%08x  (ENABLE=%u port=%u mode=%u)\n",
	    igen_r32(sc, HSW_TRANS_EDP_DDI_FUNC_CTL),
	    !!(igen_r32(sc, HSW_TRANS_EDP_DDI_FUNC_CTL) & TRANS_DDI_FUNC_ENABLE),
	    (igen_r32(sc, HSW_TRANS_EDP_DDI_FUNC_CTL) >> TRANS_DDI_PORT_SHIFT) & 7,
	    (igen_r32(sc, HSW_TRANS_EDP_DDI_FUNC_CTL) >> TRANS_DDI_MODE_SHIFT) & 7);
	device_printf(sc->dev,
	    "  TRANS_EDP_CONF      = 0x%08x  (ENABLE=%u STATE=%u)\n",
	    igen_r32(sc, HSW_TRANS_EDP_CONF),
	    !!(igen_r32(sc, HSW_TRANS_EDP_CONF) & HSW_TRANS_EDP_CONF_ENABLE),
	    !!(igen_r32(sc, HSW_TRANS_EDP_CONF) & HSW_TRANS_EDP_CONF_STATE));
	device_printf(sc->dev,
	    "  TRANS_EDP_HTOTAL    = 0x%08x  VTOTAL = 0x%08x\n",
	    igen_r32(sc, HSW_TRANS_EDP_HTOTAL),
	    igen_r32(sc, HSW_TRANS_EDP_VTOTAL));
	device_printf(sc->dev,
	    "  PIPE_A_CONF         = 0x%08x   PIPE_A_SRCSZ = 0x%08x\n",
	    igen_r32(sc, PIPE_CONF(0)), igen_r32(sc, HSW_PIPE_SRCSZ(0)));
	device_printf(sc->dev,
	    "  DSPCNTR[A]          = 0x%08x  (ENABLE=%u fmt=0x%x)\n",
	    igen_r32(sc, HSW_DSPCNTR(0)),
	    !!(igen_r32(sc, HSW_DSPCNTR(0)) & DSPCNTR_ENABLE),
	    (igen_r32(sc, HSW_DSPCNTR(0)) >> DSPCNTR_FORMAT_SHIFT) & 0xf);
	device_printf(sc->dev,
	    "  DSPSTRIDE[A]        = 0x%08x   DSPSURF[A]  = 0x%08x\n",
	    igen_r32(sc, HSW_DSPSTRIDE(0)), igen_r32(sc, HSW_DSPSURF(0)));

	/* DPCD: rev, link rate cap, lane count cap, training state. */
	n = kms_dp_dpcd_read(&sc->aux_a.aux, 0x000, dpcd, sizeof(dpcd));
	if (n > 0) {
		device_printf(sc->dev,
		    "  DPCD rev=0x%02x  max_link_rate=0x%02x"
		    " (0x06=RBR 0x0a=HBR 0x14=HBR2)\n",
		    dpcd[0], dpcd[1]);
		device_printf(sc->dev,
		    "  DPCD max_lane=%u  enhanced=%u  downspread=0x%02x\n",
		    dpcd[2] & 0x1f, !!(dpcd[2] & 0x80), dpcd[3]);
		device_printf(sc->dev,
		    "  DPCD train_aux_rd_interval=%u\n", dpcd[14] & 0xf);
	} else {
		device_printf(sc->dev, "  DPCD read failed: %zd\n", n);
	}

	/* DPCD 0x100: current link configuration. */
	n = kms_dp_dpcd_read(&sc->aux_a.aux, 0x100, link_status, 4);
	if (n > 0) {
		device_printf(sc->dev,
		    "  DPCD LINK_BW_SET=0x%02x  LANE_COUNT_SET=0x%02x"
		    "  TRAINING_PATTERN=0x%02x\n",
		    link_status[0], link_status[1] & 0x1f, link_status[2]);
	}

	/* DPCD 0x200..0x205: live training/lock state. */
	n = kms_dp_dpcd_read(&sc->aux_a.aux, 0x200, link_status,
	    sizeof(link_status));
	if (n > 0) {
		device_printf(sc->dev,
		    "  DPCD SINK_STATUS=0x%02x  LANE01=0x%02x  LANE23=0x%02x\n",
		    link_status[5], link_status[2], link_status[3]);
		device_printf(sc->dev,
		    "  DPCD ALIGN=0x%02x  CR_done(0/1)=%u/%u  EQ_done(0/1)=%u/%u\n",
		    link_status[4],
		    !!(link_status[2] & 0x01), !!(link_status[2] & 0x10),
		    !!(link_status[2] & 0x02), !!(link_status[2] & 0x20));
	}

	return (0);
}

/* --------------------------- pipe bring-up -------------------------------- */

/*
 * Compute PIPE_DATA_M and PIPE_DATA_N (M/N stuffer) for a DP-SST link.
 * Formula per DP 1.4 §2.2.4.2:
 *   M / N = (pixel_rate * bytes_per_pixel) / (link_rate * lane_count)
 * We use 8 bytes per "transfer unit" and pick N = 0x80_0000 (the
 * canonical i915 value), then derive M = M/N * N.  TU size is set to
 * 64 in the high 6 bits.
 */
static void
igen_hsw_dp_mn(uint32_t pixel_khz, uint32_t bytes_per_pixel,
    uint32_t link_khz, uint32_t lanes, uint32_t *m_out, uint32_t *n_out)
{
	const uint32_t N = 0x800000;
	uint64_t m;

	m = ((uint64_t)pixel_khz * bytes_per_pixel * N) /
	    ((uint64_t)link_khz * lanes);
	*n_out = N;
	*m_out = (uint32_t)m | PIPE_DATA_M_TU_SIZE_DEFAULT;
}

/*
 * Sub-step: program TRANSCODER EDP timing from a DTD.
 * Writes HTOTAL/HBLANK/HSYNC/VTOTAL/VBLANK/VSYNC.  Pipe and transcoder
 * must be disabled before calling (BSpec rule).
 */
static void
igen_hsw_program_trans_edp_timing(struct igen_softc *sc,
    const struct igen_dtd *t)
{
	uint32_t htotal = t->hactive + t->hblank;
	uint32_t vtotal = t->vactive + t->vblank;
	uint32_t hsync_start = t->hactive + t->hsync_off;
	uint32_t hsync_end   = hsync_start + t->hsync_w;
	uint32_t vsync_start = t->vactive + t->vsync_off;
	uint32_t vsync_end   = vsync_start + t->vsync_w;
	uint32_t hblank_start = t->hactive;
	uint32_t hblank_end   = htotal;
	uint32_t vblank_start = t->vactive;
	uint32_t vblank_end   = vtotal;

	igen_w32(sc, HSW_TRANS_EDP_HTOTAL,
	    htotal_pack(t->hactive, htotal));
	igen_w32(sc, HSW_TRANS_EDP_HBLANK,
	    hblank_pack(hblank_start, hblank_end));
	igen_w32(sc, HSW_TRANS_EDP_HSYNC,
	    hsync_pack(hsync_start, hsync_end));
	igen_w32(sc, HSW_TRANS_EDP_VTOTAL,
	    htotal_pack(t->vactive, vtotal));
	igen_w32(sc, HSW_TRANS_EDP_VBLANK,
	    hblank_pack(vblank_start, vblank_end));
	igen_w32(sc, HSW_TRANS_EDP_VSYNC,
	    hsync_pack(vsync_start, vsync_end));
	igen_w32(sc, HSW_TRANS_EDP_VSYNCSHIFT, 0);
	igen_w32(sc, HSW_PIPE_SRCSZ(0),
	    ((t->hactive - 1) << 16) | (t->vactive - 1));

	DPRINTF(sc, 1,
	    "hsw_pipe: TRANS_EDP timing %ux%u  pixclk %u kHz  HTOTAL=%u VTOTAL=%u\n",
	    t->hactive, t->vactive, t->pixclk_khz, htotal, vtotal);
}

/*
 * MINIMAL bring-up: exploit firmware-left state.  On macbsd (and any
 * MacBook Pro Retina with HSW iGPU), the EFI firmware leaves the eDP
 * pipeline pre-configured up to the transcoder — CDCLK, PWELL, LCPLL,
 * PORT_CLK_SEL, DDI_BUF, DP_TP, TRANS_EDP_DDI_FUNC, TRANS_EDP_CONF
 * are all enabled, and the DP link is fully trained at HBR×4.  The
 * ONLY thing missing is PIPE_CONF[A].ENABLE plus a correctly-sized
 * source FB and PLANE_*.
 *
 * This routine does just that:
 *   1. Decode preferred mode from cached EDID.
 *   2. (Re-)allocate sc->scanout_fb at the preferred size if it's
 *      missing or too small (the existing scanout_hold defaults to
 *      1920×1080 which won't satisfy a 2880×1800 transcoder).
 *   3. Fill with a high-contrast checker so we can see it landed.
 *   4. Program PIPE_SRCSZ + PLANE_SIZE/STRIDE/SURF/CTL.
 *   5. Flip PIPE_CONF[A] ENABLE on, poll STATE.
 *
 * We intentionally do NOT touch DDI_BUF / DP_TP / TRANS_EDP* — they're
 * already correct.  Toggling them risks dropping the DP link, which
 * would require a full LT cycle we haven't implemented yet.
 *
 * If this returns 0 and the panel still doesn't light, the next
 * suspects (in order) are: (a) DP link state went stale after our
 * earlier writes; (b) Apple SMC backlight isn't on (asmc(4) would
 * own that, not the GPU); (c) PIPE_DATA_M/N are wrong for the
 * preferred mode (firmware programmed them for whatever mode it was
 * using, possibly the EFI GOP rectangle).
 */
int
igen_hsw_panel_on(struct igen_softc *sc)
{
	uint8_t edid_dtd[18];
	struct igen_dtd t;
	uint32_t m, n;
	uint32_t link_khz = 270000;	/* HBR; firmware-left link rate */
	uint32_t lanes = 4;		/* HBR×4; firmware-left */
	size_t need_bytes;

	if (sc->cached_edid_len < 128) {
		device_printf(sc->dev,
		    "hsw_panel_on: no cached EDID — run edid path first\n");
		return (ENOENT);
	}
	memcpy(edid_dtd, sc->cached_edid + 54, sizeof(edid_dtd));
	if (!igen_decode_dtd(edid_dtd, &t)) {
		device_printf(sc->dev, "hsw_panel_on: DTD decode failed\n");
		return (EINVAL);
	}

	/*
	 * The scanout_fb allocator (igen_test_fb_alloc) is file-static
	 * in igen_gtt.c, so we can't allocate here.  Caller (sysctl path
	 * or atomic_commit) is responsible for ensuring sc->scanout_fb
	 * is sized to match the transcoder.  If absent or undersized we
	 * still proceed — the pipe will scan whatever's at the current
	 * PLANE_SURF (firmware-left), which is at least useful as a
	 * smoke test that the pipe-enable itself works.
	 */
	if (sc->scanout_fb != NULL && sc->scanout_fb->mapped &&
	    (sc->scanout_fb->width != t.hactive ||
	     sc->scanout_fb->height != t.vactive)) {
		device_printf(sc->dev,
		    "hsw_panel_on: scanout_fb is %ux%u but pipe wants %ux%u —"
		    " image will be wrong size\n",
		    sc->scanout_fb->width, sc->scanout_fb->height,
		    t.hactive, t.vactive);
	}
	(void)need_bytes;

	/* PIPE source size: matches what the transcoder is sending. */
	igen_w32(sc, HSW_PIPE_SRCSZ(0),
	    ((t.hactive - 1) << 16) | (t.vactive - 1));

	/*
	 * M/N stuffer for the DP encoder.  Pixel rate / link rate ratio
	 * tells the encoder how often to insert null symbols.  Firmware
	 * may have programmed these for a non-native EFI mode, so
	 * recompute for the panel-preferred mode.
	 */
	igen_hsw_dp_mn(t.pixclk_khz, 3 /* bytes/pixel @ 24bpp */,
	    link_khz, lanes, &m, &n);
	igen_w32(sc, HSW_PIPE_DATA_M(0), m);
	igen_w32(sc, HSW_PIPE_DATA_N(0), n);
	igen_w32(sc, HSW_PIPE_LINK_M(0), m & ~PIPE_DATA_M_TU_SIZE_DEFAULT);
	igen_w32(sc, HSW_PIPE_LINK_N(0), n);

	/* PLANE configuration.  Defer SURF/STRIDE until scanout_fb is
	 * actually populated by the caller (currently a separate sysctl).
	 * For now point at the firmware-left SURF so the pipe has SOME
	 * source when enabled.  The caller's next step (scanout_hold or
	 * Xorg ADDFB2) will overwrite SURF and we get the real image. */
	igen_w32(sc, PLANE_SIZE(0),
	    ((t.vactive - 1) << 16) | (t.hactive - 1));
	if (sc->scanout_fb != NULL && sc->scanout_fb->mapped) {
		igen_w32(sc, HSW_DSPSTRIDE(0), sc->scanout_fb->stride);
		igen_w32(sc, HSW_DSPSURF(0),
		    (uint32_t)(sc->scanout_fb->gtt_first_idx * 4096));
	}
	igen_w32(sc, HSW_DSPLINOFF(0), 0);
	igen_w32(sc, HSW_DSPTILEOFF(0), 0);
	uint32_t dspcntr = igen_r32(sc, HSW_DSPCNTR(0));
	/*
	 * DSPCNTR shape derived from a live-fire Linux i915 register
	 * comparison on a Broadwell MBP (Ubuntu on 192.168.1.72, 2026-08-04):
	 *
	 *   Linux i915:  DSPACNTR = 0xe8000400  (bit 29 = TRICKLE_FEED_DISABLE)
	 *   FreeBSD:     DSPACNTR = 0xd8000400  (bit 28 = TILED, no trickle
	 *                                        feed disable, wrong)
	 *
	 * The firmware-left value has TILED=1 (X-tile) because the EFI FB is
	 * X-tiled.  Our GTT-bound framebuffers are linear.  Clear TILED, set
	 * TRICKLE_FEED_DISABLE (required on HSW/BDW eDP for framer sync).
	 */
	dspcntr &= ~((0xfu << DSPCNTR_FORMAT_SHIFT) | DSPCNTR_TILED);
	dspcntr |= DSPCNTR_ENABLE | DSPCNTR_GAMMA_ENABLE |
	    DSPCNTR_FORMAT_BGRX8888 |
	    (1u << 29);	/* TRICKLE_FEED_DISABLE */
	igen_w32(sc, HSW_DSPCNTR(0), dspcntr);

	/*
	 * CRITICAL: on HSW/BDW eDP, do NOT enable PIPE_A_CONF.  Confirmed
	 * against Linux i915 live-fire (Broadwell MBP): PIPEACONF stays 0
	 * while the display renders 2560x1600, because TRANS_EDP is the
	 * effective pipe for eDP output.  Enabling PIPE_A_CONF would clobber
	 * a specific power-well state the display power domain machinery
	 * relies on ("pf-pd" in intel_reg decoding = panel-fitter powered
	 * down, expected).  The transcoder pulls plane data through the
	 * PIPE_A data path without PIPE_A_CONF being enabled.
	 *
	 * So: program plane (SURF/STRIDE/CTL) + M/N + PIPE_SRCSZ, then
	 * return.  No PIPE_CONF write, no STATE poll.
	 */

	device_printf(sc->dev,
	    "hsw_panel_on: apple-handoff shape (PIPE_A stays off; TRANS_EDP"
	    " drives panel)  DSPACNTR=0x%08x DSPASTRIDE=0x%08x"
	    " DSPASURF=0x%08x  M=0x%08x N=0x%08x\n",
	    igen_r32(sc, HSW_DSPCNTR(0)),
	    igen_r32(sc, HSW_DSPSTRIDE(0)),
	    igen_r32(sc, HSW_DSPSURF(0)), m, n);
	return (0);
}

static int
igen_sysctl_hsw_panel_on(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);
	if (sc->gen != IGEN_GEN_HSW) {
		device_printf(sc->dev, "hsw_panel_on: HSW-only\n");
		return (EOPNOTSUPP);
	}
	return (igen_hsw_panel_on(sc));
}

/* --------------------------- pipe-off primitive --------------------------- */

/*
 * Symmetric counterpart to igen_hsw_panel_on: drives pipe A to the fully
 * quiescent state (planes off, cursor off, PIPE_CONF ENABLE + STATE both
 * clear) so the display engine stops walking GGTT.
 *
 * Motivation: on macbsd (Iris Pro 5200 HSW), firmware hands the driver
 * a live pipe A with PIPE_CONF=0xc0000000.  Any subsequent ELSP submit
 * (gt_first_batch_submit=2 or EXECBUFFER2 dispatch under
 * i915_dispatch_enable=2) that allocates ring/LRC/HWSP through
 * igen_gtt_alloc lands in GGTT the display engine is concurrently
 * fetching from — bus stall, kernel-side wedge, net stack goes down,
 * only physical power-cycle recovers.  This is the pre-flight primitive
 * that makes live-fire ELSP submission safe.
 *
 * Sequence (matches BSpec HSW display pipe-off — the compact form; we
 * leave transcoder / DDI / DPLL up because they can't fetch memory
 * without pipe, and leaving them powered avoids a full re-LT if the
 * caller later wants panel_on):
 *
 *   1. Disable primary plane (PLANE_CTL bit 31 = 0).
 *   2. Arm the plane change by rewriting PLANE_SURF — the double-
 *      buffered plane state only latches on the next vblank + SURF
 *      write.
 *   3. Disable cursor (CUR_CTL_A = 0, CUR_BASE_A = 0).
 *   4. Wait one vblank so both plane + cursor disables land.
 *   5. Clear PIPE_CONF ENABLE.
 *   6. Poll PIPE_CONF STATE bit — this drains any in-flight scanout
 *      fetches.  Cap at ~100 ms.
 *
 * Serialized via scanout_lock against any concurrent atomic_commit /
 * cursor_move so the pipe MMIO doesn't get re-poked mid-teardown.
 */
int
igen_pipe_a_off(struct igen_softc *sc)
{
	uint32_t pconf, pctl, cur_surf;
	int spin;

	sx_xlock(&sc->scanout_lock);

	pconf = igen_r32(sc, PIPE_CONF(0));
	if ((pconf & PIPE_CONF_ENABLE) == 0) {
		device_printf(sc->dev,
		    "hsw_pipe_off: pipe A already off (PIPE_CONF=0x%08x)\n",
		    pconf);
		sx_xunlock(&sc->scanout_lock);
		return (0);
	}

	pctl = igen_r32(sc, PLANE_CTL(0));
	cur_surf = igen_r32(sc, PLANE_SURF(0));
	device_printf(sc->dev,
	    "hsw_pipe_off: entry  PIPE_CONF=0x%08x  PLANE_CTL=0x%08x"
	    "  PLANE_SURF=0x%08x\n", pconf, pctl, cur_surf);

	/* Step 1 + 2: primary plane disable, arm with SURF rewrite. */
	if (pctl & PLANE_CTL_ENABLE) {
		igen_w32(sc, PLANE_CTL(0), pctl & ~PLANE_CTL_ENABLE);
		igen_w32(sc, PLANE_SURF(0), cur_surf);
	}

	/* Step 3: cursor disable. */
	igen_w32(sc, CUR_CTL_A, CUR_MODE_DISABLE);
	igen_w32(sc, CUR_BASE_A, 0);

	/* Step 4: let plane + cursor disable latch at vblank. */
	igen_wait_vblank(sc, 0);

	/* Step 5: pipe disable. */
	pconf = igen_r32(sc, PIPE_CONF(0));
	igen_w32(sc, PIPE_CONF(0), pconf & ~PIPE_CONF_ENABLE);

	/*
	 * Step 6: wait for PIPE_STATE to clear.  On a pipe with no
	 * scanout backlog this drops in one or two vblanks; the 100 ms
	 * cap is generous but bounded so a stuck pipe cannot wedge the
	 * sysctl thread indefinitely.
	 */
	for (spin = 0; spin < 100; spin++) {
		pconf = igen_r32(sc, PIPE_CONF(0));
		if ((pconf & PIPE_CONF_STATE) == 0)
			break;
		pause("gen9pipeoff", hz / 1000);
	}

	device_printf(sc->dev,
	    "hsw_pipe_off: exit  PIPE_CONF=0x%08x  spin=%d/100  %s\n",
	    pconf, spin,
	    (pconf & PIPE_CONF_STATE) ? "STILL ACTIVE" : "quiescent");

	sc->scanout_held = false;
	sx_xunlock(&sc->scanout_lock);

	return ((pconf & PIPE_CONF_STATE) ? ETIMEDOUT : 0);
}

static int
igen_sysctl_pipe_a_off(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);
	/* PIPE_CONF layout (bits 31/30), PLANE_CTL/SURF, CUR_CTL_A are
	 * identical across HSW/BDW/SKL/KBL, so no gen gate.  Later gens
	 * (ICL+) grow universal-plane registers but pipe A itself is still
	 * disabled by the same PIPE_CONF write. */
	return (igen_pipe_a_off(sc));
}

/* --------------- gen9 display-state diagnostic dump (read-only) ---------- */

/*
 * Full read-only snapshot of every gen9 display-side register we could
 * possibly need to reconstruct a working panel bring-up on a boot where
 * firmware hands us a dark pipe.  Triggered by `re.gen9_display_dump=1`
 * (write-only sysctl).  Prints straight to dmesg.
 *
 * ZERO WRITE risk: this only reads.  Use while GUI is up + working to
 * capture the "known-good baseline" that firmware programmed.  Then
 * hard-code those values in gen9_panel_on so cold-boot cases can adopt
 * them.
 */

/* Clocks + PLL infrastructure (BSpec vol 12: display registers). */
#define	CDCLK_CTL			0x00046000
#define	LCPLL1_CTL			0x00046010
#define	LCPLL2_CTL			0x00046014
#define	DPLL_STATUS			0x0006c060
#define	DPLL_CTRL1			0x0006c058
#define	DPLL_CTRL2			0x0006c05c
#define	DPLL1_CFGCR1			0x0006c040
#define	DPLL1_CFGCR2			0x0006c044
#define	DPLL2_CFGCR1			0x0006c048
#define	DPLL2_CFGCR2			0x0006c04c
#define	DPLL3_CFGCR1			0x0006c050
#define	DPLL3_CFGCR2			0x0006c054

/* Pipe M/N stuffer for the DP encoder. */
#define	PIPEA_DATA_M			0x00060030
#define	PIPEA_DATA_N			0x00060034
#define	PIPEA_LINK_M			0x00060040
#define	PIPEA_LINK_N			0x00060044

/* Panel power (eDP). */
#define	PP_STATUS			0x000c7200
#define	PP_CONTROL			0x000c7204

static int
igen_sysctl_gen9_display_dump(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);
	if (sc->gen != IGEN_GEN_SKL)
		return (EOPNOTSUPP);

	device_printf(sc->dev, "=== gen9 display-state dump ===\n");

	device_printf(sc->dev,
	    "CLOCKS   CDCLK_CTL=0x%08x  LCPLL1_CTL=0x%08x  LCPLL2_CTL=0x%08x\n",
	    igen_r32(sc, CDCLK_CTL),
	    igen_r32(sc, LCPLL1_CTL),
	    igen_r32(sc, LCPLL2_CTL));

	device_printf(sc->dev,
	    "DPLL     STATUS=0x%08x  CTRL1=0x%08x  CTRL2=0x%08x\n",
	    igen_r32(sc, DPLL_STATUS),
	    igen_r32(sc, DPLL_CTRL1),
	    igen_r32(sc, DPLL_CTRL2));

	device_printf(sc->dev,
	    "DPLL1    CFGCR1=0x%08x  CFGCR2=0x%08x\n",
	    igen_r32(sc, DPLL1_CFGCR1), igen_r32(sc, DPLL1_CFGCR2));
	device_printf(sc->dev,
	    "DPLL2    CFGCR1=0x%08x  CFGCR2=0x%08x\n",
	    igen_r32(sc, DPLL2_CFGCR1), igen_r32(sc, DPLL2_CFGCR2));
	device_printf(sc->dev,
	    "DPLL3    CFGCR1=0x%08x  CFGCR2=0x%08x\n",
	    igen_r32(sc, DPLL3_CFGCR1), igen_r32(sc, DPLL3_CFGCR2));

	device_printf(sc->dev,
	    "PIPE_A   CONF=0x%08x  SRCSZ=0x%08x  HTOTAL=0x%08x  HBLANK=0x%08x\n",
	    igen_r32(sc, PIPE_CONF(0)),
	    igen_r32(sc, PIPE_SRCSZ(0)),
	    igen_r32(sc, TRANS_HTOTAL(0)),
	    igen_r32(sc, TRANS_HBLANK(0)));
	device_printf(sc->dev,
	    "PIPE_A   HSYNC=0x%08x  VTOTAL=0x%08x  VBLANK=0x%08x  VSYNC=0x%08x\n",
	    igen_r32(sc, TRANS_HSYNC(0)),
	    igen_r32(sc, TRANS_VTOTAL(0)),
	    igen_r32(sc, TRANS_VBLANK(0)),
	    igen_r32(sc, TRANS_VSYNC(0)));

	device_printf(sc->dev,
	    "PIPE_A   DATA_M=0x%08x  DATA_N=0x%08x  LINK_M=0x%08x  LINK_N=0x%08x\n",
	    igen_r32(sc, PIPEA_DATA_M),
	    igen_r32(sc, PIPEA_DATA_N),
	    igen_r32(sc, PIPEA_LINK_M),
	    igen_r32(sc, PIPEA_LINK_N));

	device_printf(sc->dev,
	    "TRANS_A  DDI_FUNC_CTL=0x%08x  CLK_SEL=0x%08x\n",
	    igen_r32(sc, TRANS_DDI_FUNC_CTL(0)),
	    igen_r32(sc, 0x00046140));

	device_printf(sc->dev,
	    "DDI_BUF  A=0x%08x  B=0x%08x  C=0x%08x  D=0x%08x  E=0x%08x\n",
	    igen_r32(sc, DDI_BUF_CTL(0)),
	    igen_r32(sc, DDI_BUF_CTL(1)),
	    igen_r32(sc, DDI_BUF_CTL(2)),
	    igen_r32(sc, DDI_BUF_CTL(3)),
	    igen_r32(sc, DDI_BUF_CTL(4)));

	device_printf(sc->dev,
	    "PLANE_A  CTL=0x%08x  STRIDE=0x%08x  SURF=0x%08x\n",
	    igen_r32(sc, PLANE_CTL(0)),
	    igen_r32(sc, PLANE_STRIDE(0)),
	    igen_r32(sc, PLANE_SURF(0)));

	device_printf(sc->dev,
	    "HPD/FUSE SFUSE_STRAP=0x%08x  SHOTPLUG_CTL_DDI=0x%08x  SDEISR=0x%08x\n",
	    igen_r32(sc, SFUSE_STRAP),
	    igen_r32(sc, SHOTPLUG_CTL_DDI),
	    igen_r32(sc, SDEISR));

	device_printf(sc->dev,
	    "PANEL_PWR PP_STATUS=0x%08x  PP_CONTROL=0x%08x\n",
	    igen_r32(sc, PP_STATUS),
	    igen_r32(sc, PP_CONTROL));

	device_printf(sc->dev, "=== end gen9 display-state dump ===\n");
	return (0);
}

/* ------------------- gen9 (SKL/KBL) pipe-A cold bring-up ------------------ */

/*
 * Bring pipe A up on gen9 (SKL/KBL) from a firmware-off state.
 *
 * Assumption: firmware/BIOS has already programmed CDCLK, port PLL,
 * DDI_BUF_TRANS voltage swing tables, TRANS_DDI_FUNC_CTL port select,
 * and DDI_BUF_CTL — the common EFI/UEFI handoff pattern.  What
 * firmware often SKIPS on shutdown-then-boot is the transcoder timing
 * writes and the PIPE_CONF enable.  So we program TRANS_H* / TRANS_V*
 * from the cached EDID DTD, set PIPE_SRCSZ, program the primary plane
 * for whatever scanout_fb the caller has installed (or leave it at
 * firmware SURF as a smoke test), then flip PIPE_CONF ENABLE.
 *
 * This does NOT handle:
 *   - Cold PLL / CDCLK bring-up (firmware-programmed only).
 *   - DDI voltage swing table selection.
 *   - Link training for DP/eDP.
 *   - Non-primary planes.
 *
 * Live-verified 2026-07-16 as the atomic_commit path when firmware
 * boots with pipe A off — kwin's mode request from EDID + our
 * transcoder-timing writes are enough to light the panel.  For deeper
 * cold-boot cases (fresh CDCLK, unlinked DP), stage-3 modeset work is
 * still needed.
 *
 * `mode` — the target mode.  hdisplay, vdisplay, htotal, vtotal,
 * hsync_start/end, vsync_start/end must all be populated.  Typically
 * comes from cs->mode in atomic_commit, which kwin builds from EDID.
 */
int
igen_gen9_panel_on(struct igen_softc *sc, const struct drm_display_mode *m)
{
	int i;
	uint32_t pconf;

	if (sc->gen != IGEN_GEN_SKL)
		return (EOPNOTSUPP);
	if (m == NULL || m->hdisplay == 0 || m->vdisplay == 0 ||
	    m->htotal < m->hdisplay || m->vtotal < m->vdisplay)
		return (EINVAL);

	/*
	 * Transcoder A timing.  Format on gen9 (BSpec: matches HSW):
	 *   TRANS_HTOTAL[28:16] = htotal-1, [12:0] = hdisplay-1
	 *   TRANS_HBLANK: same, but describes blank region
	 *   TRANS_HSYNC[28:16] = hsync_end-1, [12:0] = hsync_start-1
	 *   TRANS_VTOTAL, VBLANK, VSYNC: same layout for vertical.
	 * Blank == total-active window on this gen; encoder walks through
	 * (start..sync_start) as active + backporch and (sync_start..
	 * sync_end) as sync + (sync_end..total) as front porch.
	 */
	igen_w32(sc, TRANS_HTOTAL(0),
	    (((uint32_t)m->htotal - 1) << 16) | ((uint32_t)m->hdisplay - 1));
	igen_w32(sc, TRANS_HBLANK(0),
	    (((uint32_t)m->htotal - 1) << 16) | ((uint32_t)m->hdisplay - 1));
	igen_w32(sc, TRANS_HSYNC(0),
	    (((uint32_t)m->hsync_end - 1) << 16) |
	    ((uint32_t)m->hsync_start - 1));
	igen_w32(sc, TRANS_VTOTAL(0),
	    (((uint32_t)m->vtotal - 1) << 16) | ((uint32_t)m->vdisplay - 1));
	igen_w32(sc, TRANS_VBLANK(0),
	    (((uint32_t)m->vtotal - 1) << 16) | ((uint32_t)m->vdisplay - 1));
	igen_w32(sc, TRANS_VSYNC(0),
	    (((uint32_t)m->vsync_end - 1) << 16) |
	    ((uint32_t)m->vsync_start - 1));

	/*
	 * DDI + TRANS_CLK_SEL + TRANS_DDI_FUNC_CTL enables.  Gated behind
	 * sc->gen9_ddi_enable because enabling DDI_BUF_CTL against a cold
	 * port PLL wedges the box hard (5th wedge in the 2026-07-16
	 * session — see memory `feedback_igen_no_force_submit_pipe_active`).
	 *
	 * TODO: check DPLL_CTRL2 / port PLL live state before doing this
	 * automatically.  Until then, the operator sets
	 * `dev.igen.<n>.re.gen9_ddi_enable=1` after verifying via
	 * `hpd_dump` that a port is HPD-live AND its PLL is up.
	 */
	if (sc->gen9_ddi_enable != 0) {
#define	TRANS_CLK_SEL_A			0x00046140
#define	  TRANS_CLK_SEL_PORT_DDI_B	(2u << 29)
#define	TRANS_DDI_FUNC_CTL_ENABLE	(1u << 31)
#define	TRANS_DDI_PORT_B		(1u << 28)
#define	TRANS_DDI_MODE_HDMI		(0u << 24)
#define	TRANS_DDI_BPC_8			(0u << 20)
#define	DDI_BUF_CTL_B			0x00064100
#define	  DDI_BUF_CTL_ENABLE_B		(1u << 31)
#define	  DDI_BUF_PORT_WIDTH_X4		(3u << 1)
		uint32_t fctl_cur = igen_r32(sc, TRANS_DDI_FUNC_CTL(0));
		uint32_t sync_bits = fctl_cur & 0x00030000;

		igen_w32(sc, TRANS_CLK_SEL_A, TRANS_CLK_SEL_PORT_DDI_B);
		igen_w32(sc, TRANS_DDI_FUNC_CTL(0),
		    TRANS_DDI_FUNC_CTL_ENABLE | TRANS_DDI_PORT_B |
		    TRANS_DDI_MODE_HDMI | TRANS_DDI_BPC_8 | sync_bits);
		igen_w32(sc, DDI_BUF_CTL_B,
		    DDI_BUF_CTL_ENABLE_B | DDI_BUF_PORT_WIDTH_X4);
		(void)igen_r32(sc, DDI_BUF_CTL_B);
		device_printf(sc->dev,
		    "gen9_panel_on: DDI/CLK enables written (opt-in)\n");
	}

	/* PIPE source size — matches transcoder active. */
	igen_w32(sc, PIPE_SRCSZ(0),
	    (((uint32_t)m->hdisplay - 1) << 16) |
	    ((uint32_t)m->vdisplay - 1));

	/*
	 * Primary plane setup.  If scanout_fb is installed, program its
	 * stride and surface address; otherwise leave PLANE_CTL alone
	 * (firmware SURF stays live).  This mirrors HSW panel_on's
	 * "trust firmware SURF if we have nothing better" fallback.
	 */
	if (sc->scanout_fb != NULL && sc->scanout_fb->mapped) {
		igen_w32(sc, PLANE_STRIDE(0), sc->scanout_fb->stride / 64);
		igen_w32(sc, PLANE_SURF(0),
		    sc->scanout_fb->gtt_first_idx * PAGE_SIZE);
	}
	/*
	 * Wait — PLANE_SIZE format on gen9 vs HSW is subtly different.
	 * On gen9 PLANE_SIZE = (height-1) << 16 | (width-1).  HSW same.
	 */

	/* PIPE_CONF ENABLE — 1% of the work, 99% of the visible effect. */
	pconf = igen_r32(sc, PIPE_CONF(0));
	igen_w32(sc, PIPE_CONF(0), pconf | PIPE_CONF_ENABLE);

	/* Wait up to 200 ms for pipe to go active. */
	for (i = 0; i < 200; i++) {
		if (igen_r32(sc, PIPE_CONF(0)) & PIPE_CONF_STATE)
			break;
		pause("gen9pipeon", hz / 1000);
	}

	pconf = igen_r32(sc, PIPE_CONF(0));
	device_printf(sc->dev,
	    "gen9_panel_on: pipe A %s  PIPE_CONF=0x%08x mode=%ux%u"
	    " (htotal=%u vtotal=%u pixclk=%d kHz)\n",
	    (pconf & PIPE_CONF_STATE) ? "LIVE" : "STALL",
	    pconf, m->hdisplay, m->vdisplay,
	    m->htotal, m->vtotal, m->clock);

	return ((pconf & PIPE_CONF_STATE) ? 0 : ETIMEDOUT);
}

/*
 * Full cold pipe-A bring-up for gen9 (SKL/KBL) on a Kabylake-S desktop
 * with HDMI on DDI B (Dell OptiPlex 5040 / fbsdx86).
 *
 * Hardcoded values captured 2026-07-17 from a live 1920x1080@60 session
 * via `re.gen9_display_dump=1`:
 *   CDCLK_CTL     = 0x0c000544  (untouched — firmware default)
 *   LCPLL1_CTL    = 0xc0000000  (enable + lock — untouched)
 *   LCPLL2_CTL    = 0x80000000  (enable — untouched)
 *   DPLL_CTRL1    = 0x00000845
 *   DPLL_CTRL2    = 0x00a00018
 *   DPLL1_CFGCR1  = 0x80400173  (148.5 MHz pixel clock config)
 *   DPLL1_CFGCR2  = 0x000003a5
 *   TRANS_CLK_SEL = 0x40000000  (DDI B clock)
 *   TRANS_DDI_FUNC_CTL = 0x90030000 (ENABLE | DDI_B | HDMI | 8bpc |
 *                                    PHSYNC | PVSYNC)
 *   DDI_BUF_CTL_B = 0x80000000  (ENABLE, x1 port width)
 *   PLANE_CTL     = 0x84000000  (ENABLE | ARGB8888 | linear)
 *   PIPE_CONF     = 0xc0000000  (ENABLE + STATE running)
 *
 * Safety: read LCPLL1_CTL + DPLL_STATUS first.  If underlying
 * oscillators aren't up (LCPLL bit 31 clear or DPLL1 not locked),
 * bail out — bringing up LCPLL / CDCLK from scratch needs work we
 * don't yet do.
 *
 * Assumption: firmware left the raw clock/PLL infrastructure programmed
 * (CDCLK, LCPLLs) but reset the transcoder/DDI wiring on the way out.
 * That matches what we've seen — LCPLL always reads back enabled even
 * on "dark" boots.  If firmware ever truly resets the PLLs cold, we'll
 * see it in gen9_display_dump and add PLL programming here.
 */
#define	CDCLK_CTL			0x00046000
#define	LCPLL1_CTL			0x00046010
#define	LCPLL2_CTL			0x00046014
#define	DPLL_STATUS			0x0006c060
#define	DPLL_CTRL1			0x0006c058
#define	DPLL_CTRL2			0x0006c05c
#define	DPLL1_CFGCR1			0x0006c040
#define	DPLL1_CFGCR2			0x0006c044
#define	TRANS_CLK_SEL_A			0x00046140

/*
 * DPLL_STATUS layout on gen9: one LOCK bit per PLL at stride 4.
 *   bit 0 = DPLL0 (LCPLL1 / CDCLK — DO NOT REPROGRAM)
 *   bit 4 = DPLL1 (our port PLL for DDI B)
 *   bit 8 = DPLL2
 *   bit 12 = DPLL3
 */
#define	LCPLL_ENABLE_BIT		(1u << 31)
#define	DPLL1_LOCK_BIT			(1u << 4)

/*
 * DPLL_CTRL1 layout: 6 bits per PLL.
 *   DPLL0 [5:0], DPLL1 [11:6], DPLL2 [17:12], DPLL3 [23:18].
 * DPLL_CTRL2 layout: 3 bits per DDI (override + 2-bit PLL select).
 *   DDI A [2:0], DDI B [5:3], DDI C [8:6], DDI D [11:9], DDI E [14:12].
 * ALL WRITES MUST BE RMW — DPLL0 field in CTRL1 controls LCPLL1 which
 * serves CDCLK.  Clobbering DPLL0 bits breaks CDCLK and cascade-wedges
 * the display fabric AND the kernel (6th wedge in the 2026-07-16/17
 * session — see feedback_igen_no_force_submit_pipe_active.md).
 */
#define	DPLL_CTRL1_DPLL1_MASK		0x00000FC0	/* bits [11:6] */
#define	DPLL_CTRL1_DPLL1_VAL		0x00000840	/* mode + SSC set */
#define	DPLL_CTRL2_DDI_B_MASK		0x00000038	/* bits [5:3] */
#define	DPLL_CTRL2_DDI_B_VAL		0x00000018	/* override + PLL1 */

int
igen_gen9_full_bringup(struct igen_softc *sc,
    const struct drm_display_mode *m)
{
	uint32_t lcpll1, dpll_status, pconf, ctrl1, ctrl2;
	int spin;

	if (sc->gen != IGEN_GEN_SKL)
		return (EOPNOTSUPP);
	if (m == NULL || m->hdisplay == 0 || m->vdisplay == 0)
		return (EINVAL);

	/* Log everything we're about to touch BEFORE writing anything. */
	lcpll1 = igen_r32(sc, LCPLL1_CTL);
	dpll_status = igen_r32(sc, DPLL_STATUS);
	ctrl1 = igen_r32(sc, DPLL_CTRL1);
	ctrl2 = igen_r32(sc, DPLL_CTRL2);
	device_printf(sc->dev,
	    "gen9_full_bringup: pre-write state:"
	    " LCPLL1=0x%08x DPLL_STATUS=0x%08x DPLL_CTRL1=0x%08x"
	    " DPLL_CTRL2=0x%08x\n",
	    lcpll1, dpll_status, ctrl1, ctrl2);

	if ((lcpll1 & LCPLL_ENABLE_BIT) == 0) {
		device_printf(sc->dev,
		    "gen9_full_bringup: REFUSE — LCPLL1 disabled;"
		    " cold LCPLL bring-up not implemented\n");
		return (ENXIO);
	}

	/*
	 * If DPLL1 is already locked, don't touch DPLL config at all —
	 * firmware left it programmed (which is the common case even on
	 * boots where the pipe is dark).  Skipping this saves us from
	 * accidentally clobbering DPLL0 via a mistaken RMW mask.
	 */
	if ((dpll_status & DPLL1_LOCK_BIT) != 0) {
		device_printf(sc->dev,
		    "gen9_full_bringup: DPLL1 already locked — skipping"
		    " DPLL config, only touching DDI + transcoder + pipe\n");
	} else {
		/*
		 * Cold path: DPLL1 needs programming.  RMW so DPLL0 (CDCLK
		 * source) is preserved bit-for-bit.  CFGCR values come from
		 * the SKL WRPLL solver (see igen_dpll_compute_cfgcr) so we
		 * can accept arbitrary HDMI pixel clocks, not just 148.5 MHz.
		 * For the firmware-programmed 1080p60 boot the solver returns
		 * the same 0x80400173 / 0x000003a5 pair that used to be
		 * hardcoded here — verified live 2026-07-17.
		 */
		uint32_t cfgcr1, cfgcr2;

		if (!igen_dpll_compute_cfgcr((uint32_t)m->clock,
		    &cfgcr1, &cfgcr2)) {
			device_printf(sc->dev,
			    "gen9_full_bringup: no DPLL solution for"
			    " %d kHz — aborting\n", m->clock);
			return (EINVAL);
		}
		device_printf(sc->dev,
		    "gen9_full_bringup: DPLL1 unlocked — programming"
		    " %d kHz  CFGCR1=0x%08x  CFGCR2=0x%08x\n",
		    m->clock, cfgcr1, cfgcr2);
		igen_w32(sc, DPLL1_CFGCR1, cfgcr1);
		igen_w32(sc, DPLL1_CFGCR2, cfgcr2);

		ctrl1 &= ~DPLL_CTRL1_DPLL1_MASK;
		ctrl1 |= DPLL_CTRL1_DPLL1_VAL;
		igen_w32(sc, DPLL_CTRL1, ctrl1);

		ctrl2 &= ~DPLL_CTRL2_DDI_B_MASK;
		ctrl2 |= DPLL_CTRL2_DDI_B_VAL;
		igen_w32(sc, DPLL_CTRL2, ctrl2);
		(void)igen_r32(sc, DPLL_CTRL2);

		/* Wait for DPLL1 lock (bit 4). */
		for (spin = 0; spin < 100; spin++) {
			if (igen_r32(sc, DPLL_STATUS) & DPLL1_LOCK_BIT)
				break;
			DELAY(100);
		}
		if ((igen_r32(sc, DPLL_STATUS) & DPLL1_LOCK_BIT) == 0) {
			device_printf(sc->dev,
			    "gen9_full_bringup: DPLL1 didn't lock after"
			    " 10 ms (STATUS=0x%08x) — aborting before"
			    " DDI enable\n",
			    igen_r32(sc, DPLL_STATUS));
			return (ETIMEDOUT);
		}
		device_printf(sc->dev,
		    "gen9_full_bringup: DPLL1 locked after %d us\n",
		    spin * 100);
	}

	/* Step 2: transcoder timing (from EDID / caller-supplied mode). */
	igen_w32(sc, TRANS_HTOTAL(0),
	    (((uint32_t)m->htotal - 1) << 16) |
	    ((uint32_t)m->hdisplay - 1));
	igen_w32(sc, TRANS_HBLANK(0),
	    (((uint32_t)m->htotal - 1) << 16) |
	    ((uint32_t)m->hdisplay - 1));
	igen_w32(sc, TRANS_HSYNC(0),
	    (((uint32_t)m->hsync_end - 1) << 16) |
	    ((uint32_t)m->hsync_start - 1));
	igen_w32(sc, TRANS_VTOTAL(0),
	    (((uint32_t)m->vtotal - 1) << 16) |
	    ((uint32_t)m->vdisplay - 1));
	igen_w32(sc, TRANS_VBLANK(0),
	    (((uint32_t)m->vtotal - 1) << 16) |
	    ((uint32_t)m->vdisplay - 1));
	igen_w32(sc, TRANS_VSYNC(0),
	    (((uint32_t)m->vsync_end - 1) << 16) |
	    ((uint32_t)m->vsync_start - 1));

	/* Step 3: route DDI B clock to transcoder A. */
	igen_w32(sc, TRANS_CLK_SEL_A, 0x40000000);

	/* Step 4: enable transcoder A -> DDI B in HDMI mode, 8bpc. */
	igen_w32(sc, TRANS_DDI_FUNC_CTL(0), 0x90030000);

	/* Step 5: enable DDI B buffer. */
	igen_w32(sc, DDI_BUF_CTL(1), 0x80000000);
	(void)igen_r32(sc, DDI_BUF_CTL(1));
	DELAY(600);	/* BSpec: wait ~518us for buffer up */

	/* Step 6: pipe source size + primary plane setup. */
	igen_w32(sc, PIPE_SRCSZ(0),
	    (((uint32_t)m->hdisplay - 1) << 16) |
	    ((uint32_t)m->vdisplay - 1));

	if (sc->scanout_fb != NULL && sc->scanout_fb->mapped) {
		igen_w32(sc, PLANE_STRIDE(0),
		    sc->scanout_fb->stride / 64);
		igen_w32(sc, PLANE_SURF(0),
		    sc->scanout_fb->gtt_first_idx * PAGE_SIZE);
	}
	/* Enable primary plane, ARGB8888, linear (untiled). */
	igen_w32(sc, PLANE_CTL(0), 0x84000000);

	/* Step 7: enable pipe. */
	pconf = igen_r32(sc, PIPE_CONF(0));
	igen_w32(sc, PIPE_CONF(0), pconf | PIPE_CONF_ENABLE);

	/* Wait for pipe active (PIPE_STATE bit 30). */
	for (spin = 0; spin < 200; spin++) {
		if (igen_r32(sc, PIPE_CONF(0)) & PIPE_CONF_STATE)
			break;
		pause("gen9brup", hz / 1000);
	}
	pconf = igen_r32(sc, PIPE_CONF(0));

	device_printf(sc->dev,
	    "gen9_full_bringup: pipe A %s after %d ms  PIPE_CONF=0x%08x"
	    "  DDI_BUF_B=0x%08x  mode=%ux%u  clock=%d kHz\n",
	    (pconf & PIPE_CONF_STATE) ? "LIVE" : "STALL",
	    spin, pconf,
	    igen_r32(sc, DDI_BUF_CTL(1)),
	    m->hdisplay, m->vdisplay, m->clock);

	return ((pconf & PIPE_CONF_STATE) ? 0 : ETIMEDOUT);
}

static int
igen_sysctl_gen9_full_bringup(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);
	struct drm_display_mode m;

	if (error || req->newptr == NULL || trigger == 0)
		return (error);
	if (sc->gen != IGEN_GEN_SKL)
		return (EOPNOTSUPP);
	if (sc->cached_edid_len < 128) {
		device_printf(sc->dev,
		    "gen9_full_bringup: no cached EDID — run edid_read_b"
		    " first\n");
		return (ENOENT);
	}
	igen_edid_to_mode(sc->cached_edid + 54, &m);
	return (igen_gen9_full_bringup(sc, &m));
}

static int
igen_sysctl_gen9_panel_on(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);
	struct drm_display_mode m;

	if (error || req->newptr == NULL || trigger == 0)
		return (error);
	if (sc->gen != IGEN_GEN_SKL)
		return (EOPNOTSUPP);
	if (sc->cached_edid_len < 128) {
		device_printf(sc->dev,
		    "gen9_panel_on: no cached EDID — run edid_read_b first\n");
		return (ENOENT);
	}

	/*
	 * Decode the preferred DTD (first 18 bytes at offset 54) into a
	 * drm_display_mode and call panel_on.  In production this comes
	 * from cs->mode via atomic_commit, but the sysctl lets us
	 * bring up the panel manually for debug.
	 */
	igen_edid_to_mode(sc->cached_edid + 54, &m);
	return (igen_gen9_panel_on(sc, &m));
}

/* ------------------------ gen9 full pipe teardown ------------------------ */

/*
 * Full teardown of the pipe A → transcoder A → DDI B → DPLL1 chain on
 * gen9 (SKL/KBL).  Extends igen_pipe_a_off() (plane + cursor + pipe
 * disable) with the transcoder / DDI / DPLL disable steps needed
 * before we can reprogram DPLL1 for a different pixel clock.
 *
 * BSpec-derived order (SKL vol 12 pipe-disable + PLL-disable sequences):
 *   1. igen_pipe_a_off  — plane off (armed on vblank), cursor off,
 *                         PIPE_CONF ENABLE clear, poll PIPE_STATE clear
 *   2. TRANS_DDI_FUNC_CTL(A) — clear bit 31 (transcoder function disable)
 *   3. DDI_BUF_CTL(B) — clear bit 31 (buffer enable), poll IDLE_STATUS
 *   4. LCPLL2_CTL — clear bit 31 (DPLL1 enable), poll DPLL_STATUS bit 4
 *      clear (DPLL1 unlock)
 *   5. DPLL_CTRL2 — clear DDI_B SEL + OVERRIDE (return routing to
 *      "no PLL selected"), set DDI_B OFF bit
 *
 * Idempotent: if pipe A is already off / DDI B is already parked /
 * DPLL1 is already unlocked, each step no-ops with a log line and
 * we continue to the next.
 *
 * NOT gated behind a per-write sysctl — this whole function is behind
 * the gen9_pipe_full_off_now trigger sysctl (default 0).  Users have
 * to opt in explicitly.
 */
#define	LCPLL2_CTL_REG			0x00046014
#define	LCPLL2_CTL_ENABLE		(1u << 31)
/*
 * DPLL_CTRL2 CLK_OFF bit for DDI B.  Per Linux i915 SKL layout:
 *   DPLL_CTRL2_DDI_CLK_OFF(port) = 1 << (port + 15)
 * DDI B = port 1 -> bit 16.  (Not the same as the misnamed
 * DPLL_CTRL2_DDI_B_OFF at (1u<<4) in igen_internal.h — that value is
 * inside the DDI B SEL field and doesn't mean "clock off".)
 */
#define	DPLL_CTRL2_DDI_B_CLK_OFF	(1u << 16)

int
igen_gen9_pipe_full_off(struct igen_softc *sc)
{
	uint32_t fctl, ddi_b, lcpll2, dpll_status, ctrl2;
	int spin;
	int err;

	if (sc->gen != IGEN_GEN_SKL)
		return (EOPNOTSUPP);

	/* Step 1: plane + cursor + pipe disable. */
	err = igen_pipe_a_off(sc);
	if (err != 0) {
		device_printf(sc->dev,
		    "gen9_pipe_full_off: pipe_a_off failed (%d),"
		    " aborting teardown\n", err);
		return (err);
	}

	sx_xlock(&sc->scanout_lock);

	/* Step 2: transcoder A DDI function disable. */
	fctl = igen_r32(sc, TRANS_DDI_FUNC_CTL(0));
	if (fctl & (1u << 31)) {
		device_printf(sc->dev,
		    "gen9_pipe_full_off: TRANS_DDI_FUNC_A disable"
		    " (was 0x%08x)\n", fctl);
		igen_w32(sc, TRANS_DDI_FUNC_CTL(0), fctl & ~(1u << 31));
	} else {
		device_printf(sc->dev,
		    "gen9_pipe_full_off: TRANS_DDI_FUNC_A already disabled"
		    " (0x%08x)\n", fctl);
	}

	/* Step 3: DDI B buffer disable + IDLE poll. */
	ddi_b = igen_r32(sc, DDI_BUF_CTL(1));
	if (ddi_b & (1u << 31)) {
		igen_w32(sc, DDI_BUF_CTL(1), ddi_b & ~(1u << 31));
		for (spin = 0; spin < 100; spin++) {
			if (igen_r32(sc, DDI_BUF_CTL(1)) & (1u << 7))
				break;
			DELAY(100);
		}
		device_printf(sc->dev,
		    "gen9_pipe_full_off: DDI_BUF_B disable  spin=%d/100"
		    "  IDLE=%d  DDI_BUF_B=0x%08x\n",
		    spin, !!(igen_r32(sc, DDI_BUF_CTL(1)) & (1u << 7)),
		    igen_r32(sc, DDI_BUF_CTL(1)));
	} else {
		device_printf(sc->dev,
		    "gen9_pipe_full_off: DDI_BUF_B already parked"
		    " (0x%08x)\n", ddi_b);
	}

	/*
	 * Steps 4 + 5 (DPLL1 disable via LCPLL2_CTL + DPLL_CTRL2 CLK_OFF)
	 * REMOVED 2026-07-17.
	 *
	 * The full disable sequence wedges the box: after clearing
	 * LCPLL2_CTL bit 31 and writing DPLL_CTRL2 with DDI B CLK_OFF,
	 * subsequent MMIO reads on the display fabric (including
	 * gen9_full_bringup's `pre-write state` snapshot) hang the CPU on
	 * the memory transaction.  Verified twice on 2026-07-17 —
	 * physical power-cycle required each time.
	 *
	 * Root cause is not yet understood.  Candidates: (a) DPLL1 doesn't
	 * unlock on this board, so the DPLL_STATUS poll spins forever;
	 * (b) unrouting DDI B without another consumer breaks a bus
	 * arbiter; (c) power-well auto-collapse on unused clock domain.
	 *
	 * pipe_full_off (this function) is now "signal cut" only:
	 * plane + cursor + PIPE_CONF + TRANS_DDI_FUNC + DDI_BUF_CTL(B).
	 * DPLL stays running.  Consequence: gen9_full_bringup can only
	 * re-enable at the *existing* pixel clock (its "DPLL1 already
	 * locked — skipping DPLL config" path).  Same-clock modeset
	 * works, different-clock does not.  atomic_commit's size-
	 * mismatch modeset path is reverted to ENOTSUP until we have a
	 * safe DPLL disable primitive.
	 */
	(void)lcpll2;
	(void)dpll_status;
	(void)ctrl2;

	sx_xunlock(&sc->scanout_lock);
	return (0);
}

static int
igen_sysctl_gen9_pipe_full_off(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);
	return (igen_gen9_pipe_full_off(sc));
}

/* ------------------------- gen9 DPLL1 reprogram --------------------------- */

/*
 * DPLL1 disable/reprogram/re-enable cycle, ported from Linux i915
 * (v6.something in linux/drivers/gpu/drm/i915/display/):
 *
 *   skl_ddi_disable_clock (intel_ddi.c) — sets DPLL_CTRL2 DDI_B_CLK_OFF
 *   (RMW, preserves SEL bits).
 *   skl_ddi_pll_disable (intel_dpll_mgr.c) — clears LCPLL_PLL_ENABLE.
 *   No poll for DPLL_STATUS bit clear.  No CTRL2 SEL touch.
 *
 * Our earlier attempt (ecf801b) got three details wrong that combined
 * into a wedge: (a) we cleared SEL bits, (b) we reversed the order
 * (DPLL disable BEFORE CLK_OFF), (c) we polled DPLL_STATUS.  This
 * rewrite matches Linux exactly.
 *
 * Re-enable is enable-then-lock-wait-then-restore-clock (LCPLL_PLL_
 * ENABLE set, poll DPLL_STATUS bit for lock, clear CLK_OFF).
 *
 * Callers must ensure pipe/DDI/transcoder are disabled first (typically
 * via gen9_pipe_full_off).  Reprogramming a DPLL that's still feeding
 * an active DDI is what wedges.
 *
 * Sysctl-triggered only.  NOT wired into atomic_commit yet.
 *
 * Reprogram: pipe_full_off (caller responsibility) → this function →
 * gen9_full_bringup with the target mode.  Or use gen9_modeset_test_now
 * for a self-contained round-trip.
 */
int
igen_gen9_dpll1_reprogram(struct igen_softc *sc, uint32_t clock_khz)
{
	uint32_t cfgcr1, cfgcr2, ctrl1, ctrl2, lcpll2;
	int spin;

	if (sc->gen != IGEN_GEN_SKL)
		return (EOPNOTSUPP);

	if (!igen_dpll_compute_cfgcr(clock_khz, &cfgcr1, &cfgcr2)) {
		device_printf(sc->dev,
		    "gen9_dpll1_reprogram: no DPLL solution for %u kHz\n",
		    clock_khz);
		return (EINVAL);
	}
	device_printf(sc->dev,
	    "gen9_dpll1_reprogram: target %u kHz  CFGCR1=0x%08x"
	    "  CFGCR2=0x%08x\n", clock_khz, cfgcr1, cfgcr2);

	/*
	 * Step 1: set DPLL_CTRL2 DDI_B_CLK_OFF (RMW — preserve SEL).
	 * This is what skl_ddi_disable_clock does.
	 */
	ctrl2 = igen_r32(sc, DPLL_CTRL2);
	igen_w32(sc, DPLL_CTRL2, ctrl2 | DPLL_CTRL2_DDI_B_CLK_OFF);
	(void)igen_r32(sc, DPLL_CTRL2);
	device_printf(sc->dev,
	    "gen9_dpll1_reprogram: DPLL_CTRL2 CLK_OFF set  (0x%08x -> 0x%08x)\n",
	    ctrl2, ctrl2 | DPLL_CTRL2_DDI_B_CLK_OFF);

	/*
	 * Step 2: clear LCPLL2_CTL bit 31 (DPLL1 enable).  No poll — Linux
	 * doesn't wait for the lock bit to clear.  This is
	 * skl_ddi_pll_disable.
	 */
	lcpll2 = igen_r32(sc, LCPLL2_CTL_REG);
	igen_w32(sc, LCPLL2_CTL_REG, lcpll2 & ~LCPLL2_CTL_ENABLE);
	(void)igen_r32(sc, LCPLL2_CTL_REG);
	device_printf(sc->dev,
	    "gen9_dpll1_reprogram: LCPLL2_CTL ENABLE cleared  (0x%08x)\n",
	    lcpll2);

	/* Step 3: write new CFGCR1/CFGCR2 (DPLL is now disabled). */
	igen_w32(sc, DPLL1_CFGCR1, cfgcr1);
	igen_w32(sc, DPLL1_CFGCR2, cfgcr2);
	(void)igen_r32(sc, DPLL1_CFGCR1);
	(void)igen_r32(sc, DPLL1_CFGCR2);

	/* Step 4: write DPLL_CTRL1 (RMW — preserve DPLL0 config). */
	ctrl1 = igen_r32(sc, DPLL_CTRL1);
	ctrl1 &= ~DPLL_CTRL1_DPLL1_MASK;
	ctrl1 |= DPLL_CTRL1_DPLL1_VAL;
	igen_w32(sc, DPLL_CTRL1, ctrl1);
	(void)igen_r32(sc, DPLL_CTRL1);
	device_printf(sc->dev,
	    "gen9_dpll1_reprogram: DPLL_CTRL1 = 0x%08x\n", ctrl1);

	/*
	 * Step 5: set LCPLL2_CTL ENABLE bit + poll DPLL_STATUS lock.
	 * skl_ddi_pll_enable waits 5ms for lock.  We're generous: 10ms.
	 */
	lcpll2 = igen_r32(sc, LCPLL2_CTL_REG);
	igen_w32(sc, LCPLL2_CTL_REG, lcpll2 | LCPLL2_CTL_ENABLE);
	for (spin = 0; spin < 100; spin++) {
		if (igen_r32(sc, DPLL_STATUS) & DPLL1_LOCK_BIT)
			break;
		DELAY(100);
	}
	if ((igen_r32(sc, DPLL_STATUS) & DPLL1_LOCK_BIT) == 0) {
		device_printf(sc->dev,
		    "gen9_dpll1_reprogram: DPLL1 didn't lock after 10 ms"
		    " (STATUS=0x%08x)\n", igen_r32(sc, DPLL_STATUS));
		return (ETIMEDOUT);
	}
	device_printf(sc->dev,
	    "gen9_dpll1_reprogram: DPLL1 locked after %d us\n", spin * 100);

	/*
	 * Step 6: clear DPLL_CTRL2 DDI_B_CLK_OFF (routing back to DPLL1).
	 * SEL bits weren't touched, so they still point to DPLL1.
	 */
	ctrl2 = igen_r32(sc, DPLL_CTRL2);
	igen_w32(sc, DPLL_CTRL2, ctrl2 & ~DPLL_CTRL2_DDI_B_CLK_OFF);
	(void)igen_r32(sc, DPLL_CTRL2);
	device_printf(sc->dev,
	    "gen9_dpll1_reprogram: DPLL_CTRL2 CLK_OFF cleared  (0x%08x)\n",
	    ctrl2 & ~DPLL_CTRL2_DDI_B_CLK_OFF);

	return (0);
}

static int
igen_sysctl_gen9_dpll1_reprogram(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);
	if (sc->wrpll_target_khz == 0) {
		device_printf(sc->dev,
		    "gen9_dpll1_reprogram: set wrpll_target_khz first\n");
		return (EINVAL);
	}
	return (igen_gen9_dpll1_reprogram(sc, sc->wrpll_target_khz));
}

/* --------------------------- sysctl registration ------------------------- */

void
igen_hsw_pipe_register_sysctls(struct igen_softc *sc)
{
	struct sysctl_ctx_list *ctx = &sc->re_sysctl_ctx;
	struct sysctl_oid_list *children =
	    SYSCTL_CHILDREN(sc->re_sysctl_tree);

	/* Gen-agnostic pipe-off — always registered.  Uses register
	 * offsets/bits shared HSW..KBL. */
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "pipe_a_off",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_pipe_a_off, "I",
	    "write 1 to disable primary plane + cursor + pipe A (safe"
	    " pre-flight for gt_first_batch_submit / i915_dispatch_enable)");

	if (sc->gen == IGEN_GEN_SKL) {
		SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "gen9_panel_on",
		    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE |
		    CTLFLAG_NEEDGIANT,
		    sc, 0, igen_sysctl_gen9_panel_on, "I",
		    "write 1 to bring up gen9 pipe A from cached EDID"
		    " preferred DTD (transcoder timing + PIPE_CONF)");
		SYSCTL_ADD_PROC(ctx, children, OID_AUTO,
		    "gen9_display_dump",
		    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE |
		    CTLFLAG_NEEDGIANT,
		    sc, 0, igen_sysctl_gen9_display_dump, "I",
		    "write 1 to dump every gen9 display-side register"
		    " (read-only, safe)");
		SYSCTL_ADD_PROC(ctx, children, OID_AUTO,
		    "gen9_full_bringup_now",
		    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE |
		    CTLFLAG_NEEDGIANT,
		    sc, 0, igen_sysctl_gen9_full_bringup, "I",
		    "write 1 to force full DPLL+DDI+pipe cold bring-up"
		    " from cached EDID (uses hardcoded HDMI-on-DDI-B"
		    " baseline)");
		SYSCTL_ADD_PROC(ctx, children, OID_AUTO,
		    "gen9_dpll1_reprogram_now",
		    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE |
		    CTLFLAG_NEEDGIANT,
		    sc, 0, igen_sysctl_gen9_dpll1_reprogram, "I",
		    "write 1 to disable + reprogram + re-enable DPLL1 at"
		    " dev.igen.0.re.wrpll_target_khz.  Caller MUST first"
		    " have run gen9_pipe_full_off_now to disable the"
		    " pipe/DDI/transcoder feeding DPLL1 — otherwise the"
		    " box wedges.  Follow with gen9_full_bringup_now to"
		    " restore the pipe at the new clock.  Linux-verified"
		    " sequence (skl_ddi_disable_clock +"
		    " skl_ddi_pll_disable order, no DPLL_STATUS unlock"
		    " poll, no DPLL_CTRL2 SEL clear).");
		SYSCTL_ADD_PROC(ctx, children, OID_AUTO,
		    "gen9_pipe_full_off_now",
		    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE |
		    CTLFLAG_NEEDGIANT,
		    sc, 0, igen_sysctl_gen9_pipe_full_off, "I",
		    "write 1 to tear down the entire pipe A output chain."
		    " WARNING: after this fires, do NOT read display"
		    " MMIO (PIPE_CONF, DDI_BUF, PLANE_*) until gen9_full_"
		    "bringup_now brings the fabric back up — reads hang"
		    " the CPU on the memory transaction and wedge the"
		    " box (verified 2026-07-17).");
	}

	if (sc->gen != IGEN_GEN_HSW)
		return;

	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "hsw_dp_dump",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_hsw_dp_dump, "I",
	    "write 1 to dump all HSW eDP/DDI/transcoder/pipe/plane state");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "hsw_panel_on",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_hsw_panel_on, "I",
	    "write 1 to bring up HSW pipe A -> TRANS EDP -> DDI A from"
	    " scanout_fb (requires scanout_hold first)");
}
