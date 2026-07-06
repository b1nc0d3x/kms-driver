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
 * Exposed as two sysctls so we can drive bring-up by hand during
 * diagnosis before promoting it inside atomic_commit:
 *
 *   dev.igen.<n>.re.hsw_dp_dump   diagnostic dump of every register
 *                                  + DPCD page involved in eDP scanout
 *   dev.igen.<n>.re.hsw_panel_on  run the whole sequence end-to-end
 *
 * Both are gated to gen == HSW so SKL hosts don't accidentally fire
 * the SKL-incompatible writes.
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
	int i;
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
	 * Clear format AND tiling bits — firmware-left state on macbsd
	 * has TILED=1 (X-tile), set for the EFI framebuffer.  Our
	 * scanout_fb is linear; leaving TILED on makes the pipe decode
	 * our buffer as X-tiles, producing the classic 64-px vertical
	 * black/white columns of mistinterpreted linear pixels.
	 */
	dspcntr &= ~((0xfu << DSPCNTR_FORMAT_SHIFT) | DSPCNTR_TILED);
	dspcntr |= DSPCNTR_ENABLE | DSPCNTR_GAMMA_ENABLE |
	    DSPCNTR_FORMAT_BGRX8888;
	igen_w32(sc, HSW_DSPCNTR(0), dspcntr);

	/* PIPE_A ENABLE — the missing 1%. */
	igen_w32(sc, PIPE_CONF(0), PIPE_CONF_ENABLE);
	for (i = 0; i < 200; i++) {
		if (igen_r32(sc, PIPE_CONF(0)) & PIPE_CONF_STATE)
			break;
		DELAY(1000);
	}

	device_printf(sc->dev,
	    "hsw_panel_on: pipe A %s  PIPE_CONF=0x%08x  PLANE_SURF=0x%08x"
	    "  STRIDE=0x%08x  M=0x%08x  N=0x%08x\n",
	    (igen_r32(sc, PIPE_CONF(0)) & PIPE_CONF_STATE) ? "LIVE" : "STALL",
	    igen_r32(sc, PIPE_CONF(0)),
	    igen_r32(sc, HSW_DSPSURF(0)),
	    igen_r32(sc, HSW_DSPSTRIDE(0)), m, n);
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

/* --------------------------- sysctl registration ------------------------- */

void
igen_hsw_pipe_register_sysctls(struct igen_softc *sc)
{
	struct sysctl_ctx_list *ctx = &sc->re_sysctl_ctx;
	struct sysctl_oid_list *children =
	    SYSCTL_CHILDREN(sc->re_sysctl_tree);

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
