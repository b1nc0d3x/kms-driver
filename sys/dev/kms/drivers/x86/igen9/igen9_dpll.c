/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * igen9 DPLL / WRPLL / pipe-resume.  Split out of igen9.c for
 * size + topical cohesion: CDCLK + LCPLL/WRPLL + DPLL_CTRL[12] +
 * DDI_BUF_TRANS voltage swing + pipe + transcoder + plane writes
 * for trying to re-arm scanout from a cold pipe (try_pipe_resume),
 * plus the polled vblank wait that atomic_commit uses to lock the
 * PLANE_SURF write to a vblank edge.
 *
 * Sysctls registered here:
 *   dev.igen9.<n>.re.clock_state          CDCLK + LCPLL + DPLL state
 *   dev.igen9.<n>.re.wrpll_target_khz     RW solver target
 *   dev.igen9.<n>.re.wrpll_calc           solve target -> DCO/P0/P1/P2
 *   dev.igen9.<n>.re.wrpll_dpll_id        2 = DPLL2, 3 = DPLL3
 *   dev.igen9.<n>.re.wrpll_dump           print CFGCR1/CFGCR2
 *   dev.igen9.<n>.re.wrpll_program        program CFGCR1/CFGCR2
 *   dev.igen9.<n>.re.wrpll_route_port     RW (0=A..4=E)
 *   dev.igen9.<n>.re.wrpll_enable         CTRL1 + ENABLE + LOCK poll
 *   dev.igen9.<n>.re.wrpll_disable
 *   dev.igen9.<n>.re.wrpll_route          CTRL2 select + OVERRIDE
 *   dev.igen9.<n>.re.wrpll_unroute
 *   dev.igen9.<n>.re.wrpll_force_clear    emergency ENABLE clear
 *   dev.igen9.<n>.re.pw1_up               PW1 power-well request
 *   dev.igen9.<n>.re.try_pipe_resume      cold pipe re-arm experiment
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/proc.h>
#include <sys/sysctl.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>

#include <kms/drm_atomic.h>
#include <kms/drm_connector.h>
#include <kms/drm_crtc.h>
#include <kms/drm_device.h>
#include <kms/drm_drv.h>
#include <kms/drm_encoder.h>
#include <kms/drm_framebuffer.h>
#include <kms/drm_modes.h>
#include <kms/drm_plane.h>

#include "igen9_internal.h"

/* --------------------------- CDCLK / DPLL readback ------------------------ */

/*
 * SKL+ Central Display Clock.  Per BSpec:
 *   CDCLK_CTL bits[26:24] = CD2X_DIVIDER_SELECT
 *     000 = div1, 001 = div1.5, 010 = div2, 011 = div4
 *   CDCLK_CTL bits[10:0]  = CD_FREQ_DECIMAL
 *     a one-less-than-twice-frequency encoding; e.g. 337.5 MHz = 0x0a8
 *
 * SKL CDCLK_FREQ tier table (per BSpec):
 *   337.5 MHz   freq_dec = 0x344  cd2x = div1
 *   450 MHz     freq_dec = 0x468  cd2x = div1
 *   540 MHz     freq_dec = 0x540  cd2x = div1
 *   675 MHz     freq_dec = 0x650  cd2x = div1
 */
#define	CDCLK_CTL		0x00046000

static const char *
igen9_cdclk_decode(uint32_t cdclk_ctl)
{
	/*
	 * freq_decimal field encodes (2*MHz - 2) so:
	 *   337.5 MHz -> 0x2A1, 450 MHz -> 0x382, 540 MHz -> 0x434,
	 *   617.143 MHz -> 0x4D2, 675 MHz -> 0x544.
	 * The 0x000a8 / 0x000a4 pre-SKL tier values were i915-internal
	 * constants for a separate encoding; SKL+ uses these.
	 */
	uint32_t freq_dec = cdclk_ctl & 0x7ff;
	switch (freq_dec) {
	case 0x2a1: return "337.5 MHz";
	case 0x382: return "450 MHz";
	case 0x434: return "540 MHz";
	case 0x4d2: return "617.143 MHz";
	case 0x544: return "675 MHz";
	default:  return "unknown";
	}
}

/*
 * DPLL routing on SKL+: each port reads its clock from a DPLL via the
 * DPLL_CTRL2 SELECT bits.  DPLL_CTRL1 carries the link rate per DPLL.
 */
#define	DPLL_CTRL1		0x00006c058
#define	DPLL_CTRL2		0x00006c05c
#define	LCPLL1_CTL		0x00046010
#define	LCPLL2_CTL		0x00046014

static int
igen9_sysctl_clock_state(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	uint32_t cdclk = igen9_r32(sc, CDCLK_CTL);
	uint32_t lcpll1 = igen9_r32(sc, LCPLL1_CTL);
	uint32_t lcpll2 = igen9_r32(sc, LCPLL2_CTL);
	uint32_t dpll1 = igen9_r32(sc, DPLL_CTRL1);
	uint32_t dpll2 = igen9_r32(sc, DPLL_CTRL2);

	device_printf(sc->dev,
	    "clock: CDCLK_CTL=0x%08x  (%s, cd2x_div=%u, freq_dec=0x%03x)\n",
	    cdclk, igen9_cdclk_decode(cdclk),
	    (cdclk >> 22) & 0x7, cdclk & 0x7ff);
	device_printf(sc->dev,
	    "clock: LCPLL1_CTL=0x%08x  LCPLL2_CTL=0x%08x\n",
	    lcpll1, lcpll2);
	device_printf(sc->dev,
	    "clock: DPLL_CTRL1=0x%08x  DPLL_CTRL2=0x%08x\n", dpll1, dpll2);

	/*
	 * DPLL_CTRL2 layout (SKL+, per i915 reg defs):
	 *   per DDI port idx 0..4 (A..E):
	 *     bit (port*3)     = SEL_OVERRIDE (1 = SELECT bits are authoritative)
	 *     bits (port*3+1)+ = CLOCK_SELECT (2 bits: 0=DPLL0..3=DPLL3)
	 *     bit (15+port)    = CLOCK_OFF (1 = gated)
	 *
	 * Earlier code used `3+port` for OFF -- that overlapped OVERRIDE/SELECT
	 * fields and silently misreported live state.
	 */
	for (int port = 0; port < 5; port++) {
		uint32_t off = (dpll2 >> (15 + port)) & 1;
		uint32_t sel = (dpll2 >> (1 + port * 3)) & 0x3;
		uint32_t ovr = (dpll2 >> (port * 3)) & 1;
		device_printf(sc->dev,
		    "  DDI_%c: %s  DPLL_SEL=%u (DPLL%u)%s\n",
		    'A' + port, off ? "CLOCK_OFF" : "clock-on", sel, sel,
		    ovr ? "  OVERRIDE" : "");
	}
	return (0);
}

/* --------------------------- WRPLL solver --------------------------------- */

/*
 * SKL+ WRPLL (DPLL2 / DPLL3) -- fractional-N synthesizer for arbitrary
 * HDMI pixel clocks.  Solved from BSpec / i915 skl_ddi_calculate_wrpll.
 *
 * Reference clock     : 24 MHz
 * VCO target window   : 8.4 GHz to 9.0 GHz
 * Output chain        : VCO / 5 / (P0 * P1 * P2)
 * Programmed via CFGCR1 (DCO_INTEGER + DCO_FRACTION) and
 *                CFGCR2 (P0_QDIV, P1_KDIV, P2_PDIV encodings).
 *
 * Algorithm:
 *   1) Compute the total divider 'd' such that pll_clock = pixel * 5
 *      and VCO = pll_clock * d lands in [8400, 9000] MHz.
 *   2) Decompose d into (P0, P1, P2) using i915's even/odd table.
 *   3) DCO_INTEGER = VCO_kHz / 24000;
 *      DCO_FRACTION = ((VCO_kHz % 24000) << 15) / 24000.
 *
 * P0 codes: 1->0, 2->1, 3->2, 7->4   (P0_QDIV in CFGCR2)
 * P2 codes: 5->0, 2->1, 3->2, 1->3   (P2_PDIV in CFGCR2)
 * P1 is the K-divider, 1..8 raw.
 */
#define	WRPLL_VCO_MIN_KHZ	8400000
#define	WRPLL_VCO_MAX_KHZ	9000000
#define	WRPLL_REF_KHZ		24000

static const uint8_t wrpll_even_dividers[] = {
	4, 6, 8, 10, 12, 14, 16, 18, 20, 24, 28, 30, 32, 36, 40, 42,
	44, 48, 52, 54, 56, 60, 64, 66, 68, 70, 72, 76, 78, 80, 84,
	88, 90, 92, 96, 98
};
static const uint8_t wrpll_odd_dividers[] = { 3, 5, 7, 9, 15, 21, 35 };

static bool
igen9_wrpll_decompose(uint32_t d, uint8_t *p0, uint8_t *p1, uint8_t *p2)
{
	if ((d % 2) == 0) {
		uint32_t half = d / 2;

		if (half == 1 || half == 2 || half == 3 || half == 5) {
			*p0 = 2;
			*p1 = 1;
			*p2 = (uint8_t)half;
		} else if ((half % 2) == 0) {
			*p0 = 2;
			*p1 = (uint8_t)(half / 2);
			*p2 = 2;
		} else if ((half % 3) == 0) {
			*p0 = 3;
			*p1 = (uint8_t)(half / 3);
			*p2 = 2;
		} else if ((half % 7) == 0) {
			*p0 = 7;
			*p1 = (uint8_t)(half / 7);
			*p2 = 2;
		} else {
			return (false);
		}
		return (true);
	}
	switch (d) {
	case 3:  *p0 = 3; *p1 = 1; *p2 = 1; return (true);
	case 5:  *p0 = 5; *p1 = 1; *p2 = 1; return (true);
	case 7:  *p0 = 7; *p1 = 1; *p2 = 1; return (true);
	case 9:  *p0 = 3; *p1 = 1; *p2 = 3; return (true);
	case 15: *p0 = 3; *p1 = 1; *p2 = 5; return (true);
	case 21: *p0 = 7; *p1 = 1; *p2 = 3; return (true);
	case 35: *p0 = 7; *p1 = 1; *p2 = 5; return (true);
	}
	return (false);
}

static bool
igen9_wrpll_solve(uint32_t pixel_khz, uint8_t *out_p0, uint8_t *out_p1,
    uint8_t *out_p2, uint16_t *out_dco_int, uint16_t *out_dco_frac,
    uint64_t *out_vco_khz)
{
	uint32_t pll_khz = pixel_khz * 5;
	uint32_t best_d = 0;
	uint64_t best_vco = 0;
	uint64_t best_dev = UINT64_MAX;
	uint64_t center = ((uint64_t)WRPLL_VCO_MIN_KHZ +
	    WRPLL_VCO_MAX_KHZ) / 2;
	uint8_t p0 = 0, p1 = 0, p2 = 0;

	for (size_t i = 0; i < nitems(wrpll_even_dividers); i++) {
		uint32_t d = wrpll_even_dividers[i];
		uint64_t vco = (uint64_t)pll_khz * d;
		uint8_t a, b, c;
		uint64_t dev;

		if (vco < WRPLL_VCO_MIN_KHZ || vco > WRPLL_VCO_MAX_KHZ)
			continue;
		if (!igen9_wrpll_decompose(d, &a, &b, &c))
			continue;
		dev = vco > center ? vco - center : center - vco;
		if (dev < best_dev) {
			best_dev = dev;
			best_d = d;
			best_vco = vco;
			p0 = a;
			p1 = b;
			p2 = c;
		}
	}
	for (size_t i = 0; i < nitems(wrpll_odd_dividers); i++) {
		uint32_t d = wrpll_odd_dividers[i];
		uint64_t vco = (uint64_t)pll_khz * d;
		uint8_t a, b, c;
		uint64_t dev;

		if (vco < WRPLL_VCO_MIN_KHZ || vco > WRPLL_VCO_MAX_KHZ)
			continue;
		if (!igen9_wrpll_decompose(d, &a, &b, &c))
			continue;
		dev = vco > center ? vco - center : center - vco;
		if (dev < best_dev) {
			best_dev = dev;
			best_d = d;
			best_vco = vco;
			p0 = a;
			p1 = b;
			p2 = c;
		}
	}
	if (best_d == 0)
		return (false);

	*out_p0 = p0;
	*out_p1 = p1;
	*out_p2 = p2;
	*out_dco_int = (uint16_t)(best_vco / WRPLL_REF_KHZ);
	*out_dco_frac = (uint16_t)(((best_vco % WRPLL_REF_KHZ) << 15) /
	    WRPLL_REF_KHZ);
	*out_vco_khz = best_vco;
	return (true);
}

static int
igen9_sysctl_wrpll_calc(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	uint8_t p0, p1, p2;
	uint16_t dco_int, dco_frac;
	uint64_t vco;
	uint32_t pixel_khz;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	pixel_khz = sc->wrpll_target_khz;
	if (pixel_khz == 0) {
		device_printf(sc->dev,
		    "wrpll: set dev.igen9.0.re.wrpll_target_khz first\n");
		return (0);
	}
	if (!igen9_wrpll_solve(pixel_khz, &p0, &p1, &p2,
	    &dco_int, &dco_frac, &vco)) {
		device_printf(sc->dev,
		    "wrpll: no solution for %u kHz (VCO out of range)\n",
		    pixel_khz);
		return (0);
	}
	device_printf(sc->dev,
	    "wrpll: target=%u kHz  VCO=%llu kHz  div=%u (P0=%u P1=%u P2=%u)\n",
	    pixel_khz, (unsigned long long)vco, p0 * p1 * p2, p0, p1, p2);
	device_printf(sc->dev,
	    "wrpll: DCO_INTEGER=%u (0x%03x)  DCO_FRACTION=%u (0x%04x)\n",
	    dco_int, dco_int, dco_frac, dco_frac);
	device_printf(sc->dev,
	    "wrpll: verify pixel = VCO / 5 / div = %llu kHz\n",
	    (unsigned long long)(vco / 5 / (p0 * p1 * p2)));
	return (0);
}

/* --------------------------- WRPLL programming ---------------------------- */

/*
 * SKL+ WRPLL programming.  BSpec layout:
 *
 *   DPLL2_CFGCR1 = 0x6C040    DPLL2_CFGCR2 = 0x6C044    (WRPLL1)
 *   DPLL3_CFGCR1 = 0x6C048    DPLL3_CFGCR2 = 0x6C04C    (WRPLL2)
 *
 *   CFGCR1:
 *     bit 31         FREQ_ENABLE        must be set when valid
 *     bits 23:9      DCO_FRACTION       15-bit fractional part
 *     bits 8:0       DCO_INTEGER        9-bit integer part
 *
 *   CFGCR2:
 *     bits 15:8      QDIV_RATIO         P1 (Q) value, raw 1..255
 *     bit 7          QDIV_MODE          1 = Q divider engaged (P1>1)
 *     bits 6:5       KDIV               P2 encoded:
 *                                         0=K5, 1=K2, 2=K3, 3=K1
 *     bits 4:2       PDIV               P0 encoded:
 *                                         0=P1, 1=P2, 2=P3, 4=P7
 *     bits 1:0       CENTRAL_FREQ       00=9.6 GHz, 01=9.0 GHz, 11=8.4 GHz
 *
 * For HDMI WRPLL we always select 8.4 GHz central (CF=3): the VCO range
 * [8.4, 9.0] GHz brackets it and the solver targets the centre.
 */
#define	WRPLL_CFGCR1(id)	(0x6c040u + ((id) - 2u) * 8u)
#define	WRPLL_CFGCR2(id)	(0x6c044u + ((id) - 2u) * 8u)

#define	CFGCR1_FREQ_ENABLE	(1u << 31)
#define	CFGCR2_QDIV_RATIO_SHIFT	8
#define	CFGCR2_QDIV_MODE	(1u << 7)
#define	CFGCR2_KDIV_SHIFT	5
#define	CFGCR2_PDIV_SHIFT	2

static uint32_t
igen9_wrpll_encode_cfgcr1(uint16_t dco_int, uint16_t dco_frac)
{
	return (CFGCR1_FREQ_ENABLE |
	    ((uint32_t)(dco_frac & 0x7fffu) << 9) |
	    (uint32_t)(dco_int & 0x1ffu));
}

/*
 * Central frequency hint: tells the analog which of 9.6 / 9.0 / 8.4 GHz
 * to bias the VCO around.  Pick the nearest.  Firmware on KBL-S 8086:5912
 * uses 9.0 GHz for an 8910 MHz VCO, confirming this rule.
 */
static uint32_t
igen9_wrpll_central_freq_bits(uint64_t vco_khz)
{
	uint64_t d96 = vco_khz > 9600000 ? vco_khz - 9600000 :
	    9600000 - vco_khz;
	uint64_t d90 = vco_khz > 9000000 ? vco_khz - 9000000 :
	    9000000 - vco_khz;
	uint64_t d84 = vco_khz > 8400000 ? vco_khz - 8400000 :
	    8400000 - vco_khz;

	if (d84 <= d90 && d84 <= d96)
		return (3);	/* 8.4 GHz */
	if (d90 <= d96)
		return (1);	/* 9.0 GHz */
	return (0);		/* 9.6 GHz */
}

static bool
igen9_wrpll_encode_cfgcr2(uint8_t p0, uint8_t p1, uint8_t p2,
    uint64_t vco_khz, uint32_t *out)
{
	uint32_t v = 0;
	uint32_t pdiv;
	uint32_t kdiv;

	switch (p0) {
	case 1: pdiv = 0; break;
	case 2: pdiv = 1; break;
	case 3: pdiv = 2; break;
	case 7: pdiv = 4; break;
	default: return (false);
	}
	switch (p2) {
	case 5: kdiv = 0; break;
	case 2: kdiv = 1; break;
	case 3: kdiv = 2; break;
	case 1: kdiv = 3; break;
	default: return (false);
	}
	v |= ((uint32_t)p1 << CFGCR2_QDIV_RATIO_SHIFT);
	if (p1 > 1)
		v |= CFGCR2_QDIV_MODE;
	v |= (kdiv << CFGCR2_KDIV_SHIFT);
	v |= (pdiv << CFGCR2_PDIV_SHIFT);
	v |= igen9_wrpll_central_freq_bits(vco_khz);
	*out = v;
	return (true);
}

static void
igen9_wrpll_decode_cfgcr(uint32_t cfgcr1, uint32_t cfgcr2,
    device_t dev)
{
	static const uint8_t kdiv_to_p2[] = { 5, 2, 3, 1 };
	static const uint8_t pdiv_to_p0[] = { 1, 2, 3, 0, 7, 0, 0, 0 };
	uint32_t dco_int  = cfgcr1 & 0x1ffu;
	uint32_t dco_frac = (cfgcr1 >> 9) & 0x7fffu;
	uint32_t p1 = (cfgcr2 >> CFGCR2_QDIV_RATIO_SHIFT) & 0xffu;
	uint32_t kdiv = (cfgcr2 >> CFGCR2_KDIV_SHIFT) & 0x3u;
	uint32_t pdiv = (cfgcr2 >> CFGCR2_PDIV_SHIFT) & 0x7u;
	uint8_t p0 = pdiv_to_p0[pdiv];
	uint8_t p2 = kdiv_to_p2[kdiv];
	const char *cf_str;

	switch (cfgcr2 & 3) {
	case 0: cf_str = "9.6 GHz"; break;
	case 1: cf_str = "9.0 GHz"; break;
	case 3: cf_str = "8.4 GHz"; break;
	default: cf_str = "RSVD";
	}

	if (!(cfgcr1 & CFGCR1_FREQ_ENABLE)) {
		device_printf(dev, "  CFGCR1 FREQ_ENABLE=0 (PLL not"
		    " programmed)\n");
		return;
	}
	if (p0 == 0 || p2 == 0) {
		device_printf(dev,
		    "  CFGCR2 pdiv=%u kdiv=%u invalid (raw 0x%08x)\n",
		    pdiv, kdiv, cfgcr2);
		return;
	}
	uint64_t vco = (uint64_t)dco_int * WRPLL_REF_KHZ +
	    ((uint64_t)dco_frac * WRPLL_REF_KHZ) / 32768ull;
	uint32_t divider = (uint32_t)p0 * p1 * p2;
	uint64_t pixel = divider ? vco / 5 / divider : 0;

	device_printf(dev,
	    "  DCO_INT=%u (0x%03x)  DCO_FRAC=%u (0x%04x)  CENTRAL=%s\n",
	    dco_int, dco_int, dco_frac, dco_frac, cf_str);
	device_printf(dev,
	    "  P0=%u  P1=%u (qdiv_mode=%d)  P2=%u  div=%u\n",
	    p0, p1, (cfgcr2 & CFGCR2_QDIV_MODE) ? 1 : 0, p2, divider);
	device_printf(dev,
	    "  VCO=%llu kHz  pixel=%llu kHz\n",
	    (unsigned long long)vco, (unsigned long long)pixel);
}

static int
igen9_sysctl_wrpll_dump(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	uint32_t id, cfgcr1, cfgcr2;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	id = sc->wrpll_dpll_id;
	if (id != 2 && id != 3) {
		device_printf(sc->dev,
		    "wrpll_dump: wrpll_dpll_id must be 2 or 3\n");
		return (0);
	}
	cfgcr1 = igen9_r32(sc, WRPLL_CFGCR1(id));
	cfgcr2 = igen9_r32(sc, WRPLL_CFGCR2(id));
	device_printf(sc->dev,
	    "wrpll DPLL%u: CFGCR1=0x%08x  CFGCR2=0x%08x\n",
	    id, cfgcr1, cfgcr2);
	igen9_wrpll_decode_cfgcr(cfgcr1, cfgcr2, sc->dev);
	return (0);
}

static int
igen9_sysctl_wrpll_program(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	uint8_t p0, p1, p2;
	uint16_t dco_int, dco_frac;
	uint64_t vco;
	uint32_t id, pixel_khz, cfgcr1, cfgcr2;
	uint32_t old1, old2, back1, back2;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	id = sc->wrpll_dpll_id;
	if (id != 2 && id != 3) {
		device_printf(sc->dev,
		    "wrpll: wrpll_dpll_id must be 2 or 3 (got %u)\n", id);
		return (EINVAL);
	}
	/*
	 * Guard rail: DPLL_CTRL2 routing tells us which DDIs are driven
	 * by which PLL.  If any DDI's DPLL_SEL == this PLL and the DDI
	 * is clocked on, the PLL is live -- never reprogram a live PLL.
	 */
	uint32_t ctrl2 = igen9_r32(sc, DPLL_CTRL2);
	for (int port = 0; port < 5; port++) {
		uint32_t off = (ctrl2 >> (15 + port)) & 1;
		uint32_t sel = (ctrl2 >> (1 + port * 3)) & 0x3;
		if (!off && sel == id) {
			device_printf(sc->dev,
			    "wrpll: REFUSE: DDI_%c is using DPLL%u live"
			    " (DPLL_CTRL2=0x%08x)\n",
			    'A' + port, id, ctrl2);
			return (EBUSY);
		}
	}

	pixel_khz = sc->wrpll_target_khz;
	if (pixel_khz == 0) {
		device_printf(sc->dev,
		    "wrpll: set wrpll_target_khz first\n");
		return (EINVAL);
	}
	if (!igen9_wrpll_solve(pixel_khz, &p0, &p1, &p2,
	    &dco_int, &dco_frac, &vco)) {
		device_printf(sc->dev,
		    "wrpll: no solution for %u kHz\n", pixel_khz);
		return (EINVAL);
	}

	cfgcr1 = igen9_wrpll_encode_cfgcr1(dco_int, dco_frac);
	if (!igen9_wrpll_encode_cfgcr2(p0, p1, p2, vco, &cfgcr2)) {
		device_printf(sc->dev,
		    "wrpll: encode failed for P0=%u P1=%u P2=%u\n", p0, p1, p2);
		return (EINVAL);
	}

	old1 = igen9_r32(sc, WRPLL_CFGCR1(id));
	old2 = igen9_r32(sc, WRPLL_CFGCR2(id));
	device_printf(sc->dev,
	    "wrpll DPLL%u: pre  CFGCR1=0x%08x CFGCR2=0x%08x\n",
	    id, old1, old2);
	device_printf(sc->dev,
	    "wrpll DPLL%u: prog CFGCR1=0x%08x CFGCR2=0x%08x"
	    " (target=%u kHz)\n",
	    id, cfgcr1, cfgcr2, pixel_khz);

	igen9_w32(sc, WRPLL_CFGCR1(id), cfgcr1);
	igen9_w32(sc, WRPLL_CFGCR2(id), cfgcr2);

	back1 = igen9_r32(sc, WRPLL_CFGCR1(id));
	back2 = igen9_r32(sc, WRPLL_CFGCR2(id));
	device_printf(sc->dev,
	    "wrpll DPLL%u: post CFGCR1=0x%08x CFGCR2=0x%08x\n",
	    id, back1, back2);
	igen9_wrpll_decode_cfgcr(back1, back2, sc->dev);

	if (back1 != cfgcr1 || back2 != cfgcr2)
		device_printf(sc->dev,
		    "wrpll DPLL%u: WARNING readback mismatch\n", id);
	return (0);
}

/* --------------------------- WRPLL enable / route ------------------------- */

/*
 * Per-DPLL ENABLE register offsets.  DPLL0/1 are LCPLLs (own ctl); DPLL2/3
 * are WRPLLs.  Bit 31 = PLL_ENABLE (RW), bit 30 = PLL_LOCK (RO).
 *
 * From the BSpec PLL enable sequence (SKL+ HDMI path):
 *   1. Program CFGCR1/CFGCR2  (already done)
 *   2. Set DPLL_CTRL1[(id*6)+OVERRIDE] and DPLL_CTRL1[(id*6)+HDMI_MODE]
 *   3. Set ENABLE bit in DPLLx_ENABLE
 *   4. Poll LOCK bit (HW asserts within ~600 us)
 *   5. Re-mux DDI clock via DPLL_CTRL2 (DDI_x SEL=id, OVERRIDE=1, OFF=0)
 *
 * Disable is the reverse: gate the DDI clock first, then clear ENABLE.
 */
#define	WRPLL_ENABLE_REG(id)		((id) == 2 ? 0x46040u : 0x46060u)
#define	WRPLL_ENABLE_BIT		(1u << 31)	/* PLL_ENABLE */
#define	WRPLL_LOCK_BIT			(1u << 30)	/* PLL_LOCK */
#define	WRPLL_POWER_ENABLE_BIT		(1u << 27)	/* PLL_POWER_ENABLE */
#define	WRPLL_POWER_STATE_BIT		(1u << 26)	/* PLL_POWER_STATE */
#define	CTRL1_OVERRIDE(id)		(1u << ((id) * 6))
#define	CTRL1_HDMI_MODE(id)		(1u << ((id) * 6 + 1))
#define	CTRL2_DDI_OVERRIDE(p)		(1u << ((p) * 3))
#define	CTRL2_DDI_SEL_MASK(p)		(3u << ((p) * 3 + 1))
#define	CTRL2_DDI_SEL(id, p)		((uint32_t)(id) << ((p) * 3 + 1))
#define	CTRL2_DDI_OFF(p)		(1u << (15 + (p)))

static int
igen9_sysctl_wrpll_enable(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	uint32_t id, cfgcr1, ctrl1, enreg, en, lock;
	int i;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	id = sc->wrpll_dpll_id;
	if (id != 2 && id != 3) {
		device_printf(sc->dev,
		    "wrpll: wrpll_dpll_id must be 2 or 3\n");
		return (EINVAL);
	}
	cfgcr1 = igen9_r32(sc, WRPLL_CFGCR1(id));
	if (!(cfgcr1 & CFGCR1_FREQ_ENABLE)) {
		device_printf(sc->dev,
		    "wrpll DPLL%u: CFGCR1 FREQ_ENABLE clear; program it"
		    " first\n", id);
		return (EINVAL);
	}

	/*
	 * Step 1.5: PW1 (display PLL power well) must be up.  PW1 is index 1
	 * in HSW_PWR_WELL_CTL2; without it DPLL2/3 can't lock and ENABLE
	 * leaves the silicon in a stuck "enabled-but-not-locked" state.
	 * We refuse rather than risk wedging the box.
	 */
	/* PW1 STATE = bit (idx*2) in HSW_PWR_WELL_CTL2 @ 0x45404; idx=1. */
	uint32_t pwr = igen9_r32(sc, 0x45404);
	if (!(pwr & (1u << 2))) {
		device_printf(sc->dev,
		    "wrpll DPLL%u: REFUSE: PW1 not up (CTL2=0x%08x);"
		    " enable PW1 first\n", id, pwr);
		return (ENXIO);
	}

	/*
	 * Step 2: CTRL1 OVERRIDE + HDMI_MODE for this DPLL.  Clear the
	 * 6-bit per-DPLL field first to wipe any stale link-rate / SSC bits
	 * so the new write is authoritative.  Per BSpec, CTRL1 must be
	 * latched before CFGCR1/CFGCR2 so the PLL knows which fields to
	 * read.  Write CTRL1, posting-read, then re-program CFGCR1/CFGCR2
	 * (already populated by wrpll_program) so the registers are stuffed
	 * AFTER CTRL1 mode select.
	 */
	ctrl1 = igen9_r32(sc, DPLL_CTRL1);
	uint32_t per_dpll_mask = 0x3fu << (id * 6);
	uint32_t new_ctrl1 = (ctrl1 & ~per_dpll_mask) |
	    CTRL1_OVERRIDE(id) | CTRL1_HDMI_MODE(id);

	if (new_ctrl1 != ctrl1) {
		device_printf(sc->dev,
		    "wrpll DPLL%u: CTRL1 0x%08x -> 0x%08x\n",
		    id, ctrl1, new_ctrl1);
		igen9_w32(sc, DPLL_CTRL1, new_ctrl1);
	}
	(void)igen9_r32(sc, DPLL_CTRL1);	/* posting read */

	/* Re-program CFGCR1/CFGCR2 now that CTRL1 selected HDMI_MODE. */
	uint32_t cfgcr2 = igen9_r32(sc, WRPLL_CFGCR2(id));
	igen9_w32(sc, WRPLL_CFGCR1(id), cfgcr1);
	igen9_w32(sc, WRPLL_CFGCR2(id), cfgcr2);
	(void)igen9_r32(sc, WRPLL_CFGCR2(id));
	device_printf(sc->dev,
	    "wrpll DPLL%u: re-stuffed CFGCR1=0x%08x CFGCR2=0x%08x\n",
	    id, cfgcr1, cfgcr2);

	/*
	 * Step 3: ENABLE.  SKL/KBL has no separate POWER_ENABLE/POWER_STATE
	 * handshake -- that was added in ICL and later.  Just set bit 31
	 * and wait for bit 30 (LOCK).
	 */
	enreg = WRPLL_ENABLE_REG(id);
	en = igen9_r32(sc, enreg);
	device_printf(sc->dev,
	    "wrpll DPLL%u: ENABLE_REG[0x%05x]=0x%08x (pre)\n", id, enreg, en);

	/*
	 * Firmware's working LCPLL1_CTL reads back 0xc0000000 when locked
	 * (bit31 ENABLE + bit30 LOCK, nothing else).  My OR-write was
	 * preserving the mystery lower bits (3/4/10/13/21) from the
	 * 0x00202418 pre-state -- those may be RW config that's wrong by
	 * default.  Try writing just bit 31, matching firmware's LCPLL1
	 * layout, to see if clearing the lower bits unblocks lock.
	 */
	if (!(en & WRPLL_ENABLE_BIT))
		igen9_w32(sc, enreg, WRPLL_ENABLE_BIT);
	(void)igen9_r32(sc, enreg);	/* posting read */

	/* Poll LOCK.  Generous 50 ms in case BSpec's 600 us is wrong. */
	for (i = 0; i < 500; i++) {
		lock = igen9_r32(sc, enreg);
		if (lock & WRPLL_LOCK_BIT)
			break;
		DELAY(100);
	}
	device_printf(sc->dev,
	    "wrpll DPLL%u: ENABLE_REG=0x%08x  LOCK=%d  after %d us\n",
	    id, lock, !!(lock & WRPLL_LOCK_BIT), i * 100);

	if (!(lock & WRPLL_LOCK_BIT)) {
		/*
		 * Critical: undo our ENABLE write so the silicon doesn't
		 * sit in "enabled but unlocked" forever.
		 */
		uint32_t v = igen9_r32(sc, enreg);
		igen9_w32(sc, enreg, v & ~WRPLL_ENABLE_BIT);
		device_printf(sc->dev,
		    "wrpll DPLL%u: FAILED to lock; ENABLE cleared\n", id);
		return (EIO);
	}
	return (0);
}

static int
igen9_sysctl_wrpll_disable(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	uint32_t id, ctrl2, enreg, en;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	id = sc->wrpll_dpll_id;
	if (id != 2 && id != 3) {
		device_printf(sc->dev,
		    "wrpll: wrpll_dpll_id must be 2 or 3\n");
		return (EINVAL);
	}
	/* Same liveness guard as wrpll_program. */
	ctrl2 = igen9_r32(sc, DPLL_CTRL2);
	for (int port = 0; port < 5; port++) {
		uint32_t off = (ctrl2 >> (15 + port)) & 1;
		uint32_t sel = (ctrl2 >> (1 + port * 3)) & 0x3;
		if (!off && sel == id) {
			device_printf(sc->dev,
			    "wrpll DPLL%u: REFUSE disable: DDI_%c still routed"
			    " (CTRL2=0x%08x); unroute first\n",
			    id, 'A' + port, ctrl2);
			return (EBUSY);
		}
	}
	enreg = WRPLL_ENABLE_REG(id);
	en = igen9_r32(sc, enreg);
	/*
	 * Clear ENABLE first, brief wait, then clear POWER_ENABLE.  BSpec
	 * disable order is the reverse of enable.
	 */
	uint32_t after_en = en & ~WRPLL_ENABLE_BIT;
	device_printf(sc->dev,
	    "wrpll DPLL%u: disable, ENABLE_REG 0x%08x -> 0x%08x\n",
	    id, en, after_en);
	igen9_w32(sc, enreg, after_en);
	return (0);
}

static int
igen9_sysctl_wrpll_route(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	uint32_t id, port, ctrl2, new_ctrl2, enreg, en;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	id = sc->wrpll_dpll_id;
	port = sc->wrpll_route_port;
	if (id != 2 && id != 3) {
		device_printf(sc->dev,
		    "wrpll_route: wrpll_dpll_id must be 2 or 3\n");
		return (EINVAL);
	}
	if (port > 4) {
		device_printf(sc->dev,
		    "wrpll_route: wrpll_route_port must be 0..4\n");
		return (EINVAL);
	}

	/* Refuse if the PLL isn't locked. */
	enreg = WRPLL_ENABLE_REG(id);
	en = igen9_r32(sc, enreg);
	if (!(en & WRPLL_LOCK_BIT)) {
		device_printf(sc->dev,
		    "wrpll_route: DPLL%u not locked (ENABLE_REG=0x%08x);"
		    " run wrpll_enable first\n", id, en);
		return (EAGAIN);
	}

	ctrl2 = igen9_r32(sc, DPLL_CTRL2);
	new_ctrl2 = ctrl2;
	new_ctrl2 &= ~CTRL2_DDI_SEL_MASK(port);
	new_ctrl2 |= CTRL2_DDI_SEL(id, port);
	new_ctrl2 |= CTRL2_DDI_OVERRIDE(port);
	new_ctrl2 &= ~CTRL2_DDI_OFF(port);

	device_printf(sc->dev,
	    "wrpll_route: DDI_%c -> DPLL%u  CTRL2 0x%08x -> 0x%08x\n",
	    'A' + port, id, ctrl2, new_ctrl2);
	igen9_w32(sc, DPLL_CTRL2, new_ctrl2);

	/* Read-back. */
	uint32_t back = igen9_r32(sc, DPLL_CTRL2);
	uint32_t off  = (back >> (15 + port)) & 1;
	uint32_t sel  = (back >> (1 + port * 3)) & 0x3;
	uint32_t ovr  = (back >> (port * 3)) & 1;
	device_printf(sc->dev,
	    "wrpll_route: post CTRL2=0x%08x  DDI_%c: %s SEL=%u OVERRIDE=%u\n",
	    back, 'A' + port,
	    off ? "CLOCK_OFF" : "clock-on", sel, ovr);
	return (0);
}

static int
igen9_sysctl_wrpll_force_clear(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	uint32_t id, enreg, en;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	id = sc->wrpll_dpll_id;
	if (id != 2 && id != 3) {
		device_printf(sc->dev,
		    "wrpll_force_clear: wrpll_dpll_id must be 2 or 3\n");
		return (EINVAL);
	}
	enreg = WRPLL_ENABLE_REG(id);
	en = igen9_r32(sc, enreg);
	uint32_t cleared = en & ~WRPLL_ENABLE_BIT;
	device_printf(sc->dev,
	    "wrpll_force_clear DPLL%u: ENABLE_REG 0x%08x -> 0x%08x"
	    " (ENABLE cleared)\n", id, en, cleared);
	igen9_w32(sc, enreg, cleared);
	return (0);
}

/*
 * Request PW1 (display PLL power well, idx=1 in HSW_PWR_WELL_CTL2).
 * PW1 powers the analog domain for DPLL2/DPLL3 (the WRPLLs).  Firmware
 * leaves it down when it drives the live link from DPLL0/LCPLL1, so
 * we have to opt in before we can lock a WRPLL.  Layout per port idx:
 *   bit (idx*2)   = STATE  (RO; HW asserts when well is up)
 *   bit (idx*2+1) = REQ    (RW; software sets to request)
 * PW1 STATE = bit 2, REQ = bit 3.  Poll STATE for up to 10 ms.
 */
static int
igen9_sysctl_pw1_up(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	uint32_t v;
	int i, trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	v = igen9_r32(sc, 0x45404);
	device_printf(sc->dev, "pw1_up: HSW_PWR_WELL_CTL2 pre=0x%08x"
	    " (PW1 STATE=%d REQ=%d)\n",
	    v, (v >> 2) & 1, (v >> 3) & 1);

	if ((v & (1u << 2)) != 0) {
		device_printf(sc->dev, "pw1_up: already up\n");
		return (0);
	}

	igen9_w32(sc, 0x45404, v | (1u << 3));	/* set REQ for PW1 */
	for (i = 0; i < 100; i++) {
		v = igen9_r32(sc, 0x45404);
		if (v & (1u << 2))
			break;
		DELAY(100);
	}
	device_printf(sc->dev, "pw1_up: post=0x%08x  STATE=%d  after %d us\n",
	    v, (v >> 2) & 1, i * 100);

	return ((v & (1u << 2)) ? 0 : EIO);
}

static int
igen9_sysctl_wrpll_unroute(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	uint32_t port, ctrl2, new_ctrl2;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	port = sc->wrpll_route_port;
	if (port > 4) {
		device_printf(sc->dev,
		    "wrpll_unroute: wrpll_route_port must be 0..4\n");
		return (EINVAL);
	}
	/* Hard refuse DDI_B -- that's the live firmware-driven panel. */
	if (port == 1) {
		device_printf(sc->dev,
		    "wrpll_unroute: REFUSE: DDI_B carries the live"
		    " firmware-driven mode\n");
		return (EBUSY);
	}
	ctrl2 = igen9_r32(sc, DPLL_CTRL2);
	new_ctrl2 = ctrl2 | CTRL2_DDI_OFF(port);
	device_printf(sc->dev,
	    "wrpll_unroute: DDI_%c  CTRL2 0x%08x -> 0x%08x\n",
	    'A' + port, ctrl2, new_ctrl2);
	igen9_w32(sc, DPLL_CTRL2, new_ctrl2);
	return (0);
}

/* --------------------------- pipe resume ---------------------------------- */

/*
 * Minimal "resume the firmware mode" sequence using BASELINE values
 * captured 2026-06-10 on this board.  Assumes the upstream clocks
 * (CDCLK, LCPLL1, DPLL0/1) are still alive -- the clock_state readback
 * confirms they are.  Writes the BASELINE timing into transcoder A,
 * routes it to DDI_B, points the primary plane at GTT[0], then enables
 * pipe -> plane -> DDI buffer in that order.
 *
 * Doesn't handle from-cold start (would need full CDCLK/DPLL bring-up).
 */

#define	PIPE_SRCSZ(p)		(0x7001c + (p) * 0x1000)
#define	DDI_BUF_CTL(d)		(0x64000 + (d) * 0x100)

/*
 * Power-well control.  Per BSpec / i915 HSW_PWR_WELL_CTL_REQ pattern:
 *   bit (idx*2)   = STATE (RO; HW asserts when the well is up)
 *   bit (idx*2+1) = REQ   (RW; software asserts to request power up)
 * PW1=idx1, PW2=idx2, DDI_A_E=idx3, DDI_B=idx4, DDI_C=idx5, DDI_D=idx6.
 * SKL needs PW2 + per-DDI wells up before pipe/DDI register writes
 * have any effect on the analog side.
 */
#define	HSW_PWR_WELL_CTL2	0x00045404
#define	PWR_WELL_REQ(idx)	(1u << ((idx) * 2 + 1))
#define	PWR_WELL_STATE(idx)	(1u << ((idx) * 2))
#define	PW_IDX_PW2		2
#define	PW_IDX_DDI_B		4

/*
 * SKL DPLL enable / lock registers.
 *   DPLL0 = LCPLL1 (0x46010), DPLL1 = LCPLL2 (0x46014),
 *   DPLL2 = WRPLL1 (0x46040), DPLL3 = WRPLL2 (0x46060).
 * Each: bit 30 = ENABLE (RW), bit 31 = LOCK (RO).
 */
#define	SKL_DPLL0_ENABLE	0x46010
#define	SKL_DPLL1_ENABLE	0x46014
#define	DPLL_ENABLE_BIT		(1u << 31)	/* LCPLL_PLL_ENABLE */
#define	DPLL_LOCK_BIT		(1u << 30)	/* LCPLL_PLL_LOCK */
#define	  DDI_BUF_CTL_ENABLE_BIT		(1u << 31)
#define	  DDI_BUF_CTL_TRANS_SELECT_SHIFT	24
#define	  DDI_BUF_CTL_PORT_WIDTH_X4		(3u << 1)
#define	  DDI_BUF_CTL_INIT_DISPLAY_DETECTED	(1u << 0)
#define	  DDI_BUF_CTL_IDLE_STATUS		(1u << 7)
#define	DPLL_CTRL2_DDI_B_OFF	(1u << 4)

/*
 * DDI_BUF_TRANS table — analog voltage swing / pre-emphasis params
 * per port.  Each port has its own 10-entry bank of (LO, HI) pairs
 * at 0x64E00 + port*0x60.  DDI_BUF_CTL bits[27:24] picks the active
 * entry.  Values lifted from i915's skl_ddi_translations_hdmi
 * (intel_ddi.c) -- level 8 is the standard "1000 mV / 0 dB" entry
 * that works for HDMI 1.4 up to 165 MHz pixclk.
 */
#define	DDI_BUF_TRANS_LO(p, i)	(0x64E00 + (p) * 0x60 + (i) * 8)
#define	DDI_BUF_TRANS_HI(p, i)	(0x64E04 + (p) * 0x60 + (i) * 8)

static const uint32_t skl_hdmi_ddi_trans[10][2] = {
	{ 0x00000018, 0x000000A0 },
	{ 0x00005012, 0x000000B0 },
	{ 0x00007011, 0x000000CB },
	{ 0x00000018, 0x000000E1 },
	{ 0x00000018, 0x000000A1 },
	{ 0x00000018, 0x000000B0 },
	{ 0x00000018, 0x000000CA },
	{ 0x00000018, 0x000000C0 },
	{ 0x80008712, 0x000000C0 },	/* level 8 — default HDMI */
	{ 0x80008712, 0x000000C7 },
};

static int
igen9_sysctl_try_pipe_resume(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	/*
	 * 0) Power wells: request PW2 + DDI_B and poll for STATE up.  Without
	 *    these the pipe/DDI register writes hit a powered-off domain and
	 *    silently get dropped on retain.
	 */
	uint32_t pwr = igen9_r32(sc, HSW_PWR_WELL_CTL2);
	uint32_t want = PWR_WELL_REQ(PW_IDX_PW2) | PWR_WELL_REQ(PW_IDX_DDI_B);
	igen9_w32(sc, HSW_PWR_WELL_CTL2, pwr | want);
	for (int spin = 0; spin < 100; spin++) {
		uint32_t s = igen9_r32(sc, HSW_PWR_WELL_CTL2);
		uint32_t need = PWR_WELL_STATE(PW_IDX_PW2) |
		    PWR_WELL_STATE(PW_IDX_DDI_B);
		if ((s & need) == need) {
			device_printf(sc->dev,
			    "resume: PW2 + DDI_B up after %d ms (CTL2=0x%08x)\n",
			    spin, s);
			break;
		}
		DELAY(1000);
	}

	/*
	 * 0b) Enable DPLL1 (LCPLL2) + poll LOCK.
	 *
	 * Sequence per i915 skl_ddi_pll_enable:
	 *   1. Disable ENABLE (in case of a stuck half-state)
	 *   2. Write DPLL_CTRL1 with HDMI_MODE + OVERRIDE_EN for DPLL1
	 *   3. Write CFGCR1 + CFGCR2 with the tuned DCO / divider values
	 *      (these are the firmware-captured BASELINE values for the
	 *      148.5 MHz HDMI pixel clock target)
	 *   4. Posting read of CFGCR2
	 *   5. Set ENABLE
	 *   6. Poll LOCK (BSpec says <5 ms)
	 */
	uint32_t lcpll2 = igen9_r32(sc, SKL_DPLL1_ENABLE);
	/*
	 * Diagnostic: dump all 4 SKL DPLLs so we can see which one
	 * firmware actually uses for the live HDMI scanout.  DPLL1
	 * (LCPLL2) had ENABLE=1 LOCK=0 — a dead PLL — so it can't be
	 * the source; this lets us find the real one.
	 *   DPLL0 = LCPLL1 @ 0x46010 (usually CDCLK)
	 *   DPLL1 = LCPLL2 @ 0x46014
	 *   DPLL2 = WRPLL1 @ 0x46040
	 *   DPLL3 = WRPLL2 @ 0x46060
	 */
	for (int id = 0; id < 4; id++) {
		uint32_t off = (id == 0) ? 0x46010 :
		    (id == 1) ? 0x46014 :
		    (id == 2) ? 0x46040 : 0x46060;
		uint32_t v = igen9_r32(sc, off);
		device_printf(sc->dev,
		    "resume: DPLL%d @0x%05x = 0x%08x  (ENABLE=%d LOCK=%d)\n",
		    id, off, v,
		    (v & DPLL_ENABLE_BIT) ? 1 : 0,
		    (v & DPLL_LOCK_BIT) ? 1 : 0);
	}
	uint32_t ctrl2 = igen9_r32(sc, DPLL_CTRL2);
	for (int port = 0; port < 5; port++) {
		uint32_t sel = (ctrl2 >> (port * 3 + 1)) & 0x3;
		bool off_bit = (ctrl2 >> (port + 15)) & 0x1;
		device_printf(sc->dev,
		    "resume: DDI_%c clk_sel=DPLL%u  clk_off=%d\n",
		    'A' + port, sel, off_bit ? 1 : 0);
	}
	igen9_w32(sc, SKL_DPLL1_ENABLE, lcpll2 & ~DPLL_ENABLE_BIT);
	(void)igen9_r32(sc, SKL_DPLL1_ENABLE);

	/*
	 * DPLL_CTRL1 (0x6c058) is six bits per DPLL.  For DPLL_n the
	 * slot starts at bit (n*6) with the layout:
	 *   +0  OVERRIDE
	 *   +1  HDMI_MODE
	 *   +2  SSC
	 *   +3..+5  LINK_RATE
	 * So DPLL1 OVERRIDE = bit 6, HDMI_MODE = bit 7.  Previously we
	 * wrote bit 6 + bit 11 calling them "HDMI_MODE + OVERRIDE_EN"
	 * — bit 11 is LINK_RATE[2], not OVERRIDE; and HDMI_MODE was
	 * never set, so the PLL stayed in link-rate mode and CFGCR1/2
	 * were ignored, which is exactly why LOCK never asserted.
	 */
#define	SKL_DPLL_CTRL1_OVERRIDE(id)	(1u << ((id) * 6))
#define	SKL_DPLL_CTRL1_HDMI_MODE(id)	(1u << ((id) * 6 + 1))
	uint32_t ctrl1 = igen9_r32(sc, DPLL_CTRL1);
	igen9_w32(sc, DPLL_CTRL1,
	    ctrl1 | SKL_DPLL_CTRL1_OVERRIDE(1) | SKL_DPLL_CTRL1_HDMI_MODE(1));
	device_printf(sc->dev,
	    "resume: DPLL_CTRL1 0x%08x -> 0x%08x (DPLL1 HDMI_MODE + OVERRIDE)\n",
	    ctrl1, igen9_r32(sc, DPLL_CTRL1));

	/* CFGCR1/2 — firmware-tuned for 148.5 MHz HDMI. */
	igen9_w32(sc, 0x6c040, 0x80400173);
	igen9_w32(sc, 0x6c044, 0x000003a5);
	(void)igen9_r32(sc, 0x6c044);	/* posting */

	igen9_w32(sc, SKL_DPLL1_ENABLE,
	    igen9_r32(sc, SKL_DPLL1_ENABLE) | DPLL_ENABLE_BIT);
	bool locked = false;
	for (int spin = 0; spin < 50; spin++) {
		uint32_t v = igen9_r32(sc, SKL_DPLL1_ENABLE);
		if (v & DPLL_LOCK_BIT) {
			device_printf(sc->dev,
			    "resume: DPLL1 LOCK after %d * 100us (LCPLL2=0x%08x)\n",
			    spin, v);
			locked = true;
			break;
		}
		DELAY(100);
	}
	if (!locked)
		device_printf(sc->dev,
		    "resume: DPLL1 NOT LOCKED  (LCPLL2_CTL=0x%08x)\n",
		    igen9_r32(sc, SKL_DPLL1_ENABLE));

	/* 1) Enable DDI_B port clock: clear CLOCK_OFF bit in DPLL_CTRL2. */
	uint32_t dpll2 = igen9_r32(sc, DPLL_CTRL2);
	igen9_w32(sc, DPLL_CTRL2, dpll2 & ~DPLL_CTRL2_DDI_B_OFF);
	device_printf(sc->dev,
	    "resume: DPLL_CTRL2 0x%08x -> 0x%08x (DDI_B clock on)\n",
	    dpll2, dpll2 & ~DPLL_CTRL2_DDI_B_OFF);

	/* 2) Transcoder A timing — BASELINE values for 1920x1080@60. */
	igen9_w32(sc, TRANS_HTOTAL(0), 0x0897077f);
	igen9_w32(sc, TRANS_HBLANK(0), 0x0897077f);
	igen9_w32(sc, TRANS_HSYNC(0),  0x080307d7);
	igen9_w32(sc, TRANS_VTOTAL(0), 0x04640437);
	igen9_w32(sc, TRANS_VBLANK(0), 0x04640437);
	igen9_w32(sc, TRANS_VSYNC(0),  0x043e0439);
	igen9_w32(sc, PIPE_SRCSZ(0),   ((uint32_t)1919 << 16) | 1079);

	/* 3) Route transcoder A to DDI_B in HDMI mode (BASELINE value). */
	igen9_w32(sc, TRANS_DDI_FUNC_CTL(0), 0x90030000);

	/* 4) Enable Pipe A. */
	igen9_w32(sc, PIPE_CONF(0), PIPE_CONF_ENABLE);
	DELAY(100);
	uint32_t pconf = igen9_r32(sc, PIPE_CONF(0));
	device_printf(sc->dev, "resume: PIPE_CONF=0x%08x\n", pconf);

	/* 5) Primary plane: XRGB8888 linear, 1920x1080, stride 7680, surf=0. */
	igen9_w32(sc, PLANE_STRIDE(0), 7680 / 64);
	igen9_w32(sc, PLANE_SIZE(0),
	    ((uint32_t)1079 << 16) | 1919);
	igen9_w32(sc, PLANE_SURF(0), 0);
	igen9_w32(sc, PLANE_CTL(0),
	    PLANE_CTL_ENABLE | (0x4 << PLANE_CTL_FORMAT_SHIFT));

	/*
	 * 6) Program DDI_B analog buffer: voltage swing table + control
	 *    word per i915 intel_ddi_pre_enable_hdmi.  PORT=1 (DDI_B).
	 *    TRANS_SELECT=8 picks the default HDMI 1000 mV / 0 dB entry,
	 *    PORT_WIDTH_X4 is the mandatory HDMI 4-lane (TMDS clock + 3
	 *    differential pairs), INIT_DISPLAY_DETECTED arms the
	 *    "display present" gate.  ENABLE goes last.
	 */
	for (int i = 0; i < 10; i++) {
		igen9_w32(sc, DDI_BUF_TRANS_LO(1, i),
		    skl_hdmi_ddi_trans[i][0]);
		igen9_w32(sc, DDI_BUF_TRANS_HI(1, i),
		    skl_hdmi_ddi_trans[i][1]);
	}
	igen9_w32(sc, DDI_BUF_CTL(1),
	    DDI_BUF_CTL_ENABLE_BIT |
	    (8u << DDI_BUF_CTL_TRANS_SELECT_SHIFT) |
	    DDI_BUF_CTL_PORT_WIDTH_X4 |
	    DDI_BUF_CTL_INIT_DISPLAY_DETECTED);

	/* Poll IDLE_STATUS to clear; BSpec says < 600 us. */
	for (int spin = 0; spin < 200; spin++) {
		uint32_t bc = igen9_r32(sc, DDI_BUF_CTL(1));
		if ((bc & DDI_BUF_CTL_IDLE_STATUS) == 0) {
			device_printf(sc->dev,
			    "resume: DDI_B left IDLE after %d us\n",
			    spin * 10);
			break;
		}
		DELAY(10);
	}

	DELAY(20000);	/* ~20 ms for HW to stabilise */
	device_printf(sc->dev,
	    "resume: PIPE_CONF=0x%08x  PLANE_CTL=0x%08x  DDI_BUF_B=0x%08x\n",
	    igen9_r32(sc, PIPE_CONF(0)),
	    igen9_r32(sc, PLANE_CTL(0)),
	    igen9_r32(sc, DDI_BUF_CTL(1)));
	uint32_t fc1 = igen9_r32(sc, PIPE_FRMCOUNT(0));
	pause("gen9rsm", hz / 4);
	uint32_t fc2 = igen9_r32(sc, PIPE_FRMCOUNT(0));
	device_printf(sc->dev,
	    "resume: FRMCOUNT delta over 250 ms = %u (expect ~15 if 60 Hz)\n",
	    fc2 - fc1);
	return (0);
}

/* --------------------------- vblank sync ---------------------------------- */

/*
 * Wait until PIPE_FRMCOUNT advances at least once.  At 60 Hz a vblank
 * arrives every ~16.7 ms; the polled cost is bounded by the loop cap.
 * Cap at 50 iterations of 1 ms (= 50 ms) so a stalled pipe never wedges
 * atomic_commit.
 */
void
igen9_wait_vblank(struct igen9_softc *sc, int pipe)
{
	uint32_t start = igen9_r32(sc, PIPE_FRMCOUNT(pipe));
	for (int spin = 0; spin < 50; spin++) {
		if (igen9_r32(sc, PIPE_FRMCOUNT(pipe)) != start)
			return;
		pause("gen9vbl", hz / 1000);
	}
}

/*
 * Register all DPLL/WRPLL/clock/pipe-resume sysctls under the device's
 * .re. subtree.  Called from igen9.c's igen9_re_sysctls_init right after
 * the re_sysctl_tree node is built; the context list + tree pointer are
 * borrowed from the softc.
 */
void
igen9_dpll_register_sysctls(struct igen9_softc *sc)
{
	struct sysctl_ctx_list *ctx = &sc->re_sysctl_ctx;
	struct sysctl_oid_list *children =
	    SYSCTL_CHILDREN(sc->re_sysctl_tree);

	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "clock_state",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_clock_state, "I",
	    "write 1 to dump CDCLK / LCPLL / DPLL / DDI clock-on-off state");

	SYSCTL_ADD_UINT(ctx, children, OID_AUTO,
	    "wrpll_target_khz", CTLFLAG_RW, &sc->wrpll_target_khz, 0,
	    "WRPLL solver target pixel clock in kHz");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "wrpll_calc",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_wrpll_calc, "I",
	    "write 1 to solve WRPLL (DCO_INT/DCO_FRAC/P0/P1/P2) for"
	    " wrpll_target_khz");

	sc->wrpll_dpll_id = 2;
	SYSCTL_ADD_UINT(ctx, children, OID_AUTO,
	    "wrpll_dpll_id", CTLFLAG_RW, &sc->wrpll_dpll_id, 0,
	    "WRPLL to program / dump: 2 = DPLL2, 3 = DPLL3");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "wrpll_dump",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_wrpll_dump, "I",
	    "write 1 to print CFGCR1/CFGCR2 of wrpll_dpll_id (decoded)");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "wrpll_program",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_wrpll_program, "I",
	    "write 1 to solve wrpll_target_khz and program CFGCR1/CFGCR2"
	    " of wrpll_dpll_id (does NOT enable PLL or re-mux DDIs)");

	sc->wrpll_route_port = 2;	/* DDI_C */
	SYSCTL_ADD_UINT(ctx, children, OID_AUTO,
	    "wrpll_route_port", CTLFLAG_RW, &sc->wrpll_route_port, 0,
	    "DDI to re-mux to wrpll_dpll_id: 0=A 1=B 2=C 3=D 4=E");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "wrpll_enable",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_wrpll_enable, "I",
	    "write 1 to enable wrpll_dpll_id (CTRL1 HDMI_MODE + ENABLE +"
	    " poll LOCK)");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "wrpll_disable",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_wrpll_disable, "I",
	    "write 1 to disable wrpll_dpll_id");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "wrpll_route",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_wrpll_route, "I",
	    "write 1 to route DDI[wrpll_route_port] clock to wrpll_dpll_id"
	    " (CTRL2 OFF=0 SEL=id OVERRIDE=1)");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "wrpll_unroute",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_wrpll_unroute, "I",
	    "write 1 to gate DDI[wrpll_route_port] clock off (CTRL2 OFF=1)");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "wrpll_force_clear",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_wrpll_force_clear, "I",
	    "emergency: write 1 to unconditionally clear ENABLE of"
	    " wrpll_dpll_id (no liveness check)");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "pw1_up",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_pw1_up, "I",
	    "write 1 to request PW1 (display PLL power well); required"
	    " for DPLL2/3 enable.  Firmware leaves it down because it"
	    " drives the live link from DPLL0/LCPLL1.");

	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "try_pipe_resume",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_try_pipe_resume, "I",
	    "write 1 to write BASELINE timing/format and enable Pipe A ->"
	    " DDI_B (HDMI 1920x1080@60); upstream clocks must be alive");
}
