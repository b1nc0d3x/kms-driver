/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Rockchip RK3399 display-subsystem driver targeting kms.
 *
 * Phase 9a (this file as of initial check-in): newbus probe + attach,
 * matches "rockchip,display-subsystem", calls kms_dev_register,
 * publishes a virtual KMS topology with one CRTC + one primary plane +
 * one virtual encoder + one virtual connector that exposes the canned
 * 1920x1080@60 mode the stub uses.  No VOP MMIO yet — that lands in
 * Phase 9b/c alongside the actual modeset path.
 *
 * Coexistence: this driver matches the same DT node as the in-tree
 * rk_drm (drm2) driver.  Both can't bind simultaneously; the kernel
 * config that loads rk_kms must `nodevice rk_drm`.  When the
 * full port lands (Phase 9f), rk_drm is removed from RP64KERN_RKDRM
 * and rk_kms takes over card0.
 *
 * Per-SoC source files belong here (sys/dev/kms/drivers/) while
 * the port is in flux.  Once stable they move to sys/arm64/rockchip/
 * to match the rest of the SoC drivers — the location is bookkeeping,
 * not API.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <arm/include/fdt.h>
#include <machine/bus.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_page.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <drm/drm_fourcc.h>
#include <drm/drm_mode.h>

#include <kms/drm_connector.h>
#include <kms/drm_crtc.h>
#include <kms/drm_device.h>
#include <kms/drm_drv.h>
#include <kms/drm_encoder.h>
#include <kms/drm_framebuffer.h>
#include <kms/drm_gem.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_modes.h>
#include <kms/drm_plane.h>

#define	RK_KMS_DESC	"Rockchip RK3399 display (kms)"

/*
 * Same DPRINTF idiom rk_cdn_dp uses: gated on the softc's debug
 * counter so the cold path is one branch.  Set
 * dev.rk_kms.0.debug=1 to enable.
 */
#define	DPRINTF(sc, ...)						\
	do {								\
		if ((sc)->debug > 0)					\
			device_printf((sc)->dev, __VA_ARGS__);		\
	} while (0)

/*
 * RK3399 hardware-block physical addresses.  Lifted verbatim from
 * sys/arm64/rockchip/rk_drm_hw.c (rk_drm.c reference) — the same
 * SoC, the same map.  Phase 9b owns these mappings; Phase 9c writes
 * registers through them.
 *
 * VOP_BIG  : 4K-capable big VOP, primary scanout for the HDMI path.
 * VOP_LIT  : Secondary VOP used for dual-VOP coexistence (HDMI on
 *            VOP_LIT, USB-C DP on VOP_BIG) — mapped but unused until
 *            Phase 9e.
 * GRF      : General Register Files, holds SOC_CON6 (output mux) and
 *            related bridge-routing knobs.
 * CRU      : Clock & Reset Unit, gates DCLK lanes.
 * PMU      : Power Management Unit (display power domain).
 * PMUCRU   : PMU's CRU.
 */
#define	RK3399_VOP_BIG_PA	0xff900000UL
#define	RK3399_VOP_BIG_SIZE	0x10000UL
#define	RK3399_VOP_LIT_PA	0xff8f0000UL
#define	RK3399_VOP_LIT_SIZE	0x10000UL
#define	RK3399_GRF_PA		0xff770000UL
#define	RK3399_GRF_SIZE		0x10000UL
#define	RK3399_CRU_PA		0xff760000UL
#define	RK3399_CRU_SIZE		0x1000UL
#define	RK3399_PMU_PA		0xff310000UL
#define	RK3399_PMU_SIZE		0x1000UL
#define	RK3399_PMUCRU_PA	0xff750000UL
#define	RK3399_PMUCRU_SIZE	0x1000UL
#define	RK3399_HDMI_PA		0xff940000UL
#define	RK3399_HDMI_SIZE	0x20000UL

/*
 * VOP register offsets (subset).  Names match Rockchip's RK3399 TRM
 * §22 (Video Output Processor) and the in-tree rk_drm_hw.c symbols
 * the existing driver uses.  Phase 9c writes only the timing block
 * (HTOTAL / HACT / VTOTAL / VACT) plus the SYS_CTRL enable bit; that's
 * enough to validate the addressing path without touching VPLL,
 * win0 framebuffer setup, or the trigger register.
 */
#define	VOP_REG_CFG_DONE	0x0000
#define	VOP_REG_SYS_CTRL	0x0008
#define	VOP_REG_DSP_CTRL0	0x0010
#define	VOP_REG_DSP_CTRL1	0x0014
#define	VOP_REG_DSP_HTOTAL	0x0188
#define	VOP_REG_DSP_HACT	0x018c
#define	VOP_REG_DSP_VTOTAL	0x0190
#define	VOP_REG_DSP_VACT	0x0194

#define	VOP_SYS_CTRL_STANDBY	(1u << 22)
#define	VOP_SYS_CTRL_MMU_EN	(1u << 20)
#define	VOP_SYS_CTRL_ENABLE	(1u << 11)
#define	VOP_SYS_CTRL_RGB_EN	(1u << 12)
#define	VOP_SYS_CTRL_HDMI_EN	(1u << 13)

/*
 * WIN0 (primary plane) register block.  Programmed at scanout time.
 * Values mirror the in-tree rk_drm reference; bit-pack semantics
 * documented in the RK3399 TRM §22.4.
 */
#define	VOP_REG_WIN0_CTRL0	0x0030
#define	VOP_REG_WIN0_YRGB_BUFSIZE 0x0038
#define	VOP_REG_WIN0_VIR	0x003c
#define	VOP_REG_WIN0_YRGB_MST	0x0040
#define	VOP_REG_WIN0_ACT_INFO	0x0048
#define	VOP_REG_WIN0_DSP_INFO	0x004c
#define	VOP_REG_WIN0_DSP_ST	0x0050
#define	VOP_REG_WIN0_SRC_ALPHA	0x0060
#define	VOP_REG_WIN0_DST_ALPHA	0x0064
#define	VOP_REG_WIN0_CTRL2	0x006c
#define	VOP_REG_POST_DSP_HACT	0x0170
#define	VOP_REG_POST_DSP_VACT	0x0174

#define	VOP_WIN0_CTRL0_UPPER_HDMI 0x00000000u
#define	VOP_WIN0_LB_MODE_RGB	(4u << 5)
#define	VOP_WIN0_DATA_FMT_XRGB8888 0x00000000u
#define	VOP_WIN0_CTRL0_LOWER	(VOP_WIN0_LB_MODE_RGB |			\
				 VOP_WIN0_DATA_FMT_XRGB8888 | 0x01u)
#define	VOP_WIN0_CTRL2_PRIMARY	0x00000021u

/*
 * CRU / VPLL registers — bits the rk_drm reference uses.  Names from
 * the RK3399 TRM §3.2 (CRU).
 */
#define	CRU_VPLL_CON0		0x0080
#define	CRU_VPLL_CON1		0x0084
#define	CRU_VPLL_CON2		0x0088
#define	CRU_VPLL_CON3		0x008c

#define	CRU_PLL_BYPASS		(1u << 1)
#define	CRU_PLL_POWER_DOWN	(1u << 0)
#define	CRU_PLL_DSMPD		(1u << 3)
#define	CRU_PLL_MODE_SLOW	0u
#define	CRU_PLL_MODE_NORMAL	(1u << 8)
#define	CRU_VPLL_CON2_LOCK	(1u << 31)

/*
 * GRF output-mux registers.  SOC_CON20 picks which VOP drives HDMI
 * and EDP; SOC_CON9 picks which VOP drives the Cadence MHDP (USB-C
 * DP).  All three writes use the "hiword update" encoding: high 16
 * bits are the bit mask, low 16 bits are the value to apply.
 *
 *   SOC_CON20[5] EDP_LCDC_SEL : 0 = VOP_BIG → eDP, 1 = VOP_LIT
 *   SOC_CON20[6] HDMI_LCDC_SEL: 0 = VOP_BIG → HDMI, 1 = VOP_LIT
 *   SOC_CON9[12] DP_SEL_VOP_LIT: 0 = VOP_BIG → MHDP, 1 = VOP_LIT
 */
#define	GRF_SOC_CON9		0x6224
#define	GRF_SOC_CON20		0x6250
#define	GRF_GPIO4C_IOMUX	0x0e028
#define	GRF_HDMI_LCDC_SEL	(1u << 6)
#define	GRF_EDP_LCDC_SEL	(1u << 5)
#define	GRF_DP_SEL_VOP_LIT	(1u << 12)

/*
 * Pin-mux value that selects I2C3HDMI mode for GPIO4C pins (DDC SDA /
 * SCL).  Lifted verbatim from the rk_drm reference — matches the
 * value the existing driver writes when it routes VOP→HDMI.
 */
#define	GRF_GPIO4C_I2C3HDMI	0x003f0005u

/*
 * Output selector exposed via sysctl.  Defaults to HDMI; users with
 * USB-C DP topologies switch via dev.rk_kms.0.output=1 before
 * the first SETCRTC.  Phase 9e wires the GRF mux only — actual DP
 * link bring-up lands in a later phase when rk_cdn_dp is ported.
 */
#define	RK_KMS_OUT_HDMI	0
#define	RK_KMS_OUT_DP	1

/*
 * Pixel-clock → VPLL coefficient table.  Lifted verbatim from
 * rk_drm_hw.c (which itself derives the values from Rockchip's BSP
 * + measurement on rp64dbg / armbsd).  Phase 9d ships the most-used
 * subset; rk_drm's full table can be back-ported as more sinks land.
 */
struct rk_kms_pll_rate {
	uint32_t	clock_khz;
	uint16_t	refdiv;
	uint16_t	fbdiv;
	uint16_t	postdiv1;
	uint16_t	postdiv2;
};

static const struct rk_kms_pll_rate rk_kms_pll_rates[] = {
	{  25200, 5,  21, 4, 1 },
	{  27000, 1,  27, 6, 4 },
	{  65000, 1,  65, 6, 4 },
	{  74250, 2,  99, 4, 4 },
	{ 108000, 3,  54, 4, 1 },
	{ 148500, 4,  99, 4, 1 },
};

#define	RK_KMS_PLL_TOLERANCE_KHZ	250

struct rk_kms_softc {
	device_t			 dev;
	struct drm_device		*drm_dev;

	/* KMS objects.  Phase 9a uses static storage to mirror the stub
	 * pattern; Phase 9c will wire these to per-VOP MMIO resources. */
	struct drm_crtc			 crtc;
	struct drm_plane		 primary;
	struct drm_encoder		 encoder;
	struct drm_connector		 connector;

	/*
	 * MMIO mappings.  bsh / size pairs follow the rk_drm reference.
	 * Phase 9b maps them; Phase 9c starts writing.  Unmapped on
	 * detach.  hw_attached gates writes against a half-attached
	 * driver.
	 */
	bus_space_tag_t			 bst;
	bus_space_handle_t		 vop_big_bsh;
	bus_space_handle_t		 vop_lit_bsh;
	bus_space_handle_t		 grf_bsh;
	bus_space_handle_t		 cru_bsh;
	bus_space_handle_t		 pmu_bsh;
	bus_space_handle_t		 pmucru_bsh;
	/*
	 * HDMI controller MMIO is mapped via pmap_mapdev (128 KiB
	 * region — too large for bus_space_map without a memory rman).
	 * va is the kernel virtual base; nothing reads it yet (Phase 9f
	 * wires DW HDMI bring-up).
	 */
	vm_offset_t			 hdmi_va;
	bool				 hw_attached;

	/*
	 * Sysctl gate for actual VOP register programming.  Defaults to
	 * 0 (log-only) so the .ko is safe to kldload on a system whose
	 * display is already live under rk_drm or vt.  Set
	 * dev.rk_kms.0.commit_modeset=1 to enable real writes.
	 */
	int				 commit_modeset;

	/*
	 * Output selector: HDMI vs USB-C DP.  Controls which GRF mux
	 * value set_config writes.  Phase 9e wires the mux only; Phase
	 * 9f programs the DW HDMI controller and Phase 9g hands off to
	 * a ported rk_cdn_dp for the DP side.
	 */
	int				 output;

	/*
	 * Debug verbosity (0 = quiet, 1 = trace MMIO + set_config).
	 * Controlled by dev.rk_kms.0.debug.
	 */
	int				 debug;
};

static const struct ofw_compat_data rk_kms_compat_data[] = {
	{ "rockchip,display-subsystem",	1 },
	{ NULL,				0 }
};

static const struct drm_driver rk_kms_driver = {
	.name		= "rk_kms",
	.desc		= "Rockchip RK3399 display (kms)",
	.date		= "20260608",
	.major		= 0,
	.minor		= 1,
	.patchlevel	= 0,
	.driver_features = 0,
};

static const uint32_t rk_kms_primary_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGB565,
};

/*
 * Hand-rolled CEA-861 1920x1080@60 (148.5 MHz pixel clock) — same
 * timings the stub uses.  Once Phase 9b wires the DP/HDMI side this
 * comes from an EDID parse instead.
 */
static int
rk_kms_get_modes(struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	if (connector->mode_count > 0)
		return (0);
	mode = kms_mode_create();
	mode->clock = 148500;
	mode->hdisplay = 1920;
	mode->hsync_start = 2008;
	mode->hsync_end = 2052;
	mode->htotal = 2200;
	mode->vdisplay = 1080;
	mode->vsync_start = 1084;
	mode->vsync_end = 1089;
	mode->vtotal = 1125;
	mode->flags = KMS_MODE_FLAG_PHSYNC | KMS_MODE_FLAG_PVSYNC;
	mode->type = KMS_MODE_TYPE_DRIVER |
	    KMS_MODE_TYPE_PREFERRED;
	kms_connector_add_mode(connector, mode);
	return (1);
}

/*
 * VOP MMIO accessors.  Phase 9c uses VOP_BIG only; Phase 9e wires
 * VOP_LIT for the HDMI side of the dual-VOP coexistence path.
 */
static inline uint32_t
vop_big_read(struct rk_kms_softc *sc, bus_size_t off)
{
	return (bus_space_read_4(sc->bst, sc->vop_big_bsh, off));
}

static inline void
vop_big_write(struct rk_kms_softc *sc, bus_size_t off, uint32_t val)
{
	bus_space_write_4(sc->bst, sc->vop_big_bsh, off, val);
}

static inline uint32_t
cru_read(struct rk_kms_softc *sc, bus_size_t off)
{
	return (bus_space_read_4(sc->bst, sc->cru_bsh, off));
}

static inline void
cru_write(struct rk_kms_softc *sc, bus_size_t off, uint32_t val)
{
	bus_space_write_4(sc->bst, sc->cru_bsh, off, val);
}

static inline void
grf_write(struct rk_kms_softc *sc, bus_size_t off, uint32_t val)
{
	bus_space_write_4(sc->bst, sc->grf_bsh, off, val);
	bus_space_barrier(sc->bst, sc->grf_bsh, off, 4,
	    BUS_SPACE_BARRIER_WRITE);
}

/*
 * Route VOP_BIG to drive the HDMI controller.  Also flips GPIO4C pin
 * mux to I2C3HDMI so the DDC lines are usable for an EDID read once
 * Phase 9f wires that path.
 *
 * SOC_CON20[6] HDMI_LCDC_SEL = 0 selects VOP_BIG.  Hiword-update:
 * write only the mask bit in the high half with the value cleared in
 * the low half.
 */
static void
rk_kms_route_vop_big_to_hdmi(struct rk_kms_softc *sc)
{
	grf_write(sc, GRF_SOC_CON20, (GRF_HDMI_LCDC_SEL << 16));
	grf_write(sc, GRF_GPIO4C_IOMUX, GRF_GPIO4C_I2C3HDMI);
	DPRINTF(sc, "GRF: routed VOP_BIG -> HDMI (SOC_CON20[6]=0)\n");
}

/*
 * Route VOP_BIG to drive the Cadence MHDP (USB-C DP) controller.
 * The dual mux dance:
 *   SOC_CON20[5] EDP_LCDC_SEL = 0  (VOP_BIG -> eDP, kept clear)
 *   SOC_CON20[6] HDMI_LCDC_SEL = 1 (VOP_LIT -> HDMI, freeing HDMI off
 *                                   VOP_BIG so the DP path owns it)
 *   SOC_CON9[12]  DP_SEL_VOP_LIT = 0 (VOP_BIG -> MHDP)
 *
 * Same sequence the in-tree rk_drm uses when output_select == USBC_DP
 * — necessary even when no DP cable is plugged so HPD events land on
 * the right controller.
 */
static void
rk_kms_route_vop_big_to_dp(struct rk_kms_softc *sc)
{
	grf_write(sc, GRF_SOC_CON20, (GRF_EDP_LCDC_SEL << 16));
	grf_write(sc, GRF_SOC_CON20,
	    (GRF_HDMI_LCDC_SEL << 16) | GRF_HDMI_LCDC_SEL);
	grf_write(sc, GRF_SOC_CON9, (GRF_DP_SEL_VOP_LIT << 16));
	DPRINTF(sc, "GRF: routed VOP_BIG -> DP (SOC_CON20[6]=1, "
	    "CON9[12]=0)\n");
}

/*
 * Pick the PLL coefficient row whose synthesized clock is closest to
 * the requested pixel clock — within a configurable tolerance so
 * userspace timing perturbations don't fail the entire modeset.
 */
static const struct rk_kms_pll_rate *
rk_kms_find_pll_rate(uint32_t clock_khz)
{
	const struct rk_kms_pll_rate *best = NULL;
	uint32_t best_delta = UINT32_MAX;
	uint32_t i, delta;

	for (i = 0; i < nitems(rk_kms_pll_rates); i++) {
		const struct rk_kms_pll_rate *r = &rk_kms_pll_rates[i];

		delta = (r->clock_khz > clock_khz) ?
		    (r->clock_khz - clock_khz) :
		    (clock_khz - r->clock_khz);
		if (delta < best_delta) {
			best_delta = delta;
			best = r;
		}
	}
	if (best == NULL || best_delta > RK_KMS_PLL_TOLERANCE_KHZ)
		return (NULL);
	return (best);
}

/*
 * Drive VPLL to the requested pixel clock.  Sequence matches the
 * rk_drm reference: bypass + slow mode, write fbdiv/refdiv/postdivs,
 * wait for PLL_LOCK in CON2, then flip to normal mode.  Returns 0 on
 * success, EINVAL if the clock is unsupported, ETIMEDOUT on PLL lock
 * failure.
 */
static int
rk_kms_program_vpll(struct rk_kms_softc *sc, uint32_t clock_khz)
{
	const struct rk_kms_pll_rate *r;
	const uint32_t con3_mask = (0x3u << 8) | CRU_PLL_DSMPD |
	    CRU_PLL_BYPASS | CRU_PLL_POWER_DOWN;
	const uint32_t con1_mask = (0x7u << 12) | (0x7u << 8) | 0x3fu;
	uint32_t con3;
	int i;

	r = rk_kms_find_pll_rate(clock_khz);
	if (r == NULL)
		return (EINVAL);

	con3 = CRU_PLL_MODE_SLOW | CRU_PLL_DSMPD | CRU_PLL_POWER_DOWN;
	cru_write(sc, CRU_VPLL_CON3, (con3_mask << 16) | con3);
	DELAY(2);

	cru_write(sc, CRU_VPLL_CON0, (0x0fffu << 16) | r->fbdiv);
	cru_write(sc, CRU_VPLL_CON1, (con1_mask << 16) |
	    (r->postdiv2 << 12) | (r->postdiv1 << 8) | r->refdiv);
	cru_write(sc, CRU_VPLL_CON2, 0u);

	con3 = CRU_PLL_MODE_SLOW | CRU_PLL_DSMPD;
	cru_write(sc, CRU_VPLL_CON3, (con3_mask << 16) | con3);

	for (i = 0; i < 5000; i++) {
		if (cru_read(sc, CRU_VPLL_CON2) & CRU_VPLL_CON2_LOCK)
			break;
		DELAY(10);
	}
	if (i == 5000)
		return (ETIMEDOUT);

	con3 = CRU_PLL_MODE_NORMAL | CRU_PLL_DSMPD;
	cru_write(sc, CRU_VPLL_CON3, (con3_mask << 16) | con3);
	return (0);
}

/*
 * Resolve the physical base of a framebuffer's first GEM plane.  Our
 * drm_gem_object stores pages as a contiguous wired allocation
 * (vm_page_alloc_noobj_contig), so the base address of pages[0] is
 * the linear scanout address VOP_REG_WIN0_YRGB_MST wants.  Returns 0
 * if the framebuffer has no GEM backing.
 */
static vm_paddr_t
rk_kms_fb_paddr(struct drm_framebuffer *fb)
{
	struct drm_gem_object *obj;

	if (fb == NULL)
		return (0);
	obj = fb->gem_objs[0];
	if (obj == NULL || obj->pages == NULL || obj->npages == 0)
		return (0);
	return (VM_PAGE_TO_PHYS(obj->pages[0]));
}

/*
 * Program VOP_BIG WIN0 to scan out `fb` according to `mode`.  Bit
 * patterns mirror rk_drm_vop_program_win0_opaque (HDMI route): no
 * CSC, no forced-opaque alpha, XRGB8888 line-buffer mode.  All 32-bit
 * registers; shadow latch is the trailing CFG_DONE write back in
 * vop_program_timing.
 */
static void
rk_kms_vop_program_win0(struct rk_kms_softc *sc,
    const struct drm_display_mode *mode, struct drm_framebuffer *fb,
    uint32_t hact_start, uint32_t vact_start)
{
	uint32_t stride_bytes, stride_words;
	vm_paddr_t pa;

	pa = rk_kms_fb_paddr(fb);
	if (pa == 0) {
		DPRINTF(sc, "win0: no fb pa, skipping\n");
		return;
	}
	stride_bytes = roundup2(mode->hdisplay, 16) * 4u;	/* XR24 */
	stride_words = stride_bytes / 4u;

	vop_big_write(sc, VOP_REG_WIN0_YRGB_BUFSIZE, 0u);
	vop_big_write(sc, VOP_REG_WIN0_VIR, stride_words);
	vop_big_write(sc, VOP_REG_WIN0_YRGB_MST, (uint32_t)pa);
	vop_big_write(sc, VOP_REG_WIN0_ACT_INFO,
	    (((uint32_t)mode->vdisplay - 1u) << 16) |
	    ((uint32_t)mode->hdisplay - 1u));
	vop_big_write(sc, VOP_REG_WIN0_DSP_INFO,
	    (((uint32_t)mode->vdisplay - 1u) << 16) |
	    ((uint32_t)mode->hdisplay - 1u));
	vop_big_write(sc, VOP_REG_WIN0_DSP_ST,
	    (vact_start << 16) | hact_start);
	vop_big_write(sc, VOP_REG_WIN0_CTRL2, VOP_WIN0_CTRL2_PRIMARY);
	vop_big_write(sc, VOP_REG_POST_DSP_HACT,
	    (hact_start << 16) | (hact_start + mode->hdisplay));
	vop_big_write(sc, VOP_REG_POST_DSP_VACT,
	    (vact_start << 16) | (vact_start + mode->vdisplay));
	vop_big_write(sc, VOP_REG_WIN0_CTRL0,
	    VOP_WIN0_CTRL0_UPPER_HDMI | VOP_WIN0_CTRL0_LOWER);
	DPRINTF(sc, "win0: pa=0x%jx stride=%u (%u words)\n",
	    (uintmax_t)pa, stride_bytes, stride_words);
}

/*
 * Compute VOP DSP timing field values from a drm_display_mode.  Same
 * arithmetic the in-tree rk_drm uses: HACT_ST is the offset from the
 * sync-start to the active area, formed as (htotal - hsync_start +
 * hskew).  The trailing END value is start + display extent.
 */
static uint32_t
rk_kms_hact_start(const struct drm_display_mode *m)
{
	return ((uint32_t)(m->htotal - m->hsync_start) + m->hskew);
}

static uint32_t
rk_kms_vact_start(const struct drm_display_mode *m)
{
	return ((uint32_t)(m->vtotal - m->vsync_start));
}

/*
 * Phase 9c "real" modeset.  Programs the VOP_BIG timing block + flips
 * SYS_CTRL out of standby and into HDMI-driven RGB mode.  Does NOT
 * touch:
 *   - VPLL (DCLK frequency) — Phase 9c assumes whatever U-Boot or the
 *     previous driver left in place is close enough to the requested
 *     pixel clock for the validation pattern to be readable.  Phase
 *     9d wires the CRU CLKSEL_CON47/49 dance from the rk_drm reference.
 *   - WIN0 framebuffer setup — that needs the scanout buffer's
 *     physical base, which arrives in Phase 9d alongside the page-
 *     flip path.
 *   - HDMI PHY / EDID / TMDS — Phase 9e.
 * What we DO program is exactly the surface that lets a logic analyzer
 * see the VOP emitting the right vsync rate, which is the cheapest way
 * to validate the addressing chain on real silicon.
 */
static void
rk_kms_vop_program_timing(struct rk_kms_softc *sc,
    const struct drm_display_mode *mode, struct drm_framebuffer *fb)
{
	uint32_t hact_start = rk_kms_hact_start(mode);
	uint32_t vact_start = rk_kms_vact_start(mode);
	uint32_t hsync_len = mode->hsync_end - mode->hsync_start;
	uint32_t vsync_len = mode->vsync_end - mode->vsync_start;
	uint32_t sys_ctrl;
	int error;

	if (sc->output == RK_KMS_OUT_DP)
		rk_kms_route_vop_big_to_dp(sc);
	else
		rk_kms_route_vop_big_to_hdmi(sc);

	error = rk_kms_program_vpll(sc, mode->clock);
	if (error != 0)
		DPRINTF(sc, "VPLL %u kHz: %d (continuing with current PLL)\n",
		    mode->clock, error);

	sys_ctrl = vop_big_read(sc, VOP_REG_SYS_CTRL);
	sys_ctrl &= ~(VOP_SYS_CTRL_STANDBY | VOP_SYS_CTRL_MMU_EN);
	sys_ctrl |= VOP_SYS_CTRL_ENABLE | VOP_SYS_CTRL_RGB_EN |
	    VOP_SYS_CTRL_HDMI_EN;
	vop_big_write(sc, VOP_REG_SYS_CTRL, sys_ctrl);

	vop_big_write(sc, VOP_REG_DSP_HTOTAL,
	    ((uint32_t)mode->htotal << 16) | hsync_len);
	vop_big_write(sc, VOP_REG_DSP_HACT,
	    (hact_start << 16) | (hact_start + mode->hdisplay));
	vop_big_write(sc, VOP_REG_DSP_VTOTAL,
	    ((uint32_t)mode->vtotal << 16) | vsync_len);
	vop_big_write(sc, VOP_REG_DSP_VACT,
	    (vact_start << 16) | (vact_start + mode->vdisplay));

	rk_kms_vop_program_win0(sc, mode, fb, hact_start, vact_start);

	/* Shadow-register commit.  Same value-of-1 the rk_drm reference
	 * uses to latch the timing block in one shot. */
	vop_big_write(sc, VOP_REG_CFG_DONE, 0x00010001);
}

static int
rk_kms_set_config(struct drm_mode_set *set)
{
	struct rk_kms_softc *sc;

	sc = set->crtc->dev->driver_priv;
	if (set->mode != NULL) {
		DPRINTF(sc, "set_config: %ux%u clock=%u fb=%u commit=%d\n",
		    set->mode->hdisplay, set->mode->vdisplay,
		    set->mode->clock,
		    set->fb != NULL ? set->fb->base.id : 0,
		    sc->commit_modeset);
		if (sc->commit_modeset != 0 && sc->hw_attached)
			rk_kms_vop_program_timing(sc, set->mode, set->fb);
	} else {
		DPRINTF(sc, "set_config: blank (commit=%d)\n",
		    sc->commit_modeset);
		if (sc->commit_modeset != 0 && sc->hw_attached) {
			uint32_t sys_ctrl;

			sys_ctrl = vop_big_read(sc, VOP_REG_SYS_CTRL);
			sys_ctrl |= VOP_SYS_CTRL_STANDBY;
			vop_big_write(sc, VOP_REG_SYS_CTRL, sys_ctrl);
			vop_big_write(sc, VOP_REG_CFG_DONE, 0x00010001);
		}
	}
	return (0);
}

static const struct drm_crtc_funcs rk_kms_crtc_funcs = {
	.set_config = rk_kms_set_config,
};
static const struct drm_plane_funcs rk_kms_plane_funcs = { 0 };
static const struct drm_encoder_funcs rk_kms_encoder_funcs = { 0 };
static const struct drm_connector_funcs rk_kms_connector_funcs = {
	.get_modes = rk_kms_get_modes,
};

static int
rk_kms_topology_init(struct rk_kms_softc *sc)
{
	struct drm_mode_config *mc = &sc->drm_dev->mode_config;
	uint32_t crtc_mask;
	int error;

	mc->max_width = 4096;
	mc->max_height = 4096;

	error = kms_crtc_init(sc->drm_dev, &sc->crtc,
	    &rk_kms_crtc_funcs);
	if (error != 0)
		return (error);
	crtc_mask = 1u << sc->crtc.index;

	error = kms_plane_init(sc->drm_dev, &sc->primary,
	    &rk_kms_plane_funcs, DRM_PLANE_TYPE_PRIMARY, crtc_mask,
	    rk_kms_primary_formats,
	    nitems(rk_kms_primary_formats));
	if (error != 0)
		return (error);
	sc->crtc.primary_plane = &sc->primary;

	error = kms_encoder_init(sc->drm_dev, &sc->encoder,
	    &rk_kms_encoder_funcs, DRM_MODE_ENCODER_TMDS);
	if (error != 0)
		return (error);
	sc->encoder.possible_crtcs = crtc_mask;

	error = kms_connector_init(sc->drm_dev, &sc->connector,
	    &rk_kms_connector_funcs, DRM_MODE_CONNECTOR_HDMIA);
	if (error != 0)
		return (error);
	sc->connector.status = connector_status_connected;
	sc->connector.mm_width = 600;
	sc->connector.mm_height = 340;
	kms_connector_attach_encoder(&sc->connector, &sc->encoder);
	return (0);
}

static void
rk_kms_topology_teardown(struct rk_kms_softc *sc)
{
	kms_connector_cleanup(&sc->connector);
	kms_encoder_cleanup(&sc->encoder);
	kms_plane_cleanup(&sc->primary);
	kms_crtc_cleanup(&sc->crtc);
}

/*
 * MMIO map / unmap helpers.  Both VOPs + GRF + CRU + PMU + PMUCRU
 * are mapped at attach time — total ~84 KiB of kernel virtual.  The
 * memory cost is trivial and pre-mapping keeps the modeset path
 * (Phase 9c) free of failable allocations.  Unmapped symmetrically
 * on detach.
 */
static int
rk_kms_hw_attach(struct rk_kms_softc *sc)
{
	int error = 0;

	sc->bst = fdtbus_bs_tag;

	error = bus_space_map(sc->bst, RK3399_VOP_BIG_PA,
	    RK3399_VOP_BIG_SIZE, 0, &sc->vop_big_bsh);
	if (error != 0)
		return (error);
	error = bus_space_map(sc->bst, RK3399_VOP_LIT_PA,
	    RK3399_VOP_LIT_SIZE, 0, &sc->vop_lit_bsh);
	if (error != 0)
		return (error);
	error = bus_space_map(sc->bst, RK3399_GRF_PA, RK3399_GRF_SIZE, 0,
	    &sc->grf_bsh);
	if (error != 0)
		return (error);
	error = bus_space_map(sc->bst, RK3399_CRU_PA, RK3399_CRU_SIZE, 0,
	    &sc->cru_bsh);
	if (error != 0)
		return (error);
	error = bus_space_map(sc->bst, RK3399_PMU_PA, RK3399_PMU_SIZE, 0,
	    &sc->pmu_bsh);
	if (error != 0)
		return (error);
	error = bus_space_map(sc->bst, RK3399_PMUCRU_PA, RK3399_PMUCRU_SIZE,
	    0, &sc->pmucru_bsh);
	if (error != 0)
		return (error);

	/*
	 * HDMI controller is 128 KiB — past bus_space_map's comfort
	 * zone on this platform, so map through pmap_mapdev like the
	 * rk_drm reference does.  Phase 9f hangs the DW HDMI bring-up
	 * off this VA.
	 */
	sc->hdmi_va = (vm_offset_t)pmap_mapdev(RK3399_HDMI_PA,
	    RK3399_HDMI_SIZE);
	if (sc->hdmi_va == 0)
		return (ENXIO);

	sc->hw_attached = true;
	DPRINTF(sc, "MMIO mapped: VOP_BIG/LIT @ 0xff9{0,8f}0000 "
	    "GRF @ 0xff770000 CRU @ 0xff760000 HDMI @ 0xff940000\n");
	return (0);
}

static void
rk_kms_hw_detach(struct rk_kms_softc *sc)
{
	if (!sc->hw_attached)
		return;
	if (sc->vop_big_bsh != 0)
		bus_space_unmap(sc->bst, sc->vop_big_bsh,
		    RK3399_VOP_BIG_SIZE);
	if (sc->vop_lit_bsh != 0)
		bus_space_unmap(sc->bst, sc->vop_lit_bsh,
		    RK3399_VOP_LIT_SIZE);
	if (sc->grf_bsh != 0)
		bus_space_unmap(sc->bst, sc->grf_bsh, RK3399_GRF_SIZE);
	if (sc->cru_bsh != 0)
		bus_space_unmap(sc->bst, sc->cru_bsh, RK3399_CRU_SIZE);
	if (sc->pmu_bsh != 0)
		bus_space_unmap(sc->bst, sc->pmu_bsh, RK3399_PMU_SIZE);
	if (sc->pmucru_bsh != 0)
		bus_space_unmap(sc->bst, sc->pmucru_bsh, RK3399_PMUCRU_SIZE);
	if (sc->hdmi_va != 0) {
		pmap_unmapdev((void *)sc->hdmi_va, RK3399_HDMI_SIZE);
		sc->hdmi_va = 0;
	}
	sc->hw_attached = false;
}

static int
rk_kms_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, rk_kms_compat_data)->ocd_data
	    == 0)
		return (ENXIO);
	device_set_desc(dev, RK_KMS_DESC);
	return (BUS_PROBE_DEFAULT);
}

static int
rk_kms_attach(device_t dev)
{
	struct rk_kms_softc *sc;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	error = rk_kms_hw_attach(sc);
	if (error != 0) {
		device_printf(dev, "hw_attach: %d\n", error);
		return (error);
	}

	error = kms_dev_register(&rk_kms_driver, sc, &sc->drm_dev);
	if (error != 0) {
		device_printf(dev, "kms_dev_register: %d\n", error);
		rk_kms_hw_detach(sc);
		return (error);
	}

	error = rk_kms_topology_init(sc);
	if (error != 0) {
		device_printf(dev, "topology init: %d\n", error);
		kms_dev_unregister(sc->drm_dev);
		sc->drm_dev = NULL;
		rk_kms_hw_detach(sc);
		return (error);
	}

	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "commit_modeset", CTLFLAG_RW, &sc->commit_modeset, 0,
	    "Actually program VOP registers on set_config (0 = log only)");
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "debug", CTLFLAG_RW, &sc->debug, 0,
	    "Debug verbosity (0 = quiet, 1 = trace attach + set_config)");
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "output", CTLFLAG_RW, &sc->output, 0,
	    "Output selector: 0 = HDMI, 1 = USB-C DP (Phase 9e: GRF mux "
	    "only; DP bring-up TBD)");

	device_printf(dev, "registered (Phase 9c: VOP code wired behind "
	    "commit_modeset sysctl, default off)\n");
	return (0);
}

static int
rk_kms_detach(device_t dev)
{
	struct rk_kms_softc *sc;

	sc = device_get_softc(dev);
	if (sc->drm_dev != NULL) {
		rk_kms_topology_teardown(sc);
		kms_dev_unregister(sc->drm_dev);
		sc->drm_dev = NULL;
	}
	rk_kms_hw_detach(sc);
	return (0);
}

static device_method_t rk_kms_methods[] = {
	DEVMETHOD(device_probe,		rk_kms_probe),
	DEVMETHOD(device_attach,	rk_kms_attach),
	DEVMETHOD(device_detach,	rk_kms_detach),
	DEVMETHOD_END
};

static driver_t rk_kms_driver_kdrv = {
	"rk_kms",
	rk_kms_methods,
	sizeof(struct rk_kms_softc),
};

DRIVER_MODULE(rk_kms, simplebus, rk_kms_driver_kdrv, 0, 0);
MODULE_VERSION(rk_kms, 1);
MODULE_DEPEND(rk_kms, kms, 1, 1, 1);
