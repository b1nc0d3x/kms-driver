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
#include <sys/callout.h>
#include <sys/taskqueue.h>

#include <arm/include/fdt.h>
#include <machine/bus.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_page.h>

#include <sys/fbio.h>
#include <dev/vt/vt.h>
#include <dev/vt/hw/fb/vt_fb.h>
/*
 * vt/vt.h #defines DPRINTF for its own debug noise; we re-define it
 * below with a softc-aware variant.  Undef here so the local
 * definition wins without warnings.
 */
#undef DPRINTF

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <drm/drm_fourcc.h>
#include <drm/drm_mode.h>

#include <kms/drm_atomic.h>
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
#include <kms/drm_vblank.h>

/*
 * DMT-style 1920x1080 timing the XYM W156F1 panel accepts over USB-C
 * DP.  Xorg's default is CEA-VIC 16 with hsync_len=44 (narrow + PHSYNC)
 * which the panel ignores at the DP input.  Wide-NHSYNC variant below
 * (htotal/vtotal unchanged, but hsync placement & width and polarity
 * differ from VIC 16) lights the panel.  Mirrors rk_dp_forced_mode.h
 * in the in-tree rk_drm.
 */
#define	RK_DP_FORCED_CLOCK_KHZ		148500
#define	RK_DP_FORCED_HDISPLAY		1920
#define	RK_DP_FORCED_HSYNC_START	2008
#define	RK_DP_FORCED_HSYNC_END		2152
#define	RK_DP_FORCED_HTOTAL		2200
#define	RK_DP_FORCED_VDISPLAY		1080
#define	RK_DP_FORCED_VSYNC_START	1084
#define	RK_DP_FORCED_VSYNC_END		1089
#define	RK_DP_FORCED_VTOTAL		1125

/*
 * Standard CEA 1920x1080@60 timing for HDMI scanout.  HDMI sinks accept
 * the CEA-VIC 16 narrow-PHSYNC variant directly (unlike the XYM panel
 * over DP which needs DMT-style wide-NHSYNC).
 */
#define	RK_HDMI_FORCED_CLOCK_KHZ	148500
#define	RK_HDMI_FORCED_HDISPLAY		1920
#define	RK_HDMI_FORCED_HSYNC_START	2008
#define	RK_HDMI_FORCED_HSYNC_END	2052
#define	RK_HDMI_FORCED_HTOTAL		2200
#define	RK_HDMI_FORCED_VDISPLAY		1080
#define	RK_HDMI_FORCED_VSYNC_START	1084
#define	RK_HDMI_FORCED_VSYNC_END		1089
#define	RK_HDMI_FORCED_VTOTAL		1125

/*
 * Output configuration target.  The `config` sysctl selects which
 * physical output the driver brings up as a self-contained path —
 * each config has its own complete VOP + framer + PHY bring-up so the
 * two never share half-state.  HDMI and DP touch different IP blocks
 * (DW HDMI controller + Innosilicon PHY vs. Cadence MHDP + Synopsys
 * TYPEC PHY) but compete for VOP_BIG.  Switching `config` re-runs the
 * target's bring-up from scratch; whatever the other side had latched
 * gets overwritten as a side-effect of the comprehensive write
 * sequence rather than via an explicit teardown.
 */
enum rk_kms_config {
	RK_KMS_CONFIG_NONE = 0,
	RK_KMS_CONFIG_HDMI = 1,
	RK_KMS_CONFIG_DP   = 2,
};

/*
 * Cadence MHDP (USB-C DP) entry points.  rk_cdn_dp.c declares these
 * as exported; we MODULE_DEPEND on rk_cdn_dp so the kernel linker
 * resolves them at load time (no runtime symbol lookup — see the
 * mhorne review feedback rule that cost a previous PR a round-trip).
 * auto_bringup_default runs the full 19-stage bring-up the user
 * already validated under rk_drm.  enable_mode programs the framer
 * timing.  set_video_active drives the framer enable bit.
 */
int	rk_cdn_dp_auto_bringup_default(void);
int	rk_cdn_dp_enable_mode(uint32_t clock, uint16_t hdisplay,
	    uint16_t hsync_start, uint16_t hsync_end, uint16_t htotal,
	    uint16_t vdisplay, uint16_t vsync_start, uint16_t vsync_end,
	    uint16_t vtotal, uint32_t flags);
int	rk_cdn_dp_set_video_active_first(bool active);
int	rk_cdn_dp_get_cached_edid(device_t dev, uint8_t *buf, size_t len);

/*
 * fusb302 helpers — declared in <dev/iicbus/usb/fusb302_var.h>, but
 * fusb302 is statically linked into RP64KERN_KMS so the
 * symbols are always resolvable from this module.  Used by the
 * altmode-entry poller (Phase 11) to fire DP bring-up automatically
 * once fusb302 sees the cable enter DP altmode, without waiting for
 * SETCRTC from X.
 */
#include <dev/iicbus/usb/fusb302_var.h>

struct rk_kms_softc;
static void rk_kms_display_domain_sanity(struct rk_kms_softc *sc);
static int rk_kms_dp_modeset(struct rk_kms_softc *sc,
    const struct drm_display_mode *mode);
static void rk_kms_usbc_poll(void *arg);

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
#define	VOP_SYS_CTRL_MIPI_DUAL	(1u << 11)
#define	VOP_SYS_CTRL_MIPI_EN	(1u << 15)
#define	VOP_SYS_CTRL_ENABLE	(1u << 11)
#define	VOP_SYS_CTRL_RGB_EN	(1u << 12)
#define	VOP_SYS_CTRL_HDMI_EN	(1u << 13)

#define	VOP_DSP_OUT_MODE_MASK	0x0000000fu
#define	VOP_DSP_OUT_MODE_AAAA	0x0000000fu	/* RGB+alpha 30bpp */
#define	VOP_DSP_CTRL0_PIN_POL_MASK	(0x7u << 4)
#define	VOP_DSP_CTRL0_DCLK_POL		(1u << 7)
#define	VOP_DSP_CTRL0_P2I_EN		(1u << 5)
#define	VOP_DSP_CTRL0_INTERLACE		(1u << 10)
#define	VOP_DSP_CTRL1_DP_PIN_POL_MASK	(0x7u << 16)
#define	VOP_DSP_CTRL1_DP_DCLK_POL	(1u << 19)
#define	VOP_DSP_CTRL1_HDMI_PIN_POL_MASK	(0x7u << 20)
#define	VOP_DSP_CTRL1_HDMI_DCLK_POL	(1u << 23)

/*
 * Gamma LUT (RK3399 VOP_BIG):
 *   DSP_CTRL1[0]     dsp_lut_en           1 = enable LUT (blocks pixels)
 *   DSP_CTRL1[7]     update_gamma_lut     1 = pending write
 *   GAMMA_LUT_ADDR   0x2000 + n*4         1024-entry table, each 32-bit
 *                                          packed as R:G:B[9:0] each
 *   DBG_POST_REG1[1] lut_buffer_index     toggles on double-buffer swap
 */
#define	VOP_DSP_CTRL1_LUT_EN		(1u << 0)
#define	VOP_DSP_CTRL1_UPDATE_LUT	(1u << 7)
#define	VOP_REG_GAMMA_LUT		0x2000
#define	VOP_REG_DBG_POST_REG1		0x036c
#define	VOP_DBG_POST_LUT_BUF_IDX	(1u << 1)
#define	VOP_GAMMA_LUT_ENTRIES		1024
#define	VOP_REG_POST_SCL_CTRL		0x0180
#define	VOP_REG_DSP_BG			0x0018

/*
 * WIN0 (primary plane) register block.  Programmed at scanout time.
 * Values mirror the in-tree rk_drm reference; bit-pack semantics
 * documented in the RK3399 TRM §22.4.
 */
#define	VOP_REG_WIN0_CTRL0	0x0030
#define	VOP_REG_WIN0_CTRL1	0x0034	/* scaler mode/skip bits */
#define	VOP_REG_WIN0_YRGB_BUFSIZE 0x0038
#define	VOP_REG_WIN0_VIR	0x003c
#define	VOP_REG_WIN0_YRGB_MST	0x0040
#define	VOP_REG_WIN0_ACT_INFO	0x0048
#define	VOP_REG_WIN0_DSP_INFO	0x004c
#define	VOP_REG_WIN0_DSP_ST	0x0050
#define	VOP_REG_WIN0_SCL_FACTOR_YRGB 0x0054	/* [15:0]=hor, [31:16]=ver */
#define	VOP_REG_WIN0_SCL_FACTOR_CBR  0x0058
#define	VOP_REG_WIN0_SCL_OFFSET	0x005c
#define	VOP_REG_WIN0_SRC_ALPHA	0x0060
#define	VOP_REG_WIN0_DST_ALPHA	0x0064
#define	VOP_REG_WIN0_CTRL2	0x006c

/*
 * WIN0_CTRL0 lb_mode field: 3 bits at [7:5].
 *   0=LB_YUV_3840X5, 1=LB_YUV_2560X8, 2=LB_RGB_3840X2,
 *   3=LB_RGB_2560X4, 4=LB_RGB_1920X5, 5=LB_RGB_1280X8.
 * Per Linux drm/rockchip: RGB pick by src width:
 *   >2560 → LB_RGB_3840X2, >1920 → LB_RGB_2560X4, else LB_RGB_1920X5.
 */
#define	VOP_WIN0_CTRL0_LB_MODE_SHIFT	5
#define	VOP_WIN0_LB_RGB_3840X2	2
#define	VOP_WIN0_LB_RGB_2560X4	3
#define	VOP_WIN0_LB_RGB_1920X5	4

/*
 * WIN0_CTRL1 scaler-mode field packing (per rk3288_win_full_scl_ext
 * in Linux drm/rockchip/rockchip_vop_reg.c).  Only YRGB fields matter
 * for our XR24 packed-RGB scanout — CBCR half is zeroed.
 *   [17:16] yrgb_hor_scl_mode  (0=NONE, 1=UP, 2=DOWN)
 *   [19:18] yrgb_ver_scl_mode  (same)
 *   [21:20] yrgb_hsd_mode      (down: 0=BIL, 1=AVG) — unused when UP
 *   [22]    yrgb_vsu_mode      (up:   0=BIL, 1=BIC)
 *   [23]    yrgb_vsd_mode      (down: 0=BIL, 1=AVG) — unused when UP
 */
#define	VOP_WIN0_SCL_MODE_NONE	0
#define	VOP_WIN0_SCL_MODE_UP	1
#define	VOP_WIN0_SCL_MODE_DOWN	2
#define	VOP_WIN0_SCL_VSU_BIC	1

/*
 * WIN2 (area/cursor plane).  RK3288/RK3399 VOP_BIG has WIN2 as a
 * simpler area window with four subareas; we use area 0 only for
 * cursor blit.  Register layout mirrors rk3288_win23_data from Linux
 * drm/rockchip/rockchip_vop_reg.c.
 *   CTRL0[0]    = area0 gate  (1 = ungated)
 *   CTRL0[3:1]  = format0     (0=ARGB8888)
 *   CTRL0[4]    = area0 enable
 *   CTRL0[12]   = rb_swap
 *   DSP_INFO0   = (h-1)<<16 | (w-1)  (12+12 bits masked 0x0fff0fff)
 *   DSP_ST0     = y<<16 | x          (13+13 bits masked 0x1fff1fff)
 *   MST0        = fb pa (32-bit)
 *   VIR0_1[12:0]= area0 stride (words)
 */
#define	VOP_REG_WIN2_CTRL0	0x00b0
#define	VOP_REG_WIN2_CTRL1	0x00b4
#define	VOP_REG_WIN2_VIR0_1	0x00b8
#define	VOP_REG_WIN2_MST0	0x00c0
#define	VOP_REG_WIN2_DSP_INFO0	0x00c4
#define	VOP_REG_WIN2_DSP_ST0	0x00c8
#define	VOP_REG_WIN2_SRC_ALPHA_CTRL 0x00dc
#define	VOP_REG_WIN2_DST_ALPHA_CTRL 0x00ec
#define	VOP_WIN2_CTRL0_ENABLE	(1u << 4)
#define	VOP_WIN2_CTRL0_GATE	(1u << 0)
#define	VOP_WIN2_CTRL0_FMT_ARGB	(0u << 1)
/*
 * Per-pixel alpha blend for the cursor plane.  Values decoded from
 * Linux drm/rockchip vop_plane_atomic_update on RK3288+ WIN2:
 *   src_alpha_ctl:
 *     [0]   SRC_ALPHA_EN    = 1     (use per-pixel src alpha)
 *     [1]   SRC_COLOR_M0    = 0     (ALPHA_SRC_PRE_MUL)
 *     [2]   SRC_ALPHA_M0    = 0     (ALPHA_STRAIGHT)
 *     [4:3] SRC_BLEND_M0    = 1     (ALPHA_PER_PIX)
 *     [5]   SRC_ALPHA_CAL_M0= 0     (ALPHA_NO_SATURATION)
 *     [8:6] SRC_FACTOR_M0   = 1     (ALPHA_ONE — pass src through)
 *   = 0x49
 *   dst_alpha_ctl:
 *     [8:6] DST_FACTOR_M0   = 3     (ALPHA_SRC_INVERSE — 1 - src_alpha
 *                                    blends the underlying primary
 *                                    fb wherever the cursor is
 *                                    transparent)
 *   = 0xc0
 */
#define	VOP_WIN2_SRC_ALPHA_STD	0x00000049u
#define	VOP_WIN2_DST_ALPHA_STD	0x000000c0u
#define	VOP_REG_POST_DSP_HACT	0x0170
#define	VOP_REG_POST_DSP_VACT	0x0174

#define	VOP_WIN0_CTRL0_UPPER_HDMI 0x00000000u
/*
 * DP path needs CSC bits set in WIN0_CTRL0[31:25] + force-opaque alpha
 * in SRC/DST_ALPHA — the Cadence DP framer drops pixels without them.
 * Empirically validated against the rk_drm reference (rk_drm_hw.c
 * RK_DRM_WIN0_ROUTE_DP).
 */
#define	VOP_WIN0_CTRL0_UPPER_DP	0x3a000000u
#define	VOP_WIN0_SRC_ALPHA_OPAQUE 0x00ff0000u
#define	VOP_WIN0_DST_ALPHA_OPAQUE 0x00000000u
#define	VOP_WIN0_LB_MODE_RGB	(4u << 5)
#define	VOP_WIN0_DATA_FMT_XRGB8888 0x00000000u
#define	VOP_WIN0_CTRL0_LOWER	(VOP_WIN0_LB_MODE_RGB |			\
				 VOP_WIN0_DATA_FMT_XRGB8888 | 0x01u)
#define	VOP_WIN0_CTRL2_PRIMARY	0x00000021u

/*
 * CRU / VPLL registers — bits the rk_drm reference uses.  Names from
 * the RK3399 TRM §3.2 (CRU).
 */
/*
 * RK3399 CRU PLL bank — VPLL_CON0..3 sit at 0x00c0..0x00cc.  An earlier
 * draft of this driver had these at 0x0080..0x008c (which is CPLL —
 * one of the CPU/AXI clock sources).  Power-cycling CPLL by way of the
 * VPLL slow-mode dance instantly AXI-hangs the issuing CPU on the next
 * load.  Cross-checked against rk_drm_hw.c (RK_DRM_CRU_VPLL_CON*) and
 * the TRM PLL map.
 */
#define	CRU_VPLL_CON0		0x00c0
#define	CRU_VPLL_CON1		0x00c4
#define	CRU_VPLL_CON2		0x00c8
#define	CRU_VPLL_CON3		0x00cc

#define	CRU_PLL_BYPASS		(1u << 1)
#define	CRU_PLL_POWER_DOWN	(1u << 0)
#define	CRU_PLL_DSMPD		(1u << 3)
#define	CRU_PLL_MODE_SLOW	0u
#define	CRU_PLL_MODE_NORMAL	(1u << 8)
#define	CRU_VPLL_CON2_LOCK	(1u << 31)

/*
 * CRU clock gates for the VOP block.  Both registers use hiword-update
 * encoding (write mask<<16 to ungate the named bits).
 *
 *   CLKGATE_CON10 : VOP0 PLL outputs — DCLK_VOP0_DIV, ACLK_VOP0_PRE,
 *                   HCLK_VOP0_PRE (bits 8/9/12)
 *   CLKGATE_CON28 : VOP_BIG fabric clocks — ACLK_VOPB / HCLK_VOPB /
 *                   DCLK_VOPB and friends (bits 0-7)
 * Values mirror rk_drm_hw.c (RK_DRM_CRU_CLKGATE_{VOP0,VOPB}_MASK).
 */
#define	CRU_CLKGATE_CON10	0x0328
#define	CRU_CLKGATE_CON28	0x0370

/*
 * DCLK source muxes and softreset for VOP_BIG.  After VPLL is at the
 * target rate, CLKSEL_CON47 / CON49 pick "VPLL → DCLK_VOP0" and the
 * divider.  Bit patterns mirror the working HDMI path in rk_drm_hw.c:
 *   CLKSEL_CON47 low  = (3<<8)|(1<<6)|1
 *      mask           = (0x1f<<8)|(0x3<<6)|0x1f
 *   CLKSEL_CON49 low  = 0x0000
 *      mask           = (1<<11)|(0x3<<8)|0xff
 *
 * SOFTRST_CON17 bit 8 = DCLK_VOP0 reset request; pulse to resync the
 * VOP timing engine to the new clock.
 */
#define	CRU_CLKSEL_CON47	0x01bc
#define	CRU_CLKSEL_CON49	0x01c4
#define	CRU_SOFTRST_CON17	0x0444
#define	CRU_DRESETN_VOP0_REQ	(1u << 8)
#define	CRU_CLKGATE_VOP0_MASK	((1u << 12) | (1u << 9) | (1u << 8))
#define	CRU_CLKGATE_VOPB_MASK	\
	((1u << 7) | (1u << 6) | (1u << 5) | (1u << 4) | \
	 (1u << 3) | (1u << 2) | (1u << 1) | (1u << 0))

/*
 * PMU power-domain + bus-idle registers, plus PMUCRU GATEDIS.  The VOP
 * sits inside power-domain PD_VO (bit 20 of PMU_PWRDN_{CON,ST}).  If
 * PWRDN_ST shows PD_VO down, we clear the corresponding CON bit and
 * poll ST.  Once VO is up, drop BUS_IDLE_REQ for VOPB/VOPL and set
 * GATEDIS_VOPB on PMUCRU so the always-on side of the VOP clock can
 * actually run.  Sequence matches rk_drm_display_domain_sanity().
 */
#define	PMU_PWRDN_CON		0x0014
#define	PMU_PWRDN_ST		0x0018
#define	PMU_BUS_IDLE_REQ	0x0060
#define	PMU_PD_VO		(1u << 20)
#define	PMU_IDLE_VOPB		(1u << 7)
#define	PMU_IDLE_VOPL		(1u << 8)

#define	PMUCRU_GATEDIS_CON0	0x0130
#define	PMUCRU_GATEDIS_VOPB	(1u << 19)

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
 * Designware HDMI controller register offsets (subset).  Names match
 * the rk_drm reference; numbers are word-indexed (each "1-byte"
 * register lives at off << 2 in MMIO because the bridge widens the
 * 8-bit DW interface to AXI).
 */
#define	HDMI_IH_I2CMPHY_STAT0	0x0108
#define	HDMI_TX_INVID0		0x0200
#define	HDMI_VP_PR_CD		0x0801
#define	HDMI_VP_STUFF		0x0802
#define	HDMI_VP_REMAP		0x0803
#define	HDMI_VP_CONF		0x0804
#define	HDMI_FC_INVIDCONF	0x1000
#define	HDMI_FC_INHACTV0	0x1001
#define	HDMI_FC_INHACTV1	0x1002
#define	HDMI_FC_INHBLANK0	0x1003
#define	HDMI_FC_INHBLANK1	0x1004
#define	HDMI_FC_INVACTV0	0x1005
#define	HDMI_FC_INVACTV1	0x1006
#define	HDMI_FC_INVBLANK	0x1007
#define	HDMI_FC_HSYNCINDELAY0	0x1008
#define	HDMI_FC_HSYNCINDELAY1	0x1009
#define	HDMI_FC_HSYNCINWIDTH0	0x100a
#define	HDMI_FC_HSYNCINWIDTH1	0x100b
#define	HDMI_FC_VSYNCINDELAY	0x100c
#define	HDMI_FC_VSYNCINWIDTH	0x100d
#define	HDMI_FC_CTRLDUR		0x1011
#define	HDMI_FC_EXCTRLDUR	0x1012
#define	HDMI_FC_EXCTRLSPAC	0x1013
#define	HDMI_FC_CH0PREAM	0x1014
#define	HDMI_FC_CH1PREAM	0x1015
#define	HDMI_FC_CH2PREAM	0x1016
#define	HDMI_FC_AVICONF3	0x1017
#define	HDMI_FC_GCP		0x1018
#define	HDMI_FC_AVICONF0	0x1019
#define	HDMI_FC_AVICONF1	0x101a
#define	HDMI_FC_AVICONF2	0x101b
#define	HDMI_FC_AVIVID		0x101c
#define	HDMI_FC_PACKET_TX_EN	0x10e3
#define	HDMI_PKT_SEND_CTL	0x0640
#define	HDMI_A_HDCPCFG0		0x5000
#define	HDMI_A_HDCPCFG1		0x5001
#define	HDMI_A_VIDPOLCFG	0x5009

#define	HDMI_FC_INVIDCONF_VSYNC_HIGH	0x40
#define	HDMI_FC_INVIDCONF_HSYNC_HIGH	0x20
#define	HDMI_FC_INVIDCONF_DE_HIGH	0x10
#define	HDMI_FC_INVIDCONF_HDMI_MODE	0x08
#define	HDMI_FC_INVIDCONF_R_V_BLANK_HIGH 0x02
#define	HDMI_FC_INVIDCONF_INTERLACED	0x01
#define	HDMI_FC_AVICONF1_PICTURE_ASPECT_4_3  (1u << 4)
#define	HDMI_FC_AVICONF1_PICTURE_ASPECT_16_9 (2u << 4)
#define	HDMI_FC_PACKET_TX_EN_AVI	(1u << 2)
#define	HDMI_FC_PACKET_TX_EN_GCP	(1u << 1)
#define	HDMI_A_HDCPCFG0_HDMIDVI		(1u << 0)
#define	HDMI_A_HDCPCFG1_SWRESETN	(1u << 0)
#define	HDMI_A_HDCPCFG1_ENCRYPTIONDISABLE (1u << 1)
#define	HDMI_A_HDCPCFG1_PH2UPSHFTENC	(1u << 2)
#define	HDMI_A_HDCPCFG1_DEFAULT					\
	(HDMI_A_HDCPCFG1_SWRESETN |				\
	 HDMI_A_HDCPCFG1_ENCRYPTIONDISABLE |			\
	 HDMI_A_HDCPCFG1_PH2UPSHFTENC)
#define	HDMI_A_VIDPOLCFG_DATAENPOL	(1u << 4)
#define	HDMI_PKT_SEND_CTL_AVI_INFO_UP	(1u << 6)
#define	HDMI_PKT_SEND_CTL_AVI_INFO_EN	(1u << 2)
#define	HDMI_FC_VSYNCINWIDTH_DUMMY 0	/* unused, kept for symmetry */
#define	HDMI_MC_CLKDIS		0x4001
#define	HDMI_MC_SWRSTZREQ	0x4002
#define	HDMI_MC_FLOWCTRL	0x4004
#define	HDMI_MC_PHYRSTZ		0x4005
#define	HDMI_MC_LOCKONCLOCK	0x4006
#define	HDMI_MC_HEACPHY_RST	0x4007
#define	HDMI_BASE_SFRDIVLOW	0x4015
#define	HDMI_BASE_SFRDIVHIGH	0x4016
#define	HDMI_PHY_CONF0		0x3000
#define	HDMI_PHY_STAT0		0x3004
#define	HDMI_PHY_JTAG_CFG	0x300a
#define	HDMI_PHY_I2CM_SLAVE	0x3020
#define	HDMI_PHY_I2CM_ADDRESS	0x3021
#define	HDMI_PHY_I2CM_DATAO_1	0x3022
#define	HDMI_PHY_I2CM_DATAO_0	0x3023
#define	HDMI_PHY_I2CM_OPERATION	0x3026
#define	HDMI_PHY_I2CM_CTLINT	0x3028
#define	HDMI_PHY_I2CM_DIV	0x3029
#define	HDMI_PHY_I2CM_SOFTRSTZ	0x302a
#define	HDMI_PHY_I2CM_SS_HCNT1	0x302b
#define	HDMI_PHY_I2CM_SS_HCNT0	0x302c
#define	HDMI_PHY_I2CM_SS_LCNT1	0x302d
#define	HDMI_PHY_I2CM_SS_LCNT0	0x302e
#define	HDMI_PHY_I2CM_FS_HCNT1	0x302f
#define	HDMI_PHY_I2CM_FS_HCNT0	0x3030
#define	HDMI_PHY_I2CM_FS_LCNT1	0x3031
#define	HDMI_PHY_I2CM_FS_LCNT0	0x3032
#define	HDMI_PHY_I2CM_SDA_HOLD	0x3033

#define	HDMI_PHY_I2C_CKCALCTRL	0x05
#define	HDMI_PHY_I2C_CPCE_CTRL	0x06
#define	HDMI_PHY_I2C_GMPCTRL	0x10
#define	HDMI_PHY_I2C_PLLPHBYCTRL 0x13
#define	HDMI_PHY_I2C_CURRCTRL	0x0b
#define	HDMI_PHY_I2C_VLEVCTRL	0x0e
#define	HDMI_PHY_I2C_TXTERM	0x19
#define	HDMI_PHY_I2C_CKSYMTXCTRL 0x09
#define	HDMI_PHY_I2C_MSM_CTRL	0x12

#define	HDMI_PHY_I2C_ADDR	0x69
/*
 * PHY i2c bit-timing + SFR clock divider defaults.  Cross-checked
 * against rk_drm_hw.c; an earlier draft of this driver had a different
 * set of values (likely from a sibling Synopsys DW HDMI variant or an
 * older BSP) which produced clean SCL but NACKs on every transfer.
 * BASE_SFRDIV{LOW,HIGH} together set the SFR clock period; with the
 * wrong divider the slave never sees a valid sample window.
 */
#define	HDMI_PHY_I2CM_DIV_DEFAULT  0x0b
#define	HDMI_PHY_I2CM_SS_HCNT0_DEFAULT 0x6c
#define	HDMI_PHY_I2CM_SS_LCNT0_DEFAULT 0x7f
#define	HDMI_PHY_I2CM_FS_HCNT0_DEFAULT 0x11
#define	HDMI_PHY_I2CM_FS_LCNT0_DEFAULT 0x24
#define	HDMI_PHY_I2CM_SDA_HOLD_DEFAULT 0x09
#define	HDMI_BASE_SFRDIVLOW_DEFAULT   0x93
#define	HDMI_BASE_SFRDIVHIGH_DEFAULT  0x69
#define	HDMI_PHY_JTAG_CFG_I2C	0x80
#define	HDMI_PHY_MSM_CTRL_FB_CLK 0x0006
/*
 * Bit 15 of CKCALCTRL is the override-enable.  rk_drm writes 0x8000
 * here, which tells the PHY to skip its auto clock-calibration path
 * and use the explicit override values fed by CPCE/GMP/CURR.  Without
 * bit 15 the PHY runs auto-cal and intermittently fails to lock at
 * higher pixel rates.
 */
#define	HDMI_PHY_I2C_CKCALCTRL_OVERRIDE 0x8000

#define	HDMI_PHY_CONF0_PDDQ	(1u << 1)
#define	HDMI_PHY_CONF0_PDZ	(1u << 2)
#define	HDMI_PHY_CONF0_ENTMDS	(1u << 3)
#define	HDMI_PHY_CONF0_SVSRET	(1u << 4)
#define	HDMI_PHY_CONF0_TXPWRON	(1u << 6)
#define	HDMI_PHY_CONF0_SELDIPIF	(1u << 5)
#define	HDMI_PHY_CONF0_SELDATAENPOL (1u << 7)

#define	HDMI_MC_SWRST_PIXEL	(1u << 0)
#define	HDMI_MC_SWRST_TMDS	(1u << 1)
#define	HDMI_MC_CLKDIS_CECCLK_DISABLE (1u << 6)

/*
 * Synopsys MPLL + PHY coefficient tables — three pixel clocks for the
 * Phase 9f bring-up bracket.  Full rk_drm tables get back-ported as
 * more sinks/modes need them.
 */
struct rk_kms_mpll_config {
	uint32_t	pixel_clock;
	uint16_t	cpce;
	uint16_t	gmp;
	uint16_t	curr;
};

struct rk_kms_phy_config {
	uint32_t	pixel_clock;
	uint16_t	sym;
	uint16_t	term;
	uint16_t	vlev;
};

static const struct rk_kms_mpll_config rk_kms_mpll_configs[] = {
	{  74250, 0x0072, 0x0001, 0x0028 },
	{ 148500, 0x0051, 0x0003, 0x0000 },
	{      0, 0x0051, 0x0003, 0x0000 }, /* sentinel: clamps to 148500 */
};

static const struct rk_kms_phy_config rk_kms_phy_configs[] = {
	{  74250, 0x8009, 0x0004, 0x0272 },
	{ 148500, 0x802b, 0x0004, 0x028d },
	{      0, 0x0000, 0x0000, 0x0000 },
};

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
	 * Active output configuration.  `config_target` mirrors the user-
	 * facing sysctl (0 = none, 1 = hdmi, 2 = dp).  `config_active`
	 * tracks what's actually been brought up; the two diverge only
	 * during an in-flight switch.  When a config is set, the bring-up
	 * functions overwrite the legacy `output` / `hdmi_enable` /
	 * `dp_enable` / `commit_modeset` knobs as needed — the legacy
	 * knobs remain for fine-grained debug.
	 */
	int				 config_target;
	int				 config_active;

	/*
	 * Output selector: HDMI vs USB-C DP.  Controls which GRF mux
	 * value set_config writes.  Phase 9e wires the mux only; Phase
	 * 9f programs the DW HDMI controller and Phase 9g hands off to
	 * a ported rk_cdn_dp for the DP side.
	 */
	int				 output;

	/*
	 * Gate for DW HDMI bring-up (PHY init + TMDS framer).  Defaults
	 * to 0.  Independent from commit_modeset so the VOP-only path
	 * can be exercised on hardware without touching the PHY (e.g.
	 * to validate vsync/hsync timing on a logic analyzer).
	 */
	int				 hdmi_enable;

	/*
	 * Gate for USB-C DP bring-up.  Defaults 0.  With output=DP and
	 * dp_enable=1, set_config calls rk_cdn_dp_auto_bringup_default
	 * (idempotent — succeeds quickly if the user already brought
	 * the link up via sysctl), then rk_cdn_dp_enable_mode, then
	 * rk_cdn_dp_set_video_active_first(true).
	 */
	int				 dp_enable;

	/*
	 * VBLANK ticker: a self-rearming timeout_task on
	 * taskqueue_thread.  Period derives from the current mode's
	 * vrefresh.  Default off via vblank_enable; set_config flips
	 * it on when commit_modeset is on and a valid mode arrives.
	 *
	 * Phase 9g part 2 simulates vblank IRQs in software — the real
	 * VOP_BIG IRQ wiring would require either a child VOP DT node
	 * binding or hardcoded SPI numbers, both of which add risk
	 * relative to a software timer that's good to a few hundred
	 * microseconds of jitter.  X / wlroots can't tell the
	 * difference.
	 */
	struct timeout_task		 vblank_task;
	int				 vblank_ticks;
	bool				 vblank_running;
	int				 vblank_enable;

	/*
	 * Optional DCLK_VOP0 soft-reset pulse inside the vop_timing stage.
	 * rk_drm only does this on the first scanout (cold attach) — doing
	 * it on every modeset stops an already-scanning VOP for ~41 ms.
	 * Default off; flip to 1 if a stale DCLK domain needs reseating.
	 */
	int				 vop_dclk_reset;

	/*
	 * For USB-C DP, override Xorg's CEA-VIC 16 modeline with the
	 * DMT-style wide-NHSYNC 1920x1080 timing the XYM panel actually
	 * accepts.  See RK_DP_FORCED_* constants above.  Default 1 — Xorg
	 * has no way to know about the panel quirk.
	 */
	int				 dp_force_mode;

	/*
	 * The RK3399 Innosilicon HDMI PHY doesn't seem to assert the
	 * generic Synopsys TX_PHY_LOCK bit even when the link is up and
	 * the TMDS / pixel clock domains are locked (per MC_LOCKONCLOCK).
	 * When this knob is set, phy_init logs the timeout but returns
	 * success, so finish_mode + enable_hdmi_mode get a chance to run.
	 * Default on until the real lock indicator is identified.
	 */
	int				 hdmi_skip_lock_check;

	/*
	 * Cache-flush the FB pages before each VOP DMA scan-out program.
	 * Belt-and-braces on top of the kms-core DIRTYFB handler:
	 * modeset-time flush ensures the initial fb bind sees the
	 * freshest DRAM content even if no DIRTYFB has fired yet.
	 * Default on (write 0 to compare / disable if it wedges).
	 */
	int				 cache_flush_fb;

	/*
	 * Debug verbosity (0 = quiet, 1 = trace MMIO + set_config).
	 * Controlled by dev.rk_kms.0.debug.
	 */
	int				 debug;

	/*
	 * USB-C DP altmode-entry poller (Phase 11).
	 *
	 * Polls fusb302's attach_seq + dp_altmode state every 500 ms.
	 * When a fresh attach completes with DP altmode ready+valid AND
	 * we have not yet brought the link up for this attach_seq, fire
	 * rk_cdn_dp_auto_bringup_default + set_video_active_first(true).
	 *
	 * The alternative — wire a notify callback through rk_typec_phy
	 * — would be cleaner but means new ABI on rk_typec_phy.  Polling
	 * costs one i2c-free softc-field read per tick and has no side
	 * effects; the heavy work only runs on edge.
	 *
	 * Gated by dp_enable=1 and output==RK_KMS_OUT_DP.
	 *
	 * usbc_attach_seq_done is the last attach_seq we successfully
	 * fired bring-up against.  Setting it back to 0 (via the
	 * usbc_bringup_now sysctl with arg 2) forces a re-fire even
	 * without a fresh attach.
	 */
	struct callout			 usbc_poll;
	uint32_t			 usbc_attach_seq_done;

	/*
	 * Last observed fusb302 CC-line attach state.  usbc_poll compares
	 * this against a fresh fusb302_get_typec_status() every 500 ms;
	 * any transition triggers kms_connector_hotplug so open drm fds
	 * see DRM_EVENT_CONNECTOR_HOTPLUG and devd emits a
	 * kms/cardN/hotplug broadcast.
	 */
	bool				 usbc_last_attached;

	/*
	 * Enable hotplug event dispatch (default OFF).  See usbc_poll
	 * comment above the gate for rationale.
	 */
	int				 hotplug_enable;

	/*
	 * EDID probe latch — set once the first successful
	 * rk_cdn_dp_get_cached_edid parse pushes the blob into the
	 * DRM connector property.  get_modes re-checks this every call
	 * so we retry cheaply until cdn_dp reaches stage 19.
	 */
	bool				 edid_probed;

	/*
	 * Hardware cursor plane state (VOP WIN2 area 0).  cursor_bo
	 * pins the current cursor GEM object so its pages don't get
	 * freed while VOP is scanning them; NULL when the cursor is
	 * disabled.  cursor_hot_x / _y let cursor_move translate the
	 * userspace hotspot-relative coordinate into a top-left screen
	 * position for WIN2_DSP_ST0.
	 */
	struct drm_gem_object		*cursor_bo;
	uint32_t			 cursor_width;
	uint32_t			 cursor_height;
	int32_t				 cursor_hot_x;
	int32_t				 cursor_hot_y;
	int32_t				 cursor_x;
	int32_t				 cursor_y;

	/*
	 * HW cursor default OFF — enabling it caused the DP display to
	 * drop signal on the XYM W156F1 panel (WIN2 alpha_ctl writes
	 * appear to disturb WIN0's blend against DP framer somehow).
	 * Set dev.rk_kms.0.hw_cursor=1 to try after the primary display
	 * is up and stable.
	 */
	int				 hw_cursor_enable;
	bool				 usbc_poll_armed;

	/*
	 * Phase 12 — vt console framebuffer bridge.
	 *
	 * Allocate a contiguous physical framebuffer at attach time,
	 * publish it via the fb_getinfo DEVMETHOD, and attach fbd_driver
	 * as our child.  fbd publishes /dev/fb0 + registers with vt_fb,
	 * which gives us /dev/ttyv* — which is what Xorg's xf86OpenConsole
	 * needs to grab a VT.
	 *
	 * VOP scanout from this buffer is wired by the SETCRTC path or
	 * by the bootfb sysctl in Phase 12 part 2.  For Phase 12 part 1
	 * we just want the device nodes to exist so Xorg can start.
	 *
	 * Hardcoded 1920x1080 XRGB8888 boot fb; matches the canned mode
	 * the stub connector advertises.
	 */
	bus_dma_tag_t			 fb_dma_tag;
	bus_dmamap_t			 fb_dma_map;
	vm_offset_t			 fb_va;
	vm_paddr_t			 fb_pa;
	size_t				 fb_size;
	struct fb_info			 fb_info;
	bool				 fb_published;
	bool				 vt_fb_attached;
};

#define	RK_KMS_FB_WIDTH	1920
#define	RK_KMS_FB_HEIGHT	1080
#define	RK_KMS_FB_BPP	32	/* XRGB8888 */
#define	RK_KMS_FB_STRIDE	(RK_KMS_FB_WIDTH * 4)
#define	RK_KMS_FB_SIZE	(RK_KMS_FB_STRIDE *		\
				 RK_KMS_FB_HEIGHT)

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
/*
 * Table of DMT / CTA-861 modes to advertise on the connector.  1920x1080
 * stays index 0 (PREFERRED) so Xorg defaults to it, but exposing the
 * standard fallbacks lets xrandr / display settings offer alternatives.
 * dp_force_mode only rewrites the timing when Xorg picks 1920x1080; for
 * anything smaller we send the actual selected mode's timing to VOP.
 */
struct rk_kms_advertised_mode {
	uint32_t	clock;		/* kHz */
	uint16_t	hdisplay, hsync_start, hsync_end, htotal;
	uint16_t	vdisplay, vsync_start, vsync_end, vtotal;
	uint32_t	flags;
	bool		preferred;
};

static const struct rk_kms_advertised_mode rk_kms_mode_table[] = {
	/*
	 * Native mode MUST stay at index 0 — set_config + page_flip
	 * substitute rk_kms_mode_table[0] as the outer DSP timing
	 * whenever Xorg picks anything else (so the DP link keeps its
	 * trained pixel clock).  The WIN0 hardware scaler in
	 * vop_program_win0 upscales the fb from the picked mode's
	 * dimensions to native.
	 */
	{ 148500, 1920, 2008, 2052, 2200, 1080, 1084, 1089, 1125,
	  KMS_MODE_FLAG_PHSYNC | KMS_MODE_FLAG_PVSYNC, true },
	/* 1600x900@60 DMT-CVT-RB */
	{ 108000, 1600, 1624, 1704, 1800,  900,  901,  904,  1000,
	  KMS_MODE_FLAG_PHSYNC | KMS_MODE_FLAG_NVSYNC, false },
	/* 1366x768@60 DMT-CVT-RB */
	{  85500, 1366, 1436, 1579, 1792,  768,  771,  774,  798,
	  KMS_MODE_FLAG_PHSYNC | KMS_MODE_FLAG_PVSYNC, false },
	/* 1280x720@60 CEA-VIC 4 */
	{  74250, 1280, 1390, 1430, 1650,  720,  725,  730,  750,
	  KMS_MODE_FLAG_PHSYNC | KMS_MODE_FLAG_PVSYNC, false },
	/* 1024x768@60 DMT */
	{  65000, 1024, 1048, 1184, 1344,  768,  771,  777,  806,
	  KMS_MODE_FLAG_NHSYNC | KMS_MODE_FLAG_NVSYNC, false },
	/* 800x600@60 DMT */
	{  40000,  800,  840,  968, 1056,  600,  601,  605,  628,
	  KMS_MODE_FLAG_PHSYNC | KMS_MODE_FLAG_PVSYNC, false },
};

/*
 * EDID probe.  Pulls the 128-byte block cdn_dp cached during link
 * training via rk_cdn_dp_get_cached_edid, validates the header, and
 * publishes:
 *   - physical dimensions (mm_width / mm_height) from EDID bytes
 *     0x15 (h_size cm) and 0x16 (v_size cm) so Xorg reports correct DPI.
 *   - the raw blob to the DRM connector's EDID property via
 *     kms_connector_update_edid, so xrandr / xdpyinfo / desktop
 *     display settings can identify the panel and its supported modes.
 *
 * We do NOT synthesize new mode table entries from EDID DTDs — the
 * XYM W156F1 panel we validate against advertises a preferred timing
 * that trips dp_force_mode's compat path.  Advertising the panel's
 * exact DTD would just make Xorg pick a mode that we then override
 * inside vop_program_timing.  Keeping the hand-tuned mode table +
 * publishing EDID metadata is the least-surprising combination.
 *
 * Returns 0 on success (probe complete or already probed), non-zero
 * on transient failure (cdn_dp not up yet — safe to retry later).
 */
static int
rk_kms_probe_edid(struct rk_kms_softc *sc)
{
	device_t cdev;
	devclass_t cdc;
	uint8_t edid[512];		/* base block + up to 3 extensions */
	size_t used;
	int error;

	if (sc->edid_probed)
		return (0);
	cdc = devclass_find("rk_cdn_dp");
	if (cdc == NULL) {
		DPRINTF(sc, "EDID: rk_cdn_dp devclass not found\n");
		return (ENXIO);
	}
	cdev = devclass_get_device(cdc, 0);
	if (cdev == NULL) {
		DPRINTF(sc, "EDID: rk_cdn_dp device_t not found\n");
		return (ENXIO);
	}
	error = rk_cdn_dp_get_cached_edid(cdev, edid, sizeof(edid));
	if (error != 0) {
		DPRINTF(sc, "EDID: get_cached_edid rc=%d\n", error);
		return (error);
	}
	/* EDID header magic: 00 FF FF FF FF FF FF 00. */
	if (edid[0] != 0x00 || edid[1] != 0xff || edid[2] != 0xff ||
	    edid[3] != 0xff || edid[4] != 0xff || edid[5] != 0xff ||
	    edid[6] != 0xff || edid[7] != 0x00) {
		DPRINTF(sc, "EDID: bad header magic (%02x %02x ...)\n",
		    edid[0], edid[1]);
		return (EINVAL);
	}
	/* Physical dimensions (cm). */
	if (edid[0x15] != 0 && edid[0x16] != 0) {
		sc->connector.mm_width = (uint32_t)edid[0x15] * 10u;
		sc->connector.mm_height = (uint32_t)edid[0x16] * 10u;
	}
	/*
	 * Publish the blob so DRM_PROP_EDID readers see it.  Silently
	 * ignore failure — the property may not exist on stub-built
	 * connectors.  Not a hard error either way.  Publish the actual
	 * used length (base 128 + 128 per extension block, capped at
	 * sizeof(edid)) so xrandr sees the full descriptor set.
	 */
	used = 128u + 128u * (size_t)edid[126];
	if (used > sizeof(edid))
		used = sizeof(edid);
	(void)kms_connector_update_edid(&sc->connector, edid, used);
	sc->edid_probed = true;
	DPRINTF(sc, "EDID: probed OK; mfg=%02x%02x prod=0x%02x%02x mm=%ux%u\n",
	    edid[8], edid[9], edid[0x0b], edid[0x0a],
	    sc->connector.mm_width, sc->connector.mm_height);
	return (0);
}

static int
rk_kms_get_modes(struct drm_connector *connector)
{
	struct rk_kms_softc *sc = __containerof(connector,
	    struct rk_kms_softc, connector);
	struct drm_display_mode *mode;
	unsigned int i;
	int added = 0;

	/* Best-effort EDID probe on every re-probe pass (cheap when
	 * already cached — early-out on edid_probed flag). */
	(void)rk_kms_probe_edid(sc);
	if (connector->mode_count > 0)
		return (0);
	for (i = 0; i < nitems(rk_kms_mode_table); i++) {
		const struct rk_kms_advertised_mode *m = &rk_kms_mode_table[i];

		mode = kms_mode_create();
		mode->clock = m->clock;
		mode->hdisplay = m->hdisplay;
		mode->hsync_start = m->hsync_start;
		mode->hsync_end = m->hsync_end;
		mode->htotal = m->htotal;
		mode->vdisplay = m->vdisplay;
		mode->vsync_start = m->vsync_start;
		mode->vsync_end = m->vsync_end;
		mode->vtotal = m->vtotal;
		mode->flags = m->flags;
		mode->type = KMS_MODE_TYPE_DRIVER |
		    (m->preferred ? KMS_MODE_TYPE_PREFERRED : 0);
		kms_connector_add_mode(connector, mode);
		added++;
	}
	return (added);
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

static inline uint32_t
pmu_read(struct rk_kms_softc *sc, bus_size_t off)
{
	return (bus_space_read_4(sc->bst, sc->pmu_bsh, off));
}

static inline void
pmu_write(struct rk_kms_softc *sc, bus_size_t off, uint32_t val)
{
	bus_space_write_4(sc->bst, sc->pmu_bsh, off, val);
}

static inline uint32_t
pmucru_read(struct rk_kms_softc *sc, bus_size_t off)
{
	return (bus_space_read_4(sc->bst, sc->pmucru_bsh, off));
}

static inline void
pmucru_write(struct rk_kms_softc *sc, bus_size_t off, uint32_t val)
{
	bus_space_write_4(sc->bst, sc->pmucru_bsh, off, val);
}

/*
 * HDMI controller register accessors.  The Designware IP exposes
 * 8-bit registers, but the RK3399 bridge widens them to 32-bit AXI
 * with the byte position kept in the low 8 bits and the register
 * offset multiplied by 4.  Read/write 1 byte at a time exactly the
 * way the rk_drm reference does — same dsb/isb fence on writes.
 */
static inline uint8_t
hdmi_read1(struct rk_kms_softc *sc, size_t off)
{
	volatile uint32_t *reg;

	reg = (volatile uint32_t *)(sc->hdmi_va + (off << 2));
	return ((uint8_t)(*reg & 0xff));
}

static inline void
hdmi_write1(struct rk_kms_softc *sc, size_t off, uint8_t val)
{
	volatile uint32_t *reg;

	reg = (volatile uint32_t *)(sc->hdmi_va + (off << 2));
	*reg = val;
	__asm volatile("dsb sy" ::: "memory");
	__asm volatile("isb" ::: "memory");
}

/*
 * Pick the MPLL / PHY rows for a given pixel clock.  Both tables
 * terminate with a sentinel whose pixel_clock = 0; if the requested
 * clock exceeds every defined entry, the sentinel's coefficients
 * (the 148.5 MHz set) are returned so callers always get usable
 * values for the bracket Phase 9f covers.  Below 74.25 MHz the
 * 74.25 row applies — coarser than rk_drm's full table but enough
 * for the standard 1080p/720p bring-up.
 */
static const struct rk_kms_mpll_config *
rk_kms_find_mpll(uint32_t clock_khz)
{
	const struct rk_kms_mpll_config *r;
	size_t i;

	for (i = 0; i < nitems(rk_kms_mpll_configs); i++) {
		r = &rk_kms_mpll_configs[i];
		if (r->pixel_clock == 0 || r->pixel_clock >= clock_khz)
			return (r);
	}
	return (&rk_kms_mpll_configs[nitems(rk_kms_mpll_configs) - 1]);
}

static const struct rk_kms_phy_config *
rk_kms_find_phy(uint32_t clock_khz)
{
	const struct rk_kms_phy_config *r;
	size_t i;

	for (i = 0; i < nitems(rk_kms_phy_configs); i++) {
		r = &rk_kms_phy_configs[i];
		if (r->pixel_clock == 0 || r->pixel_clock >= clock_khz)
			return (r);
	}
	return (&rk_kms_phy_configs[nitems(rk_kms_phy_configs) - 1]);
}

/*
 * PHY I2C engine reset.  The DW HDMI's PHY config is reached through
 * an internal I2C master that talks to the Synopsys PHY.  The bus
 * sometimes wedges between modesets, so the reset returns ETIMEDOUT
 * if the SOFTRSTZ doesn't latch.
 */
static int
rk_kms_hdmi_phy_i2c_reset(struct rk_kms_softc *sc)
{
	int t;

	hdmi_write1(sc, HDMI_PHY_I2CM_SOFTRSTZ, 0x00);
	for (t = 100; t > 0; t--) {
		if ((hdmi_read1(sc, HDMI_PHY_I2CM_SOFTRSTZ) & 0x01) != 0)
			return (0);
		DELAY(10);
	}
	DPRINTF(sc, "phy i2c reset timed out (SOFTRSTZ=0x%02x)\n",
	    hdmi_read1(sc, HDMI_PHY_I2CM_SOFTRSTZ));
	return (ETIMEDOUT);
}

/*
 * Write one 16-bit value into a PHY register through the I2C master.
 * Big-endian on the wire.  Polls IH_I2CMPHY_STAT0 + CTLINT for the
 * done flag or an error.  200 ms max timeout.
 */
static int
rk_kms_hdmi_phy_i2c_write(struct rk_kms_softc *sc, uint8_t reg,
    uint16_t val)
{
	uint8_t err, sticky, ctlint_init, stat0_init;
	int t;

	if (rk_kms_hdmi_phy_i2c_reset(sc) != 0) {
		DPRINTF(sc, "phy i2c reset failed (reg=0x%02x)\n", reg);
		return (ETIMEDOUT);
	}
	ctlint_init = hdmi_read1(sc, HDMI_PHY_I2CM_CTLINT);
	stat0_init = hdmi_read1(sc, HDMI_IH_I2CMPHY_STAT0);

	hdmi_write1(sc, HDMI_IH_I2CMPHY_STAT0, 0x03);
	hdmi_write1(sc, HDMI_PHY_I2CM_SLAVE, HDMI_PHY_I2C_ADDR);
	hdmi_write1(sc, HDMI_PHY_I2CM_ADDRESS, reg);
	hdmi_write1(sc, HDMI_PHY_I2CM_DATAO_1, (val >> 8) & 0xff);
	hdmi_write1(sc, HDMI_PHY_I2CM_DATAO_0, val & 0xff);
	hdmi_write1(sc, HDMI_PHY_I2CM_OPERATION, 0x10);

	/*
	 * IH_I2CMPHY_STAT0 sticky bits:
	 *   bit 0 = error (NACK / arb-loss / device fault)
	 *   bit 1 = done  (transfer completed with ACK)
	 * An earlier draft of this driver treated bit 0 as "done" — which
	 * meant every successful transfer (bit 1 set) timed out and every
	 * NACK was misread as success.  Cross-check against rk_drm_hw.c
	 * rk_drm_hdmi_phy_i2c_write.
	 */
	for (t = 200; t > 0; t--) {
		DELAY(1000);
		err = hdmi_read1(sc, HDMI_PHY_I2CM_CTLINT);
		sticky = hdmi_read1(sc, HDMI_IH_I2CMPHY_STAT0);
		if ((sticky & 0x01) != 0 || (err & 0x10) != 0 ||
		    (err & 0x01) != 0) {
			DPRINTF(sc, "phy i2c write reg=0x%02x val=0x%04x "
			    "err=0x%02x sticky=0x%02x\n", reg, val, err,
			    sticky);
			hdmi_write1(sc, HDMI_IH_I2CMPHY_STAT0, sticky);
			(void)rk_kms_hdmi_phy_i2c_reset(sc);
			return (EIO);
		}
		if ((sticky & 0x02) != 0) {
			hdmi_write1(sc, HDMI_IH_I2CMPHY_STAT0, 0x02);
			return (0);
		}
	}
	DPRINTF(sc, "phy i2c write reg=0x%02x val=0x%04x TIMEOUT "
	    "(ctlint init/final=0x%02x/0x%02x stat0 init/final=0x%02x/0x%02x)\n",
	    reg, val, ctlint_init, err, stat0_init, sticky);
	(void)rk_kms_hdmi_phy_i2c_reset(sc);
	return (ETIMEDOUT);
}

/*
 * Toggle the DW HDMI main reset bits.  Used to force the TMDS + pixel
 * domains to re-init after a PHY swap.
 */
static void
rk_kms_hdmi_toggle_main_reset(struct rk_kms_softc *sc, uint8_t bits)
{
	uint8_t reg;

	/*
	 * SWRSTZREQ bits are active-low: 0 = reset asserted, 1 = released.
	 * Preserve the state of unrelated reset domains across the toggle —
	 * an earlier draft of this function blasted 0xff after the assert
	 * which inadvertently deasserts every other reset on the controller
	 * (HEACPHY, CEC, audio…) and confuses the PLL-lock check.
	 */
	reg = hdmi_read1(sc, HDMI_MC_SWRSTZREQ);
	hdmi_write1(sc, HDMI_MC_SWRSTZREQ, reg & ~bits);
	DELAY(100);
	hdmi_write1(sc, HDMI_MC_SWRSTZREQ, reg | bits);
}

/*
 * DW HDMI PHY init.  Lifted from rk_drm_hdmi_phy_init: gates four CRU
 * lanes, then runs the PHY soft-reset + MPLL coefficient write + PHY
 * power-up sequence twice (Rockchip's BSP says the first pass is
 * unreliable when the PHY was previously off, the second always
 * locks).  Returns ETIMEDOUT if PHY_STAT0[0] (TX_READY) never goes
 * high within 100 ms after the TMDS reset.
 *
 * Gated behind dev.rk_kms.0.hdmi_enable so the .ko is safe to
 * load on a live-display kernel without smashing the panel.  Phase
 * 9f part 2 lands the TMDS framer + AVI infoframe path.
 */
static int
rk_kms_hdmi_phy_init(struct rk_kms_softc *sc,
    const struct drm_display_mode *mode)
{
	const struct rk_kms_mpll_config *mpll;
	const struct rk_kms_phy_config *phy;
	uint16_t vsync_len;
	uint8_t cfg0;
	int iter, timeout;

	mpll = rk_kms_find_mpll(mode->clock);
	phy = rk_kms_find_phy(mode->clock);

	/*
	 * CRU CLKSEL_CON gates feeding the HDMI lanes.  The values below
	 * mirror rk_drm: each pair clears the disable bits in the high
	 * 16 (mask) half and sets the enable value in the low half.
	 */
	cru_write(sc, 0x0240, (1u << 25) | (1u << 26));
	cru_write(sc, 0x0244, (1u << 18));
	cru_write(sc, 0x0250, (1u << 28));
	cru_write(sc, 0x0254, (1u << 24));
	DELAY(10000);

	vsync_len = (uint16_t)(mode->vsync_end - mode->vsync_start);

	for (iter = 0; iter < 2; iter++) {
		hdmi_write1(sc, HDMI_MC_FLOWCTRL, 0x00);
		hdmi_write1(sc, HDMI_MC_PHYRSTZ, 0x01);
		hdmi_write1(sc, HDMI_VP_PR_CD, 0x40);
		DELAY(5000);
		hdmi_write1(sc, HDMI_MC_PHYRSTZ, 0x00);
		DELAY(5000);
		hdmi_write1(sc, HDMI_MC_HEACPHY_RST, 0x01);

		hdmi_write1(sc, HDMI_BASE_SFRDIVLOW,
		    HDMI_BASE_SFRDIVLOW_DEFAULT);
		hdmi_write1(sc, HDMI_BASE_SFRDIVHIGH,
		    HDMI_BASE_SFRDIVHIGH_DEFAULT);
		hdmi_write1(sc, HDMI_PHY_JTAG_CFG, HDMI_PHY_JTAG_CFG_I2C);

		if (rk_kms_hdmi_phy_i2c_reset(sc) != 0) {
			DPRINTF(sc, "phy i2c reset timed out\n");
			return (ETIMEDOUT);
		}

		hdmi_write1(sc, HDMI_PHY_I2CM_SLAVE, HDMI_PHY_I2C_ADDR);
		hdmi_write1(sc, HDMI_PHY_I2CM_DIV, HDMI_PHY_I2CM_DIV_DEFAULT);
		hdmi_write1(sc, HDMI_PHY_I2CM_SS_HCNT1, 0x00);
		hdmi_write1(sc, HDMI_PHY_I2CM_SS_HCNT0,
		    HDMI_PHY_I2CM_SS_HCNT0_DEFAULT);
		hdmi_write1(sc, HDMI_PHY_I2CM_SS_LCNT1, 0x00);
		hdmi_write1(sc, HDMI_PHY_I2CM_SS_LCNT0,
		    HDMI_PHY_I2CM_SS_LCNT0_DEFAULT);
		hdmi_write1(sc, HDMI_PHY_I2CM_FS_HCNT1, 0x00);
		hdmi_write1(sc, HDMI_PHY_I2CM_FS_HCNT0,
		    HDMI_PHY_I2CM_FS_HCNT0_DEFAULT);
		hdmi_write1(sc, HDMI_PHY_I2CM_FS_LCNT1, 0x00);
		hdmi_write1(sc, HDMI_PHY_I2CM_FS_LCNT0,
		    HDMI_PHY_I2CM_FS_LCNT0_DEFAULT);
		hdmi_write1(sc, HDMI_PHY_I2CM_SDA_HOLD,
		    HDMI_PHY_I2CM_SDA_HOLD_DEFAULT);

		cfg0 = hdmi_read1(sc, HDMI_PHY_CONF0);
		cfg0 |= HDMI_PHY_CONF0_SELDATAENPOL | HDMI_PHY_CONF0_PDDQ;
		cfg0 &= ~(HDMI_PHY_CONF0_SELDIPIF | HDMI_PHY_CONF0_ENTMDS |
		    HDMI_PHY_CONF0_PDZ | HDMI_PHY_CONF0_TXPWRON |
		    HDMI_PHY_CONF0_SVSRET);
		hdmi_write1(sc, HDMI_PHY_CONF0, cfg0);
		DELAY(1000);

		if (rk_kms_hdmi_phy_i2c_write(sc,
		    HDMI_PHY_I2C_CPCE_CTRL, mpll->cpce) != 0 ||
		    rk_kms_hdmi_phy_i2c_write(sc,
		    HDMI_PHY_I2C_GMPCTRL, mpll->gmp) != 0 ||
		    rk_kms_hdmi_phy_i2c_write(sc,
		    HDMI_PHY_I2C_CURRCTRL, mpll->curr) != 0 ||
		    rk_kms_hdmi_phy_i2c_write(sc,
		    HDMI_PHY_I2C_PLLPHBYCTRL, 0x0000) != 0 ||
		    rk_kms_hdmi_phy_i2c_write(sc,
		    HDMI_PHY_I2C_MSM_CTRL, HDMI_PHY_MSM_CTRL_FB_CLK) != 0 ||
		    rk_kms_hdmi_phy_i2c_write(sc,
		    HDMI_PHY_I2C_TXTERM, phy->term) != 0 ||
		    rk_kms_hdmi_phy_i2c_write(sc,
		    HDMI_PHY_I2C_CKSYMTXCTRL, phy->sym) != 0 ||
		    rk_kms_hdmi_phy_i2c_write(sc,
		    HDMI_PHY_I2C_VLEVCTRL, phy->vlev) != 0 ||
		    rk_kms_hdmi_phy_i2c_write(sc,
		    HDMI_PHY_I2C_CKCALCTRL,
		    HDMI_PHY_I2C_CKCALCTRL_OVERRIDE) != 0)
			return (EIO);

		cfg0 = hdmi_read1(sc, HDMI_PHY_CONF0);
		cfg0 |= HDMI_PHY_CONF0_PDZ;
		hdmi_write1(sc, HDMI_PHY_CONF0, cfg0);
		DELAY(1000);

		cfg0 &= ~HDMI_PHY_CONF0_ENTMDS;
		hdmi_write1(sc, HDMI_PHY_CONF0, cfg0);
		DELAY(1000);
		cfg0 |= HDMI_PHY_CONF0_ENTMDS;
		hdmi_write1(sc, HDMI_PHY_CONF0, cfg0);
		DELAY(1000);

		cfg0 |= HDMI_PHY_CONF0_TXPWRON | HDMI_PHY_CONF0_SVSRET;
		cfg0 &= ~HDMI_PHY_CONF0_PDDQ;
		hdmi_write1(sc, HDMI_PHY_CONF0, cfg0);
		DELAY(5000);
	}

	hdmi_write1(sc, HDMI_MC_CLKDIS, 0x00);
	hdmi_write1(sc, HDMI_FC_VSYNCINWIDTH, (uint8_t)vsync_len);
	rk_kms_hdmi_toggle_main_reset(sc,
	    HDMI_MC_SWRST_TMDS | HDMI_MC_SWRST_PIXEL);

	for (timeout = 20; timeout > 0; timeout--) {
		DELAY(5000);
		if ((hdmi_read1(sc, HDMI_PHY_STAT0) & 0x01) != 0) {
			DPRINTF(sc, "PHY locked at %u kHz "
			    "(mpll=%04x phy_sym=%04x)\n", mode->clock,
			    mpll->cpce, phy->sym);
			return (0);
		}
	}
	/*
	 * The RK3399 Innosilicon HDMI PHY appears to advance through its
	 * power-up FSM without ever asserting PHY_STAT0[0] (the Synopsys-
	 * generic TX_PHY_LOCK bit).  Observed evidence: MC_LOCKONCLOCK
	 * reports the TMDS / pixel clock domains locked, and the panel
	 * sees the framer output as "flashing color → no signal" (i.e.
	 * the link is up but downstream config — finish_mode + enable_
	 * hdmi_mode — is being skipped because we bailed on the lock-bit
	 * timeout).  Promote the timeout to a non-fatal warning when
	 * hdmi_skip_lock_check is set, so the rest of the bring-up
	 * proceeds and we can see whether AVI infoframe + HDMIDVI bit +
	 * VP_STUFF/CONF make the sink lock.  Default on while we sort
	 * out the real lock indicator.
	 */
	if (sc->hdmi_skip_lock_check != 0) {
		DPRINTF(sc, "PHY lock bit timeout — proceeding anyway "
		    "(stat0=0x%02x lock=0x%02x conf0=0x%02x)\n",
		    hdmi_read1(sc, HDMI_PHY_STAT0),
		    hdmi_read1(sc, HDMI_MC_LOCKONCLOCK),
		    hdmi_read1(sc, HDMI_PHY_CONF0));
		return (0);
	}
	DPRINTF(sc, "PHY pll lock timeout stat0=0x%02x lock=0x%02x "
	    "conf0=0x%02x\n",
	    hdmi_read1(sc, HDMI_PHY_STAT0),
	    hdmi_read1(sc, HDMI_MC_LOCKONCLOCK),
	    hdmi_read1(sc, HDMI_PHY_CONF0));
	return (ETIMEDOUT);
}

/*
 * CEA-861 Video Identification Code lookup.  rk_drm calls drm2's
 * drm_mode_cea_vic; that's a symbol we can't use (drm2 collision
 * rules).  Cover the common bring-up modes inline — VIC 16 for
 * 1920x1080@60, 4 for 1280x720@60, 0 ("unknown / DVI mode") for
 * anything else.  Returning 0 sends a generic AVI infoframe, which
 * most sinks accept.  Phase 9g could grow the table but the 1080p
 * row is what canned-mode tests use.
 */
static uint8_t
rk_kms_cea_vic(const struct drm_display_mode *m)
{
	uint32_t refresh = kms_mode_vrefresh(m);

	if (m->hdisplay == 1920 && m->vdisplay == 1080 && refresh == 60)
		return (16);
	if (m->hdisplay == 1280 && m->vdisplay == 720 && refresh == 60)
		return (4);
	if (m->hdisplay == 720 && m->vdisplay == 480 && refresh == 60)
		return (3);
	return (0);
}

/*
 * Picture aspect ratio for the AVI infoframe.  Match rk_drm: 16:9 if
 * the display extent satisfies hdisplay * 9 == vdisplay * 16, 4:3 if
 * hdisplay * 3 == vdisplay * 4, otherwise unspecified (0).
 */
static uint8_t
rk_kms_picture_aspect(const struct drm_display_mode *m)
{
	if ((uint32_t)m->hdisplay * 9 == (uint32_t)m->vdisplay * 16)
		return (HDMI_FC_AVICONF1_PICTURE_ASPECT_16_9);
	if ((uint32_t)m->hdisplay * 3 == (uint32_t)m->vdisplay * 4)
		return (HDMI_FC_AVICONF1_PICTURE_ASPECT_4_3);
	return (0);
}

/*
 * Clear the TMDS-overflow latch.  Sequence lifted from rk_drm — drop
 * TMDS out of reset, write INVIDCONF four times, then re-arm TMDS.
 * Required after any change to FC_INVIDCONF because the IP latches
 * the new sync polarity only after several writes.
 */
static void
rk_kms_hdmi_clear_overflow(struct rk_kms_softc *sc)
{
	uint8_t swrstz, invidconf;
	int i;

	swrstz = hdmi_read1(sc, HDMI_MC_SWRSTZREQ);
	hdmi_write1(sc, HDMI_MC_SWRSTZREQ, swrstz & ~HDMI_MC_SWRST_TMDS);
	invidconf = hdmi_read1(sc, HDMI_FC_INVIDCONF);
	for (i = 0; i < 4; i++)
		hdmi_write1(sc, HDMI_FC_INVIDCONF, invidconf);
	hdmi_write1(sc, HDMI_MC_SWRSTZREQ, swrstz | HDMI_MC_SWRST_TMDS);
}

/*
 * Program the video composer's input timing block.  hdmi_mode = true
 * sets HDMI_MODE in INVIDCONF (vs DVI).  All timing values are
 * 12-bit-or-less and split across two 1-byte registers, low first.
 */
static void
rk_kms_hdmi_program_av_composer(struct rk_kms_softc *sc,
    const struct drm_display_mode *m, bool hdmi_mode)
{
	uint8_t inv;
	uint16_t hblank = m->htotal - m->hdisplay;
	uint16_t vblank = m->vtotal - m->vdisplay;
	uint16_t hfp = m->hsync_start - m->hdisplay;
	uint16_t vfp = m->vsync_start - m->vdisplay;
	uint16_t hsync = m->hsync_end - m->hsync_start;
	uint16_t vsync = m->vsync_end - m->vsync_start;

	inv = HDMI_FC_INVIDCONF_DE_HIGH;
	if (m->flags & KMS_MODE_FLAG_PVSYNC)
		inv |= HDMI_FC_INVIDCONF_VSYNC_HIGH;
	if (m->flags & KMS_MODE_FLAG_PHSYNC)
		inv |= HDMI_FC_INVIDCONF_HSYNC_HIGH;
	if (m->flags & KMS_MODE_FLAG_INTERLACE)
		inv |= HDMI_FC_INVIDCONF_R_V_BLANK_HIGH |
		    HDMI_FC_INVIDCONF_INTERLACED;
	if (hdmi_mode)
		inv |= HDMI_FC_INVIDCONF_HDMI_MODE;

	hdmi_write1(sc, HDMI_FC_INVIDCONF, inv);
	hdmi_write1(sc, HDMI_FC_INHACTV1, (m->hdisplay >> 8) & 0xff);
	hdmi_write1(sc, HDMI_FC_INHACTV0, m->hdisplay & 0xff);
	hdmi_write1(sc, HDMI_FC_INVACTV1, (m->vdisplay >> 8) & 0xff);
	hdmi_write1(sc, HDMI_FC_INVACTV0, m->vdisplay & 0xff);
	hdmi_write1(sc, HDMI_FC_INHBLANK1, (hblank >> 8) & 0xff);
	hdmi_write1(sc, HDMI_FC_INHBLANK0, hblank & 0xff);
	hdmi_write1(sc, HDMI_FC_INVBLANK, vblank & 0xff);
	hdmi_write1(sc, HDMI_FC_HSYNCINDELAY1, (hfp >> 8) & 0xff);
	hdmi_write1(sc, HDMI_FC_HSYNCINDELAY0, hfp & 0xff);
	hdmi_write1(sc, HDMI_FC_VSYNCINDELAY, vfp & 0xff);
	hdmi_write1(sc, HDMI_FC_HSYNCINWIDTH1, (hsync >> 8) & 0xff);
	hdmi_write1(sc, HDMI_FC_HSYNCINWIDTH0, hsync & 0xff);
	hdmi_write1(sc, HDMI_FC_VSYNCINWIDTH, vsync & 0xff);

	hdmi_write1(sc, HDMI_FC_AVIVID, rk_kms_cea_vic(m));
}

/*
 * DVI-mode bring-up — programs the video composer with hdmi_mode=false
 * and clears the HDMIDVI bit so the HDCP shim treats the link as DVI.
 * Called from hdmi_init_mode below before the PHY MPLL i2c writes;
 * without it the DW HDMI controller leaves the framer in a state that
 * blocks the PHY i2c master (its done bit never fires).
 */
static void
rk_kms_hdmi_enable_dvi(struct rk_kms_softc *sc,
    const struct drm_display_mode *m)
{
	uint8_t hdcpcfg0;

	rk_kms_hdmi_program_av_composer(sc, m, false);
	hdcpcfg0 = hdmi_read1(sc, HDMI_A_HDCPCFG0);
	hdcpcfg0 &= ~HDMI_A_HDCPCFG0_HDMIDVI;
	hdmi_write1(sc, HDMI_A_HDCPCFG0, hdcpcfg0);
	hdmi_write1(sc, HDMI_A_HDCPCFG1, HDMI_A_HDCPCFG1_DEFAULT);
	hdmi_write1(sc, HDMI_A_VIDPOLCFG, HDMI_A_VIDPOLCFG_DATAENPOL);
}

/*
 * Pre-PHY DW HDMI controller init.  Runs before hdmi_phy_init so the
 * framer's control durations, channel preambles, and AVI config land
 * in a known state — the PHY i2c master inside the controller won't
 * respond otherwise.  Mirrors rk_drm_dw_hdmi_init_mode.
 */
static void
rk_kms_hdmi_init_mode(struct rk_kms_softc *sc,
    const struct drm_display_mode *m)
{
	rk_kms_hdmi_enable_dvi(sc, m);
	hdmi_write1(sc, HDMI_FC_CTRLDUR, 12);
	hdmi_write1(sc, HDMI_FC_EXCTRLDUR, 32);
	hdmi_write1(sc, HDMI_FC_EXCTRLSPAC, 1);
	hdmi_write1(sc, HDMI_FC_CH0PREAM, 0x0b);
	hdmi_write1(sc, HDMI_FC_CH1PREAM, 0x16);
	hdmi_write1(sc, HDMI_FC_CH2PREAM, 0x21);
	hdmi_write1(sc, HDMI_FC_AVICONF3, 0x00);
	hdmi_write1(sc, HDMI_FC_GCP, 0x00);
	hdmi_write1(sc, HDMI_FC_AVICONF0, 0x00);
	hdmi_write1(sc, HDMI_FC_AVICONF1, 0x00);
	hdmi_write1(sc, HDMI_FC_AVICONF2, 0x00);
	hdmi_write1(sc, HDMI_FC_AVIVID, rk_kms_cea_vic(m));
	/*
	 * 0x01ff is the FC_PRCONF (pixel-repetition) low byte; 0x0184 is
	 * the VP_HDCP_KEEPOUT register.  rk_drm writes both with these
	 * fixed values — no datasheet name, but the live captures show
	 * 0xfe at 0x0184 on a working link.
	 */
	hdmi_write1(sc, 0x01ff, 0x00);
	hdmi_write1(sc, 0x0184, 0xfe);
}

/*
 * Post-PHY DW HDMI video composer setup.  Programs MC_CLKDIS to ungate
 * the pixel/TMDS clocks, VP_STUFF/VP_CONF/VP_REMAP for 8bpc output
 * mode, and VSYNC width.  Without this finish step the framer trains
 * the PHY but never opens its video path → panel sees signal but no
 * picture.  Mirrors rk_drm_dw_hdmi_finish_mode.
 */
static void
rk_kms_hdmi_finish_mode(struct rk_kms_softc *sc,
    const struct drm_display_mode *m)
{
	uint8_t clkdis;
	uint8_t val;
	uint16_t vsync = m->vsync_end - m->vsync_start;

	hdmi_write1(sc, HDMI_MC_FLOWCTRL, 0x00);
	hdmi_write1(sc, HDMI_FC_CTRLDUR, 12);
	hdmi_write1(sc, HDMI_FC_EXCTRLDUR, 32);
	hdmi_write1(sc, HDMI_FC_EXCTRLSPAC, 1);
	hdmi_write1(sc, HDMI_FC_CH0PREAM, 0x0b);
	hdmi_write1(sc, HDMI_FC_CH1PREAM, 0x16);
	hdmi_write1(sc, HDMI_FC_CH2PREAM, 0x21);

	clkdis = hdmi_read1(sc, HDMI_MC_CLKDIS) &
	    HDMI_MC_CLKDIS_CECCLK_DISABLE;
	clkdis |= (uint8_t)~HDMI_MC_CLKDIS_CECCLK_DISABLE;
	clkdis &= ~0x01;
	hdmi_write1(sc, HDMI_MC_CLKDIS, clkdis);
	clkdis &= ~0x02;
	hdmi_write1(sc, HDMI_MC_CLKDIS, clkdis);
	hdmi_write1(sc, HDMI_FC_VSYNCINWIDTH, vsync & 0xff);

	hdmi_write1(sc, HDMI_VP_PR_CD, 0x40);

	val = hdmi_read1(sc, HDMI_VP_STUFF);
	val &= ~0x01;
	val |= 0x01;
	hdmi_write1(sc, HDMI_VP_STUFF, val);

	val = hdmi_read1(sc, HDMI_VP_CONF);
	val &= ~(0x10 | 0x04);
	val |= 0x04;
	hdmi_write1(sc, HDMI_VP_CONF, val);

	hdmi_write1(sc, HDMI_VP_REMAP, 0x00);

	val = hdmi_read1(sc, HDMI_VP_CONF);
	val &= ~(0x40 | 0x20 | 0x08);
	val |= 0x40;
	hdmi_write1(sc, HDMI_VP_CONF, val);

	val = hdmi_read1(sc, HDMI_VP_STUFF);
	val &= ~(0x02 | 0x04);
	val |= 0x02 | 0x04;
	hdmi_write1(sc, HDMI_VP_STUFF, val);

	val = hdmi_read1(sc, HDMI_VP_CONF);
	val &= ~0x03;
	val |= 0x03;
	hdmi_write1(sc, HDMI_VP_CONF, val);

	/*
	 * TX (Transmitter) tail: rk_drm writes these at the end of
	 * dw_hdmi_finish_mode.  TX_INVID0 = 0x01 marks the input video
	 * source as "internal" (VP-fed) instead of an external IF; 0x0201
	 * = 0x07 enables YCC/RGB conversion paths in the TX block; the
	 * 0x0202..0x0207 zero-fill clears the rest of the TX_INVID
	 * register file.  Without these the controller leaves residual TX
	 * config in place from u-boot's splash setup and the framer can
	 * mis-route pixels on the way to the PHY.
	 */
	hdmi_write1(sc, HDMI_TX_INVID0, 0x01);
	hdmi_write1(sc, 0x0201, 0x07);
	hdmi_write1(sc, 0x0202, 0x00);
	hdmi_write1(sc, 0x0203, 0x00);
	hdmi_write1(sc, 0x0204, 0x00);
	hdmi_write1(sc, 0x0205, 0x00);
	hdmi_write1(sc, 0x0206, 0x00);
	hdmi_write1(sc, 0x0207, 0x00);

	rk_kms_hdmi_clear_overflow(sc);
}

/*
 * Bring the DW HDMI controller into HDMI mode for the active modeline.
 * Programs the video composer, AVI infoframe, AVI-packet enables, and
 * HDCP shim into their open/disabled state.  Called after PHY init
 * completes.  Mirrors rk_drm_hdmi_enable_hdmi_mode.
 */
static void
rk_kms_hdmi_enable(struct rk_kms_softc *sc,
    const struct drm_display_mode *m)
{
	uint8_t hdcpcfg0, pkt_en, aspect, vic;

	rk_kms_hdmi_program_av_composer(sc, m, true);
	aspect = rk_kms_picture_aspect(m);
	vic = rk_kms_cea_vic(m);

	hdmi_write1(sc, HDMI_FC_AVICONF3, 0x00);
	hdmi_write1(sc, HDMI_FC_AVICONF0, 0x00);
	hdmi_write1(sc, HDMI_FC_AVICONF1, aspect);
	hdmi_write1(sc, HDMI_FC_AVICONF2, 0x00);
	hdmi_write1(sc, HDMI_FC_AVIVID, vic);

	pkt_en = hdmi_read1(sc, HDMI_FC_PACKET_TX_EN);
	pkt_en |= HDMI_FC_PACKET_TX_EN_AVI | HDMI_FC_PACKET_TX_EN_GCP;
	hdmi_write1(sc, HDMI_FC_PACKET_TX_EN, pkt_en);
	hdmi_write1(sc, HDMI_PKT_SEND_CTL, HDMI_PKT_SEND_CTL_AVI_INFO_UP);
	DELAY(10);
	hdmi_write1(sc, HDMI_PKT_SEND_CTL, HDMI_PKT_SEND_CTL_AVI_INFO_EN);

	hdcpcfg0 = hdmi_read1(sc, HDMI_A_HDCPCFG0);
	hdcpcfg0 |= HDMI_A_HDCPCFG0_HDMIDVI;
	hdmi_write1(sc, HDMI_A_HDCPCFG0, hdcpcfg0);
	hdmi_write1(sc, HDMI_A_HDCPCFG1, HDMI_A_HDCPCFG1_DEFAULT);
	hdmi_write1(sc, HDMI_A_VIDPOLCFG, HDMI_A_VIDPOLCFG_DATAENPOL);

	rk_kms_hdmi_clear_overflow(sc);
	DPRINTF(sc, "HDMI enable: vic=%u aspect=0x%02x mode=%s\n", vic,
	    aspect, m->name);
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
 * if the framebuffer has no GEM backing or if the base lives above
 * 4 GiB — VOP_BIG's WIN0_YRGB_MST is a 32-bit register, so silently
 * truncating a >4 GiB pa would steer the DMA engine into nonsense
 * and panic the box on the first scanout.  Caller treats "0" as
 * "skip programming WIN0," which is the safe behavior until GEM
 * starts allocating into a 32-bit-low pool.
 */
static vm_paddr_t
rk_kms_fb_paddr(struct drm_framebuffer *fb)
{
	struct drm_gem_object *obj;
	vm_paddr_t pa;

	if (fb == NULL)
		return (0);
	obj = fb->gem_objs[0];
	if (obj == NULL || obj->pages == NULL || obj->npages == 0)
		return (0);
	pa = VM_PAGE_TO_PHYS(obj->pages[0]);
	if (pa > UINT32_MAX)
		return (0);
	return (pa);
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
		struct drm_gem_object *obj;
		vm_paddr_t raw;

		obj = (fb != NULL) ? fb->gem_objs[0] : NULL;
		raw = (obj != NULL && obj->pages != NULL && obj->npages > 0)
		    ? VM_PAGE_TO_PHYS(obj->pages[0]) : 0;
		DPRINTF(sc, "win0: skipping (pa=0x%jx unaddressable by VOP)\n",
		    (uintmax_t)raw);
		return;
	}
	/*
	 * Cache coherency (2026-07-20): RK3399 VOP DMA is non-coherent
	 * with the CPU caches.  For UNCACHEABLE-mapped GEM pages this is
	 * a belt-and-braces flush; for legacy WB-mapped code paths (x86
	 * cross-arch, or if we ever switch memattr back) it's the only
	 * thing that guarantees fb DRAM has the latest pixels the caller
	 * expects to be scanned out.  Walk pages one-by-one — GEM
	 * allocator is not required to hand back physically-contiguous
	 * pages, so flushing a single npages*PAGE_SIZE range off pa
	 * (contiguous only for page 0) would touch unrelated physical
	 * memory and can SEA-fault on arm64 if it walks off DRAM.  Gated
	 * by cache_flush_fb sysctl (default on).
	 */
	if (sc->cache_flush_fb && fb != NULL && fb->gem_objs[0] != NULL &&
	    fb->gem_objs[0]->pages != NULL &&
	    fb->gem_objs[0]->npages > 0) {
		struct drm_gem_object *cobj = fb->gem_objs[0];
		for (uint32_t i = 0; i < cobj->npages; i++) {
			vm_paddr_t ppa = VM_PAGE_TO_PHYS(cobj->pages[i]);
			void *va = (void *)PHYS_TO_DMAP(ppa);
			cpu_dcache_wb_range(va, PAGE_SIZE);
		}
	}
	/*
	 * Source dims come from the fb (Xorg's chosen mode allocates a
	 * matching-size dumb buffer).  Destination dims come from the
	 * display timing (always native 1920x1080 for our DP link).  If
	 * fb != dst, engage the VOP hardware scaler so we can advertise
	 * lower-res modes without retraining DP.
	 */
	uint32_t src_w = (fb != NULL) ? fb->width : mode->hdisplay;
	uint32_t src_h = (fb != NULL) ? fb->height : mode->vdisplay;
	uint32_t dst_w = mode->hdisplay;
	uint32_t dst_h = mode->vdisplay;
	uint32_t ctrl1 = 0;
	uint32_t ctrl0_upper;
	uint32_t lb_mode;

	stride_bytes = roundup2(src_w, 16) * 4u;	/* XR24 stride */
	stride_words = stride_bytes / 4u;

	vop_big_write(sc, VOP_REG_WIN0_YRGB_BUFSIZE, 0u);
	vop_big_write(sc, VOP_REG_WIN0_VIR, stride_words);
	vop_big_write(sc, VOP_REG_WIN0_YRGB_MST, (uint32_t)pa);
	vop_big_write(sc, VOP_REG_WIN0_ACT_INFO,
	    ((src_h - 1u) << 16) | (src_w - 1u));
	vop_big_write(sc, VOP_REG_WIN0_DSP_INFO,
	    ((dst_h - 1u) << 16) | (dst_w - 1u));
	vop_big_write(sc, VOP_REG_WIN0_DSP_ST,
	    (vact_start << 16) | hact_start);
	if (sc->output == RK_KMS_OUT_DP) {
		vop_big_write(sc, VOP_REG_WIN0_SRC_ALPHA,
		    VOP_WIN0_SRC_ALPHA_OPAQUE);
		vop_big_write(sc, VOP_REG_WIN0_DST_ALPHA,
		    VOP_WIN0_DST_ALPHA_OPAQUE);
	}
	vop_big_write(sc, VOP_REG_WIN0_CTRL2, VOP_WIN0_CTRL2_PRIMARY);
	vop_big_write(sc, VOP_REG_POST_DSP_HACT,
	    (hact_start << 16) | (hact_start + dst_w));
	vop_big_write(sc, VOP_REG_POST_DSP_VACT,
	    (vact_start << 16) | (vact_start + dst_h));

	/*
	 * Scaler programming.  Formula per Linux
	 * drm/rockchip/rockchip_drm_vop.h scl_cal_scale(src, dst, 16):
	 *   factor = ((src * 2 - 3) << 15) / (dst - 1)
	 * Same math for both bicubic UP and bilinear UP paths — the
	 * filter selection is a separate mode bit.
	 */
	if (src_w != dst_w || src_h != dst_h) {
		uint32_t fx, fy, mode_h, mode_v;

		fx = (uint32_t)(((int64_t)(src_w * 2 - 3) << 15) /
		    (int64_t)(dst_w - 1)) & 0xffff;
		fy = (uint32_t)(((int64_t)(src_h * 2 - 3) << 15) /
		    (int64_t)(dst_h - 1)) & 0xffff;
		vop_big_write(sc, VOP_REG_WIN0_SCL_FACTOR_YRGB,
		    (fy << 16) | fx);
		vop_big_write(sc, VOP_REG_WIN0_SCL_FACTOR_CBR, 0);
		vop_big_write(sc, VOP_REG_WIN0_SCL_OFFSET, 0);

		mode_h = (src_w < dst_w) ? VOP_WIN0_SCL_MODE_UP :
		    (src_w > dst_w) ? VOP_WIN0_SCL_MODE_DOWN : 0;
		mode_v = (src_h < dst_h) ? VOP_WIN0_SCL_MODE_UP :
		    (src_h > dst_h) ? VOP_WIN0_SCL_MODE_DOWN : 0;
		/*
		 * bit 15: line_load_mode (0 = normal, load 1 line at a time)
		 * bit 0:  yrgb_axi_gather_en (enable AXI burst gather —
		 *         required for scaler to fetch source pixels)
		 * bits 11:8 yrgb_axi_gather_num = 0 (max burst)
		 */
		ctrl1 = (mode_h << 16) | (mode_v << 18) |
		    (VOP_WIN0_SCL_VSU_BIC << 22) |
		    (1u << 0);
		lb_mode = (src_w > 2560) ? VOP_WIN0_LB_RGB_3840X2 :
		    (src_w > 1920) ? VOP_WIN0_LB_RGB_2560X4 :
		    VOP_WIN0_LB_RGB_1920X5;
	} else {
		/* Passthrough: no scaler, no line-buffer preallocation. */
		vop_big_write(sc, VOP_REG_WIN0_SCL_FACTOR_YRGB, 0);
		vop_big_write(sc, VOP_REG_WIN0_SCL_FACTOR_CBR, 0);
		vop_big_write(sc, VOP_REG_WIN0_SCL_OFFSET, 0);
		lb_mode = VOP_WIN0_LB_RGB_1920X5;
	}
	vop_big_write(sc, VOP_REG_WIN0_CTRL1, ctrl1);

	ctrl0_upper = (sc->output == RK_KMS_OUT_DP ?
	    VOP_WIN0_CTRL0_UPPER_DP : VOP_WIN0_CTRL0_UPPER_HDMI) |
	    VOP_WIN0_CTRL0_LOWER;
	vop_big_write(sc, VOP_REG_WIN0_CTRL0,
	    ctrl0_upper | (lb_mode << VOP_WIN0_CTRL0_LB_MODE_SHIFT));
	DPRINTF(sc, "win0: pa=0x%jx src=%ux%u dst=%ux%u stride=%u"
	    " ctrl1=0x%08x lb=%u\n",
	    (uintmax_t)pa, src_w, src_h, dst_w, dst_h, stride_bytes,
	    ctrl1, lb_mode);
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
 * SYS_CTRL out of standby and into HDMI-driven RGB mode.
 *
 * commit_modeset is a bitmask of stages to actually execute.  Each
 * stage represents one chunk of hardware writes that could AXI-hang
 * the CPU if a clock/PD prerequisite is missing.  Bracketing every
 * stage with DPRINTF("STAGE foo STARTING") + DPRINTF("STAGE foo
 * DONE") lets us bisect the freeze with debug=1 by watching the
 * framebuffer console: the last "STARTING" line without a matching
 * "DONE" names the culprit access.
 *
 *   commit_modeset == 0 : set_config is a no-op (gated in caller)
 *   commit_modeset != 0 : run the dry-run summary, then execute every
 *                         stage whose bit is set.
 *
 * Bisection recipe: set 0x01, restart slim, check dmesg.  If alive, OR
 * in the next bit (0x03), repeat.  Power-cycle on freeze; the previous
 * boot's "STARTING" on the framebuffer names the bad stage.
 */
#define	RK_KMS_STAGE_ROUTE	(1u << 0)
#define	RK_KMS_STAGE_VPLL		(1u << 1)
#define	RK_KMS_STAGE_VOP_SYS	(1u << 2)
#define	RK_KMS_STAGE_VOP_TIMING	(1u << 3)
#define	RK_KMS_STAGE_WIN0		(1u << 4)
#define	RK_KMS_STAGE_HDMI_DP	(1u << 5)
#define	RK_KMS_STAGE_CFG_DONE	(1u << 6)
#define	RK_KMS_STAGE_ALL		0x7fu

static void
rk_kms_vop_program_timing(struct rk_kms_softc *sc,
    const struct drm_display_mode *mode_in, struct drm_framebuffer *fb)
{
	struct drm_display_mode forced;
	const struct drm_display_mode *mode = mode_in;
	uint32_t hact_start, vact_start, hsync_len, vsync_len;
	uint32_t stages = sc->commit_modeset;
	uint32_t sys_ctrl;
	vm_paddr_t pa;
	uint32_t stride;
	int error;

	/*
	 * Re-arm PD_VO / VOPB BUS_IDLE / GATEDIS_VOPB at the top of every
	 * modeset.  attach() runs domain_sanity once but the boot fb's
	 * power state can drift between attach and userspace's first
	 * SETCRTC; without this re-run the very first VOP MMIO from a
	 * SETCRTC-driven modeset has hung the AXI bus on cold boots.
	 * Mirrors what rk_drm does on every modeset.  Idempotent: every
	 * write is masked or value-equality-checked.
	 */
	rk_kms_display_domain_sanity(sc);

	/*
	 * For USB-C DP, swap in the DMT-style timing the XYM panel needs.
	 * Mode is `const` because the framework hands us a shared pointer,
	 * but we only mutate our local stack copy + redirect the pointer
	 * before any HW write.  htotal/vtotal/clock match CEA 1080p60 so
	 * VPLL math doesn't change; only the hsync placement+width and
	 * sync polarity differ.  flags forced to NHSYNC + PVSYNC.
	 */
	if (sc->output == RK_KMS_OUT_DP && sc->dp_force_mode != 0 &&
	    mode_in->clock >= RK_DP_FORCED_CLOCK_KHZ - 250 &&
	    mode_in->clock <= RK_DP_FORCED_CLOCK_KHZ + 250 &&
	    mode_in->hdisplay == RK_DP_FORCED_HDISPLAY &&
	    mode_in->vdisplay == RK_DP_FORCED_VDISPLAY) {
		forced = *mode_in;
		forced.clock = RK_DP_FORCED_CLOCK_KHZ;
		forced.hsync_start = RK_DP_FORCED_HSYNC_START;
		forced.hsync_end = RK_DP_FORCED_HSYNC_END;
		forced.htotal = RK_DP_FORCED_HTOTAL;
		forced.vsync_start = RK_DP_FORCED_VSYNC_START;
		forced.vsync_end = RK_DP_FORCED_VSYNC_END;
		forced.vtotal = RK_DP_FORCED_VTOTAL;
		forced.flags &= ~(KMS_MODE_FLAG_PHSYNC |
		    KMS_MODE_FLAG_NHSYNC |
		    KMS_MODE_FLAG_PVSYNC |
		    KMS_MODE_FLAG_NVSYNC);
		forced.flags |= KMS_MODE_FLAG_NHSYNC |
		    KMS_MODE_FLAG_PVSYNC;
		mode = &forced;
		DPRINTF(sc, "DP forced mode: hsync %u-%u (len %u) "
		    "NHSYNC+PVSYNC\n",
		    forced.hsync_start, forced.hsync_end,
		    forced.hsync_end - forced.hsync_start);
	}
	hact_start = rk_kms_hact_start(mode);
	vact_start = rk_kms_vact_start(mode);
	hsync_len = mode->hsync_end - mode->hsync_start;
	vsync_len = mode->vsync_end - mode->vsync_start;

	pa = rk_kms_fb_paddr(fb);
	stride = roundup2(mode->hdisplay, 16) * 4u;
	DPRINTF(sc, "modeset: out=%d %ux%u clk=%u h(s=%u t=%u "
	    "as=%u) v(s=%u t=%u as=%u) fb pa=0x%jx stride=%u stages=0x%x\n",
	    sc->output, mode->hdisplay, mode->vdisplay, mode->clock,
	    hsync_len, mode->htotal, hact_start, vsync_len, mode->vtotal,
	    vact_start, (uintmax_t)pa, stride, stages);

	if (stages == 0)
		return;

	if ((stages & RK_KMS_STAGE_ROUTE) != 0) {
		DPRINTF(sc, "STAGE route STARTING\n");
		if (sc->output == RK_KMS_OUT_DP)
			rk_kms_route_vop_big_to_dp(sc);
		else
			rk_kms_route_vop_big_to_hdmi(sc);
		DPRINTF(sc, "STAGE route DONE\n");
	}

	if ((stages & RK_KMS_STAGE_VPLL) != 0) {
		DPRINTF(sc, "STAGE vpll STARTING (%u kHz)\n",
		    mode->clock);
		error = rk_kms_program_vpll(sc, mode->clock);
		/*
		 * Point DCLK_VOP0 at the freshly programmed VPLL.  Mirrors
		 * rk_drm_vop_init_mode()'s post-VPLL CLKSEL writes — without
		 * these the VOP still scans out, but the pixel clock is
		 * sourced from whatever U-Boot picked, so the panel sees
		 * mistimed pixels and reports "no signal".
		 */
		cru_write(sc, CRU_CLKSEL_CON47,
		    ((((0x1fu << 8) | (0x3u << 6) | 0x1fu) << 16) |
		    ((3u << 8) | (1u << 6) | 1u)));
		cru_write(sc, CRU_CLKSEL_CON49,
		    ((((1u << 11) | (0x3u << 8) | 0xffu) << 16) | 0x0000u));
		DPRINTF(sc, "STAGE vpll DONE rc=%d (DCLK mux set)\n",
		    error);
	}

	if ((stages & RK_KMS_STAGE_VOP_SYS) != 0) {
		DPRINTF(sc, "STAGE vop_sys STARTING\n");
		sys_ctrl = vop_big_read(sc, VOP_REG_SYS_CTRL);
		DPRINTF(sc, "STAGE vop_sys read=0x%x\n", sys_ctrl);
		sys_ctrl &= ~(VOP_SYS_CTRL_STANDBY | VOP_SYS_CTRL_MMU_EN);
		sys_ctrl |= VOP_SYS_CTRL_ENABLE | VOP_SYS_CTRL_RGB_EN |
		    VOP_SYS_CTRL_HDMI_EN;
		vop_big_write(sc, VOP_REG_SYS_CTRL, sys_ctrl);
		DPRINTF(sc, "STAGE vop_sys DONE write=0x%x\n",
		    sys_ctrl);
	}

	if ((stages & RK_KMS_STAGE_VOP_TIMING) != 0) {
		uint32_t dsp_ctrl0, dsp_ctrl1, post_scl, pin_pol;

		DPRINTF(sc, "STAGE vop_timing STARTING\n");
		/*
		 * DSP_CTRL0 = AAAA out_mode (the Cadence DP framer and DW HDMI
		 * controller both want 32-bit RGB+alpha).  Pin polarity lives
		 * in different fields per output:
		 *   HDMI: DSP_CTRL1[22:20] = pin pol, [23] = DCLK pol
		 *   DP:   DSP_CTRL1[18:16] = pin pol, [19] = DCLK pol (force 0)
		 * On RK3399 (VOP 3.5) DSP_CTRL0[6:4] are NOT pin polarity (the
		 * legacy bits) — must be cleared on DP, leftover HDMI state
		 * there confuses the Cadence framer.
		 */
		dsp_ctrl0 = vop_big_read(sc, VOP_REG_DSP_CTRL0);
		dsp_ctrl0 &= ~VOP_DSP_OUT_MODE_MASK;
		dsp_ctrl0 |= VOP_DSP_OUT_MODE_AAAA;
		dsp_ctrl1 = vop_big_read(sc, VOP_REG_DSP_CTRL1);

		pin_pol = 0;
		if ((mode->flags & KMS_MODE_FLAG_NHSYNC) == 0)
			pin_pol |= (1u << 0);
		if ((mode->flags & KMS_MODE_FLAG_NVSYNC) == 0)
			pin_pol |= (1u << 1);

		if (sc->output == RK_KMS_OUT_DP) {
			dsp_ctrl0 &= ~(VOP_DSP_CTRL0_PIN_POL_MASK |
			    VOP_DSP_CTRL0_DCLK_POL |
			    VOP_DSP_CTRL0_P2I_EN |
			    VOP_DSP_CTRL0_INTERLACE);
			dsp_ctrl0 &= ~(0x1fu << 12);	/* dsp_data_swap=0 */
			dsp_ctrl1 &= ~(VOP_DSP_CTRL1_DP_PIN_POL_MASK |
			    VOP_DSP_CTRL1_DP_DCLK_POL |
			    VOP_DSP_CTRL1_HDMI_PIN_POL_MASK |
			    VOP_DSP_CTRL1_HDMI_DCLK_POL);
			dsp_ctrl1 |= (pin_pol & 0x7) << 16;
			/* No dither down/pre-dither on the AAAA path. */
			dsp_ctrl1 &= ~((1u << 3) | (1u << 2) | (1u << 1));
			dsp_ctrl1 |= (1u << 4);
		} else {
			dsp_ctrl1 &= ~(VOP_DSP_CTRL1_HDMI_PIN_POL_MASK |
			    VOP_DSP_CTRL1_HDMI_DCLK_POL);
			dsp_ctrl1 |= ((pin_pol & 0x7) << 20) |
			    VOP_DSP_CTRL1_HDMI_DCLK_POL;
		}
		vop_big_write(sc, VOP_REG_DSP_CTRL0, dsp_ctrl0);
		vop_big_write(sc, VOP_REG_DSP_CTRL1, dsp_ctrl1);

		if (sc->output == RK_KMS_OUT_DP) {
			/*
			 * Clear post-scaler YUV-out + background.  Stale YUV
			 * state from u-boot has been observed to produce a
			 * trained DP link that emits no visible pixels.
			 */
			post_scl = vop_big_read(sc, VOP_REG_POST_SCL_CTRL);
			post_scl &= ~(1u << 2);
			vop_big_write(sc, VOP_REG_POST_SCL_CTRL, post_scl);
			vop_big_write(sc, VOP_REG_DSP_BG, 0u);
		}

		vop_big_write(sc, VOP_REG_DSP_HTOTAL,
		    ((uint32_t)mode->htotal << 16) | hsync_len);
		vop_big_write(sc, VOP_REG_DSP_HACT,
		    (hact_start << 16) | (hact_start + mode->hdisplay));
		vop_big_write(sc, VOP_REG_DSP_VTOTAL,
		    ((uint32_t)mode->vtotal << 16) | vsync_len);
		vop_big_write(sc, VOP_REG_DSP_VACT,
		    (vact_start << 16) | (vact_start + mode->vdisplay));

		/*
		 * Optional DCLK_VOP0 reset pulse, gated by vop_dclk_reset
		 * sysctl.  rk_drm only pulses on first scanout (cold attach)
		 * to avoid putting an already-running VOP into 41 ms of
		 * reset.  We don't track first-vs-subsequent state yet, so
		 * default off; operator flips to 1 if a stale clock domain
		 * needs reseating after VPLL changes.
		 */
		if (sc->vop_dclk_reset != 0) {
			cru_write(sc, CRU_SOFTRST_CON17,
			    (CRU_DRESETN_VOP0_REQ << 16) |
			    CRU_DRESETN_VOP0_REQ);
			DELAY(1000);
			cru_write(sc, CRU_SOFTRST_CON17,
			    (CRU_DRESETN_VOP0_REQ << 16));
			DELAY(40000);
		}

		DPRINTF(sc, "STAGE vop_timing DONE dsp_ctrl0=0x%x "
		    "dsp_ctrl1=0x%x\n", dsp_ctrl0, dsp_ctrl1);
	}

	if ((stages & RK_KMS_STAGE_WIN0) != 0) {
		DPRINTF(sc, "STAGE win0 STARTING\n");
		rk_kms_vop_program_win0(sc, mode, fb, hact_start,
		    vact_start);
		DPRINTF(sc, "STAGE win0 DONE\n");
	}

	/*
	 * Latch the VOP shadow registers BEFORE the framer (HDMI or DP) is
	 * told to consume our stream.  Without this ordering the Cadence DP
	 * framer (and DW HDMI controller) capture the still-stale u-boot
	 * timing, see a mistimed pixel clock, drop pixels, and the panel
	 * gets no signal at all.  rk_drm does the equivalent inside its
	 * per-output vop_init_mode_* before calling cdn_dp_enable_mode.
	 */
	if ((stages & RK_KMS_STAGE_CFG_DONE) != 0) {
		DPRINTF(sc, "STAGE cfg_done STARTING\n");
		vop_big_write(sc, VOP_REG_CFG_DONE, 0x00010001);
		DPRINTF(sc, "STAGE cfg_done DONE\n");
	}

	if ((stages & RK_KMS_STAGE_HDMI_DP) != 0) {
		DPRINTF(sc, "STAGE hdmi_dp STARTING\n");
		if (sc->hdmi_enable != 0 &&
		    sc->output == RK_KMS_OUT_HDMI) {
			/*
			 * Pre-PHY init must run before phy_init or the PHY
			 * i2c master never asserts done — the framer left in
			 * an indeterminate state blocks the i2c bus.  See
			 * rk_drm_hw_modeset_hdmi_lit for the canonical order:
			 * init_mode → phy_init → finish_mode → enable.
			 */
			rk_kms_hdmi_init_mode(sc, mode);
			error = rk_kms_hdmi_phy_init(sc, mode);
			if (error != 0) {
				DPRINTF(sc, "HDMI PHY init failed: %d\n",
				    error);
			} else {
				rk_kms_hdmi_finish_mode(sc, mode);
				rk_kms_hdmi_enable(sc, mode);
			}
		}
		if (sc->dp_enable != 0 && sc->output == RK_KMS_OUT_DP)
			(void)rk_kms_dp_modeset(sc, mode);
		DPRINTF(sc, "STAGE hdmi_dp DONE\n");
	}

}

/*
 * Translate kms mode flags to the bit set rk_cdn_dp_enable_mode
 * expects.  rk_cdn_dp's "flags" argument carries the DRM_MODE_FLAG_*
 * polarity bits (PHSYNC / NHSYNC / PVSYNC / NVSYNC) plus the
 * interlace bit, in the same encoding the Linux uapi uses.  The
 * kms-side constants match the uapi by design (Phase 5), so
 * the conversion is a 1:1 copy with a narrow type cast.
 */
static uint32_t
rk_kms_dp_mode_flags(const struct drm_display_mode *m)
{
	uint32_t f = 0;

	if (m->flags & KMS_MODE_FLAG_PHSYNC)
		f |= 0x0001u;
	if (m->flags & KMS_MODE_FLAG_NHSYNC)
		f |= 0x0002u;
	if (m->flags & KMS_MODE_FLAG_PVSYNC)
		f |= 0x0004u;
	if (m->flags & KMS_MODE_FLAG_NVSYNC)
		f |= 0x0008u;
	if (m->flags & KMS_MODE_FLAG_INTERLACE)
		f |= 0x0010u;
	return (f);
}

/*
 * Drive the Cadence MHDP DP TX through a modeset.  Three calls in
 * order: auto-bringup (idempotent — fast no-op once link is up),
 * enable_mode (programs framer timing from the modeline), and
 * set_video_active(true) (flips the video-on enable bit).  All three
 * are exported by rk_cdn_dp; MODULE_DEPEND below pulls them in at
 * load time.
 *
 * Logs the bring-up + enable_mode results via DPRINTF.  A failure in
 * auto-bringup typically means the USB-C cable orientation or PD
 * altmode hasn't latched yet; that's recoverable by retrying the
 * modeset once the user wiggles the cable or the fusb302 stack
 * completes its altmode handshake.  rk_cdn_dp owns the retry loop;
 * we just call it.
 */
static int
rk_kms_dp_modeset(struct rk_kms_softc *sc,
    const struct drm_display_mode *mode)
{
	int error;

	error = rk_cdn_dp_auto_bringup_default();
	if (error != 0) {
		DPRINTF(sc, "DP auto-bringup failed: %d\n", error);
		return (error);
	}

	error = rk_cdn_dp_enable_mode(mode->clock,
	    mode->hdisplay, mode->hsync_start, mode->hsync_end,
	    mode->htotal, mode->vdisplay, mode->vsync_start,
	    mode->vsync_end, mode->vtotal,
	    rk_kms_dp_mode_flags(mode));
	if (error != 0) {
		DPRINTF(sc, "DP enable_mode failed: %d\n", error);
		return (error);
	}

	error = rk_cdn_dp_set_video_active_first(true);
	if (error != 0) {
		DPRINTF(sc, "DP set_video_active failed: %d\n", error);
		return (error);
	}
	DPRINTF(sc, "DP modeset %ux%u clock=%u: video active\n",
	    mode->hdisplay, mode->vdisplay, mode->clock);
	return (0);
}

/*
 * Phase 12 — boot framebuffer + fbd bridge.
 *
 * Allocate a 1920x1080 XRGB8888 framebuffer via bus_dma so we have both
 * a kernel VA (for vt to write to) and a contig physical address
 * (for VOP_BIG WIN0 to scan).  Populate sc->fb_info so the fb_getinfo
 * DEVMETHOD has something to return.  The actual VOP wire-up to scan
 * from sc->fb_pa is left to SETCRTC / a future bootfb sysctl — Phase 12
 * part 1 only wants /dev/ttyv* to exist so Xorg can grab a console.
 */
static void
rk_kms_fb_dma_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	bus_addr_t *fb_busaddr = arg;

	if (error != 0 || nseg != 1)
		return;
	*fb_busaddr = segs[0].ds_addr;
}

static int
rk_kms_fb_alloc(struct rk_kms_softc *sc)
{
	bus_addr_t fb_busaddr;
	void *fb_kva;
	int error;

	sc->fb_size = RK_KMS_FB_SIZE;
	error = bus_dma_tag_create(NULL, PAGE_SIZE, 0,
	    BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR,
	    NULL, NULL, sc->fb_size, 1, sc->fb_size, 0,
	    NULL, NULL, &sc->fb_dma_tag);
	if (error != 0)
		return (error);

	error = bus_dmamem_alloc(sc->fb_dma_tag, &fb_kva,
	    BUS_DMA_WAITOK | BUS_DMA_COHERENT | BUS_DMA_ZERO,
	    &sc->fb_dma_map);
	if (error != 0) {
		bus_dma_tag_destroy(sc->fb_dma_tag);
		sc->fb_dma_tag = NULL;
		return (error);
	}

	fb_busaddr = 0;
	error = bus_dmamap_load(sc->fb_dma_tag, sc->fb_dma_map, fb_kva,
	    sc->fb_size, rk_kms_fb_dma_cb, &fb_busaddr, BUS_DMA_WAITOK);
	if (error != 0 || fb_busaddr == 0) {
		bus_dmamem_free(sc->fb_dma_tag, fb_kva, sc->fb_dma_map);
		bus_dma_tag_destroy(sc->fb_dma_tag);
		sc->fb_dma_tag = NULL;
		sc->fb_dma_map = NULL;
		return (error != 0 ? error : ENXIO);
	}

	sc->fb_va = (vm_offset_t)fb_kva;
	sc->fb_pa = (vm_paddr_t)fb_busaddr;

	memset(&sc->fb_info, 0, sizeof(sc->fb_info));
	sc->fb_info.fb_name = device_get_nameunit(sc->dev);
	sc->fb_info.fb_vbase = sc->fb_va;
	sc->fb_info.fb_pbase = sc->fb_pa;
	sc->fb_info.fb_size = sc->fb_size;
	sc->fb_info.fb_bpp = RK_KMS_FB_BPP;
	sc->fb_info.fb_depth = 24;
	sc->fb_info.fb_width = RK_KMS_FB_WIDTH;
	sc->fb_info.fb_height = RK_KMS_FB_HEIGHT;
	sc->fb_info.fb_stride = RK_KMS_FB_STRIDE;
	sc->fb_info.fb_flags = FB_FLAG_MEMATTR;
	sc->fb_info.fb_memattr = VM_MEMATTR_WRITE_COMBINING;
	sc->fb_published = true;
	device_printf(sc->dev,
	    "boot fb: pa=0x%jx va=0x%lx size=%zu stride=%d %dx%d bpp=%d\n",
	    (uintmax_t)sc->fb_pa, (unsigned long)sc->fb_va, sc->fb_size,
	    sc->fb_info.fb_stride, sc->fb_info.fb_width,
	    sc->fb_info.fb_height, sc->fb_info.fb_bpp);
	return (0);
}

static void
rk_kms_fb_free(struct rk_kms_softc *sc)
{
	sc->fb_published = false;
	if (sc->fb_dma_tag != NULL) {
		if (sc->fb_dma_map != NULL && sc->fb_va != 0)
			bus_dmamap_unload(sc->fb_dma_tag, sc->fb_dma_map);
		if (sc->fb_va != 0)
			bus_dmamem_free(sc->fb_dma_tag, (void *)sc->fb_va,
			    sc->fb_dma_map);
		bus_dma_tag_destroy(sc->fb_dma_tag);
	}
	sc->fb_dma_tag = NULL;
	sc->fb_dma_map = NULL;
	sc->fb_va = 0;
	sc->fb_pa = 0;
	sc->fb_size = 0;
}

/*
 * Fill an empty struct drm_display_mode with the canned timing for
 * the requested config target.  Used by the config_{hdmi,dp} entry
 * points so they don't depend on Xorg / SETCRTC having handed us a
 * mode.  HDMI: standard CEA 1920x1080@60 (narrow PHSYNC).  DP: the
 * DMT-style wide-NHSYNC variant the XYM panel accepts.
 */
static void
rk_kms_fill_forced_mode(struct drm_display_mode *mode, int target)
{
	memset(mode, 0, sizeof(*mode));
	if (target == RK_KMS_CONFIG_HDMI) {
		mode->clock = RK_HDMI_FORCED_CLOCK_KHZ;
		mode->hdisplay = RK_HDMI_FORCED_HDISPLAY;
		mode->hsync_start = RK_HDMI_FORCED_HSYNC_START;
		mode->hsync_end = RK_HDMI_FORCED_HSYNC_END;
		mode->htotal = RK_HDMI_FORCED_HTOTAL;
		mode->vdisplay = RK_HDMI_FORCED_VDISPLAY;
		mode->vsync_start = RK_HDMI_FORCED_VSYNC_START;
		mode->vsync_end = RK_HDMI_FORCED_VSYNC_END;
		mode->vtotal = RK_HDMI_FORCED_VTOTAL;
		mode->flags = KMS_MODE_FLAG_PHSYNC |
		    KMS_MODE_FLAG_PVSYNC;
		strlcpy(mode->name, "1920x1080", sizeof(mode->name));
	} else {
		mode->clock = RK_DP_FORCED_CLOCK_KHZ;
		mode->hdisplay = RK_DP_FORCED_HDISPLAY;
		mode->hsync_start = RK_DP_FORCED_HSYNC_START;
		mode->hsync_end = RK_DP_FORCED_HSYNC_END;
		mode->htotal = RK_DP_FORCED_HTOTAL;
		mode->vdisplay = RK_DP_FORCED_VDISPLAY;
		mode->vsync_start = RK_DP_FORCED_VSYNC_START;
		mode->vsync_end = RK_DP_FORCED_VSYNC_END;
		mode->vtotal = RK_DP_FORCED_VTOTAL;
		mode->flags = KMS_MODE_FLAG_NHSYNC |
		    KMS_MODE_FLAG_PVSYNC;
		strlcpy(mode->name, "1920x1080_dp", sizeof(mode->name));
	}
}

/*
 * Self-contained HDMI bring-up.  Sets the legacy knobs to the values
 * the HDMI path needs, runs display_domain_sanity to (re-)arm the
 * power / clock state, then drives the full VOP + DW HDMI
 * controller + PHY sequence via vop_program_timing with the forced
 * CEA 1080p mode.  Skip STAGE_WIN0 because we don't have a fresh
 * framebuffer pa to hand it; whatever VOP was scanning previously
 * keeps scanning out, which is enough to validate PHY lock against a
 * connected sink.
 */
static int
rk_kms_config_hdmi(struct rk_kms_softc *sc)
{
	struct drm_display_mode mode;

	if (!sc->hw_attached) {
		DPRINTF(sc, "config_hdmi: hw not attached\n");
		return (ENXIO);
	}
	rk_kms_fill_forced_mode(&mode, RK_KMS_CONFIG_HDMI);
	sc->output = RK_KMS_OUT_HDMI;
	sc->hdmi_enable = 1;
	sc->dp_enable = 0;
	sc->commit_modeset = RK_KMS_STAGE_ALL &
	    ~RK_KMS_STAGE_WIN0;
	DPRINTF(sc, "config_hdmi: starting (%ux%u@%u)\n",
	    mode.hdisplay, mode.vdisplay, mode.clock);
	rk_kms_display_domain_sanity(sc);
	rk_kms_vop_program_timing(sc, &mode, NULL);
	sc->config_active = RK_KMS_CONFIG_HDMI;
	/*
	 * Drop commit_modeset back to 0 now that our bring-up has
	 * latched.  Userspace SETCRTC / ATOMIC paths use the same gate,
	 * and an unchanged 0x6f would let a subsequent mode=NULL
	 * (Xorg's blank-before-set, slim/Wayland teardown, etc.) write
	 * VOP STANDBY and darken the panel we just lit.  The user can
	 * still flip commit_modeset by hand to opt back into real
	 * userspace modesets once the proper SETCRTC mode flow is wired.
	 */
	sc->commit_modeset = 0;
	DPRINTF(sc, "config_hdmi: done\n");
	return (0);
}

/*
 * Self-contained USB-C DP bring-up.  Mirror image of config_hdmi:
 * sets the legacy knobs to the DP-friendly defaults, then runs the
 * full bring-up against the forced DMT-style 1080p mode.  The
 * STAGE_HDMI_DP stage internally calls rk_kms_dp_modeset,
 * which talks to the rk_cdn_dp module.
 */
static int
rk_kms_config_dp(struct rk_kms_softc *sc)
{
	struct drm_display_mode mode;

	if (!sc->hw_attached) {
		DPRINTF(sc, "config_dp: hw not attached\n");
		return (ENXIO);
	}
	rk_kms_fill_forced_mode(&mode, RK_KMS_CONFIG_DP);
	sc->output = RK_KMS_OUT_DP;
	sc->hdmi_enable = 0;
	sc->dp_enable = 1;
	sc->commit_modeset = RK_KMS_STAGE_ALL &
	    ~RK_KMS_STAGE_WIN0;
	DPRINTF(sc, "config_dp: starting (%ux%u@%u)\n",
	    mode.hdisplay, mode.vdisplay, mode.clock);
	rk_kms_display_domain_sanity(sc);
	rk_kms_vop_program_timing(sc, &mode, NULL);
	sc->config_active = RK_KMS_CONFIG_DP;
	/* See config_hdmi for the commit_modeset reset rationale. */
	sc->commit_modeset = 0;
	DPRINTF(sc, "config_dp: done\n");
	return (0);
}

/*
 * `dev.rk_kms.0.config` sysctl handler.  Write 1 to bring up
 * HDMI, 2 for DP, 0 is currently a no-op (real teardown wiring
 * lands later).  Reads return the active config.
 */
static int
rk_kms_config_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rk_kms_softc *sc = arg1;
	int target = sc->config_active;
	int error;

	error = sysctl_handle_int(oidp, &target, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	sc->config_target = target;
	switch (target) {
	case RK_KMS_CONFIG_HDMI:
		return (rk_kms_config_hdmi(sc));
	case RK_KMS_CONFIG_DP:
		return (rk_kms_config_dp(sc));
	case RK_KMS_CONFIG_NONE:
		/* No teardown path yet; leave whatever was active alone. */
		return (0);
	}
	return (EINVAL);
}

/*
 * Run cdn_dp's full 19-stage bring-up + frame video-active arm.
 * Shared between the sysctl trigger and the altmode-entry poller.
 * Both calls are idempotent; on a link that's already trained this
 * is fast.  Returns the two errno values so callers can log them.
 */
static void
rk_kms_usbc_bringup(struct rk_kms_softc *sc, const char *cause)
{
	int brerr, vaerr;

	brerr = rk_cdn_dp_auto_bringup_default();
	vaerr = rk_cdn_dp_set_video_active_first(true);
	DPRINTF(sc, "%s: auto_bringup=%d video_active=%d\n", cause, brerr,
	    vaerr);
}

/*
 * Trigger cdn_dp's full 19-stage bring-up + frame video-active arm,
 * without going through SETCRTC.  Useful for manual debug; the
 * Phase 11 poller below normally takes care of this automatically.
 *
 * Writing 1 fires the bring-up.  Writing 2 also resets the poller's
 * "already fired against this attach_seq" tracker so the next poll
 * tick re-fires (useful when the bring-up succeeded but the sink
 * dropped HPD and we want to retry without yanking the cable).
 */
static int
rk_kms_usbc_bringup_now_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct rk_kms_softc *sc = arg1;
	int trigger = 0;
	int error;

	error = sysctl_handle_int(oidp, &trigger, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (trigger == 0)
		return (0);
	if (trigger >= 2)
		sc->usbc_attach_seq_done = 0;
	rk_kms_usbc_bringup(sc, "usbc_bringup_now");
	return (0);
}

/*
 * Phase 11 — altmode-entry poller.
 *
 * Runs every 500 ms while the driver is attached.  Reads fusb302's
 * attach_seq and DP altmode state; if a fresh cable attach has gone
 * all the way through DP altmode entry (dp_ready && valid) and we
 * haven't yet fired bring-up for this attach_seq, fire it.
 *
 * Why polling instead of a notify callback: a notify path would need
 * new ABI on either fusb302 or rk_typec_phy and ordering guarantees
 * I'd rather not commit to before getting the polled version working
 * end-to-end.  500 ms is comfortably above any visible-to-user
 * latency once the cable is in.
 */
static void
rk_kms_usbc_poll(void *arg)
{
	struct rk_kms_softc *sc = arg;
	device_t fdev;
	devclass_t fdc;
	struct rk3399_typec_dp_altmode_status alt;
	struct fusb302_typec_status ts;
	uint32_t seq;
	bool now_attached;

	if (!sc->usbc_poll_armed)
		return;
	if (sc->dp_enable == 0 || sc->output != RK_KMS_OUT_DP)
		goto reschedule;

	fdc = devclass_find("fusb302");
	if (fdc == NULL)
		goto reschedule;
	fdev = devclass_get_device(fdc, 0);
	if (fdev == NULL)
		goto reschedule;

	/*
	 * Hotplug detection: fusb302 reports the CC-line attach state.
	 * Compare to the last known state; on any transition, fire
	 * kms_connector_hotplug so open drm_files receive
	 * DRM_EVENT_CONNECTOR_HOTPLUG and devd emits a kms/cardN/hotplug
	 * broadcast (which rc scripts / desktop session managers can hook
	 * for automatic display re-probe on cable insertion/removal).
	 *
	 * Gated by dev.rk_kms.0.hotplug sysctl — default OFF because
	 * kms_connector_hotplug's per-fd event dispatch has raced with
	 * Xorg's atomic probe path and wedged the box on some boots.
	 * Turn on to test hotplug flows manually.
	 */
	if (sc->hotplug_enable != 0 &&
	    fusb302_get_typec_status(fdev, &ts) == 0) {
		now_attached = ts.attached;
		if (now_attached != sc->usbc_last_attached) {
			enum drm_connector_status new_status = now_attached ?
			    connector_status_connected :
			    connector_status_disconnected;
			DPRINTF(sc, "hotplug: attached %d -> %d\n",
			    sc->usbc_last_attached, now_attached);
			kms_connector_hotplug(&sc->connector, new_status);
			sc->usbc_last_attached = now_attached;
			if (!now_attached) {
				/*
				 * On disconnect, forget the last seq so the
				 * next fresh attach re-fires bring-up even
				 * if the fusb302 seq happened to match.
				 */
				sc->usbc_attach_seq_done = 0;
			}
		}
	}

	seq = fusb302_get_attach_seq(fdev);
	if (seq == sc->usbc_attach_seq_done)
		goto reschedule;
	if (fusb302_get_dp_altmode_state(fdev, &alt) != 0)
		goto reschedule;
	if (!alt.valid || !alt.dp_ready)
		goto reschedule;

	DPRINTF(sc, "usbc_poll: altmode ready (seq=%u pin=%u status=0x%x); "
	    "firing bring-up\n", seq, alt.pin_assignment, alt.dp_status);
	rk_kms_usbc_bringup(sc, "usbc_poll");
	sc->usbc_attach_seq_done = seq;

reschedule:
	if (sc->usbc_poll_armed)
		callout_reset(&sc->usbc_poll, hz / 2,
		    rk_kms_usbc_poll, sc);
}

/*
 * Self-rearming vblank ticker.  Each fire advances the framework's
 * per-CRTC sequence counter, wakes WAIT_VBLANK sleepers, and delivers
 * any pending FLIP_COMPLETE event from a PAGE_FLIP_EVENT-armed
 * page-flip.  Re-enqueues itself until vblank_running goes false.
 */
static void
rk_kms_vblank_task(void *arg, int pending __unused)
{
	struct rk_kms_softc *sc = arg;

	if (!sc->vblank_running)
		return;
	kms_vblank_handler(&sc->crtc);
	if (sc->vblank_running && sc->vblank_ticks > 0)
		taskqueue_enqueue_timeout(taskqueue_thread, &sc->vblank_task,
		    sc->vblank_ticks);
}

static void
rk_kms_vblank_start(struct rk_kms_softc *sc,
    const struct drm_display_mode *mode)
{
	uint32_t hz_v;

	if (!sc->vblank_enable || sc->vblank_running)
		return;
	hz_v = kms_mode_vrefresh(mode);
	if (hz_v == 0)
		hz_v = 60;
	sc->vblank_ticks = (int)(hz / hz_v);
	if (sc->vblank_ticks <= 0)
		sc->vblank_ticks = 1;
	sc->vblank_running = true;
	DPRINTF(sc, "vblank ticker start: %u Hz (%d ticks/period)\n",
	    hz_v, sc->vblank_ticks);
	taskqueue_enqueue_timeout(taskqueue_thread, &sc->vblank_task,
	    sc->vblank_ticks);
}

static void
rk_kms_vblank_stop(struct rk_kms_softc *sc)
{
	if (!sc->vblank_running)
		return;
	sc->vblank_running = false;
	taskqueue_cancel_timeout(taskqueue_thread, &sc->vblank_task, NULL);
	taskqueue_drain_timeout(taskqueue_thread, &sc->vblank_task);
	DPRINTF(sc, "vblank ticker stop\n");
}

/*
 * Program VOP WIN2 area 0 as a hardware cursor.  pa == 0 disables
 * the plane; otherwise binds the (w x h) ARGB8888 bitmap at pa and
 * positions it at (x, y) on the output.  Callers must hold enough
 * state that cursor_bo pages stay pinned while VOP DMA is active.
 */
static void
rk_kms_vop_program_cursor(struct rk_kms_softc *sc, vm_paddr_t pa,
    uint32_t w, uint32_t h, int32_t x, int32_t y)
{
	uint32_t stride_words;

	if (pa == 0 || w == 0 || h == 0) {
		vop_big_write(sc, VOP_REG_WIN2_CTRL0, 0);
		return;
	}
	/* Clamp negative on-screen coords to 0; WIN2 DSP_ST0 is
	 * unsigned 13-bit fields.  Off-screen cursor is legal — we
	 * simply won't program a negative position. */
	if (x < 0) x = 0;
	if (y < 0) y = 0;
	stride_words = (w * 4u) / 4u;	/* ARGB8888 → 4B/pixel */

	vop_big_write(sc, VOP_REG_WIN2_VIR0_1, stride_words & 0x1fff);
	vop_big_write(sc, VOP_REG_WIN2_MST0, (uint32_t)pa);
	vop_big_write(sc, VOP_REG_WIN2_DSP_INFO0,
	    (((h - 1u) & 0xfff) << 16) | ((w - 1u) & 0xfff));
	vop_big_write(sc, VOP_REG_WIN2_DSP_ST0,
	    (((uint32_t)y & 0x1fff) << 16) | ((uint32_t)x & 0x1fff));
	/*
	 * Per-pixel alpha blend — without this the transparent regions
	 * of the cursor bitmap render as opaque black instead of showing
	 * the underlying primary fb through.
	 */
	vop_big_write(sc, VOP_REG_WIN2_SRC_ALPHA_CTRL, VOP_WIN2_SRC_ALPHA_STD);
	vop_big_write(sc, VOP_REG_WIN2_DST_ALPHA_CTRL, VOP_WIN2_DST_ALPHA_STD);
	vop_big_write(sc, VOP_REG_WIN2_CTRL0,
	    VOP_WIN2_CTRL0_GATE | VOP_WIN2_CTRL0_FMT_ARGB |
	    VOP_WIN2_CTRL0_ENABLE);
	if (sc->commit_modeset & RK_KMS_STAGE_CFG_DONE)
		vop_big_write(sc, VOP_REG_CFG_DONE, 0x00010001);
}

static int
rk_kms_cursor_set(struct drm_crtc *crtc, struct drm_file *file,
    uint32_t handle, uint32_t width, uint32_t height,
    int32_t hot_x, int32_t hot_y)
{
	struct rk_kms_softc *sc = crtc->dev->driver_priv;
	struct drm_gem_object *obj;
	vm_paddr_t pa;

	if (!sc->hw_attached)
		return (0);
	if (sc->hw_cursor_enable == 0)
		return (ENOTTY);	/* signal Xorg to draw SW cursor */
	if (handle == 0 || width == 0 || height == 0) {
		/* Disable path — release any pinned BO and blank WIN2. */
		if (sc->cursor_bo != NULL) {
			kms_gem_object_put(sc->cursor_bo);
			sc->cursor_bo = NULL;
		}
		sc->cursor_width = 0;
		sc->cursor_height = 0;
		rk_kms_vop_program_cursor(sc, 0, 0, 0, 0, 0);
		return (0);
	}
	obj = kms_gem_handle_lookup(file, handle);
	if (obj == NULL)
		return (ENOENT);
	if (obj->pages == NULL || obj->npages == 0 ||
	    obj->size < (size_t)width * height * 4u) {
		kms_gem_object_put(obj);
		return (EINVAL);
	}
	pa = VM_PAGE_TO_PHYS(obj->pages[0]);
	if (pa > UINT32_MAX) {
		kms_gem_object_put(obj);
		return (ERANGE);
	}
	/* Swap in — release the previous pin (if any) after grabbing the
	 * new ref so a same-handle re-set doesn't briefly drop refs. */
	if (sc->cursor_bo != NULL)
		kms_gem_object_put(sc->cursor_bo);
	sc->cursor_bo = obj;
	sc->cursor_width = width;
	sc->cursor_height = height;
	sc->cursor_hot_x = hot_x;
	sc->cursor_hot_y = hot_y;
	rk_kms_vop_program_cursor(sc, pa, width, height,
	    sc->cursor_x - hot_x, sc->cursor_y - hot_y);
	DPRINTF(sc, "cursor_set: %ux%u pa=0x%jx hot=%d,%d\n",
	    width, height, (uintmax_t)pa, hot_x, hot_y);
	return (0);
}

static int
rk_kms_cursor_move(struct drm_crtc *crtc, int32_t x, int32_t y)
{
	struct rk_kms_softc *sc = crtc->dev->driver_priv;
	vm_paddr_t pa;

	if (!sc->hw_attached)
		return (0);
	if (sc->hw_cursor_enable == 0)
		return (ENOTTY);	/* signal Xorg to draw SW cursor */
	sc->cursor_x = x;
	sc->cursor_y = y;
	if (sc->cursor_bo == NULL || sc->cursor_width == 0)
		return (0);
	pa = VM_PAGE_TO_PHYS(sc->cursor_bo->pages[0]);
	rk_kms_vop_program_cursor(sc, pa,
	    sc->cursor_width, sc->cursor_height,
	    x - sc->cursor_hot_x, y - sc->cursor_hot_y);
	return (0);
}

/*
 * Substitute the native (rk_kms_mode_table[0]) mode for any non-native
 * pick.  Shared by set_config, page_flip, and atomic_commit — keeps
 * the DP link at its trained pixel clock; the WIN0 scaler in
 * vop_program_win0 upscales the fb.  Caller passes storage for the
 * synthesized native mode; return value is either that storage or the
 * incoming mode pointer.
 */
static const struct drm_display_mode *
rk_kms_outer_mode(const struct drm_display_mode *in,
    struct drm_display_mode *scratch)
{
	const struct rk_kms_advertised_mode *n = &rk_kms_mode_table[0];

	if (in->hdisplay == n->hdisplay && in->vdisplay == n->vdisplay)
		return (in);
	memset(scratch, 0, sizeof(*scratch));
	scratch->clock = n->clock;
	scratch->hdisplay = n->hdisplay;
	scratch->hsync_start = n->hsync_start;
	scratch->hsync_end = n->hsync_end;
	scratch->htotal = n->htotal;
	scratch->vdisplay = n->vdisplay;
	scratch->vsync_start = n->vsync_start;
	scratch->vsync_end = n->vsync_end;
	scratch->vtotal = n->vtotal;
	scratch->flags = n->flags;
	return (scratch);
}

static int
rk_kms_set_config(struct drm_mode_set *set)
{
	struct rk_kms_softc *sc;
	struct drm_display_mode dst_mode;
	const struct drm_display_mode *outer;

	sc = set->crtc->dev->driver_priv;
	if (set->mode != NULL) {
		DPRINTF(sc, "set_config: %ux%u clock=%u fb=%u commit=%d\n",
		    set->mode->hdisplay, set->mode->vdisplay,
		    set->mode->clock,
		    set->fb != NULL ? set->fb->base.id : 0,
		    sc->commit_modeset);
		/*
		 * See rk_kms_outer_mode() — pin DSP timing to trained
		 * native to survive panel-quirk MSA changes; WIN0 scaler
		 * handles fb upscale for non-native modes.
		 */
		outer = rk_kms_outer_mode(set->mode, &dst_mode);
		if (sc->commit_modeset != 0 && sc->hw_attached)
			rk_kms_vop_program_timing(sc, outer, set->fb);
		rk_kms_vblank_start(sc, set->mode);
	} else {
		DPRINTF(sc, "set_config: blank (commit=%d)\n",
		    sc->commit_modeset);
		rk_kms_vblank_stop(sc);
		if ((sc->commit_modeset &
		    (RK_KMS_STAGE_VOP_SYS | RK_KMS_STAGE_CFG_DONE))
		    != 0 && sc->hw_attached) {
			uint32_t sys_ctrl;

			DPRINTF(sc, "STAGE blank STARTING\n");
			sys_ctrl = vop_big_read(sc, VOP_REG_SYS_CTRL);
			sys_ctrl |= VOP_SYS_CTRL_STANDBY;
			vop_big_write(sc, VOP_REG_SYS_CTRL, sys_ctrl);
			vop_big_write(sc, VOP_REG_CFG_DONE, 0x00010001);
			DPRINTF(sc, "STAGE blank DONE\n");
		}
	}
	return (0);
}

/*
 * PAGE_FLIP: Xorg's modesetting driver may swap between front/back
 * scanout buffers via PAGE_FLIP instead of full SETCRTC.  Without a
 * driver hook the framework silently updates crtc->primary_fb but
 * leaves VOP WIN0_YRGB_MST pointing at the original SETCRTC fb — VOP
 * keeps scanning the stale buffer while Xorg renders to the new one.
 * Re-run WIN0 program + CFG_DONE latch so the new fb pa lands on the
 * hardware for the next scan.
 */
static int
rk_kms_page_flip(struct drm_crtc *crtc, struct drm_framebuffer *fb,
    uint32_t flags __unused, uint64_t user_data __unused)
{
	struct rk_kms_softc *sc = crtc->dev->driver_priv;
	struct drm_display_mode scratch;
	const struct drm_display_mode *outer;

	if (!sc->hw_attached || fb == NULL)
		return (0);
	if ((sc->commit_modeset & RK_KMS_STAGE_WIN0) == 0)
		return (0);
	outer = rk_kms_outer_mode(&crtc->mode, &scratch);
	DPRINTF(sc, "page_flip: fb=%u src=%ux%u dst=%ux%u\n",
	    fb->base.id, fb->width, fb->height,
	    outer->hdisplay, outer->vdisplay);
	rk_kms_vop_program_win0(sc, outer, fb,
	    rk_kms_hact_start(outer),
	    rk_kms_vact_start(outer));
	if (sc->commit_modeset & RK_KMS_STAGE_CFG_DONE)
		vop_big_write(sc, VOP_REG_CFG_DONE, 0x00010001);
	return (0);
}

static const struct drm_crtc_funcs rk_kms_crtc_funcs = {
	.set_config = rk_kms_set_config,
	.page_flip = rk_kms_page_flip,
	.cursor_set = rk_kms_cursor_set,
	.cursor_move = rk_kms_cursor_move,
};
static const struct drm_plane_funcs rk_kms_plane_funcs = { 0 };

/*
 * Atomic-modeset hooks.  The framework hands us a fully populated
 * drm_atomic_state (Phase 8 step 2 of the kms framework wires the
 * property -> state-field resolver), so check + commit just walk the
 * per-object state slots.
 *
 * Phase 1 scope:
 *   atomic_check  — minimal validation; reject obviously-broken inputs
 *                   (mode with zero hdisplay / zero clock, plane fb
 *                   without a GEM backing).  Returns 0 otherwise.
 *                   Does not touch hardware.
 *   atomic_commit — when commit_modeset (the legacy debug gate) is
 *                   non-zero AND hw_attached, walks every crtc_state
 *                   with mode_changed set and drives
 *                   rk_kms_vop_program_timing() with the requested
 *                   mode + the framebuffer from the first
 *                   plane_state routed to that CRTC.  When
 *                   commit_modeset is zero the commit is a no-op
 *                   apart from a DPRINTF — same shape as the legacy
 *                   set_config path so existing operator workflows
 *                   keep working through the atomic ioctl.
 *
 * The driver does not stash the state pointer; the framework owns
 * the lifetime and frees on return.  Phase 8b will introduce state
 * swap, at which point the driver can stash for deferred completion.
 */
static int
rk_kms_atomic_check(struct drm_device *dev __unused,
    struct drm_atomic_state *state)
{
	uint32_t i;

	if (state == NULL)
		return (EINVAL);
	for (i = 0; i < state->num_crtc; i++) {
		const struct drm_crtc_state *cs = state->crtc_states[i];

		if (cs == NULL)
			continue;
		/*
		 * Validate the mode only when the CRTC is being driven on
		 * with a real timing.  active=false (blank) or
		 * mode_changed=false (only ACTIVE flipped, or only a plane
		 * routing change) both bypass the dimension check.
		 */
		if (!cs->mode_changed || !cs->active)
			continue;
		if (cs->mode.hdisplay == 0 || cs->mode.vdisplay == 0 ||
		    cs->mode.clock == 0)
			return (EINVAL);
	}
	for (i = 0; i < state->num_plane; i++) {
		const struct drm_plane_state *ps = state->plane_states[i];

		if (ps == NULL || ps->fb == NULL)
			continue;
		if (ps->fb->gem_objs[0] == NULL)
			return (EINVAL);
	}
	return (0);
}

/*
 * Pick the framebuffer routed to a particular CRTC out of the atomic
 * state.  We have exactly one primary plane per CRTC in Phase 1, so
 * the first plane_state whose crtc field matches wins.  Returns NULL
 * if no plane is routed (e.g. blanking transition).
 */
static struct drm_framebuffer *
rk_kms_atomic_pick_fb(struct drm_atomic_state *state, struct drm_crtc *crtc)
{
	uint32_t i;

	for (i = 0; i < state->num_plane; i++) {
		struct drm_plane_state *ps = state->plane_states[i];

		if (ps != NULL && ps->crtc == crtc && ps->fb != NULL)
			return (ps->fb);
	}
	return (NULL);
}

static int
rk_kms_atomic_commit(struct drm_device *dev,
    struct drm_atomic_state *state, bool nonblock __unused)
{
	struct rk_kms_softc *sc;
	uint32_t i;
	struct drm_display_mode scratch;
	const struct drm_display_mode *outer;

	if (state == NULL)
		return (EINVAL);
	sc = dev->driver_priv;

	for (i = 0; i < state->num_crtc; i++) {
		struct drm_crtc_state *cs = state->crtc_states[i];
		struct drm_framebuffer *fb;

		if (cs == NULL)
			continue;
		fb = rk_kms_atomic_pick_fb(state, cs->crtc);
		DPRINTF(sc, "atomic_commit: crtc=%u mode=%ux%u@%u "
		    "active=%d fb=%u mode_changed=%d planes_changed=%d "
		    "commit=%d\n",
		    cs->crtc->base.id, cs->mode.hdisplay, cs->mode.vdisplay,
		    cs->mode.clock, cs->active,
		    fb != NULL ? fb->base.id : 0,
		    cs->mode_changed, cs->planes_changed, sc->commit_modeset);
		if (!cs->mode_changed && !cs->planes_changed &&
		    !cs->connectors_changed)
			continue;
		if (sc->commit_modeset == 0 || !sc->hw_attached)
			continue;
		if (cs->active && cs->mode.hdisplay > 0) {
			outer = rk_kms_outer_mode(&cs->mode, &scratch);
			if (cs->mode_changed) {
				/*
				 * Full modeset: program VOP timing (which
				 * in turn programs WIN0 + finishes cdn_dp
				 * + latches CFG_DONE).
				 */
				rk_kms_vop_program_timing(sc, outer, fb);
			} else if (cs->planes_changed && fb != NULL) {
				/*
				 * Plane-only atomic commit (compositor
				 * front/back flip): reprogram WIN0 with the
				 * new fb, then latch CFG_DONE so VOP picks
				 * up the new source on the next scan.
				 */
				rk_kms_vop_program_win0(sc, outer, fb,
				    rk_kms_hact_start(outer),
				    rk_kms_vact_start(outer));
				if (sc->commit_modeset & RK_KMS_STAGE_CFG_DONE)
					vop_big_write(sc, VOP_REG_CFG_DONE,
					    0x00010001);
			}
			/*
			 * Keep crtc->primary_fb in sync so subsequent
			 * GETCRTC + our fb_flush callout see the current
			 * scan-out buffer.  Refcount is managed by the
			 * atomic core when it swaps state.
			 */
			if (fb != NULL)
				cs->crtc->primary_fb = fb;
		}
	}
	return (0);
}

static const struct drm_mode_config_funcs rk_kms_mode_config_funcs = {
	.atomic_check  = rk_kms_atomic_check,
	.atomic_commit = rk_kms_atomic_commit,
};
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
/*
 * Bring PD_VO up and ungate the VOP clock tree so any VOP MMIO access
 * actually returns instead of AXI-hanging the CPU.  u-boot leaves the
 * display block in whatever state suited its splash logic — usually on
 * for the framebuffer console, but not guaranteed.  Mirrors the order
 * rk_drm_display_domain_sanity() uses on the in-tree driver:
 *
 *   1. If PD_VO is currently down (PWRDN_ST bit set), clear it in
 *      PWRDN_CON and poll ST until the domain reports up (10us × 1000).
 *   2. Clear VOPB+VOPL idle requests in BUS_IDLE_REQ so the AXI bridge
 *      starts forwarding transactions to them.
 *   3. Set GATEDIS_VOPB in PMUCRU so the always-on side of VOPB's clock
 *      survives even when nothing else is driving it.
 *   4. Hiword-ungate VOP0 + VOPB gates in CRU_CLKGATE_CON10 / CON28.
 *
 * Safe to run when everything's already on — every write is idempotent.
 */
static void
rk_kms_display_domain_sanity(struct rk_kms_softc *sc)
{
	uint32_t pwrdn_con, pwrdn_st, idle_req, gatedis0;
	int i;

	pwrdn_con = pmu_read(sc, PMU_PWRDN_CON);
	pwrdn_st = pmu_read(sc, PMU_PWRDN_ST);
	idle_req = pmu_read(sc, PMU_BUS_IDLE_REQ);
	gatedis0 = pmucru_read(sc, PMUCRU_GATEDIS_CON0);
	DPRINTF(sc, "PD sanity in: pwrdn_con=0x%x st=0x%x idle=0x%x "
	    "gatedis0=0x%x\n", pwrdn_con, pwrdn_st, idle_req, gatedis0);

	if ((pwrdn_st & PMU_PD_VO) != 0) {
		pmu_write(sc, PMU_PWRDN_CON, pwrdn_con & ~PMU_PD_VO);
		for (i = 0; i < 1000; i++) {
			pwrdn_st = pmu_read(sc, PMU_PWRDN_ST);
			if ((pwrdn_st & PMU_PD_VO) == 0)
				break;
			DELAY(10);
		}
		DPRINTF(sc, "PD sanity: PD_VO powered up after %d poll(s)\n",
		    i);
	}

	if ((idle_req & (PMU_IDLE_VOPB | PMU_IDLE_VOPL)) != 0)
		pmu_write(sc, PMU_BUS_IDLE_REQ,
		    idle_req & ~(PMU_IDLE_VOPB | PMU_IDLE_VOPL));

	if ((gatedis0 & PMUCRU_GATEDIS_VOPB) == 0)
		pmucru_write(sc, PMUCRU_GATEDIS_CON0,
		    gatedis0 | PMUCRU_GATEDIS_VOPB);

	cru_write(sc, CRU_CLKGATE_CON10, (CRU_CLKGATE_VOP0_MASK << 16));
	cru_write(sc, CRU_CLKGATE_CON28, (CRU_CLKGATE_VOPB_MASK << 16));
}

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
	rk_kms_display_domain_sanity(sc);
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
	TIMEOUT_TASK_INIT(taskqueue_thread, &sc->vblank_task, 0,
	    rk_kms_vblank_task, sc);

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

	/*
	 * Install the atomic hooks before any topology object gets a
	 * chance to be reached by an ATOMIC ioctl.  Setting funcs here
	 * promotes ATOMIC from the framework's legacy property-table
	 * fallback (which can't actually drive HW) to the real
	 * check + commit dispatch.  Operator-facing modeset gates
	 * (commit_modeset etc.) still apply inside the commit hook.
	 */
	sc->drm_dev->mode_config.funcs = &rk_kms_mode_config_funcs;

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
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "hdmi_enable", CTLFLAG_RW, &sc->hdmi_enable, 0,
	    "Enable DW HDMI PHY bring-up on set_config "
	    "(Phase 9f part 1: PHY only)");
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "vblank_enable", CTLFLAG_RW, &sc->vblank_enable, 0,
	    "Run a software vblank ticker at the active mode's "
	    "refresh rate (Phase 9g part 2)");
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "dp_enable", CTLFLAG_RW, &sc->dp_enable, 0,
	    "Drive Cadence MHDP DP TX through set_config "
	    "(Phase 9h: requires output=1)");
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "vop_dclk_reset", CTLFLAG_RW, &sc->vop_dclk_reset, 0,
	    "Pulse CRU_SOFTRST_CON17 DCLK_VOP0 reset inside vop_timing "
	    "stage (default off; rk_drm only pulses on first scanout)");
	sc->dp_force_mode = 1;
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "dp_force_mode", CTLFLAG_RW, &sc->dp_force_mode, 0,
	    "Replace CEA-VIC 1080p60 with DMT-style wide-NHSYNC timing the "
	    "XYM panel accepts (default on)");
	sc->hdmi_skip_lock_check = 1;
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "hdmi_skip_lock_check", CTLFLAG_RW,
	    &sc->hdmi_skip_lock_check, 0,
	    "Treat the DW HDMI PHY_STAT0[0] lock-bit timeout as non-fatal "
	    "(default on — Innosilicon PHY doesn't assert that bit even "
	    "when the link is up)");
	sc->cache_flush_fb = 1;
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "cache_flush_fb", CTLFLAG_RW, &sc->cache_flush_fb, 0,
	    "DC CVAC the fb pages before each VOP DMA program "
	    "(default on — VOP is non-coherent on RK3399)");
	sc->hotplug_enable = 0;
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "hotplug", CTLFLAG_RW, &sc->hotplug_enable, 0,
	    "Enable DP hotplug event dispatch (default off — "
	    "kms_connector_hotplug's per-fd events have wedged Xorg's "
	    "atomic probe path on boot)");
	sc->hw_cursor_enable = 0;
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "hw_cursor", CTLFLAG_RW, &sc->hw_cursor_enable, 0,
	    "Enable HW cursor plane (VOP WIN2, default off — DP display "
	    "drops signal on the XYM W156F1 panel with WIN2 alpha writes)");
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "usbc_bringup_now",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE, sc, 0,
	    rk_kms_usbc_bringup_now_sysctl, "I",
	    "Write 1 to run rk_cdn_dp_auto_bringup_default + "
	    "set_video_active_first(true) without going through "
	    "SETCRTC.  Idempotent.  Write 2 to also reset the poller "
	    "tracker so the next tick re-fires.");
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "config",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    rk_kms_config_sysctl, "I",
	    "Active output configuration: 0=none, 1=hdmi, 2=dp.  Writing "
	    "a value runs that target's full self-contained bring-up "
	    "(display_domain_sanity + VOP + framer + PHY).  Writes are "
	    "idempotent: re-writing the active config re-runs its "
	    "bring-up sequence.  Switching to another config overwrites "
	    "the previous side's state as a side-effect of the new "
	    "side's comprehensive write sequence.");

	/*
	 * Arm the Phase 11 altmode-entry poller.  No work happens
	 * until dp_enable=1 + output=1, but we run the tick
	 * unconditionally so toggling either sysctl post-boot starts
	 * the bring-up on the next half-second tick.
	 */
	callout_init(&sc->usbc_poll, 1);
	sc->usbc_poll_armed = true;
	callout_reset(&sc->usbc_poll, hz / 2, rk_kms_usbc_poll, sc);

	/*
	 * Phase 12 — boot fb + direct vt_fb_attach.  Skipping the
	 * fbd_driver middleman because fbd is Giant-locked and tagged
	 * for removal by FreeBSD 16.0; calling vt_fb_attach with our
	 * fb_info directly does the same vt registration without the
	 * deprecated fbd layer.  Non-fatal on failure: card0 + DP still
	 * work for headless / DRM-direct clients, we just lose
	 * /dev/ttyv* (so Xorg can't grab a VT).
	 */
	if (rk_kms_fb_alloc(sc) != 0) {
		device_printf(dev,
		    "boot fb alloc failed; vt bridge skipped\n");
	} else {
		int verr = vt_fb_attach(&sc->fb_info);
		if (verr != 0)
			device_printf(dev,
			    "vt_fb_attach failed: %d (no /dev/ttyv*)\n", verr);
		else
			sc->vt_fb_attached = true;
	}

	device_printf(dev, "registered (Phase 9c: VOP code wired behind "
	    "commit_modeset sysctl, default off)\n");
	return (0);
}

static int
rk_kms_detach(device_t dev)
{
	struct rk_kms_softc *sc;

	sc = device_get_softc(dev);
	/*
	 * vt(4)'s flush callout reaches into fb_info->fb_vbase from a
	 * softclock thread on its own schedule.  vt_fb_detach() does not
	 * synchronously drain that callout, so freeing the bus_dma-backed
	 * framebuffer here races a pending vt_fb_bitblt_bitmap() write
	 * and panics with a data abort on a stale VA.  Once vt is bound
	 * to our fb, the only safe way to swap the module is a reboot.
	 * Refuse devctl detach / kldunload to make the rule enforceable;
	 * shutdown is fine because the system is already going down.
	 */
	if (sc->vt_fb_attached) {
		device_printf(dev, "detach refused: vt_fb is live; reboot to "
		    "swap rk_kms\n");
		return (EBUSY);
	}
	sc->usbc_poll_armed = false;
	callout_drain(&sc->usbc_poll);
	if (sc->cursor_bo != NULL) {
		kms_gem_object_put(sc->cursor_bo);
		sc->cursor_bo = NULL;
	}
	rk_kms_fb_free(sc);
	rk_kms_vblank_stop(sc);
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
/*
 * The rockchip,display-subsystem node lives at the device-tree root
 * on the upstream RK3399 DTS, so it's enumerated directly by ofwbus.
 * Match how the in-tree rk_drm does it (DRIVER_MODULE on both buses)
 * so the driver attaches regardless of where the node hangs.
 */
DRIVER_MODULE(rk_kms, ofwbus, rk_kms_driver_kdrv, 0, 0);
MODULE_VERSION(rk_kms, 1);
MODULE_DEPEND(rk_kms, kms, 1, 1, 1);
/*
 * Phase 9h: hard dependency on rk_cdn_dp so the DP-side externs
 * (auto_bringup_default, enable_mode, set_video_active_first)
 * resolve at load time.  Matches the in-tree rkdev kernel where
 * rk_cdn_dp is statically linked; on a stripped kernel without it,
 * kldload rk_kms would (correctly) fail with "missing symbol"
 * rather than crash later from a NULL function pointer.
 */
MODULE_DEPEND(rk_kms, rk_cdn_dp, 1, 1, 1);
/*
 * fusb302 supplies the altmode-entry signals (attach_seq + DP altmode
 * status) that the Phase 11 poller reads.  Declared MODULE_DEPEND to
 * fail kldload cleanly if someone runs on a kernel where fusb302 was
 * removed instead of crashing on first call.
 */
MODULE_DEPEND(rk_kms, fusb302, 1, 2, 2);
