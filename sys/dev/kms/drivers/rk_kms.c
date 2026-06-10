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
#include "fb_if.h"

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
#include <kms/drm_vblank.h>

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
#define	HDMI_PHY_I2CM_DIV_DEFAULT  0x0a
#define	HDMI_PHY_I2CM_SS_HCNT0_DEFAULT 0x4f
#define	HDMI_PHY_I2CM_SS_LCNT0_DEFAULT 0x91
#define	HDMI_PHY_I2CM_FS_HCNT0_DEFAULT 0x0f
#define	HDMI_PHY_I2CM_FS_LCNT0_DEFAULT 0x21
#define	HDMI_PHY_I2CM_SDA_HOLD_DEFAULT 0x08
#define	HDMI_BASE_SFRDIVLOW_DEFAULT   0xff
#define	HDMI_BASE_SFRDIVHIGH_DEFAULT  0x00
#define	HDMI_PHY_JTAG_CFG_I2C	0x80
#define	HDMI_PHY_MSM_CTRL_FB_CLK 0x0006
#define	HDMI_PHY_I2C_CKCALCTRL_OVERRIDE 0x0000

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
	uint8_t err, sticky;
	int t;

	if (rk_kms_hdmi_phy_i2c_reset(sc) != 0)
		return (ETIMEDOUT);

	hdmi_write1(sc, HDMI_IH_I2CMPHY_STAT0, 0x03);
	hdmi_write1(sc, HDMI_PHY_I2CM_SLAVE, HDMI_PHY_I2C_ADDR);
	hdmi_write1(sc, HDMI_PHY_I2CM_ADDRESS, reg);
	hdmi_write1(sc, HDMI_PHY_I2CM_DATAO_1, (val >> 8) & 0xff);
	hdmi_write1(sc, HDMI_PHY_I2CM_DATAO_0, val & 0xff);
	hdmi_write1(sc, HDMI_PHY_I2CM_OPERATION, 0x10);

	for (t = 200; t > 0; t--) {
		DELAY(1000);
		err = hdmi_read1(sc, HDMI_PHY_I2CM_CTLINT);
		sticky = hdmi_read1(sc, HDMI_IH_I2CMPHY_STAT0);
		if ((sticky & 0x01) != 0)
			return (0);
		if ((err & 0x10) != 0 || (err & 0x01) != 0) {
			DPRINTF(sc, "phy i2c write reg=0x%02x val=0x%04x "
			    "err=0x%02x sticky=0x%02x\n", reg, val, err,
			    sticky);
			return (EIO);
		}
	}
	return (ETIMEDOUT);
}

/*
 * Toggle the DW HDMI main reset bits.  Used to force the TMDS + pixel
 * domains to re-init after a PHY swap.
 */
static void
rk_kms_hdmi_toggle_main_reset(struct rk_kms_softc *sc, uint8_t bits)
{
	hdmi_write1(sc, HDMI_MC_SWRSTZREQ, (uint8_t)~bits);
	DELAY(100);
	hdmi_write1(sc, HDMI_MC_SWRSTZREQ, 0xff);
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

	/*
	 * HDMI bring-up — PHY only in 9f part 1.  TMDS framer + AVI
	 * infoframes land in 9f part 2; until then the panel won't see
	 * a clean signal even with hdmi_enable=1, but the PHY locking
	 * to the requested clock is itself the diagnostic value.
	 */
	if (sc->hdmi_enable != 0 && sc->output == RK_KMS_OUT_HDMI) {
		error = rk_kms_hdmi_phy_init(sc, mode);
		if (error != 0) {
			DPRINTF(sc, "HDMI PHY init failed: %d\n", error);
		} else {
			rk_kms_hdmi_enable(sc, mode);
		}
	}

	if (sc->dp_enable != 0 && sc->output == RK_KMS_OUT_DP)
		(void)rk_kms_dp_modeset(sc, mode);

	/* Shadow-register commit.  Same value-of-1 the rk_drm reference
	 * uses to latch the timing block in one shot. */
	vop_big_write(sc, VOP_REG_CFG_DONE, 0x00010001);
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

static struct fb_info *
rk_kms_fb_getinfo(device_t dev)
{
	struct rk_kms_softc *sc = device_get_softc(dev);

	if (!sc->fb_published)
		return (NULL);
	return (&sc->fb_info);
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
	device_printf(sc->dev,
	    "%s: auto_bringup=%d video_active=%d\n", cause, brerr, vaerr);
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
	uint32_t seq;

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

	seq = fusb302_get_attach_seq(fdev);
	if (seq == sc->usbc_attach_seq_done)
		goto reschedule;
	if (fusb302_get_dp_altmode_state(fdev, &alt) != 0)
		goto reschedule;
	if (!alt.valid || !alt.dp_ready)
		goto reschedule;

	device_printf(sc->dev,
	    "usbc_poll: altmode ready (seq=%u pin=%u status=0x%x); firing "
	    "bring-up\n", seq, alt.pin_assignment, alt.dp_status);
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
		rk_kms_vblank_start(sc, set->mode);
	} else {
		DPRINTF(sc, "set_config: blank (commit=%d)\n",
		    sc->commit_modeset);
		rk_kms_vblank_stop(sc);
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
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "usbc_bringup_now",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE, sc, 0,
	    rk_kms_usbc_bringup_now_sysctl, "I",
	    "Write 1 to run rk_cdn_dp_auto_bringup_default + "
	    "set_video_active_first(true) without going through "
	    "SETCRTC.  Idempotent.  Write 2 to also reset the poller "
	    "tracker so the next tick re-fires.");

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
	 * Phase 12 — boot fb + fbd child so vt has something to
	 * register against.  Non-fatal if allocation fails; we just
	 * lose /dev/ttyv* and Xorg can't start, but card0 / DP path
	 * still work for headless / DRM-direct clients.
	 */
	if (rk_kms_fb_alloc(sc) != 0) {
		device_printf(dev,
		    "boot fb alloc failed; fbd / vt bridge skipped\n");
	} else {
		device_t fbdev = device_add_child(dev, "fbd",
		    device_get_unit(dev));
		if (fbdev == NULL)
			device_printf(dev, "fbd child add failed\n");
		else
			bus_attach_children(dev);
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
	sc->usbc_poll_armed = false;
	callout_drain(&sc->usbc_poll);
	bus_generic_detach(dev);
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
	DEVMETHOD(fb_getinfo,		rk_kms_fb_getinfo),
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

/*
 * Phase 12 — attach fbd as our child so it can call our fb_getinfo
 * DEVMETHOD, register with vt_fb, and publish /dev/fb0 + /dev/ttyv*.
 */
extern driver_t fbd_driver;
DRIVER_MODULE(fbd, rk_kms, fbd_driver, 0, 0);
