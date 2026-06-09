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
#include <kms/drm_mode_config.h>
#include <kms/drm_modes.h>
#include <kms/drm_plane.h>

#define	RK_KMS_DESC	"Rockchip RK3399 display (kms)"

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
	bool				 hw_attached;

	/*
	 * Sysctl gate for actual VOP register programming.  Defaults to
	 * 0 (log-only) so the .ko is safe to kldload on a system whose
	 * display is already live under rk_drm or vt.  Set
	 * dev.rk_kms.0.commit_modeset=1 to enable real writes.
	 */
	int				 commit_modeset;
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
    const struct drm_display_mode *mode)
{
	uint32_t hact_start = rk_kms_hact_start(mode);
	uint32_t vact_start = rk_kms_vact_start(mode);
	uint32_t hsync_len = mode->hsync_end - mode->hsync_start;
	uint32_t vsync_len = mode->vsync_end - mode->vsync_start;
	uint32_t sys_ctrl;

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
		device_printf(sc->dev,
		    "set_config: %ux%u clock=%u fb=%u commit=%d\n",
		    set->mode->hdisplay, set->mode->vdisplay,
		    set->mode->clock,
		    set->fb != NULL ? set->fb->base.id : 0,
		    sc->commit_modeset);
		if (sc->commit_modeset != 0 && sc->hw_attached)
			rk_kms_vop_program_timing(sc, set->mode);
	} else {
		device_printf(sc->dev, "set_config: blank (commit=%d)\n",
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

	sc->hw_attached = true;
	device_printf(sc->dev,
	    "MMIO mapped: VOP_BIG/LIT @ 0xff9{0,8f}0000 GRF @ 0xff770000 "
	    "CRU @ 0xff760000\n");
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
