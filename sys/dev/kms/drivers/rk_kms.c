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
 * Phase 9b set_config: log the proposed modeline + framebuffer so
 * userspace verification (drm_probe / xrandr) shows the framework
 * accepted the request.  No register writes yet — Phase 9c lifts
 * vop_init_mode / vop_enable_overlay from rk_drm into a real
 * modeset.  Storing nothing on the CRTC is intentional: the framework
 * already records mode_valid / x / y / primary_fb after this returns.
 */
static int
rk_kms_set_config(struct drm_mode_set *set)
{
	struct rk_kms_softc *sc;

	sc = set->crtc->dev->driver_priv;
	if (set->mode != NULL) {
		device_printf(sc->dev,
		    "set_config: %ux%u clock=%u fb=%u (Phase 9b: log only)\n",
		    set->mode->hdisplay, set->mode->vdisplay,
		    set->mode->clock,
		    set->fb != NULL ? set->fb->base.id : 0);
	} else {
		device_printf(sc->dev,
		    "set_config: blank (Phase 9b: log only)\n");
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

	device_printf(dev, "registered (Phase 9b: MMIO mapped, set_config "
	    "logs only — no VOP writes yet)\n");
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
