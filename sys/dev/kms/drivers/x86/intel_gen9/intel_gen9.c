/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Skeleton Intel Gen9 (Skylake / Kabylake / Coffee Lake) iGPU
 * consumer for the kms framework.
 *
 * Scope of this initial scaffold:
 *   - PCI probe + match against the gen9 ID table
 *   - bus_alloc_resource on BAR0 (GTTMMADR) + BAR2 (GMADR)
 *   - kms_dev_register so /dev/dri/cardN comes up
 *   - empty mode_config_funcs and a single fake CRTC/encoder/connector
 *     just to exercise the GETRESOURCES round-trip on x86
 *
 * What this is NOT:
 *   - a real modesetter.  Display engine programming (DDI / pipe / plane /
 *     transcoder setup), GMBus / AUX, EDID, panel power sequencer, watermarks,
 *     and all the Intel "lots of state in lots of registers" work lands in
 *     follow-up commits.  The point of this first commit is to prove the
 *     kms framework attaches cleanly on amd64 to real PCI hardware.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sx.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>

#include <kms/drm_atomic.h>
#include <kms/drm_connector.h>
#include <kms/drm_crtc.h>
#include <kms/drm_device.h>
#include <kms/drm_drv.h>
#include <kms/drm_encoder.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_modes.h>
#include <kms/drm_plane.h>

#define	INTEL_VENDOR_ID		0x8086

/*
 * Gen9 PCI IDs.  Sourced from Intel's "graphics-pciids" header; this is a
 * representative subset that covers the bulk of Skylake-S / -H / -U laptops
 * and desktops 2015-2018 plus Kabylake / Coffee Lake refreshes (which share
 * the gen9 / gen9.5 display engine).  Add more here as they're observed
 * in the wild.
 */
static const struct {
	uint16_t	id;
	const char	*desc;
} intel_gen9_ids[] = {
	{ 0x1902, "Intel HD 510 (Skylake GT1)" },
	{ 0x1906, "Intel HD 510 (Skylake GT1 ULT)" },
	{ 0x190b, "Intel HD 510 (Skylake GT1 Halo)" },
	{ 0x190e, "Intel HD 510 (Skylake GT1 Mobile)" },
	{ 0x1912, "Intel HD 530 (Skylake GT2)" },
	{ 0x1916, "Intel HD 520 (Skylake GT2 ULT)" },
	{ 0x191b, "Intel HD 530 (Skylake GT2 Halo)" },
	{ 0x191d, "Intel HD P530 (Skylake GT2 Workstation)" },
	{ 0x191e, "Intel HD 515 (Skylake GT2 Mobile)" },
	{ 0x1921, "Intel HD 520 (Skylake GT2F)" },
	{ 0x1923, "Intel Iris 540 (Skylake GT3)" },
	{ 0x1926, "Intel Iris 540/550 (Skylake GT3e)" },
	{ 0x1927, "Intel Iris 550 (Skylake GT3e)" },
	{ 0x192b, "Intel Iris Pro 580 (Skylake GT4e)" },
	{ 0x5902, "Intel HD 610 (Kabylake GT1)" },
	{ 0x5906, "Intel HD 610 (Kabylake GT1 ULT)" },
	{ 0x590b, "Intel HD 610 (Kabylake GT1 Halo)" },
	{ 0x590e, "Intel HD 610 (Kabylake GT1 Mobile)" },
	{ 0x5912, "Intel HD 630 (Kabylake GT2)" },
	{ 0x5916, "Intel HD 620 (Kabylake GT2 ULT)" },
	{ 0x591a, "Intel HD P630 (Kabylake GT2 Mobile WS)" },
	{ 0x591b, "Intel HD 630 (Kabylake GT2 Halo)" },
	{ 0x591d, "Intel HD P630 (Kabylake GT2 Workstation)" },
	{ 0x591e, "Intel HD 615 (Kabylake GT2 Mobile)" },
	{ 0x5921, "Intel HD 620 (Kabylake GT2F)" },
	{ 0x5923, "Intel Iris Plus 640 (Kabylake GT3)" },
	{ 0x5926, "Intel Iris Plus 640 (Kabylake GT3e)" },
	{ 0x5927, "Intel Iris Plus 650 (Kabylake GT3e)" },
	{ 0x3e90, "Intel UHD 610 (Coffee Lake GT1)" },
	{ 0x3e91, "Intel UHD 630 (Coffee Lake GT2)" },
	{ 0x3e92, "Intel UHD 630 (Coffee Lake GT2)" },
	{ 0x3e93, "Intel UHD 610 (Coffee Lake GT1)" },
	{ 0x3e96, "Intel UHD P630 (Coffee Lake GT2 Workstation)" },
	{ 0x3e98, "Intel UHD 630 (Coffee Lake GT2)" },
	{ 0x3e9b, "Intel UHD 630 (Coffee Lake GT2 Halo)" },
	{ 0x3ea5, "Intel Iris Plus 655 (Coffee Lake GT3e)" },
};

MALLOC_DECLARE(M_KMS);

struct intel_gen9_softc {
	device_t		 dev;
	struct drm_device	*drm_dev;
	uint16_t		 pci_id;

	/* MMIO + GTT */
	int			 mmio_rid;
	struct resource		*mmio_res;
	int			 gmadr_rid;
	struct resource		*gmadr_res;

	/* Minimal KMS topology — one of each, all stubs. */
	struct drm_crtc		 crtc;
	struct drm_encoder	 encoder;
	struct drm_connector	 connector;
};

static const struct drm_driver intel_gen9_driver = {
	.name		= "intel_gen9",
	.desc		= "Intel Gen9 iGPU (kms framework)",
	.date		= "20260613",
	.major		= 0,
	.minor		= 1,
	.patchlevel	= 0,
};

static const struct drm_crtc_funcs intel_gen9_crtc_funcs = { 0 };
static const struct drm_encoder_funcs intel_gen9_encoder_funcs = { 0 };
static const struct drm_connector_funcs intel_gen9_connector_funcs = { 0 };

/*
 * Driver atomic hooks.  Trivial first-cut so MODE_ATOMIC against this
 * driver succeeds without actually programming HW.  Real HW writes land
 * once the display engine bring-up code is written.
 */
static int
intel_gen9_atomic_check(struct drm_device *dev __unused,
    struct drm_atomic_state *state __unused)
{
	return (0);
}

static int
intel_gen9_atomic_commit(struct drm_device *dev __unused,
    struct drm_atomic_state *state __unused, bool nonblock __unused)
{
	return (0);
}

static const struct drm_mode_config_funcs intel_gen9_mode_config_funcs = {
	.atomic_check  = intel_gen9_atomic_check,
	.atomic_commit = intel_gen9_atomic_commit,
};

static int
intel_gen9_probe(device_t dev)
{
	uint16_t vid = pci_get_vendor(dev);
	uint16_t did = pci_get_device(dev);
	size_t i;

	if (vid != INTEL_VENDOR_ID)
		return (ENXIO);
	for (i = 0; i < nitems(intel_gen9_ids); i++) {
		if (intel_gen9_ids[i].id == did) {
			device_set_desc(dev, intel_gen9_ids[i].desc);
			return (BUS_PROBE_DEFAULT);
		}
	}
	return (ENXIO);
}

static int
intel_gen9_attach(device_t dev)
{
	struct intel_gen9_softc *sc = device_get_softc(dev);
	int error;

	sc->dev = dev;
	sc->pci_id = pci_get_device(dev);

	/*
	 * BAR0 (PCIR_BAR(0) = 0x10) is GTTMMADR — register MMIO + the
	 * graphics translation table.  Size is 16 MiB on gen9.
	 * BAR2 (PCIR_BAR(2) = 0x18) is GMADR — the aperture into VRAM /
	 * stolen / GTT-mapped buffers.  Size varies by SKU.
	 *
	 * We map both lazily — failing here just disables the driver,
	 * which is the right thing since modesetting can't happen
	 * without MMIO.
	 */
	sc->mmio_rid = PCIR_BAR(0);
	sc->mmio_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->mmio_rid, RF_ACTIVE);
	if (sc->mmio_res == NULL) {
		device_printf(dev, "BAR0/GTTMMADR alloc failed\n");
		return (ENXIO);
	}
	sc->gmadr_rid = PCIR_BAR(2);
	sc->gmadr_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->gmadr_rid, RF_ACTIVE);
	if (sc->gmadr_res == NULL) {
		device_printf(dev, "BAR2/GMADR alloc failed\n");
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mmio_rid,
		    sc->mmio_res);
		return (ENXIO);
	}

	/*
	 * Register with the kms framework: this creates /dev/dri/cardN and
	 * the cdev plus all the ioctl plumbing.  After this returns, user
	 * land can already do GET_VERSION / GET_UNIQUE / GET_CAP against
	 * us — there's just nothing visible on the connectors yet.
	 */
	error = kms_dev_register(&intel_gen9_driver, sc, &sc->drm_dev);
	if (error != 0) {
		device_printf(dev, "kms_dev_register: %d\n", error);
		bus_release_resource(dev, SYS_RES_MEMORY, sc->gmadr_rid,
		    sc->gmadr_res);
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mmio_rid,
		    sc->mmio_res);
		return (error);
	}

	/* Install atomic hooks before any object becomes reachable. */
	sc->drm_dev->mode_config.funcs = &intel_gen9_mode_config_funcs;

	/*
	 * Single stub of each KMS object so GETRESOURCES returns non-empty
	 * and atomic-aware userspace can enumerate something.  Real
	 * topology (one CRTC per pipe, encoders per DDI, connectors per
	 * physical port) lands once the display engine is decoded.
	 */
	error = kms_crtc_init(sc->drm_dev, &sc->crtc, &intel_gen9_crtc_funcs);
	if (error == 0)
		error = kms_encoder_init(sc->drm_dev, &sc->encoder,
		    &intel_gen9_encoder_funcs, DRM_MODE_ENCODER_DAC);
	if (error == 0)
		error = kms_connector_init(sc->drm_dev, &sc->connector,
		    &intel_gen9_connector_funcs, DRM_MODE_CONNECTOR_VGA);
	if (error != 0)
		device_printf(dev, "topology init: %d (will appear with"
		    " empty/partial resources)\n", error);

	device_printf(dev, "attached: PCI 8086:%04x as /dev/dri/card%d\n",
	    sc->pci_id, sc->drm_dev->minor);
	return (0);
}

static int
intel_gen9_detach(device_t dev)
{
	struct intel_gen9_softc *sc = device_get_softc(dev);

	if (sc->drm_dev != NULL) {
		kms_connector_cleanup(&sc->connector);
		kms_encoder_cleanup(&sc->encoder);
		kms_crtc_cleanup(&sc->crtc);
		kms_dev_unregister(sc->drm_dev);
		sc->drm_dev = NULL;
	}
	if (sc->gmadr_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->gmadr_rid,
		    sc->gmadr_res);
	if (sc->mmio_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mmio_rid,
		    sc->mmio_res);
	return (0);
}

static device_method_t intel_gen9_methods[] = {
	DEVMETHOD(device_probe,		intel_gen9_probe),
	DEVMETHOD(device_attach,	intel_gen9_attach),
	DEVMETHOD(device_detach,	intel_gen9_detach),
	DEVMETHOD_END
};

static driver_t intel_gen9_driver_t = {
	"intel_gen9",
	intel_gen9_methods,
	sizeof(struct intel_gen9_softc),
};

DRIVER_MODULE(intel_gen9, pci, intel_gen9_driver_t, 0, 0);
MODULE_VERSION(intel_gen9, 1);
MODULE_DEPEND(intel_gen9, kms, 1, 1, 1);
MODULE_DEPEND(intel_gen9, pci, 1, 1, 1);
