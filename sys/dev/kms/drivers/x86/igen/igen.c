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
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <sys/sx.h>
#include <sys/sysctl.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <vm/vm.h>
#include <vm/pmap.h>

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
#include <kms/drm_framebuffer.h>
#include <kms/drm_gem.h>
#include <kms/drm_plane.h>
#include <kms/drm_vblank.h>

#include "igen_internal.h"

#include <vm/vm_page.h>

#define	INTEL_PCI_VENDOR	0x8086

/*
 * Intel PCI IDs igen attaches to.  Sourced from Intel's
 * "graphics-pciids" header.  Each entry carries the silicon generation
 * it belongs to:
 *   75 = Haswell (gen 7.5) — display engine: LCPLL-fed CDCLK,
 *        WRPLL1/2 + SPLL for clock generation, single PWELL1 for
 *        non-AON power, DDI A (eDP) / B / C / D + DDI E on -ULT.
 *    9 = Skylake / Kabylake / Coffee Lake — gen9/9.5 display engine:
 *        CD2X-divider CDCLK, DPLL0..3 with WRPLL fractional-N solver,
 *        PWELL1 + PWELL2 + DC_OFF/DC5/DC6.
 * igen_attach copies gen into the softc so per-gen code paths can
 * branch on it without re-scanning this table.  Use IGEN_GEN_*
 * (igen_internal.h) at comparison sites.
 */
static const struct {
	uint16_t	id;
	uint8_t		gen;
	const char	*desc;
} igen_ids[] = {
	/*
	 * Haswell (gen 7.5).  All shipping desktop / mobile / mobile
	 * GT3e SKUs, including the BGA-only Crystal Well GT3e (Iris Pro
	 * 5200) found in NUCs and the 2013-2015 Retina MBPs.
	 *
	 * Encoding rule per i915: bits[15:12]=0x0 desktop/mobile chassis
	 * tag, bits[11:8] mark the GT tier (0/1=GT1, 2=GT2, 6=GT3 / 5200,
	 * a=ULT), bits[7:0] are stepping/sub-SKU.
	 */
	{ 0x0402, IGEN_GEN_HSW, "Intel HD (Haswell Desktop GT1)" },
	{ 0x0412, IGEN_GEN_HSW, "Intel HD 4600 (Haswell Desktop GT2)" },
	{ 0x0422, IGEN_GEN_HSW, "Intel HD (Haswell Desktop GT3)" },
	{ 0x0406, IGEN_GEN_HSW, "Intel HD (Haswell Mobile GT1)" },
	{ 0x0416, IGEN_GEN_HSW, "Intel HD 4600 (Haswell Mobile GT2)" },
	{ 0x0426, IGEN_GEN_HSW, "Intel HD 5000 (Haswell Mobile GT3)" },
	{ 0x040a, IGEN_GEN_HSW, "Intel HD (Haswell Server GT1)" },
	{ 0x041a, IGEN_GEN_HSW, "Intel HD P4600/P4700 (Haswell Server GT2)" },
	{ 0x042a, IGEN_GEN_HSW, "Intel HD (Haswell Server GT3)" },
	{ 0x040b, IGEN_GEN_HSW, "Intel HD (Haswell Reserved GT1)" },
	{ 0x041b, IGEN_GEN_HSW, "Intel HD (Haswell Reserved GT2)" },
	{ 0x042b, IGEN_GEN_HSW, "Intel HD (Haswell Reserved GT3)" },
	{ 0x040e, IGEN_GEN_HSW, "Intel HD (Haswell CRW GT1)" },
	{ 0x041e, IGEN_GEN_HSW, "Intel HD 4400 (Haswell CRW GT2)" },
	{ 0x042e, IGEN_GEN_HSW, "Intel HD (Haswell CRW GT3)" },
	{ 0x0a02, IGEN_GEN_HSW, "Intel HD (Haswell ULT GT1)" },
	{ 0x0a12, IGEN_GEN_HSW, "Intel HD (Haswell ULT GT2)" },
	{ 0x0a22, IGEN_GEN_HSW, "Intel HD 5000 (Haswell ULT GT3)" },
	{ 0x0a06, IGEN_GEN_HSW, "Intel HD (Haswell ULT GT1)" },
	{ 0x0a16, IGEN_GEN_HSW, "Intel HD 4400 (Haswell ULT GT2)" },
	{ 0x0a26, IGEN_GEN_HSW, "Intel HD 5000 (Haswell ULT GT3)" },
	{ 0x0a0a, IGEN_GEN_HSW, "Intel HD (Haswell ULT GT1)" },
	{ 0x0a1a, IGEN_GEN_HSW, "Intel HD (Haswell ULT GT2)" },
	{ 0x0a2a, IGEN_GEN_HSW, "Intel HD (Haswell ULT GT3)" },
	{ 0x0a0b, IGEN_GEN_HSW, "Intel HD (Haswell ULT GT1 Reserved)" },
	{ 0x0a1b, IGEN_GEN_HSW, "Intel HD (Haswell ULT GT2 Reserved)" },
	{ 0x0a2b, IGEN_GEN_HSW, "Intel HD (Haswell ULT GT3 Reserved)" },
	{ 0x0a0e, IGEN_GEN_HSW, "Intel HD (Haswell ULX GT1)" },
	{ 0x0a1e, IGEN_GEN_HSW, "Intel HD (Haswell ULX GT2)" },
	{ 0x0a2e, IGEN_GEN_HSW, "Intel Iris 5100 (Haswell ULX GT3)" },
	{ 0x0c02, IGEN_GEN_HSW, "Intel HD (Haswell SDV GT1)" },
	{ 0x0c12, IGEN_GEN_HSW, "Intel HD (Haswell SDV GT2)" },
	{ 0x0c22, IGEN_GEN_HSW, "Intel HD (Haswell SDV GT3)" },
	{ 0x0c06, IGEN_GEN_HSW, "Intel HD (Haswell SDV Mobile GT1)" },
	{ 0x0c16, IGEN_GEN_HSW, "Intel HD P4600/P4700 (Haswell SDV Mobile GT2)" },
	{ 0x0c26, IGEN_GEN_HSW, "Intel HD (Haswell SDV Mobile GT3)" },
	{ 0x0c0a, IGEN_GEN_HSW, "Intel HD (Haswell SDV Server GT1)" },
	{ 0x0c1a, IGEN_GEN_HSW, "Intel HD (Haswell SDV Server GT2)" },
	{ 0x0c2a, IGEN_GEN_HSW, "Intel HD (Haswell SDV Server GT3)" },
	{ 0x0c0b, IGEN_GEN_HSW, "Intel HD (Haswell SDV Reserved GT1)" },
	{ 0x0c1b, IGEN_GEN_HSW, "Intel HD (Haswell SDV Reserved GT2)" },
	{ 0x0c2b, IGEN_GEN_HSW, "Intel HD (Haswell SDV Reserved GT3)" },
	{ 0x0c0e, IGEN_GEN_HSW, "Intel HD (Haswell SDV CRW GT1)" },
	{ 0x0c1e, IGEN_GEN_HSW, "Intel HD (Haswell SDV CRW GT2)" },
	{ 0x0c2e, IGEN_GEN_HSW, "Intel HD (Haswell SDV CRW GT3)" },
	{ 0x0d02, IGEN_GEN_HSW, "Intel HD (Haswell CRW Desktop GT1)" },
	{ 0x0d12, IGEN_GEN_HSW, "Intel HD 4600 (Haswell CRW Desktop GT2)" },
	{ 0x0d22, IGEN_GEN_HSW, "Intel Iris Pro 5200 (Haswell CRW Desktop GT3e)" },
	{ 0x0d06, IGEN_GEN_HSW, "Intel HD (Haswell CRW Mobile GT1)" },
	{ 0x0d16, IGEN_GEN_HSW, "Intel HD 4600 (Haswell CRW Mobile GT2)" },
	{ 0x0d26, IGEN_GEN_HSW, "Intel Iris Pro 5200 (Haswell CRW Mobile GT3e)" },
	{ 0x0d0a, IGEN_GEN_HSW, "Intel HD (Haswell CRW Server GT1)" },
	{ 0x0d1a, IGEN_GEN_HSW, "Intel HD (Haswell CRW Server GT2)" },
	{ 0x0d2a, IGEN_GEN_HSW, "Intel Iris Pro P5200 (Haswell CRW Server GT3e)" },
	{ 0x0d0b, IGEN_GEN_HSW, "Intel HD (Haswell CRW Reserved GT1)" },
	{ 0x0d1b, IGEN_GEN_HSW, "Intel HD (Haswell CRW Reserved GT2)" },
	{ 0x0d2b, IGEN_GEN_HSW, "Intel Iris Pro (Haswell CRW Reserved GT3e)" },
	{ 0x0d0e, IGEN_GEN_HSW, "Intel HD (Haswell CRW GT1)" },
	{ 0x0d1e, IGEN_GEN_HSW, "Intel HD (Haswell CRW GT2)" },
	{ 0x0d2e, IGEN_GEN_HSW, "Intel Iris Pro (Haswell CRW GT3e)" },

	/* Skylake / Kabylake / Coffee Lake — gen 9 / 9.5. */
	{ 0x1902, 9, "Intel HD 510 (Skylake GT1)" },
	{ 0x1906, 9, "Intel HD 510 (Skylake GT1 ULT)" },
	{ 0x190b, 9, "Intel HD 510 (Skylake GT1 Halo)" },
	{ 0x190e, 9, "Intel HD 510 (Skylake GT1 Mobile)" },
	{ 0x1912, 9, "Intel HD 530 (Skylake GT2)" },
	{ 0x1916, 9, "Intel HD 520 (Skylake GT2 ULT)" },
	{ 0x191b, 9, "Intel HD 530 (Skylake GT2 Halo)" },
	{ 0x191d, 9, "Intel HD P530 (Skylake GT2 Workstation)" },
	{ 0x191e, 9, "Intel HD 515 (Skylake GT2 Mobile)" },
	{ 0x1921, 9, "Intel HD 520 (Skylake GT2F)" },
	{ 0x1923, 9, "Intel Iris 540 (Skylake GT3)" },
	{ 0x1926, 9, "Intel Iris 540/550 (Skylake GT3e)" },
	{ 0x1927, 9, "Intel Iris 550 (Skylake GT3e)" },
	{ 0x192b, 9, "Intel Iris Pro 580 (Skylake GT4e)" },
	{ 0x5902, 9, "Intel HD 610 (Kabylake GT1)" },
	{ 0x5906, 9, "Intel HD 610 (Kabylake GT1 ULT)" },
	{ 0x590b, 9, "Intel HD 610 (Kabylake GT1 Halo)" },
	{ 0x590e, 9, "Intel HD 610 (Kabylake GT1 Mobile)" },
	{ 0x5912, 9, "Intel HD 630 (Kabylake GT2)" },
	{ 0x5916, 9, "Intel HD 620 (Kabylake GT2 ULT)" },
	{ 0x591a, 9, "Intel HD P630 (Kabylake GT2 Mobile WS)" },
	{ 0x591b, 9, "Intel HD 630 (Kabylake GT2 Halo)" },
	{ 0x591d, 9, "Intel HD P630 (Kabylake GT2 Workstation)" },
	{ 0x591e, 9, "Intel HD 615 (Kabylake GT2 Mobile)" },
	{ 0x5921, 9, "Intel HD 620 (Kabylake GT2F)" },
	{ 0x5923, 9, "Intel Iris Plus 640 (Kabylake GT3)" },
	{ 0x5926, 9, "Intel Iris Plus 640 (Kabylake GT3e)" },
	{ 0x5927, 9, "Intel Iris Plus 650 (Kabylake GT3e)" },
	{ 0x3e90, 9, "Intel UHD 610 (Coffee Lake GT1)" },
	{ 0x3e91, 9, "Intel UHD 630 (Coffee Lake GT2)" },
	{ 0x3e92, 9, "Intel UHD 630 (Coffee Lake GT2)" },
	{ 0x3e93, 9, "Intel UHD 610 (Coffee Lake GT1)" },
	{ 0x3e96, 9, "Intel UHD P630 (Coffee Lake GT2 Workstation)" },
	{ 0x3e98, 9, "Intel UHD 630 (Coffee Lake GT2)" },
	{ 0x3e9b, 9, "Intel UHD 630 (Coffee Lake GT2 Halo)" },
	{ 0x3ea5, 9, "Intel Iris Plus 655 (Coffee Lake GT3e)" },
};

MALLOC_DECLARE(M_KMS);

/*
 * DPRINTF, USER_FB_GTT_*, struct igen_owned_fb, struct igen_softc, and
 * the inline igen_r32 / igen_w32 accessors all live in
 * igen_internal.h so both compilation units share them.
 */

static void igen_owned_fb_destroy(struct drm_framebuffer *fb);
/* igen_owned_fb_funcs is defined non-static below so igen_gtt.c can reference it. */

/*
 * MMIO ranges we care about on gen9 display.  Bracket the regions, not
 * the full 16 MiB BAR — saves snapshot/diff memory and keeps the diff
 * signal-to-noise high.  Add ranges here as bring-up uncovers new state.
 *
 * Stride is 4 bytes (32-bit registers) for every range; gen9 doesn't have
 * any 8/16-bit-only MMIO that matters for display.
 */
struct igen_range {
	uint32_t	start;
	uint32_t	end;	/* inclusive */
	const char	*name;
};

static const struct igen_range igen_ranges[] = {
	{ 0x00044000, 0x00044100, "INT/HOTPLUG" },
	{ 0x00045000, 0x000455ff, "PWR/DC_STATE" },
	{ 0x00046000, 0x000460ff, "CDCLK/DPLL_CTRL" },
	/*
	 * Only the low 256 bytes here.  The PHY_BC analog channel at
	 * 0x6c100..0x6cfff cannot be read while pipe A is doing live HDMI
	 * scanout-to-DDI_B without stalling the display-engine bus and
	 * wedging the iGPU.  snapshot_save fires from attach() at boot
	 * while firmware's pipe A is still live, so we keep this range
	 * narrow.  The deeper PHY scan is exposed via phy_scan_bc (gated
	 * on PIPE_CONF ENABLE) for one-shot RE captures.
	 */
	{ 0x0006c000, 0x0006c0ff, "DPLL_CFGCR/STATUS/CTRL" },
	{ 0x00060000, 0x000613ff, "TRANS_A/B/C/EDP" },
	{ 0x00064000, 0x000643ff, "DDI_BUF_A/B/C/D/E" },
	{ 0x00068000, 0x000683ff, "TRANS_DDI_FUNC_CTL" },
	{ 0x00070000, 0x000703ff, "PIPE_A" },
	{ 0x00071000, 0x000713ff, "PIPE_B" },
	{ 0x00072000, 0x000723ff, "PIPE_C" },
	{ 0x00070080, 0x000700ff, "PIPEA_GMCH/DSL" },
	{ 0x00130000, 0x0013005f, "NORTH_GMBUS" },
	{ 0x00162000, 0x001623ff, "DDI_AUX_A" },
	{ 0x00164000, 0x001643ff, "DDI_AUX_B" },
	{ 0x00164100, 0x001641ff, "DDI_AUX_C" },
	{ 0x00164200, 0x001642ff, "DDI_AUX_D" },
	{ 0x000c4000, 0x000c43ff, "PCH_FDI/SDE" },
	{ 0x000c6000, 0x000c61ff, "PCH_GMBUS" },
};

/* struct igen_softc + igen_r32/igen_w32 live in igen_internal.h. */

/* ----------------------------- MMIO RE helpers ---------------------------- */

/*
 * Compute how many 32-bit words a full snapshot of igen_ranges[]
 * occupies.  Caller-relative offset of register `addr` within the
 * snapshot is the prefix-sum walk in igen_snapshot_index().
 */
static size_t
igen_snapshot_total_words(void)
{
	size_t words = 0;

	for (size_t i = 0; i < nitems(igen_ranges); i++) {
		words += (igen_ranges[i].end -
		    igen_ranges[i].start) / 4 + 1;
	}
	return (words);
}

/*
 * Return the index into sc->snapshot[] corresponding to MMIO offset
 * `addr`, or -1 if `addr` isn't in any tracked range.  O(N) over the
 * range table — N is ~20 so this is fine in sysctl/debug paths.
 */
static ssize_t
igen_snapshot_index(uint32_t addr)
{
	size_t base = 0;

	for (size_t i = 0; i < nitems(igen_ranges); i++) {
		const struct igen_range *r = &igen_ranges[i];
		size_t words = (r->end - r->start) / 4 + 1;

		if (addr >= r->start && addr <= r->end)
			return ((ssize_t)(base + (addr - r->start) / 4));
		base += words;
	}
	return (-1);
}

static void
igen_snapshot_save(struct igen_softc *sc)
{
	sx_xlock(&sc->re_lock);
	if (sc->snapshot == NULL) {
		sc->snapshot_words = igen_snapshot_total_words();
		sc->snapshot = malloc(sc->snapshot_words * sizeof(uint32_t),
		    M_KMS, M_WAITOK | M_ZERO);
	}
	size_t idx = 0;
	for (size_t i = 0; i < nitems(igen_ranges); i++) {
		const struct igen_range *r = &igen_ranges[i];
		for (uint32_t a = r->start; a <= r->end; a += 4)
			sc->snapshot[idx++] = igen_r32(sc, a);
	}
	sc->snapshot_valid = true;
	sx_xunlock(&sc->re_lock);
	device_printf(sc->dev, "snapshot saved (%zu words / %zu bytes)\n",
	    sc->snapshot_words, sc->snapshot_words * sizeof(uint32_t));
}

static void
igen_snapshot_diff(struct igen_softc *sc)
{
	uint32_t changes = 0;

	sx_slock(&sc->re_lock);
	if (!sc->snapshot_valid) {
		sx_sunlock(&sc->re_lock);
		device_printf(sc->dev,
		    "no snapshot saved — write 1 to mmio_snapshot_save first\n");
		return;
	}
	size_t idx = 0;
	for (size_t i = 0; i < nitems(igen_ranges); i++) {
		const struct igen_range *r = &igen_ranges[i];
		for (uint32_t a = r->start; a <= r->end; a += 4, idx++) {
			uint32_t cur = igen_r32(sc, a);

			if (cur != sc->snapshot[idx]) {
				device_printf(sc->dev,
				    "  %s 0x%08x: 0x%08x -> 0x%08x\n",
				    r->name, a, sc->snapshot[idx], cur);
				changes++;
			}
		}
	}
	sx_sunlock(&sc->re_lock);
	device_printf(sc->dev, "snapshot diff: %u registers changed\n", changes);
}

/*
 * Toggle each bit of (sc->bit_scan_addr) individually, observe whether
 * any other tracked register changed as a side effect, then restore.
 * Use sparingly: this is a strong probe for whether a write is even
 * landing, plus side-effect discovery for poorly documented bits.
 */
static void
igen_bit_scan(struct igen_softc *sc)
{
	uint32_t addr = sc->bit_scan_addr;
	uint32_t orig, observed, bit_mask;

	if (igen_snapshot_index(addr) < 0) {
		device_printf(sc->dev,
		    "bit_scan: 0x%08x not in any tracked range\n", addr);
		return;
	}
	igen_snapshot_save(sc);
	sx_xlock(&sc->re_lock);
	orig = igen_r32(sc, addr);
	device_printf(sc->dev,
	    "bit_scan @0x%08x: orig=0x%08x (will toggle 32 bits)\n",
	    addr, orig);
	for (int bit = 0; bit < 32; bit++) {
		if (sc->bit_scan_skip & (1u << bit))
			continue;
		bit_mask = 1u << bit;
		igen_w32(sc, addr, orig ^ bit_mask);
		observed = igen_r32(sc, addr);
		device_printf(sc->dev,
		    "  bit %2d: wrote 0x%08x, readback 0x%08x %s\n",
		    bit, orig ^ bit_mask, observed,
		    ((observed ^ orig) & bit_mask) ? "RW" : "RO/clamped");
		igen_w32(sc, addr, orig);
	}
	sx_xunlock(&sc->re_lock);
	device_printf(sc->dev, "bit_scan done; running side-effect diff:\n");
	igen_snapshot_diff(sc);
}

static int
igen_sysctl_snapshot_save(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL)
		return (error);
	if (trigger != 0)
		igen_snapshot_save(sc);
	return (0);
}

static int
igen_sysctl_snapshot_diff(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL)
		return (error);
	if (trigger != 0)
		igen_snapshot_diff(sc);
	return (0);
}

static int
igen_sysctl_mmio_read(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	uint32_t addr = sc->poke_addr;
	uint32_t val;
	int error;

	val = igen_r32(sc, addr);
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (req->newptr != NULL)
		return (EPERM);
	return (error);
}

static int
igen_sysctl_mmio_write(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	uint32_t val = 0;
	int error = sysctl_handle_int(oidp, &val, 0, req);

	if (error || req->newptr == NULL)
		return (error);
	device_printf(sc->dev, "mmio_write: 0x%08x <- 0x%08x\n",
	    sc->poke_addr, val);
	igen_w32(sc, sc->poke_addr, val);
	return (0);
}

static int
igen_sysctl_bit_scan(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL)
		return (error);
	if (trigger != 0)
		igen_bit_scan(sc);
	return (0);
}

/* igen_sysctl_edid_read_b lives in igen_gmbus.c. */
static int	igen_sysctl_vbt_dump(SYSCTL_HANDLER_ARGS);
/* igen_sysctl_hpd_dump lives in igen_hpd.c. */
static int	igen_sysctl_cap_dump(SYSCTL_HANDLER_ARGS);
/* DPLL/WRPLL/pw1 sysctl handlers live in igen_dpll.c. */
static int	igen_sysctl_current_mode(SYSCTL_HANDLER_ARGS);
/* GTT / scanout playground sysctl handlers live in igen_gtt.c. */
/* igen_edid_to_mode: public — declared in igen_internal.h. */
static int	igen_attach_edid_modes(struct igen_softc *sc);

static void
igen_re_sysctls_init(struct igen_softc *sc)
{
	struct sysctl_oid_list *children;

	sx_init(&sc->re_lock, "igen_re");
	sx_init(&sc->scanout_lock, "igen_scanout");
	sysctl_ctx_init(&sc->re_sysctl_ctx);
	sc->re_sysctl_tree = SYSCTL_ADD_NODE(&sc->re_sysctl_ctx,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev)),
	    OID_AUTO, "re", CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
	    "MMIO reverse-engineering scaffold");
	children = SYSCTL_CHILDREN(sc->re_sysctl_tree);

	/*
	 * Debug verbosity at the device root (not under .re/) so the
	 * tunable name dev.igen.<n>.debug matches the cross-driver
	 * convention.  CTLFLAG_RWTUN makes it settable both at boot via
	 * loader.conf and at runtime via sysctl.
	 */
	SYSCTL_ADD_INT(&sc->re_sysctl_ctx,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev)),
	    OID_AUTO, "debug", CTLFLAG_RWTUN, &sc->sc_debug, 0,
	    "Debug verbosity: 0=silent, 1=milestones, 2=protocol,"
	    " 3=per-frame, 4=hex");

	/*
	 * EXECBUFFER2 -> ELSP dispatch gate.  Default 0 (validate-only)
	 * so a bad batch from userspace can't wedge the display pipe
	 * during KMS bring-up.
	 */
	SYSCTL_ADD_INT(&sc->re_sysctl_ctx, children,
	    OID_AUTO, "i915_dispatch_enable", CTLFLAG_RWTUN,
	    &sc->i915_dispatch_enable, 0,
	    "EXECBUFFER2 dispatch: 0=off, 1=idle-pipe only, 2=force");

	/*
	 * gen9 DDI-side enable gate for igen_gen9_panel_on.  Default 0
	 * because enabling DDI_BUF_CTL against a cold port PLL wedges
	 * the whole display fabric.  Operator opts in after verifying
	 * the port PLL is up (via hpd_dump / SDEISR HPD_LIVE).
	 */
	SYSCTL_ADD_INT(&sc->re_sysctl_ctx, children,
	    OID_AUTO, "gen9_ddi_enable", CTLFLAG_RWTUN,
	    &sc->gen9_ddi_enable, 0,
	    "gen9_panel_on writes TRANS_CLK_SEL + TRANS_DDI_FUNC_CTL +"
	    " DDI_BUF_CTL enables (0=off, 1=on — DANGER without PLL)");

	/*
	 * gen9 FULL cold-boot pipe-A bring-up.  Default off — atomic_commit
	 * falls back to the safer transcoder-timing-only gen9_panel_on.
	 * Setting to 1 makes atomic_commit call igen_gen9_full_bringup on
	 * the "firmware handed us a dark pipe" path.  Live-verified on
	 * this Dell OptiPlex 5040; other boards need their own captured
	 * baseline before enabling.
	 */
	SYSCTL_ADD_INT(&sc->re_sysctl_ctx, children,
	    OID_AUTO, "gen9_full_bringup", CTLFLAG_RWTUN,
	    &sc->gen9_full_bringup, 0,
	    "atomic_commit uses full DPLL+DDI+pipe cold bring-up when"
	    " firmware boots dark (0=off, 1=on — only fires when pipe"
	    " is dark; safe check on LCPLL/DPLL state before touching"
	    " anything)");

	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "mmio_snapshot_save",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_snapshot_save, "I",
	    "write 1 to snapshot all tracked MMIO ranges");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "mmio_snapshot_diff",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_snapshot_diff, "I",
	    "write 1 to log changes since last snapshot");
	SYSCTL_ADD_UINT(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "mmio_addr", CTLFLAG_RW, &sc->poke_addr, 0,
	    "MMIO byte-offset for mmio_read / mmio_write");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "mmio_read",
	    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_mmio_read, "IU",
	    "read [mmio_addr] (32-bit)");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "mmio_write",
	    CTLTYPE_UINT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_mmio_write, "IU",
	    "write value to [mmio_addr] (32-bit)");
	SYSCTL_ADD_UINT(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "bit_scan_addr", CTLFLAG_RW, &sc->bit_scan_addr, 0,
	    "MMIO byte-offset for bit_scan");
	SYSCTL_ADD_UINT(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "bit_scan_skip", CTLFLAG_RW, &sc->bit_scan_skip, 0,
	    "bitmask of bits bit_scan should leave alone");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "bit_scan",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_bit_scan, "I",
	    "write 1 to scan bit_scan_addr and diff side-effects");
	/* edid_read_b sysctl is owned by igen_gmbus.c. */
	igen_gmbus_register_sysctls(sc);

	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "vbt_dump",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_vbt_dump, "I",
	    "write 1 to map OpRegion via ASLS and walk VBT child devices");
	/* hpd_dump sysctl is owned by igen_hpd.c. */
	igen_hpd_register_sysctls(sc);
	/* phy_dump_bc / phy_scan_bc sysctls are owned by igen_phy.c. */
	igen_phy_register_sysctls(sc);
	/* gt_status sysctl is owned by igen_gt.c. */
	igen_gt_register_sysctls(sc);
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "cap_dump",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_cap_dump, "I",
	    "write 1 to print per-DDI capability table (VBT x SFUSE x HPD)");
	/*
	 * DPLL / WRPLL / clock_state / try_pipe_resume / pw1_up sysctls
	 * are owned by igen_dpll.c.
	 */
	igen_dpll_register_sysctls(sc);
	/*
	 * Haswell pipe/transcoder/DDI bring-up sysctls (no-op on
	 * non-HSW gens; the function itself early-returns).
	 */
	igen_hsw_pipe_register_sysctls(sc);

	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "current_mode",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_current_mode, "I",
	    "write 1 to read back live pipe/transcoder timing as a mode");
	/*
	 * gtt_dump, gtt_alloc_test, test_fb_make, test_fb_flip,
	 * scanout_hold, expose_scanout_fb are owned by igen_gtt.c.
	 */
	igen_gtt_register_sysctls(sc);

	SYSCTL_ADD_U64(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "vblank_a_count", CTLFLAG_RD, &sc->vblank_count_pipe_a, 0,
	    "Pipe A vblanks observed via IRQ");
	SYSCTL_ADD_U64(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "irq_total_count", CTLFLAG_RD, &sc->irq_total_count, 0,
	    "total display-engine IRQs serviced");
}

static void
igen_re_sysctls_fini(struct igen_softc *sc)
{
	sysctl_ctx_free(&sc->re_sysctl_ctx);
	if (sc->snapshot != NULL) {
		free(sc->snapshot, M_KMS);
		sc->snapshot = NULL;
		sc->snapshot_valid = false;
	}
	sx_destroy(&sc->re_lock);
	sx_destroy(&sc->scanout_lock);
}


/* -------------------------------- VBT ------------------------------------- */

/*
 * The Intel firmware exposes a "graphics OpRegion" — a ~8 KiB blob in
 * system RAM whose physical address is in PCI config register ASLS
 * (offset 0xFC).  The OpRegion header starts with "IntelGraphicsMem"
 * and contains four mailboxes.  Mailbox 4, at offset 0x400, holds the
 * VBT (Video BIOS Table).  The VBT is the firmware-provided routing
 * table — the canonical source for which physical port is wired to
 * which DDI, and which GMBus pin its DDC line is on.  Linux's i915
 * parses this; we have to too, otherwise the static SKL+ pin map is
 * a guess.
 */

#define	ASLS_PCI_CFG		0xFC
#define	OPREGION_SIZE		8192
#define	OPREGION_VBT_OFFSET	0x400

struct vbt_header {
	uint8_t		signature[20];	/* "$VBT <platform>" */
	uint16_t	version;
	uint16_t	header_size;
	uint16_t	vbt_size;
	uint8_t		vbt_checksum;
	uint8_t		reserved0;
	uint32_t	bdb_offset;
	uint32_t	aim_offset[4];
} __packed;

struct bdb_header {
	uint8_t		signature[16];	/* "BIOS_DATA_BLOCK " */
	uint16_t	version;
	uint16_t	header_size;
	uint16_t	bdb_size;
} __packed;

#define	BDB_GENERAL_DEFINITIONS	2

struct bdb_general_definitions {
	uint8_t		crt_ddc_gmbus_pin;
	uint8_t		dpms_acpi:1;
	uint8_t		skip_boot_crt_detect:1;
	uint8_t		dpms_aim:1;
	uint8_t		rsvd1:5;
	uint8_t		boot_display[2];
	uint8_t		child_dev_size;
	uint8_t		devices[0];
} __packed;

/*
 * SKL+ child_device_config — 38 bytes, modern layout.
 * Only the fields needed for routing are spelled out; bytes between are
 * intentionally opaque.
 */
struct child_device_config {
	uint16_t	handle;		/* device handle */
	uint16_t	device_type;	/* bitmask; HDMI=0x60D2 etc. */
	uint8_t		device_id[10];
	uint16_t	addin_offset;
	uint8_t		dvo_port;	/* HDMIB=1, DPB=7, etc. */
	uint8_t		i2c_pin;	/* GMBUS pin (not DDC) */
	uint8_t		slave_addr;
	uint8_t		ddc_pin;	/* GMBUS pin for DDC */
	uint16_t	edid_ptr;
	uint8_t		dvo_cfg;
	uint8_t		flags2;
	uint8_t		compat;
	uint8_t		aux_channel;
	uint8_t		dongle_detect;
	uint8_t		pipe_cap:2;
	uint8_t		sdvo_stall:1;
	uint8_t		hpd_status:2;
	uint8_t		integrated_encoder:1;
	uint8_t		capabilities_rsvd:2;
	uint8_t		dvo_wiring;
	uint8_t		mipi_bridge_type;
	uint16_t	device_class_ext;
	uint8_t		dvo_function;
} __packed;

static const char *
igen_dvo_port_name(uint8_t p)
{
	switch (p) {
	case 0:  return "HDMI-A";
	case 1:  return "HDMI-B";
	case 2:  return "HDMI-C";
	case 3:  return "HDMI-D";
	case 4:  return "LVDS";
	case 5:  return "TV";
	case 6:  return "CRT";
	case 7:  return "DP-B";
	case 8:  return "DP-C";
	case 9:  return "DP-D";
	case 10: return "DP-A";
	case 11: return "DP-E";
	case 12: return "HDMI-E";
	default: return "?";
	}
}

static const char *
igen_device_type_name(uint16_t t)
{
	switch (t) {
	case 0x1806: return "eDP";
	case 0x60D2: return "HDMI";
	case 0x60D6: return "DP+HDMI(dual)";
	case 0x68C6: return "DP";
	case 0x1022: return "internal LFP";
	case 0x1009: return "TV";
	default:     return "?";
	}
}

static int
igen_sysctl_vbt_dump(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	uint32_t asls;
	void *va;
	uint8_t *blob;
	struct vbt_header *vbt;
	struct bdb_header *bdb;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	asls = pci_read_config(sc->dev, ASLS_PCI_CFG, 4);
	if (asls == 0 || asls == 0xffffffff) {
		device_printf(sc->dev, "vbt: ASLS empty (0x%08x)\n", asls);
		return (0);
	}
	device_printf(sc->dev, "vbt: ASLS=0x%08x  mapping %u bytes\n",
	    asls, OPREGION_SIZE);

	va = pmap_mapdev(asls, OPREGION_SIZE);
	if (va == NULL) {
		device_printf(sc->dev, "vbt: pmap_mapdev failed\n");
		return (0);
	}
	blob = (uint8_t *)va;

	/* OpRegion header signature check. */
	device_printf(sc->dev, "vbt: OpRegion sig: %c%c%c%c%c%c%c%c\n",
	    blob[0], blob[1], blob[2], blob[3],
	    blob[4], blob[5], blob[6], blob[7]);

	vbt = (struct vbt_header *)(blob + OPREGION_VBT_OFFSET);
	if (memcmp(vbt->signature, "$VBT", 4) != 0) {
		device_printf(sc->dev,
		    "vbt: no $VBT at OpRegion+0x400 (sig: %c%c%c%c)\n",
		    vbt->signature[0], vbt->signature[1],
		    vbt->signature[2], vbt->signature[3]);
		pmap_unmapdev(va, OPREGION_SIZE);
		return (0);
	}
	device_printf(sc->dev,
	    "vbt: $VBT found, version=%u vbt_size=%u bdb_offset=%u\n",
	    vbt->version, vbt->vbt_size, vbt->bdb_offset);

	bdb = (struct bdb_header *)((uint8_t *)vbt + vbt->bdb_offset);
	if (memcmp(bdb->signature, "BIOS_DATA_BLOCK", 15) != 0) {
		device_printf(sc->dev, "vbt: no BDB signature\n");
		pmap_unmapdev(va, OPREGION_SIZE);
		return (0);
	}
	device_printf(sc->dev,
	    "vbt: BDB version=%u bdb_size=%u\n", bdb->version, bdb->bdb_size);

	/* Walk BDB blocks looking for GENERAL_DEFINITIONS (id=2). */
	uint8_t *p = (uint8_t *)bdb + bdb->header_size;
	uint8_t *end = (uint8_t *)bdb + bdb->bdb_size;
	while (p + 3 <= end) {
		uint8_t id = p[0];
		uint16_t size = p[1] | (p[2] << 8);
		uint8_t *body = p + 3;

		if (id == BDB_GENERAL_DEFINITIONS) {
			struct bdb_general_definitions *gd =
			    (struct bdb_general_definitions *)body;
			device_printf(sc->dev,
			    "vbt: BDB block 2 size=%u  CRT_DDC_pin=%u"
			    "  child_dev_size=%u\n",
			    size, gd->crt_ddc_gmbus_pin, gd->child_dev_size);

			uint8_t *dev = gd->devices;
			uint8_t *dev_end = body + size;
			int n = 0;
			while (dev + gd->child_dev_size <= dev_end) {
				struct child_device_config *cd =
				    (struct child_device_config *)dev;
				if (cd->device_type != 0) {
					device_printf(sc->dev,
					    "  child[%d]: handle=0x%04x"
					    "  type=0x%04x (%s)"
					    "  dvo_port=%u (%s)"
					    "  ddc_pin=%u  aux_ch=0x%02x"
					    "\n",
					    n, cd->handle, cd->device_type,
					    igen_device_type_name(
						cd->device_type),
					    cd->dvo_port,
					    igen_dvo_port_name(
						cd->dvo_port),
					    cd->ddc_pin, cd->aux_channel);
				}
				dev += gd->child_dev_size;
				n++;
			}
			break;
		}
		p = body + size;
	}

	pmap_unmapdev(va, OPREGION_SIZE);
	return (0);
}

/* -------------------------- pipe/transcoder readback ---------------------- */

/*
 * Transcoder timing registers per pipe.  HTOTAL / HBLANK / HSYNC / VTOTAL
 * / VBLANK / VSYNC layout (per i915 + BSpec):
 *   bits[28:16] = "(end + 1) - 1"        (e.g. HTOTAL field = htotal-1)
 *   bits[12:0]  = "(start + 1) - 1"      (e.g. HACTIVE field = hactive-1)
 *
 * So decoded value = raw field + 1.
 *
 * Each transcoder bank starts at 0x60000 + (transcoder * 0x1000).
 */
#define	TRANS_HTOTAL(t)		(0x60000 + (t) * 0x1000)
#define	TRANS_HBLANK(t)		(0x60004 + (t) * 0x1000)
#define	TRANS_HSYNC(t)		(0x60008 + (t) * 0x1000)
#define	TRANS_VTOTAL(t)		(0x6000c + (t) * 0x1000)
#define	TRANS_VBLANK(t)		(0x60010 + (t) * 0x1000)
#define	TRANS_VSYNC(t)		(0x60014 + (t) * 0x1000)
#define	TRANS_DDI_FUNC_CTL(t)	(0x60400 + (t) * 0x1000)
#define	PIPE_CONF(p)		(0x70008 + (p) * 0x1000)
#define	  PIPE_CONF_ENABLE	(1u << 31)
#define	  PIPE_CONF_STATE	(1u << 30)
/* DDI_BUF_CTL — 0x100 stride; DDI B = port 1 → 0x64100.  Bit 31 = ENABLE. */
#define	DDI_BUF_CTL_REG(p)	(0x64000 + (p) * 0x100)
#define	  DDI_BUF_CTL_EN	(1u << 31)

/*
 * SKL+ universal-plane primary registers (plane 1 of each pipe).
 * Plane banks are 0x100 apart inside each pipe block.  Pipe stride is
 * 0x1000.  Universal plane 1 of pipe A = 0x70180.
 */
#define	PLANE_CTL(p)		(0x70180 + (p) * 0x1000)
#define	  PLANE_CTL_ENABLE	(1u << 31)
#define	  PLANE_CTL_GAMMA_ENA	(1u << 30)
#define	  PLANE_CTL_FORMAT_SHIFT 24
#define	  PLANE_CTL_FORMAT_MASK	(0xf << 24)
#define	  PLANE_CTL_TILED_SHIFT	10
#define	  PLANE_CTL_TILED_MASK	(0x7 << 10)
#define	PLANE_STRIDE(p)		(0x70188 + (p) * 0x1000)
#define	PLANE_POS(p)		(0x7018c + (p) * 0x1000)
#define	PLANE_SIZE(p)		(0x70190 + (p) * 0x1000)
#define	PLANE_OFFSET(p)		(0x701a4 + (p) * 0x1000)

static const char *
igen_plane_format_name(uint32_t f)
{
	switch (f) {
	case 0x0: return "YUV422-8";
	case 0x1: return "XRGB2101010";
	case 0x2: return "XRGB16161616F";
	case 0x4: return "XRGB8888";
	case 0x6: return "XBGR2101010";
	case 0x8: return "RGB565";
	case 0xc: return "XBGR8888";
	case 0xe: return "Y210/Y212/Y216";
	default:  return "?";
	}
}

static const char *
igen_plane_tiling_name(uint32_t t)
{
	switch (t) {
	case 0: return "linear";
	case 1: return "X-tile";
	case 4: return "Y-tile";
	case 5: return "Yf-tile";
	default: return "?";
	}
}

static void
igen_read_pipe_mode(struct igen_softc *sc, int pipe,
    struct drm_display_mode *m)
{
	uint32_t htotal = igen_r32(sc, TRANS_HTOTAL(pipe));
	uint32_t hsync  = igen_r32(sc, TRANS_HSYNC(pipe));
	uint32_t vtotal = igen_r32(sc, TRANS_VTOTAL(pipe));
	uint32_t vsync  = igen_r32(sc, TRANS_VSYNC(pipe));
	uint32_t fctl   = igen_r32(sc, TRANS_DDI_FUNC_CTL(pipe));

	memset(m, 0, sizeof(*m));
	m->hdisplay    = (htotal & 0x1fff) + 1;
	m->htotal      = ((htotal >> 16) & 0x1fff) + 1;
	m->hsync_start = (hsync & 0x1fff) + 1;
	m->hsync_end   = ((hsync >> 16) & 0x1fff) + 1;
	m->vdisplay    = (vtotal & 0x1fff) + 1;
	m->vtotal      = ((vtotal >> 16) & 0x1fff) + 1;
	m->vsync_start = (vsync & 0x1fff) + 1;
	m->vsync_end   = ((vsync >> 16) & 0x1fff) + 1;
	/* Sync polarity from TRANS_DDI_FUNC_CTL bits 17 (PVSYNC) / 16 (PHSYNC). */
	m->flags  = (fctl & (1u << 16)) ? KMS_MODE_FLAG_PHSYNC : KMS_MODE_FLAG_NHSYNC;
	m->flags |= (fctl & (1u << 17)) ? KMS_MODE_FLAG_PVSYNC : KMS_MODE_FLAG_NVSYNC;
	/*
	 * Pixclk from a register isn't trivial on gen9 (needs CDCLK +
	 * DPLL_CTRL2 decode).  Leave 0 for now; userspace tools fall back
	 * to vrefresh-from-totals if it's the only mode available.
	 */
	m->clock = 0;
	m->vrefresh = 0;
}

static int
igen_sysctl_current_mode(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	struct drm_display_mode m;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	for (int pipe = 0; pipe < 3; pipe++) {
		uint32_t pconf = igen_r32(sc, PIPE_CONF(pipe));
		bool active = (pconf & (PIPE_CONF_ENABLE | PIPE_CONF_STATE))
		    == (PIPE_CONF_ENABLE | PIPE_CONF_STATE);
		device_printf(sc->dev,
		    "pipe %c: PIPE_CONF=0x%08x  %s\n",
		    'A' + pipe, pconf, active ? "ACTIVE" : "idle");
		if (!active)
			continue;
		igen_read_pipe_mode(sc, pipe, &m);
		device_printf(sc->dev,
		    "  %ux%u  htotal=%u  vtotal=%u  hs=%u..%u  vs=%u..%u"
		    "  flags=0x%x\n",
		    m.hdisplay, m.vdisplay, m.htotal, m.vtotal,
		    m.hsync_start, m.hsync_end,
		    m.vsync_start, m.vsync_end, m.flags);

		uint32_t pctl  = igen_r32(sc, PLANE_CTL(pipe));
		uint32_t psurf = igen_r32(sc, PLANE_SURF(pipe));
		uint32_t pstr  = igen_r32(sc, PLANE_STRIDE(pipe));
		uint32_t psize = igen_r32(sc, PLANE_SIZE(pipe));
		uint32_t poff  = igen_r32(sc, PLANE_OFFSET(pipe));
		uint32_t fmt   = (pctl & PLANE_CTL_FORMAT_MASK) >>
		    PLANE_CTL_FORMAT_SHIFT;
		uint32_t tile  = (pctl & PLANE_CTL_TILED_MASK) >>
		    PLANE_CTL_TILED_SHIFT;
		uint32_t w = (psize & 0x1fff) + 1;
		uint32_t h = ((psize >> 16) & 0x1fff) + 1;
		device_printf(sc->dev,
		    "  plane1: CTL=0x%08x  en=%d  fmt=%s  tile=%s\n",
		    pctl, !!(pctl & PLANE_CTL_ENABLE),
		    igen_plane_format_name(fmt),
		    igen_plane_tiling_name(tile));
		device_printf(sc->dev,
		    "          SURF=0x%08x  STRIDE=%u (raw=0x%x)"
		    "  SIZE=%ux%u  OFFSET=0x%08x\n",
		    psurf, pstr * 64, pstr, w, h, poff);
	}
	return (0);
}

/* -------------------------- EDID -> connector mode ------------------------ */

/*
 * Decode an EDID 18-byte detailed timing descriptor (block starts at byte
 * offset 54, 72, 90, or 108 of the 128-byte EDID) into a drm_display_mode.
 * Layout per VESA E-EDID:
 *   0-1:  pixel clock in 10 kHz units (LE)
 *   2:    horizontal active low 8 bits
 *   3:    horizontal blanking low 8 bits
 *   4:    horizontal active high 4 bits [7:4] | blanking high 4 bits [3:0]
 *   5:    vertical active low 8 bits
 *   6:    vertical blanking low 8 bits
 *   7:    vertical active high 4 bits [7:4] | blanking high 4 bits [3:0]
 *   8:    h sync offset low 8 bits
 *   9:    h sync pulse low 8 bits
 *   10:   v sync offset low 4 [7:4] | v sync pulse low 4 [3:0]
 *   11:   h offset high 2 bits [7:6] | h pulse high 2 [5:4] |
 *         v offset high 2 [3:2] | v pulse high 2 [1:0]
 *   12-13: image width / height mm
 *   14:   h border / v border (ignored)
 *   17:   feature flags incl sync polarity bits [2:1] (h=2, v=1)
 */
void
igen_edid_to_mode(const uint8_t *d, struct drm_display_mode *m)
{
	uint32_t pixclk_10kHz = d[0] | ((uint32_t)d[1] << 8);
	uint16_t hactive = d[2] | ((uint16_t)(d[4] >> 4) << 8);
	uint16_t hblank  = d[3] | ((uint16_t)(d[4] & 0xf) << 8);
	uint16_t vactive = d[5] | ((uint16_t)(d[7] >> 4) << 8);
	uint16_t vblank  = d[6] | ((uint16_t)(d[7] & 0xf) << 8);
	uint16_t hoff = d[8]  | ((uint16_t)((d[11] >> 6) & 0x3) << 8);
	uint16_t hpw  = d[9]  | ((uint16_t)((d[11] >> 4) & 0x3) << 8);
	uint16_t voff = (d[10] >> 4) | ((uint16_t)((d[11] >> 2) & 0x3) << 4);
	uint16_t vpw  = (d[10] & 0xf) | ((uint16_t)((d[11] >> 0) & 0x3) << 4);

	m->clock = pixclk_10kHz * 10;	/* kHz */
	m->hdisplay = hactive;
	m->hsync_start = hactive + hoff;
	m->hsync_end = hactive + hoff + hpw;
	m->htotal = hactive + hblank;
	m->vdisplay = vactive;
	m->vsync_start = vactive + voff;
	m->vsync_end = vactive + voff + vpw;
	m->vtotal = vactive + vblank;
	m->flags = (d[17] & 0x02) ? KMS_MODE_FLAG_PHSYNC : KMS_MODE_FLAG_NHSYNC;
	m->flags |= (d[17] & 0x04) ? KMS_MODE_FLAG_PVSYNC : KMS_MODE_FLAG_NVSYNC;
	m->type = KMS_MODE_TYPE_DRIVER | KMS_MODE_TYPE_PREFERRED;
	m->width_mm = d[12] | ((uint32_t)(d[14] >> 4) << 8);
	m->height_mm = d[13] | ((uint32_t)(d[14] & 0xf) << 8);
	m->vrefresh = kms_mode_vrefresh(m);
	kms_mode_set_name(m);
}

/*
 * Table of well-known modes with exact timings.  Established Timings +
 * Standard Timings only carry (resolution, refresh) — no exact H/V
 * totals, sync widths, or pixel clock — so we can't compute a mode
 * from them directly.  Instead we match to this table and copy the
 * exact fields.  Values sourced from VESA DMT / CEA-861 spec.
 * Adding a mode here immediately makes it selectable if the monitor
 * advertises it via ET1 or ST.
 */
struct igen_stock_mode {
	uint16_t	hd, vd;
	uint16_t	refresh;
	uint32_t	clock_khz;
	uint16_t	hs_start, hs_end, htotal;
	uint16_t	vs_start, vs_end, vtotal;
	uint32_t	flags;
};
static const struct igen_stock_mode igen_stock_modes[] = {
	/* VESA DMT — hpol/vpol per DMT spec.  0x1=nvsync/nhsync, 0x5=nhsync/pvsync,
	 * 0x9=phsync/nvsync, 0xa=phsync/pvsync (see KMS_MODE_FLAG_[NP][HV]SYNC bits). */
	{  640,  480, 60,  25175,  656,  752,  800, 490,  492,  525, 0x5 },
	{  640,  480, 72,  31500,  664,  704,  832, 489,  492,  520, 0x5 },
	{  640,  480, 75,  31500,  656,  720,  840, 481,  484,  500, 0x5 },
	{  800,  600, 56,  36000,  824,  896, 1024, 601,  603,  625, 0xa },
	{  800,  600, 60,  40000,  840,  968, 1056, 601,  605,  628, 0xa },
	{  800,  600, 72,  50000,  856,  976, 1040, 637,  643,  666, 0xa },
	{  800,  600, 75,  49500,  816,  896, 1056, 601,  604,  625, 0xa },
	{ 1024,  768, 60,  65000, 1048, 1184, 1344, 771,  777,  806, 0x5 },
	{ 1152,  864, 75, 108000, 1216, 1344, 1600, 865,  868,  900, 0xa },
	{ 1152,  864, 60,  81624, 1216, 1336, 1520, 865,  869,  895, 0x6 }, /* CVT-RB */
	{ 1280,  720, 60,  74250, 1390, 1430, 1650, 725,  730,  750, 0xa }, /* CEA VIC 4 */
	{ 1280,  800, 60,  83500, 1352, 1480, 1680, 803,  809,  831, 0x5 },
	{ 1280, 1024, 60, 108000, 1328, 1440, 1688,1025, 1028, 1066, 0xa },
	{ 1440,  900, 60, 106500, 1520, 1672, 1904, 903,  909,  934, 0x5 },
	{ 1600,  900, 60, 108000, 1624, 1704, 1800, 901,  904,  1000, 0xa }, /* CVT-RB */
	{ 1680, 1050, 60, 146250, 1784, 1960, 2240,1053, 1059, 1089, 0x5 },
	{ 1920, 1080, 60, 148500, 2008, 2052, 2200,1084, 1089, 1125, 0xa }, /* CEA VIC 16 */
	{ 1920, 1200, 60, 154000, 1968, 2000, 2080,1203, 1209, 1235, 0x5 }, /* CVT-RB */
	{ 2560, 1440, 60, 241500, 2608, 2640, 2720,1443, 1448, 1481, 0x5 }, /* CVT-RB */
	{ 3840, 2160, 60, 594000, 4016, 4104, 4400,2168, 2178, 2250, 0xa }, /* CEA VIC 97 */
	{ 3840, 2160, 30, 297000, 4016, 4104, 4400,2168, 2178, 2250, 0xa }, /* CEA VIC 95 */
};

static const struct igen_stock_mode *
igen_stock_lookup(uint16_t hd, uint16_t vd, uint16_t refresh)
{
	for (size_t i = 0; i < nitems(igen_stock_modes); i++) {
		if (igen_stock_modes[i].hd == hd &&
		    igen_stock_modes[i].vd == vd &&
		    igen_stock_modes[i].refresh == refresh)
			return (&igen_stock_modes[i]);
	}
	return (NULL);
}

static void
igen_stock_to_mode(const struct igen_stock_mode *sm, struct drm_display_mode *m)
{
	m->clock       = sm->clock_khz;
	m->hdisplay    = sm->hd;
	m->hsync_start = sm->hs_start;
	m->hsync_end   = sm->hs_end;
	m->htotal      = sm->htotal;
	m->vdisplay    = sm->vd;
	m->vsync_start = sm->vs_start;
	m->vsync_end   = sm->vs_end;
	m->vtotal      = sm->vtotal;
	m->flags       = sm->flags;
	m->type        = KMS_MODE_TYPE_DRIVER;
	m->vrefresh    = kms_mode_vrefresh(m);
	kms_mode_set_name(m);
}

/*
 * Established Timings 1 (EDID byte 35) — bitmap of 8 legacy modes.
 * Order matches VESA E-EDID.  Zero-refresh entries are non-timings
 * (aspect ratio hints) that we skip.
 */
struct igen_et_bit { uint16_t hd, vd, refresh; };
static const struct igen_et_bit igen_et1_bits[8] = {
	{  800,  600, 60 },  /* bit 0 */
	{  800,  600, 56 },  /* bit 1 */
	{  640,  480, 75 },  /* bit 2 */
	{  640,  480, 72 },  /* bit 3 */
	{  640,  480, 67 },  /* bit 4 — Apple Mac II timing; not in stock table */
	{  640,  480, 60 },  /* bit 5 */
	{  720,  400, 88 },  /* bit 6 — not in stock table */
	{  720,  400, 70 },  /* bit 7 — not in stock table */
};

static void
igen_publish_established_timings(struct igen_softc *sc, const uint8_t *edid,
    int *published)
{
	uint8_t et1 = edid[35];

	for (int b = 0; b < 8; b++) {
		if ((et1 & (1u << b)) == 0)
			continue;
		const struct igen_et_bit *e = &igen_et1_bits[b];
		const struct igen_stock_mode *sm =
		    igen_stock_lookup(e->hd, e->vd, e->refresh);
		if (sm == NULL)
			continue;
		struct drm_display_mode *m = kms_mode_create();
		if (m == NULL)
			return;
		igen_stock_to_mode(sm, m);
		kms_connector_add_mode(&sc->connector, m);
		(*published)++;
		device_printf(sc->dev,
		    "edid/est1: added mode %s @%u kHz  %u Hz\n",
		    m->name, m->clock, m->vrefresh);
	}
}

/*
 * Standard Timings (EDID bytes 38..53, 8 * 2 bytes).  Encoding:
 *   byte 0: horizontal resolution = (byte + 31) * 8
 *   byte 1: bits 7:6 = aspect ratio, bits 5:0 = refresh - 60
 *     aspect: 00=16:10 (EDID >=1.3, else 1:1), 01=4:3, 10=5:4, 11=16:9
 * (byte 0 == 0x01 && byte 1 == 0x01) is the unused-slot sentinel.
 */
static void
igen_publish_standard_timings(struct igen_softc *sc, const uint8_t *edid,
    int *published)
{
	for (int i = 0; i < 8; i++) {
		uint8_t b0 = edid[38 + i * 2];
		uint8_t b1 = edid[39 + i * 2];
		if (b0 == 0x01 && b1 == 0x01)
			continue;
		uint16_t hd = (b0 + 31) * 8;
		uint16_t vd;
		switch ((b1 >> 6) & 0x3) {
		case 0: vd = (hd * 10) / 16; break;	/* 16:10 */
		case 1: vd = (hd * 3) / 4; break;	/* 4:3   */
		case 2: vd = (hd * 4) / 5; break;	/* 5:4   */
		case 3: vd = (hd * 9) / 16; break;	/* 16:9  */
		default: continue;
		}
		uint16_t refresh = (b1 & 0x3f) + 60;
		const struct igen_stock_mode *sm =
		    igen_stock_lookup(hd, vd, refresh);
		if (sm == NULL)
			continue;
		struct drm_display_mode *m = kms_mode_create();
		if (m == NULL)
			return;
		igen_stock_to_mode(sm, m);
		kms_connector_add_mode(&sc->connector, m);
		(*published)++;
		device_printf(sc->dev,
		    "edid/std: added mode %s @%u kHz  %u Hz\n",
		    m->name, m->clock, m->vrefresh);
	}
}

/*
 * Walk the populated EDID buffer's 4 DTD slots, build drm_display_mode
 * entries, and publish them on the connector.  Shared between the
 * GMBus (HDMI) and AUX (DP/eDP) EDID acquisition paths so the parse
 * logic doesn't fork.
 */
static int
igen_publish_edid(struct igen_softc *sc, const uint8_t *edid, size_t len)
{
	int published = 0;

	if (len < 128 || edid[0] != 0x00 || edid[1] != 0xff || edid[7] != 0x00) {
		device_printf(sc->dev, "edid: header invalid\n");
		return (EIO);
	}
	for (int i = 54; i <= 108; i += 18) {
		if (edid[i] == 0 && edid[i + 1] == 0)
			continue;
		struct drm_display_mode *m = kms_mode_create();
		if (m == NULL)
			return (ENOMEM);
		igen_edid_to_mode(&edid[i], m);
		kms_connector_add_mode(&sc->connector, m);
		published++;
		device_printf(sc->dev,
		    "edid: added mode %s @%u kHz  %u Hz  flags=0x%x\n",
		    m->name, m->clock, m->vrefresh, m->flags);
	}
	/*
	 * Established + Standard timings on the base block.  These give
	 * the compositor real choices without needing to also parse the
	 * CEA extension block (bytes 128+) yet.
	 */
	igen_publish_established_timings(sc, edid, &published);
	igen_publish_standard_timings(sc, edid, &published);
	(void)kms_connector_update_edid(&sc->connector, edid, len);
	/*
	 * NOTE: kms_connector_hotplug tried here 2026-07-17 and wedged the
	 * box during boot — the mode_config.mutex + dev_lock acquisition
	 * inside hotplug interacts badly with something in the attach-time
	 * path.  Reverted to bare status assign.  A safer approach is to
	 * fire the hotplug event later (from a rc.d post-attach hook, or
	 * from a runtime sysctl) after the drm_device is fully initialized.
	 */
	sc->connector.status = connector_status_connected;
	/*
	 * Cache the base block locally so the HSW bring-up path can
	 * re-parse the preferred DTD when programming the transcoder
	 * without going back through the framework property store.
	 */
	if (len >= sizeof(sc->cached_edid)) {
		memcpy(sc->cached_edid, edid, sizeof(sc->cached_edid));
		sc->cached_edid_len = sizeof(sc->cached_edid);
	}
	return (published > 0 ? 0 : ENOENT);
}

/*
 * Try AUX-EDID on DDI_A.  Used on HSW when PORT_CLK_SEL of DDI_A is
 * driven by LCPLL (the firmware-programmed eDP panel).  EDID lives at
 * I2C address 0x50 over the DP I2C-over-AUX channel.  Returns 0 on
 * success.
 *
 * Validates eDP capability before reading EDID: a successful DPCD rev
 * read confirms the AUX channel is wired and the sink answers — that
 * also tells us whether we should re-tag the connector as eDP.
 */
static int
igen_attach_edid_modes_aux_a(struct igen_softc *sc)
{
	uint8_t edid[128];
	uint8_t dpcd_rev = 0;
	ssize_t got;
	int error;

	got = kms_dp_dpcd_read(&sc->aux_a.aux, 0x000, &dpcd_rev, 1);
	if (got != 1) {
		DPRINTF(sc, 1, "edid/aux: DPCD rev read failed (%zd)\n", got);
		return (EIO);
	}
	device_printf(sc->dev, "edid/aux: DDI_A DPCD rev=0x%02x\n", dpcd_rev);

	error = igen_aux_i2c_read_block(&sc->aux_a.aux, EDID_SLAVE, 0,
	    edid, sizeof(edid));
	if (error < 0) {
		DPRINTF(sc, 1, "edid/aux: I2C-AUX read failed (%d)\n", error);
		return (-error);
	}
	if (error < (int)sizeof(edid)) {
		device_printf(sc->dev,
		    "edid/aux: short read %d/%zu — sink truncated\n",
		    error, sizeof(edid));
		return (EIO);
	}
	return (igen_publish_edid(sc, edid, sizeof(edid)));
}

/*
 * Try GMBus-EDID on PCH pin DDI_B (canonical SKL+ HDMI-B map).  The
 * pre-HSW path; retained for HDMI ports on any gen that wires a
 * working GMBus pin (HSW with a real HDMI sink, SKL with HDMI-B
 * captured in the original baseline).
 */
static int
igen_attach_edid_modes_gmbus_b(struct igen_softc *sc)
{
	uint8_t edid[128];
	int error = EIO;

	/*
	 * Cold-boot state of GMBus often leaves INUSE/SW_CLR_INT stale
	 * enough that the first xfer NAKs.  Retry up to 4 times — by the
	 * 2nd attempt the bus is consistently warm.
	 */
	for (int try = 0; try < 4; try++) {
		error = igen_gmbus_read_block(sc, GMBUS_PIN_DDI_B,
		    EDID_SLAVE, 0, edid, sizeof(edid));
		if (error == 0)
			break;
		DPRINTF(sc, 1,
		    "edid/gmbus: read attempt %d failed (%d), retrying\n",
		    try + 1, error);
		DELAY(5000);
	}
	if (error != 0)
		return (error);
	return (igen_publish_edid(sc, edid, sizeof(edid)));
}

static int
igen_attach_edid_modes(struct igen_softc *sc)
{
	int error = ENOENT;

	/*
	 * On HSW, the firmware almost always brings up the eDP panel on
	 * DDI_A (LCPLL-fed PORT_CLK_SEL).  Try AUX-EDID there first so
	 * the internal panel registers as the live sink with its native
	 * mode — without this the Apple Retina panel on macbsd shows up
	 * with zero modes and Xorg refuses to set a config.
	 *
	 * Detection rule: PORT_CLK_SEL of DDI_A is non-NONE (top 3 bits
	 * != 7).  That covers LCPLL_2700 / LCPLL_1350 / LCPLL_810 / SPLL
	 * / WRPLL1 / WRPLL2 — anything except "port idle".
	 */
	if (sc->gen == IGEN_GEN_HSW) {
		uint32_t sel = igen_r32(sc, 0x00046100u);	/* DDI_A */

		if (((sel >> 29) & 7) != 7) {
			error = igen_attach_edid_modes_aux_a(sc);
			if (error == 0)
				return (0);
			device_printf(sc->dev,
			    "edid/aux: DDI_A read failed (%d), trying GMBus-B"
			    " as fallback\n", error);
		}
	}

	/* GMBus path (SKL canonical; HSW fallback when DDI_A is idle). */
	error = igen_attach_edid_modes_gmbus_b(sc);
	if (error != 0)
		device_printf(sc->dev,
		    "edid: all paths failed — connector stays UNKNOWN\n");
	return (error);
}

/* HPD live decoder lives in igen_hpd.c. */

/* --------------------------- silicon capability table --------------------- */

/*
 * Per-DDI capability summary.  Tied to three independent oracles:
 *   1) VBT child_device_config -- what the board manufacturer wired this
 *      port for (eDP / HDMI / DP / dual)
 *   2) SFUSE_STRAP -- which DDIs the silicon fused on for this SKU (some
 *      KBL desktop parts ship with DDI_D fused off)
 *   3) SDEISR HPD live -- which DDIs currently see a connected sink
 *
 * The intersection of (1) and (2) is what's *available* on this board;
 * (3) is what's *connected right now*.  Captured on fbsdx86 2026-06-13
 * as the canonical 8086:5912 / HD 630 / KBL-S desktop result:
 *
 *   DDI  silicon  VBT-type        live-HPD  notes
 *   ---  -------  --------------  --------  -----------------------------
 *   A    n/a      eDP             0         no internal panel on desktop
 *   B    yes      HDMI            1         firmware-driven XYM 1080p60
 *   C    yes      DP+HDMI(dual)   0         available, no sink connected
 *   D    FUSED    DP+HDMI(dual)   0         SFUSE_STRAP bit 1 == 0
 *   E    n/a      DP              0         no PCH route on this SKU
 *
 * Conclusion: max 2 concurrent displays on this SKU (DDI_B + DDI_C).
 */
static int
igen_sysctl_cap_dump(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	uint32_t asls, sfuse, sde;
	void *va;
	uint8_t *blob;
	struct vbt_header *vbt;
	struct bdb_header *bdb;
	uint16_t per_ddi_type[5] = { 0, 0, 0, 0, 0 };
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	sfuse = igen_r32(sc, SFUSE_STRAP);
	sde   = igen_r32(sc, SDEISR);

	/*
	 * Walk the VBT to fill per_ddi_type[].  We map dvo_port back to a
	 * DDI index: HDMI-B=1/DP-B=7 -> DDI_B(1), HDMI-C=2/DP-C=8 -> DDI_C(2),
	 * HDMI-D=3/DP-D=9 -> DDI_D(3), HDMI-A=0/DP-A=10 -> DDI_A(0),
	 * HDMI-E=12/DP-E=11 -> DDI_E(4).
	 */
	asls = pci_read_config(sc->dev, ASLS_PCI_CFG, 4);
	if (asls != 0 && asls != 0xffffffff &&
	    (va = pmap_mapdev(asls, OPREGION_SIZE)) != NULL) {
		blob = (uint8_t *)va;
		vbt = (struct vbt_header *)(blob + OPREGION_VBT_OFFSET);
		if (memcmp(vbt->signature, "$VBT", 4) == 0) {
			bdb = (struct bdb_header *)((uint8_t *)vbt +
			    vbt->bdb_offset);
			if (memcmp(bdb->signature, "BIOS_DATA_BLOCK", 15) == 0) {
				uint8_t *p = (uint8_t *)bdb + bdb->header_size;
				uint8_t *end = (uint8_t *)bdb + bdb->bdb_size;
				while (p + 3 <= end) {
					uint8_t id = p[0];
					uint16_t sz = p[1] | (p[2] << 8);
					uint8_t *body = p + 3;

					if (id == BDB_GENERAL_DEFINITIONS) {
						struct bdb_general_definitions
						    *gd = (void *)body;
						uint8_t *dev = gd->devices;
						uint8_t *dend = body + sz;
						while (dev + gd->child_dev_size
						    <= dend) {
							struct
							    child_device_config
							    *cd = (void *)dev;
							int ddi = -1;
							switch (cd->dvo_port) {
							case 0:
							case 10: ddi = 0; break;
							case 1:
							case 7:  ddi = 1; break;
							case 2:
							case 8:  ddi = 2; break;
							case 3:
							case 9:  ddi = 3; break;
							case 11:
							case 12: ddi = 4; break;
							}
							if (ddi >= 0 &&
							    cd->device_type != 0
							    && per_ddi_type[
							    ddi] == 0)
								per_ddi_type[
								    ddi] =
								    cd->
								    device_type;
							dev += gd->
							    child_dev_size;
						}
						break;
					}
					p = body + sz;
				}
			}
		}
		pmap_unmapdev(va, OPREGION_SIZE);
	}

	device_printf(sc->dev,
	    "cap: SFUSE_STRAP=0x%08x  SDEISR=0x%08x\n", sfuse, sde);
	device_printf(sc->dev,
	    "cap: DDI  silicon  VBT-type            live-HPD\n");
	for (int ddi = 0; ddi < 5; ddi++) {
		int silicon = 0;
		int hpd = 0;
		const char *vtype = per_ddi_type[ddi] ?
		    igen_device_type_name(per_ddi_type[ddi]) : "(none)";

		switch (ddi) {
		case 1: silicon = (sfuse >> 2) & 1; break;	/* DDI_B */
		case 2: silicon = (sfuse >> 1) & 1; break;	/* DDI_C */
		case 3: silicon = (sfuse >> 0) & 1; break;	/* DDI_D */
		default: silicon = -1;				/* not strapped */
		}
		switch (ddi) {
		case 1: hpd = (sde >> 21) & 1; break;
		case 2: hpd = (sde >> 22) & 1; break;
		case 3: hpd = (sde >> 23) & 1; break;
		case 4: hpd = (sde >> 24) & 1; break;
		}
		device_printf(sc->dev,
		    "cap: %c    %-7s  %-18s  %d\n",
		    'A' + ddi,
		    silicon < 0 ? "n/a" : (silicon ? "yes" : "FUSED"),
		    vtype, hpd);
	}
	return (0);
}

/* ------------------------------ IRQ + vblank ------------------------------ */

/*
 * Display Engine interrupts on gen8+ are a two-level hierarchy:
 *   GEN8_MASTER_IRQ at 0x44200 — top-level enable + DE_PIPE_n_IRQ bits
 *     that say which pipe has a pending event
 *   GEN8_DE_PIPE_<IIR|IMR|IER|ISR>(pipe) — per-pipe banks at
 *     0x44400 + pipe*0x10000.  Bit 0 of these is vblank.
 *
 * Programming sequence (matches i915 gen8_irq_postinstall):
 *   1) GEN8_MASTER_IRQ = 0                 (disable while configuring)
 *   2) IIR = ~0                            (clear stale)
 *   3) IMR = ~GEN8_PIPE_VBLANK             (unmask vblank only)
 *   4) IER = GEN8_PIPE_VBLANK              (enable vblank only)
 *   5) GEN8_MASTER_IRQ = MASTER_CTL | DE_PIPE_A_IRQ
 *
 * Service order (in the handler):
 *   1) master = read GEN8_MASTER_IRQ; if (master & MASTER_CTL) == 0 ret
 *   2) for each PIPE_n_IRQ bit set in master:
 *        iir = read GEN8_DE_PIPE_IIR(n)
 *        if vblank, count
 *        write iir back to W1C
 *   3) GEN8_MASTER_IRQ readback acts as posting flush
 */
#define	GEN8_MASTER_IRQ			0x00044200
#define	  GEN8_MASTER_IRQ_CONTROL	(1u << 31)
#define	  GEN8_DE_PIPE_A_IRQ		(1u << 16)
#define	  GEN8_DE_PIPE_B_IRQ		(1u << 17)
#define	  GEN8_DE_PIPE_C_IRQ		(1u << 18)

#define	GEN8_DE_PIPE_ISR(p)		(0x00044400 + (p) * 0x10000)
#define	GEN8_DE_PIPE_IMR(p)		(0x00044404 + (p) * 0x10000)
#define	GEN8_DE_PIPE_IIR(p)		(0x00044408 + (p) * 0x10000)
#define	GEN8_DE_PIPE_IER(p)		(0x0004440c + (p) * 0x10000)
#define	GEN8_PIPE_VBLANK		(1u << 0)

/*
 * Haswell (gen 7.5) IRQ layout — completely different from BDW+.
 * Single DEISR/DEIMR/DEIIR/DEIER block at 0x44000, with per-pipe and
 * per-port event bits packed into bit fields rather than per-pipe
 * banks.  We define the addresses here for the gen-aware path; full
 * HSW IRQ handler implementation is a follow-up.
 *
 * IMPORTANT: writing GEN8_* registers above on HSW lands on RESERVED
 * MMIO offsets that, in observed practice, destabilise the PCIe link
 * arbiter and take down xhci (USB host controller halt → USB ethernet
 * goes offline → we lose ssh).  Until HSW IRQ handling is implemented,
 * the safe path is to skip MSI/IRQ alloc and register access entirely
 * for HSW and rely on framework polled vblank.
 */
#define	HSW_DEISR			0x00044000
#define	HSW_DEIMR			0x00044004
#define	HSW_DEIIR			0x00044008
#define	HSW_DEIER			0x0004400c
#define	  HSW_DE_PIPE_A_VBLANK		(1u << 0)
#define	  HSW_DE_PIPE_B_VBLANK		(1u << 5)
#define	  HSW_DE_PIPE_C_VBLANK		(1u << 10)
#define	  HSW_DE_MASTER_IRQ_CONTROL	(0u)	/* HSW has no master bit —
						   DEIER bit being set per-event
						   is the enable. */

static void
igen_irq_handler(void *arg)
{
	struct igen_softc *sc = arg;
	uint32_t master, master_w;

	master = igen_r32(sc, GEN8_MASTER_IRQ);
	if ((master & GEN8_MASTER_IRQ_CONTROL) == 0)
		return;
	/* Disable master while servicing; re-enable at end. */
	master_w = master & ~GEN8_MASTER_IRQ_CONTROL;
	igen_w32(sc, GEN8_MASTER_IRQ, master_w);

	sc->irq_total_count++;
	bool pipe_a_vblank = false;
	if (master & GEN8_DE_PIPE_A_IRQ) {
		uint32_t iir = igen_r32(sc, GEN8_DE_PIPE_IIR(0));
		if (iir & GEN8_PIPE_VBLANK) {
			sc->vblank_count_pipe_a++;
			pipe_a_vblank = true;
		}
		igen_w32(sc, GEN8_DE_PIPE_IIR(0), iir);
	}
	if (master & GEN8_DE_PIPE_B_IRQ) {
		uint32_t iir = igen_r32(sc, GEN8_DE_PIPE_IIR(1));
		if (iir & GEN8_PIPE_VBLANK)
			sc->vblank_count_pipe_b++;
		igen_w32(sc, GEN8_DE_PIPE_IIR(1), iir);
	}
	if (master & GEN8_DE_PIPE_C_IRQ) {
		uint32_t iir = igen_r32(sc, GEN8_DE_PIPE_IIR(2));
		if (iir & GEN8_PIPE_VBLANK)
			sc->vblank_count_pipe_c++;
		igen_w32(sc, GEN8_DE_PIPE_IIR(2), iir);
	}

	igen_w32(sc, GEN8_MASTER_IRQ,
	    master_w | GEN8_MASTER_IRQ_CONTROL);
	(void)igen_r32(sc, GEN8_MASTER_IRQ);	/* posting flush */

	/*
	 * Deliver to framework AFTER re-enabling master: kms_vblank_handler
	 * takes mode_config.mutex (sleep-capable sx) and walks the per-file
	 * pending-flip list to dispatch DRM_EVENT_FLIP_COMPLETE.  Cheap in
	 * the no-pending-flip case (one locked check + sequence++).
	 */
	if (pipe_a_vblank)
		kms_vblank_handler(&sc->crtc);
}

static int
igen_irq_setup(struct igen_softc *sc)
{
	int msi_count = 1;
	int error;

	/*
	 * HSW uses a different IRQ register layout (DEIMR/DEIIR/DEIER/
	 * DEISR @ 0x44000-0x4400C, single regs not per-pipe) and the
	 * GEN8_* writes below land on reserved MMIO that destabilises
	 * the PCIe link arbiter (observed: xhci controller halt → USB
	 * ethernet drop on macbsd).  Skip IRQ alloc on HSW entirely
	 * until the HSW IRQ path is implemented; framework polled
	 * vblank covers display correctness in the meantime.
	 */
	if (sc->gen == IGEN_GEN_HSW) {
		device_printf(sc->dev,
		    "irq: skipped on gen75/HSW (using polled vblank;"
		    " HSW DEIMR/DEIIR/DEIER path not implemented yet)\n");
		return (0);
	}

	/*
	 * Quiesce: master off, per-pipe banks all masked / IIRs cleared.
	 * Pipes B/C stay fully off (no scanout there); Pipe A gets the
	 * vblank source unmasked + enabled after MSI is hooked.
	 */
	igen_w32(sc, GEN8_MASTER_IRQ, 0);
	for (int p = 0; p < 3; p++) {
		igen_w32(sc, GEN8_DE_PIPE_IMR(p), 0xffffffff);
		igen_w32(sc, GEN8_DE_PIPE_IER(p), 0);
		igen_w32(sc, GEN8_DE_PIPE_IIR(p), 0xffffffff);
	}
	(void)igen_r32(sc, GEN8_MASTER_IRQ);

	if (pci_alloc_msi(sc->dev, &msi_count) != 0 || msi_count < 1) {
		device_printf(sc->dev, "MSI alloc failed; falling back to INTx\n");
		sc->irq_rid = 0;
	} else {
		sc->irq_rid = 1;
	}
	sc->irq_res = bus_alloc_resource_any(sc->dev, SYS_RES_IRQ,
	    &sc->irq_rid, RF_ACTIVE | RF_SHAREABLE);
	if (sc->irq_res == NULL) {
		device_printf(sc->dev, "IRQ alloc failed\n");
		if (sc->irq_rid == 1)
			pci_release_msi(sc->dev);
		return (ENXIO);
	}
	error = bus_setup_intr(sc->dev, sc->irq_res,
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, igen_irq_handler, sc,
	    &sc->irq_cookie);
	if (error != 0) {
		device_printf(sc->dev, "bus_setup_intr: %d\n", error);
		bus_release_resource(sc->dev, SYS_RES_IRQ, sc->irq_rid,
		    sc->irq_res);
		sc->irq_res = NULL;
		if (sc->irq_rid == 1)
			pci_release_msi(sc->dev);
		return (error);
	}

	/*
	 * Arm Pipe A vblank only (the firmware-active pipe).  B/C remain
	 * fully masked.  Master IRQ_CONTROL turns the whole tree on last.
	 */
	igen_w32(sc, GEN8_DE_PIPE_IIR(0), 0xffffffff);
	igen_w32(sc, GEN8_DE_PIPE_IMR(0), ~GEN8_PIPE_VBLANK);
	igen_w32(sc, GEN8_DE_PIPE_IER(0), GEN8_PIPE_VBLANK);
	igen_w32(sc, GEN8_MASTER_IRQ,
	    GEN8_MASTER_IRQ_CONTROL | GEN8_DE_PIPE_A_IRQ);
	(void)igen_r32(sc, GEN8_MASTER_IRQ);
	DPRINTF(sc, 0, "irq: MSI armed, Pipe A vblank enabled\n");
	return (0);
}

static void
igen_irq_teardown(struct igen_softc *sc)
{
	if (sc->irq_res == NULL)
		return;
	/*
	 * Master off, per-pipe banks masked + cleared.  Only valid on
	 * gens that use the GEN8 IRQ layout (BDW+ / SKL+).  HSW never
	 * brought irq_res up so we won't reach here on HSW, but guard
	 * anyway in case future code changes that.
	 */
	if (sc->gen != IGEN_GEN_HSW) {
		igen_w32(sc, GEN8_MASTER_IRQ, 0);
		for (int p = 0; p < 3; p++) {
			igen_w32(sc, GEN8_DE_PIPE_IMR(p), 0xffffffff);
			igen_w32(sc, GEN8_DE_PIPE_IER(p), 0);
			igen_w32(sc, GEN8_DE_PIPE_IIR(p), 0xffffffff);
		}
	}

	bus_teardown_intr(sc->dev, sc->irq_res, sc->irq_cookie);
	bus_release_resource(sc->dev, SYS_RES_IRQ, sc->irq_rid, sc->irq_res);
	if (sc->irq_rid == 1)
		pci_release_msi(sc->dev);
	sc->irq_res = NULL;
}

/* ----------------------------- driver glue -------------------------------- */

static const struct drm_driver igen_driver = {
	/*
	 * Report Linux i915's identity so libdrm name-based matching and
	 * Mesa's PCI-ID-then-name driver lookup land iris_dri.so.  Without
	 * a name match Mesa falls back to (null) and EGL fails to import
	 * dma-bufs because kms_swrast doesn't implement
	 * EGL_EXT_image_dma_buf_import.  The version below mirrors the
	 * Linux 5.x i915 driver header so iris's verify_kernel_version
	 * is satisfied.
	 */
	.name		= "i915",
	.desc		= "Intel Gen9 iGPU (i915 uAPI compatible)",
	.date		= "20260613",
	.major		= 1,
	.minor		= 6,
	.patchlevel	= 0,
	.ioctl		= igen_i915_ioctl,
};

/*
 * Legacy KMS CRTC ops.  Xorg's modesetting driver uses these (not the
 * atomic state machine) for SETCRTC + PAGE_FLIP.  We accept the request
 * and record the new state; the actual transcoder timing rewrite + plane
 * surface update are no-ops for now -- this unblocks Xorg's "no usable
 * configuration" gate so the X server actually starts.  Real scanout of
 * userspace-allocated dumb buffers requires GTT mapping of the BO pages,
 * which is the next mile.
 */
/*
 * Map the user-allocated dumb buffer into the GTT and arm PLANE_SURF /
 * PLANE_STRIDE to scan from it.  Called from both set_config and
 * page_flip.  The GTT slot allocator (igen_gtt_bind_user_fb) caches
 * per-fb mappings so repeated arming of the same fb is cheap.
 */
static void
igen_program_scanout(struct igen_softc *sc, struct drm_framebuffer *fb)
{
	uint32_t surf;
	uint32_t stride;

	if (fb == NULL)
		return;

	surf = igen_gtt_bind_user_fb(sc, fb);
	if (surf == 0) {
		device_printf(sc->dev,
		    "program_scanout: gtt_bind failed for fb %u (%ux%u, pitch=%u)\n",
		    fb->base.id, fb->width, fb->height, fb->pitches[0]);
		return;
	}

	/* PLANE_STRIDE encodes bytes-per-row / 64. */
	stride = fb->pitches[0] / 64;
	igen_w32(sc, PLANE_STRIDE(0), stride);
	igen_w32(sc, PLANE_SURF(0), surf);
	DPRINTF(sc, 1,
	    "program_scanout: fb %u (%ux%u pitch=%u) -> PLANE_SURF=0x%08x"
	    " STRIDE=%u\n",
	    fb->base.id, fb->width, fb->height, fb->pitches[0],
	    surf, stride);
}

static int
igen_legacy_set_config(struct drm_mode_set *set)
{
	struct drm_crtc *crtc;
	struct igen_softc *sc;

	if (set == NULL || (crtc = set->crtc) == NULL)
		return (EINVAL);
	sc = crtc->dev->driver_priv;

	if (set->mode == NULL) {
		crtc->mode_valid = 0;
		crtc->enabled = false;
		crtc->primary_fb = NULL;
		return (0);
	}

	/*
	 * On gen9 with gen9_full_bringup enabled, route mode-changing
	 * SETCRTC through the modeset chain (pipe_full_off →
	 * dpll1_reprogram → full_bringup).  Otherwise legacy SETCRTC
	 * only swapped the plane fb, leaving pipe/DPLL at the old mode —
	 * so tools that only speak legacy DRM (like our modeset_test)
	 * could never actually change resolution.
	 */
	if (sc->gen == IGEN_GEN_SKL && sc->gen9_full_bringup != 0) {
		struct drm_display_mode live;
		igen_read_pipe_mode(sc, 0, &live);
		if (set->mode->hdisplay != live.hdisplay ||
		    set->mode->vdisplay != live.vdisplay) {
			int perr;

			device_printf(sc->dev,
			    "legacy_set_config: modeset %ux%u -> %ux%u"
			    " @%d kHz\n",
			    live.hdisplay, live.vdisplay,
			    set->mode->hdisplay, set->mode->vdisplay,
			    set->mode->clock);
			perr = igen_gen9_pipe_full_off(sc);
			if (perr == 0)
				perr = igen_gen9_dpll1_reprogram(sc,
				    (uint32_t)set->mode->clock);
			if (perr == 0)
				perr = igen_gen9_full_bringup(sc, set->mode);
			if (perr != 0) {
				device_printf(sc->dev,
				    "legacy_set_config: modeset failed"
				    " (%d)\n", perr);
				return (perr);
			}
		}
	}

	crtc->mode = *set->mode;
	crtc->mode_valid = 1;
	crtc->enabled = true;
	crtc->primary_fb = set->fb;
	crtc->x = set->x;
	crtc->y = set->y;

	igen_program_scanout(sc, set->fb);
	return (0);
}

static int
igen_legacy_page_flip(struct drm_crtc *crtc, struct drm_framebuffer *fb,
    uint32_t flags __unused, uint64_t user_data __unused)
{
	struct igen_softc *sc;

	if (crtc == NULL)
		return (EINVAL);
	sc = crtc->dev->driver_priv;
	crtc->primary_fb = fb;
	igen_program_scanout(sc, fb);
	return (0);
}

/*
 * Skylake / KBL / CFL hardware cursor on CUR_*_A.  The cursor BO is a
 * 64/128/256-wide ARGB8888 bitmap from userspace, lives in a dedicated
 * GTT slot at CURSOR_GTT_FIRST.  CUR_POS_A encodes the top-left in a
 * signed-magnitude form: bit 15 = X-negative, bits 14:0 = |X|; bit 31 =
 * Y-negative, bits 30:16 = |Y|.  Userspace coordinates address the
 * cursor's reference point (hotspot); we subtract it from each MOVE to
 * derive top-left.
 */
/* CUR_CTL_A / CUR_BASE_A / CUR_POS_A + modes live in igen_internal.h so
 * igen_hsw_pipe.c's pipe-off primitive can disable the cursor too. */

static int
igen_cursor_set(struct drm_crtc *crtc, struct drm_file *file,
    uint32_t handle, uint32_t width, uint32_t height,
    int32_t hot_x, int32_t hot_y)
{
	struct igen_softc *sc;
	struct drm_gem_object *obj, *prev;
	uint32_t mode, surf;

	if (crtc == NULL)
		return (EINVAL);
	sc = crtc->dev->driver_priv;

	sc->cursor_hot_x = hot_x;
	sc->cursor_hot_y = hot_y;
	sc->cursor_w = width;
	sc->cursor_h = height;

	if (handle == 0 || width == 0 || height == 0) {
		sx_xlock(&sc->scanout_lock);
		igen_w32(sc, CUR_CTL_A, CUR_MODE_DISABLE);
		igen_w32(sc, CUR_BASE_A, 0);
		sx_xunlock(&sc->scanout_lock);
		prev = sc->cursor_obj;
		sc->cursor_obj = NULL;
		if (prev != NULL)
			kms_gem_object_put(prev);
		return (0);
	}

	if (width != height ||
	    (width != 64 && width != 128 && width != 256))
		return (EINVAL);

	obj = kms_gem_handle_lookup(file, handle);
	if (obj == NULL)
		return (ENOENT);

	surf = igen_gtt_bind_cursor(sc, obj);
	if (surf == 0) {
		kms_gem_object_put(obj);
		return (ENOMEM);
	}

	switch (width) {
	case 64:
		mode = CUR_MODE_64_ARGB;
		break;
	case 128:
		mode = CUR_MODE_128_ARGB;
		break;
	default:
		mode = CUR_MODE_256_ARGB;
		break;
	}

	prev = sc->cursor_obj;
	sc->cursor_obj = obj;
	/* Serialize with atomic_commit + cursor_move (same pipe A regs). */
	sx_xlock(&sc->scanout_lock);
	igen_w32(sc, CUR_BASE_A, surf);
	igen_w32(sc, CUR_CTL_A, mode);
	sx_xunlock(&sc->scanout_lock);
	if (prev != NULL)
		kms_gem_object_put(prev);
	return (0);
}

static int
igen_cursor_move(struct drm_crtc *crtc, int32_t x, int32_t y)
{
	struct igen_softc *sc;
	int32_t tlx, tly;
	uint32_t pos = 0;

	if (crtc == NULL)
		return (EINVAL);
	sc = crtc->dev->driver_priv;

	tlx = x - sc->cursor_hot_x;
	tly = y - sc->cursor_hot_y;

	if (tlx < 0)
		pos |= (1u << 15) | ((uint32_t)(-tlx) & 0x7fffu);
	else
		pos |= ((uint32_t)tlx & 0x7fffu);
	if (tly < 0)
		pos |= (1u << 31) | (((uint32_t)(-tly) & 0x7fffu) << 16);
	else
		pos |= (((uint32_t)tly & 0x7fffu) << 16);

	/*
	 * Serialize with atomic_commit — both target pipe A registers
	 * and share the scanout timing.  Without the lock a cursor move
	 * during a plane swap can land between the atomic_commit's
	 * vblank wait and its PLANE_SURF write, corrupting the frame.
	 */
	sx_xlock(&sc->scanout_lock);
	igen_w32(sc, CUR_POS_A, pos);
	sx_xunlock(&sc->scanout_lock);
	return (0);
}

static const struct drm_crtc_funcs igen_crtc_funcs = {
	.set_config = igen_legacy_set_config,
	.page_flip = igen_legacy_page_flip,
	.cursor_set = igen_cursor_set,
	.cursor_move = igen_cursor_move,
};
static const struct drm_plane_funcs igen_plane_funcs = { 0 };
static const struct drm_encoder_funcs igen_encoder_funcs = { 0 };
static const struct drm_connector_funcs igen_connector_funcs = { 0 };

static void igen_owned_fb_destroy(struct drm_framebuffer *fb) { (void)fb; }

const struct drm_framebuffer_funcs igen_owned_fb_funcs = {
	.destroy = igen_owned_fb_destroy,
};

static const uint32_t igen_plane_formats[] = {
	0x34325258,	/* 'XR24' = DRM_FORMAT_XRGB8888 */
	0x34325241,	/* 'AR24' = DRM_FORMAT_ARGB8888 */
	0x34324258,	/* 'XB24' = DRM_FORMAT_XBGR8888 */
	0x34324241,	/* 'AB24' = DRM_FORMAT_ABGR8888 */
	0x36314752,	/* 'RG16' = DRM_FORMAT_RGB565    */
};

/*
 * Driver atomic hooks.  Trivial first-cut so MODE_ATOMIC against this
 * driver succeeds without actually programming HW.  Real HW writes land
 * once the display engine bring-up code is written.
 */
static int
igen_atomic_check(struct drm_device *dev __unused,
    struct drm_atomic_state *state __unused)
{
	return (0);
}

/*
 * Atomic commit: today this driver only safely handles "the requested
 * timing matches what firmware already programmed."  That's the
 * stand-the-display-up case after EDID-on-attach.  Real modeset (DDI
 * voltage swing, DPLL/CDCLK, port-width, transcoder timing writes) is
 * still TODO — until then, fall through to no-op for matching modes
 * and decline mismatches rather than half-program them.
 */
static int
igen_atomic_commit(struct drm_device *dev, struct drm_atomic_state *state,
    bool nonblock __unused)
{
	struct igen_softc *sc = dev->driver_priv;
	int error = 0;

	sx_xlock(&sc->scanout_lock);
	for (uint32_t i = 0; i < state->num_crtc; i++) {
		struct drm_crtc_state *cs = state->crtc_states[i];
		struct drm_display_mode live;

		if (cs == NULL || !cs->mode_changed)
			continue;
		if (!cs->active) {
			/* Pipe-off: also TODO, but logging is harmless. */
			DPRINTF(sc, 1,
			    "atomic_commit: pipe %u off-request (no-op)\n", i);
			continue;
		}

		/*
		 * HSW cold-panel bring-up.  Macbsd (15" MBP Retina) firmware
		 * leaves the display half-up: CDCLK live and both PWELLs on
		 * but PIPE_CONF reads 0 — no transcoder, no DDI buffer, dark
		 * panel.  On that path atomic_commit used to reject with
		 * ENOTSUP because live.hdisplay came back as 1 (register was
		 * 0, +1 in read_pipe_mode) and no size match landed.
		 *
		 * Instead: on HSW pipe 0 only, if the pipe isn't currently
		 * scanning, walk the panel-on sequence from cached EDID.  It
		 * programs TRANS_EDP timing + PIPE_DATA_M/N + PLANE_SIZE +
		 * PIPE_CONF ENABLE.  When that returns 0 the pipe is live at
		 * the panel's native mode and the size-compare below is
		 * meaningful.  hsw_panel_on is a no-op on non-HSW gens
		 * (they're already running from firmware).
		 */
		if ((sc->gen == IGEN_GEN_HSW || sc->gen == IGEN_GEN_SKL) &&
		    i == 0) {
			/*
			 * "Needs bring-up" is broader than "PIPE_CONF bits
			 * clear."  Two live-verified handoff shapes:
			 *
			 *  1. Cold physical boot: PIPE_CONF=0xc0000000
			 *     (ENABLE + STATE set) but TRANS_HTOTAL/VTOTAL
			 *     are zero — the pipe is "scanning" but the
			 *     transcoder produces a null frame and
			 *     read_pipe_mode reports 1x1.
			 *
			 *  2. Warm/software reboot: TRANS_HTOTAL/VTOTAL
			 *     survive stale valid (probe reports 1920x1080)
			 *     but DDI_BUF_CTL(B) got cleared and/or
			 *     PIPE_CONF STATE dropped during teardown, so
			 *     HDMI is dark even though timing regs look OK.
			 *
			 * Cover both: bring-up when (a) probe timing is
			 * bogus, OR (b) PIPE_CONF STATE bit is clear, OR
			 * (c) DDI B buffer isn't enabled.  Any one of those
			 * means no pixels are reaching the panel.
			 */
			struct drm_display_mode probe;
			uint32_t pconf_live = igen_r32(sc, PIPE_CONF(0));
			uint32_t ddi_b_live = igen_r32(sc, DDI_BUF_CTL_REG(1));
			igen_read_pipe_mode(sc, 0, &probe);
			bool needs_bringup = (probe.hdisplay < 32 ||
			    probe.vdisplay < 32 ||
			    (pconf_live & PIPE_CONF_STATE) == 0 ||
			    (ddi_b_live & DDI_BUF_CTL_EN) == 0);
			if (needs_bringup)
				device_printf(sc->dev,
				    "atomic_commit: bring-up trigger"
				    " probe=%ux%u PIPE_CONF=0x%08x"
				    " DDI_BUF_B=0x%08x\n",
				    probe.hdisplay, probe.vdisplay,
				    pconf_live, ddi_b_live);
			if (needs_bringup) {
				int perr;
				if (sc->gen == IGEN_GEN_HSW)
					perr = igen_hsw_panel_on(sc);
				else if (sc->gen9_full_bringup != 0)
					perr = igen_gen9_full_bringup(sc,
					    &cs->mode);
				else
					perr = igen_gen9_panel_on(sc,
					    &cs->mode);
				if (perr != 0) {
					device_printf(sc->dev,
					    "atomic_commit: panel_on"
					    " (gen %d) failed: %d\n",
					    sc->gen, perr);
					error = perr;
					goto out;
				}
			}
		}

		igen_read_pipe_mode(sc, 0, &live);
		/*
		 * Loosened match: accept any mode whose visible size
		 * (hdisplay x vdisplay) matches the live pipe's.  We
		 * don't reprogram timing yet, but Wayland compositors
		 * (kwin, weston) build their requested mode from EDID
		 * detailed-timing blocks which routinely differ in
		 * porch / sync widths from what firmware programmed.
		 * Strict (htotal == htotal) match returns ENOTSUP →
		 * kwin logs "Applying output configuration failed" →
		 * the compositor never finishes setup, EGLImageKHR
		 * allocations fail with BAD_ALLOC, the session collapses.
		 *
		 * Accepting size-only matches is safe because the pipe
		 * is already scanning live timing that produces those
		 * visible dimensions.  When DPLL1 + transcoder
		 * reprogramming lands (Phase 3 of the cold-modeset work
		 * tracked in BASELINE.txt), this check tightens back up.
		 */
		if (cs->mode.hdisplay != live.hdisplay ||
		    cs->mode.vdisplay != live.vdisplay) {
			int perr;

			/*
			 * Full modeset — pipe/DDI teardown → DPLL1 reprogram
			 * for the new pixel clock → pipe/DDI back up.  Uses
			 * the Linux-ordered DPLL cycle (see
			 * project_igen_dpll_reprogram_working_2026_07_17.md
			 * for the wedge history and the working sequence).
			 * Gated behind gen9_full_bringup as before.
			 */
			if (sc->gen != IGEN_GEN_SKL ||
			    sc->gen9_full_bringup == 0) {
				device_printf(sc->dev,
				    "atomic_commit: requested %ux%u != live"
				    " %ux%u; set gen9_full_bringup=1 to"
				    " enable modeset\n",
				    cs->mode.hdisplay, cs->mode.vdisplay,
				    live.hdisplay, live.vdisplay);
				error = ENOTSUP;
				goto out;
			}

			device_printf(sc->dev,
			    "atomic_commit: modeset %ux%u @%d kHz ->"
			    " %ux%u @%d kHz\n",
			    live.hdisplay, live.vdisplay, live.clock,
			    cs->mode.hdisplay, cs->mode.vdisplay,
			    cs->mode.clock);

			perr = igen_gen9_pipe_full_off(sc);
			if (perr != 0) {
				device_printf(sc->dev,
				    "atomic_commit: pipe_full_off failed:"
				    " %d\n", perr);
				error = perr;
				goto out;
			}
			perr = igen_gen9_dpll1_reprogram(sc,
			    (uint32_t)cs->mode.clock);
			if (perr != 0) {
				device_printf(sc->dev,
				    "atomic_commit: dpll1_reprogram failed:"
				    " %d\n", perr);
				error = perr;
				goto out;
			}
			perr = igen_gen9_full_bringup(sc, &cs->mode);
			if (perr != 0) {
				device_printf(sc->dev,
				    "atomic_commit: gen9_full_bringup failed"
				    " after modeset: %d\n", perr);
				error = perr;
				goto out;
			}
			igen_read_pipe_mode(sc, 0, &live);
		}
		if (cs->mode.htotal != live.htotal ||
		    cs->mode.vtotal != live.vtotal) {
			DPRINTF(sc, 1,
			    "atomic_commit: pipe %u size %ux%u matches live;"
			    " timing differs (req htotal=%u vtotal=%u,"
			    " live htotal=%u vtotal=%u) — accepting no-op\n",
			    i, cs->mode.hdisplay, cs->mode.vdisplay,
			    cs->mode.htotal, cs->mode.vtotal,
			    live.htotal, live.vtotal);
		} else {
			DPRINTF(sc, 1,
			    "atomic_commit: pipe %u — requested matches live"
			    " %ux%u  (no-op)\n", i,
			    live.hdisplay, live.vdisplay);
		}
	}

	/*
	 * Plane swap: if userspace pointed our primary plane at one of our
	 * own owned framebuffers, write PLANE_SURF to its GTT offset.
	 * Setting plane->fb = NULL (disable) is the caller's way to clear
	 * the user buffer — we restore the firmware FB rather than
	 * blanking, to keep the user from accidentally losing their tty.
	 */
	for (uint32_t i = 0; i < state->num_plane; i++) {
		struct drm_plane_state *ps = state->plane_states[i];

		if (ps == NULL || ps->plane != &sc->primary)
			continue;

		if (ps->fb != NULL && ps->fb->funcs == &igen_owned_fb_funcs) {
			struct igen_owned_fb *ofb = __containerof(
			    ps->fb, struct igen_owned_fb, base);
			uint32_t new_surf = ofb->test_fb->gtt_first_idx *
			    PAGE_SIZE;
			if (!sc->scanout_held)
				sc->scanout_prev_surf =
				    igen_r32(sc, PLANE_SURF(0));
			igen_wait_vblank(sc, 0);
			igen_w32(sc, PLANE_SURF(0), new_surf);
			sc->scanout_held = true;
			DPRINTF(sc, 1,
			    "atomic_commit: plane FB_ID %u -> PLANE_SURF=0x%08x\n",
			    ps->fb->base.id, new_surf);
		} else if (ps->fb != NULL) {
			/*
			 * Generic dumb-buffer FB (CREATE_DUMB + ADDFB2 path).
			 * Bind its GEM pages into our user-FB GTT range,
			 * then point PLANE_SURF at that range.  Sync the
			 * PLANE_SURF write to a vblank edge so the frame
			 * currently scanning out completes before HW
			 * latches the new SURF -- eliminates partial-frame
			 * tearing on the page-flip path.
			 */
			uint32_t new_surf = igen_gtt_bind_user_fb(sc,
			    ps->fb);
			uint32_t new_stride = ps->fb->pitches[0] / 64;
			if (new_surf == 0) {
				device_printf(sc->dev,
				    "atomic_commit: FB_ID %u has no GEM"
				    " backing\n", ps->fb->base.id);
				continue;
			}
			if (!sc->scanout_held)
				sc->scanout_prev_surf =
				    igen_r32(sc, PLANE_SURF(0));
			igen_wait_vblank(sc, 0);
			/*
			 * Update STRIDE + SURF together.  Modeset changes the
			 * fb size + pitch; setting SURF alone left the OLD
			 * stride latched (from gen9_full_bringup's boot-fb),
			 * so the scanout read the new fb with the wrong
			 * pitch and produced garbled output ("rendering goes
			 * haywire" post-modeset — user report 2026-07-17).
			 */
			igen_w32(sc, PLANE_STRIDE(0), new_stride);
			igen_w32(sc, PLANE_SURF(0), new_surf);
			DPRINTF(sc, 1,
			    "atomic_commit: plane FB_ID %u -> PLANE_SURF="
			    "0x%08x  STRIDE=%u  (%ux%u pitch=%u)\n",
			    ps->fb->base.id, new_surf, new_stride,
			    ps->fb->width, ps->fb->height,
			    ps->fb->pitches[0]);
			sc->scanout_held = true;
		} else if (ps->fb == NULL && sc->scanout_held) {
			igen_wait_vblank(sc, 0);
			igen_w32(sc, PLANE_SURF(0), sc->scanout_prev_surf);
			sc->scanout_held = false;
			DPRINTF(sc, 1,
			    "atomic_commit: plane fb=NULL -> PLANE_SURF"
			    " restored to 0x%08x\n", sc->scanout_prev_surf);
		}
	}
out:
	sx_xunlock(&sc->scanout_lock);
	return (error);
}

static const struct drm_mode_config_funcs igen_mode_config_funcs = {
	.atomic_check  = igen_atomic_check,
	.atomic_commit = igen_atomic_commit,
};

static int
igen_probe(device_t dev)
{
	uint16_t vid = pci_get_vendor(dev);
	uint16_t did = pci_get_device(dev);
	size_t i;

	if (vid != INTEL_PCI_VENDOR)
		return (ENXIO);
	for (i = 0; i < nitems(igen_ids); i++) {
		if (igen_ids[i].id == did) {
			device_set_desc(dev, igen_ids[i].desc);
			return (BUS_PROBE_DEFAULT);
		}
	}
	return (ENXIO);
}

static int
igen_attach(device_t dev)
{
	struct igen_softc *sc = device_get_softc(dev);
	int error;

	sc->dev = dev;
	sc->pci_id = pci_get_device(dev);
	for (size_t i = 0; i < nitems(igen_ids); i++) {
		if (igen_ids[i].id == sc->pci_id) {
			sc->gen = igen_ids[i].gen;
			break;
		}
	}

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
	error = kms_dev_register(&igen_driver, sc, &sc->drm_dev);
	if (error != 0) {
		device_printf(dev, "kms_dev_register: %d\n", error);
		bus_release_resource(dev, SYS_RES_MEMORY, sc->gmadr_rid,
		    sc->gmadr_res);
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mmio_rid,
		    sc->mmio_res);
		return (error);
	}

	/*
	 * Publish hw.dri.<minor>.busid so libdrm's drmGetDeviceFromDevId
	 * can map our cdev back to a PCI domain/bus/slot/func.  Without
	 * this, Wayland compositors fail "couldn't find dev node for drm
	 * device" and wedge before issuing a single ioctl.
	 */
	kms_set_busid_pci(sc->drm_dev,
	    pci_get_domain(dev), pci_get_bus(dev),
	    pci_get_slot(dev), pci_get_function(dev));

	/* Install atomic hooks before any object becomes reachable. */
	sc->drm_dev->mode_config.funcs = &igen_mode_config_funcs;

	/*
	 * RE scaffold: live MMIO snapshot/diff/poke/bit-scan via sysctl.
	 * Required before any display engine bring-up since the only honest
	 * way to characterise gen9 modeset state is to capture what the
	 * firmware / loader / previous driver left behind, then watch what
	 * each write actually does.
	 */
	igen_re_sysctls_init(sc);
	igen_snapshot_save(sc);

	/*
	 * Single stub of each KMS object so GETRESOURCES returns non-empty
	 * and atomic-aware userspace can enumerate something.  Real
	 * topology (one CRTC per pipe, encoders per DDI, connectors per
	 * physical port) lands once the display engine is decoded.
	 */
	error = kms_crtc_init(sc->drm_dev, &sc->crtc, &igen_crtc_funcs);
	if (error == 0)
		error = kms_plane_init(sc->drm_dev, &sc->primary,
		    &igen_plane_funcs, DRM_PLANE_TYPE_PRIMARY,
		    1u, igen_plane_formats,
		    nitems(igen_plane_formats));
	if (error == 0)
		error = kms_encoder_init(sc->drm_dev, &sc->encoder,
		    &igen_encoder_funcs, DRM_MODE_ENCODER_TMDS);
	if (error == 0) {
		/*
		 * Encoder must declare which CRTCs it can drive (bitmask of
		 * CRTC indices) and the connector must be linked to it.
		 * Without these two writes Xorg modesetting cannot find a
		 * usable encoder/CRTC path from the connector and fails the
		 * initial config with "No modes".
		 */
		sc->encoder.possible_crtcs = 1u;
	}
	/*
	 * Connector type: on HSW with DDI_A driven by LCPLL the firmware
	 * is scanning the internal eDP panel and the right userspace
	 * tag is eDP, not HDMIA.  Apple eDP panels also refuse a few
	 * power-management DPCD writes that Xorg modesetting emits when
	 * the connector is mistyped — so getting this right matters for
	 * the first modeset, not just for cosmetics.
	 */
	uint32_t conn_type = DRM_MODE_CONNECTOR_HDMIA;
	if (sc->gen == IGEN_GEN_HSW) {
		uint32_t sel_a = igen_r32(sc, 0x00046100u);

		if (((sel_a >> 29) & 7) != 7)
			conn_type = DRM_MODE_CONNECTOR_eDP;
	}
	if (error == 0)
		error = kms_connector_init(sc->drm_dev, &sc->connector,
		    &igen_connector_funcs, conn_type);
	if (error == 0)
		error = kms_connector_attach_encoder(&sc->connector,
		    &sc->encoder);

	/*
	 * AUX channel for DDI A (always; the framework helpers are
	 * harmless to register even if no eDP panel is wired).  Adding
	 * DDI B/C/D/E channels lands when external DP/DP++ sinks are
	 * detected via VBT or HPD live state.
	 */
	igen_aux_init(sc, &sc->aux_a, 0, "DDI_A");
	if (error != 0)
		device_printf(dev, "topology init: %d (will appear with"
		    " empty/partial resources)\n", error);

	/*
	 * IRQ — MSI + Pipe A vblank.  Non-fatal; falling back to polled
	 * vblank still works (igen_wait_vblank reads PIPE_FRMCOUNT).
	 */
	(void)igen_irq_setup(sc);

	/*
	 * Best-effort EDID-on-attach: try to fetch the DDI_B EDID via GMBus
	 * and populate the connector's mode list.  Failure leaves the
	 * connector in UNKNOWN with no modes — userspace GETCONNECTOR still
	 * works, it just sees a connector with no detected sink.
	 */
	(void)igen_attach_edid_modes(sc);

	/*
	 * Attach-time auto-fire of gen9_full_bringup was tried but produced
	 * a delayed session wedge (2026-07-17, 8th wedge in the KMS session).
	 * Reverted.  Rely on the atomic_commit trigger widened to also fire
	 * when DDI_BUF_CTL(B) or PIPE_CONF STATE is clear.
	 */

	device_printf(dev, "attached: PCI 8086:%04x gen%d as /dev/dri/card%d\n",
	    sc->pci_id, sc->gen, sc->drm_dev->minor);
	return (0);
}

static int
igen_detach(device_t dev)
{
	struct igen_softc *sc = device_get_softc(dev);

	igen_gtt_detach(sc);
	if (sc->drm_dev != NULL) {
		igen_irq_teardown(sc);
		igen_re_sysctls_fini(sc);
		kms_connector_cleanup(&sc->connector);
		kms_encoder_cleanup(&sc->encoder);
		kms_plane_cleanup(&sc->primary);
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

static device_method_t igen_methods[] = {
	DEVMETHOD(device_probe,		igen_probe),
	DEVMETHOD(device_attach,	igen_attach),
	DEVMETHOD(device_detach,	igen_detach),
	DEVMETHOD_END
};

static driver_t igen_driver_t = {
	"igen",
	igen_methods,
	sizeof(struct igen_softc),
};

DRIVER_MODULE(igen, pci, igen_driver_t, 0, 0);
MODULE_VERSION(igen, 1);
MODULE_DEPEND(igen, kms, 1, 1, 1);
MODULE_DEPEND(igen, pci, 1, 1, 1);
