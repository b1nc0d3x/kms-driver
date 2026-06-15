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

#include "igen9_internal.h"

#include <vm/vm_page.h>

#define	INTEL_PCI_VENDOR	0x8086

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
} igen9_ids[] = {
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

/*
 * DPRINTF, USER_FB_GTT_*, struct igen9_owned_fb, struct igen9_softc, and
 * the inline igen9_r32 / igen9_w32 accessors all live in
 * igen9_internal.h so both compilation units share them.
 */

static void igen9_owned_fb_destroy(struct drm_framebuffer *fb);
static const struct drm_framebuffer_funcs igen9_owned_fb_funcs;

/*
 * MMIO ranges we care about on gen9 display.  Bracket the regions, not
 * the full 16 MiB BAR — saves snapshot/diff memory and keeps the diff
 * signal-to-noise high.  Add ranges here as bring-up uncovers new state.
 *
 * Stride is 4 bytes (32-bit registers) for every range; gen9 doesn't have
 * any 8/16-bit-only MMIO that matters for display.
 */
struct igen9_range {
	uint32_t	start;
	uint32_t	end;	/* inclusive */
	const char	*name;
};

static const struct igen9_range igen9_ranges[] = {
	{ 0x00044000, 0x00044100, "INT/HOTPLUG" },
	{ 0x00045000, 0x000455ff, "PWR/DC_STATE" },
	{ 0x00046000, 0x000460ff, "CDCLK/DPLL_CTRL" },
	{ 0x0006c000, 0x0006c0ff, "AUDIO_PIN" },
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

/* struct igen9_softc + igen9_r32/igen9_w32 live in igen9_internal.h. */

/* ----------------------------- MMIO RE helpers ---------------------------- */

/*
 * Compute how many 32-bit words a full snapshot of igen9_ranges[]
 * occupies.  Caller-relative offset of register `addr` within the
 * snapshot is the prefix-sum walk in igen9_snapshot_index().
 */
static size_t
igen9_snapshot_total_words(void)
{
	size_t words = 0;

	for (size_t i = 0; i < nitems(igen9_ranges); i++) {
		words += (igen9_ranges[i].end -
		    igen9_ranges[i].start) / 4 + 1;
	}
	return (words);
}

/*
 * Return the index into sc->snapshot[] corresponding to MMIO offset
 * `addr`, or -1 if `addr` isn't in any tracked range.  O(N) over the
 * range table — N is ~20 so this is fine in sysctl/debug paths.
 */
static ssize_t
igen9_snapshot_index(uint32_t addr)
{
	size_t base = 0;

	for (size_t i = 0; i < nitems(igen9_ranges); i++) {
		const struct igen9_range *r = &igen9_ranges[i];
		size_t words = (r->end - r->start) / 4 + 1;

		if (addr >= r->start && addr <= r->end)
			return ((ssize_t)(base + (addr - r->start) / 4));
		base += words;
	}
	return (-1);
}

static void
igen9_snapshot_save(struct igen9_softc *sc)
{
	sx_xlock(&sc->re_lock);
	if (sc->snapshot == NULL) {
		sc->snapshot_words = igen9_snapshot_total_words();
		sc->snapshot = malloc(sc->snapshot_words * sizeof(uint32_t),
		    M_KMS, M_WAITOK | M_ZERO);
	}
	size_t idx = 0;
	for (size_t i = 0; i < nitems(igen9_ranges); i++) {
		const struct igen9_range *r = &igen9_ranges[i];
		for (uint32_t a = r->start; a <= r->end; a += 4)
			sc->snapshot[idx++] = igen9_r32(sc, a);
	}
	sc->snapshot_valid = true;
	sx_xunlock(&sc->re_lock);
	device_printf(sc->dev, "snapshot saved (%zu words / %zu bytes)\n",
	    sc->snapshot_words, sc->snapshot_words * sizeof(uint32_t));
}

static void
igen9_snapshot_diff(struct igen9_softc *sc)
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
	for (size_t i = 0; i < nitems(igen9_ranges); i++) {
		const struct igen9_range *r = &igen9_ranges[i];
		for (uint32_t a = r->start; a <= r->end; a += 4, idx++) {
			uint32_t cur = igen9_r32(sc, a);

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
igen9_bit_scan(struct igen9_softc *sc)
{
	uint32_t addr = sc->bit_scan_addr;
	uint32_t orig, observed, bit_mask;

	if (igen9_snapshot_index(addr) < 0) {
		device_printf(sc->dev,
		    "bit_scan: 0x%08x not in any tracked range\n", addr);
		return;
	}
	igen9_snapshot_save(sc);
	sx_xlock(&sc->re_lock);
	orig = igen9_r32(sc, addr);
	device_printf(sc->dev,
	    "bit_scan @0x%08x: orig=0x%08x (will toggle 32 bits)\n",
	    addr, orig);
	for (int bit = 0; bit < 32; bit++) {
		if (sc->bit_scan_skip & (1u << bit))
			continue;
		bit_mask = 1u << bit;
		igen9_w32(sc, addr, orig ^ bit_mask);
		observed = igen9_r32(sc, addr);
		device_printf(sc->dev,
		    "  bit %2d: wrote 0x%08x, readback 0x%08x %s\n",
		    bit, orig ^ bit_mask, observed,
		    ((observed ^ orig) & bit_mask) ? "RW" : "RO/clamped");
		igen9_w32(sc, addr, orig);
	}
	sx_xunlock(&sc->re_lock);
	device_printf(sc->dev, "bit_scan done; running side-effect diff:\n");
	igen9_snapshot_diff(sc);
}

static int
igen9_sysctl_snapshot_save(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL)
		return (error);
	if (trigger != 0)
		igen9_snapshot_save(sc);
	return (0);
}

static int
igen9_sysctl_snapshot_diff(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL)
		return (error);
	if (trigger != 0)
		igen9_snapshot_diff(sc);
	return (0);
}

static int
igen9_sysctl_mmio_read(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	uint32_t addr = sc->poke_addr;
	uint32_t val;
	int error;

	val = igen9_r32(sc, addr);
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (req->newptr != NULL)
		return (EPERM);
	return (error);
}

static int
igen9_sysctl_mmio_write(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	uint32_t val = 0;
	int error = sysctl_handle_int(oidp, &val, 0, req);

	if (error || req->newptr == NULL)
		return (error);
	device_printf(sc->dev, "mmio_write: 0x%08x <- 0x%08x\n",
	    sc->poke_addr, val);
	igen9_w32(sc, sc->poke_addr, val);
	return (0);
}

static int
igen9_sysctl_bit_scan(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL)
		return (error);
	if (trigger != 0)
		igen9_bit_scan(sc);
	return (0);
}

/* igen9_sysctl_edid_read_b lives in igen9_gmbus.c. */
static int	igen9_sysctl_vbt_dump(SYSCTL_HANDLER_ARGS);
static int	igen9_sysctl_hpd_dump(SYSCTL_HANDLER_ARGS);
static int	igen9_sysctl_cap_dump(SYSCTL_HANDLER_ARGS);
/* DPLL/WRPLL/pw1 sysctl handlers live in igen9_dpll.c. */
static int	igen9_sysctl_current_mode(SYSCTL_HANDLER_ARGS);
static int	igen9_sysctl_gtt_dump(SYSCTL_HANDLER_ARGS);
static int	igen9_sysctl_gtt_alloc_test(SYSCTL_HANDLER_ARGS);
static int	igen9_sysctl_test_fb_make(SYSCTL_HANDLER_ARGS);
static int	igen9_sysctl_test_fb_flip(SYSCTL_HANDLER_ARGS);
static int	igen9_sysctl_scanout_hold(SYSCTL_HANDLER_ARGS);
static int	igen9_sysctl_expose_scanout_fb(SYSCTL_HANDLER_ARGS);
static void	igen9_edid_to_mode(const uint8_t *dtd,
		    struct drm_display_mode *m);
static int	igen9_attach_edid_modes(struct igen9_softc *sc);

static void
igen9_re_sysctls_init(struct igen9_softc *sc)
{
	struct sysctl_oid_list *children;

	sx_init(&sc->re_lock, "igen9_re");
	sysctl_ctx_init(&sc->re_sysctl_ctx);
	sc->re_sysctl_tree = SYSCTL_ADD_NODE(&sc->re_sysctl_ctx,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev)),
	    OID_AUTO, "re", CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
	    "MMIO reverse-engineering scaffold");
	children = SYSCTL_CHILDREN(sc->re_sysctl_tree);

	/*
	 * Debug verbosity at the device root (not under .re/) so the
	 * tunable name dev.igen9.<n>.debug matches the cross-driver
	 * convention.  CTLFLAG_RWTUN makes it settable both at boot via
	 * loader.conf and at runtime via sysctl.
	 */
	SYSCTL_ADD_INT(&sc->re_sysctl_ctx,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev)),
	    OID_AUTO, "debug", CTLFLAG_RWTUN, &sc->sc_debug, 0,
	    "Debug verbosity: 0=silent, 1=milestones, 2=protocol,"
	    " 3=per-frame, 4=hex");

	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "mmio_snapshot_save",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_snapshot_save, "I",
	    "write 1 to snapshot all tracked MMIO ranges");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "mmio_snapshot_diff",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_snapshot_diff, "I",
	    "write 1 to log changes since last snapshot");
	SYSCTL_ADD_UINT(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "mmio_addr", CTLFLAG_RW, &sc->poke_addr, 0,
	    "MMIO byte-offset for mmio_read / mmio_write");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "mmio_read",
	    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_mmio_read, "IU",
	    "read [mmio_addr] (32-bit)");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "mmio_write",
	    CTLTYPE_UINT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_mmio_write, "IU",
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
	    sc, 0, igen9_sysctl_bit_scan, "I",
	    "write 1 to scan bit_scan_addr and diff side-effects");
	/* edid_read_b sysctl is owned by igen9_gmbus.c. */
	igen9_gmbus_register_sysctls(sc);

	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "vbt_dump",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_vbt_dump, "I",
	    "write 1 to map OpRegion via ASLS and walk VBT child devices");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "hpd_dump",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_hpd_dump, "I",
	    "write 1 to dump SFUSE_STRAP / SHOTPLUG_CTL_DDI / SDEISR live HPD");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "cap_dump",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_cap_dump, "I",
	    "write 1 to print per-DDI capability table (VBT x SFUSE x HPD)");
	/*
	 * DPLL / WRPLL / clock_state / try_pipe_resume / pw1_up sysctls
	 * are owned by igen9_dpll.c.
	 */
	igen9_dpll_register_sysctls(sc);

	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "current_mode",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_current_mode, "I",
	    "write 1 to read back live pipe/transcoder timing as a mode");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "gtt_dump",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_gtt_dump, "I",
	    "write 1 to scan first 2048 GTT PTEs and print first 8");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "gtt_alloc_test",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_gtt_alloc_test, "I",
	    "write 1 to alloc 1 page, map at GTT[2048], read back, free");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "test_fb_make",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_test_fb_make, "I",
	    "write 1 to alloc + GTT-map + checker-fill an 8 MiB 1920x1080 FB");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "test_fb_flip",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_test_fb_flip, "I",
	    "write N (seconds 1..10) to flip PLANE_SURF to our checker FB"
	    " then restore");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "scanout_hold",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_scanout_hold, "I",
	    "1 = checker, 2 = diagnostic gradient+grid, both flip PLANE_SURF"
	    " and HOLD;  0 = restore firmware FB + free");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "expose_scanout_fb",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen9_sysctl_expose_scanout_fb, "I",
	    "write 1 to allocate + register a driver-owned drm_framebuffer"
	    " for userspace MODE_ATOMIC; prints FB_ID, CRTC_ID, PLANE_ID");
	SYSCTL_ADD_U64(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "vblank_a_count", CTLFLAG_RD, &sc->vblank_count_pipe_a, 0,
	    "Pipe A vblanks observed via IRQ");
	SYSCTL_ADD_U64(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "irq_total_count", CTLFLAG_RD, &sc->irq_total_count, 0,
	    "total display-engine IRQs serviced");
}

static void
igen9_re_sysctls_fini(struct igen9_softc *sc)
{
	sysctl_ctx_free(&sc->re_sysctl_ctx);
	if (sc->snapshot != NULL) {
		free(sc->snapshot, M_KMS);
		sc->snapshot = NULL;
		sc->snapshot_valid = false;
	}
	sx_destroy(&sc->re_lock);
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
igen9_dvo_port_name(uint8_t p)
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
igen9_device_type_name(uint16_t t)
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
igen9_sysctl_vbt_dump(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
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
					    igen9_device_type_name(
						cd->device_type),
					    cd->dvo_port,
					    igen9_dvo_port_name(
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

/* ------------------------------- GTT -------------------------------------- */

/*
 * Graphics Translation Table — lives at BAR0 + 0x800000 on gen9.  Each
 * PTE is 8 bytes (64-bit), gen8+ layout:
 *   bit 0:        VALID/PRESENT
 *   bit 1:        WRITEABLE
 *   bits[3:2]:    PAT_INDEX
 *   bit 11:       LLC_CACHEABLE
 *   bits[63:12]:  physical page frame number (PFN)
 *
 * 8 MiB / 8 B per entry = 1 Mi entries × 4 KiB page = 4 GiB GTT-addressable.
 */
#define	GTT_BASE		0x00800000
#define	GTT_PTE_SIZE		8
#define	GTT_PTE_VALID		(1u << 0)
#define	GTT_PTE_WRITEABLE	(1u << 1)

/*
 * Primary-plane registers needed for the page-flip path.  PLANE_SURF is
 * the armed (next-vblank) surface offset; PLANE_SURFLIVE reads back the
 * surface address HW is currently scanning out.  PIPE_FRMCOUNT is the
 * vblank-incremented frame counter — a sanity check that the pipe is
 * actually running (not just that we wrote a register).
 */
#define	PLANE_SURF(p)		(0x7019c + (p) * 0x1000)
#define	PLANE_SURFLIVE(p)	(0x701ac + (p) * 0x1000)
#define	PIPE_FRMCOUNT(p)	(0x70040 + (p) * 0x1000)

static uint64_t
igen9_gtt_read(struct igen9_softc *sc, uint32_t entry_idx)
{
	uint32_t off = GTT_BASE + entry_idx * GTT_PTE_SIZE;
	uint32_t lo = igen9_r32(sc, off);
	uint32_t hi = igen9_r32(sc, off + 4);
	return ((uint64_t)hi << 32) | lo;
}

static void
igen9_gtt_write(struct igen9_softc *sc, uint32_t entry_idx,
    uint64_t pte)
{
	uint32_t off = GTT_BASE + entry_idx * GTT_PTE_SIZE;
	igen9_w32(sc, off, (uint32_t)pte);
	igen9_w32(sc, off + 4, (uint32_t)(pte >> 32));
}

/*
 * Owned framebuffer: contiguous kernel buffer + GTT mapping at a chosen
 * offset.  Lives on the softc so a follow-up page-flip sysctl can swap
 * PLANE_SURF to test_fb.gtt_offset and back to 0.
 */
struct igen9_test_fb {
	void		*va;
	vm_paddr_t	 pa;
	size_t		 size;
	uint32_t	 gtt_first_idx;
	uint32_t	 gtt_count;
	uint32_t	 width;
	uint32_t	 height;
	uint32_t	 stride;	/* bytes */
	bool		 mapped;
};

/* Test-FB lives at GTT[0x80000..] — same safe-past-firmware zone. */
#define	TEST_FB_GTT_FIRST	0x80000
/*
 * USER_FB_GTT_* are forward-declared near the softc.  Each slot is
 * 2048 entries (8 MiB scanout buffer) so a 1920x1080 XR24 FB fits in
 * one slot.  Per-fb slot cache lives on the softc; 3-buffer animation
 * gets 3 distinct PLANE_SURF addresses and HW never re-maps a live
 * scanout buffer.
 */

/*
 * Bind a generic drm_framebuffer (whose GEM object holds the backing
 * pages) into the GTT so HW can scan from it.  Walks ps->fb->gem_objs[0]
 * page array, writes one GTT PTE per page at USER_FB_GTT_FIRST.
 * Returns the GTT byte offset usable in PLANE_SURF, or 0 on error.
 *
 * Caller-owned: we don't unmap on FB destroy because the same range is
 * reused for the next user FB.  Single active scanout buffer at a time.
 */
static uint32_t
igen9_gtt_bind_user_fb(struct igen9_softc *sc,
    struct drm_framebuffer *fb)
{
	struct drm_gem_object *obj = fb->gem_objs[0];

	if (obj == NULL || obj->pages == NULL || obj->npages == 0)
		return (0);
	if (obj->npages > USER_FB_GTT_SLOT_PAGES) {
		device_printf(sc->dev,
		    "gtt_bind: FB too big (%u pages, slot=%u)\n",
		    (unsigned)obj->npages, USER_FB_GTT_SLOT_PAGES);
		return (0);
	}

	/* Cache hit: reuse the slot we already mapped this FB into. */
	for (uint32_t i = 0; i < USER_FB_GTT_NSLOTS; i++)
		if (sc->user_fb_slots[i].fb == fb)
			return (sc->user_fb_slots[i].surf);

	/* Find empty slot; if none, round-robin evict. */
	uint32_t slot = USER_FB_GTT_NSLOTS;
	for (uint32_t i = 0; i < USER_FB_GTT_NSLOTS; i++)
		if (sc->user_fb_slots[i].fb == NULL) { slot = i; break; }
	if (slot == USER_FB_GTT_NSLOTS) {
		slot = sc->user_fb_next_slot;
		sc->user_fb_next_slot =
		    (sc->user_fb_next_slot + 1) % USER_FB_GTT_NSLOTS;
	}

	uint32_t first_idx = USER_FB_GTT_FIRST +
	    slot * USER_FB_GTT_SLOT_PAGES;
	for (size_t i = 0; i < obj->npages; i++) {
		vm_paddr_t pa = VM_PAGE_TO_PHYS(obj->pages[i]);
		uint64_t pte = (pa & ~0xfffULL) | GTT_PTE_VALID |
		    GTT_PTE_WRITEABLE;
		igen9_gtt_write(sc, first_idx + i, pte);
	}
	uint32_t surf = first_idx * PAGE_SIZE;
	sc->user_fb_slots[slot].fb = fb;
	sc->user_fb_slots[slot].surf = surf;
	return (surf);
}

static int
igen9_test_fb_alloc(struct igen9_softc *sc,
    struct igen9_test_fb *fb, uint32_t w, uint32_t h)
{
	fb->width = w;
	fb->height = h;
	fb->stride = w * 4;
	fb->size = (size_t)fb->stride * h;
	fb->size = (fb->size + PAGE_SIZE - 1) & ~(size_t)(PAGE_SIZE - 1);
	fb->gtt_count = fb->size / PAGE_SIZE;
	fb->gtt_first_idx = TEST_FB_GTT_FIRST;

	fb->va = contigmalloc(fb->size, M_KMS, M_WAITOK | M_ZERO,
	    0, ~(vm_paddr_t)0, PAGE_SIZE, 0);
	if (fb->va == NULL)
		return (ENOMEM);
	fb->pa = pmap_kextract((vm_offset_t)fb->va);

	for (uint32_t i = 0; i < fb->gtt_count; i++) {
		uint64_t pte = ((fb->pa + (uint64_t)i * PAGE_SIZE) & ~0xfffULL)
		    | GTT_PTE_VALID | GTT_PTE_WRITEABLE;
		igen9_gtt_write(sc, fb->gtt_first_idx + i, pte);
	}
	fb->mapped = true;
	return (0);
}

static void
igen9_test_fb_free(struct igen9_softc *sc,
    struct igen9_test_fb *fb)
{
	if (fb->mapped) {
		for (uint32_t i = 0; i < fb->gtt_count; i++)
			igen9_gtt_write(sc, fb->gtt_first_idx + i, 0);
		fb->mapped = false;
	}
	if (fb->va != NULL) {
		contigfree(fb->va, fb->size, M_KMS);
		fb->va = NULL;
	}
}

/* 64×64 checkerboard, two-color, in XRGB8888. */
static void
igen9_test_fb_fill_checker(struct igen9_test_fb *fb,
    uint32_t color_a, uint32_t color_b)
{
	uint32_t *px = (uint32_t *)fb->va;
	uint32_t row_stride_px = fb->stride / 4;

	for (uint32_t y = 0; y < fb->height; y++) {
		for (uint32_t x = 0; x < fb->width; x++) {
			bool tile = ((x / 64) + (y / 64)) & 1;
			px[y * row_stride_px + x] = tile ? color_a : color_b;
		}
	}
}

/*
 * Self-describing diagnostic pattern in XRGB8888:
 *   - horizontal R gradient, vertical G gradient (proves color depth)
 *   - black grid every 100 px (proves x/y addressing + stride)
 *   - solid white 32-px squares in each corner (anchors orientation)
 *   - solid red 64-px band along top, green 64-px band along bottom
 *     (immediately recognisable as "this is our buffer not the desktop")
 */
static void
igen9_test_fb_fill_diag(struct igen9_test_fb *fb)
{
	uint32_t *px = (uint32_t *)fb->va;
	uint32_t row_stride_px = fb->stride / 4;
	uint32_t w = fb->width;
	uint32_t h = fb->height;

	for (uint32_t y = 0; y < h; y++) {
		for (uint32_t x = 0; x < w; x++) {
			uint32_t r = (x * 255) / (w - 1);
			uint32_t g = (y * 255) / (h - 1);
			uint32_t color = (r << 16) | (g << 8);

			if ((x % 100) == 0 || (y % 100) == 0)
				color = 0;
			if (y < 64)
				color = 0x00ff0000;	/* top red band */
			else if (y >= h - 64)
				color = 0x0000ff00;	/* bottom green band */
			if ((x < 32 && y < 32) ||
			    (x >= w - 32 && y < 32) ||
			    (x < 32 && y >= h - 32) ||
			    (x >= w - 32 && y >= h - 32))
				color = 0x00ffffff;	/* corner anchors */
			px[y * row_stride_px + x] = color;
		}
	}
}

/*
 * Animation kthread.  Owns the held FB while it's alive: each tick
 * stamps a moving white dot onto the FB at a new position, demonstrating
 * that CPU writes through the kernel mapping reach physical memory that
 * HW reads via GTT.  Sleeps 16 ms between frames (~60 Hz) to align with
 * scanout rate.
 */
static void
igen9_anim_thread(void *arg)
{
	struct igen9_softc *sc = arg;
	uint32_t frame = 0;
	uint32_t prev_x = 0, prev_y = 0;

	while (!sc->anim_stop && sc->scanout_held && sc->scanout_fb != NULL) {
		struct igen9_test_fb *fb = sc->scanout_fb;
		uint32_t *px = (uint32_t *)fb->va;
		uint32_t row_stride_px = fb->stride / 4;

		/* Lissajous figure scaled to interior of FB minus 64-px banding. */
		uint32_t cx = fb->width  / 2;
		uint32_t cy = fb->height / 2;
		uint32_t r1 = (fb->width  / 2) - 100;
		uint32_t r2 = (fb->height / 2) - 100;
		uint32_t t = frame;
		int sin_t1 = (int)((int64_t)r1 *
		    (int)((t * 3) % 360) / 180 - r1);
		int sin_t2 = (int)((int64_t)r2 *
		    (int)((t * 5) % 360) / 180 - r2);
		uint32_t x = cx + sin_t1;
		uint32_t y = cy + sin_t2;
		if (x >= fb->width)  x = fb->width - 32;
		if (y >= fb->height) y = fb->height - 32;

		/* Erase previous 32x32 dot. */
		for (uint32_t dy = 0; dy < 32 && (prev_y + dy) < fb->height; dy++) {
			for (uint32_t dx = 0; dx < 32 && (prev_x + dx) < fb->width; dx++)
				px[(prev_y + dy) * row_stride_px + (prev_x + dx)] =
				    0x00202020;	/* dark grey trail */
		}
		/* Draw new dot. */
		for (uint32_t dy = 0; dy < 32 && (y + dy) < fb->height; dy++) {
			for (uint32_t dx = 0; dx < 32 && (x + dx) < fb->width; dx++)
				px[(y + dy) * row_stride_px + (x + dx)] = 0x00ffffff;
		}
		prev_x = x; prev_y = y;
		frame++;
		pause("gen9ani", hz / 60);
	}
	sc->anim_active = false;
	kthread_exit();
}

static void
igen9_anim_start(struct igen9_softc *sc)
{
	if (sc->anim_active)
		return;
	sc->anim_stop = false;
	sc->anim_active = true;
	if (kthread_add(igen9_anim_thread, sc, NULL, &sc->anim_td,
	    0, 0, "gen9anim") != 0) {
		sc->anim_active = false;
		device_printf(sc->dev, "anim: kthread_add failed\n");
	}
}

static void
igen9_anim_stop(struct igen9_softc *sc)
{
	if (!sc->anim_active)
		return;
	sc->anim_stop = true;
	while (sc->anim_active)
		pause("gen9axw", hz / 100);
}

/*
 * expose_scanout_fb: allocate a test_fb (if not already there), wrap it
 * in a driver-owned drm_framebuffer, and register with the framework
 * so userspace can reference it by FB_ID in an atomic_commit.  Prints
 * the assigned FB_ID + the CRTC + plane IDs needed to use it.
 *
 * One-shot: subsequent triggers print the existing FB_ID without
 * allocating again.  Cleared on detach.
 */
static struct igen9_owned_fb *igen9_exposed_fb = NULL;

static int
igen9_sysctl_expose_scanout_fb(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	if (igen9_exposed_fb != NULL) {
		device_printf(sc->dev,
		    "expose_scanout_fb: already exposed as FB_ID=%u"
		    "  CRTC_ID=%u  PLANE_ID=%u\n",
		    igen9_exposed_fb->base.base.id,
		    sc->crtc.base.id, sc->primary.base.id);
		return (0);
	}

	struct igen9_owned_fb *ofb = malloc(sizeof(*ofb), M_KMS,
	    M_WAITOK | M_ZERO);
	struct igen9_test_fb *tfb = malloc(sizeof(*tfb), M_KMS,
	    M_WAITOK | M_ZERO);
	error = igen9_test_fb_alloc(sc, tfb, 1920, 1080);
	if (error != 0) {
		device_printf(sc->dev,
		    "expose_scanout_fb: alloc failed %d\n", error);
		free(tfb, M_KMS);
		free(ofb, M_KMS);
		return (0);
	}
	igen9_test_fb_fill_diag(tfb);
	ofb->test_fb = tfb;

	ofb->base.width  = tfb->width;
	ofb->base.height = tfb->height;
	ofb->base.format = 0x34325258;	/* XR24 */
	ofb->base.pitches[0] = tfb->stride;
	error = kms_framebuffer_init(sc->drm_dev, &ofb->base,
	    &igen9_owned_fb_funcs);
	if (error != 0) {
		device_printf(sc->dev,
		    "expose_scanout_fb: framebuffer_init failed %d\n", error);
		igen9_test_fb_free(sc, tfb);
		free(tfb, M_KMS);
		free(ofb, M_KMS);
		return (0);
	}
	igen9_exposed_fb = ofb;

	device_printf(sc->dev,
	    "expose_scanout_fb: exposed FB_ID=%u  CRTC_ID=%u  PLANE_ID=%u\n",
	    ofb->base.base.id, sc->crtc.base.id, sc->primary.base.id);
	device_printf(sc->dev,
	    "  format=XR24  pitch=%u  size=%ux%u  GTT byte offset=0x%llx\n",
	    tfb->stride, tfb->width, tfb->height,
	    (unsigned long long)tfb->gtt_first_idx * PAGE_SIZE);
	return (0);
}

/*
 * scanout_hold: persistent flip.  Write 1 to allocate + flip; write 0
 * to restore + free.  Lets userspace observe arbitrary work happening
 * over the static checker buffer.  Idempotent in both directions.
 */
static int
igen9_sysctl_scanout_hold(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	int hold = sc->scanout_held ? 1 : 0;
	int error = sysctl_handle_int(oidp, &hold, 0, req);

	if (error || req->newptr == NULL)
		return (error);

	if (hold && !sc->scanout_held) {
		sc->scanout_fb = malloc(sizeof(*sc->scanout_fb),
		    M_KMS, M_WAITOK | M_ZERO);
		error = igen9_test_fb_alloc(sc, sc->scanout_fb,
		    1920, 1080);
		if (error != 0) {
			device_printf(sc->dev,
			    "scanout_hold: alloc failed: %d\n", error);
			free(sc->scanout_fb, M_KMS);
			sc->scanout_fb = NULL;
			return (0);
		}
		/*
		 * hold value picks pattern:
		 *   1 -> checker (red/blue)
		 *   2 -> self-describing diagnostic gradient + grid
		 *   3 -> diagnostic + animation kthread (moving dot)
		 * Anything else > 0 -> checker.
		 */
		if (hold == 2 || hold == 3)
			igen9_test_fb_fill_diag(sc->scanout_fb);
		else
			igen9_test_fb_fill_checker(sc->scanout_fb,
			    0x00ff0000, 0x000000ff);

		sc->scanout_prev_surf = igen9_r32(sc, PLANE_SURF(0));
		uint32_t new_surf = sc->scanout_fb->gtt_first_idx * PAGE_SIZE;
		igen9_w32(sc, PLANE_SURF(0), new_surf);
		sc->scanout_held = true;

		DPRINTF(sc, 0,
		    "scanout_hold: ON  PLANE_SURF 0x%08x -> 0x%08x\n",
		    sc->scanout_prev_surf, new_surf);

		if (hold == 3)
			igen9_anim_start(sc);
	} else if (!hold && sc->scanout_held) {
		igen9_anim_stop(sc);
		igen9_w32(sc, PLANE_SURF(0), sc->scanout_prev_surf);
		pause("gen9rst", hz / 20);
		igen9_test_fb_free(sc, sc->scanout_fb);
		free(sc->scanout_fb, M_KMS);
		sc->scanout_fb = NULL;
		sc->scanout_held = false;
		DPRINTF(sc, 0,
		    "scanout_hold: OFF  PLANE_SURF restored to 0x%08x\n",
		    sc->scanout_prev_surf);
	}
	return (0);
}

static int
igen9_sysctl_test_fb_flip(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	int hold_sec = 0;
	int error = sysctl_handle_int(oidp, &hold_sec, 0, req);

	if (error || req->newptr == NULL)
		return (error);
	if (hold_sec <= 0)
		return (0);
	if (hold_sec > 10)
		hold_sec = 10;

	struct igen9_test_fb fb = { 0 };
	error = igen9_test_fb_alloc(sc, &fb, 1920, 1080);
	if (error != 0) {
		device_printf(sc->dev, "test_fb_flip: alloc failed: %d\n",
		    error);
		return (0);
	}
	igen9_test_fb_fill_checker(&fb, 0x00ff0000, 0x000000ff);

	/*
	 * Flip plane 1 of pipe A to scan from our buffer.  Our FB matches
	 * the firmware-programmed scanout in every observable parameter
	 * (1920x1080, XRGB8888, linear, stride 7680) so a single PLANE_SURF
	 * write is the entire change.  PLANE_SURF is the "armed" register —
	 * the change takes effect at the next vblank.
	 */
	uint32_t prev_surf = igen9_r32(sc, PLANE_SURF(0));
	uint32_t new_surf  = fb.gtt_first_idx * PAGE_SIZE;
	uint32_t live_before = igen9_r32(sc, PLANE_SURFLIVE(0));
	uint32_t frm_before  = igen9_r32(sc, PIPE_FRMCOUNT(0));

	device_printf(sc->dev,
	    "test_fb_flip: PLANE_SURF 0x%08x -> 0x%08x (hold %d s)\n",
	    prev_surf, new_surf, hold_sec);
	device_printf(sc->dev,
	    "  pre:   SURFLIVE=0x%08x  FRMCOUNT=%u\n",
	    live_before, frm_before);

	igen9_w32(sc, PLANE_SURF(0), new_surf);

	/*
	 * Sample SURFLIVE shortly after the arm to confirm HW latched it
	 * at the next vblank.  At 60 Hz a vblank arrives every ~16.7 ms;
	 * 50 ms of pause is plenty.
	 */
	pause("gen9arm", hz / 20);
	uint32_t live_armed = igen9_r32(sc, PLANE_SURFLIVE(0));
	device_printf(sc->dev,
	    "  armed: SURFLIVE=0x%08x  %s\n", live_armed,
	    ((live_armed & 0xfffff000) == new_surf) ?
	    "FLIP TOOK" : "FLIP DID NOT TAKE");

	pause("gen9flp", hold_sec * hz);
	uint32_t live_end = igen9_r32(sc, PLANE_SURFLIVE(0));
	uint32_t frm_end  = igen9_r32(sc, PIPE_FRMCOUNT(0));

	igen9_w32(sc, PLANE_SURF(0), prev_surf);
	pause("gen9rst", hz / 20);
	uint32_t live_restored = igen9_r32(sc, PLANE_SURFLIVE(0));

	device_printf(sc->dev,
	    "  during: SURFLIVE=0x%08x  FRMCOUNT=%u  (advanced %u frames)\n",
	    live_end, frm_end, frm_end - frm_before);
	device_printf(sc->dev,
	    "  after restore: SURFLIVE=0x%08x  %s\n",
	    live_restored,
	    ((live_restored & 0xfffff000) == (prev_surf & 0xfffff000)) ?
	    "RESTORED OK" : "RESTORE FAILED");
	igen9_test_fb_free(sc, &fb);
	return (0);
}

static int
igen9_sysctl_test_fb_make(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	struct igen9_test_fb fb = { 0 };
	error = igen9_test_fb_alloc(sc, &fb, 1920, 1080);
	if (error != 0) {
		device_printf(sc->dev,
		    "test_fb: alloc failed (%d) — need %u KiB contig\n",
		    error, 1920 * 1080 * 4 / 1024);
		return (0);
	}
	igen9_test_fb_fill_checker(&fb, 0x00ff0000, 0x000000ff);

	uint32_t *first = (uint32_t *)fb.va;
	device_printf(sc->dev,
	    "test_fb: %ux%u stride=%u size=%zu KiB  va=%p  pa=0x%llx\n",
	    fb.width, fb.height, fb.stride, fb.size / 1024,
	    fb.va, (unsigned long long)fb.pa);
	device_printf(sc->dev,
	    "  GTT mapped at [%u..%u]  GTT byte offset=0x%llx\n",
	    fb.gtt_first_idx, fb.gtt_first_idx + fb.gtt_count - 1,
	    (unsigned long long)fb.gtt_first_idx * PAGE_SIZE);
	device_printf(sc->dev,
	    "  first 4 px: 0x%08x 0x%08x 0x%08x 0x%08x\n",
	    first[0], first[1], first[2], first[3]);
	device_printf(sc->dev,
	    "  GTT[%u] readback = 0x%016llx (expect VALID+WRITEABLE + pa)\n",
	    fb.gtt_first_idx,
	    (unsigned long long)igen9_gtt_read(sc, fb.gtt_first_idx));

	igen9_test_fb_free(sc, &fb);
	device_printf(sc->dev, "test_fb: freed cleanly\n");
	return (0);
}

/*
 * Allocate one wired kernel page, map it via GTT entry `idx`, read the
 * PTE back to prove the GTT is writable, then free the page (clearing
 * the PTE first).  This is the smoke-test that the GTT write path
 * works before we build the page-flip logic on top.
 */
static int
igen9_sysctl_gtt_alloc_test(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	/*
	 * Pick an index well past anything firmware might be using.  The
	 * full GTT covers 4 GiB / 4 KiB = 1Mi entries; index 0x80000 is at
	 * 2 GiB into the GTT-mapped address space, way beyond any plausible
	 * scanout / engine context reservation.
	 */
	const uint32_t test_idx = 0x80000;
	void *va = contigmalloc(PAGE_SIZE, M_KMS, M_WAITOK | M_ZERO,
	    0, ~(vm_paddr_t)0, PAGE_SIZE, 0);
	if (va == NULL) {
		device_printf(sc->dev, "gtt_alloc_test: contigmalloc failed\n");
		return (0);
	}
	vm_paddr_t pa = pmap_kextract((vm_offset_t)va);
	uint64_t pte = (pa & ~0xfffULL) | GTT_PTE_VALID | GTT_PTE_WRITEABLE;

	uint64_t before = igen9_gtt_read(sc, test_idx);
	igen9_gtt_write(sc, test_idx, pte);
	uint64_t after = igen9_gtt_read(sc, test_idx);

	device_printf(sc->dev,
	    "gtt_alloc_test: va=%p  pa=0x%llx  wrote PTE=0x%llx\n",
	    va, (unsigned long long)pa, (unsigned long long)pte);
	device_printf(sc->dev,
	    "  GTT[%u] before=0x%016llx  after=0x%016llx  %s\n",
	    test_idx,
	    (unsigned long long)before, (unsigned long long)after,
	    (after == pte) ? "RW OK" : "MISMATCH");

	/*
	 * Restore the original PTE before freeing the page so HW can't
	 * dangling-ref it AND any firmware mapping that happened to live
	 * here stays intact.
	 */
	igen9_gtt_write(sc, test_idx, before);
	contigfree(va, PAGE_SIZE, M_KMS);
	return (0);
}

static int
igen9_sysctl_gtt_dump(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	uint32_t valid_count = 0, last_pfn = 0, runs = 0;
	uint64_t first_pfn = 0;
	for (uint32_t i = 0; i < 2048; i++) {	/* first 8 MiB of GTT */
		uint64_t pte = igen9_gtt_read(sc, i);
		if (pte & GTT_PTE_VALID) {
			uint64_t pfn = (pte >> 12);
			if (valid_count == 0)
				first_pfn = pfn;
			if (valid_count == 0 || pfn != last_pfn + 1)
				runs++;
			last_pfn = pfn;
			valid_count++;
		}
	}
	device_printf(sc->dev,
	    "gtt: in first 2048 PTEs: %u valid, %u contiguous runs,"
	    " first PFN=0x%llx (phys 0x%llx)\n",
	    valid_count, runs,
	    (unsigned long long)first_pfn,
	    (unsigned long long)(first_pfn << 12));

	/* Pretty-print the first 8 PTEs verbatim. */
	for (uint32_t i = 0; i < 8; i++) {
		uint64_t pte = igen9_gtt_read(sc, i);
		device_printf(sc->dev,
		    "  GTT[%u] = 0x%016llx  %s%s  PFN=0x%llx\n",
		    i, (unsigned long long)pte,
		    (pte & GTT_PTE_VALID) ? "V" : "-",
		    (pte & GTT_PTE_WRITEABLE) ? "W" : "-",
		    (unsigned long long)(pte >> 12));
	}
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
igen9_plane_format_name(uint32_t f)
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
igen9_plane_tiling_name(uint32_t t)
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
igen9_read_pipe_mode(struct igen9_softc *sc, int pipe,
    struct drm_display_mode *m)
{
	uint32_t htotal = igen9_r32(sc, TRANS_HTOTAL(pipe));
	uint32_t hsync  = igen9_r32(sc, TRANS_HSYNC(pipe));
	uint32_t vtotal = igen9_r32(sc, TRANS_VTOTAL(pipe));
	uint32_t vsync  = igen9_r32(sc, TRANS_VSYNC(pipe));
	uint32_t fctl   = igen9_r32(sc, TRANS_DDI_FUNC_CTL(pipe));

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
igen9_sysctl_current_mode(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	struct drm_display_mode m;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	for (int pipe = 0; pipe < 3; pipe++) {
		uint32_t pconf = igen9_r32(sc, PIPE_CONF(pipe));
		bool active = (pconf & (PIPE_CONF_ENABLE | PIPE_CONF_STATE))
		    == (PIPE_CONF_ENABLE | PIPE_CONF_STATE);
		device_printf(sc->dev,
		    "pipe %c: PIPE_CONF=0x%08x  %s\n",
		    'A' + pipe, pconf, active ? "ACTIVE" : "idle");
		if (!active)
			continue;
		igen9_read_pipe_mode(sc, pipe, &m);
		device_printf(sc->dev,
		    "  %ux%u  htotal=%u  vtotal=%u  hs=%u..%u  vs=%u..%u"
		    "  flags=0x%x\n",
		    m.hdisplay, m.vdisplay, m.htotal, m.vtotal,
		    m.hsync_start, m.hsync_end,
		    m.vsync_start, m.vsync_end, m.flags);

		uint32_t pctl  = igen9_r32(sc, PLANE_CTL(pipe));
		uint32_t psurf = igen9_r32(sc, PLANE_SURF(pipe));
		uint32_t pstr  = igen9_r32(sc, PLANE_STRIDE(pipe));
		uint32_t psize = igen9_r32(sc, PLANE_SIZE(pipe));
		uint32_t poff  = igen9_r32(sc, PLANE_OFFSET(pipe));
		uint32_t fmt   = (pctl & PLANE_CTL_FORMAT_MASK) >>
		    PLANE_CTL_FORMAT_SHIFT;
		uint32_t tile  = (pctl & PLANE_CTL_TILED_MASK) >>
		    PLANE_CTL_TILED_SHIFT;
		uint32_t w = (psize & 0x1fff) + 1;
		uint32_t h = ((psize >> 16) & 0x1fff) + 1;
		device_printf(sc->dev,
		    "  plane1: CTL=0x%08x  en=%d  fmt=%s  tile=%s\n",
		    pctl, !!(pctl & PLANE_CTL_ENABLE),
		    igen9_plane_format_name(fmt),
		    igen9_plane_tiling_name(tile));
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
static void
igen9_edid_to_mode(const uint8_t *d, struct drm_display_mode *m)
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

static int
igen9_attach_edid_modes(struct igen9_softc *sc)
{
	uint8_t edid[128];
	int error = EIO;

	/*
	 * Cold-boot state of GMBus often leaves INUSE/SW_CLR_INT stale
	 * enough that the first xfer NAKs.  Retry up to 4 times — by the
	 * 2nd attempt the bus is consistently warm.
	 */
	for (int try = 0; try < 4; try++) {
		error = igen9_gmbus_read_block(sc, GMBUS_PIN_DDI_B,
		    EDID_SLAVE, 0, edid, sizeof(edid));
		if (error == 0)
			break;
		DPRINTF(sc, 1,
		    "edid: GMBus read attempt %d failed (%d), retrying\n",
		    try + 1, error);
		DELAY(5000);
	}
	if (error != 0) {
		device_printf(sc->dev,
		    "edid: GMBus read gave up — connector stays UNKNOWN\n");
		return (error);
	}
	if (edid[0] != 0x00 || edid[1] != 0xff || edid[7] != 0x00) {
		device_printf(sc->dev, "edid: header invalid\n");
		return (EIO);
	}

	for (int i = 54; i <= 108; i += 18) {
		/* Skip non-DTD descriptors (pixclk == 0). */
		if (edid[i] == 0 && edid[i + 1] == 0)
			continue;
		struct drm_display_mode *m = kms_mode_create();
		if (m == NULL)
			return (ENOMEM);
		igen9_edid_to_mode(&edid[i], m);
		kms_connector_add_mode(&sc->connector, m);
		DPRINTF(sc, 0,
		    "edid: added mode %s @%u kHz  %u Hz  flags=0x%x\n",
		    m->name, m->clock, m->vrefresh, m->flags);
	}
	sc->connector.status = connector_status_connected;
	return (0);
}

/* ------------------------------- HPD -------------------------------------- */

/*
 * Live hot-plug detect status.  SKL+ has two register paths:
 *   - SFUSE_STRAP (0xc2014): bits[2:0] set per DDI present on package
 *     (fuse-set at boot; not real-time, just "this PORT exists")
 *   - SHOTPLUG_CTL_DDI (0xc4030): 4 bits per port [A,B,C,D,E]:
 *       bit hpd_pin*4+0: short-pulse seen
 *       bit hpd_pin*4+1: long-pulse seen (plug/unplug edge)
 *       bit hpd_pin*4+4: HPD irq enable
 *   - SDEISR (0xc4000): PCH interrupt status; DDI HPD live bits here too
 */

#define	SFUSE_STRAP		0x000c2014
#define	SHOTPLUG_CTL_DDI	0x000c4030
#define	SDEISR			0x000c4000

static int
igen9_sysctl_hpd_dump(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
	uint32_t sfuse, hot, sde;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	sfuse = igen9_r32(sc, SFUSE_STRAP);
	hot   = igen9_r32(sc, SHOTPLUG_CTL_DDI);
	sde   = igen9_r32(sc, SDEISR);

	device_printf(sc->dev,
	    "hpd: SFUSE_STRAP=0x%08x  SHOTPLUG_CTL_DDI=0x%08x  SDEISR=0x%08x\n",
	    sfuse, hot, sde);

	device_printf(sc->dev,
	    "  SFUSE_STRAP DDI present: B=%d C=%d D=%d\n",
	    (sfuse >> 2) & 1, (sfuse >> 1) & 1, (sfuse >> 0) & 1);

	/* SHOTPLUG_CTL_DDI: 4 bits per port; bit0..3=port A, 4..7=B, ... */
	for (int p = 0; p < 5; p++) {
		uint32_t f = (hot >> (p * 4)) & 0xf;
		device_printf(sc->dev,
		    "  SHOTPLUG_CTL DDI_%c: en=%d  short_pulse=%d  long_pulse=%d  raw=0x%x\n",
		    'A' + p, (f >> 4) & 1, f & 1, (f >> 1) & 1, f);
	}

	/* SDEISR HPD live bits (SKL+ desktop):
	 * bit 21 = DDI_B HPD live
	 * bit 22 = DDI_C
	 * bit 23 = DDI_D
	 * bit 24 = DDI_E
	 */
	device_printf(sc->dev,
	    "  SDEISR HPD live: B=%d C=%d D=%d E=%d\n",
	    (sde >> 21) & 1, (sde >> 22) & 1,
	    (sde >> 23) & 1, (sde >> 24) & 1);
	return (0);
}

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
igen9_sysctl_cap_dump(SYSCTL_HANDLER_ARGS)
{
	struct igen9_softc *sc = arg1;
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

	sfuse = igen9_r32(sc, SFUSE_STRAP);
	sde   = igen9_r32(sc, SDEISR);

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
		    igen9_device_type_name(per_ddi_type[ddi]) : "(none)";

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

static void
igen9_irq_handler(void *arg)
{
	struct igen9_softc *sc = arg;
	uint32_t master, master_w;

	master = igen9_r32(sc, GEN8_MASTER_IRQ);
	if ((master & GEN8_MASTER_IRQ_CONTROL) == 0)
		return;
	/* Disable master while servicing; re-enable at end. */
	master_w = master & ~GEN8_MASTER_IRQ_CONTROL;
	igen9_w32(sc, GEN8_MASTER_IRQ, master_w);

	sc->irq_total_count++;
	bool pipe_a_vblank = false;
	if (master & GEN8_DE_PIPE_A_IRQ) {
		uint32_t iir = igen9_r32(sc, GEN8_DE_PIPE_IIR(0));
		if (iir & GEN8_PIPE_VBLANK) {
			sc->vblank_count_pipe_a++;
			pipe_a_vblank = true;
		}
		igen9_w32(sc, GEN8_DE_PIPE_IIR(0), iir);
	}
	if (master & GEN8_DE_PIPE_B_IRQ) {
		uint32_t iir = igen9_r32(sc, GEN8_DE_PIPE_IIR(1));
		if (iir & GEN8_PIPE_VBLANK)
			sc->vblank_count_pipe_b++;
		igen9_w32(sc, GEN8_DE_PIPE_IIR(1), iir);
	}
	if (master & GEN8_DE_PIPE_C_IRQ) {
		uint32_t iir = igen9_r32(sc, GEN8_DE_PIPE_IIR(2));
		if (iir & GEN8_PIPE_VBLANK)
			sc->vblank_count_pipe_c++;
		igen9_w32(sc, GEN8_DE_PIPE_IIR(2), iir);
	}

	igen9_w32(sc, GEN8_MASTER_IRQ,
	    master_w | GEN8_MASTER_IRQ_CONTROL);
	(void)igen9_r32(sc, GEN8_MASTER_IRQ);	/* posting flush */

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
igen9_irq_setup(struct igen9_softc *sc)
{
	int msi_count = 1;
	int error;

	/*
	 * Quiesce: master off, per-pipe banks all masked / IIRs cleared.
	 * Pipes B/C stay fully off (no scanout there); Pipe A gets the
	 * vblank source unmasked + enabled after MSI is hooked.
	 */
	igen9_w32(sc, GEN8_MASTER_IRQ, 0);
	for (int p = 0; p < 3; p++) {
		igen9_w32(sc, GEN8_DE_PIPE_IMR(p), 0xffffffff);
		igen9_w32(sc, GEN8_DE_PIPE_IER(p), 0);
		igen9_w32(sc, GEN8_DE_PIPE_IIR(p), 0xffffffff);
	}
	(void)igen9_r32(sc, GEN8_MASTER_IRQ);

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
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, igen9_irq_handler, sc,
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
	igen9_w32(sc, GEN8_DE_PIPE_IIR(0), 0xffffffff);
	igen9_w32(sc, GEN8_DE_PIPE_IMR(0), ~GEN8_PIPE_VBLANK);
	igen9_w32(sc, GEN8_DE_PIPE_IER(0), GEN8_PIPE_VBLANK);
	igen9_w32(sc, GEN8_MASTER_IRQ,
	    GEN8_MASTER_IRQ_CONTROL | GEN8_DE_PIPE_A_IRQ);
	(void)igen9_r32(sc, GEN8_MASTER_IRQ);
	DPRINTF(sc, 0, "irq: MSI armed, Pipe A vblank enabled\n");
	return (0);
}

static void
igen9_irq_teardown(struct igen9_softc *sc)
{
	if (sc->irq_res == NULL)
		return;
	/* Master off, per-pipe banks masked + cleared. */
	igen9_w32(sc, GEN8_MASTER_IRQ, 0);
	for (int p = 0; p < 3; p++) {
		igen9_w32(sc, GEN8_DE_PIPE_IMR(p), 0xffffffff);
		igen9_w32(sc, GEN8_DE_PIPE_IER(p), 0);
		igen9_w32(sc, GEN8_DE_PIPE_IIR(p), 0xffffffff);
	}

	bus_teardown_intr(sc->dev, sc->irq_res, sc->irq_cookie);
	bus_release_resource(sc->dev, SYS_RES_IRQ, sc->irq_rid, sc->irq_res);
	if (sc->irq_rid == 1)
		pci_release_msi(sc->dev);
	sc->irq_res = NULL;
}

/* ----------------------------- driver glue -------------------------------- */

static const struct drm_driver igen9_driver = {
	.name		= "igen9",
	.desc		= "Intel Gen9 iGPU (kms framework)",
	.date		= "20260613",
	.major		= 0,
	.minor		= 1,
	.patchlevel	= 0,
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
static int
igen9_legacy_set_config(struct drm_mode_set *set)
{
	struct drm_crtc *crtc;

	if (set == NULL || (crtc = set->crtc) == NULL)
		return (EINVAL);

	if (set->mode == NULL) {
		crtc->mode_valid = 0;
		crtc->enabled = false;
		crtc->primary_fb = NULL;
		return (0);
	}

	crtc->mode = *set->mode;
	crtc->mode_valid = 1;
	crtc->enabled = true;
	crtc->primary_fb = set->fb;
	crtc->x = set->x;
	crtc->y = set->y;
	return (0);
}

static int
igen9_legacy_page_flip(struct drm_crtc *crtc, struct drm_framebuffer *fb,
    uint32_t flags __unused, uint64_t user_data __unused)
{
	if (crtc == NULL)
		return (EINVAL);
	crtc->primary_fb = fb;
	return (0);
}

static const struct drm_crtc_funcs igen9_crtc_funcs = {
	.set_config = igen9_legacy_set_config,
	.page_flip = igen9_legacy_page_flip,
};
static const struct drm_plane_funcs igen9_plane_funcs = { 0 };
static const struct drm_encoder_funcs igen9_encoder_funcs = { 0 };
static const struct drm_connector_funcs igen9_connector_funcs = { 0 };

static void igen9_owned_fb_destroy(struct drm_framebuffer *fb) { (void)fb; }

static const struct drm_framebuffer_funcs igen9_owned_fb_funcs = {
	.destroy = igen9_owned_fb_destroy,
};

static const uint32_t igen9_plane_formats[] = {
	0x34325258,	/* 'XR24' = DRM_FORMAT_XRGB8888 */
};

/*
 * Driver atomic hooks.  Trivial first-cut so MODE_ATOMIC against this
 * driver succeeds without actually programming HW.  Real HW writes land
 * once the display engine bring-up code is written.
 */
static int
igen9_atomic_check(struct drm_device *dev __unused,
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
igen9_atomic_commit(struct drm_device *dev, struct drm_atomic_state *state,
    bool nonblock __unused)
{
	struct igen9_softc *sc = dev->driver_priv;

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
		igen9_read_pipe_mode(sc, 0, &live);
		if (cs->mode.hdisplay != live.hdisplay ||
		    cs->mode.vdisplay != live.vdisplay ||
		    cs->mode.htotal != live.htotal ||
		    cs->mode.vtotal != live.vtotal) {
			device_printf(sc->dev,
			    "atomic_commit: requested %ux%u (htotal=%u vtotal=%u)"
			    " != live %ux%u (htotal=%u vtotal=%u);"
			    " full modeset not yet implemented\n",
			    cs->mode.hdisplay, cs->mode.vdisplay,
			    cs->mode.htotal, cs->mode.vtotal,
			    live.hdisplay, live.vdisplay,
			    live.htotal, live.vtotal);
			return (ENOTSUP);
		}
		DPRINTF(sc, 1,
		    "atomic_commit: pipe %u — requested matches live"
		    " %ux%u  (no-op)\n", i, live.hdisplay, live.vdisplay);
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

		if (ps->fb != NULL && ps->fb->funcs == &igen9_owned_fb_funcs) {
			struct igen9_owned_fb *ofb = __containerof(
			    ps->fb, struct igen9_owned_fb, base);
			uint32_t new_surf = ofb->test_fb->gtt_first_idx *
			    PAGE_SIZE;
			if (!sc->scanout_held)
				sc->scanout_prev_surf =
				    igen9_r32(sc, PLANE_SURF(0));
			igen9_w32(sc, PLANE_SURF(0), new_surf);
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
			uint32_t new_surf = igen9_gtt_bind_user_fb(sc,
			    ps->fb);
			if (new_surf == 0) {
				device_printf(sc->dev,
				    "atomic_commit: FB_ID %u has no GEM"
				    " backing\n", ps->fb->base.id);
				continue;
			}
			if (!sc->scanout_held)
				sc->scanout_prev_surf =
				    igen9_r32(sc, PLANE_SURF(0));
			igen9_wait_vblank(sc, 0);
			igen9_w32(sc, PLANE_SURF(0), new_surf);
			sc->scanout_held = true;
		} else if (ps->fb == NULL && sc->scanout_held) {
			igen9_w32(sc, PLANE_SURF(0), sc->scanout_prev_surf);
			sc->scanout_held = false;
			DPRINTF(sc, 1,
			    "atomic_commit: plane fb=NULL -> PLANE_SURF"
			    " restored to 0x%08x\n", sc->scanout_prev_surf);
		}
	}
	return (0);
}

static const struct drm_mode_config_funcs igen9_mode_config_funcs = {
	.atomic_check  = igen9_atomic_check,
	.atomic_commit = igen9_atomic_commit,
};

static int
igen9_probe(device_t dev)
{
	uint16_t vid = pci_get_vendor(dev);
	uint16_t did = pci_get_device(dev);
	size_t i;

	if (vid != INTEL_PCI_VENDOR)
		return (ENXIO);
	for (i = 0; i < nitems(igen9_ids); i++) {
		if (igen9_ids[i].id == did) {
			device_set_desc(dev, igen9_ids[i].desc);
			return (BUS_PROBE_DEFAULT);
		}
	}
	return (ENXIO);
}

static int
igen9_attach(device_t dev)
{
	struct igen9_softc *sc = device_get_softc(dev);
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
	error = kms_dev_register(&igen9_driver, sc, &sc->drm_dev);
	if (error != 0) {
		device_printf(dev, "kms_dev_register: %d\n", error);
		bus_release_resource(dev, SYS_RES_MEMORY, sc->gmadr_rid,
		    sc->gmadr_res);
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mmio_rid,
		    sc->mmio_res);
		return (error);
	}

	/* Install atomic hooks before any object becomes reachable. */
	sc->drm_dev->mode_config.funcs = &igen9_mode_config_funcs;

	/*
	 * RE scaffold: live MMIO snapshot/diff/poke/bit-scan via sysctl.
	 * Required before any display engine bring-up since the only honest
	 * way to characterise gen9 modeset state is to capture what the
	 * firmware / loader / previous driver left behind, then watch what
	 * each write actually does.
	 */
	igen9_re_sysctls_init(sc);
	igen9_snapshot_save(sc);

	/*
	 * Single stub of each KMS object so GETRESOURCES returns non-empty
	 * and atomic-aware userspace can enumerate something.  Real
	 * topology (one CRTC per pipe, encoders per DDI, connectors per
	 * physical port) lands once the display engine is decoded.
	 */
	error = kms_crtc_init(sc->drm_dev, &sc->crtc, &igen9_crtc_funcs);
	if (error == 0)
		error = kms_plane_init(sc->drm_dev, &sc->primary,
		    &igen9_plane_funcs, DRM_PLANE_TYPE_PRIMARY,
		    1u, igen9_plane_formats,
		    nitems(igen9_plane_formats));
	if (error == 0)
		error = kms_encoder_init(sc->drm_dev, &sc->encoder,
		    &igen9_encoder_funcs, DRM_MODE_ENCODER_TMDS);
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
	if (error == 0)
		error = kms_connector_init(sc->drm_dev, &sc->connector,
		    &igen9_connector_funcs,
		    DRM_MODE_CONNECTOR_HDMIA);
	if (error == 0)
		error = kms_connector_attach_encoder(&sc->connector,
		    &sc->encoder);
	if (error != 0)
		device_printf(dev, "topology init: %d (will appear with"
		    " empty/partial resources)\n", error);

	/*
	 * IRQ — MSI + Pipe A vblank.  Non-fatal; falling back to polled
	 * vblank still works (igen9_wait_vblank reads PIPE_FRMCOUNT).
	 */
	(void)igen9_irq_setup(sc);

	/*
	 * Best-effort EDID-on-attach: try to fetch the DDI_B EDID via GMBus
	 * and populate the connector's mode list.  Failure leaves the
	 * connector in UNKNOWN with no modes — userspace GETCONNECTOR still
	 * works, it just sees a connector with no detected sink.
	 */
	(void)igen9_attach_edid_modes(sc);

	device_printf(dev, "attached: PCI 8086:%04x as /dev/dri/card%d\n",
	    sc->pci_id, sc->drm_dev->minor);
	return (0);
}

static int
igen9_detach(device_t dev)
{
	struct igen9_softc *sc = device_get_softc(dev);

	if (sc->scanout_held && sc->scanout_fb != NULL) {
		igen9_anim_stop(sc);
		igen9_w32(sc, PLANE_SURF(0), sc->scanout_prev_surf);
		pause("gen9rst", hz / 20);
		igen9_test_fb_free(sc, sc->scanout_fb);
		free(sc->scanout_fb, M_KMS);
		sc->scanout_fb = NULL;
		sc->scanout_held = false;
	}
	if (igen9_exposed_fb != NULL) {
		if (sc->scanout_held) {
			igen9_w32(sc, PLANE_SURF(0),
			    sc->scanout_prev_surf);
			sc->scanout_held = false;
		}
		kms_framebuffer_cleanup(&igen9_exposed_fb->base);
		igen9_test_fb_free(sc, igen9_exposed_fb->test_fb);
		free(igen9_exposed_fb->test_fb, M_KMS);
		free(igen9_exposed_fb, M_KMS);
		igen9_exposed_fb = NULL;
	}
	if (sc->drm_dev != NULL) {
		igen9_irq_teardown(sc);
		igen9_re_sysctls_fini(sc);
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

static device_method_t igen9_methods[] = {
	DEVMETHOD(device_probe,		igen9_probe),
	DEVMETHOD(device_attach,	igen9_attach),
	DEVMETHOD(device_detach,	igen9_detach),
	DEVMETHOD_END
};

static driver_t igen9_driver_t = {
	"igen9",
	igen9_methods,
	sizeof(struct igen9_softc),
};

DRIVER_MODULE(igen9, pci, igen9_driver_t, 0, 0);
MODULE_VERSION(igen9, 1);
MODULE_DEPEND(igen9, kms, 1, 1, 1);
MODULE_DEPEND(igen9, pci, 1, 1, 1);
