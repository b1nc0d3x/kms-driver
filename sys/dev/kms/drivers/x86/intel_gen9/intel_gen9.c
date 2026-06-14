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
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
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
#include <kms/drm_plane.h>

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

/*
 * MMIO ranges we care about on gen9 display.  Bracket the regions, not
 * the full 16 MiB BAR — saves snapshot/diff memory and keeps the diff
 * signal-to-noise high.  Add ranges here as bring-up uncovers new state.
 *
 * Stride is 4 bytes (32-bit registers) for every range; gen9 doesn't have
 * any 8/16-bit-only MMIO that matters for display.
 */
struct intel_gen9_range {
	uint32_t	start;
	uint32_t	end;	/* inclusive */
	const char	*name;
};

static const struct intel_gen9_range intel_gen9_ranges[] = {
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

	/* MMIO RE scaffold (snapshot/diff/poke/bit-scan). */
	struct sx		 re_lock;
	uint32_t		*snapshot;	/* compact, one entry per
						 * 4-byte word in ranges[] */
	size_t			 snapshot_words;
	bool			 snapshot_valid;
	uint32_t		 poke_addr;	/* set via sysctl; consumed
						 * by poke_value sysctl */
	uint32_t		 bit_scan_addr;
	uint32_t		 bit_scan_skip; /* per-cycle skip count */
	struct sysctl_ctx_list	 re_sysctl_ctx;
	struct sysctl_oid	*re_sysctl_tree;
};

/* ----------------------------- MMIO RE helpers ---------------------------- */

static inline uint32_t
intel_gen9_r32(struct intel_gen9_softc *sc, uint32_t off)
{
	return (bus_read_4(sc->mmio_res, off));
}

static inline void
intel_gen9_w32(struct intel_gen9_softc *sc, uint32_t off, uint32_t val)
{
	bus_write_4(sc->mmio_res, off, val);
}

/*
 * Compute how many 32-bit words a full snapshot of intel_gen9_ranges[]
 * occupies.  Caller-relative offset of register `addr` within the
 * snapshot is the prefix-sum walk in intel_gen9_snapshot_index().
 */
static size_t
intel_gen9_snapshot_total_words(void)
{
	size_t words = 0;

	for (size_t i = 0; i < nitems(intel_gen9_ranges); i++) {
		words += (intel_gen9_ranges[i].end -
		    intel_gen9_ranges[i].start) / 4 + 1;
	}
	return (words);
}

/*
 * Return the index into sc->snapshot[] corresponding to MMIO offset
 * `addr`, or -1 if `addr` isn't in any tracked range.  O(N) over the
 * range table — N is ~20 so this is fine in sysctl/debug paths.
 */
static ssize_t
intel_gen9_snapshot_index(uint32_t addr)
{
	size_t base = 0;

	for (size_t i = 0; i < nitems(intel_gen9_ranges); i++) {
		const struct intel_gen9_range *r = &intel_gen9_ranges[i];
		size_t words = (r->end - r->start) / 4 + 1;

		if (addr >= r->start && addr <= r->end)
			return ((ssize_t)(base + (addr - r->start) / 4));
		base += words;
	}
	return (-1);
}

static void
intel_gen9_snapshot_save(struct intel_gen9_softc *sc)
{
	sx_xlock(&sc->re_lock);
	if (sc->snapshot == NULL) {
		sc->snapshot_words = intel_gen9_snapshot_total_words();
		sc->snapshot = malloc(sc->snapshot_words * sizeof(uint32_t),
		    M_KMS, M_WAITOK | M_ZERO);
	}
	size_t idx = 0;
	for (size_t i = 0; i < nitems(intel_gen9_ranges); i++) {
		const struct intel_gen9_range *r = &intel_gen9_ranges[i];
		for (uint32_t a = r->start; a <= r->end; a += 4)
			sc->snapshot[idx++] = intel_gen9_r32(sc, a);
	}
	sc->snapshot_valid = true;
	sx_xunlock(&sc->re_lock);
	device_printf(sc->dev, "snapshot saved (%zu words / %zu bytes)\n",
	    sc->snapshot_words, sc->snapshot_words * sizeof(uint32_t));
}

static void
intel_gen9_snapshot_diff(struct intel_gen9_softc *sc)
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
	for (size_t i = 0; i < nitems(intel_gen9_ranges); i++) {
		const struct intel_gen9_range *r = &intel_gen9_ranges[i];
		for (uint32_t a = r->start; a <= r->end; a += 4, idx++) {
			uint32_t cur = intel_gen9_r32(sc, a);

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
intel_gen9_bit_scan(struct intel_gen9_softc *sc)
{
	uint32_t addr = sc->bit_scan_addr;
	uint32_t orig, observed, bit_mask;

	if (intel_gen9_snapshot_index(addr) < 0) {
		device_printf(sc->dev,
		    "bit_scan: 0x%08x not in any tracked range\n", addr);
		return;
	}
	intel_gen9_snapshot_save(sc);
	sx_xlock(&sc->re_lock);
	orig = intel_gen9_r32(sc, addr);
	device_printf(sc->dev,
	    "bit_scan @0x%08x: orig=0x%08x (will toggle 32 bits)\n",
	    addr, orig);
	for (int bit = 0; bit < 32; bit++) {
		if (sc->bit_scan_skip & (1u << bit))
			continue;
		bit_mask = 1u << bit;
		intel_gen9_w32(sc, addr, orig ^ bit_mask);
		observed = intel_gen9_r32(sc, addr);
		device_printf(sc->dev,
		    "  bit %2d: wrote 0x%08x, readback 0x%08x %s\n",
		    bit, orig ^ bit_mask, observed,
		    ((observed ^ orig) & bit_mask) ? "RW" : "RO/clamped");
		intel_gen9_w32(sc, addr, orig);
	}
	sx_xunlock(&sc->re_lock);
	device_printf(sc->dev, "bit_scan done; running side-effect diff:\n");
	intel_gen9_snapshot_diff(sc);
}

static int
intel_gen9_sysctl_snapshot_save(SYSCTL_HANDLER_ARGS)
{
	struct intel_gen9_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL)
		return (error);
	if (trigger != 0)
		intel_gen9_snapshot_save(sc);
	return (0);
}

static int
intel_gen9_sysctl_snapshot_diff(SYSCTL_HANDLER_ARGS)
{
	struct intel_gen9_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL)
		return (error);
	if (trigger != 0)
		intel_gen9_snapshot_diff(sc);
	return (0);
}

static int
intel_gen9_sysctl_mmio_read(SYSCTL_HANDLER_ARGS)
{
	struct intel_gen9_softc *sc = arg1;
	uint32_t addr = sc->poke_addr;
	uint32_t val;
	int error;

	val = intel_gen9_r32(sc, addr);
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (req->newptr != NULL)
		return (EPERM);
	return (error);
}

static int
intel_gen9_sysctl_mmio_write(SYSCTL_HANDLER_ARGS)
{
	struct intel_gen9_softc *sc = arg1;
	uint32_t val = 0;
	int error = sysctl_handle_int(oidp, &val, 0, req);

	if (error || req->newptr == NULL)
		return (error);
	device_printf(sc->dev, "mmio_write: 0x%08x <- 0x%08x\n",
	    sc->poke_addr, val);
	intel_gen9_w32(sc, sc->poke_addr, val);
	return (0);
}

static int
intel_gen9_sysctl_bit_scan(SYSCTL_HANDLER_ARGS)
{
	struct intel_gen9_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL)
		return (error);
	if (trigger != 0)
		intel_gen9_bit_scan(sc);
	return (0);
}

static int	intel_gen9_sysctl_edid_read_b(SYSCTL_HANDLER_ARGS);
static int	intel_gen9_sysctl_vbt_dump(SYSCTL_HANDLER_ARGS);
static int	intel_gen9_sysctl_hpd_dump(SYSCTL_HANDLER_ARGS);

static void
intel_gen9_re_sysctls_init(struct intel_gen9_softc *sc)
{
	struct sysctl_oid_list *children;

	sx_init(&sc->re_lock, "intel_gen9_re");
	sysctl_ctx_init(&sc->re_sysctl_ctx);
	sc->re_sysctl_tree = SYSCTL_ADD_NODE(&sc->re_sysctl_ctx,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev)),
	    OID_AUTO, "re", CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
	    "MMIO reverse-engineering scaffold");
	children = SYSCTL_CHILDREN(sc->re_sysctl_tree);

	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "mmio_snapshot_save",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, intel_gen9_sysctl_snapshot_save, "I",
	    "write 1 to snapshot all tracked MMIO ranges");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "mmio_snapshot_diff",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, intel_gen9_sysctl_snapshot_diff, "I",
	    "write 1 to log changes since last snapshot");
	SYSCTL_ADD_UINT(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "mmio_addr", CTLFLAG_RW, &sc->poke_addr, 0,
	    "MMIO byte-offset for mmio_read / mmio_write");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "mmio_read",
	    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, intel_gen9_sysctl_mmio_read, "IU",
	    "read [mmio_addr] (32-bit)");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "mmio_write",
	    CTLTYPE_UINT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, intel_gen9_sysctl_mmio_write, "IU",
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
	    sc, 0, intel_gen9_sysctl_bit_scan, "I",
	    "write 1 to scan bit_scan_addr and diff side-effects");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "edid_read_b",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, intel_gen9_sysctl_edid_read_b, "I",
	    "write 1 to GMBus-read 128 bytes of EDID block 0 from DDI_B");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "vbt_dump",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, intel_gen9_sysctl_vbt_dump, "I",
	    "write 1 to map OpRegion via ASLS and walk VBT child devices");
	SYSCTL_ADD_PROC(&sc->re_sysctl_ctx, children, OID_AUTO,
	    "hpd_dump",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, intel_gen9_sysctl_hpd_dump, "I",
	    "write 1 to dump SFUSE_STRAP / SHOTPLUG_CTL_DDI / SDEISR live HPD");
}

static void
intel_gen9_re_sysctls_fini(struct intel_gen9_softc *sc)
{
	sysctl_ctx_free(&sc->re_sysctl_ctx);
	if (sc->snapshot != NULL) {
		free(sc->snapshot, M_KMS);
		sc->snapshot = NULL;
		sc->snapshot_valid = false;
	}
	sx_destroy(&sc->re_lock);
}

/* ---------------------------- GMBus / EDID -------------------------------- */

/*
 * SKL/KBL PCH GMBus.  Lives in display south block at PCH_DISPLAY_BASE +
 * 0x5100 = 0xc5100.  All DDI DDC paths funnel through this one controller;
 * which physical DDI is reached depends on the GMBUS0 port-number field
 * (pin pair).  i915's pin-pair numbering for SKL+ north display:
 *   1 = SSC   (panel sense)
 *   2 = VGADDC
 *   3 = PANEL
 *   4 = DDI_C   (HDMI-C / DP-C)
 *   5 = DDI_B   (HDMI-B / DP-B / SDVO)
 *   6 = DDI_D   (HDMI-D / DP-D)
 */

#define	GMBUS0			0x000c5100
#define	GMBUS1			0x000c5104
#define	GMBUS2			0x000c5108
#define	GMBUS3			0x000c510c
#define	GMBUS4			0x000c5110
#define	GMBUS5			0x000c5120

/*
 * SKL/KBL/CFL Display WA #0868: PCH GMBus unit clock is gated by default
 * on Sunrise Point / Kaby Point PCH.  Without explicitly disabling the
 * gate before a transaction, GMBUS0 writes go through but the IOs never
 * drive — pins look electrically dead with no NAK and no HW_RDY.
 * Bit 31 of SOUTH_DSPCLK_GATE_D is the gate-disable.
 */
#define	SOUTH_DSPCLK_GATE_D		0x000c2020
#define	  PCH_GMBUSUNIT_CLK_GATE_DIS	(1u << 31)

#define	GMBUS_RATE_100KHZ	(0 << 8)
#define	GMBUS_BYTE_CNT_OVERRIDE	(1u << 6)	/* GMBUS0 — allow >9 byte */
#define	GMBUS_PIN_DDI_C		4
#define	GMBUS_PIN_DDI_B		5
#define	GMBUS_PIN_DDI_D		6

#define	GMBUS_SW_CLR_INT	(1u << 31)
#define	GMBUS_SW_RDY		(1u << 30)
#define	GMBUS_CYCLE_WAIT	(1u << 25)	/* wait for next phase */
#define	GMBUS_CYCLE_STOP	(4u << 25)	/* stop on completion */
#define	GMBUS_BYTE_COUNT_SHIFT	16
#define	GMBUS_SLAVE_ADDR_SHIFT	1
#define	GMBUS_SLAVE_READ	1
#define	GMBUS_SLAVE_WRITE	0

#define	GMBUS_HW_RDY		(1u << 11)
#define	GMBUS_HW_WAIT		(1u << 14)
#define	GMBUS_NAK		(1u << 10)
#define	GMBUS_ACTIVE		(1u << 9)
#define	GMBUS_INUSE		(1u << 15)

#define	EDID_SLAVE		0x50

#define	DDI_BUF_CTL_A		0x64000
#define	DDI_BUF_CTL_B		0x64100
#define	DDI_BUF_CTL_C		0x64200
#define	DDI_BUF_CTL_D		0x64300
#define	DDI_BUF_CTL_E		0x64400
#define	DDI_BUF_CTL_ENABLE	(1u << 31)
#define	DDI_BUF_IS_IDLE		(1u << 7)

static void
intel_gen9_ddi_buf_wake(struct intel_gen9_softc *sc, uint32_t buf_ctl_reg)
{
	uint32_t v = intel_gen9_r32(sc, buf_ctl_reg);

	device_printf(sc->dev, "ddi_buf 0x%05x: pre=0x%08x (idle=%d)\n",
	    buf_ctl_reg, v, (v & DDI_BUF_IS_IDLE) != 0);

	/*
	 * Touch ENABLE again; i915 sometimes does this explicitly to wake
	 * the DDC line.  Then poll for !IDLE.  IDLE clear ~500us after
	 * ENABLE rises per BSpec.
	 */
	intel_gen9_w32(sc, buf_ctl_reg, v | DDI_BUF_CTL_ENABLE);
	for (int i = 0; i < 100; i++) {
		v = intel_gen9_r32(sc, buf_ctl_reg);
		if ((v & DDI_BUF_IS_IDLE) == 0)
			break;
		DELAY(10);
	}
	device_printf(sc->dev, "ddi_buf 0x%05x: post=0x%08x (idle=%d)\n",
	    buf_ctl_reg, v, (v & DDI_BUF_IS_IDLE) != 0);
}

static int
intel_gen9_gmbus_wait(struct intel_gen9_softc *sc, uint32_t bit)
{
	uint32_t s;

	for (int spin = 0; spin < 50000; spin++) {
		s = intel_gen9_r32(sc, GMBUS2);
		if (s & GMBUS_NAK)
			return (EIO);
		if (s & bit)
			return (0);
		DELAY(10);
	}
	return (ETIMEDOUT);
}

static int
intel_gen9_gmbus_read_block(struct intel_gen9_softc *sc, uint32_t pin,
    uint8_t slave, uint8_t offset, uint8_t *buf, size_t len)
{
	uint32_t cmd, val;
	int error;
	size_t got = 0;

	if (len == 0 || len > 256)
		return (EINVAL);

	/*
	 * Display WA #0868: disable PCH GMBus clock gating.  Keep it
	 * disabled across transactions — toggling per-xfer leaves the
	 * controller in a transient state that causes the next xfer to
	 * NAK or wedge.  Power cost is negligible for a polled DDC path.
	 */
	uint32_t gate = intel_gen9_r32(sc, SOUTH_DSPCLK_GATE_D);
	if ((gate & PCH_GMBUSUNIT_CLK_GATE_DIS) == 0)
		intel_gen9_w32(sc, SOUTH_DSPCLK_GATE_D,
		    gate | PCH_GMBUSUNIT_CLK_GATE_DIS);

	/*
	 * Quiesce the controller: clear any stuck interrupt, ensure 2-byte
	 * index off, then park GMBUS0=0 before reprogramming the pin.
	 * GMBUS2 INUSE is write-1-to-clear; without explicitly W1C-ing it
	 * a failed prior transaction (or BIOS hand-off) wedges every
	 * subsequent xfer at HW_RDY because the bus stays "in use".
	 */
	intel_gen9_w32(sc, GMBUS0, 0);
	intel_gen9_w32(sc, GMBUS4, 0);
	intel_gen9_w32(sc, GMBUS5, 0);
	intel_gen9_w32(sc, GMBUS1, GMBUS_SW_CLR_INT);
	intel_gen9_w32(sc, GMBUS1, 0);
	if (intel_gen9_r32(sc, GMBUS2) & GMBUS_INUSE) {
		device_printf(sc->dev, "gmbus: INUSE stuck, clearing\n");
		intel_gen9_w32(sc, GMBUS2, GMBUS_INUSE);
	}
	intel_gen9_w32(sc, GMBUS0, pin | GMBUS_RATE_100KHZ);

	/*
	 * Phase 1: write the EDID byte offset (segment 0).  CYCLE_WAIT
	 * keeps the bus owned past this transaction so the read phase
	 * issues a repeated START rather than a fresh STOP/START.
	 */
	cmd = GMBUS_SW_RDY | GMBUS_CYCLE_WAIT |
	    ((uint32_t)1 << GMBUS_BYTE_COUNT_SHIFT) |
	    ((uint32_t)slave << GMBUS_SLAVE_ADDR_SHIFT) |
	    GMBUS_SLAVE_WRITE;
	intel_gen9_w32(sc, GMBUS3, offset);
	intel_gen9_w32(sc, GMBUS1, cmd);
	error = intel_gen9_gmbus_wait(sc, GMBUS_HW_WAIT);
	if (error != 0) {
		device_printf(sc->dev, "gmbus: wait HW_WAIT after addr: %d\n",
		    error);
		goto out;
	}
	/*
	 * EDID EEPROMs need a settle delay between offset-pointer write and
	 * the read phase; without it the EEPROM clocks out stale-pointer
	 * data (often 0x00) for the first few bytes.
	 */
	DELAY(500);

	/*
	 * Phase 2: read `len` bytes.  CYCLE_WAIT (not STOP) here matches
	 * i915: it keeps the bus open for the whole multi-byte read and
	 * relies on the unconditional STOP terminator in the `out:` block
	 * to drive STOP on the wire after the last byte.
	 */
	cmd = GMBUS_SW_RDY | GMBUS_CYCLE_WAIT |
	    ((uint32_t)len << GMBUS_BYTE_COUNT_SHIFT) |
	    ((uint32_t)slave << GMBUS_SLAVE_ADDR_SHIFT) |
	    GMBUS_SLAVE_READ;
	intel_gen9_w32(sc, GMBUS1, cmd);

	while (got < len) {
		error = intel_gen9_gmbus_wait(sc, GMBUS_HW_RDY);
		if (error != 0) {
			uint32_t s = intel_gen9_r32(sc, GMBUS2);
			device_printf(sc->dev,
			    "gmbus: wait HW_RDY at %zu/%zu: %d  GMBUS2=0x%08x\n",
			    got, len, error, s);
			goto out;
		}
		val = intel_gen9_r32(sc, GMBUS3);
		for (int i = 0; i < 4 && got < len; i++)
			buf[got++] = (val >> (i * 8)) & 0xff;
	}
out:
	/*
	 * Per i915: always terminate with an explicit CYCLE_STOP+SW_RDY
	 * write to GMBUS1 to drive STOP on the wire and re-park the FSM,
	 * even on success.  Then tri-state pin, W1C INUSE, leave the
	 * clock-gate disabled for the next xfer.
	 */
	intel_gen9_w32(sc, GMBUS1, GMBUS_SW_RDY | GMBUS_CYCLE_STOP);
	(void)intel_gen9_gmbus_wait(sc, GMBUS_HW_WAIT);
	intel_gen9_w32(sc, GMBUS0, 0);
	intel_gen9_w32(sc, GMBUS1, GMBUS_SW_CLR_INT);
	intel_gen9_w32(sc, GMBUS1, 0);
	intel_gen9_w32(sc, GMBUS2, GMBUS_INUSE);
	return (error);
}

static int
intel_gen9_sysctl_edid_read_b(SYSCTL_HANDLER_ARGS)
{
	struct intel_gen9_softc *sc = arg1;
	uint8_t edid[128];
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL)
		return (error);
	if (trigger == 0)
		return (0);

	/*
	 * trigger=4: scan slave addresses on the only electrically alive
	 * pin (pin 4) with a 1-byte read.  ACK without NAK = slave present.
	 */
	if (trigger == 4) {
		for (uint8_t s = 0x08; s <= 0x77; s++) {
			uint8_t one = 0;
			uint32_t cmd, val, snap;
			error = 0;

			intel_gen9_w32(sc, GMBUS0, 0);
			intel_gen9_w32(sc, GMBUS4, 0);
			intel_gen9_w32(sc, GMBUS5, 0);
			intel_gen9_w32(sc, GMBUS1, GMBUS_SW_CLR_INT);
			intel_gen9_w32(sc, GMBUS1, 0);
			if (intel_gen9_r32(sc, GMBUS2) & GMBUS_INUSE)
				intel_gen9_w32(sc, GMBUS2, GMBUS_INUSE);
			intel_gen9_w32(sc, GMBUS0, 4 | GMBUS_RATE_100KHZ);

			cmd = GMBUS_SW_RDY | GMBUS_CYCLE_STOP |
			    ((uint32_t)1 << GMBUS_BYTE_COUNT_SHIFT) |
			    ((uint32_t)s << GMBUS_SLAVE_ADDR_SHIFT) |
			    GMBUS_SLAVE_READ;
			intel_gen9_w32(sc, GMBUS1, cmd);

			for (int spin = 0; spin < 5000; spin++) {
				snap = intel_gen9_r32(sc, GMBUS2);
				if (snap & (GMBUS_NAK | GMBUS_HW_RDY |
				    GMBUS_HW_WAIT))
					break;
				DELAY(10);
			}
			val = (snap & GMBUS_HW_RDY) ?
			    intel_gen9_r32(sc, GMBUS3) : 0;
			one = val & 0xff;
			if ((snap & GMBUS_NAK) == 0) {
				device_printf(sc->dev,
				    "scan: slave 0x%02x ACK!  GMBUS2=0x%08x"
				    "  byte0=0x%02x\n", s, snap, one);
			}
			intel_gen9_w32(sc, GMBUS0, 0);
		}
		device_printf(sc->dev, "scan done\n");
		return (0);
	}

	/*
	 * trigger=3: wake DDI_BUF_CTL_B then read EDID on pin 5 (canonical
	 * SKL+ DDI_B pin).  Goal: prove that the live HDMI port's DDC line
	 * is gated on DDI_BUF being active rather than just having ENABLE
	 * set passively.  Fallback to pin 4 if pin 5 still times out.
	 */
	if (trigger == 3) {
		intel_gen9_ddi_buf_wake(sc, DDI_BUF_CTL_B);
		DELAY(2000);
		for (uint32_t pin = 5; pin >= 4; pin--) {
			memset(edid, 0, sizeof(edid));
			error = intel_gen9_gmbus_read_block(sc, pin,
			    EDID_SLAVE, 0, edid, 16);
			device_printf(sc->dev,
			    "post-wake pin %u: err=%d  first16:"
			    " %02x %02x %02x %02x %02x %02x %02x %02x"
			    "  %02x %02x %02x %02x %02x %02x %02x %02x\n",
			    pin, error,
			    edid[0], edid[1], edid[2], edid[3],
			    edid[4], edid[5], edid[6], edid[7],
			    edid[8], edid[9], edid[10], edid[11],
			    edid[12], edid[13], edid[14], edid[15]);
			if (error == 0)
				break;
		}
		return (0);
	}

	/*
	 * Pin sweep: try every GMBUS pin in [1..9] with the EDID slave.
	 * The first pin that completes without NAK is the one wired to the
	 * physical DDC line of the active connector.  This bypasses any
	 * static assumption about port->pin mapping.
	 */
	if (trigger == 2) {
		for (uint32_t pin = 1; pin <= 9; pin++) {
			memset(edid, 0, sizeof(edid));
			error = intel_gen9_gmbus_read_block(sc, pin, EDID_SLAVE,
			    0, edid, 16);
			device_printf(sc->dev,
			    "pin %u: err=%d  first16: %02x %02x %02x %02x"
			    " %02x %02x %02x %02x  %02x %02x %02x %02x"
			    " %02x %02x %02x %02x\n", pin, error,
			    edid[0], edid[1], edid[2], edid[3],
			    edid[4], edid[5], edid[6], edid[7],
			    edid[8], edid[9], edid[10], edid[11],
			    edid[12], edid[13], edid[14], edid[15]);
		}
		return (0);
	}

	memset(edid, 0, sizeof(edid));
	error = intel_gen9_gmbus_read_block(sc, GMBUS_PIN_DDI_B, EDID_SLAVE,
	    0, edid, sizeof(edid));
	if (error != 0) {
		device_printf(sc->dev,
		    "edid_read_b: gmbus failed: %d  partial: "
		    "%02x %02x %02x %02x %02x %02x %02x %02x\n", error,
		    edid[0], edid[1], edid[2], edid[3],
		    edid[4], edid[5], edid[6], edid[7]);
		return (0);
	}
	device_printf(sc->dev, "edid_read_b: 128 bytes read:\n");
	for (int row = 0; row < 16; row++) {
		device_printf(sc->dev,
		    "  %02x: %02x %02x %02x %02x %02x %02x %02x %02x\n",
		    row * 8,
		    edid[row*8+0], edid[row*8+1], edid[row*8+2], edid[row*8+3],
		    edid[row*8+4], edid[row*8+5], edid[row*8+6], edid[row*8+7]);
	}
	/* Quick sanity decode: bytes 8-9 = manufacturer, 54+ = first DTD. */
	device_printf(sc->dev,
	    "  header OK=%d  mfg=%02x%02x  prod=%02x%02x  serial=%02x%02x%02x%02x\n",
	    (edid[0] == 0x00 && edid[1] == 0xff && edid[2] == 0xff &&
	     edid[7] == 0x00),
	    edid[8], edid[9], edid[10], edid[11],
	    edid[12], edid[13], edid[14], edid[15]);
	return (0);
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
intel_gen9_dvo_port_name(uint8_t p)
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
intel_gen9_device_type_name(uint16_t t)
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
intel_gen9_sysctl_vbt_dump(SYSCTL_HANDLER_ARGS)
{
	struct intel_gen9_softc *sc = arg1;
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
					    intel_gen9_device_type_name(
						cd->device_type),
					    cd->dvo_port,
					    intel_gen9_dvo_port_name(
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
intel_gen9_sysctl_hpd_dump(SYSCTL_HANDLER_ARGS)
{
	struct intel_gen9_softc *sc = arg1;
	uint32_t sfuse, hot, sde;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	sfuse = intel_gen9_r32(sc, SFUSE_STRAP);
	hot   = intel_gen9_r32(sc, SHOTPLUG_CTL_DDI);
	sde   = intel_gen9_r32(sc, SDEISR);

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

/* ----------------------------- driver glue -------------------------------- */

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

	if (vid != INTEL_PCI_VENDOR)
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
	 * RE scaffold: live MMIO snapshot/diff/poke/bit-scan via sysctl.
	 * Required before any display engine bring-up since the only honest
	 * way to characterise gen9 modeset state is to capture what the
	 * firmware / loader / previous driver left behind, then watch what
	 * each write actually does.
	 */
	intel_gen9_re_sysctls_init(sc);
	intel_gen9_snapshot_save(sc);

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
		intel_gen9_re_sysctls_fini(sc);
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
