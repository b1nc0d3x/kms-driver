/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * igen internal API — shared between igen.c (driver glue, RE harness,
 * GMBus, VBT, GTT, plane atomic path) and igen_dpll.c (CDCLK, DPLL /
 * WRPLL, pipe resume from cold).  Not exported to consumers; only the
 * .c files in this driver include this header.
 */

#ifndef _IGEN_INTERNAL_H_
#define _IGEN_INTERNAL_H_

/*
 * Silicon generation constants used at every dispatch site.  Kept as
 * integer "gen*10" values so HSW (gen 7.5) is representable alongside
 * the whole-integer gens.  Use these in comparisons rather than bare
 * literals so the dispatch seams are greppable.
 */
#define	IGEN_GEN_HSW	75	/* Haswell (gen 7.5) */
#define	IGEN_GEN_SKL	9	/* Skylake / Kabylake / Coffee Lake */
#define	IGEN_GEN_ICL	11	/* Ice Lake (placeholder) */
#define	IGEN_GEN_TGL	12	/* Tiger Lake+ (placeholder) */

#include <sys/types.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/sx.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <kms/drm_atomic.h>
#include <kms/drm_connector.h>
#include <kms/drm_crtc.h>
#include <kms/drm_encoder.h>
#include <kms/drm_framebuffer.h>
#include <kms/drm_plane.h>

#include "igen_aux.h"

/*
 * 8 MiB scratch framebuffer used by the scanout playground.  Owned by
 * igen_gtt.c; igen.c needs its definition because atomic_commit reads
 * gtt_first_idx to compute PLANE_SURF when an owned_fb is bound, and
 * the softc holds a pointer to one.
 */
struct igen_test_fb {
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

/*
 * User-FB GTT slot cache sizes.  Each ADDFB2 dumb buffer gets one slot
 * of USER_FB_GTT_SLOT_PAGES (32 MiB) at USER_FB_GTT_FIRST + slot*PAGES.
 *
 * 32 MiB / slot covers the high end of what modesetting hands us in
 * practice: 2880×1800 XR24 (Retina-class) is ~20 MiB, 3840×2160 XR24
 * (4K) is ~32 MiB on the nose.  At 8 slots that's a 256 MiB GTT
 * footprint, well inside the 2 GiB HSW aperture.  Bumping NSLOTS or
 * SLOT_PAGES further only costs GTT address space, not memory.
 */
#define	USER_FB_GTT_FIRST	0xa0000
#define	USER_FB_GTT_SLOT_PAGES	8192
#define	USER_FB_GTT_NSLOTS	8

/*
 * HW cursor GTT slot.  Sits just above the user-FB slot range so it
 * never collides; 64 pages is exactly enough for the largest cursor
 * mode the silicon advertises (256x256 ARGB8888 = 256 KiB).
 */
#define	CURSOR_GTT_FIRST	(USER_FB_GTT_FIRST + \
				    USER_FB_GTT_NSLOTS * USER_FB_GTT_SLOT_PAGES)
#define	CURSOR_GTT_PAGES	64

/*
 * Driver-owned drm_framebuffer wrapper.  expose_scanout_fb sysctl wires
 * the already-allocated test_fb into the framework's mode-object table.
 */
struct igen_owned_fb {
	struct drm_framebuffer	 base;
	struct igen_test_fb	*test_fb;
};

struct igen_softc {
	device_t		 dev;
	struct drm_device	*drm_dev;
	uint16_t		 pci_id;

	/*
	 * Silicon generation, as the integer "gen number" with one
	 * fractional digit times ten so all values fit in an int:
	 *   75 = HSW (Haswell / gen 7.5: BDW shares display IP enough
	 *        to land here too, but only HSW is wired up today)
	 *    9 = SKL / KBL / CFL (gen 9 / 9.5 display engine)
	 *   11 = ICL, 12 = TGL+
	 * Populated at attach from the PCI ID table.  Gen-specific code
	 * paths (CDCLK programming, DPLL / WRPLL topology, DDI_BUF_TRANS
	 * tables, power-well topology, PHY layout) branch on this rather
	 * than carrying separate driver names.  Use the IGEN_GEN_* macros
	 * below at the comparison sites so the seam is greppable.
	 */
	int			 gen;

	int			 mmio_rid;
	struct resource		*mmio_res;
	int			 gmadr_rid;
	struct resource		*gmadr_res;

	/* IRQ — single MSI vector covers all display interrupts. */
	int			 irq_rid;
	struct resource		*irq_res;
	void			*irq_cookie;
	uint64_t		 vblank_count_pipe_a;
	uint64_t		 vblank_count_pipe_b;
	uint64_t		 vblank_count_pipe_c;
	uint64_t		 irq_total_count;

	/* Debug verbosity level (0 = silent except errors). */
	int			 sc_debug;

	/*
	 * User-batch dispatch gate for EXECBUFFER2.
	 *   0 = ignore (validate + return, submit no ring work) — default.
	 *   1 = submit only when the display pipe is idle.
	 *   2 = force-submit even if pipe is active (bring-up smoke tests
	 *       — likely to disturb scanout; use with pipe pre-off).
	 * Runtime knob: dev.igen.<n>.re.i915_dispatch_enable (CTLFLAG_RWTUN).
	 */
	int			 i915_dispatch_enable;

	/*
	 * gen9 DDI cold bring-up gate.  0 = default, safe: gen9_panel_on
	 * writes only transcoder timing + PIPE_CONF ENABLE.  1 = also
	 * writes TRANS_CLK_SEL, TRANS_DDI_FUNC_CTL, DDI_BUF_CTL enable
	 * bits.  Setting to 1 without a warm port PLL wedges the box —
	 * live-verified 5th wedge 2026-07-16 — so it stays OFF unless the
	 * operator understands the PLL prerequisites.
	 */
	int			 gen9_ddi_enable;

	/*
	 * gen9 FULL cold-boot pipe-A bring-up gate.  0 = default, safe:
	 * standard atomic_commit path.  1 = when firmware boots dark,
	 * atomic_commit calls igen_gen9_full_bringup() which programs
	 * DPLL_CTRL1/2, DPLL1_CFGCR1/2, all TRANS_*, TRANS_CLK_SEL,
	 * TRANS_DDI_FUNC_CTL, DDI_BUF_CTL(B), PLANE_*, and PIPE_CONF —
	 * hardcoded values captured from a live 1920x1080@60 HDMI on
	 * DDI B session on this specific Dell OptiPlex 5040 (fbsdx86).
	 * The sequence checks LCPLL1/DPLL_STATUS first and refuses if
	 * the underlying oscillators aren't up.  Set via sysctl:
	 *   dev.igen.<n>.re.gen9_full_bringup=1
	 */
	int			 gen9_full_bringup;

	/* Minimal KMS topology — one of each, all stubs. */
	struct drm_crtc		 crtc;
	struct drm_plane	 primary;
	struct drm_encoder	 encoder;
	struct drm_connector	 connector;

	/*
	 * Per-DDI AUX channels.  Only those a physical port actually
	 * uses are wired up at attach (DDI A always, others on demand
	 * as DP/eDP sinks are detected via VBT/HPD).  Lives in igen_aux.c.
	 */
	struct igen_aux_channel	 aux_a;

	/*
	 * Cached EDID base block (128 bytes) from the last successful
	 * acquisition path — GMBus or AUX-I2C.  The framework stores the
	 * full EDID as a property blob (private), so we keep a local
	 * copy here for the bring-up path to re-parse the DTD timings
	 * when it programs the transcoder.  Length is 128 on a successful
	 * read, 0 if we never got a valid EDID.
	 */
	uint8_t			 cached_edid[256];
	size_t			 cached_edid_len;

	/* Driver-owned scanout buffer (scanout_hold sysctl). */
	struct igen_test_fb	*scanout_fb;
	uint32_t		 scanout_prev_surf;
	bool			 scanout_held;

	/*
	 * Serializes atomic_commit and cursor_move.  Both touch pipe A
	 * MMIO registers (PLANE_SURF, CUR_POS_A, CUR_BASE_A, CUR_CTL_A)
	 * and, without this, a mouse motion event racing an atomic frame
	 * commit lets both writers hit the pipe between vblanks, tearing
	 * the scanout.  sx (not mtx) because we call igen_wait_vblank
	 * under it — that path pauses.
	 */
	struct sx		 scanout_lock;

	/* Animation kthread. */
	struct thread		*anim_td;
	volatile bool		 anim_stop;
	bool			 anim_active;

	/* Per-FB GTT slot cache for ADDFB2 dumb buffers. */
	struct {
		struct drm_framebuffer	*fb;
		uint32_t		 surf;
	}			 user_fb_slots[USER_FB_GTT_NSLOTS];
	uint32_t		 user_fb_next_slot;

	/*
	 * HW cursor state — the GEM bitmap currently bound to the cursor
	 * GTT slot and the hotspot offset cached from the last MODE_CURSOR2
	 * BO write so subsequent MOVE-only commands can convert the
	 * userspace coordinate to a top-left position for CUR_POS_A.
	 */
	struct drm_gem_object	*cursor_obj;
	int32_t			 cursor_hot_x;
	int32_t			 cursor_hot_y;
	uint32_t		 cursor_w;
	uint32_t		 cursor_h;

	/* MMIO RE scaffold. */
	struct sx		 re_lock;
	uint32_t		*snapshot;
	size_t			 snapshot_words;
	bool			 snapshot_valid;
	uint32_t		 poke_addr;
	uint32_t		 bit_scan_addr;
	uint32_t		 bit_scan_skip;
	uint32_t		 wrpll_target_khz;
	uint32_t		 wrpll_dpll_id;	/* 2 = DPLL2, 3 = DPLL3 */
	uint32_t		 wrpll_route_port; /* 0=A..4=E */
	struct sysctl_ctx_list	 re_sysctl_ctx;
	struct sysctl_oid	*re_sysctl_tree;
};

/*
 * 4-level debug print idiom.  Gated on sc->sc_debug; runtime knob
 * dev.igen.<n>.debug (CTLFLAG_RWTUN).
 */
#define	DPRINTF(sc, level, ...)						\
	do {								\
		if ((sc)->sc_debug > (level))				\
			device_printf((sc)->dev, __VA_ARGS__);		\
	} while (0)

/*
 * Cross-file register macros — used in both igen.c and igen_dpll.c.
 * Kept here so neither file needs the other's private contents.  See
 * BSpec SKL display chapter for full register layout.
 */
#define	PIPE_CONF(p)		(0x70008 + (p) * 0x1000)
#define	  PIPE_CONF_ENABLE	(1u << 31)
#define	  PIPE_CONF_STATE	(1u << 30)
#define	PIPE_SRCSZ(p)		(0x7001c + (p) * 0x1000)
#define	PIPE_FRMCOUNT(p)	(0x70040 + (p) * 0x1000)

#define	TRANS_HTOTAL(t)		(0x60000 + (t) * 0x1000)
#define	TRANS_HBLANK(t)		(0x60004 + (t) * 0x1000)
#define	TRANS_HSYNC(t)		(0x60008 + (t) * 0x1000)
#define	TRANS_VTOTAL(t)		(0x6000c + (t) * 0x1000)
#define	TRANS_VBLANK(t)		(0x60010 + (t) * 0x1000)
#define	TRANS_VSYNC(t)		(0x60014 + (t) * 0x1000)
#define	TRANS_DDI_FUNC_CTL(t)	(0x60400 + (t) * 0x1000)

#define	PLANE_CTL(p)		(0x70180 + (p) * 0x1000)
#define	  PLANE_CTL_ENABLE	(1u << 31)
#define	  PLANE_CTL_FORMAT_SHIFT 24
#define	PLANE_STRIDE(p)		(0x70188 + (p) * 0x1000)
#define	PLANE_SIZE(p)		(0x70190 + (p) * 0x1000)
#define	PLANE_SURF(p)		(0x7019c + (p) * 0x1000)

#define	CUR_CTL_A		0x70080
#define	CUR_BASE_A		0x70084
#define	CUR_POS_A		0x70088
#define	  CUR_MODE_DISABLE	0x00u
#define	  CUR_MODE_64_ARGB	0x07u
#define	  CUR_MODE_128_ARGB	0x22u
#define	  CUR_MODE_256_ARGB	0x27u

#define	DDI_BUF_CTL(d)		(0x64000 + (d) * 0x100)
#define	  DDI_BUF_CTL_ENABLE_BIT		(1u << 31)
#define	  DDI_BUF_CTL_TRANS_SELECT_SHIFT	24
#define	  DDI_BUF_CTL_PORT_WIDTH_X4		(3u << 1)
#define	  DDI_BUF_CTL_INIT_DISPLAY_DETECTED	(1u << 0)
#define	  DDI_BUF_CTL_IDLE_STATUS		(1u << 7)

#define	DPLL_CTRL2_DDI_B_OFF	(1u << 4)

/*
 * HPD / fuse / live-sink registers.  Used by igen_hpd.c's HPD decoder and
 * by igen.c's silicon capability table — both want the same three oracles.
 */
#define	SFUSE_STRAP		0x000c2014
#define	SHOTPLUG_CTL_DDI	0x000c4030
#define	SDEISR			0x000c4000

/* Low-level MMIO accessors.  Inline so every TU gets its own copy. */
static inline uint32_t
igen_r32(struct igen_softc *sc, uint32_t off)
{
	return (bus_read_4(sc->mmio_res, off));
}

static inline void
igen_w32(struct igen_softc *sc, uint32_t off, uint32_t val)
{
	bus_write_4(sc->mmio_res, off, val);
}

/*
 * Cross-file entry points.
 *
 * igen.c hosts: driver glue, attach/detach, atomic hooks, RE harness,
 *                GMBus + EDID, VBT, GTT, scanout playground, IRQ.
 *
 * igen_dpll.c hosts: CDCLK + DPLL + WRPLL state inspection + program /
 *                     enable / route, plus the pipe-resume from-cold
 *                     experiment (try_pipe_resume) and the polled
 *                     wait_vblank helper used by atomic_commit.
 */

/* Registered as children of dev.igen.<n>.re by igen.c at attach. */
void	igen_dpll_register_sysctls(struct igen_softc *sc);

/* Polled vblank wait — used by atomic_commit before PLANE_SURF write. */
void	igen_wait_vblank(struct igen_softc *sc, int pipe);

/*
 * igen_gmbus.c — PCH GMBus driver and the edid_read_b sysctl.  EDID
 * acquisition happens here; the parse + drm_display_mode build lives
 * in igen.c's connector attach path.
 */
#define	GMBUS_PIN_DDI_B		5	/* SKL+ canonical pin map: DDI_B */
#define	EDID_SLAVE		0x50

int	igen_gmbus_read_block(struct igen_softc *sc, uint32_t pin,
	    uint8_t slave, uint8_t offset, uint8_t *buf, size_t len);
void	igen_gmbus_register_sysctls(struct igen_softc *sc);

/*
 * igen_hpd.c — live hot-plug detect decoder.  Pulls SFUSE_STRAP +
 * SHOTPLUG_CTL_DDI + SDEISR and prints per-DDI fuse / enable / pulse /
 * live-HPD state.  Sysctl-only; no IRQ wiring yet.
 */
void	igen_hpd_register_sysctls(struct igen_softc *sc);

/*
 * igen_phy.c — SKL display PHY / PCS / Common Lane RE substrate.
 * Sysctl-only; reads (no writes) the analog layer below DDI_BUF_CTL.
 */
void	igen_phy_register_sysctls(struct igen_softc *sc);

/*
 * igen_gt.c — iGPU render-engine ("GT") RE substrate.  Phase 1 only;
 * gt_status sysctl takes forcewake on the render domain, dumps RCS +
 * GuC state, and releases.  Phase 2+ adds execlist submission, GuC
 * firmware load, first batch buffer.
 */
void	igen_gt_register_sysctls(struct igen_softc *sc);

/*
 * EXECBUFFER2 entry point.  Composes a private ring that jumps to
 * `batch_ggtt`, live-submits via ELSP.  `force` overrides the pipe-
 * active refusal (destructive — only safe when the compositor has
 * no visible surface, e.g. headless boot).
 */
int	igen_gt_submit_user_batch(struct igen_softc *sc,
	    uint64_t batch_ggtt, bool force);

/*
 * igen_gtt.c — GTT introspection + RW helpers, 8 MiB scratch FB allocator,
 * per-FB GTT slot cache for ADDFB2 dumb buffers, persistent scanout
 * buffer (scanout_hold), animation kthread.  &igen_owned_fb_funcs is
 * defined in igen.c and shared via extern; igen_gtt.c installs it as
 * the funcs table for expose_scanout_fb's drm_framebuffer.
 */
extern const struct drm_framebuffer_funcs igen_owned_fb_funcs;

uint32_t igen_gtt_bind_user_fb(struct igen_softc *sc,
	    struct drm_framebuffer *fb);
uint32_t igen_gtt_bind_cursor(struct igen_softc *sc,
	    struct drm_gem_object *obj);
int	igen_gtt_bind_gem_at(struct igen_softc *sc,
	    struct drm_gem_object *obj, uint64_t ggtt_byte_addr);
int	igen_i915_ioctl(struct drm_file *file, u_long cmd, void *data);
uint64_t igen_gtt_read(struct igen_softc *sc, uint32_t entry_idx);
void	igen_gtt_write(struct igen_softc *sc, uint32_t entry_idx,
	    uint64_t pte);
void	igen_test_fb_free(struct igen_softc *sc, struct igen_test_fb *fb);
void	igen_anim_stop(struct igen_softc *sc);
void	igen_gtt_register_sysctls(struct igen_softc *sc);

/* Unwind held scanout + exposed FB on detach. */
void	igen_gtt_detach(struct igen_softc *sc);

/*
 * igen_hsw_pipe.c — Haswell-specific pipe/transcoder/DDI bring-up.
 * Owns the hsw_dp_dump (diagnostic) + hsw_panel_on (drives the panel
 * from cold) sysctls and the lower-level helpers behind them.  The
 * register_sysctls function is a no-op on non-HSW gens so igen.c can
 * call it unconditionally.
 */
int	igen_hsw_panel_on(struct igen_softc *sc);
int	igen_gen9_panel_on(struct igen_softc *sc,
	    const struct drm_display_mode *m);
int	igen_gen9_full_bringup(struct igen_softc *sc,
	    const struct drm_display_mode *m);
int	igen_gen9_pipe_full_off(struct igen_softc *sc);
int	igen_gen9_dpll1_reprogram(struct igen_softc *sc, uint32_t clock_khz);
int	igen_pipe_a_off(struct igen_softc *sc);
void	igen_edid_to_mode(const uint8_t *dtd, struct drm_display_mode *m);
void	igen_hsw_pipe_register_sysctls(struct igen_softc *sc);

/*
 * igen_dpll.c — DPLL/CFGCR encoding.
 *
 * igen_dpll_compute_cfgcr solves the SKL+ WRPLL-style DPLL for an HDMI
 * pixel clock and returns the two CFGCR register values ready for
 * DPLL{1,2,3}_CFGCR{1,2}.  Returns false when no VCO solution exists.
 */
bool	igen_dpll_compute_cfgcr(uint32_t pixel_khz, uint32_t *out_cfgcr1,
	    uint32_t *out_cfgcr2);

#endif /* _IGEN_INTERNAL_H_ */
