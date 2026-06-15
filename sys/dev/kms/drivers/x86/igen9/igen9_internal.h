/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * igen9 internal API — shared between igen9.c (driver glue, RE harness,
 * GMBus, VBT, GTT, plane atomic path) and igen9_dpll.c (CDCLK, DPLL /
 * WRPLL, pipe resume from cold).  Not exported to consumers; only the
 * .c files in this driver include this header.
 */

#ifndef _IGEN9_INTERNAL_H_
#define _IGEN9_INTERNAL_H_

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

struct igen9_test_fb;

/*
 * User-FB GTT slot cache sizes.  Each ADDFB2 dumb buffer gets one slot
 * of USER_FB_GTT_SLOT_PAGES (8 MiB) at USER_FB_GTT_FIRST + slot*PAGES.
 */
#define	USER_FB_GTT_FIRST	0xa0000
#define	USER_FB_GTT_SLOT_PAGES	2048
#define	USER_FB_GTT_NSLOTS	8

/*
 * Driver-owned drm_framebuffer wrapper.  expose_scanout_fb sysctl wires
 * the already-allocated test_fb into the framework's mode-object table.
 */
struct igen9_owned_fb {
	struct drm_framebuffer	 base;
	struct igen9_test_fb	*test_fb;
};

struct igen9_softc {
	device_t		 dev;
	struct drm_device	*drm_dev;
	uint16_t		 pci_id;

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

	/* Minimal KMS topology — one of each, all stubs. */
	struct drm_crtc		 crtc;
	struct drm_plane	 primary;
	struct drm_encoder	 encoder;
	struct drm_connector	 connector;

	/* Driver-owned scanout buffer (scanout_hold sysctl). */
	struct igen9_test_fb	*scanout_fb;
	uint32_t		 scanout_prev_surf;
	bool			 scanout_held;

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
 * dev.igen9.<n>.debug (CTLFLAG_RWTUN).
 */
#define	DPRINTF(sc, level, ...)						\
	do {								\
		if ((sc)->sc_debug > (level))				\
			device_printf((sc)->dev, __VA_ARGS__);		\
	} while (0)

/*
 * Cross-file register macros — used in both igen9.c and igen9_dpll.c.
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

#define	DDI_BUF_CTL(d)		(0x64000 + (d) * 0x100)
#define	  DDI_BUF_CTL_ENABLE_BIT		(1u << 31)
#define	  DDI_BUF_CTL_TRANS_SELECT_SHIFT	24
#define	  DDI_BUF_CTL_PORT_WIDTH_X4		(3u << 1)
#define	  DDI_BUF_CTL_INIT_DISPLAY_DETECTED	(1u << 0)
#define	  DDI_BUF_CTL_IDLE_STATUS		(1u << 7)

#define	DPLL_CTRL2_DDI_B_OFF	(1u << 4)

/* Low-level MMIO accessors.  Inline so every TU gets its own copy. */
static inline uint32_t
igen9_r32(struct igen9_softc *sc, uint32_t off)
{
	return (bus_read_4(sc->mmio_res, off));
}

static inline void
igen9_w32(struct igen9_softc *sc, uint32_t off, uint32_t val)
{
	bus_write_4(sc->mmio_res, off, val);
}

/*
 * Cross-file entry points.
 *
 * igen9.c hosts: driver glue, attach/detach, atomic hooks, RE harness,
 *                GMBus + EDID, VBT, GTT, scanout playground, IRQ.
 *
 * igen9_dpll.c hosts: CDCLK + DPLL + WRPLL state inspection + program /
 *                     enable / route, plus the pipe-resume from-cold
 *                     experiment (try_pipe_resume) and the polled
 *                     wait_vblank helper used by atomic_commit.
 */

/* Registered as children of dev.igen9.<n>.re by igen9.c at attach. */
void	igen9_dpll_register_sysctls(struct igen9_softc *sc);

/* Polled vblank wait — used by atomic_commit before PLANE_SURF write. */
void	igen9_wait_vblank(struct igen9_softc *sc, int pipe);

/*
 * igen9_gmbus.c — PCH GMBus driver and the edid_read_b sysctl.  EDID
 * acquisition happens here; the parse + drm_display_mode build lives
 * in igen9.c's connector attach path.
 */
#define	GMBUS_PIN_DDI_B		5	/* SKL+ canonical pin map: DDI_B */
#define	EDID_SLAVE		0x50

int	igen9_gmbus_read_block(struct igen9_softc *sc, uint32_t pin,
	    uint8_t slave, uint8_t offset, uint8_t *buf, size_t len);
void	igen9_gmbus_register_sysctls(struct igen9_softc *sc);

#endif /* _IGEN9_INTERNAL_H_ */
