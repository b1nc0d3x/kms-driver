/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * igen GTT + scanout playground.
 *
 * The Graphics Translation Table at BAR0 + 0x800000 covers 4 GiB of
 * graphics address space at 4 KiB page granularity (gen8+ 8-byte PTEs).
 * This file owns:
 *   - GTT introspection (gtt_dump) + read/write helpers
 *   - 8 MiB scratch FB allocator (igen_test_fb_alloc + GTT slot cache)
 *   - Driver-owned persistent scanout buffer + animation kthread
 *     (scanout_hold sysctl, diagnostic patterns, Lissajous dot)
 *   - User-FB GTT slot allocator (igen_gtt_bind_user_fb) consumed by
 *     atomic_commit in igen.c to give each ADDFB2 dumb buffer its own
 *     permanent GTT mapping
 *   - expose_scanout_fb: registers a driver-owned drm_framebuffer
 *     against the kms mode-object table so atomic_test programs can
 *     bind it by FB_ID
 *
 * Exported entry points:
 *   uint32_t igen_gtt_bind_user_fb(sc, fb);
 *   void     igen_test_fb_free(sc, fb);
 *   void     igen_anim_stop(sc);
 *   void     igen_gtt_register_sysctls(sc);
 *
 * Sysctls registered here (children of dev.igen.<n>.re.):
 *   gtt_dump, gtt_alloc_test, test_fb_make, test_fb_flip,
 *   scanout_hold, expose_scanout_fb.
 *
 * &igen_owned_fb_funcs is defined in igen.c and referenced here by
 * pointer comparison + as the funcs table for the exposed FB.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/proc.h>
#include <sys/sysctl.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_page.h>

#include <drm/drm.h>
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
#include <kms/drm_plane.h>

#include "igen_internal.h"

MALLOC_DECLARE(M_KMS);

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

uint64_t
igen_gtt_read(struct igen_softc *sc, uint32_t entry_idx)
{
	uint32_t off = GTT_BASE + entry_idx * GTT_PTE_SIZE;
	uint32_t lo = igen_r32(sc, off);
	uint32_t hi = igen_r32(sc, off + 4);
	return ((uint64_t)hi << 32) | lo;
}

void
igen_gtt_write(struct igen_softc *sc, uint32_t entry_idx,
    uint64_t pte)
{
	uint32_t off = GTT_BASE + entry_idx * GTT_PTE_SIZE;
	igen_w32(sc, off, (uint32_t)pte);
	igen_w32(sc, off + 4, (uint32_t)(pte >> 32));
}

/* struct igen_test_fb is now defined in igen_internal.h. */

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
uint32_t
igen_gtt_bind_user_fb(struct igen_softc *sc,
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
		igen_gtt_write(sc, first_idx + i, pte);
	}
	uint32_t surf = first_idx * PAGE_SIZE;
	sc->user_fb_slots[slot].fb = fb;
	sc->user_fb_slots[slot].surf = surf;
	return (surf);
}

uint32_t
igen_gtt_bind_cursor(struct igen_softc *sc, struct drm_gem_object *obj)
{
	if (obj == NULL || obj->pages == NULL || obj->npages == 0)
		return (0);
	if (obj->npages > CURSOR_GTT_PAGES)
		return (0);
	for (size_t i = 0; i < obj->npages; i++) {
		vm_paddr_t pa = VM_PAGE_TO_PHYS(obj->pages[i]);
		uint64_t pte = (pa & ~0xfffULL) | GTT_PTE_VALID |
		    GTT_PTE_WRITEABLE;
		igen_gtt_write(sc, CURSOR_GTT_FIRST + i, pte);
	}
	return ((uint32_t)CURSOR_GTT_FIRST * PAGE_SIZE);
}

static int
igen_test_fb_alloc(struct igen_softc *sc,
    struct igen_test_fb *fb, uint32_t w, uint32_t h)
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
		igen_gtt_write(sc, fb->gtt_first_idx + i, pte);
	}
	fb->mapped = true;
	return (0);
}

void
igen_test_fb_free(struct igen_softc *sc,
    struct igen_test_fb *fb)
{
	if (fb->mapped) {
		for (uint32_t i = 0; i < fb->gtt_count; i++)
			igen_gtt_write(sc, fb->gtt_first_idx + i, 0);
		fb->mapped = false;
	}
	if (fb->va != NULL) {
		contigfree(fb->va, fb->size, M_KMS);
		fb->va = NULL;
	}
}

/* 64×64 checkerboard, two-color, in XRGB8888. */
static void
igen_test_fb_fill_checker(struct igen_test_fb *fb,
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
igen_test_fb_fill_diag(struct igen_test_fb *fb)
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
igen_anim_thread(void *arg)
{
	struct igen_softc *sc = arg;
	uint32_t frame = 0;
	uint32_t prev_x = 0, prev_y = 0;

	while (!sc->anim_stop && sc->scanout_held && sc->scanout_fb != NULL) {
		struct igen_test_fb *fb = sc->scanout_fb;
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
igen_anim_start(struct igen_softc *sc)
{
	if (sc->anim_active)
		return;
	sc->anim_stop = false;
	sc->anim_active = true;
	if (kthread_add(igen_anim_thread, sc, NULL, &sc->anim_td,
	    0, 0, "gen9anim") != 0) {
		sc->anim_active = false;
		device_printf(sc->dev, "anim: kthread_add failed\n");
	}
}

void
igen_anim_stop(struct igen_softc *sc)
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
static struct igen_owned_fb *igen_exposed_fb = NULL;

static int
igen_sysctl_expose_scanout_fb(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	if (igen_exposed_fb != NULL) {
		device_printf(sc->dev,
		    "expose_scanout_fb: already exposed as FB_ID=%u"
		    "  CRTC_ID=%u  PLANE_ID=%u\n",
		    igen_exposed_fb->base.base.id,
		    sc->crtc.base.id, sc->primary.base.id);
		return (0);
	}

	struct igen_owned_fb *ofb = malloc(sizeof(*ofb), M_KMS,
	    M_WAITOK | M_ZERO);
	struct igen_test_fb *tfb = malloc(sizeof(*tfb), M_KMS,
	    M_WAITOK | M_ZERO);
	error = igen_test_fb_alloc(sc, tfb, 1920, 1080);
	if (error != 0) {
		device_printf(sc->dev,
		    "expose_scanout_fb: alloc failed %d\n", error);
		free(tfb, M_KMS);
		free(ofb, M_KMS);
		return (0);
	}
	igen_test_fb_fill_diag(tfb);
	ofb->test_fb = tfb;

	ofb->base.width  = tfb->width;
	ofb->base.height = tfb->height;
	ofb->base.format = 0x34325258;	/* XR24 */
	ofb->base.pitches[0] = tfb->stride;
	error = kms_framebuffer_init(sc->drm_dev, &ofb->base,
	    &igen_owned_fb_funcs);
	if (error != 0) {
		device_printf(sc->dev,
		    "expose_scanout_fb: framebuffer_init failed %d\n", error);
		igen_test_fb_free(sc, tfb);
		free(tfb, M_KMS);
		free(ofb, M_KMS);
		return (0);
	}
	igen_exposed_fb = ofb;

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
igen_sysctl_scanout_hold(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	int hold = sc->scanout_held ? 1 : 0;
	int error = sysctl_handle_int(oidp, &hold, 0, req);

	if (error || req->newptr == NULL)
		return (error);

	if (hold && !sc->scanout_held) {
		sc->scanout_fb = malloc(sizeof(*sc->scanout_fb),
		    M_KMS, M_WAITOK | M_ZERO);
		error = igen_test_fb_alloc(sc, sc->scanout_fb,
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
			igen_test_fb_fill_diag(sc->scanout_fb);
		else
			igen_test_fb_fill_checker(sc->scanout_fb,
			    0x00ff0000, 0x000000ff);

		sc->scanout_prev_surf = igen_r32(sc, PLANE_SURF(0));
		uint32_t new_surf = sc->scanout_fb->gtt_first_idx * PAGE_SIZE;
		igen_w32(sc, PLANE_SURF(0), new_surf);
		sc->scanout_held = true;

		DPRINTF(sc, 0,
		    "scanout_hold: ON  PLANE_SURF 0x%08x -> 0x%08x\n",
		    sc->scanout_prev_surf, new_surf);

		if (hold == 3)
			igen_anim_start(sc);
	} else if (!hold && sc->scanout_held) {
		igen_anim_stop(sc);
		igen_w32(sc, PLANE_SURF(0), sc->scanout_prev_surf);
		pause("gen9rst", hz / 20);
		igen_test_fb_free(sc, sc->scanout_fb);
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
igen_sysctl_test_fb_flip(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	int hold_sec = 0;
	int error = sysctl_handle_int(oidp, &hold_sec, 0, req);

	if (error || req->newptr == NULL)
		return (error);
	if (hold_sec <= 0)
		return (0);
	if (hold_sec > 10)
		hold_sec = 10;

	struct igen_test_fb fb = { 0 };
	error = igen_test_fb_alloc(sc, &fb, 1920, 1080);
	if (error != 0) {
		device_printf(sc->dev, "test_fb_flip: alloc failed: %d\n",
		    error);
		return (0);
	}
	igen_test_fb_fill_checker(&fb, 0x00ff0000, 0x000000ff);

	/*
	 * Flip plane 1 of pipe A to scan from our buffer.  Our FB matches
	 * the firmware-programmed scanout in every observable parameter
	 * (1920x1080, XRGB8888, linear, stride 7680) so a single PLANE_SURF
	 * write is the entire change.  PLANE_SURF is the "armed" register —
	 * the change takes effect at the next vblank.
	 */
	uint32_t prev_surf = igen_r32(sc, PLANE_SURF(0));
	uint32_t new_surf  = fb.gtt_first_idx * PAGE_SIZE;
	uint32_t live_before = igen_r32(sc, PLANE_SURFLIVE(0));
	uint32_t frm_before  = igen_r32(sc, PIPE_FRMCOUNT(0));

	device_printf(sc->dev,
	    "test_fb_flip: PLANE_SURF 0x%08x -> 0x%08x (hold %d s)\n",
	    prev_surf, new_surf, hold_sec);
	device_printf(sc->dev,
	    "  pre:   SURFLIVE=0x%08x  FRMCOUNT=%u\n",
	    live_before, frm_before);

	igen_w32(sc, PLANE_SURF(0), new_surf);

	/*
	 * Sample SURFLIVE shortly after the arm to confirm HW latched it
	 * at the next vblank.  At 60 Hz a vblank arrives every ~16.7 ms;
	 * 50 ms of pause is plenty.
	 */
	pause("gen9arm", hz / 20);
	uint32_t live_armed = igen_r32(sc, PLANE_SURFLIVE(0));
	device_printf(sc->dev,
	    "  armed: SURFLIVE=0x%08x  %s\n", live_armed,
	    ((live_armed & 0xfffff000) == new_surf) ?
	    "FLIP TOOK" : "FLIP DID NOT TAKE");

	pause("gen9flp", hold_sec * hz);
	uint32_t live_end = igen_r32(sc, PLANE_SURFLIVE(0));
	uint32_t frm_end  = igen_r32(sc, PIPE_FRMCOUNT(0));

	igen_w32(sc, PLANE_SURF(0), prev_surf);
	pause("gen9rst", hz / 20);
	uint32_t live_restored = igen_r32(sc, PLANE_SURFLIVE(0));

	device_printf(sc->dev,
	    "  during: SURFLIVE=0x%08x  FRMCOUNT=%u  (advanced %u frames)\n",
	    live_end, frm_end, frm_end - frm_before);
	device_printf(sc->dev,
	    "  after restore: SURFLIVE=0x%08x  %s\n",
	    live_restored,
	    ((live_restored & 0xfffff000) == (prev_surf & 0xfffff000)) ?
	    "RESTORED OK" : "RESTORE FAILED");
	igen_test_fb_free(sc, &fb);
	return (0);
}

static int
igen_sysctl_test_fb_make(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	struct igen_test_fb fb = { 0 };
	error = igen_test_fb_alloc(sc, &fb, 1920, 1080);
	if (error != 0) {
		device_printf(sc->dev,
		    "test_fb: alloc failed (%d) — need %u KiB contig\n",
		    error, 1920 * 1080 * 4 / 1024);
		return (0);
	}
	igen_test_fb_fill_checker(&fb, 0x00ff0000, 0x000000ff);

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
	    (unsigned long long)igen_gtt_read(sc, fb.gtt_first_idx));

	igen_test_fb_free(sc, &fb);
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
igen_sysctl_gtt_alloc_test(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
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

	uint64_t before = igen_gtt_read(sc, test_idx);
	igen_gtt_write(sc, test_idx, pte);
	uint64_t after = igen_gtt_read(sc, test_idx);

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
	igen_gtt_write(sc, test_idx, before);
	contigfree(va, PAGE_SIZE, M_KMS);
	return (0);
}

static int
igen_sysctl_gtt_dump(SYSCTL_HANDLER_ARGS)
{
	struct igen_softc *sc = arg1;
	int trigger = 0;
	int error = sysctl_handle_int(oidp, &trigger, 0, req);

	if (error || req->newptr == NULL || trigger == 0)
		return (error);

	uint32_t valid_count = 0, last_pfn = 0, runs = 0;
	uint64_t first_pfn = 0;
	for (uint32_t i = 0; i < 2048; i++) {	/* first 8 MiB of GTT */
		uint64_t pte = igen_gtt_read(sc, i);
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
		uint64_t pte = igen_gtt_read(sc, i);
		device_printf(sc->dev,
		    "  GTT[%u] = 0x%016llx  %s%s  PFN=0x%llx\n",
		    i, (unsigned long long)pte,
		    (pte & GTT_PTE_VALID) ? "V" : "-",
		    (pte & GTT_PTE_WRITEABLE) ? "W" : "-",
		    (unsigned long long)(pte >> 12));
	}
	return (0);
}

/*
 * Register the GTT + scanout-playground sysctls under dev.igen.<n>.re.
 * Called from igen_re_sysctls_init in igen.c.
 */
void
igen_gtt_register_sysctls(struct igen_softc *sc)
{
	struct sysctl_ctx_list *ctx = &sc->re_sysctl_ctx;
	struct sysctl_oid_list *children =
	    SYSCTL_CHILDREN(sc->re_sysctl_tree);

	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "gtt_dump",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_gtt_dump, "I",
	    "write 1 to scan first 2048 GTT PTEs and print first 8");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "gtt_alloc_test",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_gtt_alloc_test, "I",
	    "write 1 to alloc 1 page, map at GTT[2048], read back, free");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "test_fb_make",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_test_fb_make, "I",
	    "write 1 to alloc + GTT-map + checker-fill an 8 MiB 1920x1080 FB");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "test_fb_flip",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_test_fb_flip, "I",
	    "write N (seconds 1..10) to flip PLANE_SURF to our checker FB"
	    " then restore");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "scanout_hold",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_scanout_hold, "I",
	    "1 = checker, 2 = diagnostic gradient+grid, both flip PLANE_SURF"
	    " and HOLD;  0 = restore firmware FB + free");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "expose_scanout_fb",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT,
	    sc, 0, igen_sysctl_expose_scanout_fb, "I",
	    "write 1 to allocate + register a driver-owned drm_framebuffer"
	    " for userspace MODE_ATOMIC; prints FB_ID, CRTC_ID, PLANE_ID");
}

/*
 * Detach unwind: stop the animation kthread, restore firmware PLANE_SURF
 * if we still hold it, free the persistent scanout_fb and the exposed
 * drm_framebuffer.  Called from igen.c igen_detach.
 */
void
igen_gtt_detach(struct igen_softc *sc)
{
	if (sc->scanout_held && sc->scanout_fb != NULL) {
		igen_anim_stop(sc);
		igen_w32(sc, PLANE_SURF(0), sc->scanout_prev_surf);
		pause("igenrst", hz / 20);
		igen_test_fb_free(sc, sc->scanout_fb);
		free(sc->scanout_fb, M_KMS);
		sc->scanout_fb = NULL;
		sc->scanout_held = false;
	}
	if (igen_exposed_fb != NULL) {
		if (sc->scanout_held) {
			igen_w32(sc, PLANE_SURF(0), sc->scanout_prev_surf);
			sc->scanout_held = false;
		}
		kms_framebuffer_cleanup(&igen_exposed_fb->base);
		igen_test_fb_free(sc, igen_exposed_fb->test_fb);
		free(igen_exposed_fb->test_fb, M_KMS);
		free(igen_exposed_fb, M_KMS);
		igen_exposed_fb = NULL;
	}
}
