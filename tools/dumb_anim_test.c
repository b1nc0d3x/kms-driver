/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Triple-buffered animation through the standard DRM atomic path.
 *
 * Allocates 3 dumb buffers, ADDFB2's each.  Per frame: pick the next
 * buffer, draw a moving sprite + frame counter, atomic-commit the
 * plane to that FB_ID.  Sleeps to keep ~60 Hz.  This is the structure
 * of a real compositor render loop: separate buffer per inflight frame,
 * page-flip on vblank, no in-place drawing of the scanout buffer.
 *
 * Usage: dumb_anim_test <CRTC_ID> [frames=300] [/dev/dri/cardN]
 *
 * Raw ioctls only; no libdrm.  No threading -- single loop, sleep
 * between frames.  Tearing is expected (we don't wait on vblank
 * events); that's a future framework feature.
 */

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>

#define WIDTH  1920
#define HEIGHT 1080
#define BPP    32
#define FMT    0x34325258
#define NBUFS  3

struct dumb_buf {
	uint32_t handle;
	uint32_t fb_id;
	uint32_t pitch;
	uint64_t size;
	uint32_t *ptr;
};

struct plane_props {
	uint32_t fb_id, crtc_id;
	uint32_t src_x, src_y, src_w, src_h;
	uint32_t crtc_x, crtc_y, crtc_w, crtc_h;
};

static int
alloc_dumb(int fd, struct dumb_buf *b)
{
	struct drm_mode_create_dumb cd = { .width = WIDTH, .height = HEIGHT,
	    .bpp = BPP };
	if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &cd) != 0) return (-1);
	struct drm_mode_map_dumb md = { .handle = cd.handle };
	if (ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &md) != 0) return (-2);
	void *p = mmap(NULL, cd.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
	    md.offset);
	if (p == MAP_FAILED) return (-3);
	struct drm_mode_fb_cmd2 fc = { .width = WIDTH, .height = HEIGHT,
	    .pixel_format = FMT };
	fc.handles[0] = cd.handle;
	fc.pitches[0] = cd.pitch;
	if (ioctl(fd, DRM_IOCTL_MODE_ADDFB2, &fc) != 0) return (-4);
	b->handle = cd.handle;
	b->pitch  = cd.pitch;
	b->size   = cd.size;
	b->ptr    = p;
	b->fb_id  = fc.fb_id;
	return (0);
}

static int
find_first_primary_plane(int fd, uint32_t *plane_id_out)
{
	struct drm_mode_get_plane_res r;
	uint32_t ids[16];
	memset(&r, 0, sizeof(r));
	r.plane_id_ptr = (uintptr_t)ids;
	r.count_planes = 16;
	if (ioctl(fd, DRM_IOCTL_MODE_GETPLANERESOURCES, &r) != 0) return (-1);
	if (r.count_planes == 0) return (-1);
	*plane_id_out = ids[0];
	return (0);
}

static int
resolve_plane_props(int fd, uint32_t pid, struct plane_props *p)
{
	struct drm_mode_obj_get_properties op;
	uint32_t pids[64];
	uint64_t pvals[64];
	memset(&op, 0, sizeof(op));
	op.obj_id = pid;
	op.obj_type = DRM_MODE_OBJECT_PLANE;
	op.props_ptr = (uintptr_t)pids;
	op.prop_values_ptr = (uintptr_t)pvals;
	op.count_props = 64;
	if (ioctl(fd, DRM_IOCTL_MODE_OBJ_GETPROPERTIES, &op) != 0) return (-1);
	for (uint32_t i = 0; i < op.count_props; i++) {
		struct drm_mode_get_property gp;
		memset(&gp, 0, sizeof(gp));
		gp.prop_id = pids[i];
		if (ioctl(fd, DRM_IOCTL_MODE_GETPROPERTY, &gp) != 0) continue;
#define M(field, str) if (strcmp(gp.name, str)==0) { p->field = pids[i]; continue; }
		M(fb_id, "FB_ID"); M(crtc_id, "CRTC_ID");
		M(src_x, "SRC_X"); M(src_y, "SRC_Y");
		M(src_w, "SRC_W"); M(src_h, "SRC_H");
		M(crtc_x, "CRTC_X"); M(crtc_y, "CRTC_Y");
		M(crtc_w, "CRTC_W"); M(crtc_h, "CRTC_H");
#undef M
	}
	return (p->fb_id && p->crtc_id) ? 0 : -1;
}

static int
do_atomic(int fd, uint32_t plane_id, struct plane_props *p,
    uint32_t fb_id, uint32_t crtc_id)
{
	uint32_t obj_ids[1] = { plane_id };
	uint32_t cprops[1] = { 10 };
	uint32_t pids[10] = { p->fb_id, p->crtc_id,
	    p->src_x, p->src_y, p->src_w, p->src_h,
	    p->crtc_x, p->crtc_y, p->crtc_w, p->crtc_h };
	uint64_t pvals[10] = { fb_id, crtc_id,
	    0, 0, WIDTH << 16, HEIGHT << 16,
	    0, 0, WIDTH, HEIGHT };
	struct drm_mode_atomic at;
	memset(&at, 0, sizeof(at));
	at.count_objs = 1;
	at.objs_ptr = (uintptr_t)obj_ids;
	at.count_props_ptr = (uintptr_t)cprops;
	at.props_ptr = (uintptr_t)pids;
	at.prop_values_ptr = (uintptr_t)pvals;
	return ioctl(fd, DRM_IOCTL_MODE_ATOMIC, &at);
}

static void
draw_frame(uint32_t *px, uint32_t pitch_bytes, uint32_t frame)
{
	uint32_t row = pitch_bytes / 4;
	uint32_t bg = 0x00101030;	/* dark blue */
	for (uint32_t y = 0; y < HEIGHT; y++)
		for (uint32_t x = 0; x < WIDTH; x++)
			px[y * row + x] = bg;

	/* Animated sprite: 256x256 square moving in a horizontal sweep */
	uint32_t sw = 256, sh = 256;
	uint32_t span = WIDTH - sw - 64;
	uint32_t sx = 32 + (frame * 8) % span;
	uint32_t sy = (HEIGHT - sh) / 2;
	uint32_t color = 0x00ff8000 ^ ((frame & 0x3f) << 18);
	for (uint32_t y = 0; y < sh; y++)
		for (uint32_t x = 0; x < sw; x++)
			px[(sy + y) * row + (sx + x)] = color;

	/* Top + bottom bars carry the buffer index in a colored stripe */
	uint32_t stripe = (frame % NBUFS == 0) ? 0x00ff0000 :
	                  (frame % NBUFS == 1) ? 0x0000ff00 : 0x000000ff;
	for (uint32_t y = 0; y < 32; y++)
		for (uint32_t x = 0; x < WIDTH; x++)
			px[y * row + x] = stripe;
	for (uint32_t y = HEIGHT - 32; y < HEIGHT; y++)
		for (uint32_t x = 0; x < WIDTH; x++)
			px[y * row + x] = stripe;
}

int
main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "usage: %s <CRTC_ID> [frames=300] "
		    "[/dev/dri/cardN]\n", argv[0]);
		return (1);
	}
	uint32_t crtc_id = (uint32_t)atoi(argv[1]);
	int frames = (argc > 2) ? atoi(argv[2]) : 300;
	const char *path = (argc > 3) ? argv[3] : "/dev/dri/card0";

	int fd = open(path, O_RDWR);
	if (fd < 0) { perror(path); return (2); }

	struct dumb_buf bufs[NBUFS];
	for (int i = 0; i < NBUFS; i++) {
		if (alloc_dumb(fd, &bufs[i]) != 0) {
			fprintf(stderr, "alloc_dumb[%d] failed\n", i);
			return (3);
		}
		printf("buf[%d]: handle=%u fb_id=%u\n",
		    i, bufs[i].handle, bufs[i].fb_id);
	}

	struct drm_set_client_cap cap = {
	    .capability = DRM_CLIENT_CAP_UNIVERSAL_PLANES, .value = 1 };
	(void)ioctl(fd, DRM_IOCTL_SET_CLIENT_CAP, &cap);
	cap.capability = DRM_CLIENT_CAP_ATOMIC;
	if (ioctl(fd, DRM_IOCTL_SET_CLIENT_CAP, &cap) != 0) {
		perror("ATOMIC cap"); return (4);
	}
	uint32_t plane_id;
	if (find_first_primary_plane(fd, &plane_id) != 0) return (5);
	struct plane_props pp = { 0 };
	if (resolve_plane_props(fd, plane_id, &pp) != 0) return (6);

	printf("animating %d frames at plane=%u crtc=%u\n",
	    frames, plane_id, crtc_id);

	struct timespec t0;
	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (int f = 0; f < frames; f++) {
		struct dumb_buf *b = &bufs[f % NBUFS];
		draw_frame(b->ptr, b->pitch, f);
		do_atomic(fd, plane_id, &pp, b->fb_id, crtc_id);

		struct timespec want;
		want.tv_sec  = t0.tv_sec  + (f + 1) * 16 / 1000;
		want.tv_nsec = t0.tv_nsec + ((f + 1) * 16 % 1000) * 1000000;
		if (want.tv_nsec >= 1000000000) {
			want.tv_sec++;
			want.tv_nsec -= 1000000000;
		}
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &want, NULL);
	}

	(void)do_atomic(fd, plane_id, &pp, 0, 0);
	for (int i = 0; i < NBUFS; i++) {
		uint32_t fbi = bufs[i].fb_id;
		(void)ioctl(fd, DRM_IOCTL_MODE_RMFB, &fbi);
		munmap(bufs[i].ptr, bufs[i].size);
		struct drm_mode_destroy_dumb dd = { .handle = bufs[i].handle };
		(void)ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd);
	}
	close(fd);
	return (0);
}
