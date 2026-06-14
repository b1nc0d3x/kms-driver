/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * End-to-end userspace renderer demo:
 *   1. CREATE_DUMB a 1920x1080 XRGB8888 buffer
 *   2. MAP_DUMB + mmap to get a userspace pointer
 *   3. Draw a recognisable pattern (purple/yellow checkerboard with
 *      green border) so we can visually distinguish from the firmware
 *      framebuffer AND from the driver-owned diag pattern
 *   4. ADDFB2 to wrap it in a drm_framebuffer
 *   5. SET_CLIENT_CAP_ATOMIC + UNIVERSAL_PLANES
 *   6. Discover first primary plane + its property ids
 *   7. DRM_IOCTL_MODE_ATOMIC: bind plane to our dumb FB
 *   8. Hold N seconds
 *   9. Atomic clear plane -> firmware FB restored
 *   10. RMFB + DESTROY_DUMB
 *
 * This exercises every layer of the framework + driver: GEM alloc,
 * mmap, ADDFB2, atomic resolve, driver GTT-bind, PLANE_SURF write.
 *
 * Usage: dumb_flip_test <CRTC_ID> [hold_sec=4] [/dev/dri/cardN]
 *
 * Raw ioctls only; no libdrm.
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

#include <drm/drm.h>
#include <drm/drm_mode.h>

#define WIDTH  1920
#define HEIGHT 1080
#define BPP    32
#define FMT    0x34325258	/* DRM_FORMAT_XRGB8888 ('XR24') */

struct plane_props {
	uint32_t fb_id, crtc_id;
	uint32_t src_x, src_y, src_w, src_h;
	uint32_t crtc_x, crtc_y, crtc_w, crtc_h;
};

static int
find_first_primary_plane(int fd, uint32_t *plane_id_out)
{
	struct drm_mode_get_plane_res r;
	uint32_t ids[16];
	memset(&r, 0, sizeof(r));
	r.plane_id_ptr = (uintptr_t)ids;
	r.count_planes = 16;
	if (ioctl(fd, DRM_IOCTL_MODE_GETPLANERESOURCES, &r) != 0) {
		perror("GETPLANERESOURCES");
		return (-1);
	}
	if (r.count_planes == 0) return (-1);
	*plane_id_out = ids[0];
	return (0);
}

static int
resolve_plane_props(int fd, uint32_t plane_id, struct plane_props *p)
{
	struct drm_mode_obj_get_properties op;
	uint32_t pids[64];
	uint64_t pvals[64];
	memset(&op, 0, sizeof(op));
	op.obj_id = plane_id;
	op.obj_type = DRM_MODE_OBJECT_PLANE;
	op.props_ptr = (uintptr_t)pids;
	op.prop_values_ptr = (uintptr_t)pvals;
	op.count_props = 64;
	if (ioctl(fd, DRM_IOCTL_MODE_OBJ_GETPROPERTIES, &op) != 0)
		return (-1);
	for (uint32_t i = 0; i < op.count_props; i++) {
		struct drm_mode_get_property gp;
		memset(&gp, 0, sizeof(gp));
		gp.prop_id = pids[i];
		if (ioctl(fd, DRM_IOCTL_MODE_GETPROPERTY, &gp) != 0)
			continue;
#define M(field, str) if (strcmp(gp.name, str)==0) { p->field = pids[i]; continue; }
		M(fb_id, "FB_ID");      M(crtc_id, "CRTC_ID");
		M(src_x, "SRC_X");      M(src_y, "SRC_Y");
		M(src_w, "SRC_W");      M(src_h, "SRC_H");
		M(crtc_x, "CRTC_X");    M(crtc_y, "CRTC_Y");
		M(crtc_w, "CRTC_W");    M(crtc_h, "CRTC_H");
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
	uint32_t pids[10] = {
		p->fb_id, p->crtc_id,
		p->src_x, p->src_y, p->src_w, p->src_h,
		p->crtc_x, p->crtc_y, p->crtc_w, p->crtc_h,
	};
	uint64_t pvals[10] = {
		fb_id, crtc_id,
		0, 0, WIDTH << 16, HEIGHT << 16,
		0, 0, WIDTH, HEIGHT,
	};
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
draw_pattern(uint32_t *px, uint32_t pitch_bytes)
{
	uint32_t row_stride = pitch_bytes / 4;
	uint32_t purple = 0x00800080;
	uint32_t yellow = 0x00ffff00;
	uint32_t green  = 0x0000ff00;
	for (uint32_t y = 0; y < HEIGHT; y++) {
		for (uint32_t x = 0; x < WIDTH; x++) {
			uint32_t c;
			if (y < 32 || y >= HEIGHT - 32 ||
			    x < 32 || x >= WIDTH - 32)
				c = green;
			else
				c = (((x / 80) + (y / 80)) & 1) ?
				    purple : yellow;
			px[y * row_stride + x] = c;
		}
	}
}

int
main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr,
		    "usage: %s <CRTC_ID> [hold_sec=4] [/dev/dri/cardN]\n",
		    argv[0]);
		return (1);
	}
	uint32_t crtc_id = (uint32_t)atoi(argv[1]);
	int hold = (argc > 2) ? atoi(argv[2]) : 4;
	const char *path = (argc > 3) ? argv[3] : "/dev/dri/card0";

	int fd = open(path, O_RDWR);
	if (fd < 0) { perror(path); return (2); }

	/* 1. CREATE_DUMB */
	struct drm_mode_create_dumb cd;
	memset(&cd, 0, sizeof(cd));
	cd.width = WIDTH; cd.height = HEIGHT; cd.bpp = BPP;
	if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &cd) != 0) {
		perror("CREATE_DUMB"); return (3);
	}
	printf("CREATE_DUMB: handle=%u pitch=%u size=%llu\n",
	    cd.handle, cd.pitch, (unsigned long long)cd.size);

	/* 2. MAP_DUMB + mmap */
	struct drm_mode_map_dumb md;
	memset(&md, 0, sizeof(md));
	md.handle = cd.handle;
	if (ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &md) != 0) {
		perror("MAP_DUMB"); return (4);
	}
	void *ptr = mmap(NULL, cd.size, PROT_READ | PROT_WRITE, MAP_SHARED,
	    fd, md.offset);
	if (ptr == MAP_FAILED) { perror("mmap"); return (5); }
	printf("MAP_DUMB + mmap: ptr=%p  offset=0x%llx\n",
	    ptr, (unsigned long long)md.offset);

	/* 3. Draw */
	draw_pattern((uint32_t *)ptr, cd.pitch);

	/* 4. ADDFB2 */
	struct drm_mode_fb_cmd2 fc;
	memset(&fc, 0, sizeof(fc));
	fc.width = WIDTH; fc.height = HEIGHT; fc.pixel_format = FMT;
	fc.handles[0] = cd.handle;
	fc.pitches[0] = cd.pitch;
	fc.offsets[0] = 0;
	if (ioctl(fd, DRM_IOCTL_MODE_ADDFB2, &fc) != 0) {
		perror("ADDFB2"); return (6);
	}
	printf("ADDFB2: fb_id=%u\n", fc.fb_id);

	/* 5. Atomic caps */
	struct drm_set_client_cap cap = {
	    .capability = DRM_CLIENT_CAP_UNIVERSAL_PLANES, .value = 1,
	};
	(void)ioctl(fd, DRM_IOCTL_SET_CLIENT_CAP, &cap);
	cap.capability = DRM_CLIENT_CAP_ATOMIC;
	if (ioctl(fd, DRM_IOCTL_SET_CLIENT_CAP, &cap) != 0) {
		perror("SET_CLIENT_CAP(ATOMIC)"); return (7);
	}

	/* 6. Discover plane + props */
	uint32_t plane_id;
	if (find_first_primary_plane(fd, &plane_id) != 0) return (8);
	struct plane_props pp = { 0 };
	if (resolve_plane_props(fd, plane_id, &pp) != 0) return (9);

	/* 7. Flip */
	printf("ATOMIC: plane=%u fb=%u crtc=%u (hold %d s)\n",
	    plane_id, fc.fb_id, crtc_id, hold);
	if (do_atomic(fd, plane_id, &pp, fc.fb_id, crtc_id) != 0) {
		perror("MODE_ATOMIC(flip)"); return (10);
	}
	sleep(hold);

	/* 8. Restore */
	printf("ATOMIC: clearing plane fb=0\n");
	if (do_atomic(fd, plane_id, &pp, 0, 0) != 0) {
		perror("MODE_ATOMIC(clear)"); return (11);
	}

	/* 9. Cleanup */
	uint32_t fb_id = fc.fb_id;
	if (ioctl(fd, DRM_IOCTL_MODE_RMFB, &fb_id) != 0) perror("RMFB");
	munmap(ptr, cd.size);
	struct drm_mode_destroy_dumb dd = { .handle = cd.handle };
	if (ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd) != 0)
		perror("DESTROY_DUMB");

	close(fd);
	return (0);
}
