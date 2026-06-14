/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * End-to-end DRM_IOCTL_MODE_ATOMIC plane-flip exerciser.
 *
 * Discovers the first primary plane's property IDs (FB_ID, CRTC_ID,
 * SRC_X/Y/W/H, CRTC_X/Y/W/H), then issues an atomic commit that points
 * the plane at the given FB_ID with the full 1920x1080 source / dest
 * rectangle.  Holds for N seconds.  Then issues a second atomic with
 * FB_ID = 0 to clear the plane (driver restores firmware surface).
 *
 * The whole point: prove the substrate works through the standard DRM
 * userspace ABI, not just via sysctls.
 *
 * Usage: flip_test <FB_ID> <CRTC_ID> [hold_seconds] [/dev/dri/cardN]
 *
 *   FB_ID    -- assigned by `sysctl dev.igen9.0.re.expose_scanout_fb=1`
 *   CRTC_ID  -- printed by the same sysctl
 *
 * Raw ioctls — no libdrm.
 */

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>

struct plane_props {
	uint32_t fb_id, crtc_id;
	uint32_t src_x, src_y, src_w, src_h;
	uint32_t crtc_x, crtc_y, crtc_w, crtc_h;
};

static int
find_first_primary_plane(int fd, uint32_t *plane_id_out)
{
	struct drm_mode_get_plane_res r;
	uint32_t plane_ids[16];
	memset(&r, 0, sizeof(r));
	r.plane_id_ptr = (uintptr_t)plane_ids;
	r.count_planes = 16;
	if (ioctl(fd, DRM_IOCTL_MODE_GETPLANERESOURCES, &r) != 0) {
		perror("GETPLANERESOURCES");
		return (-1);
	}
	if (r.count_planes == 0) {
		fprintf(stderr, "no planes\n");
		return (-1);
	}
	*plane_id_out = plane_ids[0];
	printf("plane_id=%u (first of %u)\n", *plane_id_out, r.count_planes);
	return (0);
}

static int
resolve_plane_props(int fd, uint32_t plane_id, struct plane_props *p)
{
	struct drm_mode_obj_get_properties op;
	uint32_t prop_ids[64];
	uint64_t prop_vals[64];

	memset(&op, 0, sizeof(op));
	op.obj_id = plane_id;
	op.obj_type = DRM_MODE_OBJECT_PLANE;
	op.props_ptr = (uintptr_t)prop_ids;
	op.prop_values_ptr = (uintptr_t)prop_vals;
	op.count_props = 64;
	if (ioctl(fd, DRM_IOCTL_MODE_OBJ_GETPROPERTIES, &op) != 0) {
		perror("OBJ_GETPROPERTIES(plane)");
		return (-1);
	}
	for (uint32_t i = 0; i < op.count_props; i++) {
		struct drm_mode_get_property gp;
		memset(&gp, 0, sizeof(gp));
		gp.prop_id = prop_ids[i];
		if (ioctl(fd, DRM_IOCTL_MODE_GETPROPERTY, &gp) != 0)
			continue;
#define MATCH(field, str) \
		if (strcmp(gp.name, str) == 0) { p->field = prop_ids[i]; continue; }
		MATCH(fb_id, "FB_ID");
		MATCH(crtc_id, "CRTC_ID");
		MATCH(src_x, "SRC_X");
		MATCH(src_y, "SRC_Y");
		MATCH(src_w, "SRC_W");
		MATCH(src_h, "SRC_H");
		MATCH(crtc_x, "CRTC_X");
		MATCH(crtc_y, "CRTC_Y");
		MATCH(crtc_w, "CRTC_W");
		MATCH(crtc_h, "CRTC_H");
#undef MATCH
	}
	if (p->fb_id == 0 || p->crtc_id == 0) {
		fprintf(stderr, "did not resolve all required plane props\n");
		return (-1);
	}
	printf("plane props: FB_ID=%u CRTC_ID=%u SRC=%u/%u/%u/%u "
	    "CRTC=%u/%u/%u/%u\n",
	    p->fb_id, p->crtc_id, p->src_x, p->src_y, p->src_w, p->src_h,
	    p->crtc_x, p->crtc_y, p->crtc_w, p->crtc_h);
	return (0);
}

static int
do_atomic(int fd, uint32_t plane_id, struct plane_props *p,
    uint32_t fb_id, uint32_t crtc_id)
{
	uint32_t obj_ids[1] = { plane_id };
	uint32_t count_props[1] = { 10 };
	uint32_t prop_ids[10] = {
		p->fb_id, p->crtc_id,
		p->src_x, p->src_y, p->src_w, p->src_h,
		p->crtc_x, p->crtc_y, p->crtc_w, p->crtc_h,
	};
	uint64_t prop_vals[10] = {
		fb_id, crtc_id,
		0, 0, 1920u << 16, 1080u << 16,
		0, 0, 1920, 1080,
	};
	struct drm_mode_atomic at;
	memset(&at, 0, sizeof(at));
	at.flags = 0;
	at.count_objs = 1;
	at.objs_ptr = (uintptr_t)obj_ids;
	at.count_props_ptr = (uintptr_t)count_props;
	at.props_ptr = (uintptr_t)prop_ids;
	at.prop_values_ptr = (uintptr_t)prop_vals;
	if (ioctl(fd, DRM_IOCTL_MODE_ATOMIC, &at) != 0) {
		perror("MODE_ATOMIC");
		return (-1);
	}
	return (0);
}

int
main(int argc, char **argv)
{
	if (argc < 3) {
		fprintf(stderr, "usage: %s <FB_ID> <CRTC_ID> "
		    "[hold_sec=3] [/dev/dri/cardN=/dev/dri/card0]\n", argv[0]);
		return (1);
	}
	uint32_t fb_id   = (uint32_t)atoi(argv[1]);
	uint32_t crtc_id = (uint32_t)atoi(argv[2]);
	int hold = (argc > 3) ? atoi(argv[3]) : 3;
	const char *path = (argc > 4) ? argv[4] : "/dev/dri/card0";

	int fd = open(path, O_RDWR);
	if (fd < 0) {
		perror(path);
		return (2);
	}
	struct drm_set_client_cap cap = {
		.capability = DRM_CLIENT_CAP_UNIVERSAL_PLANES,
		.value = 1,
	};
	(void)ioctl(fd, DRM_IOCTL_SET_CLIENT_CAP, &cap);
	cap.capability = DRM_CLIENT_CAP_ATOMIC;
	if (ioctl(fd, DRM_IOCTL_SET_CLIENT_CAP, &cap) != 0) {
		perror("SET_CLIENT_CAP(ATOMIC)");
		return (3);
	}

	uint32_t plane_id;
	if (find_first_primary_plane(fd, &plane_id) != 0)
		return (4);
	struct plane_props pp = { 0 };
	if (resolve_plane_props(fd, plane_id, &pp) != 0)
		return (5);

	printf("ATOMIC: plane=%u fb=%u crtc=%u (hold %d s)\n",
	    plane_id, fb_id, crtc_id, hold);
	if (do_atomic(fd, plane_id, &pp, fb_id, crtc_id) != 0)
		return (6);
	sleep(hold);

	printf("ATOMIC: clearing plane fb=0\n");
	if (do_atomic(fd, plane_id, &pp, 0, 0) != 0)
		return (7);
	close(fd);
	return (0);
}
