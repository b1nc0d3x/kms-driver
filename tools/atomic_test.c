/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Minimal MODE_ATOMIC round-trip exerciser.  Opens /dev/dri/cardN,
 * opts into atomic, finds the first CRTC + its ACTIVE property id,
 * and issues a single-prop DRM_MODE_ATOMIC setting ACTIVE=1.  On
 * success the kernel-side rk_kms_atomic_commit path runs and prints
 * `rk_kms0: atomic_commit: ...` in dmesg.
 *
 * Doesn't link libdrm — speaks raw ioctls so it works under any
 * FreeBSD where /usr/include/drm/drm.h + drm_mode.h are present.
 *
 * Usage: atomic_test [/dev/dri/cardN]   (default /dev/dri/card0)
 */

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>

static int
find_active_prop(int fd, uint32_t crtc_id, uint32_t *prop_id_out)
{
	struct drm_mode_obj_get_properties r;
	uint32_t prop_ids[64];
	uint64_t prop_vals[64];
	uint32_t i;
	struct drm_mode_get_property gp;

	memset(&r, 0, sizeof(r));
	r.obj_id = crtc_id;
	r.obj_type = DRM_MODE_OBJECT_CRTC;
	r.props_ptr = (uintptr_t)prop_ids;
	r.prop_values_ptr = (uintptr_t)prop_vals;
	r.count_props = sizeof(prop_ids) / sizeof(prop_ids[0]);
	if (ioctl(fd, DRM_IOCTL_MODE_OBJ_GETPROPERTIES, &r) < 0) {
		perror("OBJ_GETPROPERTIES");
		return (-1);
	}
	printf("CRTC %u has %u properties\n", crtc_id, r.count_props);
	for (i = 0; i < r.count_props && i < 64; i++) {
		memset(&gp, 0, sizeof(gp));
		gp.prop_id = prop_ids[i];
		if (ioctl(fd, DRM_IOCTL_MODE_GETPROPERTY, &gp) < 0)
			continue;
		printf("  prop %u: %s = %llu\n", prop_ids[i], gp.name,
		    (unsigned long long)prop_vals[i]);
		if (strcmp(gp.name, "ACTIVE") == 0) {
			*prop_id_out = prop_ids[i];
			return (0);
		}
	}
	return (-1);
}

int
main(int argc, char **argv)
{
	const char *path = argc > 1 ? argv[1] : "/dev/dri/card0";
	int fd;
	struct drm_set_client_cap cap;
	struct drm_mode_card_res res;
	uint32_t crtc_ids[16];
	uint32_t crtc_id;
	uint32_t active_prop;
	struct drm_mode_atomic atomic;
	uint32_t objs[1];
	uint32_t counts[1];
	uint32_t props[1];
	uint64_t vals[1];

	fd = open(path, O_RDWR);
	if (fd < 0) {
		perror(path);
		return (1);
	}

	memset(&cap, 0, sizeof(cap));
	cap.capability = DRM_CLIENT_CAP_ATOMIC;
	cap.value = 1;
	if (ioctl(fd, DRM_IOCTL_SET_CLIENT_CAP, &cap) < 0) {
		perror("SET_CLIENT_CAP(ATOMIC)");
		return (1);
	}
	printf("ATOMIC capability acquired\n");

	memset(&res, 0, sizeof(res));
	res.crtc_id_ptr = (uintptr_t)crtc_ids;
	res.count_crtcs = sizeof(crtc_ids) / sizeof(crtc_ids[0]);
	if (ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res) < 0) {
		perror("GETRESOURCES");
		return (1);
	}
	if (res.count_crtcs == 0) {
		fprintf(stderr, "no CRTCs\n");
		return (1);
	}
	crtc_id = crtc_ids[0];
	printf("First CRTC id = %u\n", crtc_id);

	if (find_active_prop(fd, crtc_id, &active_prop) != 0) {
		fprintf(stderr, "no ACTIVE property on CRTC %u\n", crtc_id);
		return (1);
	}
	printf("ACTIVE property id = %u\n", active_prop);

	objs[0] = crtc_id;
	counts[0] = 1;
	props[0] = active_prop;
	vals[0] = 1;

	memset(&atomic, 0, sizeof(atomic));
	atomic.flags = 0;
	atomic.count_objs = 1;
	atomic.objs_ptr = (uintptr_t)objs;
	atomic.count_props_ptr = (uintptr_t)counts;
	atomic.props_ptr = (uintptr_t)props;
	atomic.prop_values_ptr = (uintptr_t)vals;

	printf("issuing MODE_ATOMIC: crtc=%u ACTIVE=1\n", crtc_id);
	if (ioctl(fd, DRM_IOCTL_MODE_ATOMIC, &atomic) < 0) {
		fprintf(stderr, "MODE_ATOMIC failed: %s\n", strerror(errno));
		return (1);
	}
	printf("MODE_ATOMIC succeeded — check dmesg for rk_kms0 atomic_commit\n");
	close(fd);
	return (0);
}
