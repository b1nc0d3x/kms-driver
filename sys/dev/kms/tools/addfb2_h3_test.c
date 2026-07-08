/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * H3 regression: verify kms_framebuffer_check_geometry rejects
 * malformed ADDFB2 requests.
 *
 * The check landed in commit 0657e68 (kms-review round 2) to close
 * the CVE-2026-46209 attack pattern where an undersized FB backing
 * BO makes the scanout engine DMA past the end.  This tool opens
 * /dev/dri/cardN, allocates a tiny dumb BO, and pokes ADDFB2 with
 * six shapes:
 *   1. baseline legit           — must succeed
 *   2. height >> BO             — reject EINVAL
 *   3. width * cpp overflows u32 — reject EINVAL
 *   4. pitch < width * cpp      — reject EINVAL
 *   5. offset past end of BO    — reject EINVAL
 *   6. planar format (YV12)     — reject EINVAL (no cpp in table)
 *
 * Exits 0 if all six behave as expected, non-zero otherwise.
 */

#include <sys/ioctl.h>
#include <sys/types.h>

#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>
#include <drm/drm_fourcc.h>

static int fd = -1;
static int passes = 0, fails = 0;

static uint32_t
create_dumb(uint32_t w, uint32_t h, uint32_t *out_pitch, uint64_t *out_size)
{
	struct drm_mode_create_dumb c;

	memset(&c, 0, sizeof(c));
	c.width = w;
	c.height = h;
	c.bpp = 32;
	if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &c) != 0) {
		warn("CREATE_DUMB");
		return (0);
	}
	if (out_pitch)
		*out_pitch = c.pitch;
	if (out_size)
		*out_size = c.size;
	return (c.handle);
}

static void
destroy_dumb(uint32_t handle)
{
	struct drm_mode_destroy_dumb d;

	memset(&d, 0, sizeof(d));
	d.handle = handle;
	(void)ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &d);
}

static void
rmfb(uint32_t fb_id)
{
	(void)ioctl(fd, DRM_IOCTL_MODE_RMFB, &fb_id);
}

static void
expect_pass(const char *label, struct drm_mode_fb_cmd2 *cmd)
{
	int r = ioctl(fd, DRM_IOCTL_MODE_ADDFB2, cmd);
	if (r == 0) {
		printf("[PASS] %s (fb_id=%u)\n", label, cmd->fb_id);
		passes++;
		rmfb(cmd->fb_id);
	} else {
		printf("[FAIL] %s — got errno=%d (%s), wanted success\n",
		    label, errno, strerror(errno));
		fails++;
	}
}

static void
expect_reject(const char *label, struct drm_mode_fb_cmd2 *cmd)
{
	int r = ioctl(fd, DRM_IOCTL_MODE_ADDFB2, cmd);
	if (r != 0 && errno == EINVAL) {
		printf("[PASS] %s — rejected with EINVAL as expected\n",
		    label);
		passes++;
	} else if (r == 0) {
		printf("[FAIL] %s — accepted (fb_id=%u), wanted EINVAL\n",
		    label, cmd->fb_id);
		fails++;
		rmfb(cmd->fb_id);
	} else {
		printf("[FAIL] %s — got errno=%d (%s), wanted EINVAL\n",
		    label, errno, strerror(errno));
		fails++;
	}
}

int
main(int argc, char **argv)
{
	const char *node = "/dev/dri/card0";
	uint32_t handle, pitch;
	uint64_t bo_size;
	struct drm_mode_fb_cmd2 cmd;

	if (argc > 1)
		node = argv[1];
	fd = open(node, O_RDWR | O_CLOEXEC);
	if (fd < 0)
		err(1, "open %s", node);
	printf("addfb2_h3_test on %s\n", node);

	/* Small BO: 64x64 XRGB8888 → pitch=256, size >= 16384. */
	handle = create_dumb(64, 64, &pitch, &bo_size);
	if (handle == 0)
		return (1);
	printf("baseline BO: 64x64, pitch=%u, size=%llu\n",
	    pitch, (unsigned long long)bo_size);

	/* 1. Baseline: 64x64 legit → must succeed. */
	memset(&cmd, 0, sizeof(cmd));
	cmd.width = 64;
	cmd.height = 64;
	cmd.pixel_format = DRM_FORMAT_XRGB8888;
	cmd.handles[0] = handle;
	cmd.pitches[0] = pitch;
	expect_pass("baseline 64x64 XR24 pitch=256", &cmd);

	/* 2. Height overflow: address 1024 rows → BO too small. */
	memset(&cmd, 0, sizeof(cmd));
	cmd.width = 64;
	cmd.height = 1024;
	cmd.pixel_format = DRM_FORMAT_XRGB8888;
	cmd.handles[0] = handle;
	cmd.pitches[0] = pitch;
	expect_reject("height=1024 vs 64x64 BO", &cmd);

	/* 3. Width * cpp overflows 32-bit line_bytes. */
	memset(&cmd, 0, sizeof(cmd));
	cmd.width = 0x40000000u;	/* 1<<30 pixels; * 4 bytes = 1<<32 */
	cmd.height = 1;
	cmd.pixel_format = DRM_FORMAT_XRGB8888;
	cmd.handles[0] = handle;
	cmd.pitches[0] = pitch;
	expect_reject("width=2^30 XR24 (line_bytes 32-bit overflow)", &cmd);

	/* 4. Pitch < width * cpp. */
	memset(&cmd, 0, sizeof(cmd));
	cmd.width = 64;
	cmd.height = 64;
	cmd.pixel_format = DRM_FORMAT_XRGB8888;
	cmd.handles[0] = handle;
	cmd.pitches[0] = 128;	/* need 256 */
	expect_reject("pitch=128 < width*cpp=256", &cmd);

	/* 5. Offset drives (offset + pitch*(h-1) + width*cpp) past BO. */
	memset(&cmd, 0, sizeof(cmd));
	cmd.width = 64;
	cmd.height = 64;
	cmd.pixel_format = DRM_FORMAT_XRGB8888;
	cmd.handles[0] = handle;
	cmd.pitches[0] = pitch;
	cmd.offsets[0] = (uint32_t)bo_size;	/* at end → +size > BO */
	expect_reject("offset=bo_size (walks off end)", &cmd);

	/* 6. Planar / paletted format (not in cpp table). */
	memset(&cmd, 0, sizeof(cmd));
	cmd.width = 64;
	cmd.height = 64;
	cmd.pixel_format = DRM_FORMAT_YVU420;
	cmd.handles[0] = handle;
	cmd.pitches[0] = pitch;
	expect_reject("YV12 (unsupported planar format)", &cmd);

	destroy_dumb(handle);
	close(fd);

	printf("\n=== %d pass / %d fail ===\n", passes, fails);
	return (fails == 0 ? 0 : 1);
}
