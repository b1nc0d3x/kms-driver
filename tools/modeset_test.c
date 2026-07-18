/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * modeset_test — issue DRM_IOCTL_MODE_SETCRTC with a chosen mode.
 * Exercises the full-modeset path in atomic_commit (igen.c) so we can
 * verify DPLL reprogramming + pipe teardown/rebuild end to end.
 *
 * Usage:
 *   modeset_test [-d /dev/dri/card0] [-l | -m <index>]
 *     -l           list connector modes and exit
 *     -m <index>   set mode at that connector-mode-list index (default: 0)
 *
 * Compositor + Xorg must be stopped first (they hold DRM master).
 *
 * Build: cc -O2 -o modeset_test modeset_test.c
 */

#include <sys/ioctl.h>
#include <sys/types.h>

#include <err.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>

#define	nitems_(x)	(sizeof(x) / sizeof((x)[0]))

static void
usage(void)
{
	fprintf(stderr,
	    "usage: modeset_test [-d /dev/dri/card0] "
	    "[-l | -m <index> | -w W -h H -c CLOCK_kHz]\n"
	    "  -l           list connector modes\n"
	    "  -m <index>   set mode at connector-list index\n"
	    "  -w W -h H -c CLOCK  synthesize a CVT-RB-ish mode at that size\n"
	    "                       (bypasses connector mode list — use to force\n"
	    "                       4K etc. into the modeset chain even when\n"
	    "                       EDID doesn't advertise them)\n");
	exit(2);
}

/*
 * CVT-RB (reduced blanking) v1-ish timing generator.  Not spec-perfect
 * but close enough for the driver's DPLL solver + transcoder programming
 * to accept.  Used with -w -h -c to force off-list modes.
 */
static void
synth_cvt_rb(uint32_t hd, uint32_t vd, uint32_t clock_khz,
    struct drm_mode_modeinfo *m)
{
	uint32_t hblank = 160;
	uint32_t vblank = 26;
	uint32_t hsync = 32;
	uint32_t hoff = 48;
	uint32_t vsync = 8;
	uint32_t voff = 3;

	memset(m, 0, sizeof(*m));
	m->clock = clock_khz;
	m->hdisplay = hd;
	m->hsync_start = hd + hoff;
	m->hsync_end = hd + hoff + hsync;
	m->htotal = hd + hblank;
	m->vdisplay = vd;
	m->vsync_start = vd + voff;
	m->vsync_end = vd + voff + vsync;
	m->vtotal = vd + vblank;
	m->flags = 0xa;	/* +hsync +vsync — CVT-RB uses positive sync */
	m->vrefresh = (clock_khz * 1000ULL) / (m->htotal * m->vtotal);
	snprintf(m->name, sizeof(m->name), "%ux%u", hd, vd);
}

int
main(int argc, char **argv)
{
	const char *devnode = "/dev/dri/card0";
	int list = 0;
	int mode_index = -1;
	int custom_w = 0, custom_h = 0, custom_c = 0;
	int ch, fd;
	struct drm_mode_card_res res;
	uint32_t crtc_ids[8], enc_ids[16], conn_ids[16], fb_ids[16];
	struct drm_mode_get_connector conn;
	struct drm_mode_modeinfo modes[64];
	uint32_t encoder_ids[16];
	struct drm_mode_create_dumb cd;
	struct drm_mode_fb_cmd2 fbcmd;
	struct drm_mode_crtc setcrtc;
	uint32_t conn_array[1];
	uint32_t chosen_conn, chosen_crtc;

	while ((ch = getopt(argc, argv, "d:lm:w:h:c:")) != -1) {
		switch (ch) {
		case 'd': devnode = optarg; break;
		case 'l': list = 1; break;
		case 'm': mode_index = atoi(optarg); break;
		case 'w': custom_w = atoi(optarg); break;
		case 'h': custom_h = atoi(optarg); break;
		case 'c': custom_c = atoi(optarg); break;
		default:  usage();
		}
	}
	if (mode_index == -1 && custom_w == 0 && !list)
		mode_index = 0;

	fd = open(devnode, O_RDWR | O_CLOEXEC);
	if (fd < 0)
		err(1, "open %s", devnode);

	/* Resources: CRTCs + connectors. */
	memset(&res, 0, sizeof(res));
	if (ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res) != 0)
		err(1, "MODE_GETRESOURCES count");
	if (res.count_crtcs > nitems_(crtc_ids))
		err(1, "too many crtcs");
	if (res.count_connectors > nitems_(conn_ids))
		err(1, "too many connectors");

	res.crtc_id_ptr = (uintptr_t)crtc_ids;
	res.encoder_id_ptr = (uintptr_t)enc_ids;
	res.connector_id_ptr = (uintptr_t)conn_ids;
	res.fb_id_ptr = (uintptr_t)fb_ids;
	if (ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res) != 0)
		err(1, "MODE_GETRESOURCES fill");
	if (res.count_crtcs == 0 || res.count_connectors == 0)
		errx(1, "no crtcs/connectors");
	chosen_crtc = crtc_ids[0];
	chosen_conn = conn_ids[0];
	printf("crtc=%u  connector=%u\n", chosen_crtc, chosen_conn);

	/* Connector modes. */
	memset(&conn, 0, sizeof(conn));
	conn.connector_id = chosen_conn;
	if (ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &conn) != 0)
		err(1, "MODE_GETCONNECTOR count");
	if (conn.count_modes == 0)
		errx(1, "connector has no modes");
	if (conn.count_modes > nitems_(modes))
		conn.count_modes = nitems_(modes);
	if (conn.count_encoders > nitems_(encoder_ids))
		conn.count_encoders = nitems_(encoder_ids);
	conn.modes_ptr = (uintptr_t)modes;
	conn.encoders_ptr = (uintptr_t)encoder_ids;
	conn.props_ptr = 0;
	conn.prop_values_ptr = 0;
	conn.count_props = 0;
	if (ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &conn) != 0)
		err(1, "MODE_GETCONNECTOR fill");

	if (list) {
		printf("connector %u modes:\n", chosen_conn);
		for (uint32_t i = 0; i < conn.count_modes; i++) {
			printf("  [%u] %ux%u  %u kHz  flags=0x%x  name=\"%.*s\"\n",
			    i, modes[i].hdisplay, modes[i].vdisplay,
			    modes[i].clock, modes[i].flags,
			    (int)sizeof(modes[i].name), modes[i].name);
		}
		close(fd);
		return (0);
	}

	struct drm_mode_modeinfo chosen_mode;

	if (custom_w != 0) {
		if (custom_h == 0 || custom_c == 0)
			errx(1, "-w requires -h and -c");
		synth_cvt_rb(custom_w, custom_h, custom_c, &chosen_mode);
		printf("setting SYNTH mode %ux%u @ %u kHz (CVT-RB-ish)\n",
		    custom_w, custom_h, custom_c);
	} else {
		if ((uint32_t)mode_index >= conn.count_modes)
			errx(1, "mode index %d out of range (0..%u)",
			    mode_index, conn.count_modes - 1);
		chosen_mode = modes[mode_index];
		printf("setting mode [%d] %ux%u @ %u kHz\n",
		    mode_index, chosen_mode.hdisplay,
		    chosen_mode.vdisplay, chosen_mode.clock);
	}

	/* Dumb fb sized for chosen mode. */
	memset(&cd, 0, sizeof(cd));
	cd.width = chosen_mode.hdisplay;
	cd.height = chosen_mode.vdisplay;
	cd.bpp = 32;
	if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &cd) != 0)
		err(1, "CREATE_DUMB");

	memset(&fbcmd, 0, sizeof(fbcmd));
	fbcmd.width = cd.width;
	fbcmd.height = cd.height;
	fbcmd.pixel_format = 0x34325258; /* DRM_FORMAT_XRGB8888 */
	fbcmd.handles[0] = cd.handle;
	fbcmd.pitches[0] = cd.pitch;
	if (ioctl(fd, DRM_IOCTL_MODE_ADDFB2, &fbcmd) != 0)
		err(1, "ADDFB2");
	printf("fb=%u  pitch=%u  size=%llu\n", fbcmd.fb_id, cd.pitch,
	    (unsigned long long)cd.size);

	memset(&setcrtc, 0, sizeof(setcrtc));
	conn_array[0] = chosen_conn;
	setcrtc.set_connectors_ptr = (uintptr_t)conn_array;
	setcrtc.count_connectors = 1;
	setcrtc.crtc_id = chosen_crtc;
	setcrtc.fb_id = fbcmd.fb_id;
	setcrtc.mode_valid = 1;
	setcrtc.mode = chosen_mode;
	if (ioctl(fd, DRM_IOCTL_MODE_SETCRTC, &setcrtc) != 0)
		err(1, "SETCRTC");
	printf("SETCRTC OK — check dmesg for the atomic_commit path\n");

	close(fd);
	return (0);
}
