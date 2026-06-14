/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Walk a /dev/dri/cardN node: enumerate connectors via
 * DRM_IOCTL_MODE_GETRESOURCES + DRM_IOCTL_MODE_GETCONNECTOR, print
 * each connector's modes.  Used to verify a driver's
 * EDID-on-attach path: a connected sink should show one or more
 * DTD-derived modes; an empty mode list means the driver didn't
 * populate the connector or the EDID read failed.
 *
 * Speaks raw ioctls so libdrm isn't required.
 *
 * Usage: connector_test [/dev/dri/cardN]   (default /dev/dri/card0)
 */

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>

int
main(int argc, char **argv)
{
	const char *path = (argc > 1) ? argv[1] : "/dev/dri/card0";
	int fd = open(path, O_RDWR);
	if (fd < 0) {
		perror(path);
		return (1);
	}

	struct drm_mode_card_res res;
	uint32_t conn_ids[16];
	uint32_t crtc_ids[16];
	uint32_t enc_ids[16];
	uint32_t fb_ids[16];

	memset(&res, 0, sizeof(res));
	res.connector_id_ptr = (uintptr_t)conn_ids;
	res.crtc_id_ptr = (uintptr_t)crtc_ids;
	res.encoder_id_ptr = (uintptr_t)enc_ids;
	res.fb_id_ptr = (uintptr_t)fb_ids;
	res.count_connectors = 16;
	res.count_crtcs = 16;
	res.count_encoders = 16;
	res.count_fbs = 16;
	if (ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res) != 0) {
		perror("GETRESOURCES");
		return (2);
	}

	printf("%s: %u connectors, %u crtcs, %u encoders\n",
	    path, res.count_connectors, res.count_crtcs, res.count_encoders);

	for (uint32_t i = 0; i < res.count_connectors; i++) {
		struct drm_mode_get_connector c;
		struct drm_mode_modeinfo modes[16];

		memset(&c, 0, sizeof(c));
		c.connector_id = conn_ids[i];
		c.count_modes = 16;
		c.modes_ptr = (uintptr_t)modes;

		if (ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &c) != 0) {
			perror("GETCONNECTOR");
			continue;
		}

		printf("connector %u: type=%u  status=%u  %u modes\n",
		    c.connector_id, c.connector_type, c.connection,
		    c.count_modes);
		uint32_t n = c.count_modes < 16 ? c.count_modes : 16;
		for (uint32_t m = 0; m < n; m++) {
			printf("  [%u] %s  %ux%u @%u Hz  pixclk=%u\n",
			    m, modes[m].name,
			    modes[m].hdisplay, modes[m].vdisplay,
			    modes[m].vrefresh, modes[m].clock);
		}
	}
	return (0);
}
