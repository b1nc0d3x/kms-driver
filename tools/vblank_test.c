/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Verify vblank events are delivered through the framework's event
 * queue: WAIT_VBLANK with _DRM_VBLANK_EVENT enqueues an event that the
 * driver's IRQ handler (via kms_vblank_handler) dispatches when the
 * pipe vblank IRQ fires.  We then read it from the fd.
 *
 * Counts elapsed vs vblank-pacing rate to confirm we're locked to
 * scanout, not just sleeping.
 *
 * Usage: vblank_test [/dev/dri/cardN] [n_frames=120]
 *
 * Raw ioctls only.
 */

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>

#define DRM_EVENT_VBLANK 0x01

struct vblank_event_header {
	uint32_t type;
	uint32_t length;
};

struct vblank_event {
	struct vblank_event_header base;
	uint64_t user_data;
	uint32_t tv_sec;
	uint32_t tv_usec;
	uint32_t sequence;
	uint32_t crtc_id;
};

int
main(int argc, char **argv)
{
	const char *path = (argc > 1) ? argv[1] : "/dev/dri/card0";
	int n = (argc > 2) ? atoi(argv[2]) : 120;

	int fd = open(path, O_RDWR);
	if (fd < 0) { perror(path); return (1); }

	struct timespec t0, t1;
	clock_gettime(CLOCK_MONOTONIC, &t0);

	for (int i = 0; i < n; i++) {
		union drm_wait_vblank arg;
		memset(&arg, 0, sizeof(arg));
		arg.request.type = _DRM_VBLANK_RELATIVE | _DRM_VBLANK_EVENT;
		arg.request.sequence = 1;
		arg.request.signal = 0xdead0000 | (uint64_t)i;
		if (ioctl(fd, DRM_IOCTL_WAIT_VBLANK, &arg) != 0) {
			perror("WAIT_VBLANK");
			return (2);
		}
		struct vblank_event ev;
		ssize_t r = read(fd, &ev, sizeof(ev));
		if (r != sizeof(ev)) {
			fprintf(stderr, "short read: %zd\n", r);
			return (3);
		}
		if (ev.base.type != DRM_EVENT_VBLANK) {
			fprintf(stderr, "unexpected event type 0x%x\n",
			    ev.base.type);
			return (4);
		}
		if (i < 4 || i % 20 == 0) {
			printf("[%3d] sequence=%u  crtc=%u  ts=%u.%06u"
			    "  user_data=0x%llx\n", i, ev.sequence,
			    ev.crtc_id, ev.tv_sec, ev.tv_usec,
			    (unsigned long long)ev.user_data);
		}
	}

	clock_gettime(CLOCK_MONOTONIC, &t1);
	double elapsed = (t1.tv_sec - t0.tv_sec) +
	    (t1.tv_nsec - t0.tv_nsec) / 1e9;
	printf("\n%d vblank events in %.3f s -> %.2f Hz\n",
	    n, elapsed, n / elapsed);
	close(fd);
	return (0);
}
