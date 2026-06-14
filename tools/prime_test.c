/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * PRIME export/import smoke test.
 *
 *   1. CREATE_DUMB a small buffer, mmap, write a magic
 *   2. PRIME_HANDLE_TO_FD -> get a dma-buf fd
 *   3. mmap the fd directly -- verify the magic round-trips through
 *      the file-backed path (proves fo_mmap shares pages with cdev mmap)
 *   4. PRIME_FD_TO_HANDLE on a fresh /dev/dri/cardN fd -- verify the
 *      buffer reappears as a new GEM handle whose MAP_DUMB mmap also
 *      shows the same magic (proves cross-fd buffer sharing)
 *
 * Raw ioctls only.
 *
 * Usage: prime_test [/dev/dri/cardN]
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

#define MAGIC 0xDEADBEEFCAFEBABEull
#define SIZE  (64 * 1024)

int
main(int argc, char **argv)
{
	const char *path = (argc > 1) ? argv[1] : "/dev/dri/card0";

	int fd = open(path, O_RDWR);
	if (fd < 0) { perror(path); return (1); }

	struct drm_mode_create_dumb cd = { .width = 256, .height = 64,
	    .bpp = 32 };
	if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &cd) != 0) {
		perror("CREATE_DUMB"); return (2);
	}
	printf("CREATE_DUMB: handle=%u size=%llu\n",
	    cd.handle, (unsigned long long)cd.size);

	struct drm_mode_map_dumb md = { .handle = cd.handle };
	if (ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &md) != 0) {
		perror("MAP_DUMB"); return (3);
	}
	uint64_t *p = mmap(NULL, cd.size, PROT_READ | PROT_WRITE,
	    MAP_SHARED, fd, md.offset);
	if (p == MAP_FAILED) { perror("mmap"); return (4); }
	p[0] = MAGIC;
	printf("wrote MAGIC via cdev mmap @ p[0]=0x%llx\n",
	    (unsigned long long)p[0]);

	struct drm_prime_handle ph = { .handle = cd.handle, .flags = 0 };
	if (ioctl(fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &ph) != 0) {
		perror("PRIME_HANDLE_TO_FD"); return (5);
	}
	printf("PRIME_HANDLE_TO_FD: dma_buf_fd=%d\n", ph.fd);

	uint64_t *q = mmap(NULL, cd.size, PROT_READ | PROT_WRITE,
	    MAP_SHARED, ph.fd, 0);
	if (q == MAP_FAILED) { perror("mmap dma_buf_fd"); return (6); }
	printf("mmap via dma-buf fd: q[0]=0x%llx %s\n",
	    (unsigned long long)q[0],
	    (q[0] == MAGIC) ? "MAGIC OK" : "MAGIC MISMATCH");

	/* Open a fresh /dev/dri fd to simulate another process and
	 * import via FD_TO_HANDLE. */
	int fd2 = open(path, O_RDWR);
	if (fd2 < 0) { perror("open #2"); return (7); }
	struct drm_prime_handle ih = { .fd = ph.fd, .flags = 0 };
	if (ioctl(fd2, DRM_IOCTL_PRIME_FD_TO_HANDLE, &ih) != 0) {
		perror("PRIME_FD_TO_HANDLE"); return (8);
	}
	printf("PRIME_FD_TO_HANDLE (fresh fd): handle=%u\n", ih.handle);

	struct drm_mode_map_dumb md2 = { .handle = ih.handle };
	if (ioctl(fd2, DRM_IOCTL_MODE_MAP_DUMB, &md2) != 0) {
		perror("MAP_DUMB(imported)"); return (9);
	}
	uint64_t *r = mmap(NULL, cd.size, PROT_READ | PROT_WRITE,
	    MAP_SHARED, fd2, md2.offset);
	if (r == MAP_FAILED) { perror("mmap imported"); return (10); }
	printf("mmap via imported handle on fresh fd: r[0]=0x%llx %s\n",
	    (unsigned long long)r[0],
	    (r[0] == MAGIC) ? "MAGIC OK" : "MAGIC MISMATCH");

	munmap(p, cd.size);
	munmap(q, cd.size);
	munmap(r, cd.size);
	close(ph.fd);
	close(fd2);
	close(fd);
	return (0);
}
