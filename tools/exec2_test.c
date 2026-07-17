/*
 * EXECBUFFER2 with MI_STORE_DWORD_IMM proof-write. (unchanged)
 *
 * Layout inside the single BO (softpin address = softpin_ggtt):
 *   [0x00..0x13]  batch: MI_STORE_DWORD_IMM header, addr_lo, addr_hi,
 *                        data, MI_BATCH_BUFFER_END  (5 dwords = 20 B)
 *   [0x100]       store target — engine writes 0xcafebabe here
 *
 * After the engine completes, we CLFLUSH the target line and read it
 * back from the CPU-mapped BO.  If it shows 0xcafebabe, the engine
 * really consulted our GGTT PTE, jumped to our BO, executed our
 * instruction stream, and stored to the address we pinned — end to
 * end, no scratch, no ambiguity.
 *
 * Build: cc -O2 -o exec2_test exec2_test.c
 */
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

struct drm_i915_gem_create {
	uint64_t size;
	uint32_t handle;
	uint32_t pad;
};

struct drm_i915_gem_mmap_offset {
	uint32_t handle;
	uint32_t pad;
	uint64_t offset;
	uint64_t flags;
};

struct drm_i915_gem_exec_object2 {
	uint32_t handle;
	uint32_t relocation_count;
	uint64_t relocs_ptr;
	uint64_t alignment;
	uint64_t offset;
	uint64_t flags;
	uint64_t rsvd1;
	uint64_t rsvd2;
};

struct drm_i915_gem_execbuffer2 {
	uint64_t buffers_ptr;
	uint32_t buffer_count;
	uint32_t batch_start_offset;
	uint32_t batch_len;
	uint32_t DR1;
	uint32_t DR4;
	uint32_t num_cliprects;
	uint64_t cliprects_ptr;
	uint64_t flags;
	uint64_t rsvd1;
	uint64_t rsvd2;
};

#define DRM_IOCTL_I915_GEM_CREATE	_IOWR('d', 0x40 + 0x1b, struct drm_i915_gem_create)
#define DRM_IOCTL_I915_GEM_MMAP_OFFSET	_IOWR('d', 0x40 + 0x40, struct drm_i915_gem_mmap_offset)
#define DRM_IOCTL_I915_GEM_EXECBUFFER2	_IOWR('d', 0x40 + 0x29, struct drm_i915_gem_execbuffer2)

/* MI_STORE_DWORD_IMM_GEN4 = (0x20 << 23) | (4 - 2) = 0x10000002.
 * MI_USE_GGTT = 1 << 22 = 0x00400000. */
#define MI_STORE_DWORD_IMM_GGTT		0x10400002u
#define MI_BATCH_BUFFER_END		0x05000000u

#define STORE_OFFSET_IN_BO		0x100u
#define STORE_MAGIC			0xcafebabeu

static inline void
clflush(const volatile void *p)
{
	__asm__ volatile ("clflush (%0)" : : "r"(p) : "memory");
	__asm__ volatile ("mfence" ::: "memory");
}

int
main(int argc, char **argv)
{
	uint64_t softpin = (argc > 1) ? strtoull(argv[1], NULL, 0)
	    : 0x1000000ULL;
	const char *path = "/dev/dri/renderD128";
	int fd = open(path, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "open(%s): %s\n", path, strerror(errno));
		return 1;
	}
	printf("opened %s -> fd %d, softpin=0x%llx\n", path, fd,
	    (unsigned long long)softpin);

	struct drm_i915_gem_create c = { .size = 4096 };
	if (ioctl(fd, DRM_IOCTL_I915_GEM_CREATE, &c) < 0) {
		fprintf(stderr, "GEM_CREATE: %s\n", strerror(errno));
		return 2;
	}
	printf("GEM_CREATE ok: handle=%u\n", c.handle);

	struct drm_i915_gem_mmap_offset mo = { .handle = c.handle };
	if (ioctl(fd, DRM_IOCTL_I915_GEM_MMAP_OFFSET, &mo) < 0) {
		fprintf(stderr, "MMAP_OFFSET: %s\n", strerror(errno));
		return 3;
	}
	void *map = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
	    (off_t)mo.offset);
	if (map == MAP_FAILED) {
		fprintf(stderr, "mmap: %s\n", strerror(errno));
		return 4;
	}
	uint32_t *bo = (uint32_t *)map;

	/* Compose batch: STORE_DWORD_IMM 0xcafebabe -> softpin + 0x100. */
	uint64_t store_target = softpin + STORE_OFFSET_IN_BO;
	bo[0] = MI_STORE_DWORD_IMM_GGTT;
	bo[1] = (uint32_t)(store_target & 0xffffffffu);
	bo[2] = (uint32_t)(store_target >> 32);
	bo[3] = STORE_MAGIC;
	bo[4] = MI_BATCH_BUFFER_END;

	/* Poison the target so we can tell if the engine wrote it. */
	uint32_t *tgt = (uint32_t *)((uint8_t *)bo + STORE_OFFSET_IN_BO);
	*tgt = 0xdeadbeef;

	/* Push our writes out to RAM before the engine reads them. */
	for (unsigned off = 0; off <= STORE_OFFSET_IN_BO; off += 64)
		clflush((uint8_t *)bo + off);

	printf("batch: %08x %08x %08x %08x %08x\n",
	    bo[0], bo[1], bo[2], bo[3], bo[4]);
	printf("target-pre: 0x%08x (expect 0xdeadbeef)\n", *tgt);

	struct drm_i915_gem_exec_object2 obj = {
		.handle = c.handle,
		.offset = softpin,
	};
	struct drm_i915_gem_execbuffer2 eb = {
		.buffers_ptr = (uintptr_t)&obj,
		.buffer_count = 1,
		.batch_start_offset = 0,
		.batch_len = 20,
	};
	int r = ioctl(fd, DRM_IOCTL_I915_GEM_EXECBUFFER2, &eb);
	printf("EXECBUFFER2: r=%d errno=%d (%s)\n", r, errno,
	    r < 0 ? strerror(errno) : "ok");

	/* Force CPU cache reload for the target line and read back. */
	clflush(tgt);
	uint32_t got = *(volatile uint32_t *)tgt;
	printf("target-post: 0x%08x (expect 0x%08x)\n", got, STORE_MAGIC);
	if (got == STORE_MAGIC)
		printf("PROOF: engine executed our batch and stored via"
		    " softpin-bound PTE.\n");
	else
		printf("MISS: engine did not update target — check GGTT bind,"
		    " cache coherency, or batch encoding.\n");

	munmap(map, 4096);
	close(fd);
	return (got == STORE_MAGIC) ? 0 : 5;
}
