/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Dump the RK3399 DW HDMI controller MMIO range via /dev/mem.
 *
 * The DW HDMI controller maps each 8-bit register to a 32-bit MMIO
 * word: register N lives at memory offset (N << 2), low byte.  We
 * read each word and print the low byte, addressed by the logical
 * register number rk_drm / rk_kms use.
 *
 * Usage: dump_hdmi_regs [base_pa] [size_bytes]
 *   defaults: 0xff940000  0x20000  (full DW HDMI block)
 *
 * Compare two captures (one from a working rk_drm session, one from
 * kms just before phy_init's PLL-lock wait) with diff -u.
 */

#include <sys/types.h>
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

int
main(int argc, char **argv)
{
	off_t pa = 0xff940000;
	size_t size = 0x20000;
	int fd;
	volatile uint32_t *base;
	size_t n_regs, i;

	if (argc > 1)
		pa = strtoull(argv[1], NULL, 0);
	if (argc > 2)
		size = strtoull(argv[2], NULL, 0);

	fd = open("/dev/mem", O_RDONLY);
	if (fd < 0) {
		fprintf(stderr, "open /dev/mem: %s\n", strerror(errno));
		return (1);
	}
	base = mmap(NULL, size, PROT_READ, MAP_SHARED, fd, pa);
	if (base == MAP_FAILED) {
		fprintf(stderr, "mmap %#jx size %zu: %s\n", (uintmax_t)pa,
		    size, strerror(errno));
		return (1);
	}

	n_regs = size / 4;
	printf("# DW HDMI MMIO dump base=%#jx words=%zu\n", (uintmax_t)pa,
	    n_regs);
	for (i = 0; i < n_regs; i++) {
		uint32_t word = base[i];
		uint8_t b = (uint8_t)(word & 0xff);
		printf("%04zx %02x\n", i, b);
	}

	munmap((void *)base, size);
	close(fd);
	return (0);
}
