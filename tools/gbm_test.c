/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * libgbm-kms smoke test.
 *
 *   1. gbm_create_device on /dev/dri/cardN
 *   2. gbm_bo_create 1920x1080 XRGB8888
 *   3. gbm_bo_map / write a pattern / gbm_bo_unmap
 *   4. gbm_bo_get_fd  -> use as dma-buf
 *   5. gbm_bo_get_handle / get_stride / get_size readbacks
 *   6. gbm_bo_destroy / gbm_device_destroy
 */

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

#include <gbm.h>

int
main(int argc, char **argv)
{
	const char *path = (argc > 1) ? argv[1] : "/dev/dri/card0";
	int fd = open(path, O_RDWR);
	if (fd < 0) { perror(path); return (1); }

	struct gbm_device *dev = gbm_create_device(fd);
	if (dev == NULL) { perror("gbm_create_device"); return (2); }
	printf("device: backend=%s fd=%d\n",
	    gbm_device_get_backend_name(dev), gbm_device_get_fd(dev));

	struct gbm_bo *bo = gbm_bo_create(dev, 1920, 1080,
	    GBM_FORMAT_XRGB8888, GBM_BO_USE_SCANOUT | GBM_BO_USE_LINEAR);
	if (bo == NULL) { perror("gbm_bo_create"); return (3); }
	printf("bo: w=%u h=%u fmt=0x%x stride=%u size=%zu handle=%u\n",
	    gbm_bo_get_width(bo), gbm_bo_get_height(bo),
	    gbm_bo_get_format(bo), gbm_bo_get_stride(bo),
	    gbm_bo_get_size(bo), gbm_bo_get_handle(bo).u32);

	void *map_data;
	uint32_t stride;
	void *ptr = gbm_bo_map(bo, 0, 0, 1920, 1080,
	    GBM_BO_TRANSFER_READ_WRITE, &stride, &map_data);
	if (ptr == NULL) { perror("gbm_bo_map"); return (4); }
	printf("map: ptr=%p stride=%u\n", ptr, stride);

	/* Write a diagonal pixel so the cdev mmap from prime_test could
	 * verify the same pages. */
	uint32_t *px = ptr;
	for (uint32_t i = 0; i < 256; i++)
		px[i * (stride / 4) + i] = 0x00ff00ff;
	gbm_bo_unmap(bo, map_data);
	printf("unmap ok\n");

	int dmabuf_fd = gbm_bo_get_fd(bo);
	if (dmabuf_fd < 0) { perror("gbm_bo_get_fd"); return (5); }
	printf("dmabuf_fd: %d (PRIME export)\n", dmabuf_fd);

	gbm_bo_destroy(bo);
	gbm_device_destroy(dev);
	close(fd);
	printf("OK\n");
	return (0);
}
