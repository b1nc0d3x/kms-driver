/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * libgbm-kms: GBM (Generic Buffer Management) implementation backed
 * by the kms framework's CREATE_DUMB + PRIME ioctls.
 *
 * ABI-compatible with Mesa's <gbm.h> for the modesetting / software-
 * composition subset.  No Mesa, no LinuxKPI, no GL.
 *
 * What's covered:
 *   - gbm_device   wraps a /dev/dri/cardN fd
 *   - gbm_bo       wraps a CREATE_DUMB GEM with stride / handle / fd
 *   - gbm_bo_map / gbm_bo_unmap for CPU-side rendering (pixman, etc.)
 *
 * What's NOT (yet) covered:
 *   - gbm_surface  (front/back-buffer abstraction used by EGL)
 *   - GBM_BO_USE_RENDERING + modifier negotiation (need a GPU
 *     command-stream driver first)
 */

#ifndef _GBM_KMS_H_
#define _GBM_KMS_H_

#include <stdint.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct gbm_device;
struct gbm_bo;

/* DRM_FORMAT_* fourccs, mirrored.  Add as needed. */
#define GBM_FORMAT_C8           0x20203843
#define GBM_FORMAT_R8           0x20203852
#define GBM_FORMAT_RGB565       0x36315052
#define GBM_FORMAT_XRGB8888     0x34325258
#define GBM_FORMAT_XBGR8888     0x34324258
#define GBM_FORMAT_ARGB8888     0x34325241
#define GBM_FORMAT_ABGR8888     0x34324241

/* Use-case bits (matches Mesa gbm.h). */
#define GBM_BO_USE_SCANOUT      (1 << 0)
#define GBM_BO_USE_CURSOR       (1 << 1)
#define GBM_BO_USE_RENDERING    (1 << 2)
#define GBM_BO_USE_WRITE        (1 << 3)
#define GBM_BO_USE_LINEAR       (1 << 4)

/* Transfer flags for gbm_bo_map (Mesa-compatible). */
#define GBM_BO_TRANSFER_READ        (1 << 0)
#define GBM_BO_TRANSFER_WRITE       (1 << 1)
#define GBM_BO_TRANSFER_READ_WRITE  (GBM_BO_TRANSFER_READ | GBM_BO_TRANSFER_WRITE)

union gbm_bo_handle {
	void	*ptr;
	int32_t	 s32;
	uint32_t u32;
	int64_t	 s64;
	uint64_t u64;
};

struct gbm_device *gbm_create_device(int fd);
void	gbm_device_destroy(struct gbm_device *gbm);
int	gbm_device_get_fd(struct gbm_device *gbm);
const char *gbm_device_get_backend_name(struct gbm_device *gbm);

struct gbm_bo *gbm_bo_create(struct gbm_device *gbm,
	    uint32_t width, uint32_t height, uint32_t format, uint32_t flags);
struct gbm_bo *gbm_bo_import(struct gbm_device *gbm,
	    uint32_t type, void *buffer, uint32_t flags);
#define GBM_BO_IMPORT_FD 0x5503		/* one type carried as the fd */
void	gbm_bo_destroy(struct gbm_bo *bo);

uint32_t gbm_bo_get_width(struct gbm_bo *bo);
uint32_t gbm_bo_get_height(struct gbm_bo *bo);
uint32_t gbm_bo_get_stride(struct gbm_bo *bo);
uint32_t gbm_bo_get_format(struct gbm_bo *bo);
struct gbm_device *gbm_bo_get_device(struct gbm_bo *bo);
union gbm_bo_handle gbm_bo_get_handle(struct gbm_bo *bo);
int	gbm_bo_get_fd(struct gbm_bo *bo);
size_t	gbm_bo_get_size(struct gbm_bo *bo);

void *	gbm_bo_map(struct gbm_bo *bo, uint32_t x, uint32_t y,
	    uint32_t width, uint32_t height, uint32_t flags,
	    uint32_t *stride, void **map_data);
void	gbm_bo_unmap(struct gbm_bo *bo, void *map_data);

#ifdef __cplusplus
}
#endif

#endif /* _GBM_KMS_H_ */
