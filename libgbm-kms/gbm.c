/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * libgbm-kms — Generic Buffer Management on top of the kms framework.
 *
 * Every gbm_bo is a CREATE_DUMB GEM.  gbm_bo_get_fd is PRIME_HANDLE_TO_FD.
 * gbm_bo_map / unmap are MAP_DUMB + mmap.  Width/height/format/stride
 * are stashed at creation.  No Mesa, no LinuxKPI.
 */

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>

#include <gbm.h>

struct gbm_device {
	int	fd;
};

struct gbm_bo {
	struct gbm_device *dev;
	uint32_t	width;
	uint32_t	height;
	uint32_t	format;
	uint32_t	stride;
	uint32_t	handle;
	int		dma_buf_fd;	/* -1 until gbm_bo_get_fd */
	size_t		size;
};

struct gbm_map_data {
	void	*ptr;
	size_t	 size;
};

static int
gbm_format_bpp(uint32_t format)
{
	switch (format) {
	case GBM_FORMAT_C8:
	case GBM_FORMAT_R8:
		return (8);
	case GBM_FORMAT_RGB565:
		return (16);
	case GBM_FORMAT_XRGB8888:
	case GBM_FORMAT_XBGR8888:
	case GBM_FORMAT_ARGB8888:
	case GBM_FORMAT_ABGR8888:
		return (32);
	default:
		return (0);
	}
}

struct gbm_device *
gbm_create_device(int fd)
{
	struct gbm_device *dev;

	if (fd < 0) {
		errno = EINVAL;
		return (NULL);
	}
	dev = calloc(1, sizeof(*dev));
	if (dev == NULL)
		return (NULL);
	dev->fd = fd;
	return (dev);
}

void
gbm_device_destroy(struct gbm_device *dev)
{
	free(dev);	/* fd is caller-owned */
}

int
gbm_device_get_fd(struct gbm_device *dev)
{
	return (dev != NULL ? dev->fd : -1);
}

const char *
gbm_device_get_backend_name(struct gbm_device *dev __unused)
{
	return ("kms");
}

struct gbm_bo *
gbm_bo_create(struct gbm_device *dev, uint32_t width, uint32_t height,
    uint32_t format, uint32_t flags __unused)
{
	struct gbm_bo *bo;
	struct drm_mode_create_dumb cd;
	int bpp;

	if (dev == NULL || width == 0 || height == 0) {
		errno = EINVAL;
		return (NULL);
	}
	bpp = gbm_format_bpp(format);
	if (bpp == 0) {
		errno = EINVAL;
		return (NULL);
	}

	memset(&cd, 0, sizeof(cd));
	cd.width = width;
	cd.height = height;
	cd.bpp = bpp;
	if (ioctl(dev->fd, DRM_IOCTL_MODE_CREATE_DUMB, &cd) != 0)
		return (NULL);

	bo = calloc(1, sizeof(*bo));
	if (bo == NULL) {
		struct drm_mode_destroy_dumb dd = { .handle = cd.handle };
		(void)ioctl(dev->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd);
		errno = ENOMEM;
		return (NULL);
	}
	bo->dev = dev;
	bo->width = width;
	bo->height = height;
	bo->format = format;
	bo->stride = cd.pitch;
	bo->handle = cd.handle;
	bo->dma_buf_fd = -1;
	bo->size = cd.size;
	return (bo);
}

void
gbm_bo_destroy(struct gbm_bo *bo)
{
	struct drm_mode_destroy_dumb dd;

	if (bo == NULL)
		return;
	if (bo->dma_buf_fd >= 0)
		close(bo->dma_buf_fd);
	memset(&dd, 0, sizeof(dd));
	dd.handle = bo->handle;
	(void)ioctl(bo->dev->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd);
	free(bo);
}

uint32_t gbm_bo_get_width(struct gbm_bo *bo)   { return (bo->width); }
uint32_t gbm_bo_get_height(struct gbm_bo *bo)  { return (bo->height); }
uint32_t gbm_bo_get_stride(struct gbm_bo *bo)  { return (bo->stride); }
uint32_t gbm_bo_get_format(struct gbm_bo *bo)  { return (bo->format); }
size_t   gbm_bo_get_size(struct gbm_bo *bo)    { return (bo->size); }
struct gbm_device *gbm_bo_get_device(struct gbm_bo *bo) { return (bo->dev); }

union gbm_bo_handle
gbm_bo_get_handle(struct gbm_bo *bo)
{
	union gbm_bo_handle h = { 0 };

	if (bo != NULL)
		h.u32 = bo->handle;
	return (h);
}

int
gbm_bo_get_fd(struct gbm_bo *bo)
{
	struct drm_prime_handle ph;

	if (bo == NULL)
		return (-1);
	if (bo->dma_buf_fd >= 0)
		return (bo->dma_buf_fd);

	memset(&ph, 0, sizeof(ph));
	ph.handle = bo->handle;
	ph.flags = 0;	/* the kernel CLOEXEC is on by default in our impl */
	if (ioctl(bo->dev->fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &ph) != 0)
		return (-1);
	bo->dma_buf_fd = ph.fd;
	return (bo->dma_buf_fd);
}

void *
gbm_bo_map(struct gbm_bo *bo, uint32_t x, uint32_t y, uint32_t width,
    uint32_t height, uint32_t flags __unused, uint32_t *stride,
    void **map_data)
{
	struct drm_mode_map_dumb md;
	struct gbm_map_data *m;
	void *ptr;
	int prot;

	if (bo == NULL || stride == NULL || map_data == NULL) {
		errno = EINVAL;
		return (NULL);
	}
	if (x + width > bo->width || y + height > bo->height) {
		errno = EINVAL;
		return (NULL);
	}

	memset(&md, 0, sizeof(md));
	md.handle = bo->handle;
	if (ioctl(bo->dev->fd, DRM_IOCTL_MODE_MAP_DUMB, &md) != 0)
		return (NULL);

	prot = 0;
	if (flags & GBM_BO_TRANSFER_READ)  prot |= PROT_READ;
	if (flags & GBM_BO_TRANSFER_WRITE) prot |= PROT_WRITE;
	if (prot == 0) prot = PROT_READ | PROT_WRITE;

	/*
	 * Mesa GBM lets you map a sub-rectangle but the underlying
	 * resource is page-granular, so we mmap the whole BO and return
	 * a pointer offset to the (x, y) corner.  Stride is the BO's
	 * stride (caller passes width/height for clipping bookkeeping).
	 */
	ptr = mmap(NULL, bo->size, prot, MAP_SHARED, bo->dev->fd, md.offset);
	if (ptr == MAP_FAILED)
		return (NULL);

	m = calloc(1, sizeof(*m));
	if (m == NULL) {
		munmap(ptr, bo->size);
		errno = ENOMEM;
		return (NULL);
	}
	m->ptr = ptr;
	m->size = bo->size;
	*map_data = m;
	*stride = bo->stride;
	return ((char *)ptr + y * bo->stride + x * (gbm_format_bpp(bo->format) / 8));
}

void
gbm_bo_unmap(struct gbm_bo *bo __unused, void *map_data)
{
	struct gbm_map_data *m = map_data;

	if (m == NULL)
		return;
	munmap(m->ptr, m->size);
	free(m);
}

struct gbm_bo *
gbm_bo_import(struct gbm_device *dev, uint32_t type, void *buffer,
    uint32_t flags __unused)
{
	struct drm_prime_handle ph;
	struct gbm_bo *bo;

	if (dev == NULL || buffer == NULL || type != GBM_BO_IMPORT_FD) {
		errno = EINVAL;
		return (NULL);
	}
	memset(&ph, 0, sizeof(ph));
	ph.fd = *(int *)buffer;
	if (ioctl(dev->fd, DRM_IOCTL_PRIME_FD_TO_HANDLE, &ph) != 0)
		return (NULL);

	bo = calloc(1, sizeof(*bo));
	if (bo == NULL) {
		errno = ENOMEM;
		return (NULL);
	}
	bo->dev = dev;
	bo->handle = ph.handle;
	bo->dma_buf_fd = -1;
	/*
	 * Geometry of an imported BO isn't carried by PRIME alone; the
	 * caller must set width/height/stride/format via the FD's
	 * out-of-band metadata path (Wayland dmabuf protocol carries it).
	 * For our smoke-test case where producer + consumer share this
	 * library, the import is just a handle promotion.
	 */
	return (bo);
}
