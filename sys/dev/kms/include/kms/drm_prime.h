/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * PRIME / DMA-BUF for the kms framework.
 *
 * Userspace export: a GEM handle on one drm_file becomes a regular
 * file descriptor that any other process can receive (e.g. over a
 * Unix-domain socket) and either mmap directly or re-import as a
 * GEM handle on its own drm_file via FD_TO_HANDLE.
 *
 * Implementation: a tiny FreeBSD `struct fileops` wraps each exported
 * GEM.  fo_mmap returns the GEM's cdev_pager so the dma-buf fd and
 * the original cdev mmap path produce identical pages.  fo_close
 * drops the GEM reference held for the exported lifetime.
 *
 * No LinuxKPI: only sys/file.h + sys/filedesc.h + vm primitives.
 */

#ifndef _KMS_DRM_PRIME_H_
#define _KMS_DRM_PRIME_H_

#include <sys/types.h>

struct drm_file;
struct drm_prime_handle;
struct drm_gem_object;
struct file;

int	kms_ioctl_prime_handle_to_fd(struct drm_file *file,
	    struct drm_prime_handle *args);
int	kms_ioctl_prime_fd_to_handle(struct drm_file *file,
	    struct drm_prime_handle *args);

/*
 * Cross-driver accessor.  If `fp` is a kms prime fd (returned by
 * DRM_IOCTL_PRIME_HANDLE_TO_FD), returns the underlying
 * drm_gem_object with an added reference.  Caller must drop the ref
 * via kms_gem_object_put() when done.  Returns NULL for any other
 * fd type.
 *
 * Intended for consumer drivers that want to import a kms-scanout BO
 * (e.g. fgpu importing a rk_kms framebuffer as a GPU BO backing).
 */
struct drm_gem_object *kms_prime_fd_to_gem(struct file *fp);

#endif /* _KMS_DRM_PRIME_H_ */
