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

int	kms_ioctl_prime_handle_to_fd(struct drm_file *file,
	    struct drm_prime_handle *args);
int	kms_ioctl_prime_fd_to_handle(struct drm_file *file,
	    struct drm_prime_handle *args);

#endif /* _KMS_DRM_PRIME_H_ */
