/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#ifndef _KMS_DRM_FILE_H_
#define _KMS_DRM_FILE_H_

#include <sys/types.h>
#include <sys/queue.h>

struct drm_device;

/*
 * Per-open state.  Allocated in d_open, freed in d_close.  Lives on
 * drm_device->files between those two calls.  All ioctl handlers
 * receive the drm_file as their first argument so they can find the
 * owning device and per-client state (auth magic, capabilities, etc.).
 */
struct drm_file {
	struct drm_device	*dev;
	bool			 authenticated;
	bool			 is_master;
	uint32_t		 magic;
	uint32_t		 client_caps;
	TAILQ_ENTRY(drm_file)	 link;
};

#endif /* _KMS_DRM_FILE_H_ */
