/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#ifndef _KMS_DRM_FILE_H_
#define _KMS_DRM_FILE_H_

#include <sys/types.h>
#include <sys/queue.h>
#include <sys/sx.h>

struct drm_device;
struct drm_gem_handle;
TAILQ_HEAD(drm_gem_handle_list, drm_gem_handle);

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

	/*
	 * GEM handle table.  handle_lock protects the list and
	 * next_handle counter; held briefly across create/delete/lookup.
	 * Each handle holds one reference on its backing drm_gem_object;
	 * file dtor releases all of them via kms_gem_release_all.
	 */
	struct sx			 handle_lock;
	struct drm_gem_handle_list	 handles;
	uint32_t			 next_handle;
};

#endif /* _KMS_DRM_FILE_H_ */
