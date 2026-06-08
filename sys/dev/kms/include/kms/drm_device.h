/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 */

#ifndef _KMS_DRM_DEVICE_H_
#define _KMS_DRM_DEVICE_H_

#include <sys/types.h>
#include <sys/queue.h>
#include <sys/sx.h>

#include <kms/drm_mode_config.h>

struct cdev;
struct drm_driver;
struct drm_file;

/*
 * One per registered DRM card.  Created by the driver via
 * kms_dev_register(), torn down by kms_dev_unregister().
 *
 * Lifetime: the device storage is refcounted independently from the
 * cdev.  kms_dev_register holds the initial reference and creates
 * the cdev.  kms_dev_unregister destroys the cdev (so no new
 * opens succeed), removes the device from the registry, and drops the
 * initial reference.  Each open file holds an additional reference,
 * dropped in the file dtor.  The storage is freed (and the embedded
 * mode_config / dev_lock destroyed) when the last reference falls —
 * which may be either the unregister call itself (no opens outstanding)
 * or the last file close after unregister.
 */
struct drm_gem_object;
TAILQ_HEAD(drm_gem_object_list, drm_gem_object);

struct drm_device {
	struct sx			 dev_lock;
	const struct drm_driver		*driver;
	struct cdev			*cdev;
	int				 minor;
	u_int				 open_count;	/* live opens */
	volatile u_int			 refs;		/* refcount(9) */
	TAILQ_HEAD(, drm_file)		 files;
	TAILQ_ENTRY(drm_device)		 link;
	void				*driver_priv;
	struct drm_mode_config		 mode_config;

	/*
	 * GEM bookkeeping.  gem_lock protects the gem_objects list and
	 * the mmap_offset_counter; held briefly during create / lookup /
	 * destroy.  Counter monotonically advances by one PAGE_SIZE-
	 * aligned slot per BO so cdev d_mmap_single can map an offset
	 * back to a single object via list scan.
	 */
	struct sx			 gem_lock;
	struct drm_gem_object_list	 gem_objects;
	uint64_t			 mmap_offset_counter;
};

#endif /* _KMS_DRM_DEVICE_H_ */
