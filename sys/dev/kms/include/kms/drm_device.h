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
 * Lifetime: from successful register() until last open file is closed
 * after unregister().  Closes on outstanding files block unregister.
 */
struct drm_device {
	struct sx			 dev_lock;
	const struct drm_driver		*driver;
	struct cdev			*cdev;
	int				 minor;
	int				 open_count;
	TAILQ_HEAD(, drm_file)		 files;
	TAILQ_ENTRY(drm_device)		 link;
	void				*driver_priv;
	struct drm_mode_config		 mode_config;
};

#endif /* _KMS_DRM_DEVICE_H_ */
