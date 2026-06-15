/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#ifndef _KMS_DRM_FILE_H_
#define _KMS_DRM_FILE_H_

#include <sys/types.h>
#include <sys/queue.h>
#include <sys/mutex.h>
#include <sys/selinfo.h>
#include <sys/sx.h>

struct drm_device;
struct drm_gem_handle;
struct drm_pending_event;
TAILQ_HEAD(drm_gem_handle_list, drm_gem_handle);
TAILQ_HEAD(drm_pending_event_list, drm_pending_event);

/*
 * Pending kernel→user event queued on a drm_file's event ring.  The
 * payload is variable-length (a drm_event header followed by the
 * type-specific bytes — drm_event_vblank for VBLANK/FLIP_COMPLETE).
 * read() drains entries one at a time and only when the user buffer
 * is large enough to hold the whole event, matching Linux semantics.
 */
struct drm_pending_event {
	uint32_t			 length;
	void				*data;	/* malloc'd, length bytes */
	TAILQ_ENTRY(drm_pending_event)	 link;
};

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
	/*
	 * Render-node opens (/dev/dri/renderD<128+N>) get is_render_node=true
	 * and a tighter ioctl whitelist enforced in kms_ioctl: no SET_MASTER,
	 * no MODE_SETCRTC / PAGE_FLIP / ATOMIC, no ADDFB(2) / RMFB,
	 * no WAIT_VBLANK.  PRIME + dumb-buffer + GET_CAP / GET_RESOURCES
	 * stay permitted so Mesa can allocate render buffers without
	 * needing master.
	 */
	bool			 is_render_node;
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

	/*
	 * Kernel→user event ring.  event_mtx protects events list +
	 * pending byte count + the cv-equivalent wait channel.  Drivers
	 * push events via kms_send_event from any context (most
	 * commonly an IRQ vblank handler); userspace pops via read().
	 * Select/poll integration is via the selinfo (Phase 9g+1).
	 */
	struct mtx			 event_mtx;
	struct drm_pending_event_list	 events;
	uint32_t			 events_bytes;
	struct selinfo			 event_select;
};

#endif /* _KMS_DRM_FILE_H_ */
