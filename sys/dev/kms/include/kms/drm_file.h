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
struct kms_syncobj;
TAILQ_HEAD(drm_gem_handle_list, drm_gem_handle);
TAILQ_HEAD(drm_pending_event_list, drm_pending_event);
TAILQ_HEAD(kms_syncobj_list, kms_syncobj);

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
	/*
	 * refs counts owners of this drm_file:
	 *   1 for the d_open (dropped by kms_file_dtor),
	 *   +1 per in-flight vblank/flip event that will call kms_send_event
	 *      from an IRQ context after mc->mutex has been dropped.
	 * Structure lives until refs hits 0.  kms_file_get / kms_file_put
	 * from drm_events.c IRQ paths keep it alive across the H4 race
	 * window between "detach event to ready list, drop mc->mutex" and
	 * "kms_send_event on pe->file".
	 */
	volatile u_int		 refs;
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

	/*
	 * Sync-object handle table.  Separate from the GEM handle table
	 * because Linux DRM keeps the two namespaces independent — a GEM
	 * handle of 5 and a syncobj handle of 5 are unrelated.  All
	 * syncobjs allocated here are stubs (no GPU-side fence backing);
	 * see drm_syncobj.c for the WAIT/RESET/SIGNAL semantics.
	 */
	struct sx			 syncobj_lock;
	struct kms_syncobj_list		 syncobjs;
	uint32_t			 next_syncobj_handle;
};

#endif /* _KMS_DRM_FILE_H_ */
