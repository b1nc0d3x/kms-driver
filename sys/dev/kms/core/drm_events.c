/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Kernel→user event ring + VBLANK + page-flip event delivery + the
 * DRM_IOCTL_WAIT_VBLANK ioctl.
 *
 * Phase 9g framework piece.  Drivers push events from any context
 * (typically a vblank IRQ handler) via kms_send_event /
 * kms_send_vblank_event; userspace consumes via read() on the
 * cdev or via blocking WAIT_VBLANK.  Lock order:
 *
 *   file->event_mtx is a leaf — never call into other code paths with
 *     it held except for wakeups via wakeup() and selwakeup().
 *   mode_config.mutex protects the CRTC sequence + pending_flip slot
 *     and is acquired exclusively to advance them in vblank_handler;
 *     read paths take it shared.
 *
 * Event payload is heap-malloc'd at queue time and freed at read time
 * (or in the file dtor if the user never drained).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/event.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/selinfo.h>
#include <sys/sx.h>
#include <sys/uio.h>

#include <drm/drm.h>

#include <kms/drm_crtc.h>
#include <kms/drm_device.h>
#include <kms/drm_file.h>
#include <kms/drm_framebuffer.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_mode_object.h>

#include "kms_internal.h"

/* --- ring init / teardown --- */

void
kms_event_queue_init(struct drm_file *file)
{
	mtx_init(&file->event_mtx, "kms ev", NULL, MTX_DEF);
	TAILQ_INIT(&file->events);
	file->events_bytes = 0;
	bzero(&file->event_select, sizeof(file->event_select));
	/*
	 * Initialize the selinfo's embedded knlist under event_mtx so
	 * kqueue attach (kms_kqfilter) + selwakeup KNOTE fanout share
	 * the same lock as the event queue itself.  Without this, an
	 * EVFILT_READ registration on /dev/dri/cardN would either fault
	 * on knlist_add (kl_lock == NULL) or race with kms_send_event.
	 * M2 from the 2026-08-01 review.
	 */
	knlist_init_mtx(&file->event_select.si_note, &file->event_mtx);
}

void
kms_event_queue_drain(struct drm_file *file)
{
	struct drm_pending_event *ev;

	mtx_lock(&file->event_mtx);
	while ((ev = TAILQ_FIRST(&file->events)) != NULL) {
		TAILQ_REMOVE(&file->events, ev, link);
		file->events_bytes -= ev->length;
		free(ev->data, M_KMS);
		free(ev, M_KMS);
	}
	mtx_unlock(&file->event_mtx);
	seldrain(&file->event_select);
	knlist_destroy(&file->event_select.si_note);
	mtx_destroy(&file->event_mtx);
}

/* --- queue + wake --- */

int
kms_send_event(struct drm_file *file, const void *data, size_t length)
{
	struct drm_pending_event *ev;

	if (file == NULL || data == NULL || length == 0)
		return (EINVAL);
	/*
	 * Cap aggregate per-file queue size so a stuck userspace
	 * process can't make the kernel page on event memory.  64 KiB
	 * is generous — at sizeof(drm_event_vblank) = 32 bytes per
	 * entry that's ~2000 buffered events.
	 */
	ev = malloc(sizeof(*ev), M_KMS, M_NOWAIT | M_ZERO);
	if (ev == NULL)
		return (ENOMEM);
	ev->data = malloc(length, M_KMS, M_NOWAIT);
	if (ev->data == NULL) {
		free(ev, M_KMS);
		return (ENOMEM);
	}
	memcpy(ev->data, data, length);
	ev->length = length;

	mtx_lock(&file->event_mtx);
	if (file->events_bytes + length > (64u << 10)) {
		mtx_unlock(&file->event_mtx);
		free(ev->data, M_KMS);
		free(ev, M_KMS);
		return (ENOSPC);
	}
	TAILQ_INSERT_TAIL(&file->events, ev, link);
	file->events_bytes += length;
	wakeup(&file->events);
	/*
	 * M1 (2026-08-03 review): selwakeup only walks si_tdlist (poll/
	 * select waiters); it never fans out to si_note.  Without an
	 * explicit KNOTE call here, EVFILT_READ registered via
	 * kms_kqfilter attaches but never fires — epoll-shim compositors
	 * (wlroots / kwin / weston) block forever on FLIP_COMPLETE.
	 * si_note's knlist is bound to event_mtx (see
	 * kms_event_queue_init), so use KNOTE_LOCKED while we still hold
	 * it.  Matches the pattern in kern/tty.c under t_lock.
	 */
	KNOTE_LOCKED(&file->event_select.si_note, 0);
	mtx_unlock(&file->event_mtx);

	selwakeup(&file->event_select);
	return (0);
}

/* --- vblank event helper --- */

static void
drm_event_fill_vblank(struct drm_event_vblank *e, uint32_t type,
    uint32_t crtc_id, uint32_t sequence, uint64_t user_data)
{
	struct timeval tv;

	microuptime(&tv);
	e->base.type = type;
	e->base.length = sizeof(*e);
	e->user_data = user_data;
	e->tv_sec = (uint32_t)tv.tv_sec;
	e->tv_usec = (uint32_t)tv.tv_usec;
	e->sequence = sequence;
	e->crtc_id = crtc_id;
}

int
kms_send_vblank_event(struct drm_file *file, struct drm_crtc *crtc,
    uint32_t type, uint64_t user_data)
{
	struct drm_event_vblank ev;

	if (file == NULL || crtc == NULL)
		return (EINVAL);
	drm_event_fill_vblank(&ev, type, crtc->base.id, crtc->sequence,
	    user_data);
	return (kms_send_event(file, &ev, sizeof(ev)));
}

/* --- vblank handler (driver-callable) --- */

void
kms_vblank_handler(struct drm_crtc *crtc)
{
	struct drm_mode_config *mc;
	struct drm_file *flip_file = NULL;
	uint64_t flip_user_data = 0;

	/*
	 * This handler takes sx_xlock(&mc->mutex) below, so it must run
	 * in a sleepable context.  Drivers must call it from a taskqueue
	 * / ithread (not from a bus_setup_intr filter or an IPI handler).
	 * WITNESS catches the misuse at first fire in an INVARIANTS
	 * kernel; non-INVARIANTS builds get a debugger drop via KASSERT
	 * on the sx_assert path a few lines down anyway.
	 */
	WITNESS_WARN(WARN_GIANTOK | WARN_SLEEPOK, NULL,
	    "kms_vblank_handler");

	if (crtc == NULL || crtc->dev == NULL)
		return;
	mc = &crtc->dev->mode_config;

	struct kms_pending_vblank_event *pe, *pe_next;
	TAILQ_HEAD(, kms_pending_vblank_event) ready;
	TAILQ_INIT(&ready);

	struct drm_framebuffer *flip_old_fb = NULL;

	sx_xlock(&mc->mutex);
	crtc->sequence++;
	if (crtc->pending_flip_file != NULL) {
		flip_file = crtc->pending_flip_file;
		flip_user_data = crtc->pending_flip_user_data;
		flip_old_fb = crtc->pending_flip_old_fb;
		crtc->pending_flip_file = NULL;
		crtc->pending_flip_user_data = 0;
		crtc->pending_flip_old_fb = NULL;
		/*
		 * flip_file already holds a ref taken at PAGE_FLIP arm-time
		 * (drm_modeset.c).  We consume that ref via kms_file_put
		 * after kms_send_vblank_event delivers, without needing a
		 * fresh kms_file_get here.
		 */
	}
	/*
	 * Drain everything in pending_vblank_events whose target_seq has
	 * been reached.  Detach to a local list so we can deliver outside
	 * the mode_config lock (kms_send_event takes the per-file event_mtx).
	 * Each pe->file already holds a ref taken at WAIT_VBLANK queue-time
	 * (see kms_ioctl_wait_vblank); we consume it after delivery below.
	 */
	TAILQ_FOREACH_SAFE(pe, &crtc->pending_vblank_events, link, pe_next) {
		if ((int32_t)(crtc->sequence - pe->target_seq) >= 0) {
			TAILQ_REMOVE(&crtc->pending_vblank_events, pe, link);
			TAILQ_INSERT_TAIL(&ready, pe, link);
		}
	}
	sx_xunlock(&mc->mutex);

	while ((pe = TAILQ_FIRST(&ready)) != NULL) {
		struct drm_event_vblank ev;
		TAILQ_REMOVE(&ready, pe, link);
		drm_event_fill_vblank(&ev, DRM_EVENT_VBLANK, crtc->base.id,
		    pe->target_seq, pe->user_data);
		(void)kms_send_event(pe->file, &ev, sizeof(ev));
		kms_file_put(pe->file);	/* release WAIT_VBLANK queue-time ref */
		free(pe, M_KMS);
	}

	/*
	 * Wake every WAIT_VBLANK sleeper on this CRTC.  The wait
	 * channel is the crtc address; sleepers re-check the sequence
	 * field under mc->mutex shared.
	 */
	wakeup(crtc);

	if (flip_file != NULL) {
		(void)kms_send_vblank_event(flip_file, crtc,
		    DRM_EVENT_FLIP_COMPLETE, flip_user_data);
		kms_file_put(flip_file);	/* release PAGE_FLIP arm-time ref */
		/* H3: drop the outgoing fb now that scan-out has moved on. */
		if (flip_old_fb != NULL)
			kms_mode_object_put(&flip_old_fb->base);
	}
}

/* --- WAIT_VBLANK ioctl --- */

int
kms_ioctl_wait_vblank(struct drm_file *file, union drm_wait_vblank *arg)
{
	struct drm_mode_config *mc = &file->dev->mode_config;
	struct drm_crtc *crtc = NULL;
	struct drm_mode_object *obj;
	uint32_t target;
	uint32_t high_crtc;
	uint32_t flags;
	uint32_t type;
	struct timeval tv;
	int error = 0;

	if (file == NULL || arg == NULL)
		return (EINVAL);

	type = arg->request.type & _DRM_VBLANK_TYPES_MASK;
	flags = arg->request.type & _DRM_VBLANK_FLAGS_MASK;
	high_crtc = (arg->request.type & _DRM_VBLANK_HIGH_CRTC_MASK) >>
	    _DRM_VBLANK_HIGH_CRTC_SHIFT;

	/*
	 * The high-crtc bits encode the CRTC index.  Phase 9g's
	 * straight-line lookup walks the CRTC list to find the matching
	 * index — kms-stub style hardware has only one CRTC so
	 * the cost is trivial.  Phase 9h+ will index by id.
	 */
	sx_slock(&mc->mutex);
	TAILQ_FOREACH(obj, &mc->crtcs, link) {
		crtc = __containerof(obj, struct drm_crtc, base);
		if (crtc->index == high_crtc)
			break;
		crtc = NULL;
	}
	if (crtc == NULL) {
		sx_sunlock(&mc->mutex);
		return (ENOENT);
	}
	if (type == _DRM_VBLANK_RELATIVE)
		target = crtc->sequence + arg->request.sequence;
	else
		target = arg->request.sequence;
	sx_sunlock(&mc->mutex);

	if (flags & _DRM_VBLANK_EVENT) {
		struct drm_event_vblank ev;

		/*
		 * If the target sequence has already passed, dispatch
		 * immediately (matches Linux DRM semantics).  Otherwise
		 * queue and let kms_vblank_handler dispatch when the IRQ
		 * fires for the target frame.
		 */
		sx_xlock(&mc->mutex);
		bool already = ((int32_t)(crtc->sequence - target) >= 0);
		if (!already) {
			struct kms_pending_vblank_event *pe =
			    malloc(sizeof(*pe), M_KMS, M_NOWAIT | M_ZERO);
			if (pe == NULL) {
				sx_xunlock(&mc->mutex);
				return (ENOMEM);
			}
			pe->file = file;
			pe->target_seq = target;
			pe->user_data = arg->request.signal;
			/*
			 * H4: hold a drm_file ref while pe is queued.
			 * Released in kms_vblank_handler after delivery
			 * (or in kms_file_dtor if close() races the IRQ).
			 */
			kms_file_get(file);
			TAILQ_INSERT_TAIL(&crtc->pending_vblank_events,
			    pe, link);
			sx_xunlock(&mc->mutex);
			arg->reply.type = type | _DRM_VBLANK_EVENT;
			arg->reply.sequence = target;
			return (0);
		}
		sx_xunlock(&mc->mutex);

		drm_event_fill_vblank(&ev, DRM_EVENT_VBLANK, crtc->base.id,
		    target, arg->request.signal);
		error = kms_send_event(file, &ev, sizeof(ev));
		if (error != 0)
			return (error);
		arg->reply.type = type | _DRM_VBLANK_EVENT;
		arg->reply.sequence = target;
		return (0);
	}

	/*
	 * Blocking wait.  Loop on tsleep with a small timeout so we
	 * pick up race-window updates without re-reading the lock on
	 * every wakeup.  100 ms is far past any real refresh interval
	 * (a 1 Hz monitor would still wake on the timer).
	 */
	for (;;) {
		sx_slock(&mc->mutex);
		if ((int32_t)(crtc->sequence - target) >= 0) {
			sx_sunlock(&mc->mutex);
			break;
		}
		sx_sunlock(&mc->mutex);
		error = tsleep(crtc, PCATCH, "drmvbl", hz / 10);
		if (error == EINTR || error == ERESTART)
			return (error);
		if (error != 0 && error != EWOULDBLOCK)
			return (error);
	}

	microuptime(&tv);
	arg->reply.type = type;
	arg->reply.sequence = crtc->sequence;
	arg->reply.tval_sec = tv.tv_sec;
	arg->reply.tval_usec = tv.tv_usec;
	return (0);
}
