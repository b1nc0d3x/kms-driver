/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Stub-level DRM sync-object handling.  Real timeline semantics would
 * require backing each syncobj with a deferred fence the GPU command
 * stream can signal asynchronously; we don't have GPU command
 * submission yet, so every syncobj is treated as already-signaled.
 *
 * What works for the compositor:
 *   - CREATE / DESTROY round-trips so kwin's per-frame allocator stops
 *     spinning on ENOTTY (the symptom that drove this code in).
 *   - WAIT / RESET / SIGNAL return success after validating that the
 *     referenced handles exist.
 *
 * What HANDLE_TO_FD does:
 *   Returns an eventfd primed with count=1 so a userspace poll(POLLIN)
 *   returns immediately.  Compositors (kwin_wayland in particular) can't
 *   accept ENOTTY here — they hot-spin retrying — but they will treat
 *   any poll-ready fd as an already-signaled fence and move on.  We
 *   don't distinguish EXPORT_SYNC_FILE from raw syncobj export: with no
 *   GPU-side backing every fence is already-signaled and both fd shapes
 *   collapse to the same "poll ready now" contract.
 *
 * What's intentionally absent:
 *   - FD_TO_HANDLE: not yet hit in traces; add a stub when a caller
 *     needs it.
 *   - Timeline variants (TIMELINE_WAIT / TIMELINE_SIGNAL / QUERY /
 *     TRANSFER / EVENTFD): better to fall back than lie about timeline
 *     points, and no observed caller needs them.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/fcntl.h>
#include <sys/file.h>		/* struct file, must precede eventfd.h */
#include <sys/filedesc.h>
#include <sys/eventfd.h>
#include <sys/malloc.h>
#include <sys/queue.h>
#include <sys/sx.h>

#include <drm/drm.h>

#include <kms/drm_file.h>

#include "kms_internal.h"

MALLOC_DECLARE(M_KMS);

struct kms_syncobj {
	uint32_t			 id;
	TAILQ_ENTRY(kms_syncobj)	 link;
};

static struct kms_syncobj *
kms_syncobj_lookup_locked(struct drm_file *file, uint32_t handle)
{
	struct kms_syncobj *s;

	TAILQ_FOREACH(s, &file->syncobjs, link) {
		if (s->id == handle)
			return (s);
	}
	return (NULL);
}

int
kms_ioctl_syncobj_create(struct drm_file *file,
    struct drm_syncobj_create *args)
{
	struct kms_syncobj *s;

	if ((args->flags & ~DRM_SYNCOBJ_CREATE_SIGNALED) != 0)
		return (EINVAL);
	s = malloc(sizeof(*s), M_KMS, M_WAITOK | M_ZERO);

	sx_xlock(&file->syncobj_lock);
	if (file->next_syncobj_handle == UINT32_MAX) {
		sx_xunlock(&file->syncobj_lock);
		free(s, M_KMS);
		return (ENOSPC);
	}
	s->id = ++file->next_syncobj_handle;
	TAILQ_INSERT_TAIL(&file->syncobjs, s, link);
	sx_xunlock(&file->syncobj_lock);

	args->handle = s->id;
	return (0);
}

int
kms_ioctl_syncobj_destroy(struct drm_file *file,
    struct drm_syncobj_destroy *args)
{
	struct kms_syncobj *s;

	sx_xlock(&file->syncobj_lock);
	s = kms_syncobj_lookup_locked(file, args->handle);
	if (s == NULL) {
		sx_xunlock(&file->syncobj_lock);
		return (ENOENT);
	}
	TAILQ_REMOVE(&file->syncobjs, s, link);
	sx_xunlock(&file->syncobj_lock);

	free(s, M_KMS);
	return (0);
}

/*
 * Copy in an array of syncobj handles described by (uptr, count) and
 * verify each one resolves to a live syncobj on this file.  The caller
 * frees the returned buffer; on failure the buffer is freed here.
 * Caps count_handles at 4096 to keep a single hostile WAIT from
 * allocating an unbounded kernel buffer.
 */
static int
kms_syncobj_copyin_handles(struct drm_file *file, uint64_t uptr,
    uint32_t count, uint32_t **handles_out)
{
	uint32_t *handles;
	size_t bytes;
	int error;
	uint32_t i;

	if (count == 0 || count > 4096)
		return (EINVAL);

	bytes = (size_t)count * sizeof(uint32_t);
	handles = malloc(bytes, M_TEMP, M_WAITOK);
	error = copyin((void *)(uintptr_t)uptr, handles, bytes);
	if (error != 0) {
		free(handles, M_TEMP);
		return (error);
	}

	sx_slock(&file->syncobj_lock);
	for (i = 0; i < count; i++) {
		if (kms_syncobj_lookup_locked(file, handles[i]) == NULL) {
			sx_sunlock(&file->syncobj_lock);
			free(handles, M_TEMP);
			return (ENOENT);
		}
	}
	sx_sunlock(&file->syncobj_lock);

	*handles_out = handles;
	return (0);
}

int
kms_ioctl_syncobj_wait(struct drm_file *file,
    struct drm_syncobj_wait *args)
{
	uint32_t *handles;
	int error;

	error = kms_syncobj_copyin_handles(file, args->handles,
	    args->count_handles, &handles);
	if (error != 0)
		return (error);
	free(handles, M_TEMP);

	/*
	 * All syncobjs are treated as already-signaled.  Report handle 0
	 * (the first) as the one that signaled to satisfy the "not waiting
	 * for all" caller contract.
	 */
	args->first_signaled = 0;
	args->pad = 0;
	return (0);
}

int
kms_ioctl_syncobj_reset(struct drm_file *file,
    struct drm_syncobj_array *args)
{
	uint32_t *handles;
	int error;

	error = kms_syncobj_copyin_handles(file, args->handles,
	    args->count_handles, &handles);
	if (error != 0)
		return (error);
	free(handles, M_TEMP);
	return (0);
}

int
kms_ioctl_syncobj_signal(struct drm_file *file,
    struct drm_syncobj_array *args)
{
	uint32_t *handles;
	int error;

	error = kms_syncobj_copyin_handles(file, args->handles,
	    args->count_handles, &handles);
	if (error != 0)
		return (error);
	free(handles, M_TEMP);
	return (0);
}

int
kms_ioctl_syncobj_handle_to_fd(struct thread *td, struct drm_file *file,
    struct drm_syncobj_handle *args)
{
	struct file *fp;
	int error, fd;

	if (args->pad != 0)
		return (EINVAL);

	sx_slock(&file->syncobj_lock);
	if (kms_syncobj_lookup_locked(file, args->handle) == NULL) {
		sx_sunlock(&file->syncobj_lock);
		return (ENOENT);
	}
	sx_sunlock(&file->syncobj_lock);

	error = falloc_noinstall(td, &fp);
	if (error != 0)
		return (error);
	error = eventfd_create_file(td, fp, 1, EFD_CLOEXEC);
	if (error != 0) {
		fdrop(fp, td);
		return (error);
	}
	error = finstall(td, fp, &fd, O_CLOEXEC, NULL);
	fdrop(fp, td);
	if (error != 0)
		return (error);

	args->fd = fd;
	return (0);
}

void
kms_syncobj_release_all(struct drm_file *file)
{
	struct kms_syncobj *s;

	if (file == NULL)
		return;
	for (;;) {
		sx_xlock(&file->syncobj_lock);
		s = TAILQ_FIRST(&file->syncobjs);
		if (s == NULL) {
			sx_xunlock(&file->syncobj_lock);
			break;
		}
		TAILQ_REMOVE(&file->syncobjs, s, link);
		sx_xunlock(&file->syncobj_lock);
		free(s, M_KMS);
	}
}
