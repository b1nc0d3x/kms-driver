/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Minimal GEM (Graphics Execution Manager) layer for kms.
 *
 * Phase 6 scope:
 *   - struct drm_gem_object: refcounted kernel-side buffer object
 *     backed by contiguous wired pages.
 *   - Per-drm_file handle table mapping 32-bit user handles to
 *     drm_gem_object pointers.
 *   - Dumb-buffer creation (CREATE_DUMB), offset assignment for mmap
 *     (MAP_DUMB), and lifecycle (DESTROY_DUMB).
 *   - cdev_pager wiring so userspace mmap returns the same kernel
 *     pages the BO holds.
 *
 * Full GEM (named objects, GEM_OPEN, PRIME fd-handle import/export)
 * isn't needed for the rk_drm port and is deferred.  kms is a
 * dumb-buffer KMS framework first; if a future driver needs PRIME,
 * the handle table is the place that grows.
 */

#ifndef _KMS_DRM_GEM_H_
#define _KMS_DRM_GEM_H_

#include <sys/types.h>
#include <sys/queue.h>

struct drm_device;
struct drm_file;
struct drm_gem_object;
struct vm_page;
typedef struct vm_object *vm_object_t;

/*
 * Kernel-side buffer object.  Lifetime is refcounted independently
 * from any drm_file handle table membership: PRIME would let two
 * file handles share one BO, and the GEM object itself outlives any
 * single handle while a userspace mmap is still active.
 *
 * Pages are wired and pre-inserted into the cdev_pager at create
 * time, so vm_pager_get_pages on the userspace mmap path never has
 * to fault them in.  Same shape as virtio_drm and tegra.
 */
struct drm_gem_object {
	struct drm_device		*dev;
	size_t				 size;		/* bytes, page-
							 * rounded */
	size_t				 npages;
	struct vm_page			**pages;
	vm_object_t			 pager;		/* OBJT_MGTDEVICE */

	uint64_t			 mmap_offset;	/* userspace mmap
							 * key */
	volatile u_int			 refs;

	TAILQ_ENTRY(drm_gem_object)	 device_link;	/* on dev->gem_objects */
};

/*
 * Allocate a GEM object backed by `size` bytes of contiguous wired
 * memory.  Size is rounded up to a multiple of PAGE_SIZE.  On success
 * the returned object has refs=1 and is inserted into the device's
 * GEM object list.  The mmap_offset is assigned automatically.
 *
 * Returns the new object on success; NULL on allocation failure.
 */
struct drm_gem_object *kms_gem_object_create(struct drm_device *dev,
	    size_t size);

/*
 * Acquire a reference to a GEM object.  Used by handle-table inserts
 * and by the cdev d_mmap_single path that pins the object for the
 * duration of the user mapping.
 */
void	kms_gem_object_get(struct drm_gem_object *obj);

/*
 * Drop a reference.  Last release tears down pages, the cdev_pager,
 * removes the object from the device list, and frees the storage.
 */
void	kms_gem_object_put(struct drm_gem_object *obj);

/*
 * Look up a GEM object by its mmap_offset on this device.  Returns
 * a referenced object on success (caller must put), NULL if no
 * object owns that offset.  Used from the cdev d_mmap_single path.
 */
struct drm_gem_object *kms_gem_object_lookup_offset(struct drm_device *dev,
	    uint64_t offset);

/*
 * Range-based sibling used by the per-page d_mmap path — returns the
 * GEM object whose [mmap_offset, mmap_offset+size) window contains
 * offset, along with that window's base so the caller can compute the
 * intra-object page index.  Caller must put the returned reference.
 */
struct drm_gem_object *kms_gem_object_lookup_offset_containing(
	    struct drm_device *dev, uint64_t offset, uint64_t *base_out);

/*
 * Per-drm_file handle table operations.  Handles are 32-bit ids
 * unique within one drm_file; the same GEM object may receive
 * different handles in different files (Phase 9+ PRIME import) but
 * Phase 6 has a 1:1 file-owns-object model.
 *
 * _create installs the object into the file table, bumps the
 * object's refcount, and returns the allocated handle.  _delete
 * removes the handle and drops the file's ref.  _lookup returns a
 * referenced object (caller must put) or NULL.
 */
int	kms_gem_handle_create(struct drm_file *file,
	    struct drm_gem_object *obj, uint32_t *handle_out);
int	kms_gem_handle_delete(struct drm_file *file, uint32_t handle);
struct drm_gem_object *kms_gem_handle_lookup(struct drm_file *file,
	    uint32_t handle);

/*
 * H2 mmap-ownership gate: true if `file` holds a handle to `obj`.
 * Used to block cross-fd BO mapping via device-global mmap_offsets.
 */
bool	kms_file_owns_gem(struct drm_file *file,
	    struct drm_gem_object *obj);

/*
 * M4 PRIME dedup: returns the handle id if `file` has one for `obj`,
 * else 0.  drm_prime uses this to keep the 1:1 dmabuf→handle contract
 * Mesa/gbm expects.
 */
uint32_t kms_gem_handle_find_by_obj(struct drm_file *file,
	    struct drm_gem_object *obj);

/*
 * Release every handle still in a file's handle table.  Invoked from
 * the file dtor so a process that exits with open BOs doesn't leak
 * them.  Safe to call on a file that owns zero handles.
 */
void	kms_gem_release_all(struct drm_file *file);

#endif /* _KMS_DRM_GEM_H_ */
