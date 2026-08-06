/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * GEM object lifecycle + per-file handle table + cdev_pager backing.
 *
 * Pages are allocated contiguous + wired via vm_page_alloc_noobj_contig
 * and pre-inserted into a cdev_pager (OBJT_MGTDEVICE).  Userspace mmap
 * on the cdev re-acquires the same pager via cdev_pager_allocate
 * keyed on the GEM object pointer, so the user mapping shares pages
 * with whatever kernel KVA mapping the driver established.  Same
 * pattern as tegra, virtio_drm, and vmsvga_drm Phase C.2.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/queue.h>
#include <sys/refcount.h>
#include <sys/pctrie.h>
#include <sys/rwlock.h>
#include <sys/sx.h>
#include <sys/sysctl.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <vm/vm_pageout.h>
#include <vm/vm_param.h>
#include <vm/vm_pager.h>
#include <vm/vm_radix.h>

#include <kms/drm_device.h>
#include <kms/drm_file.h>
#include <kms/drm_gem.h>

#include "kms_internal.h"

SYSCTL_DECL(_kern_kms);
#ifdef KMS_DEBUG_GEM_PA
/*
 * Test-only surface for cross-driver buffer-sharing bring-up (fgpu ↔
 * kms handshakes before real dmabuf export landed).  World-readable
 * physical addresses are a KASLR-bypass oracle and hardware-exploit
 * targeting aid; a stock build MUST NOT expose this.  Build with
 * -DKMS_DEBUG_GEM_PA when you actually need it, and only then.
 */
vm_paddr_t kms_debug_last_gem_pa = 0;
SYSCTL_QUAD(_kern_kms, OID_AUTO, last_gem_pa, CTLFLAG_RD,
    &kms_debug_last_gem_pa, 0,
    "Physical address (page 0) of the most recently created GEM object. "
    "DEBUG BUILD ONLY.");
#endif

/*
 * Per-file handle entry.  GEM objects live on the device-side list
 * (drm_device->gem_objects); the handle table just maps a 32-bit
 * user-visible id to the same object so userspace ioctls can look it
 * up by handle without seeing the kernel pointer.
 */
struct drm_gem_handle {
	uint32_t			 id;
	struct drm_gem_object		*obj;
	TAILQ_ENTRY(drm_gem_handle)	 link;
};

/* ----- pager callbacks ----- */

/*
 * Fault handler is a no-op: every obj->pages[i] is pre-inserted into
 * the pager's pctrie at object-create time (see kms_gem_object_create
 * below), so the VM fast path finds the real managed page without ever
 * calling in here.  Mirrors tegra_gem_pager_fault (arm/nvidia/drm2/
 * tegra_bo.c:324) — proven to route user CPU writes at Xorg's mmap
 * directly to obj->pages[]'s DRAM.
 *
 * Prior implementations tried the linuxkpi vm_page_getfake pattern
 * (linux_cdev_pager_fault, linux_compat.c:453).  It works for one-shot
 * linear writes (write_dumb) but on fbsdmac 2026-08-06 Xorg's mmap
 * writes landed at a DIFFERENT phys addr than obj->pages[i] — xwd
 * -root showed content, kernel fb_scan via PHYS_TO_DMAP saw all zeros.
 * Tegra's pre-insert-real-pages-marked-PG_FICTITIOUS pattern doesn't
 * have that alias.
 */
static int
drm_gem_pager_fault(vm_object_t vm_obj __unused, vm_ooffset_t offset __unused,
    int prot __unused, vm_page_t *mres __unused)
{
	return (VM_PAGER_FAIL);
}

static int
drm_gem_pager_ctor(void *handle __unused, vm_ooffset_t size __unused,
    vm_prot_t prot __unused, vm_ooffset_t foff __unused,
    struct ucred *cred __unused, u_short *color)
{
	if (color != NULL)
		*color = 0;
	return (0);
}

/*
 * Pager destructor: invoked by the VM subsystem when the pager's
 * last reference drops.  This is the canonical free path for the
 * GEM object's pages and storage — it fires either from
 * drm_gem_object_release (no outstanding mmaps) or from the last
 * munmap (mappings outlived the handle table entry).  Either way,
 * by the time we get here nothing references this BO.
 */
static void
drm_gem_pager_dtor(void *handle)
{
	struct drm_gem_object *obj = handle;
	struct pctrie_iter pctrie_pages;
	vm_page_t m;
	size_t i;

	/*
	 * Each obj->pages[i] was inserted into obj->pager's pctrie via
	 * vm_page_iter_insert at create time.  Mirror tegra_bo_destruct
	 * (arm/nvidia/drm2/tegra_bo.c) to unhook them properly before
	 * vm_page_free: iterate the pctrie, busy each page,
	 * cdev_mgtdev_pager_free_page to detach, clear PG_FICTITIOUS +
	 * restore VPO_UNMANAGED so vm_page_free walks the normal path.
	 */
	if (obj->pager != NULL) {
		vm_page_iter_init(&pctrie_pages, obj->pager);
		VM_OBJECT_WLOCK(obj->pager);
		for (i = 0; i < obj->npages; i++) {
			m = vm_radix_iter_lookup(&pctrie_pages, i);
			if (m == NULL)
				continue;
			vm_page_busy_acquire(m, 0);
			cdev_mgtdev_pager_free_page(&pctrie_pages, m);
			m->flags &= ~PG_FICTITIOUS;
			m->oflags |= VPO_UNMANAGED;
			vm_page_unwire_noq(m);
			vm_page_free(m);
		}
		VM_OBJECT_WUNLOCK(obj->pager);
	} else {
		for (i = 0; i < obj->npages; i++) {
			m = obj->pages[i];
			if (m == NULL)
				continue;
			vm_page_unwire_noq(m);
			vm_page_free(m);
		}
	}
	free(obj->pages, M_KMS);
	free(obj, M_KMS);
}

static struct cdev_pager_ops drm_gem_pager_ops = {
	.cdev_pg_fault	= drm_gem_pager_fault,
	.cdev_pg_ctor	= drm_gem_pager_ctor,
	.cdev_pg_dtor	= drm_gem_pager_dtor,
};

/* ----- object lifecycle ----- */

struct drm_gem_object *
kms_gem_object_create(struct drm_device *dev, size_t size)
{
	struct drm_gem_object *obj;
	vm_page_t m;
	size_t npages;
	size_t i;
	int tries;

	if (dev == NULL)
		return (NULL);
	size = round_page(size);
	if (size == 0)
		return (NULL);
	/*
	 * H6 (2026-08-03 review): cap total object size at 256 MiB.
	 * CREATE_DUMB has its own 256 MiB cap in the dumb path, but
	 * I915_GEM_CREATE and any other driver-side creator lands here
	 * with an unchecked userspace-supplied size — the pages-array
	 * malloc below is npages * sizeof(void*), so a u64 size close to
	 * SIZE_MAX would kill the kernel with M_WAITOK before the
	 * contig alloc even ran.  Match the DUMB cap.
	 */
	if (size > (256UL << 20))
		return (NULL);
	npages = atop(size);

	obj = malloc(sizeof(*obj), M_KMS, M_WAITOK | M_ZERO);
	obj->dev = dev;
	obj->size = size;
	obj->npages = npages;
	obj->pages = malloc(sizeof(*obj->pages) * npages, M_KMS,
	    M_WAITOK | M_ZERO);
	refcount_init(&obj->refs, 1);

	tries = 0;
retry_alloc:
	/*
	 * Contiguous wired allocation — DRM scanout needs the buffer
	 * physically linear, and Phase 6's stub-validation path doesn't
	 * need IOMMU scatter-gather.  Zeroed so userspace doesn't see
	 * old kernel data.
	 */
	m = vm_page_alloc_noobj_contig(VM_ALLOC_WIRED | VM_ALLOC_ZERO,
	    npages, 0, ~0UL, PAGE_SIZE, 0,
#ifdef __aarch64__
	    /*
	     * arm64 aliases VM_MEMATTR_WRITE_COMBINING to WRITE_THROUGH
	     * (still cached) — that leaves Xorg's scattered pixel writes
	     * stranded in the D-cache while the RK3399 VOP DMA scans DRAM.
	     * UNCACHEABLE routes every write straight to DRAM.  Slower for
	     * pure-CPU workloads but scanout has no coherency plumbing.
	     */
	    VM_MEMATTR_UNCACHEABLE
#else
	    /*
	     * x86: WRITE_COMBINING is the standard scanout memory
	     * attribute (what Linux i915 uses for stolen + system scanout
	     * pages) and matches what modesetting_drv's XRender fallback
	     * expects — writes flush through the WC combining buffer on
	     * SFENCE/read.  WB is also correct architecturally but modeset
	     * driver's fbCompositeGeneral loop may issue MOVNTPS or
	     * similar streaming stores that only make sense on WC pages.
	     */
	    VM_MEMATTR_WRITE_COMBINING
#endif
	    );
	if (m == NULL) {
		if (tries++ < 3) {
			vm_page_reclaim_contig(0, npages, 0, ~0UL,
			    PAGE_SIZE, 0);
			goto retry_alloc;
		}
		free(obj->pages, M_KMS);
		free(obj, M_KMS);
		return (NULL);
	}
	for (i = 0; i < npages; i++, m++) {
		m->valid = VM_PAGE_BITS_ALL;
		/*
		 * Per-page memattr backs the PTE created by pmap_enter when
		 * userspace mmaps this BO.  Both arm64 (UNCACHEABLE) and
		 * x86 (WRITE_COMBINING) need the explicit set to ensure
		 * Xorg's mmap uses the same memattr as vm_page_alloc chose
		 * for the physical pages — see vm_page_alloc_noobj_contig
		 * call above for rationale.
		 */
#ifdef __aarch64__
		pmap_page_set_memattr(m, VM_MEMATTR_UNCACHEABLE);
#else
		pmap_page_set_memattr(m, VM_MEMATTR_WRITE_COMBINING);
#endif
		obj->pages[i] = m;
		/*
		 * Force-zero via PHYS_TO_DMAP.  VM_ALLOC_ZERO skips the
		 * pmap_zero_page call for pages already flagged PG_ZERO,
		 * but on x86 the EFI-fb backing pages get flagged clean by
		 * the boot-time page pool even though they still hold the
		 * loader logo image.  Result: the very first Xorg fb=23
		 * shows the FreeBSD loader menu until Xorg overwrites, and
		 * on the Apple eDP handoff where XRender writes don't
		 * currently reach obj->pages, that "until" is forever.
		 * Verified 2026-08-06 fbsdmac panel capture.
		 */
		bzero((void *)PHYS_TO_DMAP(VM_PAGE_TO_PHYS(m)), PAGE_SIZE);
	}

	/*
	 * One-shot boot log naming the memattr we requested.  If the
	 * platform's PAT config falls back WC->UC (older hypervisors) or
	 * WC->WB (broken PAT), scanout misbehaviour vs Xorg's XRender is
	 * hard to distinguish from a genuine driver bug — this line makes
	 * the intent grep-able from dmesg so an operator seeing black
	 * panels can rule PAT out in one step.
	 */
	{
		static bool memattr_logged = false;
		if (!memattr_logged) {
			memattr_logged = true;
#ifdef __aarch64__
			printf("kms/gem: scanout pages tagged "
			    "VM_MEMATTR_UNCACHEABLE\n");
#else
			printf("kms/gem: scanout pages tagged "
			    "VM_MEMATTR_WRITE_COMBINING (needs PAT slot)\n");
#endif
		}
	}

	/*
	 * Create the cdev_pager keyed on the GEM object pointer.  Faults
	 * dispatch through drm_gem_pager_fault which returns a fake page
	 * carrying obj->pages[pindex]'s paddr (linuxkpi idiom).
	 */
	obj->pager = cdev_pager_allocate(obj, OBJT_MGTDEVICE,
	    &drm_gem_pager_ops, size, 0, 0, NULL);
	/*
	 * Set the pager's PTE-time memattr so any fault path that goes
	 * through it still produces the correct memattr PTE.  Belt-and-
	 * braces with the per-page memattr above.
	 */
	if (obj->pager != NULL) {
#ifdef __aarch64__
		vm_object_set_memattr(obj->pager, VM_MEMATTR_UNCACHEABLE);
#else
		vm_object_set_memattr(obj->pager, VM_MEMATTR_WRITE_COMBINING);
#endif
	}
	if (obj->pager == NULL) {
		for (i = 0; i < npages; i++) {
			vm_page_unwire_noq(obj->pages[i]);
			vm_page_free(obj->pages[i]);
		}
		free(obj->pages, M_KMS);
		free(obj, M_KMS);
		return (NULL);
	}

	/*
	 * Pre-insert every real managed page into the pager's pctrie via
	 * vm_page_iter_insert.  Tegra idiom (arm/nvidia/drm2/tegra_bo.c
	 * :tegra_bo_init_pager).  VM's fault fast path finds the page in
	 * the pctrie and calls pmap_enter directly with the real vm_page,
	 * so user mmap PTEs land on obj->pages[i]'s DRAM.  Clearing
	 * VPO_UNMANAGED + setting PG_FICTITIOUS is the required pmap
	 * contract for cdev_pager pre-insertion of contig-allocated
	 * pages: pmap treats them as managed for tracking but skips the
	 * PHYS_TO_VM_PAGE round-trip.
	 */
	{
		struct pctrie_iter pctrie_pages;

		vm_page_iter_init(&pctrie_pages, obj->pager);
		VM_OBJECT_WLOCK(obj->pager);
		for (i = 0; i < npages; i++) {
			vm_page_t p = obj->pages[i];

			p->oflags &= ~VPO_UNMANAGED;
			p->flags |= PG_FICTITIOUS;
			if (vm_page_iter_insert(p, obj->pager, i,
			    &pctrie_pages) != 0) {
				VM_OBJECT_WUNLOCK(obj->pager);
				printf("kms/gem: vm_page_iter_insert failed "
				    "at pindex %zu\n", i);
				vm_object_deallocate(obj->pager);
				obj->pager = NULL;
				for (size_t j = 0; j < npages; j++) {
					obj->pages[j]->flags &=
					    ~PG_FICTITIOUS;
					obj->pages[j]->oflags |=
					    VPO_UNMANAGED;
					vm_page_unwire_noq(obj->pages[j]);
					vm_page_free(obj->pages[j]);
				}
				free(obj->pages, M_KMS);
				free(obj, M_KMS);
				return (NULL);
			}
		}
		VM_OBJECT_WUNLOCK(obj->pager);
	}

	sx_xlock(&dev->gem_lock);
	obj->mmap_offset = dev->mmap_offset_counter;
	dev->mmap_offset_counter += size;
	TAILQ_INSERT_TAIL(&dev->gem_objects, obj, device_link);
	sx_xunlock(&dev->gem_lock);

#ifdef KMS_DEBUG_GEM_PA
	/*
	 * Debug hook — remember the pa of page 0 of the most-recently
	 * created GEM.  Read via the kern.kms.last_gem_pa sysctl (also
	 * DEBUG-only) by cross-driver test rigs.  Compiled out of a
	 * stock build; see the sysctl decl above for why.
	 */
	kms_debug_last_gem_pa = VM_PAGE_TO_PHYS(obj->pages[0]);
#endif

	return (obj);
}

void
kms_gem_object_get(struct drm_gem_object *obj)
{
	refcount_acquire(&obj->refs);
}

/*
 * obj->refs hit zero.  Remove from the device list (so no further
 * lookups can race the teardown), then drop the pager's creation
 * reference.  If no mmaps are outstanding, that's the last pager
 * ref and drm_gem_pager_dtor fires synchronously below, freeing
 * pages + the obj storage.  If userspace still has the BO mmap'd,
 * the dtor fires on the last munmap instead — userspace can read
 * the buffer past destroy_dumb, which is the documented Linux
 * behavior.  After vm_object_deallocate we must not touch obj.
 */
static void
drm_gem_object_release(struct drm_gem_object *obj)
{
	sx_xlock(&obj->dev->gem_lock);
	TAILQ_REMOVE(&obj->dev->gem_objects, obj, device_link);
	sx_xunlock(&obj->dev->gem_lock);

	vm_object_deallocate(obj->pager);
}

void
kms_gem_object_put(struct drm_gem_object *obj)
{
	if (obj == NULL)
		return;
	if (refcount_release(&obj->refs))
		drm_gem_object_release(obj);
}

/*
 * Range-based sibling used by the per-page d_mmap path.  Userspace
 * fault addresses arrive as (mmap_offset + page_offset_within_object)
 * so we can't demand an exact match on obj->mmap_offset — return the
 * object whose [mmap_offset, mmap_offset+size) range contains offset,
 * and report its base so the caller can compute
 * page_idx = (offset - base) / PAGE_SIZE.
 */
struct drm_gem_object *
kms_gem_object_lookup_offset_containing(struct drm_device *dev,
    uint64_t offset, uint64_t *base_out)
{
	struct drm_gem_object *obj;

	sx_slock(&dev->gem_lock);
	TAILQ_FOREACH(obj, &dev->gem_objects, device_link) {
		if (offset < obj->mmap_offset ||
		    offset >= obj->mmap_offset + obj->size)
			continue;
		if (!refcount_acquire_if_not_zero(&obj->refs))
			break;
		sx_sunlock(&dev->gem_lock);
		if (base_out != NULL)
			*base_out = obj->mmap_offset;
		return (obj);
	}
	sx_sunlock(&dev->gem_lock);
	return (NULL);
}

struct drm_gem_object *
kms_gem_object_lookup_offset(struct drm_device *dev, uint64_t offset)
{
	struct drm_gem_object *obj;

	sx_slock(&dev->gem_lock);
	TAILQ_FOREACH(obj, &dev->gem_objects, device_link) {
		if (obj->mmap_offset != offset)
			continue;
		/*
		 * Concurrent put may have driven refs to 0 already but not
		 * yet reached drm_gem_object_release's TAILQ_REMOVE (that
		 * removal needs gem_lock exclusive, which is blocked by our
		 * slock).  Refcount_acquire on a zero ref would resurrect a
		 * dead object; if_not_zero rejects it so the caller sees
		 * ENOENT — same behaviour as if put ordering had won the
		 * race by a hair.
		 */
		if (!refcount_acquire_if_not_zero(&obj->refs))
			break;
		sx_sunlock(&dev->gem_lock);
		return (obj);
	}
	sx_sunlock(&dev->gem_lock);
	return (NULL);
}

/* ----- handle table ----- */

int
kms_gem_handle_create(struct drm_file *file, struct drm_gem_object *obj,
    uint32_t *handle_out)
{
	struct drm_gem_handle *h;

	if (file == NULL || obj == NULL || handle_out == NULL)
		return (EINVAL);

	h = malloc(sizeof(*h), M_KMS, M_WAITOK | M_ZERO);
	kms_gem_object_get(obj);
	h->obj = obj;

	sx_xlock(&file->handle_lock);
	if (file->next_handle == UINT32_MAX) {
		sx_xunlock(&file->handle_lock);
		kms_gem_object_put(obj);
		free(h, M_KMS);
		return (ENOSPC);
	}
	h->id = ++file->next_handle;
	TAILQ_INSERT_TAIL(&file->handles, h, link);
	sx_xunlock(&file->handle_lock);

	*handle_out = h->id;
	return (0);
}

int
kms_gem_handle_delete(struct drm_file *file, uint32_t handle)
{
	struct drm_gem_handle *h;

	sx_xlock(&file->handle_lock);
	TAILQ_FOREACH(h, &file->handles, link) {
		if (h->id == handle) {
			TAILQ_REMOVE(&file->handles, h, link);
			sx_xunlock(&file->handle_lock);
			kms_gem_object_put(h->obj);
			free(h, M_KMS);
			return (0);
		}
	}
	sx_xunlock(&file->handle_lock);
	return (ENOENT);
}

struct drm_gem_object *
kms_gem_handle_lookup(struct drm_file *file, uint32_t handle)
{
	struct drm_gem_handle *h;
	struct drm_gem_object *obj = NULL;

	sx_slock(&file->handle_lock);
	TAILQ_FOREACH(h, &file->handles, link) {
		if (h->id == handle) {
			obj = h->obj;
			kms_gem_object_get(obj);
			break;
		}
	}
	sx_sunlock(&file->handle_lock);
	return (obj);
}

/*
 * H2 mmap-ownership gate helper.  Returns true if `file` holds any
 * handle referencing `obj`.  Used by kms_mmap_single to block
 * cross-fd BO mapping via device-global mmap_offsets.
 */
bool
kms_file_owns_gem(struct drm_file *file, struct drm_gem_object *obj)
{
	struct drm_gem_handle *h;
	bool owned = false;

	if (file == NULL || obj == NULL)
		return (false);
	sx_slock(&file->handle_lock);
	TAILQ_FOREACH(h, &file->handles, link) {
		if (h->obj == obj) {
			owned = true;
			break;
		}
	}
	sx_sunlock(&file->handle_lock);
	return (owned);
}

/*
 * M4 PRIME dedup helper.  Returns the handle id if `file` already has
 * a handle referencing `obj`, or 0 if none.  Linux DRM PRIME contract
 * is 1:1 — importing the same dmabuf twice on one drm_file must
 * return the SAME handle so Mesa/gbm's caches stay coherent.
 */
uint32_t
kms_gem_handle_find_by_obj(struct drm_file *file, struct drm_gem_object *obj)
{
	struct drm_gem_handle *h;
	uint32_t id = 0;

	if (file == NULL || obj == NULL)
		return (0);
	sx_slock(&file->handle_lock);
	TAILQ_FOREACH(h, &file->handles, link) {
		if (h->obj == obj) {
			id = h->id;
			break;
		}
	}
	sx_sunlock(&file->handle_lock);
	return (id);
}

void
kms_gem_release_all(struct drm_file *file)
{
	struct drm_gem_handle *h;

	if (file == NULL)
		return;
	/*
	 * Pull entries out one at a time under the lock, drop the lock,
	 * then put the ref — the put may recurse into the device-side
	 * gem_lock (drm_gem_object_free), and holding handle_lock during
	 * that would create an unnecessary dependency.
	 */
	for (;;) {
		sx_xlock(&file->handle_lock);
		h = TAILQ_FIRST(&file->handles);
		if (h == NULL) {
			sx_xunlock(&file->handle_lock);
			break;
		}
		TAILQ_REMOVE(&file->handles, h, link);
		sx_xunlock(&file->handle_lock);

		kms_gem_object_put(h->obj);
		free(h, M_KMS);
	}
}
