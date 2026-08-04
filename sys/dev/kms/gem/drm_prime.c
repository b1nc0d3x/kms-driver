/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * PRIME / DMA-BUF — native FreeBSD implementation.
 *
 * Each exported GEM gets wrapped in a struct file whose fileops bridge
 * fd-mmap back to the same OBJT_MGTDEVICE cdev_pager used by the
 * /dev/dri/cardN MAP_DUMB + mmap path.  That guarantees the producer
 * process and any consumer (after FD_TO_HANDLE or direct mmap on the
 * fd) see the same physical pages.
 *
 * The DFLAG_PASSABLE flag on our fileops lets the fd cross Unix-domain
 * socket SCM_RIGHTS messages -- which is exactly how a Wayland client
 * hands its rendered surface to the compositor.
 *
 * No LinuxKPI: only the FreeBSD file/fileops/vm_object machinery.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/capsicum.h>
#include <sys/file.h>
#include <sys/filedesc.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/proc.h>
#include <sys/stat.h>
#include <sys/sx.h>
#include <sys/user.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_object.h>

#include <kms/drm_device.h>
#include <kms/drm_file.h>
#include <kms/drm_gem.h>
#include <kms/drm_prime.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>

#include "../core/kms_internal.h"

MALLOC_DECLARE(M_KMS);

/*
 * Per-file state for an exported dma-buf.  Holds the GEM ref taken
 * at HANDLE_TO_FD; dropped in the fileops close.  The cdev_pager
 * underneath the GEM doesn't need its own ref bumped here because the
 * GEM ref keeps it alive, and fo_mmap takes a fresh vm_object_reference
 * each time userspace maps.
 */
struct kms_prime_file {
	struct drm_gem_object	*obj;
};

static int	kms_prime_fo_mmap(struct file *fp, vm_map_t map,
		    vm_offset_t *addr, vm_size_t size, vm_prot_t prot,
		    vm_prot_t cap_maxprot, int flags, vm_ooffset_t foff,
		    struct thread *td);
static int	kms_prime_fo_stat(struct file *fp, struct stat *sb,
		    struct ucred *active_cred);
static int	kms_prime_fo_close(struct file *fp, struct thread *td);
static int	kms_prime_fo_fill_kinfo(struct file *fp,
		    struct kinfo_file *kif, struct filedesc *fdp);

static struct fileops kms_prime_fileops = {
	.fo_read	= invfo_rdwr,
	.fo_write	= invfo_rdwr,
	.fo_truncate	= invfo_truncate,
	.fo_ioctl	= invfo_ioctl,
	.fo_poll	= invfo_poll,
	.fo_kqfilter	= invfo_kqfilter,
	.fo_stat	= kms_prime_fo_stat,
	.fo_close	= kms_prime_fo_close,
	.fo_chmod	= invfo_chmod,
	.fo_chown	= invfo_chown,
	.fo_sendfile	= invfo_sendfile,
	.fo_mmap	= kms_prime_fo_mmap,
	.fo_fill_kinfo	= kms_prime_fo_fill_kinfo,
	.fo_flags	= DFLAG_PASSABLE,
};

static int
kms_prime_fo_close(struct file *fp, struct thread *td __unused)
{
	struct kms_prime_file *pf = fp->f_data;

	if (pf == NULL)
		return (0);
	if (pf->obj != NULL)
		kms_gem_object_put(pf->obj);
	free(pf, M_KMS);
	fp->f_data = NULL;
	return (0);
}

/*
 * mmap of the dma-buf fd.  Return the same vm_object the GEM already
 * has -- the cdev mmap path returns the same one, so producer + consumer
 * always share pages.
 */
static int
kms_prime_fo_mmap(struct file *fp, vm_map_t map __unused,
    vm_offset_t *addr __unused, vm_size_t size, vm_prot_t prot __unused,
    vm_prot_t cap_maxprot __unused, int flags __unused,
    vm_ooffset_t foff, struct thread *td __unused)
{
	struct kms_prime_file *pf = fp->f_data;
	vm_ooffset_t end;

	if (pf == NULL || pf->obj == NULL || pf->obj->pager == NULL)
		return (ENXIO);
	/*
	 * Reject partial / past-end mappings.  Previous check ignored foff,
	 * so a caller passing (foff=obj->size-4, size=8) would slide 4 bytes
	 * past the end into whatever the pager returns for an
	 * out-of-range offset.  Overflow-check the sum too.
	 */
	end = foff + (vm_ooffset_t)size;
	if (end < foff || end > (vm_ooffset_t)pf->obj->size)
		return (EINVAL);
	/*
	 * The actual vm_object install happens in vm_mmap_object via the
	 * caller (kern_mmap).  FreeBSD's fo_mmap doesn't return the object
	 * itself; instead it's responsible for completing the mapping by
	 * calling vm_mmap_object with the right backing.
	 */
	vm_object_reference(pf->obj->pager);
	return (vm_mmap_object(map, addr, size, prot, cap_maxprot, flags,
	    pf->obj->pager, foff, FALSE, td));
}

static int
kms_prime_fo_stat(struct file *fp, struct stat *sb,
    struct ucred *active_cred)
{
	struct kms_prime_file *pf = fp->f_data;

	bzero(sb, sizeof(*sb));
	sb->st_mode = S_IFCHR | 0600;
	sb->st_uid = active_cred->cr_uid;
	sb->st_gid = active_cred->cr_gid;
	sb->st_size = (pf != NULL && pf->obj != NULL) ?
	    pf->obj->size : 0;
	return (0);
}

static int
kms_prime_fo_fill_kinfo(struct file *fp __unused, struct kinfo_file *kif,
    struct filedesc *fdp __unused)
{
	kif->kf_type = KF_TYPE_UNKNOWN;	/* no dedicated KF_TYPE_DMABUF */
	return (0);
}

/*
 * Cross-driver accessor.  Consumer drivers (e.g. fgpu) can pass a
 * struct file * and receive the underlying drm_gem_object with an
 * added reference — or NULL if the fd is not a kms prime fd.  This
 * lets other drivers import a kms scanout BO without having to
 * introspect our internal fileops / private-data layout.
 *
 * Caller must drop the returned reference via kms_gem_object_put().
 */
struct drm_gem_object *
kms_prime_fd_to_gem(struct file *fp)
{
	struct kms_prime_file *pf;

	if (fp == NULL || fp->f_ops != &kms_prime_fileops)
		return (NULL);
	pf = fp->f_data;
	if (pf == NULL || pf->obj == NULL)
		return (NULL);
	kms_gem_object_get(pf->obj);
	return (pf->obj);
}

int
kms_ioctl_prime_handle_to_fd(struct drm_file *file,
    struct drm_prime_handle *args)
{
	struct kms_prime_file *pf;
	struct drm_gem_object *obj;
	struct file *fp;
	int fd;
	int error;

	if (file == NULL || args == NULL || args->handle == 0)
		return (EINVAL);

	/*
	 * Linux passes DRM_CLOEXEC / DRM_RDWR via args->flags.  We
	 * always allocate with O_CLOEXEC + O_RDWR semantics; explicit
	 * RDWR is implied by mmap working at all on our fileops.
	 * (Reject unknown flag bits to leave room for future use.)
	 */
	if (args->flags & ~(DRM_CLOEXEC | DRM_RDWR))
		return (EINVAL);

	obj = kms_gem_handle_lookup(file, args->handle);
	if (obj == NULL)
		return (ENOENT);

	pf = malloc(sizeof(*pf), M_KMS, M_WAITOK | M_ZERO);
	pf->obj = obj;	/* ref taken by lookup is transferred to pf */

	error = falloc_caps(curthread, &fp, &fd,
	    (args->flags & DRM_CLOEXEC) ? O_CLOEXEC : 0, NULL);
	if (error != 0) {
		free(pf, M_KMS);
		kms_gem_object_put(obj);
		return (error);
	}
	finit(fp, FREAD | FWRITE, DTYPE_NONE, pf, &kms_prime_fileops);
	fdrop(fp, curthread);

	args->fd = fd;
	return (0);
}

int
kms_ioctl_prime_fd_to_handle(struct drm_file *file,
    struct drm_prime_handle *args)
{
	struct file *fp;
	struct kms_prime_file *pf;
	uint32_t handle;
	int error;

	if (file == NULL || args == NULL || args->fd < 0)
		return (EINVAL);

	error = fget(curthread, args->fd, &cap_no_rights, &fp);
	if (error != 0)
		return (error);

	if (fp->f_ops != &kms_prime_fileops) {
		fdrop(fp, curthread);
		return (EINVAL);
	}
	pf = fp->f_data;
	if (pf == NULL || pf->obj == NULL) {
		fdrop(fp, curthread);
		return (EINVAL);
	}

	/*
	 * M4 PRIME dedup: if this file already holds a handle for this
	 * dmabuf's underlying GEM, return the SAME handle id.  Linux
	 * DRM PRIME is 1:1 — Mesa/gbm cache BO→handle mappings and
	 * close() the handle once, so returning two ids for one obj
	 * causes a use-after-close on the second cached handle.
	 */
	handle = kms_gem_handle_find_by_obj(file, pf->obj);
	if (handle != 0) {
		fdrop(fp, curthread);
		args->handle = handle;
		return (0);
	}
	/*
	 * No existing handle — create fresh.  kms_gem_handle_create takes
	 * its own ref on success and balances it on internal-failure, so
	 * we must not double-get here.
	 */
	error = kms_gem_handle_create(file, pf->obj, &handle);
	fdrop(fp, curthread);
	if (error != 0)
		return (error);
	args->handle = handle;
	return (0);
}

/* ---- fgpu bridge (optional) --------------------------------------------- */

/*
 * If fgpu.ko is loaded, register a struct fgpu_dmabuf_ops so that
 * FGPU_IOC_BO_IMPORT can accept our prime fds — enabling
 * "kms CREATE_DUMB → PRIME_HANDLE_TO_FD → fgpu BO_IMPORT" cross-driver
 * flows without kms.ko hard-depending on fgpu.ko.  Symbol resolved
 * at SYSINIT time via kldsym(9); no-op if fgpu.ko is absent.
 *
 * Kept in-tree here (rather than a separate bridge module) because
 * the four adapter functions are 20 lines of glue and duplicating the
 * build harness for a single .ko is worse than an optional symbol
 * lookup.
 */
#include <sys/linker.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <vm/vm_page.h>

struct fgpu_dmabuf_ops {
	void		*(*fd_to_gem)(struct file *fp);
	void		 (*gem_put)(void *gem_cookie);
	size_t		 (*gem_size)(void *gem_cookie);
	vm_paddr_t	 (*gem_first_pa)(void *gem_cookie);
};

static void *
kms_bridge_fd_to_gem(struct file *fp)
{

	return (kms_prime_fd_to_gem(fp));
}

static void
kms_bridge_gem_put(void *gem_cookie)
{

	kms_gem_object_put((struct drm_gem_object *)gem_cookie);
}

static size_t
kms_bridge_gem_size(void *gem_cookie)
{
	struct drm_gem_object *obj = gem_cookie;

	return (obj != NULL ? obj->size : 0);
}

static vm_paddr_t
kms_bridge_gem_first_pa(void *gem_cookie)
{
	struct drm_gem_object *obj = gem_cookie;

	if (obj == NULL || obj->pages == NULL || obj->npages == 0)
		return (0);
	return (VM_PAGE_TO_PHYS(obj->pages[0]));
}

static const struct fgpu_dmabuf_ops kms_bridge_ops = {
	.fd_to_gem	= kms_bridge_fd_to_gem,
	.gem_put	= kms_bridge_gem_put,
	.gem_size	= kms_bridge_gem_size,
	.gem_first_pa	= kms_bridge_gem_first_pa,
};

static int (*fgpu_register_dmabuf_ops_p)(const struct fgpu_dmabuf_ops *);
static void (*fgpu_unregister_dmabuf_ops_p)(void);
static bool kms_bridge_registered;

int kms_bridge_rearm(void);	/* forward decl — definition after SYSINIT */

/*
 * linker_file_foreach predicate — checks whether `lf` exports both
 * fgpu API symbols, and if so caches their addresses into the
 * *_p globals + returns non-zero to stop iteration.
 */
static int
kms_bridge_find_fgpu(linker_file_t lf, void *ctx __unused)
{
	caddr_t reg, unreg;

	reg = linker_file_lookup_symbol(lf, "fgpu_register_dmabuf_ops", 0);
	unreg = linker_file_lookup_symbol(lf, "fgpu_unregister_dmabuf_ops", 0);
	if (reg == NULL || unreg == NULL)
		return (0);
	fgpu_register_dmabuf_ops_p =
	    (int (*)(const struct fgpu_dmabuf_ops *))reg;
	fgpu_unregister_dmabuf_ops_p = (void (*)(void))unreg;
	return (1);	/* found; stop iteration */
}

static void
kms_bridge_fgpu_init(void *unused __unused)
{

	/*
	 * Best-effort lookup at kms SYSINIT time — fgpu.ko may not be
	 * loaded yet, or may load later and call kms_bridge_rearm()
	 * itself.  If fgpu.ko is present we register immediately.
	 */
	(void)kms_bridge_rearm();
}
SYSINIT(kms_bridge_fgpu, SI_SUB_DRIVERS, SI_ORDER_ANY,
    kms_bridge_fgpu_init, NULL);

/*
 * Re-run the bridge scan + registration.  Always re-looks-up fgpu's
 * symbols (so a reloaded fgpu.ko gets a fresh pointer) and calls
 * register unconditionally.  Public so fgpu.ko can call it from its
 * own SYSINIT — solves the "kms boots without fgpu; fgpu loads
 * later" order dependency AND the "smoketest kldunload/kldload
 * fgpu" reload case (fgpu's ops state was wiped when its cdev
 * detached, so re-registering restores it).
 */
int
kms_bridge_rearm(void)
{

	fgpu_register_dmabuf_ops_p = NULL;
	fgpu_unregister_dmabuf_ops_p = NULL;
	kms_bridge_registered = false;
	(void)linker_file_foreach(kms_bridge_find_fgpu, NULL);
	if (fgpu_register_dmabuf_ops_p == NULL ||
	    fgpu_unregister_dmabuf_ops_p == NULL)
		return (ENOENT);
	if (fgpu_register_dmabuf_ops_p(&kms_bridge_ops) != 0)
		return (EIO);
	kms_bridge_registered = true;
	printf("kms: registered fgpu dmabuf bridge\n");
	return (0);
}

static void
kms_bridge_fgpu_fini(void *unused __unused)
{

	if (kms_bridge_registered && fgpu_unregister_dmabuf_ops_p != NULL)
		fgpu_unregister_dmabuf_ops_p();
}
SYSUNINIT(kms_bridge_fgpu, SI_SUB_DRIVERS, SI_ORDER_ANY,
    kms_bridge_fgpu_fini, NULL);
