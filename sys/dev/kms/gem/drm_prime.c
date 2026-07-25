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
    vm_ooffset_t foff __unused, struct thread *td __unused)
{
	struct kms_prime_file *pf = fp->f_data;

	if (pf == NULL || pf->obj == NULL)
		return (ENXIO);
	if (size > pf->obj->size)
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
    struct ucred *active_cred __unused)
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
	 * Create a fresh handle on the importing drm_file pointing at the
	 * same underlying GEM.  kms_gem_handle_create takes its own ref on
	 * success and balances it on internal-failure, so we must not
	 * double-get here — a stray get with only an error-path put leaks
	 * one ref per successful import (pinning the wired contig pages
	 * for the life of the driver).
	 */
	error = kms_gem_handle_create(file, pf->obj, &handle);
	fdrop(fp, curthread);
	if (error != 0)
		return (error);
	args->handle = handle;
	return (0);
}
