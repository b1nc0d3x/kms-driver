/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/errno.h>
#include <sys/proc.h>
#include <sys/sysctl.h>

#include <drm/drm.h>
#include <kms/drm_device.h>
#include <kms/drm_drv.h>
#include <kms/drm_file.h>
#include <kms/drm_gem.h>
#include <kms/drm_prime.h>

#include "kms_internal.h"

/*
 * Per-ioctl trace knob.  When > 0, kms_ioctl prints "kms: ioctl 0xXXX
 * file=PTR pid=N is_render=Y" at entry.  Single-line per call so it can
 * be greppedmatched up with kdump output.  Default off to keep dmesg
 * quiet on production paths.  Bump to debug compositor wedges:
 *   sudo sysctl kern.kms.ioctl_trace=1
 * Bumping it after a kwin hang lets the next interaction show the
 * last ioctl before the freeze - the wedge sits right after the last
 * traced line.
 */
static int kms_ioctl_trace = 0;
SYSCTL_NODE(_kern, OID_AUTO, kms, CTLFLAG_RD, NULL, "KMS framework");
SYSCTL_INT(_kern_kms, OID_AUTO, ioctl_trace, CTLFLAG_RWTUN,
    &kms_ioctl_trace, 0,
    "Print every DRM ioctl entry + return (0 off, 1 entry, 2 entry+exit)");

/*
 * Copy a NUL-terminated kernel string out to a user buffer described
 * by (uptr, *ulen).  *ulen on entry is the buffer capacity; on return
 * holds the source length (not including NUL).  Truncates without
 * error if the buffer is too small, matching Linux behavior.  uptr
 * may be NULL when userspace is probing for the required length.
 */
static int
drm_copy_name_out(const char *src, char *uptr, size_t *ulen)
{
	size_t srclen, n;
	int error;

	srclen = (src != NULL) ? strlen(src) : 0;
	if (uptr != NULL && *ulen > 0) {
		n = MIN(*ulen, srclen);
		error = copyout(src, uptr, n);
		if (error != 0)
			return (error);
	}
	*ulen = srclen;
	return (0);
}

static int
drm_ioctl_version(struct drm_file *file, struct drm_version *v)
{
	const struct drm_driver *drv;
	int error;

	drv = file->dev->driver;
	v->version_major = drv->major;
	v->version_minor = drv->minor;
	v->version_patchlevel = drv->patchlevel;
	error = drm_copy_name_out(drv->name, v->name, &v->name_len);
	if (error != 0)
		return (error);
	error = drm_copy_name_out(drv->date, v->date, &v->date_len);
	if (error != 0)
		return (error);
	return (drm_copy_name_out(drv->desc, v->desc, &v->desc_len));
}

static int
drm_ioctl_get_unique(struct drm_file *file, struct drm_unique *u)
{
	char busid[32];

	snprintf(busid, sizeof(busid), "kms:%d", file->dev->minor);
	return (drm_copy_name_out(busid, u->unique, &u->unique_len));
}

static int
drm_ioctl_set_version(struct drm_file *file __unused,
    struct drm_set_version *v)
{
	/*
	 * Userspace requests a DRM-interface version; we report ours
	 * back and accept whatever they asked for as long as the major
	 * matches.  drm_interface_version is 1.4 (legacy DRI) for now;
	 * later phases bump to advertise DRI3 / atomic / etc.
	 */
	if (v->drm_di_major != -1 && v->drm_di_major != 1)
		return (EINVAL);
	v->drm_di_major = 1;
	v->drm_di_minor = 4;
	v->drm_dd_major = file->dev->driver->major;
	v->drm_dd_minor = file->dev->driver->minor;
	return (0);
}

static int
drm_ioctl_get_cap(struct drm_file *file __unused, struct drm_get_cap *c)
{
	/*
	 * Known capabilities — we recognize them, return 0 to mean
	 * "supported but disabled / no value yet."  This is the
	 * distinction libdrm draws between "driver says no" (return 0,
	 * value 0) and "driver doesn't know this cap" (EINVAL).  Returning
	 * 0 for unknown ids would mislead libdrm into thinking we
	 * explicitly disabled features it never asked us about.
	 *
	 * Caps with non-zero values (cursor dimensions, prime flags, etc.)
	 * get populated as the phases that implement them land.
	 */
	switch (c->capability) {
	case DRM_CAP_DUMB_BUFFER:
		/*
		 * Phase 6 implements CREATE_DUMB / MAP_DUMB / DESTROY_DUMB +
		 * mmap via cdev_pager.  Xorg's modesetting driver refuses to
		 * load without this, even if it never calls CREATE_DUMB.
		 */
		c->value = 1;
		return (0);
	case DRM_CAP_DUMB_PREFERRED_DEPTH:
		c->value = 24;
		return (0);
	case DRM_CAP_DUMB_PREFER_SHADOW:
		c->value = 1;
		return (0);
	case DRM_CAP_TIMESTAMP_MONOTONIC:
		c->value = 1;
		return (0);
	case DRM_CAP_PRIME:
		/*
		 * drm_prime.c implements both HANDLE_TO_FD (export) and
		 * FD_TO_HANDLE (import).  Bits per Linux uapi:
		 *   DRM_PRIME_CAP_EXPORT = 1, DRM_PRIME_CAP_IMPORT = 2.
		 */
		c->value = 1 | 2;
		return (0);
	case DRM_CAP_CRTC_IN_VBLANK_EVENT:
		/*
		 * kms_send_vblank_event sets drm_event_vblank.crtc_id so
		 * userspace can route the event by CRTC -- this is what the
		 * cap advertises.
		 */
		c->value = 1;
		return (0);
	case DRM_CAP_VBLANK_HIGH_CRTC:
		/*
		 * WAIT_VBLANK accepts the high-CRTC bits in arg.request.type
		 * to disambiguate which pipe to wait on — supported.
		 */
		c->value = 1;
		return (0);
	case DRM_CAP_CURSOR_WIDTH:
	case DRM_CAP_CURSOR_HEIGHT:
		/*
		 * Wayland compositors (kwin_wayland, weston) read these
		 * before they will composite a pointer surface.  Reporting
		 * 0 made kwin stall trying to allocate a cursor BO of
		 * "max-supported" 0×0.  We don't have a hardware cursor
		 * plane yet so the compositor will draw the cursor into
		 * the primary surface (software-cursor), but it needs a
		 * non-zero hint to make that decision.  64×64 is the
		 * universal "small enough for any plane" SKL+ legal size.
		 */
		c->value = 64;
		return (0);
	case DRM_CAP_ADDFB2_MODIFIERS:
		/*
		 * We accept ADDFB2 with the FB_MODIFIERS flag set — the
		 * modifier value is ignored (we treat every surface as
		 * linear) but the ioctl shape is supported, which is what
		 * the cap actually advertises.  Mesa's GBM backend gates
		 * dmabuf import on this.
		 */
		c->value = 1;
		return (0);
	case DRM_CAP_ASYNC_PAGE_FLIP:
	case DRM_CAP_PAGE_FLIP_TARGET:
	case DRM_CAP_SYNCOBJ:
	case DRM_CAP_SYNCOBJ_TIMELINE:
	case DRM_CAP_ATOMIC_ASYNC_PAGE_FLIP:
		c->value = 0;
		return (0);
	}
	return (EINVAL);
}

int
kms_ioctl(struct cdev *cdev __unused, u_long cmd, caddr_t data,
    int fflag __unused, struct thread *td __unused)
{
	struct drm_file *file;
	int error;

	error = devfs_get_cdevpriv((void **)&file);
	if (error != 0)
		return (error);

	/*
	 * Trace only the "rare and can-hang" ioctls.  Skipping high-frequency
	 * read-side calls (GET_CAP, GETCRTC, GETPLANE, GETCONNECTOR,
	 * OBJ_GETPROPERTIES, ...) keeps the dmesg trace per second in the
	 * dozens instead of the thousands; the wedge-suspect ioctls are
	 * one-per-frame at most.
	 */
	if (kms_ioctl_trace > 0) {
		const char *name = NULL;
		char buf[24];

		switch (cmd) {
		case DRM_IOCTL_MODE_SETCRTC:
			name = "MODE_SETCRTC"; break;
		case DRM_IOCTL_MODE_PAGE_FLIP:
			name = "MODE_PAGE_FLIP"; break;
		case DRM_IOCTL_MODE_ATOMIC:
			name = "MODE_ATOMIC"; break;
		case DRM_IOCTL_WAIT_VBLANK:
			name = "WAIT_VBLANK"; break;
		case DRM_IOCTL_MODE_ADDFB2:
			name = "MODE_ADDFB2"; break;
		case DRM_IOCTL_MODE_RMFB:
			name = "MODE_RMFB"; break;
		case DRM_IOCTL_MODE_CREATE_DUMB:
			name = "MODE_CREATE_DUMB"; break;
		case DRM_IOCTL_MODE_DESTROY_DUMB:
			name = "MODE_DESTROY_DUMB"; break;
		case DRM_IOCTL_GEM_CLOSE:
			name = "GEM_CLOSE"; break;
		case DRM_IOCTL_PRIME_HANDLE_TO_FD:
			name = "PRIME_HANDLE_TO_FD"; break;
		case DRM_IOCTL_PRIME_FD_TO_HANDLE:
			name = "PRIME_FD_TO_HANDLE"; break;
		case DRM_IOCTL_SET_MASTER:
			name = "SET_MASTER"; break;
		case DRM_IOCTL_DROP_MASTER:
			name = "DROP_MASTER"; break;
		}
		/*
		 * Level 2: anything not in the curated list above gets printed
		 * by hex command number — needed to surface property-write /
		 * GETPLANE / GETCRTC traffic that the curated list skips when
		 * debugging a compositor wedge.
		 */
		if (name == NULL && kms_ioctl_trace >= 2) {
			snprintf(buf, sizeof(buf), "0x%lx", cmd);
			name = buf;
		}
		if (name != NULL)
			printf("kms: ioctl %s file=%p pid=%d is_render=%d\n",
			    name, file, curthread->td_proc->p_pid,
			    file->is_render_node ? 1 : 0);
	}

	/*
	 * Render-node ioctl gate.  Per Linux DRM render-node ABI, opens of
	 * /dev/dri/renderD<128+N> must reject KMS modesetting and event
	 * paths so Mesa's "which node am I on" probe identifies the cdev
	 * correctly and routes buffer allocation to the render node while
	 * keeping modesetting on the card node.  Permit GET_CAP /
	 * SET_CLIENT_CAP / VERSION / GET_RESOURCES (read-only inventory),
	 * dumb-buffer + PRIME (Mesa allocation surface), GEM CLOSE.
	 */
	if (file->is_render_node) {
		switch (cmd) {
		case DRM_IOCTL_SET_MASTER:
		case DRM_IOCTL_DROP_MASTER:
		case DRM_IOCTL_AUTH_MAGIC:
		case DRM_IOCTL_GET_MAGIC:
		case DRM_IOCTL_MODE_SETCRTC:
		case DRM_IOCTL_MODE_PAGE_FLIP:
		case DRM_IOCTL_MODE_ATOMIC:
		case DRM_IOCTL_MODE_ADDFB:
		case DRM_IOCTL_MODE_ADDFB2:
		case DRM_IOCTL_MODE_RMFB:
		case DRM_IOCTL_WAIT_VBLANK:
			return (EACCES);
		}
	}

	switch (cmd) {
	case DRM_IOCTL_MODE_LIST_LESSEES: {
		/*
		 * No DRM-lease support; report zero lessees.  Xorg's
		 * modesetting driver probes this during DRI2 init and
		 * derefs a result field unconditionally, so returning
		 * ENOTTY here makes it segfault rather than fall back to
		 * the no-leases path.  Just say "no lessees" and move on.
		 */
		struct drm_mode_list_lessees *r =
		    (struct drm_mode_list_lessees *)data;

		r->count_lessees = 0;
		r->pad = 0;
		return (0);
	}
	case DRM_IOCTL_SET_MASTER: {
		/*
		 * Become master on this device.  Demote any other current
		 * master on the same drm_device so file->is_master reflects
		 * the one-master invariant Linux DRM enforces.  Without this
		 * a second opener (Xwayland forked from kwin, a screencast
		 * helper, ...) calling SET_MASTER would remain non-master
		 * and any subsequent AUTH_MAGIC returns EACCES.
		 */
		struct drm_file *peer;

		sx_xlock(&file->dev->dev_lock);
		TAILQ_FOREACH(peer, &file->dev->files, link) {
			if (peer != file)
				peer->is_master = false;
		}
		file->is_master = true;
		sx_xunlock(&file->dev->dev_lock);
		return (0);
	}
	case DRM_IOCTL_DROP_MASTER:
		sx_xlock(&file->dev->dev_lock);
		file->is_master = false;
		sx_xunlock(&file->dev->dev_lock);
		return (0);
	case DRM_IOCTL_GET_MAGIC: {
		/*
		 * Legacy DRM auth.  A non-master client opens the cdev,
		 * calls GET_MAGIC to receive a cookie, hands the cookie to
		 * the master (Xorg, kwin_wayland) over a side channel.
		 * The master then calls AUTH_MAGIC with that cookie which
		 * marks the client's drm_file as authenticated, granting
		 * access to ioctls gated on file->authenticated.
		 *
		 * Modern render-node clients bypass this entirely (the
		 * render node is unauthenticated by design).  But kwin's
		 * card-node path still issues GET/AUTH_MAGIC and logs
		 * "Failed to authenticate the drm magic token" when we
		 * return ENOTTY.  Stubbing out the pair makes that log
		 * clean and unblocks any legacy DRI2 client that wanders
		 * onto our card node.
		 *
		 * Magic 0 means "not yet assigned" — bump file->magic to
		 * the device's next counter and hand it back.  Subsequent
		 * GET_MAGIC calls on the same file return the same value
		 * (idempotent per Linux behaviour).
		 */
		struct drm_auth *a = (struct drm_auth *)data;

		if (file->magic == 0) {
			sx_xlock(&file->dev->dev_lock);
			file->magic = ++file->dev->next_magic;
			sx_xunlock(&file->dev->dev_lock);
		}
		a->magic = file->magic;
		return (0);
	}
	case DRM_IOCTL_AUTH_MAGIC: {
		/*
		 * Master-side: walk every drm_file open on this device
		 * looking for one whose magic matches the cookie.  Mark
		 * authenticated, return success.  Non-matching cookie =
		 * EINVAL.  Master gate is loose — only the implicit
		 * is_master opener is allowed to call this.
		 */
		struct drm_auth *a = (struct drm_auth *)data;
		struct drm_file *peer;
		int err = EINVAL;

		if (!file->is_master)
			return (EACCES);
		if (a->magic == 0)
			return (EINVAL);

		sx_xlock(&file->dev->dev_lock);
		TAILQ_FOREACH(peer, &file->dev->files, link) {
			if (peer->magic == a->magic) {
				peer->authenticated = true;
				err = 0;
				break;
			}
		}
		sx_xunlock(&file->dev->dev_lock);
		return (err);
	}
	case DRM_IOCTL_VERSION:
		return (drm_ioctl_version(file, (struct drm_version *)data));
	case DRM_IOCTL_GET_UNIQUE:
		return (drm_ioctl_get_unique(file,
		    (struct drm_unique *)data));
	case DRM_IOCTL_SET_VERSION:
		return (drm_ioctl_set_version(file,
		    (struct drm_set_version *)data));
	case DRM_IOCTL_GET_CAP:
		return (drm_ioctl_get_cap(file, (struct drm_get_cap *)data));
	case DRM_IOCTL_MODE_GETRESOURCES:
		return (kms_ioctl_mode_getresources(file,
		    (struct drm_mode_card_res *)data));
	case DRM_IOCTL_MODE_GETCRTC:
		return (kms_ioctl_mode_getcrtc(file,
		    (struct drm_mode_crtc *)data));
	case DRM_IOCTL_MODE_GETENCODER:
		return (kms_ioctl_mode_getencoder(file,
		    (struct drm_mode_get_encoder *)data));
	case DRM_IOCTL_MODE_GETCONNECTOR:
		return (kms_ioctl_mode_getconnector(file,
		    (struct drm_mode_get_connector *)data));
	case DRM_IOCTL_MODE_GETPLANERESOURCES:
		return (kms_ioctl_mode_getplane_resources(file,
		    (struct drm_mode_get_plane_res *)data));
	case DRM_IOCTL_MODE_GETPLANE:
		return (kms_ioctl_mode_getplane(file,
		    (struct drm_mode_get_plane *)data));
	case DRM_IOCTL_MODE_CREATE_DUMB:
		return (kms_ioctl_mode_create_dumb(file,
		    (struct drm_mode_create_dumb *)data));
	case DRM_IOCTL_MODE_MAP_DUMB:
		return (kms_ioctl_mode_map_dumb(file,
		    (struct drm_mode_map_dumb *)data));
	case DRM_IOCTL_MODE_DESTROY_DUMB:
		return (kms_ioctl_mode_destroy_dumb(file,
		    (struct drm_mode_destroy_dumb *)data));
	case DRM_IOCTL_GEM_CLOSE:
		return (kms_gem_handle_delete(file,
		    ((struct drm_gem_close *)data)->handle));
	case DRM_IOCTL_PRIME_HANDLE_TO_FD:
		return (kms_ioctl_prime_handle_to_fd(file,
		    (struct drm_prime_handle *)data));
	case DRM_IOCTL_PRIME_FD_TO_HANDLE:
		return (kms_ioctl_prime_fd_to_handle(file,
		    (struct drm_prime_handle *)data));
	case DRM_IOCTL_MODE_ADDFB:
		return (kms_ioctl_mode_addfb(file,
		    (struct drm_mode_fb_cmd *)data));
	case DRM_IOCTL_MODE_ADDFB2:
		return (kms_ioctl_mode_addfb2(file,
		    (struct drm_mode_fb_cmd2 *)data));
	case DRM_IOCTL_MODE_RMFB:
		return (kms_ioctl_mode_rmfb(file, (uint32_t *)data));
	case DRM_IOCTL_MODE_SETCRTC:
		return (kms_ioctl_mode_setcrtc(file,
		    (struct drm_mode_crtc *)data));
	case DRM_IOCTL_MODE_PAGE_FLIP:
		return (kms_ioctl_mode_page_flip(file,
		    (struct drm_mode_crtc_page_flip *)data));
	case DRM_IOCTL_SET_CLIENT_CAP:
		return (kms_ioctl_set_client_cap(file,
		    (struct drm_set_client_cap *)data));
	case DRM_IOCTL_MODE_GETPROPERTY:
		return (kms_ioctl_mode_getproperty(file,
		    (struct drm_mode_get_property *)data));
	case DRM_IOCTL_MODE_OBJ_GETPROPERTIES:
		return (kms_ioctl_mode_obj_getproperties(file,
		    (struct drm_mode_obj_get_properties *)data));
	case DRM_IOCTL_MODE_OBJ_SETPROPERTY:
		return (kms_ioctl_mode_obj_setproperty(file,
		    (struct drm_mode_obj_set_property *)data));
	case DRM_IOCTL_MODE_CREATEPROPBLOB:
		return (kms_ioctl_mode_createpropblob(file,
		    (struct drm_mode_create_blob *)data));
	case DRM_IOCTL_MODE_DESTROYPROPBLOB:
		return (kms_ioctl_mode_destroypropblob(file,
		    (struct drm_mode_destroy_blob *)data));
	case DRM_IOCTL_MODE_GETPROPBLOB:
		return (kms_ioctl_mode_getpropblob(file,
		    (struct drm_mode_get_blob *)data));
	case DRM_IOCTL_MODE_ATOMIC:
		return (kms_ioctl_mode_atomic(file,
		    (struct drm_mode_atomic *)data));
	case DRM_IOCTL_WAIT_VBLANK:
		return (kms_ioctl_wait_vblank(file,
		    (union drm_wait_vblank *)data));
	case DRM_IOCTL_MODE_CURSOR:
		return (kms_ioctl_mode_cursor(file,
		    (struct drm_mode_cursor *)data));
	case DRM_IOCTL_MODE_CURSOR2:
		return (kms_ioctl_mode_cursor2(file,
		    (struct drm_mode_cursor2 *)data));
	}
	if (file->dev->driver != NULL && file->dev->driver->ioctl != NULL)
		return (file->dev->driver->ioctl(file, cmd, data));
	return (ENOTTY);
}
