/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Native kms(4) consumer for the paravirtual VirtIO GPU device.
 *
 * Sits alongside rk_kms (Rockchip VOP) and igen (Haswell iGPU) in
 * sys/dev/kms/drivers/.  Depends on kms.ko for the framework core;
 * the framework itself never learns about virtio-gpu.
 *
 * The in-base virtio_gpu(4) driver owns the vt(4) console framebuffer
 * (fbio path).  This driver binds at BUS_PROBE_VENDOR so it wins the
 * device_t attach, then hands the fb interface back to virtio_gpu's
 * fbio machinery via a firstopen/lastclose handover — one device_t,
 * two consumers, no fights.
 *
 * Phase A (this commit): softc + probe/attach/detach + drm_driver
 * registration only.  Real virtio_gpu protocol wiring, mode-config
 * topology (CRTC/plane/encoder/connector), GEM/framebuffer and
 * console handover land in Phase B..F.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/sglist.h>
#include <sys/sx.h>
#include <sys/sysctl.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/virtio/virtio.h>
#include <dev/virtio/virtqueue.h>
#include <dev/virtio/gpu/virtio_gpu.h>

#include <kms/drm_atomic.h>
#include <kms/drm_connector.h>
#include <kms/drm_crtc.h>
#include <kms/drm_device.h>
#include <kms/drm_drv.h>
#include <kms/drm_encoder.h>
#include <kms/drm_framebuffer.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_modes.h>
#include <kms/drm_plane.h>

#include <drm/drm_fourcc.h>
#include <drm/drm_mode.h>

#include "virtio_if.h"

/*
 * Phase B protocol constants.  VIRTIO_GPU_MAX_SCANOUTS is 16 in the
 * spec (hardcoded, not negotiated); we cap our per-instance array at
 * that so a malicious host can't overflow us if the config lies about
 * num_scanouts.  DISPLAY_INFO fetch happens once at attach and once
 * per hotplug event; the cached result feeds Phase C connector-mode
 * enumeration.
 */
#define	VIRTIO_KMS_MAX_SCANOUTS		VIRTIO_GPU_MAX_SCANOUTS

/*
 * Cached per-scanout state: the display info reply enables and sizes
 * each scanout that the host has active.  Extended in Phase F with
 * the EDID blob when the host advertises VIRTIO_GPU_F_EDID.
 */
struct virtio_kms_scanout {
	uint32_t	enabled;
	uint32_t	width;
	uint32_t	height;
	uint32_t	x;
	uint32_t	y;
	uint32_t	flags;		/* pmode.flags from resp */
};

#define	VIRTIO_KMS_DRIVER_NAME	"virtio_kms"
#define	VIRTIO_KMS_DRIVER_DESC	"VirtIO GPU (kms)"
#define	VIRTIO_KMS_DRIVER_DATE	"20260804"

MALLOC_DEFINE(M_VIRTIO_KMS, "virtio_kms", "VirtIO KMS driver");

/*
 * Per-instance state.  Attach fills the softc, kms_dev_register hands
 * back the shared struct drm_device.  Ordering is important: the softc
 * has to survive as long as drm_dev because the framework can call
 * back into our driver ops from any file-op path.
 */
struct virtio_kms_softc {
	device_t		 dev;
	struct drm_device	*drm_dev;

	struct sx		 sx;		/* driver-wide serializer */
	struct mtx		 mtx;		/* short critical sections */

	/* virtio plumbing (Phase B). */
	struct virtqueue	*ctrl_vq;	/* command / response */
	struct virtqueue	*cursor_vq;	/* cursor updates */
	struct sx		 ctrl_sx;	/* serialize ctrl_vq req/resp */
	uint32_t		 features;	/* negotiated VIRTIO_GPU_F_* */
	uint64_t		 next_fence;	/* fence_id counter */

	/* Config snapshot + cached scanout topology. */
	struct virtio_gpu_config gpucfg;
	uint32_t		 num_scanouts;	/* clamped to MAX */
	struct virtio_kms_scanout scanouts[VIRTIO_KMS_MAX_SCANOUTS];

	/*
	 * Mode-config topology (Phase C).  Single-scanout for now;
	 * multi-scanout is Phase F.  Objects embed directly in the softc
	 * so their lifetime is trivially bounded by attach/detach.
	 */
	struct drm_crtc		 crtc;
	struct drm_plane	 primary;
	struct drm_encoder	 encoder;
	struct drm_connector	 connector;

	int			 debug;		/* dev.virtio_kms.N.debug */
};

#define	VKMS_DPRINTF(sc, ...) do {					\
	if ((sc)->debug > 0)						\
		device_printf((sc)->dev, __VA_ARGS__);			\
} while (0)

/* Forward decls for the mode-config layer used by attach. */
static int	virtio_kms_topology_init(struct virtio_kms_softc *sc);
static void	virtio_kms_topology_teardown(struct virtio_kms_softc *sc);
static const struct drm_mode_config_funcs virtio_kms_mode_config_funcs;

/*
 * Driver descriptor.  driver_features=0 for now; ATOMIC / RENDER caps
 * flip on once the topology and command queue are wired.
 */
static const struct drm_driver virtio_kms_driver = {
	.name		= VIRTIO_KMS_DRIVER_NAME,
	.desc		= VIRTIO_KMS_DRIVER_DESC,
	.date		= VIRTIO_KMS_DRIVER_DATE,
	.major		= 0,
	.minor		= 1,
	.patchlevel	= 0,
	.driver_features = 0,
};

static struct virtio_feature_desc virtio_kms_feature_desc[] = {
	{ VIRTIO_GPU_F_VIRGL,		"VIRGL" },
	{ VIRTIO_GPU_F_EDID,		"EDID" },
	{ 0, NULL }
};

/*
 * Read the device config snapshot into sc->gpucfg.  Mirrors the
 * in-base virtio_gpu(4) VTGPU_GET_CONFIG expansion; centralised here
 * so Phase F hotplug-event handling can re-read events_read / clear
 * without duplicating the offset math.
 */
static void
virtio_kms_read_config(struct virtio_kms_softc *sc)
{
	device_t dev = sc->dev;
	struct virtio_gpu_config *cfg = &sc->gpucfg;

#define	VKMS_GET_CFG(_field)						\
	virtio_read_device_config(dev,					\
	    offsetof(struct virtio_gpu_config, _field),			\
	    &cfg->_field, sizeof(cfg->_field))

	VKMS_GET_CFG(events_read);
	VKMS_GET_CFG(events_clear);
	VKMS_GET_CFG(num_scanouts);
	VKMS_GET_CFG(num_capsets);

#undef	VKMS_GET_CFG
}

/*
 * Allocate the two GPU virtqueues.  ctrl_vq carries every command +
 * its response (2D setup, scanout config, EDID, display-info,
 * hotplug), cursor_vq carries VIRTIO_GPU_CMD_UPDATE_CURSOR /
 * MOVE_CURSOR.  Both queues stay untouched until Phase C wires the
 * command surfaces to them.
 */
static int
virtio_kms_alloc_queues(struct virtio_kms_softc *sc)
{
	struct vq_alloc_info info[2];
	int nvqs = 2;

	VQ_ALLOC_INFO_INIT(&info[0], 0, NULL, sc, &sc->ctrl_vq,
	    "%s ctrl", device_get_nameunit(sc->dev));
	VQ_ALLOC_INFO_INIT(&info[1], 0, NULL, sc, &sc->cursor_vq,
	    "%s cursor", device_get_nameunit(sc->dev));

	return (virtio_alloc_virtqueues(sc->dev, nvqs, info));
}

/*
 * Synchronous request / response over ctrl_vq.  Mirrors the in-base
 * driver's vtgpu_req_resp: build a two-segment sglist (req is device-
 * read, resp is device-write), enqueue with 1 readable + 1 writable
 * seg, notify the queue, poll for completion.
 *
 * Caller fills the command header (type / flags / fence_id) inside
 * req.  We hold ctrl_sx across the whole cycle so only one in-flight
 * command is on the queue at a time; the spec allows out-of-order
 * completion but Phase B never issues more than one request in
 * parallel, and the polled-completion path is cheaper than a
 * per-request wait.
 */
static int
virtio_kms_req_resp(struct virtio_kms_softc *sc, void *req, size_t reqlen,
    void *resp, size_t resplen)
{
	struct sglist sg;
	struct sglist_seg segs[2];
	int error;

	sglist_init(&sg, 2, segs);
	error = sglist_append(&sg, req, reqlen);
	if (error != 0) {
		device_printf(sc->dev,
		    "req_resp: sglist_append(req) rc=%d\n", error);
		return (error);
	}
	error = sglist_append(&sg, resp, resplen);
	if (error != 0) {
		device_printf(sc->dev,
		    "req_resp: sglist_append(resp) rc=%d\n", error);
		return (error);
	}

	sx_xlock(&sc->ctrl_sx);
	error = virtqueue_enqueue(sc->ctrl_vq, resp, &sg, 1, 1);
	if (error != 0) {
		sx_xunlock(&sc->ctrl_sx);
		device_printf(sc->dev,
		    "req_resp: virtqueue_enqueue rc=%d\n", error);
		return (error);
	}
	virtqueue_notify(sc->ctrl_vq);
	virtqueue_poll(sc->ctrl_vq, NULL);
	sx_xunlock(&sc->ctrl_sx);

	return (0);
}

/*
 * Issue GET_DISPLAY_INFO, cache the pmode[] table into sc->scanouts.
 * The response gives up to VIRTIO_GPU_MAX_SCANOUTS entries; the host
 * marks each one enabled + sizes the mode.  Phase C hands this table
 * to the connector-mode enumeration; Phase F re-runs it on hotplug.
 */
static int
virtio_kms_fetch_display_info(struct virtio_kms_softc *sc)
{
	struct {
		struct virtio_gpu_ctrl_hdr	req;
		char				pad;
		struct virtio_gpu_resp_display_info resp;
	} s;
	uint32_t nscan, i;
	int error;

	bzero(&s, sizeof(s));
	s.req.type = htole32(VIRTIO_GPU_CMD_GET_DISPLAY_INFO);
	s.req.flags = htole32(VIRTIO_GPU_FLAG_FENCE);
	s.req.fence_id = htole64(atomic_fetchadd_64(&sc->next_fence, 1));

	error = virtio_kms_req_resp(sc, &s.req, sizeof(s.req),
	    &s.resp, sizeof(s.resp));
	if (error != 0)
		return (error);

	if (le32toh(s.resp.hdr.type) != VIRTIO_GPU_RESP_OK_DISPLAY_INFO) {
		device_printf(sc->dev,
		    "display_info: unexpected resp type 0x%x\n",
		    le32toh(s.resp.hdr.type));
		return (EIO);
	}

	nscan = sc->gpucfg.num_scanouts;
	if (nscan > VIRTIO_KMS_MAX_SCANOUTS)
		nscan = VIRTIO_KMS_MAX_SCANOUTS;
	sc->num_scanouts = nscan;

	bzero(sc->scanouts, sizeof(sc->scanouts));
	for (i = 0; i < nscan; i++) {
		sc->scanouts[i].enabled = le32toh(s.resp.pmodes[i].enabled);
		sc->scanouts[i].width   = le32toh(s.resp.pmodes[i].r.width);
		sc->scanouts[i].height  = le32toh(s.resp.pmodes[i].r.height);
		sc->scanouts[i].x       = le32toh(s.resp.pmodes[i].r.x);
		sc->scanouts[i].y       = le32toh(s.resp.pmodes[i].r.y);
		sc->scanouts[i].flags   = le32toh(s.resp.pmodes[i].flags);
		VKMS_DPRINTF(sc, "scanout[%u] enabled=%u %ux%u @%u,%u\n",
		    i, sc->scanouts[i].enabled,
		    sc->scanouts[i].width, sc->scanouts[i].height,
		    sc->scanouts[i].x, sc->scanouts[i].y);
	}
	return (0);
}

static int
virtio_kms_probe(device_t dev)
{
	if (virtio_get_device_type(dev) != VIRTIO_ID_GPU)
		return (ENXIO);
	device_set_desc(dev, VIRTIO_KMS_DRIVER_DESC);
	/*
	 * Beat the in-base virtio_gpu(4) at BUS_PROBE_VENDOR.  In-base
	 * driver stays loadable and keeps its fbio hooks; console
	 * ownership handover is done at firstopen/lastclose time
	 * (Phase E), not by driver replacement.
	 */
	return (BUS_PROBE_VENDOR);
}

static int
virtio_kms_attach(device_t dev)
{
	struct virtio_kms_softc *sc;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->next_fence = 1;
	sx_init(&sc->sx, "virtio_kms");
	sx_init(&sc->ctrl_sx, "virtio_kms ctrl");
	mtx_init(&sc->mtx, "virtio_kms", NULL, MTX_DEF);

	virtio_set_feature_desc(dev, virtio_kms_feature_desc);
	sc->features = virtio_negotiate_features(dev, 0);

	virtio_kms_read_config(sc);

	error = virtio_kms_alloc_queues(sc);
	if (error != 0) {
		device_printf(dev, "alloc_queues failed: %d\n", error);
		goto fail_locks;
	}
	error = virtio_setup_intr(dev, INTR_TYPE_MISC);
	if (error != 0) {
		device_printf(dev, "setup_intr failed: %d\n", error);
		goto fail_locks;
	}

	error = virtio_kms_fetch_display_info(sc);
	if (error != 0) {
		device_printf(dev,
		    "fetch_display_info rc=%d — will retry on hotplug\n",
		    error);
		/* Non-fatal: attach continues without a topology snapshot. */
	}

	error = kms_dev_register(&virtio_kms_driver, sc, &sc->drm_dev);
	if (error != 0) {
		device_printf(dev,
		    "kms_dev_register failed: %d\n", error);
		goto fail_locks;
	}

	/*
	 * Install the atomic hooks BEFORE topology_init so any
	 * userspace-visible ATOMIC ioctl that races the attach path
	 * takes the real check + commit dispatch rather than the
	 * framework's legacy property-table fallback.
	 */
	sc->drm_dev->mode_config.funcs = &virtio_kms_mode_config_funcs;

	error = virtio_kms_topology_init(sc);
	if (error != 0) {
		device_printf(dev, "topology_init failed: %d\n", error);
		kms_dev_unregister(sc->drm_dev);
		sc->drm_dev = NULL;
		goto fail_locks;
	}

	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "debug", CTLFLAG_RW, &sc->debug, 0,
	    "Debug verbosity (0 = quiet)");
	SYSCTL_ADD_UINT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "num_scanouts", CTLFLAG_RD, &sc->num_scanouts, 0,
	    "Number of active scanouts reported by the host");

	device_printf(dev,
	    "attached (features 0x%x, num_scanouts %u, num_capsets %u)\n",
	    sc->features, sc->gpucfg.num_scanouts, sc->gpucfg.num_capsets);
	return (0);

fail_locks:
	mtx_destroy(&sc->mtx);
	sx_destroy(&sc->ctrl_sx);
	sx_destroy(&sc->sx);
	return (error);
}

static int
virtio_kms_detach(device_t dev)
{
	struct virtio_kms_softc *sc = device_get_softc(dev);

	if (sc->drm_dev != NULL) {
		virtio_kms_topology_teardown(sc);
		kms_dev_unregister(sc->drm_dev);
		sc->drm_dev = NULL;
	}
	mtx_destroy(&sc->mtx);
	sx_destroy(&sc->ctrl_sx);
	sx_destroy(&sc->sx);
	return (0);
}

/*
 * ============================================================
 * Phase C: mode-config topology
 * ============================================================
 *
 * Single scanout so far: one CRTC + one primary plane + one
 * writeback-flavour virtual encoder + one virtual connector.  The
 * connector's `get_modes` hook consumes the cached sc->scanouts[0]
 * entry populated by Phase B's fetch_display_info, so userspace
 * (Xorg / Wayland / kmscube probe) sees whatever geometry the host
 * currently advertises.
 *
 * Phase F extends to N scanouts, at which point CRTC/plane/encoder/
 * connector each grow into per-scanout arrays.  Phase D wires the
 * atomic_commit body to actually push a SET_SCANOUT down ctrl_vq;
 * this commit only sets the funcs table so the property-driven ATOMIC
 * path can drive the framework's own state machine.
 */

/*
 * Look up the softc from any of the four topology objects by walking
 * back through drm_dev->driver_priv.  Cheaper than embedding a back
 * pointer in each object.
 */
static inline struct virtio_kms_softc *
virtio_kms_dev_to_sc(struct drm_device *drm_dev)
{
	return ((struct virtio_kms_softc *)drm_dev->driver_priv);
}

/*
 * Connector .get_modes: publish one CVT-derived preferred mode per
 * enabled scanout entry.  Phase C exposes only scanouts[0]; Phase F
 * multiplexes onto N connectors so each one gets its own scanout row.
 * Returns the count of modes added.
 */
static int
virtio_kms_get_modes(struct drm_connector *connector)
{
	struct virtio_kms_softc *sc = virtio_kms_dev_to_sc(connector->dev);
	struct drm_display_mode *mode;
	const struct virtio_kms_scanout *s = &sc->scanouts[0];
	uint32_t hdisplay, vdisplay;
	int added = 0;

	if (sc->num_scanouts == 0 || s->enabled == 0) {
		/*
		 * Fallback until we have live topology (e.g. attach ran but
		 * fetch_display_info bounced): advertise 1024x768 so
		 * userspace still gets something to bind against.  Phase F
		 * removes this fallback once hotplug re-runs display_info.
		 */
		hdisplay = 1024;
		vdisplay = 768;
	} else {
		hdisplay = s->width;
		vdisplay = s->height;
	}

	mode = kms_mode_create();
	if (mode == NULL)
		return (0);
	mode->hdisplay = hdisplay;
	mode->vdisplay = vdisplay;
	mode->hsync_start = hdisplay + 88;
	mode->hsync_end   = hdisplay + 88 + 44;
	mode->htotal      = hdisplay + 88 + 44 + 148;
	mode->vsync_start = vdisplay + 4;
	mode->vsync_end   = vdisplay + 4 + 5;
	mode->vtotal      = vdisplay + 4 + 5 + 36;
	mode->clock       = mode->htotal * mode->vtotal * 60 / 1000;
	mode->type        = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	mode->vrefresh    = 60;
	kms_mode_set_name(mode);
	kms_connector_add_mode(connector, mode);
	added++;

	VKMS_DPRINTF(sc, "get_modes: published %ux%u (added=%d)\n",
	    hdisplay, vdisplay, added);
	return (added);
}

/*
 * CRTC set_config: legacy modeset ioctl entry.  Phase C stubs it out
 * so the framework's legacy-property fallback doesn't hit a NULL and
 * userspace's SETCRTC returns success even before the real modeset
 * lands.  Phase D fills in the SET_SCANOUT + RESOURCE_FLUSH sequence
 * against the framebuffer the caller passed.
 */
static int
virtio_kms_set_config(struct drm_mode_set *set)
{
	if (set == NULL || set->crtc == NULL)
		return (EINVAL);
	return (0);
}

static const struct drm_crtc_funcs virtio_kms_crtc_funcs = {
	.set_config = virtio_kms_set_config,
};
static const struct drm_plane_funcs virtio_kms_plane_funcs = { 0 };
static const struct drm_encoder_funcs virtio_kms_encoder_funcs = { 0 };
static const struct drm_connector_funcs virtio_kms_connector_funcs = {
	.get_modes = virtio_kms_get_modes,
};

/*
 * Atomic check: sanity-only in Phase C.  The framework hands us a
 * fully-populated drm_atomic_state and we walk it for obvious
 * garbage.  Phase D adds framebuffer-side validation.
 */
static int
virtio_kms_atomic_check(struct drm_device *dev __unused,
    struct drm_atomic_state *state)
{
	uint32_t i;

	if (state == NULL)
		return (EINVAL);
	for (i = 0; i < state->num_crtc; i++) {
		const struct drm_crtc_state *cs = state->crtc_states[i];

		if (cs == NULL || !cs->mode_changed || !cs->active)
			continue;
		if (cs->mode.hdisplay == 0 || cs->mode.vdisplay == 0 ||
		    cs->mode.clock == 0)
			return (EINVAL);
	}
	return (0);
}

/*
 * Atomic commit: Phase C is a no-op wrapper that just returns success.
 * Phase D walks the CRTC states and issues VIRTIO_GPU_CMD_SET_SCANOUT
 * + RESOURCE_FLUSH for each mode_changed / active_changed CRTC,
 * using the framebuffer routed to that CRTC's primary plane.
 */
static int
virtio_kms_atomic_commit(struct drm_device *dev __unused,
    struct drm_atomic_state *state __unused, bool nonblock __unused)
{
	return (0);
}

static const struct drm_mode_config_funcs virtio_kms_mode_config_funcs = {
	.atomic_check  = virtio_kms_atomic_check,
	.atomic_commit = virtio_kms_atomic_commit,
};

/*
 * Primary-plane format list.  virtio-gpu's 2D command surface
 * accepts BGRA / XRGB variants; XRGB8888 is the universal fit
 * for Xorg-modesetting + Wayland weston.  Phase F extends this
 * once the driver knows the negotiated feature set (e.g. VirGL
 * enables 3D formats).
 */
static const uint32_t virtio_kms_primary_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

static int
virtio_kms_topology_init(struct virtio_kms_softc *sc)
{
	struct drm_mode_config *mc = &sc->drm_dev->mode_config;
	uint32_t crtc_mask;
	int error;

	/*
	 * Cap the mode-config canvas at 4Kx4K.  virtio-gpu can advertise
	 * larger surfaces if the host is willing, but 4K is a sane
	 * default that matches rk_kms and every commodity display userspace
	 * currently tries.
	 */
	mc->max_width  = 4096;
	mc->max_height = 4096;

	error = kms_crtc_init(sc->drm_dev, &sc->crtc,
	    &virtio_kms_crtc_funcs);
	if (error != 0)
		return (error);
	crtc_mask = 1u << sc->crtc.index;

	error = kms_plane_init(sc->drm_dev, &sc->primary,
	    &virtio_kms_plane_funcs, DRM_PLANE_TYPE_PRIMARY, crtc_mask,
	    virtio_kms_primary_formats,
	    nitems(virtio_kms_primary_formats));
	if (error != 0)
		goto fail_crtc;
	sc->crtc.primary_plane = &sc->primary;

	error = kms_encoder_init(sc->drm_dev, &sc->encoder,
	    &virtio_kms_encoder_funcs, DRM_MODE_ENCODER_VIRTUAL);
	if (error != 0)
		goto fail_plane;
	sc->encoder.possible_crtcs = crtc_mask;

	error = kms_connector_init(sc->drm_dev, &sc->connector,
	    &virtio_kms_connector_funcs, DRM_MODE_CONNECTOR_VIRTUAL);
	if (error != 0)
		goto fail_encoder;
	sc->connector.status = connector_status_connected;
	kms_connector_attach_encoder(&sc->connector, &sc->encoder);

	return (0);

fail_encoder:
	kms_encoder_cleanup(&sc->encoder);
fail_plane:
	kms_plane_cleanup(&sc->primary);
fail_crtc:
	kms_crtc_cleanup(&sc->crtc);
	return (error);
}

static void
virtio_kms_topology_teardown(struct virtio_kms_softc *sc)
{
	kms_connector_cleanup(&sc->connector);
	kms_encoder_cleanup(&sc->encoder);
	kms_plane_cleanup(&sc->primary);
	kms_crtc_cleanup(&sc->crtc);
}

static device_method_t virtio_kms_methods[] = {
	DEVMETHOD(device_probe,		virtio_kms_probe),
	DEVMETHOD(device_attach,	virtio_kms_attach),
	DEVMETHOD(device_detach,	virtio_kms_detach),
	DEVMETHOD_END
};

static driver_t virtio_kms_driver_kdrv = {
	VIRTIO_KMS_DRIVER_NAME,
	virtio_kms_methods,
	sizeof(struct virtio_kms_softc),
};

DRIVER_MODULE(virtio_kms, virtio_pci, virtio_kms_driver_kdrv, NULL, NULL);
DRIVER_MODULE(virtio_kms, virtio_mmio, virtio_kms_driver_kdrv, NULL, NULL);
MODULE_VERSION(virtio_kms, 1);
MODULE_DEPEND(virtio_kms, kms, 1, 1, 1);
MODULE_DEPEND(virtio_kms, virtio, 1, 1, 1);
