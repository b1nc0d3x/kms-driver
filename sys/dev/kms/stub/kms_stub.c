/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * kms stub: registers a fake card with one CRTC, one encoder,
 * and one connector so userspace can validate the cdev + KMS object
 * lifecycle without any real silicon driver.  Carries no
 * framebuffers, no modes, no atomic state — those land in later
 * phases.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>

#include <drm/drm_fourcc.h>
#include <drm/drm_mode.h>
#include <kms/drm_connector.h>
#include <kms/drm_crtc.h>
#include <kms/drm_device.h>
#include <kms/drm_drv.h>
#include <kms/drm_encoder.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_modes.h>
#include <kms/drm_plane.h>

/*
 * Reuse M_KMS from the framework.  Defining a second malloc
 * type from a stub during kldload has triggered uma zone creation
 * issues; sharing the framework's type avoids that path entirely.
 */
MALLOC_DECLARE(M_KMS);

static const struct drm_driver kms_stub_driver = {
	.name		= "stub",
	.desc		= "KMS phase-5 stub",
	.date		= "20260608",
	.major		= 0,
	.minor		= 5,
	.patchlevel	= 0,
	.driver_features = 0,
};

static struct drm_device	*kms_stub_dev;
static struct drm_crtc		 kms_stub_crtc_storage;
static struct drm_plane		 kms_stub_primary_storage;
static struct drm_plane		 kms_stub_cursor_storage;
static struct drm_encoder	 kms_stub_encoder_storage;
static struct drm_connector	 kms_stub_connector_storage;
static struct drm_crtc		*kms_stub_crtc;
static struct drm_plane		*kms_stub_primary;
static struct drm_plane		*kms_stub_cursor;
static struct drm_encoder	*kms_stub_encoder;
static struct drm_connector	*kms_stub_connector;

static const uint32_t stub_primary_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGB565,
};

static const uint32_t stub_cursor_formats[] = {
	DRM_FORMAT_ARGB8888,
};

static void
stub_crtc_destroy(struct drm_crtc *crtc __unused)
{
}

static void
stub_encoder_destroy(struct drm_encoder *encoder __unused)
{
}

static void
stub_connector_destroy(struct drm_connector *connector __unused)
{
}

/*
 * Hand-rolled CEA-861 1920x1080@60 (148.5 MHz pixel clock).  Stub fills
 * a single mode so userspace probes returning count_modes > 0 can be
 * exercised end-to-end without an EDID parser.
 */
static int
stub_connector_get_modes(struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	if (connector->mode_count > 0)
		return (0);

	mode = kms_mode_create();
	mode->clock = 148500;
	mode->hdisplay = 1920;
	mode->hsync_start = 2008;
	mode->hsync_end = 2052;
	mode->htotal = 2200;
	mode->vdisplay = 1080;
	mode->vsync_start = 1084;
	mode->vsync_end = 1089;
	mode->vtotal = 1125;
	mode->flags = KMS_MODE_FLAG_PHSYNC | KMS_MODE_FLAG_PVSYNC;
	mode->type = KMS_MODE_TYPE_DRIVER |
	    KMS_MODE_TYPE_PREFERRED;
	kms_connector_add_mode(connector, mode);
	return (1);
}

static void
stub_plane_destroy(struct drm_plane *plane __unused)
{
}

static const struct drm_crtc_funcs stub_crtc_funcs = {
	.destroy = stub_crtc_destroy,
};

static const struct drm_encoder_funcs stub_encoder_funcs = {
	.destroy = stub_encoder_destroy,
};

static const struct drm_connector_funcs stub_connector_funcs = {
	.destroy = stub_connector_destroy,
	.get_modes = stub_connector_get_modes,
};

static const struct drm_plane_funcs stub_plane_funcs = {
	.destroy = stub_plane_destroy,
};

static int
kms_stub_attach(void)
{
	struct drm_mode_config *mc = &kms_stub_dev->mode_config;
	struct drm_crtc *crtc;
	struct drm_plane *primary, *cursor;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	uint32_t crtc_mask;
	int error;

	/*
	 * Defaults a real driver would override before registering any
	 * connector.  4096 caps a not-yet-implemented modeset path —
	 * sanity for userspace probes that print mode_config limits.
	 */
	mc->min_width = 0;
	mc->max_width = 4096;
	mc->min_height = 0;
	mc->max_height = 4096;

	memset(&kms_stub_crtc_storage, 0,
	    sizeof(kms_stub_crtc_storage));
	memset(&kms_stub_primary_storage, 0,
	    sizeof(kms_stub_primary_storage));
	memset(&kms_stub_cursor_storage, 0,
	    sizeof(kms_stub_cursor_storage));
	memset(&kms_stub_encoder_storage, 0,
	    sizeof(kms_stub_encoder_storage));
	memset(&kms_stub_connector_storage, 0,
	    sizeof(kms_stub_connector_storage));
	crtc = &kms_stub_crtc_storage;
	primary = &kms_stub_primary_storage;
	cursor = &kms_stub_cursor_storage;
	encoder = &kms_stub_encoder_storage;
	connector = &kms_stub_connector_storage;

	error = kms_crtc_init(kms_stub_dev, crtc, &stub_crtc_funcs);
	if (error != 0)
		return (error);
	kms_stub_crtc = crtc;
	crtc_mask = 1u << crtc->index;

	error = kms_plane_init(kms_stub_dev, primary,
	    &stub_plane_funcs, DRM_PLANE_TYPE_PRIMARY, crtc_mask,
	    stub_primary_formats, nitems(stub_primary_formats));
	if (error != 0)
		goto fail_primary;
	kms_stub_primary = primary;
	crtc->primary_plane = primary;

	error = kms_plane_init(kms_stub_dev, cursor,
	    &stub_plane_funcs, DRM_PLANE_TYPE_CURSOR, crtc_mask,
	    stub_cursor_formats, nitems(stub_cursor_formats));
	if (error != 0)
		goto fail_cursor;
	kms_stub_cursor = cursor;

	error = kms_encoder_init(kms_stub_dev, encoder,
	    &stub_encoder_funcs, DRM_MODE_ENCODER_TMDS);
	if (error != 0)
		goto fail_encoder;
	encoder->possible_crtcs = crtc_mask;
	kms_stub_encoder = encoder;

	error = kms_connector_init(kms_stub_dev, connector,
	    &stub_connector_funcs, DRM_MODE_CONNECTOR_HDMIA);
	if (error != 0)
		goto fail_connector;
	connector->status = connector_status_disconnected;
	kms_connector_attach_encoder(connector, encoder);
	kms_stub_connector = connector;

	return (0);

fail_connector:
	kms_encoder_cleanup(encoder);
	kms_stub_encoder = NULL;
fail_encoder:
	kms_plane_cleanup(cursor);
	kms_stub_cursor = NULL;
fail_cursor:
	kms_plane_cleanup(primary);
	kms_stub_primary = NULL;
	crtc->primary_plane = NULL;
fail_primary:
	kms_crtc_cleanup(crtc);
	kms_stub_crtc = NULL;
	return (error);
}

static void
kms_stub_detach(void)
{
	if (kms_stub_connector != NULL) {
		kms_connector_cleanup(kms_stub_connector);
		kms_stub_connector = NULL;
	}
	if (kms_stub_encoder != NULL) {
		kms_encoder_cleanup(kms_stub_encoder);
		kms_stub_encoder = NULL;
	}
	if (kms_stub_cursor != NULL) {
		kms_plane_cleanup(kms_stub_cursor);
		kms_stub_cursor = NULL;
	}
	if (kms_stub_primary != NULL) {
		kms_plane_cleanup(kms_stub_primary);
		kms_stub_primary = NULL;
	}
	if (kms_stub_crtc != NULL) {
		kms_crtc_cleanup(kms_stub_crtc);
		kms_stub_crtc = NULL;
	}
}

static int
kms_stub_modevent(module_t mod __unused, int what, void *arg __unused)
{
	int error;

	switch (what) {
	case MOD_LOAD:
		error = kms_dev_register(&kms_stub_driver, NULL,
		    &kms_stub_dev);
		if (error != 0) {
			printf("kms_stub: register failed: %d\n", error);
			return (error);
		}
		printf("kms_stub: about to attach KMS objects\n");
		error = kms_stub_attach();
		printf("kms_stub: attach returned %d\n", error);
		if (error != 0) {
			kms_dev_unregister(kms_stub_dev);
			kms_stub_dev = NULL;
			return (error);
		}
		return (0);
	case MOD_UNLOAD:
		kms_stub_detach();
		if (kms_stub_dev != NULL) {
			kms_dev_unregister(kms_stub_dev);
			kms_stub_dev = NULL;
		}
		return (0);
	}
	return (EOPNOTSUPP);
}

static moduledata_t kms_stub_mod = {
	"kms_stub",
	kms_stub_modevent,
	NULL,
};
DECLARE_MODULE(kms_stub, kms_stub_mod, SI_SUB_DRIVERS,
    SI_ORDER_ANY);
MODULE_VERSION(kms_stub, 1);
MODULE_DEPEND(kms_stub, kms, 1, 1, 1);
