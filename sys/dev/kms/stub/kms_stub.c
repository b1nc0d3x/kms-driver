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

#include <drm/drm_mode.h>
#include <kms/drm_connector.h>
#include <kms/drm_crtc.h>
#include <kms/drm_device.h>
#include <kms/drm_drv.h>
#include <kms/drm_encoder.h>

/*
 * Reuse M_KMS from the framework.  Defining a second malloc
 * type from a stub during kldload has triggered uma zone creation
 * issues; sharing the framework's type avoids that path entirely.
 */
MALLOC_DECLARE(M_KMS);

static const struct drm_driver kms_stub_driver = {
	.name		= "stub",
	.desc		= "KMS phase-4 stub",
	.date		= "20260607",
	.major		= 0,
	.minor		= 4,
	.patchlevel	= 0,
	.driver_features = 0,
};

static struct drm_device	*kms_stub_dev;
static struct drm_crtc		 kms_stub_crtc_storage;
static struct drm_encoder	 kms_stub_encoder_storage;
static struct drm_connector	 kms_stub_connector_storage;
static struct drm_crtc		*kms_stub_crtc;
static struct drm_encoder	*kms_stub_encoder;
static struct drm_connector	*kms_stub_connector;

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

static const struct drm_crtc_funcs stub_crtc_funcs = {
	.destroy = stub_crtc_destroy,
};

static const struct drm_encoder_funcs stub_encoder_funcs = {
	.destroy = stub_encoder_destroy,
};

static const struct drm_connector_funcs stub_connector_funcs = {
	.destroy = stub_connector_destroy,
};

static int
kms_stub_attach(void)
{
	struct drm_crtc *crtc;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	int error;

	memset(&kms_stub_crtc_storage, 0,
	    sizeof(kms_stub_crtc_storage));
	memset(&kms_stub_encoder_storage, 0,
	    sizeof(kms_stub_encoder_storage));
	memset(&kms_stub_connector_storage, 0,
	    sizeof(kms_stub_connector_storage));
	crtc = &kms_stub_crtc_storage;
	encoder = &kms_stub_encoder_storage;
	connector = &kms_stub_connector_storage;

	error = kms_crtc_init(kms_stub_dev, crtc, &stub_crtc_funcs);
	if (error != 0)
		return (error);
	kms_stub_crtc = crtc;

	error = kms_encoder_init(kms_stub_dev, encoder,
	    &stub_encoder_funcs, DRM_MODE_ENCODER_TMDS);
	if (error != 0) {
		kms_crtc_cleanup(crtc);
		kms_stub_crtc = NULL;
		return (error);
	}
	encoder->possible_crtcs = 1u << crtc->index;
	kms_stub_encoder = encoder;

	error = kms_connector_init(kms_stub_dev, connector,
	    &stub_connector_funcs, DRM_MODE_CONNECTOR_HDMIA);
	if (error != 0) {
		kms_encoder_cleanup(encoder);
		kms_stub_encoder = NULL;
		kms_crtc_cleanup(crtc);
		kms_stub_crtc = NULL;
		return (error);
	}
	connector->status = connector_status_disconnected;
	kms_connector_attach_encoder(connector, encoder);
	kms_stub_connector = connector;

	return (0);
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
