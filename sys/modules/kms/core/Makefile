.PATH: ${SRCTOP}/sys/dev/kms/core

KMOD=	kms

SRCS=	drm_connector.c \
	drm_crtc.c \
	drm_drv.c \
	drm_encoder.c \
	drm_file.c \
	drm_framebuffer.c \
	drm_ioctl.c \
	drm_mode_config.c \
	drm_mode_ioctl.c \
	drm_mode_object.c \
	drm_plane.c

CFLAGS+=	-I${SRCTOP}/sys/dev/kms/include
CFLAGS+=	-I${SRCTOP}/sys/dev/kms/uapi

EXPORT_SYMS=	M_KMS \
		kms_connector_attach_encoder \
		kms_connector_cleanup \
		kms_connector_init \
		kms_crtc_cleanup \
		kms_crtc_init \
		kms_dev_register \
		kms_dev_unregister \
		kms_encoder_cleanup \
		kms_encoder_init \
		kms_framebuffer_cleanup \
		kms_framebuffer_init \
		kms_mode_object_find \
		kms_mode_object_put \
		kms_mode_object_register \
		kms_mode_object_unregister \
		kms_plane_cleanup \
		kms_plane_init

.include <bsd.kmod.mk>
