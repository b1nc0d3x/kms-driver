.PATH: ${SRCTOP}/sys/dev/kms/core
.PATH: ${SRCTOP}/sys/dev/kms/display
.PATH: ${SRCTOP}/sys/dev/kms/gem

KMOD=	kms

SRCS=	drm_connector.c \
	drm_crtc.c \
	drm_dp_helper.c \
	drm_drv.c \
	drm_edid.c \
	drm_encoder.c \
	drm_file.c \
	drm_framebuffer.c \
	drm_gem.c \
	drm_gem_dumb.c \
	drm_ioctl.c \
	drm_mode_config.c \
	drm_mode_ioctl.c \
	drm_mode_object.c \
	drm_modes.c \
	drm_modeset.c \
	drm_plane.c

CFLAGS+=	-I${SRCTOP}/sys/dev/kms/include
CFLAGS+=	-I${SRCTOP}/sys/dev/kms/uapi
CFLAGS+=	-I${SRCTOP}/sys/dev/kms/core

EXPORT_SYMS=	M_KMS \
		drm_display_mode_to_modeinfo \
		kms_gem_handle_create \
		kms_gem_handle_delete \
		kms_gem_handle_lookup \
		kms_gem_object_create \
		kms_gem_object_get \
		kms_gem_object_lookup_offset \
		kms_gem_object_put \
		kms_gem_release_all \
		drm_dp_aux_init \
		drm_dp_aux_transfer \
		drm_dp_dpcd_read \
		drm_dp_dpcd_write \
		drm_edid_add_modes \
		drm_edid_checksum \
		drm_edid_parse \
		drm_mode_create \
		drm_mode_destroy \
		drm_mode_set_name \
		drm_mode_vrefresh \
		kms_connector_add_mode \
		kms_connector_attach_encoder \
		kms_connector_cleanup \
		kms_connector_init \
		kms_connector_modes_clear \
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
		kms_mode_object_register_locked \
		kms_mode_object_unregister \
		kms_plane_cleanup \
		kms_plane_init

.include <bsd.kmod.mk>
