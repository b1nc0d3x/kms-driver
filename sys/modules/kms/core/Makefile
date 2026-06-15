.PATH: ${SRCTOP}/sys/dev/kms/core
.PATH: ${SRCTOP}/sys/dev/kms/display
.PATH: ${SRCTOP}/sys/dev/kms/gem

KMOD=	kms

SRCS=	drm_atomic.c \
	drm_connector.c \
	drm_crtc.c \
	drm_dp_helper.c \
	drm_drv.c \
	drm_edid.c \
	drm_encoder.c \
	drm_events.c \
	drm_file.c \
	drm_framebuffer.c \
	drm_gem.c \
	drm_gem_dumb.c \
	drm_prime.c \
	drm_ioctl.c \
	drm_mode_config.c \
	drm_mode_ioctl.c \
	drm_mode_object.c \
	drm_modes.c \
	drm_modeset.c \
	drm_plane.c \
	drm_property.c \
	vnode_if.h

CFLAGS+=	-I${SRCTOP}/sys/dev/kms/include
CFLAGS+=	-I${SRCTOP}/sys/dev/kms/uapi
CFLAGS+=	-I${SRCTOP}/sys/dev/kms/core

EXPORT_SYMS=	M_KMS \
		kms_display_mode_to_modeinfo \
		kms_gem_handle_create \
		kms_gem_handle_delete \
		kms_gem_handle_lookup \
		kms_gem_object_create \
		kms_gem_object_get \
		kms_gem_object_lookup_offset \
		kms_gem_object_put \
		kms_gem_release_all \
		kms_dp_aux_init \
		kms_dp_aux_transfer \
		kms_dp_dpcd_read \
		kms_dp_dpcd_write \
		kms_edid_add_modes \
		kms_edid_checksum \
		kms_edid_parse \
		kms_mode_create \
		kms_mode_destroy \
		kms_mode_set_name \
		kms_mode_vrefresh \
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
		kms_plane_init \
		kms_send_event \
		kms_send_vblank_event \
		kms_vblank_handler

.include <bsd.kmod.mk>
