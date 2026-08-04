.PATH: ${SRCTOP}/sys/dev/kms/drivers/virtio

KMOD=	virtio_kms

SRCS=	virtio_kms.c \
	bus_if.h device_if.h \
	fb_if.h virtio_if.h virtio_bus_if.h \
	opt_acpi.h opt_drm.h opt_evdev.h opt_kbd.h opt_platform.h \
	opt_splash.h opt_syscons.h opt_teken.h

CFLAGS+=	-I${SRCTOP}/sys/dev/kms/include
CFLAGS+=	-I${SRCTOP}/sys/dev/kms/uapi

.include <bsd.kmod.mk>
