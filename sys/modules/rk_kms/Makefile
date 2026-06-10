.PATH: ${SRCTOP}/sys/dev/kms/drivers

KMOD=	rk_kms

SRCS=	rk_kms.c \
	bus_if.h device_if.h ofw_bus_if.h fb_if.h

CFLAGS+=	-I${SRCTOP}/sys/dev/kms/include
CFLAGS+=	-I${SRCTOP}/sys/dev/kms/uapi

.include <bsd.kmod.mk>
