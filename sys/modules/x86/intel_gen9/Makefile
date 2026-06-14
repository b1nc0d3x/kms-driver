.PATH: ${SRCTOP}/sys/dev/kms/drivers/x86/intel_gen9

KMOD=	intel_gen9

SRCS=	intel_gen9.c \
	bus_if.h device_if.h pci_if.h \
	opt_platform.h

CFLAGS+=	-I${SRCTOP}/sys/dev/kms/include
CFLAGS+=	-I${SRCTOP}/sys/dev/kms/uapi

.include <bsd.kmod.mk>
