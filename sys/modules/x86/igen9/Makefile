.PATH: ${SRCTOP}/sys/dev/kms/drivers/x86/igen9

KMOD=	igen9

SRCS=	igen9.c \
	igen9_dpll.c \
	bus_if.h device_if.h pci_if.h \
	opt_platform.h

CFLAGS+=	-I${SRCTOP}/sys/dev/kms/include
CFLAGS+=	-I${SRCTOP}/sys/dev/kms/uapi

.include <bsd.kmod.mk>
