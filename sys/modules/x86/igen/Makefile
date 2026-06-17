.PATH: ${SRCTOP}/sys/dev/kms/drivers/x86/igen

KMOD=	igen

SRCS=	igen.c \
	igen_dpll.c \
	igen_gmbus.c \
	igen_gt.c \
	igen_gtt.c \
	igen_hpd.c \
	igen_i915_uapi.c \
	igen_phy.c \
	bus_if.h device_if.h pci_if.h \
	opt_platform.h

CFLAGS+=	-I${SRCTOP}/sys/dev/kms/include
CFLAGS+=	-I${SRCTOP}/sys/dev/kms/uapi

.include <bsd.kmod.mk>
