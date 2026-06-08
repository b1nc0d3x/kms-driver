.PATH: ${SRCTOP}/sys/dev/kms/stub

KMOD=	kms_stub

SRCS=	kms_stub.c

CFLAGS+=	-I${SRCTOP}/sys/dev/kms/include
CFLAGS+=	-I${SRCTOP}/sys/dev/kms/uapi

.include <bsd.kmod.mk>
