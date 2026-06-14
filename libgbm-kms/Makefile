.PATH: ${.CURDIR}

LIB=		gbm-kms
SHLIB_MAJOR=	1
SRCS=		gbm.c
INCS=		gbm.h
INCSDIR=	/usr/local/include

CFLAGS+=	-I${.CURDIR}/include
CFLAGS+=	-I${.CURDIR}/../sys/dev/kms/uapi
CFLAGS+=	-fPIC -Wall -Wextra

LIBDIR=		/usr/local/lib

.include <bsd.lib.mk>
