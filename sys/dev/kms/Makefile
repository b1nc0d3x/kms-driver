# $FreeBSD$

.PATH:	${.CURDIR}/core

KMOD=	kms

SRCS=	drm_drv.c \
	drm_file.c \
	drm_ioctl.c

CFLAGS+= -I${.CURDIR}/include
CFLAGS+= -I${.CURDIR}/uapi

EXPORT_SYMS= kms_dev_register \
	     kms_dev_unregister

.include <bsd.kmod.mk>
