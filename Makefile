# Top-level standalone build for the kms driver framework + drivers.
#
# Targets:
#   make              build kms.ko + rk_kms.ko  (rk_kms needs the
#                     FUSB302 USB-C PD controller driver installed at
#                     dev/iicbus/usb/fusb302_var.h — NOT in stock
#                     FreeBSD src.  Framework-only builds use kms-only)
#   make kms-only     build just kms.ko (framework only, no HW deps)
#   make install      install built modules (uses DESTDIR)
#   make clean        remove build artifacts
#
# Cross-build for arm64 from an amd64 host:
#   env MACHINE=arm64 MACHINE_ARCH=aarch64 \
#       CC="cc --target=aarch64-unknown-freebsd15.0" \
#       make -B
#
# Override SYSDIR to point at a non-standard kernel source tree:
#   make SYSDIR=/path/to/freebsd/sys

SYSDIR?=	/usr/src/sys
SRCTOP:=	${.CURDIR}

.PHONY: all kms-only modules clean install

all: modules

kms-only:
	${MAKE} -C ${.CURDIR}/sys/modules/kms/core \
	    SRCTOP=${SRCTOP} SYSDIR=${SYSDIR}

modules: kms-only
	${MAKE} -C ${.CURDIR}/sys/modules/rk_kms \
	    SRCTOP=${SRCTOP} SYSDIR=${SYSDIR}

clean:
	${MAKE} -C ${.CURDIR}/sys/modules/kms/core clean \
	    SRCTOP=${SRCTOP} SYSDIR=${SYSDIR} || true
	${MAKE} -C ${.CURDIR}/sys/modules/rk_kms clean \
	    SRCTOP=${SRCTOP} SYSDIR=${SYSDIR} || true

install:
	${MAKE} -C ${.CURDIR}/sys/modules/kms/core install \
	    SRCTOP=${SRCTOP} SYSDIR=${SYSDIR} DESTDIR=${DESTDIR}
	${MAKE} -C ${.CURDIR}/sys/modules/rk_kms install \
	    SRCTOP=${SRCTOP} SYSDIR=${SYSDIR} DESTDIR=${DESTDIR}
