# Top-level standalone build for kms.
#
# Targets:
#   make              build kms.ko + kms_stub.ko for host
#   make probe        build the userspace drm_probe smoke test
#   make all          both of the above
#   make install      install built modules + tools (uses DESTDIR)
#   make clean        remove build artifacts
#
# Cross-build for arm64 from amd64 host:
#   make TARGET=arm64 TARGET_ARCH=aarch64
# (Requires a populated /usr/obj/usr/src/arm64.aarch64/tmp cross toolchain
#  from a prior `make buildkernel TARGET=arm64 TARGET_ARCH=aarch64` in
#  /usr/src — kms does not ship its own cross toolchain.)
#
# Override SYSDIR to point at a non-standard kernel source tree:
#   make SYSDIR=/path/to/freebsd/sys

SYSDIR?=	/usr/src/sys
SRCTOP:=	${.CURDIR}

.PHONY: all modules probe clean install

all: modules probe

modules:
	${MAKE} -C ${.CURDIR}/sys/modules/kms \
	    SRCTOP=${SRCTOP} SYSDIR=${SYSDIR}

probe:
	${MAKE} -C ${.CURDIR}/sys/dev/kms/tools \
	    SRCTOP=${SRCTOP}

clean:
	${MAKE} -C ${.CURDIR}/sys/modules/kms clean \
	    SRCTOP=${SRCTOP} SYSDIR=${SYSDIR} || true
	${MAKE} -C ${.CURDIR}/sys/dev/kms/tools clean \
	    SRCTOP=${SRCTOP} || true

install:
	${MAKE} -C ${.CURDIR}/sys/modules/kms install \
	    SRCTOP=${SRCTOP} SYSDIR=${SYSDIR}
