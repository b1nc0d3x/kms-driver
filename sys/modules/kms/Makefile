SUBDIR=	core
SUBDIR+=	stub

#
# Hardware-consumer modules.  Each lives in its own subdir under
# sys/modules/<arch>/ so kernel-build conventions match the rest of
# the tree.  Gated on MACHINE_ARCH so an arm64 build doesn't try to
# compile x86 Intel display drivers and vice-versa.
#
.if ${MACHINE_ARCH} == "amd64" || ${MACHINE_ARCH} == "i386"
SUBDIR+=	../x86/igen
.endif

.include <bsd.subdir.mk>
