PORTNAME=	kms
DISTVERSION=	g20260802
CATEGORIES=	graphics kld

MAINTAINER=	b1nc0d3x@gmail.com
COMMENT=	Native FreeBSD KMS driver framework (arm64 / amd64)
WWW=		https://github.com/b1nc0d3x/kms-driver

LICENSE=	BSD2CLAUSE
LICENSE_FILE=	${WRKSRC}/LICENSE

ONLY_FOR_ARCHS=		aarch64 amd64
ONLY_FOR_ARCHS_REASON=	kms currently ships modules for arm64 and amd64 only

USES=		kmod uidfix

USE_GITHUB=	yes
GH_ACCOUNT=	b1nc0d3x
GH_PROJECT=	kms-driver
GH_TAGNAME=	main

# The stock target only builds kms.ko (the framework).  Vendor drivers
# in sys/dev/kms/drivers/ have their own kernel-source dependencies
# (e.g. rk_kms needs FUSB302, igen needs the intel-gen host cscape) and
# are not built here; they can be assembled from the source drop after
# install.
ALL_TARGET=	kms-only

PLIST_FILES=	${KMODDIR}/kms.ko

do-install:
	${MKDIR} ${STAGEDIR}${KMODDIR}
	${INSTALL_KLD} ${WRKSRC}/sys/modules/kms/core/kms.ko \
	    ${STAGEDIR}${KMODDIR}/kms.ko

.include <bsd.port.mk>
