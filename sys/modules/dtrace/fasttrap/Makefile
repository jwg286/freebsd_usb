# $FreeBSD: src/sys/modules/dtrace/fasttrap/Makefile,v 1.1 2008/05/17 02:31:18 jb Exp $

.PATH: ${.CURDIR}/../../../cddl/dev/fasttrap

KMOD=		fasttrap
SRCS=		fasttrap.c
SRCS+=		vnode_if.h

CFLAGS+=	-I${.CURDIR}/../../../cddl/compat/opensolaris \
		-I${.CURDIR}/../../../cddl/contrib/opensolaris/uts/common \
		-I${.CURDIR}/../../..

.if ${MACHINE_ARCH} == "amd64" ||  ${MACHINE_ARCH} == "i386"
CFLAGS+=	-I${.CURDIR}/../../../cddl/contrib/opensolaris/uts/intel
.endif

CFLAGS+=	-DSMP -DDEBUG

.include <bsd.kmod.mk>
