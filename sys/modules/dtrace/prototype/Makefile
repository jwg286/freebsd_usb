# $FreeBSD: src/sys/modules/dtrace/prototype/Makefile,v 1.1 2008/05/17 02:31:18 jb Exp $

.PATH: ${.CURDIR}/../../../cddl/dev

KMOD=		prototype
SRCS=		prototype.c
SRCS+=		vnode_if.h

CFLAGS+=	-I${.CURDIR}/../../../cddl/compat/opensolaris \
		-I${.CURDIR}/../../../cddl/contrib/opensolaris/uts/common \
		-I${.CURDIR}/../../..

.include <bsd.kmod.mk>
