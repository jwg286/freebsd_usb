# $FreeBSD: src/sys/modules/linprocfs/Makefile,v 1.17 2009/06/08 19:57:35 bz Exp $

.PATH: ${.CURDIR}/../../compat/linprocfs

KMOD=	linprocfs
SRCS=	vnode_if.h \
	device_if.h bus_if.h \
	linprocfs.c \
	opt_compat.h

.if ${MACHINE_ARCH} == "amd64"
CFLAGS+=-DCOMPAT_LINUX32
.endif

.include <bsd.kmod.mk>
