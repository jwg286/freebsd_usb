# $FreeBSD: src/sys/modules/mqueue/Makefile,v 1.3 2010/06/22 20:52:35 thompsa Exp $

.PATH: ${.CURDIR}/../../kern

KMOD=	mqueuefs
SRCS=	uipc_mqueue.c \
	vnode_if.h \
	opt_posix.h opt_compat.h

.include <bsd.kmod.mk>
