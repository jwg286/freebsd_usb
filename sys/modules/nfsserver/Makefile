# $FreeBSD: src/sys/modules/nfsserver/Makefile,v 1.32 2010/02/16 20:00:21 marius Exp $

.PATH: ${.CURDIR}/../../nfsserver
KMOD=	nfsserver
SRCS=	vnode_if.h \
	nfs_fha.c nfs_serv.c nfs_srvkrpc.c nfs_srvsubs.c \
	opt_mac.h \
	opt_kgssapi.h \
	opt_nfs.h
SRCS+=	opt_inet6.h

.if !defined(KERNBUILDDIR)
NFS_INET6?=	1	# 0/1 - requires INET6 to be configured in kernel

.if ${NFS_INET6} > 0
opt_inet6.h:
	echo "#define INET6 1" > ${.TARGET}
.endif
.endif

.include <bsd.kmod.mk>
