# $FreeBSD: src/sys/modules/scc/Makefile,v 1.4 2008/05/04 14:59:25 marius Exp $

.PATH: ${.CURDIR}/../../dev/scc

.if ${MACHINE} == "sparc64"
scc_bfe= scc_bfe_ebus.c scc_bfe_sbus.c
.endif
.if ${MACHINE_ARCH} == "powerpc"
scc_bfe= scc_bfe_macio.c scc_bfe_quicc.c
.endif

KMOD=	scc
SRCS=	${scc_bfe} scc_core.c scc_if.c scc_if.h \
	scc_dev_sab82532.c scc_dev_z8530.c
SRCS+=	bus_if.h device_if.h ofw_bus_if.h serdev_if.c serdev_if.h

MFILES=	dev/ofw/ofw_bus_if.m dev/scc/scc_if.m \
	kern/bus_if.m kern/device_if.m kern/serdev_if.m

.include <bsd.kmod.mk>
