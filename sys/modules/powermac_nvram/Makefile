# $FreeBSD: src/sys/modules/powermac_nvram/Makefile,v 1.3 2008/11/29 08:45:20 bz Exp $

.PATH: ${.CURDIR}/../../dev/powermac_nvram

KMOD=	powermac_nvram
SRCS=	powermac_nvram.c powermac_nvramvar.h
SRCS+=	bus_if.h device_if.h
SRCS+=	ofw_bus_if.h

.include <bsd.kmod.mk>
