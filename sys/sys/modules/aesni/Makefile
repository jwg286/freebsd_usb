# $FreeBSD: src/sys/modules/aesni/Makefile,v 1.1 2010/07/23 11:00:46 kib Exp $

.PATH: ${.CURDIR}/../../crypto/aesni

KMOD=	aesni
SRCS=	aesni.c aesni_wrap.c
SRCS+=	aesencdec_$(MACHINE_CPUARCH).S aeskeys_$(MACHINE_CPUARCH).S
SRCS+=	device_if.h bus_if.h opt_bus.h cryptodev_if.h

.include <bsd.kmod.mk>
