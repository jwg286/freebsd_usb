# $FreeBSD: src/sys/modules/x86bios/Makefile,v 1.3 2009/10/19 20:58:10 jkim Exp $

KMOD=	x86bios
SRCS=	opt_x86bios.h
SRCS+=	bus_if.h device_if.h pci_if.h

.PATH:	${.CURDIR}/../../contrib/x86emu
.PATH:	${.CURDIR}/../../compat/x86bios
SRCS+=	x86bios.c x86emu.c

.include <bsd.kmod.mk>
