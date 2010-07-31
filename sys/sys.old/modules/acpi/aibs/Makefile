# $FreeBSD: src/sys/modules/acpi/aibs/Makefile,v 1.1 2010/06/25 15:32:46 rpaulo Exp $

.PATH:	${.CURDIR}/../../../dev/acpi_support

KMOD=	aibs
SRCS=	atk0110.c
SRCS+=	opt_acpi.h acpi_if.h bus_if.h device_if.h
SRCS+=	opt_ddb.h

.include <bsd.kmod.mk>
