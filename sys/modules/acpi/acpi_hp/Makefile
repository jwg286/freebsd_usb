# $FreeBSD: src/sys/modules/acpi/acpi_hp/Makefile,v 1.1 2009/06/23 13:17:25 rpaulo Exp $

.PATH:	${.CURDIR}/../../../dev/acpi_support

KMOD=	acpi_hp
CFLAGS+=-I${.CURDIR}/../../../dev/acpi_support
SRCS=	acpi_hp.c opt_acpi.h acpi_if.h acpi_wmi_if.h device_if.h bus_if.h

.include <bsd.kmod.mk>
