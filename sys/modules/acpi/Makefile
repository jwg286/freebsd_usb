# $FreeBSD: src/sys/modules/acpi/Makefile,v 1.56 2010/07/25 17:55:57 rpaulo Exp $

.if ${MACHINE} == "i386"
SUBDIR=		acpi
.endif

SUBDIR+=	acpi_asus acpi_fujitsu acpi_hp acpi_ibm	\
		acpi_panasonic acpi_sony acpi_toshiba acpi_video	\
		acpi_dock acpi_wmi aibs

.include <bsd.subdir.mk>
