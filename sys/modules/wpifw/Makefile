# $FreeBSD: src/sys/modules/wpifw/Makefile,v 1.2 2008/10/07 19:42:56 sam Exp $

.PATH: ${.CURDIR}/../../contrib/dev/wpi

KMOD=	wpifw
FIRMWS=	iwlwifi-3945-2.14.4.fw:wpifw:2144

CLEANFILES=	iwlwifi-3945-2.14.4.fw

iwlwifi-3945-2.14.4.fw: ${.CURDIR}/../../contrib/dev/wpi/iwlwifi-3945-2.14.4.fw.uu
	uudecode -p ${.CURDIR}/../../contrib/dev/wpi/iwlwifi-3945-2.14.4.fw.uu > ${.TARGET}

.include <bsd.kmod.mk>
