# $FreeBSD: src/sys/modules/mii/Makefile,v 1.36 2009/03/30 16:01:09 imp Exp $

.PATH: ${.CURDIR}/../../dev/mii

KMOD=	miibus
SRCS=	acphy.c amphy.c atphy.c axphy.c bmtphy.c brgphy.c bus_if.h
SRCS+=	ciphy.c device_if.h
SRCS+=	e1000phy.c exphy.c gentbi.c icsphy.c inphy.c ip1000phy.c jmphy.c
SRCS+=	lxtphy.c miibus_if.c miibus_if.h mii.c miidevs.h mii_physubr.c
SRCS+=	mlphy.c nsgphy.c nsphy.c nsphyter.c pci_if.h pnaphy.c qsphy.c
SRCS+=	rgephy.c rlphy.c ruephy.c tdkphy.c tlphy.c truephy.c ukphy.c
SRCS+=	ukphy_subr.c
SRCS+=	xmphy.c

EXPORT_SYMS=	mii_mediachg	\
		mii_phy_probe	\
		mii_phy_reset	\
		mii_pollstat	\
		mii_tick

.include <bsd.kmod.mk>
