# $FreeBSD: src/sys/modules/siftr/Makefile,v 1.1 2010/07/03 13:32:39 lstewart Exp $

.include <bsd.own.mk>

.PATH:  ${.CURDIR}/../../netinet
KMOD=	siftr
SRCS=	siftr.c

# Uncomment to add IPv6 support
#CFLAGS+=-DSIFTR_IPV6

.include <bsd.kmod.mk>
