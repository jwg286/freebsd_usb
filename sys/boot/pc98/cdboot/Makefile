# $FreeBSD: src/sys/boot/pc98/cdboot/Makefile,v 1.2 2010/06/03 17:42:32 ed Exp $

PROG=	cdboot
STRIP=
BINMODE=${NOBINMODE}
NO_MAN=
SRCS=	${PROG}.s

ORG=	0x0000

LDFLAGS=-e start -Ttext ${ORG} -Wl,-N,-S,--oformat,binary

.include <bsd.prog.mk>
