# $FreeBSD: src/sys/modules/sound/driver/Makefile,v 1.28 2010/07/13 12:47:31 nwhitehorn Exp $

SUBDIR=		ad1816 als4000 atiixp cs4281 csa ds1 emu10k1 emu10kx
SUBDIR+=	envy24 envy24ht es137x ess fm801 hda ich maestro maestro3
SUBDIR+=	neomagic sb16 sb8 sbc solo spicds t4dwave via8233
SUBDIR+=	via82c686 vibes driver uaudio

.if ${MACHINE_ARCH} == "i386" || ${MACHINE_ARCH} == "amd64"
SUBDIR+=	cmi mss
.endif

.if ${MACHINE_ARCH} == "sparc64"
.if ${MACHINE} == "sparc64"
SUBDIR+=	audiocs
.endif
.endif

.if ${MACHINE_CPUARCH} == "powerpc"
SUBDIR+=	ai2s davbus
.endif

.include <bsd.subdir.mk>
