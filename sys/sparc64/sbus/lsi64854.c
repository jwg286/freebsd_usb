/*-
 * Copyright (c) 2004 Scott Long
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/*	$NetBSD: lsi64854.c,v 1.25 2005/02/27 00:27:02 perry Exp $ */

/*-
 * Copyright (c) 1998 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Paul Kranenburg.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *        This product includes software developed by the NetBSD
 *        Foundation, Inc. and its contributors.
 * 4. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/sparc64/sbus/lsi64854.c,v 1.9 2008/09/08 20:20:44 marius Exp $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <cam/cam.h>
#include <cam/cam_ccb.h>
#include <cam/scsi/scsi_all.h>

#include <sparc64/sbus/lsi64854reg.h>
#include <sparc64/sbus/lsi64854var.h>

#include <dev/esp/ncr53c9xreg.h>
#include <dev/esp/ncr53c9xvar.h>

#ifdef DEBUG
#define LDB_SCSI	1
#define LDB_ENET	2
#define LDB_PP		4
#define LDB_ANY		0xff
int lsi64854debug = 0;
#define DPRINTF(a,x) do { if (lsi64854debug & (a)) printf x ; } while (0)
#else
#define DPRINTF(a,x)
#endif

#define MAX_DMA_SZ	(16*1024*1024)

static void	lsi64854_reset(struct lsi64854_softc *);
static void	lsi64854_map_scsi(void *, bus_dma_segment_t *, int, int);
static int	lsi64854_setup(struct lsi64854_softc *, caddr_t *, size_t *,
		    int, size_t *);
static int	lsi64854_scsi_intr(void *);
static int	lsi64854_enet_intr(void *);
static int	lsi64854_setup_pp(struct lsi64854_softc *, caddr_t *, size_t *,
		    int, size_t *);
static int	lsi64854_pp_intr(void *);

/*
 * Finish attaching this DMA device.
 * Front-end must fill in these fields:
 *	sc_res
 *	sc_burst
 *	sc_channel (one of SCSI, ENET, PP)
 *	sc_client (one of SCSI, ENET, PP `soft_c' pointers)
 */
int
lsi64854_attach(struct lsi64854_softc *sc)
{
	bus_dma_lock_t *lockfunc;
	struct ncr53c9x_softc *nsc;
	void *lockfuncarg;
	uint32_t csr;
	int error;

	lockfunc = NULL;
	lockfuncarg = NULL;

	switch (sc->sc_channel) {
	case L64854_CHANNEL_SCSI:
		nsc = sc->sc_client;
		if (NCR_LOCK_INITIALIZED(nsc) == 0) {
			device_printf(sc->sc_dev, "mutex not initialized\n");
			return (ENXIO);
		}
		lockfunc = busdma_lock_mutex;
		lockfuncarg = &nsc->sc_lock;
		sc->intr = lsi64854_scsi_intr;
		sc->setup = lsi64854_setup;
		break;
	case L64854_CHANNEL_ENET:
		sc->intr = lsi64854_enet_intr;
		break;
	case L64854_CHANNEL_PP:
		sc->intr = lsi64854_pp_intr;
		sc->setup = lsi64854_setup_pp;
		break;
	default:
		device_printf(sc->sc_dev, "unknown channel\n");
	}
	sc->reset = lsi64854_reset;

	if (sc->setup != NULL) {
		error = bus_dma_tag_create(
		    sc->sc_parent_dmat,		/* parent */
		    1, 0,			/* alignment, boundary */
		    BUS_SPACE_MAXADDR,		/* lowaddr */
		    BUS_SPACE_MAXADDR,		/* highaddr */
		    NULL, NULL,			/* filter, filterarg */
		    MAX_DMA_SZ,			/* maxsize */
		    1,				/* nsegments */
		    MAX_DMA_SZ,			/* maxsegsize */
		    BUS_DMA_ALLOCNOW,		/* flags */
		    lockfunc, lockfuncarg,	/* lockfunc, lockfuncarg */
		    &sc->sc_buffer_dmat);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "cannot allocate buffer DMA tag\n");
			return (error);
		}

		error = bus_dmamap_create(sc->sc_buffer_dmat, 0,
		    &sc->sc_dmamap);
		if (error != 0) {
			device_printf(sc->sc_dev, "DMA map create failed\n");
			bus_dma_tag_destroy(sc->sc_buffer_dmat);
			return (error);
		}
	}

	csr = L64854_GCSR(sc);
	sc->sc_rev = csr & L64854_DEVID;
	if (sc->sc_rev == DMAREV_HME)
		return (0);
	device_printf(sc->sc_dev, "DMA rev. ");
	switch (sc->sc_rev) {
	case DMAREV_0:
		printf("0");
		break;
	case DMAREV_ESC:
		printf("ESC");
		break;
	case DMAREV_1:
		printf("1");
		break;
	case DMAREV_PLUS:
		printf("1+");
		break;
	case DMAREV_2:
		printf("2");
		break;
	default:
		printf("unknown (0x%x)", sc->sc_rev);
	}

	DPRINTF(LDB_ANY, (", burst 0x%x, csr 0x%x", sc->sc_burst, csr));
	printf("\n");

	return (0);
}

int
lsi64854_detach(struct lsi64854_softc *sc)
{

	if (sc->setup != NULL) {
		bus_dmamap_sync(sc->sc_buffer_dmat, sc->sc_dmamap,
		    (L64854_GCSR(sc) & L64854_WRITE) != 0 ?
		    BUS_DMASYNC_PREREAD : BUS_DMASYNC_PREWRITE);
		bus_dmamap_unload(sc->sc_buffer_dmat, sc->sc_dmamap);
		bus_dmamap_destroy(sc->sc_buffer_dmat, sc->sc_dmamap);
		bus_dma_tag_destroy(sc->sc_buffer_dmat);
	}

	return (0);
}

/*
 * DMAWAIT waits while condition is true.
 */
#define DMAWAIT(SC, COND, MSG, DONTPANIC) do if (COND) {		\
	int count = 500000;						\
	while ((COND) && --count > 0) DELAY(1);				\
	if (count == 0) {						\
		printf("%s: line %d: CSR = 0x%lx\n", __FILE__, __LINE__, \
			(u_long)L64854_GCSR(SC));			\
		if (DONTPANIC)						\
			printf(MSG);					\
		else							\
			panic(MSG);					\
	}								\
} while (0)

#define DMA_DRAIN(sc, dontpanic) do {					\
	uint32_t csr;							\
	/*								\
	 * DMA rev0 & rev1: we are not allowed to touch the DMA "flush"	\
	 *     and "drain" bits while it is still thinking about a	\
	 *     request.							\
	 * other revs: D_ESC_R_PEND bit reads as 0			\
	 */								\
	DMAWAIT(sc, L64854_GCSR(sc) & D_ESC_R_PEND, "R_PEND", dontpanic);\
	if (sc->sc_rev != DMAREV_HME) {                                 \
	        /*							\
	         * Select drain bit based on revision			\
	         * also clears errors and D_TC flag			\
	         */							\
	        csr = L64854_GCSR(sc);					\
	        if (sc->sc_rev == DMAREV_1 || sc->sc_rev == DMAREV_0)	\
		        csr |= D_ESC_DRAIN;				\
	        else							\
		        csr |= L64854_INVALIDATE;			\
									\
	        L64854_SCSR(sc,csr);					\
	}								\
	/*								\
	 * Wait for draining to finish					\
	 * rev0 & rev1 call this PACKCNT				\
	 */								\
	DMAWAIT(sc, L64854_GCSR(sc) & L64854_DRAINING, "DRAINING", dontpanic);\
} while(0)

#define DMA_FLUSH(sc, dontpanic) do {					\
	uint32_t csr;							\
	/*								\
	 * DMA rev0 & rev1: we are not allowed to touch the DMA "flush"	\
	 *     and "drain" bits while it is still thinking about a	\
	 *     request.							\
	 * other revs: D_ESC_R_PEND bit reads as 0			\
	 */								\
	DMAWAIT(sc, L64854_GCSR(sc) & D_ESC_R_PEND, "R_PEND", dontpanic);\
	csr = L64854_GCSR(sc);					\
	csr &= ~(L64854_WRITE|L64854_EN_DMA); /* no-ops on ENET */	\
	csr |= L64854_INVALIDATE;	 	/* XXX FAS ? */		\
	L64854_SCSR(sc,csr);						\
} while(0)

static void
lsi64854_reset(struct lsi64854_softc *sc)
{
	uint32_t csr;

	DMA_FLUSH(sc, 1);
	csr = L64854_GCSR(sc);

	DPRINTF(LDB_ANY, ("%s: csr 0x%x\n", __func__, csr));

	if (sc->sc_dmasize != 0) {
		bus_dmamap_sync(sc->sc_buffer_dmat, sc->sc_dmamap,
		    (csr & D_WRITE) != 0 ? BUS_DMASYNC_PREREAD :
		    BUS_DMASYNC_PREWRITE);
		bus_dmamap_unload(sc->sc_buffer_dmat, sc->sc_dmamap);
	}

	if (sc->sc_rev == DMAREV_HME)
		L64854_SCSR(sc, csr | D_HW_RESET_FAS366);

	csr |= L64854_RESET;		/* reset DMA */
	L64854_SCSR(sc, csr);
	DELAY(200);			/* > 10 Sbus clocks(?) */

	/*DMAWAIT1(sc); why was this here? */
	csr = L64854_GCSR(sc);
	csr &= ~L64854_RESET;		/* de-assert reset line */
	L64854_SCSR(sc, csr);
	DELAY(5);			/* allow a few ticks to settle */

	csr = L64854_GCSR(sc);
	csr |= L64854_INT_EN;		/* enable interrupts */
	if (sc->sc_rev > DMAREV_1 && sc->sc_channel == L64854_CHANNEL_SCSI) {
		if (sc->sc_rev == DMAREV_HME)
			csr |= D_TWO_CYCLE;
		else
			csr |= D_FASTER;
	}

	/* Set burst */
	switch (sc->sc_rev) {
	case DMAREV_HME:
	case DMAREV_2:
		csr &= ~L64854_BURST_SIZE;
		if (sc->sc_burst == 32)
			csr |= L64854_BURST_32;
		else if (sc->sc_burst == 16)
			csr |= L64854_BURST_16;
		else
			csr |= L64854_BURST_0;
		break;
	case DMAREV_ESC:
		csr |= D_ESC_AUTODRAIN;	/* Auto-drain */
		if (sc->sc_burst == 32)
			csr &= ~D_ESC_BURST;
		else
			csr |= D_ESC_BURST;
		break;
	default:
		break;
	}
	L64854_SCSR(sc, csr);

	if (sc->sc_rev == DMAREV_HME) {
		bus_write_4(sc->sc_res, L64854_REG_ADDR, 0);
		sc->sc_dmactl = csr;
	}
	sc->sc_active = 0;

	DPRINTF(LDB_ANY, ("%s: done, csr 0x%x\n", __func__, csr));
}

static void
lsi64854_map_scsi(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	struct lsi64854_softc *sc;

	sc = (struct lsi64854_softc *)arg;

	if (nseg != 1)
		panic("%s: cannot map %d segments\n", __func__, nseg);

	bus_dmamap_sync(sc->sc_buffer_dmat, sc->sc_dmamap,
	    sc->sc_datain ? BUS_DMASYNC_PREREAD : BUS_DMASYNC_PREWRITE);
	bus_write_4(sc->sc_res, L64854_REG_ADDR, segs[0].ds_addr);
}

#define	DMAMAX(a)	(MAX_DMA_SZ - ((a) & (MAX_DMA_SZ - 1)))
/*
 * setup a DMA transfer
 */
static int
lsi64854_setup(struct lsi64854_softc *sc, caddr_t *addr, size_t *len,
    int datain, size_t *dmasize)
{
	long bcnt;
	uint32_t csr;

	DMA_FLUSH(sc, 0);

#if 0
	DMACSR(sc) &= ~D_INT_EN;
#endif
	sc->sc_dmaaddr = addr;
	sc->sc_dmalen = len;
	sc->sc_datain = datain;

	/*
	 * The rules say we cannot transfer more than the limit
	 * of this DMA chip (64k for old and 16Mb for new),
	 * and we cannot cross a 16Mb boundary.
	 */
	*dmasize = sc->sc_dmasize =
	    ulmin(*dmasize, DMAMAX((size_t)*sc->sc_dmaaddr));

	DPRINTF(LDB_ANY, ("%s: dmasize=%ld\n", __func__, (long)sc->sc_dmasize));

	/*
	 * XXX what length?
	 */
	if (sc->sc_rev == DMAREV_HME) {
		L64854_SCSR(sc, sc->sc_dmactl | L64854_RESET);
		L64854_SCSR(sc, sc->sc_dmactl);

		bus_write_4(sc->sc_res, L64854_REG_CNT, *dmasize);
	}

	/* Program the DMA address */
	if (sc->sc_dmasize != 0)
		if (bus_dmamap_load(sc->sc_buffer_dmat, sc->sc_dmamap,
		    *sc->sc_dmaaddr, sc->sc_dmasize, lsi64854_map_scsi, sc, 0))
			panic("%s: cannot allocate DVMA address", __func__);

	if (sc->sc_rev == DMAREV_ESC) {
		/* DMA ESC chip bug work-around */
		bcnt = sc->sc_dmasize;
		if (((bcnt + (long)*sc->sc_dmaaddr) & PAGE_MASK_8K) != 0)
			bcnt = roundup(bcnt, PAGE_SIZE_8K);
		bus_write_4(sc->sc_res, L64854_REG_CNT, bcnt);
	}

	/* Setup DMA control register */
	csr = L64854_GCSR(sc);

	if (datain)
		csr |= L64854_WRITE;
	else
		csr &= ~L64854_WRITE;
	csr |= L64854_INT_EN;

	if (sc->sc_rev == DMAREV_HME)
		csr |= (D_DSBL_SCSI_DRN | D_EN_DMA);

	L64854_SCSR(sc, csr);

	return (0);
}

/*
 * Pseudo (chained) interrupt from the esp driver to kick the
 * current running DMA transfer. Called from ncr53c9x_intr()
 * for now.
 *
 * return 1 if it was a DMA continue.
 */
static int
lsi64854_scsi_intr(void *arg)
{
	struct lsi64854_softc *sc = arg;
	struct ncr53c9x_softc *nsc = sc->sc_client;
	int trans, resid;
	uint32_t csr;

	csr = L64854_GCSR(sc);

	DPRINTF(LDB_SCSI, ("%s: addr 0x%x, csr %b\n", __func__,
	     bus_read_4(sc->sc_res, L64854_REG_ADDR), csr, DDMACSR_BITS));

	if (csr & (D_ERR_PEND|D_SLAVE_ERR)) {
		device_printf(sc->sc_dev, "error: csr=%b\n", csr, DDMACSR_BITS);
		csr &= ~D_EN_DMA;	/* Stop DMA */
		/* Invalidate the queue; SLAVE_ERR bit is write-to-clear */
		csr |= D_INVALIDATE|D_SLAVE_ERR;
		L64854_SCSR(sc, csr);
		return (-1);
	}

	/* This is an "assertion" :) */
	if (sc->sc_active == 0)
		panic("%s: DMA wasn't active", __func__);

	DMA_DRAIN(sc, 0);

	/* DMA has stopped */
	csr &= ~D_EN_DMA;
	L64854_SCSR(sc, csr);
	sc->sc_active = 0;

	if (sc->sc_dmasize == 0) {
		/* A "Transfer Pad" operation completed */
		DPRINTF(LDB_SCSI, ("%s: discarded %d bytes (tcl=%d, tcm=%d)\n",
		    __func__, NCR_READ_REG(nsc, NCR_TCL) |
		    (NCR_READ_REG(nsc, NCR_TCM) << 8),
		    NCR_READ_REG(nsc, NCR_TCL), NCR_READ_REG(nsc, NCR_TCM)));
		return (0);
	}

	resid = 0;
	/*
	 * If a transfer onto the SCSI bus gets interrupted by the device
	 * (e.g. for a SAVEPOINTER message), the data in the FIFO counts
	 * as residual since the NCR53C9X counter registers get decremented
	 * as bytes are clocked into the FIFO.
	 */
	if (!(csr & D_WRITE) &&
	    (resid = (NCR_READ_REG(nsc, NCR_FFLAG) & NCRFIFO_FF)) != 0) {
		DPRINTF(LDB_SCSI, ("%s: empty esp FIFO of %d ", __func__,
		    resid));
		if (nsc->sc_rev == NCR_VARIANT_FAS366 &&
		    (NCR_READ_REG(nsc, NCR_CFG3) & NCRFASCFG3_EWIDE))
			resid <<= 1;
	}

	if ((nsc->sc_espstat & NCRSTAT_TC) == 0) {
		/*
		 * `Terminal count' is off, so read the residue
		 * out of the NCR53C9X counter registers.
		 */
		resid += (NCR_READ_REG(nsc, NCR_TCL) |
		    (NCR_READ_REG(nsc, NCR_TCM) << 8) |
		    ((nsc->sc_cfg2 & NCRCFG2_FE) ?
		    (NCR_READ_REG(nsc, NCR_TCH) << 16) : 0));

		if (resid == 0 && sc->sc_dmasize == 65536 &&
		    (nsc->sc_cfg2 & NCRCFG2_FE) == 0)
			/* A transfer of 64K is encoded as `TCL=TCM=0' */
			resid = 65536;
	}

	trans = sc->sc_dmasize - resid;
	if (trans < 0) {			/* transfered < 0? */
#if 0
		/*
		 * This situation can happen in perfectly normal operation
		 * if the ESP is reselected while using DMA to select
		 * another target.  As such, don't print the warning.
		 */
		device_printf(sc->sc_dev, "xfer (%d) > req (%d)\n", trans,
		    sc->sc_dmasize);
#endif
		trans = sc->sc_dmasize;
	}

	DPRINTF(LDB_SCSI, ("%s: tcl=%d, tcm=%d, tch=%d; trans=%d, resid=%d\n",
	    __func__, NCR_READ_REG(nsc, NCR_TCL), NCR_READ_REG(nsc, NCR_TCM),
	    (nsc->sc_cfg2 & NCRCFG2_FE) ? NCR_READ_REG(nsc, NCR_TCH) : 0,
	    trans, resid));

	if (sc->sc_dmasize != 0) {
		bus_dmamap_sync(sc->sc_buffer_dmat, sc->sc_dmamap,
		    (csr & D_WRITE) != 0 ? BUS_DMASYNC_POSTREAD :
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->sc_buffer_dmat, sc->sc_dmamap);
	}

	*sc->sc_dmalen -= trans;
	*sc->sc_dmaaddr += trans;

#if 0	/* this is not normal operation just yet */
	if (*sc->sc_dmalen == 0 || nsc->sc_phase != nsc->sc_prevphase)
		return (0);

	/* and again */
	dma_start(sc, sc->sc_dmaaddr, sc->sc_dmalen, DMACSR(sc) & D_WRITE);
	return (1);
#endif
	return (0);
}

/*
 * Pseudo (chained) interrupt to le driver to handle DMA errors.
 */
static int
lsi64854_enet_intr(void *arg)
{
	struct lsi64854_softc *sc = arg;
	uint32_t csr;
	int i, rv;

	csr = L64854_GCSR(sc);

	/* If the DMA logic shows an interrupt, claim it */
	rv = ((csr & E_INT_PEND) != 0) ? 1 : 0;

	if (csr & (E_ERR_PEND|E_SLAVE_ERR)) {
		device_printf(sc->sc_dev, "error: csr=%b\n", csr, EDMACSR_BITS);
		csr &= ~L64854_EN_DMA;	/* Stop DMA */
		/* Invalidate the queue; SLAVE_ERR bit is write-to-clear */
		csr |= E_INVALIDATE|E_SLAVE_ERR;
		L64854_SCSR(sc, csr);
		/* Will be drained with the LE_C0_IDON interrupt. */
		sc->sc_dodrain = 1;
		return (-1);
	}

	/* XXX - is this necessary with E_DSBL_WR_INVAL on? */
	if (sc->sc_dodrain) {
		i = 10;
		csr |= E_DRAIN;
		L64854_SCSR(sc, csr);
		while (i-- > 0 && (L64854_GCSR(sc) & E_DRAINING))
			DELAY(1);
		sc->sc_dodrain = 0;
	}

	return (rv);
}

static void
lsi64854_map_pp(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{
	struct lsi64854_softc *sc;

	sc = (struct lsi64854_softc *)arg;

	if (nsegs != 1)
		panic("%s: cannot map %d segments\n", __func__, nsegs);

	bus_dmamap_sync(sc->sc_buffer_dmat, sc->sc_dmamap, sc->sc_datain ?
	    BUS_DMASYNC_PREREAD : BUS_DMASYNC_PREWRITE);
	bus_write_4(sc->sc_res, L64854_REG_ADDR, segs[0].ds_addr);

	bus_write_4(sc->sc_res, L64854_REG_CNT, sc->sc_dmasize);
}

/*
 * setup a DMA transfer
 */
static int
lsi64854_setup_pp(struct lsi64854_softc *sc, caddr_t *addr, size_t *len,
    int datain, size_t *dmasize)
{
	uint32_t csr;

	DMA_FLUSH(sc, 0);

	sc->sc_dmaaddr = addr;
	sc->sc_dmalen = len;
	sc->sc_datain = datain;

	DPRINTF(LDB_PP, ("%s: pp start %ld@%p,%d\n", __func__,
	    (long)*sc->sc_dmalen, *sc->sc_dmaaddr, datain ? 1 : 0));

	/*
	 * the rules say we cannot transfer more than the limit
	 * of this DMA chip (64k for old and 16Mb for new),
	 * and we cannot cross a 16Mb boundary.
	 */
	*dmasize = sc->sc_dmasize =
	    ulmin(*dmasize, DMAMAX((size_t)*sc->sc_dmaaddr));

	DPRINTF(LDB_PP, ("%s: dmasize=%ld\n", __func__, (long)sc->sc_dmasize));

	/* Program the DMA address */
	if (sc->sc_dmasize != 0)
		if (bus_dmamap_load(sc->sc_buffer_dmat, sc->sc_dmamap,
		    *sc->sc_dmaaddr, sc->sc_dmasize, lsi64854_map_pp, sc, 0))
			panic("%s: pp cannot allocate DVMA address", __func__);

	/* Setup DMA control register */
	csr = L64854_GCSR(sc);
	csr &= ~L64854_BURST_SIZE;
	if (sc->sc_burst == 32)
		csr |= L64854_BURST_32;
	else if (sc->sc_burst == 16)
		csr |= L64854_BURST_16;
	else
		csr |= L64854_BURST_0;
	csr |= P_EN_DMA|P_INT_EN|P_EN_CNT;
#if 0
	/* This bit is read-only in PP csr register */
	if (datain)
		csr |= P_WRITE;
	else
		csr &= ~P_WRITE;
#endif
	L64854_SCSR(sc, csr);

	return (0);
}

/*
 * Parallel port DMA interrupt.
 */
static int
lsi64854_pp_intr(void *arg)
{
	struct lsi64854_softc *sc = arg;
	int ret, trans, resid = 0;
	uint32_t csr;

	csr = L64854_GCSR(sc);

	DPRINTF(LDB_PP, ("%s: addr 0x%x, csr %b\n", __func__,
	    bus_read_4(sc->sc_res, L64854_REG_ADDR), csr, PDMACSR_BITS));

	if (csr & (P_ERR_PEND|P_SLAVE_ERR)) {
		resid = bus_read_4(sc->sc_res, L64854_REG_CNT);
		device_printf(sc->sc_dev, "error: resid %d csr=%b\n", resid,
		    csr, PDMACSR_BITS);
		csr &= ~P_EN_DMA;	/* Stop DMA */
		/* Invalidate the queue; SLAVE_ERR bit is write-to-clear */
		csr |= P_INVALIDATE|P_SLAVE_ERR;
		L64854_SCSR(sc, csr);
		return (-1);
	}

	ret = (csr & P_INT_PEND) != 0;

	if (sc->sc_active != 0) {
		DMA_DRAIN(sc, 0);
		resid = bus_read_4(sc->sc_res, L64854_REG_CNT);
	}

	/* DMA has stopped */
	csr &= ~D_EN_DMA;
	L64854_SCSR(sc, csr);
	sc->sc_active = 0;

	trans = sc->sc_dmasize - resid;
	if (trans < 0)				/* transfered < 0? */
		trans = sc->sc_dmasize;
	*sc->sc_dmalen -= trans;
	*sc->sc_dmaaddr += trans;

	if (sc->sc_dmasize != 0) {
		bus_dmamap_sync(sc->sc_buffer_dmat, sc->sc_dmamap,
		    (csr & D_WRITE) != 0 ? BUS_DMASYNC_POSTREAD :
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->sc_buffer_dmat, sc->sc_dmamap);
	}

	return (ret != 0);
}
