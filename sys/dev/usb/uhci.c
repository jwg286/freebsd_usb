/*	$NetBSD: uhci.c,v 1.170 2003/02/19 01:35:04 augustss Exp $	*/

/*	Also already incorporated from NetBSD:
 *	$NetBSD: uhci.c,v 1.172 2003/02/23 04:19:26 simonb Exp $
 *	$NetBSD: uhci.c,v 1.173 2003/05/13 04:41:59 gson Exp $
 *	$NetBSD: uhci.c,v 1.175 2003/09/12 16:18:08 mycroft Exp $
 *	$NetBSD: uhci.c,v 1.176 2003/11/04 19:11:21 mycroft Exp $
 *	$NetBSD: uhci.c,v 1.177 2003/12/29 08:17:10 toshii Exp $
 *	$NetBSD: uhci.c,v 1.178 2004/03/02 16:32:05 martin Exp $
 *	$NetBSD: uhci.c,v 1.180 2004/07/17 20:12:03 mycroft Exp $
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: stable/7/sys/dev/usb/uhci.c 196167 2009-08-13 07:21:24Z n_hibma $");


/*-
 * Copyright (c) 1998 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Lennart Augustsson (lennart@augustsson.net) at
 * Carlstedt Research & Technology.
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

/*
 * USB Universal Host Controller driver.
 * Handles e.g. PIIX3 and PIIX4.
 *
 * UHCI spec: http://developer.intel.com/design/USB/UHCI11D.htm
 * USB spec: http://www.usb.org/developers/docs/usbspec.zip
 * PIIXn spec: ftp://download.intel.com/design/intarch/datashts/29055002.pdf
 *             ftp://download.intel.com/design/intarch/datashts/29056201.pdf
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/endian.h>
#include <sys/module.h>
#include <sys/bus.h>
#if defined(DIAGNOSTIC) && defined(__i386__)
#include <machine/cpu.h>
#endif
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/queue.h>
#include <sys/sysctl.h>

#include <machine/bus.h>
#include <machine/endian.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdivar.h>
#include <dev/usb/usb_mem.h>
#include <dev/usb/usb_quirks.h>

#include <dev/usb/uhcireg.h>
#include <dev/usb/uhcivar.h>

/* Use bandwidth reclamation for control transfers. Some devices choke on it. */
/*#define UHCI_CTL_LOOP */

#define delay(d)		DELAY(d)

#define MS_TO_TICKS(ms) ((ms) * hz / 1000)

#ifdef USB_DEBUG
uhci_softc_t *thesc;
#define DPRINTF(x)	if (uhcidebug) printf x
#define DPRINTFN(n,x)	if (uhcidebug>(n)) printf x
int uhcidebug = 0;
int uhcinoloop = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, uhci, CTLFLAG_RW, 0, "USB uhci");
SYSCTL_INT(_hw_usb_uhci, OID_AUTO, debug, CTLFLAG_RW,
	   &uhcidebug, 0, "uhci debug level");
SYSCTL_INT(_hw_usb_uhci, OID_AUTO, loop, CTLFLAG_RW,
	   &uhcinoloop, 0, "uhci noloop");
#define bitmask_snprintf(q,f,b,l) snprintf((b), (l), "%b", (q), (f))
#else
#define DPRINTF(x)
#define DPRINTFN(n,x)
#endif

struct uhci_pipe {
	struct usbd_pipe pipe;
	int nexttoggle;

	u_char aborting;
	usbd_xfer_handle abortstart, abortend;

	/* Info needed for different pipe kinds. */
	union {
		/* Control pipe */
		struct {
			uhci_soft_qh_t *sqh;
			usb_dma_t reqdma;
			uhci_soft_td_t *setup, *stat;
			u_int length;
		} ctl;
		/* Interrupt pipe */
		struct {
			int npoll;
			int isread;
			uhci_soft_qh_t **qhs;
		} intr;
		/* Bulk pipe */
		struct {
			uhci_soft_qh_t *sqh;
			u_int length;
			int isread;
		} bulk;
		/* Iso pipe */
		struct iso {
			uhci_soft_td_t **stds;
			int next, inuse;
		} iso;
	} u;
};

static usbd_status	uhci_run(uhci_softc_t *, int run);

static usbd_status	uhci_allocm(struct usbd_bus *, usb_dma_t *, u_int32_t);
static void		uhci_freem(struct usbd_bus *, usb_dma_t *);

static usbd_xfer_handle	uhci_allocx(struct usbd_bus *);
static void		uhci_freex(struct usbd_bus *, usbd_xfer_handle);

static usbd_status	uhci_open(usbd_pipe_handle);
static void		uhci_poll(struct usbd_bus *);
static void		uhci_softintr(void *);
static uhci_soft_td_t  *uhci_alloc_std(uhci_softc_t *);
static uhci_soft_qh_t  *uhci_alloc_sqh(uhci_softc_t *);
static void		uhci_globalreset(uhci_softc_t *);
static void		uhci_reset(uhci_softc_t *);
#ifdef USB_DEBUG
static void		uhci_dump_all(uhci_softc_t *);
static void		uhci_dumpregs(uhci_softc_t *);
static void		uhci_dump_qh(uhci_soft_qh_t *);
#endif

#define UBARR(sc) bus_space_barrier((sc)->iot, (sc)->ioh, 0, (sc)->sc_size, \
			BUS_SPACE_BARRIER_READ|BUS_SPACE_BARRIER_WRITE)
#define UWRITE1(sc, r, x) \
 do { UBARR(sc); bus_space_write_1((sc)->iot, (sc)->ioh, (r), (x)); \
 } while (/*CONSTCOND*/0)
#define UWRITE2(sc, r, x) \
 do { UBARR(sc); bus_space_write_2((sc)->iot, (sc)->ioh, (r), (x)); \
 } while (/*CONSTCOND*/0)
#define UWRITE4(sc, r, x) \
 do { UBARR(sc); bus_space_write_4((sc)->iot, (sc)->ioh, (r), (x)); \
 } while (/*CONSTCOND*/0)
#define UREAD1(sc, r) (UBARR(sc), bus_space_read_1((sc)->iot, (sc)->ioh, (r)))
#define UREAD2(sc, r) (UBARR(sc), bus_space_read_2((sc)->iot, (sc)->ioh, (r)))
#define UREAD4(sc, r) (UBARR(sc), bus_space_read_4((sc)->iot, (sc)->ioh, (r)))

#define UHCICMD(sc, cmd) UWRITE2(sc, UHCI_CMD, cmd)
#define UHCISTS(sc) UREAD2(sc, UHCI_STS)

#define UHCI_RESET_TIMEOUT 100	/* ms, reset timeout */

#define UHCI_CURFRAME(sc) (UREAD2(sc, UHCI_FRNUM) & UHCI_FRNUM_MASK)

#define UHCI_INTR_ENDPT 1

struct usbd_bus_methods uhci_bus_methods = {
	uhci_open,
	uhci_softintr,
	uhci_poll,
	uhci_allocm,
	uhci_freem,
	uhci_allocx,
	uhci_freex,
};

void
uhci_reset(uhci_softc_t *sc)
{
	int n;

	UHCICMD(sc, UHCI_CMD_HCRESET);
	/* The reset bit goes low when the controller is done. */
	for (n = 0; n < UHCI_RESET_TIMEOUT &&
		    (UREAD2(sc, UHCI_CMD) & UHCI_CMD_HCRESET); n++)
		usb_delay_ms(&sc->sc_bus, 1);
	if (n >= UHCI_RESET_TIMEOUT)
		printf("%s: controller did not reset\n",
		       device_get_nameunit(sc->sc_bus.bdev));
}

void
uhci_globalreset(uhci_softc_t *sc)
{
	UHCICMD(sc, UHCI_CMD_GRESET);	/* global reset */
	usb_delay_ms(&sc->sc_bus, USB_BUS_RESET_DELAY); /* wait a little */
	UHCICMD(sc, 0);			/* do nothing */
}

usbd_status
uhci_init(uhci_softc_t *sc)
{
	usbd_status err;
	int i, j;
	uhci_soft_qh_t *clsqh, *chsqh, *bsqh, *sqh, *lsqh;
	uhci_soft_td_t *std;

	DPRINTFN(1,("uhci_init: start\n"));

#ifdef USB_DEBUG
	thesc = sc;

	if (uhcidebug > 2)
		uhci_dumpregs(sc);
#endif

	UHCI_LOCK_INIT(sc);

	UWRITE2(sc, UHCI_INTR, 0);		/* disable interrupts */
	uhci_globalreset(sc);			/* reset the controller */
	uhci_reset(sc);

	/* Allocate and initialize real frame array. */
	err = usb_allocmem(&sc->sc_bus,
		  UHCI_FRAMELIST_COUNT * sizeof(uhci_physaddr_t),
		  UHCI_FRAMELIST_ALIGN, &sc->sc_dma);
	if (err)
		return (err);
	sc->sc_pframes = KERNADDR(&sc->sc_dma, 0);
	UWRITE2(sc, UHCI_FRNUM, 0);		/* set frame number to 0 */
	UWRITE4(sc, UHCI_FLBASEADDR, DMAADDR(&sc->sc_dma, 0)); /* set frame list*/

	/*
	 * Allocate a TD, inactive, that hangs from the last QH.
	 * This is to avoid a bug in the PIIX that makes it run berserk
	 * otherwise.
	 */
	std = uhci_alloc_std(sc);
	if (std == NULL)
		return (USBD_NOMEM);
	std->link.std = NULL;
	std->td.td_link = htole32(UHCI_PTR_T);
	std->td.td_status = htole32(0); /* inactive */
	std->td.td_token = htole32(0);
	std->td.td_buffer = htole32(0);

	/* Allocate the dummy QH marking the end and used for looping the QHs.*/
	lsqh = uhci_alloc_sqh(sc);
	if (lsqh == NULL)
		return (USBD_NOMEM);
	lsqh->hlink = NULL;
	lsqh->qh.qh_hlink = htole32(UHCI_PTR_T);	/* end of QH chain */
	lsqh->elink = std;
	lsqh->qh.qh_elink = htole32(std->physaddr | UHCI_PTR_TD);
	sc->sc_last_qh = lsqh;

	/* Allocate the dummy QH where bulk traffic will be queued. */
	bsqh = uhci_alloc_sqh(sc);
	if (bsqh == NULL)
		return (USBD_NOMEM);
	bsqh->hlink = lsqh;
	bsqh->qh.qh_hlink = htole32(lsqh->physaddr | UHCI_PTR_QH);
	bsqh->elink = NULL;
	bsqh->qh.qh_elink = htole32(UHCI_PTR_T);
	sc->sc_bulk_start = sc->sc_bulk_end = bsqh;

	/* Allocate dummy QH where high speed control traffic will be queued. */
	chsqh = uhci_alloc_sqh(sc);
	if (chsqh == NULL)
		return (USBD_NOMEM);
	chsqh->hlink = bsqh;
	chsqh->qh.qh_hlink = htole32(bsqh->physaddr | UHCI_PTR_QH);
	chsqh->elink = NULL;
	chsqh->qh.qh_elink = htole32(UHCI_PTR_T);
	sc->sc_hctl_start = sc->sc_hctl_end = chsqh;

	/* Allocate dummy QH where control traffic will be queued. */
	clsqh = uhci_alloc_sqh(sc);
	if (clsqh == NULL)
		return (USBD_NOMEM);
	clsqh->hlink = chsqh;
	clsqh->qh.qh_hlink = htole32(chsqh->physaddr | UHCI_PTR_QH);
	clsqh->elink = NULL;
	clsqh->qh.qh_elink = htole32(UHCI_PTR_T);
	sc->sc_lctl_start = sc->sc_lctl_end = clsqh;

	/*
	 * Make all (virtual) frame list pointers point to the interrupt
	 * queue heads and the interrupt queue heads at the control
	 * queue head and point the physical frame list to the virtual.
	 */
	for(i = 0; i < UHCI_VFRAMELIST_COUNT; i++) {
		std = uhci_alloc_std(sc);
		sqh = uhci_alloc_sqh(sc);
		if (std == NULL || sqh == NULL)
			return (USBD_NOMEM);
		std->link.sqh = sqh;
		std->td.td_link = htole32(sqh->physaddr | UHCI_PTR_QH);
		std->td.td_status = htole32(UHCI_TD_IOS); /* iso, inactive */
		std->td.td_token = htole32(0);
		std->td.td_buffer = htole32(0);
		sqh->hlink = clsqh;
		sqh->qh.qh_hlink = htole32(clsqh->physaddr | UHCI_PTR_QH);
		sqh->elink = NULL;
		sqh->qh.qh_elink = htole32(UHCI_PTR_T);
		sc->sc_vframes[i].htd = std;
		sc->sc_vframes[i].etd = std;
		sc->sc_vframes[i].hqh = sqh;
		sc->sc_vframes[i].eqh = sqh;
		for (j = i;
		     j < UHCI_FRAMELIST_COUNT;
		     j += UHCI_VFRAMELIST_COUNT)
			sc->sc_pframes[j] = htole32(std->physaddr);
	}

	LIST_INIT(&sc->sc_intrhead);

	STAILQ_INIT(&sc->sc_free_xfers);

	callout_init(&sc->sc_poll_handle, 0);

	/* Set up the bus struct. */
	sc->sc_bus.methods = &uhci_bus_methods;
	sc->sc_bus.pipe_size = sizeof(struct uhci_pipe);

#if defined(__NetBSD__) || defined(__OpenBSD__)
	sc->sc_suspend = PWR_RESUME;
	sc->sc_powerhook = powerhook_establish(uhci_power, sc);
	sc->sc_shutdownhook = shutdownhook_establish(uhci_shutdown, sc);
#endif

	DPRINTFN(1,("uhci_init: enabling\n"));
	UWRITE2(sc, UHCI_INTR, UHCI_INTR_TOCRCIE | UHCI_INTR_RIE |
		UHCI_INTR_IOCE | UHCI_INTR_SPIE);	/* enable interrupts */

	UHCICMD(sc, UHCI_CMD_MAXP); /* Assume 64 byte packets at frame end */

	return (uhci_run(sc, 1));		/* and here we go... */
}

usbd_status
uhci_run(uhci_softc_t *sc, int run)
{
	int n, running;
	u_int16_t cmd;

	run = run != 0;
	UHCI_LOCK(sc);
	DPRINTF(("uhci_run: setting run=%d\n", run));
	cmd = UREAD2(sc, UHCI_CMD);
	if (run)
		cmd |= UHCI_CMD_RS;
	else
		cmd &= ~UHCI_CMD_RS;
	UHCICMD(sc, cmd);
	for(n = 0; n < 10; n++) {
		running = !(UREAD2(sc, UHCI_STS) & UHCI_STS_HCH);
		/* return when we've entered the state we want */
		if (run == running) {
			UHCI_UNLOCK(sc);
			DPRINTF(("uhci_run: done cmd=0x%x sts=0x%x\n",
				 UREAD2(sc, UHCI_CMD), UREAD2(sc, UHCI_STS)));
			return (USBD_NORMAL_COMPLETION);
		}
		usb_delay_ms(&sc->sc_bus, 1);
	}
	UHCI_UNLOCK(sc);
	printf("%s: cannot %s\n", device_get_nameunit(sc->sc_bus.bdev),
	       run ? "start" : "stop");
	return (USBD_IOERROR);
}

/*
 * Memory management routines.
 *  uhci_alloc_std allocates TDs
 *  uhci_alloc_sqh allocates QHs
 * These two routines do their own free list management,
 * partly for speed, partly because allocating DMAable memory
 * has page size granularaity so much memory would be wasted if
 * only one TD/QH (32 bytes) was placed in each allocated chunk.
 */

static uhci_soft_td_t *
uhci_alloc_std(uhci_softc_t *sc)
{
	uhci_soft_td_t *std;
	usbd_status err;
	int i, offs;
	usb_dma_t dma;

	if (sc->sc_freetds == NULL) {
		DPRINTFN(2,("uhci_alloc_std: allocating chunk\n"));
		err = usb_allocmem(&sc->sc_bus, UHCI_STD_SIZE * UHCI_STD_CHUNK,
			  UHCI_TD_ALIGN, &dma);
		if (err)
			return (0);
		for(i = 0; i < UHCI_STD_CHUNK; i++) {
			offs = i * UHCI_STD_SIZE;
			std = KERNADDR(&dma, offs);
			std->physaddr = DMAADDR(&dma, offs);
			std->link.std = sc->sc_freetds;
			std->aux_dma.block = NULL;
			std->aux_data = NULL;
			std->aux_len = 0;
			sc->sc_freetds = std;
		}
	}
	std = sc->sc_freetds;
	sc->sc_freetds = std->link.std;
	memset(&std->td, 0, sizeof(uhci_td_t));
	return std;
}

uhci_soft_qh_t *
uhci_alloc_sqh(uhci_softc_t *sc)
{
	uhci_soft_qh_t *sqh;
	usbd_status err;
	int i, offs;
	usb_dma_t dma;

	if (sc->sc_freeqhs == NULL) {
		DPRINTFN(2, ("uhci_alloc_sqh: allocating chunk\n"));
		err = usb_allocmem(&sc->sc_bus, UHCI_SQH_SIZE * UHCI_SQH_CHUNK,
			  UHCI_QH_ALIGN, &dma);
		if (err)
			return (0);
		for(i = 0; i < UHCI_SQH_CHUNK; i++) {
			offs = i * UHCI_SQH_SIZE;
			sqh = KERNADDR(&dma, offs);
			sqh->physaddr = DMAADDR(&dma, offs);
			sqh->hlink = sc->sc_freeqhs;
			sc->sc_freeqhs = sqh;
		}
	}
	sqh = sc->sc_freeqhs;
	sc->sc_freeqhs = sqh->hlink;
	memset(&sqh->qh, 0, sizeof(uhci_qh_t));
	return (sqh);
}

int
uhci_detach(struct uhci_softc *sc, int flags)
{

	printf("%s: TODO\n", __func__);
	return (0);
}

static int uhci_intr1(uhci_softc_t *);

int
uhci_intr(void *arg)
{
	uhci_softc_t *sc = arg;

	if (sc->sc_dying)
		return (0);

	DPRINTFN(15,("uhci_intr: real interrupt\n"));
	if (sc->sc_bus.use_polling) {
#ifdef DIAGNOSTIC
		printf("uhci_intr: ignored interrupt while polling\n");
#endif
		return (0);
	}
	return (uhci_intr1(sc));
}

int
uhci_intr1(uhci_softc_t *sc)
{

	int status;
	int ack;

	/*
	 * It can happen that an interrupt will be delivered to
	 * us before the device has been fully attached and the
	 * softc struct has been configured. Usually this happens
	 * when kldloading the USB support as a module after the
	 * system has been booted. If we detect this condition,
	 * we need to squelch the unwanted interrupts until we're
	 * ready for them.
	 */
	if (sc->sc_bus.bdev == NULL) {
		UWRITE2(sc, UHCI_STS, 0xFFFF);	/* ack pending interrupts */
		uhci_run(sc, 0);		/* stop the controller */
		UWRITE2(sc, UHCI_INTR, 0);	/* disable interrupts */
		return(0);
	}

#ifdef USB_DEBUG
	if (uhcidebug > 15) {
		DPRINTF(("%s: uhci_intr1\n", device_get_nameunit(sc->sc_bus.bdev)));
		uhci_dumpregs(sc);
	}
#endif
	status = UREAD2(sc, UHCI_STS) & UHCI_STS_ALLINTRS;
	if (status == 0)	/* The interrupt was not for us. */
		return (0);

#if defined(DIAGNOSTIC) && defined(__NetBSD__)
	if (sc->sc_suspend != PWR_RESUME)
		printf("uhci_intr: suspended sts=0x%x\n", status);
#endif

	if (sc->sc_suspend != PWR_RESUME) {
		printf("%s: interrupt while not operating ignored\n",
		       device_get_nameunit(sc->sc_bus.bdev));
		UWRITE2(sc, UHCI_STS, status); /* acknowledge the ints */
		return (0);
	}

	ack = 0;
	if (status & UHCI_STS_USBINT)
		ack |= UHCI_STS_USBINT;
	if (status & UHCI_STS_USBEI)
		ack |= UHCI_STS_USBEI;
	if (status & UHCI_STS_RD) {
		ack |= UHCI_STS_RD;
#ifdef USB_DEBUG
		printf("%s: resume detect\n", device_get_nameunit(sc->sc_bus.bdev));
#endif
	}
	if (status & UHCI_STS_HSE) {
		ack |= UHCI_STS_HSE;
		printf("%s: host system error\n", device_get_nameunit(sc->sc_bus.bdev));
	}
	if (status & UHCI_STS_HCPE) {
		ack |= UHCI_STS_HCPE;
		printf("%s: host controller process error\n",
		       device_get_nameunit(sc->sc_bus.bdev));
	}
	if (status & UHCI_STS_HCH) {
		if (!sc->sc_dying) {
			ack |= UHCI_STS_HCH;
			printf("%s: host controller halted\n",
			       device_get_nameunit(sc->sc_bus.bdev));
		}
	}

	if (!ack)
		return (0);	/* nothing to acknowledge */

	UWRITE2(sc, UHCI_STS, ack & ~UHCI_STS_HCH); /* acknowledge the ints */

	if (ack & UHCI_STS_HCH) {
		/* Restart the controller, by Manuel Bouyer */
		sc->sc_saved_frnum = UREAD2(sc, UHCI_FRNUM);
		sc->sc_saved_sof = UREAD1(sc, UHCI_SOF);

		sc->sc_bus.use_polling++;
		uhci_run(sc, 0); /* stop the controller */
		UWRITE2(sc, UHCI_INTR, 0); /* disable intrs */

		uhci_globalreset(sc);
		uhci_reset(sc);

		/* restore saved state */
		UWRITE4(sc, UHCI_FLBASEADDR, DMAADDR(&sc->sc_dma, 0));
		UWRITE2(sc, UHCI_FRNUM, sc->sc_saved_frnum);
		UWRITE1(sc, UHCI_SOF, sc->sc_saved_sof);

		UWRITE2(sc, UHCI_INTR, UHCI_INTR_TOCRCIE | UHCI_INTR_RIE |
			UHCI_INTR_IOCE | UHCI_INTR_SPIE); /* re-enable intrs */
		UHCICMD(sc, UHCI_CMD_MAXP);

		uhci_run(sc, 1); /* and start traffic again */
		sc->sc_bus.use_polling--;

		if (UREAD2(sc, UHCI_STS) & UHCI_STS_HCH) {
			printf("%s: host controller couldn't be restarted\n",
			       device_get_nameunit(sc->sc_bus.bdev));
#ifdef USB_DEBUG
			uhci_dump_all(sc);
#endif
			sc->sc_dying = 1;
			return (0);
		}

		printf("%s: host controller restarted\n",
		       device_get_nameunit(sc->sc_bus.bdev));
	}

	sc->sc_bus.no_intrs++;
	usb_schedsoftintr(&sc->sc_bus);

	DPRINTFN(15, ("%s: uhci_intr: exit\n", device_get_nameunit(sc->sc_bus.bdev)));

	return (1);
}

void
usb_schedsoftintr(usbd_bus_handle bus)
{
	DPRINTFN(10,("usb_schedsoftintr: polling=%d\n", bus->use_polling));
#ifdef USB_USE_SOFTINTR
	if (bus->use_polling) {
		bus->methods->soft_intr(bus);
	} else {
#ifdef __HAVE_GENERIC_SOFT_INTERRUPTS
		softintr_schedule(bus->soft);
#else
		if (!callout_pending(&bus->softi))
			callout_reset(&bus->softi, 0, bus->methods->soft_intr,
			    bus);
#endif /* __HAVE_GENERIC_SOFT_INTERRUPTS */
	}
#else
       bus->methods->soft_intr(bus);
#endif /* USB_USE_SOFTINTR */
}

#ifdef USB_DEBUG
void
uhci_dump_all(uhci_softc_t *sc)
{
	uhci_dumpregs(sc);
	printf("intrs=%d\n", sc->sc_bus.no_intrs);
	/*printf("framelist[i].link = %08x\n", sc->sc_framelist[0].link);*/
	uhci_dump_qh(sc->sc_lctl_start);
}

static void
uhci_dumpregs(uhci_softc_t *sc)
{
	DPRINTFN(-1,("%s regs: cmd=%04x, sts=%04x, intr=%04x, frnum=%04x, "
		     "flbase=%08x, sof=%04x, portsc1=%04x, portsc2=%04x\n",
		     device_get_nameunit(sc->sc_bus.bdev),
		     UREAD2(sc, UHCI_CMD),
		     UREAD2(sc, UHCI_STS),
		     UREAD2(sc, UHCI_INTR),
		     UREAD2(sc, UHCI_FRNUM),
		     UREAD4(sc, UHCI_FLBASEADDR),
		     UREAD1(sc, UHCI_SOF),
		     UREAD2(sc, UHCI_PORTSC1),
		     UREAD2(sc, UHCI_PORTSC2)));
}

void
uhci_dump_qh(uhci_soft_qh_t *sqh)
{
	DPRINTFN(-1,("QH(%p) at %08x: hlink=%08x elink=%08x\n", sqh,
	    (int)sqh->physaddr, le32toh(sqh->qh.qh_hlink),
	    le32toh(sqh->qh.qh_elink)));
}
#endif

usbd_status
uhci_allocm(struct usbd_bus *bus, usb_dma_t *dma, u_int32_t size)
{
	return (usb_allocmem(bus, size, 0, dma));
}

void
uhci_freem(struct usbd_bus *bus, usb_dma_t *dma)
{
	usb_freemem(bus, dma);
}

usbd_xfer_handle
uhci_allocx(struct usbd_bus *bus)
{

	TODO();
	return (NULL);
}

void
uhci_freex(struct usbd_bus *bus, usbd_xfer_handle xfer)
{

	TODO();
}

/* Open a new pipe. */
usbd_status
uhci_open(usbd_pipe_handle pipe)
{

	TODO();
	return (USBD_INVAL);
}

void
uhci_poll(struct usbd_bus *bus)
{

	TODO();
}

void
uhci_softintr(void *v)
{

	TODO();
}

int
uhci_driver_load(module_t mod, int what, void *arg)
{

	switch (what) {
	case MOD_LOAD:
		usbmem_driver_load();
		break;
	case MOD_UNLOAD:
		usbmem_driver_unload();
		break;
	}
	return usbd_driver_load(mod, what, 0);
}
