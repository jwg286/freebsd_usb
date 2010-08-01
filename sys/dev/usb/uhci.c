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
#include <sys/taskqueue.h>

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

static void		uhci_reset_proc(void *, int);
static void		uhci_poll_hub(void *);
static void		uhci_aux_dma_complete(uhci_soft_td_t *, int);
static usbd_status	uhci_portreset(uhci_softc_t*, int);
static void		uhci_timeout_task(void *);
static void		uhci_check_intr(uhci_softc_t *, uhci_intr_info_t *);
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

static usbd_status	uhci_root_ctrl_transfer(usbd_xfer_handle);
static usbd_status	uhci_root_ctrl_start(usbd_xfer_handle);
static void		uhci_root_ctrl_abort(usbd_xfer_handle);
static void		uhci_root_ctrl_close(usbd_pipe_handle);
static void		uhci_root_ctrl_done(usbd_xfer_handle);

static usbd_status	uhci_root_intr_transfer(usbd_xfer_handle);
static usbd_status	uhci_root_intr_start(usbd_xfer_handle);
static void		uhci_root_intr_abort(usbd_xfer_handle);
static void		uhci_root_intr_close(usbd_pipe_handle);
static void		uhci_root_intr_done(usbd_xfer_handle);

static usbd_status	uhci_device_ctrl_transfer(usbd_xfer_handle);
static usbd_status	uhci_device_ctrl_start(usbd_xfer_handle);
static void		uhci_device_ctrl_abort(usbd_xfer_handle);
static void		uhci_device_ctrl_close(usbd_pipe_handle);
static void		uhci_device_ctrl_done(usbd_xfer_handle);

static usbd_status	uhci_device_intr_transfer(usbd_xfer_handle);
static usbd_status	uhci_device_intr_start(usbd_xfer_handle);
static void		uhci_device_intr_abort(usbd_xfer_handle);
static void		uhci_device_intr_close(usbd_pipe_handle);
static void		uhci_device_intr_done(usbd_xfer_handle);

static usbd_status	uhci_device_isoc_transfer(usbd_xfer_handle);
static usbd_status	uhci_device_isoc_start(usbd_xfer_handle);
static void		uhci_device_isoc_abort(usbd_xfer_handle);
static void		uhci_device_isoc_close(usbd_pipe_handle);
static void		uhci_device_isoc_done(usbd_xfer_handle);

static usbd_status	uhci_device_bulk_transfer(usbd_xfer_handle);
static usbd_status	uhci_device_bulk_start(usbd_xfer_handle);
static void		uhci_device_bulk_abort(usbd_xfer_handle);
static void		uhci_device_bulk_close(usbd_pipe_handle);
static void		uhci_device_bulk_done(usbd_xfer_handle);

static usbd_status	uhci_setup_isoc(usbd_pipe_handle pipe);

static void		uhci_device_clear_toggle(usbd_pipe_handle pipe);
static void		uhci_noop(usbd_pipe_handle pipe);

static usbd_status	uhci_device_setintr(uhci_softc_t *sc,
			    struct uhci_pipe *pipe, int ival);

static void		uhci_free_sqh(uhci_softc_t *, uhci_soft_qh_t *);
static void		uhci_free_std(uhci_softc_t *, uhci_soft_td_t *);
static int		uhci_str(usb_string_descriptor_t *, int, char *);
static void		uhci_transfer_complete(usbd_xfer_handle xfer);

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

struct usbd_pipe_methods uhci_root_ctrl_methods = {
	uhci_root_ctrl_transfer,
	uhci_root_ctrl_start,
	uhci_root_ctrl_abort,
	uhci_root_ctrl_close,
	uhci_noop,
	uhci_root_ctrl_done,
};

struct usbd_pipe_methods uhci_root_intr_methods = {
	uhci_root_intr_transfer,
	uhci_root_intr_start,
	uhci_root_intr_abort,
	uhci_root_intr_close,
	uhci_noop,
	uhci_root_intr_done,
};

struct usbd_pipe_methods uhci_device_ctrl_methods = {
	uhci_device_ctrl_transfer,
	uhci_device_ctrl_start,
	uhci_device_ctrl_abort,
	uhci_device_ctrl_close,
	uhci_noop,
	uhci_device_ctrl_done,
};

struct usbd_pipe_methods uhci_device_intr_methods = {
	uhci_device_intr_transfer,
	uhci_device_intr_start,
	uhci_device_intr_abort,
	uhci_device_intr_close,
	uhci_device_clear_toggle,
	uhci_device_intr_done,
};

struct usbd_pipe_methods uhci_device_isoc_methods = {
	uhci_device_isoc_transfer,
	uhci_device_isoc_start,
	uhci_device_isoc_abort,
	uhci_device_isoc_close,
	uhci_noop,
	uhci_device_isoc_done,
};

struct usbd_pipe_methods uhci_device_bulk_methods = {
	uhci_device_bulk_transfer,
	uhci_device_bulk_start,
	uhci_device_bulk_abort,
	uhci_device_bulk_close,
	uhci_device_clear_toggle,
	uhci_device_bulk_done,
};

/*
 * Data structures and routines to emulate the root hub.
 */
usb_device_descriptor_t uhci_devd = {
	USB_DEVICE_DESCRIPTOR_SIZE,
	UDESC_DEVICE,		/* type */
	{0x00, 0x01},		/* USB version */
	UDCLASS_HUB,		/* class */
	UDSUBCLASS_HUB,		/* subclass */
	UDPROTO_FSHUB,		/* protocol */
	64,			/* max packet */
	{0},{0},{0x00,0x01},	/* device id */
	1,2,0,			/* string indicies */
	1			/* # of configurations */
};

usb_config_descriptor_t uhci_confd = {
	USB_CONFIG_DESCRIPTOR_SIZE,
	UDESC_CONFIG,
	{USB_CONFIG_DESCRIPTOR_SIZE +
	 USB_INTERFACE_DESCRIPTOR_SIZE +
	 USB_ENDPOINT_DESCRIPTOR_SIZE},
	1,
	1,
	0,
	UC_SELF_POWERED,
	0			/* max power */
};

usb_interface_descriptor_t uhci_ifcd = {
	USB_INTERFACE_DESCRIPTOR_SIZE,
	UDESC_INTERFACE,
	0,
	0,
	1,
	UICLASS_HUB,
	UISUBCLASS_HUB,
	UIPROTO_FSHUB,
	0
};

usb_endpoint_descriptor_t uhci_endpd = {
	USB_ENDPOINT_DESCRIPTOR_SIZE,
	UDESC_ENDPOINT,
	UE_DIR_IN | UHCI_INTR_ENDPT,
	UE_INTERRUPT,
	{8},
	255
};

usb_hub_descriptor_t uhci_hubd_piix = {
	USB_HUB_DESCRIPTOR_SIZE,
	UDESC_HUB,
	2,
	{ UHD_PWR_NO_SWITCH | UHD_OC_INDIVIDUAL, 0 },
	50,			/* power on to power good */
	0,
	{ 0x00 },		/* both ports are removable */
};

int
uhci_str(usb_string_descriptor_t *p, int l, char *s)
{
	int i;

	if (l == 0)
		return (0);
	p->bLength = 2 * strlen(s) + 2;
	if (l == 1)
		return (1);
	p->bDescriptorType = UDESC_STRING;
	l -= 2;
	for (i = 0; s[i] && l > 1; i++, l -= 2)
		USETW2(p->bString[i], 0, s[i]);
	return (2*i+2);
}

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
	TASK_INIT(&sc->sc_resettask, 0, uhci_reset_proc, sc);

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
		UHCI_UNLOCK(sc);
		usb_delay_ms(&sc->sc_bus, 1);
		UHCI_LOCK(sc);
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
	usbd_xfer_handle xfer;
	int rv = 0;

	sc->sc_dying = 1;

	UWRITE2(sc, UHCI_INTR, 0);		/* disable interrupts */
	uhci_run(sc, 0);

#if defined(__NetBSD__) || defined(__OpenBSD__)
	powerhook_disestablish(sc->sc_powerhook);
	shutdownhook_disestablish(sc->sc_shutdownhook);
#endif

	/* Free all xfers associated with this HC. */
	for (;;) {
		xfer = STAILQ_FIRST(&sc->sc_free_xfers);
		if (xfer == NULL)
			break;
		STAILQ_REMOVE_HEAD(&sc->sc_free_xfers, next);
		free(xfer, M_USB);
	}

	/* XXX free other data structures XXX */
	usb_freemem(&sc->sc_bus, &sc->sc_dma);

	return (rv);
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
		taskqueue_enqueue(sc->sc_tq, &sc->sc_resettask);
		return (0);
	}

	sc->sc_bus.no_intrs++;
	usb_schedsoftintr(&sc->sc_bus);

	DPRINTFN(15, ("%s: uhci_intr: exit\n", device_get_nameunit(sc->sc_bus.bdev)));

	return (1);
}

static void
uhci_reset_proc(void *arg, int npending)
{
	uhci_softc_t *sc = arg;

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
	}

	printf("%s: host controller restarted\n",
	    device_get_nameunit(sc->sc_bus.bdev));
}

void
usb_schedsoftintr(usbd_bus_handle bus)
{

	DPRINTFN(10,("usb_schedsoftintr: polling=%d\n", bus->use_polling));
	KASSERT(bus->methods != NULL, ("method pointer is NULL"));
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

void
uhci_timeout_task(void *addr)
{

	/* XXX locking */
	TODO();
}

usbd_xfer_handle
uhci_allocx(struct usbd_bus *bus)
{
	struct uhci_softc *sc = (struct uhci_softc *)bus;
	usbd_xfer_handle xfer;

	UHCI_LOCK(sc);
	xfer = STAILQ_FIRST(&sc->sc_free_xfers);
	if (xfer != NULL) {
		STAILQ_REMOVE_HEAD(&sc->sc_free_xfers, next);
#ifdef DIAGNOSTIC
		if (xfer->busy_free != XFER_FREE) {
			printf("uhci_allocx: xfer=%p not free, 0x%08x\n", xfer,
			       xfer->busy_free);
		}
#endif
	} else {
		xfer = malloc(sizeof(struct uhci_xfer), M_USB, M_NOWAIT);
	}
	if (xfer != NULL) {
		memset(xfer, 0, sizeof (struct uhci_xfer));
		UXFER(xfer)->iinfo.sc = sc;
		usb_init_task(&UXFER(xfer)->abort_task, uhci_timeout_task,
		    xfer);
		UXFER(xfer)->uhci_xfer_flags = 0;
#ifdef DIAGNOSTIC
		UXFER(xfer)->iinfo.isdone = 1;
		xfer->busy_free = XFER_BUSY;
#endif
	}
	UHCI_UNLOCK(sc);
	return (xfer);
}

void
uhci_freex(struct usbd_bus *bus, usbd_xfer_handle xfer)
{
	struct uhci_softc *sc = (struct uhci_softc *)bus;

	UHCI_LOCK(sc);
#ifdef DIAGNOSTIC
	if (xfer->busy_free != XFER_BUSY) {
		printf("uhci_freex: xfer=%p not busy, 0x%08x\n", xfer,
		       xfer->busy_free);
		UHCI_UNLOCK(sc);
		return;
	}
	xfer->busy_free = XFER_FREE;
	if (!UXFER(xfer)->iinfo.isdone) {
		printf("uhci_freex: !isdone\n");
		UHCI_UNLOCK(sc);
		return;
	}
#endif
	STAILQ_INSERT_HEAD(&sc->sc_free_xfers, xfer, next);
	UHCI_UNLOCK(sc);
}

/* Open a new pipe. */
usbd_status
uhci_open(usbd_pipe_handle pipe)
{
	uhci_softc_t *sc = (uhci_softc_t *)pipe->device->bus;
	struct uhci_pipe *upipe = (struct uhci_pipe *)pipe;
	usb_endpoint_descriptor_t *ed = pipe->endpoint->edesc;
	usbd_status err;
	int ival;

	DPRINTFN(1, ("uhci_open: pipe=%p, addr=%d, endpt=%d (%d)\n",
		     pipe, pipe->device->address,
		     ed->bEndpointAddress, sc->sc_addr));

	upipe->aborting = 0;
	upipe->nexttoggle = pipe->endpoint->savedtoggle;

	if (pipe->device->address == sc->sc_addr) {
		switch (ed->bEndpointAddress) {
		case USB_CONTROL_ENDPOINT:
			pipe->methods = &uhci_root_ctrl_methods;
			break;
		case UE_DIR_IN | UHCI_INTR_ENDPT:
			pipe->methods = &uhci_root_intr_methods;
			break;
		default:
			return (USBD_INVAL);
		}
	} else {
		switch (ed->bmAttributes & UE_XFERTYPE) {
		case UE_CONTROL:
			pipe->methods = &uhci_device_ctrl_methods;
			upipe->u.ctl.sqh = uhci_alloc_sqh(sc);
			if (upipe->u.ctl.sqh == NULL)
				goto bad;
			upipe->u.ctl.setup = uhci_alloc_std(sc);
			if (upipe->u.ctl.setup == NULL) {
				uhci_free_sqh(sc, upipe->u.ctl.sqh);
				goto bad;
			}
			upipe->u.ctl.stat = uhci_alloc_std(sc);
			if (upipe->u.ctl.stat == NULL) {
				uhci_free_sqh(sc, upipe->u.ctl.sqh);
				uhci_free_std(sc, upipe->u.ctl.setup);
				goto bad;
			}
			err = usb_allocmem(&sc->sc_bus,
				  sizeof(usb_device_request_t),
				  0, &upipe->u.ctl.reqdma);
			if (err) {
				uhci_free_sqh(sc, upipe->u.ctl.sqh);
				uhci_free_std(sc, upipe->u.ctl.setup);
				uhci_free_std(sc, upipe->u.ctl.stat);
				goto bad;
			}
			break;
		case UE_INTERRUPT:
			pipe->methods = &uhci_device_intr_methods;
			ival = pipe->interval;
			if (ival == USBD_DEFAULT_INTERVAL)
				ival = ed->bInterval;
			return (uhci_device_setintr(sc, upipe, ival));
		case UE_ISOCHRONOUS:
			pipe->methods = &uhci_device_isoc_methods;
			return (uhci_setup_isoc(pipe));
		case UE_BULK:
			pipe->methods = &uhci_device_bulk_methods;
			upipe->u.bulk.sqh = uhci_alloc_sqh(sc);
			if (upipe->u.bulk.sqh == NULL)
				goto bad;
			break;
		}
	}
	return (USBD_NORMAL_COMPLETION);

 bad:
	return (USBD_NOMEM);
}

void
uhci_poll(struct usbd_bus *bus)
{

	TODO();
}

void
uhci_softintr(void *v)
{
	uhci_softc_t *sc = v;
	uhci_intr_info_t *ii, *nextii;

	DPRINTFN(10,("%s: uhci_softintr (%d)\n", device_get_nameunit(sc->sc_bus.bdev),
		     sc->sc_bus.intr_context));

	sc->sc_bus.intr_context++;

	/*
	 * Interrupts on UHCI really suck.  When the host controller
	 * interrupts because a transfer is completed there is no
	 * way of knowing which transfer it was.  You can scan down
	 * the TDs and QHs of the previous frame to limit the search,
	 * but that assumes that the interrupt was not delayed by more
	 * than 1 ms, which may not always be true (e.g. after debug
	 * output on a slow console).
	 * We scan all interrupt descriptors to see if any have
	 * completed.
	 */
	LIST_FOREACH_SAFE(ii, &sc->sc_intrhead, list, nextii)
		uhci_check_intr(sc, ii);

#ifdef USB_USE_SOFTINTR
	if (sc->sc_softwake) {
		sc->sc_softwake = 0;
		wakeup(&sc->sc_softwake);
	}
#endif /* USB_USE_SOFTINTR */

	sc->sc_bus.intr_context--;
}

/* Check for an interrupt. */
void
uhci_check_intr(uhci_softc_t *sc, uhci_intr_info_t *ii)
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

void
uhci_free_sqh(uhci_softc_t *sc, uhci_soft_qh_t *sqh)
{
	sqh->hlink = sc->sc_freeqhs;
	sc->sc_freeqhs = sqh;
}

usbd_status
uhci_device_setintr(uhci_softc_t *sc, struct uhci_pipe *upipe, int ival)
{

	TODO();
	return (USBD_INVAL);
}

/*
 * Simulate a hardware hub by handling all the necessary requests.
 */
usbd_status
uhci_root_ctrl_transfer(usbd_xfer_handle xfer)
{
	usbd_status err;

	/* Insert last in queue. */
	err = usb_insert_transfer(xfer);
	if (err)
		return (err);

	/*
	 * Pipe isn't running (otherwise err would be USBD_INPROG),
	 * so start it first.
	 */
	return (uhci_root_ctrl_start(STAILQ_FIRST(&xfer->pipe->queue)));
}

usbd_status
uhci_root_ctrl_start(usbd_xfer_handle xfer)
{
	uhci_softc_t *sc = (uhci_softc_t *)xfer->pipe->device->bus;
	usb_device_request_t *req;
	void *buf = NULL;
	int port, x;
	int len, value, index, status, change, l, totlen = 0;
	usb_port_status_t ps;
	usbd_status err;

	if (sc->sc_dying)
		return (USBD_IOERROR);

#ifdef DIAGNOSTIC
	if (!(xfer->rqflags & URQ_REQUEST))
		panic("uhci_root_ctrl_transfer: not a request");
#endif
	req = &xfer->request;

	DPRINTFN(2,("uhci_root_ctrl_control type=0x%02x request=%02x\n",
		    req->bmRequestType, req->bRequest));

	len = UGETW(req->wLength);
	value = UGETW(req->wValue);
	index = UGETW(req->wIndex);

	if (len != 0)
		buf = xfer->buffer;

#define C(x,y) ((x) | ((y) << 8))
	switch(C(req->bRequest, req->bmRequestType)) {
	case C(UR_CLEAR_FEATURE, UT_WRITE_DEVICE):
	case C(UR_CLEAR_FEATURE, UT_WRITE_INTERFACE):
	case C(UR_CLEAR_FEATURE, UT_WRITE_ENDPOINT):
		/*
		 * DEVICE_REMOTE_WAKEUP and ENDPOINT_HALT are no-ops
		 * for the integrated root hub.
		 */
		break;
	case C(UR_GET_CONFIG, UT_READ_DEVICE):
		if (len > 0) {
			*(u_int8_t *)buf = sc->sc_conf;
			totlen = 1;
		}
		break;
	case C(UR_GET_DESCRIPTOR, UT_READ_DEVICE):
		DPRINTFN(2,("uhci_root_ctrl_control wValue=0x%04x\n", value));
		switch(value >> 8) {
		case UDESC_DEVICE:
			if ((value & 0xff) != 0) {
				err = USBD_IOERROR;
				goto ret;
			}
			totlen = l = min(len, USB_DEVICE_DESCRIPTOR_SIZE);
			USETW(uhci_devd.idVendor, sc->sc_id_vendor);
			memcpy(buf, &uhci_devd, l);
			break;
		case UDESC_CONFIG:
			if ((value & 0xff) != 0) {
				err = USBD_IOERROR;
				goto ret;
			}
			totlen = l = min(len, USB_CONFIG_DESCRIPTOR_SIZE);
			memcpy(buf, &uhci_confd, l);
			buf = (char *)buf + l;
			len -= l;
			l = min(len, USB_INTERFACE_DESCRIPTOR_SIZE);
			totlen += l;
			memcpy(buf, &uhci_ifcd, l);
			buf = (char *)buf + l;
			len -= l;
			l = min(len, USB_ENDPOINT_DESCRIPTOR_SIZE);
			totlen += l;
			memcpy(buf, &uhci_endpd, l);
			break;
		case UDESC_STRING:
			if (len == 0)
				break;
			*(u_int8_t *)buf = 0;
			totlen = 1;
			switch (value & 0xff) {
			case 1: /* Vendor */
				totlen = uhci_str(buf, len, sc->sc_vendor);
				break;
			case 2: /* Product */
				totlen = uhci_str(buf, len, "UHCI root hub");
				break;
			}
			break;
		default:
			err = USBD_IOERROR;
			goto ret;
		}
		break;
	case C(UR_GET_INTERFACE, UT_READ_INTERFACE):
		if (len > 0) {
			*(u_int8_t *)buf = 0;
			totlen = 1;
		}
		break;
	case C(UR_GET_STATUS, UT_READ_DEVICE):
		if (len > 1) {
			USETW(((usb_status_t *)buf)->wStatus,UDS_SELF_POWERED);
			totlen = 2;
		}
		break;
	case C(UR_GET_STATUS, UT_READ_INTERFACE):
	case C(UR_GET_STATUS, UT_READ_ENDPOINT):
		if (len > 1) {
			USETW(((usb_status_t *)buf)->wStatus, 0);
			totlen = 2;
		}
		break;
	case C(UR_SET_ADDRESS, UT_WRITE_DEVICE):
		if (value >= USB_MAX_DEVICES) {
			err = USBD_IOERROR;
			goto ret;
		}
		sc->sc_addr = value;
		break;
	case C(UR_SET_CONFIG, UT_WRITE_DEVICE):
		if (value != 0 && value != 1) {
			err = USBD_IOERROR;
			goto ret;
		}
		sc->sc_conf = value;
		break;
	case C(UR_SET_DESCRIPTOR, UT_WRITE_DEVICE):
		break;
	case C(UR_SET_FEATURE, UT_WRITE_DEVICE):
	case C(UR_SET_FEATURE, UT_WRITE_INTERFACE):
	case C(UR_SET_FEATURE, UT_WRITE_ENDPOINT):
		err = USBD_IOERROR;
		goto ret;
	case C(UR_SET_INTERFACE, UT_WRITE_INTERFACE):
		break;
	case C(UR_SYNCH_FRAME, UT_WRITE_ENDPOINT):
		break;
	/* Hub requests */
	case C(UR_CLEAR_FEATURE, UT_WRITE_CLASS_DEVICE):
		break;
	case C(UR_CLEAR_FEATURE, UT_WRITE_CLASS_OTHER):
		DPRINTFN(3, ("uhci_root_ctrl_control: UR_CLEAR_PORT_FEATURE "
			     "port=%d feature=%d\n",
			     index, value));
		if (index == 1)
			port = UHCI_PORTSC1;
		else if (index == 2)
			port = UHCI_PORTSC2;
		else {
			err = USBD_IOERROR;
			goto ret;
		}
		switch(value) {
		case UHF_PORT_ENABLE:
			x = URWMASK(UREAD2(sc, port));
			UWRITE2(sc, port, x & ~UHCI_PORTSC_PE);
			break;
		case UHF_PORT_SUSPEND:
			x = URWMASK(UREAD2(sc, port));
			UWRITE2(sc, port, x & ~UHCI_PORTSC_SUSP);
			break;
		case UHF_PORT_RESET:
			x = URWMASK(UREAD2(sc, port));
			UWRITE2(sc, port, x & ~UHCI_PORTSC_PR);
			break;
		case UHF_C_PORT_CONNECTION:
			x = URWMASK(UREAD2(sc, port));
			UWRITE2(sc, port, x | UHCI_PORTSC_CSC);
			break;
		case UHF_C_PORT_ENABLE:
			x = URWMASK(UREAD2(sc, port));
			UWRITE2(sc, port, x | UHCI_PORTSC_POEDC);
			break;
		case UHF_C_PORT_OVER_CURRENT:
			x = URWMASK(UREAD2(sc, port));
			UWRITE2(sc, port, x | UHCI_PORTSC_OCIC);
			break;
		case UHF_C_PORT_RESET:
			sc->sc_isreset = 0;
			err = USBD_NORMAL_COMPLETION;
			goto ret;
		case UHF_PORT_CONNECTION:
		case UHF_PORT_OVER_CURRENT:
		case UHF_PORT_POWER:
		case UHF_PORT_LOW_SPEED:
		case UHF_C_PORT_SUSPEND:
		default:
			err = USBD_IOERROR;
			goto ret;
		}
		break;
	case C(UR_GET_BUS_STATE, UT_READ_CLASS_OTHER):
		if (index == 1)
			port = UHCI_PORTSC1;
		else if (index == 2)
			port = UHCI_PORTSC2;
		else {
			err = USBD_IOERROR;
			goto ret;
		}
		if (len > 0) {
			*(u_int8_t *)buf =
				(UREAD2(sc, port) & UHCI_PORTSC_LS) >>
				UHCI_PORTSC_LS_SHIFT;
			totlen = 1;
		}
		break;
	case C(UR_GET_DESCRIPTOR, UT_READ_CLASS_DEVICE):
		if ((value & 0xff) != 0) {
			err = USBD_IOERROR;
			goto ret;
		}
		l = min(len, USB_HUB_DESCRIPTOR_SIZE);
		totlen = l;
		memcpy(buf, &uhci_hubd_piix, l);
		break;
	case C(UR_GET_STATUS, UT_READ_CLASS_DEVICE):
		if (len != 4) {
			err = USBD_IOERROR;
			goto ret;
		}
		memset(buf, 0, len);
		totlen = len;
		break;
	case C(UR_GET_STATUS, UT_READ_CLASS_OTHER):
		if (index == 1)
			port = UHCI_PORTSC1;
		else if (index == 2)
			port = UHCI_PORTSC2;
		else {
			err = USBD_IOERROR;
			goto ret;
		}
		if (len != 4) {
			err = USBD_IOERROR;
			goto ret;
		}
		x = UREAD2(sc, port);
		status = change = 0;
		if (x & UHCI_PORTSC_CCS)
			status |= UPS_CURRENT_CONNECT_STATUS;
		if (x & UHCI_PORTSC_CSC)
			change |= UPS_C_CONNECT_STATUS;
		if (x & UHCI_PORTSC_PE)
			status |= UPS_PORT_ENABLED;
		if (x & UHCI_PORTSC_POEDC)
			change |= UPS_C_PORT_ENABLED;
		if (x & UHCI_PORTSC_OCI)
			status |= UPS_OVERCURRENT_INDICATOR;
		if (x & UHCI_PORTSC_OCIC)
			change |= UPS_C_OVERCURRENT_INDICATOR;
		if (x & UHCI_PORTSC_SUSP)
			status |= UPS_SUSPEND;
		if (x & UHCI_PORTSC_LSDA)
			status |= UPS_LOW_SPEED;
		status |= UPS_PORT_POWER;
		if (sc->sc_isreset)
			change |= UPS_C_PORT_RESET;
		USETW(ps.wPortStatus, status);
		USETW(ps.wPortChange, change);
		l = min(len, sizeof ps);
		memcpy(buf, &ps, l);
		totlen = l;
		break;
	case C(UR_SET_DESCRIPTOR, UT_WRITE_CLASS_DEVICE):
		err = USBD_IOERROR;
		goto ret;
	case C(UR_SET_FEATURE, UT_WRITE_CLASS_DEVICE):
		break;
	case C(UR_SET_FEATURE, UT_WRITE_CLASS_OTHER):
		if (index == 1)
			port = UHCI_PORTSC1;
		else if (index == 2)
			port = UHCI_PORTSC2;
		else {
			err = USBD_IOERROR;
			goto ret;
		}
		switch(value) {
		case UHF_PORT_ENABLE:
			x = URWMASK(UREAD2(sc, port));
			UWRITE2(sc, port, x | UHCI_PORTSC_PE);
			break;
		case UHF_PORT_SUSPEND:
			x = URWMASK(UREAD2(sc, port));
			UWRITE2(sc, port, x | UHCI_PORTSC_SUSP);
			break;
		case UHF_PORT_RESET:
			err = uhci_portreset(sc, index);
			goto ret;
		case UHF_PORT_POWER:
			/* Pretend we turned on power */
			err = USBD_NORMAL_COMPLETION;
			goto ret;
		case UHF_C_PORT_CONNECTION:
		case UHF_C_PORT_ENABLE:
		case UHF_C_PORT_OVER_CURRENT:
		case UHF_PORT_CONNECTION:
		case UHF_PORT_OVER_CURRENT:
		case UHF_PORT_LOW_SPEED:
		case UHF_C_PORT_SUSPEND:
		case UHF_C_PORT_RESET:
		default:
			err = USBD_IOERROR;
			goto ret;
		}
		break;
	default:
		err = USBD_IOERROR;
		goto ret;
	}
	xfer->actlen = totlen;
	err = USBD_NORMAL_COMPLETION;
 ret:
	xfer->status = err;
	uhci_transfer_complete(xfer);
	return (USBD_IN_PROGRESS);
}

/* Abort a root control request. */
void
uhci_root_ctrl_abort(usbd_xfer_handle xfer)
{
	/* Nothing to do, all transfers are synchronous. */
}

/* Close the root pipe. */
void
uhci_root_ctrl_close(usbd_pipe_handle pipe)
{
	DPRINTF(("uhci_root_ctrl_close\n"));
}

void
uhci_root_ctrl_done(usbd_xfer_handle xfer)
{

	/* do nothing */
}

usbd_status
uhci_root_intr_transfer(usbd_xfer_handle xfer)
{
	usbd_status err;

	/* Insert last in queue. */
	err = usb_insert_transfer(xfer);
	if (err)
		return (err);

	/*
	 * Pipe isn't running (otherwise err would be USBD_INPROG),
	 * so start it first.
	 */
	return (uhci_root_intr_start(STAILQ_FIRST(&xfer->pipe->queue)));
}

/* Start a transfer on the root interrupt pipe */
usbd_status
uhci_root_intr_start(usbd_xfer_handle xfer)
{
	usbd_pipe_handle pipe = xfer->pipe;
	uhci_softc_t *sc = (uhci_softc_t *)pipe->device->bus;

	DPRINTFN(3, ("uhci_root_intr_start: xfer=%p len=%d flags=%d\n",
		     xfer, xfer->length, xfer->flags));

	if (sc->sc_dying)
		return (USBD_IOERROR);

	sc->sc_ival = MS_TO_TICKS(xfer->pipe->endpoint->edesc->bInterval);
	callout_reset(&sc->sc_poll_handle, sc->sc_ival, uhci_poll_hub, xfer);
	sc->sc_intr_xfer = xfer;
	return (USBD_IN_PROGRESS);
}

/* Close the root interrupt pipe. */
void
uhci_root_intr_close(usbd_pipe_handle pipe)
{
	uhci_softc_t *sc = (uhci_softc_t *)pipe->device->bus;

	callout_stop(&sc->sc_poll_handle);
	sc->sc_intr_xfer = NULL;
	DPRINTF(("uhci_root_intr_close\n"));
}

/* Abort a root interrupt request. */
void
uhci_root_intr_abort(usbd_xfer_handle xfer)
{
	uhci_softc_t *sc = (uhci_softc_t *)xfer->pipe->device->bus;

	callout_stop(&sc->sc_poll_handle);
	sc->sc_intr_xfer = NULL;

	if (xfer->pipe->intrxfer == xfer) {
		DPRINTF(("uhci_root_intr_abort: remove\n"));
		xfer->pipe->intrxfer = 0;
	}
	xfer->status = USBD_CANCELLED;
#ifdef DIAGNOSTIC
	UXFER(xfer)->iinfo.isdone = 1;
#endif
	uhci_transfer_complete(xfer);
}

void
uhci_root_intr_done(usbd_xfer_handle xfer)
{

	/* do nothing */
}

usbd_status
uhci_device_ctrl_transfer(usbd_xfer_handle xfer)
{

	TODO();
	return (USBD_INVAL);
}

usbd_status
uhci_device_ctrl_start(usbd_xfer_handle xfer)
{

	TODO();
	return (USBD_INVAL);
}

/* Abort a device control request. */
void
uhci_device_ctrl_abort(usbd_xfer_handle xfer)
{

	TODO();
}

/* Close a device control pipe. */
void
uhci_device_ctrl_close(usbd_pipe_handle pipe)
{
}

/* Deallocate request data structures */
void
uhci_device_ctrl_done(usbd_xfer_handle xfer)
{

	TODO();
}

usbd_status
uhci_device_intr_transfer(usbd_xfer_handle xfer)
{

	TODO();
	return (USBD_INVAL);
}

usbd_status
uhci_device_intr_start(usbd_xfer_handle xfer)
{

	TODO();
	return (USBD_INVAL);
}

/* Abort a device interrupt request. */
void
uhci_device_intr_abort(usbd_xfer_handle xfer)
{

	TODO();
}

/* Close a device interrupt pipe. */
void
uhci_device_intr_close(usbd_pipe_handle pipe)
{

	TODO();
}

void
uhci_device_intr_done(usbd_xfer_handle xfer)
{

	TODO();
}

usbd_status
uhci_device_isoc_transfer(usbd_xfer_handle xfer)
{

	TODO();
	return (USBD_INVAL);
}

usbd_status
uhci_device_isoc_start(usbd_xfer_handle xfer)
{

	TODO();
	return (USBD_INVAL);
}

void
uhci_device_isoc_abort(usbd_xfer_handle xfer)
{

	TODO();
}

void
uhci_device_isoc_close(usbd_pipe_handle pipe)
{

	TODO();
}

void
uhci_device_isoc_done(usbd_xfer_handle xfer)
{

	TODO();
}

usbd_status
uhci_device_bulk_transfer(usbd_xfer_handle xfer)
{

	TODO();
	return (USBD_INVAL);
}

usbd_status
uhci_device_bulk_start(usbd_xfer_handle xfer)
{

	TODO();
	return (USBD_INVAL);
}

/* Abort a device bulk request. */
void
uhci_device_bulk_abort(usbd_xfer_handle xfer)
{

	TODO();
}

/* Close a device bulk pipe. */
void
uhci_device_bulk_close(usbd_pipe_handle pipe)
{

	TODO();
}

/* Deallocate request data structures */
void
uhci_device_bulk_done(usbd_xfer_handle xfer)
{

	TODO();
}

usbd_status
uhci_setup_isoc(usbd_pipe_handle pipe)
{

	TODO();
	return (USBD_INVAL);
}

void
uhci_device_clear_toggle(usbd_pipe_handle pipe)
{
	struct uhci_pipe *upipe = (struct uhci_pipe *)pipe;
	upipe->nexttoggle = 0;
}

void
uhci_noop(usbd_pipe_handle pipe)
{
}

void
uhci_free_std(uhci_softc_t *sc, uhci_soft_td_t *std)
{
#ifdef DIAGNOSTIC
#define TD_IS_FREE 0x12345678
	if (le32toh(std->td.td_token) == TD_IS_FREE) {
		printf("uhci_free_std: freeing free TD %p\n", std);
		return;
	}
	std->td.td_token = htole32(TD_IS_FREE);
#endif
	if (std->aux_dma.block != NULL) {
		usb_freemem(&sc->sc_bus, &std->aux_dma);
		std->aux_dma.block = NULL;
		std->aux_data = NULL;
		std->aux_len = 0;
	}
	std->link.std = sc->sc_freetds;
	sc->sc_freetds = std;
}

/*
 * The USB hub protocol requires that SET_FEATURE(PORT_RESET) also
 * enables the port, and also states that SET_FEATURE(PORT_ENABLE)
 * should not be used by the USB subsystem.  As we cannot issue a
 * SET_FEATURE(PORT_ENABLE) externally, we must ensure that the port
 * will be enabled as part of the reset.
 *
 * On the VT83C572, the port cannot be successfully enabled until the
 * outstanding "port enable change" and "connection status change"
 * events have been reset.
 */
static usbd_status
uhci_portreset(uhci_softc_t *sc, int index)
{
	int lim, port, x;

	if (index == 1)
		port = UHCI_PORTSC1;
	else if (index == 2)
		port = UHCI_PORTSC2;
	else
		return (USBD_IOERROR);

	x = URWMASK(UREAD2(sc, port));
	UWRITE2(sc, port, x | UHCI_PORTSC_PR);

	usb_delay_ms(&sc->sc_bus, USB_PORT_ROOT_RESET_DELAY);

	DPRINTFN(3,("uhci port %d reset, status0 = 0x%04x\n",
		    index, UREAD2(sc, port)));

	x = URWMASK(UREAD2(sc, port));
	UWRITE2(sc, port, x & ~UHCI_PORTSC_PR);

	delay(100);

	DPRINTFN(3,("uhci port %d reset, status1 = 0x%04x\n",
		    index, UREAD2(sc, port)));

	x = URWMASK(UREAD2(sc, port));
	UWRITE2(sc, port, x  | UHCI_PORTSC_PE);

	for (lim = 10; --lim > 0;) {
		usb_delay_ms(&sc->sc_bus, USB_PORT_RESET_DELAY);

		x = UREAD2(sc, port);

		DPRINTFN(3,("uhci port %d iteration %u, status = 0x%04x\n",
			    index, lim, x));

		if (!(x & UHCI_PORTSC_CCS)) {
			/*
			 * No device is connected (or was disconnected
			 * during reset).  Consider the port reset.
			 * The delay must be long enough to ensure on
			 * the initial iteration that the device
			 * connection will have been registered.  50ms
			 * appears to be sufficient, but 20ms is not.
			 */
			DPRINTFN(3,("uhci port %d loop %u, device detached\n",
				    index, lim));
			break;
		}

		if (x & (UHCI_PORTSC_POEDC | UHCI_PORTSC_CSC)) {
			/*
			 * Port enabled changed and/or connection
			 * status changed were set.  Reset either or
			 * both raised flags (by writing a 1 to that
			 * bit), and wait again for state to settle.
			 */
			UWRITE2(sc, port, URWMASK(x) |
				(x & (UHCI_PORTSC_POEDC | UHCI_PORTSC_CSC)));
			continue;
		}

		if (x & UHCI_PORTSC_PE)
			/* Port is enabled */
			break;

		UWRITE2(sc, port, URWMASK(x) | UHCI_PORTSC_PE);
	}

	DPRINTFN(3,("uhci port %d reset, status2 = 0x%04x\n",
		    index, UREAD2(sc, port)));

	if (lim <= 0) {
		DPRINTFN(1,("uhci port %d reset timed out\n", index));
		return (USBD_TIMEOUT);
	}

	sc->sc_isreset = 1;
	return (USBD_NORMAL_COMPLETION);
}

/*
 * Perform any UHCI-specific transfer completion operations, then
 * call usb_transfer_complete().
 */
static void
uhci_transfer_complete(usbd_xfer_handle xfer)
{
	uhci_intr_info_t *ii = &UXFER(xfer)->iinfo;
	struct uhci_pipe *upipe = (struct uhci_pipe *)xfer->pipe;
	uhci_soft_td_t *p;
	int i, isread, n;

	/* XXX, must be an easier way to detect reads... */
	isread = ((xfer->rqflags & URQ_REQUEST) &&
	    (xfer->request.bmRequestType & UT_READ)) ||
            (xfer->pipe->endpoint->edesc->bEndpointAddress & UE_DIR_IN);

	/* Copy back from any auxillary buffers after a read operation. */
	if (xfer->nframes == 0) {
		for (p = ii->stdstart; p != NULL; p = p->link.std) {
			if (p->aux_data != NULL)
				uhci_aux_dma_complete(p, isread);
		}
	} else {
		if (xfer->nframes != 0) {
			/* Isoc transfer, do things differently. */
			n = UXFER(xfer)->curframe;
			for (i = 0; i < xfer->nframes; i++) {
				p = upipe->u.iso.stds[n];
				if (p->aux_data != NULL)
					uhci_aux_dma_complete(p, isread);
				if (++n >= UHCI_VFRAMELIST_COUNT)
				n = 0;
			}
		}
	}

	usb_transfer_complete(xfer);
}

static void
uhci_aux_dma_complete(uhci_soft_td_t *std, int isread)
{
	if (isread) {
		bus_dmamap_sync(std->aux_dma.block->tag,
		    std->aux_dma.block->map, BUS_DMASYNC_POSTREAD);
		bcopy(KERNADDR(&std->aux_dma, 0), std->aux_data, std->aux_len);
	}
}

/*
 * This routine is executed periodically and simulates interrupts
 * from the root controller interrupt pipe for port status change.
 */
void
uhci_poll_hub(void *addr)
{
	usbd_xfer_handle xfer = addr;
	usbd_pipe_handle pipe = xfer->pipe;
	usbd_device_handle dev = pipe->device;
	uhci_softc_t *sc = (uhci_softc_t *)dev->bus;
	u_char *p;

	DPRINTFN(20, ("uhci_poll_hub\n"));

	callout_reset(&sc->sc_poll_handle, sc->sc_ival, uhci_poll_hub, xfer);

	p = xfer->buffer;
	p[0] = 0;
	if (UREAD2(sc, UHCI_PORTSC1) & (UHCI_PORTSC_CSC|UHCI_PORTSC_OCIC))
		p[0] |= 1<<1;
	if (UREAD2(sc, UHCI_PORTSC2) & (UHCI_PORTSC_CSC|UHCI_PORTSC_OCIC))
		p[0] |= 1<<2;
	if (p[0] == 0)
		/* No change, try again in a while */
		return;

	xfer->actlen = 1;
	xfer->status = USBD_NORMAL_COMPLETION;
	dev->bus->intr_context++;
	uhci_transfer_complete(xfer);
	dev->bus->intr_context--;
}
