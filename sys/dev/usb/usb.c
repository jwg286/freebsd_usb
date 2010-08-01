/*	$NetBSD: usb.c,v 1.68 2002/02/20 20:30:12 christos Exp $	*/

/* Also already merged from NetBSD:
 *	$NetBSD: usb.c,v 1.70 2002/05/09 21:54:32 augustss Exp $
 *	$NetBSD: usb.c,v 1.71 2002/06/01 23:51:04 lukem Exp $
 *	$NetBSD: usb.c,v 1.73 2002/09/23 05:51:19 simonb Exp $
 *	$NetBSD: usb.c,v 1.80 2003/11/07 17:03:25 wiz Exp $
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: stable/7/sys/dev/usb/usb.c 170960 2007-06-20 05:11:37Z imp $");

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
 * USB specifications and other documentation can be found at
 * http://www.usb.org/developers/docs/ and
 * http://www.usb.org/developers/devclass_docs/
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/unistd.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/fcntl.h>
#include <sys/filio.h>
#include <sys/uio.h>
#include <sys/kthread.h>
#include <sys/proc.h>
#include <sys/conf.h>
#include <sys/poll.h>
#include <sys/selinfo.h>
#include <sys/signalvar.h>
#include <sys/sysctl.h>
#include <sys/uio.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>

#define USBUNIT(d)	(minor(d))	/* usb_discover device nodes, kthread */
#define USB_DEV_MINOR	255		/* event queue device */

MALLOC_DEFINE(M_USB, "USB", "USB");
MALLOC_DEFINE(M_USBDEV, "USBdev", "USB device");
MALLOC_DEFINE(M_USBHC, "USBHC", "USB host controller");

#include "usb_if.h"

#include <machine/bus.h>

#include <dev/usb/usbdivar.h>
#include <dev/usb/usb_quirks.h>

/* Define this unconditionally in case a kernel module is loaded that
 * has been compiled with debugging options.
 */
SYSCTL_NODE(_hw, OID_AUTO, usb, CTLFLAG_RW, 0, "USB debugging");

#ifdef USB_DEBUG
#define DPRINTF(x)	if (usbdebug) printf x
#define DPRINTFN(n,x)	if (usbdebug>(n)) printf x
int	usbdebug = 0;
SYSCTL_INT(_hw_usb, OID_AUTO, debug, CTLFLAG_RW,
	   &usbdebug, 0, "usb debug level");
/*
 * 0  - do usual exploration
 * 1  - do not use timeout exploration
 * >1 - do no exploration
 */
int	usb_noexplore = 0;
#else
#define DPRINTF(x)
#define DPRINTFN(n,x)
#endif

struct usb_softc {
	device_t	sc_dev;		/* base device */
	struct cdev	*sc_usbdev;	/* /dev/usbN device */
	TAILQ_ENTRY(usb_softc) sc_coldexplist; /* cold needs-explore list */
	usbd_bus_handle sc_bus;		/* USB controller */
	struct usbd_port sc_port;	/* dummy port for root hub */

	struct proc	*sc_event_thread;

	char		sc_dying;
};

struct usb_taskq {
	TAILQ_HEAD(, usb_task) tasks;
	struct proc *task_thread_proc;
	const char *name;
	int taskcreated;		/* task thread exists. */
};

static struct usb_taskq usb_taskq[USB_NUM_TASKQS];

d_open_t  usbopen;
d_close_t usbclose;
d_read_t usbread;
d_ioctl_t usbioctl;
d_poll_t usbpoll;

struct cdevsw usb_cdevsw = {
	.d_version =	D_VERSION,
	.d_flags =	D_NEEDGIANT,
	.d_open =	usbopen,
	.d_close =	usbclose,
	.d_read =	usbread,
	.d_ioctl =	usbioctl,
	.d_poll =	usbpoll,
	.d_name =	"usb",
};

static void	usb_discover(void *);
static void	usb_create_event_thread(void *);
static void	usb_event_thread(void *);
static void	usb_task_thread(void *);

static struct cdev *usb_dev;		/* The /dev/usb device. */
static int usb_ndevs;			/* Number of /dev/usbN devices. */
/* Busses to explore at the end of boot-time device configuration. */
static TAILQ_HEAD(, usb_softc) usb_coldexplist =
    TAILQ_HEAD_INITIALIZER(usb_coldexplist);

static void usb_add_event(int, struct usb_event *);

static const char *usbrev_str[] = USBREV_STR;

static device_probe_t usb_match;
static device_attach_t usb_attach;
static device_detach_t usb_detach;
static bus_child_detached_t usb_child_detached;

static device_method_t usb_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		usb_match),
	DEVMETHOD(device_attach,	usb_attach),
	DEVMETHOD(device_detach,	usb_detach),
	DEVMETHOD(device_suspend,	bus_generic_suspend),
	DEVMETHOD(device_resume,	bus_generic_resume),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),

	/* Bus interface */
	DEVMETHOD(bus_child_detached,	usb_child_detached),

	{ 0, 0 }
};

static driver_t usb_driver = {
	"usb",
	usb_methods,
	sizeof(struct usb_softc)
};

static devclass_t usb_devclass;

DRIVER_MODULE(usb, ohci, usb_driver, usb_devclass, 0, 0);
DRIVER_MODULE(usb, uhci, usb_driver, usb_devclass, 0, 0);
DRIVER_MODULE(usb, ehci, usb_driver, usb_devclass, 0, 0);
DRIVER_MODULE(usb, slhci, usb_driver, usb_devclass, 0, 0);
MODULE_VERSION(usb, 1);

static int
usb_match(device_t self)
{
	DPRINTF(("usbd_match\n"));
	return (UMATCH_GENERIC);
}

static int
usb_attach(device_t self)
{
	struct usb_softc *sc = device_get_softc(self);
	void *aux = device_get_ivars(self);
	usbd_device_handle dev;
	usbd_status err;
	int usbrev;
	int speed;
	struct usb_event ue;

	sc->sc_dev = self;

	DPRINTF(("usbd_attach\n"));

	usbd_init();
	sc->sc_bus = aux;
	sc->sc_bus->usbctl = sc;
	sc->sc_port.power = USB_MAX_POWER;

	printf("%s", device_get_nameunit(sc->sc_dev));
	usbrev = sc->sc_bus->usbrev;
	printf(": USB revision %s", usbrev_str[usbrev]);
	switch (usbrev) {
	case USBREV_1_0:
	case USBREV_1_1:
		speed = USB_SPEED_FULL;
		break;
	case USBREV_2_0:
		speed = USB_SPEED_HIGH;
		break;
	default:
		printf(", not supported\n");
		sc->sc_dying = 1;
		return ENXIO;
	}
	printf("\n");

	/* Make sure not to use msleep() if we are cold booting. */
	if (cold)
		sc->sc_bus->use_polling++;

	ue.u.ue_ctrlr.ue_bus = device_get_unit(sc->sc_dev);
	usb_add_event(USB_EVENT_CTRLR_ATTACH, &ue);

#ifdef USB_USE_SOFTINTR
#ifdef __HAVE_GENERIC_SOFT_INTERRUPTS
	/* XXX we should have our own level */
	sc->sc_bus->soft = softintr_establish(IPL_SOFTNET,
	    sc->sc_bus->methods->soft_intr, sc->sc_bus);
	if (sc->sc_bus->soft == NULL) {
		printf("%s: can't register softintr\n", device_get_nameunit(sc->sc_dev));
		sc->sc_dying = 1;
		return ENXIO;
	}
#else
	usb_callout_init(sc->sc_bus->softi);
#endif
#endif

	err = usbd_new_device(sc->sc_dev, sc->sc_bus, 0, speed, 0,
		  &sc->sc_port);
	if (!err) {
		dev = sc->sc_port.device;
		if (dev->hub == NULL) {
			sc->sc_dying = 1;
			printf("%s: root device is not a hub\n",
			       device_get_nameunit(sc->sc_dev));
			return ENXIO;
		}
		sc->sc_bus->root_hub = dev;
#if 1
		/*
		 * Turning this code off will delay attachment of USB devices
		 * until the USB event thread is running, which means that
		 * the keyboard will not work until after cold boot.
		 */
		if (cold) {
			/* Explore high-speed busses before others. */
			if (speed == USB_SPEED_HIGH)
				dev->hub->explore(sc->sc_bus->root_hub);
			else
				TAILQ_INSERT_TAIL(&usb_coldexplist, sc,
				    sc_coldexplist);
		}
#endif
	} else {
		printf("%s: root hub problem, error=%d\n",
		       device_get_nameunit(sc->sc_dev), err);
		sc->sc_dying = 1;
	}
	if (cold)
		sc->sc_bus->use_polling--;

	/* XXX really do right config_pending_incr(); */
	usb_create_event_thread(sc);
	/* The per controller devices (used for usb_discover) */
	/* XXX This is redundant now, but old usbd's will want it */
	sc->sc_usbdev = make_dev(&usb_cdevsw, device_get_unit(self), UID_ROOT,
	    GID_OPERATOR, 0660, "usb%d", device_get_unit(self));
	if (atomic_cmpset_int(&usb_ndevs, 0, 1)) {
		/* The device spitting out events */
		usb_dev = make_dev(&usb_cdevsw, USB_DEV_MINOR, UID_ROOT,
		    GID_OPERATOR, 0660, "usb");
	} else
		atomic_add_int(&usb_ndevs, 1);
	return 0;
}

static int
usb_detach(device_t self)
{
	struct usb_softc *sc = device_get_softc(self);
	struct usb_event ue;
	struct usb_taskq *taskq;
	int i;

	DPRINTF(("usb_detach: start\n"));

	sc->sc_dying = 1;

	/* Make all devices disconnect. */
	if (sc->sc_port.device != NULL)
		usb_disconnect_port(&sc->sc_port, self);

	/* Kill off event thread. */
	if (sc->sc_event_thread != NULL) {
		wakeup(&sc->sc_bus->needs_explore);
		if (tsleep(sc, PWAIT, "usbdet", hz * 60))
			printf("%s: event thread didn't die\n",
			       device_get_nameunit(sc->sc_dev));
		DPRINTF(("usb_detach: event thread dead\n"));
	}

	destroy_dev(sc->sc_usbdev);
	if (--usb_ndevs == 0) {
		destroy_dev(usb_dev);
		usb_dev = NULL;
		for (i = 0; i < USB_NUM_TASKQS; i++) {
			taskq = &usb_taskq[i];
			wakeup(&taskq->tasks);
			if (tsleep(&taskq->taskcreated, PWAIT, "usbtdt",
			    hz * 60)) {
				printf("usb task thread %s didn't die\n",
				    taskq->name);
			}
		}
	}

	usbd_finish();

#ifdef USB_USE_SOFTINTR
#ifdef __HAVE_GENERIC_SOFT_INTERRUPTS
	if (sc->sc_bus->soft != NULL) {
		softintr_disestablish(sc->sc_bus->soft);
		sc->sc_bus->soft = NULL;
	}
#else
	callout_stop(&sc->sc_bus->softi);
#endif
#endif

	ue.u.ue_ctrlr.ue_bus = device_get_unit(sc->sc_dev);
	usb_add_event(USB_EVENT_CTRLR_DETACH, &ue);

	return (0);
}

static void
usb_child_detached(device_t self, device_t child)
{
	struct usb_softc *sc = device_get_softc(self);

	/* XXX, should check it is the right device. */
	sc->sc_port.device = NULL;
}

void
usb_add_event(int type, struct usb_event *uep)
{

	TODO();
}

static const char *taskq_names[] = USB_TASKQ_NAMES;

void
usb_create_event_thread(void *arg)
{
	struct usb_softc *sc = arg;
	struct usb_taskq *taskq;
	int i;

	if (kproc_create(usb_event_thread, sc, &sc->sc_event_thread,
	      RFHIGHPID, 0, device_get_nameunit(sc->sc_dev))) {
		printf("%s: unable to create event thread for\n",
		       device_get_nameunit(sc->sc_dev));
		panic("usb_create_event_thread");
	}
	for (i = 0; i < USB_NUM_TASKQS; i++) {
		taskq = &usb_taskq[i];

		if (taskq->taskcreated == 0) {
			taskq->taskcreated = 1;
			taskq->name = taskq_names[i];
			TAILQ_INIT(&taskq->tasks);
			if (kproc_create(usb_task_thread, taskq,
			    &taskq->task_thread_proc, RFHIGHPID, 0,
			    taskq->name)) {
				printf("unable to create task thread\n");
				panic("usb_create_event_thread task");
			}
		}
	}
}

int
usbopen(struct cdev *dev, int flag, int mode, struct thread *p)
{

	TODO();
	return (ENXIO);
}

int
usbread(struct cdev *dev, struct uio *uio, int flag)
{

	TODO();
	return (ENXIO);
}

int
usbclose(struct cdev *dev, int flag, int mode, struct thread *p)
{

	TODO();
	return (ENXIO);
}

int
usbioctl(struct cdev *devt, u_long cmd, caddr_t data, int flag, struct thread *p)
{

	TODO();
	return (ENXIO);
}

int
usbpoll(struct cdev *dev, int events, struct thread *p)
{

	TODO();
	return (ENXIO);
}

void
usbd_add_dev_event(int type, usbd_device_handle udev)
{

	TODO();
}

void
usbd_add_drv_event(int type, usbd_device_handle udev, device_t dev)
{

	TODO();
}

void
usb_needs_explore(usbd_device_handle dev)
{
	DPRINTFN(2,("usb_needs_explore\n"));
	dev->bus->needs_explore = 1;
	wakeup(&dev->bus->needs_explore);
}

void
usb_task_thread(void *arg)
{
	struct usb_task *task;
	struct usb_taskq *taskq;
	struct mtx mtx;

	taskq = arg;
	DPRINTF(("usb_task_thread: start taskq %s\n", taskq->name));

	bzero(&mtx, sizeof(struct mtx));
	mtx_init(&mtx, taskq->name, NULL, MTX_DEF);

	while (usb_ndevs > 0) {
		mtx_lock(&mtx);
		task = TAILQ_FIRST(&taskq->tasks);
		if (task == NULL) {
			msleep(&taskq->tasks, &mtx, PWAIT, "usbtsk", 0);
			task = TAILQ_FIRST(&taskq->tasks);
		}
		DPRINTFN(2,("usb_task_thread: woke up task=%p\n", task));
		if (task != NULL) {
			TAILQ_REMOVE(&taskq->tasks, task, next);
			task->queue = -1;
			mtx_unlock(&mtx);
			task->fun(task->arg);
			mtx_lock(&mtx);
		}
		mtx_unlock(&mtx);
	}

	mtx_destroy(&mtx);

	taskq->taskcreated = 0;
	wakeup(&taskq->taskcreated);

	DPRINTF(("usb_event_thread: exit\n"));
	kproc_exit(0);
}

void
usb_event_thread(void *arg)
{
	static int newthread_wchan;
	struct usb_softc *sc = arg;

	TODO();
	DPRINTF(("usb_event_thread: start\n"));

	/*
	 * In case this controller is a companion controller to an
	 * EHCI controller we need to wait until the EHCI controller
	 * has grabbed the port. What we do here is wait until no new
	 * USB threads have been created in a while. XXX we actually
	 * just want to wait for the PCI slot to be fully scanned.
	 *
	 * Note that when you `kldload usb' it actually attaches the
	 * devices in order that the drivers appear in the kld, not the
	 * normal PCI order, since the addition of each driver within
	 * usb.ko (ohci, ehci etc.) causes a separate PCI bus re-scan.
	 */
	wakeup(&newthread_wchan);
	for (;;) {
		if (tsleep(&newthread_wchan , PWAIT, "usbets", hz * 4) != 0)
			break;
	}

	/* Make sure first discover does something. */
	sc->sc_bus->needs_explore = 1;
	usb_discover(sc);
	/* XXX really do right config_pending_decr(); */

	while (!sc->sc_dying) {
#ifdef USB_DEBUG
		if (usb_noexplore < 2)
#endif
		usb_discover(sc);
#ifdef USB_DEBUG
		(void)tsleep(&sc->sc_bus->needs_explore, PWAIT, "usbevt",
		    usb_noexplore ? 0 : hz * 60);
#else
		(void)tsleep(&sc->sc_bus->needs_explore, PWAIT, "usbevt",
		    hz * 60);
#endif
		DPRINTFN(2,("usb_event_thread: woke up\n"));
	}
	sc->sc_event_thread = NULL;

	/* In case parent is waiting for us to exit. */
	wakeup(sc);

	DPRINTF(("usb_event_thread: exit\n"));
	kproc_exit(0);
}

/* Explore device tree from the root. */
static void
usb_discover(void *v)
{
	struct usb_softc *sc = v;

	DPRINTFN(2,("usb_discover\n"));
#ifdef USB_DEBUG
	if (usb_noexplore > 1)
		return;
#endif

	/*
	 * We need mutual exclusion while traversing the device tree,
	 * but this is guaranteed since this function is only called
	 * from the event thread for the controller.
	 */
	TODO(); /* LOCK! */
	while (sc->sc_bus->needs_explore && !sc->sc_dying) {
		sc->sc_bus->needs_explore = 0;
		sc->sc_bus->root_hub->hub->explore(sc->sc_bus->root_hub);
	}
}
