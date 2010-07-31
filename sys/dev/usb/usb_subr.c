/*	$NetBSD: usb_subr.c,v 1.99 2002/07/11 21:14:34 augustss Exp $	*/

/* Also already have from NetBSD:
 *	$NetBSD: usb_subr.c,v 1.102 2003/01/01 16:21:50 augustss Exp $
 *	$NetBSD: usb_subr.c,v 1.103 2003/01/10 11:19:13 augustss Exp $
 *	$NetBSD: usb_subr.c,v 1.111 2004/03/15 10:35:04 augustss Exp $
 *	$NetBSD: usb_subr.c,v 1.114 2004/06/23 02:30:52 mycroft Exp $
 *	$NetBSD: usb_subr.c,v 1.115 2004/06/23 05:23:19 mycroft Exp $
 *	$NetBSD: usb_subr.c,v 1.116 2004/06/23 06:27:54 mycroft Exp $
 *	$NetBSD: usb_subr.c,v 1.119 2004/10/23 13:26:33 augustss Exp $
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: stable/7/sys/dev/usb/usb_subr.c 190251 2009-03-22 06:37:14Z n_hibma $");

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

#include "opt_usb.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/proc.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/usb/usb.h>

#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbdivar.h>
#include "usbdevs.h"
#include <dev/usb/usb_quirks.h>

#define delay(d)         DELAY(d)

#ifdef USB_DEBUG
#define DPRINTF(x)	if (usbdebug) printf x
#define DPRINTFN(n,x)	if (usbdebug>(n)) printf x
extern int usbdebug;
#else
#define DPRINTF(x)
#define DPRINTFN(n,x)
#endif

/* Delay for a certain number of ms */
void
usb_delay_ms(usbd_bus_handle bus, u_int ms)
{
	/* Wait at least two clock ticks so we know the time has passed. */
	if (bus->use_polling || cold)
		delay((ms+1) * 1000);
	else
		pause("usbdly", (ms*hz+999)/1000 + 1);
}

/*
 * Called when a new device has been put in the powered state,
 * but not yet in the addressed state.
 * Get initial descriptor, set the address, get full descriptor,
 * and attach a driver.
 */
usbd_status
usbd_new_device(device_t parent, usbd_bus_handle bus, int depth,
		int speed, int port, struct usbd_port *up)
{

	TODO();
	return (USBD_INVAL);
}
