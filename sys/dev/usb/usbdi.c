/*	$NetBSD: usbdi.c,v 1.106 2004/10/24 12:52:40 augustss Exp $	*/

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: stable/7/sys/dev/usb/usbdi.c 190251 2009-03-22 06:37:14Z n_hibma $");

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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/module.h>
#include <sys/bus.h>
#include "usb_if.h"
#if defined(DIAGNOSTIC) && defined(__i386__)
#include <machine/cpu.h>
#endif
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/malloc.h>
#include <sys/proc.h>

#include <machine/bus.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbdivar.h>
#include <dev/usb/usb_mem.h>
#include <dev/usb/usb_quirks.h>

#include "usb_if.h"
#define delay(d)	DELAY(d)

#ifdef USB_DEBUG
#define DPRINTF(x)	if (usbdebug) printf x
#define DPRINTFN(n,x)	if (usbdebug>(n)) printf x
extern int usbdebug;
#else
#define DPRINTF(x)
#define DPRINTFN(n,x)
#endif

static void usbd_start_transfer(void *arg, bus_dma_segment_t *segs, int nseg,
    int error);
static void usbd_start_next(usbd_pipe_handle pipe);
static usbd_status usbd_open_pipe_ival
	(usbd_interface_handle, u_int8_t, u_int8_t, usbd_pipe_handle *, int);
static void usbd_do_request_async_cb
	(usbd_xfer_handle, usbd_private_handle, usbd_status);

static int usbd_nbuses = 0;

void
usbd_init(void)
{
	usbd_nbuses++;
}

void
usbd_finish(void)
{
	--usbd_nbuses;
}

static __inline int
usbd_xfer_isread(usbd_xfer_handle xfer)
{
	if (xfer->rqflags & URQ_REQUEST)
		return (xfer->request.bmRequestType & UT_READ);
	else
		return (xfer->pipe->endpoint->edesc->bEndpointAddress &
			UE_DIR_IN);
}

int
usbd_driver_load(module_t mod, int what, void *arg)
{
	/* XXX should implement something like a function that removes all generic devices */

 	return (0);
}

usbd_status
usbd_clear_endpoint_stall(usbd_pipe_handle pipe)
{

	TODO();
	return (USBD_INVAL);
}

usbd_status
usbd_do_request(usbd_device_handle dev, usb_device_request_t *req, void *data)
{
	return (usbd_do_request_flags(dev, req, data, 0, 0,
				      USBD_DEFAULT_TIMEOUT));
}

usbd_status
usbd_do_request_flags(usbd_device_handle dev, usb_device_request_t *req,
		      void *data, u_int16_t flags, int *actlen, u_int32_t timo)
{
	return (usbd_do_request_flags_pipe(dev, dev->default_pipe, req,
					   data, flags, actlen, timo));
}

usbd_status
usbd_do_request_flags_pipe(usbd_device_handle dev, usbd_pipe_handle pipe,
	usb_device_request_t *req, void *data, u_int16_t flags, int *actlen,
	u_int32_t timeout)
{
	usbd_xfer_handle xfer;
	usbd_status err;

#ifdef DIAGNOSTIC
/* XXX amd64 too? */
#if defined(__i386__)
	KASSERT(curthread->td_intr_nesting_level == 0,
	       	("usbd_do_request: in interrupt context"));
#endif
	if (dev->bus->intr_context) {
		printf("usbd_do_request: not in process context\n");
		return (USBD_INVAL);
	}
#endif

	xfer = usbd_alloc_xfer(dev);
	if (xfer == NULL)
		return (USBD_NOMEM);
	usbd_setup_default_xfer(xfer, dev, 0, timeout, req,
				data, UGETW(req->wLength), flags, 0);
	xfer->pipe = pipe;
	err = usbd_sync_transfer(xfer);
#if defined(USB_DEBUG) || defined(DIAGNOSTIC)
	if (xfer->actlen > xfer->length)
		DPRINTF(("usbd_do_request: overrun addr=%d type=0x%02x req=0x"
			 "%02x val=%d index=%d rlen=%d length=%d actlen=%d\n",
			 dev->address, xfer->request.bmRequestType,
			 xfer->request.bRequest, UGETW(xfer->request.wValue),
			 UGETW(xfer->request.wIndex),
			 UGETW(xfer->request.wLength),
			 xfer->length, xfer->actlen));
#endif
	if (actlen != NULL)
		*actlen = xfer->actlen;
	if (err == USBD_STALLED) {
		/*
		 * The control endpoint has stalled.  Control endpoints
		 * should not halt, but some may do so anyway so clear
		 * any halt condition.
		 */
		usb_device_request_t treq;
		usb_status_t status;
		u_int16_t s;
		usbd_status nerr;

		treq.bmRequestType = UT_READ_ENDPOINT;
		treq.bRequest = UR_GET_STATUS;
		USETW(treq.wValue, 0);
		USETW(treq.wIndex, 0);
		USETW(treq.wLength, sizeof(usb_status_t));
		usbd_setup_default_xfer(xfer, dev, 0, USBD_DEFAULT_TIMEOUT,
					   &treq, &status,sizeof(usb_status_t),
					   0, 0);
		nerr = usbd_sync_transfer(xfer);
		if (nerr)
			goto bad;
		s = UGETW(status.wStatus);
		DPRINTF(("usbd_do_request: status = 0x%04x\n", s));
		if (!(s & UES_HALT))
			goto bad;
		treq.bmRequestType = UT_WRITE_ENDPOINT;
		treq.bRequest = UR_CLEAR_FEATURE;
		USETW(treq.wValue, UF_ENDPOINT_HALT);
		USETW(treq.wIndex, 0);
		USETW(treq.wLength, 0);
		usbd_setup_default_xfer(xfer, dev, 0, USBD_DEFAULT_TIMEOUT,
					   &treq, &status, 0, 0, 0);
		nerr = usbd_sync_transfer(xfer);
		if (nerr)
			goto bad;
	}

 bad:
	usbd_free_xfer(xfer);
	return (err);
}

usbd_xfer_handle
usbd_alloc_xfer(usbd_device_handle dev)
{
	usbd_xfer_handle xfer;

	xfer = dev->bus->methods->allocx(dev->bus);
	if (xfer == NULL)
		return (NULL);
	xfer->device = dev;
	callout_init(&xfer->timeout_handle, 0);
	DPRINTFN(5,("usbd_alloc_xfer() = %p\n", xfer));
	return (xfer);
}

void
usbd_setup_default_xfer(usbd_xfer_handle xfer, usbd_device_handle dev,
			usbd_private_handle priv, u_int32_t timeout,
			usb_device_request_t *req, void *buffer,
			u_int32_t length, u_int16_t flags,
			usbd_callback callback)
{
	xfer->pipe = dev->default_pipe;
	xfer->priv = priv;
	xfer->buffer = buffer;
	xfer->length = length;
	xfer->actlen = 0;
	xfer->flags = flags;
	xfer->timeout = timeout;
	xfer->status = USBD_NOT_STARTED;
	xfer->callback = callback;
	xfer->request = *req;
	xfer->rqflags |= URQ_REQUEST;
	xfer->nframes = 0;
}

/* Like usbd_transfer(), but waits for completion. */
usbd_status
usbd_sync_transfer(usbd_xfer_handle xfer)
{
	xfer->flags |= USBD_SYNCHRONOUS;
	return (usbd_transfer(xfer));
}

usbd_status
usbd_free_xfer(usbd_xfer_handle xfer)
{
	DPRINTFN(5,("usbd_free_xfer: %p\n", xfer));
	if (xfer->rqflags & URQ_DEV_DMABUF)
		usbd_free_buffer(xfer);
/* XXX Does FreeBSD need to do something similar? */
#if defined(__NetBSD__) && defined(DIAGNOSTIC)
	if (callout_pending(&xfer->timeout_handle)) {
		callout_stop(&xfer->timeout_handle);
		printf("usbd_free_xfer: timout_handle pending");
	}
#endif
	xfer->device->bus->methods->freex(xfer->device->bus, xfer);
	return (USBD_NORMAL_COMPLETION);
}

usbd_status
usbd_transfer(usbd_xfer_handle xfer)
{
	usbd_pipe_handle pipe = xfer->pipe;
	struct usb_dma_mapping *dmap = &xfer->dmamap;
	usbd_status err;
	u_int size;

	DPRINTFN(5,("usbd_transfer: xfer=%p, flags=%d, pipe=%p, running=%d\n",
		    xfer, xfer->flags, pipe, pipe->running));
#ifdef USB_DEBUG
	if (usbdebug > 5)
		usbd_dump_queue(pipe);
#endif
	xfer->done = 0;

	if (pipe->aborting)
		return (USBD_CANCELLED);

	size = xfer->length;
	/* If there is no buffer, allocate one. */
	if (!(xfer->rqflags & URQ_DEV_DMABUF) && size != 0) {
		bus_dma_tag_t tag = pipe->device->bus->buffer_dmatag;

#ifdef DIAGNOSTIC
		if (xfer->rqflags & URQ_AUTO_DMABUF)
			printf("usbd_transfer: has old buffer!\n");
#endif
		err = bus_dmamap_create(tag, 0, &dmap->map);
		if (err)
			return (USBD_NOMEM);

		xfer->rqflags |= URQ_AUTO_DMABUF;
		err = bus_dmamap_load(tag, dmap->map, xfer->buffer, size,
		    usbd_start_transfer, xfer, 0);
		if (err != 0 && err != EINPROGRESS) {
			xfer->rqflags &= ~URQ_AUTO_DMABUF;
			bus_dmamap_destroy(tag, dmap->map);
			return (USBD_INVAL);
		}
	} else if (size != 0) {
		usbd_start_transfer(xfer, dmap->segs, dmap->nsegs, 0);
	} else {
		usbd_start_transfer(xfer, NULL, 0, 0);
	}

	if (!(xfer->flags & USBD_SYNCHRONOUS))
		return (xfer->done ? 0 : USBD_IN_PROGRESS);

	/* Sync transfer, wait for completion. */
	while (!xfer->done) {
		if (pipe->device->bus->use_polling)
			panic("usbd_transfer: not done");
		tsleep(xfer, PRIBIO, "usbsyn", 0);
	}
	return (xfer->status);
}

static void
usbd_start_transfer(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	usbd_xfer_handle xfer = arg;
	usbd_pipe_handle pipe = xfer->pipe;
	struct usb_dma_mapping *dmap = &xfer->dmamap;
	bus_dma_tag_t tag = pipe->device->bus->buffer_dmatag;
	int err, i;

	if (error != 0) {
		KASSERT(xfer->rqflags & URQ_AUTO_DMABUF,
		    ("usbd_start_transfer: error with non-auto buf"));
		if (nseg > 0)
			bus_dmamap_unload(tag, dmap->map);
		bus_dmamap_destroy(tag, dmap->map);
		/* XXX */
		usb_insert_transfer(xfer);
		xfer->status = USBD_IOERROR;
		usb_transfer_complete(xfer);
		return;
	}

	if (segs != dmap->segs) {
		for (i = 0; i < nseg; i++)
			dmap->segs[i] = segs[i];
	}
	dmap->nsegs = nseg;

	if (nseg > 0) {
		if (!usbd_xfer_isread(xfer)) {
			/*
			 * Copy data if it is not already in the correct
			 * buffer.
			 */
			if (!(xfer->flags & USBD_NO_COPY) &&
			    xfer->allocbuf != NULL &&
			    xfer->buffer != xfer->allocbuf)
				memcpy(xfer->allocbuf, xfer->buffer,
				    xfer->length);
			bus_dmamap_sync(tag, dmap->map, BUS_DMASYNC_PREWRITE);
		} else if (xfer->rqflags & URQ_REQUEST) {
			/*
			 * Even if we have no data portion we still need to
			 * sync the dmamap for the request data in the SETUP
			 * packet.
			 */
			bus_dmamap_sync(tag, dmap->map,
			    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
		} else
			bus_dmamap_sync(tag, dmap->map, BUS_DMASYNC_PREREAD);
	}
	err = pipe->methods->transfer(xfer);
	if (err != USBD_IN_PROGRESS && err) {
		if (xfer->rqflags & URQ_AUTO_DMABUF) {
			bus_dmamap_unload(tag, dmap->map);
			bus_dmamap_destroy(tag, dmap->map);
			xfer->rqflags &= ~URQ_AUTO_DMABUF;
		}
		xfer->status = err;
		usb_transfer_complete(xfer);
		return;
	}
}

void
usbd_free_buffer(usbd_xfer_handle xfer)
{
	struct usb_dma_mapping *dmap = &xfer->dmamap;
	bus_dma_tag_t tag = xfer->device->bus->buffer_dmatag;

	KASSERT((xfer->rqflags & (URQ_DEV_DMABUF | URQ_AUTO_DMABUF)) ==
	    URQ_DEV_DMABUF, ("usbd_free_buffer: no/auto buffer"));

	xfer->rqflags &= ~URQ_DEV_DMABUF;
	bus_dmamap_unload(tag, dmap->map);
	bus_dmamap_destroy(tag, dmap->map);
	free(xfer->allocbuf, M_USB);
	xfer->allocbuf = NULL;
}

void
usbd_dump_queue(usbd_pipe_handle pipe)
{
	usbd_xfer_handle xfer;

	printf("usbd_dump_queue: pipe=%p\n", pipe);
	STAILQ_FOREACH(xfer, &pipe->queue, next) {
		printf("  xfer=%p\n", xfer);
	}
}

usbd_status
usb_insert_transfer(usbd_xfer_handle xfer)
{
	usbd_pipe_handle pipe = xfer->pipe;
	usbd_status err;

	DPRINTFN(5,("usb_insert_transfer: pipe=%p running=%d timeout=%d\n",
		    pipe, pipe->running, xfer->timeout));
#ifdef DIAGNOSTIC
	if (xfer->busy_free != XFER_BUSY) {
		printf("usb_insert_transfer: xfer=%p not busy 0x%08x\n",
		       xfer, xfer->busy_free);
		return (USBD_INVAL);
	}
	xfer->busy_free = XFER_ONQU;
#endif
	USBD_PIPE_LOCK(pipe);
	KASSERT(STAILQ_FIRST(&pipe->queue) != xfer, ("usb_insert_transfer"));
	STAILQ_INSERT_TAIL(&pipe->queue, xfer, next);
	if (pipe->running)
		err = USBD_IN_PROGRESS;
	else {
		pipe->running = 1;
		err = USBD_NORMAL_COMPLETION;
	}
	USBD_PIPE_UNLOCK(pipe);
	return (err);
}

void
usbd_start_next(usbd_pipe_handle pipe)
{
	usbd_xfer_handle xfer;
	usbd_status err;

#ifdef DIAGNOSTIC
	if (pipe == NULL) {
		printf("usbd_start_next: pipe == NULL\n");
		return;
	}
	if (pipe->methods == NULL || pipe->methods->start == NULL) {
		printf("usbd_start_next: pipe=%p no start method\n", pipe);
		return;
	}
#endif

	USBD_PIPE_LOCK(pipe);
	/* Get next request in queue. */
	xfer = STAILQ_FIRST(&pipe->queue);
	DPRINTFN(5, ("usbd_start_next: pipe=%p, xfer=%p\n", pipe, xfer));
	if (xfer == NULL) {
		pipe->running = 0;
	} else {
		err = pipe->methods->start(xfer);
		if (err != USBD_IN_PROGRESS) {
			printf("usbd_start_next: error=%d\n", err);
			pipe->running = 0;
			/* XXX do what? */
		}
	}
	USBD_PIPE_UNLOCK(pipe);
}

void
usb_transfer_complete(usbd_xfer_handle xfer)
{
	usbd_pipe_handle pipe = xfer->pipe;
	struct usb_dma_mapping *dmap = &xfer->dmamap;
	bus_dma_tag_t tag = pipe->device->bus->buffer_dmatag;
	int sync = xfer->flags & USBD_SYNCHRONOUS;
	int erred = xfer->status == USBD_CANCELLED ||
	    xfer->status == USBD_TIMEOUT;
	int repeat = pipe->repeat;
	int polling;

	DPRINTFN(5, ("usb_transfer_complete: pipe=%p xfer=%p status=%d "
		     "actlen=%d\n", pipe, xfer, xfer->status, xfer->actlen));
#ifdef DIAGNOSTIC
	if (xfer->busy_free != XFER_ONQU) {
		printf("usb_transfer_complete: xfer=%p not busy 0x%08x\n",
		       xfer, xfer->busy_free);
		return;
	}
#endif

#ifdef DIAGNOSTIC
	if (pipe == NULL) {
		printf("usbd_transfer_cb: pipe==0, xfer=%p\n", xfer);
		return;
	}
#endif
	polling = pipe->device->bus->use_polling;
	/* XXXX */
	if (polling)
		pipe->running = 0;

	if (xfer->actlen != 0 && usbd_xfer_isread(xfer)) {
		bus_dmamap_sync(tag, dmap->map, BUS_DMASYNC_POSTREAD);
		/* Copy data if it is not already in the correct buffer. */
		if (!(xfer->flags & USBD_NO_COPY) && xfer->allocbuf != NULL &&
		    xfer->buffer != xfer->allocbuf)
			memcpy(xfer->buffer, xfer->allocbuf, xfer->actlen);
	}

	/* if we mapped the buffer in usbd_transfer() we unmap it here. */
	if (xfer->rqflags & URQ_AUTO_DMABUF) {
		if (!repeat) {
			bus_dmamap_unload(tag, dmap->map);
			bus_dmamap_destroy(tag, dmap->map);
			xfer->rqflags &= ~URQ_AUTO_DMABUF;
		}
	}

	if (!repeat) {
		/* Remove request from queue. */
#ifdef DIAGNOSTIC
		xfer->busy_free = XFER_BUSY;
#endif
		KASSERT(STAILQ_FIRST(&pipe->queue) == xfer,
		    ("usb_transfer_complete: bad dequeue"));
		STAILQ_REMOVE_HEAD(&pipe->queue, next);
	}
	DPRINTFN(5,("usb_transfer_complete: repeat=%d new head=%p\n",
		    repeat, STAILQ_FIRST(&pipe->queue)));

	/* Count completed transfers. */
	++pipe->device->bus->stats.uds_requests
		[pipe->endpoint->edesc->bmAttributes & UE_XFERTYPE];

	xfer->done = 1;
	if (!xfer->status && xfer->actlen < xfer->length &&
	    !(xfer->flags & USBD_SHORT_XFER_OK)) {
		DPRINTFN(-1,("usbd_transfer_cb: short transfer %d<%d\n",
			     xfer->actlen, xfer->length));
		xfer->status = USBD_SHORT_XFER;
	}

	/*
	 * For repeat operations, call the callback first, as the xfer
	 * will not go away and the "done" method may modify it. Otherwise
	 * reverse the order in case the callback wants to free or reuse
	 * the xfer.
	 */
	if (repeat) {
		if (xfer->callback)
			xfer->callback(xfer, xfer->priv, xfer->status);
		pipe->methods->done(xfer);
	} else {
		pipe->methods->done(xfer);
		if (xfer->callback)
			xfer->callback(xfer, xfer->priv, xfer->status);
	}

	if (sync && !polling)
		wakeup(xfer);

	if (!repeat) {
		/* XXX should we stop the queue on all errors? */
		if (erred && pipe->iface != NULL)	/* not control pipe */
			pipe->running = 0;
		else
			usbd_start_next(pipe);
	}
}

/* Dequeue all pipe operations, called at splusb(). */
static usbd_status
usbd_ar_pipe(usbd_pipe_handle pipe)
{
	usbd_xfer_handle xfer;

	USBD_PIPE_LOCK_ASSERT(pipe);

	DPRINTFN(2,("usbd_ar_pipe: pipe=%p\n", pipe));
#ifdef USB_DEBUG
	if (usbdebug > 5)
		usbd_dump_queue(pipe);
#endif
	pipe->repeat = 0;
	pipe->aborting = 1;
	while ((xfer = STAILQ_FIRST(&pipe->queue)) != NULL) {
		DPRINTFN(2,("usbd_ar_pipe: pipe=%p xfer=%p (methods=%p)\n",
			    pipe, xfer, pipe->methods));
		/* Make the HC abort it (and invoke the callback). */
		pipe->methods->abort(xfer);
		KASSERT(STAILQ_FIRST(&pipe->queue) != xfer, ("usbd_ar_pipe"));
		/* XXX only for non-0 usbd_clear_endpoint_stall(pipe); */
	}
	pipe->aborting = 0;
	return (USBD_NORMAL_COMPLETION);
}

usbd_status
usbd_abort_pipe(usbd_pipe_handle pipe)
{
	usbd_status err;

#ifdef DIAGNOSTIC
	if (pipe == NULL) {
		printf("usbd_close_pipe: pipe==NULL\n");
		return (USBD_NORMAL_COMPLETION);
	}
#endif
	USBD_PIPE_LOCK(pipe);
	err = usbd_ar_pipe(pipe);
	USBD_PIPE_UNLOCK(pipe);
	return (err);
}

usbd_status
usbd_get_string(usbd_device_handle dev, int si, char *buf, size_t len)
{
	int swap = dev->quirks->uq_flags & UQ_SWAP_UNICODE;
	usb_string_descriptor_t us;
	char *s;
	int i, n;
	u_int16_t c;
	usbd_status err;
	int size;

	buf[0] = '\0';
	if (len == 0)
		return (USBD_NORMAL_COMPLETION);
	if (si == 0)
		return (USBD_INVAL);
	if (dev->quirks->uq_flags & UQ_NO_STRINGS)
		return (USBD_STALLED);
	if (dev->langid == USBD_NOLANG) {
		/* Set up default language */
		err = usbd_get_string_desc(dev, USB_LANGUAGE_TABLE, 0, &us,
		    &size);
		if (err || size < 4) {
			DPRINTFN(-1,("usbd_get_string: getting lang failed, using 0\n"));
			dev->langid = 0; /* Well, just pick something then */
		} else {
			/* Pick the first language as the default. */
			dev->langid = UGETW(us.bString[0]);
		}
	}
	err = usbd_get_string_desc(dev, si, dev->langid, &us, &size);
	if (err)
		return (err);
	s = buf;
	n = size / 2 - 1;
	for (i = 0; i < n && i < len - 1; i++) {
		c = UGETW(us.bString[i]);
		/* Convert from Unicode, handle buggy strings. */
		if ((c & 0xff00) == 0)
			*s++ = c;
		else if ((c & 0x00ff) == 0 && swap)
			*s++ = c >> 8;
		else
			*s++ = '?';
	}
	*s++ = 0;
	return (USBD_NORMAL_COMPLETION);
}

usbd_status
usbd_device2interface_handle(usbd_device_handle dev,
			     u_int8_t ifaceno, usbd_interface_handle *iface)
{
	if (dev->cdesc == NULL)
		return (USBD_NOT_CONFIGURED);
	if (ifaceno >= dev->cdesc->bNumInterface)
		return (USBD_INVAL);
	*iface = &dev->ifaces[ifaceno];
	return (USBD_NORMAL_COMPLETION);
}

usb_interface_descriptor_t *
usbd_get_interface_descriptor(usbd_interface_handle iface)
{
#ifdef DIAGNOSTIC
	if (iface == NULL) {
		printf("usbd_get_interface_descriptor: dev == NULL\n");
		return (NULL);
	}
#endif
	return (iface->idesc);
}

usbd_status
usbd_get_config_desc(usbd_device_handle dev, int confidx,
		     usb_config_descriptor_t *d)
{
	usbd_status err;

	DPRINTFN(3,("usbd_get_config_desc: confidx=%d\n", confidx));
	err = usbd_get_desc(dev, UDESC_CONFIG, confidx,
			    USB_CONFIG_DESCRIPTOR_SIZE, d);
	if (err)
		return (err);
	if (d->bDescriptorType != UDESC_CONFIG) {
		DPRINTFN(-1,("usbd_get_config_desc: confidx=%d, bad desc "
			     "len=%d type=%d\n",
			     confidx, d->bLength, d->bDescriptorType));
		return (USBD_INVAL);
	}
	return (USBD_NORMAL_COMPLETION);
}

usbd_status
usbd_get_device_status(usbd_device_handle dev, usb_status_t *st)
{
	usb_device_request_t req;

	req.bmRequestType = UT_READ_DEVICE;
	req.bRequest = UR_GET_STATUS;
	USETW(req.wValue, 0);
	USETW(req.wIndex, 0);
	USETW(req.wLength, sizeof(usb_status_t));
	return (usbd_do_request(dev, &req, st));
}

usb_device_descriptor_t *
usbd_get_device_descriptor(usbd_device_handle dev)
{
	return (&dev->ddesc);
}

usb_endpoint_descriptor_t *
usbd_interface2endpoint_descriptor(usbd_interface_handle iface, u_int8_t index)
{
	if (index >= iface->idesc->bNumEndpoints)
		return (0);
	return (iface->endpoints[index].edesc);
}

usbd_status
usbd_open_pipe_ival(usbd_interface_handle iface, u_int8_t address,
		    u_int8_t flags, usbd_pipe_handle *pipe, int ival)
{
	usbd_pipe_handle p;
	struct usbd_endpoint *ep;
	usbd_status err;
	int i;

	DPRINTFN(3,("usbd_open_pipe: iface=%p address=0x%x flags=0x%x\n",
		    iface, address, flags));

	for (i = 0; i < iface->idesc->bNumEndpoints; i++) {
		ep = &iface->endpoints[i];
		if (ep->edesc == NULL)
			return (USBD_IOERROR);
		if (ep->edesc->bEndpointAddress == address)
			goto found;
	}
	return (USBD_BAD_ADDRESS);
 found:
	if ((flags & USBD_EXCLUSIVE_USE) && ep->refcnt != 0)
		return (USBD_IN_USE);
	err = usbd_setup_pipe(iface->device, iface, ep, ival, &p);
	if (err)
		return (err);
	LIST_INSERT_HEAD(&iface->pipes, p, next);
	*pipe = p;
	return (USBD_NORMAL_COMPLETION);
}

usbd_status
usbd_open_pipe_intr(usbd_interface_handle iface, u_int8_t address,
		    u_int8_t flags, usbd_pipe_handle *pipe,
		    usbd_private_handle priv, void *buffer, u_int32_t len,
		    usbd_callback cb, int ival)
{
	usbd_status err;
	usbd_xfer_handle xfer;
	usbd_pipe_handle ipipe;

	DPRINTFN(3,("usbd_open_pipe_intr: address=0x%x flags=0x%x len=%d\n",
		    address, flags, len));

	err = usbd_open_pipe_ival(iface, address, USBD_EXCLUSIVE_USE,
				  &ipipe, ival);
	if (err)
		return (err);
	xfer = usbd_alloc_xfer(iface->device);
	if (xfer == NULL) {
		err = USBD_NOMEM;
		goto bad1;
	}
	usbd_setup_xfer(xfer, ipipe, priv, buffer, len, flags,
	    USBD_NO_TIMEOUT, cb);
	ipipe->intrxfer = xfer;
	ipipe->repeat = 1;
	err = usbd_transfer(xfer);
	*pipe = ipipe;
	if (err != USBD_IN_PROGRESS && err)
		goto bad2;
	return (USBD_NORMAL_COMPLETION);

 bad2:
	ipipe->intrxfer = NULL;
	ipipe->repeat = 0;
	usbd_free_xfer(xfer);
 bad1:
	usbd_close_pipe(ipipe);
	return (err);
}

usbd_status
usbd_clear_endpoint_stall_async(usbd_pipe_handle pipe)
{
	usbd_device_handle dev = pipe->device;
	usb_device_request_t req;
	usbd_status err;

	pipe->methods->cleartoggle(pipe);

	req.bmRequestType = UT_WRITE_ENDPOINT;
	req.bRequest = UR_CLEAR_FEATURE;
	USETW(req.wValue, UF_ENDPOINT_HALT);
	USETW(req.wIndex, pipe->endpoint->edesc->bEndpointAddress);
	USETW(req.wLength, 0);
	err = usbd_do_request_async(dev, &req, 0);
	return (err);
}

void
usbd_setup_xfer(usbd_xfer_handle xfer, usbd_pipe_handle pipe,
		usbd_private_handle priv, void *buffer, u_int32_t length,
		u_int16_t flags, u_int32_t timeout,
		usbd_callback callback)
{
	xfer->pipe = pipe;
	xfer->priv = priv;
	xfer->buffer = buffer;
	xfer->length = length;
	xfer->actlen = 0;
	xfer->flags = flags;
	xfer->timeout = timeout;
	xfer->status = USBD_NOT_STARTED;
	xfer->callback = callback;
	xfer->rqflags &= ~URQ_REQUEST;
	xfer->nframes = 0;
}

usbd_status
usbd_close_pipe(usbd_pipe_handle pipe)
{
#ifdef DIAGNOSTIC
	if (pipe == NULL) {
		printf("usbd_close_pipe: pipe==NULL\n");
		return (USBD_NORMAL_COMPLETION);
	}
#endif

	if (--pipe->refcnt != 0)
		return (USBD_NORMAL_COMPLETION);
	if (! STAILQ_EMPTY(&pipe->queue))
		return (USBD_PENDING_REQUESTS);
	LIST_REMOVE(pipe, next);
	pipe->endpoint->refcnt--;
	pipe->methods->close(pipe);
	if (pipe->intrxfer != NULL)
		usbd_free_xfer(pipe->intrxfer);
	free(pipe, M_USB);
	return (USBD_NORMAL_COMPLETION);
}

void
usbd_do_request_async_cb(usbd_xfer_handle xfer, usbd_private_handle priv,
			 usbd_status status)
{
#if defined(USB_DEBUG) || defined(DIAGNOSTIC)
	if (xfer->actlen > xfer->length)
		DPRINTF(("usbd_do_request: overrun addr=%d type=0x%02x req=0x"
			 "%02x val=%d index=%d rlen=%d length=%d actlen=%d\n",
			 xfer->pipe->device->address,
			 xfer->request.bmRequestType,
			 xfer->request.bRequest, UGETW(xfer->request.wValue),
			 UGETW(xfer->request.wIndex),
			 UGETW(xfer->request.wLength),
			 xfer->length, xfer->actlen));
#endif
	usbd_free_xfer(xfer);
}

/*
 * Execute a request without waiting for completion.
 * Can be used from interrupt context.
 */
usbd_status
usbd_do_request_async(usbd_device_handle dev, usb_device_request_t *req,
		      void *data)
{
	usbd_xfer_handle xfer;
	usbd_status err;

	xfer = usbd_alloc_xfer(dev);
	if (xfer == NULL)
		return (USBD_NOMEM);
	usbd_setup_default_xfer(xfer, dev, 0, USBD_DEFAULT_TIMEOUT, req,
	    data, UGETW(req->wLength), 0, usbd_do_request_async_cb);
	err = usbd_transfer(xfer);
	if (err != USBD_IN_PROGRESS && err) {
		usbd_free_xfer(xfer);
		return (err);
	}
	return (USBD_NORMAL_COMPLETION);
}
