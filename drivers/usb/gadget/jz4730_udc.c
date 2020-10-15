/*
 * JZ4730 USB Device Controller driver
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

/*
 * This device has ep0 and six bulk/interrupt/iso endpoints.
 *
 *  - Endpoint numbering is fixed: ep0, ep1in-int, ep2in-bulk, ep3in-bulk,
 *	  ep4in-iso, ep5out-bulk, ep6out-bulk, ep7out-iso.
 *  - Gadget drivers can choose ep maxpacket (8/16/32/64).
 *  - Just PIO mode currently.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/usb.h>
#include <linux/usb/gadget.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>

#include <asm-mips/mach-jz4730/regs.h>
#include <asm-mips/mach-jz4730/ops.h>

#include "jz4730_udc.h"

//#define DEBUG(fmt,args...) printk(KERN_DEBUG fmt , ## args)
//#define DEBUG_EP0(fmt,args...) printk(KERN_DEBUG fmt , ## args)

#ifndef DEBUG
# define DEBUG(fmt,args...) do {} while(0)
#endif
#ifndef DEBUG_EP0
# define DEBUG_EP0(fmt,args...) do {} while(0)
#endif

#define	DRIVER_DESC		"JZ4730 USB Device Controller"
#define	DRIVER_VERSION		"20 Sep 2007"

static const char driver_name [] = "jz4730_udc";
static const char driver_desc [] = DRIVER_DESC;

static unsigned int udc_debug = 0; /* 0: normal mode, 1: test udc cable type mode */

module_param(udc_debug, int, 0);
MODULE_PARM_DESC(udc_debug, "test udc cable type");

#ifdef CONFIG_JZ_UDC_HOTPLUG
extern int jz_udc_active; /* 0: No actions; 1: Have actions */
#endif

/*
 * Local declarations.
 */
static void nuke(struct jz4730_ep *, int status);
static inline void pio_irq_enable(struct jz4730_ep *ep);
static inline void pio_irq_disable(struct jz4730_ep *ep);
static void jz4730_udc_release (struct device *dev) {}
/*-------------------------------------------------------------------------*/

static int jz4730_ep_enable(struct usb_ep *_ep, 
			    const struct usb_endpoint_descriptor *desc)
{
	struct jz4730_udc *dev;
	struct jz4730_ep *ep;
	unsigned long flags;
	u32 max;

	ep = container_of(_ep, struct jz4730_ep, ep);
	if (!_ep || !desc || ep->desc
	    || desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;
	dev = ep->dev;
	if (ep == &dev->ep[0])
		return -EINVAL;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;
	if (ep->index != (desc->bEndpointAddress & 0x0f))
		return -EINVAL;

	switch (desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) {
	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		break;
	default:
		return -EINVAL;
	}

//	max = le16_to_cpu(get_unaligned(&desc->wMaxPacketSize));
	max = 64;
	ep->is_in = (USB_DIR_IN & desc->bEndpointAddress) != 0;

	spin_lock_irqsave(&ep->dev->lock, flags);

	ep->stopped = 0;
	ep->desc = desc;
	ep->ep.maxpacket = max;

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	DEBUG("enable %s %s maxpacket %u\n", ep->ep.name,
	      ep->is_in ? "IN" : "OUT", max);

	return 0;
}

static int jz4730_ep_disable(struct usb_ep *_ep)
{
	struct jz4730_ep *ep;
	struct jz4730_udc *dev;
	unsigned long flags;

	ep = container_of(_ep, struct jz4730_ep, ep);
	if (!_ep || !ep->desc)
		return -ENODEV;
	dev = ep->dev;
	if (dev->ep0state == EP0_SUSPEND)
		return -EBUSY;

	DEBUG("disable %s\n", _ep->name);

	spin_lock_irqsave(&dev->lock, flags);

	/* Nuke all pending requests */
	nuke(ep, -ESHUTDOWN);

	/* Disable ep IRQ */
	pio_irq_disable(ep);

	ep->desc = 0;
	ep->stopped = 1;

	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}

static struct usb_request *jz4730_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct jz4730_request *req;

	req = kzalloc(sizeof *req, gfp_flags);
	if (!req)
		return 0;

	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

static void jz4730_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct jz4730_request *req;

	req = container_of(_req, struct jz4730_request, req);
	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

static void *jz4730_alloc_buffer(struct usb_ep *_ep, unsigned bytes,
				 dma_addr_t *dma, gfp_t gfp_flags)
{
	void *retval;

	retval = kmalloc(bytes, gfp_flags & ~(__GFP_DMA | __GFP_HIGHMEM));
	if (retval)
		*dma = virt_to_phys(retval);
	return retval;
}

static void jz4730_free_buffer(struct usb_ep *_ep, void *buf,
			       dma_addr_t dma, unsigned bytes)
{
	kfree(buf);
}

/*
 *	done - retire a request; caller blocked irqs
 */
static void done(struct jz4730_ep *ep, struct jz4730_request *req, int status)
{
	struct jz4730_udc *dev;
	unsigned stopped = ep->stopped;

	list_del_init(&req->queue);

	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	if (status && status != -ESHUTDOWN)
		DEBUG("complete %s req %p stat %d len %u/%u\n",
		      ep->ep.name, &req->req, status,
		      req->req.actual, req->req.length);

	dev = ep->dev;

	/* don't modify queue heads during completion callback */
	ep->stopped = 1;
	spin_unlock(&dev->lock);
	req->req.complete(&ep->ep, &req->req);
	spin_lock(&dev->lock);
	ep->stopped = stopped;
}

/*-------------------------------------------------------------------------*/

static __inline__ int write_packet(struct jz4730_ep *ep,
				   struct jz4730_request *req, int max)
{
	u8 *buf;	
	int length, nlong, nbyte;
	volatile u32 *fifo = (volatile u32 *)ep->fifo;

	buf = req->req.buf + req->req.actual;
	prefetch(buf);

	length = req->req.length - req->req.actual;
	length = min(length, max);
	req->req.actual += length;

	DEBUG("Write %d (max %d), fifo %p\n", length, max, fifo);

	if (!length) {
		/* Send ZLP */
		writel(0, (unsigned int *)UDC_TXZLP);
		writel(0x12345678, (unsigned int *)fifo);
	}
	else {
		nlong = length >> 2;
		nbyte = length & 0x3;
		while (nlong--) {
			*fifo = *((u32 *)buf);
			buf += 4;
		}
		while (nbyte--) {
			*((volatile u8 *)fifo) = *buf++;
		}
	}
		
	writel(0, (unsigned int *)UDC_TXCONFIRM);

	return length;
}

static __inline__ int read_packet(struct jz4730_ep *ep, 
				  struct jz4730_request *req, int count)
{
	u8 *buf;
	int length, nlong, nbyte;
	volatile u32 *fifo = (volatile u32 *)ep->fifo;

	buf = req->req.buf + req->req.actual;
	prefetchw(buf);

	length = req->req.length - req->req.actual;
	length = min(length, count);
	req->req.actual += length;

	DEBUG("Read %d, fifo %p\n", length, fifo);

	nlong = length >> 2;
	nbyte = length & 0x3;
	while (nlong--) {
		*((u32 *)buf) = *fifo;
		buf += 4;
	}
	if (nbyte) {
		u32 data = *fifo;
		while (nbyte--) {
			*buf++ = data & 0x0ff;
			data >>= 8;
		}
	}

	REG32(UDC_RXCONFIRM);

	return length;
}

/** Write request to FIFO (max write == maxp size)
 *  Return:  0 = still running, 1 = completed, negative = errno
 */
static int write_fifo(struct jz4730_ep *ep, struct jz4730_request *req)
{
	u32 max, count;
	int is_last;

	max = ep->ep.maxpacket;

	count = write_packet(ep, req, max);

	/* last packet often short (sometimes a zlp, especially on ep0) */
	if (unlikely(count != max)) {
		is_last = 1;
	} else {
		if (likely(req->req.length != req->req.actual)
		    || req->req.zero)
			is_last = 0;
		else
			is_last = 1;
	}

	DEBUG("write %s (%d)(IN) %d bytes%s req %p %d/%d is_last %d\n",
	      ep->ep.name, ep->index, count,
	      (count != ep->ep.maxpacket) ? " (short)" : "",
	      req, req->req.actual, req->req.length, is_last);

	/* requests complete when all IN data is in the FIFO,
	 * or sometimes later, if a zlp was needed.
	 */
	if (is_last) {
		done(ep, req, 0);
		return 1;
	}

	return 0;
}

/** Read to request from FIFO (max read == bytes in fifo)
 *  Return:  0 = still running, 1 = completed, negative = errno
 */
static int read_fifo(struct jz4730_ep *ep, struct jz4730_request *req, u32 count)
{
	int is_short;

	is_short = (count < ep->ep.maxpacket);

	count = read_packet(ep, req, count);

	DEBUG("read %s %u bytes%s OUT req %p %u/%u is_short %d\n",
	      ep->ep.name, count, (count < ep->ep.maxpacket) ? "(short)" : "",
	      req, req->req.actual, req->req.length, is_short);

	/* completion */
	if (is_short || req->req.actual == req->req.length) {
		done(ep, req, 0);
		return 1;
	}

	/* finished that packet.  the next one may be waiting... */
	return 0;
}

static inline void pio_irq_enable(struct jz4730_ep *ep)
{
	switch (ep->index) {
	case 0:
		REG_UDC_EPIntMR &= ~0x1;
		break;
	case 1:
	case 2:
	case 3:
	case 4:
		REG_UDC_EPIntMR &= ~(1 << ep->index);
		break;
	case 5:
	case 6:
	case 7:
		REG_UDC_EPIntMR &= ~(1 << (ep->index + 16));
		break;
	}
}

static inline void pio_irq_disable(struct jz4730_ep *ep)
{
	switch (ep->index) {
	case 0:
		REG_UDC_EPIntMR |= 0x1;
		break;
	case 1:
	case 2:
	case 3:
	case 4:
		REG_UDC_EPIntMR |= (1 << ep->index);
		break;
	case 5:
	case 6:
	case 7:
		REG_UDC_EPIntMR |= (1 << (ep->index + 16));
		break;
	}
}

/*-------------------------------------------------------------------------*/

static int
jz4730_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct jz4730_request *req;
	struct jz4730_ep *ep;
	struct jz4730_udc *dev;
	unsigned long flags;
	int status;

	/* always require a cpu-view buffer so pio works */
	req = container_of(_req, struct jz4730_request, req);
	if (unlikely(!_req || !_req->complete 
			|| !_req->buf || !list_empty(&req->queue)))
		return -EINVAL;
	ep = container_of(_ep, struct jz4730_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->index != 0)))
		return -EINVAL;
	dev = ep->dev;
	if (unlikely(!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN))
		return -ESHUTDOWN;

	DEBUG("%s queue req %p, len %u buf %p\n",
	      _ep->name, _req, _req->length, _req->buf);

	spin_lock_irqsave(&dev->lock, flags);

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	/* for ep0 IN without premature status, zlp is required and
	 * writing EOP starts the status stage (OUT).
	 */
	if (unlikely(ep->index == 0 && ep->is_in))
		_req->zero = 1;

	/* kickstart this i/o queue? */
	status = 0;
	if (list_empty(&ep->queue) && likely(!ep->stopped)) {
		if (unlikely(ep->index == 0)) {
			pio_irq_enable(ep);
  			if (ep->irq_pending || 
  			    (REG_UDC_EPIntR & UDC_EPIntR_OUTEP0)) {
  				u32 stats, count;
  
  				stats = REG_UDC_EP0OutSR;
  				if (stats & UDC_EPSR_OUT_RCVDATA) {
  					ep->irq_pending = 0;
  					REG_UDC_EP0OutSR &= ~UDC_EPSR_OUT_MASK;
  					if (REG_UDC_EPIntR & UDC_EPIntR_OUTEP0)
  						REG_UDC_EPIntR = UDC_EPIntR_OUTEP0;
  
  					count = OUT_COUNT(stats);
  					if (read_fifo(ep, req, count) == 1)
  						req = 0;
  				}
  			}

		} else if (ep->is_in) {
			/* EP1 ~ EP4 */
			if (ep->irq_pending || 
			    (REG_UDC_EPIntR & UDC_EPIntR_INEP2)) {
				if (REG_UDC_EP2InSR & UDC_EPSR_IN) {
					ep->irq_pending = 0;
					REG_UDC_EP2InSR &= ~UDC_EPSR_IN;
					if (REG_UDC_EPIntR & UDC_EPIntR_INEP2)
						REG_UDC_EPIntR = UDC_EPIntR_INEP2;

					if (write_fifo(ep, req) == 1)
						req = 0;
				}
			}
			pio_irq_enable(ep);
		} else {
			/* EP5 ~ EP7 */
			pio_irq_enable(ep);

			if (ep->irq_pending || 
			    (REG_UDC_EPIntR & UDC_EPIntR_OUTEP5)) {
				u32 stats, count;

				stats = REG_UDC_EP5OutSR;
				if (stats & UDC_EPSR_OUT_RCVDATA) {
					ep->irq_pending = 0;
					REG_UDC_EP5OutSR &= ~UDC_EPSR_OUT_MASK;
					if (REG_UDC_EPIntR & UDC_EPIntR_OUTEP5)
						REG_UDC_EPIntR = UDC_EPIntR_OUTEP5;

					count = OUT_COUNT(stats);
					if (read_fifo(ep, req, count) == 1)
						req = 0;
				}
			}
		} 
	}

	/* pio or dma irq handler advances the queue. */
	if (likely(req != 0)) {
		list_add_tail(&req->queue, &ep->queue);
	}

	spin_unlock_irqrestore(&dev->lock, flags);

	return status;
}

/* dequeue ALL requests */
static void nuke(struct jz4730_ep *ep, int status)
{
	struct jz4730_request *req;

	if (list_empty(&ep->queue))
		return;
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct jz4730_request, queue);
		done(ep, req, status);
	}
}

/* dequeue JUST ONE request */
static int jz4730_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct jz4730_request *req;
	struct jz4730_ep *ep;
	struct jz4730_udc *dev;
	unsigned long flags;
	int stopped;

	ep = container_of(_ep, struct jz4730_ep, ep);
	if (!_ep || !_req || (!ep->desc && ep->index != 0))
		return -EINVAL;
	dev = ep->dev;
	if (!dev->driver)
		return -ESHUTDOWN;

	DEBUG("%s %s %p\n", __FUNCTION__, _ep->name,_req);

	spin_lock_irqsave(&dev->lock, flags);
	stopped = ep->stopped;
	ep->stopped = 1;

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		spin_unlock_irqrestore (&dev->lock, flags);
		return -EINVAL;
	}
	
	/* queue head may be partially complete. */
	if (ep->queue.next == &req->queue) {
		done (ep, req, -ECONNRESET);
		req = 0;
	}

	if (req)
		done (ep, req, -ECONNRESET);
	ep->stopped = stopped;

	spin_unlock_irqrestore (&ep->dev->lock, flags);
	return req ? 0 : -EOPNOTSUPP;
}

/*-------------------------------------------------------------------------*/

static void jz4730_clear_halt(struct jz4730_ep *ep)
{
	if (ep->stopped) {
		ep->stopped = 0;
	}
}

static int jz4730_set_halt(struct usb_ep *_ep, int value)
{
	struct jz4730_ep *ep;
	unsigned long flags;
	int retval = 0;

	if (!_ep)
		return -ENODEV;
	ep = container_of (_ep, struct jz4730_ep, ep);
	if (!ep->dev->driver || ep->dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;
	if (ep->desc /* not ep0 */ && (ep->desc->bmAttributes & 0x03)
						== USB_ENDPOINT_XFER_ISOC)
		return -EINVAL;

	if (ep->index == 0) {
		if (value) {
			ep->dev->ep0state = EP0_STALL;
			ep->dev->ep[0].stopped = 1;
		} else
			return -EINVAL;

	/* don't change EPxSTATUS_EP_INVALID to READY */
	} else if (!ep->desc) {
		DEBUG("%s %s inactive?\n", __FUNCTION__, ep->ep.name);
		return -EINVAL;
	}

	spin_lock_irqsave(&ep->dev->lock, flags);

	if (!list_empty(&ep->queue))
		retval = -EAGAIN;
	else if (!value)
		jz4730_clear_halt(ep);
	else {
		ep->stopped = 1;
	}

	spin_unlock_irqrestore(&ep->dev->lock, flags);
	return retval;
}

static int jz4730_fifo_status(struct usb_ep *_ep)
{
	struct jz4730_ep *ep;
	u32 size = 0;

	if (!_ep)
		return -ENODEV;
	ep = container_of(_ep, struct jz4730_ep, ep);

	/* size is only reported sanely for OUT */
	if (ep->is_in)
		return -EOPNOTSUPP;

	return size;
}

static void jz4730_fifo_flush(struct usb_ep *_ep)
{
	struct jz4730_ep *ep;

	if (!_ep)
		return;
	ep = container_of(_ep, struct jz4730_ep, ep);

	/* don't change EPxSTATUS_EP_INVALID to READY */
	if (!ep->desc && ep->index != 0) {
		return;
	}
}

static struct usb_ep_ops jz4730_ep_ops = {
	.enable		= jz4730_ep_enable,
	.disable	= jz4730_ep_disable,

	.alloc_request	= jz4730_alloc_request,
	.free_request	= jz4730_free_request,
#if 0
	.alloc_buffer	= jz4730_alloc_buffer,
	.free_buffer	= jz4730_free_buffer,
#endif
	.queue		= jz4730_queue,
	.dequeue	= jz4730_dequeue,

	.set_halt	= jz4730_set_halt,
	.fifo_status	= jz4730_fifo_status,
	.fifo_flush	= jz4730_fifo_flush,
};

/*-------------------------------------------------------------------------*/

static int jz4730_get_frame(struct usb_gadget *_gadget)
{
	return -EOPNOTSUPP;
}

static const struct usb_gadget_ops jz4730_ops = {
	.get_frame	= jz4730_get_frame,
	// no remote wakeup
	// not selfpowered
};

/*-------------------------------------------------------------------------*/

static void udc_reinit(struct jz4730_udc *dev)
{
	static char *names [] = { "ep0", "ep1in-int", "ep2in-bulk", "ep3in-bulk",
				  "ep4in-iso", "ep5out-bulk", "ep6out-bulk",
				  "ep7out-iso" };
	int i;

	INIT_LIST_HEAD (&dev->gadget.ep_list);
	dev->gadget.ep0 = &dev->ep[0].ep;
	dev->gadget.speed = USB_SPEED_UNKNOWN;
	dev->ep0state = EP0_DISCONNECT;

	for (i = 0; i < MAX_EP_NUM; i++) {
		struct jz4730_ep *ep = &dev->ep[i];

		ep->index = i;
		ep->ep.name = names[i];
		ep->fifo = ep_fifo[i];
		ep->ep.maxpacket = 64;

		ep->ep.ops = &jz4730_ep_ops;
		list_add_tail (&ep->ep.ep_list, &dev->gadget.ep_list);
		ep->dev = dev;
		INIT_LIST_HEAD(&ep->queue);

		ep->desc = 0;
		ep->stopped = 1;
		ep->irq_pending = 0;
	}

	dev->ep[0].ep.maxpacket = MAX_EP0_SIZE;
	list_del_init(&dev->ep[0].ep.ep_list);
}

/* Reset udc registers */
static void udc_reset(struct jz4730_udc *dev)
{
	REG_UDC_DevIntMR = 0x32; /* Enable RESET and SC interrupts */
	REG_UDC_EPIntMR  = 0x0;  /* Enable all EP interrupts */
	REG_UDC_DevCFGR = 0x17;
	REG_UDC_DevCR = 0x0;

	REG_UDC_EP0InCR = (0 << 4) | (1 << 1);
	REG_UDC_EP0InCR = (0 << 4);
	REG_UDC_EP1InCR = (3 << 4) | (1 << 1);
	REG_UDC_EP1InCR = (3 << 4);
	REG_UDC_EP2InCR = (2 << 4) | (1 << 1);
	REG_UDC_EP2InCR = (2 << 4);
	REG_UDC_EP3InCR = (2 << 4) | (1 << 1);
	REG_UDC_EP3InCR = (2 << 4);
	REG_UDC_EP4InCR = (1 << 4) | (1 << 1);
	REG_UDC_EP4InCR = (1 << 4);

	REG_UDC_EP0OutCR = (0 << 4);
	REG_UDC_EP5OutCR = (2 << 4);
	REG_UDC_EP6OutCR = (2 << 4);
	REG_UDC_EP7OutCR = (1 << 4);

	REG_UDC_EP0InSR = 0;
	REG_UDC_EP1InSR = 0;
	REG_UDC_EP2InSR = 0;
	REG_UDC_EP3InSR = 0;
	REG_UDC_EP4InSR = 0;
	REG_UDC_EP5OutSR = 0;
	REG_UDC_EP6OutSR = 0;
	REG_UDC_EP7OutSR = 0;

	REG_UDC_EP0InBSR = MAX_EP0_SIZE/4;
	REG_UDC_EP1InBSR = MAX_EP1_SIZE/4;
	REG_UDC_EP2InBSR = MAX_EP2_SIZE/4;
	REG_UDC_EP3InBSR = MAX_EP3_SIZE/4;
	REG_UDC_EP4InBSR = MAX_EP4_SIZE/4;

	REG_UDC_EP0InMPSR = MAX_EP0_SIZE;
	REG_UDC_EP1InMPSR = MAX_EP1_SIZE;
	REG_UDC_EP2InMPSR = MAX_EP2_SIZE;
	REG_UDC_EP3InMPSR = MAX_EP3_SIZE;
	REG_UDC_EP4InMPSR = MAX_EP4_SIZE;

	REG_UDC_EP0OutMPSR = MAX_EP0_SIZE;
	REG_UDC_EP5OutMPSR = MAX_EP5_SIZE;
	REG_UDC_EP6OutMPSR = MAX_EP6_SIZE;
	REG_UDC_EP7OutMPSR = MAX_EP7_SIZE;

	REG_UDC_EP0InfR = (MAX_EP0_SIZE << 19) | (0 << 15) | (0 << 11) | (0x1 << 7) | (0 << 5) | (0 << 4) | (0 << 0);
	REG_UDC_EP1InfR = (MAX_EP1_SIZE << 19) | (0 << 15) | (0 << 11) | (0x1 << 7) | (3 << 5) | (1 << 4) | (1 << 0);
	REG_UDC_EP2InfR = (MAX_EP2_SIZE << 19) | (0 << 15) | (0 << 11) | (0x1 << 7) | (2 << 5) | (1 << 4) | (2 << 0);
	REG_UDC_EP3InfR = (MAX_EP3_SIZE << 19) | (0 << 15) | (0 << 11) | (0x1 << 7) | (2 << 5) | (1 << 4) | (3 << 0);
	REG_UDC_EP4InfR = (MAX_EP4_SIZE << 19) | (0 << 15) | (0 << 11) | (0x1 << 7) | (1 << 5) | (1 << 4) | (4 << 0);
	REG_UDC_EP5InfR = (MAX_EP5_SIZE << 19) | (0 << 15) | (0 << 11) | (0x1 << 7) | (2 << 5) | (0 << 4) | (5 << 0);
	REG_UDC_EP6InfR = (MAX_EP6_SIZE << 19) | (0 << 15) | (0 << 11) | (0x1 << 7) | (2 << 5) | (0 << 4) | (6 << 0);
	REG_UDC_EP7InfR = (MAX_EP7_SIZE << 19) | (0 << 15) | (0 << 11) | (0x1 << 7) | (1 << 5) | (0 << 4) | (7 << 0);	

	REG_UDC_STCMAR = 0xffff;
}

static void ep0_start(struct jz4730_udc *dev)
{
	udc_reset(dev);
	udc_reinit(dev);

	/* expect ep0 requests when the host drops reset */
	dev->gadget.speed = USB_SPEED_FULL;
	dev->ep0state = EP0_IDLE;
}

static void udc_enable(struct jz4730_udc *dev)
{
	/* Enable udc and enable all interrupts */
	__intc_unmask_irq(IRQ_UDC);
	__harb_usb0_udc();

	/* start enumeration now, or after power detect irq */
	ep0_start(dev);
}

/*-------------------------------------------------------------------------*/

/* keeping it simple:
 * - one bus driver, initted first;
 * - one function driver, initted second
 */

static struct jz4730_udc	*the_controller;

/* when a driver is successfully registered, it will receive
 * control requests including set_configuration(), which enables
 * non-control requests.  then usb traffic follows until a
 * disconnect is reported.  then a host may connect again, or
 * the driver might get unbound.
 */
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct jz4730_udc	*dev = the_controller;
	int			retval;
	
	if (!driver
//			|| driver->speed != USB_SPEED_FULL
			|| !driver->bind
			|| !driver->unbind
			|| !driver->disconnect
			|| !driver->setup)
	{
	printk("\n -EINVAL");
		return -EINVAL;
	}
	if (!dev)
		return -ENODEV;

	if (dev->driver)
		return -EBUSY;

	/* hook up the driver */
	dev->driver = driver;
	retval = driver->bind(&dev->gadget);
	if (retval) {
		DEBUG("bind to driver %s --> error %d\n",
		      driver->driver.name, retval);
		dev->driver = 0;
		return retval;
	}
	/* then enable host detection and ep0; and we're ready
	 * for set_configuration as well as eventual disconnect.
	 */
	udc_enable(dev);

	DEBUG("registered gadget driver '%s'\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

static void
stop_activity(struct jz4730_udc *dev, struct usb_gadget_driver *driver)
{
	unsigned	i;

	DEBUG("%s\n", __FUNCTION__);

	if (dev->gadget.speed == USB_SPEED_UNKNOWN)
		driver = 0;

	/* disconnect gadget driver after quiesceing hw and the driver */
	udc_reset (dev);
	for (i = 0; i < MAX_EP_NUM; i++)
		nuke(&dev->ep [i], -ESHUTDOWN);
	if (driver) {
		spin_unlock(&dev->lock);
		driver->disconnect(&dev->gadget);
		spin_lock(&dev->lock);
	}

	if (dev->driver)
		udc_enable(dev);
}

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct jz4730_udc	*dev = the_controller;
	unsigned long	flags;

	/* disable UDC irq */
	__intc_mask_irq(IRQ_UDC);
	__harb_usb0_uhc();

	if (!dev)
		return -ENODEV;
	if (!driver || driver != dev->driver)
		return -EINVAL;

	spin_lock_irqsave(&dev->lock, flags);
	dev->driver = 0;
	stop_activity(dev, driver);
	spin_unlock_irqrestore(&dev->lock, flags);

	driver->unbind(&dev->gadget);

	DEBUG("unregistered driver '%s'\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

static void jz4730_epn_out(struct jz4730_udc *dev, int ep_idx, u32 count)
{
	struct jz4730_request *req;
	struct jz4730_ep *ep = &dev->ep[ep_idx];

	req = list_entry(ep->queue.next, struct jz4730_request, queue);
	read_fifo(ep, req, count);
}

static void jz4730_epn_in(struct jz4730_udc *dev, int ep_idx)
{
	struct jz4730_request *req;
	struct jz4730_ep *ep = &dev->ep[ep_idx];

	req = list_entry(ep->queue.next, struct jz4730_request, queue);
	write_fifo(ep, req);
}

/****************************************************************/
/* End Point 0 related functions                                */
/****************************************************************/

/* return:  0 = still running, 1 = completed, negative = errno */
static int write_fifo_ep0(struct jz4730_ep *ep, struct jz4730_request *req)
{
	u32 max, count;
	int is_last;

	max = ep->ep.maxpacket;

	count = write_packet(ep, req, max);

	/* last packet is usually short (or a zlp) */
	if (unlikely(count != max))
		is_last = 1;
	else {
		if (likely(req->req.length != req->req.actual) || req->req.zero)
			is_last = 0;
		else
			is_last = 1;
	}

	DEBUG_EP0("%s: wrote %s %d bytes%s %d left %p\n", __FUNCTION__,
		  ep->ep.name, count,
		  is_last ? "/L" : "", req->req.length - req->req.actual, req);

	/* requests complete when all IN data is in the FIFO */
	if (is_last) {
		done(ep, req, 0);
		return 1;
	}

	return 0;
}

/*
 * Simulate a USB_REQ_SET_CONFIGURATION to the function driver,
 * this is required to enable the endpoints of the function driver.
 * UDC should let software have the chance to handle this standard 
 * request, unfortunately UDC can't do that.
 */
static void psudo_set_config(void)
{
	struct jz4730_udc *dev = (struct jz4730_udc *) the_controller;
	struct usb_ctrlrequest ctrl;
	int tmp;

	/* SETUP packet */
	ctrl.bRequestType = 0x00;
	ctrl.bRequest = USB_REQ_SET_CONFIGURATION;
	ctrl.wValue = 1;
	ctrl.wIndex = 0;
	ctrl.wLength = 0;

	nuke(&dev->ep[0], 0);
	dev->ep[0].stopped = 0;

	if (likely(ctrl.bRequestType & USB_DIR_IN)) {
		dev->ep[0].is_in = 1;
		dev->ep0state = EP0_IN;
	} else {
		dev->ep[0].is_in = 0;
		dev->ep0state = EP0_OUT;
	}

	/* delegate everything to the gadget driver.
	 * it may respond after this irq handler returns.
	 */
	spin_unlock (&dev->lock);
	tmp = dev->driver->setup(&dev->gadget, &ctrl);
	spin_lock (&dev->lock);
	if (unlikely(tmp < 0)) {
		DEBUG_EP0("req %02x.%02x protocol STALL; err %d\n",
			  ctrl.bRequestType, ctrl.bRequest, tmp);
		dev->ep[0].stopped = 1;
		dev->ep0state = EP0_STALL;
	}
}

/* 
 * Read 8 bytes setup packet from EP0 RX buffer
 */
static void read_setup_packet(u8 *buf)
{
	u32 *tmp = (u32 *)buf;

	*tmp++ = readl((unsigned int *)RXFIFO);
	*tmp++ = readl((unsigned int *)RXFIFO);

	REG32(UDC_RXCONFIRM);
}

static void jz4730_ep0_setup(struct jz4730_udc *dev)
{
	struct jz4730_ep *ep = &dev->ep[0];
	struct usb_ctrlrequest ctrl;
	int tmp;

	/* read control req from fifo (8 bytes) */
	read_setup_packet((unsigned char *) &ctrl);

	DEBUG_EP0("SETUP %02x.%02x v%04x i%04x l%04x\n",
		  ctrl.bRequestType, ctrl.bRequest,
		  ctrl.wValue, ctrl.wIndex, ctrl.wLength);

	/* Set direction of EP0 */
	if (likely(ctrl.bRequestType & USB_DIR_IN)) {
		ep->is_in = 1;
		dev->ep0state = EP0_IN;
	} else {
		ep->is_in = 0;
		dev->ep0state = EP0_OUT;
	}

	/* Nuke all previous transfers */
	nuke(ep, 0);
	ep->stopped = 0;

	/* delegate everything to the gadget driver.
	 * it may respond after this irq handler returns.
	 */
	if (likely((u32)dev->driver)) {
		/* device-2-host (IN) or no data setup command, process immediately */
		spin_unlock(&dev->lock);
		tmp = dev->driver->setup(&dev->gadget, &ctrl);
		spin_lock(&dev->lock);

		if (unlikely(tmp < 0)) {
			/* setup processing failed, force stall */
			DEBUG_EP0("req %02x.%02x protocol STALL; err %d\n",
				  ctrl.bRequestType, ctrl.bRequest, tmp);
			dev->ep0state = EP0_STALL;
		}
	}
}

static int jz4730_ep0_in(struct jz4730_udc *dev)
{
	struct jz4730_request *req;
	struct jz4730_ep *ep = &dev->ep[0];
	int ret;

	if (list_empty(&ep->queue))
		req = 0;
	else
		req = list_entry(ep->queue.next, struct jz4730_request, queue);

	if (!req) {
		DEBUG_EP0("%s: NULL REQ\n", __FUNCTION__);
		return 0;
	}

	ret = write_fifo_ep0(ep, req);

	return ret;
}

static void jz4730_ep0_out(struct jz4730_udc *dev)
{
	u32 epsr;
  	struct jz4730_ep *ep = &dev->ep[0];

	epsr = REG_UDC_EP0OutSR;
	REG_UDC_EP0OutSR &= ~UDC_EPSR_OUT_MASK;

	if (epsr & UDC_EPSR_OUT_RCVSETUP) {
		jz4730_ep0_setup(dev);
	}
	else if (epsr & UDC_EPSR_OUT_RCVDATA) {
		u32 count = __udc_ep0out_packet_size();
		if (count == 0) {
			readl((unsigned int *)UDC_RXCONFIRM); // ack zero packet
		}
		else {
			/* EP0 OUT Data */
  			if (list_empty(&ep->queue)) {
  				ep->irq_pending = 1;
  				pio_irq_disable(ep);
  			}
  			else
  				jz4730_epn_out(dev, 0, count);

		}
	}
}

static void handle_reset_irq(struct jz4730_udc *dev)
{
	int i;

	/* clear any status */
	REG_UDC_EPIntR = 0xffffffff;
	REG_UDC_DevIntR = 0xffffffff;

	/* reset udc */
	udc_reset(dev);

	/* reset driver status */
	for (i = 0; i < MAX_EP_NUM; i++) {
		struct jz4730_ep *ep = &dev->ep[i];

		ep->irq_pending = 0;
//		nuke(ep, 0);
  		nuke(ep, -ESHUTDOWN);
	}
}

static irqreturn_t jz4730_udc_irq(int irq, void *_dev)
{
	struct jz4730_udc *dev = _dev;
	struct jz4730_ep *ep;

	u32 intr_dev, intr_ep, stats, count;

	spin_lock(&dev->lock);

	intr_dev = REG_UDC_DevIntR;
	intr_ep = REG_UDC_EPIntR;

	DEBUG("*** udc irq intr_dev=0x%x intr_ep=0x%x\n", intr_dev, intr_ep);

	if (!intr_dev && !intr_ep) {
		spin_unlock(&dev->lock);
		return IRQ_HANDLED;
	}

	if (udc_debug) {
#ifdef CONFIG_JZ_UDC_HOTPLUG
		jz_udc_active = 1;
#endif
		REG_UDC_DevIntR = intr_dev;
		REG_UDC_EPIntR = intr_ep;
		__harb_usb0_uhc();
		__intc_mask_irq(IRQ_UDC);
		spin_unlock(&dev->lock);
		return IRQ_HANDLED;
	}

	if (intr_dev) {
		if (intr_dev & UDC_DevIntR_SC) {
			psudo_set_config();
			udelay(100);
		}

		if (intr_dev & UDC_DevIntR_UR) {
#ifdef CONFIG_JZ_UDC_HOTPLUG
			jz_udc_active = 1;
#endif
			handle_reset_irq(dev);
		}

		REG_UDC_DevIntR = intr_dev;
	}

	if (intr_ep & UDC_EPIntR_OUTEP0) {
		REG_UDC_EPIntR = UDC_EPIntR_OUTEP0;
		jz4730_ep0_out(dev);
	}

	if (intr_ep & UDC_EPIntR_INEP0) {
		ep = &dev->ep[0];
		if (list_empty(&ep->queue)) {
			pio_irq_disable(ep);
		}
		else {
			stats = REG_UDC_EP0InSR;
			if (stats & UDC_EPSR_IN) {
				REG_UDC_EPIntR = UDC_EPIntR_INEP0;
				REG_UDC_EP0InSR &= ~UDC_EPSR_IN;

				jz4730_ep0_in(dev);
			}
		}
	}

	if (intr_ep & UDC_EPIntR_OUTEP5) {
		REG_UDC_EPIntR = UDC_EPIntR_OUTEP5;
		ep = &dev->ep[5];
		if (list_empty(&ep->queue)) {
			ep->irq_pending = 1;
			pio_irq_disable(ep);
		}
		else {
			stats = REG_UDC_EP5OutSR;
			if (stats & UDC_EPSR_OUT_RCVDATA) {
				REG_UDC_EP5OutSR &= ~UDC_EPSR_OUT_MASK;

				count = OUT_COUNT(stats);
				jz4730_epn_out(dev, 5, count);
			}
		}
	}

	if (intr_ep & UDC_EPIntR_INEP2) {
		ep = &dev->ep[2];
		if (list_empty(&ep->queue)) {
			ep->irq_pending = 1;
			pio_irq_disable(ep);
		}
		else {
			stats = REG_UDC_EP2InSR;
			if (stats & UDC_EPSR_IN) {
				REG_UDC_EP2InSR &= ~UDC_EPSR_IN;
				jz4730_epn_in(dev, 2);
			}
		}

		REG_UDC_EPIntR = UDC_EPIntR_INEP2;
	}

  	if (intr_ep & UDC_EPIntR_INEP1) {
  		ep = &dev->ep[1];
  		if (list_empty(&ep->queue)) {
  			ep->irq_pending = 1;
  			pio_irq_disable(ep);
  		}
  		else {
  			stats = REG_UDC_EP1InSR;
  			if (stats & UDC_EPSR_IN) {
  				REG_UDC_EP1InSR &= ~UDC_EPSR_IN;
  				jz4730_epn_in(dev, 1);
  			}
  		}
  
  		REG_UDC_EPIntR = UDC_EPIntR_INEP1;
  	}

	spin_unlock(&dev->lock);
	return IRQ_HANDLED;
}

/*-------------------------------------------------------------------------*/

static struct jz4730_udc udc_dev = {
	.usb_address = 0,

	.gadget = {
		.ops = &jz4730_ops,
		.ep0 = &udc_dev.ep[0].ep,
		.name = driver_name,
		.dev = {
			.bus_id = "gadget",
		},
	},
	/* control endpoint  no need to init here!*/ 
	/* control endpoint */
};


/* tear down the binding between this driver and the pci device */
static int jz4730_udc_remove(struct platform_device *pdev)
{
	struct jz4730_udc *dev = platform_get_drvdata(pdev);

	if (dev->driver)
		return -EBUSY;

	/* USB port0 as UHC */
	__harb_usb0_uhc();

	/* reset udc */
	udc_reset(dev);

	/* clear any status */
	REG_UDC_EPIntR = 0xffffffff;
	REG_UDC_DevIntR = 0xffffffff;

	/* disable all UDC interrupts */
	REG_UDC_DevIntMR = 0xffffffff;
	REG_UDC_EPIntMR  = 0xffffffff;

	free_irq(IRQ_UDC, dev);
	platform_set_drvdata(pdev, 0);
	device_unregister(&dev->gadget.dev);
	the_controller = 0;

	return 0;
}

static int jz4730_udc_probe(struct platform_device *pdev)
{
	struct jz4730_udc *dev = &udc_dev;
	int retval,rc;

	/* if you want to support more than one controller in a system,
	 * usb_gadget_driver_{register,unregister}() must change.
	 */
	if (the_controller) {
		printk("Check the_controller: %s\n", driver_name);
		return -EBUSY;
	}

	spin_lock_init(&dev->lock);
	device_initialize(&dev->gadget.dev);
	dev->gadget.dev.parent = &pdev->dev;      //if no,can only insmod once!!
	dev->gadget.dev.release = jz4730_udc_release;
	rc = device_register (&dev->gadget.dev);
	if (rc < 0)
		return rc;
	platform_set_drvdata(pdev, dev);

	/*
	 * Note: we just mask INTC irq but allow UDC irq.
	 * This avoid that we miss any UDC irqs.
	 */

	/* To avoid any UDC irqs here, we call cli() first */
//	cli();

	/* disable INTC irq */
	__intc_mask_irq(IRQ_UDC);

	/* init to known state, then setup irqs */
	udc_reset(dev);
	udc_reinit(dev);

	/* request UDC irq */
	if (request_irq(IRQ_UDC, jz4730_udc_irq, IRQF_DISABLED, // SA_INTERRUPT,
			driver_name, dev) != 0) {
		printk(KERN_INFO "request UDC interrupt %d failed\n", IRQ_UDC);
		retval = -EBUSY;
		goto done;
	}

	/* disable INTC irq again since request_irq has enabled it */
	__intc_mask_irq(IRQ_UDC);
	__intc_ack_irq(IRQ_UDC);

	/* Re-enable irqs */
//	sti();

	printk(KERN_INFO "%s\n", driver_desc);
	printk(KERN_INFO "version: " DRIVER_VERSION "\n");

	/* done */
	the_controller = dev;

	return 0;

done:
	if (dev)
		jz4730_udc_remove (pdev);
	return retval;
}

static struct platform_driver udc_driver = {
	.probe		= jz4730_udc_probe,
	.remove		= jz4730_udc_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= (char *) driver_name,
		.owner	= THIS_MODULE,
	},
};
static struct platform_device		the_udc_pdev = {
	.name		= (char *) driver_name,
	.id		= -1,
	.dev		= {
		.release	= jz4730_udc_release,
	},
};

/*-------------------------------------------------------------------------*/

static int __init udc_init (void)
{
        platform_driver_register(&udc_driver);
	return platform_device_register (&the_udc_pdev);
}

static void __exit udc_exit (void)
{
	platform_driver_unregister(&udc_driver);
	platform_device_unregister(&the_udc_pdev);
}

module_init(udc_init);
module_exit(udc_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Wei Jianli <jlwei@ingenic.cn>");
MODULE_LICENSE("GPL");
