/*
 * linux/drivers/usb/gadget/jz4740_udc.c
 *
 * Ingenic JZ4740 on-chip high speed USB device controller
 *
 * Copyright (C) 2006 - 2008 Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*
 * This device has ep0, two bulk-in/interrupt-in endpoints, and one bulk-out endpoint.
 *
 *  - Endpoint numbering is fixed: ep0, ep1in-int, ep2in-bulk, ep1out-bulk.
 *  - DMA works with bulk-in (channel 1) and bulk-out (channel 2) endpoints.
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
#include <linux/timer.h>
#include <linux/proc_fs.h>
#include <linux/usb.h>
#include <linux/usb/gadget.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/jzsoc.h>

#include "jz4740_udc.h"

#ifdef CONFIG_JZ_UDC_HOTPLUG
#include "udc_hotplug.c"
#endif

//#define DEBUG(fmt,args...) printk(KERN_DEBUG fmt , ## args)
//#define DEBUG(fmt,args...) printk(fmt , ## args)
//#define DEBUG_EP0(fmt,args...) printk(fmt , ## args)
//#define DEBUG_SETUP(fmt,args...) printk(fmt , ## args)

#ifndef DEBUG
# define DEBUG(fmt,args...) do {} while(0)
#endif
#ifndef DEBUG_EP0
# define NO_STATES
# define DEBUG_EP0(fmt,args...) do {} while(0)
#endif
#ifndef DEBUG_SETUP
# define DEBUG_SETUP(fmt,args...) do {} while(0)
#endif

static unsigned int use_dma = 1;   /* 1: use DMA, 0: use PIO */

module_param(use_dma, int, 0);
MODULE_PARM_DESC(use_dma, "DMA mode enable flag");


/*
 *  Local definintions.
 */

#define	DRIVER_VERSION		"13-Mar-2008"
#define	DRIVER_DESC		"JZ4740 USB Device Controller"

static const char	gadget_name [] = "jz4740_udc";

struct jz4740_udc *the_controller;

static const char driver_name [] = "jz4740_udc";
static const char driver_desc [] = DRIVER_DESC;
static const char ep0name[] = "ep0";

#ifndef NO_STATES
static char *state_names[] = {
	"WAIT_FOR_SETUP",
	"DATA_STATE_XMIT",
	"DATA_STATE_NEED_ZLP",
	"WAIT_FOR_OUT_STATUS",
	"DATA_STATE_RECV"
};
#endif
static const u8 usb_test_packet[53] = {
        /* implicit SYNC then DATA0 to start */

        /* JKJKJKJK x9 */
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        /* JJKKJJKK x8 */
        0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
        /* JJJJKKKK x8 */
        0xee, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
        /* JJJJJJJKKKKKKK x8 */
        0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        /* JJJJJJJK x8 */
        0x7f, 0xbf, 0xdf, 0xef, 0xf7, 0xfb, 0xfd,
        /* JKKKKKKK x10, JK */
        0xfc, 0x7e, 0xbf, 0xdf, 0xef, 0xf7, 0xfb, 0xfd, 0x7e

        /* implicit CRC16 then EOP to end */
};
static int test_mode = 0;
static int test_mode_nr = 0;


static void usb_write_testpacket(struct jz4740_ep *ep)
{
#if 1
	u8 *buf;
        int length, nlong, nbyte;
        volatile u32 *fifo = (volatile u32 *)ep->fifo;

	length = sizeof(usb_test_packet);
        buf = usb_test_packet;
        prefetch(buf);

        DEBUG("Write %d (max %d), fifo %p\n", length, max, fifo);

        nlong = length >> 2;
        nbyte = length & 0x3;
        while (nlong--) {
                *fifo = *((u32 *)buf);
                buf += 4;
        }
        while (nbyte--) {
                *((volatile u8 *)fifo) = *buf++;
        }
#endif
}


//#define TEST_ON_WIN7
#ifdef TEST_ON_WIN7
/* used for WIN7 OR VISTA TEST */
static unsigned int out_flag = 0;
struct timer_list irq_timer;
static void irq_timer_func(unsigned long data)
{

	if (out_flag) {
		out_flag = 0;
		printk(" PC removed???.\n");
	}
}
#endif


/*
 * Local declarations.
 */
static int jz4740_ep_enable(struct usb_ep *_ep, 
			    const struct usb_endpoint_descriptor *desc);
static int jz4740_ep_disable(struct usb_ep *_ep);
static struct usb_request *jz4740_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags);
static void jz4740_free_request(struct usb_ep *_ep, struct usb_request *_req);

static int jz4740_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags);
static int jz4740_dequeue(struct usb_ep *_ep, struct usb_request *_req);
static int jz4740_set_halt(struct usb_ep *_ep, int value);
static int jz4740_fifo_status(struct usb_ep *_ep);
static void jz4740_fifo_flush(struct usb_ep *_ep);

static void jz4740_ep0_kick(struct jz4740_udc *dev, struct jz4740_ep *ep, struct usb_request *req);
static void jz4740_handle_ep0(struct jz4740_udc *dev, u32 intr);

static void done(struct jz4740_ep *ep, struct jz4740_request *req,
		 int status);
static void pio_irq_enable(struct jz4740_ep *ep);
static void pio_irq_disable(struct jz4740_ep *ep);
static void stop_activity(struct jz4740_udc *dev,
			  struct usb_gadget_driver *driver);
static void nuke(struct jz4740_ep *ep, int status);
static void flush(struct jz4740_ep *ep);
static void udc_enable(struct jz4740_udc *dev);
static void udc_set_address(struct jz4740_udc *dev, unsigned char address);
static void jz4740_udc_release (struct device *dev) {}

extern void *dma_alloc_noncoherent(struct device *dev, size_t size,
				   dma_addr_t *dma_handle, gfp_t flag);
extern void dma_free_noncoherent(struct device *dev, size_t size,
				 void *vaddr, dma_addr_t dma_handle);

static struct usb_ep_ops jz4740_ep_ops = {
	.enable		= jz4740_ep_enable,
	.disable	= jz4740_ep_disable,

	.alloc_request	= jz4740_alloc_request,
	.free_request	= jz4740_free_request,

	.queue		= jz4740_queue,
	.dequeue	= jz4740_dequeue,

	.set_halt	= jz4740_set_halt,
	.fifo_status	= jz4740_fifo_status,
	.fifo_flush	= jz4740_fifo_flush,
};


/*-------------------------------------------------------------------------*/

/* inline functions of register read/write/set/clear  */

static __inline__ u8 usb_readb(u32 port)
{
	return *(volatile u8 *)port;
}

static __inline__ u16 usb_readw(u32 port)
{
	return *(volatile u16 *)port;
}

static __inline__ u32 usb_readl(u32 port)
{
	return *(volatile u32 *)port;
}

static __inline__ void usb_writeb(u32 port, u8 val)
{
	*(volatile u8 *)port = val;
}

static __inline__ void usb_writew(u32 port, u16 val)
{
	*(volatile u16 *)port = val;
}

static __inline__ void usb_writel(u32 port, u32 val)
{
	*(volatile u32 *)port = val;
}

static __inline__ void usb_setb(u32 port, u8 val)
{
	volatile u8 *ioport = (volatile u8 *)(port);
	*ioport = (*ioport) | val;
}

static __inline__ void usb_setw(u32 port, u16 val)
{
	volatile u16 *ioport = (volatile u16 *)(port);
	*ioport = (*ioport) | val;
}

static __inline__ void usb_setl(u32 port, u32 val)
{
	volatile u32 *ioport = (volatile u32 *)(port);
	*ioport = (*ioport) | val;
}

static __inline__ void usb_clearb(u32 port, u8 val)
{
	volatile u8 *ioport = (volatile u8 *)(port);
	*ioport = (*ioport) & ~val;
}

static __inline__ void usb_clearw(u32 port, u16 val)
{
	volatile u16 *ioport = (volatile u16 *)(port);
	*ioport = (*ioport) & ~val;
}

static __inline__ void usb_clearl(u32 port, u32 val)
{
	volatile u32 *ioport = (volatile u32 *)(port);
	*ioport = (*ioport) & ~val;
}

/*-------------------------------------------------------------------------*/

static __inline__ int write_packet(struct jz4740_ep *ep,
				   struct jz4740_request *req, int max)
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

	nlong = length >> 2;
	nbyte = length & 0x3;
	while (nlong--) {
		*fifo = *((u32 *)buf);
		buf += 4;
	}
	while (nbyte--) {
		*((volatile u8 *)fifo) = *buf++;
	}

	return length;
}

static __inline__ int read_packet(struct jz4740_ep *ep, 
				  struct jz4740_request *req, int count)
{
	u8 *buf;
	int length, nlong, nbyte;
	volatile u32 *fifo = (volatile u32 *)ep->fifo;
	char *tmp = req->req.buf;

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
	while (nbyte--) {
		*buf++ = *((volatile u8 *)fifo);
	}

#ifdef  TEST_ON_WIN7
	if (tmp[15] == 0x35)
		out_flag = 1;
#endif
	return length;
}

/*-------------------------------------------------------------------------*/

/*
 * 	udc_disable - disable USB device controller
 */
static void udc_disable(struct jz4740_udc *dev)
{
	DEBUG("%s, %p\n", __FUNCTION__, dev);

	udc_set_address(dev, 0);

	/* Disable interrupts */
	usb_writew(USB_REG_INTRINE, 0);
	usb_writew(USB_REG_INTROUTE, 0);
	usb_writeb(USB_REG_INTRUSBE, 0);

	/* Disable DMA */
	usb_writel(USB_REG_CNTL1, 0);
	usb_writel(USB_REG_CNTL2, 0);

	/* Disconnect from usb */
	usb_clearb(USB_REG_POWER, USB_POWER_SOFTCONN);

	/* Disable the USB PHY */
#ifdef CONFIG_SOC_JZ4740
	REG_CPM_SCR &= ~CPM_SCR_USBPHY_ENABLE;
#elif defined(CONFIG_SOC_JZ4750) || defined(CONFIG_SOC_JZ4750D) || defined(CONFIG_SOC_JZ4750L)
	REG_CPM_OPCR &= ~CPM_OPCR_UDCPHY_ENABLE;
#endif

	dev->ep0state = WAIT_FOR_SETUP;
	dev->gadget.speed = USB_SPEED_UNKNOWN;
}

/*
 * 	udc_reinit - initialize software state
 */
static void udc_reinit(struct jz4740_udc *dev)
{
	u32 i;

	DEBUG("%s, %p\n", __FUNCTION__, dev);

	/* device/ep0 records init */
	INIT_LIST_HEAD(&dev->gadget.ep_list);
	INIT_LIST_HEAD(&dev->gadget.ep0->ep_list);
	dev->ep0state = WAIT_FOR_SETUP;

	for (i = 0; i < UDC_MAX_ENDPOINTS; i++) {
		struct jz4740_ep *ep = &dev->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &dev->gadget.ep_list);

		INIT_LIST_HEAD(&ep->queue);
		ep->desc = 0;
		ep->stopped = 0;
		ep->pio_irqs = 0;
	}
}

/* until it's enabled, this UDC should be completely invisible
 * to any USB host.
 */
static void udc_enable(struct jz4740_udc *dev)
{
	int i;

	DEBUG("%s, %p\n", __FUNCTION__, dev);

	dev->gadget.speed = USB_SPEED_UNKNOWN;

	/* Flush FIFO for each */
	for (i = 0; i < UDC_MAX_ENDPOINTS; i++) {
		struct jz4740_ep *ep = &dev->ep[i];

		usb_set_index(ep_index(ep));
		flush(ep);
	}

	/* Set this bit to allow the UDC entering low-power mode when
	 * there are no actions on the USB bus.
	 * UDC still works during this bit was set.
	 */
	__cpm_stop_udc();

	/* Enable the USB PHY */
#ifdef CONFIG_SOC_JZ4740
	REG_CPM_SCR |= CPM_SCR_USBPHY_ENABLE;
#elif defined(CONFIG_SOC_JZ4750) || defined(CONFIG_SOC_JZ4750D) || defined(CONFIG_SOC_JZ4750L)
	REG_CPM_OPCR |= CPM_OPCR_UDCPHY_ENABLE;
#endif

	/* Disable interrupts */
	usb_writew(USB_REG_INTRINE, 0);
	usb_writew(USB_REG_INTROUTE, 0);
	usb_writeb(USB_REG_INTRUSBE, 0);

	/* Enable interrupts */
	usb_setw(USB_REG_INTRINE, USB_INTR_EP0);
	usb_setb(USB_REG_INTRUSBE, USB_INTR_RESET);
	/* Don't enable rest of the interrupts */
	/* usb_setw(USB_REG_INTRINE, USB_INTR_INEP1 | USB_INTR_INEP2);
	   usb_setw(USB_REG_INTROUTE, USB_INTR_OUTEP1); */

	/* Enable SUSPEND */
	/* usb_setb(USB_REG_POWER, USB_POWER_SUSPENDM); */

	/* Enable HS Mode */
	usb_setb(USB_REG_POWER, USB_POWER_HSENAB);

	/* Let host detect UDC:
	 * Software must write a 1 to the PMR:USB_POWER_SOFTCONN bit to turn this
	 * transistor on and pull the USBDP pin HIGH.
	 */
	usb_setb(USB_REG_POWER, USB_POWER_SOFTCONN);

	
}

/*-------------------------------------------------------------------------*/
#ifdef CONFIG_JZ_UDC_HOTPLUG
static int jz4740_udc_uh_event(struct notifier_block *n, unsigned long val, void *data)
{
	struct jz4740_udc *dev = the_controller;
	unsigned long flags;
	
	int state = *((int *)data);

	if (!dev || !dev->driver)
		return 0;

	switch (val) {
		case UH_NOTIFY_CABLE_STATE:
			switch (state) {
				case UH_CABLE_STATE_OFFLINE:
				case UH_CABLE_STATE_POWER:
					spin_lock_irqsave(&dev->lock, flags);
					stop_activity(dev, dev->driver);
					spin_unlock_irqrestore(&dev->lock, flags);

					udc_disable(dev);

					break;

				case UH_CABLE_STATE_USB:
					udc_enable(dev);

					break;
			}
	}

	return 0;
}

static struct notifier_block jz4740_udc_nb = {
	.notifier_call = jz4740_udc_uh_event,
};
#endif

/* keeping it simple:
 * - one bus driver, initted first;
 * - one function driver, initted second
 */

/*
 * Register entry point for the peripheral controller driver.
 */

int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct jz4740_udc *dev = the_controller;
	int retval;

	if (!driver
	    || !driver->bind
	    || !driver->unbind || !driver->disconnect || !driver->setup)
	{
		printk("\n-EINVAL");
		return -EINVAL;
	}
	if (!dev)
	{
		printk("\n-ENODEV");
		return -ENODEV;
	}
	if (dev->driver)
	{
		printk("\n-ENODEV");
		return -EBUSY;
	}

	/* hook up the driver */
	dev->driver = driver;
	retval = driver->bind(&dev->gadget);
	if (retval) {
		DEBUG("%s: bind to driver %s --> error %d\n", dev->gadget.name,
		            driver->driver.name, retval);
		dev->driver = 0;
		return retval;
	}

#ifdef CONFIG_JZ_UDC_HOTPLUG
	uh_register_notifier(&jz4740_udc_nb);
#else
	/* then enable host detection and ep0; and we're ready
	 * for set_configuration as well as eventual disconnect.
	 */
	udc_enable(dev);
#endif

	DEBUG("%s: registered gadget driver '%s'\n", dev->gadget.name, 
	      driver->driver.name);

	return 0;
}

EXPORT_SYMBOL(usb_gadget_register_driver);

static void stop_activity(struct jz4740_udc *dev,
			  struct usb_gadget_driver *driver)
{
	int i;

	DEBUG("%s\n", __FUNCTION__);

	/* don't disconnect drivers more than once */
	if (dev->gadget.speed == USB_SPEED_UNKNOWN)
		driver = 0;
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	/* prevent new request submissions, kill any outstanding requests  */
	for (i = 0; i < UDC_MAX_ENDPOINTS; i++) {
		struct jz4740_ep *ep = &dev->ep[i];

		ep->stopped = 1;

		usb_set_index(ep_index(ep));
		nuke(ep, -ESHUTDOWN);
	}

	/* report disconnect; the driver is already quiesced */
	if (driver) {
		spin_unlock(&dev->lock);
		driver->disconnect(&dev->gadget);
		spin_lock(&dev->lock);
	}

	/* re-init driver-visible data structures */
	udc_reinit(dev);
}

/*
 * Unregister entry point for the peripheral controller driver.
 */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct jz4740_udc *dev = the_controller;
	unsigned long flags;

	if (!dev)
		return -ENODEV;
	if (!driver || driver != dev->driver)
		return -EINVAL;

#ifdef CONFIG_JZ_UDC_HOTPLUG
	uh_unregister_notifier(&jz4740_udc_nb);
#endif

	spin_lock_irqsave(&dev->lock, flags);
	dev->driver = 0;
	stop_activity(dev, driver);
	spin_unlock_irqrestore(&dev->lock, flags);

	driver->unbind(&dev->gadget);

	udc_disable(dev);

	DEBUG("unregistered driver '%s'\n", driver->driver.name);

	return 0;
}

EXPORT_SYMBOL(usb_gadget_unregister_driver);

/*-------------------------------------------------------------------------*/

/*
 * Starting DMA using mode 1
 */
static void kick_dma(struct jz4740_ep *ep, struct jz4740_request *req)
{
	u32 count = req->req.length;
	u32 physaddr = virt_to_phys((void *)req->req.buf);

	usb_set_index(ep_index(ep));
	if (ep_is_in(ep)) { /* Bulk-IN transfer using DMA channel 1 */
		ep->reg_addr = USB_REG_ADDR1;

		dma_cache_wback_inv((unsigned long)req->req.buf, count);

		pio_irq_enable(ep);

		usb_writeb(USB_REG_INCSRH,
			   USB_INCSRH_DMAREQENAB | USB_INCSRH_AUTOSET | USB_INCSRH_DMAREQMODE);

		usb_writel(USB_REG_ADDR1, physaddr);
		usb_writel(USB_REG_COUNT1, count);
		usb_writel(USB_REG_CNTL1, USB_CNTL_ENA | USB_CNTL_DIR_IN | USB_CNTL_MODE_1 | 
			   USB_CNTL_INTR_EN | USB_CNTL_BURST_16 | USB_CNTL_EP(ep_index(ep)));
	}
	else { /* Bulk-OUT transfer using DMA channel 2 */
		ep->reg_addr = USB_REG_ADDR2;

		dma_cache_wback_inv((unsigned long)req->req.buf, count);

		pio_irq_enable(ep);

		usb_setb(USB_REG_OUTCSRH,
			 USB_OUTCSRH_DMAREQENAB | USB_OUTCSRH_AUTOCLR | USB_OUTCSRH_DMAREQMODE);

		usb_writel(USB_REG_ADDR2, physaddr);
		usb_writel(USB_REG_COUNT2, count);
		usb_writel(USB_REG_CNTL2, USB_CNTL_ENA | USB_CNTL_MODE_1 | 
			   USB_CNTL_INTR_EN | USB_CNTL_BURST_16 | USB_CNTL_EP(ep_index(ep)));
	}
}

/*-------------------------------------------------------------------------*/

/** Write request to FIFO (max write == maxp size)
 *  Return:  0 = still running, 1 = completed, negative = errno
 *  NOTE: INDEX register must be set for EP
 */
static int write_fifo(struct jz4740_ep *ep, struct jz4740_request *req)
{
	u32 max, csr;
	u32 physaddr = virt_to_phys((void *)req->req.buf);

	max = le16_to_cpu(ep->desc->wMaxPacketSize);

	if (use_dma) {
		u32 dma_count;

		/* DMA interrupt generated due to the last packet loaded into the FIFO */

		dma_count = usb_readl(ep->reg_addr) - physaddr;
		req->req.actual += dma_count;

		if (dma_count % max) {
			/* If the last packet is less than MAXP, set INPKTRDY manually */
			usb_setb(ep->csr, USB_INCSR_INPKTRDY);
		}

		done(ep, req, 0);
		if (list_empty(&ep->queue)) {
			pio_irq_disable(ep);
			return 1;
		}
		else {
			/* advance the request queue */
			req = list_entry(ep->queue.next, struct jz4740_request, queue);
			kick_dma(ep, req);
			return 0;
		}
	}

	/*
	 * PIO mode handling starts here ...
	 */

	csr = usb_readb(ep->csr);

	if (!(csr & USB_INCSR_FFNOTEMPT)) {
		unsigned count;
		int is_last, is_short;

		count = write_packet(ep, req, max);
		usb_setb(ep->csr, USB_INCSR_INPKTRDY);

		/* last packet is usually short (or a zlp) */
		if (unlikely(count != max))
			is_last = is_short = 1;
		else {
			if (likely(req->req.length != req->req.actual)
			    || req->req.zero)
				is_last = 0;
			else
				is_last = 1;
			/* interrupt/iso maxpacket may not fill the fifo */
			is_short = unlikely(max < ep_maxpacket(ep));
		}

		DEBUG("%s: wrote %s %d bytes%s%s %d left %p\n", __FUNCTION__,
		      ep->ep.name, count,
		      is_last ? "/L" : "", is_short ? "/S" : "",
		      req->req.length - req->req.actual, req);

		/* requests complete when all IN data is in the FIFO */
		if (is_last) {
			done(ep, req, 0);
			if (list_empty(&ep->queue)) {
				pio_irq_disable(ep);
			}
			return 1;
		}
	} else {
		DEBUG("Hmm.. %d ep FIFO is not empty!\n", ep_index(ep));
	}

	return 0;
}

/** Read to request from FIFO (max read == bytes in fifo)
 *  Return:  0 = still running, 1 = completed, negative = errno
 *  NOTE: INDEX register must be set for EP
 */
static int read_fifo(struct jz4740_ep *ep, struct jz4740_request *req)
{
	u32 csr;
	unsigned count, is_short;
	u32 physaddr = virt_to_phys((void *)req->req.buf);

	if (use_dma) {
		u32 dma_count;

		/* DMA interrupt generated due to a packet less than MAXP loaded into the FIFO */
		dma_count = usb_readl(ep->reg_addr) - physaddr;
		req->req.actual += dma_count;
		
		/* Disable interrupt and DMA */
		pio_irq_disable(ep);
		usb_writel(USB_REG_CNTL2, 0);

		/* Read all bytes from this packet */
		count = usb_readw(USB_REG_OUTCOUNT);
		count = read_packet(ep, req, count);

		if (count) {
			/* If the last packet is greater than zero, clear OUTPKTRDY manually */
			usb_clearb(ep->csr, USB_OUTCSR_OUTPKTRDY);
		}

		done(ep, req, 0);

		if (!list_empty(&ep->queue)) {
			/* advance the request queue */
			req = list_entry(ep->queue.next, struct jz4740_request, queue);
			kick_dma(ep, req);
		}

		return 1;
	}

	/*
	 * PIO mode handling starts here ...
	 */

	/* make sure there's a packet in the FIFO. */
	csr = usb_readb(ep->csr);
	if (!(csr & USB_OUTCSR_OUTPKTRDY)) {
		DEBUG("%s: Packet NOT ready!\n", __FUNCTION__);
		return -EINVAL;
	}

	/* read all bytes from this packet */
	count = usb_readw(USB_REG_OUTCOUNT);

	is_short = (count < ep->ep.maxpacket);

	count = read_packet(ep, req, count);

	DEBUG("read %s %02x, %d bytes%s req %p %d/%d\n",
	      ep->ep.name, csr, count,
	      is_short ? "/S" : "", req, req->req.actual, req->req.length);

	/* Clear OutPktRdy */
	usb_clearb(ep->csr, USB_OUTCSR_OUTPKTRDY);

	/* completion */
	if (is_short || req->req.actual == req->req.length) {
		done(ep, req, 0);

		if (list_empty(&ep->queue))
			pio_irq_disable(ep);
		return 1;
	}

	/* finished that packet.  the next one may be waiting... */
	return 0;
}

/*
 *	done - retire a request; caller blocked irqs
 *  INDEX register is preserved to keep same
 */
static void done(struct jz4740_ep *ep, struct jz4740_request *req, int status)
{
	unsigned int stopped = ep->stopped;
	u32 index;

	DEBUG("%s, %p\n", __FUNCTION__, ep);
	list_del_init(&req->queue);

	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	if (status && status != -ESHUTDOWN)
		DEBUG("complete %s req %p stat %d len %u/%u\n",
		      ep->ep.name, &req->req, status,
		      req->req.actual, req->req.length);

	/* don't modify queue heads during completion callback */
	ep->stopped = 1;
	/* Read current index (completion may modify it) */
	index = usb_readb(USB_REG_INDEX);

	spin_unlock(&ep->dev->lock);
	req->req.complete(&ep->ep, &req->req);
	spin_lock(&ep->dev->lock);

	/* Restore index */
	usb_set_index(index);
	ep->stopped = stopped;
}

/** Enable EP interrupt */
static void pio_irq_enable(struct jz4740_ep *ep)
{
	DEBUG("%s: EP%d %s\n", __FUNCTION__, ep_index(ep), ep_is_in(ep) ? "IN": "OUT");

	if (ep_is_in(ep)) {
		switch (ep_index(ep)) {
		case 1:
			usb_setw(USB_REG_INTRINE, USB_INTR_INEP1);
			break;
		case 2:
			usb_setw(USB_REG_INTRINE, USB_INTR_INEP2);
			break;
		default:
			DEBUG("Unknown endpoint: %d\n", ep_index(ep));
			break;
		}
	}
	else {
		switch (ep_index(ep)) {
		case 1:
			usb_setw(USB_REG_INTROUTE, USB_INTR_OUTEP1);
			break;
		default:
			DEBUG("Unknown endpoint: %d\n", ep_index(ep));
			break;
		}
	}
}

/** Disable EP interrupt */
static void pio_irq_disable(struct jz4740_ep *ep)
{
	DEBUG("%s: EP%d %s\n", __FUNCTION__, ep_index(ep), ep_is_in(ep) ? "IN": "OUT");

	if (ep_is_in(ep)) {
		switch (ep_index(ep)) {
		case 1:
			usb_clearw(USB_REG_INTRINE, USB_INTR_INEP1);
			break;
		case 2:
			usb_clearw(USB_REG_INTRINE, USB_INTR_INEP2);
			break;
		default:
			DEBUG("Unknown endpoint: %d\n", ep_index(ep));
			break;
		}
	}
	else {
		switch (ep_index(ep)) {
		case 1:
			usb_clearw(USB_REG_INTROUTE, USB_INTR_OUTEP1);
			break;
		default:
			DEBUG("Unknown endpoint: %d\n", ep_index(ep));
			break;
		}
	}
}

/*
 * 	nuke - dequeue ALL requests
 */
static void nuke(struct jz4740_ep *ep, int status)
{
	struct jz4740_request *req;

	DEBUG("%s, %p\n", __FUNCTION__, ep);

	/* Flush FIFO */
	flush(ep);

	/* called with irqs blocked */
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct jz4740_request, queue);
		done(ep, req, status);
	}

	/* Disable IRQ if EP is enabled (has descriptor) */
	if (ep->desc)
		pio_irq_disable(ep);
}

/** Flush EP FIFO
 * NOTE: INDEX register must be set before this call
 */
static void flush(struct jz4740_ep *ep)
{
	DEBUG("%s, %p\n", __FUNCTION__, ep);

	switch (ep->ep_type) {
	case ep_control:
		break;

	case ep_bulk_in:
	case ep_interrupt:
		usb_setb(ep->csr, USB_INCSR_FF);
		break;

	case ep_bulk_out:
		usb_setb(ep->csr, USB_OUTCSR_FF);
		break;
	}
}

/**
 * jz4740_in_epn - handle IN interrupt
 */
static void jz4740_in_epn(struct jz4740_udc *dev, u32 ep_idx, u32 intr)
{
	u32 csr;
	struct jz4740_ep *ep = &dev->ep[ep_idx + 1];
	struct jz4740_request *req;

	usb_set_index(ep_index(ep));

	csr = usb_readb(ep->csr);
	DEBUG("%s: %d, csr %x\n", __FUNCTION__, ep_idx, csr);

	if (csr & USB_INCSR_SENTSTALL) {
		DEBUG("USB_INCSR_SENTSTALL\n");
		usb_clearb(ep->csr, USB_INCSR_SENTSTALL);
		return;
	}

	if (!ep->desc) {
		DEBUG("%s: NO EP DESC\n", __FUNCTION__);
		return;
	}

	if (list_empty(&ep->queue))
		req = 0;
	else
		req = list_entry(ep->queue.next, struct jz4740_request, queue);

	DEBUG("req: %p\n", req);

	if (!req)
		return;

	write_fifo(ep, req);
}

/*
 * Bulk OUT (recv)
 */
static void jz4740_out_epn(struct jz4740_udc *dev, u32 ep_idx, u32 intr)
{
	struct jz4740_ep *ep = &dev->ep[ep_idx];
	struct jz4740_request *req;

	DEBUG("%s: %d\n", __FUNCTION__, ep_idx);

	usb_set_index(ep_index(ep));
	if (ep->desc) {
		u32 csr;

		if (use_dma) {
			/* DMA starts here ... */
			if (list_empty(&ep->queue))
				req = 0;
			else
				req = list_entry(ep->queue.next, struct jz4740_request, queue);

			if (req)
				read_fifo(ep, req);
			return;
		}

		/*
		 * PIO mode starts here ...
		 */

		while ((csr = usb_readb(ep->csr)) & 
		       (USB_OUTCSR_OUTPKTRDY | USB_OUTCSR_SENTSTALL)) {
			DEBUG("%s: %x\n", __FUNCTION__, csr);

			if (csr & USB_OUTCSR_SENTSTALL) {
				DEBUG("%s: stall sent, flush fifo\n",
				      __FUNCTION__);
				/* usb_set(USB_OUT_CSR1_FIFO_FLUSH, ep->csr1); */
				flush(ep);
			} else if (csr & USB_OUTCSR_OUTPKTRDY) {
				if (list_empty(&ep->queue))
					req = 0;
				else
					req =
						list_entry(ep->queue.next,
							   struct jz4740_request,
							   queue);

				if (!req) {
					DEBUG("%s: NULL REQ %d\n",
					      __FUNCTION__, ep_idx);
					break;
				} else {
					read_fifo(ep, req);
				}
			}
		}
	} else {
		/* Throw packet away.. */
		printk("%s: ep %p ep_indx %d No descriptor?!?\n", __FUNCTION__, ep, ep_idx);
		flush(ep);
	}
}

static int jz4740_ep_enable(struct usb_ep *_ep,
			    const struct usb_endpoint_descriptor *desc)
{
	struct jz4740_ep *ep;
	struct jz4740_udc *dev;
	unsigned long flags;
	u32 max, csrh = 0;

	ep = container_of(_ep, struct jz4740_ep, ep);
	if (!_ep || !desc || ep->desc || _ep->name == ep0name
	    || desc->bDescriptorType != USB_DT_ENDPOINT
	    || ep->bEndpointAddress != desc->bEndpointAddress) {
		DEBUG("%s, bad ep or descriptor\n", __FUNCTION__);
		return -EINVAL;
	}

	/* xfer types must match, except that interrupt ~= bulk */
	if (ep->bmAttributes != desc->bmAttributes
	    && ep->bmAttributes != USB_ENDPOINT_XFER_BULK
	    && desc->bmAttributes != USB_ENDPOINT_XFER_INT) {
		DEBUG("%s, %s type mismatch\n", __FUNCTION__, _ep->name);
		return -EINVAL;
	}

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		DEBUG("%s, bogus device state\n", __FUNCTION__);
		return -ESHUTDOWN;
	}

	max = le16_to_cpu(desc->wMaxPacketSize);

	/* Configure the endpoint */
	usb_set_index(desc->bEndpointAddress & 0x0F);
	if (ep_is_in(ep)) {
		usb_writew(USB_REG_INMAXP, max);
		switch (desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) {
		case USB_ENDPOINT_XFER_BULK:
		case USB_ENDPOINT_XFER_INT:
			csrh &= ~USB_INCSRH_ISO;
			break;
		case USB_ENDPOINT_XFER_ISOC:
			csrh |= USB_INCSRH_ISO;
			break;
		}
		usb_writeb(USB_REG_INCSRH, csrh);
	}
	else {
		usb_writew(USB_REG_OUTMAXP, max);
		switch (desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) {
		case USB_ENDPOINT_XFER_BULK:
			 csrh &= ~USB_OUTCSRH_ISO;
			break;
		case USB_ENDPOINT_XFER_INT:
			csrh &= ~USB_OUTCSRH_ISO;
			csrh |= USB_OUTCSRH_DNYT;
			break;
		case USB_ENDPOINT_XFER_ISOC:
			csrh |= USB_OUTCSRH_ISO;
			break;
		}
		usb_writeb(USB_REG_OUTCSRH, csrh);
	}

	spin_lock_irqsave(&ep->dev->lock, flags);

	ep->stopped = 0;
	ep->desc = desc;
	ep->pio_irqs = 0;
	ep->ep.maxpacket = max;

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	/* Reset halt state (does flush) */
	jz4740_set_halt(_ep, 0);

	DEBUG("%s: enabled %s\n", __FUNCTION__, _ep->name);

	return 0;
}

/** Disable EP
 *  NOTE: Sets INDEX register
 */
static int jz4740_ep_disable(struct usb_ep *_ep)
{
	struct jz4740_ep *ep;
	unsigned long flags;

	DEBUG("%s, %p\n", __FUNCTION__, _ep);

	ep = container_of(_ep, struct jz4740_ep, ep);
	if (!_ep || !ep->desc) {
		DEBUG("%s, %s not enabled\n", __FUNCTION__,
		      _ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	spin_lock_irqsave(&ep->dev->lock, flags);

	usb_set_index(ep_index(ep));

	/* Nuke all pending requests (does flush) */
	nuke(ep, -ESHUTDOWN);

	/* Disable ep IRQ */
	pio_irq_disable(ep);

	ep->desc = 0;
	ep->stopped = 1;

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	DEBUG("%s: disabled %s\n", __FUNCTION__, _ep->name);
	return 0;
}

static struct usb_request *jz4740_alloc_request(struct usb_ep *ep, gfp_t gfp_flags)
{
	struct jz4740_request *req;

	DEBUG("%s, %p\n", __FUNCTION__, ep);

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req)
		return 0;

	INIT_LIST_HEAD(&req->queue);

	return &req->req;
}

static void jz4740_free_request(struct usb_ep *ep, struct usb_request *_req)
{
	struct jz4740_request *req;

	DEBUG("%s, %p\n", __FUNCTION__, ep);

	req = container_of(_req, struct jz4740_request, req);
	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

/*--------------------------------------------------------------------*/

/** Queue one request
 *  Kickstart transfer if needed
 *  NOTE: Sets INDEX register
 */
static int jz4740_queue(struct usb_ep *_ep, struct usb_request *_req,
			gfp_t gfp_flags)
{
	struct jz4740_request *req;
	struct jz4740_ep *ep;
	struct jz4740_udc *dev;
	unsigned long flags;

	DEBUG("%s, %p\n", __FUNCTION__, _ep);

	req = container_of(_req, struct jz4740_request, req);
	if (unlikely
	    (!_req || !_req->complete || !_req->buf
	     || !list_empty(&req->queue))) {
		DEBUG("%s, bad params\n", __FUNCTION__);
		return -EINVAL;
	}

	ep = container_of(_ep, struct jz4740_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		DEBUG("%s, bad ep\n", __FUNCTION__);
		return -EINVAL;
	}

	dev = ep->dev;
	if (unlikely(!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)) {
		DEBUG("%s, bogus device state %p\n", __FUNCTION__, dev->driver);
		return -ESHUTDOWN;
	}

	DEBUG("%s queue req %p, len %d buf %p\n", _ep->name, _req, _req->length,
	      _req->buf);

	spin_lock_irqsave(&dev->lock, flags);

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	/* kickstart this i/o queue? */
	DEBUG("Add to %d Q %d %d\n", ep_index(ep), list_empty(&ep->queue),
	      ep->stopped);
	if (list_empty(&ep->queue) && likely(!ep->stopped)) {
		u32 csr;

		if (unlikely(ep_index(ep) == 0)) {
			/* EP0 */
			usb_set_index(0);
			list_add_tail(&req->queue, &ep->queue);
			jz4740_ep0_kick(dev, ep, _req);
			goto done;
		} else if (use_dma) {
			/* DMA */
			kick_dma(ep, req);
		}
		/* PIO */
		else if (ep_is_in(ep)) {
			/* EP1 & EP2 */
			usb_set_index(ep_index(ep));
			csr = usb_readb(ep->csr);
			pio_irq_enable(ep);
			if (!(csr & USB_INCSR_FFNOTEMPT)) {
				if (write_fifo(ep, req) == 1)
					req = 0;
			}
		} else {
			/* EP1 */
			usb_set_index(ep_index(ep));
			csr = usb_readb(ep->csr);
			pio_irq_enable(ep);
			if (csr & USB_OUTCSR_OUTPKTRDY) {
				if (read_fifo(ep, req) == 1)
					req = 0;
			}
		}
	}

	/* pio or dma irq handler advances the queue. */
	if (likely(req != 0))
		list_add_tail(&req->queue, &ep->queue);
done:
	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}

/* dequeue JUST ONE request */
static int jz4740_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct jz4740_ep *ep;
	struct jz4740_request *req;
	unsigned long flags;

	DEBUG("%s, %p\n", __FUNCTION__, _ep);

	ep = container_of(_ep, struct jz4740_ep, ep);
	if (!_ep || ep->ep.name == ep0name)
		return -EINVAL;

	spin_lock_irqsave(&ep->dev->lock, flags);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		spin_unlock_irqrestore(&ep->dev->lock, flags);
		return -EINVAL;
	}

	done(ep, req, -ECONNRESET);

	spin_unlock_irqrestore(&ep->dev->lock, flags);
	return 0;
}

/** Halt specific EP
 *  Return 0 if success
 *  NOTE: Sets INDEX register to EP !
 */
static int jz4740_set_halt(struct usb_ep *_ep, int value)
{
	struct jz4740_ep *ep;
	unsigned long flags;

	ep = container_of(_ep, struct jz4740_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		DEBUG("%s, bad ep\n", __FUNCTION__);
		return -EINVAL;
	}

	usb_set_index(ep_index(ep));

	DEBUG("%s, ep %d, val %d\n", __FUNCTION__, ep_index(ep), value);

	spin_lock_irqsave(&ep->dev->lock, flags);

	if (ep_index(ep) == 0) {
		/* EP0 */
		usb_setb(USB_REG_CSR0, USB_CSR0_SENDSTALL);
	} else if (ep_is_in(ep)) {
		u32 csr = usb_readb(ep->csr);
		if (value && ((csr & USB_INCSR_FFNOTEMPT)
			      || !list_empty(&ep->queue))) {
			/*
			 * Attempts to halt IN endpoints will fail (returning -EAGAIN)
			 * if any transfer requests are still queued, or if the controller
			 * FIFO still holds bytes that the host hasnt collected.
			 */
			spin_unlock_irqrestore(&ep->dev->lock, flags);
			DEBUG
			    ("Attempt to halt IN endpoint failed (returning -EAGAIN) %d %d\n",
			     (csr & USB_INCSR_FFNOTEMPT),
			     !list_empty(&ep->queue));
			return -EAGAIN;
		}
		flush(ep);
		if (value) {
			usb_setb(ep->csr, USB_INCSR_SENDSTALL);
		}
		else {
			usb_clearb(ep->csr, USB_INCSR_SENDSTALL);
			usb_setb(ep->csr, USB_INCSR_CDT);
		}
	} else {

		flush(ep);
		if (value) {
			usb_setb(ep->csr, USB_OUTCSR_SENDSTALL);
		}
		else {
			usb_clearb(ep->csr, USB_OUTCSR_SENDSTALL);
			usb_setb(ep->csr, USB_OUTCSR_CDT);
		}
	}

	if (value) {
		ep->stopped = 1;
	} else {
		ep->stopped = 0;
	}

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	DEBUG("%s %s halted\n", _ep->name, value == 0 ? "NOT" : "IS");

	return 0;
}

/** Return bytes in EP FIFO
 *  NOTE: Sets INDEX register to EP
 */
static int jz4740_fifo_status(struct usb_ep *_ep)
{
	u32 csr;
	int count = 0;
	struct jz4740_ep *ep;

	ep = container_of(_ep, struct jz4740_ep, ep);
	if (!_ep) {
		DEBUG("%s, bad ep\n", __FUNCTION__);
		return -ENODEV;
	}

	DEBUG("%s, %d\n", __FUNCTION__, ep_index(ep));

	/* LPD can't report unclaimed bytes from IN fifos */
	if (ep_is_in(ep))
		return -EOPNOTSUPP;

	usb_set_index(ep_index(ep));

	csr = usb_readb(ep->csr);
	if (ep->dev->gadget.speed != USB_SPEED_UNKNOWN ||
	    csr & 0x1) {
		count = usb_readw(USB_REG_OUTCOUNT);
	}

	return count;
}

/** Flush EP FIFO
 *  NOTE: Sets INDEX register to EP
 */
static void jz4740_fifo_flush(struct usb_ep *_ep)
{
	struct jz4740_ep *ep;

	ep = container_of(_ep, struct jz4740_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		DEBUG("%s, bad ep\n", __FUNCTION__);
		return;
	}

	usb_set_index(ep_index(ep));
	flush(ep);
}

/****************************************************************/
/* End Point 0 related functions                                */
/****************************************************************/

/* return:  0 = still running, 1 = completed, negative = errno */
static int write_fifo_ep0(struct jz4740_ep *ep, struct jz4740_request *req)
{
	u32 max;
	unsigned count;
	int is_last;

	max = ep_maxpacket(ep);

	if (req->req.length == 0) {
		is_last = 1;
		goto done;
	}
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

done:
	/* requests complete when all IN data is in the FIFO */
	if (is_last) {
		done(ep, req, 0);
		return 1;
	}

	return 0;
}

static __inline__ int jz4740_fifo_read(struct jz4740_ep *ep,
				       unsigned char *cp, int max)
{
	int bytes;
	int count = usb_readw(USB_REG_OUTCOUNT);
	volatile u8 *fifo = (volatile u8 *)ep->fifo;

	if (count > max)
		count = max;
	bytes = count;
	while (count--)
		*cp++ = *fifo;
	return bytes;
}

static __inline__ void jz4740_fifo_write(struct jz4740_ep *ep,
					 unsigned char *cp, int count)
{
	volatile u8 *fifo = (volatile u8 *)ep->fifo;
	DEBUG_EP0("fifo_write: %d %d\n", ep_index(ep), count);
	while (count--)
		*fifo = *cp++;
}

static int read_fifo_ep0(struct jz4740_ep *ep, struct jz4740_request *req)
{
	u32 csr;
	u8 *buf;
	unsigned bufferspace, count, is_short = 0;
	volatile u8 *fifo = (volatile u8 *)ep->fifo;

	DEBUG_EP0("%s\n", __FUNCTION__);

	csr = usb_readb(USB_REG_CSR0);
	if (!(csr & USB_CSR0_OUTPKTRDY))
		return 0;

	if (req->req.length == 0)
		goto done;
	buf = req->req.buf + req->req.actual;
	prefetchw(buf);
	bufferspace = req->req.length - req->req.actual;

	/* read all bytes from this packet */
	if (likely(csr & USB_CSR0_OUTPKTRDY)) {
		count = usb_readw(USB_REG_OUTCOUNT);
		req->req.actual += min(count, bufferspace);
	} else			/* zlp */
		count = 0;

	is_short = (count < ep->ep.maxpacket);
	DEBUG_EP0("read %s %02x, %d bytes%s req %p %d/%d\n",
		  ep->ep.name, csr, count,
		  is_short ? "/S" : "", req, req->req.actual, req->req.length);

	while (likely(count-- != 0)) {
		u8 byte = (u8) (*fifo & 0xff);

		if (unlikely(bufferspace == 0)) {
			/* this happens when the driver's buffer
			 * is smaller than what the host sent.
			 * discard the extra data.
			 */
			if (req->req.status != -EOVERFLOW)
				DEBUG_EP0("%s overflow %d\n", ep->ep.name,
					  count);
			req->req.status = -EOVERFLOW;
		} else {
			*buf++ = byte;
			bufferspace--;
		}
	}
done:

	/* completion */
	if (is_short || req->req.actual == req->req.length) {
		done(ep, req, 0);
		return 1;
	}

	/* finished that packet.  the next one may be waiting... */
	return 0;
}

/**
 * udc_set_address - set the USB address for this device
 * @address:
 *
 * Called from control endpoint function after it decodes a set address setup packet.
 */
static void udc_set_address(struct jz4740_udc *dev, unsigned char address)
{
	DEBUG_EP0("%s: %d\n", __FUNCTION__, address);

	dev->usb_address = address;
	usb_writeb(USB_REG_FADDR, address);
}

/*
 * DATA_STATE_RECV (USB_CSR0_OUTPKTRDY)
 *      - if error
 *              set USB_CSR0_SVDOUTPKTRDY | USB_CSR0_DATAEND | USB_CSR0_SENDSTALL bits
 *      - else
 *              set USB_CSR0_SVDOUTPKTRDY bit
 				if last set USB_CSR0_DATAEND bit
 */
static void jz4740_ep0_out(struct jz4740_udc *dev, u32 csr)
{
	struct jz4740_request *req;
	struct jz4740_ep *ep = &dev->ep[0];
	int ret;

	DEBUG_EP0("%s: %x\n", __FUNCTION__, csr);

	if (list_empty(&ep->queue))
		req = 0;
	else
		req = list_entry(ep->queue.next, struct jz4740_request, queue);

	if (req) {
		ret = read_fifo_ep0(ep, req);
		if (ret) {
			/* Done! */
			DEBUG_EP0("%s: finished, waiting for status\n",
				  __FUNCTION__);
			usb_setb(USB_REG_CSR0, (USB_CSR0_SVDOUTPKTRDY | USB_CSR0_DATAEND));
			dev->ep0state = WAIT_FOR_SETUP;
		} else {
			/* Not done yet.. */
			DEBUG_EP0("%s: not finished\n", __FUNCTION__);
			usb_setb(USB_REG_CSR0, USB_CSR0_SVDOUTPKTRDY);
		}
	} else {
		DEBUG_EP0("NO REQ??!\n");
	}
}

/*
 * DATA_STATE_XMIT
 */
static int jz4740_ep0_in(struct jz4740_udc *dev, u32 csr)
{
	struct jz4740_request *req;
	struct jz4740_ep *ep = &dev->ep[0];
	int ret, need_zlp = 0;

	DEBUG_EP0("%s: %x\n", __FUNCTION__, csr);

	if (list_empty(&ep->queue))
		req = 0;
	else
		req = list_entry(ep->queue.next, struct jz4740_request, queue);

	if (!req) {
		DEBUG_EP0("%s: NULL REQ\n", __FUNCTION__);
		return 0;
	}


	if (req->req.length - req->req.actual == EP0_MAXPACKETSIZE) {
		/* Next write will end with the packet size, */
		/* so we need zero-length-packet */
		need_zlp = 1;
	}

	ret = write_fifo_ep0(ep, req);

	if (ret == 1 && !need_zlp) {
		/* Last packet */
		DEBUG_EP0("%s: finished, waiting for status\n", __FUNCTION__);

		usb_setb(USB_REG_CSR0, (USB_CSR0_INPKTRDY | USB_CSR0_DATAEND));
		dev->ep0state = WAIT_FOR_SETUP;
	} else {
		DEBUG_EP0("%s: not finished\n", __FUNCTION__);
		usb_setb(USB_REG_CSR0, USB_CSR0_INPKTRDY);
	}

	if (need_zlp) {
		DEBUG_EP0("%s: Need ZLP!\n", __FUNCTION__);
		usb_setb(USB_REG_CSR0, USB_CSR0_INPKTRDY);
		dev->ep0state = DATA_STATE_NEED_ZLP;
	}

	return 1;
}

#if 0
static int jz4740_handle_get_status(struct jz4740_udc *dev,
				    struct usb_ctrlrequest *ctrl)
{
	struct jz4740_ep *ep0 = &dev->ep[0];
	struct jz4740_ep *qep;
	int reqtype = (ctrl->bRequestType & USB_RECIP_MASK);
	u16 val = 0;

	if (reqtype == USB_RECIP_INTERFACE) {
		/* This is not supported.
		 * And according to the USB spec, this one does nothing..
		 * Just return 0
		 */
		DEBUG_SETUP("GET_STATUS: USB_RECIP_INTERFACE\n");
	} else if (reqtype == USB_RECIP_DEVICE) {
		DEBUG_SETUP("GET_STATUS: USB_RECIP_DEVICE\n");
		val |= (1 << 0);	/* Self powered */
		/*val |= (1<<1); *//* Remote wakeup */
	} else if (reqtype == USB_RECIP_ENDPOINT) {
		int ep_num = (ctrl->wIndex & ~USB_DIR_IN);

		DEBUG_SETUP
			("GET_STATUS: USB_RECIP_ENDPOINT (%d), ctrl->wLength = %d\n",
			 ep_num, ctrl->wLength);

		if (ctrl->wLength > 2 || ep_num > 3)
			return -EOPNOTSUPP;

		qep = &dev->ep[ep_num];
		if (ep_is_in(qep) != ((ctrl->wIndex & USB_DIR_IN) ? 1 : 0)
		    && ep_index(qep) != 0) {
			return -EOPNOTSUPP;
		}

		usb_set_index(ep_index(qep));

		/* Return status on next IN token */
		switch (qep->ep_type) {
		case ep_control:
			val =
			    (usb_readb(qep->csr) & USB_CSR0_SENDSTALL) ==
			    USB_CSR0_SENDSTALL;
			break;
		case ep_bulk_in:
		case ep_interrupt:
			val =
			    (usb_readb(qep->csr) & USB_INCSR_SENDSTALL) ==
			    USB_INCSR_SENDSTALL;
			break;
		case ep_bulk_out:
			val =
			    (usb_readb(qep->csr) & USB_OUTCSR_SENDSTALL) ==
			    USB_OUTCSR_SENDSTALL;
			break;
		}

		/* Back to EP0 index */
		usb_set_index(0);

		DEBUG_SETUP("GET_STATUS, ep: %d (%x), val = %d\n", ep_num,
			    ctrl->wIndex, val);
	} else {
		DEBUG_SETUP("Unknown REQ TYPE: %d\n", reqtype);
		return -EOPNOTSUPP;
	}

	/* Clear "out packet ready" */
	usb_setb(USB_REG_CSR0, USB_CSR0_SVDOUTPKTRDY);
	/* Put status to FIFO */
	jz4740_fifo_write(ep0, (u8 *) & val, sizeof(val));
	/* Issue "In packet ready" */
	usb_setb(USB_REG_CSR0, (USB_CSR0_INPKTRDY | USB_CSR0_DATAEND));

	return 0;
}
#endif

/*
 * WAIT_FOR_SETUP (OUTPKTRDY)
 *      - read data packet from EP0 FIFO
 *      - decode command
 *      - if error
 *              set USB_CSR0_SVDOUTPKTRDY | USB_CSR0_DATAEND | USB_CSR0_SENDSTALL bits
 *      - else
 *              set USB_CSR0_SVDOUTPKTRDY | USB_CSR0_DATAEND bits
 */
static void jz4740_ep0_setup(struct jz4740_udc *dev, u32 csr)
{
	struct jz4740_ep *ep = &dev->ep[0];
	struct usb_ctrlrequest ctrl;
	int i;
	int clear = 1;

	DEBUG_SETUP("%s: %x\n", __FUNCTION__, csr);

	/* Nuke all previous transfers */
	nuke(ep, -EPROTO);

	/* read control req from fifo (8 bytes) */
	jz4740_fifo_read(ep, (unsigned char *)&ctrl, 8);

	DEBUG_SETUP("SETUP %02x.%02x v%04x i%04x l%04x\n",
		    ctrl.bRequestType, ctrl.bRequest,
		    ctrl.wValue, ctrl.wIndex, ctrl.wLength);

	/* Set direction of EP0 */
	if (likely(ctrl.bRequestType & USB_DIR_IN)) {
		ep->bEndpointAddress |= USB_DIR_IN;
		usb_setb(USB_REG_CSR0, USB_CSR0_SVDOUTPKTRDY);
	} else {
		ep->bEndpointAddress &= ~USB_DIR_IN;
	}

	/* Handle some SETUP packets ourselves */
	switch (ctrl.bRequest) {
	case USB_REQ_SET_ADDRESS:
		if (ctrl.bRequestType != (USB_TYPE_STANDARD | USB_RECIP_DEVICE))
			break;

		DEBUG_SETUP("USB_REQ_SET_ADDRESS (%d)\n", ctrl.wValue);
		udc_set_address(dev, ctrl.wValue);
		usb_setb(USB_REG_CSR0, (USB_CSR0_SVDOUTPKTRDY | USB_CSR0_DATAEND));
		return;

	case USB_REQ_SET_CONFIGURATION:
		if (ctrl.bRequestType != (USB_TYPE_STANDARD | USB_RECIP_DEVICE))
			break;

		DEBUG_SETUP("USB_REQ_SET_CONFIGURATION (%d)\n", ctrl.wValue);
		usb_setb(USB_REG_CSR0, (USB_CSR0_SVDOUTPKTRDY | USB_CSR0_DATAEND));

		/* Enable RESUME and SUSPEND interrupts */
		usb_setb(USB_REG_INTRUSBE, (USB_INTR_RESUME | USB_INTR_SUSPEND));
		break;

	case USB_REQ_SET_INTERFACE:
		if (ctrl.bRequestType != (USB_TYPE_STANDARD | USB_RECIP_DEVICE))
			break;

		DEBUG_SETUP("USB_REQ_SET_INTERFACE (%d)\n", ctrl.wValue);
		usb_setb(USB_REG_CSR0, (USB_CSR0_SVDOUTPKTRDY | USB_CSR0_DATAEND));
		break;

//	case USB_REQ_GET_STATUS:
//		if (jz4740_handle_get_status(dev, &ctrl) == 0)
//			return;

	case USB_REQ_CLEAR_FEATURE:
	case USB_REQ_SET_FEATURE:
		if (ctrl.bRequestType == USB_RECIP_ENDPOINT) {
			struct jz4740_ep *qep;
			int ep_num = (ctrl.wIndex & 0x0f);

			/* Support only HALT feature */
			if (ctrl.wValue != 0 || ctrl.wLength != 0
			    || ep_num > 3 || ep_num < 1)
				break;

			qep = &dev->ep[ep_num];
			spin_unlock(&dev->lock);
			if (ctrl.bRequest == USB_REQ_SET_FEATURE) {
				DEBUG_SETUP("SET_FEATURE (%d)\n",
					    ep_num);
				jz4740_set_halt(&qep->ep, 1);
			} else {
				DEBUG_SETUP("CLR_FEATURE (%d)\n",
						ep_num);
				jz4740_set_halt(&qep->ep, 0);
			}
			spin_lock(&dev->lock);

			usb_set_index(0);

			/* Reply with a ZLP on next IN token */
			usb_setb(USB_REG_CSR0, 
					(USB_CSR0_SVDOUTPKTRDY | USB_CSR0_DATAEND));
			return;
		}

		if (ctrl.bRequestType == USB_RECIP_DEVICE) {
			switch (ctrl.wValue) {
				case USB_DEVICE_TEST_MODE:
					test_mode = 1;
					if (dev->gadget.speed != USB_SPEED_HIGH)
						goto stall;
					if (ctrl.wIndex & 0xff)
						goto stall;

					switch (ctrl.wIndex >> 8) {
						case 1:
							pr_debug("TEST_J\n");
							/* TEST_J */
							test_mode_nr =
								USB_TEST_J;
							break;
						case 2:
							/* TEST_K */
							pr_debug("TEST_K\n");
							test_mode_nr =
								USB_TEST_K;
							break;
						case 3:
							/* TEST_SE0_NAK */
							pr_debug("TEST_SE0_NAK\n");
							test_mode_nr =
								USB_TEST_SE0NAK;
							break;
						case 4:
							/* TEST_PACKET */
							pr_debug("TEST_PACKET\n");
							test_mode_nr =
								USB_TEST_PACKET;
							break;

						case 0xc0:
							/* TEST_FORCE_HS */
							pr_debug("TEST_FORCE_HS\n");
							test_mode_nr =
								USB_TEST_FORCE_HS;
							break;
						case 0xc1:
							/* TEST_FORCE_FS */
							pr_debug("TEST_FORCE_FS\n");
							test_mode_nr =
								USB_TEST_FORCE_FS;
							break;
						default:
							goto stall;
					}

			}
stall:
			usb_setb(USB_REG_CSR0, 
					(USB_CSR0_SVDOUTPKTRDY | USB_CSR0_DATAEND));
			return;
		}
		break;

	default:
		break;
	}
	if (clear) {
		DEBUG_EP0("Clear OUTPKTRDY.\n");
		usb_setb(USB_REG_CSR0, USB_CSR0_SVDOUTPKTRDY);
	}
	/* gadget drivers see class/vendor specific requests,
	 * {SET,GET}_{INTERFACE,DESCRIPTOR,CONFIGURATION}, 
	 * and more.
	 */
	if (likely((u32)dev->driver)) {
		/* device-2-host (IN) or no data setup command, process immediately */
		spin_unlock(&dev->lock);

		i = dev->driver->setup(&dev->gadget, &ctrl);
		spin_lock(&dev->lock);

		if (unlikely(i < 0)) {
			/* setup processing failed, force stall */
			DEBUG_SETUP
			    ("  --> ERROR: gadget setup FAILED (stalling), setup returned %d\n",
			     i);
			usb_set_index(0);
			usb_setb(USB_REG_CSR0, (USB_CSR0_SVDOUTPKTRDY | USB_CSR0_DATAEND | USB_CSR0_SENDSTALL));

			/* ep->stopped = 1; */
			dev->ep0state = WAIT_FOR_SETUP;
		}
		else {
			DEBUG_SETUP("gadget driver setup ok (%d)\n", ctrl.wLength);
		}
	}
}

/*
 * DATA_STATE_NEED_ZLP
 */
static void jz4740_ep0_in_zlp(struct jz4740_udc *dev, u32 csr)
{
	DEBUG_EP0("%s: %x\n", __FUNCTION__, csr);

	usb_setb(USB_REG_CSR0, (USB_CSR0_INPKTRDY | USB_CSR0_DATAEND));
	dev->ep0state = WAIT_FOR_SETUP;
}

/*
 * handle ep0 interrupt
 */
static void jz4740_handle_ep0(struct jz4740_udc *dev, u32 intr)
{
	struct jz4740_ep *ep = &dev->ep[0];
	u32 csr;

	/* Set index 0 */
	usb_set_index(0);
	csr = usb_readb(USB_REG_CSR0);

	DEBUG_EP0("%s: csr = %x  state = \n", __FUNCTION__, csr);//, state_names[dev->ep0state]);

	/*
	 * if SENT_STALL is set
	 *      - clear the SENT_STALL bit
	 */
	if (csr & USB_CSR0_SENTSTALL) {
		DEBUG_EP0("%s: USB_CSR0_SENTSTALL is set: %x\n", __FUNCTION__, csr);
		usb_clearb(USB_REG_CSR0, USB_CSR0_SENDSTALL | USB_CSR0_SENTSTALL);
		nuke(ep, -ECONNABORTED);
		dev->ep0state = WAIT_FOR_SETUP;
		return;
	}

	/*
	 * if a transfer is in progress && INPKTRDY and OUTPKTRDY are clear
	 *      - fill EP0 FIFO
	 *      - if last packet
	 *      -       set IN_PKT_RDY | DATA_END
	 *      - else
	 *              set IN_PKT_RDY
	 */
	if (!(csr & (USB_CSR0_INPKTRDY | USB_CSR0_OUTPKTRDY))) {
		DEBUG_EP0("%s: INPKTRDY and OUTPKTRDY are clear\n",
			  __FUNCTION__);

		switch (dev->ep0state) {
		case DATA_STATE_XMIT:
			DEBUG_EP0("continue with DATA_STATE_XMIT\n");
			jz4740_ep0_in(dev, csr);
			return;
		case DATA_STATE_NEED_ZLP:
			DEBUG_EP0("continue with DATA_STATE_NEED_ZLP\n");
			jz4740_ep0_in_zlp(dev, csr);
			return;
		default:
			/* Stall? */
//			DEBUG_EP0("Odd state!! state = %s\n",
//				  state_names[dev->ep0state]);
			dev->ep0state = WAIT_FOR_SETUP;
			/* nuke(ep, 0); */
			/* usb_setb(ep->csr, USB_CSR0_SENDSTALL); */
//			break;
			return;
		}
	}

	/*
	 * if SETUPEND is set
	 *      - abort the last transfer
	 *      - set SERVICED_SETUP_END_BIT
	 */
	if (csr & USB_CSR0_SETUPEND) {
		DEBUG_EP0("%s: USB_CSR0_SETUPEND is set: %x\n", __FUNCTION__, csr);

		usb_setb(USB_REG_CSR0, USB_CSR0_SVDSETUPEND);
		nuke(ep, 0);
		dev->ep0state = WAIT_FOR_SETUP;
	}

	/*
	 * if USB_CSR0_OUTPKTRDY is set
	 *      - read data packet from EP0 FIFO
	 *      - decode command
	 *      - if error
	 *              set SVDOUTPKTRDY | DATAEND | SENDSTALL bits
	 *      - else
	 *              set SVDOUTPKTRDY | DATAEND bits
	 */
	if (csr & USB_CSR0_OUTPKTRDY) {

		DEBUG_EP0("%s: EP0_OUT_PKT_RDY is set: %x\n", __FUNCTION__,
			  csr);

		switch (dev->ep0state) {
		case WAIT_FOR_SETUP:
			DEBUG_EP0("WAIT_FOR_SETUP\n");
			jz4740_ep0_setup(dev, csr);
			break;

		case DATA_STATE_RECV:
			DEBUG_EP0("DATA_STATE_RECV\n");
			jz4740_ep0_out(dev, csr);
			break;

		default:
			/* send stall? */
			DEBUG_EP0("strange state!! 2. send stall? state = %d\n",
				  dev->ep0state);
			break;
		}
	}
}

static void jz4740_ep0_kick(struct jz4740_udc *dev, struct jz4740_ep *ep, struct usb_request *req)
{
	u32 csr;

	csr = usb_readb(USB_REG_CSR0);

	DEBUG_EP0("%s: %x\n", __FUNCTION__, csr);

	if (ep_is_in(ep)) {
		dev->ep0state = DATA_STATE_XMIT;
		jz4740_ep0_in(dev, csr);
	} else {
		dev->ep0state = DATA_STATE_RECV;
		/* out request will wait for the interrupt except the zero length state */
		if (!req->length)
			jz4740_ep0_out(dev, csr);
	}
}

static void jz4740_handle_reset(struct jz4740_udc *dev)
{
	udc_set_address(dev, 0);

	/* Disable interrupts */
	usb_writew(USB_REG_INTRINE, 0);
	usb_writew(USB_REG_INTROUTE, 0);
	usb_writeb(USB_REG_INTRUSBE, 0);

	/* Disable DMA */
	usb_writel(USB_REG_CNTL1, 0);
	usb_writel(USB_REG_CNTL2, 0);

	stop_activity(dev, dev->driver);

	/* Enable interrupts */
	usb_setw(USB_REG_INTRINE, USB_INTR_EP0);
	usb_setb(USB_REG_INTRUSBE, USB_INTR_RESET);

	dev->ep0state = WAIT_FOR_SETUP;
	
	dev->gadget.speed = (usb_readb(USB_REG_POWER) & USB_POWER_HSMODE) ? 
		USB_SPEED_HIGH : USB_SPEED_FULL;

	DEBUG_SETUP("%s: address = %d, speed = %s\n", __FUNCTION__, dev->usb_address,
		    (dev->gadget.speed == USB_SPEED_HIGH) ? "HIGH":"FULL");
}

static void jz4740_handle_resume(struct jz4740_udc *dev)
{
	if (dev->gadget.speed != USB_SPEED_UNKNOWN
			&& dev->driver && dev->driver->resume)
		dev->driver->resume(&dev->gadget);

	return;
}

static void jz4740_handle_suspend(struct jz4740_udc *dev)
{
	/* Host unloaded from us, can do something, such as flushing
	   the NAND block cache etc. */

	if (dev->gadget.speed != USB_SPEED_UNKNOWN
			&& dev->driver && dev->driver->suspend)
		dev->driver->suspend(&dev->gadget);
	
	return;
}

/*
 *	jz4740 usb device interrupt handler.
 */
static irqreturn_t jz4740_udc_irq(int irq, void *_dev)
{
	struct jz4740_udc *dev = _dev;

	int do_keep_alive = 1;

	u32 intr_usb = usb_readb(USB_REG_INTRUSB) & 0x7; /* mask SOF */
	u32 intr_in  = usb_readw(USB_REG_INTRIN);
	u32 intr_out = usb_readw(USB_REG_INTROUT);
	u32 intr_dma = usb_readb(USB_REG_INTR);

	if (!intr_usb && !intr_in && !intr_out && !intr_dma)
		return IRQ_HANDLED;

	DEBUG("intr_out = %x intr_in=%x intr_usb=%x\n", 
	      intr_out, intr_in, intr_usb);

	spin_lock(&dev->lock);

	if (test_mode) {
		if (test_mode_nr == USB_TEST_PACKET)
			usb_write_testpacket(&(dev->ep[0]));
		usb_setb(USB_REG_TESTMODE, test_mode_nr);
	}

	/* Check for resume from suspend mode */
	if ((intr_usb & USB_INTR_RESUME) && 
	    (usb_readb(USB_REG_INTRUSBE) & USB_INTR_RESUME)) {
		DEBUG("USB resume\n");
		jz4740_handle_resume(dev);

		do_keep_alive = 0;
	}

	/* Check for system interrupts */
	if (intr_usb & USB_INTR_RESET) {
		DEBUG("USB reset\n");
		jz4740_handle_reset(dev);
	}

	/* Check for endpoint 0 interrupt */
	if (intr_in & USB_INTR_EP0) {
		DEBUG("USB_INTR_EP0 (control)\n");
		jz4740_handle_ep0(dev, intr_in);
	}

	/* Check for Bulk-IN DMA interrupt */
	if (intr_dma & 0x1) {
		int ep_num;
		ep_num = (usb_readl(USB_REG_CNTL1) >> 4) & 0xf;
		jz4740_in_epn(dev, ep_num, intr_in);
	}

	/* Check for Bulk-OUT DMA interrupt */
	if (intr_dma & 0x2) {
		int ep_num;
#ifdef TEST_ON_WIN7
		if (out_flag)
			out_flag = 0;
#endif
		ep_num = (usb_readl(USB_REG_CNTL2) >> 4) & 0xf;
		jz4740_out_epn(dev, ep_num, intr_out);
	}

	/* Check for each configured endpoint interrupt */
	if (intr_in & USB_INTR_INEP1) {
		DEBUG("USB_INTR_INEP1\n");
		jz4740_in_epn(dev, 1, intr_in);
	}

	if (intr_in & USB_INTR_INEP2) {
		DEBUG("USB_INTR_INEP2\n");
		jz4740_in_epn(dev, 2, intr_in);
	}

	if (intr_out & USB_INTR_OUTEP1) {
		DEBUG("USB_INTR_OUTEP1\n");
		jz4740_out_epn(dev, 1, intr_out);
	}

	/* Check for suspend mode */
	if ((intr_usb & USB_INTR_SUSPEND) && 
	    (usb_readb(USB_REG_INTRUSBE) & USB_INTR_SUSPEND)) {
		DEBUG("USB suspend\n");
		jz4740_handle_suspend(dev);

		do_keep_alive = 0;
	}

#ifdef CONFIG_JZ_UDC_HOTPLUG
	if (do_keep_alive)
		uh_alive();
#endif

	spin_unlock(&dev->lock);

#ifdef	TEST_ON_WIN7
	if (out_flag == 1)
		mod_timer(&irq_timer, 400+jiffies);
#endif
	return IRQ_HANDLED;
}

/*-------------------------------------------------------------------------*/

static int jz4740_udc_get_frame(struct usb_gadget *_gadget)
{
	DEBUG("%s, %p\n", __FUNCTION__, _gadget);
	return usb_readw(USB_REG_FRAME);
}

static int jz4740_udc_wakeup(struct usb_gadget *_gadget)
{
	/* host may not have enabled remote wakeup */
	/*if ((UDCCS0 & UDCCS0_DRWF) == 0)
	   return -EHOSTUNREACH;
	   udc_set_mask_UDCCR(UDCCR_RSM); */
	return -ENOTSUPP;
}

static const struct usb_gadget_ops jz4740_udc_ops = {
	.get_frame = jz4740_udc_get_frame,
	.wakeup = jz4740_udc_wakeup,
	/* current versions must always be self-powered */
};

/*-------------------------------------------------------------------------*/

static struct jz4740_udc udc_dev = {
	.usb_address = 0,

	.gadget = {
		.ops = &jz4740_udc_ops,
		.ep0 = &udc_dev.ep[0].ep,
		.name = driver_name,
		.dev = {
			.init_name = "gadget",
		},
	},

	/* control endpoint */
	.ep[0] = {
		.ep = {
			.name = ep0name,
			.ops = &jz4740_ep_ops,
			.maxpacket = EP0_MAXPACKETSIZE,
		},
		.dev = &udc_dev,

		.bEndpointAddress = 0,
		.bmAttributes = 0,

		.ep_type = ep_control,
		.fifo = USB_FIFO_EP0,
		.csr = USB_REG_CSR0,
	},

	/* bulk out endpoint */
	.ep[1] = {
		.ep = {
			.name = "ep1out-bulk",
			.ops = &jz4740_ep_ops,
			.maxpacket = EPBULK_MAXPACKETSIZE,
		},
		.dev = &udc_dev,

		.bEndpointAddress = 1,
		.bmAttributes = USB_ENDPOINT_XFER_BULK,

		.ep_type = ep_bulk_out,
		.fifo = USB_FIFO_EP1,
		.csr = USB_REG_OUTCSR,
	},

	/* bulk in endpoint */
	.ep[2] = {
		.ep = {
			.name = "ep1in-bulk",
			.ops = &jz4740_ep_ops,
			.maxpacket = EPBULK_MAXPACKETSIZE,
		},
		.dev = &udc_dev,

		.bEndpointAddress = USB_DIR_IN | 1,
		.bmAttributes = USB_ENDPOINT_XFER_BULK,

		.ep_type = ep_bulk_in,
		.fifo = USB_FIFO_EP1,
		.csr = USB_REG_INCSR,
	},

	/* interrupt in endpoint */
	.ep[3] = {
		.ep = {
			.name = "ep2in-int",
			.ops = &jz4740_ep_ops,
			.maxpacket = EPINTR_MAXPACKETSIZE,
		},
		.dev = &udc_dev,

		.bEndpointAddress = USB_DIR_IN | 2,
		.bmAttributes = USB_ENDPOINT_XFER_INT,

		.ep_type = ep_interrupt,
		.fifo = USB_FIFO_EP2,
		.csr = USB_REG_INCSR,
	},
};



static int jz4740_udc_probe(struct platform_device *pdev)
{
	struct jz4740_udc *dev = &udc_dev;
	int rc;

	DEBUG("%s\n", __FUNCTION__);

#ifdef 	TEST_ON_WIN7
	setup_timer(&irq_timer, irq_timer_func, 0);
#endif

	spin_lock_init(&dev->lock);
	the_controller = dev;

	dev->dev = &pdev->dev;
	dev->gadget.dev.parent = &pdev->dev;

//	strcpy (dum->gadget.dev.bus_id, "gadget");
	dev->gadget.dev.release = jz4740_udc_release;
	if ((rc = device_register (&dev->gadget.dev)) < 0)
		return rc;
	platform_set_drvdata(pdev, dev);

	udc_disable(dev);
	udc_reinit(dev);

	/* irq setup */
	if (request_irq(IRQ_UDC, jz4740_udc_irq, IRQF_DISABLED,//SA_SHIRQ/*|SA_SAMPLE_RANDOM*/,
			driver_name, dev) != 0) {
		printk(KERN_INFO "request UDC interrupt %d failed\n", IRQ_UDC);
		return -EBUSY;
	}

	printk(KERN_INFO "%s\n", driver_desc);
	printk(KERN_INFO "version: " DRIVER_VERSION "\n");
	
#ifdef CONFIG_JZ_UDC_HOTPLUG
	uh_setup(pdev);

	printk(KERN_INFO "Support UDC Hotplug.\n");
#endif

	return 0;
}

static int jz4740_udc_remove(struct platform_device *pdev)
{
	struct jz4740_udc *dev = platform_get_drvdata(pdev);
	DEBUG("%s: %p\n", __FUNCTION__, dev);

	if (dev->driver)
		return -EBUSY;
	
	udc_disable(dev);

#ifdef CONFIG_JZ_UDC_HOTPLUG
	uh_cleanup(pdev);
#endif

	free_irq(IRQ_UDC, dev);
	platform_set_drvdata(pdev, 0);
	device_unregister(&dev->gadget.dev);
	the_controller = 0;

	return 0;
}

static struct platform_driver udc_driver = {
	.probe		= jz4740_udc_probe,
	.remove		= jz4740_udc_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= (char *) driver_name,
		.owner	= THIS_MODULE,
	},
};

static struct platform_device		the_udc_pdev = {
	.name		= (char *) gadget_name,
	.id		= -1,
	.dev		= {
		.release	= jz4740_udc_release,
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
#ifdef TEST_ON_WIN7
	del_timer(&irq_timer);
#endif
}

module_init(udc_init);
module_exit(udc_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Wei Jianli <jlwei@ingenic.cn>");
MODULE_LICENSE("GPL");
