/*
 * JZ4730 USB Device Controller driver
 *
 * Copyright (C) 2005 by Wei Jianli
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __JZ4730_UDC_H__
#define __JZ4730_UDC_H__

/* DRIVER DATA STRUCTURES and UTILITIES */
#define MAX_EP_NUM 		8	/* Number of endpoints on this UDC */

#define MAX_EP0_SIZE 		32
#define MAX_EP1_SIZE 		64
#define MAX_EP2_SIZE 		64
#define MAX_EP3_SIZE 		64
#define MAX_EP4_SIZE 		64
#define MAX_EP5_SIZE 		64
#define MAX_EP6_SIZE		64
#define MAX_EP7_SIZE		64

// UDC FIFO
#define RXFIFO    		(UDC_RXFIFO)    /* EP0 OUT, EP5-7 OUT */
#define TXFIFOEP0 		(UDC_TXFIFOEP0) /* EP0 IN */
#define TXFIFOEP1 		(TXFIFOEP0 + MAX_EP0_SIZE) /* EP1 IN */
#define TXFIFOEP2 		(TXFIFOEP1 + MAX_EP1_SIZE) /* EP2 IN */
#define TXFIFOEP3 		(TXFIFOEP2 + MAX_EP2_SIZE) /* EP3 IN */
#define TXFIFOEP4 		(TXFIFOEP3 + MAX_EP3_SIZE) /* EP4 IN */

static u32 ep_fifo[MAX_EP_NUM] = {TXFIFOEP0, TXFIFOEP1, TXFIFOEP2, 
				  TXFIFOEP3, TXFIFOEP4, RXFIFO, RXFIFO, 
				  RXFIFO};

#define OUT_COUNT(stats) \
	((stats&UDC_EPSR_RXPKTSIZE_MASK)>>UDC_EPSR_RXPKTSIZE_BIT)

struct jz4730_ep {
	struct usb_ep ep;
	struct jz4730_udc *dev;

	u8 index;
	u8 is_in;
	u8 stopped;
	u8 irq_pending;
	u32 fifo;

	struct list_head queue;
	const struct usb_endpoint_descriptor *desc;
};

struct jz4730_request {
	struct usb_request req;
	struct list_head queue;
};

enum ep0state {
	EP0_DISCONNECT,		/* no host */
	EP0_IDLE,		/* between STATUS ack and SETUP report */
	EP0_IN, EP0_OUT, 	/* data stage */
	EP0_STATUS,		/* status stage */
	EP0_STALL,		/* data or status stages */
	EP0_SUSPEND,		/* usb suspend */
};

struct jz4730_udc {
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;
	spinlock_t lock;

	struct jz4730_ep ep[MAX_EP_NUM];
	enum ep0state ep0state;
	unsigned char usb_address;
};

/*-------------------------------------------------------------------------*/

/* 2.5 stuff that's sometimes missing in 2.4 */

#ifndef container_of
#define	container_of	list_entry
#endif

#ifndef likely
#define likely(x)	(x)
#define unlikely(x)	(x)
#endif

#ifndef BUG_ON
#define BUG_ON(condition) do { if (unlikely((condition)!=0)) BUG(); } while(0)
#endif

#ifndef WARN_ON
#define	WARN_ON(x)	do { } while (0)
#endif

#ifndef	IRQ_NONE
typedef void irqreturn_t;
#define IRQ_NONE
#define IRQ_HANDLED
#define IRQ_RETVAL(x)
#endif

#endif /* __JZ4730_UDC_H__ */
