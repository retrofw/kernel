/*
 * JZ4770 On-Chip MAC Driver
 *
 *  Copyright (C) 2010 - 2011  Ingenic Semiconductor Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/crc32.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/skbuff.h>
#include <linux/platform_device.h>

#include <linux/dma-mapping.h>

#include "jz4770_mac.h"

#include <asm/jzsoc.h>

#ifndef CONFIG_MDIO_GPIO
static void jz4770_mdio_poll(void);
#endif

/* driver version: 1.0 */

#define JZMAC_DRV_NAME		"jz4770_mac"
#define JZMAC_DRV_VERSION	"1.0"
#define JZMAC_DRV_DESC		"JZ4770 on-chip Ethernet MAC driver"

MODULE_AUTHOR("Lutts Wolf <slcao@ingenic.cn>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(JZMAC_DRV_DESC);
MODULE_ALIAS("platform:jz4770_mac");

/* MDC  = 2.5 MHz */
#ifndef JZMAC_MDC_CLK
#define JZMAC_MDC_CLK 2500000
#endif

#define COPYBREAK_DEFAULT 256
static unsigned int copybreak __read_mostly = COPYBREAK_DEFAULT;
module_param(copybreak, uint, 0644);
MODULE_PARM_DESC(copybreak,
		 "Maximum size of packet that is copied to a new buffer on receive");

#define PKT_BUF_SZ (1580 + NET_IP_ALIGN)

#define MAX_TIMEOUT_CNT	5000

#define JZMAC_RX_BUFFER_WRITE	16	/* Must be power of 2 */

static inline unsigned char str2hexnum(unsigned char c)
{
        if (c >= '0' && c <= '9')
                return c - '0';
        if (c >= 'a' && c <= 'f')
                return c - 'a' + 10;
        if (c >= 'A' && c <= 'F')
                return c - 'A' + 10;

        return 0; /* foo */
}

static inline void str2eaddr(unsigned char *ea, unsigned char *str)
{
        int i;

        for (i = 0; i < 6; i++) {
                unsigned char num;

                if ((*str == '.') || (*str == ':'))
                        str++;
                num  = str2hexnum(*str++) << 4;
                num |= str2hexnum(*str++);
                ea[i] = num;
        }
}

static int bootargs_ethaddr = 0;
static unsigned char ethaddr_hex[6];

static int __init ethernet_addr_setup(char *str)
{
	if (!str) {
	        printk("ethaddr not set in command line\n");
		return -1;
	}
	bootargs_ethaddr = 1;
	str2eaddr(ethaddr_hex, str);

	return 0;
}

__setup("ethaddr=", ethernet_addr_setup);

/* debug routines */
__attribute__((__unused__)) static void jzmac_dump_pkt_data(unsigned char *data, int len) {
	int i = 0;
        printk("\t0x0000: ");
        for (i = 0; i < len; i++) {
                printk("%02x", data[i]);

                if (i % 2)
                        printk(" ");

                if ( (i != 0) && ((i % 16) == 15) )
                        printk("\n\t0x%04x: ", i+1);
        }
	printk("\n");
}

__attribute__((__unused__)) static void jzmac_dump_skb_data(struct sk_buff *skb) {
        printk("\n\n===================================\n");
        printk("head = 0x%08x, data = 0x%08x, tail = 0x%08x, end = 0x%08x\n",
               (unsigned int)(skb->head), (unsigned int)(skb->data),
               (unsigned int)(skb->tail), (unsigned int)(skb->end));
        printk("len = %d\n", skb->len);
	jzmac_dump_pkt_data(skb->data, skb->len);
        printk("\n=====================================\n");
}

__attribute__((__unused__)) static void jzmac_dump_dma_regs(const char *func, int line)
{
	printk("=======================%s:%d================\n", func, line);
	printk("TX_CTRL = 0x%08x RX_CTRL = 0x%08x\n", REG32(MAC_DMA_TX_CTRL), REG32(MAC_DMA_RX_CTRL));
	printk("TX_DESC = 0x%08x RX_DESC = 0x%08x\n", REG32(MAC_DMA_TX_DESC), REG32(MAC_DMA_RX_DESC));
	printk("TX_STAT = 0x%08x RX_STAT = 0x%08x\n", REG32(MAC_DMA_TX_STATUS), REG32(MAC_DMA_RX_STATUS));
	printk("INTR_MASK = 0x%08x DMA_INTR = 0x%08x\n", REG32(MAC_DMA_INTR_MASK), REG32(MAC_DMA_INTR));
	printk("==================================================\n");
}

__attribute__((__unused__)) static void jzmac_dump_mii_regs(const char *func, int line)
{
	printk("=======================%s:%d=================\n", func, line);
	printk("MII_MCR1 = 0x%08x    MII_MCR2 = 0x%08x\n", REG32(MAC_MII_MAC1), REG32(MAC_MII_MAC2));
	printk("MII_IPGR = 0x%08x    MII_IPGT = 0x%08x\n", REG32(MAC_MII_IPGR), REG32(MAC_MII_IPGT));
	printk("MII_CLRT = 0x%08x    MII_MAXF = 0x%08x\n", REG32(MAC_MII_CLRT), REG32(MAC_MII_MAXF));
	printk("MII_SUPP = 0x%08x    MII_TEST = 0x%08x\n", REG32(MAC_MII_SUPP), REG32(MAC_MII_TEST));
	printk("MII_MCMD = 0x%08x    MII_MADR = 0x%08x\n", REG32(MAC_MII_MCMD), REG32(MAC_MII_MADR));
	printk("MII_MIND = 0x%08x    MII_SA0  = 0x%08x\n", REG32(MAC_MII_MIND), REG32(MAC_MII_SA0));
	printk("MII_SA1  = 0x%08x    MII_SA2  = 0x%08x\n", REG32(MAC_MII_SA1), REG32(MAC_MII_SA2));
	printk("MII_MCFG = 0x%08x\n", REG32(MAC_MII_MCFG));
	printk("==================================================\n");
}

__attribute__((__unused__)) static void jzmac_dump_fifo_regs(const char *func, int line)
{
	printk("=================%s:%d=====================\n", func, line);
	printk("FIFO_CFG_R0 = 0x%08x\n", REG32(MAC_FIFO_CFG_R0));
	printk("FIFO_CFG_R1 = 0x%08x\n", REG32(MAC_FIFO_CFG_R1));
	printk("FIFO_CFG_R2 = 0x%08x\n", REG32(MAC_FIFO_CFG_R2));
	printk("FIFO_CFG_R3 = 0x%08x\n", REG32(MAC_FIFO_CFG_R3));
	printk("FIFO_CFG_R4 = 0x%08x\n", REG32(MAC_FIFO_CFG_R4));
	printk("FIFO_CFG_R5 = 0x%08x\n", REG32(MAC_FIFO_CFG_R5));
	printk("==================================================\n");
}

__attribute__((__unused__)) static void jzmac_dump_stat_regs(const char *func, int line)
{

}

__attribute__((__unused__)) static void jzmac_dump_sal_regs(const char *func, int line)
{
	printk("=================%s:%d=====================\n", func, line);
	printk("SAL_AFR = 0x%08x, SAL_HT1 = 0x%08x, SAL_HT2 = 0x%08x\n",
	       REG32(MAC_SAL_AFR), REG32(MAC_SAL_HT1), REG32(MAC_SAL_HT2));
	printk("==================================================\n");
}

__attribute__((__unused__)) static void jzmac_dump_all_regs(const char *func, int line) {
	jzmac_dump_dma_regs(func, line);
	jzmac_dump_mii_regs(func, line);
	jzmac_dump_fifo_regs(func, line);
	jzmac_dump_stat_regs(func, line);
	jzmac_dump_sal_regs(func, line);
}

__attribute__((__unused__)) static void jzmac_dump_dma_buffer_info(struct jzmac_buffer *buffer_info) {
	printk("\tbuffer_info(%p):\n", buffer_info);
	printk("\t\tskb = %p\n", buffer_info->skb);
	printk("\t\tdma = 0x%08x\n", buffer_info->dma);
	printk("\t\tlen = %u\n", buffer_info->length);
	printk("\t\ttrans = %d\n", buffer_info->transfering);
	printk("\t\tinvalid = %d\n", buffer_info->invalid);
#ifdef JZMAC_ENABLE_4BYTE_WORKAROUND
	printk("\t\twork_around = %p\n", buffer_info->work_around);
#endif
}

__attribute__((__unused__)) static void jzmac_dump_dma_desc(struct jzmac_dma_desc *desc) {
	printk("\tdma desc(%p):\n", desc);
	printk("\t\taddr = 0x%08x\n", desc->pkt_addr);
	printk("\t\tsize = %u\n", desc->pkt_size);
	printk("\t\tnext = 0x%08x\n", desc->next_desc);
}

__attribute__((__unused__)) static void jzmac_dump_rx_desc(struct jz4770_mac_local *lp) {
	int i = 0;
	printk("\n===================rx====================\n");
	printk("count = %d, next_to_use = %d next_to_clean = %d\n",
	       lp->rx_ring.count, lp->rx_ring.next_to_use, lp->rx_ring.next_to_clean);
	for (i = 0; i < JZMAC_RX_DESC_COUNT; i++) {
		struct jzmac_dma_desc *desc = lp->rx_ring.desc + i;
		struct jzmac_buffer *b = lp->rx_ring.buffer_info + i;

		printk("desc %d:\n", i);
		jzmac_dump_dma_desc(desc);
		jzmac_dump_dma_buffer_info(b);
	}
	printk("\n=========================================\n");
}

__attribute__((__unused__)) static void jzmac_dump_tx_desc(struct jz4770_mac_local *lp) {
	int i = 0;
	printk("\n===================tx====================\n");
	printk("count = %d, next_to_use = %d next_to_clean = %d\n",
	       lp->tx_ring.count, lp->tx_ring.next_to_use, lp->tx_ring.next_to_clean);
	for (i = 0; i < JZMAC_TX_DESC_COUNT; i++) {
		struct jzmac_dma_desc *desc = lp->tx_ring.desc + i;
		struct jzmac_buffer *b = lp->tx_ring.buffer_info + i;

		printk("desc %d:\n", i);
		jzmac_dump_dma_desc(desc);
		jzmac_dump_dma_buffer_info(b);
	}
	printk("\n=========================================\n");
}

__attribute__((__unused__)) static void jzmac_dump_all_desc(struct jz4770_mac_local *lp) {
	jzmac_dump_rx_desc(lp);
	jzmac_dump_tx_desc(lp);
}

__attribute__((__unused__)) static int get_rx_index_by_desc(struct jz4770_mac_local *lp, struct jzmac_dma_desc *desc) {
	int i = 0;

	for (i = 0; i < JZMAC_RX_DESC_COUNT; i++) {
		if ( (lp->rx_ring.desc + i) == desc)
			return i;
	}

	BUG_ON(i == JZMAC_RX_DESC_COUNT);
	return -1;
}

static void jzmac_restart_rx_dma(struct jz4770_mac_local *lp) {
	if (__jzmac_rx_dma_stopped()) {
#if 0
		struct jzmac_dma_desc *stop_desc = (struct jzmac_dma_desc *)REG32(MAC_DMA_RX_DESC);
		int stop_idx = -1;
		struct jzmac_buffer *stop_buf = NULL;
		stop_desc = (struct jzmac_dma_desc *)CKSEG1ADDR(stop_desc);

		if (stop_desc) {
			stop_idx = get_rx_index_by_desc(lp, stop_desc);
			stop_buf = lp->rx_ring.buffer_info + stop_idx;

			printk("===>rx dma stopped: stop_desc = %p idx = %d\n", stop_desc, stop_idx);
			jzmac_dump_dma_desc(stop_desc);
			jzmac_dump_dma_buffer_info(stop_buf);
			printk("============================================\n");
		}

#endif
		__jzmac_enable_rx_dma();
	}
}

static void jzmac_alloc_rx_buffers(struct jz4770_mac_local *lp, int cleaned_count,
				   int restart_dma) {
	int i = 0;
	struct jzmac_buffer *buffer_info;
	struct sk_buff *skb;
	struct jzmac_rx_ring *rx_ring = &lp->rx_ring;
	struct jzmac_dma_desc *rx_desc;
	struct jzmac_dma_desc *first_desc;
	int first;

	first = rx_ring->next_to_use;
	i = rx_ring->next_to_use;
	rx_desc = JZMAC_RX_DESC(*rx_ring, i);
	first_desc = rx_desc;
	buffer_info = &rx_ring->buffer_info[i];

	while (cleaned_count--) {
		skb = buffer_info->skb;
		if (skb) {
			skb_trim(skb, 0);
			goto map_skb;
		}

		skb = netdev_alloc_skb(lp->netdev, PKT_BUF_SZ);
		if (unlikely(!skb)) {
			/* Better luck next round */
			lp->alloc_rx_buff_failed++;
			break;
		}

		skb_reserve(skb, NET_IP_ALIGN);

		buffer_info->skb = skb;
		buffer_info->length = PKT_BUF_SZ - NET_IP_ALIGN;
	map_skb:
		buffer_info->dma = dma_map_single(&lp->netdev->dev,
						  skb->data, PKT_BUF_SZ - NET_IP_ALIGN,
						  DMA_FROM_DEVICE);
		rx_desc->pkt_addr = cpu_to_le32(buffer_info->dma);

		/* clr invalid first, then start transfer */
		buffer_info->invalid = 0;
		/* start transfer */
		__desc_rx_as_valid(rx_desc);

		/* next */
		if (unlikely(++i == rx_ring->count))
			i = 0;

		wmb();

		rx_desc = JZMAC_RX_DESC(*rx_ring, i);
		buffer_info = &rx_ring->buffer_info[i];
	}


	if (likely(rx_ring->next_to_use != i)) {
		rx_ring->next_to_use = i;
		/* sanity check: ensure next_to_use is not used */
		rx_desc->pkt_addr = cpu_to_le32(0);
		__desc_rx_as_invalid(rx_desc);
		buffer_info->invalid = 1;
		wmb();

		/* assure that if there's any buffer space, dma is enabled */
		if (likely(restart_dma))
			jzmac_restart_rx_dma(lp);
	}
}

static int desc_list_init_rx(struct jz4770_mac_local *lp) {
	int i;
	struct jzmac_dma_desc *tail;
	int size;

	/* rx init */
	lp->rx_ring.count = JZMAC_RX_DESC_COUNT;

	size = lp->rx_ring.count * sizeof(struct jzmac_buffer);
	lp->rx_ring.buffer_info = vmalloc(size);
	if (!lp->rx_ring.buffer_info) {
		printk(KERN_ERR "Unable to allocate memory for the receive descriptor ring\n");
		return -ENOMEM;
	}
	memset(lp->rx_ring.buffer_info, 0, size);

	lp->rx_ring.desc = dma_alloc_noncoherent(&lp->netdev->dev,
						 lp->rx_ring.count * sizeof(struct jzmac_dma_desc),
						 &lp->rx_ring.dma, GFP_KERNEL);

	if (lp->rx_ring.desc == NULL) {
		vfree(lp->rx_ring.buffer_info);
		lp->rx_ring.buffer_info = NULL;
		return -ENOMEM;
	}

	dma_cache_wback_inv((unsigned long)lp->rx_ring.desc,
			    lp->rx_ring.count * sizeof(struct jzmac_dma_desc));

	/* we always use uncached address for descriptors */
	lp->rx_ring.desc = (struct jzmac_dma_desc *)CKSEG1ADDR(lp->rx_ring.desc);

	tail = lp->rx_ring.desc;
	for (i = 0; i < JZMAC_RX_DESC_COUNT; i++) {
		struct jzmac_dma_desc *desc = lp->rx_ring.desc + i;

		desc->pkt_addr = cpu_to_le32(0);
		desc->pkt_size = cpu_to_le32(EMPTY_FLAG_MASK); /* owned by DMA, can fill data */
		tail->next_desc = CPHYSADDR(desc);	       /* treat it as an array */
		tail = desc;
	}
	tail->next_desc = CPHYSADDR(lp->rx_ring.desc);	/* rx_list is a circle */

	lp->rx_ring.next_to_use = lp->rx_ring.next_to_clean = 0;
	jzmac_alloc_rx_buffers(lp, JZMAC_DESC_UNUSED(&lp->rx_ring), 0);

	return 0;
}

static void desc_list_free_rx(struct jz4770_mac_local *lp) {
	struct jzmac_buffer *b;
	int i = 0;

	if (lp->rx_ring.desc)
		dma_free_noncoherent(&lp->netdev->dev,
				     lp->rx_ring.count * sizeof(struct jzmac_dma_desc),
				     (void *)CKSEG0ADDR(lp->rx_ring.desc),
				     lp->rx_ring.dma);

	if (lp->rx_ring.buffer_info) {
		b = lp->rx_ring.buffer_info;

		for(i = 0; i < JZMAC_RX_DESC_COUNT; i++) {
			if (b[i].skb) {
				skb_dma_unmap(&lp->netdev->dev,
					      b[i].skb,
					      DMA_TO_DEVICE);
				dev_kfree_skb_any(b[i].skb);
				b[i].skb = NULL;
			}
		}
	}
}

/* MUST ensure that rx is stopped ans rx_dma is disabled */
static void desc_list_reinit_rx(struct jz4770_mac_local *lp) {
	struct jzmac_dma_desc *desc;
	int i = 0;

	BUG_ON(!__jzmac_rx_dma_stopped());

	for (i = 0; i < JZMAC_RX_DESC_COUNT; i++) {
		desc = lp->rx_ring.desc + i;

		desc->pkt_addr = cpu_to_le32(0);
		desc->pkt_size = cpu_to_le32(EMPTY_FLAG_MASK); /* owned by DMA, can fill data */
	}

	lp->rx_ring.next_to_use = lp->rx_ring.next_to_clean = 0;
	jzmac_alloc_rx_buffers(lp, JZMAC_DESC_UNUSED(&lp->rx_ring), 0);
}

static int desc_list_init_tx(struct jz4770_mac_local *lp) {
	int i;
	struct jzmac_dma_desc *tail;
	int size;

	/* tx init */
	lp->tx_ring.count = JZMAC_TX_DESC_COUNT;

	size = lp->tx_ring.count * sizeof(struct jzmac_buffer);
	lp->tx_ring.buffer_info = vmalloc(size);
	if (!lp->tx_ring.buffer_info) {
		printk(KERN_ERR"Unable to allocate memory for the receive descriptor ring\n");
		return -ENOMEM;
	}
	memset(lp->tx_ring.buffer_info, 0, size);

	lp->tx_ring.desc = dma_alloc_noncoherent(&lp->netdev->dev,
						 lp->tx_ring.count * sizeof(struct jzmac_dma_desc),
						 &lp->tx_ring.dma, GFP_KERNEL);

	if (lp->tx_ring.desc == NULL) {
		vfree(lp->tx_ring.buffer_info);
		lp->tx_ring.buffer_info = NULL;
		return -ENOMEM;
	}

	dma_cache_wback_inv((unsigned long)lp->tx_ring.desc,
			    lp->tx_ring.count * sizeof(struct jzmac_dma_desc));

	/* we always use uncached address for descriptors */
	lp->tx_ring.desc = (struct jzmac_dma_desc *)CKSEG1ADDR(lp->tx_ring.desc);

	tail = lp->tx_ring.desc;
	for (i = 0; i < JZMAC_TX_DESC_COUNT; i++) {
		struct jzmac_dma_desc *desc = lp->tx_ring.desc + i;

		desc->pkt_addr = cpu_to_le32(0);
		desc->pkt_size = cpu_to_le32(EMPTY_FLAG_MASK); /* owned by CPU, no valid data (transfer done!) */
		tail->next_desc = CPHYSADDR(desc);	       /* treat it as an array */
		tail = desc;
	}
	tail->next_desc = CPHYSADDR(lp->tx_ring.desc);	/* tx_list is a circle */
	lp->tx_ring.next_to_use = lp->tx_ring.next_to_clean = 0;
#ifdef JZMAC_ENABLE_4BYTE_WORKAROUND
	{
		struct jzmac_buffer *b;
		dma_addr_t tmp_addr;
		for (i = 0; i < JZMAC_TX_DESC_COUNT; i++) {
			b = lp->tx_ring.buffer_info + i;
			b->work_around = dma_alloc_noncoherent(&lp->netdev->dev,
							       PKT_BUF_SZ,
							       &tmp_addr, GFP_KERNEL);
		}
	}
#endif

	return 0;
}

__attribute__((__unused__)) static int get_tx_index_by_desc(struct jz4770_mac_local *lp, struct jzmac_dma_desc *desc) {
	int i = 0;

	for (i = 0; i < JZMAC_TX_DESC_COUNT; i++) {
		if ( (lp->tx_ring.desc + i) == desc)
			return i;
	}

	BUG_ON(i == JZMAC_TX_DESC_COUNT);
	return -1;
}

static void jzmac_unmap_and_free_tx_resource(struct jz4770_mac_local *lp,
					     struct jzmac_buffer *buffer_info)
{
	buffer_info->transfering = 0;
	buffer_info->dma = 0;
	if (buffer_info->skb) {
		skb_dma_unmap(&lp->netdev->dev,
			      buffer_info->skb,
		              DMA_TO_DEVICE);
		dev_kfree_skb_any(buffer_info->skb);
		buffer_info->skb = NULL;
	}
	buffer_info->time_stamp = 0;
}

static void desc_list_free_tx(struct jz4770_mac_local *lp) {
	struct jzmac_buffer *b;
	int i = 0;

	if (lp->tx_ring.desc)
		dma_free_noncoherent(&lp->netdev->dev,
				     lp->tx_ring.count * sizeof(struct jzmac_dma_desc),
				     (void *)CKSEG0ADDR(lp->tx_ring.desc),
				     lp->tx_ring.dma);

	if (lp->tx_ring.buffer_info) {
		b = lp->tx_ring.buffer_info;

		for (i = 0; i < JZMAC_TX_DESC_COUNT; i++) {
			// panic("===>ahha, testing! please do not goes here(%s:%d)!!!\n", __func__, __LINE__);
			jzmac_unmap_and_free_tx_resource(lp, b + i);
#ifdef JZMAC_ENABLE_4BYTE_WORKAROUND
			dma_free_noncoherent(&lp->netdev->dev,
					     PKT_BUF_SZ,
					     b[i].work_around,
					     (dma_addr_t)CPHYSADDR(b[i].work_around));
#endif
		}

	}
}

/* must assure that tx transfer are stopped and tx_dma is disabled */
static void desc_list_reinit_tx(struct jz4770_mac_local *lp) {
	int i = 0;
	struct jzmac_dma_desc *desc;
	struct jzmac_buffer *b;

	BUG_ON(!__jzmac_tx_dma_stopped());

	for (i = 0; i < JZMAC_TX_DESC_COUNT; i++) {
		desc = lp->tx_ring.desc + i;
		b = lp->tx_ring.buffer_info + i;

		desc->pkt_size = cpu_to_le32(EMPTY_FLAG_MASK); /* owned by CPU, no valid data (transfer done!) */
		desc->pkt_addr = cpu_to_le32(0);

		// panic("===>ahha, testing! please do not goes here(%s:%d)!!!\n", __func__, __LINE__);
		jzmac_unmap_and_free_tx_resource(lp, b);
	}

	lp->tx_ring.next_to_use = lp->tx_ring.next_to_clean = 0;
}

static void desc_list_free(struct jz4770_mac_local *lp)
{
	desc_list_free_rx(lp);
	desc_list_free_tx(lp);
}

static void desc_list_reinit(struct jz4770_mac_local *lp) {
	desc_list_reinit_rx(lp);
	desc_list_reinit_tx(lp);
	//jzmac_dump_all_desc(lp);
}

static int desc_list_init(struct jz4770_mac_local *lp)
{
	if (desc_list_init_rx(lp) < 0)
		goto init_error;

	if (desc_list_init_tx(lp) < 0)
		goto init_error;


	//jzmac_dump_all_desc(lp);
	return 0;

 init_error:
	desc_list_free(lp);
	printk(KERN_ERR JZMAC_DRV_NAME ": kmalloc failed\n");
	return -ENOMEM;
}


/*---PHY CONTROL AND CONFIGURATION-----------------------------------------*/
static void jz4770_mac_adjust_link(struct net_device *dev)
{
	struct jz4770_mac_local *lp = netdev_priv(dev);
	struct phy_device *phydev = lp->phydev;
	unsigned long flags;
	int new_state = 0;

	//printk("===>ajust link, old_duplex = %d, old_speed = %d, old_link = %d\n",
	//       lp->old_duplex, lp->old_speed, lp->old_link);

	spin_lock_irqsave(&lp->link_lock, flags);
	if (phydev->link) {
		/* Now we make sure that we can be in full duplex mode.
		 * If not, we operate in half-duplex mode. */
		if (phydev->duplex != lp->old_duplex) {
			new_state = 1;


			if (phydev->duplex) {
				__jzmac_enable_full_duplex();

			} else {
				__jzmac_disable_full_duplex();
			}

			lp->old_duplex = phydev->duplex;
		}

		if (phydev->speed != lp->old_speed) {
#if defined(CONFIG_JZ4770_MAC_RMII)
			switch (phydev->speed) {
			case 10:
				__jzmac_set_rmii_10M();
				break;
			case 100:
				__jzmac_set_rmii_100M();
				break;
			default:
				printk(KERN_WARNING
				       "%s: Ack!  Speed (%d) is not 10/100!\n",
				       JZMAC_DRV_NAME, phydev->speed);
				break;
			}
#endif

			new_state = 1;
			lp->old_speed = phydev->speed;
		}

		if (!lp->old_link) {
			new_state = 1;
			lp->old_link = 1;
			netif_carrier_on(dev);
			//mod_timer(&lp->watchdog_timer, jiffies + 1);
		}
	} else if (lp->old_link) {
		new_state = 1;
		lp->old_link = 0;
		lp->old_speed = 0;
		lp->old_duplex = -1;
		netif_carrier_off(dev);
	}

	if (new_state)
		phy_print_status(phydev);

	//printk("===>ajust link, new_duplex = %d, new_speed = %d, new_link = %d\n",
	//       lp->old_duplex, lp->old_speed, lp->old_link);

	spin_unlock_irqrestore(&lp->link_lock, flags);

}

static int mii_probe(struct net_device *dev)
{
	struct jz4770_mac_local *lp = netdev_priv(dev);
	struct phy_device *phydev = NULL;
	int clkdiv[8] = { 4, 4, 6, 8, 10, 14, 20, 28 };
	int i;

	/* search for connect PHY device */
	for (i = 0; i < PHY_MAX_ADDR; i++) {
		struct phy_device *const tmp_phydev = lp->mii_bus->phy_map[i];

		if (!tmp_phydev)
			continue; /* no PHY here... */

		phydev = tmp_phydev;
		break; /* found it */
	}

	/* now we are supposed to have a proper phydev, to attach to... */
	if (!phydev) {
		printk(KERN_INFO "%s: Don't found any phy device at all\n",
		       dev->name);
		return -ENODEV;
	}

#if defined(CONFIG_JZ4770_MAC_RMII)
	phydev = phy_connect(dev, dev_name(&phydev->dev), &jz4770_mac_adjust_link,
			     0, PHY_INTERFACE_MODE_RMII);
#else
	phydev = phy_connect(dev, dev_name(&phydev->dev), &jz4770_mac_adjust_link,
			     0, PHY_INTERFACE_MODE_MII);
#endif

	if (IS_ERR(phydev)) {
		printk(KERN_ERR "%s: Could not attach to PHY\n", dev->name);
		return PTR_ERR(phydev);
	}

	/* mask with MAC supported features */
	phydev->supported &= (SUPPORTED_10baseT_Half
			      | SUPPORTED_10baseT_Full
			      | SUPPORTED_100baseT_Half
			      | SUPPORTED_100baseT_Full
			      | SUPPORTED_Autoneg
			      | SUPPORTED_Pause | SUPPORTED_Asym_Pause
			      | SUPPORTED_MII
			      | SUPPORTED_TP);

	phydev->advertising = phydev->supported;

	lp->old_link = 0;
	lp->old_speed = 0;
	lp->old_duplex = -1;
	lp->phydev = phydev;

	printk(KERN_INFO "%s: attached PHY driver [%s] "
	       "(mii_bus:phy_addr=%s, irq=%d, mdc_clk=%dHz\n",
	       JZMAC_DRV_NAME, phydev->drv->name, dev_name(&phydev->dev), phydev->irq,
	       cpm_get_clock(CGU_H2CLK) / clkdiv[__jzmac_get_mdc_clkdiv()]);

	return 0;
}

/**
 * jzmac_update_stats - Update the board statistics counters
 * @lp: board private structure
 **/

void jzmac_update_stats(struct jz4770_mac_local *lp)
{
	if ((lp->old_link == 0) || (lp->old_speed == 0) || (lp->old_duplex == -1))
		return;

#if 1
	{
		int i = 0;
		int j = 0;
		printk("****************statistics****************\n");
		for (j = 0, i = 0x80; i <= 0x12c; i += 4, j++) {
			printk("0x%03x: %010u ", i, REG32(ETHC_BASE + i));

			if ( (j & 0x3) == 3)
				printk("\n");
		}
		printk("\n******************************************\n");
	}
#endif

#if 0
	//spin_lock_irqsave(&lp->stats_lock, flags);
	//spin_lock(&lp->stats_lock);

	/* Fill out the OS statistics structure */
	lp->net_stats.multicast = REG32(MAC_STAT_RMCA);
	lp->net_stats.collisions = REG32(MAC_STAT_RBCA);

	/* Rx Errors */

	/* RLEC on some newer hardware can be incorrect so build
	 * our own version based on RUC and ROC */
	lp->net_stats.rx_errors = lp->stats.rxerrc +
		lp->stats.crcerrs + lp->stats.algnerrc +
		lp->stats.ruc + lp->stats.roc +
		lp->stats.cexterr;
	lp->net_stats.rx_length_errors = REG32(MAC_STAT_RFLR);
	lp->net_stats.rx_crc_errors = REG32(MAC_STAT_RFCS);
	lp->net_stats.rx_frame_errors = REG32(MAC_STAT_RALN);
	lp->net_stats.rx_missed_errors = lp->stats.mpc;

	/* Tx Errors */
	lp->stats.txerrc = lp->stats.ecol + lp->stats.latecol;
	lp->net_stats.tx_errors = lp->stats.txerrc;
	lp->net_stats.tx_aborted_errors = lp->stats.ecol;
	lp->net_stats.tx_window_errors = lp->stats.latecol;
	lp->net_stats.tx_carrier_errors = lp->stats.tncrs;
	if (hw->bad_tx_carr_stats_fd &&
	    lp->link_duplex == FULL_DUPLEX) {
		lp->net_stats.tx_carrier_errors = 0;
		lp->stats.tncrs = 0;
	}

	/* Tx Dropped needs to be maintained elsewhere */

	/* Phy Stats */
	if (hw->media_type == e1000_media_type_copper) {
		if ((lp->link_speed == SPEED_1000) &&
		    (!e1000_read_phy_reg(hw, PHY_1000T_STATUS, &phy_tmp))) {
			phy_tmp &= PHY_IDLE_ERROR_COUNT_MASK;
			lp->phy_stats.idle_errors += phy_tmp;
		}

		if ((hw->mac_type <= e1000_82546) &&
		    (hw->phy_type == e1000_phy_m88) &&
		    !e1000_read_phy_reg(hw, M88E1000_RX_ERR_CNTR, &phy_tmp))
			lp->phy_stats.receive_errors += phy_tmp;
	}

	/* Management Stats */
	if (hw->has_smbus) {
		lp->stats.mgptc += er32(MGTPTC);
		lp->stats.mgprc += er32(MGTPRC);
		lp->stats.mgpdc += er32(MGTPDC);
	}

	//spin_unlock_irqrestore(&lp->stats_lock, flags);
	//spin_unlock(&lp->stats_lock);
#endif
}

/**
 * jzmac_watchdog - Timer Call-back
 * @data: pointer to lp cast into an unsigned long
 **/
static void jzmac_watchdog(unsigned long data) {
	struct jz4770_mac_local *lp = (struct jz4770_mac_local *)data;
	//struct net_device *netdev = lp->netdev;

	jzmac_update_stats(lp);

	mod_timer(&lp->watchdog_timer, round_jiffies(jiffies + 5 * HZ));
}

static int __jzmac_maybe_stop_tx(struct net_device *netdev, int size)
{
	struct jz4770_mac_local *lp = netdev_priv(netdev);
	struct jzmac_tx_ring *tx_ring = &lp->tx_ring;

	netif_stop_queue(netdev);
	smp_mb();

	/* We need to check again in a case another CPU has just
	 * made room available. */
	if (likely(JZMAC_DESC_UNUSED(tx_ring) < size))
		return -EBUSY;

	/* A reprieve! */
	netif_start_queue(netdev);
	++lp->restart_queue;
	return 0;
}

static int jzmac_maybe_stop_tx(struct net_device *netdev,
                               struct jzmac_tx_ring *tx_ring, int size)
{
	if (likely(JZMAC_DESC_UNUSED(tx_ring) >= size))
		return 0;
	return __jzmac_maybe_stop_tx(netdev, size);
}

static int jzmac_tx_map(struct jz4770_mac_local *lp,
			struct jzmac_tx_ring *tx_ring,
			struct sk_buff *skb)
{
	struct jzmac_buffer *buffer_info;
	unsigned int i;

	i = tx_ring->next_to_use;

	if (skb_dma_map(&lp->netdev->dev, skb, DMA_TO_DEVICE)) {
		dev_err(&lp->netdev->dev, "TX DMA map failed\n");
		return 0;
	}

	buffer_info = &tx_ring->buffer_info[i];

	//buffer_info->length = (skb->len < ETH_ZLEN) ? ETH_ZLEN : skb->len;
	buffer_info->length = skb->len;
	buffer_info->dma = skb_shinfo(skb)->dma_head;
	buffer_info->time_stamp = jiffies;
	buffer_info->skb = skb;
	return 1;
}

static void jzmac_restart_tx_dma(struct jz4770_mac_local *lp) {
#if 0			      /* for debug */
	if (__jzmac_tx_dma_stopped()) {
		struct jzmac_dma_desc *stop_desc = (struct jzmac_dma_desc *)REG32(MAC_DMA_TX_DESC);
		int stop_idx = -1;
		struct jzmac_buffer *stop_buf = NULL;
		stop_desc = (struct jzmac_dma_desc *)CKSEG1ADDR(stop_desc);

#if 1
		if (stop_desc) {
			stop_idx = get_tx_index_by_desc(lp, stop_desc);
			stop_buf = lp->tx_ring.buffer_info + stop_idx;

			printk("===>tx dma stopped: stop_desc = %p idx = %d tx_status = 0x%08x, curr_use_idx = %u next_to_clean = %d\n",
			       stop_desc, stop_idx,  REG32(MAC_DMA_TX_STATUS), curr_use_idx, lp->tx_ring.next_to_clean);
			jzmac_dump_dma_desc(stop_desc);
			jzmac_dump_dma_buffer_info(stop_buf);
			printk("============================================\n");
		}
#endif
	}
#endif

	__jzmac_tx_status_clr_err();
	__jzmac_enable_tx_dma();
	/* ensure irq is enabled */
	__jzmac_enable_usefull_tx_irq();
}

static void jzmac_tx_queue(struct jz4770_mac_local *lp,
			   struct jzmac_tx_ring *tx_ring)
{
	struct jzmac_dma_desc *tx_desc = NULL;
	struct jzmac_buffer *buffer_info;
	unsigned int i;

	i = tx_ring->next_to_use;

	buffer_info = &tx_ring->buffer_info[i];
	tx_desc = JZMAC_TX_DESC(*tx_ring, i);

#ifdef JZMAC_ENABLE_4BYTE_WORKAROUND
	if (unlikely(jzmac_tx_align_invalid(buffer_info->dma))) {
		memcpy(buffer_info->work_around, (void *)CKSEG1ADDR(buffer_info->dma), buffer_info->length);
		dma_cache_wback_inv((unsigned long)buffer_info->work_around, PKT_BUF_SZ);
		tx_desc->pkt_addr = cpu_to_le32(CPHYSADDR(buffer_info->work_around));
	} else
		tx_desc->pkt_addr = cpu_to_le32(buffer_info->dma);
#else
	tx_desc->pkt_addr = cpu_to_le32(buffer_info->dma);
#endif

	//udelay(150);
	tx_desc->pkt_size = cpu_to_le32(buffer_info->length & ~EMPTY_FLAG_MASK);

	wmb();

	buffer_info->transfering = 1;

	if (unlikely(++i == tx_ring->count)) i = 0;
	tx_ring->next_to_use = i;

	wmb();
	jzmac_restart_tx_dma(lp);
}


static int jz4770_mac_hard_start_xmit(struct sk_buff *skb,
				    struct net_device *netdev)
{
	struct jz4770_mac_local *lp = netdev_priv(netdev);
	struct jzmac_tx_ring *tx_ring;
	unsigned int first;
	int count = 1;

	tx_ring = &lp->tx_ring;

	if (unlikely(skb->len <= 0)) {
		printk(JZMAC_DRV_NAME ": WARNING: skb->len < 0\n");
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

#ifdef JZMAC_ENABLE_4BYTE_WORKAROUND
	if (unlikely(jzmac_tx_align_invalid(skb->data) &&
		     (skb->len > (PKT_BUF_SZ - NET_IP_ALIGN)))) {
		printk(JZMAC_DRV_NAME ": WARNING: skb->len(%u) too large for unaligned address\n", skb->len);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
#endif

	if (skb_shinfo(skb)->nr_frags) {
		printk(JZMAC_DRV_NAME ": WARNING: fragment packet do not handled!!!\n");
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	/* need: count + 2 desc gap to keep tail from touching
	 * head, otherwise try next time */
	if (unlikely(jzmac_maybe_stop_tx(netdev, tx_ring, count + 2)))
		return NETDEV_TX_BUSY;

	first = tx_ring->next_to_use;
	count = jzmac_tx_map(lp, tx_ring, skb);

	if (likely(count)) {
		jzmac_tx_queue(lp, tx_ring);
		//jzmac_dump_all_regs(__func__, __LINE__);
	} else {
		dev_kfree_skb_any(skb);
		tx_ring->buffer_info[first].time_stamp = 0;
		tx_ring->next_to_use = first;
	}

	return NETDEV_TX_OK;
}

static bool jzmac_clean_tx_irq(struct jz4770_mac_local *lp) {
	struct net_device *netdev = lp->netdev;
	struct jzmac_dma_desc *desc;
	struct jzmac_buffer *buffer_info;
	struct jzmac_tx_ring *tx_ring = &lp->tx_ring;
	unsigned int i;
	unsigned int count = 0;
	unsigned int total_tx_bytes=0, total_tx_packets=0;

	i = tx_ring->next_to_clean;
	desc = JZMAC_TX_DESC(*tx_ring, i);
	buffer_info = &tx_ring->buffer_info[i];

#if 0			      /* just for debug */
	printk("===>%s: next_to_clean = %d, tx_status = 0x%08x, tx_ctrl = 0x%08x\n",
	       __func__, i, REG32(MAC_DMA_TX_STATUS), REG32(MAC_DMA_TX_CTRL));
	jzmac_dump_dma_desc(desc);
	jzmac_dump_dma_buffer_info(buffer_info);
#endif
	while (buffer_info->transfering &&
	       __desc_tx_transfer_done(desc) &&
	       (count < tx_ring->count)) {
		buffer_info->transfering = 0;
		count++;

		total_tx_packets ++;
		total_tx_bytes += buffer_info->length;

		jzmac_unmap_and_free_tx_resource(lp, buffer_info);

		i++;
		if (unlikely(i == tx_ring->count)) i = 0;

		desc = JZMAC_TX_DESC(*tx_ring, i);
		buffer_info = &tx_ring->buffer_info[i];
	}

	tx_ring->next_to_clean = i;
	//printk("===>tx: %d pkts cleaned\n", count);

#define TX_WAKE_THRESHOLD 16
	if (unlikely(count && netif_carrier_ok(netdev) &&
		     JZMAC_DESC_UNUSED(tx_ring) >= TX_WAKE_THRESHOLD)) {
		/* Make sure that anybody stopping the queue after this
		 * sees the new next_to_clean.
		 */
		smp_mb();
		if (netif_queue_stopped(netdev)) {
			netif_wake_queue(netdev);
			++lp->restart_queue;
		}
	}

	lp->total_tx_bytes += total_tx_bytes;
	lp->total_tx_packets += total_tx_packets;
	lp->net_stats.tx_bytes += total_tx_bytes;
	lp->net_stats.tx_packets += total_tx_packets;

	return (count < tx_ring->count);
}

static bool jzmac_clean_rx_irq(struct jz4770_mac_local *lp,
			       int *work_done, int work_to_do) {

	struct net_device *netdev = lp->netdev;
	struct jzmac_rx_ring *rx_ring = &lp->rx_ring;
	struct jzmac_dma_desc *rx_desc, *next_rxd;
	struct jzmac_buffer *buffer_info, *next_buffer;
	u32 length;
	unsigned int i;
	int cleaned_count = 0;
	bool cleaned = false;
	unsigned int total_rx_bytes=0, total_rx_packets=0;

	i = rx_ring->next_to_clean;
	rx_desc = JZMAC_RX_DESC(*rx_ring, i);
	buffer_info = &rx_ring->buffer_info[i];

	/* except the slot not used, if transfer done, buffer_info->invalid is always 0 */
	while (__desc_rx_transfer_done(rx_desc) && (!buffer_info->invalid)) {
		struct sk_buff *skb;

		if (*work_done >= work_to_do)
			break;
		(*work_done)++;

		buffer_info->invalid = 1;
		skb = buffer_info->skb;
		buffer_info->skb = NULL; /* cleaned */

		prefetch(skb->data - NET_IP_ALIGN);

		if (++i == rx_ring->count) i = 0;

		next_rxd = JZMAC_RX_DESC(*rx_ring, i);
		prefetch(next_rxd);

		next_buffer = &rx_ring->buffer_info[i];

		cleaned = true;
		cleaned_count++;

		length = __desc_get_pkt_size(le32_to_cpu(rx_desc->pkt_size));
#if 0
		printk("============================================\n");
		jzmac_dump_pkt_data((unsigned char *)CKSEG1ADDR(buffer_info->dma),
				    length - 4);
		printk("============================================\n");
#endif
		dma_unmap_single(&lp->netdev->dev,
				 buffer_info->dma, buffer_info->length,
				 DMA_FROM_DEVICE);
		buffer_info->dma = 0;


		/* adjust length to remove Ethernet CRC, this must be
		 * done after the TBI_ACCEPT workaround above */
		length -= 4;

		/* probably a little skewed due to removing CRC */
		total_rx_bytes += length;
		total_rx_packets++;

		/* code added for copybreak, this should improve
		 * performance for small packets with large amounts
		 * of reassembly being done in the stack */
		if (length < copybreak) {
			struct sk_buff *new_skb =
				netdev_alloc_skb(netdev, length + NET_IP_ALIGN);

			if (new_skb) {
				skb_reserve(new_skb, NET_IP_ALIGN);
				skb_copy_to_linear_data_offset(new_skb,
							       -NET_IP_ALIGN,
							       (skb->data -
							        NET_IP_ALIGN),
							       (length +
							        NET_IP_ALIGN));
				/* save the skb in buffer_info as good */
				buffer_info->skb = skb;
				skb = new_skb;
			}
			/* else just continue with the old one */
		}

		/* end copybreak code */
		skb_put(skb, length);
		skb->protocol = eth_type_trans(skb, netdev);

		//jzmac_dump_skb_data(skb);
		netif_receive_skb(skb);
		//netdev->last_rx = jiffies;

		__jzmac_reduce_rx_pktcnt();

		rx_desc->pkt_size = EMPTY_FLAG_MASK;

#if 1
		/* return some buffers to hardware, one at a time is too slow */
		if (unlikely(cleaned_count >= JZMAC_RX_BUFFER_WRITE)) {
			jzmac_alloc_rx_buffers(lp, cleaned_count, 1);
			cleaned_count = 0;
		}
#endif
		/* use prefetched values */
		rx_desc = next_rxd;
		buffer_info = next_buffer;
	}

	rx_ring->next_to_clean = i;

	cleaned_count = JZMAC_DESC_UNUSED(rx_ring);
	if (cleaned_count)
		jzmac_alloc_rx_buffers(lp, cleaned_count, 1);

	lp->total_rx_packets += total_rx_packets;
	lp->total_rx_bytes += total_rx_bytes;
	lp->net_stats.rx_bytes += total_rx_bytes;
	lp->net_stats.rx_packets += total_rx_packets;

	return cleaned;
}

/**
 * jzmac_clean - NAPI Rx polling callback
 **/
static int jzmac_clean(struct napi_struct *napi, int budget) {
	struct jz4770_mac_local *lp = container_of(napi, struct jz4770_mac_local, napi);
	int tx_cleaned = 0;
	int work_done = 0;

	spin_lock(&lp->napi_poll_lock);

	tx_cleaned = jzmac_clean_tx_irq(lp);

	jzmac_clean_rx_irq(lp, &work_done, budget);

	if (!tx_cleaned)
		work_done = budget;

	//printk("===>workdone = %d, budget = %d\n", work_done, budget);
	//jzmac_dump_all_regs(__func__, __LINE__);

	/* If budget not fully consumed, exit the polling mode */
	if (work_done < budget) {
		napi_complete(napi);
		__jzmac_enable_usefull_irq();
	}

	spin_unlock(&lp->napi_poll_lock);
	return work_done;
}



/* interrupt routine to handle rx and error signal */
static irqreturn_t jz4770_mac_interrupt(int irq, void *data)
{
	struct net_device *netdev = data;
	struct jz4770_mac_local *lp = netdev_priv(netdev);
	unsigned int	tx_status;
	unsigned int	rx_status;

	__jzmac_disable_all_irq();

	/* Read tx & rx state reg, which indicate action */
	tx_status = REG32(MAC_DMA_TX_STATUS);
	rx_status = REG32(MAC_DMA_RX_STATUS);

	//printk("===>tx_status = 0x%08x rx_status = 0x%08x\n", tx_status, rx_status);
	//jzmac_dump_all_regs(__func__, __LINE__);
	//jzmac_dump_all_desc(lp);

	__jzmac_tx_status_clr_err();
	__jzmac_rx_status_clr_err();

	if (__jzmac_tx_status_err())
		__jzmac_disable_tx_dma();

	if (__jzmac_rx_status_err())
		__jzmac_disable_rx_dma();

	//counter = (rx_sts & RSR_PKTCNT_MASK) >> 16;
	//if (counter != 0) {
	if (likely(napi_schedule_prep(&lp->napi))) {
		lp->total_tx_bytes = 0;
		lp->total_tx_packets = 0;
		lp->total_rx_bytes = 0;
		lp->total_rx_packets = 0;
		__napi_schedule(&lp->napi);
	} else {
		/* this really should not happen! if it does it is basically a
		 * bug, but not a hard error, so enable ints and continue */
		__jzmac_enable_usefull_irq();
		//__eth_enable_rx_irq();
	}

	return IRQ_HANDLED;
}

/* MAC control and configuration */
static void config_mac(void)
{
	__jzmac_reset_mii();
	__jzmac_reset_all_logic();
	__jzmac_reset_rand_gen();

#if defined(CONFIG_JZ4770_MAC_RMII)
	__jzmac_set_mii_mode();
#endif

	// Enable tx & rx flow control, enable receive
	__jzmac_enable_tx_pause();
	__jzmac_enable_rx_pause();
	__jzmac_mii_enable_rx();


	// Enable loopack mode
	//mac_cfg_1 |= MCR1_LB;

	/* bit 7	bit 6		bit 5
	   MCR2_ADPE	MCR2_VPE	MCR2_PCE
	   x		x		0		No pad, check CRC
	   > 0		0		1		Pad to 60B, append CRC
	   x		1		1		Pad to 64B, append CRC
	   1		0		1		if un-tagged, Pad to 60B, append CRC
	   if VLAN tagged, Pad to 64B, append CRC

	   if set MCR2_PCE(bit 5)
	   MCR2_CE(bit 4) must be set.

	   We need to pad frame to 60B and append 4-byte CRC.
	*/

	//__jzmac_enable_exc_defer();
	//__jzmac_disable_bp_nb();
	//__jzmac_enable_backoff();
	//__jzmac_mii_preamble_any();
	//__jzmac_reject_pure_preamble();
	__jzmac_enable_auto_pad();
	__jzmac_enable_padding();
	__jzmac_enable_crc();
	//__jzmac_disallow_huge_frm();
	__jzmac_enable_length_check();

	__jzmac_enable_full_duplex();

	__jzmac_set_nb2b_ipg2(0x12);
	__jzmac_set_nb2b_ipg1(0xc);
}

static void config_fifo(void)
{
	__jzmac_fifo_reset_all_module();
	REG32(MAC_FIFO_CFG_R0) |= 0x80000000;

	__jzmac_set_min_frm_ram(1536);
	__jzmac_set_xoffrtx(4);

	__jzmac_set_max_rx_watermark(1588);
	//__jzmac_set_max_rx_watermark(1536 * 2);
	__jzmac_set_min_rx_watermark(64);


	/* for 2k FiFo tx */
	__jzmac_set_max_tx_watermark(512);
	//__jzmac_set_max_tx_watermark(0x1b0 * 4);
	__jzmac_set_min_tx_watermark(0x180 * 4);

	__jzmac_enable_pause_frm();

	REG32(MAC_FIFO_CFG_R4) &= 0x0000;
	REG32(MAC_FIFO_CFG_R5) |= 0xffff;

	__jzmac_set_drop_cond(RSV_RUO);

	//__jzmac_set_drop_cond(RSV_DN);

	//__jzmac_set_drop_cond(RSV_MP);
	//__jzmac_set_drop_cond(RSV_BP);

	//__jzmac_set_drop_cond(RSV_LOR); /* seems not worked normal */
	//__jzmac_set_drop_cond(RSV_LCE);

	__jzmac_set_drop_cond(RSV_CRCE);
	__jzmac_set_drop_cond(RSV_RCV);

	__jzmac_fifo_enable_all_module();
}

static void config_sal(void)
{
	__jzmac_accept_multicast_q();
	__jzmac_accept_broadcast();
	__sal_set_hash_table(0, 0);
}

static void config_stat(void)
{
	__jzmac_disable_statistics();
	__jzmac_clear_counters();
	// clear carry registers
	REG32(MAC_STAT_CAR1) = REG32(MAC_STAT_CAR1);
	REG32(MAC_STAT_CAR2) = REG32(MAC_STAT_CAR2);
	__jzmac_stat_disable_carry_irq();

	__jzmac_enable_statistics();

}

static void jz4770_mac_stop_activity(void) {
	__jzmac_disable_tx_dma();
	__jzmac_disable_rx_dma();
	__jzmac_mii_disable_rx();
	__jzmac_disable_all_irq();
}

static void jz4770_mac_disable(struct jz4770_mac_local *lp) {
	/* First ensure that the upper network stack is stopped */
	/* can be netif_tx_disable when NETIF_F_LLTX is removed */
	netif_stop_queue(lp->netdev); /* tx */
	napi_disable(&lp->napi); /* rx */

	jz4770_mac_stop_activity();
	del_timer_sync(&lp->watchdog_timer);

	spin_lock(&lp->napi_poll_lock);

	/* Clear status registers */
	__jzmac_clear_tx_pkt_cnt();
	__jzmac_clear_rx_pkt_cnt();
	__jzmac_tx_status_clr_err();
	__jzmac_rx_status_clr_err();

	__jzmac_fifo_reset_all_module();
	__jzmac_fifo_disable_all_module();

	desc_list_reinit(lp);

	spin_unlock(&lp->napi_poll_lock);
}

static void jz4770_mac_configure(struct jz4770_mac_local *lp) {
	/* Wait MII done */
#ifndef CONFIG_MDIO_GPIO
	jz4770_mdio_poll();
#endif

	__jzmac_disable_tx_dma();
	__jzmac_disable_rx_dma();

	__jzmac_mii_disable_rx();
	__jzmac_disable_all_irq();

	config_mac();

	config_fifo();

	config_sal();

	config_stat();

	__jzmac_set_tx_desc_addr(lp->tx_ring.dma);
	__jzmac_set_rx_desc_addr(lp->rx_ring.dma);

	/* Burst length: 4, 8 or 16 */
	//__eth_dma_set_burst_len(BURST_LEN_4);

	/* Clear status registers */
	__jzmac_clear_tx_pkt_cnt();
	__jzmac_clear_rx_pkt_cnt();
	__jzmac_tx_status_clr_err();
	__jzmac_rx_status_clr_err();
}

/*
 * Enable Interrupts, Receive, and Transmit(The same sequence as jz4770_mac_open, only a bit different)
 */
static void jz4770_mac_enable(struct jz4770_mac_local *lp) {
	jz4770_mac_configure(lp);

	napi_enable(&lp->napi);
	__jzmac_enable_usefull_irq();
	/* we only enable rx here */
	__jzmac_enable_rx_dma();
	/* We can accept TX packets again */
	lp->netdev->trans_start = jiffies;
	netif_wake_queue(lp->netdev);
}

static void jzmac_reinit_locked(struct jz4770_mac_local *lp)
{
	WARN_ON(in_interrupt());
	jz4770_mac_disable(lp);
	jz4770_mac_enable(lp);
}

/* Our watchdog timed out. Called by the networking layer */
static void jz4770_mac_tx_timeout(struct net_device *dev)
{
	struct jz4770_mac_local *lp = netdev_priv(dev);

	/* Do the reset outside of interrupt context */
	lp->tx_timeout_count++;
	schedule_work(&lp->reset_task);
}

static void jzmac_reset_task(struct work_struct *work)
{
	struct jz4770_mac_local *lp =
		container_of(work, struct jz4770_mac_local, reset_task);

	jzmac_reinit_locked(lp);
}

/*
 * Multicast filter and config multicast hash table
 */
#define MULTICAST_FILTER_LIMIT 64

static void jzmac_multicast_hash(struct net_device *dev)
{
	u32 emac_hashhi, emac_hashlo;
	struct dev_mc_list *dmi = dev->mc_list;
	char *addrs;
	int i;
	u32 crc;

	emac_hashhi = emac_hashlo = 0;

	for (i = 0; i < dev->mc_count; i++) {
		addrs = dmi->dmi_addr;
		dmi = dmi->next;

		/* skip non-multicast addresses */
		if (!(*addrs & 1))
			continue;

		crc = ether_crc(ETH_ALEN, addrs);
		crc >>= 26;

		if (crc & 0x20)
			emac_hashhi |= 1 << (crc & 0x1f);
		else
			emac_hashlo |= 1 << (crc & 0x1f);
	}

	__sal_set_hash_table(emac_hashhi, emac_hashlo);

	return;
}

static void jzmac_set_multicast_list(struct net_device *dev)
{
	if (dev->flags & IFF_PROMISC) {
		/* Accept any kinds of packets */
		__jzmac_enable_promiscuous();

		/* FIXME: need save the previous value */
		__sal_set_hash_table(0xffffffff, 0xffffffff);
		printk("%s: Enter promisc mode!\n",dev->name);
	} else  if ((dev->flags & IFF_ALLMULTI) || (dev->mc_count > MULTICAST_FILTER_LIMIT)) {
		/* Accept all multicast packets */

		__jzmac_enable_promiscuous();
		__jzmac_accept_broadcast();
		__jzmac_clr_drop_cond(RSV_MP);

		__sal_set_hash_table(0xffffffff, 0xffffffff);
		printk("%s: Enter allmulticast mode!   %d \n",dev->name,dev->mc_count);
	} else if (dev->mc_count) {
		/* Update multicast hash table */
		jzmac_multicast_hash(dev);
		__jzmac_accept_multicast_q();
	} else {
		/*FIXME: clear promisc or multicast mode */
		__jzmac_disable_promiscuous();
	}
}

static void setup_mac_addr(u8 *mac_addr) {
	REG16(MAC_MII_SA0) = ((mac_addr[0]) << 8) | mac_addr[1];
	REG16(MAC_MII_SA1) = ((mac_addr[2]) << 8) | mac_addr[3];
	REG16(MAC_MII_SA2) = ((mac_addr[4]) << 8) | mac_addr[5];
}

static int jz4770_mac_set_mac_address(struct net_device *dev, void *p) {
	struct sockaddr *addr = p;
	if (netif_running(dev))
		return -EBUSY;
	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);
	setup_mac_addr(dev->dev_addr);
	return 0;
}

/*
 * Open and Initialize the interface
 *
 * Set up everything, reset the card, etc..
 */
static int jz4770_mac_open(struct net_device *dev)
{
	struct jz4770_mac_local *lp = netdev_priv(dev);
	int retval;

	pr_debug("%s: %s\n", dev->name, __func__);

	/*
	 * Check that the address is valid.  If its not, refuse
	 * to bring the device up.  The user must specify an
	 * address using ifconfig eth0 hw ether xx:xx:xx:xx:xx:xx
	 */
	if (!is_valid_ether_addr(dev->dev_addr)) {
		printk(KERN_WARNING JZMAC_DRV_NAME ": no valid ethernet hw addr\n");
		return -EINVAL;
	}

	/* initial rx and tx list */
	retval = desc_list_init(lp);

	if (retval)
		return retval;

	jz4770_mac_stop_activity();

	phy_write(lp->phydev, MII_BMCR, BMCR_RESET);
	//while(phy_read(lp->phydev, MII_BMCR) & BMCR_RESET);
	phy_start(lp->phydev);

	setup_mac_addr(dev->dev_addr);
	jz4770_mac_configure(lp);

	napi_enable(&lp->napi);
	__jzmac_enable_usefull_irq();

	/* we only enable rx here */
	__jzmac_enable_rx_dma();
	/* We can accept TX packets again */
	lp->netdev->trans_start = jiffies;
	netif_start_queue(dev);

	return 0;
}

/*
 * this makes the board clean up everything that it can
 * and not talk to the outside world.   Caused by
 * an 'ifconfig ethX down'
 */
static int jz4770_mac_close(struct net_device *dev)
{
	struct jz4770_mac_local *lp = netdev_priv(dev);

	jz4770_mac_disable(lp);

	netif_carrier_off(dev);

	phy_stop(lp->phydev);
	phy_write(lp->phydev, MII_BMCR, BMCR_PDOWN);

	/* free the rx/tx buffers */
	desc_list_free(lp);
	return 0;
}

static struct net_device_stats *jz4770_mac_get_stats(struct net_device *netdev)
{
	struct jz4770_mac_local *lp = netdev_priv(netdev);

	/* only return the current stats */
	return &lp->net_stats;
}

static int jz4770_mac_change_mtu(struct net_device *netdev, int new_mtu) {
	printk("===>new_mtu = %d\n", new_mtu);
	return eth_change_mtu(netdev, new_mtu);
}

static const struct net_device_ops jz4770_mac_netdev_ops = {
	.ndo_open		= jz4770_mac_open,
	.ndo_stop		= jz4770_mac_close,
	.ndo_start_xmit		= jz4770_mac_hard_start_xmit,
	.ndo_get_stats		= jz4770_mac_get_stats,
	.ndo_set_mac_address	= jz4770_mac_set_mac_address,
	.ndo_tx_timeout		= jz4770_mac_tx_timeout,
	.ndo_set_multicast_list	= jzmac_set_multicast_list,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= jz4770_mac_change_mtu,
	//.ndo_change_mtu		= eth_change_mtu,
};

static int __devinit jz4770_mac_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	struct jz4770_mac_local *lp;
	struct platform_device *pd;
	int rc;
	int i;

	jz4770_mac_stop_activity();

	ndev = alloc_etherdev(sizeof(struct jz4770_mac_local));
	if (!ndev) {
		dev_err(&pdev->dev, "Cannot allocate net device!\n");
		return -ENOMEM;
	}

	SET_NETDEV_DEV(ndev, &pdev->dev);
	platform_set_drvdata(pdev, ndev);
	lp = netdev_priv(ndev);
	lp->netdev = ndev;
	lp->pdev = pdev;

	/* configure MAC address */
	if (bootargs_ethaddr) {
		for (i=0; i<6; i++)
			ndev->dev_addr[i] = ethaddr_hex[i];
	} else
		random_ether_addr(ndev->dev_addr);

	setup_mac_addr(ndev->dev_addr);

	if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "Cannot get platform device jz4770_mii_bus!\n");
		rc = -ENODEV;
		goto out_err_probe_mac;
	}
	pd = pdev->dev.platform_data;
	lp->mii_bus = platform_get_drvdata(pd);
#ifndef CONFIG_MDIO_GPIO
	lp->mii_bus->priv = ndev;
#endif

	rc = mii_probe(ndev);
	if (rc) {
		dev_err(&pdev->dev, "MII Probe failed!\n");
		goto out_err_mii_probe;
	}

	/* Fill in the fields of the device structure with ethernet values. */
	ether_setup(ndev);

	ndev->netdev_ops = &jz4770_mac_netdev_ops;
	//ndev->ethtool_ops = &jz4770_mac_ethtool_ops;
	ndev->watchdog_timeo = 2 * HZ;
	netif_napi_add(ndev, &lp->napi, jzmac_clean, 32);

	spin_lock_init(&lp->link_lock);
	spin_lock_init(&lp->napi_poll_lock);

	init_timer(&lp->watchdog_timer);
	lp->watchdog_timer.function = &jzmac_watchdog;
	lp->watchdog_timer.data = (unsigned long)lp;

	INIT_WORK(&lp->reset_task, jzmac_reset_task);

	/* register irq handler */
	rc = request_irq(IRQ_ETH, jz4770_mac_interrupt,
			 IRQF_DISABLED, "jzmac", ndev);
	if (rc) {
		dev_err(&pdev->dev, "Cannot request JZ4770 MAC IRQ!\n");
		rc = -EBUSY;
		goto out_err_request_irq;
	}

	strcpy(ndev->name, "eth%d");
	rc = register_netdev(ndev);
	if (rc) {
		dev_err(&pdev->dev, "Cannot register net device!\n");
		goto out_err_reg_ndev;
	}

	/* now, print out the card info, in a short format.. */
	dev_info(&pdev->dev, "%s, Version %s\n", JZMAC_DRV_DESC, JZMAC_DRV_VERSION);

	return 0;

 out_err_reg_ndev:
	free_irq(IRQ_ETH, ndev);
 out_err_request_irq:
 out_err_mii_probe:
#ifndef CONFIG_MDIO_GPIO
	mdiobus_unregister(lp->mii_bus);
	mdiobus_free(lp->mii_bus);
#endif
 out_err_probe_mac:
	platform_set_drvdata(pdev, NULL);
	free_netdev(ndev);

	return rc;
}

static int __devexit jz4770_mac_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct jz4770_mac_local *lp = netdev_priv(ndev);

	platform_set_drvdata(pdev, NULL);

#ifndef CONFIG_MDIO_GPIO
	lp->mii_bus->priv = NULL;
#endif

	unregister_netdev(ndev);

	free_irq(IRQ_ETH, ndev);

	free_netdev(ndev);

	return 0;
}

#ifdef CONFIG_PM
static int jz4770_mac_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct net_device *net_dev = platform_get_drvdata(pdev);

	if (netif_running(net_dev))
		jz4770_mac_close(net_dev);

	return 0;
}

static int jz4770_mac_resume(struct platform_device *pdev)
{
	struct net_device *net_dev = platform_get_drvdata(pdev);

	if (netif_running(net_dev))
		jz4770_mac_open(net_dev);

	return 0;
}
#else
#define jz4770_mac_suspend NULL
#define jz4770_mac_resume NULL
#endif	/* CONFIG_PM */

#ifndef CONFIG_MDIO_GPIO
/* MII Bus platform routines */
/*
 * MII operations
 */

/* Wait until the previous MDC/MDIO transaction has completed */
static void jz4770_mdio_poll(void)
{
	int timeout_cnt = MAX_TIMEOUT_CNT;

	while (__jzmac_mii_is_busy()) {
		udelay(1);
		if (timeout_cnt-- < 0) {
			printk(KERN_ERR JZMAC_DRV_NAME
			       ": wait MDC/MDIO transaction to complete timeout\n");
			break;
		}
	}
}

/* Read an off-chip register in a PHY through the MDC/MDIO port */
static int jz4770_mdiobus_read(struct mii_bus *bus, int phy_addr, int regnum)
{
	int retval;

	__jzmac_mii_mgt_leave_reset();
	jz4770_mdio_poll();

	__jzmac_send_mii_read_cmd(phy_addr, regnum, 0);

	jz4770_mdio_poll();

	retval = (int)__jzmac_mii_read_data();

	__jzmac_mii_mgt_enter_reset();

	return retval;
}

/* Write an off-chip register in a PHY through the MDC/MDIO port */
static int jz4770_mdiobus_write(struct mii_bus *bus, int phy_addr, int regnum,
				u16 value)
{
	__jzmac_mii_mgt_leave_reset();
	jz4770_mdio_poll();

	__jzmac_send_mii_write_cmd(phy_addr, regnum, value);

	jz4770_mdio_poll();
	__jzmac_mii_mgt_enter_reset();

	return 0;
}

static int jz4770_mdiobus_reset(struct mii_bus *bus)
{
	return 0;
}

static int __devinit jz4770_mii_bus_probe(struct platform_device *pdev)
{
	struct mii_bus *miibus;
	int rc, i;
	u32 mdc_div;

	cpm_start_clock(CGM_MAC);
	__gpio_as_eth();

#ifdef JZMAC_PHY_RESET_PIN
	__gpio_as_output0(JZMAC_PHY_RESET_PIN);
	mdelay(10);
        __gpio_as_output1(JZMAC_PHY_RESET_PIN);
#endif

	__jzmac_disable_all_irq();

	mdc_div = cpm_get_clock(CGU_H2CLK) / JZMAC_MDC_CLK;
	if (mdc_div > 7) mdc_div = 7;
	__jzmac_set_mdc_clkdiv(mdc_div);
	__jzmac_mdc_with_preamble();
	__jzmac_reset_mii();
	__jzmac_reset_all_logic();
	__jzmac_reset_rand_gen();
#if defined(CONFIG_JZ4770_MAC_RMII)
	__jzmac_set_mii_mode();
#endif
	__jzmac_mii_mgt_enter_reset();

	rc = -ENOMEM;
	miibus = mdiobus_alloc();
	if (miibus == NULL)
		goto out_err_alloc;
	miibus->read = jz4770_mdiobus_read;
	miibus->write = jz4770_mdiobus_write;
	miibus->reset = jz4770_mdiobus_reset;

	miibus->parent = &pdev->dev;
	miibus->name = "jz4770_mii_bus";
	snprintf(miibus->id, MII_BUS_ID_SIZE, "0");
	miibus->irq = kmalloc(sizeof(int)*PHY_MAX_ADDR, GFP_KERNEL);
	if (miibus->irq == NULL)
		goto out_err_alloc;
	for (i = 0; i < PHY_MAX_ADDR; ++i)
		miibus->irq[i] = PHY_POLL;

	rc = mdiobus_register(miibus);
	if (rc) {
		dev_err(&pdev->dev, "Cannot register MDIO bus!\n");
		goto out_err_mdiobus_register;
	}

	platform_set_drvdata(pdev, miibus);

	return 0;

 out_err_mdiobus_register:
	mdiobus_free(miibus);
 out_err_alloc:
	return rc;
}

static int __devexit jz4770_mii_bus_remove(struct platform_device *pdev)
{
	struct mii_bus *miibus = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	mdiobus_unregister(miibus);
	mdiobus_free(miibus);
	return 0;
}


static struct platform_driver jz4770_mii_bus_driver = {
	.probe = jz4770_mii_bus_probe,
	.remove = __devexit_p(jz4770_mii_bus_remove),
	.driver = {
		.name = "jz4770_mii_bus",
		.owner	= THIS_MODULE,
	},
};
#else  /* CONFIG_MDIO_GPIO */
static void jz4770_mdio_gpio_init(void) {
	cpm_start_clock(CGM_MAC);
	__gpio_as_eth();

#ifdef JZMAC_PHY_RESET_PIN
	__gpio_as_output0(JZMAC_PHY_RESET_PIN);
	mdelay(10);
        __gpio_as_output1(JZMAC_PHY_RESET_PIN);
#endif

	__jzmac_disable_all_irq();
}
#endif	/* CONFIG_MDIO_GPIO */

static struct platform_driver jz4770_mac_driver = {
	.probe = jz4770_mac_probe,
	.remove = __devexit_p(jz4770_mac_remove),
	.resume = jz4770_mac_resume,
	.suspend = jz4770_mac_suspend,
	.driver = {
		.name = JZMAC_DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init jz4770_mac_init(void)
{
//	__gpio_as_output1(BUS_RJ45_PWR_EN); //allen add
	
#ifndef CONFIG_MDIO_GPIO
	int ret;
	ret = platform_driver_register(&jz4770_mii_bus_driver);
	if (!ret)
		return platform_driver_register(&jz4770_mac_driver);
	return -ENODEV;
#else
	return platform_driver_register(&jz4770_mac_driver);
#endif
}

module_init(jz4770_mac_init);

static void __exit jz4770_mac_cleanup(void)
{
	platform_driver_unregister(&jz4770_mac_driver);
#ifndef CONFIG_MDIO_GPIO
	platform_driver_unregister(&jz4770_mii_bus_driver);
#endif
}

module_exit(jz4770_mac_cleanup);

