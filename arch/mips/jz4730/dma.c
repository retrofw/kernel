/*
 * linux/arch/mips/jz4730/dma.c
 *
 * JZ4730 DMA PC-like APIs.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/soundcard.h>

#include <asm/system.h>
#include <asm/addrspace.h>
#include <asm/jzsoc.h>

/*
 * A note on resource allocation:
 *
 * All drivers needing DMA channels, should allocate and release them
 * through the public routines `jz_request_dma()' and `jz_free_dma()'.
 *
 * In order to avoid problems, all processes should allocate resources in
 * the same sequence and release them in the reverse order.
 *
 * So, when allocating DMAs and IRQs, first allocate the DMA, then the IRQ.
 * When releasing them, first release the IRQ, then release the DMA. The
 * main reason for this order is that, if you are requesting the DMA buffer
 * done interrupt, you won't know the irq number until the DMA channel is
 * returned from jz_request_dma().
 */

struct jz_dma_chan jz_dma_table[NUM_DMA] = {
	{dev_id:-1,},
	{dev_id:-1,},
	{dev_id:-1,},
	{dev_id:-1,},
	{dev_id:-1,},
	{dev_id:-1,},
};


// Device FIFO addresses and default DMA modes
static const struct {
	unsigned int fifo_addr;
	unsigned int dma_mode;
	unsigned int dma_source;
} dma_dev_table[NUM_DMA_DEV] = {
	{CPHYSADDR(UART0_BASE), DMA_8bit_TX_CONF|DMA_MODE_WRITE, DMAC_DRSR_RS_UART0OUT},
	{CPHYSADDR(UART0_BASE), DMA_8bit_RX_CONF|DMA_MODE_READ, DMAC_DRSR_RS_UART0IN},
	{CPHYSADDR(UART1_BASE), DMA_8bit_TX_CONF|DMA_MODE_WRITE, DMAC_DRSR_RS_UART1OUT},
	{CPHYSADDR(UART1_BASE), DMA_8bit_RX_CONF|DMA_MODE_READ, DMAC_DRSR_RS_UART1IN},
	{CPHYSADDR(UART2_BASE), DMA_8bit_TX_CONF|DMA_MODE_WRITE, DMAC_DRSR_RS_UART2OUT},
	{CPHYSADDR(UART2_BASE), DMA_8bit_RX_CONF|DMA_MODE_READ, DMAC_DRSR_RS_UART2IN},
	{CPHYSADDR(UART3_BASE), DMA_8bit_TX_CONF|DMA_MODE_WRITE, DMAC_DRSR_RS_UART3OUT},
	{CPHYSADDR(UART3_BASE), DMA_8bit_RX_CONF|DMA_MODE_READ, DMAC_DRSR_RS_UART3IN},
	{CPHYSADDR(SSI_DR), DMA_32bit_TX_CONF|DMA_MODE_WRITE, DMAC_DRSR_RS_SSIOUT},
	{CPHYSADDR(SSI_DR), DMA_32bit_RX_CONF|DMA_MODE_READ, DMAC_DRSR_RS_SSIIN},
	{CPHYSADDR(MSC_TXFIFO), DMA_32bit_TX_CONF|DMA_MODE_WRITE, DMAC_DRSR_RS_MSCOUT},
	{CPHYSADDR(MSC_RXFIFO), DMA_32bit_RX_CONF|DMA_MODE_READ, DMAC_DRSR_RS_MSCIN},
	{CPHYSADDR(AIC_DR), DMA_32bit_TX_CONF|DMA_MODE_WRITE, DMAC_DRSR_RS_AICOUT},
	{CPHYSADDR(AIC_DR), DMA_32bit_RX_CONF|DMA_MODE_READ, DMAC_DRSR_RS_AICIN},
	{0, DMA_AUTOINIT, 0},
};


int jz_dma_read_proc(char *buf, char **start, off_t fpos,
			 int length, int *eof, void *data)
{
	int i, len = 0;
	struct jz_dma_chan *chan;

	for (i = 0; i < NUM_DMA; i++) {
		if ((chan = get_dma_chan(i)) != NULL) {
			len += sprintf(buf + len, "%2d: %s\n",
				       i, chan->dev_str);
		}
	}

	if (fpos >= len) {
		*start = buf;
		*eof = 1;
		return 0;
	}
	*start = buf + fpos;
	if ((len -= fpos) > length)
		return length;
	*eof = 1;
	return len;
}


void dump_jz_dma_channel(unsigned int dmanr)
{
	struct jz_dma_chan *chan;

	if (dmanr > NUM_DMA)
		return;
	chan = &jz_dma_table[dmanr];

	printk(KERN_INFO "DMA%d Register Dump:\n", dmanr);
	printk(KERN_INFO "  DMACR= 0x%08x\n", REG_DMAC_DMACR);
	printk(KERN_INFO "  DSAR = 0x%08x\n", REG_DMAC_DSAR(dmanr));
	printk(KERN_INFO "  DDAR = 0x%08x\n", REG_DMAC_DDAR(dmanr));
	printk(KERN_INFO "  DTCR = 0x%08x\n", REG_DMAC_DTCR(dmanr));
	printk(KERN_INFO "  DRSR = 0x%08x\n", REG_DMAC_DRSR(dmanr));
	printk(KERN_INFO "  DCCSR = 0x%08x\n", REG_DMAC_DCCSR(dmanr));
}


/**
 * jz_request_dma - dynamically allcate an idle DMA channel to return
 * @dev_id: the specified dma device id or DMA_ID_RAW_REQ
 * @dev_str: the specified dma device string name
 * @irqhandler: the irq handler, or NULL
 * @irqflags: the irq handler flags
 * @irq_dev_id: the irq handler device id for shared irq
 *
 * Finds a free channel, and binds the requested device to it.
 * Returns the allocated channel number, or negative on error.
 * Requests the DMA done IRQ if irqhandler != NULL.
 *
*/
int jz_request_dma(int dev_id, const char *dev_str,
		     irqreturn_t (*irqhandler)(int, void *),
		     unsigned long irqflags,
		     void *irq_dev_id)
{
	struct jz_dma_chan *chan;
	int i, ret;

	if (dev_id < 0 || dev_id >= NUM_DMA_DEV)
		return -EINVAL;

	for (i = 0; i < NUM_DMA; i++) {
		if (jz_dma_table[i].dev_id < 0)
			break;
	}
	if (i == NUM_DMA)
		return -ENODEV;

	chan = &jz_dma_table[i];

	if (irqhandler) {
		chan->irq = IRQ_DMA_0 + i;	// see intc.h
		chan->irq_dev = irq_dev_id;
		if ((ret = request_irq(chan->irq, irqhandler, irqflags,
				       dev_str, chan->irq_dev))) {
			chan->irq = 0;
			chan->irq_dev = NULL;
			return ret;
		}
	} else {
		chan->irq = 0;
		chan->irq_dev = NULL;
	}

	// fill it in
	chan->io = i;
	chan->dev_id = dev_id;
	chan->dev_str = dev_str;
	chan->fifo_addr = dma_dev_table[dev_id].fifo_addr;
	chan->mode = dma_dev_table[dev_id].dma_mode;
	chan->source = dma_dev_table[dev_id].dma_source;

	return i;
}

void jz_free_dma(unsigned int dmanr)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	if (!chan) {
		printk("Trying to free DMA%d\n", dmanr);
		return;
	}

	disable_dma(dmanr);
	if (chan->irq)
		free_irq(chan->irq, chan->irq_dev);

	chan->irq = 0;
	chan->irq_dev = NULL;
	chan->dev_id = -1;
}

void jz_set_dma_dest_width(int dmanr, int nbit)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	if (!chan)
	       	return;
	chan->mode &= ~DMAC_DCCSR_DWDH_MASK;
	switch (nbit) {
	case 8:
		chan->mode |= DMAC_DCCSR_DWDH_8;
		break;
	case 16:
		chan->mode |= DMAC_DCCSR_DWDH_16;
		break;
	case 32:
		chan->mode |= DMAC_DCCSR_DWDH_32;
		break;
	}
}

void jz_set_dma_src_width(int dmanr, int nbit)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	if (!chan)
	       	return;
	chan->mode &= ~DMAC_DCCSR_SWDH_MASK;
	switch (nbit) {
	case 8:
		chan->mode |= DMAC_DCCSR_SWDH_8;
		break;
	case 16:
		chan->mode |= DMAC_DCCSR_SWDH_16;
		break;
	case 32:
		chan->mode |= DMAC_DCCSR_SWDH_32;
		break;
	}
}

void jz_set_dma_block_size(int dmanr, int nbyte)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	if (!chan)
		return;
	chan->mode &= ~DMAC_DCCSR_DS_MASK;
	switch (nbyte) {
	case 1:
		chan->mode |= DMAC_DCCSR_DS_8b;
		break;
	case 2:
		chan->mode |= DMAC_DCCSR_DS_16b;
		break;
	case 4:
		chan->mode |= DMAC_DCCSR_DS_32b;
		break;
	case 16:
		chan->mode |= DMAC_DCCSR_DS_16B;
		break;
	case 32:
		chan->mode |= DMAC_DCCSR_DS_32B;
		break;
	}
}

/**
 * jz_set_dma_mode - do the raw settings for the specified DMA channel
 * @dmanr: the specified DMA channel
 * @mode: dma operate mode, DMA_MODE_READ or DMA_MODE_WRITE
 * @dma_mode: dma raw mode
 * @dma_source: dma raw request source
 * @fifo_addr: dma raw device fifo address
 *
 * Ensure call jz_request_dma(DMA_ID_RAW_REQ, ...) first, then call
 * jz_set_dma_mode() rather than set_dma_mode() if you work with
 * and external request dma device.
 *
 * NOTE: Don not dynamically allocate dma channel if one external request
 *       dma device will occupy this channel.
*/
int jz_set_dma_mode(unsigned int dmanr, unsigned int mode,
		      unsigned int dma_mode, unsigned int dma_source,
		      unsigned int fifo_addr)
{
	int dev_id, i;
	struct jz_dma_chan *chan;

	if (dmanr > NUM_DMA)
		return -ENODEV;
	for (i = 0; i < NUM_DMA; i++) {
		if (jz_dma_table[i].dev_id < 0)
			break;
	}
	if (i == NUM_DMA)
		return -ENODEV;

	chan = &jz_dma_table[dmanr];
	dev_id = chan->dev_id;
	if (dev_id > 0) {
		printk(KERN_DEBUG "%s sets the allocated DMA channel %d!\n",
		       __FUNCTION__, dmanr);
		return -ENODEV;
	}

	/* clone it from the dynamically allocated. */
	if (i != dmanr) {
		chan->irq = jz_dma_table[i].irq;
		chan->irq_dev = jz_dma_table[i].irq_dev;
		chan->dev_str = jz_dma_table[i].dev_str;
		jz_dma_table[i].irq = 0;
		jz_dma_table[i].irq_dev = NULL;
		jz_dma_table[i].dev_id = -1;
	}
	chan->dev_id = DMA_ID_RAW_SET;
	chan->io = dmanr;
	chan->fifo_addr = fifo_addr;
	chan->mode = dma_mode;
	chan->source = dma_source;

	set_dma_mode(dmanr, dma_mode);

	return dmanr;
}

void enable_dma(unsigned int dmanr)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	if (!chan)
		return;

	REG_DMAC_DCCSR(chan->io) &= ~(DMAC_DCCSR_HLT | DMAC_DCCSR_TC | DMAC_DCCSR_AR);
	__dmac_enable_channel(dmanr);
	if (chan->irq)
		__dmac_channel_enable_irq(dmanr);
}

#define DMA_DISABLE_POLL 0x5000

void disable_dma(unsigned int dmanr)
{
	int i;
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	if (!chan)
		return;
	if (!__dmac_channel_enabled(dmanr))
		return;

	for (i = 0; i < DMA_DISABLE_POLL; i++)
		if (__dmac_channel_transmit_end_detected(dmanr))
			break;
#if 0
	if (i == DMA_DISABLE_POLL)
		printk(KERN_INFO "disable_dma: poll expired!\n");
#endif

	__dmac_disable_channel(dmanr);
	if (chan->irq)
		__dmac_channel_disable_irq(dmanr);
}

/* note: DMA_MODE_MASK is simulated by sw, DCCSR_MODE_MASK mask hw bits */
void set_dma_mode(unsigned int dmanr, unsigned int mode)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	if (!chan)
		return;
	mode &= ~(DMAC_DCCSR_TC | DMAC_DCCSR_AR);
	chan->mode |= mode & ~(DMAC_DCCSR_SAM | DMAC_DCCSR_EACKM | DMAC_DCCSR_DAM);
	mode &= DMA_MODE_MASK;
	if (mode == DMA_MODE_READ) {
		chan->mode |= DMAC_DCCSR_DAM;
		chan->mode &= ~DMAC_DCCSR_SAM;
	} else if (mode == DMA_MODE_WRITE) {
		chan->mode |= DMAC_DCCSR_SAM | DMAC_DCCSR_EACKM;
		chan->mode &= ~DMAC_DCCSR_DAM;
	} else {
		printk(KERN_DEBUG "set_dma_mode() support DMA_MODE_READ or DMA_MODE_WRITE!\n");
	}
	REG_DMAC_DCCSR(chan->io) = chan->mode & ~DMA_MODE_MASK;
	REG_DMAC_DRSR(chan->io) = chan->source;
}

void jz_set_oss_dma(unsigned int dmanr, unsigned int mode, unsigned int audio_fmt)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);

	if (!chan)
		return;

	switch (audio_fmt) {
	case AFMT_U8:
		/* SNDRV_PCM_FORMAT_S8 burst mode : 32BIT */
		break;
	case AFMT_S16_LE:
		/* SNDRV_PCM_FORMAT_S16_LE burst mode : 16BYTE */
		if (mode == DMA_MODE_READ) {
			mode &= ~(DMAC_DCCSR_TC | DMAC_DCCSR_AR);
			chan->mode = DMA_AIC_32_16BYTE_RX_CMD | DMA_MODE_READ;
			chan->mode |= mode & ~(DMAC_DCCSR_SAM | DMAC_DCCSR_EACKM | DMAC_DCCSR_DAM);
			mode &= DMA_MODE_MASK;
			chan->mode |= DMAC_DCCSR_DAM;
			chan->mode &= ~DMAC_DCCSR_SAM;
		} else if (mode == DMA_MODE_WRITE) {
			mode &= ~(DMAC_DCCSR_TC | DMAC_DCCSR_AR);
			chan->mode = DMA_AIC_32_16BYTE_TX_CMD | DMA_MODE_WRITE;
			chan->mode |= mode & ~(DMAC_DCCSR_SAM | DMAC_DCCSR_EACKM |DMAC_DCCSR_DAM);
			mode &= DMA_MODE_MASK;
			chan->mode |= DMAC_DCCSR_SAM | DMAC_DCCSR_EACKM;
			chan->mode &= ~DMAC_DCCSR_DAM;
		} else
			printk("jz_set_oss_dma() just supports DMA_MODE_READ or DMA_MODE_WRITE!\n");
		
		REG_DMAC_DCCSR(chan->io) = chan->mode & ~DMA_MODE_MASK;
		REG_DMAC_DRSR(chan->io) = chan->source;
		break;
	}
}

void jz_set_alsa_dma(unsigned int dmanr, unsigned int mode, unsigned int audio_fmt)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);

	if (!chan)
		return;

	switch (audio_fmt) {
	case 8:
		/* SNDRV_PCM_FORMAT_S8 burst mode : 32BIT */
		break;
	case 16:
		/* SNDRV_PCM_FORMAT_S16_LE burst mode : 16BYTE */
		if (mode == DMA_MODE_READ) {
			mode &= ~(DMAC_DCCSR_TC | DMAC_DCCSR_AR);
			chan->mode = DMA_AIC_16BYTE_RX_CMD | DMA_MODE_READ;
			chan->mode |= mode & ~(DMAC_DCCSR_SAM | DMAC_DCCSR_EACKM | DMAC_DCCSR_DAM);
			mode &= DMA_MODE_MASK;
			chan->mode |= DMAC_DCCSR_DAM;
			chan->mode &= ~DMAC_DCCSR_SAM;
		} else if (mode == DMA_MODE_WRITE) {
			mode &= ~(DMAC_DCCSR_TC | DMAC_DCCSR_AR);
			chan->mode = DMA_AIC_16BYTE_TX_CMD | DMA_MODE_WRITE;
			chan->mode |= mode & ~(DMAC_DCCSR_SAM | DMAC_DCCSR_EACKM | DMAC_DCCSR_DAM);
			mode &= DMA_MODE_MASK;
			chan->mode |= DMAC_DCCSR_SAM | DMAC_DCCSR_EACKM;
			chan->mode &= ~DMAC_DCCSR_DAM;
		} else
			printk("jz_set_alsa_dma() just supports DMA_MODE_READ or DMA_MODE_WRITE!\n");
		
		REG_DMAC_DCCSR(chan->io) = chan->mode & ~DMA_MODE_MASK;
		REG_DMAC_DRSR(chan->io) = chan->source;
		break;
	}
}

void set_dma_addr(unsigned int dmanr, unsigned int a)
{
	unsigned int mode;
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	if (!chan)
		return;
	mode = chan->mode & DMA_MODE_MASK;
	if (mode == DMA_MODE_READ) {
		REG_DMAC_DSAR(chan->io) = chan->fifo_addr;
		REG_DMAC_DDAR(chan->io) = a;
	} else if (mode == DMA_MODE_WRITE) {
		REG_DMAC_DSAR(chan->io) = a;
		REG_DMAC_DDAR(chan->io) = chan->fifo_addr;
	} else
		printk(KERN_DEBUG "Driver should call set_dma_mode() ahead set_dma_addr()!\n");
}

void set_dma_count(unsigned int dmanr, unsigned int count)
{
	unsigned int mode;
	int dma_ds[] = {4, 1, 2, 16, 32};
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	if (!chan)
	       	return;
       	mode = (chan->mode & DMAC_DCCSR_DS_MASK) >> DMAC_DCCSR_DS_BIT;
	count = count / dma_ds[mode];
	REG_DMAC_DTCR(chan->io) = count;
}

int get_dma_residue(unsigned int dmanr)
{
	int count;
	unsigned int mode;
	int dma_ds[] = {4, 1, 2, 16, 32};
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	if (!chan)
		return 0;

	mode = (chan->mode & DMAC_DCCSR_DS_MASK) >> DMAC_DCCSR_DS_BIT;
	count = REG_DMAC_DTCR(chan->io);
	count = count * dma_ds[mode];

	return count;
}

EXPORT_SYMBOL(jz_dma_table);
EXPORT_SYMBOL(jz_request_dma);
EXPORT_SYMBOL(jz_free_dma);
EXPORT_SYMBOL(jz_set_dma_src_width);
EXPORT_SYMBOL(jz_set_dma_dest_width);
EXPORT_SYMBOL(jz_set_dma_block_size);
EXPORT_SYMBOL(jz_set_dma_mode);
EXPORT_SYMBOL(set_dma_mode);
EXPORT_SYMBOL(jz_set_oss_dma);
EXPORT_SYMBOL(jz_set_alsa_dma);
EXPORT_SYMBOL(set_dma_addr);
EXPORT_SYMBOL(set_dma_count);
EXPORT_SYMBOL(get_dma_residue);
EXPORT_SYMBOL(enable_dma);
EXPORT_SYMBOL(disable_dma);
EXPORT_SYMBOL(dump_jz_dma_channel);
