/*
 *  linux/include/asm-mips/mach-jz4730/dma.h
 *
 *  JZ4730 DMA definition.
 *
 *  Copyright (C) 2006 - 2007 Ingenic Semiconductor Inc.
 *
 *  Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4730_DMA_H__
#define __ASM_JZ4730_DMA_H__

#include <linux/interrupt.h>
#include <asm/io.h>			/* need byte IO */
#include <linux/spinlock.h>		/* And spinlocks */
#include <linux/delay.h>
#include <asm/system.h>

#define DMA_UNIT_32     32
#define DMA_UNIT_16     16


/* block-mode  EOP: high  DREQ: high  DACK: low*/
#define DMA_BLOCK_CONF					\
	DMAC_DCCSR_TM |					\
	DMAC_DCCSR_DS_8b | DMAC_DCCSR_RDIL_IGN |	\
	DMAC_DCCSR_ERDM_HLEVEL | DMAC_DCCSR_EACKS

/* single-mode  EOP: high  DREQ: high  DACK: low */
#define DMA_SINGLE_CONF					\
	DMAC_DCCSR_DS_8b | DMAC_DCCSR_RDIL_IGN |	\
	DMAC_DCCSR_ERDM_HLEVEL | DMAC_DCCSR_EACKS

#define DMA_8bit_RX_CONF				\
	DMAC_DCCSR_DAM |				\
	DMAC_DCCSR_SWDH_8 | DMAC_DCCSR_DWDH_32 |	\
	DMAC_DCCSR_DS_8b | DMAC_DCCSR_RDIL_IGN

#define DMA_8bit_TX_CONF				\
	DMAC_DCCSR_SAM |				\
	DMAC_DCCSR_SWDH_32 | DMAC_DCCSR_DWDH_8 |	\
	DMAC_DCCSR_DS_8b | DMAC_DCCSR_RDIL_IGN

#define DMA_16bit_RX_CONF				\
	DMAC_DCCSR_DAM |				\
	DMAC_DCCSR_SWDH_16 | DMAC_DCCSR_DWDH_32 |	\
	DMAC_DCCSR_DS_16b | DMAC_DCCSR_RDIL_IGN

#define DMA_16bit_TX_CONF				\
	DMAC_DCCSR_SAM |				\
	DMAC_DCCSR_SWDH_32 | DMAC_DCCSR_DWDH_16 |	\
	DMAC_DCCSR_DS_16b | DMAC_DCCSR_RDIL_IGN

#define DMA_32bit_RX_CONF				\
	DMAC_DCCSR_DAM |				\
	DMAC_DCCSR_SWDH_32 | DMAC_DCCSR_DWDH_32 |	\
	DMAC_DCCSR_DS_32b | DMAC_DCCSR_RDIL_IGN

#define DMA_32bit_TX_CONF				\
	DMAC_DCCSR_SAM |				\
	DMAC_DCCSR_SWDH_32 | DMAC_DCCSR_DWDH_32 |	\
	DMAC_DCCSR_DS_32b | DMAC_DCCSR_RDIL_IGN

#define DMA_16BYTE_RX_CONF				\
	DMAC_DCCSR_DAM |				\
	DMAC_DCCSR_SWDH_8 | DMAC_DCCSR_DWDH_32 |	\
	DMAC_DCCSR_DS_16B | DMAC_DCCSR_RDIL_IGN

#define DMA_16BYTE_TX_CONF				\
	DMAC_DCCSR_SAM |				\
	DMAC_DCCSR_SWDH_32 | DMAC_DCCSR_DWDH_8 |	\
	DMAC_DCCSR_DS_16B | DMAC_DCCSR_RDIL_IGN

#define DMA_AIC_32_16BYTE_TX_CMD			\
	DMAC_DCCSR_SAM |				\
	DMAC_DCCSR_SWDH_32 | DMAC_DCCSR_DWDH_32 |	\
	DMAC_DCCSR_DS_16B | DMAC_DCCSR_RDIL_IGN

#define DMA_AIC_32_16BYTE_RX_CMD			\
	DMAC_DCCSR_DAM |				\
	DMAC_DCCSR_SWDH_32 | DMAC_DCCSR_DWDH_32 |	\
	DMAC_DCCSR_DS_16B | DMAC_DCCSR_RDIL_IGN

#define DMA_AIC_16BIT_TX_CMD				\
	DMAC_DCCSR_SAM |				\
	DMAC_DCCSR_SWDH_16 | DMAC_DCCSR_DWDH_16 |	\
	DMAC_DCCSR_DS_16b | DMAC_DCCSR_RDIL_IGN

#define DMA_AIC_16BIT_RX_CMD				\
	DMAC_DCCSR_DAM |				\
	DMAC_DCCSR_SWDH_16 | DMAC_DCCSR_DWDH_16 |	\
	DMAC_DCCSR_DS_16b | DMAC_DCCSR_RDIL_IGN

#define DMA_AIC_16BYTE_RX_CMD				\
	DMAC_DCCSR_DAM |				\
	DMAC_DCCSR_SWDH_16 | DMAC_DCCSR_DWDH_16 |	\
	DMAC_DCCSR_DS_16B | DMAC_DCCSR_RDIL_IGN

#define DMA_AIC_16BYTE_TX_CMD				\
	DMAC_DCCSR_SAM |				\
	DMAC_DCCSR_SWDH_16 | DMAC_DCCSR_DWDH_16 |	\
	DMAC_DCCSR_DS_16B | DMAC_DCCSR_RDIL_IGN

/* DMA Device ID's follow */
enum {
	DMA_ID_UART0_TX = 0,
	DMA_ID_UART0_RX,
	DMA_ID_UART1_TX,
	DMA_ID_UART1_RX,
	DMA_ID_UART2_TX,
	DMA_ID_UART2_RX,
	DMA_ID_UART3_TX,
	DMA_ID_UART3_RX,
	DMA_ID_SSI_TX,
	DMA_ID_SSI_RX,
	DMA_ID_MSC_TX,
	DMA_ID_MSC_RX,
	DMA_ID_AIC_TX,
	DMA_ID_AIC_RX,
	DMA_ID_BLOCK,		/* DREQ */
	DMA_ID_SINGLE,		/* DREQ */
	DMA_ID_PCMCIA0_TX,
	DMA_ID_PCMCIA0_RX,
	DMA_ID_PCMCIA1_TX,
	DMA_ID_PCMCIA2_RX,
	DMA_ID_AUTO,
	DMA_ID_RAW_SET,
	NUM_DMA_DEV
};

/* dummy DCCSR bit, i386 style DMA macros compitable */
#define DMA_MODE_READ		0	/* I/O to memory, no autoinit,
					 * increment, single mode */
#define DMA_MODE_WRITE		1	/* memory to I/O, no autoinit,
					 * increment, single mode */
#define DMA_MODE_CASCADE	2	/* pass thru DREQ->HRQ,
					 * DACK<-HLDA only */
#define DMA_AUTOINIT		3
#define DMA_MODE_MASK		3

struct jz_dma_chan {
	int dev_id;		/* this channel is allocated if >=0,
				 * free otherwise */
	unsigned int io;
	const char *dev_str;
	int irq;
	void *irq_dev;
	unsigned int fifo_addr;
	unsigned int mode;
	unsigned int source;
};

extern struct jz_dma_chan jz_dma_table[];

extern int jz_request_dma(int dev_id,
			      const char *dev_str,
			      irqreturn_t (*irqhandler)(int, void *),
			      unsigned long irqflags,
			      void *irq_dev_id);
extern void jz_free_dma(unsigned int dmanr);

extern int jz_dma_read_proc(char *buf, char **start, off_t fpos,
			      int length, int *eof, void *data);
extern void dump_jz_dma_channel(unsigned int dmanr);

extern void enable_dma(unsigned int dmanr);
extern void disable_dma(unsigned int dmanr);
extern void set_dma_addr(unsigned int dmanr, unsigned int a);
extern void set_dma_count(unsigned int dmanr, unsigned int count);
extern void set_dma_mode(unsigned int dmanr, unsigned int mode);
extern void jz_set_oss_dma(unsigned int dmanr, unsigned int mode, unsigned int audio_fmt);
extern void jz_set_alsa_dma(unsigned int dmanr, unsigned int mode, unsigned int audio_fmt);
extern int get_dma_residue(unsigned int dmanr);

extern spinlock_t  dma_spin_lock;

static __inline__ unsigned long claim_dma_lock(void)
{
	unsigned long flags;
	spin_lock_irqsave(&dma_spin_lock, flags);
	return flags;
}

static __inline__ void release_dma_lock(unsigned long flags)
{
	spin_unlock_irqrestore(&dma_spin_lock, flags);
}

/* Clear the 'DMA Pointer Flip Flop'.
 * Write 0 for LSB/MSB, 1 for MSB/LSB access.
 */
#define clear_dma_ff(channel)

static __inline__ struct jz_dma_chan *get_dma_chan(unsigned int dmanr)
{
	if (dmanr > NUM_DMA
	    || jz_dma_table[dmanr].dev_id < 0)
		return NULL;
	return &jz_dma_table[dmanr];
}

static __inline__ int dma_halted(unsigned int dmanr)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	if (!chan)
		return 1;
	return  __dmac_channel_transmit_halt_detected(dmanr) ? 1 : 0;
}

static __inline__ unsigned int get_dma_mode(unsigned int dmanr)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	if (!chan)
		return 0;
	return chan->mode;
}

static __inline__ void clear_dma_done(unsigned int dmanr)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	if (!chan)
		return;
	REG_DMAC_DCCSR(chan->io) &= ~(DMAC_DCCSR_HLT | DMAC_DCCSR_TC | DMAC_DCCSR_AR);
}

static __inline__ void clear_dma_halt(unsigned int dmanr)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	if (!chan)
		return;
	REG_DMAC_DCCSR(chan->io) &= ~(DMAC_DCCSR_HLT);
	REG_DMAC_DMACR &= ~(DMAC_DMACR_HTR);
}
static __inline__ void clear_dma_flag(unsigned int dmanr)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	if (!chan)
		return;
	REG_DMAC_DCCSR(chan->io) &= ~(DMAC_DCCSR_HLT | DMAC_DCCSR_TC | DMAC_DCCSR_AR);
	REG_DMAC_DMACR &= ~(DMAC_DMACR_HTR | DMAC_DMACR_AER);
}

static __inline__ void set_dma_page(unsigned int dmanr, char pagenr)
{
}

static __inline__ unsigned int get_dma_done_status(unsigned int dmanr)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	unsigned long dccsr;
	if (!chan)
		return 0;

	dccsr = REG_DMAC_DCCSR(chan->io);
	return dccsr & (DMAC_DCCSR_HLT | DMAC_DCCSR_TC | DMAC_DCCSR_AR);
}

static __inline__ int get_dma_done_irq(unsigned int dmanr)
{
	struct jz_dma_chan *chan = get_dma_chan(dmanr);
	if (!chan)
		return -1;

	return chan->irq;
}

#endif  /* __ASM_JZ4730_DMA_H__ */
