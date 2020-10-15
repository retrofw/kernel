/*
 *  linux/drivers/mmc/jz_mmc.c - JZ SD/MMC driver
 *
 *  Copyright (C) 2005 - 2008 Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mm.h>
#include <linux/signal.h>
#include <linux/pm.h>
#include <linux/scatterlist.h>

#include <asm/io.h>
#include <asm/scatterlist.h>
#include <asm/jzsoc.h>

#include "jz_mmc.h"

#define DRIVER_NAME	"jz-mmc"

#define NR_SG	1

#if defined(CONFIG_SOC_JZ4725) || defined(CONFIG_SOC_JZ4720)
#undef USE_DMA
#else
#define USE_DMA 
#endif

struct jz_mmc_host {
	struct mmc_host *mmc;
	spinlock_t lock;
	struct {
		int len;
		int dir;
	} dma;
	struct {
		int index;
		int offset;
		int len;
	} pio;
	int irq;
	unsigned int clkrt;
	unsigned int cmdat;
	unsigned int imask;
	unsigned int power_mode;
	struct jz_mmc_platform_data *pdata;
	struct mmc_request *mrq;
	struct mmc_command *cmd;
	struct mmc_data *data;
	dma_addr_t sg_dma;
	struct jzsoc_dma_desc *sg_cpu;
	unsigned int dma_len;
	unsigned int dma_dir;
	struct pm_dev *pmdev;
};

static int r_type = 0;

#define MMC_IRQ_MASK()				\
do {						\
	REG_MSC_IMASK = 0xff;			\
	REG_MSC_IREG = 0xff;			\
} while (0)

static int rxdmachan = 0;
static int txdmachan = 0;
static int mmc_slot_enable = 0;

/* Stop the MMC clock and wait while it happens */
static inline int jz_mmc_stop_clock(void)
{
	int timeout = 1000;

	REG_MSC_STRPCL = MSC_STRPCL_CLOCK_CONTROL_STOP;
	while (timeout && (REG_MSC_STAT & MSC_STAT_CLK_EN)) {
		timeout--;
		if (timeout == 0)
			return 0;
		udelay(1);
	}
	return MMC_NO_ERROR;
}

/* Start the MMC clock and operation */
static inline int jz_mmc_start_clock(void)
{
	REG_MSC_STRPCL =
	    MSC_STRPCL_CLOCK_CONTROL_START | MSC_STRPCL_START_OP;
	return MMC_NO_ERROR;
}

static inline u32 jz_mmc_calc_clkrt(int is_sd, u32 rate)
{
	u32 clkrt;
	u32 clk_src = is_sd ? 24000000 : 20000000;

	clkrt = 0;
	while (rate < clk_src) {
		clkrt++;
		clk_src >>= 1;
	}
	return clkrt;
}

/* Select the MMC clock frequency */
static int jz_mmc_set_clock(u32 rate)
{
	int clkrt;

	jz_mmc_stop_clock();
	__cpm_select_msc_clk(1);	/* select clock source from CPM */
	clkrt = jz_mmc_calc_clkrt(1, rate);
	REG_MSC_CLKRT = clkrt;
	return MMC_NO_ERROR;
}

static void jz_mmc_enable_irq(struct jz_mmc_host *host, unsigned int mask)
{
	unsigned long flags;
	spin_lock_irqsave(&host->lock, flags);
	host->imask &= ~mask;
	REG_MSC_IMASK = host->imask;
	spin_unlock_irqrestore(&host->lock, flags);
}

static void jz_mmc_disable_irq(struct jz_mmc_host *host, unsigned int mask)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->imask |= mask;
	REG_MSC_IMASK = host->imask;
	spin_unlock_irqrestore(&host->lock, flags);
}

void jz_set_dma_block_size(int dmanr, int nbyte);

#ifdef USE_DMA
static inline void
jz_mmc_start_dma(int chan, unsigned long phyaddr, int count, int mode)
{
	unsigned long flags;

	flags = claim_dma_lock();
	disable_dma(chan);
	clear_dma_ff(chan);
	jz_set_dma_block_size(chan, 32);
	set_dma_mode(chan, mode);
	set_dma_addr(chan, phyaddr);
	set_dma_count(chan, count + 31);
	enable_dma(chan);
	release_dma_lock(flags);
}

static irqreturn_t jz_mmc_dma_rx_callback(int irq, void *devid)
{
	int chan = rxdmachan;

	disable_dma(chan);
	if (__dmac_channel_address_error_detected(chan)) {
		printk(KERN_DEBUG "%s: DMAC address error.\n",
		       __FUNCTION__);
		__dmac_channel_clear_address_error(chan);
	}
	if (__dmac_channel_transmit_end_detected(chan)) {
		__dmac_channel_clear_transmit_end(chan);
	}
	return IRQ_HANDLED;
}
static irqreturn_t jz_mmc_dma_tx_callback(int irq, void *devid)
{
	int chan = txdmachan;

	disable_dma(chan);
	if (__dmac_channel_address_error_detected(chan)) {
		printk(KERN_DEBUG "%s: DMAC address error.\n",
		       __FUNCTION__);
		__dmac_channel_clear_address_error(chan);
	}
	if (__dmac_channel_transmit_end_detected(chan)) {
		__dmac_channel_clear_transmit_end(chan);
	}
	return IRQ_HANDLED;
}

/* Prepare DMA to start data transfer from the MMC card */
static void jz_mmc_rx_setup_data(struct jz_mmc_host *host,
				 struct mmc_data *data)
{
	unsigned int nob = data->blocks;
	int channelrx = rxdmachan;
	int i;
	u32 size;

	if (data->flags & MMC_DATA_STREAM)
		nob = 0xffff;

	REG_MSC_NOB = nob;
	REG_MSC_BLKLEN = data->blksz;
	size = nob * data->blksz;

	if (data->flags & MMC_DATA_READ) {
		host->dma.dir = DMA_FROM_DEVICE;
	} else {
		host->dma.dir = DMA_TO_DEVICE;
	}

	host->dma.len =
	    dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
		       host->dma.dir);

	for (i = 0; i < host->dma.len; i++) {
		host->sg_cpu[i].dtadr = sg_dma_address(&data->sg[i]);
		host->sg_cpu[i].dcmd = sg_dma_len(&data->sg[i]);
		dma_cache_wback_inv((unsigned long)
				    CKSEG0ADDR(sg_dma_address(data->sg)) +
				    data->sg->offset,
				    host->sg_cpu[i].dcmd);
		jz_mmc_start_dma(channelrx, host->sg_cpu[i].dtadr,
				 host->sg_cpu[i].dcmd, DMA_MODE_READ);
	}
}

/* Prepare DMA to start data transfer from the MMC card */
static void jz_mmc_tx_setup_data(struct jz_mmc_host *host,
				 struct mmc_data *data)
{
	unsigned int nob = data->blocks;
	int channeltx = txdmachan;
	int i;
	u32 size;

	if (data->flags & MMC_DATA_STREAM)
		nob = 0xffff;

	REG_MSC_NOB = nob;
	REG_MSC_BLKLEN = data->blksz;
	size = nob * data->blksz;

	if (data->flags & MMC_DATA_READ) {
		host->dma.dir = DMA_FROM_DEVICE;
	} else {
		host->dma.dir = DMA_TO_DEVICE;
	}

	host->dma.len =
		dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
			   host->dma.dir);

	for (i = 0; i < host->dma.len; i++) {
		host->sg_cpu[i].dtadr = sg_dma_address(&data->sg[i]);
		host->sg_cpu[i].dcmd = sg_dma_len(&data->sg[i]);
		dma_cache_wback_inv((unsigned long)
				    CKSEG0ADDR(sg_dma_address(data->sg)) +
				    data->sg->offset,
				    host->sg_cpu[i].dcmd);
		jz_mmc_start_dma(channeltx, host->sg_cpu[i].dtadr,
				 host->sg_cpu[i].dcmd, DMA_MODE_WRITE);
	}
}
#else
static void jz_mmc_receive_pio(struct jz_mmc_host *host)
{

	struct mmc_data *data = 0;
	int sg_len = 0, max = 0, count = 0;
	u32 *buf = 0;
	struct scatterlist *sg;
	unsigned int nob;

	data = host->mrq->data;
	nob = data->blocks;
	REG_MSC_NOB = nob;
	REG_MSC_BLKLEN = data->blksz;

	max = host->pio.len;
	if (host->pio.index < host->dma.len) {
		sg = &data->sg[host->pio.index];
		buf = sg_virt(sg) + host->pio.offset;

		/* This is the space left inside the buffer */
		sg_len = sg_dma_len(&data->sg[host->pio.index]) - host->pio.offset;
		/* Check to if we need less then the size of the sg_buffer */
		if (sg_len < max) max = sg_len;
	}
	max = max / 4;
	for(count = 0; count < max; count++) {
		while (REG_MSC_STAT & MSC_STAT_DATA_FIFO_EMPTY)
			;
		*buf++ = REG_MSC_RXFIFO;
	} 
	host->pio.len -= count;
	host->pio.offset += count;

	if (sg_len && count == sg_len) {
		host->pio.index++;
		host->pio.offset = 0;
	}
}

static void jz_mmc_send_pio(struct jz_mmc_host *host)
{

	struct mmc_data *data = 0;
	int sg_len, max, count = 0;
	u32 *wbuf = 0;
	struct scatterlist *sg;
	unsigned int nob;

	data = host->mrq->data;
	nob = data->blocks;

	REG_MSC_NOB = nob;
	REG_MSC_BLKLEN = data->blksz;

	/* This is the pointer to the data buffer */
	sg = &data->sg[host->pio.index];
	wbuf = sg_virt(sg) + host->pio.offset;

	/* This is the space left inside the buffer */
	sg_len = data->sg[host->pio.index].length - host->pio.offset;

	/* Check to if we need less then the size of the sg_buffer */
	max = (sg_len > host->pio.len) ? host->pio.len : sg_len;
	max = max / 4;
	for(count = 0; count < max; count++ ) {
		while (REG_MSC_STAT & MSC_STAT_DATA_FIFO_FULL)
				;
		REG_MSC_TXFIFO = *wbuf++;
	}

	host->pio.len -= count;
	host->pio.offset += count;

	if (count == sg_len) {
		host->pio.index++;
		host->pio.offset = 0;
	}
}

static int
jz_mmc_prepare_data(struct jz_mmc_host *host, struct mmc_data *data)
{
	int datalen = data->blocks * data->blksz;

	host->dma.dir = DMA_BIDIRECTIONAL;
	host->dma.len = dma_map_sg(mmc_dev(host->mmc), data->sg,
				   data->sg_len, host->dma.dir);
	if (host->dma.len == 0)
		return -ETIMEDOUT;

	host->pio.index = 0;
	host->pio.offset = 0;
	host->pio.len = datalen;
	return 0;
}
#endif

static int jz_mmc_cmd_done(struct jz_mmc_host *host, unsigned int stat);

static void jz_mmc_finish_request(struct jz_mmc_host *host, struct mmc_request *mrq)
{
	jz_mmc_stop_clock();
	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;
	mmc_request_done(host->mmc, mrq);
}

static void jz_mmc_start_cmd(struct jz_mmc_host *host,
			     struct mmc_command *cmd, unsigned int cmdat)
{
	u32 timeout = 0x3fffff;
	unsigned int stat;
	struct jz_mmc_host *hst = host;
	WARN_ON(host->cmd != NULL);
	host->cmd = cmd;

	/* stop MMC clock */
	jz_mmc_stop_clock();

	/* mask interrupts */
	REG_MSC_IMASK = 0xff;

	/* clear status */
	REG_MSC_IREG = 0xff;

	if (cmd->flags & MMC_RSP_BUSY)
		cmdat |= MSC_CMDAT_BUSY;

#define RSP_TYPE(x)	((x) & ~(MMC_RSP_BUSY|MMC_RSP_OPCODE))
	switch (RSP_TYPE(mmc_resp_type(cmd))) {
	case RSP_TYPE(MMC_RSP_R1):	/* r1,r1b, r6, r7 */
		cmdat |= MSC_CMDAT_RESPONSE_R1;
		r_type = 1;
		break;
	case RSP_TYPE(MMC_RSP_R3):
		cmdat |= MSC_CMDAT_RESPONSE_R3;
		r_type = 1;
		break;
	case RSP_TYPE(MMC_RSP_R2):
		cmdat |= MSC_CMDAT_RESPONSE_R2;
		r_type = 2;
		break;
	default:
		break;
	}
	REG_MSC_CMD = cmd->opcode;

	/* Set argument */
#ifdef CONFIG_JZ_MMC_BUS_1
	if (cmd->opcode == 6) {
		/* set  1 bit sd card bus*/
		if (cmd->arg ==2)  
			REG_MSC_ARG = 0;

		/* set  1 bit mmc card bus*/
		if (cmd->arg == 0x3b70101)
			REG_MSC_ARG = 0x3b70001;
	} else
		REG_MSC_ARG = cmd->arg;
#else
	REG_MSC_ARG = cmd->arg;
#endif

	/* Set command */
	REG_MSC_CMDAT = cmdat;

	/* Send command */
	jz_mmc_start_clock();

	while (timeout-- && !(REG_MSC_STAT & MSC_STAT_END_CMD_RES))
		;

	REG_MSC_IREG = MSC_IREG_END_CMD_RES;	/* clear irq flag */
	if (cmd->opcode == 12) {
		while (timeout-- && !(REG_MSC_IREG & MSC_IREG_PRG_DONE))
			;
		REG_MSC_IREG = MSC_IREG_PRG_DONE;	/* clear status */
	}
	if (!mmc_slot_enable) {
		/* It seems that MSC can't report the MSC_STAT_TIME_OUT_RES when
		 * card was removed. We force to return here.
		 */
		cmd->error = -ETIMEDOUT;
		jz_mmc_finish_request(hst, hst->mrq);
		return;
	}

	if (SD_IO_SEND_OP_COND == cmd->opcode) {
		/* 
		 * Don't support SDIO card currently.
		 */
		cmd->error = -ETIMEDOUT;
		jz_mmc_finish_request(hst, hst->mrq);
		return;
	}

	/* Check for status */
	stat = REG_MSC_STAT;
	jz_mmc_cmd_done(hst, stat);
	if (host->data) {
		if (cmd->opcode == MMC_WRITE_BLOCK || cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK)
#ifdef USE_DMA
			jz_mmc_tx_setup_data(host, host->data);
#else
			jz_mmc_send_pio(host);
		else 
			jz_mmc_receive_pio(host);
#endif
	}
}

static int jz_mmc_cmd_done(struct jz_mmc_host *host, unsigned int stat)
{
	struct mmc_command *cmd = host->cmd;
	int i, temp[16];
	u8 *buf;
	u32 data, v, w1, w2;

	if (!cmd)
		return 0;

	host->cmd = NULL;
	buf = (u8 *) temp;
	switch (r_type) {
	case 1:
	{
		data = REG_MSC_RES;
		buf[0] = (data >> 8) & 0xff;
		buf[1] = data & 0xff;
		data = REG_MSC_RES;
		buf[2] = (data >> 8) & 0xff;
		buf[3] = data & 0xff;
		data = REG_MSC_RES;
		buf[4] = data & 0xff;
		cmd->resp[0] =
			buf[1] << 24 | buf[2] << 16 | buf[3] << 8 |
			buf[4];
		break;
	}
	case 2:
	{
		data = REG_MSC_RES;
		v = data & 0xffff;
		for (i = 0; i < 4; i++) {
			data = REG_MSC_RES;
			w1 = data & 0xffff;
			data = REG_MSC_RES;
			w2 = data & 0xffff;
			cmd->resp[i] = v << 24 | w1 << 8 | w2 >> 8;
			v = w2;
		}
		break;
	}
	case 0:
		break;
	}
	if (stat & MSC_STAT_TIME_OUT_RES) {
		printk("MSC_STAT_TIME_OUT_RES\n");
		cmd->error = -ETIMEDOUT;
	} else if (stat & MSC_STAT_CRC_RES_ERR && cmd->flags & MMC_RSP_CRC) {
		printk("MSC_STAT_CRC\n");
		if (cmd->opcode == MMC_ALL_SEND_CID ||
		    cmd->opcode == MMC_SEND_CSD ||
		    cmd->opcode == MMC_SEND_CID) {
			/* a bogus CRC error can appear if the msb of
			   the 15 byte response is a one */
			if ((cmd->resp[0] & 0x80000000) == 0)
				cmd->error = -EILSEQ;
		}
	}
	/*
	 * Did I mention this is Sick.  We always need to
	 * discard the upper 8 bits of the first 16-bit word.
	 */
	if (host->data && cmd->error == 0)
		jz_mmc_enable_irq(host, MSC_IMASK_DATA_TRAN_DONE);
	else
		jz_mmc_finish_request(host, host->mrq);

	return 1;
}

static int jz_mmc_data_done(struct jz_mmc_host *host, unsigned int stat)
{
	struct mmc_data *data = host->data;

	if (!data)
		return 0;
	REG_MSC_IREG = MSC_IREG_DATA_TRAN_DONE;	/* clear status */
	jz_mmc_stop_clock();
	dma_unmap_sg(mmc_dev(host->mmc), data->sg, host->dma_len,
		     host->dma_dir);
	if (stat & MSC_STAT_TIME_OUT_READ) {
		printk("MMC/SD timeout, MMC_STAT 0x%x\n", stat);
		data->error = -ETIMEDOUT;
	} else if (REG_MSC_STAT &
		   (MSC_STAT_CRC_READ_ERROR | MSC_STAT_CRC_WRITE_ERROR)) {
		printk("MMC/SD CRC error, MMC_STAT 0x%x\n", stat);
		data->error = -EILSEQ;
	}
	/*
	 * There appears to be a hardware design bug here.  There seems to
	 * be no way to find out how much data was transferred to the card.
	 * This means that if there was an error on any block, we mark all
	 * data blocks as being in error.
	 */
	if (data->error == 0)
		data->bytes_xfered = data->blocks * data->blksz;
	else
		data->bytes_xfered = 0;

	jz_mmc_disable_irq(host, MSC_IMASK_DATA_TRAN_DONE);
	host->data = NULL;
	if (host->mrq->stop) {
		jz_mmc_stop_clock();
		jz_mmc_start_cmd(host, host->mrq->stop, 0);
	} else {
		jz_mmc_finish_request(host, host->mrq);
	}
	return 1;
}

static void jz_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct jz_mmc_host *host = mmc_priv(mmc);
	unsigned int cmdat;

	/* stop MMC clock */
	jz_mmc_stop_clock();

	/* Save current request for the future processing */
	host->mrq = mrq;
	host->data = mrq->data;
	cmdat = host->cmdat;
	host->cmdat &= ~MSC_CMDAT_INIT;

	if (mrq->data) {
		cmdat &= ~MSC_CMDAT_BUSY;
#ifdef USE_DMA
		if ((mrq->cmd->opcode == 51) | (mrq->cmd->opcode == 8) | (mrq->cmd->opcode == 6))

			cmdat |=
				MSC_CMDAT_BUS_WIDTH_1BIT | MSC_CMDAT_DATA_EN |
				MSC_CMDAT_DMA_EN;
		else {
#ifdef CONFIG_JZ_MMC_BUS_1
			cmdat &= ~MSC_CMDAT_BUS_WIDTH_4BIT;	
			cmdat |= MSC_CMDAT_BUS_WIDTH_1BIT | MSC_CMDAT_DATA_EN |
				MSC_CMDAT_DMA_EN;
#else
			cmdat |= MSC_CMDAT_DATA_EN | MSC_CMDAT_DMA_EN;
#endif
		}
		if (mrq->data->flags & MMC_DATA_WRITE)
			cmdat |= MSC_CMDAT_WRITE;

		if (mrq->data->flags & MMC_DATA_STREAM)
			cmdat |= MSC_CMDAT_STREAM_BLOCK;
		if (mrq->cmd->opcode != MMC_WRITE_BLOCK
		    && mrq->cmd->opcode != MMC_WRITE_MULTIPLE_BLOCK)
			jz_mmc_rx_setup_data(host, mrq->data);
#else /*USE_DMA*/

		if ((mrq->cmd->opcode == 51) | (mrq->cmd->opcode == 8) | (mrq->cmd->opcode == 6))
			cmdat |= MSC_CMDAT_BUS_WIDTH_1BIT | MSC_CMDAT_DATA_EN;
		else {
#ifdef CONFIG_JZ_MMC_BUS_1
			cmdat &= ~MSC_CMDAT_BUS_WIDTH_4BIT;	
			cmdat |= MSC_CMDAT_BUS_WIDTH_1BIT | MSC_CMDAT_DATA_EN;
#else
			cmdat |= MSC_CMDAT_DATA_EN;
#endif
		}
		if (mrq->data->flags & MMC_DATA_WRITE)
			cmdat |= MSC_CMDAT_WRITE;

		if (mrq->data->flags & MMC_DATA_STREAM)
			cmdat |= MSC_CMDAT_STREAM_BLOCK;
		jz_mmc_prepare_data(host, host->data);
#endif /*USE_DMA*/
	}
	jz_mmc_start_cmd(host, mrq->cmd, cmdat);
}

static irqreturn_t jz_mmc_irq(int irq, void *devid)
{
	struct jz_mmc_host *host = devid;
	unsigned int ireg;
	int handled = 0;

	ireg = REG_MSC_IREG;

	if (ireg) {
		unsigned stat = REG_MSC_STAT;
		if (ireg & MSC_IREG_DATA_TRAN_DONE)
			handled |= jz_mmc_data_done(host, stat);
	}
	return IRQ_RETVAL(handled);
}

/* Returns true if MMC slot is empty */
static int jz_mmc_slot_is_empty(int slot)
{
	int empty;

	empty = (__msc_card_detected(slot) == 0) ? 1 : 0;

	if (empty) {
		/* wait for card insertion */
#ifdef CONFIG_MIPS_JZ4740_LYRA
		__gpio_as_irq_rise_edge(MSC_HOTPLUG_PIN);
#else
		__gpio_as_irq_fall_edge(MSC_HOTPLUG_PIN);
#endif
	} else {
		/* wait for card removal */
#ifdef CONFIG_MIPS_JZ4740_LYRA
		__gpio_as_irq_fall_edge(MSC_HOTPLUG_PIN);
#else
		__gpio_as_irq_rise_edge(MSC_HOTPLUG_PIN);
#endif
	}

	return empty;
}

static irqreturn_t jz_mmc_detect_irq(int irq, void *devid)
{
	struct jz_mmc_host *host = (struct jz_mmc_host *) devid;

	if (jz_mmc_slot_is_empty(0)) {
		mmc_slot_enable = 0;
		mmc_detect_change(host->mmc, 50);
	} else {
		mmc_slot_enable = 1;
		mmc_detect_change(host->mmc, 50);
	}
	return IRQ_HANDLED;
}

static int jz_mmc_get_ro(struct mmc_host *mmc)
{
	struct jz_mmc_host *host = mmc_priv(mmc);

	if (host->pdata && host->pdata->get_ro)
		return host->pdata->get_ro(mmc_dev(mmc));
	/* Host doesn't support read only detection so assume writeable */
	return 0;
}

/* set clock and power */
static void jz_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct jz_mmc_host *host = mmc_priv(mmc);

	if (ios->clock)
		jz_mmc_set_clock(ios->clock);
	else
		jz_mmc_stop_clock();

	if (host->power_mode != ios->power_mode) {
		host->power_mode = ios->power_mode;

		if (ios->power_mode == MMC_POWER_ON)
			host->cmdat |= CMDAT_INIT;
	}

	if ((ios->bus_width == MMC_BUS_WIDTH_4) || (ios->bus_width == MMC_BUS_WIDTH_8)) 
		host->cmdat |= MSC_CMDAT_BUS_WIDTH_4BIT;
	else 
		host->cmdat &= ~MSC_CMDAT_BUS_WIDTH_4BIT;
}

static const struct mmc_host_ops jz_mmc_ops = {
	.request = jz_mmc_request,
	.get_ro = jz_mmc_get_ro,
	.set_ios = jz_mmc_set_ios,
};

static int jz_mmc_probe(struct platform_device *pdev)
{
	int retval;
	struct mmc_host *mmc;
	struct jz_mmc_host *host = NULL;
	int irq;
	struct resource *r;

	__gpio_as_msc();
	__msc_init_io();
	__msc_enable_power();

	__msc_reset();

	/* On reset, stop MMC clock */
	jz_mmc_stop_clock();

	MMC_IRQ_MASK();

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!r || irq < 0)
		return -ENXIO;

	r = request_mem_region(r->start, SZ_4K, DRIVER_NAME);
	if (!r)
		return -EBUSY;

	mmc = mmc_alloc_host(sizeof(struct jz_mmc_host), &pdev->dev);
	if (!mmc) {
		retval = -ENOMEM;
		goto out;
	}
	mmc->ops = &jz_mmc_ops;
	mmc->f_min = MMC_CLOCK_SLOW;
	mmc->f_max = SD_CLOCK_FAST;
	/*
	 * We can do SG-DMA, but we don't because we never know how much
	 * data we successfully wrote to the card.
	 */
	mmc->max_phys_segs = NR_SG;
	/*
	 * Our hardware DMA can handle a maximum of one page per SG entry.
	 */
	mmc->max_seg_size = PAGE_SIZE;
	/*
	 * Block length register is 10 bits.
	 */
	mmc->max_blk_size = 1023;
	/*
	 * Block count register is 16 bits.
	 */
	mmc->max_blk_count = 65535;
	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->pdata = pdev->dev.platform_data;
	mmc->ocr_avail = host->pdata ?
		host->pdata->ocr_mask : MMC_VDD_32_33 | MMC_VDD_33_34;
	host->mmc->caps =
		MMC_CAP_4_BIT_DATA | MMC_CAP_SD_HIGHSPEED
		| MMC_CAP_MMC_HIGHSPEED;
	/*
	 *MMC_CAP_4_BIT_DATA    (1 << 0)    The host can do 4 bit transfers  
	 *
	 */
	host->sg_cpu =
		dma_alloc_coherent(&pdev->dev, PAGE_SIZE, &host->sg_dma,
				   GFP_KERNEL);
	if (!host->sg_cpu) {
		retval = -ENOMEM;
		goto out;
	}
	spin_lock_init(&host->lock);
	host->irq = IRQ_MSC;
	host->imask = 0xff;
	/*
	 * Ensure that the host controller is shut down, and setup
	 * with our defaults.
	 */
	retval = request_irq(IRQ_MSC, jz_mmc_irq, 0, "MMC/SD", host);
	if (retval) {
		printk(KERN_ERR "MMC/SD: can't request MMC/SD IRQ\n");
		return retval;
	}
	jz_mmc_slot_is_empty(0);
	/* Request card detect interrupt */

	retval = request_irq(MSC_HOTPLUG_IRQ, jz_mmc_detect_irq, 0,	//SA_INTERRUPT,
			     "MMC card detect", host);
	if (retval) {
		printk(KERN_ERR "MMC/SD: can't request card detect IRQ\n");
		goto err1;
	}
#ifdef USE_DMA
	/* Request MMC Rx DMA channel */
	rxdmachan =
		jz_request_dma(DMA_ID_MSC_RX, "MMC Rx", jz_mmc_dma_rx_callback,
			       0, host);
	if (rxdmachan < 0) {
		printk(KERN_ERR "jz_request_dma failed for MMC Rx\n");
		goto err2;
	}

	/* Request MMC Tx DMA channel */
	txdmachan =
		jz_request_dma(DMA_ID_MSC_TX, "MMC Tx", jz_mmc_dma_tx_callback,
			       0, host);
	if (txdmachan < 0) {
		printk(KERN_ERR "jz_request_dma failed for MMC Tx\n");
		goto err3;
	}
#endif
	platform_set_drvdata(pdev, mmc);
	mmc_add_host(mmc);

	printk(JZ_SOC_NAME ": SD/MMC card driver registered.\n");

	/* Detect card during initialization */
#ifdef CONFIG_SOC_JZ4740
	if (!jz_mmc_slot_is_empty(0)) {
		mmc_slot_enable = 1;
		mmc_detect_change(host->mmc, 0);
	}
#endif
	return 0;

err1:free_irq(IRQ_MSC, &host);
#ifdef USE_DMA
 err2:jz_free_dma(rxdmachan);
 err3:jz_free_dma(txdmachan);
#endif
out:
	if (host) {
		if (host->sg_cpu)
			dma_free_coherent(&pdev->dev, PAGE_SIZE,
					  host->sg_cpu, host->sg_dma);
	}
	if (mmc)
		mmc_free_host(mmc);
	return -1;
}

static int jz_mmc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	unsigned long flags;

	platform_set_drvdata(pdev, NULL);

	if (mmc) {
		struct jz_mmc_host *host = mmc_priv(mmc);

		if (host->pdata && host->pdata->exit)
			host->pdata->exit(&pdev->dev, mmc);

		mmc_remove_host(mmc);

		local_irq_save(flags);
		jz_mmc_stop_clock();
		__msc_disable_power();
		jz_free_dma(rxdmachan);
		jz_free_dma(txdmachan);
		free_irq(IRQ_MSC, host);
		local_irq_restore(flags);
		mmc_free_host(mmc);
	}
	return 0;
}

#ifdef CONFIG_PM
pm_message_t state;
static int jz_mmc_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	__msc_disable_power();
	if (mmc)
		ret = mmc_suspend_host(mmc, state);

	return ret;
}

static int jz_mmc_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;
#if 0
	/*for sandisk BB0807011816D and other strange cards*/
	int i;

	for(i = 104; i < 110; i++) 
		__gpio_as_input(i); 

	/* perhaps you should mdelay more */
	mdelay(1000);
	__gpio_as_msc();
#endif	
	__msc_init_io();
	__msc_enable_power();
	__msc_reset();

	if (!jz_mmc_slot_is_empty(0)) {
		mmc_slot_enable = 1;
		mmc_detect_change(mmc, 10);
	}

	if (mmc)
		ret = mmc_resume_host(mmc);

	return ret;
}
#else
#define jz_mmc_suspend	NULL
#define jz_mmc_resume	NULL
#endif

static struct platform_driver jz_mmc_driver = {
	.probe = jz_mmc_probe,
	.remove = jz_mmc_remove,
	.suspend = jz_mmc_suspend,
	.resume = jz_mmc_resume,
	.driver = {
		   .name = DRIVER_NAME,
		   },
};

static int __init jz_mmc_init(void)
{
	return platform_driver_register(&jz_mmc_driver);
}

static void __exit jz_mmc_exit(void)
{
	platform_driver_unregister(&jz_mmc_driver);
}

module_init(jz_mmc_init);
module_exit(jz_mmc_exit);

MODULE_DESCRIPTION("JZ47XX SD/Multimedia Card Interface Driver");
MODULE_LICENSE("GPL");
