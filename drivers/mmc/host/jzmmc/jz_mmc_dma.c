/*
 *  linux/drivers/mmc/host/jz_mmc/dma/jz_mmc_dma.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */


#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <asm/jzsoc.h>
#include "include/jz_mmc_dma.h"
#include "include/jz_mmc_host.h"

#define JZMMC_BUFFER_NEEDS_BOUNCE(buffer)  (((unsigned long)(buffer) & 0x3) || !virt_addr_valid((buffer)))

void jz_mmc_stop_dma(struct jz_mmc_host *host)
{
	u32 old_counter = REG_DMAC_DTCR(host->dma.channel);
	u32 cur_counter;

	//WARN(1, "mmc%d called %s\n", host->pdev_id, __FUNCTION__);
	/* wait for the counter not change */
	while (1) {		     /* wait forever, even when the card is removed */
		schedule_timeout(3); /* 30ms */
		cur_counter = REG_DMAC_DTCR(host->dma.channel);
		if (cur_counter == old_counter)
			break;
		old_counter = cur_counter;
	}

	// Stop all
	REG_DMAC_DCCSR(host->dma.channel) = 0;

	// Clear all
	REG_DMAC_DCMD(host->dma.channel) = 0;
	REG_DMAC_DSAR(host->dma.channel) = 0;
	REG_DMAC_DTAR(host->dma.channel) = 0;
	REG_DMAC_DTCR(host->dma.channel) = 0;
	REG_DMAC_DRSR(host->dma.channel) = 0;
	REG_DMAC_DDA(host->dma.channel) = 0;
}

#ifndef DMAC_DRSR_RS_MSC2OUT
#define DMAC_DRSR_RS_MSC2OUT 0
#endif

#ifndef DMAC_DRSR_RS_MSC2IN
#define DMAC_DRSR_RS_MSC2IN 0
#endif

#define MSC_SET_OUT_REQ_SRC(ctrler_id, dst)				\
	do {								\
		switch((ctrler_id)) {					\
		case 0:							\
			(dst) = DMAC_DRSR_RS_MSC0OUT;			\
			break;						\
		case 1:							\
			(dst) = DMAC_DRSR_RS_MSC1OUT;			\
			break;						\
		case 2:							\
			(dst) = DMAC_DRSR_RS_MSC2OUT;			\
			break;						\
		default:						\
			BUG();						\
		}							\
	} while(0)


#define MSC_SET_IN_REQ_SRC(ctrler_id, dst)				\
	do {								\
		switch((ctrler_id)) {					\
		case 0:							\
			(dst) = DMAC_DRSR_RS_MSC0IN;			\
			break;						\
		case 1:							\
			(dst) = DMAC_DRSR_RS_MSC1IN;			\
			break;						\
		case 2:							\
			(dst) = DMAC_DRSR_RS_MSC2IN;			\
			break;						\
		default:						\
			BUG();						\
		}							\
	} while (0)

#ifdef USE_DMA_DESC
static int sg_to_desc(struct scatterlist *sgentry, JZ_MSC_DMA_DESC *first_desc,
		      int *desc_pos /* IN OUT */, int mode, int ctrl_id,
		      struct jz_mmc_host *host) {
	JZ_MSC_DMA_DESC *desc = NULL;
	int pos = *desc_pos;
	unsigned int next;
	unsigned int dma_len;

	dma_addr_t max_burst_dma_addr = 0;
	unsigned int max_burst_len = 0;

	dma_addr_t burst4_dma_addr = 0;
	unsigned int burst4_len = 0;

	dma_addr_t dma_desc_phys_addr = CPHYSADDR((unsigned long)first_desc);

	/* if the start address is not aligned to word, fall back to PIO mode */
	if (sg_phys(sgentry) & 0x3) {
		//printk("address not aligned. addr = 0x%08x\n", sg_virt(sgentry));
		return -1;
	}

	dma_len = sg_dma_len(sgentry);

	/* if the length is not multiple of 4, fall back to PIO mode */
	if (dma_len % 4) {
		//printk("length is not multiple of 4., len = %d\n", dma_len);
		return -1;
	}

	max_burst_dma_addr = sg_phys(sgentry);
	max_burst_len = dma_len & (~63);

	burst4_dma_addr = max_burst_dma_addr + max_burst_len;
	burst4_len = dma_len & 63;

	if (max_burst_len) {
		desc = first_desc + pos;
		next = (dma_desc_phys_addr + (pos + 1) * (sizeof(JZ_MSC_DMA_DESC))) >> 4;

		desc->dcmd = DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_LINK;

#ifdef USE_DMA_BUSRT_64
		desc->dcmd |= DMAC_DCMD_DS_64BYTE;
#else
		desc->dcmd |= DMAC_DCMD_DS_32BYTE;
#endif

		if (DMA_MODE_WRITE == mode) {
			desc->dcmd |= DMAC_DCMD_SAI;
			desc->dsadr = (unsigned int)max_burst_dma_addr;
			desc->dtadr = CPHYSADDR(MSC_TXFIFO(ctrl_id));
			MSC_SET_OUT_REQ_SRC(ctrl_id, desc->dreqt);
		} else {
			desc->dcmd |= DMAC_DCMD_DAI;
			desc->dsadr = CPHYSADDR(MSC_RXFIFO(ctrl_id));
			desc->dtadr = (unsigned int)max_burst_dma_addr;
			MSC_SET_IN_REQ_SRC(ctrl_id, desc->dreqt);
		}

#ifdef USE_DMA_BUSRT_64
		desc->ddadr = (next << 24) | (max_burst_len / 64) ;
#else
		desc->ddadr = (next << 24) | (max_burst_len / 32) ;
#endif

		pos ++;
	}

	if (burst4_len) {
		desc = first_desc + pos;
		next = (dma_desc_phys_addr + (pos + 1) * (sizeof(JZ_MSC_DMA_DESC))) >> 4;
		desc->dcmd = DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BIT | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_LINK;

		if (DMA_MODE_WRITE == mode) {
			desc->dcmd |= DMAC_DCMD_SAI;
			desc->dsadr = (unsigned int)burst4_dma_addr;
			desc->dtadr = CPHYSADDR(MSC_TXFIFO(ctrl_id));
			MSC_SET_OUT_REQ_SRC(ctrl_id, desc->dreqt);
		} else {
			desc->dcmd |= DMAC_DCMD_DAI;
			desc->dsadr = CPHYSADDR(MSC_RXFIFO(ctrl_id));
			desc->dtadr = (unsigned int)burst4_dma_addr;
			MSC_SET_IN_REQ_SRC(ctrl_id, desc->dreqt);
		}
		desc->ddadr = (next << 24) | (burst4_len >> 2) ;

		pos ++;
	}

	*desc_pos = pos;

	return 0;
}

#ifdef MSC_DEBUG_DMA
#define JZ_MSC_RECORD_DESC_NUM(num)	host->num_desc = (num)
#else
#define JZ_MSC_RECORD_DESC_NUM(num)	do {  } while(0)
#endif

int jz_mmc_start_scatter_dma(int chan, struct jz_mmc_host *host,
			      struct scatterlist *sg, unsigned int sg_len, int mode) {
	int i = 0;
	int desc_pos = 0;
	dma_addr_t dma_desc_phy_addr = 0;
	struct mmc_data *data = host->curr_mrq->data;
	struct scatterlist *sgentry;
	JZ_MSC_DMA_DESC *desc;
	JZ_MSC_DMA_DESC *desc_first;
	unsigned long flags;
	unsigned long start_time = jiffies;
	int ret = 0;

	while (REG_DMAC_DMACR(chan / HALF_DMA_NUM) & (DMAC_DMACR_HLT | DMAC_DMACR_AR)) {
		if (jiffies - start_time > 10) { /* 100ms */
			printk("DMAC unavailable! REG_DMAC_DMACR(%d) = 0x%08x\n", chan / HALF_DMA_NUM, REG_DMAC_DMACR(chan / HALF_DMA_NUM));
			jz_mmc_stop_dma(host);
			break;
		}
	}

	start_time = jiffies;
	while (REG_DMAC_DCCSR(chan) & (DMAC_DCCSR_HLT | DMAC_DCCSR_TT | DMAC_DCCSR_AR)) {
		if (jiffies - start_time > 10) { /* 100ms */
			printk("DMA channel %d unavailable! REG_DMAC_DCCSR(%d) = 0x%08x\n", chan, chan, REG_DMAC_DCCSR(chan));
			jz_mmc_stop_dma(host);
			break;
		}
	}

	REG_DMAC_DCCSR(chan) |= DMAC_DCCSR_DES8;
	REG_DMAC_DCCSR(chan) &= ~DMAC_DCCSR_NDES;

	/* Setup request source */
	if (DMA_MODE_WRITE == mode) {
		MSC_SET_OUT_REQ_SRC(host->pdev_id, REG_DMAC_DRSR(chan));
	} else {
		MSC_SET_IN_REQ_SRC(host->pdev_id, REG_DMAC_DRSR(chan));
	}

#ifdef MSC_DEBUG_DMA
	if (DMA_MODE_WRITE == mode) {
		host->last_direction = 1;
	} else {
		host->last_direction = 0;
	}
#endif

	desc = host->dma_desc;
	JZ_MSC_RECORD_DESC_NUM(desc_pos);
	desc_first = desc;

	dma_desc_phy_addr  = CPHYSADDR((unsigned long)desc);

	memset(desc, 0, PAGE_SIZE);

	desc_pos = 0;
	flags = claim_dma_lock();
	for_each_sg(data->sg, sgentry, host->dma.len, i) {
		ret = sg_to_desc(sgentry, desc, &desc_pos, mode, host->pdev_id, host);
		if (ret < 0)
			goto out;
	}

	desc = desc + (desc_pos - 1);
	desc->dcmd |= DMAC_DCMD_TIE;
	desc->dcmd &= ~DMAC_DCMD_LINK;
	desc->ddadr &= ~0xff000000;

	dma_cache_wback_inv((unsigned long)desc_first, desc_pos * sizeof(JZ_MSC_DMA_DESC));

        /* Setup DMA descriptor address */
	REG_DMAC_DDA(chan) = dma_desc_phy_addr;

	/* DMA doorbell set -- start DMA now ... */
	REG_DMAC_DMADBSR(chan / HALF_DMA_NUM) = 1 << (chan - (chan / HALF_DMA_NUM) * HALF_DMA_NUM) ;

	/* Enable DMA */
	REG_DMAC_DMACR(chan / HALF_DMA_NUM) |= DMAC_DMACR_DMAE;

	REG_DMAC_DCCSR(chan) |= DMAC_DCCSR_EN;

 out:
	release_dma_lock(flags);
	return ret;

}
#else
void jz_mmc_start_normal_dma(struct jz_mmc_host *host, unsigned long phyaddr, int count, int mode, int ds)
{
	unsigned long flags;
	unsigned long start_time = jiffies;
	int chan = host->dma.channel;
	u32 dma_cmd = 0;
	u32 src_addr = 0;
	u32 dst_addr = 0;
	u32 req_src = 0;

	while (REG_DMAC_DMACR(chan / HALF_DMA_NUM) & (DMAC_DMACR_HLT | DMAC_DMACR_AR)) {
		if (jiffies - start_time > 10) { /* 100ms */
			printk("DMAC unavailable! REG_DMAC_DMACR(%d) = 0x%08x\n", chan / HALF_DMA_NUM, REG_DMAC_DMACR(chan / HALF_DMA_NUM));
			jz_mmc_stop_dma(host);
			break;
		}
	}

	start_time = jiffies;
	while (REG_DMAC_DCCSR(chan) & (DMAC_DCCSR_HLT | DMAC_DCCSR_TT | DMAC_DCCSR_AR)) {
		if (jiffies - start_time > 10) { /* 100ms */
			printk("DMA channel %d unavailable! REG_DMAC_DCCSR(%d) = 0x%08x\n", chan, chan, REG_DMAC_DCCSR(chan));
			jz_mmc_stop_dma(host);
			break;
		}
	}

	flags = claim_dma_lock();
	dma_cmd = DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_TIE;
	switch (ds) {
#ifdef USE_DMA_BUSRT_64
	case 64:
		dma_cmd |= DMAC_DCMD_DS_64BYTE;
		break;
#endif

	case 32:
		dma_cmd |= DMAC_DCMD_DS_32BYTE;
		break;

	case 16:
		dma_cmd |= DMAC_DCMD_DS_16BYTE;
		break;

	case 4:
		dma_cmd |= DMAC_DCMD_DS_32BIT;
		break;

	default:
		;
	}
	if (DMA_MODE_WRITE == mode) {
		dma_cmd |= DMAC_DCMD_SAI;
		src_addr = (unsigned int)phyaddr;      /* DMA source address */
		dst_addr = CPHYSADDR(MSC_TXFIFO(host->pdev_id));      /* DMA target address */
		MSC_SET_OUT_REQ_SRC(host->pdev_id, req_src);
	} else {
		dma_cmd |= DMAC_DCMD_DAI;
		src_addr = CPHYSADDR(MSC_RXFIFO(host->pdev_id));
		dst_addr = (unsigned int)phyaddr;
		MSC_SET_IN_REQ_SRC(host->pdev_id, req_src);
	}

	REG_DMAC_DCCSR(chan) |= DMAC_DCCSR_NDES; /* No-descriptor transfer */
	REG_DMAC_DSAR(chan) = src_addr;
	REG_DMAC_DTAR(chan) = dst_addr;
	REG_DMAC_DTCR(chan) = (count + ds - 1) / ds;
	REG_DMAC_DCMD(chan) = dma_cmd;
	REG_DMAC_DRSR(chan) = req_src;

	REG_DMAC_DMACR(chan / HALF_DMA_NUM) |= DMAC_DMACR_DMAE;
	REG_DMAC_DCCSR(chan) |= DMAC_DCCSR_EN;

	release_dma_lock(flags);
}
#endif

static void jz_mmc_highmem_dma_map_sg(struct scatterlist *sgl, unsigned int nents, int is_write)
{
	struct sg_mapping_iter miter;
	unsigned long flags;
	unsigned int sg_flags = SG_MITER_ATOMIC;

	if (is_write)
		sg_flags |= SG_MITER_FROM_SG;
	else
		sg_flags |= SG_MITER_TO_SG;

	sg_miter_start(&miter, sgl, nents, sg_flags);

	local_irq_save(flags);

	while (sg_miter_next(&miter)) {
		if (is_write)
			dma_cache_wback_inv((unsigned long)miter.addr, miter.length);
		else
			dma_cache_inv((unsigned long)miter.addr, miter.length);
	}

	sg_miter_stop(&miter);
	local_irq_restore(flags);
}

int jz_mmc_start_dma(struct jz_mmc_host *host) {
	struct mmc_data *data = host->curr_mrq->data;
	int mode;
#ifndef USE_DMA_DESC
	int i;
	int ds = 4;
	struct scatterlist *sgentry;
#endif
	int ret = 0;

	host->transfer_mode = JZ_TRANS_MODE_DMA;

	if (data->flags & MMC_DATA_WRITE) {
		mode = DMA_MODE_WRITE;
		host->dma.dir = DMA_TO_DEVICE;
	} else {
		mode = DMA_MODE_READ;
		host->dma.dir = DMA_FROM_DEVICE;
	}

#ifndef CONFIG_HIGHMEM
	host->dma.len =
	    dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
		       host->dma.dir);
#else
	jz_mmc_highmem_dma_map_sg(data->sg, data->sg_len, (mode == DMA_MODE_WRITE));
	host->dma.len = data->sg_len;
#endif

#ifdef USE_DMA_DESC
	ret = jz_mmc_start_scatter_dma(host->dma.channel, host, data->sg, host->dma.len, mode);
#else
	for_each_sg(data->sg, sgentry, host->dma.len, i) {
		dma_cache_wback_inv((unsigned long)CKSEG0ADDR(sg_phys(sgentry) + data->sg->offset),
				    sg_dma_len(sgentry));

		if (unlikely((sg_dma_len(sgentry) & 0x3))) {
			ret = -1;
			break;
		}

		if (unlikely(sg_phys(sgentry) & 0x3)) {
			ret = -1;
			break;
		}

		if ((likely(sg_dma_len(sgentry) % 32 == 0)))
			ds = 32; /* 32 byte */
		else if (sg_dma_len(sgentry) % 16 == 0)
			ds = 16; /* 16 byte */
		else
			ds = 4; /* default to 4 byte */

		/*
		 * FIXME: bug here!!!!! if NR_SG > 1(current NR_SG==1),
		 * must wait for current dma done, then next sg
		 */
		jz_mmc_start_normal_dma(host, sg_phys(sgentry),
				 sg_dma_len(sgentry), mode, ds);
	}
#endif

	if (ret < 0) {
		dma_unmap_sg(mmc_dev(host->mmc), data->sg, host->dma.len,
			     host->dma.dir);
		host->transfer_mode = JZ_TRANS_MODE_NULL;
	}

	return ret;
}

static irqreturn_t jz_mmc_dma_callback(int irq, void *devid)
{
	struct jz_mmc_host *host = devid;
	int chan = host->dma.channel;

	disable_dma(chan);

	host->data_err = 0;
	if (__dmac_channel_address_error_detected(chan)) {
		printk("%s: DMAC address error.\n",
		       __FUNCTION__);
		__dmac_channel_clear_address_error(chan);
		host->data_err = 1;
		wmb();
	}

	if (__dmac_channel_transmit_halt_detected(chan)) {
		printk("%s: DMA chan%d Halted.\n", __func__, chan);
		__dmac_channel_clear_transmit_halt(chan);
		host->data_err = 1;
		wmb();
	}
	if (__dmac_channel_transmit_end_detected(chan)) {
		__dmac_channel_clear_transmit_end(chan);
	}

	if (host->dma.dir == DMA_FROM_DEVICE) {
		host->data_ack = 1;
		wmb();
		wake_up_interruptible(&host->data_wait_queue);
	}

	return IRQ_HANDLED;
}

static char *msc_dma_name[] = {
	"msc0_dma",
	"msc1_dma",
	"msc2_dma",
};

static int jz_mmc_init_dma(struct jz_mmc_host *host)
{
	if (host->dma_id < 0)
		return 0;     /* not use dma */

	host->dma.channel = jz_request_dma(host->dma_id,
					   msc_dma_name[host->pdev_id],
					   jz_mmc_dma_callback,
					   0, host);
	if (host->dma.channel < 0) {
		printk(KERN_ERR "jz_request_dma failed for MMC Rx\n");
		goto err_out;
	}

	REG_DMAC_DMACR(host->dma.channel / HALF_DMA_NUM) |= DMAC_DMACR_FMSC;

#ifdef USE_DMA_DESC
	host->dma_desc = (JZ_MSC_DMA_DESC *)__get_free_pages(GFP_KERNEL, 0);
#endif

	return 0;
err_out:
	return -ENODEV;
}

static void jz_mmc_deinit_dma(struct jz_mmc_host *host)
{
	jz_free_dma(host->dma.channel);
}

int jz_mmc_dma_register(struct jz_mmc_dma *dma)
{
	if(dma == NULL)
		return -ENOMEM;

	dma->init = jz_mmc_init_dma;
	dma->deinit = jz_mmc_deinit_dma;

	return 0;
}
