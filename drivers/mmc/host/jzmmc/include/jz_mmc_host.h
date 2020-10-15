/*
 *  linux/drivers/mmc/host/jz_mmc/jz_mmc_host.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ_MMC_HOST_H__
#define __JZ_MMC_HOST_H__

#include <linux/semaphore.h>
#include <asm/jzsoc.h>
#include <linux/device.h>

#define JZ_MSC_USE_DMA 1

/* NOTE: when not use bounce buffer, we will force to use descriptor(see following code) */
#define USE_DMA_DESC
//#define USE_DMA_UNCACHE
//#define MSC_DEBUG_DMA

#ifndef CONFIG_MMC_BLOCK_BOUNCE
#ifndef USE_DMA_DESC
#define USE_DMA_DESC
#endif
#endif


#if defined(CONFIG_SOC_JZ4760) || defined(CONFIG_SOC_JZ4760B) || defined(CONFIG_SOC_JZ4770)

#define JZ_MSC_DMA_DESC jz_dma_desc_8word
#define USE_DMA_BUSRT_64

#else
#define JZ_MSC_DMA_DESC jz_dma_desc_8word

#endif

#define MMC_CLOCK_MIN    400000      /* 400 kHz for initial setup */
#define MMC_CLOCK_FAST  20000000      /* 20 MHz for maximum for normal operation */

/*
 * SD_CLOCK_MAX can be any value,
 * The typical value is 24MHZ and 48MHZ,
 * maybe you can try 50MHZ, but what will happen? who knowns? :-)
 *
 * Note that we cannot support >24MHZ on JZ4760(B),
 * and 48MHZ is supposed to unstable on JZ4770 for some SD cards
 *
 * We are very sorry, but you can try faster clock at your own risk.
 */
#if defined(CONFIG_SOC_JZ4770)
#define SD_CLOCK_MAX   48000000
#else
#define SD_CLOCK_MAX   24000000
#endif

/*
 * Please don't change this macro,
 * if you want a frequency less than 24M, change SD_CLOCK_MAX instead
 */
#define SD_CLOCK_24M   24000000

struct jz_mmc_host {
	struct mmc_host *mmc;
	struct semaphore mutex;

	/* host resources */
	//void __iomem *base;
	unsigned int pdev_id;
	int irq;
	int dma_id;
	struct jz_mmc_platform_data *plat;

	/* mmc request related */
	unsigned int cmdat;
	struct mmc_request *curr_mrq;
	int curr_res_type;

	/* data transter related */
	struct {
		int len;
		int dir;
		int channel;
	} dma;
#ifdef USE_DMA_DESC
#ifdef MSC_DEBUG_DMA
	int num_desc;
	int last_direction;
#endif
	JZ_MSC_DMA_DESC *dma_desc;
#endif
	wait_queue_head_t data_wait_queue;
	volatile int data_ack;
	volatile int data_err;

	/* PIO states */
	volatile int transfer_end;

#define JZ_TRANS_MODE_NULL	0
#define JZ_TRANS_MODE_DMA	1
#define JZ_TRANS_MODE_PIO	2
	int transfer_mode;    /* PIO or DMA */

#if 0
	wait_queue_head_t status_check_queue;
	struct timer_list status_check_timer;
	u32 status;
	u32 st_mask;
	int st_check_timeout;
	int st_check_interval;
	int en_usr_intr;
#endif

	/* card detect related */
	volatile unsigned int eject;
	volatile unsigned int oldstat;
	struct delayed_work gpio_jiq_work;
	atomic_t detect_refcnt;
	struct timer_list timer;
	volatile int sleeping;
};

void jz_mmc_finish_request(struct jz_mmc_host *host, struct mmc_request *mrq);

#endif /* __JZ_MMC_HOST_H__ */
