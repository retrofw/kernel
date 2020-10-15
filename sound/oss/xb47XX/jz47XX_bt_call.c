/*
 * Linux/sound/oss/jz4760_bt_call.c
 *
 * DLV CODEC driver for Ingenic Jz4750 MIPS processor
 *
 * 2010-07-xx	Lucifer <yliu@ingenic.cn>
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/sound.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <linux/proc_fs.h>
#include <linux/soundcard.h>
#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/mm.h>

#include <asm/hardirq.h>
#include <asm/jzsoc.h>

#include "jz47XX_bt_call.h"

struct bt_call_pcm_info gbt_call_pcm;
extern void bt_call_down_stream_handle_buffer(void);
extern void bt_call_up_stream_handle_buffer(void);

#if 0
static void dump_pcmc_reg(void)
{
	printk("REG_DMAC_DMACKE(0) = 0x%08x\n", REG_DMAC_DMACKE(0));
	printk("REG_DMAC_DMACKE(1) = 0x%08x\n", REG_DMAC_DMACKE(1));
	printk("REG_CPM_CLKGR	= 0x%08x\n", REG_CPM_CLKGR);
	printk("REG_CPM_CPCCR	= 0x%08x\n", REG_CPM_CPCCR);
	printk("REG_CPM_PCMCDR	= 0x%08x\n", REG_CPM_PCMCDR);
	printk("REG_PCM_PCLT	= 0x%08x\n", REG_PCM_PCTL);
	printk("REG_PCM_PCFG	= 0x%08x\n", REG_PCM_PCFG);
	printk("REG_PCM_PINTC 	= 0x%08x\n", REG_PCM_PINTC);
	printk("REG_PCM_PINTS	= 0x%08x\n", REG_PCM_PINTS);
	printk("REG_PCM_PDIV	= 0x%08x\n", REG_PCM_PDIV);
}

static void dump_dma(unsigned int dmanr, const char *str)
{
	printk("DMA%d Registers, %s:\n", dmanr, str);
	printk("\tDMACR	= 0x%08x\n", REG_DMAC_DMACR(dmanr/HALF_DMA_NUM));
	printk("\tDSAR	= 0x%08x\n", REG_DMAC_DSAR(dmanr));
	printk("\tDTAR	= 0x%08x\n", REG_DMAC_DTAR(dmanr));
	printk("\tDTCR	= 0x%08x\n", REG_DMAC_DTCR(dmanr));
	printk("\tDRSR	= 0x%08x\n", REG_DMAC_DRSR(dmanr));
	printk("\tDCCSR	= 0x%08x\n", REG_DMAC_DCCSR(dmanr));
	printk("\tDCMD	= 0x%08x\n", REG_DMAC_DCMD(dmanr));
	printk("\tDDA	= 0x%08x\n", REG_DMAC_DDA(dmanr));
	printk("\tDMADBR= 0x%08x\n", REG_DMAC_DMADBR(dmanr/HALF_DMA_NUM));
}

#endif

static void bt_call_pcm_reset(void)
{
	__pcm_reset(CUR_PCM);
	__pcm_flush_fifo(CUR_PCM);
	__pcm_disable_txfifo(CUR_PCM);
	__pcm_disable_rxfifo(CUR_PCM);
	__pcm_disable_tfs_intr(CUR_PCM);
	__pcm_enable_tur_intr(CUR_PCM);
	__pcm_disable_rfs_intr(CUR_PCM);
	__pcm_enable_ror_intr(CUR_PCM);
	__pcm_disable_ror_intr(CUR_PCM);
	__pcm_disable_receive_dma(CUR_PCM);
	__pcm_disable_transmit_dma(CUR_PCM);
}

static void bt_call_pcm_cfg(struct bt_call_pcm_cfg *cfg)
{
	if (cfg->omsb_pos == PCM_MSB_POS_SAME)
		__pcm_omsb_same_sync(CUR_PCM);
	else
		__pcm_omsb_next_sync(CUR_PCM);

	if (cfg->imsb_pos == PCM_MSB_POS_SAME)
		__pcm_imsb_same_sync(CUR_PCM);
	else
		__pcm_imsb_next_sync(CUR_PCM);

	__pcm_set_iss(CUR_PCM, cfg->iss_bit);
	__pcm_set_oss(CUR_PCM, cfg->oss_bit);

	if (cfg->mode == PCM_MODE_MASTER)
		__pcm_as_master(CUR_PCM);
	else
		__pcm_as_slave(CUR_PCM);

	__pcm_set_valid_slot(CUR_PCM, cfg->time_slot);
	__pcm_set_sync_len(CUR_PCM, 0);
}

/* GPIO init */
static void bt_call_pcm_gpio_init(void)
{
	__gpio_as_pcm(CUR_PCM);
}

/* clk size in Hz */
static void bt_call_pcm_clk_init(int clk)
{
	REG_CPM_CLKGR1 &= ~( 1<<8 );
	__pcm_enable(CUR_PCM);
	__pcm_clk_enable(CUR_PCM);

	__pcm_set_clk_rate(CUR_PCM, 12000000,1024000);
	/* PCMSYNC must be 8000 Hz. 2048000 / 256 = 8000 */
	__pcm_set_sync_rate(CUR_PCM, 1024000,clk);
	__pcm_set_sync_len(CUR_PCM, 0);
}

static irqreturn_t pcm_record_dma_irq(int irq, void *dev_id)
{
	int dma = gbt_call_pcm.dma_in_ch;

	disable_dma(dma);
	if (__dmac_channel_address_error_detected(dma)) {
		printk(KERN_DEBUG "%s: DMAC address error.\n", __FUNCTION__);
		__dmac_channel_clear_address_error(dma);
	}
	if (__dmac_channel_transmit_end_detected(dma)) {
		__dmac_channel_clear_transmit_end(dma);
		bt_call_up_stream_handle_buffer();
	}

	return IRQ_HANDLED;
}

static irqreturn_t pcm_replay_dma_irq(int irq, void *dev_id)
{
	int dma = gbt_call_pcm.dma_out_ch;
	disable_dma(dma);
	if (__dmac_channel_address_error_detected(dma)) {
		printk(KERN_DEBUG "%s: DMAC address error.\n", __FUNCTION__);
		__dmac_channel_clear_address_error(dma);
	}
	if (__dmac_channel_transmit_end_detected(dma)) {
		__dmac_channel_clear_transmit_end(dma);
		bt_call_down_stream_handle_buffer();
	}

	return IRQ_HANDLED;
}

static irqreturn_t bt_call_pcm_irq(int irq, void *dev_id)
{
	unsigned int pints = REG_PCM_PINTS(CUR_PCM);
	if (pints & PINTS_TUR) {
		printk("PCM TXFIFO under run!\n");
		REG_PCM_PINTS(CUR_PCM) &= ~PINTS_TUR;
	} else if (pints & PINTS_ROR) {
		printk("PCM RXFIFO over run!\n");
		REG_PCM_PINTS(CUR_PCM) &= ~PINTS_ROR;
	}

	return IRQ_HANDLED;
}

/* clk size in Hz */
void bt_call_pcm_init(void)
{
	int err;
	struct bt_call_pcm_cfg cfg;

	bt_call_pcm_gpio_init();
	bt_call_pcm_clk_init(8000);
	bt_call_pcm_reset();

	cfg.omsb_pos = cfg.imsb_pos = PCM_MSB_POS_NEXT;
	cfg.iss_bit = cfg.oss_bit = PCM_SSBIT_16;
	cfg.time_slot = 0;
	cfg.mode = PCM_MODE_SLAVE;
	bt_call_pcm_cfg(&cfg);

	if ((gbt_call_pcm.dma_out_ch = jz_request_dma(DMA_ID_PCM_TX, "PCM output", 
					       pcm_replay_dma_irq, IRQF_DISABLED, NULL)) < 0) {
		printk(KERN_ERR "can't reqeust DMA DAC channel.\n");
		return;
	}

	if ((gbt_call_pcm.dma_in_ch = jz_request_dma(DMA_ID_PCM_RX, "PCM input", 
					       pcm_record_dma_irq, IRQF_DISABLED, NULL)) < 0) {
		printk(KERN_ERR "can't reqeust DMA ADC channel.\n");
		return;
	}

	printk("JzSOC On-Chip PCM controller registered (DAC: DMA(play):%d \n "
	       "ADC: DMA(record):%d \n", 
	       gbt_call_pcm.dma_out_ch, gbt_call_pcm.dma_in_ch);

	err = request_irq(IRQ_PCM, bt_call_pcm_irq, IRQF_DISABLED, "pcm irq", NULL);
	if (err < 0)
		printk("can't allocate pcm irq.\n");
	
//	dump_pcmc_reg();
}

void bt_call_pcm_start_dma(unsigned char *buf, int size, int path)
{
	int chan;

	/* replay mode */
	if (path == BT_CALL_DOWN_STREAM) {
		chan = gbt_call_pcm.dma_out_ch;

		__pcm_enable_transmit_dma(CUR_PCM);
		__pcm_enable_txfifo(CUR_PCM);
		__pcm_set_transmit_trigger(CUR_PCM, 15);
		disable_dma(chan);
		clear_dma_ff(chan);

		/* we discard one sound channel by using 32bit mode */
		/* DMA write 32bit data into PCM, PCM will discard high 16bit itself */
		REG_DMAC_DCMD(chan) =  DMAC_DCMD_SAI | DMAC_DCMD_SWDH_32 | DMAC_DCMD_TIE | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BIT;
		REG_DMAC_DRSR(chan) =  DMAC_DRSR_RS_PMOUT;
		REG_DMAC_DTAR(chan) =  (unsigned int)CPHYSADDR(PCM_PDP(CUR_PCM));
		REG_DMAC_DSAR(chan) =  (unsigned int)CPHYSADDR(buf);
		/* get one sample(32bit) one time */
		REG_DMAC_DTCR(chan) = size >> 2;
		__dmac_enable_module(1);
		__dmac_enable_module(0);
		enable_dma(chan);
//		dump_dma(chan, "pcmout");
	} else {
		/* record mode */
		chan = gbt_call_pcm.dma_in_ch;

		__pcm_enable_receive_dma(CUR_PCM);
		__pcm_set_receive_trigger(CUR_PCM, 4);
		__pcm_enable_rxfifo(CUR_PCM);
		disable_dma(chan);
		clear_dma_ff(chan);

		REG_DMAC_DMACR(chan / HALF_DMA_NUM) |= 1;
		REG_DMAC_DCMD(chan) =  DMAC_DCMD_DAI | DMAC_DCMD_SWDH_16 | DMAC_DCMD_TIE | DMAC_DCMD_DWDH_16 | DMAC_DCMD_DS_16BIT;
		REG_DMAC_DRSR(chan) =  DMAC_DRSR_RS_PMIN;
		REG_DMAC_DTAR(chan) =  (unsigned int)CPHYSADDR(buf);
		REG_DMAC_DSAR(chan) =  (unsigned int)CPHYSADDR(PCM_PDP(CUR_PCM));
		/* get one sample(16bit) one time */
		REG_DMAC_DTCR(chan) = size >> 1;
		__dmac_enable_module(1);
		enable_dma(chan);
	}
}

void bt_call_pcm_stop(void)
{
	/* disable PCM */
	__pcm_disable(CUR_PCM);

	/* disable DMA first */
	disable_dma(gbt_call_pcm.dma_out_ch);
	clear_dma_ff(gbt_call_pcm.dma_out_ch);

	disable_dma(gbt_call_pcm.dma_in_ch);
	clear_dma_ff(gbt_call_pcm.dma_in_ch);

	/* free dma channel */
	jz_free_dma(gbt_call_pcm.dma_out_ch);
	jz_free_dma(gbt_call_pcm.dma_in_ch);

	/* clean all PCM irq flags*/

	/* free irq */
	free_irq(IRQ_PCM,NULL);
}
