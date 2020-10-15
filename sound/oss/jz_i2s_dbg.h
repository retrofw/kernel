/*
 * Linux/sound/oss/jz_i2s_dbg.h
 *
 * 2010-01-xx	Jason <xwang@ingenic.cn>
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ_I2S_DBG_H__
#define __JZ_I2S_DBG_H__


/*
 * Select debug level by define various value
 *
 * Detailedness level ( 2 > 1 > 0 > undefine)
 *
 * Depend on AIC_DEBUG_LEVELX
 */
#define AIC_DEBUG_LEVEL			3


#if AIC_DEBUG_LEVEL == 2
  #define AIC_DEBUG_LEVEL2		1
#endif

#if AIC_DEBUG_LEVEL == 1
  #define AIC_DEBUG_LEVEL1		1
#endif

#if AIC_DEBUG_LEVEL == 0
  #define AIC_DEBUG_LEVEL0		1
#endif

#ifdef AIC_DEBUG_LEVEL2
  #define REG_DEBUG			0
  #define DMA_DEBUG			0
  #define BUF_DEBUG			0
  #define Q_DEBUG			0
  #define TRACE_DEBUG			0
  #define IRQ_DEBUG			0
  #define IOC_DEBUG			0
  #define CODEC_DEBUG			0
  #define OTHER_DEBUG			0
#endif

#ifdef AIC_DEBUG_LEVEL1
  #define OTHER_DEBUG			0
  #define IRQ_DEBUG			0
  #define DMA_DEBUG			0
  #define REG_DEBUG			0
//  #define IOC_DEBUG			0
#endif

#ifdef AIC_DEBUG_LEVEL0
  #define IRQ_DEBUG			0
  #define TRACE_DEBUG			0
#endif

//#ifdef CODEC_DEBUG
#if 0
  #define DPRINT_CODEC(msg...)		printk(msg)
#else
  #define DPRINT_CODEC(msg...)		do{} while (0)
#endif

#ifdef REG_DEBUG
  #define DUMP_AIC_REGS(msg...)		dump_aic_regs(msg)
  #define DUMP_CODEC_REGS(msg...)	dump_dlv_regs(msg)
#else
  #define DUMP_AIC_REGS(msg...)		do{} while (0)
  #define DUMP_CODEC_REGS(msg...)	do{} while (0)
#endif

#ifdef IOC_DEBUG
  #define DPRINT_IOC(msg...)		printk(msg)
  #define DPRINT_DLV_IOC_CMD(msg...)	dlv_print_ioc_cmd(msg)
  #define DPRINT_MIXER_IOC_CMD(msg...)	mixer_print_ioc_cmd(msg)
  #define DPRINT_DSP_IOC_CMD(msg...)	dsp_print_ioc_cmd(msg)
#else
  #define DPRINT_IOC(msg...)		do{} while (0)
  #define DPRINT_DLV_IOC_CMD(msg...)	do{} while (0)
  #define DPRINT_MIXER_IOC_CMD(msg...)	do{} while (0)
  #define DPRINT_DSP_IOC_CMD(msg...)	do{} while (0)
#endif

#ifdef BUF_DEBUG
  #define DUMP_BUF(msg...)		dump_buf(msg)
  #define DPRINT_BUF(msg...)		printk(msg)
#else
  #define DUMP_BUF(msg...)		do{} while (0)
  #define DPRINT_BUF(msg...)		do{} while (0)
#endif

#ifdef DMA_DEBUG
  #define DUMP_DMA(arg...)		dump_dma(arg)
  #define DPRINT_DMA(msg...)		printk(msg)
#else
  #define DUMP_DMA(arg...)		do{} while (0)
  #define DPRINT_DMA(msg...)		do{} while (0)
#endif

#ifdef TRACE_DEBUG
  #define ENTER()			printk("Enter: %s, %s:%i\n", __FUNCTION__, __FILE__, __LINE__)
  #define LEAVE()			printk("Leave: %s, %s:%i\n", __FUNCTION__, __FILE__, __LINE__)
  #define DPRINT_TRC(msg...)		printk(msg)
#else
  #define ENTER()			do{} while (0)
  #define LEAVE()			do{} while (0)
  #define DPRINT_TRC(msg...)		do{} while (0)
#endif

#ifdef IRQ_DEBUG
  #define DPRINT_IRQ(msg...)		printk(msg)
#else
  #define DPRINT_IRQ(msg...)		do{} while (0)
#endif

#ifdef Q_DEBUG
  #define DUMP_NODE(msg...)		dump_node(msg)
  #define DUMP_LIST(msg...)		dump_list(msg)
  #define DPRINT_Q(msg...)		printk(msg)
#else
  #define DUMP_NODE(msg...)		do{} while (0)
  #define DUMP_LIST(msg...)		do{} while (0)
  #define DPRINT_Q(msg...)		do{} while (0)
#endif

#ifdef OTHER_DEBUG
  #define DPRINT(msg...)		printk(msg)
#else
  #define DPRINT(msg...)		do{} while (0)
#endif


#endif /*__JZ_I2S_DBG_H__*/
