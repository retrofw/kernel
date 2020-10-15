/*
 * Linux/sound/oss/jz47xx_i2s.c
 *
 * I2S controller for Ingenic Jz47xx MIPS processor
 *
 * 2011-11-xx	liulu <lliu@ingenic.cn>
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */
#include <linux/module.h>
#include <linux/soundcard.h>
#include <asm/jzsoc.h>
#include <linux/fs.h>
#include "jz_snd.h"

#define I2S_RFIFO_DEPTH 32
#define I2S_TFIFO_DEPTH 64
//FIXME: use external codec????
static int is_recording = 0;
static int is_playing = 0;
/* static int g_record_data_width = 16; */
/* static int g_record_channels = 2; */

static int jz47xx_i2s_debug = 0;
module_param(jz47xx_i2s_debug, int, 0644);
#define JZ47XX_I2S_DEBUG_MSG(msg...)			\
	do {					\
		if (jz47xx_i2s_debug)		\
			printk("I2S: " msg);	\
	} while(0)


void jz47xx_i2s_dump_regs(const char *str)
{
	char *regname[] = {"aicfr","aiccr","aiccr1","aiccr2","i2scr","aicsr","acsr","i2ssr",
			   "accar", "accdr", "acsar", "acsdr", "i2sdiv", "aicdr"};
	int i;
	unsigned int addr;

	printk("AIC regs dump, %s\n", str);
	for (i = 0; i < 13*4; i += 4) {
		addr = 0xb0020000 + i;
		printk("%s\t0x%08x -> 0x%08x\n", regname[i/4], addr, *(unsigned int *)addr);
	}
}

void jz47xx_i2s_tx_ctrl(int on)
{
	JZ47XX_I2S_DEBUG_MSG("enter %s, on = %d\n", __func__, on);
	if (on) {
		is_playing = 1;
        /* enable replay */
        __i2s_enable_transmit_dma();
		__i2s_enable_replay();
		__i2s_enable();
	} else {
		is_playing = 0;
		/* disable replay & capture */
		__i2s_disable_replay();
		__i2s_disable_transmit_dma();

		if (!is_recording)
			__i2s_disable();
	}
}

void jz47xx_i2s_rx_ctrl(int on)
{
	JZ47XX_I2S_DEBUG_MSG("enter %s, on = %d\n", __func__, on);
	if (on) {
		is_recording = 1;
                /* enable capture */
		__i2s_enable_receive_dma();
		__i2s_enable_record();
		__i2s_enable();
	} else {
		is_recording = 0;
                /* disable replay & capture */
		__i2s_disable_record();
		__i2s_disable_receive_dma();

		if (!is_playing)
			__i2s_disable();
	}
}

int jz47xx_i2s_set_channels(int mode, int channels)
{
	JZ47XX_I2S_DEBUG_MSG("enter %s, mode = %d, channels = %d\n", __func__, mode, channels);
	if (mode & MODE_REPLAY) {
		if (channels == 1) {
			__aic_enable_mono2stereo();
			__aic_out_channel_select(0);
		} else {
			__aic_disable_mono2stereo();
			__aic_out_channel_select(1);
		}
	}
	if (mode & MODE_RECORD) {
        /* g_record_channels = channels; */
        /* g_record_channels = 2; */
        /* __i2s_set_receive_trigger((g_record_data_width / 8 * g_record_channels) * 2 / 2 - 1); */
	}

    return 0;
}

#define I2S_FIFO_DEPTH 32
int jz47xx_i2s_set_width(int mode, int width){
    int onetrans_bit = 16*8;

	JZ47XX_I2S_DEBUG_MSG("enter %s, mode = %d, width = %d\n", __func__, mode, width);
    switch(width){
    case 8:
        onetrans_bit = 16 * 8;
        break;
    case 16:
        onetrans_bit = 16 * 8;
        break;
    case 17 ... 32:
        onetrans_bit = 32 * 8;
        break;
    default:
        printk("%s: Unkown mode(sound data width) %d\n", __FUNCTION__, width);
        break;
    }
	if (mode & MODE_REPLAY) {
        __i2s_set_oss_sample_size(width);
		if ((I2S_FIFO_DEPTH - onetrans_bit / width) >= 30) {
			__i2s_set_transmit_trigger(14);
		} else {
			__i2s_set_transmit_trigger((I2S_FIFO_DEPTH - onetrans_bit / width) / 2);
		}
	}
    if (mode & MODE_RECORD){
        __i2s_set_iss_sample_size(width);
        /* g_record_data_width = width; */
        /* __i2s_set_receive_trigger((g_record_data_width / 8 * g_record_channels) * 2 / 2 - 1); */
		__i2s_set_receive_trigger((onetrans_bit / width) / 2);
	}

	return 0;
}

int __init jz47xx_i2s_init(void)
{
	JZ47XX_I2S_DEBUG_MSG("enter %s\n", __func__);

	cpm_start_clock(CGM_AIC);
	/* Select exclk as i2s clock */
	cpm_set_clock(CGU_I2SCLK, JZ_EXTAL);
	REG_AIC_I2SCR |= AIC_I2SCR_ESCLK;

	__i2s_disable();
	__aic_disable_transmit_dma();
	__aic_disable_receive_dma();
	__i2s_disable_record();
	__i2s_disable_replay();
	__i2s_disable_loopback();

	__i2s_internal_codec();
	__i2s_as_slave();
	__i2s_select_i2s();
	__aic_select_i2s();
	__aic_play_lastsample();
	__i2s_set_transmit_trigger(I2S_TFIFO_DEPTH / 4);
	__i2s_set_receive_trigger(I2S_RFIFO_DEPTH / 4);
	__i2s_send_rfirst();

	__aic_write_tfifo(0x0);
	__aic_write_tfifo(0x0);
	__i2s_enable_replay();
	__i2s_enable();
	mdelay(1);

	jz47xx_i2s_tx_ctrl(0);
	jz47xx_i2s_rx_ctrl(0);


    return 0;
}

void __exit jz47xx_i2s_deinit(void)
{
	JZ47XX_I2S_DEBUG_MSG("enter %s\n", __func__);
    //do nothing
    return;
}
