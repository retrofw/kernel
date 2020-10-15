/*
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include "jz4750-pcm.h"
#include "jz4750-i2s.h"
#include "../codecs/jzdlv.h"

static struct jz4750_dma_client jz4750_dma_client_out = {
	.name = "I2S PCM Stereo out"
};

static struct jz4750_dma_client jz4750_dma_client_in = {
	.name = "I2S PCM Stereo in"
};

static struct jz4750_pcm_dma_params jz4750_i2s_pcm_stereo_out = {
	.client		= &jz4750_dma_client_out,
	.channel	= DMA_ID_AIC_TX,
	.dma_addr	= AIC_DR,
	.dma_size	= 2,
};

static struct jz4750_pcm_dma_params jz4750_i2s_pcm_stereo_in = {
	.client		= &jz4750_dma_client_in,
	.channel	= DMA_ID_AIC_RX,
	.dma_addr	= AIC_DR,
	.dma_size	= 2,
};

static int jz4750_i2s_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	/*struct snd_soc_pcm_runtime *rtd = substream->private_data;
	  struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;*/

	return 0;
}

static int jz4750_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/* 1 : ac97 , 0 : i2s */
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
	        /* 0 : slave */ 
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		/* 1 : master */
		break;
	default:
		break;
	}
	
	return 0;
}

/* 
* Set Jz4750 Clock source
*/
static int jz4750_i2s_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
		int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static void jz4750_snd_tx_ctrl(int on)
{
	if (on) { 
                /* enable replay */
	        __i2s_enable_transmit_dma();
		__i2s_enable_replay();
		__i2s_enable();

	} else {
		/* disable replay & capture */
		__i2s_disable_replay();
		__i2s_disable_record();
		__i2s_disable_receive_dma();
		__i2s_disable_transmit_dma();
		__i2s_disable();
	}
}

static void jz4750_snd_rx_ctrl(int on)
{
	if (on) { 
                /* enable capture */
		__i2s_enable_receive_dma();
		__i2s_enable_record();
		__i2s_enable();

	} else { 
                /* disable replay & capture */
		__i2s_disable_replay();
		__i2s_disable_record();
		__i2s_disable_receive_dma();
		__i2s_disable_transmit_dma();
		__i2s_disable();
	}
}

static int jz4750_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int channels = params_channels(params);
	
	jz4750_snd_rx_ctrl(0);
	jz4750_snd_rx_ctrl(0);
	write_codec_file_bit(5, 0, 7);//PMR1.SB_DAC->0
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		cpu_dai->dma_data = &jz4750_i2s_pcm_stereo_out;
		
		if (channels == 1)
			__aic_enable_mono2stereo();
		else
			__aic_disable_mono2stereo();
	} else
		rtd->dai->cpu_dai->dma_data = &jz4750_i2s_pcm_stereo_in;

#if 1
	switch (channels) {
	case 1:
		write_codec_file_bit(1, 1, 6);//CR1.MONO->1 for Mono
		break;
	case 2:
		write_codec_file_bit(1, 0, 6);//CR1.MONO->0 for Stereo
		break;
	}
#endif
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		__i2s_set_transmit_trigger(4);
		__i2s_set_receive_trigger(3);
		__i2s_set_oss_sample_size(8);
		__i2s_set_iss_sample_size(8);
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		/* playback sample:16 bits, burst:16 bytes */
		__i2s_set_transmit_trigger(4);
		/* capture sample:16 bits, burst:16 bytes */
		__i2s_set_receive_trigger(3);
		__i2s_set_oss_sample_size(16);
		__i2s_set_iss_sample_size(16);
		/* DAC path and ADC path */
		write_codec_file(2, 0x00);
		break;
	}

	return 0;
}

static int jz4750_i2s_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	int ret = 0;
	
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			jz4750_snd_rx_ctrl(1);
		else
			jz4750_snd_tx_ctrl(1);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			jz4750_snd_rx_ctrl(0);
		else
			jz4750_snd_tx_ctrl(0);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void jz4750_i2s_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
	} else {
	}

	return;
}

static int jz4750_i2s_probe(struct platform_device *pdev, struct snd_soc_dai *dai)	
{
	__i2s_internal_codec();
	__i2s_as_slave();
	__i2s_select_i2s();
	__aic_select_i2s();
	mdelay(2);

	__i2s_disable();
	mdelay(2);
	REG_AIC_I2SCR = 0x10;
   	__i2s_disable();
	__i2s_internal_codec();
	__i2s_as_slave();
	__i2s_select_i2s();
	__aic_select_i2s();
	__i2s_set_oss_sample_size(16);
	__i2s_set_iss_sample_size(16);
        __aic_play_lastsample();

	__i2s_disable_record();
	__i2s_disable_replay();
	__i2s_disable_loopback();
	__i2s_set_transmit_trigger(7);
	__i2s_set_receive_trigger(7);

	jz4750_snd_tx_ctrl(0);
	jz4750_snd_rx_ctrl(0);

	return 0;
}

#ifdef CONFIG_PM
static int jz4750_i2s_suspend(struct snd_soc_dai *dai)
{
	if (!dai->active)
		return 0;

	return 0;
}

static int jz4750_i2s_resume(struct snd_soc_dai *dai)
{
	if (!dai->active)
		return 0;

	return 0;
}

#else
#define jz4750_i2s_suspend	NULL
#define jz4750_i2s_resume	NULL
#endif

#define JZ4750_I2S_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_12000 | SNDRV_PCM_RATE_16000 |\
		SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_24000 |\
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
		SNDRV_PCM_RATE_48000)

static struct snd_soc_dai_ops jz4750_i2s_dai_ops = {
	.startup = jz4750_i2s_startup,
	.shutdown = jz4750_i2s_shutdown,
	.trigger = jz4750_i2s_trigger,
	.hw_params = jz4750_i2s_hw_params,
	.set_fmt = jz4750_i2s_set_dai_fmt,
	.set_sysclk = jz4750_i2s_set_dai_sysclk,
};

struct snd_soc_dai jz4750_i2s_dai = {
	.name = "jz4750-i2s",
	.id = 0,
	.probe = jz4750_i2s_probe,
	.suspend = jz4750_i2s_suspend,
	.resume = jz4750_i2s_resume,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = JZ4750_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE,}, 
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = JZ4750_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &jz4750_i2s_dai_ops,
};

EXPORT_SYMBOL_GPL(jz4750_i2s_dai);

static int __init jz4750_i2s_init(void)
{
	return snd_soc_register_dai(&jz4750_i2s_dai);
}
module_init(jz4750_i2s_init);

static void __exit jz4750_i2s_exit(void)
{
	snd_soc_unregister_dai(&jz4750_i2s_dai);
}
module_exit(jz4750_i2s_exit);

/* Module information */
MODULE_AUTHOR("Richard, cjfeng@ingenic.cn, www.ingenic.cn");
MODULE_DESCRIPTION("jz4750 I2S SoC Interface");
MODULE_LICENSE("GPL");
