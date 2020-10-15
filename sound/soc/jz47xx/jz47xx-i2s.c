#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include "jz47xx-pcm.h"
#include "jz47xx-i2s.h"
#include "../codecs/jzdlv.h"

#define I2S_RFIFO_DEPTH 32
#define I2S_TFIFO_DEPTH 64

static int jz_i2s_debug = 0;
module_param(jz_i2s_debug, int, 0644);
#define I2S_DEBUG_MSG(msg...)			\
	do {					\
		if (jz_i2s_debug)		\
			printk("I2S: " msg);	\
	} while(0)

static struct jz47xx_dma_client jz47xx_dma_client_out = {
	.name = "I2S PCM Stereo out"
};

static struct jz47xx_dma_client jz47xx_dma_client_in = {
	.name = "I2S PCM Stereo in"
};

static struct jz47xx_pcm_dma_params jz47xx_i2s_pcm_stereo_out = {
	.client		= &jz47xx_dma_client_out,
	.channel	= DMA_ID_AIC_TX,
	.dma_addr	= AIC_DR,
	.dma_size	= 2,
};

static struct jz47xx_pcm_dma_params jz47xx_i2s_pcm_stereo_in = {
	.client		= &jz47xx_dma_client_in,
	.channel	= DMA_ID_AIC_RX,
	.dma_addr	= AIC_DR,
	.dma_size	= 2,
};

static int jz47xx_i2s_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	return 0;
}

static int jz47xx_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai,
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
* Set Jz47xx Clock source
*/
static int jz47xx_i2s_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
		int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static int is_recording = 0;
static int is_playing = 0;

static void jz47xx_snd_tx_ctrl(int on)
{
	if (on)
	{
		is_playing = 1;

        /* enable replay */
	    __i2s_enable_transmit_dma();
		__i2s_enable_replay();
		__i2s_enable();

	} else if (is_playing) {
		is_playing = 0;
		/* disable replay & capture */
		__i2s_disable_transmit_dma();
		__aic_write_tfifo(0x0);
		__aic_write_tfifo(0x0);
		//mdelay(1); //allen del
		while(!__aic_transmit_underrun());
		__i2s_disable_replay();
		__aic_clear_errors();

		if (!is_recording)
			__i2s_disable();
	}
}

static void jz47xx_snd_rx_ctrl(int on)
{
	if (on) 
	{
		is_recording = 1;
		/* enable capture */
		__aic_flush_rfifo();
		mdelay(10);
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

static int jz47xx_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int channels = params_channels(params);

	I2S_DEBUG_MSG("enter %s, substream = %s\n",
		      __func__,
		      (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");

	/* NOTE: when use internal codec, nothing to do with sample rate here.
	 * 	if use external codec and bit clock is provided by I2S controller, set clock rate here!!!
	 */

	/* set channel params */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		cpu_dai->dma_data = &jz47xx_i2s_pcm_stereo_out;
		if (channels == 1) {
			__aic_enable_mono2stereo();
			__aic_out_channel_select(0);
		} else {
			__aic_disable_mono2stereo();
			__aic_out_channel_select(1);
		}
	} else
		rtd->dai->cpu_dai->dma_data = &jz47xx_i2s_pcm_stereo_in;


	/* set format */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_U8:
			__aic_enable_unsignadj();
			__i2s_set_oss_sample_size(8);
			break;
		case SNDRV_PCM_FORMAT_S8:
			__aic_disable_unsignadj();
			__i2s_set_oss_sample_size(8);
			break;
		case SNDRV_PCM_FORMAT_U16_LE:
			__aic_enable_unsignadj();
			__i2s_set_oss_sample_size(16);
			break;
		case SNDRV_PCM_FORMAT_S16_LE:
			__aic_disable_unsignadj();
			__i2s_set_oss_sample_size(16);
			break;
		case SNDRV_PCM_FORMAT_S24_3LE:
			__aic_disable_unsignadj();
			__i2s_set_oss_sample_size(24);
			break;
		}
		
	}
	else {
		int sound_data_width = 0;
		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_U8:
			__aic_enable_unsignadj();
			__i2s_set_iss_sample_size(8);
			sound_data_width = 8;
			break;
		case SNDRV_PCM_FORMAT_S8:
			__aic_disable_unsignadj();
			__i2s_set_iss_sample_size(8);
			sound_data_width = 8;
			break;
		case SNDRV_PCM_FORMAT_S16_LE:
			__aic_disable_unsignadj();
			__i2s_set_iss_sample_size(16);
			sound_data_width = 16;
			break;
		case SNDRV_PCM_FORMAT_S24_3LE:
		default:
			__aic_disable_unsignadj();
			__i2s_set_iss_sample_size(24);
			sound_data_width = 24;
			break;
		}

		/* use 2 sample as trigger */
		__i2s_set_receive_trigger((sound_data_width / 8 * channels) * 2 / 2 - 1);
	}

	return 0;
}

static int jz47xx_i2s_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	int ret = 0;

	I2S_DEBUG_MSG("enter %s, substream = %s cmd = %d\n", __func__,(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture", cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			jz47xx_snd_rx_ctrl(1);
		else
			jz47xx_snd_tx_ctrl(1);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			jz47xx_snd_rx_ctrl(0);
		else
			jz47xx_snd_tx_ctrl(0);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void jz47xx_i2s_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
	} else {
	}

	return;
}

#define __gpio_as_ex_i2s()		\
do {					\
	/*__gpio_as_func2(4*32 + 5); 	sclk_rest*/  \
	__gpio_as_func1(3*32 + 12); /*bclk*/ \
	__gpio_as_func0(3*32 + 13);/*LRCLK*/	 \
	__gpio_as_func0(4*32 + 7);	/*sdao*/  \
	/*__gpio_as_func0(4*32 + 6);	 sdai*/\
} while (0)

static void internal_i2s_init(void)
{
	/* Select exclk as i2s clock */
	cpm_set_clock(CGU_I2SCLK, JZ_EXTAL);
    __i2s_disable();
	__i2s_disable_transmit_dma();
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
	//__i2s_send_rfirst();

	__aic_write_tfifo(0x0);
	__aic_write_tfifo(0x0);
	__i2s_enable_replay();
	__i2s_enable();
	mdelay(1);

	__i2s_disable_replay();
	__i2s_disable();

	jz47xx_snd_tx_ctrl(0);
	jz47xx_snd_rx_ctrl(0);
}

void jz47xx_external_i2s_init(void)
{
	__i2s_disable();
	__aic_select_i2s();
	__aic_play_lastsample();
	
	__i2s_disable_transmit_dma();
	__aic_disable_receive_dma();
	__aic_disable_record();
    __aic_disable_replay();
	__aic_disable_loopback();
	
	__spdif_disable_transmit_dma();
	__spdif_select_i2s();
	__spdif_disable();
	
	__gpio_as_ex_i2s();
	__i2s_external_codec();
	__i2s_select_msbjustified(); //__i2s_select_i2s();
	__i2s_as_slave();

#if 1//allen change
	__i2s_set_transmit_trigger(20);//12
#else
	__i2s_set_transmit_trigger(I2S_TFIFO_DEPTH / 4);
#endif

	__i2s_set_receive_trigger(I2S_RFIFO_DEPTH / 4);

	__i2s_send_lfirst();//__i2s_send_rfirst();
	
	jz47xx_snd_tx_ctrl(0);
	jz47xx_snd_rx_ctrl(0);
}

EXPORT_SYMBOL(jz47xx_external_i2s_init);

static int jz47xx_i2s_probe(struct platform_device *pdev, struct snd_soc_dai *dai)
{
	cpm_start_clock(CGM_AIC);
	
	/* Select exclk as i2s clock */
	REG_AIC_I2SCR |= AIC_I2SCR_ESCLK;
	
	//internal_i2s_init();
	//jz47xx_external_i2s_init(); //power-on needn't init

	return 0;
}

#ifdef CONFIG_PM
static int jz47xx_i2s_suspend(struct snd_soc_dai *dai)
{
	return 0;
}

static int jz47xx_i2s_resume(struct snd_soc_dai *dai)
{
	return 0;
}

#else
#define jz47xx_i2s_suspend	NULL
#define jz47xx_i2s_resume	NULL
#endif

#define JZ47xx_I2S_RATES (SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |	\
			  SNDRV_PCM_RATE_48000  | SNDRV_PCM_RATE_64000  |	\
			  SNDRV_PCM_RATE_88200  | SNDRV_PCM_RATE_96000  |	\
			  SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000  )

static struct snd_soc_dai_ops jz47xx_i2s_dai_ops = {
	.startup = jz47xx_i2s_startup,
	.shutdown = jz47xx_i2s_shutdown,
	.trigger = jz47xx_i2s_trigger,
	.hw_params = jz47xx_i2s_hw_params,
	.set_fmt = jz47xx_i2s_set_dai_fmt,
	.set_sysclk = jz47xx_i2s_set_dai_sysclk,
};

//#define JZ_I2S_FORMATS (SNDRV_PCM_FMTBIT_S8  | SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_3LE)
#define JZ_I2S_FORMATS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE)

struct snd_soc_dai jz47xx_i2s_dai = {
	.name = "jz47xx-i2s",
	.id = 0,
	.probe = jz47xx_i2s_probe,
	.suspend = jz47xx_i2s_suspend,
	.resume = jz47xx_i2s_resume,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = JZ47xx_I2S_RATES,
		.formats = JZ_I2S_FORMATS,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = JZ47xx_I2S_RATES,
		.formats = JZ_I2S_FORMATS,
	},
	.ops = &jz47xx_i2s_dai_ops,
};

EXPORT_SYMBOL_GPL(jz47xx_i2s_dai);

static int __init jz47xx_i2s_init(void)
{
	return snd_soc_register_dai(&jz47xx_i2s_dai);
}
module_init(jz47xx_i2s_init);

static void __exit jz47xx_i2s_exit(void)
{
	snd_soc_unregister_dai(&jz47xx_i2s_dai);
}
module_exit(jz47xx_i2s_exit);

MODULE_AUTHOR("Lutts Wolf <slcao@ingenic.cn>");
MODULE_DESCRIPTION("jz47xx I2S SoC Interface");
MODULE_LICENSE("GPL");
