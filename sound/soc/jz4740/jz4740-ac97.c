/*
 * linux/sound/jz4740-ac97.c -- AC97 support for the Ingenic jz4740 chip.
 *
 * Author:	Richard
 * Created:	Dec 02, 2007
 * Copyright:	Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <asm/irq.h>
#include <linux/mutex.h>
#include <asm/hardware.h>
#include <asm/arch/audio.h>

#include "jz4740-pcm.h"
#include "jz4740-ac97.h"

static DEFINE_MUTEX(car_mutex);
static DECLARE_WAIT_QUEUE_HEAD(gsr_wq);
static volatile long gsr_bits;

static unsigned short jz4740_ac97_read(struct snd_ac97 *ac97,
	unsigned short reg)
{
	unsigned short val = -1;
	volatile u32 *reg_addr;

	mutex_lock(&car_mutex);

out:	mutex_unlock(&car_mutex);
	return val;
}

static void jz4740_ac97_write(struct snd_ac97 *ac97, unsigned short reg,
	unsigned short val)
{
	volatile u32 *reg_addr;

	mutex_lock(&car_mutex);

	mutex_unlock(&car_mutex);
}

static void jz4740_ac97_warm_reset(struct snd_ac97 *ac97)
{
	gsr_bits = 0;
}

static void jz4740_ac97_cold_reset(struct snd_ac97 *ac97)
{
}

static irqreturn_t jz4740_ac97_irq(int irq, void *dev_id)
{
	long status;
	return IRQ_NONE;
}

struct snd_ac97_bus_ops soc_ac97_ops = {
	.read	= jz4740_ac97_read,
	.write	= jz4740_ac97_write,
	.warm_reset	= jz4740_ac97_warm_reset,
	.reset	= jz4740_ac97_cold_reset,
};

static struct jz4740_pcm_dma_params jz4740_ac97_pcm_stereo_out = {
	.name			= "AC97 PCM Stereo out",
	.dev_addr		= __PREG(PCDR),
	.drcmr			= &DRCMRTXPCDR,
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST32 | DCMD_WIDTH4,
};

static struct jz4740_pcm_dma_params jz4740_ac97_pcm_stereo_in = {
	.name			= "AC97 PCM Stereo in",
	.dev_addr		= __PREG(PCDR),
	.drcmr			= &DRCMRRXPCDR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST32 | DCMD_WIDTH4,
};

static struct jz4740_pcm_dma_params jz4740_ac97_pcm_aux_mono_out = {
	.name			= "AC97 Aux PCM (Slot 5) Mono out",
	.dev_addr		= __PREG(MODR),
	.drcmr			= &DRCMRTXMODR,
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST16 | DCMD_WIDTH2,
};

static struct jz4740_pcm_dma_params jz4740_ac97_pcm_aux_mono_in = {
	.name			= "AC97 Aux PCM (Slot 5) Mono in",
	.dev_addr		= __PREG(MODR),
	.drcmr			= &DRCMRRXMODR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST16 | DCMD_WIDTH2,
};

static struct jz4740_pcm_dma_params jz4740_ac97_pcm_mic_mono_in = {
	.name			= "AC97 Mic PCM (Slot 6) Mono in",
	.dev_addr		= __PREG(MCDR),
	.drcmr			= &DRCMRRXMCDR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST16 | DCMD_WIDTH2,
};

#ifdef CONFIG_PM
static int jz4740_ac97_suspend(struct platform_device *pdev,
	struct snd_soc_cpu_dai *dai)
{
	return 0;
}

static int jz4740_ac97_resume(struct platform_device *pdev,
	struct snd_soc_cpu_dai *dai)
{
	return 0;
}

#else
#define jz4740_ac97_suspend	NULL
#define jz4740_ac97_resume	NULL
#endif

static int jz4740_ac97_probe(struct platform_device *pdev)
{
	int ret;

	return 0;
}

static void jz4740_ac97_remove(struct platform_device *pdev)
{
}

static int jz4740_ac97_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		cpu_dai->dma_data = &jz4740_ac97_pcm_stereo_out;
	else
		cpu_dai->dma_data = &jz4740_ac97_pcm_stereo_in;

	return 0;
}

static int jz4740_ac97_hw_aux_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		cpu_dai->dma_data = &jz4740_ac97_pcm_aux_mono_out;
	else
		cpu_dai->dma_data = &jz4740_ac97_pcm_aux_mono_in;

	return 0;
}

static int jz4740_ac97_hw_mic_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return -ENODEV;
	else
		cpu_dai->dma_data = &jz4740_ac97_pcm_mic_mono_in;

	return 0;
}

#define JZ4740_AC97_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000)

struct snd_soc_cpu_dai jz4740_ac97_dai[] = {
{
	.name = "jz4740-ac97",
	.id = 0,
	.type = SND_SOC_DAI_AC97,
	.probe = jz4740_ac97_probe,
	.remove = jz4740_ac97_remove,
	.suspend = jz4740_ac97_suspend,
	.resume = jz4740_ac97_resume,
	.playback = {
		.stream_name = "AC97 Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = JZ4740_AC97_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.stream_name = "AC97 Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = JZ4740_AC97_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = {
		.hw_params = jz4740_ac97_hw_params,},
},
{
	.name = "jz4740-ac97-aux",
	.id = 1,
	.type = SND_SOC_DAI_AC97,
	.playback = {
		.stream_name = "AC97 Aux Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = JZ4740_AC97_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.stream_name = "AC97 Aux Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = JZ4740_AC97_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = {
		.hw_params = jz4740_ac97_hw_aux_params,},
},
{
	.name = "jz4740-ac97-mic",
	.id = 2,
	.type = SND_SOC_DAI_AC97,
	.capture = {
		.stream_name = "AC97 Mic Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = JZ4740_AC97_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = {
		.hw_params = jz4740_ac97_hw_mic_params,},
},
};

EXPORT_SYMBOL_GPL(jz4740_ac97_dai);
EXPORT_SYMBOL_GPL(soc_ac97_ops);

MODULE_AUTHOR("Richard");
MODULE_DESCRIPTION("AC97 driver for the Ingenic jz4740 chip");
MODULE_LICENSE("GPL");
