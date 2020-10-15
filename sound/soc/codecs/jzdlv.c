/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "../jz4750/jz4750-pcm.h"
#include "jzdlv.h"

#define AUDIO_NAME "jzdlv"
#define JZDLV_VERSION "1.0"

/*
 * Debug
 */

#define JZDLV_DEBUG 0

#ifdef JZDLV_DEBUG
#define dbg(format, arg...) \
	printk(KERN_DEBUG AUDIO_NAME ": " format "\n" , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif
#define err(format, arg...) \
	printk(KERN_ERR AUDIO_NAME ": " format "\n" , ## arg)
#define info(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": " format "\n" , ## arg)
#define warn(format, arg...) \
	printk(KERN_WARNING AUDIO_NAME ": " format "\n" , ## arg)

/* codec private data */
struct jzdlv_data {
	unsigned int sysclk;
	struct snd_soc_codec codec;
};

static struct jzdlv_data jzdlv_data;
/*
 * jzdlv register cache
 */
static u16 jzdlv_reg[JZDLV_CACHEREGNUM];

int read_codec_file(int addr)
{
	while (__icdc_rgwr_ready());
	__icdc_set_addr(addr);
	mdelay(1);
	return(__icdc_get_value());
}

static void printk_codec_files(void)
{
	int cnt, val;

	//printk("\n");
#if 0	
	printk("REG_CPM_I2SCDR=0x%08x\n",REG_CPM_I2SCDR);
	printk("REG_CPM_CLKGR=0x%08x\n",REG_CPM_CLKGR);
	printk("REG_CPM_CPCCR=0x%08x\n",REG_CPM_CPCCR);
	printk("REG_AIC_FR=0x%08x\n",REG_AIC_FR);
	printk("REG_AIC_CR=0x%08x\n",REG_AIC_CR);		
	printk("REG_AIC_I2SCR=0x%08x\n",REG_AIC_I2SCR);
	printk("REG_AIC_SR=0x%08x\n",REG_AIC_SR);
	printk("REG_ICDC_RGDATA=0x%08x\n",REG_ICDC_RGDATA);
#endif
	for (cnt = 0; cnt < JZDLV_CACHEREGNUM ; cnt++) {
		val = read_codec_file(cnt);
		jzdlv_reg[cnt] = val;
		//printk(" ( %d  :  0x%x ) ",cnt ,jzdlv_reg[cnt]);
	}
	//printk("\n");
}

void write_codec_file(int addr, int val)
{
	while (__icdc_rgwr_ready());
	__icdc_set_addr(addr);
	__icdc_set_cmd(val); /* write */
	mdelay(1);
	__icdc_set_rgwr();
	mdelay(1);
	//jzdlv_reg[addr] = val;
}
EXPORT_SYMBOL(write_codec_file);

int write_codec_file_bit(int addr, int bitval, int mask_bit)
{
	int val;
	while (__icdc_rgwr_ready());
	__icdc_set_addr(addr);
	mdelay(1);
	val = __icdc_get_value(); /* read */

	val &= ~(1 << mask_bit);
	if (bitval == 1)
		val |= 1 << mask_bit;
#if 0
	while (__icdc_rgwr_ready());
	__icdc_set_addr(addr);
	__icdc_set_cmd(val); /* write */
	mdelay(1);
	__icdc_set_rgwr();
	mdelay(1);
#else
	write_codec_file(addr, val);
#endif
	while (__icdc_rgwr_ready());
	__icdc_set_addr(addr);
	val = __icdc_get_value(); /* read */	
	
	if (((val >> mask_bit) & bitval) == bitval)
		return 1;
	else 
		return 0;
}
EXPORT_SYMBOL(write_codec_file_bit);

/*
 * read jzdlv register cache
 */
static inline unsigned int jzdlv_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;

	if (reg >= JZDLV_CACHEREGNUM)
		return -1;
	return cache[reg];
}

static inline unsigned int jzdlv_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u8 data;
	data = reg;
	if (codec->hw_write(codec->control_data, &data, 1) != 1)
		return -EIO;

	if (codec->hw_read(codec->control_data, &data, 1) != 1)
		return -EIO;

	return data;
}

/*
 * write jzdlv register cache
 */
static inline void jzdlv_write_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg, u16 value)
{
	u16 *cache = codec->reg_cache;

	if (reg >= JZDLV_CACHEREGNUM) {
		return;
	}

	cache[reg] = value;

}

/*
 * write to the jzdlv register space
 */
static int jzdlv_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	jzdlv_write_reg_cache(codec, reg, value);
	if(codec->hw_write)
		codec->hw_write(&value, NULL, reg);	
	return 0;
}

/* set Audio data replay */
void set_audio_data_replay(void)
{
	write_codec_file(9, 0xff);
	write_codec_file(8, 0x20);// only CCMC
	mdelay(10);

	/* DAC path */
	write_codec_file_bit(1, 0, 4);//CR1.HP_DIS->0
	write_codec_file_bit(5, 1, 3);//PMR1.SB_LIN->1
	write_codec_file_bit(5, 1, 0);//PMR1.SB_IND->1
	
	write_codec_file_bit(1, 0, 2);//CR1.BYPASS->0
	write_codec_file_bit(1, 1, 3);//CR1.DACSEL->1
	
	write_codec_file_bit(5, 0, 5);//PMR1.SB_MIX->0
	mdelay(100);
	write_codec_file_bit(5, 0, 6);//PMR1.SB_OUT->0
	write_codec_file_bit(5, 0, 7);//PMR1.SB_DAC->0
	write_codec_file_bit(1, 1, 7);//CR1.SB_MICBIAS->1
	mdelay(100);
	write_codec_file_bit(1, 0, 5);//DAC_MUTE->0
}

/* unset Audio data replay */
void unset_audio_data_replay(void)
{
	write_codec_file_bit(1, 1, 5);//DAC_MUTE->1
	mdelay(200);
	write_codec_file_bit(5, 1, 6);//SB_OUT->1
	write_codec_file_bit(5, 1, 7);//SB_DAC->1
	write_codec_file_bit(5, 1, 4);//SB_MIX->1
	write_codec_file_bit(6, 1, 0);//SB_SLEEP->1
	write_codec_file_bit(6, 1, 1);//SB->1

	write_codec_file(9, 0xff);
	write_codec_file(8, 0x3f);
}

/* set Record MIC input audio without playback */
static void set_record_mic_input_audio_without_playback(void)
{
	/* ADC path for MIC IN */
	write_codec_file_bit(1, 1, 2);
	write_codec_file_bit(1, 0, 7);//CR1.SB_MICBIAS->0
	//write_codec_file_bit(1, 1, 6);//CR1.MONO->1
	
	write_codec_file(22, 0x40);//mic 1
	write_codec_file_bit(23, 0, 7);//AGC1.AGC_EN->0
	write_codec_file_bit(3, 1, 7);//CR1.HP_DIS->1
	write_codec_file_bit(5, 1, 3);//PMR1.SB_LIN->1
	write_codec_file_bit(5, 1, 0);//PMR1.SB_IND->1
	
	write_codec_file_bit(1, 0, 2);//CR1.BYPASS->0
	write_codec_file_bit(1, 0, 3);//CR1.DACSEL->0
	write_codec_file_bit(6, 1, 3);// gain set
	
	write_codec_file_bit(5, 0, 5);//PMR1.SB_MIX->0
	mdelay(100);
	write_codec_file_bit(5, 0, 6);//PMR1.SB_OUT->0
	write_codec_file(1, 0x4);
}

/* unset Record MIC input audio without playback */
static void unset_record_mic_input_audio_without_playback(void)
{
	/* ADC path for MIC IN */
	write_codec_file_bit(5, 1, 4);//SB_ADC->1
	write_codec_file_bit(1, 1, 7);//CR1.SB_MICBIAS->1
	write_codec_file(22, 0xc0);//CR3.SB_MIC1
}

#if 0
static irqreturn_t aic_codec_irq(int irq, void *dev_id)
{
	u8 file_9 = read_codec_file(9);
	u8 file_8 = read_codec_file(8);
	
	//printk("--- 8:0x%x  9:0x%x ---\n",file_8,file_9);
	if ((file_9 & 0x1f) == 0x10) {
		// have hp short circuit
		write_codec_file(8, 0x3f);//mask all interrupt
		write_codec_file_bit(5, 1, 6);//SB_OUT->1
		mdelay(300);
		while ((read_codec_file(9) & 0x4) != 0x4);
		while ((read_codec_file(9) & 0x10) == 0x10) {
			write_codec_file(9, 0x10);
		}
		write_codec_file_bit(5, 0, 6);//SB_OUT->0
		mdelay(300);
		while ((read_codec_file(9) & 0x8) != 0x8);
		write_codec_file(9, file_9);
		write_codec_file(8, file_8);

		return IRQ_HANDLED;
	}

	if (file_9 & 0x8)
		ramp_up_end = jiffies;
	else if (file_9 & 0x4)
		ramp_down_end = jiffies;
	else if (file_9 & 0x2)
		gain_up_end = jiffies;
	else if (file_9 & 0x1)
		gain_down_end = jiffies;

	write_codec_file(9, file_9);
	if (file_9 & 0xf)
		wake_up(&pop_wait_queue);
	while (REG_ICDC_RGDATA & 0x100);
	
	return IRQ_HANDLED;
}
#else
static irqreturn_t aic_codec_irq(int irq, void *dev_id)
{
	u8 file_9 = read_codec_file(9);
	u8 file_8 = read_codec_file(8);
	
	//printk("--- 1  8:0x%x  9:0x%x ---\n",file_8,file_9);
	if ((file_9 & 0x1f) == 0x10) {
		write_codec_file(8, 0x3f);
		write_codec_file_bit(5, 1, 6);//SB_OUT->1
		mdelay(300);
		while ((read_codec_file(9) & 0x4) != 0x4);
		while ((read_codec_file(9) & 0x10) == 0x10) {
			write_codec_file(9, 0x10);
		}
		write_codec_file_bit(5, 0, 6);//SB_OUT->0
		mdelay(300);
		while ((read_codec_file(9) & 0x8) != 0x8);
		write_codec_file(9, file_9);
		write_codec_file(8, file_8);
	    
		return IRQ_HANDLED;
	}
	/*if (file_9 & 0x8)
		ramp_up_end = jiffies;
	else if (file_9 & 0x4)
		ramp_down_end = jiffies;
	else if (file_9 & 0x2)
		gain_up_end = jiffies;
	else if (file_9 & 0x1)
	gain_down_end = jiffies;*/

	write_codec_file(9, file_9);
	/*if (file_9 & 0xf)
	  wake_up(&pop_wait_queue);*/
	while (REG_ICDC_RGDATA & 0x100);
	
	return IRQ_HANDLED;
}
#endif

static int jzdlv_reset(struct snd_soc_codec *codec)
{
	/* reset DLV codec. from hibernate mode to sleep mode */
	write_codec_file(0, 0xf);
	write_codec_file_bit(6, 0, 0);
	write_codec_file_bit(6, 0, 1);
	mdelay(200);
	//write_codec_file(0, 0xf);
	write_codec_file_bit(5, 0, 7);//PMR1.SB_DAC->0
	write_codec_file_bit(5, 0, 4);//PMR1.SB_ADC->0
	mdelay(10);//wait for stability

	return 0;
}


#if 0
static int jzdlv_sync(struct snd_soc_codec *codec)
{
	u16 *cache = codec->reg_cache;
	int i, r = 0;

	for (i = 0; i < JZDLV_CACHEREGNUM; i++)
		r |= jzdlv_write(codec, i, cache[i]);

	return r;
};
#endif

static const struct snd_kcontrol_new jzdlv_snd_controls[] = {

	//SOC_DOUBLE_R("Master Playback Volume", 1, 1, 0, 3, 0),
	SOC_DOUBLE_R("Master Playback Volume", DLV_CGR8, DLV_CGR9, 0, 31, 0),
	//SOC_DOUBLE_R("MICBG", ICODEC_2_LOW, ICODEC_2_LOW, 4, 3, 0),
	//SOC_DOUBLE_R("Line", 2, 2, 0, 31, 0),
	SOC_DOUBLE_R("Line", DLV_CGR10, DLV_CGR10, 0, 31, 0),
};

/* add non dapm controls */
static int jzdlv_add_controls(struct snd_soc_codec *codec)
{
	return snd_soc_add_controls(codec, jzdlv_snd_controls, ARRAY_SIZE(jzdlv_snd_controls));
}

static const struct snd_soc_dapm_widget jzdlv_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("LHPOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),
	SND_SOC_DAPM_OUTPUT("RHPOUT"),
	SND_SOC_DAPM_INPUT("MICIN"),
	SND_SOC_DAPM_INPUT("RLINEIN"),
	SND_SOC_DAPM_INPUT("LLINEIN"),
};

static const struct snd_soc_dapm_route intercon_routes [] = {
	/* output mixer */
	{"Output Mixer", "Line Bypass Switch", "Line Input"},
	{"Output Mixer", "HiFi Playback Switch", "DAC"},
	{"Output Mixer", "Mic Sidetone Switch", "Mic Bias"},

	/* outputs */
	{"RHPOUT", NULL, "Output Mixer"},
	{"ROUT", NULL, "Output Mixer"},
	{"LHPOUT", NULL, "Output Mixer"},
	{"LOUT", NULL, "Output Mixer"},

	/* input mux */
	{"Input Mux", "Line In", "Line Input"},
	{"Input Mux", "Mic", "Mic Bias"},
	{"ADC", NULL, "Input Mux"},

	/* inputs */
	{"Line Input", NULL, "LLINEIN"},
	{"Line Input", NULL, "RLINEIN"},
	{"Mic Bias", NULL, "MICIN"},
};

static void init_codec(void)
{
	/* reset DLV codec. from hibernate mode to sleep mode */
	write_codec_file(0, 0xf);
	write_codec_file_bit(6, 0, 0);
	write_codec_file_bit(6, 0, 1);
	mdelay(200);
	//write_codec_file(0, 0xf);
	write_codec_file_bit(5, 0, 7);//PMR1.SB_DAC->0
	write_codec_file_bit(5, 0, 4);//PMR1.SB_ADC->0
	mdelay(10);//wait for stability
}

static int jzdlv_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, jzdlv_dapm_widgets, ARRAY_SIZE(jzdlv_dapm_widgets));
	/* set up audio path interconnects */
	snd_soc_dapm_add_routes(codec, intercon_routes, ARRAY_SIZE(intercon_routes));
	snd_soc_dapm_new_widgets(codec);
	return 0;
}

static int jzdlv_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	int speed = 0;
	int val = 0;
	
	/* sample channel */
	switch (params_channels(params)) {
	case 1:
		write_codec_file_bit(1, 1, 6);//CR1.MONO->1 for Mono
		break;
	case 2:
		write_codec_file_bit(1, 0, 6);//CR1.MONO->0 for Stereo
		break;
	}
	/* sample rate */
	switch (params_rate(params)) {
	case 8000:
		speed = 10;
		break;
	case 9600:
		speed = 9;
		break;
	case 11025:
		speed = 8;
		break;
	case 12000:
		speed = 7;
		break;
	case 16000:
		speed = 6;
		break;
	case 22050:
		speed = 5;
		break;
	case 24000:
		speed = 4;
		break;
	case 32000:
		speed = 3;
		break;
	case 44100:
		speed = 2;
		break;
	case 48000:
		speed = 1;
		break;
	case 96000:
		speed = 0;
		break;
	default:
		printk(" invalid rate :0x%08x\n",params_rate(params));
	}

	val = (speed << 4) | speed;
	jzdlv_write(codec, DLV_CCR2, val);
    
	return 0;
}

static int jzdlv_pcm_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		break;
	case SNDRV_PCM_TRIGGER_START:
		//case SNDRV_PCM_TRIGGER_RESUME:
		
		init_codec();
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			set_audio_data_replay();
			write_codec_file_bit(5, 0, 7);//PMR1.SB_DAC->0
			mdelay(300);
			REG_AIC_I2SCR = 0x10;
			mdelay(20);
			__aic_flush_fifo();
		} else {
			set_record_mic_input_audio_without_playback();
			mdelay(10);
			REG_AIC_I2SCR = 0x10;
			mdelay(20);
			__aic_flush_fifo();
			write_codec_file_bit(5, 1, 7);
		}
		
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		//case SNDRV_PCM_TRIGGER_SUSPEND:

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			unset_audio_data_replay();
		} else {
			unset_record_mic_input_audio_without_playback();
		}
		break;

	default:
		ret = -EINVAL;
	}
	
	return ret;
}

static int jzdlv_pcm_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	/*struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec; */

	return 0;
}

static void jzdlv_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;

	/* deactivate */
	if (!codec->active) {
		udelay(50);
	}
}

static int jzdlv_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 reg_val = jzdlv_read_reg_cache(codec, 2/*DLV_1_LOW*/);

	if (mute != 0) 
		mute = 1;
	if (mute)
		reg_val = reg_val | (0x1 << 14);
	else
		reg_val = reg_val & ~(0x1 << 14);
	
	//jzdlv_write(codec, DLV_1_LOW, reg_val);
	return 0;
}

static int jzdlv_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct jzdlv_data *jzdlv = codec->private_data;

	jzdlv->sysclk = freq;
	return 0;
}
/*
 * Set's ADC and Voice DAC format. called by apus_hw_params() in apus.c
 */
static int jzdlv_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */

	/* set master/slave audio interface. codec side */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
                /* set master mode for codec */
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		/* set slave mode for codec */
		break;
	default:
		return -EINVAL;
	}

	/* interface format . set some parameter for codec side */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S: 
		/* set I2S mode for codec */
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		/* set right J mode */
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		/* set left J mode */
		break;
	case SND_SOC_DAIFMT_DSP_A:
		/* set dsp A mode */
		break;
	case SND_SOC_DAIFMT_DSP_B:
		/* set dsp B mode */
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion. codec side */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:		
		break;
	case SND_SOC_DAIFMT_IB_NF:
		break;
	case SND_SOC_DAIFMT_NB_IF:
		break;
	default:
		return -EINVAL;
		}

	/* jzcodec_write(codec, 0, val); */
	return 0;
}

static int jzdlv_dapm_event(struct snd_soc_codec *codec, int event)
{
/*	u16 reg_val; */

	switch (event) {
	case SNDRV_CTL_POWER_D0: /* full On */
		/* vref/mid, osc on, dac unmute */
		/* u16 reg_val = jzcodec_read_reg_cache(codec, ICODEC_1_LOW); */
		/* jzcodec_write(codec, 0, val); */
		break;
	case SNDRV_CTL_POWER_D1: /* partial On */
	case SNDRV_CTL_POWER_D2: /* partial On */
		break;
	case SNDRV_CTL_POWER_D3hot: /* Off, with power */
		/* everything off except vref/vmid, */
		/*reg_val = 0x0800;
		jzcodec_write_reg_cache(codec, ICODEC_1_LOW, reg_val);
		reg_val = 0x0017;
		jzcodec_write_reg_cache(codec, ICODEC_1_HIGH, reg_val);
		REG_ICDC_CDCCR1 = jzcodec_reg[0];
		mdelay(2);
		reg_val = 0x2102;
		jzcodec_write_reg_cache(codec, ICODEC_1_LOW, reg_val);
		reg_val = 0x001f;
		jzcodec_write_reg_cache(codec, ICODEC_1_HIGH, reg_val);
		REG_ICDC_CDCCR1 = jzcodec_reg[0];
		mdelay(2);
		reg_val = 0x3302;
		jzcodec_write_reg_cache(codec, ICODEC_1_LOW, reg_val);
		reg_val = 0x0003;
		jzcodec_write_reg_cache(codec, ICODEC_1_HIGH, reg_val);
		REG_ICDC_CDCCR1 = jzcodec_reg[0];*/
		break;
	case SNDRV_CTL_POWER_D3cold: /* Off, without power */
		/* everything off, dac mute, inactive */
		/*reg_val = 0x2302;
		jzcodec_write(codec, ICODEC_1_LOW, reg_val);
		reg_val = 0x001b;
		jzcodec_write(codec, ICODEC_1_HIGH, reg_val);
		mdelay(1);
		reg_val = 0x2102;
		jzcodec_write(codec, ICODEC_1_LOW, reg_val);
		reg_val = 0x001b;
		jzcodec_write(codec, ICODEC_1_HIGH, reg_val);*/
		break;
	}
	//codec->dapm_state = event;
	return 0;
}

#define JZDLV_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
		SNDRV_PCM_RATE_48000)

#define JZDLV_FORMATS (SNDRV_PCM_FORMAT_S8 | SNDRV_PCM_FMTBIT_S16_LE)

static struct snd_soc_dai_ops jzdlv_dai_ops = {
	.trigger = jzdlv_pcm_trigger,
	.prepare = jzdlv_pcm_prepare,
	.hw_params = jzdlv_hw_params,
	.shutdown = jzdlv_shutdown,
	.digital_mute = jzdlv_mute,
	.set_sysclk = jzdlv_set_dai_sysclk,
	.set_fmt = jzdlv_set_dai_fmt,
};

struct snd_soc_dai jzdlv_dai = {
	.name = "JZDLV",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = JZDLV_RATES,
		.formats = JZDLV_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = JZDLV_RATES,
		.formats = JZDLV_FORMATS,
	},
	.ops = &jzdlv_dai_ops,
};
EXPORT_SYMBOL_GPL(jzdlv_dai);

#ifdef CONFIG_PM
//static u16 jzdlv_reg_pm[JZDLV_CACHEREGNUM];
static int jzdlv_suspend(struct platform_device *pdev, pm_message_t state)
{
//	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	//struct snd_soc_codec *codec = socdev->card->codec;
#if 0
	jzcodec_reg_pm[ICODEC_1_LOW] = jzcodec_read_reg_cache(codec, ICODEC_1_LOW);
	jzcodec_reg_pm[ICODEC_1_HIGH] = jzcodec_read_reg_cache(codec, ICODEC_1_HIGH);
	jzcodec_reg_pm[ICODEC_2_LOW] = jzcodec_read_reg_cache(codec, ICODEC_2_LOW);
	jzcodec_reg_pm[ICODEC_2_HIGH] = jzcodec_read_reg_cache(codec, ICODEC_2_HIGH);

	jzcodec_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
#endif
	return 0;
}

static int jzdlv_resume(struct platform_device *pdev)
{
//	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	//struct snd_soc_codec *codec = socdev->card->codec;
	//u16 reg_val;
#if 0
	jzcodec_dapm_event(codec, SNDRV_CTL_POWER_D3hot);
	reg_val = jzcodec_reg_pm[ICODEC_1_LOW];
	jzcodec_write(codec, ICODEC_1_LOW, reg_val);
	reg_val = jzcodec_reg_pm[ICODEC_1_HIGH];
	jzcodec_write(codec, ICODEC_1_HIGH, reg_val);
	reg_val = jzcodec_reg_pm[ICODEC_2_LOW];
	jzcodec_write(codec, ICODEC_2_LOW, reg_val);
	reg_val = jzcodec_reg_pm[ICODEC_2_HIGH];
	jzcodec_write(codec, ICODEC_2_HIGH, reg_val);

	jzcodec_dapm_event(codec, codec->suspend_dapm_state);
#endif
	return 0;
}
#else
#define jzdlv_suspend	NULL
#define jzdlv_resume	NULL
#endif
/*
 * initialise the JZDLV driver
 * register the mixer and dsp interfaces with the kernel
 */
static struct snd_soc_device *jzdlv_socdev;

static int write_codec_reg(u16 * add, char * name, int reg)
{
	write_codec_file(reg, *add);
       
	return 0;
}

static int jzdlv_soc_codec_setup(struct snd_soc_codec *codec)
{
	int ret;

	/* Add other interfaces here ,no I2C connection */
	codec->hw_write = (hw_write_t)write_codec_reg;
	//codec->hw_read = (hw_read_t)read_codec_reg;

	/*REG_CPM_CPCCR &= ~(1 << 31);
	  REG_CPM_CPCCR &= ~(1 << 30);*/
	write_codec_file(0, 0xf);

	REG_AIC_I2SCR = 0x10;
	__i2s_internal_codec();
	__i2s_as_slave();
	__i2s_select_i2s();
	__aic_select_i2s();
	__aic_reset();
	mdelay(10);
	REG_AIC_I2SCR = 0x10;
	mdelay(20);

	/* power on DLV */
	write_codec_file(8, 0x3f);
	write_codec_file(9, 0xff);
	mdelay(10);
	
	/* Stage 1 Start. */
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	codec->name = "JZDLV";
	codec->owner = THIS_MODULE;
	codec->read = jzdlv_read_reg_cache;
	codec->write = jzdlv_write;
	//codec->dapm_event = jzdlv_dapm_event;
	codec->dai = &jzdlv_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = sizeof(jzdlv_reg);
	codec->reg_cache = kmemdup(jzdlv_reg, sizeof(jzdlv_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL) {
		printk(KERN_ERR "jzdlv: no enough memory for kmemdup.\n");
		return -ENOMEM;
	}

	jzdlv_reset(codec);

	ret = snd_soc_register_codec(codec);
	if (ret < 0) {
		printk(KERN_ERR "jzdlv: failed to register codec.\n");
		kfree(codec->reg_cache);
		return ret;
	}
	
	return 0;
}

static int jzdlv_soc_dev_setup(struct snd_soc_device *socdev, struct snd_soc_codec *codec)
{
	int ret;

	socdev->card->codec = codec;
	mutex_init(&codec->mutex);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "jzdlv: failed to create pcms\n");
		goto err;
	}
	
	/* power on device */
	jzdlv_dapm_event(codec, SNDRV_CTL_POWER_D3hot);

	ret = jzdlv_add_controls(codec);
	if (ret)
		goto err;

	ret = jzdlv_add_widgets(codec);
	if (ret)
		goto err;

	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "jzcodec: failed to register card\n");
		goto err;
	}

	mdelay(10);
	REG_AIC_I2SCR = 0x10;
	mdelay(20);

	/* power on DLV */
	write_codec_file(9, 0xff);
	write_codec_file(8, 0x3f);

	ret = request_irq(IRQ_AIC, aic_codec_irq, IRQF_DISABLED, "aic_codec_irq", NULL);
	if (ret) {
		printk("Could not get aic codec irq %d\n", IRQ_AIC);
		goto err;
	}

	printk_codec_files();

	return ret;

err:	
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return ret;
}

static int jzdlv_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = &jzdlv_data.codec;

	jzdlv_socdev = socdev;

	return jzdlv_soc_dev_setup(socdev, codec);
}

/* power down chip */
static int jzdlv_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = &jzdlv_data.codec;

	if (codec->control_data)
		jzdlv_dapm_event(codec, SNDRV_CTL_POWER_D3cold);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	kfree(codec->private_data);
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_jzdlv = {
	.probe = 	jzdlv_probe,
	.remove = 	jzdlv_remove,
	.suspend = 	jzdlv_suspend,
	.resume =	jzdlv_resume,
};

EXPORT_SYMBOL(soc_codec_dev_jzdlv);

static int __init jzdlv_init(void)
{
	struct snd_soc_codec *codec = &jzdlv_data.codec;
	
	int rv;
	
	codec->private_data = &jzdlv_data;
	rv = jzdlv_soc_codec_setup(codec);
	if (rv)
		return rv;
	
	return snd_soc_register_dai(&jzdlv_dai);
}
module_init(jzdlv_init);

static void __exit jzdlv_exit(void)
{
	snd_soc_unregister_dai(&jzdlv_dai);
}
module_exit(jzdlv_exit);

MODULE_DESCRIPTION("ASoC JZDLV driver");
MODULE_AUTHOR("Richard");
MODULE_LICENSE("GPL");
