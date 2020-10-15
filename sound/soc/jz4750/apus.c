/*
 * apus.c  --  SoC audio for APUS
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/jzsoc.h>

#include "../codecs/jzdlv.h"
#include "jz4750-pcm.h"
#include "jz4750-i2s.h"

#define APUS_HP        0
#define APUS_MIC       1
#define APUS_LINE      2
#define APUS_HEADSET   3
#define APUS_HP_OFF    4
#define APUS_SPK_ON    0
#define APUS_SPK_OFF   1

static int apus_jack_func;
static int apus_spk_func;

unsigned short set_scoop_gpio(struct device *dev, unsigned short bit)
{
	unsigned short gpio_bit = 0;

	return gpio_bit;
}

unsigned short reset_scoop_gpio(struct device *dev, unsigned short bit)
{
	unsigned short gpio_bit = 0;

	return gpio_bit;
}

static void apus_ext_control(struct snd_soc_codec *codec)
{
	int spk = 0, mic = 0, line = 0, hp = 0, hs = 0;

	/* set up jack connection */
	switch (apus_jack_func) {
	case APUS_HP:
		hp = 1;
		/* set = unmute headphone */
		//set_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_L);
		//set_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_R);
		break;
	case APUS_MIC:
		mic = 1;
		/* reset = mute headphone */
		//reset_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_L);
		//reset_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_R);
		break;
	case APUS_LINE:
		line = 1;
		//reset_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_L);
		//reset_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_R);
		break;
	case APUS_HEADSET:
		hs = 1;
		mic = 1;
		//reset_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_L);
		//set_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_R);
		break;
	}

	if (apus_spk_func == APUS_SPK_ON)
		spk = 1;

	/* set the enpoints to their new connetion states */
	if (spk)
		snd_soc_dapm_enable_pin(codec, "Ext Spk");
	else 
		snd_soc_dapm_disable_pin(codec, "Ext Spk");

	if (mic) 
		snd_soc_dapm_enable_pin(codec, "Mic Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Mic Jack");
	
	if (line)
		snd_soc_dapm_enable_pin(codec, "Line Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Line Jack");
	
	if (hp) 
		snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Headphone Jack");
	
	if (hp)
		snd_soc_dapm_enable_pin(codec, "Headset Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Headset Jack");

	/* signal a DAPM event */
	snd_soc_dapm_sync(codec);
}

static int apus_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->card->codec;

	/* check the jack status at stream startup */
	apus_ext_control(codec);
	return 0;
}

/* we need to unmute the HP at shutdown as the mute burns power on apus */
static void apus_shutdown(struct snd_pcm_substream *substream)
{
	/*struct snd_soc_pcm_runtime *rtd = substream->private_data;
	  struct snd_soc_codec *codec = rtd->socdev->codec;*/

	return;
}

static int apus_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;

	/* set codec DAI configuration */
	ret = codec_dai->ops->set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = cpu_dai->ops->set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = codec_dai->ops->set_sysclk(codec_dai, JZDLV_SYSCLK, 111,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set the I2S system clock as input (unused) */
	ret = cpu_dai->ops->set_sysclk(cpu_dai, JZ4750_I2S_SYSCLK, 0,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops apus_ops = {
	.startup = apus_startup,
	.hw_params = apus_hw_params,
	.shutdown = apus_shutdown,
};

static int apus_get_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = apus_jack_func;
	return 0;
}

static int apus_set_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (apus_jack_func == ucontrol->value.integer.value[0])
		return 0;

	apus_jack_func = ucontrol->value.integer.value[0];
	apus_ext_control(codec);
	return 1;
}

static int apus_get_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = apus_spk_func;
	return 0;
}

static int apus_set_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (apus_spk_func == ucontrol->value.integer.value[0])
		return 0;

	apus_spk_func = ucontrol->value.integer.value[0];
	apus_ext_control(codec);
	return 1;
}

static int apus_amp_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event))
		//set_scoop_gpio(&corgiscoop_device.dev, APUS_SCP_APM_ON);
		;
	else
		//reset_scoop_gpio(&corgiscoop_device.dev, APUS_SCP_APM_ON);
		;

	return 0;
}

static int apus_mic_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event))
		//set_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MIC_BIAS);
		;
	else
		//reset_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MIC_BIAS);
		;
		
	return 0;
}

/* apus machine dapm widgets */
static const struct snd_soc_dapm_widget jzdlv_dapm_widgets[] = {
SND_SOC_DAPM_HP("Headphone Jack", NULL),
SND_SOC_DAPM_MIC("Mic Jack",apus_mic_event),
SND_SOC_DAPM_SPK("Ext Spk", apus_amp_event),
SND_SOC_DAPM_LINE("Line Jack", NULL),
SND_SOC_DAPM_HP("Headset Jack", NULL),
};

/* apus machine audio map (connections to the codec pins) */
static const struct snd_soc_dapm_route audio_map [] = {
	/* headset Jack  - in = micin, out = LHPOUT*/
	{"Headset Jack", NULL, "LHPOUT"},

	/* headphone connected to LHPOUT1, RHPOUT1 */
	{"Headphone Jack", NULL, "LHPOUT"},
	{"Headphone Jack", NULL, "RHPOUT"},

	/* speaker connected to LOUT, ROUT */
	{"Ext Spk", NULL, "ROUT"},
	{"Ext Spk", NULL, "LOUT"},

	/* mic is connected to MICIN (via right channel of headphone jack) */
	{"MICIN", NULL, "Mic Jack"},

	/* Same as the above but no mic bias for line signals */
	{"MICIN", NULL, "Line Jack"},
};

static const char *jack_function[] = {"Headphone", "Mic", "Line", "Headset",
	"Off"};
static const char *spk_function[] = {"On", "Off"};
static const struct soc_enum apus_enum[] = {
	SOC_ENUM_SINGLE_EXT(5, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
};

static const struct snd_kcontrol_new jzdlv_apus_controls[] = {
	SOC_ENUM_EXT("Jack Function", apus_enum[0], apus_get_jack,
		apus_set_jack),
	SOC_ENUM_EXT("Speaker Function", apus_enum[1], apus_get_spk,
		apus_set_spk),
};

/*
 * Apus for a jzdlv as connected on jz4750 Device
 */
static int apus_jzdlv_init(struct snd_soc_codec *codec)
{
	int err;

	snd_soc_dapm_nc_pin(codec, "LLINEIN");
	snd_soc_dapm_nc_pin(codec, "RLINEIN");
	
	/* Add apus specific controls */
	err = snd_soc_add_controls(codec, jzdlv_apus_controls, ARRAY_SIZE(jzdlv_apus_controls));
	if (err < 0)
		return err;

	/* Add apus specific widgets */
	snd_soc_dapm_new_controls(codec, jzdlv_dapm_widgets, ARRAY_SIZE(jzdlv_dapm_widgets));
	
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);
	return 0;
}

/* apus digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link apus_dai = {
	.name = "JZDLV",
	.stream_name = "JZDLV",
	.cpu_dai = &jz4750_i2s_dai,
	.codec_dai = &jzdlv_dai,
	.init = apus_jzdlv_init,
	.ops = &apus_ops,
};

/* apus audio machine driver */
static struct snd_soc_card snd_soc_card_apus = {
	.name = "Apus",
	.dai_link = &apus_dai,
	.platform = &jz4750_soc_platform,
	.num_links = 1,
};

/* apus audio subsystem */
static struct snd_soc_device apus_snd_devdata = {
	.card = &snd_soc_card_apus,
	.codec_dev = &soc_codec_dev_jzdlv,
	//.codec_data
};

static struct platform_device *apus_snd_device;

static int __init apus_init(void)
{
	int ret;

	apus_snd_device = platform_device_alloc("soc-audio", -1);

	if (!apus_snd_device)
		return -ENOMEM;

	platform_set_drvdata(apus_snd_device, &apus_snd_devdata);
	apus_snd_devdata.dev = &apus_snd_device->dev;
	ret = platform_device_add(apus_snd_device);

	if (ret)
		platform_device_put(apus_snd_device);

	return ret;
}

static void __exit apus_exit(void)
{
	platform_device_unregister(apus_snd_device);
}

module_init(apus_init);
module_exit(apus_exit);

/* Module information */
MODULE_AUTHOR("Richard");
MODULE_DESCRIPTION("ALSA SoC Apus");
MODULE_LICENSE("GPL");
