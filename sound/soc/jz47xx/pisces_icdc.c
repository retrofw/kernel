/*
 * pisces_icdc.c  --  SoC audio for PISCES with internal codec
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

#include "../codecs/jz4770_icdc.h"
#include "jz47xx-pcm.h"
#include "jz47xx-i2s.h"

#if 0
/* currently we have nothing to do when startup & powerdown */
static int pisces_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->card->codec;

	return 0;
}

static void pisces_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->card->codec;

	return;
}
#endif

static int pisces_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;

	/* set codec DAI configuration */
#if 0
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;
#endif

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, JZ47XX_I2S_SYSCLK,
				     SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;
	/* we do not have to set clkdiv for internal codec */

#if 0
	/* set codec DAI configuration */
#if 0
	ret = codec_dai->ops->set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;
#endif

	/* set cpu DAI configuration */
	ret = cpu_dai->ops->set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = codec_dai->ops->set_sysclk(codec_dai, 0, 111,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set the I2S system clock as input (unused) */
	ret = cpu_dai->ops->set_sysclk(cpu_dai, JZ47XX_I2S_SYSCLK, 0,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;
#endif

	return 0;
}

static struct snd_soc_ops pisces_ops = {
	//.startup = pisces_startup,
	.hw_params = pisces_hw_params,
	//.shutdown = pisces_shutdown,
};

/* define the scenarios */
#define PISCES_MODE_HP_BIT		BIT0
#define PISCES_MODE_LINEOUT_BIT		BIT1
#define PISCES_MODE_MIC1_BIT		BIT2
#define PISCES_MODE_MIC2_BIT		BIT3


#define PISCES_AUDIO_OFF		0
#define PISCES_STEREO_TO_HEADPONES	1
#define PISCES_STEREO_TO_LINEOUT	2
#define PISCES_STEREO_TO_HP_LINEOUT	3
#define PISCES_CAPTURE_MIC1		4
#define PISCES_CAPTURE_MIC2		5
#define PISCES_CAPTURE_MIC1_MIC2	6

#define PISCES_MODE_HP_MIC1		7
#define PISCES_MODE_HP_MIC2		8
#define PISCES_MODE_HP_MIC1_MIC2	9

#define PISCES_MODE_LO_MIC1		10
#define PISCES_MODE_LO_MIC2		11
#define PISCES_MODE_LO_MIC1_MIC2	12


#define PISCES_MODE_HP_LO_MIC1		13
#define PISCES_MODE_HP_LO_MIC2		14
#define PISCES_MODE_HP_LO_MIC1_MIC2	15

static const char *pisces_scenarios[] = {
	"Off",
	"Headphone",
	"Lineout",
	"HP+Lineout",
	"Mic1",
	"Mic2",
	"Mic1+Mic2",

	"HP+Mic1",
	"HP+Mic2",
	"HP+MIC1+Mic2",

	"Lineout+Mic1",
	"Lineout+Mic2",
	"Lineout+Mic1+Mic2",

	"HP+LO+Mic1",
	"HP+LO+Mic2",
	"HP+LO+Mic1+Mic2",
};

static const struct soc_enum pisces_scenario_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(pisces_scenarios), pisces_scenarios),
};

static int pisces_scenario;

static int pisces_get_scenario(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = pisces_scenario;
	return 0;
}

#define LM4890_SHUTDOWN_PIN	GPA(17)

static int set_scenario_endpoints(struct snd_soc_codec *codec, int scenario)
{
	unsigned int mode_bits = 0;

	switch (pisces_scenario) {
	case PISCES_STEREO_TO_HEADPONES:
		mode_bits |= PISCES_MODE_HP_BIT;
		break;
	case PISCES_STEREO_TO_LINEOUT:
		mode_bits |= PISCES_MODE_LINEOUT_BIT;
		break;
	case PISCES_STEREO_TO_HP_LINEOUT:
		mode_bits |= PISCES_MODE_HP_BIT;
		mode_bits |= PISCES_MODE_LINEOUT_BIT;
		break;
	case PISCES_CAPTURE_MIC1:
		mode_bits |= PISCES_MODE_MIC1_BIT;
		break;
	case PISCES_CAPTURE_MIC2:
		mode_bits |= PISCES_MODE_MIC2_BIT;
		break;
	case PISCES_CAPTURE_MIC1_MIC2:
		mode_bits |= PISCES_MODE_MIC1_BIT;
		mode_bits |= PISCES_MODE_MIC2_BIT;
		break;

	case PISCES_MODE_HP_MIC1:
		mode_bits |= PISCES_MODE_MIC1_BIT;
		mode_bits |= PISCES_MODE_HP_BIT;
		break;
	case PISCES_MODE_HP_MIC2:
		mode_bits |= PISCES_MODE_HP_BIT;
		mode_bits |= PISCES_MODE_MIC2_BIT;
		break;
	case PISCES_MODE_HP_MIC1_MIC2:
		mode_bits |= PISCES_MODE_HP_BIT;
		mode_bits |= PISCES_MODE_MIC1_BIT;
		mode_bits |= PISCES_MODE_MIC2_BIT;
		break;

	case PISCES_MODE_LO_MIC1:
		mode_bits |= PISCES_MODE_LINEOUT_BIT;
		mode_bits |= PISCES_MODE_MIC1_BIT;
		break;
	case PISCES_MODE_LO_MIC2:
		mode_bits |= PISCES_MODE_LINEOUT_BIT;
		mode_bits |= PISCES_MODE_MIC2_BIT;
		break;
	case PISCES_MODE_LO_MIC1_MIC2:
		mode_bits |= PISCES_MODE_LINEOUT_BIT;
		mode_bits |= PISCES_MODE_MIC1_BIT;
		mode_bits |= PISCES_MODE_MIC2_BIT;
		break;


	case PISCES_MODE_HP_LO_MIC1:
		mode_bits |= PISCES_MODE_HP_BIT;
		mode_bits |= PISCES_MODE_LINEOUT_BIT;
		mode_bits |= PISCES_MODE_MIC1_BIT;
		break;
	case PISCES_MODE_HP_LO_MIC2:
		mode_bits |= PISCES_MODE_HP_BIT;
		mode_bits |= PISCES_MODE_LINEOUT_BIT;
		mode_bits |= PISCES_MODE_MIC2_BIT;
		break;
	case PISCES_MODE_HP_LO_MIC1_MIC2:
		mode_bits |= PISCES_MODE_HP_BIT;
		mode_bits |= PISCES_MODE_LINEOUT_BIT;
		mode_bits |= PISCES_MODE_MIC1_BIT;
		mode_bits |= PISCES_MODE_MIC2_BIT;
		break;
	default:
		;
	}

	if (mode_bits & PISCES_MODE_HP_BIT)
		snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Headphone Jack");

	if (mode_bits & PISCES_MODE_LINEOUT_BIT) {
		snd_soc_dapm_enable_pin(codec, "Lineout Jack");
		__gpio_as_output1(LM4890_SHUTDOWN_PIN);
	} else {
		snd_soc_dapm_disable_pin(codec, "Lineout Jack");
		__gpio_as_output0(LM4890_SHUTDOWN_PIN);
	}

	if (mode_bits & PISCES_MODE_MIC1_BIT)
		snd_soc_dapm_enable_pin(codec, "Mic1 Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Mic1 Jack");

	if (mode_bits & PISCES_MODE_MIC2_BIT)
		snd_soc_dapm_enable_pin(codec, "Mic2 Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Mic2 Jack");

	snd_soc_dapm_sync(codec);

	return 0;
}

static int pisces_set_scenario(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (pisces_scenario == ucontrol->value.integer.value[0])
		return 0;

	pisces_scenario = ucontrol->value.integer.value[0];
	set_scenario_endpoints(codec, pisces_scenario);
	return 1;
}

/* pisces machine dapm widgets */
static const struct snd_soc_dapm_widget jz_icdc_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic1 Jack", NULL),
	SND_SOC_DAPM_MIC("Mic2 Jack", NULL),
	SND_SOC_DAPM_LINE("Lineout Jack", NULL),
};

/* pisces machine audio map (connections to the codec pins) */
static const struct snd_soc_dapm_route audio_map [] = {
	/* Destination Widget(sink)  <=== Path Name <=== Source Widget */

	/* headphone connected to LHPOUT/RHPOUT */
	{"Headphone Jack", NULL, "LHPOUT"},
	{"Headphone Jack", NULL, "RHPOUT"},

	{ "Lineout Jack", NULL, "LOUT" },
	{ "Lineout Jack", NULL, "ROUT" },

	/* mic is connected to MICIN (via right channel of headphone jack) */
	{"MIC1P", NULL, "Mic Bias"},
	{"MIC1N", NULL, "Mic Bias"}, /* no such connection, but not harm */
	{"Mic Bias", NULL, "Mic1 Jack"},

	{"MIC2P", NULL, "Mic Bias"},
	{"MIC2N", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic2 Jack"},
};

/* If you have any amplifiers, you can add its controls here */
static const struct snd_kcontrol_new jz_icdc_pisces_controls[] = {
	SOC_ENUM_EXT("Pisces Mode", pisces_scenario_enum[0],
		     pisces_get_scenario, pisces_set_scenario),
};

/*
 * Pisces for a jz_icdc as connected on jz4770 Device
 */
static int pisces_jz_icdc_init(struct snd_soc_codec *codec)
{
	int err;

	/* set up codec pins not used */
	/* on Pisces board, linein pin are not connected */
	snd_soc_dapm_nc_pin(codec, "LLINEIN");
	snd_soc_dapm_nc_pin(codec, "RLINEIN");

	/* Add pisces specific controls */
	err = snd_soc_add_controls(codec, jz_icdc_pisces_controls,
				   ARRAY_SIZE(jz_icdc_pisces_controls));
	if (err < 0)
		return err;

	/* Add pisces specific widgets */
	snd_soc_dapm_new_controls(codec, jz_icdc_dapm_widgets, ARRAY_SIZE(jz_icdc_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);

	/* set endpoints to default mode */
	set_scenario_endpoints(codec, PISCES_AUDIO_OFF);

	return 0;
}

/* pisces digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link pisces_dai = {
	.name = "JZ_ICDC",
	.stream_name = "JZ_ICDC",
	.cpu_dai = &jz47xx_i2s_dai,
	.codec_dai = &jz_icdc_dai, /* defined in sound/soc/codecs/jz4770-icdc.c */
	.init = pisces_jz_icdc_init,
	.ops = &pisces_ops,
};

/* pisces audio machine driver */
static struct snd_soc_card snd_soc_card_pisces = {
	.name = "Pisces",      /* used by state file, use aplay -l or arecord -l will see this name */
	.dai_link = &pisces_dai,
	.platform = &jz47xx_soc_platform,/* defined in sound/soc/jz47xx/jz47xx-pcm.c */
	.num_links = 1,	      /* ARRAY_SIZE(pisces_dai), though pisces_dai is not an array *n* */
};

/* pisces audio subsystem */
static struct snd_soc_device pisces_snd_devdata = {
	.card = &snd_soc_card_pisces,
	.codec_dev = &jz_icdc_soc_codec_dev, /* defined in sound/soc/codecs/jz4770-icdc.c */
	//.codec_data = your private data here,
};

static struct platform_device *pisces_snd_device;

static int __init pisces_init(void)
{
	int ret;

	pisces_snd_device = platform_device_alloc("soc-audio", -1);

	if (!pisces_snd_device)
		return -ENOMEM;

	platform_set_drvdata(pisces_snd_device, &pisces_snd_devdata);
	pisces_snd_devdata.dev = &pisces_snd_device->dev;
	ret = platform_device_add(pisces_snd_device);

	if (ret)
		platform_device_put(pisces_snd_device);

	return ret;
}

static void __exit pisces_exit(void)
{
	platform_device_unregister(pisces_snd_device);
}

module_init(pisces_init);
module_exit(pisces_exit);

/* Module information */
MODULE_AUTHOR("Lutts Wolf <slcao@ingenic.cn>");
MODULE_DESCRIPTION("ALSA SoC Internel Codec Pisces");
MODULE_LICENSE("GPL");
