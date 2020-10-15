#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>

#include "jz4760_icdc.h"

static int jz_icdc_debug = 0;
module_param(jz_icdc_debug, int, 0644);

#define DEBUG_MSG(msg...)			\
	do {					\
		if (jz_icdc_debug)		\
			printk("ICDC: " msg);	\
	} while(0)

/* codec private data */
struct jz_icdc_priv {
	struct snd_soc_codec codec;
	u8 reg_cache[JZ_ICDC_MAX_NUM];
};

static int bypass_enable = 0;

/*
 * read register cache
 */
static inline unsigned int jz_icdc_read_reg_cache(struct snd_soc_codec *codec,
						 unsigned int reg)
{
	u8 *cache = codec->reg_cache;

	if (reg >= JZ_ICDC_MAX_NUM)
		return -1;

	return cache[reg];
}

/*
 * write register cache
 */
static inline void jz_icdc_write_reg_cache(struct snd_soc_codec *codec,
					  unsigned int reg, unsigned int value)
{
	u8 *cache = codec->reg_cache;

	if (reg >= JZ_ICDC_MAX_NUM)
		return;

	cache[reg] = (u8)value;
}

static inline u8 jz_icdc_read_reg_hw(struct snd_soc_codec *codec, unsigned int reg)
{
        volatile int val;
	u8 *cache = codec->reg_cache;

        while (__icdc_rgwr_ready()) {
                ;//nothing...
        }

        __icdc_set_addr(reg);

	/* wait 4+ cycle */
	val= __icdc_get_value();
        val = __icdc_get_value();
        val = __icdc_get_value();
        val = __icdc_get_value();
        val = __icdc_get_value();

	val = __icdc_get_value();

        cache[reg] =  (u8)val;

	return cache[reg];
}

static inline void jz_icdc_write_reg_hw(unsigned int reg, u8 value)
{
        while (__icdc_rgwr_ready());

        REG_ICDC_RGADW = ICDC_RGADW_RGWR | ((reg << ICDC_RGADW_RGADDR_LSB) | value);

        while (__icdc_rgwr_ready());
}

static int jz_icdc_hw_write(void * unused, const char* data, int num) {
	jz_icdc_write_reg_hw(data[0], data[1]);

	return 2;
}

/*
 * write to the register space
 */
static int jz_icdc_write(struct snd_soc_codec *codec, unsigned int reg,
			unsigned int value)
{
	u8 data[2];

	data[0] = reg;
	data[1] = value;

	DEBUG_MSG("write reg=0x%02x to val=0x%02x\n", reg, value);

	jz_icdc_write_reg_cache(codec, reg, value);

	if (reg < JZ_ICDC_MAX_REGNUM) {
		if (codec->hw_write(codec->control_data, data, 2) == 2)
			return 0;
		else
			return -EIO;
	}

	return 0;
}

static inline void jz_icdc_update_reg(struct snd_soc_codec *codec,
					 unsigned int reg,
					 int lsb, int mask, u8 nval) {
	u8 oval = jz_icdc_read_reg_cache(codec, reg);

	oval &= ~(mask << lsb);
	oval |= (nval << lsb);

	jz_icdc_write(codec, reg, oval);
}

__attribute__((__unused__)) static void dump_icdc_regs(struct snd_soc_codec *codec,
		   const char *func, int line)
{
        unsigned int i;

        printk("codec register dump, %s:%d:\n", func, line);
        for (i = 0; i < JZ_ICDC_MAX_REGNUM; i++)
                printk("address = 0x%02x, data = 0x%02x\n",
		       i, jz_icdc_read_reg_cache(codec, i));
}

__attribute__((__unused__)) static void dump_aic_regs(const char *func, int line)
{
        char *regname[] = {"aicfr","aiccr","aiccr1","aiccr2","i2scr","aicsr","acsr","i2ssr",
                           "accar", "accdr", "acsar", "acsdr", "i2sdiv"};
        int i;
        unsigned int addr;

        printk("AIC regs dump, %s:%d:\n", func, line);
        for (i = 0; i <= 0x30; i += 4) {
                addr = 0xb0020000 + i;
                printk("%s\t0x%08x -> 0x%08x\n", regname[i/4], addr, *(unsigned int *)addr);
        }
}

static void turn_on_sb_hp(struct snd_soc_codec *codec) {
	if (jz_icdc_read_reg_cache(codec, JZ_ICDC_PMR2) & (1 << ICDC_PMR2_SB_HP)) {
		/* power on sb hp */
		jz_icdc_update_reg(codec, JZ_ICDC_PMR2, ICDC_PMR2_SB_HP, 0x1, 0);


		while( !(jz_icdc_read_reg_hw(codec, JZ_ICDC_IFR) & (1 << ICDC_IFR_RUP)))
			mdelay(10);

		mdelay(1);

		/* clear RUP flag */
		jz_icdc_update_reg(codec, JZ_ICDC_IFR, ICDC_IFR_RUP, 0x1, 1);
	}
}

static void turn_off_sb_hp(struct snd_soc_codec *codec) {
	if (!(jz_icdc_read_reg_cache(codec, JZ_ICDC_PMR2) & (1 << ICDC_PMR2_SB_HP))) {
		/* power on sb hp */
		jz_icdc_update_reg(codec, JZ_ICDC_PMR2, ICDC_PMR2_SB_HP, 0x1, 1);


		while( !(jz_icdc_read_reg_hw(codec, JZ_ICDC_IFR) & (1 << ICDC_IFR_RDO)))
			mdelay(10);

		mdelay(1);

		/* clear RUP flag */
		jz_icdc_update_reg(codec, JZ_ICDC_IFR, ICDC_IFR_RDO, 0x1, 1);
	}
}

static void turn_on_dac(struct snd_soc_codec *codec) {
	if (jz_icdc_read_reg_cache(codec, JZ_ICDC_CR2) & (1 << ICDC_CR2_DAC_MUTE)) {
		/* power on sb hp */
		jz_icdc_update_reg(codec, JZ_ICDC_CR2, ICDC_CR2_DAC_MUTE, 0x1, 0);


		while( !(jz_icdc_read_reg_hw(codec, JZ_ICDC_IFR) & (1 << ICDC_IFR_GUP)))
			mdelay(10);

		mdelay(1);

		/* clear RUP flag */
		jz_icdc_update_reg(codec, JZ_ICDC_IFR, ICDC_IFR_GUP, 0x1, 1);
	}
}

static void turn_off_dac(struct snd_soc_codec *codec) {
	if (!(jz_icdc_read_reg_cache(codec, JZ_ICDC_CR2) & (1 << ICDC_CR2_DAC_MUTE))) {
		/* power on sb hp */
		jz_icdc_update_reg(codec, JZ_ICDC_CR2, ICDC_CR2_DAC_MUTE, 0x1, 1);


		while( !(jz_icdc_read_reg_hw(codec, JZ_ICDC_IFR) & (1 << ICDC_IFR_GDO)))
			mdelay(10);

		mdelay(1);

		/* clear RUP flag */
		jz_icdc_update_reg(codec, JZ_ICDC_IFR, ICDC_IFR_GDO, 0x1, 1);
	}
}

static int jz_icdc_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		jz_icdc_update_reg(codec, JZ_ICDC_PMR1, ICDC_PMR1_SB_BIAS, 0x1, 0);
		mdelay(10);
		break;
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_OFF:
		jz_icdc_update_reg(codec, JZ_ICDC_PMR1, ICDC_PMR1_SB_BIAS, 0x1, 1);
		mdelay(10);
		break;
	}
	codec->bias_level = level;
	return 0;
}

static int jz_icdc_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;

	int playback = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);

	int bit_width = -1;
	int speed = -1;

	/* bit width */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		bit_width = 0;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
		bit_width = 1;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		bit_width = 2;
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
		bit_width = 3;
		break;
	}

	if (bit_width < 0)
		return -EINVAL;

	if (playback)
		jz_icdc_update_reg(codec, JZ_ICDC_AICR,
				   ICDC_AICR_DAC_ADWL_LSB,
				   ICDC_AICR_DAC_ADWL_MASK,
				   bit_width);
	else
		jz_icdc_update_reg(codec, JZ_ICDC_AICR,
				   ICDC_AICR_ADC_ADWL_LSB,
				   ICDC_AICR_ADC_ADWL_MASK,
				   bit_width);
	/* channels */
	if (playback) {
		switch(params_channels(params)) {
		case 1:
			/* stereo-to-mono for DAC path */
			jz_icdc_update_reg(codec, JZ_ICDC_CR2,
					   ICDC_CR2_MONO, 0x1, 1);
			break;
		case 2:
			jz_icdc_update_reg(codec, JZ_ICDC_CR2,
					   ICDC_CR2_MONO, 0x1, 0);
			break;
		}
	}

	/* sample rate */
	switch (params_rate(params)) {
        case 96000:
                speed = 0;
                break;
        case 48000:
                speed = 1;
                break;
        case 44100:
                speed = 2;
                break;
        case 32000:
                speed = 3;
                break;
        case 24000:
                speed = 4;
                break;
        case 22050:
                speed = 5;
                break;
        case 16000:
                speed = 6;
                break;
        case 12000:
                speed = 7;
                break;
        case 11025:
                speed = 8;
                break;
        case 8000:
                speed = 9;
		break;
        }

	if (speed < 0)
		return -EINVAL;

	if (playback)
		jz_icdc_update_reg(codec, JZ_ICDC_CCR2,
				   ICDC_CCR2_DAC_FREQ_LSB,
				   ICDC_CCR2_DAC_FREQ_MASK,
				   speed);
	else
		jz_icdc_update_reg(codec, JZ_ICDC_CCR2,
				   ICDC_CCR2_ADC_FREQ_LSB,
				   ICDC_CCR2_ADC_FREQ_MASK,
				   speed);

	return 0;
}

static void jz_icdc_shutdown(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;

	int playback = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);

	DEBUG_MSG("enter jz_icdc_powerdown, playback = %d\n", playback);

	if (playback || bypass_enable) {
		turn_off_dac(codec);

		/* anti-pop workaround */
		__aic_write_tfifo(0x0);
		__aic_write_tfifo(0x0);
		__i2s_enable_replay();
		__i2s_enable();
		mdelay(1);
		__i2s_disable_replay();
		__i2s_disable();

		turn_off_sb_hp(codec);

		/* power down SB_DAC */
		jz_icdc_update_reg(codec, JZ_ICDC_PMR2, ICDC_PMR2_SB_DAC, 0x1, 1);
		mdelay(1);
	}
	mdelay(10);
}

static int jz_icdc_pcm_trigger(struct snd_pcm_substream *substream,
			       int cmd, struct snd_soc_dai *codec_dai) {
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	int ret = 0;

	DEBUG_MSG("enter %s:%d substream = %s bypass_enable = %d cmd = %d\n",
		  __func__, __LINE__,
		  (substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "playback" : "capture"),
		  bypass_enable,
		  cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if ((substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ||
		    bypass_enable){
			jz_icdc_update_reg(codec, JZ_ICDC_PMR2, ICDC_PMR2_SB_DAC, 0x1, 0);
			mdelay(1);

			jz_icdc_update_reg(codec, JZ_ICDC_CR1, ICDC_CR1_HP_MUTE, 0x1, 0);
			mdelay(1);

			turn_on_sb_hp(codec);

			turn_on_dac(codec);
			//dump_icdc_regs(codec, __func__, __LINE__);
			//dump_aic_regs(__func__, __LINE__);
		} else {
			jz_icdc_set_bias_level(codec, SND_SOC_BIAS_ON);
			jz_icdc_update_reg(codec, JZ_ICDC_CR2, ICDC_CR2_NOMAD, 0x1, 0);
			mdelay(2);

			//dump_icdc_regs(codec, __func__, __LINE__);
			//dump_aic_regs(__func__, __LINE__);
		}

		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* do nothing */
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int jz_icdc_mute(struct snd_soc_dai *dai, int mute)
{
	return 0;
}

static int jz_icdc_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	return 0;
}

#define JZ_ICDC_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |	\
		       SNDRV_PCM_RATE_12000 | SNDRV_PCM_RATE_16000 |	\
		       SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_24000 |	\
		       SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |	\
		       SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000)

#define JZ_ICDC_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S18_3LE | \
			 SNDRV_PCM_FMTBIT_S20_3LE |SNDRV_PCM_FMTBIT_S24_3LE)

static struct snd_soc_dai_ops jz_icdc_dai_ops = {
	.hw_params	= jz_icdc_hw_params,
	.trigger	= jz_icdc_pcm_trigger,
	.shutdown	= jz_icdc_shutdown,
	.digital_mute	= jz_icdc_mute,
	.set_sysclk	= jz_icdc_set_dai_sysclk,
};

struct snd_soc_dai jz_icdc_dai = {
	.name = "jz4760-icdc",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = JZ_ICDC_RATES,
		.formats = JZ_ICDC_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = JZ_ICDC_RATES,
		.formats = JZ_ICDC_FORMATS,},
	.ops = &jz_icdc_dai_ops,
};
EXPORT_SYMBOL_GPL(jz_icdc_dai);

static struct snd_soc_codec *jz4760_codec;
static struct jz_icdc_priv jz4760_priv;

/* unit: 0.01dB */
static const DECLARE_TLV_DB_SCALE(dac_tlv, -3100, 100, 0);
static const DECLARE_TLV_DB_SCALE(adc_tlv, 0, 100, 0);
static const DECLARE_TLV_DB_SCALE(out_tlv, -2500, 100, 0);
static const DECLARE_TLV_DB_SCALE(mic1_boost_tlv, 0, 400, 0);
static const DECLARE_TLV_DB_SCALE(mic2_boost_tlv, 0, 400, 0);
static const DECLARE_TLV_DB_SCALE(linein_tlv, -2500, 100, 0);

static const struct snd_kcontrol_new jz_icdc_snd_controls[] = {
	/* playback gain control */
	SOC_DOUBLE_R_TLV("PCM Volume", JZ_ICDC_CGR5, JZ_ICDC_CGR6, 0, 31, 1, dac_tlv),
	SOC_DOUBLE_R_TLV("Master Playback Volume", JZ_ICDC_CGR1, JZ_ICDC_CGR2,
			 0, 31, 1, out_tlv),

	/* record gain control */
	SOC_DOUBLE_R_TLV("ADC Capture Volume", JZ_ICDC_CGR8, JZ_ICDC_CGR9, 0, 23, 0,
			 adc_tlv),

	SOC_SINGLE_TLV("Mic1 Capture Volume", JZ_ICDC_CGR7, 3, 5, 0,
		       mic1_boost_tlv),
	SOC_SINGLE_TLV("Mic2 Capture Volume", JZ_ICDC_CGR7, 0, 5, 0,
		       mic2_boost_tlv),

	SOC_DOUBLE_R_TLV("Linein Bypass Capture Volume", JZ_ICDC_CGR3, JZ_ICDC_CGR4,
			 0, 31, 1, linein_tlv),
};

static int lineout_event(struct snd_soc_dapm_widget *w,
		     struct snd_kcontrol *kcontrol, int event) {
	struct snd_soc_codec *codec = w->codec;
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		jz_icdc_update_reg(codec, JZ_ICDC_CR1, ICDC_CR1_LOUT_MUTE, 0x1, 0);
		break;

	case SND_SOC_DAPM_POST_PMD:
		jz_icdc_update_reg(codec, JZ_ICDC_CR1, ICDC_CR1_LOUT_MUTE, 0x1, 1);
		break;
	}

	return 0;
}

static int btl_out_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event) {
	struct snd_soc_codec *codec = w->codec;
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		jz_icdc_update_reg(codec, JZ_ICDC_CR1, ICDC_CR1_BTL_MUTE, 0x1, 0);
		break;

	case SND_SOC_DAPM_POST_PMD:
		jz_icdc_update_reg(codec, JZ_ICDC_CR1, ICDC_CR1_BTL_MUTE, 0x1, 1);
		break;
	}

	return 0;
}

static const char *jz_icdc_input_sel[] = {"Mic 1", "Mic 2", "Line In"};
static const char *jz_icdc_output_sel[] = {"Mic 1b", "Mic 2b", "Line Inb", "Stereo DAC"};

#define INSEL_FROM_MIC1		0
#define INSEL_FROM_MIC2		1
#define INSEL_FROM_LINEIN	2

#define OUTSEL_FROM_MIC1	0
#define OUTSEL_FROM_MIC2	1
#define OUTSEL_FROM_LINEIN	2
#define OUTSEL_FROM_DAC		3

static int out_mux_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event) {

	struct snd_soc_codec *codec = w->codec;
	u8 *cache = codec->reg_cache;

	int mic_stereo = 0;
	u8 outsel = 0xff;      /* an invalid value */


	switch (event) {
	case SND_SOC_DAPM_POST_REG:
		DEBUG_MSG("%s:%d, POST_REG, loutsel = %d routsel = %d\n",
		       __func__, __LINE__, cache[JZ_ICDC_LOUTSEL], cache[JZ_ICDC_ROUTSEL]);

		/*
		 * panda out, take care!!!
		 * only the following are supported, else the result is unexpected!
		 *	* L(mic1) + R(mic1)
		 *	* L(mic2) + R(mic2)
		 *	* L(mic1) + R(mic2)
		 *	* L(mic2) + R(mic1)
		 *	* L(linein) + R(linein)
		 *	* L(DAC) + R(DAC)
		 **/
		if ((cache[JZ_ICDC_LOUTSEL] == OUTSEL_FROM_DAC) &&
		    (cache[JZ_ICDC_ROUTSEL] == OUTSEL_FROM_DAC)) {

			outsel = 3;

		} else if ((cache[JZ_ICDC_LOUTSEL] == OUTSEL_FROM_MIC1) &&
		    (cache[JZ_ICDC_ROUTSEL] == OUTSEL_FROM_MIC1)) {

			outsel = 0;

		} else if ((cache[JZ_ICDC_LOUTSEL] == OUTSEL_FROM_MIC2) &&
			   (cache[JZ_ICDC_ROUTSEL] == OUTSEL_FROM_MIC2)) {
			outsel = 1;

		} else if ((cache[JZ_ICDC_LOUTSEL] == OUTSEL_FROM_MIC1) &&
			   (cache[JZ_ICDC_ROUTSEL] == OUTSEL_FROM_MIC2)) {

			mic_stereo = 1;
			outsel = 1;

		} else if ((cache[JZ_ICDC_LOUTSEL] == OUTSEL_FROM_MIC2) &&
			   (cache[JZ_ICDC_ROUTSEL] == OUTSEL_FROM_MIC1)) {

			mic_stereo = 1;
			outsel = 0;

		} else if ((cache[JZ_ICDC_LOUTSEL] == OUTSEL_FROM_LINEIN) &&
			   (cache[JZ_ICDC_ROUTSEL] == OUTSEL_FROM_LINEIN)) {

			outsel = 2;

		}

		DEBUG_MSG("%s:%d, outsel = %d, mic_stereo = %d\n",
		       __func__, __LINE__, outsel, mic_stereo);

		if (outsel != 0xff) {
			jz_icdc_update_reg(codec, JZ_ICDC_CR1, 0, 0x3, outsel);

			if (mic_stereo)
				jz_icdc_update_reg(codec, JZ_ICDC_CR3, 1, 0x1, 1);
			else
				jz_icdc_update_reg(codec, JZ_ICDC_CR3, 1, 0x1, 0);


			if (outsel != 3)
				bypass_enable = 1;
			else
				bypass_enable = 0;
		}

		break;
	}

	return 0;
}

static int capture_mux_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event) {

	struct snd_soc_codec *codec = w->codec;
	u8 *cache = codec->reg_cache;

	int mic_stereo = 0;
	u8 insel = 0xff;      /* an invalid value */

	switch (event) {
	case SND_SOC_DAPM_POST_REG:
		DEBUG_MSG("%s:%d, POST_REG, linsel = %d rinsel = %d\n",
			  __func__, __LINE__, cache[JZ_ICDC_LINSEL], cache[JZ_ICDC_RINSEL]);

		/*
		 * panda out, take care!!!
		 * only the following are supported, else the result is unexpected!
		 *	* L(mic1) + R(mic1)
		 *	* L(mic2) + R(mic2)
		 *	* L(mic1) + R(mic2)
		 *	* L(mic2) + R(mic1)
		 *	* L(linein) + R(linein)
		 **/
		if ((cache[JZ_ICDC_LINSEL] == INSEL_FROM_MIC1) &&
		    (cache[JZ_ICDC_RINSEL] == INSEL_FROM_MIC1)) {

			insel = 0;

		} else if ((cache[JZ_ICDC_LINSEL] == INSEL_FROM_MIC2) &&
			   (cache[JZ_ICDC_RINSEL] == INSEL_FROM_MIC2)) {

			insel = 1;

		} else if ((cache[JZ_ICDC_LINSEL] == INSEL_FROM_LINEIN) &&
			   (cache[JZ_ICDC_RINSEL] == INSEL_FROM_LINEIN)) {

			insel = 2;

		} else if ((cache[JZ_ICDC_LINSEL] == INSEL_FROM_MIC1) &&
			   (cache[JZ_ICDC_RINSEL] == INSEL_FROM_MIC2)) {

			insel = 1;
			mic_stereo = 1;

		} else if ((cache[JZ_ICDC_LINSEL] == INSEL_FROM_MIC2) &&
			   (cache[JZ_ICDC_RINSEL] == INSEL_FROM_MIC1)) {

			insel = 0;
			mic_stereo = 1;
		}

		DEBUG_MSG("%s:%d, insel = %d, mic_stereo = %d\n",
			  __func__, __LINE__, insel, mic_stereo);

		if (insel != 0xff) {
			jz_icdc_update_reg(codec, JZ_ICDC_CR3, 2, 0x3, insel);

			if (mic_stereo)
				jz_icdc_update_reg(codec, JZ_ICDC_CR3, 1, 0x1, 1);
			else
				jz_icdc_update_reg(codec, JZ_ICDC_CR3, 1, 0x1, 0);
		}

		break;

	default:
		break;
	}
	return 0;
}


static const struct soc_enum jz_icdc_enum[] = {
	SOC_ENUM_SINGLE(JZ_ICDC_LOUTSEL, 0, 4, jz_icdc_output_sel),
	SOC_ENUM_SINGLE(JZ_ICDC_ROUTSEL, 0, 4, jz_icdc_output_sel),


	SOC_ENUM_SINGLE(JZ_ICDC_LINSEL, 0, 3, jz_icdc_input_sel),
	SOC_ENUM_SINGLE(JZ_ICDC_RINSEL, 0, 3, jz_icdc_input_sel),
};

static const struct snd_kcontrol_new icdc_left_output_mux_controls =
	SOC_DAPM_ENUM("Route", jz_icdc_enum[0]);

static const struct snd_kcontrol_new icdc_right_output_mux_controls =
	SOC_DAPM_ENUM("Route", jz_icdc_enum[1]);

static const struct snd_kcontrol_new icdc_adc_left_controls =
	SOC_DAPM_ENUM("Route", jz_icdc_enum[2]);

static const struct snd_kcontrol_new icdc_adc_right_controls =
	SOC_DAPM_ENUM("Route", jz_icdc_enum[3]);


static const struct snd_soc_dapm_widget jz_icdc_dapm_widgets[] = {
	SND_SOC_DAPM_PGA("HP Out", JZ_ICDC_CR1, ICDC_CR1_HP_MUTE, 1, NULL, 0),

	SND_SOC_DAPM_PGA_E("Line Out", JZ_ICDC_PMR2, 1, 1, NULL, 0,
			   lineout_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("BTL Out", JZ_ICDC_PMR2, 2, 1, NULL, 0,
			   btl_out_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA("Line Input", JZ_ICDC_PMR1, 4, 1, NULL, 0),

	SND_SOC_DAPM_PGA("Mic1 Input", JZ_ICDC_PMR1, 3, 1, NULL, 0),

	SND_SOC_DAPM_PGA("Mic2 Input", JZ_ICDC_PMR1, 2, 1, NULL, 0),

	SND_SOC_DAPM_PGA("Linein Bypass", JZ_ICDC_PMR1, 1, 1, NULL, 0),

	SND_SOC_DAPM_ADC("ADC", "HiFi Capture", JZ_ICDC_PMR2, 4, 1),
	SND_SOC_DAPM_DAC("DAC", "HiFi Playback", SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MICBIAS("Mic Bias", JZ_ICDC_PMR1, 0, 1),

	SND_SOC_DAPM_MUX_E("Left Out Mux", SND_SOC_NOPM, 0, 0,
			   &icdc_left_output_mux_controls,
			   out_mux_event,
			   SND_SOC_DAPM_POST_REG),

	SND_SOC_DAPM_MUX_E("Right Out Mux", SND_SOC_NOPM, 0, 0,
			   &icdc_right_output_mux_controls,
			   out_mux_event,
			   SND_SOC_DAPM_POST_REG),

	SND_SOC_DAPM_MUX_E("Capture Left Mux", SND_SOC_NOPM, 0, 0,
			   &icdc_adc_left_controls,
			   capture_mux_event,
			   SND_SOC_DAPM_POST_REG),

	SND_SOC_DAPM_MUX_E("Capture Right Mux", SND_SOC_NOPM, 0, 0,
			   &icdc_adc_right_controls,
			   capture_mux_event,
			   SND_SOC_DAPM_POST_REG),

	SND_SOC_DAPM_OUTPUT("LHPOUT"),
	SND_SOC_DAPM_OUTPUT("RHPOUT"),

	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),

	SND_SOC_DAPM_OUTPUT("BTLP"),
	SND_SOC_DAPM_OUTPUT("BTLN"),

	/* FIXME: determine MICDIFF here. */
	SND_SOC_DAPM_INPUT("MIC1P"),
	SND_SOC_DAPM_INPUT("MIC1N"),
	SND_SOC_DAPM_INPUT("MIC2P"),
	SND_SOC_DAPM_INPUT("MIC2N"),

	SND_SOC_DAPM_INPUT("LLINEIN"),
	SND_SOC_DAPM_INPUT("RLINEIN"),
};

static const struct snd_soc_dapm_route intercon[] = {
	/* Destination Widget  <=== Path Name <=== Source Widget */
	{ "Mic1 Input", NULL, "MIC1P" },
	{ "Mic1 Input", NULL, "MIC1N" },

	{ "Mic2 Input", NULL, "MIC2P" },
	{ "Mic2 Input", NULL, "MIC2N" },

	{ "Line Input", NULL, "LLINEIN" },
	{ "Line Input", NULL, "RLINEIN" },


	{ "Capture Left Mux", "Mic 1", "Mic1 Input"  },
	{ "Capture Left Mux", "Mic 2", "Mic2 Input"  },
	{ "Capture Left Mux", "Line In", "Line Input" },

	{ "Capture Right Mux", "Mic 1", "Mic1 Input"  },
	{ "Capture Right Mux", "Mic 2", "Mic2 Input"  },
	{ "Capture Right Mux", "Line In", "Line Input" },

	{ "ADC", NULL, "Capture Right Mux" },

	{ "Left Out Mux", "Mic 1b", "Mic1 Input" },
	{ "Left Out Mux", "Mic 2b", "Mic2 Input" },

	{ "Left Out Mux", "Line Inb", "Line Input" },

	{ "Left Out Mux", "Stereo DAC", "DAC" },


	{ "Right Out Mux", "Mic 1b", "Mic1 Input" },
	{ "Right Out Mux", "Mic 2b", "Mic2 Input" },

	{ "Right Out Mux", "Line Inb", "Line Input" },

	{ "Right Out Mux", "Stereo DAC", "DAC" },

	{ "HP Out", NULL, "Left Out Mux" },
	{ "HP Out", NULL, "Right Out Mux" },

	{ "LHPOUT", NULL, "HP Out"},
	{ "RHPOUT", NULL, "HP Out"},

	{ "Line Out", NULL, "LHPOUT" },
	{ "Line Out", NULL, "RHPOUT" },

	{ "LOUT", NULL, "Line Out"},
	{ "ROUT", NULL, "Line Out"},

	{ "BTL Out", NULL, "LOUT" },
	{ "BTL Out", NULL, "ROUT" },

	{ "BTLP", NULL, "BTL Out"},
	{ "BTLN", NULL, "BTL Out"},

};

static int jz_icdc_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, jz_icdc_dapm_widgets,
				  ARRAY_SIZE(jz_icdc_dapm_widgets));

	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

static int jz_icdc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int jz_icdc_resume(struct platform_device *pdev)
{
	return 0;
}

static int jz_icdc_probe(struct platform_device *pdev)
{
	//lepus_snd_devdata
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	if (jz4760_codec == NULL) {
		dev_err(&pdev->dev, "Codec device not registered\n");
		return -ENODEV;
	}

	socdev->card->codec = jz4760_codec;
	codec = jz4760_codec;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_WARNING"failed to create pcms: %d\n", ret);
		goto pcm_err;
	}

	snd_soc_add_controls(codec, jz_icdc_snd_controls,
			     ARRAY_SIZE(jz_icdc_snd_controls));
	jz_icdc_add_widgets(codec);
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		printk(KERN_WARNING"failed to register card: %d\n", ret);
		goto card_err;
	}

	return ret;

 card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
 pcm_err:
	return ret;
}

/* power down chip */
static int jz_icdc_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return 0;
}

struct snd_soc_codec_device jz_icdc_soc_codec_dev = {
	.probe = 	jz_icdc_probe,
	.remove = 	jz_icdc_remove,
	.suspend =	jz_icdc_suspend,
	.resume =	jz_icdc_resume,
};
EXPORT_SYMBOL_GPL(jz_icdc_soc_codec_dev);


static int jz_icdc_register(void)
{
	int ret, i;
	struct snd_soc_codec *codec = &jz4760_priv.codec;
	u8 *cache = NULL;

	cpm_start_clock(CGM_AIC);

	memset(&jz4760_priv, 0, sizeof(jz4760_priv));

	if (jz4760_codec) {
		dev_err(codec->dev, "Multiple Codec devices not supported, and this should not happen\n");
		ret = -EINVAL;
		goto err;
	}

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	codec->name = "jz4760-icdc";
	codec->owner = THIS_MODULE;
	codec->read = jz_icdc_read_reg_cache;
	codec->hw_write = jz_icdc_hw_write;
	codec->write = jz_icdc_write;
	codec->bias_level = SND_SOC_BIAS_OFF;
	codec->set_bias_level = jz_icdc_set_bias_level;
	codec->dai = &jz_icdc_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = JZ_ICDC_MAX_NUM;
	codec->reg_cache = &jz4760_priv.reg_cache;
	codec->private_data = &jz4760_priv;

	cache = codec->reg_cache;
	for (i = 0; i < JZ_ICDC_MAX_REGNUM; i++)
		cache[i] = jz_icdc_read_reg_hw(codec, i);

	cache[JZ_ICDC_LOUTSEL] = OUTSEL_FROM_DAC;
	cache[JZ_ICDC_ROUTSEL] = OUTSEL_FROM_DAC;

	cache[JZ_ICDC_LINSEL] = INSEL_FROM_MIC1;
	cache[JZ_ICDC_RINSEL] = INSEL_FROM_MIC1;

	//INIT_DELAYED_WORK(&codec->delayed_work, jz_icdc_work);

	/* init codec params */
	/* ADC/DAC: serial + i2s */
	jz_icdc_update_reg(codec, JZ_ICDC_AICR, 0, 0xf, 0xf);

	jz_icdc_update_reg(codec, JZ_ICDC_PMR1, ICDC_PMR1_SB_AIP, 0x1, 0);

	jz_icdc_update_reg(codec, JZ_ICDC_ICR,
			   ICDC_ICR_INT_FORM_LSB,
			   ICDC_ICR_INT_FORM_MASK,
			   ICDC_INT_FORM_HIGH);

	jz_icdc_update_reg(codec, JZ_ICDC_IFR, 0, 0x7f, 0x7f);
	cache[JZ_ICDC_IFR] = jz_icdc_read_reg_hw(codec, JZ_ICDC_IFR);

	jz_icdc_update_reg(codec, JZ_ICDC_ICR, 0, 0x3f,
			   ((1 <<ICDC_ICR_GDO_MASK) |
			    (1 << ICDC_ICR_GUP_MASK) |
			    (1 << ICDC_ICR_RDO_MASK) |
			    (1 << ICDC_ICR_RUP_MASK) |
			    (1 <<ICDC_ICR_JACK_MASK)));

	jz_icdc_update_reg(codec, JZ_ICDC_CCR1,
			   ICDC_CCR1_CRYSTAL_LSB,
			   ICDC_CCR1_CRYSTAL_MASK,
			   ICDC_CRYSTAL_12M);

	/* 0: 16ohm/220uF, 1: 10kohm/1uF */
	jz_icdc_update_reg(codec, JZ_ICDC_CR1,
			   ICDC_CR1_LOAD, 0x1,
			   0);

	/* disable AGC */
	jz_icdc_update_reg(codec, JZ_ICDC_AGC1, 7, 0x1, 0);
	/* default to MICDIFF */
	jz_icdc_update_reg(codec, JZ_ICDC_CR3, 0, 0x1, 1);

	/* mic mono */
	jz_icdc_update_reg(codec, JZ_ICDC_CR3, 1, 0x1, 0);

	/* mute lineout/BTL/HP */
	jz_icdc_update_reg(codec, JZ_ICDC_CR1, ICDC_CR1_BTL_MUTE, 0x1, 1);
	jz_icdc_update_reg(codec, JZ_ICDC_CR1, ICDC_CR1_LOUT_MUTE, 0x1, 1);
	jz_icdc_update_reg(codec, JZ_ICDC_CR1, ICDC_CR1_HP_MUTE, 0x1, 1);

	/* default to NOMAD mode */
	jz_icdc_update_reg(codec, JZ_ICDC_CR2, ICDC_CR2_NOMAD, 0x1, 1);

	jz4760_codec = codec;

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		printk(KERN_WARNING"Failed to register codec: %d\n", ret);
		return ret;
	}

	ret = snd_soc_register_dai(&jz_icdc_dai);
	if (ret != 0) {
		printk(KERN_WARNING"Failed to register DAI: %d\n", ret);
		snd_soc_unregister_codec(codec);
		return ret;
	}

	/* These steps are too time consuming, so we do it here */
	/* clr SB */
	jz_icdc_update_reg(codec, JZ_ICDC_PMR1, ICDC_PMR1_SB, 0x1, 0);
	mdelay(300);

	/* clr SB_SLEEP */
	jz_icdc_update_reg(codec, JZ_ICDC_PMR1, ICDC_PMR1_SB_SLEEP, 0x1, 0);
	mdelay(400);

	return 0;

 err:
	return ret;
}

static void jz_icdc_unregister(void)
{
	/* clr SB_SLEEP */
	jz_icdc_update_reg(&jz4760_priv.codec, JZ_ICDC_PMR1,
			   ICDC_PMR1_SB_SLEEP, 0x1, 1);
	mdelay(10);

	/* clr SB */
	jz_icdc_update_reg(&jz4760_priv.codec, JZ_ICDC_PMR1,
			   ICDC_PMR1_SB, 0x1, 1);

	jz_icdc_set_bias_level(&jz4760_priv.codec, SND_SOC_BIAS_OFF);
	snd_soc_unregister_dai(&jz_icdc_dai);
	snd_soc_unregister_codec(&jz4760_priv.codec);
	jz4760_codec = NULL;
}

static int __init jz_icdc_modinit(void)
{
	return jz_icdc_register();
}
module_init(jz_icdc_modinit);

static void __exit jz_icdc_exit(void)
{
	jz_icdc_unregister();
}
module_exit(jz_icdc_exit);

MODULE_DESCRIPTION("Jz4760 Internal Codec Driver");
MODULE_AUTHOR("Lutts Wolf<slcao@ingenic.cn>");
MODULE_LICENSE("GPL");
