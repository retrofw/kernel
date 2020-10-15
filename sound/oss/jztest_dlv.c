/*
 * Linux/sound/oss/jz_dlv.c
 *
 * DLV CODEC driver for Ingenic Jz4750 MIPS processor
 *
 * 2009-12-xx	Steven <dsqiu@ingenic.cn>
 * 2010-01-xx	Jason <xwang@ingenic.cn>
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
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

#include "sound_config.h"

#include "jz_audio.h"
#include "jz_codec.h"
#include "jz4750_dlv.h"
#include "jz_i2s_dbg.h"

#define REPLAY		1
#define RECORD		2

#define POWER_ON	0
#define POWER_OFF	1

#define switch_SB_DAC(pwrstat)			\
do {						\
	dlv_write_reg_bit(5, pwrstat, 7);	\
} while (0)					\

#define switch_SB_OUT(pwrstat)			\
do {						\
	dlv_write_reg_bit(5, pwrstat, 6);	\
} while (0)					\

#define switch_SB_MIX(pwrstat)			\
do {						\
	dlv_write_reg_bit(5, pwrstat, 5);	\
} while (0)					\

#define switch_SB_ADC(pwrstat)			\
do {						\
	dlv_write_reg_bit(5, pwrstat, 4);	\
} while (0)					\

/*
#define ENTER()			printk("Enter: %s, %s:%i\n", __FUNCTION__, __FILE__, __LINE__)
#define LEAVE()			printk("Leave: %s, %s:%i\n", __FUNCTION__, __FILE__, __LINE__)
*/

#ifdef IOC_DEBUG
static dlv_print_ioc_cmd(int cmd)
{
	char *dlv_ioc_cmd[] = {
		"CODEC_SET_MODE",		"CODEC_CLEAR_MODE",		"CODEC_SET_GPIO_PIN",
		"CODEC_EACH_TIME_INIT",		"CODEC_SET_STARTUP_PARAM",	"CODEC_SET_VOLUME_TABLE",
		"CODEC_SET_RECORD",		"CODEC_SET_REPLAY",		"CODEC_SET_REPLAY_RECORD",
		"CODEC_TURN_ON",		"CODEC_TURN_OFF",		"CODEC_SET_REPLAY_SPEED",
		"CODEC_RESET",			"CODEC_GET_MIXER_OLD_INFO",	"CODEC_GET_MIXER_INFO",
		"CODEC_SET_BASS",		"CODEC_SET_VOLUME",		"CODEC_SET_MIC",
		"CODEC_SET_LINE",		"CODEC_I2S_RESUME",		"CODEC_I2S_SUSPEND",
		"CODEC_PIN_INIT",		"CODEC_SET_SOME_FUNC",		"CODEC_CLEAR_RECORD",
		"CODEC_CLEAR_REPLAY",		"CODEC_SET_REPLAY_HP_OR_SPKR",	"CODEC_SET_DIRECT_MODE",
		"CODEC_CLEAR_DIRECT_MODE",	"CODEC_SET_LINEIN2HP",		"CODEC_CLEAR_LINEIN2HP",
		"CODEC_ANTI_POP",		"CODEC_TURN_REPLAY",		"CODEC_SET_REPLAY_CHANNEL",
		"CODEC_SET_REPLAY_FORMAT",	"CODEC_SET_RECORD_CHANNEL",	"CODEC_SET_RECORD_FORMAT",
		"CODEC_SET_RECORD_SPEED",	"CODEC_DAC_MUTE"
	};

	if (cmd >= (sizeof(dlv_ioc_cmd) / sizeof(dlv_ioc_cmd[0]))) {
		printk("%s: Unkown command !\n", __FUNCTION__);
	} else {
		printk("IOC CMD NAME = %s\n", dlv_ioc_cmd[cmd - 1]);
	}
}
#endif

#if 1
/*
 * CODEC registers access routines
 */

/**
 * CODEC read register
 *
 * addr:        address of register
 * return:      value of register
 */
static inline int dlv_read_reg(int addr)
{
        volatile int reg;
        while (__icdc_rgwr_ready()) {
                ;//nothing...
        }
        __icdc_set_addr(addr);
        reg = __icdc_get_value();
        reg = __icdc_get_value();
        reg = __icdc_get_value();
        reg = __icdc_get_value();
        reg = __icdc_get_value();
        return __icdc_get_value();
}

/**
 * CODEC write register
 *
 * addr:        address of register
 * val:         value to set
 */
void dlv_write_reg(int addr, int val)
{
        volatile int reg;
        while (__icdc_rgwr_ready()) {
                ;//nothing...
        }
        REG_ICDC_RGADW = ((addr << ICDC_RGADW_RGADDR_LSB) | val);
        __icdc_set_rgwr();
        reg = __icdc_rgwr_ready();
        reg = __icdc_rgwr_ready();
        reg = __icdc_rgwr_ready();
        reg = __icdc_rgwr_ready();
        reg = __icdc_rgwr_ready();
        reg = __icdc_rgwr_ready();
        while (__icdc_rgwr_ready()) {
                ;//nothing...
        }
}

/**
 * CODEC write a bit of a register
 *
 * addr:        address of register
 * bitval:      bit value to modifiy
 * mask_bit:    indicate which bit will be modifiy
 */
static int dlv_write_reg_bit(int addr, int bitval, int mask_bit)
{
        int val = dlv_read_reg(addr);

        if (bitval)
                val |= (1 << mask_bit);
        else
                val &= ~(1 << mask_bit);
        dlv_write_reg(addr, val);

        return 1;
}

#else
static inline int dlv_read_reg(int addr)
{
	while (__icdc_rgwr_ready()) {
		;//nothing...
	}
	__icdc_set_addr(addr);
	return __icdc_get_value();
}

void dlv_write_reg(int addr, int val)
{
#if 0
	while (__icdc_rgwr_ready()) {
		;//nothing...
	}
	REG_ICDC_RGADW = ((addr << ICDC_RGADW_RGADDR_LSB) | val);
	__icdc_set_rgwr();
	while (__icdc_rgwr_ready()) {
		;//nothing...
	}
#else
	volatile int reg;
	while (__idc_rgwr_ready()) {
		; // nothing
	}
	REG_ICDC_RGADW = ((addr << ICDC_RGADW_RGADDR_LSB) | val);
	__icdc_set_rgwr();
	reg = __icdc_rgwr_ready();
	reg = __icdc_rgwr_ready();
	reg = __icdc_rgwr_ready();
	reg = __icdc_rgwr_ready();
	reg = __icdc_rgwr_ready();
	reg = __icdc_rgwr_ready();
	while (__icdc_rgwr_ready()) {
		; //nothing...
	}
#endif
}

static int dlv_write_reg_bit(int addr, int bitval, int mask_bit)
{
	int val;
	while (__icdc_rgwr_ready()) {
		;//nothing...
	}
	__icdc_set_addr(addr);
	mdelay(1);
	/* read */
	val = __icdc_get_value();
	while (__icdc_rgwr_ready()) {
		;//nothing...
	}

	__icdc_set_addr(addr);
	val &= ~(1 << mask_bit);
	if (bitval == 1) {
		val |= 1 << mask_bit;
	}

	__icdc_set_cmd(val); /* write */
	mdelay(1);
	__icdc_set_rgwr();
	mdelay(1);

	while (__icdc_rgwr_ready()) {
		;//nothing...
	}
	__icdc_set_addr(addr);
	val = __icdc_get_value(); /* read */	
	
	if (((val >> mask_bit) & bitval) == bitval) {
		return 1;
	} else {
		return 0;
	}
}
#endif

/*
 * DLV CODEC operations routines
 */

static void dlv_each_time_init(void)
{
	ENTER();
	__i2s_disable();
	__i2s_as_slave();
#if AIC_BASE == 0xb0020000
	__aic_internal_codec();
#else
	__aic0_external_codec();
	__aic_internal_codec();
#endif
	//__i2s_set_oss_sample_size(16);
	//__i2s_set_iss_sample_size(16);
	LEAVE();
}

static void dlv_set_mode(void)
{
	ENTER();
	/*REG_CPM_CPCCR &= ~(1 << 31);
	  REG_CPM_CPCCR &= ~(1 << 30);*/
	dlv_write_reg(0, 0xf);
	dlv_write_reg(8, 0x2f);
	dlv_write_reg(9, 0xff);
	schedule_timeout(2);

	dlv_write_reg_bit(6, 0, 1);//PMR2.SB->0
	msleep(10);
	dlv_write_reg_bit(6, 0, 0);//PMR2.SB->0
	msleep(10);
	DPRINT_CODEC("##### cleared codec reg6\n");
	dlv_write_reg_bit(1, 0, 3);//PMR2.SB->0

	dlv_write_reg_bit(5, 0, 4);//SB_ADC->1
//	set_record_mic_input_audio_with_audio_data_replay();
//	reset_dlv_codec();
	LEAVE();
}

static void dlv_reset(void)
{
	int i = 0;
	int reg = 0;
	ENTER();
	/* reset DLV codec. from hibernate mode to sleep mode */
#if 0
	for (i = 0; i < 27; i++) {
		dlv_write_reg(i, 0x5a);
	} 

	for (i = 0; i < 27; i++) {
		reg = dlv_read_reg(i);
		if (reg != 0x5a)  {
			printk("reg%d error: 0x%02x\n", i, reg);
		}
	}
#endif
	dlv_write_reg(0, 0xf);
	dlv_write_reg_bit(6, 0, 0);
	dlv_write_reg_bit(6, 0, 1);

	//2010-01-31 Jason add
	dlv_write_reg(22, 0x40);//mic 1

	schedule_timeout(20);
	//dlv_write_reg(0, 0xf);
	dlv_write_reg_bit(5, 0, 7);//PMR1.SB_DAC->0
	dlv_write_reg_bit(5, 0, 4);//PMR1.SB_ADC->0
	schedule_timeout(2); ;//wait for stability
	LEAVE();
}

static int dlv_set_startup_param(void)
{
	ENTER();
	LEAVE();
//	__i2s_disable_transmit_intr();
//	__i2s_disable_receive_intr();
	return 1;
}

//@@@@@@@@@@@@@@@@@@@@
/* set Audio data replay */
static void dlv_set_replay(void)
{
	ENTER();

	//dump_dlv_regs("enter dlv_set_replay");
	//printk("===>enter %s:%d, REG[0x9] = 0x%02x\n", __func__, __LINE__, dlv_read_reg(0x9));
	//msleep(5000);

	/* DAC path */
	dlv_write_reg(9, 0xff);
	//dlv_write_reg(8, 0x30);
	//dlv_write_reg(8, 0x20);

	schedule_timeout(2);
	//printk("===>enter %s:%d\n", __func__, __LINE__);
	//msleep(5000);
	dlv_write_reg_bit(1, 0, 4);//CR1.HP_DIS->0
	//printk("===>enter %s:%d\n", __func__, __LINE__);
	//msleep(5000);

	dlv_write_reg_bit(5, 1, 3);//PMR1.SB_LIN->1
	//printk("===>enter %s:%d\n", __func__, __LINE__);
	//msleep(5000);
	dlv_write_reg_bit(5, 1, 0);//PMR1.SB_IND->1
	//printk("===>enter %s:%d\n", __func__, __LINE__);
	//msleep(5000);

	//2010-01-31 Jason marked
	dlv_write_reg_bit(1, 0, 2);//CR1.BYPASS->0
	//printk("===>enter %s:%d\n", __func__, __LINE__);
	//msleep(5000);
	dlv_write_reg_bit(1, 1, 3);//CR1.DACSEL->1
	//printk("===>enter %s:%d\n", __func__, __LINE__);
	//msleep(5000);

	dlv_write_reg_bit(1, 0, 5);//MUTE
	//printk("===>enter %s:%d\n", __func__, __LINE__);
	//msleep(5000);

	dlv_write_reg_bit(5, 0, 5);//PMR1.SB_MIX->0
	//printk("===>enter %s:%d\n", __func__, __LINE__);
	//msleep(5000);
	//mdelay(100);
	//dlv_write_reg_bit(5, 0, 6);//PMR1.SB_OUT->0

	//2010-01-31 Jason marked
	//dlv_write_reg_bit(1, 1, 7);//CR1.SB_MICBIAS->1

	//mdelay(300);
	//dump_dlv_regs("leave dlv_set_replay");
	LEAVE();
}

#if 0
// @@@@@@@@@@@@@@@@@@@@@@@@
/* set Record MIC input audio without playback */
static void set_record_mic_input_audio_without_playback(void)
{
	ENTER();
	// 2010-01-20 Jason added
/*
	dlv_write_reg_bit(6, 0, 0);//SB_SLEEP->0
	dlv_write_reg_bit(6, 0, 1);//SB->0
	dlv_write_reg_bit(5, 0, 4);//SB_ADC->0
*/
	/* ADC path for MIC IN */
	dlv_write_reg(9, 0xff);
	dlv_write_reg(8, 0x3f);
	dlv_write_reg_bit(23, 0, 7);//AGC1.AGC_EN->0
	schedule_timeout(2);
	dlv_write_reg_bit(1, 1, 2);
	//dlv_write_reg_bit(1, 1, 6);//CR1.MONO->1
	
	dlv_write_reg(22, 0x40);//mic 1
	dlv_write_reg_bit(3, 1, 7);//CR1.HP_DIS->1
	dlv_write_reg_bit(5, 1, 3);//PMR1.SB_LIN->1
	dlv_write_reg_bit(5, 1, 0);//PMR1.SB_IND->1
	
	dlv_write_reg_bit(1, 0, 2);//CR1.BYPASS->0
	dlv_write_reg_bit(1, 0, 3);//CR1.DACSEL->0
	//dlv_write_reg_bit(6, 1, 3);// gain set

	dlv_write_reg_bit(5, 0, 5);//PMR1.SB_MIX->0
	schedule_timeout(10);
	dlv_write_reg_bit(5, 0, 6);//PMR1.SB_OUT->0
	dlv_write_reg(1, 0x4);

	dlv_write_reg(19, (80 * 32 / 100) | ((80 * 32 / 100) << 4));

	// 2010-01-19 Jason added

	dlv_write_reg_bit(6, 0, 0);//SB_SLEEP->0
	dlv_write_reg_bit(6, 0, 1);//SB->0
	dlv_write_reg_bit(5, 0, 4);//SB_ADC->0

	LEAVE();
}

/* unset Record MIC input audio without playback */
static void unset_record_mic_input_audio_without_playback(void)
{
	ENTER();
	/* ADC path for MIC IN */
	// 2010-01-20 Jason modified
//	dlv_write_reg_bit(5, 1, 4);//SB_ADC->1
	dlv_write_reg_bit(1, 1, 7);//CR1.SB_MICBIAS->1
	dlv_write_reg(22, 0xc0);//CR3.SB_MIC1
	dlv_write_reg_bit(5, 1, 6);//PMR1.SB_OUT->1
	dlv_write_reg_bit(1, 1, 5);//DAC_MUTE->1
//	dlv_write_reg_bit(6, 1, 0);//SB_SLEEP->1
//	dlv_write_reg_bit(6, 1, 1);//SB->1
	LEAVE();
}
#endif

#if 0
/* set Record LINE input audio without playback */
static void set_record_line_input_audio_without_playback(void)
{
	ENTER();
	/* ADC path for LINE IN */	
	dlv_write_reg(9, 0xff);
	dlv_write_reg(8, 0x3f);
	mdelay(10);
	dlv_write_reg(22, 0xf6);//line in 1
	dlv_write_reg_bit(23, 0, 7);//AGC1.AGC_EN->0
	dlv_write_reg_bit(3, 1, 7);//CR1.HP_DIS->1
	dlv_write_reg_bit(5, 0, 3);//PMR1.SB_LIN->0
	dlv_write_reg_bit(5, 1, 0);//PMR1.SB_IND->1
	
	dlv_write_reg_bit(1, 0, 2);//CR1.BYPASS->0
	dlv_write_reg_bit(1, 0, 3);//CR1.DACSEL->0
	mdelay(10);
	dlv_write_reg_bit(5, 0, 5);//PMR1.SB_MIX->0
	mdelay(100);
	dlv_write_reg_bit(5, 0, 6);//PMR1.SB_OUT->0
	dlv_write_reg(1, 0x4);
	LEAVE();
}
#endif

#if 0
/* unset Record LINE input audio without playback */
static void unset_record_line_input_audio_without_playback(void)
{
	ENTER();
	/* ADC path for LINE IN */
	dlv_write_reg_bit(5, 1, 4);//SB_ADC->1
	dlv_write_reg_bit(5, 1, 3);//ONR1.SB_LIN->1

	dlv_write_reg(22, 0xc0);//CR3.SB_MIC1
	dlv_write_reg_bit(5, 1, 6);//PMR1.SB_OUT->1
	dlv_write_reg_bit(1, 1, 5);//DAC_MUTE->1
	dlv_write_reg_bit(6, 1, 0);//SB_SLEEP->1
	dlv_write_reg_bit(6, 1, 1);//SB->1
	LEAVE();
}
#endif

#if 0
/* set Playback LINE input audio direct only */
static void set_playback_line_input_audio_direct_only(void)
{
	ENTER();
// need fix !!!
//	jz_audio_reset();//or init_codec()
	REG_AIC_I2SCR = 0x10;
	dlv_write_reg(9, 0xff);
	dlv_write_reg(8, 0x3f);
	mdelay(10);
	dlv_write_reg(22, 0xf6);//line in 1
	dlv_write_reg_bit(23, 0, 7);//AGC1.AGC_EN->0
	mdelay(10);
	dlv_write_reg_bit(1, 1, 2);//CR1.HP_BYPASS->1
	dlv_write_reg_bit(1, 0, 4);//CR1.HP_DIS->0
	dlv_write_reg_bit(1, 0, 3);//CR1.DACSEL->0
	dlv_write_reg_bit(5, 1, 0);//PMR1.SB_IND->1
	dlv_write_reg_bit(5, 0, 3);//PMR1.SB_LIN->0

	dlv_write_reg_bit(5, 0, 5);//PMR1.SB_MIX->0
	mdelay(100);
	dlv_write_reg_bit(5, 0, 6);//PMR1.SB_OUT->0
	//dlv_write_reg_bit(5, 1, 7);//PMR1.SB_DAC->1
	//dlv_write_reg_bit(5, 1, 4);//PMR1.SB_ADC->1
	LEAVE();
}
#endif

#if 0
/* unset Playback LINE input audio direct only */
static void unset_playback_line_input_audio_direct_only(void)
{
	ENTER();
	dlv_write_reg_bit(6, 0, 3);//GIM->0
	dlv_write_reg_bit(1, 0, 2);//PMR1.BYPASS->0
	dlv_write_reg_bit(5, 1, 3);//PMR1.SB_LINE->1
	dlv_write_reg_bit(5, 1, 6);//PMR1.SB_OUT->1
	mdelay(100);
	dlv_write_reg_bit(5, 1, 5);//PMR1.SB_MIX->1
	dlv_write_reg_bit(6, 1, 0);//SB_SLEEP->1
	dlv_write_reg_bit(6, 1, 1);//SB->1
	LEAVE();
}
#endif

#if 0
/* set Record MIC input audio with direct playback */
static void set_record_mic_input_audio_with_direct_playback(void)
{
	ENTER();
	dlv_write_reg_bit(23, 0, 7);//AGC1.AGC_EN->0
	dlv_write_reg(9, 0xff);
	dlv_write_reg(8, 0x3f);
	mdelay(10);
	
	dlv_write_reg(22, 0x60);//mic 1
	dlv_write_reg_bit(23, 0, 7);//AGC1.AGC_EN->0
	dlv_write_reg_bit(5, 1, 3);//PMR1.SB_LIN->1
	dlv_write_reg_bit(5, 1, 0);//PMR1.SB_IND->1
	dlv_write_reg_bit(1, 0, 7);//CR1.SB_MICBIAS->0
	dlv_write_reg_bit(1, 0, 4);//CR1.HP_DIS->0
	
	dlv_write_reg_bit(1, 0, 2);//CR1.BYPASS->0
	dlv_write_reg_bit(1, 0, 3);//CR1.DACSEL->0
	dlv_write_reg_bit(6, 1, 3);// gain set
	
	dlv_write_reg_bit(5, 0, 5);//PMR1.SB_MIX->0
	mdelay(100);
	dlv_write_reg_bit(5, 0, 6);//PMR1.SB_OUT->0
	//dlv_write_reg(1, 0x4);
	LEAVE();
}
#endif

#if 0
/* unset Record MIC input audio with direct playback */
static void unset_record_mic_input_audio_with_direct_playback(void)
{
	ENTER();
	/* ADC path for MIC IN */
	dlv_write_reg_bit(5, 1, 4);//SB_ADC->1
	dlv_write_reg_bit(1, 1, 7);//CR1.SB_MICBIAS->1
	dlv_write_reg(22, 0xc0);//CR3.SB_MIC1
	dlv_write_reg_bit(5, 1, 6);//PMR1.SB_OUT->1
	dlv_write_reg_bit(1, 1, 5);//DAC_MUTE->1
	dlv_write_reg_bit(6, 1, 0);//SB_SLEEP->1
	dlv_write_reg_bit(6, 1, 1);//SB->1
	LEAVE();
}
#endif

#if 0
/* set Record playing audio mixed with MIC input audio */
static void set_record_playing_audio_mixed_with_mic_input_audio(void)
{
	ENTER();
	dlv_write_reg_bit(23, 0, 7);//AGC1.AGC_EN->0
	dlv_write_reg(9, 0xff);
	//dlv_write_reg(8, 0x30);
	dlv_write_reg(8, 0x20);
	
	schedule_timeout(2);
	dlv_write_reg(22, 0x63);//mic 1
	
	dlv_write_reg_bit(1, 0, 2);//CR1.BYPASS->0
	dlv_write_reg_bit(6, 1, 3);// gain set

	dlv_write_reg_bit(1, 0, 4);//CR1.HP_DIS->0	
	dlv_write_reg_bit(5, 1, 3);//PMR1.SB_LIN->1
	dlv_write_reg_bit(5, 1, 0);//PMR1.SB_IND->1
	dlv_write_reg_bit(1, 0, 7);//CR1.SB_MICBIAS->0
	dlv_write_reg_bit(22, 0, 7);//CR3.SB_MIC->0
	dlv_write_reg_bit(1, 1, 3);//CR1.DACSEL->1
	dlv_write_reg_bit(5, 0, 5);//PMR1.SB_MIX->0
	dlv_write_reg_bit(5, 0, 4);//PMR1.SB_MIX->0
	LEAVE();
}
#endif

#if 0
/* unset Record playing audio mixed with MIC input audio */
static void unset_record_playing_audio_mixed_with_mic_input_audio(void)
{
	ENTER();
	/* ADC path */
	dlv_write_reg_bit(5, 1, 4);//SB_ADC->1
	dlv_write_reg_bit(1, 1, 7);//CR1.SB_MICBIAS->1
	//dlv_write_reg_bit(1, 1, 6);//CR1.MONO->1
	dlv_write_reg(22, 0xc0);//CR3.SB_MIC1->1
	//dlv_write_reg_bit(1, 1, 5);//DAC_MUTE->1
	//dlv_write_reg_bit(5, 1, 6);//SB_OUT->1
//	dlv_write_reg_bit(5, 1, 7);//SB_DAC->1
	dlv_write_reg_bit(5, 1, 5);//SB_MIX->1
	dlv_write_reg_bit(6, 1, 0);//SB_SLEEP->1
	dlv_write_reg_bit(6, 1, 1);//SB->1
	LEAVE();
}
#endif


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
/* set Record MIC input audio with Audio data replay (full duplex) */
static void set_record_mic_input_audio_with_audio_data_replay(void)
{
	ENTER();
	printk("when run here ?????\n");
	dlv_write_reg_bit(23, 0, 7);//AGC1.AGC_EN->0
	dlv_write_reg(9, 0xff);
	//dlv_write_reg(8, 0x30);
	dlv_write_reg(8, 0x20);
	dlv_write_reg_bit(1, 0, 4);//CR1.HP_DIS->0	
	dlv_write_reg_bit(5, 1, 3);//PMR1.SB_LIN->1
	dlv_write_reg_bit(5, 1, 0);//PMR1.SB_IND->1

	dlv_write_reg_bit(22, 0, 7);//CR3.SB_MIC->0

	dlv_write_reg_bit(1, 0, 7);//CR1.SB_MICBIAS->0

	dlv_write_reg_bit(1, 1, 3);//CR1.DACSEL->1
	dlv_write_reg_bit(5, 0, 5);//PMR1.SB_MIX->0
	LEAVE();
}

/* unset Record MIC input audio with Audio data replay (full duplex) */
static void unset_record_mic_input_audio_with_audio_data_replay(void)
{
	ENTER();
	/* ADC path */
	printk("@@@ %s", __FUNCTION__);
	dlv_write_reg_bit(5, 1, 4);//SB_ADC->1
	dlv_write_reg_bit(1, 1, 7);//CR1.SB_MICBIAS->1
	//dlv_write_reg_bit(1, 1, 6);//CR1.MONO->1
	dlv_write_reg(22, 0xc0);//CR3.SB_MIC1->1
//	dlv_write_reg_bit(5, 1, 7);//SB_DAC->1
	dlv_write_reg_bit(5, 1, 5);//SB_MIX->1

	// 2009-01-20 Jason marked
//	dlv_write_reg_bit(6, 1, 0);//SB_SLEEP->1
//	dlv_write_reg_bit(6, 1, 1);//SB->1
	LEAVE();
}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
/* set Record LINE input audio with Audio data replay (full duplex for linein) */
static void set_record_line_input_audio_with_audio_data_replay(void)
{
	ENTER();
	dlv_write_reg(9, 0xff);
	//dlv_write_reg(8, 0x30);
	dlv_write_reg(8, 0x20);
	dlv_write_reg_bit(1, 0, 4);//CR1.HP_DIS->0	
	dlv_write_reg_bit(5, 0, 3);//PMR1.SB_LIN->0
	dlv_write_reg_bit(5, 1, 0);//PMR1.SB_IND->1
	dlv_write_reg_bit(1, 1, 7);//CR1.SB_MICBIAS->1
	//dlv_write_reg_bit(22, 1, 7);//CR3.SB_MIC->1
	dlv_write_reg_bit(1, 1, 3);//CR1.DACSEL->1
	dlv_write_reg_bit(5, 0, 5);//PMR1.SB_MIX->0

	dlv_write_reg(22, 0xc6);//line in 1
	dlv_write_reg_bit(23, 0, 7);//AGC1.AGC_EN->0
	dlv_write_reg_bit(1, 0, 2);//CR1.BYPASS->0
	dlv_write_reg_bit(5, 0, 5);//PMR1.SB_MIX->0
	LEAVE();
}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
/* unset Record LINE input audio with Audio data replay (full duplex for linein) */
static void unset_record_line_input_audio_with_audio_data_replay(void)
{
	ENTER();
	/* ADC path */
	printk("@@@ %s", __FUNCTION__);

	dlv_write_reg_bit(5, 1, 4);//SB_ADC->1
	dlv_write_reg_bit(1, 1, 7);//CR1.SB_MICBIAS->1
	//dlv_write_reg_bit(1, 1, 6);//CR1.MONO->1
	dlv_write_reg(22, 0xc0);//CR3.SB_MIC1->1
//	dlv_write_reg_bit(5, 1, 7);//SB_DAC->1
	dlv_write_reg_bit(5, 1, 5);//SB_MIX->1

	// 2010-01-20 Jason masked
//	dlv_write_reg_bit(6, 1, 0);//SB_SLEEP->1
//	dlv_write_reg_bit(6, 1, 1);//SB->1
	LEAVE();
}

#if 0
/* unset Audio data replay */
static void unset_audio_data_replay(void)
{
	ENTER();
	//dlv_write_reg_bit(1, 1, 5);//DAC_MUTE->1
	//mdelay(800);
	//dlv_write_reg_bit(5, 1, 6);//SB_OUT->1
	//mdelay(800);
//	dlv_write_reg_bit(5, 1, 7);//SB_DAC->1
	dlv_write_reg_bit(5, 1, 4);//SB_MIX->1
	dlv_write_reg_bit(6, 1, 0);//SB_SLEEP->1
	dlv_write_reg_bit(6, 1, 1);//SB->1
	LEAVE();
}
#endif

static int dlv_set_replay_speed(int rate)
{
	int speed = 0, val;
#define MAX_RATE_COUNT 11
	int mrate[MAX_RATE_COUNT] = { 
		96000, 48000, 44100, 32000,
		24000, 22050, 16000, 12000,
		11025, 9600 , 8000};

	for (val = 0; val < MAX_RATE_COUNT; val++) {
		if (rate >= mrate[val]) {
			speed = val;
			break;
		}
	}
	if (rate < mrate[MAX_RATE_COUNT - 1]) {
		speed = MAX_RATE_COUNT - 1;
	}
	val = dlv_read_reg(4);
	val &= 0xf;
	val = (speed << 4) | val;
	dlv_write_reg(4, val);	
	return mrate[speed];
}

static int dlv_set_record_speed(int rate)
{
	int speed = 0, val;
#define MAX_RATE_COUNT 11
	int mrate[MAX_RATE_COUNT] = { 
		96000, 48000, 44100, 32000,
		24000, 22050, 16000, 12000,
		11025, 9600 , 8000};
	for (val = 0; val < MAX_RATE_COUNT; val++) {
		if (rate >= mrate[val]) {
			speed = val;
			break;
		}
	}
	if (rate < mrate[MAX_RATE_COUNT - 1]) {
		speed = MAX_RATE_COUNT - 1;
	}
	val = dlv_read_reg(4);
	val &= 0xf0;
	val = (speed) | val;
	dlv_write_reg(4, val);	
	return mrate[speed];	
}

static void dlv_get_mixer_old_info(mixer_info *info)
{
	strncpy(info->id, "JZDLV", sizeof(info->id));
	strncpy(info->name, "Jz internal codec dlv on jz4750", sizeof(info->name));
}

static void dlv_get_mixer_info(mixer_info *old_info)
{
	strncpy(old_info->id, "JZDLV", sizeof(old_info->id));
	strncpy(old_info->name, "Jz internal codec dlv on jz4750", sizeof(old_info->name));
}

static void dlv_set_mic(int val)
{
	int cur_vol;

	ENTER();

	/* set gain */
	dlv_write_reg_bit(6, 1, 3);//GIM
	cur_vol = 31 * val / 100;
	cur_vol |= cur_vol << 4;
	dlv_write_reg(19, cur_vol);//GIL,GIR

	LEAVE();
}

static void dlv_set_line(int val)
{
	int cur_vol;

	ENTER();
	/* set gain */
	cur_vol = 31 * val / 100;
	cur_vol &= 0x1f;
	dlv_write_reg(11, cur_vol);//GO1L
	dlv_write_reg(12, cur_vol);//GO1R

	LEAVE();
}

static void dlv_set_volume(int val)
{
	unsigned long cur_vol;

	ENTER();

	/* To protect circut and to avoid shutting down CODEC,
	 * valume must less then 60% of the max
	 */
	if (val > 60) {
		val = 60;
	}
	cur_vol = 31 * (100 - val) / 100;

	dlv_write_reg(17, cur_vol | 0x80);

	DPRINT_CODEC("$$$$$ val = %d, REG_17 = 0x%02x, REG_18 = 0x%02x\n",
		     val, dlv_read_reg(17), dlv_read_reg(18));

	LEAVE();
}

/*
 * Base on set_record_mic_input_audio_without_playback()
 */
static void dlv_set_record(void)
{
	ENTER();

	//dump_dlv_regs("enter dlv_set_record");

	/* ADC path for MIC IN */
	dlv_write_reg(9, 0xff);
	dlv_write_reg(8, 0x2f);
	dlv_write_reg_bit(23, 0, 7);//AGC1.AGC_EN->0
	schedule_timeout(2);
	dlv_write_reg_bit(1, 1, 2);
	//dlv_write_reg_bit(1, 1, 6);//CR1.MONO->1
	
	dlv_write_reg(22, 0x40);//mic 1
	dlv_write_reg_bit(3, 1, 7);//CR1.HP_DIS->1
	dlv_write_reg_bit(5, 1, 3);//PMR1.SB_LIN->1
	dlv_write_reg_bit(5, 1, 0);//PMR1.SB_IND->1

	dlv_write_reg_bit(1, 0, 2);//CR1.BYPASS->0

	//2010-02-01 Jason marked
	//dlv_write_reg_bit(1, 0, 3);//CR1.DACSEL->0

	// 2010-01-31 Jason added
	//dlv_write_reg_bit(1, 0, 7);
	
	dlv_write_reg_bit(5, 0, 5);//PMR1.SB_MIX->0
	schedule_timeout(10);
	dlv_write_reg_bit(5, 0, 6);//PMR1.SB_OUT->0
	dlv_write_reg(1, 0x8);

	//2010-02-01 Jason masked
	//dlv_write_reg(19, (80 * 32 / 100) | ((80 * 32 / 100) << 4));

	// 2010-01-19 Jason added
	dlv_write_reg_bit(6, 0, 0);//SB_SLEEP->0
	dlv_write_reg_bit(6, 0, 1);//SB->0
	dlv_write_reg_bit(5, 0, 4);//SB_ADC->0

	//dump_dlv_regs("leave dlv_set_record");

	LEAVE();
}

static void dlv_set_replay_recode(int val)
{
	ENTER();
	if (val == USE_LINEIN) {
		/* Record LINE input audio with Audio data replay (full duplex for linein) */
		/* codec_test_line */
		printk("use line in ???\n");
		set_record_line_input_audio_with_audio_data_replay();
	}
	if (val == USE_MIC) {
		/* Record MIC input audio with Audio data replay (full duplex) */
		/* codec_test_mic */
		set_record_mic_input_audio_with_audio_data_replay();
	}
	LEAVE();
}

static void dlv_anti_pop(int mode)
{
	switch(mode) {
	case CODEC_WRMODE:
		//set SB_ADC or SB_DAC
		dlv_write_reg_bit(5, 0, 6);//PMR1.SB_OUT->0

		//2010-01-31 Jason add
		dlv_write_reg(22, 0x40);//mic 1

		//2010-01-31 Jason add
		//dlv_write_reg(1, 0x04);

		schedule_timeout(28); //280 ms 
		break;
	case CODEC_RMODE:
		// 2010-01-31 Jason marked
		//dlv_write_reg_bit(5, 1, 7);//SB_DAC->1

		//2010-01-31 Jason add
		dlv_write_reg(22, 0x40);//mic 1

		break;
	case CODEC_WMODE:
		printk("===>dlv_anti_pop!!!\n");
		dlv_write_reg_bit(6, 0, 2);     //codec_reg_clear(A_CODEC_PMR2, SB_MC);
		mdelay(5);
		dlv_write_reg_bit(6, 0, 1);     //codec_reg_clear(A_CODEC_PMR2, SB);
		mdelay(30);
		dlv_write_reg_bit(6, 0, 0);     //codec_reg_clear(A_CODEC_PMR2, SB_SLEEP);
		mdelay(1);
		dlv_write_reg_bit(5, 0, 7);     //codec_reg_clear(A_CODEC_PMR1, SB_DAC);
		mdelay(1);
		dlv_write_reg_bit(5, 0, 6);     //codec_reg_clear(A_CODEC_PMR1, SB_OUT);
		mdelay(1);
		dlv_write_reg_bit(5, 0, 5);     //codec_reg_clear(A_CODEC_PMR1, SB_MIX);
		
		msleep(350);
		break;
	}
}

static void dlv_turn_replay(int mode)
{
	ENTER();
	if (mode == USE_LINEIN) {
		unset_record_line_input_audio_with_audio_data_replay();
	}
	if (mode == USE_MIC) {
		unset_record_mic_input_audio_with_audio_data_replay();
	}
	LEAVE();
}

static void dlv_turn_off(int mode)
{
	ENTER();

	if ((mode & REPLAY) && (mode & RECORD)) {
		printk("Close DLV !!!\n");
		dlv_write_reg_bit(1, 1, 5);//DAC_MUTE->1
		schedule_timeout(20);

		// 2010-01-31 Jason marked
		//dlv_write_reg_bit(5, 1, 6);//SB_OUT->1

		dlv_write_reg(9, 0xff);
		dlv_write_reg(8, 0x2f);
	} else if (mode & REPLAY) {
		//nothing
	} else if (mode & RECORD) {
		printk("Close RECORD\n");
		dlv_write_reg(4, 0x20);
	}

	LEAVE();
}

static int dlv_set_channel(int ch)
{
	if(ch > 2) ch = 2;
	if(ch < 1) ch = 1;
	switch (ch) {
	case 1:
		dlv_write_reg_bit(1, 1, 6);//CR1.MONO->1 for Mono
		break;
	case 2:
		dlv_write_reg_bit(1, 0, 6);//CR1.MONO->0 for Stereo
		break;
	}
	return ch;
}

static int dlv_set_data_width(unsigned int mode, unsigned int width)
{
	unsigned char cr2 = dlv_read_reg(2);
	unsigned char savecr2 = cr2;
	int supported_width[4] = {16, 18, 20, 24};
	int i;

	for (i = 0; i < (sizeof(supported_width) / sizeof(supported_width[0])); i++) {
		if (supported_width[i] <= width) {
			break;
		}
	}

	if (i == (sizeof(supported_width) / sizeof(supported_width[0]))) {
		// For 8 bit width mode, handle it as 16 bit
		if (width == 8) {
			i = 0;
		} else {
			return -1;
		}
	}

	//printk("mode = %d, width = %d, selected %d\n", mode, width, i);

	switch (mode) {
	case RECORD:
		cr2 &= ~(3 << 3);
		cr2 |= (i << 3);
		break;
	case REPLAY:
		cr2 &= ~(3 << 5);
		cr2 |= (i << 5);
		break;
	}

	if (cr2 != savecr2) {
		dlv_write_reg(2, cr2);
	}

	//printk("set cr2 = %x, %x\n", cr2, savecr2);

	if (width == 8) {
		return 8;
	} else {
		return supported_width[i];
	}
}

static int dlv_mute(int val)
{
	return dlv_write_reg_bit(1, val ? 1 : 0, 5);
}
static void dlv_suspend(void)
{    
	printk("suspend\n");
	dlv_write_reg_bit(5, 1, 6);     //codec_reg_set(A_CODEC_PMR1, SB_OUT);
	mdelay(40);
	dlv_write_reg_bit(6, 1, 1);     //codec_reg_set(A_CODEC_PMR2, SB);
	dlv_write_reg_bit(6, 1, 0);     //codec_reg_set(A_CODEC_PMR2, SB_SLEEP);
	mdelay(30);
	dlv_write_reg_bit(5, 1, 5);     //codec_reg_set(A_CODEC_PMR1, SB_MIX);
	dlv_write_reg_bit(5, 1, 7);     //codec_reg_set(A_CODEC_PMR1, SB_DAC);
	dlv_write_reg_bit(5, 1, 4);     //codec_reg_set(A_CODEC_PMR1, SB_ADC);
}

static void dlv_resume(void)
{     
	dlv_write_reg_bit(6, 0, 2);     //codec_reg_clear(A_CODEC_PMR2, SB_MC);
	mdelay(5);
	dlv_write_reg_bit(6, 0, 1);     //codec_reg_clear(A_CODEC_PMR2, SB);
	mdelay(30);
	dlv_write_reg_bit(6, 0, 0);     //codec_reg_clear(A_CODEC_PMR2, SB_SLEEP);
	mdelay(1);
	dlv_write_reg_bit(5, 0, 7);     //codec_reg_clear(A_CODEC_PMR1, SB_DAC);	p380
	mdelay(1);
	dlv_write_reg_bit(5, 0, 6);     //codec_reg_clear(A_CODEC_PMR1, SB_OUT);
	mdelay(1);
	dlv_write_reg_bit(5, 0, 5);     //codec_reg_clear(A_CODEC_PMR1, SB_MIX);
	msleep(350);
 }


void dump_dlv_regs(const char * str)
{
	unsigned int i;
	unsigned char dat;
	printk("codec register, %s:\n", str);
	for (i = 0; i < 27; i++) {
		dat = dlv_read_reg(i);
		printk("addr = %2d data = 0x%02x\n", i, dat);
	}
}

static int jzdlv_ioctl(void *context, unsigned int cmd, unsigned long arg)
{
	ENTER();
	DUMP_CODEC_REGS(__FUNCTION__);
	DPRINT_CODEC("[dlv IOCTL]++++++++++++++++++++++++++++\n");
	DPRINT_CODEC("%s  cmd = %d, arg = %lu\n", __FUNCTION__, cmd, arg);
	DPRINT_DLV_IOC_CMD(cmd);
	DPRINT_CODEC("[dlv IOCTL]----------------------------\n");

	switch (cmd) {
	case CODEC_SET_MODE:
		dlv_set_mode();
		break;

	case CODEC_SET_STARTUP_PARAM:
		dlv_set_startup_param();
		break;

	case CODEC_SET_REPLAY:
		dlv_set_replay();
		break;

	case CODEC_SET_RECORD:
		dlv_set_record();
		break;

	case CODEC_SET_REPLAY_RECORD:
		dlv_set_replay_recode(arg);
		break;

	case CODEC_SET_VOLUME:
		dlv_set_volume(arg);
		break;

	case CODEC_SET_MIC:
		dlv_set_mic(arg);
		break;

	case CODEC_SET_LINE:
		dlv_set_line(arg);
		break;

	case CODEC_EACH_TIME_INIT:
		dlv_each_time_init();
		break;

	case CODEC_RESET:
		dlv_reset();
		break;

	case CODEC_ANTI_POP:
		dlv_anti_pop(arg);
		break;

	case CODEC_TURN_REPLAY:
		dlv_turn_replay(arg);
		break;

	case CODEC_TURN_OFF:
		dlv_turn_off(arg);
		break;

	case CODEC_GET_MIXER_INFO:
		dlv_get_mixer_info((mixer_info *)arg);
		break;

	case CODEC_GET_MIXER_OLD_INFO:
		dlv_get_mixer_old_info((mixer_info *)arg);
		break;

	case CODEC_SET_REPLAY_SPEED:
		return dlv_set_replay_speed(arg);

	case CODEC_SET_RECORD_SPEED:
		return dlv_set_record_speed(arg);

	case CODEC_SET_RECORD_CHANNEL:
		return arg;

	case CODEC_SET_REPLAY_CHANNEL:
		return dlv_set_channel(arg);

	case CODEC_SET_RECORD_DATA_WIDTH:
		return dlv_set_data_width(RECORD, arg);

	case CODEC_SET_REPLAY_DATA_WIDTH:
		return dlv_set_data_width(REPLAY, arg);

	case CODEC_DAC_MUTE:
		return dlv_mute(arg);

	case CODEC_I2S_SUSPEND:
		dlv_suspend();
		break;
	case CODEC_I2S_RESUME:
		dlv_resume();
		break;

	default:
		printk("%s:%d no support\n", __FUNCTION__, __LINE__);
		return -1;
	}

	LEAVE();
	return 0;
}

static struct work_struct dlv_work;

/*
 * work handler
 *
 * Mission:
 *	Restart CODEC after shut down by short circurt protection
 */
static void dlv_work_handle(struct work_struct *work)
{
	printk("CODEC: short circurt detected!\n");

	/* Renable SB OUT */
	switch_SB_OUT(POWER_OFF);
	mdelay(300);
	while ((dlv_read_reg(9) & 0x4) != 0x4) {
		;/* nothing */
	}
	while ((dlv_read_reg(9) & 0x10) == 0x10) {
		dlv_write_reg(9, 0x10);
	}
	switch_SB_OUT(POWER_ON);
	mdelay(300);
	while ((dlv_read_reg(9) & 0x8) != 0x8) {
		;/* nothing */
	}

	/* Enable CCMC interrupt ... clear bit 4*/
	dlv_write_reg(8, 0x2f);
}

static spinlock_t dlv_irq_lock;

static irqreturn_t dlv_codec_irq(int irq, void *dev_id)
{
	unsigned char reg_9;

	spin_lock(dlv_irq_lock);

	/* Clear interrupt flag */
	reg_9 = dlv_read_reg(9);
	dlv_write_reg(9, reg_9);

	/* Mask CCMC temporarily */
	dlv_write_reg(8, 0x3f);

	REG_AIC_SR = 0x78; //???

	/* Start work when output short circuit has been detected */
	if ((reg_9 & 0x10) == 0x10) {
		schedule_work(&dlv_work);
	}

/*
	reg_9 = dlv_read_reg(9);
	reg_8 = dlv_read_reg(8);
	printk("reg_8 = %x, reg_9 = %x\n", reg_8, reg_9);
*/
	spin_unlock(dlv_irq_lock);
	return IRQ_HANDLED;
}

static int __init init_dlv(void)
{
	int retval;

	spin_lock_init(&dlv_irq_lock);
	INIT_WORK(&dlv_work, dlv_work_handle);
	register_jz_codecs((void *)jzdlv_ioctl);
	dlv_reset();

	retval = request_irq(IRQ_AIC, dlv_codec_irq, IRQF_DISABLED, "dlv_codec_irq", NULL);
	if (retval) {
		printk("Could not get aic codec irq %d\n", IRQ_AIC);
		return retval;
	}

	return 0;
}

static void __exit cleanup_dlv(void)
{
	free_irq(IRQ_AIC, NULL);
}

module_init(init_dlv);
module_exit(cleanup_dlv);
