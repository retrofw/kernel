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
#include <linux/platform_device.h>

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

#include "jz_codec.h"
#include "jz4770_dlv.h"
#include "jz_i2s_dbg.h"

static void dlv_set_line(int val);

// Hanvon
//static int spk_state;//add by wll

//#define ANTI_POP_TIMER
#define ANTI_POP_WORK_STRUCT

#ifdef ANTI_POP_WORK_STRUCT
  #ifdef ANTI_POP_TIMER
    #error "Error: both ANTI_POP_TIMER and ANTI_POP_TIMER are defined."
  #endif
#endif

#ifndef ANTI_POP_WORK_STRUCT
  #ifndef ANTI_POP_TIMER
    #error "Error: both ANTI_POP_TIMER and ANTI_POP_TIMER are undefined."
  #endif
#endif

#if 0
#define switch_SB_LINE_IN(pwrstat)                                      \
do {                                                            \
        dlv_write_reg_bit(DLV_REG_PMR1, pwrstat, PMR1_SB_LINE); \
                                                                \
} while (0)

#define switch_SB_ADC(pwrstat)                                  \
do {                                                            \
        dlv_write_reg_bit(DLV_REG_PMR2, pwrstat, PMR2_SB_ADC);  \
} while (0)

#define switch_SB_MIC1(pwrstat)                                 \
do {                                                            \
        dlv_write_reg_bit(DLV_REG_PMR1, pwrstat, PMR1_SB_MIC1); \
        dlv_write_reg_bit(DLV_REG_PMR1, pwrstat, PMR1_SB_MIC2); \
        dlv_write_reg_bit(DLV_REG_PMR1, pwrstat, PMR1_SB_MICBIAS);\
                                                                \
} while (0)
#endif

enum device_t {
	SND_DEVICE_DEFAULT = 0,
	SND_DEVICE_CURRENT,
	SND_DEVICE_HANDSET,
	SND_DEVICE_HEADSET,
	SND_DEVICE_SPEAKER,
	SND_DEVICE_BT,
	SND_DEVICE_BT_EC_OFF,
	SND_DEVICE_HEADSET_AND_SPEAKER,
	SND_DEVICE_TTY_FULL,
	SND_DEVICE_CARKIT,
	SND_DEVICE_FM_SPEAKER,
	SND_DEVICE_FM_HEADSET,
	SND_DEVICE_NO_MIC_HEADSET,
	SND_DEVICE_COUNT
};

static unsigned int g_current_out_dev = 0;
static unsigned int g_bluetooth_enabled = 0;
static struct semaphore *g_dlv_sem = 0;
static int greplay_volume;

static spinlock_t g_dlv_sem_lock;

#ifdef CONFIG_JZ4760_PT701
extern int gstate_hp;
extern int gstate_dock;
#endif

#define DLV_DEBUG_SEM(x,y...)
//printk(x,##y);

#define DLV_LOCK()								\
	do{											\
		spin_lock(&g_dlv_sem_lock);				\
		if(g_dlv_sem)							\
			down(g_dlv_sem);					\
		spin_unlock(&g_dlv_sem_lock);			\
		DLV_DEBUG_SEM("dlvsemlock lock\n");		\
	}while(0)

#define DLV_UNLOCK()							\
	do{											\
		spin_lock(&g_dlv_sem_lock);				\
		if(g_dlv_sem)							\
			up(g_dlv_sem);						\
		spin_unlock(&g_dlv_sem_lock);			\
		DLV_DEBUG_SEM("dlvsemlock unlock\n");		\
	}while(0)

#define DLV_LOCKINIT()							\
	do{											\
		spin_lock(&g_dlv_sem_lock);										\
		if(g_dlv_sem == NULL)											\
			g_dlv_sem = (struct semaphore *)vmalloc(sizeof(struct semaphore)); \
		if(g_dlv_sem)													\
			init_MUTEX_LOCKED(g_dlv_sem);								\
		spin_unlock(&g_dlv_sem_lock);									\
		DLV_DEBUG_SEM("dlvsemlock init\n");									\
	}while(0)

#define DLV_LOCKDEINIT()						\
	do{											\
		spin_lock(&g_dlv_sem_lock);				\
		if(g_dlv_sem)							\
			vfree(g_dlv_sem);					\
		g_dlv_sem = NULL;						\
		spin_unlock(&g_dlv_sem_lock);			\
		DLV_DEBUG_SEM("dlvsemlock deinit\n");		\
	}while(0)

static int should_up = 0;
#ifdef HP_SENSE_DETECT
static jz_hp_switch_data_t *g_switch_data = NULL;
#endif
/*
static unsigned int g_prev_dev = 0;
static unsigned int g_codec_mode = 0;
static unsigned int g_speaker_mute = 0;
static unsigned int g_receiver_mute = 0;
static unsigned int g_mic1_mute = 0;
static unsigned int g_mic2_mute = 0;
static unsigned int g_volumes[SND_DEVICE_COUNT];
*/

/* Audio route ops */
static void dlv_anti_pop_part(void);
 void dlv_enable_hp_out(void);
 void dlv_disable_hp_out(void);
static void dlv_enable_receiver(void);
static void dlv_disable_receiver(void);
 void dlv_enable_line_out(void);
 void dlv_disable_line_out(void);
static void dlv_enable_line_in_record(int insel);
static void dlv_set_line_in(void);
static void dlv_disable_line_in_record(void);
static void dlv_enable_line_in_bypass_hp(void);
static void dlv_disable_line_in_bypass_hp(void);
static void dlv_enable_mic_1(void);
static void dlv_disable_mic_1(void);
static void dlv_enable_mic_2(void);
static void dlv_disable_mic_2(void);
#ifdef CONFIG_JZ4760_PT701
static void dlv_set_device(struct snd_device_config *snd_dev_cfg);
#endif
static void board_set_record(void);

#if defined(IOC_DEBUG) || defined(ROUTETE_DEBUG)
static void dlv_print_ioc_cmd(int cmd)
{
	char *dlv_ioc_cmd[] = {
		"CODEC_SET_MODE",		"CODEC_CLEAR_MODE",		"CODEC_SET_GPIO_PIN",
		"CODEC_EACH_TIME_INIT",		"CODEC_SET_STARTUP_PARAM",	"CODEC_SET_VOLUME_TABLE",
		"CODEC_SET_RECORD",		"CODEC_SET_REPLAY",		"CODEC_SET_REPLAY_RECORD",
		"CODEC_TURN_ON",		"CODEC_TURN_OFF",		"CODEC_SET_REPLAY_RATE",
		"CODEC_RESET",			"CODEC_GET_MIXER_OLD_INFO",	"CODEC_GET_MIXER_INFO",
		"CODEC_SET_BASS",		"CODEC_SET_REPLAY_VOLUME",	"CODEC_SET_MIC_VOLUME",
		"CODEC_SET_LINE",		"CODEC_I2S_RESUME",		"CODEC_I2S_SUSPEND",
		"CODEC_PIN_INIT",		"CODEC_SET_SOME_FUNC",		"CODEC_CLEAR_RECORD",
		"CODEC_CLEAR_REPLAY",		"CODEC_SET_REPLAY_HP_OR_SPKR",	"CODEC_SET_DIRECT_MODE",
		"CODEC_CLEAR_DIRECT_MODE",	"CODEC_SET_LINEIN2HP",		"CODEC_CLEAR_LINEIN2HP",
		"CODEC_ANTI_POP",		"CODEC_TURN_REPLAY",		"CODEC_SET_REPLAY_CHANNEL",
		"CODEC_SET_REPLAY_FORMAT",	"CODEC_SET_RECORD_CHANNEL",	"CODEC_SET_RECORD_FORMAT",
		"CODEC_SET_RECORD_RATE",	"CODEC_DAC_MUTE",		"CODEC_SET_LINEIN2BTL",
		"CODEC_CLEAR_LINEIN2BTL",	"CODEC_SET_DEVICE",		"CODEC_MUTE_DEVICE"
	};

	if (cmd >= (sizeof(dlv_ioc_cmd) / sizeof(dlv_ioc_cmd[0]))) {
		printk("%s: Unkown command !\n", __FUNCTION__);
	} else {
		printk("IOC CMD NAME = %s\n", dlv_ioc_cmd[cmd - 1]);
	}
}
#endif


/*
 * CODEC registers access routines
 */

/**
 * CODEC read register
 *
 * addr:	address of register
 * return:	value of register
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
 * addr:	address of register
 * val:		value to set
 */
void dlv_write_reg(int addr, int val)
{
	volatile int reg;

	while (__icdc_rgwr_ready()) {
		;//nothing...
	}
	REG_ICDC_RGADW = ICDC_RGADW_RGWR | ((addr << ICDC_RGADW_RGADDR_LSB) | val);
	//__icdc_set_rgwr();
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

static inline void dlv_sleep_wait_bitset(int reg, unsigned bit_mask, int stime, int line)
{
	while(!(dlv_read_reg(reg) & bit_mask)) {
		msleep(stime);
	}
}

static inline void dlv_sleep_wait_bitclear(int reg, unsigned bit_mask, int stime)
{
	while((dlv_read_reg(reg) & bit_mask))
		msleep(stime);
}

/**
 * CODEC write a bit of a register
 *
 * addr:	address of register
 * bitval:	bit value to modifiy
 * mask_bit:	indicate which bit will be modifiy
 */
static int dlv_write_reg_bit(int addr, int bitval, int bit_mask)
{
	int val = dlv_read_reg(addr);

	if (bitval)
		val |= bit_mask;
	else
		val &= ~bit_mask;
	dlv_write_reg(addr, val);

	return 1;
}

static int dlv_update_reg(int addr,  int val, int mask) {
	int old_val = dlv_read_reg(addr);

	old_val &= ~mask;
	old_val |= (val & mask);

	dlv_write_reg(addr, old_val);

	return 1;
}

/*
 * DLV CODEC operations routines
 */

static inline void turn_on_dac(int timeout)
{
	if(__dlv_get_dac_mute()){
		/* clear IFR_GUP */
		__dlv_set_irq_flag(DLC_IFR_GUP);
		mdelay(300);
		/* turn on dac */
		__dlv_disable_dac_mute();
		/* wait IFR_GUP set */
		dlv_sleep_wait_bitset(DLC_IFR, DLC_IFR_GUP, timeout,__LINE__);
		udelay(100);  /* 4 SYS_CLK cycles */
		__dlv_set_irq_flag(DLC_IFR_GUP);
	}
}

static inline void turn_off_dac(int timeout)
{
 	if (!(__dlv_get_dac_mute())){
		/* clear IFR_GDO */
		__dlv_set_irq_flag(DLC_IFR_GDO);
		/* turn off dac */
		__dlv_enable_dac_mute();
		/* wait IFR_GDO set */
		dlv_sleep_wait_bitset(DLC_IFR, DLC_IFR_GDO, timeout,__LINE__);
		udelay(100);
		__dlv_set_irq_flag(DLC_IFR_GDO);
	}
}

static inline void turn_on_sb_hp(void)
{
	if (__dlv_get_sb_hp() != POWER_ON){
		/* clear IFR_RUP */
		__dlv_set_irq_flag(DLC_IFR_RUP);
		/* turn on sb_hp */
		__dlv_set_cap_less();
        	__dlv_switch_sb_hp(POWER_ON);
		/* wait IFR_RUP set */
        	dlv_sleep_wait_bitset(DLC_IFR, DLC_IFR_RUP, 100,__LINE__);
		udelay(100);
		__dlv_set_irq_flag(DLC_IFR_RUP);
	}

}
static inline void turn_off_sb_hp(void)
{
	if (__dlv_get_sb_hp() != POWER_OFF){
		/* clear IFR_RDO */
		__dlv_set_irq_flag(DLC_IFR_RDO);
		/* turn off sb_hp */
		__dlv_set_cap_couple();
        	__dlv_switch_sb_hp(POWER_OFF);
		/* wait IFR_RDO set */
        	dlv_sleep_wait_bitset(DLC_IFR, DLC_IFR_RDO, 100,__LINE__);
		udelay(100);
		__dlv_set_irq_flag(DLC_IFR_RUP);
	}
}
void dump_aic_regs2(const char *str)
{
	char *regname[] = {"aicfr","aiccr","aiccr1","aiccr2","i2scr","aicsr","acsr","i2ssr"};
	int i;
	unsigned int addr;

	printk("AIC regs dump, %s\n", str);
	for (i = 0; i < 0x1c; i += 4) {
		addr = 0xb0020000 + i;
		printk("%s\t0x%08x -> 0x%08x\n", regname[i/4], addr, *(unsigned int *)addr);
	}
}


static void dlv_shutdown(void) {
	unsigned long start_time = jiffies;
#if 0			      /* Hanvon */
	spk_state=__gpio_get_pin(GPIO_SPK_SHUD);
	__gpio_clear_pin(GPIO_SPK_SHUD);
#endif
	turn_off_dac(5);
	// wll@20101020
	//dump_dlv_regs(__FUNCTION__);
	//dump_aic_regs2(__FUNCTION__);
        __aic_write_tfifo(0x0);
        __aic_write_tfifo(0x0);
	__i2s_enable_replay();
	msleep(1);
	//__i2s_disable_replay();
	turn_off_sb_hp();
	__dlv_switch_sb_line_out(POWER_OFF);
	mdelay(1);
	__dlv_enable_hp_mute();
 	mdelay(1);
	__dlv_switch_sb_dac(POWER_OFF);

	DPRINT_CODEC("CODEC shutdown finish in %d jiffies! pmr1 = 0x%02x, pmr2 = 0x%02x\n",
		     (jiffies - start_time), dlv_read_reg(DLV_REG_PMR1), dlv_read_reg(DLV_REG_PMR2));
}

static void dlv_each_time_init(void)
{
	ENTER();
	__i2s_disable();
	__i2s_as_slave();
	__aic_internal_codec();
	LEAVE();
}

static void dlv_init(void)
{
	ENTER();
	DLV_LOCKINIT();

#ifdef CONFIG_JZ4760_PT701
	/* disable speaker output */
	__gpio_as_output(GPIO_SPK_EN);
	__gpio_clear_pin(GPIO_SPK_EN);
	/* disable external speaker output */
	__gpio_as_output(GPIO_DOCK_SPK_EN);
	__gpio_clear_pin(GPIO_DOCK_SPK_EN);
#endif
	__dlv_switch_sb_micbias(POWER_ON);
	__dlv_set_int_form(DLC_ICR_INT_HIGH);

	__dlv_set_irq_mask(ICR_COMMON_MASK);
	__dlv_set_irq_flag(0x7f);

	__dlv_set_12m_crystal();
	//__dlv_set_10kohm_load(); //1uF
	__dlv_set_16ohm_load(); // 220uF

        dlv_anti_pop_part();

	g_current_out_dev = SND_DEVICE_SPEAKER;

	__dlv_set_irq_flag(IFR_ALL_FLAG);
	LEAVE();
}

/**
 * Set replay rate
 *
 * rate:	replay rate to set
 * return:	replay rate after set
 */
static int dlv_set_replay_rate(int rate)
{
	int speed = 0, val;

	int mrate[] = {
		96000, 48000, 44100, 32000, 24000,
		22050, 16000, 12000, 11025, 8000
	};

	for (val = 0; val < ARRAY_SIZE(mrate); val++) {
		if (rate >= mrate[val]) {
			speed = val;
			break;
		}
	}
	if (rate < mrate[ARRAY_SIZE(mrate) - 1]) {
		speed = ARRAY_SIZE(mrate) - 1;
	}

	__dlv_set_dac_sample_rate(speed);

	return mrate[speed];
}


/**
 * Set record rate
 *
 * rate:	record rate to set
 * return:	record rate after set
 */
static int dlv_set_record_rate(int rate)
{
	int speed = 0, val;

	int mrate[] = {
		96000, 48000, 44100, 32000, 24000,
		22050, 16000, 12000, 11025, 8000
	};

	for (val = 0; val < ARRAY_SIZE(mrate); val++) {
		if (rate >= mrate[val]) {
			speed = val;
			break;
		}
	}
	if (rate < mrate[ARRAY_SIZE(mrate) - 1]) {
		speed = ARRAY_SIZE(mrate) - 1;
	}

	__dlv_set_adc_sample_rate(speed);

	return mrate[speed];
}

static void dlv_get_mixer_old_info(mixer_info *info)
{
	strncpy(info->id, "JZDLV", sizeof(info->id));
	strncpy(info->name, "Jz internal codec dlv on jz4760", sizeof(info->name));
}

static void dlv_get_mixer_info(mixer_info *old_info)
{
	strncpy(old_info->id, "JZDLV", sizeof(old_info->id));
	strncpy(old_info->name, "Jz internal codec dlv on jz4760", sizeof(old_info->name));
}


//static void dlv_set_replay_volume(int val);

static int first_from_init_i2s = 1;
static void dlv_turn_off(int mode)
{
	ENTER();

	if (mode & REPLAY) {

		//printk("JZ DLV: Close REPLAY\n");
		//dlv_disable_receiver();
		//dlv_shutdown();
		//dlv_set_replay_volume(0);
		__dlv_enable_hp_mute();
		udelay(500);
		turn_off_dac(5);
		if (!first_from_init_i2s) {
			__aic_write_tfifo(0x0);
			__aic_write_tfifo(0x0);
		}
		first_from_init_i2s = 0;
	}

	if (mode & RECORD) {
		//printk("JZ DLV: Close RECORD\n");
		dlv_disable_mic_1();
		dlv_disable_mic_2();
	}
	DLV_LOCKDEINIT();
	LEAVE();
}

static int dlv_set_channel(int ch)
{
	ch = (ch >= 2) + 1;

	switch (ch) {
	case 1:
		// MONO->1 for Mono
		__dlv_enable_dac_mono();
		break;
	case 2:
		// MONO->0 for Stereo
		__dlv_disable_dac_mono();
		break;
	}

	return ch;
}

static int dlv_set_data_width(unsigned int mode, unsigned int width)
{
	unsigned char aicr = dlv_read_reg(DLC_AICR_DAC);
	unsigned char aicr_bak = aicr;
	int supported_width[] = {16, 18, 20, 24};
	int wd = -1;
	int i;

	if (width < 16)
		wd = 16;
	else if (width > 24)
		wd = 24;
	else {
		for (i = 0; i < ARRAY_SIZE(supported_width); i++) {
			if (supported_width[i] <= width)
				wd = supported_width[i];
			else
				break;
		}
	}

	if (mode == REPLAY) {
		switch(wd) {
		case 16:
			__dlv_dac_16bit_sample();
			break;
		case 18:
			__dlv_dac_18bit_sample();
			break;
		case 20:
			__dlv_dac_20bit_sample();
			break;
		case 24:
			__dlv_dac_24bit_sample();
			break;
		default:
			;
		}
	} else {
		switch(wd) {
		case 16:
			__dlv_adc_16bit_sample();
			break;
		case 18:
			__dlv_adc_18bit_sample();
			break;
		case 20:
			__dlv_adc_20bit_sample();
			break;
		case 24:
			__dlv_adc_24bit_sample();
			break;
		default:
			;
		}
	}

	return wd;
}

static int dlv_mute(int val)
{
	if (val)
		turn_off_dac(10);
	else
		turn_on_dac(10);
	return 0;
}

void dump_dlv_regs(const char * str)
{
	unsigned int i;
	unsigned char data;
	printk("codec register dump, %s:\n", str);
	for (i = 0; i < 34; i++) {
		data = dlv_read_reg(i);
		printk("address = 0x%02x, data = 0x%02x\n", i, data);
	}
}


void board_set_record(void)
{
#ifdef Z800_CONFIG
#endif

#ifdef CONFIG_JZ4760_ALTAIR
	dlv_enable_mic_1();
#endif

#ifdef CONFIG_JZ4760_PT701
	dlv_enable_mic_1();
#endif

	dlv_enable_mic_1();
	//dlv_enable_mic_2();
	//dlv_enable_line_in_record(0);
}

/********************************************************************************
 *				Anti-pop					*
 ********************************************************************************/

#ifdef ANTI_POP_WORK_STRUCT
static struct work_struct dlv_anti_pop_work;
#endif

static int first_start = 1;

static void dlv_anti_pop_part(void)
{
	unsigned start_time = jiffies;

	if (first_start) {
		first_start = 0;
		__dlv_switch_sb(POWER_ON);
		mdelay(300);

		__dlv_switch_sb_sleep(POWER_ON);
		mdelay(400);
	}

	__dlv_switch_sb_dac(POWER_ON);
	udelay(500);

	//__dlv_enable_hp_mute();
	__dlv_disable_hp_mute();
	mdelay(1);

	turn_on_sb_hp();
	__dlv_switch_sb_line_out(POWER_ON);

	mdelay(1);
}

/**
 * Work handler for dlv_anti_pop_work
 *
 * Perform an anti-pop startup sequence with msleep (block operation).
 */

static void dlv_anti_pop_work_handler(struct work_struct *work)
{
	dlv_anti_pop_part();
	turn_on_dac(10);
	__dlv_disable_hp_mute();

#if 0
	if(spk_state==1)
		__gpio_set_pin(GPIO_SPK_SHUD);
#endif

#ifdef CONFIG_JZ4760_PT701
	/* if headphone not attached */
	if (!gstate_hp) {
		if (gstate_dock == 1) {
			printk("pt701 audio select external speaker!");
			/* disable internal speaker output */
			__gpio_as_output(GPIO_SPK_EN);
			__gpio_clear_pin(GPIO_SPK_EN);
			/* enable external speaker output */
			__gpio_as_output(GPIO_DOCK_SPK_EN);
			__gpio_set_pin(GPIO_DOCK_SPK_EN);
		} else {
			printk("pt701 audio select internal speaker!");
			/* enable internal speaker output */
			__gpio_as_output(GPIO_SPK_EN);
			__gpio_set_pin(GPIO_SPK_EN);
			/* disable external speaker output */
			__gpio_as_output(GPIO_DOCK_SPK_EN);
			__gpio_clear_pin(GPIO_DOCK_SPK_EN);
		}
	}
#endif
//	up(&g_dlv_sem_lock);

	DLV_UNLOCK();
}

#ifdef ANTI_POP_TIMER

#define ACTION_COUNT			6
/* Time consuming (jiffie is 10ms) */

#if 1

#define ACTION_0_DELAY_TIME		1
#define ACTION_1_DELAY_TIME		1
#define ACTION_2_DELAY_TIME		1
#define ACTION_3_DELAY_TIME		1
#define ACTION_4_DELAY_TIME		1
#define ACTION_5_DELAY_TIME		50

#else

#define ACTION_0_DELAY_TIME		25
#define ACTION_1_DELAY_TIME		2
#define ACTION_2_DELAY_TIME		2
#define ACTION_3_DELAY_TIME		2
#define ACTION_4_DELAY_TIME		30

#endif

struct jz_anti_pop_timer_t {
	struct timer_list timer;
	int cur_startup_action;
	void (*dlv_anti_pop_actions[ACTION_COUNT])(void);
	int delay_time[ACTION_COUNT];
};

static inline int get_action_delay_time(struct jz_anti_pop_timer_t* apt)
{
	return apt->delay_time[apt->cur_startup_action];
}

static inline void dlv_anti_pop_action_0(void)
{
//	__dlv_set_irq_mask(ICR_COMMON_MASK);
	__dlv_switch_sb(POWER_ON);
}

static inline void dlv_anti_pop_action_1(void)
{
	__dlv_switch_sb_sleep(POWER_ON);
}

static inline void dlv_anti_pop_action_2(void)
{
	__dlv_switch_sb_dac(POWER_ON);
}

static inline void dlv_anti_pop_action_3(void)
{
	__dlv_enable_hp_mute();
}

static inline void dlv_anti_pop_action_4(void)
{
#if 1
	if (__dlv_get_sb_hp() != POWER_ON){
		__dlv_switch_sb_hp(POWER_ON);
//		dlv_sleep_wait_bitset(0xb, IFR_RUP, 100,__LINE__);
	}

#else
	if (__dlv_get_sb_hp() != POWER_ON){
		__dlv_switch_sb_hp(POWER_ON);
		dlv_sleep_wait_bitset(0xb, IFR_RUP, 100,__LINE__);
	}
	__dlv_reset_rup();

	if(__dlv_get_dac_mute()){
		__dlv_disable_dac_mute();
		dlv_sleep_wait_bitset(0xb, IFR_GUP, 100,__LINE__);
	}
	__dlv_reset_gup();
#endif
}

static inline void dlv_anti_pop_action_5(void)
{
	while(!(dlv_read_reg(0x0b) & (1 << IFR_RUP))) {
		printk("DLV delay waiting IFR_RUP \n");
		mdelay(10);
	}
}

static struct jz_anti_pop_timer_t anti_pop_timer = {
	.cur_startup_action = 0,
	.dlv_anti_pop_actions = {
		dlv_anti_pop_action_0,
		dlv_anti_pop_action_1,
		dlv_anti_pop_action_2,
		dlv_anti_pop_action_3,
		dlv_anti_pop_action_4,
		dlv_anti_pop_action_5
	},
	.delay_time = {
		ACTION_0_DELAY_TIME,
		ACTION_1_DELAY_TIME,
		ACTION_2_DELAY_TIME,
		ACTION_3_DELAY_TIME,
		ACTION_4_DELAY_TIME,
		ACTION_5_DELAY_TIME
	}
};

static void dlv_timer_callback(void)
{
	//printk("%s, startup action = %d\n", __FUNCTION__, anti_pop_timer.cur_startup_action);

	int action_number = anti_pop_timer.cur_startup_action++;

	if (action_number < ACTION_COUNT) {
		anti_pop_timer.dlv_anti_pop_actions[action_number]();
		mod_timer(&anti_pop_timer.timer, jiffies + anti_pop_timer.delay_time[action_number]);


	} else if (action_number == ACTION_COUNT) {
		del_timer_sync(&anti_pop_timer.timer);
		anti_pop_timer.cur_startup_action = 0;

		__dlv_disable_dac_mute();

		__dlv_disable_hp_mute();
		/*ulock*/
		DLV_UNLOCK();
	} else {
		printk("JZ DLV: startup state error, cur_startup_action = %d\n", anti_pop_timer.cur_startup_action);
	}

}
#endif

/**
 * Start startup anti-pop sequence.
 *
 * This is unblock operation that implement by using timer or work struct.
 *
 * Typically, for resuming from sleep.
 */
static void dlv_enable_hp_without_pop(void)
{
#ifdef ANTI_POP_TIMER
	init_timer(&anti_pop_timer.timer);
	anti_pop_timer.cur_startup_action = 0;
	anti_pop_timer.timer.function = dlv_timer_callback;
	dlv_timer_callback();
#endif

#ifdef ANTI_POP_WORK_STRUCT
	schedule_work(&dlv_anti_pop_work);
#endif
}

/**
 * DLV anti-pop "ioctl" routine.
 * This function should only be called in device open function.
 */
static void dlv_anti_pop(int mode)
{
	switch(mode) {
	case CODEC_WRMODE:
		break;
	case CODEC_RMODE:
		break;
	case CODEC_WMODE:
		/* Call work handler directly to anti-pop at system start.
		 * Notice that this is block operation...
		 * We use this way to ensure the operation of device openning
		 * has already completed before any other operations like ioctl.
		 */
		if (__dlv_get_sb_hp() != POWER_ON){
			printk("*************************************************\n");
			should_up = 1;
			dlv_anti_pop_part();
		}
		break;
	}
}

/********************************************************************************
 *				Volume control					*
 ********************************************************************************/

static void dlv_set_voice_volume(int vol)
{
	__dlv_set_line_in_bypass_volume(vol);
}


static int is_first_set_volume = 1;
extern unsigned int l009_globle_volume;
 
void dlv_set_replay_volume(int val)
{
#if 0
	unsigned long fixed_vol;

	ENTER();

	if (val < 0)
		val = 0;
	if (val > 100)
		val = 100;

	//fixed_vol = 6 +  (25 * (100 - val) / 100);
	fixed_vol = 31 * (100 - val) / 100;
#ifdef SNR_TEST
	//dlv_write_reg(0x10, 2);
	//dlv_write_reg(0x11, 2);
#endif
	__dlv_set_hp_volume(fixed_vol);

#if 1
	if (val == 0) {
		__dlv_set_dac_gain(0x1f);
		dlv_mute(1);
	} else {
		__dlv_set_dac_gain(0x06);
		dlv_mute(0);
	}
#endif

	DPRINT_CODEC("$$$$$ val = %d, DLV_REG_CGR1 = 0x%02x\n",
		     val, dlv_read_reg(DLV_REG_GCR1));

	LEAVE();
#else
	unsigned int dac_gain;


	//dac_gain = 6 + (25 * (100 - val) / 100);
	dac_gain = 0 + (31 * (100 - val) / 100);
	__dlv_set_hp_volume(0x6);
	__dlv_set_dac_gain(dac_gain);
	__dlv_set_line_in_bypass_volume(dac_gain);
	if (val == 0) {
		dlv_mute(1);
	} else {
		dlv_mute(0);
	}



#endif
}
EXPORT_SYMBOL(dlv_set_replay_volume);

static void dlv_set_mic_volume(int val)
{
	int fixed_vol;

	ENTER();

#ifdef JZDLV_USE_AGC
	fixed_vol = 15 * (100 - val) / 100;
	//dlv_write_reg(0x15, (1 << 7) | (val << 2)); /* target: 0x0~0x3c */
#else
	fixed_vol = 31 * val / 100;
	__dlv_set_adc_gain(fixed_vol);
	__dlv_set_mic1_boost(0);
	__dlv_set_mic2_boost(0);
#endif

	LEAVE();
}

/********************************************************************************
 *				ROUTE FUNCTIONS					*
 ********************************************************************************/

/**
 * Enable HP OUT replay mode
 *
 */
 void dlv_enable_hp_out(void)
{
	//__dlv_enable_nomad();
#if 0  //old
	__dlv_disable_dac_left_only();
	__dlv_hp_dac_to_lr();
	__dlv_disable_li_for_bypass();
	__dlv_switch_sb_line_out(POWER_OFF);
#esle  //for fm no sound
    __dlv_disable_hp_mute();
#endif
	mdelay(1);
}
EXPORT_SYMBOL(dlv_enable_hp_out);

/**
 * Disable HP OUT replay mode
 *
 */
 void dlv_disable_hp_out(void)
{
	//turn_off_sb_hp();
	__dlv_enable_hp_mute();
	//turn_off_dac(5);
	mdelay(10);
}
EXPORT_SYMBOL(dlv_disable_hp_out);

 void dlv_enable_line_out(void);

/**
 * Enable RECEIVER replay mode
 *
 */
static void dlv_enable_receiver(void)
{

	//old
#if 0
	dlv_enable_hp_out();
#else
	//change for fm no sound
	__dlv_disable_li_for_bypass();
	__dlv_disable_dac_left_only();
	__dlv_hp_dac_to_lr();
	__dlv_lineout_from_dac();
#endif

}

/* for sorting test */
static void dlv_set_replay_record(unsigned int input_src, int bypass) {
	if (input_src == USE_MIC) {
		//old
#if 0
		dlv_disable_mic_2();
		dlv_disable_line_in_record();
		dlv_disable_hp_out();

		dlv_enable_mic_2();
		dlv_enable_hp_out();
#else
		//change for fm no sound
		dlv_disable_mic_2();
		dlv_disable_line_in_record();
		__dlv_disable_li_for_bypass();
		__dlv_disable_dac_left_only();
		__dlv_hp_dac_to_lr();
		__dlv_lineout_from_dac();

		dlv_enable_mic_2();

#endif
	} else if (input_src == USE_LINEIN) {	      /* linein */
		//old
#if 0
		dlv_disable_mic_1();
		dlv_disable_mic_2();
		dlv_disable_line_out();

		dlv_enable_line_in_record(0);
		dlv_enable_hp_out();

		__dlv_hp_linein_to_lr();
		__dlv_enable_li_for_bypass();
		dlv_set_line(100);

		__dlv_disable_hp_mute();
		turn_on_dac(5);
#else
		//change for fm no sound
		dlv_disable_mic_1();
		dlv_disable_mic_2();

		__dlv_enable_li_for_bypass();
		dlv_set_line(100);


		__dlv_hp_linein_to_lr();
		__dlv_lineout_from_bypass();


#endif
	}
}

/**
 * Disable RECEIVER replay mode
 *
 */
static void dlv_disable_receiver(void)
{

	dlv_disable_hp_out();
}

/**
 * Enable LINE OUT replay mode
 *
 */
 void dlv_enable_line_out(void)
{


	//old
#if 0
	__dlv_switch_sb_line_out(POWER_ON);
	__dlv_disable_lineout_mute();
	__dlv_lineout_from_dac();
#else
	//chnage for fm no sound
	__dlv_disable_lineout_mute();
#endif
}
EXPORT_SYMBOL(dlv_enable_line_out);


/**
 * Disable LINE OUT replay mode
 *
 */
 void dlv_disable_line_out(void)
{
	//old
#if 0
	__dlv_switch_sb_line_out(POWER_OFF);
#else
	//change for fm no sound
	__dlv_enable_lineout_mute();
#endif
}
EXPORT_SYMBOL(dlv_disable_line_out);
 void dlv_disable_hp_mute(void)
{
	__dlv_disable_hp_mute();
}
EXPORT_SYMBOL(dlv_disable_hp_mute);


static void dlv_set_line(int val)
{
        int cur_vol;

        ENTER();
        /* set gain */
        cur_vol = 31 * val / 100;
        cur_vol &= 0x1f;
        /* ???
        dlv_write_reg(11, cur_vol);//GO1L
        dlv_write_reg(12, cur_vol);//GO1R
        */

	__dlv_set_line_in_bypass_volume(val);
        LEAVE();
}

static void dlv_set_line_in(void)
{
        printk("[-- route --] %s\n", __FUNCTION__);

	__dlv_disable_agc();
	__dlv_enable_micdiff();
	__dlv_enable_lineout_mute();

	__dlv_switch_sb_line_out(POWER_OFF);

	__dlv_enable_li_for_adc();
	__dlv_adc_linein_to_lr();

	__dlv_disable_mic1();
	__dlv_disable_adc();
}



static void dlv_enable_line_in_record(int insel)
{

	__dlv_disable_agc();

	schedule_timeout(2);

	__dlv_enable_li_for_adc();
	__dlv_adc_linein_to_lr();
	__dlv_set_adc_gain(0x0);
	__dlv_disable_adc_left_only();
	__dlv_disable_adc_mono();
	__dlv_enable_adc();
}

/**
 * Disable LINE IN record mode
 *
 * Disable every channels independently ???
 */
static void dlv_disable_line_in_record(void)
{
	__dlv_disable_adc();
}

/**
 * Enable LINE IN bypass to HP
 *
 * Depend: LINE IN was enabled, HP out was enabled
 */
static void dlv_enable_line_in_bypass_hp(void)
{

	// Yunfeng@Jul27'10 added for FM's audio path
	__dlv_switch_sb(POWER_ON);
	__dlv_switch_sb_sleep(POWER_ON);

	__dlv_set_16ohm_load();

	__dlv_switch_sb_line_out(POWER_OFF);
	if (__dlv_get_sb_hp() != POWER_ON) {
		__dlv_switch_sb_hp(POWER_ON);
		dlv_sleep_wait_bitset(DLC_IFR, DLC_IFR_RUP, 100,__LINE__);
	}
	__dlv_enable_li_for_adc();
	__dlv_enable_li_for_bypass();

	__dlv_hp_linein_to_lr();

	__dlv_disable_dac_mute();
	__dlv_enable_lineout_mute();
	__dlv_disable_hp_mute();
	__dlv_disable_dac_left_only();
}

/**
 * Disable LINE IN bypass to HP
 *
 */
static void dlv_disable_line_in_bypass_hp(void)
{
}

/**
 * Enable digital record mix into DA
 *
 */
static void dlv_enable_rec_2_dac(void)
{
	__dlv_hp_mic1_to_lr();
}

/**
 * Disable ditital record mix into DA.
 *
 */
static void dlv_disable_rec_2_dac(void)
{
	//__dlv_set_mix_rec_only();
}

/**
 * Enable MIC 2 record mode
 *
 * Caution: Please config regs according specific board
 */
static void dlv_enable_mic_2(void)
{
	static int first = 1;
	ENTER();

	__dlv_enable_adc();
	//schedule_timeout(2);

	//__dlv_enable_agc();
	__dlv_disable_agc();

	__dlv_switch_sb_micbias(POWER_ON);

	__dlv_enable_micdiff();

	/* For z800, main MIC is connected to MIC2 of JZ4760 */
	__dlv_adc_mic2_to_lr();

	__dlv_disable_mic_stereo();
	__dlv_enable_adc_mono();

	__dlv_enable_adc_left_only();

	//__dlv_set_mic2_boost(0x0);
	//__dlv_set_adc_gain(0x0);
#if 0
	/* Depend on board situation. For z800, we set max. */
#if 0
	/* max */
	__dlv_set_mic2_boost(0x3f);
	__dlv_set_adc_gain(0x9f);
#endif

#if 1
	/* for record test */
	__dlv_set_mic2_boost(0x3f);
	__dlv_set_adc_gain(0x80);
#endif

#if 0
	/* min */
	__dlv_set_mic2_boost(0x0);
	__dlv_set_adc_gain(0x8f);
#endif
#endif

	__dlv_enable_mic2();

#if 0
	if (first) {
		dlv_write_reg(0x15, 0x0); /* target: 0x0~0x3c */
		dlv_write_reg(0x16, (1 << 7) | (0x2<< 4) | dlv_read_reg(0x16));  /* noise gate[7 6:4] & hold[3:0] */
		dlv_write_reg(0x17, 0x0);			   /* ATK[7:4] and DCY[3:0] */
		dlv_write_reg(0x18, 0x1f);			   /* AGC_MAX */
		dlv_write_reg(0x19, 0x0);			   /* AGC_MIN */
		first = 0;
	}
#endif
	__dlv_disable_agc();
	//__dlv_enable_agc();

//	dump_dlv_regs("enable mic2");

	//DUMP_CODEC_REGS("leave dlv_enable_mic_2\n");
	//dump_dlv_regs("leave dlv_enable_mic_2\n");
	LEAVE();
}

/**
 * Disable MIC 2 record mode
 *
 */
static void dlv_disable_mic_2(void)
{
	__dlv_disable_agc();
	__dlv_switch_sb_micbias(POWER_OFF);
	__dlv_disable_adc();
	mdelay(2);
	__dlv_disable_mic1();
}

/**
 * Enable MIC 1 record mode
 *
 */
static void dlv_enable_mic_1(void)
{
	ENTER();

	__dlv_enable_adc();
	mdelay(20);
//	__dlv_enable_agc();
	__dlv_disable_agc();

	__dlv_switch_sb_micbias(POWER_ON);

	__dlv_enable_micdiff();

	/* For z800, main MIC is connected to MIC2 of JZ4760 */
	__dlv_adc_mic1_to_lr();

	__dlv_disable_mic_stereo();

	__dlv_enable_adc_left_only();

#if 0
	/* Depend on board situation. For z800, we set max. */
#if 0
	/* max */
	__dlv_set_mic1_boost(0x3f);
	__dlv_set_adc_gain(0x80);
#endif

#if 0
	/* for record test */
	__dlv_set_mic1_boost(0x3f);
	_-dlv_set_adc_gain(0x80);
#endif

#if 1
	/* min */
	__dlv_set_mic1_boost(0x0);
	__dlv_set_adc_gain(0x8f);
#endif
#endif

	//__dlv_set_mic1_boost(0x0);
	__dlv_set_mic1_boost(0x5);
	__dlv_set_adc_gain(0x0);

	__dlv_enable_mic1();

	__dlv_disable_agc();

//	dump_dlv_regs("enable mic1");

	//DUMP_CODEC_REGS("leave dlv_enable_mic_2\n");
	//dump_dlv_regs("leave dlv_enable_mic_2\n");
	LEAVE();
}

/**
 * Disable MIC 1 record mode
 *
 */
static void dlv_disable_mic_1(void)
{
	__dlv_disable_agc();
	__dlv_switch_sb_micbias(POWER_OFF);
	__dlv_disable_adc();
	mdelay(2);
	__dlv_disable_mic1();
	//DUMP_CODEC_REGS("leave dlv_disable_mic_1\n");
}

#ifdef CONFIG_JZ4760_PT701
/**
 * CODEC set device
 *
 * NEED TO FIX: do not enable device immediately, just set to g_current_out_dev (for output).
 *
 */
static void dlv_set_device(struct snd_device_config *snd_dev_cfg)
{

	dlv_enable_hp_out();
//	__dlv_disable_hp_mute();

	switch (snd_dev_cfg->device) {
	case SND_DEVICE_HANDSET:
		break;
	case SND_DEVICE_SPEAKER:
		if (gstate_dock) {
			printk("pt701 audio select external speaker!");
			/* external speaker attached */
			/* disable internal speaker output */
			__gpio_as_output(GPIO_SPK_EN);
			__gpio_clear_pin(GPIO_SPK_EN);
			/* enable external speaker output */
			__gpio_as_output(GPIO_DOCK_SPK_EN);
			__gpio_set_pin(GPIO_DOCK_SPK_EN);
		} else {
			printk("pt701 audio select internal speaker!");
			/* external speaker not attached */
			/* enable internal speaker output */
			__gpio_as_output(GPIO_SPK_EN);
			__gpio_set_pin(GPIO_SPK_EN);
			/* disable external speaker output */
			__gpio_as_output(GPIO_DOCK_SPK_EN);
			__gpio_clear_pin(GPIO_DOCK_SPK_EN);
		}
		__dlv_set_godr(0x00);
		break;
	case SND_DEVICE_HEADSET:
		__dlv_set_godr(0x93);
	case SND_DEVICE_HEADSET_AND_SPEAKER:
		printk("pt701 audio select headphone!");
		/* disable speaker output */
		__gpio_as_output(GPIO_SPK_EN);
		__gpio_clear_pin(GPIO_SPK_EN);
		/* disable external speaker output */
		__gpio_as_output(GPIO_DOCK_SPK_EN);
		__gpio_clear_pin(GPIO_DOCK_SPK_EN);
		__dlv_set_godr(0x8f);
		break;
	case SND_DEVICE_BT:
		break;
	default:
		printk("JZ DLV: Unkown ioctl argument in SND_SET_DEVICE\n");
	};

	g_current_out_dev = snd_dev_cfg->device;
}
#else
/**
 * CODEC set device
 *
 * NEED TO FIX: do not enable device immediately, just set to g_current_out_dev (for output).
 *
 */
#if 0
static void dlv_set_device(struct snd_device_config *snd_dev_cfg)
{
	switch (snd_dev_cfg->device) {
	case SND_DEVICE_HANDSET:
		dlv_enable_receiver();
		break;
	case SND_DEVICE_HEADSET:

		break;
	case SND_DEVICE_SPEAKER:
		dlv_enable_speaker();
		break;
	case SND_DEVICE_HEADSET_AND_SPEAKER:
		dlv_enable_speaker();
		break;
	case SND_DEVICE_BT:
		break;
	default:
		printk("JZ DLV: Unkown ioctl argument in SND_SET_DEVICE\n");
	};

	g_current_out_dev = snd_dev_cfg->device;
}
#endif
#endif

static void dlv_debug(int arg)
{
	switch(arg) {
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		printk("JZ DLV: Unknown debug number\n");
		break;
	};
}

static void dlv_suspend(void)
{
	DPRINT_CODEC("CODEC suspend ...\n");
	dlv_shutdown();
}

static void dlv_resume(void)
{
#ifdef CONFIG_JZ4760_PT701
	/* if headphone out */
	if (!gstate_hp) {
		/* disable speaker output */
		__gpio_as_output(GPIO_SPK_EN);
		__gpio_clear_pin(GPIO_SPK_EN);
		/* disable external speaker output */
		__gpio_as_output(GPIO_DOCK_SPK_EN);
		__gpio_clear_pin(GPIO_DOCK_SPK_EN);
	}
#endif

	DPRINT_CODEC("CODEC resume ...\n");
	dlv_enable_hp_without_pop();
}

static void dlv_reset(void)
{
	__dlv_set_dac_serial();
	__dlv_set_adc_serial();
	__dlv_dac_i2s_mode();
	__dlv_adc_i2s_mode();
	__dlv_set_dac_lrswap();
	__dlv_enable_adc_lrswap();
	//__dlv_set_cap_less();
	__dlv_set_cap_couple();

	/* default settings */
	__dlv_hp_dac_to_lr(); /* a little pop click if select DAC in this step */
	dlv_set_replay_rate(44100);
	dlv_set_data_width(REPLAY, 16);
	dlv_set_channel(2);
}

/**
 * CODEC ioctl (simulated) routine
 *
 * Provide control interface for i2s driver
 */
static int jzdlv_ioctl(void *context, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	//int start_jiffies = 0;
	//int end_jiffies = 0;
	ENTER();
	DUMP_CODEC_REGS(__FUNCTION__);
	DPRINT_CODEC("[dlv IOCTL]++++++++++++++++++++++++++++\n");
	DPRINT_CODEC("%s  cmd = %d, arg = %lu\n", __FUNCTION__, cmd, arg);
	DPRINT_DLV_IOC_CMD(cmd);
	DPRINT_CODEC("[dlv IOCTL]----------------------------\n");

	should_up = 1;
	DLV_LOCK();
	switch (cmd) {
	case CODEC_FIRST_OUTPUT:
//		dlv_set_replay_volume(greplay_volume);
		//__dlv_disable_dac_mute();
		//start_jiffies = jiffies;
		//mdelay(300);
		__dlv_disable_hp_mute();
		turn_on_dac(5);
		//__dlv_set_hp_volume(curr_vol);
		//end_jiffies = jiffies;
		//printk("turn on DAC_MUTE duration: %d\n", end_jiffies - start_jiffies);
		break;

	case CODEC_INIT:
		dlv_init();
		break;

	case CODEC_SET_MODE:
		/* Add new mode to current mode. */
		//dlv_set_mode(g_codec_mode | arg);
		break;

	case CODEC_CLEAR_MODE:
		/* Remove dest mode from current mode. */
		//dlv_set_mode(g_codec_mode & (~arg));
		break;

	case CODEC_SET_LINEIN2HP:
		dlv_enable_line_in_bypass_hp();
		break;

	case CODEC_CLEAR_LINEIN2HP:
		dlv_disable_line_in_bypass_hp();
		break;

	case CODEC_SET_STARTUP_PARAM:
		break;

	case CODEC_SET_REPLAY:
		dlv_enable_receiver();
		break;

	case CODEC_SET_RECORD:
#if 0
		if (arg == USE_LINEIN)
			dlv_set_line_in();
		else if (arg == USE_MIC)
			board_set_record();
#endif
		board_set_record();
		break;

	case CODEC_SET_REC_2_DAC:
		if (arg) {
			printk("JZ DLV: SET REC 2 ADC ---- %d\n", (int)arg);
			dlv_enable_line_in_record(3);
			dlv_enable_rec_2_dac();
		}
		break;

	case CODEC_SET_REPLAY_RECORD:
		dlv_set_replay_record(arg, 1);
		break;

	case CODEC_SET_REPLAY_VOLUME:
//		greplay_volume = arg;
		dlv_set_replay_volume(arg);
		break;

	case CODEC_SET_MIC_VOLUME:
		dlv_set_mic_volume(arg);
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
		//dlv_turn_replay(arg);
		printk("JZ DLV: should not run here -- CODEC_TURN_REPLAY\n");
		BUG_ON(1);
		break;

	case CODEC_TURN_OFF:
		dlv_turn_off(arg);
		break;

	case CODEC_I2S_SUSPEND:
		dlv_suspend();
		break;

	case CODEC_I2S_RESUME:
		should_up = 0;
		dlv_resume();
		break;

	case CODEC_GET_MIXER_INFO:
		dlv_get_mixer_info((mixer_info *)arg);
		break;

	case CODEC_GET_MIXER_OLD_INFO:
		dlv_get_mixer_old_info((mixer_info *)arg);
		break;

	case CODEC_SET_REPLAY_RATE:
		ret = dlv_set_replay_rate(arg);
		break;

	case CODEC_SET_RECORD_RATE:
		ret = dlv_set_record_rate(arg);
		break;

	case CODEC_SET_RECORD_CHANNEL:
		ret =  arg;
		break;

	case CODEC_SET_REPLAY_CHANNEL:
		ret = dlv_set_channel(arg);
		break;

	case CODEC_SET_RECORD_DATA_WIDTH:
		ret = dlv_set_data_width(RECORD, arg);
		break;

	case CODEC_SET_REPLAY_DATA_WIDTH:
		ret = dlv_set_data_width(REPLAY, arg);
		break;

	case CODEC_DAC_MUTE:
		ret = dlv_mute(arg);
		break;

	case CODEC_DEBUG_ROUTINE:
		dlv_debug(arg);
		break;

	default:
		printk("JZ DLV:%s:%d: Unkown IOC commond\n", __FUNCTION__, __LINE__);
		ret = -1;
	}

	if (should_up)
	{
		DLV_UNLOCK();
	}
	LEAVE();
	return ret;
}

/**
 * CODEC restart routine
 *
 */
static void dlv_restart(void)
{

}

static struct work_struct dlv_irq_work;
static spinlock_t dlv_irq_lock;
//static int i = 0;
static int handling_sclr = 0;


/**
 * CODEC short circut handler
 *
 * To protect CODEC, CODEC will be shutdown when short circut occured.
 * Then we have to restart it.
 */
static inline void dlv_short_circut_handler(void)
{
	unsigned short curr_vol;
	unsigned int	dlv_ifr, delay;

#define VOL_DELAY_BASE 22               //per VOL delay time in ms

	printk("JZ DLV: Short circut detected! restart CODEC hp out finish.\n");

	curr_vol = dlv_read_reg(DLC_GCR_HPL);
	delay = VOL_DELAY_BASE * (0x20 - (curr_vol & 0x1f));

	/* min volume */
	__dlv_set_hp_volume(0x1f);

	printk("Short circut volume delay %d ms curr_vol=%x \n", delay,curr_vol);
	msleep(delay);


	//turn_off_sb_hp();
	/* clear IFR_RDO */
	//__dlv_set_irq_flag(DLC_IFR_RDO);
	/* turn off sb_hp */
	//__dlv_switch_sb_hp(POWER_OFF);

	turn_off_dac(5);
	msleep(1);
	/* clear IFR_RDO */
	__dlv_set_irq_flag(DLC_IFR_RDO);
	/* turn off sb_hp */
	__dlv_switch_sb_hp(POWER_OFF);
	mdelay(300);
	__dlv_enable_hp_mute();
 	mdelay(1);
	__dlv_switch_sb_dac(POWER_OFF);

	mdelay(1);
	__dlv_switch_sb_sleep(POWER_OFF);
	mdelay(1);
	__dlv_switch_sb(POWER_OFF);
	mdelay(1);

	while (1) {
		dlv_ifr = __dlv_get_irq_flag();
		printk("waiting for short circuit recover finish ----- dlv_ifr = 0x%02x\n", dlv_ifr);
		if ((dlv_ifr & (DLC_IFR_SCLR | DLC_IFR_SCMC2)) == 0)
			break;
		__dlv_set_irq_flag((DLC_IFR_SCLR | DLC_IFR_SCMC2));
		msleep(10);
	}

	//turn_on_sb_hp();

	__dlv_switch_sb(POWER_ON);
	mdelay(300);

	__dlv_switch_sb_sleep(POWER_ON);

	__dlv_switch_sb_dac(POWER_ON);
	udelay(500);

	__dlv_enable_hp_mute();
	mdelay(1);

	//turn_on_sb_hp();
	/* clear IFR_RUP */
	__dlv_set_irq_flag(DLC_IFR_RUP);
	/* turn on sb_hp */
	__dlv_switch_sb_hp(POWER_ON);
	mdelay(300);

	turn_on_dac(10);
	__dlv_disable_hp_mute();

	__dlv_set_hp_volume(curr_vol);
	msleep(delay);
	// do not clear RDO and RUP
	//__dlv_set_irq_flag((1 << IFR_RDO) | (1 << IFR_RUP));
}
#ifdef HP_SENSE_DETECT
static void dlv_jack_handler(int new_switch_status)
{
	//just set the state is enough,
	if (g_switch_data) {
		switch_set_state(&g_switch_data->sdev, new_switch_status);
      	}
}
#endif

#ifdef HP_SENSE_DETECT

/**
 * CODEC work queue handler
 *
 * Handle bottom-half of SCLR & JACKE irq
 *
 */
static void dlv_irq_work_handler(struct work_struct *work)
{
	unsigned int dlv_ifr;
	unsigned long flags;

	int old_status;
	int new_status;
	int i;
	int j;
	DLV_LOCK();
	do {
		dlv_ifr = __dlv_get_irq_flag();

		if (dlv_ifr & (DLC_IFR_SCLR | DLC_IFR_SCMC2)) {
			dlv_short_circut_handler();
		}

		dlv_ifr = __dlv_get_irq_flag();
#if 1
		/* Updata SCLR */
		__dlv_set_irq_flag((DLC_IFR_SCLR | DLC_IFR_SCMC2));
//		gf_short_circuit_flow_run = 0;
		/* Unmask SCLR */
		__dlv_set_irq_mask(ICR_COMMON_MASK);
#endif
		printk("JZ DLV: Short circut detected! dlv_ifr = 0x%02x\n", dlv_ifr);

	} while(dlv_ifr & (DLC_IFR_SCLR | DLC_IFR_SCMC2));

	handling_sclr = 0;

	if (dlv_ifr & (1 << IFR_JACKE)) {
_ensure_stable:
		j = 0;
		old_status = (__dlv_get_irq_flag() & (1 << IFR_JACK)) != 0;
		/* Read status at least 3 times to make sure it is stable. */
		for (i = 0; i < 3; ++i) {
			new_status = (__dlv_get_irq_flag() & (1 << IFR_JACK)) != 0;

			if (old_status != new_status) {
				j += i + 1;
			}
			old_status = new_status;
			msleep(50);
		}
	}

	spin_lock_irqsave(&dlv_irq_lock, flags);

	/* Clear current irq flag */
	__dlv_set_irq_flag(dlv_ifr);

	/* Unmask SCLR & JACK (ifdef HP_SENSE_DETECT) */
	__dlv_set_irq_mask(ICR_COMMON_MASK);

	spin_unlock_irqrestore(&dlv_irq_lock, flags);

	/* If the jack status has changed, we have to redo the process. */
	if (dlv_ifr & (1 << IFR_JACKE)) {
		msleep(50);
		new_status = (__dlv_get_irq_flag() & (1 << IFR_JACK)) != 0;
		if (new_status != old_status) {
			goto _ensure_stable;
		}
	}

	/* Report status */
	dlv_jack_handler(new_status);
	DLV_UNLOCK();
}

/**
 * IRQ routine
 *
 * Now we are only interested in SCLR and JACKE.
 */
static irqreturn_t dlv_codec_irq(int irq, void *dev_id)
{
	unsigned char dlv_icr;
	unsigned char dlv_ifr;

	unsigned int aic_reg;
	unsigned long flags;

	spin_lock_irqsave(&dlv_irq_lock, flags);

	dlv_ifr = __dlv_get_irq_flag();
	dlv_icr = __dlv_get_irq_mask();

	/* Mask all irq temporarily */
	__dlv_set_irq_mask(ICR_ALL_MASK);

	aic_reg = REG_AIC_SR;

	REG_AIC_SR = 0x78; // legacy ... need check

	if (!(dlv_ifr & (DLC_IFR_SCLR | DLC_IFR_SCMC2 | DLC_IFR_JACKE))) {
		/* CODEC may generate irq with flag = 0xc0.
		 * We have to ingore it in this case as there is no mask for the reserve bit.
		 */
		//printk("JZ DLV: Unkown irq detected, mask = 0x%08x, flag = 0x%08x\n", dlv_icr, dlv_ifr);

		printk("AIC interrupt ??? AIC_SR = 0x%08x\n", aic_reg);

		/* Unmask SCLR & JACK (ifdef HP_SENSE_DETECT) */
//		__dlv_set_irq_mask(ICR_COMMON_MASK);

		spin_unlock_irqrestore(&dlv_irq_lock, flags);
		return IRQ_HANDLED;

	} else {

		spin_unlock_irqrestore(&dlv_irq_lock, flags);

		if (handling_sclr == 0) {
			handling_sclr = 1;
			/* Handle SCLR in work queue. */
			schedule_work(&dlv_irq_work);
		}

		return IRQ_HANDLED;
	}
}

#else

/**
 * CODEC work queue handler
 *
 * Handle bottom-half of SCLR & JACKE irq
 *
 */
static void dlv_irq_work_handler(struct work_struct *work)
{
	unsigned int	dlv_ifr;
	unsigned long	flags;
	DLV_LOCK();
	do {
		dlv_ifr = __dlv_get_irq_flag();

		if (dlv_ifr & (DLC_IFR_SCLR | DLC_IFR_SCMC2)) {
			dlv_short_circut_handler();
		}

		dlv_ifr = __dlv_get_irq_flag();
#if 1
		/* Updata SCLR */
		__dlv_set_irq_flag(DLC_IFR_SCLR | DLC_IFR_SCMC2);
//		gf_short_circuit_flow_run = 0;
		/* Unmask SCLR */
		__dlv_set_irq_mask(ICR_COMMON_MASK);
#endif
		printk("JZ DLV: Short circut detected! dlv_ifr = 0x%02x\n",dlv_ifr);

	} while(dlv_ifr & (DLC_IFR_SCLR | DLC_IFR_SCMC2));

	handling_sclr = 0;
	DLV_UNLOCK();
}

/**
 * IRQ routine
 *
 * Now we are only interested in SCLR.
 */
static irqreturn_t dlv_codec_irq(int irq, void *dev_id)
{
	unsigned char dlv_ifr;

	unsigned int aic_reg;
	unsigned long flags;

	printk("===>enter dlv_codec_irq!!!\n");

	spin_lock_irqsave(&dlv_irq_lock, flags);

	dlv_ifr = __dlv_get_irq_flag();

	/* Mask all irq temporarily */
	__dlv_set_irq_mask(ICR_ALL_MASK);

	aic_reg = REG_AIC_SR;

	REG_AIC_SR = 0xe0; // legacy ... need check

	if (!(dlv_ifr & (DLC_IFR_SCLR | DLC_IFR_SCMC2 | DLC_IFR_JACK))) {
		/* CODEC may generate irq with flag = 0xc0.
		 * We have to ingore it in this case as there is no mask for the reserve bit.
		 */
		printk("AIC interrupt, AIC_SR = 0x%02x, IFR = 0x%02x\n", aic_reg, dlv_ifr);

		/* Unmask SCLR & JACK (ifdef HP_SENSE_DETECT) */
		__dlv_set_irq_mask(ICR_COMMON_MASK);

		spin_unlock_irqrestore(&dlv_irq_lock, flags);
		return IRQ_HANDLED;

	} else {
		spin_unlock_irqrestore(&dlv_irq_lock, flags);

		/* Handle SCLR and JACK in work queue. */
		schedule_work(&dlv_irq_work);

		return IRQ_HANDLED;
	}
}

#endif // ifdef HP_SENSE_DETECT


#ifdef HP_SENSE_DETECT

/*
 * Headphone sense switch registration & initialization
 */
static ssize_t jz_hp_switch_print_state(struct switch_dev *sdev, char *buf)
{
	jz_hp_switch_data_t	*switch_data =
		container_of(sdev, jz_hp_switch_data_t, sdev);
	const char *state;

	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

static int jz_hp_switch_probe(struct platform_device *pdev)
{
	jz_hp_switch_data_t *switch_data;
	jz_hp_switch_platform_data_t *pdata = pdev->dev.platform_data;
	int ret = 0;

	switch_data = kzalloc(sizeof(jz_hp_switch_data_t), GFP_KERNEL);
	if (!switch_data) {
		printk("JZ HP Switch kzalloc failed (%s:%d)\n", __FUNCTION__, __LINE__);
		return -ENOMEM;
	}
	g_switch_data = switch_data;

	switch_data->sdev.name		= pdata->name;
	switch_data->name_on		= pdata->name_on;
	switch_data->name_off		= pdata->name_off;
	switch_data->state_on		= pdata->state_on;
	switch_data->state_off		= pdata->state_off;
	switch_data->sdev.print_state	= jz_hp_switch_print_state;

	if ((ret = switch_dev_register(&switch_data->sdev))) {
		printk("JZ HP Switch: Could net register switch device\n");
		return ret;
	}

	ret = __dlv_get_irq_flag();

	switch_set_state(&switch_data->sdev, dlv_read_reg(DLV_REG_IFR) & (1 << IFR_JACK));

	return 0;
}

static int __devexit jz_hp_switch_remove(struct platform_device *pdev)
{
	switch_dev_unregister(&g_switch_data->sdev);
	kfree(g_switch_data);

	return 0;
}

static struct platform_driver jz_hp_switch_driver = {
	.probe		= jz_hp_switch_probe,
	.remove		= __devexit_p(jz_hp_switch_remove),
	.driver		= {
		.name	= "hp-switch",
		.owner	= THIS_MODULE,
	},
};
#endif  // ifdef HP_SENSE_DETECT

/**
 * Module init
 */
static int __init init_dlv(void)
{
	int retval;

	cpm_start_clock(CGM_AIC);

	spin_lock_init(&dlv_irq_lock);

	spin_lock_init(&g_dlv_sem_lock);

	INIT_WORK(&dlv_irq_work, dlv_irq_work_handler);

#ifdef ANTI_POP_WORK_STRUCT
	INIT_WORK(&dlv_anti_pop_work, dlv_anti_pop_work_handler);
#endif

	register_jz_codecs((void *)jzdlv_ioctl);

	dlv_reset();
	retval = request_irq(IRQ_AIC, dlv_codec_irq, IRQF_DISABLED, "dlv_codec_irq", NULL);
	if (retval) {
		printk("JZ DLV: Could not get AIC CODEC irq %d\n", IRQ_AIC);
		return retval;
	}

#ifdef HP_SENSE_DETECT
	retval = platform_driver_register(&jz_hp_switch_driver);
	if (retval) {
		printk("JZ HP Switch: Could net register headphone sense switch\n");
		return retval;
	}
#endif
	return 0;
}

/**
 * Module exit
 */
static void __exit cleanup_dlv(void)
{

	free_irq(IRQ_AIC, NULL);
#ifdef HP_SENSE_DETECT
	platform_driver_unregister(&jz_hp_switch_driver);
#endif
}

module_init(init_dlv);
module_exit(cleanup_dlv);


