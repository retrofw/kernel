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
#include "jz4760_dlv.h"
#include "jz_i2s_dbg.h"

// Hanvon
//static int spk_state;//add by wll
static int first_start = 1;

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
/*
static unsigned int g_prev_dev = 0;
static unsigned int g_codec_mode = 0;
static unsigned int g_speaker_mute = 0;
static unsigned int g_receiver_mute = 0;
static unsigned int g_mic1_mute = 0;
static unsigned int g_mic2_mute = 0;
static unsigned int g_volumes[SND_DEVICE_COUNT];
static void dlv_enable_speaker(void);
static void dlv_disable_speaker(void);
*/

/* Audio route ops */
static void dlv_anti_pop_part(void);
static void dlv_enable_hp_out(void);
static void dlv_disable_hp_out(void);
static void dlv_enable_btl(void);
static void dlv_disable_btl(void);
static void dlv_enable_receiver(void);
static void dlv_disable_receiver(void);
static void dlv_enable_line_out(void);
static void dlv_disable_line_out(void);
static void dlv_enable_line_in_record(int insel);
static void dlv_set_line_in(void);
static void dlv_disable_line_in_record(void);
static void dlv_enable_line_in_bypass_hp(void);
static void dlv_disable_line_in_bypass_hp(void);
static void dlv_enable_line_in_bypass_btl(void);
static void dlv_disable_line_in_bypass_btl(void);
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

static inline void dlv_sleep_wait_bitset(int reg, unsigned bit, int stime, int line)
{
	int count = 0;
	while(!(dlv_read_reg(reg) & (1 << bit))) {
		msleep(stime);
		if(count++ >3){
			printk("wait reg %02x bit %d set fail\n",reg, bit);
			break;
		}
	}
}

static inline void dlv_sleep_wait_bitclear(int reg, unsigned bit, int stime)
{
	int count = 0;
	while((dlv_read_reg(reg) & (1 << bit))){
		msleep(stime);
		if(count++ >3){
			printk("wait reg %02x bit %d clear fail\n", reg, bit);
			break;
		}
	}
}

/**
 * CODEC write a bit of a register
 *
 * addr:	address of register
 * bitval:	bit value to modifiy
 * mask_bit:	indicate which bit will be modifiy
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

/*
 * DLV CODEC operations routines
 */

static inline void turn_on_dac(int timeout)
{
	if(__dlv_get_dac_mute()){
		/* clear IFR_GUP */
		__dlv_set_irq_flag(1 << IFR_GUP);
		/* turn on dac */
		__dlv_disable_dac_mute();
		/* wait IFR_GUP set */
		dlv_sleep_wait_bitset(0xb, IFR_GUP, timeout,__LINE__);
	}
}

static inline void turn_off_dac(int timeout)
{
 	if (!(__dlv_get_dac_mute())){
		/* clear IFR_GDO */
		__dlv_set_irq_flag(1 << IFR_GDO);
		/* turn off dac */
		__dlv_enable_dac_mute();
		/* wait IFR_GDO set */
		dlv_sleep_wait_bitset(0xb, IFR_GDO, timeout,__LINE__);
	}
}

static inline void turn_on_sb_hp(void)
{
	if (__dlv_get_sb_hp() != POWER_ON){
		/* clear IFR_RUP */
		__dlv_set_irq_flag(1 << IFR_RUP);
		/* turn on sb_hp */
        	__dlv_switch_sb_hp(POWER_ON);
		/* wait IFR_RUP set */
        	dlv_sleep_wait_bitset(0xb, IFR_RUP, 100,__LINE__);
	}

}
static inline void turn_off_sb_hp(void)
{
	if (__dlv_get_sb_hp() != POWER_OFF){
		/* clear IFR_RDO */
		__dlv_set_irq_flag(1 << IFR_RDO);
		/* turn off sb_hp */
        	__dlv_switch_sb_hp(POWER_OFF);
		/* wait IFR_RDO set */
        	dlv_sleep_wait_bitset(0xb, IFR_RDO, 100,__LINE__);
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
	mdelay(1);
	__dlv_enable_hp_mute();
 	mdelay(1);
	__dlv_switch_sb_dac(POWER_OFF);

	mdelay(1);
	__dlv_switch_sb_sleep(POWER_OFF);
	mdelay(1);
	__dlv_switch_sb(POWER_OFF);
	mdelay(1);
	first_start = 1;

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

	__dlv_switch_sb_micbias(POWER_ON);
	__dlv_set_int_form(ICR_INT_HIGH_8CYCLES);

	__dlv_set_irq_mask(ICR_COMMON_MASK);
	__dlv_set_irq_flag(0x3f);

	__dlv_set_12m_crystal();
	//__dlv_set_10kohm_load(); //1uF
	__dlv_set_16ohm_load(); // 220uF

    dlv_anti_pop_part();

	g_current_out_dev = SND_DEVICE_SPEAKER;

	LEAVE();
}

static void dlv_reset(void)
{
	ENTER();

	dlv_write_reg_bit(DLV_REG_AICR, 1, AICR_DAC_SERIAL);
	dlv_write_reg_bit(DLV_REG_AICR, 1, AICR_ADC_SERIAL);

	/* reset DLV codec. from hibernate mode to sleep mode */
	dlv_write_reg(DLV_REG_AICR, 0xf);
	dlv_write_reg_bit(DLV_REG_PMR1, 0, PMR1_SB);

	schedule_timeout(30);

	dlv_write_reg_bit(DLV_REG_PMR1, 0, PMR1_SB_SLEEP);


	dlv_write_reg(DLV_REG_CR3, 1 << CR3_MICSTEREO);//mic 1

	schedule_timeout(20);

	dlv_write_reg_bit(DLV_REG_PMR1, 0, PMR1_SB_AIP);

	//dlv_write_reg_bit(5, 0, 7);//PMR1.SB_DAC->0
	//dlv_write_reg_bit(5, 0, 4);//PMR1.SB_ADC->0
	dlv_write_reg_bit(DLV_REG_PMR2, 0, PMR2_SB_DAC);
	dlv_write_reg_bit(DLV_REG_PMR2, 0, PMR2_SB_ADC);

	schedule_timeout(2); ;//wait for stability
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

	int mrate[MAX_RATE_COUNT] = {
		96000, 48000, 44100, 32000, 24000,
		22050, 16000, 12000, 11025, 8000
	};

	for (val = 0; val < MAX_RATE_COUNT; val++) {
		if (rate >= mrate[val]) {
			speed = val;
			break;
		}
	}
	if (rate < mrate[MAX_RATE_COUNT - 1]) {
		speed = MAX_RATE_COUNT - 1;
	}
	val = dlv_read_reg(DLV_REG_CCR2);
	val &= 0xf;
	val = (speed << 4) | val;
	dlv_write_reg(DLV_REG_CCR2, val);

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

	int mrate[MAX_RATE_COUNT] = {
		96000, 48000, 44100, 32000, 24000,
		22050, 16000, 12000, 11025, 8000
	};

	for (val = 0; val < MAX_RATE_COUNT; val++) {
		if (rate >= mrate[val]) {
			speed = val;
			break;
		}
	}
	if (rate < mrate[MAX_RATE_COUNT - 1]) {
		speed = MAX_RATE_COUNT - 1;
	}
	val = dlv_read_reg(DLV_REG_CCR2);
	val &= 0xf0;
	val = speed | val;
	dlv_write_reg(DLV_REG_CCR2, val);

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

static void dlv_turn_off(int mode)
{
	ENTER();

	if ((mode & REPLAY) && (mode & RECORD)) {

		printk("JZ DLV: Close REPLAY and RECORD\n");
#if 1
		//dlv_write_reg_bit(1, 1, 5);//DAC_MUTE->1
		schedule_timeout(20);

		// 2010-01-31 Jason marked
		//dlv_write_reg_bit(5, 1, 6);//SB_OUT->1

		dlv_write_reg(9, 0xff);
		dlv_write_reg(8, 0x2f);
#endif
	} else if (mode & REPLAY) {

		printk("JZ DLV: Close REPLAY\n");
		//dlv_disable_receiver();
		//dlv_shutdown();
		//dlv_set_replay_volume(0);
		__dlv_enable_hp_mute();
		udelay(500);
		turn_off_dac(5);
		__aic_write_tfifo(0x0);
		__aic_write_tfifo(0x0);
		//nothing
	} else if (mode & RECORD) {
		printk("JZ DLV: Close RECORD\n");
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
		__dlv_enable_dac_m2s();
		break;
	case 2:
		// MONO->0 for Stereo
		__dlv_disable_dac_m2s();
		break;
	}

	return ch;
}

static int dlv_set_data_width(unsigned int mode, unsigned int width)
{
	unsigned char aicr = dlv_read_reg(DLV_REG_AICR);
	unsigned char aicr_bak = aicr;
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
		aicr &= ~(3 << 4);
		aicr |= (i << 4);
		break;
	case REPLAY:
		aicr &= ~(3 << 6);
		aicr |= (i << 6);
		break;
	}

	if (aicr != aicr_bak) {
		dlv_write_reg(DLV_REG_AICR, aicr);
	}

	if (width == 8) {
		return 8;
	} else {
		return supported_width[i];
	}
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
	for (i = 0; i < 32; i++) {
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
}

/********************************************************************************
 *				Anti-pop					*
 ********************************************************************************/
static struct work_struct dlv_anti_pop_work;
inline static void udelay_jz(int usec)
{
  int i = 0;
  i = usec * 50;
  while(i--)
  {
    __asm__(
        "nop\n\t"
        "nop\n\t"
        "nop\n\t"
        "nop\n\t"
        "nop\n\t"
        );
  }
}

inline static void mdelay_jz(int usec)
{
  int i = 1000;
  while(i--)
    udelay_jz(usec);

}

static void dlv_anti_pop_part(void)
{
	unsigned start_time = jiffies;
	if (first_start) {
		first_start = 0;
		__dlv_switch_sb(POWER_ON);
                mdelay_jz(30);
                __dlv_switch_sb_sleep(POWER_ON);
		mdelay_jz(40);
	}
	__dlv_switch_sb_dac(POWER_ON);
	udelay_jz(500);

	__dlv_enable_hp_mute();
	mdelay_jz(1);

	turn_on_sb_hp();
	mdelay_jz(1);
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


/**
 * Start startup anti-pop sequence.
 *
 * This is unblock operation that implement by using timer or work struct.
 *
 * Typically, for resuming from sleep.
 */
static void dlv_enable_hp_without_pop(void)
{
	schedule_work(&dlv_anti_pop_work);
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

void dlv_set_replay_volume(int val)
{
	unsigned long fixed_vol;

	ENTER();

    printk("volume is %d\n",val);
    if(val > 100)
      val = 100;

	fixed_vol = 6 +  (25 * (100 - val) / 100);

	__dlv_set_hp_volume(fixed_vol);

	if (val == 0) {
		__dlv_set_godr(0x1f | 0x80);
		dlv_mute(1);
	}
	else
	{
		__dlv_set_godr(0x00 | 0x80);
		dlv_mute(0);
	}

	DPRINT_CODEC("$$$$$ val = %d, DLV_REG_CGR1 = 0x%02x\n",
		     val, dlv_read_reg(DLV_REG_GCR1));

	LEAVE();
}

static void dlv_set_mic_volume(int val)
{
	int fixed_vol;

	ENTER();

#ifdef JZDLV_USE_AGC
	fixed_vol = 15 * (100 - val) / 100;
	dlv_write_reg(0x15, (1 << 7) | (val << 2)); /* target: 0x0~0x3c */
#else
/*
	fixed_vol = 31 * val / 100;
	__dlv_set_gidr(fixed_vol | 0x80);
*/
	/* If volume <= 50,it's change the GIM gain mainly, otherwise change the GID gain */
	if(val <= 50){
                fixed_vol = 5 * val / 50;
                __dlv_set_gim(fixed_vol << 3);

                fixed_vol = (5 * val % 50) / 13;
                __dlv_set_gidr(fixed_vol | 0x80);
        }else{
                __dlv_set_gim(0x5 << 3);

                fixed_vol = (val - 50) / 3;
                __dlv_set_gidr(fixed_vol | 0x80);
        }
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
static void dlv_enable_hp_out(void)
{
	__dlv_enable_nomad();
	__dlv_disable_dac_right_only();
	__dlv_set_outsel(DAC_OUTPUT);
	__dlv_switch_sb_bypass(POWER_OFF);
	__dlv_switch_sb_line_out(POWER_OFF);
	__dlv_switch_sb_btl(POWER_OFF);

	__dlv_switch_sb_aip(POWER_ON);
	mdelay(1);
#if 0
	__dlv_switch_sb(POWER_ON);
	// reference up time
	mdelay(300);
	__dlv_switch_sb_sleep(POWER_ON);
	//sleep up time
	mdelay(500);
	__dlv_switch_sb_dac(POWER_ON);
	//dac up time
	udelay(500);
	__dlv_disable_hp_mute();
	mdelay(1);
	turn_on_sb_hp();
#endif

//	mdelay(300);
//	__dlv_disable_dac_mute();
}

/**
 * Disable HP OUT replay mode
 *
 */
static void dlv_disable_hp_out(void)
{
	turn_off_sb_hp();
	__dlv_enable_hp_mute();
	turn_off_dac(5);
	//__dlv_switch_sb_dac(POWER_ON);
	mdelay(10);
}

/**
 * Enable btl replay mode
 *
 */
static void dlv_enable_btl(void)
{
	if (__dlv_get_sb_hp() != POWER_ON) {
		__dlv_switch_sb_hp(POWER_ON);
		dlv_sleep_wait_bitset(0xb, IFR_RUP, 100,__LINE__);
	}
	__dlv_switch_sb_bypass(POWER_OFF);
	__dlv_switch_sb_line_out(POWER_ON);
	__dlv_switch_sb_btl(POWER_ON);
	__dlv_disable_dac_mute();
	__dlv_disable_hp_mute();
	__dlv_disable_dac_right_only();
	__dlv_disable_nomad();
	__dlv_disable_lineout_mute();
	__dlv_disable_btl_mute();

	//dlv_set_replay_volume(50);

	DUMP_CODEC_REGS("after dlv_enable_btl\n");
}

/**
 * Disable btl replay mode
 *
 */
static void dlv_disable_btl(void)
{
	__dlv_switch_sb_btl(POWER_OFF);
	__dlv_switch_sb_line_out(POWER_OFF);

//	DUMP_CODEC_REGS("after dlv_disable_speaker\n");
}

/**
 * Enable SPEAKER replay mode
 *
 */
// extern unsigned int panle_mode;
extern unsigned int l009_globle_volume;
static void dlv_enable_speaker(void)
{
#ifdef  HP_POWER_EN
//	dlv_enable_btl();
	if (/*!panle_mode &&*/ l009_globle_volume > 0){
		__gpio_as_func0(HP_POWER_EN);
		__gpio_as_output(HP_POWER_EN);
		__gpio_enable_pull(HP_POWER_EN);
		__gpio_set_pin(HP_POWER_EN);
	}else{
		__gpio_as_func0(HP_POWER_EN);
		__gpio_as_output(HP_POWER_EN);
		__gpio_clear_pin(HP_POWER_EN);
	}
#endif
}

/**
 * Disable SPEAKER replay mode
 *
 */
static void dlv_disable_speaker(void)
{
#ifdef HP_POWER_EN
//	dlv_disable_btl();
  __gpio_as_func0(HP_POWER_EN);
  __gpio_as_output(HP_POWER_EN);
  __gpio_clear_pin(HP_POWER_EN);
#endif
}

static void dlv_enable_receiver(void)
{
//	__dlv_set_16ohm_load();
//	__dlv_set_10kohm_load();

	dlv_enable_hp_out();
}

static void dlv_set_replay_record(unsigned int input_src, int bypass) {
	if (input_src == USE_MIC)
		dlv_enable_line_in_record(0);
	else if (input_src == USE_LINEIN)      /* linein */
	{
		printk("select fm audio mode!\n");
		dlv_enable_line_in_bypass_hp();
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
static void dlv_enable_line_out(void)
{

	__dlv_set_10kohm_load();
	__dlv_switch_sb_dac(POWER_ON);
	if (__dlv_get_sb_hp() != POWER_ON) {
		__dlv_switch_sb_hp(POWER_ON);
		dlv_sleep_wait_bitset(0xb, IFR_RUP, 100,__LINE__);
	}
	__dlv_switch_sb_btl(POWER_OFF);
	__dlv_switch_sb_line_out(POWER_ON);


	__dlv_set_outsel(DAC_OUTPUT);

	__dlv_disable_nomad();
	__dlv_disable_dac_right_only();
//	__dlv_enable_dac_right_only();
	//dlv_set_replay_volume(70);
	__dlv_disable_dac_mute();
	__dlv_disable_hp_mute();
	__dlv_disable_lineout_mute();
}

/**
 * Disable LINE OUT replay mode
 *
 */
static void dlv_disable_line_out(void)
{
	//DUMP_CODEC_REGS("after dlv_disable_line_out\n");
}

/**
 * Enable LINE IN record mode

 *
 * insel:	1	line in left channel
 * insel:	2	line in right channel
 * insel:	3	line in left & right channels
 */
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

        dlv_write_reg(DLV_REG_GCR3, val);//GIL,GIR
        dlv_write_reg(DLV_REG_GCR4, val);//GIL,GIR
        LEAVE();
}

static void dlv_set_line_in(void)
{
    printk("[-- route --] %s\n", __FUNCTION__);

    dlv_write_reg_bit(DLV_REG_CR2, 0, CR2_NOMAD);
    dlv_write_reg_bit(DLV_REG_AGC1, 0, AGC1_AGCEN);//AGC1.AGC_EN->0
    dlv_write_reg_bit(DLV_REG_CR3, 0, CR3_MICDIFF);
    dlv_write_reg_bit(DLV_REG_PMR1, 0, PMR1_SB_AIP);

    schedule_timeout(2);

    dlv_write_reg_bit(DLV_REG_CR1, 1, CR1_BTL_MUTE);
    dlv_write_reg_bit(DLV_REG_CR1, 1, CR1_LINEOUT_MUTE);
    dlv_write_reg_bit(DLV_REG_PMR2, 1, PMR2_SB_LOUT);
    dlv_write_reg_bit(DLV_REG_PMR2, 1, PMR2_SB_BTL);

    switch_SB_LINE_IN(POWER_ON);
    __dlv_set_insel(LINE_INPUT);

    switch_SB_MIC1(POWER_OFF);
    switch_SB_ADC(POWER_ON);
}



static void dlv_enable_line_in_record(int insel)
{

	dlv_write_reg_bit(DLV_REG_AGC1, 0, AGC1_AGCEN);//AGC1.AGC_EN->0

	schedule_timeout(2);

	__dlv_set_mic_stereo();
	__dlv_enable_mic_diff();
	__dlv_switch_sb_line_in(POWER_ON);
	__dlv_set_insel(LINE_INPUT);
	__dlv_set_gim(0x3f);
	__dlv_set_gidr(0xf);
	__dlv_set_gidl(0xf);
	__dlv_switch_sb_adc(POWER_ON);

	//DUMP_CODEC_REGS("after dlv_enable_line_in\n");
}

/**
 * Disable LINE IN record mode
 *
 * Disable every channels independently ???
 */
static void dlv_disable_line_in_record(void)
{
	//DUMP_CODEC_REGS("after dlv_disable_line_in\n");
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
		dlv_sleep_wait_bitset(0xb, IFR_RUP, 100,__LINE__);
	}
	__dlv_switch_sb_line_in(POWER_ON);
	__dlv_switch_sb_btl(POWER_OFF);
	__dlv_switch_sb_bypass(POWER_ON);

	__dlv_set_outsel(BYPASS_PATH);

	__dlv_disable_dac_mute();
	__dlv_enable_lineout_mute();
	__dlv_disable_hp_mute();
	__dlv_enable_btl_mute();
	__dlv_disable_dac_right_only();
	__dlv_disable_nomad();
}

/**
 * Disable LINE IN bypass to HP
 *
 */
static void dlv_disable_line_in_bypass_hp(void)
{
}

/**
 * Enable LINE IN bypass to BTL
 *
 * Depend: LINE IN was enabled, BTL was enabled
 */
static void dlv_enable_line_in_bypass_btl(void)
{

	__dlv_switch_sb(POWER_ON);
	__dlv_switch_sb_sleep(POWER_ON);

	__dlv_set_16ohm_load();

	__dlv_switch_sb_line_out(POWER_ON);
	if (__dlv_get_sb_hp() != POWER_ON) {
		__dlv_switch_sb_hp(POWER_ON);
		dlv_sleep_wait_bitset(0xb, IFR_RUP, 100,__LINE__);
	}
	__dlv_switch_sb_line_in(POWER_ON);
	__dlv_switch_sb_btl(POWER_ON);
	__dlv_switch_sb_bypass(POWER_ON);

	__dlv_set_outsel(BYPASS_PATH);

	__dlv_disable_dac_mute();
	__dlv_disable_lineout_mute();
	__dlv_disable_hp_mute();
	__dlv_disable_btl_mute();
	__dlv_disable_dac_right_only();
	__dlv_disable_nomad();


	//dump_dlv_regs("after enable line in bypass btl\n");
}

/**
 * Disable LINE IN bypass to BTL
 *
 */
static void dlv_disable_line_in_bypass_btl(void)
{

	__dlv_set_outsel(DAC_OUTPUT);
	__dlv_switch_sb_bypass(POWER_OFF);
	__dlv_switch_sb_line_in(POWER_OFF);
}

/**
 * Enable digital record mix into DA
 *
 */
static void dlv_enable_rec_2_dac(void)
{
	__dlv_set_mix_rec_2_dac();
}

/**
 * Disable ditital record mix into DA.
 *
 */
static void dlv_disable_rec_2_dac(void)
{
	__dlv_set_mix_rec_only();
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


	dlv_write_reg(DLV_REG_CCR2, 0x29);

	/* Ensure CODEC is opened */
	__dlv_switch_sb_aip(POWER_ON);
	schedule_timeout(2);
	__dlv_switch_sb(POWER_ON);
	schedule_timeout(2);
	__dlv_switch_sb_sleep(POWER_ON);
	schedule_timeout(2);
	__dlv_switch_sb_adc(POWER_ON);
	schedule_timeout(2);

	//__dlv_enable_agc();
	__dlv_disable_agc();

	__dlv_switch_sb_micbias(POWER_ON);

	/* For altair, ear_mic is single-ended input. */
#ifdef CONFIG_JZ4760_ALTAIR
	//__dlv_disable_mic_diff();
	__dlv_enable_mic_diff();
#endif
	/* For z800, mic is differential input. */
#ifdef CONFIG_JZ4760_Z800
	__dlv_enable_mic_diff();
#endif

	__dlv_enable_mic_diff();

	/* For z800, main MIC is connected to MIC2 of JZ4760 */
	__dlv_set_insel(MIC2_TO_LR);

	__dlv_set_mic_mono();

	__dlv_enable_adc_right_only();
#if 0
	/* Depend on board situation. For z800, we set max. */
#if 0
	/* max */
	__dlv_set_gim(0x3f);
	__dlv_set_gidr(0x9f);
#endif

#if 1
	/* for record test */
	__dlv_set_gim(0x3f);
	__dlv_set_gidr(0x80);
#endif

#if 0
	/* min */
	__dlv_set_gim(0x0);
	__dlv_set_gidr(0x8f);
	//__dlv_set_gidr(0x80);
	//__dlv_set_gidl(0x0);
#endif
#endif

	__dlv_switch_sb_mic2(POWER_ON);

	if (first) {
		dlv_write_reg(0x15, 0x0); /* target: 0x0~0x3c */
		dlv_write_reg(0x16, (1 << 7) | (0x2<< 4) | dlv_read_reg(0x16));  /* noise gate[7 6:4] & hold[3:0] */
		dlv_write_reg(0x17, 0x0);			   /* ATK[7:4] and DCY[3:0] */
		dlv_write_reg(0x18, 0x1f);			   /* AGC_MAX */
		dlv_write_reg(0x19, 0x0);			   /* AGC_MIN */
		first = 0;
	}
	//__dlv_disable_agc();
	__dlv_enable_agc();

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
	__dlv_switch_sb_mic2(POWER_OFF);
	//DUMP_CODEC_REGS("leave dlv_disable_mic_2\n");
}

/**
 * Enable MIC 1 record mode
 *
 */
static void dlv_enable_mic_1(void)
{
	ENTER();


	dlv_write_reg(DLV_REG_CCR2, 0x29);

	/* Ensure CODEC is opened */
	__dlv_switch_sb_aip(POWER_ON);
	schedule_timeout(2);
	__dlv_switch_sb(POWER_ON);
	schedule_timeout(2);
	__dlv_switch_sb_sleep(POWER_ON);
	schedule_timeout(2);
	__dlv_switch_sb_adc(POWER_ON);
	schedule_timeout(2);

//	__dlv_enable_agc();
	__dlv_disable_agc();

	__dlv_switch_sb_micbias(POWER_ON);

	/* For altair, ear_mic is single-ended input. */
#ifdef CONFIG_JZ4760_ALTAIR
	__dlv_disable_mic_diff();
#endif
	/* For z800, mic is differential input. */
#ifdef CONFIG_JZ4760_Z800
	__dlv_enable_mic_diff();
#endif

#ifdef CONFIG_JZ4760_LEPUS
	__dlv_enable_mic_diff();
#endif

	/* For z800, main MIC is connected to MIC2 of JZ4760 */
	__dlv_set_insel(MIC1_TO_LR);

	__dlv_set_mic_mono();

	__dlv_enable_adc_right_only();

#if 0
	/* Depend on board situation. For z800, we set max. */
#if 0
	/* max */
	__dlv_set_gim(0x3f);
	__dlv_set_gidr(0x9f);
#endif

#if 0
	/* for record test */
	__dlv_set_gim(0x3f);
	__dlv_set_gidr(0x80);
#endif

#if 1
	/* min */
	__dlv_set_gim(0x0);
	__dlv_set_gidr(0x8f);
	//__dlv_set_gidl(0x0);
#endif
#endif

	__dlv_switch_sb_mic1(POWER_ON);

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
	__dlv_switch_sb_mic1(POWER_OFF);
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

#endif

static void dlv_set_standby(unsigned int sw)
{
	switch(g_current_out_dev) {
	case SND_DEVICE_HANDSET:

		if (sw == POWER_ON) {
			//printk("%s:%d enable receiver\n", __FUNCTION__, __LINE__);
			dlv_enable_receiver();
		} else {
			//printk("%s:%d disable btl\n", __FUNCTION__, __LINE__);
			//dlv_disable_btl();
			dlv_disable_receiver();
		}


	case (SND_DEVICE_SPEAKER + SND_DEVICE_HEADSET):
	case SND_DEVICE_HEADSET:

		if (sw == POWER_ON) {
			//printk("%s:%d enable hp out\n", __FUNCTION__, __LINE__);
			dlv_enable_hp_out();
		} else {
			//printk("%s:%d disable btl\n", __FUNCTION__, __LINE__);
				dlv_disable_btl();
		}

	case SND_DEVICE_SPEAKER:

		if (sw == POWER_ON) {
			//printk("%s:%d enable speaker\n", __FUNCTION__, __LINE__);

			/* 2010-06-28 Jason.
			 * To avoid noisy, do not open speaker after set device.
			 * The operation of openning speaker is at the beginning of write
			 */

			//dlv_enable_speaker();
			dlv_disable_btl();
		} else {
			//printk("%s:%d disable btl\n", __FUNCTION__, __LINE__);
			dlv_disable_btl();
		}
		break;

	default:
		printk("JZ DLV: Unkown ioctl argument in SND_SET_STANDBY (%d)\n", g_current_out_dev);

	}
}

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

/**
 * CODEC ioctl (simulated) routine
 *
 * Provide control interface for i2s driver
 */
static int jzdlv_ioctl(void *context, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

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
		turn_on_dac(5);
		__dlv_disable_hp_mute();
		dlv_enable_line_out();
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

	case CODEC_SET_STANDBY:
		dlv_set_standby(arg);
		break;

#if 0
	case CODEC_SET_DEVICE:
		dlv_set_device((struct snd_device_config *)arg);
		break;
#endif

	case CODEC_SET_SPEAKER_POWER:
		if (arg == 1) {
			dlv_enable_speaker();
			//printk("speaker!!!!\n");

                } else {
                  dlv_disable_speaker();
                  dlv_enable_hp_out();

		}
		break;

	case CODEC_SET_LINEIN2BTL:
		dlv_enable_line_in_bypass_btl();
		break;

	case CODEC_CLEAR_LINEIN2BTL:
		dlv_disable_line_in_bypass_btl();
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
#if 1  //allen open
		if (arg == USE_LINEIN)
			dlv_set_line_in();
		else if (arg == USE_MIC)
			board_set_record();
#else
		board_set_record();
#endif
		break;

	case CODEC_SET_REC_2_DAC:
		if (arg) {
			printk("JZ DLV: SET REC 2 ADC ---- %d\n", (int)arg);
			dlv_enable_line_in_record(3);
			dlv_enable_rec_2_dac();
		} else {
			printk("JZ DLV: SET REC 2 ADC ---- %d\n", (int)arg);
			dlv_disable_line_in_record();
			dlv_disable_rec_2_dac();
			dlv_enable_line_in_bypass_btl();
		}
		break;

	case CODEC_SET_REPLAY_RECORD:
		dlv_set_replay_record(arg,1);
		printk("JZ DLV: should not run here -- CODEC_SET_REPLAY_RECORD");
		//BUG_ON(1);
		break;

	case CODEC_SET_REPLAY_VOLUME:
//		greplay_volume = arg;
		dlv_set_replay_volume(arg);
		break;

	case CODEC_SET_MIC_VOLUME:
		dlv_enable_line_in_record(0);
		dlv_set_mic_volume(arg);
		break;

	case CODEC_SET_LINE:
		dlv_set_line(arg);
		break;

	case CODEC_EACH_TIME_INIT:
		dlv_each_time_init();//set in-codec
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


static struct work_struct dlv_irq_work;
static spinlock_t dlv_irq_lock;
//static int i = 0;
static int handling_scmc = 0;


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

	curr_vol = dlv_read_reg(DLV_REG_GCR1);  //read the current HP volume
	delay = VOL_DELAY_BASE * (0x20 - (curr_vol & 0x1f));

	dlv_write_reg(0x0c, 0x1f);
	dlv_write_reg(0x0d, 0x1f);

	printk("Short circut volume delay %d ms curr_vol=%x \n", delay,curr_vol);
	msleep(delay);

	//clear rdo rup
	__dlv_set_irq_flag((1 << IFR_RDO) | (1 << IFR_RUP));

	__dlv_switch_sb_hp(POWER_OFF);

	//wait rdo finish
	dlv_sleep_wait_bitset(0xb, IFR_RDO, 100,__LINE__);

	while (1) {
		dlv_ifr = __dlv_get_irq_flag();
		printk("waiting for SCMC recover finish ----- dlv_ifr = 0x%02x\n", dlv_ifr);
		if ((dlv_ifr & (1 << IFR_SCMC)) == 0)
			break;
		__dlv_set_irq_flag((1 << IFR_SCMC));
		msleep(10);
	}

	__dlv_switch_sb_hp(POWER_ON);
	// wait rup finish
	dlv_sleep_wait_bitset(0xb, IFR_RUP, 100,__LINE__);
	dlv_write_reg(0x0c, curr_vol);
	dlv_write_reg(0x0d, curr_vol);
	msleep(delay);
	// do not clear RDO and RUP
	//__dlv_set_irq_flag((1 << IFR_RDO) | (1 << IFR_RUP));
}


/**
 * CODEC work queue handler
 *
 * Handle bottom-half of SCMC & JACKE irq
 *
 */
static void dlv_irq_work_handler(struct work_struct *work)
{
	unsigned int	dlv_ifr;
	unsigned long	flags;
	DLV_LOCK();
	do {
		dlv_ifr = __dlv_get_irq_flag();

		if (dlv_ifr & (1 << IFR_SCMC)) {
			dlv_short_circut_handler();
		}

		dlv_ifr = __dlv_get_irq_flag();
#if 1
		/* Updata SCMC */
		__dlv_set_irq_flag((1 << IFR_SCMC));
//		gf_short_circuit_flow_run = 0;
		/* Unmask SCMC */
		__dlv_set_irq_mask(ICR_COMMON_MASK);
#endif
		printk("JZ DLV: Short circut detected! dlv_ifr = 0x%02x\n",dlv_ifr);

	} while(dlv_ifr & (1 << IFR_SCMC));

	handling_scmc = 0;
	DLV_UNLOCK();
}

/**
 * IRQ routine
 *
 * Now we are only interested in SCMC.
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

	/* check FIF0*/
//	printk("IRQ:aic REG_AIC_CR => 0x%x, REG_AIC_SR => 0x%x\n", REG_AIC_CR,REG_AIC_SR);
    if(__aic_transmit_underrun()){
            printk("FIFO underrun!\n");
            REG_AIC_SR &= ~AIC_SR_TUR;
    }

    if(__aic_receive_overrun()){
            printk("FIFO overflow!\n");
            REG_AIC_SR &= ~AIC_SR_ROR;
    }

	/* Mask all irq temporarily */
	__dlv_set_irq_mask(ICR_ALL_MASK);

	aic_reg = REG_AIC_SR;

	REG_AIC_SR = 0x78; // legacy ... need check

	if (!(dlv_ifr & ((1 << IFR_SCMC) | (1 << IFR_JACKE)))) {
		/* CODEC may generate irq with flag = 0xc0.
		 * We have to ingore it in this case as there is no mask for the reserve bit.
		 */
		//printk("JZ DLV: Unkown irq detected, mask = 0x%08x, flag = 0x%08x\n", dlv_icr, dlv_ifr);

		printk("AIC interrupt ??? AIC_SR = 0x%08x\n", aic_reg);

		__dlv_set_irq_mask(ICR_COMMON_MASK);

		spin_unlock_irqrestore(&dlv_irq_lock, flags);
		return IRQ_HANDLED;

	} else {
		spin_unlock_irqrestore(&dlv_irq_lock, flags);

		/* Handle SCMC and JACK in work queue. */
		schedule_work(&dlv_irq_work);

		return IRQ_HANDLED;
	}
}


extern int hp_in;

#ifdef EARPHONE_DETE

static struct timer_list hp_irq_timer;

static void hp_ack_timer(unsigned long data)
{
	if (
		(__gpio_get_pin(EARPHONE_DETE) != DETE_ACTIV_LEVEL) &&
		(__gpio_get_pin(AV_OUT_DETE) != DETE_ACTIV_LEVEL)
	)
  	{
		hp_in = 0;
	    printk("%s: hp not connected\n",__func__);
	    dlv_enable_speaker();
	}
  	else
  	{
	  	hp_in = 1;
	    printk("%s: hp connected\n",__func__);
    	dlv_disable_speaker();
  	}
}

static irqreturn_t hp_pnp_irq(int irq, void *dev_id)
{
  /* mask interrupt */
  __gpio_mask_irq(EARPHONE_DETE);
  __gpio_mask_irq(AV_OUT_DETE);
  //printk("hp dete irq");
	__gpio_disable_pull(EARPHONE_DETE);
	if(__gpio_get_pin(EARPHONE_DETE) != 0)
		__gpio_as_irq_low_level(EARPHONE_DETE);
	else
		__gpio_as_irq_high_level(EARPHONE_DETE);

	__gpio_disable_pull(AV_OUT_DETE);
	if(__gpio_get_pin(AV_OUT_DETE) != 0)
		__gpio_as_irq_low_level(AV_OUT_DETE);
	else
		__gpio_as_irq_high_level(AV_OUT_DETE);

  hp_irq_timer.expires = jiffies + 1*HZ;
  del_timer(&hp_irq_timer);
  add_timer(&hp_irq_timer);
  __gpio_unmask_irq(EARPHONE_DETE);
  __gpio_unmask_irq(AV_OUT_DETE);
  return IRQ_HANDLED;
}
#endif

static int __init init_dlv(void)
{
	int retval;

	printk("=====>enter %s\n", __func__);


	cpm_start_clock(CGM_AIC);

	spin_lock_init(&dlv_irq_lock);

	spin_lock_init(&g_dlv_sem_lock);

	INIT_WORK(&dlv_irq_work, dlv_irq_work_handler);

	INIT_WORK(&dlv_anti_pop_work, dlv_anti_pop_work_handler);

	register_jz_codecs((void *)jzdlv_ioctl);

	dlv_reset();
	retval = request_irq(IRQ_AIC, dlv_codec_irq, IRQF_DISABLED, "dlv_codec_irq", NULL);
	if (retval) {
		printk("JZ DLV: Could not get AIC CODEC irq %d\n", IRQ_AIC);
		return retval;
	}

#ifdef  EARPHONE_DETE

        init_timer(&hp_irq_timer);
        hp_irq_timer.function = hp_ack_timer;
        hp_irq_timer.data = 0;
        hp_irq_timer.expires = jiffies + HZ;
        add_timer(&hp_irq_timer);
        //dlv_set_replay_volume(95);

        __gpio_as_func0(EARPHONE_DETE);
        __gpio_as_input(EARPHONE_DETE);

        __gpio_as_func0(AV_OUT_DETE);
        __gpio_as_input(AV_OUT_DETE);


	#if 0
        __gpio_enable_pull(EARPHONE_DETE);
		if(__gpio_get_pin(EARPHONE_DETE) != 0)
          	__gpio_as_irq_fall_edge(EARPHONE_DETE);
        else
        	__gpio_as_irq_rise_edge(EARPHONE_DETE);
	#else
		__gpio_disable_pull(EARPHONE_DETE);
        if(__gpio_get_pin(EARPHONE_DETE) != 0)
        	__gpio_as_irq_low_level(EARPHONE_DETE);
        else
        	__gpio_as_irq_high_level(EARPHONE_DETE);

		__gpio_disable_pull(AV_OUT_DETE);
        if(__gpio_get_pin(AV_OUT_DETE) != 0)
        	__gpio_as_irq_low_level(AV_OUT_DETE);
        else
        	__gpio_as_irq_high_level(AV_OUT_DETE);
	#endif

        int ret;
        ret = request_irq(EARPHONE_DETE_IRQ, hp_pnp_irq,
            IRQF_DISABLED, "hp_pnp", NULL);
        if (ret) {
          printk("%s %d Could not get HP irq %d\n",__FILE__,__LINE__, EARPHONE_DETE_IRQ);
          return ret;
        }
#else
		hp_in = 0;
	    dlv_enable_speaker();

#endif
        return 0;
}

/**
 * Module exit
 */
static void __exit cleanup_dlv(void)
{
	free_irq(IRQ_AIC, NULL);
}

module_init(init_dlv);
module_exit(cleanup_dlv);
