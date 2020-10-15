/*
 * Linux/sound/oss/jz_dlv.c
 *
 * DLV CODEC driver for Ingenic Jz4750 MIPS processor
 *
 * 2009-12-xx	Steven <dsqiu@ingenic.cn>
 * 2010-01-xx	Jason <xwang@ingenic.cn>
 * 2010-11-xx   jbbi <jbbi@ingenic.cn>
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
#include <linux/vmalloc.h>

#include <asm/hardirq.h>
#include <asm/jzsoc.h>
//#include <mach/chip-aic.h>
//#include <linux/jz_audio.h>

#include "../jz47XX_codec.h"

/***************************************************************************************\
 *                                                                                     *
 *global variable and structure interface                                              *
 *                                                                                     *
\***************************************************************************************/

static unsigned int cur_route = -1;
static unsigned int keep_old_route = -1;

static struct workqueue_struct *dlv_work_queue;
static struct work_struct dlv_irq_work;
static spinlock_t dlv_irq_lock;
static int handling_scmc = 0;

static unsigned int g_current_out_dev;

#ifdef CONFIG_HP_SENSE_DETECT
static jz_hp_switch_data_t *g_switch_data = NULL;
#endif 

extern unsigned int DEFAULT_REPLAY_ROUTE;
extern unsigned int DEFAULT_RECORD_ROUTE;
extern unsigned int DEFAULT_CALL_RECORD_ROUTE;

/*---------------------*/
static jz_dlv_platform_data_t dlv_platform_data_init_val = {
	.dlv_replay_volume_base = 0,
	.dlv_record_volume_base = 0,
	.default_replay_route = ROUTE_COUNT,
	.default_record_route = ROUTE_COUNT,
	.default_call_record_route = ROUTE_COUNT,
	.dlv_set_device = NULL,
	.dlv_set_standby = NULL,
        .dlv_set_gpio_before_set_route = NULL,
	.dlv_set_gpio_after_set_route = NULL,
	.dlv_init_part = NULL,
	.dlv_turn_off_part = NULL,
	.dlv_shutdown_part = NULL,
	.dlv_reset_part = NULL,
	.dlv_suspend_part = NULL,
	.dlv_resume_part = NULL,
	.dlv_anti_pop_part = NULL,
	.dlv_board_mute = NULL,
};

static jz_dlv_platform_data_t *dlv_platform_data = &dlv_platform_data_init_val;

static int dlv_mute(int val);

/*=================== lock ============================*/
static struct semaphore *g_dlv_sem = 0;

#define DLV_DEBUG_SEM(x,y...) //printk(x,##y);

#define DLV_LOCK()							\
	do{								\
		if(g_dlv_sem)						\
			down(g_dlv_sem);				\
		DLV_DEBUG_SEM("dlvsemlock lock\n");			\
	}while(0)

#define DLV_UNLOCK()							\
	do{								\
		if(g_dlv_sem)						\
			up(g_dlv_sem);					\
		DLV_DEBUG_SEM("dlvsemlock unlock\n");			\
	}while(0)

#define DLV_LOCKINIT()							\
	do{								\
		if(g_dlv_sem == NULL)					\
			g_dlv_sem = (struct semaphore *)vmalloc(sizeof(struct semaphore)); \
		if(g_dlv_sem)						\
			init_MUTEX_LOCKED(g_dlv_sem);			\
		DLV_DEBUG_SEM("dlvsemlock init\n");			\
	}while(0)

#define DLV_LOCKDEINIT()						\
	do{								\
		if(g_dlv_sem)						\
			vfree(g_dlv_sem);				\
		g_dlv_sem = NULL;					\
		DLV_DEBUG_SEM("dlvsemlock deinit\n");			\
	}while(0)


/*==============================================================*/
/**
 * dlv_sleep
 *
 *if use in suspend and resume, should use delay
 */
//static int g_dlv_sleep_mode = 1;
static int g_dlv_sleep_mode = 0;
void dlv_sleep(int ms)
{
	if(g_dlv_sleep_mode)
		msleep(ms);
	else
		mdelay(ms);
}

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

static inline int dlv_sleep_wait_bitset(int reg, unsigned bit, int stime, int line)
{
	int count = 0, err= 0;
	while(!(dlv_read_reg(reg) & (1 << bit))) {
		//printk("DLV waiting reg(%2x) bit(%2x) set %d \n",reg, bit, line);
		dlv_sleep(stime);
		count++;
		if(count > 10){
			printk("%s %d timeout\n",__FILE__,line);
			err = 1;
			break;
		}
	}
	return err;
}

static inline void dlv_sleep_wait_bitclear(int reg, unsigned bit, int stime)
{
	int count = 0;

	while((dlv_read_reg(reg) & (1 << bit)))
	{
	      dlv_sleep(stime);
		  count++;
		  if(count > 10){
			  printk("%s %d timeout\n",__FILE__,__LINE__);
			  break;
		  }
	}
}

/***************************************************************************************\
 *                                                                                     *
 *debug part                                                                           *
 *                                                                                     *
\***************************************************************************************/
/*###############################################*/

#define DLV_DUMP_IOC_CMD		0
#define DLV_DUMP_ROUTE_REGS		0
#define DLV_DUMP_ROUTE_PART_REGS	0
#define DLV_DUMP_GAIN_PART_REGS		0
#define DLV_DUMP_ROUTE_NAME		1

/*##############################################*/

#if DLV_DUMP_IOC_CMD 
static void dlv_print_ioc_cmd(int cmd)
{
	int i;

	int cmd_arr[] = {
		CODEC_INIT,			CODEC_TURN_OFF,
		CODEC_SHUTDOWN,			CODEC_RESET,
		CODEC_SUSPEND,			CODEC_RESUME,
		CODEC_ANTI_POP, 		CODEC_SET_ROUTE,
 		CODEC_SET_DEVICE,		CODEC_SET_RECORD_RATE,
 		CODEC_SET_RECORD_DATA_WIDTH, 	CODEC_SET_MIC_VOLUME,
		CODEC_SET_RECORD_CHANNEL, 	CODEC_SET_REPLAY_RATE,
 		CODEC_SET_REPLAY_DATA_WIDTH,   	CODEC_SET_REPLAY_VOLUME,
		CODEC_SET_REPLAY_CHANNEL, 	CODEC_DAC_MUTE,
		CODEC_DEBUG_ROUTINE,		CODEC_SET_STANDBY
	};

	char *cmd_str[] = {
		"CODEC_INIT", 			"CODEC_TURN_OFF",
		"CODEC_SHUTDOWN", 		"CODEC_RESET",
		"CODEC_SUSPEND",		"CODEC_RESUME",
		"CODEC_ANTI_POP", 		"CODEC_SET_ROUTE",
		"CODEC_SET_DEVICE",		"CODEC_SET_RECORD_RATE",
		"CODEC_SET_RECORD_DATA_WIDTH", 	"CODEC_SET_MIC_VOLUME",
		"CODEC_SET_RECORD_CHANNEL", 	"CODEC_SET_REPLAY_RATE",
		"CODEC_SET_REPLAY_DATA_WIDTH", 	"CODEC_SET_REPLAY_VOLUME",
		"CODEC_SET_REPLAY_CHANNEL", 	"CODEC_DAC_MUTE",
		"CODEC_DEBUG_ROUTINE",		"CODEC_SET_STANDBY"
	};

	for ( i = 0; i < sizeof(cmd_arr) / sizeof(int); i++) {
		if (cmd_arr[i] == cmd) {
			printk("CODEC IOC: Command name : %s\n", cmd_str[i]);
			return;
		}
	}

	if (i == sizeof(cmd_arr) / sizeof(int)) {
		printk("CODEC IOC: command is not under control\n");
	}
}
#endif //DLV_DUMP_IOC_CMD 

#if DLV_DUMP_ROUTE_NAME
static void dlv_print_route_name(int route)
{
	int i;

	int route_arr[] = {
		ROUTE_ALL_CLEAR,
		ROUTE_REPLAY_CLEAR,
		ROUTE_RECORD_CLEAR,
		RECORD_MIC1_MONO_DIFF_WITH_BIAS,
		RECORD_MIC1_MONO_DIFF_WITHOUT_BIAS,
		RECORD_MIC2_MONO_DIFF_WITH_BIAS,
		RECORD_MIC2_MONO_DIFF_WITHOUT_BIAS,
		REPLAY_OUT,
		REPLAY_HP_STEREO,
		REPLAY_LINEOUT_MONO,
		REPLAY_BTL,
		BYPASS_MIC1_DIFF_WITH_BIAS_TO_OUT,
		BYPASS_MIC1_DIFF_WITH_BIAS_TO_HP,
		BYPASS_MIC1_DIFF_WITH_BIAS_TO_LINEOUT_MONO,
		BYPASS_MIC1_DIFF_WITH_BIAS_TO_BTL,
		BYPASS_MIC2_DIFF_WITH_BIAS_TO_OUT,
		BYPASS_MIC2_DIFF_WITH_BIAS_TO_HP,
		BYPASS_MIC2_DIFF_WITH_BIAS_TO_LINEOUT_MONO,
		BYPASS_MIC2_DIFF_WITH_BIAS_TO_BTL,
		BYPASS_LINEIN_TO_OUT,
		BYPASS_LINEIN_TO_HP,
		BYPASS_LINEIN_TO_LINEOUT_MONO,
		BYPASS_LINEIN_TO_BTL,
		RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_OUT,
		RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_HP,
		RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_LINEOUT_MONO,
		RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_BTL,
		RECORD_MIC2_MONO_DIFF_WITHOUT_BIAS_REPLAY_LINEOUT_MONO
	};

	char *route_str[] = {
		"ROUTE_ALL_CLEAR",
		"ROUTE_REPLAY_CLEAR",
		"ROUTE_RECORD_CLEAR",
		"RECORD_MIC1_MONO_DIFF_WITH_BIAS",
		"RECORD_MIC1_MONO_DIFF_WITHOUT_BIAS",
		"RECORD_MIC2_MONO_DIFF_WITH_BIAS",
		"RECORD_MIC2_MONO_DIFF_WITHOUT_BIAS",
		"REPLAY_OUT",
		"REPLAY_HP_STEREO",
		"REPLAY_LINEOUT_MONO",
		"REPLAY_BTL",
		"BYPASS_MIC1_DIFF_WITH_BIAS_TO_OUT",
		"BYPASS_MIC1_DIFF_WITH_BIAS_TO_HP",
		"BYPASS_MIC1_DIFF_WITH_BIAS_TO_LINEOUT_MONO",
		"BYPASS_MIC1_DIFF_WITH_BIAS_TO_BTL",
		"BYPASS_MIC2_DIFF_WITH_BIAS_TO_OUT",
		"BYPASS_MIC2_DIFF_WITH_BIAS_TO_HP",
		"BYPASS_MIC2_DIFF_WITH_BIAS_TO_LINEOUT_MONO",
		"BYPASS_MIC2_DIFF_WITH_BIAS_TO_BTL",
		"BYPASS_LINEIN_TO_OUT",
		"BYPASS_LINEIN_TO_HP",
		"BYPASS_LINEIN_TO_LINEOUT_MONO",
		"BYPASS_LINEIN_TO_BTL",
		"RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_OUT",
		"RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_HP",
		"RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_LINEOUT_MONO",
		"RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_BTL",
		"RECORD_MIC2_MONO_DIFF_WITHOUT_BIAS_REPLAY_LINEOUT_MONO",
	};

	for ( i = 0; i < sizeof(route_arr) / sizeof(unsigned int); i++) {
		if (route_arr[i] == route) {
			printk("\nCODEC SET ROUTE: Route name : %s\n", route_str[i]);
			return;
		}
	}

	if (i == sizeof(route_arr) / sizeof(unsigned int)) {
		printk("\nCODEC SET ROUTE: Route is not configed yet!\n");
	}
}
#endif //DLV_DUMP_ROUTE_NAME

void dump_dlv_regs(void)
{
	unsigned int i;
	unsigned char data;
	for (i = 0; i < 32; i++) {
		data = dlv_read_reg(i);
		printk("address = 0x%02x, data = 0x%02x\n", i, data);
	}
}

void dump_dlv_route_regs(void)
{
	unsigned int i;
	unsigned char data;
	for (i = 0x2; i < 0xA; i++) {
		data = dlv_read_reg(i);
		printk("address = 0x%02x, data = 0x%02x\n", i, data);
	}
}

void dump_dlv_gain_regs(void)
{
	unsigned int i;
	unsigned char data;
	for (i = 0xC; i < 0x15; i++) {
		data = dlv_read_reg(i);
		printk("address = 0x%02x, data = 0x%02x\n", i, data);
	}
}
/*=========================================================*/

#if DLV_DUMP_ROUTE_NAME
#define DUMP_ROUTE_NAME(route) dlv_print_route_name(route)
#else //DLV_DUMP_ROUTE_NAME
#define DUMP_ROUTE_NAME(route)
#endif //DLV_DUMP_ROUTE_NAME

/*-------------------*/
#if DLV_DUMP_IOC_CMD 
#define DUMP_IOC_CMD()								\
	do {									\
		printk("[dlv IOCTL]++++++++++++++++++++++++++++\n");		\
		printk("%s  cmd = %d, arg = %lu\n", __func__, cmd, arg); 	\
		dlv_print_ioc_cmd(cmd);						\
		printk("[dlv IOCTL]----------------------------\n");		\
										\
	} while (0)
#else //DLV_DUMP_IOC_CMD
#define DUMP_IOC_CMD()	
#endif //DLV_DUMP_IOC_CMD

#if DLV_DUMP_ROUTE_REGS
#define DUMP_ROUTE_REGS(value)							\
	do {									\
		printk("codec register dump,%s\tline:%d-----%s:\n",		\
		       __func__, __LINE__, value);				\
		dump_dlv_regs();						\
										\
	} while (0)
#else //DLV_DUMP_ROUTE_REGS
#define DUMP_ROUTE_REGS(value)
#endif //DLV_DUMP_ROUTE_REGS

#if DLV_DUMP_ROUTE_PART_REGS
#define DUMP_ROUTE_PART_REGS(value)						\
	do {									\
		if (mode != DISABLE) {						\
			printk("codec register dump,%s\tline:%d-----%s:\n", 	\
			       __func__, __LINE__, value);			\
			dump_dlv_route_regs();					\
		}								\
										\
	} while (0)
#else //DLV_DUMP_ROUTE_PART_REGS
#define DUMP_ROUTE_PART_REGS(value)
#endif //DLV_DUMP_ROUTE_PART_REGS

#if DLV_DUMP_GAIN_PART_REGS
#define DUMP_GAIN_PART_REGS(value)						\
	do {									\
		printk("codec register dump,%s\tline:%d-----%s:\n", 		\
		       __func__, __LINE__, value);				\
		dump_dlv_gain_regs();						\
										\
	} while (0)
#else //DLV_DUMP_GAIN_PART_REGS
#define DUMP_GAIN_PART_REGS(value)
#endif //DLV_DUMP_GAIN_PART_REGS

/***************************************************************************************\
 *                                                                                     *
 *route part and attibute                                                              *
 *                                                                                     *
\***************************************************************************************/
/*=========================power on==========================*/
static void dlv_set_route_ready(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	if(__dlv_get_sb_aip() == POWER_OFF)
	{
		__dlv_switch_sb_aip(POWER_ON);
	}
	/*wait a typical time 250ms to get into sleep mode*/
	if(__dlv_get_sb() == POWER_OFF)
	{
		__dlv_switch_sb(POWER_ON);
		mdelay(250);
	}
	/*wait a typical time 200ms for adc (450ms for dac) to get into normal mode*/
	if(__dlv_get_sb_sleep() == POWER_OFF)
	{
		__dlv_switch_sb_sleep(POWER_ON);
		if(mode == ROUTE_READY_FOR_ADC)
			mdelay(200);
		else
			mdelay(450);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

/*=================route part functions======================*/

static void dlv_set_mic1(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case MIC1_DIFF_WITH_MICBIAS:
		if(__dlv_get_sb_mic1() == POWER_OFF)
		{
			__dlv_switch_sb_mic1(POWER_ON);
			schedule_timeout(2);
		}
		if(__dlv_get_sb_micbias() == POWER_OFF)
		{
			__dlv_switch_sb_micbias(POWER_ON);
			schedule_timeout(2);
		}
		__dlv_enable_mic_diff();
		break;

	case MIC1_DIFF_WITHOUT_MICBIAS:
		if(__dlv_get_sb_mic1() == POWER_OFF)
		{
			__dlv_switch_sb_mic1(POWER_ON);
			schedule_timeout(2);
		}
		if(__dlv_get_sb_micbias() == POWER_ON)
		{
			__dlv_switch_sb_micbias(POWER_OFF);
			schedule_timeout(2);
		}
		__dlv_enable_mic_diff();
		break;

	case MIC1_SING_WITH_MICBIAS:
		if(__dlv_get_sb_mic1() == POWER_OFF)
		{
			__dlv_switch_sb_mic1(POWER_ON);
			schedule_timeout(2);
		}
		if(__dlv_get_sb_micbias() == POWER_OFF)
		{
			__dlv_switch_sb_micbias(POWER_ON);
			schedule_timeout(2);
		}
		__dlv_disable_mic_diff();
		break;

	case MIC1_SING_WITHOUT_MICBIAS:
		if(__dlv_get_sb_mic1() == POWER_OFF)
		{
			__dlv_switch_sb_mic1(POWER_ON);
			schedule_timeout(2);
		}
		if(__dlv_get_sb_micbias() == POWER_ON)
		{
			__dlv_switch_sb_micbias(POWER_OFF);
			schedule_timeout(2);
		}
		__dlv_disable_mic_diff();
		break;

	case MIC1_DISABLE:
		if(__dlv_get_sb_mic1() == POWER_ON)
		{
			__dlv_switch_sb_mic1(POWER_OFF);
			schedule_timeout(2);
		}
		break;

	default:
		printk("JZ_DLV: line: %d, mic1 mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_mic2(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case MIC2_DIFF_WITH_MICBIAS:
		if(__dlv_get_sb_mic2() == POWER_OFF)
		{
			__dlv_switch_sb_mic2(POWER_ON);
			schedule_timeout(2);
		}
		if(__dlv_get_sb_micbias() == POWER_OFF)
		{
			__dlv_switch_sb_micbias(POWER_ON);
			schedule_timeout(2);
		}
		__dlv_enable_mic_diff();
		break;

	case MIC2_DIFF_WITHOUT_MICBIAS:
		if(__dlv_get_sb_mic2() == POWER_OFF)
		{
			__dlv_switch_sb_mic2(POWER_ON);
			schedule_timeout(2);
		}
		if(__dlv_get_sb_micbias() == POWER_ON)
		{
			__dlv_switch_sb_micbias(POWER_OFF);
			schedule_timeout(2);
		}
		__dlv_enable_mic_diff();
		break;

	case MIC2_SING_WITH_MICBIAS:
		if(__dlv_get_sb_mic2() == POWER_OFF)
		{
			__dlv_switch_sb_mic1(POWER_ON);
			schedule_timeout(2);
		}
		if(__dlv_get_sb_micbias() == POWER_OFF)
		{
			__dlv_switch_sb_micbias(POWER_ON);
			schedule_timeout(2);
		}
		__dlv_disable_mic_diff();
		break;

	case MIC2_SING_WITHOUT_MICBIAS:
		if(__dlv_get_sb_mic2() == POWER_OFF)
		{
			__dlv_switch_sb_mic2(POWER_ON);
			schedule_timeout(2);
		}
		if(__dlv_get_sb_micbias() == POWER_ON)
		{
			__dlv_switch_sb_micbias(POWER_OFF);
			schedule_timeout(2);
		}
		__dlv_disable_mic_diff();
		break;

	case MIC2_DISABLE:
		if(__dlv_get_sb_mic2() == POWER_ON)
		{
			__dlv_switch_sb_mic2(POWER_OFF);
			schedule_timeout(2);
		}
		break;

	default:
		printk("JZ_DLV: line: %d, mic2 mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_linein(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){
		
	case LINEIN_WITHOUT_BYPASS:
		if(__dlv_get_sb_line_in() == POWER_OFF)
		{
			__dlv_switch_sb_line_in(POWER_ON);
			schedule_timeout(2);
		}
		if(__dlv_get_sb_bypass() == POWER_ON)
		{
		   __dlv_switch_sb_bypass(POWER_OFF);
		   schedule_timeout(2);
		}
		break;

	case LINEIN_WITH_BYPASS:
		if(__dlv_get_sb_line_in() == POWER_OFF)
		{
			__dlv_switch_sb_line_in(POWER_ON);
			schedule_timeout(2);
		}
		if(__dlv_get_sb_bypass() == POWER_OFF)
		{
			__dlv_switch_sb_bypass(POWER_ON);
			schedule_timeout(2);
		}
		break;

	case LINEIN_DISABLE:
		if(__dlv_get_sb_line_in() == POWER_ON)
		{
			__dlv_switch_sb_line_in(POWER_OFF);
			schedule_timeout(2);
		}
		break;
		
	default:
		printk("JZ_DLV: line: %d, linein mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_agc(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case AGC_ENABLE:
		__dlv_enable_agc();
		break;
		
	case AGC_DISABLE:
		__dlv_disable_agc();
		break;

	default:
		printk("JZ_DLV: line: %d, agc mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_record_mux(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case RECORD_MUX_MIC1_TO_LR:
		__dlv_set_mic_mono();
		__dlv_set_insel(MIC1_TO_LR);
		break;

	case RECORD_MUX_MIC2_TO_LR:
		__dlv_set_mic_mono();
		__dlv_set_insel(MIC2_TO_LR);
		break;

	case RECORD_MUX_MIC1_TO_R_MIC2_TO_L:
		__dlv_set_mic_stereo();
		__dlv_set_insel(MIC1_TO_R_MIC2_TO_L);
		break;

	case RECORD_MUX_MIC2_TO_R_MIC1_TO_L:
		__dlv_set_mic_stereo();
		__dlv_set_insel(MIC2_TO_R_MIC1_TO_L);
		break;

	case RECORD_MUX_LINE_IN:
		__dlv_set_insel(BYPASS_PATH);
		break;

	default:
		printk("JZ_DLV: line: %d, record mux mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_adc(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case ADC_STEREO:
		if(__dlv_get_sb_adc() == POWER_OFF)
		{
			__dlv_switch_sb_adc(POWER_ON);
			schedule_timeout(2);
		}
		__dlv_disable_adc_right_only();
		break;

	case ADC_MONO:
		if(__dlv_get_sb_adc() == POWER_OFF)
		{
			__dlv_switch_sb_adc(POWER_ON);
			schedule_timeout(2);
		}
		__dlv_enable_adc_right_only();
		break;

	case ADC_DISABLE:
		if(__dlv_get_sb_adc() == POWER_ON)
		{
			__dlv_switch_sb_adc(POWER_OFF);
			schedule_timeout(2);
		}
		break;

	default:
		printk("JZ_DLV: line: %d, adc mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_record_mixer(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case RECORD_MIXER_MIX1_INPUT_ONLY:
		__dlv_set_mix1_mode(MIX1_RECORD_INPUT_ONLY);
		break;

	case RECORD_MIXER_MIX1_INPUT_AND_DAC:
		__dlv_set_mix1_mode(MIX1_RECORD_INPUT_AND_DAC);
		break;

	default:
		printk("JZ_DLV: line: %d, record mixer mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_replay_mixer(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case REPLAY_MIXER_PLAYBACK_DAC_ONLY:
		__dlv_set_mix2_mode(MIX2_PLAYBACK_DAC_ONLY);
		break;

	case REPLAY_MIXER_PLAYBACK_DAC_AND_ADC:
		__dlv_set_mix2_mode(MIX2_PLAYBACK_DAC_AND_ADC);
		break;

	default:
		printk("JZ_DLV: line: %d, replay mixer mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_dac(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case DAC_STEREO:
		if(__dlv_get_sb_dac() == POWER_OFF){
			__dlv_switch_sb_dac(POWER_ON);
			udelay(500);
		}
		if(__dlv_get_dac_mute()){
			/* clear IFR_GUP */
			__dlv_set_irq_flag(1 << IFR_GUP);
			/* turn on dac */
			__dlv_disable_dac_mute();
			/* wait IFR_GUP set */
			dlv_sleep_wait_bitset(0xb, IFR_GUP, 100,__LINE__);
		}
		__dlv_disable_dac_right_only();
		break;

	case DAC_MONO:
		if(__dlv_get_sb_dac() == POWER_OFF){
			__dlv_switch_sb_dac(POWER_ON);
			udelay(500);
		}
		if(__dlv_get_dac_mute()){
			/* clear IFR_GUP */
			__dlv_set_irq_flag(1 << IFR_GUP);
			/* turn on dac */
			__dlv_disable_dac_mute();
			/* wait IFR_GUP set */
			dlv_sleep_wait_bitset(0xb, IFR_GUP, 100,__LINE__);
		}
		__dlv_enable_dac_right_only();
		break;

	case DAC_DISABLE:
		if(__dlv_get_sb_dac() == POWER_ON){
			if (!(__dlv_get_dac_mute())){
				/* clear IFR_GDO */
				__dlv_set_irq_flag(1 << IFR_GDO);
				/* turn off dac */
				__dlv_enable_dac_mute();
				/* wait IFR_GDO set */
				dlv_sleep_wait_bitset(0xb, IFR_GDO, 100,__LINE__);
			}
			__dlv_switch_sb_dac(POWER_OFF);
		}
		break;

	default:
		printk("JZ_DLV: line: %d, dac mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_replay_filter(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case REPLAY_FILTER_STEREO:
		__dlv_disable_dac_m2s();
		break;

	case REPLAY_FILTER_MONO:
		__dlv_enable_dac_m2s();
		break;

	default:
		printk("JZ_DLV: line: %d, replay filter mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_replay_mux(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case REPLAY_MUX_MIC1_TO_LR:
		__dlv_set_outsel(MIC1_TO_LR);
		break;

	case REPLAY_MUX_MIC2_TO_LR:
		__dlv_set_outsel(MIC2_TO_LR);
		break;

	case REPLAY_MUX_MIC1_TO_R_MIC2_TO_L:
		__dlv_set_outsel(MIC1_TO_R_MIC2_TO_L);
		break;

	case REPLAY_MUX_MIC2_TO_R_MIC1_TO_L:
		__dlv_set_outsel(MIC2_TO_R_MIC1_TO_L);
		break;

	case REPLAY_MUX_BYPASS_PATH:
		__dlv_set_outsel(BYPASS_PATH);
		break;

	case REPLAY_MUX_DAC_OUTPUT:
		__dlv_set_outsel(DAC_OUTPUT);
		break;

	default:
		printk("JZ_DLV: line: %d, replay mux mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_hp(int mode)
{
	int load_flag = 0;

	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case HP_ENABLE:
		__dlv_set_16ohm_load();
		__dlv_enable_nomad();
		//__dlv_disable_hp_mute();  // delete by jintao,you can change it only by change volume 
		mdelay(1);
		if(__dlv_get_sb_hp() == POWER_OFF)
		{
			if(__dlv_get_sb_bypass() == POWER_ON){

				__dlv_switch_sb_bypass(POWER_OFF);
				mdelay(1);

				// enable dac mute to reduce pop noise
				if((__dlv_get_dac_mute() == 0) && (__dlv_get_sb_dac() == POWER_ON)){
					
					/* enable dac mute */
					__dlv_set_irq_flag(1 << IFR_GDO);
					__dlv_enable_dac_mute();
					dlv_sleep_wait_bitset(0xb, IFR_GDO, 100,__LINE__);
					
					udelay(500);

					/* turn on sb_hp */
					__dlv_set_irq_flag(1 << IFR_RUP);
					__dlv_switch_sb_hp(POWER_ON);
					dlv_sleep_wait_bitset(0xb, IFR_RUP, 100,__LINE__);

					/*disable dac mute*/
					__dlv_set_irq_flag(1 << IFR_GUP);
					__dlv_disable_dac_mute();
					dlv_sleep_wait_bitset(0xb, IFR_GUP, 100,__LINE__);
					
				} else {

					/* turn on sb_hp */
					__dlv_set_irq_flag(1 << IFR_RUP);
					__dlv_switch_sb_hp(POWER_ON);
					dlv_sleep_wait_bitset(0xb, IFR_RUP, 100,__LINE__);
				}

				__dlv_switch_sb_bypass(POWER_ON);

			} else { //bypass power off

				// enable dac mute to reduce pop noise
				if((__dlv_get_dac_mute() == 0) && (__dlv_get_sb_dac() == POWER_ON)){
					
					/* enable dac mute */
					__dlv_set_irq_flag(1 << IFR_GDO);
					__dlv_enable_dac_mute();
					dlv_sleep_wait_bitset(0xb, IFR_GDO, 100,__LINE__);

					udelay(500);

					/* turn on sb_hp */
					__dlv_set_irq_flag(1 << IFR_RUP);
					__dlv_switch_sb_hp(POWER_ON);
					dlv_sleep_wait_bitset(0xb, IFR_RUP, 100,__LINE__);

					/*disable dac mute*/
					__dlv_set_irq_flag(1 << IFR_GUP);
					__dlv_disable_dac_mute();
					dlv_sleep_wait_bitset(0xb, IFR_GUP, 100,__LINE__);
					
				} else {

					/* turn on sb_hp */
					__dlv_set_irq_flag(1 << IFR_RUP);
					__dlv_switch_sb_hp(POWER_ON);
					dlv_sleep_wait_bitset(0xb, IFR_RUP, 100,__LINE__);
				}
			}
		}
		break;

	case HP_DISABLE:
		if(__dlv_get_sb_hp() == POWER_ON)
		{
			/* set 16ohm load to keep away from the bug can not waited RDO */
			if(__dlv_get_load() == LOAD_10KOHM)
			{
				__dlv_set_16ohm_load();
				load_flag = 1;
			}

			if(__dlv_get_sb_bypass() == POWER_ON){

				__dlv_switch_sb_bypass(POWER_OFF);
				mdelay(1);

				if((__dlv_get_dac_mute() == 0) && (__dlv_get_sb_dac() == POWER_ON)){
			
					/* enable dac mute */
					__dlv_set_irq_flag(1 << IFR_GDO);
					__dlv_enable_dac_mute();
					dlv_sleep_wait_bitset(0xb, IFR_GDO, 100,__LINE__);
					
					udelay(500);

					/* turn off sb_hp */
					__dlv_set_irq_flag(1 << IFR_RDO);
					__dlv_switch_sb_hp(POWER_OFF);   
					dlv_sleep_wait_bitset(0xb, IFR_RDO, 100,__LINE__);

					/*disable dac mute*/
					__dlv_set_irq_flag(1 << IFR_GUP);
					__dlv_disable_dac_mute();
					dlv_sleep_wait_bitset(0xb, IFR_GUP, 100,__LINE__);

				} else {

					/* turn off sb_hp */
					__dlv_set_irq_flag(1 << IFR_RDO);
					__dlv_switch_sb_hp(POWER_OFF);   
					dlv_sleep_wait_bitset(0xb, IFR_RDO, 100,__LINE__);
				}

				__dlv_switch_sb_bypass(POWER_OFF);

			} else {

				if((__dlv_get_dac_mute() == 0) && (__dlv_get_sb_dac() == POWER_ON)){
			
					/* enable dac mute */
					__dlv_set_irq_flag(1 << IFR_GDO);
					__dlv_enable_dac_mute();
					dlv_sleep_wait_bitset(0xb, IFR_GDO, 100,__LINE__);
					
					udelay(500);

					/* turn off sb_hp */
					__dlv_set_irq_flag(1 << IFR_RDO);
					__dlv_switch_sb_hp(POWER_OFF);   
					dlv_sleep_wait_bitset(0xb, IFR_RDO, 100,__LINE__);


					/*disable dac mute*/
					__dlv_set_irq_flag(1 << IFR_GUP);
					__dlv_disable_dac_mute();
					dlv_sleep_wait_bitset(0xb, IFR_GUP, 100,__LINE__);

				} else {

					/* turn off sb_hp */
					__dlv_set_irq_flag(1 << IFR_RDO);
					__dlv_switch_sb_hp(POWER_OFF);   
					dlv_sleep_wait_bitset(0xb, IFR_RDO, 100,__LINE__);
				}	
			}
			if(load_flag)
				__dlv_set_10kohm_load();

			__dlv_enable_hp_mute();
		}
		break;

	default:
		printk("JZ_DLV: line: %d, hp mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_lineout(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case LINEOUT_STEREO:
		__dlv_set_10kohm_load();
		__dlv_disable_nomad();
		break;

	case LINEOUT_MONO:
		__dlv_set_10kohm_load();
		if(__dlv_get_sb_line_out() == POWER_OFF)
		{
			__dlv_switch_sb_line_out(POWER_ON);
			schedule_timeout(2);
		}
		__dlv_disable_lineout_mute();
		__dlv_disable_nomad();
		break;

	case LINEOUT_DISABLE:
		if(__dlv_get_sb_line_out() == POWER_ON)
		{
			__dlv_switch_sb_line_out(POWER_OFF);
			schedule_timeout(2);
		}
		break;

	default:
		printk("JZ_DLV: line: %d, lineout mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_btl(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case BTL_ENABLE:
		if(__dlv_get_sb_btl() == POWER_OFF)
		{
			__dlv_switch_sb_btl(POWER_ON);
			schedule_timeout(2);
		}
		__dlv_disable_btl_mute();
		break;

	case BTL_DISABLE:
		if(__dlv_get_sb_btl() == POWER_ON)
		{
			__dlv_switch_sb_btl(POWER_OFF);
			schedule_timeout(2);
		}
		break;

	default:
		printk("JZ_DLV: line: %d, btl mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

/*=================route attibute(gain) functions======================*/

static void dlv_set_gain_mic1(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");

	if (gain < 0)
		gain = 0;
	else if (gain > 20)
		gain = 20;

	val = gain / 4;

	__dlv_set_gm1(val);

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_mic2(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");

	if (gain < 0)
		gain = 0;
	else if (gain > 20)
		gain = 20;

	val = (gain % 20) / 4;

	__dlv_set_gm2(val);

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_linein_left(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");
	
	if (gain > 6)
		gain = 6;
	else if (gain < -25)
		gain = -25;

	val = (6 - gain);

	__dlv_set_gil(val);

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_linein_right(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");

	if (gain > 6)
		gain = 6;
	else if (gain < -25)
		gain = -25;

	val = (6 - gain);

	__dlv_set_gir(val);

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_adc_left(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");

	if (gain < 0)
		gain = 0;
	else if ( gain > 23)
		gain = 23;
	
	val = gain;

	__dlv_set_gidl(val);

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_adc_right(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");

	if (gain < 0)
		gain = 0;
	else if ( gain > 23)
		gain = 23;
	
	val = gain;

	__dlv_set_gidr(val);

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_record_mixer(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");
	
	if (gain > 0)
		gain = 0;
	else if (gain < -31)
		gain = -31;

	val = -gain;

	__dlv_set_gimix(val);

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_replay_mixer(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");

	if (gain > 0)
		gain = 0;
	else if (gain < -31)
		gain = -31;

	val = -gain;

	__dlv_set_gomix(val);

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_dac_left(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");

	if (gain > 0)
		gain = 0;
	else if (gain < -31)
		gain = -31;

	val = -gain;

	__dlv_set_godl(val);

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_dac_right(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");

	if (gain > 0)
		gain = 0;
	else if (gain < -31)
		gain = -31;

	val = -gain;

	__dlv_set_godr(val);

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_hp_left(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");

	if (gain > 6)
		gain = 6;
	else if (gain < -25)
		gain = -25;

	val = (6 - gain);

	__dlv_set_gol(val);

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_hp_right(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");

	if (gain > 6)
		gain = 6;
	else if (gain < -25)
		gain = -25;

	val = (6 - gain);

	__dlv_set_gor(val);

	DUMP_GAIN_PART_REGS("leave");
}
/***************************************************************************************\
 *                                                                                     *
 *dlv route                                                                            *
 *                                                                                     *
\***************************************************************************************/

static void dlv_set_route_base(const void *arg)
{	
	route_conf_base *conf = (route_conf_base *)arg;

	/*codec turn on sb and sb_sleep*/
	if (conf->route_ready_mode)
		dlv_set_route_ready(conf->route_ready_mode);

	/*--------------route---------------*/
	/* record path */
	if (conf->route_mic1_mode)
		dlv_set_mic1(conf->route_mic1_mode);

	if (conf->route_mic2_mode)
		dlv_set_mic2(conf->route_mic2_mode);

	if (conf->route_linein_mode)
		dlv_set_linein(conf->route_linein_mode);

	if (conf->route_record_mux_mode)
		dlv_set_record_mux(conf->route_record_mux_mode);

	if (conf->route_adc_mode)
		dlv_set_adc(conf->route_adc_mode);

	if (conf->route_record_mixer_mode)
		dlv_set_record_mixer(conf->route_record_mixer_mode);

	/* replay path */
	if (conf->route_replay_mixer_mode)
		dlv_set_replay_mixer(conf->route_replay_mixer_mode);

	if (conf->route_replay_filter_mode)
		dlv_set_replay_filter(conf->route_replay_filter_mode);

	if (conf->route_dac_mode)
		dlv_set_dac(conf->route_dac_mode);

	if (conf->route_replay_mux_mode)
		dlv_set_replay_mux(conf->route_replay_mux_mode);

	if (conf->route_hp_mode)
		dlv_set_hp(conf->route_hp_mode);

	if (conf->route_lineout_mode)
		dlv_set_lineout(conf->route_lineout_mode);

	if (conf->route_btl_mode)
		dlv_set_btl(conf->route_btl_mode);

	/*----------------attibute-------------*/
	/* auto gain */
	if (conf->attibute_agc_mode)
		dlv_set_agc(conf->attibute_agc_mode);

	/* gain , use 32 instead of 0 */
	if (conf->attibute_mic1_gain) {
		if (conf->attibute_mic1_gain == 32)
			dlv_set_gain_mic1(0);
		else 
			dlv_set_gain_mic1(conf->attibute_mic1_gain);
	}

	if (conf->attibute_mic2_gain) {
		if (conf->attibute_mic2_gain == 32)
			dlv_set_gain_mic2(0);
		else
			dlv_set_gain_mic2(conf->attibute_mic2_gain);
	}

	if (conf->attibute_linein_l_gain) {
		if (conf->attibute_linein_l_gain == 32)
			dlv_set_gain_linein_left(0);
		else
			dlv_set_gain_linein_left(conf->attibute_linein_l_gain);
	}

	if (conf->attibute_linein_r_gain) {
		if (conf->attibute_linein_r_gain == 32)
			dlv_set_gain_linein_right(0);
		else 
			dlv_set_gain_linein_right(conf->attibute_linein_r_gain);
	}

	if (conf->attibute_adc_l_gain) {
		if (conf->attibute_adc_l_gain == 32)
			dlv_set_gain_adc_left(0);
		else
			dlv_set_gain_adc_left(conf->attibute_adc_l_gain);
	}

	if (conf->attibute_adc_r_gain) {
		if (conf->attibute_adc_r_gain == 32)
			dlv_set_gain_adc_right(0);
		else 
			dlv_set_gain_adc_right(conf->attibute_adc_r_gain);
	}

	if (conf->attibute_record_mixer_gain) {
		if (conf->attibute_record_mixer_gain == 32)
			dlv_set_gain_record_mixer(0);
		else 
			dlv_set_gain_record_mixer(conf->attibute_record_mixer_gain);
	}

	if (conf->attibute_replay_mixer_gain) {
		if (conf->attibute_replay_mixer_gain == 32)
			dlv_set_gain_replay_mixer(0);
		else 
			dlv_set_gain_replay_mixer(conf->attibute_replay_mixer_gain);
	}

	if (conf->attibute_dac_l_gain) {
		if (conf->attibute_dac_l_gain == 32)
			dlv_set_gain_dac_left(0);
		else
			dlv_set_gain_dac_left(conf->attibute_dac_l_gain);
	}
	
	if (conf->attibute_dac_r_gain) {
		if (conf->attibute_dac_r_gain == 32)
			dlv_set_gain_dac_right(0);
		else
			dlv_set_gain_dac_right(conf->attibute_dac_r_gain);
	}

	if (conf->attibute_hp_l_gain) {
		if (conf->attibute_hp_l_gain == 32)
			dlv_set_gain_hp_left(0);
		else
			dlv_set_gain_hp_left(conf->attibute_hp_l_gain);
	}

	if (conf->attibute_hp_r_gain) {
		if (conf->attibute_hp_r_gain == 32)
			dlv_set_gain_hp_right(0);
		else
			dlv_set_gain_hp_right(conf->attibute_hp_r_gain);
	}
}

/***************************************************************************************\
 *                                                                                     *
 *ioctl support function                                                               *
 *                                                                                     *
\***************************************************************************************/

/*------------------sub fun-------------------*/

/**
 * CODEC set gpio before set route and dlv set gpio after set route
 *
 * these two function below is just demo frames, they should be realized 
 * depend on difficent boards, they should not be modifiy here
 *
 **/

static int dlv_set_gpio_before_set_route(int route)
{
	printk("JZ_DLV: waring, %s() is a default function\n", __func__);

	switch(route){

	case ROUTE_ALL_CLEAR:
		break;

	case RECORD_MIC1_MONO_DIFF_WITH_BIAS:
		break;

	case REPLAY_HP_STEREO:
		break;

	/* and so on */

	default:
		printk("%s: dlv set route gpio error!, undecleard route\n", __func__);
	}
	
	return 0;
}

static int dlv_set_gpio_after_set_route(int route)
{
	printk("JZ_DLV: waring, %s() is a default function\n", __func__);

	switch(route){

	case ROUTE_ALL_CLEAR:
		break;

	case RECORD_MIC1_MONO_DIFF_WITH_BIAS:
		break;

	case REPLAY_HP_STEREO:
		break;
		
	/* and so on */

	default:
		printk("%s: dlv set route gpio error!, undecleard route\n", __func__);
	}

	return 0;
}

/*-----------------main fun-------------------*/

int dlv_set_route(int route)
{
	int i = 0;

	DUMP_ROUTE_REGS("enter");

	/* set gpio befor set route */
	if(dlv_platform_data->dlv_set_gpio_before_set_route)
	{
		if(dlv_platform_data->dlv_set_gpio_before_set_route(route))
		{
			dlv_set_gpio_before_set_route(route);
		}
	} else
		dlv_set_gpio_before_set_route(route);

	/* set route */
	DUMP_ROUTE_NAME(route);

	if(cur_route != route)
	{
		for (i = 0; i < ROUTE_COUNT; i ++)
		{
			if (route == dlv_route_info[i].route_name)
			{
				/* set route */
				dlv_set_route_base(dlv_route_info[i].route_conf);
				/* keep_old_route is used in resume part */
				keep_old_route = cur_route;
				/* change cur_route */
				cur_route = route;
				break;
			}
		}
		if (i == ROUTE_COUNT)
			printk("SET_ROUTE: dlv set route error!, undecleard route, route = %d\n", route);
	} else 
		printk("SET_ROUTE: need not to set!, current route is route now!\n");
	
	/* set gpio after set route */
	if(dlv_platform_data->dlv_set_gpio_after_set_route)
	{
		if(dlv_platform_data->dlv_set_gpio_after_set_route(route))
		{
			dlv_set_gpio_after_set_route(route);
		}
	} else
		dlv_set_gpio_after_set_route(route);

	DUMP_ROUTE_REGS("leave");

	return cur_route;
}

/*----------------------------------------*/
/****** dlv_init ********/
/**
 * CODEC dlv init part
 *
 * it will do the initialization as default, it can be recode 
 * depend on difficent boards if necessary
 *
 **/

static int dlv_init_part(void)
{
	printk("JZ_DLV: waring, %s() is a default function\n", __func__);

	__dlv_set_int_form(ICR_INT_HIGH_8CYCLES);

	__dlv_set_irq_mask(ICR_COMMON_MASK);
	__dlv_set_irq_flag(0x3f);

	__dlv_set_12m_crystal();

	return 0;
}

static int dlv_init(void)
{
	int ret;

	DLV_LOCKINIT();

	/* set default route */
	if(dlv_platform_data->default_replay_route && (dlv_platform_data->default_replay_route != ROUTE_COUNT))
		DEFAULT_REPLAY_ROUTE = dlv_platform_data->default_replay_route;

	if(dlv_platform_data->default_record_route && (dlv_platform_data->default_record_route != ROUTE_COUNT))
		DEFAULT_RECORD_ROUTE = dlv_platform_data->default_record_route;

	if(dlv_platform_data->default_call_record_route && (dlv_platform_data->default_call_record_route != ROUTE_COUNT))
		DEFAULT_CALL_RECORD_ROUTE = dlv_platform_data->default_call_record_route;

	g_current_out_dev = DEFAULT_REPLAY_ROUTE;

	/* dlv init */
	if(dlv_platform_data->dlv_init_part)
	{
		ret = dlv_platform_data->dlv_init_part();
		if(ret)
		{
			ret = dlv_init_part();
		}
	} else
		ret = dlv_init_part();

	return ret;
}

/****** dlv_turn_off ********/
/**
 * CODEC dlv turn off part
 *
 * it will turn off the codec by modes as default, it can be recode 
 * depend on difficent boards if necessary
 *
 **/
static int dlv_turn_off_part(int mode)
{
	int ret;
	int route = keep_old_route;

	printk("JZ_DLV: waring, %s() is a default function\n", __func__);

	if ((mode & REPLAY) && (mode & RECORD)) {
		printk("JZ DLV: Close REPLAY & RECORD\n");
		/* set 16ohm load to keep away from the bug can not waited RDO */
		if(__dlv_get_load() == LOAD_10KOHM)
			__dlv_set_16ohm_load();

		ret = dlv_set_route(ROUTE_ALL_CLEAR);
		if(ret != ROUTE_ALL_CLEAR)
		{
			printk("JZ CODEC: dlv_turn_off_part replay & record mode error!\n");
			return -1;
		}
	} else if (mode & REPLAY) {
		printk("JZ DLV: Close REPLAY\n");
		/* set 16ohm load to keep away from the bug can not waited RDO */
/*  
		if(__dlv_get_load() == LOAD_10KOHM)
			__dlv_set_16ohm_load();
*/
		ret = dlv_set_route(ROUTE_REPLAY_CLEAR);
		if(ret != ROUTE_REPLAY_CLEAR)
		{
			printk("JZ CODEC: dlv_turn_off_part replay mode error!\n");
			return -1;
		}

	} else if (mode & RECORD) {
		printk("JZ DLV: Close RECORD\n");
		ret = dlv_set_route(ROUTE_RECORD_CLEAR);
		if(ret != ROUTE_RECORD_CLEAR)
		{
			printk("JZ CODEC: dlv_turn_off_part record mode error!\n");
			return -1;
		}
		
		ret = dlv_set_route(route);
		if(ret != route)
		{
			printk("JZ CODEC: %s record mode error!\n", __func__);
			return -1;
		}
	}

	return 0;
}

static int dlv_turn_off(int mode)
{
	int ret;

	if(dlv_platform_data->dlv_turn_off_part)
	{
		ret = dlv_platform_data->dlv_turn_off_part(mode);
		if(ret)
		{
			ret = dlv_turn_off_part(mode);
		}
	} else
		ret = dlv_turn_off_part(mode);

	//DLV_LOCKDEINIT();  

	return ret;
}

/****** dlv_shutdown *******/
/**
 * CODEC dlv shutdown part
 *
 * it will shutdown the gpio when system shutdown,
 * it can be recode depend on difficent boards if necessary
 *
 **/

static int dlv_shutdown_part(void)
{
	printk("JZ_DLV: waring, %s() is a default function\n", __func__);

	return 0;
}

static int dlv_shutdown(void)
{
	int ret;
		
	if(dlv_platform_data->dlv_shutdown_part)
	{
		ret = dlv_platform_data->dlv_shutdown_part();
		if(ret)
		{
			ret = dlv_shutdown_part();
		}
	} else
		ret = dlv_shutdown_part();

	return ret;
}

/****** dlv_reset **********/
/**
 * CODEC dlv reset part
 *
 * it will run to set the codec when codec power on as default,
 * it can be recode depend on difficent boards if necessary
 *
 **/
static int dlv_reset_part(void)
{
	printk("JZ_DLV: waring, %s() is a default function\n", __func__);

	/* select serial interface and work mode of adc and dac */
	__dlv_select_adc_digital_interface(SERIAL_INTERFACE);
	__dlv_select_dac_digital_interface(SERIAL_INTERFACE);

	__dlv_select_adc_work_mode(I2S_MODE);
	__dlv_select_dac_work_mode(I2S_MODE);

	/* reset codec ready for set route */
	dlv_set_route_ready(ROUTE_READY_FOR_DAC);

	__dlv_set_irq_mask(0x2f);
	printk("__dlv_get_irq_mask=0x%2x\n",__dlv_get_irq_mask());

	return 0;
}

static int dlv_reset(void)
{
	int ret;

	if(dlv_platform_data->dlv_reset_part)
	{
		ret = dlv_platform_data->dlv_reset_part();
		if(ret)
		{
			ret = dlv_reset_part();
		}
	} else
		ret = dlv_reset_part();

	return ret;
}

/******** dlv_anti_pop ********/
/**
 * CODEC dlv anti pop part
 *
 * it will be used as default, it can be recode 
 * depend on difficent boards if necessary
 *
 **/
static int dlv_anti_pop_part(void)
{
//	printk("JZ_DLV: waring, %s() is a default function\n", __func__);

	if (__dlv_get_sb_hp() != POWER_ON)
	{
		__dlv_switch_sb(POWER_ON);
		mdelay(10);
		
		__dlv_switch_sb_sleep(POWER_ON);
		mdelay(10);
		
		__dlv_switch_sb_dac(POWER_ON);
		udelay(500);
		
		__dlv_disable_hp_mute();
		mdelay(1);
		
		/* turn on sb_hp */
		__dlv_set_irq_flag(1 << IFR_RUP);
		__dlv_switch_sb_hp(POWER_ON);   
		dlv_sleep_wait_bitset(0xb, IFR_RUP, 100,__LINE__);
	
		mdelay(1);
	}

	return 0;
}

static int dlv_anti_pop(int mode)
{
	int ret = 0;

	switch(mode) {
	case CODEC_WRMODE:
		break;
	case CODEC_RMODE:
		break;
	case CODEC_WMODE:
		if(dlv_platform_data->dlv_anti_pop_part)
		{
			ret = dlv_platform_data->dlv_anti_pop_part();
			if(ret)
			{
				ret = dlv_anti_pop_part();
			}
		} else
			ret = dlv_anti_pop_part();
		break;
	}

	return ret;
}

/******** dlv_suspend ************/
/**
 * CODEC dlv suspend part
 *
 * it will do the suspend as default, it can be recode 
 * depend on difficent boards if necessary
 *
 **/
static int dlv_suspend_part(void)
{
	int ret;

	printk("JZ_DLV: waring, %s() is a default function\n", __func__);

	ret = dlv_set_route(ROUTE_ALL_CLEAR);
	if(ret != ROUTE_ALL_CLEAR)
	{
		printk("JZ CODEC: dlv_suspend_part error!\n");
		return -1;
	}

	return 0;
}

static int dlv_suspend(void)
{
	int ret;
	g_dlv_sleep_mode = 0;
	
	/* set 16ohm load to keep away from the bug can not waited RDO */
	if(__dlv_get_load() == LOAD_10KOHM)
		__dlv_set_16ohm_load();

	if(dlv_platform_data->dlv_suspend_part)
	{
		ret = dlv_platform_data->dlv_suspend_part();
		if(ret)
		{
			ret = dlv_suspend_part();
		}
	} else
		ret = dlv_suspend_part();

	return ret;
}

/********* dlv_resume ***********/
/**
 * CODEC dlv resume part
 *
 * it will do the resume as default, it can be recode 
 * depend on difficent boards if necessary
 *
 **/

static int dlv_resume_part(void)
{
	int ret;
	int route = keep_old_route;

	printk("JZ_DLV: waring, %s() is a default function\n", __func__);

	/*default, the resume will restore the route before suspend*/
	ret = dlv_set_route(route);

	if(ret != route)
	{
		printk("JZ CODEC: dlv_resume_part error!\n");
		return -1;
	}

	return 0;
}

static int dlv_resume(void)
{
	int ret;
	if(dlv_platform_data->dlv_resume_part)
	{
		ret = dlv_platform_data->dlv_resume_part();
		if(ret)
		{
			ret = dlv_resume_part();
		}
	} else
		ret = dlv_resume_part();

	g_dlv_sleep_mode = 1;

	return ret;
}

/*---------------------------------------*/

/**
 * CODEC set device
 *
 * this is just a demo function, and it will be use as default 
 * if it is not realized depend on difficent boards 
 *
 */
static int dlv_set_device(struct snd_device_config *snd_dev_cfg)
{
	int ret;
	int iserror = 0;

	printk("JZ_DLV: waring, %s() is a default function\n", __func__);

	switch (snd_dev_cfg->device) {

	case SND_DEVICE_HEADSET:
		ret = dlv_set_route(REPLAY_HP_STEREO);
		if(ret != REPLAY_HP_STEREO)
		{
			printk("JZ CODEC: set device SND_DEVICE_HEADSET error!\n");
			return -1;
		}
		break;

	case SND_DEVICE_HANDSET:
		ret = dlv_set_route(REPLAY_LINEOUT_MONO);
		if(ret != REPLAY_LINEOUT_MONO)
		{
			printk("JZ CODEC: set device SND_DEVICE_HANDSET error!\n");
			return -1;
		}
		break;

	case SND_DEVICE_SPEAKER:
		ret = dlv_set_route(REPLAY_BTL);
		if(ret != REPLAY_BTL)
		{
			printk("JZ CODEC: set device SND_DEVICE_SPEAKER error!\n");
			return -1;
		}
		break;

	case SND_DEVICE_HEADSET_AND_SPEAKER:
		ret = dlv_set_route(REPLAY_BTL);
		if(ret != REPLAY_BTL)
		{
			printk("JZ CODEC: set device SND_DEVICE_HEADSET_AND_SPEAKER error!\n");
			return -1;
		}			
		break;

	default:
		iserror = 1;
		printk("JZ DLV: Unkown ioctl argument in SND_SET_DEVICE\n");
	};

	if (!iserror)
		g_current_out_dev = snd_dev_cfg->device;

	return 0;
}

/*---------------------------------------*/

/**
 * CODEC set standby
 *
 * this is just a demo function, and it will be use as default 
 * if it is not realized depend on difficent boards 
 *
 */

static int dlv_set_standby(unsigned int sw)
{
	printk("JZ_DLV: waring, %s() is a default function\n", __func__);

	switch(g_current_out_dev) {

	case SND_DEVICE_HEADSET:
		if (sw == POWER_ON) {
			/* set the relevant route */
		} else {
			/* clean the relevant route */
		}
		break;

	case SND_DEVICE_HANDSET:
		if (sw == POWER_ON) {
			/* set the relevant route */
		} else {
			/* clean the relevant route */
		}
		break;

	case SND_DEVICE_SPEAKER:
		if (sw == POWER_ON) {
			/* set the relevant route */
		} else {
			/* clean the relevant route */
		}
		break;

	case SND_DEVICE_HEADSET_AND_SPEAKER:
		if (sw == POWER_ON) {
			/* set the relevant route */
		} else {
			/* clean the relevant route */
		}
		break;

	default:
		printk("JZ DLV: Unkown ioctl argument in SND_SET_STANDBY\n");

	}

	return 0;
}

/*---------------------------------------*/
/**
 * CODEC set record rate & data width & volume & channel  
 *
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

	__dlv_select_adc_samp_rate(speed);

	return mrate[speed];
}

static int dlv_set_record_data_width(int width)
{
	int supported_width[4] = {16, 18, 20, 24};
	int fix_width;

	for(fix_width = 0; fix_width < 3; fix_width ++)
	{
		if (width <= supported_width[fix_width])
			break;
	}

	__dlv_select_dac_word_length(fix_width);

	return width;
}

static int dlv_set_record_volume(int val)
{
	/*just set analog gm1 and gm2*/
	int fixed_vol;
	int volume_base;

	if(dlv_platform_data->dlv_record_volume_base)
	{
		volume_base = dlv_platform_data->dlv_record_volume_base;

		fixed_vol = (volume_base >> 2) + 
			     ((5 - (volume_base >> 2)) * val / 100);		
	}
	else 
		fixed_vol = (5 * val / 100);

	__dlv_set_gm1(fixed_vol);
	__dlv_set_gm2(fixed_vol);

	return val;
}

static int dlv_set_record_channel(int channel)
{
	return channel;
}

/*---------------------------------------*/
/**
 * CODEC set replay rate & data width & volume & channel  
 *
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

	__dlv_select_dac_samp_rate(speed);
	
	return mrate[speed];
}

static int dlv_set_replay_data_width(int width)
{
	int supported_width[4] = {16, 18, 20, 24};
	int fix_width;

	for(fix_width = 0; fix_width < 3; fix_width ++)
	{
		if (width <= supported_width[fix_width])
			break;
	}

	__dlv_select_dac_word_length(fix_width);

	return width;
}

static int dlv_set_replay_volume(int val)
{
	/*just set analog gol and gor*/
	unsigned long fixed_vol;
	int volume_base;

	if(dlv_platform_data->dlv_replay_volume_base)
	{
		volume_base = dlv_platform_data->dlv_replay_volume_base;


		fixed_vol = (6 - volume_base) + 
			((25 + volume_base) * (100 - val) / 100);
	}
	else 
		fixed_vol = (6 + (25 * (100 - val) / 100));
	
	if(val == 0)
	{
		fixed_vol = 0x1f;
	}
	if(val < 6)
	{
	  fixed_vol = 30;    
    }

	if (val == 0) {
		if(__dlv_get_hp_mute() == 0){
			__dlv_enable_hp_mute();
		}
	} else {
		if(__dlv_get_hp_mute()){
			__dlv_disable_hp_mute();
		}
	}
	__dlv_set_hp_volume(fixed_vol);

	return val;
}

static int dlv_set_replay_channel(int channel)
{
	channel = (channel >= 2) + 1;

	switch (channel) {
	case 1:
		// MONO->1 for Mono
		dlv_set_replay_filter(REPLAY_FILTER_MONO);
		break;
	case 2:
		// MONO->0 for Stereo
		dlv_set_replay_filter(REPLAY_FILTER_STEREO);
		break;
	}

	return channel;
}

/*---------------------------------------*/
/**
 * CODEC set mute
 *  
 * set dac mute used for anti pop
 *
 */

static int dlv_mute(int val)
{
	if(val){
		if(__dlv_get_dac_mute() == 0){		
			/* enable dac mute */
			__dlv_set_irq_flag(1 << IFR_GDO);
			__dlv_enable_dac_mute();
			dlv_sleep_wait_bitset(0xb, IFR_GDO, 100,__LINE__);
		}
	} else {
		if(__dlv_get_dac_mute()){
			/* disable dac mute */
			__dlv_set_irq_flag(1 << IFR_GUP);
			__dlv_disable_dac_mute();
			dlv_sleep_wait_bitset(0xb, IFR_GUP, 100,__LINE__);
		}
	}
	
	return 0;
}

static int dlv_bsp_mute(int val)
{
	int ret = -1;

	if(dlv_platform_data->dlv_board_mute){
		if(val == 0){
			ret = dlv_platform_data->dlv_board_mute(0);
	//		__dlv_disable_dac_mute();
	//		__dlv_disable_hp_mute();
		}else{
	//		__dlv_enable_hp_mute();
	//		__dlv_enable_dac_mute();
			ret = dlv_platform_data->dlv_board_mute(1);
		}
	}
	
	//if(ret) printk("dlv_bsp_mute: enable=%d fail!\n",val);
	
	return ret;
}
/*---------------------------------------*/

static int dlv_debug_routine(void *arg)
{
	return 0;
}

/***************************************************************************************\
 *                                                                                     *
 *control interface                                                                    *
 *                                                                                     *
\***************************************************************************************/
/**
 * CODEC ioctl (simulated) routine
 *
 * Provide control interface for i2s driver
 */
static int jzdlv_ioctl(void *context, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	DUMP_IOC_CMD();

	DLV_LOCK();
	{
		switch (cmd) {

		case CODEC_INIT:
			ret = dlv_init();
			break;

		case CODEC_TURN_OFF:
			ret = dlv_turn_off(arg);
			break;

		case CODEC_SHUTDOWN:
			ret = dlv_shutdown();
			break;

		case CODEC_RESET:
			ret = dlv_reset();
			break;

		case CODEC_SUSPEND:
			ret = dlv_suspend();
			break;

		case CODEC_RESUME:
			ret = dlv_resume();
			break;

		case CODEC_ANTI_POP:
			ret = dlv_anti_pop((int)arg);
			break;

		case CODEC_SET_ROUTE:
			ret = dlv_set_route((int)arg);
			break;

		case CODEC_SET_DEVICE:
			if (dlv_platform_data->dlv_set_device)
			{
				ret = dlv_platform_data->dlv_set_device((struct snd_device_config *)arg);
				if (ret)
				{
					ret = dlv_set_device((struct snd_device_config *)arg);
				}
			} else
				ret = dlv_set_device((struct snd_device_config *)arg);
			break;

		case CODEC_SET_STANDBY:
			if (dlv_platform_data->dlv_set_standby)
			{
				ret = dlv_platform_data->dlv_set_standby((unsigned int)arg);
				if (ret)
				{
					ret = dlv_set_standby((unsigned int)arg);
				}
			} else
				ret = dlv_set_standby((unsigned int)arg);
			break;

		case CODEC_SET_RECORD_RATE:
			ret = dlv_set_record_rate((int)arg);
			break;

		case CODEC_SET_RECORD_DATA_WIDTH:
			ret = dlv_set_record_data_width((int)arg);
			break;

		case CODEC_SET_MIC_VOLUME:
			ret = dlv_set_record_volume((int)arg);
			break;

		case CODEC_SET_RECORD_CHANNEL:
			ret = dlv_set_record_channel((int)arg);
			break;

		case CODEC_SET_REPLAY_RATE:
			ret = dlv_set_replay_rate((int)arg);
			break;

		case CODEC_SET_REPLAY_DATA_WIDTH:
			ret = dlv_set_replay_data_width((int)arg);
			break;

		case CODEC_SET_REPLAY_VOLUME:
			ret = dlv_set_replay_volume((int)arg);
			break;

		case CODEC_SET_REPLAY_CHANNEL:
			ret = dlv_set_replay_channel((int)arg);
			break;

		case CODEC_DAC_MUTE:
			ret = dlv_mute((int)arg);
			break;

		case CODEC_BSP_MUTE:
			ret = dlv_bsp_mute((int)arg);
			break;

		case CODEC_DEBUG_ROUTINE:
			ret = dlv_debug_routine((void *)arg);
			break;

		default:
			printk("JZ DLV:%s:%d: Unkown IOC commond\n", __FUNCTION__, __LINE__);
			ret = -1;
		}
	}
	DLV_UNLOCK();

	return ret;
}

/***************************************************************************************\
 *                                                                                     *
 *irq handle                                                                           *
 *                                                                                     *
\***************************************************************************************/

/**
 * CODEC short circut handler
 *
 * To protect CODEC, CODEC will be shutdown when short circut occured.
 * Then we have to restart it.
 */
static inline void dlv_short_circut_handler(void)
{
	unsigned int	curr_vol;
	unsigned int	load_flag = 0;
	unsigned int	dlv_ifr, delay;
	int err = 0;
	
	#define VOL_DELAY_BASE 22               //per VOL delay time in ms
	
	curr_vol = dlv_read_reg(DLV_REG_GCR1);  //read the current HP volume	 
	printk("JZ DLV: Short circut detected! restart CODEC. A$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
	delay = VOL_DELAY_BASE * (0x20 - (curr_vol & 0x1f));

	/* set hp gain to min */
	dlv_set_gain_hp_left(0x1f);
	dlv_set_gain_hp_right(0x1f);

	printk("Short circut volume delay %d ms curr_vol=%x \n", delay,(curr_vol & 0x1f));
	dlv_sleep(delay);

	/* set 16ohm load to keep away from the bug can not waited RDO */
	if(__dlv_get_load() == LOAD_10KOHM)
	{
		__dlv_set_16ohm_load();
		load_flag = 1;
	}

#ifndef CONFIG_HP_SENSE_DETECT
	//clear rdo rup
	__dlv_set_irq_flag((1 << IFR_RDO) | (1 << IFR_RUP));

	/* turn off sb_hp */
	__dlv_switch_sb_hp(POWER_OFF);
//	dlv_sleep_wait_bitset(0xb, IFR_RDO, 100,__LINE__);
	err = dlv_sleep_wait_bitset(0xb, IFR_RDO, 100,__LINE__);
	if(__dlv_get_sb_hp() == POWER_OFF){
		printk("POWER_OFF !!!\n");
	}else{
		printk("POWER_ON\n");
	}
	if(err){
_ERR_IFR_RDO:
		printk("ERROR IFR_RDO,,, IN SB MODE! REBOOT!\n");

	}
#endif //nodef CONFIG_HP_SENSE_DETECT

	while (1) {
		dlv_ifr = __dlv_get_irq_flag();
		printk("waiting for SCMC recover finish ----- dlv_ifr = 0x%02x\n", dlv_ifr);
		if ((dlv_ifr & (1 << IFR_SCMC)) == 0) 
			break; 
		__dlv_set_irq_flag((1 << IFR_SCMC));
		dlv_sleep(10);
	}

#ifndef CONFIG_HP_SENSE_DETECT
	/* turn on sb_hp */
	__dlv_switch_sb_hp(POWER_ON);
	dlv_sleep_wait_bitset(0xb, IFR_RUP, 100,__LINE__);
#endif //nodef CONFIG_HP_SENSE_DETECT


	if(load_flag)
		__dlv_set_10kohm_load();

	/* restore hp gain */
//	dlv_set_gain_hp_left(curr_vol);
//	dlv_set_gain_hp_right(curr_vol);
       
        __dlv_set_hp_volume(curr_vol);

	dlv_sleep(delay);

	dlv_ifr = __dlv_get_irq_flag();
	printk("dlv_ifr = 0x%02x\n",dlv_ifr);
	//if(dlv_ifr & (1<< IFR_RDO)){
	if(!(dlv_ifr & (1<< IFR_RUP))){
		printk("output stage is not ramp up !!!\n");
//		goto _ERR_IFR_RDO;
	}
	printk("JZ DLV: Short circut restart CODEC hp out finish.\n");
}

#ifdef CONFIG_HP_SENSE_DETECT

static void dlv_jack_handler(int new_switch_status)
{
	/*just set the state is enough,*/
	if (g_switch_data) {
		switch_set_state(&g_switch_data->sdev, new_switch_status);
      	}
}

/**
 * CODEC work queue handler
 *
 * Handle bottom-half of SCMC & JACKE irq
 *
 */
static void dlv_irq_work_handler(struct work_struct *work)
{
	unsigned int dlv_ifr;
	unsigned long flags;

	int old_status = 0;
	int new_status = 0;
	int i;
	int j;
	DLV_LOCK();
	do {
		dlv_ifr = __dlv_get_irq_flag();

		if (dlv_ifr & (1 << IFR_SCMC)) {
			dlv_short_circut_handler();
		}
		
		dlv_ifr = __dlv_get_irq_flag();

		/* Updata SCMC */
		__dlv_set_irq_flag((1 << IFR_SCMC));

		/* Unmask SCMC */
		__dlv_set_irq_mask(ICR_COMMON_MASK);

		printk("JZ DLV: Short circut detected! dlv_ifr = 0x%02x\n", dlv_ifr);
		
	} while(dlv_ifr & (1 << IFR_SCMC));
	
	handling_scmc = 0;

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
			dlv_sleep(50);
		}
	}

	spin_lock_irqsave(dlv_irq_lock, flags);

	/* Clear current irq flag */
	__dlv_set_irq_flag(dlv_ifr);

	/* Unmask SCMC & JACK (ifdef CONFIG_HP_SENSE_DETECT) */
	__dlv_set_irq_mask(ICR_COMMON_MASK);

	spin_unlock_irqrestore(dlv_irq_lock, flags);

	/* If the jack status has changed, we have to redo the process. */
	if (dlv_ifr & (1 << IFR_JACKE)) {
		dlv_sleep(50);
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
 * Now we are only interested in SCMC and JACKE.
 */
static irqreturn_t dlv_codec_irq(int irq, void *dev_id)
{
	unsigned char dlv_icr;
	unsigned char dlv_ifr;

	unsigned int aic_reg;
	unsigned long flags;

	spin_lock_irqsave(dlv_irq_lock, flags);

	dlv_ifr = __dlv_get_irq_flag();
	dlv_icr = __dlv_get_irq_mask();

	/* Mask all irq temporarily */
	__dlv_set_irq_mask(ICR_ALL_MASK);

	aic_reg = REG_AIC_SR;
	
	REG_AIC_SR = 0x78; // legacy ... need check

	if (!(dlv_ifr & ((1 << IFR_SCMC) | (1 << IFR_JACKE)))) {
		/* CODEC may generate irq with flag = 0xc0.
		 * We have to ingore it in this case as there is no mask for the reserve bit.
		 */
		printk("AIC interrupt ??? AIC_SR = 0x%08x\n", aic_reg);

		/* Unmask SCMC & JACK (ifdef CONFIG_HP_SENSE_DETECT) */

		spin_unlock_irqrestore(dlv_irq_lock, flags);
		return IRQ_HANDLED;

	} else {

		spin_unlock_irqrestore(dlv_irq_lock, flags);

		if (handling_scmc == 0) {
			handling_scmc = 1;
			/* Handle SCMC in work queue. */
			queue_work(dlv_work_queue, &dlv_irq_work);
		}

		return IRQ_HANDLED;
	}
}

#else

/**
 * CODEC work queue handler
 *
 * Handle bottom-half of SCMC & JACKE irq
 *
 */
static void dlv_irq_work_handler(struct work_struct *work)
{
	unsigned int	dlv_ifr;
	DLV_LOCK();
	do {
		dlv_ifr = __dlv_get_irq_flag();

		if (dlv_ifr & (1 << IFR_SCMC)) {
			dlv_short_circut_handler();
		}
		
		dlv_ifr = __dlv_get_irq_flag();

		/* Updata SCMC */
		__dlv_set_irq_flag((1 << IFR_SCMC));

		/* Unmask SCMC */
		__dlv_set_irq_mask(ICR_COMMON_MASK);

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

//		printk("AIC interrupt ??? AIC_SR = 0x%08x\n", aic_reg);

		/* Unmask SCMC & JACK (ifdef CONFIG_HP_SENSE_DETECT) */
		__dlv_set_irq_mask(ICR_COMMON_MASK);

		spin_unlock_irqrestore(&dlv_irq_lock, flags);
		return IRQ_HANDLED;

	} else {
		spin_unlock_irqrestore(&dlv_irq_lock, flags);

		/* Handle SCMC and JACK in work queue. */
		queue_work(dlv_work_queue, &dlv_irq_work);

		return IRQ_HANDLED;
	}
}

#endif // ifdef CONFIG_HP_SENSE_DETECT


#ifdef CONFIG_HP_SENSE_DETECT

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
#endif  // ifdef CONFIG_HP_SENSE_DETECT

/*------------------------------------------*/

static int jz_dlv_probe(struct platform_device *pdev)
{
	dlv_platform_data = pdev->dev.platform_data;

	return 0;
}

static int __devexit jz_dlv_remove(struct platform_device *pdev)
{
	dlv_platform_data = NULL;
	return 0;
}

static struct platform_driver jz_dlv_driver = {
	.probe		= jz_dlv_probe,
	.remove		= __devexit_p(jz_dlv_remove),
	.driver		= {
		.name	= "jz_dlv",
		.owner	= THIS_MODULE,
	},
};

/***************************************************************************************\
 *module init                                                                          *
\***************************************************************************************/
static int __init init_dlv(void)
{
	int retval;

	cpm_start_clock(CGM_AIC);

	spin_lock_init(&dlv_irq_lock);

	INIT_WORK(&dlv_irq_work, dlv_irq_work_handler);

	dlv_work_queue = create_singlethread_workqueue("dlv_irq_wq");

	if (!dlv_work_queue) {
		// this can not happen, if happen, we die!
		BUG();
	}

	printk("==================================================\n");
	register_jz_codecs((void *)jzdlv_ioctl);

	dlv_reset_part();                     //codec power on
	
	dlv_set_replay_volume(INIT_VOLUME);   //init replay volume

	retval = request_irq(IRQ_AIC, dlv_codec_irq, IRQF_DISABLED, "dlv_codec_irq", NULL);
	if (retval) {
		printk("JZ DLV: Could not get AIC CODEC irq %d\n", IRQ_AIC);
		return retval;
	}
//	REG_AIC_CR |= AIC_CR_ETUR;
	
#ifdef CONFIG_HP_SENSE_DETECT
	retval = platform_driver_register(&jz_hp_switch_driver);
	if (retval) {
		printk("JZ HP Switch: Could net register headphone sense switch\n");
		return retval;
	}
#endif

	retval = platform_driver_register(&jz_dlv_driver);
	if (retval) {
		printk("JZ CODEC: Could net register jz_dlv_driver\n");
		return retval;
	}

	return 0;
}

/**
 * Module exit
 */
static void __exit cleanup_dlv(void)
{

	free_irq(IRQ_AIC, NULL);
#ifdef CONFIG_HP_SENSE_DETECT
	platform_driver_unregister(&jz_hp_switch_driver);
#endif
	platform_driver_unregister(&jz_dlv_driver);
}

module_init(init_dlv);
module_exit(cleanup_dlv);
