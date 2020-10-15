/*
 * Linux/sound/oss/xb47XX/xb4770/jz4770_dlv.c
 *
 * DLV CODEC driver for Ingenic Jz4770 MIPS processor
 *
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
#include <mach/chip-aic.h>

#include "../jz47XX_codec.h"

/*###############################################*/

#define DLV_DUMP_IOC_CMD		0
#define DLV_DUMP_ROUTE_REGS		0
#define DLV_DUMP_ROUTE_PART_REGS	0
#define DLV_DUMP_GAIN_PART_REGS		0
#define DLV_DUMP_ROUTE_NAME		1

/*##############################################*/

/***************************************************************************************\
 *                                                                                     *
 *global variable and structure interface                                              *
 *                                                                                     *
\***************************************************************************************/

static unsigned int cur_route = -1;
unsigned int keep_old_route = -1;

static struct workqueue_struct *dlv_work_queue;
static struct work_struct dlv_irq_work;
static spinlock_t dlv_irq_lock;
static int handling_scmc = 0;

unsigned int g_current_out_dev;

#ifdef CONFIG_HP_SENSE_DETECT
static jz_hp_switch_data_t *g_switch_data = NULL;
#endif 

/*---------------------*/
static jz_dlv_platform_data_t dlv_platform_data_init_val = {
	.dlv_sys_clk = SYS_CLK_12M,
	.dlv_dmic_clk = DMIC_CLK_OFF,
	.dlv_replay_volume_base = 0,
	.dlv_record_volume_base = 0,
	.dlv_record_digital_volume_base = 0,
	.dlv_replay_digital_volume_base = 0,
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
};

jz_dlv_platform_data_t *dlv_platform_data = &dlv_platform_data_init_val;

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
			sema_init(g_dlv_sem,0);				\
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
static int g_dlv_sleep_mode = 1;
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

static inline void dlv_sleep_wait_bitset(int reg, unsigned bit, int stime, int line)
{
	int count = 0;
	while(!(dlv_read_reg(reg) & (1 << bit))) {
		//printk("DLV waiting reg(%2x) bit(%2x) set %d \n",reg, bit, line);
		dlv_sleep(stime);
		count++;
		if(count > 10){
			printk("%s %d timeout\n",__FILE__,__LINE__);
			break;
		}
	}
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
		RECORD_MIC2_MONO_DIFF_WITH_BIAS,
		REPLAY_HP_STEREO_CAP_LESS,
		REPLAY_HP_STEREO_WITH_CAP,
		REPLAY_HP_STEREO_CAP_LESS_AND_LINEOUT,
		REPLAY_HP_STEREO_WITH_CAP_AND_LINEOUT,
		REPLAY_LINEOUT,
		BYPASS_MIC1_DIFF_WITH_BIAS_TO_HP_CAP_LESS,
		BYPASS_MIC1_DIFF_WITH_BIAS_TO_LINEOUT,
		BYPASS_MIC2_DIFF_WITH_BIAS_TO_HP_CAP_LESS,
		BYPASS_MIC2_DIFF_WITH_BIAS_TO_LINEOUT,
		BYPASS_LINEIN_TO_HP_CAP_LESS,
		BYPASS_LINEIN_TO_LINEOUT,
		RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_HP_CAP_LESS,
		RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_LINEOUT,
	};

	char *route_str[] = {
		"ROUTE_ALL_CLEAR",
		"ROUTE_REPLAY_CLEAR",
		"ROUTE_RECORD_CLEAR",
		"RECORD_MIC1_MONO_DIFF_WITH_BIAS",
		"RECORD_MIC2_MONO_DIFF_WITH_BIAS",
		"REPLAY_HP_STEREO_CAP_LESS",
		"REPLAY_HP_STEREO_WITH_CAP",
		"REPLAY_HP_STEREO_CAP_LESS_AND_LINEOUT",
		"REPLAY_HP_STEREO_WITH_CAP_AND_LINEOUT",
		"REPLAY_LINEOUT",
		"BYPASS_MIC1_DIFF_WITH_BIAS_TO_HP_CAP_LESS",
		"BYPASS_MIC1_DIFF_WITH_BIAS_TO_LINEOUT",
		"BYPASS_MIC2_DIFF_WITH_BIAS_TO_HP_CAP_LESS",
		"BYPASS_MIC2_DIFF_WITH_BIAS_TO_LINEOUT",
		"BYPASS_LINEIN_TO_HP_CAP_LESS",
		"BYPASS_LINEIN_TO_LINEOUT",
		"RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_HP_CAP_LESS",
		"RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_LINEOUT",
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

	/*wait a typical time 250ms to get into sleep mode*/
	if(__dlv_get_sb() == POWER_OFF)
	{
		__dlv_switch_sb(POWER_ON);
		msleep(250);
	}
	/*wait a typical time 200ms for adc (450ms for dac) to get into normal mode*/
	if(__dlv_get_sb_sleep() == POWER_OFF)
	{
		__dlv_switch_sb_sleep(POWER_ON);
		if(mode == ROUTE_READY_FOR_ADC)
			msleep(200);
		else
			msleep(450);
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

static void dlv_set_linein_to_adc(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){
		
	case LINEIN_TO_ADC_ENABLE:
		if(__dlv_get_sb_linein_to_adc() == POWER_OFF)
		{
			__dlv_switch_sb_linein_to_adc(POWER_ON);
			schedule_timeout(2);
		}
		break;

	case LINEIN_TO_ADC_DISABLE:
		if(__dlv_get_sb_linein_to_adc() == POWER_ON)
		{
			__dlv_switch_sb_linein_to_adc(POWER_OFF);
			schedule_timeout(2);
		}
		break;
		
	default:
		printk("JZ_DLV: line: %d, linein mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_linein_to_bypass(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){
		
	case LINEIN_TO_BYPASS_ENABLE:
		if(__dlv_get_sb_linein_to_bypass() == POWER_OFF)
		{
			__dlv_switch_sb_linein_to_bypass(POWER_ON);
			schedule_timeout(2);
		}
		break;

	case LINEIN_TO_BYPASS_DISABLE:
		if(__dlv_get_sb_linein_to_bypass() == POWER_ON)
		{
			__dlv_switch_sb_linein_to_bypass(POWER_OFF);
			schedule_timeout(2);
		}
		break;
		
	default:
		printk("JZ_DLV: line: %d, linein mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

void dlv_set_agc(int mode)//----------------------------------------------------------------------=================?
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
		/* if digital micphone is not select, */
		/* 4770 codec auto set ADC_LEFT_ONLY to 1 */
		__dlv_set_mic_mono();
		__dlv_set_adc_insel(MIC1_TO_LR);
		__dlv_set_dmic_insel(DMIC_SEL_ADC);
		break;

	case RECORD_MUX_MIC2_TO_LR:
		/* if digital micphone is not select, */
		/* 4770 codec auto set ADC_LEFT_ONLY to 1 */
		__dlv_set_mic_mono();
		__dlv_set_adc_insel(MIC2_TO_LR);
		__dlv_set_dmic_insel(DMIC_SEL_ADC);
		break;

	case RECORD_MUX_MIC1_TO_R_MIC2_TO_L:
		__dlv_set_mic_stereo();
		__dlv_set_adc_insel(MIC1_TO_R_MIC2_TO_L);
		__dlv_set_dmic_insel(DMIC_SEL_ADC);
		break;

	case RECORD_MUX_MIC2_TO_R_MIC1_TO_L:
		__dlv_set_mic_stereo();
		__dlv_set_adc_insel(MIC2_TO_R_MIC1_TO_L);
		__dlv_set_dmic_insel(DMIC_SEL_ADC);
		break;

	case RECORD_MUX_LINE_IN:
		__dlv_set_adc_insel(BYPASS_PATH);
		__dlv_set_dmic_insel(DMIC_SEL_ADC);
		break;

	case RECORD_MUX_DIGITAL_MIC:
		__dlv_set_dmic_insel(DMIC_SEL_DIGITAL_MIC);
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
		__dlv_set_adc_stereo();
		__dlv_disable_adc_left_only();
		break;

	case ADC_STEREO_WITH_LEFT_ONLY:
		if(__dlv_get_sb_adc() == POWER_OFF)
		{
			__dlv_switch_sb_adc(POWER_ON);
			schedule_timeout(2);
		}
		__dlv_set_adc_stereo();
		__dlv_enable_adc_left_only();
		break;

	case ADC_MONO:
		/*When ADC_MONO=1, the left and right channels are mixed in digital
		  part: the result is emitted on both left and right channel of ADC digital
		  output. It corresponds to the average of left and right channels when
		  ADC_MONO=0.*/
		if(__dlv_get_sb_adc() == POWER_OFF)
		{
			__dlv_switch_sb_adc(POWER_ON);
			schedule_timeout(2);
		}
		__dlv_set_adc_mono();
		__dlv_disable_adc_left_only();
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
		__dlv_set_rec_mix_mode(MIX1_RECORD_INPUT_ONLY);
		break;

	case RECORD_MIXER_MIX1_INPUT_AND_DAC:
		__dlv_set_rec_mix_mode(MIX1_RECORD_INPUT_AND_DAC);
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
		__dlv_set_dac_mix_mode(MIX2_PLAYBACK_DAC_ONLY);
		break;

	case REPLAY_MIXER_PLAYBACK_DAC_AND_ADC:
		__dlv_set_dac_mix_mode(MIX2_PLAYBACK_DAC_AND_ADC);
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
			dlv_sleep_wait_bitset(DLV_REG_IFR, IFR_GUP, 100,__LINE__);
		}
		__dlv_set_dac_stereo();
		__dlv_disable_dac_left_only();
		break;

	case DAC_STEREO_WITH_LEFT_ONLY:
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
			dlv_sleep_wait_bitset(DLV_REG_IFR, IFR_GUP, 100,__LINE__);
		}
		__dlv_set_dac_stereo();
		__dlv_enable_dac_left_only();
		break;

	case DAC_MONO:
		/*When DAC_MONO=1, the left and right channels are mixed in digital
		  part: the result is emitted on both left and right channel of DAC output. It
		  corresponds to the average of left and right channels when
		  DAC_MONO=0.*/
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
			dlv_sleep_wait_bitset(DLV_REG_IFR, IFR_GUP, 100,__LINE__);
		}
		__dlv_set_dac_mono();
		__dlv_disable_dac_left_only();
		break;

	case DAC_DISABLE:
		if(__dlv_get_sb_dac() == POWER_ON){
			if (!(__dlv_get_dac_mute())){
				/* clear IFR_GDO */
				__dlv_set_irq_flag(1 << IFR_GDO);
				/* turn off dac */
				__dlv_enable_dac_mute();
				/* wait IFR_GDO set */
				dlv_sleep_wait_bitset(DLV_REG_IFR, IFR_GDO, 100,__LINE__);
			}
			__dlv_switch_sb_dac(POWER_OFF);
		}
		break;

	default:
		printk("JZ_DLV: line: %d, dac mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_hp_mux(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case HP_MUX_MIC1_TO_LR:
		__dlv_set_mic_mono();
		__dlv_set_hp_sel(MIC1_TO_LR);
		break;

	case HP_MUX_MIC2_TO_LR:
		__dlv_set_mic_mono();
		__dlv_set_hp_sel(MIC2_TO_LR);
		break;

	case HP_MUX_MIC1_TO_R_MIC2_TO_L:
		__dlv_set_mic_stereo();
		__dlv_set_hp_sel(MIC1_TO_R_MIC2_TO_L);
		break;

	case HP_MUX_MIC2_TO_R_MIC1_TO_L:
		__dlv_set_mic_stereo();
		__dlv_set_hp_sel(MIC2_TO_R_MIC1_TO_L);
		break;

	case HP_MUX_BYPASS_PATH:
		__dlv_set_hp_sel(BYPASS_PATH);
		break;

	case HP_MUX_DAC_OUTPUT:
		__dlv_set_hp_sel(DAC_OUTPUT);
		break;

	default:
		printk("JZ_DLV: line: %d, replay mux mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_hp(int mode)
{	
	int linein_to_bypass_power_on = 0;
	int dac_mute_not_enable = 0;
	int load_flag = 0;

	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case HP_ENABLE_WITH_CAP:
	case HP_ENABLE_CAP_LESS:
		__dlv_set_16ohm_load();
		__dlv_disable_hp_mute();
#if 0
		if (__dlv_get_sb_hpcm() == POWER_OFF)
			__dlv_switch_sb_hpcm(POWER_ON);
#else
		if (mode == HP_ENABLE_CAP_LESS) {
			if (__dlv_get_sb_hpcm() == POWER_OFF)
				__dlv_switch_sb_hpcm(POWER_ON);
		} else {
			if (__dlv_get_sb_hpcm() == POWER_ON)
				__dlv_switch_sb_hpcm(POWER_OFF);
		}
#endif
		mdelay(1);
		if(__dlv_get_sb_hp() == POWER_OFF)
		{
			if(__dlv_get_sb_linein_to_bypass() == POWER_ON){
				__dlv_switch_sb_linein_to_bypass(POWER_OFF);
				linein_to_bypass_power_on = 1;
			}

			if((__dlv_get_dac_mute() == 0) && (__dlv_get_sb_dac() == POWER_ON)){
				/* enable dac mute */
				__dlv_set_irq_flag(1 << IFR_GDO);
				__dlv_enable_dac_mute();
				dlv_sleep_wait_bitset(DLV_REG_IFR, IFR_GDO, 100,__LINE__);

				dac_mute_not_enable = 1;
			}
			
			/* turn on sb_hp */
			__dlv_set_irq_flag(1 << IFR_RUP);
			__dlv_switch_sb_hp(POWER_ON);
			dlv_sleep_wait_bitset(DLV_REG_IFR, IFR_RUP, 100,__LINE__);

			if(linein_to_bypass_power_on){
				__dlv_switch_sb_linein_to_bypass(POWER_ON);
			}

			if(dac_mute_not_enable){
				/*disable dac mute*/
				__dlv_set_irq_flag(1 << IFR_GUP);
				__dlv_disable_dac_mute();
				dlv_sleep_wait_bitset(DLV_REG_IFR, IFR_GUP, 100,__LINE__);
			}
		}
		break;

	case HP_DISABLE:
		if(__dlv_get_sb_hp() == POWER_ON)
		{
			if(__dlv_get_sb_linein_to_bypass() == POWER_ON){
				__dlv_switch_sb_linein_to_bypass(POWER_OFF);
				linein_to_bypass_power_on = 1;
			}

			if((__dlv_get_dac_mute() == 0) && (__dlv_get_sb_dac() == POWER_ON)){
				/* enable dac mute */
				__dlv_set_irq_flag(1 << IFR_GDO);
				__dlv_enable_dac_mute();
				dlv_sleep_wait_bitset(DLV_REG_IFR, IFR_GDO, 100,__LINE__);

				dac_mute_not_enable = 1;
			}

			/* set 16ohm load to keep away from the bug can not waited RDO */
			if(__dlv_get_load() == LOAD_10KOHM)
			{
				__dlv_set_16ohm_load();
				load_flag = 1;
			}
			
			/* turn off sb_hp */
			__dlv_set_irq_flag(1 << IFR_RDO);
			__dlv_switch_sb_hp(POWER_OFF);   
			dlv_sleep_wait_bitset(DLV_REG_IFR, IFR_RDO, 100,__LINE__);

			if(load_flag)
				__dlv_set_10kohm_load();

			if(linein_to_bypass_power_on){
				__dlv_switch_sb_linein_to_bypass(POWER_ON);
			}

			if(dac_mute_not_enable){
				/*disable dac mute*/
				__dlv_set_irq_flag(1 << IFR_GUP);
				__dlv_disable_dac_mute();
				dlv_sleep_wait_bitset(DLV_REG_IFR, IFR_GUP, 100,__LINE__);
			}
			__dlv_enable_hp_mute();
		}
		break;

	default:
		printk("JZ_DLV: line: %d, hp mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_lineout_mux(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case LO_MUX_MIC1_EN:
		__dlv_set_mic_mono();
		__dlv_set_lineout_sel(LO_SEL_MIC1);
		break;

	case LO_MUX_MIC2_EN:
		__dlv_set_mic_mono();
		__dlv_set_lineout_sel(LO_SEL_MIC2);
		break;

	case LO_MUX_MIC1_AND_MIC2_EN:
		__dlv_set_mic_stereo();
		__dlv_set_lineout_sel(LO_SEL_MIC1_AND_MIC2);
		break;

	case LO_MUX_BYPASS_PATH:
		__dlv_set_lineout_sel(BYPASS_PATH);
		break;

	case LO_MUX_DAC_OUTPUT:
		__dlv_set_lineout_sel(DAC_OUTPUT);
		break;

	default:
		printk("JZ_DLV: line: %d, replay mux mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_lineout(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case LINEOUT_ENABLE:
		//__dlv_set_10kohm_load();
		if(__dlv_get_sb_line_out() == POWER_OFF)
		{
			__dlv_switch_sb_line_out(POWER_ON);
			schedule_timeout(2);
		}
		__dlv_disable_lineout_mute();
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

/*=================route attibute(gain) functions======================*/

//--------------------- mic1
static int dlv_get_gain_mic1(void)
{
	int val,gain;
	DUMP_GAIN_PART_REGS("enter");

        val =  __dlv_get_gm1();
	gain = val * 4;

	DUMP_GAIN_PART_REGS("leave");
	return gain;
}

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

	if (dlv_get_gain_mic1() != gain)
		printk("JZ_DLV: dlv_set_gain_mic1 error!\n");

	DUMP_GAIN_PART_REGS("leave");
}

//--------------------- mic2
static int dlv_get_gain_mic2(void)
{
	int val,gain;
	DUMP_GAIN_PART_REGS("enter");
	
	val =  __dlv_get_gm2();
	gain = val * 4;

	DUMP_GAIN_PART_REGS("leave");
	return gain;
}

static void dlv_set_gain_mic2(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");

	if (gain < 0)
		gain = 0;
	else if (gain > 20)
		gain = 20;

	val = gain / 4;

	__dlv_set_gm2(val);

	if (dlv_get_gain_mic2() != gain)
		printk("JZ_DLV: dlv_set_gain_mic2 error!\n");

	DUMP_GAIN_PART_REGS("leave");
}


//--------------------- line in left

static int dlv_get_gain_linein_left(void)
{
	int val,gain;
	DUMP_GAIN_PART_REGS("enter");

	val = __dlv_get_gil();
	
	gain = (6 - val);

	DUMP_GAIN_PART_REGS("leave");
	return gain;
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

	if (dlv_get_gain_linein_left() != gain)
		printk("JZ_DLV: dlv_set_gain_linein_left error!\n");

	DUMP_GAIN_PART_REGS("leave");
}

//--------------------- line in right
static int dlv_get_gain_linein_right(void)
{
	int val,gain;
	DUMP_GAIN_PART_REGS("enter");

	val = __dlv_get_gir();

	gain = (6 - val);

	DUMP_GAIN_PART_REGS("leave");
	return gain;
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

	if (dlv_get_gain_linein_right() != gain)
		printk("JZ_DLV: dlv_set_gain_linein_right error!\n");

	DUMP_GAIN_PART_REGS("leave");
}

//--------------------- adc left
static int dlv_get_gain_adc_left(void)
{
	int val,gain;
	DUMP_GAIN_PART_REGS("enter");

	val = __dlv_get_gidl();
	
	gain = val;

	DUMP_GAIN_PART_REGS("leave");

	return gain;
}

static void dlv_set_gain_adc_left(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");

	if (gain < 0)
		gain = 0;
	else if ( gain > 43)
		gain = 43;
	
	val = gain;

	__dlv_set_gidl(val);

	if (dlv_get_gain_adc_left() != gain)
		printk("JZ_DLV: dlv_set_gain_adc_left error!\n");

	DUMP_GAIN_PART_REGS("leave");
}

//--------------------- adc right
static int dlv_get_gain_adc_right(void)
{
	int val,gain;
	DUMP_GAIN_PART_REGS("enter");

	val = __dlv_get_gidr();
	
	gain = val;

	DUMP_GAIN_PART_REGS("leave");
	
	return gain;
}

static void dlv_set_gain_adc_right(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");

	if (gain < 0)
		gain = 0;
	else if ( gain > 43)
		gain = 43;
	
	val = gain;

	__dlv_set_gidr(val);

	if (dlv_get_gain_adc_right() != gain)
		printk("JZ_DLV: dlv_set_gain_adc_right error!\n");

	DUMP_GAIN_PART_REGS("leave");
}

//--------------------- record mixer
int dlv_get_gain_record_mixer (void)
{
	int val,gain;
	DUMP_GAIN_PART_REGS("enter");

	val = __dlv_get_gimix();

	gain = -val;

	DUMP_GAIN_PART_REGS("leave");
	return gain;
}

void dlv_set_gain_record_mixer(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");
	
	if (gain > 0)
		gain = 0;
	else if (gain < -31)
		gain = -31;

	val = -gain;

	__dlv_set_gimix(val);

	if (dlv_get_gain_record_mixer() != gain)
		printk("JZ_DLV: dlv_set_gain_record_mixer error!\n");

	DUMP_GAIN_PART_REGS("leave");
}

//--------------------- replay mixer
static int dlv_get_gain_replay_mixer(void)
{
	int val,gain;
	DUMP_GAIN_PART_REGS("enter");

	val = __dlv_get_gomix();
	
	gain = -val;

	DUMP_GAIN_PART_REGS("leave");
	return gain;
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

	if (dlv_get_gain_replay_mixer() != gain)
		printk("JZ_DLV: dlv_set_gain_replay_mixer error!\n");

	DUMP_GAIN_PART_REGS("leave");
}

//--------------------- dac left
static int dlv_get_gain_dac_left(void)
{
	int val,gain;
	DUMP_GAIN_PART_REGS("enter");

	val = __dlv_get_godl();

	gain = -val;

	DUMP_GAIN_PART_REGS("leave");

	return gain;
}

void dlv_set_gain_dac_left(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");
	if (gain > 0)
		gain = 0;
	else if (gain < -31)
		gain = -31;

       	val = -gain;
       
	__dlv_set_godl(val);

	if (dlv_get_gain_dac_left() != gain)
		printk("JZ_DLV: dlv_set_gain_dac_left error!\n");

	DUMP_GAIN_PART_REGS("leave");
}

//--------------------- dac right
int dlv_get_gain_dac_right(void)
{
	int val,gain;
	DUMP_GAIN_PART_REGS("enter");

	val = __dlv_get_godr();

	gain = -val;

	DUMP_GAIN_PART_REGS("leave");

	return gain;
}

void dlv_set_gain_dac_right(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");

	if (gain > 0)
		gain = 0;
	else if (gain < -31)
		gain = -31;

	val = -gain;

	__dlv_set_godr(val);

	if (dlv_get_gain_dac_right() != gain)
		printk("JZ_DLV: dlv_set_gain_dac_right error!\n");

	DUMP_GAIN_PART_REGS("leave");
}

//--------------------- hp left
static int dlv_get_gain_hp_left(void)
{
	int val,gain;
	DUMP_GAIN_PART_REGS("enter");

	val = __dlv_get_gol();
	
	gain = (6 - val);

	DUMP_GAIN_PART_REGS("leave");
	return gain;
}

void dlv_set_gain_hp_left(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");

	if (gain > 6)
		gain = 6;
	else if (gain < -25)
		gain = -25;

	val = (6 - gain);

	__dlv_set_gol(val);

	if (dlv_get_gain_hp_left() != gain)
		printk("JZ_DLV: dlv_set_gain_hp_left error!\n");

	DUMP_GAIN_PART_REGS("leave");
}

//--------------------- hp right
static int dlv_get_gain_hp_right(void)
{
	int val,gain;
	DUMP_GAIN_PART_REGS("enter");

	val = __dlv_get_gor();

	gain = (6 - val);

	DUMP_GAIN_PART_REGS("leave");
	return gain;
}

void dlv_set_gain_hp_right(int gain)
{
	int val;

	DUMP_GAIN_PART_REGS("enter");

	if (gain > 6)
		gain = 6;
	else if (gain < -25)
		gain = -25;

	val = (6 - gain);

	__dlv_set_gor(val);

	if (dlv_get_gain_hp_right() != gain)
		printk("JZ_DLV: dlv_set_gain_hp_right error!\n");

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

	if (conf->route_linein_to_adc_mode)
		dlv_set_linein_to_adc(conf->route_linein_to_adc_mode);

	if (conf->route_linein_to_bypass_mode)
		dlv_set_linein_to_bypass(conf->route_linein_to_bypass_mode);

	if (conf->route_record_mux_mode)
		dlv_set_record_mux(conf->route_record_mux_mode);

	if (conf->route_adc_mode)
		dlv_set_adc(conf->route_adc_mode);

	if (conf->route_record_mixer_mode)
		dlv_set_record_mixer(conf->route_record_mixer_mode);

	/* replay path */
	if (conf->route_replay_mixer_mode)
		dlv_set_replay_mixer(conf->route_replay_mixer_mode);

	if (conf->route_dac_mode)
		dlv_set_dac(conf->route_dac_mode);

	if (conf->route_hp_mux_mode)
		dlv_set_hp_mux(conf->route_hp_mux_mode);

	if (conf->route_hp_mode)
		dlv_set_hp(conf->route_hp_mode);

	if (conf->route_lineout_mux_mode)
		dlv_set_lineout_mux(conf->route_lineout_mux_mode);

	if (conf->route_lineout_mode)
		dlv_set_lineout(conf->route_lineout_mode);

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
	/* temp 2012-2-16 by sbhuang */
	if(dlv_platform_data->dlv_replay_hp_output_gain){
		dlv_set_gain_hp_right(dlv_platform_data->dlv_replay_hp_output_gain);	
		dlv_set_gain_hp_left(dlv_platform_data->dlv_replay_hp_output_gain);	
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

		/* switch table */

	default:
		printk("%s: dlv set route gpio error!, undecleard route\n", __func__);
	}
	
	return 0;
}

static int dlv_set_gpio_after_set_route(int route)
{
	printk("JZ_DLV: waring, %s() is a default function\n", __func__);

	switch(route){

		/* switch table */

	default:
		printk("%s: dlv set route gpio error!, undecleard route\n", __func__);
	}

	return 0;
}

/*-----------------main fun-------------------*/

int dlv_set_route(int route)
{
	int i = 0;
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

	return 0;
}

static int dlv_init(void)
{
	int ret;

	DLV_LOCKINIT();

	/* the generated IRQ is high level whit 8 SYS_CLK */
	__dlv_set_int_form(ICR_INT_HIGH_8CYCLES);

	/* set IRQ mask and clear IRQ flags*/
	__dlv_set_irq_mask(ICR_COMMON_MASK);
	__dlv_set_irq_flag(REG_IFR_MASK);

	/* set SYS_CLK to 12MHZ */
	__dlv_set_crystal(dlv_platform_data->dlv_sys_clk);

	/* enable DMIC_CLK */
	__dlv_set_dmic_clock(dlv_platform_data->dlv_dmic_clk);

	/* disable ADC/DAC LRSWP */
	__dlv_set_adc_lrswap(LRSWAP_DISABLE);
	__dlv_set_dac_lrswap(LRSWAP_DISABLE);
	
	/*  set record digtal volume base */
	if(dlv_platform_data->dlv_record_digital_volume_base){
		dlv_set_gain_adc_left(dlv_platform_data->dlv_record_digital_volume_base);
		dlv_set_gain_adc_right(dlv_platform_data->dlv_record_digital_volume_base);
	}
	
	/* set replay digital volume base */
	if(dlv_platform_data->dlv_replay_digital_volume_base){
		dlv_set_gain_dac_left(dlv_platform_data->dlv_replay_digital_volume_base);
		dlv_set_gain_dac_right(dlv_platform_data->dlv_replay_digital_volume_base);
	}

	if(dlv_platform_data->dlv_replay_hp_output_gain){
		dlv_set_gain_hp_right(dlv_platform_data->dlv_replay_hp_output_gain);	
		dlv_set_gain_hp_left(dlv_platform_data->dlv_replay_hp_output_gain);	
	}
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
#if 1
		/* shutdown sequence */
		__dlv_enable_dac_mute();
		dlv_sleep_wait_bitset(DLV_REG_IFR, IFR_GDO, 100,__LINE__);
	        __dlv_set_irq_flag(1 << IFR_GDO);
  	        __dlv_switch_sb_hp(POWER_OFF);
		dlv_sleep_wait_bitset(DLV_REG_IFR, IFR_RDO, 100,__LINE__);
		__dlv_set_irq_flag(1 << IFR_RDO);		
		__dlv_enable_hp_mute();
		udelay(500);
  		__dlv_switch_sb_dac(POWER_OFF);
		udelay(500);
		__dlv_switch_sb_sleep(POWER_OFF);
		dlv_sleep(10);
      		__dlv_switch_sb(POWER_OFF);
		dlv_sleep(10);
#else
		ret = dlv_set_route(ROUTE_ALL_CLEAR);
		if(ret != ROUTE_ALL_CLEAR)
		{
			printk("JZ CODEC: dlv_turn_off_part replay & record mode error!\n");
			return -1;
		}
		__dlv_switch_sb_sleep(POWER_OFF);
		__dlv_switch_sb(POWER_OFF);
#endif  
	} else if (mode & REPLAY) {
		printk("JZ DLV: Close REPLAY\n");
/*
		ret = dlv_set_route(ROUTE_REPLAY_CLEAR);
		if(ret != ROUTE_REPLAY_CLEAR)
		{
			printk("JZ CODEC: dlv_turn_off_part replay mode error!\n");
			return -1;
		}
*/
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

	DLV_LOCKDEINIT();  

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

	return 0;
}

static int dlv_reset(void)
{
	int ret;

	/* select serial interface and work mode of adc and dac */
	__dlv_select_adc_digital_interface(SERIAL_INTERFACE);
	__dlv_select_dac_digital_interface(SERIAL_INTERFACE);

	__dlv_select_adc_work_mode(I2S_MODE);
	__dlv_select_dac_work_mode(I2S_MODE);

	/* reset codec ready for set route */
	dlv_set_route_ready(ROUTE_READY_FOR_DAC);

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
	printk("JZ_DLV: waring, %s() is a default function\n", __func__);

	if (__dlv_get_sb_hp() != POWER_ON)
	{
		__dlv_switch_sb(POWER_ON);
		dlv_sleep(10);
		
		__dlv_switch_sb_sleep(POWER_ON);
		dlv_sleep(10);
		
		__dlv_switch_sb_dac(POWER_ON);
		udelay(500);
		
		__dlv_enable_hp_mute();
		mdelay(1);
		
		/* turn on sb_hp */
		__dlv_clear_rup();
        	__dlv_switch_sb_hp(POWER_ON);   
        	dlv_sleep_wait_bitset(DLV_REG_IFR, IFR_RUP, 100,__LINE__);
	
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
		ret = dlv_set_route(REPLAY_HP_STEREO_CAP_LESS);
		if(ret != REPLAY_HP_STEREO_CAP_LESS)
		{
			return -1;
		}
		break;

	case SND_DEVICE_HANDSET:
	case SND_DEVICE_SPEAKER:
	case SND_DEVICE_HEADSET_AND_SPEAKER:
		ret = dlv_set_route(REPLAY_LINEOUT);
		if(ret != REPLAY_LINEOUT)
		{
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
		if (sw == STANDBY) {
			/* set the relevant route */
		} else {
			/* clean the relevant route */
		}
		break;

	case SND_DEVICE_HANDSET:
		if (sw == STANDBY) {
			/* set the relevant route */
		} else {
			/* clean the relevant route */
		}
		break;

	case SND_DEVICE_SPEAKER:
		if (sw == STANDBY) {
			/* set the relevant route */
		} else {
			/* clean the relevant route */
		}
		break;

	case SND_DEVICE_HEADSET_AND_SPEAKER:
		if (sw == STANDBY) {
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

	__dlv_set_gol(fixed_vol);
	__dlv_set_gor(fixed_vol);

	return val;
}

static int dlv_set_replay_channel(int channel)
{
	return channel;
}

/*---------------------------------------*/
/**
 * CODEC set/clear adc/dac lrswap
 *  
 */
static int dlv_set_adc_lrswap(void)
{
	__dlv_set_adc_lrswap(LRSWAP_ENABLE);

	return 0;
}

static int dlv_clear_adc_lrswap(void)
{
	__dlv_set_adc_lrswap(LRSWAP_DISABLE);

	return 0;
}

static int dlv_set_dac_lrswap(void)
{
	__dlv_set_dac_lrswap(LRSWAP_ENABLE);

	return 0;
}

static int dlv_clear_dac_lrswap(void)
{
	__dlv_set_dac_lrswap(LRSWAP_DISABLE);

	return 0;
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
			dlv_sleep_wait_bitset(DLV_REG_IFR, IFR_GDO, 100,__LINE__);
		}
	} else {
		if(__dlv_get_dac_mute()){
			/* disable dac mute */
			__dlv_set_irq_flag(1 << IFR_GUP);
			__dlv_disable_dac_mute();
			dlv_sleep_wait_bitset(DLV_REG_IFR, IFR_GUP, 100,__LINE__);
		}
	}
	
	return 0;
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
			break;
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

		case CODEC_SET_ADC_LRSWAP:
			ret = dlv_set_adc_lrswap();
			break;

		case CODEC_CLEAR_ADC_LRSWAP:
			ret = dlv_clear_adc_lrswap();
			break;

		case CODEC_SET_DAC_LRSWAP:
			ret = dlv_set_dac_lrswap();
			break;

		case CODEC_CLEAR_DAC_LRSWAP:
			ret = dlv_clear_dac_lrswap();
			break;

		case CODEC_DAC_MUTE:
			ret = dlv_mute((int)arg);
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
	int	curr_hp_left_vol;
	int	curr_hp_right_vol;
	unsigned int	load_flag = 0;
	unsigned int	dlv_ifr, delay;
	
	#define VOL_DELAY_BASE 22               //per VOL delay time in ms

	printk("JZ DLV: Short circut detected! restart CODEC.\n");

	curr_hp_left_vol = dlv_get_gain_hp_left();
	curr_hp_right_vol = dlv_get_gain_hp_right();

	/* delay */
	delay = VOL_DELAY_BASE * (25 + (curr_hp_left_vol + curr_hp_right_vol)/2);

	/* set hp gain to min */
	dlv_set_gain_hp_left(-25);
	dlv_set_gain_hp_right(-25);

	printk("Short circut volume delay %d ms curr_hp_left_vol=%x curr_hp_right_vol=%x \n",
	       delay, curr_hp_left_vol, curr_hp_right_vol);
	dlv_sleep(delay);

	/* set 16ohm load to keep away from the bug can not waited RDO */
	if(__dlv_get_load() == LOAD_10KOHM)
	{
		__dlv_set_16ohm_load();
		load_flag = 1;
	}

#ifndef CONFIG_HP_SENSE_DETECT
	//clear rdo rup
	__dlv_clear_rdo();
	__dlv_clear_rup();

	/* turn off sb_hp */
	__dlv_switch_sb_hp(POWER_OFF);
	dlv_sleep_wait_bitset(DLV_REG_IFR, IFR_RDO, 100,__LINE__);
#endif //nodef CONFIG_HP_SENSE_DETECT

	while (1) {
		dlv_ifr = __dlv_get_irq_flag();
		printk("waiting for SCMC recover finish ----- dlv_ifr = 0x%02x\n", dlv_ifr);
		if ((dlv_ifr & ((1 << IFR_SCMC) | (1 << IFR_SCLR))) == 0) 
			break; 
		__dlv_set_irq_flag((1 << IFR_SCMC));
		__dlv_set_irq_flag((1 << IFR_SCLR));
		dlv_sleep(10);
	}

#ifndef CONFIG_HP_SENSE_DETECT
	/* turn on sb_hp */
	__dlv_switch_sb_hp(POWER_ON);
	dlv_sleep_wait_bitset(DLV_REG_IFR, IFR_RUP, 100,__LINE__);
#endif //nodef CONFIG_HP_SENSE_DETECT

	if(load_flag)
		__dlv_set_10kohm_load();

	/* restore hp gain */
	dlv_set_gain_hp_left(curr_hp_left_vol);
	dlv_set_gain_hp_right(curr_hp_right_vol);

	dlv_sleep(delay);

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

		if (dlv_ifr & ((1 << IFR_SCMC) | (1 << IFR_SCLR))) {
			dlv_short_circut_handler();
		}
		
		dlv_ifr = __dlv_get_irq_flag();

		/* Updata SCMC */
		__dlv_set_irq_flag((1 << IFR_SCMC));
		__dlv_set_irq_flag((1 << IFR_SCLR));

		/* Unmask SCMC */
		__dlv_set_irq_mask(ICR_COMMON_MASK);

		printk("JZ DLV: Short circut detected! dlv_ifr = 0x%02x\n", dlv_ifr);
		
	} while(dlv_ifr & ((1 << IFR_SCMC) | (1 << IFR_SCLR)));
	
	handling_scmc = 0;

	if (dlv_ifr & (1 << IFR_JACK_EVENT)) {
_ensure_stable:
		j = 0;
		old_status = (__dlv_get_irq_flag() & (1 << IFR_JACK_EVENT)) != 0;
		/* Read status at least 3 times to make sure it is stable. */
		for (i = 0; i < 3; ++i) {
			new_status = (dlv_read_reg(DLV_REG_SR) & (1 << SR_JACK)) != 0;

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
	if (dlv_ifr & (1 << IFR_JACK_EVENT)) {
		dlv_sleep(50);
		new_status = (dlv_read_reg(DLV_REG_SR) & (1 << SR_JACK)) != 0;
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

	if (!(dlv_ifr & ((1 << IFR_SCMC) | (1 << IFR_JACK_EVENT) | (1 << IFR_SCLR)))) {
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

		if (dlv_ifr & ((1 << IFR_SCMC) | (1 << IFR_SCLR))) {
			dlv_short_circut_handler();
		}
		
		dlv_ifr = __dlv_get_irq_flag();

		/* Updata SCMC/SCLR */
		__dlv_set_irq_flag((1 << IFR_SCMC));
		__dlv_set_irq_flag((1 << IFR_SCLR));

		/* Unmask SCMC/SCLR */
		__dlv_set_irq_mask(ICR_COMMON_MASK);

		printk("JZ DLV: Short circut detected! dlv_ifr = 0x%02x\n",dlv_ifr);
		
	} while(dlv_ifr & ((1 << IFR_SCMC) | (1 << IFR_SCLR)));
	
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

	/* Mask all irq temporarily */
	__dlv_set_irq_mask(ICR_ALL_MASK);

	aic_reg = REG_AIC_SR;
	
	REG_AIC_SR = 0x78; // legacy ... need check

	if (!(dlv_ifr & ((1 << IFR_SCMC) | (1 << IFR_JACK_EVENT) | (1 << IFR_SCLR)))) {
		/* CODEC may generate irq with flag = 0xc0.
		 * We have to ingore it in this case as there is no mask for the reserve bit.
		 */

		printk("AIC interrupt ??? AIC_SR = 0x%08x\n", aic_reg);

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

	switch_set_state(&switch_data->sdev, dlv_read_reg(DLV_REG_IFR) & (1 << IFR_JACK_EVENT));

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
 *                                                                                     *
 *module init                                                                          *
 *                                                                                     *
\***************************************************************************************/

/**
 * Module init
 */
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

	register_jz_codecs((void *)jzdlv_ioctl);

	dlv_reset_part();
	retval = request_irq(IRQ_AIC, dlv_codec_irq, IRQF_DISABLED, "dlv_codec_irq", NULL);
	if (retval) {
		printk("JZ DLV: Could not get AIC CODEC irq %d\n", IRQ_AIC);
		return retval;
	}

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
