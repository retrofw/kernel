/*
 * Linux/sound/oss/jz_dlv.c
 *
 * DLV CODEC driver for Ingenic Jz4750 MIPS processor
 *
 * 2009-12-xx	Steven <dsqiu@ingenic.cn>
 * 2010-01-xx	Jason <xwang@ingenic.cn>
 * 2010-11-xx   jbbi <jbbi@ingenic.cn>
 * 2014-06-01   tjin <tjin@ingenic.cn>   
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
//#include <linux/jz_audio.h>

#include "../jz47XX_codec.h"

#define DUMP_FUNC() printk("ES9023_DLV:%s\tline:%d\n", __func__, __LINE__)

#define GPIO_ES9023_DIF			(32*4 + 4)
#define GPIO_ES9023_MUTE		(32*4 + 13)
//#define GPIO_ES9023_PDN		(32*4 + 4)
//#define GPIO_TAS5713_INPUT_SELECT	(32*5 + 3)


/****
 *
 *  es9023_reset is a init step,you can read es9023 spec to understand it.
 *
 * ***/
void es9023_init(void)
{
        __gpio_as_output(GPIO_ES9023_DIF);
	__gpio_clear_pin(GPIO_ES9023_DIF);	 //select cpu's i2s output
        
	__gpio_as_output(GPIO_ES9023_MUTE);
	__gpio_clear_pin(GPIO_ES9023_MUTE);	 //enable es9023 mute

	return ;
}

void es9023_deinit(void)
{
	__gpio_as_output(GPIO_ES9023_MUTE);
	__gpio_clear_pin(GPIO_ES9023_MUTE);	 //enable es9023 mute

	//__gpio_clear_pin(GPIO_ES9023_PDN);   
	return ;
}

static void es9023_shutdown(void)
{
	printk("es9023_shutdown\n");
	//__gpio_clear_pin(GPIO_ES9023_PDN);   
	return ;
}

static void es9023_wakeup(void)
{
	printk("es9023_wakeup\n");
	//__gpio_set_pin(GPIO_ES9023_PDN);    
	return ;
}

/*
int tas5713_input_select(int input)
{
	printk(KERN_DEBUG "tas5713_input_select %s !\n", input?"adc line in":"local play");
	__gpio_as_output(GPIO_TAS5713_INPUT_SELECT);
	
	if (!input) {
		tas5713_dac_flag = 0;
		__gpio_clear_pin(GPIO_TAS5713_INPUT_SELECT);  //select cpu  i2s output
		tas5713_set_mute(1); // mute the tas5713 output if tvout
		
	} else {
		tas5713_dac_flag = 1;
		__gpio_set_pin(GPIO_TAS5713_INPUT_SELECT);  //select pcm1808 adc  i2s output
		tas5713_set_mute(0); // unmute
	}
	return 0;
}
*/

/***************************************************************************************\
 *global variable and structure interface                                              *
\***************************************************************************************/

static unsigned int g_current_out_dev;

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
			init_MUTEX(g_dlv_sem);			\
		DLV_DEBUG_SEM("dlvsemlock init\n");			\
	}while(0)

#define DLV_LOCKDEINIT()						\
	do{								\
		if(g_dlv_sem)						\
			vfree(g_dlv_sem);				\
		g_dlv_sem = NULL;					\
		DLV_DEBUG_SEM("dlvsemlock deinit\n");			\
	}while(0)

/***************************************************************************************\
 *debug part                                                                           *
\***************************************************************************************/
/*###############################################*/

#define DLV_DUMP_IOC_CMD		0

/*##############################################*/
/***
 *
 *This is jzdlv_ioctl() command list.jzdlv_ioctl() will be called in jz47xx_i2s.c to configure the codec.
 *
 * ***/
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
/*=========================================================*/

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

/***************************************************************************************\
 *                                                                                     *
 *dlv route        This is first designed for jz47xx internel codec route, other outside codecs can use it freely as you need. Here retains the route interface for future.
 *                                                                                     *
\***************************************************************************************/

/***************************************************************************************\
 *ioctl support function                                                               *
\***************************************************************************************/

/*------------------sub fun-------------------*/

static void DAMP_MuteOn(void)
{
	__gpio_clear_pin(GPIO_ES9023_MUTE);	 //GPIO MUTE 
	return;	
}

static void DAMP_MuteOff(void)
{
	__gpio_set_pin(GPIO_ES9023_MUTE);	 //GPIO MUTE 
	return;	
}

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
	return 0;
}

static int dlv_init(void)
{
	int ret;

	/* set default route */
	g_current_out_dev = REPLAY_HP_STEREO;

	/* dlv init */
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
	printk("ES9023_DLV: waring, %s() is a default function\n", __func__);

	if ((mode & REPLAY) && (mode & RECORD)) {
		printk("ES9023 DLV: Close REPLAY & RECORD\n");
		//es9023_shutdown();

	} else if (mode & REPLAY) {
		printk("ES9023 DLV: Close REPLAY\n");
		//DAMP_MuteOn();
		//es9023_shutdown();

	} else if (mode & RECORD) {
		printk("ES9023 DLV: Close RECORD\n");
		//es9023_shutdown();
	}

	return 0;
}

static int dlv_turn_off(int mode)
{
	int ret;

	ret = dlv_turn_off_part(mode);

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
	return 0;
}

static int dlv_shutdown(void)
{
	int ret;
		
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
	return 0;
}

static int dlv_reset(void)
{
	int ret;

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
	//DAMP_MuteOff();
	//es9023_wakeup();
	return 0;
}

static int dlv_anti_pop(int mode)
{
	int ret = 0;

	switch(mode) {
	case CODEC_WRMODE:
	case CODEC_RMODE:
	case CODEC_WMODE:
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
	DAMP_MuteOn();
	return 0;
}

static int dlv_suspend(void)
{
	int ret;
	
	ret = dlv_suspend_part();

	return ret;
}

/********* dlv_resume ***********/
/**
 * CODEC dlv resume part
 *
 * it will do the resume as default, it can be changed 
 * depend on difficent boards if necessary
 *
 **/

static int dlv_resume_part(void)
{
	DAMP_MuteOff();
	return 0;
}

static int dlv_resume(void)
{
	int ret;
	ret = dlv_resume_part();

	return ret;
}

/*---------------------------------------*/

/**
 * CODEC set device
 *
 * This is just a demo function, and it will be use as default. 
 * Wether it need to be realized, depends on different boards.
 * Here just a unuseful interface for es9023. 
 *
 */
static int dlv_set_device(struct snd_device_config *snd_dev_cfg)
{
	int iserror = 0;

	printk("ES9023_DLV: waring, %s() is a default function\n", __func__);

	switch (snd_dev_cfg->device) {

	case SND_DEVICE_HEADSET:
		break;

	case SND_DEVICE_HANDSET:
		break;

	case SND_DEVICE_SPEAKER:
		break;

	case SND_DEVICE_HEADSET_AND_SPEAKER:
		break;

	default:
		iserror = 1;
		printk("ES9023 DLV: Unkown ioctl argument in SND_SET_DEVICE\n");
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
	switch(g_current_out_dev) {

	case SND_DEVICE_HEADSET:
		break;

	case SND_DEVICE_HANDSET:
		break;

	case SND_DEVICE_SPEAKER:
		break;

	case SND_DEVICE_HEADSET_AND_SPEAKER:
		break;

	default:
		printk("ES9023 DLV: Unkown ioctl argument in SND_SET_STANDBY\n");

	}

	return 0;
}

/*---------------------------------------*/
/**
 * CODEC set record rate & data width & volume & channel, es9023 can't support it.It is just interface. You can use it for other outside codecs in the future.  
 *
 */

static int dlv_set_record_rate(int rate)
{
	printk("es9023 can't support record\n");
	return rate;
}

static int dlv_set_record_data_width(int width)
{
	printk("es9023 can't support record\n");
	return width;
}

static int dlv_set_record_volume(int val)
{
	printk("es9023 can't support record\n");
	return val;
}

static int dlv_set_record_channel(int channel)
{
	printk("es9023 can't support record\n");
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

	int cpmrate[MAX_RATE_COUNT] = {
		24576000,12288000,11289600,8192000,6144000,
		5644800, 4096000, 3072000, 2822400,2048000		
	};
	
	for (val = 0; val < MAX_RATE_COUNT - 1; val++) {
		if (rate >= (mrate[val] + mrate[val+1])/2) {
			speed = val;
			break;
		}
	}
	if (val == (MAX_RATE_COUNT - 1))
		speed = val;
		
	cpm_set_clock(CGU_I2SCLK, cpmrate[speed]);
	
	printk("%s  %d\n",__FUNCTION__, mrate[speed]);

	return mrate[speed];
}

static int dlv_set_replay_data_width(int width)
{
	int supported_width[3] = {16, 20, 24};
	int fix_width;

	for(fix_width = 0; fix_width < 3; fix_width ++)
	{
		if (width <= supported_width[fix_width])
			break;
	}

	return width;
}


static int dlv_set_replay_volume(int vol)
{
	if(vol < 0)
		vol = 0;
	else if(vol > 100)
		vol = 100;
	
	if(vol == 0){
		DAMP_MuteOn();
	}else{
		DAMP_MuteOff();
	}
	
	mdelay(10);
	return vol;
}

/* The function is not be use, just a interface for the future. */
static int dlv_set_replay_channel(int channel)
{
	channel = (channel >= 2) + 1;

	switch (channel) {
	case 1:
		// MONO->1 for Mono
		break;
	case 2:
		// MONO->0 for Stereo
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

	return 0;
}
/*---------------------------------------*/

static int dlv_debug_routine(void *arg)
{
	return 0;
}

/**
 * CODEC ioctl (simulated) routine. It will be called by jz47xx_i2s.c 
 * to configure the codec.
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
			ret = (int)arg;
			break;

		case CODEC_SET_DEVICE:
			ret = dlv_set_device((struct snd_device_config *)arg);
			break;

		case CODEC_SET_STANDBY:
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

		case CODEC_DEBUG_ROUTINE:
			ret = dlv_debug_routine((void *)arg);
			break;

		default:
			printk("ES9023 DLV:%s:%d: Unkown IOC commond\n", __FUNCTION__, __LINE__);
			ret = -1;
		}
	}
	DLV_UNLOCK();

	return ret;
}

/**
 * Module init
 */
static int __init init_dlv(void)
{
	cpm_start_clock(CGM_AIC);

	DLV_LOCKINIT();

	printk("=============================================\n");
	
	/* just notice jz47xx_i2s.c the codec's ioctl function */
	register_jz_codecs_ex((void *)jzdlv_ioctl, NULL);  

	/* GPIO pin set */
	es9023_init();       

	return 0;
}

/**
 * Module exit
 */
static void __exit cleanup_dlv(void)
{
	/* GPIO pin power down */
	es9023_deinit();

	DLV_LOCKDEINIT();  
	return ;
}

module_init(init_dlv);
module_exit(cleanup_dlv);
