/*
 * Linux/sound/oss/xb47XX/npca110p/npca110p_dlv.c
 *
 * DLV CODEC driver for Ingenic Jz4760b MIPS processor
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
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/input.h>

#include "../jz47XX_codec.h"

#include "npca110p.h"
#include "npca110p.c"

/***************************************************************************************\
 *                                                                                     *
 *global variable and structure interface                                              *
 *                                                                                     *
\*******************************************************************************
********/

static unsigned int cur_route = REPLAY_HP_STEREO;
static unsigned int keep_old_route = -1;

static unsigned int replay_sound = 100;
static unsigned int speaker_power_last = 0;

/* 0 ---> 16 bit */
/* 1 ---> 18 bit */
/* 2 ---> 20 bit */
/* 3 ---> 24 bit */
static unsigned int replay_data_width = 3;
static unsigned int record_data_width = 3;

#ifdef CONFIG_NPCA110P_LINEIN_CONTROL_BY_USERSPACE
static int user_linein_enable = 0;
static struct mutex user_linein_enable_mutex;
#endif

int linein_last = 0;
struct input_dev *linein_input = NULL;
struct delayed_work linein_delayed_work;
static BLOCKING_NOTIFIER_HEAD(npca110p_notify_chain);
int replay_last = 0;
EXPORT_SYMBOL_GPL(replay_last);
struct semaphore linein_is_playing;
EXPORT_SYMBOL_GPL(linein_is_playing);

extern void i2c_jz_setclk(struct i2c_client *client,unsigned long i2cclk);
static struct i2c_client *npca110p_client = NULL;

static int dlv_bsp_mute(int val);
static int dlv_set_replay_volume(int vol);

/*=================== lock ============================*/
static struct semaphore *g_dlv_sem = 0;

#define DLV_LOCK()							\
	do{								\
		if(g_dlv_sem)						\
			down(g_dlv_sem);				\
	}while(0)

#define DLV_UNLOCK()							\
	do{								\
		if(g_dlv_sem)						\
			up(g_dlv_sem);					\
	}while(0)

#define DLV_LOCKINIT()							\
	do{								\
		if(g_dlv_sem == NULL)					\
			g_dlv_sem = (struct semaphore *)vmalloc(sizeof(struct semaphore)); \
		if(g_dlv_sem)						\
			init_MUTEX(g_dlv_sem);			\
	}while(0)

#define DLV_LOCKDEINIT()						\
	do{								\
		if(g_dlv_sem)						\
			vfree(g_dlv_sem);				\
		g_dlv_sem = NULL;					\
	}while(0)

/**
 * dlv_sleep
 *
 *if use in suspend and resume, should use delay
 */
static int g_dlv_sleep_mode = 0;
void dlv_sleep(int ms)
{
	if(g_dlv_sleep_mode)
		msleep(ms);
	else
		mdelay(ms);
}

static void speaker_power(int on)
{
        __gpio_as_output(GPIO_SPEAKER_POWER);

        if (on)
                __gpio_set_pin(GPIO_SPEAKER_POWER);
        else
                __gpio_clear_pin(GPIO_SPEAKER_POWER);
}

static void npca110p_power(int on)
{
        __gpio_as_output(GPIO_NPCA110P_POWER);

        if (on)
                __gpio_set_pin(GPIO_NPCA110P_POWER);
        else
                __gpio_clear_pin(GPIO_NPCA110P_POWER);
}

static void npca110p_reset(void)
{
        __gpio_as_output(GPIO_NPCA110P_RESET);
        __gpio_clear_pin(GPIO_NPCA110P_RESET);
        dlv_sleep(5);
        __gpio_set_pin(GPIO_NPCA110P_RESET);
        dlv_sleep(10);
}

static int npca110p_i2c_write(const char *data, unsigned int len)
{
        int ret = 0;
        struct i2c_client *client = npca110p_client;

        ret = i2c_master_send(client, data, len);

        if (ret < 0)
                printk("npca110p_i2c_write err\n");
        return ret;
}

static void set_audio_dac_vol(unsigned int volume)
{
        unsigned char maxx_dsp_tx_buf[3] = {0,};
        int ret = 0;
        maxx_dsp_tx_buf[0] = (unsigned char)(volume >> 16);
        maxx_dsp_tx_buf[1] = (unsigned char)(volume >> 8);
        maxx_dsp_tx_buf[2] = (unsigned char)(volume & 0xff);
        ret = npca110p_i2c_write(maxx_dsp_tx_buf, sizeof(maxx_dsp_tx_buf) / sizeof(*maxx_dsp_tx_buf));
        if (ret < 0)
                printk("===>set audio dac vol error\n");
}

static void set_audio_codec_volume(unsigned char volume)
{
        unsigned int current_vol = 0;
        if(volume > 0x54)
                volume = 50;
        set_audio_dac_vol(0xffad80);
        current_vol = (unsigned int)((volume << 8) | volume);
        set_audio_dac_vol(current_vol);
        set_audio_dac_vol(0xffad86);
        set_audio_dac_vol(0x000000);
        set_audio_dac_vol(0xffad86);
        set_audio_dac_vol(0x000001);
}

static void set_audio_codec_maxxbass(unsigned char volume)
{
        unsigned char maxx_dsp_tx_buf[3] = {0,};
        int ret = 0;

        maxx_dsp_tx_buf[0] = 0x00;
        maxx_dsp_tx_buf[1] = 0xd1;
        maxx_dsp_tx_buf[2] = g_maxxbass_value[volume];
        ret = npca110p_i2c_write(maxx_dsp_tx_buf, sizeof(maxx_dsp_tx_buf) / sizeof(*maxx_dsp_tx_buf));
        if (ret < 0)
                printk("===>set audio codec maxxbass error\n");
}

static int npca110p_reconfig(void)
{
        int ret = 0;

        dlv_bsp_mute(1);
        npca110p_reset();
        ret = npca110p_i2c_write(g_abMaxxDSPCommands, sizeof(g_abMaxxDSPCommands) / sizeof(*g_abMaxxDSPCommands));
        if (ret < 0) {
                printk("===>codec reconf error\n");
                return ret;
        }
        dlv_bsp_mute(0);
        dlv_set_replay_volume(replay_sound);
        return 0;
}

/***************************************************************************************\
 *                                                                                     *
 *route part and attibute                                                              *
 *                                                                                     *
\***************************************************************************************/
static void dlv_set_linein(int mode)
{
        int ret = 0;
	switch(mode){

	case LINEIN_WITH_BYPASS:
                npca110p_reset();
                ret = npca110p_i2c_write(g_abMaxxDSPCommands_linein, sizeof(g_abMaxxDSPCommands_linein) / sizeof(*g_abMaxxDSPCommands_linein));
                if (ret < 0)
                        printk("===>codec set linein error\n");
                dlv_set_replay_volume(replay_sound);
		break;
	default:
		printk("JZ_DLV: line: %d, linein mode error!\n", __LINE__);
	}
}

static void dlv_set_lineout(int mode)
{
        int ret = 0;
	switch(mode){

	case LINEOUT_STEREO:
                npca110p_reset();
                ret = npca110p_i2c_write(g_abMaxxDSPCommands, sizeof(g_abMaxxDSPCommands) / sizeof(*g_abMaxxDSPCommands));
                if (ret < 0)
                        printk("===>codec set lineout error\n");
                dlv_set_replay_volume(replay_sound);
		break;
	default:
		printk("JZ_DLV: line: %d, lineout mode error!\n", __LINE__);
	}
}

/***************************************************************************************\
 *                                                                                     *
 *dlv route                                                                            *
 *                                                                                     *
\***************************************************************************************/

static void dlv_set_route_base(const void *arg)
{
	route_conf_base *conf = (route_conf_base *)arg;

	/*--------------route---------------*/
	/* record path */
	if (conf->route_linein_mode)
		dlv_set_linein(conf->route_linein_mode);

	if (conf->route_lineout_mode)
		dlv_set_lineout(conf->route_lineout_mode);
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
        if (speaker_power_last)
                speaker_power(0);
	return 0;
}

static int dlv_set_gpio_after_set_route(int route)
{
        if (speaker_power_last)
                speaker_power(1);
	return 0;
}

/*-----------------main fun-------------------*/

int dlv_set_route(int route)
{
	int i = 0;

        dlv_set_gpio_before_set_route(route);
	if(cur_route != route)
	{
		for (i = 0; i < ROUTE_COUNT; i ++)
		{
			if (route == dlv_route_info[i].route_name)
			{
				dlv_set_route_base(dlv_route_info[i].route_conf);
				keep_old_route = cur_route;
				cur_route = route;
				break;
			}
		}
		if (i == ROUTE_COUNT)
			printk("SET_ROUTE: dlv set route error!, undecleard route, route = %d\n", route);
	} else
		printk("SET_ROUTE: need not to set!, current route is route now!\n");
        dlv_set_gpio_after_set_route(route);

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
        int ret = 0;

        npca110p_reset();
        ret = npca110p_i2c_write(g_abMaxxDSPCommands, sizeof(g_abMaxxDSPCommands) / sizeof(*g_abMaxxDSPCommands));
        if (ret < 0) {
                printk("===>codec init error\n");
                return ret;
        }

        return ret;
}

static int dlv_init(void)
{
	int ret = 0;

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
        speaker_power(0);
        npca110p_power(0);
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
        npca110p_reset();
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
        /* need do nothing. */
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
                ret = dlv_anti_pop_part();
		break;
	}

        /* need do nothing. */

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

	ret = dlv_set_route(ROUTE_ALL_CLEAR);
	if(ret != ROUTE_ALL_CLEAR)
	{
		printk("JZ CODEC: dlv_suspend_part error!\n");
		return -1;
	}

        if (speaker_power_last)
                speaker_power(0);

	return 0;
}

static int dlv_suspend(void)
{
	int ret;
	g_dlv_sleep_mode = 0;

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
        int ret = 0;
        int route = keep_old_route;
	ret = dlv_set_route(route);

	if(ret != route)
	{
		printk("JZ CODEC: dlv_resume_part error!\n");
		return -1;
	}

        if (speaker_power_last)
                speaker_power(1);

	return 0;
}

static int dlv_resume(void)
{
	int ret;

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

	switch (snd_dev_cfg->device) {

	case SND_DEVICE_SPEAKER:
		ret = dlv_set_route(REPLAY_HP_STEREO);
		if(ret != REPLAY_HP_STEREO)
		{
			printk("JZ CODEC: set device SND_DEVICE_SPEAKER error!\n");
			return -1;
		}
		break;

        case SND_DEVICE_LINEIN:
                ret = dlv_set_route(BYPASS_LINEIN_TO_OUT);
		if(ret != BYPASS_LINEIN_TO_OUT)
		{
			printk("JZ CODEC: set device SND_DEVICE_HEADSET_AND_SPEAKER error!\n");
			return -1;
		}
                break;

	default:
		printk("JZ DLV: Unkown ioctl argument in SND_SET_DEVICE\n");
	};

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
        /* we need do nothing */
	return 0;
}

/*---------------------------------------*/
/**
 * CODEC set record rate & data width & volume & channel
 *
 */

static int dlv_set_record_rate(int rate)
{
        /* don't use record mode */
	return rate;
}

static int dlv_set_record_data_width(int width)
{
        /* don't use record mode */
	return width;
}

static int dlv_set_record_volume(int val)
{
        /* don't use record mode */
	return val;
}

static int dlv_set_record_channel(int channel)
{
        /* don't use record mode */
	return channel;
}

/*---------------------------------------*/
/**
 * CODEC set replay rate & data width & volume & channel
 *
 */

static int dlv_set_replay_rate(int rate)
{
        if (rate != 44100 && rate != 48000) {
                printk("===>CARE: codec don't support this rate\n");
                goto finish;
        }

        if (rate == 44100) {
		cpm_set_clock(CGU_I2SCLK, 11289600);
                g_abMaxxDSPCommands[SAMPLE_RATE_OFFSET_ONE] = SAMPLE_RATE_ONE_44100;
                g_abMaxxDSPCommands[SAMPLE_RATE_OFFSET_TWO] = SAMPLE_RATE_TWO_44100;
        } else {
                cpm_set_clock(CGU_I2SCLK, 12288000);
                g_abMaxxDSPCommands[SAMPLE_RATE_OFFSET_ONE] = SAMPLE_RATE_ONE_48000;
                g_abMaxxDSPCommands[SAMPLE_RATE_OFFSET_TWO] = SAMPLE_RATE_TWO_48000;
        }

        npca110p_reconfig();
finish:
       	return rate;
}

static int dlv_set_replay_data_width(int width)
{
	unsigned int supported_width[4] = {16, 18, 20, 24};
	unsigned int fix_width = 0;
        int ret = 0;
        unsigned char buf[3] = {0xbf, 0xc4, 0xc5};

        if (width == replay_data_width)
                goto finish;
	for(; fix_width < 4; ++fix_width)
	{
		if (width <= supported_width[fix_width])
			break;
	}

	if (fix_width == 4) {
                printk("===>npca110p don't support %d data width\n", width);
                goto finish;
        }

        buf[0] = (buf[0] & REPLAY_DATA_WIDTH_MASK) | REPLAY_DATA_WIDTH(fix_width) | RECORD_DATA_WIDTH(record_data_width);

        ret = npca110p_i2c_write(buf, sizeof(buf) / sizeof(*buf));
        if (ret < 0)
                printk("===>set replay data width error\n");
        replay_data_width = width;

finish:
        return width;
}

static int dlv_set_replay_volume(int vol)
{
        int index = 0;
        if (vol < 0)
		vol = 0;
	else if (vol > 100)
		vol = 100;

        index = 20 - (100 - vol) / 5;
        index = index != 0 ? index - 1 : index;

        set_audio_codec_volume(volume_map_to_maxxi_audio[index]);
        set_audio_codec_maxxbass(index);
        speaker_power((!!vol) && speaker_power_last);
        replay_sound = vol;

	return vol;
}

static int dlv_set_replay_channel(int channel)
{
        /* need do nothing. */

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
        /* we only use adc1, so here don't mute adc2 */
        if (val) {
                set_audio_dac_vol(0xffad80);
                set_audio_dac_vol(0x03ffff);
        } else {
                set_audio_dac_vol(0xffad80);
                set_audio_dac_vol(0x00ffff);
                dlv_set_replay_volume(replay_sound);
        }
	return 0;
}

static int dlv_bsp_mute(int val)
{
	int ret = 0;

        speaker_power(!val);
        speaker_power_last = !val;

        return ret;
}
/*---------------------------------------*/

static int dlv_debug_routine(void *arg)
{
        printk("=====>CARE: %s, %d\n", __func__, __LINE__);
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

static void detect_linein_edge(void)
{
        if(gpio_get_value(GPIO_LINEIN_DETECT) == 0) {
                __gpio_as_irq_rise_edge(GPIO_LINEIN_DETECT);
	} else {
		__gpio_as_irq_fall_edge(GPIO_LINEIN_DETECT);
	}
}

int npca110p_register_state_notifier(struct notifier_block *nb)
{
        return blocking_notifier_chain_register(&npca110p_notify_chain, nb);
}
EXPORT_SYMBOL(npca110p_register_state_notifier);

void npca110p_unregister_state_notifier(struct notifier_block *nb)
{
        blocking_notifier_chain_unregister(&npca110p_notify_chain, nb);
}
EXPORT_SYMBOL(npca110p_unregister_state_notifier);

static void npca110p_notifier_call_chain_sync(int state)
{
	blocking_notifier_call_chain(&npca110p_notify_chain, state, NULL);
}

static void linein_plug_change(int in)
{
#ifdef CONFIG_NPCA110P_LINEIN_CONTROL_BY_USERSPACE
        mutex_lock(&user_linein_enable_mutex);
        if (!user_linein_enable) {
                mutex_unlock(&user_linein_enable_mutex);
                return;
        }
        mutex_unlock(&user_linein_enable_mutex);
#endif
        npca110p_notifier_call_chain_sync(in);
}

static void linein_detect_work(struct work_struct *work)
{
        int value = 0;
        value = gpio_get_value(GPIO_LINEIN_DETECT);

        detect_linein_edge();

#ifdef CONFIG_NPCA110P_LINEIN_CONTROL_BY_USERSPACE
        mutex_lock(&user_linein_enable_mutex);
#endif
        if (value == linein_last)
#ifdef CONFIG_NPCA110P_LINEIN_CONTROL_BY_USERSPACE
        {
                mutex_unlock(&user_linein_enable_mutex);
#endif
                goto finish;
#ifdef CONFIG_NPCA110P_LINEIN_CONTROL_BY_USERSPACE
        }
#endif
        linein_last = value;
#ifdef CONFIG_NPCA110P_LINEIN_CONTROL_BY_USERSPACE
        mutex_unlock(&user_linein_enable_mutex);
#endif

        linein_plug_change(value == GPIO_LINEIN_ACTIVE ? 1 : 0);
        input_event(linein_input, EV_MSC, MSC_RAW,
                    value == GPIO_LINEIN_ACTIVE ? 1 : 0);
        input_sync(linein_input);
finish:
        enable_irq(GPIO_LINEIN_IRQ);
}

static irqreturn_t linein_detect_interrupt(int irq, void *dev_id)
{
	disable_irq_nosync(GPIO_LINEIN_IRQ);
	schedule_delayed_work(&linein_delayed_work, msecs_to_jiffies(300));

	return IRQ_HANDLED;
}

#ifdef CONFIG_NPCA110P_LINEIN_CONTROL_BY_USERSPACE
static ssize_t npca110p_linein_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
        int count = 0;

        mutex_lock(&user_linein_enable_mutex);
        count = sprintf(buf, "%d\n", user_linein_enable);
        mutex_unlock(&user_linein_enable_mutex);
        return count;
}

static ssize_t npca110p_linein_set(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
        int linein_enable = 0;

        if (strncmp(buf, "1", 1) == 0)
                linein_enable = 1;
        else
                linein_enable = 0;

        mutex_lock(&user_linein_enable_mutex);
        if (linein_enable == user_linein_enable)
                goto finish;
        if (!linein_enable && cur_route != REPLAY_HP_STEREO)
                npca110p_notifier_call_chain_sync(0);
        user_linein_enable = linein_enable;
        if (!user_linein_enable)
                goto finish;
        linein_enable = linein_last == GPIO_LINEIN_ACTIVE;
        if ((linein_enable && (cur_route == BYPASS_LINEIN_TO_OUT)) ||
            (!linein_enable && (cur_route == REPLAY_HP_STEREO)))
                goto finish;
        npca110p_notifier_call_chain_sync(linein_last == GPIO_LINEIN_ACTIVE ? 1 : 0);
finish:
        mutex_unlock(&user_linein_enable_mutex);
        return count;
}

static DEVICE_ATTR(linein, S_IWUSR | S_IRUSR | S_IROTH,
                   npca110p_linein_show, npca110p_linein_set);

static struct attribute *npca110p_attributes[] = {
	&dev_attr_linein.attr,
	NULL
};

static const struct attribute_group npca110p_attr_group = {
	.attrs = npca110p_attributes,
};
#endif

static int __devinit npca110p_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
        int ret = 0;

        __gpio_mask_irq(GPIO_LINEIN_DETECT);
        __gpio_as_input(GPIO_LINEIN_DETECT);
        __gpio_enable_pull(GPIO_LINEIN_DETECT);
        ret = request_irq(GPIO_LINEIN_IRQ,
                          linein_detect_interrupt,
                          IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                          "linein-detect", NULL);
        if (ret) {
                goto request_irq_failed;
        } else
                disable_irq(GPIO_LINEIN_IRQ);

        linein_input = input_allocate_device();
        if (linein_input == NULL) {
                ret = -ENOMEM;
                goto no_mem;
        }
        linein_input->name = "npca110p";
        linein_input->dev.parent = &client->dev;

        __set_bit(EV_MSC, linein_input->evbit);
        __set_bit(MSC_RAW, linein_input->mscbit);

        if ((ret = input_register_device(linein_input)) < 0) {
                goto input_reg_error;
        }

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ret = -ENODEV;
                goto check_i2c_error;
	}

	npca110p_client = client;
	i2c_jz_setclk(client, 100 * 1000);

        INIT_DELAYED_WORK(&linein_delayed_work, linein_detect_work);
        sema_init(&linein_is_playing, 1);

#ifdef CONFIG_NPCA110P_LINEIN_CONTROL_BY_USERSPACE
        mutex_init(&user_linein_enable_mutex);
        ret = sysfs_create_group(&client->dev.kobj, &npca110p_attr_group);
	if (ret < 0)
                goto sysfs_error;

#endif
        return 0;

#ifdef CONFIG_NPCA110P_LINEIN_CONTROL_BY_USERSPACE
sysfs_error:
#endif
check_i2c_error:
        input_unregister_device(linein_input);
input_reg_error:
        input_free_device(linein_input);
no_mem:
        free_irq(GPIO_LINEIN_IRQ, NULL);
request_irq_failed:

	return ret;
}
static int __devexit npca110p_i2c_remove(struct i2c_client *client)
{
	npca110p_client = NULL;
        free_irq(GPIO_LINEIN_IRQ, NULL);
        input_unregister_device(linein_input);
        input_free_device(linein_input);
        linein_input = NULL;
        return 0;
}

#ifdef CONFIG_PM
static int npca110p_suspend(struct i2c_client *client, pm_message_t state)
{
        return 0;
}

static int npca110p_resume(struct i2c_client *client)
{
        return 0;
}
#endif

static const struct i2c_device_id npca110p_id[] = {
        { "npca110p", 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, npca110p_id);

static struct i2c_driver npca110p_i2c_driver = {
        .driver.name = "npca110p",
        .probe       = npca110p_i2c_probe,
        .remove      = __devexit_p(npca110p_i2c_remove),
#ifdef CONFIG_PM
        .suspend = npca110p_suspend,
        .resume = npca110p_resume,
#endif
        .id_table    = npca110p_id,
};

/***************************************************************************************\
 *module init                                                                          *
\***************************************************************************************/
static int __init init_dlv(void)
{
	int ret = 0;

	cpm_start_clock(CGM_AIC);

        DLV_LOCKINIT();
        register_jz_codecs_ex((void *)jzdlv_ioctl, NULL);
        npca110p_power(1);

        ret = i2c_add_driver(&npca110p_i2c_driver);
        if (ret) {
		printk("JZ CODEC: Could net register npca110p driver\n");
                goto i2c_add_error;
        }

	return 0;

i2c_add_error:
        DLV_LOCKDEINIT();

        return ret;
}

/**
 * Module exit
 */
static void __exit cleanup_dlv(void)
{
	i2c_del_driver(&npca110p_i2c_driver);
        npca110p_power(0);
	DLV_LOCKDEINIT();
        cpm_stop_clock(CGM_AIC);
}

module_init(init_dlv);
module_exit(cleanup_dlv);

static int __init npca110p_late_init(void)
{
        __gpio_ack_irq(GPIO_LINEIN_DETECT);
        linein_last = gpio_get_value(GPIO_LINEIN_DETECT);
        if (linein_last == GPIO_LINEIN_ACTIVE ? 1 : 0)
                linein_plug_change(1);
        detect_linein_edge();
        __gpio_unmask_irq(GPIO_LINEIN_IRQ);
        enable_irq(GPIO_LINEIN_IRQ);

        return 0;
}

late_initcall(npca110p_late_init);

