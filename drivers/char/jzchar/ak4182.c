/*
 * ak4182.c using national microwire protocol
 *
 * Touch screen driver interface to the AK4182A .
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/pm.h>
#include <linux/pm_legacy.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/jzsoc.h>

#include "jz_ts.h"
#include "ak4182.h"

#define TS_PIN  GPIO_TS_PENIRQ
#define TS_IRQ  (IRQ_GPIO_0 + TS_PIN)

static int samples = 5;
static int first_time = 0;
static unsigned long last_x, last_y, last_p;

static int adcsync = 0;

static struct ak4182 *ak;

extern unsigned int (*codec_read_battery)(void);

/*------------------JzSoc SSI configure----------------*/
static void ak4182_ssi_reset(void)
{
	REG_SSI_CR0 = 0x0000;
	REG_SSI_CR1 = 0x00007960;
	REG_SSI_SR  = 0x00000098;
	REG_SSI_ITR = 0x0000;
	REG_SSI_ICR = 0x00;
	REG_SSI_GR  = 0x0000;

	__ssi_disable();
	__ssi_flush_fifo();
	__ssi_clear_errors();
	__ssi_select_ce();
}

static void ak4182_ssi_enable(void)
{
	__ssi_enable();
}

#ifdef CONFIG_PM
static void ak4182_ssi_disable(void)
{
	__ssi_disable();
}
#endif

static void ak4182_ssi_set_trans_mode_format(void)
{
	__ssi_microwire_format();
	__ssi_set_msb();
	__ssi_set_microwire_command_length(8);
	__ssi_set_frame_length(12);
}

static void ak4182_ssi_set_clk_div_ratio(int dev_clk, int ssi_clk)
{
	__ssi_set_clk(dev_clk, ssi_clk);
}

static void ak4182_ssi_set_normal_mode(void)
{
	__ssi_normal_mode();
}

static void ak4182_ssi_set_IRQ(void)
{
	__ssi_disable_tx_intr();
	__ssi_disable_rx_intr();
}

/*------------------ AK4182 routines ------------------*/
static inline void ak4182_reg_write(unsigned short val)
{
	__ssi_transmit_data(val);
}

static inline unsigned int ak4182_reg_read(void)
{
	unsigned int val;
	val = __ssi_receive_data();
	return val;
}

static unsigned int ak4182_adc_read(int cmd_code, int sync)
{
	unsigned int val, timeout = 10000;
	unsigned int status,valid1,valid2,dataentry;

	ak4182_reg_write(cmd_code);
	udelay(2);//wait 2 D_CLK
	for (;;) {
		status =0;
		status = REG_SSI_SR;
		valid1 = (status>>7) & 1;
		valid2 = (status>>6) & 1;
		if( valid1==1 && valid2==0 )//SSI transfer is finished
		{
			//Receive FIFO data entry number
			dataentry = val = 0;
			dataentry = (status>>8) & 0x1F;
			if( dataentry > 5 )
			{
				printk("R-FIFO entry=%d,SSI transfer is wrong!\n",dataentry);
				while(dataentry > 0)
				{
					ak4182_reg_read();
					dataentry--;
				}
				return 0;
			}
			while(dataentry > 0)
			{
				val = ak4182_reg_read();
				dataentry--;
			}
			return val;
		}

		if (--timeout == 0)
			break;
		udelay(1);
	}
	return 0;
}


//enable pen down IRQ
static void ak4182_enable_irq(void)
{
	unsigned long flags;

	spin_lock_irqsave(&ak->lock, flags);
	__gpio_unmask_irq(TS_PIN);
	spin_unlock_irqrestore(&ak->lock, flags);
}

//disable pen down IRQ
static void ak4182_disable_irq(void)
{
	unsigned long flags;

	spin_lock_irqsave(&ak->lock, flags);
	__gpio_mask_irq(TS_PIN);
//	spin_unlock_irqrestore(&ucb->lock, flags);
	spin_unlock_irqrestore(&ak->lock, flags);
}
/*
 * Switch to X position mode and measure Y plate.  We switch the plate
 * configuration in pressure mode, then switch to position mode.  This
 * gives a faster response time.  Even so, we need to wait about 55us
 * for things to stabilise.
 */
static inline unsigned int ak4182_ts_read_xpos(void)
{
	return ak4182_adc_read(0xD0, adcsync);//X-axis,0xD0 for 12bit,0xD8 for 8bit
}


/*
 * Switch to pressure mode, and read pressure.  We don't need to wait
 * here, since both plates are being driven.
 */
static inline unsigned int ak4182_ts_read_pressure(void)
{
	unsigned int z1,z2,xpos,pressureval=0;//300 Om
	//Z1 pressure
	z1 = ak4182_adc_read(0xB0, adcsync);//0xB0 for 12bit,0xB8 for 8bit
	if(z1>0)
	{
		//Z2 pressure
		z2 = ak4182_adc_read(0xC0, adcsync);//0xC0 for 12bit,0xC8 for 8bit
		if(z2>z1)
		{
			xpos = ak4182_ts_read_xpos();
			pressureval = (300*xpos*(z2-z1))/(4096*z1);
		}
	}
	
	return pressureval;
}


/*
 * Switch to Y position mode and measure X plate.  We switch the plate
 * configuration in pressure mode, then switch to position mode.  This
 * gives a faster response time.  Even so, we need to wait about 55us
 * for things to stabilise.
 */
static inline unsigned int ak4182_ts_read_ypos(void)
{
	return ak4182_adc_read(0x90, adcsync);//Y-axis,0x90 for 12bit,0x98 for 8bit
}

/*------------------------------------------------------------
 * Read the battery voltage
 */

unsigned int ak4182_read_battery(void)
{
	unsigned int v;
	int bat_val[5];
	int total = 0, max_bat, min_bat;
	
	v = ak4182_adc_read(0xA7, adcsync);
	v = ak4182_adc_read(0xA7, adcsync);
	for(v = 0;v <= 4;v++)
		bat_val[v] =  ak4182_adc_read(0xA7, adcsync);

	ak4182_adc_read(0xA4, adcsync);
	max_bat =  min_bat = bat_val[0];
	for(v = 0;v <= 4;v++) {
		total += bat_val[v];
		if(bat_val[v] > max_bat)
			max_bat = bat_val[v];
		if(bat_val[v] < min_bat)
			min_bat = bat_val[v];
	}
	total = total - max_bat - min_bat;
	v = total / 3;
	return v;
}

/*------------------ Calibrate samples -------------------*/

#define DIFF(a,b) ((a>b)?(a-b):(b-a))

static int calibrate_samples(void *xbuf, void *ybuf, void *pbuf, int count)
{
	unsigned long *xp = (unsigned long *)xbuf;
	unsigned long *yp = (unsigned long *)ybuf;
	unsigned long *pp = (unsigned long *)pbuf;
	unsigned long x_cal = 0, y_cal = 0, p_cal = 0, tmp;
	int ignored, i, j;
	int valid = 0;

	/* throw away the max cases */
	tmp = xp[0];
	ignored = 0;
	for (i = 1; i < count; i++) {
		if (xp[i] > tmp) {
			tmp = xp[i];
			ignored = i;
		}
	}//find the max val
	j = 0;
	for (i = 0; i < count; i++) {
		if (i == ignored)
			continue;
		xp[j++] = xp[i];
	}//shift val and delete the max val

	tmp = yp[0];
	ignored = 0;
	for (i = 1; i < count; i++) {
		if (yp[i] > tmp) {
			tmp = yp[i];
			ignored = i;
		}
	}
	j = 0;
	for (i = 0; i < count; i++) {
		if (i == ignored)
			continue;
		yp[j++] = yp[i];
	}

	tmp = pp[0];
	ignored = 0;
	for (i = 1; i < count; i++) {
		if (pp[i] > tmp) {
			tmp = pp[i];
			ignored = i;
		}
	}
	j = 0;
	for (i = 0; i < count; i++) {
		if (i == ignored)
			continue;
		pp[j++] = pp[i];
	}

	/* throw away the min cases */

	count -= 1; // decrement by 1

	tmp = xp[0];
	ignored = 0;
	for (i = 1; i < count; i++) {
		if (xp[i] < tmp) {
			tmp = xp[i];
			ignored = i;
		}
	}
	j = 0;
	for (i = 0; i < count; i++) {
		if (i == ignored)
			continue;
		xp[j++] = xp[i];
	}

	tmp = yp[0];
	ignored = 0;
	for (i = 1; i < count; i++) {
		if (yp[i] < tmp) {
			tmp = yp[i];
			ignored = i;
		}
	}
	j = 0;
	for (i = 0; i < count; i++) {
		if (i == ignored)
			continue;
		yp[j++] = yp[i];
	}

	tmp = pp[0];
	ignored = 0;
	for (i = 1; i < count; i++) {
		if (pp[i] < tmp) {
			tmp = pp[i];
			ignored = i;
		}
	}
	j = 0;
	for (i = 0; i < count; i++) {
		if (i == ignored)
			continue;
		pp[j++] = pp[i];
	}

	count -= 1;  // decrement by 1

	/* calculate the average of the rest */
	for (i = 0; i < count; i++) {
		x_cal += xp[i];
		y_cal += yp[i];
		p_cal += pp[i];
	}
	x_cal /= count;
	y_cal /= count;
	p_cal /= count;

	if (first_time) {
		first_time = 0;
		last_x = x_cal;
		last_y = y_cal;
		last_p = p_cal;
		valid = 1;
	}
	else {
		if ((DIFF(x_cal, last_x) > 100) ||
		    (DIFF(y_cal, last_y) > 100))
			valid = 0;
		else
			valid = 1;
	}

	//printk("x_cal=%d y_cal=%d p_cal=%d valid=%d\n", x_cal, y_cal, p_cal, valid);

	if (valid) {
		*xp = last_x = x_cal;
		*yp = last_y = y_cal;
		*pp = last_p = p_cal;
	}

	return valid;
}

  
#define TSMAXX 945
#define TSMAXY 830
#define TSMINX 90
#define TSMINY 105

#define SCREEN_X 480
#define SCREEN_Y 272

static unsigned long transform_to_screen_x(struct jz_ts_t *ts, unsigned long x )
{       

        if (ts->minx)
	{
		if (x < ts->minx) x = ts->minx;
             	if (x > ts->maxx) x = ts->maxx;

		return (x - ts->minx) * SCREEN_X / (ts->maxx - ts->minx);
	}
	else
	{
		if (x < TSMINX) x = TSMINX;
		if (x > TSMAXX) x = TSMAXX;

		return (x - TSMINX) * SCREEN_X / (TSMAXX - TSMINX);
	}
}

static unsigned long transform_to_screen_y(struct jz_ts_t *ts, unsigned long y)
{
       if (ts->miny)
	{
		if (y < ts->miny) y = ts->miny;
		if (y > ts->maxy) y = ts->maxy;
		
		return (y - ts->miny) * SCREEN_Y / (ts->maxy - ts->miny);
	}
	else
	{
		if (y < TSMINY) y = TSMINY;
		if (y > TSMAXY) y = TSMAXY;
		
		return (y - TSMINY) * SCREEN_Y / (TSMAXY - TSMINY);
	}
}
 
/*------------------ Common routines -------------------*/

void ts_enable_irq(void)
{
	/* interrupt mode */
	ak4182_enable_irq();
	enable_irq(TS_IRQ);
}

void ts_disable_irq(void)
{
	ak4182_disable_irq();
	disable_irq(TS_IRQ);
}

int ts_request_irq(u32 *irq, 
		   irqreturn_t (*handler)(int, void *),
		   const char *devname,
		   void *dev_id)
{
	int retval;

	/* return the irq number */
	*irq = TS_IRQ;
	/* initializate ssi for AK4182 */
	ak4182_ssi_reset();
	ak4182_ssi_set_trans_mode_format();
	ak4182_ssi_set_normal_mode();
	ak4182_ssi_set_clk_div_ratio(JZ_EXTAL, 200*1000);//DCLK is 1.5M Hz max
	ak4182_ssi_set_IRQ();
     
	ak4182_enable_irq();
	
	/* enable gpio irq */
	__gpio_as_irq_fall_edge(TS_PIN);

	/* register irq handler */
	retval = request_irq(TS_IRQ, handler, IRQF_DISABLED, devname, dev_id);
	ak4182_ssi_enable();
	udelay(10);
	return retval;
}

void ts_free_irq(struct jz_ts_t *ts)
{
	free_irq(ts->pendown_irq, ts);
	//Close SSI mode
	ak4182_ssi_reset();
}

void ts_irq_callback(void)
{
	/* clear interrupt status */
	__gpio_ack_irq(TS_PIN);
	first_time = 1; // first time to acquire sample
}

int PenIsDown(void)
{
	unsigned int p;
	p = ak4182_ts_read_pressure();
	return (p > 100) ? 1 : 0;
}

/*
 * Acquire Raw pen coodinate data and compute touch screen
 * pressure resistance. Hold spinlock when calling.
 */
int AcquireEvent(struct jz_ts_t *ts, struct ts_event *event)
{
	unsigned int x_raw[8], y_raw[8], p_raw[8];
	int valid, i;

	for (i = 0; i < samples; i++) {
		x_raw[i] = ak4182_ts_read_xpos();
	}
	for (i = 0; i < samples; i++) {
		y_raw[i] = ak4182_ts_read_ypos();
	}
	for (i = 0; i < samples; i++) {
		p_raw[i] = ak4182_ts_read_pressure();
	}

	valid = calibrate_samples(x_raw, y_raw, p_raw, samples);

	if (valid) {
		unsigned int x_scr, y_scr;
	     
		if(ts->filter) {
			x_scr = transform_to_screen_x(ts, x_raw[0]);
			y_scr = transform_to_screen_y(ts, y_raw[0]);

			if (ts->prints)
				printk("filter:x_raw:%d,y_raw:%d,x_tran:%d,y_tran:%d\n", x_raw[0], y_raw[0], x_scr, y_scr);
		}
		else {
			x_scr = x_raw[0];
			y_scr = y_raw[0];

			if (ts->prints)
				printk("no filter:x_raw=%d y_raw=%d \n", x_raw[0], y_raw[0]);
		}

		event->x = x_scr;
		event->y = y_scr;
		event->pressure = (u16)p_raw[0];
		event->status = PENDOWN;
		return 1;
	}
	return 0;
}

#ifdef CONFIG_PM

/*
 * Suspend the Touch pad.
 */
static int ak4182_suspend(struct ak4182 *ak , int state)
{
	ak4182_ssi_disable();

	return 0;
}

/*
 * Resume the Touch panel.
 */
static int ak4182_resume(struct ak4182 *ak)
{
	/* initializate ssi for AK4182 */
	ak4182_ssi_reset();
	ak4182_ssi_set_trans_mode_format();
	ak4182_ssi_set_normal_mode();
	ak4182_ssi_set_clk_div_ratio(JZ_EXTAL, 200*1000);//DCLK is 1.5M Hz max
	ak4182_ssi_set_IRQ();

	ak4182_enable_irq();

	ak4182_ssi_enable();

	return 0;
}

static int ak4182_pm_callback(struct pm_dev *pm_dev, pm_request_t rqst, void *data)
{
	int ret;
	struct ak4182 *akinfo = pm_dev->data;

	if (!akinfo) 
		return -EINVAL;


	switch (rqst) {
	case PM_SUSPEND:
		ret = ak4182_suspend(akinfo, (int)data);
		break;

	case PM_RESUME:
		ret = ak4182_resume(akinfo);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

#endif /* CONFIG_PM */


/*
 * Module init and exit
 */

int __init ak4182_init(void)
{
	ak = kmalloc(sizeof(struct ak4182), GFP_KERNEL);
	if (!ak) return -ENOMEM;

	memset(ak, 0, sizeof(struct ak4182));

	codec_read_battery = ak4182_read_battery;

	spin_lock_init(&ak->lock);
	sema_init(&ak->adc_sem, 1);

	//initialize AK4182 register
	__gpio_clear_pin(73);
	__gpio_as_output(73);
	mdelay(2);
	__gpio_set_pin(73);
	__gpio_as_ssi();

	ak4182_read_battery();

#ifdef	CONFIG_PM
	ak->pmdev = pm_register(PM_SYS_DEV, PM_SYS_UNKNOWN, ak4182_pm_callback);
	if (ak->pmdev)
	{
		ak->pmdev->data = ak;
	}
#endif

	printk(JZ_SOC_NAME": AK4182 touch screen driver initialized.\n");

	return 0;
}

void ak4182_cleanup(void)
{
}

module_init(ak4182_init);
module_exit(ak4182_cleanup);

