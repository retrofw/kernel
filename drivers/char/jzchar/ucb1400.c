/*
 * ucb1400.c
 *
 * Touch screen driver interface to the UCB1400 codec.
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
#include <linux/ac97_codec.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/jzsoc.h>

#include "jz_ts.h"
#include "ucb1400.h"

#ifndef GPIO_TS_PENIRQ
#define GPIO_TS_PENIRQ 68
#endif

#define TS_PIN  GPIO_TS_PENIRQ
#define TS_IRQ  (IRQ_GPIO_0 + TS_PIN)

static int samples = 5; /* we sample 5 every time, and throw away the max and min cases, then use the average of the other 3 samples */
static int first_time = 0;
static unsigned long last_x, last_y, last_p;

static int adcsync = 0;

static unsigned int ucb_id = 0;
static struct ucb1400 *ucb;

extern struct ac97_codec * find_ac97_codec(void);

extern unsigned int (*codec_read_battery)(void);

/*------------------ UCB1400 routines ------------------*/

static inline void ucb1400_reg_write(unsigned char reg, unsigned short val)
{
	struct ac97_codec *codec = find_ac97_codec();
	if (!codec)
		return;
	codec->codec_write(codec, reg, val);
}

static inline unsigned int ucb1400_reg_read(unsigned char reg)
{
	struct ac97_codec *codec = find_ac97_codec();
	if (!codec)
		return 0;
	return codec->codec_read(codec, reg);
}

static void ucb1400_adc_enable(void)
{
	down(&ucb->adc_sem);

	ucb->adc_cr |= UCB_ADC_ENA;
	ucb1400_reg_write(UCB_ADC_CR, ucb->adc_cr);
}

static unsigned int ucb1400_adc_read(int adc_channel, int sync)
{
	unsigned int val, timeout = 10000;

	if (sync)
		adc_channel |= UCB_ADC_SYNC_ENA;

	ucb1400_reg_write(UCB_ADC_CR, ucb->adc_cr | adc_channel);
	ucb1400_reg_write(UCB_ADC_CR, ucb->adc_cr | adc_channel | UCB_ADC_START);

	for (;;) {
		val = ucb1400_reg_read(UCB_ADC_DATA);
		if (val & UCB_ADC_DAT_VAL)
			break;
		if (--timeout == 0)
			break;
		udelay(1);
	}

	return UCB_ADC_DAT(val);
}

static void ucb1400_adc_disable(void)
{
	ucb->adc_cr &= ~UCB_ADC_ENA;
	ucb1400_reg_write(UCB_ADC_CR, ucb->adc_cr);

	up(&ucb->adc_sem);
}

static void ucb1400_enable_irq(unsigned int idx, int edges)
{
	unsigned long flags;

	if (idx < 16) {
		spin_lock_irqsave(&ucb->lock, flags);

		/* This prevents spurious interrupts on the UCB1400 */
		ucb1400_reg_write(UCB_IE_CLEAR, 1 << idx);
		ucb1400_reg_write(UCB_IE_CLEAR, 0);

		if (edges & UCB_RISING) {
			ucb->irq_ris_enbl |= 1 << idx;
			ucb1400_reg_write(UCB_IE_RIS, ucb->irq_ris_enbl);
		}
		if (edges & UCB_FALLING) {
			ucb->irq_fal_enbl |= 1 << idx;
			ucb1400_reg_write(UCB_IE_FAL, ucb->irq_fal_enbl);
		}
		spin_unlock_irqrestore(&ucb->lock, flags);
	}
}

static void ucb1400_disable_irq(unsigned int idx, int edges)
{
	unsigned long flags;

	if (idx < 16) {
		spin_lock_irqsave(&ucb->lock, flags);

		if (edges & UCB_RISING) {
			ucb->irq_ris_enbl &= ~(1 << idx);
			ucb1400_reg_write(UCB_IE_RIS, ucb->irq_ris_enbl);
		}
		if (edges & UCB_FALLING) {
			ucb->irq_fal_enbl &= ~(1 << idx);
			ucb1400_reg_write(UCB_IE_FAL, ucb->irq_fal_enbl);
		}
		spin_unlock_irqrestore(&ucb->lock, flags);
	}
}

/*
 * Switch to interrupt mode.
 */
static inline void ucb1400_ts_mode_int(void)
{
	if (!ucb_id) {
		ucb_id = ucb1400_reg_read(UCB_ID);
	}

	if (ucb_id == UCB_ID_1400_BUGGY)
		ucb1400_reg_write(UCB_TS_CR,
				  UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_GND |
				  UCB_TS_CR_MODE_INT);
	else
		ucb1400_reg_write(UCB_TS_CR,
				  UCB_TS_CR_TSMX_POW | UCB_TS_CR_TSPX_POW |
				  UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_GND |
				  UCB_TS_CR_MODE_INT);
}

/*
 * Switch to pressure mode, and read pressure.  We don't need to wait
 * here, since both plates are being driven.
 */
static inline unsigned int ucb1400_ts_read_pressure(void)
{
	ucb1400_reg_write(UCB_TS_CR,
			  UCB_TS_CR_TSMX_POW | UCB_TS_CR_TSPX_POW |
			  UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_GND |
			  UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);

	return ucb1400_adc_read(UCB_ADC_INP_TSPY, adcsync);
}

/*
 * Switch to X position mode and measure Y plate.  We switch the plate
 * configuration in pressure mode, then switch to position mode.  This
 * gives a faster response time.  Even so, we need to wait about 55us
 * for things to stabilise.
 */
static inline unsigned int ucb1400_ts_read_xpos(void)
{
	ucb1400_reg_write(UCB_TS_CR,
			  UCB_TS_CR_TSMX_GND | UCB_TS_CR_TSPX_POW |
			  UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);
	ucb1400_reg_write(UCB_TS_CR,
			  UCB_TS_CR_TSMX_GND | UCB_TS_CR_TSPX_POW |
			  UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);
	ucb1400_reg_write(UCB_TS_CR,
			  UCB_TS_CR_TSMX_GND | UCB_TS_CR_TSPX_POW |
			  UCB_TS_CR_MODE_POS | UCB_TS_CR_BIAS_ENA);

	udelay(55);

	return ucb1400_adc_read(UCB_ADC_INP_TSPY, adcsync);
}

/*
 * Switch to Y position mode and measure X plate.  We switch the plate
 * configuration in pressure mode, then switch to position mode.  This
 * gives a faster response time.  Even so, we need to wait about 55us
 * for things to stabilise.
 */
static inline unsigned int ucb1400_ts_read_ypos(void)
{
	ucb1400_reg_write(UCB_TS_CR,
			  UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_POW |
			  UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);
	ucb1400_reg_write(UCB_TS_CR,
			  UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_POW |
			  UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);
	ucb1400_reg_write(UCB_TS_CR,
			  UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_POW |
			  UCB_TS_CR_MODE_POS | UCB_TS_CR_BIAS_ENA);

	udelay(55);

	return ucb1400_adc_read(UCB_ADC_INP_TSPX, adcsync);
}

/*------------------------------------------------------------
 * Read the battery voltage
 */

unsigned int ucb1400_read_battery(void)
{
	unsigned int v;

	ucb1400_adc_enable();

	// read twice to reduce fault value
	v =  ucb1400_adc_read(UCB_ADC_INP_AD0, adcsync);
	v =  ucb1400_adc_read(UCB_ADC_INP_AD0, adcsync);

	ucb1400_adc_disable();

//	printk("ucb1400_read_battery v=%d\n", v);

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
		if ((DIFF(x_cal, last_x) > 50) ||
		    (DIFF(y_cal, last_y) > 50))
			valid = 0;
		else
			valid = 1;
	}

//	printk("x_cal=%d y_cal=%d p_cal=%d valid=%d\n", x_cal, y_cal, p_cal, valid);

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
       if (ts->minx)
	{
		if (y < ts->minx) y = ts->miny;
		if (y > ts->maxx) y = ts->maxy;
		
		return (y - ts->miny) * SCREEN_Y / (ts->maxy - ts->miny);
	}
	else
	{
		if (y < TSMINX) y = TSMINY;
		if (y > TSMAXX) y = TSMAXY;
		
		return (y - TSMINY) * SCREEN_Y / (TSMAXY - TSMINY);
	}
}
 
/*------------------ Common routines -------------------*/

void ts_enable_irq(void)
{
	/* interrupt mode */
	ucb1400_ts_mode_int();
	ucb1400_enable_irq(UCB_IRQ_TSPX, UCB_FALLING);

	enable_irq(TS_IRQ);
}

void ts_disable_irq(void)
{
	ucb1400_disable_irq(UCB_IRQ_TSPX, UCB_FALLING);
	disable_irq(TS_IRQ);
}

int ts_request_irq(u32 *irq, 
		   void (*handler)(int, void *, struct pt_regs *),
		   const char *devname,
		   void *dev_id)
{
	int retval;

	/* return the irq number */
	*irq = TS_IRQ;

	/* interrupt mode */
	ucb1400_ts_mode_int();
	ucb1400_enable_irq(UCB_IRQ_TSPX, UCB_FALLING);

	/* enable gpio irq */
	__gpio_as_irq_rise_edge(TS_PIN);

	/* register irq handler */
	retval = request_irq(TS_IRQ, handler, SA_INTERRUPT, devname, dev_id);

	return retval;
}

void ts_free_irq(struct jz_ts_t *ts)
{
	free_irq(ts->pendown_irq, ts);
}

void ts_irq_callback(void)
{
	/* clear interrupt status */
	ucb1400_reg_write(UCB_IE_CLEAR, ucb1400_reg_read(UCB_IE_STATUS));
	__gpio_ack_irq(TS_PIN);

	first_time = 1; // first time to acquire sample
}

int PenIsDown(void)
{
	unsigned int p;

	ucb1400_adc_enable();
	p = ucb1400_ts_read_pressure();
	ucb1400_adc_disable();

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

	/* Enable ADC */
	ucb1400_adc_enable();

	for (i = 0; i < samples; i++) {
		x_raw[i] = ucb1400_ts_read_xpos();
	}
	for (i = 0; i < samples; i++) {
		y_raw[i] = ucb1400_ts_read_ypos();
	}
	for (i = 0; i < samples; i++) {
		p_raw[i] = ucb1400_ts_read_pressure();
	}

        /* Disable ADC */
	ucb1400_adc_disable();

	valid = calibrate_samples(x_raw, y_raw, p_raw, samples);

	if (valid) {
		unsigned int x_scr, y_scr;

		if(ts->filter) {
			x_scr = transform_to_screen_x(ts, x_raw[0]);
			y_scr = transform_to_screen_y(ts, y_raw[0]);

			if (ts->prints)
				printk("x_raw=%d y_raw=%d x_transform=%d y_transform=%d\n", x_raw[0], y_raw[0], x_scr, y_scr);
		}
		else {
			x_scr = x_raw[0];
			y_scr = y_raw[0];

			if (ts->prints)
				printk("x_raw=%d y_raw=%d \n", x_raw[0], y_raw[0]);
		}

		event->x = x_scr;
		event->y = y_scr;
		event->pressure = (u16)p_raw[0];
		event->status = PENDOWN;
		return 1;
	}
	return 0;
}

/*
 * Module init and exit
 */

int __init ucb1400_init(void)
{
	ucb = kmalloc(sizeof(struct ucb1400), GFP_KERNEL);
	if (!ucb) return -ENOMEM;

	memset(ucb, 0, sizeof(struct ucb1400));

	codec_read_battery = ucb1400_read_battery;

	spin_lock_init(&ucb->lock);
	sema_init(&ucb->adc_sem, 1);

	return 0;
}

void ucb1400_cleanup(void)
{
	kfree(ucb);
}

module_init(ucb1400_init);
module_exit(ucb1400_cleanup);
