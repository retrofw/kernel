/*
 * linux/drivers/char/jzchar/sadc.c
 *
 * SAR-ADC driver for JZ4740.
 *
 * Copyright (C) 2006  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/spinlock.h>

#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/jzsoc.h>

#include "jzchars.h"
#include "jz_ts.h"

MODULE_AUTHOR("Jianli Wei<jlwei@ingenic.cn>");
MODULE_DESCRIPTION("JZ4740 SADC driver");
MODULE_LICENSE("GPL");

#define SADC_NAME        "sadc"
static DECLARE_WAIT_QUEUE_HEAD (sadc_wait_queue);

struct sadc_device {
	int mode;
	int dma_chan;
	char *ts_buf;
	char *pbat_buf;
};

static struct sadc_device *sadc_dev;

static int samples = 3; /* we sample 3 every time */
static int first_time = 0;
static unsigned long last_x, last_y, last_p;

typedef struct datasource {
        u16 xbuf;
        u16 ybuf;
	u16 zbuf;
}datasource_t;

static datasource_t data_s;
static unsigned int p;
static unsigned int old_x, old_y;
extern unsigned int (*codec_read_battery)(void);

/* 
 * set adc clock to 12MHz/div. A/D works at freq between 500KHz to 6MHz.
 */
static void sadc_init_clock(int div)
{
	if (div < 2) div = 2;
	if (div > 23) div = 23;
#if defined(CONFIG_SOC_JZ4740)	
	REG_SADC_CFG &= ~SADC_CFG_CLKDIV_MASK;
	REG_SADC_CFG |= (div - 1) << SADC_CFG_CLKDIV_BIT;
#endif
#if defined(CONFIG_SOC_JZ4750) || defined(CONFIG_SOC_JZ4750D)
	REG_SADC_ADCLK &= ~SADC_ADCLK_CLKDIV_MASK;
	REG_SADC_ADCLK |= (div - 1) << SADC_ADCLK_CLKDIV_BIT;
	REG_SADC_ADCLK &= ~SADC_ADCLK_CLKDIV_BIT;
	REG_SADC_ADCLK |= 39 << SADC_ADCLK_CLKDIV_10_BIT;  /* if div ==3,here is 39 */
#endif
}

void start_sadcin(void)
{
	REG_SADC_CTRL &= ~SADC_CTRL_SRDYM; /* enable interrupt */
	REG_SADC_ENA |= SADC_ENA_SADCINEN;
}

void start_pbat_adc(void)
{
	REG_SADC_CFG |= SADC_CFG_PBAT_HIGH ;   /* full baterry voltage >= 2.5V */
//  	REG_SADC_CFG |= SADC_CFG_PBAT_LOW;    /* full baterry voltage < 2.5V */

  	REG_SADC_ENA |= SADC_ENA_PBATEN;      /* Enable pbat adc */
}

void start_ts_adc(void)
{
	REG_SADC_SAMETIME = 10;  /* about 0.1 ms,you can change it */
	REG_SADC_WAITTIME = 2;  /* about 0.02 ms,you can change it */

	REG_SADC_CFG &= ~(SADC_CFG_TS_DMA | SADC_CFG_XYZ_MASK | SADC_CFG_SNUM_MASK);
	REG_SADC_CFG |= (SADC_CFG_EXIN | SADC_CFG_XYZ | SADC_CFG_SNUM_3);
	REG_SADC_CTRL |= (SADC_CTRL_TSRDYM|SADC_CTRL_PBATRDYM|SADC_CTRL_PENUM |SADC_CTRL_SRDYM);
	REG_SADC_CTRL &= ~SADC_CTRL_PENDM;
	REG_SADC_ENA |= SADC_ENA_TSEN;
}

static int  jz4740_adc_read(struct jz_ts_t *ts)
{
	struct datasource *ds = &data_s;
	u32 xybuf,z;

	if (!(REG_SADC_STATE & SADC_STATE_TSRDY)) {
		/* sleep */
		REG_SADC_CTRL &= ~SADC_CTRL_TSRDYM;
		ts->sleeping = 1;
		sleep_on(&sadc_wait_queue);
	}
	ts->sleeping = 0;

	xybuf = REG_SADC_TSDAT;
	ds->xbuf = (xybuf>>16) & 0x0fff;
	ds->ybuf = (xybuf)& 0x0fff;
	z = REG_SADC_TSDAT;
	ds->zbuf = z& 0x0fff;
  	REG_SADC_STATE &= ~SADC_STATE_TSRDY;
	return 0;
}

/*------------------------------------------------------------
 * Read the battery voltage
 */
unsigned int jz4740_read_battery(void)
{
	unsigned int v;
	unsigned int timeout = 0x3ff;
	u16 pbat;

	if(!(REG_SADC_STATE & SADC_STATE_PBATRDY) ==1)
		start_pbat_adc();

	while(!(REG_SADC_STATE & SADC_STATE_PBATRDY) && --timeout)
		;

	pbat = REG_SADC_BATDAT;
	v = pbat & 0x0fff;
	REG_SADC_STATE = SADC_STATE_PBATRDY;
	return v;
}

/*------------------ Calibrate samples -------------------*/

#define DIFF(a,b) (((a)>(b))?((a)-(b)):((b)-(a)))
#define MIN(a,b) (((a)<(b))?(a):(b))

#if 0
#define XM 36 /* XM and YM may be changed for your screen */
#define YM 20
static int calibrate_samples(void *xbuf, void *ybuf, void *pbuf, int count)
{
	unsigned long usd0,usd1,usd2;
	int xMaxError = XM,yMaxError = YM;
	int x_valid = 0,y_valid = 0,valid = 0;
	unsigned long x_cal = 0, y_cal = 0, p_cal = 0;
	unsigned long *xp = (unsigned long *)xbuf;
	unsigned long *yp = (unsigned long *)ybuf;
	unsigned long *pp = (unsigned long *)pbuf;

	usd0 = (xp[0] > xp[1]) ? (xp[0] - xp[1]) : (xp[1] - xp[0]);
	usd1 = (xp[1] > xp[2]) ? (xp[1] - xp[2]) : (xp[2] - xp[1]);
	usd2 = (xp[2] > xp[0]) ? (xp[2] - xp[0]) : (xp[0] - xp[2]);

	if ( usd0 < usd1)
		x_cal = xp[0] + ((usd2 < usd0) ? xp[2] : xp[1]);
	else
		x_cal= xp[2] + ((usd2 < usd1) ? xp[0] : xp[1]);
	x_cal >>= 1;
	
	if ( (usd0 < xMaxError) && (usd1 < xMaxError) && (usd2 < xMaxError) ) 
		x_valid = 1;
	
	usd0 = (yp[0] > yp[1]) ? (yp[0] - yp[1]) : (yp[1] - yp[0]);
	usd1 = (yp[1] > yp[2]) ? (yp[1] - yp[2]) : (yp[2] - yp[1]);
	usd2 = (yp[2] > yp[0]) ? (yp[2] - yp[0]) : (yp[0] - yp[2]);
	
	if ( usd0 < usd1)
		y_cal = yp[0] + ((usd2 < usd0) ? yp[2] : yp[1]);
	else
		y_cal = yp[2] + ((usd2 < usd1) ? yp[0] : yp[1]);
	
	y_cal >>= 1;
	
	if ( (usd0 < yMaxError) && (usd1 < yMaxError) && (usd2 < yMaxError) ) 
		y_valid = 1;
	
	if( x_valid && y_valid)
		valid = 1;

	usd0 = (pp[0] > pp[1]) ? (pp[0] - pp[1]) : (pp[1] - pp[0]);
	usd1 = (pp[1] > pp[2]) ? (pp[1] - pp[2]) : (pp[2] - pp[1]);
	usd2 = (pp[2] > pp[0]) ? (pp[2] - pp[0]) : (pp[0] - pp[2]);

	if ( usd0 < usd1)
		p_cal = pp[0] + ((usd2 < usd0) ? pp[2] : pp[1]);
	else
		p_cal= pp[2] + ((usd2 < usd1) ? pp[0] : pp[1]);

	p_cal >>= 1;

	if (first_time) {
		first_time = 0;
		last_x = x_cal;
		last_y = y_cal;
		last_p = p_cal;
	}
	else{
		if ((DIFF(x_cal, last_x) > 50) ||
		    (DIFF(y_cal, last_y) > 50))
			valid = 0;
		else
			valid = 1;
	}
	*xp = last_x = x_cal;
	*yp = last_y = y_cal;
	*pp = last_p = p_cal;

	return valid;
}
#endif

static int calibrate_samples(void *xbuf, void *ybuf, void *pbuf, int count)
{
	unsigned long *xp = (unsigned long *)xbuf;
	unsigned long *yp = (unsigned long *)ybuf;
	unsigned long *pp = (unsigned long *)pbuf;
	unsigned long x_cal = 0, y_cal = 0, p_cal = 0;
	int i;
	int valid = 1;

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
	}
	else {
		if ((DIFF(x_cal, last_x) > 50) ||
		    (DIFF(y_cal, last_y) > 50))
			valid = 0;
		else
			valid = 1;
	}

	*xp = last_x = x_cal;
	*yp = last_y = y_cal;
	*pp = last_p = p_cal;

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

/*
 * File operations
 */
static int sadc_open(struct inode *inode, struct file *filp);
static int sadc_release(struct inode *inode, struct file *filp);
static ssize_t sadc_read(struct file *filp, char *buf, size_t size, loff_t *l);
static ssize_t sadc_write(struct file *filp, const char *buf, size_t size, loff_t *l);
static int sadc_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

static struct file_operations sadc_fops = 
{
	open:		sadc_open,
	release:	sadc_release,
	read:		sadc_read,
	write:		sadc_write,
	ioctl:		sadc_ioctl
};

static int sadc_open(struct inode *inode, struct file *filp)
{
 	try_module_get(THIS_MODULE);
	return 0;
}

static int sadc_release(struct inode *inode, struct file *filp)
{
 	module_put(THIS_MODULE);
	return 0;
}

static ssize_t sadc_read(struct file *filp, char *buf, size_t size, loff_t *l)
{
	return size;
}

static ssize_t sadc_write(struct file *filp, const char *buf, size_t size, loff_t *l)
{
	return size;
}

static int sadc_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	default:
		printk("Not supported command: 0x%x\n", cmd);
		return -EINVAL;
		break;
	}
	return 0;
}

/*------------------ Common routines -------------------*/

void ts_enable_irq(void)
{
	REG_SADC_CTRL &= ~SADC_CTRL_PENDM;
}

void ts_disable_irq(void)
{
	REG_SADC_CTRL |=  (SADC_CTRL_PENDM | SADC_CTRL_PENUM);
}

void ts_free_irq(struct jz_ts_t *ts)
{
	free_irq(ts->pendown_irq, ts);
}

void ts_data_ready(void)
{
	REG_SADC_CTRL |= SADC_CTRL_TSRDYM;
	wake_up(&sadc_wait_queue);
}

/*
 * Interrupt handler
 */
void ts_irq_callback(void)
{
	u32 state;

	state = REG_SADC_STATE;
	if (!(REG_SADC_CTRL&SADC_CTRL_PENDM)&&(REG_SADC_STATE & SADC_STATE_PEND)) {
		REG_SADC_STATE = SADC_STATE_PEND;
		REG_SADC_STATE = SADC_STATE_PENU;
		REG_SADC_CTRL |= SADC_CTRL_PENDM;
		REG_SADC_CTRL &= ~SADC_CTRL_PENUM;
		p = 1;
	} 

	if (!(REG_SADC_CTRL&SADC_CTRL_PENUM)&&(REG_SADC_STATE & SADC_STATE_PENU)) {		
		REG_SADC_STATE = SADC_STATE_PENU;
		REG_SADC_CTRL |= SADC_CTRL_PENUM;
		REG_SADC_CTRL &= ~SADC_CTRL_PENDM;
		p = 0;
	}

	first_time = 1; // first time to acquire sample
}

int PenIsDown(void)
{
	return p;
}

int ts_request_irq(u32 *irq, 
		   irqreturn_t (*handler)(int, void *),
		   const char *devname,
		   void *dev_id)
{
	int ret;

	/* return the irq number */
	*irq = IRQ_SADC;
	ts_disable_irq();
	/* interrupt mode */
	ret = request_irq(IRQ_SADC, handler, IRQF_DISABLED, 
			  devname, dev_id); 
	if(ret)
		printk("failed irq \n");

	start_ts_adc();
	return ret;
}

/*
 * Acquire Raw pen coodinate data and compute touch screen
 * pressure resistance. Hold spinlock when calling.
 */
int AcquireEvent(struct jz_ts_t *ts, struct ts_event *event)
{
	unsigned int x_raw[8], y_raw[8], p_raw[8];
	int valid, i;
	unsigned int avl_x, avl_y, diff_x, diff_y;
	struct datasource *ds = &data_s;
	avl_x = avl_y = 0;

	for (i = 0; i < samples; i++) {
		if (jz4740_adc_read(ts)) {
			return 0;
		}

		x_raw[i] = ds->ybuf;
		y_raw[i] = ds->xbuf;
		p_raw[i] = ds->zbuf;
		avl_x += x_raw[i];
		avl_y += y_raw[i];
#if 0
		printk("x_raw=%x y_raw=%x z_raw=%x\n",x_raw[i],y_raw[i],p_raw[i]);
#endif
	}

	avl_x /= samples;
	avl_y /= samples;
#define MAX_DELTA 20
	valid = 1;

	for (i = 1; i < samples; i++)
	{
		if ((100 * DIFF(x_raw[i],x_raw[i-1])/MIN(x_raw[i],x_raw[i-1])) > MAX_DELTA) {
			valid = 0;
			break;
		}

		if ((100 * DIFF(y_raw[i],y_raw[i-1])/MIN(y_raw[i],y_raw[i-1])) > MAX_DELTA) {
			valid = 0;
			break;
		}

		if ((100 * DIFF(p_raw[i],p_raw[i-1])/MIN(p_raw[i],p_raw[i-1])) > MAX_DELTA) {
			valid = 0;
			break;
		}
	}

	if (valid) {
		if (ts->first_read) {
			ts->first_read = 0;
			old_x = avl_x;
			old_y = avl_y;
		}
		diff_x = DIFF(old_x, avl_x);
		diff_y = DIFF(old_y, avl_y);
		if (diff_x < 100 && diff_y < 100) {
			old_x = avl_x;
			old_y = avl_y;
		} else
			valid = 0;
	}
	if (valid) {
		valid = calibrate_samples(x_raw, y_raw, p_raw, samples);
	}

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
static int __init sadc_init(void)
{
	struct sadc_device *dev;
	int ret;

	/* allocate device */
	dev = kmalloc(sizeof(struct sadc_device), GFP_KERNEL);
	if (!dev) return -ENOMEM;

	sadc_dev = dev;
	ret = jz_register_chrdev(SADC_MINOR, SADC_NAME, &sadc_fops, dev);
	if (ret < 0) {
		kfree(dev);
		return ret;
	}

	codec_read_battery = jz4740_read_battery;
	sadc_init_clock(3);

	printk(JZ_SOC_NAME": SAR-ADC driver registered.\n");
	return 0;
}

static void __exit sadc_exit(void)
{
	struct sadc_device *dev = sadc_dev;

	free_irq(IRQ_SADC, dev);
	jz_unregister_chrdev(SADC_MINOR, SADC_NAME);
	kfree(dev);
}

module_init(sadc_init);
module_exit(sadc_exit);
