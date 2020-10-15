/*
 * wm9712.c
 *
 * Touch screen driver interface to the Wolfson WM9712 codec.
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
#include "wm9712.h"

#define POLL_TIMES	10

static int samples = 1;
static int inited = 0, started = 0;

extern struct ac97_codec * find_ac97_codec(void);
extern int PenIsDown(void);


static inline void wm9712_reg_write(unsigned int reg, unsigned int val)
{
	struct ac97_codec *codec = find_ac97_codec();
	if (!codec)
		return;
	codec->codec_write(codec, reg, val);
}

static inline unsigned int wm9712_reg_read(unsigned int reg)
{
	struct ac97_codec *codec = find_ac97_codec();
	if (!codec)
		return 0;
	return codec->codec_read(codec, reg);
}

static unsigned int wm9712_adc_read(int adc_channel)
{
	unsigned int val;

	if (!PenIsDown())
		return 0;

	val = wm9712_reg_read(DIGI_REG1);
	wm9712_reg_write(DIGI_REG1, val|adc_channel|DIGI_REG1_POLL);

	for (;;) {
		if (wm9712_reg_read(0x54) & (1 << 12)) {
			val = wm9712_reg_read(DIGI_READBACK);
			break;
		}
	}

	/* stop the measure */
	wm9712_reg_write(DIGI_REG1, 0);

	return (val & 0x0fff);
}

static struct timer_list pndn_timer;
static void (*irq_handler)(int, void *, struct pt_regs *) = NULL;

void ts_irq_callback(void)
{
#ifdef TS_IRQ
	__gpio_ack_irq(TS_IRQ);
#else
#endif
}

void ts_enable_irq(void)
{
	if (!inited)
		return;
#ifdef TS_IRQ
	enable_irq(TS_IRQ);
#else
	pndn_timer.expires = jiffies + HZ/POLL_TIMES;
	add_timer(&pndn_timer);
#endif
}

void ts_disable_irq(void)
{
	if (!inited)
		return;
#ifdef TS_IRQ
	disable_irq(TS_IRQ);
#endif
}

#ifndef TS_IRQ
static void pndn_detect(unsigned long data)
{
	if (PenIsDown()) {
		if (!started)
			return;
		if (irq_handler)
			irq_handler(NULL, data, NULL);
	} else {
		pndn_timer.expires = jiffies + HZ/POLL_TIMES;
		add_timer(&pndn_timer);
	}
}
#endif

void ts_free_irq(struct jz_ts_t *ts)
{
#ifdef TS_IRQ
	free_irq(ts->pendown_irq, ts);
#else
	started = 0;
	del_timer_sync(&pndn_timer);
#endif
}

int ts_request_irq(u32 *irq, 
		   void (*handler)(int, void *, struct pt_regs *),
		   const char *devname,
		   void *dev_id)
{
	/* 4wire, Ip=400uA, Rpu=64Kohm/64, wake-up on pendown without
	 * reset, meassure on pen down. Do not use wait mode.
	 */
	started = 1;
	if (!inited) {
		wm9712_reg_write(DIGI_REG2,
				 DIGI_REG2_WIRE_4 |
				 DIGI_REG2_PIL_200uA |
				 (31 << DIGI_REG2_RPU_BIT) |
				 DIGI_REG2_PRP_ALLON |
				 DIGI_REG2_RPR_NWOR);
		/* Polling mode and no measurement */
		wm9712_reg_write(DIGI_REG1, 0);
	}

#ifdef TS_IRQ
	/* Generate irq request on PENDOWN pin, pendown cause the level high */
	wm9712_reg_write(0x56, wm9712_reg_read(0x56) & ~(1 << 3));
	wm9712_reg_write(0x4c, wm9712_reg_read(0x4c) & ~(1 << 3));

	*irq = TS_IRQ;
	return request_irq(TS_IRQ, handler, SA_INTERRUPT, devname, dev_id);
#else
	if (!inited) {
		irq_handler = handler;
		init_timer(&pndn_timer);
		pndn_timer.function = pndn_detect;
		pndn_timer.data = (unsigned long)dev_id;
		pndn_timer.expires = jiffies + HZ/POLL_TIMES;
		add_timer(&pndn_timer);
		inited = 1;
	} else {
		pndn_timer.expires = jiffies + HZ/POLL_TIMES;
		add_timer(&pndn_timer);
	}
	return 0;
#endif
}

int PenIsDown(void)
{
	if (wm9712_reg_read(DIGI_READBACK) & DIGI_READBACK_PNDN)
		return 1;
	return 0;
}

#if defined(CONFIG_MIPS_JZ4730_GPS)
#define adj_data(r1, r2, r3, s)			\
do {						\
	if (r1 < 0x90)				\
		r1 = 0x90;			\
	if (r2 < 0xed)				\
		r2 = 0xed;			\
	r1 = ((r1 - 0x90) * 240) / 3354;	\
	r2 = ((r2 - 0xed) * 320) / 3671;	\
	if (r1 > 239)				\
		r1 = 239;			\
	if (r2 > 319)				\
		r2 = 319;			\
						\
	*s = r2;				\
	*(s+1) = 239 - r1;			\
	*(s+2) = z_raw;				\
} while (0)
#endif

#ifndef adj_data
#define adj_data(r1, r2, r3, s)
#endif

static int read_adc(unsigned int *sdata)
{
	unsigned long x_raw=0, y_raw=0, z_raw=0, t, fail = 0;
	int i;

	for (i=0; i<samples; i++) {
		t = wm9712_adc_read(ADCSEL_XPOS); 
		if (t == 0)
			fail = 1;
		x_raw += t; 
		t = wm9712_adc_read(ADCSEL_YPOS);
		if (t == 0)
			fail = 1;
		y_raw += t;
		t = wm9712_adc_read(ADCSEL_PRESSURE);
		if (t == 0)
			fail = 1;
		z_raw += t;
	}
	
	if (fail)
		return 0;

	if (samples > 1) {
		x_raw = (x_raw + (samples>>1)) / samples;
		y_raw = (y_raw + (samples>>1)) / samples;
		z_raw = (z_raw + (samples>>1)) / samples;
	}

	adj_data (x_raw, y_raw, z_raw, sdata);

	return 1;
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
 * Acquire Raw pen coodinate data and compute touch screen
 * pressure resistance. Hold spinlock when calling.
 */
int AcquireEvent(struct jz_ts_t *ts, struct ts_event *event)
{
	unsigned int s[3];
	unsigned int x_scr, y_scr;
	if (!read_adc(s))
		return 0;
	if(ts->filter) {
			x_scr = transform_to_screen_x(ts, s[0]);
			y_scr = transform_to_screen_y(ts, s[1]);

			if (ts->prints)
				printk("x_raw=%d y_raw=%d x_transform=%d y_transform=%d\n", s[0], s[1], x_scr, y_scr);		}
	else {
		x_scr = s[0];
		y_scr = s[1];

		if (ts->prints)
			printk("x_raw=%d y_raw=%d \n", s[0], s[1]);
		}
	event->x = x_scr;
	event->y = y_scr;
	event->pressure = (u16)s[2];
	event->status = PENDOWN;
	return 1;
#if 0
	do_gettimeofday(&event->stamp);
#endif
}

int __init wm9712_init(void)
{
	return 0;
}

void wm9712_cleanup(void)
{
}

module_init(wm9712_init);
module_exit(wm9712_cleanup);

