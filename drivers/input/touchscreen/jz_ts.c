/*
 * JZ Touch Screen Driver
 *
 * Copyright (c) 2005 - 2009  Ingenic Semiconductor Inc.
 *
 * Author: Jason <xwang@ingenic.cn> 20090219
 *         Regen <lhhuang@ingenic.cn> 20090324 add adkey
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#include <asm/irq.h>
#include <asm/gpio.h>
#include <asm/jzsoc.h>

#if 1
#define DBG printk("%s(%d) \n",__FUNCTION__,__LINE__);
#else
#define DBG
#endif

#define TS_NAME "jz-ts"

#define KEY_SCAN_INTERVAL   5
#define TS_SCAN_INTERVAL	0

/* from qwerty.kl of android */
#define  DPAD_CENTER            232
#define  DPAD_DOWN              108
#define  DPAD_UP                103
#define  DPAD_LEFT              105
#define  DPAD_RIGHT             106

/* TS event status */
#define PENUP			0x00
#define PENDOWN			0x01

/* Sample times in one sample process	*/
//#define SAMPLE_TIMES		3

#define SAMPLE_TIMES		5
#define DROP_SAMPLE_TIMES   1  /* min drop 1 sample */
#define CAL_SAMPLE_TIMES (SAMPLE_TIMES - DROP_SAMPLE_TIMES)
#define VIRTUAL_SAMPLE      3 /* min >= 2 */ 
/* Min pressure value. If less than it, filt the point.
 * Mask it if it is not useful for you
 */
//#define MIN_PRESSURE		0x100

/* Max delta x distance between current point and last point.	*/
#define MAX_DELTA_X_OF_2_POINTS	200
/* Max delta x distance between current point and last point.	*/
#define MAX_DELTA_Y_OF_2_POINTS	120

/* Max delta between points in one sample process
 * Verify method :
 * 	(diff value / min value) * 100 <= MAX_DELTA_OF_SAMPLING
 */
#define MAX_DELTA_OF_SAMPLING	20


#define TS_ABS(x)       ((x) > 0 ? (x): -(x))
#define DIFF(a,b)		(((a)>(b))?((a)-(b)):((b)-(a)))
#define MIN(a,b)		(((a)<(b))?(a):(b))


/************************************************************************/
/*	SAR ADC OPS							*/
/************************************************************************/

typedef struct datasource {
    u16 xbuf;
    u16 ybuf;
	u16 zbuf;
    u16 reserve;
}datasource_t;
struct ts_event {
	u16 status;
	u16 x;
	u16 y;
	u16 pressure;
	u16 pad;
};
#define TOUCH_TYPE 1  
#define BAT_TYPE   2
#define SADC_TYPE  4


//sadc touch fifo size 2 * 32bit
#define FIFO_MAX_SIZE 2

/*
 * TS deriver
 */
struct jz_ts_t {
    int touch_cal_count;

    unsigned int ts_fifo[FIFO_MAX_SIZE][CAL_SAMPLE_TIMES];
    datasource_t data_s;
    struct ts_event event;
    int event_valid;
    
    
    int cal_type; /* current calibrate type */
  int oldbat_value;
	//struct timer_list acq_timer;	// Timer for triggering acquisitions
#ifdef CONFIG_JZ_ADKEY
	struct timer_list key_timer;	// for adkey
	int active_low;		// for adkey's interrupt pin
#endif
	wait_queue_head_t wait;		// read wait queue
	spinlock_t lock;

	/* Following 4 members use to pass arguments from u-boot to tell us the ts data.
	 * But in Android we do not use them.
	 */
/*
	int minx, miny, maxx, maxy;
*/
	int first_read;

	char	phys[32];
	struct input_dev *input_dev;
};

static struct jz_ts_t *jz_ts;

/*
 * TS Event type
 */

#ifdef CONFIG_JZ_ADKEY
struct ad_keys_button {
	int code;		/* input event code */
	int val;                /* the ad value of the key */
	int fuzz;               /* the error(+-fuzz) allowed of the ad value of the key */
};
static struct ad_keys_button ad_buttons[] = {
	{
		.code = DPAD_LEFT,
		.val = DPAD_LEFT_LEVEL,
		.fuzz = 40,
	},
	{
		.code = DPAD_DOWN,
		.val = DPAD_DOWN_LEVEL,
		.fuzz = 40,
	},
	{
		.code = DPAD_UP,
		.val = DPAD_UP_LEVEL,
		.fuzz = 40,
	},
	{
		.code = DPAD_CENTER,
		.val = DPAD_CENTER_LEVEL,
		.fuzz = 40,
	},
	{
		.code = DPAD_RIGHT,
		.val = DPAD_RIGHT_LEVEL,
		.fuzz = 40,
	},
};
#define KEY_NUM (sizeof(ad_buttons) / sizeof(struct ad_keys_button))
#endif


//static unsigned int p;

static DECLARE_WAIT_QUEUE_HEAD (sadc_wait_queue);

extern unsigned int (*codec_read_battery)(void);
#if 0
static void reg_debug(void)
{
        printk("\t####CTRL####################################################\n");
        printk("\tPEND %s,   ", REG_SADC_CTRL & SADC_CTRL_PENDM ? "masked" : "enabled");
        printk("PENU %s,   ", REG_SADC_CTRL & SADC_CTRL_PENUM ? "masked" : "enabled");
        printk("TSRDY %s\n", REG_SADC_CTRL & SADC_CTRL_TSRDYM ? "masked" : "enabled");
        printk("\t----STATE---------------------------------------------------\n");
        printk("\tIRQ actived: %s,   %s,   %s\n",
               REG_SADC_STATE & SADC_STATE_PEND ? "pen down" : "        ",
               REG_SADC_STATE & SADC_STATE_PENU ? "pen up  " : "        ",
               REG_SADC_STATE & SADC_STATE_TSRDY ? "sample  " : "        ");
        printk("\t############################################################\n");
}
#endif
/* 
 * set adc clock to 24MHz/div. A/D works at freq between 500KHz to 8MHz.
 */
static void sadc_init_clock(int div)
{
   
    REG_CPM_CLKGR &= ~CPM_CLKGR_SADC;
    
    if (div < 2) div = 2;
	if (div > 23) div = 23;
#if defined(CONFIG_SOC_JZ4740)	
	REG_SADC_CFG &= ~SADC_CFG_CLKDIV_MASK;
	REG_SADC_CFG |= (div - 1) << SADC_CFG_CLKDIV_BIT;
#endif
#if defined(CONFIG_SOC_JZ4750) || defined(CONFIG_SOC_JZ4750D)
    div = 48;
    
	REG_SADC_ADCLK &= ~SADC_ADCLK_CLKDIV_MASK;
	REG_SADC_ADCLK |= (div - 1) << SADC_ADCLK_CLKDIV_BIT;
	REG_SADC_ADCLK &= ~SADC_ADCLK_CLKDIV_BIT;
//	REG_SADC_ADCLK |= 39 << SADC_ADCLK_CLKDIV_10_BIT;  /* if div ==3,here is 39 */

    REG_SADC_ADCLK |= 4 << SADC_ADCLK_CLKDIV_10_BIT;  /* if div ==48,here is 4 */
    
    // if(SADC_STATE_SLEEPND & REG_SADC_STATE)
    if( REG_SADC_STATE)
    {
        REG_SADC_ENA |= SADC_ENA_EXIT_SLP;
        while(REG_SADC_ENA & SADC_ENA_EXIT_SLP);
	//  REG_SADC_STATE = SADC_STATE_SLEEPND;
    }
    
 #endif
}

static inline void sadc_start_sadcin(void)
{
	REG_SADC_ENA |= SADC_ENA_SADCINEN;
}

static inline void sadc_start_pbat(void)
{
  	REG_SADC_ENA |= SADC_ENA_PBATEN;      /* Enable pbat adc */
}

static inline void ts_enable_pendown_irq(void)
{
	REG_SADC_CTRL &= ~SADC_CTRL_PENDM;
}

static inline void ts_enable_penup_irq(void)
{
	REG_SADC_CTRL &= ~SADC_CTRL_PENUM;
}

static inline void ts_disable_pendown_irq(void)
{
	REG_SADC_CTRL |= SADC_CTRL_PENDM;
}

static inline void ts_disable_penup_irq(void)
{
	REG_SADC_CTRL |= SADC_CTRL_PENUM;
}

static inline void sadc_enable_ts(void)
{
	REG_SADC_ENA |= SADC_ENA_TSEN;
}

static inline void sadc_disable_ts(void)
{
	REG_SADC_ENA &= ~SADC_ENA_TSEN;
}

static inline void sadc_start_ts(void)
{
	REG_SADC_SAMETIME = 10;	    /* about 0.02 ms,you can change it */
	REG_SADC_WAITTIME = 2;	/* about 3.33 ms,you can change it */
	
	REG_SADC_CFG &= ~(SADC_CFG_TS_DMA | SADC_CFG_XYZ_MASK | SADC_CFG_SNUM_MASK |SADC_CFG_EXIN | SADC_CFG_CLKOUT_NUM_MASK);
	REG_SADC_CFG |= (SADC_CFG_XYZ1Z2 | SADC_CFG_SNUM(SAMPLE_TIMES) | SADC_CFG_SPZZ | SADC_CFG_DNUM(VIRTUAL_SAMPLE));
	
	if (CFG_PBAT_DIV == 1)
		REG_SADC_CFG |= SADC_CFG_PBAT_HIGH;   /* full baterry voltage >= 2.5V */
	else
		REG_SADC_CFG |= SADC_CFG_PBAT_LOW;    /* full baterry voltage < 2.5V */
	
//  REG_SADC_CTRL = (SADC_STATE_SLEEPND | SADC_CTRL_TSRDYM | SADC_CTRL_PBATRDYM | SADC_CTRL_PENUM |SADC_CTRL_SRDYM);
	REG_SADC_CTRL = ( SADC_CTRL_TSRDYM | SADC_CTRL_PBATRDYM | SADC_CTRL_PENUM |SADC_CTRL_SRDYM);
	REG_SADC_STATE = REG_SADC_STATE;
	
	REG_SADC_ENA |= SADC_ENA_TSEN;
}

/** 
 * Read the battery voltage
 */
unsigned int jz_read_battery(void)
{
	unsigned int v;
	unsigned int timeout = 0x3fff;
	u16 pbat;
	spin_lock_irq(&jz_ts->lock);
	if(jz_ts->oldbat_value == 0xffffffff)
	{
	  //printk("==========================\n");
	  sadc_start_pbat();
	  while(!(REG_SADC_STATE & SADC_STATE_PBATRDY) && --timeout)    ;
	  pbat = REG_SADC_BATDAT;
	  v = pbat & 0x0fff;
	  jz_ts->oldbat_value = v;
	  REG_SADC_STATE = SADC_STATE_PBATRDY; // 
	  REG_SADC_CTRL &= ~SADC_CTRL_PBATRDYM; 

	  //printk("==========================\n");
	}
	if(!(jz_ts->cal_type & BAT_TYPE)) 
	{  
	  jz_ts->cal_type |= BAT_TYPE;
	  sadc_start_pbat();
	  //printk("start next pbat\n");
	}
        
	v = jz_ts->oldbat_value;
	//printk("pbat = %d\n",v);
	spin_unlock_irq(&jz_ts->lock);
	return v;
}

static inline void ts_data_ready(void)
{
	REG_SADC_CTRL |= SADC_CTRL_TSRDYM;
}

#ifdef CONFIG_JZ_ADKEY
/** 
 * Read the battery voltage
 */
static unsigned int read_sadcin(void)
{
	unsigned int v;
	unsigned int timeout = 0x3ff;
	u16 val;
	jz_ts->cal_type |= SADC_TYPE;
	if(!(REG_SADC_STATE & SADC_STATE_SRDY))
		sadc_start_sadcin();

	while(!(REG_SADC_STATE & SADC_STATE_SRDY) && --timeout)
		;

	val = REG_SADC_SADDAT;
	v = val & 0x0fff;
	REG_SADC_STATE = SADC_STATE_SRDY;    
	jz_ts->cal_type &= ~SADC_TYPE;
	return v;
}

static unsigned int key_scan(int ad_val)
{
	 int i;

	 for(i = 0; i<KEY_NUM; i++) {
		if((ad_buttons[i].val + ad_buttons[i].fuzz >= ad_val) && 
		   (ad_val >=ad_buttons[i].val - ad_buttons[i].fuzz)) {
			return ad_buttons[i].code;
		}
	 }
	 return -1;
}

static void key_timer_callback(unsigned long data)
{
	struct jz_ts_t *ts = (struct jz_ts_t *)data;
	int state;
	int active_low = ts->active_low;
	int ad_val, code;
	static int old_code;

	state = __gpio_get_pin(GPIO_ADKEY_INT);
	ad_val = read_sadcin();

        if (active_low) {
		if (state == 0) {
			/* press down */
			code = key_scan(ad_val);
			old_code = code;
			input_report_key(ts->input_dev, code, 1);
			//emily input_sync(ts->input_dev);
			mod_timer(&ts->key_timer, jiffies + KEY_SCAN_INTERVAL);
		} else {
			/* up */
			input_report_key(ts->input_dev, old_code, 0);
			//emily input_sync(ts->input_dev);
			udelay(1000);
			__gpio_as_irq_fall_edge(GPIO_ADKEY_INT);
		}
	} else {
		if (state == 1) {
			/* press down */
			code = key_scan(ad_val);
			old_code = code;
			input_report_key(ts->input_dev, code, 1);
			//emily input_sync(ts->input_dev);
			mod_timer(&ts->key_timer, jiffies + KEY_SCAN_INTERVAL);
		} else {
			/* up */
			input_report_key(ts->input_dev, old_code, 0);
			//emily input_sync(ts->input_dev);
			udelay(1000);
			__gpio_as_irq_rise_edge(GPIO_ADKEY_INT);
		}
	}
}

static irqreturn_t key_interrupt(int irq, void * dev_id)
{
	struct jz_ts_t *ts = dev_id;
	DBG;
	__gpio_ack_irq(GPIO_ADKEY_INT);
	__gpio_as_input(GPIO_ADKEY_INT);
	sadc_start_sadcin();
	mod_timer(&ts->key_timer, jiffies + KEY_SCAN_INTERVAL);
	return IRQ_HANDLED;
}
#endif

/************************************************************************/
/*	Touch Screen module						*/
/************************************************************************/

#define TSMAXX		3920
#define TSMAXY		3700
#define TSMAXZ		(1024) /* measure data */

#define TSMINX		150
#define TSMINY		270
#define TSMINZ		0


#define SCREEN_MAXX	1023
#define SCREEN_MAXY	1023
#define PRESS_MAXZ      256

static unsigned long transform_to_screen_x(struct jz_ts_t *ts, unsigned long x )
{
/* Now we don't need u-boot to tell us the ts data.	*/
/*
	if (ts->minx)
	{
		if (x < ts->minx) x = ts->minx;
		if (x > ts->maxx) x = ts->maxx;

		return (x - ts->minx) * SCREEN_MAXX / (ts->maxx - ts->minx);
	}
	else
	{
*/
	if (x < TSMINX) x = TSMINX;
	if (x > TSMAXX) x = TSMAXX;

	return (x - TSMINX) * SCREEN_MAXX / (TSMAXX - TSMINX);
/*
	}
*/
}

static unsigned long transform_to_screen_y(struct jz_ts_t *ts, unsigned long y)
{
/* Now we don't need u-boot to tell us the ts data.	*/
/*
	if (ts->miny)
	{
		if (y < ts->miny) y = ts->miny;
		if (y > ts->maxy) y = ts->maxy;

		return (ts->maxy - y) * SCREEN_MAXY / (ts->maxy - ts->miny);
	}
	else
	{
*/
	if (y < TSMINY) y = TSMINY;
	if (y > TSMAXY) y = TSMAXY;

	return (TSMAXY - y) * SCREEN_MAXY / (TSMAXY - TSMINY);
/*
	}
*/
}
static unsigned long transform_to_screen_z(struct jz_ts_t *ts, unsigned long z){
    if(z < TSMINZ) z = TSMINZ;
    if (z > TSMAXY) z = TSMAXY;
    return (TSMAXZ - z) * PRESS_MAXZ / (TSMAXZ - TSMINZ);
}
    /* R plane calibrate,please look up spec 11th page*/
    
#define Yr_PLANE  272
#define Xr_PLANE  480

#define Touch_Formula_One(z1,z2,ref,r) ({	\
	int z;					\
	if((z1) > 0){				\
	  z = ((ref) * (z2)) / (z1);		\
	  if((z2) > (z1)) z = (z * r - (ref) * r) / (4096);	\
	  else z = 0;				\
	}else					\
	  z = 4095;				\
	z;					\
      })
 

static int ts_data_filter(struct jz_ts_t *ts){
  int i,xt = 0,yt = 0,zt1 = 0,zt2 = 0,zt3 = 0,zt4 = 0,t1_count = 0,t2_count = 0,z;
  
  datasource_t *ds = &ts->data_s;
  int t,xmin = 0x0fff,ymin = 0x0fff,xmax = 0,ymax = 0;//,z1min = 0xfff,z1max = 0,z2min = 0xfff,z2max = 0;
    
    /* fifo high 16 bit = y,fifo low 16 bit = x */
    
    for(i = 0;i < CAL_SAMPLE_TIMES;i++){

        t = (ts->ts_fifo[0][i] & 0x0fff);
#if (CAL_SAMPLE_TIMES >= 3)
        if(t > xmax) xmax = t;
        if(t < xmin) xmin = t;
#endif        
        xt += t;
        t = (ts->ts_fifo[0][i] >> 16) & 0x0fff;
#if (CAL_SAMPLE_TIMES >= 3)   
        if(t > ymax) ymax = t;
        if(t < ymin) ymin = t;
#endif
        
        yt += t;
	if(ts->ts_fifo[1][i] & 0x8000)
	{
	  t = (ts->ts_fifo[1][i] & 0x0fff);
	  zt1 += t;

	  t = (ts->ts_fifo[1][i] >> 16) & 0x0fff;
	  zt2 += t;
	  
	  t1_count++;
        }else
	{
	  t = (ts->ts_fifo[1][i] & 0x0fff);
	  zt3 += t;

	  t = (ts->ts_fifo[1][i] >> 16) & 0x0fff;
	  zt4 += t;
	  
	  t2_count++;
	}
    }
#if (CAL_SAMPLE_TIMES >= 3)   
        xt = xt - xmin - xmax;
        yt = yt - ymin - ymax;
#endif
    
    xt /= (CAL_SAMPLE_TIMES - 2);
    yt /= (CAL_SAMPLE_TIMES - 2);
    if(t1_count > 0)
    { 
      zt1 /= t1_count;
      zt2 /= t1_count;
      zt1 = Touch_Formula_One(zt1,zt2,xt,Xr_PLANE);
    }
    if(t2_count)
    {
      zt3 /= t2_count;
      zt4 /= t2_count;
      zt3 = Touch_Formula_One(zt3,zt4,yt,Yr_PLANE);
    }
    if((t1_count) && (t2_count))
      z = (zt1 + zt3) / 2;
    else if(t1_count)
      z = zt1;
    else if(t2_count)
      z = zt3;
    else
      z = 0;

    ds->xbuf = xt;
    ds->ybuf = yt;
    ds->zbuf = z;
    return 1;
    
}
static void ts_transform_data(struct jz_ts_t *ts){

    struct ts_event *event = &ts->event;
    //   event->x = transform_to_screen_x(ts,ts->data_s.xbuf);
    // event->y = transform_to_screen_y(ts,ts->data_s.ybuf);
    // event->pressure = transform_to_screen_z(ts,ts->data_s.zbuf);
    event->x =ts->data_s.xbuf;
    event->y =ts->data_s.ybuf;
    event->pressure =ts->data_s.zbuf;
   
    if(event->pressure == 0) event->pressure = 1;
}
static void handle_ts_event(struct jz_ts_t *ts){
    struct ts_event *event = &ts->event;    
 
    input_report_abs(ts->input_dev, ABS_X, event->x);
    input_report_abs(ts->input_dev, ABS_Y, event->y);
    input_report_abs(ts->input_dev, ABS_PRESSURE, event->pressure);

//    printk("event->x = %d,event->y = %d event->pressure = %d\n",event->x,event->y,event->pressure);
    
    /* Android need it ... */
    input_report_key(ts->input_dev, BTN_TOUCH, 1);   
    
    input_sync(ts->input_dev);
    
}
    
static void handle_touch(struct jz_ts_t *ts,unsigned int *data,int size){
    /* drop no touch calibrate points */
    if(ts->cal_type & (~TOUCH_TYPE))
        ts->cal_type |= ~TOUCH_TYPE;
    if(ts->event_valid){
        handle_ts_event(ts);
        ts->event_valid = 0;
    }
    
    if(ts->touch_cal_count >= DROP_SAMPLE_TIMES)
    {
        if(ts->touch_cal_count < SAMPLE_TIMES){
            ts->ts_fifo[0][ts->touch_cal_count - DROP_SAMPLE_TIMES] = data[0];
            ts->ts_fifo[1][ts->touch_cal_count - DROP_SAMPLE_TIMES] = data[1];
        }else
        {        
            /* drop sample*/
            if(ts->cal_type & TOUCH_TYPE){
                if(ts_data_filter(ts)){
                    ts->event_valid = 1;
                    ts_transform_data(ts);
                }
                
            }
            ts->touch_cal_count = 0;
        }
    }
    ts->touch_cal_count++;
}

static void handle_pbat(struct jz_ts_t *ts,unsigned int *fifo,int size){
	ts->oldbat_value = (*fifo) & 0xfff; // max data = 4096
	jz_ts->cal_type &= ~BAT_TYPE;
	//printk("interrupt pbat v = %d\n",ts->oldbat_value);
	REG_SADC_ENA &= ~SADC_ENA_PBATEN;
}
static void handle_sadc(struct jz_ts_t *ts,unsigned int *fifo,int size){
    
}

static irqreturn_t sadc_interrupt(int irq, void * dev_id)
{
    struct jz_ts_t *ts = dev_id;
    unsigned int state;
    unsigned int fifo[FIFO_MAX_SIZE];
    static int pen_is_down = 0;
    
    spin_lock_irq(&ts->lock);
   
    state = REG_SADC_STATE & (~REG_SADC_CTRL);    
    /* first handle pen up interrupt */ 
    if(state & SADC_STATE_PENU){
        /* REG_SADC_CTRL used in pendown & penup mutex */
        
        REG_SADC_CTRL |= SADC_CTRL_PENUM;
        REG_SADC_CTRL &= ~SADC_CTRL_PENDM;
        
        if(pen_is_down == 1)
        {
            /*  mask pen up and wait pen down */
            REG_SADC_CTRL |= SADC_CTRL_TSRDYM;
            {
                input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
                /* Android need it ... */
                input_report_key(ts->input_dev, BTN_TOUCH, 0);
                input_sync(ts->input_dev);
                ts->cal_type &= ~TOUCH_TYPE;
                ts->event_valid = 0;
            }
            
        }
        pen_is_down = 0;
    }else if(state & SADC_CTRL_PENDM){
        /* REG_SADC_CTRL used in pendown & penup mutex */
        REG_SADC_CTRL |= SADC_CTRL_PENDM;
        REG_SADC_CTRL &= ~SADC_CTRL_PENUM;
        REG_SADC_CTRL &= ~SADC_CTRL_TSRDYM;
        
        if(pen_is_down == 0){
            /*  mask pen down and wait pen up */
            pen_is_down = 1;
            ts->event_valid = 0;
            ts->cal_type |= TOUCH_TYPE;
            ts->touch_cal_count = 0;
        }
        state |= SADC_STATE_PENU;        
	//  state |= SADC_STATE_SLEEPND;      
    }else if(state & SADC_STATE_TSRDY){

        fifo[0] = REG_SADC_TSDAT;
        fifo[1] = REG_SADC_TSDAT;
        
        /* alone here clear state */
        REG_SADC_STATE  = SADC_STATE_TSRDY;
        
        if(pen_is_down)
            handle_touch(ts,fifo,2);
        
    }else if(state & SADC_STATE_PBATRDY){

        
        fifo[0] = REG_SADC_BATDAT;
        handle_pbat(ts,fifo,1);

    }else if(state & SADC_STATE_SRDY){
        /* no use */
        fifo[0] = REG_SADC_SADDAT;
        handle_sadc(ts,fifo,1);
        
    }//else if(state & SADC_STATE_SLEEPND){
    else if(state){
        //after power will use 
        //REG_SADC_CTRL |= SADC_STATE_SLEEPND;
        
    }
    //when data count not is set_count penup is not clear; 
    if(!(state & SADC_STATE_TSRDY))
        REG_SADC_STATE = state;    
    spin_unlock_irq(&ts->lock);
    
    return IRQ_HANDLED;
}

#if 0
static void jz_acq_timer(unsigned long data)
{
	struct jz_ts_t *ts = (struct jz_ts_t *)data;
	spin_lock_irq(&ts->lock);
	//printk("REG_SADC_CTRL = %x REG_SADC_ENA = %x REG_SADC_CFG = %x\n",REG_SADC_CTRL,REG_SADC_ENA,REG_SADC_CFG);
	//printk("REG_SADC_STATE = %x REG_CPM_CLKGR = %x\n",REG_SADC_STATE,REG_CPM_CLKGR);    
	// schedule next acquire
	ts->acq_timer.expires = jiffies + 100;//TS_SCAN_INTERVAL;
	del_timer(&ts->acq_timer);
	add_timer(&ts->acq_timer);
	REG_CPM_CLKGR &= ~CPM_CLKGR_SADC;
	spin_unlock_irq(&ts->lock);

}
#endif

static int __init jz_ts_init(void)
{
	struct input_dev	*input_dev;
	struct jz_ts_t		*ts;
	int	error;

	DBG;
	ts = jz_ts = kzalloc(sizeof(struct jz_ts_t), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev)
		return -ENOMEM;

	input_dev->name = "qwerty"; /* Set to 'qwerty' to load /system/usr/keychars/qwerty.kcm.bin by Android */
	input_dev->phys = ts->phys;



/*
 	old:
	input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
*/

	/* For Android */
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
    
#ifdef CONFIG_JZ_ADKEY
	set_bit(DPAD_CENTER, input_dev->keybit);
	set_bit(DPAD_DOWN, input_dev->keybit);
	set_bit(DPAD_UP, input_dev->keybit);
	set_bit(DPAD_LEFT, input_dev->keybit);
	set_bit(DPAD_RIGHT, input_dev->keybit);
#endif
	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAXX + 1, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAXY + 1, 0, 0);
    input_set_abs_params(input_dev, ABS_PRESSURE, 0, PRESS_MAXZ + 1, 0, 0);
	input_set_drvdata(input_dev, ts);
	error = input_register_device(input_dev);
    
	strcpy(ts->phys, "input/ts0");
	spin_lock_init(&ts->lock);

	ts->input_dev = input_dev;
#if 0
	// Init ts acquisition timer function

	init_timer(&ts->acq_timer);
	ts->acq_timer.function = jz_acq_timer;
	ts->acq_timer.data = (unsigned long)ts;
	ts->irq_enabled = 1;
	ts->acq_timer.expires = jiffies + 100;
	add_timer(&ts->acq_timer);
#endif    
	if (error) {
		printk("Input device register failed !\n");
		goto err_free_dev;
	}
    
	sadc_init_clock(6);
	//ts_disable_pendown_irq();
	//ts_disable_penup_irq();
	REG_SADC_CTRL = 0x3f;
	
	error = request_irq(IRQ_SADC, sadc_interrupt, IRQF_DISABLED, TS_NAME, ts);
	if (error) {
		pr_err("unable to get PenDown IRQ %d", IRQ_SADC);
		goto err_free_irq;
	}
	ts->cal_type = 0;
	
	ts->oldbat_value = 0xffffffff;  //battery cal data first is invalid
	
	DBG;    
#ifdef CONFIG_JZ_ADKEY
	// Init key acquisition timer function
	init_timer(&ts->key_timer);
	ts->key_timer.function = key_timer_callback;
	ts->key_timer.data = (unsigned long)ts;
	ts->active_low = ACTIVE_LOW_ADKEY;

	error = request_irq(IRQ_GPIO_0 + GPIO_ADKEY_INT, key_interrupt, IRQF_DISABLED, TS_NAME, ts);
	if (error) {
		pr_err("unable to get AD KEY IRQ %d", IRQ_GPIO_0 + GPIO_ADKEY_INT);
		goto err_free_irq;
	}

	__gpio_disable_pull(GPIO_ADKEY_INT);
	if(ts->active_low)
		__gpio_as_irq_fall_edge(GPIO_ADKEY_INT);
	else
		__gpio_as_irq_rise_edge(GPIO_ADKEY_INT);

#endif
	sadc_start_ts();

	printk("input: JZ Touch Screen registered.\n");

	return 0;

err_free_irq:
	free_irq(IRQ_SADC, ts);
#ifdef CONFIG_JZ_ADKEY
	free_irq(IRQ_GPIO_0 + GPIO_ADKEY_INT, ts);
#endif
err_free_dev:
	input_free_device(ts->input_dev);
	kfree(ts);
        return 0;
}

static void __exit jz_ts_exit(void)
{

	ts_disable_pendown_irq();
	ts_disable_penup_irq();
	sadc_disable_ts();
	free_irq(IRQ_SADC, jz_ts);
	input_unregister_device(jz_ts->input_dev);

}

module_init(jz_ts_init);
module_exit(jz_ts_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("JZ TouchScreen Driver");
MODULE_AUTHOR("Jason <xwang@ingenic.com>");
