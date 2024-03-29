/*
 * linux/drivers/char/jzchar/sensor.c
 *
 * Common G-Sensor Driver
 *
 * Copyright (C) 2006  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/jzsoc.h>
#include <linux/proc_fs.h>

#define USE_GPIO_I2C 1

#include "jzchars.h"
#include "gsensor_i2c.h"


#if USE_GPIO_I2C
#include "fm_i2c.h"
#endif

MODULE_AUTHOR("caijicheng@umidotech.com");
MODULE_DESCRIPTION("mxc6225xu Sensor Driver");
MODULE_LICENSE("GPL");

#define SENSOR_NAME	"mxc6225xu"
#define SENSOR_I2C_ADDR		0x15

//#define GPIO_GSENSOR_VCC (32+31)


char xout,yout;
#define READ_INTER 8

char read_xdata = 0x00;
char read_ydata = 0x01;
static void l009_gsensor_reset()
{

#if USE_GPIO_I2C
	return ;
#endif

#if 0
	i2c_close();
	__cpm_stop_i2c();
	__cpm_start_i2c();
	i2c_open();
#else
	__i2c_disable(1);
	cpm_stop_clock(CGM_I2C1);
	cpm_start_clock(CGM_I2C1);
	__i2c_enable(1);
#endif
}

unsigned int l009_gsensor_read()
{
	static unsigned int val = 0;
	static unsigned long old_jiffies = 0;

	if((jiffies - old_jiffies) < READ_INTER)
	return val;
	else
	old_jiffies = jiffies;

	val = 0;
        int ret = 0;
#if USE_GPIO_I2C
	ret = gsensor_i2c_read_device(0x00,1,(char *)&read_xdata);
#else
        read_xdata = 0x00;
        ret = gsensor_i2c_rxdata((char *)&read_xdata, 1); // read x
#endif
        if(ret < 0)
        {
          l009_gsensor_reset();
          return val;
        }
#if USE_GPIO_I2C
	ret = gsensor_i2c_read_device(0x01,1,(char *)&read_ydata);
#else
	read_ydata = 0x01;
        ret = gsensor_i2c_rxdata((char *)&read_ydata, 1); // read y
#endif
	if(ret < 0)
        {
          l009_gsensor_reset();
          return val;
        }

#if USE_GPIO_I2C
	printk("gpio gsensor read xdata is  %d  ydata is %d\n",read_xdata,read_ydata);
#else
	printk("gsensor read xdata is  %d  ydata is %d\n",read_xdata,read_ydata);
#endif

	if(read_ydata > 12 )	    val |= 0x01;    //UP
	if(read_ydata < -12 )     val |= 0x02;    //DOWN
	if(read_xdata < -12 )     val |=  0x08;   //LEFT
	if(read_xdata > 12 )      val |=  0x04;   //RIGHT
	printk("val = 0x%x\n",val);
	return val;
}
EXPORT_SYMBOL(l009_gsensor_read);



/*
 * Module init and exit
 */

static int __init sensor_init(void)
{
	return 0;
}

static void __exit sensor_exit(void)
{
}

module_init(sensor_init);
module_exit(sensor_exit);
