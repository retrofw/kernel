/*
 * linux/drivers/char/jzchar/sensor.c
 *
 * Common CMOS Camera Sensor Driver
 *
 * Copyright (C) 2006  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

//#include <linux/config.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
//#include <linux/jz-chars.h>
//#include "jz-chars.h"

#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/jzsoc.h>

#include <linux/videodev.h>
#include <media/v4l2-common.h>
#include <linux/video_decoder.h>

MODULE_AUTHOR("Jianli Wei<jlwei@ingenic.cn>");
MODULE_DESCRIPTION("Common CMOS Camera Sensor Driver");
MODULE_LICENSE("GPL");

/*
 *	ioctl commands
 */
#define IOCTL_SET_ADDR            0 /* set i2c address */
#define IOCTL_SET_CLK             1 /* set i2c clock */
#define IOCTL_WRITE_REG           2 /* write sensor register */
#define IOCTL_READ_REG            3 /* read sensor register */

/*
 *	i2c related
 */
static unsigned int i2c_addr = 0x42;
static unsigned int i2c_clk = 100000;

struct video_device *jz_sensor;

static void write_reg(u8 reg, u8 val)
{
	i2c_open();
	i2c_setclk(i2c_clk);
	i2c_write((i2c_addr >> 1), &val, reg, 1);
	i2c_close();
}

static u8 read_reg(u8 reg)
{
	u8 val;

	i2c_open();
	i2c_setclk(i2c_clk);
	i2c_read((i2c_addr >> 1), &val, reg, 1);
	i2c_close();
	return val;
}

/*
 * fops routines
 */

static int sensor_open(struct inode *inode, struct file *filp);
static int sensor_release(struct inode *inode, struct file *filp);
static ssize_t sensor_read(struct file *filp, char *buf, size_t size, loff_t *l);
static ssize_t sensor_write(struct file *filp, const char *buf, size_t size, loff_t *l);
static int sensor_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);

static struct file_operations sensor_fops = 
{
	open:		sensor_open,
	release:	sensor_release,
	read:		sensor_read,
	write:		sensor_write,
	ioctl:		sensor_ioctl,
};

static int sensor_open(struct inode *inode, struct file *filp)
{
	try_module_get(THIS_MODULE);
 	return 0;
}

static int sensor_release(struct inode *inode, struct file *filp)
{
	module_put(THIS_MODULE);
	return 0;	
}

static ssize_t sensor_read(struct file *filp, char *buf, size_t size, loff_t *l)
{
	printk("sensor: read is not implemented\n");
	return -1;
}

static ssize_t sensor_write(struct file *filp, const char *buf, size_t size, loff_t *l)
{
	printk("sensor: write is not implemented\n");
	return -1;
}

static int sensor_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	switch (cmd) {
	case IOCTL_SET_ADDR:
		if (copy_from_user(&i2c_addr, (void *)arg, 4))
			return -EFAULT;
		break;
	case IOCTL_SET_CLK:
		if (copy_from_user(&i2c_clk, (void *)arg, 4))
			return -EFAULT;
		break;
	case IOCTL_WRITE_REG:
	{
		u8 regval[2];

		if (copy_from_user(regval, (void *)arg, 2))
			return -EFAULT;

		write_reg(regval[0], regval[1]);
		break;
	}
	case IOCTL_READ_REG:
	{
		u8 reg, val;

		if (copy_from_user(&reg, (void *)arg, 1))
			return -EFAULT;

		val = read_reg(reg);

		if (copy_to_user((void *)(arg + 1), &val, 1))
			return -EFAULT;
		break;
	}
	default:
		printk("Not supported command: 0x%x\n", cmd);
		return -EINVAL;
		break;
	}
	return ret;
}

static struct video_device jz_v4l_device = {
	.name		= "jz sensor",
	//.type		= VID_TYPE_CAPTURE | VID_TYPE_SUBCAPTURE |
	//	VID_TYPE_CLIPPING | VID_TYPE_SCALES, VID_TYPE_OVERLAY
	.fops		= &sensor_fops,
	.minor		= -1,
};

/*
 * Module init and exit
 */

static int __init jz_sensor_init(void)
{
	int ret;
//	cim_dev = kzalloc(sizeof(struct cim_device), GFP_KERNEL);
	jz_sensor = video_device_alloc();
	memcpy(jz_sensor, &jz_v4l_device, sizeof(struct video_device));
	jz_sensor->release = video_device_release;
//	ret = jz_register_chrdev(SENSOR_MINOR, "sensor", &sensor_fops, NULL);
	ret = video_register_device(jz_sensor, VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		return ret;
	}

	printk("Ingenic CMOS camera sensor driver registered\n");

	return 0;
}

static void __exit jz_sensor_exit(void)
{
//	jz_unregister_chrdev(SENSOR_MINOR, "sensor");
	video_unregister_device(jz_sensor);
}

module_init(jz_sensor_init);
module_exit(jz_sensor_exit);
