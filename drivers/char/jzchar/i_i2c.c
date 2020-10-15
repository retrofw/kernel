/*
 * JZ4750 Simple I2C Userspace Driver.
 *
 * Copyright (c) 2005-2010  Ingenic Semiconductor Inc.
 * Author: River <zwang@ingenic.cn>
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/miscdevice.h>

#include <asm/uaccess.h>
#include <asm/jzsoc.h>

#include "i_i2c_abi.h"

/* ------------- CUSTOM: Add your own devices/timings here. -------------*/
static struct i_i2c_dev i_i2c_devs[] = {
	{
		.id = I_I2C_ID_AT24C16B,

		.name = "AT24C16B",
		.address = 0x50,	/* 7 bit device address. */
		.cap = I_I2C_CAP_SEQ_READ | I_I2C_CAP_SEQ_WRITE, /* Device capabilites. */
		.size = 16 * 1024,	/* The range of offset will be checked when set. */
		.read_size = 16 * 1024, /* SEQ Read size. */
		.write_size = 16,	/* SEQ Write size. */
	},
};

static struct i_i2c_timing i_i2c_timings[] = {
	{
		.id = I_I2C_ID_AT24C16B,

		.clk = 100 * 1000,	/* I2C Device clock - Default: 100K. */
		.timeout = 100 * 1000,	/* MAX allowed timeout in loops */
		.t_wr = 5,		/* tWR / t(Stop - Next Start) in ms */
	},
};
/*--------------------------------------------------------------------*/

#define DRV_NAME "Simple I2C Userspace Driver"
#define DRV_VERSION "0.1"

static struct i_i2c_dev *find_dev(int id)
{
	struct i_i2c_dev *dev;
	unsigned int i;

	dev = i_i2c_devs;
	for (i = 0; i < sizeof(i_i2c_devs) / sizeof(struct i_i2c_dev); i++, dev++) 
		if (dev->id == id)
			return dev;

	return NULL;
}

static struct i_i2c_timing *find_timing(int id)
{
	struct i_i2c_timing *timing;
	unsigned int i;

	timing = i_i2c_timings;
	for (i = 0; i < sizeof(i_i2c_timings) / sizeof(struct i_i2c_timing); i++, timing++) 
		if (timing->id == id)
			return timing;

	return NULL;
}

static void prepare_device_and_timing(void)
{
	struct i_i2c_dev *dev;
	struct i_i2c_timing *timing;

	unsigned int i;
	
	dev = i_i2c_devs;

	for (i = 0; i < sizeof(i_i2c_devs) / sizeof(struct i_i2c_dev); i++, dev++) {
		if (!dev->timing_id)
			timing = find_timing(dev->id);
		else
			timing = find_timing(dev->timing_id);

		if (!timing) {
			printk(KERN_ERR "%s(): Cannot find timing for device: %s.\n", __func__, dev->name);
			continue;
		}
		
		dev->timing = timing;
		dev->timing_id = timing->id;
		
		i_i2c_init_dev(dev);
		
		printk("Found I2C Device: %s - Address: 0x%x.\n", dev->name, dev->address);
	}
	
	return;
}

static int i_i2c_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i_i2c_dev *dev;
	struct i_i2c_control control;
	
	if (copy_from_user(&control, (void __user *)arg, sizeof(control))) {
		return -EFAULT;
	}

	dev = find_dev(control.id);
	if (!dev || !dev->timing) {
		printk(KERN_ERR "%s(): ID %d not found or not timing attached.\n", __func__, control.id);
		return -ENODEV;
	}

	if (dev->size && control.offset + control.count > dev->size) {
		return -EFAULT;
	}

	switch (cmd) {
		case I_I2C_IOC_READ_DEV:
			if (!access_ok(VERIFY_WRITE, control.buf, control.count)) {
				return -EFAULT;
			}
			
			return i_i2c_read_dev(dev, control.offset, control.buf, control.count);

		case I_I2C_IOC_WRITE_DEV:
			if (!access_ok(VERIFY_READ, control.buf, control.count)) {
				return -EFAULT;
			}

			return i_i2c_write_dev(dev, control.offset, control.buf, control.count);		
		default:
			return -EINVAL;
	}
	
	return 0;
}

static const struct file_operations i_i2c_fops = {
	.owner = THIS_MODULE,
	.ioctl = i_i2c_ioctl,
};

static struct miscdevice i_i2c_misc_device = {
        .minor          = MISC_DYNAMIC_MINOR,
        .name           = "i_i2c",
        .fops           = &i_i2c_fops,
};

static int __init i_i2c_init(void)
{
	int rv;

	printk(KERN_INFO JZ_SOC_NAME": %s - %s.\n", DRV_NAME, DRV_VERSION);

	prepare_device_and_timing();
	
	rv = misc_register(&i_i2c_misc_device);
	if (rv) {
		printk(KERN_ERR "%s(): Failed to register misc device.\n", __func__);
		return rv;
	}
	
	printk(KERN_INFO JZ_SOC_NAME": %s Registered.\n", DRV_NAME);

	return 0;
}

static void __exit i_i2c_exit(void)
{
	misc_deregister(&i_i2c_misc_device);
}

MODULE_AUTHOR("River Wang <zwang@ingenic.cn>");
MODULE_DESCRIPTION("Ingenic Simple I2C Userspace Driver");
MODULE_LICENSE("GPL");

module_init(i_i2c_init);
module_exit(i_i2c_exit);
