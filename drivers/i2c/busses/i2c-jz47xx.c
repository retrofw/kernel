
/*
 * I2C adapter for the INGENIC I2C bus access.
 *
 * Copyright (C) 2006 - 2009 Ingenic Semiconductor Inc.
 * Author: <cwjia@ingenic.cn>
 * Date:20091027 modified by <zhzhao@ingenic.cn> 
 * Date:20091105 modified by <hlguo@ingenic.cn>
 * Date:20091120 modified by <hlguo@ingenic.cn>
 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <linux/module.h>
#include <asm/addrspace.h>

#include <asm/jzsoc.h>
#include "i2c-jz47xx.h"

/* I2C protocol */
#define I2C_READ	1
#define I2C_WRITE	0

#define TIMEOUT         1000

#define __reg_printk() printk("%s:%d:REG_I2C_CR=%x ,REG_I2C_SR = %x , REG_I2C_GR = %x\n",\
			      __FUNCTION__,__LINE__,REG_I2C_CR,REG_I2C_SR,REG_I2C_GR);

/* The value of the most significant byte of sub_addr 
 * indicate the length of sub address:
 * zero:1 byte, non-zero:2 bytes
 */
unsigned long sub_addr;

struct jz_i2c {
	spinlock_t		lock;
	wait_queue_head_t	wait;
	struct i2c_msg		*msg;
	unsigned int		msg_num;
	unsigned int		slave_addr;
	struct i2c_adapter	adap;
	struct clk		*clk;
};

/*
 * I2C bus protocol basic routines
 */

static int check_i2c_is_busy(void)
{
	int timeout = TIMEOUT;
	while (__i2c_is_busy() && --timeout){
		udelay(10);
	}
	if (!timeout){
		printk("i2c is busy---\n");
		return -ETIMEDOUT;
	}
	else
		return 0;
}

static void delay_i2c_clock(int count)
{
	volatile int i, j;
	for (i = 0; i < count; i++) {
		for (j = 0; j < 2000; j++) {
			;
		}
	}
}

static int i2c_put_data(unsigned char data)
{
	if (__i2c_check_drf()){
		printk("WARNING: need clear DRF first\n");
		__i2c_clear_drf();
		delay_i2c_clock(1);
	}
	__i2c_write(data);
	__i2c_set_drf();

	do {
		delay_i2c_clock(1);
	} while (__i2c_check_drf() != 0);

	/* wait for the i2c controller set the ack BIT*/
	delay_i2c_clock(1);

	if (__i2c_received_ack())
		return 0;
	else {
		printk("%s ERROR, get an NACK\n", __FUNCTION__);
		return -ETIMEDOUT;
	}
}


static int i2c_get_data(unsigned char *data, int ack)
{
	while (__i2c_check_drf() == 0)
		delay_i2c_clock(1);

	*data = __i2c_read();

	/* wait for the i2c controller from TRANSFERRING to IDLE*/
	delay_i2c_clock(1);

	__i2c_clear_drf();

	if (!ack) {
		__i2c_send_nack();
		__i2c_send_stop();
	}

	return 0;
}

/*
 * I2C interface
 */
void  i2c_jz_setclk(struct i2c_client *client,unsigned long i2cclk)
{
	__i2c_set_clk(jz_clocks.extalclk, i2cclk);
}

static int xfer_read(unsigned char device, unsigned char *buf, int length)
{
	int cnt = 0;

	if (length == 1)
		__i2c_send_nack();
	else
		__i2c_send_ack();
#if defined(CONFIG_TOUCHSCREEN_JZ_MT4D)
	if ((device == 0x40) && __gpio_get_pin(GPIO_ATTN)) {
		return -EBUSY;
	}
#endif
	__i2c_send_start();
	__i2c_write((device << 1) | I2C_READ);
	__i2c_set_drf();

	/* wait for i2c controller from IDLE to TRANSFERRING*/
	delay_i2c_clock(1);

	while (!__i2c_transmit_ended())
		delay_i2c_clock(1);
	if (!__i2c_received_ack())
		goto xfer_read_err;

	do {
		i2c_get_data(buf, (length - cnt) != 2);

		cnt++;
		buf++;
	} while (cnt < length);

	if (length == 1)
		__i2c_send_stop();

	do {
		__i2c_clear_drf();
		/* wait for i2c controller from TRANSFERRING to IDLE*/
		delay_i2c_clock(8);
	} while (__i2c_check_drf());

	return 0;

xfer_read_err:
	__i2c_send_stop();
	printk("Read I2C device 0x%2x failed.\n", device);

	return -ENODEV;
}

static int xfer_write(unsigned char device, unsigned char *buf, int length)
{
	int cnt = 0, ret = 0;

	__i2c_send_start();
	if (i2c_put_data( (device << 1) | I2C_WRITE ) < 0) {
		ret = -ENODEV;
		goto xfer_write_err;
	}

	while (cnt < length) {
		ret = i2c_put_data(*buf);
		if (ret < 0) 
			goto xfer_write_err;

		cnt++;
		buf++;
	}

xfer_write_err:
	__i2c_send_stop();
	while (!__i2c_transmit_ended())
		delay_i2c_clock(1);

	if (ret == -ENODEV)
		printk("Write I2C device 0x%2x failed\n", device);

	return ret;
}

static int i2c_jz_xfer(struct i2c_adapter *adap, struct i2c_msg *pmsg, int num)
{
	int ret, i;
	
	dev_dbg(&adap->dev, "jz47xx_xfer: processing %d messages:\n", num);
	for (i = 0; i < num; i++) {
		dev_dbg(&adap->dev, " #%d: %sing %d byte%s %s 0x%02x\n", i,
			pmsg->flags & I2C_M_RD ? "read" : "writ",
			pmsg->len, pmsg->len > 1 ? "s" : "",
			pmsg->flags & I2C_M_RD ? "from" : "to",	pmsg->addr);
		if (pmsg->len && pmsg->buf) {	/* sanity check */
			if (pmsg->flags & I2C_M_RD){
				ret = xfer_read(pmsg->addr, pmsg->buf, pmsg->len);
			} else {
				ret = xfer_write(pmsg->addr, pmsg->buf, pmsg->len);
			}
			if (ret)
				return ret;
			/* Wait until transfer is finished */
		}
		dev_dbg(&adap->dev, "transfer complete\n");
		pmsg++;		/* next message */
	}
	
	return i;
}

static u32 i2c_jz_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm i2c_jz_algorithm = {
	.master_xfer	= i2c_jz_xfer,
	.functionality	= i2c_jz_functionality,
};

static int i2c_jz_probe(struct platform_device *dev)
{
	struct jz_i2c *i2c;
	struct i2c_jz_platform_data *plat = dev->dev.platform_data;
	int ret;

	__gpio_as_i2c();
	__i2c_set_clk(jz_clocks.extalclk, 100000); /* default 100 KHz */
	udelay(10);
	__i2c_enable();
	__reg_printk();
	i2c = kzalloc(sizeof(struct jz_i2c), GFP_KERNEL);
	if (!i2c) {
		printk("There is no enough memory\n");
		ret = -ENOMEM;
		goto emalloc;
	}

	i2c->adap.owner   = THIS_MODULE;
	i2c->adap.algo    = &i2c_jz_algorithm;
	i2c->adap.retries = 5;
	spin_lock_init(&i2c->lock);
	init_waitqueue_head(&i2c->wait);
	sprintf(i2c->adap.name, "jz_i2c-i2c.%u", dev->id);
	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &dev->dev;

	if (plat) {
		i2c->adap.class = plat->class;
	}

	/*
	 * If "dev->id" is negative we consider it as zero.
	 * The reason to do so is to avoid sysfs names that only make
	 * sense when there are multiple adapters.
	 */
	i2c->adap.nr = dev->id != -1 ? dev->id : 0;
	/* ret = i2c_add_adapter(&i2c->adap); */
	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		printk(KERN_INFO "I2C: Failed to add bus\n");
		goto eadapt;
	}

	platform_set_drvdata(dev, i2c);
	dev_info(&dev->dev, "JZ47xx i2c bus driver.\n");
	return 0;
eadapt:
	__i2c_disable();
emalloc:
	return ret;
}

static int i2c_jz_remove(struct platform_device *dev)
{
	struct i2c_adapter *adapter = platform_get_drvdata(dev);
	int rc;

	rc = i2c_del_adapter(adapter);
	platform_set_drvdata(dev, NULL);
	return rc;
}

static struct platform_driver i2c_jz_driver = {
	.probe		= i2c_jz_probe,
	.remove		= i2c_jz_remove,
	.driver		= {
		.name	= "jz_i2c",
	},
};

static int __init i2c_adap_jz_init(void)
{
	return platform_driver_register(&i2c_jz_driver);
}

static void __exit i2c_adap_jz_exit(void)
{
	return platform_driver_unregister(&i2c_jz_driver);
}

MODULE_LICENSE("GPL");
subsys_initcall(i2c_adap_jz_init);
//arch_initcall(i2c_adap_jz_init);
module_exit(i2c_adap_jz_exit);
