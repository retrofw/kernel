/*
 * linux/arch/mips/jz4750/i2c.c
 *
 * JZ4750 Simple I2C Driver.
 *
 * Copyright (c) 2005-2010  Ingenic Semiconductor Inc.
 * Author: River <zwang@ingenic.cn>
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/jzsoc.h>

#define I2C_BUS_TIMING_START_HOLD	1	/* us */
#define I2C_BUS_TIMING_STOP_HOLD	10	/* us */

#define I2C_CTRL_WAIT_ENABLE		1	/* us */
#define I2C_CTRL_WAIT_DISABLE		1	/* us */

#define PFX "I_I2C"

#define D(fmt, args...) \
//	printk(KERN_ERR PFX": %s(): LINE: %d - "fmt"\n", __func__, __LINE__, ##args)

#define E(fmt, args...) \
	printk(KERN_ERR PFX": %s(): LINE: %d - "fmt"\n", __func__, __LINE__, ##args)

/* Controller. */
struct i2c_ctrl {
	spinlock_t lock;

	unsigned long clk;
};

static struct i2c_ctrl g_i2c_ctrl;

static inline void i2c_ctrl_enable(void)
{
	D("Called.");

	__i2c_enable();
	
	udelay(I2C_CTRL_WAIT_ENABLE);
	
	return;
}
	
static inline void i2c_ctrl_disable(void)
{
	D("Called.");

	__i2c_disable();

	udelay(I2C_CTRL_WAIT_DISABLE);
	
	return;
}

static inline void i2c_ctrl_set_clk(unsigned long clk)
{
	struct i2c_ctrl *ctrl = &g_i2c_ctrl;
	
	D("Called.");

	if (ctrl->clk != clk) {
		D("Set clock.");

		__i2c_set_clk(jz_clocks.extalclk, clk);
		ctrl->clk = clk;

		mdelay(1);
	}

	return;
}

/*
 * I2C bus protocol basic routines
 */
static inline int __i2c_put_data(unsigned char data, unsigned long timeout)
{
	unsigned long t; 
	
	__i2c_write(data);
	__i2c_set_drf();
	
	t = timeout;
	while (__i2c_check_drf() != 0 && t)
		t--;

	if (!t) {
		E("__i2c_check_drf() timeout, Data: 0x%x. timeout: %lu.", data, timeout);
		return -ETIMEDOUT;
	}

	while (!__i2c_transmit_ended() && t)
		t--;

	if (!t) {
		E("__i2c_transmit_ended() timeout, Data: 0x%x, timeout: %lu.", data, timeout);
		return -ETIMEDOUT;
	}

	t = timeout;
	while (!__i2c_received_ack() && t)
		t--;

	if (!t) {
		E("__i2c_received_ack() timeout, Data: 0x%x, timeout: %lu.", data, timeout);
		return -ETIMEDOUT;
	}

	return 0;
}

static inline int __i2c_get_data(unsigned char *data, int ack, unsigned long timeout)
{
	unsigned long t;

	ack ? __i2c_send_ack() : __i2c_send_nack();
	
	t = timeout;
	while (__i2c_check_drf() == 0 && t)
		t--;

	if (!t) {
		E("__i2c_check_drf() timeout. timeout: %lu", timeout);
		return -ETIMEDOUT;
	}

	*data = __i2c_read();

	__i2c_clear_drf();

	return 0;
}

static inline int put_data(struct i_i2c_dev *dev, unsigned char data)
{
	return	__i2c_put_data(data, dev->timing->timeout);
}

static inline int get_data(struct i_i2c_dev *dev, unsigned char *data, int ack)
{
	return	__i2c_get_data(data, ack, dev->timing->timeout);
}

static inline void i2c_start(void)
{
	__i2c_send_start();
	
	udelay(I2C_BUS_TIMING_START_HOLD);

	return;
}

static inline void i2c_stop(void)
{
	__i2c_send_stop();
	
	udelay(I2C_BUS_TIMING_STOP_HOLD);	

	return;
}

static inline int i2c_start_and_send_address(struct i_i2c_dev *dev, off_t off, int dir)
{
	unsigned int ra = ((dev->address << 1) & 0xff) | 0x1;
	unsigned int wa = ((dev->address << 1) & 0xff);
	
	int rv;

	i2c_start();

	if (put_data(dev, wa) < 0) {
		E("Failed to send write address.");
		rv = -ENODEV;
		goto err;
	}

	if (dev->cap & I_I2C_CAP_16BIT_OFFSET_MSB) {
		if (put_data(dev, (off >> 8) & 0xFF) < 0) {
			E("Failed to send off >> 8 MSB.");
			rv = -EINVAL;
			goto err;
		}
	}

	if (put_data(dev, (off & 0xFF)) < 0) {
		E("Failed to send off.");
		rv = -EINVAL;
		goto err;
	}

	if (dev->cap & I_I2C_CAP_16BIT_OFFSET_LSB) {
		if (put_data(dev, (off >> 8) & 0xFF) < 0) {
			E("Failed to send off >> 8 LSB.");
			rv = -EINVAL;
			goto err;
		}
	}

	if (dir == I_I2C_IO_DIR_READ) {
		if (dev->flags & I_I2C_FLAG_STOP_BEFORE_RESTART) 
			i2c_stop();

		i2c_start();

		if (put_data(dev, ra) < 0) {
			E("Failed to send read address.");
			rv = -ENODEV;
			goto err;
		}
	}

	return 0;

err:
	i2c_stop();
	return rv;
}

static inline int i2c_read_data(struct i_i2c_dev *dev, char *buf, size_t count)
{
	int i, rv;
	int ack = 1;
	
//	D("buf: 0x%p, count: %d.", buf, count);

	for (i = 0; i < count; i++) {
		if (i == count - 1) {
//			D("Nack.");
			ack = 0;
		}

		rv = get_data(dev, buf + i, ack);
		if (rv) {
			E("get_data() failed: rv: %d i: %d.", rv, i);
			return rv;
		}
	}
	
	return 0;
}

static inline int i2c_write_data(struct i_i2c_dev *dev, char *buf, size_t count)
{
	unsigned long i;
	int rv;
	
//	D("buf: 0x%p, count: %d.", buf, count);

	for (i = 0; i < count; i++) {
		rv = put_data(dev, buf[i]);
		if (rv) {
			E("put_data() failed: %d.\n", rv);
			return rv;
		}
	}
	
	return 0;
}

static int do_i2c(struct i_i2c_dev *dev, off_t off, char *buf, size_t count, int dir)
{
	struct i2c_ctrl *ctrl = &g_i2c_ctrl;
	struct i_i2c_timing *timing = dev->timing;

	unsigned long flags;
	unsigned long io_size, size;

	unsigned long i;
	int rv = 0;

	spin_lock_irqsave(&ctrl->lock, flags);
	
	i2c_ctrl_enable();
	i2c_ctrl_set_clk(timing->clk);

	io_size = dir ? dev->write_size : dev->read_size;
	size = count < io_size ? count : io_size;

	for (i = 0; i < count; i += io_size) {
		rv = i2c_start_and_send_address(dev, off + i, dir);
		if (rv) {
			E("i2c_start_and_send_address() failed: %d, i: %lu.", rv, i);
			i2c_stop();
			goto err;
		}
		
//		D("Pass send address.");

		if (!dir)
			rv = i2c_read_data(dev, buf + i, size);
		else
			rv = i2c_write_data(dev, buf + i, size);

//		D("Pass rw data.");

		if (rv) {
			E("i2c_read/write_data() failed: %d, i: %lu.", rv, i);
			i2c_stop();
			goto err;
		}
		
		i2c_stop();
		
		if (dir && timing->t_wr) 
			mdelay(timing->t_wr);
	}
	
err:
	i2c_ctrl_disable();

	spin_unlock_irqrestore(&ctrl->lock, flags);

	return rv;
}

/*
 * I2C interface
 */
int i_i2c_read_dev(struct i_i2c_dev *dev, off_t off, void *buf, size_t count)
{
	D("off: %d, buf: 0x%p, count: %d.", off, buf, count);

	return do_i2c(dev, off, buf, count, 0);
}
EXPORT_SYMBOL(i_i2c_read_dev);

int i_i2c_write_dev(struct i_i2c_dev *dev, off_t off, void *buf, size_t count)
{
	D("off: %d, buf: 0x%p, count: %d.", off, buf, count);

	return do_i2c(dev, off, buf, count, 1);
}
EXPORT_SYMBOL(i_i2c_write_dev);

int i_i2c_init_dev(struct i_i2c_dev *dev)
{
	struct i_i2c_timing *timing = dev->timing;

	if (!timing) {
		printk(KERN_ERR PFX": %s(): Please setup the timing of I2C device: 0x%p.\n", __func__, dev);
		return -EINVAL;
	}	
	
	if (!timing->timeout)
		timing->timeout = (100 * 1000);
	
	if (!timing->clk)
		timing->clk = (100 * 1000); /* default 100 KHz */
	
	if (!(dev->cap & I_I2C_CAP_SEQ_READ))  
		dev->read_size = 1;
	
	if (!(dev->cap & I_I2C_CAP_SEQ_WRITE))
		dev->write_size = 1;

	return 0;	
}
EXPORT_SYMBOL(i_i2c_init_dev);

static int __init i_i2c_init(void)
{
	struct i2c_ctrl *ctrl = &g_i2c_ctrl;

	spin_lock_init(&ctrl->lock);
	
	__gpio_as_i2c();

	i2c_ctrl_set_clk(100 * 1000);

	printk(KERN_INFO JZ_SOC_NAME": Simple I2C Driver Registered.\n");

	return 0;
}

module_init(i_i2c_init);
