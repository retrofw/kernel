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

/* I2C protocol */
#define I2C_READ	1
#define I2C_WRITE	0

//#define DEBUG_I2C

#ifdef DEBUG_I2C
#define D(fmt, args...)							\
	({								\
		printk(KERN_ERR PFX": %s(): LINE: %d - "fmt"\n", __func__, __LINE__, ##args); \
		printk(KERN_ERR PFX": CR = %#0x SR = %#0x\n", REG_I2C_CR, REG_I2C_SR); \
	})

#define E(fmt, args...)				\
	({								\
		printk(KERN_ERR PFX": %s(): LINE: %d - "fmt"\n", __func__, __LINE__, ##args); \
		printk(KERN_ERR PFX": CR = %#0x SR = %#0x\n", REG_I2C_CR, REG_I2C_SR); \
	})
#else
#define D(fmt, args...) do {  } while(0)
#define E(fmt, args...) do {  } while(0)
#endif

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

#if 0
	while (!__i2c_transmit_ended())
		delay_i2c_clock(1);
#endif

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

	//__i2c_send_stop();

	return 0;

xfer_read_err:
	__i2c_send_stop();
	printk("Read I2C device 0x%2x failed.\n", device);

	return -ENODEV;
}

static int xfer_write(unsigned char device, unsigned char *buf, int length, int stop_before_restart)
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
	if ( (ret < 0) || stop_before_restart)
		__i2c_send_stop();
	while (!__i2c_transmit_ended()) ;
		delay_i2c_clock(1);

	if (ret == -ENODEV)
		printk("Write I2C device 0x%2x failed\n", device);

	return ret;
}

static __inline__ int __write_mem_location(struct i_i2c_dev *dev, off_t off, unsigned char offset[2])
{
	int idx = 0;

	if (dev->cap & I_I2C_CAP_16BIT_OFFSET_MSB) {
		offset[idx++] = (off >> 8) & 0xFF;
	}

	offset[idx++] = off & 0xFF;

	if (dev->cap & I_I2C_CAP_16BIT_OFFSET_LSB) {
		offset[idx++] = (off >> 8) & 0xFF;
	}

	return idx;
}

static int write_mem_location(struct i_i2c_dev *dev, off_t off)
{
	int nb_off = 0;
	int ret;
	unsigned char offset[2] = { 0 };

	nb_off = __write_mem_location(dev, off, offset);
		/* write the offset */
	ret = xfer_write(dev->address, offset, nb_off, 0);
	if (ret != 0)
		printk("write dev(%#0x) offset(%d) error, ret = %d\n", dev->address, (int)off, ret);

	return ret;
}

/*
 * I2C interface
 */
int i_i2c_read_dev(struct i_i2c_dev *dev, off_t off, void *buf, size_t count)
{
	int ret = 0;
	int rd_bytes = 0;
	//struct i2c_ctrl *ctrl = &g_i2c_ctrl;
	struct i_i2c_timing *timing = dev->timing;

	//unsigned long flags;
	unsigned long io_size, size;

	unsigned long i;
	unsigned char *data = (unsigned char *)buf;

	//spin_lock_irqsave(&ctrl->lock, flags);

	i2c_ctrl_set_clk(timing->clk);
	i2c_ctrl_enable();

	io_size = dev->read_size;
	size = count < io_size ? count : io_size;

	for (i = 0; i < count; i += io_size) {
		ret = write_mem_location(dev, off + i);
		if (ret != 0)
			goto err;
		/* read data */
		rd_bytes = ( (count - i) > size) ? size : (count - i);
		ret = xfer_read(dev->address, data + i, rd_bytes);
		if (ret != 0)
			goto err;
	}

	ret = count;
 err:
	i2c_ctrl_disable();
	//spin_unlock_irqrestore(&ctrl->lock, flags);
	return ret;
}
EXPORT_SYMBOL(i_i2c_read_dev);

int i_i2c_write_dev(struct i_i2c_dev *dev, off_t off, void *buf, size_t count)
{
	int ret = 0;
	int n_bytes = 0;
	//struct i2c_ctrl *ctrl = &g_i2c_ctrl;
	struct i_i2c_timing *timing = dev->timing;

	//unsigned long flags;
	unsigned long io_size, size;

	unsigned long i;
	unsigned char *buffer = (unsigned char *)buf;
	unsigned char *data = NULL;

	int nb_off = 0;
	unsigned char offset[2] = { 0 };

	//spin_lock_irqsave(&ctrl->lock, flags);

	i2c_ctrl_set_clk(timing->clk);
	i2c_ctrl_enable();

	io_size = dev->write_size;
	size = count < io_size ? count : io_size;

	data = kzalloc(size + 2, GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < count; i += size) {
		// printk("===>write offset = %d\n", (int)(off + i));
		memset(data, 0, size + 2);
		nb_off =  __write_mem_location(dev, off + i, offset);
		memcpy(data, offset, nb_off);

		n_bytes = ( (count - i) > size) ? size : (count - i);
		memcpy(data + nb_off, buffer + i, n_bytes);

		ret = xfer_write(dev->address, data, n_bytes + nb_off, 1);
		if (ret != 0)
			goto err;

		if (dev->timing->t_wr)
			mdelay(dev->timing->t_wr);
	}



	ret = count;

 err:
	i2c_ctrl_disable();
	//spin_unlock_irqrestore(&ctrl->lock, flags);
	return ret;
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


static struct i_i2c_timing i2c_tm_default = {
	.timeout = 100 * 1000,
	.clk = 100 * 1000, /* default 100KHZ */
	.t_wr = 5,	   /* eeprom write cycle time: 5ms */
};

static struct i_i2c_dev i2c_dev_default = {
	.name = "default_i2c_adapter",
	.cap = I_I2C_CAP_SEQ_READ | I_I2C_CAP_SEQ_WRITE,
	.flags = 0,
	.read_size = 16, /* 8K, the max size of data we can read in a transaction(S--->P) */
	.write_size = 16, /* 8K the max size of data we can write in a transaction(S--->P) */
	.lock = SPIN_LOCK_UNLOCKED,
	.timing = &i2c_tm_default,
};

void i2c_open(void)
{
	i2c_ctrl_set_clk(i2c_dev_default.timing->clk);
	i2c_ctrl_enable();
}
EXPORT_SYMBOL(i2c_open);

void i2c_close(void)
{
	i2c_ctrl_disable();
}
EXPORT_SYMBOL(i2c_close);

void i2c_setclk(unsigned int i2cclk)
{
	i2c_ctrl_set_clk(i2cclk);
}
EXPORT_SYMBOL(i2c_setclk);

int i2c_lseek(unsigned char device, unsigned char offset)
{
#if 0
	int ret = 0;
	spin_lock(&i2c_dev_default.lock); /* maybe semaphore is more suitable!!!!! */

	__i2c_send_nack();	/* Master does not send ACK, slave sends it */
	__i2c_send_start();
	if (i2c_put_data(device << 1) < 0)
		goto device_err;
	if (i2c_put_data(offset) < 0)
		goto address_err;

	spin_unlock(&i2c_dev_default.lock);
	return 0;

 device_err:
	printk(KERN_DEBUG "No I2C device (0x%02x) installed.\n", device);
	__i2c_send_stop();
	spin_unlock(&i2c_dev_default.lock);
	return -ENODEV;
 address_err:
	printk(KERN_DEBUG "No I2C device (0x%02x) response.\n", device);
	__i2c_send_stop();
	spin_unlock(&i2c_dev_default.lock);
	return -EREMOTEIO;
#else
	return 0;
#endif
}
EXPORT_SYMBOL(i2c_lseek);

int i2c_read(unsigned char device, unsigned char *buf,
	       unsigned char address, int count)
{
	int ret = 0;

#ifdef DEBUG_I2C
	printk("device = %#0x, address = %#0x\n",
	       device, address);
#endif

	spin_lock(&i2c_dev_default.lock); /* maybe semaphore is more suitable!!!!! */
	/* device ----> address
	 * address ---> offset
	 */

	i2c_dev_default.address = device;
	ret = i_i2c_read_dev(&i2c_dev_default, address, buf, count);

	spin_unlock(&i2c_dev_default.lock);

	return ret;
}
EXPORT_SYMBOL(i2c_read);

int i2c_write(unsigned char device, unsigned char *buf,
		unsigned char address, int count)
{
	int ret = 0;

#ifdef DEBUG_I2C
	printk("device = %#0x, address = %#0x\n",
	       device, address);
#endif

	spin_lock(&i2c_dev_default.lock); /* maybe semaphore is more suitable!!!!! */

	i2c_dev_default.address = device;
	ret = i_i2c_write_dev(&i2c_dev_default, address, buf, count);

	spin_unlock(&i2c_dev_default.lock);

	return ret;
}
EXPORT_SYMBOL(i2c_write);

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
