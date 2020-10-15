/*
 * linux/drivers/misc/jz_sensor.c
 *
 * Virtual device driver with tricky appoach to manage TCSM
 *
 * Copyright (C) 2006  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <asm/jzsoc.h>
#include <linux/i2c.h>
#include "jz_sensor.h"


static inline int sensor_i2c_master_recv(struct i2c_client *client, char *buf ,int count);
static inline int sensor_i2c_master_send(struct i2c_client *client,const char *buf ,int count);
extern void i2c_jz_setclk(struct i2c_client *client,unsigned long i2cclk);

void sensor_set_i2c_speed(struct i2c_client *client,unsigned long speed)
{
#if defined(CONFIG_SOC_JZ4760) || defined(CONFIG_JZ4760_F4760) || defined(CONFIG_JZ4810_F4810)
	i2c_jz_setclk(client,speed);
#endif
	//printk("set sensor i2c write read speed = %d hz\n",speed);
}

int sensor_write_reg(struct i2c_client *client,unsigned char reg, unsigned char val)
{
	unsigned char msg[2];
	int ret;

	memcpy(&msg[0], &reg, 1);
	memcpy(&msg[1], &val, 1);

	ret = sensor_i2c_master_send(client, msg, 2);

	if (ret < 0)
	{
		printk("RET<0\n");
		return ret;
	}
	if (ret < 2)
	{
		printk("RET<2\n");
		return -EIO;
	}

	return 0;
}

unsigned char sensor_read_reg(struct i2c_client *client,unsigned char reg)
{
	int ret;
	unsigned char retval;

	ret = sensor_i2c_master_send(client,&reg,1);

	if (ret < 0)
		return ret;
	if (ret != 1)
		return -EIO;

	ret = sensor_i2c_master_recv(client, &retval, 1);
	if (ret < 0) {
		printk("%s: ret < 0\n", __FUNCTION__);
		return ret;
	}
	if (ret != 1) {
		printk("%s: ret != 1\n", __FUNCTION__);
		return -EIO;
	}
	return retval;
}

char sensor_read_reg_nostop(struct i2c_client *client, unsigned char reg) {
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg[2];
	char tmp[2];

	memset(msg, 0, sizeof(struct i2c_msg) * 2);

	tmp[0] = reg;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = tmp;

	msg[1].addr = client->addr;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = tmp + 1;

	ret = i2c_transfer(adap, msg, 2);
	if (ret != 2)
		return -1;

	return tmp[1];
}

int sensor_write_reg16(struct i2c_client *client,unsigned short reg, unsigned char val)
{
	unsigned char msg[3],tmp;
	int ret;

	memcpy(&msg[0], &reg, 2);
	memcpy(&msg[2], &val, 1);

	tmp=msg[0];
	msg[0]=msg[1];
	msg[1]=tmp;

	ret = sensor_i2c_master_send(client, msg, 3);
	if (ret < 0)
	{
		printk("RET < 0\n");
		return ret;
	}
	if (ret < 3)
	{
		printk("RET<3\n");
		return -EIO;
	}

	return 0;
}


unsigned char sensor_read_reg16(struct i2c_client *client,unsigned short reg)
{
	unsigned char retval;
	unsigned char msg[2],tmp;
	int ret;

	memcpy(&msg[0], &reg, 2);

	tmp=msg[0];
	msg[0]=msg[1];
	msg[1]=tmp;


	ret = sensor_i2c_master_send(client,msg,2);

	if (ret < 0) {
		printk("%s: ret < 0\n", __FUNCTION__);
		return ret;
	}

	if (ret != 2) {
		printk("%s: ret != 2\n", __FUNCTION__);
		return -EIO;
	}

	ret = sensor_i2c_master_recv(client, &retval, 1);
	if (ret < 0) {
		printk("%s: ret < 0\n", __FUNCTION__);
		return ret;
	}
	if (ret != 1) {
		printk("%s: ret != 1\n", __FUNCTION__);
		return -EIO;
	}

	//printk("read value = %x\n",(unsigned int)retval);
	return retval;
}

static inline int sensor_i2c_master_send(struct i2c_client *client,const char *buf ,int count)
{
	int ret;
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (char *)buf;
	ret = i2c_transfer(adap, &msg, 1);

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	   transmitted, else error code. */
	return (ret == 1) ? count : ret;
}

static inline int sensor_i2c_master_recv(struct i2c_client *client, char *buf ,int count)
{
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;
	int ret;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = buf;
	ret = i2c_transfer(adap, &msg, 1);

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	   transmitted, else error code. */
	return (ret == 1) ? count : ret;
}


