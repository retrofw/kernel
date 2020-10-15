/*
 * linux/drivers/misc/jz_sensor.h -- Ingenic CIM driver
 *
 * Copyright (C) 2005-2010, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#ifndef __JZ_SENSOR_H__
#define __JZ_SENSOR_H__

#include<linux/i2c.h>

extern void sensor_set_i2c_speed(struct i2c_client *client,unsigned long speed);
extern int sensor_write_reg(struct i2c_client *client,unsigned char reg, unsigned char val);
extern int sensor_write_reg16(struct i2c_client *client,unsigned short reg, unsigned char val);
extern unsigned char sensor_read_reg(struct i2c_client *client,unsigned char reg);
extern char sensor_read_reg_nostop(struct i2c_client *client,unsigned char reg);
extern unsigned char sensor_read_reg16(struct i2c_client *client,unsigned short reg);

#endif
