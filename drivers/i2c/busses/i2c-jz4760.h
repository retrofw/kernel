/*
 *  i2c_jz47xx.h
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef _I2C_JZ_H_
#define _I2C_JZ_H_

struct i2c_slave_client;

struct i2c_jz_platform_data {
	unsigned int		slave_addr;
	struct i2c_slave_client	*slave;
	unsigned int		class;
};

extern void jz_set_i2c_info(struct i2c_jz_platform_data *info);
#endif
