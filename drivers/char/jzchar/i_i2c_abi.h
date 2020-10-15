/*
 * JZ4750 Simple I2C Userspace Driver ABI Definations.
 *
 * Copyright (c) 2005-2010  Ingenic Semiconductor Inc.
 * Author: River <zwang@ingenic.cn>
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */

#ifndef __I_I2C_ABI_H__
#define __I_I2C_ABI_H__

/* -------------- CUSTOM: Add your device/timing ID here. --------------*/
/* Device ID */
enum {
	I_I2C_ID_AT24C16B = 1,	/* ID must start from 1. */
};
/* ---------------------------------------------------------------------*/

struct i_i2c_control {
	int id;			/* Device ID. */
	off_t offset;		/* Offset. */
	void *buf;		/* IO buffer. */
	size_t count;		/* IO count. */
};

#define I_I2C_IOC_READ_DEV	_IOW('I', 1, struct i_i2c_control)
#define I_I2C_IOC_WRITE_DEV	_IOR('I', 2, struct i_i2c_control)

#endif
