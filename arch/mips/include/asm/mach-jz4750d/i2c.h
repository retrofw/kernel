/*
 * linux/arch/mips/jz4750/i2c.c
 *
 * JZ4750 I2C Simple Driver.
 *
 * Copyright (c) 2005-2010  Ingenic Semiconductor Inc.
 * Author: River <zwang@ingenic.cn>
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */

#ifndef __I_I2C_H__
#define __I_I2C_H__

enum {
	I_I2C_IO_DIR_READ = 0,
	I_I2C_IO_DIR_WRITE,	
};

enum {
	I_I2C_CAP_SEQ_READ = (1 << 0),
	I_I2C_CAP_SEQ_WRITE = (1 << 1),
	I_I2C_CAP_16BIT_OFFSET_MSB = (1 << 2),
	I_I2C_CAP_16BIT_OFFSET_LSB = (1 << 3),
};

enum {
	I_I2C_FLAG_STOP_BEFORE_RESTART = (1 << 0),
};

struct i_i2c_timing {
	int id;

	unsigned long clk;
	unsigned long timeout;

	unsigned long t_wr;
};

struct i_i2c_dev {
	int id;
	
	char *name;
	
	unsigned int address;

	spinlock_t lock;

	unsigned long cap;
	unsigned long flags;

	unsigned long size;

	unsigned int read_size;
	unsigned int write_size;
	
	int timing_id;
	struct i_i2c_timing *timing;
};

int i_i2c_read_dev(struct i_i2c_dev *dev, off_t off, void *buf, size_t count);

int i_i2c_write_dev(struct i_i2c_dev *dev, off_t off, void *buf, size_t count);

int i_i2c_init_dev(struct i_i2c_dev *dev);

#endif
