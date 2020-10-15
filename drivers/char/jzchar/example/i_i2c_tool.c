/*
 * JZ4750 Simple I2C Userspace Example.
 *
 * Copyright (c) 2005-2010  Ingenic Semiconductor Inc.
 * Author: River <zwang@ingenic.cn>
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "../i_i2c_abi.h"

#define I2C_DEV "/dev/i_i2c"
#define SZ_BUF	2048

int main(int argc, char **argv)
{
	struct i_i2c_control control;
	
	unsigned char buf[SZ_BUF];
	
	int fd;
	int count = 64;

	int i;
	int rv;

	fd = open(I2C_DEV, O_RDWR);
	if (fd == -1) {
		perror("open():");
		exit(EXIT_FAILURE);
	}
	
	control.id = I_I2C_ID_AT24C16B;
	control.offset = 0;
	control.buf = buf;
	control.count = count;
	
	rv = ioctl(fd, I_I2C_IOC_READ_DEV, &control);
	if (rv)	{
		perror("ioctl():");
		exit(EXIT_FAILURE);
	}
	
	for (i = 0; i < count; i++)
		fprintf(stderr, "%x ", buf[i]);

	fprintf(stderr, "\n", buf[i]);
	
	if (argc != 1)
		for (i = 0; i < count; i++)
			buf[i] = i;
	else
		for (i = 0; i < count; i++)
			buf[i] = 0;

	control.id = I_I2C_ID_AT24C16B;
	control.offset = 0;
	control.buf = buf;
	control.count = count;

	rv = ioctl(fd, I_I2C_IOC_WRITE_DEV, &control);	
	if (rv)	{
		perror("ioctl():");
		exit(EXIT_FAILURE);
	}
	
	close(fd);

	return 0;
}
