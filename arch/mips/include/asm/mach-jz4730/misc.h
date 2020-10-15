/*
 *  linux/include/asm-mips/mach-jz4730/misc.h
 *
 *  JZ4730 miscillaneous definitions.
 *
 *  Copyright (C) 2006 - 2007 Ingenic Semiconductor Inc.
 *
 *  Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4730_MISC_H__
#define __ASM_JZ4730_MISC_H__

/*
 * I2C routines
 */

extern void i2c_open(void);
extern void i2c_close(void);
extern void i2c_setclk(unsigned int i2cclk);
extern int i2c_read(unsigned char, unsigned char *, unsigned char, int);
extern int i2c_write(unsigned char, unsigned char *, unsigned char, int);

#endif /* __ASM_JZ4730_MISC_H__ */
