/*
 * linux/arch/mips/jz4740/board-leo.c
 *
 * JZ4740 LEO board setup routines.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <lhhuang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <asm/jzsoc.h>

extern void (*jz_timer_callback)(void);

static void dancing(void)
{
	static unsigned char slash[] = "\\|/-";
	static volatile unsigned char *p = (unsigned char *)0xb6000016;
	static unsigned int count = 0;
	*p = slash[count++];
	count &= 3;
}

static void leo_timer_callback(void)
{
	static unsigned long count = 0;

	if ((++count) % 10 == 0) {
		dancing();
		count = 0;
	}
}

static void __init board_cpm_setup(void)
{
	/* Stop unused module clocks here.
	 * We have started all module clocks at arch/mips/jz4740/setup.c.
	 */
}

static void __init board_gpio_setup(void)
{
	/* All GPIO pins should have been initialized by the boot-loader */
}

void __init jz_board_setup(void)
{
	board_cpm_setup();
	board_gpio_setup();
	printk(" BOARD SETUP");
	jz_timer_callback = leo_timer_callback;
}
