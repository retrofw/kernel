/*
 * linux/arch/mips/jz4750/board-apus.c
 *
 * JZ4750 APUS board setup routines.
 *
 * Copyright (c) 2006-2008  Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
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
}

static void apus_timer_callback(void)
{
	static unsigned long count = 0;

	if ((++count) % 50 == 0) {
		dancing();
		count = 0;
	}
}

static void __init board_cpm_setup(void)
{
	/* Stop unused module clocks here.
	 * We have started all module clocks at arch/mips/jz4750/setup.c.
	 */
}

static void __init board_gpio_setup(void)
{
	__gpio_as_pcm();
}

void __init jz_board_setup(void)
{
	printk("JZ4750 SLT_50 board setup\n");

	board_cpm_setup();
	board_gpio_setup();

	jz_timer_callback = apus_timer_callback;
}
