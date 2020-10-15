/*
 * linux/arch/mips/jz4730/reset.c
 *
 * JZ4730 reset routines.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/sched.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/processor.h>
#include <asm/reboot.h>
#include <asm/system.h>
#include <asm/jzsoc.h>

void jz_restart(char *command)
{
	__wdt_set_count(0xffffffff-32);	/* reset after 1/1024 s */
	__wdt_start();
	while (1);
}

void jz_halt(void)
{
	__wdt_set_count(0xffffffff-32);	/* reset after 1/1024 s */
	__wdt_start();
	while (1);
}

void jz_power_off(void)
{
	jz_halt();
}
