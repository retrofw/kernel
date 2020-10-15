/*
 * linux/arch/mips/jz4760/board-f4760.c
 *
 * JZ4760 F4760 board setup routines.
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
#include <linux/i2c.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <asm/jzsoc.h>


#define WM831X_LDO_MAX_NAME 6

extern void (*jz_timer_callback)(void);

static void dancing(void)
{
	static unsigned char slash[] = "\\|/-";
//	static volatile unsigned char *p = (unsigned char *)0xb6000058;
	static volatile unsigned char *p = (unsigned char *)0xb6000016;
	static unsigned int count = 0;
	*p = slash[count++];
	count &= 3;
}

static void f4810_timer_callback(void)
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
	 * We have started all module clocks at arch/mips/jz4760/setup.c.
	 */
}

static void __init board_gpio_setup(void)
{
	/*
	 * Initialize SDRAM pins
	 */
}

static struct i2c_board_info falcon_i2c0_devs[] __initdata = {
        {
                I2C_BOARD_INFO("cm3511", 0x30),
        },
        {
                I2C_BOARD_INFO("ov3640", 0x3c),
        },
        {
                I2C_BOARD_INFO("ov7690", 0x21),
        },
        {
        },
};

void __init board_i2c_init(void) {
        i2c_register_board_info(0, falcon_i2c0_devs, ARRAY_SIZE(falcon_i2c0_devs));
}

void __init jz_board_setup(void)
{

	printk("JZ4810 F4810 board setup\n");
//	jz_restart(NULL);
	board_cpm_setup();
	board_gpio_setup();

	jz_timer_callback = f4810_timer_callback;
}

/**
 * Called by arch/mips/kernel/proc.c when 'cat /proc/cpuinfo'.
 * Android requires the 'Hardware:' field in cpuinfo to setup the init.%hardware%.rc.
 */
const char *get_board_type(void)
{
	return "f4810";
}

/*****
 * Wm831x init
 *****/
