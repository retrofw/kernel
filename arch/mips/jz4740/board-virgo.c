/*
 * linux/arch/mips/jz4740/board-virgo.c
 *
 * JZ4720 VIRGO board setup routines.
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
	static unsigned int count = 0;

	count ++;
	count &= 1;
	if (count)
		__gpio_set_pin(GPIO_LED_EN);
	else
		__gpio_clear_pin(GPIO_LED_EN);
}

static void virgo_timer_callback(void)
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
	 * We have started all module clocks at arch/mips/jz4740/setup.c.
	 */
}

static void __init board_gpio_setup(void)
{
	/*
	 * Most of the GPIO pins should have been initialized by the boot-loader
	 */

	/*
	 * Initialize MSC pins
	 */
	__gpio_as_msc();

	/*
	 * Initialize LCD pins
	 */
//	__gpio_as_lcd_18bit();

	/*
	 * Initialize SSI pins
	 */
	__gpio_as_ssi();

	/*
	 * Initialize I2C pins
	 */
	__gpio_as_i2c();

	/*
	 * Initialize Other pins
	 */
	__gpio_as_output(GPIO_SD_VCC_EN_N);
	__gpio_clear_pin(GPIO_SD_VCC_EN_N);

	__gpio_as_input(GPIO_SD_CD_N);
	__gpio_disable_pull(GPIO_SD_CD_N);

//	__gpio_as_input(GPIO_SD_WP);
//	__gpio_disable_pull(GPIO_SD_WP);

	__gpio_as_input(GPIO_DC_DETE_N);
//	__gpio_as_input(GPIO_CHARG_STAT_N);
	__gpio_as_input(GPIO_USB_DETE);

	__gpio_as_output(GPIO_DISP_OFF_N);

//	__gpio_as_output(GPIO_LED_EN);
}

void __init jz_board_setup(void)
{
	printk("JZ4720 VIRGO board setup\n");

	board_cpm_setup();
	board_gpio_setup();

	jz_timer_callback = virgo_timer_callback;
}
