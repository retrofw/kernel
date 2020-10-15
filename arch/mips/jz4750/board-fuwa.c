/*
 * linux/arch/mips/jz4750/board-fuwa.c
 *
 * JZ4750 FUWA board setup routines.
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
	static unsigned char slash[] = "\\|/-";
//	static volatile unsigned char *p = (unsigned char *)0xb6000058;
	static volatile unsigned char *p = (unsigned char *)0xb6000016;
	static unsigned int count = 0;
	*p = slash[count++];
	count &= 3;
}

static void fuwa_timer_callback(void)
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
	/*
	 * Initialize SDRAM pins
	 */

	/* PORT A: D0 ~ D31 */
	REG_GPIO_PXFUNS(0) = 0xffffffff;
	REG_GPIO_PXSELC(0) = 0xffffffff;

	/* PORT B: A0 ~ A16, DCS#, RAS#, CAS#, CKE#, RDWE#, CKO#, WE0# */
	REG_GPIO_PXFUNS(1) = 0x81f9ffff;
	REG_GPIO_PXSELC(1) = 0x81f9ffff;

	/* PORT C: WE1#, WE2#, WE3# */
	REG_GPIO_PXFUNS(2) = 0x07000000;
	REG_GPIO_PXSELC(2) = 0x07000000;


	/*
	 * Initialize UART0 pins
	 */

	/* PORT D: TXD/RXD */
	REG_GPIO_PXFUNS(3) = 0x06000000;
	REG_GPIO_PXSELS(3) = 0x06000000;


	/*
	 * Initialize LED pins
	 */
	__gpio_as_lcd_18bit();

	/* CS2# */
	REG_GPIO_PXFUNS(1) = 0x04000000;
	REG_GPIO_PXSELC(1) = 0x04000000;
	
	__gpio_as_pcm();
}

void __init jz_board_setup(void)
{
	printk("JZ4750 FUWA board setup\n");

	board_cpm_setup();
	board_gpio_setup();

	jz_timer_callback = fuwa_timer_callback;
}
