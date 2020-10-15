/*
 * linux/arch/mips/jz4730/board-pmp.c
 *
 * JZ4730 PMP board setup routines.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
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
	static unsigned int count = 0;

	count ++;
	count &= 1;
	if (count)
		__gpio_set_pin(GPIO_LED_EN);
	else
		__gpio_clear_pin(GPIO_LED_EN);
}

static void pmp_timer_ack(void)
{
	static unsigned int count = 0;
	count ++;
	if (count % 100 == 0) {
		count = 0;
		dancing();
	}
}

static void __init board_cpm_setup(void)
{
	__cpm_start_all();
}

static void __init board_gpio_setup(void)
{
	/*
	 * Most of the gpios have been setup in the bootloader.
	 */

	__harb_usb0_uhc();
	__gpio_as_dma();
	__gpio_as_eth();
	__gpio_as_usb();
	__gpio_as_lcd_master();
#if defined(CONFIG_I2S_AK4642EN)
	__gpio_as_scc1();
#endif
#if defined(CONFIG_I2S_TSC2301) || defined(CONFIG_I2S_TLC320AIC23)
	__gpio_as_ssi();
#endif
	//__gpio_as_ac97();
#if defined(CONFIG_I2S_TSC2301) || defined(CONFIG_I2S_TLC320AIC23) || defined(CONFIG_I2S_CS42L51)
	__gpio_as_i2s_slave();
#endif
	__gpio_as_cim();
	__gpio_as_msc();

	__gpio_as_output(GPIO_LED_EN);
	__gpio_set_pin(GPIO_LED_EN);

	 __gpio_as_output(GPIO_DISP_OFF_N);
	__gpio_set_pin(GPIO_DISP_OFF_N);
	__gpio_as_output(GPIO_PWM0);
	__gpio_set_pin(GPIO_PWM0);

	__gpio_as_input(GPIO_RTC_IRQ);
	__gpio_as_output(GPIO_USB_CLK_EN);
	__gpio_set_pin(GPIO_USB_CLK_EN);

	__gpio_as_input(GPIO_CHARG_STAT);
	__gpio_disable_pull(GPIO_CHARG_STAT);

        __gpio_as_input(GPIO_UDC_HOTPLUG);
        __gpio_disable_pull(GPIO_UDC_HOTPLUG);
        __gpio_disable_pull(54); /* fixed ic bug, the pull of gpio pin 86 is as pin 54 */
}

void __init jz_board_setup(void)
{
	printk("JZ4730 PMP board setup\n");

	board_cpm_setup();
	board_gpio_setup();

	jz_timer_callback = pmp_timer_ack;
}
