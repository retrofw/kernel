/*
 *  linux arch/mips/include/asm/mach-jz4810/board-f4810.h
 *
 *  JZ4810-based F4810 board ver 1.x definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4810_F4810_H__
#define __ASM_JZ4810_F4810_H__

#define CONFIG_FPGA

/*======================================================================
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		24000000  /* Main extal freq: 12 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq: 32.768 KHz */
#define CFG_DIV		        2	/* cpu/extclk; only for FPGA */

/*======================================================================
 * GPIO
 */
#define GPIO_SD0_VCC_EN_N	113 /* GPD17 */
#define GPIO_SD0_CD_N		110 /* GPD14 */
#define GPIO_SD0_WP		112 /* GPD16 */

#define GPIO_USB_DETE		102 /* GPD6 */
#define GPIO_DC_DETE_N		103 /* GPD7 */
#define GPIO_CHARG_STAT_N	111 /* GPD15 */
#define GPIO_DISP_OFF_N		121 /* GPD25, LCD_REV */
//#define GPIO_LED_EN       	124 /* GPD28 */

#define GPIO_UDC_HOTPLUG	GPIO_USB_DETE

/*======================================================================
 * LCD backlight
 */

#define GPIO_LCD_PWM   		(32*4+4) /* GPE4 PWM4 */
#define LCD_PWM_CHN 4    /* pwm channel */
#define LCD_PWM_FULL 101
#define LCD_DEFAULT_BACKLIGHT		80
#define LCD_MAX_BACKLIGHT		100
#define LCD_MIN_BACKLIGHT		1

/* 100 level: 0,1,...,100 */

#define __lcd_set_backlight_level(n)	\
do {					\
	__gpio_as_output1(GPIO_LCD_PWM);	\
} while (0)

#define __lcd_close_backlight()		\
do {					\
	__gpio_as_output0(GPIO_LCD_PWM);	\
} while (0)

/*======================================================================
 * MMC/SD
 */

#define MSC0_WP_PIN		GPIO_SD0_WP
#define MSC0_HOTPLUG_PIN		GPIO_SD0_CD_N
#define MSC0_HOTPLUG_IRQ		(IRQ_GPIO_0 + GPIO_SD0_CD_N)

#if 0
#define __msc0_init_io()				\
do {						\
	__gpio_as_output1(GPIO_SD0_VCC_EN_N);	\
	__gpio_as_input(GPIO_SD0_CD_N);		\
} while (0)
#endif
#define __msc0_init_io()				\
do {						\
	__gpio_as_input(GPIO_SD0_CD_N);		\
} while (0)

#define __msc0_enable_power()			\
do {						\
	__gpio_as_output0(GPIO_SD0_VCC_EN_N);	\
} while (0)

#define __msc0_disable_power()			\
do {						\
	__gpio_as_output1(GPIO_SD0_VCC_EN_N);	\
} while (0)

#define __msc0_card_detected(s)			\
({						\
	int detected = 1;			\
	if (__gpio_get_pin(GPIO_SD0_CD_N))	\
		detected = 0;			\
	detected;				\
})

#if 0
#define __msc_init_io()				\
do {						\
	__gpio_as_output(GPIO_SD_VCC_EN_N);	\
	__gpio_as_input(GPIO_SD_CD_N);		\
} while (0)

#define __msc_enable_power()			\
do {						\
	__gpio_clear_pin(GPIO_SD_VCC_EN_N);	\
} while (0)

#define __msc_disable_power()			\
do {						\
	__gpio_set_pin(GPIO_SD_VCC_EN_N);	\
} while (0)

#define __msc_card_detected(s)			\
({						\
	int detected = 1;			\
	if (__gpio_get_pin(GPIO_SD_CD_N))	\
		detected = 0;			\
	detected;				\
})
#endif
#define ACTIVE_LOW_MSC0_CD	1
#define ACTIVE_LOW_MSC1_CD	1

#define JZ_BOOTUP_UART_TXD	(32 * 2 + 30)
#define JZ_BOOTUP_UART_RXD	(32 * 2 + 28)
#define JZ_EARLY_UART_BASE	UART2_BASE

#endif /* __ASM_JZ4810_F4810_H__ */
