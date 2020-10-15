/*
 *  linux/include/asm-mips/mach-jz4760b/board-f4760b.h
 *
 *  JZ4760B-based F4760b board ver 1.x definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4760B_F4760b_H__
#define __ASM_JZ4760B_F4760b_H__

#define CONFIG_FPGA	 /* fuwa is an FPGA board */

/*====================================================================== 
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		24000000  /* Main extal freq: 12 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq: 32.768 KHz */
#define CFG_DIV		2	/* cpu/extclk; only for FPGA */

/*====================================================================== 
 * GPIO
 */
#define GPIO_SD_VCC_EN_N	113 /* GPD17 */
#define GPIO_SD_CD_N		110 /* GPD14 */
#define GPIO_SD_WP		112 /* GPD16 */
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
/* 100 level: 0,1,...,100 */
#define __lcd_set_backlight_level(n)	\
do {					\
	__gpio_as_output(GPIO_LCD_PWM);	\
	__gpio_set_pin(GPIO_LCD_PWM);	\
} while (0)

#define __lcd_close_backlight()		\
do {					\
	__gpio_as_output(GPIO_LCD_PWM);	\
	__gpio_clear_pin(GPIO_LCD_PWM);	\
} while (0)

/*====================================================================== 
 * MMC/SD
 */

#define MSC_WP_PIN		GPIO_SD_WP
#define MSC_HOTPLUG_PIN		GPIO_SD_CD_N
#define MSC_HOTPLUG_IRQ		(IRQ_GPIO_0 + GPIO_SD_CD_N)

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

#endif /* __ASM_JZ4760B_F4760b_H__ */
