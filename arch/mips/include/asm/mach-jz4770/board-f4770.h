/*
 *  linux arch/mips/include/asm/mach-jz4770/board-f4770.h
 *
 *  JZ4770-based F4770 board ver 1.x definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4770_F4770_H__
#define __ASM_JZ4770_F4770_H__

//#define CONFIG_FPGA

/*======================================================================
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		12000000  /* Main extal freq:	12 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq:	32.768 KHz */
//#define CFG_DIV		        2	/* cpu/extclk; only for FPGA */

/*======================================================================
 * GPIO
 */
#define OTG_HOTPLUG_PIN		(32 + 5)
#define GPIO_OTG_ID_PIN         (32*5+18)
#define OTG_HOTPLUG_IRQ         (IRQ_GPIO_0 + OTG_HOTPLUG_PIN)
#define GPIO_OTG_ID_IRQ         (IRQ_GPIO_0 + GPIO_OTG_ID_PIN)
#define GPIO_OTG_STABLE_JIFFIES 10



#define GPIO_I2C1_SDA           (32*3+30)       /* GPE14 */
#define GPIO_I2C1_SCK           (32*3+31)       /* GPE17 */


#define GPIO_SD0_VCC_EN_N	GPF(20)
#define GPIO_SD0_CD_N		GPA(25)
#define GPIO_SD0_WP_N		GPA(26)
#define GPIO_SD1_VCC_EN_N	GPE(9)
#define GPIO_SD1_CD_N		GPB(2)
#define GPIO_SD1_WP_N		GPB(3)

#define ACTIVE_LOW_MSC0_CD	1
#define ACTIVE_LOW_MSC1_CD	1

#define MSC0_WP_PIN		GPIO_SD0_WP_N
#define MSC0_HOTPLUG_PIN	GPIO_SD0_CD_N
#define MSC0_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD0_CD_N)

#define MSC1_WP_PIN		GPIO_SD1_WP_N
#define MSC1_HOTPLUG_PIN	GPIO_SD1_CD_N
#define MSC1_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD1_CD_N)

#define GPIO_USB_DETE		102 /* GPD6 */
#define GPIO_DC_DETE_N		103 /* GPD7 */
#define GPIO_CHARG_STAT_N	111 /* GPD15 */
#define GPIO_DISP_OFF_N		121 /* GPD25, LCD_REV */
//#define GPIO_LED_EN       	124 /* GPD28 */

#define GPIO_UDC_HOTPLUG	GPIO_USB_DETE

#define GPIO_POWER_ON           (32 * 0 + 30)  /* GPA30 */
/*======================================================================
 * LCD backlight
 */

#define GPIO_LCD_PWM   		(32*4+1) /* GPE4 PWM4 */
#define LCD_PWM_CHN 1    /* pwm channel */
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

#define ACTIVE_LOW_WAKE_UP 	1

#if 1
/* use uart2 as default */
#define JZ_BOOTUP_UART_TXD	(32 * 2 + 30)
#define JZ_BOOTUP_UART_RXD	(32 * 2 + 28)
#define JZ_EARLY_UART_BASE	UART2_BASE
#else
#define JZ_BOOTUP_UART_TXD	(32 * 4 + 5)
#define JZ_BOOTUP_UART_RXD	(32 * 3 + 12)
#define JZ_EARLY_UART_BASE	UART3_BASE
#endif

#endif /* __ASM_JZ4770_F4770_H__ */
