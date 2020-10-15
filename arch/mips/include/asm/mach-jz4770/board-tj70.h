/*
 *  linux arch/mips/include/asm/mach-jz4770/board-tj70.h
 *
 *  JZ4770-based tj70 board ver 1.x definition.
 *
 *  Copyright (C) 2008 - 2012 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *  Author: Kage <kkshen@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4770_TJ70_H__
#define __ASM_JZ4770_TJ70_H__

//#define CONFIG_FPGA

/*======================================================================
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		12000000  /* Main extal freq:	12 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq:	32.768 KHz */

/*======================================================================
 * GPIO
 */
#define JZMAC_PHY_RESET_PIN		GPE(8)

#define OTG_HOTPLUG_PIN         (32 + 5)
#define GPIO_OTG_ID_PIN         (32*5+18)
#define OTG_HOTPLUG_IRQ         (IRQ_GPIO_0 + OTG_HOTPLUG_PIN)
#define GPIO_OTG_ID_IRQ         (IRQ_GPIO_0 + GPIO_OTG_ID_PIN)
#define GPIO_OTG_STABLE_JIFFIES 10


#define GPIO_I2C1_SDA           (32*4+0)       /* GPE0 */
#define GPIO_I2C1_SCK           (32*4+17)       /* GPE17 */

#define EP932M_RESET_PIN        (32*4+6)

#define GPIO_SD0_VCC_EN_N	GPF(20)
#define GPIO_SD0_CD_N		GPA(25)
#define GPIO_SD0_WP_N		GPA(26)
#define GPIO_SD1_VCC_EN_N	GPE(9)
#define GPIO_SD1_CD_N		GPB(2)

#define MSC0_WP_PIN			GPIO_SD0_WP_N
#define MSC0_HOTPLUG_PIN	GPIO_SD0_CD_N
#define MSC0_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD0_CD_N)

#define MSC1_HOTPLUG_PIN	GPIO_SD1_CD_N
#define MSC1_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD1_CD_N)

#define GPIO_USB_DETE		102 /* GPD6 */
#define GPIO_DC_DETE_N		103 /* GPD7 */
#define GPIO_CHARG_STAT_N	111 /* GPD15 */
#define GPIO_DISP_OFF_N		121 /* GPD25, LCD_REV */
//#define GPIO_LED_EN       	124 /* GPD28 */

#define GPIO_UDC_HOTPLUG		GPIO_USB_DETE

#define GPIO_POWER_ON           (32 * 0 + 30)  /* GPA30 */


#define GPIO_TS_I2C_INT         GPE(0)
#define GPIO_TS_I2C_IRQ         (IRQ_GPIO_0 + GPIO_TS_I2C_INT)

/*====================================================================
 * GPIO KEYS and ADKEYS (GPIO_WAKEUP used for end call)
 */
#define GPIO_HOME		(32 * 2 + 31)	// SW3-GPC31
#define GPIO_MENU		(32 * 3 + 18)	// SW6-boot_sel1-GPD18
#define GPIO_CALL		(32 * 2 + 29)	// SW1-GPC29
#define GPIO_BACK		(32 * 3 + 9)	// SW4-GPD9
#define GPIO_ENDCALL    (32 * 0 + 30)	// WAKEUP-GPA30
#define GPIO_VOLUMEDOWN (32 * 3 + 17)	// SW7-boot_sel0-GPD17
#define GPIO_VOLUMEUP   (32 * 3 + 19)	// SW5-boot_sel2-GPD19

/*======================================================================
 * LCD backlight
 */

#define GPIO_LCD_PWM   		(32*4+1) /* GPE4 PWM4 */
#define LCD_PWM_CHN			1    /* pwm channel */
#define LCD_PWM_FULL		101
#define LCD_DEFAULT_BACKLIGHT		80
#define LCD_MAX_BACKLIGHT			100
#define LCD_MIN_BACKLIGHT			1
#define PWM_BACKLIGHT_CHIP			1	/*0: digital pusle; 1: PWM*/

/* 100 level: 0,1,...,100 */

#if PWM_BACKLIGHT_CHIP

#define __lcd_init_backlight(n)					\
do {											\
	__lcd_set_backlight_level(n);				\
} while (0)

/* 100 level: 0,1,...,100 */
#define __lcd_set_backlight_level(n)				\
do {								\
	__gpio_as_pwm(1);                               \
	__tcu_disable_pwm_output(LCD_PWM_CHN);			\
	__tcu_stop_counter(LCD_PWM_CHN);			\
	__tcu_init_pwm_output_high(LCD_PWM_CHN);		\
	__tcu_set_pwm_output_shutdown_abrupt(LCD_PWM_CHN);	\
	__tcu_select_clk_div1(LCD_PWM_CHN);			\
	__tcu_mask_full_match_irq(LCD_PWM_CHN);			\
	__tcu_mask_half_match_irq(LCD_PWM_CHN);			\
	__tcu_clear_counter_to_zero(LCD_PWM_CHN);		\
	__tcu_set_full_data(LCD_PWM_CHN, JZ_EXTAL / 30000);	\
	__tcu_set_half_data(LCD_PWM_CHN, JZ_EXTAL / 30000 * n / LCD_PWM_FULL); \
	__tcu_enable_pwm_output(LCD_PWM_CHN);			\
	__tcu_select_extalclk(LCD_PWM_CHN);			\
	__tcu_start_counter(LCD_PWM_CHN);			\
} while (0)

#define __lcd_close_backlight()					\
do {								\
	__gpio_as_output0(GPIO_LCD_PWM);				\
} while (0)

#else	/* PWM_BACKLIGHT_CHIP */

#define __send_low_pulse(n)					\
do {								\
	unsigned int i;						\
	for (i = n; i > 0; i--)	{				\
		__gpio_as_output0(GPIO_LCD_PWM);		\
		udelay(1);					\
		__gpio_as_output1(GPIO_LCD_PWM);		\
		udelay(3);					\
	}							\
} while (0)

#define MAX_BRIGHTNESS_STEP	16				/* RT9365 supports 16 brightness step */
#define CONVERT_FACTOR		(256/MAX_BRIGHTNESS_STEP)	/* System support 256 brightness step */

#define __lcd_init_backlight(n)					\
do {								\
	unsigned int tmp = (n)/CONVERT_FACTOR + 1;		\
	__gpio_as_output1(GPIO_LCD_PWM);			\
	udelay(30);						\
	__send_low_pulse(MAX_BRIGHTNESS_STEP-tmp);		\
} while (0)

#define __lcd_set_backlight_level(n)					\
do {									\
	unsigned int last = lcd_backlight_level / CONVERT_FACTOR + 1;	\
	unsigned int tmp = (n) / CONVERT_FACTOR + 1;			\
	if (tmp <= last) {						\
		__send_low_pulse(last-tmp);				\
	} else {							\
		__send_low_pulse(last + MAX_BRIGHTNESS_STEP - tmp);	\
	}								\
	udelay(30);							\
} while (0)

#define __lcd_close_backlight()					\
do {								\
	__gpio_as_output0(GPIO_LCD_PWM); 			\
} while (0)

#endif	/*PWM_BACKLIGHT_CHIP*/

/*
 * The key interrupt pin is low voltage or fall edge acitve
 */
#define ACTIVE_LOW_HOME		1
#define ACTIVE_LOW_MENU		1
#define ACTIVE_LOW_BACK		1
#define ACTIVE_LOW_CALL		1
#define ACTIVE_LOW_ENDCALL  1
#define ACTIVE_LOW_VOLUMEDOWN   1
#define ACTIVE_LOW_VOLUMEUP     1
#define ACTIVE_LOW_ADKEY	1
#define ACTIVE_LOW_WAKE_UP 	1
#define ACTIVE_LOW_MSC0_CD	1
#define ACTIVE_LOW_MSC1_CD	1

/* use uart2 as default */
#define JZ_BOOTUP_UART_TXD	(32 * 2 + 30)
#define JZ_BOOTUP_UART_RXD	(32 * 2 + 28)
#define JZ_EARLY_UART_BASE	UART2_BASE

#endif /* __ASM_JZ4770_TJ70_H__ */
