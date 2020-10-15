/*
 * linux/include/asm-mips/mach-jz4760/board-lepus.h
 *
 * JZ4760-based LEPUS board ver 1.0 definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Inc.
 *
 * Author: James<ljia@ingenic.cn>
 *		Based on board-cygnus.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4760_LEPUS_H__
#define __ASM_JZ4760_LEPUS_H__

#define __GPIO(p, n) (32 * (p - 'A') + n)

#define UNUSED_GPIO_PIN		(0xffffffff)

#define GPIO_POWER_ON		__GPIO('A', 30)
#define ACTIVE_LOW_WAKE_UP	1

/*======================================================================
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL			12000000  /* Main extal freq:	12 MHz */
#define JZ_EXTAL2			32768     /* RTC extal freq:	32.768 KHz */

#define GPIO_SD0_VCC_EN_N	0 // boot card
#define GPIO_SD0_CD_N		UNUSED_GPIO_PIN
#define GPIO_SD2_VCC_EN_N	__GPIO('F', 3) //tf
#define GPIO_SD2_CD_N		__GPIO('F', 0)

#define GPIO_SD1_VCC_EN_N	UNUSED_GPIO_PIN
#define GPIO_SD1_CD_N		UNUSED_GPIO_PIN

/*====================================================================
 *  ADKEYS LEVEL
 */
#define DPAD_LEFT_LEVEL		186 // 0.15V, 186=0.15/3.3*4096
#define DPAD_DOWN_LEVEL		2482 // 2.0V
#define DPAD_UP_LEVEL		1985 // 1.6V
#define DPAD_CENTER_LEVEL	1489 // 1.2V
#define DPAD_RIGHT_LEVEL	868 // 0.7V

//battery detect
//#define CHARGE_DET		__GPIO('E', 13)
//#define GPIO_TS_I2C_INT	__GPIO('D', 4)
//#define GPIO_TS_I2C_IRQ	(IRQ_GPIO_0 + GPIO_TS_I2C_INT)

#define GPIO_USB_DETE		__GPIO('D', 7) //__GPIO('B', 21)
#define UDC_HOTPLUG_PIN		GPIO_USB_DETE

#define OTG_HOTPLUG_PIN		GPIO_USB_DETE
#define OTG_HOTPLUG_IRQ		(IRQ_GPIO_0 + OTG_HOTPLUG_PIN)
#define GPIO_OTG_ID_PIN		__GPIO('A', 11)
#define GPIO_OTG_ID_IRQ		(IRQ_GPIO_0 + GPIO_OTG_ID_PIN)
//#define GPIO_OTG_DRVVBUS	__GPIO('E', 10)
#define GPIO_OTG_STABLE_JIFFIES	10
//#define GPIO_OTG_POWER_CTRL	__GPIO('B', 29)

/*======================================================================
 * MMC/SD
 */
//#define MSC0_WP_PIN		GPIO_SD0_WP_N
#define MSC0_HOTPLUG_PIN	GPIO_SD0_CD_N
#define MSC0_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD0_CD_N)

//#define MSC1_WP_PIN		GPIO_SD1_WP_N
#define MSC2_HOTPLUG_PIN	GPIO_SD2_CD_N
#define MSC2_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD2_CD_N)
#define ACTIVE_LOW_MSC0_CD	1
#define ACTIVE_LOW_MSC2_CD	1
#define MSC1_HOTPLUG_PIN	GPIO_SD1_CD_N
#define MSC1_HOTPLUG_IRQ	(IRQ_GPIO_0 + MSC1_HOTPLUG_PIN)
#define ACTIVE_LOW_MSC1_CD	1

#define __msc0_enable_power() \
do { \
	__gpio_clear_pin(GPIO_SD0_VCC_EN_N); \
} while (0)

#define __msc0_disable_power() \
do { \
	__gpio_set_pin(GPIO_SD0_VCC_EN_N); \
} while (0)

#define __msc2_enable_power() \
do { \
	__gpio_clear_pin(GPIO_SD2_VCC_EN_N); \
} while (0)

#define __msc2_disable_power() \
do { \
	__gpio_set_pin(GPIO_SD2_VCC_EN_N); \
} while (0)

#define __msc1_enable_power() \
do { \
	__gpio_clear_pin(GPIO_SD1_VCC_EN_N); \
} while (0)

#define __msc1_disable_power() \
do { \
	__gpio_set_pin(GPIO_SD1_VCC_EN_N); \
} while (0)

/*======================================================================
 * SPI
 */
#define GPIO_SSI0_CE0			__GPIO('B', 29)
#define GPIO_SSI0_CE1			__GPIO('B', 31)
#define GPIO_SSI0_GPC			__GPIO('B', 30)
#define GPIO_SSI1_CE0			__GPIO('D', 29)
#define GPIO_SSI1_CE1			__GPIO('D', 30)

#define SPI_CHIPSELECT_NUM_A	GPIO_SSI0_CE0
#define SPI_CHIPSELECT_NUM_B	GPIO_SSI0_CE1
#define SPI_CHIPSELECT_NUM_C	GPIO_SSI1_CE0

#define SPI0_BUS		0
#define SPI1_BUS		1

/*======================================================================
 * LCD backlight
 */
#define LCD_PWM_CHN				1 /* pwm channel */
#define GPIO_LCD_PWM			(__GPIO('E', 0) + LCD_PWM_CHN)
#define GPIO_LCD_VCC_EN_N		UNUSED_GPIO_PIN // __GPIO('E', 25)

#define LCD_PWM_FREQ	15000 // 30000 /* pwm freq */
#define LCD_PWM_FULL	101 // 256

#define LCD_DEFAULT_BACKLIGHT	30
#define LCD_MAX_BACKLIGHT		100
#define LCD_MIN_BACKLIGHT		1

#define __lcd_init_backlight(n) \
do { \
	__lcd_set_backlight_level(n); \
} while (0)

#if 0 //for test
#define __lcd_set_backlight_level(n) \
do { \
	__gpio_as_output(GPIO_LCD_PWM); \
	__gpio_set_pin(GPIO_LCD_PWM); \
} while (0)

#else

/* 100 level: 0,1,...,100 */
#define __lcd_set_backlight_level(n) \
do { \
	__gpio_as_pwm(1); \
	__tcu_disable_pwm_output(LCD_PWM_CHN); \
	__tcu_stop_counter(LCD_PWM_CHN); \
	__tcu_init_pwm_output_high(LCD_PWM_CHN); \
	__tcu_set_pwm_output_shutdown_abrupt(LCD_PWM_CHN); \
	__tcu_select_clk_div1(LCD_PWM_CHN); \
	__tcu_mask_full_match_irq(LCD_PWM_CHN); \
	__tcu_mask_half_match_irq(LCD_PWM_CHN); \
	__tcu_clear_counter_to_zero(LCD_PWM_CHN); \
	__tcu_set_full_data(LCD_PWM_CHN, JZ_EXTAL / LCD_PWM_FREQ); \
	__tcu_set_half_data(LCD_PWM_CHN, JZ_EXTAL / LCD_PWM_FREQ * n / LCD_PWM_FULL); \
	__tcu_enable_pwm_output(LCD_PWM_CHN); \
	__tcu_select_extalclk(LCD_PWM_CHN); \
	__tcu_start_counter(LCD_PWM_CHN); \
} while (0)
#endif

#define __lcd_close_backlight() \
do { \
	__gpio_as_output(GPIO_LCD_PWM); \
	__gpio_clear_pin(GPIO_LCD_PWM); \
} while (0)

#define HDMI_RST_N_PIN		__GPIO('E', 6)
#define HDMI_POWERON_EN		__GPIO('B', 6)
//#define HDMI_POWERON_EN_5V	__GPIO('D', 10)
#define JZ_SYSTEM_5V_PWR	__GPIO('D', 10)

#define HDMI_DETE_PIN		__GPIO('E', 12)
#define HDMI_DETE_PIN_IRQ	(IRQ_GPIO_0 + HDMI_DETE_PIN)

#define HDMI_I2C_SCL		__GPIO('D', 31)
#define HDMI_I2C_SDL		__GPIO('D', 30)
#define GPIO_HDMI_INT_N		0
#define GPIO_HDMI_HPD		__GPIO('E', 11)
#define CONFIG_HDMI_HOTPLUG_HPD_CONNECT_LOW_ACTIVE

#define  HP_POWER_EN		__GPIO('E', 9) //shutdown amp
#define  EARPHONE_DETE		__GPIO('D', 6) //hp detect
#define  EARPHONE_DETE_IRQ	(IRQ_GPIO_0 + EARPHONE_DETE)
#define  DETE_ACTIV_LEVEL	0 // 1--is hight 0-- is low

#define AV_OUT_DETE			__GPIO('D', 25)
#define AV_OUT_DETE_IRQ		(IRQ_GPIO_0 + AV_OUT_DETE)

#define UMIDO_KEY_UP		__GPIO('B', 25)
#define UMIDO_KEY_DOWN		__GPIO('B', 24)
#define UMIDO_KEY_RIGHT		__GPIO('B', 26)
#define UMIDO_KEY_LEFT		__GPIO('D', 0) //__GPIO('B', 27)
#define UMIDO_KEY_A			__GPIO('D', 22) //__GPIO('F', 6)
#define UMIDO_KEY_B			__GPIO('D', 23) //__GPIO('F', 11)
#define UMIDO_KEY_X			__GPIO('E', 7) //__GPIO('F', 4)
#define UMIDO_KEY_Y			__GPIO('E', 11) //__GPIO('F', 5)
#define UMIDO_KEY_L			__GPIO('B', 23)
#define UMIDO_KEY_R			__GPIO('D', 24)//__GPIO('F', 8)
#define UMIDO_KEY_SELECT	__GPIO('D', 17)
#define UMIDO_KEY_START		__GPIO('D', 18)

#define UMIDO_KEY_LED		__GPIO('D', 21)
#define BATTERY_LOW_LED		__GPIO('E', 31)
//#define UMIDO_KEY_VOL_UP	__GPIO('D', 25) //__GPIO('F', 10)
//#define UMIDO_KEY_VOL_DOWN	__GPIO('D', 21) //__GPIO('F', 9)

#define SAMPLE_TIMES		5

//#define HOLD_DETET		__GPIO('B', 28)
//#define  FM_ANT_EN		__GPIO('B', 28)

#define JZ_EARLY_UART_BASE UART1_BASE

#endif /* __ASM_JZ4760_LEPUS_H__ */
