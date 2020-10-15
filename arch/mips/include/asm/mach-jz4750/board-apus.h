/*
 *  linux/include/asm-mips/mach-jz4750/board-apus.h
 *
 *  JZ4750-based APUS board ver 1.x definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4750_APUS_H__
#define __ASM_JZ4750_APUS_H__

/*======================================================================
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		24000000  /* Main extal freq: 24 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq: 32.768 KHz */

/*======================================================================
 * GPIO
 */
#define GPIO_DISP_OFF_N         (32*4+25) /* GPE25 */
#define GPIO_SD0_VCC_EN_N	(32*2+10) /* GPC10 */
#define GPIO_SD0_CD_N		(32*2+11) /* GPC11 */
#define GPIO_SD0_WP		(32*2+12) /* GPC12 */
#define GPIO_SD1_VCC_EN_N	(32*2+13) /* GPC13 */
#define GPIO_SD1_CD_N		(32*2+14) /* GPC14 */
#define GPIO_USB_DETE		(32*2+15) /* GPC15 */
#define GPIO_DC_DETE_N		(32*2+8)  /* GPC8 */
#define GPIO_CHARG_STAT_N	(32*2+9)  /* GPC9 */
#define GPIO_LCD_VCC_EN_N	(32*3+30) /* GPC10 */
#define GPIO_LCD_PWM   		(32*4+24) /* GPE24 */
#define GPIO_UDC_HOTPLUG	GPIO_USB_DETE


/*======================================================================
 * LCD backlight
 */
#define LCD_PWM_CHN 4    /* pwm channel */

#define LCD_MAX_BACKLIGHT		100
#define LCD_MIN_BACKLIGHT		1
#define LCD_DEFAULT_BACKLIGHT		80

/* LCD Backlight PWM Control - River. */
#define HAVE_LCD_PWM_CONTROL	1

#ifdef HAVE_LCD_PWM_CONTROL
static inline void __lcd_pwm_set_backlight_level(int n)
{
	__tcu_stop_counter(LCD_PWM_CHN);

	__tcu_set_pwm_output_shutdown_abrupt(LCD_PWM_CHN);
	__tcu_disable_pwm_output(LCD_PWM_CHN);

	__tcu_set_count(LCD_PWM_CHN, 0);
	__tcu_set_full_data(LCD_PWM_CHN, LCD_MAX_BACKLIGHT + 1);
	__tcu_set_half_data(LCD_PWM_CHN, n);

	__tcu_enable_pwm_output(LCD_PWM_CHN);
	__tcu_start_counter(LCD_PWM_CHN);

	return;
}

static inline void __lcd_pwm_start(void)
{
	__gpio_as_pwm(4);

	__tcu_stop_counter(LCD_PWM_CHN);

	__tcu_select_extalclk(LCD_PWM_CHN);
	__tcu_select_clk_div4(LCD_PWM_CHN);
	__tcu_init_pwm_output_high(LCD_PWM_CHN);

	__lcd_pwm_set_backlight_level(LCD_DEFAULT_BACKLIGHT);

	return;
}

static inline void __lcd_pwm_stop(void)
{
	__tcu_stop_counter(LCD_PWM_CHN);

	__tcu_set_pwm_output_shutdown_abrupt(LCD_PWM_CHN);
	__tcu_disable_pwm_output(LCD_PWM_CHN);

	return;
}

#define __lcd_set_backlight_level(n) __lcd_pwm_set_backlight_level(n)

#else /* Old GPIO Control */
/* 100 level: 0,1,...,100 */
#define __lcd_set_backlight_level(n)	\
do {					\
	__gpio_as_output(GPIO_LCD_PWM);	\
	__gpio_set_pin(GPIO_LCD_PWM);	\
} while (0)
#endif

#define __lcd_close_backlight()		\
do {					\
	__gpio_as_output(GPIO_LCD_PWM);	\
	__gpio_clear_pin(GPIO_LCD_PWM);	\
} while (0)

//20091116
#define CFG_PBAT_DIV            4
/*====================================================================
 * GPIO KEYS and ADKEYS
 */
#define GPIO_HOME		(32*5+22) // SW3-GPF22
#define GPIO_MENU		(32*5+20) // SW5-GPF20
#define GPIO_CALL		(32*5+23) // SW2-GPF23
#define GPIO_ENDCALL		(32*2+31) // SW6-boot_sel1-GPC31
#define GPIO_BACK		(32*5+21) // SW4-GPF21
#define GPIO_SW7		(32*2+30) // SW7-boot_sel0-GPC30
#define GPIO_ADKEY_INT		(32+30)   // GPB30
/*====================================================================
 *  ADKEYS LEVEL
 */

#define DPAD_LEFT_LEVEL		225	//0.18105V, 225=0.18105/3.3*4096
#define DPAD_DOWN_LEVEL		535	//0.4314V
#define DPAD_UP_LEVEL		887	//0.7143V
#define DPAD_CENTER_LEVEL	1422	//1.1456V
#define DPAD_RIGHT_LEVEL	2333	//1.88V
/*
 * The key interrupt pin is low voltage or fall edge acitve
 */
#define ACTIVE_LOW_HOME		1
#define ACTIVE_LOW_MENU		1
#define ACTIVE_LOW_BACK		1
#define ACTIVE_LOW_CALL		1
#define ACTIVE_LOW_ENDCALL	0
#define ACTIVE_LOW_SW10		1
#define ACTIVE_LOW_ADKEY	0
#define ACTIVE_LOW_MSC0_CD	1 /* work when GPIO_SD1_CD_N is low */
#define ACTIVE_LOW_MSC1_CD	0 /* work when GPIO_SD1_CD_N is high */
#define ACTIVE_WAKE_UP 		1


/*======================================================================
 * MMC/SD
 */

#define MSC0_WP_PIN		GPIO_SD0_WP
#define MSC0_HOTPLUG_PIN	GPIO_SD0_CD_N
#define MSC0_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD0_CD_N)

#define MSC1_WP_PIN		GPIO_SD1_WP
#define MSC1_HOTPLUG_PIN	GPIO_SD1_CD_N
#define MSC1_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD1_CD_N)

#define __msc0_init_io()			\
do {						\
	__gpio_as_output(GPIO_SD0_VCC_EN_N);	\
	__gpio_as_input(GPIO_SD0_CD_N);		\
} while (0)

#define __msc0_enable_power()			\
do {						\
	__gpio_clear_pin(GPIO_SD0_VCC_EN_N);	\
} while (0)

#define __msc0_disable_power()			\
do {						\
	__gpio_set_pin(GPIO_SD0_VCC_EN_N);	\
} while (0)

#define __msc0_card_detected(s)			\
({						\
	int detected = 1;			\
	if (__gpio_get_pin(GPIO_SD0_CD_N))	\
		detected = 0;			\
	detected;				\
})

#define __msc1_init_io()			\
do {						\
	__gpio_as_output(GPIO_SD1_VCC_EN_N);	\
	__gpio_as_input(GPIO_SD1_CD_N);		\
} while (0)

#define __msc1_enable_power()			\
do {						\
	__gpio_clear_pin(GPIO_SD1_VCC_EN_N);	\
} while (0)

#define __msc1_disable_power()			\
do {						\
	__gpio_set_pin(GPIO_SD1_VCC_EN_N);	\
} while (0)

#define __msc1_card_detected(s)			\
({						\
	int detected = 0;			\
	if (__gpio_get_pin(GPIO_SD1_CD_N))	\
		detected = 1;			\
	detected;				\
})

#define JZ_EARLY_UART_BASE UART3_BASE

/*======================================================================
 * SPI 
 */
#define  GPIO_SSI0_CE0	(32*1+29)
#define  GPIO_SSI0_CE1	(32*1+31)
#define  GPIO_SSI0_GPC	(32*1+30)
#define  GPIO_SSI1_CE0	(32*3+29)
#define  GPIO_SSI1_CE1	(32*3+30)

#define	 SPI_CHIPSELECT_NUM_A		GPIO_SSI0_CE0
#define	 SPI_CHIPSELECT_NUM_B		GPIO_SSI0_CE1
#define	 SPI_CHIPSELECT_NUM_C		GPIO_SSI1_CE0

#define  SPI0_BUS		0
#define  SPI1_BUS		1


#endif /* __ASM_JZ4750_APUS_H__ */
