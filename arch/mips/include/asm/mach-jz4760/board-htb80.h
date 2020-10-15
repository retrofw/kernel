/*
 * linux/include/asm-mips/mach-jz4760/board-htb80.h
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

#ifndef __ASM_JZ4760_HTB80_H__
#define __ASM_JZ4760_HTB80_H__

/*======================================================================
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		12000000  /* Main extal freq:	12 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq:	32.768 KHz */

/*======================================================================
 * SYSTEM GPIO
 */

/*  */
/* TF-card on MSC1 */
#define GPIO_SD1_VCC_EN_N	(32 * 1 + 23)  /* GPB23 */
#define GPIO_SD1_CD_N		(32 * 1 + 22) /* GPB22 */
#define MSC1_HOTPLUG_PIN	GPIO_SD1_CD_N
#define ACTIVE_LOW_MSC1_CD	1

/* wifi on MSC2 */
#define	GPIO_WIFI_PWR_EN	(32 * 5 + 0) /* GPF0 */
#define GPIO_WIFI_PDn		(32 * 5 + 3) /* GPF3 */
#define GPIO_WIFI_WAKEUP	(32 * 5 + 2) /* GPF2 */
#define GPIO_WIFI_MODE		(32 * 5 + 1) /* GPF1 */

#define GPIO_SD2_VCC_EN_N	GPIO_WIFI_PWR_EN
#define GPIO_SD2_CD_N		GPIO_WIFI_MODE
#define MSC2_HOTPLUG_PIN	GPIO_SD2_CD_N
#define ACTIVE_LOW_MSC2_CD	1

#define __msc1_enable_power()				\
	do {						\
		__gpio_clear_pin(GPIO_SD1_VCC_EN_N);	\
	} while (0)

#define __msc1_disable_power()				\
	do {						\
		__gpio_set_pin(GPIO_SD1_VCC_EN_N);	\
	} while (0)

#define __msc1_enable_power()				\
	do {						\
		__gpio_clear_pin(GPIO_SD1_VCC_EN_N);	\
	} while (0)

#define __msc1_disable_power()				\
	do {						\
		__gpio_set_pin(GPIO_SD1_VCC_EN_N);	\
	} while (0)








#define GPIO_DISP_OFF_N		(32 * 5 + 6) /* GPF6 */ //???




#define GPIO_USB_DETE		(32 * 4 + 19) /* GPE19 */

#define GPIO_LCD_PWM		(32 * 4 + 1) /* GPE01 */
#define GPIO_LCD_VCC_EN_N       (32 * 1 + 31) /* GPB31 */

#define GPIO_BOOT_SEL0		(32 * 3 + 17) /* GPD17 */
#define GPIO_BOOT_SEL1		(32 * 3 + 18) /* GPD18 */
#define GPIO_BOOT_SEL2		(32 * 3 + 19) /* GPD19 */

/* Ethernet: WE#, RD#, CS5# */
#define GPIO_NET_INT		(32 * 5 + 5)  /* GPF5 */

#define GPIO_GSM_RI             (32 * 3 + 8) /* GPD08, waking cpu from sleep when a call comes in. */
#define GPIO_GSM_RI_ACK         (32 * 0 + 29) /* GPA29, notify baseband not to send data to cpu when cpu is sleeping. */
#define GPIO_GSM_WAKE           (32 * 1 + 27) /* GPB27 */
#define GPIO_GSM_WAKE_ACK       (32 * 1 + 30) /* GPB30 */

#define GPIO_POWER_ON           (32 * 0 + 30)  /* GPA30 */

/*====================================================================
 * GPIO KEYS and ADKEYS (GPIO_WAKEUP used for end call)
 */
#define GPIO_HOME		(32 * 2 + 29) // SW3-GPC29
#define GPIO_MENU		(32 * 3 + 19) // SW6-boot_sel2-GPD19
#define GPIO_CALL		(32 * 2 + 31) // SW1-GPC31
#define GPIO_BACK		(32 * 3 + 27) // SW4-GPD27
#define GPIO_ENDCALL    (32 * 0 + 30) // WAKEUP-GPA30
#define GPIO_VOLUMEDOWN (32 * 3 + 18) // SW7-boot_sel1-GPD18
#define GPIO_VOLUMEUP   (32 * 3 + 17) // SW8-boot_sel0-GPD17

#define GPIO_ADKEY_INT	(32 * 4 + 8)  // GPE8

/*====================================================================
 *  ADKEYS LEVEL
 */
#define DPAD_LEFT_LEVEL		186 	//0.15V, 186=0.15/3.3*4096
#define DPAD_DOWN_LEVEL		2482    //2.0V
#define DPAD_UP_LEVEL		1985    //1.6V
#define DPAD_CENTER_LEVEL	1489    //1.2V
#define DPAD_RIGHT_LEVEL	868     //0.7V


#define GPIO_TS_I2C_INT         (32 * 1 + 20)   //GPB20
#define GPIO_TS_I2C_IRQ         (IRQ_GPIO_0 + GPIO_TS_I2C_INT)

/*======================================================================
 * Analog input for VBAT is the battery voltage divided by CFG_PBAT_DIV.
 */
//#define CFG_PBAT_DIV            4

/*
 * M-T4D touchscreen
 */
//#define LCD_INT		(32*2+20)	/* WAIT_N GPC20 interrupt pin */
//#define IOSWITCH	(32*5+23)	/* GPF23 */


/*======================================================================
 * MMC/SD
 */
#define MSC0_WP_PIN		GPIO_SD0_WP_N
#define MSC0_HOTPLUG_PIN	GPIO_SD0_CD_N
#define MSC0_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD0_CD_N)

#define MSC1_WP_PIN		GPIO_SD1_WP_N
#define MSC1_HOTPLUG_PIN	GPIO_SD1_CD_N
#define MSC1_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD1_CD_N)

/*======================================================================
 * LCD backlight
 */
#define LCD_PWM_CHN 1   /* pwm channel */
#define LCD_PWM_FULL 256
#define PWM_BACKLIGHT_CHIP	0	/*0: digital pusle; 1: PWM*/
#define LCD_DEFAULT_BACKLIGHT		80
#define LCD_MAX_BACKLIGHT		100
#define LCD_MIN_BACKLIGHT		1

#if 1
#if PWM_BACKLIGHT_CHIP

#define __lcd_init_backlight(n)					\
do {    							\
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
	__gpio_as_output(GPIO_LCD_PWM);				\
	__gpio_clear_pin(GPIO_LCD_PWM);				\
} while (0)

#else	/* PWM_BACKLIGHT_CHIP */

#define __send_low_pulse(n)					\
do {								\
	unsigned int i;						\
	for (i = n; i > 0; i--)	{				\
		__gpio_clear_pin(GPIO_LCD_PWM);			\
		udelay(1);					\
		__gpio_set_pin(GPIO_LCD_PWM);			\
		udelay(3);					\
	}							\
} while (0)

#define MAX_BRIGHTNESS_STEP	16				/* RT9365 supports 16 brightness step */
#define CONVERT_FACTOR		(256/MAX_BRIGHTNESS_STEP)	/* System support 256 brightness step */

#define __lcd_init_backlight(n)					\
do {								\
	unsigned int tmp = (n)/CONVERT_FACTOR + 1;		\
	__gpio_as_output(GPIO_LCD_PWM);				\
	__gpio_set_pin(GPIO_LCD_PWM);				\
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
	__gpio_as_output(GPIO_LCD_PWM); 			\
	__gpio_clear_pin(GPIO_LCD_PWM); 			\
} while (0)

#endif	/*PWM_BACKLIGHT_CHIP*/
#endif  // if 0

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

/* mplayer keys */
#define GPIO_MP_VOLUMEUP	(32 * 2 + 29) // SW3-GPC29
#define GPIO_MP_VOLUMEDOWN	(32 * 2 + 31) // SW1-GPC31
#define GPIO_MP_MUTE		(32 * 3 + 19) // SW6-boot_sel2-GPD19
#define GPIO_MP_PAUSE		(32 * 3 + 27) // SW4-GPD27
#define GPIO_MP_PLAY		(32 * 0 + 30) // SW9-WAKEUP-GPA30
#define GPIO_MP_REWIND		(32 * 3 + 18) // SW7-boot_sel1-GPD18
#define GPIO_MP_FORWARD		(32 * 3 + 17) // SW8-boot_sel0-GPD17

#define ACTIVE_LOW_MUTE		1
#define ACTIVE_LOW_PUASE	1
#define ACTIVE_LOW_PLAY		1
#define ACTIVE_LOW_REWIND	1
#define ACTIVE_LOW_FORWARD	1


#define GPIO_SW3	(32 * 2 + 29) // SW3-GPC29
#define GPIO_SW1	(32 * 2 + 31) // SW1-GPC31
#define GPIO_SW6	(32 * 3 + 19) // SW6-boot_sel2-GPD19
#define GPIO_SW4	(32 * 3 + 27) // SW4-GPD27
#define GPIO_SW9	(32 * 0 + 30) // SW9-WAKEUP-GPA30
#define GPIO_SW7	(32 * 3 + 18) // SW7-boot_sel1-GPD18
#define GPIO_SW8	(32 * 3 + 17) // SW8-boot_sel0-GPD17

#define ACTIVE_LOW_SW3		1
#define ACTIVE_LOW_SW1		1
#define ACTIVE_LOW_SW6		1
#define ACTIVE_LOW_SW4		1
#define ACTIVE_LOW_SW9		1
#define ACTIVE_LOW_SW7   	1
#define ACTIVE_LOW_SW8     	1

#define JZ_EARLY_UART_BASE UART1_BASE

#endif /* __ASM_JZ4760_HTB80_H__ */
