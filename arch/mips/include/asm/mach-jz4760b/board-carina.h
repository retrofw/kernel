/*
 * linux/include/asm-mips/mach-jz4760b/board-carina.h
 *
 * JZ4760B-based CARINA board ver 1.0 definition.
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

#ifndef __ASM_JZ4760B_CARINA_H__
#define __ASM_JZ4760B_CARINA_H__
#define CARINA_32
/*======================================================================
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		12000000  /* Main extal freq:	12 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq:	32.768 KHz */


#define __GPIO(p, n) (32 * (p - 'A') + n)
#ifdef CARINA_32
#define GPIO_OTG_ID_PIN		__GPIO('D', 7)
#else
#define GPIO_OTG_ID_PIN         __GPIO('E', 6)
#define GPIO_OTG_VBUS_PIN	__GPIO('D', 20)
//#define OTG_HOTPLUG_PIN         __GPIO('E', 19)
#endif
#define GPIO_OTG_ID_IRQ         (IRQ_GPIO_0 + GPIO_OTG_ID_PIN)
//#define OTG_HOTPLUG_IRQ         (IRQ_GPIO_0 + OTG_HOTPLUG_PIN)

#define GPIO_OTG_STABLE_JIFFIES 10



/*======================================================================
 * SYSTEM GPIO
 */
#define GPIO_UNUSE_PIN	(32 * 5 + 11)
#define GPIO_SD0_VCC_EN_N	(32 * 5 + 9)  /* GPF9 */
#define GPIO_SD0_CD_N		(32 * 1 + 22) /* GPB22 */
#define GPIO_SD0_WP_N		(32 * 5 + 4)  /* GPF4 */

#define GPIO_SD1_VCC_EN_N	(32 * 5 + 6)  /* GPF6 */
#define GPIO_SD1_CD_N		(32 * 3 + 23) /* GPD23 */

#define GPIO_BOOT_SEL0		(32 * 3 + 17) /* GPD17 */
#define GPIO_BOOT_SEL1		(32 * 3 + 18) /* GPD18 */
#define GPIO_BOOT_SEL2		(32 * 3 + 19) /* GPD19 */

#define GPIO_POWER_ON           (32 * 0 + 30)  /* GPA30 */

/*====================================================================
 * GPIO KEYS and ADKEYS (GPIO_WAKEUP used for end call)
 */
#define GPIO_WPS			(32 * 3 + 17) // SW1-BOOT_SEL0
#ifdef CARINA_32
#define	GPIO_PAUSE		(32 * 4 + 14)
#else
#define GPIO_PAUSE		(32 * 3 + 18) // SW2-BOOT_SEL1
#endif
#define ACTIVE_LOW_WPS		1
#define ACTIVE_LOW_PAUSE	1

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
#ifdef CARINA_32
#define OTG_SWITCH_PIN	(32 * 1 + 17)
#define NAND_GPIO_WP_PIN	(32 * 5 + 10)
#define WL_PWR_PIN	(32 * 4 + 13)
#else
#define NAND_GPIO_WP_PIN	(32 * 5 + 4)
#define WL_PWR_PIN	(32 * 4 + 11)
#endif

/*======================================================================
 * MMC/SD
 */
#define MSC0_POWER_PIN	GPIO_SD0_VCC_EN_N
#define MSC0_WP_PIN		GPIO_SD0_WP_N
#define MSC0_HOTPLUG_PIN	GPIO_SD0_CD_N
#define MSC0_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD0_CD_N)


#ifdef	CARINA_32
#define MSC1_POWER_PIN	GPIO_UNUSE_PIN
#define MSC1_WP_PIN		GPIO_UNUSE_PIN
#define MSC1_HOTPLUG_PIN	(32 * 0 + 28)
#define MSC1_HOTPLUG_IRQ	(IRQ_GPIO_0 + MSC1_HOTPLUG_PIN)
#else
#define MSC1_POWER_PIN	GPIO_SD1_VCC_EN_N
#define MSC1_WP_PIN		GPIO_SD1_WP_N
#define MSC1_HOTPLUG_PIN	GPIO_SD1_CD_N
#define MSC1_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD1_CD_N)
#endif

#define __msc0_init_io()			\
do {						\
	__gpio_as_output(MSC0_POWER_PIN);	\
	__gpio_as_input(MSC0_HOTPLUG_PIN);		\
} while (0)

#define __msc0_enable_power()			\
do {						\
	__gpio_clear_pin(MSC0_POWER_PIN);	\
} while (0)

#define __msc0_disable_power()			\
do {						\
	__gpio_set_pin(MSC0_POWER_PIN);	\
} while (0)

#define __msc0_card_detected(s)			\
({						\
	int detected = 1;			\
	if (__gpio_get_pin(MSC0_HOTPLUG_PIN))	\
		detected = 0;			\
	detected;				\
})

#define __msc1_init_io()			\
do {						\
	__gpio_as_output(MSC1_POWER_PIN);	\
	__gpio_as_input(MSC1_HOTPLUG_PIN);		\
} while (0)

#define __msc1_enable_power()			\
do {						\
	__gpio_clear_pin(MSC1_POWER_PIN);	\
} while (0)

#define __msc1_disable_power()			\
do {						\
	__gpio_set_pin(MSC1_POWER_PIN);	\
} while (0)

#define __msc1_card_detected(s)			\
({						\
	int detected = 1;			\
	if (__gpio_get_pin(MSC1_HOTPLUG_PIN))	\
		detected = 0;			\
	detected;				\
})
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


/*======================================================================
 * LCD backlight
 */
#define LCD_PWM_CHN 1   /* pwm channel */
#define LCD_PWM_FULL 256
#define PWM_BACKLIGHT_CHIP	1	/*0: digital pusle; 1: PWM*/
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
do {    								\
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

/*======================================================================
 * AUDIO HP
 */
#define GPIO_HP_MUTE		GPF(7)

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

#endif /* __ASM_JZ4760B_CARINA_H__ */
