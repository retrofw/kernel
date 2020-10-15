/*
 * linux/include/asm-mips/mach-jz4760b/board-lynx.h
 *
 * JZ4760B-based LYNX board ver 1.0 definition.
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

#ifndef __ASM_JZ4760B_LYNX_H__
#define __ASM_JZ4760B_LYNX_H__
#include <linux/act8930_power.h>
/*======================================================================
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		12000000  /* Main extal freq:	12 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq:	32.768 KHz */


#define __GPIO(p, n) (32 * (p - 'A') + n)

#define GPIO_OTG_ID_PIN         __GPIO('E', 2)

#define OTG_HOTPLUG_PIN         __GPIO('E', 19)

#define GPIO_OTG_ID_IRQ         (IRQ_GPIO_0 + GPIO_OTG_ID_PIN)
#define OTG_HOTPLUG_IRQ         (IRQ_GPIO_0 + OTG_HOTPLUG_PIN)

#define GPIO_OTG_STABLE_JIFFIES 10


/*======================================================================
 * HDMI 
 */
#define EP932M_RESET_PIN (32*4+11)

/*======================================================================
 * SYSTEM GPIO
 */
#define GPIO_DISP_OFF_N		(32 * 5 + 6) /* GPF6 */ //???

#define GPIO_SD0_VCC_EN_N	(32 * 5 + 9)  /* GPF9 */
#define GPIO_SD0_CD_N		(32 * 1 + 22) /* GPB22 */
#define GPIO_SD0_WP_N		(32 * 5 + 4)  /* GPF4 */
#define GPIO_SD1_VCC_EN_N	(32 * 3 + 9)  /* GPD9 */
#define GPIO_SD1_CD_N		(32 * 1 + 22) /* GPB22 */

#define GPIO_USB_DETE		(32 * 5 + 8) /* GPF8 */

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
#define GPIO_HOME		(32 * 3 + 17) // SW3-GPD17
#define GPIO_MENU		(32 * 4 + 25) // SW6-boot_sel2-GPE25
//#define GPIO_CALL		(32 * 2 + 31) // SW1-GPC31
#define GPIO_BACK		(32 * 3 + 27) // SW4-GPD27
#define GPIO_ENDCALL    (32 * 0 + 30) // WAKEUP-GPA30
#define GPIO_VOLUMEDOWN (32 * 3 + 18) // SW7-boot_sel1-GPD18
#define GPIO_VOLUMEUP   (32 * 4 + 26) // SW8-boot_sel0-GPE26

#define GPIO_ADKEY_INT	(32 * 4 + 8)  // GPE8
/*====================================================================
 *  ADKEYS LEVEL
 */
#define DPAD_LEFT_LEVEL		186 	//0.15V, 186=0.15/3.3*4096
#define DPAD_DOWN_LEVEL		2482    //2.0V
#define DPAD_UP_LEVEL		1985    //1.6V
#define DPAD_CENTER_LEVEL	1489    //1.2V
#define DPAD_RIGHT_LEVEL	868     //0.7V


#define GPIO_TS_I2C_INT         (32 * 3 + 20)   //GPD20
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
	int detected = 1;			\
	if (__gpio_get_pin(GPIO_SD1_CD_N))	\
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
* PMU ACT8930
*/
#define ACT8930_CPU_CORE    ACT8930_LDO1_VOLTAGE_SET
#define ACT8930_DDR2        ACT8930_LDO3_VOLTAGE_SET
#define ACT8930_HDMI_1_8    ACT8930_LDO5_VOLTAGE_SET
#define ACT8930_HDMI_3_3    ACT8930_LDO4_VOLTAGE_SET
#define ACT8930_WIFI        ACT8930_LDO6_VOLTAGE_SET
#define ACT8930_COMPASS     ACT8930_LDO7_VOLTAGE_SET

/*======================================================================
166  * headphone and speaker
167  */
#define GPIO_HEAD_DET       (32 * 1 + 23) //GPB23
#define GPIO_SPK_EN       (32 * 5 + 11) /*PF11*/

#define GPIO_SPEAKER_EN     (32 * 5 + 11) //GPF11

#define GPIO_HP_MUTE       (32 * 3 + 29) /*PD29*
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

#endif /* __ASM_JZ4760B_LYNX_H__ */
