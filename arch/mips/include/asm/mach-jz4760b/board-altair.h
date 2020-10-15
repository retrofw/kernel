/*
 * linux/include/asm-mips/mach-jz4760b/board-altair.h
 *
 * JZ4760B-based ALTAIR board definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Inc.
 *
 * Author: Jason<xwang@ingenic.cn>
 *		Based on board-aquila.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4760B_ALTAIR_H__
#define __ASM_JZ4760B_ALTAIR_H__

/*======================================================================
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		12000000  /* Main extal freq:	24 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq:	32.768 KHz */

#define __GPIO(p, n) (32 * (p - 'A') + n)

/*======================================================================
 * SYSTEM GPIO
 */
#define GPIO_DISP_OFF_N		(32 * 3 + 17) /* GPD17 */

#define GPIO_SD0_VCC_EN_N	(32 * 1 + 22) /* GPB22 */
#define GPIO_SD0_WP_N		(32 * 1 + 23) /* GPB23 */
#define GPIO_SD0_CD_N		(32 * 1 + 24) /* GPB24 */

#define GPIO_SD1_VCC_EN_N	(32 * 1 + 2) /* GPB02 */
#define GPIO_SD1_CD_N		(32 * 1 + 3) /* GPB03 */

#define GPIO_USB_DETE		__GPIO('B', 4)

#define GPIO_LCD_PWM		(32 * 4 + 1) /* GPE01 */

#define GPIO_AMPEN_N		(32 * 4 + 11) /* GPE11 */

#define GPIO_BOOT_SEL0		(32 * 3 + 17) /* GPD17 */
#define GPIO_BOOT_SEL1		(32 * 3 + 18) /* GPD18 */
#define GPIO_BOOT_SEL2		(32 * 3 + 19) /* GPD19 */

#define GPIO_WM831x_DETECT      (32 * 1 + 5)  /* GPB5 */

/*====================================================================
 *  KEYPAD
 */
#define KEY_C0			(32 * 4 + 8) //GPE08
#define KEY_C1                  (32 * 4 + 9) //GPE09
#define KEY_C2                  (32 * 5 + 4) //GPF04
#define KEY_C3                  (32 * 5 + 5) //GPF05
#define KEY_C4                  (32 * 5 + 6) //GPF06

#define KEY_R0			(32 * 5 + 7) //GPF07
#define KEY_R1                  (32 * 5 + 8) //GPF08
#define KEY_R2                  (32 * 5 + 9) //GPF09
#define KEY_R3                  (32 * 5 + 10) //GPF10
#define KEY_R4                  (32 * 5 + 11) //GPF11

//keycode for android
#define KEY_CENTER              232
#define KEY_CALL                231
#define KEY_POUND               228
#define KEY_STAR                227

/*======================================================================
 * Analog input for VBAT is the battery voltage divided by CFG_PBAT_DIV.
 */
//#define CFG_PBAT_DIV            4

/*
 * M-T4D touchscreen
 */
#define LCD_INT			(32 * 0 + 16) /* GPA16 interrupt pin */
#define LCD_INT_IRQ		(IRQ_GPIO_0 + LCD_INT)

/* 
 * E-Compass and G-Sensor
 */
#define COMPASS_RSTN	(32*3+5)	/* GPD5 */
#define COMPASS_INTR	(32*3+4)	/* GPD4 */
#define COMPASS_INT_IRQ	(IRQ_GPIO_0 + COMPASS_INTR)	/* irq number */

#define SENSOR_INT1	(32*4+26)	/* GPE26 */
#define SENSOR_INT2	(32*4+27)	/* GPE27 */
#define SENSOR_INT1_IRQ	(IRQ_GPIO_0 + SENSOR_INT1)	/* irq number */

/*
 * Used for GPIO-based bitbanging I2C
 */
#define IOSWITCH_EN		(32*4+6)	/* GPE6 */
#define CIM_I2C_SCK		(32*5+2)	/* GPF2 */
#define CIM_I2C_SDA		(32*5+1)	/* GPF1 */


/*======================================================================
 * MMC/SD
 */
#define MSC1_HOTPLUG_PIN	GPIO_SD1_CD_N
#define MSC1_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD1_CD_N)

/*====================================================================== 
 * Modem
 */
#if defined(CONFIG_GSM_IW368)
#define GPIO_GSM_ONOFF          (32 * 1 + 25)
#define GPIO_GSM_PWR_STATUS     (32 * 0 + 28)
#define GPIO_GSM_RI             (32 * 3 + 8)  /* GPD08, waking cpu from sleep when a call comes in. */
#define GPIO_GSM_RI_ACK         (32 * 0 + 29) /* GPA29, notify baseband not to send data to cpu when cpu is sleeping. */
#define GPIO_GSM_WAKE           (32 * 1 + 27) /* GPB27 */
#define GPIO_GSM_WAKE_ACK       (32 * 1 + 30) /* GPB30 */
#endif

#if defined(CONFIG_TDSCDMA_LC6311)
#define GPIO_TD_POWERON        (32 * 3 + 9)
#define GPIO_TD_RESET          (32 * 3 + 19)  /* shared with boot_sel2 */
#endif

/*======================================================================
 * LCD backlight
 */
#define LCD_PWM_CHN		4	/* pwm channel */
#define LCD_PWM_FULL		256
#define PWM_BACKLIGHT_CHIP	0	/*0: digital pusle; 1: PWM*/

#if 1 // temporarily
#if PWM_BACKLIGHT_CHIP

#define __lcd_init_backlight(n)					\
do {								\
} while (0)

/* 100 level: 0,1,...,100 */
#define __lcd_set_backlight_level(n)				\
do {								\
	__gpio_as_pwm(4);					\
	__tcu_disable_pwm_output(LCD_PWM_CHN);			\
	__tcu_stop_counter(LCD_PWM_CHN);			\
	__tcu_init_pwm_output_high(LCD_PWM_CHN);		\
	__tcu_set_pwm_output_shutdown_abrupt(LCD_PWM_CHN);	\
	__tcu_select_clk_div1(LCD_PWM_CHN);			\
	__tcu_mask_full_match_irq(LCD_PWM_CHN);			\
	__tcu_mask_half_match_irq(LCD_PWM_CHN);			\
	__tcu_set_count(LCD_PWM_CHN,0);				\
	__tcu_set_full_data(LCD_PWM_CHN, __cpm_get_extalclk() / 30000);\
	__tcu_set_half_data(LCD_PWM_CHN, __cpm_get_extalclk() / 30000 * n / (LCD_PWM_FULL - 1));\
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
#define ACTIVE_LOW_MSC0_CD	1 /* work when GPIO_SD1_CD_N is low */
#define ACTIVE_LOW_MSC1_CD	0 /* work when GPIO_SD1_CD_N is low */

#endif /* __ASM_JZ4760B_ALTAIR_H__ */
