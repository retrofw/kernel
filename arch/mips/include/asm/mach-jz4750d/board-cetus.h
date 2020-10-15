/*
 *  linux/include/asm-mips/mach-jz4750d/board-cetus.h
 *
 *  JZ4750D-based CETUS board ver 1.x definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4750D_CETUS_H__
#define __ASM_JZ4750D_CETUS_H__

/*====================================================================== 
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		24000000
#define JZ_EXTAL2		32768     /* RTC extal freq: 32.768 KHz */
//#define CFG_DIV                 1         /* hclk=pclk=mclk=CFG_EXTAL/CFG_DIV, just for FPGA board */


/*====================================================================== 
 * GPIO
 */
#define GPIO_OV5640_RST         (32*4+18)
#define GPIO_OV5640_PWDN        (32*4+19)

#define GPIO_SD0_VCC_EN_N	(32*4+0) /* CIM_D0 */
#define GPIO_SD0_CD_N		(32*4+1) /* CIM_D1 */
#define GPIO_SD0_WP		(32*4+2) /* CIM_D2 */
#define GPIO_SD1_VCC_EN_N	(32*4+3) /* CIM_D3 */
#define GPIO_SD1_CD_N		(32*4+4) /* CIM_D4 */

#define GPIO_USB_DETE		(32*4+6) /* CIM_D6 */
#define GPIO_DC_DETE_N		(32*4+8) /* CIM_MCLK */
#define GPIO_CHARG_STAT_N	(32*4+10) /* CIM_VSYNC */
#define GPIO_DISP_OFF_N		(32*4+18) /* SDATO */
#define GPIO_LCD_VCC_EN_N	(32*4+19) /* SDATI */
//#define GPIO_LED_EN       	124 /* GPD28 */

#define GPIO_UDC_HOTPLUG	GPIO_USB_DETE

/*====================================================================== 
 * LCD backlight
 */
#define GPIO_LCD_PWM   		(32*4+22) /* GPE22 PWM2 */ 
#define LCD_PWM_CHN 2    /* pwm channel */

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
	__gpio_as_pwm(2);

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

#else

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

//20091118
/*====================================================================
 * GPIO KEYS and ADKEYS
 */
#define GPIO_HOME		(32*5+12) // SW4-GPF12 SSI_DR
#define GPIO_MENU		(32*2+31) // SW2-GPC31 boot_sel1
#define GPIO_BACK		(32*5+11) // SW3-GPF11 SSI_DT
#define GPIO_CALL		(32*5+10) // SW5-GPF10 SSI_CLK
#define GPIO_ENDCALL		(32*4+7)  // SW6-GPE7  CIM_D7
#define GPIO_SW10		(32*4+25) // SW10-GPE25 UART1_TXD
#define GPIO_ADKEY_INT		(32*4+11) // KEY_INT-GPE11  CIM_HSYNC

/*====================================================================
 * GWT GPIO
 */
#define GPIO_GWTC9XXXB_INT              (32 * 4 + 7)    //GPE7
//#define GPIO_GWTC9XXXB_RST              0
#define GPIO_GWTC9XXXB_IRQ              (IRQ_GPIO_0 + GPIO_GWTC9XXXB_INT )/* Used for GPIO-based bitbanging I2C*/
/*====================================================================
 *  ADKEYS LEVEL
 */

#define DPAD_LEFT_LEVEL		869	//0.7V, 225=0.18105/3.3*4096
#define DPAD_DOWN_LEVEL		1986	//1.6V
#define DPAD_UP_LEVEL		2482	//2.0V
#define DPAD_CENTER_LEVEL	1489	//1.2V
#define DPAD_RIGHT_LEVEL	186	//0.15V

/*====================================================================== 
 * Analog input for VBAT is the battery voltage divided by CFG_PBAT_DIV.
 */
#define CFG_PBAT_DIV            1

/*
 * The GPIO interrupt pin is low voltage or fall edge acitve
 */
#define ACTIVE_LOW_HOME		1
#define ACTIVE_LOW_MENU		1
#define ACTIVE_LOW_BACK		1
#define ACTIVE_LOW_CALL		1
#define ACTIVE_LOW_ENDCALL	1
#define ACTIVE_LOW_SW10		1
#define ACTIVE_LOW_ADKEY	1
#define ACTIVE_LOW_MSC0_CD	1 /* work when GPIO_SD0_CD_N = 0 */
#define ACTIVE_LOW_MSC1_CD	1 /* work when GPIO_SD1_CD_N = 0 */
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

/*
#define __msc0_card_detected(s)			\
({						\
	int detected = 1;			\
	if (__gpio_get_pin(GPIO_SD0_CD_N))	\
		detected = 0;			\
	detected;				\
})
*/

#if ACTIVE_LOW_MSC0_CD == 1 /* work when cd is low */
#define __msc0_card_detected(s)			\
({						\
	int detected = 1;			\
	if (__gpio_get_pin(GPIO_SD0_CD_N))	\
		detected = 0;			\
	detected;				\
})
#else
#define __msc0_card_detected(s)			\
({						\
	int detected = 0;			\
	if (__gpio_get_pin(GPIO_SD0_CD_N))	\
		detected = 1;			\
	detected;				\
})
#endif /*ACTIVE_LOW_MSC0_CD*/

#define __msc1_init_io()			\
do {						\
	__gpio_as_output(GPIO_SD1_VCC_EN_N);	\
	/*	__gpio_as_input(GPIO_SD1_CD_N);*/	\
} while (0)

#define __msc1_enable_power()			\
do {						\
	__gpio_clear_pin(GPIO_SD1_VCC_EN_N);	\
} while (0)

#define __msc1_disable_power()			\
do {						\
	__gpio_set_pin(GPIO_SD1_VCC_EN_N);	\
} while (0)

/*
#define __msc1_card_detected(s)			\
({						\
	int detected = 0;			\
	if (!(__gpio_get_pin(GPIO_SD1_CD_N)))	\
		detected = 1;			\
	detected;				\
})
*/

#if ACTIVE_LOW_MSC1_CD == 1 /* work when cd is low */
#define __msc1_card_detected(s)			\
({						\
	int detected = 1;			\
	if (__gpio_get_pin(GPIO_SD1_CD_N))	\
		detected = 0;			\
	detected;				\
})
#else
#define __msc1_card_detected(s)			\
({						\
	int detected = 0;			\
	if (__gpio_get_pin(GPIO_SD1_CD_N))	\
		detected = 1;			\
	detected;				\
})
#endif /*ACTIVE_LOW_MSC1_CD*/



/*======================================================================
 * SPI 
 */
#define GPIO_SSI0_CE0	(32*1+29)
#define GPIO_SSI0_CE1	(32*1+31)
#define SSI0_GPC_PIN	(32*1+30)
#define GPI1_SSI0_CE0	(32*1+29)		/* same as SSI0, for avoiding compilation error and ... */
#define GPI1_SSI0_CE1	(32*1+31)

#define	 SPI_CHIPSELECT_NUM_A		GPIO_SSI0_CE0
#define	 SPI_CHIPSELECT_NUM_B		GPIO_SSI0_CE1
#define	 SPI_CHIPSELECT_NUM_C		GPIO_SSI1_CE0

#define  SPI0_BUS		0
#define  SPI1_BUS		1

#endif /* __ASM_JZ4750d_CETUS_H__ */
