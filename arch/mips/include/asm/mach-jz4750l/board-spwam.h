/*
 *  linux/include/asm-mips/mach-jz4750l/board-spwam.h
 *
 *  JZ4750L-based SPWAM board ver 1.x definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4750L_SPWAM_H__
#define __ASM_JZ4750L_SPWAM_H__


/*
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		12000000  /* Main extal oscillator: 12 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq: 32.768 KHz */
//#define CFG_DIV                 1         /* hclk=pclk=mclk=CFG_EXTAL/CFG_DIV, just for FPGA board */

/*
 * UART_BASE Select
 */
#define IRDA_BASE		UART0_BASE
#define UART_BASE		UART0_BASE

/*
 * ADKEYS
 */
#define GPIO_ADKEY_INT		(32*3+16) // KEY_INT - GPD16

/*
 *  ADKEYS LEVEL
 */
#define DPAD_MENU_LEVEL		244	//0.6V, SDATA = (4096*Vsacin)/3.3v
#define DPAD_UP_LEVEL		890	//1.0V
#define DPAD_DOWN_LEVEL		1486	//1.4V
#define DPAD_LEFT_LEVEL		1982	//1.75V 
#define DPAD_CENTER_LEVEL	2428	//2.0V	Enter key
#define DPAD_RIGHT_LEVEL	2870	//2.4V

/*
 * Analog input for VBAT is the battery voltage divided by CFG_PBAT_DIV.
 */
#define CFG_PBAT_DIV            1
#define is_share_mode() ((REG_EMC_BCR & EMC_BCR_BSR_MASK) == EMC_BCR_BSR_SHARE)
/*
 * Analog input for VBAT is the battery voltage divided by CFG_PBAT_DIV.
 */
#define ACTIVE_LOW_ADKEY	1


/*
 * GPIO
 */
#define GPIO_SD0_CD_N		GPIO_PC(1)	/* GPC1 */

#define GPIO_SD1_CD_N		GPIO_PC(4)	/* GPC4 */

#define GPIO_USB_DETE		GPIO_PD(6)	/* GPD6  */
#define GPIO_DC_DETE_N		GPIO_PD(7)	/* GPD7 */
#define GPIO_CHARG_STAT_N	GPIO_PD(15)	/* GPD15 */
#define GPIO_DISP_OFF_N		GPIO_PD(25)	/* GPD25, LCD_REV */
//#define GPIO_LED_EN       	GPIO_PD(28)	/* GPD28 */
#define GPIO_PEN_IRQ		GPIO_PD(22)	/* GPD22 */

#define GPIO_UDC_HOTPLUG	GPIO_USB_DETE

/*====================================================================== 
 * LCD backlight
 */
#define GPIO_LCD_PWM   		(32*2+14) /* GPE14 PWM4 */

#define LCD_MAX_BACKLIGHT		100
#define LCD_MIN_BACKLIGHT		1
#define LCD_DEFAULT_BACKLIGHT		80

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

#endif /* __ASM_JZ4750L_SPWAM_H__ */
