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
#define __ASM_JZ4770_F4770_H__ //#define CONFIG_FPGA

/*======================================================================
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		12000000  /* Main extal freq:	12 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq:	32.768 KHz */
//#define CFG_DIV		        2	/* cpu/extclk; only for FPGA */

#define __GPIO(p, n) (32 * (p - 'A') + n)

#define UNUSED_GPIO_PIN  (0xffffffff)

#define TVE_MODE_PCLK 27000000

#define UMIDO_KEY_UP			__GPIO('E', 21)
#define UMIDO_KEY_DOWN			__GPIO('E', 25)
#define UMIDO_KEY_LEFT			__GPIO('E', 23)
#define UMIDO_KEY_RIGHT			__GPIO('E', 24)
#define UMIDO_KEY_A				__GPIO('E', 29)
#define UMIDO_KEY_B				__GPIO('E', 20)
#define UMIDO_KEY_X				__GPIO('E', 27)
#define UMIDO_KEY_Y				__GPIO('E', 28)
#define UMIDO_KEY_UP_L1			__GPIO('E', 22)
#define UMIDO_KEY_UP_L2			__GPIO('E', 26)
#define UMIDO_KEY_UP_R1			__GPIO('B', 31)
#define UMIDO_KEY_UP_R2			__GPIO('B', 28)
#define UMIDO_KEY_DOWN_L1		__GPIO('B', 20)
#define UMIDO_KEY_DOWN_L2		__GPIO('B', 21)
#define UMIDO_KEY_DOWN_R1		__GPIO('B', 29)
#define UMIDO_KEY_DOWN_R2		__GPIO('E', 11)
#define UMIDO_KEY_START			__GPIO('D', 18)
#define UMIDO_KEY_SELECT		__GPIO('B', 30)

#define UMIDO_KEY_UP_ACTIVELOW			1
#define UMIDO_KEY_DOWN_ACTIVELOW		1
#define UMIDO_KEY_LEFT_ACTIVELOW		1
#define UMIDO_KEY_RIGHT_ACTIVELOW		1
#define UMIDO_KEY_A_ACTIVELOW			1
#define UMIDO_KEY_B_ACTIVELOW			1
#define UMIDO_KEY_X_ACTIVELOW			1
#define UMIDO_KEY_Y_ACTIVELOW			1
#define UMIDO_KEY_UP_L1_ACTIVELOW		1
#define UMIDO_KEY_UP_L2_ACTIVELOW		1
#define UMIDO_KEY_UP_R1_ACTIVELOW		1
#define UMIDO_KEY_UP_R2_ACTIVELOW		1
#define UMIDO_KEY_DOWN_L1_ACTIVELOW		1
#define UMIDO_KEY_DOWN_L2_ACTIVELOW		1
#define UMIDO_KEY_DOWN_R1_ACTIVELOW		1
#define UMIDO_KEY_DOWN_R2_ACTIVELOW		1
#define UMIDO_KEY_START_ACTIVELOW		0
#define UMIDO_KEY_SELECT_ACTIVELOW		1

#define GPIO_AMPEN			__GPIO('F', 20)
#define GPIO_HP_PIN			__GPIO('B', 3)
#define GPIO_HP_OFF			__GPIO('F', 3)
#define GPIO_GSENSOR_VCC	__GPIO('E', 18)

#define GPIO_MOTO_PW		__GPIO('E', 4)
#define GPIO_DC_PIN			__GPIO('F', 5) //TV_OUT
#define TV_OUT_FLAG			1
#define AV_OUT_DETE			__GPIO('F', 21)
#define AV_OUT_DETE_IRQ		(IRQ_GPIO_0 + AV_OUT_DETE) //HDMI
#define HDMI_DETE_PIN		__GPIO('F', 19)
#define HDMI_DETE_PIN_IRQ	(IRQ_GPIO_0 + HDMI_DETE_PIN)

#ifdef CONFIG_HDMI_IT6610_OLD
#define HDMI_RST_N_PIN		__GPIO('E', 6)
#define SIO_C				__GPIO('D', 7)
#define SIO_D				__GPIO('D', 6)
#endif

#ifdef CONFIG_HDMI_IT6610
#define GPIO_HDMI_RST_N		__GPIO('E', 6)
#define GPIO_HDMI_PS2_KCLK	__GPIO('D', 7)
#define GPIO_HDMI_PS2_KDATA	__GPIO('D', 6)
#define GPIO_HDMI_HPD		__GPIO('F', 12)
#endif

/*======================================================================
 *
 *  USB OTG config
 */
#define GPIO_UDC_HOTPLUG			__GPIO('B', 10)
#define OTG_HOTPLUG_PIN				GPIO_UDC_HOTPLUG // (32 + 5)
#define GPIO_OTG_ID_PIN				__GPIO('F', 18)
#define OTG_HOTPLUG_IRQ				(IRQ_GPIO_0 + OTG_HOTPLUG_PIN)
#define GPIO_OTG_ID_IRQ				(IRQ_GPIO_0 + GPIO_OTG_ID_PIN)
#define GPIO_OTG_STABLE_JIFFIES		10

#define GPIO_I2C1_SDA				__GPIO('D', 5) /* GPE0 */
#define GPIO_I2C1_SCK				__GPIO('D', 4) /* GPE17 */

#define GPIO_TS_I2C_INT				__GPIO('E', 0)
#define GPIO_TS_I2C_IRQ				(IRQ_GPIO_0 + GPIO_TS_I2C_INT)

#define EP932M_RESET_PIN			__GPIO('E', 6)

/*
 *  msc0 config
 */
#if !defined(CONFIG_JZ_SYSTEM_AT_CARD)
	#define GPIO_SD0_VCC_EN_N	__GPIO('F', 20)
	#define GPIO_SD0_CD_N		__GPIO('A', 25)
	#define MSC0_WP_PIN			__GPIO('A', 26)
	#define MSC0_HOTPLUG_PIN	GPIO_SD0_CD_N
	#define MSC0_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD0_CD_N)
#else
	#define MSC0_HOTPLUG_PIN	UNUSED_GPIO_PIN
#endif
/*
 *   msc1 config
 */
#define GPIO_SD1_VCC_EN_N		__GPIO('E', 9)
#define GPIO_SD1_CD_N			__GPIO('B', 2)
#define MSC1_HOTPLUG_PIN		GPIO_SD1_CD_N
#define MSC1_HOTPLUG_IRQ		(IRQ_GPIO_0 + GPIO_SD1_CD_N)
/*
 *  msc2 config
 */
#define GPIO_SD2_VCC_EN_N		__GPIO('E', 19)
#define GPIO_SD2_CD_N			__GPIO('B', 12)
#define MSC2_HOTPLUG_PIN		GPIO_SD2_CD_N
#define MSC2_HOTPLUG_IRQ		(IRQ_GPIO_0 + GPIO_SD2_CD_N)

#define ACTIVE_LOW_MSC0_CD		1
#define ACTIVE_LOW_MSC1_CD		1
#define ACTIVE_LOW_MSC2_CD		1

/*
 *  ethernet config
*/
#define JZMAC_PHY_RESET_PIN		__GPIO('E', 8)

#define BUS_RJ45_PWR_EN			__GPIO('B', 12)
#define AX8872_PWR_EN			__GPIO('B', 6)
/*
* hub and ttys
*/
#define HUB_3V3_EN				__GPIO('B', 17)
#define SDIOWIFI_PWR_EN			__GPIO('D', 3)
#define PL_5V_EN				__GPIO('B', 18)
#define UART_3V3_EN				__GPIO('E', 17)

/*
* tty232-tty422-tty485 switch
*/
#define F81438_SLEW				__GPIO('E', 17)

#define F81438_MODE1			__GPIO('E', 14)
#define F81438_MODE2			__GPIO('E', 16)
#define F81438_SHUTDOWN			__GPIO('E', 18)

#define GPIO_DC_DETE_N			103 /* GPD7 */
#define GPIO_CHARG_STAT_N		111 /* GPD15 */
#define GPIO_DISP_OFF_N			121 /* GPD25, LCD_REV */
//#define GPIO_LED_EN			124 /* GPD28 */

#define GPIO_POWER_ON			__GPIO('A', 30)  /* GPA30 */
#define ACTIVE_LOW_WAKE_UP		1


/*======================================================================
 * LCD backlight
 */

#define GPIO_LCD_VCC_EN_N		__GPIO('E', 13) /*GPE13*/
#define GPIO_LCD_PWM			__GPIO('E', 1) /* GPE4 PWM4 */
#define LCD_PWM_CHN				1    /* pwm channel */
#define LCD_PWM_FULL			101
#define LCD_DEFAULT_BACKLIGHT	80
#define LCD_MAX_BACKLIGHT		100
#define LCD_MIN_BACKLIGHT		1


#define __lcd_init_backlight(n) \
do { \
	__lcd_set_backlight_level(n); \
} while (0)

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
	__tcu_set_full_data(LCD_PWM_CHN, JZ_EXTAL / 30000); \
	__tcu_set_half_data(LCD_PWM_CHN, JZ_EXTAL / 30000 * n / LCD_PWM_FULL); \
	__tcu_enable_pwm_output(LCD_PWM_CHN); \
	__tcu_select_extalclk(LCD_PWM_CHN); \
	__tcu_start_counter(LCD_PWM_CHN); \
} while (0)

#define __lcd_close_backlight() \
do { \
      __tcu_stop_counter(LCD_PWM_CHN); \
      __gpio_as_output0(GPIO_LCD_PWM); \
} while (0)

#define JZ_EARLY_UART_BASE		UART2_BASE

#define HAVE_OTHER_UART //this project need uart0 uart3

 /* use uart2 as default */
#define JZ_BOOTUP_UART_TXD		__GPIO('C', 30)
#define JZ_BOOTUP_UART_RXD		__GPIO('C', 28)

#define JZ_BOOTUP_UART0_TXD		__GPIO('F', 3)
#define JZ_BOOTUP_UART0_RXD		__GPIO('F', 0)

#define JZ_BOOTUP_UART3_TXD		__GPIO('E', 5)
#define JZ_BOOTUP_UART3_RXD		__GPIO('D', 12)

#endif /* __ASM_JZ4770_F4770_H__ */
