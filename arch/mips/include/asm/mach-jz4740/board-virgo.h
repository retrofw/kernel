/*
 *  linux/include/asm-mips/mach-jz4740/board-virgo.h
 *
 *  JZ4720-based VIRGO board ver 1.x definition.
 *
 *  Copyright (C) 2006 - 2007 Ingenic Semiconductor Inc.
 *
 *  Author: <lhhuang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4720_VIRGO_H__
#define __ASM_JZ4720_VIRGO_H__

/*====================================================================== 
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		12000000  /* Main extal freq: 12 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq: 32.768 KHz */

/*====================================================================== 
 * GPIO VIRGO(JZ4720)
 */
#define GPIO_SD_VCC_EN_N	115 /* GPD19 */	
#define GPIO_SD_CD_N		116 /* GPD20 */
#define GPIO_USB_DETE		114 /* GPD18 */
#define GPIO_DC_DETE_N		120 /* GPD24 */
#define GPIO_DISP_OFF_N		118 /* GPD22 */
#define GPIO_LED_EN       	117 /* GPD21 */

#define GPIO_UDC_HOTPLUG	GPIO_USB_DETE

/*====================================================================== 
 * MMC/SD
 */

#define MSC_HOTPLUG_PIN		GPIO_SD_CD_N
#define MSC_HOTPLUG_IRQ		(IRQ_GPIO_0 + GPIO_SD_CD_N)

#define __msc_init_io()				\
do {						\
	__gpio_as_output(GPIO_SD_VCC_EN_N);	\
	__gpio_as_input(GPIO_SD_CD_N);		\
} while (0)

#define __msc_enable_power()			\
do {						\
	__gpio_clear_pin(GPIO_SD_VCC_EN_N);	\
} while (0)

#define __msc_disable_power()			\
do {						\
	__gpio_set_pin(GPIO_SD_VCC_EN_N);	\
} while (0)

#define __msc_card_detected(s)			\
({						\
	int detected = 1;			\
	if (__gpio_get_pin(GPIO_SD_CD_N))	\
		detected = 0;			\
	detected;				\
})

#endif /* __ASM_JZ4720_VIRGO_H__ */
