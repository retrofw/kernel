/*
 * linux/drivers/video/jzslcd.h -- Ingenic On-Chip SLCD frame buffer device
 *
 * Copyright (C) 2005-2007, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __JZSLCD_H__
#define __JZSLCD_H__

#define UINT16 unsigned short
#define UINT32 unsigned int

#define NR_PALETTE	256
/* Jz LCDFB supported I/O controls. */
#define FBIOSETBACKLIGHT	0x4688
#define FBIODISPON		0x4689
#define FBIODISPOFF		0x468a
#define FBIORESET		0x468b
#define FBIOPRINT_REG		0x468c
#define FBIO_REFRESH_ALWAYS	0x468d
#define FBIO_REFRESH_EVENTS	0x468e
#define FBIO_DO_REFRESH		0x468f
#define FBIO_SET_REG		0x4690

#ifdef CONFIG_JZ_SLCD_LGDP4551
#define PIN_CS_N 	(32*2+18)	/* Chip select      :SLCD_WR: GPC18 */ 
#define PIN_RESET_N 	(32*2+21)	/* LCD reset        :SLCD_RST: GPC21*/ 
#define PIN_RS_N 	(32*2+19)

#define	__slcd_special_pin_init() \
do { \
	__gpio_as_output(PIN_CS_N); 	\
	__gpio_as_output(PIN_RESET_N); 	\
	__gpio_clear_pin(PIN_CS_N); /* Clear CS */\
	mdelay(100);			\
} while(0)

#define __slcd_special_on() 		\
do {	/* RESET# */			\
	__gpio_set_pin(PIN_RESET_N);	\
	mdelay(10);			\
	__gpio_clear_pin(PIN_RESET_N);	\
	mdelay(10);			\
	__gpio_set_pin(PIN_RESET_N);	\
	mdelay(100);			\
	Mcupanel_RegSet(0x0015,0x0050);	\
	Mcupanel_RegSet(0x0011,0x0000);	\
	Mcupanel_RegSet(0x0010,0x3628);	\
	Mcupanel_RegSet(0x0012,0x0002);	\
	Mcupanel_RegSet(0x0013,0x0E47);	\
	udelay(100);			\
	Mcupanel_RegSet(0x0012,0x0012);	\
	udelay(100);			\
	Mcupanel_RegSet(0x0010,0x3620);	\
	Mcupanel_RegSet(0x0013,0x2E47);	\
	udelay(50);			\
	Mcupanel_RegSet(0x0030,0x0000);	\
	Mcupanel_RegSet(0x0031,0x0502);	\
	Mcupanel_RegSet(0x0032,0x0307);	\
	Mcupanel_RegSet(0x0033,0x0304);	\
	Mcupanel_RegSet(0x0034,0x0004);	\
	Mcupanel_RegSet(0x0035,0x0401);	\
	Mcupanel_RegSet(0x0036,0x0707);	\
	Mcupanel_RegSet(0x0037,0x0303);	\
	Mcupanel_RegSet(0x0038,0x1E02);	\
	Mcupanel_RegSet(0x0039,0x1E02);	\
	Mcupanel_RegSet(0x0001,0x0000);	\
	Mcupanel_RegSet(0x0002,0x0300);	\
	if (jzfb.bpp == 16)		\
		Mcupanel_RegSet(0x0003,0x10B8); /*8-bit system interface two transfers
						  up:0x10B8 down:0x1088 left:0x1090 right:0x10a0*/ \
	else	\
		if (jzfb.bpp == 32)\
			Mcupanel_RegSet(0x0003,0xD0B8);/*8-bit system interface three transfers,666
							 up:0xD0B8 down:0xD088 left:0xD090 right:0xD0A0*/ \
	Mcupanel_RegSet(0x0008,0x0204);\
	Mcupanel_RegSet(0x000A,0x0008);\
	Mcupanel_RegSet(0x0060,0x3100);\
	Mcupanel_RegSet(0x0061,0x0001);\
	Mcupanel_RegSet(0x0090,0x0052);\
	Mcupanel_RegSet(0x0092,0x000F);\
	Mcupanel_RegSet(0x0093,0x0001);\
	Mcupanel_RegSet(0x009A,0x0008);\
	Mcupanel_RegSet(0x00A3,0x0010);\
	Mcupanel_RegSet(0x0050,0x0000);\
	Mcupanel_RegSet(0x0051,0x00EF);\
	Mcupanel_RegSet(0x0052,0x0000);\
	Mcupanel_RegSet(0x0053,0x018F);\
	/*===Display_On_Function=== */ \
	Mcupanel_RegSet(0x0007,0x0001);\
	Mcupanel_RegSet(0x0007,0x0021);\
	Mcupanel_RegSet(0x0007,0x0023);\
	Mcupanel_RegSet(0x0007,0x0033);\
	Mcupanel_RegSet(0x0007,0x0133);\
	Mcupanel_Command(0x0022);/*Write Data to GRAM	*/  \
	udelay(1);		\
	Mcupanel_SetAddr(0,0);	\
	mdelay(100);		\
} while (0)

#define __slcd_special_off() 		\
do { \
} while(0)
#endif /*CONFIG_JZ_SLCD_LGDP4551_xxBUS*/

#ifdef CONFIG_JZ_SLCD_SPFD5420A

  //#define PIN_CS_N 	(32*2+18)	// Chip select 	//GPC18;
#define PIN_CS_N 	(32*2+22)	// Chip select 	//GPC18;
#define PIN_RESET_N 	(32*1+18)	// LCD reset   	//GPB18;
#define PIN_RS_N 	(32*2+19)	// LCD RS		//GPC19;
#define PIN_POWER_N	(32*3+0)	//Power off 	//GPD0;
#define PIN_FMARK_N	(32*3+1)	//fmark			//GPD1;

#define GAMMA()	\
do {	\
	Mcupanel_RegSet(0x0300,0x0101);	\
	Mcupanel_RegSet(0x0301,0x0b27);	\
	Mcupanel_RegSet(0x0302,0x132a);	\
	Mcupanel_RegSet(0x0303,0x2a13);	\
	Mcupanel_RegSet(0x0304,0x270b);	\
	Mcupanel_RegSet(0x0305,0x0101);	\
	Mcupanel_RegSet(0x0306,0x1205);	\
	Mcupanel_RegSet(0x0307,0x0512);	\
	Mcupanel_RegSet(0x0308,0x0005);	\
	Mcupanel_RegSet(0x0309,0x0003);	\
	Mcupanel_RegSet(0x030a,0x0f04);	\
	Mcupanel_RegSet(0x030b,0x0f00);	\
	Mcupanel_RegSet(0x030c,0x000f);	\
	Mcupanel_RegSet(0x030d,0x040f);	\
	Mcupanel_RegSet(0x030e,0x0300);	\
	Mcupanel_RegSet(0x030f,0x0500);	\
	/*** secorrect gamma2 ***/	\
	Mcupanel_RegSet(0x0400,0x3500);	\
	Mcupanel_RegSet(0x0401,0x0001);	\
	Mcupanel_RegSet(0x0404,0x0000);	\
	Mcupanel_RegSet(0x0500,0x0000);	\
	Mcupanel_RegSet(0x0501,0x0000);	\
	Mcupanel_RegSet(0x0502,0x0000);	\
	Mcupanel_RegSet(0x0503,0x0000);	\
	Mcupanel_RegSet(0x0504,0x0000);	\
	Mcupanel_RegSet(0x0505,0x0000);	\
	Mcupanel_RegSet(0x0600,0x0000);	\
	Mcupanel_RegSet(0x0606,0x0000);	\
	Mcupanel_RegSet(0x06f0,0x0000);	\
	Mcupanel_RegSet(0x07f0,0x5420);	\
	Mcupanel_RegSet(0x07f3,0x288a);	\
	Mcupanel_RegSet(0x07f4,0x0022);	\
	Mcupanel_RegSet(0x07f5,0x0001);	\
	Mcupanel_RegSet(0x07f0,0x0000);	\
} while(0)

#define __slcd_special_on()	\
do {      \
	__gpio_set_pin(PIN_RESET_N);	\
	mdelay(10);	\
	__gpio_clear_pin(PIN_RESET_N);	\
	mdelay(10);	\
	__gpio_set_pin(PIN_RESET_N);	\
	mdelay(100);	\
	if (jzfb.bus == 18) {\
	Mcupanel_RegSet(0x0606,0x0000);	\
	udelay(10);	\
	Mcupanel_RegSet(0x0007,0x0001);	\
	udelay(10);	\
	Mcupanel_RegSet(0x0110,0x0001);	\
	udelay(10);	\
	Mcupanel_RegSet(0x0100,0x17b0);	\
	Mcupanel_RegSet(0x0101,0x0147);	\
	Mcupanel_RegSet(0x0102,0x019d);	\
	Mcupanel_RegSet(0x0103,0x8600);	\
	Mcupanel_RegSet(0x0281,0x0010);	\
	udelay(10);	\
	Mcupanel_RegSet(0x0102,0x01bd);	\
	udelay(10);	\
	/************initial************/\
	Mcupanel_RegSet(0x0000,0x0000);	\
	Mcupanel_RegSet(0x0001,0x0000);	\
	Mcupanel_RegSet(0x0002,0x0400);	\
	Mcupanel_RegSet(0x0003,0x1288); /*up:0x1288 down:0x12B8 left:0x1290 right:0x12A0*/ \
	Mcupanel_RegSet(0x0006,0x0000);	\
	Mcupanel_RegSet(0x0008,0x0503);	\
	Mcupanel_RegSet(0x0009,0x0001);	\
	Mcupanel_RegSet(0x000b,0x0010);	\
	Mcupanel_RegSet(0x000c,0x0000);	\
	Mcupanel_RegSet(0x000f,0x0000);	\
	Mcupanel_RegSet(0x0007,0x0001);	\
	Mcupanel_RegSet(0x0010,0x0010);	\
	Mcupanel_RegSet(0x0011,0x0202);	\
	Mcupanel_RegSet(0x0012,0x0300);	\
	Mcupanel_RegSet(0x0020,0x021e);	\
	Mcupanel_RegSet(0x0021,0x0202);	\
	Mcupanel_RegSet(0x0022,0x0100);	\
	Mcupanel_RegSet(0x0090,0x0000);	\
	Mcupanel_RegSet(0x0092,0x0000);	\
	Mcupanel_RegSet(0x0100,0x16b0);	\
	Mcupanel_RegSet(0x0101,0x0147);	\
	Mcupanel_RegSet(0x0102,0x01bd);	\
	Mcupanel_RegSet(0x0103,0x2c00);	\
    	Mcupanel_RegSet(0x0107,0x0000);	\
	Mcupanel_RegSet(0x0110,0x0001);	\
	Mcupanel_RegSet(0x0210,0x0000);	\
	Mcupanel_RegSet(0x0211,0x00ef);	\
	Mcupanel_RegSet(0x0212,0x0000);	\
	Mcupanel_RegSet(0x0213,0x018f);	\
	Mcupanel_RegSet(0x0280,0x0000);	\
	Mcupanel_RegSet(0x0281,0x0001);	\
	Mcupanel_RegSet(0x0282,0x0000);	\
	GAMMA();	\
 	Mcupanel_RegSet(0x0007,0x0173);	\
	} else {		\
		Mcupanel_RegSet(0x0600, 0x0001);   /*soft reset*/	\
		mdelay(10); 		\
		Mcupanel_RegSet(0x0600, 0x0000);   /*soft reset*/	\
		mdelay(10);						\
		Mcupanel_RegSet(0x0606, 0x0000);   /*i80-i/F Endian Control*/ \
		/*===User setting===    */				\
		Mcupanel_RegSet(0x0001, 0x0000);/* Driver Output Control-----0x0100 SM(bit10) | 0x400*/ \
		Mcupanel_RegSet(0x0002, 0x0100);   /*LCD Driving Wave Control      0x0100 */ \
		if (jzfb.bpp == 16)					\
			Mcupanel_RegSet(0x0003, 0x50A8);/*Entry Mode 0x1030*/ \
		else /*bpp = 18*/					\
			Mcupanel_RegSet(0x0003, 0x1010 | 0xC8);   /*Entry Mode 0x1030*/	\
		/*#endif								*/ \
		Mcupanel_RegSet(0x0006, 0x0000);   /*Outline Sharpening Control*/\     
		Mcupanel_RegSet(0x0008, 0x0808);   /*Sets the number of lines for front/back porch period*/\
		Mcupanel_RegSet(0x0009, 0x0001);   /*Display Control 3   */\ 
		Mcupanel_RegSet(0x000B, 0x0010);   /*Low Power Control*/\
		Mcupanel_RegSet(0x000C, 0x0000);   /*External Display Interface Control 1 /*0x0001*/\
		Mcupanel_RegSet(0x000F, 0x0000);   /*External Display Interface Control 2         */\
		Mcupanel_RegSet(0x0400, 0xB104);/*Base Image Number of Line---GS(bit15) | 0x8000*/ \
		Mcupanel_RegSet(0x0401, 0x0001);   /*Base Image Display        0x0001*/\
		Mcupanel_RegSet(0x0404, 0x0000);   /*Base Image Vertical Scroll Control    0x0000*/\
		Mcupanel_RegSet(0x0500, 0x0000);   /*Partial Image 1: Display Position*/\
		Mcupanel_RegSet(0x0501, 0x0000);   /*RAM Address (Start Line Address) */\
		Mcupanel_RegSet(0x0502, 0x018f);   /*RAM Address (End Line Address)  */	\
		Mcupanel_RegSet(0x0503, 0x0000);   /*Partial Image 2: Display Position  RAM Address*/\
		Mcupanel_RegSet(0x0504, 0x0000);   /*RAM Address (Start Line Address) */\
		Mcupanel_RegSet(0x0505, 0x0000);   /*RAM Address (End Line Address)*/\
		/*Panel interface control===*/\
		Mcupanel_RegSet(0x0010, 0x0011);   /*Division Ratio,Clocks per Line  14  */\
		mdelay(10); \
		Mcupanel_RegSet(0x0011, 0x0202);   /*Division Ratio,Clocks per Line*/\
		Mcupanel_RegSet(0x0012, 0x0300);   /*Sets low power VCOM drive period.   */\
		mdelay(10); \
		Mcupanel_RegSet(0x0020, 0x021e);   /*Panel Interface Control 4  */\
		Mcupanel_RegSet(0x0021, 0x0202);   /*Panel Interface Control 5 */\
		Mcupanel_RegSet(0x0022, 0x0100);   /*Panel Interface Control 6*/\ 
		Mcupanel_RegSet(0x0090, 0x0000);   /*Frame Marker Control  */\
		Mcupanel_RegSet(0x0092, 0x0000);   /*MDDI Sub-display Control  */\
		/*===Gamma setting===    */\
		Mcupanel_RegSet(0x0300, 0x0101);   /*γ Control*/\
		Mcupanel_RegSet(0x0301, 0x0000);   /*γ Control*/\
		Mcupanel_RegSet(0x0302, 0x0016);   /*γ Control*/\
		Mcupanel_RegSet(0x0303, 0x2913);   /*γ Control*/\
		Mcupanel_RegSet(0x0304, 0x260B);   /*γ Control*/\
		Mcupanel_RegSet(0x0305, 0x0101);   /*γ Control*/\
		Mcupanel_RegSet(0x0306, 0x1204);   /*γ Control*/\
		Mcupanel_RegSet(0x0307, 0x0415);   /*γ Control*/\
		Mcupanel_RegSet(0x0308, 0x0205);   /*γ Control*/\
		Mcupanel_RegSet(0x0309, 0x0303);   /*γ Control*/\
		Mcupanel_RegSet(0x030a, 0x0E05);   /*γ Control*/\
		Mcupanel_RegSet(0x030b, 0x0D01);   /*γ Control*/\
		Mcupanel_RegSet(0x030c, 0x010D);   /*γ Control*/\
		Mcupanel_RegSet(0x030d, 0x050E);   /*γ Control*/\
		Mcupanel_RegSet(0x030e, 0x0303);   /*γ Control*/\
		Mcupanel_RegSet(0x030f, 0x0502);   /*γ Control*/\
		/*===Power on sequence===*/\
		Mcupanel_RegSet(0x0007, 0x0001);   /*Display Control 1*/\
		Mcupanel_RegSet(0x0110, 0x0001);   /*Power supply startup enable bit*/\
		Mcupanel_RegSet(0x0112, 0x0060);   /*Power Control 7*/\
		Mcupanel_RegSet(0x0100, 0x16B0);   /*Power Control 1 */\
		Mcupanel_RegSet(0x0101, 0x0115);   /*Power Control 2*/\
		Mcupanel_RegSet(0x0102, 0x0119);   /*Starts VLOUT3,Sets the VREG1OUT.*/\
		mdelay(50); \
		Mcupanel_RegSet(0x0103, 0x2E00);   /*set the amplitude of VCOM*/\
		mdelay(50);\
		Mcupanel_RegSet(0x0282, 0x0093);/*0x008E);/*0x0093);   /*VCOMH voltage*/\
		Mcupanel_RegSet(0x0281, 0x000A);   /*Selects the factor of VREG1OUT to generate VCOMH. */\
		Mcupanel_RegSet(0x0102, 0x01BE);   /*Starts VLOUT3,Sets the VREG1OUT.*/\
		mdelay(10);\
		/*Address */\
		Mcupanel_RegSet(0x0210, 0x0000);   /*Window Horizontal RAM Address Start*/\
		Mcupanel_RegSet(0x0211, 0x00ef);   /*Window Horizontal RAM Address End*/\
		Mcupanel_RegSet(0x0212, 0x0000);   /*Window Vertical RAM Address Start*/\
		Mcupanel_RegSet(0x0213, 0x018f);   /*Window Vertical RAM Address End */\
		Mcupanel_RegSet(0x0200, 0x0000);   /*RAM Address Set (Horizontal Address)*/\
		Mcupanel_RegSet(0x0201, 0x018f);   /*RAM Address Set (Vertical Address)*/ \
		/*===Display_On_Function===*/\
		Mcupanel_RegSet(0x0007, 0x0021);   /*Display Control 1 */\
		mdelay(50);   /*40*/\
		Mcupanel_RegSet(0x0007, 0x0061);   /*Display Control 1 */\
		mdelay(50);   /*100*/\
		Mcupanel_RegSet(0x0007, 0x0173);   /*Display Control 1 */\
		mdelay(50);   /*300*/\
	}\
	  Mcupanel_Command(0x0202);                  /*Write Data to GRAM	*/  \
	udelay(10);\
	Mcupanel_SetAddr(0,0);\
	udelay(100);\
} while(0)

#define __slcd_special_pin_init() \
do {	\
	__gpio_as_output(PIN_CS_N);	\
	__gpio_as_output(PIN_RESET_N);	\
	__gpio_clear_pin(PIN_CS_N); /* Clear CS */	\
	__gpio_as_output(PIN_POWER_N);	\
	mdelay(100);	\
} while(0)

#endif /*CONFIG_JZ_SLCD_SPFD5420A*/

#ifndef __slcd_special_pin_init
#define __slcd_special_pin_init()
#endif
#ifndef __slcd_special_on
#define __slcd_special_on()
#endif
#ifndef __slcd_special_off
#define __slcd_special_off()
#endif

/*
 * Platform specific definition
 */
#if defined(CONFIG_SOC_JZ4740)
#if defined(CONFIG_JZ4740_PAVO)
#define GPIO_PWM    123		/* GP_D27 */
#define PWM_CHN 4    /* pwm channel */
#define PWM_FULL 101
/* 100 level: 0,1,...,100 */
#define __slcd_set_backlight_level(n)\
do { \
	__gpio_as_output(32*3+27);	\
	__gpio_set_pin(32*3+27);	\
} while (0)

#define __slcd_close_backlight() \
do { \
	__gpio_as_output(GPIO_PWM);	\
	__gpio_clear_pin(GPIO_PWM);	\
} while (0)

#else

#define __slcd_set_backlight_level(n)
#define __slcd_close_backlight()

#endif /* #if defined(CONFIG_MIPS_JZ4740_PAVO) */

#define __slcd_display_pin_init() \
do { \
	__slcd_special_pin_init(); \
} while (0)

#define __slcd_display_on() \
do { \
	__slcd_special_on(); \
	__slcd_set_backlight_level(80); \
} while (0)

#define __slcd_display_off() \
do { \
	__slcd_special_off(); \
	__slcd_close_backlight(); \
} while (0)

#endif /* CONFIG_SOC_JZ4740 */
#endif  /*__JZSLCD_H__*/

