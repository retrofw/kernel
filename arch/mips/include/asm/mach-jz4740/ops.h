/*
 *  linux/include/asm-mips/mach-jz4740/ops.h
 *
 *  Ingenic's JZ4740 common include.
 *
 *  Copyright (C) 2006 - 2007 Ingenic Semiconductor Inc.
 *
 *  Author: <yliu@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef __JZ4740_OPS_H__
#define __JZ4740_OPS_H__

/*
 * Definition of Module Operations
 */

/***************************************************************************
 * GPIO
 ***************************************************************************/

//------------------------------------------------------
// GPIO Pins Description
//
// PORT 0:
//
// PIN/BIT N		FUNC0		FUNC1
//	0		D0		-
//	1		D1		-
//	2		D2		-
//	3		D3		-
//	4		D4		-
//	5		D5		-
//	6		D6		-
//	7		D7		-
//	8		D8		-
//	9		D9		-
//	10		D10		-
//	11		D11		-
//	12		D12		-
//	13		D13		-
//	14		D14		-
//	15		D15		-
//	16		D16		-
//	17		D17		-
//	18		D18		-
//	19		D19		-
//	20		D20		-
//	21		D21		-
//	22		D22		-
//	23		D23		-
//	24		D24		-
//	25		D25		-
//	26		D26		-
//	27		D27		-
//	28		D28		-
//	29		D29		-
//	30		D30		-
//	31		D31		-
//
//------------------------------------------------------
// PORT 1:
//
// PIN/BIT N		FUNC0		FUNC1
//	0		A0		-
//	1		A1		-
//	2		A2		-
//	3		A3		-
//	4		A4		-
//	5		A5		-
//	6		A6		-
//	7		A7		-
//	8		A8		-
//	9		A9		-
//	10		A10		-
//	11		A11		-
//	12		A12		-
//	13		A13		-
//	14		A14		-
//	15		A15/CL		-
//	16		A16/AL		-
//	17		LCD_CLS		A21
//	18		LCD_SPL		A22
//	19		DCS#		-
//	20		RAS#		-
//	21		CAS#		-
//	22		RDWE#/BUFD#	-
//	23		CKE		-
//	24		CKO		-
//	25		CS1#		-
//	26		CS2#		-
//	27		CS3#		-
//	28		CS4#		-
//	29		RD#		-
//	30		WR#		-
//	31		WE0#		-
//
// Note: PIN15&16 are CL&AL when connecting to NAND flash.
//------------------------------------------------------
// PORT 2:
//
// PIN/BIT N		FUNC0		FUNC1
//	0		LCD_D0		-
//	1		LCD_D1		-
//	2		LCD_D2		-
//	3		LCD_D3		-
//	4		LCD_D4		-
//	5		LCD_D5		-
//	6		LCD_D6		-
//	7		LCD_D7		-
//	8		LCD_D8		-
//	9		LCD_D9		-
//	10		LCD_D10		-
//	11		LCD_D11		-
//	12		LCD_D12		-
//	13		LCD_D13		-
//	14		LCD_D14		-
//	15		LCD_D15		-
//	16		LCD_D16		-
//	17		LCD_D17		-
//	18		LCD_PCLK	-
//	19		LCD_HSYNC	-
//	20		LCD_VSYNC	-
//	21		LCD_DE		-
//	22		LCD_PS		A19
//	23		LCD_REV		A20
//	24		WE1#		-
//	25		WE2#		-
//	26		WE3#		-
//	27		WAIT#		-
//	28		FRE#		-
//	29		FWE#		-
//	30(NOTE:FRB#)	-		-
//	31		-		-
//
// NOTE(1): PIN30 is used for FRB# when connecting to NAND flash.
//------------------------------------------------------
// PORT 3:
//
// PIN/BIT N		FUNC0		FUNC1
//	0		CIM_D0		-
//	1		CIM_D1		-
//	2		CIM_D2		-
//	3		CIM_D3		-
//	4		CIM_D4		-
//	5		CIM_D5		-
//	6		CIM_D6		-
//	7		CIM_D7		-
//	8		MSC_CMD		-
//	9		MSC_CLK		-
//	10		MSC_D0		-
//	11		MSC_D1		-
//	12		MSC_D2		-
//	13		MSC_D3		-
//	14		CIM_MCLK	-
//	15		CIM_PCLK	-
//	16		CIM_VSYNC	-
//	17		CIM_HSYNC	-
//	18		SSI_CLK		SCLK_RSTN
//	19		SSI_CE0#	BIT_CLK(AIC)
//	20		SSI_DT		SDATA_OUT(AIC)
//	21		SSI_DR		SDATA_IN(AIC)
//	22		SSI_CE1#&GPC	SYNC(AIC)
//	23		PWM0		I2C_SDA
//	24		PWM1		I2C_SCK
//	25		PWM2		UART0_TxD
//	26		PWM3		UART0_RxD
//	27		PWM4		A17
//	28		PWM5		A18
//	29		-		-
//	30		PWM6		UART0_CTS/UART1_RxD
//	31		PWM7		UART0_RTS/UART1_TxD
//
//////////////////////////////////////////////////////////

/* 
 * p is the port number (0,1,2,3)
 * o is the pin offset (0-31) inside the port
 * n is the absolute number of a pin (0-127), regardless of the port
 */

//-------------------------------------------
// Function Pins Mode

#define __gpio_as_func0(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXFUNS(p) = (1 << o);		\
	REG_GPIO_PXSELC(p) = (1 << o);		\
} while (0)

#define __gpio_as_func1(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXFUNS(p) = (1 << o);		\
	REG_GPIO_PXSELS(p) = (1 << o);		\
} while (0)

/*
 * D0 ~ D31, A0 ~ A16, DCS#, RAS#, CAS#, CKE#, 
 * RDWE#, CKO#, WE0#, WE1#, WE2#, WE3#
 */
#define __gpio_as_sdram_32bit()			\
do {						\
	REG_GPIO_PXFUNS(0) = 0xffffffff;	\
	REG_GPIO_PXSELC(0) = 0xffffffff;	\
	REG_GPIO_PXPES(0) = 0xffffffff;		\
	REG_GPIO_PXFUNS(1) = 0x81f9ffff;	\
	REG_GPIO_PXSELC(1) = 0x81f9ffff;	\
	REG_GPIO_PXPES(1) = 0x81f9ffff;		\
	REG_GPIO_PXFUNS(2) = 0x07000000;	\
	REG_GPIO_PXSELC(2) = 0x07000000;	\
	REG_GPIO_PXPES(2) = 0x07000000;		\
} while (0)

/*
 * D0 ~ D15, A0 ~ A16, DCS#, RAS#, CAS#, CKE#, 
 * RDWE#, CKO#, WE0#, WE1#
 */
#define __gpio_as_sdram_16bit()			\
do {						\
	REG_GPIO_PXFUNS(0) = 0x5442bfaa;	\
	REG_GPIO_PXSELC(0) = 0x5442bfaa;	\
	REG_GPIO_PXPES(0) = 0x5442bfaa;		\
	REG_GPIO_PXFUNS(1) = 0x81f9ffff;	\
	REG_GPIO_PXSELC(1) = 0x81f9ffff;	\
	REG_GPIO_PXPES(1) = 0x81f9ffff;		\
	REG_GPIO_PXFUNS(2) = 0x01000000;	\
	REG_GPIO_PXSELC(2) = 0x01000000;	\
	REG_GPIO_PXPES(2) = 0x01000000;		\
} while (0)

/*
 * CS1#, CLE, ALE, FRE#, FWE#, FRB#, RDWE#/BUFD#
 */
#define __gpio_as_nand()			\
do {						\
	REG_GPIO_PXFUNS(1) = 0x02018000;	\
	REG_GPIO_PXSELC(1) = 0x02018000;	\
	REG_GPIO_PXPES(1) = 0x02018000;		\
	REG_GPIO_PXFUNS(2) = 0x30000000;	\
	REG_GPIO_PXSELC(2) = 0x30000000;	\
	REG_GPIO_PXPES(2) = 0x30000000;		\
	REG_GPIO_PXFUNC(2) = 0x40000000;	\
	REG_GPIO_PXSELC(2) = 0x40000000;	\
	REG_GPIO_PXDIRC(2) = 0x40000000;	\
	REG_GPIO_PXPES(2) = 0x40000000;		\
	REG_GPIO_PXFUNS(1) = 0x00400000;	\
	REG_GPIO_PXSELC(1) = 0x00400000;	\
} while (0)

/*
 * CS4#, RD#, WR#, WAIT#, A0 ~ A22, D0 ~ D7
 */
#define __gpio_as_nor_8bit()			\
do {						\
	REG_GPIO_PXFUNS(0) = 0x000000ff;	\
	REG_GPIO_PXSELC(0) = 0x000000ff;	\
	REG_GPIO_PXPES(0) = 0x000000ff;		\
	REG_GPIO_PXFUNS(1) = 0x7041ffff;	\
	REG_GPIO_PXSELC(1) = 0x7041ffff;	\
	REG_GPIO_PXPES(1) = 0x7041ffff;		\
	REG_GPIO_PXFUNS(1) = 0x00060000;	\
	REG_GPIO_PXSELS(1) = 0x00060000;	\
	REG_GPIO_PXPES(1) = 0x00060000;		\
	REG_GPIO_PXFUNS(2) = 0x08000000;	\
	REG_GPIO_PXSELC(2) = 0x08000000;	\
	REG_GPIO_PXPES(2) = 0x08000000;		\
	REG_GPIO_PXFUNS(2) = 0x00c00000;	\
	REG_GPIO_PXSELS(2) = 0x00c00000;	\
	REG_GPIO_PXPES(2) = 0x00c00000;		\
	REG_GPIO_PXFUNS(3) = 0x18000000;	\
	REG_GPIO_PXSELS(3) = 0x18000000;	\
	REG_GPIO_PXPES(3) = 0x18000000;		\
} while (0)

/*
 * CS4#, RD#, WR#, WAIT#, A0 ~ A22, D0 ~ D15
 */
#define __gpio_as_nor_16bit()			\
do {						\
	REG_GPIO_PXFUNS(0) = 0x0000ffff;	\
	REG_GPIO_PXSELC(0) = 0x0000ffff;	\
	REG_GPIO_PXPES(0) = 0x0000ffff;		\
	REG_GPIO_PXFUNS(1) = 0x7041ffff;	\
	REG_GPIO_PXSELC(1) = 0x7041ffff;	\
	REG_GPIO_PXPES(1) = 0x7041ffff;		\
	REG_GPIO_PXFUNS(1) = 0x00060000;	\
	REG_GPIO_PXSELS(1) = 0x00060000;	\
	REG_GPIO_PXPES(1) = 0x00060000;		\
	REG_GPIO_PXFUNS(2) = 0x08000000;	\
	REG_GPIO_PXSELC(2) = 0x08000000;	\
	REG_GPIO_PXPES(2) = 0x08000000;		\
	REG_GPIO_PXFUNS(2) = 0x00c00000;	\
	REG_GPIO_PXSELS(2) = 0x00c00000;	\
	REG_GPIO_PXPES(2) = 0x00c00000;		\
	REG_GPIO_PXFUNS(3) = 0x18000000;	\
	REG_GPIO_PXSELS(3) = 0x18000000;	\
	REG_GPIO_PXPES(3) = 0x18000000;		\
} while (0)

/*
 * UART0_TxD, UART_RxD0
 */
#define __gpio_as_uart0()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x06000000;	\
	REG_GPIO_PXSELS(3) = 0x06000000;	\
	REG_GPIO_PXPES(3) = 0x06000000;		\
} while (0)

/*
 * UART0_CTS, UART0_RTS
 */
#define __gpio_as_ctsrts()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0xc0000000;	\
	REG_GPIO_PXSELS(3) = 0xc0000000;	\
	REG_GPIO_PXTRGC(3) = 0xc0000000;	\
	REG_GPIO_PXPES(3) = 0xc0000000;		\
} while (0)

/*
 * UART1_TxD, UART1_RxD1
 */
#define __gpio_as_uart1()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0xc0000000;	\
	REG_GPIO_PXSELC(3) = 0xc0000000;	\
	REG_GPIO_PXTRGS(3) = 0xc0000000;	\
	REG_GPIO_PXPES(3) = 0xc0000000;		\
} while (0)

/*
 * LCD_D0~LCD_D15, LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_16bit()			\
do {						\
	REG_GPIO_PXFUNS(2) = 0x003cffff;	\
	REG_GPIO_PXSELC(2) = 0x003cffff;	\
	REG_GPIO_PXPES(2) = 0x003cffff;		\
} while (0)

/*
 * LCD_D0~LCD_D17, LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_18bit()			\
do {						\
	REG_GPIO_PXFUNS(2) = 0x003fffff;	\
	REG_GPIO_PXSELC(2) = 0x003fffff;	\
	REG_GPIO_PXPES(2) = 0x003fffff;		\
} while (0)

/*
 * LCD_PS, LCD_REV, LCD_CLS, LCD_SPL
 */
#define __gpio_as_lcd_special()			\
do {						\
	REG_GPIO_PXFUNS(1) = 0x00060000;	\
	REG_GPIO_PXSELC(1) = 0x00060000;	\
	REG_GPIO_PXPES(1)  = 0x00060000;	\
	REG_GPIO_PXFUNS(2) = 0x00c00000;	\
	REG_GPIO_PXSELC(2) = 0x00c00000;	\
	REG_GPIO_PXPES(2)  = 0x00c00000;	\
} while (0)

/* LCD_D0~LCD_D7, SLCD_RS, SLCD_CS */
#define __gpio_as_slcd_8bit()			\
do {						\
	REG_GPIO_PXFUNS(2) = 0x001800ff;	\
	REG_GPIO_PXSELC(2) = 0x001800ff;	\
} while (0)

/* LCD_D0~LCD_D7, SLCD_RS, SLCD_CS */
#define __gpio_as_slcd_9bit()			\
do {						\
	REG_GPIO_PXFUNS(2) = 0x001801ff;	\
	REG_GPIO_PXSELC(2) = 0x001801ff;	\
} while (0)

/* LCD_D0~LCD_D15, SLCD_RS, SLCD_CS */
#define __gpio_as_slcd_16bit()			\
do {						\
	REG_GPIO_PXFUNS(2) = 0x0018ffff;	\
	REG_GPIO_PXSELC(2) = 0x0018ffff;	\
} while (0)

/* LCD_D0~LCD_D17, SLCD_RS, SLCD_CS */
#define __gpio_as_slcd_18bit()			\
do {						\
	REG_GPIO_PXFUNS(2) = 0x001bffff;	\
	REG_GPIO_PXSELC(2) = 0x001bffff;	\
} while (0)

/*
 * CIM_D0~CIM_D7, CIM_MCLK, CIM_PCLK, CIM_VSYNC, CIM_HSYNC
 */
#define __gpio_as_cim()				\
do {						\
	REG_GPIO_PXFUNS(3) = 0x0003c0ff;	\
	REG_GPIO_PXSELC(3) = 0x0003c0ff;	\
	REG_GPIO_PXPES(3) = 0x0003c0ff;		\
} while (0)

/*
 * SDATA_OUT, SDATA_IN, BIT_CLK, SYNC, SCLK_RESET
 */
#define __gpio_as_aic()				\
do {						\
	REG_GPIO_PXFUNS(3) = 0x007c0000;	\
	REG_GPIO_PXSELS(3) = 0x007c0000;	\
	REG_GPIO_PXPES(3) = 0x007c0000;		\
} while (0)

/*
 * MSC_CMD, MSC_CLK, MSC_D0 ~ MSC_D3
 */
#define __gpio_as_msc()				\
do {						\
	REG_GPIO_PXFUNS(3) = 0x00003f00;	\
	REG_GPIO_PXSELC(3) = 0x00003f00;	\
	REG_GPIO_PXPES(3) = 0x00003f00;		\
} while (0)

/*
 * SSI_CS0, SSI_CLK, SSI_DT, SSI_DR
 */
#define __gpio_as_ssi()				\
do {						\
	REG_GPIO_PXFUNS(3) = 0x003c0000;	\
	REG_GPIO_PXSELC(3) = 0x003c0000;	\
	REG_GPIO_PXPES(3) = 0x003c0000;		\
} while (0)

/*
 * I2C_SCK, I2C_SDA
 */
#define __gpio_as_i2c()				\
do {						\
	REG_GPIO_PXFUNS(3) = 0x01800000;	\
	REG_GPIO_PXSELS(3) = 0x01800000;	\
	REG_GPIO_PXPES(3) = 0x01800000;		\
} while (0)

/*
 * PWM0
 */
#define __gpio_as_pwm0()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x00800000;	\
	REG_GPIO_PXSELC(3) = 0x00800000;	\
	REG_GPIO_PXPES(3) = 0x00800000;		\
} while (0)

/*
 * PWM1
 */
#define __gpio_as_pwm1()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x01000000;	\
	REG_GPIO_PXSELC(3) = 0x01000000;	\
	REG_GPIO_PXPES(3) = 0x01000000;		\
} while (0)

/*
 * PWM2
 */
#define __gpio_as_pwm2()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x02000000;	\
	REG_GPIO_PXSELC(3) = 0x02000000;	\
	REG_GPIO_PXPES(3) = 0x02000000;		\
} while (0)

/*
 * PWM3
 */
#define __gpio_as_pwm3()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x04000000;	\
	REG_GPIO_PXSELC(3) = 0x04000000;	\
	REG_GPIO_PXPES(3) = 0x04000000;		\
} while (0)

/*
 * PWM4
 */
#define __gpio_as_pwm4()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x08000000;	\
	REG_GPIO_PXSELC(3) = 0x08000000;	\
	REG_GPIO_PXPES(3) = 0x08000000;		\
} while (0)

/*
 * PWM5
 */
#define __gpio_as_pwm5()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x10000000;	\
	REG_GPIO_PXSELC(3) = 0x10000000;	\
	REG_GPIO_PXPES(3) = 0x10000000;		\
} while (0)

/*
 * PWM6
 */
#define __gpio_as_pwm6()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x40000000;	\
	REG_GPIO_PXSELC(3) = 0x40000000;	\
	REG_GPIO_PXPES(3) = 0x40000000;		\
} while (0)

/*
 * PWM7
 */
#define __gpio_as_pwm7()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x80000000;	\
	REG_GPIO_PXSELC(3) = 0x80000000;	\
	REG_GPIO_PXPES(3) = 0x80000000;		\
} while (0)

/*
 * n = 0 ~ 7
 */
#define __gpio_as_pwm(n)	__gpio_as_pwm##n()

//-------------------------------------------
// GPIO or Interrupt Mode

#define __gpio_get_port(p)	(REG_GPIO_PXPIN(p))

#define __gpio_port_as_output(p, o)		\
do {						\
    REG_GPIO_PXFUNC(p) = (1 << (o));		\
    REG_GPIO_PXSELC(p) = (1 << (o));		\
    REG_GPIO_PXDIRS(p) = (1 << (o));		\
} while (0)

#define __gpio_port_as_input(p, o)		\
do {						\
    REG_GPIO_PXFUNC(p) = (1 << (o));		\
    REG_GPIO_PXSELC(p) = (1 << (o));		\
    REG_GPIO_PXDIRC(p) = (1 << (o));		\
} while (0)

#define __gpio_as_output(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	__gpio_port_as_output(p, o);		\
} while (0)

#define __gpio_as_input(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	__gpio_port_as_input(p, o);		\
} while (0)

#define __gpio_set_pin(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXDATS(p) = (1 << o);		\
} while (0)

#define __gpio_clear_pin(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXDATC(p) = (1 << o);		\
} while (0)

#define __gpio_get_pin(n)			\
({						\
	unsigned int p, o, v;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	if (__gpio_get_port(p) & (1 << o))	\
		v = 1;				\
	else					\
		v = 0;				\
	v;					\
})

#define __gpio_as_irq_high_level(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMS(p) = (1 << o);		\
	REG_GPIO_PXTRGC(p) = (1 << o);		\
	REG_GPIO_PXFUNC(p) = (1 << o);		\
	REG_GPIO_PXSELS(p) = (1 << o);		\
	REG_GPIO_PXDIRS(p) = (1 << o);		\
	REG_GPIO_PXFLGC(p) = (1 << o);		\
	REG_GPIO_PXIMC(p) = (1 << o);		\
} while (0)

#define __gpio_as_irq_low_level(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMS(p) = (1 << o);		\
	REG_GPIO_PXTRGC(p) = (1 << o);		\
	REG_GPIO_PXFUNC(p) = (1 << o);		\
	REG_GPIO_PXSELS(p) = (1 << o);		\
	REG_GPIO_PXDIRC(p) = (1 << o);		\
	REG_GPIO_PXFLGC(p) = (1 << o);		\
	REG_GPIO_PXIMC(p) = (1 << o);		\
} while (0)

#define __gpio_as_irq_rise_edge(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMS(p) = (1 << o);		\
	REG_GPIO_PXTRGS(p) = (1 << o);		\
	REG_GPIO_PXFUNC(p) = (1 << o);		\
	REG_GPIO_PXSELS(p) = (1 << o);		\
	REG_GPIO_PXDIRS(p) = (1 << o);		\
	REG_GPIO_PXFLGC(p) = (1 << o);		\
	REG_GPIO_PXIMC(p) = (1 << o);		\
} while (0)

#define __gpio_as_irq_fall_edge(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMS(p) = (1 << o);		\
	REG_GPIO_PXTRGS(p) = (1 << o);		\
	REG_GPIO_PXFUNC(p) = (1 << o);		\
	REG_GPIO_PXSELS(p) = (1 << o);		\
	REG_GPIO_PXDIRC(p) = (1 << o);		\
	REG_GPIO_PXFLGC(p) = (1 << o);		\
	REG_GPIO_PXIMC(p) = (1 << o);		\
} while (0)

#define __gpio_mask_irq(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMS(p) = (1 << o);		\
} while (0)

#define __gpio_unmask_irq(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMC(p) = (1 << o);		\
} while (0)

#define __gpio_ack_irq(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXFLGC(p) = (1 << o);		\
} while (0)

#define __gpio_get_irq()			\
({						\
	unsigned int p, i, tmp, v = 0;		\
	for (p = 3; p >= 0; p--) {		\
		tmp = REG_GPIO_PXFLG(p);	\
		for (i = 0; i < 32; i++)	\
			if (tmp & (1 << i))	\
				v = (32*p + i);	\
	}					\
	v;					\
})

#define __gpio_group_irq(n)			\
({						\
	register int tmp, i;			\
	tmp = REG_GPIO_PXFLG((n));		\
	for (i=31;i>=0;i--)			\
		if (tmp & (1 << i))		\
			break;			\
	i;					\
})

#define __gpio_enable_pull(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXPEC(p) = (1 << o);		\
} while (0)

#define __gpio_disable_pull(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXPES(p) = (1 << o);		\
} while (0)


/***************************************************************************
 * CPM
 ***************************************************************************/
#define __cpm_get_pllm() \
	((REG_CPM_CPPCR & CPM_CPPCR_PLLM_MASK) >> CPM_CPPCR_PLLM_BIT)
#define __cpm_get_plln() \
	((REG_CPM_CPPCR & CPM_CPPCR_PLLN_MASK) >> CPM_CPPCR_PLLN_BIT)
#define __cpm_get_pllod() \
	((REG_CPM_CPPCR & CPM_CPPCR_PLLOD_MASK) >> CPM_CPPCR_PLLOD_BIT)

#define __cpm_get_cdiv() \
	((REG_CPM_CPCCR & CPM_CPCCR_CDIV_MASK) >> CPM_CPCCR_CDIV_BIT)
#define __cpm_get_hdiv() \
	((REG_CPM_CPCCR & CPM_CPCCR_HDIV_MASK) >> CPM_CPCCR_HDIV_BIT)
#define __cpm_get_pdiv() \
	((REG_CPM_CPCCR & CPM_CPCCR_PDIV_MASK) >> CPM_CPCCR_PDIV_BIT)
#define __cpm_get_mdiv() \
	((REG_CPM_CPCCR & CPM_CPCCR_MDIV_MASK) >> CPM_CPCCR_MDIV_BIT)
#define __cpm_get_ldiv() \
	((REG_CPM_CPCCR & CPM_CPCCR_LDIV_MASK) >> CPM_CPCCR_LDIV_BIT)
#define __cpm_get_udiv() \
	((REG_CPM_CPCCR & CPM_CPCCR_UDIV_MASK) >> CPM_CPCCR_UDIV_BIT)
#define __cpm_get_i2sdiv() \
	((REG_CPM_I2SCDR & CPM_I2SCDR_I2SDIV_MASK) >> CPM_I2SCDR_I2SDIV_BIT)
#define __cpm_get_pixdiv() \
	((REG_CPM_LPCDR & CPM_LPCDR_PIXDIV_MASK) >> CPM_LPCDR_PIXDIV_BIT)
#define __cpm_get_mscdiv() \
	((REG_CPM_MSCCDR & CPM_MSCCDR_MSCDIV_MASK) >> CPM_MSCCDR_MSCDIV_BIT)
#define __cpm_get_uhcdiv() \
	((REG_CPM_UHCCDR & CPM_UHCCDR_UHCDIV_MASK) >> CPM_UHCCDR_UHCDIV_BIT)
#define __cpm_get_ssidiv() \
	((REG_CPM_SSICCDR & CPM_SSICDR_SSICDIV_MASK) >> CPM_SSICDR_SSIDIV_BIT)

#define __cpm_set_cdiv(v) \
	(REG_CPM_CPCCR = (REG_CPM_CPCCR & ~CPM_CPCCR_CDIV_MASK) | ((v) << (CPM_CPCCR_CDIV_BIT)))
#define __cpm_set_hdiv(v) \
	(REG_CPM_CPCCR = (REG_CPM_CPCCR & ~CPM_CPCCR_HDIV_MASK) | ((v) << (CPM_CPCCR_HDIV_BIT)))
#define __cpm_set_pdiv(v) \
	(REG_CPM_CPCCR = (REG_CPM_CPCCR & ~CPM_CPCCR_PDIV_MASK) | ((v) << (CPM_CPCCR_PDIV_BIT)))
#define __cpm_set_mdiv(v) \
	(REG_CPM_CPCCR = (REG_CPM_CPCCR & ~CPM_CPCCR_MDIV_MASK) | ((v) << (CPM_CPCCR_MDIV_BIT)))
#define __cpm_set_ldiv(v) \
	(REG_CPM_CPCCR = (REG_CPM_CPCCR & ~CPM_CPCCR_LDIV_MASK) | ((v) << (CPM_CPCCR_LDIV_BIT)))
#define __cpm_set_udiv(v) \
	(REG_CPM_CPCCR = (REG_CPM_CPCCR & ~CPM_CPCCR_UDIV_MASK) | ((v) << (CPM_CPCCR_UDIV_BIT)))
#define __cpm_set_i2sdiv(v) \
	(REG_CPM_I2SCDR = (REG_CPM_I2SCDR & ~CPM_I2SCDR_I2SDIV_MASK) | ((v) << (CPM_I2SCDR_I2SDIV_BIT)))
#define __cpm_set_pixdiv(v) \
	(REG_CPM_LPCDR = (REG_CPM_LPCDR & ~CPM_LPCDR_PIXDIV_MASK) | ((v) << (CPM_LPCDR_PIXDIV_BIT)))
#define __cpm_set_mscdiv(v) \
	(REG_CPM_MSCCDR = (REG_CPM_MSCCDR & ~CPM_MSCCDR_MSCDIV_MASK) | ((v) << (CPM_MSCCDR_MSCDIV_BIT)))
#define __cpm_set_uhcdiv(v) \
	(REG_CPM_UHCCDR = (REG_CPM_UHCCDR & ~CPM_UHCCDR_UHCDIV_MASK) | ((v) << (CPM_UHCCDR_UHCDIV_BIT)))
#define __cpm_ssiclk_select_exclk() \
	(REG_CPM_SSICDR &= ~CPM_SSICDR_SCS)
#define __cpm_ssiclk_select_pllout() \
	(REG_CPM_SSICDR |= CPM_SSICDR_SCS)
#define __cpm_set_ssidiv(v) \
	(REG_CPM_SSICDR = (REG_CPM_SSICDR & ~CPM_SSICDR_SSIDIV_MASK) | ((v) << (CPM_SSICDR_SSIDIV_BIT)))

#define __cpm_select_i2sclk_exclk()	(REG_CPM_CPCCR &= ~CPM_CPCCR_I2CS)
#define __cpm_select_i2sclk_pll()	(REG_CPM_CPCCR |= CPM_CPCCR_I2CS)
#define __cpm_enable_cko()		(REG_CPM_CPCCR |= CPM_CPCCR_CLKOEN)
#define __cpm_select_usbclk_exclk()	(REG_CPM_CPCCR &= ~CPM_CPCCR_UCS)
#define __cpm_select_usbclk_pll()	(REG_CPM_CPCCR |= CPM_CPCCR_UCS)
#define __cpm_enable_pll_change()	(REG_CPM_CPCCR |= CPM_CPCCR_CE)
#define __cpm_pllout_direct()		(REG_CPM_CPCCR |= CPM_CPCCR_PCS)
#define __cpm_pllout_div2()		(REG_CPM_CPCCR &= ~CPM_CPCCR_PCS)

#define __cpm_pll_is_on()		(REG_CPM_CPPCR & CPM_CPPCR_PLLS)
#define __cpm_pll_bypass()		(REG_CPM_CPPCR |= CPM_CPPCR_PLLBP)
#define __cpm_pll_enable()		(REG_CPM_CPPCR |= CPM_CPPCR_PLLEN)

#define __cpm_get_cclk_doze_duty() \
	((REG_CPM_LCR & CPM_LCR_DOZE_DUTY_MASK) >> CPM_LCR_DOZE_DUTY_BIT)
#define __cpm_set_cclk_doze_duty(v) \
	(REG_CPM_LCR = (REG_CPM_LCR & ~CPM_LCR_DOZE_DUTY_MASK) | ((v) << (CPM_LCR_DOZE_DUTY_BIT)))

#define __cpm_doze_mode()		(REG_CPM_LCR |= CPM_LCR_DOZE_ON)
#define __cpm_idle_mode() \
	(REG_CPM_LCR = (REG_CPM_LCR & ~CPM_LCR_LPM_MASK) | CPM_LCR_LPM_IDLE)
#define __cpm_sleep_mode() \
	(REG_CPM_LCR = (REG_CPM_LCR & ~CPM_LCR_LPM_MASK) | CPM_LCR_LPM_SLEEP)

#define __cpm_stop_all() 	(REG_CPM_CLKGR = 0x7fff)
#define __cpm_stop_uart1()	(REG_CPM_CLKGR |= CPM_CLKGR_UART1)
#define __cpm_stop_uhc()	(REG_CPM_CLKGR |= CPM_CLKGR_UHC)
#define __cpm_stop_ipu()	(REG_CPM_CLKGR |= CPM_CLKGR_IPU)
#define __cpm_stop_dmac()	(REG_CPM_CLKGR |= CPM_CLKGR_DMAC)
#define __cpm_stop_udc()	(REG_CPM_CLKGR |= CPM_CLKGR_UDC)
#define __cpm_stop_lcd()	(REG_CPM_CLKGR |= CPM_CLKGR_LCD)
#define __cpm_stop_cim()	(REG_CPM_CLKGR |= CPM_CLKGR_CIM)
#define __cpm_stop_sadc()	(REG_CPM_CLKGR |= CPM_CLKGR_SADC)
#define __cpm_stop_msc()	(REG_CPM_CLKGR |= CPM_CLKGR_MSC)
#define __cpm_stop_aic1()	(REG_CPM_CLKGR |= CPM_CLKGR_AIC1)
#define __cpm_stop_aic2()	(REG_CPM_CLKGR |= CPM_CLKGR_AIC2)
#define __cpm_stop_ssi()	(REG_CPM_CLKGR |= CPM_CLKGR_SSI)
#define __cpm_stop_i2c()	(REG_CPM_CLKGR |= CPM_CLKGR_I2C)
#define __cpm_stop_rtc()	(REG_CPM_CLKGR |= CPM_CLKGR_RTC)
#define __cpm_stop_tcu()	(REG_CPM_CLKGR |= CPM_CLKGR_TCU)
#define __cpm_stop_uart0()	(REG_CPM_CLKGR |= CPM_CLKGR_UART0)

#define __cpm_start_all() 	(REG_CPM_CLKGR = 0x0)
#define __cpm_start_uart1()	(REG_CPM_CLKGR &= ~CPM_CLKGR_UART1)
#define __cpm_start_uhc()	(REG_CPM_CLKGR &= ~CPM_CLKGR_UHC)
#define __cpm_start_ipu()	(REG_CPM_CLKGR &= ~CPM_CLKGR_IPU)
#define __cpm_start_dmac()	(REG_CPM_CLKGR &= ~CPM_CLKGR_DMAC)
#define __cpm_start_udc()	(REG_CPM_CLKGR &= ~CPM_CLKGR_UDC)
#define __cpm_start_lcd()	(REG_CPM_CLKGR &= ~CPM_CLKGR_LCD)
#define __cpm_start_cim()	(REG_CPM_CLKGR &= ~CPM_CLKGR_CIM)
#define __cpm_start_sadc()	(REG_CPM_CLKGR &= ~CPM_CLKGR_SADC)
#define __cpm_start_msc()	(REG_CPM_CLKGR &= ~CPM_CLKGR_MSC)
#define __cpm_start_aic1()	(REG_CPM_CLKGR &= ~CPM_CLKGR_AIC1)
#define __cpm_start_aic2()	(REG_CPM_CLKGR &= ~CPM_CLKGR_AIC2)
#define __cpm_start_ssi()	(REG_CPM_CLKGR &= ~CPM_CLKGR_SSI)
#define __cpm_start_i2c()	(REG_CPM_CLKGR &= ~CPM_CLKGR_I2C)
#define __cpm_start_rtc()	(REG_CPM_CLKGR &= ~CPM_CLKGR_RTC)
#define __cpm_start_tcu()	(REG_CPM_CLKGR &= ~CPM_CLKGR_TCU)
#define __cpm_start_uart0()	(REG_CPM_CLKGR &= ~CPM_CLKGR_UART0)

#define __cpm_get_o1st() \
	((REG_CPM_SCR & CPM_SCR_O1ST_MASK) >> CPM_SCR_O1ST_BIT)
#define __cpm_set_o1st(v) \
	(REG_CPM_SCR = (REG_CPM_SCR & ~CPM_SCR_O1ST_MASK) | ((v) << (CPM_SCR_O1ST_BIT)))
#define __cpm_suspend_usbphy()		(REG_CPM_SCR |= CPM_SCR_USBPHY_SUSPEND)
#define __cpm_enable_osc_in_sleep()	(REG_CPM_SCR |= CPM_SCR_OSC_ENABLE)


/***************************************************************************
 * TCU
 ***************************************************************************/
// where 'n' is the TCU channel
#define __tcu_select_extalclk(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~(TCU_TCSR_EXT_EN | TCU_TCSR_RTC_EN | TCU_TCSR_PCK_EN)) | TCU_TCSR_EXT_EN)
#define __tcu_select_rtcclk(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~(TCU_TCSR_EXT_EN | TCU_TCSR_RTC_EN | TCU_TCSR_PCK_EN)) | TCU_TCSR_RTC_EN)
#define __tcu_select_pclk(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~(TCU_TCSR_EXT_EN | TCU_TCSR_RTC_EN | TCU_TCSR_PCK_EN)) | TCU_TCSR_PCK_EN)

#define __tcu_select_clk_div1(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~TCU_TCSR_PRESCALE_MASK) | TCU_TCSR_PRESCALE1)
#define __tcu_select_clk_div4(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~TCU_TCSR_PRESCALE_MASK) | TCU_TCSR_PRESCALE4)
#define __tcu_select_clk_div16(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~TCU_TCSR_PRESCALE_MASK) | TCU_TCSR_PRESCALE16)
#define __tcu_select_clk_div64(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~TCU_TCSR_PRESCALE_MASK) | TCU_TCSR_PRESCALE64)
#define __tcu_select_clk_div256(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~TCU_TCSR_PRESCALE_MASK) | TCU_TCSR_PRESCALE256)
#define __tcu_select_clk_div1024(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~TCU_TCSR_PRESCALE_MASK) | TCU_TCSR_PRESCALE1024)

#define __tcu_enable_pwm_output(n)	( REG_TCU_TCSR((n)) |= TCU_TCSR_PWM_EN )
#define __tcu_disable_pwm_output(n)	( REG_TCU_TCSR((n)) &= ~TCU_TCSR_PWM_EN )

#define __tcu_init_pwm_output_high(n)	( REG_TCU_TCSR((n)) |= TCU_TCSR_PWM_INITL_HIGH )
#define __tcu_init_pwm_output_low(n)	( REG_TCU_TCSR((n)) &= ~TCU_TCSR_PWM_INITL_HIGH )

#define __tcu_set_pwm_output_shutdown_graceful(n)	( REG_TCU_TCSR((n)) &= ~TCU_TCSR_PWM_SD )
#define __tcu_set_pwm_output_shutdown_abrupt(n)		( REG_TCU_TCSR((n)) |= TCU_TCSR_PWM_SD )

#define __tcu_start_counter(n)		( REG_TCU_TESR |= (1 << (n)) )
#define __tcu_stop_counter(n)		( REG_TCU_TECR |= (1 << (n)) )

#define __tcu_half_match_flag(n)	( REG_TCU_TFR & (1 << ((n) + 16)) )
#define __tcu_full_match_flag(n)	( REG_TCU_TFR & (1 << (n)) )
#define __tcu_set_half_match_flag(n)	( REG_TCU_TFSR = (1 << ((n) + 16)) )
#define __tcu_set_full_match_flag(n)	( REG_TCU_TFSR = (1 << (n)) )
#define __tcu_clear_half_match_flag(n)	( REG_TCU_TFCR = (1 << ((n) + 16)) )
#define __tcu_clear_full_match_flag(n)	( REG_TCU_TFCR = (1 << (n)) )
#define __tcu_mask_half_match_irq(n)	( REG_TCU_TMSR = (1 << ((n) + 16)) )
#define __tcu_mask_full_match_irq(n)	( REG_TCU_TMSR = (1 << (n)) )
#define __tcu_unmask_half_match_irq(n)	( REG_TCU_TMCR = (1 << ((n) + 16)) )
#define __tcu_unmask_full_match_irq(n)	( REG_TCU_TMCR = (1 << (n)) )

#define __tcu_wdt_clock_stopped()	( REG_TCU_TSR & TCU_TSSR_WDTSC )
#define __tcu_timer_clock_stopped(n)	( REG_TCU_TSR & (1 << (n)) )

#define __tcu_start_wdt_clock()		( REG_TCU_TSCR = TCU_TSSR_WDTSC )
#define __tcu_start_timer_clock(n)	( REG_TCU_TSCR = (1 << (n)) )

#define __tcu_stop_wdt_clock()		( REG_TCU_TSSR = TCU_TSSR_WDTSC )
#define __tcu_stop_timer_clock(n)	( REG_TCU_TSSR = (1 << (n)) )

#define __tcu_get_count(n)		( REG_TCU_TCNT((n)) )
#define __tcu_set_count(n,v)		( REG_TCU_TCNT((n)) = (v) )
#define __tcu_set_full_data(n,v)	( REG_TCU_TDFR((n)) = (v) )
#define __tcu_set_half_data(n,v)	( REG_TCU_TDHR((n)) = (v) )


/***************************************************************************
 * WDT
 ***************************************************************************/
#define __wdt_start()			( REG_WDT_TCER |= WDT_TCER_TCEN )
#define __wdt_stop()			( REG_WDT_TCER &= ~WDT_TCER_TCEN )
#define __wdt_set_count(v)		( REG_WDT_TCNT = (v) )
#define __wdt_set_data(v)		( REG_WDT_TDR = (v) )

#define __wdt_select_extalclk() \
	(REG_WDT_TCSR = (REG_WDT_TCSR & ~(WDT_TCSR_EXT_EN | WDT_TCSR_RTC_EN | WDT_TCSR_PCK_EN)) | WDT_TCSR_EXT_EN)
#define __wdt_select_rtcclk() \
	(REG_WDT_TCSR = (REG_WDT_TCSR & ~(WDT_TCSR_EXT_EN | WDT_TCSR_RTC_EN | WDT_TCSR_PCK_EN)) | WDT_TCSR_RTC_EN)
#define __wdt_select_pclk() \
	(REG_WDT_TCSR = (REG_WDT_TCSR & ~(WDT_TCSR_EXT_EN | WDT_TCSR_RTC_EN | WDT_TCSR_PCK_EN)) | WDT_TCSR_PCK_EN)

#define __wdt_select_clk_div1() \
	(REG_WDT_TCSR = (REG_WDT_TCSR & ~WDT_TCSR_PRESCALE_MASK) | WDT_TCSR_PRESCALE1)
#define __wdt_select_clk_div4() \
	(REG_WDT_TCSR = (REG_WDT_TCSR & ~WDT_TCSR_PRESCALE_MASK) | WDT_TCSR_PRESCALE4)
#define __wdt_select_clk_div16() \
	(REG_WDT_TCSR = (REG_WDT_TCSR & ~WDT_TCSR_PRESCALE_MASK) | WDT_TCSR_PRESCALE16)
#define __wdt_select_clk_div64() \
	(REG_WDT_TCSR = (REG_WDT_TCSR & ~WDT_TCSR_PRESCALE_MASK) | WDT_TCSR_PRESCALE64)
#define __wdt_select_clk_div256() \
	(REG_WDT_TCSR = (REG_WDT_TCSR & ~WDT_TCSR_PRESCALE_MASK) | WDT_TCSR_PRESCALE256)
#define __wdt_select_clk_div1024() \
	(REG_WDT_TCSR = (REG_WDT_TCSR & ~WDT_TCSR_PRESCALE_MASK) | WDT_TCSR_PRESCALE1024)


/***************************************************************************
 * UART
 ***************************************************************************/

#define __uart_enable(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_FCR) |= UARTFCR_UUE | UARTFCR_FE )
#define __uart_disable(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_FCR) = ~UARTFCR_UUE )

#define __uart_enable_transmit_irq(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_IER) |= UARTIER_TIE )
#define __uart_disable_transmit_irq(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_IER) &= ~UARTIER_TIE )

#define __uart_enable_receive_irq(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_IER) |= UARTIER_RIE | UARTIER_RLIE | UARTIER_RTIE )
#define __uart_disable_receive_irq(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_IER) &= ~(UARTIER_RIE | UARTIER_RLIE | UARTIER_RTIE) )

#define __uart_enable_loopback(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_MCR) |= UARTMCR_LOOP )
#define __uart_disable_loopback(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_MCR) &= ~UARTMCR_LOOP )

#define __uart_set_8n1(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_LCR) = UARTLCR_WLEN_8 )

#define __uart_set_baud(n, devclk, baud)						\
  do {											\
	REG8(UART_BASE + UART_OFF*(n) + OFF_LCR) |= UARTLCR_DLAB;			\
	REG8(UART_BASE + UART_OFF*(n) + OFF_DLLR) = (devclk / 16 / baud) & 0xff;	\
	REG8(UART_BASE + UART_OFF*(n) + OFF_DLHR) = ((devclk / 16 / baud) >> 8) & 0xff;	\
	REG8(UART_BASE + UART_OFF*(n) + OFF_LCR) &= ~UARTLCR_DLAB;			\
  } while (0)

#define __uart_parity_error(n) \
  ( (REG8(UART_BASE + UART_OFF*(n) + OFF_LSR) & UARTLSR_PER) != 0 )

#define __uart_clear_errors(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_LSR) &= ~(UARTLSR_ORER | UARTLSR_BRK | UARTLSR_FER | UARTLSR_PER | UARTLSR_RFER) )

#define __uart_transmit_fifo_empty(n) \
  ( (REG8(UART_BASE + UART_OFF*(n) + OFF_LSR) & UARTLSR_TDRQ) != 0 )

#define __uart_transmit_end(n) \
  ( (REG8(UART_BASE + UART_OFF*(n) + OFF_LSR) & UARTLSR_TEMT) != 0 )

#define __uart_transmit_char(n, ch) \
  REG8(UART_BASE + UART_OFF*(n) + OFF_TDR) = (ch)

#define __uart_receive_fifo_full(n) \
  ( (REG8(UART_BASE + UART_OFF*(n) + OFF_LSR) & UARTLSR_DR) != 0 )

#define __uart_receive_ready(n) \
  ( (REG8(UART_BASE + UART_OFF*(n) + OFF_LSR) & UARTLSR_DR) != 0 )

#define __uart_receive_char(n) \
  REG8(UART_BASE + UART_OFF*(n) + OFF_RDR)

#define __uart_disable_irda() \
  ( REG8(IRDA_BASE + OFF_SIRCR) &= ~(SIRCR_TSIRE | SIRCR_RSIRE) )
#define __uart_enable_irda() \
  /* Tx high pulse as 0, Rx low pulse as 0 */ \
  ( REG8(IRDA_BASE + OFF_SIRCR) = SIRCR_TSIRE | SIRCR_RSIRE | SIRCR_RXPL | SIRCR_TPWS )


/***************************************************************************
 * DMAC
 ***************************************************************************/

/* n is the DMA channel (0 - 5) */

#define __dmac_enable_module() \
  ( REG_DMAC_DMACR |= DMAC_DMACR_DMAE | DMAC_DMACR_PR_RR )
#define __dmac_disable_module() \
  ( REG_DMAC_DMACR &= ~DMAC_DMACR_DMAE )

/* p=0,1,2,3 */
#define __dmac_set_priority(p) 				\
do {							\
	REG_DMAC_DMACR &= ~DMAC_DMACR_PR_MASK;		\
	REG_DMAC_DMACR |= ((p) << DMAC_DMACR_PR_BIT);	\
} while (0)

#define __dmac_test_halt_error() ( REG_DMAC_DMACR & DMAC_DMACR_HLT )
#define __dmac_test_addr_error() ( REG_DMAC_DMACR & DMAC_DMACR_AR )

#define __dmac_enable_descriptor(n) \
  ( REG_DMAC_DCCSR((n)) &= ~DMAC_DCCSR_NDES )
#define __dmac_disable_descriptor(n) \
  ( REG_DMAC_DCCSR((n)) |= DMAC_DCCSR_NDES )

#define __dmac_enable_channel(n) \
  ( REG_DMAC_DCCSR((n)) |= DMAC_DCCSR_EN )
#define __dmac_disable_channel(n) \
  ( REG_DMAC_DCCSR((n)) &= ~DMAC_DCCSR_EN )
#define __dmac_channel_enabled(n) \
  ( REG_DMAC_DCCSR((n)) & DMAC_DCCSR_EN )

#define __dmac_channel_enable_irq(n) \
  ( REG_DMAC_DCMD((n)) |= DMAC_DCMD_TIE )
#define __dmac_channel_disable_irq(n) \
  ( REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_TIE )

#define __dmac_channel_transmit_halt_detected(n) \
  (  REG_DMAC_DCCSR((n)) & DMAC_DCCSR_HLT )
#define __dmac_channel_transmit_end_detected(n) \
  (  REG_DMAC_DCCSR((n)) & DMAC_DCCSR_TT )
#define __dmac_channel_address_error_detected(n) \
  (  REG_DMAC_DCCSR((n)) & DMAC_DCCSR_AR )
#define __dmac_channel_count_terminated_detected(n) \
  (  REG_DMAC_DCCSR((n)) & DMAC_DCCSR_CT )
#define __dmac_channel_descriptor_invalid_detected(n) \
  (  REG_DMAC_DCCSR((n)) & DMAC_DCCSR_INV )

#define __dmac_channel_clear_transmit_halt(n) \
  (  REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_HLT )
#define __dmac_channel_clear_transmit_end(n) \
  (  REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_TT )
#define __dmac_channel_clear_address_error(n) \
  (  REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_AR )
#define __dmac_channel_clear_count_terminated(n) \
  (  REG_DMAC_DCCSR((n)) &= ~DMAC_DCCSR_CT )
#define __dmac_channel_clear_descriptor_invalid(n) \
  (  REG_DMAC_DCCSR((n)) &= ~DMAC_DCCSR_INV )

#define __dmac_channel_set_single_mode(n) \
  (  REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_TM )
#define __dmac_channel_set_block_mode(n) \
  (  REG_DMAC_DCMD((n)) |= DMAC_DCMD_TM )

#define __dmac_channel_set_transfer_unit_32bit(n)	\
do {							\
	REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_DS_MASK;	\
	REG_DMAC_DCMD((n)) |= DMAC_DCMD_DS_32BIT;	\
} while (0)

#define __dmac_channel_set_transfer_unit_16bit(n)	\
do {							\
	REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_DS_MASK;	\
	REG_DMAC_DCMD((n)) |= DMAC_DCMD_DS_16BIT;	\
} while (0)

#define __dmac_channel_set_transfer_unit_8bit(n)	\
do {							\
	REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_DS_MASK;	\
	REG_DMAC_DCMD((n)) |= DMAC_DCMD_DS_8BIT;	\
} while (0)

#define __dmac_channel_set_transfer_unit_16byte(n)	\
do {							\
	REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_DS_MASK;	\
	REG_DMAC_DCMD((n)) |= DMAC_DCMD_DS_16BYTE;	\
} while (0)

#define __dmac_channel_set_transfer_unit_32byte(n)	\
do {							\
	REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_DS_MASK;	\
	REG_DMAC_DCMD((n)) |= DMAC_DCMD_DS_32BYTE;	\
} while (0)

/* w=8,16,32 */
#define __dmac_channel_set_dest_port_width(n,w)		\
do {							\
	REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_DWDH_MASK;	\
	REG_DMAC_DCMD((n)) |= DMAC_DCMD_DWDH_##w;	\
} while (0)

/* w=8,16,32 */
#define __dmac_channel_set_src_port_width(n,w)		\
do {							\
	REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_SWDH_MASK;	\
	REG_DMAC_DCMD((n)) |= DMAC_DCMD_SWDH_##w;	\
} while (0)

/* v=0-15 */
#define __dmac_channel_set_rdil(n,v)				\
do {								\
	REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_RDIL_MASK;		\
	REG_DMAC_DCMD((n) |= ((v) << DMAC_DCMD_RDIL_BIT);	\
} while (0)

#define __dmac_channel_dest_addr_fixed(n) \
  (  REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_DAI )
#define __dmac_channel_dest_addr_increment(n) \
  (  REG_DMAC_DCMD((n)) |= DMAC_DCMD_DAI )

#define __dmac_channel_src_addr_fixed(n) \
  (  REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_SAI )
#define __dmac_channel_src_addr_increment(n) \
  (  REG_DMAC_DCMD((n)) |= DMAC_DCMD_SAI )

#define __dmac_channel_set_doorbell(n) \
  (  REG_DMAC_DMADBSR = (1 << (n)) )

#define __dmac_channel_irq_detected(n)  ( REG_DMAC_DMAIPR & (1 << (n)) )
#define __dmac_channel_ack_irq(n)       ( REG_DMAC_DMAIPR &= ~(1 << (n)) )

static __inline__ int __dmac_get_irq(void)
{
	int i;
	for (i = 0; i < MAX_DMA_NUM; i++)
		if (__dmac_channel_irq_detected(i))
			return i;
	return -1;
}


/***************************************************************************
 * AIC (AC'97 & I2S Controller)
 ***************************************************************************/

#define __aic_enable()		( REG_AIC_FR |= AIC_FR_ENB )
#define __aic_disable()		( REG_AIC_FR &= ~AIC_FR_ENB )

#define __aic_select_ac97()	( REG_AIC_FR &= ~AIC_FR_AUSEL )
#define __aic_select_i2s()	( REG_AIC_FR |= AIC_FR_AUSEL )

#define __aic_play_zero()	( REG_AIC_FR &= ~AIC_FR_LSMP )
#define __aic_play_lastsample()	( REG_AIC_FR |= AIC_FR_LSMP )

#define __i2s_as_master()	( REG_AIC_FR |= AIC_FR_BCKD | AIC_FR_SYNCD )
#define __i2s_as_slave()	( REG_AIC_FR &= ~(AIC_FR_BCKD | AIC_FR_SYNCD) )
#define __aic_reset_status()          ( REG_AIC_FR & AIC_FR_RST )

#define __aic_reset()                                   \
do {                                                    \
        REG_AIC_FR |= AIC_FR_RST;                       \
} while(0)


#define __aic_set_transmit_trigger(n) 			\
do {							\
	REG_AIC_FR &= ~AIC_FR_TFTH_MASK;		\
	REG_AIC_FR |= ((n) << AIC_FR_TFTH_BIT);		\
} while(0)

#define __aic_set_receive_trigger(n) 			\
do {							\
	REG_AIC_FR &= ~AIC_FR_RFTH_MASK;		\
	REG_AIC_FR |= ((n) << AIC_FR_RFTH_BIT);		\
} while(0)

#define __aic_enable_record()	( REG_AIC_CR |= AIC_CR_EREC )
#define __aic_disable_record()	( REG_AIC_CR &= ~AIC_CR_EREC )
#define __aic_enable_replay()	( REG_AIC_CR |= AIC_CR_ERPL )
#define __aic_disable_replay()	( REG_AIC_CR &= ~AIC_CR_ERPL )
#define __aic_enable_loopback()	( REG_AIC_CR |= AIC_CR_ENLBF )
#define __aic_disable_loopback() ( REG_AIC_CR &= ~AIC_CR_ENLBF )

#define __aic_flush_fifo()	( REG_AIC_CR |= AIC_CR_FLUSH )
#define __aic_unflush_fifo()	( REG_AIC_CR &= ~AIC_CR_FLUSH )

#define __aic_enable_transmit_intr() \
  ( REG_AIC_CR |= (AIC_CR_ETFS | AIC_CR_ETUR) )
#define __aic_disable_transmit_intr() \
  ( REG_AIC_CR &= ~(AIC_CR_ETFS | AIC_CR_ETUR) )
#define __aic_enable_receive_intr() \
  ( REG_AIC_CR |= (AIC_CR_ERFS | AIC_CR_EROR) )
#define __aic_disable_receive_intr() \
  ( REG_AIC_CR &= ~(AIC_CR_ERFS | AIC_CR_EROR) )

#define __aic_enable_transmit_dma()  ( REG_AIC_CR |= AIC_CR_TDMS )
#define __aic_disable_transmit_dma() ( REG_AIC_CR &= ~AIC_CR_TDMS )
#define __aic_enable_receive_dma()   ( REG_AIC_CR |= AIC_CR_RDMS )
#define __aic_disable_receive_dma()  ( REG_AIC_CR &= ~AIC_CR_RDMS )

#define __aic_enable_mono2stereo()   ( REG_AIC_CR |= AIC_CR_M2S )
#define __aic_disable_mono2stereo()  ( REG_AIC_CR &= ~AIC_CR_M2S )
#define __aic_enable_byteswap()      ( REG_AIC_CR |= AIC_CR_ENDSW )
#define __aic_disable_byteswap()     ( REG_AIC_CR &= ~AIC_CR_ENDSW )
#define __aic_enable_unsignadj()     ( REG_AIC_CR |= AIC_CR_AVSTSU )
#define __aic_disable_unsignadj()    ( REG_AIC_CR &= ~AIC_CR_AVSTSU )

#define AC97_PCM_XS_L_FRONT   	AIC_ACCR1_XS_SLOT3
#define AC97_PCM_XS_R_FRONT   	AIC_ACCR1_XS_SLOT4
#define AC97_PCM_XS_CENTER    	AIC_ACCR1_XS_SLOT6
#define AC97_PCM_XS_L_SURR    	AIC_ACCR1_XS_SLOT7
#define AC97_PCM_XS_R_SURR    	AIC_ACCR1_XS_SLOT8
#define AC97_PCM_XS_LFE       	AIC_ACCR1_XS_SLOT9

#define AC97_PCM_RS_L_FRONT   	AIC_ACCR1_RS_SLOT3
#define AC97_PCM_RS_R_FRONT   	AIC_ACCR1_RS_SLOT4
#define AC97_PCM_RS_CENTER    	AIC_ACCR1_RS_SLOT6
#define AC97_PCM_RS_L_SURR    	AIC_ACCR1_RS_SLOT7
#define AC97_PCM_RS_R_SURR    	AIC_ACCR1_RS_SLOT8
#define AC97_PCM_RS_LFE       	AIC_ACCR1_RS_SLOT9

#define __ac97_set_xs_none()	( REG_AIC_ACCR1 &= ~AIC_ACCR1_XS_MASK )
#define __ac97_set_xs_mono() 						\
do {									\
	REG_AIC_ACCR1 &= ~AIC_ACCR1_XS_MASK;				\
	REG_AIC_ACCR1 |= AC97_PCM_XS_R_FRONT;				\
} while(0)
#define __ac97_set_xs_stereo() 						\
do {									\
	REG_AIC_ACCR1 &= ~AIC_ACCR1_XS_MASK;				\
	REG_AIC_ACCR1 |= AC97_PCM_XS_L_FRONT | AC97_PCM_XS_R_FRONT;	\
} while(0)

/* In fact, only stereo is support now. */ 
#define __ac97_set_rs_none()	( REG_AIC_ACCR1 &= ~AIC_ACCR1_RS_MASK )
#define __ac97_set_rs_mono() 						\
do {									\
	REG_AIC_ACCR1 &= ~AIC_ACCR1_RS_MASK;				\
	REG_AIC_ACCR1 |= AC97_PCM_RS_R_FRONT;				\
} while(0)
#define __ac97_set_rs_stereo() 						\
do {									\
	REG_AIC_ACCR1 &= ~AIC_ACCR1_RS_MASK;				\
	REG_AIC_ACCR1 |= AC97_PCM_RS_L_FRONT | AC97_PCM_RS_R_FRONT;	\
} while(0)

#define __ac97_warm_reset_codec()		\
 do {						\
	REG_AIC_ACCR2 |= AIC_ACCR2_SA;		\
	REG_AIC_ACCR2 |= AIC_ACCR2_SS;		\
	udelay(2);				\
	REG_AIC_ACCR2 &= ~AIC_ACCR2_SS;		\
	REG_AIC_ACCR2 &= ~AIC_ACCR2_SA;		\
 } while (0)

#define __ac97_cold_reset_codec()		\
 do {						\
	REG_AIC_ACCR2 |=  AIC_ACCR2_SR;		\
	udelay(2);				\
	REG_AIC_ACCR2 &= ~AIC_ACCR2_SR;		\
 } while (0)

/* n=8,16,18,20 */
#define __ac97_set_iass(n) \
 ( REG_AIC_ACCR2 = (REG_AIC_ACCR2 & ~AIC_ACCR2_IASS_MASK) | AIC_ACCR2_IASS_##n##BIT )
#define __ac97_set_oass(n) \
 ( REG_AIC_ACCR2 = (REG_AIC_ACCR2 & ~AIC_ACCR2_OASS_MASK) | AIC_ACCR2_OASS_##n##BIT )

#define __i2s_select_i2s()            ( REG_AIC_I2SCR &= ~AIC_I2SCR_AMSL )
#define __i2s_select_msbjustified()   ( REG_AIC_I2SCR |= AIC_I2SCR_AMSL )

/* n=8,16,18,20,24 */
/*#define __i2s_set_sample_size(n) \
 ( REG_AIC_I2SCR |= (REG_AIC_I2SCR & ~AIC_I2SCR_WL_MASK) | AIC_I2SCR_WL_##n##BIT )*/

#define __i2s_set_oss_sample_size(n) \
 ( REG_AIC_CR = (REG_AIC_CR & ~AIC_CR_OSS_MASK) | AIC_CR_OSS_##n##BIT )
#define __i2s_set_iss_sample_size(n) \
 ( REG_AIC_CR = (REG_AIC_CR & ~AIC_CR_ISS_MASK) | AIC_CR_ISS_##n##BIT )

#define __i2s_stop_bitclk()   ( REG_AIC_I2SCR |= AIC_I2SCR_STPBK )
#define __i2s_start_bitclk()  ( REG_AIC_I2SCR &= ~AIC_I2SCR_STPBK )

#define __aic_transmit_request()  ( REG_AIC_SR & AIC_SR_TFS )
#define __aic_receive_request()   ( REG_AIC_SR & AIC_SR_RFS )
#define __aic_transmit_underrun() ( REG_AIC_SR & AIC_SR_TUR )
#define __aic_receive_overrun()   ( REG_AIC_SR & AIC_SR_ROR )

#define __aic_clear_errors()      ( REG_AIC_SR &= ~(AIC_SR_TUR | AIC_SR_ROR) )

#define __aic_get_transmit_resident() \
  ( (REG_AIC_SR & AIC_SR_TFL_MASK) >> AIC_SR_TFL_BIT )
#define __aic_get_receive_count() \
  ( (REG_AIC_SR & AIC_SR_RFL_MASK) >> AIC_SR_RFL_BIT )

#define __ac97_command_transmitted()     ( REG_AIC_ACSR & AIC_ACSR_CADT )
#define __ac97_status_received()         ( REG_AIC_ACSR & AIC_ACSR_SADR )
#define __ac97_status_receive_timeout()  ( REG_AIC_ACSR & AIC_ACSR_RSTO )
#define __ac97_codec_is_low_power_mode() ( REG_AIC_ACSR & AIC_ACSR_CLPM )
#define __ac97_codec_is_ready()          ( REG_AIC_ACSR & AIC_ACSR_CRDY )
#define __ac97_slot_error_detected()     ( REG_AIC_ACSR & AIC_ACSR_SLTERR )
#define __ac97_clear_slot_error()        ( REG_AIC_ACSR &= ~AIC_ACSR_SLTERR )

#define __i2s_is_busy()         ( REG_AIC_I2SSR & AIC_I2SSR_BSY )

#define CODEC_READ_CMD	        (1 << 19)
#define CODEC_WRITE_CMD	        (0 << 19)
#define CODEC_REG_INDEX_BIT     12
#define CODEC_REG_INDEX_MASK	(0x7f << CODEC_REG_INDEX_BIT)	/* 18:12 */
#define CODEC_REG_DATA_BIT      4
#define CODEC_REG_DATA_MASK	(0x0ffff << 4)	/* 19:4 */

#define __ac97_out_rcmd_addr(reg) 					\
do { 									\
    REG_AIC_ACCAR = CODEC_READ_CMD | ((reg) << CODEC_REG_INDEX_BIT); 	\
} while (0)

#define __ac97_out_wcmd_addr(reg) 					\
do { 									\
    REG_AIC_ACCAR = CODEC_WRITE_CMD | ((reg) << CODEC_REG_INDEX_BIT); 	\
} while (0)

#define __ac97_out_data(value) 						\
do { 									\
    REG_AIC_ACCDR = ((value) << CODEC_REG_DATA_BIT); 			\
} while (0)

#define __ac97_in_data() \
 ( (REG_AIC_ACSDR & CODEC_REG_DATA_MASK) >> CODEC_REG_DATA_BIT )

#define __ac97_in_status_addr() \
 ( (REG_AIC_ACSAR & CODEC_REG_INDEX_MASK) >> CODEC_REG_INDEX_BIT )

#define __i2s_set_sample_rate(i2sclk, sync) \
  ( REG_AIC_I2SDIV = ((i2sclk) / (4*64)) / (sync) )

#define __aic_write_tfifo(v)  ( REG_AIC_DR = (v) )
#define __aic_read_rfifo()    ( REG_AIC_DR )

#define __aic_internal_codec()  ( REG_AIC_FR |= AIC_FR_ICDC ) 
#define __aic_external_codec()  ( REG_AIC_FR &= ~AIC_FR_ICDC )

//
// Define next ops for AC97 compatible
//

#define AC97_ACSR	AIC_ACSR

#define __ac97_enable()		__aic_enable(); __aic_select_ac97()
#define __ac97_disable()	__aic_disable()
#define __ac97_reset()		__aic_reset()

#define __ac97_set_transmit_trigger(n)	__aic_set_transmit_trigger(n)
#define __ac97_set_receive_trigger(n)	__aic_set_receive_trigger(n)

#define __ac97_enable_record()		__aic_enable_record()
#define __ac97_disable_record()		__aic_disable_record()
#define __ac97_enable_replay()		__aic_enable_replay()
#define __ac97_disable_replay()		__aic_disable_replay()
#define __ac97_enable_loopback()	__aic_enable_loopback()
#define __ac97_disable_loopback()	__aic_disable_loopback()

#define __ac97_enable_transmit_dma()	__aic_enable_transmit_dma()
#define __ac97_disable_transmit_dma()	__aic_disable_transmit_dma()
#define __ac97_enable_receive_dma()	__aic_enable_receive_dma()
#define __ac97_disable_receive_dma()	__aic_disable_receive_dma()

#define __ac97_transmit_request()	__aic_transmit_request()
#define __ac97_receive_request()	__aic_receive_request()
#define __ac97_transmit_underrun()	__aic_transmit_underrun()
#define __ac97_receive_overrun()	__aic_receive_overrun()

#define __ac97_clear_errors()		__aic_clear_errors()

#define __ac97_get_transmit_resident()	__aic_get_transmit_resident()
#define __ac97_get_receive_count()	__aic_get_receive_count()

#define __ac97_enable_transmit_intr()	__aic_enable_transmit_intr()
#define __ac97_disable_transmit_intr()	__aic_disable_transmit_intr()
#define __ac97_enable_receive_intr()	__aic_enable_receive_intr()
#define __ac97_disable_receive_intr()	__aic_disable_receive_intr()

#define __ac97_write_tfifo(v)		__aic_write_tfifo(v)
#define __ac97_read_rfifo()		__aic_read_rfifo()

//
// Define next ops for I2S compatible
//

#define I2S_ACSR	AIC_I2SSR

#define __i2s_enable()		 __aic_enable(); __aic_select_i2s()
#define __i2s_disable()		__aic_disable()
#define __i2s_reset()		__aic_reset()

#define __i2s_set_transmit_trigger(n)	__aic_set_transmit_trigger(n)
#define __i2s_set_receive_trigger(n)	__aic_set_receive_trigger(n)

#define __i2s_enable_record()		__aic_enable_record()
#define __i2s_disable_record()		__aic_disable_record()
#define __i2s_enable_replay()		__aic_enable_replay()
#define __i2s_disable_replay()		__aic_disable_replay()
#define __i2s_enable_loopback()		__aic_enable_loopback()
#define __i2s_disable_loopback()	__aic_disable_loopback()

#define __i2s_enable_transmit_dma()	__aic_enable_transmit_dma()
#define __i2s_disable_transmit_dma()	__aic_disable_transmit_dma()
#define __i2s_enable_receive_dma()	__aic_enable_receive_dma()
#define __i2s_disable_receive_dma()	__aic_disable_receive_dma()

#define __i2s_transmit_request()	__aic_transmit_request()
#define __i2s_receive_request()		__aic_receive_request()
#define __i2s_transmit_underrun()	__aic_transmit_underrun()
#define __i2s_receive_overrun()		__aic_receive_overrun()

#define __i2s_clear_errors()		__aic_clear_errors()

#define __i2s_get_transmit_resident()	__aic_get_transmit_resident()
#define __i2s_get_receive_count()	__aic_get_receive_count()

#define __i2s_enable_transmit_intr()	__aic_enable_transmit_intr()
#define __i2s_disable_transmit_intr()	__aic_disable_transmit_intr()
#define __i2s_enable_receive_intr()	__aic_enable_receive_intr()
#define __i2s_disable_receive_intr()	__aic_disable_receive_intr()

#define __i2s_write_tfifo(v)		__aic_write_tfifo(v)
#define __i2s_read_rfifo()		__aic_read_rfifo()

#define __i2s_reset_codec()			\
 do {						\
 } while (0)


/***************************************************************************
 * ICDC
 ***************************************************************************/
#define __i2s_internal_codec()         __aic_internal_codec()
#define __i2s_external_codec()         __aic_external_codec()

/***************************************************************************
 * INTC
 ***************************************************************************/
#define __intc_unmask_irq(n)	( REG_INTC_IMCR = (1 << (n)) )
#define __intc_mask_irq(n)	( REG_INTC_IMSR = (1 << (n)) )
#define __intc_ack_irq(n)	( REG_INTC_IPR = (1 << (n)) )


/***************************************************************************
 * I2C
 ***************************************************************************/

#define __i2c_enable()		( REG_I2C_CR |= I2C_CR_I2CE )
#define __i2c_disable()		( REG_I2C_CR &= ~I2C_CR_I2CE )

#define __i2c_send_start()	( REG_I2C_CR |= I2C_CR_STA )
#define __i2c_send_stop()	( REG_I2C_CR |= I2C_CR_STO )
#define __i2c_send_ack()	( REG_I2C_CR &= ~I2C_CR_AC )
#define __i2c_send_nack()	( REG_I2C_CR |= I2C_CR_AC )

#define __i2c_set_drf()		( REG_I2C_SR |= I2C_SR_DRF )
#define __i2c_clear_drf()	( REG_I2C_SR &= ~I2C_SR_DRF )
#define __i2c_check_drf()	( REG_I2C_SR & I2C_SR_DRF )

#define __i2c_received_ack()	( !(REG_I2C_SR & I2C_SR_ACKF) )
#define __i2c_is_busy()		( REG_I2C_SR & I2C_SR_BUSY )
#define __i2c_transmit_ended()	( REG_I2C_SR & I2C_SR_TEND )

#define __i2c_set_clk(dev_clk, i2c_clk) \
  ( REG_I2C_GR = (dev_clk) / (16*(i2c_clk)) - 1 )

#define __i2c_read()		( REG_I2C_DR )
#define __i2c_write(val)	( REG_I2C_DR = (val) )


/***************************************************************************
 * MSC
 ***************************************************************************/

#define __msc_start_op() \
  ( REG_MSC_STRPCL = MSC_STRPCL_START_OP | MSC_STRPCL_CLOCK_CONTROL_START )

#define __msc_set_resto(to) 	( REG_MSC_RESTO = to )
#define __msc_set_rdto(to) 	( REG_MSC_RDTO = to )
#define __msc_set_cmd(cmd) 	( REG_MSC_CMD = cmd )
#define __msc_set_arg(arg) 	( REG_MSC_ARG = arg )
#define __msc_set_nob(nob) 	( REG_MSC_NOB = nob )
#define __msc_get_nob() 	( REG_MSC_NOB )
#define __msc_set_blklen(len) 	( REG_MSC_BLKLEN = len )
#define __msc_set_cmdat(cmdat) 	( REG_MSC_CMDAT = cmdat )
#define __msc_set_cmdat_ioabort() 	( REG_MSC_CMDAT |= MSC_CMDAT_IO_ABORT )
#define __msc_clear_cmdat_ioabort() 	( REG_MSC_CMDAT &= ~MSC_CMDAT_IO_ABORT )

#define __msc_set_cmdat_bus_width1() 			\
do { 							\
	REG_MSC_CMDAT &= ~MSC_CMDAT_BUS_WIDTH_MASK; 	\
	REG_MSC_CMDAT |= MSC_CMDAT_BUS_WIDTH_1BIT; 	\
} while(0)

#define __msc_set_cmdat_bus_width4() 			\
do { 							\
	REG_MSC_CMDAT &= ~MSC_CMDAT_BUS_WIDTH_MASK; 	\
	REG_MSC_CMDAT |= MSC_CMDAT_BUS_WIDTH_4BIT; 	\
} while(0)

#define __msc_set_cmdat_dma_en() ( REG_MSC_CMDAT |= MSC_CMDAT_DMA_EN )
#define __msc_set_cmdat_init() 	( REG_MSC_CMDAT |= MSC_CMDAT_INIT )
#define __msc_set_cmdat_busy() 	( REG_MSC_CMDAT |= MSC_CMDAT_BUSY )
#define __msc_set_cmdat_stream() ( REG_MSC_CMDAT |= MSC_CMDAT_STREAM_BLOCK )
#define __msc_set_cmdat_block() ( REG_MSC_CMDAT &= ~MSC_CMDAT_STREAM_BLOCK )
#define __msc_set_cmdat_read() 	( REG_MSC_CMDAT &= ~MSC_CMDAT_WRITE_READ )
#define __msc_set_cmdat_write() ( REG_MSC_CMDAT |= MSC_CMDAT_WRITE_READ )
#define __msc_set_cmdat_data_en() ( REG_MSC_CMDAT |= MSC_CMDAT_DATA_EN )

/* r is MSC_CMDAT_RESPONSE_FORMAT_Rx or MSC_CMDAT_RESPONSE_FORMAT_NONE */
#define __msc_set_cmdat_res_format(r) 				\
do { 								\
	REG_MSC_CMDAT &= ~MSC_CMDAT_RESPONSE_FORMAT_MASK; 	\
	REG_MSC_CMDAT |= (r); 					\
} while(0)

#define __msc_clear_cmdat() \
  REG_MSC_CMDAT &= ~( MSC_CMDAT_IO_ABORT | MSC_CMDAT_DMA_EN | MSC_CMDAT_INIT| \
  MSC_CMDAT_BUSY | MSC_CMDAT_STREAM_BLOCK | MSC_CMDAT_WRITE_READ | \
  MSC_CMDAT_DATA_EN | MSC_CMDAT_RESPONSE_FORMAT_MASK )

#define __msc_get_imask() 		( REG_MSC_IMASK )
#define __msc_mask_all_intrs() 		( REG_MSC_IMASK = 0xff )
#define __msc_unmask_all_intrs() 	( REG_MSC_IMASK = 0x00 )
#define __msc_mask_rd() 		( REG_MSC_IMASK |= MSC_IMASK_RXFIFO_RD_REQ )
#define __msc_unmask_rd() 		( REG_MSC_IMASK &= ~MSC_IMASK_RXFIFO_RD_REQ )
#define __msc_mask_wr() 		( REG_MSC_IMASK |= MSC_IMASK_TXFIFO_WR_REQ )
#define __msc_unmask_wr() 		( REG_MSC_IMASK &= ~MSC_IMASK_TXFIFO_WR_REQ )
#define __msc_mask_endcmdres() 		( REG_MSC_IMASK |= MSC_IMASK_END_CMD_RES )
#define __msc_unmask_endcmdres() 	( REG_MSC_IMASK &= ~MSC_IMASK_END_CMD_RES )
#define __msc_mask_datatrandone() 	( REG_MSC_IMASK |= MSC_IMASK_DATA_TRAN_DONE )
#define __msc_unmask_datatrandone() 	( REG_MSC_IMASK &= ~MSC_IMASK_DATA_TRAN_DONE )
#define __msc_mask_prgdone() 		( REG_MSC_IMASK |= MSC_IMASK_PRG_DONE )
#define __msc_unmask_prgdone() 		( REG_MSC_IMASK &= ~MSC_IMASK_PRG_DONE )

/* n=0,1,2,3,4,5,6,7 */
#define __msc_set_clkrt(n) 	\
do { 				\
	REG_MSC_CLKRT = n;	\
} while(0)

#define __msc_get_ireg() 		( REG_MSC_IREG )
#define __msc_ireg_rd() 		( REG_MSC_IREG & MSC_IREG_RXFIFO_RD_REQ )
#define __msc_ireg_wr() 		( REG_MSC_IREG & MSC_IREG_TXFIFO_WR_REQ )
#define __msc_ireg_end_cmd_res() 	( REG_MSC_IREG & MSC_IREG_END_CMD_RES )
#define __msc_ireg_data_tran_done() 	( REG_MSC_IREG & MSC_IREG_DATA_TRAN_DONE )
#define __msc_ireg_prg_done() 		( REG_MSC_IREG & MSC_IREG_PRG_DONE )
#define __msc_ireg_clear_end_cmd_res() 	( REG_MSC_IREG = MSC_IREG_END_CMD_RES )
#define __msc_ireg_clear_data_tran_done() ( REG_MSC_IREG = MSC_IREG_DATA_TRAN_DONE )
#define __msc_ireg_clear_prg_done() 	( REG_MSC_IREG = MSC_IREG_PRG_DONE )

#define __msc_get_stat() 		( REG_MSC_STAT )
#define __msc_stat_not_end_cmd_res() 	( (REG_MSC_STAT & MSC_STAT_END_CMD_RES) == 0)
#define __msc_stat_crc_err() \
  ( REG_MSC_STAT & (MSC_STAT_CRC_RES_ERR | MSC_STAT_CRC_READ_ERROR | MSC_STAT_CRC_WRITE_ERROR_YES) )
#define __msc_stat_res_crc_err() 	( REG_MSC_STAT & MSC_STAT_CRC_RES_ERR )
#define __msc_stat_rd_crc_err() 	( REG_MSC_STAT & MSC_STAT_CRC_READ_ERROR )
#define __msc_stat_wr_crc_err() 	( REG_MSC_STAT & MSC_STAT_CRC_WRITE_ERROR_YES )
#define __msc_stat_resto_err() 		( REG_MSC_STAT & MSC_STAT_TIME_OUT_RES )
#define __msc_stat_rdto_err() 		( REG_MSC_STAT & MSC_STAT_TIME_OUT_READ )

#define __msc_rd_resfifo() 		( REG_MSC_RES )
#define __msc_rd_rxfifo()  		( REG_MSC_RXFIFO )
#define __msc_wr_txfifo(v)  		( REG_MSC_TXFIFO = v )

#define __msc_reset() 						\
do { 								\
	REG_MSC_STRPCL = MSC_STRPCL_RESET;			\
 	while (REG_MSC_STAT & MSC_STAT_IS_RESETTING);		\
} while (0)

#define __msc_start_clk() 					\
do { 								\
	REG_MSC_STRPCL = MSC_STRPCL_CLOCK_CONTROL_START;	\
} while (0)

#define __msc_stop_clk() 					\
do { 								\
	REG_MSC_STRPCL = MSC_STRPCL_CLOCK_CONTROL_STOP;	\
} while (0)

#define MMC_CLK 19169200
#define SD_CLK  24576000

/* msc_clk should little than pclk and little than clk retrieve from card */
#define __msc_calc_clk_divisor(type,dev_clk,msc_clk,lv)		\
do {								\
	unsigned int rate, pclk, i;				\
	pclk = dev_clk;						\
	rate = type?SD_CLK:MMC_CLK;				\
  	if (msc_clk && msc_clk < pclk)				\
    		pclk = msc_clk;					\
	i = 0;							\
  	while (pclk < rate)					\
    	{							\
      		i ++;						\
      		rate >>= 1;					\
    	}							\
  	lv = i;							\
} while(0)

/* divide rate to little than or equal to 400kHz */
#define __msc_calc_slow_clk_divisor(type, lv)			\
do {								\
	unsigned int rate, i;					\
	rate = (type?SD_CLK:MMC_CLK)/1000/400;			\
	i = 0;							\
	while (rate > 0)					\
    	{							\
      		rate >>= 1;					\
      		i ++;						\
    	}							\
  	lv = i;							\
} while(0)


/***************************************************************************
 * SSI
 ***************************************************************************/

#define __ssi_enable()    ( REG_SSI_CR0 |= SSI_CR0_SSIE )
#define __ssi_disable()   ( REG_SSI_CR0 &= ~SSI_CR0_SSIE )
#define __ssi_select_ce() ( REG_SSI_CR0 &= ~SSI_CR0_FSEL )

#define __ssi_normal_mode() ( REG_SSI_ITR &= ~SSI_ITR_IVLTM_MASK )

#define __ssi_select_ce2() 		\
do { 					\
	REG_SSI_CR0 |= SSI_CR0_FSEL; 	\
	REG_SSI_CR1 &= ~SSI_CR1_MULTS; 	\
} while (0)

#define __ssi_select_gpc() 		\
do { 					\
	REG_SSI_CR0 &= ~SSI_CR0_FSEL; 	\
	REG_SSI_CR1 |= SSI_CR1_MULTS; 	\
} while (0)

#define __ssi_enable_tx_intr() 	\
  ( REG_SSI_CR0 |= SSI_CR0_TIE | SSI_CR0_TEIE )

#define __ssi_disable_tx_intr() \
  ( REG_SSI_CR0 &= ~(SSI_CR0_TIE | SSI_CR0_TEIE) )

#define __ssi_enable_rx_intr() 	\
  ( REG_SSI_CR0 |= SSI_CR0_RIE | SSI_CR0_REIE )

#define __ssi_disable_rx_intr() \
  ( REG_SSI_CR0 &= ~(SSI_CR0_RIE | SSI_CR0_REIE) )

#define __ssi_enable_txfifo_half_empty_intr()  \
	( REG_SSI_CR0 |= SSI_CR0_TIE )
#define __ssi_disable_txfifo_half_empty_intr()  \
	( REG_SSI_CR0 &= ~SSI_CR0_TIE )
#define __ssi_enable_tx_error_intr()  \
	( REG_SSI_CR0 |= SSI_CR0_TEIE )
#define __ssi_disable_tx_error_intr()  \
	( REG_SSI_CR0 &= ~SSI_CR0_TEIE )

#define __ssi_enable_rxfifo_half_full_intr()  \
	( REG_SSI_CR0 |= SSI_CR0_RIE )
#define __ssi_disable_rxfifo_half_full_intr()  \
	( REG_SSI_CR0 &= ~SSI_CR0_RIE )
#define __ssi_enable_rx_error_intr()  \
	( REG_SSI_CR0 |= SSI_CR0_REIE )
#define __ssi_disable_rx_error_intr()  \
	( REG_SSI_CR0 &= ~SSI_CR0_REIE )

#define __ssi_enable_loopback()  ( REG_SSI_CR0 |= SSI_CR0_LOOP )
#define __ssi_disable_loopback() ( REG_SSI_CR0 &= ~SSI_CR0_LOOP )

#define __ssi_enable_receive()   ( REG_SSI_CR0 &= ~SSI_CR0_DISREV )
#define __ssi_disable_receive()  ( REG_SSI_CR0 |= SSI_CR0_DISREV )

#define __ssi_finish_receive() 	\
  ( REG_SSI_CR0 |= (SSI_CR0_RFINE | SSI_CR0_RFINC) )

#define __ssi_disable_recvfinish() \
  ( REG_SSI_CR0 &= ~(SSI_CR0_RFINE | SSI_CR0_RFINC) )

#define __ssi_flush_txfifo()   ( REG_SSI_CR0 |= SSI_CR0_TFLUSH )
#define __ssi_flush_rxfifo()   ( REG_SSI_CR0 |= SSI_CR0_RFLUSH )

#define __ssi_flush_fifo() \
  ( REG_SSI_CR0 |= SSI_CR0_TFLUSH | SSI_CR0_RFLUSH )

#define __ssi_finish_transmit() ( REG_SSI_CR1 &= ~SSI_CR1_UNFIN )
#define __ssi_wait_transmit() ( REG_SSI_CR1 |= SSI_CR1_UNFIN )

#define __ssi_spi_format() 					\
do { 								\
	REG_SSI_CR1 &= ~SSI_CR1_FMAT_MASK; 			\
	REG_SSI_CR1 |= SSI_CR1_FMAT_SPI; 			\
	REG_SSI_CR1 &= ~(SSI_CR1_TFVCK_MASK|SSI_CR1_TCKFI_MASK);\
	REG_SSI_CR1 |= (SSI_CR1_TFVCK_1 | SSI_CR1_TCKFI_1);	\
} while (0)

/* TI's SSP format, must clear SSI_CR1.UNFIN */
#define __ssi_ssp_format() 					\
do { 								\
	REG_SSI_CR1 &= ~(SSI_CR1_FMAT_MASK | SSI_CR1_UNFIN); 	\
	REG_SSI_CR1 |= SSI_CR1_FMAT_SSP; 			\
} while (0)

/* National's Microwire format, must clear SSI_CR0.RFINE, and set max delay */
#define __ssi_microwire_format() 				\
do { 								\
	REG_SSI_CR1 &= ~SSI_CR1_FMAT_MASK; 			\
	REG_SSI_CR1 |= SSI_CR1_FMAT_MW1; 			\
	REG_SSI_CR1 &= ~(SSI_CR1_TFVCK_MASK|SSI_CR1_TCKFI_MASK);\
	REG_SSI_CR1 |= (SSI_CR1_TFVCK_3 | SSI_CR1_TCKFI_3);	\
	REG_SSI_CR0 &= ~SSI_CR0_RFINE; 				\
} while (0)

/* CE# level (FRMHL), CE# in interval time (ITFRM),
   clock phase and polarity (PHA POL),
   interval time (SSIITR), interval characters/frame (SSIICR) */

 /* frmhl,endian,mcom,flen,pha,pol MASK */
#define SSICR1_MISC_MASK 					\
	( SSI_CR1_FRMHL_MASK | SSI_CR1_LFST | SSI_CR1_MCOM_MASK	\
	| SSI_CR1_FLEN_MASK | SSI_CR1_PHA | SSI_CR1_POL )	\

#define __ssi_spi_set_misc(frmhl,endian,flen,mcom,pha,pol)	\
do { 								\
	REG_SSI_CR1 &= ~SSICR1_MISC_MASK; 			\
	REG_SSI_CR1 |= ((frmhl) << 30) | ((endian) << 25) | 	\
		 (((mcom) - 1) << 12) | (((flen) - 2) << 4) | 	\
	         ((pha) << 1) | (pol); 				\
} while(0)

/* Transfer with MSB or LSB first */
#define __ssi_set_msb() ( REG_SSI_CR1 &= ~SSI_CR1_LFST )
#define __ssi_set_lsb() ( REG_SSI_CR1 |= SSI_CR1_LFST )

#define __ssi_set_frame_length(n) \
    REG_SSI_CR1 = (REG_SSI_CR1 & ~SSI_CR1_FLEN_MASK) | (((n) - 2) << 4) 

/* n = 1 - 16 */
#define __ssi_set_microwire_command_length(n) \
	( REG_SSI_CR1 = ((REG_SSI_CR1 & ~SSI_CR1_MCOM_MASK) | SSI_CR1_MCOM_##n##BIT) )

/* Set the clock phase for SPI */
#define __ssi_set_spi_clock_phase(n) \
	( REG_SSI_CR1 = ((REG_SSI_CR1 & ~SSI_CR1_PHA) | ((n&0x1)<< 1)))

/* Set the clock polarity for SPI */
#define __ssi_set_spi_clock_polarity(n) \
    ( REG_SSI_CR1 = ((REG_SSI_CR1 & ~SSI_CR1_POL) | (n&0x1)) )

/* n = ix8 */
#define __ssi_set_tx_trigger(n) 		\
do { 						\
	REG_SSI_CR1 &= ~SSI_CR1_TTRG_MASK; 	\
	REG_SSI_CR1 |= (n/8)<<SSI_CR1_TTRG_BIT; \
} while (0)

/* n = ix8 */
#define __ssi_set_rx_trigger(n) 		\
do { 						\
	REG_SSI_CR1 &= ~SSI_CR1_RTRG_MASK; 	\
	REG_SSI_CR1 |= (n/8)<<SSI_CR1_RTRG_BIT; \
} while (0)

#define __ssi_get_txfifo_count() \
    ( (REG_SSI_SR & SSI_SR_TFIFONUM_MASK) >> SSI_SR_TFIFONUM_BIT )

#define __ssi_get_rxfifo_count() \
    ( (REG_SSI_SR & SSI_SR_RFIFONUM_MASK) >> SSI_SR_RFIFONUM_BIT )

#define __ssi_transfer_end()	( REG_SSI_SR & SSI_SR_END )
#define __ssi_is_busy()		( REG_SSI_SR & SSI_SR_BUSY )

#define __ssi_txfifo_full()	( REG_SSI_SR & SSI_SR_TFF )
#define __ssi_rxfifo_empty()	( REG_SSI_SR & SSI_SR_RFE )
#define __ssi_rxfifo_half_full()	( REG_SSI_SR & SSI_SR_RFHF )
#define __ssi_txfifo_half_empty()	( REG_SSI_SR & SSI_SR_TFHE )
#define __ssi_underrun()		( REG_SSI_SR & SSI_SR_UNDR )
#define __ssi_overrun()			( REG_SSI_SR & SSI_SR_OVER )
#define __ssi_clear_underrun()		( REG_SSI_SR = ~SSI_SR_UNDR )
#define __ssi_clear_overrun()		( REG_SSI_SR = ~SSI_SR_OVER )
#define __ssi_clear_errors() \
    ( REG_SSI_SR &= ~(SSI_SR_UNDR | SSI_SR_OVER) )


#define __ssi_set_clk(dev_clk, ssi_clk) \
  ( REG_SSI_GR = (dev_clk) / (2*(ssi_clk)) - 1 )

#define __ssi_receive_data()    REG_SSI_DR
#define __ssi_transmit_data(v)  ( REG_SSI_DR = (v) )


/***************************************************************************
 * CIM
 ***************************************************************************/

#define __cim_enable()	( REG_CIM_CTRL |= CIM_CTRL_ENA )
#define __cim_disable()	( REG_CIM_CTRL &= ~CIM_CTRL_ENA )

#define __cim_input_data_inverse()	( REG_CIM_CFG |= CIM_CFG_INV_DAT )
#define __cim_input_data_normal()	( REG_CIM_CFG &= ~CIM_CFG_INV_DAT )

#define __cim_vsync_active_low()	( REG_CIM_CFG |= CIM_CFG_VSP )
#define __cim_vsync_active_high()	( REG_CIM_CFG &= ~CIM_CFG_VSP )

#define __cim_hsync_active_low()	( REG_CIM_CFG |= CIM_CFG_HSP )
#define __cim_hsync_active_high()	( REG_CIM_CFG &= ~CIM_CFG_HSP )

#define __cim_sample_data_at_pclk_falling_edge() \
  ( REG_CIM_CFG |= CIM_CFG_PCP )
#define __cim_sample_data_at_pclk_rising_edge() \
  ( REG_CIM_CFG &= ~CIM_CFG_PCP )

#define __cim_enable_dummy_zero()	( REG_CIM_CFG |= CIM_CFG_DUMMY_ZERO )
#define __cim_disable_dummy_zero()	( REG_CIM_CFG &= ~CIM_CFG_DUMMY_ZERO )

#define __cim_select_external_vsync()	( REG_CIM_CFG |= CIM_CFG_EXT_VSYNC )
#define __cim_select_internal_vsync()	( REG_CIM_CFG &= ~CIM_CFG_EXT_VSYNC )

/* n=0-7 */
#define __cim_set_data_packing_mode(n) 		\
do {						\
    REG_CIM_CFG &= ~CIM_CFG_PACK_MASK; 		\
    REG_CIM_CFG |= (CIM_CFG_PACK_##n); 		\
} while (0)

#define __cim_enable_ccir656_progressive_mode()	\
do {						\
    REG_CIM_CFG &= ~CIM_CFG_DSM_MASK; 		\
    REG_CIM_CFG |= CIM_CFG_DSM_CPM; 		\
} while (0)

#define __cim_enable_ccir656_interlace_mode()	\
do {						\
    REG_CIM_CFG &= ~CIM_CFG_DSM_MASK; 		\
    REG_CIM_CFG |= CIM_CFG_DSM_CIM; 		\
} while (0)

#define __cim_enable_gated_clock_mode()		\
do {						\
    REG_CIM_CFG &= ~CIM_CFG_DSM_MASK; 		\
    REG_CIM_CFG |= CIM_CFG_DSM_GCM; 		\
} while (0)

#define __cim_enable_nongated_clock_mode()	\
do {						\
    REG_CIM_CFG &= ~CIM_CFG_DSM_MASK; 		\
    REG_CIM_CFG |= CIM_CFG_DSM_NGCM; 		\
} while (0)

/* sclk:system bus clock
 * mclk: CIM master clock
 */
#define __cim_set_master_clk(sclk, mclk)			\
do {								\
    REG_CIM_CTRL &= ~CIM_CTRL_MCLKDIV_MASK;			\
    REG_CIM_CTRL |= (((sclk)/(mclk) - 1) << CIM_CTRL_MCLKDIV_BIT);	\
} while (0)

#define __cim_enable_sof_intr() \
  ( REG_CIM_CTRL |= CIM_CTRL_DMA_SOFM )
#define __cim_disable_sof_intr() \
  ( REG_CIM_CTRL &= ~CIM_CTRL_DMA_SOFM )

#define __cim_enable_eof_intr() \
  ( REG_CIM_CTRL |= CIM_CTRL_DMA_EOFM )
#define __cim_disable_eof_intr() \
  ( REG_CIM_CTRL &= ~CIM_CTRL_DMA_EOFM )

#define __cim_enable_stop_intr() \
  ( REG_CIM_CTRL |= CIM_CTRL_DMA_STOPM )
#define __cim_disable_stop_intr() \
  ( REG_CIM_CTRL &= ~CIM_CTRL_DMA_STOPM )

#define __cim_enable_trig_intr() \
  ( REG_CIM_CTRL |= CIM_CTRL_RXF_TRIGM )
#define __cim_disable_trig_intr() \
  ( REG_CIM_CTRL &= ~CIM_CTRL_RXF_TRIGM )

#define __cim_enable_rxfifo_overflow_intr() \
  ( REG_CIM_CTRL |= CIM_CTRL_RXF_OFM )
#define __cim_disable_rxfifo_overflow_intr() \
  ( REG_CIM_CTRL &= ~CIM_CTRL_RXF_OFM )

/* n=1-16 */
#define __cim_set_frame_rate(n) 		\
do {						\
    REG_CIM_CTRL &= ~CIM_CTRL_FRC_MASK; 	\
    REG_CIM_CTRL |= CIM_CTRL_FRC_##n; 		\
} while (0)

#define __cim_enable_dma()   ( REG_CIM_CTRL |= CIM_CTRL_DMA_EN )
#define __cim_disable_dma()  ( REG_CIM_CTRL &= ~CIM_CTRL_DMA_EN )

#define __cim_reset_rxfifo() ( REG_CIM_CTRL |= CIM_CTRL_RXF_RST )
#define __cim_unreset_rxfifo() ( REG_CIM_CTRL &= ~CIM_CTRL_RXF_RST )

/* n=4,8,12,16,20,24,28,32 */
#define __cim_set_rxfifo_trigger(n) 		\
do {						\
    REG_CIM_CTRL &= ~CIM_CTRL_RXF_TRIG_MASK; 	\
    REG_CIM_CTRL |= CIM_CTRL_RXF_TRIG_##n; 	\
} while (0)

#define __cim_clear_state()   	     ( REG_CIM_STATE = 0 )

#define __cim_disable_done()   	     ( REG_CIM_STATE & CIM_STATE_VDD )
#define __cim_rxfifo_empty()   	     ( REG_CIM_STATE & CIM_STATE_RXF_EMPTY )
#define __cim_rxfifo_reach_trigger() ( REG_CIM_STATE & CIM_STATE_RXF_TRIG )
#define __cim_rxfifo_overflow()      ( REG_CIM_STATE & CIM_STATE_RXF_OF )
#define __cim_clear_rxfifo_overflow() ( REG_CIM_STATE &= ~CIM_STATE_RXF_OF )
#define __cim_dma_stop()   	     ( REG_CIM_STATE & CIM_STATE_DMA_STOP )
#define __cim_dma_eof()   	     ( REG_CIM_STATE & CIM_STATE_DMA_EOF )
#define __cim_dma_sof()   	     ( REG_CIM_STATE & CIM_STATE_DMA_SOF )

#define __cim_get_iid()   	     ( REG_CIM_IID )
#define __cim_get_image_data()       ( REG_CIM_RXFIFO )
#define __cim_get_dam_cmd()          ( REG_CIM_CMD )

#define __cim_set_da(a)              ( REG_CIM_DA = (a) )

/***************************************************************************
 * LCD
 ***************************************************************************/
#define __lcd_as_smart_lcd() 		( REG_LCD_CFG |= (1<<LCD_CFG_LCDPIN_BIT) )
#define __lcd_as_general_lcd() 		( REG_LCD_CFG &= ~(1<<LCD_CFG_LCDPIN_BIT) )

#define __lcd_set_dis()			( REG_LCD_CTRL |= LCD_CTRL_DIS )
#define __lcd_clr_dis()			( REG_LCD_CTRL &= ~LCD_CTRL_DIS )

#define __lcd_set_ena()			( REG_LCD_CTRL |= LCD_CTRL_ENA )
#define __lcd_clr_ena()			( REG_LCD_CTRL &= ~LCD_CTRL_ENA )

/* n=1,2,4,8,16 */
#define __lcd_set_bpp(n) \
  ( REG_LCD_CTRL = (REG_LCD_CTRL & ~LCD_CTRL_BPP_MASK) | LCD_CTRL_BPP_##n )

/* n=4,8,16 */
#define __lcd_set_burst_length(n) 		\
do {						\
	REG_LCD_CTRL &= ~LCD_CTRL_BST_MASK;	\
	REG_LCD_CTRL |= LCD_CTRL_BST_n##;	\
} while (0)

#define __lcd_select_rgb565()		( REG_LCD_CTRL &= ~LCD_CTRL_RGB555 )
#define __lcd_select_rgb555()		( REG_LCD_CTRL |= LCD_CTRL_RGB555 )

#define __lcd_set_ofup()		( REG_LCD_CTRL |= LCD_CTRL_OFUP )
#define __lcd_clr_ofup()		( REG_LCD_CTRL &= ~LCD_CTRL_OFUP )

/* n=2,4,16 */
#define __lcd_set_stn_frc(n) 			\
do {						\
	REG_LCD_CTRL &= ~LCD_CTRL_FRC_MASK;	\
	REG_LCD_CTRL |= LCD_CTRL_FRC_n##;	\
} while (0)


#define __lcd_pixel_endian_little()	( REG_LCD_CTRL |= LCD_CTRL_PEDN )
#define __lcd_pixel_endian_big()	( REG_LCD_CTRL &= ~LCD_CTRL_PEDN )

#define __lcd_reverse_byte_endian()	( REG_LCD_CTRL |= LCD_CTRL_BEDN )
#define __lcd_normal_byte_endian()	( REG_LCD_CTRL &= ~LCD_CTRL_BEDN )

#define __lcd_enable_eof_intr()		( REG_LCD_CTRL |= LCD_CTRL_EOFM )
#define __lcd_disable_eof_intr()	( REG_LCD_CTRL &= ~LCD_CTRL_EOFM )

#define __lcd_enable_sof_intr()		( REG_LCD_CTRL |= LCD_CTRL_SOFM )
#define __lcd_disable_sof_intr()	( REG_LCD_CTRL &= ~LCD_CTRL_SOFM )

#define __lcd_enable_ofu_intr()		( REG_LCD_CTRL |= LCD_CTRL_OFUM )
#define __lcd_disable_ofu_intr()	( REG_LCD_CTRL &= ~LCD_CTRL_OFUM )

#define __lcd_enable_ifu0_intr()	( REG_LCD_CTRL |= LCD_CTRL_IFUM0 )
#define __lcd_disable_ifu0_intr()	( REG_LCD_CTRL &= ~LCD_CTRL_IFUM0 )

#define __lcd_enable_ifu1_intr()	( REG_LCD_CTRL |= LCD_CTRL_IFUM1 )
#define __lcd_disable_ifu1_intr()	( REG_LCD_CTRL &= ~LCD_CTRL_IFUM1 )

#define __lcd_enable_ldd_intr()		( REG_LCD_CTRL |= LCD_CTRL_LDDM )
#define __lcd_disable_ldd_intr()	( REG_LCD_CTRL &= ~LCD_CTRL_LDDM )

#define __lcd_enable_qd_intr()		( REG_LCD_CTRL |= LCD_CTRL_QDM )
#define __lcd_disable_qd_intr()		( REG_LCD_CTRL &= ~LCD_CTRL_QDM )


/* LCD status register indication */

#define __lcd_quick_disable_done()	( REG_LCD_STATE & LCD_STATE_QD )
#define __lcd_disable_done()		( REG_LCD_STATE & LCD_STATE_LDD )
#define __lcd_infifo0_underrun()	( REG_LCD_STATE & LCD_STATE_IFU0 )
#define __lcd_infifo1_underrun()	( REG_LCD_STATE & LCD_STATE_IFU1 )
#define __lcd_outfifo_underrun()	( REG_LCD_STATE & LCD_STATE_OFU )
#define __lcd_start_of_frame()		( REG_LCD_STATE & LCD_STATE_SOF )
#define __lcd_end_of_frame()		( REG_LCD_STATE & LCD_STATE_EOF )

#define __lcd_clr_outfifounderrun()	( REG_LCD_STATE &= ~LCD_STATE_OFU )
#define __lcd_clr_sof()			( REG_LCD_STATE &= ~LCD_STATE_SOF )
#define __lcd_clr_eof()			( REG_LCD_STATE &= ~LCD_STATE_EOF )

#define __lcd_panel_white()		( REG_LCD_CFG |= LCD_CFG_WHITE )
#define __lcd_panel_black()		( REG_LCD_CFG &= ~LCD_CFG_WHITE )

/* n=1,2,4,8 for single mono-STN 
 * n=4,8 for dual mono-STN
 */
#define __lcd_set_panel_datawidth(n) 		\
do { 						\
	REG_LCD_CFG &= ~LCD_CFG_PDW_MASK; 	\
	REG_LCD_CFG |= LCD_CFG_PDW_n##;		\
} while (0)

/* m=LCD_CFG_MODE_GENERUIC_TFT_xxx */
#define __lcd_set_panel_mode(m) 		\
do {						\
	REG_LCD_CFG &= ~LCD_CFG_MODE_MASK;	\
	REG_LCD_CFG |= (m);			\
} while(0)

/* n = 0-255 */
#define __lcd_disable_ac_bias()		( REG_LCD_IO = 0xff )
#define __lcd_set_ac_bias(n) 			\
do {						\
	REG_LCD_IO &= ~LCD_IO_ACB_MASK;		\
	REG_LCD_IO |= ((n) << LCD_IO_ACB_BIT);	\
} while(0)

#define __lcd_io_set_dir()		( REG_LCD_IO |= LCD_IO_DIR )
#define __lcd_io_clr_dir()		( REG_LCD_IO &= ~LCD_IO_DIR )

#define __lcd_io_set_dep()		( REG_LCD_IO |= LCD_IO_DEP )
#define __lcd_io_clr_dep()		( REG_LCD_IO &= ~LCD_IO_DEP )

#define __lcd_io_set_vsp()		( REG_LCD_IO |= LCD_IO_VSP )
#define __lcd_io_clr_vsp()		( REG_LCD_IO &= ~LCD_IO_VSP )

#define __lcd_io_set_hsp()		( REG_LCD_IO |= LCD_IO_HSP )
#define __lcd_io_clr_hsp()		( REG_LCD_IO &= ~LCD_IO_HSP )

#define __lcd_io_set_pcp()		( REG_LCD_IO |= LCD_IO_PCP )
#define __lcd_io_clr_pcp()		( REG_LCD_IO &= ~LCD_IO_PCP )

#define __lcd_vsync_get_vps() \
  ( (REG_LCD_VSYNC & LCD_VSYNC_VPS_MASK) >> LCD_VSYNC_VPS_BIT )

#define __lcd_vsync_get_vpe() \
  ( (REG_LCD_VSYNC & LCD_VSYNC_VPE_MASK) >> LCD_VSYNC_VPE_BIT )
#define __lcd_vsync_set_vpe(n) 				\
do {							\
	REG_LCD_VSYNC &= ~LCD_VSYNC_VPE_MASK;		\
	REG_LCD_VSYNC |= (n) << LCD_VSYNC_VPE_BIT;	\
} while (0)

#define __lcd_hsync_get_hps() \
  ( (REG_LCD_HSYNC & LCD_HSYNC_HPS_MASK) >> LCD_HSYNC_HPS_BIT )
#define __lcd_hsync_set_hps(n) 				\
do {							\
	REG_LCD_HSYNC &= ~LCD_HSYNC_HPS_MASK;		\
	REG_LCD_HSYNC |= (n) << LCD_HSYNC_HPS_BIT;	\
} while (0)

#define __lcd_hsync_get_hpe() \
  ( (REG_LCD_HSYNC & LCD_HSYNC_HPE_MASK) >> LCD_VSYNC_HPE_BIT )
#define __lcd_hsync_set_hpe(n) 				\
do {							\
	REG_LCD_HSYNC &= ~LCD_HSYNC_HPE_MASK;		\
	REG_LCD_HSYNC |= (n) << LCD_HSYNC_HPE_BIT;	\
} while (0)

#define __lcd_vat_get_ht() \
  ( (REG_LCD_VAT & LCD_VAT_HT_MASK) >> LCD_VAT_HT_BIT )
#define __lcd_vat_set_ht(n) 				\
do {							\
	REG_LCD_VAT &= ~LCD_VAT_HT_MASK;		\
	REG_LCD_VAT |= (n) << LCD_VAT_HT_BIT;		\
} while (0)

#define __lcd_vat_get_vt() \
  ( (REG_LCD_VAT & LCD_VAT_VT_MASK) >> LCD_VAT_VT_BIT )
#define __lcd_vat_set_vt(n) 				\
do {							\
	REG_LCD_VAT &= ~LCD_VAT_VT_MASK;		\
	REG_LCD_VAT |= (n) << LCD_VAT_VT_BIT;		\
} while (0)

#define __lcd_dah_get_hds() \
  ( (REG_LCD_DAH & LCD_DAH_HDS_MASK) >> LCD_DAH_HDS_BIT )
#define __lcd_dah_set_hds(n) 				\
do {							\
	REG_LCD_DAH &= ~LCD_DAH_HDS_MASK;		\
	REG_LCD_DAH |= (n) << LCD_DAH_HDS_BIT;		\
} while (0)

#define __lcd_dah_get_hde() \
  ( (REG_LCD_DAH & LCD_DAH_HDE_MASK) >> LCD_DAH_HDE_BIT )
#define __lcd_dah_set_hde(n) 				\
do {							\
	REG_LCD_DAH &= ~LCD_DAH_HDE_MASK;		\
	REG_LCD_DAH |= (n) << LCD_DAH_HDE_BIT;		\
} while (0)

#define __lcd_dav_get_vds() \
  ( (REG_LCD_DAV & LCD_DAV_VDS_MASK) >> LCD_DAV_VDS_BIT )
#define __lcd_dav_set_vds(n) 				\
do {							\
	REG_LCD_DAV &= ~LCD_DAV_VDS_MASK;		\
	REG_LCD_DAV |= (n) << LCD_DAV_VDS_BIT;		\
} while (0)

#define __lcd_dav_get_vde() \
  ( (REG_LCD_DAV & LCD_DAV_VDE_MASK) >> LCD_DAV_VDE_BIT )
#define __lcd_dav_set_vde(n) 				\
do {							\
	REG_LCD_DAV &= ~LCD_DAV_VDE_MASK;		\
	REG_LCD_DAV |= (n) << LCD_DAV_VDE_BIT;		\
} while (0)

#define __lcd_cmd0_set_sofint()		( REG_LCD_CMD0 |= LCD_CMD_SOFINT )
#define __lcd_cmd0_clr_sofint()		( REG_LCD_CMD0 &= ~LCD_CMD_SOFINT )
#define __lcd_cmd1_set_sofint()		( REG_LCD_CMD1 |= LCD_CMD_SOFINT )
#define __lcd_cmd1_clr_sofint()		( REG_LCD_CMD1 &= ~LCD_CMD_SOFINT )

#define __lcd_cmd0_set_eofint()		( REG_LCD_CMD0 |= LCD_CMD_EOFINT )
#define __lcd_cmd0_clr_eofint()		( REG_LCD_CMD0 &= ~LCD_CMD_EOFINT )
#define __lcd_cmd1_set_eofint()		( REG_LCD_CMD1 |= LCD_CMD_EOFINT )
#define __lcd_cmd1_clr_eofint()		( REG_LCD_CMD1 &= ~LCD_CMD_EOFINT )

#define __lcd_cmd0_set_pal()		( REG_LCD_CMD0 |= LCD_CMD_PAL )
#define __lcd_cmd0_clr_pal()		( REG_LCD_CMD0 &= ~LCD_CMD_PAL )

#define __lcd_cmd0_get_len() \
  ( (REG_LCD_CMD0 & LCD_CMD_LEN_MASK) >> LCD_CMD_LEN_BIT )
#define __lcd_cmd1_get_len() \
  ( (REG_LCD_CMD1 & LCD_CMD_LEN_MASK) >> LCD_CMD_LEN_BIT )

/*******************************************************
 * SMART LCD
 *******************************************************/

#define __slcd_dma_enable() (REG_SLCD_CTRL |= SLCD_CTRL_DMA_EN)
#define __slcd_dma_disable() \
do {\
	while (REG_SLCD_STATE & SLCD_STATE_BUSY); 	\
	REG_SLCD_CTRL &= ~SLCD_CTRL_DMA_EN;		\
} while(0)

/*******************************************************
 * SMART LCD
 *******************************************************/

#define __slcd_dma_enable() (REG_SLCD_CTRL |= SLCD_CTRL_DMA_EN)
#define __slcd_dma_disable() \
do {\
	while (REG_SLCD_STATE & SLCD_STATE_BUSY); 	\
	REG_SLCD_CTRL &= ~SLCD_CTRL_DMA_EN;		\
} while(0)

/***************************************************************************
 * RTC ops
 ***************************************************************************/

#define __rtc_write_ready()  ( (REG_RTC_RCR & RTC_RCR_WRDY) >> RTC_RCR_WRDY_BIT )
#define __rtc_enabled()        ( REG_RTC_RCR |= RTC_RCR_RTCE )
#define __rtc_disabled()         ( REG_RTC_RCR &= ~RTC_RCR_RTCE )
#define __rtc_enable_alarm()         ( REG_RTC_RCR |= RTC_RCR_AE )
#define __rtc_disable_alarm()         ( REG_RTC_RCR &= ~RTC_RCR_AE )
#define __rtc_enable_alarm_irq()         ( REG_RTC_RCR |= RTC_RCR_AIE )
#define __rtc_disable_alarm_irq()         ( REG_RTC_RCR &= ~RTC_RCR_AIE )
#define __rtc_enable_1Hz_irq()         ( REG_RTC_RCR |= RTC_RCR_1HZIE )
#define __rtc_disable_1Hz_irq()         ( REG_RTC_RCR &= ~RTC_RCR_1HZIE )

#define __rtc_get_1Hz_flag()           ( (REG_RTC_RCR >> RTC_RCR_1HZ_BIT) & 0x1 )
#define __rtc_clear_1Hz_flag()           ( REG_RTC_RCR &= ~RTC_RCR_1HZ )
#define __rtc_get_alarm_flag()           ( (REG_RTC_RCR >> RTC_RCR_AF_BIT) & 0x1 )
#define __rtc_clear_alarm_flag()           ( REG_RTC_RCR &= ~RTC_RCR_AF )

#define __rtc_get_second()   ( REG_RTC_RSR )
#define __rtc_set_second(v)   ( REG_RTC_RSR = v )

#define __rtc_get_alarm_second()   ( REG_RTC_RSAR )
#define __rtc_set_alarm_second(v)   ( REG_RTC_RSAR = v )

#define __rtc_RGR_is_locked()       ( (REG_RTC_RGR >> RTC_RGR_LOCK) )
#define __rtc_lock_RGR()       ( REG_RTC_RGR |= RTC_RGR_LOCK )
#define __rtc_unlock_RGR()       ( REG_RTC_RGR &= ~RTC_RGR_LOCK )
#define __rtc_get_adjc_val()       ( (REG_RTC_RGR & RTC_RGR_ADJC_MASK) >> RTC_RGR_ADJC_BIT )
#define __rtc_set_adjc_val(v)      \
       ( REG_RTC_RGR = ( (REG_RTC_RGR & ~RTC_RGR_ADJC_MASK) | (v << RTC_RGR_ADJC_BIT) ))
#define __rtc_get_nc1Hz_val()       ( (REG_RTC_RGR & RTC_RGR_NC1HZ_MASK) >> RTC_RGR_NC1HZ_BIT )
#define __rtc_set_nc1Hz_val(v)      \
       ( REG_RTC_RGR = ( (REG_RTC_RGR & ~RTC_RGR_NC1HZ_MASK) | (v << RTC_RGR_NC1HZ_BIT) ))

#define __rtc_power_down()            ( REG_RTC_HCR |= RTC_HCR_PD )

#define __rtc_get_hwfcr_val()         ( REG_RTC_HWFCR & RTC_HWFCR_MASK )
#define __rtc_set_hwfcr_val(v)         ( REG_RTC_HWFCR = (v) & RTC_HWFCR_MASK )
#define __rtc_get_hrcr_val()         ( REG_RTC_HRCR & RTC_HRCR_MASK )
#define __rtc_set_hrcr_val(v)         ( REG_RTC_HRCR = (v) & RTC_HRCR_MASK )

#define __rtc_enable_alarm_wakeup()        ( REG_RTC_HWCR |= RTC_HWCR_EALM )
#define __rtc_disable_alarm_wakeup()        ( REG_RTC_HWCR &= ~RTC_HWCR_EALM )

#define __rtc_status_hib_reset_occur()        ( (REG_RTC_HWRSR >> RTC_HWRSR_HR) & 0x1 )
#define __rtc_status_ppr_reset_occur()        ( (REG_RTC_HWRSR >> RTC_HWRSR_PPR) & 0x1 )
#define __rtc_status_wakeup_pin_waken_up()    ( (REG_RTC_HWRSR >> RTC_HWRSR_PIN) & 0x1 )
#define __rtc_status_alarm_waken_up()        ( (REG_RTC_HWRSR >> RTC_HWRSR_ALM) & 0x1 )
#define __rtc_clear_hib_stat_all()               ( REG_RTC_HWRSR = 0 )

#define __rtc_get_scratch_pattern() 		(REG_RTC_HSPR)
#define __rtc_set_scratch_pattern(n) 		(REG_RTC_HSPR = n )



#endif /* __JZ4740_OPS_H__ */
