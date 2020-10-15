/*
 * linux/include/asm-mips/mach-jz4750/ops.h
 *
 * JZ4750 register definition.
 *
 * Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef __JZ4750_OPS_H__
#define __JZ4750_OPS_H__

/*
 * Definition of Module Operations
 */

/***************************************************************************
 * EMC
 ***************************************************************************/
#define is_share_mode() ((REG_EMC_BCR & EMC_BCR_BSR_MASK) == EMC_BCR_BSR_SHARE)
#define is_normal_order() (!(REG_EMC_BCR & EMC_BCR_PK_SEL))

/***************************************************************************
 * GPIO
 ***************************************************************************/

//------------------------------------------------------
// GPIO Pins Description
//
// PORT 0:
//
// PIN/BIT N		FUNC0		FUNC1		NOTE
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
// PIN/BIT N		FUNC0		FUNC1		NOTE
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
//	15		A15/CLE		SA3
//	16		DCS0#		-
//	17		RAS#		-
//	18		CAS#		-
//	19		RDWE#/BUFD#	-
//	20		WE0#		-
//	21		WE1#		-
//	22		WE2#		-
//	23		WE3#		-
//	24		CKO		-		Note1
//	25		CKE		-
//	26		SSI0_CLK	-
//	27		SSI0_DT		-
//	28		SSI0_DR		-
//	29		SSI0_CE0#	-
//	30		SSI0_CE1#_GPC	-
//	31		SSI0_CE2#	-
//
// Note1: BIT24: it is CKO when chip is reset
//
//------------------------------------------------------
// PORT 2:
//
// PIN/BIT N		FUNC0		FUNC1		NOTE
//	0		SD0		A20
//	1		SD1		A21
//	2		SD2		A22
//	3		SD3		A23
//	4		SD4		A24
//	5		SD5		A25
//	6		SD6		-
//	7		SD7		-
//	8		SD8		TSDI0
//	9		SD9		TSDI1
//	10		SD10		TSDI2
//	11		SD11		TSDI3
//	12		SD12		TSDI4
//	13		SD13		TSDI5
//	14		SD14		TSDI6
//	15		SD15		TSDI7
//	16		A16/ALE		SA4
//	17		SA0		A17
//	18		SA1		A18
//	19		SA2		A19
//	20		WAIT#		-		Note2
//	21		CS1#		-
//	22		CS2#		-
//	23		CS3#		-
//	24		CS4#		-
//	25		RD#		-
//	26		WR#		-
//	27		FRB#		-		Note3
//	28		FRE#		-
//	29		FWE#		-
//	30		BOOT_SEL0	-		Note4
//	31		BOOT_SEL1	-		Note5
//
// Note2: BIT20: it is WAIT# pin when chip is reset
//
// Note3: BIT27: when NAND is used, it should connect to NANF FRB#.
//
// Note4: BIT30: it is BOOT_SEL0 when chip is reset, it can used as output GPIO.
//
// Note5: BIT31: it is BOOT_SEL1 when chip is reset, it can used as general GPIO.
//
//------------------------------------------------------
// PORT 3:
//
// PIN/BIT N		FUNC0		FUNC1		NOTE
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
//	22		LCD_CLS		-
//	23		LCD_SPL		-
//	24		LCD_PS		-
//	25		LCD_REV		-
//	26		SSI1_CLK	-
//	27		SSI1_DT		-
//	28		SSI1_DR		-
//	29		SSI1_CE0#	-
//	30		SSI1_CE1#	-
//	31		-		-
//
//------------------------------------------------------
// PORT 4:
//
// PIN/BIT N		FUNC0		FUNC1		NOTE
//	0		CIM_D0		-
//	1		CIM_D1		-
//	2		CIM_D2		-
//	3		CIM_D3		-
//	4		CIM_D4		-
//	5		CIM_D5		-
//	6		CIM_D6		-
//	7		CIM_D7		-
//	8		CIM_MCLK	-
//	9		CIM_PCLK	-
//	10		CIM_VSYNC	-
//	11		CIM_HSYNC	-
//	12		I2C_SDA		-
//	13		I2C_SCK		-
//	14		-		-
//	15		-		-
//	16		UART1_RxD	-
//	17		UART1_TxD	-
//	18		UART1_CTS	PCM_DIN
//	19		UART1_RTS	PCM_DOUT
//	20		PWM0		PCM_CLK
//	21		PWM1		PCM_SYN
//	22		PWM2		SCLK_RSTN
//	23		PWM3		BCLK
//	24		PWM4		SYNC
//	25		PWM5		OWI
//	26		SDATO		UART2_TxD
//	27		SDATI		UART2_RxD
//	28		DCS1#		-
//	29		-		-
//	30		WKUP		-		Note6
//	31		-		-		Note7
//
// Note6: BIT30: it is only used as input and interrupt, and with no pull-up and pull-down
//
// Note7: BIT31: it is used to select the function of UART or JTAG set by PESEL[31]
//        PESEL[31] = 0, select JTAG function
//        PESEL[31] = 1, select UART function
//
//------------------------------------------------------
// PORT 5:
//
// PIN/BIT N		FUNC0		FUNC1		NOTE
//	0		MSC0_D0		-
//	1		MSC0_D1		-
//	2		MSC0_D2		DREQ
//	3		MSC0_D3		DACK
//	4		MSC0_D4		UART0_RxD
//	5		MSC0_D5		UART0_TxD
//	6		MSC0_D6		UART0_CTS
//	7		MSC0_D7		UART0_RTS
//	8		MSC0_CLK	-
//	9		MSC0_CMD	-
//	10		MSC1_D0		-
//	11		MSC1_D1		-
//	12		MSC1_D2		-
//	13		MSC1_D3		-
//	14		MSC1_CLK	-
//	15		MSC1_CMD	-
//	16		UART3_RxD	-
//	17		UART3_TxD	-
//	18		UART3_CTS	-
//	19		UART3_RTS	-
//	20		TSCLK		-
//	21		TSSTR		-
//	22		TSFRM		-
//	23		TSFAIL		-
//	24		-		-
//	25		-		-
//	26		-		-
//	27		-		-
//	28		-		-
//	29		-		-
//	30		-		-
//	31		-		-
//
//////////////////////////////////////////////////////////

/*
 * p is the port number (0,1,2,3,4,5)
 * o is the pin offset (0-31) inside the port
 * n is the absolute number of a pin (0-191), regardless of the port
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
 * D0 ~ D31, A0 ~ A14, DCS0#, RAS#, CAS#,
 * RDWE#, WE0#, WE1#, WE2#, WE3#, CKO#, CKE#
 */
#define __gpio_as_sdram_32bit()			\
do {						\
	REG_GPIO_PXFUNS(0) = 0xffffffff;	\
	REG_GPIO_PXSELC(0) = 0xffffffff;	\
	REG_GPIO_PXPES(0) = 0xffffffff;		\
	REG_GPIO_PXFUNS(1) = 0x03ff7fff;	\
	REG_GPIO_PXSELC(1) = 0x03ff7fff;	\
	REG_GPIO_PXPES(1) = 0x03ff7fff;		\
} while (0)

/*
 * D0 ~ D15, A0 ~ A14, DCS0#, RAS#, CAS#,
 * RDWE#, WE0#, WE1#, WE2#, WE3#, CKO#, CKE#
 */
#define __gpio_as_sdram_16bit()						\
do {								        \
	if (is_normal_order()) {					\
		/* 32/16-bit data normal order */			\
		REG_GPIO_PXFUNS(0) = 0x0000ffff;			\
		REG_GPIO_PXSELC(0) = 0x0000ffff;			\
		REG_GPIO_PXPES(0) = 0x0000ffff;				\
	} else {							\
		/* 16-bit data special order */				\
		REG_GPIO_PXFUNS(0) = 0x00ffff00;			\
		REG_GPIO_PXSELC(0) = 0x00ffff00;			\
		REG_GPIO_PXPES(0) = 0x00ffff00;				\
	}								\
	REG_GPIO_PXFUNS(1) = 0x03ff7fff;				\
	REG_GPIO_PXSELC(1) = 0x03ff7fff;				\
	REG_GPIO_PXPES(1) = 0x03ff7fff;					\
} while (0)

/*
 * D0 ~ D7, CS1#, CLE, ALE, FRE#, FWE#, FRB#, RDWE#/BUFD#
 * @n: chip select number(1 ~ 4)
 */
#define __gpio_as_nand_8bit(n)						\
do {		              						\
	if (!is_share_mode()) {						\
		/* unshare mode */					\
		REG_GPIO_PXFUNS(2) = 0x000000ff; /* SD0~SD7 */		\
		REG_GPIO_PXSELC(2) = 0x000000ff;			\
		REG_GPIO_PXPES(2) = 0x000000ff;				\
		REG_GPIO_PXFUNS(1) = 0x00008000; /* CLE(SA3) */		\
		REG_GPIO_PXSELS(1) = 0x00008000;			\
		REG_GPIO_PXPES(1) = 0x00008000;				\
		REG_GPIO_PXFUNS(2) = 0x00010000; /* ALE(SA4) */		\
		REG_GPIO_PXSELS(2) = 0x00010000;			\
		REG_GPIO_PXPES(2) = 0x00010000;				\
	} else {							\
		/* share mode */					\
		if (is_normal_order()) {	              		\
			/* 32/16-bit data normal order */		\
			REG_GPIO_PXFUNS(0) = 0x000000ff; /* D0~D7 */	\
			REG_GPIO_PXSELC(0) = 0x000000ff;		\
			REG_GPIO_PXPES(0) = 0x000000ff;			\
		} else {						\
			/* 16-bit data special order */			\
			REG_GPIO_PXFUNS(0) = 0x0000ff00; /* D0~D7 */	\
			REG_GPIO_PXSELC(0) = 0x0000ff00;		\
			REG_GPIO_PXPES(0) = 0x0000ff00;			\
		}							\
		REG_GPIO_PXFUNS(1) = 0x00008000; /* CLE(A15) */		\
		REG_GPIO_PXSELC(1) = 0x00008000;			\
		REG_GPIO_PXPES(1) = 0x00008000;				\
		REG_GPIO_PXFUNS(2) = 0x00010000; /* ALE(A16) */		\
		REG_GPIO_PXSELC(2) = 0x00010000;			\
		REG_GPIO_PXPES(2) = 0x00010000;				\
	}								\
	REG_GPIO_PXFUNS(2) = 0x00200000 << ((n)-1); /* CSn */		\
	REG_GPIO_PXSELC(2) = 0x00200000 << ((n)-1);			\
	REG_GPIO_PXPES(2) = 0x00200000 << ((n)-1);			\
									\
        REG_GPIO_PXFUNS(1) = 0x00080000; /* RDWE#/BUFD# */		\
        REG_GPIO_PXSELC(1) = 0x00080000;				\
	REG_GPIO_PXPES(1) = 0x00080000;					\
	REG_GPIO_PXFUNS(2) = 0x30000000; /* FRE#, FWE# */		\
	REG_GPIO_PXSELC(2) = 0x30000000;				\
	REG_GPIO_PXPES(2) = 0x30000000;					\
	REG_GPIO_PXFUNC(2) = 0x08000000; /* FRB#(input) */		\
	REG_GPIO_PXSELC(2) = 0x08000000;				\
	REG_GPIO_PXDIRC(2) = 0x08000000;				\
	REG_GPIO_PXPES(2) = 0x08000000;					\
} while (0)


/*
 * CS4#, RD#, WR#, WAIT#, A0 ~ A22, D0 ~ D7
 * @n: chip select number(1 ~ 4)
 */
#define __gpio_as_nor_8bit(n)						\
do {								        \
	if (is_normal_order()) {					\
		/* 32/16-bit data normal order */			\
		REG_GPIO_PXFUNS(0) = 0x000000ff;			\
		REG_GPIO_PXSELC(0) = 0x000000ff;			\
		REG_GPIO_PXPES(0) = 0x000000ff;				\
	} else {							\
		/* 16-bit data special order */				\
		REG_GPIO_PXFUNS(0) = 0x0000ff00;			\
		REG_GPIO_PXSELC(0) = 0x0000ff00;			\
		REG_GPIO_PXPES(0) = 0x0000ff00;				\
	}								\
	REG_GPIO_PXFUNS(2) = 0x00200000 << ((n)-1); /* CSn */		\
	REG_GPIO_PXSELC(2) = 0x00200000 << ((n)-1);			\
	REG_GPIO_PXPES(2) = 0x00200000 << ((n)-1);			\
									\
	REG_GPIO_PXFUNS(1) = 0x0000ffff; /* A0~A15 */			\
	REG_GPIO_PXSELC(1) = 0x0000ffff;				\
	REG_GPIO_PXPES(1) = 0x0000ffff;					\
	REG_GPIO_PXFUNS(2) = 0x06110007; /* RD#, WR#, WAIT#, A20~A22 */	\
	REG_GPIO_PXSELC(2) = 0x06110007;				\
	REG_GPIO_PXPES(2) = 0x06110007;					\
	REG_GPIO_PXFUNS(2) = 0x000e0000; /* A17~A19 */	        	\
	REG_GPIO_PXSELS(2) = 0x000e0000;				\
	REG_GPIO_PXPES(2) = 0x000e0000;					\
} while (0)

/*
 * CS4#, RD#, WR#, WAIT#, A0 ~ A22, D0 ~ D15
 * @n: chip select number(1 ~ 4)
 */
#define __gpio_as_nor_16bit(n)						\
do {	               							\
	if (is_normal_order()) {					\
		/* 32/16-bit data normal order */			\
		REG_GPIO_PXFUNS(0) = 0x0000ffff;			\
		REG_GPIO_PXSELC(0) = 0x0000ffff;			\
		REG_GPIO_PXPES(0) = 0x0000ffff;				\
	} else {							\
		/* 16-bit data special order */				\
		REG_GPIO_PXFUNS(0) = 0x00ffff00;			\
		REG_GPIO_PXSELC(0) = 0x00ffff00;			\
		REG_GPIO_PXPES(0) = 0x00ffff00;				\
	}								\
	REG_GPIO_PXFUNS(2) = 0x00200000 << ((n)-1); /* CSn */		\
	REG_GPIO_PXSELC(2) = 0x00200000 << ((n)-1);			\
	REG_GPIO_PXPES(2) = 0x00200000 << ((n)-1);			\
									\
	REG_GPIO_PXFUNS(1) = 0x0000ffff; /* A0~A15 */			\
	REG_GPIO_PXSELC(1) = 0x0000ffff;				\
	REG_GPIO_PXPES(1) = 0x0000ffff;					\
	REG_GPIO_PXFUNS(2) = 0x06110007; /* RD#, WR#, WAIT#, A20~A22 */	\
	REG_GPIO_PXSELC(2) = 0x06110007;				\
	REG_GPIO_PXPES(2) = 0x06110007;					\
	REG_GPIO_PXFUNS(2) = 0x000e0000; /* A17~A19 */	        	\
	REG_GPIO_PXSELS(2) = 0x000e0000;				\
	REG_GPIO_PXPES(2) = 0x000e0000;					\
} while (0)

/*
 * UART0_TxD, UART0_RxD
 */
#define __gpio_as_uart0()			\
do {						\
	REG_GPIO_PXFUNS(5) = 0x00000030;	\
	REG_GPIO_PXSELS(5) = 0x00000030;	\
	REG_GPIO_PXPES(5) = 0x00000030;		\
} while (0)

/*
 * UART0_TxD, UART0_RxD, UART0_CTS, UART0_RTS
 */
#define __gpio_as_uart0_ctsrts()		\
do {						\
	REG_GPIO_PXFUNS(5) = 0x000000f0;	\
	REG_GPIO_PXSELS(5) = 0x000000f0;	\
	REG_GPIO_PXPES(5) = 0x000000f0;		\
} while (0)

/*
 * UART1_TxD, UART1_RxD
 */
#define __gpio_as_uart1()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x00030000;	\
	REG_GPIO_PXSELC(4) = 0x00030000;	\
	REG_GPIO_PXPES(4) = 0x00030000;		\
} while (0)

/*
 * UART1_TxD, UART1_RxD, UART1_CTS, UART1_RTS
 */
#define __gpio_as_uart1_ctsrts()		\
do {						\
	REG_GPIO_PXFUNS(4) = 0x000f0000;	\
	REG_GPIO_PXSELC(4) = 0x000f0000;	\
	REG_GPIO_PXPES(4) = 0x000f0000;		\
} while (0)

/*
 * UART2_TxD, UART2_RxD
 */
#define __gpio_as_uart2()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x0c000000;	\
	REG_GPIO_PXSELS(4) = 0x0c000000;	\
	REG_GPIO_PXPES(4) = 0x0c000000;		\
} while (0)

/*
 * UART3_TxD, UART3_RxD
 */
#define __gpio_as_uart3()			\
do {						\
	REG_GPIO_PXFUNS(5) = 0x00030000;	\
	REG_GPIO_PXSELC(5) = 0x00030000;	\
	REG_GPIO_PXPES(5) = 0x00030000;		\
} while (0)

/*
 * UART3_TxD, UART3_RxD, UART3_CTS, UART3_RTS
 */
#define __gpio_as_uart3_ctsrts()		\
do {						\
	REG_GPIO_PXFUNS(5) = 0x000f0000;	\
	REG_GPIO_PXSELC(5) = 0x000f0000;	\
	REG_GPIO_PXPES(5) = 0x000f0000;		\
} while (0)

/*
 * TSCLK, TSSTR, TSFRM, TSFAIL, TSDI0~7
 */
#define __gpio_as_tssi()			\
do {						\
	REG_GPIO_PXFUNS(2) = 0x0000ff00;	\
	REG_GPIO_PXSELS(2) = 0x0000ff00;	\
	REG_GPIO_PXPES(2) = 0x0000ff00;		\
	REG_GPIO_PXFUNS(5) = 0x00f00000;	\
	REG_GPIO_PXSELC(5) = 0x00f00000;	\
	REG_GPIO_PXPES(5) = 0x00f00000;		\
} while (0)

/*
 * LCD_D0~LCD_D7, LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_8bit()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x003c00ff;	\
	REG_GPIO_PXSELC(3) = 0x003c00ff;	\
	REG_GPIO_PXPES(3) = 0x003c00ff;		\
} while (0)

/*
 * LCD_D0~LCD_D15, LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_16bit()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x003cffff;	\
	REG_GPIO_PXSELC(3) = 0x003cffff;	\
	REG_GPIO_PXPES(3) = 0x003cffff;		\
} while (0)

/*
 * LCD_D0~LCD_D17, LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_18bit()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x003fffff;	\
	REG_GPIO_PXSELC(3) = 0x003fffff;	\
	REG_GPIO_PXPES(3) = 0x003fffff;		\
} while (0)

/*
 * LCD_D0~LCD_D17, LCD_D_R1, LCD_D_G0, LCD_D_G1, LCD_D_B1,
 * LCD_D_R0, LCD_D_B0, LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_24bit()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x003fffff;	\
	REG_GPIO_PXSELC(3) = 0x003fffff;	\
	REG_GPIO_PXPES(3)  = 0x003fffff;	\
	REG_GPIO_PXFUNS(3) = 0x03c00000;	\
	REG_GPIO_PXSELS(3) = 0x03c00000;	\
	REG_GPIO_PXPES(3)  = 0x03c00000;	\
	REG_GPIO_PXFUNS(5) = 0x000c0000;	\
	REG_GPIO_PXSELS(5) = 0x000c0000;	\
	REG_GPIO_PXPES(5)  = 0x000c0000;	\
} while (0)

/*
 * SLCD_DAT0~7, SLCD_CLK, SLCD_RS, SLCD_CS
 */
#define __gpio_as_lcd_smart_pal_8bit()		\
do {						\
	REG_GPIO_PXFUNS(3) = 0x001c00ff;	\
	REG_GPIO_PXSELC(3) = 0x001c00ff;	\
	REG_GPIO_PXPES(3) = 0x001c00ff;		\
} while (0)

/*
 * SLCD_DAT0~15, SLCD_CLK, SLCD_RS, SLCD_CS
 */
#define __gpio_as_lcd_smart_pal_15bit()		\
do {						\
	REG_GPIO_PXFUNS(3) = 0x001cffff;	\
	REG_GPIO_PXSELC(3) = 0x001cffff;	\
	REG_GPIO_PXPES(3) = 0x001cffff;		\
} while (0)

/*
 * SLCD_DAT0~17, SLCD_CLK, SLCD_RS, SLCD_CS
 */
#define __gpio_as_lcd_smart_pal_17bit()		\
do {						\
	REG_GPIO_PXFUNS(3) = 0x001fffff;	\
	REG_GPIO_PXSELC(3) = 0x001fffff;	\
	REG_GPIO_PXPES(3) = 0x001fffff;		\
} while (0)

/*
 * SLCD_DAT15, SLCD_CLK, SLCD_RS, SLCD_CS
 */
#define __gpio_as_lcd_smart_serial()		\
do {						\
	REG_GPIO_PXFUNS(3) = 0x001c8000;	\
	REG_GPIO_PXSELC(3) = 0x001c8000;	\
	REG_GPIO_PXPES(3) = 0x001c8000;		\
} while (0)

/*
 *  LCD_CLS, LCD_SPL, LCD_PS, LCD_REV
 */
#define __gpio_as_lcd_special()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x03C00000;	\
	REG_GPIO_PXSELC(3) = 0x03C00000;	\
	REG_GPIO_PXPES(3)  = 0x03C00000;	\
} while (0)

/*
 * CIM_D0~CIM_D7, CIM_MCLK, CIM_PCLK, CIM_VSYNC, CIM_HSYNC
 */
#define __gpio_as_cim()				\
do {						\
	REG_GPIO_PXFUNS(4) = 0x00000fff;	\
	REG_GPIO_PXSELC(4) = 0x00000fff;	\
	REG_GPIO_PXPES(4)  = 0x00000fff;	\
} while (0)

/*
 * SDATO, SDATI, BCLK, SYNC, SCLK_RSTN(gpio sepc) or
 * SDATA_OUT, SDATA_IN, BIT_CLK, SYNC, SCLK_RESET(aic spec)
 */
#define __gpio_as_aic()				\
do {						\
	REG_GPIO_PXFUNS(4) = 0x0c000000;	\
	REG_GPIO_PXSELS(4) = 0x0c000000;	\
	REG_GPIO_PXPES(4)  = 0x0c000000;	\
	REG_GPIO_PXFUNS(4) = 0x00e00000;	\
	REG_GPIO_PXSELC(4) = 0x00e00000;	\
	REG_GPIO_PXPES(4)  = 0x00e00000;	\
} while (0)

/*
 * PCM_DIN, PCM_DOUT, PCM_CLK, PCM_SYN
*/
#define __gpio_as_pcm() 			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x003c0000;	\
	REG_GPIO_PXSELS(4) = 0x003c0000;	\
	REG_GPIO_PXPES(4)  = 0x003c0000; 	\
} while (0)

/*
 * OWI
*/
#define __gpio_as_owi() 			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x02000000;	\
	REG_GPIO_PXSELS(4) = 0x02000000;	\
	REG_GPIO_PXPES(4) = 0x02000000;		\
} while (0)

/*
 * MSC0_CMD, MSC0_CLK, MSC0_D0 ~ MSC0_D3
 */
#define __gpio_as_msc0_4bit()			\
do {						\
	REG_GPIO_PXFUNS(5) = 0x0000030f;	\
	REG_GPIO_PXSELC(5) = 0x0000030f;	\
	REG_GPIO_PXPES(5)  = 0x0000030f;	\
} while (0)

/*
 * MSC0_CMD, MSC0_CLK, MSC0_D0 ~ MSC0_D7
 */
#define __gpio_as_msc0_8bit()			\
do {						\
	REG_GPIO_PXFUNS(5) = 0x000003ff;	\
	REG_GPIO_PXSELC(5) = 0x000003ff;	\
	REG_GPIO_PXPES(5)  = 0x000003ff;	\
} while (0)

/*
 * MSC1_CMD, MSC1_CLK, MSC1_D0 ~ MSC1_D3
 */
#define __gpio_as_msc1_4bit()			\
do {						\
	REG_GPIO_PXFUNS(5) = 0x0000fc00;	\
	REG_GPIO_PXSELC(5) = 0x0000fc00;	\
	REG_GPIO_PXPES(5)  = 0x0000fc00;	\
} while (0)

#define __gpio_as_msc 	__gpio_as_msc0_8bit /* default as msc0 8bit */
#define __gpio_as_msc0 	__gpio_as_msc0_8bit /* msc0 default as 8bit */
#define __gpio_as_msc1 	__gpio_as_msc1_4bit /* msc1 only support 4bit */

/*
 * SSI0_CE0, SSI0_CE1#_GPC, SSI0_CE2, SSI0_CLK, SSI0_DT, SSI0_DR
 */
#define __gpio_as_ssi0()			\
do {						\
	REG_GPIO_PXFUNS(1) = 0xfc000000;	\
	REG_GPIO_PXSELC(1) = 0xfc000000;	\
	REG_GPIO_PXPES(1)  = 0xfc000000;	\
} while (0)

/*
 * SSI1_CE0, SSI1_CE1, SSI1_CLK, SSI1_DT, SSI1_DR
 */
#define __gpio_as_ssi1()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x7c000000;	\
	REG_GPIO_PXSELC(3) = 0x7c000000;	\
	REG_GPIO_PXPES(3)  = 0x7c000000;	\
} while (0)

/* n = 0(SSI0), 1(SSI1) */
#define __gpio_as_ssi(n)	 __gpio_as_ssi##n()

#define __gpio_as_ssi0_x()			\
do {						\
	REG_GPIO_PXFUNS(1) = 0x1c000000;	\
	REG_GPIO_PXSELC(1) = 0x1c000000;	\
	REG_GPIO_PXPES(1)  = 0x1c000000;	\
} while (0)

#define __gpio_as_ssi1_x()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x1c000000;	\
	REG_GPIO_PXSELC(3) = 0x1c000000;	\
	REG_GPIO_PXPES(3)  = 0x1c000000;	\
} while (0)

/*
 * I2C_SCK, I2C_SDA
 */
#define __gpio_as_i2c()				\
do {						\
	REG_GPIO_PXFUNS(4) = 0x00003000;	\
	REG_GPIO_PXSELC(4) = 0x00003000;	\
	REG_GPIO_PXPES(4)  = 0x00003000;	\
} while (0)

/*
 * PWM0
 */
#define __gpio_as_pwm0()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x00100000;	\
	REG_GPIO_PXSELC(4) = 0x00100000;	\
	REG_GPIO_PXPES(4) = 0x00100000;		\
} while (0)

/*
 * PWM1
 */
#define __gpio_as_pwm1()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x00200000;	\
	REG_GPIO_PXSELC(4) = 0x00200000;	\
	REG_GPIO_PXPES(4) = 0x00200000;		\
} while (0)

/*
 * PWM2
 */
#define __gpio_as_pwm2()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x00400000;	\
	REG_GPIO_PXSELC(4) = 0x00400000;	\
	REG_GPIO_PXPES(4) = 0x00400000;		\
} while (0)

/*
 * PWM3
 */
#define __gpio_as_pwm3()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x00800000;	\
	REG_GPIO_PXSELC(4) = 0x00800000;	\
	REG_GPIO_PXPES(4) = 0x00800000;		\
} while (0)

/*
 * PWM4
 */
#define __gpio_as_pwm4()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x01000000;	\
	REG_GPIO_PXSELC(4) = 0x01000000;	\
	REG_GPIO_PXPES(4) = 0x01000000;		\
} while (0)

/*
 * PWM5
 */
#define __gpio_as_pwm5()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x02000000;	\
	REG_GPIO_PXSELC(4) = 0x02000000;	\
	REG_GPIO_PXPES(4) = 0x02000000;		\
} while (0)

/*
 * n = 0 ~ 5
 */
#define __gpio_as_pwm(n)	__gpio_as_pwm##n()

/*
 * DREQ
 */
#define __gpio_as_dreq()			\
do {						\
	REG_GPIO_PXFUNS(5) = 0x00000004;	\
	REG_GPIO_PXSELS(5) = 0x00000004;	\
	REG_GPIO_PXPES(5) = 0x00000004;		\
} while (0)

/*
 * DACK
 */
#define __gpio_as_dack()			\
do {						\
	REG_GPIO_PXFUNS(5) = 0x00000008;	\
	REG_GPIO_PXSELS(5) = 0x00000008;	\
	REG_GPIO_PXPES(5) = 0x00000008;		\
} while (0)

/*
 * GPIO or Interrupt Mode
 */
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
#define __cpm_get_mscdiv(n) \
	((REG_CPM_MSCCDR(n) & CPM_MSCCDR_MSCDIV_MASK) >> CPM_MSCCDR_MSCDIV_BIT)
#define __cpm_get_uhcdiv() \
	((REG_CPM_UHCCDR & CPM_UHCCDR_UHCDIV_MASK) >> CPM_UHCCDR_UHCDIV_BIT)
#define __cpm_get_ssidiv() \
	((REG_CPM_SSICDR & CPM_SSICDR_SSIDIV_MASK) >> CPM_SSICDR_SSIDIV_BIT)
#define __cpm_get_pcmdiv(v) \
	((REG_CPM_PCMCDR & CPM_PCMCDR_PCMCD_MASK) >> CPM_PCMCDR_PCMCD_BIT)

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
#define __cpm_set_mscdiv(n, v) \
	(REG_CPM_MSCCDR(n) = (REG_CPM_MSCCDR(n) & ~CPM_MSCCDR_MSCDIV_MASK) | ((v) << (CPM_MSCCDR_MSCDIV_BIT)))
#define __cpm_set_uhcdiv(v) \
	(REG_CPM_UHCCDR = (REG_CPM_UHCCDR & ~CPM_UHCCDR_UHCDIV_MASK) | ((v) << (CPM_UHCCDR_UHCDIV_BIT)))
#define __cpm_set_ssidiv(v) \
	(REG_CPM_SSICDR = (REG_CPM_SSICDR & ~CPM_SSICDR_SSIDIV_MASK) | ((v) << (CPM_SSICDR_SSIDIV_BIT)))
#define __cpm_set_pcmdiv(v) \
	(REG_CPM_PCMCDR = (REG_CPM_PCMCDR & ~CPM_PCMCDR_PCMCD_MASK) | ((v) << (CPM_PCMCDR_PCMCD_BIT)))

#define __cpm_select_pcmclk_pll() 	(REG_CPM_PCMCDR |= CPM_PCMCDR_PCMS)
#define __cpm_select_pcmclk_exclk() 	(REG_CPM_PCMCDR &= ~CPM_PCMCDR_PCMS)
#define __cpm_select_pixclk_ext()	(REG_CPM_LPCDR |= CPM_LPCDR_LPCS)
#define __cpm_select_pixclk_pll()	(REG_CPM_LPCDR &= ~CPM_LPCDR_LPCS)
#define __cpm_select_tveclk_exclk()	(REG_CPM_LPCDR |= CPM_CPCCR_LSCS)
#define __cpm_select_tveclk_pll()	(REG_CPM_LPCDR &= ~CPM_LPCDR_LSCS)
#define __cpm_select_pixclk_lcd()	(REG_CPM_LPCDR &= ~CPM_LPCDR_LTCS)
#define __cpm_select_pixclk_tve()	(REG_CPM_LPCDR |= CPM_LPCDR_LTCS)
#define __cpm_select_i2sclk_exclk()	(REG_CPM_CPCCR &= ~CPM_CPCCR_I2CS)
#define __cpm_select_i2sclk_pll()	(REG_CPM_CPCCR |= CPM_CPCCR_I2CS)
#define __cpm_select_usbclk_exclk()	(REG_CPM_CPCCR &= ~CPM_CPCCR_UCS)
#define __cpm_select_usbclk_pll()	(REG_CPM_CPCCR |= CPM_CPCCR_UCS)

#define __cpm_enable_cko()
#define __cpm_exclk_direct()		(REG_CPM_CPCCR &= ~CPM_CPCCR_ECS)
#define __cpm_exclk_div2()             	(REG_CPM_CPCCR |= CPM_CPCCR_ECS)
#define __cpm_enable_pll_change()	(REG_CPM_CPCCR |= CPM_CPCCR_CE)
#define __cpm_pllout_direct()		(REG_CPM_CPCCR |= CPM_CPCCR_PCS)
#define __cpm_pllout_div2()		(REG_CPM_CPCCR &= ~CPM_CPCCR_PCS)
#define __cpm_pll_enable()		(REG_CPM_CPPCR |= CPM_CPPCR_PLLEN)

#define __cpm_pll_is_off()		(REG_CPM_CPPSR & CPM_CPPSR_PLLOFF)
#define __cpm_pll_is_on()		(REG_CPM_CPPSR & CPM_CPPSR_PLLON)
#define __cpm_pll_bypass()		(REG_CPM_CPPSR |= CPM_CPPSR_PLLBP)

#define __cpm_get_cclk_doze_duty() \
	((REG_CPM_LCR & CPM_LCR_DOZE_DUTY_MASK) >> CPM_LCR_DOZE_DUTY_BIT)
#define __cpm_set_cclk_doze_duty(v) \
	(REG_CPM_LCR = (REG_CPM_LCR & ~CPM_LCR_DOZE_DUTY_MASK) | ((v) << (CPM_LCR_DOZE_DUTY_BIT)))

#define __cpm_doze_mode()		(REG_CPM_LCR |= CPM_LCR_DOZE_ON)
#define __cpm_idle_mode() \
	(REG_CPM_LCR = (REG_CPM_LCR & ~CPM_LCR_LPM_MASK) | CPM_LCR_LPM_IDLE)
#define __cpm_sleep_mode() \
	(REG_CPM_LCR = (REG_CPM_LCR & ~CPM_LCR_LPM_MASK) | CPM_LCR_LPM_SLEEP)

#define __cpm_stop_all() 	(REG_CPM_CLKGR = 0x1fffffff)
#define __cpm_stop_cimram()	(REG_CPM_CLKGR |= CPM_CLKGR_CIMRAM)
#define __cpm_stop_idct()	(REG_CPM_CLKGR |= CPM_CLKGR_IDCT)
#define __cpm_stop_db()	        (REG_CPM_CLKGR |= CPM_CLKGR_DB)
#define __cpm_stop_me()	        (REG_CPM_CLKGR |= CPM_CLKGR_ME)
#define __cpm_stop_mc()	        (REG_CPM_CLKGR |= CPM_CLKGR_MC)
#define __cpm_stop_tve()        (REG_CPM_CLKGR |= CPM_CLKGR_TVE)
#define __cpm_stop_tssi()       (REG_CPM_CLKGR |= CPM_CLKGR_TSSI)
#define __cpm_stop_owi()        (REG_CPM_CLKGR |= CPM_CLKGR_OWI)
#define __cpm_stop_pcm()        (REG_CPM_CLKGR |= CPM_CLKGR_PCM)
#define __cpm_stop_uart3()	(REG_CPM_CLKGR |= CPM_CLKGR_UART3)
#define __cpm_stop_uart2()	(REG_CPM_CLKGR |= CPM_CLKGR_UART2)
#define __cpm_stop_uart1()	(REG_CPM_CLKGR |= CPM_CLKGR_UART1)
#define __cpm_stop_uhc()	(REG_CPM_CLKGR |= CPM_CLKGR_UHC)
#define __cpm_stop_ipu()	(REG_CPM_CLKGR |= CPM_CLKGR_IPU)
#define __cpm_stop_dmac()	(REG_CPM_CLKGR |= CPM_CLKGR_DMAC)
#define __cpm_stop_udc()	(REG_CPM_CLKGR |= CPM_CLKGR_UDC)
#define __cpm_stop_lcd()	(REG_CPM_CLKGR |= CPM_CLKGR_LCD)
#define __cpm_stop_cim()	(REG_CPM_CLKGR |= CPM_CLKGR_CIM)
#define __cpm_stop_sadc()	(REG_CPM_CLKGR |= CPM_CLKGR_SADC)
#define __cpm_stop_msc(n)	(REG_CPM_CLKGR |= CPM_CLKGR_MSC##n)
#define __cpm_stop_aic1()	(REG_CPM_CLKGR |= CPM_CLKGR_AIC1)
#define __cpm_stop_aic2()	(REG_CPM_CLKGR |= CPM_CLKGR_AIC2)
#define __cpm_stop_ssi(n)	(REG_CPM_CLKGR |= CPM_CLKGR_SSI##n)
#define __cpm_stop_i2c()	(REG_CPM_CLKGR |= CPM_CLKGR_I2C)
#define __cpm_stop_rtc()	(REG_CPM_CLKGR |= CPM_CLKGR_RTC)
#define __cpm_stop_tcu()	(REG_CPM_CLKGR |= CPM_CLKGR_TCU)
#define __cpm_stop_uart0()	(REG_CPM_CLKGR |= CPM_CLKGR_UART0)

#define __cpm_start_all() 	(REG_CPM_CLKGR = 0x0)
#define __cpm_start_cimram()	(REG_CPM_CLKGR &= ~CPM_CLKGR_CIMRAM)
#define __cpm_start_idct()	(REG_CPM_CLKGR &= ~CPM_CLKGR_IDCT)
#define __cpm_start_db()        (REG_CPM_CLKGR &= ~CPM_CLKGR_DB)
#define __cpm_start_me()        (REG_CPM_CLKGR &= ~CPM_CLKGR_ME)
#define __cpm_start_mc()        (REG_CPM_CLKGR &= ~CPM_CLKGR_MC)
#define __cpm_start_tve()        (REG_CPM_CLKGR &= ~CPM_CLKGR_TVE)
#define __cpm_start_tssi()       (REG_CPM_CLKGR &= ~CPM_CLKGR_TSSI)
#define __cpm_start_owi()        (REG_CPM_CLKGR &= ~CPM_CLKGR_OWI)
#define __cpm_start_pcm()        (REG_CPM_CLKGR &= ~CPM_CLKGR_PCM)
#define __cpm_start_uart3()	(REG_CPM_CLKGR &= ~CPM_CLKGR_UART3)
#define __cpm_start_uart2()	(REG_CPM_CLKGR &= ~CPM_CLKGR_UART2)
#define __cpm_start_uart1()	(REG_CPM_CLKGR &= ~CPM_CLKGR_UART1)
#define __cpm_start_uhc()	(REG_CPM_CLKGR &= ~CPM_CLKGR_UHC)
#define __cpm_start_ipu()	(REG_CPM_CLKGR &= ~CPM_CLKGR_IPU)
#define __cpm_start_dmac()	(REG_CPM_CLKGR &= ~CPM_CLKGR_DMAC)
#define __cpm_start_udc()	(REG_CPM_CLKGR &= ~CPM_CLKGR_UDC)
#define __cpm_start_lcd()	(REG_CPM_CLKGR &= ~CPM_CLKGR_LCD)
#define __cpm_start_cim()	(REG_CPM_CLKGR &= ~CPM_CLKGR_CIM)
#define __cpm_start_sadc()	(REG_CPM_CLKGR &= ~CPM_CLKGR_SADC)
#define __cpm_start_msc(n)	(REG_CPM_CLKGR &= ~CPM_CLKGR_MSC##n)
#define __cpm_start_aic1()	(REG_CPM_CLKGR &= ~CPM_CLKGR_AIC1)
#define __cpm_start_aic2()	(REG_CPM_CLKGR &= ~CPM_CLKGR_AIC2)
#define __cpm_start_ssi(n)	(REG_CPM_CLKGR &= ~CPM_CLKGR_SSI##n)
#define __cpm_start_i2c()	(REG_CPM_CLKGR &= ~CPM_CLKGR_I2C)
#define __cpm_start_rtc()	(REG_CPM_CLKGR &= ~CPM_CLKGR_RTC)
#define __cpm_start_tcu()	(REG_CPM_CLKGR &= ~CPM_CLKGR_TCU)
#define __cpm_start_uart0()	(REG_CPM_CLKGR &= ~CPM_CLKGR_UART0)

#define __cpm_get_o1st() \
	((REG_CPM_OPCR & CPM_OPCR_O1ST_MASK) >> CPM_OPCR_O1ST_BIT)
#define __cpm_set_o1st(v) \
	(REG_CPM_OPCR = (REG_CPM_OPCR & ~CPM_OPCR_O1ST_MASK) | ((v) << (CPM_OPCR_O1ST_BIT)))
#define __cpm_enable_uhcphy()		(REG_CPM_OPCR &= ~CPM_OPCR_UHCPHY_DISABLE)
#define __cpm_suspend_uhcphy()		(REG_CPM_OPCR |= CPM_OPCR_UHCPHY_DISABLE)
#define __cpm_enable_udcphy()		(REG_CPM_OPCR |= CPM_OPCR_UDCPHY_ENABLE)
#define __cpm_suspend_udcphy()		(REG_CPM_OPCR &= ~CPM_OPCR_UDCPHY_ENABLE)
#define __cpm_enable_osc_in_sleep()	(REG_CPM_OPCR |= CPM_OPCR_OSC_ENABLE)
#define __cpm_disable_osc_in_sleep()	(REG_CPM_OPCR &= ~CPM_OPCR_OSC_ENABLE)
#define __cpm_select_rtcclk_rtc()	(REG_CPM_OPCR |= CPM_OPCR_ERCS)
#define __cpm_select_rtcclk_exclk()	(REG_CPM_OPCR &= ~CPM_OPCR_ERCS)


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
#define __tcu_disable_pclk(n) \
	REG_TCU_TCSR(n) = (REG_TCU_TCSR((n)) & ~TCU_TCSR_PCK_EN);
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

#define __tcu_enable_pwm_output(n)	(REG_TCU_TCSR((n)) |= TCU_TCSR_PWM_EN)
#define __tcu_disable_pwm_output(n)	(REG_TCU_TCSR((n)) &= ~TCU_TCSR_PWM_EN)

#define __tcu_init_pwm_output_high(n)	(REG_TCU_TCSR((n)) |= TCU_TCSR_PWM_INITL_HIGH)
#define __tcu_init_pwm_output_low(n)	(REG_TCU_TCSR((n)) &= ~TCU_TCSR_PWM_INITL_HIGH)

#define __tcu_set_pwm_output_shutdown_graceful(n)	(REG_TCU_TCSR((n)) &= ~TCU_TCSR_PWM_SD)
#define __tcu_set_pwm_output_shutdown_abrupt(n)		(REG_TCU_TCSR((n)) |= TCU_TCSR_PWM_SD)

#define __tcu_clear_counter_to_zero(n)	(REG_TCU_TCSR((n)) |= TCU_TCSR_CNT_CLRZ)

#define __tcu_ost_enabled()		(REG_TCU_TER & TCU_TER_OSTEN)
#define __tcu_enable_ost()		(REG_TCU_TESR = TCU_TESR_OSTST)
#define __tcu_disable_ost()		(REG_TCU_TECR = TCU_TECR_OSTCL)

#define __tcu_counter_enabled(n)	(REG_TCU_TER & (1 << (n)))
#define __tcu_start_counter(n)		(REG_TCU_TESR |= (1 << (n)))
#define __tcu_stop_counter(n)		(REG_TCU_TECR |= (1 << (n)))

#define __tcu_half_match_flag(n)	(REG_TCU_TFR & (1 << ((n) + 16)))
#define __tcu_full_match_flag(n)	(REG_TCU_TFR & (1 << (n)))
#define __tcu_set_half_match_flag(n)	(REG_TCU_TFSR = (1 << ((n) + 16)))
#define __tcu_set_full_match_flag(n)	(REG_TCU_TFSR = (1 << (n)))
#define __tcu_clear_half_match_flag(n)	(REG_TCU_TFCR = (1 << ((n) + 16)))
#define __tcu_clear_full_match_flag(n)	(REG_TCU_TFCR = (1 << (n)))
#define __tcu_mask_half_match_irq(n)	(REG_TCU_TMSR = (1 << ((n) + 16)))
#define __tcu_mask_full_match_irq(n)	(REG_TCU_TMSR = (1 << (n)))
#define __tcu_unmask_half_match_irq(n)	(REG_TCU_TMCR = (1 << ((n) + 16)))
#define __tcu_unmask_full_match_irq(n)	(REG_TCU_TMCR = (1 << (n)))

#define __tcu_ost_match_flag()		(REG_TCU_TFR & TCU_TFR_OSTFLAG)
#define __tcu_set_ost_match_flag()	(REG_TCU_TFSR = TCU_TFSR_OSTFST)
#define __tcu_clear_ost_match_flag()	(REG_TCU_TFCR = TCU_TFCR_OSTFCL)
#define __tcu_ost_match_irq_masked()	(REG_TCU_TMR & TCU_TMR_OSTMASK)
#define __tcu_mask_ost_match_irq()	(REG_TCU_TMSR = TCU_TMSR_OSTMST)
#define __tcu_unmask_ost_match_irq()	(REG_TCU_TMCR = TCU_TMCR_OSTMCL)

#define __tcu_wdt_clock_stopped()	(REG_TCU_TSR & TCU_TSSR_WDTSC)
#define __tcu_ost_clock_stopped()	(REG_TCU_TSR & TCU_TSR_OST)
#define __tcu_timer_clock_stopped(n)	(REG_TCU_TSR & (1 << (n)))

#define __tcu_start_wdt_clock()		(REG_TCU_TSCR = TCU_TSSR_WDTSC)
#define __tcu_start_ost_clock()		(REG_TCU_TSCR = TCU_TSCR_OSTSC)
#define __tcu_start_timer_clock(n)	(REG_TCU_TSCR = (1 << (n)))

#define __tcu_stop_wdt_clock()		(REG_TCU_TSSR = TCU_TSSR_WDTSC)
#define __tcu_stop_ost_clock()		(REG_TCU_TSSR = TCU_TSSR_OSTSS)
#define __tcu_stop_timer_clock(n)	(REG_TCU_TSSR = (1 << (n)))

#define __tcu_get_count(n)		(REG_TCU_TCNT((n)))
#define __tcu_set_count(n,v)		(REG_TCU_TCNT((n)) = (v))
#define __tcu_set_full_data(n,v)	(REG_TCU_TDFR((n)) = (v))
#define __tcu_set_half_data(n,v)	(REG_TCU_TDHR((n)) = (v))

/* TCU2, counter 1, 2*/
#define __tcu_read_real_value(n)	(REG_TCU_TSTR & (1 << ((n) + 16)))
#define __tcu_read_false_value(n)	(REG_TCU_TSTR & (1 << ((n) + 16)))
#define __tcu_counter_busy(n)		(REG_TCU_TSTR & (1 << (n)))
#define __tcu_counter_ready(n)		(REG_TCU_TSTR & (1 << (n)))

#define __tcu_set_read_real_value(n)	(REG_TCU_TSTSR = (1 << ((n) + 16)))
#define __tcu_set_read_false_value(n)	(REG_TCU_TSTCR = (1 << ((n) + 16)))
#define __tcu_set_counter_busy(n)	(REG_TCU_TSTSR = (1 << (n)))
#define __tcu_set_counter_ready(n)	(REG_TCU_TSTCR = (1 << (n)))

/* ost counter */
#define __ostcu_set_pwm_output_shutdown_graceful()	(REG_TCU_OSTCSR &= ~TCU_TCSR_PWM_SD)
#define __ostcu_set_ost_output_shutdown_abrupt()	(REG_TCU_OSTCSR |= TCU_TCSR_PWM_SD)
#define __ostcu_select_clk_div1() \
	(REG_TCU_OSTCSR = (REG_TCU_OSTCSR & ~TCU_OSTCSR_PRESCALE_MASK) | TCU_OSTCSR_PRESCALE1)
#define __ostcu_select_clk_div4() \
	(REG_TCU_OSTCSR = (REG_TCU_OSTCSR & ~TCU_OSTCSR_PRESCALE_MASK) | TCU_OSTCSR_PRESCALE4)
#define __ostcu_select_clk_div16() \
	(REG_TCU_OSTCSR = (REG_TCU_OSTCSR & ~TCU_OSTCSR_PRESCALE_MASK) | TCU_OSTCSR_PRESCALE16)
#define __ostcu_select_clk_div64() \
	(REG_TCU_OSTCSR = (REG_TCU_OSTCSR & ~TCU_OSTCSR_PRESCALE_MASK) | TCU_OSTCSR_PRESCALE64)
#define __ostcu_select_clk_div256() \
	(REG_TCU_OSTCSR = (REG_TCU_OSTCSR & ~TCU_OSTCSR_PRESCALE_MASK) | TCU_OSTCSR_PRESCALE256)
#define __ostcu_select_clk_div1024() \
	(REG_TCU_OSTCSR = (REG_TCU_OSTCSR & ~TCU_OSTCSR_PRESCALE_MASK) | TCU_OSTCSR_PRESCALE1024)
#define __ostcu_select_rtcclk() \
	(REG_TCU_OSTCSR = (REG_TCU_OSTCSR & ~(TCU_OSTCSR_EXT_EN | TCU_OSTCSR_RTC_EN | TCU_OSTCSR_PCK_EN)) | TCU_OSTCSR_RTC_EN)
#define __ostcu_select_extalclk() \
	(REG_TCU_OSTCSR = (REG_TCU_OSTCSR & ~(TCU_OSTCSR_EXT_EN | TCU_OSTCSR_RTC_EN | TCU_OSTCSR_PCK_EN)) | TCU_OSTCSR_EXT_EN)
#define __ostcu_select_pclk() \
	(REG_TCU_OSTCSR = (REG_TCU_OSTCSR & ~(TCU_OSTCSR_EXT_EN | TCU_OSTCSR_RTC_EN | TCU_OSTCSR_PCK_EN)) | TCU_OSTCSR_PCK_EN)


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

/* m is the DMA controller index (0, 1), n is the DMA channel index (0 - 11) */

#define __dmac_enable_module(m) \
	( REG_DMAC_DMACR(m) |= DMAC_DMACR_DMAE | DMAC_DMACR_PR_012345 )
#define __dmac_disable_module(m) \
	( REG_DMAC_DMACR(m) &= ~DMAC_DMACR_DMAE )

/* p=0,1,2,3 */
#define __dmac_set_priority(m,p)			\
do {							\
	REG_DMAC_DMACR(m) &= ~DMAC_DMACR_PR_MASK;	\
	REG_DMAC_DMACR(m) |= ((p) << DMAC_DMACR_PR_BIT);	\
} while (0)

#define __dmac_test_halt_error(m) ( REG_DMAC_DMACR(m) & DMAC_DMACR_HLT )
#define __dmac_test_addr_error(m) ( REG_DMAC_DMACR(m) & DMAC_DMACR_AR )

#define __dmac_channel_enable_clk(n) \
	REG_DMAC_DMACKE((n)/HALF_DMA_NUM) |= 1 << ((n)-(n)/HALF_DMA_NUM*HALF_DMA_NUM);

#define __dmac_enable_descriptor(n) \
  ( REG_DMAC_DCCSR((n)) &= ~DMAC_DCCSR_NDES )
#define __dmac_disable_descriptor(n) \
  ( REG_DMAC_DCCSR((n)) |= DMAC_DCCSR_NDES )

#define __dmac_enable_channel(n)                 \
do {                                             \
	REG_DMAC_DCCSR((n)) |= DMAC_DCCSR_EN;    \
} while (0)
#define __dmac_disable_channel(n)                \
do {                                             \
	REG_DMAC_DCCSR((n)) &= ~DMAC_DCCSR_EN;   \
} while (0)
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

#define __dmac_channel_clear_transmit_halt(n)				\
	do {								\
		/* clear both channel halt error and globle halt error */ \
		REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_HLT;			\
		REG_DMAC_DMACR(n/HALF_DMA_NUM) &= ~DMAC_DMACR_HLT;	\
	} while (0)
#define __dmac_channel_clear_transmit_end(n) \
  (  REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_TT )
#define __dmac_channel_clear_address_error(n)				\
	do {								\
		REG_DMAC_DDA(n) = 0; /* clear descriptor address register */ \
		REG_DMAC_DSAR(n) = 0; /* clear source address register */ \
		REG_DMAC_DTAR(n) = 0; /* clear target address register */ \
		/* clear both channel addr error and globle address error */ \
		REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_AR;			\
		REG_DMAC_DMACR(n/HALF_DMA_NUM) &= ~DMAC_DMACR_AR;	\
	} while (0)
#define __dmac_channel_clear_count_terminated(n) \
  (  REG_DMAC_DCCSR((n)) &= ~DMAC_DCCSR_CT )
#define __dmac_channel_clear_descriptor_invalid(n) \
  (  REG_DMAC_DCCSR((n)) &= ~DMAC_DCCSR_INV )

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

#define __dmac_channel_set_doorbell(n)	\
	(  REG_DMAC_DMADBSR((n)/HALF_DMA_NUM) = (1 << ((n)-(n)/HALF_DMA_NUM*HALF_DMA_NUM)) )

#define __dmac_channel_irq_detected(n)  ( REG_DMAC_DMAIPR((n)/HALF_DMA_NUM) & (1 << ((n)-(n)/HALF_DMA_NUM*HALF_DMA_NUM)) )
#define __dmac_channel_ack_irq(n)       ( REG_DMAC_DMAIPR((n)/HALF_DMA_NUM) &= ~(1 <<((n)-(n)/HALF_DMA_NUM*HALF_DMA_NUM)) )

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

/*************************************************************************
 * PCM Controller operation
 *************************************************************************/

#define __pcm_enable()          ( REG_PCM_CTL |= PCM_CTL_PCMEN )
#define __pcm_disable()         ( REG_PCM_CTL &= ~PCM_CTL_PCMEN )

#define __pcm_clk_enable()      ( REG_PCM_CTL |= PCM_CTL_CLKEN )
#define __pcm_clk_disable()     ( REG_PCM_CTL &= ~PCM_CTL_CLKEN )

#define __pcm_reset()           ( REG_PCM_CTL |= PCM_CTL_RST )
#define __pcm_flush_fifo()	( REG_PCM_CTL |= PCM_CTL_FLUSH )

#define __pcm_enable_record()		( REG_PCM_CTL |= PCM_CTL_EREC )
#define __pcm_disable_record()		( REG_PCM_CTL &= ~PCM_CTL_EREC )
#define __pcm_enable_playback()		( REG_PCM_CTL |= PCM_CTL_ERPL )
#define __pcm_disable_playback()	( REG_PCM_CTL &= ~PCM_CTL_ERPL )

#define __pcm_enable_rxfifo()           __pcm_enable_record()
#define __pcm_disable_rxfifo()          __pcm_disable_record()
#define __pcm_enable_txfifo()           __pcm_enable_playback()
#define __pcm_disable_txfifo()          __pcm_disable_playback()

#define __pcm_last_sample()     ( REG_PCM_CTL |= PCM_CTL_LSMP )
#define __pcm_zero_sample()     ( REG_PCM_CTL &= ~PCM_CTL_LSMP )

#define __pcm_enable_transmit_dma()    ( REG_PCM_CTL |= PCM_CTL_ETDMA )
#define __pcm_disable_transmit_dma()   ( REG_PCM_CTL &= ~PCM_CTL_ETDMA )
#define __pcm_enable_receive_dma()     ( REG_PCM_CTL |= PCM_CTL_ERDMA )
#define __pcm_disable_receive_dma()    ( REG_PCM_CTL &= ~PCM_CTL_ERDMA )

#define __pcm_as_master()     ( REG_PCM_CFG &= PCM_CFG_MODE )
#define __pcm_as_slave()      ( REG_PCM_CFG |= ~PCM_CFG_MODE )

#define __pcm_set_transmit_trigger(n) 			\
do {							\
	REG_PCM_CFG &= ~PCM_CFG_TFTH_MASK;		\
	REG_PCM_CFG |= ((n) << PCM_CFG_TFTH_BIT);	\
} while(0)

#define __pcm_set_receive_trigger(n) 			\
do {							\
	REG_PCM_CFG &= ~PCM_CFG_RFTH_MASK;		\
	REG_PCM_CFG |= ((n) << PCM_CFG_RFTH_BIT);	\
} while(0)

#define __pcm_omsb_same_sync()   ( REG_PCM_CFG &= ~PCM_CFG_OMSBPOS )
#define __pcm_omsb_next_sync()   ( REG_PCM_CFG |= PCM_CFG_OMSBPOS )

#define __pcm_imsb_same_sync()   ( REG_PCM_CFG &= ~PCM_CFG_IMSBPOS )
#define __pcm_imsb_next_sync()   ( REG_PCM_CFG |= PCM_CFG_IMSBPOS )

/* set input sample size 8 or 16*/
#define __pcm_set_iss(n) \
( REG_PCM_CFG = (REG_PCM_CFG & ~PCM_CFG_ISS_MASK) | PCM_CFG_ISS_##n )
/* set output sample size 8 or 16*/
#define __pcm_set_oss(n) \
( REG_PCM_CFG = (REG_PCM_CFG & ~PCM_CFG_OSS_MASK) | PCM_CFG_OSS_##n )

#define __pcm_set_valid_slot(n) \
( REG_PCM_CFG = (REG_PCM_CFG & ~PCM_CFG_SLOT_MASK) | PCM_CFG_SLOT_##n )

#define __pcm_write_data(v)           ( REG_PCM_DP = (v) )
#define __pcm_read_data()             ( REG_PCM_DP )

#define __pcm_enable_tfs_intr()       ( REG_PCM_INTC |= PCM_INTC_ETFS )
#define __pcm_disable_tfs_intr()      ( REG_PCM_INTC &= ~PCM_INTC_ETFS )

#define __pcm_enable_tur_intr()       ( REG_PCM_INTC |= PCM_INTC_ETUR )
#define __pcm_disable_tur_intr()      ( REG_PCM_INTC &= ~PCM_INTC_ETUR )

#define __pcm_enable_rfs_intr()       ( REG_PCM_INTC |= PCM_INTC_ERFS )
#define __pcm_disable_rfs_intr()      ( REG_PCM_INTC &= ~PCM_INTC_ERFS )

#define __pcm_enable_ror_intr()       ( REG_PCM_INTC |= PCM_INTC_EROR )
#define __pcm_disable_ror_intr()      ( REG_PCM_INTC &= ~PCM_INTC_EROR )

#define __pcm_ints_valid_tx() \
( ((REG_PCM_INTS & PCM_INTS_TFL_MASK) >> PCM_INTS_TFL_BIT) )
#define __pcm_ints_valid_rx() \
( ((REG_PCM_INTS & PCM_INTS_RFL_MASK) >> PCM_INTS_RFL_BIT) )

#define __pcm_set_clk_div(n) \
( REG_PCM_DIV = (REG_PCM_DIV & ~PCM_DIV_CLKDIV_MASK) | ((n) << PCM_DIV_CLKDIV_BIT) )

/* sysclk(cpm_pcm_sysclk) Hz is created by cpm logic, and pcmclk Hz is the pcm in/out clock wanted */
#define __pcm_set_clk_rate(sysclk, pcmclk) \
__pcm_set_clk_div(((sysclk) / (pcmclk) - 1))

#define __pcm_set_sync_div(n) \
( REG_PCM_DIV = (REG_PCM_DIV & ~PCM_DIV_SYNDIV_MASK) | ((n) << PCM_DIV_SYNDIV_BIT) )

/* pcmclk is source clock Hz, and sync is the frame sync clock Hz wanted */
#define __pcm_set_sync_rate(pcmclk, sync) \
__pcm_set_sync_div(((pcmclk) / (8 * (sync)) - 1))

 /* set sync length in pcmclk n = 0 ... 63 */
#define __pcm_set_sync_len(n) \
( REG_PCM_DIV = (REG_PCM_DIV & ~PCM_DIV_SYNL_MASK) | (n << PCM_DIV_SYNL_BIT) )


/***************************************************************************
 * ICDC
 ***************************************************************************/
#define __i2s_internal_codec()         __aic_internal_codec()
#define __i2s_external_codec()         __aic_external_codec()

#define __icdc_clk_ready()             ( REG_ICDC_CKCFG & ICDC_CKCFG_CKRDY )
#define __icdc_sel_adc()               ( REG_ICDC_CKCFG |= ICDC_CKCFG_SELAD )
#define __icdc_sel_dac()               ( REG_ICDC_CKCFG &= ~ICDC_CKCFG_SELAD )

#define __icdc_set_rgwr()              ( REG_ICDC_RGADW |= ICDC_RGADW_RGWR )
#define __icdc_clear_rgwr()            ( REG_ICDC_RGADW &= ~ICDC_RGADW_RGWR )
#define __icdc_rgwr_ready()            ( REG_ICDC_RGADW & ICDC_RGADW_RGWR )

#define __icdc_set_addr(n)				\
do {          						\
	REG_ICDC_RGADW &= ~ICDC_RGADW_RGADDR_MASK;	\
	REG_ICDC_RGADW |= (n) << ICDC_RGADW_RGADDR_BIT;	\
} while(0)

#define __icdc_set_cmd(n)				\
do {          						\
	REG_ICDC_RGADW &= ~ICDC_RGADW_RGDIN_MASK;	\
	REG_ICDC_RGADW |= (n) << ICDC_RGADW_RGDIN_BIT;	\
} while(0)

#define __icdc_irq_pending()            ( REG_ICDC_RGDATA & ICDC_RGDATA_IRQ )
#define __icdc_get_value()              ( REG_ICDC_RGDATA & ICDC_RGDATA_RGDOUT_MASK )

/***************************************************************************
 * INTC
 ***************************************************************************/
#define __intc_unmask_irq(n)	( REG_INTC_IMCR = (1 << (n)) )
#define __intc_mask_irq(n)	( REG_INTC_IMSR = (1 << (n)) )
#define __intc_ack_irq(n)	( REG_INTC_IPR = (1 << (n)) ) /* A dummy ack, as the Pending Register is Read Only. Should we remove __intc_ack_irq() */


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
#define __i2c_ack_is_nack()	(REG_I2C_SR & I2C_SR_ACKF)
#define __i2c_ack_is_ack()	(!(REG_I2C_SR & I2C_SR_ACKF))

#define __i2c_is_busy()		( REG_I2C_SR & I2C_SR_BUSY )
#define __i2c_transmit_ended()	( REG_I2C_SR & I2C_SR_TEND )

#define __i2c_set_clk(dev_clk, i2c_clk) \
  ( REG_I2C_GR = (dev_clk) / (16*(i2c_clk)) - 1 )

#define __i2c_read()		( REG_I2C_DR )
#define __i2c_write(val)	( REG_I2C_DR = (val) )


/***************************************************************************
 * MSC
 ***************************************************************************/
/* n = 0, 1 (MSC0, MSC1) */

#define __msc_start_op(n) \
	( REG_MSC_STRPCL(n) = MSC_STRPCL_START_OP | MSC_STRPCL_CLOCK_CONTROL_START )

#define __msc_set_resto(n, to)  	( REG_MSC_RESTO(n) = to )
#define __msc_set_rdto(n, to)   	( REG_MSC_RDTO(n) = to )
#define __msc_set_cmd(n, cmd)   	( REG_MSC_CMD(n) = cmd )
#define __msc_set_arg(n, arg)   	( REG_MSC_ARG(n) = arg )
#define __msc_set_nob(n, nob)      	( REG_MSC_NOB(n) = nob )
#define __msc_get_nob(n)        	( REG_MSC_NOB(n) )
#define __msc_set_blklen(n, len)        ( REG_MSC_BLKLEN(n) = len )
#define __msc_set_cmdat(n, cmdat)   	( REG_MSC_CMDAT(n) = cmdat )
#define __msc_set_cmdat_ioabort(n) 	( REG_MSC_CMDAT(n) |= MSC_CMDAT_IO_ABORT )
#define __msc_clear_cmdat_ioabort(n) 	( REG_MSC_CMDAT(n) &= ~MSC_CMDAT_IO_ABORT )

#define __msc_set_cmdat_bus_width1(n) 			\
do { 							\
	REG_MSC_CMDAT(n) &= ~MSC_CMDAT_BUS_WIDTH_MASK; 	\
	REG_MSC_CMDAT(n) |= MSC_CMDAT_BUS_WIDTH_1BIT; 	\
} while(0)

#define __msc_set_cmdat_bus_width4(n) 			\
do { 							\
	REG_MSC_CMDAT(n) &= ~MSC_CMDAT_BUS_WIDTH_MASK; 	\
	REG_MSC_CMDAT(n) |= MSC_CMDAT_BUS_WIDTH_4BIT; 	\
} while(0)

#define __msc_set_cmdat_dma_en(n)       ( REG_MSC_CMDAT(n) |= MSC_CMDAT_DMA_EN )
#define __msc_set_cmdat_init(n) 	( REG_MSC_CMDAT(n) |= MSC_CMDAT_INIT )
#define __msc_set_cmdat_busy(n) 	( REG_MSC_CMDAT(n) |= MSC_CMDAT_BUSY )
#define __msc_set_cmdat_stream(n)       ( REG_MSC_CMDAT(n) |= MSC_CMDAT_STREAM_BLOCK )
#define __msc_set_cmdat_block(n)        ( REG_MSC_CMDAT(n) &= ~MSC_CMDAT_STREAM_BLOCK )
#define __msc_set_cmdat_read(n) 	( REG_MSC_CMDAT(n) &= ~MSC_CMDAT_WRITE_READ )
#define __msc_set_cmdat_write(n)        ( REG_MSC_CMDAT(n) |= MSC_CMDAT_WRITE_READ )
#define __msc_set_cmdat_data_en(n)      ( REG_MSC_CMDAT(n) |= MSC_CMDAT_DATA_EN )

/* r is MSC_CMDAT_RESPONSE_FORMAT_Rx or MSC_CMDAT_RESPONSE_FORMAT_NONE */
#define __msc_set_cmdat_res_format(n, r)				\
do { 								\
	REG_MSC_CMDAT(n) &= ~MSC_CMDAT_RESPONSE_FORMAT_MASK; 	\
	REG_MSC_CMDAT(n) |= (r); 					\
} while(0)

#define __msc_clear_cmdat(n) \
  REG_MSC_CMDAT(n) &= ~( MSC_CMDAT_IO_ABORT | MSC_CMDAT_DMA_EN | MSC_CMDAT_INIT| \
  MSC_CMDAT_BUSY | MSC_CMDAT_STREAM_BLOCK | MSC_CMDAT_WRITE_READ | \
  MSC_CMDAT_DATA_EN | MSC_CMDAT_RESPONSE_FORMAT_MASK )

#define __msc_get_imask(n) 		( REG_MSC_IMASK(n) )
#define __msc_mask_all_intrs(n) 	( REG_MSC_IMASK(n) = 0xff )
#define __msc_unmask_all_intrs(n) 	( REG_MSC_IMASK(n) = 0x00 )
#define __msc_mask_rd(n) 		( REG_MSC_IMASK(n) |= MSC_IMASK_RXFIFO_RD_REQ )
#define __msc_unmask_rd(n) 		( REG_MSC_IMASK(n) &= ~MSC_IMASK_RXFIFO_RD_REQ )
#define __msc_mask_wr(n) 		( REG_MSC_IMASK(n) |= MSC_IMASK_TXFIFO_WR_REQ )
#define __msc_unmask_wr(n) 		( REG_MSC_IMASK(n) &= ~MSC_IMASK_TXFIFO_WR_REQ )
#define __msc_mask_endcmdres(n) 	( REG_MSC_IMASK(n) |= MSC_IMASK_END_CMD_RES )
#define __msc_unmask_endcmdres(n) 	( REG_MSC_IMASK(n) &= ~MSC_IMASK_END_CMD_RES )
#define __msc_mask_datatrandone(n) 	( REG_MSC_IMASK(n) |= MSC_IMASK_DATA_TRAN_DONE )
#define __msc_unmask_datatrandone(n) 	( REG_MSC_IMASK(n) &= ~MSC_IMASK_DATA_TRAN_DONE )
#define __msc_mask_prgdone(n) 		( REG_MSC_IMASK(n) |= MSC_IMASK_PRG_DONE )
#define __msc_unmask_prgdone(n) 	( REG_MSC_IMASK(n) &= ~MSC_IMASK_PRG_DONE )

/* m=0,1,2,3,4,5,6,7 */
#define __msc_set_clkrt(n, m) 	\
do { 				\
	REG_MSC_CLKRT(n) = m;	\
} while(0)

#define __msc_get_ireg(n) 	        	( REG_MSC_IREG(n) )
#define __msc_ireg_rd(n) 	        	( REG_MSC_IREG(n) & MSC_IREG_RXFIFO_RD_REQ )
#define __msc_ireg_wr(n) 	        	( REG_MSC_IREG(n) & MSC_IREG_TXFIFO_WR_REQ )
#define __msc_ireg_end_cmd_res(n)       	( REG_MSC_IREG(n) & MSC_IREG_END_CMD_RES )
#define __msc_ireg_data_tran_done(n)     	( REG_MSC_IREG(n) & MSC_IREG_DATA_TRAN_DONE )
#define __msc_ireg_prg_done(n) 	        	( REG_MSC_IREG(n) & MSC_IREG_PRG_DONE )
#define __msc_ireg_clear_end_cmd_res(n)         ( REG_MSC_IREG(n) = MSC_IREG_END_CMD_RES )
#define __msc_ireg_clear_data_tran_done(n)      ( REG_MSC_IREG(n) = MSC_IREG_DATA_TRAN_DONE )
#define __msc_ireg_clear_prg_done(n)     	( REG_MSC_IREG(n) = MSC_IREG_PRG_DONE )

#define __msc_get_stat(n) 		( REG_MSC_STAT(n) )
#define __msc_stat_not_end_cmd_res(n) 	( (REG_MSC_STAT(n) & MSC_STAT_END_CMD_RES) == 0)
#define __msc_stat_crc_err(n) \
  ( REG_MSC_STAT(n) & (MSC_STAT_CRC_RES_ERR | MSC_STAT_CRC_READ_ERROR | MSC_STAT_CRC_WRITE_ERROR_YES) )
#define __msc_stat_res_crc_err(n) 	( REG_MSC_STAT(n) & MSC_STAT_CRC_RES_ERR )
#define __msc_stat_rd_crc_err(n) 	( REG_MSC_STAT(n) & MSC_STAT_CRC_READ_ERROR )
#define __msc_stat_wr_crc_err(n) 	( REG_MSC_STAT(n) & MSC_STAT_CRC_WRITE_ERROR_YES )
#define __msc_stat_resto_err(n) 	( REG_MSC_STAT(n) & MSC_STAT_TIME_OUT_RES )
#define __msc_stat_rdto_err(n) 		( REG_MSC_STAT(n) & MSC_STAT_TIME_OUT_READ )

#define __msc_rd_resfifo(n) 		( REG_MSC_RES(n) )
#define __msc_rd_rxfifo(n)  		( REG_MSC_RXFIFO(n) )
#define __msc_wr_txfifo(n, v)  		( REG_MSC_TXFIFO(n) = v )

#define __msc_reset(n) 						\
do { 								\
	REG_MSC_STRPCL(n) = MSC_STRPCL_RESET;			\
 	while (REG_MSC_STAT(n) & MSC_STAT_IS_RESETTING);		\
} while (0)

#define __msc_start_clk(n) 					\
do { 								\
	REG_MSC_STRPCL(n) = MSC_STRPCL_CLOCK_CONTROL_START;	\
} while (0)

#define __msc_stop_clk(n) 					\
do { 								\
	REG_MSC_STRPCL(n) = MSC_STRPCL_CLOCK_CONTROL_STOP;	\
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
 * SSI (Synchronous Serial Interface)
 ***************************************************************************/
/* n = 0, 1 (SSI0, SSI1) */
#define __ssi_enable(n) 	( REG_SSI_CR0(n) |= SSI_CR0_SSIE )
#define __ssi_disable(n) 	( REG_SSI_CR0(n) &= ~SSI_CR0_SSIE )
#define __ssi_select_ce(n) 	( REG_SSI_CR0(n) &= ~SSI_CR0_FSEL )

#define __ssi_normal_mode(n) ( REG_SSI_ITR(n) &= ~SSI_ITR_IVLTM_MASK )

#define __ssi_select_ce2(n) 		\
do { 					\
	REG_SSI_CR0(n) |= SSI_CR0_FSEL; 	\
	REG_SSI_CR1(n) &= ~SSI_CR1_MULTS;	\
} while (0)

#define __ssi_select_gpc(n) 			\
do { 						\
	REG_SSI_CR0(n) &= ~SSI_CR0_FSEL;	\
	REG_SSI_CR1(n) |= SSI_CR1_MULTS;	\
} while (0)

#define __ssi_underrun_auto_clear(n) 		\
do { 						\
	REG_SSI_CR0(n) |= SSI_CR0_EACLRUN; 	\
} while (0)

#define __ssi_underrun_clear_manually(n) 	\
do { 						\
	REG_SSI_CR0(n) &= ~SSI_CR0_EACLRUN; 	\
} while (0)

#define __ssi_enable_tx_intr(n)					\
	( REG_SSI_CR0(n) |= SSI_CR0_TIE | SSI_CR0_TEIE )

#define __ssi_disable_tx_intr(n)				\
	( REG_SSI_CR0(n) &= ~(SSI_CR0_TIE | SSI_CR0_TEIE) )

#define __ssi_enable_rx_intr(n)					\
	( REG_SSI_CR0(n) |= SSI_CR0_RIE | SSI_CR0_REIE )

#define __ssi_disable_rx_intr(n)				\
	( REG_SSI_CR0(n) &= ~(SSI_CR0_RIE | SSI_CR0_REIE) )

#define __ssi_enable_txfifo_half_empty_intr(n)  \
	( REG_SSI_CR0(n) |= SSI_CR0_TIE )
#define __ssi_disable_txfifo_half_empty_intr(n)	\
	( REG_SSI_CR0(n) &= ~SSI_CR0_TIE )
#define __ssi_enable_tx_error_intr(n)		\
	( REG_SSI_CR0(n) |= SSI_CR0_TEIE )
#define __ssi_disable_tx_error_intr(n)		\
	( REG_SSI_CR0(n) &= ~SSI_CR0_TEIE )
#define __ssi_enable_rxfifo_half_full_intr(n)	\
	( REG_SSI_CR0(n) |= SSI_CR0_RIE )
#define __ssi_disable_rxfifo_half_full_intr(n)  \
	( REG_SSI_CR0(n) &= ~SSI_CR0_RIE )
#define __ssi_enable_rx_error_intr(n)		\
	( REG_SSI_CR0(n) |= SSI_CR0_REIE )
#define __ssi_disable_rx_error_intr(n)		\
	( REG_SSI_CR0(n) &= ~SSI_CR0_REIE )

#define __ssi_enable_loopback(n)  ( REG_SSI_CR0(n) |= SSI_CR0_LOOP )
#define __ssi_disable_loopback(n) ( REG_SSI_CR0(n) &= ~SSI_CR0_LOOP )

#define __ssi_enable_receive(n)   ( REG_SSI_CR0(n) &= ~SSI_CR0_DISREV )
#define __ssi_disable_receive(n)  ( REG_SSI_CR0(n) |= SSI_CR0_DISREV )

#define __ssi_finish_receive(n)					\
	( REG_SSI_CR0(n) |= (SSI_CR0_RFINE | SSI_CR0_RFINC) )

#define __ssi_disable_recvfinish(n)				\
	( REG_SSI_CR0(n) &= ~(SSI_CR0_RFINE | SSI_CR0_RFINC) )

#define __ssi_flush_txfifo(n)   	( REG_SSI_CR0(n) |= SSI_CR0_TFLUSH )
#define __ssi_flush_rxfifo(n)   	( REG_SSI_CR0(n) |= SSI_CR0_RFLUSH )

#define __ssi_flush_fifo(n)					\
	( REG_SSI_CR0(n) |= SSI_CR0_TFLUSH | SSI_CR0_RFLUSH )

#define __ssi_finish_transmit(n) 	( REG_SSI_CR1(n) &= ~SSI_CR1_UNFIN )
#define __ssi_wait_transmit(n) 		( REG_SSI_CR1(n) |= SSI_CR1_UNFIN )
#define __ssi_use_busy_wait_mode(n) 	__ssi_wait_transmit(n)
#define __ssi_unset_busy_wait_mode(n) 	__ssi_finish_transmit(n)

#define __ssi_spi_format(n)						\
	do {								\
		REG_SSI_CR1(n) &= ~SSI_CR1_FMAT_MASK; 			\
		REG_SSI_CR1(n) |= SSI_CR1_FMAT_SPI;			\
		REG_SSI_CR1(n) &= ~(SSI_CR1_TFVCK_MASK|SSI_CR1_TCKFI_MASK); \
		REG_SSI_CR1(n) |= (SSI_CR1_TFVCK_1 | SSI_CR1_TCKFI_1);	\
	} while (0)

/* TI's SSP format, must clear SSI_CR1.UNFIN */
#define __ssi_ssp_format(n)						\
	do { 								\
		REG_SSI_CR1(n) &= ~(SSI_CR1_FMAT_MASK | SSI_CR1_UNFIN);	\
		REG_SSI_CR1(n) |= SSI_CR1_FMAT_SSP;			\
	} while (0)

/* National's Microwire format, must clear SSI_CR0.RFINE, and set max delay */
#define __ssi_microwire_format(n)					\
	do {								\
		REG_SSI_CR1(n) &= ~SSI_CR1_FMAT_MASK; 			\
		REG_SSI_CR1(n) |= SSI_CR1_FMAT_MW1;			\
		REG_SSI_CR1(n) &= ~(SSI_CR1_TFVCK_MASK|SSI_CR1_TCKFI_MASK); \
		REG_SSI_CR1(n) |= (SSI_CR1_TFVCK_3 | SSI_CR1_TCKFI_3);	\
		REG_SSI_CR0(n) &= ~SSI_CR0_RFINE;			\
	} while (0)

/* CE# level (FRMHL), CE# in interval time (ITFRM),
   clock phase and polarity (PHA POL),
   interval time (SSIITR), interval characters/frame (SSIICR) */

/* frmhl,endian,mcom,flen,pha,pol MASK */
#define SSICR1_MISC_MASK 					\
	( SSI_CR1_FRMHL_MASK | SSI_CR1_LFST | SSI_CR1_MCOM_MASK	\
	  | SSI_CR1_FLEN_MASK | SSI_CR1_PHA | SSI_CR1_POL )

#define __ssi_spi_set_misc(n,frmhl,endian,flen,mcom,pha,pol)		\
	do {								\
		REG_SSI_CR1(n) &= ~SSICR1_MISC_MASK;			\
		REG_SSI_CR1(n) |= ((frmhl) << 30) | ((endian) << 25) | 	\
			(((mcom) - 1) << 12) | (((flen) - 2) << 4) | 	\
			((pha) << 1) | (pol); 				\
	} while(0)

/* Transfer with MSB or LSB first */
#define __ssi_set_msb(n) ( REG_SSI_CR1(n) &= ~SSI_CR1_LFST )
#define __ssi_set_lsb(n) ( REG_SSI_CR1(n) |= SSI_CR1_LFST )

#define __ssi_set_frame_length(n, m)					\
	REG_SSI_CR1(n) = (REG_SSI_CR1(n) & ~SSI_CR1_FLEN_MASK) | (((m) - 2) << 4)

/* m = 1 - 16 */
#define __ssi_set_microwire_command_length(n,m)				\
	( REG_SSI_CR1(n) = ((REG_SSI_CR1(n) & ~SSI_CR1_MCOM_MASK) | SSI_CR1_MCOM_##m##BIT) )

/* Set the clock phase for SPI */
#define __ssi_set_spi_clock_phase(n, m)					\
	( REG_SSI_CR1(n) = ((REG_SSI_CR1(n) & ~SSI_CR1_PHA) | (((m)&0x1)<< 1)))

/* Set the clock polarity for SPI */
#define __ssi_set_spi_clock_polarity(n, p)				\
	( REG_SSI_CR1(n) = ((REG_SSI_CR1(n) & ~SSI_CR1_POL) | ((p)&0x1)) )

/* SSI tx trigger, m = i x 8 */
#define __ssi_set_tx_trigger(n, m)				\
	do {							\
		REG_SSI_CR1(n) &= ~SSI_CR1_TTRG_MASK;		\
		REG_SSI_CR1(n) |= ((m)/8)<<SSI_CR1_TTRG_BIT;	\
	} while (0)

/* SSI rx trigger, m = i x 8 */
#define __ssi_set_rx_trigger(n, m)				\
	do {							\
		REG_SSI_CR1(n) &= ~SSI_CR1_RTRG_MASK;		\
		REG_SSI_CR1(n) |= ((m)/8)<<SSI_CR1_RTRG_BIT;	\
	} while (0)

#define __ssi_get_txfifo_count(n)					\
	( (REG_SSI_SR(n) & SSI_SR_TFIFONUM_MASK) >> SSI_SR_TFIFONUM_BIT )

#define __ssi_get_rxfifo_count(n)					\
	( (REG_SSI_SR(n) & SSI_SR_RFIFONUM_MASK) >> SSI_SR_RFIFONUM_BIT )

#define __ssi_transfer_end(n)		( REG_SSI_SR(n) & SSI_SR_END )
#define __ssi_is_busy(n)		( REG_SSI_SR(n) & SSI_SR_BUSY )

#define __ssi_txfifo_full(n)		( REG_SSI_SR(n) & SSI_SR_TFF )
#define __ssi_rxfifo_empty(n)		( REG_SSI_SR(n) & SSI_SR_RFE )
#define __ssi_rxfifo_half_full(n)	( REG_SSI_SR(n) & SSI_SR_RFHF )
#define __ssi_txfifo_half_empty(n)	( REG_SSI_SR(n) & SSI_SR_TFHE )
#define __ssi_underrun(n)		( REG_SSI_SR(n) & SSI_SR_UNDR )
#define __ssi_overrun(n)		( REG_SSI_SR(n) & SSI_SR_OVER )
#define __ssi_clear_underrun(n)		( REG_SSI_SR(n) = ~SSI_SR_UNDR )
#define __ssi_clear_overrun(n)		( REG_SSI_SR(n) = ~SSI_SR_OVER )
#define __ssi_clear_errors(n)		( REG_SSI_SR(n) &= ~(SSI_SR_UNDR | SSI_SR_OVER) )

#define __ssi_set_clk(n, dev_clk, ssi_clk)			\
	( REG_SSI_GR(n) = (dev_clk) / (2*(ssi_clk)) - 1 )

#define __ssi_receive_data(n) 		REG_SSI_DR(n)
#define __ssi_transmit_data(n, v) 	(REG_SSI_DR(n) = (v))

#define __ssi_set_grdiv(n,v)			(REG_SSI_GR(n) = v)
#define __ssi_get_grdiv(n)				(REG_SSI_GR(n))

#define __ssi_txfifo_half_empty_intr(n)  \
	( REG_SSI_CR0(n) & SSI_CR0_TIE )
#define __ssi_rxfifo_half_full_intr(n)	\
	( REG_SSI_CR0(n) & SSI_CR0_RIE )
	
#define __ssi_tx_error_intr(n)		\
	( REG_SSI_CR0(n) & SSI_CR0_TEIE )
#define __ssi_rx_error_intr(n)		\
	( REG_SSI_CR0(n) & SSI_CR0_REIE )	
	
/***************************************************************************
 * CIM
 ***************************************************************************/

#define __cim_enable()	( REG_CIM_CTRL |= CIM_CTRL_ENA )
#define __cim_disable()	( REG_CIM_CTRL &= ~CIM_CTRL_ENA )

/* n = 0, 1, 2, 3 */
#define __cim_set_input_data_stream_order(n)				\
	do {								\
		REG_CIM_CFG &= CIM_CFG_ORDER_MASK;			\
		REG_CIM_CFG |= ((n)<<CIM_CFG_ORDER_BIT)&CIM_CFG_ORDER_MASK; \
	} while (0)

#define __cim_input_data_format_select_RGB()	\
	do {					\
		REG_CIM_CFG &= CIM_CFG_DF_MASK;	\
		REG_CIM_CFG |= CIM_CFG_DF_RGB;	\
	} while (0)

#define __cim_input_data_format_select_YUV444()		\
	do {						\
		REG_CIM_CFG &= CIM_CFG_DF_MASK;		\
		REG_CIM_CFG |= CIM_CFG_DF_YUV444;	\
	} while (0)

#define __cim_input_data_format_select_YUV422()		\
	do {						\
		REG_CIM_CFG &= CIM_CFG_DF_MASK;		\
		REG_CIM_CFG |= CIM_CFG_DF_YUV422;	\
	} while (0)

#define __cim_input_data_format_select_ITU656()		\
	do {						\
		REG_CIM_CFG &= CIM_CFG_DF_MASK;		\
		REG_CIM_CFG |= CIM_CFG_DF_ITU656;	\
	} while (0)

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
	REG_CIM_CFG &= ~CIM_CFG_PACK_MASK;	\
	REG_CIM_CFG |= (CIM_CFG_PACK_##n);	\
} while (0)

#define __cim_enable_bypass_func() 	(REG_CIM_CFG |= CIM_CFG_BYPASS)
#define __cim_disable_bypass_func() 	(REG_CIM_CFG &= ~CIM_CFG_BYPASS_MASK)

#define __cim_enable_ccir656_progressive_mode()	\
do {						\
	REG_CIM_CFG &= ~CIM_CFG_DSM_MASK;	\
	REG_CIM_CFG |= CIM_CFG_DSM_CPM;		\
} while (0)

#define __cim_enable_ccir656_interlace_mode()	\
do {						\
	REG_CIM_CFG &= ~CIM_CFG_DSM_MASK;	\
	REG_CIM_CFG |= CIM_CFG_DSM_CIM;		\
} while (0)

#define __cim_enable_gated_clock_mode()		\
do {						\
	REG_CIM_CFG &= ~CIM_CFG_DSM_MASK;	\
	REG_CIM_CFG |= CIM_CFG_DSM_GCM;		\
} while (0)

#define __cim_enable_nongated_clock_mode()	\
do {						\
	REG_CIM_CFG &= ~CIM_CFG_DSM_MASK;	\
	REG_CIM_CFG |= CIM_CFG_DSM_NGCM;	\
} while (0)

/* sclk:system bus clock
 * mclk: CIM master clock
 */
#define __cim_set_master_clk(sclk, mclk)			\
do {								\
	REG_CIM_CTRL &= ~CIM_CTRL_MCLKDIV_MASK;				\
	REG_CIM_CTRL |= (((sclk)/(mclk) - 1) << CIM_CTRL_MCLKDIV_BIT);	\
} while (0)
/* n=1-16 */
#define __cim_set_frame_rate(n) 		\
do {						\
	REG_CIM_CTRL &= ~CIM_CTRL_FRC_MASK; 	\
	REG_CIM_CTRL |= CIM_CTRL_FRC_##n;	\
} while (0)

#define __cim_enable_size_func() \
	( REG_CIM_CTRL |= CIM_CTRL_SIZEEN )
#define __cim_disable_size_func() \
	( REG_CIM_CTRL &= ~CIM_CTRL_SIZEEN_MASK )

#define __cim_enable_vdd_intr() \
	( REG_CIM_CTRL |= CIM_CTRL_VDDM )
#define __cim_disable_vdd_intr() \
	( REG_CIM_CTRL &= ~CIM_CTRL_VDDM )

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

#define __cim_enable_rxfifo_overflow_intr()	\
	( REG_CIM_CTRL |= CIM_CTRL_RXF_OFM )
#define __cim_disable_rxfifo_overflow_intr()	\
	( REG_CIM_CTRL &= ~CIM_CTRL_RXF_OFM )

/* n=4,8,12,16,20,24,28,32 */
#define __cim_set_rxfifo_trigger(n) 		\
do {						\
	REG_CIM_CTRL &= ~CIM_CTRL_RXF_TRIG_MASK; 	\
	REG_CIM_CTRL |= CIM_CTRL_RXF_TRIG_##n;		\
} while (0)
#define __cim_enable_fast_mode() 	( REG_CIM_CTRL |= CIM_CTRL_FAST_MODE )
#define __cim_disable_fast_mode() 	( REG_CIM_CTRL &= ~CIM_CTRL_FAST_MODE )
#define __cim_use_normal_mode() 	__cim_disable_fast_mode()
#define __cim_enable_dma()   ( REG_CIM_CTRL |= CIM_CTRL_DMA_EN )
#define __cim_disable_dma()  ( REG_CIM_CTRL &= ~CIM_CTRL_DMA_EN )
#define __cim_reset_rxfifo() ( REG_CIM_CTRL |= CIM_CTRL_RXF_RST )
#define __cim_unreset_rxfifo() ( REG_CIM_CTRL &= ~CIM_CTRL_RXF_RST )

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
#define __cim_get_fid()   	     ( REG_CIM_FID )
#define __cim_get_image_data()       ( REG_CIM_RXFIFO )
#define __cim_get_dma_cmd()          ( REG_CIM_CMD )

#define __cim_set_da(a)              ( REG_CIM_DA = (a) )

#define __cim_set_line(a) 	( REG_CIM_SIZE = (REG_CIM_SIZE&(~CIM_SIZE_LPF_MASK))|((a)<<CIM_SIZE_LPF_BIT) )
#define __cim_set_pixel(a) 	( REG_CIM_SIZE = (REG_CIM_SIZE&(~CIM_SIZE_PPL_MASK))|((a)<<CIM_SIZE_PPL_BIT) )
#define __cim_get_line() 	((REG_CIM_SIZE&CIM_SIZE_LPF_MASK)>>CIM_SIZE_LPF_BIT)
#define __cim_get_pixel() 	((REG_CIM_SIZE&CIM_SIZE_PPL_MASK)>>CIM_SIZE_PPL_BIT)

#define __cim_set_v_offset(a) 	( REG_CIM_OFFSET = (REG_CIM_OFFSET&(~CIM_OFFSET_V_MASK)) | ((a)<<CIM_OFFSET_V_BIT) )
#define __cim_set_h_offset(a) 	( REG_CIM_OFFSET = (REG_CIM_OFFSET&(~CIM_OFFSET_H_MASK)) | ((a)<<CIM_OFFSET_H_BIT) )
#define __cim_get_v_offset() 	((REG_CIM_OFFSET&CIM_OFFSET_V_MASK)>>CIM_OFFSET_V_BIT)
#define __cim_get_h_offset() 	((REG_CIM_OFFSET&CIM_OFFSET_H_MASK)>>CIM_OFFSET_H_BIT)

/*************************************************************************
 * SLCD (Smart LCD Controller)
 *************************************************************************/
#define __slcd_set_data_18bit() \
  ( REG_SLCD_CFG = (REG_SLCD_CFG & ~SLCD_CFG_DWIDTH_MASK) | SLCD_CFG_DWIDTH_18BIT )
#define __slcd_set_data_16bit() \
  ( REG_SLCD_CFG = (REG_SLCD_CFG & ~SLCD_CFG_DWIDTH_MASK) | SLCD_CFG_DWIDTH_16BIT )
#define __slcd_set_data_8bit_x3() \
  ( REG_SLCD_CFG = (REG_SLCD_CFG & ~SLCD_CFG_DWIDTH_MASK) | SLCD_CFG_DWIDTH_8BIT_x3 )
#define __slcd_set_data_8bit_x2() \
  ( REG_SLCD_CFG = (REG_SLCD_CFG & ~SLCD_CFG_DWIDTH_MASK) | SLCD_CFG_DWIDTH_8BIT_x2 )
#define __slcd_set_data_8bit_x1() \
  ( REG_SLCD_CFG = (REG_SLCD_CFG & ~SLCD_CFG_DWIDTH_MASK) | SLCD_CFG_DWIDTH_8BIT_x1 )
#define __slcd_set_data_24bit() \
  ( REG_SLCD_CFG = (REG_SLCD_CFG & ~SLCD_CFG_DWIDTH_MASK) | SLCD_CFG_DWIDTH_24BIT )
#define __slcd_set_data_9bit_x2() \
  ( REG_SLCD_CFG = (REG_SLCD_CFG & ~SLCD_CFG_DWIDTH_MASK) | SLCD_CFG_DWIDTH_9BIT_x2 )

#define __slcd_set_cmd_16bit() \
  ( REG_SLCD_CFG = (REG_SLCD_CFG & ~SLCD_CFG_CWIDTH_MASK) | SLCD_CFG_CWIDTH_16BIT )
#define __slcd_set_cmd_8bit() \
  ( REG_SLCD_CFG = (REG_SLCD_CFG & ~SLCD_CFG_CWIDTH_MASK) | SLCD_CFG_CWIDTH_8BIT )
#define __slcd_set_cmd_18bit() \
  ( REG_SLCD_CFG = (REG_SLCD_CFG & ~SLCD_CFG_CWIDTH_MASK) | SLCD_CFG_CWIDTH_18BIT )
#define __slcd_set_cmd_24bit() \
  ( REG_SLCD_CFG = (REG_SLCD_CFG & ~SLCD_CFG_CWIDTH_MASK) | SLCD_CFG_CWIDTH_24BIT )

#define __slcd_set_cs_high()        ( REG_SLCD_CFG |= SLCD_CFG_CS_ACTIVE_HIGH )
#define __slcd_set_cs_low()         ( REG_SLCD_CFG &= ~SLCD_CFG_CS_ACTIVE_HIGH )

#define __slcd_set_rs_high()        ( REG_SLCD_CFG |= SLCD_CFG_RS_CMD_HIGH )
#define __slcd_set_rs_low()         ( REG_SLCD_CFG &= ~SLCD_CFG_RS_CMD_HIGH )

#define __slcd_set_clk_falling()    ( REG_SLCD_CFG &= ~SLCD_CFG_CLK_ACTIVE_RISING )
#define __slcd_set_clk_rising()     ( REG_SLCD_CFG |= SLCD_CFG_CLK_ACTIVE_RISING )

#define __slcd_set_parallel_type()  ( REG_SLCD_CFG &= ~SLCD_CFG_TYPE_SERIAL )
#define __slcd_set_serial_type()    ( REG_SLCD_CFG |= SLCD_CFG_TYPE_SERIAL )

/* SLCD Control Register */
#define __slcd_enable_dma()         ( REG_SLCD_CTRL |= SLCD_CTRL_DMA_EN )
#define __slcd_disable_dma()        ( REG_SLCD_CTRL &= ~SLCD_CTRL_DMA_EN )

/* SLCD Status Register */
#define __slcd_is_busy()            ( REG_SLCD_STATE & SLCD_STATE_BUSY )

/* SLCD Data Register */
#define __slcd_set_cmd_rs()         ( REG_SLCD_DATA |= SLCD_DATA_RS_COMMAND)
#define __slcd_set_data_rs()        ( REG_SLCD_DATA &= ~SLCD_DATA_RS_COMMAND)


/***************************************************************************
 * LCD
 ***************************************************************************/

/***************************************************************************
 * LCD
 ***************************************************************************/
#define __lcd_as_smart_lcd() 		( REG_LCD_CFG |= ( LCD_CFG_LCDPIN_SLCD | LCD_CFG_MODE_SLCD))
#define __lcd_as_general_lcd() 		( REG_LCD_CFG &= ~( LCD_CFG_LCDPIN_SLCD | LCD_CFG_MODE_SLCD))

#define __lcd_enable_tvepeh() 		( REG_LCD_CFG |= LCD_CFG_TVEPEH )
#define __lcd_disable_tvepeh() 		( REG_LCD_CFG &= ~LCD_CFG_TVEPEH )

#define __lcd_enable_fuhold() 		( REG_LCD_CFG |= LCD_CFG_FUHOLD )
#define __lcd_disable_fuhold() 		( REG_LCD_CFG &= ~LCD_CFG_FUHOLD )

#define __lcd_des_8word() 		( REG_LCD_CFG |= LCD_CFG_NEWDES )
#define __lcd_des_4word() 		( REG_LCD_CFG &= ~LCD_CFG_NEWDES )

#define __lcd_enable_bypass_pal() 	( REG_LCD_CFG |= LCD_CFG_PALBP )
#define __lcd_disable_bypass_pal() 	( REG_LCD_CFG &= ~LCD_CFG_PALBP )

#define __lcd_set_lcdpnl_term()		( REG_LCD_CTRL |= LCD_CFG_TVEN )
#define __lcd_set_tv_term()		( REG_LCD_CTRL &= ~LCD_CFG_TVEN )

#define __lcd_enable_auto_recover() 	( REG_LCD_CFG |= LCD_CFG_RECOVER )
#define __lcd_disable_auto_recover() 	( REG_LCD_CFG &= ~LCD_CFG_RECOVER )

#define __lcd_enable_dither() 	        ( REG_LCD_CFG |= LCD_CFG_DITHER )
#define __lcd_disable_dither() 	        ( REG_LCD_CFG &= ~LCD_CFG_DITHER )

#define __lcd_disable_ps_mode()	        ( REG_LCD_CFG |= LCD_CFG_PSM )
#define __lcd_enable_ps_mode()	        ( REG_LCD_CFG &= ~LCD_CFG_PSM )

#define __lcd_disable_cls_mode() 	( REG_LCD_CFG |= LCD_CFG_CLSM )
#define __lcd_enable_cls_mode()	        ( REG_LCD_CFG &= ~LCD_CFG_CLSM )

#define __lcd_disable_spl_mode() 	( REG_LCD_CFG |= LCD_CFG_SPLM )
#define __lcd_enable_spl_mode()	        ( REG_LCD_CFG &= ~LCD_CFG_SPLM )

#define __lcd_disable_rev_mode() 	( REG_LCD_CFG |= LCD_CFG_REVM )
#define __lcd_enable_rev_mode()	        ( REG_LCD_CFG &= ~LCD_CFG_REVM )

#define __lcd_disable_hsync_mode() 	( REG_LCD_CFG |= LCD_CFG_HSYNM )
#define __lcd_enable_hsync_mode()	( REG_LCD_CFG &= ~LCD_CFG_HSYNM )

#define __lcd_disable_pclk_mode() 	( REG_LCD_CFG |= LCD_CFG_PCLKM )
#define __lcd_enable_pclk_mode()	( REG_LCD_CFG &= ~LCD_CFG_PCLKM )

#define __lcd_normal_outdata()          ( REG_LCD_CFG &= ~LCD_CFG_INVDAT )
#define __lcd_inverse_outdata()         ( REG_LCD_CFG |= LCD_CFG_INVDAT )

#define __lcd_sync_input()              ( REG_LCD_CFG |= LCD_CFG_SYNDIR_IN )
#define __lcd_sync_output()             ( REG_LCD_CFG &= ~LCD_CFG_SYNDIR_IN )

#define __lcd_hsync_active_high()       ( REG_LCD_CFG &= ~LCD_CFG_HSP )
#define __lcd_hsync_active_low()        ( REG_LCD_CFG |= LCD_CFG_HSP )

#define __lcd_pclk_rising()             ( REG_LCD_CFG &= ~LCD_CFG_PCP )
#define __lcd_pclk_falling()            ( REG_LCD_CFG |= LCD_CFG_PCP )

#define __lcd_de_active_high()          ( REG_LCD_CFG &= ~LCD_CFG_DEP )
#define __lcd_de_active_low()           ( REG_LCD_CFG |= LCD_CFG_DEP )

#define __lcd_vsync_rising()            ( REG_LCD_CFG &= ~LCD_CFG_VSP )
#define __lcd_vsync_falling()           ( REG_LCD_CFG |= LCD_CFG_VSP )

#define __lcd_set_16_tftpnl() \
  ( REG_LCD_CFG = (REG_LCD_CFG & ~LCD_CFG_MODE_TFT_MASK) | LCD_CFG_MODE_TFT_16BIT )

#define __lcd_set_18_tftpnl() \
  ( REG_LCD_CFG = (REG_LCD_CFG & ~LCD_CFG_MODE_TFT_MASK) | LCD_CFG_MODE_TFT_18BIT )

#define __lcd_set_24_tftpnl()		( REG_LCD_CFG |= LCD_CFG_MODE_TFT_24BIT )

/*
 * n=1,2,4,8 for single mono-STN
 * n=4,8 for dual mono-STN
 */
#define __lcd_set_panel_datawidth(n) 		\
do { 						\
	REG_LCD_CFG &= ~LCD_CFG_PDW_MASK; 	\
	REG_LCD_CFG |= LCD_CFG_PDW_n##;		\
} while (0)

/* m = LCD_CFG_MODE_GENERUIC_TFT_xxx */
#define __lcd_set_panel_mode(m) 		\
do {						\
	REG_LCD_CFG &= ~LCD_CFG_MODE_MASK;	\
	REG_LCD_CFG |= (m);			\
} while(0)

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

#define __lcd_reverse_byte_endian()	( REG_LCD_CTRL |= LCD_CTRL_BEDN )
#define __lcd_normal_byte_endian()	( REG_LCD_CTRL &= ~LCD_CTRL_BEDN )

#define __lcd_pixel_endian_little()	( REG_LCD_CTRL |= LCD_CTRL_PEDN )
#define __lcd_pixel_endian_big()	( REG_LCD_CTRL &= ~LCD_CTRL_PEDN )

#define __lcd_set_dis()			( REG_LCD_CTRL |= LCD_CTRL_DIS )
#define __lcd_clr_dis()			( REG_LCD_CTRL &= ~LCD_CTRL_DIS )

#define __lcd_set_ena()			( REG_LCD_CTRL |= LCD_CTRL_ENA )
#define __lcd_clr_ena()			( REG_LCD_CTRL &= ~LCD_CTRL_ENA )

/* n=1,2,4,8,16 */
#define __lcd_set_bpp(n) \
  ( REG_LCD_CTRL = (REG_LCD_CTRL & ~LCD_CTRL_BPP_MASK) | LCD_CTRL_BPP_##n )

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

/* OSD functions */
#define __lcd_enable_osd() 	(REG_LCD_OSDC |= LCD_OSDC_OSDEN)
#define __lcd_enable_f0() 	(REG_LCD_OSDC |= LCD_OSDC_F0EN)
#define __lcd_enable_f1()	(REG_LCD_OSDC |= LCD_OSDC_F1EN)
#define __lcd_enable_alpha() 	(REG_LCD_OSDC |= LCD_OSDC_ALPHAEN)
#define __lcd_enable_alphamd()	(REG_LCD_OSDC |= LCD_OSDC_ALPHAMD)

#define __lcd_disable_osd()	(REG_LCD_OSDC &= ~LCD_OSDC_OSDEN)
#define __lcd_disable_f0() 	(REG_LCD_OSDC &= ~LCD_OSDC_F0EN)
#define __lcd_disable_f1() 	(REG_LCD_OSDC &= ~LCD_OSDC_F1EN)
#define __lcd_disable_alpha()	(REG_LCD_OSDC &= ~LCD_OSDC_ALPHAEN)
#define __lcd_disable_alphamd()	(REG_LCD_OSDC &= ~LCD_OSDC_ALPHAMD)

/* OSD Controll Register */
#define __lcd_fg1_use_ipu() 		(REG_LCD_OSDCTRL |= LCD_OSDCTRL_IPU)
#define __lcd_fg1_use_dma_chan1() 	(REG_LCD_OSDCTRL &= ~LCD_OSDCTRL_IPU)
#define __lcd_fg1_unuse_ipu() 		__lcd_fg1_use_dma_chan1()
#define __lcd_osd_rgb555_mode()         ( REG_LCD_OSDCTRL |= LCD_OSDCTRL_RGB555 )
#define __lcd_osd_rgb565_mode()         ( REG_LCD_OSDCTRL &= ~LCD_OSDCTRL_RGB555 )
#define __lcd_osd_change_size()         ( REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES )
#define __lcd_osd_bpp_15_16() \
  ( REG_LCD_OSDCTRL = (REG_LCD_OSDCTRL & ~LCD_OSDCTRL_OSDBPP_MASK) | LCD_OSDCTRL_OSDBPP_15_16 )
#define __lcd_osd_bpp_18_24() \
  ( REG_LCD_OSDCTRL = (REG_LCD_OSDCTRL & ~LCD_OSDCTRL_OSDBPP_MASK) | LCD_OSDCTRL_OSDBPP_18_24 )

/* OSD State Register */
#define __lcd_start_of_fg1()		( REG_LCD_STATE & LCD_OSDS_SOF1 )
#define __lcd_end_of_fg1()		( REG_LCD_STATE & LCD_OSDS_EOF1 )
#define __lcd_start_of_fg0()		( REG_LCD_STATE & LCD_OSDS_SOF0 )
#define __lcd_end_of_fg0()		( REG_LCD_STATE & LCD_OSDS_EOF0 )
#define __lcd_change_is_rdy()		( REG_LCD_STATE & LCD_OSDS_READY )

/* Foreground Color Key Register 0,1(foreground 0, foreground 1) */
#define __lcd_enable_colorkey0()	(REG_LCD_KEY0 |= LCD_KEY_KEYEN)
#define __lcd_enable_colorkey1()	(REG_LCD_KEY1 |= LCD_KEY_KEYEN)
#define __lcd_enable_colorkey0_md() 	(REG_LCD_KEY0 |= LCD_KEY_KEYMD)
#define __lcd_enable_colorkey1_md() 	(REG_LCD_KEY1 |= LCD_KEY_KEYMD)
#define __lcd_set_colorkey0(key) 	(REG_LCD_KEY0 = (REG_LCD_KEY0&~0xFFFFFF)|(key))
#define __lcd_set_colorkey1(key) 	(REG_LCD_KEY1 = (REG_LCD_KEY1&~0xFFFFFF)|(key))

#define __lcd_disable_colorkey0() 	(REG_LCD_KEY0 &= ~LCD_KEY_KEYEN)
#define __lcd_disable_colorkey1() 	(REG_LCD_KEY1 &= ~LCD_KEY_KEYEN)
#define __lcd_disable_colorkey0_md() 	(REG_LCD_KEY0 &= ~LCD_KEY_KEYMD)
#define __lcd_disable_colorkey1_md() 	(REG_LCD_KEY1 &= ~LCD_KEY_KEYMD)

/* IPU Restart Register */
#define __lcd_enable_ipu_restart() 	(REG_LCD_IPUR |= LCD_IPUR_IPUREN)
#define __lcd_disable_ipu_restart() 	(REG_LCD_IPUR &= ~LCD_IPUR_IPUREN)
#define __lcd_set_ipu_restart_triger(n)	(REG_LCD_IPUR = (REG_LCD_IPUR&(~0xFFFFFF))|(n))

/* RGB Control Register */
#define __lcd_enable_rgb_dummy() 	(REG_LCD_RGBC |= LCD_RGBC_RGBDM)
#define __lcd_disable_rgb_dummy() 	(REG_LCD_RGBC &= ~LCD_RGBC_RGBDM)

#define __lcd_dummy_rgb() 	(REG_LCD_RGBC |= LCD_RGBC_DMM)
#define __lcd_rgb_dummy() 	(REG_LCD_RGBC &= ~LCD_RGBC_DMM)

#define __lcd_rgb2ycc() 	(REG_LCD_RGBC |= LCD_RGBC_YCC)
#define __lcd_notrgb2ycc() 	(REG_LCD_RGBC &= ~LCD_RGBC_YCC)

#define __lcd_odd_mode_rgb() \
  ( REG_LCD_RGBC = (REG_LCD_RGBC & ~LCD_RGBC_ODDRGB_MASK) | LCD_RGBC_ODD_RGB )
#define __lcd_odd_mode_rbg() \
  ( REG_LCD_RGBC = (REG_LCD_RGBC & ~LCD_RGBC_ODDRGB_MASK) | LCD_RGBC_ODD_RBG )
#define __lcd_odd_mode_grb() \
  ( REG_LCD_RGBC = (REG_LCD_RGBC & ~LCD_RGBC_ODDRGB_MASK) | LCD_RGBC_ODD_GRB)

#define __lcd_odd_mode_gbr() \
  ( REG_LCD_RGBC = (REG_LCD_RGBC & ~LCD_RGBC_ODDRGB_MASK) | LCD_RGBC_ODD_GBR)
#define __lcd_odd_mode_brg() \
  ( REG_LCD_RGBC = (REG_LCD_RGBC & ~LCD_RGBC_ODDRGB_MASK) | LCD_RGBC_ODD_BRG)
#define __lcd_odd_mode_bgr() \
  ( REG_LCD_RGBC = (REG_LCD_RGBC & ~LCD_RGBC_ODDRGB_MASK) | LCD_RGBC_ODD_BGR)

#define __lcd_even_mode_rgb() \
  ( REG_LCD_RGBC = (REG_LCD_RGBC & ~LCD_RGBC_EVENRGB_MASK) | LCD_RGBC_EVEN_RGB )
#define __lcd_even_mode_rbg() \
  ( REG_LCD_RGBC = (REG_LCD_RGBC & ~LCD_RGBC_EVENRGB_MASK) | LCD_RGBC_EVEN_RBG )
#define __lcd_even_mode_grb() \
  ( REG_LCD_RGBC = (REG_LCD_RGBC & ~LCD_RGBC_EVENRGB_MASK) | LCD_RGBC_EVEN_GRB)

#define __lcd_even_mode_gbr() \
  ( REG_LCD_RGBC = (REG_LCD_RGBC & ~LCD_RGBC_EVENRGB_MASK) | LCD_RGBC_EVEN_GBR)
#define __lcd_even_mode_brg() \
  ( REG_LCD_RGBC = (REG_LCD_RGBC & ~LCD_RGBC_EVENRGB_MASK) | LCD_RGBC_EVEN_BRG)
#define __lcd_even_mode_bgr() \
  ( REG_LCD_RGBC = (REG_LCD_RGBC & ~LCD_RGBC_EVENRGB_MASK) | LCD_RGBC_EVEN_BGR)

/* Vertical Synchronize Register */
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

/* DMA Command Register */
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

/*************************************************************************
 * TVE (TV Encoder Controller) ops
 *************************************************************************/
/* TV Encoder Control register ops */
#define __tve_soft_reset()		(REG_TVE_CTRL |= TVE_CTRL_SWRST)

#define __tve_output_colorbar()		(REG_TVE_CTRL |= TVE_CTRL_CLBAR)
#define __tve_output_video()		(REG_TVE_CTRL &= ~TVE_CTRL_CLBAR)

#define __tve_input_cr_first()		(REG_TVE_CTRL |= TVE_CTRL_CR1ST)
#define __tve_input_cb_first()		(REG_TVE_CTRL &= ~TVE_CTRL_CR1ST)

#define __tve_set_0_as_black()		(REG_TVE_CTRL |= TVE_CTRL_ZBLACK)
#define __tve_set_16_as_black()		(REG_TVE_CTRL &= ~TVE_CTRL_ZBLACK)

#define __tve_ena_invert_top_bottom()	(REG_TVE_CTRL |= TVE_CTRL_FINV)
#define __tve_dis_invert_top_bottom()	(REG_TVE_CTRL &= ~TVE_CTRL_FINV)

#define __tve_set_pal_mode()		(REG_TVE_CTRL |= TVE_CTRL_PAL)
#define __tve_set_ntsc_mode()		(REG_TVE_CTRL &= ~TVE_CTRL_PAL)

#define __tve_set_pal_dura()		(REG_TVE_CTRL |= TVE_CTRL_SYNCT)
#define __tve_set_ntsc_dura()		(REG_TVE_CTRL &= ~TVE_CTRL_SYNCT)

/* n = 0 ~ 3 */
#define __tve_set_c_bandwidth(n) \
do {\
	REG_TVE_CTRL &= ~TVE_CTRL_CBW_MASK;\
	REG_TVE_CTRL |= (n) << TVE_CTRL_CBW_BIT;	\
}while(0)

/* n = 0 ~ 3 */
#define __tve_set_c_gain(n) \
do {\
	REG_TVE_CTRL &= ~TVE_CTRL_CGAIN_MASK;\
	(REG_TVE_CTRL |= (n) << TVE_CTRL_CGAIN_BIT;	\
}while(0)

/* n = 0 ~ 7 */
#define __tve_set_yc_delay(n)				\
do {							\
	REG_TVE_CTRL &= ~TVE_CTRL_YCDLY_MASK		\
	REG_TVE_CTRL |= ((n) << TVE_CTRL_YCDLY_BIT);	\
} while(0)

#define __tve_disable_all_dacs()	(REG_TVE_CTRL |= TVE_CTRL_DAPD)
#define __tve_disable_dac1()		(REG_TVE_CTRL |= TVE_CTRL_DAPD1)
#define __tve_enable_dac1()		(REG_TVE_CTRL &= ~TVE_CTRL_DAPD1)
#define __tve_disable_dac2()		(REG_TVE_CTRL |= TVE_CTRL_DAPD2)
#define __tve_enable_dac2()		(REG_TVE_CTRL &= ~TVE_CTRL_DAPD2)
#define __tve_disable_dac3()		(REG_TVE_CTRL |= TVE_CTRL_DAPD3)
#define __tve_enable_dac3()		(REG_TVE_CTRL &= ~TVE_CTRL_DAPD3)

#define __tve_enable_svideo_fmt()	(REG_TVE_CTRL |= TVE_CTRL_ECVBS)
#define __tve_enable_cvbs_fmt()		(REG_TVE_CTRL &= ~TVE_CTRL_ECVBS)

/* TV Encoder Frame Configure register ops */
/* n = 0 ~ 255 */
#define __tve_set_first_video_line(n)		\
do {\
		REG_TVE_FRCFG &= ~TVE_FRCFG_L1ST_MASK;\
		REG_TVE_FRCFG |= (n) << TVE_FRCFG_L1ST_BIT;\
} while(0)
/* n = 0 ~ 1023 */
#define __tve_set_line_num_per_frm(n)		\
do {\
		REG_TVE_FRCFG &= ~TVE_FRCFG_NLINE_MASK;\
		REG_TVE_CFG |= (n) << TVE_FRCFG_NLINE_BIT;\
} while(0)
#define __tve_get_video_line_num()\
	(((REG_TVE_FRCFG & TVE_FRCFG_NLINE_MASK) >> TVE_FRCFG_NLINE_BIT) - 1 - 2 * ((REG_TVE_FRCFG & TVE_FRCFG_L1ST_MASK) >> TVE_FRCFG_L1ST_BIT))

/* TV Encoder Signal Level Configure register ops */
/* n = 0 ~ 1023 */
#define __tve_set_white_level(n)		\
do {\
		REG_TVE_SLCFG1 &= ~TVE_SLCFG1_WHITEL_MASK;\
		REG_TVE_SLCFG1 |= (n) << TVE_SLCFG1_WHITEL_BIT;\
} while(0)
/* n = 0 ~ 1023 */
#define __tve_set_black_level(n)		\
do {\
		REG_TVE_SLCFG1 &= ~TVE_SLCFG1_BLACKL_MASK;\
		REG_TVE_SLCFG1 |= (n) << TVE_SLCFG1_BLACKL_BIT;\
} while(0)
/* n = 0 ~ 1023 */
#define __tve_set_blank_level(n)		\
do {\
		REG_TVE_SLCFG2 &= ~TVE_SLCFG2_BLANKL_MASK;\
		REG_TVE_SLCFG2 |= (n) << TVE_SLCFG2_BLANKL_BIT;\
} while(0)
/* n = 0 ~ 1023 */
#define __tve_set_vbi_blank_level(n)		\
do {\
		REG_TVE_SLCFG2 &= ~TVE_SLCFG2_VBLANKL_MASK;\
		REG_TVE_SLCFG2 |= (n) << TVE_SLCFG2_VBLANKL_BIT;\
} while(0)
/* n = 0 ~ 1023 */
#define __tve_set_sync_level(n)		\
do {\
		REG_TVE_SLCFG3 &= ~TVE_SLCFG3_SYNCL_MASK;\
		REG_TVE_SLCFG3 |= (n) << TVE_SLCFG3_SYNCL_BIT;\
} while(0)

/* TV Encoder Signal Level Configure register ops */
/* n = 0 ~ 31 */
#define __tve_set_front_porch(n)		\
do {\
		REG_TVE_LTCFG1 &= ~TVE_LTCFG1_FRONTP_MASK;\
		REG_TVE_LTCFG1 |= (n) << TVE_LTCFG1_FRONTP_BIT;	\
} while(0)
/* n = 0 ~ 127 */
#define __tve_set_hsync_width(n)		\
do {\
		REG_TVE_LTCFG1 &= ~TVE_LTCFG1_HSYNCW_MASK;\
		REG_TVE_LTCFG1 |= (n) << TVE_LTCFG1_HSYNCW_BIT;	\
} while(0)
/* n = 0 ~ 127 */
#define __tve_set_back_porch(n)		\
do {\
		REG_TVE_LTCFG1 &= ~TVE_LTCFG1_BACKP_MASK;\
		REG_TVE_LTCFG1 |= (n) << TVE_LTCFG1_BACKP_BIT;	\
} while(0)
/* n = 0 ~ 2047 */
#define __tve_set_active_linec(n)		\
do {\
		REG_TVE_LTCFG2 &= ~TVE_LTCFG2_ACTLIN_MASK;\
		REG_TVE_LTCFG2 |= (n) << TVE_LTCFG2_ACTLIN_BIT;	\
} while(0)
/* n = 0 ~ 31 */
#define __tve_set_breezy_way(n)		\
do {\
		REG_TVE_LTCFG2 &= ~TVE_LTCFG2_PREBW_MASK;\
		REG_TVE_LTCFG2 |= (n) << TVE_LTCFG2_PREBW_BIT;	\
} while(0)

/* n = 0 ~ 127 */
#define __tve_set_burst_width(n)		\
do {\
		REG_TVE_LTCFG2 &= ~TVE_LTCFG2_BURSTW_MASK;\
		REG_TVE_LTCFG2 |= (n) << TVE_LTCFG2_BURSTW_BIT;	\
} while(0)

/* TV Encoder Chrominance filter and Modulation register ops */
/* n = 0 ~ (2^32-1) */
#define __tve_set_c_sub_carrier_freq(n)  REG_TVE_CFREQ = (n)
/* n = 0 ~ 255 */
#define __tve_set_c_sub_carrier_init_phase(n) \
do {   \
	REG_TVE_CPHASE &= ~TVE_CPHASE_INITPH_MASK;	\
	REG_TVE_CPHASE |= (n) << TVE_CPHASE_INITPH_BIT;	\
} while(0)
/* n = 0 ~ 255 */
#define __tve_set_c_sub_carrier_act_phase(n) \
do {   \
	REG_TVE_CPHASE &= ~TVE_CPHASE_ACTPH_MASK;	\
	REG_TVE_CPHASE |= (n) << TVE_CPHASE_ACTPH_BIT;	\
} while(0)
/* n = 0 ~ 255 */
#define __tve_set_c_phase_rst_period(n) \
do {   \
	REG_TVE_CPHASE &= ~TVE_CPHASE_CCRSTP_MASK;	\
	REG_TVE_CPHASE |= (n) << TVE_CPHASE_CCRSTP_BIT;	\
} while(0)
/* n = 0 ~ 255 */
#define __tve_set_cb_burst_amp(n) \
do {   \
	REG_TVE_CBCRCFG &= ~TVE_CBCRCFG_CBBA_MASK;	\
	REG_TVE_CBCRCFG |= (n) << TVE_CBCRCFG_CBBA_BIT;	\
} while(0)
/* n = 0 ~ 255 */
#define __tve_set_cr_burst_amp(n) \
do {   \
	REG_TVE_CBCRCFG &= ~TVE_CBCRCFG_CRBA_MASK;	\
	REG_TVE_CBCRCFG |= (n) << TVE_CBCRCFG_CRBA_BIT;	\
} while(0)
/* n = 0 ~ 255 */
#define __tve_set_cb_gain_amp(n) \
do {   \
	REG_TVE_CBCRCFG &= ~TVE_CBCRCFG_CBGAIN_MASK;	\
	REG_TVE_CBCRCFG |= (n) << TVE_CBCRCFG_CBGAIN_BIT;	\
} while(0)
/* n = 0 ~ 255 */
#define __tve_set_cr_gain_amp(n) \
do {   \
	REG_TVE_CBCRCFG &= ~TVE_CBCRCFG_CRGAIN_MASK;	\
	REG_TVE_CBCRCFG |= (n) << TVE_CBCRCFG_CRGAIN_BIT;	\
} while(0)

/* TV Encoder Wide Screen Signal Control register ops */
/* n = 0 ~ 7 */
#define __tve_set_notch_freq(n) \
do {   \
	REG_TVE_WSSCR &= ~TVE_WSSCR_NCHFREQ_MASK;	\
	REG_TVE_WSSCR |= (n) << TVE_WSSCR_NCHFREQ_BIT;	\
} while(0)
/* n = 0 ~ 7 */
#define __tve_set_notch_width()	(REG_TVE_WSSCR |= TVE_WSSCR_NCHW_BIT)
#define __tve_clear_notch_width()	(REG_TVE_WSSCR &= ~TVE_WSSCR_NCHW_BIT)
#define __tve_enable_notch()		(REG_TVE_WSSCR |= TVE_WSSCR_ENCH_BIT)
#define __tve_disable_notch()		(REG_TVE_WSSCR &= ~TVE_WSSCR_ENCH_BIT)
/* n = 0 ~ 7 */
#define __tve_set_wss_edge(n) \
do {   \
	REG_TVE_WSSCR &= ~TVE_WSSCR_WSSEDGE_MASK;	\
	REG_TVE_WSSCR |= (n) << TVE_WSSCR_WSSEDGE_BIT;	\
} while(0)
#define __tve_set_wss_clkbyp()		(REG_TVE_WSSCR |= TVE_WSSCR_WSSCKBP_BIT)
#define __tve_set_wss_type()		(REG_TVE_WSSCR |= TVE_WSSCR_WSSTP_BIT)
#define __tve_enable_wssf1()		(REG_TVE_WSSCR |= TVE_WSSCR_EWSS1_BIT)
#define __tve_enable_wssf0()		(REG_TVE_WSSCR |= TVE_WSSCR_EWSS0_BIT)

/* TV Encoder Wide Screen Signal Configure register 1, 2 and 3 ops */
/* n = 0 ~ 1023 */
#define __tve_set_wss_level(n) \
do {   \
	REG_TVE_WSSCFG1 &= ~TVE_WSSCFG1_WSSL_MASK;	\
	REG_TVE_WSSCFG1 |= (n) << TVE_WSSCFG1_WSSL_BIT;	\
} while(0)
/* n = 0 ~ 4095 */
#define __tve_set_wss_freq(n) \
do {   \
	REG_TVE_WSSCFG1 &= ~TVE_WSSCFG1_WSSFREQ_MASK;	\
	REG_TVE_WSSCFG1 |= (n) << TVE_WSSCFG1_WSSFREQ_BIT;	\
} while(0)
/* n = 0, 1; l = 0 ~ 255 */
#define __tve_set_wss_line(n,v)			\
do {   \
	REG_TVE_WSSCFG##n &= ~TVE_WSSCFG_WSSLINE_MASK;	\
	REG_TVE_WSSCFG##n |= (v) << TVE_WSSCFG_WSSLINE_BIT;	\
} while(0)
/* n = 0, 1; d = 0 ~ (2^20-1) */
#define __tve_set_wss_data(n, v)			\
do {   \
	REG_TVE_WSSCFG##n &= ~TVE_WSSCFG_WSSLINE_MASK;	\
	REG_TVE_WSSCFG##n |= (v) << TVE_WSSCFG_WSSLINE_BIT;	\
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

#define __rtc_status_hib_reset_occur()        ( REG_RTC_HWRSR & RTC_HWRSR_HR )
#define __rtc_status_ppr_reset_occur()        ( REG_RTC_HWRSR & RTC_HWRSR_PPR )
#define __rtc_status_wakeup_pin_waken_up()    ( REG_RTC_HWRSR & RTC_HWRSR_PIN )
#define __rtc_status_alarm_waken_up()        ( REG_RTC_HWRSR & RTC_HWRSR_ALM )
#define __rtc_clear_hib_stat_all()               ( REG_RTC_HWRSR = 0 )

#define __rtc_get_scratch_pattern() 		(REG_RTC_HSPR)
#define __rtc_set_scratch_pattern(n) 		(REG_RTC_HSPR = n )

/*************************************************************************
 * BCH
 *************************************************************************/
#define __ecc_encoding_4bit()                                   \
do {				   		        	\
	REG_BCH_CRS = BCH_CR_ENCE | BCH_CR_BRST | BCH_CR_BCHE;  \
	REG_BCH_CRC = BCH_CR_BSEL8;				\
} while(0)
#define __ecc_decoding_4bit()                           \
do {                                                    \
	REG_BCH_CRS = BCH_CR_BRST | BCH_CR_BCHE;	\
	REG_BCH_CRC = BCH_CR_ENCE | BCH_CR_BSEL8;	\
} while(0)
#define __ecc_encoding_8bit()                                                   \
do {				   		                        	\
	REG_BCH_CRS = BCH_CR_ENCE | BCH_CR_BRST | BCH_CR_BSEL8 | BCH_CR_BCHE;   \
} while(0)
#define __ecc_decoding_8bit()                                        \
do {                                                                 \
	REG_BCH_CRS = BCH_CR_BRST | BCH_CR_BSEL8 | BCH_CR_BCHE;	     \
	REG_BCH_CRC = BCH_CR_ENCE;	                             \
} while(0)
#define __ecc_dma_enable()        ( REG_BCH_CRS = BCH_CR_DMAE )
#define __ecc_dma_disable()       ( REG_BCH_CRC = BCH_CR_DMAE )
#define __ecc_disable()           ( REG_BCH_CRC = BCH_CR_BCHE )
#define __ecc_encode_sync()       while (!(REG_BCH_INTS & BCH_INTS_ENCF))
#define __ecc_decode_sync()       while (!(REG_BCH_INTS & BCH_INTS_DECF))
#define __ecc_cnt_dec(n)                                             \
do {                                                                 \
        REG_BCH_CNT &= ~(BCH_CNT_DEC_MASK << BCH_CNT_DEC_BIT);       \
        REG_BCH_CNT = (n) << BCH_CNT_DEC_BIT;                        \
} while(0)
#define __ecc_cnt_enc(n)                                             \
do {                                                                 \
        REG_BCH_CNT &= ~(BCH_CNT_ENC_MASK << BCH_CNT_ENC_BIT);       \
        REG_BCH_CNT = (n) << BCH_CNT_ENC_BIT;                        \
} while(0)

/***************************************************************************
 * OWI (one-wire bus)  ops
 ***************************************************************************/

/* OW control register ops */
#define __owi_enable_all_interrupts()      ( REG_OWI_CTL = (OWI_CTL_EBYTE | OWI_CTL_EBIT | OWI_CTL_ERST) )
#define __owi_disable_all_interrupts()     ( REG_OWI_CTL = 0 )

#define __owi_enable_byte_interrupt()      ( REG_OWI_CTL |= OWI_CTL_EBYTE )
#define __owi_disable_byte_interrupt()     ( REG_OWI_CTL &= ~OWI_CTL_EBYTE )
#define __owi_enable_bit_interrupt()       ( REG_OWI_CTL |= OWI_CTL_EBIT )
#define __owi_disable_bit_interrupt()      ( REG_OWI_CTL &= ~OWI_CTL_EBIT )
#define __owi_enable_rst_interrupt()       ( REG_OWI_CTL |= OWI_CTL_ERST )
#define __owi_disable_rst_interrupt()      ( REG_OWI_CTL &=~OWI_CTL_ERST )

/* OW configure register ops */
#define __owi_select_regular_mode()        ( REG_OWI_CFG &= ~OWI_CFG_MODE )
#define __owi_select_overdrive_mode()      ( REG_OWI_CFG |= OWI_CFG_MODE )

#define __owi_set_rddata()  ( REG_OWI_CFG |= OWI_CFG_RDDATA )
#define __owi_clr_rddata()  ( REG_OWI_CFG &= ~OWI_CFG_RDDATA )
#define __owi_get_rddata()  ( REG_OWI_CFG & OWI_CFG_RDDATA )

#define __owi_set_wrdata()  ( REG_OWI_CFG |= OWI_CFG_WRDATA )
#define __owi_clr_wrdata()  ( REG_OWI_CFG &= ~OWI_CFG_WRDATA )
#define __owi_get_wrdata()  ( REG_OWI_CFG & OWI_CFG_WRDATA )

#define __owi_get_rdst()    ( REG_OWI_CFG & OWI_CFG_RDST )

#define __owi_set_wr1rd()   ( REG_OWI_CFG |= OWI_CFG_WR1RD )
#define __owi_clr_wr1rd()   ( REG_OWI_CFG &= ~OWI_CFG_WR1RD )
#define __owi_get_wr1rd()   ( REG_OWI_CFG & OWI_CFG_WR1RD )

#define __owi_set_wr0()     ( REG_OWI_CFG |= OWI_CFG_WR0 )
#define __owi_clr_wr0()     ( REG_OWI_CFG &= ~OWI_CFG_WR0 )
#define __owi_get_wr0()     ( REG_OWI_CFG & OWI_CFG_WR0 )

#define __owi_set_rst()     ( REG_OWI_CFG |= OWI_CFG_RST )
#define __owi_clr_rst()     ( REG_OWI_CFG &= ~OWI_CFG_RST )
#define __owi_get_rst()     ( REG_OWI_CFG & OWI_CFG_RST )

#define __owi_enable_ow_ops()  ( REG_OWI_CFG |= OWI_CFG_ENA )
#define __owi_disable_ow_ops() ( REG_OWI_CFG &= ~OWI_CFG_ENA )
#define __owi_get_enable()     ( REG_OWI_CFG & OWI_CFG_ENA )

#define __owi_wait_ops_rdy()                \
	do {				    \
		while(__owi_get_enable());  \
		udelay(1);		    \
	} while(0);

/* OW status register ops */
#define __owi_clr_sts()           ( REG_OWI_STS = 0 )
#define __owi_get_sts_pst()       ( REG_OWI_STS & OWI_STS_PST )
#define __owi_get_sts_byte_rdy()  ( REG_OWI_STS & OWI_STS_BYTE_RDY )
#define __owi_get_sts_bit_rdy()   ( REG_OWI_STS & OWI_STS_BIT_RDY )
#define __owi_get_sts_pst_rdy()   ( REG_OWI_STS & OWI_STS_PST_RDY )

/*************************************************************************
 * TSSI MPEG 2-TS slave interface operation
 *************************************************************************/
#define __tssi_enable()                       ( REG_TSSI_ENA |= TSSI_ENA_ENA )
#define __tssi_disable()                      ( REG_TSSI_ENA &= ~TSSI_ENA_ENA )
#define __tssi_soft_reset()                   ( REG_TSSI_ENA |= TSSI_ENA_SFT_RST )
#define __tssi_dma_enable()                   ( REG_TSSI_ENA |= TSSI_ENA_DMA_EN )
#define __tssi_dma_disable()                  ( REG_TSSI_ENA &= ~TSSI_ENA_DMA_EN )
#define __tssi_filter_enable()                ( REG_TSSI_ENA |= TSSI_ENA_PID_EN )
#define __tssi_filter_disable()               ( REG_TSSI_ENA &= ~TSSI_ENA_PID_EN )

/* n = 4, 8, 16 */
#define __tssi_set_tigger_num(n)			\
	do {						\
		REG_TSSI_CFG &= ~TSSI_CFG_TRIG_MASK;	\
		REG_TSSI_CFG |= TSSI_CFG_TRIG_##n;	\
	} while (0)

#define __tssi_set_wd_1()                     ( REG_TSSI_CFG |= TSSI_CFG_END_WD )
#define __tssi_set_wd_0()                     ( REG_TSSI_CFG &= ~TSSI_CFG_END_WD )

#define __tssi_set_bt_1()                     ( REG_TSSI_CFG |= TSSI_CFG_END_BD )
#define __tssi_set_bt_0()                     ( REG_TSSI_CFG &= ~TSSI_CFG_END_BD )

#define __tssi_set_data_pola_high()           ( REG_TSSI_CFG |= TSSI_CFG_TSDI_H )
#define __tssi_set_data_pola_low()            ( REG_TSSI_CFG &= ~TSSI_CFG_TSDI_H )

#define __tssi_set_data_use_data0()           ( REG_TSSI_CFG |= TSSI_CFG_USE_0 )
#define __tssi_set_data_use_data7()           ( REG_TSSI_CFG &= ~TSSI_CFG_USE_0 )

#define __tssi_select_clk_fast()              ( REG_TSSI_CFG &= ~TSSI_CFG_TSCLK_CH )
#define __tssi_select_clk_slow()              ( REG_TSSI_CFG |= TSSI_CFG_TSCLK_CH )

#define __tssi_select_serail_mode()           ( REG_TSSI_CFG &= ~TSSI_CFG_PARAL )
#define __tssi_select_paral_mode()            ( REG_TSSI_CFG |= TSSI_CFG_PARAL )

#define __tssi_select_clk_nega_edge()         ( REG_TSSI_CFG &= ~TSSI_CFG_TSCLK_P )
#define __tssi_select_clk_posi_edge()         ( REG_TSSI_CFG |= TSSI_CFG_TSCLK_P )

#define __tssi_select_frm_act_high()          ( REG_TSSI_CFG |= TSSI_CFG_TSFRM_H )
#define __tssi_select_frm_act_low()           ( REG_TSSI_CFG &= ~TSSI_CFG_TSFRM_H )

#define __tssi_select_str_act_high()          ( REG_TSSI_CFG |= TSSI_CFG_TSSTR_H )
#define __tssi_select_str_act_low()           ( REG_TSSI_CFG &= ~TSSI_CFG_TSSTR_H )

#define __tssi_select_fail_act_high()         ( REG_TSSI_CFG |= TSSI_CFG_TSFAIL_H )
#define __tssi_select_fail_act_low()          ( REG_TSSI_CFG &= ~TSSI_CFG_TSFAIL_H )

#define __tssi_enable_ovrn_irq()              ( REG_TSSI_CTRL &= ~TSSI_CTRL_OVRNM )
#define __tssi_disable_ovrn_irq()             ( REG_TSSI_CTRL |= TSSI_CTRL_OVRNM )

#define __tssi_enable_trig_irq()              ( REG_TSSI_CTRL &= ~TSSI_CTRL_TRIGM )
#define __tssi_disable_trig_irq()             ( REG_TSSI_CTRL |= TSSI_CTRL_TRIGM )

#define __tssi_state_is_overrun()             ( REG_TSSI_STAT & TSSI_STAT_OVRN )
#define __tssi_state_trigger_meet()           ( REG_TSSI_STAT & TSSI_STAT_TRIG )
#define __tssi_clear_state()                  ( REG_TSSI_STAT = 0 ) /* write 0??? */
#define __tssi_state_clear_overrun()          ( REG_TSSI_STAT = TSSI_STAT_OVRN )

#define __tssi_enable_filte_pid0()            ( REG_TSSI_PEN |= TSSI_PEN_PID0 )
#define __tssi_disable_filte_pid0()           ( REG_TSSI_PEN &= ~TSSI_PEN_PID0 )

/* m = 0, ..., 15 */
#define __tssi_enable_pid_filter(m)				\
	do {							\
		int n = (m);					\
		if ( n>=0 && n <(TSSI_PID_MAX*2) ) {		\
			if ( n >= TSSI_PID_MAX ) n += 8;	\
			REG_TSSI_PEN |= ( 1 << n );		\
		}						\
	} while (0)

/* m = 0, ..., 15 */
#define __tssi_disable_pid_filter(m)				       \
	do {							       \
		int n = (m);					       \
		if ( n>=0 && n <(TSSI_PID_MAX*2) ) {		       \
			if ( n >= TSSI_PID_MAX ) n += 8;	       \
			REG_TSSI_PEN &= ~( 1 << n );		       \
		}						       \
	} while (0)

/* n = 0, ..., 7 */
#define __tssi_set_pid0(n, pid0)					\
	do {								\
		REG_TSSI_PID(n) &= ~TSSI_PID_PID0_MASK;			\
		REG_TSSI_PID(n) |= ((pid0)<<TSSI_PID_PID0_BIT)&TSSI_PID_PID0_MASK; \
	}while (0)
/* n = 0, ..., 7 */
#define __tssi_set_pid1(n, pid1)					\
	do {								\
		REG_TSSI_PID(n) &= ~TSSI_PID_PID1_MASK;			\
		REG_TSSI_PID(n) |= ((pid1)<<TSSI_PID_PID1_BIT)&TSSI_PID_PID1_MASK; \
	}while (0)

/* n = 0, ..., 15 */
#define __tssi_set_pid(n, pid)						\
	do {								\
		if ( n>=0 && n < TSSI_PID_MAX*2) {			\
			if ( n < TSSI_PID_MAX )				\
				__tssi_set_pid0(n, pid);		\
			else						\
				__tssi_set_pid1(n-TSSI_PID_MAX, pid);	\
		}							\
	}while (0)


#if 0
/*************************************************************************
 * IPU (Image Processing Unit)
 *************************************************************************/
#define u32 volatile unsigned long

#define write_reg(reg, val)	\
do {				\
	*(u32 *)(reg) = (val);	\
} while(0)

#define read_reg(reg, off)	(*(u32 *)((reg)+(off)))


#define set_ipu_fmt(rgb_888_out_fmt, rgb_out_oft, out_fmt, yuv_pkg_out, in_oft, in_fmt ) \
({ write_reg( (IPU_V_BASE + REG_D_FMT), ((in_fmt) & IN_FMT_MSK)<<IN_FMT_SFT \
| ((in_oft) & IN_OFT_MSK)<< IN_OFT_SFT \
| ((out_fmt) & OUT_FMT_MSK)<<OUT_FMT_SFT \
| ((yuv_pkg_out) & YUV_PKG_OUT_MSK ) << YUV_PKG_OUT_SFT \
| ((rgb_888_out_fmt) & RGB888_FMT_MSK ) << RGB888_FMT_SFT \
| ((rgb_out_oft) & RGB_OUT_OFT_MSK ) << RGB_OUT_OFT_SFT); \
})
#define set_y_addr(y_addr) \
({ write_reg( (IPU_V_BASE + REG_Y_ADDR), y_addr); \
})
#define set_u_addr(u_addr) \
({ write_reg( (IPU_V_BASE + REG_U_ADDR), u_addr); \
})

#define set_v_addr(v_addr) \
({ write_reg( (IPU_V_BASE + REG_V_ADDR), v_addr); \
})

#define set_y_phy_t_addr(y_phy_t_addr) \
({ write_reg( (IPU_V_BASE + REG_Y_PHY_T_ADDR), y_phy_t_addr); \
})

#define set_u_phy_t_addr(u_phy_t_addr) \
({ write_reg( (IPU_V_BASE + REG_U_PHY_T_ADDR), u_phy_t_addr); \
})

#define set_v_phy_t_addr(v_phy_t_addr) \
({ write_reg( (IPU_V_BASE + REG_V_PHY_T_ADDR), v_phy_t_addr); \
})

#define set_out_phy_t_addr(out_phy_t_addr) \
({ write_reg( (IPU_V_BASE + REG_OUT_PHY_T_ADDR), out_phy_t_addr); \
})

#define set_inframe_gsize(width, height, y_stride, u_stride, v_stride) \
({ write_reg( (IPU_V_BASE + REG_IN_FM_GS), ((width) & IN_FM_W_MSK)<<IN_FM_W_SFT \
| ((height) & IN_FM_H_MSK)<<IN_FM_H_SFT); \
 write_reg( (IPU_V_BASE + REG_Y_STRIDE), ((y_stride) & Y_S_MSK)<<Y_S_SFT); \
 write_reg( (IPU_V_BASE + REG_UV_STRIDE), ((u_stride) & U_S_MSK)<<U_S_SFT \
| ((v_stride) & V_S_MSK)<<V_S_SFT); \
})
#define set_out_addr(out_addr) \
({ write_reg( (IPU_V_BASE + REG_OUT_ADDR), out_addr); \
})
#define set_outframe_gsize(width, height, o_stride) \
({ write_reg( (IPU_V_BASE + REG_OUT_GS), ((width) & OUT_FM_W_MSK)<<OUT_FM_W_SFT \
| ((height) & OUT_FM_H_MSK)<<OUT_FM_H_SFT); \
 write_reg( (IPU_V_BASE + REG_OUT_STRIDE), ((o_stride) & OUT_S_MSK)<<OUT_S_SFT); \
})
#define set_rsz_lut_end(h_end, v_end) \
({ write_reg( (IPU_V_BASE + REG_RSZ_COEF_INDEX), ((h_end) & HE_IDX_MSK)<<HE_IDX_SFT \
| ((v_end) & VE_IDX_MSK)<<VE_IDX_SFT); \
})
#define set_csc_c0(c0_coeff) \
({ write_reg( (IPU_V_BASE + REG_CSC_CO_COEF), ((c0_coeff) & CX_COEF_MSK)<<CX_COEF_SFT); \
})
#define set_csc_c1(c1_coeff) \
({ write_reg( (IPU_V_BASE + REG_CSC_C1_COEF), ((c1_coeff) & CX_COEF_MSK)<<CX_COEF_SFT); \
})
#define set_csc_c2(c2_coeff) \
({ write_reg( (IPU_V_BASE + REG_CSC_C2_COEF), ((c2_coeff) & CX_COEF_MSK)<<CX_COEF_SFT); \
})
#define set_csc_c3(c3_coeff) \
({ write_reg( (IPU_V_BASE + REG_CSC_C3_COEF), ((c3_coeff) & CX_COEF_MSK)<<CX_COEF_SFT); \
})
#define set_csc_c4(c4_coeff) \
({ write_reg( (IPU_V_BASE + REG_CSC_C4_COEF), ((c4_coeff) & CX_COEF_MSK)<<CX_COEF_SFT); \
})
#define set_hrsz_lut_coef(coef, in_n, out_n) \
({ write_reg( (IPU_V_BASE + HRSZ_LUT_BASE ), ((coef) & W_COEF_MSK)<<W_COEF_SFT \
| ((in_n) & IN_N_MSK)<<IN_N_SFT | ((out_n) & OUT_N_MSK)<<OUT_N_SFT); \
})
#define set_vrsz_lut_coef(coef, in_n, out_n) \
({ write_reg( (IPU_V_BASE + VRSZ_LUT_BASE), ((coef) & W_COEF_MSK)<<W_COEF_SFT \
| ((in_n) & IN_N_MSK)<<IN_N_SFT | ((out_n) & OUT_N_MSK)<<OUT_N_SFT); \
})

#define set_primary_ctrl(vrsz_en, hrsz_en,csc_en, irq_en) \
({ write_reg( (IPU_V_BASE + REG_CTRL), ((irq_en) & FM_IRQ_EN_MSK)<<FM_IRQ_EN_SFT \
| ((vrsz_en) & VRSZ_EN_MSK)<<VRSZ_EN_SFT \
| ((hrsz_en) & HRSZ_EN_MSK)<<HRSZ_EN_SFT \
| ((csc_en) & CSC_EN_MSK)<<CSC_EN_SFT \
| (read_reg(IPU_V_BASE, REG_CTRL)) \
& ~(CSC_EN_MSK<<CSC_EN_SFT | FM_IRQ_EN_MSK<<FM_IRQ_EN_SFT | VRSZ_EN_MSK<<VRSZ_EN_SFT | HRSZ_EN_MSK<<HRSZ_EN_SFT ) ); \
})

#define set_source_ctrl(pkg_sel, spage_sel) \
({ write_reg( (IPU_V_BASE + REG_CTRL), ((pkg_sel) & PKG_SEL_MSK  )<< PKG_SEL_SFT \
| ((spage_sel) & SPAGE_MAP_MSK )<< SPAGE_MAP_SFT \
| (read_reg(IPU_V_BASE, REG_CTRL)) \
& ~(SPAGE_MAP_MSK << SPAGE_MAP_SFT | PKG_SEL_MSK << PKG_SEL_SFT ) ) ; \
})

#define set_out_ctrl(lcdc_sel, dpage_sel, disp_sel) \
({ write_reg( (IPU_V_BASE + REG_CTRL), ((lcdc_sel) & LCDC_SEL_MSK  )<< LCDC_SEL_SFT \
| ((dpage_sel) & DPAGE_SEL_MSK )<< DPAGE_SEL_SFT \
| ((disp_sel) & DISP_SEL_MSK )<< DISP_SEL_SFT \
| (read_reg(IPU_V_BASE, REG_CTRL)) \
& ~(LCDC_SEL_MSK<< LCDC_SEL_SFT | DPAGE_SEL_MSK << DPAGE_SEL_SFT | DISP_SEL_MSK << DISP_SEL_SFT ) ); \
})

#define set_scale_ctrl(v_scal, h_scal) \
({ write_reg( (IPU_V_BASE + REG_CTRL), ((v_scal) & V_SCALE_MSK)<<V_SCALE_SFT \
| ((h_scal) & H_SCALE_MSK)<<H_SCALE_SFT \
| (read_reg(IPU_V_BASE, REG_CTRL)) & ~(V_SCALE_MSK<<V_SCALE_SFT | H_SCALE_MSK<<H_SCALE_SFT ) ); \
})


#define set_csc_ofset_para(chrom_oft, luma_oft) \
({ write_reg( (IPU_V_BASE + REG_CSC_OFSET_PARA ), ((chrom_oft) & CHROM_OF_MSK ) << CHROM_OF_SFT \
| ((luma_oft) & LUMA_OF_MSK ) << LUMA_OF_SFT ) ; \
})

#define sw_reset_ipu() \
({ write_reg( (IPU_V_BASE + REG_CTRL), (read_reg(IPU_V_BASE, REG_CTRL)) \
| IPU_RST_MSK<<IPU_RST_SFT); \
})
#define enable_ipu() \
({ write_reg( (IPU_V_BASE + REG_CTRL), (read_reg(IPU_V_BASE, REG_CTRL)) | 0x1); \
})
#define disable_ipu() \
({ write_reg( (IPU_V_BASE + REG_CTRL), (read_reg(IPU_V_BASE, REG_CTRL)) & ~0x1); \
})
#define run_ipu() \
({ write_reg( (IPU_V_BASE + REG_CTRL), (read_reg(IPU_V_BASE, REG_CTRL)) | 0x2); \
})
#define stop_ipu() \
({ write_reg( (IPU_V_BASE + REG_CTRL), (read_reg(IPU_V_BASE, REG_CTRL)) & ~0x2); \
})

#define polling_end_flag() \
({ (read_reg(IPU_V_BASE, REG_STATUS)) & 0x01; \
})

#define start_vlut_coef_write() \
({ write_reg( (IPU_V_BASE + VRSZ_LUT_BASE), ( 0x1<<12 ) ); \
})

#define start_hlut_coef_write() \
({ write_reg( (IPU_V_BASE + HRSZ_LUT_BASE), ( 0x01<<12 ) ); \
})

#define clear_end_flag() \
({ write_reg( (IPU_V_BASE + REG_STATUS), 0); \
})
#endif /* #if 0 */


#endif /* __JZ4750_OPS_H__ */
