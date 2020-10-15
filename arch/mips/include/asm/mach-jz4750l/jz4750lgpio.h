/*
 * linux/include/asm-mips/mach-jz4750l/jz4750lgpio.h
 *
 * JZ4750L GPIO register definition
 *
 * Copyright (C) 2008-2014 Ingenic Semiconductor Co., Ltd.
 */
#ifndef __JZ4750LGPIO_H__
#define __JZ4750LGPIO_H__

/* Define the module base addresses */
#define	GPIO_BASE	0xB0010000

/*************************************************************************
 * GPIO (General-Purpose I/O Ports)
 *
 * n = 0,1,2,3,4 (PORTA, PORTB, PORTC, PORTD, PORTE)
 *************************************************************************/
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
//	16		-		-
//	17		-		-
//	18		-		-
//	19		-		-
//	20		-		-
//	21		-		-
//	22		-		-
//	23		-		-
//	24		-		-
//	25		-		-
//	26		-		-
//	27		-		-
//	28		-		-
//	29		-		-
//	30		-		-
//	31		-		-
//
//------------------------------------------------------
// PORT 1:
//
// PIN/BIT N		FUNC0		FUNC1		FUNC2         NOTE
//	0		A0		-		-
//	1		A1		-		-
//	2		A2		-		-
//	3		A3		-		-
//	4		A4		-		-
//	5		A5		-		-
//	6		A6		-		-
//	7		A7		-		-
//	8		A8		-		-
//	9		A9		-		-
//	10		A10		-		-
//	11		A11		-		-
//	12		A12		-		-
//	13		A13		-		-
//	14		A14		-		-
//	15		-		-		-
//	16		DCS0#		-		-
//	17		RAS#		-		-
//	18		CAS#		-		-
//	19		SDWE#/BUFD#	-		-
//	20		WE0#		-		-
//	21		WE1#		-		-
//	22		-		-		-
//	23		-		-		-
//	24		CKO		-		-		Note1
//	25		CKE		-		-
//	26		-		-		-
//	27		-		-		-
//	28		-		-		-
//	29		-		-		-
//	30		-		-		-		Note2
//	31		WKUP		-		-		Note3
//
// Note1: BIT24: it is CKO when chip is reset
//
// Note2: BIT30: No corresponding pin exists for this GPIO. It is only used to
//        select the function between UART and JTAG, which share the same set of
//        pins, by using register PBSEL [30].
//        When PBSEL [30]=0, select JTAG function.
//        When PBSEL [30]=1, select UART function.
//
// Note3: PB31: it is can only be used as input and interrupt, no pull-up and pull-down.
//
//------------------------------------------------------
// PORT 2:
//
// PIN/BIT N		FUNC0		FUNC1		FUNC2		NOTE
//	0		A17		CIM_D0		-
//	1		A18		CIM_D1		-
//	2		A19		CIM_D2		-
//	3		A20		CIM_D3		-
//	4		A21		CIM_D4		-
//	5		A22		CIM_D5		-
//	6		A23		CIM_D6		DREQ
//	7		A24		CIM_D7		DACK
//	8		A15/CLE		MSC0_CLK	-
//	9		A16/ALE		MSC0_CMD	-
//	10		PWM0		I2C_SDA		-
//	11		PWM1		I2C_SCK		-
//	12		PWM2		UART_TXD	-
//	13		PWM3		UART_RXD	-
//	14		PWM4		-		-
//	15		PWM5		-		-
//	16		SSI_CLK		-		-
//	17		SSI_DT		-		-
//	18		SSI_DR		DREQ		-
//	19		SSI_CE0		DACK		-
//	20		WAIT#		-		-
//	21		CS1#		-		- 
//	22		CS2#		MSC_D3		-
//	23		CS3#		-		-
//	24		CS4#		-		-
//	25		RD#		-		-
//	26		WR#		-		-
//	27		MSC0_D2		-		-		Note4
//	28		FRE#		MSC0_D0		-
//	29		FWE#		MSC0_D1		-
//	30		-       	-		-		Note5,7
//	31		-       	-		-		Note6,7
//
// Note4: BIT27: If NAND flash is used, it should connect to NAND FRB. (NAND flash ready/busy)
//        If NAND flash is not used, it is used as general GPIO.
//
// Note5: BIT30: GPIO group C bit 30 is used as BOOT_SEL0 input during boot.
// Note6: BIT31: GPIO group C bit 31 is used as BOOT_SEL1 input during boot.
// Note7: BOOT_SEL1, BOOT_SEL0 are used to select boot source and function during the processor boot.
//
//------------------------------------------------------
// PORT 3:
//
// PIN/BIT N		FUNC0		FUNC1		FUNC2		NOTE
//	0		LCD_B2		-
//	1		LCD_B3		-
//	2		LCD_B4		-
//	3		LCD_B5		-
//	4		LCD_B6		-
//	5		LCD_B7		-
//	6		LCD_G2		-
//	7		LCD_G3		-
//	8		LCD_G4		-
//	9		LCD_G5		-
//	10		LCD_G6		-
//	11		LCD_G7		-
//	12		LCD_R2		-
//	13		LCD_R3		-
//	14		LCD_R4		-
//	15		LCD_R5		-
//	16		LCD_R6		-
//	17		LCD_R7		-
//	18		LCD_PCLK	-
//	19		LCD_HSYNC	-
//	20		LCD_VSYNC	-
//	21		LCD_DE		-
//	22		LCD_CLS		LCD_R1		CIM_MCLK
//	23		LCD_SPL		LCD_G0		CIM_HSYNC
//	24		LCD_PS		LCD_G1		CIM_PCLK
//	25		LCD_REV		LCD_B1		CIM_VSYNC
//	26		MSC1_CLK	-
//	27		MSC1_CMD	-
//	28		MSC1_D0		-
//	29		MSC1_D1		-
//	30		MSC1_D2		-
//	31		MSC1_D3		-
//
//////////////////////////////////////////////////////////

/*----------------------------------------------------------------
 * p is the port number (0,1,2,3)
 * o is the pin offset (0-31) inside the port
 * n is the absolute number of a pin (0-127), regardless of the port
 */
#define GPIO_BASEA	GPIO_BASE + (0)*0x100
#define GPIO_BASEB	GPIO_BASE + (1)*0x100
#define GPIO_BASEC	GPIO_BASE + (2)*0x100
#define GPIO_BASED	GPIO_BASE + (3)*0x100

#define GPIO_PA(x)	(32 * 0 + (x))
#define GPIO_PB(x)	(32 * 1 + (x))
#define GPIO_PC(x)	(32 * 2 + (x))
#define GPIO_PD(x)	(32 * 3 + (x))

#define GPIO_PORT_NUM	4
#define MAX_GPIO_NUM	128
#define GPIO_WAKEUP     (32 * 1 + 31)

#define GPIO_PXPIN(n)	(GPIO_BASE + (0x00 + (n)*0x100)) /* PIN Level Register */
#define GPIO_PXDAT(n)	(GPIO_BASE + (0x10 + (n)*0x100)) /* Port Data Register */
#define GPIO_PXDATS(n)	(GPIO_BASE + (0x14 + (n)*0x100)) /* Port Data Set Register */
#define GPIO_PXDATC(n)	(GPIO_BASE + (0x18 + (n)*0x100)) /* Port Data Clear Register */
#define GPIO_PXIM(n)	(GPIO_BASE + (0x20 + (n)*0x100)) /* Interrupt Mask Register */
#define GPIO_PXIMS(n)	(GPIO_BASE + (0x24 + (n)*0x100)) /* Interrupt Mask Set Reg */
#define GPIO_PXIMC(n)	(GPIO_BASE + (0x28 + (n)*0x100)) /* Interrupt Mask Clear Reg */
#define GPIO_PXPE(n)	(GPIO_BASE + (0x30 + (n)*0x100)) /* Pull Enable Register */
#define GPIO_PXPES(n)	(GPIO_BASE + (0x34 + (n)*0x100)) /* Pull Enable Set Reg. */
#define GPIO_PXPEC(n)	(GPIO_BASE + (0x38 + (n)*0x100)) /* Pull Enable Clear Reg. */
#define GPIO_PXFUN(n)	(GPIO_BASE + (0x40 + (n)*0x100)) /* Function Register */
#define GPIO_PXFUNS(n)	(GPIO_BASE + (0x44 + (n)*0x100)) /* Function Set Register */
#define GPIO_PXFUNC(n)	(GPIO_BASE + (0x48 + (n)*0x100)) /* Function Clear Register */
#define GPIO_PXSEL(n)	(GPIO_BASE + (0x50 + (n)*0x100)) /* Select Register */
#define GPIO_PXSELS(n)	(GPIO_BASE + (0x54 + (n)*0x100)) /* Select Set Register */
#define GPIO_PXSELC(n)	(GPIO_BASE + (0x58 + (n)*0x100)) /* Select Clear Register */
#define GPIO_PXDIR(n)	(GPIO_BASE + (0x60 + (n)*0x100)) /* Direction Register */
#define GPIO_PXDIRS(n)	(GPIO_BASE + (0x64 + (n)*0x100)) /* Direction Set Register */
#define GPIO_PXDIRC(n)	(GPIO_BASE + (0x68 + (n)*0x100)) /* Direction Clear Register */
#define GPIO_PXTRG(n)	(GPIO_BASE + (0x70 + (n)*0x100)) /* Trigger Register */
#define GPIO_PXTRGS(n)	(GPIO_BASE + (0x74 + (n)*0x100)) /* Trigger Set Register */
#define GPIO_PXTRGC(n)	(GPIO_BASE + (0x78 + (n)*0x100)) /* Trigger Set Register */
#define GPIO_PXFLG(n)	(GPIO_BASE + (0x80 + (n)*0x100)) /* Port Flag Register */
#define GPIO_PXFLGC(n)	(GPIO_BASE + (0x14 + (n)*0x100)) /* Port Flag Clear Register */

#define REG_GPIO_PXPIN(n)	REG32(GPIO_PXPIN((n)))  /* PIN level */
#define REG_GPIO_PXDAT(n)	REG32(GPIO_PXDAT((n)))  /* 1: interrupt pending */
#define REG_GPIO_PXDATS(n)	REG32(GPIO_PXDATS((n)))
#define REG_GPIO_PXDATC(n)	REG32(GPIO_PXDATC((n)))
#define REG_GPIO_PXIM(n)	REG32(GPIO_PXIM((n)))   /* 1: mask pin interrupt */
#define REG_GPIO_PXIMS(n)	REG32(GPIO_PXIMS((n)))
#define REG_GPIO_PXIMC(n)	REG32(GPIO_PXIMC((n)))
#define REG_GPIO_PXPE(n)	REG32(GPIO_PXPE((n)))   /* 1: disable pull up/down */
#define REG_GPIO_PXPES(n)	REG32(GPIO_PXPES((n)))
#define REG_GPIO_PXPEC(n)	REG32(GPIO_PXPEC((n)))
#define REG_GPIO_PXFUN(n)	REG32(GPIO_PXFUN((n)))  /* 0:GPIO or intr, 1:FUNC */
#define REG_GPIO_PXFUNS(n)	REG32(GPIO_PXFUNS((n)))
#define REG_GPIO_PXFUNC(n)	REG32(GPIO_PXFUNC((n)))
#define REG_GPIO_PXSEL(n)	REG32(GPIO_PXSEL((n))) /* 0:GPIO/Fun0,1:intr/fun1*/
#define REG_GPIO_PXSELS(n)	REG32(GPIO_PXSELS((n)))
#define REG_GPIO_PXSELC(n)	REG32(GPIO_PXSELC((n)))
#define REG_GPIO_PXDIR(n)	REG32(GPIO_PXDIR((n))) /* 0:input/low-level-trig/falling-edge-trig, 1:output/high-level-trig/rising-edge-trig */
#define REG_GPIO_PXDIRS(n)	REG32(GPIO_PXDIRS((n)))
#define REG_GPIO_PXDIRC(n)	REG32(GPIO_PXDIRC((n)))
#define REG_GPIO_PXTRG(n)	REG32(GPIO_PXTRG((n))) /* 0:level-trigger, 1:edge-trigger */
#define REG_GPIO_PXTRGS(n)	REG32(GPIO_PXTRGS((n)))
#define REG_GPIO_PXTRGC(n)	REG32(GPIO_PXTRGC((n)))
#define REG_GPIO_PXFLG(n)	REG32(GPIO_PXFLG((n))) /* interrupt flag */
#define REG_GPIO_PXFLGC(n)	REG32(GPIO_PXFLGC((n))) /* interrupt flag */

/*
 * GPIO Operations
 */
#ifndef __MIPS_ASSEMBLER
//-------------------------------------------
// Function Pins Mode
#define __gpio_as_func0(n)				\
	do {						\
		unsigned int p, o;			\
		p = (n) / 32;				\
		o = (n) % 32;				\
		REG_GPIO_PXFUNS(p) = (1 << o);		\
		REG_GPIO_PXTRGC(p) = (1 << o);		\
		REG_GPIO_PXSELC(p) = (1 << o);		\
	} while (0)

#define __gpio_as_func1(n)				\
	do {						\
		unsigned int p, o;			\
		p = (n) / 32;				\
		o = (n) % 32;				\
		REG_GPIO_PXFUNS(p) = (1 << o);		\
		REG_GPIO_PXTRGC(p) = (1 << o);		\
		REG_GPIO_PXSELS(p) = (1 << o);		\
	} while (0)

#define __gpio_as_func2(n)				\
	do {						\
		unsigned int p, o;			\
		p = (n) / 32;				\
		o = (n) % 32;				\
		REG_GPIO_PXFUNS(p) = (1 << o);		\
		REG_GPIO_PXTRGS(p) = (1 << o);		\
		REG_GPIO_PXSELC(p) = (1 << o);		\
	} while (0)


/*
 * D0 ~ D15, A0 ~ A14, DCS0#, RAS#, CAS#, 
 * RDWE#, WE0#, WE1#, CKO#, CKE#
 */
#define __gpio_as_sdram_16bit()				\
	do {						\
		REG_GPIO_PXFUNS(0) = 0x0000ffff;	\
		REG_GPIO_PXSELC(0) = 0x0000ffff;	\
		REG_GPIO_PXPES(0) = 0x0000ffff;		\
		REG_GPIO_PXFUNS(1) = 0x03ff7fff;	\
		REG_GPIO_PXSELC(1) = 0x03ff7fff;	\
		REG_GPIO_PXPES(1) = 0x03ff7fff;		\
	} while (0)

/*
 * D0 ~ D7, CS1#, CLE, ALE, FRE#, FWE#, FRB#, RDWE#/BUFD#
 * @n: chip select number(1 ~ 4)
 */
#define __gpio_as_nand_8bit(n)							\
	do {		              						\
		/* 32/16-bit data bus */					\
		REG_GPIO_PXFUNS(0) = 0x000000ff; /* D0~D7 */			\
		REG_GPIO_PXSELC(0) = 0x000000ff;				\
		REG_GPIO_PXPES(0) = 0x000000ff;					\
		REG_GPIO_PXFUNS(2) = 0x00000300; /* CLE(A15), ALE(A16) */	\
		REG_GPIO_PXSELC(2) = 0x00000300;				\
		REG_GPIO_PXPES(2) = 0x0000300;					\
		\
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
#define __gpio_as_nor_8bit(n)							\
	do {								        \
		/* 32/16-bit data bus */					\
		REG_GPIO_PXFUNS(0) = 0x000000ff;				\
		REG_GPIO_PXSELC(0) = 0x000000ff;				\
		REG_GPIO_PXPES(0) = 0x000000ff;					\
		\
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
#define __gpio_as_nor_16bit(n)							\
	do {	               							\
		/* 32/16-bit data normal order */				\
		REG_GPIO_PXFUNS(0) = 0x0000ffff;				\
		REG_GPIO_PXSELC(0) = 0x0000ffff;				\
		REG_GPIO_PXPES(0) = 0x0000ffff;					\
		\
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
#define __gpio_as_uart0()				\
	do {						\
		REG_GPIO_PXFUNS(2) = 0x00003000;	\
		REG_GPIO_PXTRGC(2) = 0x00003000;	\
		REG_GPIO_PXSELS(2) = 0x00003000;	\
		REG_GPIO_PXPES(2) = 0x00003000;		\
	} while (0)

/*
 * UART0_TxD, UART0_RxD, UART0_CTS, UART0_RTS
 */
#define __gpio_as_uart0_ctsrts()			\
	do {						\
		REG_GPIO_PXFUNS(3) = 0xf0000000;	\
		REG_GPIO_PXSELC(3) = 0xf0000000;	\
		REG_GPIO_PXPES(3) = 0xf0000000;		\
	} while (0)

/*
 * UART1_TxD, UART1_RxD
 */
#define __gpio_as_uart1()				\
	do {						\
		REG_GPIO_PXTRGC(4) = 0x02800000;	\
		REG_GPIO_PXFUNS(4) = 0x02800000;	\
		REG_GPIO_PXSELS(4) = 0x02800000;	\
		REG_GPIO_PXPES(4) = 0x02800000;		\
	} while (0)

/*
 * TSCLK, TSSTR, TSFRM, TSFAIL, TSDI0~7
 */
#define __gpio_as_tssi()				\
	do {						\
		REG_GPIO_PXFUNS(2) = 0x0000ff00;	\
		REG_GPIO_PXSELS(2) = 0x0000ff00;	\
		REG_GPIO_PXPES(2) = 0x0000ff00;		\
		REG_GPIO_PXFUNS(3) = 0xf0000000;	\
		REG_GPIO_PXSELS(3) = 0xf0000000;	\
		REG_GPIO_PXPES(3) = 0xf0000000;		\
	} while (0)

/*
 * LCD_D0~LCD_D7, LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_8bit()				\
	do {						\
		REG_GPIO_PXFUNS(3) = 0x003c00ff;	\
		REG_GPIO_PXTRGC(3) = 0x003c00ff;	\
		REG_GPIO_PXSELC(3) = 0x003c00ff;	\
		REG_GPIO_PXPES(3) = 0x003c00ff;		\
	} while (0)

/*
 * LCD_D0~LCD_D15, LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_16bit()				\
	do {						\
		REG_GPIO_PXFUNS(3) = 0x003cffff;	\
		REG_GPIO_PXTRGC(3) = 0x003cffff;	\
		REG_GPIO_PXSELC(3) = 0x003cffff;	\
		REG_GPIO_PXPES(3) = 0x003cffff;		\
	} while (0)

/*
 * LCD_R2~LCD_R7, LCD_G2~LCD_G7, LCD_B2~LCD_B7,
 * LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_18bit()				\
	do {						\
		REG_GPIO_PXFUNS(3) = 0x003fffff;	\
		REG_GPIO_PXTRGC(3) = 0x003fffff;	\
		REG_GPIO_PXSELC(3) = 0x003fffff;	\
		REG_GPIO_PXPES(3) = 0x003fffff;		\
	} while (0)

/*
 * LCD_D0~LCD_D17, LCD_D_R1, LCD_D_G0, LCD_D_G1, LCD_D_B1,
 * LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_24bit()				\
	do {						\
		REG_GPIO_PXFUNS(3) = 0x003fffff;	\
		REG_GPIO_PXTRGC(3) = 0x003fffff;	\
		REG_GPIO_PXSELC(3) = 0x003fffff;	\
		REG_GPIO_PXPES(3)  = 0x003fffff;	\
		REG_GPIO_PXFUNS(3) = 0x03c00000;	\
		REG_GPIO_PXTRGC(3) = 0x03c00000;	\
		REG_GPIO_PXSELS(3) = 0x03c00000;	\
		REG_GPIO_PXPES(3)  = 0x03c00000;	\
	} while (0)

/*
 *  LCD_CLS, LCD_SPL, LCD_PS, LCD_REV
 */
#define __gpio_as_lcd_special()				\
	do {						\
		REG_GPIO_PXFUNS(3) = 0x03C00000;	\
		REG_GPIO_PXTRGC(3) = 0x03C00000;	\
		REG_GPIO_PXSELC(3) = 0x03C00000;	\
		REG_GPIO_PXPES(3)  = 0x03C00000;	\
	} while (0)

/*
 * CIM_D0~CIM_D7, CIM_MCLK, CIM_PCLK, CIM_VSYNC, CIM_HSYNC
 */
#define __gpio_as_cim()					\
	do {						\
		REG_GPIO_PXFUNS(2) = 0x00000ff;		\
		REG_GPIO_PXTRGS(2) = 0x00000ff;		\
		REG_GPIO_PXSELC(2) = 0x00000ff;		\
		REG_GPIO_PXPES(2)  = 0x00000ff;		\
		REG_GPIO_PXFUNS(3) = 0x03c00000;	\
		REG_GPIO_PXTRGS(3) = 0x03c00000;	\
		REG_GPIO_PXSELC(3) = 0x03c00000;	\
		REG_GPIO_PXPES(3)  = 0x03c00000;	\
	} while (0)


/* 
 * SDATO, SDATI, BCLK, SYNC, SCLK_RSTN(gpio sepc) or
 * SDATA_OUT, SDATA_IN, BIT_CLK, SYNC, SCLK_RESET(aic spec)
 */
#define __gpio_as_aic()					\
	do {						\
		REG_GPIO_PXFUNS(4) = 0x16c00000;	\
		REG_GPIO_PXTRGC(4) = 0x02c00000;	\
		REG_GPIO_PXTRGS(4) = 0x14000000;	\
		REG_GPIO_PXSELC(4) = 0x14c00000;	\
		REG_GPIO_PXSELS(4) = 0x02000000;	\
		REG_GPIO_PXPES(4)  = 0x16c00000;	\
	} while (0)

/*
 * MSC0_CMD, MSC0_CLK, MSC0_D0 ~ MSC0_D3
 */
#define __gpio_as_msc0_4bit()				\
	do {						\
		REG_GPIO_PXFUNS(2) = 0x38400300;	\
		REG_GPIO_PXTRGC(2) = 0x38400300;	\
		REG_GPIO_PXSELS(2) = 0x30400300;	\
		REG_GPIO_PXSELC(2) = 0x08000000;	\
		REG_GPIO_PXPES(2)  = 0x38400300;	\
	} while (0)


/*
 * MSC1_CMD, MSC1_CLK, MSC1_D0 ~ MSC1_D3
 */
#define __gpio_as_msc1_4bit()				\
	do {						\
		REG_GPIO_PXFUNS(3) = 0xfc000000;	\
		REG_GPIO_PXTRGC(3) = 0xfc000000;	\
		REG_GPIO_PXSELC(3) = 0xfc000000;	\
		REG_GPIO_PXPES(3)  = 0xfc000000;	\
	} while (0)

#define __gpio_as_msc 	__gpio_as_msc0_4bit /* default as msc0 4bit */
#define __gpio_as_msc0 	__gpio_as_msc0_4bit /* msc0 default as 4bit */
#define __gpio_as_msc1 	__gpio_as_msc1_4bit /* msc1 only support 4bit */

/*
 * SSI_CE0, SSI_CLK, SSI_DT, SSI_DR
 */
#define __gpio_as_ssi()					\
	do {						\
		REG_GPIO_PXFUNS(2) = 0x000f0000;	\
		REG_GPIO_PXTRGC(2) = 0x000f0000;	\
		REG_GPIO_PXSELC(2) = 0x000f0000;	\
		REG_GPIO_PXPES(2)  = 0x000f0000;	\
	} while (0)

/*
 * SSI_CLK, SSI_DT, SSI_DR
 */
#define __gpio_as_ssi_without_ce()			\
	do {						\
		REG_GPIO_PXFUNS(2) = 0x00070000;	\
		REG_GPIO_PXTRGC(2) = 0x00070000;	\
		REG_GPIO_PXSELC(2) = 0x00070000;	\
		REG_GPIO_PXPES(2)  = 0x00070000;	\
	} while (0)

/*
 * I2C_SCK, I2C_SDA
 */
#define __gpio_as_i2c()					\
	do {						\
		REG_GPIO_PXFUNS(2) = 0x00000c00;	\
		REG_GPIO_PXTRGC(2) = 0x00000c00;	\
		REG_GPIO_PXSELS(2) = 0x00000c00;	\
		REG_GPIO_PXPES(2)  = 0x00000c00;	\
	} while (0)

/*
 * PWM0
 */
#define __gpio_as_pwm0()				\
	do {						\
		REG_GPIO_PXFUNS(2) = 0x00000400;	\
		REG_GPIO_PXTRGC(2) = 0x00000400;	\
		REG_GPIO_PXSELC(2) = 0x00000400;	\
		REG_GPIO_PXPES(2) = 0x00000400;		\
	} while (0)

/*
 * PWM1
 */
#define __gpio_as_pwm1()				\
	do {						\
		REG_GPIO_PXFUNS(2) = 0x00000800;	\
		REG_GPIO_PXTRGC(2) = 0x00000800;	\
		REG_GPIO_PXSELC(2) = 0x00000800;	\
		REG_GPIO_PXPES(2) = 0x00000800;		\
	} while (0)

/*
 * PWM2
 */
#define __gpio_as_pwm2()				\
	do {						\
		REG_GPIO_PXFUNS(2) = 0x00001000;	\
		REG_GPIO_PXTRGC(2) = 0x00001000;	\
		REG_GPIO_PXSELC(2) = 0x00001000;	\
		REG_GPIO_PXPES(2) = 0x00001000;		\
	} while (0)

/*
 * PWM3
 */
#define __gpio_as_pwm3()				\
	do {						\
		REG_GPIO_PXFUNS(2) = 0x00002000;	\
		REG_GPIO_PXTRGC(2) = 0x00002000;	\
		REG_GPIO_PXSELC(2) = 0x00002000;	\
		REG_GPIO_PXPES(2) = 0x00002000;		\
	} while (0)

/*
 * PWM4
 */
#define __gpio_as_pwm4()				\
	do {						\
		REG_GPIO_PXFUNS(2) = 0x00004000;	\
		REG_GPIO_PXTRGC(2) = 0x00004000;	\
		REG_GPIO_PXSELC(2) = 0x00004000;	\
		REG_GPIO_PXPES(2) = 0x00004000;		\
	} while (0)

/*
 * PWM5
 */
#define __gpio_as_pwm5()				\
	do {						\
		REG_GPIO_PXFUNS(2) = 0x00008000;	\
		REG_GPIO_PXTRGC(2) = 0x00008000;	\
		REG_GPIO_PXSELC(2) = 0x00008000;	\
		REG_GPIO_PXPES(2) = 0x00008000;		\
	} while (0)

/*
 * n = 0 ~ 5
 */
#define __gpio_as_pwm(n)	__gpio_as_pwm##n()

/*
 * DREQ
 */
#define __gpio_as_dreq()				\
	do {						\
		REG_GPIO_PXFUNS(2) = 0x00000040;	\
		REG_GPIO_PXTRGS(2) = 0x00000040;	\
		REG_GPIO_PXSELC(2) = 0x00000040;	\
		REG_GPIO_PXPES(2) = 0x00000040;		\
	} while (0)

/*
 * DACK
 */
#define __gpio_as_dack()				\
	do {						\
		REG_GPIO_PXFUNS(2) = 0x00000080;	\
		REG_GPIO_PXTRGS(2) = 0x00000080;	\
		REG_GPIO_PXSELC(2) = 0x00000080;	\
		REG_GPIO_PXPES(2) = 0x00000080;		\
	} while (0)

/*
 * GPIO or Interrupt Mode
 */
#define __gpio_get_port(p)	(REG_GPIO_PXPIN(p))

#define __gpio_port_as_output(p, o)			\
	do {						\
		REG_GPIO_PXFUNC(p) = (1 << (o));	\
		REG_GPIO_PXSELC(p) = (1 << (o));	\
		REG_GPIO_PXDIRS(p) = (1 << (o));	\
	} while (0)

#define __gpio_port_as_input(p, o)			\
	do {						\
		REG_GPIO_PXFUNC(p) = (1 << (o));	\
		REG_GPIO_PXSELC(p) = (1 << (o));	\
		REG_GPIO_PXDIRC(p) = (1 << (o));	\
	} while (0)

#define __gpio_as_output(n)				\
	do {						\
		unsigned int p, o;			\
		p = (n) / 32;				\
		o = (n) % 32;				\
		__gpio_port_as_output(p, o);		\
	} while (0)

#define __gpio_as_input(n)				\
	do {						\
		unsigned int p, o;			\
		p = (n) / 32;				\
		o = (n) % 32;				\
		__gpio_port_as_input(p, o);		\
	} while (0)

#define __gpio_set_pin(n)				\
	do {						\
		unsigned int p, o;			\
		p = (n) / 32;				\
		o = (n) % 32;				\
		REG_GPIO_PXDATS(p) = (1 << o);		\
	} while (0)

#define __gpio_clear_pin(n)				\
	do {						\
		unsigned int p, o;			\
		p = (n) / 32;				\
		o = (n) % 32;				\
		REG_GPIO_PXDATC(p) = (1 << o);		\
	} while (0)

#define __gpio_as_output1(n)				\
	do {						\
		unsigned int p, o;			\
		p = (n) / 32;				\
		o = (n) % 32;				\
		REG_GPIO_PXDATS(p) = (1 << o);		\
		__gpio_port_as_output(p, o);		\
	} while (0)

#define __gpio_as_output0(n)				\
	do {						\
		unsigned int p, o;			\
		p = (n) / 32;				\
		o = (n) % 32;				\
		REG_GPIO_PXDATC(p) = (1 << o);		\
		__gpio_port_as_output(p, o);		\
	} while (0)

#define __gpio_get_pin(n)				\
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

#define __gpio_as_irq_high_level(n)			\
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

#define __gpio_as_irq_low_level(n)			\
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

#define __gpio_as_irq_rise_edge(n)			\
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

#define __gpio_as_irq_fall_edge(n)			\
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

#define __gpio_mask_irq(n)				\
	do {						\
		unsigned int p, o;			\
		p = (n) / 32;				\
		o = (n) % 32;				\
		REG_GPIO_PXIMS(p) = (1 << o);		\
	} while (0)

#define __gpio_unmask_irq(n)				\
	do {						\
		unsigned int p, o;			\
		p = (n) / 32;				\
		o = (n) % 32;				\
		REG_GPIO_PXIMC(p) = (1 << o);		\
	} while (0)

#define __gpio_ack_irq(n)				\
	do {						\
		unsigned int p, o;			\
		p = (n) / 32;				\
		o = (n) % 32;				\
		REG_GPIO_PXFLGC(p) = (1 << o);		\
	} while (0)

#define __gpio_get_irq()				\
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

#define __gpio_group_irq(n)				\
	({						\
		register int tmp, i;			\
		tmp = REG_GPIO_PXFLG((n));		\
		for (i=31;i>=0;i--)			\
			if (tmp & (1 << i))		\
				break;			\
		i;					\
	})

#define __gpio_enable_pull(n)				\
	do {						\
		unsigned int p, o;			\
		p = (n) / 32;				\
		o = (n) % 32;				\
		REG_GPIO_PXPEC(p) = (1 << o);		\
	} while (0)

#define __gpio_disable_pull(n)				\
	do {						\
		unsigned int p, o;			\
		p = (n) / 32;				\
		o = (n) % 32;				\
		REG_GPIO_PXPES(p) = (1 << o);		\
	} while (0)

#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4750LGPIO_H__ */
