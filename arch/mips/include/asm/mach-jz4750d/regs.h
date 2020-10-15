/*
 * linux/include/asm-mips/mach-jz4750d/regs.h
 *
 * JZ4750D register definition.
 *
 * Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __JZ4750D_REGS_H__
#define __JZ4750D_REGS_H__

#if defined(__ASSEMBLY__) || defined(__LANGUAGE_ASSEMBLY)
#define REG8(addr)	(addr)
#define REG16(addr)	(addr)
#define REG32(addr)	(addr)
#else
#define REG8(addr)	*((volatile unsigned char *)(addr))
#define REG16(addr)	*((volatile unsigned short *)(addr))
#define REG32(addr)	*((volatile unsigned int *)(addr))
#endif

/*
 * Define the module base addresses
 */
#define	CPM_BASE	0xB0000000
#define	INTC_BASE	0xB0001000
#define	TCU_BASE	0xB0002000
#define	WDT_BASE	0xB0002000
#define	RTC_BASE	0xB0003000
#define	GPIO_BASE	0xB0010000
#define	AIC_BASE	0xB0020000
#define	ICDC_BASE	0xB0020000
#define	MSC_BASE	0xB0021000
#define	UART0_BASE	0xB0030000
#define	UART1_BASE	0xB0031000
#define	UART2_BASE	0xB0032000
#define	UART3_BASE	0xB0033000
#define	I2C_BASE	0xB0042000
#define	SSI_BASE	0xB0043000
#define	SADC_BASE	0xB0070000
#define PCM_BASE        0xB0071000
#define	EMC_BASE	0xB3010000
#define	DMAC_BASE	0xB3020000
#define	UHC_BASE	0xB3030000
#define	UDC_BASE	0xB3040000
#define	LCD_BASE	0xB3050000
#define	SLCD_BASE	0xB3050000
#define	TVE_BASE	0xB3050100
#define	CIM_BASE	0xB3060000
#define IPU_BASE	0xB3080000
#define ME_BASE		0xB3090000
#define MC_BASE		0xB30A0000
#define BCH_BASE	0xB30D0000
#define	ETH_BASE	0xB3100000
#define	TCSM_BASE	0xF4000000
#define OWI_BASE	0XB0072000
#define OTP_BASE	0xB3012000
#define TSSI_BASE	0xB0073000

/*************************************************************************
 * INTC (Interrupt Controller)
 *************************************************************************/
#define INTC_ISR	(INTC_BASE + 0x00)
#define INTC_IMR	(INTC_BASE + 0x04)
#define INTC_IMSR	(INTC_BASE + 0x08)
#define INTC_IMCR	(INTC_BASE + 0x0c)
#define INTC_IPR	(INTC_BASE + 0x10)
#define INTC_ISSR	(INTC_BASE + 0x18)  /* Interrupt Controller Source Set Register */
#define INTC_ISCR	(INTC_BASE + 0x1c)  /* Interrupt Controller Source Clear Register */

#define REG_INTC_ISR	REG32(INTC_ISR)
#define REG_INTC_IMR	REG32(INTC_IMR)
#define REG_INTC_IMSR	REG32(INTC_IMSR)
#define REG_INTC_IMCR	REG32(INTC_IMCR)
#define REG_INTC_IPR	REG32(INTC_IPR)
#define REG_INTC_ISSR   REG32(INTC_ISSR)
#define REG_INTC_ISCR   REG32(INTC_ISCR)

// 1st-level interrupts
#define IRQ_ETH		0
#define IRQ_SFT		4
#define IRQ_I2C		5
#define IRQ_RTC		6
#define IRQ_UART2	7
#define IRQ_UART1	8
#define IRQ_UART0	9
#define IRQ_AIC 	10
#define IRQ_GPIO5	11
#define IRQ_GPIO4	12
#define IRQ_GPIO3	13
#define IRQ_GPIO2	14
#define IRQ_GPIO1	15
#define IRQ_GPIO0	16
#define IRQ_BCH		17
#define IRQ_SADC	18
#define IRQ_CIM		19
#define IRQ_TSSI	20
#define IRQ_TCU2	21
#define IRQ_TCU1	22
#define IRQ_TCU0	23
#define IRQ_MSC1	24
#define IRQ_MSC0	25
#define IRQ_SSI   	26
#define IRQ_UDC		27
#define IRQ_DMAC1	28
#define IRQ_DMAC0	29
#define IRQ_IPU		30
#define IRQ_LCD		31

// 2nd-level interrupts
#define IRQ_DMA_0	32  /* 32 to 43 for DMAC0's 0-5  and DMAC1's 0-5 */
#define IRQ_GPIO_0	48  /* 48 to 240 for GPIO pin 0 to 192 */

#define NUM_DMA         MAX_DMA_NUM	/* 12 */
#define NUM_GPIO        MAX_GPIO_NUM	/* GPIO NUM: 192, Jz4750D real num GPIO 178 */


/*************************************************************************
 * RTC
 *************************************************************************/
#define RTC_RCR		(RTC_BASE + 0x00) /* RTC Control Register */
#define RTC_RSR		(RTC_BASE + 0x04) /* RTC Second Register */
#define RTC_RSAR	(RTC_BASE + 0x08) /* RTC Second Alarm Register */
#define RTC_RGR		(RTC_BASE + 0x0c) /* RTC Regulator Register */

#define RTC_HCR		(RTC_BASE + 0x20) /* Hibernate Control Register */
#define RTC_HWFCR	(RTC_BASE + 0x24) /* Hibernate Wakeup Filter Counter Reg */
#define RTC_HRCR	(RTC_BASE + 0x28) /* Hibernate Reset Counter Register */
#define RTC_HWCR	(RTC_BASE + 0x2c) /* Hibernate Wakeup Control Register */
#define RTC_HWRSR	(RTC_BASE + 0x30) /* Hibernate Wakeup Status Register */
#define RTC_HSPR	(RTC_BASE + 0x34) /* Hibernate Scratch Pattern Register */

#define REG_RTC_RCR	REG32(RTC_RCR)
#define REG_RTC_RSR	REG32(RTC_RSR)
#define REG_RTC_RSAR	REG32(RTC_RSAR)
#define REG_RTC_RGR	REG32(RTC_RGR)
#define REG_RTC_HCR	REG32(RTC_HCR)
#define REG_RTC_HWFCR	REG32(RTC_HWFCR)
#define REG_RTC_HRCR	REG32(RTC_HRCR)
#define REG_RTC_HWCR	REG32(RTC_HWCR)
#define REG_RTC_HWRSR	REG32(RTC_HWRSR)
#define REG_RTC_HSPR	REG32(RTC_HSPR)

/* RTC Control Register */
#define RTC_RCR_WRDY_BIT 7
#define RTC_RCR_WRDY	(1 << 7)  /* Write Ready Flag */
#define RTC_RCR_1HZ_BIT	6
#define RTC_RCR_1HZ	(1 << RTC_RCR_1HZ_BIT)  /* 1Hz Flag */
#define RTC_RCR_1HZIE	(1 << 5)  /* 1Hz Interrupt Enable */
#define RTC_RCR_AF_BIT	4
#define RTC_RCR_AF	(1 << RTC_RCR_AF_BIT)  /* Alarm Flag */
#define RTC_RCR_AIE	(1 << 3)  /* Alarm Interrupt Enable */
#define RTC_RCR_AE	(1 << 2)  /* Alarm Enable */
#define RTC_RCR_RTCE	(1 << 0)  /* RTC Enable */

/* RTC Regulator Register */
#define RTC_RGR_LOCK		(1 << 31) /* Lock Bit */
#define RTC_RGR_ADJC_BIT	16
#define RTC_RGR_ADJC_MASK	(0x3ff << RTC_RGR_ADJC_BIT)
#define RTC_RGR_NC1HZ_BIT	0
#define RTC_RGR_NC1HZ_MASK	(0xffff << RTC_RGR_NC1HZ_BIT)

/* Hibernate Control Register */
#define RTC_HCR_PD		(1 << 0)  /* Power Down */

/* Hibernate Wakeup Filter Counter Register */
#define RTC_HWFCR_BIT		5
#define RTC_HWFCR_MASK		(0x7ff << RTC_HWFCR_BIT)

/* Hibernate Reset Counter Register */
#define RTC_HRCR_BIT		5
#define RTC_HRCR_MASK		(0x7f << RTC_HRCR_BIT)

/* Hibernate Wakeup Control Register */
#define RTC_HWCR_EALM		(1 << 0)  /* RTC alarm wakeup enable */

/* Hibernate Wakeup Status Register */
#define RTC_HWRSR_HR		(1 << 5)  /* Hibernate reset */
#define RTC_HWRSR_PPR		(1 << 4)  /* PPR reset */
#define RTC_HWRSR_PIN		(1 << 1)  /* Wakeup pin status bit */
#define RTC_HWRSR_ALM		(1 << 0)  /* RTC alarm status bit */


/*************************************************************************
 * CPM (Clock reset and Power control Management)
 *************************************************************************/
#define CPM_CPCCR	(CPM_BASE+0x00)
#define CPM_CPPCR	(CPM_BASE+0x10)
#define CPM_CPPSR	(CPM_BASE+0x14) /* PLL Switch and Status Register */
#define CPM_I2SCDR	(CPM_BASE+0x60)
#define CPM_LPCDR	(CPM_BASE+0x64)
#define CPM_MSCCDR(n)	(CPM_BASE+0x68) /* MSC0(n=0) or MSC1(n=1) device clock divider Register */
#define CPM_UHCCDR	(CPM_BASE+0x6C)
#define CPM_SSICDR	(CPM_BASE+0x74)
#define CPM_PCMCDR	(CPM_BASE+0x7C) /* PCM device clock divider Register */

#define CPM_LCR		(CPM_BASE+0x04)
#define CPM_CLKGR	(CPM_BASE+0x20)
#define CPM_OPCR	(CPM_BASE+0x24) /* Oscillator and Power Control Register */

#define CPM_RSR		(CPM_BASE+0x08)

#define REG_CPM_CPCCR   	REG32(CPM_CPCCR)
#define REG_CPM_CPPCR    	REG32(CPM_CPPCR)
#define REG_CPM_CPPSR	        REG32(CPM_CPPSR)
#define REG_CPM_I2SCDR  	REG32(CPM_I2SCDR)
#define REG_CPM_LPCDR   	REG32(CPM_LPCDR)
#define REG_CPM_MSCCDR(n)	REG32(CPM_MSCCDR(n))
#define REG_CPM_UHCCDR   	REG32(CPM_UHCCDR)
#define REG_CPM_SSICDR  	REG32(CPM_SSICDR)
#define REG_CPM_PCMCDR          REG32(CPM_PCMCDR)

#define REG_CPM_LCR	REG32(CPM_LCR)
#define REG_CPM_CLKGR	REG32(CPM_CLKGR)
#define REG_CPM_OPCR	REG32(CPM_OPCR)

#define REG_CPM_RSR	REG32(CPM_RSR)

/* Clock Control Register */
#define CPM_CPCCR_I2CS		(1 << 31)
#define CPM_CPCCR_ECS   	(1 << 30) /* Select the between EXCLK and EXCLK/2 output */
#define CPM_CPCCR_UCS		(1 << 29)
#define CPM_CPCCR_UDIV_BIT	23
#define CPM_CPCCR_UDIV_MASK	(0x3f << CPM_CPCCR_UDIV_BIT)
#define CPM_CPCCR_CE		(1 << 22)
#define CPM_CPCCR_PCS		(1 << 21)
#define CPM_CPCCR_H1DIV_BIT	16
#define CPM_CPCCR_H1DIV_MASK	(0x1f << CPM_CPCCR_H1DIV_BIT)
#define CPM_CPCCR_MDIV_BIT	12
#define CPM_CPCCR_MDIV_MASK	(0x0f << CPM_CPCCR_MDIV_BIT)
#define CPM_CPCCR_PDIV_BIT	8
#define CPM_CPCCR_PDIV_MASK	(0x0f << CPM_CPCCR_PDIV_BIT)
#define CPM_CPCCR_HDIV_BIT	4
#define CPM_CPCCR_HDIV_MASK	(0x0f << CPM_CPCCR_HDIV_BIT)
#define CPM_CPCCR_CDIV_BIT	0
#define CPM_CPCCR_CDIV_MASK	(0x0f << CPM_CPCCR_CDIV_BIT)

/* PLL Switch and Status Register */
#define CPM_CPPSR_PLLOFF        (1<<31)
#define CPM_CPPSR_PLLBP         (1<<30)
#define CPM_CPPSR_PLLON         (1<<29)
#define CPM_CPPSR_PS            (1<<28) /* Indicate whether the PLL parameters' change has finished */
#define CPM_CPPSR_FS            (1<<27) /* Indicate whether the main clock's change has finished */
#define CPM_CPPSR_CS            (1<<26) /* Indicate whether the clock switch has finished */
#define CPM_CPPSR_PM            (1<<1)  /* Clock switch mode */
#define CPM_CPPSR_FM            (1<<0)  /* Clock frequency change mode */

/* I2S Clock Divider Register */
#define CPM_I2SCDR_I2SDIV_BIT	0
#define CPM_I2SCDR_I2SDIV_MASK	(0x1ff << CPM_I2SCDR_I2SDIV_BIT)

/* LCD Pixel Clock Divider Register */
#define CPM_LPCDR_LSCS	        (1<<31) /* TV encoder Source Pixel Clock Selection */
#define CPM_LPCDR_LTCS	        (1<<30) /* LCD TV Encoder or Panel pix clock Selection */
#define CPM_LPCDR_PIXDIV_BIT	0
#define CPM_LPCDR_PIXDIV_MASK	(0x7ff << CPM_LPCDR_PIXDIV_BIT)

/* MSC Clock Divider Register */
#define CPM_MSCCDR_MSCDIV_BIT	0
#define CPM_MSCCDR_MSCDIV_MASK	(0x1f << CPM_MSCCDR_MSCDIV_BIT)

/* UHC Clock Divider Register */
#define CPM_UHCCDR_UHCDIV_BIT	0
#define CPM_UHCCDR_UHCDIV_MASK	(0xf << CPM_UHCCDR_UHCDIV_BIT)

/* SSI Clock Divider Register */
#define CPM_SSICDR_SSIDIV_BIT	0
#define CPM_SSICDR_SSIDIV_MASK	(0xf << CPM_SSICDR_SSIDIV_BIT)

/* PCM device clock divider Register */
#define CPM_PCMCDR_PCMS         31 /* PCM source clock Selection */
#define CPM_PCMCDR_PCMCD_BIT    0
#define CPM_PCMCDR_PCMCD_MASK   (0x1ff << CPM_PCMCDR_PCMCD_BIT)

/* PLL Control Register */
#define CPM_CPPCR_PLLM_BIT	23
#define CPM_CPPCR_PLLM_MASK	(0x1ff << CPM_CPPCR_PLLM_BIT)
#define CPM_CPPCR_PLLN_BIT	18
#define CPM_CPPCR_PLLN_MASK	(0x1f << CPM_CPPCR_PLLN_BIT)
#define CPM_CPPCR_PLLOD_BIT	16
#define CPM_CPPCR_PLLOD_MASK	(0x03 << CPM_CPPCR_PLLOD_BIT)
#define CPM_CPPCR_PLLS		(1 << 10) /* obsolete, replaced by CPM_CPPSR_PLLON */
#define CPM_CPPCR_PLLBP		(1 << 9)
#define CPM_CPPCR_PLLEN		(1 << 8)
#define CPM_CPPCR_PLLST_BIT	0
#define CPM_CPPCR_PLLST_MASK	(0xff << CPM_CPPCR_PLLST_BIT)

/* Low Power Control Register */
#define CPM_LCR_DOZE_DUTY_BIT 	3
#define CPM_LCR_DOZE_DUTY_MASK 	(0x1f << CPM_LCR_DOZE_DUTY_BIT)
#define CPM_LCR_DOZE_ON		(1 << 2)
#define CPM_LCR_LPM_BIT		0
#define CPM_LCR_LPM_MASK	(0x3 << CPM_LCR_LPM_BIT)
  #define CPM_LCR_LPM_IDLE	(0x0 << CPM_LCR_LPM_BIT)
  #define CPM_LCR_LPM_SLEEP	(0x1 << CPM_LCR_LPM_BIT)

/* Clock Gate Register */
#define CPM_CLKGR_AUX_CPU	(1 << 24)
#define CPM_CLKGR_AHB1  	(1 << 23)
#define CPM_CLKGR_IDCT  	(1 << 22)
#define CPM_CLKGR_DB    	(1 << 21)
#define CPM_CLKGR_ME    	(1 << 20)
#define CPM_CLKGR_MC    	(1 << 19)
#define CPM_CLKGR_TVE    	(1 << 18)
#define CPM_CLKGR_TSSI    	(1 << 17)
#define CPM_CLKGR_MSC1    	(1 << 16)
#define CPM_CLKGR_UART2    	(1 << 15)
#define CPM_CLKGR_UART1		(1 << 14)
#define CPM_CLKGR_IPU		(1 << 13)
#define CPM_CLKGR_DMAC		(1 << 12)
#define CPM_CLKGR_BCH		(1 << 11)
#define CPM_CLKGR_UDC		(1 << 10)
#define CPM_CLKGR_LCD		(1 << 9)
#define CPM_CLKGR_CIM		(1 << 8)
#define CPM_CLKGR_SADC		(1 << 7)
#define CPM_CLKGR_MSC0		(1 << 6)
#define CPM_CLKGR_AIC		(1 << 5)
#define CPM_CLKGR_SSI		(1 << 4)
#define CPM_CLKGR_I2C		(1 << 3)
#define CPM_CLKGR_RTC		(1 << 2)
#define CPM_CLKGR_TCU		(1 << 1)
#define CPM_CLKGR_UART0		(1 << 0)

/* Oscillator and Power Control Register */
#define CPM_OPCR_O1ST_BIT	8
#define CPM_OPCR_O1ST_MASK	(0xff << CPM_SCR_O1ST_BIT)
#define CPM_OPCR_UHCPHY_DISABLE	(1 << 7)
#define CPM_OPCR_UDCPHY_ENABLE	(1 << 6)
#define CPM_OPCR_OSC_ENABLE	(1 << 4)
#define CPM_OPCR_ERCS           (1 << 2) /* EXCLK/512 clock and RTCLK clock selection */
#define CPM_OPCR_MOSE           (1 << 1) /* Main Oscillator Enable */
#define CPM_OPCR_MCS            (1 << 0) /* Main clock source select register */

/* Reset Status Register */
#define CPM_RSR_HR		(1 << 2)
#define CPM_RSR_WR		(1 << 1)
#define CPM_RSR_PR		(1 << 0)


/*************************************************************************
 * TCU (Timer Counter Unit)
 *************************************************************************/
#define TCU_TSTR	(TCU_BASE + 0xF0) /* Timer Status Register,Only Used In Tcu2 Mode */
#define TCU_TSTSR	(TCU_BASE + 0xF4) /* Timer Status Set Register */
#define TCU_TSTCR	(TCU_BASE + 0xF8) /* Timer Status Clear Register */
#define TCU_TSR		(TCU_BASE + 0x1C) /* Timer Stop Register */
#define TCU_TSSR	(TCU_BASE + 0x2C) /* Timer Stop Set Register */
#define TCU_TSCR	(TCU_BASE + 0x3C) /* Timer Stop Clear Register */
#define TCU_TER		(TCU_BASE + 0x10) /* Timer Counter Enable Register */
#define TCU_TESR	(TCU_BASE + 0x14) /* Timer Counter Enable Set Register */
#define TCU_TECR	(TCU_BASE + 0x18) /* Timer Counter Enable Clear Register */
#define TCU_TFR		(TCU_BASE + 0x20) /* Timer Flag Register */
#define TCU_TFSR	(TCU_BASE + 0x24) /* Timer Flag Set Register */
#define TCU_TFCR	(TCU_BASE + 0x28) /* Timer Flag Clear Register */
#define TCU_TMR		(TCU_BASE + 0x30) /* Timer Mask Register */
#define TCU_TMSR	(TCU_BASE + 0x34) /* Timer Mask Set Register */
#define TCU_TMCR	(TCU_BASE + 0x38) /* Timer Mask Clear Register */

#define TCU_TDFR0	(TCU_BASE + 0x40) /* Timer Data Full Register */
#define TCU_TDHR0	(TCU_BASE + 0x44) /* Timer Data Half Register */
#define TCU_TCNT0	(TCU_BASE + 0x48) /* Timer Counter Register */
#define TCU_TCSR0	(TCU_BASE + 0x4C) /* Timer Control Register */
#define TCU_TDFR1	(TCU_BASE + 0x50)
#define TCU_TDHR1	(TCU_BASE + 0x54)
#define TCU_TCNT1	(TCU_BASE + 0x58)
#define TCU_TCSR1	(TCU_BASE + 0x5C)
#define TCU_TDFR2	(TCU_BASE + 0x60)
#define TCU_TDHR2	(TCU_BASE + 0x64)
#define TCU_TCNT2	(TCU_BASE + 0x68)
#define TCU_TCSR2	(TCU_BASE + 0x6C)
#define TCU_TDFR3	(TCU_BASE + 0x70)
#define TCU_TDHR3	(TCU_BASE + 0x74)
#define TCU_TCNT3	(TCU_BASE + 0x78)
#define TCU_TCSR3	(TCU_BASE + 0x7C)
#define TCU_TDFR4	(TCU_BASE + 0x80)
#define TCU_TDHR4	(TCU_BASE + 0x84)
#define TCU_TCNT4	(TCU_BASE + 0x88)
#define TCU_TCSR4	(TCU_BASE + 0x8C)
#define TCU_TDFR5	(TCU_BASE + 0x90)
#define TCU_TDHR5	(TCU_BASE + 0x94)
#define TCU_TCNT5	(TCU_BASE + 0x98)
#define TCU_TCSR5	(TCU_BASE + 0x9C)

#define REG_TCU_TSTR	REG32(TCU_TSTR)
#define REG_TCU_TSTSR	REG32(TCU_TSTSR)
#define REG_TCU_TSTCR	REG32(TCU_TSTCR)
#define REG_TCU_TSR	REG32(TCU_TSR)
#define REG_TCU_TSSR	REG32(TCU_TSSR)
#define REG_TCU_TSCR	REG32(TCU_TSCR)
#define REG_TCU_TER	REG16(TCU_TER)
#define REG_TCU_TESR	REG32(TCU_TESR)
#define REG_TCU_TECR	REG32(TCU_TECR)
#define REG_TCU_TFR	REG32(TCU_TFR)
#define REG_TCU_TFSR	REG32(TCU_TFSR)
#define REG_TCU_TFCR	REG32(TCU_TFCR)
#define REG_TCU_TMR	REG32(TCU_TMR)
#define REG_TCU_TMSR	REG32(TCU_TMSR)
#define REG_TCU_TMCR	REG32(TCU_TMCR)
#define REG_TCU_TDFR0	REG16(TCU_TDFR0)
#define REG_TCU_TDHR0	REG16(TCU_TDHR0)
#define REG_TCU_TCNT0	REG16(TCU_TCNT0)
#define REG_TCU_TCSR0	REG16(TCU_TCSR0)
#define REG_TCU_TDFR1	REG16(TCU_TDFR1)
#define REG_TCU_TDHR1	REG16(TCU_TDHR1)
#define REG_TCU_TCNT1	REG16(TCU_TCNT1)
#define REG_TCU_TCSR1	REG16(TCU_TCSR1)
#define REG_TCU_TDFR2	REG16(TCU_TDFR2)
#define REG_TCU_TDHR2	REG16(TCU_TDHR2)
#define REG_TCU_TCNT2	REG16(TCU_TCNT2)
#define REG_TCU_TCSR2	REG16(TCU_TCSR2)
#define REG_TCU_TDFR3	REG16(TCU_TDFR3)
#define REG_TCU_TDHR3	REG16(TCU_TDHR3)
#define REG_TCU_TCNT3	REG16(TCU_TCNT3)
#define REG_TCU_TCSR3	REG16(TCU_TCSR3)
#define REG_TCU_TDFR4	REG16(TCU_TDFR4)
#define REG_TCU_TDHR4	REG16(TCU_TDHR4)
#define REG_TCU_TCNT4	REG16(TCU_TCNT4)
#define REG_TCU_TCSR4	REG16(TCU_TCSR4)

// n = 0,1,2,3,4,5
#define TCU_TDFR(n)	(TCU_BASE + (0x40 + (n)*0x10)) /* Timer Data Full Reg */
#define TCU_TDHR(n)	(TCU_BASE + (0x44 + (n)*0x10)) /* Timer Data Half Reg */
#define TCU_TCNT(n)	(TCU_BASE + (0x48 + (n)*0x10)) /* Timer Counter Reg */
#define TCU_TCSR(n)	(TCU_BASE + (0x4C + (n)*0x10)) /* Timer Control Reg */
#define TCU_OSTDR	(TCU_BASE + 0xe0) /* Operating System Timer Data Reg */
#define TCU_OSTCNT	(TCU_BASE + 0xe8) /* Operating System Timer Counter Reg */
#define TCU_OSTCSR	(TCU_BASE + 0xeC) /* Operating System Timer Control Reg */

#define REG_TCU_TDFR(n)	REG16(TCU_TDFR((n)))
#define REG_TCU_TDHR(n)	REG16(TCU_TDHR((n)))
#define REG_TCU_TCNT(n)	REG16(TCU_TCNT((n)))
#define REG_TCU_TCSR(n)	REG16(TCU_TCSR((n)))
#define REG_TCU_OSTDR   REG32(TCU_OSTDR)
#define REG_TCU_OSTCNT  REG32(TCU_OSTCNT)
#define REG_TCU_OSTCSR  REG32(TCU_OSTCSR)

// Register definitions
#define TCU_TSTR_REAL2		(1 << 18) /* only used in TCU2 mode */
#define TCU_TSTR_REAL1		(1 << 17) /* only used in TCU2 mode */
#define TCU_TSTR_BUSY2		(1 << 2)  /* only used in TCU2 mode */
#define TCU_TSTR_BUSY1		(1 << 1)  /* only used in TCU2 mode */

#define TCU_TSTSR_REAL2		(1 << 18)
#define TCU_TSTSR_REAL1		(1 << 17)
#define TCU_TSTSR_BUSY2		(1 << 2)
#define TCU_TSTSR_BUSY1		(1 << 1)

#define TCU_TSTCR_REAL2		(1 << 18)
#define TCU_TSTCR_REAL1		(1 << 17)
#define TCU_TSTCR_BUSY2		(1 << 2)
#define TCU_TSTCR_BUSY1		(1 << 1)

#define TCU_TSR_WDTS		(1 << 16) /*the clock supplies to wdt is stopped */
#define TCU_TSR_OSTS		(1 << 15) /*the clock supplies to osts is stopped */
#define TCU_TSR_STOP5		(1 << 5)  /*the clock supplies to timer5 is stopped */
#define TCU_TSR_STOP4		(1 << 4)  /*the clock supplies to timer4 is stopped */
#define TCU_TSR_STOP3		(1 << 3)  /*the clock supplies to timer3 is stopped */
#define TCU_TSR_STOP2		(1 << 2)  /*the clock supplies to timer2 is stopped */
#define TCU_TSR_STOP1		(1 << 1)  /*the clock supplies to timer1 is stopped */
#define TCU_TSR_STOP0		(1 << 0)  /*the clock supplies to timer0 is stopped */

#define TCU_TSSR_WDTSS		(1 << 16)
#define TCU_TSSR_OSTSS		(1 << 15)
#define TCU_TSSR_STPS5		(1 << 5)
#define TCU_TSSR_STPS4		(1 << 4)
#define TCU_TSSR_STPS3		(1 << 3)
#define TCU_TSSR_STPS2		(1 << 2)
#define TCU_TSSR_STPS1		(1 << 1)
#define TCU_TSSR_STPS0		(1 << 0)

#define TCU_TSCR_WDTSC		(1 << 16)
#define TCU_TSCR_OSTSC		(1 << 15)
#define TCU_TSCR_STPC5		(1 << 5)
#define TCU_TSCR_STPC4		(1 << 4)
#define TCU_TSCR_STPC3		(1 << 3)
#define TCU_TSCR_STPC2		(1 << 2)
#define TCU_TSCR_STPC1		(1 << 1)
#define TCU_TSCR_STPC0		(1 << 0)

#define TCU_TER_OSTEN		(1 << 15) /* enable the counter in ost */
#define TCU_TER_TCEN5		(1 << 5)  /* enable the counter in timer5 */
#define TCU_TER_TCEN4		(1 << 4)
#define TCU_TER_TCEN3		(1 << 3)
#define TCU_TER_TCEN2		(1 << 2)
#define TCU_TER_TCEN1		(1 << 1)
#define TCU_TER_TCEN0		(1 << 0)

#define TCU_TESR_OSTST		(1 << 15)
#define TCU_TESR_TCST5		(1 << 5)
#define TCU_TESR_TCST4		(1 << 4)
#define TCU_TESR_TCST3		(1 << 3)
#define TCU_TESR_TCST2		(1 << 2)
#define TCU_TESR_TCST1		(1 << 1)
#define TCU_TESR_TCST0		(1 << 0)

#define TCU_TECR_OSTCL		(1 << 15)
#define TCU_TECR_TCCL5		(1 << 5)
#define TCU_TECR_TCCL4		(1 << 4)
#define TCU_TECR_TCCL3		(1 << 3)
#define TCU_TECR_TCCL2		(1 << 2)
#define TCU_TECR_TCCL1		(1 << 1)
#define TCU_TECR_TCCL0		(1 << 0)

#define TCU_TFR_HFLAG5		(1 << 21) /* half comparison match flag */
#define TCU_TFR_HFLAG4		(1 << 20)
#define TCU_TFR_HFLAG3		(1 << 19)
#define TCU_TFR_HFLAG2		(1 << 18)
#define TCU_TFR_HFLAG1		(1 << 17)
#define TCU_TFR_HFLAG0		(1 << 16)
#define TCU_TFR_OSTFLAG		(1 << 15) /* ost comparison match flag */
#define TCU_TFR_FFLAG5		(1 << 5)  /* full comparison match flag */
#define TCU_TFR_FFLAG4		(1 << 4)
#define TCU_TFR_FFLAG3		(1 << 3)
#define TCU_TFR_FFLAG2		(1 << 2)
#define TCU_TFR_FFLAG1		(1 << 1)
#define TCU_TFR_FFLAG0		(1 << 0)

#define TCU_TFSR_HFST5		(1 << 21)
#define TCU_TFSR_HFST4		(1 << 20)
#define TCU_TFSR_HFST3		(1 << 19)
#define TCU_TFSR_HFST2		(1 << 18)
#define TCU_TFSR_HFST1		(1 << 17)
#define TCU_TFSR_HFST0		(1 << 16)
#define TCU_TFSR_OSTFST		(1 << 15)
#define TCU_TFSR_FFST5		(1 << 5)
#define TCU_TFSR_FFST4		(1 << 4)
#define TCU_TFSR_FFST3		(1 << 3)
#define TCU_TFSR_FFST2		(1 << 2)
#define TCU_TFSR_FFST1		(1 << 1)
#define TCU_TFSR_FFST0		(1 << 0)

#define TCU_TFCR_HFCL5		(1 << 21)
#define TCU_TFCR_HFCL4		(1 << 20)
#define TCU_TFCR_HFCL3		(1 << 19)
#define TCU_TFCR_HFCL2		(1 << 18)
#define TCU_TFCR_HFCL1		(1 << 17)
#define TCU_TFCR_HFCL0		(1 << 16)
#define TCU_TFCR_OSTFCL		(1 << 15)
#define TCU_TFCR_FFCL5		(1 << 5)
#define TCU_TFCR_FFCL4		(1 << 4)
#define TCU_TFCR_FFCL3		(1 << 3)
#define TCU_TFCR_FFCL2		(1 << 2)
#define TCU_TFCR_FFCL1		(1 << 1)
#define TCU_TFCR_FFCL0		(1 << 0)

#define TCU_TMR_HMASK5		(1 << 21) /* half comparison match interrupt mask */
#define TCU_TMR_HMASK4		(1 << 20)
#define TCU_TMR_HMASK3		(1 << 19)
#define TCU_TMR_HMASK2		(1 << 18)
#define TCU_TMR_HMASK1		(1 << 17)
#define TCU_TMR_HMASK0		(1 << 16)
#define TCU_TMR_OSTMASK		(1 << 15) /* ost comparison match interrupt mask */
#define TCU_TMR_FMASK5		(1 << 5)  /* full comparison match interrupt mask */
#define TCU_TMR_FMASK4		(1 << 4)
#define TCU_TMR_FMASK3		(1 << 3)
#define TCU_TMR_FMASK2		(1 << 2)
#define TCU_TMR_FMASK1		(1 << 1)
#define TCU_TMR_FMASK0		(1 << 0)

#define TCU_TMSR_HMST5		(1 << 21)
#define TCU_TMSR_HMST4		(1 << 20)
#define TCU_TMSR_HMST3		(1 << 19)
#define TCU_TMSR_HMST2		(1 << 18)
#define TCU_TMSR_HMST1		(1 << 17)
#define TCU_TMSR_HMST0		(1 << 16)
#define TCU_TMSR_OSTMST		(1 << 15)
#define TCU_TMSR_FMST5		(1 << 5)
#define TCU_TMSR_FMST4		(1 << 4)
#define TCU_TMSR_FMST3		(1 << 3)
#define TCU_TMSR_FMST2		(1 << 2)
#define TCU_TMSR_FMST1		(1 << 1)
#define TCU_TMSR_FMST0		(1 << 0)

#define TCU_TMCR_HMCL5		(1 << 21)
#define TCU_TMCR_HMCL4		(1 << 20)
#define TCU_TMCR_HMCL3		(1 << 19)
#define TCU_TMCR_HMCL2		(1 << 18)
#define TCU_TMCR_HMCL1		(1 << 17)
#define TCU_TMCR_HMCL0		(1 << 16)
#define TCU_TMCR_OSTMCL		(1 << 15)
#define TCU_TMCR_FMCL5		(1 << 5)
#define TCU_TMCR_FMCL4		(1 << 4)
#define TCU_TMCR_FMCL3		(1 << 3)
#define TCU_TMCR_FMCL2		(1 << 2)
#define TCU_TMCR_FMCL1		(1 << 1)
#define TCU_TMCR_FMCL0		(1 << 0)

#define TCU_TCSR_CNT_CLRZ	(1 << 10) /* clear counter to 0, only used in TCU2 mode */
#define TCU_TCSR_PWM_SD		(1 << 9)  /* shut down the pwm output only used in TCU1 mode */
#define TCU_TCSR_PWM_INITL_HIGH	(1 << 8)  /* selects an initial output level for pwm output */
#define TCU_TCSR_PWM_EN		(1 << 7)  /* pwm pin output enable */
#define TCU_TCSR_PRESCALE_BIT	3         /* select the tcnt count clock frequency*/
#define TCU_TCSR_PRESCALE_MASK	(0x7 << TCU_TCSR_PRESCALE_BIT)
  #define TCU_TCSR_PRESCALE1	(0x0 << TCU_TCSR_PRESCALE_BIT)
  #define TCU_TCSR_PRESCALE4	(0x1 << TCU_TCSR_PRESCALE_BIT)
  #define TCU_TCSR_PRESCALE16	(0x2 << TCU_TCSR_PRESCALE_BIT)
  #define TCU_TCSR_PRESCALE64	(0x3 << TCU_TCSR_PRESCALE_BIT)
  #define TCU_TCSR_PRESCALE256	(0x4 << TCU_TCSR_PRESCALE_BIT)
  #define TCU_TCSR_PRESCALE1024	(0x5 << TCU_TCSR_PRESCALE_BIT)
#define TCU_TCSR_EXT_EN		(1 << 2)  /* select extal as the timer clock input */
#define TCU_TCSR_RTC_EN		(1 << 1)  /* select rtcclk as the timer clock input */
#define TCU_TCSR_PCK_EN		(1 << 0)  /* select pclk as the timer clock input */

#define TCU_TSTR_REAL2		(1 << 18) /* the value read from counter 2 is a real value */
#define TCU_TSTR_REAL1		(1 << 17)
#define TCU_TSTR_BUSY2		(1 << 2)  /* the counter 2 is busy now */
#define TCU_TSTR_BUSY1		(1 << 1)

#define TCU_TSTSR_REALS2	(1 << 18)
#define TCU_TSTSR_REALS1	(1 << 17)
#define TCU_TSTSR_BUSYS2	(1 << 2)
#define TCU_TSTSR_BUSYS1	(1 << 1)

#define TCU_TSTCR_REALC2	(1 << 18)
#define TCU_TSTCR_REALC1	(1 << 17)
#define TCU_TSTCR_BUSYC2	(1 << 2)
#define TCU_TSTCR_BUSYC1	(1 << 1)

#define TCU_OSTCR_CNT_MD		(1 << 15) /* when the value counter is equal to compare value,the counter is go on increasing till overflow,and then icrease from 0 */
#define TCU_OSTCR_PWM_SD		(1 << 9) /* shut down the pwm output, only used in TCU1 mode */
#define TCU_OSTCSR_PRESCALE_BIT		3
#define TCU_OSTCSR_PRESCALE_MASK	(0x7 << TCU_OSTCSR_PRESCALE_BIT)
  #define TCU_OSTCSR_PRESCALE1		(0x0 << TCU_OSTCSR_PRESCALE_BIT)
  #define TCU_OSTCSR_PRESCALE4		(0x1 << TCU_OSTCSR_PRESCALE_BIT)
  #define TCU_OSTCSR_PRESCALE16		(0x2 << TCU_OSTCSR_PRESCALE_BIT)
  #define TCU_OSTCSR_PRESCALE64		(0x3 << TCU_OSTCSR_PRESCALE_BIT)
  #define TCU_OSTCSR_PRESCALE256	(0x4 << TCU_OSTCSR_PRESCALE_BIT)
  #define TCU_OSTCSR_PRESCALE1024	(0x5 << TCU_OSTCSR_PRESCALE_BIT)
#define TCU_OSTCSR_EXT_EN		(1 << 2) /* select extal as the timer clock input */
#define TCU_OSTCSR_RTC_EN		(1 << 1) /* select rtcclk as the timer clock input */
#define TCU_OSTCSR_PCK_EN		(1 << 0) /* select pclk as the timer clock input */

/*************************************************************************
 * WDT (WatchDog Timer)
 *************************************************************************/
#define WDT_TDR		(WDT_BASE + 0x00)
#define WDT_TCER	(WDT_BASE + 0x04)
#define WDT_TCNT	(WDT_BASE + 0x08)
#define WDT_TCSR	(WDT_BASE + 0x0C)

#define REG_WDT_TDR	REG16(WDT_TDR)
#define REG_WDT_TCER	REG8(WDT_TCER)
#define REG_WDT_TCNT	REG16(WDT_TCNT)
#define REG_WDT_TCSR	REG16(WDT_TCSR)

// Register definition
#define WDT_TCSR_PRESCALE_BIT	3
#define WDT_TCSR_PRESCALE_MASK	(0x7 << WDT_TCSR_PRESCALE_BIT)
  #define WDT_TCSR_PRESCALE1	(0x0 << WDT_TCSR_PRESCALE_BIT)
  #define WDT_TCSR_PRESCALE4	(0x1 << WDT_TCSR_PRESCALE_BIT)
  #define WDT_TCSR_PRESCALE16	(0x2 << WDT_TCSR_PRESCALE_BIT)
  #define WDT_TCSR_PRESCALE64	(0x3 << WDT_TCSR_PRESCALE_BIT)
  #define WDT_TCSR_PRESCALE256	(0x4 << WDT_TCSR_PRESCALE_BIT)
  #define WDT_TCSR_PRESCALE1024	(0x5 << WDT_TCSR_PRESCALE_BIT)
#define WDT_TCSR_EXT_EN		(1 << 2)
#define WDT_TCSR_RTC_EN		(1 << 1)
#define WDT_TCSR_PCK_EN		(1 << 0)

#define WDT_TCER_TCEN		(1 << 0)


/*************************************************************************
 * DMAC (DMA Controller)
 *************************************************************************/

#define MAX_DMA_NUM	8  /* max 8 channels */
#define HALF_DMA_NUM	4   /* the number of one dma controller's channels */

/* m is the DMA controller index (0, 1), n is the DMA channel index (0 - 11) */

#define DMAC_DSAR(n)  (DMAC_BASE + ((n)/HALF_DMA_NUM*0x100 + 0x00 + ((n)-(n)/HALF_DMA_NUM*HALF_DMA_NUM) * 0x20)) /* DMA source address */
#define DMAC_DTAR(n)  (DMAC_BASE + ((n)/HALF_DMA_NUM*0x100 + 0x04 + ((n)-(n)/HALF_DMA_NUM*HALF_DMA_NUM) * 0x20)) /* DMA target address */
#define DMAC_DTCR(n)  (DMAC_BASE + ((n)/HALF_DMA_NUM*0x100 + 0x08 + ((n)-(n)/HALF_DMA_NUM*HALF_DMA_NUM) * 0x20)) /* DMA transfer count */
#define DMAC_DRSR(n)  (DMAC_BASE + ((n)/HALF_DMA_NUM*0x100 + 0x0c + ((n)-(n)/HALF_DMA_NUM*HALF_DMA_NUM) * 0x20)) /* DMA request source */
#define DMAC_DCCSR(n) (DMAC_BASE + ((n)/HALF_DMA_NUM*0x100 + 0x10 + ((n)-(n)/HALF_DMA_NUM*HALF_DMA_NUM) * 0x20)) /* DMA control/status */
#define DMAC_DCMD(n)  (DMAC_BASE + ((n)/HALF_DMA_NUM*0x100 + 0x14 + ((n)-(n)/HALF_DMA_NUM*HALF_DMA_NUM) * 0x20)) /* DMA command */
#define DMAC_DDA(n)   (DMAC_BASE + ((n)/HALF_DMA_NUM*0x100 + 0x18 + ((n)-(n)/HALF_DMA_NUM*HALF_DMA_NUM) * 0x20)) /* DMA descriptor address */
#define DMAC_DSD(n)   (DMAC_BASE + ((n)/HALF_DMA_NUM*0x100 + 0xc0 + ((n)-(n)/HALF_DMA_NUM*HALF_DMA_NUM) * 0x04)) /* DMA Stride Address */

#define DMAC_DMACR(m)	(DMAC_BASE + 0x0300 + 0x100 * (m))              /* DMA control register */
#define DMAC_DMAIPR(m)	(DMAC_BASE + 0x0304 + 0x100 * (m))              /* DMA interrupt pending */
#define DMAC_DMADBR(m)	(DMAC_BASE + 0x0308 + 0x100 * (m))              /* DMA doorbell */
#define DMAC_DMADBSR(m)	(DMAC_BASE + 0x030C + 0x100 * (m))              /* DMA doorbell set */
#define DMAC_DMACKE(m)  (DMAC_BASE + 0x0310 + 0x100 * (m))

#define REG_DMAC_DSAR(n)	REG32(DMAC_DSAR((n)))
#define REG_DMAC_DTAR(n)	REG32(DMAC_DTAR((n)))
#define REG_DMAC_DTCR(n)	REG32(DMAC_DTCR((n)))
#define REG_DMAC_DRSR(n)	REG32(DMAC_DRSR((n)))
#define REG_DMAC_DCCSR(n)	REG32(DMAC_DCCSR((n)))
#define REG_DMAC_DCMD(n)	REG32(DMAC_DCMD((n)))
#define REG_DMAC_DDA(n)		REG32(DMAC_DDA((n)))
#define REG_DMAC_DSD(n)         REG32(DMAC_DSD(n))
#define REG_DMAC_DMACR(m)	REG32(DMAC_DMACR(m))
#define REG_DMAC_DMAIPR(m)	REG32(DMAC_DMAIPR(m))
#define REG_DMAC_DMADBR(m)	REG32(DMAC_DMADBR(m))
#define REG_DMAC_DMADBSR(m)	REG32(DMAC_DMADBSR(m))
#define REG_DMAC_DMACKE(m)      REG32(DMAC_DMACKE(m))

// DMA request source register
#define DMAC_DRSR_RS_BIT	0
#define DMAC_DRSR_RS_MASK	(0x3f << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_EXT	(0 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_NAND	(1 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_BCH_ENC	(2 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_BCH_DEC	(3 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_AUTO	(8 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_TSSIIN	(9 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART3OUT	(14 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART3IN	(15 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART2OUT	(16 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART2IN	(17 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART1OUT	(18 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART1IN	(19 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART0OUT	(20 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART0IN	(21 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_SSI0OUT	(22 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_SSI0IN	(23 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_AICOUT	(24 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_AICIN	(25 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_MSC0OUT	(26 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_MSC0IN	(27 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_TCU	(28 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_SADC	(29 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_MSC1OUT	(30 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_MSC1IN	(31 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_SSI1OUT	(32 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_SSI1IN	(33 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_PMOUT	(34 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_PMIN	(35 << DMAC_DRSR_RS_BIT)

// DMA channel control/status register
#define DMAC_DCCSR_NDES		(1 << 31) /* descriptor (0) or not (1) ? */
#define DMAC_DCCSR_DES8    	(1 << 30) /* Descriptor 8 Word */
#define DMAC_DCCSR_DES4    	(0 << 30) /* Descriptor 4 Word */
#define DMAC_DCCSR_CDOA_BIT	16        /* copy of DMA offset address */
#define DMAC_DCCSR_CDOA_MASK	(0xff << DMAC_DCCSR_CDOA_BIT)
#define DMAC_DCCSR_BERR		(1 << 7)  /* BCH error within this transfer, Only for channel 0 */
#define DMAC_DCCSR_INV		(1 << 6)  /* descriptor invalid */
#define DMAC_DCCSR_AR		(1 << 4)  /* address error */
#define DMAC_DCCSR_TT		(1 << 3)  /* transfer terminated */
#define DMAC_DCCSR_HLT		(1 << 2)  /* DMA halted */
#define DMAC_DCCSR_CT		(1 << 1)  /* count terminated */
#define DMAC_DCCSR_EN		(1 << 0)  /* channel enable bit */

// DMA channel command register
#define DMAC_DCMD_EACKS_LOW  	(1 << 31) /* External DACK Output Level Select, active low */
#define DMAC_DCMD_EACKS_HIGH  	(0 << 31) /* External DACK Output Level Select, active high */
#define DMAC_DCMD_EACKM_WRITE 	(1 << 30) /* External DACK Output Mode Select, output in write cycle */
#define DMAC_DCMD_EACKM_READ 	(0 << 30) /* External DACK Output Mode Select, output in read cycle */
#define DMAC_DCMD_ERDM_BIT      28        /* External DREQ Detection Mode Select */
#define DMAC_DCMD_ERDM_MASK     (0x03 << DMAC_DCMD_ERDM_BIT)
  #define DMAC_DCMD_ERDM_LOW    (0 << DMAC_DCMD_ERDM_BIT)
  #define DMAC_DCMD_ERDM_FALL   (1 << DMAC_DCMD_ERDM_BIT)
  #define DMAC_DCMD_ERDM_HIGH   (2 << DMAC_DCMD_ERDM_BIT)
  #define DMAC_DCMD_ERDM_RISE   (3 << DMAC_DCMD_ERDM_BIT)
#define DMAC_DCMD_BLAST		(1 << 25) /* BCH last */
#define DMAC_DCMD_SAI		(1 << 23) /* source address increment */
#define DMAC_DCMD_DAI		(1 << 22) /* dest address increment */
#define DMAC_DCMD_RDIL_BIT	16        /* request detection interval length */
#define DMAC_DCMD_RDIL_MASK	(0x0f << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_IGN	(0 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_2	(1 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_4	(2 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_8	(3 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_12	(4 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_16	(5 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_20	(6 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_24	(7 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_28	(8 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_32	(9 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_48	(10 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_60	(11 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_64	(12 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_124	(13 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_128	(14 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_200	(15 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_SWDH_BIT	14  /* source port width */
#define DMAC_DCMD_SWDH_MASK	(0x03 << DMAC_DCMD_SWDH_BIT)
  #define DMAC_DCMD_SWDH_32	(0 << DMAC_DCMD_SWDH_BIT)
  #define DMAC_DCMD_SWDH_8	(1 << DMAC_DCMD_SWDH_BIT)
  #define DMAC_DCMD_SWDH_16	(2 << DMAC_DCMD_SWDH_BIT)
#define DMAC_DCMD_DWDH_BIT	12  /* dest port width */
#define DMAC_DCMD_DWDH_MASK	(0x03 << DMAC_DCMD_DWDH_BIT)
  #define DMAC_DCMD_DWDH_32	(0 << DMAC_DCMD_DWDH_BIT)
  #define DMAC_DCMD_DWDH_8	(1 << DMAC_DCMD_DWDH_BIT)
  #define DMAC_DCMD_DWDH_16	(2 << DMAC_DCMD_DWDH_BIT)
#define DMAC_DCMD_DS_BIT	8  /* transfer data size of a data unit */
#define DMAC_DCMD_DS_MASK	(0x07 << DMAC_DCMD_DS_BIT)
  #define DMAC_DCMD_DS_32BIT	(0 << DMAC_DCMD_DS_BIT)
  #define DMAC_DCMD_DS_8BIT	(1 << DMAC_DCMD_DS_BIT)
  #define DMAC_DCMD_DS_16BIT	(2 << DMAC_DCMD_DS_BIT)
  #define DMAC_DCMD_DS_16BYTE	(3 << DMAC_DCMD_DS_BIT)
  #define DMAC_DCMD_DS_32BYTE	(4 << DMAC_DCMD_DS_BIT)
#define DMAC_DCMD_STDE   	(1 << 5)  /* Stride Disable/Enable */
#define DMAC_DCMD_DES_V		(1 << 4)  /* descriptor valid flag */
#define DMAC_DCMD_DES_VM	(1 << 3)  /* descriptor valid mask: 1:support V-bit */
#define DMAC_DCMD_DES_VIE	(1 << 2)  /* DMA valid error interrupt enable */
#define DMAC_DCMD_TIE		(1 << 1)  /* DMA transfer interrupt enable */
#define DMAC_DCMD_LINK		(1 << 0)  /* descriptor link enable */

// DMA descriptor address register
#define DMAC_DDA_BASE_BIT	12  /* descriptor base address */
#define DMAC_DDA_BASE_MASK	(0x0fffff << DMAC_DDA_BASE_BIT)
#define DMAC_DDA_OFFSET_BIT	4   /* descriptor offset address */
#define DMAC_DDA_OFFSET_MASK	(0x0ff << DMAC_DDA_OFFSET_BIT)

// DMA stride address register
#define DMAC_DSD_TSD_BIT        16  /* target stride address */
#define DMAC_DSD_TSD_MASK      	(0xffff << DMAC_DSD_TSD_BIT)
#define DMAC_DSD_SSD_BIT        0  /* source stride address */
#define DMAC_DSD_SSD_MASK      	(0xffff << DMAC_DSD_SSD_BIT)

// DMA control register
#define DMAC_DMACR_FMSC		(1 << 31)  /* MSC Fast DMA mode */
#define DMAC_DMACR_FSSI		(1 << 30)  /* SSI Fast DMA mode */
#define DMAC_DMACR_FTSSI	(1 << 29)  /* TSSI Fast DMA mode */
#define DMAC_DMACR_FUART	(1 << 28)  /* UART Fast DMA mode */
#define DMAC_DMACR_FAIC		(1 << 27)  /* AIC Fast DMA mode */
#define DMAC_DMACR_PR_BIT	8  /* channel priority mode */
#define DMAC_DMACR_PR_MASK	(0x03 << DMAC_DMACR_PR_BIT)
  #define DMAC_DMACR_PR_012345	(0 << DMAC_DMACR_PR_BIT)
  #define DMAC_DMACR_PR_120345	(1 << DMAC_DMACR_PR_BIT)
  #define DMAC_DMACR_PR_230145	(2 << DMAC_DMACR_PR_BIT)
  #define DMAC_DMACR_PR_340125	(3 << DMAC_DMACR_PR_BIT)
#define DMAC_DMACR_HLT		(1 << 3)  /* DMA halt flag */
#define DMAC_DMACR_AR		(1 << 2)  /* address error flag */
#define DMAC_DMACR_DMAE		(1 << 0)  /* DMA enable bit */

// DMA doorbell register
#define DMAC_DMADBR_DB5		(1 << 5)  /* doorbell for channel 5 */
#define DMAC_DMADBR_DB4		(1 << 4)  /* doorbell for channel 4 */
#define DMAC_DMADBR_DB3		(1 << 3)  /* doorbell for channel 3 */
#define DMAC_DMADBR_DB2		(1 << 2)  /* doorbell for channel 2 */
#define DMAC_DMADBR_DB1		(1 << 1)  /* doorbell for channel 1 */
#define DMAC_DMADBR_DB0		(1 << 0)  /* doorbell for channel 0 */

// DMA doorbell set register
#define DMAC_DMADBSR_DBS5	(1 << 5)  /* enable doorbell for channel 5 */
#define DMAC_DMADBSR_DBS4	(1 << 4)  /* enable doorbell for channel 4 */
#define DMAC_DMADBSR_DBS3	(1 << 3)  /* enable doorbell for channel 3 */
#define DMAC_DMADBSR_DBS2	(1 << 2)  /* enable doorbell for channel 2 */
#define DMAC_DMADBSR_DBS1	(1 << 1)  /* enable doorbell for channel 1 */
#define DMAC_DMADBSR_DBS0	(1 << 0)  /* enable doorbell for channel 0 */

// DMA interrupt pending register
#define DMAC_DMAIPR_CIRQ5	(1 << 5)  /* irq pending status for channel 5 */
#define DMAC_DMAIPR_CIRQ4	(1 << 4)  /* irq pending status for channel 4 */
#define DMAC_DMAIPR_CIRQ3	(1 << 3)  /* irq pending status for channel 3 */
#define DMAC_DMAIPR_CIRQ2	(1 << 2)  /* irq pending status for channel 2 */
#define DMAC_DMAIPR_CIRQ1	(1 << 1)  /* irq pending status for channel 1 */
#define DMAC_DMAIPR_CIRQ0	(1 << 0)  /* irq pending status for channel 0 */


/*************************************************************************
 * GPIO (General-Purpose I/O Ports)
 *************************************************************************/
#define MAX_GPIO_NUM	192
#define GPIO_WAKEUP     (32 * 4 + 30)

#define GPIO_BASEA GPIO_BASE +(0)*0x100                                                                  
#define GPIO_BASEB GPIO_BASE +(1)*0x100                                                                  
#define GPIO_BASEC GPIO_BASE +(2)*0x100                                                                  
#define GPIO_BASED GPIO_BASE +(3)*0x100                                                                  
#define GPIO_BASEE GPIO_BASE +(4)*0x100                                                                  
#define GPIO_BASEF GPIO_BASE +(5)*0x100
//n = 0,1,2,3,4,5 (PORTA, PORTB, PORTC, PORTD, PORTE, PORTF)
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


/*************************************************************************
 * UART
 *************************************************************************/

#define IRDA_BASE	UART0_BASE
#define UART_BASE	UART0_BASE
#define UART_OFF	0x1000

/* Register Offset */
#define OFF_RDR		(0x00)	/* R  8b H'xx */
#define OFF_TDR		(0x00)	/* W  8b H'xx */
#define OFF_DLLR	(0x00)	/* RW 8b H'00 */
#define OFF_DLHR	(0x04)	/* RW 8b H'00 */
#define OFF_IER		(0x04)	/* RW 8b H'00 */
#define OFF_ISR		(0x08)	/* R  8b H'01 */
#define OFF_FCR		(0x08)	/* W  8b H'00 */
#define OFF_LCR		(0x0C)	/* RW 8b H'00 */
#define OFF_MCR		(0x10)	/* RW 8b H'00 */
#define OFF_LSR		(0x14)	/* R  8b H'00 */
#define OFF_MSR		(0x18)	/* R  8b H'00 */
#define OFF_SPR		(0x1C)	/* RW 8b H'00 */
#define OFF_SIRCR	(0x20)	/* RW 8b H'00, UART0 */
#define OFF_UMR		(0x24)	/* RW 8b H'00, UART M Register */
#define OFF_UACR	(0x28)	/* RW 8b H'00, UART Add Cycle Register */

/* Register Address */
#define UART0_RDR	(UART0_BASE + OFF_RDR)
#define UART0_TDR	(UART0_BASE + OFF_TDR)
#define UART0_DLLR	(UART0_BASE + OFF_DLLR)
#define UART0_DLHR	(UART0_BASE + OFF_DLHR)
#define UART0_IER	(UART0_BASE + OFF_IER)
#define UART0_ISR	(UART0_BASE + OFF_ISR)
#define UART0_FCR	(UART0_BASE + OFF_FCR)
#define UART0_LCR	(UART0_BASE + OFF_LCR)
#define UART0_MCR	(UART0_BASE + OFF_MCR)
#define UART0_LSR	(UART0_BASE + OFF_LSR)
#define UART0_MSR	(UART0_BASE + OFF_MSR)
#define UART0_SPR	(UART0_BASE + OFF_SPR)
#define UART0_SIRCR	(UART0_BASE + OFF_SIRCR)
#define UART0_UMR	(UART0_BASE + OFF_UMR)
#define UART0_UACR	(UART0_BASE + OFF_UACR)

#define UART1_RDR	(UART1_BASE + OFF_RDR)
#define UART1_TDR	(UART1_BASE + OFF_TDR)
#define UART1_DLLR	(UART1_BASE + OFF_DLLR)
#define UART1_DLHR	(UART1_BASE + OFF_DLHR)
#define UART1_IER	(UART1_BASE + OFF_IER)
#define UART1_ISR	(UART1_BASE + OFF_ISR)
#define UART1_FCR	(UART1_BASE + OFF_FCR)
#define UART1_LCR	(UART1_BASE + OFF_LCR)
#define UART1_MCR	(UART1_BASE + OFF_MCR)
#define UART1_LSR	(UART1_BASE + OFF_LSR)
#define UART1_MSR	(UART1_BASE + OFF_MSR)
#define UART1_SPR	(UART1_BASE + OFF_SPR)
#define UART1_SIRCR	(UART1_BASE + OFF_SIRCR)

#define UART2_RDR	(UART2_BASE + OFF_RDR)
#define UART2_TDR	(UART2_BASE + OFF_TDR)
#define UART2_DLLR	(UART2_BASE + OFF_DLLR)
#define UART2_DLHR	(UART2_BASE + OFF_DLHR)
#define UART2_IER	(UART2_BASE + OFF_IER)
#define UART2_ISR	(UART2_BASE + OFF_ISR)
#define UART2_FCR	(UART2_BASE + OFF_FCR)
#define UART2_LCR	(UART2_BASE + OFF_LCR)
#define UART2_MCR	(UART2_BASE + OFF_MCR)
#define UART2_LSR	(UART2_BASE + OFF_LSR)
#define UART2_MSR	(UART2_BASE + OFF_MSR)
#define UART2_SPR	(UART2_BASE + OFF_SPR)
#define UART2_SIRCR	(UART2_BASE + OFF_SIRCR)

#define UART3_RDR	(UART3_BASE + OFF_RDR)
#define UART3_TDR	(UART3_BASE + OFF_TDR)
#define UART3_DLLR	(UART3_BASE + OFF_DLLR)
#define UART3_DLHR	(UART3_BASE + OFF_DLHR)
#define UART3_IER	(UART3_BASE + OFF_IER)
#define UART3_ISR	(UART3_BASE + OFF_ISR)
#define UART3_FCR	(UART3_BASE + OFF_FCR)
#define UART3_LCR	(UART3_BASE + OFF_LCR)
#define UART3_MCR	(UART3_BASE + OFF_MCR)
#define UART3_LSR	(UART3_BASE + OFF_LSR)
#define UART3_MSR	(UART3_BASE + OFF_MSR)
#define UART3_SPR	(UART3_BASE + OFF_SPR)
#define UART3_SIRCR	(UART3_BASE + OFF_SIRCR)


/*
 * Define macros for UARTIER
 * UART Interrupt Enable Register
 */
#define UARTIER_RIE	(1 << 0)	/* 0: receive fifo full interrupt disable */
#define UARTIER_TIE	(1 << 1)	/* 0: transmit fifo empty interrupt disable */
#define UARTIER_RLIE	(1 << 2)	/* 0: receive line status interrupt disable */
#define UARTIER_MIE	(1 << 3)	/* 0: modem status interrupt disable */
#define UARTIER_RTIE	(1 << 4)	/* 0: receive timeout interrupt disable */

/*
 * Define macros for UARTISR
 * UART Interrupt Status Register
 */
#define UARTISR_IP	(1 << 0)	/* 0: interrupt is pending  1: no interrupt */
#define UARTISR_IID	(7 << 1)	/* Source of Interrupt */
#define UARTISR_IID_MSI		(0 << 1)  /* Modem status interrupt */
#define UARTISR_IID_THRI	(1 << 1)  /* Transmitter holding register empty */
#define UARTISR_IID_RDI		(2 << 1)  /* Receiver data interrupt */
#define UARTISR_IID_RLSI	(3 << 1)  /* Receiver line status interrupt */
#define UARTISR_IID_RTO		(6 << 1)  /* Receive timeout */
#define UARTISR_FFMS		(3 << 6)  /* FIFO mode select, set when UARTFCR.FE is set to 1 */
#define UARTISR_FFMS_NO_FIFO	(0 << 6)
#define UARTISR_FFMS_FIFO_MODE	(3 << 6)

/*
 * Define macros for UARTFCR
 * UART FIFO Control Register
 */
#define UARTFCR_FE	(1 << 0)	/* 0: non-FIFO mode  1: FIFO mode */
#define UARTFCR_RFLS	(1 << 1)	/* write 1 to flush receive FIFO */
#define UARTFCR_TFLS	(1 << 2)	/* write 1 to flush transmit FIFO */
#define UARTFCR_DMS	(1 << 3)	/* 0: disable DMA mode */
#define UARTFCR_UUE	(1 << 4)	/* 0: disable UART */
#define UARTFCR_RTRG	(3 << 6)	/* Receive FIFO Data Trigger */
#define UARTFCR_RTRG_1	(0 << 6)
#define UARTFCR_RTRG_4	(1 << 6)
#define UARTFCR_RTRG_8	(2 << 6)
#define UARTFCR_RTRG_15	(3 << 6)

/*
 * Define macros for UARTLCR
 * UART Line Control Register
 */
#define UARTLCR_WLEN	(3 << 0)	/* word length */
#define UARTLCR_WLEN_5	(0 << 0)
#define UARTLCR_WLEN_6	(1 << 0)
#define UARTLCR_WLEN_7	(2 << 0)
#define UARTLCR_WLEN_8	(3 << 0)
#define UARTLCR_STOP	(1 << 2)	/* 0: 1 stop bit when word length is 5,6,7,8
					   1: 1.5 stop bits when 5; 2 stop bits when 6,7,8 */
#define UARTLCR_STOP1	(0 << 2)
#define UARTLCR_STOP2	(1 << 2)
#define UARTLCR_PE	(1 << 3)	/* 0: parity disable */
#define UARTLCR_PROE	(1 << 4)	/* 0: even parity  1: odd parity */
#define UARTLCR_SPAR	(1 << 5)	/* 0: sticky parity disable */
#define UARTLCR_SBRK	(1 << 6)	/* write 0 normal, write 1 send break */
#define UARTLCR_DLAB	(1 << 7)	/* 0: access UARTRDR/TDR/IER  1: access UARTDLLR/DLHR */

/*
 * Define macros for UARTLSR
 * UART Line Status Register
 */
#define UARTLSR_DR	(1 << 0)	/* 0: receive FIFO is empty  1: receive data is ready */
#define UARTLSR_ORER	(1 << 1)	/* 0: no overrun error */
#define UARTLSR_PER	(1 << 2)	/* 0: no parity error */
#define UARTLSR_FER	(1 << 3)	/* 0; no framing error */
#define UARTLSR_BRK	(1 << 4)	/* 0: no break detected  1: receive a break signal */
#define UARTLSR_TDRQ	(1 << 5)	/* 1: transmit FIFO half "empty" */
#define UARTLSR_TEMT	(1 << 6)	/* 1: transmit FIFO and shift registers empty */
#define UARTLSR_RFER	(1 << 7)	/* 0: no receive error  1: receive error in FIFO mode */

/*
 * Define macros for UARTMCR
 * UART Modem Control Register
 */
#define UARTMCR_RTS	(1 << 1)	/* 0: RTS_ output high, 1: RTS_ output low */
#define UARTMCR_LOOP	(1 << 4)	/* 0: normal  1: loopback mode */
#define UARTMCR_MCE	(1 << 7)	/* 0: modem function is disable */

/*
 * Define macros for UARTMSR
 * UART Modem Status Register
 */
#define UARTMSR_CCTS	(1 << 0)        /* 1: a change on CTS_ pin */
#define UARTMSR_CTS	(1 << 4)	/* 0: CTS_ pin is high */

/*
 * Define macros for SIRCR
 * Slow IrDA Control Register
 */
#define SIRCR_TSIRE	(1 << 0)  /* 0: transmitter is in UART mode  1: SIR mode */
#define SIRCR_RSIRE	(1 << 1)  /* 0: receiver is in UART mode  1: SIR mode */
#define SIRCR_TPWS	(1 << 2)  /* 0: transmit 0 pulse width is 3/16 of bit length
					   1: 0 pulse width is 1.6us for 115.2Kbps */
#define SIRCR_TDPL	(1 << 3)  /* 0: encoder generates a positive pulse for 0 */
#define SIRCR_RDPL	(1 << 4)  /* 0: decoder interprets positive pulse as 0 */


/*************************************************************************
 * AIC (AC97/I2S Controller)
 *************************************************************************/
#define	AIC_FR			(AIC_BASE + 0x000)
#define	AIC_CR			(AIC_BASE + 0x004)
#define	AIC_ACCR1		(AIC_BASE + 0x008)
#define	AIC_ACCR2		(AIC_BASE + 0x00C)
#define	AIC_I2SCR		(AIC_BASE + 0x010)
#define	AIC_SR			(AIC_BASE + 0x014)
#define	AIC_ACSR		(AIC_BASE + 0x018)
#define	AIC_I2SSR		(AIC_BASE + 0x01C)
#define	AIC_ACCAR		(AIC_BASE + 0x020)
#define	AIC_ACCDR		(AIC_BASE + 0x024)
#define	AIC_ACSAR		(AIC_BASE + 0x028)
#define	AIC_ACSDR		(AIC_BASE + 0x02C)
#define	AIC_I2SDIV		(AIC_BASE + 0x030)
#define	AIC_DR			(AIC_BASE + 0x034)

#define	REG_AIC_FR		REG32(AIC_FR)
#define	REG_AIC_CR		REG32(AIC_CR)
#define	REG_AIC_ACCR1		REG32(AIC_ACCR1)
#define	REG_AIC_ACCR2		REG32(AIC_ACCR2)
#define	REG_AIC_I2SCR		REG32(AIC_I2SCR)
#define	REG_AIC_SR		REG32(AIC_SR)
#define	REG_AIC_ACSR		REG32(AIC_ACSR)
#define	REG_AIC_I2SSR		REG32(AIC_I2SSR)
#define	REG_AIC_ACCAR		REG32(AIC_ACCAR)
#define	REG_AIC_ACCDR		REG32(AIC_ACCDR)
#define	REG_AIC_ACSAR		REG32(AIC_ACSAR)
#define	REG_AIC_ACSDR		REG32(AIC_ACSDR)
#define	REG_AIC_I2SDIV		REG32(AIC_I2SDIV)
#define	REG_AIC_DR		REG32(AIC_DR)

/* AIC Controller Configuration Register (AIC_FR) */

#define	AIC_FR_RFTH_BIT		12        /* Receive FIFO Threshold */
#define	AIC_FR_RFTH_MASK	(0xf << AIC_FR_RFTH_BIT)
#define	AIC_FR_TFTH_BIT		8         /* Transmit FIFO Threshold */
#define	AIC_FR_TFTH_MASK	(0xf << AIC_FR_TFTH_BIT)
#define	AIC_FR_LSMP		(1 << 6)  /* Play Zero sample or last sample */
#define	AIC_FR_ICDC		(1 << 5)  /* External(0) or Internal CODEC(1) */
#define	AIC_FR_AUSEL		(1 << 4)  /* AC97(0) or I2S/MSB-justified(1) */
#define	AIC_FR_RST		(1 << 3)  /* AIC registers reset */
#define	AIC_FR_BCKD		(1 << 2)  /* I2S BIT_CLK direction, 0:input,1:output */
#define	AIC_FR_SYNCD		(1 << 1)  /* I2S SYNC direction, 0:input,1:output */
#define	AIC_FR_ENB		(1 << 0)  /* AIC enable bit */

/* AIC Controller Common Control Register (AIC_CR) */

#define	AIC_CR_OSS_BIT		19  /* Output Sample Size from memory (AIC V2 only) */
#define	AIC_CR_OSS_MASK		(0x7 << AIC_CR_OSS_BIT)
  #define AIC_CR_OSS_8BIT	(0x0 << AIC_CR_OSS_BIT)
  #define AIC_CR_OSS_16BIT	(0x1 << AIC_CR_OSS_BIT)
  #define AIC_CR_OSS_18BIT	(0x2 << AIC_CR_OSS_BIT)
  #define AIC_CR_OSS_20BIT	(0x3 << AIC_CR_OSS_BIT)
  #define AIC_CR_OSS_24BIT	(0x4 << AIC_CR_OSS_BIT)
#define	AIC_CR_ISS_BIT		16  /* Input Sample Size from memory (AIC V2 only) */
#define	AIC_CR_ISS_MASK		(0x7 << AIC_CR_ISS_BIT)
  #define AIC_CR_ISS_8BIT	(0x0 << AIC_CR_ISS_BIT)
  #define AIC_CR_ISS_16BIT	(0x1 << AIC_CR_ISS_BIT)
  #define AIC_CR_ISS_18BIT	(0x2 << AIC_CR_ISS_BIT)
  #define AIC_CR_ISS_20BIT	(0x3 << AIC_CR_ISS_BIT)
  #define AIC_CR_ISS_24BIT	(0x4 << AIC_CR_ISS_BIT)
#define	AIC_CR_RDMS		(1 << 15)  /* Receive DMA enable */
#define	AIC_CR_TDMS		(1 << 14)  /* Transmit DMA enable */
#define	AIC_CR_M2S		(1 << 11)  /* Mono to Stereo enable */
#define	AIC_CR_ENDSW		(1 << 10)  /* Endian switch enable */
#define	AIC_CR_AVSTSU		(1 << 9)   /* Signed <-> Unsigned toggle enable */
#define	AIC_CR_FLUSH_TX		(1 << 8)   /* Flush TX FIFO */
#define	AIC_CR_FLUSH_RX		(1 << 7)   /* Flush RX FIFO */
#define	AIC_CR_EROR		(1 << 6)   /* Enable ROR interrupt */
#define	AIC_CR_ETUR		(1 << 5)   /* Enable TUR interrupt */
#define	AIC_CR_ERFS		(1 << 4)   /* Enable RFS interrupt */
#define	AIC_CR_ETFS		(1 << 3)   /* Enable TFS interrupt */
#define	AIC_CR_ENLBF		(1 << 2)   /* Enable Loopback Function */
#define	AIC_CR_ERPL		(1 << 1)   /* Enable Playback Function */
#define	AIC_CR_EREC		(1 << 0)   /* Enable Record Function */

/* AIC Controller AC-link Control Register 1 (AIC_ACCR1) */

#define	AIC_ACCR1_RS_BIT	16          /* Receive Valid Slots */
#define	AIC_ACCR1_RS_MASK	(0x3ff << AIC_ACCR1_RS_BIT)
  #define AIC_ACCR1_RS_SLOT12	  (1 << 25) /* Slot 12 valid bit */
  #define AIC_ACCR1_RS_SLOT11	  (1 << 24) /* Slot 11 valid bit */
  #define AIC_ACCR1_RS_SLOT10	  (1 << 23) /* Slot 10 valid bit */
  #define AIC_ACCR1_RS_SLOT9	  (1 << 22) /* Slot 9 valid bit, LFE */
  #define AIC_ACCR1_RS_SLOT8	  (1 << 21) /* Slot 8 valid bit, Surround Right */
  #define AIC_ACCR1_RS_SLOT7	  (1 << 20) /* Slot 7 valid bit, Surround Left */
  #define AIC_ACCR1_RS_SLOT6	  (1 << 19) /* Slot 6 valid bit, PCM Center */
  #define AIC_ACCR1_RS_SLOT5	  (1 << 18) /* Slot 5 valid bit */
  #define AIC_ACCR1_RS_SLOT4	  (1 << 17) /* Slot 4 valid bit, PCM Right */
  #define AIC_ACCR1_RS_SLOT3	  (1 << 16) /* Slot 3 valid bit, PCM Left */
#define	AIC_ACCR1_XS_BIT	0          /* Transmit Valid Slots */
#define	AIC_ACCR1_XS_MASK	(0x3ff << AIC_ACCR1_XS_BIT)
  #define AIC_ACCR1_XS_SLOT12	  (1 << 9) /* Slot 12 valid bit */
  #define AIC_ACCR1_XS_SLOT11	  (1 << 8) /* Slot 11 valid bit */
  #define AIC_ACCR1_XS_SLOT10	  (1 << 7) /* Slot 10 valid bit */
  #define AIC_ACCR1_XS_SLOT9	  (1 << 6) /* Slot 9 valid bit, LFE */
  #define AIC_ACCR1_XS_SLOT8	  (1 << 5) /* Slot 8 valid bit, Surround Right */
  #define AIC_ACCR1_XS_SLOT7	  (1 << 4) /* Slot 7 valid bit, Surround Left */
  #define AIC_ACCR1_XS_SLOT6	  (1 << 3) /* Slot 6 valid bit, PCM Center */
  #define AIC_ACCR1_XS_SLOT5	  (1 << 2) /* Slot 5 valid bit */
  #define AIC_ACCR1_XS_SLOT4	  (1 << 1) /* Slot 4 valid bit, PCM Right */
  #define AIC_ACCR1_XS_SLOT3	  (1 << 0) /* Slot 3 valid bit, PCM Left */

/* AIC Controller AC-link Control Register 2 (AIC_ACCR2) */

#define	AIC_ACCR2_ERSTO		(1 << 18) /* Enable RSTO interrupt */
#define	AIC_ACCR2_ESADR		(1 << 17) /* Enable SADR interrupt */
#define	AIC_ACCR2_ECADT		(1 << 16) /* Enable CADT interrupt */
#define	AIC_ACCR2_OASS_BIT	8  /* Output Sample Size for AC-link */
#define	AIC_ACCR2_OASS_MASK	(0x3 << AIC_ACCR2_OASS_BIT)
  #define AIC_ACCR2_OASS_20BIT	  (0 << AIC_ACCR2_OASS_BIT) /* Output Audio Sample Size is 20-bit */
  #define AIC_ACCR2_OASS_18BIT	  (1 << AIC_ACCR2_OASS_BIT) /* Output Audio Sample Size is 18-bit */
  #define AIC_ACCR2_OASS_16BIT	  (2 << AIC_ACCR2_OASS_BIT) /* Output Audio Sample Size is 16-bit */
  #define AIC_ACCR2_OASS_8BIT	  (3 << AIC_ACCR2_OASS_BIT) /* Output Audio Sample Size is 8-bit */
#define	AIC_ACCR2_IASS_BIT	6  /* Output Sample Size for AC-link */
#define	AIC_ACCR2_IASS_MASK	(0x3 << AIC_ACCR2_IASS_BIT)
  #define AIC_ACCR2_IASS_20BIT	  (0 << AIC_ACCR2_IASS_BIT) /* Input Audio Sample Size is 20-bit */
  #define AIC_ACCR2_IASS_18BIT	  (1 << AIC_ACCR2_IASS_BIT) /* Input Audio Sample Size is 18-bit */
  #define AIC_ACCR2_IASS_16BIT	  (2 << AIC_ACCR2_IASS_BIT) /* Input Audio Sample Size is 16-bit */
  #define AIC_ACCR2_IASS_8BIT	  (3 << AIC_ACCR2_IASS_BIT) /* Input Audio Sample Size is 8-bit */
#define	AIC_ACCR2_SO		(1 << 3)  /* SDATA_OUT output value */
#define	AIC_ACCR2_SR		(1 << 2)  /* RESET# pin level */
#define	AIC_ACCR2_SS		(1 << 1)  /* SYNC pin level */
#define	AIC_ACCR2_SA		(1 << 0)  /* SYNC and SDATA_OUT alternation */

/* AIC Controller I2S/MSB-justified Control Register (AIC_I2SCR) */

#define	AIC_I2SCR_STPBK		(1 << 12) /* Stop BIT_CLK for I2S/MSB-justified */
#define	AIC_I2SCR_WL_BIT	1  /* Input/Output Sample Size for I2S/MSB-justified */
#define	AIC_I2SCR_WL_MASK	(0x7 << AIC_I2SCR_WL_BIT)
  #define AIC_I2SCR_WL_24BIT	  (0 << AIC_I2SCR_WL_BIT) /* Word Length is 24 bit */
  #define AIC_I2SCR_WL_20BIT	  (1 << AIC_I2SCR_WL_BIT) /* Word Length is 20 bit */
  #define AIC_I2SCR_WL_18BIT	  (2 << AIC_I2SCR_WL_BIT) /* Word Length is 18 bit */
  #define AIC_I2SCR_WL_16BIT	  (3 << AIC_I2SCR_WL_BIT) /* Word Length is 16 bit */
  #define AIC_I2SCR_WL_8BIT	  (4 << AIC_I2SCR_WL_BIT) /* Word Length is 8 bit */
#define	AIC_I2SCR_AMSL		(1 << 0) /* 0:I2S, 1:MSB-justified */

/* AIC Controller FIFO Status Register (AIC_SR) */

#define	AIC_SR_RFL_BIT		24  /* Receive FIFO Level */
#define	AIC_SR_RFL_MASK		(0x3f << AIC_SR_RFL_BIT)
#define	AIC_SR_TFL_BIT		8   /* Transmit FIFO level */
#define	AIC_SR_TFL_MASK		(0x3f << AIC_SR_TFL_BIT)
#define	AIC_SR_ROR		(1 << 6) /* Receive FIFO Overrun */
#define	AIC_SR_TUR		(1 << 5) /* Transmit FIFO Underrun */
#define	AIC_SR_RFS		(1 << 4) /* Receive FIFO Service Request */
#define	AIC_SR_TFS		(1 << 3) /* Transmit FIFO Service Request */

/* AIC Controller AC-link Status Register (AIC_ACSR) */

#define	AIC_ACSR_SLTERR		(1 << 21) /* Slot Error Flag */
#define	AIC_ACSR_CRDY		(1 << 20) /* External CODEC Ready Flag */
#define	AIC_ACSR_CLPM		(1 << 19) /* External CODEC low power mode flag */
#define	AIC_ACSR_RSTO		(1 << 18) /* External CODEC regs read status timeout */
#define	AIC_ACSR_SADR		(1 << 17) /* External CODEC regs status addr and data received */
#define	AIC_ACSR_CADT		(1 << 16) /* Command Address and Data Transmitted */

/* AIC Controller I2S/MSB-justified Status Register (AIC_I2SSR) */

#define	AIC_I2SSR_BSY		(1 << 2)  /* AIC Busy in I2S/MSB-justified format */

/* AIC Controller AC97 codec Command Address Register (AIC_ACCAR) */

#define	AIC_ACCAR_CAR_BIT	0
#define	AIC_ACCAR_CAR_MASK	(0xfffff << AIC_ACCAR_CAR_BIT)

/* AIC Controller AC97 codec Command Data Register (AIC_ACCDR) */

#define	AIC_ACCDR_CDR_BIT	0
#define	AIC_ACCDR_CDR_MASK	(0xfffff << AIC_ACCDR_CDR_BIT)

/* AIC Controller AC97 codec Status Address Register (AIC_ACSAR) */

#define	AIC_ACSAR_SAR_BIT	0
#define	AIC_ACSAR_SAR_MASK	(0xfffff << AIC_ACSAR_SAR_BIT)

/* AIC Controller AC97 codec Status Data Register (AIC_ACSDR) */

#define	AIC_ACSDR_SDR_BIT	0
#define	AIC_ACSDR_SDR_MASK	(0xfffff << AIC_ACSDR_SDR_BIT)

/* AIC Controller I2S/MSB-justified Clock Divider Register (AIC_I2SDIV) */

#define	AIC_I2SDIV_DIV_BIT	0
#define	AIC_I2SDIV_DIV_MASK	(0x7f << AIC_I2SDIV_DIV_BIT)
  #define AIC_I2SDIV_BITCLK_3072KHZ	(0x0C << AIC_I2SDIV_DIV_BIT) /* BIT_CLK of 3.072MHz */
  #define AIC_I2SDIV_BITCLK_2836KHZ	(0x0D << AIC_I2SDIV_DIV_BIT) /* BIT_CLK of 2.836MHz */
  #define AIC_I2SDIV_BITCLK_1418KHZ	(0x1A << AIC_I2SDIV_DIV_BIT) /* BIT_CLK of 1.418MHz */
  #define AIC_I2SDIV_BITCLK_1024KHZ	(0x24 << AIC_I2SDIV_DIV_BIT) /* BIT_CLK of 1.024MHz */
  #define AIC_I2SDIV_BITCLK_7089KHZ	(0x34 << AIC_I2SDIV_DIV_BIT) /* BIT_CLK of 708.92KHz */
  #define AIC_I2SDIV_BITCLK_512KHZ	(0x48 << AIC_I2SDIV_DIV_BIT) /* BIT_CLK of 512.00KHz */


/*************************************************************************
 * ICDC (Internal CODEC)
 *************************************************************************/

#define	ICDC_CKCFG	  (ICDC_BASE + 0x00a0)  /* Clock Configure Register */
#define	ICDC_RGADW	  (ICDC_BASE + 0x00a4)  /* internal register access control */
#define	ICDC_RGDATA	  (ICDC_BASE + 0x00a8)  /* internal register data output */

#define	REG_ICDC_CKCFG		REG32(ICDC_CKCFG)
#define	REG_ICDC_RGADW		REG32(ICDC_RGADW)
#define	REG_ICDC_RGDATA		REG32(ICDC_RGDATA)

/* ICDC Clock Configure Register */
#define	ICDC_CKCFG_CKRDY	(1 << 1)
#define	ICDC_CKCFG_SELAD	(1 << 0)

/* ICDC internal register access control Register */
#define ICDC_RGADW_RGWR         (1 << 16)
#define ICDC_RGADW_RGADDR_BIT   8
#define	ICDC_RGADW_RGADDR_MASK	(0x7f << ICDC_RGADW_RGADDR_BIT)
#define ICDC_RGADW_RGDIN_BIT    0
#define	ICDC_RGADW_RGDIN_MASK	(0xff << ICDC_RGADW_RGDIN_BIT)

/* ICDC internal register data output Register */
#define ICDC_RGDATA_IRQ         (1 << 8)
#define ICDC_RGDATA_RGDOUT_BIT  0
#define ICDC_RGDATA_RGDOUT_MASK (0xff << ICDC_RGDATA_RGDOUT_BIT)

/*************************************************************************
 * PCM Controller
 *************************************************************************/

#define PCM_CTL                 (PCM_BASE + 0x000)
#define PCM_CFG                 (PCM_BASE + 0x004)
#define PCM_DP                  (PCM_BASE + 0x008)
#define PCM_INTC                (PCM_BASE + 0x00c)
#define PCM_INTS                (PCM_BASE + 0x010)
#define PCM_DIV                 (PCM_BASE + 0x014)

#define REG_PCM_CTL             REG32(PCM_CTL)
#define REG_PCM_CFG             REG32(PCM_CFG)
#define REG_PCM_DP              REG32(PCM_DP)
#define REG_PCM_INTC            REG32(PCM_INTC)
#define REG_PCM_INTS            REG32(PCM_INTS)
#define REG_PCM_DIV             REG32(PCM_DIV)

/* PCM Controller control Register (PCM_CTL) */

#define PCM_CTL_ERDMA		(1 << 9)  /* Enable Receive DMA */
#define PCM_CTL_ETDMA           (1 << 8)  /* Enable Transmit DMA */
#define PCM_CTL_LSMP		(1 << 7)  /* Play Zero sample or last sample */
#define PCM_CTL_ERPL            (1 << 6)  /* Enable Playing Back Function */
#define PCM_CTL_EREC            (1 << 5)  /* Enable Recording Function */
#define PCM_CTL_FLUSH           (1 << 4)  /* FIFO flush */
#define PCM_CTL_RST             (1 << 3)  /* Reset PCM */
#define PCM_CTL_CLKEN           (1 << 1)  /* Enable the clock division logic */
#define PCM_CTL_PCMEN           (1 << 0)  /* Enable PCM module */

/* PCM Controller configure Register (PCM_CFG) */

#define PCM_CFG_SLOT_BIT        13
#define PCM_CFG_SLOT_MASK       (0x3 << PCM_CFG_SLOT_BIT)
  #define PCM_CFG_SLOT_0	  (0 << PCM_CFG_SLOT_BIT) /* Slot is 0 */
  #define PCM_CFG_SLOT_1	  (1 << PCM_CFG_SLOT_BIT) /* Slot is 1 */
  #define PCM_CFG_SLOT_2	  (2 << PCM_CFG_SLOT_BIT) /* Slot is 2 */
  #define PCM_CFG_SLOT_3	  (3 << PCM_CFG_SLOT_BIT) /* Slot is 3 */
#define PCM_CFG_ISS_BIT         12
#define PCM_CFG_ISS_MASK        (0x1 << PCM_CFG_ISS_BIT)
  #define PCM_CFG_ISS_8           (0 << PCM_CFG_ISS_BIT)
  #define PCM_CFG_ISS_16          (1 << PCM_CFG_ISS_BIT)
#define PCM_CFG_OSS_BIT         11
#define PCM_CFG_OSS_MASK        (0x1 << PCM_CFG_OSS_BIT)
  #define PCM_CFG_OSS_8           (0 << PCM_CFG_OSS_BIT)
  #define PCM_CFG_OSS_16          (1 << PCM_CFG_OSS_BIT)
#define PCM_CFG_IMSBPOS         (1 << 10)
#define PCM_CFG_OMSBPOS         (1 << 9)
#define	PCM_CFG_RFTH_BIT	5        /* Receive FIFO Threshold */
#define	PCM_CFG_RFTH_MASK	(0xf << PCM_CFG_RFTH_BIT)
#define	PCM_CFG_TFTH_BIT	1         /* Transmit FIFO Threshold */
#define	PCM_CFG_TFTH_MASK	(0xf << PCM_CFG_TFTH_BIT)
#define PCM_CFG_MODE            (0x0 << 0)

/* PCM Controller interrupt control Register (PCM_INTC) */

#define PCM_INTC_ETFS           (1 << 3)
#define PCM_INTC_ETUR           (1 << 2)
#define PCM_INTC_ERFS           (1 << 1)
#define PCM_INTC_EROR           (1 << 0)

/* PCM Controller interrupt status Register (PCM_INTS) */

#define PCM_INTS_RSTS		(1 << 14) /* Reset or flush has not complete */
#define PCM_INTS_TFL_BIT        9
#define PCM_INTS_TFL_MASK       (0x1f << PCM_INTS_TFL_BIT)
#define PCM_INTS_TFS		(1 << 8) /* Tranmit FIFO Service Request */
#define PCM_INTS_TUR		(1 << 7) /* Transmit FIFO Under Run */
#define PCM_INTS_RFL_BIT        2
#define PCM_INTS_RFL_MASK       (0x1f << PCM_INTS_RFL_BIT)
#define PCM_INTS_RFS		(1 << 1) /* Receive FIFO Service Request */
#define PCM_INTS_ROR		(1 << 0) /* Receive FIFO Over Run */

/* PCM Controller clock division Register (PCM_DIV) */
#define PCM_DIV_SYNL_BIT        11
#define PCM_DIV_SYNL_MASK       (0x3f << PCM_DIV_SYNL_BIT)
#define PCM_DIV_SYNDIV_BIT      6
#define PCM_DIV_SYNDIV_MASK     (0x1f << PCM_DIV_SYNDIV_BIT)
#define PCM_DIV_CLKDIV_BIT      0
#define PCM_DIV_CLKDIV_MASK     (0x3f << PCM_DIV_CLKDIV_BIT)


/*************************************************************************
 * I2C
 *************************************************************************/
#define	I2C_DR			(I2C_BASE + 0x000)
#define	I2C_CR			(I2C_BASE + 0x004)
#define	I2C_SR			(I2C_BASE + 0x008)
#define	I2C_GR			(I2C_BASE + 0x00C)

#define	REG_I2C_DR		REG8(I2C_DR)
#define	REG_I2C_CR		REG8(I2C_CR)
#define REG_I2C_SR		REG8(I2C_SR)
#define REG_I2C_GR		REG16(I2C_GR)

/* I2C Control Register (I2C_CR) */

#define I2C_CR_IEN		(1 << 4)
#define I2C_CR_STA		(1 << 3)
#define I2C_CR_STO		(1 << 2)
#define I2C_CR_AC		(1 << 1)
#define I2C_CR_I2CE		(1 << 0)

/* I2C Status Register (I2C_SR) */

#define I2C_SR_STX		(1 << 4)
#define I2C_SR_BUSY		(1 << 3)
#define I2C_SR_TEND		(1 << 2)
#define I2C_SR_DRF		(1 << 1)
#define I2C_SR_ACKF		(1 << 0)


/*************************************************************************
 * SSI (Synchronous Serial Interface)
 *************************************************************************/
/* n = 0, 1 (SSI0, SSI1) */
#define	SSI_DR(n)		(SSI_BASE + 0x000 + (n)*0x2000)
#define	SSI_CR0(n)		(SSI_BASE + 0x004 + (n)*0x2000)
#define	SSI_CR1(n)		(SSI_BASE + 0x008 + (n)*0x2000)
#define	SSI_SR(n)		(SSI_BASE + 0x00C + (n)*0x2000)
#define	SSI_ITR(n)		(SSI_BASE + 0x010 + (n)*0x2000)
#define	SSI_ICR(n)		(SSI_BASE + 0x014 + (n)*0x2000)
#define	SSI_GR(n)		(SSI_BASE + 0x018 + (n)*0x2000)

#define	REG_SSI_DR(n)		REG32(SSI_DR(n))
#define	REG_SSI_CR0(n)		REG16(SSI_CR0(n))
#define	REG_SSI_CR1(n)		REG32(SSI_CR1(n))
#define	REG_SSI_SR(n)		REG32(SSI_SR(n))
#define	REG_SSI_ITR(n)		REG16(SSI_ITR(n))
#define	REG_SSI_ICR(n)		REG8(SSI_ICR(n))
#define	REG_SSI_GR(n)		REG16(SSI_GR(n))

/* SSI Data Register (SSI_DR) */

#define	SSI_DR_GPC_BIT		0
#define	SSI_DR_GPC_MASK		(0x1ff << SSI_DR_GPC_BIT)

#define SSI_MAX_FIFO_ENTRIES 	128 /* 128 txfifo and 128 rxfifo */

/* SSI Control Register 0 (SSI_CR0) */

#define SSI_CR0_SSIE		(1 << 15)
#define SSI_CR0_TIE		(1 << 14)
#define SSI_CR0_RIE		(1 << 13)
#define SSI_CR0_TEIE		(1 << 12)
#define SSI_CR0_REIE		(1 << 11)
#define SSI_CR0_LOOP		(1 << 10)
#define SSI_CR0_RFINE		(1 << 9)
#define SSI_CR0_RFINC		(1 << 8)
#define SSI_CR0_EACLRUN		(1 << 7) /* hardware auto clear underrun when TxFifo no empty */
#define SSI_CR0_FSEL		(1 << 6)
#define SSI_CR0_TFLUSH		(1 << 2)
#define SSI_CR0_RFLUSH		(1 << 1)
#define SSI_CR0_DISREV		(1 << 0)

/* SSI Control Register 1 (SSI_CR1) */

#define SSI_CR1_FRMHL_BIT	30
#define SSI_CR1_FRMHL_MASK	(0x3 << SSI_CR1_FRMHL_BIT)
  #define SSI_CR1_FRMHL_CELOW_CE2LOW	(0 << SSI_CR1_FRMHL_BIT) /* SSI_CE_ is low valid and SSI_CE2_ is low valid */
  #define SSI_CR1_FRMHL_CEHIGH_CE2LOW	(1 << SSI_CR1_FRMHL_BIT) /* SSI_CE_ is high valid and SSI_CE2_ is low valid */
  #define SSI_CR1_FRMHL_CELOW_CE2HIGH	(2 << SSI_CR1_FRMHL_BIT) /* SSI_CE_ is low valid  and SSI_CE2_ is high valid */
  #define SSI_CR1_FRMHL_CEHIGH_CE2HIGH	(3 << SSI_CR1_FRMHL_BIT) /* SSI_CE_ is high valid and SSI_CE2_ is high valid */
#define SSI_CR1_TFVCK_BIT	28
#define SSI_CR1_TFVCK_MASK	(0x3 << SSI_CR1_TFVCK_BIT)
  #define SSI_CR1_TFVCK_0	  (0 << SSI_CR1_TFVCK_BIT)
  #define SSI_CR1_TFVCK_1	  (1 << SSI_CR1_TFVCK_BIT)
  #define SSI_CR1_TFVCK_2	  (2 << SSI_CR1_TFVCK_BIT)
  #define SSI_CR1_TFVCK_3	  (3 << SSI_CR1_TFVCK_BIT)
#define SSI_CR1_TCKFI_BIT	26
#define SSI_CR1_TCKFI_MASK	(0x3 << SSI_CR1_TCKFI_BIT)
  #define SSI_CR1_TCKFI_0	  (0 << SSI_CR1_TCKFI_BIT)
  #define SSI_CR1_TCKFI_1	  (1 << SSI_CR1_TCKFI_BIT)
  #define SSI_CR1_TCKFI_2	  (2 << SSI_CR1_TCKFI_BIT)
  #define SSI_CR1_TCKFI_3	  (3 << SSI_CR1_TCKFI_BIT)
#define SSI_CR1_LFST		(1 << 25)
#define SSI_CR1_ITFRM		(1 << 24)
#define SSI_CR1_UNFIN		(1 << 23)
#define SSI_CR1_MULTS		(1 << 22)
#define SSI_CR1_FMAT_BIT	20
#define SSI_CR1_FMAT_MASK	(0x3 << SSI_CR1_FMAT_BIT)
  #define SSI_CR1_FMAT_SPI	  (0 << SSI_CR1_FMAT_BIT) /* Motorola¡¯s SPI format */
  #define SSI_CR1_FMAT_SSP	  (1 << SSI_CR1_FMAT_BIT) /* TI's SSP format */
  #define SSI_CR1_FMAT_MW1	  (2 << SSI_CR1_FMAT_BIT) /* National Microwire 1 format */
  #define SSI_CR1_FMAT_MW2	  (3 << SSI_CR1_FMAT_BIT) /* National Microwire 2 format */
#define SSI_CR1_TTRG_BIT	16 /* SSI1 TX trigger */
#define SSI_CR1_TTRG_MASK	(0xf << SSI_CR1_TTRG_BIT)
#define SSI_CR1_MCOM_BIT	12
#define SSI_CR1_MCOM_MASK	(0xf << SSI_CR1_MCOM_BIT)
  #define SSI_CR1_MCOM_1BIT	  (0x0 << SSI_CR1_MCOM_BIT) /* 1-bit command selected */
  #define SSI_CR1_MCOM_2BIT	  (0x1 << SSI_CR1_MCOM_BIT) /* 2-bit command selected */
  #define SSI_CR1_MCOM_3BIT	  (0x2 << SSI_CR1_MCOM_BIT) /* 3-bit command selected */
  #define SSI_CR1_MCOM_4BIT	  (0x3 << SSI_CR1_MCOM_BIT) /* 4-bit command selected */
  #define SSI_CR1_MCOM_5BIT	  (0x4 << SSI_CR1_MCOM_BIT) /* 5-bit command selected */
  #define SSI_CR1_MCOM_6BIT	  (0x5 << SSI_CR1_MCOM_BIT) /* 6-bit command selected */
  #define SSI_CR1_MCOM_7BIT	  (0x6 << SSI_CR1_MCOM_BIT) /* 7-bit command selected */
  #define SSI_CR1_MCOM_8BIT	  (0x7 << SSI_CR1_MCOM_BIT) /* 8-bit command selected */
  #define SSI_CR1_MCOM_9BIT	  (0x8 << SSI_CR1_MCOM_BIT) /* 9-bit command selected */
  #define SSI_CR1_MCOM_10BIT	  (0x9 << SSI_CR1_MCOM_BIT) /* 10-bit command selected */
  #define SSI_CR1_MCOM_11BIT	  (0xA << SSI_CR1_MCOM_BIT) /* 11-bit command selected */
  #define SSI_CR1_MCOM_12BIT	  (0xB << SSI_CR1_MCOM_BIT) /* 12-bit command selected */
  #define SSI_CR1_MCOM_13BIT	  (0xC << SSI_CR1_MCOM_BIT) /* 13-bit command selected */
  #define SSI_CR1_MCOM_14BIT	  (0xD << SSI_CR1_MCOM_BIT) /* 14-bit command selected */
  #define SSI_CR1_MCOM_15BIT	  (0xE << SSI_CR1_MCOM_BIT) /* 15-bit command selected */
  #define SSI_CR1_MCOM_16BIT	  (0xF << SSI_CR1_MCOM_BIT) /* 16-bit command selected */
#define SSI_CR1_RTRG_BIT	8 /* SSI RX trigger */
#define SSI_CR1_RTRG_MASK	(0xf << SSI_CR1_RTRG_BIT)
#define SSI_CR1_FLEN_BIT	4
#define SSI_CR1_FLEN_MASK	(0xf << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_2BIT	  (0x0 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_3BIT	  (0x1 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_4BIT	  (0x2 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_5BIT	  (0x3 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_6BIT	  (0x4 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_7BIT	  (0x5 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_8BIT	  (0x6 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_9BIT	  (0x7 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_10BIT	  (0x8 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_11BIT	  (0x9 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_12BIT	  (0xA << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_13BIT	  (0xB << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_14BIT	  (0xC << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_15BIT	  (0xD << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_16BIT	  (0xE << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_17BIT	  (0xF << SSI_CR1_FLEN_BIT)
#define SSI_CR1_PHA		(1 << 1)
#define SSI_CR1_POL		(1 << 0)

/* SSI Status Register (SSI_SR) */

#define SSI_SR_TFIFONUM_BIT	16
#define SSI_SR_TFIFONUM_MASK	(0xff << SSI_SR_TFIFONUM_BIT)
#define SSI_SR_RFIFONUM_BIT	8
#define SSI_SR_RFIFONUM_MASK	(0xff << SSI_SR_RFIFONUM_BIT)
#define SSI_SR_END		(1 << 7)
#define SSI_SR_BUSY		(1 << 6)
#define SSI_SR_TFF		(1 << 5)
#define SSI_SR_RFE		(1 << 4)
#define SSI_SR_TFHE		(1 << 3)
#define SSI_SR_RFHF		(1 << 2)
#define SSI_SR_UNDR		(1 << 1)
#define SSI_SR_OVER		(1 << 0)

/* SSI Interval Time Control Register (SSI_ITR) */

#define	SSI_ITR_CNTCLK		(1 << 15)
#define SSI_ITR_IVLTM_BIT	0
#define SSI_ITR_IVLTM_MASK	(0x7fff << SSI_ITR_IVLTM_BIT)


/*************************************************************************
 * MSC
 ************************************************************************/
#define JZ_MAX_MSC_NUM 2

#define JZ_MSC_ID_INVALID(msc_id) ( ((msc_id) < 0) || ( (msc_id) > JZ_MAX_MSC_NUM ) )
/* n = 0, 1 (MSC0, MSC1) */
#define	MSC_STRPCL(n)		(MSC_BASE + (n)*0x1000 + 0x000)
#define	MSC_STAT(n)		(MSC_BASE + (n)*0x1000 + 0x004)
#define	MSC_CLKRT(n)		(MSC_BASE + (n)*0x1000 + 0x008)
#define	MSC_CMDAT(n)		(MSC_BASE + (n)*0x1000 + 0x00C)
#define	MSC_RESTO(n)		(MSC_BASE + (n)*0x1000 + 0x010)
#define	MSC_RDTO(n)		(MSC_BASE + (n)*0x1000 + 0x014)
#define	MSC_BLKLEN(n)		(MSC_BASE + (n)*0x1000 + 0x018)
#define	MSC_NOB(n)		(MSC_BASE + (n)*0x1000 + 0x01C)
#define	MSC_SNOB(n)		(MSC_BASE + (n)*0x1000 + 0x020)
#define	MSC_IMASK(n)		(MSC_BASE + (n)*0x1000 + 0x024)
#define	MSC_IREG(n)		(MSC_BASE + (n)*0x1000 + 0x028)
#define	MSC_CMD(n)		(MSC_BASE + (n)*0x1000 + 0x02C)
#define	MSC_ARG(n)		(MSC_BASE + (n)*0x1000 + 0x030)
#define	MSC_RES(n)		(MSC_BASE + (n)*0x1000 + 0x034)
#define	MSC_RXFIFO(n)		(MSC_BASE + (n)*0x1000 + 0x038)
#define	MSC_TXFIFO(n)		(MSC_BASE + (n)*0x1000 + 0x03C)
#define	MSC_LPM(n)		(MSC_BASE + (n)*0x1000 + 0x040)

#define	REG_MSC_STRPCL(n)	REG16(MSC_STRPCL(n))
#define	REG_MSC_STAT(n)		REG32(MSC_STAT(n))
#define	REG_MSC_CLKRT(n)	REG16(MSC_CLKRT(n))
#define	REG_MSC_CMDAT(n)	REG32(MSC_CMDAT(n))
#define	REG_MSC_RESTO(n)	REG16(MSC_RESTO(n))
#define	REG_MSC_RDTO(n)		REG32(MSC_RDTO(n))
#define	REG_MSC_BLKLEN(n)	REG16(MSC_BLKLEN(n))
#define	REG_MSC_NOB(n)		REG16(MSC_NOB(n))
#define	REG_MSC_SNOB(n)		REG16(MSC_SNOB(n))
#define	REG_MSC_IMASK(n)	REG32(MSC_IMASK(n))
#define	REG_MSC_IREG(n)		REG16(MSC_IREG(n))
#define	REG_MSC_CMD(n)		REG8(MSC_CMD(n))
#define	REG_MSC_ARG(n)		REG32(MSC_ARG(n))
#define	REG_MSC_RES(n)		REG16(MSC_RES(n))
#define	REG_MSC_RXFIFO(n)	REG32(MSC_RXFIFO(n))
#define	REG_MSC_TXFIFO(n)	REG32(MSC_TXFIFO(n))
#define	REG_MSC_LPM(n)		REG32(MSC_LPM(n))

/* MSC Clock and Control Register (MSC_STRPCL) */
#define MSC_STRPCL_SEND_CCSD		(1 << 15) /*send command completion signal disable to ceata */
#define MSC_STRPCL_SEND_AS_CCSD		(1 << 14) /*send internally generated stop after sending ccsd */
#define MSC_STRPCL_EXIT_MULTIPLE	(1 << 7)
#define MSC_STRPCL_EXIT_TRANSFER	(1 << 6)
#define MSC_STRPCL_START_READWAIT	(1 << 5)
#define MSC_STRPCL_STOP_READWAIT	(1 << 4)
#define MSC_STRPCL_RESET		(1 << 3)
#define MSC_STRPCL_START_OP		(1 << 2)
#define MSC_STRPCL_CLOCK_CONTROL_BIT	0
#define MSC_STRPCL_CLOCK_CONTROL_MASK	(0x3 << MSC_STRPCL_CLOCK_CONTROL_BIT)
  #define MSC_STRPCL_CLOCK_CONTROL_STOP	  (0x1 << MSC_STRPCL_CLOCK_CONTROL_BIT) /* Stop MMC/SD clock */
  #define MSC_STRPCL_CLOCK_CONTROL_START  (0x2 << MSC_STRPCL_CLOCK_CONTROL_BIT) /* Start MMC/SD clock */

/* MSC Status Register (MSC_STAT) */
#define MSC_STAT_AUTO_CMD_DONE		(1 << 31) /*12 is internally generated by controller has finished */
#define MSC_STAT_IS_RESETTING		(1 << 15)
#define MSC_STAT_SDIO_INT_ACTIVE	(1 << 14)
#define MSC_STAT_PRG_DONE		(1 << 13)
#define MSC_STAT_DATA_TRAN_DONE		(1 << 12)
#define MSC_STAT_END_CMD_RES		(1 << 11)
#define MSC_STAT_DATA_FIFO_AFULL	(1 << 10)
#define MSC_STAT_IS_READWAIT		(1 << 9)
#define MSC_STAT_CLK_EN			(1 << 8)
#define MSC_STAT_DATA_FIFO_FULL		(1 << 7)
#define MSC_STAT_DATA_FIFO_EMPTY	(1 << 6)
#define MSC_STAT_CRC_RES_ERR		(1 << 5)
#define MSC_STAT_CRC_READ_ERROR		(1 << 4)
#define MSC_STAT_CRC_WRITE_ERROR_BIT	2
#define MSC_STAT_CRC_WRITE_ERROR_MASK	(0x3 << MSC_STAT_CRC_WRITE_ERROR_BIT)
  #define MSC_STAT_CRC_WRITE_ERROR_NO		(0 << MSC_STAT_CRC_WRITE_ERROR_BIT) /* No error on transmission of data */
  #define MSC_STAT_CRC_WRITE_ERROR		(1 << MSC_STAT_CRC_WRITE_ERROR_BIT) /* Card observed erroneous transmission of data */
  #define MSC_STAT_CRC_WRITE_ERROR_NOSTS	(2 << MSC_STAT_CRC_WRITE_ERROR_BIT) /* No CRC status is sent back */
#define MSC_STAT_TIME_OUT_RES		(1 << 1)
#define MSC_STAT_TIME_OUT_READ		(1 << 0)

/* MSC Bus Clock Control Register (MSC_CLKRT) */
#define	MSC_CLKRT_CLK_RATE_BIT		0
#define	MSC_CLKRT_CLK_RATE_MASK		(0x7 << MSC_CLKRT_CLK_RATE_BIT)
  #define MSC_CLKRT_CLK_RATE_DIV_1	(0x0 << MSC_CLKRT_CLK_RATE_BIT) /* CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_2	(0x1 << MSC_CLKRT_CLK_RATE_BIT) /* 1/2 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_4	(0x2 << MSC_CLKRT_CLK_RATE_BIT) /* 1/4 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_8	(0x3 << MSC_CLKRT_CLK_RATE_BIT) /* 1/8 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_16	(0x4 << MSC_CLKRT_CLK_RATE_BIT) /* 1/16 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_32	(0x5 << MSC_CLKRT_CLK_RATE_BIT) /* 1/32 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_64	(0x6 << MSC_CLKRT_CLK_RATE_BIT) /* 1/64 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_128	(0x7 << MSC_CLKRT_CLK_RATE_BIT) /* 1/128 of CLK_SRC */

/* MSC Command Sequence Control Register (MSC_CMDAT) */
#define	MSC_CMDAT_CCS_EXPECTED		(1 << 31) /* interrupts are enabled in ce-ata */
#define	MSC_CMDAT_READ_CEATA		(1 << 30)
#define	MSC_CMDAT_SDIO_PRDT		(1 << 17) /* exact 2 cycle */
#define	MSC_CMDAT_SEND_AS_STOP		(1 << 16)
#define	MSC_CMDAT_RTRG_BIT		14
  #define MSC_CMDAT_RTRG_EQUALT_8	(0x0 << MSC_CMDAT_RTRG_BIT)
  #define MSC_CMDAT_RTRG_EQUALT_16	(0x1 << MSC_CMDAT_RTRG_BIT) /* reset value */
  #define MSC_CMDAT_RTRG_EQUALT_24	(0x2 << MSC_CMDAT_RTRG_BIT)

#define	MSC_CMDAT_TTRG_BIT		12
  #define MSC_CMDAT_TTRG_LESS_8		(0x0 << MSC_CMDAT_TTRG_BIT)
  #define MSC_CMDAT_TTRG_LESS_16	(0x1 << MSC_CMDAT_TTRG_BIT) /*reset value  */
  #define MSC_CMDAT_TTRG_LESS_24	(0x2 << MSC_CMDAT_TTRG_BIT)
#define	MSC_CMDAT_STOP_ABORT		(1 << 11)
#define	MSC_CMDAT_BUS_WIDTH_BIT		9
#define	MSC_CMDAT_BUS_WIDTH_MASK	(0x3 << MSC_CMDAT_BUS_WIDTH_BIT)
  #define MSC_CMDAT_BUS_WIDTH_1BIT	(0x0 << MSC_CMDAT_BUS_WIDTH_BIT) /* 1-bit data bus */
  #define MSC_CMDAT_BUS_WIDTH_4BIT	(0x2 << MSC_CMDAT_BUS_WIDTH_BIT) /* 4-bit data bus */
  #define MSC_CMDAT_BUS_WIDTH_8BIT	(0x3 << MSC_CMDAT_BUS_WIDTH_BIT) /* 8-bit data bus */
#define	MSC_CMDAT_DMA_EN		(1 << 8)
#define	MSC_CMDAT_INIT			(1 << 7)
#define	MSC_CMDAT_BUSY			(1 << 6)
#define	MSC_CMDAT_STREAM_BLOCK		(1 << 5)
#define	MSC_CMDAT_WRITE			(1 << 4)
#define	MSC_CMDAT_READ			(0 << 4)
#define	MSC_CMDAT_DATA_EN		(1 << 3)
#define	MSC_CMDAT_RESPONSE_BIT	0
#define	MSC_CMDAT_RESPONSE_MASK	(0x7 << MSC_CMDAT_RESPONSE_BIT)
  #define MSC_CMDAT_RESPONSE_NONE (0x0 << MSC_CMDAT_RESPONSE_BIT) /* No response */
  #define MSC_CMDAT_RESPONSE_R1	  (0x1 << MSC_CMDAT_RESPONSE_BIT) /* Format R1 and R1b */
  #define MSC_CMDAT_RESPONSE_R2	  (0x2 << MSC_CMDAT_RESPONSE_BIT) /* Format R2 */
  #define MSC_CMDAT_RESPONSE_R3	  (0x3 << MSC_CMDAT_RESPONSE_BIT) /* Format R3 */
  #define MSC_CMDAT_RESPONSE_R4	  (0x4 << MSC_CMDAT_RESPONSE_BIT) /* Format R4 */
  #define MSC_CMDAT_RESPONSE_R5	  (0x5 << MSC_CMDAT_RESPONSE_BIT) /* Format R5 */
  #define MSC_CMDAT_RESPONSE_R6	  (0x6 << MSC_CMDAT_RESPONSE_BIT) /* Format R6 */

#define	CMDAT_DMA_EN	(1 << 8)
#define	CMDAT_INIT	(1 << 7)
#define	CMDAT_BUSY	(1 << 6)
#define	CMDAT_STREAM	(1 << 5)
#define	CMDAT_WRITE	(1 << 4)
#define	CMDAT_DATA_EN	(1 << 3)

/* MSC Interrupts Mask Register (MSC_IMASK) */
#define	MSC_IMASK_AUTO_CMD_DONE		(1 << 8)
#define	MSC_IMASK_SDIO			(1 << 7)
#define	MSC_IMASK_TXFIFO_WR_REQ		(1 << 6)
#define	MSC_IMASK_RXFIFO_RD_REQ		(1 << 5)
#define	MSC_IMASK_END_CMD_RES		(1 << 2)
#define	MSC_IMASK_PRG_DONE		(1 << 1)
#define	MSC_IMASK_DATA_TRAN_DONE	(1 << 0)

/* MSC Interrupts Status Register (MSC_IREG) */
#define	MSC_IREG_AUTO_CMD_DONE		(1 << 8)
#define	MSC_IREG_SDIO			(1 << 7)
#define	MSC_IREG_TXFIFO_WR_REQ		(1 << 6)
#define	MSC_IREG_RXFIFO_RD_REQ		(1 << 5)
#define	MSC_IREG_END_CMD_RES		(1 << 2)
#define	MSC_IREG_PRG_DONE		(1 << 1)
#define	MSC_IREG_DATA_TRAN_DONE		(1 << 0)

/* MSC Low Power Mode Register (MSC_LPM) */
#define	MSC_SET_LPM			(1 << 0)

/*************************************************************************
 * EMC (External Memory Controller)
 *************************************************************************/
#define EMC_BCR    	(EMC_BASE + 0x00)  /* Bus Control Register */
#define EMC_SMCR0	(EMC_BASE + 0x10)  /* Static Memory Control Register 0 */
#define EMC_SMCR1	(EMC_BASE + 0x14)  /* Static Memory Control Register 1 */
#define EMC_SMCR2	(EMC_BASE + 0x18)  /* Static Memory Control Register 2 */
#define EMC_SMCR3	(EMC_BASE + 0x1c)  /* Static Memory Control Register 3 */
#define EMC_SMCR4	(EMC_BASE + 0x20)  /* Static Memory Control Register 4 */
#define EMC_SACR0	(EMC_BASE + 0x30)  /* Static Memory Bank 0 Addr Config Reg */
#define EMC_SACR1	(EMC_BASE + 0x34)  /* Static Memory Bank 1 Addr Config Reg */
#define EMC_SACR2	(EMC_BASE + 0x38)  /* Static Memory Bank 2 Addr Config Reg */
#define EMC_SACR3	(EMC_BASE + 0x3c)  /* Static Memory Bank 3 Addr Config Reg */
#define EMC_SACR4	(EMC_BASE + 0x40)  /* Static Memory Bank 4 Addr Config Reg */

#define EMC_NFCSR	(EMC_BASE + 0x050) /* NAND Flash Control/Status Register */

#define EMC_DMCR	(EMC_BASE + 0x80)  /* DRAM Control Register */
#define EMC_RTCSR	(EMC_BASE + 0x84)  /* Refresh Time Control/Status Register */
#define EMC_RTCNT	(EMC_BASE + 0x88)  /* Refresh Timer Counter */
#define EMC_RTCOR	(EMC_BASE + 0x8c)  /* Refresh Time Constant Register */
#define EMC_DMAR0	(EMC_BASE + 0x90)  /* SDRAM Bank 0 Addr Config Register */
#define EMC_DMAR1	(EMC_BASE + 0x94)  /* SDRAM Bank 1 Addr Config Register */
#define EMC_SDMR0	(EMC_BASE + 0xa000) /* Mode Register of SDRAM bank 0 */

#define REG_EMC_BCR 	REG32(EMC_BCR)
#define REG_EMC_SMCR0	REG32(EMC_SMCR0)
#define REG_EMC_SMCR1	REG32(EMC_SMCR1)
#define REG_EMC_SMCR2	REG32(EMC_SMCR2)
#define REG_EMC_SMCR3	REG32(EMC_SMCR3)
#define REG_EMC_SMCR4	REG32(EMC_SMCR4)
#define REG_EMC_SACR0	REG32(EMC_SACR0)
#define REG_EMC_SACR1	REG32(EMC_SACR1)
#define REG_EMC_SACR2	REG32(EMC_SACR2)
#define REG_EMC_SACR3	REG32(EMC_SACR3)
#define REG_EMC_SACR4	REG32(EMC_SACR4)

#define REG_EMC_NFCSR	REG32(EMC_NFCSR)

#define REG_EMC_DMCR	REG32(EMC_DMCR)
#define REG_EMC_RTCSR	REG16(EMC_RTCSR)
#define REG_EMC_RTCNT	REG16(EMC_RTCNT)
#define REG_EMC_RTCOR	REG16(EMC_RTCOR)
#define REG_EMC_DMAR0	REG32(EMC_DMAR0)
#define REG_EMC_DMAR1	REG32(EMC_DMAR1)

/* Bus Control Register */
#define EMC_BCR_BT_SEL_BIT      30
#define EMC_BCR_BT_SEL_MASK     (0x3 << EMC_BCR_BT_SEL_BIT)
#define EMC_BCR_PK_SEL          (1 << 24)
#define EMC_BCR_BSR_MASK          (1 << 2)  /* Nand and SDRAM Bus Share Select: 0, share; 1, unshare */
  #define EMC_BCR_BSR_SHARE       (0 << 2)
  #define EMC_BCR_BSR_UNSHARE     (1 << 2)
#define EMC_BCR_BRE             (1 << 1)
#define EMC_BCR_ENDIAN          (1 << 0)

/* Static Memory Control Register */
#define EMC_SMCR_STRV_BIT	24
#define EMC_SMCR_STRV_MASK	(0x0f << EMC_SMCR_STRV_BIT)
#define EMC_SMCR_TAW_BIT	20
#define EMC_SMCR_TAW_MASK	(0x0f << EMC_SMCR_TAW_BIT)
#define EMC_SMCR_TBP_BIT	16
#define EMC_SMCR_TBP_MASK	(0x0f << EMC_SMCR_TBP_BIT)
#define EMC_SMCR_TAH_BIT	12
#define EMC_SMCR_TAH_MASK	(0x07 << EMC_SMCR_TAH_BIT)
#define EMC_SMCR_TAS_BIT	8
#define EMC_SMCR_TAS_MASK	(0x07 << EMC_SMCR_TAS_BIT)
#define EMC_SMCR_BW_BIT		6
#define EMC_SMCR_BW_MASK	(0x03 << EMC_SMCR_BW_BIT)
  #define EMC_SMCR_BW_8BIT	(0 << EMC_SMCR_BW_BIT)
  #define EMC_SMCR_BW_16BIT	(1 << EMC_SMCR_BW_BIT)
  #define EMC_SMCR_BW_32BIT	(2 << EMC_SMCR_BW_BIT)
#define EMC_SMCR_BCM		(1 << 3)
#define EMC_SMCR_BL_BIT		1
#define EMC_SMCR_BL_MASK	(0x03 << EMC_SMCR_BL_BIT)
  #define EMC_SMCR_BL_4		(0 << EMC_SMCR_BL_BIT)
  #define EMC_SMCR_BL_8		(1 << EMC_SMCR_BL_BIT)
  #define EMC_SMCR_BL_16	(2 << EMC_SMCR_BL_BIT)
  #define EMC_SMCR_BL_32	(3 << EMC_SMCR_BL_BIT)
#define EMC_SMCR_SMT		(1 << 0)

/* Static Memory Bank Addr Config Reg */
#define EMC_SACR_BASE_BIT	8
#define EMC_SACR_BASE_MASK	(0xff << EMC_SACR_BASE_BIT)
#define EMC_SACR_MASK_BIT	0
#define EMC_SACR_MASK_MASK	(0xff << EMC_SACR_MASK_BIT)

/* NAND Flash Control/Status Register */
#define EMC_NFCSR_NFCE4		(1 << 7) /* NAND Flash Enable */
#define EMC_NFCSR_NFE4		(1 << 6) /* NAND Flash FCE# Assertion Enable */
#define EMC_NFCSR_NFCE3		(1 << 5)
#define EMC_NFCSR_NFE3		(1 << 4)
#define EMC_NFCSR_NFCE2		(1 << 3)
#define EMC_NFCSR_NFE2		(1 << 2)
#define EMC_NFCSR_NFCE1		(1 << 1)
#define EMC_NFCSR_NFE1		(1 << 0)

/* DRAM Control Register */
#define EMC_DMCR_BW_BIT		31
#define EMC_DMCR_BW		(1 << EMC_DMCR_BW_BIT)
#define EMC_DMCR_CA_BIT		26
#define EMC_DMCR_CA_MASK	(0x07 << EMC_DMCR_CA_BIT)
  #define EMC_DMCR_CA_8		(0 << EMC_DMCR_CA_BIT)
  #define EMC_DMCR_CA_9		(1 << EMC_DMCR_CA_BIT)
  #define EMC_DMCR_CA_10	(2 << EMC_DMCR_CA_BIT)
  #define EMC_DMCR_CA_11	(3 << EMC_DMCR_CA_BIT)
  #define EMC_DMCR_CA_12	(4 << EMC_DMCR_CA_BIT)
#define EMC_DMCR_RMODE		(1 << 25)
#define EMC_DMCR_RFSH		(1 << 24)
#define EMC_DMCR_MRSET		(1 << 23)
#define EMC_DMCR_RA_BIT		20
#define EMC_DMCR_RA_MASK	(0x03 << EMC_DMCR_RA_BIT)
  #define EMC_DMCR_RA_11	(0 << EMC_DMCR_RA_BIT)
  #define EMC_DMCR_RA_12	(1 << EMC_DMCR_RA_BIT)
  #define EMC_DMCR_RA_13	(2 << EMC_DMCR_RA_BIT)
#define EMC_DMCR_BA_BIT		19
#define EMC_DMCR_BA		(1 << EMC_DMCR_BA_BIT)
#define EMC_DMCR_PDM		(1 << 18)
#define EMC_DMCR_EPIN		(1 << 17)
#define EMC_DMCR_MBSEL		(1 << 16)
#define EMC_DMCR_TRAS_BIT	13
#define EMC_DMCR_TRAS_MASK	(0x07 << EMC_DMCR_TRAS_BIT)
#define EMC_DMCR_RCD_BIT	11
#define EMC_DMCR_RCD_MASK	(0x03 << EMC_DMCR_RCD_BIT)
#define EMC_DMCR_TPC_BIT	8
#define EMC_DMCR_TPC_MASK	(0x07 << EMC_DMCR_TPC_BIT)
#define EMC_DMCR_TRWL_BIT	5
#define EMC_DMCR_TRWL_MASK	(0x03 << EMC_DMCR_TRWL_BIT)
#define EMC_DMCR_TRC_BIT	2
#define EMC_DMCR_TRC_MASK	(0x07 << EMC_DMCR_TRC_BIT)
#define EMC_DMCR_TCL_BIT	0
#define EMC_DMCR_TCL_MASK	(0x03 << EMC_DMCR_TCL_BIT)

/* Refresh Time Control/Status Register */
#define EMC_RTCSR_SFR		(1 << 8)    /* self refresh flag */
#define EMC_RTCSR_CMF		(1 << 7)
#define EMC_RTCSR_CKS_BIT	0
#define EMC_RTCSR_CKS_MASK	(0x07 << EMC_RTCSR_CKS_BIT)
  #define EMC_RTCSR_CKS_DISABLE	(0 << EMC_RTCSR_CKS_BIT)
  #define EMC_RTCSR_CKS_4	(1 << EMC_RTCSR_CKS_BIT)
  #define EMC_RTCSR_CKS_16	(2 << EMC_RTCSR_CKS_BIT)
  #define EMC_RTCSR_CKS_64	(3 << EMC_RTCSR_CKS_BIT)
  #define EMC_RTCSR_CKS_256	(4 << EMC_RTCSR_CKS_BIT)
  #define EMC_RTCSR_CKS_1024	(5 << EMC_RTCSR_CKS_BIT)
  #define EMC_RTCSR_CKS_2048	(6 << EMC_RTCSR_CKS_BIT)
  #define EMC_RTCSR_CKS_4096	(7 << EMC_RTCSR_CKS_BIT)

/* SDRAM Bank Address Configuration Register */
#define EMC_DMAR_BASE_BIT	8
#define EMC_DMAR_BASE_MASK	(0xff << EMC_DMAR_BASE_BIT)
#define EMC_DMAR_MASK_BIT	0
#define EMC_DMAR_MASK_MASK	(0xff << EMC_DMAR_MASK_BIT)

/* Mode Register of SDRAM bank 0 */
#define EMC_SDMR_BM		(1 << 9) /* Write Burst Mode */
#define EMC_SDMR_OM_BIT		7        /* Operating Mode */
#define EMC_SDMR_OM_MASK	(3 << EMC_SDMR_OM_BIT)
  #define EMC_SDMR_OM_NORMAL	(0 << EMC_SDMR_OM_BIT)
#define EMC_SDMR_CAS_BIT	4        /* CAS Latency */
#define EMC_SDMR_CAS_MASK	(7 << EMC_SDMR_CAS_BIT)
  #define EMC_SDMR_CAS_1	(1 << EMC_SDMR_CAS_BIT)
  #define EMC_SDMR_CAS_2	(2 << EMC_SDMR_CAS_BIT)
  #define EMC_SDMR_CAS_3	(3 << EMC_SDMR_CAS_BIT)
#define EMC_SDMR_BT_BIT		3        /* Burst Type */
#define EMC_SDMR_BT_MASK	(1 << EMC_SDMR_BT_BIT)
  #define EMC_SDMR_BT_SEQ	(0 << EMC_SDMR_BT_BIT) /* Sequential */
  #define EMC_SDMR_BT_INT	(1 << EMC_SDMR_BT_BIT) /* Interleave */
#define EMC_SDMR_BL_BIT		0        /* Burst Length */
#define EMC_SDMR_BL_MASK	(7 << EMC_SDMR_BL_BIT)
  #define EMC_SDMR_BL_1		(0 << EMC_SDMR_BL_BIT)
  #define EMC_SDMR_BL_2		(1 << EMC_SDMR_BL_BIT)
  #define EMC_SDMR_BL_4		(2 << EMC_SDMR_BL_BIT)
  #define EMC_SDMR_BL_8		(3 << EMC_SDMR_BL_BIT)

#define EMC_SDMR_CAS2_16BIT \
  (EMC_SDMR_CAS_2 | EMC_SDMR_BT_SEQ | EMC_SDMR_BL_2)
#define EMC_SDMR_CAS2_32BIT \
  (EMC_SDMR_CAS_2 | EMC_SDMR_BT_SEQ | EMC_SDMR_BL_4)
#define EMC_SDMR_CAS3_16BIT \
  (EMC_SDMR_CAS_3 | EMC_SDMR_BT_SEQ | EMC_SDMR_BL_2)
#define EMC_SDMR_CAS3_32BIT \
  (EMC_SDMR_CAS_3 | EMC_SDMR_BT_SEQ | EMC_SDMR_BL_4)


/*************************************************************************
 * CIM
 *************************************************************************/
#define	CIM_CFG			(CIM_BASE + 0x0000)
#define	CIM_CTRL		(CIM_BASE + 0x0004)
#define	CIM_STATE		(CIM_BASE + 0x0008)
#define	CIM_IID			(CIM_BASE + 0x000C)
#define	CIM_RXFIFO		(CIM_BASE + 0x0010)
#define	CIM_DA			(CIM_BASE + 0x0020)
#define	CIM_FA			(CIM_BASE + 0x0024)
#define	CIM_FID			(CIM_BASE + 0x0028)
#define	CIM_CMD			(CIM_BASE + 0x002C)
#define	CIM_SIZE		(CIM_BASE + 0x0030)
#define	CIM_OFFSET		(CIM_BASE + 0x0034)
#define	CIM_RAM_ADDR		(CIM_BASE + 0x1000)

#define	REG_CIM_CFG		REG32(CIM_CFG)
#define	REG_CIM_CTRL		REG32(CIM_CTRL)
#define	REG_CIM_STATE		REG32(CIM_STATE)
#define	REG_CIM_IID		REG32(CIM_IID)
#define	REG_CIM_RXFIFO		REG32(CIM_RXFIFO)
#define	REG_CIM_DA		REG32(CIM_DA)
#define	REG_CIM_FA		REG32(CIM_FA)
#define	REG_CIM_FID		REG32(CIM_FID)
#define	REG_CIM_CMD		REG32(CIM_CMD)
#define	REG_CIM_SIZE		REG32(CIM_SIZE)
#define	REG_CIM_OFFSET		REG32(CIM_OFFSET)

#define	CIM_CFG_ORDER_BIT	18
#define	CIM_CFG_ORDER_MASK	(0x3 << CIM_CFG_ORDER_BIT)
  #define CIM_CFG_ORDER_0	  (0x0 << CIM_CFG_ORDER_BIT) 	/* Y0CbY1Cr; YCbCr */
  #define CIM_CFG_ORDER_1	  (0x1 << CIM_CFG_ORDER_BIT)	/* Y0CrY1Cb; YCrCb */
  #define CIM_CFG_ORDER_2	  (0x2 << CIM_CFG_ORDER_BIT)	/* CbY0CrY1; CbCrY */
  #define CIM_CFG_ORDER_3	  (0x3 << CIM_CFG_ORDER_BIT)	/* CrY0CbY1; CrCbY */
#define	CIM_CFG_DF_BIT		16
#define	CIM_CFG_DF_MASK		  (0x3 << CIM_CFG_DF_BIT)
  #define CIM_CFG_DF_YUV444	  (0x1 << CIM_CFG_DF_BIT) 	/* YCbCr444 */
  #define CIM_CFG_DF_YUV422	  (0x2 << CIM_CFG_DF_BIT)	/* YCbCr422 */
  #define CIM_CFG_DF_ITU656	  (0x3 << CIM_CFG_DF_BIT)	/* ITU656 YCbCr422 */
#define	CIM_CFG_INV_DAT		(1 << 15)
#define	CIM_CFG_VSP		(1 << 14) /* VSYNC Polarity:0-rising edge active,1-falling edge active */
#define	CIM_CFG_HSP		(1 << 13) /* HSYNC Polarity:0-rising edge active,1-falling edge active */
#define	CIM_CFG_PCP		(1 << 12) /* PCLK working edge: 0-rising, 1-falling */
#define	CIM_CFG_DMA_BURST_TYPE_BIT	10
#define	CIM_CFG_DMA_BURST_TYPE_MASK	(0x3 << CIM_CFG_DMA_BURST_TYPE_BIT)
  #define	CIM_CFG_DMA_BURST_INCR4		(0 << CIM_CFG_DMA_BURST_TYPE_BIT)
  #define	CIM_CFG_DMA_BURST_INCR8		(1 << CIM_CFG_DMA_BURST_TYPE_BIT)	/* Suggested */
  #define	CIM_CFG_DMA_BURST_INCR16	(2 << CIM_CFG_DMA_BURST_TYPE_BIT)	/* Suggested High speed AHB*/

#define	CIM_CFG_DUMMY_ZERO	(1 << 9)
#define	CIM_CFG_EXT_VSYNC	(1 << 8)	/* Only for ITU656 Progressive mode */
#define	CIM_CFG_PACK_BIT	4
#define	CIM_CFG_PACK_MASK	(0x7 << CIM_CFG_PACK_BIT)
  #define CIM_CFG_PACK_0	  (0 << CIM_CFG_PACK_BIT) /* 11 22 33 44 0xY0CbY1Cr */
  #define CIM_CFG_PACK_1	  (1 << CIM_CFG_PACK_BIT) /* 22 33 44 11 0xCbY1CrY0 */
  #define CIM_CFG_PACK_2	  (2 << CIM_CFG_PACK_BIT) /* 33 44 11 22 0xY1CrY0Cb */
  #define CIM_CFG_PACK_3	  (3 << CIM_CFG_PACK_BIT) /* 44 11 22 33 0xCrY0CbY1 */
  #define CIM_CFG_PACK_4	  (4 << CIM_CFG_PACK_BIT) /* 44 33 22 11 0xCrY1CbY0 */
  #define CIM_CFG_PACK_5	  (5 << CIM_CFG_PACK_BIT) /* 33 22 11 44 0xY1CbY0Cr */
  #define CIM_CFG_PACK_6	  (6 << CIM_CFG_PACK_BIT) /* 22 11 44 33 0xCbY0CrY1 */
  #define CIM_CFG_PACK_7	  (7 << CIM_CFG_PACK_BIT) /* 11 44 33 22 0xY0CrY1Cb */
#define	CIM_CFG_BYPASS_BIT	2
#define	CIM_CFG_BYPASS_MASK	(1 << CIM_CFG_BYPASS_BIT)
  #define CIM_CFG_BYPASS	  (1 << CIM_CFG_BYPASS_BIT)
#define	CIM_CFG_DSM_BIT		0
#define	CIM_CFG_DSM_MASK	(0x3 << CIM_CFG_DSM_BIT)
  #define CIM_CFG_DSM_CPM	  (0 << CIM_CFG_DSM_BIT) /* CCIR656 Progressive Mode */
  #define CIM_CFG_DSM_CIM	  (1 << CIM_CFG_DSM_BIT) /* CCIR656 Interlace Mode */
  #define CIM_CFG_DSM_GCM	  (2 << CIM_CFG_DSM_BIT) /* Gated Clock Mode */

/* CIM Control Register  (CIM_CTRL) */
#define	CIM_CTRL_EEOF_LINE_BIT	20
#define	CIM_CTRL_EEOF_LINE_MASK	(0xfff << CIM_CTRL_EEOF_LINE_BIT)
#define	CIM_CTRL_FRC_BIT	16
#define	CIM_CTRL_FRC_MASK	(0xf << CIM_CTRL_FRC_BIT)
  #define CIM_CTRL_FRC_1	  (0x0 << CIM_CTRL_FRC_BIT) /* Sample every frame */
  #define CIM_CTRL_FRC_2	  (0x1 << CIM_CTRL_FRC_BIT) /* Sample 1/2 frame */
  #define CIM_CTRL_FRC_3	  (0x2 << CIM_CTRL_FRC_BIT) /* Sample 1/3 frame */
  #define CIM_CTRL_FRC_4	  (0x3 << CIM_CTRL_FRC_BIT) /* Sample 1/4 frame */
  #define CIM_CTRL_FRC_5	  (0x4 << CIM_CTRL_FRC_BIT) /* Sample 1/5 frame */
  #define CIM_CTRL_FRC_6	  (0x5 << CIM_CTRL_FRC_BIT) /* Sample 1/6 frame */
  #define CIM_CTRL_FRC_7	  (0x6 << CIM_CTRL_FRC_BIT) /* Sample 1/7 frame */
  #define CIM_CTRL_FRC_8	  (0x7 << CIM_CTRL_FRC_BIT) /* Sample 1/8 frame */
  #define CIM_CTRL_FRC_9	  (0x8 << CIM_CTRL_FRC_BIT) /* Sample 1/9 frame */
  #define CIM_CTRL_FRC_10	  (0x9 << CIM_CTRL_FRC_BIT) /* Sample 1/10 frame */
  #define CIM_CTRL_FRC_11	  (0xA << CIM_CTRL_FRC_BIT) /* Sample 1/11 frame */
  #define CIM_CTRL_FRC_12	  (0xB << CIM_CTRL_FRC_BIT) /* Sample 1/12 frame */
  #define CIM_CTRL_FRC_13	  (0xC << CIM_CTRL_FRC_BIT) /* Sample 1/13 frame */
  #define CIM_CTRL_FRC_14	  (0xD << CIM_CTRL_FRC_BIT) /* Sample 1/14 frame */
  #define CIM_CTRL_FRC_15	  (0xE << CIM_CTRL_FRC_BIT) /* Sample 1/15 frame */
  #define CIM_CTRL_FRC_16	  (0xF << CIM_CTRL_FRC_BIT) /* Sample 1/16 frame */

#define	CIM_CTRL_DMA_EEOF	(1 << 15)	/* Enable EEOF interrupt */
#define	CIM_CTRL_WIN_EN		(1 << 14)
#define	CIM_CTRL_VDDM		(1 << 13) /* VDD interrupt enable */
#define	CIM_CTRL_DMA_SOFM	(1 << 12)
#define	CIM_CTRL_DMA_EOFM	(1 << 11)
#define	CIM_CTRL_DMA_STOPM	(1 << 10)
#define	CIM_CTRL_RXF_TRIGM	(1 << 9)
#define	CIM_CTRL_RXF_OFM	(1 << 8)
#define	CIM_CTRL_DMA_SYNC	(1 << 7)	/*when change DA, do frame sync */
#define	CIM_CTRL_RXF_TRIG_BIT	3
#define	CIM_CTRL_RXF_TRIG_MASK	(0xf << CIM_CTRL_RXF_TRIG_BIT) /* trigger value = (n+1)*burst_type */
#define CIM_CTRL_RXF_TRIG_8     (1 << CIM_CTRL_RXF_TRIG_BIT) /* RXFIFO Trigger Value is 8 */

#define	CIM_CTRL_DMA_EN		(1 << 2) /* Enable DMA */
#define	CIM_CTRL_RXF_RST	(1 << 1) /* RxFIFO reset */
#define	CIM_CTRL_ENA		(1 << 0) /* Enable CIM */

/* CIM State Register  (CIM_STATE) */
#define	CIM_STATE_DMA_EEOF	(1 << 7) /* DMA Line EEOf irq */
#define	CIM_STATE_DMA_SOF	(1 << 6) /* DMA start irq */
#define	CIM_STATE_DMA_EOF	(1 << 5) /* DMA end irq */
#define	CIM_STATE_DMA_STOP	(1 << 4) /* DMA stop irq */
#define	CIM_STATE_RXF_OF	(1 << 3) /* RXFIFO over flow irq */
#define	CIM_STATE_RXF_TRIG	(1 << 2) /* RXFIFO triger meet irq */
#define	CIM_STATE_RXF_EMPTY	(1 << 1) /* RXFIFO empty irq */
#define	CIM_STATE_VDD		(1 << 0) /* CIM disabled irq */

/* CIM DMA Command Register (CIM_CMD) */

#define	CIM_CMD_SOFINT		(1 << 31) /* enable DMA start irq */
#define	CIM_CMD_EOFINT		(1 << 30) /* enable DMA end irq */
#define	CIM_CMD_EEOFINT		(1 << 29) /* enable DMA EEOF irq */
#define	CIM_CMD_STOP		(1 << 28) /* enable DMA stop irq */
#define	CIM_CMD_OFRCV		(1 << 27) /* enable recovery when TXFiFo overflow */
#define	CIM_CMD_LEN_BIT		0
#define	CIM_CMD_LEN_MASK	(0xffffff << CIM_CMD_LEN_BIT)

/* CIM Window-Image Size Register  (CIM_SIZE) */
#define	CIM_SIZE_LPF_BIT	16 /* Lines per freame for csc output image */
#define	CIM_SIZE_LPF_MASK	(0x1fff << CIM_SIZE_LPF_BIT)
#define	CIM_SIZE_PPL_BIT	0 /* Pixels per line for csc output image, should be an even number */
#define	CIM_SIZE_PPL_MASK	(0x1fff << CIM_SIZE_PPL_BIT)

/* CIM Image Offset Register  (CIM_OFFSET) */
#define	CIM_OFFSET_V_BIT	16 /* Vertical offset */
#define	CIM_OFFSET_V_MASK	(0xfff << CIM_OFFSET_V_BIT)
#define	CIM_OFFSET_H_BIT	0 /* Horizontal offset, should be an enen number */
#define	CIM_OFFSET_H_MASK	(0xfff << CIM_OFFSET_H_BIT) /*OFFSET_H should be even number*/

#define __cim_set_eeof_line(n)                                          \
do {                                                            \
        REG_CIM_CTRL &= ~CIM_CTRL_EEOF_LINE_MASK;               \
        REG_CIM_CTRL |= ( ((n) << CIM_CTRL_EEOF_LINE_BIT) & CIM_CTRL_EEOF_LINE_MASK ); \
} while (0)

#define __gpio_as_cim_8bit()                                   \
do {       							\
	      REG_GPIO_PXFUNS(4) = 0x00000fff;  \
              REG_GPIO_PXTRGC(4) = 0x00000fff;	\
              REG_GPIO_PXSELC(4) = 0x00000fff; \
} while (0)
/*************************************************************************
 * SADC (Smart A/D Controller)
 *************************************************************************/

#define SADC_ENA	(SADC_BASE + 0x00)  /* ADC Enable Register */
#define SADC_CFG	(SADC_BASE + 0x04)  /* ADC Configure Register */
#define SADC_CTRL	(SADC_BASE + 0x08)  /* ADC Control Register */
#define SADC_STATE	(SADC_BASE + 0x0C)  /* ADC Status Register*/
#define SADC_SAMETIME	(SADC_BASE + 0x10)  /* ADC Same Point Time Register */
#define SADC_WAITTIME	(SADC_BASE + 0x14)  /* ADC Wait Time Register */
#define SADC_TSDAT	(SADC_BASE + 0x18)  /* ADC Touch Screen Data Register */
#define SADC_BATDAT	(SADC_BASE + 0x1C)  /* ADC PBAT Data Register */
#define SADC_SADDAT	(SADC_BASE + 0x20)  /* ADC SADCIN Data Register */
#define SADC_ADCLK	(SADC_BASE + 0x28)  /* ADC Clock Divide Register */

#define REG_SADC_ENA		REG8(SADC_ENA)
#define REG_SADC_CFG		REG32(SADC_CFG)
#define REG_SADC_CTRL		REG8(SADC_CTRL)
#define REG_SADC_STATE		REG8(SADC_STATE)
#define REG_SADC_SAMETIME	REG16(SADC_SAMETIME)
#define REG_SADC_WAITTIME	REG16(SADC_WAITTIME)
#define REG_SADC_TSDAT		REG32(SADC_TSDAT)
#define REG_SADC_BATDAT		REG16(SADC_BATDAT)
#define REG_SADC_SADDAT		REG16(SADC_SADDAT)
#define REG_SADC_ADCLK		REG32(SADC_ADCLK)

/* ADC Enable Register */
#define SADC_ENA_ADEN		(1 << 7)  /* Touch Screen Enable */
#define SADC_ENA_ENTR_SLP	(1 << 6)  /* Touch Screen Enable */
#define SADC_ENA_EXIT_SLP	(1 << 5)  /* Touch Screen Enable */
#define SADC_ENA_TSEN		(1 << 2)  /* Touch Screen Enable */
#define SADC_ENA_PBATEN		(1 << 1)  /* PBAT Enable */
#define SADC_ENA_SADCINEN	(1 << 0)  /* SADCIN Enable */

/* ADC Configure Register */
#define SADC_CFG_SPZZ    (1 << 31) //added
#define SADC_CFG_EXIN           (1 << 30)
#define SADC_CFG_CLKOUT_NUM_BIT	16
#define SADC_CFG_CLKOUT_NUM_MASK (0x7 << SADC_CFG_CLKOUT_NUM_BIT)
#define SADC_CFG_DNUM(x)  (((x) - 1) << SADC_CFG_CLKOUT_NUM_BIT)
#define SADC_CFG_TS_DMA		(1 << 15)  /* Touch Screen DMA Enable */
#define SADC_CFG_XYZ_BIT	13  /* XYZ selection */
#define SADC_CFG_XYZ_MASK	(0x3 << SADC_CFG_XYZ_BIT)
  #define SADC_CFG_XY		(0 << SADC_CFG_XYZ_BIT)
  #define SADC_CFG_XYZ		(1 << SADC_CFG_XYZ_BIT)
  #define SADC_CFG_XYZ1Z2	(2 << SADC_CFG_XYZ_BIT)
#define SADC_CFG_SNUM_BIT	10  /* Sample Number */
#define SADC_CFG_SNUM_MASK	(0x7 << SADC_CFG_SNUM_BIT)
  #define SADC_CFG_SNUM_1	(0x0 << SADC_CFG_SNUM_BIT)
  #define SADC_CFG_SNUM_2	(0x1 << SADC_CFG_SNUM_BIT)
  #define SADC_CFG_SNUM_3	(0x2 << SADC_CFG_SNUM_BIT)
  #define SADC_CFG_SNUM_4	(0x3 << SADC_CFG_SNUM_BIT)
  #define SADC_CFG_SNUM_5	(0x4 << SADC_CFG_SNUM_BIT)
  #define SADC_CFG_SNUM_6	(0x5 << SADC_CFG_SNUM_BIT)
  #define SADC_CFG_SNUM_8	(0x6 << SADC_CFG_SNUM_BIT)
  #define SADC_CFG_SNUM_9	(0x7 << SADC_CFG_SNUM_BIT)
#define SADC_CFG_SNUM(x) (((x) - 1) << SADC_CFG_SNUM_BIT)//added
#define SADC_CFG_CLKDIV_BIT	5  /* AD Converter frequency clock divider */
#define SADC_CFG_CLKDIV_MASK	(0x1f << SADC_CFG_CLKDIV_BIT)
#define SADC_CFG_PBAT_HIGH	(0 << 4)  /* PBAT >= 2.5V */
#define SADC_CFG_PBAT_LOW	(1 << 4)  /* PBAT < 2.5V */
#define SADC_CFG_CMD_BIT	0  /* ADC Command */
#define SADC_CFG_CMD_MASK	(0xf << SADC_CFG_CMD_BIT)
  #define SADC_CFG_CMD_X_SE	(0x0 << SADC_CFG_CMD_BIT) /* X Single-End */
  #define SADC_CFG_CMD_Y_SE	(0x1 << SADC_CFG_CMD_BIT) /* Y Single-End */
  #define SADC_CFG_CMD_X_DIFF	(0x2 << SADC_CFG_CMD_BIT) /* X Differential */
  #define SADC_CFG_CMD_Y_DIFF	(0x3 << SADC_CFG_CMD_BIT) /* Y Differential */
  #define SADC_CFG_CMD_Z1_DIFF	(0x4 << SADC_CFG_CMD_BIT) /* Z1 Differential */
  #define SADC_CFG_CMD_Z2_DIFF	(0x5 << SADC_CFG_CMD_BIT) /* Z2 Differential */
  #define SADC_CFG_CMD_Z3_DIFF	(0x6 << SADC_CFG_CMD_BIT) /* Z3 Differential */
  #define SADC_CFG_CMD_Z4_DIFF	(0x7 << SADC_CFG_CMD_BIT) /* Z4 Differential */
  #define SADC_CFG_CMD_TP_SE	(0x8 << SADC_CFG_CMD_BIT) /* Touch Pressure */
  #define SADC_CFG_CMD_PBATH_SE	(0x9 << SADC_CFG_CMD_BIT) /* PBAT >= 2.5V */
  #define SADC_CFG_CMD_PBATL_SE	(0xa << SADC_CFG_CMD_BIT) /* PBAT < 2.5V */
  #define SADC_CFG_CMD_SADCIN_SE (0xb << SADC_CFG_CMD_BIT) /* Measure SADCIN */
  #define SADC_CFG_CMD_INT_PEN	(0xc << SADC_CFG_CMD_BIT) /* INT_PEN Enable */

/* ADC Control Register */
#define SADC_CTRL_SLPENDM	(1 << 5)  /* sleep Interrupt Mask */
#define SADC_CTRL_PENDM		(1 << 4)  /* Pen Down Interrupt Mask */
#define SADC_CTRL_PENUM		(1 << 3)  /* Pen Up Interrupt Mask */
#define SADC_CTRL_TSRDYM	(1 << 2)  /* Touch Screen Data Ready Interrupt Mask */
#define SADC_CTRL_PBATRDYM	(1 << 1)  /* PBAT Data Ready Interrupt Mask */
#define SADC_CTRL_SRDYM		(1 << 0)  /* SADCIN Data Ready Interrupt Mask */

/* ADC Status Register */
#define SADC_STATE_SLEEPND	(1 << 5)  /* Pen Down Interrupt Flag */
#define SADC_STATE_PEND		(1 << 4)  /* Pen Down Interrupt Flag */
#define SADC_STATE_PENU		(1 << 3)  /* Pen Up Interrupt Flag */
#define SADC_STATE_TSRDY	(1 << 2)  /* Touch Screen Data Ready Interrupt Flag */
#define SADC_STATE_PBATRDY	(1 << 1)  /* PBAT Data Ready Interrupt Flag */
#define SADC_STATE_SRDY		(1 << 0)  /* SADCIN Data Ready Interrupt Flag */

/* ADC Touch Screen Data Register */
#define SADC_TSDAT_DATA0_BIT	0
#define SADC_TSDAT_DATA0_MASK	(0xfff << SADC_TSDAT_DATA0_BIT)
#define SADC_TSDAT_TYPE0	(1 << 15)
#define SADC_TSDAT_DATA1_BIT	16
#define SADC_TSDAT_DATA1_MASK	(0xfff << SADC_TSDAT_DATA1_BIT)
#define SADC_TSDAT_TYPE1	(1 << 31)

/* ADC Clock Divide Register */
#define SADC_ADCLK_CLKDIV_10_BIT	16
#define SADC_ADCLK_CLKDIV_10_MASK	(0x7f << SADC_ADCLK_CLKDIV_10_BIT)
#define SADC_ADCLK_CLKDIV_BIT		0
#define SADC_ADCLK_CLKDIV_MASK		(0x3f << SADC_ADCLK_CLKDIV_BIT)

/*************************************************************************
 * SLCD (Smart LCD Controller)
 *************************************************************************/

#define SLCD_CFG	(SLCD_BASE + 0xA0)  /* SLCD Configure Register */
#define SLCD_CTRL	(SLCD_BASE + 0xA4)  /* SLCD Control Register */
#define SLCD_STATE	(SLCD_BASE + 0xA8)  /* SLCD Status Register */
#define SLCD_DATA	(SLCD_BASE + 0xAC)  /* SLCD Data Register */

#define REG_SLCD_CFG	REG32(SLCD_CFG)
#define REG_SLCD_CTRL	REG8(SLCD_CTRL)
#define REG_SLCD_STATE	REG8(SLCD_STATE)
#define REG_SLCD_DATA	REG32(SLCD_DATA)

/* SLCD Configure Register */
#define SLCD_CFG_DWIDTH_BIT	10
#define SLCD_CFG_DWIDTH_MASK	(0x7 << SLCD_CFG_DWIDTH_BIT)
  #define SLCD_CFG_DWIDTH_18BIT	(0 << SLCD_CFG_DWIDTH_BIT)
  #define SLCD_CFG_DWIDTH_16BIT	(1 << SLCD_CFG_DWIDTH_BIT)
  #define SLCD_CFG_DWIDTH_8BIT_x3	(2 << SLCD_CFG_DWIDTH_BIT)
  #define SLCD_CFG_DWIDTH_8BIT_x2	(3 << SLCD_CFG_DWIDTH_BIT)
  #define SLCD_CFG_DWIDTH_8BIT_x1	(4 << SLCD_CFG_DWIDTH_BIT)
  #define SLCD_CFG_DWIDTH_24BIT	(5 << SLCD_CFG_DWIDTH_BIT)
  #define SLCD_CFG_DWIDTH_9BIT_x2	(7 << SLCD_CFG_DWIDTH_BIT)
#define SLCD_CFG_CWIDTH_BIT	(8)
#define SLCD_CFG_CWIDTH_MASK	(0x7 << SLCD_CFG_CWIDTH_BIT)
#define SLCD_CFG_CWIDTH_16BIT	(0 << SLCD_CFG_CWIDTH_BIT)
#define SLCD_CFG_CWIDTH_8BIT	(1 << SLCD_CFG_CWIDTH_BIT)
#define SLCD_CFG_CWIDTH_18BIT	(2 << SLCD_CFG_CWIDTH_BIT)
#define SLCD_CFG_CWIDTH_24BIT	(3 << SLCD_CFG_CWIDTH_BIT)
#define SLCD_CFG_CS_ACTIVE_LOW	(0 << 4)
#define SLCD_CFG_CS_ACTIVE_HIGH	(1 << 4)
#define SLCD_CFG_RS_CMD_LOW	(0 << 3)
#define SLCD_CFG_RS_CMD_HIGH	(1 << 3)
#define SLCD_CFG_CLK_ACTIVE_FALLING	(0 << 1)
#define SLCD_CFG_CLK_ACTIVE_RISING	(1 << 1)
#define SLCD_CFG_TYPE_PARALLEL	(0 << 0)
#define SLCD_CFG_TYPE_SERIAL	(1 << 0)

/* SLCD Control Register */
#define SLCD_CTRL_DMA_MODE	(1 << 2)
#define SLCD_CTRL_DMA_START	(1 << 1)
#define SLCD_CTRL_DMA_EN	(1 << 0)

/* SLCD Status Register */
#define SLCD_STATE_BUSY		(1 << 0)

/* SLCD Data Register */
#define SLCD_DATA_RS_DATA	(0 << 31)
#define SLCD_DATA_RS_COMMAND	(1 << 31)

/*************************************************************************
 * LCD (LCD Controller)
 *************************************************************************/
#define LCD_CFG		(LCD_BASE + 0x00) /* LCD Configure Register */
#define LCD_CTRL	(LCD_BASE + 0x30) /* LCD Control Register */
#define LCD_STATE	(LCD_BASE + 0x34) /* LCD Status Register */

#define LCD_OSDC	(LCD_BASE + 0x100) /* LCD OSD Configure Register */
#define LCD_OSDCTRL	(LCD_BASE + 0x104) /* LCD OSD Control Register */
#define LCD_OSDS	(LCD_BASE + 0x108) /* LCD OSD Status Register */
#define LCD_BGC		(LCD_BASE + 0x10C) /* LCD Background Color Register */
#define LCD_KEY0	(LCD_BASE + 0x110) /* LCD Foreground Color Key Register 0 */
#define LCD_KEY1	(LCD_BASE + 0x114) /* LCD Foreground Color Key Register 1 */
#define LCD_ALPHA	(LCD_BASE + 0x118) /* LCD ALPHA Register */
#define LCD_IPUR	(LCD_BASE + 0x11C) /* LCD IPU Restart Register */

#define LCD_VAT		(LCD_BASE + 0x0c) /* Virtual Area Setting Register */
#define LCD_DAH		(LCD_BASE + 0x10) /* Display Area Horizontal Start/End Point */
#define LCD_DAV		(LCD_BASE + 0x14) /* Display Area Vertical Start/End Point */

#define LCD_XYP0	(LCD_BASE + 0x120) /* Foreground 0 XY Position Register */
#define LCD_XYP1	(LCD_BASE + 0x124) /* Foreground 1 XY Position Register */
#define LCD_SIZE0	(LCD_BASE + 0x128) /* Foreground 0 Size Register */
#define LCD_SIZE1	(LCD_BASE + 0x12C) /* Foreground 1 Size Register */
#define LCD_RGBC	(LCD_BASE + 0x90) /* RGB Controll Register */

#define LCD_VSYNC	(LCD_BASE + 0x04) /* Vertical Synchronize Register */
#define LCD_HSYNC	(LCD_BASE + 0x08) /* Horizontal Synchronize Register */
#define LCD_PS		(LCD_BASE + 0x18) /* PS Signal Setting */
#define LCD_CLS		(LCD_BASE + 0x1c) /* CLS Signal Setting */
#define LCD_SPL		(LCD_BASE + 0x20) /* SPL Signal Setting */
#define LCD_REV		(LCD_BASE + 0x24) /* REV Signal Setting */
#define LCD_IID		(LCD_BASE + 0x38) /* Interrupt ID Register */
#define LCD_DA0		(LCD_BASE + 0x40) /* Descriptor Address Register 0 */
#define LCD_SA0		(LCD_BASE + 0x44) /* Source Address Register 0 */
#define LCD_FID0	(LCD_BASE + 0x48) /* Frame ID Register 0 */
#define LCD_CMD0	(LCD_BASE + 0x4c) /* DMA Command Register 0 */
#define LCD_DA1		(LCD_BASE + 0x50) /* Descriptor Address Register 1 */
#define LCD_SA1		(LCD_BASE + 0x54) /* Source Address Register 1 */
#define LCD_FID1	(LCD_BASE + 0x58) /* Frame ID Register 1 */
#define LCD_CMD1	(LCD_BASE + 0x5c) /* DMA Command Register 1 */

#define LCD_OFFS0	(LCD_BASE + 0x60) /* DMA Offsize Register 0 */
#define LCD_PW0		(LCD_BASE + 0x64) /* DMA Page Width Register 0 */
#define LCD_CNUM0	(LCD_BASE + 0x68) /* DMA Command Counter Register 0 */
#define LCD_DESSIZE0	(LCD_BASE + 0x6C) /* Foreground Size in Descriptor 0 Register*/
#define LCD_OFFS1	(LCD_BASE + 0x70) /* DMA Offsize Register 1 */
#define LCD_PW1		(LCD_BASE + 0x74) /* DMA Page Width Register 1 */
#define LCD_CNUM1	(LCD_BASE + 0x78) /* DMA Command Counter Register 1 */
#define LCD_DESSIZE1	(LCD_BASE + 0x7C) /* Foreground Size in Descriptor 1 Register*/

#define REG_LCD_CFG	REG32(LCD_CFG)
#define REG_LCD_CTRL	REG32(LCD_CTRL)
#define REG_LCD_STATE	REG32(LCD_STATE)

#define REG_LCD_OSDC	REG16(LCD_OSDC)
#define REG_LCD_OSDCTRL	REG16(LCD_OSDCTRL)
#define REG_LCD_OSDS	REG16(LCD_OSDS)
#define REG_LCD_BGC	REG32(LCD_BGC)
#define REG_LCD_KEY0	REG32(LCD_KEY0)
#define REG_LCD_KEY1	REG32(LCD_KEY1)
#define REG_LCD_ALPHA	REG8(LCD_ALPHA)
#define REG_LCD_IPUR	REG32(LCD_IPUR)

#define REG_LCD_VAT	REG32(LCD_VAT)
#define REG_LCD_DAH	REG32(LCD_DAH)
#define REG_LCD_DAV	REG32(LCD_DAV)

#define REG_LCD_XYP0	REG32(LCD_XYP0)
#define REG_LCD_XYP1	REG32(LCD_XYP1)
#define REG_LCD_SIZE0	REG32(LCD_SIZE0)
#define REG_LCD_SIZE1	REG32(LCD_SIZE1)
#define REG_LCD_RGBC	REG16(LCD_RGBC)

#define REG_LCD_VSYNC	REG32(LCD_VSYNC)
#define REG_LCD_HSYNC	REG32(LCD_HSYNC)
#define REG_LCD_PS	REG32(LCD_PS)
#define REG_LCD_CLS	REG32(LCD_CLS)
#define REG_LCD_SPL	REG32(LCD_SPL)
#define REG_LCD_REV	REG32(LCD_REV)
#define REG_LCD_IID	REG32(LCD_IID)
#define REG_LCD_DA0	REG32(LCD_DA0)
#define REG_LCD_SA0	REG32(LCD_SA0)
#define REG_LCD_FID0	REG32(LCD_FID0)
#define REG_LCD_CMD0	REG32(LCD_CMD0)
#define REG_LCD_DA1	REG32(LCD_DA1)
#define REG_LCD_SA1	REG32(LCD_SA1)
#define REG_LCD_FID1	REG32(LCD_FID1)
#define REG_LCD_CMD1	REG32(LCD_CMD1)

#define REG_LCD_OFFS0	REG32(LCD_OFFS0)
#define REG_LCD_PW0	REG32(LCD_PW0)
#define REG_LCD_CNUM0	REG32(LCD_CNUM0)
#define REG_LCD_DESSIZE0	REG32(LCD_DESSIZE0)
#define REG_LCD_OFFS1	REG32(LCD_OFFS1)
#define REG_LCD_PW1	REG32(LCD_PW1)
#define REG_LCD_CNUM1	REG32(LCD_CNUM1)
#define REG_LCD_DESSIZE1	REG32(LCD_DESSIZE1)

/* LCD Configure Register */
#define LCD_CFG_LCDPIN_BIT	31  /* LCD pins selection */
#define LCD_CFG_LCDPIN_MASK	(0x1 << LCD_CFG_LCDPIN_BIT)
  #define LCD_CFG_LCDPIN_LCD	(0x0 << LCD_CFG_LCDPIN_BIT)
  #define LCD_CFG_LCDPIN_SLCD	(0x1 << LCD_CFG_LCDPIN_BIT)
#define LCD_CFG_TVEPEH		(1 << 30) /* TVE PAL enable extra halfline signal */
#define LCD_CFG_FUHOLD		(1 << 29) /* hold pixel clock when outFIFO underrun */
#define LCD_CFG_NEWDES		(1 << 28) /* use new descripter. old: 4words, new:8words */
#define LCD_CFG_PALBP		(1 << 27) /* bypass data format and alpha blending */
#define LCD_CFG_TVEN		(1 << 26) /* indicate the terminal is lcd or tv */
#define LCD_CFG_RECOVER		(1 << 25) /* Auto recover when output fifo underrun */
#define LCD_CFG_DITHER		(1 << 24) /* Dither function */
#define LCD_CFG_PSM		(1 << 23) /* PS signal mode */
#define LCD_CFG_CLSM		(1 << 22) /* CLS signal mode */
#define LCD_CFG_SPLM		(1 << 21) /* SPL signal mode */
#define LCD_CFG_REVM		(1 << 20) /* REV signal mode */
#define LCD_CFG_HSYNM		(1 << 19) /* HSYNC signal mode */
#define LCD_CFG_PCLKM		(1 << 18) /* PCLK signal mode */
#define LCD_CFG_INVDAT		(1 << 17) /* Inverse output data */
#define LCD_CFG_SYNDIR_IN	(1 << 16) /* VSYNC&HSYNC direction */
#define LCD_CFG_PSP		(1 << 15) /* PS pin reset state */
#define LCD_CFG_CLSP		(1 << 14) /* CLS pin reset state */
#define LCD_CFG_SPLP		(1 << 13) /* SPL pin reset state */
#define LCD_CFG_REVP		(1 << 12) /* REV pin reset state */
#define LCD_CFG_HSP		(1 << 11) /* HSYNC polarity:0-active high,1-active low */
#define LCD_CFG_PCP		(1 << 10) /* PCLK polarity:0-rising,1-falling */
#define LCD_CFG_DEP		(1 << 9)  /* DE polarity:0-active high,1-active low */
#define LCD_CFG_VSP		(1 << 8)  /* VSYNC polarity:0-rising,1-falling */
#define LCD_CFG_MODE_TFT_18BIT 	(1 << 7)  /* 18bit TFT */
#define LCD_CFG_MODE_TFT_16BIT 	(0 << 7)  /* 16bit TFT */
#define LCD_CFG_MODE_TFT_24BIT 	(1 << 6)  /* 24bit TFT */
#define LCD_CFG_PDW_BIT		4  /* STN pins utilization */
#define LCD_CFG_PDW_MASK	(0x3 << LCD_DEV_PDW_BIT)
#define LCD_CFG_PDW_1		(0 << LCD_CFG_PDW_BIT) /* LCD_D[0] */
  #define LCD_CFG_PDW_2		(1 << LCD_CFG_PDW_BIT) /* LCD_D[0:1] */
  #define LCD_CFG_PDW_4		(2 << LCD_CFG_PDW_BIT) /* LCD_D[0:3]/LCD_D[8:11] */
  #define LCD_CFG_PDW_8		(3 << LCD_CFG_PDW_BIT) /* LCD_D[0:7]/LCD_D[8:15] */
#define LCD_CFG_MODE_BIT	0  /* Display Device Mode Select */
#define LCD_CFG_MODE_MASK	(0x0f << LCD_CFG_MODE_BIT)
  #define LCD_CFG_MODE_GENERIC_TFT	(0 << LCD_CFG_MODE_BIT) /* 16,18 bit TFT */
  #define LCD_CFG_MODE_SPECIAL_TFT_1	(1 << LCD_CFG_MODE_BIT)
  #define LCD_CFG_MODE_SPECIAL_TFT_2	(2 << LCD_CFG_MODE_BIT)
  #define LCD_CFG_MODE_SPECIAL_TFT_3	(3 << LCD_CFG_MODE_BIT)
  #define LCD_CFG_MODE_NONINTER_CCIR656	(4 << LCD_CFG_MODE_BIT)
  #define LCD_CFG_MODE_INTER_CCIR656	(6 << LCD_CFG_MODE_BIT)
  #define LCD_CFG_MODE_SINGLE_CSTN	(8 << LCD_CFG_MODE_BIT)
  #define LCD_CFG_MODE_SINGLE_MSTN	(9 << LCD_CFG_MODE_BIT)
  #define LCD_CFG_MODE_DUAL_CSTN	(10 << LCD_CFG_MODE_BIT)
  #define LCD_CFG_MODE_DUAL_MSTN	(11 << LCD_CFG_MODE_BIT)
  #define LCD_CFG_MODE_SERIAL_TFT	(12 << LCD_CFG_MODE_BIT)
  #define LCD_CFG_MODE_LCM  		(13 << LCD_CFG_MODE_BIT)
  #define LCD_CFG_MODE_SLCD  		LCD_CFG_MODE_LCM

/* LCD Control Register */
#define LCD_CTRL_BST_BIT	28  /* Burst Length Selection */
#define LCD_CTRL_BST_MASK	(0x03 << LCD_CTRL_BST_BIT)
  #define LCD_CTRL_BST_4	(0 << LCD_CTRL_BST_BIT) /* 4-word */
  #define LCD_CTRL_BST_8	(1 << LCD_CTRL_BST_BIT) /* 8-word */
  #define LCD_CTRL_BST_16	(2 << LCD_CTRL_BST_BIT) /* 16-word */
  #define LCD_CTRL_BST_32	(3 << LCD_CTRL_BST_BIT) /* 32-word */
#define LCD_CTRL_RGB565		(0 << 27) /* RGB565 mode(foreground 0 in OSD mode) */
#define LCD_CTRL_RGB555		(1 << 27) /* RGB555 mode(foreground 0 in OSD mode) */
#define LCD_CTRL_OFUP		(1 << 26) /* Output FIFO underrun protection enable */
#define LCD_CTRL_FRC_BIT	24  /* STN FRC Algorithm Selection */
#define LCD_CTRL_FRC_MASK	(0x03 << LCD_CTRL_FRC_BIT)
  #define LCD_CTRL_FRC_16	(0 << LCD_CTRL_FRC_BIT) /* 16 grayscale */
  #define LCD_CTRL_FRC_4	(1 << LCD_CTRL_FRC_BIT) /* 4 grayscale */
  #define LCD_CTRL_FRC_2	(2 << LCD_CTRL_FRC_BIT) /* 2 grayscale */
#define LCD_CTRL_PDD_BIT	16  /* Load Palette Delay Counter */
#define LCD_CTRL_PDD_MASK	(0xff << LCD_CTRL_PDD_BIT)
#define LCD_CTRL_VGA		(1 << 15) /* VGA interface enable */
#define LCD_CTRL_DACTE		(1 << 14) /* DAC loop back test */
#define LCD_CTRL_EOFM		(1 << 13) /* EOF interrupt mask */
#define LCD_CTRL_SOFM		(1 << 12) /* SOF interrupt mask */
#define LCD_CTRL_OFUM		(1 << 11) /* Output FIFO underrun interrupt mask */
#define LCD_CTRL_IFUM0		(1 << 10) /* Input FIFO 0 underrun interrupt mask */
#define LCD_CTRL_IFUM1		(1 << 9)  /* Input FIFO 1 underrun interrupt mask */
#define LCD_CTRL_LDDM		(1 << 8)  /* LCD disable done interrupt mask */
#define LCD_CTRL_QDM		(1 << 7)  /* LCD quick disable done interrupt mask */
#define LCD_CTRL_BEDN		(1 << 6)  /* Endian selection */
#define LCD_CTRL_PEDN		(1 << 5)  /* Endian in byte:0-msb first, 1-lsb first */
#define LCD_CTRL_DIS		(1 << 4)  /* Disable indicate bit */
#define LCD_CTRL_ENA		(1 << 3)  /* LCD enable bit */
#define LCD_CTRL_BPP_BIT	0  /* Bits Per Pixel */
#define LCD_CTRL_BPP_MASK	(0x07 << LCD_CTRL_BPP_BIT)
  #define LCD_CTRL_BPP_1	(0 << LCD_CTRL_BPP_BIT) /* 1 bpp */
  #define LCD_CTRL_BPP_2	(1 << LCD_CTRL_BPP_BIT) /* 2 bpp */
  #define LCD_CTRL_BPP_4	(2 << LCD_CTRL_BPP_BIT) /* 4 bpp */
  #define LCD_CTRL_BPP_8	(3 << LCD_CTRL_BPP_BIT) /* 8 bpp */
  #define LCD_CTRL_BPP_16	(4 << LCD_CTRL_BPP_BIT) /* 15/16 bpp */
  #define LCD_CTRL_BPP_18_24	(5 << LCD_CTRL_BPP_BIT) /* 18/24/32 bpp */
  #define LCD_CTRL_BPP_CMPS_24	(6 << LCD_CTRL_BPP_BIT) /* 24 compress bpp */

/* LCD Status Register */
#define LCD_STATE_QD		(1 << 7) /* Quick Disable Done */
#define LCD_STATE_EOF		(1 << 5) /* EOF Flag */
#define LCD_STATE_SOF		(1 << 4) /* SOF Flag */
#define LCD_STATE_OFU		(1 << 3) /* Output FIFO Underrun */
#define LCD_STATE_IFU0		(1 << 2) /* Input FIFO 0 Underrun */
#define LCD_STATE_IFU1		(1 << 1) /* Input FIFO 1 Underrun */
#define LCD_STATE_LDD		(1 << 0) /* LCD Disabled */

/* OSD Configure Register */
#define LCD_OSDC_SOFM1		(1 << 15) /* Start of frame interrupt mask for foreground 1 */
#define LCD_OSDC_EOFM1		(1 << 14) /* End of frame interrupt mask for foreground 1 */
#define LCD_OSDC_SOFM0		(1 << 11) /* Start of frame interrupt mask for foreground 0 */
#define LCD_OSDC_EOFM0		(1 << 10) /* End of frame interrupt mask for foreground 0 */
#define LCD_OSDC_F1EN		(1 << 4) /* enable foreground 1 */
#define LCD_OSDC_F0EN		(1 << 3) /* enable foreground 0 */
#define LCD_OSDC_ALPHAEN		(1 << 2) /* enable alpha blending */
#define LCD_OSDC_ALPHAMD		(1 << 1) /* alpha blending mode */
#define LCD_OSDC_OSDEN		(1 << 0) /* OSD mode enable */

/* OSD Controll Register */
#define LCD_OSDCTRL_IPU		(1 << 15) /* input data from IPU */
#define LCD_OSDCTRL_RGB565	(0 << 4) /* foreground 1, 16bpp, 0-RGB565, 1-RGB555 */
#define LCD_OSDCTRL_RGB555	(1 << 4) /* foreground 1, 16bpp, 0-RGB565, 1-RGB555 */
#define LCD_OSDCTRL_CHANGES	(1 << 3) /* Change size flag */
#define LCD_OSDCTRL_OSDBPP_BIT	0 	 /* Bits Per Pixel of OSD Channel 1 */
#define LCD_OSDCTRL_OSDBPP_MASK	(0x7<<LCD_OSDCTRL_OSDBPP_BIT) 	 /* Bits Per Pixel of OSD Channel 1's MASK */
  #define LCD_OSDCTRL_OSDBPP_16	(4 << LCD_OSDCTRL_OSDBPP_BIT) /* RGB 15,16 bit*/
  #define LCD_OSDCTRL_OSDBPP_15_16	(4 << LCD_OSDCTRL_OSDBPP_BIT) /* RGB 15,16 bit*/
  #define LCD_OSDCTRL_OSDBPP_18_24	(5 << LCD_OSDCTRL_OSDBPP_BIT) /* RGB 18,24 bit*/
  #define LCD_OSDCTRL_OSDBPP_CMPS_24	(6 << LCD_OSDCTRL_OSDBPP_BIT) /* RGB 18,24 bit*/

/* OSD State Register */
#define LCD_OSDS_SOF1		(1 << 15) /* Start of frame flag for foreground 1 */
#define LCD_OSDS_EOF1		(1 << 14) /* End of frame flag for foreground 1 */
#define LCD_OSDS_SOF0		(1 << 11) /* Start of frame flag for foreground 0 */
#define LCD_OSDS_EOF0		(1 << 10) /* End of frame flag for foreground 0 */
#define LCD_OSDS_READY		(1 << 0)  /* Read for accept the change */

/* Background Color Register */
#define LCD_BGC_RED_OFFSET	(1 << 16)  /* Red color offset */
#define LCD_BGC_RED_MASK	(0xFF<<LCD_BGC_RED_OFFSET)
#define LCD_BGC_GREEN_OFFSET	(1 << 8)   /* Green color offset */
#define LCD_BGC_GREEN_MASK	(0xFF<<LCD_BGC_GREEN_OFFSET)
#define LCD_BGC_BLUE_OFFSET	(1 << 0)   /* Blue color offset */
#define LCD_BGC_BLUE_MASK	(0xFF<<LCD_BGC_BLUE_OFFSET)

/* Foreground Color Key Register 0,1(foreground 0, foreground 1) */
#define LCD_KEY_KEYEN		(1 << 31)   /* enable color key */
#define LCD_KEY_KEYMD		(1 << 30)   /* color key mode */
#define LCD_KEY_RED_OFFSET	16  /* Red color offset */
#define LCD_KEY_RED_MASK	(0xFF<<LCD_KEY_RED_OFFSET)
#define LCD_KEY_GREEN_OFFSET	8   /* Green color offset */
#define LCD_KEY_GREEN_MASK	(0xFF<<LCD_KEY_GREEN_OFFSET)
#define LCD_KEY_BLUE_OFFSET	0   /* Blue color offset */
#define LCD_KEY_BLUE_MASK	(0xFF<<LCD_KEY_BLUE_OFFSET)
#define LCD_KEY_MASK		(LCD_KEY_RED_MASK|LCD_KEY_GREEN_MASK|LCD_KEY_BLUE_MASK)

/* IPU Restart Register */
#define LCD_IPUR_IPUREN		(1 << 31)   /* IPU restart function enable*/
#define LCD_IPUR_IPURMASK	(0xFFFFFF)   /* IPU restart value mask*/

/* RGB Control Register */
#define LCD_RGBC_RGBDM		(1 << 15)   /* enable RGB Dummy data */
#define LCD_RGBC_DMM		(1 << 14)   /* RGB Dummy mode */
#define LCD_RGBC_YCC		(1 << 8)    /* RGB to YCC */
#define LCD_RGBC_ODDRGB_BIT	4	/* odd line serial RGB data arrangement */
#define LCD_RGBC_ODDRGB_MASK	(0x7<<LCD_RGBC_ODDRGB_BIT)
  #define LCD_RGBC_ODD_RGB	0
  #define LCD_RGBC_ODD_RBG	1
  #define LCD_RGBC_ODD_GRB	2
  #define LCD_RGBC_ODD_GBR	3
  #define LCD_RGBC_ODD_BRG	4
  #define LCD_RGBC_ODD_BGR	5
#define LCD_RGBC_EVENRGB_BIT	0	/* even line serial RGB data arrangement */
#define LCD_RGBC_EVENRGB_MASK	(0x7<<LCD_RGBC_EVENRGB_BIT)
  #define LCD_RGBC_EVEN_RGB	0
  #define LCD_RGBC_EVEN_RBG	1
  #define LCD_RGBC_EVEN_GRB	2
  #define LCD_RGBC_EVEN_GBR	3
  #define LCD_RGBC_EVEN_BRG	4
  #define LCD_RGBC_EVEN_BGR	5

/* Vertical Synchronize Register */
#define LCD_VSYNC_VPS_BIT	16  /* VSYNC pulse start in line clock, fixed to 0 */
#define LCD_VSYNC_VPS_MASK	(0xffff << LCD_VSYNC_VPS_BIT)
#define LCD_VSYNC_VPE_BIT	0   /* VSYNC pulse end in line clock */
#define LCD_VSYNC_VPE_MASK	(0xffff << LCD_VSYNC_VPS_BIT)

/* Horizontal Synchronize Register */
#define LCD_HSYNC_HPS_BIT	16  /* HSYNC pulse start position in dot clock */
#define LCD_HSYNC_HPS_MASK	(0xffff << LCD_HSYNC_HPS_BIT)
#define LCD_HSYNC_HPE_BIT	0   /* HSYNC pulse end position in dot clock */
#define LCD_HSYNC_HPE_MASK	(0xffff << LCD_HSYNC_HPE_BIT)

/* Virtual Area Setting Register */
#define LCD_VAT_HT_BIT		16  /* Horizontal Total size in dot clock */
#define LCD_VAT_HT_MASK		(0xffff << LCD_VAT_HT_BIT)
#define LCD_VAT_VT_BIT		0   /* Vertical Total size in dot clock */
#define LCD_VAT_VT_MASK		(0xffff << LCD_VAT_VT_BIT)

/* Display Area Horizontal Start/End Point Register */
#define LCD_DAH_HDS_BIT		16  /* Horizontal display area start in dot clock */
#define LCD_DAH_HDS_MASK	(0xffff << LCD_DAH_HDS_BIT)
#define LCD_DAH_HDE_BIT		0   /* Horizontal display area end in dot clock */
#define LCD_DAH_HDE_MASK	(0xffff << LCD_DAH_HDE_BIT)

/* Display Area Vertical Start/End Point Register */
#define LCD_DAV_VDS_BIT		16  /* Vertical display area start in line clock */
#define LCD_DAV_VDS_MASK	(0xffff << LCD_DAV_VDS_BIT)
#define LCD_DAV_VDE_BIT		0   /* Vertical display area end in line clock */
#define LCD_DAV_VDE_MASK	(0xffff << LCD_DAV_VDE_BIT)

/* Foreground XY Position Register */
#define LCD_XYP_YPOS_BIT	16  /* Y position bit of foreground 0 or 1 */
#define LCD_XYP_YPOS_MASK	(0xffff << LCD_XYP_YPOS_BIT)
#define LCD_XYP_XPOS_BIT	0   /* X position bit of foreground 0 or 1 */
#define LCD_XYP_XPOS_MASK	(0xffff << LCD_XYP_XPOS_BIT)

/* PS Signal Setting */
#define LCD_PS_PSS_BIT		16  /* PS signal start position in dot clock */
#define LCD_PS_PSS_MASK		(0xffff << LCD_PS_PSS_BIT)
#define LCD_PS_PSE_BIT		0   /* PS signal end position in dot clock */
#define LCD_PS_PSE_MASK		(0xffff << LCD_PS_PSE_BIT)

/* CLS Signal Setting */
#define LCD_CLS_CLSS_BIT	16  /* CLS signal start position in dot clock */
#define LCD_CLS_CLSS_MASK	(0xffff << LCD_CLS_CLSS_BIT)
#define LCD_CLS_CLSE_BIT	0   /* CLS signal end position in dot clock */
#define LCD_CLS_CLSE_MASK	(0xffff << LCD_CLS_CLSE_BIT)

/* SPL Signal Setting */
#define LCD_SPL_SPLS_BIT	16  /* SPL signal start position in dot clock */
#define LCD_SPL_SPLS_MASK	(0xffff << LCD_SPL_SPLS_BIT)
#define LCD_SPL_SPLE_BIT	0   /* SPL signal end position in dot clock */
#define LCD_SPL_SPLE_MASK	(0xffff << LCD_SPL_SPLE_BIT)

/* REV Signal Setting */
#define LCD_REV_REVS_BIT	16  /* REV signal start position in dot clock */
#define LCD_REV_REVS_MASK	(0xffff << LCD_REV_REVS_BIT)

/* DMA Command Register */
#define LCD_CMD_SOFINT		(1 << 31)
#define LCD_CMD_EOFINT		(1 << 30)
#define LCD_CMD_CMD		(1 << 29) /* indicate command in slcd mode */
#define LCD_CMD_PAL		(1 << 28)
#define LCD_CMD_LEN_BIT		0
#define LCD_CMD_LEN_MASK	(0xffffff << LCD_CMD_LEN_BIT)

/* DMA Offsize Register 0,1 */

/* DMA Page Width Register 0,1 */

/* DMA Command Counter Register 0,1 */

/* Foreground 0,1 Size Register */
#define LCD_DESSIZE_HEIGHT_BIT	16  /* height of foreground 1 */
#define LCD_DESSIZE_HEIGHT_MASK	(0xffff << LCD_DESSIZE_HEIGHT_BIT)
#define LCD_DESSIZE_WIDTH_BIT	0  /* width of foreground 1 */
#define LCD_DESSIZE_WIDTH_MASK	(0xffff << LCD_DESSIZE_WIDTH_BIT)

/*************************************************************************
 * TVE (TV Encoder Controller)
 *************************************************************************/
#define TVE_CTRL	(TVE_BASE + 0x40) /* TV Encoder Control register */
#define TVE_FRCFG	(TVE_BASE + 0x44) /* Frame configure register */
#define TVE_SLCFG1	(TVE_BASE + 0x50) /* TV signal level configure register 1 */
#define TVE_SLCFG2	(TVE_BASE + 0x54) /* TV signal level configure register 2*/
#define TVE_SLCFG3	(TVE_BASE + 0x58) /* TV signal level configure register 3*/
#define TVE_LTCFG1	(TVE_BASE + 0x60) /* Line timing configure register 1 */
#define TVE_LTCFG2	(TVE_BASE + 0x64) /* Line timing configure register 2 */
#define TVE_CFREQ	(TVE_BASE + 0x70) /* Chrominance sub-carrier frequency configure register */
#define TVE_CPHASE	(TVE_BASE + 0x74) /* Chrominance sub-carrier phase configure register */
#define TVE_CBCRCFG	(TVE_BASE + 0x78) /* Chrominance filter configure register */
#define TVE_WSSCR	(TVE_BASE + 0x80) /* Wide screen signal control register */
#define TVE_WSSCFG1	(TVE_BASE + 0x84) /* Wide screen signal configure register 1 */
#define TVE_WSSCFG2	(TVE_BASE + 0x88) /* Wide screen signal configure register 2 */
#define TVE_WSSCFG3	(TVE_BASE + 0x8c) /* Wide screen signal configure register 3 */

#define REG_TVE_CTRL     REG32(TVE_CTRL)
#define REG_TVE_FRCFG    REG32(TVE_FRCFG)
#define REG_TVE_SLCFG1   REG32(TVE_SLCFG1)
#define REG_TVE_SLCFG2   REG32(TVE_SLCFG2)
#define REG_TVE_SLCFG3   REG32(TVE_SLCFG3)
#define REG_TVE_LTCFG1   REG32(TVE_LTCFG1)
#define REG_TVE_LTCFG2   REG32(TVE_LTCFG2)
#define REG_TVE_CFREQ    REG32(TVE_CFREQ)
#define REG_TVE_CPHASE   REG32(TVE_CPHASE)
#define REG_TVE_CBCRCFG	 REG32(TVE_CBCRCFG)
#define REG_TVE_WSSCR    REG32(TVE_WSSCR)
#define REG_TVE_WSSCFG1  REG32(TVE_WSSCFG1)
#define REG_TVE_WSSCFG2	 REG32(TVE_WSSCFG2)
#define REG_TVE_WSSCFG3  REG32(TVE_WSSCFG3)

/* TV Encoder Control register */
#define TVE_CTRL_EYCBCR         (1 << 25)    /* YCbCr_enable */
#define TVE_CTRL_ECVBS          (1 << 24)    /* cvbs_enable */
#define TVE_CTRL_DAPD3	        (1 << 23)    /* DAC 3 power down */
#define TVE_CTRL_DAPD2	        (1 << 22)    /* DAC 2 power down */
#define TVE_CTRL_DAPD1	        (1 << 21)    /* DAC 1 power down */
#define TVE_CTRL_DAPD           (1 << 20)    /* power down all DACs */
#define TVE_CTRL_YCDLY_BIT      16
#define TVE_CTRL_YCDLY_MASK     (0x7 << TVE_CTRL_YCDLY_BIT)
#define TVE_CTRL_CGAIN_BIT      14
#define TVE_CTRL_CGAIN_MASK     (0x3 << TVE_CTRL_CGAIN_BIT)
  #define TVE_CTRL_CGAIN_FULL		(0 << TVE_CTRL_CGAIN_BIT) /* gain = 1 */
  #define TVE_CTRL_CGAIN_QUTR		(1 << TVE_CTRL_CGAIN_BIT) /* gain = 1/4 */
  #define TVE_CTRL_CGAIN_HALF		(2 << TVE_CTRL_CGAIN_BIT) /* gain = 1/2 */
  #define TVE_CTRL_CGAIN_THREE_QURT	(3 << TVE_CTRL_CGAIN_BIT) /* gain = 3/4 */
#define TVE_CTRL_CBW_BIT        12
#define TVE_CTRL_CBW_MASK       (0x3 << TVE_CTRL_CBW_BIT)
  #define TVE_CTRL_CBW_NARROW	(0 << TVE_CTRL_CBW_BIT) /* Narrow band */
  #define TVE_CTRL_CBW_WIDE	(1 << TVE_CTRL_CBW_BIT) /* Wide band */
  #define TVE_CTRL_CBW_EXTRA	(2 << TVE_CTRL_CBW_BIT) /* Extra wide band */
  #define TVE_CTRL_CBW_ULTRA	(3 << TVE_CTRL_CBW_BIT) /* Ultra wide band */
#define TVE_CTRL_SYNCT          (1 << 9)
#define TVE_CTRL_PAL            (1 << 8)
#define TVE_CTRL_FINV           (1 << 7) /* invert_top:1-invert top and bottom fields. */
#define TVE_CTRL_ZBLACK         (1 << 6) /* bypass_yclamp:1-Black of luminance (Y) input is 0.*/
#define TVE_CTRL_CR1ST          (1 << 5) /* uv_order:0-Cb before Cr,1-Cr before Cb */
#define TVE_CTRL_CLBAR          (1 << 4) /* Color bar mode:0-Output input video to TV,1-Output color bar to TV */
#define TVE_CTRL_SWRST          (1 << 0) /* Software reset:1-TVE is reset */

/* Signal level configure register 1 */
#define TVE_SLCFG1_BLACKL_BIT   0
#define TVE_SLCFG1_BLACKL_MASK  (0x3ff << TVE_SLCFG1_BLACKL_BIT)
#define TVE_SLCFG1_WHITEL_BIT   16
#define TVE_SLCFG1_WHITEL_MASK  (0x3ff << TVE_SLCFG1_WHITEL_BIT)

/* Signal level configure register 2 */
#define TVE_SLCFG2_BLANKL_BIT    0
#define TVE_SLCFG2_BLANKL_MASK   (0x3ff << TVE_SLCFG2_BLANKL_BIT)
#define TVE_SLCFG2_VBLANKL_BIT   16
#define TVE_SLCFG2_VBLANKL_MASK  (0x3ff << TVE_SLCFG2_VBLANKL_BIT)

/* Signal level configure register 3 */
#define TVE_SLCFG3_SYNCL_BIT   0
#define TVE_SLCFG3_SYNCL_MASK  (0xff << TVE_SLCFG3_SYNCL_BIT)

/* Line timing configure register 1 */
#define TVE_LTCFG1_BACKP_BIT   0
#define TVE_LTCFG1_BACKP_MASK  (0x7f << TVE_LTCFG1_BACKP_BIT)
#define TVE_LTCFG1_HSYNCW_BIT   8
#define TVE_LTCFG1_HSYNCW_MASK  (0x7f << TVE_LTCFG1_HSYNCW_BIT)
#define TVE_LTCFG1_FRONTP_BIT   16
#define TVE_LTCFG1_FRONTP_MASK  (0x1f << TVE_LTCFG1_FRONTP_BIT)

/* Line timing configure register 2 */
#define TVE_LTCFG2_BURSTW_BIT    0
#define TVE_LTCFG2_BURSTW_MASK   (0x3f << TVE_LTCFG2_BURSTW_BIT)
#define TVE_LTCFG2_PREBW_BIT     8
#define TVE_LTCFG2_PREBW_MASK    (0x1f << TVE_LTCFG2_PREBW_BIT)
#define TVE_LTCFG2_ACTLIN_BIT    16
#define TVE_LTCFG2_ACTLIN_MASK	(0x7ff << TVE_LTCFG2_ACTLIN_BIT)

/* Chrominance sub-carrier phase configure register */
#define TVE_CPHASE_CCRSTP_BIT    0
#define TVE_CPHASE_CCRSTP_MASK   (0x3 << TVE_CPHASE_CCRSTP_BIT)
  #define TVE_CPHASE_CCRSTP_8	(0 << TVE_CPHASE_CCRSTP_BIT) /* Every 8 field */
  #define TVE_CPHASE_CCRSTP_4	(1 << TVE_CPHASE_CCRSTP_BIT) /* Every 4 field */
  #define TVE_CPHASE_CCRSTP_2	(2 << TVE_CPHASE_CCRSTP_BIT) /* Every 2 lines */
  #define TVE_CPHASE_CCRSTP_0	(3 << TVE_CPHASE_CCRSTP_BIT) /* Never */
#define TVE_CPHASE_ACTPH_BIT     16
#define TVE_CPHASE_ACTPH_MASK    (0xff << TVE_CPHASE_ACTPH_BIT)
#define TVE_CPHASE_INITPH_BIT    24
#define TVE_CPHASE_INITPH_MASK   (0xff << TVE_CPHASE_INITPH_BIT)

/* Chrominance filter configure register */
#define TVE_CBCRCFG_CRGAIN_BIT       0
#define TVE_CBCRCFG_CRGAIN_MASK      (0xff << TVE_CBCRCFG_CRGAIN_BIT)
#define TVE_CBCRCFG_CBGAIN_BIT       8
#define TVE_CBCRCFG_CBGAIN_MASK      (0xff << TVE_CBCRCFG_CBGAIN_BIT)
#define TVE_CBCRCFG_CRBA_BIT         16
#define TVE_CBCRCFG_CRBA_MASK        (0xff << TVE_CBCRCFG_CRBA_BIT)
#define TVE_CBCRCFG_CBBA_BIT         24
#define TVE_CBCRCFG_CBBA_MASK        (0xff << TVE_CBCRCFG_CBBA_BIT)

/* Frame configure register */
#define TVE_FRCFG_NLINE_BIT          0
#define TVE_FRCFG_NLINE_MASK         (0x3ff << TVE_FRCFG_NLINE_BIT)
#define TVE_FRCFG_L1ST_BIT           16
#define TVE_FRCFG_L1ST_MASK          (0xff << TVE_FRCFG_L1ST_BIT)

/* Wide screen signal control register */
#define TVE_WSSCR_EWSS0_BIT	0
#define TVE_WSSCR_EWSS1_BIT	1
#define TVE_WSSCR_WSSTP_BIT	2
#define TVE_WSSCR_WSSCKBP_BIT	3
#define TVE_WSSCR_WSSEDGE_BIT	4
#define TVE_WSSCR_WSSEDGE_MASK	(0x7 << TVE_WSSCR_WSSEDGE_BIT)
#define TVE_WSSCR_ENCH_BIT	8
#define TVE_WSSCR_NCHW_BIT	9
#define TVE_WSSCR_NCHFREQ_BIT	12
#define TVE_WSSCR_NCHFREQ_MASK	(0x7 << TVE_WSSCR_NCHFREQ_BIT)

/*************************************************************************
 * USB Device
 *************************************************************************/
#define USB_BASE  UDC_BASE

#define USB_REG_FADDR		(USB_BASE + 0x00) /* Function Address 8-bit */
#define USB_REG_POWER		(USB_BASE + 0x01) /* Power Managemetn 8-bit */
#define USB_REG_INTRIN		(USB_BASE + 0x02) /* Interrupt IN 16-bit */
#define USB_REG_INTROUT		(USB_BASE + 0x04) /* Interrupt OUT 16-bit */
#define USB_REG_INTRINE		(USB_BASE + 0x06) /* Intr IN enable 16-bit */
#define USB_REG_INTROUTE	(USB_BASE + 0x08) /* Intr OUT enable 16-bit */
#define USB_REG_INTRUSB		(USB_BASE + 0x0a) /* Interrupt USB 8-bit */
#define USB_REG_INTRUSBE	(USB_BASE + 0x0b) /* Interrupt USB Enable 8-bit */
#define USB_REG_FRAME		(USB_BASE + 0x0c) /* Frame number 16-bit */
#define USB_REG_INDEX		(USB_BASE + 0x0e) /* Index register 8-bit */
#define USB_REG_TESTMODE	(USB_BASE + 0x0f) /* USB test mode 8-bit */

#define USB_REG_CSR0		(USB_BASE + 0x12) /* EP0 CSR 8-bit */
#define USB_REG_INMAXP		(USB_BASE + 0x10) /* EP1-2 IN Max Pkt Size 16-bit */
#define USB_REG_INCSR		(USB_BASE + 0x12) /* EP1-2 IN CSR LSB 8/16bit */
#define USB_REG_INCSRH		(USB_BASE + 0x13) /* EP1-2 IN CSR MSB 8-bit */
#define USB_REG_OUTMAXP		(USB_BASE + 0x14) /* EP1 OUT Max Pkt Size 16-bit */
#define USB_REG_OUTCSR		(USB_BASE + 0x16) /* EP1 OUT CSR LSB 8/16bit */
#define USB_REG_OUTCSRH		(USB_BASE + 0x17) /* EP1 OUT CSR MSB 8-bit */
#define USB_REG_OUTCOUNT	(USB_BASE + 0x18) /* bytes in EP0/1 OUT FIFO 16-bit */

#define USB_FIFO_EP0		(USB_BASE + 0x20)
#define USB_FIFO_EP1		(USB_BASE + 0x24)
#define USB_FIFO_EP2		(USB_BASE + 0x28)

#define USB_REG_EPINFO		(USB_BASE + 0x78) /* Endpoint information */
#define USB_REG_RAMINFO		(USB_BASE + 0x79) /* RAM information */

#define USB_REG_INTR		(USB_BASE + 0x200) /* DMA pending interrupts */
#define USB_REG_CNTL1		(USB_BASE + 0x204) /* DMA channel 1 control */
#define USB_REG_ADDR1		(USB_BASE + 0x208) /* DMA channel 1 AHB memory addr */
#define USB_REG_COUNT1		(USB_BASE + 0x20c) /* DMA channel 1 byte count */
#define USB_REG_CNTL2		(USB_BASE + 0x214) /* DMA channel 2 control */
#define USB_REG_ADDR2		(USB_BASE + 0x218) /* DMA channel 2 AHB memory addr */
#define USB_REG_COUNT2		(USB_BASE + 0x21c) /* DMA channel 2 byte count */


/* Power register bit masks */
#define USB_POWER_SUSPENDM	0x01
#define USB_POWER_RESUME	0x04
#define USB_POWER_HSMODE	0x10
#define USB_POWER_HSENAB	0x20
#define USB_POWER_SOFTCONN	0x40

/* Interrupt register bit masks */
#define USB_INTR_SUSPEND	0x01
#define USB_INTR_RESUME		0x02
#define USB_INTR_RESET		0x04

#define USB_INTR_EP0		0x0001
#define USB_INTR_INEP1		0x0002
#define USB_INTR_INEP2		0x0004
#define USB_INTR_OUTEP1		0x0002

/* CSR0 bit masks */
#define USB_CSR0_OUTPKTRDY	0x01
#define USB_CSR0_INPKTRDY	0x02
#define USB_CSR0_SENTSTALL	0x04
#define USB_CSR0_DATAEND	0x08
#define USB_CSR0_SETUPEND	0x10
#define USB_CSR0_SENDSTALL	0x20
#define USB_CSR0_SVDOUTPKTRDY	0x40
#define USB_CSR0_SVDSETUPEND	0x80

/* Endpoint CSR register bits */
#define USB_INCSRH_AUTOSET	0x80
#define USB_INCSRH_ISO		0x40
#define USB_INCSRH_MODE		0x20
#define USB_INCSRH_DMAREQENAB	0x10
#define USB_INCSRH_DMAREQMODE	0x04
#define USB_INCSR_CDT		0x40
#define USB_INCSR_SENTSTALL	0x20
#define USB_INCSR_SENDSTALL	0x10
#define USB_INCSR_FF		0x08
#define USB_INCSR_UNDERRUN	0x04
#define USB_INCSR_FFNOTEMPT	0x02
#define USB_INCSR_INPKTRDY	0x01
#define USB_OUTCSRH_AUTOCLR	0x80
#define USB_OUTCSRH_ISO		0x40
#define USB_OUTCSRH_DMAREQENAB	0x20
#define USB_OUTCSRH_DNYT	0x10
#define USB_OUTCSRH_DMAREQMODE	0x08
#define USB_OUTCSR_CDT		0x80
#define USB_OUTCSR_SENTSTALL	0x40
#define USB_OUTCSR_SENDSTALL	0x20
#define USB_OUTCSR_FF		0x10
#define USB_OUTCSR_DATAERR	0x08
#define USB_OUTCSR_OVERRUN	0x04
#define USB_OUTCSR_FFFULL	0x02
#define USB_OUTCSR_OUTPKTRDY	0x01

/* Testmode register bits */
#define USB_TEST_SE0NAK		0x01
#define USB_TEST_J		0x02
#define USB_TEST_K		0x04
#define USB_TEST_PACKET		0x08

/* DMA control bits */
#define USB_CNTL_ENA		0x01
#define USB_CNTL_DIR_IN		0x02
#define USB_CNTL_MODE_1		0x04
#define USB_CNTL_INTR_EN	0x08
#define USB_CNTL_EP(n)		((n) << 4)
#define USB_CNTL_BURST_0	(0 << 9)
#define USB_CNTL_BURST_4	(1 << 9)
#define USB_CNTL_BURST_8	(2 << 9)
#define USB_CNTL_BURST_16	(3 << 9)

/*************************************************************************
 * BCH
 *************************************************************************/
#define	BCH_CR         	(BCH_BASE + 0x00) /* BCH Control register */
#define	BCH_CRS       	(BCH_BASE + 0x04) /* BCH Control Set register */
#define	BCH_CRC       	(BCH_BASE + 0x08) /* BCH Control Clear register */
#define	BCH_CNT    	(BCH_BASE + 0x0C) /* BCH ENC/DEC Count register */
#define	BCH_DR     	(BCH_BASE + 0x10) /* BCH data register */
#define	BCH_PAR0    	(BCH_BASE + 0x14) /* BCH Parity 0 register */
#define	BCH_PAR1    	(BCH_BASE + 0x18) /* BCH Parity 1 register */
#define	BCH_PAR2    	(BCH_BASE + 0x1C) /* BCH Parity 2 register */
#define	BCH_PAR3    	(BCH_BASE + 0x20) /* BCH Parity 3 register */
#define	BCH_INTS    	(BCH_BASE + 0x24) /* BCH Interrupt Status register */
#define	BCH_ERR0        (BCH_BASE + 0x28) /* BCH Error Report 0 register */
#define	BCH_ERR1        (BCH_BASE + 0x2C) /* BCH Error Report 1 register */
#define	BCH_ERR2        (BCH_BASE + 0x30) /* BCH Error Report 2 register */
#define	BCH_ERR3        (BCH_BASE + 0x34) /* BCH Error Report 3 register */
#define	BCH_INTE        (BCH_BASE + 0x38) /* BCH Interrupt Enable register */
#define	BCH_INTES       (BCH_BASE + 0x3C) /* BCH Interrupt Set register */
#define	BCH_INTEC       (BCH_BASE + 0x40) /* BCH Interrupt Clear register */

#define	REG_BCH_CR      REG32(BCH_CR)
#define	REG_BCH_CRS     REG32(BCH_CRS)
#define	REG_BCH_CRC     REG32(BCH_CRC)
#define	REG_BCH_CNT     REG32(BCH_CNT)
#define	REG_BCH_DR      REG8(BCH_DR)
#define	REG_BCH_PAR0    REG32(BCH_PAR0)
#define	REG_BCH_PAR1    REG32(BCH_PAR1)
#define	REG_BCH_PAR2    REG32(BCH_PAR2)
#define	REG_BCH_PAR3    REG32(BCH_PAR3)
#define	REG_BCH_INTS    REG32(BCH_INTS)
#define	REG_BCH_ERR0    REG32(BCH_ERR0)
#define	REG_BCH_ERR1    REG32(BCH_ERR1)
#define	REG_BCH_ERR2    REG32(BCH_ERR2)
#define	REG_BCH_ERR3    REG32(BCH_ERR3)
#define	REG_BCH_INTE    REG32(BCH_INTE)
#define	REG_BCH_INTEC   REG32(BCH_INTEC)
#define	REG_BCH_INTES   REG32(BCH_INTES)

/* BCH Control Register*/
#define	BCH_CR_DMAE              (1 << 4)  /* BCH DMA Enable */
#define	BCH_CR_ENCE              (1 << 3)  /* BCH Encoding Select */
#define	BCH_CR_DECE              (0 << 3)  /* BCH Decoding Select */
#define	BCH_CR_BSEL8             (1 << 2)  /* 8 Bit BCH Select */
#define	BCH_CR_BSEL4             (0 << 2)  /* 4 Bit BCH Select */
#define	BCH_CR_BRST              (1 << 1)  /* BCH Reset */
#define	BCH_CR_BCHE              (1 << 0)  /* BCH Enable */

/* BCH Interrupt Status Register */
#define	BCH_INTS_ERRC_BIT        28
#define	BCH_INTS_ERRC_MASK       (0xf << BCH_INTS_ERRC_BIT)
#define	BCH_INTS_ALL0            (1 << 5)
#define	BCH_INTS_ALLf            (1 << 4)
#define	BCH_INTS_DECF            (1 << 3)
#define	BCH_INTS_ENCF            (1 << 2)
#define	BCH_INTS_UNCOR           (1 << 1)
#define	BCH_INTS_ERR             (1 << 0)

/* BCH ENC/DEC Count Register */
#define BCH_CNT_DEC_BIT          16
#define BCH_CNT_DEC_MASK         (0x3ff << BCH_CNT_DEC_BIT)
#define BCH_CNT_ENC_BIT          0
#define BCH_CNT_ENC_MASK         (0x3ff << BCH_CNT_ENC_BIT)

/* BCH Error Report Register */
#define BCH_ERR_INDEX_ODD_BIT    16
#define BCH_ERR_INDEX_ODD_MASK   (0x1fff << BCH_ERR_INDEX_ODD_BIT)
#define BCH_ERR_INDEX_EVEN_BIT   0
#define BCH_ERR_INDEX_EVEN_MASK  (0x1fff << BCH_ERR_INDEX_EVEN_BIT)

/*************************************************************************
 * OWI (One-wire Bus Controller )
 *************************************************************************/
#define OWI_CFG (OWI_BASE + 0x00) /* OWI Configure Register */
#define OWI_CTL (OWI_BASE + 0x04) /* OWI Control Register */
#define OWI_STS (OWI_BASE + 0x08) /* OWI Status Register */
#define OWI_DAT (OWI_BASE + 0x0c) /* OWI Data Register */
#define OWI_DIV (OWI_BASE + 0x10) /* OWI Clock Divide Register */

#define REG_OWI_CFG  REG8(OWI_CFG)
#define REG_OWI_CTL  REG8(OWI_CTL)
#define REG_OWI_STS  REG8(OWI_STS)
#define REG_OWI_DAT  REG8(OWI_DAT)
#define REG_OWI_DIV  REG8(OWI_DIV)

/* OWI Configure Register */
#define OWI_CFG_MODE      (1 << 7) /*  0: Regular speed mode  1: Overdrive  speed mode */
#define OWI_CFG_RDDATA    (1 << 6) /* 1: receive data from one-wire bus and stored in OWDAT*/
#define OWI_CFG_WRDATA    (1 << 5) /* 1: transmit the data in OWDAT */
#define OWI_CFG_RDST      (1 << 4) /* 1: was sampled during a read */
#define OWI_CFG_WR1RD     (1 << 3) /* 1: generate write 1 sequence on line */
#define OWI_CFG_WR0       (1 << 2) /* 1: generate write 0 sequence on line */
#define OWI_CFG_RST       (1 << 1) /* 1: generate reset pulse and sample slaves presence pulse*/
#define OWI_CFG_ENA       (1 << 0) /* 1: enable the OWI operation */

/* OWI Control Register */
#define OWI_CTL_EBYTE     (1 << 2) /* enable byte write/read interrupt */
#define OWI_CTL_EBIT      (1 << 1) /* enable bit write/read interrupt */
#define OWI_CTL_ERST      (1 << 0) /* enable reset sequence finished interrupt */

/* OWI Status Register */
#define OWI_STS_PST       (1 << 7) /* 1: one-wire bus has device on it */
#define OWI_STS_BYTE_RDY  (1 << 2) /* 1: have received or transmitted a data */
#define OWI_STS_BIT_RDY   (1 << 1) /* 1: have received or transmitted a bit */
#define OWI_STS_PST_RDY   (1 << 0) /* 1: have finished a reset pulse */

/* OWI Clock Divide Register */
#define OWI_DIV_CLKDIV_BIT  5


/*************************************************************************
 * MC (Motion Compensation)
 *************************************************************************/
#define MC_CTRL		(MC_BASE + 0x00) /* MC Control Register */
#define MC_STAT		(MC_BASE + 0x04) /* MC Status Register */
#define MC_REF_ADDR	(MC_BASE + 0x08) /* MC Reference Block Address Register */
#define MC_REF2_ADDR	(MC_BASE + 0x0C) /* MC 2nd Reference Block Address Register */
#define MC_CURR_ADDR	(MC_BASE + 0x10) /* MC Current Block Address Register */
#define MC_REF_STRD	(MC_BASE + 0x14) /* MC Reference Frame Stride Register */
#define MC_CURR_STRD	(MC_BASE + 0x18) /* MC Current Frame Stride Register */
#define MC_ITP_INFO	(MC_BASE + 0x1C) /* MC Block Interpolation Information Register */
#define MC_TAP_COEF1	(MC_BASE + 0x20) /* MC TAP Filter Coefficient 1 Register */
#define MC_TAP_COEF2	(MC_BASE + 0x24) /* MC TAP Filter Coefficient 2 Register */

#define REG_MC_CTRL		REG32(MC_CTRL)
#define REG_MC_STAT		REG32(MC_STAT)
#define REG_MC_REF_ADDR		REG32(MC_REF_ADDR)
#define REG_MC_REF2_ADDR	REG32(MC_REF2_ADDR)
#define REG_MC_CURR_ADDR	REG32(MC_CURR_ADDR)
#define REG_MC_REF_STRD		REG32(MC_REF_STRD)
#define REG_MC_CURR_STRD	REG32(MC_CURR_STRD)
#define REG_MC_ITP_INFO		REG32(MC_ITP_INFO)
#define REG_MC_TAP_COEF1	REG32(MC_TAP_COEF1)
#define REG_MC_TAP_COEF2	REG32(MC_TAP_COEF2)

/* MC Control Register */
#define MC_CTRL_CACHECLR	(1 << 2) /* MC Cache clear */
#define MC_CTRL_RESET		(1 << 1) /* MC Reset */
#define MC_CTRL_ENABLE		(1 << 0) /* MC enable */

/* MC Status Register */
#define MC_STAT_OUT_END		(1 << 0) /* Output DMA termination flag */

/* MC Reference Frame Stride Register, unit: byte */
#define MC_REF_STRD_BIT		16
#define MC_REF_STRD_MASK	(0xfff << MC_REF_STRD_BIT)
#define MC_REF_STRD2_BIT	0
#define MC_REF_STRD2_MASK	(0xfff << MC_REF_STRD2_BIT)

/* MC Current Frame Stride Register, unit: byte */
#define MC_CURR_STRD_BIT	0
#define MC_CURR_STRD_MASK	(0xfff << MC_CURR_STRD_BIT)

/* MC Block Interpolation Information Register */
#define MC_ITP_INFO_RND1_BIT	24  /* Rounding data during interpolation */
#define MC_ITP_INFO_RND1_MASK	(0xff << MC_ITP_INFO_RND1_BIT)
#define MC_ITP_INFO_RND0_BIT	16  /* Rounding data during interpolation */
#define MC_ITP_INFO_RND0_MASK	(0xff << MC_ITP_INFO_RND0_BIT)
#define MC_ITP_INFO_AVG		(1 << 12)  /* 0: output interpolated data directly; 1: doing average operation with 2nd source data after interpolating and output */
#define MC_ITP_INFO_FMT_BIT	8  /* Indicate current interpolation's type */
#define MC_ITP_INFO_RMT_MASK	(0xf << MC_ITP_INFO_RMT_BIT)
  #define MC_ITP_INFO_FMT_MPEG_HPEL  (0x0 << MC_ITP_INFO_RMT_BIT) /* MPEG Half-pixel interpolation */
  #define MC_ITP_INFO_FMT_MPEG_QPEL  (0x1 << MC_ITP_INFO_RMT_BIT) /* MPEG 8-tap Quarter-pixel interpolation */
  #define MC_ITP_INFO_FMT_H264_QPEL  (0x2 << MC_ITP_INFO_RMT_BIT) /* H264 6-tap Quarter-pixel interpolation */
  #define MC_ITP_INFO_FMT_H264_EPEL  (0x3 << MC_ITP_INFO_RMT_BIT) /* H264 2-tap Eight-pixel interpolation */
  #define MC_ITP_INFO_FMT_H264_WPDT  (0x4 << MC_ITP_INFO_RMT_BIT) /* H264 Weighted-prediction */
  #define MC_ITP_INFO_FMT_WMV2_QPEL  (0x5 << MC_ITP_INFO_RMT_BIT) /* WMV2 4-tap Quarter-pixel interpolation */
  #define MC_ITP_INFO_FMT_VC1_QPEL   (0x6 << MC_ITP_INFO_RMT_BIT) /* VC1 4-tap Quarter-pixel interpolation */
  #define MC_ITP_INFO_FMT_RV8_TPEL   (0x7 << MC_ITP_INFO_RMT_BIT) /* RV8 4-tap Third-pixel interpolation */
  #define MC_ITP_INFO_FMT_RV8_CHROM  (0x8 << MC_ITP_INFO_RMT_BIT) /* RV8 2-tap Third-pixel interpolation */
  #define MC_ITP_INFO_FMT_RV9_QPEL   (0x9 << MC_ITP_INFO_RMT_BIT) /* RV9 6-tap Quarter-pixel interpolation */
  #define MC_ITP_INFO_FMT_RV9_CHROM  (0xa << MC_ITP_INFO_RMT_BIT) /* RV9 2-tap Quarter-pixel interpolation */
#define MC_ITP_INFO_BLK_W_BIT	6  /* Indicate reference block's width, unit: pixel */
#define MC_ITP_INFO_BLK_W_MASK	(0x3 << MC_ITP_INFO_BLK_W_BIT)
  #define MC_ITP_INFO_BLK_W_2	(0x0 << MC_ITP_INFO_BLK_W_BIT)
  #define MC_ITP_INFO_BLK_W_4	(0x1 << MC_ITP_INFO_BLK_W_BIT)
  #define MC_ITP_INFO_BLK_W_8	(0x2 << MC_ITP_INFO_BLK_W_BIT)
  #define MC_ITP_INFO_BLK_W_16	(0x3 << MC_ITP_INFO_BLK_W_BIT)
#define MC_ITP_INFO_BLK_H_BIT	4  /* Indicate reference block's height, unit: pixel */
#define MC_ITP_INFO_BLK_H_MASK	(0x3 << MC_ITP_INFO_BLK_H_BIT)
  #define MC_ITP_INFO_BLK_H_2	(0x0 << MC_ITP_INFO_BLK_H_BIT)
  #define MC_ITP_INFO_BLK_H_4	(0x1 << MC_ITP_INFO_BLK_H_BIT)
  #define MC_ITP_INFO_BLK_H_8	(0x2 << MC_ITP_INFO_BLK_H_BIT)
  #define MC_ITP_INFO_BLK_H_16	(0x3 << MC_ITP_INFO_BLK_H_BIT)
#define MC_ITP_INFO_ITP_CASE_BIT	0  /* Indicate interpolation final destination pixel position */
#define MC_ITP_INFO_ITP_CASE_MASK	(0xf << MC_ITP_INFO_ITP_CASE_BIT)
  #define MC_ITP_INFO_ITP_CASE_H0V0	(0x0 << MC_ITP_INFO_ITP_CASE_BIT)
  #define MC_ITP_INFO_ITP_CASE_H1V0	(0x1 << MC_ITP_INFO_ITP_CASE_BIT)
  #define MC_ITP_INFO_ITP_CASE_H2V0	(0x2 << MC_ITP_INFO_ITP_CASE_BIT)
  #define MC_ITP_INFO_ITP_CASE_H3V0	(0x3 << MC_ITP_INFO_ITP_CASE_BIT)
  #define MC_ITP_INFO_ITP_CASE_H0V1	(0x4 << MC_ITP_INFO_ITP_CASE_BIT)
  #define MC_ITP_INFO_ITP_CASE_H1V1	(0x5 << MC_ITP_INFO_ITP_CASE_BIT)
  #define MC_ITP_INFO_ITP_CASE_H2V1	(0x6 << MC_ITP_INFO_ITP_CASE_BIT)
  #define MC_ITP_INFO_ITP_CASE_H3V1	(0x7 << MC_ITP_INFO_ITP_CASE_BIT)
  #define MC_ITP_INFO_ITP_CASE_H0V2	(0x8 << MC_ITP_INFO_ITP_CASE_BIT)
  #define MC_ITP_INFO_ITP_CASE_H1V2	(0x9 << MC_ITP_INFO_ITP_CASE_BIT)
  #define MC_ITP_INFO_ITP_CASE_H2V2	(0xa << MC_ITP_INFO_ITP_CASE_BIT)
  #define MC_ITP_INFO_ITP_CASE_H3V2	(0xb << MC_ITP_INFO_ITP_CASE_BIT)
  #define MC_ITP_INFO_ITP_CASE_H0V3	(0xc << MC_ITP_INFO_ITP_CASE_BIT)
  #define MC_ITP_INFO_ITP_CASE_H1V3	(0xd << MC_ITP_INFO_ITP_CASE_BIT)
  #define MC_ITP_INFO_ITP_CASE_H2V3	(0xe << MC_ITP_INFO_ITP_CASE_BIT)
  #define MC_ITP_INFO_ITP_CASE_H3V3	(0xf << MC_ITP_INFO_ITP_CASE_BIT)

/* MC TAP Filter Coefficient 1 Register */
#define MC_TAP_COEF1_TAP_COEF4_BIT	24
#define MC_TAP_COEF1_TAP_COEF4_MASK	(0xff << MC_TAP_COEF1_TAP_COEF4_BIT)
#define MC_TAP_COEF1_TAP_COEF3_BIT	16
#define MC_TAP_COEF1_TAP_COEF3_MASK	(0xff << MC_TAP_COEF1_TAP_COEF3_BIT)
#define MC_TAP_COEF1_TAP_COEF2_BIT	8
#define MC_TAP_COEF1_TAP_COEF2_MASK	(0xff << MC_TAP_COEF1_TAP_COEF2_BIT)
#define MC_TAP_COEF1_TAP_COEF1_BIT	0
#define MC_TAP_COEF1_TAP_COEF1_MASK	(0xff << MC_TAP_COEF1_TAP_COEF1_BIT)

/* MC TAP Filter Coefficient 2 Register */
#define MC_TAP_COEF2_TAP_COEF8_BIT	24
#define MC_TAP_COEF2_TAP_COEF8_MASK	(0xff << MC_TAP_COEF2_TAP_COEF8_BIT)
#define MC_TAP_COEF2_TAP_COEF7_BIT	16
#define MC_TAP_COEF2_TAP_COEF7_MASK	(0xff << MC_TAP_COEF2_TAP_COEF7_BIT)
#define MC_TAP_COEF2_TAP_COEF6_BIT	8
#define MC_TAP_COEF2_TAP_COEF6_MASK	(0xff << MC_TAP_COEF2_TAP_COEF6_BIT)
#define MC_TAP_COEF2_TAP_COEF5_BIT	0
#define MC_TAP_COEF2_TAP_COEF5_MASK	(0xff << MC_TAP_COEF2_TAP_COEF5_BIT)


/*************************************************************************
 * ME (Motion Estimation)
 *************************************************************************/
#define ME_CTRL		(ME_BASE + 0x00) /* ME Control Register */
#define ME_REF_ADDR	(ME_BASE + 0x04) /* ME Reference Block Address Register */
#define ME_CURR_ADDR	(ME_BASE + 0x08) /* ME Current Block Address Register */
#define ME_DIFF_ADDR	(ME_BASE + 0x0C) /* ME Difference Address Register */
#define ME_REF_STRD	(ME_BASE + 0x10) /* ME Reference Frame Stride Register */
#define ME_CURR_STRD	(ME_BASE + 0x14) /* ME Current Frame Stride Register */
#define ME_DIFF_STRD	(ME_BASE + 0x18) /* ME Difference Frame Stride Register */
#define ME_SETTINGS	(ME_BASE + 0x1C) /* ME Settings Register */
#define ME_MVD		(ME_BASE + 0x20) /* ME Motion Vector Difference Register */
#define ME_FLAG		(ME_BASE + 0x24) /* ME Flag Register */

#define REG_ME_CTRL		REG32(ME_CTRL)
#define REG_ME_REF_ADDR		REG32(ME_REF_ADDR)
#define REG_ME_CURR_ADDR	REG32(ME_CURR_ADDR)
#define REG_ME_DIFF_ADDR	REG32(ME_DIFF_ADDR)
#define REG_ME_REF_STRD		REG32(ME_REF_STRD)
#define REG_ME_CURR_STRD	REG32(ME_CURR_STRD)
#define REG_ME_DIFF_STRD	REG32(ME_DIFF_STRD)
#define REG_ME_SETTINGS		REG32(ME_SETTINGS)
#define REG_ME_MVD		REG32(ME_MVD)
#define REG_ME_FLAG		REG32(ME_FLAG)


/* ME Control Register */
#define ME_CTRL_FLUSH		(1 << 2) /* ME cache clear */
#define ME_CTRL_RESET		(1 << 1) /* ME reset */
#define ME_CTRL_ENABLE		(1 << 0) /* ME enable */

/* ME Settings Register */
#define ME_SETTINGS_SAD_GATE_BIT	16 /* The max SAD value which can be accepted */
#define ME_SETTINGS_SAD_GATE_MASK	(0xffff << ME_SETTINGS_SAD_GATE_BIT)
#define ME_SETTINGS_STEP_NUM_BIT	0  /* The max step number the search process can not exceed */
#define ME_SETTINGS_STEP_NUM_MASK	(0x3f << ME_SETTINGS_STEP_NUM_BIT)

/* ME Motion Vector Difference Register */
#define ME_MVD_MVDY_BIT		16 /* The MVD value of coordinate-Y */
#define ME_MVD_MVDY_MASK	(0xffff << ME_MVD_MVDY_BIT)
#define ME_MVD_MVDX_BIT		0  /* The MVD value of coordinate-X */
#define ME_MVD_MVDX_MASK	(0xffff << ME_MVD_MVDX_BIT)

/* ME Flag Register */
#define ME_FLAG_INTRA		(1 << 1) /* Indicate the current MB will be predicted in intra mode */
#define ME_FLAG_COMPLETED	(1 << 0) /* The ME of the current part of the MB is completed */


/*************************************************************************
 * OTP (One Time Programmable Module)
 *************************************************************************/
#define OTP_ID0		(OTP_BASE + 0x00) /* ID0 Register */
#define OTP_ID1		(OTP_BASE + 0x04) /* ID1 Register */
#define OTP_ID2		(OTP_BASE + 0x08) /* ID2 Register */
#define OTP_ID3		(OTP_BASE + 0x0C) /* ID3 Register */
#define OTP_BR0		(OTP_BASE + 0x10) /* BOOTROM0 Register */
#define OTP_BR1		(OTP_BASE + 0x14) /* BOOTROM1 Register */
#define OTP_HW0		(OTP_BASE + 0x18) /* Chip Hardware 0 Register */
#define OTP_HW1		(OTP_BASE + 0x1C) /* Chip Hardware 1 Register */

#define REG_OTP_ID0	REG32(OTP_ID0)
#define REG_OTP_ID1	REG32(OTP_ID1)
#define REG_OTP_ID2	REG32(OTP_ID2)
#define REG_OTP_ID3	REG32(OTP_ID3)
#define REG_OTP_BR0	REG32(OTP_BR0)
#define REG_OTP_BR1	REG32(OTP_BR1)
#define REG_OTP_HW0	REG32(OTP_HW0)
#define REG_OTP_HW1	REG32(OTP_HW1)

/* ID0 Register */
#define OTP_ID0_WID_BIT		24 /* Wafer ID */
#define OTP_ID0_WID_MASK	(0xff << OTP_ID0_WID_BIT)
#define OTP_ID0_MID_BIT		16 /* MASK ID */
#define OTP_ID0_MID_MASK	(0xff << OTP_ID0_MID_BIT)
#define OTP_ID0_FID_BIT		8 /* Foundary ID */
#define OTP_ID0_FID_MASK	(0xff << OTP_ID0_FID_BIT)
#define OTP_ID0_PID_BIT		0 /* Product ID */
#define OTP_ID0_PID_MASK	(0xff << OTP_ID0_PID_BIT)

/* ID1 Register */
#define OTP_ID1_LID_BIT		8 /* Lot ID */
#define OTP_ID1_LID_MASK	(0xffffff << OTP_ID1_LID_BIT)
#define OTP_ID1_TID_BIT		0 /* Test House ID */
#define OTP_ID1_TID_MASK	(0xff << OTP_ID1_TID_BIT)

/* ID2 Register */
#define OTP_ID2_XADR_BIT	24 /* Die X-dir Address */
#define OTP_ID2_XADR_MASK	(0xff << OTP_ID2_XADR_BIT)
#define OTP_ID2_YADR_BIT	16 /* Die Y-dir Address */
#define OTP_ID2_YADR_MASK	(0xff << OTP_ID2_YADR_BIT)
#define OTP_ID2_TDATE_BIT	0  /* Testing Date */
#define OTP_ID2_TDATE_MASK	(0xffff << OTP_ID2_TDATE_BIT)

/* ID3 Register */
#define OTP_ID3_CID_BIT		16 /* Customer ID */
#define OTP_ID3_CID_MASK	(0xffff << OTP_ID3_CID_BIT)
#define OTP_ID3_CP_BIT		0 /* Chip Parameters */
#define OTP_ID3_CP_MASK		(0xffff << OTP_ID3_CP_BIT)

/* BOOTROM1 Register */
#define OTP_BR1_UDCBOOT_BIT	0
#define OTP_BR1_UDCBOOT_MASK	(0xff << OTP_BR1_UDCBOOT_BIT)
  #define OTP_BR1_UDCBOOT_AUTO	(0xf0 << OTP_BR1_UDCBOOT_BIT)
  #define OTP_BR1_UDCBOOT_24M	(0x0f << OTP_BR1_UDCBOOT_BIT) /* 24MHz OSC */
  #define OTP_BR1_UDCBOOT_13M	(0x0c << OTP_BR1_UDCBOOT_BIT) /* 13MHz OSC */
  #define OTP_BR1_UDCBOOT_26M	(0x03 << OTP_BR1_UDCBOOT_BIT) /* 26MHz OSC */
  #define OTP_BR1_UDCBOOT_27M	(0x00 << OTP_BR1_UDCBOOT_BIT) /* 27MHz OSC */

/* Chip Hardware 1 Register */
#define OTP_HW1_MC_EN		(0x3 << 30) /* MC is enabled */
#define OTP_HW1_ME_EN		(0x3 << 28)
#define OTP_HW1_DE_EN		(0x3 << 26)
#define OTP_HW1_IDCT_EN		(0x3 << 24)
#define OTP_HW1_UART3_EN	(0x3 << 22)
#define OTP_HW1_UART2_EN	(0x3 << 20)
#define OTP_HW1_UART1_EN	(0x3 << 18)
#define OTP_HW1_UART0_EN	(0x3 << 16)
#define OTP_HW1_SSI1_EN		(0x3 << 14)
#define OTP_HW1_SSI0_EN		(0x3 << 12)
#define OTP_HW1_MSC1_EN		(0x3 << 10)
#define OTP_HW1_MSC0_EN		(0x3 << 8)
#define OTP_HW1_UHC_EN		(0x3 << 6)
#define OTP_HW1_TVE_EN		(0x3 << 4)
#define OTP_HW1_TSSI_EN		(0x3 << 2)
#define OTP_HW1_CIM_EN		(0x3 << 0)


/*************************************************************************
 * TSSI MPEG 2-TS slave interface
 *************************************************************************/
#define TSSI_ENA       ( TSSI_BASE + 0x00 )   /* TSSI enable register */
#define TSSI_CFG       ( TSSI_BASE + 0x04 )   /* TSSI configure register */
#define TSSI_CTRL      ( TSSI_BASE + 0x08 )   /* TSSI control register */
#define TSSI_STAT      ( TSSI_BASE + 0x0c )   /* TSSI state register */
#define TSSI_FIFO      ( TSSI_BASE + 0x10 )   /* TSSI FIFO register */
#define TSSI_PEN       ( TSSI_BASE + 0x14 )   /* TSSI PID enable register */
#define TSSI_PID(n)    ( TSSI_BASE + 0x20 + 4*(n) )   /* TSSI PID filter register */
#define TSSI_PID0      ( TSSI_BASE + 0x20 )
#define TSSI_PID1      ( TSSI_BASE + 0x24 )
#define TSSI_PID2      ( TSSI_BASE + 0x28 )
#define TSSI_PID3      ( TSSI_BASE + 0x2c )
#define TSSI_PID4      ( TSSI_BASE + 0x30 )
#define TSSI_PID5      ( TSSI_BASE + 0x34 )
#define TSSI_PID6      ( TSSI_BASE + 0x38 )
#define TSSI_PID7      ( TSSI_BASE + 0x3c )
#define TSSI_PID_MAX   8	/* max PID: 7 */

#define REG_TSSI_ENA       REG8( TSSI_ENA )
#define REG_TSSI_CFG       REG16( TSSI_CFG )
#define REG_TSSI_CTRL      REG8( TSSI_CTRL )
#define REG_TSSI_STAT      REG8( TSSI_STAT )
#define REG_TSSI_FIFO      REG32( TSSI_FIFO )
#define REG_TSSI_PEN       REG32( TSSI_PEN )
#define REG_TSSI_PID(n)    REG32( TSSI_PID(n) )
#define REG_TSSI_PID0      REG32( TSSI_PID0 )
#define REG_TSSI_PID1      REG32( TSSI_PID1 )
#define REG_TSSI_PID2      REG32( TSSI_PID2 )
#define REG_TSSI_PID3      REG32( TSSI_PID3 )
#define REG_TSSI_PID4      REG32( TSSI_PID4 )
#define REG_TSSI_PID5      REG32( TSSI_PID5 )
#define REG_TSSI_PID6      REG32( TSSI_PID6 )
#define REG_TSSI_PID7      REG32( TSSI_PID7 )

/* TSSI enable register */
#define TSSI_ENA_SFT_RST 	( 1 << 7 )      /* soft reset bit */
#define TSSI_ENA_PID_EN 	( 1 << 2 )      /* soft filtering function enable bit */
#define TSSI_ENA_DMA_EN 	( 1 << 1 )      /* DMA enable bit */
#define TSSI_ENA_ENA 		( 1 << 0 )      /* TSSI enable bit */

/* TSSI configure register */
#define TSSI_CFG_TRIG_BIT 	14 /* fifo trig number */
#define TSSI_CFG_TRIG_MASK 	( 0x3 << TSSI_CFG_TRIG_BIT)
#define TSSI_CFG_TRIG_4 	( 0 << TSSI_CFG_TRIG_BIT)
#define TSSI_CFG_TRIG_8 	( 1 << TSSI_CFG_TRIG_BIT)
#define TSSI_CFG_TRIG_16 	( 2 << TSSI_CFG_TRIG_BIT)
#define TSSI_CFG_END_WD 	( 1 << 9 )      /* order of data in word */
#define TSSI_CFG_END_BT 	( 1 << 8 )      /* order of data in byte */
#define TSSI_CFG_TSDI_H 	( 1 << 7 )      /* data pin polarity */
#define TSSI_CFG_USE_0 		( 1 << 6 )      /* serial mode data pin select */
#define TSSI_CFG_USE_TSDI0 	( 0 << 6 )      /* TSDI0 as serial mode data pin */
#define TSSI_CFG_USE_TSDI7 	( 1 << 6 )      /* TSDI7 as serial mode data pin */
#define TSSI_CFG_TSCLK_CH 	( 1 << 5 )      /* clk channel select */
#define TSSI_CFG_PARAL 		( 1 << 4 )      /* mode select */
#define TSSI_CFG_PARAL_MODE 	( 1 << 4 )      /* parallel select */
#define TSSI_CFG_SERIAL_MODE 	( 0 << 4 )      /* serial select */
#define TSSI_CFG_TSCLK_P 	( 1 << 3 )      /* clk edge select */
#define TSSI_CFG_TSFRM_H 	( 1 << 2 )      /* TSFRM polarity select */
#define TSSI_CFG_TSSTR_H 	( 1 << 1 )      /* TSSTR polarity select */
#define TSSI_CFG_TSFAIL_H 	( 1 << 0 )      /* TSFAIL polarity select */

/* TSSI control register */
#define TSSI_CTRL_OVRNM 	( 1 << 1 )      /* FIFO overrun interrupt mask bit */
#define TSSI_CTRL_TRIGM 	( 1 << 0 )      /* FIFO trigger interrupt mask bit */

/* TSSI state register */
#define TSSI_STAT_OVRN 		( 1 << 1 )      /* FIFO overrun interrupt flag bit */
#define TSSI_STAT_TRIG 		( 1 << 0 )      /* FIFO trigger interrupt flag bit */

/* TSSI PID enable register */
#define TSSI_PEN_EN00 	( 1 << 0 )      /* enable PID n */
#define TSSI_PEN_EN10 	( 1 << 1 )
#define TSSI_PEN_EN20 	( 1 << 2 )
#define TSSI_PEN_EN30 	( 1 << 3 )
#define TSSI_PEN_EN40 	( 1 << 4 )
#define TSSI_PEN_EN50 	( 1 << 5 )
#define TSSI_PEN_EN60 	( 1 << 6 )
#define TSSI_PEN_EN70 	( 1 << 7 )
#define TSSI_PEN_EN01 	( 1 << 16 )
#define TSSI_PEN_EN11 	( 1 << 17 )
#define TSSI_PEN_EN21 	( 1 << 18 )
#define TSSI_PEN_EN31 	( 1 << 19 )
#define TSSI_PEN_EN41 	( 1 << 20 )
#define TSSI_PEN_EN51 	( 1 << 21 )
#define TSSI_PEN_EN61 	( 1 << 22 )
#define TSSI_PEN_EN71 	( 1 << 23 )
#define TSSI_PEN_PID0 	( 1 << 31 ) /* PID filter enable PID0 */

/* TSSI PID Filter Registers */
#define TSSI_PID_PID1_BIT 	16
#define TSSI_PID_PID1_MASK 	(0x1FFF<<TSSI_PID_PID1_BIT)
#define TSSI_PID_PID0_BIT 	0
#define TSSI_PID_PID0_MASK 	(0x1FFF<<TSSI_PID_PID0_BIT)


/*************************************************************************
 * IPU (Image Processing Unit)
 *************************************************************************/

/* IPU Control Register */
#define REG_IPU_CTRL			(IPU_BASE + 0x0)

/* IPU Status Register */
#define REG_IPU_STATUS			(IPU_BASE + 0x4)

/* Data Format Register */
#define REG_IPU_D_FMT			(IPU_BASE + 0x8)

/* Input Y or YUV422 Packaged Data Address Register */
#define REG_IPU_Y_ADDR			(IPU_BASE + 0xc)

/* Input U Data Address Register */
#define REG_IPU_U_ADDR			(IPU_BASE + 0x10)

/* Input V Data Address Register */
#define REG_IPU_V_ADDR			(IPU_BASE + 0x14)

/* Input Geometric Size Register */
#define REG_IPU_IN_FM_GS		(IPU_BASE + 0x18)

/* Input Y Data Line Stride Register */
#define REG_IPU_Y_STRIDE		(IPU_BASE + 0x1c)

/* Input UV Data Line Stride Register */
#define REG_IPU_UV_STRIDE		(IPU_BASE + 0x20)

/* Output Frame Start Address Register */
#define REG_IPU_OUT_ADDR		(IPU_BASE + 0x24)

/* Output Geometric Size Register */
#define REG_IPU_OUT_GS			(IPU_BASE + 0x28)

/* Output Data Line Stride Register */
#define REG_IPU_OUT_STRIDE		(IPU_BASE + 0x2c)

/* Resize Coefficients Table Index Register */
#define REG_IPU_RSZ_COEF_INDEX		(IPU_BASE + 0x30)

/* CSC C0 Coefficient Register */
#define REG_IPU_CSC_CO_COEF		(IPU_BASE + 0x34)

/* CSC C1 Coefficient Register */
#define REG_IPU_CSC_C1_COEF		(IPU_BASE + 0x38)

/* CSC C2 Coefficient Register */
#define REG_IPU_CSC_C2_COEF		(IPU_BASE + 0x3c)

/* CSC C3 Coefficient Register */
#define REG_IPU_CSC_C3_COEF		(IPU_BASE + 0x40)

/* CSC C4 Coefficient Register */
#define REG_IPU_CSC_C4_COEF		(IPU_BASE + 0x44)

/* Horizontal Resize Coefficients Look Up Table Register group */
#define REG_IPU_HRSZ_LUT_BASE 		(IPU_BASE + 0x48)

/* Virtical Resize Coefficients Look Up Table Register group */
#define REG_IPU_VRSZ_LUT_BASE 		(IPU_BASE + 0x4c)

/* CSC Offset Parameter Register */
#define REG_IPU_CSC_OFSET_PARA		(IPU_BASE + 0x50)

/* Input Y Physical Table Address Register */
#define REG_IPU_Y_PHY_T_ADDR		(IPU_BASE + 0x54)

/* Input U Physical Table Address Register */
#define REG_IPU_U_PHY_T_ADDR		(IPU_BASE + 0x58)

/* Input V Physical Table Address Register */
#define REG_IPU_V_PHY_T_ADDR		(IPU_BASE + 0x5c)

/* Output Physical Table Address Register */
#define REG_IPU_OUT_PHY_T_ADDR		(IPU_BASE + 0x60)

/* IPU Control */
#define IPU_CTRL_DFIX_SEL			(1 << 17)
#define IPU_CTRL_FIELD_SEL			(1 << 16)
#define IPU_CTRL_FIELD_CONF_EN			(1 << 15)
#define IPU_CTRL_DISP_SEL			(1 << 14)
#define IPU_CTRL_DPAGE_MAP			(1 << 13)
#define IPU_CTRL_SPAGE_MAP 			(1 << 12)
#define IPU_CTRL_LCDC_SEL			(1 << 11)
#define IPU_CTRL_SPKG_SEL			(1 << 10)
#define IPU_CTRL_V_SCALE			(1 << 9)
#define IPU_CTRL_H_SCALE			(1 << 8)
#define IPU_CTRL_IPU_RST			(1 << 6)
#define IPU_CTRL_FM_IRQ_EN			(1 << 5)
#define IPU_CTRL_CSC_EN				(1 << 4)
#define IPU_CTRL_VRSZ_EN			(1 << 3)
#define IPU_CTRL_HRSZ_EN			(1 << 2)
#define IPU_CTRL_IPU_RUN			(1 << 1)
#define IPU_CTRL_CHIP_EN			(1 << 0)

/* IPU Status */
#define IPU_STAT_SIZE_ERR			(1 << 2)
#define IPU_STAT_FMT_ERR			(1 << 1)
#define IPU_STAT_OUT_END			(1 << 0)

/* IPU Data Format */
#define IPU_D_FMT_RGB_OUT_888_FMT		(1 << 24)

#define IPU_D_FMT_RGB_OUT_OFT_MASK		(0x7 << 21)

#define IPU_D_FMT_RGB_OUT_OFT_RGB		(0 << 21)
#define IPU_D_FMT_RGB_OUT_OFT_RBG		(1 << 21)
#define IPU_D_FMT_RGB_OUT_OFT_GBR		(2 << 21)
#define IPU_D_FMT_RGB_OUT_OFT_GRB		(3 << 21)
#define IPU_D_FMT_RGB_OUT_OFT_BRG		(4 << 21)
#define IPU_D_FMT_RGB_OUT_OFT_BGR		(5 << 21)

#define IPU_D_FMT_OUT_FMT_MASK			(0x3 << 19)

#define IPU_D_FMT_OUT_FMT_RGB555		(0 << 19)
#define IPU_D_FMT_OUT_FMT_RGB565		(1 << 19)
#define IPU_D_FMT_OUT_FMT_RGB888		(2 << 19)
#define IPU_D_FMT_OUT_FMT_YUV422		(3 << 19)

#define IPU_D_FMT_YUV_PKG_OUT_OFT_MASK		(0x7 << 16)

#define IPU_D_FMT_YUV_PKG_OUT_OFT_Y1UY0V	(0 << 16)
#define IPU_D_FMT_YUV_PKG_OUT_OFT_Y1VY0U	(1 << 16)
#define IPU_D_FMT_YUV_PKG_OUT_OFT_UY1VY0	(2 << 16)
#define IPU_D_FMT_YUV_PKG_OUT_OFT_VY1UY0	(3 << 16)
#define IPU_D_FMT_YUV_PKG_OUT_OFT_Y0UY1V	(4 << 16)
#define IPU_D_FMT_YUV_PKG_OUT_OFT_Y0VY1U	(5 << 16)
#define IPU_D_FMT_YUV_PKG_OUT_OFT_UY0VY1	(6 << 16)
#define IPU_D_FMT_YUV_PKG_OUT_OFT_VY0UY1	(7 << 16)

#define IPU_D_FMT_IN_OFT_MASK			(0x3 << 2)

#define IPU_D_FMT_IN_OFT_Y1UY0V			(0 << 2)
#define IPU_D_FMT_IN_OFT_Y1VY0U			(1 << 2)
#define IPU_D_FMT_IN_OFT_UY1VY0			(2 << 2)
#define IPU_D_FMT_IN_OFT_VY1UY0			(3 << 2)

#define IPU_D_FMT_IN_FMT_MASK			(0x3 << 0)

#define IPU_D_FMT_IN_FMT_YUV420			(0 << 0)
#define IPU_D_FMT_IN_FMT_YUV422			(1 << 0)
#define IPU_D_FMT_IN_FMT_YUV444			(2 << 0)
#define IPU_D_FMT_IN_FMT_YUV411			(3 << 0)

/* Input Geometric Size Register */
#define IPU_IN_FM_GS_W_MASK			(0xFFF)
#define IPU_IN_FM_GS_W(n)			((n) << 16)

#define IPU_IN_FM_GS_H_MASK			(0xFFF)
#define IPU_IN_FM_GS_H(n)			((n) << 0)

/* Input UV Data Line Stride Register */
#define IPU_UV_STRIDE_U_S_MASK			(0x1FFF)
#define IPU_UV_STRIDE_U_S(n) 			((n) << 16)

#define IPU_UV_STRIDE_V_S_MASK			(0x1FFF)
#define IPU_UV_STRIDE_V_S(n)			((n) << 0)

/* Output Geometric Size Register */
#define IPU_OUT_GS_W_MASK			(0x7FFF)
#define IPU_OUT_GS_W(n)				((n) << 16)

#define IPU_OUT_GS_H_MASK			(0x1FFF)
#define IPU_OUT_GS_H(n)				((n) << 0)

/* Resize Coefficients Table Index Register */
#define IPU_RSZ_COEF_INDEX_HE_IDX_MASK		(0x1F)
#define IPU_RSZ_COEF_INDEX_HE_IDX(n)		((n) << 16)

#define IPU_RSZ_COEF_INDEX_VE_IDX_MASK		(0x1F)
#define IPU_RSZ_COEF_INDEX_VE_IDX(n)		((n) << 0)

/* Resize Coefficients Look Up Table Register group */
#define IPU_HRSZ_COEF_LUT_START			(1 << 12)

#define IPU_HRSZ_COEF_LUT_W_COEF_MASK		(0x3FF)
#define IPU_HRSZ_COEF_LUT_W_COEF(n)		((n) << 2)

#define IPU_HRSZ_COEF_LUT_IN_EN			(1 << 1)
#define IPU_HRSZ_COEF_LUT_OUT_EN		(1 << 0)

#define IPU_VRSZ_COEF_LUT_START			(1 << 12)

#define IPU_VRSZ_COEF_LUT_W_COEF_MASK		(0x3FF)
#define IPU_VRSZ_COEF_LUT_W_COEF(n)		((n) << 2)

#define IPU_VRSZ_COEF_LUT_IN_EN			(1 << 1)
#define IPU_VRSZ_COEF_LUT_OUT_EN		(1 << 0)

/* CSC Offset Parameter Register */
#define IPU_CSC_OFFSET_PARA_CHROM_OF_MASK	(0xFF)
#define IPU_CSC_OFFSET_PARA_CHROM_OF(n)		((n) << 16)

#define IPU_CSC_OFFSET_LUMA_OF_MASK		(0xFF)
#define IPU_CSC_OFFSET_LUMA_OF(n)		((n) << 0)

#endif /* __JZ4750D_REGS_H__ */
