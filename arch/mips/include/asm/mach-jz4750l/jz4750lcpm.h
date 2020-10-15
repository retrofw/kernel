/*
 * linux/include/asm-mips/mach-jz4750l/jz4750lcpm.h
 *
 * JZ4750L CPM register definition
 *
 * Copyright (C) 2008-2014 Ingenic Semiconductor Co., Ltd.
 */
#ifndef __JZ4750LCPM_H__
#define __JZ4750LCPM_H__

/*
 * Clock reset and power controller module(CPM) address definition
 */
#define	CPM_BASE	0xB0000000

/*
 * CPM registers address definition
 */
#define CPM_CPCCR	(CPM_BASE + 0x00)
#define CPM_CPPCR	(CPM_BASE + 0x10)
#define CPM_CPPSR	(CPM_BASE + 0x14) /* PLL Switch and Status Register */
#define CPM_I2SCDR	(CPM_BASE + 0x60)
#define CPM_LPCDR	(CPM_BASE + 0x64)
#define CPM_MSCCDR(n)	(CPM_BASE + 0x68) /* MSC0(n=0) or MSC1(n=1) device clock divider Register */
#define CPM_UHCCDR	(CPM_BASE + 0x6C)
#define CPM_SSICDR	(CPM_BASE + 0x74)
#define CPM_PCMCDR	(CPM_BASE + 0x7C) /* PCM device clock divider Register */

#define CPM_LCR		(CPM_BASE + 0x04)
#define CPM_CLKGR	(CPM_BASE + 0x20)
#define CPM_OPCR	(CPM_BASE + 0x24) /* Oscillator and Power Control Register */

#define CPM_RSR		(CPM_BASE + 0x08)

/*
 * CPM registers definition
 */
#define REG_CPM_CPCCR		REG32(CPM_CPCCR)
#define REG_CPM_CPPCR		REG32(CPM_CPPCR)
#define REG_CPM_CPPSR		REG32(CPM_CPPSR)
#define REG_CPM_I2SCDR  	REG32(CPM_I2SCDR)
#define REG_CPM_LPCDR   	REG32(CPM_LPCDR)
#define REG_CPM_MSCCDR(n)	REG32(CPM_MSCCDR(n))
#define REG_CPM_UHCCDR		REG32(CPM_UHCCDR)
#define REG_CPM_SSICDR		REG32(CPM_SSICDR)
#define REG_CPM_PCMCDR		REG32(CPM_PCMCDR)

#define REG_CPM_LCR		REG32(CPM_LCR)
#define REG_CPM_CLKGR		REG32(CPM_CLKGR)
#define REG_CPM_OPCR		REG32(CPM_OPCR)

#define REG_CPM_RSR		REG32(CPM_RSR)

/* Clock Control Register */
#define CPM_CPCCR_I2CS		BIT31
#define CPM_CPCCR_ECS   	BIT30 /* Select the between EXCLK and EXCLK/2 output */
#define CPM_CPCCR_UCS		BIT29
#define CPM_CPCCR_UDIV_BIT	23
#define CPM_CPCCR_UDIV_MASK	(0x3f << CPM_CPCCR_UDIV_BIT)
#define CPM_CPCCR_CE		BIT22
#define CPM_CPCCR_PCS		BIT21
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
#define CPM_CPPSR_PLLOFF        BIT31
#define CPM_CPPSR_PLLBP         BIT30
#define CPM_CPPSR_PLLON         BIT29
#define CPM_CPPSR_PS            BIT28 /* Indicate whether the PLL parameters' change has finished */
#define CPM_CPPSR_FS            BIT27 /* Indicate whether the main clock's change has finished */
#define CPM_CPPSR_CS            BIT26 /* Indicate whether the clock switch has finished */
#define CPM_CPPSR_SM            BIT2  /* Clock stop mode */
#define CPM_CPPSR_PM            BIT1  /* Clock switch mode */
#define CPM_CPPSR_FM            BIT0  /* Clock frequency change mode */

/* I2S Clock Divider Register */
#define CPM_I2SCDR_I2SDIV_BIT	0
#define CPM_I2SCDR_I2SDIV_MASK	(0x1ff << CPM_I2SCDR_I2SDIV_BIT)

/* LCD Pixel Clock Divider Register */
#define CPM_LPCDR_PIXDIV_BIT	0
#define CPM_LPCDR_PIXDIV_MASK	(0x7ff << CPM_LPCDR_PIXDIV_BIT)

/* MSC Clock Divider Register */
#define CPM_MSCCDR_MSCDIV_BIT	0
#define CPM_MSCCDR_MSCDIV_MASK	(0x1f << CPM_MSCCDR_MSCDIV_BIT)

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
#define CPM_CPPCR_PLLS		BIT10 /* obsolete, replaced by CPM_CPPSR_PLLON */
#define CPM_CPPCR_PLLBP		BIT9
#define CPM_CPPCR_PLLEN		BIT8
#define CPM_CPPCR_PLLST_BIT	0
#define CPM_CPPCR_PLLST_MASK	(0xff << CPM_CPPCR_PLLST_BIT)

/* Low Power Control Register */
#define CPM_LCR_DOZE_DUTY_BIT 	3
#define CPM_LCR_DOZE_DUTY_MASK 	(0x1f << CPM_LCR_DOZE_DUTY_BIT)
#define CPM_LCR_DOZE_ON		BIT2
#define CPM_LCR_LPM_BIT		0
#define CPM_LCR_LPM_MASK	(0x3 << CPM_LCR_LPM_BIT)
  #define CPM_LCR_LPM_IDLE	(0x0 << CPM_LCR_LPM_BIT)
  #define CPM_LCR_LPM_SLEEP	(0x1 << CPM_LCR_LPM_BIT)

/* Clock Gate Register */
#define CPM_CLKGR_AUX_CPU	BIT24
#define CPM_CLKGR_AHB1  	BIT23
#define CPM_CLKGR_IDCT  	BIT22
#define CPM_CLKGR_DB    	BIT21
#define CPM_CLKGR_ME    	BIT20
#define CPM_CLKGR_MC    	BIT19
#define CPM_CLKGR_TVE    	BIT18
#define CPM_CLKGR_TSSI    	BIT17
#define CPM_CLKGR_MSC1    	BIT16
#define CPM_CLKGR_UART2    	BIT15
#define CPM_CLKGR_UART1		BIT14
#define CPM_CLKGR_IPU		BIT13
#define CPM_CLKGR_DMAC		BIT12
#define CPM_CLKGR_BCH		BIT11
#define CPM_CLKGR_UDC		BIT10
#define CPM_CLKGR_LCD		BIT9
#define CPM_CLKGR_CIM		BIT8
#define CPM_CLKGR_SADC		BIT7
#define CPM_CLKGR_MSC0		BIT6
#define CPM_CLKGR_AIC		BIT5
#define CPM_CLKGR_SSI		BIT4
#define CPM_CLKGR_I2C		BIT3
#define CPM_CLKGR_RTC		BIT2
#define CPM_CLKGR_TCU		BIT1
#define CPM_CLKGR_UART0		BIT0

/* Oscillator and Power Control Register */
#define CPM_OPCR_O1ST_BIT	8
#define CPM_OPCR_O1ST_MASK	(0xff << CPM_SCR_O1ST_BIT)
#define CPM_OPCR_UHCPHY_DISABLE	BIT7
#define CPM_OPCR_UDCPHY_ENABLE	BIT6
#define CPM_OPCR_OSC_ENABLE	BIT4
#define CPM_OPCR_ERCS           BIT2 /* EXCLK/512 clock and RTCLK clock selection */
#define CPM_OPCR_MOSE           BIT1 /* Main Oscillator Enable */
#define CPM_OPCR_MCS            BIT0 /* Main clock source select register */

/* Reset Status Register */
#define CPM_RSR_HR		BIT2
#define CPM_RSR_WR		BIT1
#define CPM_RSR_PR		BIT0

/*
 * CPM Register ops
 */
#ifndef __MIPS_ASSEMBLER
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
#define __cpm_get_h1div() \
	((REG_CPM_CPCCR & CPM_CPCCR_H1DIV_MASK) >> CPM_CPCCR_H1DIV_BIT)
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
#define __cpm_set_h1div(v) \
	(REG_CPM_CPCCR = (REG_CPM_CPCCR & ~CPM_CPCCR_H1DIV_MASK) | ((v) << (CPM_CPCCR_H1DIV_BIT)))
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
#define __cpm_stop_ssi(n)	(REG_CPM_CLKGR |= CPM_CLKGR_SSI)
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
#define __cpm_start_ssi(n)	(REG_CPM_CLKGR &= ~CPM_CLKGR_SSI)
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
#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4750LCPM_H__ */
