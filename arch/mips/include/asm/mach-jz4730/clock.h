/*
 *  linux/include/asm-mips/mach-jz4730/clock.h
 *
 *  JZ4730 clocks definition.
 *
 *  Copyright (C) 2006 - 2007 Ingenic Semiconductor Inc.
 *
 *  Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4730_CLOCK_H__
#define __ASM_JZ4730_CLOCK_H__

#ifndef JZ_EXTAL
#define JZ_EXTAL 3686400
#endif

#ifndef JZ_EXTAL2
#define JZ_EXTAL2 32768
#endif

/*
 * JZ4730 clocks structure
 */
typedef struct {
	unsigned int iclk;	/* CPU core clock */
	unsigned int sclk;	/* AHB bus clock */
	unsigned int mclk;	/* Memory bus clock */
	unsigned int pclk;	/* APB bus clock */
	unsigned int devclk;	/* Devcie clock to specific modules */
	unsigned int rtcclk;	/* RTC module clock */
	unsigned int uartclk;	/* UART module clock */
	unsigned int lcdclk;	/* LCD module clock */
	unsigned int pixclk;	/* LCD pixel clock */
	unsigned int usbclk;	/* USB module clock */
	unsigned int i2sclk;	/* I2S module clock */
	unsigned int mscclk;	/* MMC/SD module clock */
} jz_clocks_t;

extern jz_clocks_t jz_clocks;


static __inline__ unsigned int __cpm_get_pllout(void)
{
	unsigned int nf, nr, no, pllout;
	unsigned long plcr = REG_CPM_PLCR1;
	unsigned long od[4] = {1, 2, 2, 4};
	if (plcr & CPM_PLCR1_PLL1EN) {
		nf = (plcr & CPM_PLCR1_PLL1FD_MASK) >> CPM_PLCR1_PLL1FD_BIT;
		nr = (plcr & CPM_PLCR1_PLL1RD_MASK) >> CPM_PLCR1_PLL1RD_BIT;
		no = od[((plcr & CPM_PLCR1_PLL1OD_MASK) >> CPM_PLCR1_PLL1OD_BIT)];
		pllout = (JZ_EXTAL) / ((nr+2) * no) * (nf+2);
	} else
		pllout = JZ_EXTAL;
	return pllout;
}

static __inline__ unsigned int __cpm_get_iclk(void)
{
	unsigned int iclk;
	int div[] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32};
	unsigned long cfcr = REG_CPM_CFCR;
	unsigned long plcr = REG_CPM_PLCR1;
	if (plcr & CPM_PLCR1_PLL1EN)
		iclk = __cpm_get_pllout() /
		       div[(cfcr & CPM_CFCR_IFR_MASK) >> CPM_CFCR_IFR_BIT];
	else
		iclk = JZ_EXTAL;
	return iclk;
}

static __inline__ unsigned int __cpm_get_sclk(void)
{
	unsigned int sclk;
	int div[] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32};
	unsigned long cfcr = REG_CPM_CFCR;
	unsigned long plcr = REG_CPM_PLCR1;
	if (plcr & CPM_PLCR1_PLL1EN)
		sclk = __cpm_get_pllout() /
		       div[(cfcr & CPM_CFCR_SFR_MASK) >> CPM_CFCR_SFR_BIT];
	else
		sclk = JZ_EXTAL;
	return sclk;
}

static __inline__ unsigned int __cpm_get_mclk(void)
{
	unsigned int mclk;
	int div[] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32};
	unsigned long cfcr = REG_CPM_CFCR;
	unsigned long plcr = REG_CPM_PLCR1;
	if (plcr & CPM_PLCR1_PLL1EN)
		mclk = __cpm_get_pllout() /
		       div[(cfcr & CPM_CFCR_MFR_MASK) >> CPM_CFCR_MFR_BIT];
	else
		mclk = JZ_EXTAL;
	return mclk;
}

static __inline__ unsigned int __cpm_get_pclk(void)
{
	unsigned int devclk;
	int div[] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32};
	unsigned long cfcr = REG_CPM_CFCR;
	unsigned long plcr = REG_CPM_PLCR1;
	if (plcr & CPM_PLCR1_PLL1EN)
		devclk = __cpm_get_pllout() /
			 div[(cfcr & CPM_CFCR_PFR_MASK) >> CPM_CFCR_PFR_BIT];
	else
		devclk = JZ_EXTAL;
	return devclk;
}

static __inline__ unsigned int __cpm_get_lcdclk(void)
{
	unsigned int lcdclk;
	int div[] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32};
	unsigned long cfcr = REG_CPM_CFCR;
	unsigned long plcr = REG_CPM_PLCR1;
	if (plcr & CPM_PLCR1_PLL1EN)
		lcdclk = __cpm_get_pllout() /
			 div[(cfcr & CPM_CFCR_LFR_MASK) >> CPM_CFCR_LFR_BIT];
	else
		lcdclk = JZ_EXTAL;
	return lcdclk;
}

static __inline__ unsigned int __cpm_get_pixclk(void)
{
	unsigned int pixclk;
	unsigned long cfcr2 = REG_CPM_CFCR2;
	pixclk = __cpm_get_pllout() / (cfcr2 + 1);
	return pixclk;
}

static __inline__ unsigned int __cpm_get_devclk(void)
{
	return JZ_EXTAL;
}

static __inline__ unsigned int __cpm_get_rtcclk(void)
{
	return JZ_EXTAL2;
}

static __inline__ unsigned int __cpm_get_uartclk(void)
{
	return JZ_EXTAL;
}

static __inline__ unsigned int __cpm_get_usbclk(void)
{
	unsigned int usbclk;
	unsigned long cfcr = REG_CPM_CFCR;
	if (cfcr & CPM_CFCR_UCS)
		usbclk = 48000000;
	else
		usbclk = __cpm_get_pllout() /
			(((cfcr &CPM_CFCR_UFR_MASK) >> CPM_CFCR_UFR_BIT) + 1);
	return usbclk;
}

static __inline__ unsigned int __cpm_get_i2sclk(void)
{
	unsigned int i2sclk;
	unsigned long cfcr = REG_CPM_CFCR;
	i2sclk = __cpm_get_pllout() /
		((cfcr & CPM_CFCR_I2S) ? 2: 1);
	return i2sclk;
}

static __inline__ unsigned int __cpm_get_mscclk(void)
{
	if (REG_CPM_CFCR & CPM_CFCR_MSC)
		return 24000000;
	else
		return 16000000;
}

#endif /* __ASM_JZ4730_CLOCK_H__ */
