/*
 *  linux/include/asm-mips/mach-jz4810/clock.h
 *
 *  JZ4810 clocks definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4810_CLOCK_H__
#define __ASM_JZ4810_CLOCK_H__

#ifndef JZ_EXTAL
#define JZ_EXTAL		24000000   /* 3.6864 MHz */
#endif
#ifndef JZ_EXTAL2
#define JZ_EXTAL2		32768     /* 32.768 KHz */
#endif

/*
 * JZ4810 clocks structure
 */
typedef struct {
	unsigned int cclk;	/* CPU clock				*/
	unsigned int hclk;	/* System bus clock: AHB0,AHB1		*/
	unsigned int h1clk;	/* For compatible, the same as h1clk	*/
	unsigned int h2clk;	/* System bus clock: AHB2		*/
	unsigned int pclk;	/* Peripheral bus clock			*/
	unsigned int mclk;	/* EMC or DDR controller clock		*/
	unsigned int sclk;	/* NEMC controller clock		*/
	unsigned int cko;	/* SDRAM or DDR clock			*/
	unsigned int pixclk;	/* LCD pixel clock			*/
	unsigned int tveclk;	/* TV encoder 27M  clock		*/
	unsigned int cimmclk;	/* Clock output from CIM module		*/
	unsigned int cimpclk;	/* Clock input to CIM module		*/
	unsigned int gpuclk;	/* GPU clock				*/
	unsigned int gpsclk;	/* GPS clock				*/
	unsigned int i2sclk;	/* I2S codec clock			*/
	unsigned int bitclk;	/* AC97 bit clock			*/
	unsigned int pcmclk;	/* PCM clock				*/
	unsigned int mscclk;	/* MSC clock				*/
	unsigned int ssiclk;	/* SSI clock				*/
	unsigned int tssiclk;	/* TSSI clock				*/
	unsigned int otgclk;	/* USB OTG clock			*/
	unsigned int uhcclk;	/* USB UHCI clock			*/
	unsigned int extalclk;	/* EXTAL clock for
				   UART,I2C,TCU,USB2.0-PHY,AUDIO CODEC	*/
	unsigned int rtcclk;	/* RTC clock for CPM,INTC,RTC,TCU,WDT	*/
} jz_clocks_t;

extern jz_clocks_t jz_clocks;



/* PLL output frequency */
static __inline__ unsigned int __cpm_get_pllout(void)
{
#if defined(CONFIG_FPGA)
	return JZ_EXTAL*CFG_DIV;
#else
	unsigned long m, n, no, pllout;
	unsigned long cppcr = REG_CPM_CPPCR;
	unsigned long od[4] = {1, 2, 4, 8};
	if ((cppcr & CPM_CPPCR_PLLEN) && !(cppcr & CPM_CPPCR_PLLBP)) {
		m = __cpm_get_pllm() << 1;
		n = __cpm_get_plln();
		no = od[__cpm_get_pllod()];
		pllout = ((JZ_EXTAL) / (n * no)) * m;
	} else
		pllout = JZ_EXTAL;
	return pllout;
#endif
}

/* PLL output frequency / 2 */
static __inline__ unsigned int __cpm_get_pllout2(void)
{
#if defined(CONFIG_FPGA)
	return __cpm_get_pllout();
#else
	if (REG_CPM_CPCCR & CPM_CPCCR_PCS)
		return __cpm_get_pllout();
	else
		return __cpm_get_pllout()/2;
#endif
}

/* CPU core clock */
static __inline__ unsigned int __cpm_get_cclk(void)
{

#if defined(CONFIG_FGPA)
	return JZ_EXTAL * CFG_DIV;
#else
	int div[] = {1, 2, 3, 4, 6, 8};
	return __cpm_get_pllout() / div[__cpm_get_cdiv()];
#endif
}

/* AHB0, AHB1 clock */
static __inline__ unsigned int __cpm_get_hclk(void)
{
#if defined(CONFIG_FPGA)
	return JZ_EXTAL;
#else
	int div[] = {1, 2, 3, 4, 6, 8};

	return __cpm_get_pllout() / div[__cpm_get_hdiv()];
#endif

}

#define __cpm_get_h0clk(void)	__cpm_get_hclk()
#define __cpm_get_h1clk(void)	__cpm_get_hclk()


/* AHB2 clock */
static __inline__ unsigned int __cpm_get_h2clk(void)
{
#if defined(CONFIG_FPGA)
	return JZ_EXTAL;
#else
	int div[] = {1, 2, 3, 4, 6, 8};

	return __cpm_get_pllout() / div[__cpm_get_h2div()];
#endif

}

/* Memory bus clock */
static __inline__ unsigned int __cpm_get_mclk(void)
{
#if defined(CONFIG_FPGA)
	return JZ_EXTAL/CFG_DIV;
#else
	int div[] = {1, 2, 3, 4, 6, 8};

	return __cpm_get_pllout() / div[__cpm_get_mdiv()];
#endif
}

/* APB peripheral bus clock */
static __inline__ unsigned int __cpm_get_pclk(void)
{
#if defined(CONFIG_FPGA)
	return JZ_EXTAL/CFG_DIV;
#else
	int div[] = {1, 2, 3, 4, 6, 8};

	return __cpm_get_pllout() / div[__cpm_get_pdiv()];
#endif
}

/* LCD pixel clock */
static __inline__ unsigned int __cpm_get_pixclk(void)
{
	return __cpm_get_pllout2() / (__cpm_get_pixdiv() + 1);
}

/* I2S clock */
static __inline__ unsigned int __cpm_get_i2sclk(void)
{
	if (REG_CPM_I2SCDR & CPM_I2SCDR_I2CS) {
		if (REG_CPM_CPCCR & CPM_CPCCR_PCS) {
			return __cpm_get_pllout() / (__cpm_get_i2sdiv() + 1);
		} else {
			return __cpm_get_pllout2() / (__cpm_get_i2sdiv() + 1);
		}
	} else {
		return JZ_EXTAL;
	}
}

/* USB OTG clock */
static __inline__ unsigned int __cpm_get_otgclk(void)
{
	if (REG_CPM_USBCDR & CPM_USBCDR_UCS) {
		if (REG_CPM_CPCCR & CPM_CPCCR_PCS) {
			return __cpm_get_pllout() / (__cpm_get_otgdiv() + 1);
		} else {
			return __cpm_get_pllout2() / (__cpm_get_otgdiv() + 1);
		}
	} else {
		return JZ_EXTAL;
	}
}

/* MSC clock */
static __inline__ unsigned int __cpm_get_mscclk(int n)
{
	if (REG_CPM_MSCCDR & CPM_MSCCDR_MCS) {
		if (REG_CPM_CPCCR & CPM_CPCCR_PCS) {
			return __cpm_get_pllout() / (__cpm_get_mscdiv() + 1);
		} else {
			return __cpm_get_pllout2() / (__cpm_get_mscdiv() + 1);
		}
	} else {
		return JZ_EXTAL;
	}
}

/* EXTAL clock */
static __inline__ unsigned int __cpm_get_extalclk0(void)
{
	return JZ_EXTAL;
}

/* EXTAL clock for UART,I2C,SSI,SADC,USB-PHY */
static __inline__ unsigned int __cpm_get_extalclk(void)
{
#if defined(CONFIG_FPGA)
	return __cpm_get_extalclk0() / CFG_DIV;
#else
	if (REG_CPM_CPCCR & CPM_CPCCR_ECS)
		return __cpm_get_extalclk0() / 2;
	else
		return __cpm_get_extalclk0();
#endif

}

/* RTC clock for CPM,INTC,RTC,TCU,WDT */
static __inline__ unsigned int __cpm_get_rtcclk(void)
{
	return JZ_EXTAL2;
}

/*
 * Output 24MHz for SD and 16MHz for MMC.
 * @n: the index of MMC/SD controller
 */
static inline void __cpm_select_msc_clk(int n, int sd)
{
	unsigned int pllout2 = __cpm_get_pllout2();
	unsigned int div = 0;

	if (sd) {
		div = pllout2 / 24000000;
	}
	else {
		div = pllout2 / 16000000;
	}

	REG_CPM_MSCCDR = div - 1;
	REG_CPM_CPCCR |= CPM_CPCCR_CE;
}

/*
 * Output 48MHz for high speed card.
 */
static inline void __cpm_select_msc_clk_high(int n, int sd)
{
	unsigned int pllout2 = __cpm_get_pllout2();
	unsigned int div = 0;

	div = pllout2 / 48000000;

	REG_CPM_MSCCDR = div - 1;
	REG_CPM_CPCCR |= CPM_CPCCR_CE;
}

#endif /* __ASM_JZ4810_CLOCK_H__ */
