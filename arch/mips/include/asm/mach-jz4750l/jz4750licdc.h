/*
 * linux/include/asm-mips/mach-jz4750l/jz4750licdc.h
 *
 * JZ4750L ICDC register definition
 *
 * Copyright (C) 2008-2014 Ingenic Semiconductor Co., Ltd.
 */
#ifndef __JZ4750LICDC_H__
#define __JZ4750LICDC_H__

/* Define the module base addresses */
#define	ICDC_BASE	0xB0020000

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

/*
 * ICDC Operations
 */
#ifndef __MIPS_ASSEMBLER

#define __i2s_internal_codec()         __aic_internal_codec()
#define __i2s_external_codec()         __aic_external_codec()

#define __icdc_clk_ready()             ( REG_ICDC_CKCFG & ICDC_CKCFG_CKRDY )
#define __icdc_sel_adc()               ( REG_ICDC_CKCFG |= ICDC_CKCFG_SELAD )
#define __icdc_sel_dac()               ( REG_ICDC_CKCFG &= ~ICDC_CKCFG_SELAD )

#define __icdc_set_rgwr()              ( REG_ICDC_RGADW |= ICDC_RGADW_RGWR )
#define __icdc_clear_rgwr()            ( REG_ICDC_RGADW &= ~ICDC_RGADW_RGWR )
#define __icdc_rgwr_ready()            ( REG_ICDC_RGADW & ICDC_RGADW_RGWR )

#define __icdc_set_addr(n)					\
	do {          						\
		REG_ICDC_RGADW &= ~ICDC_RGADW_RGADDR_MASK;	\
		REG_ICDC_RGADW |= (n) << ICDC_RGADW_RGADDR_BIT;	\
	} while(0)

#define __icdc_set_cmd(n)					\
	do {          						\
		REG_ICDC_RGADW &= ~ICDC_RGADW_RGDIN_MASK;	\
		REG_ICDC_RGADW |= (n) << ICDC_RGADW_RGDIN_BIT;	\
	} while(0)

#define __icdc_irq_pending()            ( REG_ICDC_RGDATA & ICDC_RGDATA_IRQ )
#define __icdc_get_value()              ( REG_ICDC_RGDATA & ICDC_RGDATA_RGDOUT_MASK )

#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4750LICDC_H__ */
