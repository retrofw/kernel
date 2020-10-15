/*
 * linux/include/asm-mips/mach-jz4760/regs.h
 *
 * JZ4760 register definition.
 *
 * Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __JZ4760_REGS_H__
#define __JZ4760_REGS_H__


/*
 * Define the module base addresses
 */
/* AHB0 BUS Devices Base */
#define HARB0_BASE	0xB3000000
/* AHB1 BUS Devices Base */
#define HARB1_BASE	0xB3200000
#define	DMAGP0_BASE	0xB3210000
#define	DMAGP1_BASE	0xB3220000
#define	DMAGP2_BASE	0xB3230000
#define	DEBLK_BASE	0xB3270000
#define	IDCT_BASE	0xB3280000
#define	CABAC_BASE	0xB3290000
#define	TCSM0_BASE	0xB32B0000
#define	TCSM1_BASE	0xB32C0000
#define	SRAM_BASE	0xB32D0000
/* AHB2 BUS Devices Base */
#define HARB2_BASE	0xB3400000
#define UHC_BASE	0xB3430000
#define GPS_BASE	0xB3480000
#define ETHC_BASE	0xB34B0000
/* APB BUS Devices Base */
#define	PS2_BASE	0xB0060000
#define	SADC_BASE	0xB0070000


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

#define IRQ_UDC		27

#endif /* __JZ4760B_REGS_H__ */
