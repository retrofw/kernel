/*
 * linux/include/asm-mips/mach-jz4770/jz4770lcdc.h
 *
 * JZ4770 LCDC register definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ4770LVDS_H__
#define __JZ4770LVDS_H__

#define LVDS_CTRL	(LCD_BASE + 0x3C0)  
#define LVDS_PLL0	(LCD_BASE + 0x3C4)  
#define LVDS_PLL1	(LCD_BASE + 0x3C8)  

#define REG_LVDS_CTRL	REG32(LVDS_CTRL)
#define REG_LVDS_PLL0	REG32(LVDS_PLL0)
#define REG_LVDS_PLL1	REG32(LVDS_PLL1)

/* LVDS Control Register */
#define LVDS_CTRL_MODEL_SEL	BIT31
#define LVDS_CTRL_MODEL_JEIDA	(0 << 31)
#define LVDS_CTRL_MODEL_VESA	(1 << 31)

#define LVDS_CTRL_TX_PDB	(1 << 30)
#define LVDS_CTRL_TX_PDB_CK	(1 << 29)
#define LVDS_CTRL_TX_RSTB	(1 << 18)
#define LVDS_CTRL_TX_CKBIT_PHA  BIT17 
#define LVDS_CTRL_TX_CKBIT_PHA_FALLING	(1 << 17)
#define LVDS_CTRL_TX_CKBIT_PHA_RISING	(0 << 17)

#define LVDS_CTRL_TX_CKBYTE_PHA         BIT16 
#define LVDS_CTRL_TX_CKBYTE_PHA_FALLING	(1 << 16)
#define LVDS_CTRL_TX_CKBYTE_PHA_RISING	(0 << 16)

#define LVDS_CTRL_TX_CKOUT_PHA       (0x7 << 13)
#define LVDS_CTRL_TX_CKOUT_PHA_1     (0x1 << 13)
#define LVDS_CTRL_TX_CKOUT_PHA_2     (0x2 << 13)
#define LVDS_CTRL_TX_CKOUT_PHA_3     (0x3 << 13)
#define LVDS_CTRL_TX_CKOUT_PHA_4     (0x4 << 13)
#define LVDS_CTRL_TX_CKOUT_PHA_5     (0x5 << 13)
#define LVDS_CTRL_TX_CKOUT_PHA_6     (0x6 << 13)
#define LVDS_CTRL_TX_CKOUT_PHA_7     (0x7 << 13)

#define LVDS_CTRL_TX_CKOUT            BIT12
#define LVDS_CTRL_TX_CKOUT_7x         (0x1 << 12)
#define LVDS_CTRL_TX_CKOUT_1x         (0x0 << 12)
#define LVDS_CTRL_TX_OUT_SEL          BIT11
#define LVDS_CTRL_TX_OUT_CMOS_RGB     (0x1 << 11)
#define LVDS_CTRL_TX_OUT_LVDS         (0x0 << 11)
#define LVDS_CTRL_TX_DLY_SEL_BIT      8
#define LVDS_CTRL_TX_DLY_SEL          (0x7 << 8)
#define LVDS_CTRL_TX_AMP_ADJ          BIT7
#define LVDS_CTRL_TX_LVDS             BIT6
#define LVDS_CTRL_TX_CR               (0x7 << 3)
#define LVDS_CTRL_TX_CR_CK            BIT2

#define OUTPUT_SWING_150mV    (0 << 3)
#define OUTPUT_SWING_200mV    (1 << 3)
#define OUTPUT_SWING_250mV    (2 << 3)
#define OUTPUT_SWING_300mV    (3 << 3)
#define OUTPUT_SWING_350mV    (4 << 3)
#define OUTPUT_SWING_400mV    (5 << 3)
#define OUTPUT_SWING_500mV    (6 << 3)
#define OUTPUT_SWING_600mV    (7 << 3)
#define OUTPUT_SWING_650mV    (LVDS_CTRL_TX_CR_CK | 0 << 3)
#define OUTPUT_SWING_700mV    (LVDS_CTRL_TX_CR_CK | 1 << 3)
#define OUTPUT_SWING_750mV    (LVDS_CTRL_TX_CR_CK | 2 << 3)
#define OUTPUT_SWING_800mV    (LVDS_CTRL_TX_CR_CK | 3 << 3)
#define OUTPUT_SWING_850mV    (LVDS_CTRL_TX_CR_CK | 4 << 3)
#define OUTPUT_SWING_900mV    (LVDS_CTRL_TX_CR_CK | 5 << 3)
#define OUTPUT_SWING_1000mV   (LVDS_CTRL_TX_CR_CK | 6 << 3)
#define OUTPUT_SWING_1100mV   (LVDS_CTRL_TX_CR_CK | 7 << 3)

#define LVDS_CTRL_TX_OD_S     BIT1
#define LVDS_CTRL_TX_OD_EN    BIT0

#define TX_OUTPUT_CMOS             (LVDS_CTRL_TX_OD_S | LVDS_CTRL_TX_OD_EN)
#define TX_OUTPUT_LVDS_DATA        (LVDS_CTRL_TX_OD_EN)
#define TX_OUTPUT_O                (LVDS_CTRL_TX_OD_S)
#define TX_OUTPUT_HIZ              (0)

/* LVDS PLL0 Register */
#define LVDS_PLL0_PLL_LOCK		BIT31
#define LVDS_PLL0_PLL_DIS		BIT30
#define LVDS_PLL0_BG_PWD		BIT29
#define LVDS_PLL0_PLL_SSC_EN		BIT27
#define LVDS_PLL0_PLL_SSC_MODE		BIT26
#define LVDS_PLL0_PLL_TEST		BIT25
#define LVDS_PLL0_PLL_POST_DIVA		(0x3 << 21)
#define LVDS_PLL0_PLL_POST_DIVB		(0x3f << 16)
#define LVDS_PLL0_PLL_PLLN_BIT          8
#define LVDS_PLL0_PLL_PLLN		(0x7f << 8)
#define LVDS_PLL0_PLL_TEST_DIV		(0x2 << 6)
#define LVDS_PLL0_PLL_IN_BYPASS		BIT5
#define LVDS_PLL0_PLL_INDIV_BIT         0		
#define LVDS_PLL0_PLL_INDIV		(0x1f << 0)


/* LVDS PLL1 Register */
#define LVDS_PLL1_PLL_ICP_SEL_BIT       29 
#define LVDS_PLL1_PLL_ICP_SEL		(0x7 << LVDS_PLL1_PLL_ICP_SEL_BIT)
#define LVDS_PLL1_PLL_KVCO_BIT          26 
#define LVDS_PLL1_PLL_KVCO		(0x3 << LVDS_PLL1_PLL_KVCO_BIT)
#define LVDS_PLL1_PLL_IVCO_SEL_BIT      24 
#define LVDS_PLL1_PLL_IVCO_SEL		(0x3 << LVDS_PLL1_PLL_IVCO_SEL_BIT)
#define LVDS_PLL1_PLL_SSCN_BIT          17 
#define LVDS_PLL1_PLL_SSCN		(0x7f << LVDS_PLL1_PLL_SSCN_BIT)
#define LVDS_PLL1_PLL_COUNT_BIT         4
#define LVDS_PLL1_PLL_COUNT		(0x1fff << LVDS_PLL1_PLL_COUNT_BIT)
#define LVDS_PLL1_PLL_GAIN_BIT          0
#define LVDS_PLL1_PLL_GAIN		(0xf << LVDS_PLL1_PLL_GAIN_BIT)

#endif /* __JZ4770LVDS_H__ */

