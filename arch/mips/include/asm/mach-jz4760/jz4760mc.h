/*
 * jz4760mc.h
 * JZ4760 MC register definition
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 *
 * Author: whxu@ingenic.cn
 */

#ifndef __JZ4760MC_H__
#define __JZ4760MC_H__


/*
 * Motion compensation module(MC) address definition
 */
#define	MC_BASE		0xb3250000


/*
 * MC registers offset address definition
 */
#define MC_MCCR_OFFSET		(0x00)	/* rw, 32, 0x???????? */
#define MC_MCSR_OFFSET		(0x04)	/* rw, 32, 0x???????? */
#define MC_MCRBAR_OFFSET	(0x08)	/* rw, 32, 0x???????? */
#define MC_MCT1LFCR_OFFSET	(0x0c)	/* rw, 32, 0x???????? */
#define MC_MCT2LFCR_OFFSET	(0x10)	/* rw, 32, 0x???????? */
#define MC_MCCBAR_OFFSET	(0x14)	/* rw, 32, 0x???????? */
#define MC_MCIIR_OFFSET		(0x18)	/* rw, 32, 0x???????? */
#define MC_MCSIR_OFFSET		(0x1c)	/* rw, 32, 0x???????? */
#define MC_MCT1MFCR_OFFSET	(0x20)	/* rw, 32, 0x???????? */
#define MC_MCT2MFCR_OFFSET	(0x24)	/* rw, 32, 0x???????? */
#define MC_MCFGIR_OFFSET	(0x28)	/* rw, 32, 0x???????? */
#define MC_MCFCIR_OFFSET	(0x2c)	/* rw, 32, 0x???????? */
#define MC_MCRNDTR_OFFSET	(0x40)	/* rw, 32, 0x???????? */

#define MC_MC2CR_OFFSET		(0x8000)	/* rw, 32, 0x???????? */
#define MC_MC2SR_OFFSET		(0x8004)	/* rw, 32, 0x???????? */
#define MC_MC2RBAR_OFFSET	(0x8008)	/* rw, 32, 0x???????? */
#define MC_MC2CBAR_OFFSET	(0x800c)	/* rw, 32, 0x???????? */
#define MC_MC2IIR_OFFSET	(0x8010)	/* rw, 32, 0x???????? */
#define MC_MC2TFCR_OFFSET	(0x8014)	/* rw, 32, 0x???????? */
#define MC_MC2SIR_OFFSET	(0x8018)	/* rw, 32, 0x???????? */
#define MC_MC2FCIR_OFFSET	(0x801c)	/* rw, 32, 0x???????? */
#define MC_MC2RNDTR_OFFSET	(0x8040)	/* rw, 32, 0x???????? */


/*
 * MC registers address definition
 */
#define MC_MCCR		(MC_BASE + MC_MCCR_OFFSET)
#define MC_MCSR		(MC_BASE + MC_MCSR_OFFSET)
#define MC_MCRBAR	(MC_BASE + MC_MCRBAR_OFFSET)
#define MC_MCT1LFCR	(MC_BASE + MC_MCT1LFCR_OFFSET)
#define MC_MCT2LFCR	(MC_BASE + MC_MCT2LFCR_OFFSET)
#define MC_MCCBAR	(MC_BASE + MC_MCCBAR_OFFSET)
#define MC_MCIIR	(MC_BASE + MC_MCIIR_OFFSET)
#define MC_MCSIR	(MC_BASE + MC_MCSIR_OFFSET)
#define MC_MCT1MFCR	(MC_BASE + MC_MCT1MFCR_OFFSET)
#define MC_MCT2MFCR	(MC_BASE + MC_MCT2MFCR_OFFSET)
#define MC_MCFGIR	(MC_BASE + MC_MCFGIR_OFFSET)
#define MC_MCFCIR	(MC_BASE + MC_MCFCIR_OFFSET)
#define MC_MCRNDTR	(MC_BASE + MC_MCRNDTR_OFFSET)

#define MC_MC2CR	(MC_BASE + MC_MC2CR_OFFSET)
#define MC_MC2SR	(MC_BASE + MC_MC2SR_OFFSET)
#define MC_MC2RBAR	(MC_BASE + MC_MC2RBAR_OFFSET)
#define MC_MC2CBAR	(MC_BASE + MC_MC2CBAR_OFFSET)
#define MC_MC2IIR	(MC_BASE + MC_MC2IIR_OFFSET)
#define MC_MC2TFCR	(MC_BASE + MC_MC2TFCR_OFFSET)
#define MC_MC2SIR	(MC_BASE + MC_MC2SIR_OFFSET)
#define MC_MC2FCIR	(MC_BASE + MC_MC2FCIR_OFFSET)
#define MC_MC2RNDTR	(MC_BASE + MC_MC2RNDTR_OFFSET)


/*
 * MC registers common define
 */

/* MC Control Register(MCCR) */
#define MCCR_RETE		BIT16
#define MCCR_DIPE		BIT7
#define MCCR_CKGEN		BIT6
#define MCCR_FDDEN		BIT5
#define MCCR_DINSE		BIT3
#define MCCR_FAE		BIT2
#define MCCR_RST		BIT1
#define MCCR_CHEN		BIT0

#define MCCR_FDDPGN_LSB		8
#define MCCR_FDDPGN_MASK	BITS_H2L(15, MCCR_FDDPGN_LSB)

/* MC Status Register(MCSR) */
#define MCSR_DLEND		BIT1
#define MCSR_BKLEND		BIT0


#ifndef __MIPS_ASSEMBLER


#define REG_MC_MCCR		REG32(REG_MC_MCCR)
#define REG_MC_MCSR             REG32(REG_MC_MCSR)
#define REG_MC_MCRBAR           REG32(REG_MC_MCRBAR)
#define REG_MC_MCT1LFCR         REG32(REG_MC_MCT1LFCR)
#define REG_MC_MCT2LFCR         REG32(REG_MC_MCT2LFCR)
#define REG_MC_MCCBAR           REG32(REG_MC_MCCBAR)
#define REG_MC_MCIIR            REG32(REG_MC_MCIIR)
#define REG_MC_MCSIR            REG32(REG_MC_MCSIR)
#define REG_MC_MCT1MFCR         REG32(REG_MC_MCT1MFCR)
#define REG_MC_MCT2MFCR         REG32(REG_MC_MCT2MFCR)
#define REG_MC_MCFGIR           REG32(REG_MC_MCFGIR)
#define REG_MC_MCFCIR           REG32(REG_MC_MCFCIR)
#define REG_MC_MCRNDTR          REG32(REG_MC_MCRNDTR)

#define REG_MC_MC2CR            REG32(REG_MC_MC2CR)
#define REG_MC_MC2SR            REG32(REG_MC_MC2SR)
#define REG_MC_MC2RBAR          REG32(REG_MC_MC2RBAR)
#define REG_MC_MC2CBAR          REG32(REG_MC_MC2CBAR)
#define REG_MC_MC2IIR           REG32(REG_MC_MC2IIR)
#define REG_MC_MC2TFCR          REG32(REG_MC_MC2TFCR)
#define REG_MC_MC2SIR           REG32(REG_MC_MC2SIR)
#define REG_MC_MC2FCIR          REG32(REG_MC_MC2FCIR)
#define REG_MC_MC2RNDTR         REG32(REG_MC_MC2RNDTR)


#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4760MC_H__ */

