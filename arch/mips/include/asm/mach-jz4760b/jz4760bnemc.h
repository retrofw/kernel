/*
 * linux/include/asm-mips/mach-jz4760b/jz4760bnemc.h
 *
 * JZ4760B NEMC register definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ4760BNEMC_H__
#define __JZ4760BNEMC_H__


#define NEMC_BASE	0xB3410000

/*************************************************************************
 * NEMC (External Memory Controller for NAND)
 *************************************************************************/

#define NEMC_NFCSR	(NEMC_BASE + 0x050) /* NAND Flash Control/Status Register */
#define NEMC_SMCR1	(NEMC_BASE + 0x14)  /* Static Memory Control Register 1 */
#define NEMC_SMCR2	(NEMC_BASE + 0x18)
#define NEMC_SMCR3	(NEMC_BASE + 0x1c)
#define NEMC_SMCR4	(NEMC_BASE + 0x20)
#define NEMC_SMCR5	(NEMC_BASE + 0x24)
#define NEMC_SMCR6	(NEMC_BASE + 0x28)
#define NEMC_SACR1	(NEMC_BASE + 0x34)
#define NEMC_SACR2	(NEMC_BASE + 0x38)
#define NEMC_SACR3	(NEMC_BASE + 0x3c)
#define NEMC_SACR4	(NEMC_BASE + 0x40)
#define NEMC_SACR5	(NEMC_BASE + 0x44)
#define NEMC_SACR6	(NEMC_BASE + 0x48)

#define REG_NEMC_NFCSR	REG32(NEMC_NFCSR)
#define REG_NEMC_SMCR1	REG32(NEMC_SMCR1)
#define REG_NEMC_SMCR2	REG32(NEMC_SMCR2)
#define REG_NEMC_SMCR3	REG32(NEMC_SMCR3)
#define REG_NEMC_SMCR4	REG32(NEMC_SMCR4)
#define REG_NEMC_SMCR5	REG32(NEMC_SMCR5)
#define REG_NEMC_SMCR6	REG32(NEMC_SMCR6)
#define REG_NEMC_SACR1	REG32(NEMC_SACR1)
#define REG_NEMC_SACR2	REG32(NEMC_SACR2)
#define REG_NEMC_SACR3	REG32(NEMC_SACR3)
#define REG_NEMC_SACR4	REG32(NEMC_SACR4)
#define REG_NEMC_SACR5	REG32(NEMC_SACR5)
#define REG_NEMC_SACR6	REG32(NEMC_SACR6)

#define NEMC_CS1	       0xBA000000	/* read-write area in static bank 1 */
#define NEMC_CS2	       0xB8000000	/* read-write area in static bank 2 */
#define NEMC_CS3	       0xB7000000	/* read-write area in static bank 3 */
#define NEMC_CS4	       0xB6000000	/* read-write area in static bank 4 */
#define NEMC_CS5	       0xB5000000	/* read-write area in static bank 5 */
#define NEMC_CS6	       0xB4000000	/* read-write area in static bank 6 */

// PN(bit 0):0-disable, 1-enable
// PN(bit 1):0-no reset, 1-reset
// (bit 2):Reserved
// BITCNT(bit 3):0-disable, 1-enable
// BITCNT(bit 4):0-calculate, 1's number, 1-calculate 0's number
// BITCNT(bit 5):0-no reset, 1-reset bitcnt
#define NEMC_PNCR 	(NEMC_BASE+0x100)
#define NEMC_PNDR 	(NEMC_BASE+0x104)
#define NEMC_BITCNT	(NEMC_BASE+0x108)

#define REG_NEMC_PNCR REG32(NEMC_PNCR)
#define REG_NEMC_PNDR REG32(NEMC_PNDR)
#define REG_NEMC_BITCNT REG32(NEMC_BITCNT)

//#define REG_NEMC_SMCR	REG32(NEMC_SMCR)

/* Static Memory Control Register */
#define NEMC_SMCR_STRV_BIT	24
#define NEMC_SMCR_STRV_MASK	(0x1f << NEMC_SMCR_STRV_BIT)
#define NEMC_SMCR_TAW_BIT	20
#define NEMC_SMCR_TAW_MASK	(0x0f << NEMC_SMCR_TAW_BIT)
#define NEMC_SMCR_TBP_BIT	16
#define NEMC_SMCR_TBP_MASK	(0x0f << NEMC_SMCR_TBP_BIT)
#define NEMC_SMCR_TAH_BIT	12
#define NEMC_SMCR_TAH_MASK	(0x0f << NEMC_SMCR_TAH_BIT)
#define NEMC_SMCR_TAS_BIT	8
#define NEMC_SMCR_TAS_MASK	(0x0f << NEMC_SMCR_TAS_BIT)
#define NEMC_SMCR_BW_BIT	6
#define NEMC_SMCR_BW_MASK	(0x03 << NEMC_SMCR_BW_BIT)
#define NEMC_SMCR_BW_8BIT	(0 << NEMC_SMCR_BW_BIT)
#define NEMC_SMCR_BW_16BIT	(1 << NEMC_SMCR_BW_BIT)
#define NEMC_SMCR_BL_BIT	1
#define NEMC_SMCR_BL_MASK	(0x03 << NEMC_SMCR_BL_BIT)
#define NEMC_SMCR_BL_4		(0 << NEMC_SMCR_BL_BIT)
#define NEMC_SMCR_BL_8		(1 << NEMC_SMCR_BL_BIT)
#define NEMC_SMCR_BL_16		(2 << NEMC_SMCR_BL_BIT)
#define NEMC_SMCR_BL_32		(3 << NEMC_SMCR_BL_BIT)
#define NEMC_SMCR_SMT		(1 << 0)

/* Static Memory Bank Addr Config Reg */
#define NEMC_SACR_BASE_BIT	8
#define NEMC_SACR_BASE_MASK	(0xff << NEMC_SACR_BASE_BIT)
#define NEMC_SACR_MASK_BIT	0
#define NEMC_SACR_MASK_MASK	(0xff << NEMC_SACR_MASK_BIT)

/* NAND Flash Control/Status Register */
#define NEMC_NFCSR_NFCE4	(1 << 7) /* NAND Flash Enable */
#define NEMC_NFCSR_NFE4		(1 << 6) /* NAND Flash FCE# Assertion Enable */
#define NEMC_NFCSR_NFCE3	(1 << 5)
#define NEMC_NFCSR_NFE3		(1 << 4)
#define NEMC_NFCSR_NFCE2	(1 << 3)
#define NEMC_NFCSR_NFE2		(1 << 2)
#define NEMC_NFCSR_NFCE1	(1 << 1)
#define NEMC_NFCSR_NFE1		(1 << 0)


#ifndef __MIPS_ASSEMBLER

#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4760BNEMC_H__ */

