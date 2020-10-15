/*
 * linux/include/asm-mips/mach-jz4760/jz4760nemc.h
 *
 * JZ4760 NEMC register definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ4760NEMC_H__
#define __JZ4760NEMC_H__


#define NEMC_BASE	0xB3410000

#define NEMC_BCR		(NEMC_BASE + 0x0)  /* BCR */

#define NEMC_SMCR1	(NEMC_BASE + 0x14)  /* Static Memory Control Register 1 */
#define NEMC_SMCR2	(NEMC_BASE + 0x18)  /* Static Memory Control Register 2 */
#define NEMC_SMCR3	(NEMC_BASE + 0x1c)  /* Static Memory Control Register 3 */
#define NEMC_SMCR4	(NEMC_BASE + 0x20)  /* Static Memory Control Register 4 */
#define NEMC_SMCR5	(NEMC_BASE + 0x24)  /* Static Memory Control Register 5 */
#define NEMC_SMCR6	(NEMC_BASE + 0x28)  /* Static Memory Control Register 6 */
#define NEMC_SACR1	(NEMC_BASE + 0x34)  /* Static Memory Bank 1 Addr Config Reg */
#define NEMC_SACR2	(NEMC_BASE + 0x38)  /* Static Memory Bank 2 Addr Config Reg */
#define NEMC_SACR3	(NEMC_BASE + 0x3c)  /* Static Memory Bank 3 Addr Config Reg */
#define NEMC_SACR4	(NEMC_BASE + 0x40)  /* Static Memory Bank 4 Addr Config Reg */
#define NEMC_SACR5	(NEMC_BASE + 0x44)  /* Static Memory Bank 5 Addr Config Reg */
#define NEMC_SACR6	(NEMC_BASE + 0x48)  /* Static Memory Bank 6 Addr Config Reg */

#define NEMC_NFCSR	(NEMC_BASE + 0x050) /* NAND Flash Control/Status Register */

#define REG_NEMC_BCR	REG32(NEMC_BCR)
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


#define REG_NEMC_NFCSR	REG32(NEMC_NFCSR)


/* Bus Control Register */
#define NEMC_BCR_BT_SEL_BIT      30
#define NEMC_BCR_BT_SEL_MASK     (0x3 << NEMC_BCR_BT_SEL_BIT)
#define NEMC_BCR_PK_SEL          (1 << 24)
#define NEMC_BCR_BSR_MASK          (1 << 2)  /* Nand and SDRAM Bus Share Select: 0, share; 1, unshare */
  #define NEMC_BCR_BSR_SHARE       (0 << 2)
  #define NEMC_BCR_BSR_UNSHARE     (1 << 2)
#define NEMC_BCR_BRE             (1 << 1)
#define NEMC_BCR_ENDIAN          (1 << 0)

/* Static Memory Control Register */
#define NEMC_SMCR_STRV_BIT	24
#define NEMC_SMCR_STRV_MASK	(0x0f << NEMC_SMCR_STRV_BIT)
#define NEMC_SMCR_TAW_BIT	20
#define NEMC_SMCR_TAW_MASK	(0x0f << NEMC_SMCR_TAW_BIT)
#define NEMC_SMCR_TBP_BIT	16
#define NEMC_SMCR_TBP_MASK	(0x0f << NEMC_SMCR_TBP_BIT)
#define NEMC_SMCR_TAH_BIT	12
#define NEMC_SMCR_TAH_MASK	(0x07 << NEMC_SMCR_TAH_BIT)
#define NEMC_SMCR_TAS_BIT	8
#define NEMC_SMCR_TAS_MASK	(0x07 << NEMC_SMCR_TAS_BIT)
#define NEMC_SMCR_BW_BIT		6
#define NEMC_SMCR_BW_MASK	(0x03 << NEMC_SMCR_BW_BIT)
  #define NEMC_SMCR_BW_8BIT	(0 << NEMC_SMCR_BW_BIT)
  #define NEMC_SMCR_BW_16BIT	(1 << NEMC_SMCR_BW_BIT)
  #define NEMC_SMCR_BW_32BIT	(2 << NEMC_SMCR_BW_BIT)
#define NEMC_SMCR_BCM		(1 << 3)
#define NEMC_SMCR_BL_BIT		1
#define NEMC_SMCR_BL_MASK	(0x03 << NEMC_SMCR_BL_BIT)
  #define NEMC_SMCR_BL_4		(0 << NEMC_SMCR_BL_BIT)
  #define NEMC_SMCR_BL_8		(1 << NEMC_SMCR_BL_BIT)
  #define NEMC_SMCR_BL_16	(2 << NEMC_SMCR_BL_BIT)
  #define NEMC_SMCR_BL_32	(3 << NEMC_SMCR_BL_BIT)
#define NEMC_SMCR_SMT		(1 << 0)

/* Static Memory Bank Addr Config Reg */
#define NEMC_SACR_BASE_BIT	8
#define NEMC_SACR_BASE_MASK	(0xff << NEMC_SACR_BASE_BIT)
#define NEMC_SACR_MASK_BIT	0
#define NEMC_SACR_MASK_MASK	(0xff << NEMC_SACR_MASK_BIT)

/* NAND Flash Control/Status Register */
#define NEMC_NFCSR_NFCE4		(1 << 7) /* NAND Flash Enable */
#define NEMC_NFCSR_NFE4		(1 << 6) /* NAND Flash FCE# Assertion Enable */
#define NEMC_NFCSR_NFCE3		(1 << 5)
#define NEMC_NFCSR_NFE3		(1 << 4)
#define NEMC_NFCSR_NFCE2		(1 << 3)
#define NEMC_NFCSR_NFE2		(1 << 2)
#define NEMC_NFCSR_NFCE1		(1 << 1)
#define NEMC_NFCSR_NFE1		(1 << 0)

#ifndef __MIPS_ASSEMBLER

#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4760NEMC_H__ */

