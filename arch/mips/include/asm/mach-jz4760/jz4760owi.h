/*
 * jz4760owi.h
 * JZ4760 OWI register definition
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 *
 * Author: whxu@ingenic.cn
 */

#ifndef __JZ4760OWI_H__
#define __JZ4760OWI_H__


/*
 * One wire bus interface(OWI) address definition
 */
#define	OWI_BASE	0xb0072000


/*
 * OWI registers offset address definition
 */
#define OWI_OWICFG_OFFSET	(0x00)	/* rw, 8, 0x00 */
#define OWI_OWICTL_OFFSET	(0x04)	/* rw, 8, 0x00 */
#define OWI_OWISTS_OFFSET	(0x08)	/* rw, 8, 0x00 */
#define OWI_OWIDAT_OFFSET	(0x0c)	/* rw, 8, 0x00 */
#define OWI_OWIDIV_OFFSET	(0x10)	/* rw, 8, 0x00 */


/*
 * OWI registers address definition
 */
#define OWI_OWICFG	(OWI_BASE + OWI_OWICFG_OFFSET)
#define OWI_OWICTL	(OWI_BASE + OWI_OWICTL_OFFSET)
#define OWI_OWISTS	(OWI_BASE + OWI_OWISTS_OFFSET)
#define OWI_OWIDAT	(OWI_BASE + OWI_OWIDAT_OFFSET)
#define OWI_OWIDIV	(OWI_BASE + OWI_OWIDIV_OFFSET)


/*
 * OWI registers common define
 */

/* OWI configure register(OWICFG) */
#define OWICFG_MODE	BIT7
#define OWICFG_RDDATA	BIT6
#define OWICFG_WRDATA	BIT5
#define OWICFG_RDST	BIT4
#define OWICFG_WR1RD	BIT3
#define OWICFG_WR0	BIT2
#define OWICFG_RST	BIT1
#define OWICFG_ENA	BIT0

/* OWI control register(OWICTL) */
#define OWICTL_EBYTE	BIT2
#define OWICTL_EBIT	BIT1
#define OWICTL_ERST	BIT0

/* OWI status register(OWISTS) */
#define OWISTS_PST		BIT7
#define OWISTS_BYTE_RDY		BIT2
#define OWISTS_BIT_RDY		BIT1
#define OWISTS_PST_RDY		BIT0

/* OWI clock divide register(OWIDIV) */
#define OWIDIV_CLKDIV_LSB	0
#define OWIDIV_CLKDIV_MASK	BITS_H2L(5, OWIDIV_CLKDIV_LSB)


#ifndef __MIPS_ASSEMBLER

/* Basic ops */
#define REG_OWI_OWICFG  REG8(OWI_OWICFG)
#define REG_OWI_OWICTL  REG8(OWI_OWICTL)
#define REG_OWI_OWISTS  REG8(OWI_OWISTS)
#define REG_OWI_OWIDAT  REG8(OWI_OWIDAT)
#define REG_OWI_OWIDIV  REG8(OWI_OWIDIV)


#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4760OWI_H__ */

