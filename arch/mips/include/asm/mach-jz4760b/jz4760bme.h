/*
 * jz4760bme.h
 * JZ4760B ME register definition
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 *
 * Author: whxu@ingenic.cn
 */

#ifndef __JZ4760BME_H__
#define __JZ4760BME_H__


/*
 * Motion estimation module(ME) address definition
 */
#define	ME_BASE		0xb3260000


/*
 * ME registers offset address definition
 */
#define ME_MECR_OFFSET		(0x00)  /* rw, 32, 0x???????0 */
#define ME_MERBAR_OFFSET	(0x04)  /* rw, 32, 0x???????? */
#define ME_MECBAR_OFFSET	(0x08)  /* rw, 32, 0x???????? */
#define ME_MEDAR_OFFSET		(0x0c)  /* rw, 32, 0x???????? */
#define ME_MERFSR_OFFSET	(0x10)  /* rw, 32, 0x???????? */
#define ME_MECFSR_OFFSET	(0x14)  /* rw, 32, 0x???????? */
#define ME_MEDFSR_OFFSET	(0x18)  /* rw, 32, 0x???????? */
#define ME_MESR_OFFSET		(0x1c)  /* rw, 32, 0x???????? */
#define ME_MEMR_OFFSET		(0x20)  /* rw, 32, 0x???????? */
#define ME_MEFR_OFFSET		(0x24)  /* rw, 32, 0x???????? */


/*
 * ME registers address definition
 */
#define ME_MECR		(ME_BASE + ME_MECR_OFFSET)
#define ME_MERBAR	(ME_BASE + ME_MERBAR_OFFSET)
#define ME_MECBAR       (ME_BASE + ME_MECBAR_OFFSET)
#define ME_MEDAR        (ME_BASE + ME_MEDAR_OFFSET)
#define ME_MERFSR       (ME_BASE + ME_MERFSR_OFFSET)
#define ME_MECFSR       (ME_BASE + ME_MECFSR_OFFSET)
#define ME_MEDFSR       (ME_BASE + ME_MEDFSR_OFFSET)
#define ME_MESR         (ME_BASE + ME_MESR_OFFSET)
#define ME_MEMR         (ME_BASE + ME_MEMR_OFFSET)
#define ME_MEFR         (ME_BASE + ME_MEFR_OFFSET)


/*
 * ME registers common define
 */

/* ME control register(MECR) */
#define MECR_FLUSH		BIT2
#define MECR_RESET		BIT1
#define MECR_ENABLE		BIT0

/* ME settings register(MESR) */
#define MESR_GATE_LSB		16
#define MESR_GATE_MASK		BITS_H2L(31, MESR_GATE_LSB)

#define MESR_NUM_LSB		0
#define MESR_NUM_MASK		BITS_H2L(5, MESR_NUM_LSB)

/* ME MVD register(MEMR) */
#define MEMR_MVDY_LSB		16
#define MESR_MVDY_MASK		BITS_H2L(31, MEMR_MVDY_LSB)

#define MEMR_MVDX_LSB		0
#define MESR_MVDX_MASK		BITS_H2L(15, MEMR_MVDX_LSB)

/* ME flag register(MEFR) */
#define MEFR_INTRA		BIT1
#define MEFR_COMPLETED		BIT0


#ifndef __MIPS_ASSEMBLER

#define REG_ME_MECR	REG32(ME_MECR)
#define REG_ME_MERBAR	REG32(ME_MERBAR)
#define REG_ME_MECBAR	REG32(ME_MECBAR)
#define REG_ME_MEDAR	REG32(ME_MEDAR)
#define REG_ME_MERFSR	REG32(ME_MERFSR)
#define REG_ME_MECFSR	REG32(ME_MECFSR)
#define REG_ME_MEDFSR	REG32(ME_MEDFSR)
#define REG_ME_MESR	REG32(ME_MESR)
#define REG_ME_MEMR	REG32(ME_MEMR)
#define REG_ME_MEFR	REG32(ME_MEFR)

#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4760BME_H__ */

