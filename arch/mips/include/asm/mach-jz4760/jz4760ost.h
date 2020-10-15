/*
 * jz4760ost.h
 * JZ4760 OST register definition
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 *
 * Author: whxu@ingenic.cn
 */

#ifndef __JZ4760OST_H__
#define __JZ4760OST_H__


/*
 * Operating system timer module(OST) address definition
 */
#define	OST_BASE	0xb0002000


/*
 * OST registers offset address definition
 */
#define OST_OSTDR_OFFSET	(0xe0)  /* rw, 32, 0x???????? */
#define OST_OSTCNT_OFFSET	(0xe8)  /* rw, 32, 0x???????? */
#define OST_OSTCSR_OFFSET	(0xec)  /* rw, 16, 0x0000 */


/*
 * OST registers address definition
 */
#define OST_OSTDR	(OST_BASE + OST_OSTDR_OFFSET)
#define OST_OSTCNT	(OST_BASE + OST_OSTCNT_OFFSET)
#define OST_OSTCSR	(OST_BASE + OST_OSTCSR_OFFSET)


/*
 * OST registers common define
 */

/* Operating system control register(OSTCSR) */
#define OSTCSR_CNT_MD		BIT15
#define OSTCSR_SD		BIT9
#define OSTCSR_EXT_EN		BIT2
#define OSTCSR_RTC_EN		BIT1
#define OSTCSR_PCK_EN		BIT0

#define OSTCSR_PRESCALE_LSB	3
#define OSTCSR_PRESCALE_MASK	BITS_H2L(5, OSTCSR_PRESCALE_LSB)
#define OSTCSR_PRESCALE1	(0x0 << OSTCSR_PRESCALE_LSB)
#define OSTCSR_PRESCALE4	(0x1 << OSTCSR_PRESCALE_LSB)
#define OSTCSR_PRESCALE16	(0x2 << OSTCSR_PRESCALE_LSB)
#define OSTCSR_PRESCALE64	(0x3 << OSTCSR_PRESCALE_LSB)
#define OSTCSR_PRESCALE256	(0x4 << OSTCSR_PRESCALE_LSB)
#define OSTCSR_PRESCALE1024	(0x5 << OSTCSR_PRESCALE_LSB)


#ifndef __MIPS_ASSEMBLER

#define REG_OST_OSTDR		REG32(OST_OSTDR)
#define REG_OST_OSTCNT		REG32(OST_OSTCNT)
#define REG_OST_OSTCSR		REG16(OST_OSTCSR)

#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4760OST_H__ */
