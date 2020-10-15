/*
 * linux/include/asm-mips/mach-jz4750l/jz4750lbch.h
 *
 * JZ4750L BCH register definition
 *
 * Copyright (C) 2008-2014 Ingenic Semiconductor Co., Ltd.
 */
#ifndef __JZ4750LBCH_H__
#define __JZ4750LBCH_H__

/* Define the module base addresses */
#define BCH_BASE	0xB30D0000

/*************************************************************************
 * BCH
 *************************************************************************/
#define	BCH_CR         	(BCH_BASE + 0x00) /* BCH Control register */
#define	BCH_CRS       	(BCH_BASE + 0x04) /* BCH Control Set register */
#define	BCH_CRC       	(BCH_BASE + 0x08) /* BCH Control Clear register */
#define	BCH_CNT    	(BCH_BASE + 0x0C) /* BCH ENC/DEC Count register */
#define	BCH_DR     	(BCH_BASE + 0x10) /* BCH data register */
#define	BCH_PAR0    	(BCH_BASE + 0x14) /* BCH Parity 0 register */
#define	BCH_PAR1    	(BCH_BASE + 0x18) /* BCH Parity 1 register */
#define	BCH_PAR2    	(BCH_BASE + 0x1C) /* BCH Parity 2 register */
#define	BCH_PAR3    	(BCH_BASE + 0x20) /* BCH Parity 3 register */
#define	BCH_INTS    	(BCH_BASE + 0x24) /* BCH Interrupt Status register */
#define	BCH_ERR0        (BCH_BASE + 0x28) /* BCH Error Report 0 register */
#define	BCH_ERR1        (BCH_BASE + 0x2C) /* BCH Error Report 1 register */
#define	BCH_ERR2        (BCH_BASE + 0x30) /* BCH Error Report 2 register */
#define	BCH_ERR3        (BCH_BASE + 0x34) /* BCH Error Report 3 register */
#define	BCH_INTE        (BCH_BASE + 0x38) /* BCH Interrupt Enable register */
#define	BCH_INTES       (BCH_BASE + 0x3C) /* BCH Interrupt Set register */
#define	BCH_INTEC       (BCH_BASE + 0x40) /* BCH Interrupt Clear register */

#define	REG_BCH_CR      REG32(BCH_CR)
#define	REG_BCH_CRS     REG32(BCH_CRS)
#define	REG_BCH_CRC     REG32(BCH_CRC)
#define	REG_BCH_CNT     REG32(BCH_CNT)
#define	REG_BCH_DR      REG8(BCH_DR)
#define	REG_BCH_PAR0    REG32(BCH_PAR0)
#define	REG_BCH_PAR1    REG32(BCH_PAR1)
#define	REG_BCH_PAR2    REG32(BCH_PAR2)
#define	REG_BCH_PAR3    REG32(BCH_PAR3)
#define	REG_BCH_INTS    REG32(BCH_INTS)
#define	REG_BCH_ERR0    REG32(BCH_ERR0)
#define	REG_BCH_ERR1    REG32(BCH_ERR1)
#define	REG_BCH_ERR2    REG32(BCH_ERR2)
#define	REG_BCH_ERR3    REG32(BCH_ERR3)
#define	REG_BCH_INTE    REG32(BCH_INTE)
#define	REG_BCH_INTEC   REG32(BCH_INTEC)
#define	REG_BCH_INTES   REG32(BCH_INTES)

/* BCH Control Register*/
#define	BCH_CR_DMAE              (1 << 4)  /* BCH DMA Enable */
#define	BCH_CR_ENCE              (1 << 3)  /* BCH Encoding Select */
#define	BCH_CR_DECE              (0 << 3)  /* BCH Decoding Select */
#define	BCH_CR_BSEL8             (1 << 2)  /* 8 Bit BCH Select */
#define	BCH_CR_BSEL4             (0 << 2)  /* 4 Bit BCH Select */
#define	BCH_CR_BRST              (1 << 1)  /* BCH Reset */
#define	BCH_CR_BCHE              (1 << 0)  /* BCH Enable */

/* BCH Interrupt Status Register */
#define	BCH_INTS_ERRC_BIT        28
#define	BCH_INTS_ERRC_MASK       (0xf << BCH_INTS_ERRC_BIT)
#define	BCH_INTS_ALL0            (1 << 5)
#define	BCH_INTS_ALLf            (1 << 4)
#define	BCH_INTS_DECF            (1 << 3)
#define	BCH_INTS_ENCF            (1 << 2)
#define	BCH_INTS_UNCOR           (1 << 1)
#define	BCH_INTS_ERR             (1 << 0)

/* BCH ENC/DEC Count Register */
#define BCH_CNT_DEC_BIT          16
#define BCH_CNT_DEC_MASK         (0x3ff << BCH_CNT_DEC_BIT)
#define BCH_CNT_ENC_BIT          0
#define BCH_CNT_ENC_MASK         (0x3ff << BCH_CNT_ENC_BIT)

/* BCH Error Report Register */
#define BCH_ERR_INDEX_ODD_BIT    16
#define BCH_ERR_INDEX_ODD_MASK   (0x1fff << BCH_ERR_INDEX_ODD_BIT)
#define BCH_ERR_INDEX_EVEN_BIT   0
#define BCH_ERR_INDEX_EVEN_MASK  (0x1fff << BCH_ERR_INDEX_EVEN_BIT)

/*
 * BCH Operations
 */
#ifndef __MIPS_ASSEMBLER

#define __ecc_encoding_4bit()                                   	\
	do {				   		        	\
		REG_BCH_CRS = BCH_CR_ENCE | BCH_CR_BRST | BCH_CR_BCHE;  \
		REG_BCH_CRC = BCH_CR_BSEL8;				\
	} while(0)
#define __ecc_decoding_4bit()                           	\
	do {                                                    \
		REG_BCH_CRS = BCH_CR_BRST | BCH_CR_BCHE;	\
		REG_BCH_CRC = BCH_CR_ENCE | BCH_CR_BSEL8;	\
	} while(0)
#define __ecc_encoding_8bit()								\
	do {				   		                        	\
		REG_BCH_CRS = BCH_CR_ENCE | BCH_CR_BRST | BCH_CR_BSEL8 | BCH_CR_BCHE;   \
	} while(0)
#define __ecc_decoding_8bit()						\
	do {								\
		REG_BCH_CRS = BCH_CR_BRST | BCH_CR_BSEL8 | BCH_CR_BCHE;	\
		REG_BCH_CRC = BCH_CR_ENCE;				\
	} while(0)
#define __ecc_dma_enable()        ( REG_BCH_CRS = BCH_CR_DMAE )
#define __ecc_dma_disable()       ( REG_BCH_CRC = BCH_CR_DMAE )
#define __ecc_disable()           ( REG_BCH_CRC = BCH_CR_BCHE )
#define __ecc_encode_sync()       while (!(REG_BCH_INTS & BCH_INTS_ENCF))
#define __ecc_decode_sync()       while (!(REG_BCH_INTS & BCH_INTS_DECF))
#define __ecc_cnt_dec(n)						\
	do {								\
		REG_BCH_CNT &= ~(BCH_CNT_DEC_MASK << BCH_CNT_DEC_BIT);	\
		REG_BCH_CNT = (n) << BCH_CNT_DEC_BIT;			\
	} while(0)
#define __ecc_cnt_enc(n)						\
	do {								\
		REG_BCH_CNT &= ~(BCH_CNT_ENC_MASK << BCH_CNT_ENC_BIT);	\
		REG_BCH_CNT = (n) << BCH_CNT_ENC_BIT;			\
	} while(0)

#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4750LBCH_H__ */
