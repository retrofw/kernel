/*
 * jz4760pcm.h
 * JZ4760 PCM register definition
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 *
 * Author: whxu@ingenic.cn
 */

#ifndef __JZ4760PCM_H__
#define __JZ4760PCM_H__


/*
 * Pulse-code modulation module(PCM) address definition
 */
#define	PCM_BASE        0xb0071000


/*
 * PCM registers offset address definition
 */
#define PCM_PCTL_OFFSET		(0x00)	/* rw, 32, 0x00000000 */
#define PCM_PCFG_OFFSET		(0x04)  /* rw, 32, 0x00000110 */
#define PCM_PDP_OFFSET		(0x08)  /* rw, 32, 0x00000000 */
#define PCM_PINTC_OFFSET	(0x0c)  /* rw, 32, 0x00000000 */
#define PCM_PINTS_OFFSET	(0x10)  /* rw, 32, 0x00000100 */
#define PCM_PDIV_OFFSET		(0x14)  /* rw, 32, 0x00000001 */


/*
 * PCM registers address definition
 */
#define PCM_PCTL	(PCM_BASE + PCM_PCTL_OFFSET)
#define PCM_PCFG	(PCM_BASE + PCM_PCFG_OFFSET)
#define PCM_PDP		(PCM_BASE + PCM_PDP_OFFSET)
#define PCM_PINTC	(PCM_BASE + PCM_PINTC_OFFSET)
#define PCM_PINTS	(PCM_BASE + PCM_PINTS_OFFSET)
#define PCM_PDIV	(PCM_BASE + PCM_PDIV_OFFSET)


/*
 * CPM registers common define
 */

/* PCM controller control register (PCTL) */
#define PCTL_ERDMA	BIT9
#define PCTL_ETDMA	BIT8
#define PCTL_LSMP	BIT7
#define PCTL_ERPL	BIT6
#define PCTL_EREC	BIT5
#define PCTL_FLUSH	BIT4
#define PCTL_RST	BIT3
#define PCTL_CLKEN	BIT1
#define PCTL_PCMEN	BIT0

/* PCM controller configure register (PCFG) */
#define PCFG_ISS_16BIT		BIT12
#define PCFG_OSS_16BIT		BIT11
#define PCFG_IMSBPOS		BIT10
#define PCFG_OMSBPOS		BIT9
#define PCFG_MODE_SLAVE  	BIT0

#define PCFG_SLOT_LSB		13
#define PCFG_SLOT_MASK		BITS_H2L(14, PCFG_SLOT_LSB)
#define PCFG_SLOT(n)		((n) << PCFG_SLOT_LSB)

#define	PCFG_RFTH_LSB		5
#define	PCFG_RFTH_MASK		BITS_H2L(8, PCFG_RFTH_LSB)

#define	PCFG_TFTH_LSB		1
#define	PCFG_TFTH_MASK		BITS_H2L(4, PCFG_TFTH_LSB)

/* PCM controller interrupt control register(PINTC) */
#define PINTC_ETFS	BIT3
#define PINTC_ETUR	BIT2
#define PINTC_ERFS	BIT1
#define PINTC_EROR	BIT0

/* PCM controller interrupt status register(PINTS) */
#define PINTS_RSTS	BIT14
#define PINTS_TFS	BIT8
#define PINTS_TUR	BIT7
#define PINTS_RFS	BIT1
#define PINTS_ROR	BIT0

#define PINTS_TFL_LSB		9
#define PINTS_TFL_MASK		BITS_H2L(13, PINTS_TFL_LSB)

#define PINTS_RFL_LSB		2
#define PINTS_RFL_MASK		BITS_H2L(6, PINTS_RFL_LSB)

/* PCM controller clock division register(PDIV) */
#define PDIV_SYNL_LSB		11
#define PDIV_SYNL_MASK		BITS_H2L(16, PDIV_SYNL_LSB)

#define PDIV_SYNDIV_LSB		6
#define PDIV_SYNDIV_MASK	BITS_H2L(10, PDIV_SYNDIV_LSB)

#define PDIV_CLKDIV_LSB		0
#define PDIV_CLKDIV_MASK	BITS_H2L(5, PDIV_CLKDIV_LSB)


#ifndef __MIPS_ASSEMBLER


#define REG_PCM_PCTL		REG32(PCM_PCTL)
#define REG_PCM_PCFG		REG32(PCM_PCFG)
#define REG_PCM_PDP		    REG32(PCM_PDP)
#define REG_PCM_PINTC		REG32(PCM_PINTC)
#define REG_PCM_PINTS		REG32(PCM_PINTS)
#define REG_PCM_PDIV		REG32(PCM_PDIV)

/*
 * PCM_DIN, PCM_DOUT, PCM_CLK, PCM_SYN
*/
#define __gpio_as_pcm() 			\
do {						\
	unsigned int bits = BITS_H2L(3, 0);	\
	REG_GPIO_PXFUNS(3) = bits;		\
	REG_GPIO_PXSELC(3) = bits;		\
	REG_GPIO_PXTRGC(3) = bits; 		\
	REG_GPIO_PXPES(3)  = bits; 		\
} while (0)

#define __pcm_enable()          (REG_PCM_PCTL |= PCTL_PCMEN)
#define __pcm_disable()         (REG_PCM_PCTL &= ~PCTL_PCMEN)

#define __pcm_clk_enable()      (REG_PCM_PCTL |= PCTL_CLKEN)
#define __pcm_clk_disable()     (REG_PCM_PCTL &= ~PCTL_CLKEN)

#define __pcm_reset()           (REG_PCM_PCTL |= PCTL_RST)
#define __pcm_flush_fifo()	(REG_PCM_PCTL |= PCTL_FLUSH)

#define __pcm_enable_record()		(REG_PCM_PCTL |= PCTL_EREC)
#define __pcm_disable_record()		(REG_PCM_PCTL &= ~PCTL_EREC)
#define __pcm_enable_playback()		(REG_PCM_PCTL |= PCTL_ERPL)
#define __pcm_disable_playback()	(REG_PCM_PCTL &= ~PCTL_ERPL)

#define __pcm_enable_rxfifo()           __pcm_enable_record()
#define __pcm_disable_rxfifo()          __pcm_disable_record()
#define __pcm_enable_txfifo()           __pcm_enable_playback()
#define __pcm_disable_txfifo()          __pcm_disable_playback()

#define __pcm_last_sample()     (REG_PCM_PCTL |= PCTL_LSMP)
#define __pcm_zero_sample()     (REG_PCM_PCTL &= ~PCTL_LSMP)

#define __pcm_enable_transmit_dma()    (REG_PCM_PCTL |= PCTL_ETDMA)
#define __pcm_disable_transmit_dma()   (REG_PCM_PCTL &= ~PCTL_ETDMA)
#define __pcm_enable_receive_dma()     (REG_PCM_PCTL |= PCTL_ERDMA)
#define __pcm_disable_receive_dma()    (REG_PCM_PCTL &= ~PCTL_ERDMA)

#define __pcm_as_slave()         (REG_PCM_PCFG |= PCFG_MODE_SLAVE)
#define __pcm_as_master()        (REG_PCM_PCFG &= ~PCFG_MODE_SLAVE)

#define __pcm_set_transmit_trigger(n) 			\
do {							\
	REG_PCM_PCFG &= ~PCFG_TFTH_MASK;		\
	REG_PCM_PCFG |= ((n) << PCFG_TFTH_LSB);		\
} while(0)

#define __pcm_set_receive_trigger(n) 			\
do {							\
	REG_PCM_PCFG &= ~PCFG_RFTH_MASK;		\
	REG_PCM_PCFG |= ((n) << PCFG_RFTH_LSB);		\
} while(0)

#define __pcm_omsb_same_sync()   (REG_PCM_PCFG &= ~PCFG_OMSBPOS)
#define __pcm_omsb_next_sync()   (REG_PCM_PCFG |= PCFG_OMSBPOS)

#define __pcm_imsb_same_sync()   (REG_PCM_PCFG &= ~PCFG_IMSBPOS)
#define __pcm_imsb_next_sync()   (REG_PCM_PCFG |= PCFG_IMSBPOS)

#define __pcm_set_iss_16()	      (REG_PCM_PCFG |= PCFG_ISS_16BIT)
#define __pcm_set_iss_8() 	      (REG_PCM_PCFG &= ~PCFG_ISS_16BIT)

#define __pcm_set_oss_16()	       (REG_PCM_PCFG |= PCFG_OSS_16BIT)
#define __pcm_set_oss_8()            (REG_PCM_PCFG &= ~PCFG_OSS_16BIT)


#define __pcm_set_valid_slot(n)   (REG_PCM_PCFG = (REG_PCM_PCFG & ~PCFG_SLOT_MASK) | PCFG_SLOT(n))

#define __pcm_write_data(v)	(REG_PCM_PDP = (v))
#define __pcm_read_data()	(REG_PCM_PDP)

#define __pcm_enable_tfs_intr()		(REG_PCM_PINTC |= PINTC_ETFS)
#define __pcm_disable_tfs_intr()	(REG_PCM_PINTC &= ~PINTC_ETFS)

#define __pcm_enable_tur_intr()		(REG_PCM_PINTC |= PINTC_ETUR)
#define __pcm_disable_tur_intr()	(REG_PCM_PINTC &= ~PINTC_ETUR)

#define __pcm_enable_rfs_intr()		(REG_PCM_PINTC |= PINTC_ERFS)
#define __pcm_disable_rfs_intr()	(REG_PCM_PINTC &= ~PINTC_ERFS)

#define __pcm_enable_ror_intr()		(REG_PCM_PINTC |= PINTC_EROR)
#define __pcm_disable_ror_intr()	(REG_PCM_PINTC &= ~PINTC_EROR)

#define __pcm_ints_valid_tx()	(((REG_PCM_PINTS & PINTS_TFL_MASK) >> PINTS_TFL_LSB))
#define __pcm_ints_valid_rx()	(((REG_PCM_PINTS & PINTS_RFL_MASK) >> PINTS_RFL_LSB))

#define __pcm_set_clk_div(n)	\
(REG_PCM_PDIV = (REG_PCM_PDIV & ~PDIV_CLKDIV_MASK) | ((n) << PDIV_CLKDIV_LSB))

#define __pcm_set_clk_rate(sysclk, pcmclk) \
__pcm_set_clk_div(((sysclk) / (pcmclk) - 1))

#define __pcm_set_sync_div(n) \
(REG_PCM_PDIV = (REG_PCM_PDIV & ~PDIV_SYNDIV_MASK) | ((n) << PDIV_SYNDIV_LSB))

#define __pcm_set_sync_rate(pcmclk, sync) \
__pcm_set_sync_div(((pcmclk) / (8 * (sync)) - 1))

#define __pcm_set_sync_len(n) \
(REG_PCM_PDIV = (REG_PCM_PDIV & ~PDIV_SYNL_MASK) | (n << PDIV_SYNL_LSB))

#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4760PCM_H__ */

