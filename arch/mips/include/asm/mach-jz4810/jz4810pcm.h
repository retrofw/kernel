/*
 * linux/include/asm-mips/mach-jz4810/jz4810pcm.h
 *
 * JZ4810 PCM register definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ4810PCM_H__
#define __JZ4810PCM_H__


#define	PCM_BASE        0xB0071000

/*************************************************************************
 * PCM Controller
 *************************************************************************/

#define PCM_CTL                 (PCM_BASE + 0x000)
#define PCM_CFG                 (PCM_BASE + 0x004)
#define PCM_DP                  (PCM_BASE + 0x008)
#define PCM_INTC                (PCM_BASE + 0x00c)
#define PCM_INTS                (PCM_BASE + 0x010)
#define PCM_DIV                 (PCM_BASE + 0x014)

#define REG_PCM_CTL             REG32(PCM_CTL)
#define REG_PCM_CFG             REG32(PCM_CFG)
#define REG_PCM_DP              REG32(PCM_DP)
#define REG_PCM_INTC            REG32(PCM_INTC)
#define REG_PCM_INTS            REG32(PCM_INTS)
#define REG_PCM_DIV             REG32(PCM_DIV)

/* PCM Controller control Register (PCM_CTL) */

#define PCM_CTL_ERDMA		(1 << 9)  /* Enable Receive DMA */
#define PCM_CTL_ETDMA           (1 << 8)  /* Enable Transmit DMA */
#define PCM_CTL_LSMP		(1 << 7)  /* Play Zero sample or last sample */
#define PCM_CTL_ERPL            (1 << 6)  /* Enable Playing Back Function */
#define PCM_CTL_EREC            (1 << 5)  /* Enable Recording Function */
#define PCM_CTL_FLUSH           (1 << 4)  /* FIFO flush */
#define PCM_CTL_RST             (1 << 3)  /* Reset PCM */
#define PCM_CTL_CLKEN           (1 << 1)  /* Enable the clock division logic */
#define PCM_CTL_PCMEN           (1 << 0)  /* Enable PCM module */

/* PCM Controller configure Register (PCM_CFG) */

#define PCM_CFG_SLOT_BIT        13
#define PCM_CFG_SLOT_MASK       (0x3 << PCM_CFG_SLOT_BIT)
  #define PCM_CFG_SLOT_0	  (0 << PCM_CFG_SLOT_BIT) /* Slot is 0 */
  #define PCM_CFG_SLOT_1	  (1 << PCM_CFG_SLOT_BIT) /* Slot is 1 */
  #define PCM_CFG_SLOT_2	  (2 << PCM_CFG_SLOT_BIT) /* Slot is 2 */
  #define PCM_CFG_SLOT_3	  (3 << PCM_CFG_SLOT_BIT) /* Slot is 3 */
#define PCM_CFG_ISS_BIT         12
#define PCM_CFG_ISS_MASK        (0x1 << PCM_CFG_ISS_BIT)
  #define PCM_CFG_ISS_8           (0 << PCM_CFG_ISS_BIT)
  #define PCM_CFG_ISS_16          (1 << PCM_CFG_ISS_BIT)
#define PCM_CFG_OSS_BIT         11
#define PCM_CFG_OSS_MASK        (0x1 << PCM_CFG_OSS_BIT)
  #define PCM_CFG_OSS_8           (0 << PCM_CFG_OSS_BIT)
  #define PCM_CFG_OSS_16          (1 << PCM_CFG_OSS_BIT)
#define PCM_CFG_IMSBPOS         (1 << 10)
#define PCM_CFG_OMSBPOS         (1 << 9)
#define	PCM_CFG_RFTH_BIT	5        /* Receive FIFO Threshold */
#define	PCM_CFG_RFTH_MASK	(0xf << PCM_CFG_RFTH_BIT)
#define	PCM_CFG_TFTH_BIT	1         /* Transmit FIFO Threshold */
#define	PCM_CFG_TFTH_MASK	(0xf << PCM_CFG_TFTH_BIT)
#define PCM_CFG_MODE            (0x0 << 0)

/* PCM Controller interrupt control Register (PCM_INTC) */

#define PCM_INTC_ETFS           (1 << 3)
#define PCM_INTC_ETUR           (1 << 2)
#define PCM_INTC_ERFS           (1 << 1)
#define PCM_INTC_EROR           (1 << 0)

/* PCM Controller interrupt status Register (PCM_INTS) */

#define PCM_INTS_RSTS		(1 << 14) /* Reset or flush has not complete */
#define PCM_INTS_TFL_BIT        9
#define PCM_INTS_TFL_MASK       (0x1f << PCM_INTS_TFL_BIT)
#define PCM_INTS_TFS		(1 << 8) /* Tranmit FIFO Service Request */
#define PCM_INTS_TUR		(1 << 7) /* Transmit FIFO Under Run */
#define PCM_INTS_RFL_BIT        2
#define PCM_INTS_RFL_MASK       (0x1f << PCM_INTS_RFL_BIT)
#define PCM_INTS_RFS		(1 << 1) /* Receive FIFO Service Request */
#define PCM_INTS_ROR		(1 << 0) /* Receive FIFO Over Run */

/* PCM Controller clock division Register (PCM_DIV) */
#define PCM_DIV_SYNL_BIT        11
#define PCM_DIV_SYNL_MASK       (0x3f << PCM_DIV_SYNL_BIT)
#define PCM_DIV_SYNDIV_BIT      6
#define PCM_DIV_SYNDIV_MASK     (0x1f << PCM_DIV_SYNDIV_BIT)
#define PCM_DIV_CLKDIV_BIT      0
#define PCM_DIV_CLKDIV_MASK     (0x3f << PCM_DIV_CLKDIV_BIT)


#ifndef __MIPS_ASSEMBLER

/*************************************************************************
 * PCM Controller operation
 *************************************************************************/

#define __pcm_enable()          ( REG_PCM_CTL |= PCM_CTL_PCMEN )
#define __pcm_disable()         ( REG_PCM_CTL &= ~PCM_CTL_PCMEN )

#define __pcm_clk_enable()      ( REG_PCM_CTL |= PCM_CTL_CLKEN )
#define __pcm_clk_disable()     ( REG_PCM_CTL &= ~PCM_CTL_CLKEN )

#define __pcm_reset()           ( REG_PCM_CTL |= PCM_CTL_RST )
#define __pcm_flush_fifo()	( REG_PCM_CTL |= PCM_CTL_FLUSH )

#define __pcm_enable_record()		( REG_PCM_CTL |= PCM_CTL_EREC )
#define __pcm_disable_record()		( REG_PCM_CTL &= ~PCM_CTL_EREC )
#define __pcm_enable_playback()		( REG_PCM_CTL |= PCM_CTL_ERPL )
#define __pcm_disable_playback()	( REG_PCM_CTL &= ~PCM_CTL_ERPL )

#define __pcm_enable_rxfifo()           __pcm_enable_record()
#define __pcm_disable_rxfifo()          __pcm_disable_record()
#define __pcm_enable_txfifo()           __pcm_enable_playback()
#define __pcm_disable_txfifo()          __pcm_disable_playback()

#define __pcm_last_sample()     ( REG_PCM_CTL |= PCM_CTL_LSMP )
#define __pcm_zero_sample()     ( REG_PCM_CTL &= ~PCM_CTL_LSMP )

#define __pcm_enable_transmit_dma()    ( REG_PCM_CTL |= PCM_CTL_ETDMA )
#define __pcm_disable_transmit_dma()   ( REG_PCM_CTL &= ~PCM_CTL_ETDMA )
#define __pcm_enable_receive_dma()     ( REG_PCM_CTL |= PCM_CTL_ERDMA )
#define __pcm_disable_receive_dma()    ( REG_PCM_CTL &= ~PCM_CTL_ERDMA )

#define __pcm_as_master()     ( REG_PCM_CFG &= PCM_CFG_MODE )
#define __pcm_as_slave()      ( REG_PCM_CFG |= ~PCM_CFG_MODE )

#define __pcm_set_transmit_trigger(n) 			\
do {							\
	REG_PCM_CFG &= ~PCM_CFG_TFTH_MASK;		\
	REG_PCM_CFG |= ((n) << PCM_CFG_TFTH_BIT);	\
} while(0)

#define __pcm_set_receive_trigger(n) 			\
do {							\
	REG_PCM_CFG &= ~PCM_CFG_RFTH_MASK;		\
	REG_PCM_CFG |= ((n) << PCM_CFG_RFTH_BIT);	\
} while(0)

#define __pcm_omsb_same_sync()   ( REG_PCM_CFG &= ~PCM_CFG_OMSBPOS )
#define __pcm_omsb_next_sync()   ( REG_PCM_CFG |= PCM_CFG_OMSBPOS )

#define __pcm_imsb_same_sync()   ( REG_PCM_CFG &= ~PCM_CFG_IMSBPOS )
#define __pcm_imsb_next_sync()   ( REG_PCM_CFG |= PCM_CFG_IMSBPOS )

/* set input sample size 8 or 16*/
#define __pcm_set_iss(n) \
( REG_PCM_CFG = (REG_PCM_CFG & ~PCM_CFG_ISS_MASK) | PCM_CFG_ISS_##n )
/* set output sample size 8 or 16*/
#define __pcm_set_oss(n) \
( REG_PCM_CFG = (REG_PCM_CFG & ~PCM_CFG_OSS_MASK) | PCM_CFG_OSS_##n )

#define __pcm_set_valid_slot(n) \
( REG_PCM_CFG = (REG_PCM_CFG & ~PCM_CFG_SLOT_MASK) | PCM_CFG_SLOT_##n )

#define __pcm_write_data(v)           ( REG_PCM_DP = (v) )
#define __pcm_read_data()             ( REG_PCM_DP )

#define __pcm_enable_tfs_intr()       ( REG_PCM_INTC |= PCM_INTC_ETFS )
#define __pcm_disable_tfs_intr()      ( REG_PCM_INTC &= ~PCM_INTC_ETFS )

#define __pcm_enable_tur_intr()       ( REG_PCM_INTC |= PCM_INTC_ETUR )
#define __pcm_disable_tur_intr()      ( REG_PCM_INTC &= ~PCM_INTC_ETUR )

#define __pcm_enable_rfs_intr()       ( REG_PCM_INTC |= PCM_INTC_ERFS )
#define __pcm_disable_rfs_intr()      ( REG_PCM_INTC &= ~PCM_INTC_ERFS )

#define __pcm_enable_ror_intr()       ( REG_PCM_INTC |= PCM_INTC_EROR )
#define __pcm_disable_ror_intr()      ( REG_PCM_INTC &= ~PCM_INTC_EROR )

#define __pcm_ints_valid_tx() \
( ((REG_PCM_INTS & PCM_INTS_TFL_MASK) >> PCM_INTS_TFL_BIT) )
#define __pcm_ints_valid_rx() \
( ((REG_PCM_INTS & PCM_INTS_RFL_MASK) >> PCM_INTS_RFL_BIT) )

#define __pcm_set_clk_div(n) \
( REG_PCM_DIV = (REG_PCM_DIV & ~PCM_DIV_CLKDIV_MASK) | ((n) << PCM_DIV_CLKDIV_BIT) )

/* sysclk(cpm_pcm_sysclk) Hz is created by cpm logic, and pcmclk Hz is the pcm in/out clock wanted */
#define __pcm_set_clk_rate(sysclk, pcmclk) \
__pcm_set_clk_div(((sysclk) / (pcmclk) - 1)) 

#define __pcm_set_sync_div(n) \
( REG_PCM_DIV = (REG_PCM_DIV & ~PCM_DIV_SYNDIV_MASK) | ((n) << PCM_DIV_SYNDIV_BIT) )

/* pcmclk is source clock Hz, and sync is the frame sync clock Hz wanted */
#define __pcm_set_sync_rate(pcmclk, sync) \
__pcm_set_sync_div(((pcmclk) / (8 * (sync)) - 1))

 /* set sync length in pcmclk n = 0 ... 63 */
#define __pcm_set_sync_len(n) \
( REG_PCM_DIV = (REG_PCM_DIV & ~PCM_DIV_SYNL_MASK) | (n << PCM_DIV_SYNL_BIT) )

#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4810PCM_H__ */

