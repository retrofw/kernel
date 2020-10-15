/*
 * linux/include/asm-mips/mach-jz4760/jz4760scc.h
 *
 * JZ4760 SCC register definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ4760SCC_H__
#define __JZ4760SCC_H__


#define	SCC_BASE	0xB0040000

/*************************************************************************
 * SCC
 *************************************************************************/
#define	SCC_DR			(SCC_BASE + 0x000)
#define	SCC_FDR			(SCC_BASE + 0x004)
#define	SCC_CR			(SCC_BASE + 0x008)
#define	SCC_SR			(SCC_BASE + 0x00C)
#define	SCC_TFR			(SCC_BASE + 0x010)
#define	SCC_EGTR		(SCC_BASE + 0x014)
#define	SCC_ECR			(SCC_BASE + 0x018)
#define	SCC_RTOR		(SCC_BASE + 0x01C)

#define REG_SCC_DR		REG8(SCC_DR)
#define REG_SCC_FDR		REG8(SCC_FDR)
#define REG_SCC_CR		REG32(SCC_CR)
#define REG_SCC_SR		REG16(SCC_SR)
#define REG_SCC_TFR		REG16(SCC_TFR)
#define REG_SCC_EGTR		REG8(SCC_EGTR)
#define REG_SCC_ECR		REG32(SCC_ECR)
#define REG_SCC_RTOR		REG8(SCC_RTOR)

/* SCC FIFO Data Count Register (SCC_FDR) */

#define SCC_FDR_EMPTY		0x00
#define SCC_FDR_FULL		0x10

/* SCC Control Register (SCC_CR) */

#define SCC_CR_SCCE		(1 << 31)
#define SCC_CR_TRS		(1 << 30)
#define SCC_CR_T2R		(1 << 29)
#define SCC_CR_FDIV_BIT		24
#define SCC_CR_FDIV_MASK	(0x3 << SCC_CR_FDIV_BIT)
  #define SCC_CR_FDIV_1		(0 << SCC_CR_FDIV_BIT) /* SCC_CLK frequency is the same as device clock */
  #define SCC_CR_FDIV_2		(1 << SCC_CR_FDIV_BIT) /* SCC_CLK frequency is half of device clock */
#define SCC_CR_FLUSH		(1 << 23)
#define SCC_CR_TRIG_BIT		16
#define SCC_CR_TRIG_MASK	(0x3 << SCC_CR_TRIG_BIT)
  #define SCC_CR_TRIG_1		(0 << SCC_CR_TRIG_BIT) /* Receive/Transmit-FIFO Trigger is 1 */
  #define SCC_CR_TRIG_4		(1 << SCC_CR_TRIG_BIT) /* Receive/Transmit-FIFO Trigger is 4 */
  #define SCC_CR_TRIG_8		(2 << SCC_CR_TRIG_BIT) /* Receive/Transmit-FIFO Trigger is 8 */
  #define SCC_CR_TRIG_14	(3 << SCC_CR_TRIG_BIT) /* Receive/Transmit-FIFO Trigger is 14 */
#define SCC_CR_TP		(1 << 15)
#define SCC_CR_CONV		(1 << 14)
#define SCC_CR_TXIE		(1 << 13)
#define SCC_CR_RXIE		(1 << 12)
#define SCC_CR_TENDIE		(1 << 11)
#define SCC_CR_RTOIE		(1 << 10)
#define SCC_CR_ECIE		(1 << 9)
#define SCC_CR_EPIE		(1 << 8)
#define SCC_CR_RETIE		(1 << 7)
#define SCC_CR_EOIE		(1 << 6)
#define SCC_CR_TSEND		(1 << 3)
#define SCC_CR_PX_BIT		1
#define SCC_CR_PX_MASK		(0x3 << SCC_CR_PX_BIT)
  #define SCC_CR_PX_NOT_SUPPORT	(0 << SCC_CR_PX_BIT) /* SCC does not support clock stop */
  #define SCC_CR_PX_STOP_LOW	(1 << SCC_CR_PX_BIT) /* SCC_CLK stops at state low */
  #define SCC_CR_PX_STOP_HIGH	(2 << SCC_CR_PX_BIT) /* SCC_CLK stops at state high */
#define SCC_CR_CLKSTP		(1 << 0)

/* SCC Status Register (SCC_SR) */

#define SCC_SR_TRANS		(1 << 15)
#define SCC_SR_ORER		(1 << 12)
#define SCC_SR_RTO		(1 << 11)
#define SCC_SR_PER		(1 << 10)
#define SCC_SR_TFTG		(1 << 9)
#define SCC_SR_RFTG		(1 << 8)
#define SCC_SR_TEND		(1 << 7)
#define SCC_SR_RETR_3		(1 << 4)
#define SCC_SR_ECNTO		(1 << 0)


#ifndef __MIPS_ASSEMBLER

/***************************************************************************
 * SCC
 ***************************************************************************/

#define __scc_enable()			( REG_SCC_CR |= SCC_CR_SCCE )
#define __scc_disable()			( REG_SCC_CR &= ~SCC_CR_SCCE )

#define __scc_set_tx_mode()		( REG_SCC_CR |= SCC_CR_TRS )
#define __scc_set_rx_mode()		( REG_SCC_CR &= ~SCC_CR_TRS )

#define __scc_enable_t2r()		( REG_SCC_CR |= SCC_CR_T2R )
#define __scc_disable_t2r()		( REG_SCC_CR &= ~SCC_CR_T2R )

#define __scc_clk_as_devclk()			\
do {						\
  REG_SCC_CR &= ~SCC_CR_FDIV_MASK;		\
  REG_SCC_CR |= SCC_CR_FDIV_1;			\
} while (0)

#define __scc_clk_as_half_devclk()		\
do {						\
  REG_SCC_CR &= ~SCC_CR_FDIV_MASK;		\
  REG_SCC_CR |= SCC_CR_FDIV_2;			\
} while (0)

/* n=1,4,8,14 */
#define __scc_set_fifo_trigger(n)		\
do {						\
  REG_SCC_CR &= ~SCC_CR_TRIG_MASK;		\
  REG_SCC_CR |= SCC_CR_TRIG_##n;		\
} while (0)

#define __scc_set_protocol(p)			\
do {						\
	if (p)					\
	  	REG_SCC_CR |= SCC_CR_TP;	\
	else					\
	 	REG_SCC_CR &= ~SCC_CR_TP;	\
} while (0)

#define __scc_flush_fifo()		( REG_SCC_CR |= SCC_CR_FLUSH )

#define __scc_set_invert_mode()		( REG_SCC_CR |= SCC_CR_CONV )
#define __scc_set_direct_mode()		( REG_SCC_CR &= ~SCC_CR_CONV )

#define SCC_ERR_INTRS \
    ( SCC_CR_ECIE | SCC_CR_EPIE | SCC_CR_RETIE | SCC_CR_EOIE )
#define SCC_ALL_INTRS \
    ( SCC_CR_TXIE | SCC_CR_RXIE | SCC_CR_TENDIE | SCC_CR_RTOIE | \
      SCC_CR_ECIE | SCC_CR_EPIE | SCC_CR_RETIE | SCC_CR_EOIE )

#define __scc_enable_err_intrs()	( REG_SCC_CR |= SCC_ERR_INTRS )
#define __scc_disable_err_intrs()	( REG_SCC_CR &= ~SCC_ERR_INTRS )

#define SCC_ALL_ERRORS \
    ( SCC_SR_ORER | SCC_SR_RTO | SCC_SR_PER | SCC_SR_RETR_3 | SCC_SR_ECNTO)

#define __scc_clear_errors()		( REG_SCC_SR &= ~SCC_ALL_ERRORS )

#define __scc_enable_all_intrs()	( REG_SCC_CR |= SCC_ALL_INTRS )
#define __scc_disable_all_intrs()	( REG_SCC_CR &= ~SCC_ALL_INTRS )

#define __scc_enable_tx_intr()		( REG_SCC_CR |= SCC_CR_TXIE | SCC_CR_TENDIE )
#define __scc_disable_tx_intr()		( REG_SCC_CR &= ~(SCC_CR_TXIE | SCC_CR_TENDIE) )

#define __scc_enable_rx_intr()		( REG_SCC_CR |= SCC_CR_RXIE)
#define __scc_disable_rx_intr()		( REG_SCC_CR &= ~SCC_CR_RXIE)

#define __scc_set_tsend()		( REG_SCC_CR |= SCC_CR_TSEND )
#define __scc_clear_tsend()		( REG_SCC_CR &= ~SCC_CR_TSEND )

#define __scc_set_clockstop()		( REG_SCC_CR |= SCC_CR_CLKSTP )
#define __scc_clear_clockstop()		( REG_SCC_CR &= ~SCC_CR_CLKSTP )

#define __scc_clockstop_low()			\
do {						\
  REG_SCC_CR &= ~SCC_CR_PX_MASK;		\
  REG_SCC_CR |= SCC_CR_PX_STOP_LOW;		\
} while (0)

#define __scc_clockstop_high()			\
do {						\
  REG_SCC_CR &= ~SCC_CR_PX_MASK;		\
  REG_SCC_CR |= SCC_CR_PX_STOP_HIGH;		\
} while (0)

/* SCC status checking */
#define __scc_check_transfer_status()		( REG_SCC_SR & SCC_SR_TRANS )
#define __scc_check_rx_overrun_error()		( REG_SCC_SR & SCC_SR_ORER )
#define __scc_check_rx_timeout()		( REG_SCC_SR & SCC_SR_RTO )
#define __scc_check_parity_error()		( REG_SCC_SR & SCC_SR_PER )
#define __scc_check_txfifo_trigger()		( REG_SCC_SR & SCC_SR_TFTG )
#define __scc_check_rxfifo_trigger()		( REG_SCC_SR & SCC_SR_RFTG )
#define __scc_check_tx_end()			( REG_SCC_SR & SCC_SR_TEND )
#define __scc_check_retx_3()			( REG_SCC_SR & SCC_SR_RETR_3 )
#define __scc_check_ecnt_overflow()		( REG_SCC_SR & SCC_SR_ECNTO )


#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4760SCC_H__ */

