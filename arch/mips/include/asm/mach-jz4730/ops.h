/*
 *  linux/include/asm-mips/mach-jz4730/ops.h
 *
 *  JZ4730 module operations definition.
 *
 *  Copyright (C) 2006 - 2007 Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4730_OPS_H__
#define __ASM_JZ4730_OPS_H__

/***************************************************************************
 * MSC
 ***************************************************************************/

#define __msc_start_op() \
  ( REG_MSC_STRPCL = MSC_STRPCL_START_OP | MSC_STRPCL_CLOCK_CONTROL_START )

#define __msc_set_resto(to) 	( REG_MSC_RESTO = to )
#define __msc_set_rdto(to) 	( REG_MSC_RDTO = to )
#define __msc_set_cmd(cmd) 	( REG_MSC_CMD = cmd )
#define __msc_set_arg(arg) 	( REG_MSC_ARG = arg )
#define __msc_set_nob(nob) 	( REG_MSC_NOB = nob )
#define __msc_get_nob() 	( REG_MSC_NOB )
#define __msc_set_blklen(len) 	( REG_MSC_BLKLEN = len )
#define __msc_set_cmdat(cmdat) 	( REG_MSC_CMDAT = cmdat )
#define __msc_set_cmdat_ioabort() 	( REG_MSC_CMDAT |= MSC_CMDAT_IO_ABORT )
#define __msc_clear_cmdat_ioabort() 	( REG_MSC_CMDAT &= ~MSC_CMDAT_IO_ABORT )

#define __msc_set_cmdat_bus_width1() 			\
do { 							\
	REG_MSC_CMDAT &= ~MSC_CMDAT_BUS_WIDTH_MASK; 	\
	REG_MSC_CMDAT |= MSC_CMDAT_BUS_WIDTH_1BIT; 	\
} while(0)

#define __msc_set_cmdat_bus_width4() 			\
do { 							\
	REG_MSC_CMDAT &= ~MSC_CMDAT_BUS_WIDTH_MASK; 	\
	REG_MSC_CMDAT |= MSC_CMDAT_BUS_WIDTH_4BIT; 	\
} while(0)

#define __msc_set_cmdat_dma_en() ( REG_MSC_CMDAT |= MSC_CMDAT_DMA_EN )
#define __msc_set_cmdat_init() 	( REG_MSC_CMDAT |= MSC_CMDAT_INIT )
#define __msc_set_cmdat_busy() 	( REG_MSC_CMDAT |= MSC_CMDAT_BUSY )
#define __msc_set_cmdat_stream() ( REG_MSC_CMDAT |= MSC_CMDAT_STREAM_BLOCK )
#define __msc_set_cmdat_block() ( REG_MSC_CMDAT &= ~MSC_CMDAT_STREAM_BLOCK )
#define __msc_set_cmdat_read() 	( REG_MSC_CMDAT &= ~MSC_CMDAT_WRITE_READ )
#define __msc_set_cmdat_write() ( REG_MSC_CMDAT |= MSC_CMDAT_WRITE_READ )
#define __msc_set_cmdat_data_en() ( REG_MSC_CMDAT |= MSC_CMDAT_DATA_EN )

/* r is MSC_CMDAT_RESPONSE_FORMAT_Rx or MSC_CMDAT_RESPONSE_FORMAT_NONE */
#define __msc_set_cmdat_res_format(r) 				\
do { 								\
	REG_MSC_CMDAT &= ~MSC_CMDAT_RESPONSE_FORMAT_MASK; 	\
	REG_MSC_CMDAT |= (r); 					\
} while(0)

#define __msc_clear_cmdat() \
  REG_MSC_CMDAT &= ~( MSC_CMDAT_IO_ABORT | MSC_CMDAT_DMA_EN | MSC_CMDAT_INIT| \
  MSC_CMDAT_BUSY | MSC_CMDAT_STREAM_BLOCK | MSC_CMDAT_WRITE_READ | \
  MSC_CMDAT_DATA_EN | MSC_CMDAT_RESPONSE_FORMAT_MASK )

#define __msc_get_imask() 		( REG_MSC_IMASK )
#define __msc_mask_all_intrs() 		( REG_MSC_IMASK = 0xff )
#define __msc_unmask_all_intrs() 	( REG_MSC_IMASK = 0x00 )
#define __msc_mask_rd() 		( REG_MSC_IMASK |= MSC_IMASK_RXFIFO_RD_REQ )
#define __msc_unmask_rd() 		( REG_MSC_IMASK &= ~MSC_IMASK_RXFIFO_RD_REQ )
#define __msc_mask_wr() 		( REG_MSC_IMASK |= MSC_IMASK_TXFIFO_WR_REQ )
#define __msc_unmask_wr() 		( REG_MSC_IMASK &= ~MSC_IMASK_TXFIFO_WR_REQ )
#define __msc_mask_endcmdres() 		( REG_MSC_IMASK |= MSC_IMASK_END_CMD_RES )
#define __msc_unmask_endcmdres() 	( REG_MSC_IMASK &= ~MSC_IMASK_END_CMD_RES )
#define __msc_mask_datatrandone() 	( REG_MSC_IMASK |= MSC_IMASK_DATA_TRAN_DONE )
#define __msc_unmask_datatrandone() 	( REG_MSC_IMASK &= ~MSC_IMASK_DATA_TRAN_DONE )
#define __msc_mask_prgdone() 		( REG_MSC_IMASK |= MSC_IMASK_PRG_DONE )
#define __msc_unmask_prgdone() 		( REG_MSC_IMASK &= ~MSC_IMASK_PRG_DONE )

/* n=0,1,2,3,4,5,6,7 */
#define __msc_set_clkrt(n) 	\
do { 				\
	REG_MSC_CLKRT = n;	\
} while(0)

#define __msc_get_ireg() 		( REG_MSC_IREG )
#define __msc_ireg_rd() 		( REG_MSC_IREG & MSC_IREG_RXFIFO_RD_REQ )
#define __msc_ireg_wr() 		( REG_MSC_IREG & MSC_IREG_TXFIFO_WR_REQ )
#define __msc_ireg_end_cmd_res() 	( REG_MSC_IREG & MSC_IREG_END_CMD_RES )
#define __msc_ireg_data_tran_done() 	( REG_MSC_IREG & MSC_IREG_DATA_TRAN_DONE )
#define __msc_ireg_prg_done() 		( REG_MSC_IREG & MSC_IREG_PRG_DONE )
#define __msc_ireg_clear_end_cmd_res() 	( REG_MSC_IREG = MSC_IREG_END_CMD_RES )
#define __msc_ireg_clear_data_tran_done() ( REG_MSC_IREG = MSC_IREG_DATA_TRAN_DONE )
#define __msc_ireg_clear_prg_done() 	( REG_MSC_IREG = MSC_IREG_PRG_DONE )

#define __msc_get_stat() 		( REG_MSC_STAT )
#define __msc_stat_not_end_cmd_res() 	( (REG_MSC_STAT & MSC_STAT_END_CMD_RES) == 0)
#define __msc_stat_crc_err() \
  ( REG_MSC_STAT & (MSC_STAT_CRC_RES_ERR | MSC_STAT_CRC_READ_ERROR | MSC_STAT_CRC_WRITE_ERROR_YES) )
#define __msc_stat_res_crc_err() 	( REG_MSC_STAT & MSC_STAT_CRC_RES_ERR )
#define __msc_stat_rd_crc_err() 	( REG_MSC_STAT & MSC_STAT_CRC_READ_ERROR )
#define __msc_stat_wr_crc_err() 	( REG_MSC_STAT & MSC_STAT_CRC_WRITE_ERROR_YES )
#define __msc_stat_resto_err() 		( REG_MSC_STAT & MSC_STAT_TIME_OUT_RES )
#define __msc_stat_rdto_err() 		( REG_MSC_STAT & MSC_STAT_TIME_OUT_READ )

#define __msc_rd_resfifo() 		( REG_MSC_RES )
#define __msc_rd_rxfifo()  		( REG_MSC_RXFIFO )
#define __msc_wr_txfifo(v)  		( REG_MSC_TXFIFO = v )

#define __msc_reset() 						\
do { 								\
	REG_MSC_STRPCL = MSC_STRPCL_RESET;			\
 	while (REG_MSC_STAT & MSC_STAT_IS_RESETTING);		\
} while (0)

#define __msc_start_clk() 					\
do { 								\
	REG_MSC_STRPCL = MSC_STRPCL_CLOCK_CONTROL_START;	\
} while (0)

#define __msc_stop_clk() 					\
do { 								\
	REG_MSC_STRPCL = MSC_STRPCL_CLOCK_CONTROL_STOP;	\
} while (0)

#define MMC_CLK 19169200
#define SD_CLK  24576000

/* msc_clk should little than pclk and little than clk retrieve from card */
#define __msc_calc_clk_divisor(type,dev_clk,msc_clk,lv)		\
do {								\
	unsigned int rate, pclk, i;				\
	pclk = dev_clk;						\
	rate = type?SD_CLK:MMC_CLK;				\
  	if (msc_clk && msc_clk < pclk)				\
    		pclk = msc_clk;					\
	i = 0;							\
  	while (pclk < rate)					\
    	{							\
      		i ++;						\
      		rate >>= 1;					\
    	}							\
  	lv = i;							\
} while(0)

/* divide rate to little than or equal to 400kHz */
#define __msc_calc_slow_clk_divisor(type, lv)			\
do {								\
	unsigned int rate, i;					\
	rate = (type?SD_CLK:MMC_CLK)/1000/400;			\
	i = 0;							\
	while (rate > 0)					\
    	{							\
      		rate >>= 1;					\
      		i ++;						\
    	}							\
  	lv = i;							\
} while(0)

/***************************************************************************
 * RTC
 ***************************************************************************/

#define __rtc_start()	                ( REG_RTC_RCR |= RTC_RCR_START )
#define __rtc_stop()	                ( REG_RTC_RCR &= ~RTC_RCR_START )

#define __rtc_enable_alarm()	        ( REG_RTC_RCR |= RTC_RCR_AE )
#define __rtc_disable_alarm()	        ( REG_RTC_RCR &= ~RTC_RCR_AE )
#define __rtc_enable_alarm_irq()	( REG_RTC_RCR |= RTC_RCR_AIE )
#define __rtc_disable_alarm_irq()	( REG_RTC_RCR &= ~RTC_RCR_AIE )

#define __rtc_enable_1hz_irq()		( REG_RTC_RCR |= RTC_RCR_HZIE )
#define __rtc_disable_1hz_irq()		( REG_RTC_RCR &= ~RTC_RCR_HZIE )

#define __rtc_is_alarm_flag()		( REG_RTC_RCR & RTC_RCR_AF )
#define __rtc_is_1hz_flag()		( REG_RTC_RCR & RTC_RCR_HZ )
#define __rtc_clear_alarm_flag()	( REG_RTC_RCR &= ~RTC_RCR_AF )
#define __rtc_clear_1hz_flag()		( REG_RTC_RCR &= ~RTC_RCR_HZ )

#define __rtc_set_second(s)	        ( REG_RTC_RSR = (s) )
#define __rtc_get_second()	        REG_RTC_RSR
#define __rtc_set_alarm(s)	        ( REG_RTC_RSAR = (s) )
#define __rtc_get_alarm()	        REG_RTC_RSAR

#define __rtc_adjust_1hz(f32k) \
  ( REG_RTC_RGR = (REG_RTC_RGR & ~(RTC_REG_DIV_MASK | RTC_RGR_ADJ_MASK)) | f32k | 0 )
#define __rtc_lock_1hz()	( REG_RTC_RGR |= RTC_RGR_LOCK )


/***************************************************************************
 * FIR
 ***************************************************************************/

/* enable/disable fir unit */
#define __fir_enable()		( REG_FIR_CR1 |= FIR_CR1_FIRUE )
#define __fir_disable()		( REG_FIR_CR1 &= ~FIR_CR1_FIRUE )

/* enable/disable address comparison */
#define __fir_enable_ac()	( REG_FIR_CR1 |= FIR_CR1_ACE )
#define __fir_disable_ac()	( REG_FIR_CR1 &= ~FIR_CR1_ACE )

/* select frame end mode as underrun or normal */
#define __fir_set_eous()	( REG_FIR_CR1 |= FIR_CR1_EOUS )
#define __fir_clear_eous()	( REG_FIR_CR1 &= ~FIR_CR1_EOUS )

/* enable/disable transmitter idle interrupt */
#define __fir_enable_tii()	( REG_FIR_CR1 |= FIR_CR1_TIIE )
#define __fir_disable_tii()	( REG_FIR_CR1 &= ~FIR_CR1_TIIE )

/* enable/disable transmit FIFO service request interrupt */
#define __fir_enable_tfi()	( REG_FIR_CR1 |= FIR_CR1_TFIE )
#define __fir_disable_tfi()	( REG_FIR_CR1 &= ~FIR_CR1_TFIE )

/* enable/disable receive FIFO service request interrupt */
#define __fir_enable_rfi()	( REG_FIR_CR1 |= FIR_CR1_RFIE )
#define __fir_disable_rfi()	( REG_FIR_CR1 &= ~FIR_CR1_RFIE )

/* enable/disable tx function */
#define __fir_tx_enable()	( REG_FIR_CR1 |= FIR_CR1_TXE )
#define __fir_tx_disable()	( REG_FIR_CR1 &= ~FIR_CR1_TXE )

/* enable/disable rx function */
#define __fir_rx_enable()	( REG_FIR_CR1 |= FIR_CR1_RXE )
#define __fir_rx_disable()	( REG_FIR_CR1 &= ~FIR_CR1_RXE )


/* enable/disable serial infrared interaction pulse (SIP) */
#define __fir_enable_sip()	( REG_FIR_CR2 |= FIR_CR2_SIPE )
#define __fir_disable_sip()	( REG_FIR_CR2 &= ~FIR_CR2_SIPE )

/* un-inverted CRC value is sent out */
#define __fir_enable_bcrc()	( REG_FIR_CR2 |= FIR_CR2_BCRC )

/* inverted CRC value is sent out */
#define __fir_disable_bcrc()	( REG_FIR_CR2 &= ~FIR_CR2_BCRC )

/* enable/disable Transmit Frame Length Register */
#define __fir_enable_tflr()	( REG_FIR_CR2 |= FIR_CR2_TFLRS )
#define __fir_disable_tflr()	( REG_FIR_CR2 &= ~FIR_CR2_TFLRS )

/* Preamble is transmitted in idle state */
#define __fir_set_iss()	( REG_FIR_CR2 |= FIR_CR2_ISS )

/* Abort symbol is transmitted in idle state */
#define __fir_clear_iss()	( REG_FIR_CR2 &= ~FIR_CR2_ISS )

/* enable/disable loopback mode */
#define __fir_enable_loopback()	( REG_FIR_CR2 |= FIR_CR2_LMS )
#define __fir_disable_loopback()	( REG_FIR_CR2 &= ~FIR_CR2_LMS )

/* select transmit pin polarity */
#define __fir_tpp_negative()	( REG_FIR_CR2 |= FIR_CR2_TPPS )
#define __fir_tpp_positive()	( REG_FIR_CR2 &= ~FIR_CR2_TPPS )

/* select receive pin polarity */
#define __fir_rpp_negative()	( REG_FIR_CR2 |= FIR_CR2_RPPS )
#define __fir_rpp_positive()	( REG_FIR_CR2 &= ~FIR_CR2_RPPS )

/* n=16,32,64,128 */
#define __fir_set_txfifo_trigger(n) 		\
do { 						\
	REG_FIR_CR2 &= ~FIR_CR2_TTRG_MASK;	\
	REG_FIR_CR2 |= FIR_CR2_TTRG_##n;	\
} while (0)

/* n=16,32,64,128 */
#define __fir_set_rxfifo_trigger(n) 		\
do { 						\
	REG_FIR_CR2 &= ~FIR_CR2_RTRG_MASK;	\
	REG_FIR_CR2 |= FIR_CR2_RTRG_##n;	\
} while (0)


/* FIR status checking */

#define __fir_test_rfw()	( REG_FIR_SR & FIR_SR_RFW )
#define __fir_test_rfa()	( REG_FIR_SR & FIR_SR_RFA )
#define __fir_test_tfrtl()	( REG_FIR_SR & FIR_SR_TFRTL )
#define __fir_test_rfrtl()	( REG_FIR_SR & FIR_SR_RFRTL )
#define __fir_test_urun()	( REG_FIR_SR & FIR_SR_URUN )
#define __fir_test_rfte()	( REG_FIR_SR & FIR_SR_RFTE )
#define __fir_test_orun()	( REG_FIR_SR & FIR_SR_ORUN )
#define __fir_test_crce()	( REG_FIR_SR & FIR_SR_CRCE )
#define __fir_test_fend()	( REG_FIR_SR & FIR_SR_FEND )
#define __fir_test_tff()	( REG_FIR_SR & FIR_SR_TFF )
#define __fir_test_rfe()	( REG_FIR_SR & FIR_SR_RFE )
#define __fir_test_tidle()	( REG_FIR_SR & FIR_SR_TIDLE )
#define __fir_test_rb()		( REG_FIR_SR & FIR_SR_RB )

#define __fir_clear_status()					\
do { 								\
	REG_FIR_SR |= FIR_SR_RFW | FIR_SR_RFA | FIR_SR_URUN;	\
} while (0)

#define __fir_clear_rfw()	( REG_FIR_SR |= FIR_SR_RFW )
#define __fir_clear_rfa()	( REG_FIR_SR |= FIR_SR_RFA )
#define __fir_clear_urun()	( REG_FIR_SR |= FIR_SR_URUN )

#define __fir_set_tflr(len)			\
do { 						\
	REG_FIR_TFLR = len; 			\
} while (0)

#define __fir_set_addr(a)	( REG_FIR_AR = (a) )

#define __fir_write_data(data)	( REG_FIR_TDR = data )
#define __fir_read_data(data)	( data = REG_FIR_RDR )

/***************************************************************************
 * SCC
 ***************************************************************************/

#define __scc_enable(base)	( REG_SCC_CR(base) |= SCC_CR_SCCE )
#define __scc_disable(base)	( REG_SCC_CR(base) &= ~SCC_CR_SCCE )

#define __scc_set_tx_mode(base)	( REG_SCC_CR(base) |= SCC_CR_TRS )
#define __scc_set_rx_mode(base)	( REG_SCC_CR(base) &= ~SCC_CR_TRS )

#define __scc_enable_t2r(base)	( REG_SCC_CR(base) |= SCC_CR_T2R )
#define __scc_disable_t2r(base)	( REG_SCC_CR(base) &= ~SCC_CR_T2R )

#define __scc_clk_as_devclk(base)		\
do {						\
  REG_SCC_CR(base) &= ~SCC_CR_FDIV_MASK;	\
  REG_SCC_CR(base) |= SCC_CR_FDIV_1;		\
} while (0)

#define __scc_clk_as_half_devclk(base)		\
do {						\
  REG_SCC_CR(base) &= ~SCC_CR_FDIV_MASK;	\
  REG_SCC_CR(base) |= SCC_CR_FDIV_2;		\
} while (0)

/* n=1,4,8,14 */
#define __scc_set_fifo_trigger(base, n)		\
do {						\
  REG_SCC_CR(base) &= ~SCC_CR_TRIG_MASK;	\
  REG_SCC_CR(base) |= SCC_CR_TRIG_##n;		\
} while (0)

#define __scc_set_protocol(base, p)		\
do {						\
	if (p)					\
	  	REG_SCC_CR(base) |= SCC_CR_TP;	\
	else					\
	 	REG_SCC_CR(base) &= ~SCC_CR_TP;	\
} while (0)

#define __scc_flush_fifo(base)	( REG_SCC_CR(base) |= SCC_CR_FLUSH )

#define __scc_set_invert_mode(base)	( REG_SCC_CR(base) |= SCC_CR_CONV )
#define __scc_set_direct_mode(base)	( REG_SCC_CR(base) &= ~SCC_CR_CONV )

#define SCC_ERR_INTRS \
    ( SCC_CR_ECIE | SCC_CR_EPIE | SCC_CR_RETIE | SCC_CR_EOIE )
#define SCC_ALL_INTRS \
    ( SCC_CR_TXIE | SCC_CR_RXIE | SCC_CR_TENDIE | SCC_CR_RTOIE | \
      SCC_CR_ECIE | SCC_CR_EPIE | SCC_CR_RETIE | SCC_CR_EOIE )

#define __scc_enable_err_intrs(base)	( REG_SCC_CR(base) |= SCC_ERR_INTRS )
#define __scc_disable_err_intrs(base)	( REG_SCC_CR(base) &= ~SCC_ERR_INTRS )

#define SCC_ALL_ERRORS \
    ( SCC_SR_ORER | SCC_SR_RTO | SCC_SR_PER | SCC_SR_RETR_3 | SCC_SR_ECNTO)

#define __scc_clear_errors(base)	( REG_SCC_SR(base) &= ~SCC_ALL_ERRORS )

#define __scc_enable_all_intrs(base)	( REG_SCC_CR(base) |= SCC_ALL_INTRS )
#define __scc_disable_all_intrs(base)	( REG_SCC_CR(base) &= ~SCC_ALL_INTRS )

#define __scc_enable_tx_intr(base)	( REG_SCC_CR(base) |= SCC_CR_TXIE | SCC_CR_TENDIE )
#define __scc_disable_tx_intr(base)	( REG_SCC_CR(base) &= ~(SCC_CR_TXIE | SCC_CR_TENDIE) )

#define __scc_enable_rx_intr(base)	( REG_SCC_CR(base) |= SCC_CR_RXIE)
#define __scc_disable_rx_intr(base)	( REG_SCC_CR(base) &= ~SCC_CR_RXIE)

#define __scc_set_tsend(base)		( REG_SCC_CR(base) |= SCC_CR_TSEND )
#define __scc_clear_tsend(base)		( REG_SCC_CR(base) &= ~SCC_CR_TSEND )

#define __scc_set_clockstop(base)	( REG_SCC_CR(base) |= SCC_CR_CLKSTP )
#define __scc_clear_clockstop(base)	( REG_SCC_CR(base) &= ~SCC_CR_CLKSTP )

#define __scc_clockstop_low(base)		\
do {						\
  REG_SCC_CR(base) &= ~SCC_CR_PX_MASK;		\
  REG_SCC_CR(base) |= SCC_CR_PX_STOP_LOW;	\
} while (0)

#define __scc_clockstop_high(base)		\
do {						\
  REG_SCC_CR(base) &= ~SCC_CR_PX_MASK;		\
  REG_SCC_CR(base) |= SCC_CR_PX_STOP_HIGH;	\
} while (0)


/* SCC status checking */
#define __scc_check_transfer_status(base)  ( REG_SCC_SR(base) & SCC_SR_TRANS )
#define __scc_check_rx_overrun_error(base) ( REG_SCC_SR(base) & SCC_SR_ORER )
#define __scc_check_rx_timeout(base)	   ( REG_SCC_SR(base) & SCC_SR_RTO )
#define __scc_check_parity_error(base)	   ( REG_SCC_SR(base) & SCC_SR_PER )
#define __scc_check_txfifo_trigger(base)   ( REG_SCC_SR(base) & SCC_SR_TFTG )
#define __scc_check_rxfifo_trigger(base)   ( REG_SCC_SR(base) & SCC_SR_RFTG )
#define __scc_check_tx_end(base)	   ( REG_SCC_SR(base) & SCC_SR_TEND )
#define __scc_check_retx_3(base)	   ( REG_SCC_SR(base) & SCC_SR_RETR_3 )
#define __scc_check_ecnt_overflow(base)	   ( REG_SCC_SR(base) & SCC_SR_ECNTO )


/***************************************************************************
 * WDT
 ***************************************************************************/

#define __wdt_set_count(count) ( REG_WDT_WTCNT = (count) )
#define __wdt_start()          ( REG_WDT_WTCSR |= WDT_WTCSR_START )
#define __wdt_stop()           ( REG_WDT_WTCSR &= ~WDT_WTCSR_START )


/***************************************************************************
 * OST
 ***************************************************************************/

#define __ost_enable_all()         ( REG_OST_TER |= 0x07 )
#define __ost_disable_all()        ( REG_OST_TER &= ~0x07 )
#define __ost_enable_channel(n)    ( REG_OST_TER |= (1 << (n)) )
#define __ost_disable_channel(n)   ( REG_OST_TER &= ~(1 << (n)) )
#define __ost_set_reload(n, val)   ( REG_OST_TRDR(n) = (val) )
#define __ost_set_count(n, val)    ( REG_OST_TCNT(n) = (val) )
#define __ost_get_count(n)         ( REG_OST_TCNT(n) )
#define __ost_set_clock(n, cs)			\
do {						\
    REG_OST_TCSR(n) &= ~OST_TCSR_CKS_MASK; 	\
    REG_OST_TCSR(n) |= cs; 			\
} while (0)
#define __ost_set_mode(n, val)     ( REG_OST_TCSR(n) = (val) )
#define __ost_enable_interrupt(n)  ( REG_OST_TCSR(n) |= OST_TCSR_UIE )
#define __ost_disable_interrupt(n) ( REG_OST_TCSR(n) &= ~OST_TCSR_UIE )
#define __ost_uf_detected(n)       ( REG_OST_TCSR(n) & OST_TCSR_UF )
#define __ost_clear_uf(n)          ( REG_OST_TCSR(n) &= ~OST_TCSR_UF )
#define __ost_is_busy(n)           ( REG_OST_TCSR(n) & OST_TCSR_BUSY )
#define __ost_clear_busy(n)        ( REG_OST_TCSR(n) &= ~OST_TCSR_BUSY )


/***************************************************************************
 * UART
 ***************************************************************************/

#define __uart_enable(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_FCR) |= UARTFCR_UUE | UARTFCR_FE )
#define __uart_disable(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_FCR) = ~UARTFCR_UUE )

#define __uart_enable_transmit_irq(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_IER) |= UARTIER_TIE )
#define __uart_disable_transmit_irq(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_IER) &= ~UARTIER_TIE )

#define __uart_enable_receive_irq(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_IER) |= UARTIER_RIE | UARTIER_RLIE | UARTIER_RTIE )
#define __uart_disable_receive_irq(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_IER) &= ~(UARTIER_RIE | UARTIER_RLIE | UARTIER_RTIE) )

#define __uart_enable_loopback(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_MCR) |= UARTMCR_LOOP )
#define __uart_disable_loopback(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_MCR) &= ~UARTMCR_LOOP )

#define __uart_set_8n1(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_LCR) = UARTLCR_WLEN_8 )

#define __uart_set_baud(n, devclk, baud)						\
  do {											\
	REG8(UART_BASE + UART_OFF*(n) + OFF_LCR) |= UARTLCR_DLAB;			\
	REG8(UART_BASE + UART_OFF*(n) + OFF_DLLR) = (devclk / 16 / baud) & 0xff;	\
	REG8(UART_BASE + UART_OFF*(n) + OFF_DLHR) = ((devclk / 16 / baud) >> 8) & 0xff;	\
	REG8(UART_BASE + UART_OFF*(n) + OFF_LCR) &= ~UARTLCR_DLAB;			\
  } while (0)

#define __uart_parity_error(n) \
  ( (REG8(UART_BASE + UART_OFF*(n) + OFF_LSR) & UARTLSR_PER) != 0 )

#define __uart_clear_errors(n) \
  ( REG8(UART_BASE + UART_OFF*(n) + OFF_LSR) &= ~(UARTLSR_ORER | UARTLSR_BRK | UARTLSR_FER | UARTLSR_PER | UARTLSR_RFER) )

#define __uart_transmit_fifo_empty(n) \
  ( (REG8(UART_BASE + UART_OFF*(n) + OFF_LSR) & UARTLSR_TDRQ) != 0 )

#define __uart_transmit_end(n) \
  ( (REG8(UART_BASE + UART_OFF*(n) + OFF_LSR) & UARTLSR_TEMT) != 0 )

#define __uart_transmit_char(n, ch) \
  REG8(UART_BASE + UART_OFF*(n) + OFF_TDR) = (ch)

#define __uart_receive_fifo_full(n) \
  ( (REG8(UART_BASE + UART_OFF*(n) + OFF_LSR) & UARTLSR_DR) != 0 )

#define __uart_receive_ready(n) \
  ( (REG8(UART_BASE + UART_OFF*(n) + OFF_LSR) & UARTLSR_DR) != 0 )

#define __uart_receive_char(n) \
  REG8(UART_BASE + UART_OFF*(n) + OFF_RDR)

#define __uart_disable_irda() \
  ( REG8(IRDA_BASE + OFF_SIRCR) &= ~(SIRCR_TSIRE | SIRCR_RSIRE) )
#define __uart_enable_irda() \
  /* Tx high pulse as 0, Rx low pulse as 0 */ \
  ( REG8(IRDA_BASE + OFF_SIRCR) = SIRCR_TSIRE | SIRCR_RSIRE | SIRCR_RXPL | SIRCR_TPWS )


/***************************************************************************
 * INTC
 ***************************************************************************/
#define __intc_unmask_irq(n)	( REG_INTC_IMCR = (1 << (n)) )
#define __intc_mask_irq(n)	( REG_INTC_IMSR = (1 << (n)) )
#define __intc_ack_irq(n)	( REG_INTC_IPR = (1 << (n)) )

/***************************************************************************
 * CIM
 ***************************************************************************/

#define __cim_enable()	( REG_CIM_CTRL |= CIM_CTRL_ENA )
#define __cim_disable()	( REG_CIM_CTRL &= ~CIM_CTRL_ENA )

#define __cim_input_data_inverse()	( REG_CIM_CFG |= CIM_CFG_INV_DAT )
#define __cim_input_data_normal()	( REG_CIM_CFG &= ~CIM_CFG_INV_DAT )

#define __cim_vsync_active_low()	( REG_CIM_CFG |= CIM_CFG_VSP )
#define __cim_vsync_active_high()	( REG_CIM_CFG &= ~CIM_CFG_VSP )

#define __cim_hsync_active_low()	( REG_CIM_CFG |= CIM_CFG_HSP )
#define __cim_hsync_active_high()	( REG_CIM_CFG &= ~CIM_CFG_HSP )

#define __cim_sample_data_at_pclk_falling_edge() \
  ( REG_CIM_CFG |= CIM_CFG_PCP )
#define __cim_sample_data_at_pclk_rising_edge() \
  ( REG_CIM_CFG &= ~CIM_CFG_PCP )

#define __cim_enable_dummy_zero()	( REG_CIM_CFG |= CIM_CFG_DUMMY_ZERO )
#define __cim_disable_dummy_zero()	( REG_CIM_CFG &= ~CIM_CFG_DUMMY_ZERO )

#define __cim_select_external_vsync()	( REG_CIM_CFG |= CIM_CFG_EXT_VSYNC )
#define __cim_select_internal_vsync()	( REG_CIM_CFG &= ~CIM_CFG_EXT_VSYNC )

/* n=0-7 */
#define __cim_set_data_packing_mode(n) 		\
do {						\
    REG_CIM_CFG &= ~CIM_CFG_PACK_MASK; 		\
    REG_CIM_CFG |= (CIM_CFG_PACK_##n); 		\
} while (0)

#define __cim_enable_ccir656_progressive_mode()	\
do {						\
    REG_CIM_CFG &= ~CIM_CFG_DSM_MASK; 		\
    REG_CIM_CFG |= CIM_CFG_DSM_CPM; 		\
} while (0)

#define __cim_enable_ccir656_interlace_mode()	\
do {						\
    REG_CIM_CFG &= ~CIM_CFG_DSM_MASK; 		\
    REG_CIM_CFG |= CIM_CFG_DSM_CIM; 		\
} while (0)

#define __cim_enable_gated_clock_mode()		\
do {						\
    REG_CIM_CFG &= ~CIM_CFG_DSM_MASK; 		\
    REG_CIM_CFG |= CIM_CFG_DSM_GCM; 		\
} while (0)

#define __cim_enable_nongated_clock_mode()	\
do {						\
    REG_CIM_CFG &= ~CIM_CFG_DSM_MASK; 		\
    REG_CIM_CFG |= CIM_CFG_DSM_NGCM; 		\
} while (0)

/* sclk:system bus clock
 * mclk: CIM master clock
 */
#define __cim_set_master_clk(sclk, mclk)			\
do {								\
    REG_CIM_CTRL &= ~CIM_CTRL_MCLKDIV_MASK;			\
    REG_CIM_CTRL |= (((sclk)/(mclk) - 1) << CIM_CTRL_MCLKDIV_BIT);	\
} while (0)

#define __cim_enable_sof_intr() \
  ( REG_CIM_CTRL |= CIM_CTRL_DMA_SOFM )
#define __cim_disable_sof_intr() \
  ( REG_CIM_CTRL &= ~CIM_CTRL_DMA_SOFM )

#define __cim_enable_eof_intr() \
  ( REG_CIM_CTRL |= CIM_CTRL_DMA_EOFM )
#define __cim_disable_eof_intr() \
  ( REG_CIM_CTRL &= ~CIM_CTRL_DMA_EOFM )

#define __cim_enable_stop_intr() \
  ( REG_CIM_CTRL |= CIM_CTRL_DMA_STOPM )
#define __cim_disable_stop_intr() \
  ( REG_CIM_CTRL &= ~CIM_CTRL_DMA_STOPM )

#define __cim_enable_trig_intr() \
  ( REG_CIM_CTRL |= CIM_CTRL_RXF_TRIGM )
#define __cim_disable_trig_intr() \
  ( REG_CIM_CTRL &= ~CIM_CTRL_RXF_TRIGM )

#define __cim_enable_rxfifo_overflow_intr() \
  ( REG_CIM_CTRL |= CIM_CTRL_RXF_OFM )
#define __cim_disable_rxfifo_overflow_intr() \
  ( REG_CIM_CTRL &= ~CIM_CTRL_RXF_OFM )

/* n=1-16 */
#define __cim_set_frame_rate(n) 		\
do {						\
    REG_CIM_CTRL &= ~CIM_CTRL_FRC_MASK; 	\
    REG_CIM_CTRL |= CIM_CTRL_FRC_##n; 		\
} while (0)

#define __cim_enable_dma()   ( REG_CIM_CTRL |= CIM_CTRL_DMA_EN )
#define __cim_disable_dma()  ( REG_CIM_CTRL &= ~CIM_CTRL_DMA_EN )

#define __cim_reset_rxfifo() ( REG_CIM_CTRL |= CIM_CTRL_RXF_RST )
#define __cim_unreset_rxfifo() ( REG_CIM_CTRL &= ~CIM_CTRL_RXF_RST )

/* n=4,8,12,16,20,24,28,32 */
#define __cim_set_rxfifo_trigger(n) 		\
do {						\
    REG_CIM_CTRL &= ~CIM_CTRL_RXF_TRIG_MASK; 	\
    REG_CIM_CTRL |= CIM_CTRL_RXF_TRIG_##n; 	\
} while (0)

#define __cim_clear_state()   	     ( REG_CIM_STATE = 0 )

#define __cim_disable_done()   	     ( REG_CIM_STATE & CIM_STATE_VDD )
#define __cim_rxfifo_empty()   	     ( REG_CIM_STATE & CIM_STATE_RXF_EMPTY )
#define __cim_rxfifo_reach_trigger() ( REG_CIM_STATE & CIM_STATE_RXF_TRIG )
#define __cim_rxfifo_overflow()      ( REG_CIM_STATE & CIM_STATE_RXF_OF )
#define __cim_clear_rxfifo_overflow() ( REG_CIM_STATE &= ~CIM_STATE_RXF_OF )
#define __cim_dma_stop()   	     ( REG_CIM_STATE & CIM_STATE_DMA_STOP )
#define __cim_dma_eof()   	     ( REG_CIM_STATE & CIM_STATE_DMA_EOF )
#define __cim_dma_sof()   	     ( REG_CIM_STATE & CIM_STATE_DMA_SOF )

#define __cim_get_iid()   	     ( REG_CIM_IID )
#define __cim_get_image_data()       ( REG_CIM_RXFIFO )
#define __cim_get_dam_cmd()          ( REG_CIM_CMD )

#define __cim_set_da(a)              ( REG_CIM_DA = (a) )

/***************************************************************************
 * PWM
 ***************************************************************************/

/* n is the pwm channel (0,1,..) */
#define __pwm_enable_module(n)		( REG_PWM_CTR(n) |= PWM_CTR_EN )
#define __pwm_disable_module(n)		( REG_PWM_CTR(n) &= ~PWM_CTR_EN )
#define __pwm_graceful_shutdown_mode(n)	( REG_PWM_CTR(n) &= ~PWM_CTR_SD )
#define __pwm_abrupt_shutdown_mode(n)	( REG_PWM_CTR(n) |= PWM_CTR_SD )
#define __pwm_set_full_duty(n)		( REG_PWM_DUT(n) |= PWM_DUT_FDUTY )

#define __pwm_set_prescale(n, p) \
  ( REG_PWM_CTR(n) = ((REG_PWM_CTR(n) & ~PWM_CTR_PRESCALE_MASK) | (p) ) )
#define __pwm_set_period(n, p) \
  ( REG_PWM_PER(n) = ( (REG_PWM_PER(n) & ~PWM_PER_PERIOD_MASK) | (p) ) )
#define __pwm_set_duty(n, d) \
  ( REG_PWM_DUT(n) = ( (REG_PWM_DUT(n) & ~(PWM_DUT_FDUTY | PWM_DUT_DUTY_MASK)) | (d) ) )

/***************************************************************************
 * EMC
 ***************************************************************************/

#define __emc_enable_split() ( REG_EMC_BCR = EMC_BCR_BRE )
#define __emc_disable_split() ( REG_EMC_BCR = 0 )

#define __emc_smem_bus_width(n) /* 8, 16 or 32*/		\
	( REG_EMC_SMCR = (REG_EMC_SMCR & EMC_SMCR_BW_MASK) |	\
			 EMC_SMCR_BW_##n##BIT )
#define __emc_smem_byte_control() \
	( REG_EMC_SMCR = (REG_EMC_SMCR | EMC_SMCR_BCM )
#define __emc_normal_smem() \
	( REG_EMC_SMCR = (REG_EMC_SMCR & ~EMC_SMCR_SMT )
#define __emc_burst_smem() \
	( REG_EMC_SMCR = (REG_EMC_SMCR | EMC_SMCR_SMT )
#define __emc_smem_burstlen(n) /* 4, 8, 16 or 32 */ \
	( REG_EMC_SMCR = (REG_EMC_SMCR & EMC_SMCR_BL_MASK) | (EMC_SMCR_BL_##n )

/***************************************************************************
 * GPIO
 ***************************************************************************/

/* p is the port number (0,1,2,3)
 * o is the pin offset (0-31) inside the port
 * n is the absolute number of a pin (0-124), regardless of the port
 * m is the interrupt manner (low/high/falling/rising)
 */

#define __gpio_port_data(p)	( REG_GPIO_GPDR(p) )

#define __gpio_port_as_output(p, o)		\
do {						\
    unsigned int tmp;				\
    REG_GPIO_GPIER(p) &= ~(1 << (o));		\
    REG_GPIO_GPDIR(p) |= (1 << (o));		\
    if (o < 16) {				\
	tmp = REG_GPIO_GPALR(p);		\
	tmp &= ~(3 << ((o) << 1));		\
	REG_GPIO_GPALR(p) = tmp;		\
    } else {					\
	tmp = REG_GPIO_GPAUR(p);		\
	tmp &= ~(3 << (((o) - 16)<< 1));	\
	REG_GPIO_GPAUR(p) = tmp;		\
    }						\
} while (0)

#define __gpio_port_as_input(p, o)		\
do {						\
    unsigned int tmp;				\
    REG_GPIO_GPIER(p) &= ~(1 << (o));		\
    REG_GPIO_GPDIR(p) &= ~(1 << (o));		\
    if (o < 16) {				\
	tmp = REG_GPIO_GPALR(p);		\
	tmp &= ~(3 << ((o) << 1));		\
	REG_GPIO_GPALR(p) = tmp;		\
    } else {					\
	tmp = REG_GPIO_GPAUR(p);		\
	tmp &= ~(3 << (((o) - 16)<< 1));	\
	REG_GPIO_GPAUR(p) = tmp;		\
    }						\
} while (0)

#define __gpio_as_output(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	__gpio_port_as_output(p, o);		\
} while (0)

#define __gpio_as_input(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	__gpio_port_as_input(p, o);		\
} while (0)

#define __gpio_set_pin(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	__gpio_port_data(p) |= (1 << o);	\
} while (0)

#define __gpio_clear_pin(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	__gpio_port_data(p) &= ~(1 << o);	\
} while (0)

static __inline__ unsigned int __gpio_get_pin(unsigned int n)
{
	unsigned int p, o;
	p = (n) / 32;
	o = (n) % 32;
	if (__gpio_port_data(p) & (1 << o))
		return 1;
	else
		return 0;
}

#define __gpio_set_irq_detect_manner(p, o, m)	\
do {						\
    unsigned int tmp;				\
    if (o < 16) {				\
	tmp = REG_GPIO_GPIDLR(p);		\
	tmp &= ~(3 << ((o) << 1));		\
	tmp |= ((m) << ((o) << 1));		\
	REG_GPIO_GPIDLR(p) = tmp;		\
    } else {					\
	tmp = REG_GPIO_GPIDUR(p);		\
	tmp &= ~(3 << (((o)-16) << 1));		\
	tmp |= ((m) << (((o)-16) << 1));		\
	REG_GPIO_GPIDUR(p) = tmp;		\
    }						\
} while (0)

#define __gpio_port_as_irq(p, o, m)		\
do {						\
    __gpio_port_as_input(p, o);			\
    __gpio_set_irq_detect_manner(p, o, m);  	\
} while (0)

#define __gpio_as_irq(n, m)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
        __gpio_port_as_irq(p, o, m);  		\
} while (0)


#define __gpio_as_irq_high_level(n)	__gpio_as_irq(n, GPIO_IRQ_HILEVEL)
#define __gpio_as_irq_low_level(n)	__gpio_as_irq(n, GPIO_IRQ_LOLEVEL)
#define __gpio_as_irq_fall_edge(n)	__gpio_as_irq(n, GPIO_IRQ_FALLEDG)
#define __gpio_as_irq_rise_edge(n)	__gpio_as_irq(n, GPIO_IRQ_RAISEDG)


#define __gpio_mask_irq(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_GPIER(p) &= ~(1 << o);		\
} while (0)

#define __gpio_unmask_irq(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_GPIER(p) |= (1 << o);		\
} while (0)

#define __gpio_ack_irq(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_GPFR(p) |= (1 << o);		\
} while (0)


static __inline__ unsigned int __gpio_get_irq(void)
{
	unsigned int tmp, i;

	tmp = REG_GPIO_GPFR(3);
	for (i=0; i<32; i++)
		if (tmp & (1 << i))
			return 0x60 + i;
	tmp = REG_GPIO_GPFR(2);
	for (i=0; i<32; i++)
		if (tmp & (1 << i))
			return 0x40 + i;
	tmp = REG_GPIO_GPFR(1);
	for (i=0; i<32; i++)
		if (tmp & (1 << i))
			return 0x20 + i;
	tmp = REG_GPIO_GPFR(0);
	for (i=0; i<32; i++)
		if (tmp & (1 << i))
			return i;
	return 0;
}

#define __gpio_group_irq(n)			\
({						\
	register int tmp, i;			\
	tmp = REG_GPIO_GPFR((n));		\
	for (i=31;i>=0;i--)			\
		if (tmp & (1 << i))		\
			break;			\
	i;					\
})

#define __gpio_enable_pull(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_GPPUR(p) |= (1 << o);		\
} while (0)

#define __gpio_disable_pull(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_GPPUR(p) &= ~(1 << o);		\
} while (0)


/* Init the alternate function pins */


#define __gpio_as_ssi()				\
do {						\
	REG_GPIO_GPALR(2) &= 0xFC00FFFF;	\
	REG_GPIO_GPALR(2) |= 0x01550000;	\
} while (0)

#define __gpio_as_uart3()			\
do {						\
	REG_GPIO_GPAUR(0) &= 0xFFFF0000;	\
	REG_GPIO_GPAUR(0) |= 0x00005555;	\
} while (0)

#define __gpio_as_uart2()			\
do {						\
	REG_GPIO_GPALR(3) &= 0x3FFFFFFF;	\
	REG_GPIO_GPALR(3) |= 0x40000000;	\
	REG_GPIO_GPAUR(3) &= 0xF3FFFFFF;	\
	REG_GPIO_GPAUR(3) |= 0x04000000;	\
} while (0)

#define __gpio_as_uart1()			\
do {						\
	REG_GPIO_GPAUR(0) &= 0xFFF0FFFF;	\
	REG_GPIO_GPAUR(0) |= 0x00050000;	\
} while (0)

#define __gpio_as_uart0()			\
do {						\
	REG_GPIO_GPAUR(3) &= 0x0FFFFFFF;	\
	REG_GPIO_GPAUR(3) |= 0x50000000;	\
} while (0)


#define __gpio_as_scc0()			\
do {						\
	REG_GPIO_GPALR(2) &= 0xFFFFFFCC;	\
	REG_GPIO_GPALR(2) |= 0x00000011;	\
} while (0)

#define __gpio_as_scc1()			\
do {						\
	REG_GPIO_GPALR(2) &= 0xFFFFFF33;	\
	REG_GPIO_GPALR(2) |= 0x00000044;	\
} while (0)

#define __gpio_as_scc()				\
do {						\
	__gpio_as_scc0();			\
	__gpio_as_scc1();			\
} while (0)

#define __gpio_as_dma()				\
do {						\
	REG_GPIO_GPALR(0) &= 0x00FFFFFF;	\
	REG_GPIO_GPALR(0) |= 0x55000000;	\
	REG_GPIO_GPAUR(0) &= 0xFF0FFFFF;	\
	REG_GPIO_GPAUR(0) |= 0x00500000;	\
} while (0)

#define __gpio_as_msc()				\
do {						\
	REG_GPIO_GPALR(1) &= 0xFFFF000F;	\
	REG_GPIO_GPALR(1) |= 0x00005550;	\
} while (0)

#define __gpio_as_pcmcia()			\
do {						\
	REG_GPIO_GPAUR(2) &= 0xF000FFFF;	\
	REG_GPIO_GPAUR(2) |= 0x05550000;	\
} while (0)

#define __gpio_as_emc(csmask)			\
do {						\
	REG_GPIO_GPALR(2) &= 0x3FFFFFFF;	\
	REG_GPIO_GPALR(2) |= 0x40000000;	\
	REG_GPIO_GPAUR(2) &= 0xFFFF0000;	\
	REG_GPIO_GPAUR(2) |= 0x00005555;	\
} while (0)

#define __gpio_as_lcd_slave()			\
do {						\
	REG_GPIO_GPALR(1) &= 0x0000FFFF;	\
	REG_GPIO_GPALR(1) |= 0x55550000;	\
	REG_GPIO_GPAUR(1) &= 0x00000000;	\
	REG_GPIO_GPAUR(1) |= 0x55555555;	\
} while (0)

#define __gpio_as_lcd_master()			\
do {						\
	REG_GPIO_GPALR(1) &= 0x0000FFFF;	\
	REG_GPIO_GPALR(1) |= 0x55550000;	\
	REG_GPIO_GPAUR(1) &= 0x00000000;	\
	REG_GPIO_GPAUR(1) |= 0x556A5555;	\
} while (0)

#define __gpio_as_usb()				\
do {						\
	REG_GPIO_GPAUR(0) &= 0x00FFFFFF;	\
	REG_GPIO_GPAUR(0) |= 0x55000000;	\
} while (0)

#define __gpio_as_ac97()			\
do {						\
	REG_GPIO_GPALR(2) &= 0xC3FF03FF;	\
	REG_GPIO_GPALR(2) |= 0x24005400;	\
} while (0)

#define __gpio_as_i2s_slave()			\
do {						\
	REG_GPIO_GPALR(2) &= 0xC3FF0CFF;	\
	REG_GPIO_GPALR(2) |= 0x14005100;	\
} while (0)

#define __gpio_as_i2s_master()			\
do {						\
	REG_GPIO_GPALR(2) &= 0xC3FF0CFF;	\
	REG_GPIO_GPALR(2) |= 0x28005100;	\
} while (0)

#define __gpio_as_eth()				\
do {						\
	REG_GPIO_GPAUR(3) &= 0xFC000000;	\
	REG_GPIO_GPAUR(3) |= 0x01555555;	\
} while (0)

#define __gpio_as_pwm()				\
do {						\
	REG_GPIO_GPAUR(2) &= 0x0FFFFFFF;	\
	REG_GPIO_GPAUR(2) |= 0x50000000;	\
} while (0)

#define __gpio_as_ps2()				\
do {						\
	REG_GPIO_GPALR(1) &= 0xFFFFFFF0;	\
	REG_GPIO_GPALR(1) |= 0x00000005;	\
} while (0)

#define __gpio_as_uprt()			\
do {						\
	REG_GPIO_GPALR(1) &= 0x0000000F;	\
	REG_GPIO_GPALR(1) |= 0x55555550;	\
	REG_GPIO_GPALR(3) &= 0xC0000000;	\
	REG_GPIO_GPALR(3) |= 0x15555555;	\
} while (0)

#define __gpio_as_cim()				\
do {						\
	REG_GPIO_GPALR(0) &= 0xFF000000;	\
	REG_GPIO_GPALR(0) |= 0x00555555;	\
} while (0)

/***************************************************************************
 * HARB
 ***************************************************************************/

#define __harb_usb0_udc()			\
do {						\
  REG_HARB_HAPOR &= ~HARB_HAPOR_UCHSEL; 	\
} while (0)

#define __harb_usb0_uhc()			\
do {						\
  REG_HARB_HAPOR |= HARB_HAPOR_UCHSEL; 		\
} while (0)

#define __harb_set_priority(n)			\
do {						\
  REG_HARB_HAPOR = ((REG_HARB_HAPOR & ~HARB_HAPOR_PRIO_MASK) | n);	\
} while (0)

/***************************************************************************
 * I2C
 ***************************************************************************/

#define __i2c_enable()		( REG_I2C_CR |= I2C_CR_I2CE )
#define __i2c_disable()		( REG_I2C_CR &= ~I2C_CR_I2CE )

#define __i2c_send_start()	( REG_I2C_CR |= I2C_CR_STA )
#define __i2c_send_stop()	( REG_I2C_CR |= I2C_CR_STO )
#define __i2c_send_ack()	( REG_I2C_CR &= ~I2C_CR_AC )
#define __i2c_send_nack()	( REG_I2C_CR |= I2C_CR_AC )

#define __i2c_set_drf()		( REG_I2C_SR |= I2C_SR_DRF )
#define __i2c_clear_drf()	( REG_I2C_SR &= ~I2C_SR_DRF )
#define __i2c_check_drf()	( REG_I2C_SR & I2C_SR_DRF )

#define __i2c_received_ack()	( !(REG_I2C_SR & I2C_SR_ACKF) )
#define __i2c_is_busy()		( REG_I2C_SR & I2C_SR_BUSY )
#define __i2c_transmit_ended()	( REG_I2C_SR & I2C_SR_TEND )

#define __i2c_set_clk(dev_clk, i2c_clk) \
  ( REG_I2C_GR = (dev_clk) / (16*(i2c_clk)) - 1 )

#define __i2c_read()		( REG_I2C_DR )
#define __i2c_write(val)	( REG_I2C_DR = (val) )

/***************************************************************************
 * UDC
 ***************************************************************************/

#define __udc_set_16bit_phy()		( REG_UDC_DevCFGR |= UDC_DevCFGR_PI )
#define __udc_set_8bit_phy()		( REG_UDC_DevCFGR &= ~UDC_DevCFGR_PI )

#define __udc_enable_sync_frame()	( REG_UDC_DevCFGR |= UDC_DevCFGR_SS )
#define __udc_disable_sync_frame()	( REG_UDC_DevCFGR &= ~UDC_DevCFGR_SS )

#define __udc_self_powered()		( REG_UDC_DevCFGR |= UDC_DevCFGR_SP )
#define __udc_bus_powered()		( REG_UDC_DevCFGR &= ~UDC_DevCFGR_SP )

#define __udc_enable_remote_wakeup()	( REG_UDC_DevCFGR |= UDC_DevCFGR_RW )
#define __udc_disable_remote_wakeup()	( REG_UDC_DevCFGR &= ~UDC_DevCFGR_RW )

#define __udc_set_speed_high()				\
do {							\
	REG_UDC_DevCFGR &= ~UDC_DevCFGR_SPD_MASK;	\
	REG_UDC_DevCFGR |= UDC_DevCFGR_SPD_HS;		\
} while (0)

#define __udc_set_speed_full()				\
do {							\
	REG_UDC_DevCFGR &= ~UDC_DevCFGR_SPD_MASK;	\
	REG_UDC_DevCFGR |= UDC_DevCFGR_SPD_FS;		\
} while (0)

#define __udc_set_speed_low()				\
do {							\
	REG_UDC_DevCFGR &= ~UDC_DevCFGR_SPD_MASK;	\
	REG_UDC_DevCFGR |= UDC_DevCFGR_SPD_LS;		\
} while (0)


#define __udc_set_dma_mode()		( REG_UDC_DevCR |= UDC_DevCR_DM )
#define __udc_set_slave_mode()		( REG_UDC_DevCR &= ~UDC_DevCR_DM )
#define __udc_set_big_endian()		( REG_UDC_DevCR |= UDC_DevCR_BE )
#define __udc_set_little_endian()	( REG_UDC_DevCR &= ~UDC_DevCR_BE )
#define __udc_generate_resume()		( REG_UDC_DevCR |= UDC_DevCR_RES )
#define __udc_clear_resume()		( REG_UDC_DevCR &= ~UDC_DevCR_RES )


#define __udc_get_enumarated_speed()	( REG_UDC_DevSR & UDC_DevSR_ENUMSPD_MASK )
#define __udc_suspend_detected()	( REG_UDC_DevSR & UDC_DevSR_SUSP )
#define __udc_get_alternate_setting()	( (REG_UDC_DevSR & UDC_DevSR_ALT_MASK) >> UDC_DevSR_ALT_BIT )
#define __udc_get_interface_number()	( (REG_UDC_DevSR & UDC_DevSR_INTF_MASK) >> UDC_DevSR_INTF_BIT )
#define __udc_get_config_number()	( (REG_UDC_DevSR & UDC_DevSR_CFG_MASK) >> UDC_DevSR_CFG_BIT )


#define __udc_sof_detected(r)		( (r) & UDC_DevIntR_SOF )
#define __udc_usb_suspend_detected(r)	( (r) & UDC_DevIntR_US )
#define __udc_usb_reset_detected(r)	( (r) & UDC_DevIntR_UR )
#define __udc_set_interface_detected(r)	( (r) & UDC_DevIntR_SI )
#define __udc_set_config_detected(r)	( (r) & UDC_DevIntR_SC )

#define __udc_clear_sof()		( REG_UDC_DevIntR |= UDC_DevIntR_SOF )
#define __udc_clear_usb_suspend()	( REG_UDC_DevIntR |= UDC_DevIntR_US )
#define __udc_clear_usb_reset()		( REG_UDC_DevIntR |= UDC_DevIntR_UR )
#define __udc_clear_set_interface()	( REG_UDC_DevIntR |= UDC_DevIntR_SI )
#define __udc_clear_set_config()	( REG_UDC_DevIntR |= UDC_DevIntR_SC )

#define __udc_mask_sof()		( REG_UDC_DevIntMR |= UDC_DevIntR_SOF )
#define __udc_mask_usb_suspend()	( REG_UDC_DevIntMR |= UDC_DevIntR_US )
#define __udc_mask_usb_reset()		( REG_UDC_DevIntMR |= UDC_DevIntR_UR )
#define __udc_mask_set_interface()	( REG_UDC_DevIntMR |= UDC_DevIntR_SI )
#define __udc_mask_set_config()		( REG_UDC_DevIntMR |= UDC_DevIntR_SC )
#define __udc_mask_all_dev_intrs() \
  ( REG_UDC_DevIntMR = UDC_DevIntR_SOF | UDC_DevIntR_US | \
      UDC_DevIntR_UR | UDC_DevIntR_SI | UDC_DevIntR_SC )

#define __udc_unmask_sof()		( REG_UDC_DevIntMR &= ~UDC_DevIntR_SOF )
#define __udc_unmask_usb_suspend()	( REG_UDC_DevIntMR &= ~UDC_DevIntR_US )
#define __udc_unmask_usb_reset()	( REG_UDC_DevIntMR &= ~UDC_DevIntR_UR )
#define __udc_unmask_set_interface()	( REG_UDC_DevIntMR &= ~UDC_DevIntR_SI )
#define __udc_unmask_set_config()	( REG_UDC_DevIntMR &= ~UDC_DevIntR_SC )
#if 0
#define __udc_unmask_all_dev_intrs() \
  ( REG_UDC_DevIntMR = ~(UDC_DevIntR_SOF | UDC_DevIntR_US | \
      UDC_DevIntR_UR | UDC_DevIntR_SI | UDC_DevIntR_SC) )
#else
#define __udc_unmask_all_dev_intrs() \
  ( REG_UDC_DevIntMR = 0x00000000 )
#endif


#define __udc_ep0out_irq_detected(epintr) \
  ( (((epintr) & UDC_EPIntR_OUTEP_MASK) >> (UDC_EPIntR_OUTEP_BIT + 0)) & 0x1 )
#define __udc_ep5out_irq_detected(epintr) \
  ( (((epintr) & UDC_EPIntR_OUTEP_MASK) >> (UDC_EPIntR_OUTEP_BIT + 5)) & 0x1 )
#define __udc_ep6out_irq_detected(epintr) \
  ( (((epintr) & UDC_EPIntR_OUTEP_MASK) >> (UDC_EPIntR_OUTEP_BIT + 6)) & 0x1 )
#define __udc_ep7out_irq_detected(epintr) \
  ( (((epintr) & UDC_EPIntR_OUTEP_MASK) >> (UDC_EPIntR_OUTEP_BIT + 7)) & 0x1 )

#define __udc_ep0in_irq_detected(epintr) \
  ( (((epintr) & UDC_EPIntR_INEP_MASK) >> (UDC_EPIntR_INEP_BIT + 0)) & 0x1 )
#define __udc_ep1in_irq_detected(epintr) \
  ( (((epintr) & UDC_EPIntR_INEP_MASK) >> (UDC_EPIntR_INEP_BIT + 1)) & 0x1 )
#define __udc_ep2in_irq_detected(epintr) \
  ( (((epintr) & UDC_EPIntR_INEP_MASK) >> (UDC_EPIntR_INEP_BIT + 2)) & 0x1 )
#define __udc_ep3in_irq_detected(epintr) \
  ( (((epintr) & UDC_EPIntR_INEP_MASK) >> (UDC_EPIntR_INEP_BIT + 3)) & 0x1 )
#define __udc_ep4in_irq_detected(epintr) \
  ( (((epintr) & UDC_EPIntR_INEP_MASK) >> (UDC_EPIntR_INEP_BIT + 4)) & 0x1 )


#define __udc_mask_ep0out_irq() \
  ( REG_UDC_EPIntMR |= (1 << (UDC_EPIntMR_OUTEP_BIT + 0)) )
#define __udc_mask_ep5out_irq() \
  ( REG_UDC_EPIntMR |= (1 << (UDC_EPIntMR_OUTEP_BIT + 5)) )
#define __udc_mask_ep6out_irq() \
  ( REG_UDC_EPIntMR |= (1 << (UDC_EPIntMR_OUTEP_BIT + 6)) )
#define __udc_mask_ep7out_irq() \
  ( REG_UDC_EPIntMR |= (1 << (UDC_EPIntMR_OUTEP_BIT + 7)) )

#define __udc_unmask_ep0out_irq() \
  ( REG_UDC_EPIntMR &= ~(1 << (UDC_EPIntMR_OUTEP_BIT + 0)) )
#define __udc_unmask_ep5out_irq() \
  ( REG_UDC_EPIntMR &= ~(1 << (UDC_EPIntMR_OUTEP_BIT + 5)) )
#define __udc_unmask_ep6out_irq() \
  ( REG_UDC_EPIntMR &= ~(1 << (UDC_EPIntMR_OUTEP_BIT + 6)) )
#define __udc_unmask_ep7out_irq() \
  ( REG_UDC_EPIntMR &= ~(1 << (UDC_EPIntMR_OUTEP_BIT + 7)) )

#define __udc_mask_ep0in_irq() \
  ( REG_UDC_EPIntMR |= (1 << (UDC_EPIntMR_INEP_BIT + 0)) )
#define __udc_mask_ep1in_irq() \
  ( REG_UDC_EPIntMR |= (1 << (UDC_EPIntMR_INEP_BIT + 1)) )
#define __udc_mask_ep2in_irq() \
  ( REG_UDC_EPIntMR |= (1 << (UDC_EPIntMR_INEP_BIT + 2)) )
#define __udc_mask_ep3in_irq() \
  ( REG_UDC_EPIntMR |= (1 << (UDC_EPIntMR_INEP_BIT + 3)) )
#define __udc_mask_ep4in_irq() \
  ( REG_UDC_EPIntMR |= (1 << (UDC_EPIntMR_INEP_BIT + 4)) )

#define __udc_unmask_ep0in_irq() \
  ( REG_UDC_EPIntMR &= ~(1 << (UDC_EPIntMR_INEP_BIT + 0)) )
#define __udc_unmask_ep1in_irq() \
  ( REG_UDC_EPIntMR &= ~(1 << (UDC_EPIntMR_INEP_BIT + 1)) )
#define __udc_unmask_ep2in_irq() \
  ( REG_UDC_EPIntMR &= ~(1 << (UDC_EPIntMR_INEP_BIT + 2)) )
#define __udc_unmask_ep3in_irq() \
  ( REG_UDC_EPIntMR &= ~(1 << (UDC_EPIntMR_INEP_BIT + 3)) )
#define __udc_unmask_ep4in_irq() \
  ( REG_UDC_EPIntMR &= ~(1 << (UDC_EPIntMR_INEP_BIT + 4)) )

#define __udc_mask_all_ep_intrs() \
  ( REG_UDC_EPIntMR = 0xffffffff )
#define __udc_unmask_all_ep_intrs() \
  ( REG_UDC_EPIntMR = 0x00000000 )


/* ep0 only CTRL, ep1 only INTR, ep2/3/5/6 only BULK, ep4/7 only ISO */
#define __udc_config_endpoint_type()						\
do {										\
  REG_UDC_EP0InCR = (REG_UDC_EP0InCR & ~UDC_EPCR_ET_MASK) | UDC_EPCR_ET_CTRL;	\
  REG_UDC_EP0OutCR = (REG_UDC_EP0OutCR & ~UDC_EPCR_ET_MASK) | UDC_EPCR_ET_CTRL;	\
  REG_UDC_EP1InCR = (REG_UDC_EP1InCR & ~UDC_EPCR_ET_MASK) | UDC_EPCR_ET_INTR;	\
  REG_UDC_EP2InCR = (REG_UDC_EP2InCR & ~UDC_EPCR_ET_MASK) | UDC_EPCR_ET_BULK;	\
  REG_UDC_EP3InCR = (REG_UDC_EP3InCR & ~UDC_EPCR_ET_MASK) | UDC_EPCR_ET_BULK;	\
  REG_UDC_EP4InCR = (REG_UDC_EP4InCR & ~UDC_EPCR_ET_MASK) | UDC_EPCR_ET_ISO;	\
  REG_UDC_EP5OutCR = (REG_UDC_EP5OutCR & ~UDC_EPCR_ET_MASK) | UDC_EPCR_ET_BULK;	\
  REG_UDC_EP6OutCR = (REG_UDC_EP6OutCR & ~UDC_EPCR_ET_MASK) | UDC_EPCR_ET_BULK;	\
  REG_UDC_EP7OutCR = (REG_UDC_EP7OutCR & ~UDC_EPCR_ET_MASK) | UDC_EPCR_ET_ISO;	\
} while (0)

#define __udc_enable_ep0out_snoop_mode()  ( REG_UDC_EP0OutCR |= UDC_EPCR_SN )
#define __udc_enable_ep5out_snoop_mode()  ( REG_UDC_EP5OutCR |= UDC_EPCR_SN )
#define __udc_enable_ep6out_snoop_mode()  ( REG_UDC_EP6OutCR |= UDC_EPCR_SN )
#define __udc_enable_ep7out_snoop_mode()  ( REG_UDC_EP7OutCR |= UDC_EPCR_SN )

#define __udc_disable_ep0out_snoop_mode() ( REG_UDC_EP0OutCR &= ~UDC_EPCR_SN )
#define __udc_disable_ep5out_snoop_mode() ( REG_UDC_EP5OutCR &= ~UDC_EPCR_SN )
#define __udc_disable_ep6out_snoop_mode() ( REG_UDC_EP6OutCR &= ~UDC_EPCR_SN )
#define __udc_disable_ep7out_snoop_mode() ( REG_UDC_EP7OutCR &= ~UDC_EPCR_SN )

#define __udc_flush_ep0in_fifo()  ( REG_UDC_EP0InCR |= UDC_EPCR_F )
#define __udc_flush_ep1in_fifo()  ( REG_UDC_EP1InCR |= UDC_EPCR_F )
#define __udc_flush_ep2in_fifo()  ( REG_UDC_EP2InCR |= UDC_EPCR_F )
#define __udc_flush_ep3in_fifo()  ( REG_UDC_EP3InCR |= UDC_EPCR_F )
#define __udc_flush_ep4in_fifo()  ( REG_UDC_EP4InCR |= UDC_EPCR_F )

#define __udc_unflush_ep0in_fifo()  ( REG_UDC_EP0InCR &= ~UDC_EPCR_F )
#define __udc_unflush_ep1in_fifo()  ( REG_UDC_EP1InCR &= ~UDC_EPCR_F )
#define __udc_unflush_ep2in_fifo()  ( REG_UDC_EP2InCR &= ~UDC_EPCR_F )
#define __udc_unflush_ep3in_fifo()  ( REG_UDC_EP3InCR &= ~UDC_EPCR_F )
#define __udc_unflush_ep4in_fifo()  ( REG_UDC_EP4InCR &= ~UDC_EPCR_F )

#define __udc_enable_ep0in_stall()  ( REG_UDC_EP0InCR |= UDC_EPCR_S )
#define __udc_enable_ep0out_stall() ( REG_UDC_EP0OutCR |= UDC_EPCR_S )
#define __udc_enable_ep1in_stall()  ( REG_UDC_EP1InCR |= UDC_EPCR_S )
#define __udc_enable_ep2in_stall()  ( REG_UDC_EP2InCR |= UDC_EPCR_S )
#define __udc_enable_ep3in_stall()  ( REG_UDC_EP3InCR |= UDC_EPCR_S )
#define __udc_enable_ep4in_stall()  ( REG_UDC_EP4InCR |= UDC_EPCR_S )
#define __udc_enable_ep5out_stall() ( REG_UDC_EP5OutCR |= UDC_EPCR_S )
#define __udc_enable_ep6out_stall() ( REG_UDC_EP6OutCR |= UDC_EPCR_S )
#define __udc_enable_ep7out_stall() ( REG_UDC_EP7OutCR |= UDC_EPCR_S )

#define __udc_disable_ep0in_stall()  ( REG_UDC_EP0InCR &= ~UDC_EPCR_S )
#define __udc_disable_ep0out_stall() ( REG_UDC_EP0OutCR &= ~UDC_EPCR_S )
#define __udc_disable_ep1in_stall()  ( REG_UDC_EP1InCR &= ~UDC_EPCR_S )
#define __udc_disable_ep2in_stall()  ( REG_UDC_EP2InCR &= ~UDC_EPCR_S )
#define __udc_disable_ep3in_stall()  ( REG_UDC_EP3InCR &= ~UDC_EPCR_S )
#define __udc_disable_ep4in_stall()  ( REG_UDC_EP4InCR &= ~UDC_EPCR_S )
#define __udc_disable_ep5out_stall() ( REG_UDC_EP5OutCR &= ~UDC_EPCR_S )
#define __udc_disable_ep6out_stall() ( REG_UDC_EP6OutCR &= ~UDC_EPCR_S )
#define __udc_disable_ep7out_stall() ( REG_UDC_EP7OutCR &= ~UDC_EPCR_S )


#define __udc_ep0out_packet_size() \
  ( (REG_UDC_EP0OutSR & UDC_EPSR_RXPKTSIZE_MASK) >> UDC_EPSR_RXPKTSIZE_BIT )
#define __udc_ep5out_packet_size() \
  ( (REG_UDC_EP5OutSR & UDC_EPSR_RXPKTSIZE_MASK) >> UDC_EPSR_RXPKTSIZE_BIT )
#define __udc_ep6out_packet_size() \
  ( (REG_UDC_EP6OutSR & UDC_EPSR_RXPKTSIZE_MASK) >> UDC_EPSR_RXPKTSIZE_BIT )
#define __udc_ep7out_packet_size() \
  ( (REG_UDC_EP7OutSR & UDC_EPSR_RXPKTSIZE_MASK) >> UDC_EPSR_RXPKTSIZE_BIT )

#define __udc_ep0in_received_intoken()   ( (REG_UDC_EP0InSR & UDC_EPSR_IN) )
#define __udc_ep1in_received_intoken()   ( (REG_UDC_EP1InSR & UDC_EPSR_IN) )
#define __udc_ep2in_received_intoken()   ( (REG_UDC_EP2InSR & UDC_EPSR_IN) )
#define __udc_ep3in_received_intoken()   ( (REG_UDC_EP3InSR & UDC_EPSR_IN) )
#define __udc_ep4in_received_intoken()   ( (REG_UDC_EP4InSR & UDC_EPSR_IN) )

#define __udc_ep0out_received_none() \
  ( (REG_UDC_EP0OutSR & UDC_EPSR_OUT_MASK) == UDC_EPSR_OUT_NONE )
#define __udc_ep0out_received_data() \
  ( (REG_UDC_EP0OutSR & UDC_EPSR_OUT_MASK) == UDC_EPSR_OUT_RCVDATA )
#define __udc_ep0out_received_setup() \
  ( (REG_UDC_EP0OutSR & UDC_EPSR_OUT_MASK) == UDC_EPSR_OUT_RCVSETUP )

#define __udc_ep5out_received_none() \
  ( (REG_UDC_EP5OutSR & UDC_EPSR_OUT_MASK) == UDC_EPSR_OUT_NONE )
#define __udc_ep5out_received_data() \
  ( (REG_UDC_EP5OutSR & UDC_EPSR_OUT_MASK) == UDC_EPSR_OUT_RCVDATA )
#define __udc_ep5out_received_setup() \
  ( (REG_UDC_EP5OutSR & UDC_EPSR_OUT_MASK) == UDC_EPSR_OUT_RCVSETUP )

#define __udc_ep6out_received_none() \
  ( (REG_UDC_EP6OutSR & UDC_EPSR_OUT_MASK) == UDC_EPSR_OUT_NONE )
#define __udc_ep6out_received_data() \
  ( (REG_UDC_EP6OutSR & UDC_EPSR_OUT_MASK) == UDC_EPSR_OUT_RCVDATA )
#define __udc_ep6out_received_setup() \
  ( (REG_UDC_EP6OutSR & UDC_EPSR_OUT_MASK) == UDC_EPSR_OUT_RCVSETUP )

#define __udc_ep7out_received_none() \
  ( (REG_UDC_EP7OutSR & UDC_EPSR_OUT_MASK) == UDC_EPSR_OUT_NONE )
#define __udc_ep7out_received_data() \
  ( (REG_UDC_EP7OutSR & UDC_EPSR_OUT_MASK) == UDC_EPSR_OUT_RCVDATA )
#define __udc_ep7out_received_setup() \
  ( (REG_UDC_EP7OutSR & UDC_EPSR_OUT_MASK) == UDC_EPSR_OUT_RCVSETUP )

/* ep7out ISO only */
#define __udc_ep7out_get_pid() \
  ( (REG_UDC_EP7OutSR & UDC_EPSR_PID_MASK) >> UDC_EPSR_PID_BIT )


#define __udc_ep0in_set_buffer_size(n) ( REG_UDC_EP0InBSR = (n) )
#define __udc_ep1in_set_buffer_size(n) ( REG_UDC_EP1InBSR = (n) )
#define __udc_ep2in_set_buffer_size(n) ( REG_UDC_EP2InBSR = (n) )
#define __udc_ep3in_set_buffer_size(n) ( REG_UDC_EP3InBSR = (n) )
#define __udc_ep4in_set_buffer_size(n) ( REG_UDC_EP4InBSR = (n) )

#define __udc_ep0out_get_frame_number(n) ( UDC_EP0OutPFNR )
#define __udc_ep5out_get_frame_number(n) ( UDC_EP5OutPFNR )
#define __udc_ep6out_get_frame_number(n) ( UDC_EP6OutPFNR )
#define __udc_ep7out_get_frame_number(n) ( UDC_EP7OutPFNR )


#define __udc_ep0in_set_max_packet_size(n)  ( REG_UDC_EP0InMPSR = (n) )
#define __udc_ep0out_set_max_packet_size(n) ( REG_UDC_EP0OutMPSR = (n) )
#define __udc_ep1in_set_max_packet_size(n)  ( REG_UDC_EP1InMPSR = (n) )
#define __udc_ep2in_set_max_packet_size(n)  ( REG_UDC_EP2InMPSR = (n) )
#define __udc_ep3in_set_max_packet_size(n)  ( REG_UDC_EP3InMPSR = (n) )
#define __udc_ep4in_set_max_packet_size(n)  ( REG_UDC_EP4InMPSR = (n) )
#define __udc_ep5out_set_max_packet_size(n) ( REG_UDC_EP5OutMPSR = (n) )
#define __udc_ep6out_set_max_packet_size(n) ( REG_UDC_EP6OutMPSR = (n) )
#define __udc_ep7out_set_max_packet_size(n) ( REG_UDC_EP7OutMPSR = (n) )

/* set to 0xFFFF for UDC */
#define __udc_set_setup_command_address(n)  ( REG_UDC_STCMAR = (n) )

/* Init and configure EPxInfR(x=0,1,2,3,4,5,6,7)
 * c: Configuration number to which this endpoint belongs
 * i: Interface number to which this endpoint belongs
 * a: Alternate setting to which this endpoint belongs
 * p: max Packet size of this endpoint
 */

#define __udc_ep0info_init(c,i,a,p) 			\
do { 							\
  REG_UDC_EP0InfR &= ~UDC_EPInfR_MPS_MASK; 		\
  REG_UDC_EP0InfR |= ((p) << UDC_EPInfR_MPS_BIT); 	\
  REG_UDC_EP0InfR &= ~UDC_EPInfR_ALTS_MASK; 		\
  REG_UDC_EP0InfR |= ((a) << UDC_EPInfR_ALTS_BIT); 	\
  REG_UDC_EP0InfR &= ~UDC_EPInfR_IFN_MASK; 		\
  REG_UDC_EP0InfR |= ((i) << UDC_EPInfR_IFN_BIT); 	\
  REG_UDC_EP0InfR &= ~UDC_EPInfR_CGN_MASK; 		\
  REG_UDC_EP0InfR |= ((c) << UDC_EPInfR_CGN_BIT); 	\
  REG_UDC_EP0InfR &= ~UDC_EPInfR_EPT_MASK; 		\
  REG_UDC_EP0InfR |= UDC_EPInfR_EPT_CTRL; 		\
  REG_UDC_EP0InfR &= ~UDC_EPInfR_EPD; 			\
  REG_UDC_EP0InfR |= UDC_EPInfR_EPD_OUT; 		\
  REG_UDC_EP0InfR &= ~UDC_EPInfR_EPN_MASK;		\
  REG_UDC_EP0InfR |= (0 << UDC_EPInfR_EPN_BIT);		\
} while (0)

#define __udc_ep1info_init(c,i,a,p) 			\
do { 							\
  REG_UDC_EP1InfR &= ~UDC_EPInfR_MPS_MASK; 		\
  REG_UDC_EP1InfR |= ((p) << UDC_EPInfR_MPS_BIT); 	\
  REG_UDC_EP1InfR &= ~UDC_EPInfR_ALTS_MASK; 		\
  REG_UDC_EP1InfR |= ((a) << UDC_EPInfR_ALTS_BIT); 	\
  REG_UDC_EP1InfR &= ~UDC_EPInfR_IFN_MASK; 		\
  REG_UDC_EP1InfR |= ((i) << UDC_EPInfR_IFN_BIT); 	\
  REG_UDC_EP1InfR &= ~UDC_EPInfR_CGN_MASK; 		\
  REG_UDC_EP1InfR |= ((c) << UDC_EPInfR_CGN_BIT); 	\
  REG_UDC_EP1InfR &= ~UDC_EPInfR_EPT_MASK; 		\
  REG_UDC_EP1InfR |= UDC_EPInfR_EPT_INTR; 		\
  REG_UDC_EP1InfR &= ~UDC_EPInfR_EPD; 			\
  REG_UDC_EP1InfR |= UDC_EPInfR_EPD_IN; 		\
  REG_UDC_EP1InfR &= ~UDC_EPInfR_EPN_MASK;		\
  REG_UDC_EP1InfR |= (1 << UDC_EPInfR_EPN_BIT);		\
} while (0)

#define __udc_ep2info_init(c,i,a,p) 			\
do { 							\
  REG_UDC_EP2InfR &= ~UDC_EPInfR_MPS_MASK; 		\
  REG_UDC_EP2InfR |= ((p) << UDC_EPInfR_MPS_BIT); 	\
  REG_UDC_EP2InfR &= ~UDC_EPInfR_ALTS_MASK; 		\
  REG_UDC_EP2InfR |= ((a) << UDC_EPInfR_ALTS_BIT); 	\
  REG_UDC_EP2InfR &= ~UDC_EPInfR_IFN_MASK; 		\
  REG_UDC_EP2InfR |= ((i) << UDC_EPInfR_IFN_BIT); 	\
  REG_UDC_EP2InfR &= ~UDC_EPInfR_CGN_MASK; 		\
  REG_UDC_EP2InfR |= ((c) << UDC_EPInfR_CGN_BIT); 	\
  REG_UDC_EP2InfR &= ~UDC_EPInfR_EPT_MASK; 		\
  REG_UDC_EP2InfR |= UDC_EPInfR_EPT_BULK; 		\
  REG_UDC_EP2InfR &= ~UDC_EPInfR_EPD; 			\
  REG_UDC_EP2InfR |= UDC_EPInfR_EPD_IN; 		\
  REG_UDC_EP2InfR &= ~UDC_EPInfR_EPN_MASK;		\
  REG_UDC_EP2InfR |= (2 << UDC_EPInfR_EPN_BIT);		\
} while (0)

#define __udc_ep3info_init(c,i,a,p) 			\
do { 							\
  REG_UDC_EP3InfR &= ~UDC_EPInfR_MPS_MASK; 		\
  REG_UDC_EP3InfR |= ((p) << UDC_EPInfR_MPS_BIT); 	\
  REG_UDC_EP3InfR &= ~UDC_EPInfR_ALTS_MASK; 		\
  REG_UDC_EP3InfR |= ((a) << UDC_EPInfR_ALTS_BIT); 	\
  REG_UDC_EP3InfR &= ~UDC_EPInfR_IFN_MASK; 		\
  REG_UDC_EP3InfR |= ((i) << UDC_EPInfR_IFN_BIT); 	\
  REG_UDC_EP3InfR &= ~UDC_EPInfR_CGN_MASK; 		\
  REG_UDC_EP3InfR |= ((c) << UDC_EPInfR_CGN_BIT); 	\
  REG_UDC_EP3InfR &= ~UDC_EPInfR_EPT_MASK; 		\
  REG_UDC_EP3InfR |= UDC_EPInfR_EPT_BULK; 		\
  REG_UDC_EP3InfR &= ~UDC_EPInfR_EPD; 			\
  REG_UDC_EP3InfR |= UDC_EPInfR_EPD_IN; 		\
  REG_UDC_EP3InfR &= ~UDC_EPInfR_EPN_MASK;		\
  REG_UDC_EP3InfR |= (3 << UDC_EPInfR_EPN_BIT);		\
} while (0)

#define __udc_ep4info_init(c,i,a,p) 			\
do { 							\
  REG_UDC_EP4InfR &= ~UDC_EPInfR_MPS_MASK; 		\
  REG_UDC_EP4InfR |= ((p) << UDC_EPInfR_MPS_BIT); 	\
  REG_UDC_EP4InfR &= ~UDC_EPInfR_ALTS_MASK; 		\
  REG_UDC_EP4InfR |= ((a) << UDC_EPInfR_ALTS_BIT); 	\
  REG_UDC_EP4InfR &= ~UDC_EPInfR_IFN_MASK; 		\
  REG_UDC_EP4InfR |= ((i) << UDC_EPInfR_IFN_BIT); 	\
  REG_UDC_EP4InfR &= ~UDC_EPInfR_CGN_MASK; 		\
  REG_UDC_EP4InfR |= ((c) << UDC_EPInfR_CGN_BIT); 	\
  REG_UDC_EP4InfR &= ~UDC_EPInfR_EPT_MASK; 		\
  REG_UDC_EP4InfR |= UDC_EPInfR_EPT_ISO; 		\
  REG_UDC_EP4InfR &= ~UDC_EPInfR_EPD; 			\
  REG_UDC_EP4InfR |= UDC_EPInfR_EPD_IN; 		\
  REG_UDC_EP4InfR &= ~UDC_EPInfR_EPN_MASK;		\
  REG_UDC_EP4InfR |= (4 << UDC_EPInfR_EPN_BIT);		\
} while (0)

#define __udc_ep5info_init(c,i,a,p) 			\
do { 							\
  REG_UDC_EP5InfR &= ~UDC_EPInfR_MPS_MASK; 		\
  REG_UDC_EP5InfR |= ((p) << UDC_EPInfR_MPS_BIT); 	\
  REG_UDC_EP5InfR &= ~UDC_EPInfR_ALTS_MASK; 		\
  REG_UDC_EP5InfR |= ((a) << UDC_EPInfR_ALTS_BIT); 	\
  REG_UDC_EP5InfR &= ~UDC_EPInfR_IFN_MASK; 		\
  REG_UDC_EP5InfR |= ((i) << UDC_EPInfR_IFN_BIT); 	\
  REG_UDC_EP5InfR &= ~UDC_EPInfR_CGN_MASK; 		\
  REG_UDC_EP5InfR |= ((c) << UDC_EPInfR_CGN_BIT); 	\
  REG_UDC_EP5InfR &= ~UDC_EPInfR_EPT_MASK; 		\
  REG_UDC_EP5InfR |= UDC_EPInfR_EPT_BULK; 		\
  REG_UDC_EP5InfR &= ~UDC_EPInfR_EPD; 			\
  REG_UDC_EP5InfR |= UDC_EPInfR_EPD_OUT; 		\
  REG_UDC_EP5InfR &= ~UDC_EPInfR_EPN_MASK;		\
  REG_UDC_EP5InfR |= (5 << UDC_EPInfR_EPN_BIT);		\
} while (0)

#define __udc_ep6info_init(c,i,a,p) 			\
do { 							\
  REG_UDC_EP6InfR &= ~UDC_EPInfR_MPS_MASK; 		\
  REG_UDC_EP6InfR |= ((p) << UDC_EPInfR_MPS_BIT); 	\
  REG_UDC_EP6InfR &= ~UDC_EPInfR_ALTS_MASK; 		\
  REG_UDC_EP6InfR |= ((a) << UDC_EPInfR_ALTS_BIT); 	\
  REG_UDC_EP6InfR &= ~UDC_EPInfR_IFN_MASK; 		\
  REG_UDC_EP6InfR |= ((i) << UDC_EPInfR_IFN_BIT); 	\
  REG_UDC_EP6InfR &= ~UDC_EPInfR_CGN_MASK; 		\
  REG_UDC_EP6InfR |= ((c) << UDC_EPInfR_CGN_BIT); 	\
  REG_UDC_EP6InfR &= ~UDC_EPInfR_EPT_MASK; 		\
  REG_UDC_EP6InfR |= UDC_EPInfR_EPT_BULK; 		\
  REG_UDC_EP6InfR &= ~UDC_EPInfR_EPD; 			\
  REG_UDC_EP6InfR |= UDC_EPInfR_EPD_OUT; 		\
  REG_UDC_EP6InfR &= ~UDC_EPInfR_EPN_MASK;		\
  REG_UDC_EP6InfR |= (6 << UDC_EPInfR_EPN_BIT);		\
} while (0)

#define __udc_ep7info_init(c,i,a,p) 			\
do { 							\
  REG_UDC_EP7InfR &= ~UDC_EPInfR_MPS_MASK; 		\
  REG_UDC_EP7InfR |= ((p) << UDC_EPInfR_MPS_BIT); 	\
  REG_UDC_EP7InfR &= ~UDC_EPInfR_ALTS_MASK; 		\
  REG_UDC_EP7InfR |= ((a) << UDC_EPInfR_ALTS_BIT); 	\
  REG_UDC_EP7InfR &= ~UDC_EPInfR_IFN_MASK; 		\
  REG_UDC_EP7InfR |= ((i) << UDC_EPInfR_IFN_BIT); 	\
  REG_UDC_EP7InfR &= ~UDC_EPInfR_CGN_MASK; 		\
  REG_UDC_EP7InfR |= ((c) << UDC_EPInfR_CGN_BIT); 	\
  REG_UDC_EP7InfR &= ~UDC_EPInfR_EPT_MASK; 		\
  REG_UDC_EP7InfR |= UDC_EPInfR_EPT_ISO; 		\
  REG_UDC_EP7InfR &= ~UDC_EPInfR_EPD; 			\
  REG_UDC_EP7InfR |= UDC_EPInfR_EPD_OUT; 		\
  REG_UDC_EP7InfR &= ~UDC_EPInfR_EPN_MASK;		\
  REG_UDC_EP7InfR |= (7 << UDC_EPInfR_EPN_BIT);		\
} while (0)


/***************************************************************************
 * DMAC
 ***************************************************************************/

/* n is the DMA channel (0 - 7) */

#define __dmac_enable_all_channels() \
  ( REG_DMAC_DMACR |= DMAC_DMACR_DME | DMAC_DMACR_PR_ROUNDROBIN )
#define __dmac_disable_all_channels() \
  ( REG_DMAC_DMACR &= ~DMAC_DMACR_DME )

/* p=0,1,2,3 */
#define __dmac_set_priority(p) 				\
do {							\
	REG_DMAC_DMACR &= ~DMAC_DMACR_PR_MASK;		\
	REG_DMAC_DMACR |= ((p) << DMAC_DMACR_PR_BIT);	\
} while (0)

#define __dmac_test_halt_error() ( REG_DMAC_DMACR & DMAC_DMACR_HTR )
#define __dmac_test_addr_error() ( REG_DMAC_DMACR & DMAC_DMACR_AER )

#define __dmac_enable_channel(n) \
  ( REG_DMAC_DCCSR(n) |= DMAC_DCCSR_CHDE )
#define __dmac_disable_channel(n) \
  ( REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_CHDE )
#define __dmac_channel_enabled(n) \
  ( REG_DMAC_DCCSR(n) & DMAC_DCCSR_CHDE )

#define __dmac_channel_enable_irq(n) \
  ( REG_DMAC_DCCSR(n) |= DMAC_DCCSR_TCIE )
#define __dmac_channel_disable_irq(n) \
  ( REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_TCIE )

#define __dmac_channel_transmit_halt_detected(n) \
  (  REG_DMAC_DCCSR(n) & DMAC_DCCSR_HLT )
#define __dmac_channel_transmit_end_detected(n) \
  (  REG_DMAC_DCCSR(n) & DMAC_DCCSR_TC )
#define __dmac_channel_address_error_detected(n) \
  (  REG_DMAC_DCCSR(n) & DMAC_DCCSR_AR )

#define __dmac_channel_clear_transmit_halt(n) \
  (  REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_HLT )
#define __dmac_channel_clear_transmit_end(n) \
  (  REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_TC )
#define __dmac_channel_clear_address_error(n) \
  (  REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_AR )

#define __dmac_channel_set_single_mode(n) \
  (  REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_TM )
#define __dmac_channel_set_block_mode(n) \
  (  REG_DMAC_DCCSR(n) |= DMAC_DCCSR_TM )

#define __dmac_channel_set_transfer_unit_32bit(n)	\
do {							\
	REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_DS_MASK;	\
	REG_DMAC_DCCSR(n) |= DMAC_DCCSR_DS_32b;		\
} while (0)

#define __dmac_channel_set_transfer_unit_16bit(n)	\
do {							\
	REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_DS_MASK;	\
	REG_DMAC_DCCSR(n) |= DMAC_DCCSR_DS_16b;		\
} while (0)

#define __dmac_channel_set_transfer_unit_8bit(n)	\
do {							\
	REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_DS_MASK;	\
	REG_DMAC_DCCSR(n) |= DMAC_DCCSR_DS_8b;		\
} while (0)

#define __dmac_channel_set_transfer_unit_16byte(n)	\
do {							\
	REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_DS_MASK;	\
	REG_DMAC_DCCSR(n) |= DMAC_DCCSR_DS_16B;		\
} while (0)

#define __dmac_channel_set_transfer_unit_32byte(n)	\
do {							\
	REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_DS_MASK;	\
	REG_DMAC_DCCSR(n) |= DMAC_DCCSR_DS_32B;		\
} while (0)

/* w=8,16,32 */
#define __dmac_channel_set_dest_port_width(n,w)		\
do {							\
	REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_DWDH_MASK;	\
	REG_DMAC_DCCSR(n) |= DMAC_DCCSR_DWDH_##w;	\
} while (0)

/* w=8,16,32 */
#define __dmac_channel_set_src_port_width(n,w)		\
do {							\
	REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_SWDH_MASK;	\
	REG_DMAC_DCCSR(n) |= DMAC_DCCSR_SWDH_##w;	\
} while (0)

/* v=0-15 */
#define __dmac_channel_set_rdil(n,v)				\
do {								\
	REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_RDIL_MASK;		\
	REG_DMAC_DCCSR(n) |= ((v) << DMAC_DCCSR_RDIL_BIT);	\
} while (0)

#define __dmac_channel_dest_addr_fixed(n) \
  (  REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_DAM )
#define __dmac_channel_dest_addr_increment(n) \
  (  REG_DMAC_DCCSR(n) |= DMAC_DCCSR_DAM )

#define __dmac_channel_src_addr_fixed(n) \
  (  REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_SAM )
#define __dmac_channel_src_addr_increment(n) \
  (  REG_DMAC_DCCSR(n) |= DMAC_DCCSR_SAM )

#define __dmac_channel_set_eop_high(n) \
  (  REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_EOPM )
#define __dmac_channel_set_eop_low(n) \
  (  REG_DMAC_DCCSR(n) |= DMAC_DCCSR_EOPM )

#define __dmac_channel_set_erdm(n,m)				\
do {								\
	REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_SWDH_MASK;		\
	REG_DMAC_DCCSR(n) |= ((m) << DMAC_DCCSR_ERDM_BIT);	\
} while (0)

#define __dmac_channel_set_eackm(n) \
  ( REG_DMAC_DCCSR(n) |= DMAC_DCCSR_EACKM )
#define __dmac_channel_clear_eackm(n) \
  ( REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_EACKM )

#define __dmac_channel_set_eacks(n) \
  ( REG_DMAC_DCCSR(n) |= DMAC_DCCSR_EACKS )
#define __dmac_channel_clear_eacks(n) \
  ( REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_EACKS )


#define __dmac_channel_irq_detected(n) \
  ( REG_DMAC_DCCSR(n) & (DMAC_DCCSR_TC | DMAC_DCCSR_AR) )

static __inline__ int __dmac_get_irq(void)
{
	int i;
	for (i=0;i<NUM_DMA;i++)
		if (__dmac_channel_irq_detected(i))
			return i;
	return -1;
}

/***************************************************************************
 * AIC (AC'97 & I2S Controller)
 ***************************************************************************/

#define __aic_enable()		( REG_AIC_FR |= AIC_FR_ENB )
#define __aic_disable()		( REG_AIC_FR &= ~AIC_FR_ENB )
#define __aic_reset()		( REG_AIC_FR |= AIC_FR_RST )
#define __aic_select_ac97()	( REG_AIC_FR &= ~AIC_FR_AUSEL )
#define __aic_select_i2s()	( REG_AIC_FR |= AIC_FR_AUSEL )

#define __i2s_as_master()	( REG_AIC_FR |= AIC_FR_BCKD | AIC_FR_SYNCD )
#define __i2s_as_slave()	( REG_AIC_FR &= ~(AIC_FR_BCKD | AIC_FR_SYNCD) )

#define __aic_set_transmit_trigger(n) 			\
do {							\
	REG_AIC_FR &= ~AIC_FR_TFTH_MASK;		\
	REG_AIC_FR |= ((n) << AIC_FR_TFTH_BIT);		\
} while(0)

#define __aic_set_receive_trigger(n) 			\
do {							\
	REG_AIC_FR &= ~AIC_FR_RFTH_MASK;		\
	REG_AIC_FR |= ((n) << AIC_FR_RFTH_BIT);		\
} while(0)

#define __aic_enable_record()	( REG_AIC_CR |= AIC_CR_EREC )
#define __aic_disable_record()	( REG_AIC_CR &= ~AIC_CR_EREC )
#define __aic_enable_replay()	( REG_AIC_CR |= AIC_CR_ERPL )
#define __aic_disable_replay()	( REG_AIC_CR &= ~AIC_CR_ERPL )
#define __aic_enable_loopback()	( REG_AIC_CR |= AIC_CR_ENLBF )
#define __aic_disable_loopback() ( REG_AIC_CR &= ~AIC_CR_ENLBF )

#define __aic_flush_fifo()	( REG_AIC_CR |= AIC_CR_FLUSH )
#define __aic_unflush_fifo()	( REG_AIC_CR &= ~AIC_CR_FLUSH )

#define __aic_enable_transmit_intr() \
  ( REG_AIC_CR |= (AIC_CR_ETFS | AIC_CR_ETUR) )
#define __aic_disable_transmit_intr() \
  ( REG_AIC_CR &= ~(AIC_CR_ETFS | AIC_CR_ETUR) )
#define __aic_enable_receive_intr() \
  ( REG_AIC_CR |= (AIC_CR_ERFS | AIC_CR_EROR) )
#define __aic_disable_receive_intr() \
  ( REG_AIC_CR &= ~(AIC_CR_ERFS | AIC_CR_EROR) )

#define __aic_enable_transmit_dma()  ( REG_AIC_CR |= AIC_CR_TDMS )
#define __aic_disable_transmit_dma() ( REG_AIC_CR &= ~AIC_CR_TDMS )
#define __aic_enable_receive_dma()   ( REG_AIC_CR |= AIC_CR_RDMS )
#define __aic_disable_receive_dma()  ( REG_AIC_CR &= ~AIC_CR_RDMS )

#define __aic_enable_mono2stereo()
#define __aic_disable_mono2stereo()
#define __aic_enable_byteswap()
#define __aic_disable_byteswap()
#define __aic_enable_unsignadj()
#define __aic_disable_unsignadj()

#define AC97_PCM_XS_L_FRONT   	AIC_ACCR1_XS_SLOT3
#define AC97_PCM_XS_R_FRONT   	AIC_ACCR1_XS_SLOT4
#define AC97_PCM_XS_CENTER    	AIC_ACCR1_XS_SLOT6
#define AC97_PCM_XS_L_SURR    	AIC_ACCR1_XS_SLOT7
#define AC97_PCM_XS_R_SURR    	AIC_ACCR1_XS_SLOT8
#define AC97_PCM_XS_LFE       	AIC_ACCR1_XS_SLOT9

#define AC97_PCM_RS_L_FRONT   	AIC_ACCR1_RS_SLOT3
#define AC97_PCM_RS_R_FRONT   	AIC_ACCR1_RS_SLOT4
#define AC97_PCM_RS_CENTER    	AIC_ACCR1_RS_SLOT6
#define AC97_PCM_RS_L_SURR    	AIC_ACCR1_RS_SLOT7
#define AC97_PCM_RS_R_SURR    	AIC_ACCR1_RS_SLOT8
#define AC97_PCM_RS_LFE       	AIC_ACCR1_RS_SLOT9

#define __ac97_set_xs_none()	( REG_AIC_ACCR1 &= ~AIC_ACCR1_XS_MASK )
#define __ac97_set_xs_mono() 						\
do {									\
	REG_AIC_ACCR1 &= ~AIC_ACCR1_XS_MASK;				\
	REG_AIC_ACCR1 |= AC97_PCM_XS_R_FRONT;				\
} while(0)
#define __ac97_set_xs_stereo() 						\
do {									\
	REG_AIC_ACCR1 &= ~AIC_ACCR1_XS_MASK;				\
	REG_AIC_ACCR1 |= AC97_PCM_XS_L_FRONT | AC97_PCM_XS_R_FRONT;	\
} while(0)

/* In fact, only stereo is support now. */ 
#define __ac97_set_rs_none()	( REG_AIC_ACCR1 &= ~AIC_ACCR1_RS_MASK )
#define __ac97_set_rs_mono() 						\
do {									\
	REG_AIC_ACCR1 &= ~AIC_ACCR1_RS_MASK;				\
	REG_AIC_ACCR1 |= AC97_PCM_RS_R_FRONT;				\
} while(0)
#define __ac97_set_rs_stereo() 						\
do {									\
	REG_AIC_ACCR1 &= ~AIC_ACCR1_RS_MASK;				\
	REG_AIC_ACCR1 |= AC97_PCM_RS_L_FRONT | AC97_PCM_RS_R_FRONT;	\
} while(0)

#define __ac97_warm_reset_codec()		\
 do {						\
	REG_AIC_ACCR2 |= AIC_ACCR2_SA;		\
	REG_AIC_ACCR2 |= AIC_ACCR2_SS;		\
	udelay(1);				\
	REG_AIC_ACCR2 &= ~AIC_ACCR2_SS;		\
	REG_AIC_ACCR2 &= ~AIC_ACCR2_SA;		\
 } while (0)

#define Jz_AC97_RESET_BUG 1

#ifndef Jz_AC97_RESET_BUG
#define __ac97_cold_reset_codec()		\
 do {						\
	REG_AIC_ACCR2 |= AIC_ACCR2_SA;		\
	REG_AIC_ACCR2 &= ~AIC_ACCR2_SS;		\
	REG_AIC_ACCR2 |=  AIC_ACCR2_SR;		\
	udelay(1);				\
	REG_AIC_ACCR2 &= ~AIC_ACCR2_SR;		\
	REG_AIC_ACCR2 &= ~AIC_ACCR2_SA;		\
 } while (0)
#else
#define __ac97_cold_reset_codec()		\
 do {						\
        __gpio_as_output(70); /* SDATA_OUT */	\
        __gpio_as_output(71); /* SDATA_IN */	\
        __gpio_as_output(78); /* SYNC */	\
        __gpio_as_output(69); /* RESET# */	\
	__gpio_clear_pin(70);			\
	__gpio_clear_pin(71);			\
	__gpio_clear_pin(78);			\
	__gpio_clear_pin(69);			\
	udelay(10);				\
	__gpio_set_pin(69);			\
	udelay(1);				\
	__gpio_as_ac97();			\
 } while (0)
#endif

/* n=8,16,18,20 */
#define __ac97_set_iass(n) \
 ( REG_AIC_ACCR2 = (REG_AIC_ACCR2 & ~AIC_ACCR2_IASS_MASK) | AIC_ACCR2_IASS_##n##BIT )
#define __ac97_set_oass(n) \
 ( REG_AIC_ACCR2 = (REG_AIC_ACCR2 & ~AIC_ACCR2_OASS_MASK) | AIC_ACCR2_OASS_##n##BIT )

#define __i2s_select_i2s()            ( REG_AIC_I2SCR &= ~AIC_I2SCR_AMSL )
#define __i2s_select_left_justified() ( REG_AIC_I2SCR |= AIC_I2SCR_AMSL )

/* n=8,16,18,20,24 */
#define __i2s_set_sample_size(n) \
 ( REG_AIC_I2SCR = (REG_AIC_I2SCR & ~AIC_I2SCR_WL_MASK) | AIC_I2SCR_WL_##n##BIT )

#define __i2s_stop_clock()   ( REG_AIC_I2SCR |= AIC_I2SCR_STPBK )
#define __i2s_start_clock()  ( REG_AIC_I2SCR &= ~AIC_I2SCR_STPBK )

#define __aic_transmit_request()  ( REG_AIC_SR & AIC_SR_TFS )
#define __aic_receive_request()   ( REG_AIC_SR & AIC_SR_RFS )
#define __aic_transmit_underrun() ( REG_AIC_SR & AIC_SR_TUR )
#define __aic_receive_overrun()   ( REG_AIC_SR & AIC_SR_ROR )

#define __aic_clear_errors()      ( REG_AIC_SR &= ~(AIC_SR_TUR | AIC_SR_ROR) )

#define __aic_get_transmit_resident() \
  ( (REG_AIC_SR & AIC_SR_TFL_MASK) >> AIC_SR_TFL_BIT )
#define __aic_get_receive_count() \
  ( (REG_AIC_SR & AIC_SR_RFL_MASK) >> AIC_SR_RFL_BIT )

#define __ac97_command_transmitted()     ( REG_AIC_ACSR & AIC_ACSR_CADT )
#define __ac97_status_received()         ( REG_AIC_ACSR & AIC_ACSR_SADR )
#define __ac97_status_receive_timeout()  ( REG_AIC_ACSR & AIC_ACSR_RSTO )
#define __ac97_codec_is_low_power_mode() ( REG_AIC_ACSR & AIC_ACSR_CLPM )
#define __ac97_codec_is_ready()          ( REG_AIC_ACSR & AIC_ACSR_CRDY )

#define __i2s_is_busy() ( REG_AIC_I2SSR & AIC_I2SSR_BSY )

#define CODEC_READ_CMD	        (1 << 19)
#define CODEC_WRITE_CMD	        (0 << 19)
#define CODEC_REG_INDEX_BIT     12
#define CODEC_REG_INDEX_MASK	(0x7f << CODEC_REG_INDEX_BIT)	/* 18:12 */
#define CODEC_REG_DATA_BIT      4
#define CODEC_REG_DATA_MASK	(0x0ffff << 4)	/* 19:4 */

#define __ac97_out_rcmd_addr(reg) 					\
do { 									\
    REG_AIC_ACCAR = CODEC_READ_CMD | ((reg) << CODEC_REG_INDEX_BIT); 	\
} while (0)

#define __ac97_out_wcmd_addr(reg) 					\
do { 									\
    REG_AIC_ACCAR = CODEC_WRITE_CMD | ((reg) << CODEC_REG_INDEX_BIT); 	\
} while (0)

#define __ac97_out_data(value) 						\
do { 									\
    REG_AIC_ACCDR = ((value) << CODEC_REG_DATA_BIT); 			\
} while (0)

#define __ac97_in_data() \
 ( (REG_AIC_ACSDR & CODEC_REG_DATA_MASK) >> CODEC_REG_DATA_BIT )

#define __ac97_in_status_addr() \
 ( (REG_AIC_ACSAR & CODEC_REG_INDEX_MASK) >> CODEC_REG_INDEX_BIT )

#define __i2s_set_sample_rate(i2sclk, sync) \
  ( REG_AIC_I2SDIV = ((i2sclk) / (4*64)) / (sync) )

#define __aic_write_tfifo(v)  ( REG_AIC_DR = (v) )
#define __aic_read_rfifo()    ( REG_AIC_DR )

//
// Define next ops for AC97 compatible
//

#define AC97_ACSR	AIC_ACSR

#define __ac97_enable()		__aic_enable(); __aic_select_ac97()
#define __ac97_disable()	__aic_disable()
#define __ac97_reset()		__aic_reset()

#define __ac97_set_transmit_trigger(n)	__aic_set_transmit_trigger(n)
#define __ac97_set_receive_trigger(n)	__aic_set_receive_trigger(n)

#define __ac97_enable_record()		__aic_enable_record()
#define __ac97_disable_record()		__aic_disable_record()
#define __ac97_enable_replay()		__aic_enable_replay()
#define __ac97_disable_replay()		__aic_disable_replay()
#define __ac97_enable_loopback()	__aic_enable_loopback()
#define __ac97_disable_loopback()	__aic_disable_loopback()

#define __ac97_enable_transmit_dma()	__aic_enable_transmit_dma()
#define __ac97_disable_transmit_dma()	__aic_disable_transmit_dma()
#define __ac97_enable_receive_dma()	__aic_enable_receive_dma()
#define __ac97_disable_receive_dma()	__aic_disable_receive_dma()

#define __ac97_transmit_request()	__aic_transmit_request()
#define __ac97_receive_request()	__aic_receive_request()
#define __ac97_transmit_underrun()	__aic_transmit_underrun()
#define __ac97_receive_overrun()	__aic_receive_overrun()

#define __ac97_clear_errors()		__aic_clear_errors()

#define __ac97_get_transmit_resident()	__aic_get_transmit_resident()
#define __ac97_get_receive_count()	__aic_get_receive_count()

#define __ac97_enable_transmit_intr()	__aic_enable_transmit_intr()
#define __ac97_disable_transmit_intr()	__aic_disable_transmit_intr()
#define __ac97_enable_receive_intr()	__aic_enable_receive_intr()
#define __ac97_disable_receive_intr()	__aic_disable_receive_intr()

#define __ac97_write_tfifo(v)		__aic_write_tfifo(v)
#define __ac97_read_rfifo()		__aic_read_rfifo()

//
// Define next ops for I2S compatible
//

#define I2S_ACSR	AIC_I2SSR

#define __i2s_enable()		 __aic_enable(); __aic_select_i2s()
#define __i2s_disable()		__aic_disable()
#define __i2s_reset()		__aic_reset()

#define __i2s_set_transmit_trigger(n)	__aic_set_transmit_trigger(n)
#define __i2s_set_receive_trigger(n)	__aic_set_receive_trigger(n)

#define __i2s_enable_record()		__aic_enable_record()
#define __i2s_disable_record()		__aic_disable_record()
#define __i2s_enable_replay()		__aic_enable_replay()
#define __i2s_disable_replay()		__aic_disable_replay()
#define __i2s_enable_loopback()		__aic_enable_loopback()
#define __i2s_disable_loopback()	__aic_disable_loopback()

#define __i2s_enable_transmit_dma()	__aic_enable_transmit_dma()
#define __i2s_disable_transmit_dma()	__aic_disable_transmit_dma()
#define __i2s_enable_receive_dma()	__aic_enable_receive_dma()
#define __i2s_disable_receive_dma()	__aic_disable_receive_dma()

#define __i2s_transmit_request()	__aic_transmit_request()
#define __i2s_receive_request()		__aic_receive_request()
#define __i2s_transmit_underrun()	__aic_transmit_underrun()
#define __i2s_receive_overrun()		__aic_receive_overrun()

#define __i2s_clear_errors()		__aic_clear_errors()

#define __i2s_get_transmit_resident()	__aic_get_transmit_resident()
#define __i2s_get_receive_count()	__aic_get_receive_count()

#define __i2s_enable_transmit_intr()	__aic_enable_transmit_intr()
#define __i2s_disable_transmit_intr()	__aic_disable_transmit_intr()
#define __i2s_enable_receive_intr()	__aic_enable_receive_intr()
#define __i2s_disable_receive_intr()	__aic_disable_receive_intr()

#define __i2s_write_tfifo(v)		__aic_write_tfifo(v)
#define __i2s_read_rfifo()		__aic_read_rfifo()

#define __i2s_reset_codec()			\
 do {						\
        __gpio_as_output(70); /* SDATA_OUT */	\
        __gpio_as_input(71);  /* SDATA_IN */	\
        __gpio_as_output(78); /* SYNC */	\
        __gpio_as_output(69); /* RESET# */	\
	__gpio_clear_pin(70);			\
	__gpio_clear_pin(71);			\
	__gpio_clear_pin(78);			\
	__gpio_clear_pin(69);			\
        __gpio_as_i2s_master();			\
 } while (0)


/***************************************************************************
 * LCD
 ***************************************************************************/

#define __lcd_set_dis()			( REG_LCD_CTRL |= LCD_CTRL_DIS )
#define __lcd_clr_dis()			( REG_LCD_CTRL &= ~LCD_CTRL_DIS )

#define __lcd_set_ena()			( REG_LCD_CTRL |= LCD_CTRL_ENA )
#define __lcd_clr_ena()			( REG_LCD_CTRL &= ~LCD_CTRL_ENA )

/* n=1,2,4,8,16 */
#define __lcd_set_bpp(n) \
  ( REG_LCD_CTRL = (REG_LCD_CTRL & ~LCD_CTRL_BPP_MASK) | LCD_CTRL_BPP_##n )

/* n=4,8,16 */
#define __lcd_set_burst_length(n) 		\
do {						\
	REG_LCD_CTRL &= ~LCD_CTRL_BST_MASK;	\
	REG_LCD_CTRL |= LCD_CTRL_BST_n##;	\
} while (0)

#define __lcd_select_rgb565()		( REG_LCD_CTRL &= ~LCD_CTRL_RGB555 )
#define __lcd_select_rgb555()		( REG_LCD_CTRL |= LCD_CTRL_RGB555 )

#define __lcd_set_ofup()		( REG_LCD_CTRL |= LCD_CTRL_OFUP )
#define __lcd_clr_ofup()		( REG_LCD_CTRL &= ~LCD_CTRL_OFUP )

/* n=2,4,16 */
#define __lcd_set_stn_frc(n) 			\
do {						\
	REG_LCD_CTRL &= ~LCD_CTRL_FRC_MASK;	\
	REG_LCD_CTRL |= LCD_CTRL_FRC_n##;	\
} while (0)


#define __lcd_pixel_endian_little()	( REG_LCD_CTRL |= LCD_CTRL_PEDN )
#define __lcd_pixel_endian_big()	( REG_LCD_CTRL &= ~LCD_CTRL_PEDN )

#define __lcd_reverse_byte_endian()	( REG_LCD_CTRL |= LCD_CTRL_BEDN )
#define __lcd_normal_byte_endian()	( REG_LCD_CTRL &= ~LCD_CTRL_BEDN )

#define __lcd_enable_eof_intr()		( REG_LCD_CTRL |= LCD_CTRL_EOFM )
#define __lcd_disable_eof_intr()	( REG_LCD_CTRL &= ~LCD_CTRL_EOFM )

#define __lcd_enable_sof_intr()		( REG_LCD_CTRL |= LCD_CTRL_SOFM )
#define __lcd_disable_sof_intr()	( REG_LCD_CTRL &= ~LCD_CTRL_SOFM )

#define __lcd_enable_ofu_intr()		( REG_LCD_CTRL |= LCD_CTRL_OFUM )
#define __lcd_disable_ofu_intr()	( REG_LCD_CTRL &= ~LCD_CTRL_OFUM )

#define __lcd_enable_ifu0_intr()	( REG_LCD_CTRL |= LCD_CTRL_IFUM0 )
#define __lcd_disable_ifu0_intr()	( REG_LCD_CTRL &= ~LCD_CTRL_IFUM0 )

#define __lcd_enable_ifu1_intr()	( REG_LCD_CTRL |= LCD_CTRL_IFUM1 )
#define __lcd_disable_ifu1_intr()	( REG_LCD_CTRL &= ~LCD_CTRL_IFUM1 )

#define __lcd_enable_ldd_intr()		( REG_LCD_CTRL |= LCD_CTRL_LDDM )
#define __lcd_disable_ldd_intr()	( REG_LCD_CTRL &= ~LCD_CTRL_LDDM )

#define __lcd_enable_qd_intr()		( REG_LCD_CTRL |= LCD_CTRL_QDM )
#define __lcd_disable_qd_intr()		( REG_LCD_CTRL &= ~LCD_CTRL_QDM )


/* LCD status register indication */

#define __lcd_quick_disable_done()	( REG_LCD_STATE & LCD_STATE_QD )
#define __lcd_disable_done()		( REG_LCD_STATE & LCD_STATE_LDD )
#define __lcd_infifo0_underrun()	( REG_LCD_STATE & LCD_STATE_IFU0 )
#define __lcd_infifo1_underrun()	( REG_LCD_STATE & LCD_STATE_IFU1 )
#define __lcd_outfifo_underrun()	( REG_LCD_STATE & LCD_STATE_OFU )
#define __lcd_start_of_frame()		( REG_LCD_STATE & LCD_STATE_SOF )
#define __lcd_end_of_frame()		( REG_LCD_STATE & LCD_STATE_EOF )

#define __lcd_clr_outfifounderrun()	( REG_LCD_STATE &= ~LCD_STATE_OFU )
#define __lcd_clr_sof()			( REG_LCD_STATE &= ~LCD_STATE_SOF )
#define __lcd_clr_eof()			( REG_LCD_STATE &= ~LCD_STATE_EOF )

#define __lcd_panel_white()		( REG_LCD_DEV |= LCD_DEV_WHITE )
#define __lcd_panel_black()		( REG_LCD_DEV &= ~LCD_DEV_WHITE )

/* n=1,2,4,8 for single mono-STN 
 * n=4,8 for dual mono-STN
 */
#define __lcd_set_panel_datawidth(n) 		\
do { 						\
	REG_LCD_DEV &= ~LCD_DEV_PDW_MASK; 	\
	REG_LCD_DEV |= LCD_DEV_PDW_n##;		\
} while (0)

/* m=LCD_DEV_MODE_GENERUIC_TFT_xxx */
#define __lcd_set_panel_mode(m) 		\
do {						\
	REG_LCD_DEV &= ~LCD_DEV_MODE_MASK;	\
	REG_LCD_DEV |= (m);			\
} while(0)

/* n = 0-255 */
#define __lcd_disable_ac_bias()		( REG_LCD_IO = 0xff )
#define __lcd_set_ac_bias(n) 			\
do {						\
	REG_LCD_IO &= ~LCD_IO_ACB_MASK;		\
	REG_LCD_IO |= ((n) << LCD_IO_ACB_BIT);	\
} while(0)

#define __lcd_io_set_dir()		( REG_LCD_IO |= LCD_IO_DIR )
#define __lcd_io_clr_dir()		( REG_LCD_IO &= ~LCD_IO_DIR )

#define __lcd_io_set_dep()		( REG_LCD_IO |= LCD_IO_DEP )
#define __lcd_io_clr_dep()		( REG_LCD_IO &= ~LCD_IO_DEP )

#define __lcd_io_set_vsp()		( REG_LCD_IO |= LCD_IO_VSP )
#define __lcd_io_clr_vsp()		( REG_LCD_IO &= ~LCD_IO_VSP )

#define __lcd_io_set_hsp()		( REG_LCD_IO |= LCD_IO_HSP )
#define __lcd_io_clr_hsp()		( REG_LCD_IO &= ~LCD_IO_HSP )

#define __lcd_io_set_pcp()		( REG_LCD_IO |= LCD_IO_PCP )
#define __lcd_io_clr_pcp()		( REG_LCD_IO &= ~LCD_IO_PCP )

#define __lcd_vsync_get_vps() \
  ( (REG_LCD_VSYNC & LCD_VSYNC_VPS_MASK) >> LCD_VSYNC_VPS_BIT )

#define __lcd_vsync_get_vpe() \
  ( (REG_LCD_VSYNC & LCD_VSYNC_VPE_MASK) >> LCD_VSYNC_VPE_BIT )
#define __lcd_vsync_set_vpe(n) 				\
do {							\
	REG_LCD_VSYNC &= ~LCD_VSYNC_VPE_MASK;		\
	REG_LCD_VSYNC |= (n) << LCD_VSYNC_VPE_BIT;	\
} while (0)

#define __lcd_hsync_get_hps() \
  ( (REG_LCD_HSYNC & LCD_HSYNC_HPS_MASK) >> LCD_HSYNC_HPS_BIT )
#define __lcd_hsync_set_hps(n) 				\
do {							\
	REG_LCD_HSYNC &= ~LCD_HSYNC_HPS_MASK;		\
	REG_LCD_HSYNC |= (n) << LCD_HSYNC_HPS_BIT;	\
} while (0)

#define __lcd_hsync_get_hpe() \
  ( (REG_LCD_HSYNC & LCD_HSYNC_HPE_MASK) >> LCD_VSYNC_HPE_BIT )
#define __lcd_hsync_set_hpe(n) 				\
do {							\
	REG_LCD_HSYNC &= ~LCD_HSYNC_HPE_MASK;		\
	REG_LCD_HSYNC |= (n) << LCD_HSYNC_HPE_BIT;	\
} while (0)

#define __lcd_vat_get_ht() \
  ( (REG_LCD_VAT & LCD_VAT_HT_MASK) >> LCD_VAT_HT_BIT )
#define __lcd_vat_set_ht(n) 				\
do {							\
	REG_LCD_VAT &= ~LCD_VAT_HT_MASK;		\
	REG_LCD_VAT |= (n) << LCD_VAT_HT_BIT;		\
} while (0)

#define __lcd_vat_get_vt() \
  ( (REG_LCD_VAT & LCD_VAT_VT_MASK) >> LCD_VAT_VT_BIT )
#define __lcd_vat_set_vt(n) 				\
do {							\
	REG_LCD_VAT &= ~LCD_VAT_VT_MASK;		\
	REG_LCD_VAT |= (n) << LCD_VAT_VT_BIT;		\
} while (0)

#define __lcd_dah_get_hds() \
  ( (REG_LCD_DAH & LCD_DAH_HDS_MASK) >> LCD_DAH_HDS_BIT )
#define __lcd_dah_set_hds(n) 				\
do {							\
	REG_LCD_DAH &= ~LCD_DAH_HDS_MASK;		\
	REG_LCD_DAH |= (n) << LCD_DAH_HDS_BIT;		\
} while (0)

#define __lcd_dah_get_hde() \
  ( (REG_LCD_DAH & LCD_DAH_HDE_MASK) >> LCD_DAH_HDE_BIT )
#define __lcd_dah_set_hde(n) 				\
do {							\
	REG_LCD_DAH &= ~LCD_DAH_HDE_MASK;		\
	REG_LCD_DAH |= (n) << LCD_DAH_HDE_BIT;		\
} while (0)

#define __lcd_dav_get_vds() \
  ( (REG_LCD_DAV & LCD_DAV_VDS_MASK) >> LCD_DAV_VDS_BIT )
#define __lcd_dav_set_vds(n) 				\
do {							\
	REG_LCD_DAV &= ~LCD_DAV_VDS_MASK;		\
	REG_LCD_DAV |= (n) << LCD_DAV_VDS_BIT;		\
} while (0)

#define __lcd_dav_get_vde() \
  ( (REG_LCD_DAV & LCD_DAV_VDE_MASK) >> LCD_DAV_VDE_BIT )
#define __lcd_dav_set_vde(n) 				\
do {							\
	REG_LCD_DAV &= ~LCD_DAV_VDE_MASK;		\
	REG_LCD_DAV |= (n) << LCD_DAV_VDE_BIT;		\
} while (0)

#define __lcd_cmd0_set_sofint()		( REG_LCD_CMD0 |= LCD_CMD_SOFINT )
#define __lcd_cmd0_clr_sofint()		( REG_LCD_CMD0 &= ~LCD_CMD_SOFINT )
#define __lcd_cmd1_set_sofint()		( REG_LCD_CMD1 |= LCD_CMD_SOFINT )
#define __lcd_cmd1_clr_sofint()		( REG_LCD_CMD1 &= ~LCD_CMD_SOFINT )

#define __lcd_cmd0_set_eofint()		( REG_LCD_CMD0 |= LCD_CMD_EOFINT )
#define __lcd_cmd0_clr_eofint()		( REG_LCD_CMD0 &= ~LCD_CMD_EOFINT )
#define __lcd_cmd1_set_eofint()		( REG_LCD_CMD1 |= LCD_CMD_EOFINT )
#define __lcd_cmd1_clr_eofint()		( REG_LCD_CMD1 &= ~LCD_CMD_EOFINT )

#define __lcd_cmd0_set_pal()		( REG_LCD_CMD0 |= LCD_CMD_PAL )
#define __lcd_cmd0_clr_pal()		( REG_LCD_CMD0 &= ~LCD_CMD_PAL )

#define __lcd_cmd0_get_len() \
  ( (REG_LCD_CMD0 & LCD_CMD_LEN_MASK) >> LCD_CMD_LEN_BIT )
#define __lcd_cmd1_get_len() \
  ( (REG_LCD_CMD1 & LCD_CMD_LEN_MASK) >> LCD_CMD_LEN_BIT )



/***************************************************************************
 * DES
 ***************************************************************************/


/***************************************************************************
 * CPM
 ***************************************************************************/
#define __cpm_plcr1_fd() \
	((REG_CPM_PLCR1 & CPM_PLCR1_PLL1FD_MASK) >> CPM_PLCR1_PLL1FD_BIT)
#define __cpm_plcr1_rd() \
	((REG_CPM_PLCR1 & CPM_PLCR1_PLL1RD_MASK) >> CPM_PLCR1_PLL1RD_BIT)
#define __cpm_plcr1_od() \
	((REG_CPM_PLCR1 & CPM_PLCR1_PLL1OD_MASK) >> CPM_PLCR1_PLL1OD_BIT)
#define __cpm_cfcr_mfr() \
	((REG_CPM_CFCR & CPM_CFCR_MFR_MASK) >> CPM_CFCR_MFR_BIT)
#define __cpm_cfcr_pfr() \
	((REG_CPM_CFCR & CPM_CFCR_PFR_MASK) >> CPM_CFCR_PFR_BIT)
#define __cpm_cfcr_sfr() \
	((REG_CPM_CFCR & CPM_CFCR_SFR_MASK) >> CPM_CFCR_SFR_BIT)
#define __cpm_cfcr_ifr() \
	((REG_CPM_CFCR & CPM_CFCR_IFR_MASK) >> CPM_CFCR_IFR_BIT)

static __inline__ unsigned int __cpm_divisor_encode(unsigned int n)
{
	unsigned int encode[10] = {1,2,3,4,6,8,12,16,24,32};
	int i;
	for (i=0;i<10;i++)
		if (n < encode[i])
			break;
	return i;
}

#define __cpm_set_mclk_div(n) \
do { \
	REG_CPM_CFCR = (REG_CPM_CFCR & ~CPM_CFCR_MFR_MASK) | \
		       ((n) << (CPM_CFCR_MFR_BIT)); \
} while (0)

#define __cpm_set_pclk_div(n) \
do { \
	REG_CPM_CFCR = (REG_CPM_CFCR & ~CPM_CFCR_PFR_MASK) | \
		       ((n) << (CPM_CFCR_PFR_BIT)); \
} while (0)

#define __cpm_set_sclk_div(n) \
do { \
	REG_CPM_CFCR = (REG_CPM_CFCR & ~CPM_CFCR_SFR_MASK) | \
		       ((n) << (CPM_CFCR_SFR_BIT)); \
} while (0)

#define __cpm_set_iclk_div(n) \
do { \
	REG_CPM_CFCR = (REG_CPM_CFCR & ~CPM_CFCR_IFR_MASK) | \
		       ((n) << (CPM_CFCR_IFR_BIT)); \
} while (0)

#define __cpm_set_lcdclk_div(n) \
do { \
	REG_CPM_CFCR = (REG_CPM_CFCR & ~CPM_CFCR_LFR_MASK) | \
		       ((n) << (CPM_CFCR_LFR_BIT)); \
} while (0)

#define __cpm_enable_cko1()  (REG_CPM_CFCR |= CPM_CFCR_CKOEN1)
#define __cpm_enable_cko2()  (REG_CPM_CFCR |= CPM_CFCR_CKOEN2)
#define __cpm_disable_cko1()  (REG_CPM_CFCR &= ~CPM_CFCR_CKOEN1)
#define __cpm_disable_cko2()  (REG_CPM_CFCR &= ~CPM_CFCR_CKOEN2)

#define __cpm_select_msc_clk(type) \
do {                               \
  if (type == 0)                   \
    REG_CPM_CFCR &= ~CPM_CFCR_MSC; \
  else                             \
    REG_CPM_CFCR |= CPM_CFCR_MSC;  \
  REG_CPM_CFCR |= CPM_CFCR_UPE;    \
} while(0)

#define __cpm_idle_mode()					\
	(REG_CPM_LPCR = (REG_CPM_LPCR & ~CPM_LPCR_LPM_MASK) |	\
			CPM_LPCR_LPM_IDLE)
#define __cpm_sleep_mode()					\
	(REG_CPM_LPCR = (REG_CPM_LPCR & ~CPM_LPCR_LPM_MASK) |	\
			CPM_LPCR_LPM_SLEEP)
#define __cpm_hibernate_mode()					\
	(REG_CPM_LPCR = (REG_CPM_LPCR & ~CPM_LPCR_LPM_MASK) |	\
			CPM_LPCR_LPM_HIBERNATE)

#define __cpm_start_uart0() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_UART0))
#define __cpm_start_uart1() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_UART1))
#define __cpm_start_uart2() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_UART2))
#define __cpm_start_uart3() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_UART3))
#define __cpm_start_ost() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_OST))
#define __cpm_start_dmac() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_DMAC))
#define __cpm_start_uhc() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_UHC))
#define __cpm_start_lcd() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_LCD))
#define __cpm_start_i2c() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_I2C))
#define __cpm_start_aic_pclk() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_AICPCLK))
#define __cpm_start_aic_bitclk() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_AICBCLK))
#define __cpm_start_pwm0() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_PWM0))
#define __cpm_start_pwm1() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_PWM1))
#define __cpm_start_ssi() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_SSI))
#define __cpm_start_msc() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_MSC))
#define __cpm_start_scc() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_SCC))
#define __cpm_start_eth() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_ETH))
#define __cpm_start_kbc() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_KBC))
#define __cpm_start_cim() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_CIM))
#define __cpm_start_udc() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_UDC))
#define __cpm_start_uprt() \
	(REG_CPM_MSCR &= ~(1 << CPM_MSCR_MSTP_UPRT))
#define __cpm_start_all() (REG_CPM_MSCR = 0)

#define __cpm_stop_uart0() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_UART0))
#define __cpm_stop_uart1() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_UART1))
#define __cpm_stop_uart2() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_UART2))
#define __cpm_stop_uart3() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_UART3))
#define __cpm_stop_ost() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_OST))
#define __cpm_stop_dmac() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_DMAC))
#define __cpm_stop_uhc() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_UHC))
#define __cpm_stop_lcd() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_LCD))
#define __cpm_stop_i2c() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_I2C))
#define __cpm_stop_aic_pclk() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_AICPCLK))
#define __cpm_stop_aic_bitclk() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_AICBCLK))
#define __cpm_stop_pwm0() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_PWM0))
#define __cpm_stop_pwm1() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_PWM1))
#define __cpm_stop_ssi() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_SSI))
#define __cpm_stop_msc() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_MSC))
#define __cpm_stop_scc() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_SCC))
#define __cpm_stop_eth() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_ETH))
#define __cpm_stop_kbc() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_KBC))
#define __cpm_stop_cim() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_CIM))
#define __cpm_stop_udc() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_UDC))
#define __cpm_stop_uprt() \
	(REG_CPM_MSCR |= (1 << CPM_MSCR_MSTP_UPRT))
#define __cpm_stop_all() (REG_CPM_MSCR = 0xffffffff)

#define __cpm_set_pin(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	if (p == 0)				\
		REG_CPM_GSR0 |= (1 << o);	\
	else if (p == 1)			\
		REG_CPM_GSR1 |= (1 << o);	\
	else if (p == 2)			\
		REG_CPM_GSR2 |= (1 << o);	\
	else if (p == 3)			\
		REG_CPM_GSR3 |= (1 << o);	\
} while (0)

#define __cpm_clear_pin(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	if (p == 0)				\
		REG_CPM_GSR0 &= ~(1 << o);	\
	else if (p == 1)			\
		REG_CPM_GSR1 &= ~(1 << o);	\
	else if (p == 2)			\
		REG_CPM_GSR2 &= ~(1 << o);	\
	else if (p == 3)			\
		REG_CPM_GSR3 &= ~(1 << o);	\
} while (0)


/***************************************************************************
 * SSI
 ***************************************************************************/

#define __ssi_enable()    ( REG_SSI_CR0 |= SSI_CR0_SSIE )
#define __ssi_disable()   ( REG_SSI_CR0 &= ~SSI_CR0_SSIE )
#define __ssi_select_ce() ( REG_SSI_CR0 &= ~SSI_CR0_FSEL )

#define __ssi_normal_mode() ( REG_SSI_ITR &= ~SSI_ITR_IVLTM_MASK )

#define __ssi_select_ce2() 		\
do { 					\
	REG_SSI_CR0 |= SSI_CR0_FSEL; 	\
	REG_SSI_CR1 &= ~SSI_CR1_MULTS; 	\
} while (0)

#define __ssi_select_gpc() 		\
do { 					\
	REG_SSI_CR0 &= ~SSI_CR0_FSEL; 	\
	REG_SSI_CR1 |= SSI_CR1_MULTS; 	\
} while (0)

#define __ssi_enable_tx_intr() 	\
  ( REG_SSI_CR0 |= SSI_CR0_TIE | SSI_CR0_TEIE )

#define __ssi_disable_tx_intr() \
  ( REG_SSI_CR0 &= ~(SSI_CR0_TIE | SSI_CR0_TEIE) )

#define __ssi_enable_rx_intr() 	\
  ( REG_SSI_CR0 |= SSI_CR0_RIE | SSI_CR0_REIE )

#define __ssi_disable_rx_intr() \
  ( REG_SSI_CR0 &= ~(SSI_CR0_RIE | SSI_CR0_REIE) )

#define __ssi_enable_loopback()  ( REG_SSI_CR0 |= SSI_CR0_LOOP )
#define __ssi_disable_loopback() ( REG_SSI_CR0 &= ~SSI_CR0_LOOP )

#define __ssi_enable_receive()   ( REG_SSI_CR0 &= ~SSI_CR0_DISREV )
#define __ssi_disable_receive()  ( REG_SSI_CR0 |= SSI_CR0_DISREV )

#define __ssi_finish_receive() 	\
  ( REG_SSI_CR0 |= (SSI_CR0_RFINE | SSI_CR0_RFINC) )

#define __ssi_disable_recvfinish() \
  ( REG_SSI_CR0 &= ~(SSI_CR0_RFINE | SSI_CR0_RFINC) )

#define __ssi_flush_txfifo()   ( REG_SSI_CR0 |= SSI_CR0_TFLUSH )
#define __ssi_flush_rxfifo()   ( REG_SSI_CR0 |= SSI_CR0_RFLUSH )

#define __ssi_flush_fifo() \
  ( REG_SSI_CR0 |= SSI_CR0_TFLUSH | SSI_CR0_RFLUSH )

#define __ssi_finish_transmit() ( REG_SSI_CR1 &= ~SSI_CR1_UNFIN )

#define __ssi_spi_format() 					\
do { 								\
	REG_SSI_CR1 &= ~SSI_CR1_FMAT_MASK; 			\
	REG_SSI_CR1 |= SSI_CR1_FMAT_SPI; 			\
	REG_SSI_CR1 &= ~(SSI_CR1_TFVCK_MASK|SSI_CR1_TCKFI_MASK);\
	REG_SSI_CR1 |= (SSI_CR1_TFVCK_1 | SSI_CR1_TCKFI_1);	\
} while (0)

/* TI's SSP format, must clear SSI_CR1.UNFIN */
#define __ssi_ssp_format() 					\
do { 								\
	REG_SSI_CR1 &= ~(SSI_CR1_FMAT_MASK | SSI_CR1_UNFIN); 	\
	REG_SSI_CR1 |= SSI_CR1_FMAT_SSP; 			\
} while (0)

/* National's Microwire format, must clear SSI_CR0.RFINE, and set max delay */
#define __ssi_microwire_format() 				\
do { 								\
	REG_SSI_CR1 &= ~SSI_CR1_FMAT_MASK; 			\
	REG_SSI_CR1 |= SSI_CR1_FMAT_MW1; 			\
	REG_SSI_CR1 &= ~(SSI_CR1_TFVCK_MASK|SSI_CR1_TCKFI_MASK);\
	REG_SSI_CR1 |= (SSI_CR1_TFVCK_3 | SSI_CR1_TCKFI_3);	\
	REG_SSI_CR0 &= ~SSI_CR0_RFINE; 				\
} while (0)

/* CE# level (FRMHL), CE# in interval time (ITFRM),
   clock phase and polarity (PHA POL),
   interval time (SSIITR), interval characters/frame (SSIICR) */

 /* frmhl,endian,mcom,flen,pha,pol MASK */
#define SSICR1_MISC_MASK 					\
	( SSI_CR1_FRMHL_MASK | SSI_CR1_LFST | SSI_CR1_MCOM_MASK	\
	| SSI_CR1_FLEN_MASK | SSI_CR1_PHA | SSI_CR1_POL )	\

#define __ssi_spi_set_misc(frmhl,endian,flen,mcom,pha,pol)	\
do { 								\
	REG_SSI_CR1 &= ~SSICR1_MISC_MASK; 			\
	REG_SSI_CR1 |= ((frmhl) << 30) | ((endian) << 25) | 	\
		 (((mcom) - 1) << 12) | (((flen) - 2) << 4) | 	\
	         ((pha) << 1) | (pol); 				\
} while(0)

/* Transfer with MSB or LSB first */
#define __ssi_set_msb() ( REG_SSI_CR1 &= ~SSI_CR1_LFST )
#define __ssi_set_lsb() ( REG_SSI_CR1 |= SSI_CR1_LFST )

#define __ssi_set_frame_length(n) \
    REG_SSI_CR1 = (REG_SSI_CR1 & ~SSI_CR1_FLEN_MASK) | (((n) - 2) << 4) 

/* n = 1 - 16 */
#define __ssi_set_microwire_command_length(n) \
    ( REG_SSI_CR1 = ((REG_SSI_CR1 & ~SSI_CR1_MCOM_MASK) | SSI_CR1_MCOM_##n##BIT) )

/* Set the clock phase for SPI */
#define __ssi_set_spi_clock_phase(n) \
    ( REG_SSI_CR1 = ((REG_SSI_CR1 & ~SSI_CR1_PHA) | ((n&0x1) << 1 )))

/* Set the clock polarity for SPI */
#define __ssi_set_spi_clock_polarity(n) \
    ( REG_SSI_CR1 = ((REG_SSI_CR1 & ~SSI_CR1_POL) | ((n&0x1) << 0 )))

/* n = 1,4,8,14 */
#define __ssi_set_tx_trigger(n) 		\
do { 						\
	REG_SSI_CR1 &= ~SSI_CR1_TTRG_MASK; 	\
	REG_SSI_CR1 |= SSI_CR1_TTRG_##n; 	\
} while (0)

/* n = 1,4,8,14 */
#define __ssi_set_rx_trigger(n) 		\
do { 						\
	REG_SSI_CR1 &= ~SSI_CR1_RTRG_MASK; 	\
	REG_SSI_CR1 |= SSI_CR1_RTRG_##n; 	\
} while (0)

#define __ssi_get_txfifo_count() \
    ( (REG_SSI_SR & SSI_SR_TFIFONUM_MASK) >> SSI_SR_TFIFONUM_BIT )

#define __ssi_get_rxfifo_count() \
    ( (REG_SSI_SR & SSI_SR_RFIFONUM_MASK) >> SSI_SR_RFIFONUM_BIT )

#define __ssi_clear_errors() \
    ( REG_SSI_SR &= ~(SSI_SR_UNDR | SSI_SR_OVER) )

#define __ssi_transfer_end()	( REG_SSI_SR & SSI_SR_END )
#define __ssi_is_busy()		( REG_SSI_SR & SSI_SR_BUSY )

#define __ssi_txfifo_full()	( REG_SSI_SR & SSI_SR_TFF )
#define __ssi_rxfifo_empty()	( REG_SSI_SR & SSI_SR_RFE )
#define __ssi_rxfifo_noempty()	( REG_SSI_SR & SSI_SR_RFHF )
#define __ssi_rxfifo_half_full()	( REG_SSI_SR & SSI_SR_RFHF )
#define __ssi_txfifo_half_empty()	( REG_SSI_SR & SSI_SR_TFHE )
#define __ssi_underrun()	( REG_SSI_SR & SSI_SR_UNDR )
#define __ssi_overrun()	( REG_SSI_SR & SSI_SR_OVER )

#define __ssi_set_clk(dev_clk, ssi_clk) \
  ( REG_SSI_GR = (dev_clk) / (2*(ssi_clk)) - 1 )

#define __ssi_receive_data()    REG_SSI_DR
#define __ssi_transmit_data(v)  ( REG_SSI_DR = (v) )

#endif /* __ASM_JZ4730_OPS_H__ */
