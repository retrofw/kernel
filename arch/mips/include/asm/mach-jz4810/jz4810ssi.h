/*
 * linux/include/asm-mips/mach-jz4810/jz4810ssi.h
 *
 * JZ4810 SSI register definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ4810SSI_H__
#define __JZ4810SSI_H__


#define	SSI0_BASE	0xB0043000
#define	SSI1_BASE	0xB0044000
#define	SSI2_BASE	0xB0045000



/*************************************************************************
 * SSI (Synchronous Serial Interface)
 *************************************************************************/
/* n = 0, 1 (SSI0, SSI1) */
#define	SSI_DR(n)		(SSI0_BASE + 0x000 + (n)*0x2000)
#define	SSI_CR0(n)		(SSI0_BASE + 0x004 + (n)*0x2000)
#define	SSI_CR1(n)		(SSI0_BASE + 0x008 + (n)*0x2000)
#define	SSI_SR(n)		(SSI0_BASE + 0x00C + (n)*0x2000)
#define	SSI_ITR(n)		(SSI0_BASE + 0x010 + (n)*0x2000)
#define	SSI_ICR(n)		(SSI0_BASE + 0x014 + (n)*0x2000)
#define	SSI_GR(n)		(SSI0_BASE + 0x018 + (n)*0x2000)

#define	REG_SSI_DR(n)		REG32(SSI_DR(n))
#define	REG_SSI_CR0(n)		REG16(SSI_CR0(n))
#define	REG_SSI_CR1(n)		REG32(SSI_CR1(n))
#define	REG_SSI_SR(n)		REG32(SSI_SR(n))
#define	REG_SSI_ITR(n)		REG16(SSI_ITR(n))
#define	REG_SSI_ICR(n)		REG8(SSI_ICR(n))
#define	REG_SSI_GR(n)		REG16(SSI_GR(n))

/* SSI Data Register (SSI_DR) */

#define	SSI_DR_GPC_BIT		0
#define	SSI_DR_GPC_MASK		(0x1ff << SSI_DR_GPC_BIT)

#define SSI_MAX_FIFO_ENTRIES 	128 /* 128 txfifo and 128 rxfifo */

/* SSI Control Register 0 (SSI_CR0) */

#define SSI_CR0_SSIE		(1 << 15)
#define SSI_CR0_TIE		(1 << 14)
#define SSI_CR0_RIE		(1 << 13)
#define SSI_CR0_TEIE		(1 << 12)
#define SSI_CR0_REIE		(1 << 11)
#define SSI_CR0_LOOP		(1 << 10)
#define SSI_CR0_RFINE		(1 << 9)
#define SSI_CR0_RFINC		(1 << 8)
#define SSI_CR0_EACLRUN		(1 << 7) /* hardware auto clear underrun when TxFifo no empty */
#define SSI_CR0_FSEL		(1 << 6)
#define SSI_CR0_TFLUSH		(1 << 2)
#define SSI_CR0_RFLUSH		(1 << 1)
#define SSI_CR0_DISREV		(1 << 0)

/* SSI Control Register 1 (SSI_CR1) */

#define SSI_CR1_FRMHL_BIT	30
#define SSI_CR1_FRMHL_MASK	(0x3 << SSI_CR1_FRMHL_BIT)
  #define SSI_CR1_FRMHL_CELOW_CE2LOW	(0 << SSI_CR1_FRMHL_BIT) /* SSI_CE_ is low valid and SSI_CE2_ is low valid */
  #define SSI_CR1_FRMHL_CEHIGH_CE2LOW	(1 << SSI_CR1_FRMHL_BIT) /* SSI_CE_ is high valid and SSI_CE2_ is low valid */
  #define SSI_CR1_FRMHL_CELOW_CE2HIGH	(2 << SSI_CR1_FRMHL_BIT) /* SSI_CE_ is low valid  and SSI_CE2_ is high valid */
  #define SSI_CR1_FRMHL_CEHIGH_CE2HIGH	(3 << SSI_CR1_FRMHL_BIT) /* SSI_CE_ is high valid and SSI_CE2_ is high valid */
#define SSI_CR1_TFVCK_BIT	28
#define SSI_CR1_TFVCK_MASK	(0x3 << SSI_CR1_TFVCK_BIT)
  #define SSI_CR1_TFVCK_0	  (0 << SSI_CR1_TFVCK_BIT)
  #define SSI_CR1_TFVCK_1	  (1 << SSI_CR1_TFVCK_BIT)
  #define SSI_CR1_TFVCK_2	  (2 << SSI_CR1_TFVCK_BIT)
  #define SSI_CR1_TFVCK_3	  (3 << SSI_CR1_TFVCK_BIT)
#define SSI_CR1_TCKFI_BIT	26
#define SSI_CR1_TCKFI_MASK	(0x3 << SSI_CR1_TCKFI_BIT)
  #define SSI_CR1_TCKFI_0	  (0 << SSI_CR1_TCKFI_BIT)
  #define SSI_CR1_TCKFI_1	  (1 << SSI_CR1_TCKFI_BIT)
  #define SSI_CR1_TCKFI_2	  (2 << SSI_CR1_TCKFI_BIT)
  #define SSI_CR1_TCKFI_3	  (3 << SSI_CR1_TCKFI_BIT)
#define SSI_CR1_LFST		(1 << 25)
#define SSI_CR1_ITFRM		(1 << 24)
#define SSI_CR1_UNFIN		(1 << 23)
#define SSI_CR1_MULTS		(1 << 22)
#define SSI_CR1_FMAT_BIT	20
#define SSI_CR1_FMAT_MASK	(0x3 << SSI_CR1_FMAT_BIT)
  #define SSI_CR1_FMAT_SPI	  (0 << SSI_CR1_FMAT_BIT) /* Motorola¡¯s SPI format */
  #define SSI_CR1_FMAT_SSP	  (1 << SSI_CR1_FMAT_BIT) /* TI's SSP format */
  #define SSI_CR1_FMAT_MW1	  (2 << SSI_CR1_FMAT_BIT) /* National Microwire 1 format */
  #define SSI_CR1_FMAT_MW2	  (3 << SSI_CR1_FMAT_BIT) /* National Microwire 2 format */
#define SSI_CR1_TTRG_BIT	16 /* SSI1 TX trigger */
#define SSI_CR1_TTRG_MASK	(0xf << SSI_CR1_TTRG_BIT) 
#define SSI_CR1_MCOM_BIT	12
#define SSI_CR1_MCOM_MASK	(0xf << SSI_CR1_MCOM_BIT)
  #define SSI_CR1_MCOM_1BIT	  (0x0 << SSI_CR1_MCOM_BIT) /* 1-bit command selected */
  #define SSI_CR1_MCOM_2BIT	  (0x1 << SSI_CR1_MCOM_BIT) /* 2-bit command selected */
  #define SSI_CR1_MCOM_3BIT	  (0x2 << SSI_CR1_MCOM_BIT) /* 3-bit command selected */
  #define SSI_CR1_MCOM_4BIT	  (0x3 << SSI_CR1_MCOM_BIT) /* 4-bit command selected */
  #define SSI_CR1_MCOM_5BIT	  (0x4 << SSI_CR1_MCOM_BIT) /* 5-bit command selected */
  #define SSI_CR1_MCOM_6BIT	  (0x5 << SSI_CR1_MCOM_BIT) /* 6-bit command selected */
  #define SSI_CR1_MCOM_7BIT	  (0x6 << SSI_CR1_MCOM_BIT) /* 7-bit command selected */
  #define SSI_CR1_MCOM_8BIT	  (0x7 << SSI_CR1_MCOM_BIT) /* 8-bit command selected */
  #define SSI_CR1_MCOM_9BIT	  (0x8 << SSI_CR1_MCOM_BIT) /* 9-bit command selected */
  #define SSI_CR1_MCOM_10BIT	  (0x9 << SSI_CR1_MCOM_BIT) /* 10-bit command selected */
  #define SSI_CR1_MCOM_11BIT	  (0xA << SSI_CR1_MCOM_BIT) /* 11-bit command selected */
  #define SSI_CR1_MCOM_12BIT	  (0xB << SSI_CR1_MCOM_BIT) /* 12-bit command selected */
  #define SSI_CR1_MCOM_13BIT	  (0xC << SSI_CR1_MCOM_BIT) /* 13-bit command selected */
  #define SSI_CR1_MCOM_14BIT	  (0xD << SSI_CR1_MCOM_BIT) /* 14-bit command selected */
  #define SSI_CR1_MCOM_15BIT	  (0xE << SSI_CR1_MCOM_BIT) /* 15-bit command selected */
  #define SSI_CR1_MCOM_16BIT	  (0xF << SSI_CR1_MCOM_BIT) /* 16-bit command selected */
#define SSI_CR1_RTRG_BIT	8 /* SSI RX trigger */
#define SSI_CR1_RTRG_MASK	(0xf << SSI_CR1_RTRG_BIT)
#define SSI_CR1_FLEN_BIT	4
#define SSI_CR1_FLEN_MASK	(0xf << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_2BIT	  (0x0 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_3BIT	  (0x1 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_4BIT	  (0x2 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_5BIT	  (0x3 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_6BIT	  (0x4 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_7BIT	  (0x5 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_8BIT	  (0x6 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_9BIT	  (0x7 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_10BIT	  (0x8 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_11BIT	  (0x9 << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_12BIT	  (0xA << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_13BIT	  (0xB << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_14BIT	  (0xC << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_15BIT	  (0xD << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_16BIT	  (0xE << SSI_CR1_FLEN_BIT)
  #define SSI_CR1_FLEN_17BIT	  (0xF << SSI_CR1_FLEN_BIT)
#define SSI_CR1_PHA		(1 << 1)
#define SSI_CR1_POL		(1 << 0)

/* SSI Status Register (SSI_SR) */

#define SSI_SR_TFIFONUM_BIT	16
#define SSI_SR_TFIFONUM_MASK	(0xff << SSI_SR_TFIFONUM_BIT)
#define SSI_SR_RFIFONUM_BIT	8
#define SSI_SR_RFIFONUM_MASK	(0xff << SSI_SR_RFIFONUM_BIT)
#define SSI_SR_END		(1 << 7)
#define SSI_SR_BUSY		(1 << 6)
#define SSI_SR_TFF		(1 << 5)
#define SSI_SR_RFE		(1 << 4)
#define SSI_SR_TFHE		(1 << 3)
#define SSI_SR_RFHF		(1 << 2)
#define SSI_SR_UNDR		(1 << 1)
#define SSI_SR_OVER		(1 << 0)

/* SSI Interval Time Control Register (SSI_ITR) */

#define	SSI_ITR_CNTCLK		(1 << 15)
#define SSI_ITR_IVLTM_BIT	0
#define SSI_ITR_IVLTM_MASK	(0x7fff << SSI_ITR_IVLTM_BIT)


#ifndef __MIPS_ASSEMBLER

/***************************************************************************
 * SSI (Synchronous Serial Interface)
 ***************************************************************************/
/* n = 0, 1 (SSI0, SSI1) */
#define __ssi_enable(n) 	( REG_SSI_CR0(n) |= SSI_CR0_SSIE )
#define __ssi_disable(n) 	( REG_SSI_CR0(n) &= ~SSI_CR0_SSIE )
#define __ssi_select_ce(n) 	( REG_SSI_CR0(n) &= ~SSI_CR0_FSEL )

#define __ssi_normal_mode(n) ( REG_SSI_ITR(n) &= ~SSI_ITR_IVLTM_MASK )

#define __ssi_select_ce2(n) 		\
do { 					\
	REG_SSI_CR0(n) |= SSI_CR0_FSEL; 	\
	REG_SSI_CR1(n) &= ~SSI_CR1_MULTS;	\
} while (0)

#define __ssi_select_gpc(n) 			\
do { 						\
	REG_SSI_CR0(n) &= ~SSI_CR0_FSEL;	\
	REG_SSI_CR1(n) |= SSI_CR1_MULTS;	\
} while (0)

#define __ssi_underrun_auto_clear(n) 		\
do { 						\
	REG_SSI_CR0(n) |= SSI_CR0_EACLRUN; 	\
} while (0)

#define __ssi_underrun_clear_manually(n) 	\
do { 						\
	REG_SSI_CR0(n) &= ~SSI_CR0_EACLRUN; 	\
} while (0)

#define __ssi_enable_tx_intr(n)					\
	( REG_SSI_CR0(n) |= SSI_CR0_TIE | SSI_CR0_TEIE )

#define __ssi_disable_tx_intr(n)				\
	( REG_SSI_CR0(n) &= ~(SSI_CR0_TIE | SSI_CR0_TEIE) )

#define __ssi_enable_rx_intr(n)					\
	( REG_SSI_CR0(n) |= SSI_CR0_RIE | SSI_CR0_REIE )

#define __ssi_disable_rx_intr(n)				\
	( REG_SSI_CR0(n) &= ~(SSI_CR0_RIE | SSI_CR0_REIE) )

#define __ssi_enable_txfifo_half_empty_intr(n)  \
	( REG_SSI_CR0(n) |= SSI_CR0_TIE )
#define __ssi_disable_txfifo_half_empty_intr(n)	\
	( REG_SSI_CR0(n) &= ~SSI_CR0_TIE )
#define __ssi_enable_tx_error_intr(n)		\
	( REG_SSI_CR0(n) |= SSI_CR0_TEIE )
#define __ssi_disable_tx_error_intr(n)		\
	( REG_SSI_CR0(n) &= ~SSI_CR0_TEIE )
#define __ssi_enable_rxfifo_half_full_intr(n)	\
	( REG_SSI_CR0(n) |= SSI_CR0_RIE )
#define __ssi_disable_rxfifo_half_full_intr(n)  \
	( REG_SSI_CR0(n) &= ~SSI_CR0_RIE )
#define __ssi_enable_rx_error_intr(n)		\
	( REG_SSI_CR0(n) |= SSI_CR0_REIE )
#define __ssi_disable_rx_error_intr(n)		\
	( REG_SSI_CR0(n) &= ~SSI_CR0_REIE )

#define __ssi_enable_loopback(n)  ( REG_SSI_CR0(n) |= SSI_CR0_LOOP )
#define __ssi_disable_loopback(n) ( REG_SSI_CR0(n) &= ~SSI_CR0_LOOP )

#define __ssi_enable_receive(n)   ( REG_SSI_CR0(n) &= ~SSI_CR0_DISREV )
#define __ssi_disable_receive(n)  ( REG_SSI_CR0(n) |= SSI_CR0_DISREV )

#define __ssi_finish_receive(n)					\
	( REG_SSI_CR0(n) |= (SSI_CR0_RFINE | SSI_CR0_RFINC) )

#define __ssi_disable_recvfinish(n)				\
	( REG_SSI_CR0(n) &= ~(SSI_CR0_RFINE | SSI_CR0_RFINC) )

#define __ssi_flush_txfifo(n)   	( REG_SSI_CR0(n) |= SSI_CR0_TFLUSH )
#define __ssi_flush_rxfifo(n)   	( REG_SSI_CR0(n) |= SSI_CR0_RFLUSH )

#define __ssi_flush_fifo(n)					\
	( REG_SSI_CR0(n) |= SSI_CR0_TFLUSH | SSI_CR0_RFLUSH )

#define __ssi_finish_transmit(n) 	( REG_SSI_CR1(n) &= ~SSI_CR1_UNFIN )
#define __ssi_wait_transmit(n) 		( REG_SSI_CR1(n) |= SSI_CR1_UNFIN )
#define __ssi_use_busy_wait_mode(n) 	__ssi_wait_transmit(n)
#define __ssi_unset_busy_wait_mode(n) 	__ssi_finish_transmit(n)

#define __ssi_spi_format(n)						\
	do {								\
		REG_SSI_CR1(n) &= ~SSI_CR1_FMAT_MASK; 			\
		REG_SSI_CR1(n) |= SSI_CR1_FMAT_SPI;			\
		REG_SSI_CR1(n) &= ~(SSI_CR1_TFVCK_MASK|SSI_CR1_TCKFI_MASK); \
		REG_SSI_CR1(n) |= (SSI_CR1_TFVCK_1 | SSI_CR1_TCKFI_1);	\
	} while (0)

/* TI's SSP format, must clear SSI_CR1.UNFIN */
#define __ssi_ssp_format(n)						\
	do { 								\
		REG_SSI_CR1(n) &= ~(SSI_CR1_FMAT_MASK | SSI_CR1_UNFIN);	\
		REG_SSI_CR1(n) |= SSI_CR1_FMAT_SSP;			\
	} while (0)

/* National's Microwire format, must clear SSI_CR0.RFINE, and set max delay */
#define __ssi_microwire_format(n)					\
	do {								\
		REG_SSI_CR1(n) &= ~SSI_CR1_FMAT_MASK; 			\
		REG_SSI_CR1(n) |= SSI_CR1_FMAT_MW1;			\
		REG_SSI_CR1(n) &= ~(SSI_CR1_TFVCK_MASK|SSI_CR1_TCKFI_MASK); \
		REG_SSI_CR1(n) |= (SSI_CR1_TFVCK_3 | SSI_CR1_TCKFI_3);	\
		REG_SSI_CR0(n) &= ~SSI_CR0_RFINE;			\
	} while (0)

/* CE# level (FRMHL), CE# in interval time (ITFRM),
   clock phase and polarity (PHA POL),
   interval time (SSIITR), interval characters/frame (SSIICR) */

/* frmhl,endian,mcom,flen,pha,pol MASK */
#define SSICR1_MISC_MASK 					\
	( SSI_CR1_FRMHL_MASK | SSI_CR1_LFST | SSI_CR1_MCOM_MASK	\
	  | SSI_CR1_FLEN_MASK | SSI_CR1_PHA | SSI_CR1_POL )	

#define __ssi_spi_set_misc(n,frmhl,endian,flen,mcom,pha,pol)		\
	do {								\
		REG_SSI_CR1(n) &= ~SSICR1_MISC_MASK;			\
		REG_SSI_CR1(n) |= ((frmhl) << 30) | ((endian) << 25) | 	\
			(((mcom) - 1) << 12) | (((flen) - 2) << 4) | 	\
			((pha) << 1) | (pol); 				\
	} while(0)

/* Transfer with MSB or LSB first */
#define __ssi_set_msb(n) ( REG_SSI_CR1(n) &= ~SSI_CR1_LFST )
#define __ssi_set_lsb(n) ( REG_SSI_CR1(n) |= SSI_CR1_LFST )

#define __ssi_set_frame_length(n, m)					\
	REG_SSI_CR1(n) = (REG_SSI_CR1(n) & ~SSI_CR1_FLEN_MASK) | (((m) - 2) << 4) 

/* m = 1 - 16 */
#define __ssi_set_microwire_command_length(n,m)				\
	( REG_SSI_CR1(n) = ((REG_SSI_CR1(n) & ~SSI_CR1_MCOM_MASK) | SSI_CR1_MCOM_##m##BIT) )

/* Set the clock phase for SPI */
#define __ssi_set_spi_clock_phase(n, m)					\
	( REG_SSI_CR1(n) = ((REG_SSI_CR1(n) & ~SSI_CR1_PHA) | (((m)&0x1)<< 1)))

/* Set the clock polarity for SPI */
#define __ssi_set_spi_clock_polarity(n, p)				\
	( REG_SSI_CR1(n) = ((REG_SSI_CR1(n) & ~SSI_CR1_POL) | ((p)&0x1)) )

/* SSI tx trigger, m = i x 8 */
#define __ssi_set_tx_trigger(n, m)				\
	do {							\
		REG_SSI_CR1(n) &= ~SSI_CR1_TTRG_MASK;		\
		REG_SSI_CR1(n) |= ((m)/8)<<SSI_CR1_TTRG_BIT;	\
	} while (0)

/* SSI rx trigger, m = i x 8 */
#define __ssi_set_rx_trigger(n, m)				\
	do {							\
		REG_SSI_CR1(n) &= ~SSI_CR1_RTRG_MASK;		\
		REG_SSI_CR1(n) |= ((m)/8)<<SSI_CR1_RTRG_BIT;	\
	} while (0)

#define __ssi_get_txfifo_count(n)					\
	( (REG_SSI_SR(n) & SSI_SR_TFIFONUM_MASK) >> SSI_SR_TFIFONUM_BIT )

#define __ssi_get_rxfifo_count(n)					\
	( (REG_SSI_SR(n) & SSI_SR_RFIFONUM_MASK) >> SSI_SR_RFIFONUM_BIT )

#define __ssi_transfer_end(n)		( REG_SSI_SR(n) & SSI_SR_END )
#define __ssi_is_busy(n)		( REG_SSI_SR(n) & SSI_SR_BUSY )

#define __ssi_txfifo_full(n)		( REG_SSI_SR(n) & SSI_SR_TFF )
#define __ssi_rxfifo_empty(n)		( REG_SSI_SR(n) & SSI_SR_RFE )
#define __ssi_rxfifo_half_full(n)	( REG_SSI_SR(n) & SSI_SR_RFHF )
#define __ssi_txfifo_half_empty(n)	( REG_SSI_SR(n) & SSI_SR_TFHE )
#define __ssi_underrun(n)		( REG_SSI_SR(n) & SSI_SR_UNDR )
#define __ssi_overrun(n)		( REG_SSI_SR(n) & SSI_SR_OVER )
#define __ssi_clear_underrun(n)		( REG_SSI_SR(n) = ~SSI_SR_UNDR )
#define __ssi_clear_overrun(n)		( REG_SSI_SR(n) = ~SSI_SR_OVER )
#define __ssi_clear_errors(n)		( REG_SSI_SR(n) &= ~(SSI_SR_UNDR | SSI_SR_OVER) )

#define __ssi_set_clk(n, dev_clk, ssi_clk)			\
	( REG_SSI_GR(n) = (dev_clk) / (2*(ssi_clk)) - 1 )

#define __ssi_receive_data(n) 		REG_SSI_DR(n)
#define __ssi_transmit_data(n, v) 	(REG_SSI_DR(n) = (v))





#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4810SSI_H__ */

