/*
 *  linux/include/asm-mips/mach-jz4730/regs.h
 *
 *  JZ4730 registers definition.
 *
 *  Copyright (C) 2006 - 2007 Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4730_REGS_H__
#define __ASM_JZ4730_REGS_H__

#if defined(__ASSEMBLY__) || defined(__LANGUAGE_ASSEMBLY)
#define REG8(addr)	(addr)
#define REG16(addr)	(addr)
#define REG32(addr)	(addr)
#else
#define REG8(addr)	*((volatile unsigned char *)(addr))
#define REG16(addr)	*((volatile unsigned short *)(addr))
#define REG32(addr)	*((volatile unsigned int *)(addr))
#endif

#define	HARB_BASE	0xB3000000
#define	EMC_BASE	0xB3010000
#define	DMAC_BASE	0xB3020000
#define	UHC_BASE	0xB3030000
#define	UDC_BASE	0xB3040000
#define	LCD_BASE	0xB3050000
#define	CIM_BASE	0xB3060000
#define	ETH_BASE	0xB3100000
#define	NBM_BASE	0xB3F00000

#define	CPM_BASE	0xB0000000
#define	INTC_BASE	0xB0001000
#define	OST_BASE	0xB0002000
#define	RTC_BASE	0xB0003000
#define	WDT_BASE	0xB0004000
#define	GPIO_BASE	0xB0010000
#define	AIC_BASE	0xB0020000
#define	MSC_BASE	0xB0021000
#define	UART0_BASE	0xB0030000
#define	UART1_BASE	0xB0031000
#define	UART2_BASE	0xB0032000
#define	UART3_BASE	0xB0033000
#define	FIR_BASE	0xB0040000
#define	SCC_BASE	0xB0041000
#define	SCC0_BASE	0xB0041000
#define	I2C_BASE	0xB0042000
#define	SSI_BASE	0xB0043000
#define	SCC1_BASE	0xB0044000
#define	PWM0_BASE	0xB0050000
#define	PWM1_BASE	0xB0051000
#define	DES_BASE	0xB0060000
#define	UPRT_BASE	0xB0061000
#define KBC_BASE	0xB0062000




/*************************************************************************
 * MSC
 *************************************************************************/
#define	MSC_STRPCL		(MSC_BASE + 0x000)
#define	MSC_STAT		(MSC_BASE + 0x004)
#define	MSC_CLKRT		(MSC_BASE + 0x008)
#define	MSC_CMDAT		(MSC_BASE + 0x00C)
#define	MSC_RESTO		(MSC_BASE + 0x010)
#define	MSC_RDTO		(MSC_BASE + 0x014)
#define	MSC_BLKLEN		(MSC_BASE + 0x018)
#define	MSC_NOB			(MSC_BASE + 0x01C)
#define	MSC_SNOB		(MSC_BASE + 0x020)
#define	MSC_IMASK		(MSC_BASE + 0x024)
#define	MSC_IREG		(MSC_BASE + 0x028)
#define	MSC_CMD			(MSC_BASE + 0x02C)
#define	MSC_ARG			(MSC_BASE + 0x030)
#define	MSC_RES			(MSC_BASE + 0x034)
#define	MSC_RXFIFO		(MSC_BASE + 0x038)
#define	MSC_TXFIFO		(MSC_BASE + 0x03C)

#define	REG_MSC_STRPCL		REG16(MSC_STRPCL)
#define	REG_MSC_STAT		REG32(MSC_STAT)
#define	REG_MSC_CLKRT		REG16(MSC_CLKRT)
#define	REG_MSC_CMDAT		REG32(MSC_CMDAT)
#define	REG_MSC_RESTO		REG16(MSC_RESTO)
#define	REG_MSC_RDTO		REG16(MSC_RDTO)
#define	REG_MSC_BLKLEN		REG16(MSC_BLKLEN)
#define	REG_MSC_NOB		REG16(MSC_NOB)
#define	REG_MSC_SNOB		REG16(MSC_SNOB)
#define	REG_MSC_IMASK		REG16(MSC_IMASK)
#define	REG_MSC_IREG		REG16(MSC_IREG)
#define	REG_MSC_CMD		REG8(MSC_CMD)
#define	REG_MSC_ARG		REG32(MSC_ARG)
#define	REG_MSC_RES		REG16(MSC_RES)
#define	REG_MSC_RXFIFO		REG32(MSC_RXFIFO)
#define	REG_MSC_TXFIFO		REG32(MSC_TXFIFO)

/* MSC Clock and Control Register (MSC_STRPCL) */

#define MSC_STRPCL_EXIT_MULTIPLE	(1 << 7)
#define MSC_STRPCL_EXIT_TRANSFER	(1 << 6)
#define MSC_STRPCL_START_READWAIT	(1 << 5)
#define MSC_STRPCL_STOP_READWAIT	(1 << 4)
#define MSC_STRPCL_RESET		(1 << 3)
#define MSC_STRPCL_START_OP		(1 << 2)
#define MSC_STRPCL_CLOCK_CONTROL_BIT	0
#define MSC_STRPCL_CLOCK_CONTROL_MASK	(0x3 << MSC_STRPCL_CLOCK_CONTROL_BIT)
  #define MSC_STRPCL_CLOCK_CONTROL_STOP	  (0x1 << MSC_STRPCL_CLOCK_CONTROL_BIT) /* Stop MMC/SD clock */
  #define MSC_STRPCL_CLOCK_CONTROL_START  (0x2 << MSC_STRPCL_CLOCK_CONTROL_BIT) /* Start MMC/SD clock */

/* MSC Status Register (MSC_STAT) */

#define MSC_STAT_IS_RESETTING		(1 << 15)
#define MSC_STAT_SDIO_INT_ACTIVE	(1 << 14)
#define MSC_STAT_PRG_DONE		(1 << 13)
#define MSC_STAT_DATA_TRAN_DONE		(1 << 12)
#define MSC_STAT_END_CMD_RES		(1 << 11)
#define MSC_STAT_DATA_FIFO_AFULL	(1 << 10)
#define MSC_STAT_IS_READWAIT		(1 << 9)
#define MSC_STAT_CLK_EN			(1 << 8)
#define MSC_STAT_DATA_FIFO_FULL		(1 << 7)
#define MSC_STAT_DATA_FIFO_EMPTY	(1 << 6)
#define MSC_STAT_CRC_RES_ERR		(1 << 5)
#define MSC_STAT_CRC_READ_ERROR		(1 << 4)
#define MSC_STAT_CRC_WRITE_ERROR_BIT	2
#define MSC_STAT_CRC_WRITE_ERROR_MASK	(0x3 << MSC_STAT_CRC_WRITE_ERROR_BIT)
  #define MSC_STAT_CRC_WRITE_ERROR_NO		(0 << MSC_STAT_CRC_WRITE_ERROR_BIT) /* No error on transmission of data */
  #define MSC_STAT_CRC_WRITE_ERROR		(1 << MSC_STAT_CRC_WRITE_ERROR_BIT) /* Card observed erroneous transmission of data */
  #define MSC_STAT_CRC_WRITE_ERROR_NOSTS	(2 << MSC_STAT_CRC_WRITE_ERROR_BIT) /* No CRC status is sent back */
#define MSC_STAT_TIME_OUT_RES		(1 << 1)
#define MSC_STAT_TIME_OUT_READ		(1 << 0)

/* MSC Bus Clock Control Register (MSC_CLKRT) */

#define	MSC_CLKRT_CLK_RATE_BIT		0
#define	MSC_CLKRT_CLK_RATE_MASK		(0x7 << MSC_CLKRT_CLK_RATE_BIT)
  #define MSC_CLKRT_CLK_RATE_DIV_1	  (0x0 << MSC_CLKRT_CLK_RATE_BIT) /* CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_2	  (0x1 << MSC_CLKRT_CLK_RATE_BIT) /* 1/2 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_4	  (0x2 << MSC_CLKRT_CLK_RATE_BIT) /* 1/4 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_8	  (0x3 << MSC_CLKRT_CLK_RATE_BIT) /* 1/8 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_16	  (0x4 << MSC_CLKRT_CLK_RATE_BIT) /* 1/16 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_32	  (0x5 << MSC_CLKRT_CLK_RATE_BIT) /* 1/32 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_64	  (0x6 << MSC_CLKRT_CLK_RATE_BIT) /* 1/64 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_128	  (0x7 << MSC_CLKRT_CLK_RATE_BIT) /* 1/128 of CLK_SRC */

/* MSC Command Sequence Control Register (MSC_CMDAT) */

#define	MSC_CMDAT_IO_ABORT		(1 << 11)
#define	MSC_CMDAT_BUS_WIDTH_BIT		9
#define	MSC_CMDAT_BUS_WIDTH_MASK	(0x3 << MSC_CMDAT_BUS_WIDTH_BIT)
  #define MSC_CMDAT_BUS_WIDTH_1BIT	  (0x0 << MSC_CMDAT_BUS_WIDTH_BIT) /* 1-bit data bus */
  #define MSC_CMDAT_BUS_WIDTH_4BIT	  (0x2 << MSC_CMDAT_BUS_WIDTH_BIT) /* 4-bit data bus */
  #define CMDAT_BUS_WIDTH1	  (0x0 << MSC_CMDAT_BUS_WIDTH_BIT)
  #define CMDAT_BUS_WIDTH4	  (0x2 << MSC_CMDAT_BUS_WIDTH_BIT)
#define	MSC_CMDAT_DMA_EN		(1 << 8)
#define	MSC_CMDAT_INIT			(1 << 7)
#define	MSC_CMDAT_BUSY			(1 << 6)
#define	MSC_CMDAT_STREAM_BLOCK		(1 << 5)
#define	MSC_CMDAT_WRITE			(1 << 4)
#define	MSC_CMDAT_READ			(0 << 4)
#define	MSC_CMDAT_DATA_EN		(1 << 3)
#define	MSC_CMDAT_RESPONSE_BIT	0
#define	MSC_CMDAT_RESPONSE_MASK	(0x7 << MSC_CMDAT_RESPONSE_BIT)
  #define MSC_CMDAT_RESPONSE_NONE  (0x0 << MSC_CMDAT_RESPONSE_BIT) /* No response */
  #define MSC_CMDAT_RESPONSE_R1	  (0x1 << MSC_CMDAT_RESPONSE_BIT) /* Format R1 and R1b */
  #define MSC_CMDAT_RESPONSE_R2	  (0x2 << MSC_CMDAT_RESPONSE_BIT) /* Format R2 */
  #define MSC_CMDAT_RESPONSE_R3	  (0x3 << MSC_CMDAT_RESPONSE_BIT) /* Format R3 */
  #define MSC_CMDAT_RESPONSE_R4	  (0x4 << MSC_CMDAT_RESPONSE_BIT) /* Format R4 */
  #define MSC_CMDAT_RESPONSE_R5	  (0x5 << MSC_CMDAT_RESPONSE_BIT) /* Format R5 */
  #define MSC_CMDAT_RESPONSE_R6	  (0x6 << MSC_CMDAT_RESPONSE_BIT) /* Format R6 */

#define	CMDAT_DMA_EN	(1 << 8)
#define	CMDAT_INIT	(1 << 7)
#define	CMDAT_BUSY	(1 << 6)
#define	CMDAT_STREAM	(1 << 5)
#define	CMDAT_WRITE	(1 << 4)
#define	CMDAT_DATA_EN	(1 << 3)

/* MSC Interrupts Mask Register (MSC_IMASK) */

#define	MSC_IMASK_SDIO			(1 << 7)
#define	MSC_IMASK_TXFIFO_WR_REQ		(1 << 6)
#define	MSC_IMASK_RXFIFO_RD_REQ		(1 << 5)
#define	MSC_IMASK_END_CMD_RES		(1 << 2)
#define	MSC_IMASK_PRG_DONE		(1 << 1)
#define	MSC_IMASK_DATA_TRAN_DONE	(1 << 0)


/* MSC Interrupts Status Register (MSC_IREG) */

#define	MSC_IREG_SDIO			(1 << 7)
#define	MSC_IREG_TXFIFO_WR_REQ		(1 << 6)
#define	MSC_IREG_RXFIFO_RD_REQ		(1 << 5)
#define	MSC_IREG_END_CMD_RES		(1 << 2)
#define	MSC_IREG_PRG_DONE		(1 << 1)
#define	MSC_IREG_DATA_TRAN_DONE		(1 << 0)




/*************************************************************************
 * RTC
 *************************************************************************/
#define RTC_RCR		(RTC_BASE + 0x00)
#define RTC_RSR		(RTC_BASE + 0x04)
#define RTC_RSAR	(RTC_BASE + 0x08)
#define RTC_RGR		(RTC_BASE + 0x0c)

#define REG_RTC_RCR	REG32(RTC_RCR)
#define REG_RTC_RSR	REG32(RTC_RSR)
#define REG_RTC_RSAR	REG32(RTC_RSAR)
#define REG_RTC_RGR	REG32(RTC_RGR)

#define RTC_RCR_HZ	(1 << 6)
#define RTC_RCR_HZIE	(1 << 5)
#define RTC_RCR_AF	(1 << 4)
#define RTC_RCR_AIE	(1 << 3)
#define RTC_RCR_AE	(1 << 2)
#define RTC_RCR_START	(1 << 0)

#define RTC_RGR_LOCK		(1 << 31)
#define RTC_RGR_ADJ_BIT		16
#define RTC_RGR_ADJ_MASK	(0x3ff << RTC_RGR_ADJ_BIT)
#define RTC_RGR_DIV_BIT		0
#define RTC_REG_DIV_MASK	(0xff << RTC_RGR_DIV_BIT)




/*************************************************************************
 * FIR
 *************************************************************************/
#define	FIR_TDR			(FIR_BASE + 0x000)
#define	FIR_RDR			(FIR_BASE + 0x004)
#define	FIR_TFLR		(FIR_BASE + 0x008)
#define	FIR_AR			(FIR_BASE + 0x00C)
#define	FIR_CR1			(FIR_BASE + 0x010)
#define	FIR_CR2			(FIR_BASE + 0x014)
#define	FIR_SR			(FIR_BASE + 0x018)

#define	REG_FIR_TDR		REG8(FIR_TDR)
#define	REG_FIR_RDR		REG8(FIR_RDR)
#define REG_FIR_TFLR		REG16(FIR_TFLR)
#define REG_FIR_AR		REG8(FIR_AR)
#define	REG_FIR_CR1		REG8(FIR_CR1)
#define	REG_FIR_CR2		REG16(FIR_CR2)
#define REG_FIR_SR		REG16(FIR_SR)

/* FIR Control Register 1 (FIR_CR1) */

#define FIR_CR1_FIRUE		(1 << 7)
#define FIR_CR1_ACE		(1 << 6)
#define FIR_CR1_EOUS		(1 << 5)
#define FIR_CR1_TIIE		(1 << 4)
#define FIR_CR1_TFIE		(1 << 3)
#define FIR_CR1_RFIE		(1 << 2)
#define FIR_CR1_TXE		(1 << 1)
#define FIR_CR1_RXE		(1 << 0)

/* FIR Control Register 2 (FIR_CR2) */

#define FIR_CR2_SIPE		(1 << 10)
#define FIR_CR2_BCRC		(1 << 9)
#define FIR_CR2_TFLRS		(1 << 8)
#define FIR_CR2_ISS		(1 << 7)
#define FIR_CR2_LMS		(1 << 6)
#define FIR_CR2_TPPS		(1 << 5)
#define FIR_CR2_RPPS		(1 << 4)
#define FIR_CR2_TTRG_BIT	2
#define FIR_CR2_TTRG_MASK	(0x3 << FIR_CR2_TTRG_BIT)
  #define FIR_CR2_TTRG_16	  (0 << FIR_CR2_TTRG_BIT) /* Transmit Trigger Level is 16 */
  #define FIR_CR2_TTRG_32	  (1 << FIR_CR2_TTRG_BIT) /* Transmit Trigger Level is 32 */
  #define FIR_CR2_TTRG_64	  (2 << FIR_CR2_TTRG_BIT) /* Transmit Trigger Level is 64 */
  #define FIR_CR2_TTRG_128	  (3 << FIR_CR2_TTRG_BIT) /* Transmit Trigger Level is 128 */
#define FIR_CR2_RTRG_BIT	0
#define FIR_CR2_RTRG_MASK	(0x3 << FIR_CR2_RTRG_BIT)
  #define FIR_CR2_RTRG_16	  (0 << FIR_CR2_RTRG_BIT) /* Receive Trigger Level is 16 */
  #define FIR_CR2_RTRG_32	  (1 << FIR_CR2_RTRG_BIT) /* Receive Trigger Level is 32 */
  #define FIR_CR2_RTRG_64	  (2 << FIR_CR2_RTRG_BIT) /* Receive Trigger Level is 64 */
  #define FIR_CR2_RTRG_128	  (3 << FIR_CR2_RTRG_BIT) /* Receive Trigger Level is 128 */

/* FIR Status Register (FIR_SR) */

#define FIR_SR_RFW		(1 << 12)
#define FIR_SR_RFA		(1 << 11)
#define FIR_SR_TFRTL		(1 << 10)
#define FIR_SR_RFRTL		(1 << 9)
#define FIR_SR_URUN		(1 << 8)
#define FIR_SR_RFTE		(1 << 7)
#define FIR_SR_ORUN		(1 << 6)
#define FIR_SR_CRCE		(1 << 5)
#define FIR_SR_FEND		(1 << 4)
#define FIR_SR_TFF		(1 << 3)
#define FIR_SR_RFE		(1 << 2)
#define FIR_SR_TIDLE		(1 << 1)
#define FIR_SR_RB		(1 << 0)




/*************************************************************************
 * SCC
 *************************************************************************/
#define	SCC_DR(base)		((base) + 0x000)
#define	SCC_FDR(base)		((base) + 0x004)
#define	SCC_CR(base)		((base) + 0x008)
#define	SCC1_CR(base)		((base) + 0x008)
#define	SCC_SR(base)		((base) + 0x00C)
#define	SCC_TFR(base)		((base) + 0x010)
#define	SCC_EGTR(base)		((base) + 0x014)
#define	SCC_ECR(base)		((base) + 0x018)
#define	SCC_RTOR(base)		((base) + 0x01C)

#define REG_SCC_DR(base)	REG8(SCC_DR(base))
#define REG_SCC_FDR(base)	REG8(SCC_FDR(base))
#define REG_SCC_CR(base)	REG32(SCC_CR(base))
#define REG_SCC1_CR(base)	REG32(SCC1_CR(base))
#define REG_SCC_SR(base)	REG16(SCC_SR(base))
#define REG_SCC_TFR(base)	REG16(SCC_TFR(base))
#define REG_SCC_EGTR(base)	REG8(SCC_EGTR(base))
#define REG_SCC_ECR(base)	REG32(SCC_ECR(base))
#define REG_SCC_RTOR(base)	REG8(SCC_RTOR(base))

/* SCC FIFO Data Count Register (SCC_FDR) */

#define SCC_FDR_EMPTY		0x00
#define SCC_FDR_FULL		0x10

/* SCC Control Register (SCC_CR) */

#define SCC_CR_SCCE		(1 << 31)
#define SCC_CR_TRS		(1 << 30)
#define SCC_CR_T2R		(1 << 29)
#define SCC_CR_FDIV_BIT		24
#define SCC_CR_FDIV_MASK	(0x3 << SCC_CR_FDIV_BIT)
  #define SCC_CR_FDIV_1		  (0 << SCC_CR_FDIV_BIT) /* SCC_CLK frequency is the same as device clock */
  #define SCC_CR_FDIV_2		  (1 << SCC_CR_FDIV_BIT) /* SCC_CLK frequency is half of device clock */
#define SCC_CR_FLUSH		(1 << 23)
#define SCC_CR_TRIG_BIT		16
#define SCC_CR_TRIG_MASK	(0x3 << SCC_CR_TRIG_BIT)
  #define SCC_CR_TRIG_1		  (0 << SCC_CR_TRIG_BIT) /* Receive/Transmit-FIFO Trigger is 1 */
  #define SCC_CR_TRIG_4		  (1 << SCC_CR_TRIG_BIT) /* Receive/Transmit-FIFO Trigger is 4 */
  #define SCC_CR_TRIG_8		  (2 << SCC_CR_TRIG_BIT) /* Receive/Transmit-FIFO Trigger is 8 */
  #define SCC_CR_TRIG_14	  (3 << SCC_CR_TRIG_BIT) /* Receive/Transmit-FIFO Trigger is 14 */
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
  #define SCC_CR_PX_NOT_SUPPORT	  (0 << SCC_CR_PX_BIT) /* SCC does not support clock stop */
  #define SCC_CR_PX_STOP_LOW	  (1 << SCC_CR_PX_BIT) /* SCC_CLK stops at state low */
  #define SCC_CR_PX_STOP_HIGH	  (2 << SCC_CR_PX_BIT) /* SCC_CLK stops at state high */
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




/*************************************************************************
 * ETH
 *************************************************************************/
#define ETH_BMR		(ETH_BASE + 0x1000)
#define ETH_TPDR	(ETH_BASE + 0x1004)
#define ETH_RPDR	(ETH_BASE + 0x1008)
#define ETH_RAR		(ETH_BASE + 0x100C)
#define ETH_TAR		(ETH_BASE + 0x1010)
#define ETH_SR		(ETH_BASE + 0x1014)
#define ETH_CR		(ETH_BASE + 0x1018)
#define ETH_IER		(ETH_BASE + 0x101C)
#define ETH_MFCR	(ETH_BASE + 0x1020)
#define ETH_CTAR	(ETH_BASE + 0x1050)
#define ETH_CRAR	(ETH_BASE + 0x1054)
#define ETH_MCR		(ETH_BASE + 0x0000)
#define ETH_MAHR	(ETH_BASE + 0x0004)
#define ETH_MALR	(ETH_BASE + 0x0008)
#define ETH_HTHR	(ETH_BASE + 0x000C)
#define ETH_HTLR	(ETH_BASE + 0x0010)
#define ETH_MIAR	(ETH_BASE + 0x0014)
#define ETH_MIDR	(ETH_BASE + 0x0018)
#define ETH_FCR		(ETH_BASE + 0x001C)
#define ETH_VTR1	(ETH_BASE + 0x0020)
#define ETH_VTR2	(ETH_BASE + 0x0024)
#define ETH_WKFR	(ETH_BASE + 0x0028)
#define ETH_PMTR	(ETH_BASE + 0x002C)

#define REG_ETH_BMR	REG32(ETH_BMR)
#define REG_ETH_TPDR	REG32(ETH_TPDR)
#define REG_ETH_RPDR	REG32(ETH_RPDR)
#define REG_ETH_RAR	REG32(ETH_RAR)
#define REG_ETH_TAR	REG32(ETH_TAR)
#define REG_ETH_SR	REG32(ETH_SR)
#define REG_ETH_CR	REG32(ETH_CR)
#define REG_ETH_IER	REG32(ETH_IER)
#define REG_ETH_MFCR	REG32(ETH_MFCR)
#define REG_ETH_CTAR	REG32(ETH_CTAR)
#define REG_ETH_CRAR	REG32(ETH_CRAR)
#define REG_ETH_MCR	REG32(ETH_MCR)
#define REG_ETH_MAHR	REG32(ETH_MAHR)
#define REG_ETH_MALR	REG32(ETH_MALR)
#define REG_ETH_HTHR	REG32(ETH_HTHR)
#define REG_ETH_HTLR	REG32(ETH_HTLR)
#define REG_ETH_MIAR	REG32(ETH_MIAR)
#define REG_ETH_MIDR	REG32(ETH_MIDR)
#define REG_ETH_FCR	REG32(ETH_FCR)
#define REG_ETH_VTR1	REG32(ETH_VTR1)
#define REG_ETH_VTR2	REG32(ETH_VTR2)
#define REG_ETH_WKFR	REG32(ETH_WKFR)
#define REG_ETH_PMTR	REG32(ETH_PMTR)

/* Bus Mode Register (ETH_BMR) */

#define ETH_BMR_DBO		(1 << 20)
#define ETH_BMR_PBL_BIT		8
#define ETH_BMR_PBL_MASK	(0x3f << ETH_BMR_PBL_BIT)
  #define ETH_BMR_PBL_1		  (0x1 << ETH_BMR_PBL_BIT)
  #define ETH_BMR_PBL_4		  (0x4 << ETH_BMR_PBL_BIT)
#define ETH_BMR_BLE		(1 << 7)
#define ETH_BMR_DSL_BIT		2
#define ETH_BMR_DSL_MASK	(0x1f << ETH_BMR_DSL_BIT)
  #define ETH_BMR_DSL_0		  (0x0 << ETH_BMR_DSL_BIT)
  #define ETH_BMR_DSL_1		  (0x1 << ETH_BMR_DSL_BIT)
  #define ETH_BMR_DSL_2		  (0x2 << ETH_BMR_DSL_BIT)
  #define ETH_BMR_DSL_4		  (0x4 << ETH_BMR_DSL_BIT)
  #define ETH_BMR_DSL_8		  (0x8 << ETH_BMR_DSL_BIT)
#define ETH_BMR_SWR		(1 << 0)

/* DMA Status Register (ETH_SR) */

#define ETH_SR_EB_BIT		23
#define ETH_SR_EB_MASK		(0x7 << ETH_SR_EB_BIT)
  #define ETH_SR_EB_TX_ABORT	  (0x1 << ETH_SR_EB_BIT)
  #define ETH_SR_EB_RX_ABORT	  (0x2 << ETH_SR_EB_BIT)
#define ETH_SR_TS_BIT		20
#define ETH_SR_TS_MASK		(0x7 << ETH_SR_TS_BIT)
  #define ETH_SR_TS_STOP	  (0x0 << ETH_SR_TS_BIT)
  #define ETH_SR_TS_FTD		  (0x1 << ETH_SR_TS_BIT)
  #define ETH_SR_TS_WEOT	  (0x2 << ETH_SR_TS_BIT)
  #define ETH_SR_TS_QDAT	  (0x3 << ETH_SR_TS_BIT)
  #define ETH_SR_TS_SUSPEND	  (0x6 << ETH_SR_TS_BIT)
  #define ETH_SR_TS_CTD		  (0x7 << ETH_SR_TS_BIT)
#define ETH_SR_RS_BIT		17
#define ETH_SR_RS_MASK		(0x7 << ETH_SR_RS_BIT)
  #define ETH_SR_RS_STOP	  (0x0 << ETH_SR_RS_BIT)
  #define ETH_SR_RS_FRD		  (0x1 << ETH_SR_RS_BIT)
  #define ETH_SR_RS_CEOR	  (0x2 << ETH_SR_RS_BIT)
  #define ETH_SR_RS_WRP		  (0x3 << ETH_SR_RS_BIT)
  #define ETH_SR_RS_SUSPEND	  (0x4 << ETH_SR_RS_BIT)
  #define ETH_SR_RS_CRD		  (0x5 << ETH_SR_RS_BIT)
  #define ETH_SR_RS_FCF		  (0x6 << ETH_SR_RS_BIT)
  #define ETH_SR_RS_QRF		  (0x7 << ETH_SR_RS_BIT)
#define ETH_SR_NIS		(1 << 16)
#define ETH_SR_AIS		(1 << 15)
#define ETH_SR_ERI		(1 << 14)
#define ETH_SR_FBE		(1 << 13)
#define ETH_SR_ETI		(1 << 10)
#define ETH_SR_RWT		(1 << 9)
#define ETH_SR_RPS		(1 << 8)
#define ETH_SR_RU		(1 << 7)
#define ETH_SR_RI		(1 << 6)
#define ETH_SR_UNF		(1 << 5)
#define ETH_SR_TJT		(1 << 3)
#define ETH_SR_TU		(1 << 2)
#define ETH_SR_TPS		(1 << 1)
#define ETH_SR_TI		(1 << 0)

/* Control (Operation Mode) Register (ETH_CR) */

#define ETH_CR_TTM		(1 << 22)
#define ETH_CR_SF		(1 << 21)
#define ETH_CR_TR_BIT		14
#define ETH_CR_TR_MASK		(0x3 << ETH_CR_TR_BIT)
#define ETH_CR_ST		(1 << 13)
#define ETH_CR_OSF		(1 << 2)
#define ETH_CR_SR		(1 << 1)

/* Interrupt Enable Register (ETH_IER) */

#define ETH_IER_NI		(1 << 16)
#define ETH_IER_AI		(1 << 15)
#define ETH_IER_ERE		(1 << 14)
#define ETH_IER_FBE		(1 << 13)
#define ETH_IER_ET		(1 << 10)
#define ETH_IER_RWE		(1 << 9)
#define ETH_IER_RS		(1 << 8)
#define ETH_IER_RU		(1 << 7)
#define ETH_IER_RI		(1 << 6)
#define ETH_IER_UN		(1 << 5)
#define ETH_IER_TJ		(1 << 3)
#define ETH_IER_TU		(1 << 2)
#define ETH_IER_TS		(1 << 1)
#define ETH_IER_TI		(1 << 0)

/* Missed Frame and Buffer Overflow Counter Register (ETH_MFCR) */

#define ETH_MFCR_OVERFLOW_BIT	17
#define ETH_MFCR_OVERFLOW_MASK	(0x7ff << ETH_MFCR_OVERFLOW_BIT)
#define ETH_MFCR_MFC_BIT	0
#define ETH_MFCR_MFC_MASK	(0xffff << ETH_MFCR_MFC_BIT)

/* MAC Control Register (ETH_MCR) */

#define ETH_MCR_RA		(1 << 31)
#define ETH_MCR_HBD		(1 << 28)
#define ETH_MCR_PS		(1 << 27)
#define ETH_MCR_DRO		(1 << 23)
#define ETH_MCR_OM_BIT		21
#define ETH_MCR_OM_MASK		(0x3 << ETH_MCR_OM_BIT)
  #define ETH_MCR_OM_NORMAL	  (0x0 << ETH_MCR_OM_BIT)
  #define ETH_MCR_OM_INTERNAL	  (0x1 << ETH_MCR_OM_BIT)
  #define ETH_MCR_OM_EXTERNAL	  (0x2 << ETH_MCR_OM_BIT)
#define ETH_MCR_F		(1 << 20)
#define ETH_MCR_PM		(1 << 19)
#define ETH_MCR_PR		(1 << 18)
#define ETH_MCR_IF		(1 << 17)
#define ETH_MCR_PB		(1 << 16)
#define ETH_MCR_HO		(1 << 15)
#define ETH_MCR_HP		(1 << 13)
#define ETH_MCR_LCC		(1 << 12)
#define ETH_MCR_DBF		(1 << 11)
#define ETH_MCR_DTRY		(1 << 10)
#define ETH_MCR_ASTP		(1 << 8)
#define ETH_MCR_BOLMT_BIT	6
#define ETH_MCR_BOLMT_MASK	(0x3 << ETH_MCR_BOLMT_BIT)
  #define ETH_MCR_BOLMT_10	  (0 << ETH_MCR_BOLMT_BIT)
  #define ETH_MCR_BOLMT_8	  (1 << ETH_MCR_BOLMT_BIT)
  #define ETH_MCR_BOLMT_4	  (2 << ETH_MCR_BOLMT_BIT)
  #define ETH_MCR_BOLMT_1	  (3 << ETH_MCR_BOLMT_BIT)
#define ETH_MCR_DC		(1 << 5)
#define ETH_MCR_TE		(1 << 3)
#define ETH_MCR_RE		(1 << 2)

/* MII Address Register (ETH_MIAR) */

#define ETH_MIAR_PHY_ADDR_BIT	11
#define ETH_MIAR_PHY_ADDR_MASK	(0x1f << ETH_MIAR_PHY_ADDR_BIT)
#define ETH_MIAR_MII_REG_BIT	6
#define ETH_MIAR_MII_REG_MASK	(0x1f << ETH_MIAR_MII_REG_BIT)
#define ETH_MIAR_MII_WRITE	(1 << 1)
#define ETH_MIAR_MII_BUSY	(1 << 0)

/* Flow Control Register (ETH_FCR) */

#define	ETH_FCR_PAUSE_TIME_BIT	16
#define	ETH_FCR_PAUSE_TIME_MASK	(0xffff << ETH_FCR_PAUSE_TIME_BIT)
#define	ETH_FCR_PCF		(1 << 2)
#define	ETH_FCR_FCE		(1 << 1)
#define	ETH_FCR_BUSY		(1 << 0)

/* PMT Control and Status Register (ETH_PMTR) */

#define ETH_PMTR_GU		(1 << 9)
#define ETH_PMTR_RF		(1 << 6)
#define ETH_PMTR_MF		(1 << 5)
#define ETH_PMTR_RWK		(1 << 2)
#define ETH_PMTR_MPK		(1 << 1)

/* Receive Descriptor 0 (ETH_RD0) Bits */

#define ETH_RD0_OWN		(1 << 31)
#define ETH_RD0_FF		(1 << 30)
#define ETH_RD0_FL_BIT		16
#define ETH_RD0_FL_MASK		(0x3fff << ETH_RD0_FL_BIT)
#define ETH_RD0_ES		(1 << 15)
#define ETH_RD0_DE		(1 << 14)
#define ETH_RD0_LE		(1 << 12)
#define ETH_RD0_RF		(1 << 11)
#define ETH_RD0_MF		(1 << 10)
#define ETH_RD0_FD		(1 << 9)
#define ETH_RD0_LD		(1 << 8)
#define ETH_RD0_TL		(1 << 7)
#define ETH_RD0_CS		(1 << 6)
#define ETH_RD0_FT		(1 << 5)
#define ETH_RD0_WT		(1 << 4)
#define ETH_RD0_ME		(1 << 3)
#define ETH_RD0_DB		(1 << 2)
#define ETH_RD0_CE		(1 << 1)

/* Receive Descriptor 1 (ETH_RD1) Bits */

#define ETH_RD1_RER		(1 << 25)
#define ETH_RD1_RCH		(1 << 24)
#define ETH_RD1_RBS2_BIT	11
#define ETH_RD1_RBS2_MASK	(0x7ff << ETH_RD1_RBS2_BIT)
#define ETH_RD1_RBS1_BIT	0
#define ETH_RD1_RBS1_MASK	(0x7ff << ETH_RD1_RBS1_BIT)

/* Transmit Descriptor 0 (ETH_TD0) Bits */

#define ETH_TD0_OWN		(1 << 31)
#define ETH_TD0_FA		(1 << 15)
#define ETH_TD0_LOC		(1 << 11)
#define ETH_TD0_NC		(1 << 10)
#define ETH_TD0_LC		(1 << 9)
#define ETH_TD0_EC		(1 << 8)
#define ETH_TD0_HBF		(1 << 7)
#define ETH_TD0_CC_BIT		3
#define ETH_TD0_CC_MASK		(0xf << ETH_TD0_CC_BIT)
#define ETH_TD0_ED		(1 << 2)
#define ETH_TD0_UF		(1 << 1)
#define ETH_TD0_DF		(1 << 0)

/* Transmit Descriptor 1 (ETH_TD1) Bits */

#define ETH_TD1_IC		(1 << 31)
#define ETH_TD1_LS		(1 << 30)
#define ETH_TD1_FS		(1 << 29)
#define ETH_TD1_AC		(1 << 26)
#define ETH_TD1_TER		(1 << 25)
#define ETH_TD1_TCH		(1 << 24)
#define ETH_TD1_DPD		(1 << 23)
#define ETH_TD1_TBS2_BIT	11
#define ETH_TD1_TBS2_MASK	(0x7ff << ETH_TD1_TBS2_BIT)
#define ETH_TD1_TBS1_BIT	0
#define ETH_TD1_TBS1_MASK	(0x7ff << ETH_TD1_TBS1_BIT)




/*************************************************************************
 * WDT
 *************************************************************************/
#define WDT_WTCSR	(WDT_BASE + 0x00)
#define WDT_WTCNT	(WDT_BASE + 0x04)

#define REG_WDT_WTCSR	REG8(WDT_WTCSR)
#define REG_WDT_WTCNT	REG32(WDT_WTCNT)

#define WDT_WTCSR_START	(1 << 4)




/*************************************************************************
 * OST
 *************************************************************************/
#define OST_TER		(OST_BASE + 0x00)
#define OST_TRDR(n)	(OST_BASE + 0x10 + ((n) * 0x20))
#define OST_TCNT(n)	(OST_BASE + 0x14 + ((n) * 0x20))
#define OST_TCSR(n)	(OST_BASE + 0x18 + ((n) * 0x20))
#define OST_TCRB(n)	(OST_BASE + 0x1c + ((n) * 0x20))

#define REG_OST_TER	REG8(OST_TER)
#define REG_OST_TRDR(n)	REG32(OST_TRDR((n)))
#define REG_OST_TCNT(n)	REG32(OST_TCNT((n)))
#define REG_OST_TCSR(n)	REG16(OST_TCSR((n)))
#define REG_OST_TCRB(n)	REG32(OST_TCRB((n)))

#define OST_TCSR_BUSY		(1 << 7)
#define OST_TCSR_UF		(1 << 6)
#define OST_TCSR_UIE		(1 << 5)
#define OST_TCSR_CKS_BIT	0
#define OST_TCSR_CKS_MASK	(0x07 << OST_TCSR_CKS_BIT)
  #define OST_TCSR_CKS_PCLK_4	(0 << OST_TCSR_CKS_BIT)
  #define OST_TCSR_CKS_PCLK_16	(1 << OST_TCSR_CKS_BIT)
  #define OST_TCSR_CKS_PCLK_64	(2 << OST_TCSR_CKS_BIT)
  #define OST_TCSR_CKS_PCLK_256	(3 << OST_TCSR_CKS_BIT)
  #define OST_TCSR_CKS_RTCCLK	(4 << OST_TCSR_CKS_BIT)
  #define OST_TCSR_CKS_EXTAL	(5 << OST_TCSR_CKS_BIT)

#define OST_TCSR0       OST_TCSR(0)
#define OST_TCSR1       OST_TCSR(1)
#define OST_TCSR2       OST_TCSR(2)
#define OST_TRDR0       OST_TRDR(0)
#define OST_TRDR1       OST_TRDR(1)
#define OST_TRDR2       OST_TRDR(2)
#define OST_TCNT0       OST_TCNT(0)
#define OST_TCNT1       OST_TCNT(1)
#define OST_TCNT2       OST_TCNT(2)
#define OST_TCRB0       OST_TCRB(0)
#define OST_TCRB1       OST_TCRB(1)
#define OST_TCRB2       OST_TCRB(2)

/*************************************************************************
 * UART
 *************************************************************************/

#define IRDA_BASE	UART0_BASE
#define UART_BASE	UART0_BASE
#define UART_OFF	0x1000

/* register offset */
#define OFF_RDR		(0x00)	/* R  8b H'xx */
#define OFF_TDR		(0x00)	/* W  8b H'xx */
#define OFF_DLLR	(0x00)	/* RW 8b H'00 */
#define OFF_DLHR	(0x04)	/* RW 8b H'00 */
#define OFF_IER		(0x04)	/* RW 8b H'00 */
#define OFF_ISR		(0x08)	/* R  8b H'01 */
#define OFF_FCR		(0x08)	/* W  8b H'00 */
#define OFF_LCR		(0x0C)	/* RW 8b H'00 */
#define OFF_MCR		(0x10)	/* RW 8b H'00 */
#define OFF_LSR		(0x14)	/* R  8b H'00 */
#define OFF_MSR		(0x18)	/* R  8b H'00 */
#define OFF_SPR		(0x1C)	/* RW 8b H'00 */
#define OFF_MCR		(0x10)	/* RW 8b H'00 */
#define OFF_SIRCR	(0x20)	/* RW 8b H'00, UART0 */

/* register address */
#define UART0_RDR	(UART0_BASE + OFF_RDR)
#define UART0_TDR	(UART0_BASE + OFF_TDR)
#define UART0_DLLR	(UART0_BASE + OFF_DLLR)
#define UART0_DLHR	(UART0_BASE + OFF_DLHR)
#define UART0_IER	(UART0_BASE + OFF_IER)
#define UART0_ISR	(UART0_BASE + OFF_ISR)
#define UART0_FCR	(UART0_BASE + OFF_FCR)
#define UART0_LCR	(UART0_BASE + OFF_LCR)
#define UART0_MCR	(UART0_BASE + OFF_MCR)
#define UART0_LSR	(UART0_BASE + OFF_LSR)
#define UART0_MSR	(UART0_BASE + OFF_MSR)
#define UART0_SPR	(UART0_BASE + OFF_SPR)
#define UART0_SIRCR	(UART0_BASE + OFF_SIRCR)

#define UART1_RDR	(UART1_BASE + OFF_RDR)
#define UART1_TDR	(UART1_BASE + OFF_TDR)
#define UART1_DLLR	(UART1_BASE + OFF_DLLR)
#define UART1_DLHR	(UART1_BASE + OFF_DLHR)
#define UART1_IER	(UART1_BASE + OFF_IER)
#define UART1_ISR	(UART1_BASE + OFF_ISR)
#define UART1_FCR	(UART1_BASE + OFF_FCR)
#define UART1_LCR	(UART1_BASE + OFF_LCR)
#define UART1_MCR	(UART1_BASE + OFF_MCR)
#define UART1_LSR	(UART1_BASE + OFF_LSR)
#define UART1_MSR	(UART1_BASE + OFF_MSR)
#define UART1_SPR	(UART1_BASE + OFF_SPR)
#define UART1_SIRCR	(UART1_BASE + OFF_SIRCR)

#define UART2_RDR	(UART2_BASE + OFF_RDR)
#define UART2_TDR	(UART2_BASE + OFF_TDR)
#define UART2_DLLR	(UART2_BASE + OFF_DLLR)
#define UART2_DLHR	(UART2_BASE + OFF_DLHR)
#define UART2_IER	(UART2_BASE + OFF_IER)
#define UART2_ISR	(UART2_BASE + OFF_ISR)
#define UART2_FCR	(UART2_BASE + OFF_FCR)
#define UART2_LCR	(UART2_BASE + OFF_LCR)
#define UART2_MCR	(UART2_BASE + OFF_MCR)
#define UART2_LSR	(UART2_BASE + OFF_LSR)
#define UART2_MSR	(UART2_BASE + OFF_MSR)
#define UART2_SPR	(UART2_BASE + OFF_SPR)
#define UART2_SIRCR	(UART2_BASE + OFF_SIRCR)

#define UART3_RDR	(UART3_BASE + OFF_RDR)
#define UART3_TDR	(UART3_BASE + OFF_TDR)
#define UART3_DLLR	(UART3_BASE + OFF_DLLR)
#define UART3_DLHR	(UART3_BASE + OFF_DLHR)
#define UART3_IER	(UART3_BASE + OFF_IER)
#define UART3_ISR	(UART3_BASE + OFF_ISR)
#define UART3_FCR	(UART3_BASE + OFF_FCR)
#define UART3_LCR	(UART3_BASE + OFF_LCR)
#define UART3_MCR	(UART3_BASE + OFF_MCR)
#define UART3_LSR	(UART3_BASE + OFF_LSR)
#define UART3_MSR	(UART3_BASE + OFF_MSR)
#define UART3_SPR	(UART3_BASE + OFF_SPR)
#define UART3_SIRCR	(UART3_BASE + OFF_SIRCR)

/*
 * Define macros for UARTIER
 * UART Interrupt Enable Register
 */
#define UARTIER_RIE	(1 << 0)	/* 0: receive fifo "full" interrupt disable */
#define UARTIER_TIE	(1 << 1)	/* 0: transmit fifo "empty" interrupt disable */
#define UARTIER_RLIE	(1 << 2)	/* 0: receive line status interrupt disable */
#define UARTIER_MIE	(1 << 3)	/* 0: modem status interrupt disable */
#define UARTIER_RTIE	(1 << 4)	/* 0: receive timeout interrupt disable */

/*
 * Define macros for UARTISR
 * UART Interrupt Status Register
 */
#define UARTISR_IP	(1 << 0)	/* 0: interrupt is pending  1: no interrupt */
#define UARTISR_IID	(7 << 1)	/* Source of Interrupt */
#define UARTISR_IID_MSI		(0 << 1)	/* Modem status interrupt */
#define UARTISR_IID_THRI	(1 << 1)	/* Transmitter holding register empty */
#define UARTISR_IID_RDI		(2 << 1)	/* Receiver data interrupt */
#define UARTISR_IID_RLSI	(3 << 1)	/* Receiver line status interrupt */
#define UARTISR_FFMS	(3 << 6)	/* FIFO mode select, set when UARTFCR.FE is set to 1 */
#define UARTISR_FFMS_NO_FIFO	(0 << 6)
#define UARTISR_FFMS_FIFO_MODE	(3 << 6)

/*
 * Define macros for UARTFCR
 * UART FIFO Control Register
 */
#define UARTFCR_FE	(1 << 0)	/* 0: non-FIFO mode  1: FIFO mode */
#define UARTFCR_RFLS	(1 << 1)	/* write 1 to flush receive FIFO */
#define UARTFCR_TFLS	(1 << 2)	/* write 1 to flush transmit FIFO */
#define UARTFCR_DMS	(1 << 3)	/* 0: disable DMA mode */
#define UARTFCR_UUE	(1 << 4)	/* 0: disable UART */
#define UARTFCR_RTRG	(3 << 6)	/* Receive FIFO Data Trigger */
#define UARTFCR_RTRG_1	(0 << 6)
#define UARTFCR_RTRG_4	(1 << 6)
#define UARTFCR_RTRG_8	(2 << 6)
#define UARTFCR_RTRG_15	(3 << 6)

/*
 * Define macros for UARTLCR
 * UART Line Control Register
 */
#define UARTLCR_WLEN	(3 << 0)	/* word length */
#define UARTLCR_WLEN_5	(0 << 0)
#define UARTLCR_WLEN_6	(1 << 0)
#define UARTLCR_WLEN_7	(2 << 0)
#define UARTLCR_WLEN_8	(3 << 0)
#define UARTLCR_STOP	(1 << 2)	/* 0: 1 stop bit when word length is 5,6,7,8
					   1: 1.5 stop bits when 5; 2 stop bits when 6,7,8 */
#define UARTLCR_PE	(1 << 3)	/* 0: parity disable */
#define UARTLCR_PROE	(1 << 4)	/* 0: even parity  1: odd parity */
#define UARTLCR_SPAR	(1 << 5)	/* 0: sticky parity disable */
#define UARTLCR_SBRK	(1 << 6)	/* write 0 normal, write 1 send break */
#define UARTLCR_DLAB	(1 << 7)	/* 0: access UARTRDR/TDR/IER  1: access UARTDLLR/DLHR */

/*
 * Define macros for UARTLSR
 * UART Line Status Register
 */
#define UARTLSR_DR	(1 << 0)	/* 0: receive FIFO is empty  1: receive data is ready */
#define UARTLSR_ORER	(1 << 1)	/* 0: no overrun error */
#define UARTLSR_PER	(1 << 2)	/* 0: no parity error */
#define UARTLSR_FER	(1 << 3)	/* 0; no framing error */
#define UARTLSR_BRK	(1 << 4)	/* 0: no break detected  1: receive a break signal */
#define UARTLSR_TDRQ	(1 << 5)	/* 1: transmit FIFO half "empty" */
#define UARTLSR_TEMT	(1 << 6)	/* 1: transmit FIFO and shift registers empty */
#define UARTLSR_RFER	(1 << 7)	/* 0: no receive error  1: receive error in FIFO mode */

/*
 * Define macros for UARTMCR
 * UART Modem Control Register
 */
#define UARTMCR_DTR	(1 << 0)	/* 0: DTR_ ouput high */
#define UARTMCR_RTS	(1 << 1)	/* 0: RTS_ output high */
#define UARTMCR_OUT1	(1 << 2)	/* 0: UARTMSR.RI is set to 0 and RI_ input high */
#define UARTMCR_OUT2	(1 << 3)	/* 0: UARTMSR.DCD is set to 0 and DCD_ input high */
#define UARTMCR_LOOP	(1 << 4)	/* 0: normal  1: loopback mode */
#define UARTMCR_MCE	(1 << 7)	/* 0: modem function is disable */

/*
 * Define macros for UARTMSR
 * UART Modem Status Register
 */
#define UARTMSR_DCTS	(1 << 0)	/* 0: no change on CTS_ pin since last read of UARTMSR */
#define UARTMSR_DDSR	(1 << 1)	/* 0: no change on DSR_ pin since last read of UARTMSR */
#define UARTMSR_DRI	(1 << 2)	/* 0: no change on RI_ pin since last read of UARTMSR */
#define UARTMSR_DDCD	(1 << 3)	/* 0: no change on DCD_ pin since last read of UARTMSR */
#define UARTMSR_CTS	(1 << 4)	/* 0: CTS_ pin is high */
#define UARTMSR_DSR	(1 << 5)	/* 0: DSR_ pin is high */
#define UARTMSR_RI	(1 << 6)	/* 0: RI_ pin is high */
#define UARTMSR_DCD	(1 << 7)	/* 0: DCD_ pin is high */

/*
 * Define macros for SIRCR
 * Slow IrDA Control Register
 */
#define SIRCR_TSIRE	(1 << 0)	/* 0: transmitter is in UART mode  1: IrDA mode */
#define SIRCR_RSIRE	(1 << 1)	/* 0: receiver is in UART mode  1: IrDA mode */
#define SIRCR_TPWS	(1 << 2)	/* 0: transmit 0 pulse width is 3/16 of bit length
					   1: 0 pulse width is 1.6us for 115.2Kbps */
#define SIRCR_TXPL	(1 << 3)	/* 0: encoder generates a positive pulse for 0 */
#define SIRCR_RXPL	(1 << 4)	/* 0: decoder interprets positive pulse as 0 */



/*************************************************************************
 * INTC
 *************************************************************************/
#define INTC_ISR	(INTC_BASE + 0x00)
#define INTC_IMR	(INTC_BASE + 0x04)
#define INTC_IMSR	(INTC_BASE + 0x08)
#define INTC_IMCR	(INTC_BASE + 0x0c)
#define INTC_IPR	(INTC_BASE + 0x10)

#define REG_INTC_ISR	REG32(INTC_ISR)
#define REG_INTC_IMR	REG32(INTC_IMR)
#define REG_INTC_IMSR	REG32(INTC_IMSR)
#define REG_INTC_IMCR	REG32(INTC_IMCR)
#define REG_INTC_IPR	REG32(INTC_IPR)

#define IRQ_I2C		1
#define IRQ_PS2		2
#define IRQ_UPRT	3
#define IRQ_CORE	4
#define IRQ_UART3	6
#define IRQ_UART2	7
#define IRQ_UART1	8
#define IRQ_UART0	9
#define IRQ_SCC1	10
#define IRQ_SCC0	11
#define IRQ_UDC		12
#define IRQ_UHC		13
#define IRQ_MSC		14
#define IRQ_RTC		15
#define IRQ_FIR		16
#define IRQ_SSI		17
#define IRQ_CIM		18
#define IRQ_ETH		19
#define IRQ_AIC		20
#define IRQ_DMAC	21
#define IRQ_OST2	22
#define IRQ_OST1	23
#define IRQ_OST0	24
#define IRQ_GPIO3	25
#define IRQ_GPIO2	26
#define IRQ_GPIO1	27
#define IRQ_GPIO0	28
#define IRQ_LCD		30




/*************************************************************************
 * CIM
 *************************************************************************/
#define	CIM_CFG			(CIM_BASE + 0x0000)
#define	CIM_CTRL		(CIM_BASE + 0x0004)
#define	CIM_STATE		(CIM_BASE + 0x0008)
#define	CIM_IID			(CIM_BASE + 0x000C)
#define	CIM_RXFIFO		(CIM_BASE + 0x0010)
#define	CIM_DA			(CIM_BASE + 0x0020)
#define	CIM_FA			(CIM_BASE + 0x0024)
#define	CIM_FID			(CIM_BASE + 0x0028)
#define	CIM_CMD			(CIM_BASE + 0x002C)

#define	REG_CIM_CFG		REG32(CIM_CFG)
#define	REG_CIM_CTRL		REG32(CIM_CTRL)
#define	REG_CIM_STATE		REG32(CIM_STATE)
#define	REG_CIM_IID		REG32(CIM_IID)
#define	REG_CIM_RXFIFO		REG32(CIM_RXFIFO)
#define	REG_CIM_DA		REG32(CIM_DA)
#define	REG_CIM_FA		REG32(CIM_FA)
#define	REG_CIM_FID		REG32(CIM_FID)
#define	REG_CIM_CMD		REG32(CIM_CMD)

/* CIM Configuration Register  (CIM_CFG) */

#define	CIM_CFG_INV_DAT		(1 << 15)
#define	CIM_CFG_VSP		(1 << 14)
#define	CIM_CFG_HSP		(1 << 13)
#define	CIM_CFG_PCP		(1 << 12)
#define	CIM_CFG_DUMMY_ZERO	(1 << 9)
#define	CIM_CFG_EXT_VSYNC	(1 << 8)
#define	CIM_CFG_PACK_BIT	4
#define	CIM_CFG_PACK_MASK	(0x7 << CIM_CFG_PACK_BIT)
  #define CIM_CFG_PACK_0	  (0 << CIM_CFG_PACK_BIT)
  #define CIM_CFG_PACK_1	  (1 << CIM_CFG_PACK_BIT)
  #define CIM_CFG_PACK_2	  (2 << CIM_CFG_PACK_BIT)
  #define CIM_CFG_PACK_3	  (3 << CIM_CFG_PACK_BIT)
  #define CIM_CFG_PACK_4	  (4 << CIM_CFG_PACK_BIT)
  #define CIM_CFG_PACK_5	  (5 << CIM_CFG_PACK_BIT)
  #define CIM_CFG_PACK_6	  (6 << CIM_CFG_PACK_BIT)
  #define CIM_CFG_PACK_7	  (7 << CIM_CFG_PACK_BIT)
#define	CIM_CFG_DSM_BIT		0
#define	CIM_CFG_DSM_MASK	(0x3 << CIM_CFG_DSM_BIT)
  #define CIM_CFG_DSM_CPM	  (0 << CIM_CFG_DSM_BIT) /* CCIR656 Progressive Mode */
  #define CIM_CFG_DSM_CIM	  (1 << CIM_CFG_DSM_BIT) /* CCIR656 Interlace Mode */
  #define CIM_CFG_DSM_GCM	  (2 << CIM_CFG_DSM_BIT) /* Gated Clock Mode */
  #define CIM_CFG_DSM_NGCM	  (3 << CIM_CFG_DSM_BIT) /* Non-Gated Clock Mode */

/* CIM Control Register  (CIM_CTRL) */

#define	CIM_CTRL_MCLKDIV_BIT	24
#define	CIM_CTRL_MCLKDIV_MASK	(0xff << CIM_CTRL_MCLKDIV_BIT)
#define	CIM_CTRL_FRC_BIT	16
#define	CIM_CTRL_FRC_MASK	(0xf << CIM_CTRL_FRC_BIT)
  #define CIM_CTRL_FRC_1	  (0x0 << CIM_CTRL_FRC_BIT) /* Sample every frame */
  #define CIM_CTRL_FRC_2	  (0x1 << CIM_CTRL_FRC_BIT) /* Sample 1/2 frame */
  #define CIM_CTRL_FRC_3	  (0x2 << CIM_CTRL_FRC_BIT) /* Sample 1/3 frame */
  #define CIM_CTRL_FRC_4	  (0x3 << CIM_CTRL_FRC_BIT) /* Sample 1/4 frame */
  #define CIM_CTRL_FRC_5	  (0x4 << CIM_CTRL_FRC_BIT) /* Sample 1/5 frame */
  #define CIM_CTRL_FRC_6	  (0x5 << CIM_CTRL_FRC_BIT) /* Sample 1/6 frame */
  #define CIM_CTRL_FRC_7	  (0x6 << CIM_CTRL_FRC_BIT) /* Sample 1/7 frame */
  #define CIM_CTRL_FRC_8	  (0x7 << CIM_CTRL_FRC_BIT) /* Sample 1/8 frame */
  #define CIM_CTRL_FRC_9	  (0x8 << CIM_CTRL_FRC_BIT) /* Sample 1/9 frame */
  #define CIM_CTRL_FRC_10	  (0x9 << CIM_CTRL_FRC_BIT) /* Sample 1/10 frame */
  #define CIM_CTRL_FRC_11	  (0xA << CIM_CTRL_FRC_BIT) /* Sample 1/11 frame */
  #define CIM_CTRL_FRC_12	  (0xB << CIM_CTRL_FRC_BIT) /* Sample 1/12 frame */
  #define CIM_CTRL_FRC_13	  (0xC << CIM_CTRL_FRC_BIT) /* Sample 1/13 frame */
  #define CIM_CTRL_FRC_14	  (0xD << CIM_CTRL_FRC_BIT) /* Sample 1/14 frame */
  #define CIM_CTRL_FRC_15	  (0xE << CIM_CTRL_FRC_BIT) /* Sample 1/15 frame */
  #define CIM_CTRL_FRC_16	  (0xF << CIM_CTRL_FRC_BIT) /* Sample 1/16 frame */
#define	CIM_CTRL_VDDM		(1 << 13)
#define	CIM_CTRL_DMA_SOFM	(1 << 12)
#define	CIM_CTRL_DMA_EOFM	(1 << 11)
#define	CIM_CTRL_DMA_STOPM	(1 << 10)
#define	CIM_CTRL_RXF_TRIGM	(1 << 9)
#define	CIM_CTRL_RXF_OFM	(1 << 8)
#define	CIM_CTRL_RXF_TRIG_BIT	4
#define	CIM_CTRL_RXF_TRIG_MASK	(0x7 << CIM_CTRL_RXF_TRIG_BIT)
  #define CIM_CTRL_RXF_TRIG_4	  (0 << CIM_CTRL_RXF_TRIG_BIT) /* RXFIFO Trigger Value is 4 */
  #define CIM_CTRL_RXF_TRIG_8	  (1 << CIM_CTRL_RXF_TRIG_BIT) /* RXFIFO Trigger Value is 8 */
  #define CIM_CTRL_RXF_TRIG_12	  (2 << CIM_CTRL_RXF_TRIG_BIT) /* RXFIFO Trigger Value is 12 */
  #define CIM_CTRL_RXF_TRIG_16	  (3 << CIM_CTRL_RXF_TRIG_BIT) /* RXFIFO Trigger Value is 16 */
  #define CIM_CTRL_RXF_TRIG_20	  (4 << CIM_CTRL_RXF_TRIG_BIT) /* RXFIFO Trigger Value is 20 */
  #define CIM_CTRL_RXF_TRIG_24	  (5 << CIM_CTRL_RXF_TRIG_BIT) /* RXFIFO Trigger Value is 24 */
  #define CIM_CTRL_RXF_TRIG_28	  (6 << CIM_CTRL_RXF_TRIG_BIT) /* RXFIFO Trigger Value is 28 */
  #define CIM_CTRL_RXF_TRIG_32	  (7 << CIM_CTRL_RXF_TRIG_BIT) /* RXFIFO Trigger Value is 32 */
#define	CIM_CTRL_DMA_EN		(1 << 2)
#define	CIM_CTRL_RXF_RST	(1 << 1)
#define	CIM_CTRL_ENA		(1 << 0)

/* CIM State Register  (CIM_STATE) */

#define	CIM_STATE_DMA_SOF	(1 << 6)
#define	CIM_STATE_DMA_EOF	(1 << 5)
#define	CIM_STATE_DMA_STOP	(1 << 4)
#define	CIM_STATE_RXF_OF	(1 << 3)
#define	CIM_STATE_RXF_TRIG	(1 << 2)
#define	CIM_STATE_RXF_EMPTY	(1 << 1)
#define	CIM_STATE_VDD		(1 << 0)

/* CIM DMA Command Register (CIM_CMD) */

#define	CIM_CMD_SOFINT		(1 << 31)
#define	CIM_CMD_EOFINT		(1 << 30)
#define	CIM_CMD_STOP		(1 << 28)
#define	CIM_CMD_LEN_BIT		0
#define	CIM_CMD_LEN_MASK	(0xffffff << CIM_CMD_LEN_BIT)




/*************************************************************************
 * PWM
 *************************************************************************/
#define	PWM_CTR(n)		(PWM##n##_BASE + 0x000)
#define	PWM_PER(n)		(PWM##n##_BASE + 0x004)
#define	PWM_DUT(n)		(PWM##n##_BASE + 0x008)

#define	REG_PWM_CTR(n)		REG8(PWM_CTR(n))
#define	REG_PWM_PER(n)		REG16(PWM_PER(n))
#define REG_PWM_DUT(n)		REG16(PWM_DUT(n))

/* PWM Control Register (PWM_CTR) */

#define	PWM_CTR_EN		(1 << 7)
#define	PWM_CTR_SD		(1 << 6)
#define	PWM_CTR_PRESCALE_BIT	0
#define	PWM_CTR_PRESCALE_MASK	(0x3f << PWM_CTR_PRESCALE_BIT)

/* PWM Period Register (PWM_PER) */

#define	PWM_PER_PERIOD_BIT	0
#define	PWM_PER_PERIOD_MASK	(0x3ff << PWM_PER_PERIOD_BIT)

/* PWM Duty Register (PWM_DUT) */

#define PWM_DUT_FDUTY		(1 << 10)
#define PWM_DUT_DUTY_BIT	0
#define PWM_DUT_DUTY_MASK	(0x3ff << PWM_DUT_DUTY_BIT)




/*************************************************************************
 * EMC
 *************************************************************************/
#define EMC_BCR		(EMC_BASE + 0x00)
#define EMC_SMCR0	(EMC_BASE + 0x10)
#define EMC_SMCR1	(EMC_BASE + 0x14)
#define EMC_SMCR2	(EMC_BASE + 0x18)
#define EMC_SMCR3	(EMC_BASE + 0x1c)
#define EMC_SMCR4	(EMC_BASE + 0x20)
#define EMC_SMCR5	(EMC_BASE + 0x24)
#define EMC_SMCR6	(EMC_BASE + 0x28)
#define EMC_SMCR7	(EMC_BASE + 0x2c)
#define EMC_SACR0	(EMC_BASE + 0x30)
#define EMC_SACR1	(EMC_BASE + 0x34)
#define EMC_SACR2	(EMC_BASE + 0x38)
#define EMC_SACR3	(EMC_BASE + 0x3c)
#define EMC_SACR4	(EMC_BASE + 0x40)
#define EMC_SACR5	(EMC_BASE + 0x44)
#define EMC_SACR6	(EMC_BASE + 0x48)
#define EMC_SACR7	(EMC_BASE + 0x4c)
#define EMC_NFCSR	(EMC_BASE + 0x50)
#define EMC_NFECC	(EMC_BASE + 0x54)
#define EMC_PCCR1	(EMC_BASE + 0x60)
#define EMC_PCCR2	(EMC_BASE + 0x64)
#define EMC_PCCR3	(EMC_BASE + 0x68)
#define EMC_PCCR4	(EMC_BASE + 0x6c)
#define EMC_DMCR	(EMC_BASE + 0x80)
#define EMC_RTCSR	(EMC_BASE + 0x84)
#define EMC_RTCNT	(EMC_BASE + 0x88)
#define EMC_RTCOR	(EMC_BASE + 0x8c)
#define EMC_DMAR1	(EMC_BASE + 0x90)
#define EMC_DMAR2	(EMC_BASE + 0x94)
#define EMC_DMAR3	(EMC_BASE + 0x98)
#define EMC_DMAR4	(EMC_BASE + 0x9c)

#define EMC_SDMR0	(EMC_BASE + 0xa000)
#define EMC_SDMR1	(EMC_BASE + 0xb000)
#define EMC_SDMR2	(EMC_BASE + 0xc000)
#define EMC_SDMR3	(EMC_BASE + 0xd000)

#define REG_EMC_BCR	REG32(EMC_BCR)
#define REG_EMC_SMCR0	REG32(EMC_SMCR0)
#define REG_EMC_SMCR1	REG32(EMC_SMCR1)
#define REG_EMC_SMCR2	REG32(EMC_SMCR2)
#define REG_EMC_SMCR3	REG32(EMC_SMCR3)
#define REG_EMC_SMCR4	REG32(EMC_SMCR4)
#define REG_EMC_SMCR5	REG32(EMC_SMCR5)
#define REG_EMC_SMCR6	REG32(EMC_SMCR6)
#define REG_EMC_SMCR7	REG32(EMC_SMCR7)
#define REG_EMC_SACR0	REG32(EMC_SACR0)
#define REG_EMC_SACR1	REG32(EMC_SACR1)
#define REG_EMC_SACR2	REG32(EMC_SACR2)
#define REG_EMC_SACR3	REG32(EMC_SACR3)
#define REG_EMC_SACR4	REG32(EMC_SACR4)
#define REG_EMC_SACR5	REG32(EMC_SACR5)
#define REG_EMC_SACR6	REG32(EMC_SACR6)
#define REG_EMC_SACR7	REG32(EMC_SACR7)
#define REG_EMC_NFCSR	REG32(EMC_NFCSR)
#define REG_EMC_NFECC	REG32(EMC_NFECC)
#define REG_EMC_DMCR	REG32(EMC_DMCR)
#define REG_EMC_RTCSR	REG16(EMC_RTCSR)
#define REG_EMC_RTCNT	REG16(EMC_RTCNT)
#define REG_EMC_RTCOR	REG16(EMC_RTCOR)
#define REG_EMC_DMAR1	REG32(EMC_DMAR1)
#define REG_EMC_DMAR2	REG32(EMC_DMAR2)
#define REG_EMC_DMAR3	REG32(EMC_DMAR3)
#define REG_EMC_DMAR4	REG32(EMC_DMAR4)
#define REG_EMC_PCCR1	REG32(EMC_PCCR1)
#define REG_EMC_PCCR2	REG32(EMC_PCCR2)
#define REG_EMC_PCCR3	REG32(EMC_PCCR3)
#define REG_EMC_PCCR4	REG32(EMC_PCCR4)


#define EMC_BCR_BRE		(1 << 1)

#define EMC_SMCR_STRV_BIT	24
#define EMC_SMCR_STRV_MASK	(0x0f << EMC_SMCR_STRV_BIT)
#define EMC_SMCR_TAW_BIT	20
#define EMC_SMCR_TAW_MASK	(0x0f << EMC_SMCR_TAW_BIT)
#define EMC_SMCR_TBP_BIT	16
#define EMC_SMCR_TBP_MASK	(0x0f << EMC_SMCR_TBP_BIT)
#define EMC_SMCR_TAH_BIT	12
#define EMC_SMCR_TAH_MASK	(0x07 << EMC_SMCR_TAH_BIT)
#define EMC_SMCR_TAS_BIT	8
#define EMC_SMCR_TAS_MASK	(0x07 << EMC_SMCR_TAS_BIT)
#define EMC_SMCR_BW_BIT		6
#define EMC_SMCR_BW_MASK	(0x03 << EMC_SMCR_BW_BIT)
  #define EMC_SMCR_BW_8BIT	(0 << EMC_SMCR_BW_BIT)
  #define EMC_SMCR_BW_16BIT	(1 << EMC_SMCR_BW_BIT)
  #define EMC_SMCR_BW_32BIT	(2 << EMC_SMCR_BW_BIT)
#define EMC_SMCR_BCM		(1 << 3)
#define EMC_SMCR_BL_BIT		1
#define EMC_SMCR_BL_MASK	(0x03 << EMC_SMCR_BL_BIT)
  #define EMC_SMCR_BL_4		(0 << EMC_SMCR_BL_BIT)
  #define EMC_SMCR_BL_8		(1 << EMC_SMCR_BL_BIT)
  #define EMC_SMCR_BL_16	(2 << EMC_SMCR_BL_BIT)
  #define EMC_SMCR_BL_32	(3 << EMC_SMCR_BL_BIT)
#define EMC_SMCR_SMT		(1 << 0)

#define EMC_SACR_BASE_BIT	8
#define EMC_SACR_BASE_MASK	(0xff << EMC_SACR_BASE_BIT)
#define EMC_SACR_MASK_BIT	0
#define EMC_SACR_MASK_MASK	(0xff << EMC_SACR_MASK_BIT)

#define EMC_NFCSR_RB		(1 << 7)
#define EMC_NFCSR_BOOT_SEL_BIT	4
#define EMC_NFCSR_BOOT_SEL_MASK	(0x07 << EMC_NFCSR_BOOT_SEL_BIT)
#define EMC_NFCSR_ERST		(1 << 3)
#define EMC_NFCSR_ECCE		(1 << 2)
#define EMC_NFCSR_FCE		(1 << 1)
#define EMC_NFCSR_NFE		(1 << 0)

#define EMC_NFECC_ECC2_BIT	16
#define EMC_NFECC_ECC2_MASK	(0xff << EMC_NFECC_ECC2_BIT)
#define EMC_NFECC_ECC1_BIT	8
#define EMC_NFECC_ECC1_MASK	(0xff << EMC_NFECC_ECC1_BIT)
#define EMC_NFECC_ECC0_BIT	0
#define EMC_NFECC_ECC0_MASK	(0xff << EMC_NFECC_ECC0_BIT)

#define EMC_DMCR_BW_BIT		31
#define EMC_DMCR_BW		(1 << EMC_DMCR_BW_BIT)
#define EMC_DMCR_CA_BIT		26
#define EMC_DMCR_CA_MASK	(0x07 << EMC_DMCR_CA_BIT)
  #define EMC_DMCR_CA_8		(0 << EMC_DMCR_CA_BIT)
  #define EMC_DMCR_CA_9		(1 << EMC_DMCR_CA_BIT)
  #define EMC_DMCR_CA_10	(2 << EMC_DMCR_CA_BIT)
  #define EMC_DMCR_CA_11	(3 << EMC_DMCR_CA_BIT)
  #define EMC_DMCR_CA_12	(4 << EMC_DMCR_CA_BIT)
#define EMC_DMCR_RMODE		(1 << 25)
#define EMC_DMCR_RFSH		(1 << 24)
#define EMC_DMCR_MRSET		(1 << 23)
#define EMC_DMCR_RA_BIT		20
#define EMC_DMCR_RA_MASK	(0x03 << EMC_DMCR_RA_BIT)
  #define EMC_DMCR_RA_11	(0 << EMC_DMCR_RA_BIT)
  #define EMC_DMCR_RA_12	(1 << EMC_DMCR_RA_BIT)
  #define EMC_DMCR_RA_13	(2 << EMC_DMCR_RA_BIT)
#define EMC_DMCR_BA_BIT		19
#define EMC_DMCR_BA		(1 << EMC_DMCR_BA_BIT)
#define EMC_DMCR_PDM		(1 << 18)
#define EMC_DMCR_EPIN		(1 << 17)
#define EMC_DMCR_TRAS_BIT	13
#define EMC_DMCR_TRAS_MASK	(0x07 << EMC_DMCR_TRAS_BIT)
#define EMC_DMCR_RCD_BIT	11
#define EMC_DMCR_RCD_MASK	(0x03 << EMC_DMCR_RCD_BIT)
#define EMC_DMCR_TPC_BIT	8
#define EMC_DMCR_TPC_MASK	(0x07 << EMC_DMCR_TPC_BIT)
#define EMC_DMCR_TRWL_BIT	5
#define EMC_DMCR_TRWL_MASK	(0x03 << EMC_DMCR_TRWL_BIT)
#define EMC_DMCR_TRC_BIT	2
#define EMC_DMCR_TRC_MASK	(0x07 << EMC_DMCR_TRC_BIT)
#define EMC_DMCR_TCL_BIT	0
#define EMC_DMCR_TCL_MASK	(0x03 << EMC_DMCR_TCL_BIT)

#define EMC_RTCSR_CMF		(1 << 7)
#define EMC_RTCSR_CKS_BIT	0
#define EMC_RTCSR_CKS_MASK	(0x07 << EMC_RTCSR_CKS_BIT)
  #define EMC_RTCSR_CKS_DISABLE	(0 << EMC_RTCSR_CKS_BIT)
  #define EMC_RTCSR_CKS_4	(1 << EMC_RTCSR_CKS_BIT)
  #define EMC_RTCSR_CKS_16	(2 << EMC_RTCSR_CKS_BIT)
  #define EMC_RTCSR_CKS_64	(3 << EMC_RTCSR_CKS_BIT)
  #define EMC_RTCSR_CKS_256	(4 << EMC_RTCSR_CKS_BIT)
  #define EMC_RTCSR_CKS_1024	(5 << EMC_RTCSR_CKS_BIT)
  #define EMC_RTCSR_CKS_2048	(6 << EMC_RTCSR_CKS_BIT)
  #define EMC_RTCSR_CKS_4096	(7 << EMC_RTCSR_CKS_BIT)

#define EMC_DMAR_BASE_BIT	8
#define EMC_DMAR_BASE_MASK	(0xff << EMC_DMAR_BASE_BIT)
#define EMC_DMAR_MASK_BIT	0
#define EMC_DMAR_MASK_MASK	(0xff << EMC_DMAR_MASK_BIT)

#define EMC_SDMR_BM		(1 << 9)
#define EMC_SDMR_OM_BIT		7
#define EMC_SDMR_OM_MASK	(3 << EMC_SDMR_OM_BIT)
  #define EMC_SDMR_OM_NORMAL	(0 << EMC_SDMR_OM_BIT)
#define EMC_SDMR_CAS_BIT	4
#define EMC_SDMR_CAS_MASK	(7 << EMC_SDMR_CAS_BIT)
  #define EMC_SDMR_CAS_1	(1 << EMC_SDMR_CAS_BIT)
  #define EMC_SDMR_CAS_2	(2 << EMC_SDMR_CAS_BIT)
  #define EMC_SDMR_CAS_3	(3 << EMC_SDMR_CAS_BIT)
#define EMC_SDMR_BT_BIT		3
#define EMC_SDMR_BT_MASK	(1 << EMC_SDMR_BT_BIT)
  #define EMC_SDMR_BT_SEQ	(0 << EMC_SDMR_BT_BIT)
  #define EMC_SDMR_BT_INTR	(1 << EMC_SDMR_BT_BIT)
#define EMC_SDMR_BL_BIT		0
#define EMC_SDMR_BL_MASK	(7 << EMC_SDMR_BL_BIT)
  #define EMC_SDMR_BL_1		(0 << EMC_SDMR_BL_BIT)
  #define EMC_SDMR_BL_2		(1 << EMC_SDMR_BL_BIT)
  #define EMC_SDMR_BL_4		(2 << EMC_SDMR_BL_BIT)
  #define EMC_SDMR_BL_8		(3 << EMC_SDMR_BL_BIT)

#define EMC_SDMR_CAS2_16BIT \
  (EMC_SDMR_CAS_2 | EMC_SDMR_BT_SEQ | EMC_SDMR_BL_2)
#define EMC_SDMR_CAS2_32BIT \
  (EMC_SDMR_CAS_2 | EMC_SDMR_BT_SEQ | EMC_SDMR_BL_4)
#define EMC_SDMR_CAS3_16BIT \
  (EMC_SDMR_CAS_3 | EMC_SDMR_BT_SEQ | EMC_SDMR_BL_2)
#define EMC_SDMR_CAS3_32BIT \
  (EMC_SDMR_CAS_3 | EMC_SDMR_BT_SEQ | EMC_SDMR_BL_4)

#define EMC_PCCR12_AMW		(1 << 31)
#define EMC_PCCR12_AMAS_BIT	28
#define EMC_PCCR12_AMAS_MASK	(0x07 << EMC_PCCR12_AMAS_BIT)
#define EMC_PCCR12_AMAH_BIT	24
#define EMC_PCCR12_AMAH_MASK	(0x07 << EMC_PCCR12_AMAH_BIT)
#define EMC_PCCR12_AMPW_BIT	20
#define EMC_PCCR12_AMPW_MASK	(0x0f << EMC_PCCR12_AMPW_BIT)
#define EMC_PCCR12_AMRT_BIT	16
#define EMC_PCCR12_AMRT_MASK	(0x0f << EMC_PCCR12_AMRT_BIT)
#define EMC_PCCR12_CMW		(1 << 15)
#define EMC_PCCR12_CMAS_BIT	12
#define EMC_PCCR12_CMAS_MASK	(0x07 << EMC_PCCR12_CMAS_BIT)
#define EMC_PCCR12_CMAH_BIT	8
#define EMC_PCCR12_CMAH_MASK	(0x07 << EMC_PCCR12_CMAH_BIT)
#define EMC_PCCR12_CMPW_BIT	4
#define EMC_PCCR12_CMPW_MASK	(0x0f << EMC_PCCR12_CMPW_BIT)
#define EMC_PCCR12_CMRT_BIT	0
#define EMC_PCCR12_CMRT_MASK	(0x07 << EMC_PCCR12_CMRT_BIT)

#define EMC_PCCR34_DRS_BIT	16
#define EMC_PCCR34_DRS_MASK	(0x03 << EMC_PCCR34_DRS_BIT)
  #define EMC_PCCR34_DRS_SPKR	(1 << EMC_PCCR34_DRS_BIT)
  #define EMC_PCCR34_DRS_IOIS16	(2 << EMC_PCCR34_DRS_BIT)
  #define EMC_PCCR34_DRS_INPACK	(3 << EMC_PCCR34_DRS_BIT)
#define EMC_PCCR34_IOIS16	(1 << 15)
#define EMC_PCCR34_IOW		(1 << 14)
#define EMC_PCCR34_TCB_BIT	12
#define EMC_PCCR34_TCB_MASK	(0x03 << EMC_PCCR34_TCB_BIT)
#define EMC_PCCR34_IORT_BIT	8
#define EMC_PCCR34_IORT_MASK	(0x07 << EMC_PCCR34_IORT_BIT)
#define EMC_PCCR34_IOAE_BIT	6
#define EMC_PCCR34_IOAE_MASK	(0x03 << EMC_PCCR34_IOAE_BIT)
  #define EMC_PCCR34_IOAE_NONE	(0 << EMC_PCCR34_IOAE_BIT)
  #define EMC_PCCR34_IOAE_1	(1 << EMC_PCCR34_IOAE_BIT)
  #define EMC_PCCR34_IOAE_2	(2 << EMC_PCCR34_IOAE_BIT)
  #define EMC_PCCR34_IOAE_5	(3 << EMC_PCCR34_IOAE_BIT)
#define EMC_PCCR34_IOAH_BIT	4
#define EMC_PCCR34_IOAH_MASK	(0x03 << EMC_PCCR34_IOAH_BIT)
  #define EMC_PCCR34_IOAH_NONE	(0 << EMC_PCCR34_IOAH_BIT)
  #define EMC_PCCR34_IOAH_1	(1 << EMC_PCCR34_IOAH_BIT)
  #define EMC_PCCR34_IOAH_2	(2 << EMC_PCCR34_IOAH_BIT)
  #define EMC_PCCR34_IOAH_5	(3 << EMC_PCCR34_IOAH_BIT)
#define EMC_PCCR34_IOPW_BIT	0
#define EMC_PCCR34_IOPW_MASK	(0x0f << EMC_PCCR34_IOPW_BIT)




/*************************************************************************
 * GPIO
 *************************************************************************/
#define GPIO_GPDR(n)	(GPIO_BASE + (0x00 + (n)*0x30))
#define GPIO_GPDIR(n)	(GPIO_BASE + (0x04 + (n)*0x30))
#define GPIO_GPODR(n)	(GPIO_BASE + (0x08 + (n)*0x30))
#define GPIO_GPPUR(n)	(GPIO_BASE + (0x0c + (n)*0x30))
#define GPIO_GPALR(n)	(GPIO_BASE + (0x10 + (n)*0x30))
#define GPIO_GPAUR(n)	(GPIO_BASE + (0x14 + (n)*0x30))
#define GPIO_GPIDLR(n)	(GPIO_BASE + (0x18 + (n)*0x30))
#define GPIO_GPIDUR(n)	(GPIO_BASE + (0x1c + (n)*0x30))
#define GPIO_GPIER(n)	(GPIO_BASE + (0x20 + (n)*0x30))
#define GPIO_GPIMR(n)	(GPIO_BASE + (0x24 + (n)*0x30))
#define GPIO_GPFR(n)	(GPIO_BASE + (0x28 + (n)*0x30))

#define REG_GPIO_GPDR(n)	REG32(GPIO_GPDR((n)))
#define REG_GPIO_GPDIR(n)	REG32(GPIO_GPDIR((n)))
#define REG_GPIO_GPODR(n)	REG32(GPIO_GPODR((n)))
#define REG_GPIO_GPPUR(n)	REG32(GPIO_GPPUR((n)))
#define REG_GPIO_GPALR(n)	REG32(GPIO_GPALR((n)))
#define REG_GPIO_GPAUR(n)	REG32(GPIO_GPAUR((n)))
#define REG_GPIO_GPIDLR(n)	REG32(GPIO_GPIDLR((n)))
#define REG_GPIO_GPIDUR(n)	REG32(GPIO_GPIDUR((n)))
#define REG_GPIO_GPIER(n)	REG32(GPIO_GPIER((n)))
#define REG_GPIO_GPIMR(n)	REG32(GPIO_GPIMR((n)))
#define REG_GPIO_GPFR(n)	REG32(GPIO_GPFR((n)))

#define GPIO_IRQ_LOLEVEL  0
#define GPIO_IRQ_HILEVEL  1
#define GPIO_IRQ_FALLEDG  2
#define GPIO_IRQ_RAISEDG  3

#define IRQ_GPIO_0	48
#define NUM_GPIO	128

#define GPIO_GPDR0      GPIO_GPDR(0)
#define GPIO_GPDR1      GPIO_GPDR(1)
#define GPIO_GPDR2      GPIO_GPDR(2)
#define GPIO_GPDR3      GPIO_GPDR(3)
#define GPIO_GPDIR0     GPIO_GPDIR(0)
#define GPIO_GPDIR1     GPIO_GPDIR(1)
#define GPIO_GPDIR2     GPIO_GPDIR(2)
#define GPIO_GPDIR3     GPIO_GPDIR(3)
#define GPIO_GPODR0     GPIO_GPODR(0)
#define GPIO_GPODR1     GPIO_GPODR(1)
#define GPIO_GPODR2     GPIO_GPODR(2)
#define GPIO_GPODR3     GPIO_GPODR(3)
#define GPIO_GPPUR0     GPIO_GPPUR(0)
#define GPIO_GPPUR1     GPIO_GPPUR(1)
#define GPIO_GPPUR2     GPIO_GPPUR(2)
#define GPIO_GPPUR3     GPIO_GPPUR(3)
#define GPIO_GPALR0     GPIO_GPALR(0)
#define GPIO_GPALR1     GPIO_GPALR(1)
#define GPIO_GPALR2     GPIO_GPALR(2)
#define GPIO_GPALR3     GPIO_GPALR(3)
#define GPIO_GPAUR0     GPIO_GPAUR(0)
#define GPIO_GPAUR1     GPIO_GPAUR(1)
#define GPIO_GPAUR2     GPIO_GPAUR(2)
#define GPIO_GPAUR3     GPIO_GPAUR(3)
#define GPIO_GPIDLR0    GPIO_GPIDLR(0)
#define GPIO_GPIDLR1    GPIO_GPIDLR(1)
#define GPIO_GPIDLR2    GPIO_GPIDLR(2)
#define GPIO_GPIDLR3    GPIO_GPIDLR(3)
#define GPIO_GPIDUR0    GPIO_GPIDUR(0)
#define GPIO_GPIDUR1    GPIO_GPIDUR(1)
#define GPIO_GPIDUR2    GPIO_GPIDUR(2)
#define GPIO_GPIDUR3    GPIO_GPIDUR(3)
#define GPIO_GPIER0     GPIO_GPIER(0)
#define GPIO_GPIER1     GPIO_GPIER(1)
#define GPIO_GPIER2     GPIO_GPIER(2)
#define GPIO_GPIER3     GPIO_GPIER(3)
#define GPIO_GPIMR0     GPIO_GPIMR(0)
#define GPIO_GPIMR1     GPIO_GPIMR(1)
#define GPIO_GPIMR2     GPIO_GPIMR(2)
#define GPIO_GPIMR3     GPIO_GPIMR(3)
#define GPIO_GPFR0      GPIO_GPFR(0)
#define GPIO_GPFR1      GPIO_GPFR(1)
#define GPIO_GPFR2      GPIO_GPFR(2)
#define GPIO_GPFR3      GPIO_GPFR(3)


/*************************************************************************
 * HARB
 *************************************************************************/
#define	HARB_HAPOR		(HARB_BASE + 0x000)
#define	HARB_HMCTR		(HARB_BASE + 0x010)
#define	HARB_HME8H		(HARB_BASE + 0x014)
#define	HARB_HMCR1		(HARB_BASE + 0x018)
#define	HARB_HMER2		(HARB_BASE + 0x01C)
#define	HARB_HMER3		(HARB_BASE + 0x020)
#define	HARB_HMLTR		(HARB_BASE + 0x024)

#define	REG_HARB_HAPOR		REG32(HARB_HAPOR)
#define	REG_HARB_HMCTR		REG32(HARB_HMCTR)
#define	REG_HARB_HME8H		REG32(HARB_HME8H)
#define	REG_HARB_HMCR1		REG32(HARB_HMCR1)
#define	REG_HARB_HMER2		REG32(HARB_HMER2)
#define	REG_HARB_HMER3		REG32(HARB_HMER3)
#define	REG_HARB_HMLTR		REG32(HARB_HMLTR)

/* HARB Priority Order Register (HARB_HAPOR) */

#define	HARB_HAPOR_UCHSEL 		(1 << 7)
#define	HARB_HAPOR_PRIO_BIT		0
#define	HARB_HAPOR_PRIO_MASK		(0xf << HARB_HAPOR_PRIO_BIT)

/* AHB Monitor Control Register (HARB_HMCTR) */

#define	HARB_HMCTR_HET3_BIT		20
#define	HARB_HMCTR_HET3_MASK		(0xf << HARB_HMCTR_HET3_BIT)
#define	HARB_HMCTR_HMS3_BIT		16
#define	HARB_HMCTR_HMS3_MASK		(0xf << HARB_HMCTR_HMS3_BIT)
#define	HARB_HMCTR_HET2_BIT		12
#define	HARB_HMCTR_HET2_MASK		(0xf << HARB_HMCTR_HET2_BIT)
#define	HARB_HMCTR_HMS2_BIT		8
#define	HARB_HMCTR_HMS2_MASK		(0xf << HARB_HMCTR_HMS2_BIT)
#define	HARB_HMCTR_HOVF3		(1 << 7)
#define	HARB_HMCTR_HOVF2		(1 << 6)
#define	HARB_HMCTR_HOVF1		(1 << 5)
#define	HARB_HMCTR_HRST			(1 << 4)
#define	HARB_HMCTR_HEE3			(1 << 2)
#define	HARB_HMCTR_HEE2			(1 << 1)
#define	HARB_HMCTR_HEE1			(1 << 0)

/* AHB Monitor Event 8bits High Register (HARB_HME8H) */

#define HARB_HME8H_HC8H1_BIT		16
#define HARB_HME8H_HC8H1_MASK		(0xff << HARB_HME8H_HC8H1_BIT)
#define HARB_HME8H_HC8H2_BIT		8
#define HARB_HME8H_HC8H2_MASK		(0xff << HARB_HME8H_HC8H2_BIT)
#define HARB_HME8H_HC8H3_BIT		0
#define HARB_HME8H_HC8H3_MASK		(0xff << HARB_HME8H_HC8H3_BIT)

/* AHB Monitor Latency Register (HARB_HMLTR) */

#define HARB_HMLTR_HLT2_BIT		16
#define HARB_HMLTR_HLT2_MASK		(0xffff << HARB_HMLTR_HLT2_BIT)
#define HARB_HMLTR_HLT3_BIT		0
#define HARB_HMLTR_HLT3_MASK		(0xffff << HARB_HMLTR_HLT3_BIT)




/*************************************************************************
 * I2C
 *************************************************************************/
#define	I2C_DR			(I2C_BASE + 0x000)
#define	I2C_CR			(I2C_BASE + 0x004)
#define	I2C_SR			(I2C_BASE + 0x008)
#define	I2C_GR			(I2C_BASE + 0x00C)

#define	REG_I2C_DR		REG8(I2C_DR)
#define	REG_I2C_CR		REG8(I2C_CR)
#define REG_I2C_SR		REG8(I2C_SR)
#define REG_I2C_GR		REG16(I2C_GR)

/* I2C Control Register (I2C_CR) */

#define I2C_CR_IEN		(1 << 4)
#define I2C_CR_STA		(1 << 3)
#define I2C_CR_STO		(1 << 2)
#define I2C_CR_AC		(1 << 1)
#define I2C_CR_I2CE		(1 << 0)

/* I2C Status Register (I2C_SR) */

#define I2C_SR_STX		(1 << 4)
#define I2C_SR_BUSY		(1 << 3)
#define I2C_SR_TEND		(1 << 2)
#define I2C_SR_DRF		(1 << 1)
#define I2C_SR_ACKF		(1 << 0)




/*************************************************************************
 * UDC
 *************************************************************************/
#define UDC_EP0InCR	(UDC_BASE + 0x00)
#define UDC_EP0InSR	(UDC_BASE + 0x04)
#define UDC_EP0InBSR	(UDC_BASE + 0x08)
#define UDC_EP0InMPSR	(UDC_BASE + 0x0c)
#define UDC_EP0InDesR	(UDC_BASE + 0x14)
#define UDC_EP1InCR	(UDC_BASE + 0x20)
#define UDC_EP1InSR	(UDC_BASE + 0x24)
#define UDC_EP1InBSR	(UDC_BASE + 0x28)
#define UDC_EP1InMPSR	(UDC_BASE + 0x2c)
#define UDC_EP1InDesR	(UDC_BASE + 0x34)
#define UDC_EP2InCR	(UDC_BASE + 0x40)
#define UDC_EP2InSR	(UDC_BASE + 0x44)
#define UDC_EP2InBSR	(UDC_BASE + 0x48)
#define UDC_EP2InMPSR	(UDC_BASE + 0x4c)
#define UDC_EP2InDesR	(UDC_BASE + 0x54)
#define UDC_EP3InCR	(UDC_BASE + 0x60)
#define UDC_EP3InSR	(UDC_BASE + 0x64)
#define UDC_EP3InBSR	(UDC_BASE + 0x68)
#define UDC_EP3InMPSR	(UDC_BASE + 0x6c)
#define UDC_EP3InDesR	(UDC_BASE + 0x74)
#define UDC_EP4InCR	(UDC_BASE + 0x80)
#define UDC_EP4InSR	(UDC_BASE + 0x84)
#define UDC_EP4InBSR	(UDC_BASE + 0x88)
#define UDC_EP4InMPSR	(UDC_BASE + 0x8c)
#define UDC_EP4InDesR	(UDC_BASE + 0x94)

#define UDC_EP0OutCR	(UDC_BASE + 0x200)
#define UDC_EP0OutSR	(UDC_BASE + 0x204)
#define UDC_EP0OutPFNR	(UDC_BASE + 0x208)
#define UDC_EP0OutMPSR	(UDC_BASE + 0x20c)
#define UDC_EP0OutSBPR	(UDC_BASE + 0x210)
#define UDC_EP0OutDesR	(UDC_BASE + 0x214)
#define UDC_EP5OutCR	(UDC_BASE + 0x2a0)
#define UDC_EP5OutSR	(UDC_BASE + 0x2a4)
#define UDC_EP5OutPFNR	(UDC_BASE + 0x2a8)
#define UDC_EP5OutMPSR	(UDC_BASE + 0x2ac)
#define UDC_EP5OutDesR	(UDC_BASE + 0x2b4)
#define UDC_EP6OutCR	(UDC_BASE + 0x2c0)
#define UDC_EP6OutSR	(UDC_BASE + 0x2c4)
#define UDC_EP6OutPFNR	(UDC_BASE + 0x2c8)
#define UDC_EP6OutMPSR	(UDC_BASE + 0x2cc)
#define UDC_EP6OutDesR	(UDC_BASE + 0x2d4)
#define UDC_EP7OutCR	(UDC_BASE + 0x2e0)
#define UDC_EP7OutSR	(UDC_BASE + 0x2e4)
#define UDC_EP7OutPFNR	(UDC_BASE + 0x2e8)
#define UDC_EP7OutMPSR	(UDC_BASE + 0x2ec)
#define UDC_EP7OutDesR	(UDC_BASE + 0x2f4)

#define UDC_DevCFGR	(UDC_BASE + 0x400)
#define UDC_DevCR	(UDC_BASE + 0x404)
#define UDC_DevSR	(UDC_BASE + 0x408)
#define UDC_DevIntR	(UDC_BASE + 0x40c)
#define UDC_DevIntMR	(UDC_BASE + 0x410)
#define UDC_EPIntR	(UDC_BASE + 0x414)
#define UDC_EPIntMR	(UDC_BASE + 0x418)

#define UDC_STCMAR	(UDC_BASE + 0x500)
#define UDC_EP0InfR	(UDC_BASE + 0x504)
#define UDC_EP1InfR	(UDC_BASE + 0x508)
#define UDC_EP2InfR	(UDC_BASE + 0x50c)
#define UDC_EP3InfR	(UDC_BASE + 0x510)
#define UDC_EP4InfR	(UDC_BASE + 0x514)
#define UDC_EP5InfR	(UDC_BASE + 0x518)
#define UDC_EP6InfR	(UDC_BASE + 0x51c)
#define UDC_EP7InfR	(UDC_BASE + 0x520)

#define UDC_TXCONFIRM	(UDC_BASE + 0x41C)
#define UDC_TXZLP	(UDC_BASE + 0x420)
#define UDC_RXCONFIRM	(UDC_BASE + 0x41C)

#define UDC_RXFIFO	(UDC_BASE + 0x800)
#define UDC_TXFIFOEP0	(UDC_BASE + 0x840)

#define REG_UDC_EP0InCR		REG32(UDC_EP0InCR)
#define REG_UDC_EP0InSR		REG32(UDC_EP0InSR)
#define REG_UDC_EP0InBSR	REG32(UDC_EP0InBSR)
#define REG_UDC_EP0InMPSR	REG32(UDC_EP0InMPSR)
#define REG_UDC_EP0InDesR	REG32(UDC_EP0InDesR)
#define REG_UDC_EP1InCR		REG32(UDC_EP1InCR)
#define REG_UDC_EP1InSR		REG32(UDC_EP1InSR)
#define REG_UDC_EP1InBSR	REG32(UDC_EP1InBSR)
#define REG_UDC_EP1InMPSR	REG32(UDC_EP1InMPSR)
#define REG_UDC_EP1InDesR	REG32(UDC_EP1InDesR)
#define REG_UDC_EP2InCR		REG32(UDC_EP2InCR)
#define REG_UDC_EP2InSR		REG32(UDC_EP2InSR)
#define REG_UDC_EP2InBSR	REG32(UDC_EP2InBSR)
#define REG_UDC_EP2InMPSR	REG32(UDC_EP2InMPSR)
#define REG_UDC_EP2InDesR	REG32(UDC_EP2InDesR)
#define REG_UDC_EP3InCR		REG32(UDC_EP3InCR)
#define REG_UDC_EP3InSR		REG32(UDC_EP3InSR)
#define REG_UDC_EP3InBSR	REG32(UDC_EP3InBSR)
#define REG_UDC_EP3InMPSR	REG32(UDC_EP3InMPSR)
#define REG_UDC_EP3InDesR	REG32(UDC_EP3InDesR)
#define REG_UDC_EP4InCR		REG32(UDC_EP4InCR)
#define REG_UDC_EP4InSR		REG32(UDC_EP4InSR)
#define REG_UDC_EP4InBSR	REG32(UDC_EP4InBSR)
#define REG_UDC_EP4InMPSR	REG32(UDC_EP4InMPSR)
#define REG_UDC_EP4InDesR	REG32(UDC_EP4InDesR)

#define REG_UDC_EP0OutCR	REG32(UDC_EP0OutCR)
#define REG_UDC_EP0OutSR	REG32(UDC_EP0OutSR)
#define REG_UDC_EP0OutPFNR	REG32(UDC_EP0OutPFNR)
#define REG_UDC_EP0OutMPSR	REG32(UDC_EP0OutMPSR)
#define REG_UDC_EP0OutSBPR	REG32(UDC_EP0OutSBPR)
#define REG_UDC_EP0OutDesR	REG32(UDC_EP0OutDesR)
#define REG_UDC_EP5OutCR	REG32(UDC_EP5OutCR)
#define REG_UDC_EP5OutSR	REG32(UDC_EP5OutSR)
#define REG_UDC_EP5OutPFNR	REG32(UDC_EP5OutPFNR)
#define REG_UDC_EP5OutMPSR	REG32(UDC_EP5OutMPSR)
#define REG_UDC_EP5OutDesR	REG32(UDC_EP5OutDesR)
#define REG_UDC_EP6OutCR	REG32(UDC_EP6OutCR)
#define REG_UDC_EP6OutSR	REG32(UDC_EP6OutSR)
#define REG_UDC_EP6OutPFNR	REG32(UDC_EP6OutPFNR)
#define REG_UDC_EP6OutMPSR	REG32(UDC_EP6OutMPSR)
#define REG_UDC_EP6OutDesR	REG32(UDC_EP6OutDesR)
#define REG_UDC_EP7OutCR	REG32(UDC_EP7OutCR)
#define REG_UDC_EP7OutSR	REG32(UDC_EP7OutSR)
#define REG_UDC_EP7OutPFNR	REG32(UDC_EP7OutPFNR)
#define REG_UDC_EP7OutMPSR	REG32(UDC_EP7OutMPSR)
#define REG_UDC_EP7OutDesR	REG32(UDC_EP7OutDesR)

#define REG_UDC_DevCFGR		REG32(UDC_DevCFGR)
#define REG_UDC_DevCR		REG32(UDC_DevCR)
#define REG_UDC_DevSR		REG32(UDC_DevSR)
#define REG_UDC_DevIntR		REG32(UDC_DevIntR)
#define REG_UDC_DevIntMR	REG32(UDC_DevIntMR)
#define REG_UDC_EPIntR		REG32(UDC_EPIntR)
#define REG_UDC_EPIntMR		REG32(UDC_EPIntMR)

#define REG_UDC_STCMAR		REG32(UDC_STCMAR)
#define REG_UDC_EP0InfR		REG32(UDC_EP0InfR)
#define REG_UDC_EP1InfR		REG32(UDC_EP1InfR)
#define REG_UDC_EP2InfR		REG32(UDC_EP2InfR)
#define REG_UDC_EP3InfR		REG32(UDC_EP3InfR)
#define REG_UDC_EP4InfR		REG32(UDC_EP4InfR)
#define REG_UDC_EP5InfR		REG32(UDC_EP5InfR)
#define REG_UDC_EP6InfR		REG32(UDC_EP6InfR)
#define REG_UDC_EP7InfR		REG32(UDC_EP7InfR)

#define UDC_DevCFGR_PI		(1 << 5)
#define UDC_DevCFGR_SS		(1 << 4)
#define UDC_DevCFGR_SP		(1 << 3)
#define UDC_DevCFGR_RW		(1 << 2)
#define UDC_DevCFGR_SPD_BIT	0
#define UDC_DevCFGR_SPD_MASK	(0x03 << UDC_DevCFGR_SPD_BIT)
  #define UDC_DevCFGR_SPD_HS	(0 << UDC_DevCFGR_SPD_BIT)
  #define UDC_DevCFGR_SPD_LS	(2 << UDC_DevCFGR_SPD_BIT)
  #define UDC_DevCFGR_SPD_FS	(3 << UDC_DevCFGR_SPD_BIT)

#define UDC_DevCR_DM		(1 << 9)
#define UDC_DevCR_BE		(1 << 5)
#define UDC_DevCR_RES		(1 << 0)

#define UDC_DevSR_ENUMSPD_BIT	13
#define UDC_DevSR_ENUMSPD_MASK	(0x03 << UDC_DevSR_ENUMSPD_BIT)
  #define UDC_DevSR_ENUMSPD_HS	(0 << UDC_DevSR_ENUMSPD_BIT)
  #define UDC_DevSR_ENUMSPD_LS	(2 << UDC_DevSR_ENUMSPD_BIT)
  #define UDC_DevSR_ENUMSPD_FS	(3 << UDC_DevSR_ENUMSPD_BIT)
#define UDC_DevSR_SUSP		(1 << 12)
#define UDC_DevSR_ALT_BIT	8
#define UDC_DevSR_ALT_MASK	(0x0f << UDC_DevSR_ALT_BIT)
#define UDC_DevSR_INTF_BIT	4
#define UDC_DevSR_INTF_MASK	(0x0f << UDC_DevSR_INTF_BIT)
#define UDC_DevSR_CFG_BIT	0
#define UDC_DevSR_CFG_MASK	(0x0f << UDC_DevSR_CFG_BIT)

#define UDC_DevIntR_ENUM	(1 << 6)
#define UDC_DevIntR_SOF		(1 << 5)
#define UDC_DevIntR_US		(1 << 4)
#define UDC_DevIntR_UR		(1 << 3)
#define UDC_DevIntR_SI		(1 << 1)
#define UDC_DevIntR_SC		(1 << 0)

#define UDC_EPIntR_OUTEP_BIT	16
#define UDC_EPIntR_OUTEP_MASK	(0xffff << UDC_EPIntR_OUTEP_BIT)
#define UDC_EPIntR_OUTEP0       0x00010000
#define UDC_EPIntR_OUTEP5       0x00200000
#define UDC_EPIntR_OUTEP6       0x00400000
#define UDC_EPIntR_OUTEP7       0x00800000
#define UDC_EPIntR_INEP_BIT	0
#define UDC_EPIntR_INEP_MASK	(0xffff << UDC_EPIntR_INEP_BIT)
#define UDC_EPIntR_INEP0        0x00000001
#define UDC_EPIntR_INEP1        0x00000002
#define UDC_EPIntR_INEP2        0x00000004
#define UDC_EPIntR_INEP3        0x00000008
#define UDC_EPIntR_INEP4        0x00000010


#define UDC_EPIntMR_OUTEP_BIT	16
#define UDC_EPIntMR_OUTEP_MASK	(0xffff << UDC_EPIntMR_OUTEP_BIT)
#define UDC_EPIntMR_INEP_BIT	0
#define UDC_EPIntMR_INEP_MASK	(0xffff << UDC_EPIntMR_INEP_BIT)

#define UDC_EPCR_ET_BIT		4
#define UDC_EPCR_ET_MASK	(0x03 << UDC_EPCR_ET_BIT)
  #define UDC_EPCR_ET_CTRL	(0 << UDC_EPCR_ET_BIT)
  #define UDC_EPCR_ET_ISO	(1 << UDC_EPCR_ET_BIT)
  #define UDC_EPCR_ET_BULK	(2 << UDC_EPCR_ET_BIT)
  #define UDC_EPCR_ET_INTR	(3 << UDC_EPCR_ET_BIT)
#define UDC_EPCR_SN		(1 << 2)
#define UDC_EPCR_F		(1 << 1)
#define UDC_EPCR_S		(1 << 0)

#define UDC_EPSR_RXPKTSIZE_BIT	11
#define UDC_EPSR_RXPKTSIZE_MASK	(0x7ff << UDC_EPSR_RXPKTSIZE_BIT)
#define UDC_EPSR_IN		(1 << 6)
#define UDC_EPSR_OUT_BIT	4
#define UDC_EPSR_OUT_MASK	(0x03 << UDC_EPSR_OUT_BIT)
  #define UDC_EPSR_OUT_NONE	(0 << UDC_EPSR_OUT_BIT)
  #define UDC_EPSR_OUT_RCVDATA	(1 << UDC_EPSR_OUT_BIT)
  #define UDC_EPSR_OUT_RCVSETUP	(2 << UDC_EPSR_OUT_BIT)
#define UDC_EPSR_PID_BIT	0
#define UDC_EPSR_PID_MASK	(0x0f << UDC_EPSR_PID_BIT)

#define UDC_EPInfR_MPS_BIT	19
#define UDC_EPInfR_MPS_MASK	(0x3ff << UDC_EPInfR_MPS_BIT)
#define UDC_EPInfR_ALTS_BIT	15
#define UDC_EPInfR_ALTS_MASK	(0x0f << UDC_EPInfR_ALTS_BIT)
#define UDC_EPInfR_IFN_BIT	11
#define UDC_EPInfR_IFN_MASK	(0x0f << UDC_EPInfR_IFN_BIT)
#define UDC_EPInfR_CGN_BIT	7
#define UDC_EPInfR_CGN_MASK	(0x0f << UDC_EPInfR_CGN_BIT)
#define UDC_EPInfR_EPT_BIT	5
#define UDC_EPInfR_EPT_MASK	(0x03 << UDC_EPInfR_EPT_BIT)
  #define UDC_EPInfR_EPT_CTRL	(0 << UDC_EPInfR_EPT_BIT)
  #define UDC_EPInfR_EPT_ISO	(1 << UDC_EPInfR_EPT_BIT)
  #define UDC_EPInfR_EPT_BULK	(2 << UDC_EPInfR_EPT_BIT)
  #define UDC_EPInfR_EPT_INTR	(3 << UDC_EPInfR_EPT_BIT)
#define UDC_EPInfR_EPD		(1 << 4)
  #define UDC_EPInfR_EPD_OUT	(0 << 4)
  #define UDC_EPInfR_EPD_IN	(1 << 4)

#define UDC_EPInfR_EPN_BIT	0
#define UDC_EPInfR_EPN_MASK	(0xf << UDC_EPInfR_EPN_BIT)




/*************************************************************************
 * DMAC 
 *************************************************************************/
#define DMAC_DSAR(n)	(DMAC_BASE + (0x00 + (n) * 0x20))
#define DMAC_DDAR(n)	(DMAC_BASE + (0x04 + (n) * 0x20))
#define DMAC_DTCR(n)	(DMAC_BASE + (0x08 + (n) * 0x20))
#define DMAC_DRSR(n)	(DMAC_BASE + (0x0c + (n) * 0x20))
#define DMAC_DCCSR(n)	(DMAC_BASE + (0x10 + (n) * 0x20))
#define DMAC_DMAIPR	(DMAC_BASE + 0xf8)
#define DMAC_DMACR	(DMAC_BASE + 0xfc)

#define REG_DMAC_DSAR(n)	REG32(DMAC_DSAR((n)))
#define REG_DMAC_DDAR(n)	REG32(DMAC_DDAR((n)))
#define REG_DMAC_DTCR(n)	REG32(DMAC_DTCR((n)))
#define REG_DMAC_DRSR(n)	REG32(DMAC_DRSR((n)))
#define REG_DMAC_DCCSR(n)	REG32(DMAC_DCCSR((n)))
#define REG_DMAC_DMAIPR		REG32(DMAC_DMAIPR)
#define REG_DMAC_DMACR		REG32(DMAC_DMACR)

#define DMAC_DRSR_RS_BIT	0
#define DMAC_DRSR_RS_MASK	(0x1f << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_EXTREXTR		(0 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_PCMCIAOUT	(4 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_PCMCIAIN		(5 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_AUTO		(8 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_DESOUT		(10 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_DESIN		(11 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART3OUT		(14 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART3IN		(15 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART2OUT		(16 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART2IN		(17 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART1OUT		(18 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART1IN		(19 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART0OUT		(20 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART0IN		(21 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_SSIOUT		(22 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_SSIIN		(23 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_AICOUT		(24 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_AICIN		(25 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_MSCOUT		(26 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_MSCIN		(27 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_OST2		(28 << DMAC_DRSR_RS_BIT)

#define DMAC_DCCSR_EACKS	(1 << 31)
#define DMAC_DCCSR_EACKM	(1 << 30)
#define DMAC_DCCSR_ERDM_BIT	28
#define DMAC_DCCSR_ERDM_MASK	(0x03 << DMAC_DCCSR_ERDM_BIT)
  #define DMAC_DCCSR_ERDM_LLEVEL	(0 << DMAC_DCCSR_ERDM_BIT)
  #define DMAC_DCCSR_ERDM_FEDGE		(1 << DMAC_DCCSR_ERDM_BIT)
  #define DMAC_DCCSR_ERDM_HLEVEL	(2 << DMAC_DCCSR_ERDM_BIT)
  #define DMAC_DCCSR_ERDM_REDGE		(3 << DMAC_DCCSR_ERDM_BIT)
#define DMAC_DCCSR_EOPM		(1 << 27)
#define DMAC_DCCSR_SAM		(1 << 23)
#define DMAC_DCCSR_DAM		(1 << 22)
#define DMAC_DCCSR_RDIL_BIT	16
#define DMAC_DCCSR_RDIL_MASK	(0x0f << DMAC_DCCSR_RDIL_BIT)
  #define DMAC_DCCSR_RDIL_IGN	(0 << DMAC_DCCSR_RDIL_BIT)
  #define DMAC_DCCSR_RDIL_2	(1 << DMAC_DCCSR_RDIL_BIT)
  #define DMAC_DCCSR_RDIL_4	(2 << DMAC_DCCSR_RDIL_BIT)
  #define DMAC_DCCSR_RDIL_8	(3 << DMAC_DCCSR_RDIL_BIT)
  #define DMAC_DCCSR_RDIL_12	(4 << DMAC_DCCSR_RDIL_BIT)
  #define DMAC_DCCSR_RDIL_16	(5 << DMAC_DCCSR_RDIL_BIT)
  #define DMAC_DCCSR_RDIL_20	(6 << DMAC_DCCSR_RDIL_BIT)
  #define DMAC_DCCSR_RDIL_24	(7 << DMAC_DCCSR_RDIL_BIT)
  #define DMAC_DCCSR_RDIL_28	(8 << DMAC_DCCSR_RDIL_BIT)
  #define DMAC_DCCSR_RDIL_32	(9 << DMAC_DCCSR_RDIL_BIT)
  #define DMAC_DCCSR_RDIL_48	(10 << DMAC_DCCSR_RDIL_BIT)
  #define DMAC_DCCSR_RDIL_60	(11 << DMAC_DCCSR_RDIL_BIT)
  #define DMAC_DCCSR_RDIL_64	(12 << DMAC_DCCSR_RDIL_BIT)
  #define DMAC_DCCSR_RDIL_124	(13 << DMAC_DCCSR_RDIL_BIT)
  #define DMAC_DCCSR_RDIL_128	(14 << DMAC_DCCSR_RDIL_BIT)
  #define DMAC_DCCSR_RDIL_200	(15 << DMAC_DCCSR_RDIL_BIT)
#define DMAC_DCCSR_SWDH_BIT	14
#define DMAC_DCCSR_SWDH_MASK	(0x03 << DMAC_DCCSR_SWDH_BIT)
  #define DMAC_DCCSR_SWDH_32	(0 << DMAC_DCCSR_SWDH_BIT)
  #define DMAC_DCCSR_SWDH_8	(1 << DMAC_DCCSR_SWDH_BIT)
  #define DMAC_DCCSR_SWDH_16	(2 << DMAC_DCCSR_SWDH_BIT)
#define DMAC_DCCSR_DWDH_BIT	12
#define DMAC_DCCSR_DWDH_MASK	(0x03 << DMAC_DCCSR_DWDH_BIT)
  #define DMAC_DCCSR_DWDH_32	(0 << DMAC_DCCSR_DWDH_BIT)
  #define DMAC_DCCSR_DWDH_8	(1 << DMAC_DCCSR_DWDH_BIT)
  #define DMAC_DCCSR_DWDH_16	(2 << DMAC_DCCSR_DWDH_BIT)
#define DMAC_DCCSR_DS_BIT	8
#define DMAC_DCCSR_DS_MASK	(0x07 << DMAC_DCCSR_DS_BIT)
  #define DMAC_DCCSR_DS_32b	(0 << DMAC_DCCSR_DS_BIT)
  #define DMAC_DCCSR_DS_8b	(1 << DMAC_DCCSR_DS_BIT)
  #define DMAC_DCCSR_DS_16b	(2 << DMAC_DCCSR_DS_BIT)
  #define DMAC_DCCSR_DS_16B	(3 << DMAC_DCCSR_DS_BIT)
  #define DMAC_DCCSR_DS_32B	(4 << DMAC_DCCSR_DS_BIT)
#define DMAC_DCCSR_TM		(1 << 7)
#define DMAC_DCCSR_AR		(1 << 4)
#define DMAC_DCCSR_TC		(1 << 3)
#define DMAC_DCCSR_HLT		(1 << 2)
#define DMAC_DCCSR_TCIE		(1 << 1)
#define DMAC_DCCSR_CHDE		(1 << 0)

#define DMAC_DMAIPR_CINT_BIT	8
#define DMAC_DMAIPR_CINT_MASK	(0xff << DMAC_DMAIPR_CINT_BIT)

#define DMAC_DMACR_PR_BIT	8
#define DMAC_DMACR_PR_MASK	(0x03 << DMAC_DMACR_PR_BIT)
  #define DMAC_DMACR_PR_01234567	(0 << DMAC_DMACR_PR_BIT)
  #define DMAC_DMACR_PR_02314675	(1 << DMAC_DMACR_PR_BIT)
  #define DMAC_DMACR_PR_20136457	(2 << DMAC_DMACR_PR_BIT)
  #define DMAC_DMACR_PR_ROUNDROBIN	(3 << DMAC_DMACR_PR_BIT)
#define DMAC_DMACR_HTR		(1 << 3)
#define DMAC_DMACR_AER		(1 << 2)
#define DMAC_DMACR_DME		(1 << 0)

#define IRQ_DMA_0	32
#define NUM_DMA		6

#define DMAC_DSAR0      DMAC_DSAR(0)
#define DMAC_DDAR0      DMAC_DDAR(0)
#define DMAC_DTCR0      DMAC_DTCR(0)
#define DMAC_DRSR0      DMAC_DRSR(0)
#define DMAC_DCCSR0     DMAC_DCCSR(0)

#define DMAC_DSAR1      DMAC_DSAR(1)
#define DMAC_DDAR1      DMAC_DDAR(1)
#define DMAC_DTCR1      DMAC_DTCR(1)
#define DMAC_DRSR1      DMAC_DRSR(1)
#define DMAC_DCCSR1     DMAC_DCCSR(1)

#define DMAC_DSAR2      DMAC_DSAR(2)
#define DMAC_DDAR2      DMAC_DDAR(2)
#define DMAC_DTCR2      DMAC_DTCR(2)
#define DMAC_DRSR2      DMAC_DRSR(2)
#define DMAC_DCCSR2     DMAC_DCCSR(2)

#define DMAC_DSAR3      DMAC_DSAR(3)
#define DMAC_DDAR3      DMAC_DDAR(3)
#define DMAC_DTCR3      DMAC_DTCR(3)
#define DMAC_DRSR3      DMAC_DRSR(3)
#define DMAC_DCCSR3     DMAC_DCCSR(3)

#define DMAC_DSAR4      DMAC_DSAR(4)
#define DMAC_DDAR4      DMAC_DDAR(4)
#define DMAC_DTCR4      DMAC_DTCR(4)
#define DMAC_DRSR4      DMAC_DRSR(4)
#define DMAC_DCCSR4     DMAC_DCCSR(4)

#define DMAC_DSAR5      DMAC_DSAR(5)
#define DMAC_DDAR5      DMAC_DDAR(5)
#define DMAC_DTCR5      DMAC_DTCR(5)
#define DMAC_DRSR5      DMAC_DRSR(5)
#define DMAC_DCCSR5     DMAC_DCCSR(5)

#define DMAC_DSAR6      DMAC_DSAR(6)
#define DMAC_DDAR6      DMAC_DDAR(6)
#define DMAC_DTCR6      DMAC_DTCR(6)
#define DMAC_DRSR6      DMAC_DRSR(6)
#define DMAC_DCCSR6     DMAC_DCCSR(6)

#define DMAC_DSAR7      DMAC_DSAR(7)
#define DMAC_DDAR7      DMAC_DDAR(7)
#define DMAC_DTCR7      DMAC_DTCR(7)
#define DMAC_DRSR7      DMAC_DRSR(7)
#define DMAC_DCCSR7     DMAC_DCCSR(7)



/*************************************************************************
 * AIC 
 *************************************************************************/
#define	AIC_FR			(AIC_BASE + 0x000)
#define	AIC_CR			(AIC_BASE + 0x004)
#define	AIC_ACCR1		(AIC_BASE + 0x008)
#define	AIC_ACCR2		(AIC_BASE + 0x00C)
#define	AIC_I2SCR		(AIC_BASE + 0x010)
#define	AIC_SR			(AIC_BASE + 0x014)
#define	AIC_ACSR		(AIC_BASE + 0x018)
#define	AIC_I2SSR		(AIC_BASE + 0x01C)
#define	AIC_ACCAR		(AIC_BASE + 0x020)
#define	AIC_ACCDR		(AIC_BASE + 0x024)
#define	AIC_ACSAR		(AIC_BASE + 0x028)
#define	AIC_ACSDR		(AIC_BASE + 0x02C)
#define	AIC_I2SDIV		(AIC_BASE + 0x030)
#define	AIC_DR			(AIC_BASE + 0x034)

#define	REG_AIC_FR		REG32(AIC_FR)
#define	REG_AIC_CR		REG32(AIC_CR)
#define	REG_AIC_ACCR1		REG32(AIC_ACCR1)
#define	REG_AIC_ACCR2		REG32(AIC_ACCR2)
#define	REG_AIC_I2SCR		REG32(AIC_I2SCR)
#define	REG_AIC_SR		REG32(AIC_SR)
#define	REG_AIC_ACSR		REG32(AIC_ACSR)
#define	REG_AIC_I2SSR		REG32(AIC_I2SSR)
#define	REG_AIC_ACCAR		REG32(AIC_ACCAR)
#define	REG_AIC_ACCDR		REG32(AIC_ACCDR)
#define	REG_AIC_ACSAR		REG32(AIC_ACSAR)
#define	REG_AIC_ACSDR		REG32(AIC_ACSDR)
#define	REG_AIC_I2SDIV		REG32(AIC_I2SDIV)
#define	REG_AIC_DR		REG32(AIC_DR)

/* AIC Controller Configuration Register (AIC_FR) */

#define	AIC_FR_RFTH_BIT		12
#define	AIC_FR_RFTH_MASK	(0xf << AIC_FR_RFTH_BIT)
#define	AIC_FR_TFTH_BIT		8
#define	AIC_FR_TFTH_MASK	(0xf << AIC_FR_TFTH_BIT)
#define	AIC_FR_AUSEL		(1 << 4)
#define	AIC_FR_RST		(1 << 3)
#define	AIC_FR_BCKD		(1 << 2)
#define	AIC_FR_SYNCD		(1 << 1)
#define	AIC_FR_ENB		(1 << 0)

/* AIC Controller Common Control Register (AIC_CR) */

#define	AIC_CR_RDMS		(1 << 15)
#define	AIC_CR_TDMS		(1 << 14)
#define	AIC_CR_FLUSH		(1 << 8)
#define	AIC_CR_EROR		(1 << 6)
#define	AIC_CR_ETUR		(1 << 5)
#define	AIC_CR_ERFS		(1 << 4)
#define	AIC_CR_ETFS		(1 << 3)
#define	AIC_CR_ENLBF		(1 << 2)
#define	AIC_CR_ERPL		(1 << 1)
#define	AIC_CR_EREC		(1 << 0)

/* AIC Controller AC-link Control Register 1 (AIC_ACCR1) */

#define	AIC_ACCR1_RS_BIT	16
#define	AIC_ACCR1_RS_MASK	(0x3ff << AIC_ACCR1_RS_BIT)
  #define AIC_ACCR1_RS_SLOT12	  (1 << 25) /* Slot 12 valid bit */
  #define AIC_ACCR1_RS_SLOT11	  (1 << 24) /* Slot 11 valid bit */
  #define AIC_ACCR1_RS_SLOT10	  (1 << 23) /* Slot 10 valid bit */
  #define AIC_ACCR1_RS_SLOT9	  (1 << 22) /* Slot 9 valid bit */
  #define AIC_ACCR1_RS_SLOT8	  (1 << 21) /* Slot 8 valid bit */
  #define AIC_ACCR1_RS_SLOT7	  (1 << 20) /* Slot 7 valid bit */
  #define AIC_ACCR1_RS_SLOT6	  (1 << 19) /* Slot 6 valid bit */
  #define AIC_ACCR1_RS_SLOT5	  (1 << 18) /* Slot 5 valid bit */
  #define AIC_ACCR1_RS_SLOT4	  (1 << 17) /* Slot 4 valid bit */
  #define AIC_ACCR1_RS_SLOT3	  (1 << 16) /* Slot 3 valid bit */
#define	AIC_ACCR1_XS_BIT	0
#define	AIC_ACCR1_XS_MASK	(0x3ff << AIC_ACCR1_XS_BIT)
  #define AIC_ACCR1_XS_SLOT12	  (1 << 9) /* Slot 12 valid bit */
  #define AIC_ACCR1_XS_SLOT11	  (1 << 8) /* Slot 11 valid bit */
  #define AIC_ACCR1_XS_SLOT10	  (1 << 7) /* Slot 10 valid bit */
  #define AIC_ACCR1_XS_SLOT9	  (1 << 6) /* Slot 9 valid bit */
  #define AIC_ACCR1_XS_SLOT8	  (1 << 5) /* Slot 8 valid bit */
  #define AIC_ACCR1_XS_SLOT7	  (1 << 4) /* Slot 7 valid bit */
  #define AIC_ACCR1_XS_SLOT6	  (1 << 3) /* Slot 6 valid bit */
  #define AIC_ACCR1_XS_SLOT5	  (1 << 2) /* Slot 5 valid bit */
  #define AIC_ACCR1_XS_SLOT4	  (1 << 1) /* Slot 4 valid bit */
  #define AIC_ACCR1_XS_SLOT3	  (1 << 0) /* Slot 3 valid bit */

/* AIC Controller AC-link Control Register 2 (AIC_ACCR2) */

#define	AIC_ACCR2_ERSTO		(1 << 18)
#define	AIC_ACCR2_ESADR		(1 << 17)
#define	AIC_ACCR2_ECADT		(1 << 16)
#define	AIC_ACCR2_OASS_BIT	8
#define	AIC_ACCR2_OASS_MASK	(0x3 << AIC_ACCR2_OASS_BIT)
  #define AIC_ACCR2_OASS_20BIT	  (0 << AIC_ACCR2_OASS_BIT) /* Output Audio Sample Size is 20-bit */
  #define AIC_ACCR2_OASS_18BIT	  (1 << AIC_ACCR2_OASS_BIT) /* Output Audio Sample Size is 18-bit */
  #define AIC_ACCR2_OASS_16BIT	  (2 << AIC_ACCR2_OASS_BIT) /* Output Audio Sample Size is 16-bit */
  #define AIC_ACCR2_OASS_8BIT	  (3 << AIC_ACCR2_OASS_BIT) /* Output Audio Sample Size is 8-bit */
#define	AIC_ACCR2_IASS_BIT	6
#define	AIC_ACCR2_IASS_MASK	(0x3 << AIC_ACCR2_IASS_BIT)
  #define AIC_ACCR2_IASS_20BIT	  (0 << AIC_ACCR2_IASS_BIT) /* Input Audio Sample Size is 20-bit */
  #define AIC_ACCR2_IASS_18BIT	  (1 << AIC_ACCR2_IASS_BIT) /* Input Audio Sample Size is 18-bit */
  #define AIC_ACCR2_IASS_16BIT	  (2 << AIC_ACCR2_IASS_BIT) /* Input Audio Sample Size is 16-bit */
  #define AIC_ACCR2_IASS_8BIT	  (3 << AIC_ACCR2_IASS_BIT) /* Input Audio Sample Size is 8-bit */
#define	AIC_ACCR2_SO		(1 << 3)
#define	AIC_ACCR2_SR		(1 << 2)
#define	AIC_ACCR2_SS		(1 << 1)
#define	AIC_ACCR2_SA		(1 << 0)

/* AIC Controller I2S/MSB-justified Control Register (AIC_I2SCR) */

#define	AIC_I2SCR_STPBK		(1 << 12)
#define	AIC_I2SCR_WL_BIT	1
#define	AIC_I2SCR_WL_MASK	(0x7 << AIC_I2SCR_WL_BIT)
  #define AIC_I2SCR_WL_24BIT	  (0 << AIC_I2SCR_WL_BIT) /* Word Length is 24 bit */
  #define AIC_I2SCR_WL_20BIT	  (1 << AIC_I2SCR_WL_BIT) /* Word Length is 20 bit */
  #define AIC_I2SCR_WL_18BIT	  (2 << AIC_I2SCR_WL_BIT) /* Word Length is 18 bit */
  #define AIC_I2SCR_WL_16BIT	  (3 << AIC_I2SCR_WL_BIT) /* Word Length is 16 bit */
  #define AIC_I2SCR_WL_8BIT	  (4 << AIC_I2SCR_WL_BIT) /* Word Length is 8 bit */
#define	AIC_I2SCR_AMSL		(1 << 0)

/* AIC Controller FIFO Status Register (AIC_SR) */

#define	AIC_SR_RFL_BIT		24
#define	AIC_SR_RFL_MASK		(0x1f << AIC_SR_RFL_BIT)
#define	AIC_SR_TFL_BIT		8
#define	AIC_SR_TFL_MASK		(0x1f << AIC_SR_TFL_BIT)
#define	AIC_SR_ROR		(1 << 6)
#define	AIC_SR_TUR		(1 << 5)
#define	AIC_SR_RFS		(1 << 4)
#define	AIC_SR_TFS		(1 << 3)

/* AIC Controller AC-link Status Register (AIC_ACSR) */

#define	AIC_ACSR_CRDY		(1 << 20)
#define	AIC_ACSR_CLPM		(1 << 19)
#define	AIC_ACSR_RSTO		(1 << 18)
#define	AIC_ACSR_SADR		(1 << 17)
#define	AIC_ACSR_CADT		(1 << 16)

/* AIC Controller I2S/MSB-justified Status Register (AIC_I2SSR) */

#define	AIC_I2SSR_BSY		(1 << 2)

/* AIC Controller AC97 codec Command Address Register (AIC_ACCAR) */

#define	AIC_ACCAR_CAR_BIT	0
#define	AIC_ACCAR_CAR_MASK	(0xfffff << AIC_ACCAR_CAR_BIT)

/* AIC Controller AC97 codec Command Data Register (AIC_ACCDR) */

#define	AIC_ACCDR_CDR_BIT	0
#define	AIC_ACCDR_CDR_MASK	(0xfffff << AIC_ACCDR_CDR_BIT)

/* AIC Controller AC97 codec Status Address Register (AIC_ACSAR) */

#define	AIC_ACSAR_SAR_BIT	0
#define	AIC_ACSAR_SAR_MASK	(0xfffff << AIC_ACSAR_SAR_BIT)

/* AIC Controller AC97 codec Status Data Register (AIC_ACSDR) */

#define	AIC_ACSDR_SDR_BIT	0
#define	AIC_ACSDR_SDR_MASK	(0xfffff << AIC_ACSDR_SDR_BIT)

/* AIC Controller I2S/MSB-justified Clock Divider Register (AIC_I2SDIV) */

#define	AIC_I2SDIV_DIV_BIT	0
#define	AIC_I2SDIV_DIV_MASK	(0x7f << AIC_I2SDIV_DIV_BIT)
  #define AIC_I2SDIV_BITCLK_3072KHZ	(0x0C << AIC_I2SDIV_DIV_BIT) /* BIT_CLK of 3.072MHz */
  #define AIC_I2SDIV_BITCLK_2836KHZ	(0x0D << AIC_I2SDIV_DIV_BIT) /* BIT_CLK of 2.836MHz */
  #define AIC_I2SDIV_BITCLK_1418KHZ	(0x1A << AIC_I2SDIV_DIV_BIT) /* BIT_CLK of 1.418MHz */
  #define AIC_I2SDIV_BITCLK_1024KHZ	(0x24 << AIC_I2SDIV_DIV_BIT) /* BIT_CLK of 1.024MHz */
  #define AIC_I2SDIV_BITCLK_7089KHZ	(0x34 << AIC_I2SDIV_DIV_BIT) /* BIT_CLK of 708.92KHz */
  #define AIC_I2SDIV_BITCLK_512KHZ	(0x48 << AIC_I2SDIV_DIV_BIT) /* BIT_CLK of 512.00KHz */




/*************************************************************************
 * LCD 
 *************************************************************************/
#define LCD_CFG		(LCD_BASE + 0x00)
#define LCD_VSYNC	(LCD_BASE + 0x04)
#define LCD_HSYNC	(LCD_BASE + 0x08)
#define LCD_VAT		(LCD_BASE + 0x0c)
#define LCD_DAH		(LCD_BASE + 0x10)
#define LCD_DAV		(LCD_BASE + 0x14)
#define LCD_PS		(LCD_BASE + 0x18)
#define LCD_CLS		(LCD_BASE + 0x1c)
#define LCD_SPL		(LCD_BASE + 0x20)
#define LCD_REV		(LCD_BASE + 0x24)
#define LCD_CTRL	(LCD_BASE + 0x30)
#define LCD_STATE	(LCD_BASE + 0x34)
#define LCD_IID		(LCD_BASE + 0x38)
#define LCD_DA0		(LCD_BASE + 0x40)
#define LCD_SA0		(LCD_BASE + 0x44)
#define LCD_FID0	(LCD_BASE + 0x48)
#define LCD_CMD0	(LCD_BASE + 0x4c)
#define LCD_DA1		(LCD_BASE + 0x50)
#define LCD_SA1		(LCD_BASE + 0x54)
#define LCD_FID1	(LCD_BASE + 0x58)
#define LCD_CMD1	(LCD_BASE + 0x5c)

#define REG_LCD_CFG	REG32(LCD_CFG)
#define REG_LCD_VSYNC	REG32(LCD_VSYNC)
#define REG_LCD_HSYNC	REG32(LCD_HSYNC)
#define REG_LCD_VAT	REG32(LCD_VAT)
#define REG_LCD_DAH	REG32(LCD_DAH)
#define REG_LCD_DAV	REG32(LCD_DAV)
#define REG_LCD_PS	REG32(LCD_PS)
#define REG_LCD_CLS	REG32(LCD_CLS)
#define REG_LCD_SPL	REG32(LCD_SPL)
#define REG_LCD_REV	REG32(LCD_REV)
#define REG_LCD_CTRL	REG32(LCD_CTRL)
#define REG_LCD_STATE	REG32(LCD_STATE)
#define REG_LCD_IID	REG32(LCD_IID)
#define REG_LCD_DA0	REG32(LCD_DA0)
#define REG_LCD_SA0	REG32(LCD_SA0)
#define REG_LCD_FID0	REG32(LCD_FID0)
#define REG_LCD_CMD0	REG32(LCD_CMD0)
#define REG_LCD_DA1	REG32(LCD_DA1)
#define REG_LCD_SA1	REG32(LCD_SA1)
#define REG_LCD_FID1	REG32(LCD_FID1)
#define REG_LCD_CMD1	REG32(LCD_CMD1)

#define LCD_CFG_PDW_BIT		4
#define LCD_CFG_PDW_MASK	(0x03 << LCD_DEV_PDW_BIT)
  #define LCD_CFG_PDW_1		(0 << LCD_DEV_PDW_BIT)
  #define LCD_CFG_PDW_2		(1 << LCD_DEV_PDW_BIT)
  #define LCD_CFG_PDW_4		(2 << LCD_DEV_PDW_BIT)
  #define LCD_CFG_PDW_8		(3 << LCD_DEV_PDW_BIT)
#define LCD_CFG_MODE_BIT	0
#define LCD_CFG_MODE_MASK	(0x0f << LCD_DEV_MODE_BIT)
  #define LCD_CFG_MODE_GENERIC_TFT	(0 << LCD_DEV_MODE_BIT)
  #define LCD_CFG_MODE_SHARP_HR		(1 << LCD_DEV_MODE_BIT)
  #define LCD_CFG_MODE_CASIO_TFT	(2 << LCD_DEV_MODE_BIT)
  #define LCD_CFG_MODE_SAMSUNG_ALPHA	(3 << LCD_DEV_MODE_BIT)
  #define LCD_CFG_MODE_NONINTER_CCIR656	(4 << LCD_DEV_MODE_BIT)
  #define LCD_CFG_MODE_INTER_CCIR656	(6 << LCD_DEV_MODE_BIT)
  #define LCD_CFG_MODE_SINGLE_CSTN	(8 << LCD_DEV_MODE_BIT)
  #define LCD_CFG_MODE_SINGLE_MSTN	(9 << LCD_DEV_MODE_BIT)
  #define LCD_CFG_MODE_DUAL_CSTN	(10 << LCD_DEV_MODE_BIT)
  #define LCD_CFG_MODE_DUAL_MSTN	(11 << LCD_DEV_MODE_BIT)

#define LCD_VSYNC_VPS_BIT	16
#define LCD_VSYNC_VPS_MASK	(0xffff << LCD_VSYNC_VPS_BIT)
#define LCD_VSYNC_VPE_BIT	0
#define LCD_VSYNC_VPE_MASK	(0xffff << LCD_VSYNC_VPS_BIT)

#define LCD_HSYNC_HPS_BIT	16
#define LCD_HSYNC_HPS_MASK	(0xffff << LCD_HSYNC_HPS_BIT)
#define LCD_HSYNC_HPE_BIT	0
#define LCD_HSYNC_HPE_MASK	(0xffff << LCD_HSYNC_HPE_BIT)

#define LCD_VAT_HT_BIT		16
#define LCD_VAT_HT_MASK		(0xffff << LCD_VAT_HT_BIT)
#define LCD_VAT_VT_BIT		0
#define LCD_VAT_VT_MASK		(0xffff << LCD_VAT_VT_BIT)

#define LCD_DAH_HDS_BIT		16
#define LCD_DAH_HDS_MASK	(0xffff << LCD_DAH_HDS_BIT)
#define LCD_DAH_HDE_BIT		0
#define LCD_DAH_HDE_MASK	(0xffff << LCD_DAH_HDE_BIT)

#define LCD_DAV_VDS_BIT		16
#define LCD_DAV_VDS_MASK	(0xffff << LCD_DAV_VDS_BIT)
#define LCD_DAV_VDE_BIT		0
#define LCD_DAV_VDE_MASK	(0xffff << LCD_DAV_VDE_BIT)

#define LCD_CTRL_BST_BIT	28
#define LCD_CTRL_BST_MASK	(0x03 << LCD_CTRL_BST_BIT)
  #define LCD_CTRL_BST_4	(0 << LCD_CTRL_BST_BIT)
  #define LCD_CTRL_BST_8	(1 << LCD_CTRL_BST_BIT)
  #define LCD_CTRL_BST_16	(2 << LCD_CTRL_BST_BIT)
#define LCD_CTRL_RGB555		(1 << 27)
#define LCD_CTRL_OFUP		(1 << 26)
#define LCD_CTRL_FRC_BIT	24
#define LCD_CTRL_FRC_MASK	(0x03 << LCD_CTRL_FRC_BIT)
  #define LCD_CTRL_FRC_16	(0 << LCD_CTRL_FRC_BIT)
  #define LCD_CTRL_FRC_4	(1 << LCD_CTRL_FRC_BIT)
  #define LCD_CTRL_FRC_2	(2 << LCD_CTRL_FRC_BIT)
#define LCD_CTRL_PDD_BIT	16
#define LCD_CTRL_PDD_MASK	(0xff << LCD_CTRL_PDD_BIT)
#define LCD_CTRL_EOFM		(1 << 13)
#define LCD_CTRL_SOFM		(1 << 12)
#define LCD_CTRL_OFUM		(1 << 11)
#define LCD_CTRL_IFUM0		(1 << 10)
#define LCD_CTRL_IFUM1		(1 << 9)
#define LCD_CTRL_LDDM		(1 << 8)
#define LCD_CTRL_QDM		(1 << 7)
#define LCD_CTRL_BEDN		(1 << 6)
#define LCD_CTRL_PEDN		(1 << 5)
#define LCD_CTRL_DIS		(1 << 4)
#define LCD_CTRL_ENA		(1 << 3)
#define LCD_CTRL_BPP_BIT	0
#define LCD_CTRL_BPP_MASK	(0x07 << LCD_CTRL_BPP_BIT)
  #define LCD_CTRL_BPP_1	(0 << LCD_CTRL_BPP_BIT)
  #define LCD_CTRL_BPP_2	(1 << LCD_CTRL_BPP_BIT)
  #define LCD_CTRL_BPP_4	(2 << LCD_CTRL_BPP_BIT)
  #define LCD_CTRL_BPP_8	(3 << LCD_CTRL_BPP_BIT)
  #define LCD_CTRL_BPP_16	(4 << LCD_CTRL_BPP_BIT)

#define LCD_STATE_QD		(1 << 7)
#define LCD_STATE_EOF		(1 << 5)
#define LCD_STATE_SOF		(1 << 4)
#define LCD_STATE_OFU		(1 << 3)
#define LCD_STATE_IFU0		(1 << 2)
#define LCD_STATE_IFU1		(1 << 1)
#define LCD_STATE_LDD		(1 << 0)

#define LCD_CMD_SOFINT		(1 << 31)
#define LCD_CMD_EOFINT		(1 << 30)
#define LCD_CMD_PAL		(1 << 28)
#define LCD_CMD_LEN_BIT		0
#define LCD_CMD_LEN_MASK	(0xffffff << LCD_CMD_LEN_BIT)




/*************************************************************************
 * DES
 *************************************************************************/
#define	DES_CR1			(DES_BASE + 0x000)
#define	DES_CR2			(DES_BASE + 0x004)
#define	DES_SR			(DES_BASE + 0x008)
#define	DES_K1L			(DES_BASE + 0x010)
#define	DES_K1R			(DES_BASE + 0x014)
#define	DES_K2L			(DES_BASE + 0x018)
#define	DES_K2R			(DES_BASE + 0x01C)
#define	DES_K3L			(DES_BASE + 0x020)
#define	DES_K3R			(DES_BASE + 0x024)
#define	DES_IVL			(DES_BASE + 0x028)
#define	DES_IVR			(DES_BASE + 0x02C)
#define	DES_DIN			(DES_BASE + 0x030)
#define	DES_DOUT		(DES_BASE + 0x034)

#define REG_DES_CR1		REG32(DES_CR1)
#define REG_DES_CR2		REG32(DES_CR2)
#define REG_DES_SR		REG32(DES_SR)
#define REG_DES_K1L		REG32(DES_K1L)
#define REG_DES_K1R		REG32(DES_K1R)
#define REG_DES_K2L		REG32(DES_K2L)
#define REG_DES_K2R		REG32(DES_K2R)
#define REG_DES_K3L		REG32(DES_K3L)
#define REG_DES_K3R		REG32(DES_K3R)
#define REG_DES_IVL		REG32(DES_IVL)
#define REG_DES_IVR		REG32(DES_IVR)
#define REG_DES_DIN		REG32(DES_DIN)
#define REG_DES_DOUT		REG32(DES_DOUT)

/* DES Control Register 1 (DES_CR1) */

#define	DES_CR1_EN 		(1 << 0)

/* DES Control Register 2 (DES_CR2) */

#define	DES_CR2_ENDEC 		(1 << 3)
#define	DES_CR2_MODE 		(1 << 2)
#define	DES_CR2_ALG 		(1 << 1)
#define	DES_CR2_DMAE		(1 << 0)

/* DES State Register (DES_SR) */

#define DES_SR_IN_FULL		(1 << 5)
#define DES_SR_IN_LHF		(1 << 4)
#define DES_SR_IN_EMPTY		(1 << 3)
#define DES_SR_OUT_FULL		(1 << 2)
#define DES_SR_OUT_GHF		(1 << 1)
#define DES_SR_OUT_EMPTY	(1 << 0)




/*************************************************************************
 * CPM
 *************************************************************************/
#define CPM_CFCR	(CPM_BASE+0x00)
#define CPM_PLCR1	(CPM_BASE+0x10)
#define CPM_OCR		(CPM_BASE+0x1c)
#define CPM_CFCR2	(CPM_BASE+0x60)
#define CPM_LPCR	(CPM_BASE+0x04)
#define CPM_RSTR	(CPM_BASE+0x08)
#define CPM_MSCR	(CPM_BASE+0x20)
#define CPM_SCR		(CPM_BASE+0x24)
#define CPM_WRER	(CPM_BASE+0x28)
#define CPM_WFER	(CPM_BASE+0x2c)
#define CPM_WER		(CPM_BASE+0x30)
#define CPM_WSR		(CPM_BASE+0x34)
#define CPM_GSR0	(CPM_BASE+0x38)
#define CPM_GSR1	(CPM_BASE+0x3c)
#define CPM_GSR2	(CPM_BASE+0x40)
#define CPM_SPR		(CPM_BASE+0x44)
#define CPM_GSR3	(CPM_BASE+0x48)

#define REG_CPM_CFCR	REG32(CPM_CFCR)
#define REG_CPM_PLCR1	REG32(CPM_PLCR1)
#define REG_CPM_OCR	REG32(CPM_OCR)
#define REG_CPM_CFCR2	REG32(CPM_CFCR2)
#define REG_CPM_LPCR	REG32(CPM_LPCR)
#define REG_CPM_RSTR	REG32(CPM_RSTR)
#define REG_CPM_MSCR	REG32(CPM_MSCR)
#define REG_CPM_SCR	REG32(CPM_SCR)
#define REG_CPM_WRER	REG32(CPM_WRER)
#define REG_CPM_WFER	REG32(CPM_WFER)
#define REG_CPM_WER	REG32(CPM_WER)
#define REG_CPM_WSR	REG32(CPM_WSR)
#define REG_CPM_GSR0	REG32(CPM_GSR0)
#define REG_CPM_GSR1	REG32(CPM_GSR1)
#define REG_CPM_GSR2	REG32(CPM_GSR2)
#define REG_CPM_SPR	REG32(CPM_SPR)
#define REG_CPM_GSR3	REG32(CPM_GSR3)

#define CPM_CFCR_SSI		(1 << 31)
#define CPM_CFCR_LCD		(1 << 30)
#define CPM_CFCR_I2S		(1 << 29)
#define CPM_CFCR_UCS		(1 << 28)
#define CPM_CFCR_UFR_BIT	25
#define CPM_CFCR_UFR_MASK	(0x07 << CPM_CFCR_UFR_BIT)
#define CPM_CFCR_MSC		(1 << 24)
#define CPM_CFCR_CKOEN2		(1 << 23)
#define CPM_CFCR_CKOEN1		(1 << 22)
#define CPM_CFCR_UPE		(1 << 20)
#define CPM_CFCR_MFR_BIT	16
#define CPM_CFCR_MFR_MASK	(0x0f << CPM_CFCR_MFR_BIT)
#define CPM_CFCR_LFR_BIT	12
#define CPM_CFCR_LFR_MASK	(0x0f << CPM_CFCR_LFR_BIT)
#define CPM_CFCR_PFR_BIT	8
#define CPM_CFCR_PFR_MASK	(0x0f << CPM_CFCR_PFR_BIT)
#define CPM_CFCR_SFR_BIT	4
#define CPM_CFCR_SFR_MASK	(0x0f << CPM_CFCR_SFR_BIT)
#define CPM_CFCR_IFR_BIT	0
#define CPM_CFCR_IFR_MASK	(0x0f << CPM_CFCR_IFR_BIT)

#define CPM_PLCR1_PLL1FD_BIT	23
#define CPM_PLCR1_PLL1FD_MASK	(0x1ff << CPM_PLCR1_PLL1FD_BIT)
#define CPM_PLCR1_PLL1RD_BIT	18
#define CPM_PLCR1_PLL1RD_MASK	(0x1f << CPM_PLCR1_PLL1RD_BIT)
#define CPM_PLCR1_PLL1OD_BIT	16
#define CPM_PLCR1_PLL1OD_MASK	(0x03 << CPM_PLCR1_PLL1OD_BIT)
#define CPM_PLCR1_PLL1S		(1 << 10)
#define CPM_PLCR1_PLL1BP	(1 << 9)
#define CPM_PLCR1_PLL1EN	(1 << 8)
#define CPM_PLCR1_PLL1ST_BIT	0
#define CPM_PLCR1_PLL1ST_MASK	(0xff << CPM_PLCR1_PLL1ST_BIT)

#define CPM_OCR_O1ST_BIT	16
#define CPM_OCR_O1ST_MASK	(0xff << CPM_OCR_O1ST_BIT)
#define CPM_OCR_EXT_RTC_CLK	(1<<8)
#define CPM_OCR_SUSPEND_PHY1	(1<<7)
#define CPM_OCR_SUSPEND_PHY0	(1<<6)

#define CPM_CFCR2_PXFR_BIT	0
#define CPM_CFCR2_PXFR_MASK	(0x1ff << CPM_CFCR2_PXFR_BIT)

#define CPM_LPCR_DUTY_BIT	3
#define CPM_LPCR_DUTY_MASK	(0x1f << CPM_LPCR_DUTY_BIT)
#define CPM_LPCR_DOZE		(1 << 2)
#define CPM_LPCR_LPM_BIT	0
#define CPM_LPCR_LPM_MASK	(0x03 << CPM_LPCR_LPM_BIT)
  #define CPM_LPCR_LPM_IDLE		(0 << CPM_LPCR_LPM_BIT)
  #define CPM_LPCR_LPM_SLEEP		(1 << CPM_LPCR_LPM_BIT)
  #define CPM_LPCR_LPM_HIBERNATE	(2 << CPM_LPCR_LPM_BIT)

#define CPM_RSTR_SR		(1 << 2)
#define CPM_RSTR_WR		(1 << 1)
#define CPM_RSTR_HR		(1 << 0)

#define CPM_MSCR_MSTP_BIT	0
#define CPM_MSCR_MSTP_MASK	(0x1ffffff << CPM_MSCR_MSTP_BIT)
  #define CPM_MSCR_MSTP_UART0	0
  #define CPM_MSCR_MSTP_UART1	1
  #define CPM_MSCR_MSTP_UART2	2
  #define CPM_MSCR_MSTP_OST	3
  #define CPM_MSCR_MSTP_DMAC	5
  #define CPM_MSCR_MSTP_UHC	6
  #define CPM_MSCR_MSTP_LCD	7
  #define CPM_MSCR_MSTP_I2C	8
  #define CPM_MSCR_MSTP_AICPCLK 9
  #define CPM_MSCR_MSTP_PWM0	10
  #define CPM_MSCR_MSTP_PWM1	11
  #define CPM_MSCR_MSTP_SSI	12
  #define CPM_MSCR_MSTP_MSC	13
  #define CPM_MSCR_MSTP_SCC	14
  #define CPM_MSCR_MSTP_AICBCLK	18
  #define CPM_MSCR_MSTP_UART3	20
  #define CPM_MSCR_MSTP_ETH	21
  #define CPM_MSCR_MSTP_KBC	22
  #define CPM_MSCR_MSTP_CIM	23
  #define CPM_MSCR_MSTP_UDC	24
  #define CPM_MSCR_MSTP_UPRT	25

#define CPM_SCR_O1SE		(1 << 4)
#define CPM_SCR_HGP		(1 << 3)
#define CPM_SCR_HZP		(1 << 2)
#define CPM_SCR_HZM		(1 << 1)

#define CPM_WRER_RE_BIT		0
#define CPM_WRER_RE_MASK	(0xffff << CPM_WRER_RE_BIT)

#define CPM_WFER_FE_BIT		0
#define CPM_WFER_FE_MASK	(0xffff << CPM_WFER_FE_BIT)

#define CPM_WER_WERTC		(1 << 31)
#define CPM_WER_WEETH		(1 << 30)
#define CPM_WER_WE_BIT		0
#define CPM_WER_WE_MASK		(0xffff << CPM_WER_WE_BIT)

#define CPM_WSR_WSRTC		(1 << 31)
#define CPM_WSR_WSETH		(1 << 30)
#define CPM_WSR_WS_BIT		0
#define CPM_WSR_WS_MASK		(0xffff << CPM_WSR_WS_BIT)




/*************************************************************************
 * SSI
 *************************************************************************/
#define	SSI_DR			(SSI_BASE + 0x000)
#define	SSI_CR0			(SSI_BASE + 0x004)
#define	SSI_CR1			(SSI_BASE + 0x008)
#define	SSI_SR			(SSI_BASE + 0x00C)
#define	SSI_ITR			(SSI_BASE + 0x010)
#define	SSI_ICR			(SSI_BASE + 0x014)
#define	SSI_GR			(SSI_BASE + 0x018)

#define	REG_SSI_DR		REG32(SSI_DR)
#define	REG_SSI_CR0		REG16(SSI_CR0)
#define	REG_SSI_CR1		REG32(SSI_CR1)
#define	REG_SSI_SR		REG32(SSI_SR)
#define	REG_SSI_ITR		REG16(SSI_ITR)
#define	REG_SSI_ICR		REG8(SSI_ICR)
#define	REG_SSI_GR		REG16(SSI_GR)

/* SSI Data Register (SSI_DR) */

#define	SSI_DR_GPC_BIT		0
#define	SSI_DR_GPC_MASK		(0x1ff << SSI_DR_GPC_BIT)

/* SSI Control Register 0 (SSI_CR0) */

#define SSI_CR0_SSIE		(1 << 15)
#define SSI_CR0_TIE		(1 << 14)
#define SSI_CR0_RIE		(1 << 13)
#define SSI_CR0_TEIE		(1 << 12)
#define SSI_CR0_REIE		(1 << 11)
#define SSI_CR0_LOOP		(1 << 10)
#define SSI_CR0_RFINE		(1 << 9)
#define SSI_CR0_RFINC		(1 << 8)
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
#define SSI_CR1_TTRG_BIT	10
#define SSI_CR1_TTRG_MASK	(0x3 << SSI_CR1_TTRG_BIT)
  #define SSI_CR1_TTRG_1	  (0 << SSI_CR1_TTRG_BIT)/* Less than or equal to 1 */
  #define SSI_CR1_TTRG_4	  (1 << SSI_CR1_TTRG_BIT) /* Less than or equal to 4 */
  #define SSI_CR1_TTRG_8	  (2 << SSI_CR1_TTRG_BIT) /* Less than or equal to 8 */
  #define SSI_CR1_TTRG_14	  (3 << SSI_CR1_TTRG_BIT) /* Less than or equal to 14 */
#define SSI_CR1_RTRG_BIT	8
#define SSI_CR1_RTRG_MASK	(0x3 << SSI_CR1_RTRG_BIT)
  #define SSI_CR1_RTRG_1	  (0 << SSI_CR1_RTRG_BIT) /* More than or equal to 1 */
  #define SSI_CR1_RTRG_4	  (1 << SSI_CR1_RTRG_BIT) /* More than or equal to 4 */
  #define SSI_CR1_RTRG_8	  (2 << SSI_CR1_RTRG_BIT) /* More than or equal to 8 */
  #define SSI_CR1_RTRG_14	  (3 << SSI_CR1_RTRG_BIT) /* More than or equal to 14 */
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

#define SSI_SR_TFIFONUM_BIT	13
#define SSI_SR_TFIFONUM_MASK	(0x1f << SSI_SR_TFIFONUM_BIT)
#define SSI_SR_RFIFONUM_BIT	8
#define SSI_SR_RFIFONUM_MASK	(0x1f << SSI_SR_RFIFONUM_BIT)
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

#endif /* __ASM_JZ4730_REGS_H__ */
