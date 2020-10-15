/*
 * linux/include/asm-mips/mach-jz4750l/jz4750lotp.h
 *
 * JZ4750L OTP register definition
 *
 * Copyright (C) 2008-2014 Ingenic Semiconductor Co., Ltd.
 */
#ifndef __JZ4750LOTP_H__
#define __JZ4750LOTP_H__

/* Define the module base addresses */
#define OTP_BASE	0xB3012000

/*************************************************************************
 * OTP (One Time Programmable Module)
 *************************************************************************/
#define OTP_ID0		(OTP_BASE + 0x00) /* ID0 Register */
#define OTP_ID1		(OTP_BASE + 0x04) /* ID1 Register */
#define OTP_ID2		(OTP_BASE + 0x08) /* ID2 Register */
#define OTP_ID3		(OTP_BASE + 0x0C) /* ID3 Register */
#define OTP_BR0		(OTP_BASE + 0x10) /* BOOTROM0 Register */
#define OTP_BR1		(OTP_BASE + 0x14) /* BOOTROM1 Register */
#define OTP_HW0		(OTP_BASE + 0x18) /* Chip Hardware 0 Register */
#define OTP_HW1		(OTP_BASE + 0x1C) /* Chip Hardware 1 Register */

#define REG_OTP_ID0	REG32(OTP_ID0)
#define REG_OTP_ID1	REG32(OTP_ID1)
#define REG_OTP_ID2	REG32(OTP_ID2)
#define REG_OTP_ID3	REG32(OTP_ID3)
#define REG_OTP_BR0	REG32(OTP_BR0)
#define REG_OTP_BR1	REG32(OTP_BR1)
#define REG_OTP_HW0	REG32(OTP_HW0)
#define REG_OTP_HW1	REG32(OTP_HW1)

/* ID0 Register */
#define OTP_ID0_WID_BIT		24 /* Wafer ID */
#define OTP_ID0_WID_MASK	(0xff << OTP_ID0_WID_BIT)
#define OTP_ID0_MID_BIT		16 /* MASK ID */
#define OTP_ID0_MID_MASK	(0xff << OTP_ID0_MID_BIT)
#define OTP_ID0_FID_BIT		8 /* Foundary ID */
#define OTP_ID0_FID_MASK	(0xff << OTP_ID0_FID_BIT)
#define OTP_ID0_PID_BIT		0 /* Product ID */
#define OTP_ID0_PID_MASK	(0xff << OTP_ID0_PID_BIT)

/* ID1 Register */
#define OTP_ID1_LID_BIT		8 /* Lot ID */
#define OTP_ID1_LID_MASK	(0xffffff << OTP_ID1_LID_BIT)
#define OTP_ID1_TID_BIT		0 /* Test House ID */
#define OTP_ID1_TID_MASK	(0xff << OTP_ID1_TID_BIT)

/* ID2 Register */
#define OTP_ID2_XADR_BIT	24 /* Die X-dir Address */
#define OTP_ID2_XADR_MASK	(0xff << OTP_ID2_XADR_BIT)
#define OTP_ID2_YADR_BIT	16 /* Die Y-dir Address */
#define OTP_ID2_YADR_MASK	(0xff << OTP_ID2_YADR_BIT)
#define OTP_ID2_TDATE_BIT	0  /* Testing Date */
#define OTP_ID2_TDATE_MASK	(0xffff << OTP_ID2_TDATE_BIT)

/* ID3 Register */
#define OTP_ID3_CID_BIT		16 /* Customer ID */
#define OTP_ID3_CID_MASK	(0xffff << OTP_ID3_CID_BIT)
#define OTP_ID3_CP_BIT		0 /* Chip Parameters */
#define OTP_ID3_CP_MASK		(0xffff << OTP_ID3_CP_BIT)

/* BOOTROM1 Register */
#define OTP_BR1_UDCBOOT_BIT	0
#define OTP_BR1_UDCBOOT_MASK	(0xff << OTP_BR1_UDCBOOT_BIT)
  #define OTP_BR1_UDCBOOT_AUTO	(0xf0 << OTP_BR1_UDCBOOT_BIT)
  #define OTP_BR1_UDCBOOT_24M	(0x0f << OTP_BR1_UDCBOOT_BIT) /* 24MHz OSC */
  #define OTP_BR1_UDCBOOT_13M	(0x0c << OTP_BR1_UDCBOOT_BIT) /* 13MHz OSC */
  #define OTP_BR1_UDCBOOT_26M	(0x03 << OTP_BR1_UDCBOOT_BIT) /* 26MHz OSC */
  #define OTP_BR1_UDCBOOT_27M	(0x00 << OTP_BR1_UDCBOOT_BIT) /* 27MHz OSC */

/* Chip Hardware 1 Register */
#define OTP_HW1_MC_EN		(0x3 << 30) /* MC is enabled */
#define OTP_HW1_ME_EN		(0x3 << 28)
#define OTP_HW1_DE_EN		(0x3 << 26)
#define OTP_HW1_IDCT_EN		(0x3 << 24)
#define OTP_HW1_UART3_EN	(0x3 << 22)
#define OTP_HW1_UART2_EN	(0x3 << 20)
#define OTP_HW1_UART1_EN	(0x3 << 18)
#define OTP_HW1_UART0_EN	(0x3 << 16)
#define OTP_HW1_SSI1_EN		(0x3 << 14)
#define OTP_HW1_SSI0_EN		(0x3 << 12)
#define OTP_HW1_MSC1_EN		(0x3 << 10)
#define OTP_HW1_MSC0_EN		(0x3 << 8)
#define OTP_HW1_UHC_EN		(0x3 << 6)
#define OTP_HW1_TVE_EN		(0x3 << 4)
#define OTP_HW1_TSSI_EN		(0x3 << 2)
#define OTP_HW1_CIM_EN		(0x3 << 0)

#ifndef __MIPS_ASSEMBLER

#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4750LOTP_H__ */
