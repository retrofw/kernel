/*
 * linux/include/asm-mips/mach-jz4750l/jz4750lintc.h
 *
 * JZ4750L INTC register definition
 *
 * Copyright (C) 2008-2014 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ4750LINTC_H__
#define __JZ4750LINTC_H__

#define	INTC_BASE	0xB0001000

/*************************************************************************
 * INTC (Interrupt Controller)
 *************************************************************************/
#define INTC_ISR	(INTC_BASE + 0x00)
#define INTC_IMR	(INTC_BASE + 0x04)
#define INTC_IMSR	(INTC_BASE + 0x08)
#define INTC_IMCR	(INTC_BASE + 0x0c)
#define INTC_IPR	(INTC_BASE + 0x10)
#define INTC_ISSR	(INTC_BASE + 0x18)  /* Interrupt Controller Source Set Register */
#define INTC_ISCR	(INTC_BASE + 0x1c)  /* Interrupt Controller Source Clear Register */

#define REG_INTC_ISR	REG32(INTC_ISR)
#define REG_INTC_IMR	REG32(INTC_IMR)
#define REG_INTC_IMSR	REG32(INTC_IMSR)
#define REG_INTC_IMCR	REG32(INTC_IMCR)
#define REG_INTC_IPR	REG32(INTC_IPR)
#define REG_INTC_ISSR   REG32(INTC_ISSR)
#define REG_INTC_ISCR   REG32(INTC_ISCR)

/* 1st-level interrupts */
#define IRQ_ETH		0
#define IRQ_SFT		4
#define IRQ_I2C		5
#define IRQ_RTC		6
#define IRQ_UART2	7
#define IRQ_UART1	8
#define IRQ_UART0	9
#define IRQ_AIC 	10
#define IRQ_GPIO5	11
#define IRQ_GPIO4	12
#define IRQ_GPIO3	13
#define IRQ_GPIO2	14
#define IRQ_GPIO1	15
#define IRQ_GPIO0	16
#define IRQ_BCH		17
#define IRQ_SADC	18
#define IRQ_CIM		19
#define IRQ_TSSI	20
#define IRQ_TCU2	21
#define IRQ_TCU1	22
#define IRQ_TCU0	23
#define IRQ_MSC1	24
#define IRQ_MSC0	25
#define IRQ_SSI   	26
#define IRQ_UDC		27
#define IRQ_DMAC1	28
#define IRQ_DMAC0	29
#define IRQ_IPU		30
#define IRQ_LCD		31

/* 2nd-level interrupts */
#define IRQ_DMA_0	32  /* 32 to 37 for DMAC's 0-5 */
#define IRQ_GPIO_0	48  /* 48 to 175 for GPIO pin 0 to 127 */
#define IRQ_SADC_0  240	/* 240 to 245 */

#define NUM_DMA         MAX_DMA_NUM     /* Jz4750L num DMA 6 */
#define NUM_GPIO        MAX_GPIO_NUM    /* GPIO NUM: 128, Jz4750L real num GPIO 128 */
#define NUM_SADC        MAX_SADC_NUM    /* Jz4750L num SADC 6 */

/*
 * INTC Operations
 */
#ifndef __MIPS_ASSEMBLER
#define __intc_unmask_irq(n)	( REG_INTC_IMCR = (1 << (n)) )
#define __intc_mask_irq(n)	( REG_INTC_IMSR = (1 << (n)) )
#define __intc_ack_irq(n)	( REG_INTC_IPR = (1 << (n)) ) /* A dummy ack, as the Pending Register is Read Only. Should we remove __intc_ack_irq() */
#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4750LINTC_H__ */
