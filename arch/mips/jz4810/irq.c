/*
 * linux/arch/mips/jz4810/irq.c
 *
 * JZ4810 interrupt routines.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <lhhuang@ingenic.cn>
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/timex.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/bitops.h>

#include <asm/bootinfo.h>
#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/system.h>
#include <asm/jzsoc.h>

/*
 * INTC irq type
 */

static void enable_intc_irq(unsigned int irq)
{
	__intc_unmask_irq(irq);
}

static void disable_intc_irq(unsigned int irq)
{
	__intc_mask_irq(irq);
}

static void mask_and_ack_intc_irq(unsigned int irq)
{
	__intc_mask_irq(irq);
	__intc_ack_irq(irq);
}

static void end_intc_irq(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS))) {
		enable_intc_irq(irq);
	}
}

static unsigned int startup_intc_irq(unsigned int irq)
{
	enable_intc_irq(irq);
	return 0;
}

static void shutdown_intc_irq(unsigned int irq)
{
	disable_intc_irq(irq);
}

static struct irq_chip intc_irq_type = {
	.typename = "INTC",
	.startup = startup_intc_irq,
	.shutdown = shutdown_intc_irq,
	.unmask = enable_intc_irq,
	.mask = disable_intc_irq,
	.ack = mask_and_ack_intc_irq,
	.end = end_intc_irq,
};

/*
 * GPIO irq type
 */

static void enable_gpio_irq(unsigned int irq)
{
	unsigned int intc_irq;

	if (irq < (IRQ_GPIO_0 + 32)) {
		intc_irq = IRQ_GPIO0;
	}
	else if (irq < (IRQ_GPIO_0 + 64)) {
		intc_irq = IRQ_GPIO1;
	}
	else if (irq < (IRQ_GPIO_0 + 96)) {
		intc_irq = IRQ_GPIO2;
	}
	else if (irq < (IRQ_GPIO_0 + 128)) {
		intc_irq = IRQ_GPIO3;
	}
	else if (irq < (IRQ_GPIO_0 + 160)) {
		intc_irq = IRQ_GPIO4;
	}
	else {
		intc_irq = IRQ_GPIO5;
	}

	enable_intc_irq(intc_irq);
	__gpio_unmask_irq(irq - IRQ_GPIO_0);
}

static void disable_gpio_irq(unsigned int irq)
{
	__gpio_mask_irq(irq - IRQ_GPIO_0);
}

static void mask_and_ack_gpio_irq(unsigned int irq)
{
	__gpio_mask_irq(irq - IRQ_GPIO_0);
	__gpio_ack_irq(irq - IRQ_GPIO_0);
}

static void end_gpio_irq(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS))) {
		enable_gpio_irq(irq);
	}
}

static unsigned int startup_gpio_irq(unsigned int irq)
{
	enable_gpio_irq(irq);
	return 0;
}

static void shutdown_gpio_irq(unsigned int irq)
{
	disable_gpio_irq(irq);
}

static struct irq_chip gpio_irq_type = {
	.typename = "GPIO",
	.startup = startup_gpio_irq,
	.shutdown = shutdown_gpio_irq,
	.unmask = enable_gpio_irq,
	.mask = disable_gpio_irq,
	.ack = mask_and_ack_gpio_irq,
	.end = end_gpio_irq,
};

/*
 * DMA irq type
 */
static void enable_dma_irq(unsigned int irq)
{
	unsigned int intc_irq;

	if ( irq < (IRQ_DMA_0 + HALF_DMA_NUM) ) 	/* DMAC Group 0 irq */
		intc_irq = IRQ_DMAC0;
	else if ( irq < (IRQ_DMA_0 + MAX_DMA_NUM) ) 	/* DMAC Group 1 irq */
		intc_irq = IRQ_DMAC1;
	else {
		printk("%s, unexpected dma irq #%d\n", __FILE__, irq);
		return;
	}
	__intc_unmask_irq(intc_irq);
	__dmac_channel_enable_irq(irq - IRQ_DMA_0);
}

static void disable_dma_irq(unsigned int irq)
{
	int chan = irq - IRQ_DMA_0;
	__dmac_disable_channel(chan);
	__dmac_channel_disable_irq(chan);
}

static void mask_and_ack_dma_irq(unsigned int irq)
{
	unsigned int intc_irq;

	disable_dma_irq(irq);

	if ( irq < (IRQ_DMA_0 + HALF_DMA_NUM) ) 	/* DMAC Group 0 irq */
		intc_irq = IRQ_DMAC0;
	else if ( irq < (IRQ_DMA_0 + MAX_DMA_NUM) ) 	/* DMAC Group 1 irq */
		intc_irq = IRQ_DMAC1;
	else {
		printk("%s, unexpected dma irq #%d\n", __FILE__, irq);
		return ;
	}
	__intc_ack_irq(intc_irq);
	//__dmac_channel_ack_irq(irq-IRQ_DMA_0); /* needed?? add 20080506, Wolfgang */
	//__dmac_channel_disable_irq(irq - IRQ_DMA_0);
}

static void end_dma_irq(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS))) {
		enable_dma_irq(irq);
	}
}

static unsigned int startup_dma_irq(unsigned int irq)
{
	enable_dma_irq(irq);
	return 0;
}

static void shutdown_dma_irq(unsigned int irq)
{
	disable_dma_irq(irq);
}

static struct irq_chip dma_irq_type = {
	.typename = "DMA",
	.startup = startup_dma_irq,
	.shutdown = shutdown_dma_irq,
	.unmask = enable_dma_irq,
	.mask = disable_dma_irq,
	.ack = mask_and_ack_dma_irq,
	.end = end_dma_irq,
};

#if 0
/*
 * MDMA irq type
 */

static void enable_mdma_irq(unsigned int irq)
{
	unsigned int intc_irq;

	if (irq < IRQ_MDMA_0 + MAX_MDMA_NUM) 	/* DMAC Group 0 irq */
		intc_irq = IRQ_MDMA;
	else {
		printk("%s, unexpected mdma irq #%d\n", __FILE__, irq);
		return;
	}
	__intc_unmask_irq(intc_irq);
	__mdmac_channel_enable_irq(irq - IRQ_DMA_0);
}

static void disable_mdma_irq(unsigned int irq)
{
	__mdmac_channel_disable_irq(irq - IRQ_DMA_0);
}

static void mask_and_ack_mdma_irq(unsigned int irq)
{
	unsigned int intc_irq;

	if ( irq < IRQ_MDMA_0 + MAX_MDMA_NUM ) 	/* DMAC Group 0 irq */
		intc_irq = IRQ_MDMA;
	else {
		printk("%s, unexpected mdma irq #%d\n", __FILE__, irq);
		return ;
	}
	__intc_ack_irq(intc_irq);
	__mdmac_channel_ack_irq(irq-IRQ_MDMA_0); /* needed?? add 20080506, Wolfgang */
	__mdmac_channel_disable_irq(irq - IRQ_MDMA_0);
}

static void end_mdma_irq(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS))) {
		enable_mdma_irq(irq);
	}
}

static unsigned int startup_mdma_irq(unsigned int irq)
{
	enable_mdma_irq(irq);
	return 0;
}

static void shutdown_mdma_irq(unsigned int irq)
{
	disable_mdma_irq(irq);
}

static struct irq_chip mdma_irq_type = {
	.typename = "MDMA",
	.startup = startup_mdma_irq,
	.shutdown = shutdown_mdma_irq,
	.unmask = enable_mdma_irq,
	.mask = disable_mdma_irq,
	.ack = mask_and_ack_mdma_irq,
	.end = end_mdma_irq,
};
#endif

//----------------------------------------------------------------------

/*
 * BDMA irq type
 */

static void enable_bdma_irq(unsigned int irq)
{
	unsigned int intc_irq;

	if (irq < IRQ_BDMA_0 + MAX_BDMA_NUM)
		intc_irq = IRQ_BDMA;
	else {
		printk("%s, unexpected bdma irq #%d\n", __FILE__, irq);
		return;
	}
	__intc_unmask_irq(intc_irq);
	__bdmac_channel_enable_irq(irq - IRQ_DMA_0);
}

static void disable_bdma_irq(unsigned int irq)
{
	__bdmac_channel_disable_irq(irq - IRQ_DMA_0);
}

static void mask_and_ack_bdma_irq(unsigned int irq)
{
	unsigned int intc_irq;

	if ( irq < IRQ_BDMA_0 + MAX_BDMA_NUM ) 	/* DMAC Group 0 irq */
		intc_irq = IRQ_BDMA;
	else {
		printk("%s, unexpected bdma irq #%d\n", __FILE__, irq);
		return ;
	}
	__intc_ack_irq(intc_irq);
	__bdmac_channel_ack_irq(irq - IRQ_BDMA_0);
	__bdmac_channel_disable_irq(irq - IRQ_BDMA_0);
}

static void end_bdma_irq(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS))) {
		enable_bdma_irq(irq);
	}
}

static unsigned int startup_bdma_irq(unsigned int irq)
{
	enable_bdma_irq(irq);
	return 0;
}

static void shutdown_bdma_irq(unsigned int irq)
{
	disable_bdma_irq(irq);
}

static struct irq_chip bdma_irq_type = {
	.typename = "BDMA",
	.startup = startup_bdma_irq,
	.shutdown = shutdown_bdma_irq,
	.unmask = enable_bdma_irq,
	.mask = disable_bdma_irq,
	.ack = mask_and_ack_bdma_irq,
	.end = end_bdma_irq,
};

//----------------------------------------------------------------------

void __init arch_init_irq(void)
{
	int i;

	clear_c0_status(0xff04); /* clear ERL */
	set_c0_status(0x0400);   /* set IP2 */

	/* Set up INTC irq
	 */
	for (i = 0; i < NUM_INTC; i++) {
		disable_intc_irq(i);
		set_irq_chip_and_handler(i, &intc_irq_type, handle_level_irq);
	}

	/* Set up DMAC irq
	 */
	for (i = 0; i < NUM_DMA; i++) {
		disable_dma_irq(IRQ_DMA_0 + i);
		set_irq_chip_and_handler(IRQ_DMA_0 + i, &dma_irq_type, handle_level_irq);
	}

#if 0
	/* Set up MDMAC irq
	 */
	for (i = 0; i < NUM_MDMA; i++) {
		disable_mdma_irq(IRQ_MDMA_0 + i);
		set_irq_chip_and_handler(IRQ_MDMA_0 + i, &mdma_irq_type, handle_level_irq);
	}
#endif

	/* Set up BDMA irq
	 */
	for (i = 0; i < MAX_BDMA_NUM; i++) {
		disable_bdma_irq(IRQ_BDMA_0 + i);
		set_irq_chip_and_handler(IRQ_BDMA_0 + i, &bdma_irq_type, handle_level_irq);
	}

	/* Set up GPIO irq
	 */
#ifndef JZ_BOOTUP_UART_TXD
#error "JZ_BOOTUP_UART_TXD is not set, please define it int your board header file!"
#endif
#ifndef JZ_BOOTUP_UART_RXD
#error "JZ_BOOTUP_UART_RXD is not set, please define it int your board header file!"
#endif
	for (i = 0; i < NUM_GPIO; i++) {
		if (unlikely(i == JZ_BOOTUP_UART_TXD))
			continue;
		if (unlikely(i == JZ_BOOTUP_UART_RXD))
			continue;
		disable_gpio_irq(IRQ_GPIO_0 + i);
		set_irq_chip_and_handler(IRQ_GPIO_0 + i, &gpio_irq_type, handle_level_irq);
	}
}

static int plat_real_irq(int irq)
{
	int group = 0;

	if ((irq >= IRQ_GPIO5) && (irq <= IRQ_GPIO0)) {
		group = IRQ_GPIO0 - irq;
		irq = __gpio_group_irq(group);
		if (irq >= 0)
			irq += IRQ_GPIO_0 + 32 * group;
	} else {
		switch (irq) {
		case IRQ_DMAC0:
		case IRQ_DMAC1:
			irq = __dmac_get_irq();
			if (irq < 0) {
				printk("REG_DMAC_DMAIPR(0) = 0x%08x\n", REG_DMAC_DMAIPR(0));
				printk("REG_DMAC_DMAIPR(1) = 0x%08x\n", REG_DMAC_DMAIPR(1));
				return irq;
			}
			irq += IRQ_DMA_0;
			break;
#if 0
		case IRQ_MDMA:
			irq = __mdmac_get_irq();
			if (irq < 0)
				return irq;
			irq += IRQ_MDMA_0;
			break;
#endif
		case IRQ_BDMA:
			irq = __bdmac_get_irq();
			if (irq < 0)
				return irq;

			irq += IRQ_BDMA_0;
			break;
		}
	}

	return irq;
}

asmlinkage void plat_irq_dispatch(void)
{
	int irq = 0;

	unsigned long intc_ipr0 = 0, intc_ipr1 = 0;

	intc_ipr0 = REG_INTC_IPR(0);
	intc_ipr1 = REG_INTC_IPR(1);

	if (!(intc_ipr0 || intc_ipr1))	return;

	if (intc_ipr0) {
		irq = ffs(intc_ipr0) - 1;
		intc_ipr0 &= ~(1<<irq);
	} else {
		irq = ffs(intc_ipr1) - 1;
		intc_ipr1 &= ~(1<<irq);
		irq += 32;
	}

	irq = plat_real_irq(irq);
	WARN((irq < 0), "irq raised, but no irq pending!\n");
	if (irq < 0)
		return;

	do_IRQ(irq);
}
