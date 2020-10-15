/*
 * linux/arch/mips/jz4740/irq.c
 *
 * JZ4740 interrupt routines.
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
	else {
		intc_irq = IRQ_GPIO3;
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
	__intc_unmask_irq(IRQ_DMAC);
	__dmac_channel_enable_irq(irq - IRQ_DMA_0);
}

static void disable_dma_irq(unsigned int irq)
{
	__dmac_channel_disable_irq(irq - IRQ_DMA_0);
}

static void mask_and_ack_dma_irq(unsigned int irq)
{
	__intc_ack_irq(IRQ_DMAC);
	__dmac_channel_disable_irq(irq - IRQ_DMA_0);
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

//----------------------------------------------------------------------

void __init arch_init_irq(void)
{
	int i;
	
	/* CPU level interrupts is still not handled. */
	clear_c0_status(0xff04); /* clear ERL */
	set_c0_status(0x0400);   /* set IP2 */

	/* Set up INTC irq
	*/
	for (i = 0; i < 32; i++) {
		disable_intc_irq(i);
		set_irq_chip_and_handler(i, &intc_irq_type, handle_level_irq);
	}

	/* Set up DMAC irq
	 */
	for (i = 0; i < NUM_DMA; i++) {
		disable_dma_irq(IRQ_DMA_0 + i);
		set_irq_chip_and_handler(IRQ_DMA_0 + i, &dma_irq_type, handle_level_irq);
	}

	/* Set up GPIO irq
	 */
	for (i = 0; i < NUM_GPIO; i++) {
		disable_gpio_irq(IRQ_GPIO_0 + i);
		set_irq_chip_and_handler(IRQ_GPIO_0 + i, &gpio_irq_type, handle_level_irq);
	}
}

static int plat_real_irq(int irq)
{
	switch (irq) {
	case IRQ_GPIO0:
		irq = __gpio_group_irq(0) + IRQ_GPIO_0;
		break;
	case IRQ_GPIO1:
		irq = __gpio_group_irq(1) + IRQ_GPIO_0 + 32;
		break;
	case IRQ_GPIO2:
		irq = __gpio_group_irq(2) + IRQ_GPIO_0 + 64;
		break;
	case IRQ_GPIO3:
		irq = __gpio_group_irq(3) + IRQ_GPIO_0 + 96;
		break;
	case IRQ_DMAC:
		irq = __dmac_get_irq() + IRQ_DMA_0;
		break;
	}

	return irq;
}

asmlinkage void plat_irq_dispatch(void)
{
	int irq = 0;
	static unsigned long intc_ipr = 0;

	intc_ipr |= REG_INTC_IPR;

	if (!intc_ipr)	return;

	irq = ffs(intc_ipr) - 1;
	intc_ipr &= ~(1<<irq);
	
	irq = plat_real_irq(irq);
	do_IRQ(irq);
}
