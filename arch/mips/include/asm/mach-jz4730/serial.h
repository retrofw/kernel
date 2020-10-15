/*
 *  linux/include/asm-mips/mach-jz4730/serial.h
 *
 *  JZ4730 serial port definition.
 *
 *  Copyright (C) 2006 - 2007 Ingenic Semiconductor Inc.
 *
 *  Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4730_SERIAL_H__
#define __ASM_JZ4730_SERIAL_H__

#define JZ_BASE_BAUD	(JZ_EXTAL/16)
#define JZ_SERIAL_PORT_DEFNS \
	{ .baud_base = JZ_BASE_BAUD, .irq = IRQ_UART0, \
	  .flags = STD_COM_FLAGS, .iomem_base = (u8 *)UART0_BASE, \
	  .iomem_reg_shift = 2, .io_type = SERIAL_IO_MEM }, \
	{ .baud_base = JZ_BASE_BAUD, .irq = IRQ_UART1, \
	  .flags = STD_COM_FLAGS, .iomem_base = (u8 *)UART1_BASE, \
	  .iomem_reg_shift = 2, .io_type = SERIAL_IO_MEM }, \
	{ .baud_base = JZ_BASE_BAUD, .irq = IRQ_UART2, \
	  .flags = STD_COM_FLAGS, .iomem_base = (u8 *)UART2_BASE, \
	  .iomem_reg_shift = 2, .io_type = SERIAL_IO_MEM }, \
	{ .baud_base = JZ_BASE_BAUD, .irq = IRQ_UART3, \
	  .flags = STD_COM_FLAGS, .iomem_base = (u8 *)UART3_BASE, \
	  .iomem_reg_shift = 2, .io_type = SERIAL_IO_MEM },

#endif /* __ASM_JZ4730_SERIAL_H__ */
