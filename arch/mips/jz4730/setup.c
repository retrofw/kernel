/*
 * linux/arch/mips/jz4730/setup.c
 *
 * JZ4730 CPU common setup routines.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/irq.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>
#include <asm/pgtable.h>
#include <asm/time.h>
#include <asm/jzsoc.h>

#ifdef CONFIG_PC_KEYB
#include <asm/keyboard.h>
#endif

jz_clocks_t jz_clocks;

extern char * __init prom_getcmdline(void);
extern void __init jz_board_setup(void);
extern void jz_restart(char *);
extern void jz_halt(void);
extern void jz_power_off(void);
extern void jz_time_init(void);

static void __init sysclocks_setup(void)
{
#ifndef CONFIG_JZ4730_URANUS
	jz_clocks.iclk = __cpm_get_iclk();
	jz_clocks.sclk = __cpm_get_sclk();
	jz_clocks.mclk = __cpm_get_mclk();
	jz_clocks.pclk = __cpm_get_pclk();
	jz_clocks.devclk = __cpm_get_devclk();
	jz_clocks.rtcclk = __cpm_get_rtcclk();
	jz_clocks.uartclk = __cpm_get_uartclk();
	jz_clocks.lcdclk = __cpm_get_lcdclk();
	jz_clocks.pixclk = __cpm_get_pixclk();
	jz_clocks.usbclk = __cpm_get_usbclk();
	jz_clocks.i2sclk = __cpm_get_i2sclk();
	jz_clocks.mscclk = __cpm_get_mscclk();
#else  /* URANUS FPGA */

#define FPGACLK 8000000

	jz_clocks.iclk = FPGACLK;
	jz_clocks.sclk = FPGACLK;
	jz_clocks.mclk = FPGACLK;
	jz_clocks.devclk = FPGACLK;
	jz_clocks.rtcclk = FPGACLK;
	jz_clocks.uartclk = FPGACLK;
	jz_clocks.pixclk = FPGACLK;
	jz_clocks.lcdclk = FPGACLK;
	jz_clocks.usbclk = FPGACLK;
	jz_clocks.i2sclk = FPGACLK;
	jz_clocks.mscclk = FPGACLK;
#endif

	printk("CPU clock: %dMHz, System clock: %dMHz, Memory clock: %dMHz, Peripheral clock: %dMHz\n",
	       (jz_clocks.iclk + 500000) / 1000000,
	       (jz_clocks.sclk + 500000) / 1000000,
	       (jz_clocks.mclk + 500000) / 1000000,
	       (jz_clocks.pclk + 500000) / 1000000);
}

static void __init soc_cpm_setup(void)
{
	__cpm_idle_mode();
	__cpm_enable_cko1();
	__cpm_start_all();

	/* get system clocks */
	sysclocks_setup();
}

static void __init soc_harb_setup(void)
{
//	__harb_set_priority(0x00);  /* CIM>LCD>DMA>ETH>PCI>USB>CBB */
//	__harb_set_priority(0x03);  /* LCD>CIM>DMA>ETH>PCI>USB>CBB */
	__harb_set_priority(0x08);  /* DMAC>LCD>CIM>ETH>USB>CIM */
//	__harb_set_priority(0x0a);  /* ETH>LCD>CIM>DMA>PCI>USB>CBB */
}

static void __init soc_emc_setup(void)
{
}

static void __init soc_dmac_setup(void)
{
	__dmac_enable_all_channels();
}

static void __init jz_soc_setup(void)
{
	soc_cpm_setup();
	soc_harb_setup();
	soc_emc_setup();
	soc_dmac_setup();
}

static void __init jz_serial_setup(void)
{
#ifdef CONFIG_SERIAL_8250
	struct uart_port s;

	memset(&s, 0, sizeof(s));

	s.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST;
	s.iotype = UPIO_MEM;
	s.regshift = 2;
	s.uartclk = jz_clocks.uartclk;

	s.line = 0;
	s.membase = (u8 *)UART0_BASE;
	s.irq = IRQ_UART0;
	if (early_serial_setup(&s) != 0) {
		printk(KERN_ERR "Serial ttyS0 setup failed!\n");
	}

	s.line = 1;
	s.membase = (u8 *)UART1_BASE;
	s.irq = IRQ_UART1;
	if (early_serial_setup(&s) != 0) {
		printk(KERN_ERR "Serial ttyS1 setup failed!\n");
	}

	s.line = 2;
	s.membase = (u8 *)UART2_BASE;
	s.irq = IRQ_UART2;
	if (early_serial_setup(&s) != 0) {
		printk(KERN_ERR "Serial ttyS2 setup failed!\n");
	}

	s.line = 3;
	s.membase = (u8 *)UART3_BASE;
	s.irq = IRQ_UART3;
	if (early_serial_setup(&s) != 0) {
		printk(KERN_ERR "Serial ttyS3 setup failed!\n");
	}
#endif
}

void __init plat_mem_setup(void)
{
	char *argptr;

	argptr = prom_getcmdline();

	/* IO/MEM resources. */
	set_io_port_base(0);
	ioport_resource.start = 0x00000000;
	ioport_resource.end = 0xffffffff;
	iomem_resource.start = 0x00000000;
	iomem_resource.end = 0xffffffff;

	_machine_restart = jz_restart;
	_machine_halt = jz_halt;
	pm_power_off = jz_power_off;

	jz_soc_setup();    /* soc specific setup */
	jz_serial_setup(); /* serial port setup */
	jz_board_setup();  /* board specific setup */
}
