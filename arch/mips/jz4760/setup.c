/*
 * linux/arch/mips/jz4760/common/setup.c
 *
 * JZ4760 common setup routines.
 *
 * Copyright (C) 2006 Ingenic Semiconductor Inc.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
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

#ifdef CONFIG_PM
#include <asm/suspend.h>
#endif

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

#ifdef CONFIG_PM
extern void jz_pm_hibernate(void);
#else
#define jz_pm_hibernate NULL
#endif

static void __init sysclocks_setup(void)
{
#ifndef CONFIG_MIPS_JZ_EMURUS /* FPGA */

	jz_clocks.cclk = cpm_get_clock(CGU_CCLK);
	jz_clocks.hclk = cpm_get_clock(CGU_HCLK);
	jz_clocks.pclk = cpm_get_clock(CGU_PCLK);
	jz_clocks.mclk = cpm_get_clock(CGU_MCLK);
	jz_clocks.h1clk = cpm_get_clock(CGU_H2CLK);
	jz_clocks.pixclk = cpm_get_clock(CGU_LPCLK);
	jz_clocks.i2sclk = cpm_get_clock(CGU_I2SCLK);
	jz_clocks.otgclk = cpm_get_clock(CGU_OTGCLK);
	jz_clocks.mscclk = cpm_get_clock(CGU_MSCCLK);
	jz_clocks.extalclk = __cpm_get_extalclk();
	jz_clocks.rtcclk = __cpm_get_rtcclk();

#else

#define FPGACLK 8000000

	jz_clocks.cclk = FPGACLK;
	jz_clocks.hclk = FPGACLK;
	jz_clocks.pclk = FPGACLK;
	jz_clocks.mclk = FPGACLK;
	jz_clocks.h1clk = FPGACLK;
	jz_clocks.pixclk = FPGACLK;
	jz_clocks.i2sclk = FPGACLK;
	jz_clocks.usbclk = FPGACLK;
	jz_clocks.mscclk = FPGACLK;
	jz_clocks.extalclk = FPGACLK;
	jz_clocks.rtcclk = FPGACLK;
#endif

	printk("CPU clock: %dMHz, System clock: %dMHz, Peripheral clock: %dMHz, Memory clock: %dMHz\n",
	       (jz_clocks.cclk + 500000) / 1000000,
	       (jz_clocks.hclk + 500000) / 1000000,
	       (jz_clocks.pclk + 500000) / 1000000,
	       (jz_clocks.mclk + 500000) / 1000000);
}

static void __init soc_cpm_setup(void)
{
	/* Start all module clocks
	 * cpm_start_clock(CGM_ALL_MODULE);
	 */

	/* Enable device DMA */
	cpm_start_clock(CGM_DMAC);

	/* CPU enters IDLE mode when executing 'wait' instruction */
	CMSREG32(CPM_LCR, LCR_LPM_IDLE, LCR_LPM_MASK);

	/* Setup system clocks */
	sysclocks_setup();
}

static void __init soc_harb_setup(void)
{
//	__harb_set_priority(0x00);  /* CIM>LCD>DMA>ETH>PCI>USB>CBB */
//	__harb_set_priority(0x03);  /* LCD>CIM>DMA>ETH>PCI>USB>CBB */
//	__harb_set_priority(0x0a);  /* ETH>LCD>CIM>DMA>PCI>USB>CBB */
}

static void __init soc_emc_setup(void)
{
	cpm_start_clock(CGM_EMC);

	OUTREG32(EMC_PMEMBS1, 0xff000000);
	OUTREG32(EMC_PMEMBS0, 0xff000000);
}

static void __init soc_dmac_setup(void)
{
	__dmac_enable_module(0);
	__dmac_enable_module(1);
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
	REG8(UART0_FCR) |= UARTFCR_UUE; /* enable UART module */
	memset(&s, 0, sizeof(s));
	s.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST;
	s.iotype = SERIAL_IO_MEM;
	s.regshift = 2;
	s.uartclk = jz_clocks.extalclk ;

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

	__asm__ (
		"li    $2, 0xa9000000 \n\t"
		"mtc0  $2, $5, 4      \n\t"
		"nop                  \n\t"
		::"r"(2));

	/* IO/MEM resources. Which will be the addtion value in `inX' and
	 * `outX' macros defined in asm/io.h */
	set_io_port_base(0);
	ioport_resource.start	= 0x00000000;
	ioport_resource.end	= 0xffffffff;
	iomem_resource.start	= 0x00000000;
	iomem_resource.end	= 0xffffffff;

	_machine_restart = jz_restart;
	_machine_halt = jz_halt;
	pm_power_off = jz_pm_hibernate;

	jz_soc_setup();
	jz_serial_setup();
	jz_board_setup();

#ifdef CONFIG_PM
	jz_pm_init();
#endif

}

