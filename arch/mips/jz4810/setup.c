/*
 * linux/arch/mips/jz4810/common/setup.c
 * 
 * JZ4810 common setup routines.
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

#if 1
static void serial_putc (const char c)
{
	volatile u8 *uart_lsr = (volatile u8 *)(UART2_BASE + OFF_LSR);
	volatile u8 *uart_tdr = (volatile u8 *)(UART2_BASE + OFF_TDR);

	if (c == '\n') serial_putc ('\r');

	/* Wait for fifo to shift out some bytes */
	while ( !((*uart_lsr & (UART_LSR_TDRQ | UART_LSR_TEMT)) == 0x60) );

	*uart_tdr = (u8)c;
}

static void serial_puts (const char *s)
{
	while (*s) {
		serial_putc (*s++);
	}
}
#endif

static void __init sysclocks_setup(void)
{
#ifndef CONFIG_MIPS_JZ_EMURUS /* FPGA */
	jz_clocks.cclk = __cpm_get_cclk();
	jz_clocks.hclk = __cpm_get_h0clk();
	jz_clocks.pclk = __cpm_get_pclk();
	jz_clocks.mclk = __cpm_get_mclk();
	jz_clocks.h1clk = __cpm_get_h1clk();
	jz_clocks.pixclk = __cpm_get_pixclk();
	jz_clocks.i2sclk = __cpm_get_i2sclk();
	jz_clocks.otgclk = __cpm_get_otgclk();
	jz_clocks.mscclk = __cpm_get_mscclk(0);
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
	 */
	__cpm_start_all();

	/* Enable CKO to external memory */
	__cpm_enable_cko();

	/* CPU enters IDLE mode when executing 'wait' instruction */
	__cpm_idle_mode();

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
	__cpm_start_emc();
	int haili = 10000;
	while(haili --);

//	REG_EMC_PMEMBS1  = 0;
//	REG_EMC_PMEMBS0  = 0;

	REG_EMC_PMEMBS1 = 0xff000000;
	REG_EMC_PMEMBS0 = 0xff000000;
}

static void __init soc_dmac_setup(void)
{
	__dmac_enable_module(0);
	__dmac_enable_module(1);
}

static void __init jz_soc_setup(void)
{
//	soc_cpm_setup();
//	soc_harb_setup();
//	soc_emc_setup();
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
	s.uartclk = 12000000;
//	s.uartclk = jz_clocks.extalclk ;

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

void __init jz_test_setup(void)
{
	int volatile x = 1000;
	int i;

	printk(":111%s:%d\n",__FUNCTION__,__LINE__);

	printk("kernel test cs1");
#if 1
//	for (i = 0; i < 1000; i++) { 
	while(1) {
		__gpio_as_output0(21);

		x = 10000;
		while(x--)
			;
		__gpio_as_output1(21);
	}
#endif
	printk("kernel test cs1 ok");

#if 0

	REG_CPM_CPCCR |= CPM_CPCCR_CE; 
	REG_CPM_LPCDR = 7;
	REG_CPM_UHCCDR = 7;
	REG_CPM_GPSCDR = 2;
	REG_CPM_GPUCDR = 1;


	REG_CPM_PSWC0ST = 0;
	REG_CPM_PSWC1ST = 8;
	REG_CPM_PSWC2ST = 10;
	REG_CPM_PSWC3ST = 0;

	REG_CPM_OPCR &= ~CPM_OPCR_UHCPHY_DISABLE;
	REG_CPM_OPCR |= CPM_OPCR_UDCPHY_ENABLE;
#endif

#if 0
	/*stop some clk*/
	__cpm_stop_ipu();
	__cpm_stop_lcd();
	__cpm_stop_tve();
	__cpm_stop_Cim();
	__cpm_stop_mdma();
	__cpm_stop_uhc();
	__cpm_stop_gps();
	__cpm_stop_ssi2();
	__cpm_stop_ssi1();

	__cpm_stop_uart3();
	__cpm_stop_uart2();
	__cpm_stop_uart0();

	__cpm_stop_sadc();
	REG_CPM_CLKGR0 |= (0xfff < 2);
	REG_CPM_CLKGR1 = 0xffffffff;
	__cpm_stop_bch();
	__cpm_stop_dmac();
	printk("some clk have been stop!\n");

#endif
#if 0
	__cpm_start_emc();
	printk("\n-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=\nREG_CPM_CLKGR0=0x%x\n",REG_CPM_CLKGR0);
	int haili = 10000;
	while(haili --);
	printk("===================\n");
	REG_EMC_PMEMBS1  = 0;
	REG_EMC_PMEMBS0  = 0;

	REG_EMC_PMEMBS1 |= (0xff << 24);
	REG_EMC_PMEMBS0 |= (0xff << 24);
	REG_EMC_PMEMBS0 |= (0xff << 8);

	printk("---------------PMEMBS1=0x%x,PMEMBS0=0x%x\n",REG_EMC_PMEMBS1,REG_EMC_PMEMBS0);
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
	pm_power_off = jz_power_off;

	jz_soc_setup();
	jz_serial_setup();
	jz_board_setup();

#ifdef CONFIG_PM
        jz_pm_init();
#endif
}

