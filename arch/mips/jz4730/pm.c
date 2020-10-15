/*
 * linux/arch/mips/jz4730/pm.c
 *
 * Jz4730 Power Management Routines
 *
 * Copyright 2005 Ingenic Semiconductor
 *      Wei Jianli <jlwei@ingenic.cn>
 *      Huang Lihong<lhhuang@ingenic.cn>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/pm.h>
#include <linux/pm_legacy.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>   
#include <linux/sysctl.h>

#include <asm/cacheops.h>
#include <asm/jzsoc.h>

extern void jz_cpu_suspend(void);
extern void jz_cpu_resume(void);

static void jz_board_pm_suspend(void);

#define SAVE(x,s)	sleep_save[SLEEP_SAVE_##x] = REG##s(x)
#define RESTORE(x,s)	REG##s(x) = sleep_save[SLEEP_SAVE_##x]

/*
 * List of global jz4730 peripheral registers to preserve.
 * More ones like core register and general purpose register values 
 * are preserved with the stack pointer in sleep.S.
 */
enum {	SLEEP_SAVE_START = 0,

	/* CPM */
	SLEEP_SAVE_CPM_MSCR, SLEEP_SAVE_CPM_PLCR1,

	/* WDT */
	SLEEP_SAVE_WDT_WTCNT, SLEEP_SAVE_WDT_WTCSR,

	/* OST */
	SLEEP_SAVE_OST_TER,
	SLEEP_SAVE_OST_TCSR0, SLEEP_SAVE_OST_TCSR1, SLEEP_SAVE_OST_TCSR2,
	SLEEP_SAVE_OST_TRDR0, SLEEP_SAVE_OST_TRDR1, SLEEP_SAVE_OST_TRDR2,
	SLEEP_SAVE_OST_TCNT0, SLEEP_SAVE_OST_TCNT1, SLEEP_SAVE_OST_TCNT2,

	/* HARB */
	SLEEP_SAVE_HARB_HAPOR, SLEEP_SAVE_HARB_HMCTR, SLEEP_SAVE_HARB_HMLTR,

	/* EMC */
	SLEEP_SAVE_EMC_SMCR0, SLEEP_SAVE_EMC_SMCR1, SLEEP_SAVE_EMC_SMCR2, SLEEP_SAVE_EMC_SMCR3,
	SLEEP_SAVE_EMC_SMCR4, SLEEP_SAVE_EMC_SMCR5,

	/* GPIO */
	SLEEP_SAVE_GPIO_GPDR0, SLEEP_SAVE_GPIO_GPDR1, SLEEP_SAVE_GPIO_GPDR2, SLEEP_SAVE_GPIO_GPDR3,
	SLEEP_SAVE_GPIO_GPDIR0, SLEEP_SAVE_GPIO_GPDIR1, SLEEP_SAVE_GPIO_GPDIR2,	SLEEP_SAVE_GPIO_GPDIR3,
	SLEEP_SAVE_GPIO_GPODR0, SLEEP_SAVE_GPIO_GPODR1, SLEEP_SAVE_GPIO_GPODR2,	SLEEP_SAVE_GPIO_GPODR3,
	SLEEP_SAVE_GPIO_GPPUR0, SLEEP_SAVE_GPIO_GPPUR1, SLEEP_SAVE_GPIO_GPPUR2, SLEEP_SAVE_GPIO_GPPUR3,
	SLEEP_SAVE_GPIO_GPALR0, SLEEP_SAVE_GPIO_GPALR1, SLEEP_SAVE_GPIO_GPALR2,	SLEEP_SAVE_GPIO_GPALR3,
	SLEEP_SAVE_GPIO_GPAUR0, SLEEP_SAVE_GPIO_GPAUR1, SLEEP_SAVE_GPIO_GPAUR2,	SLEEP_SAVE_GPIO_GPAUR3,
	SLEEP_SAVE_GPIO_GPIDLR0, SLEEP_SAVE_GPIO_GPIDLR1, SLEEP_SAVE_GPIO_GPIDLR2, SLEEP_SAVE_GPIO_GPIDLR3,
	SLEEP_SAVE_GPIO_GPIDUR0, SLEEP_SAVE_GPIO_GPIDUR1, SLEEP_SAVE_GPIO_GPIDUR2, SLEEP_SAVE_GPIO_GPIDUR3,
	SLEEP_SAVE_GPIO_GPIER0, SLEEP_SAVE_GPIO_GPIER1, SLEEP_SAVE_GPIO_GPIER2,	SLEEP_SAVE_GPIO_GPIER3,
	SLEEP_SAVE_GPIO_GPIMR0, SLEEP_SAVE_GPIO_GPIMR1, SLEEP_SAVE_GPIO_GPIMR2, SLEEP_SAVE_GPIO_GPIMR3,
	SLEEP_SAVE_GPIO_GPFR0, SLEEP_SAVE_GPIO_GPFR1, SLEEP_SAVE_GPIO_GPFR2, SLEEP_SAVE_GPIO_GPFR3,

	/* UART(0-3) */
	SLEEP_SAVE_UART0_IER, SLEEP_SAVE_UART0_LCR, SLEEP_SAVE_UART0_MCR, SLEEP_SAVE_UART0_SPR, SLEEP_SAVE_UART0_DLLR, SLEEP_SAVE_UART0_DLHR,
	SLEEP_SAVE_UART1_IER, SLEEP_SAVE_UART1_LCR, SLEEP_SAVE_UART1_MCR, SLEEP_SAVE_UART1_SPR, SLEEP_SAVE_UART1_DLLR, SLEEP_SAVE_UART1_DLHR,
	SLEEP_SAVE_UART2_IER, SLEEP_SAVE_UART2_LCR, SLEEP_SAVE_UART2_MCR, SLEEP_SAVE_UART2_SPR, SLEEP_SAVE_UART2_DLLR, SLEEP_SAVE_UART2_DLHR,
	SLEEP_SAVE_UART3_IER, SLEEP_SAVE_UART3_LCR, SLEEP_SAVE_UART3_MCR, SLEEP_SAVE_UART3_SPR, SLEEP_SAVE_UART3_DLLR, SLEEP_SAVE_UART3_DLHR,

	/* DMAC */
	SLEEP_SAVE_DMAC_DMACR,
	SLEEP_SAVE_DMAC_DSAR0, SLEEP_SAVE_DMAC_DSAR1, SLEEP_SAVE_DMAC_DSAR2, SLEEP_SAVE_DMAC_DSAR3, SLEEP_SAVE_DMAC_DSAR4, SLEEP_SAVE_DMAC_DSAR5, SLEEP_SAVE_DMAC_DSAR6, SLEEP_SAVE_DMAC_DSAR7,
	SLEEP_SAVE_DMAC_DDAR0, SLEEP_SAVE_DMAC_DDAR1, SLEEP_SAVE_DMAC_DDAR2, SLEEP_SAVE_DMAC_DDAR3, SLEEP_SAVE_DMAC_DDAR4, SLEEP_SAVE_DMAC_DDAR5, SLEEP_SAVE_DMAC_DDAR6, SLEEP_SAVE_DMAC_DDAR7,
	SLEEP_SAVE_DMAC_DTCR0, SLEEP_SAVE_DMAC_DTCR1, SLEEP_SAVE_DMAC_DTCR2, SLEEP_SAVE_DMAC_DTCR3, SLEEP_SAVE_DMAC_DTCR4, SLEEP_SAVE_DMAC_DTCR5, SLEEP_SAVE_DMAC_DTCR6, SLEEP_SAVE_DMAC_DTCR7,
	SLEEP_SAVE_DMAC_DRSR0, SLEEP_SAVE_DMAC_DRSR1, SLEEP_SAVE_DMAC_DRSR2, SLEEP_SAVE_DMAC_DRSR3, SLEEP_SAVE_DMAC_DRSR4, SLEEP_SAVE_DMAC_DRSR5, SLEEP_SAVE_DMAC_DRSR6, SLEEP_SAVE_DMAC_DRSR7,
	SLEEP_SAVE_DMAC_DCCSR0, SLEEP_SAVE_DMAC_DCCSR1, SLEEP_SAVE_DMAC_DCCSR2, SLEEP_SAVE_DMAC_DCCSR3, SLEEP_SAVE_DMAC_DCCSR4, SLEEP_SAVE_DMAC_DCCSR5, SLEEP_SAVE_DMAC_DCCSR6, SLEEP_SAVE_DMAC_DCCSR7,

	/* INTC */
	SLEEP_SAVE_INTC_IPR, SLEEP_SAVE_INTC_ISR, SLEEP_SAVE_INTC_IMR, 

	/* Checksum */
	SLEEP_SAVE_CKSUM,

	SLEEP_SAVE_SIZE
};

static unsigned long sleep_save[SLEEP_SAVE_SIZE];

static int jz_pm_do_suspend(void)
{
	unsigned long checksum = 0;
	unsigned long imr = REG_INTC_IMR;
	int i;

	printk("Put cpu into suspend mode.\n");

	/* Mask all interrupts */
	REG_INTC_IMSR = 0xffffffff;

	/* Preserve current time */
	REG_RTC_RSR = xtime.tv_sec;

	REG_CPM_OCR |= CPM_OCR_SUSPEND_PHY0; /* suspend USB PHY 0 */
	REG_CPM_OCR |= CPM_OCR_SUSPEND_PHY1; /* suspend USB PHY 1 */
	REG_CPM_OCR |= CPM_OCR_EXT_RTC_CLK;  /* select the external RTC clock (32.768KHz) */

	/* Disable NAND ctroller */
	REG_EMC_NFCSR &= ~(EMC_NFCSR_NFE | EMC_NFCSR_FCE);

	/*
	 * Temporary solution.  This won't be necessary once
	 * we move this support into the device drivers.
	 * Save the on-chip modules
	 */
	SAVE(UART0_LCR, 8); SAVE(UART0_MCR, 8); SAVE(UART0_SPR, 8);
	REG8(UART0_LCR) |= UARTLCR_DLAB; /* Access to DLLR/DLHR */
	SAVE(UART0_DLLR, 8); SAVE(UART0_DLHR, 8);
	REG8(UART0_LCR) &= ~UARTLCR_DLAB; /* Access to IER */
	SAVE(UART0_IER, 8);

	SAVE(UART1_LCR, 8); SAVE(UART1_MCR, 8); SAVE(UART1_SPR, 8);
	REG8(UART1_LCR) |= UARTLCR_DLAB; /* Access to DLLR/DLHR */
	SAVE(UART1_DLLR, 8); SAVE(UART1_DLHR, 8);
	REG8(UART1_LCR) &= ~UARTLCR_DLAB; /* Access to IER */
	SAVE(UART1_IER, 8);

	SAVE(UART2_LCR, 8); SAVE(UART2_MCR, 8); SAVE(UART2_SPR, 8);
	REG8(UART2_LCR) |= UARTLCR_DLAB; /* Access to DLLR/DLHR */
	SAVE(UART2_DLLR, 8); SAVE(UART2_DLHR, 8);
	REG8(UART2_LCR) &= ~UARTLCR_DLAB; /* Access to IER */
	SAVE(UART2_IER, 8);

	SAVE(UART3_LCR, 8); SAVE(UART3_MCR, 8); SAVE(UART3_SPR, 8);
	REG8(UART3_LCR) |= UARTLCR_DLAB; /* Access to DLLR/DLHR */
	SAVE(UART3_DLLR, 8); SAVE(UART3_DLHR, 8);
	REG8(UART3_LCR) &= ~UARTLCR_DLAB; /* Access to IER */
	SAVE(UART3_IER, 8);

	/* Save vital registers */

	SAVE(OST_TER, 8);
	SAVE(OST_TCSR0, 16); SAVE(OST_TCSR1, 16); SAVE(OST_TCSR2, 16);
	SAVE(OST_TRDR0, 32); SAVE(OST_TRDR1, 32); SAVE(OST_TRDR2, 32);
	SAVE(OST_TCNT0, 32); SAVE(OST_TCNT1, 32); SAVE(OST_TCNT2, 32);

	SAVE(HARB_HAPOR, 32); SAVE(HARB_HMCTR, 32); SAVE(HARB_HMLTR, 32);

	SAVE(EMC_SMCR0, 32); SAVE(EMC_SMCR1, 32); SAVE(EMC_SMCR2, 32); SAVE(EMC_SMCR3, 32); 
	SAVE(EMC_SMCR4, 32); SAVE(EMC_SMCR5, 32);

	SAVE(GPIO_GPDR0, 32); SAVE(GPIO_GPDR1, 32); SAVE(GPIO_GPDR2, 32); 
	SAVE(GPIO_GPDR3, 32);
	SAVE(GPIO_GPDIR0, 32); SAVE(GPIO_GPDIR1, 32); SAVE(GPIO_GPDIR2, 32); 
	SAVE(GPIO_GPDIR3, 32);
	SAVE(GPIO_GPODR0, 32); SAVE(GPIO_GPODR1, 32); SAVE(GPIO_GPODR2, 32); 
	SAVE(GPIO_GPODR3, 32);
	SAVE(GPIO_GPPUR0, 32); SAVE(GPIO_GPPUR1, 32); SAVE(GPIO_GPPUR2, 32); 
	SAVE(GPIO_GPPUR3, 32);
	SAVE(GPIO_GPALR0, 32); SAVE(GPIO_GPALR1, 32); SAVE(GPIO_GPALR2, 32); 
	SAVE(GPIO_GPALR3, 32);
	SAVE(GPIO_GPAUR0, 32); SAVE(GPIO_GPAUR1, 32); SAVE(GPIO_GPAUR2, 32); 
	SAVE(GPIO_GPAUR3, 32);
	SAVE(GPIO_GPIDLR0, 32); SAVE(GPIO_GPIDLR1, 32);	SAVE(GPIO_GPIDLR2, 32); 
	SAVE(GPIO_GPIDLR3, 32);
	SAVE(GPIO_GPIDUR0, 32);	SAVE(GPIO_GPIDUR1, 32);	SAVE(GPIO_GPIDUR2, 32);	
	SAVE(GPIO_GPIDUR3, 32);
	SAVE(GPIO_GPIER0, 32); SAVE(GPIO_GPIER1, 32); SAVE(GPIO_GPIER2, 32); 
	SAVE(GPIO_GPIER3, 32);
	SAVE(GPIO_GPIMR0, 32); SAVE(GPIO_GPIMR1, 32); SAVE(GPIO_GPIMR2, 32); 
	SAVE(GPIO_GPIMR3, 32);
	SAVE(GPIO_GPFR0, 32); SAVE(GPIO_GPFR1, 32); SAVE(GPIO_GPFR2, 32); 
	SAVE(GPIO_GPFR3, 32);

	SAVE(DMAC_DMACR, 32);
	SAVE(DMAC_DSAR0, 32); SAVE(DMAC_DSAR1, 32); SAVE(DMAC_DSAR2, 32); SAVE(DMAC_DSAR3, 32); SAVE(DMAC_DSAR4, 32); SAVE(DMAC_DSAR5, 32); SAVE(DMAC_DSAR6, 32); SAVE(DMAC_DSAR7, 32); 
	SAVE(DMAC_DDAR0, 32); SAVE(DMAC_DDAR1, 32); SAVE(DMAC_DDAR2, 32); SAVE(DMAC_DDAR3, 32); SAVE(DMAC_DDAR4, 32); SAVE(DMAC_DDAR5, 32); SAVE(DMAC_DDAR6, 32); SAVE(DMAC_DDAR7, 32); 
	SAVE(DMAC_DTCR0, 32); SAVE(DMAC_DTCR1, 32); SAVE(DMAC_DTCR2, 32); SAVE(DMAC_DTCR3, 32); SAVE(DMAC_DTCR4, 32); SAVE(DMAC_DTCR5, 32); SAVE(DMAC_DTCR6, 32); SAVE(DMAC_DTCR7, 32); 
	SAVE(DMAC_DRSR0, 32); SAVE(DMAC_DRSR1, 32); SAVE(DMAC_DRSR2, 32); SAVE(DMAC_DRSR3, 32); SAVE(DMAC_DRSR4, 32); SAVE(DMAC_DRSR5, 32); SAVE(DMAC_DRSR6, 32); SAVE(DMAC_DRSR7, 32); 
	SAVE(DMAC_DCCSR0, 32); SAVE(DMAC_DCCSR1, 32); SAVE(DMAC_DCCSR2, 32); SAVE(DMAC_DCCSR3, 32); SAVE(DMAC_DCCSR4, 32); SAVE(DMAC_DCCSR5, 32); SAVE(DMAC_DCCSR6, 32); SAVE(DMAC_DCCSR7, 32);

	SAVE(INTC_IPR, 32);SAVE(INTC_ISR, 32);SAVE(INTC_IMR, 32);

	SAVE(WDT_WTCNT, 32);SAVE(WDT_WTCSR, 8);

	/* Mask all interrupts */
	REG_INTC_IMSR = 0xffffffff; 

	/* Save module clocks */
	SAVE(CPM_MSCR, 32);

        /* Save PLL */
	SAVE(CPM_PLCR1, 32);

	/* Stop module clocks */
	__cpm_stop_uart0();
	__cpm_stop_uart1();
	__cpm_stop_uart2();
	__cpm_stop_uart3();
	__cpm_stop_uhc();
	__cpm_stop_udc();
	__cpm_stop_eth();
	__cpm_stop_cim();
	__cpm_stop_kbc();
	__cpm_stop_scc();
	__cpm_stop_ssi();
	__cpm_stop_ost();

	/* platform-specific pm routine */
	jz_board_pm_suspend();

	/* Clear previous reset status */
	REG_CPM_RSTR &= ~(CPM_RSTR_HR | CPM_RSTR_WR | CPM_RSTR_SR);

	/* Set resume return address */
	REG_CPM_SPR = virt_to_phys(jz_cpu_resume);

	/* Before sleeping, calculate and save a checksum */
	for (i = 0; i < SLEEP_SAVE_SIZE - 1; i++)
		checksum += sleep_save[i];
	sleep_save[SLEEP_SAVE_CKSUM] = checksum;

	/* *** go zzz *** */
	jz_cpu_suspend();
#if 0
	/* after sleeping, validate the checksum */
	checksum = 0;
	for (i = 0; i < SLEEP_SAVE_SIZE - 1; i++)
		checksum += sleep_save[i];

	/* if invalid, display message and wait for a hardware reset */
	if (checksum != sleep_save[SLEEP_SAVE_CKSUM]) {
		/** Add platform-specific message display codes here **/
		while (1);
	}
#endif
	/* Restore PLL */
	RESTORE(CPM_PLCR1, 32);

	/* Restore module clocks */
	RESTORE(CPM_MSCR, 32);

	/* Ensure not to come back here if it wasn't intended */
	REG_CPM_SPR = 0;

	/* Restore registers */

	RESTORE(GPIO_GPDR0, 32); RESTORE(GPIO_GPDR1, 32); RESTORE(GPIO_GPDR2, 32); 
	RESTORE(GPIO_GPDR3, 32);
	RESTORE(GPIO_GPDIR0, 32); RESTORE(GPIO_GPDIR1, 32); RESTORE(GPIO_GPDIR2, 32); 
	RESTORE(GPIO_GPDIR3, 32);
	RESTORE(GPIO_GPODR0, 32); RESTORE(GPIO_GPODR1, 32); RESTORE(GPIO_GPODR2, 32); 
	RESTORE(GPIO_GPODR3, 32);
	RESTORE(GPIO_GPPUR0, 32); RESTORE(GPIO_GPPUR1, 32); RESTORE(GPIO_GPPUR2, 32); 
	RESTORE(GPIO_GPPUR3, 32);
	RESTORE(GPIO_GPALR0, 32); RESTORE(GPIO_GPALR1, 32); RESTORE(GPIO_GPALR2, 32); 
	RESTORE(GPIO_GPALR3, 32);
	RESTORE(GPIO_GPAUR0, 32); RESTORE(GPIO_GPAUR1, 32); RESTORE(GPIO_GPAUR2, 32); 
	RESTORE(GPIO_GPAUR3, 32);
	RESTORE(GPIO_GPIDLR0, 32);RESTORE(GPIO_GPIDLR1, 32);RESTORE(GPIO_GPIDLR2, 32);
	RESTORE(GPIO_GPIDLR3, 32);
	RESTORE(GPIO_GPIDUR0, 32);RESTORE(GPIO_GPIDUR1, 32);RESTORE(GPIO_GPIDUR2, 32); 
	RESTORE(GPIO_GPIDUR3, 32);
	RESTORE(GPIO_GPIER0, 32); RESTORE(GPIO_GPIER1, 32); RESTORE(GPIO_GPIER2, 32);
	RESTORE(GPIO_GPIER3, 32);
	RESTORE(GPIO_GPIMR0, 32); RESTORE(GPIO_GPIMR1, 32); RESTORE(GPIO_GPIMR2, 32); 
	RESTORE(GPIO_GPIMR3, 32);
	RESTORE(GPIO_GPFR0, 32); RESTORE(GPIO_GPFR1, 32); RESTORE(GPIO_GPFR2, 32); 
	RESTORE(GPIO_GPFR3, 32);

	RESTORE(EMC_SMCR0, 32); RESTORE(EMC_SMCR1, 32); RESTORE(EMC_SMCR2, 32); RESTORE(EMC_SMCR3, 32); 
	RESTORE(EMC_SMCR4, 32); RESTORE(EMC_SMCR5, 32); 

	RESTORE(HARB_HAPOR, 32); RESTORE(HARB_HMCTR, 32); RESTORE(HARB_HMLTR, 32);

	RESTORE(OST_TCNT0, 32);	RESTORE(OST_TCNT1, 32);	RESTORE(OST_TCNT2, 32);
	RESTORE(OST_TRDR0, 32);	RESTORE(OST_TRDR1, 32);	RESTORE(OST_TRDR2, 32);
	RESTORE(OST_TCSR0, 16);	RESTORE(OST_TCSR1, 16);	RESTORE(OST_TCSR2, 16);
	RESTORE(OST_TER, 8);

	RESTORE(DMAC_DMACR, 32);
	RESTORE(DMAC_DSAR0, 32); RESTORE(DMAC_DSAR1, 32); RESTORE(DMAC_DSAR2, 32); RESTORE(DMAC_DSAR3, 32); RESTORE(DMAC_DSAR4, 32); RESTORE(DMAC_DSAR5, 32); RESTORE(DMAC_DSAR6, 32); RESTORE(DMAC_DSAR7, 32); 
	RESTORE(DMAC_DDAR0, 32); RESTORE(DMAC_DDAR1, 32); RESTORE(DMAC_DDAR2, 32); RESTORE(DMAC_DDAR3, 32); RESTORE(DMAC_DDAR4, 32); RESTORE(DMAC_DDAR5, 32); RESTORE(DMAC_DDAR6, 32); RESTORE(DMAC_DDAR7, 32); 
	RESTORE(DMAC_DTCR0, 32); RESTORE(DMAC_DTCR1, 32); RESTORE(DMAC_DTCR2, 32); RESTORE(DMAC_DTCR3, 32); RESTORE(DMAC_DTCR4, 32); RESTORE(DMAC_DTCR5, 32); RESTORE(DMAC_DTCR6, 32); RESTORE(DMAC_DTCR7, 32); 
	RESTORE(DMAC_DRSR0, 32); RESTORE(DMAC_DRSR1, 32); RESTORE(DMAC_DRSR2, 32); RESTORE(DMAC_DRSR3, 32); RESTORE(DMAC_DRSR4, 32); RESTORE(DMAC_DRSR5, 32); RESTORE(DMAC_DRSR6, 32); RESTORE(DMAC_DRSR7, 32); 
	RESTORE(DMAC_DCCSR0, 32); RESTORE(DMAC_DCCSR1, 32); RESTORE(DMAC_DCCSR2, 32); RESTORE(DMAC_DCCSR3, 32); RESTORE(DMAC_DCCSR4, 32); RESTORE(DMAC_DCCSR5, 32); RESTORE(DMAC_DCCSR6, 32); RESTORE(DMAC_DCCSR7, 32);

	RESTORE(INTC_IPR, 32);RESTORE(INTC_ISR, 32);RESTORE(INTC_IMR, 32);

	REG_WDT_WTCNT = 0; RESTORE(WDT_WTCSR, 8);

	/*
	 * Temporary solution.  This won't be necessary once
	 * we move this support into the device drivers.
	 * Restore the on-chip modules.
	 */

	/* FIFO control reg, write-only */
	REG8(UART0_FCR) = UARTFCR_FE | UARTFCR_RFLS | UARTFCR_TFLS | UARTFCR_UUE;
	REG8(UART1_FCR) = UARTFCR_FE | UARTFCR_RFLS | UARTFCR_TFLS | UARTFCR_UUE;
	REG8(UART2_FCR) = UARTFCR_FE | UARTFCR_RFLS | UARTFCR_TFLS | UARTFCR_UUE;
	REG8(UART3_FCR) = UARTFCR_FE | UARTFCR_RFLS | UARTFCR_TFLS | UARTFCR_UUE;
 
	REG8(UART0_LCR) |= UARTLCR_DLAB; /* Access to DLLR/DLHR */
	RESTORE(UART0_DLLR, 8);	RESTORE(UART0_DLHR, 8);
	REG8(UART0_LCR) &= ~UARTLCR_DLAB; /* Access to IER */
	RESTORE(UART0_IER, 8);
	RESTORE(UART0_MCR, 8); RESTORE(UART0_SPR, 8); RESTORE(UART0_LCR, 8);

	REG8(UART1_LCR) |= UARTLCR_DLAB; /* Access to DLLR/DLHR */
	RESTORE(UART1_DLLR, 8);	RESTORE(UART1_DLHR, 8);
	REG8(UART1_LCR) &= ~UARTLCR_DLAB; /* Access to IER */
	RESTORE(UART1_IER, 8);
	RESTORE(UART1_MCR, 8); RESTORE(UART1_SPR, 8); RESTORE(UART1_LCR, 8);

	REG8(UART2_LCR) |= UARTLCR_DLAB; /* Access to DLLR/DLHR */
	RESTORE(UART2_DLLR, 8);	RESTORE(UART2_DLHR, 8);
	REG8(UART2_LCR) &= ~UARTLCR_DLAB; /* Access to IER */
	RESTORE(UART2_IER, 8);
	RESTORE(UART2_MCR, 8); RESTORE(UART2_SPR, 8); RESTORE(UART2_LCR, 8);

	REG8(UART3_LCR) |= UARTLCR_DLAB; /* Access to DLLR/DLHR */
	RESTORE(UART3_DLLR, 8);	RESTORE(UART3_DLHR, 8);
	REG8(UART3_LCR) &= ~UARTLCR_DLAB; /* Access to IER */
	RESTORE(UART3_IER, 8);
	RESTORE(UART3_MCR, 8); RESTORE(UART3_SPR, 8); RESTORE(UART3_LCR, 8);

	REG_CPM_OCR &= ~CPM_OCR_SUSPEND_PHY0; /* resume USB PHY 0 */
	REG_CPM_OCR &= ~CPM_OCR_SUSPEND_PHY1; /* resume USB PHY 1 */
#if 0
	REG_CPM_OCR &= ~CPM_OCR_EXT_RTC_CLK;  /* use internal RTC clock (JZ_EXTAL/128 Hz) */
#else
	REG_CPM_OCR |= CPM_OCR_EXT_RTC_CLK;  /* use external RTC clock (32.768 KHz) */
#endif

	/* Enable NAND ctroller */
	REG_EMC_NFCSR |= EMC_NFCSR_NFE;

	/* Restore current time */
	xtime.tv_sec = REG_RTC_RSR;

	/* Restore interrupts */
	REG_INTC_IMSR = imr;
	REG_INTC_IMCR = ~imr;

	return 0;
}

/* NOTES:
 * 1: Pins that are floated (NC) should be set as input and pull-enable.
 * 2: Pins that are pull-up or pull-down by outside should be set as input 
 *    and pull-disable.
 * 3: Pins that are connected to a chipset should be set as pull-disable.
 */
static void jz_board_pm_gpio_setup(void)
{
	/* CIM_D0(IN)/PULL-UP/GP0 */
	__gpio_as_input(0);
	__gpio_enable_pull(0);

	/* CIM_D1(IN)/PULL-UP/GP1 */
	__gpio_as_input(1);
	__gpio_enable_pull(1);

	/* CIM_D2(IN)/PULL-UP/GP2 */
	__gpio_as_input(2);
	__gpio_enable_pull(2);

	/* CIM_D3(IN)/PULL-UP/GP3 */
	__gpio_as_input(3);
	__gpio_enable_pull(3);

	/* CIM_D4(IN)/PULL-DOWN/GP4 */
	__gpio_as_input(4);
	__gpio_enable_pull(4);

	/* CIM_D5(IN)/PULL-DOWN/GP5 */
	__gpio_as_input(5);
	__gpio_enable_pull(5);

	/* CIM_D6(IN)/PULL-DOWN/GP6 */
	__gpio_as_input(6);
	__gpio_enable_pull(6);

	/* CIM_D7(IN)/PULL-DOWN/GP7 */
	__gpio_as_input(7);
	__gpio_enable_pull(7);

	/* CIM_VSYNC(IN)/PULL-DOWN/GP8 */
	__gpio_as_input(8);
	__gpio_enable_pull(8);

	/* CIM_HSYNC(IN)/PULL-UP/GP9 */
	__gpio_as_input(9);
	__gpio_enable_pull(9);

	/* CIM_PCLK(IN)/PULL-DOWN/GP10 */
	__gpio_as_input(10);
	__gpio_enable_pull(10);

	/* CIM_MCLK(OUT)/PULL-DOWN/GP11 */
	__gpio_as_input(11);
	__gpio_enable_pull(11);

	/* DMA_DREQ0(IN)/CHIP_MODE/PULL-UP/GP12 */
	__gpio_as_input(12);
	__gpio_enable_pull(12);

	/* DMA_DACK0(OUT)/PULL-UP/GP13 */  /* GPIO13 */
	__gpio_as_input(13);
	__gpio_disable_pull(13);

	/* GP14 */
	/* GP15 */

	/* RXD3(IN)/PULL-UP/GP16 */
	__gpio_as_input(16);
	__gpio_enable_pull(16);

	/* CTS3(IN)/PULL-UP/GP17 */
	__gpio_as_input(17);
	__gpio_enable_pull(17);

	/* GP18 */
	/* GP19 */
	/* GP20 */

	/* TXD3(OUT)/PULL-UP/GP21 */
	__gpio_as_input(21);
	__gpio_enable_pull(21);

	/* GP22 */

	/* RTS3(OUT)/PULL-UP/GP23 */
	__gpio_as_input(23);
	__gpio_enable_pull(23);

	/* RXD1(IN)/PULL-UP/GP24 */      /* IR_RXD */
	__gpio_as_input(24);
	__gpio_enable_pull(24);

	/* TXD1(OUT)/PULL-UP/GP25 */     /* IR_TXD */
	__gpio_disable_pull(25);
	__gpio_as_output(25);
	__cpm_set_pin(25);

	/* DMA_AEN(OUT)/PULL-UP/GP26 */  /* CIM_PWD_N */
	__gpio_as_input(26);
	__gpio_disable_pull(26);

	/* DMA_EOP(OUT)/PULL-UP/GP27 */  /* SW4 */
	__gpio_as_input(27);
	__gpio_disable_pull(27);

	/* USB_CLK(IN)/PULL-UP/GP28 */
	__gpio_as_input(28);
	__gpio_disable_pull(28);

	/* USB_PPWR0(OUT)/PULL-UP/GP29 */  /* USB_CLK_EN */
	__gpio_disable_pull(29);
	__gpio_as_output(29);
	__cpm_clear_pin(29);  /* disable USB 48MHz clock */
	
	/* GP30 */
	/* GP31 */

	/* PS2_KCLK(IO)/PULL-UP/GP32 */
	__gpio_as_input(32);
	__gpio_enable_pull(32);

	/* PS2_KDATA(IO)/PULL-UP/GP33 */  /* CIM_RST */
	__gpio_as_input(33);
	__gpio_enable_pull(33);

	/* MSC_D0(IO)/PULL-UP/GP34 */
	__gpio_as_input(34);
	__gpio_disable_pull(34);

	/* MSC_D1(IO)/PULL-UP/GP35 */
	__gpio_as_input(35);
	__gpio_disable_pull(35);

	/* MSC_D2(IO)/PULL-UP/GP36 */
	__gpio_as_input(36);
	__gpio_disable_pull(36);

	/* MSC_D3(IO)/PULL-UP/GP37 */
	__gpio_as_input(37);
	__gpio_disable_pull(37);

	/* MSC_CMD(IO)/PULL-UP/GP38 */
	__gpio_as_input(38);
	__gpio_disable_pull(38);

	/* MSC_CLK(OUT)/PULL-UP/GP39 */
	__gpio_as_input(39);
	__gpio_enable_pull(39);

	/* LCD_D0(OUT)/PULL-UP/GP40 */
	__gpio_as_input(40);
	__gpio_enable_pull(40);

	/* LCD_D1(OUT)/PULL-UP/GP41 */
	__gpio_as_input(41);
	__gpio_enable_pull(41);

	/* LCD_D2(OUT)/PULL-UP/GP42 */
	__gpio_as_input(42);
	__gpio_enable_pull(42);

	/* LCD_D3(OUT)/PULL-UP/GP43 */
	__gpio_as_input(43);
	__gpio_enable_pull(43);

	/* LCD_D4(OUT)/PULL-UP/GP44 */
	__gpio_as_input(44);
	__gpio_enable_pull(44);

	/* LCD_D5(OUT)/PULL-UP/GP45 */
	__gpio_as_input(45);
	__gpio_enable_pull(45);

	/* LCD_D6(OUT)/PULL-UP/GP46 */
	__gpio_as_input(46);
	__gpio_enable_pull(46);

	/* LCD_D7(OUT)/PULL-UP/GP47 */
	__gpio_as_input(47);
	__gpio_enable_pull(47);

	/* LCD_D8(OUT)/PULL-DOWN/GP48 */
	__gpio_as_input(48);
	__gpio_enable_pull(48);

	/* LCD_D9(OUT)/PULL-DOWN/GP49 */
	__gpio_as_input(49);
	__gpio_enable_pull(49);

	/* LCD_D10(OUT)/PULL-DOWN/GP50 */
	__gpio_as_input(50);
	__gpio_enable_pull(50);

	/* LCD_D11(OUT)/PULL-DOWN/GP51 */
	__gpio_as_input(51);
	__gpio_enable_pull(51);

	/* LCD_D12(OUT)/PULL-DOWN/GP52 */
	__gpio_as_input(52);
	__gpio_enable_pull(52);

	/* LCD_D13(OUT)/PULL-DOWN/GP53 */
	__gpio_as_input(53);
	__gpio_enable_pull(53);

	/* LCD_D14(OUT)/PULL-DOWN/GP54 */
	__gpio_as_input(54);
	__gpio_enable_pull(54);

	/* LCD_D15(OUT)/PULL-DOWN/GP55 */
	__gpio_as_input(55);
	__gpio_enable_pull(55);

	/* LCD_VSYNC(IN)/PULL-DOWN/GP56 */
	__gpio_as_input(56);
	__gpio_enable_pull(56);

	/* LCD_HSYNC(IN)/PULL-UP/GP57 */
	__gpio_as_input(57);
	__gpio_enable_pull(57);

	/* LCD_PCLK(IN)/PULL-DOWN/GP58 */
	__gpio_as_input(58);
	__gpio_enable_pull(58);

	/* LCD_DE(OUT)/PULL-DOWN/GP59 */
	__gpio_as_input(59);
	__gpio_enable_pull(59);

	/* LCD_SPL(OUT)/PULL-UP/GP60 */
	__gpio_as_input(60);
	__gpio_disable_pull(60);

	/* LCD_CLS(OUT)/PULL-UP/GP61 */
	__gpio_as_input(61);
	__gpio_disable_pull(61);

	/* LCD_PS(OUT)/PULL-UP/GP62 */
	__gpio_as_input(62);
	__gpio_disable_pull(62);

	/* LCD_REV(OUT)/PULL-UP/GP63 */
	__gpio_as_input(63);
	__gpio_enable_pull(63);

	/* SCC0_DAT(IO)/PULL-UP/GP64 */  /* Keypad */
	__gpio_as_input(64);
	__gpio_enable_pull(64);

	/* SCC1_DAT(IO)/PULL-UP/GP65 */  /* SW5 */
	__gpio_as_input(65);
	__gpio_disable_pull(65);

	/* SCC0_CLK(OUT)/PULL-UP/GP66 */  /* PW_O */
	__gpio_disable_pull(66);
	__gpio_as_output(66);
	__cpm_set_pin(66);

	/* SCC1_CLK(OUT)/PULL-UP/GP67 */  /* SW6 */
	__gpio_as_input(67);
	__gpio_disable_pull(67);

	/* SYS_CLK(OUT)/PULL-UP/GP68 */  /* I2S_CLK */
	__gpio_disable_pull(68);

	/* ACRESET_N(OUT)/PULL-UP/GP69 */ /* AK4642 PDN */
	__gpio_disable_pull(69);
	__gpio_as_output(69);
	__cpm_clear_pin(69);

	/* SDATA_OUT(OUT)/PULL-UP/GP70 */ /* I2S_DIN */
	__gpio_disable_pull(70);

	/* SDATA_IN(IN)/PULL-UP/GP71 */  /* I2S_DOUT */
	__gpio_disable_pull(71);

	/* SSI_CLK(OUT)/PULL-UP/GP72 */   /* SSI_CLK */
	__gpio_as_input(72);
	__gpio_enable_pull(72);

	/* SSI_CE1_N(OUT)/PULL-UP/GP73 */ /* SSI_CE1_N */
	__gpio_as_input(73);
	__gpio_enable_pull(73);

	/* SSI_DT(OUT)/PULL-UP/GP74 */    /* SSI_DT */
	__gpio_as_input(74);
	__gpio_enable_pull(74);

	/* SSI_DR(IN)/PULL-UP/GP75 */     /* SSI_DR */
	__gpio_as_input(75);
	__gpio_enable_pull(75);

	/* SSI_CE2_N(OUT)/SSI_GPC/PULL-UP/GP76 */
	__gpio_as_input(76);
	__gpio_enable_pull(76);

	/* BITCLK_IN(IN)/PULL-UP/GP77 */ /* I2S_BITCLK */
	__gpio_disable_pull(77);

	/* SYNC_IN(IN)/PULL-UP/GP78 */  /* I2S_LRCIN */
	__gpio_disable_pull(78);

	/* FRE_N(OUT)/PULL-UP/GP79 */
	__gpio_enable_pull(79);
	__gpio_as_input(79);

	/* FWE_N(OUT)/PULL-UP/GP80 */
	__gpio_enable_pull(80);
	__gpio_as_input(80);

	/* FRB_N(IN)/PULL-UP/GP81 */
	__gpio_enable_pull(81);
	__gpio_as_input(81);

	/* DCS1_N(OUT)/PULL-UP/GP82 */  /* SD_WP */
	__gpio_as_input(82);
	__gpio_enable_pull(82);

	/* CS1_N(OUT)/PULL-UP/GP83 */  /* JACK_PLUG */
	__gpio_as_input(83);
	__gpio_disable_pull(83);

	/* CS2_N(OUT)/PULL-UP/GP84 */   /* DC_DETE */
	__gpio_as_input(84);
	__gpio_disable_pull(84);

	/* CS3_N(OUT)/PULL-UP/GP85 */   /* NAND CS# */
	__gpio_enable_pull(85);
	__gpio_as_input(85);

	/* CS4_N/(OUT)PULL-UP/GP86 */   /* PULL_OFF */
	__gpio_disable_pull(86);
	__gpio_as_output(86);
//	__cpm_set_pin(86);
	__cpm_clear_pin(86);

	/* CS5_N(OUT)/PULL-UP/GP87 */   /* IR_SD */
	__gpio_as_input(87);
	__gpio_disable_pull(87);

	/* INPACK_N(IN)/PULL-UP/GP88 */  /* SW7 */
	__gpio_as_input(88);
	__gpio_disable_pull(88);

	/* BVD2(IN)/PULL-UP/GP89 */      /* SW8 */
	__gpio_as_input(89);
	__gpio_disable_pull(89);

	/* PCE1_N(OUT)/PULL-UP/GP90 */   /* SD_CD_N */
	__gpio_as_input(90);
	__gpio_enable_pull(90);

	/* PSKTSEL_N(OUT)/PULL-UP/GP91 */  /* SD_VCC_3V_EN_N */
	__gpio_disable_pull(91);
	__gpio_as_output(91);
	__cpm_clear_pin(91);

	/* IOIS16_N(IN)/PULL-UP/GP92 */    /* LED_EN */
	__gpio_disable_pull(92);
	__gpio_as_output(92);
	__cpm_clear_pin(92);

	/* PCE2_N(OUT)/PULL-UP/GP93 */     /* LCD_DISP_OFF_N */
	__gpio_disable_pull(93);
	__gpio_as_input(93);

	/* PWM0(OUT)/PULL-UP/GP94 */       /* LCD backlight off */
	__gpio_disable_pull(94);
	__gpio_as_output(94);
	__cpm_clear_pin(94);

	/* PWM1(OUT)/PULL-UP/GP95 */
	__gpio_disable_pull(95);
	__gpio_as_output(95);
	__cpm_clear_pin(95);

	/* PRT(OUT)/PULL-UP/GP96 */        /* RTC_IRQ */
	__gpio_as_input(96);
	__gpio_disable_pull(96);

	/* PRT(OUT)/PULL-UP/GP97 */        /* PW_I */
	__gpio_as_input(97);
	__gpio_disable_pull(97);

	/* PRT(OUT)/PULL-UP/GP98 */        /* Keypad */
	__gpio_as_input(98);
	__gpio_disable_pull(98);

	/* PRT(OUT)/PULL-UP/GP99 */        /* Keypad */
	__gpio_as_input(99);
	__gpio_disable_pull(99);

	/* PRT(OUT)/PULL-UP/GP100 */        /* Keypad */
	__gpio_as_input(100);
	__gpio_disable_pull(100);

	/* PRT(OUT)/PULL-UP/GP101 */        /* Keypad */
	__gpio_as_input(101);
	__gpio_disable_pull(101);

	/* PRT(OUT)/PULL-UP/GP102 */        /* Keypad */
	__gpio_as_input(102);
	__gpio_disable_pull(102);

	/* PRT(OUT)/PULL-UP/GP103 */        /* Keypad */
	__gpio_as_input(103);
	__gpio_enable_pull(103);

	/* PRT(OUT)/PULL-UP/GP104 */        /* Keypad */
	__gpio_as_input(104);
	__gpio_enable_pull(104);

	/* PRT(OUT)/PULL-UP/GP105 */        /* Keypad */
	__gpio_as_input(105);
	__gpio_enable_pull(105);

	/* PRT(OUT)/PULL-UP/GP106 */        /* 5V_ON */
	__gpio_disable_pull(106);
	__gpio_as_output(106);
	__cpm_clear_pin(106);

	/* PRT(IN)/PULL-UP/GP107 */        /* GSM_BOOT */
	__gpio_as_input(107);
	__gpio_enable_pull(107);

	/* PRT(IN)/PULL-UP/GP108 */        /* GSM_RESET */
	__gpio_as_input(108);
	__gpio_enable_pull(108);

	/* PRT(IN)/PULL-UP/GP109 */        /* GSM_EN */
	__gpio_as_input(109);
	__gpio_enable_pull(109);

	/* PRT(IN)/PULL-UP/GP110 */        /* GSM_RING */
	__gpio_as_input(110);
	__gpio_enable_pull(110);

	/* PRT(IN)/UART2_RXD/PULL-UP/GP111 */        /* Keypad */
	__gpio_as_input(111);
	__gpio_enable_pull(111);

	/* MII_TX_EN(OUT)/PULL-UP/GP112 */
	__gpio_as_input(112);
	__gpio_enable_pull(112);

	/* MII_RX_DV(IN)/PULL-UP/GP113 */
	__gpio_as_input(113);
	__gpio_enable_pull(113);

	/* MII_RX_ER(IN)/PULL-UP/GP114 */
	__gpio_as_input(114);
	__gpio_enable_pull(114);

	/* MII_COL(IN)/PULL-UP/GP115 */
	__gpio_as_input(115);
	__gpio_enable_pull(115);

	/* MII_CRS(IN)/PULL-UP/GP116 */
	__gpio_as_input(116);
	__gpio_enable_pull(116);

	/* MII_TXD0(OUT)/PULL-UP/GP117 */
	__gpio_as_input(117);
	__gpio_enable_pull(117);

	/* MII_TXD1(OUT)/PULL-UP/GP118 */
	__gpio_as_input(118);
	__gpio_enable_pull(118);

	/* MII_TXD2(OUT)/PULL-UP/GP119 */
	__gpio_as_input(119);
	__gpio_enable_pull(119);

	/* MII_TXD3(OUT)/PULL-UP/GP120 */
	__gpio_as_input(120);
	__gpio_enable_pull(120);

	/* MII_RXD0(IN)/PULL-UP/GP121 */
	__gpio_as_input(121);
	__gpio_enable_pull(121);

	/* MII_RXD1(IN)/PULL-UP/GP122 */
	__gpio_as_input(122);
	__gpio_enable_pull(122);

	/* MII_RXD2(IN)/PULL-UP/GP123 */
	__gpio_as_input(123);
	__gpio_enable_pull(123);

	/* MII_RXD3(IN)/PULL-UP/GP124 */
	__gpio_as_input(124);
	__gpio_enable_pull(124);

	/* UART2_TXD(OUT)/PULL-UP/GP125 */  /* CHARG_STAT */
	__gpio_as_output(125);
	__gpio_disable_pull(125);
	__cpm_clear_pin(125);

	/* UART0_RXD(IN)/PULL-UP/GP126 */
	__gpio_as_input(126);
	__gpio_enable_pull(126);

	/* UART0_TXD(OUT)/PULL-UP/GP127 */
	__gpio_as_input(127);
	__gpio_enable_pull(127);
}

/*
 * In order to save power most, all gpio pins should be put to their
 * proper states during low power mode.
 */
static void jz_board_pm_suspend(void)
{
	/* Setup the state of all the GPIO pins during low-power mode */
	jz_board_pm_gpio_setup();

	/* Allow next interrupts to wakeup the system.
	 */	
	REG_CPM_WER = 0;              /* Clear all first */

	/* RTC alarm */
	REG_CPM_WER |= 1 << 0;
	REG_CPM_WRER |= 1 << 0;
	REG_CPM_WFER |= 1 << 0;
	__gpio_as_irq_rise_edge(96);

	/* Power_I key */
	REG_CPM_WER |= 1 << 1;
	REG_CPM_WRER |= 1 << 1;
	REG_CPM_WFER |= 1 << 1;
	__gpio_as_irq_rise_edge(97);

	/* enable INTC irq */
	__intc_unmask_irq(IRQ_GPIO3);

#if 0
	/* Enable RTC alarm */
	REG_CPM_WER |= CPM_WER_WERTC;
	REG_RTC_RGR = 32767;
	REG_RTC_RCR &= ~RTC_RCR_AE;
	REG_RTC_RSR = 0;
	REG_RTC_RSAR = 30;
	REG_RTC_RCR = RTC_RCR_AE | RTC_RCR_AIE | RTC_RCR_START;
#endif
}

/*
 * We don't use sleep mode of jz4730 for it has bug, the suspend mode
 * implemented by hibernate mode is used instead of it.
 */
static int jz_pm_do_sleep(void)
{
	printk("It was deprecated, please use /proc/sys/pm/suspend.\n");
#if 0
	unsigned long imr = REG_INTC_IMR;

	/* Preserve current time */
	REG_RTC_RSR = xtime.tv_sec;

	/* Mask all interrupts */
	REG_INTC_IMSR = 0xffffffff;

	/* Just allow next interrupts to wakeup the system.
	 * Note: modify this according to your system.
	 */
	/* RTC alarm */
	__gpio_as_irq_fall_edge(96); /* GPIO 96 */

	/* POWER_I key */
	__gpio_as_irq_rise_edge(97); /* GPIO 97 */

	/* Enable INTC */
	__intc_unmask_irq(IRQ_GPIO3);

	/* Disable modules e.g. LCD backlight */

	/* Stop module clocks */
	__cpm_stop_uhc();

	/* Enter SLEEP mode
	 * Put SDRAM into self-refresh mode.
	 */
	REG_CPM_LPCR &= ~CPM_LPCR_LPM_MASK;
	REG_CPM_LPCR |= CPM_LPCR_LPM_SLEEP;

	__asm__(".set\tmips3\n\t"
		".set noreorder\n\t"
		".align 5\n\t"
		"wait\n\t"
		"nop\n\t"
		".set reorder\n\t"
		".set\tmips0");

	/* Restore to IDLE mode */
	REG_CPM_LPCR &= ~CPM_LPCR_LPM_MASK;
	REG_CPM_LPCR |= CPM_LPCR_LPM_IDLE;

	/* Restore clock of usb host */
	__cpm_start_uhc();

	/* Restore interrupts */
	REG_INTC_IMSR = imr;
	REG_INTC_IMCR = ~imr;
	
	/* Restore current time */
	xtime.tv_sec = REG_RTC_RSR;
#endif
	return 0;
}

#define K0BASE  KSEG0
void jz_flush_cache_all(void)
{
	unsigned long addr;

	/* Clear CP0 TagLo */
	asm volatile ("mtc0 $0, $28\n\t"::);

	for (addr = K0BASE; addr < (K0BASE + 0x4000); addr += 32) {
		asm volatile (
			".set mips3\n\t"
			" cache %0, 0(%1)\n\t"
			".set mips2\n\t"
			:
			: "I" (Index_Writeback_Inv_D), "r"(addr));

		asm volatile (
			".set mips3\n\t"
			" cache %0, 0(%1)\n\t"
			".set mips2\n\t"
			:
			: "I" (Index_Store_Tag_I), "r"(addr));
	}

	asm volatile ("sync\n\t"::);

	/* invalidate BTB */
	asm volatile (
		".set mips32\n\t"
		" mfc0 %0, $16, 7\n\t"
		" nop\n\t"
		" ori $0, 2\n\t"
		" mtc0 %0, $16, 7\n\t"
		" nop\n\t"
		".set mips2\n\t"
		:
		: "r"(addr));
}

/* Put CPU to HIBERNATE mode */
int jz_pm_suspend(void)
{
	int retval;

	pm_send_all(PM_SUSPEND, (void *)3);

	retval = jz_pm_do_suspend();

	pm_send_all(PM_RESUME, (void *)0);

	return retval;
}

#if 0
/* Put CPU to SLEEP mode */
int jz_pm_sleep(void)
{
	return jz_pm_do_sleep();
}

/* Put CPU to IDLE mode, used for dpm in linux 2.4 */
void jz_pm_idle(void)
{
	local_irq_disable();
	if (!need_resched()) {
		local_irq_enable();
		cpu_wait();
	}
}
#endif

#ifdef CONFIG_SYSCTL

/*
 * Use a temporary sysctl number. Horrid, but will be cleaned up in 2.6
 * when all the PM interfaces exist nicely.
 */
#define CTL_PM_SUSPEND   1
#define CTL_PM_HIBERNATE 2

/*----------------------------------------------------------------------------
 * Power Management sleep sysctl proc interface
 *
 * A write to /proc/sys/pm/suspend invokes this function 
 * which initiates a sleep.
 *--------------------------------------------------------------------------*/
static int sysctl_jz_pm_sleep(void)
{
	return jz_pm_suspend();
}

static struct ctl_table pm_table[] =
{
	{
		.ctl_name	= CTL_UNNUMBERED,
		.procname	= "suspend",
		.data		= NULL,
		.maxlen		= 0,
		.mode		= 0600,
		.proc_handler	= &sysctl_jz_pm_sleep,
	},
	{ .ctl_name = 0}
};

static struct ctl_table pm_dir_table[] =
{
	{
		.ctl_name	= CTL_UNNUMBERED,
		.procname	= "pm",
		.mode		= 0555,
		.child		= pm_table,
	},
	{ .ctl_name = 0}
};

#endif /* CONFIG_SYSCTL */

/*
 * Initialize power interface
 */
static int __init jz_pm_init(void)
{
	printk("Power Management for JZ\n");

#ifdef CONFIG_SYSCTL
	register_sysctl_table(pm_dir_table);
#endif

	return 0;
}

module_init(jz_pm_init);
