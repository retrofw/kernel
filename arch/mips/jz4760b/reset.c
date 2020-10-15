/*
 * linux/arch/mips/jz4760b/reset.c
 *
 * JZ4760B reset routines.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <yliu@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/sched.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/processor.h>
#include <asm/reboot.h>
#include <asm/system.h>
#include <asm/jzsoc.h>

#define RECOVERY_SIGNATURE	0x52454359	/* means "RECOVERY" */
void jz_restart(char *command)
{
#if 1
	printk("Restarting after 4ms\n");
//#if defined(CONFIG_JZ_SYSTEM_AT_CARD)
#if 1
	if ((command != NULL) && !strcmp(command, "recovery")) {
		printk("After restart will goto REVCOVERY mode!");
		cpm_set_scrpad(RECOVERY_SIGNATURE);
	} else {
		cpm_set_scrpad(0);
	}
#endif

	REG_WDT_WCSR = WCSR_PRESCALE4 | WCSR_CLKIN_EXT;
	REG_WDT_WCNT = 0;
	REG_WDT_WDR = JZ_EXTAL / 1000;   /* reset after 4ms */
	REG_TCU_TSCR = TSCR_WDT; /* enable wdt clock */
	REG_WDT_WCER = WCER_TCEN;  /* wdt start */
#else
	printk("Restarting after 1s\n");
	/* clear wakeup status register */
	rtc_write_reg(RTC_HWRSR, 0x0);

	/* Scratch pad register to be reserved */
	rtc_write_reg(RTC_HSPR, HSPR_RTCV);

	/* RTC Alarm Wakeup Enable */
	rtc_set_reg(RTC_HWCR, HWCR_EALM);

	/* Set reset pin low-level assertion time after wakeup: must  > 60ms */
	rtc_write_reg(RTC_HRCR, HRCR_WAIT_TIME(60));

	/* Set minimum wakeup_n pin low-level assertion time for wakeup: 100ms */
	rtc_write_reg(RTC_HWFCR, HWFCR_WAIT_TIME(100));

	rtc_write_reg(RTC_RTCSAR, rtc_read_reg(RTC_RTCSR) + 1);
	rtc_set_reg(RTC_RTCCR, RTCCR_AIE | RTCCR_AE | RTCCR_RTCE); /* alarm enable, alarm interrupt enable */

	/* Put CPU to hibernate mode */
	rtc_write_reg(RTC_HCR, HCR_PD);
#endif
	while (1);
}

void jz_halt(void)
{
	printk(KERN_NOTICE "\n** You can safely turn off the power\n");

	while (1)
		__asm__(".set\tmips3\n\t"
	                "wait\n\t"
			".set\tmips0");
}

void jz_power_off(void)
{
	jz_halt();
}
