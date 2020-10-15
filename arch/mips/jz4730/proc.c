/*
 * linux/arch/mips/jz4730/proc.c
 *
 * /proc/jz/ procfs for on-chip peripherals.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>

#include <asm/uaccess.h>
#include <asm/jzsoc.h>

struct proc_dir_entry *proc_jz_root;

/*
 * EMC Module
 */
static int emc_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;

	len += sprintf (page+len, "BCR:       0x%08x\n", REG_EMC_BCR);
	len += sprintf (page+len, "SMCR(0-5): 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n", REG_EMC_SMCR0, REG_EMC_SMCR1, REG_EMC_SMCR2, REG_EMC_SMCR3, REG_EMC_SMCR4, REG_EMC_SMCR5);
	len += sprintf (page+len, "SACR(0-5): 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n", REG_EMC_SACR0, REG_EMC_SACR1, REG_EMC_SACR2, REG_EMC_SACR3, REG_EMC_SACR4, REG_EMC_SACR5);
	len += sprintf (page+len, "DMCR:      0x%08x\n", REG_EMC_DMCR);
	len += sprintf (page+len, "RTCSR:     0x%04x\n", REG_EMC_RTCSR);
	len += sprintf (page+len, "RTCOR:     0x%04x\n", REG_EMC_RTCOR);
	len += sprintf (page+len, "DMAR(0-1): 0x%08x 0x%08x\n", REG_EMC_DMAR1, REG_EMC_DMAR2);
	return len;
}

/* 
 * Power Manager Module
 */
static int pmc_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;
	unsigned long lpcr = REG_CPM_LPCR;
	unsigned long mscr = REG_CPM_MSCR;

	len += sprintf (page+len, "LPCR           : 0x%08lx\n", lpcr);
	len += sprintf (page+len, "Low Power Mode : %s\n", 
			((lpcr & CPM_LPCR_LPM_MASK) == (CPM_LPCR_LPM_IDLE)) ?
			"idle" : (((lpcr & CPM_LPCR_LPM_MASK) == (CPM_LPCR_LPM_SLEEP)) ? "sleep" : "hibernate"));
	len += sprintf (page+len, "Doze Mode      : %s\n", 
			(lpcr & CPM_LPCR_DOZE) ? "on" : "off");
	if (lpcr & CPM_LPCR_DOZE)
		len += sprintf (page+len, "     duty      : %d\n", (int)((lpcr & CPM_LPCR_DUTY_MASK) >> CPM_LPCR_DUTY_BIT));
	len += sprintf (page+len, "CKO1           : %s\n",
			(REG_CPM_CFCR & CPM_CFCR_CKOEN1) ? "enable" : "disable");
	len += sprintf (page+len, "UART0          : %s\n",
			(mscr & CPM_MSCR_MSTP_UART0) ? "stopped" : "running");
	len += sprintf (page+len, "UART1          : %s\n",
			(mscr & CPM_MSCR_MSTP_UART1) ? "stopped" : "running");
	len += sprintf (page+len, "UART2          : %s\n",
			(mscr & CPM_MSCR_MSTP_UART2) ? "stopped" : "running");
	len += sprintf (page+len, "UART3          : %s\n",
			(mscr & CPM_MSCR_MSTP_UART3) ? "stopped" : "running");
	len += sprintf (page+len, "OST            : %s\n",
			(mscr & CPM_MSCR_MSTP_OST) ? "stopped" : "running");
	len += sprintf (page+len, "DMAC           : %s\n",
			(mscr & CPM_MSCR_MSTP_DMAC) ? "stopped" : "running");
	len += sprintf (page+len, "ETH            : %s\n",
			(mscr & CPM_MSCR_MSTP_ETH) ? "stopped" : "running");
	len += sprintf (page+len, "UHC/UDC        : %s\n",
			(mscr & CPM_MSCR_MSTP_UHC) ? "stopped" : "running");
	len += sprintf (page+len, "PWM0           : %s\n",
			(mscr & CPM_MSCR_MSTP_PWM0) ? "stopped" : "running");
	len += sprintf (page+len, "PWM1           : %s\n",
			(mscr & CPM_MSCR_MSTP_PWM1) ? "stopped" : "running");
	len += sprintf (page+len, "I2C            : %s\n",
			(mscr & CPM_MSCR_MSTP_I2C) ? "stopped" : "running");
	len += sprintf (page+len, "SSI            : %s\n",
			(mscr & CPM_MSCR_MSTP_SSI) ? "stopped" : "running");
	len += sprintf (page+len, "SCC            : %s\n",
			(mscr & CPM_MSCR_MSTP_SCC) ? "stopped" : "running");
	return len;
}

static int pmc_write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	REG_CPM_MSCR = simple_strtoul(buffer, 0, 16);
	return count;
}

/*
 * Clock Generation Module
 */
static int cgm_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;
	unsigned int cfcr = REG_CPM_CFCR;
	unsigned int plcr1 = REG_CPM_PLCR1;
	unsigned int div[] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32};
	unsigned int od[4] = {1, 2, 2, 4};


	len += sprintf (page+len, "PLCR1          : 0x%08x\n", plcr1);
	len += sprintf (page+len, "CFCR           : 0x%08x\n", cfcr);
	len += sprintf (page+len, "PLL            : %s\n", 
			(plcr1 & CPM_PLCR1_PLL1EN) ? "ON" : "OFF");
	len += sprintf (page+len, "NF:NR:NO       : %d:%d:%d\n",
			__cpm_plcr1_fd() + 2,
			__cpm_plcr1_rd() + 2,
			od[__cpm_plcr1_od()]
		);
	len += sprintf (page+len, "I:S:M:P        : %d:%d:%d:%d\n", 
			div[(cfcr & CPM_CFCR_IFR_MASK) >> CPM_CFCR_IFR_BIT],
			div[(cfcr & CPM_CFCR_SFR_MASK) >> CPM_CFCR_SFR_BIT],
			div[(cfcr & CPM_CFCR_MFR_MASK) >> CPM_CFCR_MFR_BIT],
			div[(cfcr & CPM_CFCR_PFR_MASK) >> CPM_CFCR_PFR_BIT]
		);
	len += sprintf (page+len, "PLL Freq       : %d MHz\n", __cpm_get_pllout()/1000000);
	len += sprintf (page+len, "ICLK           : %d MHz\n", __cpm_get_iclk()/1000000);
	len += sprintf (page+len, "SCLK           : %d MHz\n", __cpm_get_sclk()/1000000);
	len += sprintf (page+len, "MCLK           : %d MHz\n", __cpm_get_mclk()/1000000);
	len += sprintf (page+len, "PCLK           : %d MHz\n", __cpm_get_pclk()/1000000);
	len += sprintf (page+len, "DEVCLK         : %d MHz\n", __cpm_get_devclk()/1000000);
	len += sprintf (page+len, "RTCCLK         : %d KHz\n", __cpm_get_rtcclk()/1000);
	len += sprintf (page+len, "USBCLK         : %d MHz\n", __cpm_get_usbclk()/1000000);
#if defined(CONFIG_FB_JZ)
	len += sprintf (page+len, "LCDCLK         : %d MHz\n", __cpm_get_lcdclk()/1000000);
	len += sprintf (page+len, "PIXCLK         : %d MHz\n", __cpm_get_pixclk()/1000000);
#endif
	return len;
}

static int cgm_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	REG_CPM_CFCR = simple_strtoul(buffer, 0, 16);
	return count;
}

/* 
 * WDT
 */
static int wdt_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;

	len += sprintf (page+len, "WDT_WTCSR   : 0x%08x\n", REG_WDT_WTCSR);
	len += sprintf (page+len, "WDT_WTCNT   : 0x%08x\n", REG_WDT_WTCNT);

	return len;
}

static int wdt_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	unsigned long cnt = simple_strtoul(buffer, 0, 16);

	REG_WDT_WTCNT = cnt;
	REG_WDT_WTCSR = WDT_WTCSR_START;

	return count;
}

/*
 * PWM
 */

static int proc_jz_pwm_read_byte(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	return sprintf (page, "0x%02x\n", REG8(data));
}

static int proc_jz_pwm_read_word(char *page, char **start, off_t off,
			      int count, int *eof, void *data)
{
     	return sprintf (page, "0x%04x\n", REG16(data));
}

static int proc_jz_pwm_write_byte(struct file *file, const char *buffer, unsigned long count, void *data)
{
	REG8(data) = simple_strtoul(buffer, 0, 16);
	return count;
}
                         
static int proc_jz_pwm_write_word(struct file *file, const char *buffer, unsigned long count, void *data)
{
	REG16(data) =  simple_strtoul(buffer, 0, 16);
	return count;
}

#define PWM_NUM 2

static int jz_pwm_proc_init(void) 
{
	struct proc_dir_entry *proc_jz_pwm, *res;
	char name[16];
	unsigned char i;

	for (i = 0; i < PWM_NUM; i++) {
		sprintf(name, "pwm%d", i);
		proc_jz_pwm = proc_mkdir(name, proc_jz_root);
		res = create_proc_entry("control", 0600, proc_jz_pwm);
		if ( res) {
			res->read_proc  = proc_jz_pwm_read_byte;
			res->write_proc = proc_jz_pwm_write_byte;
			if (i)
				res->data = (void * )PWM_CTR(1);
			else
				res->data = (void * )PWM_CTR(0);
		}
		res = create_proc_entry("period", 0600, proc_jz_pwm);
		if ( res) {
			res->read_proc  = proc_jz_pwm_read_word;
			res->write_proc = proc_jz_pwm_write_word;
			if (i)
				res->data = (void *)PWM_PER(1);
			else
				res->data = (void *)PWM_PER(0);
		}
		res = create_proc_entry("duty", 0600, proc_jz_pwm);
		if ( res) {
			res->read_proc  = proc_jz_pwm_read_word;
			res->write_proc = proc_jz_pwm_write_word;
			if (i)
				res->data = (void * )PWM_DUT(1);
			else
				res->data = (void * )PWM_DUT(0);
		}
	}
	return 0;
}

/*
 * /proc/jz/xxx entry
 *
 */
static int __init jz_proc_init(void)
{
	struct proc_dir_entry *entry;

	/* create /proc/jz */
	proc_jz_root = proc_mkdir("jz", 0);

	/* create /proc/jz/emc */
	entry = create_proc_entry("emc", 0644, proc_jz_root);
	if (entry) {
		entry->read_proc = emc_read_proc;
		entry->write_proc = NULL;
		entry->data = NULL;
	}

	/* create /proc/jz/pmc */
	entry = create_proc_entry("pmc", 0644, proc_jz_root);
	if (entry) {
		entry->read_proc = pmc_read_proc;
		entry->write_proc = pmc_write_proc;
		entry->data = NULL;
	}

	/* create /proc/jz/cgm */
	entry = create_proc_entry("cgm", 0644, proc_jz_root);
	if (entry) {
		entry->read_proc = cgm_read_proc;
		entry->write_proc = cgm_write_proc;
		entry->data = NULL;
	}

	/* create /proc/jz/wdt */
	entry = create_proc_entry("wdt", 0644, proc_jz_root);
	if (entry) {
		entry->read_proc = wdt_read_proc;
		entry->write_proc = wdt_write_proc;
		entry->data = NULL;
	}

	/* PWM */
	jz_pwm_proc_init();

	return 0;
}

__initcall(jz_proc_init);
