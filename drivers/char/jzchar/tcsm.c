/*
 * linux/drivers/char/jzchar/tcsm.c
 *
 * Virtual device driver with tricky appoach to manage TCSM 
 *
 * Copyright (C) 2006  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/spinlock.h>

#include <asm/mipsregs.h>
#include <asm/mipsmtregs.h>

#include <asm/irq.h>
#include <asm/thread_info.h>
#include <asm/uaccess.h>
#include <asm/jzsoc.h>

#include "jzchars.h"

MODULE_AUTHOR("Jianli Wei<jlwei@ingenic.cn>");
MODULE_DESCRIPTION("Virtual Driver of TCSM");
MODULE_LICENSE("GPL");

/*
 * fops routines
 */

static int tcsm_open(struct inode *inode, struct file *filp);
static int tcsm_release(struct inode *inode, struct file *filp);
static ssize_t tcsm_read(struct file *filp, char *buf, size_t size, loff_t *l);
static ssize_t tcsm_write(struct file *filp, const char *buf, size_t size, loff_t *l);
static int tcsm_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);

static struct file_operations tcsm_fops = 
{
	open:		tcsm_open,
	release:	tcsm_release,
	read:		tcsm_read,
	write:		tcsm_write,
	ioctl:		tcsm_ioctl,
};

static int tcsm_open(struct inode *inode, struct file *filp)
{
  struct pt_regs *info = task_pt_regs(current);

#ifdef CONFIG_SOC_JZ4760B
  REG_CPM_CLKGR1 &= ~(CLKGR1_AUX | CLKGR1_OSD );
#endif

#ifdef CONFIG_SOC_JZ4770
  REG_CPM_CLKGR1 &= ~ (CLKGR1_AUX | CLKGR1_VPU);
#endif

#if defined(CONFIG_SOC_JZ4760) || defined(CONFIG_SOC_JZ4760B)
  REG_CPM_CLKGR1 &= ~ (CLKGR1_AHB1 | CLKGR1_CABAC | CLKGR1_SRAM | CLKGR1_DCT | CLKGR1_DBLK | CLKGR1_MC | CLKGR1_ME);
  REG_CPM_LCR &= ~(1 << 30);
  while(REG_CPM_LCR & LCR_PDAHB1S);
  REG_CPM_CLKGR1 |= (CLKGR1_ME);
#endif
 
  info->cp0_status &= ~0x10;// clear UM bit
  info->cp0_status |= 0x08000000; // set RP bit   a tricky
  
  return 0;
}

static int tcsm_release(struct inode *inode, struct file *filp)
{
  struct pt_regs *info = task_pt_regs(current);

#ifdef CONFIG_SOC_JZ4760B
  REG_CPM_CLKGR1 |= (CLKGR1_AUX | CLKGR1_OSD);
#endif

#ifdef CONFIG_SOC_JZ4770
  REG_CPM_CLKGR1 |= (CLKGR1_AUX | CLKGR1_VPU);
#endif

#if defined(CONFIG_SOC_JZ4760) || defined(CONFIG_SOC_JZ4760B)
  REG_CPM_CLKGR1 |= (CLKGR1_AHB1 | CLKGR1_CABAC | CLKGR1_SRAM | CLKGR1_DCT | CLKGR1_DBLK | CLKGR1_MC | CLKGR1_ME);
  REG_CPM_LCR |= (1 << 30);
#endif

  info->cp0_status |= 0x10;// set UM bit
  info->cp0_status &= ~0x08000000; // clear RP bit  a tricky
 
  return 0;	
}

static ssize_t tcsm_read(struct file *filp, char *buf, size_t size, loff_t *l)
{
	printk("tcsm: read is not implemented\n");
	return -1;
}

static ssize_t tcsm_write(struct file *filp, const char *buf, size_t size, loff_t *l)
{
	printk("tcsm: write is not implemented\n");
	return -1;
}

static int tcsm_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	printk("tcsm: ioctl is not implemented\n");
	return ret;
}

/*
 * Module init and exit
 */

static int __init tcsm_init(void)
{
	int ret;

	ret = jz_register_chrdev(TCSM_MINOR, "tcsm", &tcsm_fops, NULL);
	if (ret < 0) {
		return ret;
	}

	printk(JZ_SOC_NAME": Virtual Driver of TCSM registered.\n");
	return 0;
}

static void __exit tcsm_exit(void)
{
	jz_unregister_chrdev(TCSM_MINOR, "tcsm");
}

module_init(tcsm_init);
module_exit(tcsm_exit);
