/*
 * linux/drivers/char/jzchar/jz_ow.c
 *
 * One Wire Bus test driver
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
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/jzsoc.h>
#include "jzchars.h"

#define OW_CPU_READ_ROM           1
#define OW_INTC_READ_ROM         1
#define OW_CPU_SEARCH_ROM       0
#define OW_INTC_SEARCH_ROM     0

#define OW_DEBUG  0
#if OW_DEBUG
#define OWI_MAX   10
static char CFG[OWI_MAX];
static char CTL[OWI_MAX];
static char STS[OWI_MAX];
static char DAT[OWI_MAX];
static char DIV[OWI_MAX];
static void owi_register_dump(int i)
{
	CFG[i]=	REG_OWI_CFG;
	CTL[i]= REG_OWI_CTL;
	STS[i]= REG_OWI_STS;
	DAT[i]= REG_OWI_DAT;
	DIV[i]= REG_OWI_DIV;
}
static void owi_register_print(int i)
{
	printk(" REG_OWI_CFG: 0x%08x\n", CFG[i]);
	printk(" REG_OWI_CTL: 0x%08x\n", CTL[i]);
	printk(" REG_OWI_STS: 0x%08x\n", STS[i]);
	printk(" REG_OWI_DAT: 0x%08x\n", DAT[i]);
	printk(" REG_OWI_DIV: 0x%08x\n", DIV[i]);
}
#endif

static DECLARE_WAIT_QUEUE_HEAD (ow_wait_queue);

/*
 * fops routines
 */
static int ow_open(struct inode *inode, struct file *filp);
static int ow_release(struct inode *inode, struct file *filp);
static ssize_t ow_read(struct file *filp, char *buf, size_t size, loff_t *l);
static ssize_t ow_write(struct file *filp, const char *buf, size_t size, loff_t *l);
static int ow_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);

static void do_ow_rddata(void);
static void do_ow_wrdata(void);
static void do_ow_wr1rd(void);
static void do_ow_wr0(void);
static void do_ow_rst(void);

static void do_interrupt_mode_test(void);
static void do_cpu_mode_test(void);

static struct file_operations ow_fops = 
{
	open:		ow_open,
	release:	ow_release,
	read:		ow_read,
	write:		ow_write,
	ioctl:		ow_ioctl,
};

static int ow_open(struct inode *inode, struct file *filp)
{
	try_module_get(THIS_MODULE);
 	return 0;
}

static int ow_release(struct inode *inode, struct file *filp)
{
	module_put(THIS_MODULE);
	return 0;	
}

static ssize_t ow_read(struct file *filp, char *buf, size_t size, loff_t *l)
{
	printk("OW: read is not implemented\n");
	return -1;
}

static ssize_t ow_write(struct file *filp, const char *buf, size_t size, loff_t *l)
{
	printk("ow: write is not implemented\n");
	return -1;
}

static int ow_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	switch (cmd) {

	default:
		printk("Not supported command: 0x%x\n", cmd);
		return -EINVAL;
		break;
	}
	return ret;
}

static void do_ow_rddata(void)
{
	__owi_clr_sts();
	__owi_set_rddata();
	__owi_enable_ow_ops();
}

static void do_ow_wrdata(void)
{
	__owi_clr_sts();
	__owi_set_wrdata();
	__owi_enable_ow_ops();
}

static void do_ow_wr1rd(void)
{
	__owi_clr_sts();
	__owi_set_wr1rd();
	__owi_enable_ow_ops();
}

static void do_ow_wr0(void)
{
	__owi_clr_sts();
	__owi_set_wr0();
	__owi_enable_ow_ops();
}

static void do_ow_rst(void)
{
	__owi_clr_sts();
	__owi_set_rst();
	__owi_enable_ow_ops();
}

static irqreturn_t ow_interrupt(int irq, void *dev_id)
{
	__owi_clr_sts();
	wake_up(&ow_wait_queue);

	return IRQ_HANDLED;
}

static void ow_intcm_read_rom(char *rom)
{
	int i; 
	
	__owi_select_regular_mode();
	REG_OWI_DIV = 23;
	__owi_clr_sts();
	__intc_unmask_irq(IRQ_OWI);
	__owi_enable_all_interrupts();

	do_ow_rst();
	sleep_on(&ow_wait_queue);

	REG_OWI_DAT = 0x33;
	do_ow_wrdata();
	sleep_on(&ow_wait_queue);
	
	for(i=0; i<8; i++){
		do_ow_rddata();
		sleep_on(&ow_wait_queue);
		rom[i] = REG_OWI_DAT;
	}	
	__intc_mask_irq(IRQ_OWI);
}

static void ow_intcm_search_rom(void)
{
	int i, j; 
	int normal, reverse;
#if 1
	unsigned char rom[8]={0x01, 0xf9, 0x35, 0x53, 0x11, 0x00, 0x00, 0x3e};
#else
	unsigned char rom[8]={0x01, 0xd8, 0x10, 0x02, 0x10, 0x00, 0x00, 0x22};
#endif
	__owi_select_regular_mode(); 	
	REG_OWI_DIV =  __cpm_get_extalclk()/1000000 - 1;
	__owi_clr_sts();
	__intc_unmask_irq(IRQ_OWI);
	__owi_enable_all_interrupts();
	
        /* reset */
	do_ow_rst();
	sleep_on(&ow_wait_queue);
	
	/* send search ROM command */
	REG_OWI_DAT = 0xf0;
	do_ow_wrdata();
	sleep_on(&ow_wait_queue);

	for( i=0; i<8; i++){
		for (j=0; j<8; j++){
			do_ow_wr1rd();
			sleep_on(&ow_wait_queue);
			normal = ( __owi_get_rdst() !=0);
			printk("normal: %d\n",normal);

			do_ow_wr1rd();
			sleep_on(&ow_wait_queue);
			reverse = ( __owi_get_rdst() !=0);
			printk("reverse: %d\n",reverse);

			if(normal ==1 && reverse ==1){
				printk("Search rom INTC mode: 11  NO device found\n");
				__intc_mask_irq(IRQ_OWI);
				return;
			}
#if 1
			if ( (rom[i]>>j) & 1 ){
				printk("write 1\n");
				do_ow_wr1rd();
				sleep_on(&ow_wait_queue);
			}
			else{
				printk("write 0\n");
				do_ow_wr0();
				sleep_on(&ow_wait_queue);
			}				
			
#else
			if(normal ==0 && reverse ==0){
				if (!((rom[i]>>j) & 1) ){
					printk("write 1\n");
					do_ow_wr1rd();
					sleep_on(&ow_wait_queue);
				}
				else{
					printk("write 0\n");
					do_ow_wr0();
					sleep_on(&ow_wait_queue);
				}			
			}else{
			
				if(normal ==0){
					printk("write 0\n");
					do_ow_wr0();
					sleep_on(&ow_wait_queue);
				}
				if(normal ==1){
					printk("write 1\n");
					do_ow_wr1rd();
					sleep_on(&ow_wait_queue);
				}
			}
#endif

		}
		printk("\n\n");
	}

	printk("\nSearch rom INTC mode: device found SUCCESSFULLY\n");
	__intc_mask_irq(IRQ_OWI);
       
}

static void ow_cpum_read_rom(char *rom)
{
	int i; 
	
	__owi_select_regular_mode(); 
	REG_OWI_DIV =  __cpm_get_extalclk()/1000000 - 1;
	__owi_clr_sts();
	__owi_disable_all_interrupts();

	do_ow_rst();
	__owi_wait_ops_rdy();

	if(!__owi_get_sts_pst())
		printk("read rom no device found\n");
	 
	REG_OWI_DAT = 0x33;
	do_ow_wrdata();
	__owi_wait_ops_rdy();
	
	for(i=0; i<8; i++){
		do_ow_rddata();
		__owi_wait_ops_rdy();
		rom[i] = REG_OWI_DAT;
	}	
}


static void ow_comm_bit(unsigned comm)
{
	int i;
	for(i=0; i<8; i++){
		if ( comm & (1<<i) ) 
			do_ow_wr1rd();
		else
			do_ow_wr0();
		while(!__owi_get_sts_bit_rdy());
	}
}

static void ow_cpum_search_rom(void)
{
	int i, j; 
	int normal, reverse;
#if 1
	unsigned char rom[8]={0x01, 0xf9, 0x35, 0x53, 0x11, 0x00, 0x00, 0x3e};
#else
	unsigned char rom[8]={0x01, 0xd8, 0x10, 0x02, 0x10, 0x00, 0x00, 0x22};
#endif

	__owi_select_regular_mode(); 
	REG_OWI_DIV = 23;
	__owi_clr_sts();
	__owi_disable_all_interrupts();
	
	do_ow_rst();
	while(!__owi_get_sts_pst_rdy()) ;
	
	if(!__owi_get_sts_pst())
		printk("search rom: no device found\n");
#if 1	
	REG_OWI_DAT = 0xf0;
	do_ow_wrdata();
	while(! __owi_get_sts_byte_rdy()) ;
#else
	ow_comm_bit(0xf0);
#endif

	for( i=0; i<8; i++){
		for (j=0; j<8; j++){
			do_ow_wr1rd();
			while(!__owi_get_sts_bit_rdy()) ;
			normal = ( __owi_get_rdst() !=0);
			printk("normal: %d\n",normal);

			do_ow_wr1rd();
			while(!__owi_get_sts_bit_rdy()) ;
			reverse = ( __owi_get_rdst() !=0);
			printk("reverse: %d\n",reverse);

			if(normal ==1 && reverse ==1){
				printk("Search rom CPU mode: 11  NO device found\n");
				return;
			}

#if 1
			if ( (rom[i]>>j) & 1 ){
				printk("write 1\n");
				do_ow_wr1rd();
				while(!__owi_get_sts_bit_rdy()) ;
			}
			else{
				printk("write 0\n");
				do_ow_wr0();
				while(!__owi_get_sts_bit_rdy()) ;
			}				
			
#else
			if(normal ==0 && reverse ==0){
				if (!((rom[i]>>j) & 1) ){
					printk("write 1\n");
					do_ow_wr1rd();
					while(!__owi_get_sts_bit_rdy()) ;
				}
				else{
					printk("write 0\n");
					do_ow_wr0();
					while(!__owi_get_sts_bit_rdy()) ;
				}			
			}else{
			
				if(normal ==0){
					printk("write 0\n");
					do_ow_wr0();
					while(!__owi_get_sts_bit_rdy()) ;
				}
				if(normal ==1){
					printk("write 1\n");
					do_ow_wr1rd();
					while(!__owi_get_sts_bit_rdy()) ;
				}
			}
#endif

		}
		printk("\n\n");
	}
	printk("\nSearch rom CPU mode: device found SUCCESSFULLY\n");
}
	
static void do_interrupt_mode_test(void)
{
	int ret, i;
	unsigned char rom[8];
	
	/* interrupt mode */
	ret = request_irq(IRQ_OWI, ow_interrupt, IRQF_DISABLED, 
			  "JZ_OWI", NULL); 
	if(ret)
		printk("failed irq \n");

#if OW_INTC_READ_ROM
	ow_intcm_read_rom(rom);
	printk("\n\nAfter intc mode read ROM ops: \n");
	printk("ROM: ");
	for(i=0; i<8; i++)
		printk("0x%02x,",rom[i]);
#endif

#if OW_INTC_SEARCH_ROM
	ow_intcm_search_rom();
#endif

}

static void do_cpu_mode_test(void)
{
	
#if OW_CPU_READ_ROM
	int i;
	unsigned char rom[8];

	ow_cpum_read_rom(rom);
	printk("\n\nAfter CPU mode read ROM ops: \n");
	printk("ROM: ");
	for(i=0; i<8; i++)
		printk("0x%02x,",rom[i]);
#endif

#if OW_CPU_SEARCH_ROM
	ow_cpum_search_rom();	
#endif
}

/*
 * Module init and exit
 */
static int __init ow_init(void)
{
	int ret;

	ret = jz_register_chrdev(OW_MINOR, "ow", &ow_fops, NULL);
	if (ret < 0) {
		return ret;
	}
	__gpio_as_func1(153);

	REG_OWI_CFG=0;
	REG_OWI_CTL=0;
	REG_OWI_STS=0;
	REG_OWI_DAT=0;
	REG_OWI_DIV=0;

	do_interrupt_mode_test();
	do_cpu_mode_test();
	
	printk(JZ_SOC_NAME": OW driver registered.\n");

	return 0;
}

static void __exit ow_exit(void)
{
	free_irq(IRQ_OWI, NULL);
	jz_unregister_chrdev(OW_MINOR, "ow");
}

module_init(ow_init);
module_exit(ow_exit);

MODULE_AUTHOR("Yurong Tan<yrtan@ingenic.cn>");
MODULE_DESCRIPTION("One Wire Bus test Driver");
MODULE_LICENSE("GPL");
