/*
 * linux/drivers/video/jz_lcd.c -- Ingenic Jz LCD frame buffer device
 *
 * Copyright (C) 2005-2008, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * --------------------------------
 * NOTE:
 * This LCD driver support TFT16 TFT32 LCD, not support STN and Special TFT LCD
 * now.
 * 	It seems not necessory to support STN and Special TFT.
 * 	If it's necessary, update this driver in the future.
 * 	<Wolfgang Wang, Jun 10 2008>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/jzsoc.h>

#include "jz_aosd.h"

MODULE_DESCRIPTION("Jz ALPHA OSD driver");
MODULE_AUTHOR("lltang, <lltang@ingenic.cn>");
MODULE_LICENSE("GPL");

#define D(fmt, args...) \
//	printk(KERN_ERR "%s(): "fmt"\n", __func__, ##args)

#define E(fmt, args...) \
	printk(KERN_ERR "%s(): "fmt"\n", __func__, ##args)

extern struct jz_aosd_info *aosd_info;
extern wait_queue_head_t compress_wq;

int irq_flag = 0;

/* The following routine is only for test */

void print_aosd_registers(void)	/* debug */
{
	/* LCD Controller Resgisters */
	printk("REG_AOSD_ADDR0:\t0x%08x\n", REG_AOSD_ADDR0);
	printk("REG_AOSD_ADDR1:\t0x%08x\n", REG_AOSD_ADDR1);
	printk("REG_AOSD_ADDR2:\t0x%08x\n", REG_AOSD_ADDR2);
	printk("REG_AOSD_ADDR3:\t0x%08x\n", REG_AOSD_ADDR3);
	printk("REG_AOSD_WADDR:\t0x%08x\n", REG_AOSD_WADDR);
	printk("REG_AOSD_ADDRLEN:\t0x%08x\n", REG_AOSD_ADDRLEN);
	printk("REG_AOSD_ALPHA_VALUE:\t0x%08x\n", REG_AOSD_ALPHA_VALUE);
	printk("REG_AOSD_CTRL:\t0x%08x\n",REG_AOSD_CTRL);
	printk("REG_AOSD_INT:\t0x%08x\n", REG_AOSD_INT);
	printk("REG_AOSD_CLK_GATE:\t0x%08x\n", REG_AOSD_CLK_GATE);	

	printk("REG_COMPRESS_OFFSIZE:\t0x%08x\n", REG_COMPRESS_OFFSIZE);
	printk("REG_COMPRESS_FRAME_SIZE:\t0x%08x\n", REG_COMPRESS_FRAME_SIZE);
	printk("REG_COMPRESS_CTRL:\t0x%08x\n", REG_COMPRESS_CTRL);
	printk("REG_COMPRESS_RATIO:\t0x%08x\n", REG_COMPRESS_RATIO);
	printk("REG_COMPRESS_OFFSET:\t0x%08x\n", REG_COMPRESS_OFFSET);
	printk("REG_COMPRESS_RESULT:\t0x%08x\n", REG_COMPRESS_RESULT);

	printk("==================================\n");

}

void jz_start_alpha_blending(void)
{
	while(!(REG_AOSD_CTRL & AOSD_CTRL_FRM_END)) ;
	__osd_alpha_start();
}

void jz_start_compress(void)
{
	while(!(REG_COMPRESS_CTRL & COMPRESS_CTRL_COMP_END));
	aosd_info->compress_done = 0;
	__compress_start();
}

void jz_compress_set_mode(struct jz_aosd_info *info)
{
	unsigned int reg_ctrl;
	unsigned int width, height, src_stride, dst_stride;
	unsigned int ALIGNED;
	unsigned int with_alpha;

//	printk("<< aosd: addr0:%x, waddr:%x, width:%d, length:%d, with_alpha:%d >>\n", info->addr0, info->waddr, info->width, info->height, info->with_alpha);
	reg_ctrl = 0;
	width = info->width;
	height= info->height ;
	src_stride= info->src_stride ;
	dst_stride= info->dst_stride ;

	//with_alpha = info->with_alpha;

	/* if bpp=16, width/=2) */
	if (info->bpp == 16) {
		with_alpha = 1;
		width /= 2;
	}
	else if (info->bpp == 24) {
		with_alpha = 0;
	}
	else {						/* 32bpp */
		with_alpha = 1;
	}

	/*SET SCR AND DES ADDR*/
	REG_AOSD_ADDR0 = info->addr0;
	REG_COMPRESS_DES_ADDR = info->waddr;

	/*SET DST OFFSIZE */
	REG_COMPRESS_OFFSIZE = dst_stride;
	/*SET SCR OFFSET*/
	REG_COMPRESS_OFFSET = src_stride;
	/*SET SIZE*/
	// frame's actual size, no need to align
	REG_COMPRESS_FRAME_SIZE = (width & 0xffff) | ((height & 0xffff) << 16);		

#if defined(CONFIG_SOC_JZ4760B)
	if(info->aligned_64)
		reg_ctrl |= COMPRESS_CTRL_ALIGNED_64_WORD;
	else
		reg_ctrl &= ~(COMPRESS_CTRL_ALIGNED_64_WORD);
#endif

	if(with_alpha)
		reg_ctrl &= ~(COMPRESS_CTRL_WITHOUT_ALPHA);
	else
		reg_ctrl |= COMPRESS_CTRL_WITHOUT_ALPHA;

	reg_ctrl |= COMPRESS_CTRL_INT_MASK | COMPRESS_CTRL_COMP_ENABLE;
	/* SET CTRL*/	
	REG_COMPRESS_CTRL = reg_ctrl;
}

void jz_aosd_set_mode(struct jz_aosd_info *info)
{

	REG_AOSD_WADDR = info->waddr;
	REG_AOSD_ADDRLEN = info->width * info->height; 
	REG_AOSD_ALPHA_VALUE = info->alpha_value;

	if(info->frmlv == 4){
		REG_AOSD_CTRL = (AOSD_ALPHA_ENABLE | AOSD_CTRL_INT_MAKS |  AOSD_CTRL_FRMLV_4);
	}else if(info->frmlv == 3){
		REG_AOSD_CTRL = (AOSD_ALPHA_ENABLE | AOSD_CTRL_INT_MAKS |  AOSD_CTRL_FRMLV_3);
	}else if(info->frmlv == 2){
		REG_AOSD_CTRL = (AOSD_ALPHA_ENABLE | AOSD_CTRL_INT_MAKS |  AOSD_CTRL_FRMLV_2);
	}else{
		printk("frmlv default value is 0b01,now the frmlv < 2\n");
		return;
	}

	REG_AOSD_CTRL &= ~AOSD_CTRL_CHANNEL_LEVEL_MASK;
	REG_AOSD_CTRL |= (info->order << AOSD_CTRL_CHANNEL_LEVEL_BIT);

	REG_AOSD_CTRL &= ~AOSD_CTRL_ALPHA_MODE_MASK;
	REG_AOSD_CTRL |= (info->alpha_mode << AOSD_CTRL_ALPHA_MODE_BIT) ;

	REG_AOSD_CTRL &= ~AOSD_CTRL_FORMAT_MODE_MASK;
	if(info->bpp == 15)
		REG_AOSD_CTRL |= AOSD_CTRL_RGB555_FORMAT_MODE;
	else if(info->bpp == 16)
		REG_AOSD_CTRL |= AOSD_CTRL_RGB565_FORMAT_MODE;
	else if(info->bpp == 24)
		REG_AOSD_CTRL |= AOSD_CTRL_RGB8888_FORMAT_MODE;
	else
		printk("Sorry , we only support RGB555 RGB565 RGB8888!!!!!!\n");

}

static irqreturn_t jz_interrupt_handler(int irq, void *dev_id)
{
	unsigned int state;
	int cnt = 0;
	
	state = REG_AOSD_INT;
	D("In the lcd interrupt handler, state=0x%x\n", state);

	if (state & AOSD_INT_COMPRESS_END){
		D("compress end\n");
		aosd_info->compress_done = 1;
		REG_AOSD_ADDR0 = aosd_info->addr0; 
		REG_AOSD_INT = state & AOSD_INT_COMPRESS_END;
	}

	if (state & AOSD_INT_AOSD_END) {
		D("alpha blending end\n");

		REG_AOSD_INT = state & AOSD_INT_AOSD_END;
		REG_AOSD_ADDR0 = aosd_info->addr0; 
		REG_AOSD_ADDR1 = aosd_info->addr1; 
		REG_AOSD_ADDR2 = aosd_info->addr2; 
		REG_AOSD_ADDR3 = aosd_info->addr3; 
		aosd_info->compress_done = 1;
	}

	wake_up(&compress_wq);

	return IRQ_HANDLED;
}

int jz_aosd_init(void)
{
	int ret,irq;

#if defined(CONFIG_JZ_AOSD) && defined(CONFIG_JZ_COMPRESS)
	printk("you can not use aosd and compress at the  same time!!!!\n");
	return -1;
#endif
	init_waitqueue_head(&compress_wq);

	/* aosd and compress use the same IRQ number */
	if (request_irq(IRQ_AOSD, jz_interrupt_handler, IRQF_DISABLED,"oasd_compress", 0)) {
		printk("Faield to request ALPHA OSD IRQ.\n");
		ret = -EBUSY;
		goto failed;
	}

#if defined(CONFIG_JZ_COMPRESS)
	REG_COMPRESS_CTRL = (COMPRESS_CTRL_INT_MASK | COMPRESS_CTRL_COMP_ENABLE); /* enable compress and enable finished interrupt */
#elif defined(CONFIG_JZ_AOSD)
	REG_AOSD_CTRL = (AOSD_ALPHA_ENABLE | AOSD_CTRL_INT_MAKS);
#endif
	REG_AOSD_CLK_GATE = AOSD_CLK_GATE_EN;

	return 0;

failed:
	free_irq(IRQ_AOSD,0);
	return ret;
}

