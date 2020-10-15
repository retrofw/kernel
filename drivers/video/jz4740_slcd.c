/*
 * linux/drivers/video/jzslcd.c -- Ingenic On-Chip Smart LCD frame buffer device
 *
 * Copyright (C) 2005-2007, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
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
#include <linux/pm_legacy.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/jzsoc.h>

#include "console/fbcon.h"

#include "jz4740_slcd.h"

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define dprintk(x...)	printk(x)
#else
#define dprintk(x...)
#endif

#define print_err(f, arg...) printk(KERN_ERR DRIVER_NAME ": " f "\n", ## arg)
#define print_warn(f, arg...) printk(KERN_WARNING DRIVER_NAME ": " f "\n", ## arg)
#define print_info(f, arg...) printk(KERN_INFO DRIVER_NAME ": " f "\n", ## arg)
#ifdef DEBUG
#define print_dbg(f, arg...) printk("dbg::" __FILE__ ",LINE(%d): " f "\n", __LINE__, ## arg)
#else
#define print_dbg(f, arg...) do {} while (0)
#endif

static jz_dma_desc slcd_palette_desc __attribute__ ((aligned (16)));
static jz_dma_desc slcd_frame_desc __attribute__ ((aligned (16)));

static int dma_chan;
static dma_addr_t slcd_frame_desc_phys_addr, slcd_palette_desc_phys_addr;

static unsigned char non_link_desp = 0;
static unsigned char is_set_reg = 0;
struct lcd_cfb_info {
	struct fb_info		fb;
	struct display_switch	*dispsw;
	signed int		currcon;
	int			func_use_count;

	struct {
		u16 red, green, blue;
	} palette[NR_PALETTE];
#ifdef CONFIG_PM
	struct pm_dev *pm;
#endif
};

struct slcd_reg_info {
	unsigned int cmd;
	unsigned int data;
};
static struct slcd_reg_info reg_buf;
static struct lcd_cfb_info *jzslcd_info;

struct jzfb_info {
	unsigned int cfg;	/* panel mode and pin usage etc. */
	unsigned int w;
	unsigned int h;
	unsigned int bpp;	/* bit per pixel */
	unsigned int bus;
	unsigned int pclk;	/* pixel clk */
	
};

static struct jzfb_info jzfb = {
#ifdef CONFIG_JZ_SLCD_LGDP4551
	SLCD_CFG_CS_ACTIVE_LOW | SLCD_CFG_RS_CMD_LOW | SLCD_CFG_TYPE_PARALLEL,
	400, 240, 16, 8, 16000000 	/*16 bpp, 8 bus*/
//	240, 400, 18, 8, 16000000 	/*18 bpp, 8 bus*/
//	400, 240, 18, 8, 16000000 	/*18 bpp, 8 bus*/
#endif

#ifdef CONFIG_JZ_SLCD_SPFD5420A
	SLCD_CFG_CS_ACTIVE_LOW | SLCD_CFG_RS_CMD_LOW | SLCD_CFG_TYPE_PARALLEL,
	400, 240, 18, 18, 16000000 	/*18 bpp, 18 bus*/
#endif
};


static volatile unsigned char *slcd_palette;
static volatile unsigned char *slcd_frame;

//extern struct display fb_display[MAX_NR_CONSOLES];
static irqreturn_t slcd_dma_irq(int irq, void *dev_id);


static void Mcupanel_RegSet(UINT32 cmd, UINT32 data)
{
	switch (jzfb.bus) {
	case 8:
		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0xff00) >> 8);
		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0xff) >> 0);
		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
		REG_SLCD_DATA = SLCD_DATA_RS_DATA | (data&0xffff);
		break;
	case 9:
		data = ((data & 0xff) << 1) | ((data & 0xff00) << 2);
		data = ((data << 6) & 0xfc0000) | ((data << 4) & 0xfc00) | ((data << 2) & 0xfc);
		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0xff00) >> 8);
		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0xff) >> 0);
		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
		REG_SLCD_DATA = SLCD_DATA_RS_DATA | data;
		break;
	case 16:
		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | (cmd&0xffff);
		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
		REG_SLCD_DATA = SLCD_DATA_RS_DATA | (data&0xffff);
		break;
	case 18:
		cmd = ((cmd & 0xff) << 1) | ((cmd & 0xff00) << 2); 	
 		data = ((data & 0xff) << 1) | ((data & 0xff00) << 2);
 		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | cmd;
		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
		REG_SLCD_DATA = SLCD_DATA_RS_DATA | ((data<<6)&0xfc0000)|((data<<4)&0xfc00) | ((data<<2)&0xfc);
		break;
	default:
		printk("Don't support %d bit Bus\n", jzfb.bus );
		break;
	}
}

/* Sent a command withou data */
static void Mcupanel_Command(UINT32 cmd) {
	switch (jzfb.bus) {
	case 8:
	case 9:
		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0xff00) >> 8);
		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0xff) >> 0);
		break;
	case 16:
		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | (cmd&0xffff);
		break;
	case 18:
		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0xff00) << 2) | ((cmd&0xff) << 1);
		break;
	default:
		printk("Don't support %d bit Bus\n", jzfb.bus );
		break;
	}
}

/* Set the start address of screen, for example (0, 0) */
#ifdef CONFIG_JZ_SLCD_LGDP4551
static void Mcupanel_SetAddr(UINT16 x, UINT16 y)
{
	Mcupanel_RegSet(0x20,x) ;
	udelay(1);
	Mcupanel_RegSet(0x21,y) ;
	udelay(1);
	Mcupanel_Command(0x22);	

}
#endif
#ifdef CONFIG_JZ_SLCD_SPFD5420A
void Mcupanel_SetAddr(u32 x, u32 y) //u32
{
	Mcupanel_RegSet(0x200,x) ;
	udelay(1);
	Mcupanel_RegSet(0x201,y) ;
	udelay(1);
	Mcupanel_Command(0x202);

}

#endif

static inline u_int chan_to_field(u_int chan, struct fb_bitfield *bf)
{
        chan &= 0xffff;
        chan >>= 16 - bf->length;
        return chan << bf->offset;
}

static int jzfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			  u_int transp, struct fb_info *info)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	unsigned short *ptr, ctmp;

	print_dbg("regno:%d,RGBt:(%d,%d,%d,%d)\t", regno, red, green, blue, transp);
	if (regno >= NR_PALETTE)
		return 1;

	cfb->palette[regno].red		= red ;
	cfb->palette[regno].green	= green;
	cfb->palette[regno].blue	= blue;
	if (cfb->fb.var.bits_per_pixel <= 16) {
		red	>>= 8;
		green	>>= 8;
		blue	>>= 8;

		red	&= 0xff;
		green	&= 0xff;
		blue	&= 0xff;
	}
	switch (cfb->fb.var.bits_per_pixel) {
	case 1:
	case 2:
	case 4:
	case 8:
		/* RGB 565 */
		if (((red >> 3) == 0) && ((red >> 2) != 0))
			red = 1 << 3;
		if (((blue >> 3) == 0) && ((blue >> 2) != 0))
			blue = 1 << 3;
		ctmp = ((red >> 3) << 11) 
			| ((green >> 2) << 5) | (blue >> 3);

		ptr = (unsigned short *)slcd_palette;
		ptr = (unsigned short *)(((u32)ptr)|0xa0000000);
		ptr[regno] = ctmp;

		break;
		
	case 15:
		if (regno < 16)
			((u32 *)cfb->fb.pseudo_palette)[regno] =
				((red >> 3) << 10) | 
				((green >> 3) << 5) |
				(blue >> 3);
		break;
	case 16:
		if (regno < 16) {
			((u32 *)cfb->fb.pseudo_palette)[regno] =
				((red >> 3) << 11) | 
				((green >> 2) << 5) |
				(blue >> 3); 
		}
		break;
	case 18:
	case 24:
	case 32:
		if (regno < 16)
			((u32 *)cfb->fb.pseudo_palette)[regno] =
				(red << 16) | 
				(green << 8) |
				(blue << 0); 

/*		if (regno < 16) {
			unsigned val;
                        val  = chan_to_field(red, &cfb->fb.var.red);
                        val |= chan_to_field(green, &cfb->fb.var.green);
                        val |= chan_to_field(blue, &cfb->fb.var.blue);
			((u32 *)cfb->fb.pseudo_palette)[regno] = val;
		}
*/

		break;
	}
	return 0;
}

static int jzfb_ioctl (struct fb_info *info, unsigned int cmd, unsigned long arg )
{
	int ret = 0;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case FBIOSETBACKLIGHT:
		__slcd_set_backlight_level(arg);	/* We support 8 levels here. */
		break;
	case FBIODISPON:
		__slcd_display_on();
		break;
	case FBIODISPOFF:
		__slcd_display_off();
		break;
	case FBIO_REFRESH_ALWAYS:
		dprintk("slcd_frame_desc.dcmd = 0x%08x\n", slcd_frame_desc.dcmd);
		if (slcd_frame_desc.dcmd & DMAC_DCMD_LINK)
			printk("The Smart LCD refreshes automatically. Option is omitted!\n");
		else {
			dprintk("OPEN DMAC_DCMD_LINK \n");
			slcd_frame_desc.dcmd &= ~DMAC_DCMD_TIE;
			slcd_frame_desc.dcmd |= DMAC_DCMD_LINK;
			dma_cache_wback((unsigned long)(&slcd_frame_desc), 16);
			REG_DMAC_DCMD(dma_chan) &= ~DMAC_DCMD_TIE;
			__dmac_channel_set_doorbell(dma_chan);
		}
		break;
	case FBIO_REFRESH_EVENTS:
		dprintk("slcd_frame_desc.dcmd = 0x%08x\n", slcd_frame_desc.dcmd);
		if (!(slcd_frame_desc.dcmd & DMAC_DCMD_LINK))
			printk("The Smart LCD is refreshed by envents. Option is omitted!\n");
		else {
			non_link_desp = 1;
			REG_DMAC_DCMD(dma_chan)	|= DMAC_DCMD_TIE;
			REG_DMAC_DCMD(dma_chan) &= ~DMAC_DCMD_LINK;
		}
		break;
	case FBIO_DO_REFRESH:

		dprintk("slcd_frame_desc.dcmd = 0x%08x\n", slcd_frame_desc.dcmd);
		if (slcd_frame_desc.dcmd & DMAC_DCMD_LINK)
			printk("The Smart LCD can refresh automatically. Option is omitted!\n");
		else {
			while (REG_SLCD_STATE & SLCD_STATE_BUSY);
			__dmac_channel_set_doorbell(dma_chan);
		}		
		break;
	case FBIO_SET_REG:
		if (copy_from_user(&reg_buf, argp, sizeof(reg_buf)))
			return -EFAULT;
		is_set_reg = 1;
		REG_DMAC_DCMD(dma_chan)	|= DMAC_DCMD_TIE;
		REG_DMAC_DCMD(dma_chan) &= ~DMAC_DCMD_LINK;
		break;
	default:
		break;
	}

	return ret;
}

/* Use mmap /dev/fb can only get a non-cacheable Virtual Address. */
static int jzfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	unsigned long start;
	unsigned long off;
	u32 len;

	off = vma->vm_pgoff << PAGE_SHIFT;
	//fb->fb_get_fix(&fix, PROC_CONSOLE(info), info);

	/* frame buffer memory */
	start = cfb->fb.fix.smem_start;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + cfb->fb.fix.smem_len);
	start &= PAGE_MASK;

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);	/* Uncacheable */

#if 1
 	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
// 	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;		/* Uncacheable */
	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;	/* Write-Through */
#endif

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}

/* checks var and eventually tweaks it to something supported,
 * DO NOT MODIFY PAR */
static int jzfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	print_dbg("jzfb_check_var");
	return 0;
}


/* 
 * set the video mode according to info->var
 */
static int jzfb_set_par(struct fb_info *info)
{
//	print_dbg("jzfb_set_par");
	printk("jzfb_set_par");
	return 0;
}


/*
 * (Un)Blank the display.
 * Fix me: should we use VESA value?
 */
static int jzfb_blank(int blank_mode, struct fb_info *info)
{

	dprintk("fb_blank %d %p", blank_mode, info);

	switch (blank_mode) {

	case FB_BLANK_UNBLANK:
		/* Turn on panel */
		break;

	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		/* Turn off panel */
		break;
	default:
		break;

	}
	return 0;
}

/* 
 * pan display
 */
static int jzfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	int dy;

	if (!var || !cfb) {
		return -EINVAL;
	}

	if (var->xoffset - cfb->fb.var.xoffset) {
		/* No support for X panning for now! */
		return -EINVAL;
	}

	dy = var->yoffset - cfb->fb.var.yoffset;
	print_dbg("var.yoffset: %d", dy);
	if (dy) {

		print_dbg("Panning screen of %d lines", dy);
//		slcd_frame_desc->databuf += (cfb->fb.fix.line_length * dy);
//		slcd_frame_desc->dsadr += (cfb->fb.fix.line_length * dy);
		/* TODO: Wait for current frame to finished */
	}

	return 0;
}


/* use default function cfb_fillrect, cfb_copyarea, cfb_imageblit */
static struct fb_ops jzfb_ops = {
	.owner			= THIS_MODULE,
	.fb_setcolreg		= jzfb_setcolreg,
	.fb_check_var 		= jzfb_check_var,
	.fb_set_par 		= jzfb_set_par,
	.fb_blank		= jzfb_blank,
	.fb_pan_display		= jzfb_pan_display,
	.fb_fillrect		= cfb_fillrect,
	.fb_copyarea		= cfb_copyarea,
	.fb_imageblit		= cfb_imageblit,
	.fb_mmap		= jzfb_mmap,
	.fb_ioctl		= jzfb_ioctl,
};

static int jzfb_set_var(struct fb_var_screeninfo *var, int con,
			struct fb_info *info)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	//struct display *display;
	int chgvar = 0;

	var->height	            = jzfb.h ;
	var->width	            = jzfb.w ;
	var->bits_per_pixel	    = jzfb.bpp;

	var->vmode                  = FB_VMODE_NONINTERLACED;
	var->activate               = cfb->fb.var.activate;
	var->xres                   = var->width;
	var->yres                   = var->height;
	var->xres_virtual           = var->width;
	var->yres_virtual           = var->height;
	var->xoffset                = 0;
	var->yoffset                = 0;
	var->pixclock               = 0;
	var->left_margin            = 0;
	var->right_margin           = 0;
	var->upper_margin           = 0;
	var->lower_margin           = 0;
	var->hsync_len              = 0;
	var->vsync_len              = 0;
	var->sync                   = 0;
	var->activate              &= ~FB_ACTIVATE_TEST;
    
	/*
	 * CONUPDATE and SMOOTH_XPAN are equal.  However,
	 * SMOOTH_XPAN is only used internally by fbcon.
	 */
	if (var->vmode & FB_VMODE_CONUPDATE) {
		var->vmode |= FB_VMODE_YWRAP;
		var->xoffset = cfb->fb.var.xoffset;
		var->yoffset = cfb->fb.var.yoffset;
	}

	if (var->activate & FB_ACTIVATE_TEST)
		return 0;

	if ((var->activate & FB_ACTIVATE_MASK) != FB_ACTIVATE_NOW)
		return -EINVAL;

	if (cfb->fb.var.xres != var->xres)
		chgvar = 1;
	if (cfb->fb.var.yres != var->yres)
		chgvar = 1;
	if (cfb->fb.var.xres_virtual != var->xres_virtual)
		chgvar = 1;
	if (cfb->fb.var.yres_virtual != var->yres_virtual)
		chgvar = 1;
	if (cfb->fb.var.bits_per_pixel != var->bits_per_pixel)
		chgvar = 1;

	//display = fb_display + con;

	var->red.msb_right	= 0;
	var->green.msb_right	= 0;
	var->blue.msb_right	= 0;

	switch(var->bits_per_pixel){
	case 1:	/* Mono */
		cfb->fb.fix.visual	= FB_VISUAL_MONO01;
		cfb->fb.fix.line_length	= (var->xres * var->bits_per_pixel) / 8;
		break;
	case 2:	/* Mono */
		var->red.offset		= 0;
		var->red.length		= 2;
		var->green.offset	= 0;
		var->green.length	= 2;
		var->blue.offset	= 0;
		var->blue.length	= 2;

		cfb->fb.fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		cfb->fb.fix.line_length	= (var->xres * var->bits_per_pixel) / 8;
		break;
	case 4:	/* PSEUDOCOLOUR*/
		var->red.offset		= 0;
		var->red.length		= 4;
		var->green.offset	= 0;
		var->green.length	= 4;
		var->blue.offset	= 0;
		var->blue.length	= 4;

		cfb->fb.fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		cfb->fb.fix.line_length	= var->xres / 2;
		break;
	case 8:	/* PSEUDOCOLOUR, 256 */
		var->red.offset		= 0;
		var->red.length		= 8;
		var->green.offset	= 0;
		var->green.length	= 8;
		var->blue.offset	= 0;
		var->blue.length	= 8;

		cfb->fb.fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		cfb->fb.fix.line_length	= var->xres ;
		break;
	case 15: /* DIRECTCOLOUR, 32k */
		var->bits_per_pixel	= 15;
		var->red.offset		= 10;
		var->red.length		= 5;
		var->green.offset	= 5;
		var->green.length	= 5;
		var->blue.offset	= 0;
		var->blue.length	= 5;

		cfb->fb.fix.visual	= FB_VISUAL_DIRECTCOLOR;
		cfb->fb.fix.line_length	= var->xres_virtual * 2;
		break;
	case 16: /* DIRECTCOLOUR, 64k */
		var->bits_per_pixel	= 16;
		var->red.offset		= 11;
		var->red.length		= 5;
		var->green.offset	= 5;
		var->green.length	= 6;
		var->blue.offset	= 0;
		var->blue.length	= 5;

		cfb->fb.fix.visual	= FB_VISUAL_TRUECOLOR;
		cfb->fb.fix.line_length	= var->xres_virtual * 2;
		break;
	case 18:
	case 24:
	case 32:
		/* DIRECTCOLOUR, 256 */
		var->bits_per_pixel	= 32;

		var->red.offset		= 16;
		var->red.length		= 8;
		var->green.offset	= 8;
		var->green.length	= 8;
		var->blue.offset	= 0;
		var->blue.length	= 8;
		var->transp.offset  	= 24;
		var->transp.length 	= 8;

		cfb->fb.fix.visual	= FB_VISUAL_TRUECOLOR;
		cfb->fb.fix.line_length	= var->xres_virtual * 4;
		break;

	default: /* in theory this should never happen */
		printk(KERN_WARNING "%s: don't support for %dbpp\n",
		       cfb->fb.fix.id, var->bits_per_pixel);
		break;
	}

	cfb->fb.var = *var;
	cfb->fb.var.activate &= ~FB_ACTIVATE_ALL;

	/*
	 * If we are setting all the virtual consoles, also set the
	 * defaults used to create new consoles.
	 */
	fb_set_cmap(&cfb->fb.cmap, &cfb->fb);
	dprintk("jzfb_set_var: after fb_set_cmap...\n");

	return 0;
}

static struct lcd_cfb_info * jzfb_alloc_fb_info(void)
{
 	struct lcd_cfb_info *cfb;

	cfb = kmalloc(sizeof(struct lcd_cfb_info) + sizeof(u32) * 16, GFP_KERNEL);

	if (!cfb)
		return NULL;

	jzslcd_info = cfb;

	memset(cfb, 0, sizeof(struct lcd_cfb_info) );

	cfb->currcon		= -1;


	strcpy(cfb->fb.fix.id, "jz-slcd");
	cfb->fb.fix.type	= FB_TYPE_PACKED_PIXELS;
	cfb->fb.fix.type_aux	= 0;
	cfb->fb.fix.xpanstep	= 1;
	cfb->fb.fix.ypanstep	= 1;
	cfb->fb.fix.ywrapstep	= 0;
	cfb->fb.fix.accel	= FB_ACCEL_NONE;

	cfb->fb.var.nonstd	= 0;
	cfb->fb.var.activate	= FB_ACTIVATE_NOW;
	cfb->fb.var.height	= -1;
	cfb->fb.var.width	= -1;
	cfb->fb.var.accel_flags	= FB_ACCELF_TEXT;

	cfb->fb.fbops		= &jzfb_ops;
	cfb->fb.flags		= FBINFO_FLAG_DEFAULT;

	cfb->fb.pseudo_palette	= (void *)(cfb + 1);

	switch (jzfb.bpp) {
	case 1:
		fb_alloc_cmap(&cfb->fb.cmap, 4, 0);
		break;
	case 2:
		fb_alloc_cmap(&cfb->fb.cmap, 8, 0);
		break;
	case 4:
		fb_alloc_cmap(&cfb->fb.cmap, 32, 0);
		break;
	case 8:

	default:
		fb_alloc_cmap(&cfb->fb.cmap, 256, 0);
		break;
	}
	dprintk("fb_alloc_cmap,fb.cmap.len:%d....\n", cfb->fb.cmap.len);

	return cfb;
}

/*
 * Map screen memory
 */
static int jzfb_map_smem(struct lcd_cfb_info *cfb)
{
	struct page * map = NULL;
	unsigned char *tmp;
	unsigned int page_shift, needroom, t;

	t = jzfb.bpp;
	if (jzfb.bpp == 15)
		t = 16;
	if (jzfb.bpp == 18 || jzfb.bpp == 24)
		t = 32;
	needroom = ((jzfb.w * t + 7) >> 3) * jzfb.h;

	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;

	slcd_palette = (unsigned char *)__get_free_pages(GFP_KERNEL, 0);
	slcd_frame = (unsigned char *)__get_free_pages(GFP_KERNEL, page_shift);
	if ((!slcd_palette) || (!slcd_frame))
		return -ENOMEM;

	memset((void *)slcd_palette, 0, PAGE_SIZE);
	memset((void *)slcd_frame, 0, PAGE_SIZE << page_shift);

	map = virt_to_page(slcd_palette);
	set_bit(PG_reserved, &map->flags);

	for (tmp=(unsigned char *)slcd_frame;
	     tmp < slcd_frame + (PAGE_SIZE << page_shift);
	     tmp += PAGE_SIZE) {
		map = virt_to_page(tmp);
		set_bit(PG_reserved, &map->flags);
	}

	cfb->fb.fix.smem_start = virt_to_phys((void *)slcd_frame);

	cfb->fb.fix.smem_len = (PAGE_SIZE << page_shift);

	cfb->fb.screen_base =
		(unsigned char *)(((unsigned int)slcd_frame & 0x1fffffff) | 0xa0000000);

	if (!cfb->fb.screen_base) {
		printk("%s: unable to map screen memory\n", cfb->fb.fix.id);
		return -ENOMEM;
	}

	return 0;
}

static void jzfb_free_fb_info(struct lcd_cfb_info *cfb)
{
	if (cfb) {
		fb_alloc_cmap(&cfb->fb.cmap, 0, 0);
		kfree(cfb);
	}
}

static void jzfb_unmap_smem(struct lcd_cfb_info *cfb)
{
	struct page * map = NULL;
	unsigned char *tmp;
	unsigned int page_shift, needroom, t;

	t = jzfb.bpp;
	if (jzfb.bpp == 18 || jzfb.bpp == 24)
		t = 32;
	if (jzfb.bpp == 15)
		t = 16;
	needroom = ((jzfb.w * t + 7) >> 3) * jzfb.h;
	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;

	if (cfb && cfb->fb.screen_base) {
		iounmap(cfb->fb.screen_base);
		cfb->fb.screen_base = NULL;
		release_mem_region(cfb->fb.fix.smem_start,
				   cfb->fb.fix.smem_len);
	}

	if (slcd_palette) {
		map = virt_to_page(slcd_palette);
		clear_bit(PG_reserved, &map->flags);
		free_pages((int)slcd_palette, 0);
	}

	if (slcd_frame) {

		for (tmp=(unsigned char *)slcd_frame; 
		     tmp < slcd_frame + (PAGE_SIZE << page_shift); 
		     tmp += PAGE_SIZE) {
			map = virt_to_page(tmp);
			clear_bit(PG_reserved, &map->flags);
		}

		free_pages((int)slcd_frame, page_shift);
	}
}

static void slcd_descriptor_init(void)
{
	int i;
	int frm_size, pal_size;
	unsigned int next;
	unsigned int  slcd_frame_src_phys_addr, slcd_palette_src_phys_addr, slcd_dma_dst_phys_addr;

	i = jzfb.bpp;
	if (i == 18 || i == 24)
		i = 32;
	if (i == 15)
		i = 16;

	switch (jzfb.bpp) {
	case 1:
		pal_size = 4;
		break;
	case 2:
		pal_size = 8;
		break;
	case 4:
		pal_size = 32;
		break;
	case 8:
	default:
		pal_size = 512;
	}

	frm_size = jzfb.w * jzfb.h * jzfb.bpp / 8;

	/*Offset of next descriptor*/
	slcd_frame_desc_phys_addr = (dma_addr_t)CPHYSADDR((unsigned long)(&slcd_frame_desc));
	slcd_palette_desc_phys_addr = (dma_addr_t)CPHYSADDR((unsigned long)(&slcd_palette_desc));

	/*Soure address and Target address*/
	slcd_palette_src_phys_addr = (unsigned int)virt_to_phys(slcd_palette);
	slcd_frame_src_phys_addr = (unsigned int)virt_to_phys(slcd_frame);
	slcd_dma_dst_phys_addr = (unsigned int)CPHYSADDR(SLCD_FIFO);
	next = slcd_frame_desc_phys_addr >> 4;

	/* Prepare Palette Descriptor */
	slcd_palette_desc.dcmd = DMAC_DCMD_SAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32
		| DMAC_DCMD_DWDH_16 | DMAC_DCMD_DS_16BYTE | DMAC_DCMD_TM | DMAC_DCMD_DES_V 
		| DMAC_DCMD_DES_VIE | DMAC_DCMD_LINK;
	switch (slcd_palette_desc.dcmd & DMAC_DCMD_DS_MASK) {
	case DMAC_DCMD_DS_32BYTE:
		pal_size /= 32;
		break;
	case DMAC_DCMD_DS_16BYTE:
		pal_size /= 16;
		break;
	case DMAC_DCMD_DS_32BIT:
		pal_size /= 4;
		break;
	case DMAC_DCMD_DS_16BIT:
		pal_size /= 2;
		break;
	case DMAC_DCMD_DS_8BIT:
	default:
		break;
	}

	slcd_palette_desc.dsadr = (unsigned int)virt_to_phys(slcd_palette);	/* DMA source address */
	slcd_palette_desc.dtadr = (unsigned int)CPHYSADDR(SLCD_FIFO);	/* DMA target address */
	slcd_palette_desc.ddadr = (volatile unsigned int)((next << 24) | (pal_size & 0xffffff));	/* offset and size*/
	dma_cache_wback((unsigned long)(&slcd_palette_desc), 16);

	/*Prepare Frame Descriptor in memory*/
	switch (jzfb.bpp) {
	case 8 ... 16:
		slcd_frame_desc.dcmd = DMAC_DCMD_SAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32
			| DMAC_DCMD_DWDH_16 | DMAC_DCMD_DS_16BYTE | DMAC_DCMD_TM | DMAC_DCMD_DES_V
			| DMAC_DCMD_DES_VIE | DMAC_DCMD_LINK;
	break;

	case 17 ... 32:
		slcd_frame_desc.dcmd = DMAC_DCMD_SAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32
			| DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BYTE | DMAC_DCMD_TM | DMAC_DCMD_DES_V
			| DMAC_DCMD_DES_VIE | DMAC_DCMD_LINK;
		break;
	}
	switch (slcd_frame_desc.dcmd & DMAC_DCMD_DS_MASK) {
	case DMAC_DCMD_DS_32BYTE:
		frm_size /= 32;
		break;
	case DMAC_DCMD_DS_16BYTE:
		frm_size /= 16;
		break;
	case DMAC_DCMD_DS_32BIT:
		frm_size /= 4;
		break;
	case DMAC_DCMD_DS_16BIT:
		frm_size /= 2;
		break;
	case DMAC_DCMD_DS_8BIT:
	default:
		break;
	}
	
	slcd_frame_desc.dsadr = slcd_frame_src_phys_addr; /* DMA source address */
	slcd_frame_desc.dtadr = slcd_dma_dst_phys_addr;	/* DMA target address */
	slcd_frame_desc.ddadr = (volatile unsigned int)((next << 24) | (frm_size & 0xffffff)); /* offset and size*/
	dma_cache_wback((unsigned long)(&slcd_frame_desc), 16);
}

void slcd_hw_init(void)
{
	unsigned int val, pclk;
	int pll_div;

	REG_LCD_CFG &= ~LCD_CFG_LCDPIN_MASK;
	REG_LCD_CFG |= LCD_CFG_LCDPIN_SLCD;

	if ((jzfb.bpp == 18) | (jzfb.bpp == 24))
		jzfb.bpp = 32;

	/* Configure SLCD module for initialize smart lcd registers*/
	switch (jzfb.bus) {
	case 8:
		REG_SLCD_CFG = SLCD_CFG_BURST_8_WORD | SLCD_CFG_DWIDTH_8_x2 
			| SLCD_CFG_CWIDTH_8BIT | SLCD_CFG_CS_ACTIVE_LOW 
			| SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING 
			| SLCD_CFG_TYPE_PARALLEL;
		__gpio_as_slcd_8bit();
		break;
	case 9:
		REG_SLCD_CFG = SLCD_CFG_BURST_8_WORD | SLCD_CFG_DWIDTH_8_x2
			| SLCD_CFG_CWIDTH_8BIT | SLCD_CFG_CS_ACTIVE_LOW 
			| SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING 
			| SLCD_CFG_TYPE_PARALLEL;
		__gpio_as_slcd_9bit();
		break;
	case 16:
		REG_SLCD_CFG = SLCD_CFG_BURST_8_WORD | SLCD_CFG_DWIDTH_16
			| SLCD_CFG_CWIDTH_16BIT | SLCD_CFG_CS_ACTIVE_LOW
			| SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING
			| SLCD_CFG_TYPE_PARALLEL;
		__gpio_as_slcd_16bit();
		break;
	case 18:
		REG_SLCD_CFG = SLCD_CFG_BURST_8_WORD | SLCD_CFG_DWIDTH_18
			| SLCD_CFG_CWIDTH_18BIT | SLCD_CFG_CS_ACTIVE_LOW 
			| SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING 
			| SLCD_CFG_TYPE_PARALLEL;
		__gpio_as_slcd_18bit();
		break;
	default:
		printk("Error: Don't support BUS %d!\n", jzfb.bus);
		break;
	}

	REG_SLCD_CTRL = SLCD_CTRL_DMA_EN;
	__cpm_stop_lcd();
	pclk = jzfb.pclk;
	pll_div = ( REG_CPM_CPCCR & CPM_CPCCR_PCS ); /* clock source,0:pllout/2 1: pllout */
	pll_div = pll_div ? 1 : 2 ;
	val = ( __cpm_get_pllout()/pll_div ) / pclk;
	val--;
	if ( val > 0x1ff ) {
		printk("CPM_LPCDR too large, set it to 0x1ff\n");
		val = 0x1ff;
	}
	__cpm_set_pixdiv(val);

	REG_CPM_CPCCR |= CPM_CPCCR_CE ; /* update divide */

	jz_clocks.pixclk = __cpm_get_pixclk();
	jz_clocks.lcdclk = __cpm_get_lcdclk();
	printk("SLCDC: PixClock:%d LcdClock:%d\n",
	       jz_clocks.pixclk, jz_clocks.lcdclk);

	__cpm_start_lcd();
	udelay(1000);
	__slcd_display_pin_init();
 	__slcd_special_on();

	/* Configure SLCD module for transfer data to smart lcd GRAM*/
	switch (jzfb.bus) {
	case 8:
		switch (jzfb.bpp) {
		case 8:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x1;
			break;
		case 15:
		case 16:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x2;
			break;
		case 17 ... 32:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x3;
			break;
		default:
			printk("The BPP %d is not supported\n", jzfb.bpp);
			break;
		}
		break;
	case 9:
		switch (jzfb.bpp) {
		case 8:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x1;
			break;
		case 15 ... 16:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x2;
			break;
		case 17 ... 32:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_9_x2;
			break;
		default:
			printk("The BPP %d is not supported\n", jzfb.bpp);
			break;
		}
		break;
	case 16:
		switch (jzfb.bpp) {
		case 8:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x1;
			break;
		case 15 ... 16:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_16;
			break;
		case 17 ... 32:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x3;
			break;
		default:
			printk("The BPP %d is not supported\n", jzfb.bpp);
			break;
		}
		break;
	case 18:
		switch (jzfb.bpp) {
		case 8:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x1;
			break;
		case 15:
		case 16:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_16;
			break;
		case 17 ... 32:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_18;
			break;
		default:
			printk("The BPP %d is not supported\n", jzfb.bpp);
			break;
		}
		break;
	default:
		printk("Error: The BUS %d is not supported\n", jzfb.bus);
		break;
	}
	dprintk("SLCD_CFG=0x%x\n", REG_SLCD_CFG);
}

static irqreturn_t slcd_dma_irq(int irq, void *dev_id)
{

	if (__dmac_channel_transmit_halt_detected(dma_chan)) {
		dprintk("DMA HALT\n");
		__dmac_channel_clear_transmit_halt(dma_chan);
	}

	if (__dmac_channel_address_error_detected(dma_chan)) {
		dprintk("DMA ADDR ERROR\n");
		__dmac_channel_clear_address_error(dma_chan);
	}

	if (__dmac_channel_descriptor_invalid_detected(dma_chan)) {
		dprintk("DMA DESC INVALID\n");
		__dmac_channel_clear_descriptor_invalid(dma_chan);
	}

	if (__dmac_channel_count_terminated_detected(dma_chan)) {
		dprintk("DMA CT\n");
		__dmac_channel_clear_count_terminated(dma_chan);
		if(is_set_reg){
			printk("Close DMAC_DCMD_LINK \n");
			REG_DMAC_DCMD(dma_chan) &= ~DMAC_DCMD_LINK;
		}
		if (non_link_desp) {
			printk("Close DMAC_DCMD_LINK \n");
			/*Set to  Non-Link Descriptor*/
			REG_DMAC_DCMD(dma_chan) &= ~DMAC_DCMD_LINK;
		}
	}

	if (__dmac_channel_transmit_end_detected(dma_chan)) {
		printk("DMA TT\n");
		__dmac_channel_clear_transmit_end(dma_chan);		
		if (non_link_desp) {
			slcd_frame_desc.dcmd |= DMAC_DCMD_TIE;
			slcd_frame_desc.dcmd &= ~DMAC_DCMD_LINK;
			dma_cache_wback((unsigned long)(&slcd_frame_desc), 16);
			non_link_desp = 0;
		}
		if (is_set_reg) {
			is_set_reg = 0;
			while (REG_SLCD_STATE & SLCD_STATE_BUSY);
			REG_DMAC_DMACR &= ~DMAC_DMACR_DMAE; /* disable DMA */
			REG_DMAC_DCCSR(dma_chan) &= ~DMAC_DCCSR_EN;  /* disable DMA */
			REG_SLCD_CTRL = 0;

			/*
			 *add operation here
			 */
			Mcupanel_RegSet(reg_buf.cmd, reg_buf.data);
			Mcupanel_Command(0x0022);/*Write Data to GRAM	*/
			mdelay(100);
			REG_SLCD_CTRL = SLCD_CTRL_DMA_EN;
			REG_DMAC_DMACR = DMAC_DMACR_DMAE;
			REG_DMAC_DCCSR(dma_chan) =  DMAC_DCCSR_EN;
			__dmac_channel_set_doorbell(dma_chan);
		}
	}
	return IRQ_HANDLED;
}

static int slcd_dma_init(void)
{
	/* Request DMA channel and setup irq handler */
	dma_chan = jz_request_dma(DMA_ID_AUTO, "auto", slcd_dma_irq, 0, NULL);
	if (dma_chan < 0) {
		printk("Request DMA Failed\n");
		return -1;
	}
	printk("DMA channel %d is requested by SLCD!\n", dma_chan);

	/*Init the SLCD DMA and Enable*/
	REG_DMAC_DRSR(dma_chan) = DMAC_DRSR_RS_SLCD;
	REG_DMAC_DMACR = DMAC_DMACR_DMAE;
	REG_DMAC_DCCSR(dma_chan) =  DMAC_DCCSR_EN; /*Descriptor Transfer*/

	if (jzfb.bpp <= 8)
		REG_DMAC_DDA(dma_chan) = slcd_palette_desc_phys_addr;
	else
		REG_DMAC_DDA(dma_chan) = slcd_frame_desc_phys_addr;

	/* DMA doorbell set -- start DMA now ... */
	__dmac_channel_set_doorbell(dma_chan);
	return 0;
}

#ifdef CONFIG_PM

/*
 * Suspend the LCDC.
 */
static int jzfb_suspend(void)
{

	__slcd_close_backlight();
	__dmac_disable_channel(dma_chan);
	__slcd_dma_disable(); /* Quick Disable */
	__slcd_special_off(); 
	__cpm_stop_lcd();
	return 0;
}

/*
 * Resume the LCDC.
 */

static int jzfb_resume(void)
{
	__cpm_start_lcd();
	REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
	switch (jzfb.bpp) {
	case 8:
		/* DATA 8-bit once*/
		REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x1;
		break;
	case 15:
	case 16:
	case 18:
	case 24:
	case 32:
	/* DATA 8-bit twice*/
		REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x2;
		break;
	default:
		REG_SLCD_CFG = SLCD_CFG_DWIDTH_8_x2;
		break;
	}
	__slcd_display_pin_init();
	__slcd_special_on();

	if (jzfb.bpp == 32) {
		/* DATA 8-bit three time*/
		REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
		REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x3;
	}
	__slcd_dma_enable();
	udelay(100);
	__dmac_enable_channel(dma_chan);
	__dmac_channel_set_doorbell(dma_chan);
	mdelay(200);
	__slcd_set_backlight_level(80); 
	return 0;
}

/*
 * Power management hook.  Note that we won't be called from IRQ context,
 * unlike the blank functions above, so we may sleep.
 */
static int jzslcd_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
{
	int ret;
	struct lcd_cfb_info *cfb = pm_dev->data;

	if (!cfb) return -EINVAL;

	switch (req) {
	case PM_SUSPEND:
		ret = jzfb_suspend();
		break;

	case PM_RESUME:
		ret = jzfb_resume();
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}
#else
#define jzfb_suspend      NULL
#define jzfb_resume       NULL
#endif /* CONFIG_PM */
static int __init jzslcd_fb_init(void)
{

	struct lcd_cfb_info *cfb;
	int err = 0;

	/*the parameters of slcd*/
	cfb = jzfb_alloc_fb_info();
	if (!cfb)
		goto failed;

	err = jzfb_map_smem(cfb);
	if (err)
		goto failed;
	jzfb_set_var(&cfb->fb.var, -1, &cfb->fb);

	slcd_hw_init();

	err = register_framebuffer(&cfb->fb);
	if (err < 0) {
		printk("jzslcd_fb_init(): slcd register framebuffer err.\n");
		goto failed;
	}

	printk("fb%d: %s frame buffer device, using %dK of video memory\n",
	       cfb->fb.node, cfb->fb.fix.id, cfb->fb.fix.smem_len>>10);

	slcd_descriptor_init();
	err = slcd_dma_init();
	if (err != 0) {
		printk("SLCD Init DMA Fail!\n");
		return err;
	}
	mdelay(100);
	__slcd_set_backlight_level(80);

#ifdef CONFIG_PM
	/*
	 * Note that the console registers this as well, but we want to
	 * power down the display prior to sleeping.
	 */
//struct pm_dev __deprecated *pm_register(pm_dev_t type, unsigned long id, pm_callback callback);

	cfb->pm = pm_register(PM_SYS_DEV, PM_SYS_VGA, jzslcd_pm_callback);
	if (cfb->pm)
		cfb->pm->data = cfb;

#endif
	return 0;

failed:
	jzfb_unmap_smem(cfb);
	jzfb_free_fb_info(cfb);

	return err;
}

#if 0
static int jzfb_remove(struct device *dev)
{
	struct lcd_cfb_info *cfb = dev_get_drvdata(dev);
	jzfb_unmap_smem(cfb);
	jzfb_free_fb_info(cfb);
	return 0;
}
#endif

#if 0
static struct device_driver jzfb_driver = {
	.name		= "jz-slcd",
	.bus 		= &platform_bus_type,
	.probe		= jzfb_probe,
        .remove		= jzfb_remove,
	.suspend	= jzfb_suspend,
        .resume		= jzfb_resume,
};
#endif

static void __exit jzslcd_fb_cleanup(void)
{
	//driver_unregister(&jzfb_driver);
	//jzfb_remove();
}

module_init(jzslcd_fb_init);
module_exit(jzslcd_fb_cleanup);

MODULE_DESCRIPTION("JzSOC SLCD Controller driver");
MODULE_LICENSE("GPL");
