/*
 * linux/drivers/video/jzlcd.c -- Ingenic On-Chip LCD frame buffer device
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
#include <linux/kthread.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/jzsoc.h>

#include "console/fbcon.h"

#include "jzlcd.h"

#undef CONFIG_PM
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

struct lcd_cfb_info {
	struct fb_info		fb;
	struct display_switch	*dispsw;
	signed int		currcon;
	int			func_use_count;

	struct {
		u16 red, green, blue;
	} palette[NR_PALETTE];
#ifdef CONFIG_PM
	struct pm_dev		*pm;
#endif
#if defined(CONFIG_JZLCD_FRAMEBUFFER_ROTATE_SUPPORT)
	struct task_struct *rotate_daemon_thread;
#endif
};

static struct lcd_cfb_info *jzlcd_info;

struct jzfb_info {
	unsigned int cfg;	/* panel mode and pin usage etc. */
	unsigned int w;
	unsigned int h;
	unsigned int bpp;	/* bit per pixel */
	unsigned int fclk;	/* frame clk */
	unsigned int hsw;	/* hsync width, in pclk */
	unsigned int vsw;	/* vsync width, in line count */
	unsigned int elw;	/* end of line, in pclk */
	unsigned int blw;	/* begin of line, in pclk */
	unsigned int efw;	/* end of frame, in line count */
	unsigned int bfw;	/* begin of frame, in line count */
};

static struct jzfb_info jzfb = {
#if defined(CONFIG_JZLCD_SHARP_LQ035Q7)
	MODE_TFT_SHARP | PCLK_N | VSYNC_N,
	240, 320, 16, 60, 1, 2, 1, 2, 0, 6
#endif
#if defined(CONFIG_JZLCD_SAMSUNG_LTS350Q1)
	MODE_TFT_SAMSUNG | PCLK_N,
	240, 320, 16, 60, 1, 2, (254-240), 0, 7, 0
#endif
#if defined(CONFIG_JZLCD_SAMSUNG_LTV350QVF04)
	MODE_TFT_GEN | HSYNC_N | VSYNC_N,
	320, 240, 16, 70, 19, 4, 20, 14, 18, 6
#endif
#if defined(CONFIG_JZLCD_SAMSUNG_LTP400WQF01)
	MODE_TFT_GEN | HSYNC_N | VSYNC_N,
	480, 272, 16, 60, 41, 10, 2, 2, 2, 2
#endif

#if defined(CONFIG_JZLCD_SAMSUNG_LTP400WQF02)
	/* MODE_TFT_18BIT: JZ4740@ version */
	MODE_TFT_GEN | MODE_TFT_18BIT | HSYNC_N | VSYNC_N,
	480, 272, 32, 60, 41, 10, 2, 2, 2, 2
#endif
#if defined(CONFIG_JZLCD_TRULY_TFTG320240DTSW)
	MODE_TFT_GEN | HSYNC_N | VSYNC_N | PCLK_N,
	320, 240, 16, 85, 30, 3, 38, 20, 11, 8
#endif
#if defined(CONFIG_JZLCD_TRULY_TFTG320240DTSW_SERIAL)
	MODE_8BIT_SERIAL_TFT | HSYNC_N | VSYNC_N | PCLK_N,
	/* serial mode 280 lines, parallel mode 240 lines */
	320, 280, 32, 60, (30*3), 3, (20*3), (38*3), 46, 23 
#endif
#if defined(CONFIG_JZLCD_AUO_A030FL01_V1)
	MODE_TFT_GEN | MODE_TFT_18BIT | HSYNC_N | VSYNC_N,
	480, 272, 32, 60, 39, 10, 8, 4, 4, 2
#endif
#if defined(CONFIG_JZLCD_TRULY_TFTG240320UTSW_63W_E)
	MODE_TFT_GEN | HSYNC_N | VSYNC_N | PCLK_N | DE_N,
	320, 240, 16, 60, 3, 3, 3, 3, 3, 85 /* 320x240 */
#endif
#if defined(CONFIG_JZLCD_FOXCONN_PT035TN01) && defined(CONFIG_JZ4740_PAVO)
	MODE_TFT_GEN | HSYNC_N | VSYNC_N | MODE_TFT_18BIT | PCLK_N,
//	320, 240, 18, 110, 1, 1, 10, 50, 10, 13
	320, 240, 18, 80, 1, 1, 10, 50, 10, 13
#endif
#if defined(CONFIG_JZLCD_FOXCONN_PT035TN01) && !(defined(CONFIG_JZ4740_PAVO))
	MODE_TFT_GEN | HSYNC_N | VSYNC_N  | PCLK_N,
	320, 240, 16, 110, 1, 1, 10, 50, 10, 13
#endif
#if defined(CONFIG_JZLCD_INNOLUX_PT035TN01_SERIAL)
	MODE_8BIT_SERIAL_TFT | PCLK_N | HSYNC_N | VSYNC_N,
	320, 240, 32, 60, 1, 1, 10, 50, 10, 13
#endif
#if defined(CONFIG_JZLCD_HYNIX_HT10X21)
	MODE_TFT_GEN | PCLK_N,
	1024, 768, 16, 45, 1, 1, 75, 0, 3, 0
#endif
#if defined(CONFIG_JZLCD_TOSHIBA_LTM084P363)
	MODE_TFT_GEN | PCLK_N,
	800, 600, 16, 50, 1, 2, 199, 0, 2, 0
#endif
#if defined(CONFIG_JZLCD_INNOLUX_AT080TN42)
	MODE_TFT_SHARP | PCLK_N,
	800, 600, 16, 40, 1, 1, 255, 0, 34, 0 
#endif
#if defined(CONFIG_JZLCD_CSTN_800x600)
	MODE_STN_COLOR_DUAL | STN_DAT_PIN8,
	800, 600, 16, 30, 8, 1, 0, 0, 0, 0
#endif
#if defined(CONFIG_JZLCD_CSTN_320x240)
	MODE_STN_COLOR_SINGLE | STN_DAT_PIN8,
	320, 240, 16, 120, 8, 1, 8, 0, 0, 0
#endif
#if defined(CONFIG_JZLCD_MSTN_640x480)
	MODE_STN_MONO_DUAL | STN_DAT_PIN4,
	640, 480, 8, 110, 4, 1, 4, 0, 0, 0
#endif
#if defined(CONFIG_JZLCD_MSTN_320x240)
	MODE_STN_MONO_SINGLE | STN_DAT_PIN4,
	320, 240, 8, 110, 4, 1, 4, 0, 0, 0
#endif
#if defined(CONFIG_JZLCD_MSTN_480x320)
	MODE_STN_MONO_SINGLE | STN_DAT_PIN8
#if defined(CONFIG_JZLCD_MSTN_INVERSE)
	| DATA_INVERSE
#endif
	, 480, 320, 8, 65, 8, 1, 8, 0, 0, 0
#endif

#if defined(CONFIG_JZLCD_MSTN_240x128)
	MODE_STN_MONO_SINGLE | STN_DAT_PIN1
#if defined(CONFIG_JZLCD_MSTN_INVERSE)
	| DATA_INVERSE
#endif
	, 240, 128, 8, 100, 1, 1, 1, 0, 0, 0 
#endif
};

static struct lcd_desc *lcd_desc_base;
static struct lcd_desc *lcd_palette_desc;
static struct lcd_desc *lcd_frame_desc0;
static struct lcd_desc *lcd_frame_desc1;

static unsigned char *lcd_palette;
static unsigned char *lcd_frame[CONFIG_JZLCD_FRAMEBUFFER_MAX];
struct jz_lcd_buffer_addrs_t jz_lcd_buffer_addrs;
//extern struct display fb_display[MAX_NR_CONSOLES];
#if defined(CONFIG_JZLCD_FRAMEBUFFER_ROTATE_SUPPORT)
static unsigned char *lcd_frame_user_fb;
/* default rotate angle */
static volatile int rotate_angle = CONFIG_JZLCD_FRAMEBUFFER_DEFAULT_ROTATE_ANGLE;
#endif

#ifdef  DEBUG
static void print_regs(void)	/* debug */
{
	printk("REG_LCD_CFG:\t0x%8.8x\n", REG_LCD_CFG);
	printk("REG_LCD_VSYNC:\t0x%8.8x\n", REG_LCD_VSYNC);
	printk("REG_LCD_HSYNC:\t0x%8.8x\n", REG_LCD_HSYNC);
	printk("REG_LCD_VAT:\t0x%8.8x\n", REG_LCD_VAT);
	printk("REG_LCD_DAH:\t0x%8.8x\n", REG_LCD_DAH);
	printk("REG_LCD_DAV:\t0x%8.8x\n", REG_LCD_DAV);
	printk("REG_LCD_PS:\t0x%8.8x\n", REG_LCD_PS);
	printk("REG_LCD_CLS:\t0x%8.8x\n", REG_LCD_CLS);
	printk("REG_LCD_SPL:\t0x%8.8x\n", REG_LCD_SPL);
	printk("REG_LCD_REV:\t0x%8.8x\n", REG_LCD_REV);
	printk("REG_LCD_CTRL:\t0x%8.8x\n", REG_LCD_CTRL);
	printk("REG_LCD_STATE:\t0x%8.8x\n", REG_LCD_STATE);
	printk("REG_LCD_IID:\t0x%8.8x\n", REG_LCD_IID);
	printk("REG_LCD_DA0:\t0x%8.8x\n", REG_LCD_DA0);
	printk("REG_LCD_SA0:\t0x%8.8x\n", REG_LCD_SA0);
	printk("REG_LCD_FID0:\t0x%8.8x\n", REG_LCD_FID0);
	printk("REG_LCD_CMD0:\t0x%8.8x\n", REG_LCD_CMD0);

	printk("==================================\n");
	printk("REG_LCD_VSYNC:\t%d:%d\n", REG_LCD_VSYNC>>16, REG_LCD_VSYNC&0xfff);
	printk("REG_LCD_HSYNC:\t%d:%d\n", REG_LCD_HSYNC>>16, REG_LCD_HSYNC&0xfff);
	printk("REG_LCD_VAT:\t%d:%d\n", REG_LCD_VAT>>16, REG_LCD_VAT&0xfff);
	printk("REG_LCD_DAH:\t%d:%d\n", REG_LCD_DAH>>16, REG_LCD_DAH&0xfff);
	printk("REG_LCD_DAV:\t%d:%d\n", REG_LCD_DAV>>16, REG_LCD_DAV&0xfff);
	printk("==================================\n");

}
#else
#define print_regs()
#endif

#if defined(CONFIG_JZLCD_FRAMEBUFFER_ROTATE_SUPPORT)
static int jzfb_rotate_daemon_thread(void *info)
{
	int i,j;
	struct fb_info *fb = &jzlcd_info->fb;

	while (!kthread_should_stop()) {
#if (CONFIG_JZLCD_FRAMEBUFFER_BPP == 8)
		unsigned char *plcd_frame = (unsigned char *)lcd_frame[0];
		unsigned char *pfb = (unsigned char *) (fb->screen_base);
#elif (CONFIG_JZLCD_FRAMEBUFFER_BPP == 16)
		unsigned short *plcd_frame = (unsigned short *)lcd_frame[0];
		unsigned short *pfb = (unsigned short *) (fb->screen_base);
#elif (CONFIG_JZLCD_FRAMEBUFFER_BPP == 32)
		unsigned int *plcd_frame = (unsigned int *)lcd_frame[0];
		unsigned int *pfb = (unsigned int *) (fb->screen_base);
#else
#error	"ERROR, rotate not support this bpp."
#endif
		switch ( rotate_angle ) {
		case FB_ROTATE_UR:
			printk("%s, Warning, this shouldn't reache\n", __FUNCTION__);
			ssleep(1);
			break;
		case FB_ROTATE_UD: /* cost about 30ms, can be accelrated by dma in the future */
			plcd_frame += jzfb.w*jzfb.h -1;
			for (i=0;i<jzfb.h*jzfb.w;i++)
				*plcd_frame-- = *pfb++;
			msleep(75);
			break;
		case FB_ROTATE_CW:  /* cost about 80ms */
			for (i=1;i<fb->var.height+1; i++) {
				for (j=1; j < fb->var.width+1; j++)
					plcd_frame[j*fb->var.height-i] = *pfb++;
			}
			msleep(100); /* sleep 100ms */
			break;
		case FB_ROTATE_CCW:  /* cost about 80ms */
			for (i=0;i<fb->var.height;i++) {
				for ( j=fb->var.width-1;j>=0;j--)
					plcd_frame[j*fb->var.height+i] = *pfb++;
			}
			msleep(100); /* sleep 100ms */
			break;
		default:	/* FB_ROTATE_UR */
			dprintk("Unknown rotate(%d) type\n", rotate_angle);
			ssleep(1);
		}

		dma_cache_wback_inv((unsigned int)(lcd_frame_user_fb), fb->fix.smem_len);
	}
	return 0;
}
/* 
 * rotate param angle:
 * 	0: FB_ROTATE_UR, 0'C
 * 	1: FB_ROTATE_CW, 90'C
 * 	2: FB_ROTATE_UD, 180'C
 * 	3: FB_ROTATE_CCW, 270'C
 */
static int jzfb_rotate_change( int angle )
{
	struct fb_info *fb = &jzlcd_info->fb;

	/* clear frame buffer */
	memset((void*)lcd_frame_user_fb, 0x00, fb->fix.smem_len);
	switch ( angle ) {
	case FB_ROTATE_UR:
		fb->var.width	= fb->var.xres = fb->var.xres_virtual = jzfb.w;
		fb->var.height	= fb->var.yres = fb->var.yres_virtual = jzfb.h;
		/* change lcd controller's data buffer to lcd_frame_user_fb*/
		lcd_frame_desc0->databuf = virt_to_phys((void *)lcd_frame_user_fb);
		if ( rotate_angle != FB_ROTATE_UR )
			kthread_stop(jzlcd_info->rotate_daemon_thread);
		rotate_angle = angle;
		break;
	case FB_ROTATE_UD:
	case FB_ROTATE_CW:
	case FB_ROTATE_CCW:
		if ( angle == FB_ROTATE_UD ) {
			fb->var.width	= fb->var.xres = fb->var.xres_virtual = jzfb.w;
			fb->var.height	= fb->var.yres = fb->var.yres_virtual = jzfb.h;
		}
		else {	/* CW, CCW */
			fb->var.width	= fb->var.xres = fb->var.xres_virtual = jzfb.h;
			fb->var.height	= fb->var.yres = fb->var.yres_virtual = jzfb.w;
		}
		/* change lcd controller's data buffer to lcd_frame[0]*/
		lcd_frame_desc0->databuf = virt_to_phys((void *)lcd_frame[0]);
		if ( rotate_angle == FB_ROTATE_UR ||		\
		     jzlcd_info->rotate_daemon_thread == NULL) 
				jzlcd_info->rotate_daemon_thread = kthread_run( jzfb_rotate_daemon_thread, jzlcd_info, "%s", "jzlcd-rotate-daemon"); /* start rotate daemon */
		rotate_angle = angle;
		break;
	default:
		printk("Invalid angle(%d)\n", (unsigned int)angle);
	}
	fb->fix.line_length = fb->var.xres * CONFIG_JZLCD_FRAMEBUFFER_BPP/8;
	dma_cache_wback_inv((unsigned int)(lcd_frame_desc0), sizeof(struct lcd_desc));	
	return 0;
}

void jzfb_fb_rotate(struct fb_info *fbi, int angle)
{
	jzfb_rotate_change( angle/90 );
}
#endif	/* #if defined(CONFIG_JZLCD_FRAMEBUFFER_ROTATE_SUPPORT) */

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

//	print_dbg("regno:%d,RGBt:(%d,%d,%d,%d)\t", regno, red, green, blue, transp);
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
		if (((jzfb.cfg & MODE_MASK) == MODE_STN_MONO_SINGLE) ||
		    ((jzfb.cfg & MODE_MASK) == MODE_STN_MONO_DUAL)) {
			ctmp = (77L * red + 150L * green + 29L * blue) >> 8;
			ctmp = ((ctmp >> 3) << 11) | ((ctmp >> 2) << 5) |
				(ctmp >> 3);
		} else {
			/* RGB 565 */
			if (((red >> 3) == 0) && ((red >> 2) != 0))
				red = 1 << 3;
			if (((blue >> 3) == 0) && ((blue >> 2) != 0))
				blue = 1 << 3;
			ctmp = ((red >> 3) << 11) 
				| ((green >> 2) << 5) | (blue >> 3);
		}

		ptr = (unsigned short *)lcd_palette;
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


static int jzfb_ioctl (struct fb_info *fb, unsigned int cmd, unsigned long arg )
{
	int ret = 0;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case FBIOSETBACKLIGHT:
		__lcd_set_backlight_level(arg);	/* We support 8 levels here. */
		break;
	case FBIODISPON:
		__lcd_display_on();
		break;
	case FBIODISPOFF:
		__lcd_display_off();
		break;
	case FBIOPRINT_REGS:
		print_regs();
		break;
	case FBIOGETBUFADDRS:
		if ( copy_to_user(argp, &jz_lcd_buffer_addrs,
				  sizeof(struct jz_lcd_buffer_addrs_t)) )
			return -EFAULT;
		break;
#if defined(CONFIG_JZLCD_FRAMEBUFFER_ROTATE_SUPPORT)
	case FBIOROTATE:
		ret = jzfb_rotate_change(arg);
		break;
#endif	/* defined(CONFIG_JZLCD_FRAMEBUFFER_ROTATE_SUPPORT) */
	default:
		printk("Warn: Command(%x) not support\n", cmd);
		ret = -1;
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
 	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;		/* Uncacheable */
//	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;	/* Write-Through */
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
	print_dbg("jzfb_set_par");
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
		//case FB_BLANK_NORMAL:
			/* Turn on panel */
		__lcd_set_ena();
		__lcd_display_on();
		break;

	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
#if 0
			/* Turn off panel */
		__lcd_set_dis();
		__lcd_display_off();
#endif
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

		lcd_frame_desc0->databuf += (cfb->fb.fix.line_length * dy);
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
#if defined(CONFIG_JZLCD_FRAMEBUFFER_ROTATE_SUPPORT)
	.fb_rotate		= jzfb_fb_rotate,
#endif
};

static int jzfb_set_var(struct fb_var_screeninfo *var, int con,
			struct fb_info *info)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
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
		/* DIRECTCOLOUR, 16M */
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
	 * Update the old var.  The fbcon drivers still use this.
	 * Once they are using cfb->fb.var, this can be dropped.
	 *					--rmk
	 */
	//display->var = cfb->fb.var;
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

	jzlcd_info = cfb;

	memset(cfb, 0, sizeof(struct lcd_cfb_info) );

	cfb->currcon		= -1;


	strcpy(cfb->fb.fix.id, "jz-lcd");
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
#if defined(CONFIG_SOC_JZ4740)
	if (jzfb.bpp == 18 || jzfb.bpp == 24)
		t = 32;
	else
		t = jzfb.bpp;
#else
	if (jzfb.bpp == 15)
		t = 16;
	else
		t = jzfb.bpp;
#endif

	needroom = ((jzfb.w * t + 7) >> 3) * jzfb.h;
	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;

	/* lcd_palette room total 4KB:
	 * 0 -- 512: lcd palette
	 * 1024 -- [1024+16*3]: lcd descripters
	 * [1024+16*3] -- 4096: reserved
	 */
	lcd_palette = (unsigned char *)__get_free_pages(GFP_KERNEL, 0);
	if ((!lcd_palette))
		return -ENOMEM;

	memset((void *)lcd_palette, 0, PAGE_SIZE);
	map = virt_to_page(lcd_palette);
	set_bit(PG_reserved, &map->flags);
	lcd_desc_base  = (struct lcd_desc *)(lcd_palette + 1024);

	jz_lcd_buffer_addrs.fb_num = CONFIG_JZLCD_FRAMEBUFFER_MAX;
	printk("jzlcd use %d framebuffer:\n", CONFIG_JZLCD_FRAMEBUFFER_MAX);
	/* alloc frame buffer space */
	for ( t = 0; t < CONFIG_JZLCD_FRAMEBUFFER_MAX; t++ ) {
		lcd_frame[t] = (unsigned char *)__get_free_pages(GFP_KERNEL, page_shift);
		if ((!lcd_frame[t])) {
			printk("no mem for fb[%d]\n", t);
			return -ENOMEM;
		}
//		memset((void *)lcd_frame[t], 0, PAGE_SIZE << page_shift);
		for (tmp=(unsigned char *)lcd_frame[t];
		     tmp < lcd_frame[t] + (PAGE_SIZE << page_shift);
		     tmp += PAGE_SIZE) {
			map = virt_to_page(tmp);
			set_bit(PG_reserved, &map->flags);
		}
		jz_lcd_buffer_addrs.fb_phys_addr[t] = virt_to_phys((void *)lcd_frame[t]);
		printk("jzlcd fb[%d] phys addr =0x%08x\n", 
		       t, jz_lcd_buffer_addrs.fb_phys_addr[t]);
	}
#if !defined(CONFIG_JZLCD_FRAMEBUFFER_ROTATE_SUPPORT)
	cfb->fb.fix.smem_start = virt_to_phys((void *)lcd_frame[0]);
	cfb->fb.fix.smem_len = (PAGE_SIZE << page_shift);
	cfb->fb.screen_base =
		(unsigned char *)(((unsigned int)lcd_frame[0] & 0x1fffffff) | 0xa0000000);
#else  /* Framebuffer rotate */
	lcd_frame_user_fb = (unsigned char *)__get_free_pages(GFP_KERNEL, page_shift);
	if ((!lcd_frame_user_fb)) {
		printk("no mem for fb[%d]\n", t);
		return -ENOMEM;
	}
	memset((void *)lcd_frame_user_fb, 0, PAGE_SIZE << page_shift);
	for (tmp=(unsigned char *)lcd_frame_user_fb;
	     tmp < lcd_frame_user_fb + (PAGE_SIZE << page_shift);
	     tmp += PAGE_SIZE) {
		map = virt_to_page(tmp);
		set_bit(PG_reserved, &map->flags);
	}

	printk("Rotate userfb phys addr =0x%08x\n", 
	       (unsigned int)virt_to_phys((void *)lcd_frame_user_fb));
	cfb->fb.fix.smem_start = virt_to_phys((void *)lcd_frame_user_fb);
	cfb->fb.fix.smem_len = (PAGE_SIZE << page_shift);
	cfb->fb.screen_base = (unsigned char *)(((unsigned int)lcd_frame_user_fb & 0x1fffffff) | 0xa0000000);

#endif	/* #if defined(CONFIG_JZLCD_FRAMEBUFFER_ROTATE_SUPPORT) */
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
#if defined(CONFIG_SOC_JZ4740)
	if (jzfb.bpp == 18 || jzfb.bpp == 24)
		t = 32;
	else
		t = jzfb.bpp;
#else
	if (jzfb.bpp == 15)
		t = 16;
	else
		t = jzfb.bpp;
#endif
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

	if (lcd_palette) {
		map = virt_to_page(lcd_palette);
		clear_bit(PG_reserved, &map->flags);
		free_pages((int)lcd_palette, 0);
	}

	for ( t=0; t < CONFIG_JZLCD_FRAMEBUFFER_MAX; t++ ) {
		if (lcd_frame[t]) {
			for (tmp=(unsigned char *)lcd_frame[t]; 
			     tmp < lcd_frame[t] + (PAGE_SIZE << page_shift); 
			     tmp += PAGE_SIZE) {
				map = virt_to_page(tmp);
				clear_bit(PG_reserved, &map->flags);
			}
			free_pages((int)lcd_frame[t], page_shift);
		}
	}
#if defined(CONFIG_JZLCD_FRAMEBUFFER_ROTATE_SUPPORT)
	if (lcd_frame_user_fb) {
		for (tmp=(unsigned char *)lcd_frame_user_fb; 
		     tmp < lcd_frame_user_fb + (PAGE_SIZE << page_shift); 
		     tmp += PAGE_SIZE) {
			map = virt_to_page(tmp);
			clear_bit(PG_reserved, &map->flags);
		}
		free_pages((int)lcd_frame_user_fb, page_shift);
	}
	
#endif
}

static void lcd_descriptor_init(void)
{
	int i;
	unsigned int pal_size;
	unsigned int frm_size, ln_size;
	unsigned char dual_panel = 0;

	i = jzfb.bpp;
#if defined(CONFIG_SOC_JZ4740)
	if (i == 18 || i == 24)
		i = 32;
#else
	if (i == 15)
		i = 16;
#endif
	frm_size = (jzfb.w*jzfb.h*i)>>3;
	ln_size = (jzfb.w*i)>>3;

	if (((jzfb.cfg & MODE_MASK) == MODE_STN_COLOR_DUAL) ||
	    ((jzfb.cfg & MODE_MASK) == MODE_STN_MONO_DUAL)) {
		dual_panel = 1;
		frm_size >>= 1;
	}

	frm_size = frm_size / 4;
	ln_size = ln_size / 4;

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

	pal_size /= 4;

	lcd_frame_desc0  = lcd_desc_base + 0;
	lcd_frame_desc1  = lcd_desc_base + 1;
	lcd_palette_desc = lcd_desc_base + 2;

	jz_lcd_buffer_addrs.lcd_desc_phys_addr = (unsigned int)virt_to_phys(lcd_frame_desc0);

	/* Palette Descriptor */
	lcd_palette_desc->next_desc = (int)virt_to_phys(lcd_frame_desc0);
	lcd_palette_desc->databuf = (int)virt_to_phys((void *)lcd_palette);
	lcd_palette_desc->frame_id = (unsigned int)0xdeadbeaf;
	lcd_palette_desc->cmd = pal_size|LCD_CMD_PAL; /* Palette Descriptor */

	/* Frame Descriptor 0 */
	if (jzfb.bpp <= 8)
		lcd_frame_desc0->next_desc = (int)virt_to_phys(lcd_palette_desc);
	else
		lcd_frame_desc0->next_desc = (int)virt_to_phys(lcd_frame_desc0);
	lcd_frame_desc0->databuf = virt_to_phys((void *)lcd_frame[0]);
	lcd_frame_desc0->frame_id = (unsigned int)0xbeafbeaf;
	lcd_frame_desc0->cmd = LCD_CMD_SOFINT | LCD_CMD_EOFINT | frm_size;
	dma_cache_wback_inv((unsigned int)(lcd_palette_desc),0x10);
	dma_cache_wback_inv((unsigned int)(lcd_frame_desc0),0x10);

	if (!(dual_panel))
		return;

	/* Frame Descriptor 1 */
	lcd_frame_desc1->next_desc = (int)virt_to_phys(lcd_frame_desc1);
	lcd_frame_desc1->databuf = virt_to_phys((void *)(lcd_frame[0] + frm_size * 4));
	lcd_frame_desc1->frame_id = (unsigned int)0xdeaddead;
	lcd_frame_desc1->cmd = LCD_CMD_SOFINT | LCD_CMD_EOFINT | frm_size;
	dma_cache_wback_inv((unsigned int)(lcd_frame_desc1),0x10);
}

static int lcd_hw_init(void)
{
	unsigned int val = 0;
	unsigned int pclk;
	unsigned int stnH;
	int ret = 0;

	/* Setting Control register */
	switch (jzfb.bpp) {
	case 1:
		val |= LCD_CTRL_BPP_1;
		break;
	case 2:
		val |= LCD_CTRL_BPP_2;
		break;
	case 4:
		val |= LCD_CTRL_BPP_4;
		break;
	case 8:
		val |= LCD_CTRL_BPP_8;
		break;
	case 15:
		val |= LCD_CTRL_RGB555;
	case 16:
		val |= LCD_CTRL_BPP_16;
		break;
#if defined(CONFIG_SOC_JZ4740)
	case 17 ... 32:
		val |= LCD_CTRL_BPP_18_24;	/* target is 4bytes/pixel */
		break;
#endif
	default:
		printk("The BPP %d is not supported\n", jzfb.bpp);
		val |= LCD_CTRL_BPP_16;
		break;
	}

	switch (jzfb.cfg & MODE_MASK) {
	case MODE_STN_MONO_DUAL:
	case MODE_STN_COLOR_DUAL:
	case MODE_STN_MONO_SINGLE:
	case MODE_STN_COLOR_SINGLE:
		switch (jzfb.bpp) {
		case 1:
		case 2:
			val |= LCD_CTRL_FRC_2;
			break;
		case 4:
			val |= LCD_CTRL_FRC_4;
			break;
		case 8:
		default:
			val |= LCD_CTRL_FRC_16;
			break;
		}
		break;
	}

	val |= LCD_CTRL_BST_16;		/* Burst Length is 16WORD=64Byte */

	switch (jzfb.cfg & MODE_MASK) {
	case MODE_STN_MONO_DUAL:
	case MODE_STN_COLOR_DUAL:
	case MODE_STN_MONO_SINGLE:
	case MODE_STN_COLOR_SINGLE:
		switch (jzfb.cfg & STN_DAT_PINMASK) {
#define align2(n) (n)=((((n)+1)>>1)<<1)
#define align4(n) (n)=((((n)+3)>>2)<<2)
#define align8(n) (n)=((((n)+7)>>3)<<3)
		case STN_DAT_PIN1:
			/* Do not adjust the hori-param value. */
			break;
		case STN_DAT_PIN2:
			align2(jzfb.hsw);
			align2(jzfb.elw);
			align2(jzfb.blw);
			break;
		case STN_DAT_PIN4:
			align4(jzfb.hsw);
			align4(jzfb.elw);
			align4(jzfb.blw);
			break;
		case STN_DAT_PIN8:
			align8(jzfb.hsw);
			align8(jzfb.elw);
			align8(jzfb.blw);
			break;
		}
		break;
	}

	val |=  1 << 26;               /* Output FIFO underrun protection */
	REG_LCD_CTRL = val;

	switch (jzfb.cfg & MODE_MASK) {
	case MODE_STN_MONO_DUAL:
	case MODE_STN_COLOR_DUAL:
	case MODE_STN_MONO_SINGLE:
	case MODE_STN_COLOR_SINGLE:
		if (((jzfb.cfg & MODE_MASK) == MODE_STN_MONO_DUAL) ||
		    ((jzfb.cfg & MODE_MASK) == MODE_STN_COLOR_DUAL))
			stnH = jzfb.h >> 1;
		else
			stnH = jzfb.h;

		REG_LCD_VSYNC = (0 << 16) | jzfb.vsw;
		REG_LCD_HSYNC = ((jzfb.blw+jzfb.w) << 16) | (jzfb.blw+jzfb.w+jzfb.hsw);

		/* Screen setting */
		REG_LCD_VAT = ((jzfb.blw + jzfb.w + jzfb.hsw + jzfb.elw) << 16) | (stnH + jzfb.vsw + jzfb.bfw + jzfb.efw);
		REG_LCD_DAH = (jzfb.blw << 16) | (jzfb.blw + jzfb.w);
		REG_LCD_DAV = (0 << 16) | (stnH);

		/* AC BIAs signal */
		REG_LCD_PS = (0 << 16) | (stnH+jzfb.vsw+jzfb.efw+jzfb.bfw);

		break;

	case MODE_TFT_GEN:
	case MODE_TFT_SHARP:
	case MODE_TFT_CASIO:
	case MODE_TFT_SAMSUNG:
	case MODE_8BIT_SERIAL_TFT:
	case MODE_TFT_18BIT:
		REG_LCD_VSYNC = (0 << 16) | jzfb.vsw;
#if defined(CONFIG_JZLCD_INNOLUX_AT080TN42)
		REG_LCD_DAV = (0 << 16) | ( jzfb.h );
#else
		REG_LCD_DAV = ((jzfb.vsw + jzfb.bfw) << 16) | (jzfb.vsw + jzfb.bfw + jzfb.h);
#endif /*#if defined(CONFIG_JZLCD_INNOLUX_AT080TN42)*/
		REG_LCD_VAT = (((jzfb.blw + jzfb.w + jzfb.elw + jzfb.hsw)) << 16) | (jzfb.vsw + jzfb.bfw + jzfb.h + jzfb.efw);
		REG_LCD_HSYNC = (0 << 16) | jzfb.hsw;
		REG_LCD_DAH = ((jzfb.hsw + jzfb.blw) << 16) | (jzfb.hsw + jzfb.blw + jzfb.w);
		break;
	}

	switch (jzfb.cfg & MODE_MASK) {
	case MODE_TFT_SAMSUNG:
	{
		unsigned int total, tp_s, tp_e, ckv_s, ckv_e;
		unsigned int rev_s, rev_e, inv_s, inv_e;
		total = jzfb.blw + jzfb.w + jzfb.elw + jzfb.hsw;
		tp_s = jzfb.blw + jzfb.w + 1;
		tp_e = tp_s + 1;
		ckv_s = tp_s - jz_clocks.pixclk/(1000000000/4100);
		ckv_e = tp_s + total;
		rev_s = tp_s - 11;	/* -11.5 clk */
		rev_e = rev_s + total;
		inv_s = tp_s;
		inv_e = inv_s + total;
		REG_LCD_CLS = (tp_s << 16) | tp_e;
		REG_LCD_PS = (ckv_s << 16) | ckv_e;
		REG_LCD_SPL = (rev_s << 16) | rev_e;
		REG_LCD_REV = (inv_s << 16) | inv_e;
		jzfb.cfg |= STFT_REVHI | STFT_SPLHI;
		break;
	}
	case MODE_TFT_SHARP:
	{
		unsigned int total, cls_s, cls_e, ps_s, ps_e;
		unsigned int spl_s, spl_e, rev_s, rev_e;
		total = jzfb.blw + jzfb.w + jzfb.elw + jzfb.hsw;
#if !defined(CONFIG_JZLCD_INNOLUX_AT080TN42)
		spl_s = 1;
		spl_e = spl_s + 1;
		cls_s = 0;
		cls_e = total - 60;	/* > 4us (pclk = 80ns) */
		ps_s = cls_s;
		ps_e = cls_e;
		rev_s = total - 40;	/* > 3us (pclk = 80ns) */
		rev_e = rev_s + total;
		jzfb.cfg |= STFT_PSHI;
#else           /*#if defined(CONFIG_JZLCD_INNOLUX_AT080TN42)*/
		spl_s = total - 5; /* LD */
		spl_e = total - 3;
		cls_s = 32;	/* CKV */
		cls_e = 145;
		ps_s  = 0;      /* OEV */
		ps_e  = 45;
		rev_s = 0;	/* POL */
		rev_e = 0;
#endif          /*#if defined(CONFIG_JZLCD_INNOLUX_AT080TN42)*/
		REG_LCD_SPL = (spl_s << 16) | spl_e;
		REG_LCD_CLS = (cls_s << 16) | cls_e;
		REG_LCD_PS = (ps_s << 16) | ps_e;
		REG_LCD_REV = (rev_s << 16) | rev_e;
		break;
	}
	case MODE_TFT_CASIO:
		break;
	}

	/* Configure the LCD panel */
	REG_LCD_CFG = jzfb.cfg;

	/* Timing setting */
	__cpm_stop_lcd();

	val = jzfb.fclk; /* frame clk */

	if ( (jzfb.cfg & MODE_MASK) != MODE_8BIT_SERIAL_TFT) {
		pclk = val * (jzfb.w + jzfb.hsw + jzfb.elw + jzfb.blw) *
			(jzfb.h + jzfb.vsw + jzfb.efw + jzfb.bfw); /* Pixclk */
	}
	else {
		/* serial mode: Hsync period = 3*Width_Pixel */
		pclk = val * (jzfb.w*3 + jzfb.hsw + jzfb.elw + jzfb.blw) *
			(jzfb.h + jzfb.vsw + jzfb.efw + jzfb.bfw); /* Pixclk */
	}

	if (((jzfb.cfg & MODE_MASK) == MODE_STN_COLOR_SINGLE) ||
	    ((jzfb.cfg & MODE_MASK) == MODE_STN_COLOR_DUAL))
		pclk = (pclk * 3);

	if (((jzfb.cfg & MODE_MASK) == MODE_STN_COLOR_SINGLE) ||
	    ((jzfb.cfg & MODE_MASK) == MODE_STN_COLOR_DUAL) ||
	    ((jzfb.cfg & MODE_MASK) == MODE_STN_MONO_SINGLE) ||
	    ((jzfb.cfg & MODE_MASK) == MODE_STN_MONO_DUAL))
		pclk = pclk >> ((jzfb.cfg & STN_DAT_PINMASK) >> 4);

	if (((jzfb.cfg & MODE_MASK) == MODE_STN_COLOR_DUAL) ||
	    ((jzfb.cfg & MODE_MASK) == MODE_STN_MONO_DUAL))
		pclk >>= 1;
#if defined(CONFIG_SOC_JZ4730)
	val = __cpm_get_pllout() / pclk;
	REG_CPM_CFCR2 = val - 1;
	val = __cpm_get_pllout() / (pclk * 4);
	val = __cpm_divisor_encode(val);
	__cpm_set_lcdclk_div(val);
	REG_CPM_CFCR |= CPM_CFCR_UPE;
#elif defined(CONFIG_SOC_JZ4740)
	val = ( __cpm_get_pllout2()) / pclk;
	val--;
	if ( val > 0x3ff ) {
		printk("pixel clock divid is too large, set it to 0x3ff\n");
		val = 0x3ff;
	}
	__cpm_set_pixdiv(val);

	val = pclk * 3 ;	/* LCDClock > 2.5*Pixclock */
	val =__cpm_get_pllout() / val;
	if ( val > 0x1f ) {
		printk("lcd clock divide is too large, set it to 0x1f\n");
		val = 0x1f;
	}
	__cpm_set_ldiv( val );
	REG_CPM_CPCCR |= CPM_CPCCR_CE ; /* update divide */

#else
	printk("drivers/video/Jzlcd.c, CONFIG_MIPS, please set chip type.\n");
#endif /*#ifdef CONFIG_MIPS_JZ4730 */

	jz_clocks.pixclk = __cpm_get_pixclk();
	jz_clocks.lcdclk = __cpm_get_lcdclk();
	printk("LCDC: PixClock:%d LcdClock:%d\n",
	       jz_clocks.pixclk, jz_clocks.lcdclk);

	__cpm_start_lcd();
	udelay(1000);
	return ret;
}

static irqreturn_t lcd_interrupt_handler(int irq, void *dev_id)
{
	unsigned int state;

	state = REG_LCD_STATE;

	if (state & LCD_STATE_EOF) /* End of frame */
		REG_LCD_STATE = state & ~LCD_STATE_EOF;

	if (state & LCD_STATE_IFU0) {
		dprintk("InFiFo0 underrun\n");
		REG_LCD_STATE = state & ~LCD_STATE_IFU0;
	}

	if (state & LCD_STATE_OFU) { /* Out fifo underrun */
		REG_LCD_STATE = state & ~LCD_STATE_OFU;
		dprintk("Out FiFo underrun.\n");
	}
	return IRQ_HANDLED;
}

#ifdef CONFIG_PM

/*
 * Suspend the LCDC.
 */
static int jzfb_suspend(void)
{
	__lcd_clr_ena(); /* Quick Disable */
	__lcd_display_off();
	__cpm_stop_lcd();

	return 0;
}

/*
 * Resume the LCDC.
 */
#ifdef CONFIG_SOC_JZ4730
static int jzfb_resume(void)
{
	__cpm_start_lcd();

	__lcd_display_pin_init();

	__lcd_display_on();

	lcd_hw_init();

	if (jzfb.bpp <= 8)
		REG_LCD_DA0 = virt_to_phys(lcd_palette_desc);
	else
		REG_LCD_DA0 = virt_to_phys(lcd_frame_desc0);

	if (((jzfb.cfg & MODE_MASK) == MODE_STN_COLOR_DUAL) ||
	    ((jzfb.cfg & MODE_MASK) == MODE_STN_MONO_DUAL))
		REG_LCD_DA1 = virt_to_phys(lcd_frame_desc1);

	__lcd_set_ena();
	return 0;
}

#else
/*
 * Resume the LCDC.
 */
static int jzfb_resume(void)
{
	__cpm_start_lcd();
	__gpio_set_pin(GPIO_DISP_OFF_N); 
	__lcd_special_on();
	__lcd_set_ena();
	mdelay(200);
	__lcd_set_backlight_level(80); 

	return 0;
}
#endif  /* CONFIG_MIPS_JZ4730 */

/*
 * Power management hook.  Note that we won't be called from IRQ context,
 * unlike the blank functions above, so we may sleep.
 */
static int jzlcd_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
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

static int __init jzfb_init(void)
{
	struct lcd_cfb_info *cfb;
	int err = 0;

	/* In special mode, we only need init special pin, 
	 * as general lcd pin has init in uboot */
#if defined(CONFIG_SOC_JZ4740) || defined(CONFIG_SOC_JZ4750)
	switch (jzfb.cfg & MODE_MASK) {
	case LCD_CFG_MODE_SPECIAL_TFT_1:
	case LCD_CFG_MODE_SPECIAL_TFT_2:
	case LCD_CFG_MODE_SPECIAL_TFT_3:
		__gpio_as_lcd_special();
		break;
	default:
		;
	}
#endif
	__lcd_display_pin_init();

	cfb = jzfb_alloc_fb_info();
	if (!cfb)
		goto failed;

	err = jzfb_map_smem(cfb);
	if (err)
		goto failed;

	jzfb_set_var(&cfb->fb.var, -1, &cfb->fb);

	lcd_descriptor_init();

	err = lcd_hw_init();
	if (err)
		goto failed;

	if (jzfb.bpp <= 8)
		REG_LCD_DA0 = virt_to_phys(lcd_palette_desc);
	else
		REG_LCD_DA0 = virt_to_phys(lcd_frame_desc0);

	if (((jzfb.cfg & MODE_MASK) == MODE_STN_COLOR_DUAL) ||
	    ((jzfb.cfg & MODE_MASK) == MODE_STN_MONO_DUAL))
		REG_LCD_DA1 = virt_to_phys(lcd_frame_desc1);

	__lcd_set_ena();

	if (request_irq(IRQ_LCD, lcd_interrupt_handler, IRQF_DISABLED,
			"lcd", 0)) {
		err = -EBUSY;
		goto failed;
	}

	__lcd_enable_ofu_intr(); /* enable OutFifo underrun */
//	__lcd_enable_ifu0_intr(); /* needn't enable InFifo underrun */

#if defined(CONFIG_JZLCD_FRAMEBUFFER_ROTATE_SUPPORT)
	jzfb_rotate_change(rotate_angle);
	/* sleep n??? */
#endif
	err = register_framebuffer(&cfb->fb);
	if (err < 0) {
		dprintk("jzfb_init(): register framebuffer err.\n");
		goto failed;
	}

	printk("fb%d: %s frame buffer device, using %dK of video memory\n",
	       cfb->fb.node, cfb->fb.fix.id, cfb->fb.fix.smem_len>>10);

#ifdef CONFIG_PM
	/*
	 * Note that the console registers this as well, but we want to
	 * power down the display prior to sleeping.
	 */
	cfb->pm = pm_register(PM_SYS_DEV, PM_SYS_VGA, jzlcd_pm_callback);
	if (cfb->pm)
		cfb->pm->data = cfb;
#endif

	__lcd_display_on();

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
	.name		= "jz-lcd",
	.bus 		= &platform_bus_type,
	.probe		= jzfb_probe,
        .remove		= jzfb_remove,
	.suspend	= jzfb_suspend,
        .resume		= jzfb_resume,
};
#endif

static void __exit jzfb_cleanup(void)
{
#if defined(CONFIG_JZLCD_FRAMEBUFFER_ROTATE_SUPPORT)
	kthread_stop(jzlcd_info->rotate_daemon_thread);
#endif
//	driver_unregister(&jzfb_driver);
//	jzfb_remove();
}

module_init(jzfb_init);
module_exit(jzfb_cleanup);

MODULE_DESCRIPTION("JzSOC LCD Controller driver");
MODULE_LICENSE("GPL");
