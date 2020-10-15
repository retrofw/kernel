/*
 * linux/drivers/video/jz4760_epd.c -- Ingenic Jz4760 EPD frame buffer device
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
#include <linux/suspend.h>
#include <linux/pm.h>
#include <linux/leds.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/jzsoc.h>

#include "jz4760_epd.h"

#define DRIVER_NAME	"jz-lcd"

#define LCD_DEBUG
//#undef LCD_DEBUG

#ifdef LCD_DEBUG
#define dprintk(x...)	printk(x)
#define print_dbg(f, arg...) printk("dbg::" __FILE__ ",LINE(%d): " f "\n", __LINE__, ## arg)
#else
#define dprintk(x...)
#define print_dbg(f, arg...) do {} while (0)
#endif

#define print_err(f, arg...) printk(KERN_ERR DRIVER_NAME ": " f "\n", ## arg)
#define print_warn(f, arg...) printk(KERN_WARNING DRIVER_NAME ": " f "\n", ## arg)
#define print_info(f, arg...) printk(KERN_INFO DRIVER_NAME ": " f "\n", ## arg)

#define INTTRUPT_PRINT 0
#if INTTRUPT_PRINT
#define print(x,arg...)	printk(x, ##arg)
#else
#define print(x,arg...)
#endif

#define JZ_LCD_ID "jz-lcd"


static unsigned char *wfm;
extern int msc_read(unsigned long start_byte, u8 *dst, size_t len);

enum {
	MODE_INIT,	/* initialization */
	MODE_DU,	/* white/black level */ 
	MODE_GC,	/* gray level */
	MODE_GC4
};
int epd_mode;
int partial = 0;

enum {
	TEMP_0,		/* temperature 0: 0 */
	TEMP_1,		/* temperature 1: 10 */ 
	TEMP_2,		/* temperature 2: 25 */ 
	TEMP_3,		/* temperature 3: 40 */
	TEMP_4,
	TEMP_5,
	TEMP_6,
	TEMP_7,
	TEMP_8,
	TEMP_9,
	TEMP_10,
	TEMP_11,
	TEMP_12,
	TEMP_13,
	TEMP_14,
	TEMP_15
};
int epd_temp;

static int epd_panel;

struct epd_cfb_info {
	struct fb_info		fb0;	/* foreground 0 */
	struct fb_info		fb1;	/* foreground 1 */
#ifdef CONFIG_PM
	struct pm_dev		*pm;
#endif
};

#define NR_DMA_DESC_FG1 1
#define NR_DESC_PER_GROUP 97 /* 96 frames + 1 palette */
#define NR_GROUP_NUM 2  /* we should surport 12 palettes, 16 frames per palette, so we can support 12*16=192 frames */
#define NR_DMA_DESC_FG0 NR_DESC_PER_GROUP * NR_GROUP_NUM
//static unsigned int palette_offset = 96 * 16 * 4; /* (Bytes) 16 words per frame, 96 frames per palette */

static struct epd_cfb_info *jz4760fb_info;
static struct jz4760_lcd_dma_desc *dma_desc_base;
static struct jz4760_lcd_dma_desc *dma_desc_pal0, *dma0_desc, *dma1_desc;

unsigned char *epd_palette;
static unsigned char *epd_frame0;
static unsigned char *epd_frame1;
static unsigned char *epd_frame0_tmp;
static unsigned char *epd_frame1_tmp;

static int width = 800, height = 600, bpp = 4;
static int max_frm;

static int get_temp_sensor(void)
{
	return 0;
}

void get_temp(void) 
{
	int temp;

	temp = get_temp_sensor(); 

	if (0 <= temp && temp < 2)
		epd_temp = TEMP_0;
	else if (2 <= temp && temp < 5)
		epd_temp = TEMP_1;
	else if (5 <= temp && temp < 8)
		epd_temp = TEMP_2;
	else if (8 <= temp && temp < 11)
		epd_temp = TEMP_3;
	else if (11 <= temp && temp < 14)
		epd_temp = TEMP_4;
	else if (14 <= temp && temp < 17)
		epd_temp = TEMP_5;
	else if (17 <= temp && temp < 20)
		epd_temp = TEMP_6;
	else if (20 <= temp && temp < 23)
		epd_temp = TEMP_7;
	else if (23 <= temp && temp < 28)
		epd_temp = TEMP_8;
	else if (28 <= temp && temp < 33)
		epd_temp = TEMP_9;
	else if (33 <= temp && temp < 38)
		epd_temp = TEMP_10;
	else if (38 <= temp && temp < 50)
		epd_temp = TEMP_11;
	else
		printk("The temperature should be 0 ~ 50 degree centigrade\n"); 
}

static void print_palette(void *palette, int len)
{
	unsigned short *pal = (unsigned short *)palette;
	int i;
	printk("palette:\n");
	for (i = 0; i < len; i++) {
		if ((i * 8) % 256 == 0)
			printk("\nfrm%d:\t", i * 8 / 256);
		if (i % 16 == 0)
			printk("\n\t");
		printk("%04x ", pal[i]);
	}
	printk("\n");
}

static unsigned char * fill_waveform(unsigned int * p_max_frm, int mode)
{
	unsigned int waveform_addr, mode_addr;
	unsigned char *p = NULL;
	unsigned int pos = 512;

	if(epd_panel == 0){
		waveform_addr = (wfm[pos / 2 + 64 * mode + 4 * (epd_temp * 5) + 2] << 16) |
				(wfm[pos / 2 + 64 * mode + 4 * (epd_temp * 5) + 1] << 8) |
				(wfm[pos / 2 + 64 * mode + 4 * (epd_temp * 5)]);
		*p_max_frm = wfm[pos / 2 + 64 * mode + 4 * (epd_temp * 5) + 3];

		p = &wfm[waveform_addr];

		//printk("epd_mode = %d, epd_temp = %d\n", epd_mode, epd_temp);
		//printk("waveform_addr = 0x%x\n", waveform_addr);
		//printk("max_frm = %d\n", *p_max_frm);
	}else if(epd_panel == 1){
		waveform_addr = wfm[pos + 0x30 + 5 * epd_temp + 1] << 24 | 
				wfm[pos + 0x30 + 5 * epd_temp + 2] << 16 | 
				wfm[pos + 0x30 + 5 * epd_temp + 3] << 8 | 
				wfm[pos + 0x30 + 5 * epd_temp + 4];
		*p_max_frm = wfm[pos + waveform_addr + 6 * mode] << 8 | wfm[pos + waveform_addr + 6 * mode + 1];
		mode_addr = wfm[pos + waveform_addr + 6 * mode + 2] << 24 | 
				wfm[pos + waveform_addr + 6 * mode + 3] << 16 | 
				wfm[pos + waveform_addr + 6 * mode + 4] << 8 | 
				wfm[pos + waveform_addr + 6 * mode + 5];
		p = &wfm[pos + waveform_addr + mode_addr];

		//printk("epd_mode = %d, epd_temp = %d\n", epd_mode, epd_temp);	
		//printk("waveform_addr = 0x%x\n", waveform_addr);
		//printk("mode_addr = 0x%x\n", mode_addr);
		//printk("max_frm = %d\n", *p_max_frm);
	}

	return p;
}

static void fill_16level_gray(int max_frm, unsigned char *p)
{
	int i, j, k, offset, bits;
	int index_per_frame = 256 / 4;
	unsigned char *ptr = (unsigned char *)epd_palette;
	unsigned char pix[16] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7,
				 0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf};

	if(epd_mode == MODE_INIT)
		max_frm /= 2;

	for(k = 0; k < max_frm; k++) {
		for(i = 0; i < 16; i++){	/* old pixel value */
			for(j = 0; j < 16; j++){	/* new pixel value */
				offset = (pix[i] << 4 | pix[j]) / 4;
				bits = (pix[i] << 4 | pix[j]) % 4 * 2;
				if(epd_panel == 0)
					ptr[k * index_per_frame + offset] |= 
						((*(p + k * 64 + j * 4 + i / 4) >> (2 * (i % 4))) & 0x3) << bits;
				else if(epd_panel == 1)
					ptr[k * index_per_frame + offset] |= 
						((*(p + k * 64 + j * 4 + i / 4) >> (6 - 2 * (i % 4))) & 0x3) << bits; 
			}
		}
#if 0
		if(partial){
			ptr[k * index_per_frame + 0] &= ~(3 << 0);
			ptr[k * index_per_frame + 4] &= ~(3 << 2);
			ptr[k * index_per_frame + 8] &= ~(3 << 4);
			ptr[k * index_per_frame + 12] &= ~(3 << 6);
			ptr[k * index_per_frame + 17] &= ~(3 << 0);
			ptr[k * index_per_frame + 21] &= ~(3 << 2);
			ptr[k * index_per_frame + 25] &= ~(3 << 4);
			ptr[k * index_per_frame + 29] &= ~(3 << 6);
			ptr[k * index_per_frame + 34] &= ~(3 << 0);
			ptr[k * index_per_frame + 38] &= ~(3 << 2);
			ptr[k * index_per_frame + 42] &= ~(3 << 4);
			ptr[k * index_per_frame + 46] &= ~(3 << 6);
			ptr[k * index_per_frame + 51] &= ~(3 << 0);
			ptr[k * index_per_frame + 55] &= ~(3 << 2);
			ptr[k * index_per_frame + 59] &= ~(3 << 4);
			ptr[k * index_per_frame + 63] &= ~(3 << 6);
		}
#endif
	}
	partial = 0;
}

static void fill_epd_palette(void)
{
	unsigned char *p;

	memset(epd_palette, 0x00, 4096 * 2);

	if(epd_mode == MODE_INIT)
		p = fill_waveform(&max_frm, MODE_INIT);
	else if(epd_mode == MODE_DU)
		p = fill_waveform(&max_frm, MODE_DU);
	else if(epd_mode == MODE_GC)
		p = fill_waveform(&max_frm, MODE_GC);
	else
		p = fill_waveform(&max_frm, MODE_GC4);
		
	fill_16level_gray(max_frm,p);

	dma_cache_wback((unsigned int)(epd_palette), 4096 * 2);
	//print_palette((unsigned short *)((unsigned int)epd_palette | 0xa0000000), 2048);
}

static void fill_init_palette(void)
{
	int i, j, bit2;
	int index_per_frame = 256 / 4;
	unsigned char *ptr = (unsigned char *)epd_palette; 
	unsigned char *p;
	
	memset(epd_palette, 0x00, 4096 * 2);

	p = fill_waveform(&max_frm, MODE_INIT);
	max_frm = max_frm / 2;

	for (j = 0; j < max_frm; j++) {
		for(i = 0;i < index_per_frame; i++){
			bit2 = (*(p + j / 4) >> (6 - (j % 4) * 2)) & 0x3;
			ptr[j * index_per_frame + i] = bit2 << 6 | bit2 << 4 | bit2 << 2 | bit2;
		}
	}
		
	dma_cache_wback((unsigned int)(epd_palette), 4096 * 2);
	//print_palette((unsigned short *)((unsigned int)epd_palette | 0xa0000000), 4096);
}

static int jz4760fb0_set_mode(unsigned int mode)
{
	if(mode & 0x7f == 1)
		epd_mode = MODE_DU;
	else if(mode & 0x7f == 2)
		epd_mode = MODE_GC;
	else if(mode & 0x7f == 3)
		epd_mode = MODE_GC4;

	partial = mode & 0x80;

	fill_epd_palette();
}

static int jz4760fb0_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	//printk("%s: cmd = 0x%x\n", __func__, cmd);
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case GET_EPD_INFO:
	{
		struct epd_info {
			void * frame_index_buffer;
			unsigned long frame_index_buffer_phys;
			unsigned long frame_index_buffer_size;
			void * frame_buffer_old;
			unsigned long frame_buffer_old_phys;
			unsigned long frame_buffer_old_size;
			void * frame_buffer_new;
			unsigned long frame_buffer_new_phys;
			unsigned long frame_buffer_new_size;
		}epd;
		epd.frame_index_buffer = epd_palette;
		epd.frame_index_buffer_phys = virt_to_phys(epd_palette);
		epd.frame_index_buffer_size = 4096 * 2;

		epd.frame_buffer_old = epd_frame1;
		epd.frame_buffer_old_phys = virt_to_phys(epd_frame1_tmp);		
		epd.frame_buffer_old_size = width * height / 2;
		
		epd.frame_buffer_new = epd_frame0;
		epd.frame_buffer_new_phys = virt_to_phys(epd_frame0_tmp);		
		epd.frame_buffer_new_size = width * height / 2;
		
		copy_to_user((void __user *)arg, &epd, sizeof(epd));
		break;
	}
	case START_EPD_TRANS:
	{
		memcpy(epd_frame0, epd_frame0_tmp, width * height / 2);
		memcpy(epd_frame1, epd_frame1_tmp, width * height / 2);
		dma_cache_wback(epd_frame0, width * height / 2);
		dma_cache_wback(epd_frame1, width * height / 2);
		REG_EPD_CTRL |= EPD_CTRL_PWRON;

		break;
	}
	case LCD_DISABLE:
	{
		REG_LCD_CTRL &= ~LCD_CTRL_ENA;
		break;
	}
	case SET_EPD_MODE:
	{
		unsigned int mode;
		copy_from_user((void *)&mode, (void *)argp, sizeof(unsigned int));
		jz4760fb0_set_mode(mode);
		break;
	}
	case SET_GRAY_LEVEL:
	{
		break;
	}
	default:
		printk("FG0:%s, unknown command(0x%x)", __FILE__, cmd);
		break;
	}

	return ret;
}

static struct fb_ops jz4760fb_ops = {
	.owner			= THIS_MODULE,
	.fb_fillrect	= cfb_fillrect,
	.fb_imageblit	= cfb_imageblit,
	.fb_ioctl		= jz4760fb0_ioctl,
};

static int jz4760fb_set_var(struct fb_var_screeninfo *var,
			struct fb_info *info)
{
	struct fb_info *fb = info;

	var->height	            = height;
	var->width	            = width;
	var->bits_per_pixel	    = bpp;
	
	var->vmode              = FB_VMODE_NONINTERLACED;
	var->activate           = fb->var.activate;
	var->xres               = var->width;
	var->yres               = var->height;
	var->xres_virtual       = var->width;
	var->yres_virtual       = var->height;
	var->xoffset            = 0;
	var->yoffset            = 0;
	var->pixclock           = KHZ2PICOS(jz_clocks.pixclk/1000);

	return 0;
}

static struct epd_cfb_info * jz4760fb_alloc_fb_info(void)
{
	struct epd_cfb_info *cfb;
	cfb = kmalloc(sizeof(struct epd_cfb_info) + sizeof(u32) * 16, GFP_KERNEL);

	if (!cfb)
		return NULL;

	jz4760fb_info = cfb;

	memset(cfb, 0, sizeof(struct epd_cfb_info) );

	strcpy(cfb->fb0.fix.id, "jzepd-fg0");
	cfb->fb0.fbops		= &jz4760fb_ops;

	strcpy(cfb->fb1.fix.id, "jzepd-fg1");
	cfb->fb1.fbops		= &jz4760fb_ops;

	return cfb;
}

static int jz4760fb_map_smem(struct epd_cfb_info *cfb)
{
	unsigned long page;
	unsigned int page_shift, needroom = 0, needroom1 = 0;
	unsigned char *fb_palette, *fb_frame, *fb_frame_tmp;

    /* caculate the mem size of Foreground 0 */
	needroom1 = needroom = ((width * bpp + 7) >> 3) * height;

	/* caculate the mem size of Foreground 1 */
	needroom += ((width * bpp + 7) >> 3) * height;

	needroom += PAGE_SIZE;

	/* Alloc memory */
	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;

	fb_palette = (unsigned char *)__get_free_pages(GFP_KERNEL, 1);
	fb_frame = (unsigned char *)__get_free_pages(GFP_KERNEL, page_shift);
	fb_frame_tmp = (unsigned char *)__get_free_pages(GFP_KERNEL, page_shift);
	if ((!fb_palette) || (!fb_frame) || (!fb_frame_tmp))
		return -ENOMEM;
	memset((void *)fb_palette, 0, PAGE_SIZE);
 	memset((void *)fb_frame, 0, PAGE_SIZE << page_shift);
 	memset((void *)fb_frame_tmp, 0, PAGE_SIZE << page_shift);

	epd_palette = fb_palette;
	dma_desc_base = (struct jz4760_lcd_dma_desc *)__get_free_pages(GFP_KERNEL, 0);

	/*
	 * Set page reserved so that mmap will work. This is necessary
	 * since we'll be remapping normal memory.
	 */
	page = (unsigned long)epd_palette;
	SetPageReserved(virt_to_page((void*)page));

	page = (unsigned long)dma_desc_base;
	SetPageReserved(virt_to_page((void*)page));

	for (page = (unsigned long)fb_frame;
	     page < PAGE_ALIGN((unsigned long)fb_frame + (PAGE_SIZE << page_shift));
	     page += PAGE_SIZE) {
		SetPageReserved(virt_to_page((void*)page));
	}

	epd_frame0 = fb_frame;
	cfb->fb0.fix.smem_start = virt_to_phys((void *)epd_frame0);
	cfb->fb0.fix.smem_len = needroom1;
	cfb->fb0.screen_base =
		(unsigned char *)(((unsigned int)epd_frame0 & 0x1fffffff) | 0xa0000000);
	if (!cfb->fb0.screen_base) {
		printk("jz4760fb0, %s: unable to map screen memory\n", cfb->fb0.fix.id);
		return -ENOMEM;
	}
	memset((void *)epd_frame0, 0xff, width * height / 2);

	epd_frame1 = (unsigned char *)(((unsigned int)fb_frame + needroom1 + PAGE_SIZE) & PAGE_MASK);
	cfb->fb1.fix.smem_start = virt_to_phys((void *)epd_frame1);
	cfb->fb1.fix.smem_len = needroom1;
	cfb->fb1.screen_base =
		(unsigned char *)(((unsigned int)epd_frame1 & 0x1fffffff) | 0xa0000000);
	if (!cfb->fb1.screen_base) {
		printk("jz4760fb, %s: unable to map screen memory\n", cfb->fb1.fix.id);
		return -ENOMEM;
	}
	memset((void *)epd_frame1, 0xff, width * height / 2);

	printk("epd_frame0 = 0x%x, epd_frame1 = 0x%x\n", epd_frame0, epd_frame1);

	for (page = (unsigned long)fb_frame_tmp;
	     page < PAGE_ALIGN((unsigned long)fb_frame_tmp + (PAGE_SIZE << page_shift));
	     page += PAGE_SIZE) {
		SetPageReserved(virt_to_page((void*)page));
	}

	epd_frame0_tmp = fb_frame_tmp;
	memset((void *)epd_frame0_tmp, 0xff, width * height / 2);

	epd_frame1_tmp = (unsigned char *)(((unsigned int)fb_frame_tmp + needroom1 + PAGE_SIZE) & PAGE_MASK);
	memset((void *)epd_frame1_tmp, 0xff, width * height / 2);

	printk("epd_frame0_tmp = 0x%x, epd_frame1_tmp = 0x%x\n", epd_frame0_tmp, epd_frame1_tmp);

	wfm = (unsigned char *)__get_free_pages(GFP_KERNEL, 8);
	if (!wfm)
		return -ENOMEM;
	memset((void *)wfm, 0, PAGE_SIZE << 8);
	
	return 0;
}

static void jz4760fb_unmap_smem(struct epd_cfb_info *cfb)
{
	struct page * map = NULL;
	unsigned char *tmp;
	unsigned int page_shift, needroom;

	needroom = ((width * bpp + 7) >> 3) * height + PAGE_SIZE;

	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;

	if (cfb && cfb->fb0.screen_base) {
		iounmap(cfb->fb0.screen_base);
		cfb->fb0.screen_base = NULL;
		release_mem_region(cfb->fb0.fix.smem_start,
				   cfb->fb0.fix.smem_len);
	}

	if (cfb && cfb->fb1.screen_base) {
		iounmap(cfb->fb1.screen_base);
		cfb->fb1.screen_base = NULL;
		release_mem_region(cfb->fb1.fix.smem_start,
				   cfb->fb1.fix.smem_len);
	}
	
	if (dma_desc_base) {
		map = virt_to_page(dma_desc_base);
		clear_bit(PG_reserved, &map->flags);
		free_pages((int)dma_desc_base, 2);
	}

	if (epd_frame0) {
		for (tmp=(unsigned char *)epd_frame0; 
		     tmp < epd_frame0 + (PAGE_SIZE << page_shift); 
		     tmp += PAGE_SIZE) {
			map = virt_to_page(tmp);
			clear_bit(PG_reserved, &map->flags);
		}
		free_pages((int)epd_frame0, page_shift);
	}

	if (wfm) {
		free_pages((int)wfm, 8);
	}
}

static void jz4760fb_free_fb_info(struct epd_cfb_info *cfb)
{
	if (cfb) {
		kfree(cfb);
	}
}

static void print_lcdc_registers(void)	/* debug */
{
#if 1//def  DEBUG
	/* LCD Controller Resgisters */
	printk("REG_LCD_CFG:\t0x%08x\n", REG_LCD_CFG);
	printk("REG_LCD_CTRL:\t0x%08x\n", REG_LCD_CTRL);
	printk("REG_LCD_STATE:\t0x%08x\n", REG_LCD_STATE);
	printk("REG_LCD_OSDC:\t0x%08x\n", REG_LCD_OSDC);
	printk("REG_LCD_OSDCTRL:\t0x%08x\n", REG_LCD_OSDCTRL);
	printk("REG_LCD_OSDS:\t0x%08x\n", REG_LCD_OSDS);
	printk("REG_LCD_BGC:\t0x%08x\n", REG_LCD_BGC);
	printk("REG_LCD_KEK0:\t0x%08x\n", REG_LCD_KEY0);
	printk("REG_LCD_KEY1:\t0x%08x\n", REG_LCD_KEY1);
	printk("REG_LCD_ALPHA:\t0x%08x\n", REG_LCD_ALPHA);
	printk("REG_LCD_IPUR:\t0x%08x\n", REG_LCD_IPUR);
	printk("REG_LCD_VAT:\t0x%08x\n", REG_LCD_VAT);
	printk("REG_LCD_DAH:\t0x%08x\n", REG_LCD_DAH);
	printk("REG_LCD_DAV:\t0x%08x\n", REG_LCD_DAV);
	printk("REG_LCD_XYP0:\t0x%08x\n", REG_LCD_XYP0);
	printk("REG_LCD_XYP1:\t0x%08x\n", REG_LCD_XYP1);
	printk("REG_LCD_SIZE0:\t0x%08x\n", REG_LCD_SIZE0);
	printk("REG_LCD_SIZE1:\t0x%08x\n", REG_LCD_SIZE1);
	printk("REG_LCD_RGBC\t0x%08x\n", REG_LCD_RGBC);
	printk("REG_LCD_VSYNC:\t0x%08x\n", REG_LCD_VSYNC);
	printk("REG_LCD_HSYNC:\t0x%08x\n", REG_LCD_HSYNC);
	printk("REG_LCD_PS:\t0x%08x\n", REG_LCD_PS);
	printk("REG_LCD_CLS:\t0x%08x\n", REG_LCD_CLS);
	printk("REG_LCD_SPL:\t0x%08x\n", REG_LCD_SPL);
	printk("REG_LCD_REV:\t0x%08x\n", REG_LCD_REV);
	printk("REG_LCD_IID:\t0x%08x\n", REG_LCD_IID);
	printk("REG_LCD_DA0:\t0x%08x\n", REG_LCD_DA0);
	printk("REG_LCD_SA0:\t0x%08x\n", REG_LCD_SA0);
	printk("REG_LCD_FID0:\t0x%08x\n", REG_LCD_FID0);
	printk("REG_LCD_CMD0:\t0x%08x\n", REG_LCD_CMD0);
	printk("REG_LCD_OFFS0:\t0x%08x\n", REG_LCD_OFFS0);
	printk("REG_LCD_PW0:\t0x%08x\n", REG_LCD_PW0);
	printk("REG_LCD_CNUM0:\t0x%08x\n", REG_LCD_CNUM0);
	printk("REG_LCD_DESSIZE0:\t0x%08x\n", REG_LCD_DESSIZE0);
	printk("REG_LCD_DA1:\t0x%08x\n", REG_LCD_DA1);
	printk("REG_LCD_SA1:\t0x%08x\n", REG_LCD_SA1);
	printk("REG_LCD_FID1:\t0x%08x\n", REG_LCD_FID1);
	printk("REG_LCD_CMD1:\t0x%08x\n", REG_LCD_CMD1);
	printk("REG_LCD_OFFS1:\t0x%08x\n", REG_LCD_OFFS1);
	printk("REG_LCD_PW1:\t0x%08x\n", REG_LCD_PW1);
	printk("REG_LCD_CNUM1:\t0x%08x\n", REG_LCD_CNUM1);
	printk("REG_LCD_DESSIZE1:\t0x%08x\n", REG_LCD_DESSIZE1);
	printk("==================================\n");

	printk("REG_SLCD_CFG:\t0x%08x\n", REG_SLCD_CFG);
	printk("REG_SLCD_CTRL:\t0x%08x\n", REG_SLCD_CTRL);
	printk("REG_SLCD_STATE:\t0x%08x\n", REG_SLCD_STATE);
	printk("==================================\n");

	printk("REG_EPD_CTRL:\t0x%08x\n", REG_EPD_CTRL);
	printk("REG_EPD_STA:\t0x%08x\n", REG_EPD_STA);
	printk("REG_EPD_ISR:\t0x%08x\n", REG_EPD_ISR);
	printk("REG_EPD_CFG0:\t0x%08x\n", REG_EPD_CFG0);
	printk("REG_EPD_CFG1:\t0x%08x\n", REG_EPD_CFG1);
	printk("REG_EPD_PPL0:\t0x%08x\n", REG_EPD_PPL0);
	printk("REG_EPD_PPL1:\t0x%08x\n", REG_EPD_PPL1);
	printk("REG_EPD_VAT:\t0x%08x\n", REG_EPD_VAT);
	printk("REG_EPD_DAV:\t0x%08x\n", REG_EPD_DAV);
	printk("REG_EPD_DAH:\t0x%08x\n", REG_EPD_DAH);
	printk("REG_EPD_VSYN:\t0x%08x\n", REG_EPD_VSYNC);
	printk("REG_EPD_HSYN:\t0x%08x\n", REG_EPD_HSYNC);
	printk("REG_EPD_GDCLK:\t0x%08x\n", REG_EPD_GDCLK);
	printk("REG_EPD_GDOE:\t0x%08x\n", REG_EPD_GDOE);
	printk("REG_EPD_GDSP:\t0x%08x\n", REG_EPD_GDSP);
	printk("REG_EPD_SDOE:\t0x%08x\n", REG_EPD_SDOE);
	printk("REG_EPD_SDSP:\t0x%08x\n", REG_EPD_SDSP);
	printk("REG_EPD_PMGR0:\t0x%08x\n", REG_EPD_PMGR0);
	printk("REG_EPD_PMGR1:\t0x%08x\n", REG_EPD_PMGR1);
	printk("REG_EPD_PMGR2:\t0x%08x\n", REG_EPD_PMGR2);
	printk("REG_EPD_PMGR3:\t0x%08x\n", REG_EPD_PMGR3);
	printk("REG_EPD_PMGR4:\t0x%08x\n", REG_EPD_PMGR4);
	printk("REG_EPD_VCOM0:\t0x%08x\n", REG_EPD_VCOM0);
	printk("REG_EPD_VCOM1:\t0x%08x\n", REG_EPD_VCOM1);
	printk("REG_EPD_VCOM2:\t0x%08x\n", REG_EPD_VCOM2);
	printk("REG_EPD_VCOM3:\t0x%08x\n", REG_EPD_VCOM3);
	printk("REG_EPD_VCOM4:\t0x%08x\n", REG_EPD_VCOM4);
	printk("REG_EPD_VCOM5:\t0x%08x\n", REG_EPD_VCOM5);
	printk("REG_EPD_BORDR:\t0x%08x\n", REG_EPD_BORDR);
	printk("REG_EPD_PPL0_POS:\t0x%08x\n", REG_EPD_PPL0_POS);
	printk("REG_EPD_PPL0_SIZE:\t0x%08x\n", REG_EPD_PPL0_SIZE);
	printk("REG_EPD_PPL1_POS:\t0x%08x\n", REG_EPD_PPL1_POS);
	printk("REG_EPD_PPL1_SIZE:\t0x%08x\n", REG_EPD_PPL1_SIZE);
	printk("REG_EPD_PPL2_POS:\t0x%08x\n", REG_EPD_PPL2_POS);
	printk("REG_EPD_PPL2_SIZE:\t0x%08x\n", REG_EPD_PPL2_SIZE);
	printk("REG_EPD_PPL3_POS:\t0x%08x\n", REG_EPD_PPL3_POS);
	printk("REG_EPD_PPL3_SIZE:\t0x%08x\n", REG_EPD_PPL3_SIZE);
	printk("REG_EPD_PPL4_POS:\t0x%08x\n", REG_EPD_PPL4_POS);
	printk("REG_EPD_PPL4_SIZE:\t0x%08x\n", REG_EPD_PPL4_SIZE);
	printk("REG_EPD_PPL5_POS:\t0x%08x\n", REG_EPD_PPL5_POS);
	printk("REG_EPD_PPL5_SIZE:\t0x%08x\n", REG_EPD_PPL5_SIZE);
	printk("REG_EPD_PPL6_POS:\t0x%08x\n", REG_EPD_PPL6_POS);
	printk("REG_EPD_PPL6_SIZE:\t0x%08x\n", REG_EPD_PPL6_SIZE);
	printk("REG_EPD_PPL7_POS:\t0x%08x\n", REG_EPD_PPL7_POS);
	printk("REG_EPD_PPL7_SIZE:\t0x%08x\n", REG_EPD_PPL7_SIZE);
	printk("==================================\n");
#endif
}

static void jz4760fb_descriptor_init(void)
{
	unsigned int pal_size;
	int fg_line_size, fg_frm_size, size;

	pal_size = 96 * 16 * 4 / 4;
	
	/* DMA0 Descriptor */
	dma_desc_pal0 = dma_desc_base + 0;
	dma0_desc = dma_desc_base + 1;
	
	size = height << 16 | width;
	fg_line_size = (width * bpp / 8);
	fg_line_size = ((fg_line_size + 3) >> 2) << 2; /* word aligned */
	fg_frm_size = fg_line_size * height;
	
	dma_desc_pal0->next_desc = (unsigned int)virt_to_phys(dma0_desc);
	dma_desc_pal0->databuf 	 = (unsigned int)virt_to_phys((void *)epd_palette);
	dma_desc_pal0->frame_id  = (unsigned int)0xaaaaaaaa;
	dma_desc_pal0->cmd 	 = LCD_CMD_PAL | pal_size; /* Palette Descriptor */

	dma0_desc->next_desc = (unsigned int)virt_to_phys(dma0_desc);
	dma0_desc->databuf    = virt_to_phys((void *)epd_frame0);
	dma0_desc->frame_id   = (unsigned int)0x0000da00; 
	dma0_desc->cmd        = LCD_CMD_EOFINT | fg_frm_size / 4;	
	dma0_desc->offsize    = 0;
	dma0_desc->page_width = 0;
	dma0_desc->desc_size  = size;

	REG_LCD_DA0 = virt_to_phys(dma_desc_pal0);
	REG_LCD_SIZE0 = size;

	/* DMA1 Descriptor */
	dma1_desc = dma_desc_base + 2;
	
	dma1_desc->next_desc  = (unsigned int)virt_to_phys(dma1_desc);
	dma1_desc->databuf    = virt_to_phys((void *)epd_frame1);
	dma1_desc->frame_id   = (unsigned int)0x0da10000; 
	dma1_desc->cmd        = LCD_CMD_EOFINT | fg_frm_size / 4;
	dma1_desc->offsize    = 0;
	dma1_desc->page_width = 0;
	dma1_desc->desc_size  = size;

	REG_LCD_DA1 = virt_to_phys(dma1_desc);
	REG_LCD_SIZE1 = size;

	dma_cache_wback((unsigned int)(dma_desc_base), 3 * sizeof(struct jz4760_lcd_dma_desc));
}

static void jz4760fb_change_clock(void)
{
	unsigned int val;
	unsigned int pclk;
	/* Timing setting */
	__cpm_stop_lcd();

	if(epd_panel == 0)
		pclk = 18570000;
	else
		pclk = 16000000;

	val = __cpm_get_pllout2() / pclk;
	val--;
	dprintk("ratio: val = %d\n", val);
	if ( val > 0x7ff ) {
		printk("pixel clock divid is too large, set it to 0x7ff\n");
		val = 0x7ff;
	}

	__cpm_set_pixdiv(val);
	__cpm_enable_pll_change();

	jz_clocks.pixclk = __cpm_get_pixclk();
	printk("LCDC: PixClock:%d\n", jz_clocks.pixclk);
	
	__cpm_start_lcd();
	udelay(1000);
}

static void jz4760fb_set_mode(void)
{
	struct epd_cfb_info *cfb = jz4760fb_info;

	jz4760fb_set_var(&cfb->fb0.var, &cfb->fb0);
	jz4760fb_set_var(&cfb->fb1.var, &cfb->fb1);
}

static void init_epd_controller(void)
{
	if(epd_panel == 1){
		REG_EPD_VAT  = 0x268 << EPD_VAT_VT_BIT | 0x102 << EPD_VAT_HT_BIT;
		REG_EPD_DAH  = 0xd6 << EPD_DAH_HDE_BIT | 0xe << EPD_DAH_HDS_BIT;
		REG_EPD_DAV  = 0x25c << EPD_DAV_VDE_BIT | 0x4 << EPD_DAV_VDS_BIT;
		REG_EPD_VSYNC = 0x4 << EPD_VSYNC_VPE_BIT | 0x0 << EPD_VSYNC_VPS_BIT;
		REG_EPD_HSYNC = 0xa << EPD_HSYNC_HPE_BIT | 0x0 << EPD_HSYNC_HPS_BIT;	
		REG_EPD_GDCLK = 0xd7 << EPD_GDCLK_DIS_BIT | 0x0 << EPD_GDCLK_ENA_BIT;
		REG_EPD_GDOE  = 0x267 << EPD_GDOE_DIS_BIT | 0x0 << EPD_GDOE_ENA_BIT;
		REG_EPD_GDSP  = 0x6b << EPD_GDSP_DIS_BIT | 0x6b << EPD_GDSP_ENA_BIT;
		REG_EPD_SDOE  = 0xff << EPD_SDOE_DIS_BIT | 0xd << EPD_SDOE_ENA_BIT;
		REG_EPD_SDSP  = 0x5d << EPD_SDSP_DIS_BIT | 0x5c << EPD_SDSP_ENA_BIT;
	}else{
		REG_EPD_VAT  = 0x26b << EPD_VAT_VT_BIT | 0x12c << EPD_VAT_HT_BIT;
		REG_EPD_DAH  = 0xd6 << EPD_DAH_HDE_BIT | 0xe << EPD_DAH_HDS_BIT;
		REG_EPD_DAV  = 0x260 << EPD_DAV_VDE_BIT | 0x8 << EPD_DAV_VDS_BIT;
		REG_EPD_VSYNC = 0x4 << EPD_VSYNC_VPE_BIT | 0x0 << EPD_VSYNC_VPS_BIT;
		REG_EPD_HSYNC = 0xa << EPD_HSYNC_HPE_BIT | 0x0 << EPD_HSYNC_HPS_BIT;	
		REG_EPD_GDCLK = 0xda << EPD_GDCLK_DIS_BIT | 0x0 << EPD_GDCLK_ENA_BIT;
		REG_EPD_GDOE  = 0x267 << EPD_GDOE_DIS_BIT | 0x0 << EPD_GDOE_ENA_BIT;
		REG_EPD_GDSP  = 0x6b << EPD_GDSP_DIS_BIT | 0x6b << EPD_GDSP_ENA_BIT;
		REG_EPD_SDOE  = 0x11d << EPD_SDOE_DIS_BIT | 0xc << EPD_SDOE_ENA_BIT;
		REG_EPD_SDSP  = 0x5d << EPD_SDSP_DIS_BIT | 0x5c << EPD_SDSP_ENA_BIT;
	}
	REG_EPD_CTRL = EPD_CTRL_IMG_DONE_INTM | EPD_CTRL_PWR_OFF_INTM | EPD_CTRL_PWR_ON_INTM |
			EPD_CTRL_EPD_DMA_MODE | EPD_CTRL_EPD_ENA;
	REG_EPD_CFG0 = EPD_CFG0_GDCLK_MODE | EPD_CFG0_GDUD | EPD_CFG0_SDRL | EPD_CFG0_GDCLK_POL |
		EPD_CFG0_GDOE_POL | EPD_CFG0_SDOE_POL | EPD_CFG0_SDLE_POL | (0x1 << 10) |
		0x1 << EPD_CFG0_EPD_OBPP_BIT;
	REG_EPD_CFG1 = EPD_CFG1_SDDO_REV | 0x64 << EPD_CFG1_SDOS_BIT | 0x2 << EPD_CFG1_SDCE_NUM_BIT;
	REG_EPD_PPL0 = max_frm;
	REG_EPD_PMGR0 = 0x8 << EPD_PMGR0_PWR_DLY12_BIT | 0xfa << EPD_PMGR0_PWR_DLY01_BIT; 
	REG_EPD_PMGR1 = 0x0 << EPD_PMGR1_PWR_DLY34_BIT | 0x0 << EPD_PMGR1_PWR_DLY23_BIT; 
	REG_EPD_PMGR2 = 0x0 << EPD_PMGR2_PWR_DLY56_BIT | 0x0 << EPD_PMGR2_PWR_DLY45_BIT; 
	REG_EPD_PMGR3 = EPD_PMGR3_PWRCOM_POL | EPD_PMGR3_UNI_POL | 0x7 << EPD_PMGR3_PWR_POL_BIT |
			 0x0 << EPD_PMGR3_PWR_DLY67_BIT;
	REG_EPD_PMGR4 = 0x7 << EPD_PMGR4_PWR_ENA_BIT | 0x7 << EPD_PMGR4_PWR_VAL_BIT;
	REG_EPD_BORDR = 0x0;

	REG_EPD_PPL0_POS  = 0x0 << EPD_PPL_POS_PPL_YPOS_BIT | 0x0 << EPD_PPL_POS_PPL_XPOS_BIT;
	REG_EPD_PPL0_SIZE = 0x258 << EPD_PPL_SIZE_PPL_HEIGHT_BIT | 0x320 << EPD_PPL_SIZE_PPL_WIDTH_BIT;

	REG_LCD_CTRL = 0x4 << LCD_CTRL_BST_BIT | LCD_CTRL_SOFM | LCD_CTRL_OFUM | LCD_CTRL_BPP_4;
	REG_LCD_CFG = LCD_CFG_NEWDES | LCD_CFG_RECOVER;
	REG_LCD_OSDC = LCD_OSDC_OSDEN | LCD_OSDC_F1EN | LCD_OSDC_F0EN;
	REG_LCD_OSDCTRL = LCD_OSDCTRL_OSDBPP_4;
	REG_LCD_DAH = 0x2 << LCD_DAH_HDS_BIT | 0x322 << LCD_DAH_HDE_BIT;	
	REG_LCD_DAV = 0x2 << LCD_DAV_VDS_BIT | 0x25A << LCD_DAV_VDE_BIT;
	REG_EPD_CTRL |= EPD_CTRL_PWRON;
}

static irqreturn_t jz4760fb_interrupt_handler(int irq, void *dev_id)
{
 	unsigned int state;

	state = REG_EPD_ISR;
	//printk("--- EPD state = 0x%x ---\n", state);
	if(state & EPD_ISR_PWR_ON_INT){
		REG_EPD_ISR &= ~EPD_ISR_PWR_ON_INT;		
		REG_LCD_DA0 = virt_to_phys(dma_desc_pal0);
		REG_LCD_DA1 = virt_to_phys(dma1_desc);
		REG_EPD_CTRL |= EPD_CTRL_IMG_REF_ENA | EPD_CTRL_PPL0_FRM_ENA;
		REG_SLCD_CTRL = SLCD_CTRL_DMA_START;
		REG_LCD_CTRL |= LCD_CTRL_ENA;
	}
	if(state & EPD_ISR_PWR_OFF_INT){
		REG_EPD_ISR &= ~EPD_ISR_PWR_OFF_INT;
		REG_SLCD_CTRL = 0;
		REG_LCD_CTRL &= ~LCD_CTRL_ENA;
	}
	if(state & EPD_ISR_IMG_DONE_INT){
		REG_EPD_ISR &= ~EPD_ISR_IMG_DONE_INT;
		REG_EPD_CTRL |= EPD_CTRL_PWROFF;//multi-channel
	}

	state = REG_LCD_STATE;
	//printk("+++ LCD state = 0x%x +++\n", state);
	if (state & LCD_STATE_EOF){ /* End of frame */
		REG_LCD_STATE = state & ~LCD_STATE_EOF;
	}
	if (state & LCD_STATE_SOF){ /* Start of frame */
		REG_LCD_STATE = state & ~LCD_STATE_SOF;
	}
	if (state & LCD_STATE_IFU0) {
		REG_LCD_STATE = state & ~LCD_STATE_IFU0;
	}
	if (state & LCD_STATE_IFU1) {
		REG_LCD_STATE = state & ~LCD_STATE_IFU1;
	}
	if (state & LCD_STATE_OFU) {
		REG_LCD_STATE = state & ~LCD_STATE_OFU;
	}

	return IRQ_HANDLED;
}

static void jz4760fb_gpio_init(void)
{
	__gpio_as_lcd_24bit();
	__gpio_as_epd();
}

static int __init jz4760fb_probe(struct platform_device *pdev)
{
	struct epd_cfb_info *cfb;
	int err = 0;

	if (!pdev)
		goto failed;

	jz4760fb_gpio_init();	
	
	cfb = jz4760fb_alloc_fb_info();
	if (!cfb)
		goto failed_cfb;
	
	err = jz4760fb_map_smem(cfb);
	if (err)
		goto failed_map;

	msc_read(8 * 1024 * 1024, wfm, PAGE_SIZE << 6);
	if(wfm[0] == 0xa2)
		epd_panel = 1;
	else
		epd_panel = 0;
	
	jz4760fb_set_mode();
	
	err |= register_framebuffer(&cfb->fb0);
	err |= register_framebuffer(&cfb->fb1);
	if (err)
		goto failed_cfb;

	jz4760fb_descriptor_init();

	jz4760fb_change_clock();

	if(epd_panel == 0){
		epd_mode = 0;
		epd_temp = 1;
		fill_epd_palette();
	}else
		fill_init_palette();
	
	if (request_irq(IRQ_LCD, jz4760fb_interrupt_handler, IRQF_DISABLED,
			"lcd", 0)) {
		err = -EBUSY;
		goto failed_irq;
	}
	
	init_epd_controller();
	
	print_lcdc_registers();

	epd_mode = 2;
	epd_temp = 1;
	fill_epd_palette();

	return 0;

failed_irq:
	free_irq(IRQ_LCD, NULL);
failed_map:
	jz4760fb_unmap_smem(cfb);
failed_cfb:
	jz4760fb_free_fb_info(cfb);
failed:
	return err;
}

static int jz4760fb_remove(struct platform_device *pdev)
{
	struct epd_cfb_info *cfb = platform_get_drvdata(pdev);

	free_irq(IRQ_LCD, NULL);
	jz4760fb_unmap_smem(cfb);
	jz4760fb_free_fb_info(cfb);
	return 0;
}

#ifdef CONFIG_PM
static int jz4760fb_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("\n%s: %s\n", __FILE__, __FUNCTION__);
	__cpm_stop_lcd();
	return 0;
}

static int jz4760fb_resume(struct platform_device *pdev)
{
	printk("\n%s: %s\n", __FILE__, __FUNCTION__);
	__cpm_start_lcd();
	return 0;
}
#else
static int jz4760fb_suspend(struct platform_device *dev, pm_message_t state) NULL
static int jz4760fb_resume(struct platform_device *pdev) NULL
#endif /* CONFIG_PM */

static struct platform_driver jz_epd_driver = {
	.probe = jz4760fb_probe,
	.remove = jz4760fb_remove,
#ifdef CONFIG_PM
	.suspend = jz4760fb_suspend,
	.resume = jz4760fb_resume,
#endif
	.driver = {
		.name = "jz-lcd",
	},
};

static int __init jz4760fb_init(void)
{
	return platform_driver_register(&jz_epd_driver);
}
subsys_initcall(jz4760fb_init);

static void __exit jz4760fb_exit(void)
{
	platform_driver_unregister(&jz_epd_driver);
}
module_exit(jz4760fb_exit);

MODULE_DESCRIPTION("Jz4760 EPD Controller driver");
MODULE_LICENSE("GPL");
