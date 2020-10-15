/*
 * linux/drivers/video/jz4760_lcd.c -- Ingenic Jz4760 LCD frame buffer device
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
#include <linux/kthread.h>
#include <linux/proc_fs.h>

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
#include <linux/sysrq.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/jzsoc.h>

#include "console/fbcon.h"

#if defined(CONFIG_JZ4770_F4770) || defined(CONFIG_JZ4770_PISCES)
#include "jz4770_lcd.h"
#else
#include "jz4760_lcd.h"
#endif

#if defined(CONFIG_FB_JZ4760_TVE)
#include "jz4760_tve.h"
extern struct jz4760lcd_info jz4760_info_tve;
#endif

#include "overlay/battery_empty.h"

#ifdef CONFIG_JZ4760_SLCD_KGM701A3_TFT_SPFD5420A
#include "jz_kgm_spfd5420a.h"
#endif

MODULE_DESCRIPTION("Jz4760 LCD Controller driver");
MODULE_AUTHOR("Wolfgang Wang, <lgwang@ingenic.cn>");
MODULE_LICENSE("GPL");

// #define JZ_FB_DEBUG

extern struct jz4760lcd_info jz4760_lcd_panel;

struct jz4760lcd_info *jz4760_lcd_info = &jz4760_lcd_panel; /* default output to lcd panel */

struct lcd_cfb_info
{
	struct fb_info fb;
	struct {
		u16 red, green, blue;
	} palette[NR_PALETTE];

	int b_lcd_display;
	int b_lcd_pwm;
	int backlight_level;
};

static struct lcd_cfb_info *jz4760fb_info;

static unsigned char *lcd_palette;

static struct jz4760_lcd_dma_desc *fg0_desc = NULL;
unsigned char *lcd_frame0, *fg0_databuf = NULL;
unsigned int frame_yoffset = 0;
bool clear_fb, fg0_lock = true;

#ifdef VSYNC_OPTION
bool vsync_on = 1;
#endif
uint32_t vsync_count;
uint32_t ipu_ratio = 1;
uint32_t tve_mode = 0;

bool backlight_control = true;

spinlock_t lock;
wait_queue_head_t wait_vsync;

#define MAX_XRES TVE_WIDTH_PAL
#define MAX_YRES TVE_HEIGHT_PAL

#define LCD_SCREEN_W jz4760_lcd_info->panel.w
#define LCD_SCREEN_H jz4760_lcd_info->panel.h

int get_lcd_width(void) {
	return LCD_SCREEN_W;
}
int get_lcd_height(void) {
	return LCD_SCREEN_H;
}
EXPORT_SYMBOL(get_lcd_width);
EXPORT_SYMBOL(get_lcd_height);

void fg0_enable(bool show)
{
	if (fg0_lock) return;
	__lcd_osd_change_size();
	if (show)
		__lcd_enable_f0();
	else
		__lcd_disable_f0();
	while ((REG_LCD_OSDS & LCD_OSDS_READY) != 1);
}
EXPORT_SYMBOL(fg0_enable);

void fg0_draw(unsigned short *src, uint16_t src_w, int16_t src_h, int16_t dst_x, uint16_t dst_y)
{
	unsigned short *dst = (unsigned short *)fg0_databuf;
	uint16_t dst_w = LCD_SCREEN_W, dst_h = LCD_SCREEN_H, x, y, l;
	uint8_t  factor = 1 + (dst_w == 320 && dst_h == 480); // RG 320x480@320x240 hack

	if (dst_y < 0)
		dst += (dst_h - (src_h * factor)) * dst_w / 2; // center vertically
	else
		dst += dst_y * dst_w * factor; // y offset

	if (dst_x < 0)
		dst += (dst_w - src_w) / 2; // center horizontally
	else
		dst += dst_x; // x offset

	for (y = 0; y < src_h; y++) {
		for (l = 0; l < factor; l++) { // double h line
			for (x = 0; x < src_w; x++) {
				*dst++ = src[x + y * src_w];
			}
			dst += (dst_w - src_w);
		}
	}
}

#if defined(CONFIG_PM) && !defined(CONFIG_SOC_JZ4770)
extern int jz_pm_sleep(void);
#endif

extern void ipu_stop(void);
extern void ipu_start(int, int, int, int);
extern void ipu_update_address(void);

static void jz4760fb_set_mode(struct jz4760lcd_info *lcd_info);
static void jz4760fb_deep_set_mode(struct jz4760lcd_info *lcd_info);
static void jz4760fb_change_clock(struct jz4760lcd_info *lcd_info);

void jz4760fb_set_backlight_level(int n);

static void screen_on(void);
static void screen_off(void);

static void display_rgb_bar()
{
	int i;
	unsigned short *ptr;
	ptr = (unsigned short*)lcd_frame0;

	for (i = 0; i < LCD_SCREEN_W * LCD_SCREEN_H / 6; i++) *ptr++ = 0xf800;
	for (i = 0; i < LCD_SCREEN_W * LCD_SCREEN_H / 6; i++) *ptr++ = 0x07e0;
	for (i = 0; i < LCD_SCREEN_W * LCD_SCREEN_H / 6; i++) *ptr++ = 0x001f;
	dma_cache_wback_inv((unsigned int)lcd_frame0, LCD_SCREEN_W * LCD_SCREEN_H * 2);
}
#if defined(JZ_FB_DEBUG) /* The following routine is only for test */
#define D(fmt, args...) printk(KERN_ERR "%s:%d "fmt"\n", __func__, __LINE__, ##args)

// static void test_gpio(int gpio_num, int delay)
// {
// 	__gpio_as_output(gpio_num);
// 	while (1)
// 	{
// 		__gpio_set_pin(gpio_num);   udelay(delay);
// 		__gpio_clear_pin(gpio_num); udelay(delay);
// 	}
// }


static void print_lcdc_registers(void)	/* debug */
{
	/* LCD Controller Resgisters */
	D("REG_LCD_CFG:\t0x%08x", REG_LCD_CFG);
	D("REG_LCD_CTRL:\t0x%08x", REG_LCD_CTRL);
	D("REG_LCD_STATE:\t0x%08x", REG_LCD_STATE);
	D("REG_LCD_OSDC:\t0x%08x", REG_LCD_OSDC);
	D("REG_LCD_OSDCTRL:\t0x%08x", REG_LCD_OSDCTRL);
	D("REG_LCD_OSDS:\t0x%08x", REG_LCD_OSDS);
	D("REG_LCD_BGC:\t0x%08x", REG_LCD_BGC);
	D("REG_LCD_KEK0:\t0x%08x", REG_LCD_KEY0);
	D("REG_LCD_KEY1:\t0x%08x", REG_LCD_KEY1);
	D("REG_LCD_ALPHA:\t0x%08x", REG_LCD_ALPHA);
	D("REG_LCD_IPUR:\t0x%08x", REG_LCD_IPUR);
	D("REG_LCD_VAT:\t0x%08x", REG_LCD_VAT);
	D("REG_LCD_DAH:\t0x%08x", REG_LCD_DAH);
	D("REG_LCD_DAV:\t0x%08x", REG_LCD_DAV);
	D("REG_LCD_XYP0:\t0x%08x", REG_LCD_XYP0);
	D("REG_LCD_XYP1:\t0x%08x", REG_LCD_XYP1);
	D("REG_LCD_SIZE0:\t0x%08x", REG_LCD_SIZE0);
	D("REG_LCD_SIZE1:\t0x%08x", REG_LCD_SIZE1);
	D("REG_LCD_RGBC\t0x%08x", REG_LCD_RGBC);
	D("REG_LCD_VSYNC:\t0x%08x", REG_LCD_VSYNC);
	D("REG_LCD_HSYNC:\t0x%08x", REG_LCD_HSYNC);
	D("REG_LCD_PS:\t0x%08x", REG_LCD_PS);
	D("REG_LCD_CLS:\t0x%08x", REG_LCD_CLS);
	D("REG_LCD_SPL:\t0x%08x", REG_LCD_SPL);
	D("REG_LCD_REV:\t0x%08x", REG_LCD_REV);
	D("REG_LCD_IID:\t0x%08x", REG_LCD_IID);
	D("REG_LCD_DA0:\t0x%08x", REG_LCD_DA0);
	D("REG_LCD_SA0:\t0x%08x", REG_LCD_SA0);
	D("REG_LCD_FID0:\t0x%08x", REG_LCD_FID0);
	D("REG_LCD_CMD0:\t0x%08x", REG_LCD_CMD0);
	D("REG_LCD_OFFS0:\t0x%08x", REG_LCD_OFFS0);
	D("REG_LCD_PW0:\t0x%08x", REG_LCD_PW0);
	D("REG_LCD_CNUM0:\t0x%08x", REG_LCD_CNUM0);
	D("REG_LCD_DESSIZE0:\t0x%08x", REG_LCD_DESSIZE0);
	D("REG_LCD_DA1:\t0x%08x", REG_LCD_DA1);
	D("REG_LCD_SA1:\t0x%08x", REG_LCD_SA1);
	D("REG_LCD_FID1:\t0x%08x", REG_LCD_FID1);
	D("REG_LCD_CMD1:\t0x%08x", REG_LCD_CMD1);
	D("REG_LCD_OFFS1:\t0x%08x", REG_LCD_OFFS1);
	D("REG_LCD_PW1:\t0x%08x", REG_LCD_PW1);
	D("REG_LCD_CNUM1:\t0x%08x", REG_LCD_CNUM1);
	D("REG_LCD_DESSIZE1:\t0x%08x", REG_LCD_DESSIZE1);
	D("==================================");
	D("REG_LCD_VSYNC:\t%d:%d", REG_LCD_VSYNC>>16, REG_LCD_VSYNC&0xfff);
	D("REG_LCD_HSYNC:\t%d:%d", REG_LCD_HSYNC>>16, REG_LCD_HSYNC&0xfff);
	D("REG_LCD_VAT:\t%d:%d", REG_LCD_VAT>>16, REG_LCD_VAT&0xfff);
	D("REG_LCD_DAH:\t%d:%d", REG_LCD_DAH>>16, REG_LCD_DAH&0xfff);
	D("REG_LCD_DAV:\t%d:%d", REG_LCD_DAV>>16, REG_LCD_DAV&0xfff);
	D("==================================");

	/* Smart LCD Controller Resgisters */
	D("REG_SLCD_CFG:\t0x%08x", REG_SLCD_CFG);
	D("REG_SLCD_CTRL:\t0x%08x", REG_SLCD_CTRL);
	D("REG_SLCD_STATE:\t0x%08x", REG_SLCD_STATE);
	D("==================================");

	/* TVE Controller Resgisters */
	D("REG_TVE_CTRL:\t0x%08x", REG_TVE_CTRL);
	D("REG_TVE_FRCFG:\t0x%08x", REG_TVE_FRCFG);
	D("REG_TVE_SLCFG1:\t0x%08x", REG_TVE_SLCFG1);
	D("REG_TVE_SLCFG2:\t0x%08x", REG_TVE_SLCFG2);
	D("REG_TVE_SLCFG3:\t0x%08x", REG_TVE_SLCFG3);
	D("REG_TVE_LTCFG1:\t0x%08x", REG_TVE_LTCFG1);
	D("REG_TVE_LTCFG2:\t0x%08x", REG_TVE_LTCFG2);
	D("REG_TVE_CFREQ:\t0x%08x", REG_TVE_CFREQ);
	D("REG_TVE_CPHASE:\t0x%08x", REG_TVE_CPHASE);
	D("REG_TVE_CBCRCFG:\t0x%08x", REG_TVE_CBCRCFG);
	D("REG_TVE_WSSCR:\t0x%08x", REG_TVE_WSSCR);
	D("REG_TVE_WSSCFG1:\t0x%08x", REG_TVE_WSSCFG1);
	D("REG_TVE_WSSCFG2:\t0x%08x", REG_TVE_WSSCFG2);
	D("REG_TVE_WSSCFG3:\t0x%08x", REG_TVE_WSSCFG3);

	D("==================================");
}
#else
	#define print_lcdc_registers()
	#define D(fmt, args...)
#endif

#define E(fmt, args...) printk(KERN_ERR "%s:%d "fmt"\n", __func__, __LINE__, ##args)

static void ctrl_enable(void)
{
	D("");
	REG_LCD_STATE = 0; /* clear lcdc status */
	__lcd_slcd_special_on();
	__lcd_clr_dis();
	__lcd_set_ena(); /* enable lcdc */
	return;
}

static void ctrl_disable(void)
{
	D("");
	if (jz4760_lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD || jz4760_lcd_info->panel.cfg & LCD_CFG_TVEN) {
		__lcd_clr_ena();						   /* Smart lcd and TVE mode only support quick disable */
	} else {
		int cnt;
		/* when CPU main freq is 336MHz,wait for 30ms */
		cnt = 528000 * 30;
		__lcd_set_dis(); /* regular disable */
		//__lcd_clr_ena();
		while (!__lcd_disable_done() && cnt) {
			cnt--;
		}
		if (cnt == 0)
			D("LCD disable timeout! REG_LCD_STATE=0x%08xx", REG_LCD_STATE);
		REG_LCD_STATE &= ~LCD_STATE_LDD;
	}

	return;
}

static inline u_int chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	D("");
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int jz4760fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue, u_int transp, struct fb_info *info)
{
	// D("");
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	unsigned short *ptr, ctmp;

	//	D("regno:%d,RGBt:(%d,%d,%d,%d)\t", regno, red, green, blue, transp);
	if (regno >= NR_PALETTE)
		return 1;

	cfb->palette[regno].red = red;
	cfb->palette[regno].green = green;
	cfb->palette[regno].blue = blue;
	if (cfb->fb.var.bits_per_pixel <= 16)
	{
		red >>= 8;
		green >>= 8;
		blue >>= 8;

		red &= 0xff;
		green &= 0xff;
		blue &= 0xff;
	}
	switch (cfb->fb.var.bits_per_pixel)
	{
	case 1:
	case 2:
	case 4:
	case 8:
		if (((jz4760_lcd_info->panel.cfg & LCD_CFG_MODE_MASK) == LCD_CFG_MODE_SINGLE_MSTN) ||
			((jz4760_lcd_info->panel.cfg & LCD_CFG_MODE_MASK) == LCD_CFG_MODE_DUAL_MSTN))
		{
			ctmp = (77L * red + 150L * green + 29L * blue) >> 8;
			ctmp = ((ctmp >> 3) << 11) | ((ctmp >> 2) << 5) | (ctmp >> 3);
		}
		else
		{
			/* RGB 565 */
			if (((red >> 3) == 0) && ((red >> 2) != 0))
				red = 1 << 3;
			if (((blue >> 3) == 0) && ((blue >> 2) != 0))
				blue = 1 << 3;
			ctmp = ((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3);
		}

		ptr = (unsigned short *)lcd_palette;
		ptr = (unsigned short *)(((u32)ptr) | 0xa0000000);
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
		if (regno < 16)
		{
			((u32 *)cfb->fb.pseudo_palette)[regno] =
				((red >> 3) << 11) |
				((green >> 2) << 5) |
				(blue >> 3);
		}
		break;
	case 17 ... 32:
		if (regno < 16)
			((u32 *)cfb->fb.pseudo_palette)[regno] =
				(red << 16) |
				(green << 8) |
				(blue << 0);

		/* if (regno < 16) {
			unsigned val;
                        val  = chan_to_field(red, &cfb->fb.var.red);
                        val |= chan_to_field(green, &cfb->fb.var.green);
                        val |= chan_to_field(blue, &cfb->fb.var.blue);
			((u32 *)cfb->fb.pseudo_palette)[regno] = val;
		} */

		break;
	}
	return 0;
}

static int jz4760fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	D("");
	int ret = 0;

	void __user *argp = (void __user *)arg;

	switch (cmd)
	{
	case FBIOSETBACKLIGHT:
		jz4760fb_set_backlight_level(arg);
		break;

	case FBIODISPON:
		ctrl_enable();
		screen_on();
		break;

	case FBIODISPOFF:
		screen_off();
		ctrl_disable();
		break;

	case FBIOPRINT_REG:
		print_lcdc_registers();
		break;

	case FBIO_GET_MODE:
		D("fbio get mode\n");
		if (copy_to_user(argp, jz4760_lcd_info, sizeof(struct jz4760lcd_info)))
			return -EFAULT;
		break;

	case FBIO_SET_MODE:
		D("fbio set mode\n");

		if (copy_from_user(jz4760_lcd_info, argp, sizeof(struct jz4760lcd_info)))
			return -EFAULT;

		/* set mode */
		jz4760fb_set_mode(jz4760_lcd_info);
		break;

	case FBIO_DEEP_SET_MODE:
		D("fbio deep set mode\n");
		if (copy_from_user(jz4760_lcd_info, argp, sizeof(struct jz4760lcd_info)))
			return -EFAULT;

		jz4760fb_deep_set_mode(jz4760_lcd_info);
		break;

	case FBIO_MODE_SWITCH:
		D("FBIO_MODE_SWITCH");
		switch (arg)
		{
#if 0 //ifdef CONFIG_FB_JZ4760_TVE
		case PANEL_MODE_TVE_PAL:  /* switch to TVE_PAL mode */
		case PANEL_MODE_TVE_NTSC: /* switch to TVE_NTSC mode */
			jz4760lcd_info_switch_to_TVE(arg);
			jz4760tve_init(arg); /* tve controller init */
			udelay(100);
			cpm_start_clock(CGM_TVE);
			jz4760tve_enable_tve();
			/* turn off lcd backlight */
			screen_off();
			break;
#endif
#if defined(CONFIG_JZ4760_HDMI_DISPLAY)
		case PANEL_MODE_HDMI_480P:
			set_i2s_external_codec();
			/* turn off TVE, turn off DACn... */
			//jz4760tve_disable_tve();
			jz4760_lcd_info = &jz4760_info_hdmi_480p;
			/* turn on lcd backlight */
			screen_off();
			break;

		case PANEL_MODE_HDMI_576P:
			set_i2s_external_codec();
			/* turn off TVE, turn off DACn... */
			//jz4760tve_disable_tve();
			jz4760_lcd_info = &jz4760_info_hdmi_576p;
			/* turn on lcd backlight */
			screen_off();
			break;

		case PANEL_MODE_HDMI_720P50:
#if defined(CONFIG_SOC_JZ4760B)
			REG_LCD_PCFG = 0xc0000888;
			REG_GPIO_PXSLC(2) = 0;
			REG_GPIO_PXDS1S(2) |= 1 << 8;
			REG_GPIO_PXDS0S(2) = 0x0fffffff;

#endif
			set_i2s_external_codec();
			/* turn off TVE, turn off DACn... */
			//jz4760tve_disable_tve();
			jz4760_lcd_info = &jz4760_info_hdmi_720p50;
			/* turn on lcd backlight */
			screen_off();
			break;

		case PANEL_MODE_HDMI_720P60:
#if defined(CONFIG_SOC_JZ4760B)
			REG_LCD_PCFG = 0xc0000888;
			REG_GPIO_PXSLC(2) = 0;
			REG_GPIO_PXDS1S(2) |= 1 << 8;
			REG_GPIO_PXDS0S(2) = 0x0fffffff;
#endif
			set_i2s_external_codec();
			/* turn off TVE, turn off DACn... */
			//jz4760tve_disable_tve();
			jz4760_lcd_info = &jz4760_info_hdmi_720p60;
			/* turn on lcd backlight */
			screen_off();
			break;
#endif							   //CONFIG_JZ4760_HDMI_DISPLAY
		case PANEL_MODE_LCD_PANEL: /* switch to LCD mode */
		default:
			/* turn off TVE, turn off DACn... */
#ifdef CONFIG_FB_JZ4760_TVE
			jz4760tve_disable_tve();
			cpm_stop_clock(CGM_TVE);
#endif
			jz4760_lcd_info = &jz4760_lcd_panel;
			/* turn on lcd backlight */
			screen_on();
			break;
		}
		jz4760fb_deep_set_mode(jz4760_lcd_info);
		break;

#ifdef CONFIG_FB_JZ4760_TVE
	case FBIO_GET_TVE_MODE:
		D("fbio get TVE mode");
		if (copy_to_user(argp, jz4760_tve_info, sizeof(struct jz4760tve_info)))
			return -EFAULT;
		break;

	case FBIO_SET_TVE_MODE:
		D("fbio set TVE mode");
		if (copy_from_user(jz4760_tve_info, argp, sizeof(struct jz4760tve_info)))
			return -EFAULT;
		/* set tve mode */
		jz4760tve_set_tve_mode(jz4760_tve_info);
		break;
#endif

	default:
		E("unknown command(0x%x)", cmd);
		break;
	}

	return ret;
}

/* Use mmap /dev/fb can only get a non-cacheable Virtual Address. */
static int jz4760fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	D("");
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
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot); /* Uncacheable */

	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED; /* Uncacheable */
//	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;	/* Write-Back */

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
						   vma->vm_end - vma->vm_start,
						   vma->vm_page_prot))
	{
		return -EAGAIN;
	}
	return 0;
}

/* checks var and eventually tweaks it to something supported,
 * DO NOT MODIFY PAR */
static int jz4760fb_check_var(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	// D("");
	clear_fb = var->bits_per_pixel != fb->var.bits_per_pixel || var->xres != fb->var.xres || var->yres != fb->var.yres;
	return 0;
}

static int jzfb_wait_for_vsync(void)
{
	// D("");
	uint32_t count = vsync_count;
	long t = wait_event_interruptible_timeout(wait_vsync, count != vsync_count, HZ / 10);
	return t > 0 ? 0 : (t < 0 ? (int)t : -ETIMEDOUT);
}

static void jz4760fb_fg0_init(void)
{
	unsigned int fg0_line_size, size;

	size = PAGE_ALIGN(LCD_SCREEN_W * LCD_SCREEN_H * 2); // 16bpp / 8 = 2; 1 buffer;

	if (fg0_databuf == NULL) fg0_desc = (struct jz4760_lcd_dma_desc *)__get_free_pages(GFP_KERNEL, 0);

	fg0_line_size = (LCD_SCREEN_W * 2); // 16bpp / 8 = 2; 1 buffer;
	fg0_line_size = ((fg0_line_size + 3) >> 2) << 2; // word aligned
	fg0_line_size = fg0_line_size * LCD_SCREEN_H;

	if (fg0_databuf == NULL) fg0_databuf = alloc_pages_exact(size, GFP_KERNEL);

	memset(fg0_databuf, 0x00, size);

	size = LCD_SCREEN_H << 16 | LCD_SCREEN_W;

	fg0_desc->next_desc = (unsigned int)virt_to_phys(fg0_desc);
	fg0_desc->databuf    = virt_to_phys((void *)fg0_databuf);
	fg0_desc->frame_id   = (unsigned int)0x0000da00;
	fg0_desc->cmd        = fg0_line_size / 4;
	fg0_desc->offsize    = 0;
	fg0_desc->page_width = 0;
	fg0_desc->desc_size  = size;

	REG_LCD_DA0 = virt_to_phys(fg0_desc);
	REG_LCD_SIZE0 = size;
}


/*
 * set the video mode according to info->var
 */
static int jz4760fb_set_par(struct fb_info *info)
{
	D("");
	struct fb_var_screeninfo *var = &info->var;
	struct fb_fix_screeninfo *fix = &info->fix;

	fg0_lock = 1;

	spin_lock_irq(&lock);

	jz4760fb_change_clock(jz4760_lcd_info);
	mdelay(50);

	ipu_stop();

	int src_w = var->xres, src_h = var->yres;
	int dst_w = LCD_SCREEN_W, dst_h = LCD_SCREEN_H;
	int xoffset = 0, yoffset = 0;

	__lcd_osd_change_size();

#ifdef CONFIG_FB_JZ4760_TVE
	if (tve_mode) {
		mdelay(500);

		dst_h = (LCD_SCREEN_H / 2) & ~1;

		REG_LCD_SIZE1 = (dst_h * 2) << 16 | dst_w;
	} else
#endif
	{
		int factor = 1 + (dst_w == 320 && dst_h == 480); // RG 320x480@320x240 hack
		switch (ipu_ratio) {
			case 3: { // 4:3; this can be improved, but it works for now
				if (dst_w * factor > dst_h) {
					dst_w = dst_h * (4 * 10000 / 3);
					dst_w = dst_w / 10000;
					dst_w = dst_w & ~1;
				} else {
					dst_h = dst_w * (4 * 10000 / 3);
					dst_h = dst_h / 10000;
					dst_h = dst_h & ~1;
				}
				break;
			}
			case 2: // original - no scale
			{
				if (src_w <= dst_w && src_h * factor <= dst_h) { // image smaller than screen
					dst_w = src_w;
					dst_h = src_h * factor;
					break; // break here or fallback to case 1 and downscale
				}
			}
			case 1: // scale ratio
			{
				if (src_w != dst_w || src_h * factor != dst_h ) {
					int ri = src_w * 100000 / (src_h * factor);
					int rs = dst_w * 100000 / dst_h;

					if (rs >= ri)
						dst_w = (dst_h * src_w / (src_h * factor)) & ~1;
					if (rs <= ri)
						dst_h = (src_h * factor * dst_w / src_w) & ~1;
				}
			}
		}

		if (dst_w > LCD_SCREEN_W) dst_w = LCD_SCREEN_W;
		if (dst_h > LCD_SCREEN_H) dst_h = LCD_SCREEN_H;

		xoffset = (LCD_SCREEN_W - dst_w) / 2;
		yoffset = (LCD_SCREEN_H - dst_h) / 2;

		REG_LCD_SIZE1 = dst_h << 16 | dst_w;
	}
	while ((REG_LCD_OSDS & LCD_OSDS_READY) != 1);

	__lcd_osd_change_size();
	REG_LCD_XYP1 = yoffset << 16 | xoffset;
	while ((REG_LCD_OSDS & LCD_OSDS_READY) != 1);

	mdelay(100);

	ipu_start(src_w, src_h, dst_w, dst_h);

	if (!tve_mode) {
		fg0_lock = 0;
		fg0_enable(false);
		if (fg0_databuf == NULL) jz4760fb_fg0_init();
		fg0_draw(battery_empty, battery_empty_w, battery_empty_h, 2, 2);
	}

	if (clear_fb) {
		void *page_virt = lcd_frame0;
		fix->line_length = var->xres * (var->bits_per_pixel >> 3);
		unsigned int size = fix->line_length * var->yres * 3;
		for (; page_virt < lcd_frame0 + size; page_virt += PAGE_SIZE)
			clear_page(page_virt);
		ipu_update_address();
	}

	spin_unlock_irq(&lock);

	return 0;
}

/*
 * (Un)Blank the display.
 * Fix me: should we use VESA value?
 */
static int jz4760fb_blank(int blank_mode, struct fb_info *info)
{
	D("jz4760 fb_blank %d %p", blank_mode, info);
	switch (blank_mode)
	{
	case FB_BLANK_UNBLANK:
		//case FB_BLANK_NORMAL:
		/* Turn on panel */
		__lcd_set_ena();
		screen_on();

		break;

	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
#if 0
		/* Turn off panel */
		__lcd_display_off();
		__lcd_set_dis();
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
static int jz4760fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	// D("");
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	if (!var || !cfb)
		return -EINVAL;

	spin_lock_irq(&lock);
	frame_yoffset = var->yoffset * cfb->fb.fix.line_length;

#ifdef VSYNC_OPTION
	if (vsync_on)
#endif
	{
		jzfb_wait_for_vsync();
		ipu_update_address();
	}
#ifdef VSYNC_OPTION
	else
	{
		dma_cache_wback_inv((unsigned long)(lcd_frame0 + frame_yoffset), cfb->fb.fix.line_length * cfb->fb.var.yres);
	}
#endif
	spin_unlock_irq(&lock);
	return 0;
}

/* use default function cfb_fillrect, cfb_copyarea, cfb_imageblit */
static struct fb_ops jz4760fb_ops = {
	.owner = THIS_MODULE,
	.fb_setcolreg = jz4760fb_setcolreg,
	.fb_check_var = jz4760fb_check_var,
	.fb_set_par = jz4760fb_set_par,
	.fb_blank = jz4760fb_blank,
	.fb_pan_display = jz4760fb_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_mmap = jz4760fb_mmap,
	.fb_ioctl = jz4760fb_ioctl,
};

static int jz4760fb_set_var(struct fb_var_screeninfo *var, int con, struct fb_info *info)
{
	D("");

	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	struct jz4760lcd_info *lcd_info = jz4760_lcd_info;
	int chgvar = 0;

	var->height = lcd_info->osd.fg1.h; /* tve mode */
	var->width = lcd_info->osd.fg1.w;
	var->bits_per_pixel = lcd_info->osd.fg0.bpp;

	var->vmode = FB_VMODE_NONINTERLACED;
	var->activate = cfb->fb.var.activate;
	var->xres = var->width;
	var->yres = var->height;
	var->xres_virtual = var->width;
	var->yres_virtual = var->height;
	var->xoffset = 0;
	var->yoffset = 0;
	var->pixclock = 0;
	var->left_margin = 0;
	var->right_margin = 0;
	var->upper_margin = 0;
	var->lower_margin = 0;
	var->hsync_len = 0;
	var->vsync_len = 0;
	var->sync = 0;
	var->activate &= ~FB_ACTIVATE_TEST;

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

	if (cfb->fb.var.xres != var->xres) chgvar = 1;
	if (cfb->fb.var.yres != var->yres) chgvar = 1;
	if (cfb->fb.var.xres_virtual != var->xres_virtual) chgvar = 1;
	if (cfb->fb.var.yres_virtual != var->yres_virtual) chgvar = 1;
	if (cfb->fb.var.bits_per_pixel != var->bits_per_pixel) chgvar = 1;

	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;

	switch (var->bits_per_pixel)
	{
	case 1: /* Mono */
		cfb->fb.fix.visual = FB_VISUAL_MONO01;
		cfb->fb.fix.line_length = (var->xres * var->bits_per_pixel) / 8;
		break;
	case 2: /* Mono */
		var->red.offset = 0;
		var->red.length = 2;
		var->green.offset = 0;
		var->green.length = 2;
		var->blue.offset = 0;
		var->blue.length = 2;

		cfb->fb.fix.visual = FB_VISUAL_PSEUDOCOLOR;
		cfb->fb.fix.line_length = (var->xres * var->bits_per_pixel) / 8;
		break;
	case 4: /* PSEUDOCOLOUR*/
		var->red.offset = 0;
		var->red.length = 4;
		var->green.offset = 0;
		var->green.length = 4;
		var->blue.offset = 0;
		var->blue.length = 4;

		cfb->fb.fix.visual = FB_VISUAL_PSEUDOCOLOR;
		cfb->fb.fix.line_length = var->xres / 2;
		break;
	case 8: /* PSEUDOCOLOUR, 256 */
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 0;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;

		cfb->fb.fix.visual = FB_VISUAL_PSEUDOCOLOR;
		cfb->fb.fix.line_length = var->xres;
		break;
	case 15: /* DIRECTCOLOUR, 32k */
		var->bits_per_pixel = 15;
		var->red.offset = 10;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 5;
		var->blue.offset = 0;
		var->blue.length = 5;

		cfb->fb.fix.visual = FB_VISUAL_DIRECTCOLOR;
		cfb->fb.fix.line_length = var->xres_virtual * 2;
		break;
	case 16: /* DIRECTCOLOUR, 64k */
		var->bits_per_pixel = 16;
		var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 5;

		cfb->fb.fix.visual = FB_VISUAL_TRUECOLOR;
		cfb->fb.fix.line_length = var->xres_virtual * 2;
		break;
	case 17 ... 32:
		/* DIRECTCOLOUR, 256 */
		var->bits_per_pixel = 32;

		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 24;
		var->transp.length = 8;

		cfb->fb.fix.visual = FB_VISUAL_TRUECOLOR;
		cfb->fb.fix.line_length = var->xres_virtual * 4;
		break;

	default: /* in theory this should never happen */
		printk(KERN_WARNING "%s: don't support for %dbpp\n", cfb->fb.fix.id, var->bits_per_pixel);
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

	return 0;
}

static struct lcd_cfb_info *jz4760fb_alloc_fb_info(void)
{
	D("");
	struct lcd_cfb_info *cfb;

	cfb = kmalloc(sizeof(struct lcd_cfb_info) + sizeof(u32) * 16, GFP_KERNEL);

	if (!cfb)
		return NULL;

	jz4760fb_info = cfb;

	memset(cfb, 0, sizeof(struct lcd_cfb_info));

	cfb->backlight_level = LCD_DEFAULT_BACKLIGHT;

	strcpy(cfb->fb.fix.id, "jz-lcd");
	cfb->fb.fix.type = FB_TYPE_PACKED_PIXELS;
	cfb->fb.fix.type_aux = 0;
	cfb->fb.fix.xpanstep = 1;
	cfb->fb.fix.ypanstep = 1;
	cfb->fb.fix.ywrapstep = 0;
	cfb->fb.fix.accel = FB_ACCEL_NONE;

	cfb->fb.var.nonstd = 0;
	cfb->fb.var.activate = FB_ACTIVATE_NOW;
	cfb->fb.var.height = -1;
	cfb->fb.var.width = -1;
	cfb->fb.var.accel_flags = FB_ACCELF_TEXT;

	cfb->fb.fbops = &jz4760fb_ops;
	cfb->fb.flags = FBINFO_FLAG_DEFAULT;

	cfb->fb.pseudo_palette = (void *)(cfb + 1);

	switch (jz4760_lcd_info->osd.fg0.bpp) {
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
	D("fb_alloc_cmap,fb.cmap.len: %d", cfb->fb.cmap.len);

	return cfb;
}

/*
 * Map screen memory
 */
static int jz4760fb_map_smem(struct lcd_cfb_info *cfb)
{
	D("");
	/* Compute space for max res at 16bpp, triple buffered + ipu buffer. */
	// unsigned int size = PAGE_ALIGN(MAX_XRES * MAX_YRES * 4 * 3);
	unsigned int size = PAGE_ALIGN(MAX_XRES * MAX_YRES * 2 * 3); // 16bpp / 8 = 2; 3 buffers;
	void *page_virt;

	lcd_palette = (unsigned char *)__get_free_pages(GFP_KERNEL, 0);
	lcd_frame0 = alloc_pages_exact(size, GFP_KERNEL);

	if ((!lcd_palette) || (!lcd_frame0))
		return -ENOMEM;

	/*
	 * Set page reserved so that mmap will work. This is necessary
	 * since we'll be remapping normal memory.
	 */
	page_virt = (unsigned long)lcd_palette;
	SetPageReserved(virt_to_page((void *)page_virt));

	for (page_virt = lcd_frame0; page_virt < lcd_frame0 + size; page_virt += PAGE_SIZE) {
		SetPageReserved(virt_to_page(page_virt));
		clear_page(page_virt);
	}

	cfb->fb.fix.smem_start = virt_to_phys((void *)lcd_frame0);
	cfb->fb.fix.smem_len = (size); /* page_shift/2 ??? */
	cfb->fb.screen_base = (unsigned char *)(((unsigned int)lcd_frame0 & 0x1fffffff) | 0xa0000000);

	if (!cfb->fb.screen_base) {
		E("%s: unable to map screen memory", cfb->fb.fix.id);
		return -ENOMEM;
	}

	return 0;
}

static void jz4760fb_free_fb_info(struct lcd_cfb_info *cfb)
{
	D("");
	if (cfb) {
		fb_alloc_cmap(&cfb->fb.cmap, 0, 0);
		kfree(cfb);
	}
}

static void jz4760fb_unmap_smem(struct lcd_cfb_info *cfb)
{
	D("");
	struct page *map = NULL;
	unsigned char *tmp;
	unsigned int page_shift, needroom, bpp, w, h;

	bpp = jz4760_lcd_info->osd.fg0.bpp;
	if (bpp == 18 || bpp == 24) bpp = 32;
	if (bpp == 15) bpp = 16;
	w = jz4760_lcd_info->osd.fg0.w;
	h = jz4760_lcd_info->osd.fg0.h;
	needroom = ((w * bpp + 7) >> 3) * h;
// #if defined(CONFIG_FB_JZ4760_LCD_USE_2LAYER_FRAMEBUFFER)
	bpp = jz4760_lcd_info->osd.fg1.bpp;
	if (bpp == 18 || bpp == 24) bpp = 32;
	if (bpp == 15) bpp = 16;
	w = jz4760_lcd_info->osd.fg1.w;
	h = jz4760_lcd_info->osd.fg1.h;
	needroom += ((w * bpp + 7) >> 3) * h;
// #endif

	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;

	if (cfb && cfb->fb.screen_base) {
		iounmap(cfb->fb.screen_base);
		cfb->fb.screen_base = NULL;
		release_mem_region(cfb->fb.fix.smem_start, cfb->fb.fix.smem_len);
	}

	if (lcd_palette) {
		map = virt_to_page(lcd_palette);
		clear_bit(PG_reserved, &map->flags);
		free_pages((int)lcd_palette, 0);
	}

	if (lcd_frame0) {
		for (tmp = (unsigned char *)lcd_frame0;
			 tmp < lcd_frame0 + (PAGE_SIZE << page_shift);
			 tmp += PAGE_SIZE)
		{
			map = virt_to_page(tmp);
			clear_bit(PG_reserved, &map->flags);
		}
		free_pages((int)lcd_frame0, page_shift);
	}
}


static void jz4760fb_set_panel_mode(struct jz4760lcd_info *lcd_info)
{
	D("");
	struct jz4760lcd_panel_t *panel = &lcd_info->panel;
#ifdef CONFIG_JZ4760_VGA_DISPLAY
	REG_TVE_CTRL |= TVE_CTRL_DAPD;
	REG_TVE_CTRL &= ~(TVE_CTRL_DAPD1 | TVE_CTRL_DAPD2 | TVE_CTRL_DAPD3);
#endif

	D("NEW BPP %d", lcd_info->osd.fg1.bpp);

	/* set bpp */
	lcd_info->panel.ctrl &= ~LCD_CTRL_BPP_MASK;
	if (lcd_info->osd.fg1.bpp == 1)       lcd_info->panel.ctrl |= LCD_CTRL_BPP_1;
	else if (lcd_info->osd.fg1.bpp == 2)  lcd_info->panel.ctrl |= LCD_CTRL_BPP_2;
	else if (lcd_info->osd.fg1.bpp == 4)  lcd_info->panel.ctrl |= LCD_CTRL_BPP_4;
	else if (lcd_info->osd.fg1.bpp == 8)  lcd_info->panel.ctrl |= LCD_CTRL_BPP_8;
	else if (lcd_info->osd.fg1.bpp == 15) lcd_info->panel.ctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB555;
	else if (lcd_info->osd.fg1.bpp == 16) lcd_info->panel.ctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB565;
	else if (lcd_info->osd.fg1.bpp > 16 && lcd_info->osd.fg1.bpp <= 32) {
		lcd_info->osd.fg1.bpp = 32;
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_18_24;
	} else {
		E("The BPP %d is not supported", lcd_info->osd.fg1.bpp);
		lcd_info->osd.fg1.bpp = 32;
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_18_24;
	}

	lcd_info->panel.cfg |= LCD_CFG_NEWDES; /* use 8words descriptor always */

	REG_LCD_CTRL = lcd_info->panel.ctrl;	 /* LCDC Controll Register */
	REG_LCD_CFG = lcd_info->panel.cfg;		 /* LCDC Configure Register */
	REG_SLCD_CFG = lcd_info->panel.slcd_cfg; /* Smart LCD Configure Register */

	if (lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD) /* enable Smart LCD DMA */
		REG_SLCD_CTRL = SLCD_CTRL_DMA_EN;

	switch (lcd_info->panel.cfg & LCD_CFG_MODE_MASK) {
		case LCD_CFG_MODE_GENERIC_TFT:
		case LCD_CFG_MODE_INTER_CCIR656:
		case LCD_CFG_MODE_NONINTER_CCIR656:
		case LCD_CFG_MODE_SLCD:
		default: /* only support TFT16 TFT32, not support STN and Special TFT by now(10-06-2008)*/
			REG_LCD_VAT = ((panel->hsw + panel->blw + panel->w + panel->elw) << 16)
						 | (panel->vsw + panel->bfw + panel->h + panel->efw);
			REG_LCD_DAH = ((panel->hsw + panel->blw) << 16) | (panel->hsw + panel->blw + panel->w);
			REG_LCD_DAV = ((panel->vsw + panel->bfw) << 16) | (panel->vsw + panel->bfw + panel->h);
			REG_LCD_HSYNC = (0 << 16) | panel->hsw;
			REG_LCD_VSYNC = (0 << 16) | panel->vsw;
			break;
	}
}

static void jz4760fb_set_osd_mode(struct jz4760lcd_info *lcd_info)
{
	D("");
	struct jz4760lcd_panel_t *panel = &lcd_info->panel;
	lcd_info->osd.osd_ctrl &= ~(LCD_OSDCTRL_OSDBPP_MASK);
	if (lcd_info->osd.fg1.bpp == 15)
		lcd_info->osd.osd_ctrl |= LCD_OSDCTRL_OSDBPP_15_16 | LCD_OSDCTRL_RGB555;
	else if (lcd_info->osd.fg1.bpp == 16)
		lcd_info->osd.osd_ctrl |= LCD_OSDCTRL_OSDBPP_15_16 | LCD_OSDCTRL_RGB565;
	else {
		lcd_info->osd.fg1.bpp = 32;
		lcd_info->osd.osd_ctrl |= LCD_OSDCTRL_OSDBPP_18_24;
	}

	REG_LCD_OSDC = lcd_info->osd.osd_cfg; /* F0, F1, alpha, */

	REG_LCD_OSDCTRL = lcd_info->osd.osd_ctrl; /* IPUEN, bpp */
	REG_LCD_RGBC = lcd_info->osd.rgb_ctrl;
	REG_LCD_BGC = lcd_info->osd.bgcolor;
	REG_LCD_KEY0 = lcd_info->osd.colorkey0;
	REG_LCD_KEY1 = lcd_info->osd.colorkey1;
	REG_LCD_ALPHA = lcd_info->osd.alpha;

	REG_LCD_IPUR = LCD_IPUR_IPUREN | (panel->blw + panel->w + panel->elw) * panel->vsw / 3;
}

static void jz4760fb_foreground_resize(struct jz4760lcd_info *lcd_info)
{
	D("");
	// int fg0_line_size, fg0_frm_size, fg1_line_size, fg1_frm_size;
	/*
	 * NOTE:
	 * Foreground change sequence:
	 * 	1. Change Position Registers -> LCD_OSDCTL.Change;
	 * 	2. LCD_OSDCTRL.Change -> descripter->Size
	 * Foreground, only one of the following can be change at one time:
	 * 	1. F0 size;
	 *	2. F0 position
	 * 	3. F1 size
	 *	4. F1 position
	 */

	/*
	 * The rules of f0, f1's position:
	 * 	f0.x + f0.w <= panel.w;
	 * 	f0.y + f0.h <= panel.h;
	 *
	 * When output is LCD panel, fg.y and fg.h can be odd number or even number.
	 * When output is TVE, as the TVE has odd frame and even frame,
	 * to simplified operation, fg.y and fg.h should be even number always.
	 */

	/* Foreground 0  */
	if (lcd_info->osd.fg0.x >= lcd_info->panel.w)
		lcd_info->osd.fg0.x = lcd_info->panel.w;
	if (lcd_info->osd.fg0.y >= lcd_info->panel.h)
		lcd_info->osd.fg0.y = lcd_info->panel.h;
	if (lcd_info->osd.fg0.x + lcd_info->osd.fg0.w > lcd_info->panel.w)
		lcd_info->osd.fg0.w = lcd_info->panel.w - lcd_info->osd.fg0.x;
	if (lcd_info->osd.fg0.y + lcd_info->osd.fg0.h > lcd_info->panel.h)
		lcd_info->osd.fg0.h = lcd_info->panel.h - lcd_info->osd.fg0.y;

	// //	fg0_line_size = lcd_info->osd.fg0.w*((lcd_info->osd.fg0.bpp+7)/8);
	// fg0_line_size = (lcd_info->osd.fg0.w * (lcd_info->osd.fg0.bpp) / 8);
	// fg0_line_size = ((fg0_line_size + 3) >> 2) << 2; /* word aligned */
	// fg0_frm_size = fg0_line_size * lcd_info->osd.fg0.h;

	// fg1_line_size = lcd_info->osd.fg1.w * ((lcd_info->osd.fg1.bpp + 7) / 8);
	// fg1_line_size = ((fg1_line_size + 3) >> 2) << 2; /* word aligned */
	// fg1_frm_size = fg1_line_size * lcd_info->osd.fg1.h;

	if (lcd_info->osd.fg_change) {
		if (lcd_info->osd.fg_change & FG0_CHANGE_POSITION) { /* F0 change position */
			REG_LCD_XYP0 = lcd_info->osd.fg0.y << 16 | lcd_info->osd.fg0.x;
		}
		if (lcd_info->osd.fg_change & FG1_CHANGE_POSITION) { /* F1 change position */
			REG_LCD_XYP1 = lcd_info->osd.fg1.y << 16 | lcd_info->osd.fg1.x;
		}

		/* set change */
		if (!(lcd_info->osd.osd_ctrl & LCD_OSDCTRL_IPU) && (lcd_info->osd.fg_change != FG_CHANGE_ALL))
			REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;

		/* wait change ready??? */
		// while ( REG_LCD_OSDS & LCD_OSDS_READY )	/* fix in the future, Wolfgang, 06-20-2008 */
		D("wait LCD_OSDS_READY");

		if (lcd_info->osd.fg_change & FG0_CHANGE_SIZE) { /* change FG0 size */
			REG_LCD_SIZE0 = LCD_SCREEN_H << 16 | LCD_SCREEN_W;
		}

		if (lcd_info->osd.fg_change & FG1_CHANGE_SIZE) { /* change FG1 size */
			REG_LCD_SIZE1 = LCD_SCREEN_H << 16 | LCD_SCREEN_W;
		}

		lcd_info->osd.fg_change = FG_NOCHANGE; /* clear change flag */
	}
}

static void jz4760fb_change_clock(struct jz4760lcd_info *lcd_info)
{
	D("");

#if defined(CONFIG_FPGA)
	REG_LCD_REV = 0x00000004;
	D("Fuwa test, pixclk divide REG_LCD_REV=0x%08x", REG_LCD_REV);
	D("Fuwa test, pixclk %d", JZ_EXTAL / (((REG_LCD_REV & 0xFF) + 1) * 2));
#else
	unsigned int fclk;
	unsigned int pclk;
	/* Timing setting */
	// __cpm_stop_lcd();

#ifdef CONFIG_FB_JZ4760_TVE
	/********* In TVE mode PCLK = 27MHz ***********/
	if (lcd_info->panel.cfg & LCD_CFG_TVEN)
	{ /* LCDC output to TVE */
	   	// __cpm_stop_tve();
		OUTREG32(CPM_CPPCR0, ((90 << CPPCR0_PLLM_LSB) | (2 << CPPCR0_PLLN_LSB) | (1 << CPPCR0_PLLOD_LSB) | (0x20 << CPPCR0_PLLST_LSB) | CPPCR0_PLLEN));
		REG_CPM_LPCDR |= LPCDR_LTCS;
		pclk = 27000000;
		fclk = __cpm_get_pllout2() / pclk; /* pclk */
		fclk--;
		__cpm_set_pixdiv(fclk);
		// __cpm_start_tve();

		D("REG_CPM_LPCDR = 0x%08x", REG_CPM_LPCDR);
		__cpm_select_pixclk_tve();

		REG_CPM_CPCCR |= CPCCR_CE; /* update divide */
	}
	else
#endif /* CONFIG_FB_JZ4760_TVE */
	{ /* LCDC output to  LCD panel */
		fclk = lcd_info->panel.fclk; /* frame clk */

		if ((lcd_info->panel.cfg & LCD_CFG_MODE_MASK) != LCD_CFG_MODE_SERIAL_TFT) {
			pclk = fclk * (lcd_info->panel.w + lcd_info->panel.hsw + lcd_info->panel.elw + lcd_info->panel.blw) * (lcd_info->panel.h + lcd_info->panel.vsw + lcd_info->panel.efw + lcd_info->panel.bfw); /* Pixclk */
		} else { /* serial mode: Hsync period = 3*Width_Pixel */
			pclk = fclk * (lcd_info->panel.w * 3 + lcd_info->panel.hsw + lcd_info->panel.elw + lcd_info->panel.blw) * (lcd_info->panel.h + lcd_info->panel.vsw + lcd_info->panel.efw + lcd_info->panel.bfw); /* Pixclk */
		}

		fclk = __cpm_get_pllout2() / pclk; /* pclk */
		fclk--;

		D("ratio: fclk = %d", fclk);
		if (fclk > 0x7ff) {
			E("pixel clock divide is too large, set it to 0x7ff");
			fclk = 0x7ff;
		}

		__cpm_set_pixdiv(fclk);
		D("REG_CPM_LPCDR = 0x%08x", REG_CPM_LPCDR);
		__cpm_select_pixclk_lcd();
		REG_CPM_CPCCR |= CPCCR_CE; /* update divide */
	}

	D("REG_CPM_LPCDR = 0x%08x", REG_CPM_LPCDR);
	D("REG_CPM_CPCCR = 0x%08x", REG_CPM_CPCCR);

	jz_clocks.pixclk = __cpm_get_pixclk();
	D("[LCDC] PixClock: %d", jz_clocks.pixclk);
	// __cpm_start_lcd();
	// udelay(1000);
	/*
	 * set lcd device clock and lcd pixel clock.
	 * what about TVE mode???
	 *
	 */
#endif /* CONFIG_FPGA */
}

/*
 * jz4760fb_set_mode(), set osd configure, resize foreground
 *
 */
static void jz4760fb_set_mode(struct jz4760lcd_info *lcd_info)
{
	D("");
	struct lcd_cfb_info *cfb = jz4760fb_info;

	jz4760fb_set_osd_mode(lcd_info);
	jz4760fb_foreground_resize(lcd_info);
	jz4760fb_set_var(&cfb->fb.var, -1, &cfb->fb);
}

/*
 * jz4760fb_deep_set_mode
 *
 */
static void jz4760fb_deep_set_mode(struct jz4760lcd_info *lcd_info)
{
	D("");
	/* configurate sequence:
	 * 1. disable lcdc.
	 * 2. init frame descriptor.
	 * 3. set panel mode
	 * 4. set osd mode
	 * 5. start lcd clock in CPM
	 * 6. enable lcdc.
	 */

	__lcd_clr_ena();						 /* Quick Disable */
	lcd_info->osd.fg_change = FG_CHANGE_ALL; /* change FG0, FG1 size, postion??? */
	//jz4760fb_descriptor_init(lcd_info);
	jz4760fb_set_panel_mode(lcd_info);
	jz4760fb_set_mode(lcd_info);
	jz4760fb_change_clock(lcd_info);
	__lcd_set_ena(); /* enable lcdc */
}

static irqreturn_t jz4760fb_interrupt_handler(int irq, void *dev_id)
{
	// D("");
	vsync_count++;
	ipu_update_address();
	wake_up_interruptible_all(&wait_vsync);
	return IRQ_HANDLED;
}

#if defined(CONFIG_PM) && !defined(CONFIG_SOC_JZ4770)

/*
 * Suspend the LCDC.
 */
static int jz4760_fb_suspend(struct platform_device *pdev, pm_message_t state)
{
	D("");
	screen_off();
	//ctrl_disable();
	__lcd_clr_ena();
	__cpm_stop_lcd();
	return 0;
}

/*
 * Resume the LCDC.
 */
static int jz4760_fb_resume(struct platform_device *pdev)
{
	D("");
	__cpm_start_lcd();
	screen_on();
	__lcd_set_ena();
	return 0;
}

#else
	#define jz4760_fb_suspend NULL
	#define jz4760_fb_resume NULL
#endif /* CONFIG_PM */

/* Backlight Control Interface via sysfs
 *
 * LCDC:
 * Enabling LCDC when LCD backlight is off will only affects cfb->display.
 *
 * Backlight:
 * Changing the value of LCD backlight when LCDC is off will only affect the cfb->backlight_level.
 *
 * - River.
 */
static void screen_off(void)
{
	D("");
	struct lcd_cfb_info *cfb = jz4760fb_info;

	__lcd_close_backlight();
	__lcd_display_off();

	#ifdef HAVE_LCD_PWM_CONTROL
		if (cfb->b_lcd_pwm) {
			__lcd_pwm_stop();
			cfb->b_lcd_pwm = 0;
		}
	#endif

	cfb->b_lcd_display = 0;

	mdelay(50);
}

static void screen_on(void)
{
	D("");
	struct lcd_cfb_info *cfb = jz4760fb_info;

	__lcd_display_on();
	cfb->b_lcd_display = 1;
	mdelay(150);
	jz4760fb_set_backlight_level(cfb->backlight_level);
}

void jz4760fb_set_backlight_level(int n)
{
	D("");
	struct lcd_cfb_info *cfb = jz4760fb_info;

#if defined(CONFIG_PM) && !defined(CONFIG_SOC_JZ4770)
	if (n <= -1 && !tve_mode) { // suspend
		if (n == -1) {
			for (n = cfb->backlight_level; n > 0; n--) {
				__lcd_set_backlight_level(n);
				mdelay(3);
			}
		}

		screen_off();
		handle_sysrq('s', NULL); /* Force sync */
		jz_pm_sleep();
		screen_on();
		return;

	} else
#endif // CONFIG_PM
	if (n < 1 || tve_mode) { // Turn off LCD backlight
		screen_off();

	} else { // change the value of backlight when LCDC is enabled
		if (n > LCD_MAX_BACKLIGHT) n = LCD_MAX_BACKLIGHT;
		if (n < LCD_MIN_BACKLIGHT) n = LCD_MIN_BACKLIGHT;

		cfb->backlight_level = n;
		if (!cfb->b_lcd_display) {
			screen_on();
		} else {
			__lcd_set_backlight_level(n);
		}
	}
}

int jz4760fb_get_backlight_level(void)
{
	D("");
	return jz4760fb_info->backlight_level;
}
EXPORT_SYMBOL(jz4760fb_set_backlight_level);
EXPORT_SYMBOL(jz4760fb_get_backlight_level);

static void gpio_init(void)
{
	D("");
	__lcd_display_pin_init();  //LCD REST
#if defined(CONFIG_JZ4760_LCD_RG_V10) || defined(CONFIG_JZ4760_LCD_RG_IPS) || defined(CONFIG_JZ4760_LCD_RG300_V20)
		REG_GPIO_PXFUNS(2) = 0x000c31fc;
		REG_GPIO_PXTRGC(2) = 0x000c31fc;
		REG_GPIO_PXSELC(2) = 0x000c31fc;
		REG_GPIO_PXPES(2)  = 0x000c31fc;
		return;
#endif
	/* gpio init __gpio_as_lcd */
	if (jz4760_lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD)
		__gpio_as_lcd_8bit();
	else if (jz4760_lcd_info->panel.cfg & LCD_CFG_MODE_TFT_16BIT)
		__gpio_as_lcd_16bit();
	else if (jz4760_lcd_info->panel.cfg & LCD_CFG_MODE_TFT_24BIT)
		__gpio_as_lcd_24bit();
	else
 		__gpio_as_lcd_18bit();

	/* In special mode, we only need init special pin,
	 * as general lcd pin has init in uboot */
#if defined(CONFIG_SOC_JZ4760)
	switch (jz4760_lcd_info->panel.cfg & LCD_CFG_MODE_MASK) {
	case LCD_CFG_MODE_SPECIAL_TFT_1:
	case LCD_CFG_MODE_SPECIAL_TFT_2:
	case LCD_CFG_MODE_SPECIAL_TFT_3:
		__gpio_as_lcd_special();
		break;
	default:
		;
	}
#endif

	return;
}

static void set_bpp_to_ctrl_bpp(void)
{
	D("");
	switch (jz4760_lcd_info->osd.fg0.bpp)
	{
		case 15:
		case 16:
			break;

		case 17 ... 32:
			jz4760_lcd_info->osd.fg0.bpp = 32;
			break;

		default:
			E("FG0: BPP (%d) not support, Set BPP 32.", jz4760_lcd_info->osd.fg0.bpp);
			jz4760_lcd_info->osd.fg0.bpp = 32;
			break;
	}

	switch (jz4760_lcd_info->osd.fg1.bpp)
	{
		case 15:
		case 16:
			break;

		case 17 ... 32:
			jz4760_lcd_info->osd.fg1.bpp = 32;
			break;

		default:
			E("FG1: BPP (%d) not support, Set BPP 32.", jz4760_lcd_info->osd.fg1.bpp);
			jz4760_lcd_info->osd.fg1.bpp = 32;
			break;
	}

	return;
}

static void slcd_init(void)
{
	D("");
	/* Configure SLCD module for setting smart lcd control registers */
#if defined(CONFIG_FB_JZ4760_SLCD)
	__lcd_as_smart_lcd();
	__slcd_disable_dma();
	// __init_slcd_bus(); /* Note: modify this depend on you lcd */
#endif
	return;
}

static int jz_backlight_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	return sprintf(page, "%u\n", jz4760fb_info->backlight_level);
}

static int jz_backlight_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	backlight_control = buffer[0] != '-';

	int val = simple_strtol(buffer, 0, 10);

	if (val == 101) { // debug
		fg0_enable(true);
	} else
	if (val == 102) {
		fg0_enable(false);
	}

	jz4760fb_set_backlight_level(val);

	return count;
}

static int jz_ipu_ratio_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	return sprintf(page, "%lu\n", ipu_ratio);
}

static int jz_ipu_ratio_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	ipu_ratio = simple_strtoul(buffer, 0, 10);
	return count;
}

#ifdef VSYNC_OPTION
static int jz_vsync_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	return sprintf(page, "%lu\n", vsync_on);
}

static int jz_vsync_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	vsync_on = !!simple_strtoul(buffer, 0, 10);
	return count;
}
#endif // VSYNC_OPTION

#ifdef CONFIG_FB_JZ4760_TVE
static int jz_tvout_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	return sprintf(page, "%u\n", tve_mode);
}

static int jz_tvout_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	int new_mode = simple_strtoul(buffer, 0, 10);

	if (tve_mode == new_mode) return count;
	tve_mode = new_mode;

	ipu_stop();

	if (tve_mode == PANEL_MODE_LCD_PANEL) {
		jz4760tve_stop(); /* tve controller stop */

		jz4760_lcd_info = &jz4760_lcd_panel;

		/* turn on lcd backlight */
		screen_on();
	} else {
		jz4760_lcd_info = &jz4760_info_tve;

		if (tve_mode == PANEL_MODE_TVE_PAL) {
			jz4760_lcd_info->panel.cfg |= LCD_CFG_TVEPEH; /* TVE PAL enable extra halfline signal */
			jz4760_lcd_info->panel.w = TVE_WIDTH_PAL;
			jz4760_lcd_info->panel.h = TVE_HEIGHT_PAL;
			jz4760_lcd_info->panel.fclk = TVE_FREQ_PAL;
		} else {
			jz4760_lcd_info->panel.cfg &= ~LCD_CFG_TVEPEH; /* TVE NTSC disable extra halfline signal */
			jz4760_lcd_info->panel.w = TVE_WIDTH_NTSC;
			jz4760_lcd_info->panel.h = TVE_HEIGHT_NTSC;
			jz4760_lcd_info->panel.fclk = TVE_FREQ_NTSC;
		}

		jz4760tve_init(tve_mode); /* tve controller init */

		/* turn off lcd backlight */
		screen_off();
	}

	jz4760fb_deep_set_mode(jz4760_lcd_info);

	return count;
}
#endif // CONFIG_FB_JZ4760_TVE

static int __devinit jz4760_fb_probe(struct platform_device *dev)
{
	D("");
	struct lcd_cfb_info *cfb;

	int rv = 0;
	cpm_start_clock(CGM_IPU);
	cfb = jz4760fb_alloc_fb_info();
	if (!cfb)
		goto failed;

	screen_off();

	ctrl_disable();

	gpio_init();
	slcd_init();

	set_bpp_to_ctrl_bpp();
	init_waitqueue_head(&wait_vsync);
	spin_lock_init(&lock);

#if defined(FB_JZ4760_SLCD)
	if (request_irq(IRQ_LCD, jz4760fb_interrupt_handler, IRQF_DISABLED, "lcd", 0)) {
		D("Faield to request LCD IRQ.\n");
		rv = -EBUSY;
		goto failed;
	}
#else
	if (devm_request_irq(&dev->dev, IRQ_IPU, jz4760fb_interrupt_handler, 0, "ipu", cfb)) {
		dev_err(&dev->dev, "Failed to request IRQ.\n");
		goto failed;
	}
#endif

	rv = jz4760fb_map_smem(cfb);
	if (rv)
		goto failed;

	ctrl_enable();

	jz4760fb_deep_set_mode(jz4760_lcd_info);

	ipu_start(jz4760_lcd_info->osd.fg1.w, jz4760_lcd_info->osd.fg1.h, LCD_SCREEN_W, LCD_SCREEN_H);

	rv = register_framebuffer(&cfb->fb);
	if (rv < 0) {
		D("Failed to register framebuffer device.");
		goto failed;
	}
	printk("fb%d: %s frame buffer device, using %d KiB of video memory\n", cfb->fb.node, cfb->fb.fix.id, cfb->fb.fix.smem_len >> 10);

	memset(lcd_frame0, 0x00, LCD_SCREEN_W * LCD_SCREEN_H * 6);
#if defined(JZ_FB_DEBUG)
	display_rgb_bar();
	print_lcdc_registers();
#endif

	screen_on();

	ipu_update_address();

#if defined(JZ_FB_DEBUG)
	mdelay(3000);
#endif
	return 0;

failed:
	jz4760fb_unmap_smem(cfb);
	jz4760fb_free_fb_info(cfb);

	return rv;
}

static int __devexit jz4760_fb_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver jz4760_fb_driver = {
	.probe = jz4760_fb_probe,
	.remove = jz4760_fb_remove,
	.suspend = jz4760_fb_suspend,
	.resume = jz4760_fb_resume,
	.driver = {
		.name = "jz-lcd",
		.owner = THIS_MODULE,
	},
};

static int __init jz4760_fb_init(void)
{
	D("");
	struct proc_dir_entry *res;

	res = create_proc_entry("jz/backlight", 0, NULL);
	if (res)
	{
		res->read_proc = jz_backlight_read_proc;
		res->write_proc = jz_backlight_write_proc;
	}
#ifdef VSYNC_OPTION
	res = create_proc_entry("jz/vsync", 0, NULL);
	if (res)
	{
		res->read_proc = jz_vsync_read_proc;
		res->write_proc = jz_vsync_write_proc;
	}
#endif
	res = create_proc_entry("jz/ipu", 0, NULL);
	if (res)
	{
		res->read_proc = jz_ipu_ratio_read_proc;
		res->write_proc = jz_ipu_ratio_write_proc;
	}
#ifdef CONFIG_FB_JZ4760_TVE
	res = create_proc_entry("jz/tvout", 0, NULL);
	if (res)
	{
		res->read_proc = jz_tvout_read_proc;
		res->write_proc = jz_tvout_write_proc;
	}
#endif
	return platform_driver_register(&jz4760_fb_driver);
}

static void __exit jz4760_fb_cleanup(void)
{
	platform_driver_unregister(&jz4760_fb_driver);
}

module_init(jz4760_fb_init);
module_exit(jz4760_fb_cleanup);
