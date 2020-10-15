/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * Linux Framebuffer Routines.
 * 
 * Copyright (C) 2005-2010 Ingenic Semiconductor Inc.
 * Author: River Wang <zwang@ingenic.cn>
 *		
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

static inline u_int chan_to_field(u_int chan, struct fb_bitfield *bf)
{
        chan &= 0xffff;
        chan >>= 16 - bf->length;
        return chan << bf->offset;
}

static int jz_fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			  u_int transp, struct fb_info *info)
{
	struct jz_fb_win_info *win = container_of(info, struct jz_fb_win_info, fb);
	struct jz_fb_panel_info *panel = win->ctrl->ot->config->panel;

	unsigned short *ptr, ctmp;

#if 0	/* REVISIT. */
	if (regno >= NR_PALETTE)
		return 1;
#endif

	if (info->var.bits_per_pixel <= 16) {
		red	>>= 8;
		green	>>= 8;
		blue	>>= 8;

		red	&= 0xff;
		green	&= 0xff;
		blue	&= 0xff;
	}

	switch (info->var.bits_per_pixel) {
	case 1:
	case 2:
	case 4:
	case 8:
		if (((panel->lcd_cfg & LCD_CFG_MODE_MASK) == LCD_CFG_MODE_SINGLE_MSTN ) ||
				((panel->lcd_cfg & LCD_CFG_MODE_MASK) == LCD_CFG_MODE_DUAL_MSTN )) {
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

		ptr = (unsigned short *)win->dma_palette;
		ptr = (unsigned short *)(((u32)ptr)|0xa0000000);
		ptr[regno] = ctmp;

		break;
		
	case 15:
		if (regno < 16)
			((u32 *)info->pseudo_palette)[regno] =
				((red >> 3) << 10) | 
				((green >> 3) << 5) |
				(blue >> 3);
		break;
	case 16:
		if (regno < 16) {
			((u32 *)info->pseudo_palette)[regno] =
				((red >> 3) << 11) | 
				((green >> 2) << 5) |
				(blue >> 3); 
		}
		break;
	case 17 ... 32:
		if (regno < 16)
			((u32 *)info->pseudo_palette)[regno] =
				(red << 16) | 
				(green << 8) |
				(blue << 0); 
		break;
	}

	return 0;
}

static int jz_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	unsigned long start;
	unsigned long off;
	u32 len;

	D("Called.");

	off = vma->vm_pgoff << PAGE_SHIFT;

	/* frame buffer memory */
	start = info->fix.smem_start;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + info->fix.smem_len);
	start &= PAGE_MASK;

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;

	off += start;

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);	/* Uncacheable */

 	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
 	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;		/* Uncacheable */

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}

static int jz_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	D("Called.");

	return 0;
}

static int jz_fb_set_par(struct fb_info *info)
{
	D("Called.");

	return 0;
}

static int jz_fb_blank(int blank_mode, struct fb_info *info)
{
	struct jz_fb_ctrl *ctrl = &jz_fb_ctrl;

	D("Called.");

	switch (blank_mode) {
		case FB_BLANK_UNBLANK:
			jz_fb_hw_ot_power_on(ctrl, 1);
			jz_fb_hw_ctrl_enable(ctrl, 1);
			jz_fb_hw_ot_enable(ctrl, 1);

			break;

		case FB_BLANK_NORMAL:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_POWERDOWN:
			jz_fb_hw_ot_disable(ctrl, 1);
			jz_fb_hw_ctrl_disable(ctrl, 1);
			jz_fb_hw_ot_power_off(ctrl, 1);


			break;

		default:
			E("Unsupport blank mode.");
			break;

	}

	return 0;
}

static int jz_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	D("Called.");

	return 0;
}

/* use default function cfb_fillrect, cfb_copyarea, cfb_imageblit */
static struct fb_ops jz_fb_ops = {
	.owner			= THIS_MODULE,
	.fb_setcolreg		= jz_fb_setcolreg,
	.fb_check_var 		= jz_fb_check_var,
	.fb_set_par 		= jz_fb_set_par,
	.fb_blank		= jz_fb_blank,
	.fb_pan_display		= jz_fb_pan_display,
	.fb_fillrect		= cfb_fillrect,
	.fb_copyarea		= cfb_copyarea,
	.fb_imageblit		= cfb_imageblit,
	.fb_mmap		= jz_fb_mmap,
};

static irqreturn_t jz_fb_interrupt_handler(int irq, void *dev_id)
{
	unsigned int state;
	static int irqcnt=0;

	state = REG_LCD_STATE;
	D("In the lcd interrupt handler, state=0x%x", state);

	if (state & LCD_STATE_EOF) /* End of frame */
		REG_LCD_STATE = state & ~LCD_STATE_EOF;

	if (state & LCD_STATE_IFU0) {
		printk("%s, InFiFo0 underrun\n", __FUNCTION__);
		REG_LCD_STATE = state & ~LCD_STATE_IFU0;
	}

	if (state & LCD_STATE_IFU1) {
		printk("%s, InFiFo1 underrun\n", __FUNCTION__);
		REG_LCD_STATE = state & ~LCD_STATE_IFU1;
	}

	if (state & LCD_STATE_OFU) { /* Out fifo underrun */
		REG_LCD_STATE = state & ~LCD_STATE_OFU;
		if ( irqcnt++ > 100 ) {
			__lcd_disable_ofu_intr();
			printk("disable Out FiFo underrun irq.\n");
		}
		printk("%s, Out FiFo underrun.\n", __FUNCTION__);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_PM

/*
 * Suspend the LCDC.
 */
static int jz_fb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct jz_fb_ctrl *ctrl = &jz_fb_ctrl;

	D("Called.");

	jz_fb_hw_ot_disable(ctrl, 1);
	jz_fb_hw_ctrl_disable(ctrl, 1);
	jz_fb_hw_ot_power_off(ctrl, 1);

	return 0;
}

/*
 * Resume the LCDC.
 */
static int jz_fb_resume(struct platform_device *pdev)
{
	struct jz_fb_ctrl *ctrl = &jz_fb_ctrl;

	D("Called.");
	
	jz_fb_hw_ot_power_on(ctrl, 1);
	jz_fb_hw_ctrl_enable(ctrl, 1);
	jz_fb_hw_ot_enable(ctrl, 1);

	return 0;
}

#else
#define jzfb_suspend      NULL
#define jzfb_resume       NULL
#endif /* CONFIG_PM */

static int fb_drv_setup(struct jz_fb_win_info *win)
{
	int rv;

	dump_fb_info_fix(&win->fb.fix);	
	dump_fb_info_var(&win->fb.var);	

	rv = register_framebuffer(&win->fb);
	if (rv) {
		E("Failed to register Linux Framebuferr Driver.");
		return rv;
	}

	return rv;
}

static int fb_drv_cleanup(struct jz_fb_win_info *win)
{
	return 0;	
}

