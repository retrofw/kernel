/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * LCD Output Main Routines.
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

#ifdef CONFIG_FB_JZ475X_LCD_OUTPUT

/* -------- CUSTOM: Add your specific panel here. ----------- */
#include "auo-a043fl01v2.c"
/* ---------------------------------------------------------- */

static int lcd_output_desc_setup(struct jz_fb_win_info *win)
{
	struct jz_fb_panel_config *c = win->ctrl->ot->config;
	struct jz_fb_dma_desc *desc;
	
	D("Called.");	

	/* Setup DMA Palette Descriptor. */
	if (c->use_palette && win->index == 0) { /* Palette cannot be used on FG1. */
		unsigned long palette_size = 
			jz_fb_hw_bpp_to_palette_size(win->attr.bpp);

		D("Use Palette.");

		desc = dma_desc_get(win);
		if (!desc) {
			return -ENOMEM;
		}	

		dma_desc_init(desc);

		dma_desc_set_mem(desc, win->dma_palette);
		dma_desc_set_data_size(desc, palette_size);

		dma_desc_set_as_palette(desc);

		/* Set as DMA Descriptor Chain Head. */
		dma_desc_set_chain_head(desc);
		
		win->dma_desc_last_no_data = desc;
	}

	return 0;
}

static void lcd_output_desc_cleanup(struct jz_fb_win_info *win)
{
	/* Clean up ALL. */
	dma_desc_pool_init(win);

	return;
}

/* Dynamically Attach / Detach Framebuffer */
/* LCD Output: 1 Descriptor -> 1 Framebuffer. */
static int lcd_output_desc_attach(struct jz_fb_win_info *win, void *mem)
{
	/* Get new descriptor. */
	struct jz_fb_dma_desc *desc = dma_desc_get(win);
	struct jz_fb_win_attr_info *a = &win->attr;

	D("Called.");

	if (!desc) {
		return -ENOMEM;
	}

	dma_desc_init(desc);

	/* Attach framebuffer to desciptors. */
	dma_desc_set_mem(desc, mem);
	dma_desc_set_data_size(desc, win->fb_frame_len);

	dma_desc_set_area_size(desc, a->w, a->h);
	
	dma_desc_add_to_chain(desc);

	dump_desc(desc);

	return 0;
}

static int lcd_output_desc_detach(struct jz_fb_win_info *win, void *mem)
{
	struct jz_fb_dma_desc *desc;

	/* Find descriptor by mem pointer. */
	desc = dma_desc_mem_to_desc(win, mem);
	if (!desc) {
		return -ENODEV;
	}
	
	dma_desc_del_from_chain(desc);

	/* Release descriptor. */
	dma_desc_put(desc);
	
	return 0;
}

static struct jz_fb_ot_desc_ops lcd_output_desc_ops = {
	.setup = lcd_output_desc_setup,
	.attach = lcd_output_desc_attach,
	.detach = lcd_output_desc_detach,
	.cleanup = lcd_output_desc_cleanup,
};

static void lcd_output_enable(struct jz_fb_ot_info *ot)
{
	struct jz_fb_panel_config *c = ot->config;

	/* Call specific panel screen on routines. */
	c->ops->on(ot);

	return;
}

static void lcd_output_disable(struct jz_fb_ot_info *ot)
{
	struct jz_fb_panel_config *c = ot->config;

	/* Call specific panel screen off routines. */
	c->ops->off(ot);

	return;
}

static void lcd_output_start(struct jz_fb_ot_info *ot)
{
	struct jz_fb_panel_config *c = ot->config;
	
	/* Call specific panel working routines. */
	c->ops->normal(ot);

	return;
}

static void lcd_output_stop(struct jz_fb_ot_info *ot)
{
	struct jz_fb_panel_config *c = ot->config;

	/* Call specific panel suspend routines. */
	c->ops->idle(ot);

	return;
}

#include "lcd-control.c"

static struct jz_fb_ot_ops lcd_output_ops = {
	.enable = lcd_output_enable,
	.disable = lcd_output_disable,
	.start = lcd_output_start,
	.stop = lcd_output_stop,
	.control = lcd_output_control,
};

static int lcd_output_init(struct jz_fb_ot_info *ot)
{
	ot->name = "lcd-ot";

	ot->use_quick_disable = 0;
	ot->use_tve = 0;
	ot->use_rgb_to_yuv = 0;

	ot->desc_ops = &lcd_output_desc_ops;
	ot->ops = &lcd_output_ops;

	ot->config = &lcd_output_panel_config;
	
	return 0;
}

#endif /* CONFIG_FB_JZ475X_LCD_OUTPUT */
