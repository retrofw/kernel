/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * TVE NTSC Output Routines.
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

#ifdef CONFIG_FB_JZ475X_TVE_NTSC_OUTPUT

#include "tve-common.c"

static struct jz_fb_panel_info tve_ntsc_output_panel_info = {
	.lcd_ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	/* Underrun recover */ 
	.lcd_cfg = LCD_CFG_RECOVER | LCD_CFG_TVEN 
		| LCD_CFG_MODE_INTER_CCIR656,
		TVE_WIDTH_NTSC, TVE_HEIGHT_NTSC, TVE_FREQ_NTSC, 0, 0, 0, 0, 0, 0,
};

/* ------------ CUSTOM: Modify these by your need. ----------- */
struct jz_fb_win_attr_info tve_ntsc_output_win_init_config[] = {
	[0] = {
		.enable		= 1,
		.x		= 0,
		.y		= 0,
		.w		= TVE_WIDTH_NTSC,
		.h		= TVE_HEIGHT_NTSC,
		.bpp		= 32,
	},
	[1] = {
		.enable		= 0,
		.x		= 0,
		.y		= 0,
		.w		= TVE_WIDTH_NTSC,
		.h		= TVE_HEIGHT_NTSC,
		.bpp		= 32,
	},
};
/* ----------------------------------------------------------- */
struct jz_fb_panel_config tve_ntsc_output_panel_config = {
	.win_init_config = tve_ntsc_output_win_init_config,
	.panel = &tve_ntsc_output_panel_info,
	.use_palette = 0,

/* -------------- CUSTOM: Modify these by your need. --------- */
	.max_win_bpp = 32,
	.background_color = 0x000000FF,
/* ----------------------------------------------------------- */
};

#include "tve-common.c"

static void tve_ntsc_output_start(struct jz_fb_ot_info *ot)
{
	jz_fb_tve_set_standard(JZ_FB_TVE_STANDARD_NTSC);

	tve_output_start(ot);

	return;
}

static struct jz_fb_ot_ops tve_ntsc_output_ops = {
	.start = tve_ntsc_output_start,
	.stop = tve_output_stop,
	.control = tve_output_control,
};

static int tve_ntsc_output_init(struct jz_fb_ot_info *ot)
{
	ot->name = "tve-ntsc-ot";

	ot->use_quick_disable = 1;
	ot->use_tve = 1;
	ot->use_rgb_to_yuv = 1;

	ot->desc_ops = &tve_output_desc_ops;
	ot->ops = &tve_ntsc_output_ops;

	ot->config = &tve_ntsc_output_panel_config;
	
	return 0;
}

#endif /* CONFIG_FB_JZ475X_TVE_NTSC_OUTPUT */
