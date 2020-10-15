/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * Output Config.
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
static struct jz_fb_ot_info jz_fb_lcd_ot_info;

static int lcd_output_init(struct jz_fb_ot_info *);
#endif

#ifdef CONFIG_FB_JZ475X_TVE_PAL_OUTPUT
static struct jz_fb_ot_info jz_fb_tve_pal_ot_info;

static int tve_pal_output_init(struct jz_fb_ot_info *);
#endif

#ifdef CONFIG_FB_JZ475X_TVE_NTSC_OUTPUT
static struct jz_fb_ot_info jz_fb_tve_ntsc_ot_info;

static int tve_ntsc_output_init(struct jz_fb_ot_info *);
#endif

static struct jz_fb_ot_info *jz_fb_ots [] = {

#ifdef CONFIG_FB_JZ475X_LCD_OUTPUT
	&jz_fb_lcd_ot_info,
#endif

#ifdef CONFIG_FB_JZ475X_TVE_PAL_OUTPUT
	&jz_fb_tve_pal_ot_info,
#endif

#ifdef CONFIG_FB_JZ475X_TVE_NTSC_OUTPUT
	&jz_fb_tve_ntsc_ot_info,
#endif
	
};

typedef int jz_fb_ot_init_t(struct jz_fb_ot_info *);

static jz_fb_ot_init_t *jz_fb_ot_inits [] = {

#ifdef CONFIG_FB_JZ475X_LCD_OUTPUT
	&lcd_output_init,
#endif

#ifdef CONFIG_FB_JZ475X_TVE_PAL_OUTPUT
	&tve_pal_output_init,
#endif

#ifdef CONFIG_FB_JZ475X_TVE_NTSC_OUTPUT
	&tve_ntsc_output_init,
#endif
	
};

