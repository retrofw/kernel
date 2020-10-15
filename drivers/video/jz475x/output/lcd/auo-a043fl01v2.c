/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * LCD PANEL: AUO A043FL01V2 on APUS / CETUS Routines.
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

#ifdef CONFIG_FB_JZ475X_LCD_PANEL_AUO_A043FL01V2
static struct jz_fb_panel_info auo_panel_info = {
	.lcd_ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,
	.lcd_cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER	/* Underrun recover */ 

	/* ---------------- CUSTOM: Modify these by your panel. -----------*/
		| LCD_CFG_MODE_GENERIC_TFT		/* General TFT panel */
		| LCD_CFG_MODE_TFT_24BIT	 	/* output 18bpp */
		| LCD_CFG_HSP				/* Hsync polarity: active low */
		| LCD_CFG_VSP,	/* Vsync polarity: leading edge is falling edge */
	480, 272, 60, 41, 10, 8, 4, 4, 2,
	/* --------------------------------------------------------------- */
};

/* ------ CUSTOM: Modify these by your board. --------- */
/* Get Backlight Level. */
static int auo_panel_get_backlight(struct jz_fb_ot_info *ot)
{
	D("Called.");

	return ot->config->backlight;
}

/* Set Backlight & PWM. */
static int auo_panel_set_backlight(struct jz_fb_ot_info *ot, int backlight)
{
	D("Called.");

	ot->config->backlight = backlight;

	return 0;
}

/* 
   For Display ON / OFF. 
   Enablie / Disable Backlight / PWM Here.
*/
static int auo_panel_on(struct jz_fb_ot_info *ot)
{
	D("Called.");
	
	__gpio_set_pin(GPIO_LCD_PWM);	
	__gpio_as_output(GPIO_LCD_PWM);	

	return 0;
}

static int auo_panel_off(struct jz_fb_ot_info *ot)
{
	D("Called.");

	__gpio_clear_pin(GPIO_LCD_PWM);
	__gpio_as_output(GPIO_LCD_PWM);	

	return 0;
}

/* 
   For PM / Init Routines. 
   Set GPIO Pins to normal/idle state. 
   Enable / Disable VCC_EN/PWM etc..
 */

static int auo_panel_normal(struct jz_fb_ot_info *ot)
{
	D("Called.");
	
	__gpio_as_lcd_24bit();

#ifdef CONFIG_JZ4750_APUS
	__gpio_clear_pin(GPIO_LCD_VCC_EN_N);
#endif 

#ifdef CONFIG_JZ4750D_CETUS
	__gpio_set_pin(GPIO_LCD_VCC_EN_N);
#endif

	__gpio_as_output(GPIO_LCD_VCC_EN_N);
	
	return 0;
}

static int auo_panel_idle(struct jz_fb_ot_info *ot)
{
	D("Called.");

#ifdef CONFIG_JZ4750_APUS
	__gpio_set_pin(GPIO_LCD_VCC_EN_N);
#endif

#ifdef CONFIG_JZ4750D_CETUS
	__gpio_clear_pin(GPIO_LCD_VCC_EN_N);
#endif
	__gpio_as_output(GPIO_LCD_VCC_EN_N);

	return 0;
}
/* ---------------------------------------------------------- */

static struct jz_fb_panel_ops auo_panel_ops = {
	.get_backlight = auo_panel_get_backlight,
	.set_backlight = auo_panel_set_backlight,

	.normal = auo_panel_normal,
	.idle	= auo_panel_idle,

	.on	= auo_panel_on,
	.off	= auo_panel_off,
};

/* ------------ CUSTOM: Modify these by your need. ----------- */
struct jz_fb_win_attr_info auo_win_init_config[] = {
	[0] = {
		.enable		= 1,
		.x		= 0,
		.y		= 0,
		.w		= 480,
		.h		= 272,
		.bpp		= 32,
	},
	[1] = {
		.enable		= 1,
		.x		= 0,
		.y		= 0,
		.w		= 480,
		.h		= 272,
		.bpp		= 32,
	},
};
/* ----------------------------------------------------------- */

struct jz_fb_panel_config lcd_output_panel_config = {
	.ops = &auo_panel_ops,
	.panel = &auo_panel_info,
	.win_init_config = auo_win_init_config,

/* -------------- CUSTOM: Modify these by your need. --------- */
	.enable_delay_ms = 80,	/* Delay in ms before backlight is enabled. */
	.backlight = 80,
	.use_palette = 0,
	.max_win_bpp = 32,
	.background_color = 0x000000FF,
/* ----------------------------------------------------------- */
};

#endif	/* Define CONFIG_FB_JZ475X_LCD_PANEL_AUO_A043FL01V2 */
