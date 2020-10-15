
/*
 * linux/drivers/video/jz4760_tve.c -- Ingenic Jz4760 TVE Controller operation
 * interface.
 * Copyright (C) 2005-2008, Ingenic Semiconductor Inc.
 * Author: Wolfgang Wang, <lgwang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#include <asm/jzsoc.h>
#include "jz4760_tve.h"
#include "jz4760_lcd.h"

#if 0
struct jz4760tve_info jz4760_tve_info_PAL = {
	.ctrl = TVE_CTRL_ECVBS | (4 << TVE_CTRL_YCDLY_BIT) | TVE_CTRL_SYNCT | TVE_CTRL_PAL | TVE_CTRL_SWRST,	/* PAL, SVIDEO */
	.frcfg = (22 << TVE_FRCFG_L1ST_BIT) | (625 << TVE_FRCFG_NLINE_BIT),
	.slcfg1 = (891<<TVE_SLCFG1_WHITEL_BIT) | (384<<TVE_SLCFG1_BLACKL_BIT),
	.slcfg2 = (378<<TVE_SLCFG2_VBLANKL_BIT) | (378<<TVE_SLCFG2_BLANKL_BIT),
	.slcfg3 = (240 <<TVE_SLCFG3_SYNCL_BIT),
	.ltcfg1 = (19<<TVE_LTCFG1_FRONTP_BIT) | (64<<TVE_LTCFG1_HSYNCW_BIT) | (77<<TVE_LTCFG1_BACKP_BIT),
	.ltcfg2 = (1408 << TVE_LTCFG2_ACTLIN_BIT) | (24 << TVE_LTCFG2_PREBW_BIT) | (68 << TVE_LTCFG2_BURSTW_BIT),
	.cfreq = 0x2a098acb,
	.cphase = (0 << TVE_CPHASE_INITPH_BIT) | (0 << TVE_CPHASE_ACTPH_BIT) | (1 << TVE_CPHASE_CCRSTP_BIT),
	.cbcrcfg = (23<<TVE_CBCRCFG_CBBA_BIT) | (23<<TVE_CBCRCFG_CRBA_BIT) | (137<<TVE_CBCRCFG_CBGAIN_BIT) | (137<<TVE_CBCRCFG_CRGAIN_BIT), /* CBGAIN CRGAIN??? */
	.wsscr = 0x00000070,	/* default value */
	.wsscfg1 = 0x0,
	.wsscfg2 = 0x0,
	.wsscfg3 = 0x0,
};
#else
struct jz4760tve_info jz4760_tve_info_PAL = {
	.ctrl = TVE_CTRL_ECVBS | (4 << TVE_CTRL_YCDLY_BIT) | TVE_CTRL_SYNCT | TVE_CTRL_PAL | TVE_CTRL_SWRST,	/* PAL, SVIDEO */
	.frcfg = (23 << TVE_FRCFG_L1ST_BIT) | (625 << TVE_FRCFG_NLINE_BIT),
	.slcfg1 = (800<<TVE_SLCFG1_WHITEL_BIT) | (282<<TVE_SLCFG1_BLACKL_BIT),
	.slcfg2 = (296<<TVE_SLCFG2_VBLANKL_BIT) | (240<<TVE_SLCFG2_BLANKL_BIT),
	.slcfg3 = (72 <<TVE_SLCFG3_SYNCL_BIT),
	.ltcfg1 = (20<<TVE_LTCFG1_FRONTP_BIT) | (63<<TVE_LTCFG1_HSYNCW_BIT) | (78<<TVE_LTCFG1_BACKP_BIT),
	.ltcfg2 = (1440 << TVE_LTCFG2_ACTLIN_BIT) | (24 << TVE_LTCFG2_PREBW_BIT) | (68 << TVE_LTCFG2_BURSTW_BIT),
	.cfreq = 0x2a098acb,
	.cphase = (0 << TVE_CPHASE_INITPH_BIT) | (0 << TVE_CPHASE_ACTPH_BIT) | (1 << TVE_CPHASE_CCRSTP_BIT),
	.cbcrcfg = (32<<TVE_CBCRCFG_CBBA_BIT) | (59<<TVE_CBCRCFG_CRBA_BIT) | (137<<TVE_CBCRCFG_CBGAIN_BIT) | (137<<TVE_CBCRCFG_CRGAIN_BIT), /* CBGAIN CRGAIN??? */
	.wsscr = 0x00000070,	/* default value */
	.wsscfg1 = 0x0,
	.wsscfg2 = 0x0,
	.wsscfg3 = 0x0,
};
#endif

struct jz4760tve_info jz4760_tve_info_NTSC = {
	.ctrl = TVE_CTRL_ECVBS | (4 << TVE_CTRL_YCDLY_BIT) | TVE_CTRL_SWRST,	/* NTSC, SVIDEO */
	.frcfg = (21 << TVE_FRCFG_L1ST_BIT) | (525 << TVE_FRCFG_NLINE_BIT),
	.slcfg1 = (800<<TVE_SLCFG1_WHITEL_BIT) | (282<<TVE_SLCFG1_BLACKL_BIT),
	.slcfg2 = (296<<TVE_SLCFG2_VBLANKL_BIT) | (240<<TVE_SLCFG2_BLANKL_BIT),
	.slcfg3 = (72 <<TVE_SLCFG3_SYNCL_BIT),
	.ltcfg1 = (16<<TVE_LTCFG1_FRONTP_BIT) | (63<<TVE_LTCFG1_HSYNCW_BIT) | (59<<TVE_LTCFG1_BACKP_BIT),
	.ltcfg2 = (1440 << TVE_LTCFG2_ACTLIN_BIT) | (22 << TVE_LTCFG2_PREBW_BIT) | (68 << TVE_LTCFG2_BURSTW_BIT),
	.cfreq = 0x21f07c1f,
	.cphase = (0x17 << TVE_CPHASE_INITPH_BIT) | (0 << TVE_CPHASE_ACTPH_BIT) | (1 << TVE_CPHASE_CCRSTP_BIT),
	.cbcrcfg = (59<<TVE_CBCRCFG_CBBA_BIT) | (0<<TVE_CBCRCFG_CRBA_BIT) | (137<<TVE_CBCRCFG_CBGAIN_BIT) | (137<<TVE_CBCRCFG_CRGAIN_BIT),
	.wsscr = 0x00000070,	/* default value */
	.wsscfg1 = 0x0,
	.wsscfg2 = 0x0,
	.wsscfg3 = 0x0,
};

struct jz4760tve_info *jz4760_tve_info = &jz4760_tve_info_PAL; /* default as PAL mode */

void jz4760tve_enable_tve(void)
{
	/* enable tve controller, enable DACn??? */
	jz4760_tve_info->ctrl = (jz4760_tve_info->ctrl | TVE_CTRL_DAPD) & ( ~( TVE_CTRL_DAPD1 | TVE_CTRL_DAPD2));
	jz4760_tve_info->ctrl &= ~TVE_CTRL_SWRST;
	REG_TVE_CTRL = jz4760_tve_info->ctrl;
}

/* turn off TVE, turn off DACn... */
void jz4760tve_disable_tve(void)
{
	jz4760_tve_info->ctrl &= ~TVE_CTRL_DAPD;/* DACn disabled??? */
	jz4760_tve_info->ctrl |= TVE_CTRL_SWRST;/* DACn disabled??? */
	REG_TVE_CTRL = jz4760_tve_info->ctrl;
}

void jz4760tve_set_tve_mode( struct jz4760tve_info *tve )
{
	REG_TVE_CTRL 		= tve->ctrl;
	REG_TVE_FRCFG 		= tve->frcfg;
	REG_TVE_SLCFG1 		= tve->slcfg1;
	REG_TVE_SLCFG2 		= tve->slcfg2;
	REG_TVE_SLCFG3 		= tve->slcfg3;
	REG_TVE_LTCFG1 		= tve->ltcfg1;
	REG_TVE_LTCFG2 		= tve->ltcfg2;
	REG_TVE_CFREQ 		= tve->cfreq;
	REG_TVE_CPHASE 		= tve->cphase;
	REG_TVE_CBCRCFG 	= tve->cbcrcfg;
	REG_TVE_WSSCR 		= tve->wsscr;
	REG_TVE_WSSCFG1 	= tve->wsscfg1;
	REG_TVE_WSSCFG2 	= tve->wsscfg2;
	REG_TVE_WSSCFG3 	= tve->wsscfg3;
}

void jz4760tve_init( int tve_mode )
{
	switch ( tve_mode ) {
	case PANEL_MODE_TVE_PAL:
		jz4760_tve_info = &jz4760_tve_info_PAL;
		break;
	case PANEL_MODE_TVE_NTSC:
		jz4760_tve_info = &jz4760_tve_info_NTSC;
		break;
	}

	jz4760tve_set_tve_mode( jz4760_tve_info );

	udelay(100);
	cpm_start_clock(CGM_TVE);

	jz4760tve_enable_tve();
}

void jz4760tve_stop(void)
{
	jz4760tve_disable_tve();
	cpm_stop_clock(CGM_TVE);
}

struct jz4760lcd_info jz4760_info_tve = {
	.panel = {
		.cfg = LCD_CFG_TVEN |					// output to tve
			   LCD_CFG_NEWDES |					// 8words descriptor
			   LCD_CFG_RECOVER |				// underrun protect
			   LCD_CFG_MODE_INTER_CCIR656 |		// Interlace CCIR656 mode
			   LCD_CFG_TVEPEH,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,// 16words burst
		TVE_WIDTH_PAL, TVE_HEIGHT_PAL, TVE_FREQ_PAL, 0, 0, 0, 0, 0, 0,
	},
	.osd = {
		.osd_cfg = LCD_OSDC_OSDEN |			// Use OSD mode
				   // LCD_OSDC_ALPHAEN |	// enable alpha
				   // LCD_OSDC_F0EN,		// enable Foreground0
				   LCD_OSDC_F1EN,			// enable Foreground1
		.osd_ctrl = LCD_OSDCTRL_IPU, // |	// enable ipu
				    // LCD_OSDCTRL_OSDBPP_15_16,
		// .osd_ctrl = 0,					// disable ipu
		.rgb_ctrl = LCD_RGBC_YCC,			// enable RGB => YUV
		.bgcolor = 0x00000000,				// set background color Black
		.colorkey0 = 0x80000000,			// disable colorkey
		.colorkey1 = 0x80000000,			// disable colorkey
		.alpha = 0xA0,						// alpha value
		.ipu_restart = 0x80000100,			// ipu restart
		.fg_change = FG_CHANGE_ALL,			// change all initially
		.fg0 = {16, 0, 0, 320, 240},		// bpp, x, y, w, h
		.fg1 = {16, 0, 0, 320, 240},		// bpp, x, y, w, h
	},
};
