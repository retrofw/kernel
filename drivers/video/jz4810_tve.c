
/*
 * linux/drivers/video/jz4810_tve.c -- Ingenic Jz4810 TVE Controller operation
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
#include "jz4810_tve.h"

struct jz4810tve_info jz4810_tve_info_PAL = {
	.ctrl = (4 << TVE_CTRL_YCDLY_BIT) | TVE_CTRL_SYNCT | TVE_CTRL_PAL | TVE_CTRL_SWRST,	/* PAL, SVIDEO */
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

struct jz4810tve_info jz4810_tve_info_NTSC = {
	.ctrl = (4 << TVE_CTRL_YCDLY_BIT) | TVE_CTRL_SWRST,	/* NTSC, SVIDEO */
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

struct jz4810tve_info *jz4810_tve_info = &jz4810_tve_info_PAL; /* default as PAL mode */

void jz4810tve_enable_tve(void)
{
	/* enable tve controller, enable DACn??? */
	jz4810_tve_info->ctrl = (jz4810_tve_info->ctrl | TVE_CTRL_DAPD) & ( ~( TVE_CTRL_DAPD1 | TVE_CTRL_DAPD2));
	jz4810_tve_info->ctrl &= ~TVE_CTRL_SWRST;
	REG_TVE_CTRL = jz4810_tve_info->ctrl;
}

/* turn off TVE, turn off DACn... */
void jz4810tve_disable_tve(void)
{
	jz4810_tve_info->ctrl &= ~TVE_CTRL_DAPD;/* DACn disabled??? */
	jz4810_tve_info->ctrl |= TVE_CTRL_SWRST;/* DACn disabled??? */
	REG_TVE_CTRL = jz4810_tve_info->ctrl;
}

void jz4810tve_set_tve_mode( struct jz4810tve_info *tve )
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

void jz4810tve_init( int tve_mode )
{
	switch ( tve_mode ) {
	case PANEL_MODE_TVE_PAL:
		jz4810_tve_info = &jz4810_tve_info_PAL;
		break;
	case PANEL_MODE_TVE_NTSC:
		jz4810_tve_info = &jz4810_tve_info_NTSC;
		break;
	}

	jz4810tve_set_tve_mode( jz4810_tve_info );
//	jz4810tve_enable_tve();
}
