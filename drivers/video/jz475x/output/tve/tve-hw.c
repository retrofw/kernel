/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * TVE Controller Routines. 
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

/* TV parameter */
#define TVE_WIDTH_PAL 		720
#define TVE_HEIGHT_PAL 		573
#define TVE_FREQ_PAL 		50
#define TVE_WIDTH_NTSC 		720
#define TVE_HEIGHT_NTSC 	482
#define TVE_FREQ_NTSC 		60

struct jz_fb_tve_regs {
	unsigned int ctrl;
	unsigned int frcfg;
	unsigned int slcfg1;
	unsigned int slcfg2;
	unsigned int slcfg3;
	unsigned int ltcfg1;
	unsigned int ltcfg2;
	unsigned int cfreq;
	unsigned int cphase;
	unsigned int cbcrcfg;
	unsigned int wsscr;
	unsigned int wsscfg1;
	unsigned int wsscfg2;
	unsigned int wsscfg3;
};

struct jz_fb_tve_info {
	int output;
	int standard;
	int b_enable;

	struct jz_fb_tve_regs regs;
};

enum {
	JZ_FB_TVE_STANDARD_PAL = 0,
	JZ_FB_TVE_STANDARD_NTSC,
};

static struct jz_fb_tve_info g_tve_info;

static struct jz_fb_tve_regs tve_pal_regs_config = {
	.ctrl = (4 << TVE_CTRL_YCDLY_BIT) | TVE_CTRL_SYNCT | TVE_CTRL_PAL | TVE_CTRL_SWRST,
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

struct jz_fb_tve_regs tve_ntsc_regs_config = {
	.ctrl = (4 << TVE_CTRL_YCDLY_BIT) | TVE_CTRL_SWRST,
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

static void jz_fb_tve_write_regs(struct jz_fb_tve_regs *tve)
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

static void jz_fb_tve_dac_power_on(struct jz_fb_tve_info *info)
{
	unsigned long v = REG_TVE_CTRL;

	v &= ~(TVE_CTRL_DAPD | TVE_CTRL_DAPD1 | TVE_CTRL_DAPD2);

	switch (info->output) {
		case JZ_FB_TVE_PATH_CVBS:
			v |= TVE_CTRL_DAPD;
			break;

		case JZ_FB_TVE_PATH_SVIDEO:

#ifdef CONFIG_SOC_JZ4750D
		case JZ_FB_TVE_PATH_YUV:
#endif
			v |= (TVE_CTRL_DAPD | TVE_CTRL_DAPD1 | TVE_CTRL_DAPD2);
			break;

	}
	
	REG_TVE_CTRL = v;

	return;
}

static void jz_fb_tve_dac_power_off(struct jz_fb_tve_info *info)
{
	REG_TVE_CTRL &= ~(TVE_CTRL_DAPD | TVE_CTRL_DAPD1 | TVE_CTRL_DAPD2);

	return;
}

static void jz_fb_tve_load_standard(struct jz_fb_tve_info *info)
{
	switch (info->standard) {
		case JZ_FB_TVE_STANDARD_PAL:
			memcpy(&info->regs, 
				&tve_pal_regs_config, sizeof(struct jz_fb_tve_regs));
			break;

		case JZ_FB_TVE_STANDARD_NTSC:
			memcpy(&info->regs, 
				&tve_ntsc_regs_config, sizeof(struct jz_fb_tve_regs));
			break;
	}

	return;
}

static void jz_fb_tve_select_output(struct jz_fb_tve_info *info)
{
	switch (info->output) {

#ifdef CONFIG_SOC_JZ4750D		
		case JZ_FB_TVE_PATH_YUV:
			info->regs.ctrl |= TVE_CTRL_EYCBCR;
			break;
#endif

		case JZ_FB_TVE_PATH_CVBS:
			info->regs.ctrl |= TVE_CTRL_ECVBS;
			break;

		case JZ_FB_TVE_PATH_SVIDEO:
			break;
	}
	
	return;
}

static void jz_fb_tve_start(void)
{
	struct jz_fb_tve_info *info = &g_tve_info;
	
	jz_fb_tve_load_standard(info);
	jz_fb_tve_select_output(info);

	jz_fb_tve_write_regs(&info->regs);

	jz_fb_tve_dac_power_on(info);
	REG_TVE_CTRL &= ~TVE_CTRL_SWRST;
	
	info->b_enable = 1;
	
	return;
}

static void jz_fb_tve_stop(void)
{
	struct jz_fb_tve_info *info = &g_tve_info;
	
	jz_fb_tve_dac_power_off(info);
	REG_TVE_CTRL |= TVE_CTRL_SWRST;

	info->b_enable = 0;

	return;
}

static int jz_fb_tve_set_standard(int standard)
{
	struct jz_fb_tve_info *info = &g_tve_info;

	int rv = 0;

	switch (standard) {
		case JZ_FB_TVE_STANDARD_PAL:
		case JZ_FB_TVE_STANDARD_NTSC:
			info->standard = standard;

			break;
		default:
			rv = -EINVAL;

			break;
	}

	return rv;
}

static int jz_fb_tve_valid_output(int output)
{
	int rv = 0;

	switch (output) {
		case JZ_FB_TVE_PATH_CVBS:
		case JZ_FB_TVE_PATH_SVIDEO:

#ifdef CONFIG_SOC_JZ4750D
		case JZ_FB_TVE_PATH_YUV:
#endif
			break;
		default:
			rv = -EINVAL;

			break;
	}

	return rv;
}

static int jz_fb_tve_set_output(int output)
{
	struct jz_fb_tve_info *info = &g_tve_info;

	int rv;

	rv = jz_fb_tve_valid_output(output);
	if (rv)
		return rv;

	info->output = output;

	return 0;
}

