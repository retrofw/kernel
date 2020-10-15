/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * HW Controller Routines.
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

static int jz_fb_hw_bpp_to_palette_size(unsigned int bpp)
{
	int size;

	/* More details can be found in LCDC Spec. */
	switch (bpp) {
		case 1:
			size = 1;	/* 1 word. */
			break;
		case 2:
			size = 2;	/* 2 words. */
			break;
		case 4:
			size = 8;	/* 8 words. */
			break;
		case 8:
			size = 128;	/* 128 words. */
			break;

		default:
			return -1;
	}

	return size * 4;	
}

static void jz_fb_hw_ot_power_on(struct jz_fb_ctrl *ctrl, int force)
{
	struct jz_fb_ot_info *ot = ctrl->ot;

	D("Called.");
	
	if (!force && ot->power) {
		D("Already Power ON.");
		return;
	}

	if (ot) {
		if (ot->ops->start)
			ot->ops->start(ot);

		ot->power = 1;
	}else{
		E("No output attached.");
	}
	
	return;
}

static void jz_fb_hw_ot_power_off(struct jz_fb_ctrl *ctrl, int force)
{
	struct jz_fb_ot_info *ot = ctrl->ot;

	D("Called.");
	
	if (!force && !ot->power) {
		D("Already Power Down.");
		return;
	}

	if (ot) {
		if (ot->ops->stop)
			ot->ops->stop(ot);

		ot->power = 0;
	}else{
		E("No output attached.");
	}

	return;
}

static void jz_fb_hw_ot_enable(struct jz_fb_ctrl *ctrl, int force)
{
	struct jz_fb_ot_info *ot = ctrl->ot;
	
	unsigned int delay;

	D("Called.");
	
	if (!force && ot->state) {
		D("Already enable.");
		return;
	}

	if (ot) {
		delay = ot->config->enable_delay_ms;

		if (ot->ops->enable) {
			if (delay)
				mdelay(delay);

			ot->ops->enable(ot);
		}

		ot->state = 1;
	}else{
		E("No output attached.");
	}

	return;
}

static void jz_fb_hw_ot_disable(struct jz_fb_ctrl *ctrl, int force)
{
	struct jz_fb_ot_info *ot = ctrl->ot;

	D("Called.");
	
	if (!force && !ot->state) {
		D("Already Disable.");
		return;
	}

	if (ot) {
		if (ot->ops->disable) 
			ot->ops->disable(ot);

		ot->state = 0;
	}else{
		E("No output attached.");
	}

	return;
}

static void jz_fb_hw_ctrl_enable(struct jz_fb_ctrl *ctrl, int force)
{
	D("Called.");
	
	if (!force && ctrl->state) {
		D("Already Enable.");
		return;
	}

	REG_LCD_STATE = 0; /* clear lcdc status */

	__lcd_clr_dis();
	__lcd_set_ena(); /* enable lcdc */
	
	ctrl->state = 1;

	return;
}

static void jz_fb_hw_ctrl_disable(struct jz_fb_ctrl *ctrl, int force)
{
	D("Called.");

	if (!force && !ctrl->state) {
		D("Already Disable.");
		return;
	}

	if (ctrl->ot->use_quick_disable) {
		D("Quick Disable");
		__lcd_clr_ena();
	}else {
		/* when CPU main freq is 336MHz,wait for 16ms */
		int cnt = 336000 * 16;

		__lcd_set_dis(); /* regular disable */
		while(!__lcd_disable_done() && cnt) {
			cnt--;
		}

		D("Normal Disable");
		if (cnt == 0)
			E("LCD disable timeout! REG_LCD_STATE=0x%08xx", REG_LCD_STATE);

		REG_LCD_STATE &= ~LCD_STATE_LDD;
	}

	ctrl->state = 0;

	return;
}

static void jz_fb_hw_set_panel_info(struct jz_fb_ctrl *ctrl)
{
	struct jz_fb_panel_info *panel = ctrl->ot->config->panel;
	struct jz_fb_reg_info *reg = &ctrl->ot->reg_info;

	D("Called.");

	/* Collect Panel Config. */
	reg->lcd_cfg |= panel->lcd_cfg;
	reg->lcd_ctrl |= panel->lcd_ctrl;

	switch (panel->lcd_cfg & LCD_CFG_MODE_MASK ) {
		case LCD_CFG_MODE_GENERIC_TFT:
		case LCD_CFG_MODE_INTER_CCIR656:
		case LCD_CFG_MODE_NONINTER_CCIR656:
		default:
			reg->lcd_vat = (((panel->blw + panel->w + panel->elw + panel->hsw)) << 16) | (panel->vsw + panel->bfw + panel->h + panel->efw);
			reg->lcd_dah = ((panel->hsw + panel->blw) << 16) | (panel->hsw + panel->blw + panel->w);
			reg->lcd_dav = ((panel->vsw + panel->bfw) << 16) | (panel->vsw + panel->bfw + panel->h);
			reg->lcd_hsync = (0 << 16) | panel->hsw;
			reg->lcd_vsync = (0 << 16) | panel->vsw;
			break;
	}

	return;
}

static void jz_fb_hw_set_ot_info(struct jz_fb_ctrl *ctrl)
{
	struct jz_fb_reg_info *reg = &ctrl->ot->reg_info;
	
	D("Called.");
	
	/* Use RGB -> YUV? */
	if (ctrl->ot->use_rgb_to_yuv) {
		D("Use RGB->YUV.");

		reg->lcd_rgbc |= LCD_RGBC_YCC;
	}

	/* Set Panel Background Color. */	
	reg->lcd_bgc = ctrl->ot->config->background_color;

	return;
}

static int jz_fb_hw_set_win_bpp(struct jz_fb_win_info *win, unsigned int bpp)
{
	struct jz_fb_reg_info *reg = &win->ctrl->ot->reg_info;

	D("Called.");

	if (win->index == 0) {
		reg->lcd_ctrl &= ~LCD_CTRL_BPP_MASK;

		if (bpp == 1)
			reg->lcd_ctrl |= LCD_CTRL_BPP_1;
		else if (bpp == 2)
			reg->lcd_ctrl |= LCD_CTRL_BPP_2;
		else if (bpp == 4)
			reg->lcd_ctrl |= LCD_CTRL_BPP_4;
		else if (bpp == 8)
			reg->lcd_ctrl |= LCD_CTRL_BPP_8;
		else if (bpp == 15)
			reg->lcd_ctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB555;
		else if (bpp == 16)
			reg->lcd_ctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB565;
		else if (bpp > 16 && bpp <= 32) 
			reg->lcd_ctrl |= LCD_CTRL_BPP_18_24;
		else
			return -1;

	}else if (win->index == 1) {
		reg->lcd_osdctrl &= ~LCD_CTRL_BPP_MASK;

		if (bpp == 15)
			reg->lcd_osdctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB555;
		else if (bpp == 16)
			reg->lcd_osdctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB565;
		else if (bpp > 16 && bpp <= 32) 
			reg->lcd_osdctrl |= LCD_CTRL_BPP_18_24;
		else
			return -1;
	}

	return 0;
}

static void jz_fb_hw_set_win_enable(struct jz_fb_win_info *win, int enable)
{
	struct jz_fb_reg_info *reg = &win->ctrl->ot->reg_info;
	
	D("Called.");

	if (win->index == 0) {
		if (enable) 
			reg->lcd_osdc |= LCD_OSDC_F0EN;
		else
			reg->lcd_osdc &= ~LCD_OSDC_F0EN; 
	}else if (win->index == 1) {
		if (enable)
			reg->lcd_osdc |= LCD_OSDC_F1EN; 
		else
			reg->lcd_osdc &= ~LCD_OSDC_F1EN;
	}
	
	return;
}

static void jz_fb_hw_set_win_area(struct jz_fb_win_info *win, unsigned int x,
	unsigned int y, unsigned int w, unsigned int h)
{
	struct jz_fb_reg_info *reg = &win->ctrl->ot->reg_info;
	
	D("Called.");

	if (win->index == 0) {
		reg->lcd_xyp0 = HW_AREA_POS(x, y);
		reg->lcd_size0 = HW_AREA_SIZE(w, h);
	}else if (win->index == 1) {
		reg->lcd_xyp1 = HW_AREA_POS(x, y);
		reg->lcd_size1 = HW_AREA_SIZE(w, h);
	}

	return;
}

static void jz_fb_hw_set_dma_desc_format(struct jz_fb_ctrl *ctrl)
{
	struct jz_fb_reg_info *reg = &ctrl->ot->reg_info;	

#ifdef JZ_FB_8WORD_DMA_DESC
	reg->lcd_cfg |= LCD_CFG_NEWDES;
#else
	reg->lcd_cfg &= ~LCD_CFG_NEWDES;
#endif

	return;
}

static void jz_fb_hw_set_win_info(struct jz_fb_ctrl *ctrl)
{
	struct jz_fb_win_info *win;
	struct jz_fb_win_attr_info *wa;
	
	struct jz_fb_reg_info *reg = &ctrl->ot->reg_info;	

	int i;
	
	D("Called.");
	
	/* REVIST: Can we leave OSDEN set when F0EN and F1EN is disabled? */
	reg->lcd_osdc |= LCD_OSDC_OSDEN;
	
	for (i = 0; i < JZ_FB_NR_MAX_FG; i++) {
		win = ctrl->win[i];
		wa = &win->attr;

		jz_fb_hw_set_win_enable(win, wa->enable);	
		jz_fb_hw_set_win_area(win, wa->x, wa->y, wa->w, wa->h);
		jz_fb_hw_set_win_bpp(win, wa->bpp);
	}
	
	return;
}

static inline void dump_registers(void)
{
	DUMP("Dump LCDC Registers: ");
	DUMP("======================================");
	DUMP("REG_LCD_CFG:\t0x%08x", REG_LCD_CFG);
	DUMP("REG_LCD_CTRL:\t0x%08x", REG_LCD_CTRL);
	DUMP("REG_LCD_STATE:\t0x%08x", REG_LCD_STATE);
	DUMP("REG_LCD_OSDC:\t0x%08x", REG_LCD_OSDC);
	DUMP("REG_LCD_OSDCTRL:\t0x%08x", REG_LCD_OSDCTRL);
	DUMP("REG_LCD_OSDS:\t0x%08x", REG_LCD_OSDS);
	DUMP("REG_LCD_BGC:\t0x%08x", REG_LCD_BGC);
	DUMP("REG_LCD_KEK0:\t0x%08x", REG_LCD_KEY0);
	DUMP("REG_LCD_KEY1:\t0x%08x", REG_LCD_KEY1);
	DUMP("REG_LCD_ALPHA:\t0x%08x", REG_LCD_ALPHA);
	DUMP("REG_LCD_IPUR:\t0x%08x", REG_LCD_IPUR);
	DUMP("REG_LCD_VAT:\t0x%08x", REG_LCD_VAT);
	DUMP("REG_LCD_DAH:\t0x%08x", REG_LCD_DAH);
	DUMP("REG_LCD_DAV:\t0x%08x", REG_LCD_DAV);
	DUMP("REG_LCD_XYP0:\t0x%08x", REG_LCD_XYP0);
	DUMP("REG_LCD_XYP1:\t0x%08x", REG_LCD_XYP1);
	DUMP("REG_LCD_SIZE0:\t0x%08x", REG_LCD_SIZE0);
	DUMP("REG_LCD_SIZE1:\t0x%08x", REG_LCD_SIZE1);
	DUMP("REG_LCD_RGBC\t0x%08x", REG_LCD_RGBC);
	DUMP("REG_LCD_VSYNC:\t0x%08x", REG_LCD_VSYNC);
	DUMP("REG_LCD_HSYNC:\t0x%08x", REG_LCD_HSYNC);
	DUMP("REG_LCD_PS:\t0x%08x", REG_LCD_PS);
	DUMP("REG_LCD_CLS:\t0x%08x", REG_LCD_CLS);
	DUMP("REG_LCD_SPL:\t0x%08x", REG_LCD_SPL);
	DUMP("REG_LCD_REV:\t0x%08x", REG_LCD_REV);
	DUMP("REG_LCD_IID:\t0x%08x", REG_LCD_IID);
	DUMP("REG_LCD_DA0:\t0x%08x", REG_LCD_DA0);
	DUMP("REG_LCD_SA0:\t0x%08x", REG_LCD_SA0);
	DUMP("REG_LCD_FID0:\t0x%08x", REG_LCD_FID0);
	DUMP("REG_LCD_CMD0:\t0x%08x", REG_LCD_CMD0);
	DUMP("REG_LCD_OFFS0:\t0x%08x", REG_LCD_OFFS0);
	DUMP("REG_LCD_PW0:\t0x%08x", REG_LCD_PW0);
	DUMP("REG_LCD_CNUM0:\t0x%08x", REG_LCD_CNUM0);
	DUMP("REG_LCD_DESSIZE0:\t0x%08x", REG_LCD_DESSIZE0);
	DUMP("REG_LCD_DA1:\t0x%08x", REG_LCD_DA1);
	DUMP("REG_LCD_SA1:\t0x%08x", REG_LCD_SA1);
	DUMP("REG_LCD_FID1:\t0x%08x", REG_LCD_FID1);
	DUMP("REG_LCD_CMD1:\t0x%08x", REG_LCD_CMD1);
	DUMP("REG_LCD_OFFS1:\t0x%08x", REG_LCD_OFFS1);
	DUMP("REG_LCD_PW1:\t0x%08x", REG_LCD_PW1);
	DUMP("REG_LCD_CNUM1:\t0x%08x", REG_LCD_CNUM1);
	DUMP("REG_LCD_DESSIZE1:\t0x%08x", REG_LCD_DESSIZE1);
	DUMP("==================================");
	DUMP("REG_LCD_VSYNC:\t%d:%d", REG_LCD_VSYNC>>16, REG_LCD_VSYNC&0xfff);
	DUMP("REG_LCD_HSYNC:\t%d:%d", REG_LCD_HSYNC>>16, REG_LCD_HSYNC&0xfff);
	DUMP("REG_LCD_VAT:\t%d:%d", REG_LCD_VAT>>16, REG_LCD_VAT&0xfff);
	DUMP("REG_LCD_DAH:\t%d:%d", REG_LCD_DAH>>16, REG_LCD_DAH&0xfff);
	DUMP("REG_LCD_DAV:\t%d:%d", REG_LCD_DAV>>16, REG_LCD_DAV&0xfff);
	DUMP("==================================");
	/* Smart LCD Controller Resgisters */
	DUMP("REG_SLCD_CFG:\t0x%08x", REG_SLCD_CFG);
	DUMP("REG_SLCD_CTRL:\t0x%08x", REG_SLCD_CTRL);
	DUMP("REG_SLCD_STATE:\t0x%08x", REG_SLCD_STATE);
	DUMP("==================================");
	/* TVE Controller Resgisters */
	DUMP("REG_TVE_CTRL:\t0x%08x", REG_TVE_CTRL);
	DUMP("REG_TVE_FRCFG:\t0x%08x", REG_TVE_FRCFG);
	DUMP("REG_TVE_SLCFG1:\t0x%08x", REG_TVE_SLCFG1);
	DUMP("REG_TVE_SLCFG2:\t0x%08x", REG_TVE_SLCFG2);
	DUMP("REG_TVE_SLCFG3:\t0x%08x", REG_TVE_SLCFG3);
	DUMP("REG_TVE_LTCFG1:\t0x%08x", REG_TVE_LTCFG1);
	DUMP("REG_TVE_LTCFG2:\t0x%08x", REG_TVE_LTCFG2);
	DUMP("REG_TVE_CFREQ:\t0x%08x", REG_TVE_CFREQ);
	DUMP("REG_TVE_CPHASE:\t0x%08x", REG_TVE_CPHASE);
	DUMP("REG_TVE_CBCRCFG:\t0x%08x", REG_TVE_CBCRCFG);
	DUMP("REG_TVE_WSSCR:\t0x%08x", REG_TVE_WSSCR);
	DUMP("REG_TVE_WSSCFG1:\t0x%08x", REG_TVE_WSSCFG1);
	DUMP("REG_TVE_WSSCFG2:\t0x%08x", REG_TVE_WSSCFG2);
	DUMP("REG_TVE_WSSCFG3:\t0x%08x", REG_TVE_WSSCFG3);
	DUMP("==================================");
	
	return;
}

void dump_reg_info(struct jz_fb_reg_info *reg)
{
	DUMP("Dump reg_info: ");
	DUMP("======================================");

	DUMP("reg->lcd_cfg: 0x%08lx.", reg->lcd_cfg);	
	DUMP("reg->lcd_ctrl: 0x%08lx.", reg->lcd_ctrl);	
	DUMP("reg->lcd_vat: 0x%08lx.", reg->lcd_vat);	
	DUMP("reg->lcd_dah: 0x%08lx.", reg->lcd_dah);	
	DUMP("reg->lcd_dav: 0x%08lx.", reg->lcd_dav);	
	DUMP("reg->lcd_hsync: 0x%08lx.", reg->lcd_hsync);	
	DUMP("reg->lcd_vsync: 0x%08lx.", reg->lcd_vsync);	
	DUMP("reg->lcd_bgc: 0x%08lx.", reg->lcd_bgc);
	DUMP("reg->lcd_da0: 0x%08lx.", reg->lcd_da0);
	DUMP("reg->lcd_da1: 0x%08lx.", reg->lcd_da1);

	return;
}

static void jz_fb_hw_write_reg(struct jz_fb_ctrl *ctrl)
{
	struct jz_fb_reg_info *reg = &ctrl->ot->reg_info;	
	
	D("Called.");

	/* TODO:
	   Use flags in reg_info to divide function blocks regs IO.
	*/

	REG_LCD_CFG	= reg->lcd_cfg; /* LCDC Configure Register */
	REG_LCD_CTRL	= reg->lcd_ctrl; /* LCDC Controll Register */
	REG_LCD_OSDC 	= reg->lcd_osdc; /* F0, F1, alpha, */
	REG_LCD_OSDCTRL = reg->lcd_osdctrl; /* IPUEN, bpp */
	REG_LCD_BGC  	= reg->lcd_bgc;
	REG_LCD_KEY0 	= reg->lcd_key0;
	REG_LCD_KEY1 	= reg->lcd_key1;
	REG_LCD_ALPHA 	= reg->lcd_alpha;
	REG_LCD_IPUR 	= reg->lcd_ipur;
	REG_LCD_RGBC  	= reg->lcd_rgbc;
	REG_LCD_VAT	= reg->lcd_vat;
	REG_LCD_DAH	= reg->lcd_dah;
	REG_LCD_DAV	= reg->lcd_dav;
	REG_LCD_VSYNC	= reg->lcd_vsync;
	REG_LCD_HSYNC	= reg->lcd_hsync;
	REG_LCD_PS	= reg->lcd_ps;
	REG_LCD_REV	= reg->lcd_rev;
	REG_LCD_XYP0	= reg->lcd_xyp0;
	REG_LCD_XYP1	= reg->lcd_xyp1;
	REG_LCD_SIZE0	= reg->lcd_size0;
	REG_LCD_SIZE1	= reg->lcd_size1;
	REG_LCD_DA0	= reg->lcd_da0;
	REG_LCD_DA1	= reg->lcd_da1;

	dump_registers();

	return;
}

static void jz_fb_hw_set_pixclock(struct jz_fb_ctrl *ctrl)
{
	struct jz_fb_ot_info *ot = ctrl->ot;
	struct jz_fb_panel_info *panel = ot->config->panel;

	unsigned int pixclock, refresh_rate;
	unsigned int w, h;

	unsigned int v;

	D("Called.");

	refresh_rate = panel->fclk;

	if (ot->use_tve) {
		pixclock = 27000000;
	}else{
		if ((panel->lcd_cfg & LCD_CFG_MODE_MASK) == LCD_CFG_MODE_SERIAL_TFT) {
			/* serial mode: Hsync period = 3*Width_Pixel */
			w = (panel->w * 3 + panel->hsw + panel->elw + panel->blw);
			h = (panel->h + panel->vsw + panel->efw + panel->bfw);

			pixclock = w * h * refresh_rate;
		}else {
			w = (panel->w + panel->hsw + panel->elw + panel->blw);
			h = (panel->h + panel->vsw + panel->efw + panel->bfw);

			pixclock = w * h * refresh_rate;
		}
	}

	ctrl->pixclock = pixclock;

	/* ------------ HW: Set LCD Controller ---------------- */
	__cpm_stop_lcd();

	if (ot->use_tve) {
		REG_CPM_LPCDR |= CPM_LPCDR_LTCS;
		__cpm_select_pixclk_tve();
	}else{
		REG_CPM_LPCDR &= ~CPM_LPCDR_LTCS;
		__cpm_select_pixclk_lcd();
	}
	
	v = __cpm_get_pllout2() / pixclock;
	v--;

	__cpm_set_pixdiv(v);

#if defined(CONFIG_SOC_JZ4750) /* Jz4750D don't use LCLK */
	v = pixclock * 3 ;	/* LCDClock > 2.5*Pixclock */

	v = (__cpm_get_pllout()) / v;
	if (v > 0x1f) {
		printk("lcd clock divide is too large, set it to 0x1f\n");
		v = 0x1f;
	}

	__cpm_set_ldiv(v);
#endif

	REG_CPM_CPCCR |= CPM_CPCCR_CE; /* update divide */

	jz_clocks.pixclk = __cpm_get_pixclk();

#if defined(CONFIG_SOC_JZ4750) /* Jz4750D don't use LCLK */
	jz_clocks.lcdclk = __cpm_get_lcdclk();
	I("LCD Controller Clock: %dMHz.", jz_clocks.lcdclk / 1000 / 1000);
#endif

	I("Pixel Clock: %dMHz.", pixclock / 1000 / 1000);

	__cpm_start_lcd();

	mdelay(1);

	return;
}

static void jz_fb_hw_setup(struct jz_fb_ctrl *ctrl)
{
	struct jz_fb_ot_info *ot = ctrl->ot;
	struct jz_fb_win_info *win;

	int i, j;
	
	D("Called.");
	
	memset(&ot->reg_info, 0, (sizeof(struct jz_fb_reg_info)));

	/* Step 1: Set Pixel Clock. */
	jz_fb_hw_set_pixclock(ctrl);

	/* Step 2: Build DMA Descriptor Chain. */
	jz_fb_hw_set_dma_desc_format(ctrl);

	for (i = 0; i < JZ_FB_NR_MAX_FG; i++) {
		win = ctrl->win[i];

		/* Prepare DMA Descriptors. */
		ot->desc_ops->cleanup(win);
		ot->desc_ops->setup(win);
		
		/* Attach All Framebuffer Memory Blocks. */
		for (j = 0; j < JZ_FB_NR_MAX_FB_MEM; j++) {
			if (win->fb_mem[j]) {
				D("Win %d: Attach Framebufer : 0x%p.", i, win->fb_mem[j]);

				ot->desc_ops->attach(win, win->fb_mem[j]);
			}
		}
	}

	/* Step 3: Set Panel Info. */
	jz_fb_hw_set_panel_info(ctrl);
	
	/* Step 4: Set OT Info. */
	jz_fb_hw_set_ot_info(ctrl);

	/* Step 5: Set All Win Info. */
	jz_fb_hw_set_win_info(ctrl);
	
	/* Step 6: Write all result to HW. */
	jz_fb_hw_write_reg(ctrl);	
	
	/* Done. */
	return;
}

