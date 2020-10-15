/*
 * Ingenic JZ475X Display Controllers Driver.
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

#ifndef __JZ_FB_H__
#define __JZ_FB_H__

#include <linux/miscdevice.h>

#include "config.h"
#include "abi.h"

#define NR_PALETTE	256
#define PALETTE_SIZE	(NR_PALETTE * 2)

#define HW_AREA_POS(x, y)	( y << 16 | x)
#define HW_AREA_SIZE(w, h)	( h << 16 | w)

/* For 8 word DMA Descriptor. */
#ifdef JZ_FB_8WORD_DMA_DESC
struct jz_lcdc_8word_dma_desc {
	uint32_t next_desc; 	/* LCDDAx */
	uint32_t databuf;   	/* LCDSAx */
	uint32_t frame_id;  	/* LCDFIDx */ 
	uint32_t cmd; 		/* LCDCMDx */
	uint32_t offsize;      	/* Stride Offsize(in word) */
	uint32_t page_width; 	/* Stride Pagewidth(in word) */
	uint32_t cmd_num; 	/* Command Number(for SLCD) */
	uint32_t desc_size; 	/* Foreground Size */
};

typedef struct jz_lcdc_8word_dma_desc dma_desc_t;
#endif

/* For 4 word DMA Descriptor. */
#ifdef JZ_FB_4WORD_DMA_DESC
struct jz_lcdc_4word_dma_desc {
	unsigned int next_desc; 	/* LCDDAx */
	unsigned int databuf;   	/* LCDSAx */
	unsigned int frame_id;  	/* LCDFIDx */ 
	unsigned int cmd; 		/* LCDCMDx */
};

typedef struct jz_lcdc_4word_dma_desc dma_desc_t;
#endif

struct jz_fb_ot_info;
struct jz_fb_win_info;

struct jz_fb_dma_desc {
	dma_desc_t hw_desc;

	int index;
	int use;
	void *mem;	

	struct jz_fb_win_info *win;
	struct list_head list;
	struct list_head group;
};

struct jz_fb_panel_ops {
	int (*normal)(struct jz_fb_ot_info *ot);
	int (*idle)(struct jz_fb_ot_info *ot);
	int (*on)(struct jz_fb_ot_info *ot);
	int (*off)(struct jz_fb_ot_info *ot);
	int (*get_backlight)(struct jz_fb_ot_info *ot);
	int (*set_backlight)(struct jz_fb_ot_info *ot, int value);
};

struct jz_fb_panel_info {
	unsigned long lcd_ctrl;	/* panel mode and pin usage etc. */
	unsigned long lcd_cfg;	/* panel mode and pin usage etc. */
	unsigned int w;		/* Panel Width(in pixel) */
	unsigned int h;		/* Panel Height(in line) */
	unsigned int fclk;	/* frame clk */
	unsigned int hsw;	/* hsync width, in pclk */
	unsigned int vsw;	/* vsync width, in line count */
	unsigned int elw;	/* end of line, in pclk */
	unsigned int blw;	/* begin of line, in pclk */
	unsigned int efw;	/* end of frame, in line count */
	unsigned int bfw;	/* begin of frame, in line count */
};

struct jz_fb_panel_config {
	struct jz_fb_panel_ops *ops;
	struct jz_fb_panel_info *panel;

	struct jz_fb_win_attr_info win_runtime_config[JZ_FB_NR_MAX_FG];
	
	/* For Panel Private Routines. */
	void *priv;

	struct jz_fb_win_attr_info *win_init_config;
	
	unsigned int enable_delay_ms;

	int backlight;
	int use_palette;
	unsigned int background_color;
	unsigned int max_win_bpp;
};

struct jz_fb_reg_info {
	/* Used to config specified function block's register. */
	unsigned long flags;

	unsigned long lcd_cfg;
	unsigned long lcd_ctrl;
	unsigned long lcd_osdc;
	unsigned long lcd_osdctrl;
	unsigned long lcd_bgc;
	unsigned long lcd_key0;
	unsigned long lcd_key1;
	unsigned long lcd_alpha;
	unsigned long lcd_ipur;
	unsigned long lcd_rgbc;
	unsigned long lcd_vat;
	unsigned long lcd_dah;
	unsigned long lcd_dav;
	unsigned long lcd_xyp0;
	unsigned long lcd_xyp1;
	unsigned long lcd_size0;
	unsigned long lcd_size1;
	unsigned long lcd_vsync;
	unsigned long lcd_hsync;
	unsigned long lcd_ps;
	unsigned long lcd_rev;
	unsigned long lcd_da0;
	unsigned long lcd_da1;
};

struct jz_fb_ot_ops {
	void (*enable)(struct jz_fb_ot_info *);
	void (*disable)(struct jz_fb_ot_info *);
	void (*start)(struct jz_fb_ot_info *);
	void (*stop)(struct jz_fb_ot_info *);
	int (*control)(struct jz_fb_ot_info *, void *);
};

struct jz_fb_ot_desc_ops {
	int (*setup)(struct jz_fb_win_info *);
	void (*cleanup)(struct jz_fb_win_info *);
	int (*attach)(struct jz_fb_win_info *, void *);
	int (*detach)(struct jz_fb_win_info *, void *);
};

struct jz_fb_ot_info {
	char *name;
	int id;
	
	int state;
	int power;

	int use_quick_disable;
	int use_tve;
	int use_rgb_to_yuv;

	struct jz_fb_ot_ops *ops;
	struct jz_fb_ot_desc_ops *desc_ops;
	struct jz_fb_panel_config *config;

	struct jz_fb_reg_info reg_info;
	
	struct mutex ctrl_drv_lock;
	struct miscdevice *miscdev;

	struct jz_fb_ctrl *ctrl;
};

struct jz_fb_ctrl;

#define WIN_X(win) (win->attr.x)
#define WIN_Y(win) (win->attr.y)

#define WIN_W(win) (win->attr.w)
#define WIN_H(win) (win->attr.h)

#define WIN_BPP(win) (win->attr.bpp)

struct jz_fb_win_info {
	int index;

	struct jz_fb_win_attr_info attr;

	struct fb_info		fb;

	/* DMA Descriptor Pool. */	
	struct jz_fb_dma_desc *dma_desc[JZ_FB_NR_MAX_DMA_DESC];
	struct jz_fb_dma_desc *dma_desc_data_start;
	struct jz_fb_dma_desc *dma_desc_last_no_data;

	void *dma_desc_pool;

	/* Framebuffer Pool. */
	void *fb_mem[JZ_FB_NR_MAX_FB_MEM];
	unsigned int fb_mem_len;	/* Framebuffer Block Size. */

	/* Runtime WIN data size. */
	unsigned int fb_line_len;
	unsigned int fb_frame_len;

	void *dma_palette;
	
	void *misc_mem_start;
	void *misc_mem_cur;

	struct jz_fb_ctrl 	*ctrl;
};

struct jz_fb_ctrl {
	int state;

	unsigned int pixclock;
	
	struct jz_fb_ot_info *ot;
	struct jz_fb_win_info *win[JZ_FB_NR_MAX_FG];
};

#endif /* __JZ_FB_H__ */
