/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * ABI between Kernel Driver and Userspace Driver
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

#ifndef __JZ_FB_ABI_H__
#define __JZ_FB_ABI_H__

#include "config.h"

/* Window Attribute. */
struct jz_fb_win_attr_info {
	int enable;

	unsigned int x;
	unsigned int y;
	unsigned int w;
	unsigned int h;

	unsigned int bpp;
};

/* Output Panel State. */
enum {
	JZ_FB_PANEL_STATE_POWER_OFF = 0,
	JZ_FB_PANEL_STATE_POWER_ON,
	JZ_FB_PANEL_STATE_SCREEN_OFF,
	JZ_FB_PANEL_STATE_SCREEN_ON,
};

/* ----------- Common --------------- */
enum {
	JZ_FB_CMD_COMMON_GET_STATE = 0,
	JZ_FB_CMD_COMMON_SET_POWER,
	JZ_FB_CMD_COMMON_SET_SCREEN,
	
	JZ_FB_CMD_COMMON_GET_WIN_ATTR,
	JZ_FB_CMD_COMMON_SET_WIN_ATTR,
	
	JZ_FB_CMD_COMMON_SELECT_OT,
	JZ_FB_CMD_COMMON_END,
};

struct jz_fb_common_control {
	int id;
	int command;
	int v;

	/* TODO: We may support runtime changes in future. */
	int active_now;	

	struct jz_fb_win_attr_info win_attr[JZ_FB_NR_MAX_FG];
};

/* Command: GET_STATE. Return value: */
enum {
	JZ_FB_CMD_COMMON_STATE_OT_POWER = (1 << 0),
	JZ_FB_CMD_COMMON_STATE_OT_ENABLE = (1 << 1),
	JZ_FB_CMD_COMMON_STATE_CTRL_ENABLE = (1 << 2),
};

/* --------- OUTPUT PATH: LCD --------------- */

/* COMMAND. */
enum {
	JZ_FB_CMD_LCD_GET_BACKLIGHT = JZ_FB_CMD_COMMON_END,
	JZ_FB_CMD_LCD_SET_BACKLIGHT,
};

struct jz_fb_lcd_control {
	struct jz_fb_common_control common;
};

/* -------- OUTPUT PATH: TV NTSC/PAL -------- */
/* COMMAND. */
enum {
	JZ_FB_CMD_TVE_GET_PATH = JZ_FB_CMD_COMMON_END,
	JZ_FB_CMD_TVE_SET_PATH,
};

/* TVE Output Path. */
enum {
	JZ_FB_TVE_PATH_CVBS = 0,
	JZ_FB_TVE_PATH_SVIDEO,

#ifdef CONFIG_SOC_JZ4750D
	JZ_FB_TVE_PATH_YUV,
#endif

};

struct jz_fb_tve_control {
	struct jz_fb_common_control common;
};

#endif /* Define __JZ_FB_ABI_H__ */
