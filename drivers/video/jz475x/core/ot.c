/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * Output Control Routines.
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

struct jz_fb_ot_scan_info {
	unsigned int max_w;
	unsigned int max_h;
	unsigned int max_bpp;
};

static inline unsigned int bpp_to_ctrl_bpp(int bpp)
{
	switch (bpp) {
		case 15:
		case 16:
			break;

		case 17 ... 32:
			bpp = 32;

			break;

		default:
			E("BPP (%d) not support, Set BPP 32.\n", bpp);

			bpp = 32;

			break;
	}

	return bpp;
}

static void jz_fb_ot_scan(struct jz_fb_ctrl *ctrl, 
				struct jz_fb_ot_scan_info *info)
{	
	struct jz_fb_ot_info *o;
	jz_fb_ot_init_t *init;

	struct jz_fb_win_attr_info *a;

	unsigned int n = sizeof(jz_fb_ots) / sizeof(struct jz_fb_ot_info *);

	unsigned int i, j;
	unsigned int v;
	
	info->max_w = info->max_h = info->max_bpp = 0;

	for (i = 0; i < n; i++) {
		o = jz_fb_ots[i];
		init = jz_fb_ot_inits[i];

		/* Step 1: OT ID & Parent. */
		o->id = i;		
		o->ctrl = ctrl;
		
		/* Step 2: OT Initializer. */
		init(o);
		
		/* Step 3: Glue & Config. */
		for (j = 0; j < JZ_FB_NR_MAX_FG; j++) {
			a = &o->config->win_init_config[j];

			/* Win BPP Glue.*/
			a->bpp = bpp_to_ctrl_bpp(a->bpp);
			
			/* Load INIT config as first runtime config. */
			o->config->win_runtime_config[j] = *a;
		}
		
		/* Step 4: Collect Max Panel Info. */
		v = o->config->panel->w;
		if (info->max_w < v)
			info->max_w = v;

		v = o->config->panel->h;
		if (info->max_h < v)
			info->max_h = v;

		/* Max Win BPP Glue. */
		v = o->config->max_win_bpp;

		o->config->max_win_bpp = bpp_to_ctrl_bpp(v);

		if (info->max_bpp < v)
			info->max_bpp = v;

		D("Output: %s ID: %d Current Max w: %u, h: %u, bpp: %u.", 
				o->name, o->id, info->max_w, 
				info->max_h, info->max_bpp);

		I("Found Output: %s, ID: %d.", o->name, o->id);
	}
	
	return;
}

static int jz_fb_ot_load_runtime_config(struct jz_fb_ot_info *ot)
{
	struct jz_fb_win_info *win;
	struct jz_fb_win_attr_info *a = ot->config->win_runtime_config;

	int i;
	
	D("Called.");

	for (i = 0; i < JZ_FB_NR_MAX_FG; i++) {
		win = ot->ctrl->win[i];
		D("Attr: enable: %d, w: %d, h: %d, x: %d, y: %d, bpp: %d.",
				a->enable, a->w, a->h, a->x, a->y, a->bpp);

		jz_fb_win_set_attr(win, a + i);
	}
	
	return 0;
}

static int jz_fb_ot_set_current(struct jz_fb_ot_info *ot)
{
	struct jz_fb_ctrl *ctrl = &jz_fb_ctrl;
	struct jz_fb_win_info *win;

	int i;

	jz_fb_hw_ot_disable(ctrl, 1);
	jz_fb_hw_ctrl_disable(ctrl, 1);
	jz_fb_hw_ot_power_off(ctrl, 1);

	ctrl->ot = ot;

	/* Load OT Runtime Config. */			
	jz_fb_ot_load_runtime_config(ot);
	
	jz_fb_hw_setup(ctrl);

	for (i = 0; i < JZ_FB_NR_MAX_FG; i++) {
		win = ctrl->win[i];

		/* Update FB_INFO. */
		win_fb_info_setup(win);
	}
	
	jz_fb_hw_ot_power_on(ctrl, 1);
	jz_fb_hw_ctrl_enable(ctrl, 1);
	jz_fb_hw_ot_enable(ctrl, 1);

	return 0;
}
