/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * Output Common Control Routines.
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

/* Handle Common Controls. */
static int do_ot_common_control(struct jz_fb_ot_info *ot, 
		struct jz_fb_common_control *c, void __user *p)
{
	int rv = 0;

	int i;

	switch (c->command) {
		case JZ_FB_CMD_COMMON_GET_STATE:
			c->v = 0;

			if (ot->power)
				c->v |= JZ_FB_CMD_COMMON_STATE_OT_POWER;
			
			if (ot->state)
				c->v |= JZ_FB_CMD_COMMON_STATE_OT_ENABLE;
			
			if (ot->ctrl->state)
				c->v |= JZ_FB_CMD_COMMON_STATE_CTRL_ENABLE;

			if (copy_to_user(p, c, sizeof(struct jz_fb_common_control))) 
				rv = -EFAULT;
			
			break;

		case JZ_FB_CMD_COMMON_SET_POWER:
			if (c->v) {
				jz_fb_hw_ot_power_on(ot->ctrl, 0);
			}else{
				jz_fb_hw_ot_power_off(ot->ctrl, 0);
			}

			break;

		case JZ_FB_CMD_COMMON_SET_SCREEN:	
			if (c->v) {
				jz_fb_hw_ot_enable(ot->ctrl, 0);
			}else{
				jz_fb_hw_ot_disable(ot->ctrl, 0);
			}

			break;

		case JZ_FB_CMD_COMMON_GET_WIN_ATTR:
			memcpy(c->win_attr, ot->config->win_runtime_config, sizeof(c->win_attr));	
			if (copy_to_user(p, c, sizeof(struct jz_fb_common_control))) 
				rv = -EFAULT;

			break;

		case JZ_FB_CMD_COMMON_SET_WIN_ATTR:
			memcpy(ot->config->win_runtime_config, c->win_attr, sizeof(c->win_attr));	
			/* OT WIN Runtime CONFIG -> Current WIN Attr. 
			   This allow each OT having its own Runtime ATTR.
			*/

			jz_fb_ot_load_runtime_config(ot);

			if (c->active_now) {
				for (i = 0; i < JZ_FB_NR_MAX_FG; i++)
					jz_fb_win_active_attr(ot->ctrl->win[i]);
			}
				
			/* Attr will be actived when OT is re-selected. */
			break;
	
		case JZ_FB_CMD_COMMON_SELECT_OT:
			rv = jz_fb_ot_set_current(ot);

			break;

	}

	return rv;
}

