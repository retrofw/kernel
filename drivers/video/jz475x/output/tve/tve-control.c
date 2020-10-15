/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * TVE Common Control Routines.
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

static int tve_output_control(struct jz_fb_ot_info *ot, 
				void __user *p)
{
	struct jz_fb_tve_control lc;
	struct jz_fb_common_control *c = &lc.common;

	int rv = 0;

	if (copy_from_user(&lc, p, sizeof(struct jz_fb_tve_control))) {
		E("Failed to do copy_from_user.\n");
		return -EFAULT;
	}
	
	/* Prevent Userspace APP walk the error path. */
	if (c->id != ot->id) {
		E("Bad Output ID for TVE Path: %s.", ot->miscdev->name);
		return -EINVAL;
	}

	switch (c->command) {
		case JZ_FB_CMD_TVE_GET_PATH:
			c->v = tve_output_get_path(ot);

			if (copy_to_user(p, &lc, sizeof(struct jz_fb_tve_control))) {
				E("Failed to do copy_to_user().");
				rv = -EFAULT;
			}

			break;

		case JZ_FB_CMD_TVE_SET_PATH:
			rv = tve_output_set_path(ot, c->v);
			break;

		default:
			rv = do_ot_common_control(ot, c, p);
			break;
	}

	return rv;
}
