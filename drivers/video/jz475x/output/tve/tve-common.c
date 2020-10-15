/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * TVE Common Routines.
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

#ifndef __JZ_FB_TVE_COMMON__
#define __JZ_FB_TVE_COMMON__

static int tve_output_desc_setup(struct jz_fb_win_info *win)
{
	D("Called.");	
	
	return 0;
}

static void tve_output_desc_cleanup(struct jz_fb_win_info *win)
{
	/* Clean up ALL. */
	dma_desc_pool_init(win);

	return;
}

/* Dynamically Attach / Detach Framebuffer */
/* TVE Output: 2 Descriptors -> 1 Framebuffer. */
static int tve_output_desc_attach(struct jz_fb_win_info *win, void *mem)
{
	/* Get new descriptor. */
	struct jz_fb_dma_desc *top, *bottom;
	struct jz_fb_win_attr_info *a = &win->attr;

	D("Called.");
	
	top = dma_desc_get(win);
	if (!top) {
		return -ENOMEM;
	}

	bottom = dma_desc_get(win);
	if (!bottom) {
		return -ENOMEM;
	}

	dma_desc_init(top);
	dma_desc_init(bottom);

	/* Attach framebuffer to desciptors. */
	dma_desc_set_mem(top, mem);
	dma_desc_set_mem(bottom, mem);

	if (win->fb_frame_len & 0x1) {
		dma_desc_set_data_size(top, win->fb_frame_len / 2 + win->fb_line_len);
		dma_desc_set_data_size(bottom, win->fb_frame_len / 2 - win->fb_line_len);
	}else{
		dma_desc_set_data_size(top, win->fb_frame_len / 2);
		dma_desc_set_data_size(bottom, win->fb_frame_len / 2);
	}

	dma_desc_set_area_size(top, a->w, a->h);
	dma_desc_set_area_size(bottom, a->w, a->h);
	
	dma_desc_set_stride(top, win->fb_line_len, win->fb_line_len);
	dma_desc_set_stride(bottom, win->fb_line_len, win->fb_line_len);

	/* Add bottom to top's group -> Link bottom after top. */
	dma_desc_group_add(bottom, top);
	
	dma_desc_add_to_chain(top);

	dump_desc(top);
	dump_desc(bottom);

	return 0;
}

static int tve_output_desc_detach(struct jz_fb_win_info *win, void *mem)
{
	struct jz_fb_dma_desc *desc, *node;
	
	struct list_head *pos, *n;

	/* Locate descriptor by mem pointer. */
	desc = dma_desc_mem_to_desc(win, mem);
	if (!desc) {
		return -ENODEV;
	}
	
	/* Remove us from current chain. */
	dma_desc_del_from_chain(desc);
	
	/* Release Group. */
	list_for_each_safe(pos, n, &desc->group) {
		node = list_entry(pos, struct jz_fb_dma_desc, group);

		list_del(pos);
		dma_desc_put(node);
	}

	/* Release descriptor. */
	dma_desc_put(desc);
	
	return 0;
}

static struct jz_fb_ot_desc_ops tve_output_desc_ops = {
	.setup = tve_output_desc_setup,
	.attach = tve_output_desc_attach,
	.detach = tve_output_desc_detach,
	.cleanup = tve_output_desc_cleanup,
};

#include "tve-hw.c"

static void tve_output_start(struct jz_fb_ot_info *ot)
{
	jz_fb_tve_set_output((int)ot->config->priv);
	jz_fb_tve_start();
	
	return;
}

static void tve_output_stop(struct jz_fb_ot_info *ot)
{
	jz_fb_tve_stop();

	return;
}

static inline int tve_output_get_path(struct jz_fb_ot_info *ot)
{
	return (int)(ot->config->priv);
}

static inline int tve_output_set_path(struct jz_fb_ot_info *ot, int path)
{
	int rv;
	
	rv = jz_fb_tve_valid_output(path);
	if (rv) {
		E("Invaild Output Path: %d.", path);
		return rv;
	}

	ot->config->priv = (void *)path;
	
	return 0;
}

#include "tve-control.c"

#endif
