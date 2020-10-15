/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * DMA Descirptor Routines.
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

static void dump_desc(struct jz_fb_dma_desc *desc)
{
	DUMP("HW_DESC: Virt: 0x%08lx, Phys: 0x%08lx", &desc->hw_desc, 
			virt_to_phys(&desc->hw_desc));
	DUMP("=================================================");
	DUMP("next_desc: 0x%08lx.", desc->hw_desc.next_desc);
	DUMP("databuf: 0x%08lx.", desc->hw_desc.databuf);
	DUMP("frame_id: 0x%08lx.", desc->hw_desc.frame_id);
	DUMP("cmd: 0x%08lx.", desc->hw_desc.cmd);

#ifdef JZ_FB_8WORD_DMA_DESC
	DUMP("offsize: 0x%08lx.", desc->hw_desc.offsize);
	DUMP("page_width: 0x%08lx.", desc->hw_desc.page_width);
	DUMP("cmd_num: 0x%08lx.", desc->hw_desc.cmd_num);
	DUMP("desc_size: 0x%08lx.", desc->hw_desc.desc_size);
#endif

	return;
}

static inline void dma_desc_wback_inv(struct jz_fb_dma_desc *desc)
{
	dma_cache_wback((unsigned long)&desc->hw_desc, sizeof(dma_desc_t));

	return;
}

static unsigned long dma_desc_pool_init(struct jz_fb_win_info *win)
{
	void *mem = win->dma_desc_pool;

	int i;
	
	win->dma_desc_data_start = NULL;
	win->dma_desc_last_no_data = NULL;

	for (i = 0; i < JZ_FB_NR_MAX_DMA_DESC; i++) {
		win->dma_desc[i] = mem;

		win->dma_desc[i]->use = 0;
		win->dma_desc[i]->mem = NULL;

		win->dma_desc[i]->index = i;
		win->dma_desc[i]->win = win;

		mem += sizeof(struct jz_fb_dma_desc);
	}
		
	return sizeof(struct jz_fb_dma_desc) * JZ_FB_NR_MAX_DMA_DESC;
}

static struct jz_fb_dma_desc *dma_desc_get(struct jz_fb_win_info *win)
{
	struct jz_fb_dma_desc *desc;
	
	int i;

	for (i = 0; i < JZ_FB_NR_MAX_DMA_DESC; i++) {
		if (!win->dma_desc[i]->use) {
			desc = win->dma_desc[i];		
			desc->use = 1;

			memset(&desc->hw_desc, 0, sizeof(dma_desc_t));

			return desc;
		}
	}
	
	return NULL;
}

static void dma_desc_put(struct jz_fb_dma_desc *desc)
{
	desc->use = 0;
	
	return;
}

static void dma_desc_init(struct jz_fb_dma_desc *desc)
{
	/* TODO: Setup DMA Descriptor as static/dynamic config. 
		like EOF etc.
	*/

	desc->hw_desc.next_desc = virt_to_phys(&desc->hw_desc);
	desc->hw_desc.frame_id = desc->index;

	INIT_LIST_HEAD(&desc->list);
	INIT_LIST_HEAD(&desc->group);

	return;
}

static void dma_desc_set_mem(struct jz_fb_dma_desc *desc, void *mem)
{
	desc->mem = mem;
	desc->hw_desc.databuf = virt_to_phys(mem);

	return;
}

static void dma_desc_set_data_size(struct jz_fb_dma_desc *desc, 
					unsigned long size)
{
	desc->hw_desc.cmd |= dma_align_size(size) / 4;	/* Word. */
	
	return;
}

static void dma_desc_set_area_size(struct jz_fb_dma_desc *desc, unsigned int w,
			unsigned int h)
{
	desc->hw_desc.desc_size = HW_AREA_SIZE(w, h);

	return;
}

/* REVIST: The exact meaning of offsize and page_width. */
void dma_desc_set_stride(struct jz_fb_dma_desc *desc, 
		unsigned long offsize, unsigned long page_width)
{
	desc->hw_desc.offsize = dma_align_size(offsize) / 4;
	desc->hw_desc.page_width = dma_align_size(page_width) / 4;

	return;
}

static void dma_desc_set_as_palette(struct jz_fb_dma_desc *desc)
{
	desc->hw_desc.cmd |= LCD_CMD_PAL;
	
	return;
}

static void dma_desc_set_as_cmd(struct jz_fb_dma_desc *desc)
{
	desc->hw_desc.cmd |= LCD_CMD_CMD;
	
	return;
}

static void dma_desc_set_chain_head(struct jz_fb_dma_desc *desc)
{
	struct jz_fb_ot_info *ot = desc->win->ctrl->ot;
	struct jz_fb_reg_info *reg = &ot->reg_info;

	int win_index = desc->win->index;
	
	dma_desc_wback_inv(desc);

	if (win_index == 0) {
		REG_LCD_DA0 = reg->lcd_da0 = virt_to_phys(&desc->hw_desc);
	}else{
		REG_LCD_DA1 = reg->lcd_da1 = virt_to_phys(&desc->hw_desc);
	}
	
	return;
}

static void dma_desc_set_data_head(struct jz_fb_dma_desc *desc)
{
	struct jz_fb_win_info *win = desc->win;

	if (!win->dma_desc_last_no_data) {
		dma_desc_set_chain_head(desc);
	}else{
		win->dma_desc_last_no_data->hw_desc.next_desc = virt_to_phys(&desc->hw_desc);

		dma_desc_wback_inv(win->dma_desc_last_no_data);
		dma_desc_wback_inv(desc);
	}


	return;
}

static struct jz_fb_dma_desc *dma_desc_mem_to_desc(struct jz_fb_win_info *win, void *mem)
{
	int i;

	for (i = 0; i < JZ_FB_NR_MAX_DMA_DESC; i++) {
		if (win->dma_desc[i]->use 
			&& win->dma_desc[i]->mem == mem) {

			return win->dma_desc[i];
		}
	}

	return NULL;
}

/* Add a node to a group. */
static void dma_desc_group_add(struct jz_fb_dma_desc *desc, 
		struct jz_fb_dma_desc *head)
{
	struct jz_fb_dma_desc *tail;
	
	tail = list_entry(head->group.prev, struct jz_fb_dma_desc, group);

	tail->hw_desc.next_desc = virt_to_phys(&desc->hw_desc);
	desc->hw_desc.next_desc = virt_to_phys(&head->hw_desc);
	
	dma_desc_wback_inv(desc);
	dma_desc_wback_inv(tail);

	list_add_tail(&desc->group, &head->group);

	return;
}

/* Add a group or node to DMA Chain. */
static void dma_desc_chain_add_tail(struct jz_fb_dma_desc *desc, 
		struct jz_fb_dma_desc *chain_head)
{
	struct jz_fb_dma_desc *chain_tail, *desc_tail;
	
	chain_tail = list_entry(chain_head->list.prev, struct jz_fb_dma_desc, list);

	if (!list_empty(&desc->group)) {
		desc_tail = list_entry(desc->group.prev, struct jz_fb_dma_desc, group);
	}else{
		desc_tail = desc;
	}
	
	dma_desc_wback_inv(desc);

	desc_tail->hw_desc.next_desc = virt_to_phys(&chain_head->hw_desc);
	chain_tail->hw_desc.next_desc = virt_to_phys(&desc->hw_desc);
	
	dma_desc_wback_inv(desc_tail);
	dma_desc_wback_inv(chain_tail);

	list_add_tail(&desc->list, &chain_head->list);

	return;
}

/* Delete a node / group from DMA Chain. */
static void dma_desc_chain_del(struct jz_fb_dma_desc *desc)
{	
	struct jz_fb_dma_desc *prev, *prev_tail, *next;
	struct jz_fb_win_info *win = desc->win;

	if (list_empty(&desc->list)) /* Always deleted ? */
		return;

	prev = list_entry(desc->list.prev, struct jz_fb_dma_desc, list);
	
	/* Prev is in group ? */
	if (!list_empty(&prev->group)) {
		/* Get Prev group tail. */
		prev_tail = list_entry(prev->group.prev, struct jz_fb_dma_desc, group);
	}else{
		prev_tail = prev;
	}

	next = list_entry(desc->list.next, struct jz_fb_dma_desc, list);

	/* Should we choose new data head ? */
	if (desc == win->dma_desc_data_start) {
		/* Setup New data head. */
		if(!win->dma_desc_last_no_data) {
			dma_desc_set_chain_head(next);
		}else{
			dma_desc_set_data_head(next);			
		}

		win->dma_desc_data_start = next;
	}

	/* Remove us from the chain Prev group tail -> next group Head. */
	prev_tail->hw_desc.next_desc = virt_to_phys(&next->hw_desc);

	dma_desc_wback_inv(prev_tail);

	list_del(&desc->list);

	return;
}

static void dma_desc_add_to_chain(struct jz_fb_dma_desc *desc)
{
	struct jz_fb_win_info *win = desc->win;

	if (!win->dma_desc_data_start) {
		/* Empty. */
		if (!win->dma_desc_last_no_data) {
			/* Chain head. */
			dma_desc_set_chain_head(desc);
		}else{
			/* Only Data head. */
			dma_desc_set_data_head(desc);
		}

		win->dma_desc_data_start = desc;

		return;
	}else{
		/* Link node or group to the end of the DMA chain. */
		dma_desc_chain_add_tail(desc, win->dma_desc_data_start);
	}

	dma_desc_wback_inv(desc);

	return;
}

static void dma_desc_del_from_chain(struct jz_fb_dma_desc *desc)
{
	struct jz_fb_win_info *win = desc->win;

	if (!win->dma_desc_data_start) /* Data Chain is empty. */
		return;
	
	/* Must be 1 node / group in Data chain. */
	if (list_empty(&win->dma_desc_data_start->list))
		return;

	dma_desc_chain_del(desc);

	return;
}

