/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * Foreground Control Routines.
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

static void update_fb_data_size(struct jz_fb_win_info *win)
{
	struct jz_fb_win_attr_info *wa = &win->attr;
	
	D("Called.");

	D("w: 0x%08x, h: 0x%08x, bpp: 0x%08x.", wa->w, wa->h, wa->bpp);

	win->fb_line_len = dma_align_size(wa->w * bpp_to_data_bpp(wa->bpp) / 8);
	win->fb_frame_len = win->fb_line_len * wa->h;
	
	D("win->fb_line_len: 0x%08x, win->fb_frame_len: 0x%08x",
		win->fb_line_len, win->fb_frame_len);	
	return;
}

static struct jz_fb_win_info *alloc_win(unsigned int w, unsigned int h, unsigned int bpp)
{
 	struct jz_fb_win_info *win;

	void *fb_mem;	
	
	unsigned long status = 0;

	int rv;	
	
	D("Called.");

	/* Step 1: FB Info. */
	win = kzalloc(sizeof(struct jz_fb_win_info) + sizeof(u32) * 16, GFP_KERNEL);
	if (!win) {
		E("Failed to allocate framebuffer info.");
		return NULL;
	}
	
	set_bit(0, &status);

	/* Step 2: DMA Palette and Descriptors. */
	win->misc_mem_start = (void *)__get_free_page(GFP_KERNEL);
	if (!win->misc_mem_start) {
		E("Faield to allocate memory for palette and descriptor.");
		goto err;
	}
	
	D("Misc MEM start: Virt: 0x%p, Phys: 0x%08lx.", win->misc_mem_start,
		virt_to_phys(win->misc_mem_start));	

	set_bit(1, &status);

	win->misc_mem_cur = win->misc_mem_start;	

	/* Step 3: DMA Palette. */
	win->dma_palette = win->misc_mem_cur;

	D("DMA Palette MEM Start: Virt: 0x%p, Phys: 0x%08lx.", win->misc_mem_cur, 
		virt_to_phys(win->misc_mem_cur));

	win->misc_mem_cur += dma_align_size(PALETTE_SIZE);

	D("DMA Palette MEM Size: 0x%lx", dma_align_size(PALETTE_SIZE));	

	/* Step 4: DMA Descriptor Pool. */
	D("DMA Descriptor Pool Start: Virt: 0x%p, Phys: 0x%08lx.", win->misc_mem_cur,
		virt_to_phys(win->misc_mem_cur));	

	win->dma_desc_pool = win->misc_mem_cur;
	win->misc_mem_cur += dma_desc_pool_init(win);

	/* Step 5: Framebuffer. */
	bpp = bpp_to_data_bpp(bpp);

	win->fb_mem_len = ((w * bpp + 7) >> 3) * h;
	
	fb_mem = alloc_framebuffer(win->fb_mem_len);
	if (!fb_mem) {
		E("Failed to allocate framebuffer.");
		goto err;
	}
	
	D("Framebuffer MEM Block Size: 0x%08x.", win->fb_mem_len);

	set_bit(2, &status);

	rv = add_framebuffer_to_win(win, fb_mem);
	if (rv) {
		E("Framebuffer Pool is Full. ");
		goto err;
	}
	
	/* Always set first block as DRV Framebuffer. */
	set_drv_framebuffer(win, fb_mem);

	rv = alloc_cmap(win, bpp);
	if (rv) {
		E("Failed to allocate color map.");
		goto err;
	}

	return win;

err:	
	if (test_bit(2, &status))
		free_framebuffer(win);
	
	if (test_bit(1, &status))
		__free_page(win->misc_mem_start);

	if (test_bit(0, &status)) 
		kfree(win);

	return NULL;	
}

static void free_win(struct jz_fb_win_info *win)
{
	int i;

	D("Called.");
	
	/* Free all framebuffers in WIN. */
	for (i = 0; i < JZ_FB_NR_MAX_FB_MEM; i++) {
		if (win->fb_mem[i]) {
			free_framebuffer(win->fb_mem[i]);
		}
	}
	
	/* Free color maps. */
	free_cmap(win);
	
	/* Free Misc Memory. */
	__free_page(win->misc_mem_start);
	
	/* Free Win. */
	kfree(win);

	return;
}

static int jz_fb_win_set_attr(struct jz_fb_win_info *win, struct jz_fb_win_attr_info *attr)
{
	D("Called.");

	memcpy(&win->attr, attr, sizeof(struct jz_fb_win_attr_info));

	update_fb_data_size(win);
	
	return 0;
}

static inline int jz_fb_win_active_attr(struct jz_fb_win_info *win)
{
	D("Called.");

	/* Not implentmend yet. */
	return 0;
}
