/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * Framebuffer Routines.
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

/* ------------ Framebuffer ----------------- */
static void dump_fb_info_var(struct fb_var_screeninfo *var)
{
	DUMP("Framebuffer VAR Info: ");
	DUMP("=============================================");

	DUMP("var->xres: 0x%08lx.",var->xres);
	DUMP("var->yres: 0x%08lx.",var->yres);
	DUMP("var->xres_virtual: 0x%08lx.",var->xres_virtual);
	DUMP("var->yres_virtual: 0x%08lx.",var->yres_virtual);
	DUMP("var->xoffset: 0x%08lx.",var->xoffset);
	DUMP("var->yoffset: 0x%08lx.",var->yoffset);
	DUMP("var->bits_per_pixel: 0x%08lx.",var->bits_per_pixel);
	DUMP("var->grayscale: 0x%08lx.",var->grayscale);
	DUMP("var->nonstd: 0x%08lx.",var->nonstd);
	DUMP("var->activate: 0x%08lx.",var->activate);
	DUMP("var->height: 0x%08lx.",var->height);
	DUMP("var->width: 0x%08lx.",var->width);
	DUMP("var->accel_flags: 0x%08lx.",var->accel_flags);
	DUMP("var->pixclock: 0x%08lx.",var->pixclock);
	DUMP("var->left_margin: 0x%08lx.",var->left_margin);
	DUMP("var->right_margin: 0x%08lx.",var->right_margin);
	DUMP("var->upper_margin: 0x%08lx.",var->upper_margin);
	DUMP("var->lower_margin: 0x%08lx.",var->lower_margin);
	DUMP("var->hsync_len: 0x%08lx.",var->hsync_len);
	DUMP("var->vsync_len: 0x%08lx.",var->vsync_len);
	DUMP("var->sync: 0x%08lx.",var->sync);
	DUMP("var->vmode: 0x%08lx.",var->vmode);
	DUMP("var->rotate: 0x%08lx.",var->rotate);

	return;
}

static void dump_fb_info_fix(struct fb_fix_screeninfo *fix)
{
	DUMP("Framebuffer FIX Info: ");
	DUMP("=============================================");

	DUMP("fix->id: %s.",fix->id);

	DUMP("fix->smem_start: 0x%08lx.",fix->smem_start);
	DUMP("fix->smem_len: 0x%08lx.",fix->smem_len);
	DUMP("fix->type: 0x%08lx.",fix->type);
	DUMP("fix->type_aux: 0x%08lx.",fix->type_aux);
	DUMP("fix->visual: 0x%08lx.",fix->visual);
	DUMP("fix->xpanstep: 0x%08lx.",fix->xpanstep);
	DUMP("fix->ypanstep: 0x%08lx.",fix->ypanstep);
	DUMP("fix->ywrapstep: 0x%08lx.",fix->ywrapstep);
	DUMP("fix->line_length: 0x%08lx.",fix->line_length);
	DUMP("fix->mmio_start: 0x%08lx.",fix->mmio_start);
	DUMP("fix->mmio_len: 0x%08lx.",fix->mmio_len);
	DUMP("fix->accel: 0x%08lx.",fix->accel);

	return;
}

static inline int bpp_to_data_bpp(int bpp)
{
	switch (bpp) {
		case 32:
		case 16:
			break;

		case 15:
			bpp = 16;
			break;

		default:
			bpp = -EINVAL;
	}

	return bpp;
}

static void *alloc_framebuffer(unsigned long len)
{
	void *mem;

	unsigned int page_shift;
	unsigned long page;

	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= len)
			break;

	mem = (void *)__get_free_pages(GFP_KERNEL, page_shift);
	if (!mem) {
		return NULL;
	}

	for (page = (unsigned long)mem;
			page < PAGE_ALIGN((unsigned long)mem + (PAGE_SIZE << page_shift));
			page += PAGE_SIZE) {
		SetPageReserved(virt_to_page((void*)page));
	}

	memset(mem, 0, PAGE_SIZE << page_shift);	

	D("Allocate framebuffer: Real Size: 0x%08lx, Virt: 0x%08lx, Phys: 0%08lx", 
			PAGE_SIZE << page_shift, (unsigned long)mem, virt_to_phys(mem));

	return mem;
}

static int free_framebuffer(void *mem)
{
	return 0;
}

/* Add framebuffer to Win Framebuffer Pool. */
static int add_framebuffer_to_win(struct jz_fb_win_info *win, void *mem)
{
	int i;

	D("Called.");

	for (i = 0; i < JZ_FB_NR_MAX_FB_MEM; i++) {
		if (!win->fb_mem[i]) {
			win->fb_mem[i] = mem;
			return 0;
		}
	}

	return -1;
}

static int set_drv_framebuffer(struct jz_fb_win_info *win, void *mem)
{
	D("Called.");

	win->fb.fix.smem_start = virt_to_phys(mem);
	win->fb.fix.smem_len = win->fb_mem_len;
	win->fb.screen_base = (unsigned char *)(((unsigned long)mem&0x1fffffff) | 0xa0000000);

	return 0;
}

static int alloc_cmap(struct jz_fb_win_info *win, unsigned int bpp)
{
	int rv;

	switch (bpp) {
		case 1:
			rv = fb_alloc_cmap(&win->fb.cmap, 4, 0);
			break;
		case 2:
			rv = fb_alloc_cmap(&win->fb.cmap, 8, 0);
			break;
		case 4:
			rv = fb_alloc_cmap(&win->fb.cmap, 32, 0);
			break;
		case 8:

		default:
			rv = fb_alloc_cmap(&win->fb.cmap, 256, 0);
			break;
	}

	return rv;
}

static void free_cmap(struct jz_fb_win_info *win) 
{
	fb_dealloc_cmap(&win->fb.cmap);

	return;
}

static struct fb_ops jz_fb_ops;

static int fb_info_set_color(struct fb_info *info)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;

	D("Called.");

	dump_fb_info_var(var);

	switch (var->bits_per_pixel) {
		case 1:	/* Mono */
			fix->visual		= FB_VISUAL_MONO01;
			fix->line_length	= (var->xres * var->bits_per_pixel) / 8;

			break;

		case 2:	/* Mono */
			var->red.offset		= 0;
			var->red.length		= 2;
			var->green.offset	= 0;
			var->green.length	= 2;
			var->blue.offset	= 0;
			var->blue.length	= 2;

			fix->visual		= FB_VISUAL_PSEUDOCOLOR;
			fix->line_length	= (var->xres * var->bits_per_pixel) / 8;

			break;

		case 4:	/* PSEUDOCOLOUR */
			var->red.offset		= 0;
			var->red.length		= 4;
			var->green.offset	= 0;
			var->green.length	= 4;
			var->blue.offset	= 0;
			var->blue.length	= 4;

			fix->visual		= FB_VISUAL_PSEUDOCOLOR;
			fix->line_length	= var->xres / 2;

			break;

		case 8:	/* PSEUDOCOLOUR, 256 */
			var->red.offset		= 0;
			var->red.length		= 8;
			var->green.offset	= 0;
			var->green.length	= 8;
			var->blue.offset	= 0;
			var->blue.length	= 8;

			fix->visual		= FB_VISUAL_PSEUDOCOLOR;
			fix->line_length	= var->xres;

			break;

		case 15: /* DIRECTCOLOUR, 32k */
			var->red.offset		= 10;
			var->red.length		= 5;
			var->green.offset	= 5;
			var->green.length	= 5;
			var->blue.offset	= 0;
			var->blue.length	= 5;

			fix->visual		= FB_VISUAL_DIRECTCOLOR;
			fix->line_length	= var->xres_virtual * 2;

			break;

		case 16: /* DIRECTCOLOUR, 64k */
			var->red.offset		= 11;
			var->red.length		= 5;
			var->green.offset	= 5;
			var->green.length	= 6;
			var->blue.offset	= 0;
			var->blue.length	= 5;

			fix->visual		= FB_VISUAL_TRUECOLOR;
			fix->line_length	= var->xres_virtual * 2;

			break;

		case 17 ... 32:
			/* DIRECTCOLOUR, 256 */
			var->bits_per_pixel	= 32;

			var->red.offset		= 16;
			var->red.length		= 8;
			var->green.offset	= 8;
			var->green.length	= 8;
			var->blue.offset	= 0;
			var->blue.length	= 8;
			var->transp.offset  	= 24;
			var->transp.length 	= 8;

			fix->visual		= FB_VISUAL_TRUECOLOR;
			fix->line_length	= var->xres_virtual * 4;

			break;

		default:
			E("BPP %d not support.", var->bits_per_pixel);

			return -EINVAL;
	}

	D("fix->line_length: 0x%08x.", fix->line_length);

	fb_set_cmap(&info->cmap, info);

	return 0;
}

/* Fiil FB Info. */
static int win_fb_info_setup(struct jz_fb_win_info *win)
{
	struct jz_fb_ctrl *ctrl = win->ctrl;

	int rv;

	strcpy(win->fb.fix.id, DRV_NAME);

	win->fb.fix.type		   = FB_TYPE_PACKED_PIXELS;
	win->fb.fix.type_aux		   = 0;
	win->fb.fix.xpanstep		   = 0;
	win->fb.fix.ypanstep		   = 1;
	win->fb.fix.ywrapstep		   = 0;
	win->fb.fix.accel		   = FB_ACCEL_NONE;

	win->fb.fbops			   = &jz_fb_ops;
	win->fb.flags			   = FBINFO_FLAG_DEFAULT;

	win->fb.var.nonstd		   = 0;
	win->fb.var.activate		   = FB_ACTIVATE_NOW;
	win->fb.var.accel_flags		   = 0;
	win->fb.var.vmode                  = FB_VMODE_NONINTERLACED;

	win->fb.var.xres                   = WIN_W(win);
	win->fb.var.yres                   = WIN_H(win);
	win->fb.var.xres_virtual           = WIN_W(win);
	win->fb.var.yres_virtual           = WIN_H(win);
	win->fb.var.bits_per_pixel	   = WIN_BPP(win);
	win->fb.var.xoffset                = 0;
	win->fb.var.yoffset                = 0;

	win->fb.var.pixclock               = ctrl->pixclock;

	win->fb.var.left_margin            = 0;
	win->fb.var.right_margin           = 0;
	win->fb.var.upper_margin           = 0;
	win->fb.var.lower_margin           = 0;
	win->fb.var.hsync_len              = 0;
	win->fb.var.vsync_len              = 0;
	win->fb.var.sync                   = 0;

	win->fb.pseudo_palette		   = (void *)(win + 1);

	rv = fb_info_set_color(&win->fb);
	if (rv) {
		return rv;
	}

	return rv;
}
