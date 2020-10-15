/*
 * Dump Framebuffer Info.
 *  
 * Copyright (C) 2005-2010 Ingenic Semiconductor Inc.
 * Author: River Wang <zwang@ingenic.cn>
 * 
 *  Based on original LCDC Driver.
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <linux/fb.h>

#define DUMP(fmt, args...) \
	fprintf(stderr, fmt"\n", ##args)

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

int main(int argc, char **argv)
{
	struct fb_var_screeninfo var;
	struct fb_fix_screeninfo fix;
	
	int fd;	

	int rv;

	if (argc != 2)
		return -1;

	fd = open(argv[1], O_RDWR);
	if (fd == -1) {
		perror("open():");
		exit(EXIT_FAILURE);
	}
	
	rv = ioctl(fd, FBIOGET_VSCREENINFO, &var);
	if (rv) {
		perror("ioctl():");
		exit(EXIT_FAILURE);
	}

	rv = ioctl(fd, FBIOGET_FSCREENINFO, &fix);
	if (rv) {
		perror("ioctl():");
		exit(EXIT_FAILURE);
	}	

	dump_fb_info_var(&var);
	dump_fb_info_fix(&fix);

	return 0;
}
