/*
 * Framebuffer Test Common Routines.
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <string.h>
#include <linux/fb.h>

struct fb {
	int fd;
	uint32_t w;
	uint32_t h;
	uint32_t bpp;
	uint32_t bpp_byte;
	uint32_t mem_size;
	void *mem;
};

int fb_open(char *name, struct fb *fb)
{
	struct fb_var_screeninfo var;
	
	int rv;

	fb->fd = open(name, O_RDWR);
	if (fb->fd == -1) {
		perror("open():");
		return -1;
	}
	
	rv = ioctl(fb->fd, FBIOGET_VSCREENINFO, &var);
	if (rv) {
		perror("ioctl");
		close(fb->fd);
		return -1;
	}
	
	fb->w = var.xres;
	fb->h = var.yres;
	fb->bpp = var.bits_per_pixel;
	fb->bpp_byte = fb->bpp / 8;
	fb->mem_size = fb->w * fb->h * fb->bpp_byte;

	fb->mem = mmap(NULL, fb->mem_size, PROT_READ | PROT_WRITE, MAP_SHARED, fb->fd, 0L);
	if (fb->mem == (void *)-1) {
		perror("mmap():");
		close(fb->fd);
		return -1;
	}

	fprintf(stderr, "%s(): w: %d h: %d bpp: %d bpp_byte: %d mem_size: 0x%lu mem: %p.\n", __func__, fb->w, fb->h, fb->bpp, fb->bpp_byte, fb->mem_size, fb->mem);

	return 0;
}

int fb_close(struct fb *fb)
{
	close(fb->fd);
	munmap(fb->mem, fb->mem_size);

	return 0;
}

