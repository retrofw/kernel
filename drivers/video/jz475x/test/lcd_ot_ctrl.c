/*
 * LCD Output Path Control.
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
#include <stdint.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <string.h>

#include "../abi.h"

int main(int argc, char **argv)
{
	struct jz_fb_lcd_control control;
	struct jz_fb_common_control *c = &control.common;

	int fd;	
	int rv;

	if (argc < 2)
		return -1;

	fd = open("/dev/lcd-ot-ctrl", O_RDWR);
	if (fd == -1) {
		perror("open():");
		exit(EXIT_FAILURE);
	}
	
	c->id = 0;
	
	if (!strcmp(argv[1], "power")) {
		if (argc < 3)
			return -1;

		c->command = JZ_FB_CMD_COMMON_SET_POWER;
		c->v = strcmp(argv[2], "off");

		rv = ioctl(fd, 0, &control);
		if (rv) {
			perror("ioctl");
			return -1;
		}		

	}else if (!strcmp(argv[1], "screen")) {
		if (argc < 3)
			return -1;

		c->command = JZ_FB_CMD_COMMON_SET_SCREEN;
		c->v = strcmp(argv[2], "off");

		rv = ioctl(fd, 0, &control);
		if (rv) {
			perror("ioctl");
			return -1;
		}		

	}else if (!strcmp(argv[1], "get_attr")) {
		c->command = JZ_FB_CMD_COMMON_GET_WIN_ATTR;
		
		rv = ioctl(fd, 0, &control);
		if (rv) {
			perror("ioctl");
			return -1;
		}		

		fprintf(stderr, "Current Win ATTR:\n");
		fprintf(stderr, "WIN 0: enable: %d, x: %d, y: %d, w: %d, h: %d, bpp: %d.\n",
				c->win_attr[0].enable,
				c->win_attr[0].x,
				c->win_attr[0].y,
				c->win_attr[0].w,
				c->win_attr[0].h,
				c->win_attr[0].bpp);

		fprintf(stderr, "WIN 1: enable: %d, x: %d, y: %d, w: %d, h: %d, bpp: %d.\n",
				c->win_attr[1].enable,
				c->win_attr[1].x,
				c->win_attr[1].y,
				c->win_attr[1].w,
				c->win_attr[1].h,
				c->win_attr[1].bpp);

		return 0;

	}else if (!strcmp(argv[1], "set_attr")) {
		/*    1         2           3     4   5   6   7   8      9      */
		/* set_attr [win index] [enable] [x] [y] [w] [h] [bpp] [active] */

		unsigned int index = atoi(argv[2]);
		unsigned int enable = atoi(argv[3]);
		unsigned int x = atoi(argv[4]);
		unsigned int y = atoi(argv[5]);
		unsigned int w = atoi(argv[6]);
		unsigned int h = atoi(argv[7]);
		unsigned int bpp = atoi(argv[8]);
		unsigned int active = atoi(argv[9]);

		if (argc < 9 || index > JZ_FB_NR_MAX_FG)
			return -1;
		
		/* Get current win attr. */
		c->command = JZ_FB_CMD_COMMON_GET_WIN_ATTR;
		
		rv = ioctl(fd, 0, &control);
		if (rv) {
			perror("ioctl");
			return -1;
		}		

		c->command = JZ_FB_CMD_COMMON_SET_WIN_ATTR;
		
		c->active_now = active;

		c->win_attr[index].enable = enable;	
		c->win_attr[index].x = x;	
		c->win_attr[index].y = y;	
		c->win_attr[index].w = w;	
		c->win_attr[index].h = h;	
		c->win_attr[index].bpp = bpp;

		rv = ioctl(fd, 0, &control);
		if (rv) {
			perror("ioctl");
			return -1;
		}		
		
		return 0;	
	}else if (!strcmp(argv[1], "select")) {
		c->command = JZ_FB_CMD_COMMON_SELECT_OT;
		
		rv = ioctl(fd, 0, &control);
		if (rv) {
			perror("ioctl");
			return -1;
		}		
		
	}

	close(fd);

	return 0;
}
