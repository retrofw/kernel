/*
 * Framebuffer Fill Test.
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

#include "fb_common.c"

int main(int argc, char **argv)
{
	struct fb fb;
	uint32_t word;

	void *mem;
	int fd;	

	char *p;

	uint32_t i;

	int rv;

	if (argc != 3)
		return -1;
	
	word = strtoul(argv[2], &p, 16);	
	if (*p != '\0') {
		fprintf(stderr, "Invalid value.\n");
		exit(EXIT_FAILURE);
	}
	
	rv = fb_open(argv[1], &fb);
	if (rv) {
		fprintf(stderr, "Failed to open framebuffer device.\n");
		exit(EXIT_FAILURE);
	}

	fprintf(stderr, "Word: 0x%x, Bpp_byte: 0x%u.\n", word, fb.bpp_byte);
	
	mem = fb.mem;

	for (i = 0; i < fb.mem_size; i += fb.bpp_byte) {
		memcpy(mem, &word, fb.bpp_byte);
		mem += fb.bpp_byte;
	}
	
	fb_close(&fb);

	return 0;
}
