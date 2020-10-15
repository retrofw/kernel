/*
 * Copyright (c) International Business Machines Corp., 2006
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * An utility to generate input file CRC
 *
 * Authors:   Yurong Tan (Nancy)
 */

#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <getopt.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>

#include <libubi.h>
#include "common.h"
#include "crc32.h"

#define PROGRAM_VERSION "1.1"
#define PROGRAM_NAME    "ubicrcsf"
#define UBI_LEB_SIZE  258048
#define BUF_SIZE (UBI_LEB_SIZE + sizeof(unsigned int))

/*
 * usage: $ubicrcsf ubifs.img
 */
int main(int argc, char * const argv[])
{
	int err, ifd, tmp, i;
	int crc_sum = 0;
	struct stat st;
	char *buf=NULL;
	
	buf = malloc(BUF_SIZE);
	if(buf==NULL){
		printf("no mem\n");
		goto out_free;		
	}

	err = stat(argv[1], &st);
	if (err < 0) {
		printf("stat failed on \"%s\"", argv[1]);
		goto out_free;
	}
	
	ifd = open(argv[1],  O_RDONLY);
	if (ifd == -1) {
		printf("cannot open \"%s\"", argv[1]);
		goto out_close;
	}
	
	tmp = st.st_size/BUF_SIZE;

	for( i=0;  i< tmp;  i++ ){
		err = read(ifd,  buf, BUF_SIZE);
		if (err != BUF_SIZE) {
			printf("read error\n");
			goto out_close;
		}
		crc_sum = crc32(crc_sum, &buf[sizeof(unsigned int)], UBI_LEB_SIZE);
	}

	printf("CRC sum: %d\n",crc_sum);
	free(buf);
	close(ifd);
	return 0;
out_close:
	close(ifd);
out_free:
	free(buf);
	return -1;
}


