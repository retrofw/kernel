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
 * An utility to update UBI volumes.
 *
 * Authors: Frank Haverkamp
 *          Joshua W. Boyer
 *          Artem Bityutskiy
 *          Yurong Tan (Nancy)
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
#define PROGRAM_NAME    "ubicrcfatvol"

struct args {
	const char *node;
	long long size;
	int devn;
	char dev_name[256];
};

static struct args args = {
	.devn = -1,
};

static const char *doc = PROGRAM_NAME " version " PROGRAM_VERSION
			 " - a tool to read UBI volumes data and generate CRC.";

static const char *optionsstr =
"-h, --help                 print help message\n"
"-V, --version              print program version\n\n"
"-s, --read size\n"          
;

static const char *usage =
"Usage: " PROGRAM_NAME " <UBI volume node file name> [-s] [-h] [-V] [--help]\n"
"\t\t\t[--version] \n\n"
	"Example 1: " PROGRAM_NAME " /dev/ubi0_1 \n";

struct option long_options[] = {
	{ .name = "help",     .has_arg = 0, .flag = NULL, .val = 'h' },
	{ .name = "version",  .has_arg = 0, .flag = NULL, .val = 'V' },
	/* Deprecated -d and -B options */
	{ .name = "size",     .has_arg = 1, .flag = NULL, .val = 's' },
	{ NULL, 0, NULL, 0}
};

static int parse_opt(int argc, char * const argv[])
{
	while (1) {
		int key;

		key = getopt_long(argc, argv, "n:sh?Vd:", long_options, NULL);
		if (key == -1)
			break;

		switch (key) {

		case 'h':
		case '?':
			fprintf(stderr, "%s\n\n", doc);
			fprintf(stderr, "%s\n\n", usage);
			fprintf(stderr, "%s\n", optionsstr);
			exit(EXIT_SUCCESS);

		case 's':
			args.size = ubiutils_get_bytes(optarg);
			if (args.size <= 0)
				return errmsg("bad read size: \"%s\"", optarg);
			break;
			
		case 'V':
			fprintf(stderr, "%s\n", PROGRAM_VERSION);
			exit(EXIT_SUCCESS);

		case ':':
			return errmsg("parameter is missing");

		default:
			fprintf(stderr, "Use -h for help\n");
			return -1;
		}
	}

	/* Handle deprecated -d option */
	if (args.devn != -1) {
		sprintf(args.dev_name, "/dev/ubi%d", args.devn);
		args.node = args.dev_name;
	} else {
		if (optind == argc)
			return errmsg("UBI device name was not specified (use -h for help)");
		else if (optind != argc - 1)
			return errmsg("specify UBI device name and image file name as first 2 "
				      "parameters (use -h for help)");
	}

	args.node = argv[optind];

	return 0;
}

static int crc_volume(libubi_t libubi, struct ubi_vol_info *vol_info)
{
	int fd;
	struct ubi_leb leb;
	int i, tmp;
	int crc_sum = 0;

	leb.buf = malloc(vol_info->leb_size);
	if (!leb.buf)
		return errmsg("cannot allocate %d bytes of memory", vol_info->leb_size);

	fd = open(args.node, O_RDONLY);
	if (fd == -1) {
		sys_errmsg("cannot open UBI volume \"%s\"", args.node);
		goto out_free;
	}

	for(i=0; i < vol_info->rsvd_lebs ; i++){
		leb.lnum = i;
		tmp = ubi_leb_read_start(fd, &leb);
		if(tmp == 1)
			continue;		
		else if(tmp == 0){ 
			crc_sum = crc32(crc_sum, leb.buf, vol_info->leb_size); 		
		}else{
			printf("LEB %d read error\n", i);
			goto out_close;
		}
	}

	close(fd);
	free(leb.buf);
	printf("CRC sum: %d\n",crc_sum);
	return 0;
	goto out_close;
out_close:
	close(fd);
out_free:
	free(leb.buf);
	return -1;
}


int main(int argc, char * const argv[])
{
	int err;
	libubi_t libubi;
	struct ubi_vol_info vol_info;

	err = parse_opt(argc, argv);
	if (err)
		return -1;
		
	libubi = libubi_open(1);
	if (libubi == NULL) {
		sys_errmsg("cannot open libubi");
		goto out_libubi;
	}

	err = ubi_node_type(libubi, args.node);
	if (err == 1) {
		errmsg("\"%s\" is an UBI device node, not an UBI volume node",
		       args.node);
		goto out_libubi;
	} else if (err < 0) {
		errmsg("\"%s\" is not an UBI volume node", args.node);
		goto out_libubi;
	}

	err = ubi_get_vol_info(libubi, args.node, &vol_info);
	if (err) {
		sys_errmsg("cannot get information about UBI volume \"%s\"",
			   args.node);
		goto out_libubi;
	}

	err = crc_volume(libubi, &vol_info);
	if (err)
		goto out_libubi;
	
	libubi_close(libubi);
	return 0;

out_libubi:
	libubi_close(libubi);
	return -1;

}


