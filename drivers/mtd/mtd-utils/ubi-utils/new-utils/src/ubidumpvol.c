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

#define PROGRAM_VERSION "1.1"
#define PROGRAM_NAME    "ubidumpvol"

struct args {
	int truncate;
	const char *node;
	const char *img;
	/* For deprecated -d and -B options handling */
	int devn;
	char dev_name[256];
	int broken_update;
};

static struct args args = {
	.devn = -1,
};

static const char *doc = PROGRAM_NAME " version " PROGRAM_VERSION
			 " - a tool to write data to UBI volumes.";

static const char *optionsstr =
"-n, --vol_id=<volume id>   ID of UBI volume to update\n"
"-t, --truncate             truncate volume (wipe it out)\n"
"-h, --help                 print help message\n"
"-V, --version              print program version\n\n"
"The following are compatibility options which are deprecated, do not use them\n"
"-d, --devn=<devn>          UBI device number - may be used instead of the UBI\n"
"                           device node name in which case the utility assumes\n"
"                           that the device node is \"/dev/ubi<devn>\"\n"
"-B, --broken-update        broken update, this is for testing";

static const char *usage =
"Usage: " PROGRAM_NAME " <UBI volume node file name> [-t] [-h] [-V] [--truncate] [--help]\n"
"\t\t\t[--version] <image file>\n\n"
	"Example 1: " PROGRAM_NAME " /dev/ubi0_1 fs.img - dump UBI volume /dev/ubi0_1 to file \"fs.img\" \n";

struct option long_options[] = {
	{ .name = "truncate", .has_arg = 0, .flag = NULL, .val = 't' },
	{ .name = "help",     .has_arg = 0, .flag = NULL, .val = 'h' },
	{ .name = "version",  .has_arg = 0, .flag = NULL, .val = 'V' },
	/* Deprecated -d and -B options */
	{ .name = "devn",     .has_arg = 1, .flag = NULL, .val = 'd' },
	{ .name = "broken-update", .has_arg = 1, .flag = NULL, .val = 'B' },
	{ NULL, 0, NULL, 0}
};

static int parse_opt(int argc, char * const argv[])
{
	while (1) {
		int key;

		key = getopt_long(argc, argv, "n:th?Vd:", long_options, NULL);
		if (key == -1)
			break;

		switch (key) {
		case 't':
			args.truncate = 1;
			break;

		case 'h':
		case '?':
			fprintf(stderr, "%s\n\n", doc);
			fprintf(stderr, "%s\n\n", usage);
			fprintf(stderr, "%s\n", optionsstr);
			exit(EXIT_SUCCESS);

		case 'd':
		{
			char *endp;

			/* Handle deprecated -d option */
			warnmsg("-d is depricated and will be removed, do not use it");
			args.devn = strtoul(optarg, &endp, 0);
			if (*endp != '\0' || endp == optarg || args.devn < 0)
				return errmsg("bad UBI device number: " "\"%s\"", optarg);
			break;
		}

		case 'B':
			/* Handle deprecated -B option */
			warnmsg("-B is depricated and will be removed, do not use it");
			args.broken_update = 1;
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
		else if (optind != argc - 2)
			return errmsg("specify UBI device name and image file name as first 2 "
				      "parameters (use -h for help)");
	}

	args.node = argv[optind];
	args.img  = argv[optind + 1];

	return 0;
}

static int dump_volume(libubi_t libubi, struct ubi_vol_info *vol_info)
{
	int err, fd, ifd;
	struct ubi_leb leb;
	int i, tmp;

	leb.buf = malloc(vol_info->leb_size);
	if (!leb.buf)
		return errmsg("cannot allocate %d bytes of memory", vol_info->leb_size);

	fd = open(args.node, O_RDONLY);
	if (fd == -1) {
		sys_errmsg("cannot open UBI volume \"%s\"", args.node);
		goto out_free;
	}

	ifd = open(args.img, O_WRONLY | O_TRUNC | O_CREAT, 0644);
	if (ifd == -1) {
		sys_errmsg("cannot open \"%s\"", args.img);
		goto out_close1;
	}
	
	for(i=0; i < vol_info->rsvd_lebs; i++){
		leb.lnum = i;
		tmp = ubi_leb_read_start(fd, &leb);
		if(tmp == 1)
			continue;
		else if(tmp == 0){
			// write lnum
			err = write(ifd, (char *)&leb.lnum, sizeof(leb.lnum));
			if (err != sizeof(leb.lnum)){
				perror("Image file write error\n");
				goto out_close;	
			}
			// write LEB data
			err = write(ifd, leb.buf, vol_info->leb_size);
			if (err != vol_info->leb_size){
				perror("Image file write error\n");
				goto out_close;	
			}
		}else{
			printf("LEB %d read error\n", i);
			goto out_close;
		}
	}

	close(ifd);
	close(fd);
	free(leb.buf);
	printf("Dump Volume succeed\n");
	return 0;
	goto out_close;
out_close:
	close(ifd);
out_close1:
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

	if (!args.img && !args.truncate)
		return errmsg("incorrect arguments, use -h for help");

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

	err = dump_volume(libubi, &vol_info);
	if (err)
		goto out_libubi;
	
	libubi_close(libubi);
	return 0;

out_libubi:
	libubi_close(libubi);
	return -1;
}
