/*
 *  nandwrite.c
 *
 *  Copyright (C) 2000 Steven J. Hill (sjhill@realitydiluted.com)
 *		  2003 Thomas Gleixner (tglx@linutronix.de)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Overview:
 *   This utility writes a binary image directly to a NAND flash
 *   chip or NAND chips contained in DoC devices. This is the
 *   "inverse operation" of nanddump.
 *
 * tglx: Major rewrite to handle bad blocks, write data with or without ECC
 *	 write oob data only on request
 *
 * Bug/ToDo:
 */

#define _GNU_SOURCE
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <getopt.h>

#include <asm/types.h>
#include "mtd/mtd-user.h"

#define PROGRAM "nandwrite"
#define VERSION "$Revision: 1.1.1.1 $"

#define MAX_PAGE_SIZE	8192
#define MAX_OOB_SIZE	256
/*
 * Buffer array used for writing data
 */
unsigned char writebuf[MAX_PAGE_SIZE];
unsigned char oobreadbuf[MAX_OOB_SIZE];

// oob layouts to pass into the kernel as default
struct nand_oobinfo none_oobinfo = {
	.useecc = MTD_NANDECC_OFF,
};

struct nand_oobinfo jffs2_oobinfo = {
	.useecc = MTD_NANDECC_PLACE,
	.eccbytes = 6,
	.eccpos = { 0, 1, 2, 3, 6, 7 }
};

struct nand_oobinfo yaffs_oobinfo = {
	.useecc = MTD_NANDECC_PLACE,
	.eccbytes = 6,
	.eccpos = { 8, 9, 10, 13, 14, 15}
};

struct nand_oobinfo autoplace_oobinfo = {
	.useecc = MTD_NANDECC_AUTOPLACE,
	.eccbytes = 36
};

void display_help (void)
{
	printf("Usage: nandwrite [OPTION] MTD_DEVICE INPUTFILE\n"
			"Writes to the specified MTD device.\n"
			"\n"
			"  -a, --autoplace	Use auto oob layout\n"
			"  -j, --jffs2		force jffs2 oob layout (legacy support)\n"
			"  -y, --yaffs		force yaffs oob layout (legacy support)\n"
			"  -f, --forcelegacy	force legacy support on autoplacement enabled mtd device\n"
			"  -m, --markbad		mark blocks bad if write fails\n"
			"  -n, --noecc		write without ecc\n"
			"  -o, --oob		image contains oob data\n"
			"  -s addr, --start=addr set start address (default is 0)\n"
			"  -p, --pad             pad to page size\n"
			"  -b, --blockalign=1|2|4 set multiple of eraseblocks to align to\n"
			"  -q, --quiet		don't display progress messages\n"
			"      --help		display this help and exit\n"
			"      --version		output version information and exit\n");
	exit(0);
}

void display_version (void)
{
	printf(PROGRAM " " VERSION "\n"
			"\n"
			"Copyright (C) 2003 Thomas Gleixner \n"
			"\n"
			PROGRAM " comes with NO WARRANTY\n"
			"to the extent permitted by law.\n"
			"\n"
			"You may redistribute copies of " PROGRAM "\n"
			"under the terms of the GNU General Public Licence.\n"
			"See the file `COPYING' for more information.\n");
	exit(0);
}

char	*mtd_device, *img;
unsigned long long mtdoffset = 0;
int	quiet = 0;
int	writeoob = 0;
int	markbad = 1;
int	autoplace = 0;
int	forcejffs2 = 0;
int	forceyaffs = 0;
int	forcelegacy = 0;
int	noecc = 0;
int	pad = 0;
int	blockalign = 1; /*default to using 16K block size */

void process_options (int argc, char *argv[])
{
	int error = 0;

	for (;;) {
		int option_index = 0;
		static const char *short_options = "ab:fjmnopqs:y";
		static const struct option long_options[] = {
			{"help", no_argument, 0, 0},
			{"version", no_argument, 0, 0},
			{"autoplace", no_argument, 0, 'a'},
			{"blockalign", required_argument, 0, 'b'},
			{"forcelegacy", no_argument, 0, 'f'},
			{"jffs2", no_argument, 0, 'j'},
			{"markbad", no_argument, 0, 'm'},
			{"noecc", no_argument, 0, 'n'},
			{"oob", no_argument, 0, 'o'},
			{"pad", no_argument, 0, 'p'},
			{"quiet", no_argument, 0, 'q'},
			{"start", required_argument, 0, 's'},
			{"yaffs", no_argument, 0, 'y'},
			{0, 0, 0, 0},
		};

		int c = getopt_long(argc, argv, short_options,
				long_options, &option_index);
		if (c == EOF) {
			break;
		}

		switch (c) {
			case 0:
				switch (option_index) {
					case 0:
						display_help();
						break;
					case 1:
						display_version();
						break;
				}
				break;
			case 'q':
				quiet = 1;
				break;
			case 'a':
				autoplace = 1;
				break;
			case 'j':
				forcejffs2 = 1;
				break;
			case 'y':
				forceyaffs = 1;
				break;
			case 'f':
				forcelegacy = 1;
				break;
			case 'n':
				noecc = 1;
				break;
			case 'm':
				markbad = 1;
				break;
			case 'o':
				writeoob = 1;
				break;
			case 'p':
				pad = 1;
				break;
			case 's':
				mtdoffset = strtol (optarg, NULL, 0);
				break;
			case 'b':
				blockalign = atoi (optarg);
				break;
			case '?':
				error = 1;
				break;
		}
	}

	if ((argc - optind) != 2 || error)
		display_help ();

	mtd_device = argv[optind++];
	img = argv[optind];
}

/*
 * Main program
 */
int main(int argc, char **argv)
{
	int cnt, fd, ifd, imglen = 0, pagelen, baderaseblock, blockstart = -1;
	struct mtd_info_user meminfo;
	struct mtd_page_buf oob;
	loff_mtd_t offs;
	int ret, readlen;
	int oobinfochanged = 0;
	struct nand_oobinfo old_oobinfo;
	int i;
	
	process_options(argc, argv);

	if (pad && writeoob) {
		fprintf(stderr, "Can't pad when oob data is present.\n");
		exit(1);
	}

	/* Open the device */
	if ((fd = open(mtd_device, O_RDWR)) == -1) {
		perror("open flash");
		exit(1);
	}

	/* Fill in MTD device capability structure */
	if (ioctl(fd, MEMGETINFO, &meminfo) != 0) {
		perror("MEMGETINFO");
		close(fd);
		exit(1);
	}

	/* Set erasesize to specified number of blocks - to match jffs2
	 * (virtual) block size */
	meminfo.erasesize *= blockalign;

	/* Make sure device page sizes are valid */
	if (!(meminfo.oobsize == 16 && meminfo.writesize == 512) &&
			!(meminfo.oobsize == 8 && meminfo.writesize == 256) &&
			!(meminfo.oobsize == 64 && meminfo.writesize == 2048) &&
			!(meminfo.oobsize == 128 && meminfo.writesize == 4096) &&
			!(meminfo.oobsize == 218 && meminfo.writesize == 4096) &&
			!(meminfo.oobsize == 256 && meminfo.writesize == 8192) &&
			!(meminfo.oobsize == 436 && meminfo.writesize == 8192) &&
			!(meminfo.oobsize == 640 && meminfo.writesize == 8192)) {
		fprintf(stderr, "Unknown flash (not normal NAND)\n");
		close(fd);
		exit(1);
	}

	if (autoplace) {
		/* Read the current oob info */
		if (ioctl (fd, MEMGETOOBSEL, &old_oobinfo) != 0) {
			perror ("MEMGETOOBSEL");
			close (fd);
			exit (1);
		}

		// autoplace ECC ?
		if (autoplace && (old_oobinfo.useecc != MTD_NANDECC_AUTOPLACE)) {

			if (ioctl (fd, MEMSETOOBSEL, &autoplace_oobinfo) != 0) {
				perror ("MEMSETOOBSEL");
				close (fd);
				exit (1);
			}
			oobinfochanged = 1;
		}
	}

	memset(oobreadbuf, 0xff, MAX_OOB_SIZE);

	if (autoplace) {
		oob.ooblength = meminfo.oobsize-old_oobinfo.eccbytes; /* Get ooblength from kernel */
		printf("oobsize=%d eccbytes=%d\n", meminfo.oobsize, old_oobinfo.eccbytes);
	} else {
		oob.ooblength = meminfo.oobsize-autoplace_oobinfo.eccbytes;
		printf("oobsize=%d eccbytes=%d\n", meminfo.oobsize, autoplace_oobinfo.eccbytes);
	}

	oob.oobptr = oobreadbuf;
	oob.datptr = writebuf;

	/* Open the input file */
	if ((ifd = open(img, O_RDONLY)) == -1) {
		perror("open input file");
		goto restoreoob;
	}

	// get image length
	imglen = lseek(ifd, 0, SEEK_END);
	lseek (ifd, 0, SEEK_SET);

	pagelen = meminfo.writesize + ((writeoob == 1) ? meminfo.oobsize : 0);

	// Check, if file is pagealigned
	if ((!pad) && ((imglen % pagelen) != 0)) {
		fprintf (stderr, "Input file is not page aligned\n");
		goto closeall;
	}

	// Check, if length fits into device
	if ( ((imglen / pagelen) * meminfo.writesize) > (meminfo.size - mtdoffset)) {
		fprintf (stderr, "Image %d bytes, NAND page %d bytes, OOB area %u bytes, device size %lld bytes\n",
				imglen, pagelen, meminfo.writesize, meminfo.size);
		perror ("Input file does not fit into device");
		goto closeall;
	}

	/* Get data from input and write to the device */
	while (imglen && (mtdoffset < meminfo.size)) {
		// new eraseblock , check for bad block(s)
		// Stay in the loop to be sure if the mtdoffset changes because
		// of a bad block, that the next block that will be written to
		// is also checked. Thus avoiding errors if the block(s) after the
		// skipped block(s) is also bad (number of blocks depending on
		// the blockalign
		while (blockstart != (mtdoffset & (~meminfo.erasesize + 1))) {
			blockstart = mtdoffset & (~meminfo.erasesize + 1);
			offs = blockstart;
			baderaseblock = 0;
			i=0;
			if (!quiet)
				fprintf (stdout, "Writing data to block 0x%x\n", blockstart);

			/* Check all the blocks in an erase block for bad blocks */
			do {
				if ((ret = ioctl(fd, MEMGETBADBLOCK, &offs)) < 0) {
					perror("ioctl(MEMGETBADBLOCK)");
					goto closeall;
				}
				if (ret == 1) {
					baderaseblock = 1;
					if (!quiet)
						fprintf (stderr, "Bad block at 0x%llx, %u block(s) "
								"from 0x%x will be skipped\n",
							 offs, blockalign, blockstart);
				}

				if (baderaseblock) {
					mtdoffset = blockstart + meminfo.erasesize;
				}
				offs +=  meminfo.erasesize / blockalign ;
			} while ( offs < blockstart + meminfo.erasesize );

		}
      
		readlen = meminfo.writesize;
		if (pad && (imglen < readlen))
		{
			readlen = imglen;
			memset(writebuf + readlen, 0xff, meminfo.writesize - readlen);
		}

		/* Read Page Data from input file */
		if ((cnt = read(ifd, writebuf, readlen)) != readlen) {
			if (cnt == 0)	// EOF
				break;
			perror ("File I/O error 1 on input file");
			goto closeall;
		}

		/* Read OOB data from input file, exit on failure */
		if(writeoob) {
			if ((cnt = read(ifd, oobreadbuf, meminfo.oobsize)) != meminfo.oobsize) {
				perror ("File I/O error 2 on input file");
				goto closeall;
			}
		}
		oob.start = mtdoffset;

		// write a page include its oob to nand
		ioctl(fd, MEMWRITEPAGE, &oob);
		if(oob.datlength != meminfo.writesize){
			perror ("ioctl(MEMWRITEPAGE)");

			int rewind_blocks;
			off_t rewind_bytes;
			erase_info_t erase;

			/* Must rewind to blockstart if we can */
			rewind_blocks = (mtdoffset - blockstart) / meminfo.writesize; /* Not including the one we just attempted */
			rewind_bytes = (rewind_blocks * meminfo.writesize) + readlen;
			if (writeoob)
				rewind_bytes += (rewind_blocks + 1) * meminfo.oobsize;
			if (lseek(ifd, -rewind_bytes, SEEK_CUR) == -1) {
				perror("lseek");
				fprintf(stderr, "Failed to seek backwards to recover from write error\n");
				goto closeall;
			}
			erase.start = blockstart;
			erase.length = meminfo.erasesize;
			fprintf(stderr, "Erasing failed write from 0x%09llx-0x%09llx\n",
				erase.start, erase.start+erase.length-1);
			if (ioctl(fd, MEMERASE, &erase) != 0) {
				perror("MEMERASE");
				goto closeall;
			}

			if (markbad) {
				loff_mtd_t bad_addr = mtdoffset & (~(meminfo.erasesize / blockalign) + 1);
				fprintf(stderr, "Marking block at %09llx bad\n", (long long)bad_addr);
				if (ioctl(fd, MEMSETBADBLOCK, &bad_addr)) {
					perror("MEMSETBADBLOCK");
					/* But continue anyway */
				}
			}
			mtdoffset = blockstart + meminfo.erasesize;
			imglen += rewind_blocks * meminfo.writesize;

			continue;		
		}
		if(writeoob)
			imglen -= meminfo.oobsize;
		imglen -= readlen;
		mtdoffset += meminfo.writesize;
	}

closeall:
	close(ifd);

restoreoob:
	if (oobinfochanged == 1) {
		if (ioctl (fd, MEMSETOOBSEL, &old_oobinfo) != 0) {
			perror ("MEMSETOOBSEL");
			close (fd);
			exit (1);
		}
	}

	close(fd);

	if (imglen > 0) {
		perror ("Data was only partially written due to error\n");
		exit (1);
	}

	/* Return happy */
	return 0;
}
