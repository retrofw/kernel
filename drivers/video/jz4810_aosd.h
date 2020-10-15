/*
 * linux/drivers/video/jz4810_lcd.h -- Ingenic Jz4810 On-Chip LCD frame buffer device
 *
 * Copyright (C) 2005-2008, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __JZ4810_AOSD_H__
#define __JZ4810_AOSD_H__

struct jz4810_aosd_info {
	unsigned long addr0;
	unsigned long addr1;
	unsigned long addr2;
	unsigned long addr3;
	unsigned long waddr;

	unsigned long smem_start;
	char __iomem *addr0_base;
	__u32         addr_len;

	__u32         alpha_value;
	__u32         frmlv;
	__u32         order;
	__u32         format_mode;
	__u32         alpha_mode;
	__u32         height;
	__u32         width;
	__u32         without_alpha;
	__u32         compress_done;
	__u32         bpp;
	__u32         buf;
}; 

#define ALPHA_OSD_START         0x46a8
#define ALPHA_OSD_GET_INFO      0x46a9
#define ALPHA_OSD_SET_MODE      0x46aa
#define COMPRESS_START          0x46ab
#define COMPRESS_GET_INFO      0x46ac
#define COMPRESS_SET_MODE      0x46ad
#define ALPHA_OSD_PRINT      0x46ae
 

#endif /* __JZ4810_AOSD_H__ */
