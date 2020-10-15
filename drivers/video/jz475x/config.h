/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * SOC & Driver Config.
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

#ifndef __JZ_FB_CONFIG_H__
#define __JZ_FB_CONFIG_H__

#ifdef __KERNEL__
#include <linux/autoconf.h>
#else
#include "userspace-config.h"
#endif

#ifdef __KERNEL__
/* DEBUG. */
#define JZ_FB_DEBUG		0
#define JZ_FB_DEBUG_DUMP	0	/* Dump Structures. */
#endif

/* --------------- CONFIG FOR JZ4750 ---------------- */
#ifdef CONFIG_SOC_JZ4750

/* Limits. */
#define JZ_FB_NR_MAX_DMA_DESC	6
#define JZ_FB_NR_MAX_FB_MEM	3
#define JZ_FB_NR_MAX_FG 	2

/* REVIST: ALIGN. */
#define JZ_FB_DMA_ALIGN		4
#define JZ_FB_WIN_POS_ALIGN	2
#define JZ_FB_WIN_SIZE_ALIGN	2

/* Config. */
#define JZ_FB_8WORD_DMA_DESC	1

#define HAVE_TVE		1

#endif

/* --------------- CONFIG FOR JZ4755 ---------------- */
#ifdef CONFIG_SOC_JZ4750D

/* Limits. */
#define JZ_FB_NR_MAX_DMA_DESC	6
#define JZ_FB_NR_MAX_FB_MEM	3
#define JZ_FB_NR_MAX_FG 	2

/* REVIST: ALIGN. */
#define JZ_FB_DMA_ALIGN		4
#define JZ_FB_WIN_POS_ALIGN	2
#define JZ_FB_WIN_SIZE_ALIGN	2

/* Config. */
#define JZ_FB_8WORD_DMA_DESC	1

#define HAVE_TVE		1
#endif

#ifdef __KERNEL__

/* ------------- JZ_FB_DEBUG ----------- */
#if JZ_FB_DEBUG
#define D(fmt, args...) \
	printk(KERN_ERR "%s(): LINE: %d "fmt"\n", __func__, __LINE__, ##args)
#else
#define D(fmt, args...)
#endif

/* ------------- JZ_FB_DEBUG_DUMP ------------- */
#if JZ_FB_DEBUG_DUMP
#define DUMP(fmt, args...) \
	printk(KERN_ERR "%s(): "fmt"\n", __func__, ##args)
#else
#define DUMP(fmt, args...)
#endif

#define E(fmt, args...) \
	printk(KERN_ERR "%s(): "fmt"\n", __func__, ##args)

#define I(fmt, args...) \
	printk(KERN_INFO JZ_SOC_NAME": "DRV_NAME": "fmt"\n", ##args) 

#endif	/* Define __KERNEL__ */

#endif
