/*
 *  linux/include/asm-mips/jzsoc.h
 *
 *  Ingenic's JZXXXX SoC common include.
 *
 *  Copyright (C) 2006 - 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZSOC_H__
#define __ASM_JZSOC_H__

/*
 * SoC include
 */

#ifdef CONFIG_SOC_JZ4730
#include <asm/mach-jz4730/jz4730.h>
#endif

#ifdef CONFIG_SOC_JZ4740
#include <asm/mach-jz4740/jz4740.h>
#endif

#ifdef CONFIG_SOC_JZ4750
#include <asm/mach-jz4750/jz4750.h>
#endif

#ifdef CONFIG_SOC_JZ4750D
#include <asm/mach-jz4750d/jz4750d.h>
#endif

#ifdef CONFIG_SOC_JZ4750L
#include <asm/mach-jz4750l/jz4750l.h>
#endif

#ifdef CONFIG_SOC_JZ4760
#include <asm/mach-jz4760/jz4760.h>
#endif

#ifdef CONFIG_SOC_JZ4760B
#include <asm/mach-jz4760b/jz4760b.h>
#endif

#ifdef CONFIG_SOC_JZ4770
#include <asm/mach-jz4770/jz4770.h>
#endif

#ifdef CONFIG_SOC_JZ4810
#include <asm/mach-jz4810/jz4810.h>
#endif

#ifdef CONFIG_MFD_WM831X
#include <linux/mfd/wm831x/core.h>
#include <asm/jzpm/jz_wm831x.h>
#else
    #ifdef CONFIG_PMU_AXP192
    #include <linux/axp192.h>
    #include <asm/jzpm/jz_axp192.h>
    #elif defined(CONFIG_PMU_ACT8930_SUPPORT)
    #include <asm/jzpm/jz_act8930.h>
    #endif
#endif
/*
#else

#include <asm/jzpm/jz.h>
#endif
*/

/*
 * Generic I/O routines
 */
#define readb(addr)	(*(volatile unsigned char *)(addr))
#define readw(addr)	(*(volatile unsigned short *)(addr))
#define readl(addr)	(*(volatile unsigned int *)(addr))

#define writeb(b,addr)	((*(volatile unsigned char *)(addr)) = (b))
#define writew(b,addr)	((*(volatile unsigned short *)(addr)) = (b))
#define writel(b,addr)	((*(volatile unsigned int *)(addr)) = (b))

#endif /* __ASM_JZSOC_H__ */
