/*
 *  linux/include/asm-mips/mach-jz4770/jz4770.h
 *
 *  JZ4770 common definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4770_H__
#define __ASM_JZ4770_H__

#include <asm/mach-jz4770/regs.h>

#include <asm/mach-jz4770/jz4770misc.h>
#include <asm/mach-jz4770/jz4770gpio.h>
#include <asm/mach-jz4770/jz4770dmac.h>
#include <asm/mach-jz4770/jz4770intc.h>
#include <asm/mach-jz4770/jz4770aic.h>
#include <asm/mach-jz4770/jz4770bch.h>
#include <asm/mach-jz4770/jz4770bdma.h>
#include <asm/mach-jz4770/jz4770cim.h>
#include <asm/mach-jz4770/jz4770cpm.h>
#include <asm/mach-jz4770/jz4770ddrc.h>
#include <asm/mach-jz4770/jz4770emc.h>
#include <asm/mach-jz4770/jz4770i2c.h>
#include <asm/mach-jz4770/jz4770ipu.h>
#include <asm/mach-jz4770/jz4770lcdc.h>
#include <asm/mach-jz4770/jz4770mc.h>
#include <asm/mach-jz4770/jz4770mdma.h>
#include <asm/mach-jz4770/jz4770me.h>
#include <asm/mach-jz4770/jz4770msc.h>
#include <asm/mach-jz4770/jz4770nemc.h>
#include <asm/mach-jz4770/jz4770otg.h>
#include <asm/mach-jz4770/jz4770otp.h>
#include <asm/mach-jz4770/jz4770owi.h>
#include <asm/mach-jz4770/jz4770ost.h>
#include <asm/mach-jz4770/jz4770pcm.h>
#include <asm/mach-jz4770/jz4770rtc.h>
#include <asm/mach-jz4770/jz4770sadc.h>
#include <asm/mach-jz4770/jz4770scc.h>
#include <asm/mach-jz4770/jz4770ssi.h>
#include <asm/mach-jz4770/jz4770tcu.h>
#include <asm/mach-jz4770/jz4770tssi.h>
#include <asm/mach-jz4770/jz4770efuse.h>
#include <asm/mach-jz4770/jz4770tve.h>
#include <asm/mach-jz4770/jz4770uart.h>
#include <asm/mach-jz4770/jz4770wdt.h>
#include <asm/mach-jz4770/jz4770aosd.h>
#include <asm/mach-jz4770/jz4770lvds.h>

#include <asm/mach-jz4770/dma.h>
#include <asm/mach-jz4770/misc.h>
#include <asm/mach-jz4770/platform.h>

/*------------------------------------------------------------------
 * Platform definitions
 */

#define JZ_SOC_NAME "JZ4770"

#ifdef CONFIG_JZ4770_F4770
#include <asm/mach-jz4770/board-f4770.h>
#endif

#ifdef CONFIG_JZ4770_PISCES
#include <asm/mach-jz4770/board-pisces.h>
#endif

#ifdef CONFIG_JZ4770_TJ70
#include <asm/mach-jz4770/board-tj70.h>
#endif

/* Add other platform definition here ... */


/*------------------------------------------------------------------
 * Follows are related to platform definitions
 */

#include <asm/mach-jz4770/serial.h>

#endif /* __ASM_JZ4770_H__ */
