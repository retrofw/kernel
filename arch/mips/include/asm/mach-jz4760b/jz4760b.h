/*
 *  linux/include/asm-mips/mach-jz4760b/jz4760b.h
 *
 *  JZ4760B common definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4760B_H__
#define __ASM_JZ4760B_H__

#include <asm/mach-jz4760b/regs.h>

#include <asm/mach-jz4760b/jz4760bmisc.h>
#include <asm/mach-jz4760b/jz4760bgpio.h>
#include <asm/mach-jz4760b/jz4760bdmac.h>
#include <asm/mach-jz4760b/jz4760bintc.h>
#include <asm/mach-jz4760b/jz4760baic.h>
#include <asm/mach-jz4760b/jz4760bbch.h>
#include <asm/mach-jz4760b/jz4760bbdma.h>
#include <asm/mach-jz4760b/jz4760bcim.h>
#include <asm/mach-jz4760b/jz4760bcpm.h>
#include <asm/mach-jz4760b/jz4760bddrc.h>
#include <asm/mach-jz4760b/jz4760bemc.h>
#include <asm/mach-jz4760b/jz4760bi2c.h>
#include <asm/mach-jz4760b/jz4760bipu.h>
#include <asm/mach-jz4760b/jz4760blcdc.h>
#include <asm/mach-jz4760b/jz4760bmc.h>
#include <asm/mach-jz4760b/jz4760bmdma.h>
#include <asm/mach-jz4760b/jz4760bme.h>
#include <asm/mach-jz4760b/jz4760bmsc.h>
#include <asm/mach-jz4760b/jz4760bnemc.h>
#include <asm/mach-jz4760b/jz4760botg.h>
#include <asm/mach-jz4760b/jz4760botp.h>
#include <asm/mach-jz4760b/jz4760bowi.h>
#include <asm/mach-jz4760b/jz4760bpcm.h>
#include <asm/mach-jz4760b/jz4760brtc.h>
#include <asm/mach-jz4760b/jz4760bsadc.h>
#include <asm/mach-jz4760b/jz4760bscc.h>
#include <asm/mach-jz4760b/jz4760bssi.h>
#include <asm/mach-jz4760b/jz4760btcu.h>
#include <asm/mach-jz4760b/jz4760btssi.h>
#include <asm/mach-jz4760b/jz4760btve.h>
#include <asm/mach-jz4760b/jz4760buart.h>
#include <asm/mach-jz4760b/jz4760bwdt.h>
#include <asm/mach-jz4760b/jz4760bost.h>

#include <asm/mach-jz4760b/dma.h>
#include <asm/mach-jz4760b/misc.h>
#include <asm/mach-jz4760b/platform.h>

/*------------------------------------------------------------------
 * Platform definitions
 */

#define JZ_SOC_NAME "JZ4760B"

#ifdef CONFIG_JZ4750_FUWA
#include <asm/mach-jz4750/board-fuwa.h>
#endif

#ifdef CONFIG_JZ4760B_F4760b
#include <asm/mach-jz4760b/board-f4760b.h>
#endif

#ifdef CONFIG_JZ4760B_CYGNUS
#include <asm/mach-jz4760b/board-cygnus.h>
#endif

#ifdef CONFIG_JZ4760B_LEPUS
#include <asm/mach-jz4760b/board-lepus.h>
#endif



#ifdef CONFIG_JZ4760B_ALTAIR
#include <asm/mach-jz4760b/board-altair.h>
#endif

/* Add other platform definition here ... */


/*------------------------------------------------------------------
 * Follows are related to platform definitions
 */

//#include <asm/mach-jz4760b/clock.h>
#include <asm/mach-jz4760b/serial.h>
#include <asm/mach-jz4760b/spi.h>

#endif /* __ASM_JZ4760B_H__ */
