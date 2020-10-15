/*
 *  linux/include/asm-mips/mach-jz4740/jz4740.h
 *
 *  JZ4740 common definition.
 *
 *  Copyright (C) 2006 - 2007 Ingenic Semiconductor Inc.
 *
 *  Author: <lhhuang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4740_H__
#define __ASM_JZ4740_H__

#include <asm/mach-jz4740/regs.h>
#include <asm/mach-jz4740/ops.h>
#include <asm/mach-jz4740/dma.h>
#include <asm/mach-jz4740/misc.h>

/*------------------------------------------------------------------
 * Platform definitions
 */
#define JZ_SOC_NAME "JZ4740"

#ifdef CONFIG_JZ4740_PAVO
#include <asm/mach-jz4740/board-pavo.h>
#endif

#ifdef CONFIG_JZ4740_LEO
#include <asm/mach-jz4740/board-leo.h>
#endif

#ifdef CONFIG_JZ4740_LYRA
#include <asm/mach-jz4740/board-lyra.h>
#endif

#ifdef CONFIG_JZ4725_DIPPER
#include <asm/mach-jz4740/board-dipper.h>
#endif

#ifdef CONFIG_JZ4720_VIRGO
#include <asm/mach-jz4740/board-virgo.h>
#endif

/* Add other platform definition here ... */


/*------------------------------------------------------------------
 * Follows are related to platform definitions
 */

#include <asm/mach-jz4740/clock.h>
#include <asm/mach-jz4740/serial.h>

#endif /* __ASM_JZ4740_H__ */
