/*
 *  linux/include/asm-mips/mach-jz4750/jz4750.h
 *
 *  JZ4750 common definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4750_H__
#define __ASM_JZ4750_H__

#include <asm/mach-jz4750/regs.h>
#include <asm/mach-jz4750/ops.h>
#include <asm/mach-jz4750/dma.h>
#include <asm/mach-jz4750/misc.h>
#include <asm/mach-jz4750/platform.h>

/*------------------------------------------------------------------
 * Platform definitions
 */
#define JZ_SOC_NAME "JZ4750"

#ifdef CONFIG_JZ4750_FUWA
#include <asm/mach-jz4750/board-fuwa.h>
#endif

#ifdef CONFIG_JZ4750_APUS
#include <asm/mach-jz4750/board-apus.h>
#endif

/* Add other platform definition here ... */


/*------------------------------------------------------------------
 * Follows are related to platform definitions
 */

#include <asm/mach-jz4750/clock.h>
#include <asm/mach-jz4750/serial.h>
#include <asm/mach-jz4750/i2c.h>
#include <asm/mach-jz4750/spi.h>

#endif /* __ASM_JZ4750_H__ */
