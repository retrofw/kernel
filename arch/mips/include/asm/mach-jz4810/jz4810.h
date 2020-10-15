/*
 *  linux/include/asm-mips/mach-jz4760/jz4760.h
 *
 *  JZ4760 common definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4810_H__
#define __ASM_JZ4810_H__

#include <asm/mach-jz4810/regs.h>

#include <asm/mach-jz4810/jz4810misc.h>
#include <asm/mach-jz4810/jz4810gpio.h>
#include <asm/mach-jz4810/jz4810dmac.h>
#include <asm/mach-jz4810/jz4810intc.h>
#include <asm/mach-jz4810/jz4810aic.h>
#include <asm/mach-jz4810/jz4810bch.h>
#include <asm/mach-jz4810/jz4810bdma.h>
#include <asm/mach-jz4810/jz4810cim.h>
#include <asm/mach-jz4810/jz4810cpm.h>
#include <asm/mach-jz4810/jz4810ddrc.h>
#include <asm/mach-jz4810/jz4810emc.h>
#include <asm/mach-jz4810/jz4810i2c.h>
#include <asm/mach-jz4810/jz4810ipu.h>
#include <asm/mach-jz4810/jz4810lcdc.h>
#include <asm/mach-jz4810/jz4810mc.h>
#include <asm/mach-jz4810/jz4810mdma.h>
#include <asm/mach-jz4810/jz4810me.h>
#include <asm/mach-jz4810/jz4810msc.h>
#include <asm/mach-jz4810/jz4810nemc.h>
#include <asm/mach-jz4810/jz4810otg.h>
#include <asm/mach-jz4810/jz4810otp.h>
#include <asm/mach-jz4810/jz4810owi.h>
#include <asm/mach-jz4810/jz4810ost.h>
#include <asm/mach-jz4810/jz4810pcm.h>
#include <asm/mach-jz4810/jz4810rtc.h>
#include <asm/mach-jz4810/jz4810sadc.h>
#include <asm/mach-jz4810/jz4810scc.h>
#include <asm/mach-jz4810/jz4810ssi.h>
#include <asm/mach-jz4810/jz4810tcu.h>
#include <asm/mach-jz4810/jz4810tssi.h>
#include <asm/mach-jz4810/jz4810tve.h>
#include <asm/mach-jz4810/jz4810uart.h>
#include <asm/mach-jz4810/jz4810wdt.h>
#include <asm/mach-jz4810/jz4810aosd.h>

#include <asm/mach-jz4810/dma.h>
#include <asm/mach-jz4810/misc.h>

/*------------------------------------------------------------------
 * Platform definitions
 */

#define JZ_SOC_NAME "JZ4810"

#ifdef CONFIG_JZ4810_F4810
#include <asm/mach-jz4810/board-f4810.h>
#endif


/* Add other platform definition here ... */


/*------------------------------------------------------------------
 * Follows are related to platform definitions
 */

#include <asm/mach-jz4810/clock.h>
#include <asm/mach-jz4810/serial.h>

#endif /* __ASM_JZ4810_H__ */
