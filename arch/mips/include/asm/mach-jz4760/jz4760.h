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

#ifndef __ASM_JZ4760_H__
#define __ASM_JZ4760_H__

#include <asm/mach-jz4760/regs.h>

#include <asm/mach-jz4760/jz4760misc.h>
#include <asm/mach-jz4760/jz4760gpio.h>
#include <asm/mach-jz4760/jz4760dmac.h>
#include <asm/mach-jz4760/jz4760intc.h>
#include <asm/mach-jz4760/jz4760aic.h>
#include <asm/mach-jz4760/jz4760bch.h>
#include <asm/mach-jz4760/jz4760bdma.h>
#include <asm/mach-jz4760/jz4760cim.h>
#include <asm/mach-jz4760/jz4760cpm.h>
#include <asm/mach-jz4760/jz4760ddrc.h>
#include <asm/mach-jz4760/jz4760emc.h>
#include <asm/mach-jz4760/jz4760i2c.h>
#include <asm/mach-jz4760/jz4760ipu.h>
#include <asm/mach-jz4760/jz4760lcdc.h>
#include <asm/mach-jz4760/jz4760mc.h>
#include <asm/mach-jz4760/jz4760mdma.h>
#include <asm/mach-jz4760/jz4760me.h>
#include <asm/mach-jz4760/jz4760msc.h>
#include <asm/mach-jz4760/jz4760nemc.h>
#include <asm/mach-jz4760/jz4760otg.h>
#include <asm/mach-jz4760/jz4760otp.h>
#include <asm/mach-jz4760/jz4760owi.h>
#include <asm/mach-jz4760/jz4760pcm.h>
#include <asm/mach-jz4760/jz4760rtc.h>
#include <asm/mach-jz4760/jz4760sadc.h>
#include <asm/mach-jz4760/jz4760scc.h>
#include <asm/mach-jz4760/jz4760ssi.h>
#include <asm/mach-jz4760/jz4760tcu.h>
#include <asm/mach-jz4760/jz4760tssi.h>
#include <asm/mach-jz4760/jz4760tve.h>
#include <asm/mach-jz4760/jz4760uart.h>
#include <asm/mach-jz4760/jz4760wdt.h>
#include <asm/mach-jz4760/jz4760ost.h>

#include <asm/mach-jz4760/dma.h>
#include <asm/mach-jz4760/misc.h>
#include <asm/mach-jz4760/platform.h>

/*------------------------------------------------------------------
 * Platform definitions
 */

#define JZ_SOC_NAME "JZ4760"

#ifdef CONFIG_JZ4750_FUWA
#include <asm/mach-jz4750/board-fuwa.h>
#endif

#ifdef CONFIG_JZ4760_F4760
#include <asm/mach-jz4760/board-f4760.h>
#endif

#ifdef CONFIG_JZ4760_CYGNUS
#include <asm/mach-jz4760/board-cygnus.h>
#endif

#ifdef CONFIG_JZ4760_LEPUS
#include <asm/mach-jz4760/board-lepus.h>
#endif

#ifdef CONFIG_JZ4760_ALTAIR
#include <asm/mach-jz4760/board-altair.h>
#endif

#ifdef CONFIG_JZ4760_HTB80
#include <asm/mach-jz4760/jz4760epdc.h>
#include <asm/mach-jz4760/board-htb80.h>
#endif

/* Add other platform definition here ... */


/*------------------------------------------------------------------
 * Follows are related to platform definitions
 */

//#include <asm/mach-jz4760/clock.h>
#include <asm/mach-jz4760/serial.h>
#include <asm/mach-jz4760/spi.h>

#endif /* __ASM_JZ4760_H__ */
