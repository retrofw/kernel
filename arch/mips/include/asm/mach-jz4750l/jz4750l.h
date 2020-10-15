/*
 *  linux/include/asm-mips/mach-jz4750l/jz4750l.h
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

/*------------------------------------------------------------------
 * Device Register definitions
 */
#include <asm/mach-jz4750l/regs.h>

#include <asm/mach-jz4750l/jz4750lmisc.h>
#include <asm/mach-jz4750l/jz4750laic.h>
#include <asm/mach-jz4750l/jz4750lbch.h>
#include <asm/mach-jz4750l/jz4750lcim.h>
#include <asm/mach-jz4750l/jz4750lcpm.h>
#include <asm/mach-jz4750l/jz4750ldmac.h>
#include <asm/mach-jz4750l/jz4750lemc.h>
#include <asm/mach-jz4750l/jz4750lgpio.h>
#include <asm/mach-jz4750l/jz4750li2c.h>
#include <asm/mach-jz4750l/jz4750licdc.h>
#include <asm/mach-jz4750l/jz4750lintc.h>
#include <asm/mach-jz4750l/jz4750lipu.h>
#include <asm/mach-jz4750l/jz4750llcdc.h>
#include <asm/mach-jz4750l/jz4750lmc.h>
#include <asm/mach-jz4750l/jz4750lme.h>
#include <asm/mach-jz4750l/jz4750lmsc.h>
#include <asm/mach-jz4750l/jz4750lotg.h>
#include <asm/mach-jz4750l/jz4750lotp.h>
#include <asm/mach-jz4750l/jz4750lowi.h>
#include <asm/mach-jz4750l/jz4750lrtc.h>
#include <asm/mach-jz4750l/jz4750lsadc.h>
#include <asm/mach-jz4750l/jz4750lssi.h>
#include <asm/mach-jz4750l/jz4750ltcu.h>
#include <asm/mach-jz4750l/jz4750ltssi.h>
#include <asm/mach-jz4750l/jz4750luart.h>
#include <asm/mach-jz4750l/jz4750lwdt.h>

#include <asm/mach-jz4750l/dma.h>
#include <asm/mach-jz4750l/platform.h>
#include <asm/mach-jz4750l/misc.h>

/*------------------------------------------------------------------
 * Platform definitions
 */
#define JZ_SOC_NAME "JZ4750L"

#ifdef CONFIG_JZ4750L_F4750L
#include <asm/mach-jz4750l/board-f4750l.h>
#endif

#ifdef CONFIG_JZ4750L_SPWAM
#include <asm/mach-jz4750l/board-spwam.h>
#endif
#ifdef CONFIG_JZ4750L_TAURUS
#include <asm/mach-jz4750l/board-taurus.h>
#endif

/* Add other platform definition here ... */


/*------------------------------------------------------------------
 * Follows are related to platform definitions
 */

#include <asm/mach-jz4750l/clock.h>
#include <asm/mach-jz4750l/serial.h>
#include <asm/mach-jz4750l/spi.h>

#endif /* __ASM_JZ4750_H__ */
