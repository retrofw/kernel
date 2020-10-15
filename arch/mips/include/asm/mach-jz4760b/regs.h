/*
 * linux/include/asm-mips/mach-jz4760b/regs.h
 *
 * JZ4760B register definition.
 *
 * Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __JZ4760B_REGS_H__
#define __JZ4760B_REGS_H__


/*
 * Define the module base addresses
 */
/* AHB0 BUS Devices Base */
#define HARB0_BASE	0xB3000000
/* AHB1 BUS Devices Base */
#define HARB1_BASE	0xB3200000
#define	DMAGP0_BASE	0xB3210000
#define	DMAGP1_BASE	0xB3220000
#define	DMAGP2_BASE	0xB3230000
#define	DEBLK_BASE	0xB3270000
#define	IDCT_BASE	0xB3280000
#define	CABAC_BASE	0xB3290000
#define	TCSM0_BASE	0xB32B0000
#define	TCSM1_BASE	0xB32C0000
#define	SRAM_BASE	0xB32D0000
/* AHB2 BUS Devices Base */
#define HARB2_BASE	0xB3400000
#define UHC_BASE	0xB3430000
#define GPS_BASE	0xB3480000
#define ETHC_BASE	0xB34B0000
/* APB BUS Devices Base */
#define	PS2_BASE	0xB0060000


#endif /* __JZ4760B_REGS_H__ */
