#ifndef _JZ_KBC_H
#define _JZ_KBC_H

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */


/*
 * Standard commands.
 */

#define KBC_CMD_CTL_RCTR	0x0120
#define KBC_CMD_CTL_WCTR	0x1060
#define KBC_CMD_CTL_TEST	0x01aa
#define KBC_CMD_CTL_REPORT	0x01e0
#define KBC_CMD_CTL_RINPUT	0x01c0

#define KBC_CMD_KBD_DISABLE	0x00ad
#define KBC_CMD_KBD_ENABLE	0x00ae
#define KBC_CMD_KBD_TEST	0x01ab
#define KBC_CMD_KBD_LOOP	0x11d2

#define KBC_CMD_AUX_DISABLE	0x00a7
#define KBC_CMD_AUX_ENABLE	0x00a8
#define KBC_CMD_AUX_TEST	0x01a9
#define KBC_CMD_AUX_SEND	0x10d4
#define KBC_CMD_AUX_LOOP	0x11d3

int jz_kbc_command(unsigned char *param, int command);

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/*
 * Names.
 */

#define KBC_KBD_PHYS_DESC "isa0060/serio0"
#define KBC_AUX_PHYS_DESC "isa0060/serio1"
#define KBC_MUX_PHYS_DESC "isa0060/serio%d"

/*
 * REGs.
 */
#define KBC_DATA_REG	(PS2_BASE + 0x60)
#define KBC_COMMAND_REG	(PS2_BASE + 0x64)

#define __gpio_as_bkc() 				\
do {						\
	SETREG32(GPIO_PXINTC(3), 0xf << 4);	\
	SETREG32(GPIO_PXMASKC(3), 0xf << 4);	\
	SETREG32(GPIO_PXPAT0C(3), 0xf << 4);	\
	SETREG32(GPIO_PXPAT1C(3), 0xf << 4);	\
} while (0)

#define KBC_IRQ_ENABLE()			\
	SETREG32(INTC_ICMSR(IRQ_KBC / 32), 1 << (IRQ_KBC % 32));

#define KBC_CLK_ENABLE()			\
	CLRREG32(CPM_CLKGR0, CLKGR0_KBC);
/*
 * Register numbers.
 */

static inline int jz_kbc_read_data(void)
{
	return readb(KBC_DATA_REG);
}

static inline int jz_kbc_read_status(void)
{
	return readb(KBC_COMMAND_REG);
}

static inline void jz_kbc_write_data(int val)
{
	writeb(val, KBC_DATA_REG);
}

static inline void jz_kbc_write_command(int val)
{
	writeb(val, KBC_COMMAND_REG);
}

static int jz_kbc_platform_init(void)
{
	__gpio_as_bkc();	
	KBC_IRQ_ENABLE();		
	KBC_CLK_ENABLE();

	return 0;
}

#define KBC_CTL_TIMEOUT	10000

/*
 * Status register bits.
 */

#define KBC_STR_PARITY	0x80
#define KBC_STR_TIMEOUT	0x40
#define KBC_STR_AUXDATA	0x20
#define KBC_STR_KEYLOCK	0x10
#define KBC_STR_CMDDAT	0x08
#define KBC_STR_MUXERR	0x04
#define KBC_STR_IBF		0x02
#define	KBC_STR_OBF		0x01

/*
 * Control register bits.
 */

#define KBC_CTR_KBDINT	0x01
#define KBC_CTR_AUXINT	0x02
#define KBC_CTR_SYF	0x04	
//#define KBC_CTR_IGNKEYLOCK	0x08	Current KBC doesn't support password function.
#define KBC_CTR_KBDDIS	0x10
#define KBC_CTR_AUXDIS	0x20
#define KBC_CTR_XLATE		0x40

/*
 * Return codes.
 */

#define KBC_RET_CTL_TEST	0x55

/*
 * Expected maximum internal jz_kbc buffer size. This is used for flushing
 * the jz_kbc buffers.
 */

#define KBC_BUFFER_SIZE	1

/*
 * Number of AUX ports on controllers supporting active multiplexing
 * specification
 */

//#define KBC_NUM_MUX_PORTS	4

/*
 * Debug.
 */

#ifdef DEBUG
static unsigned long jz_kbc_start_time;
#define dbg_init() do { jz_kbc_start_time = jiffies; } while (0)
#define dbg(format, arg...) 							\
	do { 									\
		if (jz_kbc_debug)						\
			printk(KERN_INFO __FILE__ ": " format " [%d]\n" ,	\
	 			## arg, (int) (jiffies - jz_kbc_start_time));	\
	} while (0)
#else
#define dbg_init() do { } while (0)
#define dbg(format, arg...) do {} while (0)
#endif

#endif /* _JZ_KBC_H */
