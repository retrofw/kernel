/*
 * linux/include/asm-mips/mach-jz4810/jz4810owi.h
 *
 * JZ4810 OWI register definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ4770OWI_H__
#define __JZ4770OWI_H__


#define	OWI_BASE	0xB0072000

/*************************************************************************
 * OWI (One-wire Bus Controller )
 *************************************************************************/
#define OWI_CFG (OWI_BASE + 0x00) /* OWI Configure Register */
#define OWI_CTL (OWI_BASE + 0x04) /* OWI Control Register */
#define OWI_STS (OWI_BASE + 0x08) /* OWI Status Register */
#define OWI_DAT (OWI_BASE + 0x0c) /* OWI Data Register */
#define OWI_DIV (OWI_BASE + 0x10) /* OWI Clock Divide Register */

#define REG_OWI_CFG  REG8(OWI_CFG)
#define REG_OWI_CTL  REG8(OWI_CTL)
#define REG_OWI_STS  REG8(OWI_STS)
#define REG_OWI_DAT  REG8(OWI_DAT)
#define REG_OWI_DIV  REG8(OWI_DIV)

/* OWI Configure Register */
#define OWI_CFG_MODE      (1 << 7) /*  0: Regular speed mode  1: Overdrive  speed mode */
#define OWI_CFG_RDDATA    (1 << 6) /* 1: receive data from one-wire bus and stored in OWDAT*/
#define OWI_CFG_WRDATA    (1 << 5) /* 1: transmit the data in OWDAT */
#define OWI_CFG_RDST      (1 << 4) /* 1: was sampled during a read */
#define OWI_CFG_WR1RD     (1 << 3) /* 1: generate write 1 sequence on line */
#define OWI_CFG_WR0       (1 << 2) /* 1: generate write 0 sequence on line */
#define OWI_CFG_RST       (1 << 1) /* 1: generate reset pulse and sample slaves presence pulse*/
#define OWI_CFG_ENA       (1 << 0) /* 1: enable the OWI operation */

/* OWI Control Register */
#define OWI_CTL_EBYTE     (1 << 2) /* enable byte write/read interrupt */
#define OWI_CTL_EBIT      (1 << 1) /* enable bit write/read interrupt */
#define OWI_CTL_ERST      (1 << 0) /* enable reset sequence finished interrupt */

/* OWI Status Register */
#define OWI_STS_PST       (1 << 7) /* 1: one-wire bus has device on it */
#define OWI_STS_BYTE_RDY  (1 << 2) /* 1: have received or transmitted a data */
#define OWI_STS_BIT_RDY   (1 << 1) /* 1: have received or transmitted a bit */
#define OWI_STS_PST_RDY   (1 << 0) /* 1: have finished a reset pulse */

/* OWI Clock Divide Register */
#define OWI_DIV_CLKDIV_BIT  5


#ifndef __MIPS_ASSEMBLER

/***************************************************************************
 * OWI (one-wire bus)  ops
 ***************************************************************************/

/* OW control register ops */
#define __owi_enable_all_interrupts()      ( REG_OWI_CTL = (OWI_CTL_EBYTE | OWI_CTL_EBIT | OWI_CTL_ERST) )
#define __owi_disable_all_interrupts()     ( REG_OWI_CTL = 0 )

#define __owi_enable_byte_interrupt()      ( REG_OWI_CTL |= OWI_CTL_EBYTE )
#define __owi_disable_byte_interrupt()     ( REG_OWI_CTL &= ~OWI_CTL_EBYTE )
#define __owi_enable_bit_interrupt()       ( REG_OWI_CTL |= OWI_CTL_EBIT )
#define __owi_disable_bit_interrupt()      ( REG_OWI_CTL &= ~OWI_CTL_EBIT )
#define __owi_enable_rst_interrupt()       ( REG_OWI_CTL |= OWI_CTL_ERST ) 
#define __owi_disable_rst_interrupt()      ( REG_OWI_CTL &=~OWI_CTL_ERST )

/* OW configure register ops */
#define __owi_select_regular_mode()        ( REG_OWI_CFG &= ~OWI_CFG_MODE )
#define __owi_select_overdrive_mode()      ( REG_OWI_CFG |= OWI_CFG_MODE )

#define __owi_set_rddata()  ( REG_OWI_CFG |= OWI_CFG_RDDATA )
#define __owi_clr_rddata()  ( REG_OWI_CFG &= ~OWI_CFG_RDDATA )
#define __owi_get_rddata()  ( REG_OWI_CFG & OWI_CFG_RDDATA )

#define __owi_set_wrdata()  ( REG_OWI_CFG |= OWI_CFG_WRDATA )
#define __owi_clr_wrdata()  ( REG_OWI_CFG &= ~OWI_CFG_WRDATA )
#define __owi_get_wrdata()  ( REG_OWI_CFG & OWI_CFG_WRDATA )

#define __owi_get_rdst()    ( REG_OWI_CFG & OWI_CFG_RDST )

#define __owi_set_wr1rd()   ( REG_OWI_CFG |= OWI_CFG_WR1RD )
#define __owi_clr_wr1rd()   ( REG_OWI_CFG &= ~OWI_CFG_WR1RD )
#define __owi_get_wr1rd()   ( REG_OWI_CFG & OWI_CFG_WR1RD )

#define __owi_set_wr0()     ( REG_OWI_CFG |= OWI_CFG_WR0 )
#define __owi_clr_wr0()     ( REG_OWI_CFG &= ~OWI_CFG_WR0 )
#define __owi_get_wr0()     ( REG_OWI_CFG & OWI_CFG_WR0 )

#define __owi_set_rst()     ( REG_OWI_CFG |= OWI_CFG_RST )
#define __owi_clr_rst()     ( REG_OWI_CFG &= ~OWI_CFG_RST )
#define __owi_get_rst()     ( REG_OWI_CFG & OWI_CFG_RST )

#define __owi_enable_ow_ops()  ( REG_OWI_CFG |= OWI_CFG_ENA )
#define __owi_disable_ow_ops() ( REG_OWI_CFG &= ~OWI_CFG_ENA )
#define __owi_get_enable()     ( REG_OWI_CFG & OWI_CFG_ENA )

#define __owi_wait_ops_rdy()                \
	do {				    \
		while(__owi_get_enable());  \
		udelay(1);		    \
	} while(0);

/* OW status register ops */
#define __owi_clr_sts()           ( REG_OWI_STS = 0 )
#define __owi_get_sts_pst()       ( REG_OWI_STS & OWI_STS_PST )
#define __owi_get_sts_byte_rdy()  ( REG_OWI_STS & OWI_STS_BYTE_RDY )
#define __owi_get_sts_bit_rdy()   ( REG_OWI_STS & OWI_STS_BIT_RDY )
#define __owi_get_sts_pst_rdy()   ( REG_OWI_STS & OWI_STS_PST_RDY )



#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4770OWI_H__ */

