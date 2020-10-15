/*
 * linux/include/asm-mips/mach-jz4750l/jz4750ltssi.h
 *
 * JZ4750L TSSI register definition
 *
 * Copyright (C) 2008-2014 Ingenic Semiconductor Co., Ltd.
 */
#ifndef __JZ4750LTSSI_H__
#define __JZ4750LTSSI_H__

/* Define the module base addresses */
#define TSSI_BASE	0xB0073000

/*************************************************************************
 * TSSI MPEG 2-TS slave interface
 *************************************************************************/
#define TSSI_ENA       ( TSSI_BASE + 0x00 )   /* TSSI enable register */
#define TSSI_CFG       ( TSSI_BASE + 0x04 )   /* TSSI configure register */
#define TSSI_CTRL      ( TSSI_BASE + 0x08 )   /* TSSI control register */
#define TSSI_STAT      ( TSSI_BASE + 0x0c )   /* TSSI state register */
#define TSSI_FIFO      ( TSSI_BASE + 0x10 )   /* TSSI FIFO register */
#define TSSI_PEN       ( TSSI_BASE + 0x14 )   /* TSSI PID enable register */
#define TSSI_PID(n)    ( TSSI_BASE + 0x20 + 4*(n) )   /* TSSI PID filter register */
#define TSSI_PID0      ( TSSI_BASE + 0x20 )
#define TSSI_PID1      ( TSSI_BASE + 0x24 )
#define TSSI_PID2      ( TSSI_BASE + 0x28 )
#define TSSI_PID3      ( TSSI_BASE + 0x2c )
#define TSSI_PID4      ( TSSI_BASE + 0x30 )
#define TSSI_PID5      ( TSSI_BASE + 0x34 )
#define TSSI_PID6      ( TSSI_BASE + 0x38 )
#define TSSI_PID7      ( TSSI_BASE + 0x3c )
#define TSSI_PID_MAX   8	/* max PID: 7 */
 
#define REG_TSSI_ENA       REG8( TSSI_ENA )
#define REG_TSSI_CFG       REG16( TSSI_CFG )
#define REG_TSSI_CTRL      REG8( TSSI_CTRL )
#define REG_TSSI_STAT      REG8( TSSI_STAT )
#define REG_TSSI_FIFO      REG32( TSSI_FIFO )
#define REG_TSSI_PEN       REG32( TSSI_PEN )
#define REG_TSSI_PID(n)    REG32( TSSI_PID(n) )
#define REG_TSSI_PID0      REG32( TSSI_PID0 )
#define REG_TSSI_PID1      REG32( TSSI_PID1 )
#define REG_TSSI_PID2      REG32( TSSI_PID2 )
#define REG_TSSI_PID3      REG32( TSSI_PID3 )
#define REG_TSSI_PID4      REG32( TSSI_PID4 )
#define REG_TSSI_PID5      REG32( TSSI_PID5 )
#define REG_TSSI_PID6      REG32( TSSI_PID6 )
#define REG_TSSI_PID7      REG32( TSSI_PID7 )

/* TSSI enable register */
#define TSSI_ENA_SFT_RST 	( 1 << 7 )      /* soft reset bit */
#define TSSI_ENA_PID_EN 	( 1 << 2 )      /* soft filtering function enable bit */
#define TSSI_ENA_DMA_EN 	( 1 << 1 )      /* DMA enable bit */
#define TSSI_ENA_ENA 		( 1 << 0 )      /* TSSI enable bit */

/* TSSI configure register */
#define TSSI_CFG_TRIG_BIT 	14 /* fifo trig number */
#define TSSI_CFG_TRIG_MASK 	( 0x3 << TSSI_CFG_TRIG_BIT)
#define TSSI_CFG_TRIG_4 	( 0 << TSSI_CFG_TRIG_BIT)
#define TSSI_CFG_TRIG_8 	( 1 << TSSI_CFG_TRIG_BIT)
#define TSSI_CFG_TRIG_16 	( 2 << TSSI_CFG_TRIG_BIT)
#define TSSI_CFG_END_WD 	( 1 << 9 )      /* order of data in word */
#define TSSI_CFG_END_BT 	( 1 << 8 )      /* order of data in byte */
#define TSSI_CFG_TSDI_H 	( 1 << 7 )      /* data pin polarity */
#define TSSI_CFG_USE_0 		( 1 << 6 )      /* serial mode data pin select */
#define TSSI_CFG_USE_TSDI0 	( 0 << 6 )      /* TSDI0 as serial mode data pin */
#define TSSI_CFG_USE_TSDI7 	( 1 << 6 )      /* TSDI7 as serial mode data pin */
#define TSSI_CFG_TSCLK_CH 	( 1 << 5 )      /* clk channel select */
#define TSSI_CFG_PARAL 		( 1 << 4 )      /* mode select */
#define TSSI_CFG_PARAL_MODE 	( 1 << 4 )      /* parallel select */
#define TSSI_CFG_SERIAL_MODE 	( 0 << 4 )      /* serial select */
#define TSSI_CFG_TSCLK_P 	( 1 << 3 )      /* clk edge select */
#define TSSI_CFG_TSFRM_H 	( 1 << 2 )      /* TSFRM polarity select */
#define TSSI_CFG_TSSTR_H 	( 1 << 1 )      /* TSSTR polarity select */
#define TSSI_CFG_TSFAIL_H 	( 1 << 0 )      /* TSFAIL polarity select */

/* TSSI control register */
#define TSSI_CTRL_OVRNM 	( 1 << 1 )      /* FIFO overrun interrupt mask bit */
#define TSSI_CTRL_TRIGM 	( 1 << 0 )      /* FIFO trigger interrupt mask bit */

/* TSSI state register */
#define TSSI_STAT_OVRN 		( 1 << 1 )      /* FIFO overrun interrupt flag bit */
#define TSSI_STAT_TRIG 		( 1 << 0 )      /* FIFO trigger interrupt flag bit */

/* TSSI PID enable register */
#define TSSI_PEN_EN00 	( 1 << 0 )      /* enable PID n */
#define TSSI_PEN_EN10 	( 1 << 1 )      
#define TSSI_PEN_EN20 	( 1 << 2 )      
#define TSSI_PEN_EN30 	( 1 << 3 )      
#define TSSI_PEN_EN40 	( 1 << 4 )      
#define TSSI_PEN_EN50 	( 1 << 5 )      
#define TSSI_PEN_EN60 	( 1 << 6 )      
#define TSSI_PEN_EN70 	( 1 << 7 )      
#define TSSI_PEN_EN01 	( 1 << 16 )      
#define TSSI_PEN_EN11 	( 1 << 17 )      
#define TSSI_PEN_EN21 	( 1 << 18 )     
#define TSSI_PEN_EN31 	( 1 << 19 )      
#define TSSI_PEN_EN41 	( 1 << 20 )     
#define TSSI_PEN_EN51 	( 1 << 21 )      
#define TSSI_PEN_EN61 	( 1 << 22 )     
#define TSSI_PEN_EN71 	( 1 << 23 )      
#define TSSI_PEN_PID0 	( 1 << 31 ) /* PID filter enable PID0 */

/* TSSI PID Filter Registers */
#define TSSI_PID_PID1_BIT 	16
#define TSSI_PID_PID1_MASK 	(0x1FFF<<TSSI_PID_PID1_BIT)
#define TSSI_PID_PID0_BIT 	0
#define TSSI_PID_PID0_MASK 	(0x1FFF<<TSSI_PID_PID0_BIT)

/*
 * TSSI MPEG 2-TS slave interface Operations
 */
#ifndef __MIPS_ASSEMBLER

#define __tssi_enable()                       ( REG_TSSI_ENA |= TSSI_ENA_ENA )
#define __tssi_disable()                      ( REG_TSSI_ENA &= ~TSSI_ENA_ENA )
#define __tssi_soft_reset()                   ( REG_TSSI_ENA |= TSSI_ENA_SFT_RST )
#define __tssi_dma_enable()                   ( REG_TSSI_ENA |= TSSI_ENA_DMA_EN )
#define __tssi_dma_disable()                  ( REG_TSSI_ENA &= ~TSSI_ENA_DMA_EN )
#define __tssi_filter_enable()                ( REG_TSSI_ENA |= TSSI_ENA_PID_EN )
#define __tssi_filter_disable()               ( REG_TSSI_ENA &= ~TSSI_ENA_PID_EN )

/* n = 4, 8, 16 */
#define __tssi_set_tigger_num(n)			\
	do {						\
		REG_TSSI_CFG &= ~TSSI_CFG_TRIG_MASK;	\
		REG_TSSI_CFG |= TSSI_CFG_TRIG_##n;	\
	} while (0)

#define __tssi_set_wd_1()                     ( REG_TSSI_CFG |= TSSI_CFG_END_WD )
#define __tssi_set_wd_0()                     ( REG_TSSI_CFG &= ~TSSI_CFG_END_WD )

#define __tssi_set_bt_1()                     ( REG_TSSI_CFG |= TSSI_CFG_END_BD )
#define __tssi_set_bt_0()                     ( REG_TSSI_CFG &= ~TSSI_CFG_END_BD )

#define __tssi_set_data_pola_high()           ( REG_TSSI_CFG |= TSSI_CFG_TSDI_H )
#define __tssi_set_data_pola_low()            ( REG_TSSI_CFG &= ~TSSI_CFG_TSDI_H )

#define __tssi_set_data_use_data0()           ( REG_TSSI_CFG |= TSSI_CFG_USE_0 )
#define __tssi_set_data_use_data7()           ( REG_TSSI_CFG &= ~TSSI_CFG_USE_0 )

#define __tssi_select_clk_fast()              ( REG_TSSI_CFG &= ~TSSI_CFG_TSCLK_CH )
#define __tssi_select_clk_slow()              ( REG_TSSI_CFG |= TSSI_CFG_TSCLK_CH )

#define __tssi_select_serail_mode()           ( REG_TSSI_CFG &= ~TSSI_CFG_PARAL )
#define __tssi_select_paral_mode()            ( REG_TSSI_CFG |= TSSI_CFG_PARAL )

#define __tssi_select_clk_nega_edge()         ( REG_TSSI_CFG &= ~TSSI_CFG_TSCLK_P )
#define __tssi_select_clk_posi_edge()         ( REG_TSSI_CFG |= TSSI_CFG_TSCLK_P )

#define __tssi_select_frm_act_high()          ( REG_TSSI_CFG |= TSSI_CFG_TSFRM_H )
#define __tssi_select_frm_act_low()           ( REG_TSSI_CFG &= ~TSSI_CFG_TSFRM_H )

#define __tssi_select_str_act_high()          ( REG_TSSI_CFG |= TSSI_CFG_TSSTR_H )
#define __tssi_select_str_act_low()           ( REG_TSSI_CFG &= ~TSSI_CFG_TSSTR_H )

#define __tssi_select_fail_act_high()         ( REG_TSSI_CFG |= TSSI_CFG_TSFAIL_H )
#define __tssi_select_fail_act_low()          ( REG_TSSI_CFG &= ~TSSI_CFG_TSFAIL_H )

#define __tssi_enable_ovrn_irq()              ( REG_TSSI_CTRL &= ~TSSI_CTRL_OVRNM )
#define __tssi_disable_ovrn_irq()             ( REG_TSSI_CTRL |= TSSI_CTRL_OVRNM )

#define __tssi_enable_trig_irq()              ( REG_TSSI_CTRL &= ~TSSI_CTRL_TRIGM )
#define __tssi_disable_trig_irq()             ( REG_TSSI_CTRL |= TSSI_CTRL_TRIGM ) 

#define __tssi_state_is_overrun()             ( REG_TSSI_STAT & TSSI_STAT_OVRN )
#define __tssi_state_trigger_meet()           ( REG_TSSI_STAT & TSSI_STAT_TRIG )
#define __tssi_clear_state()                  ( REG_TSSI_STAT = 0 ) /* write 0??? */
#define __tssi_state_clear_overrun()          ( REG_TSSI_STAT = TSSI_STAT_OVRN )

#define __tssi_enable_filte_pid0()            ( REG_TSSI_PEN |= TSSI_PEN_PID0 )
#define __tssi_disable_filte_pid0()           ( REG_TSSI_PEN &= ~TSSI_PEN_PID0 )

/* m = 0, ..., 15 */
#define __tssi_enable_pid_filter(m)				\
	do {							\
		int n = (m);					\
		if ( n>=0 && n <(TSSI_PID_MAX*2) ) {		\
			if ( n >= TSSI_PID_MAX ) n += 8;	\
			REG_TSSI_PEN |= ( 1 << n );		\
		}						\
	} while (0)

/* m = 0, ..., 15 */
#define __tssi_disable_pid_filter(m)				\
	do {							\
		int n = (m);					\
		if ( n>=0 && n <(TSSI_PID_MAX*2) ) {		\
			if ( n >= TSSI_PID_MAX ) n += 8;	\
			REG_TSSI_PEN &= ~( 1 << n );		\
		}						\
	} while (0)

/* n = 0, ..., 7 */
#define __tssi_set_pid0(n, pid0)							\
	do {										\
		REG_TSSI_PID(n) &= ~TSSI_PID_PID0_MASK;					\
		REG_TSSI_PID(n) |= ((pid0)<<TSSI_PID_PID0_BIT)&TSSI_PID_PID0_MASK;	\
	}while (0)
/* n = 0, ..., 7 */
#define __tssi_set_pid1(n, pid1)							\
	do {										\
		REG_TSSI_PID(n) &= ~TSSI_PID_PID1_MASK;					\
		REG_TSSI_PID(n) |= ((pid1)<<TSSI_PID_PID1_BIT)&TSSI_PID_PID1_MASK;	\
	}while (0)

/* n = 0, ..., 15 */
#define __tssi_set_pid(n, pid)						\
	do {								\
		if ( n>=0 && n < TSSI_PID_MAX*2) {			\
			if ( n < TSSI_PID_MAX )				\
				__tssi_set_pid0(n, pid);		\
			else						\
				__tssi_set_pid1(n-TSSI_PID_MAX, pid);	\
		}							\
	}while (0)

#endif /* __MIPS_ASSEMBLER */

#endif /* __MIPS_ASSEMBLER */
