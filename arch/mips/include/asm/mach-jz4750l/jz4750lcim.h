/*
 * linux/include/asm-mips/mach-jz4750l/jz4750lcim.h
 *
 * JZ4750L CIM register definition
 *
 * Copyright (C) 2008-2014 Ingenic Semiconductor Co., Ltd.
 */
#ifndef __JZ4750LCIM_H__
#define __JZ4750LCIM_H__

/* Define the module base addresses */
#define	CIM_BASE	0xB3060000

/*************************************************************************
 * CIM
 *************************************************************************/
#define	CIM_CFG			(CIM_BASE + 0x0000)
#define	CIM_CTRL		(CIM_BASE + 0x0004)
#define	CIM_STATE		(CIM_BASE + 0x0008)
#define	CIM_IID			(CIM_BASE + 0x000C)
#define	CIM_RXFIFO		(CIM_BASE + 0x0010)
#define	CIM_DA			(CIM_BASE + 0x0020)
#define	CIM_FA			(CIM_BASE + 0x0024)
#define	CIM_FID			(CIM_BASE + 0x0028)
#define	CIM_CMD			(CIM_BASE + 0x002C)
#define	CIM_SIZE		(CIM_BASE + 0x0030)
#define	CIM_OFFSET		(CIM_BASE + 0x0034)
#define	CIM_RAM_ADDR		(CIM_BASE + 0x1000)

#define	REG_CIM_CFG		REG32(CIM_CFG)
#define	REG_CIM_CTRL		REG32(CIM_CTRL)
#define	REG_CIM_STATE		REG32(CIM_STATE)
#define	REG_CIM_IID		REG32(CIM_IID)
#define	REG_CIM_RXFIFO		REG32(CIM_RXFIFO)
#define	REG_CIM_DA		REG32(CIM_DA)
#define	REG_CIM_FA		REG32(CIM_FA)
#define	REG_CIM_FID		REG32(CIM_FID)
#define	REG_CIM_CMD		REG32(CIM_CMD)
#define	REG_CIM_SIZE		REG32(CIM_SIZE)
#define	REG_CIM_OFFSET		REG32(CIM_OFFSET)

#define	CIM_CFG_ORDER_BIT	18
#define	CIM_CFG_ORDER_MASK	(0x3 << CIM_CFG_ORDER_BIT)
  #define CIM_CFG_ORDER_0	  (0x0 << CIM_CFG_ORDER_BIT) 	/* Y0CbY1Cr; YCbCr */
  #define CIM_CFG_ORDER_1	  (0x1 << CIM_CFG_ORDER_BIT)	/* Y0CrY1Cb; YCrCb */
  #define CIM_CFG_ORDER_2	  (0x2 << CIM_CFG_ORDER_BIT)	/* CbY0CrY1; CbCrY */
  #define CIM_CFG_ORDER_3	  (0x3 << CIM_CFG_ORDER_BIT)	/* CrY0CbY1; CrCbY */
#define	CIM_CFG_DF_BIT		16
#define	CIM_CFG_DF_MASK		  (0x3 << CIM_CFG_DF_BIT)
  #define CIM_CFG_DF_YUV444	  (0x1 << CIM_CFG_DF_BIT) 	/* YCbCr444 */
  #define CIM_CFG_DF_YUV422	  (0x2 << CIM_CFG_DF_BIT)	/* YCbCr422 */
  #define CIM_CFG_DF_ITU656	  (0x3 << CIM_CFG_DF_BIT)	/* ITU656 YCbCr422 */
#define	CIM_CFG_INV_DAT		(1 << 15)
#define	CIM_CFG_VSP		(1 << 14) /* VSYNC Polarity:0-rising edge active,1-falling edge active */
#define	CIM_CFG_HSP		(1 << 13) /* HSYNC Polarity:0-rising edge active,1-falling edge active */
#define	CIM_CFG_PCP		(1 << 12) /* PCLK working edge: 0-rising, 1-falling */
#define	CIM_CFG_DMA_BURST_TYPE_BIT	10
#define	CIM_CFG_DMA_BURST_TYPE_MASK	(0x3 << CIM_CFG_DMA_BURST_TYPE_BIT)
  #define	CIM_CFG_DMA_BURST_INCR4		(0 << CIM_CFG_DMA_BURST_TYPE_BIT)
  #define	CIM_CFG_DMA_BURST_INCR8		(1 << CIM_CFG_DMA_BURST_TYPE_BIT)	/* Suggested */
  #define	CIM_CFG_DMA_BURST_INCR16	(2 << CIM_CFG_DMA_BURST_TYPE_BIT)	/* Suggested High speed AHB*/
#define	CIM_CFG_DUMMY_ZERO	(1 << 9)
#define	CIM_CFG_EXT_VSYNC	(1 << 8)	/* Only for ITU656 Progressive mode */
#define	CIM_CFG_PACK_BIT	4
#define	CIM_CFG_PACK_MASK	(0x7 << CIM_CFG_PACK_BIT)
  #define CIM_CFG_PACK_0	  (0 << CIM_CFG_PACK_BIT) /* 11 22 33 44 0xY0CbY1Cr */
  #define CIM_CFG_PACK_1	  (1 << CIM_CFG_PACK_BIT) /* 22 33 44 11 0xCbY1CrY0 */
  #define CIM_CFG_PACK_2	  (2 << CIM_CFG_PACK_BIT) /* 33 44 11 22 0xY1CrY0Cb */
  #define CIM_CFG_PACK_3	  (3 << CIM_CFG_PACK_BIT) /* 44 11 22 33 0xCrY0CbY1 */
  #define CIM_CFG_PACK_4	  (4 << CIM_CFG_PACK_BIT) /* 44 33 22 11 0xCrY1CbY0 */
  #define CIM_CFG_PACK_5	  (5 << CIM_CFG_PACK_BIT) /* 33 22 11 44 0xY1CbY0Cr */
  #define CIM_CFG_PACK_6	  (6 << CIM_CFG_PACK_BIT) /* 22 11 44 33 0xCbY0CrY1 */
  #define CIM_CFG_PACK_7	  (7 << CIM_CFG_PACK_BIT) /* 11 44 33 22 0xY0CrY1Cb */
#define	CIM_CFG_BYPASS_BIT	2
#define	CIM_CFG_BYPASS_MASK	(1 << CIM_CFG_BYPASS_BIT)
  #define CIM_CFG_BYPASS	  (1 << CIM_CFG_BYPASS_BIT)
#define	CIM_CFG_DSM_BIT		0
#define	CIM_CFG_DSM_MASK	(0x3 << CIM_CFG_DSM_BIT)
  #define CIM_CFG_DSM_CPM	  (0 << CIM_CFG_DSM_BIT) /* CCIR656 Progressive Mode */
  #define CIM_CFG_DSM_CIM	  (1 << CIM_CFG_DSM_BIT) /* CCIR656 Interlace Mode */
  #define CIM_CFG_DSM_GCM	  (2 << CIM_CFG_DSM_BIT) /* Gated Clock Mode */

/* CIM Control Register  (CIM_CTRL) */
#define	CIM_CTRL_EEOF_LINE_BIT	20
#define	CIM_CTRL_EEOF_LINE_MASK	(0xfff << CIM_CTRL_EEOF_LINE_BIT)
#define	CIM_CTRL_FRC_BIT	16
#define	CIM_CTRL_FRC_MASK	(0xf << CIM_CTRL_FRC_BIT)
  #define CIM_CTRL_FRC_1	  (0x0 << CIM_CTRL_FRC_BIT) /* Sample every frame */
  #define CIM_CTRL_FRC_2	  (0x1 << CIM_CTRL_FRC_BIT) /* Sample 1/2 frame */
  #define CIM_CTRL_FRC_3	  (0x2 << CIM_CTRL_FRC_BIT) /* Sample 1/3 frame */
  #define CIM_CTRL_FRC_4	  (0x3 << CIM_CTRL_FRC_BIT) /* Sample 1/4 frame */
  #define CIM_CTRL_FRC_5	  (0x4 << CIM_CTRL_FRC_BIT) /* Sample 1/5 frame */
  #define CIM_CTRL_FRC_6	  (0x5 << CIM_CTRL_FRC_BIT) /* Sample 1/6 frame */
  #define CIM_CTRL_FRC_7	  (0x6 << CIM_CTRL_FRC_BIT) /* Sample 1/7 frame */
  #define CIM_CTRL_FRC_8	  (0x7 << CIM_CTRL_FRC_BIT) /* Sample 1/8 frame */
  #define CIM_CTRL_FRC_9	  (0x8 << CIM_CTRL_FRC_BIT) /* Sample 1/9 frame */
  #define CIM_CTRL_FRC_10	  (0x9 << CIM_CTRL_FRC_BIT) /* Sample 1/10 frame */
  #define CIM_CTRL_FRC_11	  (0xA << CIM_CTRL_FRC_BIT) /* Sample 1/11 frame */
  #define CIM_CTRL_FRC_12	  (0xB << CIM_CTRL_FRC_BIT) /* Sample 1/12 frame */
  #define CIM_CTRL_FRC_13	  (0xC << CIM_CTRL_FRC_BIT) /* Sample 1/13 frame */
  #define CIM_CTRL_FRC_14	  (0xD << CIM_CTRL_FRC_BIT) /* Sample 1/14 frame */
  #define CIM_CTRL_FRC_15	  (0xE << CIM_CTRL_FRC_BIT) /* Sample 1/15 frame */
  #define CIM_CTRL_FRC_16	  (0xF << CIM_CTRL_FRC_BIT) /* Sample 1/16 frame */

#define	CIM_CTRL_DMA_EEOF	(1 << 15)	/* Enable EEOF interrupt */
#define	CIM_CTRL_WIN_EN		(1 << 14)
#define	CIM_CTRL_VDDM		(1 << 13) /* VDD interrupt enable */
#define	CIM_CTRL_DMA_SOFM	(1 << 12)
#define	CIM_CTRL_DMA_EOFM	(1 << 11)
#define	CIM_CTRL_DMA_STOPM	(1 << 10)
#define	CIM_CTRL_RXF_TRIGM	(1 << 9)
#define	CIM_CTRL_RXF_OFM	(1 << 8)
#define	CIM_CTRL_DMA_SYNC	(1 << 7)	/*when change DA, do frame sync */
#define	CIM_CTRL_RXF_TRIG_BIT	3
#define	CIM_CTRL_RXF_TRIG_MASK	(0xf << CIM_CTRL_RXF_TRIG_BIT) /* trigger value = (n+1)*burst_type */

#define	CIM_CTRL_DMA_EN		(1 << 2) /* Enable DMA */
#define	CIM_CTRL_RXF_RST	(1 << 1) /* RxFIFO reset */
#define	CIM_CTRL_ENA		(1 << 0) /* Enable CIM */

/* CIM State Register  (CIM_STATE) */
#define	CIM_STATE_DMA_EEOF	(1 << 7) /* DMA Line EEOf irq */
#define	CIM_STATE_DMA_SOF	(1 << 6) /* DMA start irq */
#define	CIM_STATE_DMA_EOF	(1 << 5) /* DMA end irq */
#define	CIM_STATE_DMA_STOP	(1 << 4) /* DMA stop irq */
#define	CIM_STATE_RXF_OF	(1 << 3) /* RXFIFO over flow irq */
#define	CIM_STATE_RXF_TRIG	(1 << 2) /* RXFIFO triger meet irq */
#define	CIM_STATE_RXF_EMPTY	(1 << 1) /* RXFIFO empty irq */
#define	CIM_STATE_VDD		(1 << 0) /* CIM disabled irq */

/* CIM DMA Command Register (CIM_CMD) */

#define	CIM_CMD_SOFINT		(1 << 31) /* enable DMA start irq */
#define	CIM_CMD_EOFINT		(1 << 30) /* enable DMA end irq */
#define	CIM_CMD_EEOFINT		(1 << 29) /* enable DMA EEOF irq */
#define	CIM_CMD_STOP		(1 << 28) /* enable DMA stop irq */
#define	CIM_CMD_OFRCV		(1 << 27) /* enable recovery when TXFiFo overflow */
#define	CIM_CMD_LEN_BIT		0
#define	CIM_CMD_LEN_MASK	(0xffffff << CIM_CMD_LEN_BIT)

/* CIM Window-Image Size Register  (CIM_SIZE) */
#define	CIM_SIZE_LPF_BIT	16 /* Lines per freame for csc output image */
#define	CIM_SIZE_LPF_MASK	(0x1fff << CIM_SIZE_LPF_BIT)
#define	CIM_SIZE_PPL_BIT	0 /* Pixels per line for csc output image, should be an even number */
#define	CIM_SIZE_PPL_MASK	(0x1fff << CIM_SIZE_PPL_BIT)

/* CIM Image Offset Register  (CIM_OFFSET) */
#define	CIM_OFFSET_V_BIT	16 /* Vertical offset */
#define	CIM_OFFSET_V_MASK	(0xfff << CIM_OFFSET_V_BIT)
#define	CIM_OFFSET_H_BIT	0 /* Horizontal offset, should be an enen number */
#define	CIM_OFFSET_H_MASK	(0xfff << CIM_OFFSET_H_BIT) /*OFFSET_H should be even number*/

/*
 * CIM Operations
 */
#ifndef __MIPS_ASSEMBLER

#define __cim_enable()	( REG_CIM_CTRL |= CIM_CTRL_ENA )
#define __cim_disable()	( REG_CIM_CTRL &= ~CIM_CTRL_ENA )

/* n = 0, 1, 2, 3 */
#define __cim_set_input_data_stream_order(n)				\
	do {								\
		REG_CIM_CFG &= CIM_CFG_ORDER_MASK;			\
		REG_CIM_CFG |= ((n)<<CIM_CFG_ORDER_BIT)&CIM_CFG_ORDER_MASK; \
	} while (0)

#define __cim_input_data_format_select_RGB()		\
	do {						\
		REG_CIM_CFG &= CIM_CFG_DF_MASK;		\
		REG_CIM_CFG |= CIM_CFG_DF_RGB;		\
	} while (0)

#define __cim_input_data_format_select_YUV444()		\
	do {						\
		REG_CIM_CFG &= CIM_CFG_DF_MASK;		\
		REG_CIM_CFG |= CIM_CFG_DF_YUV444;	\
	} while (0)

#define __cim_input_data_format_select_YUV422()		\
	do {						\
		REG_CIM_CFG &= CIM_CFG_DF_MASK;		\
		REG_CIM_CFG |= CIM_CFG_DF_YUV422;	\
	} while (0)

#define __cim_input_data_format_select_ITU656()		\
	do {						\
		REG_CIM_CFG &= CIM_CFG_DF_MASK;		\
		REG_CIM_CFG |= CIM_CFG_DF_ITU656;	\
	} while (0)

#define __cim_input_data_inverse()	( REG_CIM_CFG |= CIM_CFG_INV_DAT )
#define __cim_input_data_normal()	( REG_CIM_CFG &= ~CIM_CFG_INV_DAT )

#define __cim_vsync_active_low()	( REG_CIM_CFG |= CIM_CFG_VSP )
#define __cim_vsync_active_high()	( REG_CIM_CFG &= ~CIM_CFG_VSP )

#define __cim_hsync_active_low()	( REG_CIM_CFG |= CIM_CFG_HSP )
#define __cim_hsync_active_high()	( REG_CIM_CFG &= ~CIM_CFG_HSP )

#define __cim_sample_data_at_pclk_falling_edge() \
	( REG_CIM_CFG |= CIM_CFG_PCP )
#define __cim_sample_data_at_pclk_rising_edge() \
	( REG_CIM_CFG &= ~CIM_CFG_PCP )

#define __cim_enable_dummy_zero()	( REG_CIM_CFG |= CIM_CFG_DUMMY_ZERO )
#define __cim_disable_dummy_zero()	( REG_CIM_CFG &= ~CIM_CFG_DUMMY_ZERO )

#define __cim_select_external_vsync()	( REG_CIM_CFG |= CIM_CFG_EXT_VSYNC )
#define __cim_select_internal_vsync()	( REG_CIM_CFG &= ~CIM_CFG_EXT_VSYNC )

/* n=0-7 */
#define __cim_set_data_packing_mode(n) 			\
	do {						\
		REG_CIM_CFG &= ~CIM_CFG_PACK_MASK;	\
		REG_CIM_CFG |= (CIM_CFG_PACK_##n);	\
	} while (0)

#define __cim_enable_bypass_func() 	(REG_CIM_CFG |= CIM_CFG_BYPASS)
#define __cim_disable_bypass_func() 	(REG_CIM_CFG &= ~CIM_CFG_BYPASS_MASK)

#define __cim_enable_ccir656_progressive_mode()		\
	do {						\
		REG_CIM_CFG &= ~CIM_CFG_DSM_MASK;	\
		REG_CIM_CFG |= CIM_CFG_DSM_CPM;		\
	} while (0)

#define __cim_enable_ccir656_interlace_mode()		\
	do {						\
		REG_CIM_CFG &= ~CIM_CFG_DSM_MASK;	\
		REG_CIM_CFG |= CIM_CFG_DSM_CIM;		\
	} while (0)

#define __cim_enable_gated_clock_mode()			\
	do {						\
		REG_CIM_CFG &= ~CIM_CFG_DSM_MASK;	\
		REG_CIM_CFG |= CIM_CFG_DSM_GCM;		\
	} while (0)

#define __cim_enable_nongated_clock_mode()		\
	do {						\
		REG_CIM_CFG &= ~CIM_CFG_DSM_MASK;	\
		REG_CIM_CFG |= CIM_CFG_DSM_NGCM;	\
	} while (0)

/* sclk:system bus clock
 * mclk: CIM master clock
 */
#define __cim_set_master_clk(sclk, mclk)					\
	do {									\
		REG_CIM_CTRL &= ~CIM_CTRL_MCLKDIV_MASK;				\
		REG_CIM_CTRL |= (((sclk)/(mclk) - 1) << CIM_CTRL_MCLKDIV_BIT);	\
	} while (0)
/* n=1-16 */
#define __cim_set_frame_rate(n) 			\
	do {						\
		REG_CIM_CTRL &= ~CIM_CTRL_FRC_MASK; 	\
		REG_CIM_CTRL |= CIM_CTRL_FRC_##n;	\
	} while (0)

#define __cim_enable_size_func() \
	( REG_CIM_CTRL |= CIM_CTRL_SIZEEN )
#define __cim_disable_size_func() \
	( REG_CIM_CTRL &= ~CIM_CTRL_SIZEEN_MASK )

#define __cim_enable_vdd_intr() \
	( REG_CIM_CTRL |= CIM_CTRL_VDDM )
#define __cim_disable_vdd_intr() \
	( REG_CIM_CTRL &= ~CIM_CTRL_VDDM )

#define __cim_enable_sof_intr() \
	( REG_CIM_CTRL |= CIM_CTRL_DMA_SOFM )
#define __cim_disable_sof_intr() \
	( REG_CIM_CTRL &= ~CIM_CTRL_DMA_SOFM )

#define __cim_enable_eof_intr() \
	( REG_CIM_CTRL |= CIM_CTRL_DMA_EOFM )
#define __cim_disable_eof_intr() \
	( REG_CIM_CTRL &= ~CIM_CTRL_DMA_EOFM )

#define __cim_enable_eeof_intr() \
	( REG_CIM_CTRL |= CIM_CTRL_DMA_EEOFM )
#define __cim_disable_eeof_intr() \
	( REG_CIM_CTRL &= ~CIM_CTRL_DMA_EEOFM )

#define __cim_enable_stop_intr() \
	( REG_CIM_CTRL |= CIM_CTRL_DMA_STOPM )
#define __cim_disable_stop_intr() \
	( REG_CIM_CTRL &= ~CIM_CTRL_DMA_STOPM )

#define __cim_enable_trig_intr() \
	( REG_CIM_CTRL |= CIM_CTRL_RXF_TRIGM )
#define __cim_disable_trig_intr() \
	( REG_CIM_CTRL &= ~CIM_CTRL_RXF_TRIGM )

#define __cim_enable_rxfifo_overflow_intr()	\
	( REG_CIM_CTRL |= CIM_CTRL_RXF_OFM )
#define __cim_disable_rxfifo_overflow_intr()	\
	( REG_CIM_CTRL &= ~CIM_CTRL_RXF_OFM )

/* n=4,8,12,16,20,24,28,32 */
#define __cim_set_rxfifo_trigger(n) 				\
	do {							\
		REG_CIM_CTRL &= ~CIM_CTRL_RXF_TRIG_MASK; 	\
		REG_CIM_CTRL |= CIM_CTRL_RXF_TRIG_##n;		\
	} while (0)
#define __cim_enable_fast_mode() 	( REG_CIM_CTRL |= CIM_CTRL_FAST_MODE )
#define __cim_disable_fast_mode() 	( REG_CIM_CTRL &= ~CIM_CTRL_FAST_MODE )
#define __cim_use_normal_mode() 	__cim_disable_fast_mode()
#define __cim_enable_dma()   ( REG_CIM_CTRL |= CIM_CTRL_DMA_EN )
#define __cim_disable_dma()  ( REG_CIM_CTRL &= ~CIM_CTRL_DMA_EN )
#define __cim_reset_rxfifo() ( REG_CIM_CTRL |= CIM_CTRL_RXF_RST )
#define __cim_unreset_rxfifo() ( REG_CIM_CTRL &= ~CIM_CTRL_RXF_RST )

#define __cim_clear_state()   	     ( REG_CIM_STATE = 0 )

#define __cim_disable_done()   	     ( REG_CIM_STATE & CIM_STATE_VDD )
#define __cim_rxfifo_empty()   	     ( REG_CIM_STATE & CIM_STATE_RXF_EMPTY )
#define __cim_rxfifo_reach_trigger() ( REG_CIM_STATE & CIM_STATE_RXF_TRIG )
#define __cim_rxfifo_overflow()      ( REG_CIM_STATE & CIM_STATE_RXF_OF )
#define __cim_clear_rxfifo_overflow() ( REG_CIM_STATE &= ~CIM_STATE_RXF_OF )
#define __cim_dma_stop()   	     ( REG_CIM_STATE & CIM_STATE_DMA_STOP )
#define __cim_dma_eof()   	     ( REG_CIM_STATE & CIM_STATE_DMA_EOF )
#define __cim_dma_sof()   	     ( REG_CIM_STATE & CIM_STATE_DMA_SOF )

#define __cim_get_iid()   	     ( REG_CIM_IID )
#define __cim_get_fid()   	     ( REG_CIM_FID )
#define __cim_get_image_data()       ( REG_CIM_RXFIFO )
#define __cim_get_dma_cmd()          ( REG_CIM_CMD )

#define __cim_set_da(a)              ( REG_CIM_DA = (a) )

#define __cim_set_line(a) 	( REG_CIM_SIZE = (REG_CIM_SIZE&(~CIM_SIZE_LPF_MASK))|((a)<<CIM_SIZE_LPF_BIT) )
#define __cim_set_pixel(a) 	( REG_CIM_SIZE = (REG_CIM_SIZE&(~CIM_SIZE_PPL_MASK))|((a)<<CIM_SIZE_PPL_BIT) )
#define __cim_get_line() 	((REG_CIM_SIZE&CIM_SIZE_LPF_MASK)>>CIM_SIZE_LPF_BIT)
#define __cim_get_pixel() 	((REG_CIM_SIZE&CIM_SIZE_PPL_MASK)>>CIM_SIZE_PPL_BIT)

#define __cim_set_v_offset(a) 	( REG_CIM_OFFSET = (REG_CIM_OFFSET&(~CIM_OFFSET_V_MASK)) | ((a)<<CIM_OFFSET_V_BIT) )
#define __cim_set_h_offset(a) 	( REG_CIM_OFFSET = (REG_CIM_OFFSET&(~CIM_OFFSET_H_MASK)) | ((a)<<CIM_OFFSET_H_BIT) )
#define __cim_get_v_offset() 	((REG_CIM_OFFSET&CIM_OFFSET_V_MASK)>>CIM_OFFSET_V_BIT)
#define __cim_get_h_offset() 	((REG_CIM_OFFSET&CIM_OFFSET_H_MASK)>>CIM_OFFSET_H_BIT)

#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4750LCIM_H__ */
