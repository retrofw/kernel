#ifndef __JZ4770_MAC_REGS__
#define __JZ4770_MAC_REGS__

/* unit: bytes */
#define MAC_TX_FIFO_SIZE	2048
#define MAC_RX_FIFO_SIZE	4096

/* DMA Module */
/****************/
#define MAC_DMA_TX_CTRL		(ETHC_BASE + 0x180)

#define __jzmac_enable_tx_dma() (REG32(MAC_DMA_TX_CTRL) |= BIT0)

/* NOTE: bit0(tx enable) will be cleared by the
   built-in DMA controller whenever it encounters a Tx Underrun or Bus Error state. */
#define __jzmac_disable_tx_dma() (REG32(MAC_DMA_TX_CTRL) &= (~BIT0))

#define __jzmac_tx_dma_stopped() (!(REG32(MAC_DMA_TX_CTRL) & BIT0))


/****************/
#define MAC_DMA_TX_DESC		(ETHC_BASE + 0x184)
#define __jzmac_set_tx_desc_addr(addr)			\
	do {						\
		BUG_ON((addr) & 0x1);			\
							\
		REG32(MAC_DMA_TX_DESC) = (addr);	\
	} while(0)

/****************/
#define MAC_DMA_TX_STATUS	(ETHC_BASE + 0x188)
#define __jzmac_tx_status_err()	(REG32(MAC_DMA_TX_STATUS) & (BIT3 | BIT1))
#define __jzmac_tx_status_clr_err()	(REG32(MAC_DMA_TX_STATUS) |= (BIT3 | BIT1))

#define __jzmac_get_txed_pkt_cnt()			\
	({						\
		u32 status = REG32(MAC_DMA_TX_STATUS);	\
		(status & BITS_H2L(23, 16)) >> 16;	\
	})

#define __jzmac_is_tx_pkt_sent()			\
	({						\
		u32 status = REG32(MAC_DMA_TX_STATUS);	\
		(status & BIT0);			\
	})

#define __jzmac_tx_reduce_pkt_cnt()			\
	do {						\
		REG32(MAC_DMA_TX_STATUS) |= BIT0;	\
	} while(0)

#define __jzmac_clear_tx_pkt_cnt()			\
	do {						\
		while(__jzmac_get_txed_pkt_cnt())	\
			__jzmac_tx_reduce_pkt_cnt();	\
	} while(0)


/****************/
#define MAC_DMA_RX_CTRL		(ETHC_BASE + 0x18c)

#define __jzmac_enable_rx_dma()			\
	do {					\
		REG32(MAC_DMA_RX_CTRL) |= BIT0;	\
	} while(0)

/* The bit is cleared by the built-in DMA controller whenever it
   encounters an Rx Overflow or Bus Error state. Default is ‘0’. */
#define __jzmac_disable_rx_dma()				\
	do {							\
		REG32(MAC_DMA_RX_CTRL) &= (~BIT0);		\
	} while(0)

#define __jzmac_rx_dma_stopped() (!(REG32(MAC_DMA_RX_CTRL) & BIT0))

/****************/
#define MAC_DMA_RX_DESC		(ETHC_BASE + 0x190)
#define __jzmac_set_rx_desc_addr(addr)			\
	do {						\
		BUG_ON((addr) & 0x1);			\
		REG32(MAC_DMA_RX_DESC) = (addr);	\
	} while(0)

/****************/
#define MAC_DMA_RX_STATUS	(ETHC_BASE + 0x194)

#define __jzmac_rx_status_err() (REG32(MAC_DMA_RX_STATUS) & (BIT3 | BIT2))
#define __jzmac_rx_status_clr_err() (REG32(MAC_DMA_RX_STATUS) |= (BIT3 | BIT2))


#define __jzmac_get_rxed_pkt_cnt()			\
	({						\
		u32 status = REG32(MAC_DMA_RX_STATUS);	\
		((status & BITS_H2L(23,16)) >> 16);	\
	})

#define __jzmac_is_pkt_received()				\
	({						\
		u32 status = REG32(MAC_DMA_RX_STATUS);	\
		(status & BIT0);			\
	})

#define __jzmac_reduce_rx_pktcnt()			\
	do {						\
		REG32(MAC_DMA_RX_STATUS) |= BIT0;	\
	} while(0)

#define __jzmac_clear_rx_pkt_cnt()			\
	do {						\
		while(__jzmac_get_rxed_pkt_cnt())	\
			__jzmac_reduce_rx_pktcnt();	\
	} while(0)

/****************/
#define MAC_DMA_INTR_MASK	(ETHC_BASE + 0x198)
#define __jzmac_enable_statistics()			\
	do {						\
		REG32(MAC_DMA_INTR_MASK) |= BIT31;	\
	} while(0)

#define __jzmac_disable_statistics()			\
	do {						\
		REG32(MAC_DMA_INTR_MASK) &= (~BIT31);	\
	} while(0)

#define __jzmac_clear_counters()				\
	do {						\
		REG32(MAC_DMA_INTR_MASK) &= ~BIT30;	\
		REG32(MAC_DMA_INTR_MASK) |= BIT30;	\
	} while(0)

#define __jzmac_enable_auto_zero()			\
	do {						\
		REG32(MAC_DMA_INTR_MASK) |= BIT27;	\
	} while(0)

#define __jzmac_disable_auto_zero()			\
	do {						\
		REG32(MAC_DMA_INTR_MASK) &= (~BIT27);	\
	} while(0)

#define __JZMAC_DMA_BURST4	0x0
#define __JZMAC_DMA_BURST8	0x2
#define __JZMAC_DMA_BURST16	0x3
#define __jzmac_set_dma_burst_size(sz)				\
	do {							\
		REG32(MAC_DMA_INTR_MASK) &= (~BITS(28,27));	\
		REG32(MAC_DMA_INTR_MASK) |= ((sz) << 27);	\
	} while(0)

#define __jzmac_enable_tx_irq()				\
	do {						\
		REG32(MAC_DMA_INTR_MASK) |= BIT0;	\
	} while(0)

#define __jzmac_disable_tx_irq()				\
	do {						\
		REG32(MAC_DMA_INTR_MASK) &= (~BIT0);	\
	} while(0)

#define __jzmac_enable_urun_irq()				\
	do {						\
		REG32(MAC_DMA_INTR_MASK) |= BIT1;	\
	} while(0)

#define __jzmac_disable_urun_irq()			\
	do {						\
		REG32(MAC_DMA_INTR_MASK) &= (~BIT1);	\
	} while(0)

#define __jzmac_enable_tbus_err_irq()			\
	do {						\
		REG32(MAC_DMA_INTR_MASK) |= BIT3;	\
	} while(0)

#define __jzmac_disable_tbus_err_irq()			\
	do {						\
		REG32(MAC_DMA_INTR_MASK) &= (~BIT3);	\
	} while(0)

#define __jzmac_enable_rx_irq()				\
	do {						\
		REG32(MAC_DMA_INTR_MASK) |= BIT4;	\
	} while(0)

#define __jzmac_disable_rx_irq()				\
	do {						\
		REG32(MAC_DMA_INTR_MASK) &= (~BIT4);	\
	} while(0)

#define __jzmac_enable_xrun_irq()				\
	do {						\
		REG32(MAC_DMA_INTR_MASK) |= BIT6;	\
	} while(0)

#define __jzmac_disable_xrun_irq()			\
	do {						\
		REG32(MAC_DMA_INTR_MASK) &= (~BIT6);	\
	} while(0)

#define __jzmac_enable_rbus_err_irq()			\
	do {						\
		REG32(MAC_DMA_INTR_MASK) |= BIT7;	\
	} while(0)

#define __jzmac_disable_rbus_err_irq()			\
	do {						\
		REG32(MAC_DMA_INTR_MASK) &= (~BIT7);	\
	} while(0)

/* for tx: if there is any transmit mission, we are sure that dma&irq is enabled */
/* so we don't care underrun irq */
#define __jzmac_enable_usefull_tx_irq()		\
	do {					\
		__jzmac_enable_tx_irq();	\
		__jzmac_enable_tbus_err_irq();	\
	} while(0)

/* for rx: if there is any space left, we are sure the dma&irq is enabled */
/* do we don't care overflow irq */
#define __jzmac_enable_usefull_rx_irq()		\
	do {					\
		__jzmac_enable_rx_irq();	\
		__jzmac_enable_rbus_err_irq();	\
	} while(0)

#define __jzmac_enable_usefull_irq()			\
	do {						\
		__jzmac_enable_usefull_tx_irq();	\
		__jzmac_enable_usefull_rx_irq();	\
	} while(0)

#define __jzmac_enable_all_irq()		\
	do {					\
		__jzmac_enable_tx_irq();	\
		__jzmac_enable_urun_irq();	\
		__jzmac_enable_tbus_err_irq();	\
		__jzmac_enable_rx_irq();	\
		__jzmac_enable_xrun_irq();	\
		__jzmac_enable_rbus_err_irq();	\
	} while(0)

#define __jzmac_disable_all_irq()			\
	do {						\
		__jzmac_disable_tx_irq();		\
		__jzmac_disable_urun_irq();		\
		__jzmac_disable_tbus_err_irq();		\
		__jzmac_disable_rx_irq();		\
		__jzmac_disable_xrun_irq();		\
		__jzmac_disable_rbus_err_irq();		\
	} while(0)

/****************/
#define MAC_DMA_INTR		(ETHC_BASE + 0x19c)
/*  ETH_DMA_IR = ETH_DMA_IMR _AND_ corresponding status flags which in ETH_DMA_TSR and ETH_DMA_RSR
 *
 *  ETH_DMA_IR is read only, so the way to disable all interrupt is clear ETH_DMA_IMR or ETH_DMA_T(R)SR.
 *  Clear ETH_DMA_IMR is better. Then read status and call handle routine.
 *  Reset ETH_DMA_IMR after all interrupt-enabled status flags be cleared.
 */

/* FIFO Module */
#define MAC_FIFO_CFG_R0		(ETHC_BASE + 0x3c)
#define __jzmac_fifo_enable_all_module()				\
	do {								\
		int ___i = 0;						\
		REG32(MAC_FIFO_CFG_R0) |= BITS_H2L(12, 8);		\
		for (___i = 0;						\
		     (___i < MAX_TIMEOUT_CNT) && ((REG32(MAC_FIFO_CFG_R0) & (BITS_H2L(20, 18) | BIT16)) != (BITS_H2L(20, 18) | BIT16)); \
		     ___i++) {						\
			udelay(100);					\
		}							\
		if (___i == MAX_TIMEOUT_CNT) {				\
			printk("enable all fifo module: Wait time out! cfg_r0 = 0x%08x\n", REG32(MAC_FIFO_CFG_R0)); \
		}							\
	} while(0)

#define __jzmac_fifo_disable_all_module()				\
	do {								\
		int ___i = 0;						\
		REG32(MAC_FIFO_CFG_R0) &= ~BITS_H2L(12, 8);		\
		for ( ___i = 0;						\
		      (___i < MAX_TIMEOUT_CNT) && ((REG32(MAC_FIFO_CFG_R0) & (BIT20 | BIT19 | BIT17 | BIT16)) != 0); \
		      ___i++) {						\
			udelay(100);					\
		}							\
		if (___i == MAX_TIMEOUT_CNT) {				\
			printk("===>disable all fifo module: wait time out! cfg_r0 = 0x%08x\n", REG32(MAC_FIFO_CFG_R0)); \
		}							\
	} while(0)

#define __jzmac_fifo_reset_all_module()				\
	do {							\
		REG32(MAC_FIFO_CFG_R0) |= BITS_H2L(4, 0);	\
		mdelay(1);					\
		REG32(MAC_FIFO_CFG_R0) &= (~BITS_H2L(4, 0));	\
	} while(0)

/****************/
#define MAC_FIFO_CFG_R1		(ETHC_BASE + 0x4c)
/* typical sz: 1588 / 4 = 0x18d */
#define __jzmac_set_min_frm_ram(sz)				\
	do {							\
		int ___sz = (sz) / 4;				\
		if ((sz) & 0x3)					\
			___sz++;				\
								\
		REG32(MAC_FIFO_CFG_R1) &= (~BITS_H2L(27,16));	\
		REG32(MAC_FIFO_CFG_R1) |= (___sz << 16);	\
	} while(0)

#define __jzmac_set_xoffrtx(v)					\
	do {							\
		REG32(MAC_FIFO_CFG_R1) &= (~BITS_H2L(15,0));	\
		REG32(MAC_FIFO_CFG_R1) |= (v);			\
	} while(0)

/****************/
#define MAC_FIFO_CFG_R2		(ETHC_BASE + 0x50)
#define __jzmac_set_max_rx_watermark(sz)				\
	do {							\
		int ___sz = (sz) / 4;				\
		if ((sz) & 0x3)					\
			___sz++;				\
								\
		REG32(MAC_FIFO_CFG_R2) &= (~BITS_H2L(28,16));	\
		REG32(MAC_FIFO_CFG_R2) |= (___sz << 16);	\
	} while(0)

#define __jzmac_set_min_rx_watermark(sz)				\
	do {							\
		int ___sz = (sz) / 4;				\
		if ((sz) & 0x3)					\
			___sz++;				\
								\
		REG32(MAC_FIFO_CFG_R2) &= (~BITS_H2L(12,0));	\
		REG32(MAC_FIFO_CFG_R2) |= ___sz;		\
	} while(0)

/****************/
#define MAC_FIFO_CFG_R3		(ETHC_BASE + 0x54)
#define __jzmac_set_max_tx_watermark(sz)				\
	do {							\
		int ___sz = (sz) / 4;				\
		if ((sz) & 0x3)					\
			___sz++;				\
								\
		REG32(MAC_FIFO_CFG_R3) &= (~BITS_H2L(27,16));	\
		REG32(MAC_FIFO_CFG_R3) |= (___sz << 16);	\
	} while(0)

#define __jzmac_set_min_tx_watermark(sz)				\
	do {							\
		int ___sz = (sz) / 4;				\
		if ((sz) & 0x3)					\
			___sz++;				\
								\
		REG32(MAC_FIFO_CFG_R3) &= (~BITS_H2L(11,0));	\
		REG32(MAC_FIFO_CFG_R3) |= ___sz;		\
	} while(0)

/****************/
// Receive Status Vector
#define RSV_RVTD			(1 << 30)	// Receive VLAN Type detected
#define RSV_RUO				(1 << 29)	// Receive Unsupported Op-code
#define RSV_RPCF			(1 << 28)	// Receive Pause Control Frame
#define RSV_RCF				(1 << 27)	// Receive Control Frame
#define RSV_DN				(1 << 26)	// Dribble Nibble
#define RSV_BP				(1 << 25)	// Broadcast Packet
#define RSV_MP				(1 << 24)	// Multicast Packet
#define RSV_OK				(1 << 23)	// Receive OK
#define RSV_LOR				(1 << 22)	// Length Out of Range
#define RSV_LCE				(1 << 21)	// Length Check Error
#define RSV_CRCE			(1 << 20)	// CRC Error
#define RSV_RCV				(1 << 19)	// Receive Code Violation
#define RSV_CEPS			(1 << 18)	// Carrier Event Previously Seen
#define RSV_REPS			(1 << 17)	// RXDV Event Previously Seen
#define RSV_PPI				(1 << 16)	// Packet Previously Ignored

#define MAC_FIFO_CFG_R4		(ETHC_BASE + 0x58)
#define __jzmac_set_drop_cond(d)				\
	do {						\
		REG32(MAC_FIFO_CFG_R5) &= ~((d) >> 16);	\
		REG32(MAC_FIFO_CFG_R4) |= ((d) >> 16);	\
	} while(0)

#define __jzmac_clr_drop_cond(d)				\
	do {						\
		REG32(MAC_FIFO_CFG_R5) |= ((d) >> 16);	\
		REG32(MAC_FIFO_CFG_R4) &= ~((d) >> 16);	\
	} while(0)

/****************/
#define MAC_FIFO_CFG_R5		(ETHC_BASE + 0x5c)
#define __jzmac_enable_pause_frm()			\
	do {						\
		REG32(MAC_FIFO_CFG_R5) &= (~BIT22);	\
	} while(0)

#define __jzmac_enable_backpressure()			\
	do {						\
		REG32(MAC_FIFO_CFG_R5) |= BIT22;	\
	} while(0)

#define __jzmac_is_rx_fifo_full()				\
	({						\
		u32 r5 = REG32(MAC_FIFO_CFG_R5);	\
		(r5 & BIT21);				\
	})

#define __jzmac_wait_rx_fifo_not_full()			\
	do {						\
		REG32(MAC_FIFO_CFG_R5) |= BIT20;	\
		while(__jzmac_is_rx_fifo_full);		\
		REG32(MAC_FIFO_CFG_R5) &= ~BIT20;	\
	} while(0)

/* only in RMII */
#define __jzmac_set_clken_mode()				\
	do {						\
		REG32(MAC_FIFO_CFG_R5) |= BIT19;	\
	} while(0)

#define __jzmac_drop_less_64()				\
	do {						\
		REG32(MAC_FIFO_CFG_R5) |= BIT18;	\
	} while(0)

#define __jzmac_accept_less_64()				\
	do {						\
		REG32(MAC_FIFO_CFG_R5) &= ~BIT18;	\
	} while(0)

/* Note: we do not care the following registers */
#define MAC_FIFO_RAM_ACC_R0	(ETHC_BASE + 0x60)
#define MAC_FIFO_RAM_ACC_R1	(ETHC_BASE + 0x64)
#define MAC_FIFO_RAM_ACC_R2	(ETHC_BASE + 0x68)
#define MAC_FIFO_RAM_ACC_R3	(ETHC_BASE + 0x6c)
#define MAC_FIFO_RAM_ACC_R4	(ETHC_BASE + 0x70)
#define MAC_FIFO_RAM_ACC_R5	(ETHC_BASE + 0x74)
#define MAC_FIFO_RAM_ACC_R6	(ETHC_BASE + 0x78)
#define MAC_FIFO_RAM_ACC_R7	(ETHC_BASE + 0x7c)


/* MII Module */
#define MAC_MII_MAC1		(ETHC_BASE + 0x00)
#define __jzmac_reset_mii()			\
	do {					\
		REG32(MAC_MII_MAC1) |= BIT15;	\
		mdelay(10);			\
		REG32(MAC_MII_MAC1) &= ~BIT15;	\
	} while(0)

#define __jzmac_reset_rand_gen()			\
	do {					\
		REG32(MAC_MII_MAC1) |= BIT14;	\
		mdelay(10);			\
		REG32(MAC_MII_MAC1) &= ~BIT14;	\
	} while(0)

#define __jzmac_set_rmii_mode()			\
	do {					\
		REG32(MAC_MII_MAC1) |= BIT13;	\
	} while(0)

#define __jzmac_set_mii_mode()			\
	do {					\
		REG32(MAC_MII_MAC1) &= ~BIT13;	\
	} while(0)

#define __jzmac_reset_all_logic()				\
	do {						\
		REG32(MAC_MII_MAC1) |= BITS_H2L(11,8);	\
		mdelay(10);				\
		REG32(MAC_MII_MAC1) &= ~BITS_H2L(11,8);					\
	} while(0)

#define __jzmac_enable_loopback()			\
	do {					\
		REG32(MAC_MII_MAC1) |= BIT4;	\
	} while(0)

#define __jzmac_disable_loopback()		\
	do {					\
		REG32(MAC_MII_MAC1) &= ~BIT4;	\
	} while(0)

#define __jzmac_enable_tx_pause()			\
	do {					\
		REG32(MAC_MII_MAC1) |= BIT3;	\
	} while(0)

#define __jzmac_disable_tx_pause()			\
	do {					\
		REG32(MAC_MII_MAC1) &= ~BIT3;	\
	} while(0)

#define __jzmac_enable_rx_pause()			\
	do {					\
		REG32(MAC_MII_MAC1) |= BIT2;	\
	} while(0)

#define __jzmac_disable_rx_pause()			\
	do {					\
		REG32(MAC_MII_MAC1) &= ~BIT2;	\
	} while(0)

#define __jzmac_enable_pass_all()			\
	do {					\
		REG32(MAC_MII_MAC1) |= BIT1;	\
	} while(0)

#define __jzmac_disable_pass_all()			\
	do {					\
		REG32(MAC_MII_MAC1) &= ~BIT1;	\
	} while(0)

#define __jzmac_mii_enable_rx()			\
	do {					\
		REG32(MAC_MII_MAC1) |= BIT0;	\
	} while(0)

#define __jzmac_mii_disable_rx()			\
	do {					\
		REG32(MAC_MII_MAC1) &= ~BIT0;	\
	} while(0)

/****************/
#define MAC_MII_MAC2		(ETHC_BASE + 0x04)

/* standard */
#define __jzmac_enable_exc_defer()		\
	do {					\
		REG32(MAC_MII_MAC2) |= BIT14;	\
	} while(0)

#define __jzmac_disable_exc_defer()		\
	do {					\
		REG32(MAC_MII_MAC2) &= ~BIT14;	\
	} while(0)

#define __jzmac_enable_bp_nb()			\
	do {					\
		REG32(MAC_MII_MAC2) |= BIT13;	\
	} while(0)

/* standard */
#define __jzmac_disable_bp_nb()			\
	do {					\
		REG32(MAC_MII_MAC2) &= ~BIT13;	\
	} while(0)

/* standard */
#define __jzmac_enable_backoff()		\
	do {					\
		REG32(MAC_MII_MAC2) &= ~BIT12;	\
	} while(0)

#define __jzmac_disable_backoff()		\
	do {					\
		REG32(MAC_MII_MAC2) |= BIT12;	\
	} while(0)

#define __jzmac_mii_preamble_lt_12()		\
	do {					\
		REG32(MAC_MII_MAC2) |= BIT9;	\
	} while(0)

/* standard */
#define __jzmac_mii_preamble_any()		\
	do {					\
		REG32(MAC_MII_MAC2) &= ~BIT9;	\
	} while(0)

#define __jzmac_pass_pure_preamble()		\
	do {					\
		REG32(MAC_MII_MAC2) &= ~BIT8;	\
	} while(0)

/* standard */
#define __jzmac_reject_pure_preamble()		\
	do {					\
		REG32(MAC_MII_MAC2) |= BIT8;	\
	} while(0)

/* standard */
#define __jzmac_enable_auto_pad()			\
	do {					\
		REG32(MAC_MII_MAC2) |= BIT7;	\
	} while(0)

#define __jzmac_disable_auto_pad()		\
	do {					\
		REG32(MAC_MII_MAC2) &= ~BIT7;	\
	} while(0)

#define __jzmac_enable_vlan_pad()		\
	do {					\
		REG32(MAC_MII_MAC2) |= BIT6;	\
	} while(0)

#define __jzmac_disable_vlan_pad()		\
	do {					\
		REG32(MAC_MII_MAC2) &= ~BIT6;	\
	} while(0)

#define __jzmac_enable_padding()			\
	do {					\
		REG32(MAC_MII_MAC2) |= BIT5;	\
	} while(0)

#define __jzmac_disable_padding()			\
	do {					\
		REG32(MAC_MII_MAC2) &= ~BIT5;	\
	} while(0)

#define __jzmac_enable_crc()			\
	do {					\
		REG32(MAC_MII_MAC2) |= BIT4;	\
	} while(0)

#define __jzmac_disable_crc()			\
	do {					\
		REG32(MAC_MII_MAC2) &= ~BIT4;	\
	} while(0)

#define __jzmac_enable_dly_crc()			\
	do {					\
		REG32(MAC_MII_MAC2) |= BIT3;	\
	} while(0)

#define __jzmac_disable_dly_crc()			\
	do {					\
		REG32(MAC_MII_MAC2) &= ~BIT3;	\
	} while(0)

#define __jzmac_allow_huge_frm()			\
	do {					\
		REG32(MAC_MII_MAC2) |= BIT2;	\
	} while(0)

#define __jzmac_disallow_huge_frm()			\
	do {					\
		REG32(MAC_MII_MAC2) &= ~BIT2;	\
	} while(0)

#define __jzmac_enable_length_check()		\
	do {					\
		REG32(MAC_MII_MAC2) |= BIT1;	\
	} while(0)

#define __jzmac_disable_length_check()		\
	do {					\
		REG32(MAC_MII_MAC2) &= ~BIT1;	\
	} while(0)

#define __jzmac_enable_full_duplex()		\
	do {					\
		REG32(MAC_MII_MAC2) |= BIT0;	\
		REG32(MAC_MII_IPGT) = 0x15;	\
	} while(0)

#define __jzmac_disable_full_duplex()		\
	do {					\
		REG32(MAC_MII_MAC2) &= ~BIT0;	\
		REG32(MAC_MII_IPGT) = 0x12;	\
	} while(0)


/****************/
#define MAC_MII_IPGT		(ETHC_BASE + 0x08)

/****************/
#define MAC_MII_IPGR		(ETHC_BASE + 0x0c)

/* typical: 0x12 */
#define __jzmac_set_nb2b_ipg2(v)				\
	do {						\
		REG32(MAC_MII_IPGR) &= ~BITS_H2L(6,0);	\
		REG32(MAC_MII_IPGR) |= (v);		\
	} while(0)

/* typical: 0xc, must less than ipg2 */
#define __jzmac_set_nb2b_ipg1(v)				\
	do {						\
		REG32(MAC_MII_IPGR) &= ~BITS_H2L(14,8);	\
		REG32(MAC_MII_IPGR) |= ((v) << 8);	\
	} while(0)

/****************/
#define MAC_MII_CLRT		(ETHC_BASE + 0x10)
/* keep the default is ok */

/****************/
#define MAC_MII_MAXF		(ETHC_BASE + 0x14)

/* Note: change this when change MTU */
#define __jzmac_set_max_frm_size(sz)		\
	do {					\
		REG32(MAC_MII_MAXF) = (sz);	\
	} while(0)

/****************/
#define MAC_MII_SUPP		(ETHC_BASE + 0x18)
#define __jzmac_reset_intf_module()		\
	do {					\
		REG32(MAC_MII_SUPP) |= BIT15;	\
		mdelay(10);			\
		REG32(MAC_MII_SUPP) &= ~BIT15;	\
	} while(0)

/* default */
#define __jzmac_connect_to_phy()			\
do {						\
	REG32(MAC_MII_SUPP) |= BIT12;		\
 } while(0)

#define __jzmac_connect_to_mac()			\
	do {					\
		REG32(MAC_MII_SUPP) &= ~BIT12;	\
	} while(0)

#define __jzmac_reset_rmii()			\
	do {					\
		REG32(MAC_MII_SUPP) |= BIT11;	\
		mdelay(10);			\
		REG32(MAC_MII_SUPP) |= BIT11;	\
	} while(0)

#define __jzmac_set_rmii_100M()			\
	do {					\
		REG32(MAC_MII_SUPP) |= BIT8;	\
	} while(0)

#define __jzmac_set_rmii_10M()			\
	do {					\
		REG32(MAC_MII_SUPP) &= ~BIT8;	\
	} while(0)

#define __jzmac_reset_100x()			\
	do {					\
		REG32(MAC_MII_SUPP) |= BIT7;	\
		mdelay(10);			\
		REG32(MAC_MII_SUPP) |= BIT7;	\
	} while(0)

/****************/
#define MAC_MII_TEST		(ETHC_BASE + 0x1c)


/****************/
#define MAC_MII_MCFG		(ETHC_BASE + 0x20)
#define __jzmac_reset_mii_mgt()			\
	do {					\
		REG32(MAC_MII_MCFG) |= BIT15;	\
		mdelay(10);			\
		REG32(MAC_MII_MCFG) &= ~BIT15;	\
	} while(0)

#define __jzmac_mii_mgt_enter_reset()		\
	do {					\
		REG32(MAC_MII_MCFG) |= BIT15;	\
		REG32(MAC_MII_MCFG);		\
		REG32(MAC_MII_MCFG);		\
	} while(0)

#define __jzmac_mii_mgt_leave_reset()		\
	do {					\
		REG32(MAC_MII_MCFG) &= ~BIT15;	\
		REG32(MAC_MII_MCFG);		\
		REG32(MAC_MII_MCFG);		\
	} while(0)

#define __jzmac_set_mdc_clkdiv(div)			\
	do {						\
		REG32(MAC_MII_MCFG) &= ~BITS_H2L(4,2);	\
		REG32(MAC_MII_MCFG) |= ((div) << 2);	\
	} while(0)

#define __jzmac_get_mdc_clkdiv()	((REG32(MAC_MII_MCFG) & BITS_H2L(4,2)) >> 2)

#define __jzmac_mdc_with_preamble()	REG32(MAC_MII_MCFG) &= (~BIT1);
#define __jzmac_mdc_without_preamble()	REG32(MAC_MII_MCFG) |= BIT1;


/****************/
#define MAC_MII_MCMD		(ETHC_BASE + 0x24)
#define __jzmac_mii_enable_scan()			\
	do {					\
		REG32(MAC_MII_MCMD) |= BIT1;	\
	} while(0)

#define __jzmac_mii_disable_scan()		\
	do {					\
		REG32(MAC_MII_MCMD) &= ~BIT1;	\
	} while(0)

#define __jzmac_mii_enable_read()			\
	do {					\
		REG32(MAC_MII_MCMD) |= BIT0;	\
	} while(0)

#define __jzmac_mii_disable_read()		\
	do {					\
		REG32(MAC_MII_MCMD) &= ~BIT0;	\
	} while(0)

/****************/
#define MAC_MII_MADR		(ETHC_BASE + 0x28)
#define __jzmac_set_mii_address(pa, ra)					\
	do {								\
		REG32(MAC_MII_MADR) =					\
			(((pa) << 8)& BITS_H2L(12,8))|((ra) & BITS_H2L(4,0)); \
	} while(0)


#define __jzmac_send_mii_read_cmd(pa, ra, scan)				\
	do {								\
		__jzmac_set_mii_address(pa, ra);				\
		REG32(MAC_MII_MCMD) &= ~BITS_H2L(1,0);			\
		REG32(MAC_MII_MCMD) |=	(BIT0 | ((scan) << 1));		\
	} while(0)

#define __jzmac_send_mii_write_cmd(pa, ra, wdata)				\
	do {								\
		__jzmac_set_mii_address(pa, ra);				\
		REG32(MAC_MII_MCMD) &= ~BITS_H2L(1,0);			\
		__jzmac_mii_write_data((wdata));				\
	} while(0)




/****************/
#define MAC_MII_MWTD		(ETHC_BASE + 0x2c)
#define __jzmac_mii_write_data(d16)		(REG16(MAC_MII_MWTD) = d16)


/****************/
#define MAC_MII_MRDD		(ETHC_BASE + 0x30)
#define __jzmac_mii_read_data()			REG16(MAC_MII_MRDD)

/****************/
#define MAC_MII_MIND		(ETHC_BASE + 0x34)
#define __jzmac_mii_link_fail()  (REG32(MAC_MII_MIND) & BIT3)

#define __jzmac_mii_read_done()	(!(REG32(MAC_MII_MIND) & BIT2))

#define __jzmac_mii_scaning()	(REG32(MAC_MII_MIND) & BIT1)

#define __jzmac_mii_is_busy()	(REG32(MAC_MII_MIND) & BIT0)

/****************/
#define MAC_MII_SA0		(ETHC_BASE + 0x40)
#define MAC_MII_SA1		(ETHC_BASE + 0x44)
#define MAC_MII_SA2		(ETHC_BASE + 0x48)

/* SAL Module */
#define MAC_SAL_AFR		(ETHC_BASE + 0x1a0)
#define __jzmac_enable_promiscuous()	(REG32(MAC_SAL_AFR) |= BIT3)
#define __jzmac_disable_promiscuous()	(REG32(MAC_SAL_AFR) &= ~BIT3)
#define __jzmac_accept_multicast()	(REG32(MAC_SAL_AFR) |= BIT2)
#define __jzmac_reject_multicast()	(REG32(MAC_SAL_AFR) &= ~BIT2)

#define __jzmac_accept_multicast_q()	(REG32(MAC_SAL_AFR) |= BIT1)
#define __jzmac_reject_multicast_q()	(REG32(MAC_SAL_AFR) &= ~BIT1)

#define __jzmac_accept_broadcast()	(REG32(MAC_SAL_AFR) |= BIT0)
#define __jzmac_reject_broadcast()	(REG32(MAC_SAL_AFR) &= ~BIT0)

#define MAC_SAL_HT1		(ETHC_BASE + 0x1a4)
#define MAC_SAL_HT2		(ETHC_BASE + 0x1a8)

#define __sal_set_hash_table(_hi32, _lo32)	\
	do {					\
		REG32(MAC_SAL_HT1) = _hi32;	\
		REG32(MAC_SAL_HT2) = _lo32;	\
	} while(0)

#define __sal_get_hash_table(_hi32, _lo32)	\
	do {					\
		_hi32 = REG32(MAC_SAL_HT1);	\
		_lo32 = REG32(MAC_SAL_HT2);	\
	} while(0)

/* Operations of DMA descripter */
// Constants for DMA descriptor
#define EMPTY_FLAG_MASK			(0x01 << 31)
#define FTPP_FLAGS_MASK			(0x1f << 16)
#define FTCFRM_MASK			(0x01 << 20)
#define FTPP_PADMODE_MASK		(0x03 << 18)
#define FTPP_GENFCS_MASK		(0x01 << 17)
#define FTPP_EN_MASK			(0x01 << 16)
#define PKT_SIZE_MASK			(0x0FFF)

#define __desc_get_empty_flag(pktsize)		(pktsize & EMPTY_FLAG_MASK)

/* empty flag set to '1' means successfully transmit a packet */
#define __desc_tx_transfer_done(d)		((d)->pkt_size & EMPTY_FLAG_MASK)
#define __desc_tx_as_valid(d)			((d)->pkt_size &= ~EMPTY_FLAG_MASK)
#define __desc_tx_as_invalid(d)			((d)->pkt_size |= EMPTY_FLAG_MASK)

/* empty flag set to '0' means successfully received a packet */
#define __desc_rx_transfer_done(d)		(!((d)->pkt_size & EMPTY_FLAG_MASK))
#define __desc_rx_as_valid(d)			((d)->pkt_size |= EMPTY_FLAG_MASK)
#define __desc_rx_as_invalid(d)			((d)->pkt_size &= ~EMPTY_FLAG_MASK)

#define __desc_get_pkt_size(pktsize)		(pktsize & PKT_SIZE_MASK)


/* STAT Module */
#define MAC_STAT_TR64			(ETHC_BASE + 0x80)
#define MAC_STAT_TR127			(ETHC_BASE + 0x84)
#define MAC_STAT_TR255			(ETHC_BASE + 0x88)
#define MAC_STAT_TR511			(ETHC_BASE + 0x8c)
#define MAC_STAT_TR1K			(ETHC_BASE + 0x90)
#define MAC_STAT_TRMAX			(ETHC_BASE + 0x94)
#define MAC_STAT_TRMGV			(ETHC_BASE + 0x98)
#define MAC_STAT_RBYT			(ETHC_BASE + 0x9c)
#define MAC_STAT_RPKT			(ETHC_BASE + 0xa0)
#define MAC_STAT_RFCS			(ETHC_BASE + 0xa4)
#define MAC_STAT_RMCA			(ETHC_BASE + 0xa8)
#define MAC_STAT_RBCA			(ETHC_BASE + 0xac)
#define MAC_STAT_RXCF			(ETHC_BASE + 0xb0)
#define MAC_STAT_RXPF			(ETHC_BASE + 0xb4)
#define MAC_STAT_RXUO			(ETHC_BASE + 0xb8)
#define MAC_STAT_RALN			(ETHC_BASE + 0xbc)
#define MAC_STAT_RFLR			(ETHC_BASE + 0xc0)
#define MAC_STAT_RCDE			(ETHC_BASE + 0xc4)
#define MAC_STAT_RCSE			(ETHC_BASE + 0xc8)
#define MAC_STAT_RUND			(ETHC_BASE + 0xcc)
#define MAC_STAT_ROVR			(ETHC_BASE + 0xd0)
#define MAC_STAT_RFRG			(ETHC_BASE + 0xd4)
#define MAC_STAT_RJBR			(ETHC_BASE + 0xd8)
#define MAC_STAT_RDRP			(ETHC_BASE + 0xdc)
#define MAC_STAT_TBYT			(ETHC_BASE + 0xe0)
#define MAC_STAT_TPKT			(ETHC_BASE + 0xe4)
#define MAC_STAT_TMCA			(ETHC_BASE + 0xe8)
#define MAC_STAT_TBCA			(ETHC_BASE + 0xec)
#define MAC_STAT_TXPF			(ETHC_BASE + 0xf0)
#define MAC_STAT_TDFR			(ETHC_BASE + 0xf4)
#define MAC_STAT_TEDF			(ETHC_BASE + 0xf8)
#define MAC_STAT_TSCL			(ETHC_BASE + 0xfc)
#define MAC_STAT_TMCL			(ETHC_BASE + 0x100)
#define MAC_STAT_TLCL			(ETHC_BASE + 0x104)
#define MAC_STAT_TXCL			(ETHC_BASE + 0x108)
#define MAC_STAT_TNCL			(ETHC_BASE + 0x10c)
#define MAC_STAT_TPFH			(ETHC_BASE + 0x110)
#define MAC_STAT_TDRP			(ETHC_BASE + 0x114)
#define MAC_STAT_TJBR			(ETHC_BASE + 0x118)
#define MAC_STAT_TFCS			(ETHC_BASE + 0x11c)
#define MAC_STAT_TXCF			(ETHC_BASE + 0x120)
#define MAC_STAT_TOVR			(ETHC_BASE + 0x124)
#define MAC_STAT_TUND			(ETHC_BASE + 0x128)
#define MAC_STAT_TFRG			(ETHC_BASE + 0x12c)
#define MAC_STAT_CAR1			(ETHC_BASE + 0x130)
#define MAC_STAT_CAR2			(ETHC_BASE + 0x134)
#define MAC_STAT_CAM1			(ETHC_BASE + 0x138)
#define MAC_STAT_CAM2			(ETHC_BASE + 0x13c)

#define __jzmac_stat_disable_carry_irq()	\
	do {					\
		REG32(MAC_STAT_CAM1) = ~0;	\
		REG32(MAC_STAT_CAM2) = ~0;	\
	} while(0)

struct jzmac_hw_stats {
	u64 tr64;
	u64 tr127;
	u64 tr255;
	u64 tr511;
	u64 tr1k;
	u64 trmax;
	u64 trmgv;
	u64 rbyt;
	u64 rpkt;
	u64 rfcs;
	u64 rmca;
	u64 rbca;
	u64 rxcf;
	u64 rxpf;
	u64 rxuo;
	u64 raln;
	u64 rflr;
	u64 rcde;
	u64 rcse;
	u64 rund;
	u64 rovr;
	u64 rfrg;
	u64 rjbr;
	u64 rdrp;
	u64 tbyt;
	u64 tpkt;
	u64 tmca;
	u64 tbca;
	u64 txpf;
	u64 tdfr;
	u64 tedf;
	u64 tscl;
	u64 tmcl;
	u64 tlcl;
	u64 txcl;
	u64 tncl;
	u64 tpfh;
	u64 tdrp;
	u64 tjbr;
	u64 tfcs;
	u64 txcf;
	u64 tovr;
	u64 tund;
	u64 tfrg;
};

#endif /* __JZ4770_MAC_REGS__ */
