/*
 * JZ4770 On-Chip MAC Driver
 *
 *  Copyright (C) 2010 - 2011  Ingenic Semiconductor Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include "jz4770_mac_regs.h"

//#define JZMAC_ENABLE_4BYTE_WORKAROUND

/* 4x or 4x-1 */
#define jzmac_tx_align_invalid(saddr) ( (((u32)((saddr)) & 0x3) == 0) || (((u32)((saddr)) & 0x3) == 3) )

/* NOTE: one cache line can have 32 / 4 = 8 descriptors */
struct jzmac_dma_desc {
	volatile unsigned int pkt_addr;	// physical addr
	volatile unsigned int pkt_size;	// include empty flag
	unsigned int next_desc;	// physical addr(Note: though there is a next pointer, we use array instead of list for dma descs)

	unsigned int for_align; // for 4-word alignment
};

/* wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer */
struct jzmac_buffer {
	struct sk_buff *skb;
	dma_addr_t dma;
	unsigned long time_stamp;
	u16 length;
	volatile u8 transfering; /* used by tx */
	volatile u8 invalid;	 /* used by rx */
#ifdef JZMAC_ENABLE_4BYTE_WORKAROUND
	u8 *work_around;
#endif
};

/* TX/RX descriptor defines */
#define JZMAC_TX_DESC_COUNT                96
#define JZMAC_MAX_TXD                      256
#define JZMAC_MIN_TXD                       80

#define JZMAC_RX_DESC_COUNT                96
#define JZMAC_MAX_RXD                      256
#define JZMAC_MIN_RXD                       80

struct jzmac_tx_ring {
	/* pointer to the descriptor ring memory */
	struct jzmac_dma_desc *desc;
	/* physical address of the descriptor ring */
	dma_addr_t dma;
	/* number of descriptors in the ring */
	unsigned int count;
	/* next descriptor to associate a buffer with */
	unsigned int next_to_use;
	/* next descriptor to check for trans done status */
	unsigned int next_to_clean;
	/* array of buffer information structs */
	struct jzmac_buffer *buffer_info;
};

struct jzmac_rx_ring {
	/* pointer to the descriptor ring memory */
	struct jzmac_dma_desc *desc;
	/* physical address of the descriptor ring */
	dma_addr_t dma;
	/* number of descriptors in the ring */
	unsigned int count;
	/* next descriptor to associate a buffer with */
	unsigned int next_to_use;
	/* next descriptor to check for DD status bit */
	unsigned int next_to_clean;
	/* array of buffer information structs */
	struct jzmac_buffer *buffer_info;
};

#define JZMAC_DESC_UNUSED(R)						\
	((((R)->next_to_clean > (R)->next_to_use)			\
	  ? 0 : (R)->count) + (R)->next_to_clean - (R)->next_to_use - 1)

#define JZMAC_DESC_USED(R)  (((R)->count - 1) - JZMAC_DESC_UNUSED(R))

#define JZMAC_GET_DESC(R, i, type)	(&(((struct type *)((R).desc))[i]))
#define JZMAC_RX_DESC(R, i)		JZMAC_GET_DESC(R, i, jzmac_dma_desc)
#define JZMAC_TX_DESC(R, i)		JZMAC_GET_DESC(R, i, jzmac_dma_desc)

struct jz4770_mac_local {
	struct timer_list watchdog_timer;

	struct jzmac_tx_ring tx_ring;
	unsigned int restart_queue;
	u32 tx_timeout_count;

	struct jzmac_rx_ring rx_ring;

	struct napi_struct napi;
	spinlock_t napi_poll_lock;

	struct net_device *netdev;
	struct platform_device *pdev;
	struct net_device_stats net_stats;

	spinlock_t stats_lock;
	unsigned int total_tx_bytes;
	unsigned int total_tx_packets;
	unsigned int total_rx_bytes;
	unsigned int total_rx_packets;

	atomic_t tx_fifo_used;

	unsigned char Mac[6];	/* MAC address of the board */
	spinlock_t link_lock;

	/* MII and PHY stuffs */
	int old_link;
	int old_speed;
	int old_duplex;

	struct phy_device *phydev;
	struct mii_bus *mii_bus;

	u32 alloc_rx_buff_failed;

	struct work_struct reset_task;
};
