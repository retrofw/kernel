/*
 *  linux/drivers/net/jz_eth.h
 *
 *  Jz4730/Jz5730 On-Chip ethernet driver.
 *
 *  Copyright (C) 2005 - 2007  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef __JZ_ETH_H__
#define __JZ_ETH_H__

/* DMA control and status registers */
#define DMA_BMR                (ETH_BASE + 0x1000) // Bus mode
#define DMA_TPD                (ETH_BASE + 0x1004) // Transmit poll demand register
#define DMA_RPD                (ETH_BASE + 0x1008) // Receieve poll demand register
#define DMA_RRBA               (ETH_BASE + 0x100C) // Receieve descriptor base address
#define DMA_TRBA               (ETH_BASE + 0x1010) // Transmit descriptor base address
#define DMA_STS                (ETH_BASE + 0x1014) // Status register
#define DMA_OMR                (ETH_BASE + 0x1018) // Command register
#define DMA_IMR                (ETH_BASE + 0x101C)
#define DMA_MFC                (ETH_BASE + 0x1020)

/* DMA CSR8-CSR19 reserved */
#define DMA_CTA                (ETH_BASE + 0x1050)
#define DMA_CRA                (ETH_BASE + 0x1054)

/* Mac control and status registers */
#define MAC_MCR                (ETH_BASE + 0x0000)
#define MAC_MAH                (ETH_BASE + 0x0004)
#define MAC_MAL                (ETH_BASE + 0x0008)
#define MAC_HTH                (ETH_BASE + 0x000C)
#define MAC_HTL                (ETH_BASE + 0x0010)
#define MAC_MIIA               (ETH_BASE + 0x0014)
#define MAC_MIID               (ETH_BASE + 0x0018)
#define MAC_FCR                (ETH_BASE + 0x001C)
#define MAC_VTR1               (ETH_BASE + 0x0020)
#define MAC_VTR2               (ETH_BASE + 0x0024)

/*
 * Bus Mode Register (DMA_BMR)
 */
#define BMR_PBL    0x00003f00       /* Programmable Burst Length */
#define BMR_DSL    0x0000007c       /* Descriptor Skip Length */
#define BMR_BAR    0x00000002       /* Bus ARbitration */
#define BMR_SWR    0x00000001       /* Software Reset */

#define PBL_0      0x00000000       /*  DMA burst length = amount in RX FIFO */
#define PBL_1      0x00000100       /*  1 longword  DMA burst length */
#define PBL_2      0x00000200       /*  2 longwords DMA burst length */
#define PBL_4      0x00000400       /*  4 longwords DMA burst length */
#define PBL_8      0x00000800       /*  8 longwords DMA burst length */
#define PBL_16     0x00001000       /* 16 longwords DMA burst length */
#define PBL_32     0x00002000       /* 32 longwords DMA burst length */

#define DSL_0      0x00000000       /*  0 longword  / descriptor */
#define DSL_1      0x00000004       /*  1 longword  / descriptor */
#define DSL_2      0x00000008       /*  2 longwords / descriptor */
#define DSL_4      0x00000010       /*  4 longwords / descriptor */
#define DSL_8      0x00000020       /*  8 longwords / descriptor */
#define DSL_16     0x00000040       /* 16 longwords / descriptor */
#define DSL_32     0x00000080       /* 32 longwords / descriptor */

/*
 * Status Register (DMA_STS)
 */
#define STS_BE     0x03800000       /* Bus Error Bits */
#define STS_TS     0x00700000       /* Transmit Process State */
#define STS_RS     0x000e0000       /* Receive Process State */

#define TS_STOP    0x00000000       /* Stopped */
#define TS_FTD     0x00100000       /* Running Fetch Transmit Descriptor */
#define TS_WEOT    0x00200000       /* Running Wait for End Of Transmission */
#define TS_QDAT    0x00300000       /* Running Queue skb data into TX FIFO */
#define TS_RES     0x00400000       /* Reserved */
#define TS_SPKT    0x00500000       /* Reserved */
#define TS_SUSP    0x00600000       /* Suspended */
#define TS_CLTD    0x00700000       /* Running Close Transmit Descriptor */

#define RS_STOP    0x00000000       /* Stopped */
#define RS_FRD     0x00020000       /* Running Fetch Receive Descriptor */
#define RS_CEOR    0x00040000       /* Running Check for End of Receive Packet */
#define RS_WFRP    0x00060000       /* Running Wait for Receive Packet */
#define RS_SUSP    0x00080000       /* Suspended */
#define RS_CLRD    0x000a0000       /* Running Close Receive Descriptor */
#define RS_FLUSH   0x000c0000       /* Running Flush RX FIFO */
#define RS_QRFS    0x000e0000       /* Running Queue RX FIFO into RX Skb */

/*
 * Operation Mode Register (DMA_OMR)
 */
#define OMR_TTM    0x00400000       /* Transmit Threshold Mode */
#define OMR_SF     0x00200000       /* Store and Forward */
#define OMR_TR     0x0000c000       /* Threshold Control Bits */
#define OMR_ST     0x00002000       /* Start/Stop Transmission Command */
#define OMR_OSF    0x00000004       /* Operate on Second Frame */
#define OMR_SR     0x00000002       /* Start/Stop Receive */

#define TR_18      0x00000000       /* Threshold set to 18 (32) bytes */
#define TR_24      0x00004000       /* Threshold set to 24 (64) bytes */
#define TR_32      0x00008000       /* Threshold set to 32 (128) bytes */
#define TR_40      0x0000c000       /* Threshold set to 40 (256) bytes */

/*
 * Missed Frames Counters (DMA_MFC)
 */
//#define MFC_CNT1   0xffff0000       /* Missed Frames Counter Bits by application */
#define MFC_CNT1   0x0ffe0000       /* Missed Frames Counter Bits by application */
#define MFC_CNT2   0x0000ffff       /* Missed Frames Counter Bits by controller */

/*
 * Mac control  Register (MAC_MCR)
 */
#define MCR_RA     0x80000000       /* Receive All */
#define MCR_HBD    0x10000000       /* HeartBeat Disable */
#define MCR_PS     0x08000000       /* Port Select */
#define MCR_OWD    0x00800000       /* Receive own Disable */
#define MCR_OM     0x00600000       /* Operating(loopback) Mode */
#define MCR_FDX    0x00100000       /* Full Duplex Mode */
#define MCR_PM     0x00080000       /* Pass All Multicast */
#define MCR_PR     0x00040000       /* Promiscuous Mode */
#define MCR_IF     0x00020000       /* Inverse Filtering */
#define MCR_PB     0x00010000       /* Pass Bad Frames */
#define MCR_HO     0x00008000       /* Hash Only Filtering Mode */
#define MCR_HP     0x00002000       /* Hash/Perfect Receive Filtering Mode */
#define MCR_FC     0x00001000       /* Late Collision control */
#define MCR_BFD    0x00000800       /* Boardcast frame Disable */
#define MCR_RED    0x00000400       /* Retry Disable */
#define MCR_APS    0x00000100       /* Automatic pad stripping */
#define MCR_BL     0x000000c0       /* Back off Limit */
#define MCR_DC     0x00000020       /* Deferral check */
#define MCR_TE     0x00000008       /* Transmitter enable */
#define MCR_RE     0x00000004       /* Receiver enable */

#define MCR_MII_10  ( OMR_TTM | MCR_PS)
#define MCR_MII_100 ( MCR_HBD | MCR_PS)

/* Flow control Register (MAC_FCR) */
#define FCR_PT     0xffff0000       /* Pause time */
#define FCR_PCF    0x00000004       /* Pass control frames */
#define FCR_FCE    0x00000002       /* Flow control enable */
#define FCR_FCB    0x00000001       /* Flow control busy */


/* Constants for the interrupt mask and
 * interrupt status registers. (DMA_SIS and DMA_IMR)
 */
#define DMA_INT_NI             0x00010000       // Normal interrupt summary
#define DMA_INT_AI             0x00008000       // Abnormal interrupt summary
#define DMA_INT_ER             0x00004000       // Early receive interrupt
#define DMA_INT_FB             0x00002000       // Fatal bus error
#define DMA_INT_ET             0x00000400       // Early transmit interrupt
#define DMA_INT_RW             0x00000200       // Receive watchdog timeout
#define DMA_INT_RS             0x00000100       // Receive stop
#define DMA_INT_RU             0x00000080       // Receive buffer unavailble
#define DMA_INT_RI             0x00000040       // Receive interrupt
#define DMA_INT_UN             0x00000020       // Underflow 
#define DMA_INT_TJ             0x00000008       // Transmit jabber timeout
#define DMA_INT_TU             0x00000004       // Transmit buffer unavailble 
#define DMA_INT_TS             0x00000002       // Transmit stop
#define DMA_INT_TI             0x00000001       // Transmit interrupt

/*
 * Receive Descriptor Bit Summary
 */
#define R_OWN      0x80000000       /* Own Bit */
#define RD_FF      0x40000000       /* Filtering Fail */
#define RD_FL      0x3fff0000       /* Frame Length */
#define RD_ES      0x00008000       /* Error Summary */
#define RD_DE      0x00004000       /* Descriptor Error */
#define RD_LE      0x00001000       /* Length Error */
#define RD_RF      0x00000800       /* Runt Frame */
#define RD_MF      0x00000400       /* Multicast Frame */
#define RD_FS      0x00000200       /* First Descriptor */
#define RD_LS      0x00000100       /* Last Descriptor */
#define RD_TL      0x00000080       /* Frame Too Long */
#define RD_CS      0x00000040       /* Collision Seen */
#define RD_FT      0x00000020       /* Frame Type */
#define RD_RJ      0x00000010       /* Receive Watchdog timeout*/
#define RD_RE      0x00000008       /* Report on MII Error */
#define RD_DB      0x00000004       /* Dribbling Bit */
#define RD_CE      0x00000002       /* CRC Error */

#define RD_RER     0x02000000       /* Receive End Of Ring */
#define RD_RCH     0x01000000       /* Second Address Chained */
#define RD_RBS2    0x003ff800       /* Buffer 2 Size */
#define RD_RBS1    0x000007ff       /* Buffer 1 Size */

/*
 * Transmit Descriptor Bit Summary
 */
#define T_OWN      0x80000000       /* Own Bit */
#define TD_ES      0x00008000       /* Frame Aborted (error summary)*/
#define TD_LO      0x00000800       /* Loss Of Carrier */
#define TD_NC      0x00000400       /* No Carrier */
#define TD_LC      0x00000200       /* Late Collision */
#define TD_EC      0x00000100       /* Excessive Collisions */
#define TD_HF      0x00000080       /* Heartbeat Fail */
#define TD_CC      0x0000003c       /* Collision Counter */
#define TD_UF      0x00000002       /* Underflow Error */
#define TD_DE      0x00000001       /* Deferred */

#define TD_IC      0x80000000       /* Interrupt On Completion */
#define TD_LS      0x40000000       /* Last Segment */
#define TD_FS      0x20000000       /* First Segment */
#define TD_FT1     0x10000000       /* Filtering Type */
#define TD_SET     0x08000000       /* Setup Packet */
#define TD_AC      0x04000000       /* Add CRC Disable */
#define TD_TER     0x02000000       /* Transmit End Of Ring */
#define TD_TCH     0x01000000       /* Second Address Chained */
#define TD_DPD     0x00800000       /* Disabled Padding */
#define TD_FT0     0x00400000       /* Filtering Type */
#define TD_TBS2    0x003ff800       /* Buffer 2 Size */
#define TD_TBS1    0x000007ff       /* Buffer 1 Size */

#define PERFECT_F  0x00000000
#define HASH_F     TD_FT0
#define INVERSE_F  TD_FT1
#define HASH_O_F   (TD_FT1 | TD_F0)

/*
 * Constant setting
 */

#define IMR_DEFAULT    ( DMA_INT_TI | DMA_INT_RI |	\
                         DMA_INT_TS | DMA_INT_RS |	\
                         DMA_INT_TU | DMA_INT_RU |	\
                         DMA_INT_FB )

#define IMR_ENABLE     (DMA_INT_NI | DMA_INT_AI)

#define CRC_POLYNOMIAL_BE 0x04c11db7UL  /* Ethernet CRC, big endian */
#define CRC_POLYNOMIAL_LE 0xedb88320UL  /* Ethernet CRC, little endian */

#define HASH_TABLE_LEN   512       /* Bits */
#define HASH_BITS        0x01ff    /* 9 LS bits */

#define SETUP_FRAME_LEN  192       /* Bytes */
#define IMPERF_PA_OFFSET 156       /* Bytes */

/*
 * Address Filtering Modes
 */
#define PERFECT              0     /* 16 perfect physical addresses */
#define HASH_PERF            1     /* 1 perfect, 512 multicast addresses */
#define PERFECT_REJ          2     /* Reject 16 perfect physical addresses */
#define ALL_HASH             3     /* Hashes all physical & multicast addrs */

#define ALL                  0     /* Clear out all the setup frame */
#define PHYS_ADDR_ONLY       1     /* Update the physical address only */

/* MII register */
#define MII_BMCR       0x00          /* MII Basic Mode Control Register */
#define MII_BMSR       0x01          /* MII Basic Mode Status Register */
#define MII_ID1        0x02          /* PHY Identifier Register 1 */
#define MII_ID2        0x03          /* PHY Identifier Register 2 */
#define MII_ANAR       0x04          /* Auto Negotiation Advertisement Register */
#define MII_ANLPAR     0x05          /* Auto Negotiation Link Partner Ability */
#define MII_ANER       0x06          /* Auto Negotiation Expansion */
#define MII_DSCR       0x10          /* Davicom Specified Configration Register */
#define MII_DSCSR      0x11          /* Davicom Specified Configration/Status Register */
#define MII_10BTCSR    0x12          /* 10base-T Specified Configration/Status Register */


#define MII_PREAMBLE 0xffffffff    /* MII Management Preamble */
#define MII_TEST     0xaaaaaaaa    /* MII Test Signal */
#define MII_STRD     0x06          /* Start of Frame+Op Code: use low nibble */
#define MII_STWR     0x0a          /* Start of Frame+Op Code: use low nibble */

/*
 * MII Management Control Register
 */
#define MII_CR_RST  0x8000         /* RESET the PHY chip */
#define MII_CR_LPBK 0x4000         /* Loopback enable */
#define MII_CR_SPD  0x2000         /* 0: 10Mb/s; 1: 100Mb/s */
#define MII_CR_ASSE 0x1000         /* Auto Speed Select Enable */
#define MII_CR_PD   0x0800         /* Power Down */
#define MII_CR_ISOL 0x0400         /* Isolate Mode */
#define MII_CR_RAN  0x0200         /* Restart Auto Negotiation */
#define MII_CR_FDM  0x0100         /* Full Duplex Mode */
#define MII_CR_CTE  0x0080         /* Collision Test Enable */

/*
 * MII Management Status Register
 */
#define MII_SR_T4C  0x8000         /* 100BASE-T4 capable */
#define MII_SR_TXFD 0x4000         /* 100BASE-TX Full Duplex capable */
#define MII_SR_TXHD 0x2000         /* 100BASE-TX Half Duplex capable */
#define MII_SR_TFD  0x1000         /* 10BASE-T Full Duplex capable */
#define MII_SR_THD  0x0800         /* 10BASE-T Half Duplex capable */
#define MII_SR_ASSC 0x0020         /* Auto Speed Selection Complete*/
#define MII_SR_RFD  0x0010         /* Remote Fault Detected */
#define MII_SR_ANC  0x0008         /* Auto Negotiation capable */
#define MII_SR_LKS  0x0004         /* Link Status */
#define MII_SR_JABD 0x0002         /* Jabber Detect */
#define MII_SR_XC   0x0001         /* Extended Capabilities */

/*
 * MII Management Auto Negotiation Advertisement Register
 */
#define MII_ANA_TAF  0x03e0        /* Technology Ability Field */
#define MII_ANA_T4AM 0x0200        /* T4 Technology Ability Mask */
#define MII_ANA_TXAM 0x0180        /* TX Technology Ability Mask */
#define MII_ANA_FDAM 0x0140        /* Full Duplex Technology Ability Mask */
#define MII_ANA_HDAM 0x02a0        /* Half Duplex Technology Ability Mask */
#define MII_ANA_100M 0x0380        /* 100Mb Technology Ability Mask */
#define MII_ANA_10M  0x0060        /* 10Mb Technology Ability Mask */
#define MII_ANA_CSMA 0x0001        /* CSMA-CD Capable */

/*
 * MII Management Auto Negotiation Remote End Register
 */
#define MII_ANLPA_NP   0x8000      /* Next Page (Enable) */
#define MII_ANLPA_ACK  0x4000      /* Remote Acknowledge */
#define MII_ANLPA_RF   0x2000      /* Remote Fault */
#define MII_ANLPA_TAF  0x03e0      /* Technology Ability Field */
#define MII_ANLPA_T4AM 0x0200      /* T4 Technology Ability Mask */
#define MII_ANLPA_TXAM 0x0180      /* TX Technology Ability Mask */
#define MII_ANLPA_FDAM 0x0140      /* Full Duplex Technology Ability Mask */
#define MII_ANLPA_HDAM 0x02a0      /* Half Duplex Technology Ability Mask */
#define MII_ANLPA_100M 0x0380      /* 100Mb Technology Ability Mask */
#define MII_ANLPA_10M  0x0060      /* 10Mb Technology Ability Mask */
#define MII_ANLPA_CSMA 0x0001      /* CSMA-CD Capable */

/*
 * MII Management DAVICOM Specified Configuration And Status Register
 */
#define MII_DSCSR_100FDX       0x8000  /* 100M Full Duplex Operation Mode */    
#define MII_DSCSR_100HDX       0x4000  /* 100M Half Duplex Operation Mode */
#define MII_DSCSR_10FDX        0x2000  /* 10M  Full Duplex Operation Mode */
#define MII_DSCSR_10HDX        0x1000  /* 10M  Half Duplex Operation Mode */
#define MII_DSCSR_ANMB         0x000f  /* Auto-Negotiation Monitor Bits   */


/*
 * Used by IOCTL
 */
#define READ_COMMAND		(SIOCDEVPRIVATE+4)
#define WRITE_COMMAND		(SIOCDEVPRIVATE+5)
#define GETDRIVERINFO		(SIOCDEVPRIVATE+6)

/*
 * Device data and structure
 */

#define ETH_TX_TIMEOUT		(6*HZ)

#define RX_BUF_SIZE		1536

#define NUM_RX_DESCS		32
#define NUM_TX_DESCS		16

static const char *media_types[] = {
	"10BaseT-HD ", "10BaseT-FD ","100baseTx-HD ", 
	"100baseTx-FD", "100baseT4", 0
};

typedef struct {
	unsigned int status;
	unsigned int desc1;
	unsigned int buf1_addr;
	unsigned int next_addr;
} jz_desc_t;	

struct jz_eth_private {
	jz_desc_t tx_ring[NUM_TX_DESCS];	/* transmit descriptors */
	jz_desc_t rx_ring[NUM_RX_DESCS];	/* receive descriptors */
	dma_addr_t dma_tx_ring;                 /* bus address of tx ring */
	dma_addr_t dma_rx_ring;                 /* bus address of rx ring */
	dma_addr_t dma_rx_buf;			/* DMA address of rx buffer  */	
	unsigned int vaddr_rx_buf;		/* virtual address of rx buffer  */

	unsigned int rx_head;			/* first rx descriptor */
	unsigned int tx_head;			/* first tx descriptor */
	unsigned int tx_tail;  			/* last unacked transmit packet */
	unsigned int tx_full;			/* transmit buffers are full */
	struct sk_buff *tx_skb[NUM_TX_DESCS];	/* skbuffs for packets to transmit */

	struct net_device_stats stats;
	spinlock_t lock;

	int media;				/* Media (eg TP), mode (eg 100B)*/
	int full_duplex;			/* Current duplex setting. */
	int link_state;
	char phys[32];				/* List of attached PHY devices */
	char valid_phy;				/* Current linked phy-id with MAC */
	int mii_phy_cnt;
	int phy_type;				/* 1-RTL8309,0-DVCOM */
	struct ethtool_cmd ecmds[32];
	u16 advertising;			/* NWay media advertisement */

	pid_t thr_pid;				/* Link cheak thread ID   */
	int thread_die;
	struct completion thr_exited;
	wait_queue_head_t thr_wait;

	struct pm_dev *pmdev;
};

#endif /* __JZ_ETH_H__ */
