/*
 * ASIX AX88796C based Fast Ethernet Devices
 * Copyright (C) 2009 ASIX Electronics Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _AX796C_H_
#define _AX796C_H_

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/if_vlan.h>
#include <linux/inetdevice.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
//#include <asm/dma.h>
#include <linux/dma-mapping.h>
//#include <asm/mach/arch.h>
#include <linux/platform_device.h>

#ifndef TRUE
#define TRUE				1
#endif

#ifndef FALSE
#define FALSE				0
#endif

/* AX88796C naming declarations */
#define AX88796C_DRV_NAME	"AX88796C"
#define AX88796C_ADP_NAME	"ASIX AX88796C Fast Ethernet Adapter"
#define AX88796C_DRV_VERSION	"1.2.0"
#define PFX			AX88796C_DRV_NAME ": "

/* Configuration options */
#if defined(CONFIG_SOC_JZ4760) || defined(CONFIG_SOC_JZ4760B) 
#define DATA_PORT_ADDR		0x0020
#else
#define DATA_PORT_ADDR		0x4000
#endif


/* AX88796C specific feature settings */
/* TRUE: enable, FALSE: disable */
#define AX88796C_8BIT_MODE		FALSE
#define AX88796C_WATCHDOG_PERIOD	(1 * HZ)
#define AX88796C_WATCHDOG_RESTART	7

#define TX_DMA_MODE			TRUE
#define RX_DMA_MODE			TRUE

/* Lutts@20101019 */
#define AX88796B_PIN_COMPATIBLE		FALSE

/* TX_DMA_MODE and TX_PIO_MODE are mutually exclusive */
#if (TX_DMA_MODE)
	#define TX_PIO_MODE		FALSE
#else
	#define TX_PIO_MODE		TRUE
#endif

/* RX_DMA_MODE and RX_PIO_MODE are mutually exclusive */
#if (RX_DMA_MODE)
	#define RX_PIO_MODE		FALSE
#else
	#define RX_PIO_MODE		TRUE
#endif

#define TX_QUEUE_HIGH_WATER		45		/* Tx queue high water mark */
#define TX_QUEUE_LOW_WATER		20		/* Tx queue low water mark */


#define TX_MANUAL_DEQUEUE		FALSE
#if (TX_MANUAL_DEQUEUE)
#define TX_MANUAL_DEQUEUE_CNT		0x30
#endif

#if (AX88796C_8BIT_MODE)
static void inline AX_WRITE (u16 data, void __iomem *mem)
{
	writeb ((u8)data, mem);
	writeb ((u8)(data >> 8), mem + 1);
}

static u16 inline AX_READ (void __iomem *mem)
{
        u16 data;
        u8 *tmp = (u8 *)&data;
        *tmp = readb (mem);
        *(tmp+1) = readb (mem + 1);
        return data;
}
#else
static void inline AX_WRITE (u16 data, void __iomem *mem)
{
	writew (data, mem);
}

static u16 inline AX_READ (void __iomem *mem)
{
	return readw (mem);
}
#endif

extern void dma_start (dma_addr_t mem_addr, int len, u8 tx);

#define AX_SELECT_PAGE(a,b)		AX_WRITE (((AX_READ (b) & 0xFFF8) | a) , b)

/* Tx header feilds mask */
#define TX_HDR_SOP_DICF			0x8000
#define TX_HDR_SOP_CPHI			0x4000
#define TX_HDR_SOP_INT			0x2000
#define TX_HDR_SOP_MDEQ			0x1000
#define TX_HDR_SOP_PKTLEN		0x07FF
#define TX_HDR_SOP_SEQNUM		0xF800
#define TX_HDR_SOP_PKTLENBAR		0x07FF

#define TX_HDR_SEG_FS			0x8000
#define TX_HDR_SEG_LS			0x4000
#define TX_HDR_SEG_SEGNUM		0x3800
#define TX_HDR_SEG_SEGLEN		0x0700
#define TX_HDR_SEG_EOFST		0xC000
#define TX_HDR_SEG_EOFBITS		14
#define TX_HDR_SEG_SOFST		0x3800
#define TX_HDR_SEG_SOFBITS		11
#define TX_HDR_SEG_SEGLENBAR		0x07FF


#define TX_HDR_EOP_SEQNUM		0xF800
#define TX_HDR_EOP_PKTLEN		0x07FF
#define TX_HDR_EOP_SEQNUMBAR		0xF800
#define TX_HDR_EOP_PKTLENBAR		0x07FF

/* Rx header fields mask */
#define RX_HDR1_MCBC			0x8000
#define RX_HDR1_STUFF_PKT		0x4000
#define RX_HDR1_MII_ERR			0x2000
#define RX_HDR1_CRC_ERR			0x1000
#define RX_HDR1_PKT_LEN			0x07FF
#define RX_HDR2_SEQ_NUM			0xF800
#define RX_HDR2_PKT_LEN_BAR		0x7FFF
#define RX_HDR3_CE			(1 << 15)
#define RX_HDR3_L3_PKT_TYPE(x)		(((x) >> 13) & 0x0003)
#define RX_HDR3_L4_PKT_TYPE(x)		(((x) >> 10) & 0x0007)
#define RX_HDR3_L3_ERR			(1 << 9)
#define RX_HDR3_L4_ERR			(1 << 8)
#define RX_HDR3_PRIORITY(x)		(((x) >> 4 ) & 0x0007 )
#define RX_HDR3_STRIP			(1 << 3)
#define RX_HDR3_VLAN_ID(x)		(x & 0x0007)

/* Tx/Rx buffer offset */
#define RX_PKT_DATA_OFFSET		6

/* AX88796C Pages index */
#define PAGE0				0x00
#define PAGE1				0x01
#define PAGE2				0x02
#define PAGE3				0x03
#define PAGE4				0x04
#define PAGE5				0x05
#define PAGE6				0x06
#define PAGE7				0x07

#define AX_MCAST_FILTER_SIZE		8
#define AX_MAX_MCAST			64


/* CIR bit */
#define Chip_REV_ID			0xF0
#define IRQ_POL_HIGH_ACTIVE		0x400

/*PHY*/
#define PHY_ID_MASK	0x3F
#define REG_NUM_MASK	0x1F
#define PHY_ID		0x10

/* General packet handling feature settings */
#define MAX_PKT_LEN			0x2000
#define AX88796C_IO_EXTENT		0x6800
#define ETHER_ADDR_LEN			6
#define DWORD_ALIGNMENT			0xFFFC
#define AX88796C_CHIP_ID                0x36
#define AX88796C_PAGE_SIZE              128	/* 1 page has 128 bytes */
#define AX88796C_TX_PAGES		32	/* Number of transmit packets */
#define AX88796C_PAGE_SHIFT             0x07	/* 1 page has 128 bytes */
/* Sturctures declaration */

/* Tx headers structure */
struct tx_sop_header {
	/* bit 15-11: flags, bit 10-0: packet length */
	u16 flags_pktlen;
	/* bit 15-11: sequence number, bit 11-0: packet length bar */
	u16 seqnum_pktlenbar;
} __attribute__((packed));

struct tx_segment_header {
	/* bit 15-14: flags, bit 13-11: segment number,
	   bit 10-0: segment length */
	u16 flags_seqnum_seglen;
	/* bit 15-14: end offset, bit 13-11: start offset */
	/* bit 10-0: segment length bar */
	u16 eo_so_seglenbar;
} __attribute__((packed));

struct tx_eop_header {
	/* bit 15-11: sequence number, bit 10-0: packet length */
	u16 seqnum_pktlen;
	/* bit 15-11: sequence number bar, bit 10-0: packet length bar */
	u16 seqnumbar_pktlenbar;
} __attribute__((packed));

struct tx_header {
	struct tx_sop_header sop;
	struct tx_segment_header seg;
} __attribute__((packed));

/* Rx headers structure */
struct rx_header {
	u16 flags_len;
	u16 seq_lenbar;
	u16 flags;
} __attribute__((packed));

struct skb_data {
	struct net_device *ndev;
	size_t len;
	size_t pages;
	size_t dma_len;
	dma_addr_t phy_addr;
	struct tx_header txhdr;
	struct tx_eop_header tx_eop;
	u8 offset;
};

enum watchdog_state {
	chk_link = 0,
	chk_cable,
	ax_nop,
};

struct ax88796c_device {
	struct net_device *ndev;
	void __iomem *membase;
	struct mii_if_info mii;
	struct net_device_stats stat;	/* The new statistics table. */

	spinlock_t isr_lock;
	struct timer_list watchdog;
	enum watchdog_state w_state;
	size_t w_ticks;

	/* Tx variables */
	u16	seq_num;
	struct sk_buff_head tx_q;
	struct sk_buff_head tx_busy_q;

	u8	burst_len;
		#define DMA_BURST_LEN_2_WORD	0x00
		#define DMA_BURST_LEN_4_WORD	0x01
		#define DMA_BURST_LEN_8_WORD	0x10
		#define DMA_BURST_LEN_16_WORD	0x11

	/* Rx variables */
	struct sk_buff_head rx_busy_q;
	unsigned rx_dmaing;

	/* Platform dependent variables */
	int (*low_level_output) (struct net_device *dev,
			struct sk_buff *skb, struct skb_data *entry);
	void (*tx_dma_complete) (void *dev_temp);
	int (*low_level_input) (struct net_device *dev);
	void (*rx_dma_complete) (void *dev_temp);

	u16	wol;

	u8	plat_endian;
		#define PLAT_LITTLE_ENDIAN	0
		#define PLAT_BIG_ENDIAN		1

	u8	multi_filter[AX_MCAST_FILTER_SIZE];

	u8	checksum;
		#define AX_RX_CHECKSUM		1
		#define AX_TX_CHECKSUM		2

	u8	ps_level;
		#define AX_PS_D0		0
		#define AX_PS_D1		1
		#define AX_PS_D2		2

	int	msg_enable;

};

/* A88796C register definition */
#if (AX88796B_PIN_COMPATIBLE)
#define AX_SHIFT(x)	((x) << 1)
#else
#define AX_SHIFT(x)	((x) << 0)
#endif

	/* Definition of PAGE0 */
#define PG_PSR		AX_SHIFT(0x00)
	#define PSR_BUS_TYPE(x)		((x >> 4) & 0x07)
	#define BUS_TYPE_8BIT_SRAM	0x00
	#define BUS_TYPE_8BIT_MULPLEX	0x01
	#define BUS_TYPE_16BIT_SRAM	0x04
	#define BUS_TYPE_16BIT_MULPLEX	0x05
	#define BUS_TYPE_16BIT_LOCALBUS	0x07
	#define PSR_DEV_READY		(1 << 7)
	#define PSR_RESET		(0 << 15)
	#define PSR_RESET_CLR		(1 << 15)
#define P0_BOR		AX_SHIFT(0x02)
#define P0_FER		AX_SHIFT(0x04)
	#define FER_IPALM		(1 << 0)
	#define FER_DCRC		(1 << 1)
	#define FER_RH3M		(1 << 2)
	#define FER_HEADERSWAP		(1 << 7)
	#define FER_WSWAP		(1 << 8)
	#define FER_BSWAP		(1 << 9)
	#define FER_INTHI		(1 << 10)
	#define FER_INTLO		(0 << 10)
	#define FER_IRQ_PULL		(1 << 11)
	#define FER_RXEN		(1 << 14)
	#define FER_TXEN		(1 << 15)
#define P0_ISR		AX_SHIFT(0x06)
	#define ISR_RXPKT		(1 << 0)
	#define ISR_MDQ			(1 << 4)
	#define ISR_TXT			(1 << 5)
	#define ISR_TXPAGES		(1 << 6)
	#define ISR_TXERR		(1 << 8)
	#define ISR_LINK		(1 << 9)
#define P0_IMR		AX_SHIFT(0x08)
	#define IMR_RXPKT		(1 << 0)
	#define IMR_MDQ			(1 << 4)
	#define IMR_TXT			(1 << 5)
	#define IMR_TXPAGES		(1 << 6)
	#define IMR_TXERR		(1 << 8)
	#define IMR_LINK		(1 << 9)
	#define IMR_MASKALL		(0xFFFF)
	#define IMR_DEFAULT		(IMR_TXERR)
#define P0_WFCR		AX_SHIFT(0x0A)
	#define WFCR_PMEIND		(1 << 0) /* PME indication */
	#define WFCR_PMETYPE		(1 << 1) /* PME I/O type */
	#define WFCR_PMEPOL		(1 << 2) /* PME polarity */
	#define WFCR_PMERST		(1 << 3) /* Reset PME */
	#define WFCR_SLEEP		(1 << 4) /* Enable sleep mode */
	#define WFCR_WAKEUP		(1 << 5) /* Enable wakeup mode */
	#define WFCR_WAITEVENT		(1 << 6) /* Reserved */
	#define WFCR_CLRWAKE		(1 << 7) /* Clear wakeup */
	#define WFCR_LINKCH		(1 << 8) /* Enable link change */
	#define WFCR_MAGICP		(1 << 9) /* Enable magic packet */
	#define WFCR_WAKEF		(1 << 10) /* Enable wakeup frame */
	#define WFCR_PMEEN		(1 << 11) /* Enable PME pin */
	#define WFCR_LINKCHS		(1 << 12) /* Link change status */
	#define WFCR_MAGICPS		(1 << 13) /* Magic packet status */
	#define WFCR_WAKEFS		(1 << 14) /* Wakeup frame status */
	#define WFCR_PMES		(1 << 15) /* PME pin status */
#define P0_PSCR		AX_SHIFT(0x0C)
	#define PSCR_PS_MASK		(0xFFF0)
	#define PSCR_PS_D0		(0)
	#define PSCR_PS_D1		(1 << 0)
	#define PSCR_PS_D2		(1 << 1)
	#define PSCR_FPS		(1 << 3) /* Enable fiber mode PS */
	#define PSCR_SWPS		(1 << 4) /* Enable software PS control */
	#define PSCR_WOLPS		(1 << 5) /* Enable WOL PS */
	#define PSCR_SWWOL		(1 << 6) /* Enable software select WOL PS */
	#define PSCR_PHYOSC		(1 << 7) /* Internal PHY OSC control */
	#define PSCR_FOFEF		(1 << 8) /* Force PHY generate FEF */
	#define PSCR_FOF		(1 << 9) /* Force PHY in fiber mode */
	#define PSCR_PHYPD		(1 << 10) /* PHY power down. Active high */
	#define PSCR_PHYRST		(1 << 11) /* PHY reset signal. Active low */
	#define PSCR_PHYCSIL		(1 << 12) /* PHY cable energy detect */
	#define PSCR_PHYCOFF		(1 << 13) /* PHY cable off */
	#define PSCR_PHYLINK		(1 << 14) /* PHY link status */
	#define PSCR_EEPOK		(1 << 15) /* EEPROM load complete */
#define P0_MACCR	AX_SHIFT(0x0E)
	#define MACCR_RXFC_ENABLE	(1 << 3)
	#define MACCR_RXFC_MASK		0xFFF7
	#define MACCR_TXFC_ENABLE	(1 << 4)
	#define MACCR_TXFC_MASK		0xFFEF
	#define MACCR_PF		(1 << 7)
	#define MACCR_PMM_BITS		8
	#define MACCR_PMM_MASK		(0x1F00)
	#define MACCR_PMM_RESET		(1 << 8)
	#define MACCR_PMM_WAIT		(2 << 8)
	#define MACCR_PMM_READY		(3 << 8)
	#define MACCR_PMM_D1		(4 << 8)
	#define MACCR_PMM_D2		(5 << 8)
	#define MACCR_PMM_WAKE		(7 << 8)
	#define MACCR_PMM_D1_WAKE	(8 << 8)
	#define MACCR_PMM_D2_WAKE	(9 << 8)
	#define MACCR_PMM_SLEEP		(10 << 8)
	#define MACCR_PMM_PHY_RESET	(11 << 8)
	#define MACCR_PMM_SOFT_D1	(16 << 8)
	#define MACCR_PMM_SOFT_D2	(17 << 8)
#define P0_TFBFCR	AX_SHIFT(0x10)
	#define TFBFCR_SCHE_FREE_PAGE 	0xE07F
	#define TFBFCR_FREE_PAGE_BITS	0x07
	#define TFBFCR_FREE_PAGE_LATCH	(1 << 6)
	#define TFBFCR_SET_FREE_PAGE(x)	((x & 0x3F) << TFBFCR_FREE_PAGE_BITS)
	#define TFBFCR_TX_PAGE_SET 	(1 << 13)
	#define TFBFCR_MANU_ENTX	(1 << 15)
	#define TX_FREEBUF_MASK		0x003F
	#define TX_DPTSTART		0x4000

#define P0_TSNR		AX_SHIFT(0x12)
	#define TXNR_TXB_ERR		(1 << 5)
	#define TXNR_TXB_IDLE		(1 << 6)
	#define TSNR_PKT_CNT(x)		(((x) & 0x3F) << 8)
	#define TXNR_TXB_REINIT		(1 << 14)
	#define TSNR_TXB_START		(1 << 15)
#define P0_RTDPR	AX_SHIFT(0x14)
#define P0_RXBCR1	AX_SHIFT(0x16)
	#define RXBCR1_RXB_DISCARD	(1 << 14)
	#define RXBCR1_RXB_START	(1 << 15)
#define P0_RXBCR2	AX_SHIFT(0x18)
	#define RXBCR2_PKT_MASK		(0xFF)
	#define RXBCR2_RXPC_MASK	(0x7F)
	#define RXBCR2_RXB_READY	(1 << 13)
	#define RXBCR2_RXB_IDLE		(1 << 14)
	#define RXBCR2_RXB_REINIT	(1 << 15)
#define P0_RTWCR	AX_SHIFT(0x1A)
	#define RTWCR_RXWC_MASK		(0x3FFF)
	#define RTWCR_RX_LATCH		(1 << 15)
#define P0_RCPHR	AX_SHIFT(0x1C)
#define PG_HOST_WAKEUP	AX_SHIFT(0x1F)

	/* Definition of PAGE1 */
#define P1_RPPER	AX_SHIFT(0x02)
	#define RPPER_RXEN		(1 << 0)
#define P1_MRCR		AX_SHIFT(0x08)
#define P1_MDR		AX_SHIFT(0x0A)
#define P1_RMPR		AX_SHIFT(0x0C)
#define P1_TMPR		AX_SHIFT(0x0E)
#define P1_RXBSPCR	AX_SHIFT(0x10)
	#define RXBSPCR_STUF_WORD_CNT(x)	(((x) & 0x7000) >> 12)
	#define RXBSPCR_STUF_ENABLE		(1 << 15)
#define P1_MCR		AX_SHIFT(0x12)
	#define MCR_SBP			(1 << 8)
	#define MCR_SM			(1 << 9)
	#define MCR_CRCENLAN		(1 << 11)
	#define MCR_STP			(1 << 12)
	/* Definition of PAGE2 */
#define P2_CIR		AX_SHIFT(0x02)
#define P2_POOLCR	AX_SHIFT(0x04)
	#define POOLCR_POLL_EN		(1 << 0)
	#define POOLCR_POLL_FLOWCTRL	(1 << 1)
	#define POOLCR_POLL_BMCR	(1 << 2)
	#define POOLCR_PHYID(x)		((x) << 8)
#define P2_PHYSR	AX_SHIFT(0x06)
#define P2_MDIODR	AX_SHIFT(0x08)
#define P2_MDIOCR	AX_SHIFT(0x0A)
	#define MDIOCR_RADDR(x)		((x) & 0x1F)
	#define MDIOCR_FADDR(x)		(((x) & 0x1F) << 8)
	#define MDIOCR_VALID		(1 << 13)
	#define MDIOCR_READ		(1 << 14)
	#define MDIOCR_WRITE		(1 << 15)
#define P2_LCR0		AX_SHIFT(0x0C)
	#define LCR_LED0_EN		(1 << 0)
	#define LCR_LED0_100MODE	(1 << 1)
	#define LCR_LED0_DUPLEX		(1 << 2)
	#define LCR_LED0_LINK		(1 << 3)
	#define LCR_LED0_ACT		(1 << 4)
	#define LCR_LED0_COL		(1 << 5)
	#define LCR_LED0_10MODE		(1 << 6)
	#define LCR_LED0_DUPCOL		(1 << 7)
	#define LCR_LED1_EN		(1 << 8)
	#define LCR_LED1_100MODE	(1 << 9)
	#define LCR_LED1_DUPLEX		(1 << 10)
	#define LCR_LED1_LINK		(1 << 11)
	#define LCR_LED1_ACT		(1 << 12)
	#define LCR_LED1_COL		(1 << 13)
	#define LCR_LED1_10MODE		(1 << 14)
	#define LCR_LED1_DUPCOL		(1 << 15)
#define P2_LCR1		AX_SHIFT(0x0E)
	#define LCR_LED2_MASK		(0xFF00)
	#define LCR_LED2_EN		(1 << 0)
	#define LCR_LED2_100MODE	(1 << 1)
	#define LCR_LED2_DUPLEX		(1 << 2)
	#define LCR_LED2_LINK		(1 << 3)
	#define LCR_LED2_ACT		(1 << 4)
	#define LCR_LED2_COL		(1 << 5)
	#define LCR_LED2_10MODE		(1 << 6)
	#define LCR_LED2_DUPCOL		(1 << 7)
#define P2_IPGCR	AX_SHIFT(0x10)
#define P2_FLHWCR	AX_SHIFT(0x14)
#define P2_RXCR		AX_SHIFT(0x16)
	#define RXCR_PRO		(1 << 0)
	#define RXCR_AMALL		(1 << 1)
	#define RXCR_SEP		(1 << 2)
	#define RXCR_AB			(1 << 3)
	#define RXCR_AM			(1 << 4)
	#define RXCR_AP			(1 << 5)
	#define RXCR_ARP		(1 << 6)
#define P2_JLCR		AX_SHIFT(0x18)
#define P2_MPLR		AX_SHIFT(0x1C)

	/* Definition of PAGE3 */
#define P3_MACASR0	AX_SHIFT(0x02)
	#define P3_MACASR(x)		(P3_MACASR0 + 2*x)
	#define MACASR_LOWBYTE_MASK 	0x00FF
	#define MACASR_HIGH_BITS	0x08
#define P3_MACASR1	AX_SHIFT(0x04)
#define P3_MACASR2	AX_SHIFT(0x06)
#define P3_MFAR01	AX_SHIFT(0x08)
#define P3_MFAR_BASE	AX_SHIFT(0x08)
	#define P3_MFAR(x)		(P3_MFAR_BASE + 2*x)

#define P3_MFAR23	AX_SHIFT(0x0A)
#define P3_MFAR45	AX_SHIFT(0x0C)
#define P3_MFAR67	AX_SHIFT(0x0E)
#define P3_VID0FR	AX_SHIFT(0x10)
#define P3_VID1FR	AX_SHIFT(0x12)
#define P3_EECSR	AX_SHIFT(0x14)
#define P3_EEDR		AX_SHIFT(0x16)
#define P3_EECR		AX_SHIFT(0x18)
	#define EECR_ADDR_MASK		(0x00FF)
	#define EECR_READ_ACT		(1 << 8)
	#define EECR_WRITE_ACT		(1 << 9)
	#define EECR_WRITE_DISABLE	(1 << 10)
	#define EECR_WRITE_ENABLE	(1 << 11)
	#define EECR_EE_READY		(1 << 13)
	#define EECR_RELOAD		(1 << 14)
	#define EECR_RESET		(1 << 15)
#define P3_TPCR		AX_SHIFT(0x1A)
	#define TPCR_PATT_MASK		(0xFF)
	#define TPCR_RAND_PKT_EN	(1 << 14)
	#define TPCR_FIXED_PKT_EN	(1 << 15)
#define P3_TPLR		AX_SHIFT(0x1C)
	/* Definition of PAGE4 */
#define P4_COERCR0	AX_SHIFT(0x12)
	#define COERCR0_RXIPCE		(1 << 0)
	#define COERCR0_RXIPVE		(1 << 1)
	#define COERCR0_RXV6PE		(1 << 2)
	#define COERCR0_RXTCPE		(1 << 3)
	#define COERCR0_RXUDPE		(1 << 4)
	#define COERCR0_RXICMP		(1 << 5)
	#define COERCR0_RXIGMP		(1 << 6)
	#define COERCR0_RXICV6		(1 << 7)

	#define COERCR0_RXTCPV6		(1 << 8)
	#define COERCR0_RXUDPV6		(1 << 9)
	#define COERCR0_RXICMV6		(1 << 10)
	#define COERCR0_RXIGMV6		(1 << 11)
	#define COERCR0_RXICV6V6	(1 << 12)

	#define COERCR0_DEFAULT		(COERCR0_RXIPCE | COERCR0_RXV6PE | \
					 COERCR0_RXTCPE | COERCR0_RXUDPE | \
					 COERCR0_RXTCPV6 | COERCR0_RXUDPV6)
#define P4_COERCR1	AX_SHIFT(0x14)
	#define COERCR1_IPCEDP		(1 << 0)
	#define COERCR1_IPVEDP		(1 << 1)
	#define COERCR1_V6VEDP		(1 << 2)
	#define COERCR1_TCPEDP		(1 << 3)
	#define COERCR1_UDPEDP		(1 << 4)
	#define COERCR1_ICMPDP		(1 << 5)
	#define COERCR1_IGMPDP		(1 << 6)
	#define COERCR1_ICV6DP		(1 << 7)
	#define COERCR1_RX64TE		(1 << 8)
	#define COERCR1_RXPPPE		(1 << 9)
	#define COERCR1_TCP6DP		(1 << 10)
	#define COERCR1_UDP6DP		(1 << 11)
	#define COERCR1_IC6DP		(1 << 12)
	#define COERCR1_IG6DP		(1 << 13)
	#define COERCR1_ICV66DP		(1 << 14)
	#define COERCR1_RPCE		(1 << 15)

	#define COERCR1_DEFAULT		(COERCR1_RXPPPE)
#define P4_COETCR0	AX_SHIFT(0x16)
	#define COETCR0_TXIP		(1 << 0)
	#define COETCR0_TXTCP		(1 << 1)
	#define COETCR0_TXUDP		(1 << 2)
	#define COETCR0_TXICMP		(1 << 3)
	#define COETCR0_TXIGMP		(1 << 4)
	#define COETCR0_TXICV6		(1 << 5)
	#define COETCR0_TXTCPV6		(1 << 8)
	#define COETCR0_TXUDPV6		(1 << 9)
	#define COETCR0_TXICMV6		(1 << 10)
	#define COETCR0_TXIGMV6		(1 << 11)
	#define COETCR0_TXICV6V6	(1 << 12)

	#define COETCR0_DEFAULT		(COETCR0_TXIP | COETCR0_TXTCP | \
					 COETCR0_TXUDP | COETCR0_TXTCPV6 | \
					 COETCR0_TXUDPV6)
#define P4_COETCR1	AX_SHIFT(0x18)
	#define COETCR1_TX64TE		(1 << 0)
	#define COETCR1_TXPPPE		(1 << 1)

#define P4_COECEDR	AX_SHIFT(0x1A)
#define P4_L2CECR	AX_SHIFT(0x1C)

	/* Definition of PAGE5 */
#define P5_WFTR		AX_SHIFT(0x02)
	#define WFTR_2MS		(0x01)
	#define WFTR_4MS		(0x02)
	#define WFTR_8MS		(0x03)
	#define WFTR_16MS		(0x04)
	#define WFTR_32MS		(0x05)
	#define WFTR_64MS		(0x06)
	#define WFTR_128MS		(0x07)
	#define WFTR_256MS		(0x08)
	#define WFTR_512MS		(0x09)
	#define WFTR_1024MS		(0x0A)
	#define WFTR_2048MS		(0x0B)
	#define WFTR_4096MS		(0x0C)
	#define WFTR_8192MS		(0x0D)
	#define WFTR_16384MS		(0x0E)
	#define WFTR_32768MS		(0x0F)
#define P5_WFCCR	AX_SHIFT(0x04)
#define P5_WFCR03	AX_SHIFT(0x06)
	#define WFCR03_F0_EN		(1 << 0)
	#define WFCR03_F1_EN		(1 << 4)
	#define WFCR03_F2_EN		(1 << 8)
	#define WFCR03_F3_EN		(1 << 12)
#define P5_WFCR47	AX_SHIFT(0x08)
	#define WFCR47_F4_EN		(1 << 0)
	#define WFCR47_F5_EN		(1 << 4)
	#define WFCR47_F6_EN		(1 << 8)
	#define WFCR47_F7_EN		(1 << 12)
#define P5_WF0BMR0	AX_SHIFT(0x0A)
#define P5_WF0BMR1	AX_SHIFT(0x0C)
#define P5_WF0CR	AX_SHIFT(0x0E)
#define P5_WF0OBR	AX_SHIFT(0x10)
#define P5_WF1BMR0	AX_SHIFT(0x12)
#define P5_WF1BMR1	AX_SHIFT(0x14)
#define P5_WF1CR	AX_SHIFT(0x16)
#define P5_WF1OBR	AX_SHIFT(0x18)
#define P5_WF2BMR0	AX_SHIFT(0x1A)
#define P5_WF2BMR1	AX_SHIFT(0x1C)

	/* Definition of PAGE6 */
#define P6_WF2CR	AX_SHIFT(0x02)
#define P6_WF2OBR	AX_SHIFT(0x04)
#define P6_WF3BMR0	AX_SHIFT(0x06)
#define P6_WF3BMR1	AX_SHIFT(0x08)
#define P6_WF3CR	AX_SHIFT(0x0A)
#define P6_WF3OBR	AX_SHIFT(0x0C)
#define P6_WF4BMR0	AX_SHIFT(0x0E)
#define P6_WF4BMR1	AX_SHIFT(0x10)
#define P6_WF4CR	AX_SHIFT(0x12)
#define P6_WF4OBR	AX_SHIFT(0x14)
#define P6_WF5BMR0	AX_SHIFT(0x16)
#define P6_WF5BMR1	AX_SHIFT(0x18)
#define P6_WF5CR	AX_SHIFT(0x1A)
#define P6_WF5OBR	AX_SHIFT(0x1C)

/* Definition of PAGE7 */
#define P7_WF6BMR0	AX_SHIFT(0x02)
#define P7_WF6BMR1	AX_SHIFT(0x04)
#define P7_WF6CR	AX_SHIFT(0x06)
#define P7_WF6OBR	AX_SHIFT(0x08)
#define P7_WF7BMR0	AX_SHIFT(0x0A)
#define P7_WF7BMR1	AX_SHIFT(0x0C)
#define P7_WF7CR	AX_SHIFT(0x0E)
#define P7_WF7OBR	AX_SHIFT(0x10)
#define P7_WFR01	AX_SHIFT(0x12)
#define P7_WFR23	AX_SHIFT(0x14)
#define P7_WFR45	AX_SHIFT(0x16)
#define P7_WFR67	AX_SHIFT(0x18)
#define P7_WFPC0	AX_SHIFT(0x1A)
#define P7_WFPC1	AX_SHIFT(0x1C)

#endif /* _AX796C_H_ */
