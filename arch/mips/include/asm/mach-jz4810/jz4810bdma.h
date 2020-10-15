/*
 * linux/include/asm-mips/mach-jz4810/jz4810bdma.h
 *
 * JZ4810 BDMA register definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ4810BDMA_H__
#define __JZ4810BDMA_H__


#define BDMAC_BASE  0xB3450000


/*************************************************************************
 * BDMAC (BCH & NAND DMA Controller)
 *************************************************************************/

/* n is the DMA channel index (0 - 2) */
#define BDMAC_DSAR(n)		(BDMAC_BASE + (0x00 + (n) * 0x20)) /* DMA source address */
#define BDMAC_DTAR(n)  		(BDMAC_BASE + (0x04 + (n) * 0x20)) /* DMA target address */
#define BDMAC_DTCR(n)  		(BDMAC_BASE + (0x08 + (n) * 0x20)) /* DMA transfer count */
#define BDMAC_DRSR(n)  		(BDMAC_BASE + (0x0c + (n) * 0x20)) /* DMA request source */
#define BDMAC_DCCSR(n) 		(BDMAC_BASE + (0x10 + (n) * 0x20)) /* DMA control/status */
#define BDMAC_DCMD(n)  		(BDMAC_BASE + (0x14 + (n) * 0x20)) /* DMA command */
#define BDMAC_DDA(n)   		(BDMAC_BASE + (0x18 + (n) * 0x20)) /* DMA descriptor address */
#define BDMAC_DSD(n)   		(BDMAC_BASE + (0x1c + (n) * 0x20)) /* DMA Stride Address */
#define BDMAC_DNT(n)  		(BDMAC_BASE + (0xc0 + (n) * 0x04)) /* NAND Detect Timer */

#define BDMAC_DMACR		(BDMAC_BASE + 0x0300) 	/* DMA control register */
#define BDMAC_DMAIPR		(BDMAC_BASE + 0x0304) 	/* DMA interrupt pending */
#define BDMAC_DMADBR		(BDMAC_BASE + 0x0308) 	/* DMA doorbell */
#define BDMAC_DMADBSR		(BDMAC_BASE + 0x030C) 	/* DMA doorbell set */
#define BDMAC_DMACKE  		(BDMAC_BASE + 0x0310)
#define BDMAC_DMACKES  		(BDMAC_BASE + 0x0314)
#define BDMAC_DMACKEC  		(BDMAC_BASE + 0x0318)

#define REG_BDMAC_DSAR(n)	REG32(BDMAC_DSAR((n)))
#define REG_BDMAC_DTAR(n)	REG32(BDMAC_DTAR((n)))
#define REG_BDMAC_DTCR(n)	REG32(BDMAC_DTCR((n)))
#define REG_BDMAC_DRSR(n)	REG32(BDMAC_DRSR((n)))
#define REG_BDMAC_DCCSR(n)	REG32(BDMAC_DCCSR((n)))
#define REG_BDMAC_DCMD(n)	REG32(BDMAC_DCMD((n)))
#define REG_BDMAC_DDA(n)	REG32(BDMAC_DDA((n)))
#define REG_BDMAC_DSD(n)    REG32(BDMAC_DSD(n))
#define REG_BDMAC_DNT(n)	REG32(BDMAC_DNT(n))

#define REG_BDMAC_DMACR		REG32(BDMAC_DMACR)
#define REG_BDMAC_DMAIPR	REG32(BDMAC_DMAIPR)
#define REG_BDMAC_DMADBR	REG32(BDMAC_DMADBR)
#define REG_BDMAC_DMADBSR	REG32(BDMAC_DMADBSR)
#define REG_BDMAC_DMACKE    REG32(BDMAC_DMACKE)
#define REG_BDMAC_DMACKES    REG32(BDMAC_DMACKES)
#define REG_BDMAC_DMACKEC    REG32(BDMAC_DMACKEC)

//BDMA nand detect timer register
#define BDMAC_DNTR_DNTE          (1 << 15)  /* Nand request 0 detect timer enable */
#define BDMAC_DNTR_DNT(n)        ((n) << 0) /* Nand request 0 detect timer value */
#define BDMAC_DNTR_DNTE1          (1 << 31)  /* Nand request 1 detect timer enable */
#define BDMAC_DNTR_DNT1(n)        ((n) << 23) /* Nand request 1 detect timer value */


// BDMA request source register
#define BDMAC_DRSR_RS_BIT	0
#define BDMAC_DRSR_RS_MASK	(0x3f << DMAC_DRSR_RS_BIT)
#define BDMAC_DRSR_RS_BCH_ENC	(2 << DMAC_DRSR_RS_BIT)
#define BDMAC_DRSR_RS_BCH_DEC	(3 << DMAC_DRSR_RS_BIT)
#define BDMAC_DRSR_RS_NAND0	(6 << DMAC_DRSR_RS_BIT)
#define BDMAC_DRSR_RS_NAND1	(7 << DMAC_DRSR_RS_BIT)
#define BDMAC_DRSR_RS_AUTO	(8 << DMAC_DRSR_RS_BIT)
#define BDMAC_DRSR_RS_EXT	(12 << DMAC_DRSR_RS_BIT)

// BDMA channel control/status register
#define BDMAC_DCCSR_NDES	(1 << 31) /* descriptor (0) or not (1) ? */
#define BDMAC_DCCSR_DES8    	(1 << 30) /* Descriptor 8 Word */
#define BDMAC_DCCSR_DES4    	(0 << 30) /* Descriptor 4 Word */
#define BDMAC_DCCSR_LASTMD0    	(0 << 28) /* BCH Decoding last mode 0, there's one descriptor for decoding blcok*/
#define BDMAC_DCCSR_LASTMD1    	(1 << 28) /* BCH Decoding last mode 1, there's two descriptor for decoding blcok*/
#define BDMAC_DCCSR_LASTMD2    	(2 << 28) /* BCH Decoding last mode 2, there's three descriptor for decoding blcok*/
#define BDMAC_DCCSR_FRBS(n)	((n) << 24)
#define BDMAC_DCCSR_CDOA_BIT	16        /* copy of DMA offset address */
#define BDMAC_DCCSR_CDOA_MASK	(0xff << BDMACC_DCCSR_CDOA_BIT)
#define BDMAC_DCCSR_BERR	(0x1f << 7)  /* BCH error within this transfer, Only for channel 0 */
#define BDMAC_DCCSR_BUERR       (1 << 5)  /* BCH uncorrectable error, only for channel 0 */
#define BDMAC_DCCSR_NSERR       (1 << 5)  /* status error, only for channel 1 */
#define BDMAC_DCCSR_AR		(1 << 4)  /* address error */
#define BDMAC_DCCSR_TT		(1 << 3)  /* transfer terminated */
#define BDMAC_DCCSR_HLT		(1 << 2)  /* DMA halted */
#define BDMAC_DCCSR_BAC		(1 << 1)  /* BCH auto correction */
#define BDMAC_DCCSR_EN		(1 << 0)  /* channel enable bit */

// BDMA channel command register
#define BDMAC_DCMD_EACKS_LOW  	(1 << 31) /* External DACK Output Level Select, active low */
#define BDMAC_DCMD_EACKS_HIGH  	(0 << 31) /* External DACK Output Level Select, active high */
#define BDMAC_DCMD_EACKM_WRITE 	(1 << 30) /* External DACK Output Mode Select, output in write cycle */
#define BDMAC_DCMD_EACKM_READ 	(0 << 30) /* External DACK Output Mode Select, output in read cycle */
#define BDMAC_DCMD_ERDM_BIT	28        /* External DREQ Detection Mode Select */
  #define BDMAC_DCMD_ERDM_MASK	(0x03 << BDMAC_DCMD_ERDM_BIT)
  #define BDMAC_DCMD_ERDM_LOW	(0 << BDMAC_DCMD_ERDM_BIT)
  #define BDMAC_DCMD_ERDM_FALL	(1 << BDMAC_DCMD_ERDM_BIT)
  #define BDMAC_DCMD_ERDM_HIGH	(2 << BDMAC_DCMD_ERDM_BIT)
  #define BDMAC_DCMD_ERDM_RISE	(3 << BDMAC_DCMD_ERDM_BIT)
#define BDMAC_DCMD_BLAST	(1 << 25) /* BCH last */
#define BDMAC_DCMD_SAI		(1 << 23) /* source address increment */
#define BDMAC_DCMD_DAI		(1 << 22) /* dest address increment */
#define BDMAC_DCMD_SWDH_BIT	14  /* source port width */
  #define BDMAC_DCMD_SWDH_MASK	(0x03 << BDMAC_DCMD_SWDH_BIT)
  #define BDMAC_DCMD_SWDH_32	(0 << BDMAC_DCMD_SWDH_BIT)
  #define BDMAC_DCMD_SWDH_8	(1 << BDMAC_DCMD_SWDH_BIT)
  #define BDMAC_DCMD_SWDH_16	(2 << BDMAC_DCMD_SWDH_BIT)
#define BDMAC_DCMD_DWDH_BIT	12  /* dest port width */
  #define BDMAC_DCMD_DWDH_MASK	(0x03 << BDMAC_DCMD_DWDH_BIT)
  #define BDMAC_DCMD_DWDH_32	(0 << BDMAC_DCMD_DWDH_BIT)
  #define BDMAC_DCMD_DWDH_8	(1 << BDMAC_DCMD_DWDH_BIT)
  #define BDMAC_DCMD_DWDH_16	(2 << BDMAC_DCMD_DWDH_BIT)
#define BDMAC_DCMD_DS_BIT	8  /* transfer data size of a data unit */
  #define BDMAC_DCMD_DS_MASK	(0x07 << BDMAC_DCMD_DS_BIT)
  #define BDMAC_DCMD_DS_32BIT	(0 << BDMAC_DCMD_DS_BIT)
  #define BDMAC_DCMD_DS_8BIT	(1 << BDMAC_DCMD_DS_BIT)
  #define BDMAC_DCMD_DS_16BIT	(2 << BDMAC_DCMD_DS_BIT)
  #define BDMAC_DCMD_DS_16BYTE	(3 << BDMAC_DCMD_DS_BIT)
  #define BDMAC_DCMD_DS_32BYTE	(4 << BDMAC_DCMD_DS_BIT)
  #define BDMAC_DCMD_DS_64BYTE	(5 << BDMAC_DCMD_DS_BIT)
#define BDMAC_DCMD_NRD   	(1 << 7)  /* NAND direct read */
#define BDMAC_DCMD_NWR   	(1 << 6)  /* NAND direct write */
#define BDMAC_DCMD_NAC   	(1 << 5)  /* NAND AL/CL enable */
#define BDMAC_DCMD_NSTA		(1 << 4)  /* Nand Status Transfer Enable */
#define BDMAC_DCMD_STDE   	(1 << 2)  /* Stride Disable/Enable */
#define BDMAC_DCMD_TIE		(1 << 1)  /* DMA transfer interrupt enable */
#define BDMAC_DCMD_LINK		(1 << 0)  /* descriptor link enable */

// BDMA descriptor address register
#define BDMAC_DDA_BASE_BIT	12  /* descriptor base address */
  #define BDMAC_DDA_BASE_MASK	(0x0fffff << BDMAC_DDA_BASE_BIT)
#define BDMAC_DDA_OFFSET_BIT	4   /* descriptor offset address */
  #define BDMAC_DDA_OFFSET_MASK	(0x0ff << BDMAC_DDA_OFFSET_BIT)

// BDMA stride address register
#define BDMAC_DSD_TSD_BIT	16	/* target stride address */
  #define BDMAC_DSD_TSD_MASK	(0xffff << BDMAC_DSD_TSD_BIT)
#define BDMAC_DSD_SSD_BIT	0	/* source stride address */
  #define BDMAC_DSD_SSD_MASK	(0xffff << BDMAC_DSD_SSD_BIT)

// BDMA NAND Detect timer register
#define BDMAC_NDTCTIMER_EN	(1 << 15)  /* enable detect timer */
#define BDMAC_TAILCNT_BIT	16

// BDMA control register
#define BDMAC_DMACR_PR_BIT	8	/* channel priority mode */
  #define BDMAC_DMACR_PR_MASK	(0x03 << DMAC_DMACR_PR_BIT)
  #define BDMAC_DMACR_PR_01_2	(0 << BDMAC_DMACR_PR_BIT)
  #define BDMAC_DMACR_PR_12_0	(1 << BDMAC_DMACR_PR_BIT)
  #define BDMAC_DMACR_PR_20_1	(2 << BDMAC_DMACR_PR_BIT)
  #define BDMAC_DMACR_PR_012	(3 << BDMAC_DMACR_PR_BIT)
#define BDMAC_DMACR_HLT		(1 << 3)  /* DMA halt flag */
#define BDMAC_DMACR_AR		(1 << 2)  /* address error flag */
#define BDMAC_DMACR_DMAE	(1 << 0)  /* DMA enable bit */

// BDMA interrupt pending register
#define BDMAC_DMAIPR_CIRQ2	(1 << 2)  /* irq pending status for channel 2 */
#define BDMAC_DMAIPR_CIRQ1	(1 << 1)  /* irq pending status for channel 1 */
#define BDMAC_DMAIPR_CIRQ0	(1 << 0)  /* irq pending status for channel 0 */

// BDMA doorbell register
#define BDMAC_DMADBR_DB2	(1 << 2)  /* doorbell for channel 2 */
#define BDMAC_DMADBR_DB1	(1 << 1)  /* doorbell for channel 1 */
#define BDMAC_DMADBR_DB0	(1 << 0)  /* doorbell for channel 0 */

// BDMA doorbell set register
#define BDMAC_DMADBSR_DBS2	(1 << 2)  /* enable doorbell for channel 2 */
#define BDMAC_DMADBSR_DBS1	(1 << 1)  /* enable doorbell for channel 1 */
#define BDMAC_DMADBSR_DBS0	(1 << 0)  /* enable doorbell for channel 0 */


#ifndef __MIPS_ASSEMBLER


/***************************************************************************
 * BCH & NAND DMAC
 ***************************************************************************/

/* n is the DMA channel index (0 - 2) */

#define __bdmac_test_halt_error ( REG_BDMAC_DMACR & BDMAC_DMACR_HLT )
#define __bdmac_test_addr_error ( REG_BDMAC_DMACR & BDMAC_DMACR_AR )

#define __bdmac_channel_enable_clk(n)           \
	REG_BDMAC_DMACKE |= 1 << (n);

#define __bdmac_enable_descriptor(n) \
  ( REG_BDMAC_DCCSR((n)) &= ~BDMAC_DCCSR_NDES )
#define __bdmac_disable_descriptor(n) \
  ( REG_BDMAC_DCCSR((n)) |= BDMAC_DCCSR_NDES )

#define __bdmac_enable_channel(n)                 \
do {                                             \
	REG_BDMAC_DCCSR((n)) |= BDMAC_DCCSR_EN;    \
} while (0)
#define __bdmac_disable_channel(n)                \
do {                                             \
	REG_BDMAC_DCCSR((n)) &= ~BDMAC_DCCSR_EN;   \
} while (0)

#define __bdmac_channel_enable_irq(n) \
  ( REG_BDMAC_DCMD((n)) |= BDMAC_DCMD_TIE )
#define __bdmac_channel_disable_irq(n) \
  ( REG_BDMAC_DCMD((n)) &= ~BDMAC_DCMD_TIE )

#define __bdmac_channel_transmit_halt_detected(n) \
  (  REG_BDMAC_DCCSR((n)) & BDMAC_DCCSR_HLT )
#define __bdmac_channel_transmit_end_detected(n) \
  (  REG_BDMAC_DCCSR((n)) & BDMAC_DCCSR_TT )
/* Nand ops status error, only for channel 1 */
#define __bdmac_channel_status_error_detected() \
  (  REG_BDMAC_DCCSR(1) & BDMAC_DCCSR_NSERR )
#define __bdmac_channel_address_error_detected(n)	\
  (  REG_BDMAC_DCCSR((n)) & BDMAC_DCCSR_AR )
#define __bdmac_channel_count_terminated_detected(n) \
  (  REG_BDMAC_DCCSR((n)) & BDMAC_DCCSR_CT )
#define __bdmac_channel_descriptor_invalid_detected(n) \
  (  REG_BDMAC_DCCSR((n)) & BDMAC_DCCSR_INV )
#define __bdmac_BCH_error_detected(n) \
  (  REG_BDMAC_DCCSR((n)) & BDMAC_DCCSR_BERR )

#define __bdmac_channel_clear_transmit_halt(n)				\
	do {								\
		/* clear both channel halt error and globle halt error */ \
		REG_BDMAC_DCCSR(n) &= ~BDMAC_DCCSR_HLT;			\
		REG_BDMAC_DMACR &= ~BDMAC_DMACR_HLT;	\
	} while (0)
#define __bdmac_channel_clear_transmit_end(n) \
  (  REG_BDMAC_DCCSR(n) &= ~BDMAC_DCCSR_TT )
#define __bdmac_channel_clear_status_error() \
  ( REG_BDMAC_DCCSR(1) &= ~BDMAC_DCCSR_NSERR )
#define __bdmac_channel_clear_address_error(n)				\
	do {								\
		REG_BDMAC_DDA(n) = 0; /* clear descriptor address register */ \
		REG_BDMAC_DSAR(n) = 0; /* clear source address register */ \
		REG_BDMAC_DTAR(n) = 0; /* clear target address register */ \
		/* clear both channel addr error and globle address error */ \
		REG_BDMAC_DCCSR(n) &= ~BDMAC_DCCSR_AR;			\
		REG_BDMAC_DMACR &= ~BDMAC_DMACR_AR;	\
	} while (0)
#define __bdmac_channel_clear_count_terminated(n) \
  (  REG_BDMAC_DCCSR((n)) &= ~BDMAC_DCCSR_CT )
#define __bdmac_channel_clear_descriptor_invalid(n) \
  (  REG_BDMAC_DCCSR((n)) &= ~BDMAC_DCCSR_INV )

#define __bdmac_channel_set_transfer_unit_32bit(n)	\
do {							\
	REG_BDMAC_DCMD((n)) &= ~BDMAC_DCMD_DS_MASK;	\
	REG_BDMAC_DCMD((n)) |= BDMAC_DCMD_DS_32BIT;	\
} while (0)

#define __bdmac_channel_set_transfer_unit_16bit(n)	\
do {							\
	REG_BDMAC_DCMD((n)) &= ~BDMAC_DCMD_DS_MASK;	\
	REG_BDMAC_DCMD((n)) |= BDMAC_DCMD_DS_16BIT;	\
} while (0)

#define __bdmac_channel_set_transfer_unit_8bit(n)	\
do {							\
	REG_BDMAC_DCMD((n)) &= ~BDMAC_DCMD_DS_MASK;	\
	REG_BDMAC_DCMD((n)) |= BDMAC_DCMD_DS_8BIT;	\
} while (0)

#define __bdmac_channel_set_transfer_unit_16byte(n)	\
do {							\
	REG_BDMAC_DCMD((n)) &= ~BDMAC_DCMD_DS_MASK;	\
	REG_BDMAC_DCMD((n)) |= BDMAC_DCMD_DS_16BYTE;	\
} while (0)

#define __bdmac_channel_set_transfer_unit_32byte(n)	\
do {							\
	REG_BDMAC_DCMD((n)) &= ~BDMAC_DCMD_DS_MASK;	\
	REG_BDMAC_DCMD((n)) |= BDMAC_DCMD_DS_32BYTE;	\
} while (0)

/* w=8,16,32 */
#define __bdmac_channel_set_dest_port_width(n,w)		\
do {							\
	REG_BDMAC_DCMD((n)) &= ~BDMAC_DCMD_DWDH_MASK;	\
	REG_BDMAC_DCMD((n)) |= BDMAC_DCMD_DWDH_##w;	\
} while (0)

/* w=8,16,32 */
#define __bdmac_channel_set_src_port_width(n,w)		\
do {							\
	REG_BDMAC_DCMD((n)) &= ~BDMAC_DCMD_SWDH_MASK;	\
	REG_BDMAC_DCMD((n)) |= BDMAC_DCMD_SWDH_##w;	\
} while (0)

#define __bdmac_channel_dest_addr_fixed(n) \
	(REG_BDMAC_DCMD((n)) &= ~BDMAC_DCMD_DAI)
#define __bdmac_channel_dest_addr_increment(n) \
	(REG_BDMAC_DCMD((n)) |= BDMAC_DCMD_DAI)

#define __bdmac_channel_src_addr_fixed(n) \
	(REG_BDMAC_DCMD((n)) &= ~BDMAC_DCMD_SAI)
#define __bdmac_channel_src_addr_increment(n) \
	(REG_BDMAC_DCMD((n)) |= BDMAC_DCMD_SAI)

#define __bdmac_channel_set_doorbell(n)	\
	(REG_BDMAC_DMADBSR = (1 << (n)))

#define __bdmac_channel_irq_detected(n)  (REG_BDMAC_DMAIPR & (1 << (n)))
#define __bdmac_channel_ack_irq(n)       (REG_BDMAC_DMAIPR &= ~(1 <<(n)))

static __inline__ int __bdmac_get_irq(void)
{
	int i;
	for (i = 0; i < MAX_BDMA_NUM; i++)
		if (__bdmac_channel_irq_detected(i))
			return i;
	return -1;
}

#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4810BDMA_H__ */

