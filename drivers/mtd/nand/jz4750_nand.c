/*
 * linux/drivers/mtd/nand/jz4750_nand.c
 *
 * JZ4750 NAND driver
 *
 * Copyright (c) 2005 - 2007 Ingenic Semiconductor Inc.
 * Author: <lhhuang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include <asm/jzsoc.h>
#include "mtd/mtd_bch4bit_n8.h"

/* 32bit instead of 16byte burst is used by DMA to read or
   write NAND and BCH avoiding grabbing bus for too long */
#define DMAC_DCMD_DS_NAND    DMAC_DCMD_DS_32BIT
#define DIV_DS_NAND          4

#define DMAC_DCMD_DS_BCH     DMAC_DCMD_DS_32BIT
#define DIV_DS_BCH           4

#define DEBUG1        0
#if DEBUG1
#define	dprintk(n,x...) printk(n,##x)
#else
#define	dprintk(n,x...)
#endif

#if defined(CONFIG_MTD_HW_BCH_8BIT)
#define __ECC_ENCODING __ecc_encoding_8bit
#define __ECC_DECODING __ecc_decoding_8bit
#define ERRS_SIZE       5	/* 5 words */
#else
#define __ECC_ENCODING __ecc_encoding_4bit
#define __ECC_DECODING __ecc_decoding_4bit
#define ERRS_SIZE       3	/* 3 words */
#endif

#define NAND_DATA_PORT1	       0xB8000000	/* read-write area in static bank 1 */
#define NAND_DATA_PORT2	       0xB4000000	/* read-write area in static bank 2 */
#define NAND_DATA_PORT3	       0xAC000000	/* read-write area in static bank 3 */
#define NAND_DATA_PORT4	       0xA8000000	/* read-write area in static bank 4 */

#define NAND_ADDR_OFFSET0       0x00010000      /* address port offset for share mode */
#define NAND_CMD_OFFSET0        0x00008000      /* command port offset for share mode */
#define NAND_ADDR_OFFSET1       0x00000010      /* address port offset for unshare mode */
#define NAND_CMD_OFFSET1        0x00000008      /* command port offset for unshare mode */

#if defined(CONFIG_MTD_NAND_DMA)
#define  USE_IRQ      1
enum {
	NAND_NONE,
	NAND_PROG,
	NAND_READ
};
static volatile u8 nand_status;
static volatile int dma_ack = 0;
static volatile int dma_ack1 = 0;
static char nand_dma_chan;	/* automatically select a free channel */
static char bch_dma_chan = 0;	/* fixed to channel 0 */
static u32 *errs;
static jz_dma_desc_8word *dma_desc_enc, *dma_desc_enc1, *dma_desc_dec, *dma_desc_dec1, *dma_desc_dec2,
	*dma_desc_nand_prog, *dma_desc_nand_read;
static u32 *pval_nand_ddr;
static u8 *pval_nand_cmd_pgprog; /* for sending 0x11 or 0x10 when programing*/
#if defined(CONFIG_MTD_NAND_DMABUF)
u8 *prog_buf, *read_buf;
#endif
DECLARE_WAIT_QUEUE_HEAD(nand_prog_wait_queue);
DECLARE_WAIT_QUEUE_HEAD(nand_read_wait_queue);
#endif

struct buf_be_corrected {
	u8 *data;
	u8 *oob;
};

static u32 addr_offset;
static u32 cmd_offset;
static int nand_chips = 1;  /* Number of nand chips to be scanned */
extern int global_page; /* for two-plane operations */
extern int global_mafid; /* ID of manufacture */

/*
 * MTD structure for JzSOC board
 */
static struct mtd_info *jz_mtd = NULL;
extern struct mtd_info *jz_mtd1;
extern char all_use_planes;

int nr_partitions; /* Number of partitions */

/*
 * Define partitions for flash devices
 */
#if defined(CONFIG_JZ4750_FUWA) || defined(CONFIG_JZ4750D_FUWA1)
struct mtd_partition partition_info[] = {
	{name:"NAND BOOT partition",
	 offset:0 * 0x100000,
	 size:4 * 0x100000,
	 cpu_mode: 1,
	 use_planes: 0,
	 mtdblock_jz_invalid: 1},
	{name:"NAND KERNEL partition",
	 offset:4 * 0x100000,
	 size:4 * 0x100000,
	 cpu_mode: 1,
	 use_planes: 0,
	 mtdblock_jz_invalid: 1},
	{name:"NAND ROOTFS partition",
	 offset:8 * 0x100000,
	 size:120 * 0x100000,
	 cpu_mode: 1,
	 use_planes: 0,
	 mtdblock_jz_invalid: 1},
	{name:"NAND DATA1 partition",
	 offset:128 * 0x100000,
	 size:128 * 0x100000,
	 cpu_mode: 1,
	 use_planes: 1,
	 mtdblock_jz_invalid: 1},
	{name:"NAND DATA2 partition",
	 offset:256 * 0x100000,
	 size:256 * 0x100000,
	 cpu_mode: 1,
	 use_planes: 1,
	 mtdblock_jz_invalid: 1},
	{name:"NAND VFAT partition",
	 offset:512 * 0x100000,
	 size:512 * 0x100000,
	 cpu_mode: 0,
	 use_planes: 1,
	 mtdblock_jz_invalid: 0},
};

/* Define max reserved bad blocks for each partition.
 * This is used by the mtdblock-jz.c NAND FTL driver only.
 *
 * The NAND FTL driver reserves some good blocks which can't be
 * seen by the upper layer. When the bad block number of a partition
 * exceeds the max reserved blocks, then there is no more reserved
 * good blocks to be used by the NAND FTL driver when another bad
 * block generated.
 */
static int partition_reserved_badblocks[] = {
	2,			/* reserved blocks of mtd0 */
	2,			/* reserved blocks of mtd1 */
	10,			/* reserved blocks of mtd2 */
	10,			/* reserved blocks of mtd3 */
	20,			/* reserved blocks of mtd4 */
	20
};				/* reserved blocks of mtd5 */
#endif				/* CONFIG_JZ4750_FUWA */

#if defined(CONFIG_JZ4750_APUS) || defined(CONFIG_JZ4750D_CETUS) || defined(CONFIG_JZ4750L_TAURUS)
struct mtd_partition partition_info[] = {
	{name:"NAND BOOT partition",
	 offset:0 * 0x100000LL,
	 size:4 * 0x100000LL,
	 cpu_mode: 0,
	 use_planes: 0,
	 mtdblock_jz_invalid: 1},
	{name:"NAND KERNEL partition",
	 offset:4 * 0x100000LL,
	 size:4 * 0x100000LL,
	 cpu_mode: 0,
	 use_planes: 0,
	 mtdblock_jz_invalid: 1},
	{name:"NAND ROOTFS partition",
	 offset:8 * 0x100000LL,
	 size:504 * 0x100000LL,
	 cpu_mode: 1,
	 use_planes: 0,
	 mtdblock_jz_invalid: 1},
	{name:"NAND DATA partition",
	 offset:512 * 0x100000LL,
	 size:512 * 0x100000LL,
	 cpu_mode: 0,
	 use_planes: 1,
	 mtdblock_jz_invalid: 1},
	{name:"NAND VFAT partition",
	 offset:1024 * 0x100000LL,
	 size:1024 * 0x100000LL,
	 cpu_mode: 0,
	 use_planes: 1,
	 mtdblock_jz_invalid: 0},
};


/* Define max reserved bad blocks for each partition.
 * This is used by the mtdblock-jz.c NAND FTL driver only.
 *
 * The NAND FTL driver reserves some good blocks which can't be
 * seen by the upper layer. When the bad block number of a partition
 * exceeds the max reserved blocks, then there is no more reserved
 * good blocks to be used by the NAND FTL driver when another bad
 * block generated.
 */
static int partition_reserved_badblocks[] = {
	2,			/* reserved blocks of mtd0 */
	2,			/* reserved blocks of mtd1 */
	10,			/* reserved blocks of mtd2 */
	10,			/* reserved blocks of mtd3 */
	10,			/* reserved blocks of mtd4 */
};
#endif				/* CONFIG_JZ4750_APUS */

/*-------------------------------------------------------------------------
 * Following three functions are exported and used by the mtdblock-jz.c
 * NAND FTL driver only.
 */

unsigned short get_mtdblock_write_verify_enable(void)
{
#ifdef CONFIG_MTD_MTDBLOCK_WRITE_VERIFY_ENABLE
	return 1;
#endif
	return 0;
}

EXPORT_SYMBOL(get_mtdblock_write_verify_enable);

unsigned short get_mtdblock_oob_copies(void)
{
	return CONFIG_MTD_OOB_COPIES;
}

EXPORT_SYMBOL(get_mtdblock_oob_copies);

int *get_jz_badblock_table(void)
{
	if (sizeof(partition_reserved_badblocks) / sizeof(int) !=  nr_partitions)
		printk("partition_reserved_badblocks setting error!\n");

	return partition_reserved_badblocks;
}

EXPORT_SYMBOL(get_jz_badblock_table);

/*-------------------------------------------------------------------------*/

static void jz_hwcontrol(struct mtd_info *mtd, int dat, u32 ctrl)
{
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	u32 nandaddr = (u32)this->IO_ADDR_W;
	extern u8 nand_nce;  /* defined in nand_base.c, indicates which chip select is used for current nand chip */

	if (ctrl & NAND_CTRL_CHANGE) {
		if (ctrl & NAND_NCE) {
			switch (nand_nce) {
			case NAND_NCE1:
				this->IO_ADDR_W = this->IO_ADDR_R = (void __iomem *)NAND_DATA_PORT1;
				REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE2;
				REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE3;
				REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE4;
				REG_EMC_NFCSR |= EMC_NFCSR_NFCE1;
				break;
			case NAND_NCE2:
				this->IO_ADDR_W = this->IO_ADDR_R = (void __iomem *)NAND_DATA_PORT2;
				REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE1;
				REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE3;
				REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE4;
				REG_EMC_NFCSR |= EMC_NFCSR_NFCE2;
				break;
			case NAND_NCE3:
				this->IO_ADDR_W = this->IO_ADDR_R = (void __iomem *)NAND_DATA_PORT3;
				REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE1;
				REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE2;
				REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE4;
				REG_EMC_NFCSR |= EMC_NFCSR_NFCE3;
				break;
			case NAND_NCE4:
				this->IO_ADDR_W = this->IO_ADDR_R = (void __iomem *)NAND_DATA_PORT4;
				REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE1;
				REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE2;
				REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE3;
				REG_EMC_NFCSR |= EMC_NFCSR_NFCE4;
				break;
			default:
				printk("error: no nand_nce 0x%x\n",nand_nce);
				break;
			}
		} else {
			REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE1;
			REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE2;
			REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE3;
			REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE4;
		}

		if (ctrl & NAND_ALE)
			nandaddr = (u32)((u32)(this->IO_ADDR_W) | addr_offset);
		else
			nandaddr = (u32)((u32)(this->IO_ADDR_W) & ~addr_offset);
		if (ctrl & NAND_CLE)
			nandaddr = (u32)(nandaddr | cmd_offset);
		else
			nandaddr = (u32)(nandaddr & ~cmd_offset);
	}

	this->IO_ADDR_W = (void __iomem *)nandaddr;
	if (dat != NAND_CMD_NONE) {
		writeb(dat, this->IO_ADDR_W);
		/* printk("write cmd:0x%x to 0x%x\n",dat,(u32)this->IO_ADDR_W); */
	}
}

static int jz_device_ready(struct mtd_info *mtd)
{
	int ready, wait = 10;
	while (wait--);
	ready = __gpio_get_pin(91);
	return ready;
}

/*
 * EMC setup
 */
static void jz_device_setup(void)
{
// PORT 0:
// PORT 1:
// PORT 2:
// PIN/BIT N		FUNC0		FUNC1
//	21		CS1#		-
//	22		CS2#		-
//	23		CS3#		-
//	24		CS4#		-
#define GPIO_CS2_N (32*2+22)
#define GPIO_CS3_N (32*2+23)
#define GPIO_CS4_N (32*2+24)
#define SMCR_VAL   0x0d444400
//#define SMCR_VAL   0x05221100

	__gpio_as_nand_8bit(1);
	/* Set NFE bit */
	REG_EMC_NFCSR |= EMC_NFCSR_NFE1;
	/* Read/Write timings */
	REG_EMC_SMCR1 = SMCR_VAL;

#if defined(CONFIG_MTD_NAND_CS2)
	__gpio_as_func0(GPIO_CS2_N);
	/* Set NFE bit */
	REG_EMC_NFCSR |= EMC_NFCSR_NFE2;
	/* Read/Write timings */
	REG_EMC_SMCR2 = SMCR_VAL;
	nand_chips++;
#endif

#if defined(CONFIG_MTD_NAND_CS3)
	__gpio_as_func0(GPIO_CS3_N);
	/* Set NFE bit */
	REG_EMC_NFCSR |= EMC_NFCSR_NFE3;
	/* Read/Write timings */
	REG_EMC_SMCR3 = SMCR_VAL;
	nand_chips++;
#endif

#if defined(CONFIG_MTD_NAND_CS4)
	__gpio_as_func0(GPIO_CS4_N);
	/* Set NFE bit */
	REG_EMC_NFCSR |= EMC_NFCSR_NFE4;
	/* Read/Write timings */
	REG_EMC_SMCR4 = SMCR_VAL;
	nand_chips++;
#endif
}

#ifdef CONFIG_MTD_HW_BCH_ECC

static void jzsoc_nand_enable_bch_hwecc(struct mtd_info *mtd, int mode)
{
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	int eccsize = this->ecc.size;
	int eccbytes = this->ecc.bytes;
	int eccsteps = this->ecc.steps / this->planenum;
	int oob_per_eccsize = this->ecc.layout->eccpos[0] / eccsteps;

	REG_BCH_INTS = 0xffffffff;
	if (mode == NAND_ECC_READ) {
		__ECC_DECODING();
		__ecc_cnt_dec(eccsize + oob_per_eccsize + eccbytes);

		if (!(mtd->flags & MTD_NAND_CPU_MODE))
			__ecc_dma_enable();
		else
			__ecc_dma_disable();
	}

	if (mode == NAND_ECC_WRITE) {
		__ECC_ENCODING();
		__ecc_cnt_enc(eccsize + oob_per_eccsize);

		if (!(mtd->flags & MTD_NAND_CPU_MODE))
			__ecc_dma_enable();
		else
			__ecc_dma_disable();
	}
}

/**
 * bch_correct
 * @dat:        data to be corrected
 * @idx:        the index of error bit in an eccsize
 */
static void bch_correct(struct mtd_info *mtd, u8 * dat, int idx)
{
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	int eccsize = this->ecc.size;
	int eccsteps = this->ecc.steps / this->planenum;
	int ecc_pos = this->ecc.layout->eccpos[0];
	int oob_per_eccsize = ecc_pos / eccsteps;
	int i, bit;		/* the 'bit' of i byte is error */

	i = (idx - 1) >> 3;
	bit = (idx - 1) & 0x7;

	dprintk("error:i=%d, bit=%d\n",i,bit);

	if (i < eccsize){
		((struct buf_be_corrected *)dat)->data[i] ^= (1 << bit);
	} else if (i < eccsize + oob_per_eccsize) {
		((struct buf_be_corrected *)dat)->oob[i-eccsize] ^= (1 << bit);
	}
}

#if defined(CONFIG_MTD_NAND_DMA)

/**
 * jzsoc_nand_bch_correct_data
 * @mtd:	mtd info structure
 * @dat:        data to be corrected
 * @errs0:      pointer to the dma target buffer of bch decoding which stores BHINTS and
 *              BHERR0~3(8-bit BCH) or BHERR0~1(4-bit BCH)
 * @calc_ecc:   no used
 */
static int jzsoc_nand_bch_correct_data(struct mtd_info *mtd, u_char * dat, u_char * errs0, u_char * calc_ecc)
{
	u32 stat;
	u32 *errs = (u32 *)errs0;

	if (REG_DMAC_DCCSR(0) & DMAC_DCCSR_BERR) {
		stat = errs[0];
		dprintk("stat=%x err0:%x err1:%x \n", stat, errs[1], errs[2]);

		if (stat & BCH_INTS_ERR) {
			if (stat & BCH_INTS_UNCOR) {
				printk("NAND: Uncorrectable ECC error\n");
				return -1;
			} else {
				u32 errcnt = (stat & BCH_INTS_ERRC_MASK) >> BCH_INTS_ERRC_BIT;
				switch (errcnt) {
#if defined(CONFIG_MTD_HW_BCH_8BIT)
				case 8:
					bch_correct(mtd, dat, (errs[4] & BCH_ERR_INDEX_ODD_MASK) >> BCH_ERR_INDEX_ODD_BIT);
				case 7:
					bch_correct(mtd, dat, (errs[4] & BCH_ERR_INDEX_EVEN_MASK) >> BCH_ERR_INDEX_EVEN_BIT);
				case 6:
					bch_correct(mtd, dat, (errs[3] & BCH_ERR_INDEX_ODD_MASK) >> BCH_ERR_INDEX_ODD_BIT);
				case 5:
					bch_correct(mtd, dat, (errs[3] & BCH_ERR_INDEX_EVEN_MASK) >> BCH_ERR_INDEX_EVEN_BIT);
#endif
				case 4:
					bch_correct(mtd, dat, (errs[2] & BCH_ERR_INDEX_ODD_MASK) >> BCH_ERR_INDEX_ODD_BIT);
				case 3:
					bch_correct(mtd, dat, (errs[2] & BCH_ERR_INDEX_EVEN_MASK) >> BCH_ERR_INDEX_EVEN_BIT);
				case 2:
					bch_correct(mtd, dat, (errs[1] & BCH_ERR_INDEX_ODD_MASK) >> BCH_ERR_INDEX_ODD_BIT);
				case 1:
					bch_correct(mtd, dat, (errs[1] & BCH_ERR_INDEX_EVEN_MASK) >> BCH_ERR_INDEX_EVEN_BIT);
				default:
					break;
				}
			}
		}
	}

	return 0;
}

#endif				/* CONFIG_MTD_NAND_DMA */

/**
 * jzsoc_nand_bch_correct_data_cpu
 * @mtd:	mtd info structure
 * @dat:        data to be corrected
 * @read_ecc:   pointer to ecc buffer calculated when nand writing
 * @calc_ecc:   no used
 */
static int jzsoc_nand_bch_correct_data_cpu(struct mtd_info *mtd, u_char * dat, u_char * read_ecc, u_char * calc_ecc)
{
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	int eccsize = this->ecc.size;
	int eccbytes = this->ecc.bytes;
	int eccsteps = this->ecc.steps / this->planenum;
	int ecc_pos = this->ecc.layout->eccpos[0];
	int oob_per_eccsize = ecc_pos / eccsteps;
	short k;
	u32 stat;

	/* Write data to REG_BCH_DR */
	for (k = 0; k < eccsize; k++) {
		REG_BCH_DR = ((struct buf_be_corrected *)dat)->data[k];
	}
	/* Write oob to REG_BCH_DR */
	for (k = 0; k < oob_per_eccsize; k++) {
		REG_BCH_DR = ((struct buf_be_corrected *)dat)->oob[k];
	}
	/* Write parities to REG_BCH_DR */
	for (k = 0; k < eccbytes; k++) {
		REG_BCH_DR = read_ecc[k];
	}

	/* Wait for completion */
	__ecc_decode_sync();
	__ecc_disable();

	/* Check decoding */
	stat = REG_BCH_INTS;

	if (stat & BCH_INTS_ERR) {
		/* Error occurred */
		if (stat & BCH_INTS_UNCOR) {
			printk("NAND: Uncorrectable ECC error--\n");
			return -1;
		} else {
			u32 errcnt = (stat & BCH_INTS_ERRC_MASK) >> BCH_INTS_ERRC_BIT;
			switch (errcnt) {
#if defined(CONFIG_MTD_HW_BCH_8BIT)
			case 8:
				bch_correct(mtd, dat, (REG_BCH_ERR3 & BCH_ERR_INDEX_ODD_MASK) >> BCH_ERR_INDEX_ODD_BIT);
				/* FALL-THROUGH */
			case 7:
				bch_correct(mtd, dat, (REG_BCH_ERR3 & BCH_ERR_INDEX_EVEN_MASK) >> BCH_ERR_INDEX_EVEN_BIT);
				/* FALL-THROUGH */
			case 6:
				bch_correct(mtd, dat, (REG_BCH_ERR2 & BCH_ERR_INDEX_ODD_MASK) >> BCH_ERR_INDEX_ODD_BIT);
				/* FALL-THROUGH */
			case 5:
				bch_correct(mtd, dat, (REG_BCH_ERR2 & BCH_ERR_INDEX_EVEN_MASK) >> BCH_ERR_INDEX_EVEN_BIT);
				/* FALL-THROUGH */
#endif
			case 4:
				bch_correct(mtd, dat, (REG_BCH_ERR1 & BCH_ERR_INDEX_ODD_MASK) >> BCH_ERR_INDEX_ODD_BIT);
				/* FALL-THROUGH */
			case 3:
				bch_correct(mtd, dat, (REG_BCH_ERR1 & BCH_ERR_INDEX_EVEN_MASK) >> BCH_ERR_INDEX_EVEN_BIT);
				/* FALL-THROUGH */
			case 2:
				bch_correct(mtd, dat, (REG_BCH_ERR0 & BCH_ERR_INDEX_ODD_MASK) >> BCH_ERR_INDEX_ODD_BIT);
				/* FALL-THROUGH */
			case 1:
				bch_correct(mtd, dat, (REG_BCH_ERR0 & BCH_ERR_INDEX_EVEN_MASK) >> BCH_ERR_INDEX_EVEN_BIT);
				return 0;
			default:
				break;
			}
		}
	}

	return 0;
}


static int jzsoc_nand_calculate_bch_ecc(struct mtd_info *mtd, const u_char * dat, u_char * ecc_code)
{
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	int eccsize = this->ecc.size;
	int eccbytes = this->ecc.bytes;
	int eccsteps = this->ecc.steps / this->planenum;
	int ecc_pos = this->ecc.layout->eccpos[0];
	int oob_per_eccsize = ecc_pos / eccsteps;
	volatile u8 *paraddr = (volatile u8 *)BCH_PAR0;
	short i;

	/* Write data to REG_BCH_DR */
	for (i = 0; i < eccsize; i++) {
		REG_BCH_DR = ((struct buf_be_corrected *)dat)->data[i];
	}
	/* Write oob to REG_BCH_DR */
	for (i = 0; i < oob_per_eccsize; i++) {
		REG_BCH_DR = ((struct buf_be_corrected *)dat)->oob[i];
	}
	__ecc_encode_sync();
	__ecc_disable();

	for (i = 0; i < eccbytes; i++) {
		ecc_code[i] = *paraddr++;
	}

	return 0;
}

extern int nand_sw_bch_ops(struct mtd_info *mtd, u8 *oobdata, int ops);

#if defined(CONFIG_MTD_NAND_DMA)

/**
 * nand_write_page_hwecc_bch - [REPLACABLE] hardware ecc based page write function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	data buffer
 */
static void nand_write_page_hwecc_bch0(struct mtd_info *mtd, struct nand_chip *chip, const uint8_t * buf, u8 cmd_pgprog)
{
	int eccsize = chip->ecc.size;
	int eccsteps = chip->ecc.steps / chip->planenum;
	int eccbytes = chip->ecc.bytes;
	int ecc_pos = chip->ecc.layout->eccpos[0];
	int oob_per_eccsize = ecc_pos / eccsteps;
	int pagesize = mtd->writesize / chip->planenum;
	int oobsize = mtd->oobsize / chip->planenum;
	int i, err, timeout;
	const u8 *databuf;
	u8 *oobbuf;
	jz_dma_desc_8word *desc;

	nand_sw_bch_ops(mtd, chip->oob_poi, 1);

#if defined(CONFIG_MTD_NAND_DMABUF)
	memcpy(prog_buf, buf, pagesize);
	memcpy(prog_buf + pagesize, chip->oob_poi, oobsize);
	dma_cache_wback_inv((u32)prog_buf, pagesize + oobsize);
#else
	databuf = buf;
	oobbuf = chip->oob_poi;

	/* descriptors for encoding data blocks */
	desc = dma_desc_enc;
	for (i = 0; i < eccsteps; i++) {
		desc->dsadr = CPHYSADDR((u32)databuf) + i * eccsize;	/* DMA source address */
		desc->dtadr = CPHYSADDR((u32)oobbuf) + ecc_pos + i * eccbytes;	/* DMA target address */
		dprintk("dma_desc_enc:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr);
		desc++;
	}

	/* descriptors for encoding oob blocks */
	desc = dma_desc_enc1;
	for (i = 0; i < eccsteps; i++) {
		desc->dsadr = CPHYSADDR((u32)oobbuf) + oob_per_eccsize * i;	/* DMA source address, 28/4 = 7bytes */
		desc->dtadr = CPHYSADDR((u32)oobbuf) + ecc_pos + i * eccbytes;	/* DMA target address */
		dprintk("dma_desc_enc1:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr);
		desc++;
	}

	/* descriptor for nand programing data block */
	desc = dma_desc_nand_prog;
	desc->dsadr = CPHYSADDR((u32)databuf);	/* DMA source address */
	desc->dtadr = CPHYSADDR((u32)chip->IO_ADDR_W);  /* It will be changed when using multiply chip select */
	dprintk("dma_desc_nand_prog:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
	       desc->ddadr);

	/* descriptor for nand programing oob block */
	desc++;
	desc->dsadr = CPHYSADDR((u32)oobbuf);	/* DMA source address */
	desc->dtadr = CPHYSADDR((u32)chip->IO_ADDR_W);  /* It will be changed when using multiply chip select */
	dprintk("dma_desc_oob_prog:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
	       desc->ddadr);

	/* descriptor for __nand_cmd(CMD_PGPROG) */
	desc++;
	*pval_nand_cmd_pgprog = cmd_pgprog;
	desc->dsadr = CPHYSADDR((u32)pval_nand_cmd_pgprog);
	desc->dtadr = CPHYSADDR((u32)chip->IO_ADDR_R | cmd_offset);	/* DMA target address: cmdport */
	if (cmd_pgprog == 0x10)
		desc->dcmd |= DMAC_DCMD_LINK;  /* __nand_sync() by a DMA descriptor */
	else if (cmd_pgprog == 0x11)
		desc->dcmd &= ~DMAC_DCMD_LINK; /* __nand_sync() by polling */

	dma_cache_wback_inv((u32)dma_desc_enc, (eccsteps * 2 + 2 + 1) * (sizeof(jz_dma_desc_8word)));
	dma_cache_wback_inv((u32)databuf, pagesize);
	dma_cache_wback_inv((u32)oobbuf, oobsize);
	/* 4*6: pval_nand_ddr, pval_nand_dcs, pval_bch_ddr, pval_bch_dcs, dummy, pval_nand_cmd_pgprog */
	dma_cache_wback_inv((u32)pval_nand_ddr, 4 * 8); /* 8 words, a cache line */
#endif

	REG_DMAC_DCCSR(bch_dma_chan) = 0;
	REG_DMAC_DCCSR(nand_dma_chan) = 0;

	/* Setup DMA descriptor address */
	REG_DMAC_DDA(bch_dma_chan) = CPHYSADDR((u32)dma_desc_enc);
	REG_DMAC_DDA(nand_dma_chan) = CPHYSADDR((u32)dma_desc_nand_prog);

	/* Setup request source */
	REG_DMAC_DRSR(bch_dma_chan) = DMAC_DRSR_RS_BCH_ENC;
	REG_DMAC_DRSR(nand_dma_chan) = DMAC_DRSR_RS_AUTO;

	/* Setup DMA channel control/status register */
	REG_DMAC_DCCSR(bch_dma_chan) = DMAC_DCCSR_DES8 | DMAC_DCCSR_EN;	/* descriptor transfer, clear status, start channel */

	/* Enable DMA */
	REG_DMAC_DMACR(0) |= DMAC_DMACR_DMAE;
	REG_DMAC_DMACR(nand_dma_chan/HALF_DMA_NUM) |= DMAC_DMACR_DMAE;

	/* Enable BCH encoding */
	chip->ecc.hwctl(mtd, NAND_ECC_WRITE);

	dma_ack1 = 0;
	nand_status = NAND_PROG;

	/* DMA doorbell set -- start DMA now ... */
	__dmac_channel_set_doorbell(bch_dma_chan);

#if USE_IRQ
	if (cmd_pgprog == 0x10) {
		dprintk("nand prog before wake up\n");
		do {
			err = wait_event_interruptible_timeout(nand_prog_wait_queue, dma_ack1, 3 * HZ);
		}while(err == -ERESTARTSYS);

		nand_status = NAND_NONE;
		dprintk("nand prog after wake up\n");
		if (!err) {
			printk("*** NAND WRITE, Warning, wait event 3s timeout!\n");
			dump_jz_dma_channel(0);
			dump_jz_dma_channel(nand_dma_chan);
			printk("REG_BCH_CR=%x REG_BCH_CNT=0x%x REG_BCH_INTS=%x\n", REG_BCH_CR, REG_BCH_CNT, REG_BCH_INTS);
		}
		dprintk("timeout remain = %d\n", err);
	} else if (cmd_pgprog == 0x11) {
		timeout = 100000;
		while ((!__dmac_channel_transmit_end_detected(nand_dma_chan)) && (timeout--));
		if (timeout <= 0)
			printk("two-plane prog 0x11 timeout!\n");
	}
#else
	timeout = 100000;
	while ((!__dmac_channel_transmit_end_detected(nand_dma_chan)) && (timeout--));
	while(!chip->dev_ready(mtd));
	if (timeout <= 0)
		printk("not use irq, prog timeout!\n");
#endif
}

static void nand_write_page_hwecc_bch(struct mtd_info *mtd, struct nand_chip *chip, const uint8_t * buf)
{
	nand_write_page_hwecc_bch0(mtd, chip, buf, 0x10);
}

static void nand_write_page_hwecc_bch_planes(struct mtd_info *mtd, struct nand_chip *chip, const uint8_t * buf)
{
	int page;
	int pagesize = mtd->writesize >> 1;
	int ppb = mtd->erasesize / mtd->writesize;

	page = (global_page / ppb) * ppb + global_page; /* = global_page%ppb + (global_page/ppb)*ppb*2 */

	/* send cmd 0x80, the MSB should be valid if realplane is 4 */
	if (chip->realplanenum == 2)
	{
		if(global_mafid == 0x2c)
			chip->cmdfunc(mtd, 0x80, 0x00, page);
		else
			chip->cmdfunc(mtd, 0x80, 0x00, 0x00);
	}
	else
		chip->cmdfunc(mtd, 0x80, 0x00, page & (1 << (chip->chip_shift - chip->page_shift)));

	nand_write_page_hwecc_bch0(mtd, chip, buf, 0x11);
	chip->cmdfunc(mtd, 0x81, 0x00, page + ppb);
	nand_write_page_hwecc_bch0(mtd, chip, buf + pagesize, 0x10);
}

#endif				/* CONFIG_MTD_NAND_DMA */

static void nand_write_page_hwecc_bch_cpu(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf)
{
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps / chip->planenum;
	int oob_per_eccsize = chip->ecc.layout->eccpos[0] / eccsteps;
	int oobsize = mtd->oobsize / chip->planenum;
	int ecctotal = chip->ecc.total / chip->planenum;
        uint8_t *p = (uint8_t *)buf;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint32_t *eccpos = chip->ecc.layout->eccpos;
	static struct buf_be_corrected buf_calc0;
	struct buf_be_corrected *buf_calc = &buf_calc0;
	
	nand_sw_bch_ops(mtd, chip->oob_poi, 1);
	
	for (i = 0; i < eccsteps; i++, p += eccsize) {
		buf_calc->data = (u8 *)buf + eccsize * i;
		buf_calc->oob = chip->oob_poi + oob_per_eccsize * i;
		chip->ecc.hwctl(mtd, NAND_ECC_WRITE);
		chip->ecc.calculate(mtd, (u8 *)buf_calc, &ecc_calc[eccbytes*i]);
		chip->write_buf(mtd, p, eccsize);
	}

	for (i = 0; i < ecctotal; i++)
		chip->oob_poi[eccpos[i]] = ecc_calc[i];

	chip->write_buf(mtd, chip->oob_poi, oobsize);
}

/* nand write using two-plane mode with cpu mode */
static void nand_write_page_hwecc_bch_planes_cpu(struct mtd_info *mtd, struct nand_chip *chip,
					     const uint8_t *buf)
{
	int pagesize = mtd->writesize >> 1;
	int ppb = mtd->erasesize / mtd->writesize;
	int page;

	page = (global_page / ppb) * ppb + global_page; /* = global_page%ppb + (global_page/ppb)*ppb*2 */

	/* send cmd 0x80, the MSB should be valid if realplane is 4 */
	if (chip->realplanenum == 2)
	{
		if(global_mafid == 0x2c)
			chip->cmdfunc(mtd, 0x80, 0x00, page);
		else
			chip->cmdfunc(mtd, 0x80, 0x00, 0x00);
	}
	else
		chip->cmdfunc(mtd, 0x80, 0x00, page & (1 << (chip->chip_shift - chip->page_shift)));

	nand_write_page_hwecc_bch_cpu(mtd, chip, buf);

	chip->cmdfunc(mtd, 0x11, -1, -1); /* send cmd 0x11 */
	ndelay(100);
	while(!chip->dev_ready(mtd));

	chip->cmdfunc(mtd, 0x81, 0x00, page + ppb); /* send cmd 0x81 */
	nand_write_page_hwecc_bch_cpu(mtd, chip, buf + pagesize);
}

/**
 * nand_read_page_hwecc_bch - [REPLACABLE] hardware ecc based page read function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 *
 * Not for syndrome calculating ecc controllers which need a special oob layout
 */
#if defined(CONFIG_MTD_NAND_DMA)
static int nand_read_page_hwecc_bch0(struct mtd_info *mtd, struct nand_chip *chip, uint8_t * buf, u32 page)
{
	int i, eccsize = chip->ecc.size;
	int eccsteps = chip->ecc.steps / chip->planenum;
	int eccbytes = chip->ecc.bytes;
	int ecc_pos = chip->ecc.layout->eccpos[0];
	int oob_per_eccsize = ecc_pos / eccsteps;
	int pagesize = mtd->writesize / chip->planenum;
	int oobsize = mtd->oobsize / chip->planenum;
	u8 *databuf, *oobbuf;
	jz_dma_desc_8word *desc;
	int err;
	u32 addrport, cmdport;
	static struct buf_be_corrected buf_correct0;

	addrport = (u32)(chip->IO_ADDR_R) | addr_offset;
 	cmdport = (u32)(chip->IO_ADDR_R) | cmd_offset;

#if defined(CONFIG_MTD_NAND_DMABUF)
	databuf = read_buf;
	oobbuf = read_buf + pagesize;

	dma_cache_inv((u32)read_buf, pagesize + oobsize);	// databuf should be invalidated.
	memset(errs, 0, eccsteps * ERRS_SIZE * 4);
	dma_cache_wback_inv((u32)errs, eccsteps * ERRS_SIZE * 4);
#else

	databuf = buf;
	oobbuf = chip->oob_poi;

	/* descriptor for nand reading data block */
	desc = dma_desc_nand_read;
	desc->dsadr = CPHYSADDR((u32)chip->IO_ADDR_R); /* It will be changed when using multiply chip select */
	desc->dtadr = CPHYSADDR((u32)databuf);	/* DMA target address */

	dprintk("desc_nand_read:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
	       desc->ddadr);

	/* descriptor for nand reading oob block */
	desc++;
	desc->dsadr = CPHYSADDR((u32)chip->IO_ADDR_R); /* It will be changed when using multiply chip select */
	desc->dtadr = CPHYSADDR((u32)oobbuf);	/* DMA target address */
	dprintk("desc_oob_read:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
	       desc->ddadr);

	/* descriptors for data to be written to bch */
	desc = dma_desc_dec;
	for (i = 0; i < eccsteps; i++) {
		desc->dsadr = CPHYSADDR((u32)databuf) + i * eccsize;	/* DMA source address */
		dprintk("dma_desc_dec:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr);
		desc++;
	}

	/* descriptors for oob to be written to bch */
	desc = dma_desc_dec1;
	for (i = 0; i < eccsteps; i++) {
		desc->dsadr = CPHYSADDR((u32)oobbuf) + oob_per_eccsize * i;	/* DMA source address */
		dprintk("dma_desc_dec1:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr);
		desc++;
	}

	/* descriptors for parities to be written to bch */
	desc = dma_desc_dec2;
	for (i = 0; i < eccsteps; i++) {
		desc->dsadr = CPHYSADDR((u32)oobbuf) + ecc_pos + i * eccbytes;	/* DMA source address */
		dprintk("dma_desc_dec2:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr);
		desc++;
	}

	dma_cache_wback_inv((u32)dma_desc_nand_read, (2 + eccsteps * 3) * (sizeof(jz_dma_desc_8word)));

	memset(errs, 0, eccsteps * ERRS_SIZE * 4);
	dma_cache_inv((u32)databuf, pagesize);	// databuf should be invalidated.
	dma_cache_inv((u32)oobbuf, oobsize);	// oobbuf should be invalidated too
	dma_cache_wback_inv((u32)errs, eccsteps * ERRS_SIZE * 4);
#endif
	REG_DMAC_DCCSR(bch_dma_chan) = 0;
	REG_DMAC_DCCSR(nand_dma_chan) = 0;

	/* Setup DMA descriptor address */
	REG_DMAC_DDA(nand_dma_chan) = CPHYSADDR((u32)dma_desc_nand_read);
	REG_DMAC_DDA(bch_dma_chan) = CPHYSADDR((u32)dma_desc_dec);

	/* Setup request source */
	REG_DMAC_DRSR(nand_dma_chan) = DMAC_DRSR_RS_NAND;
	REG_DMAC_DRSR(bch_dma_chan) = DMAC_DRSR_RS_BCH_DEC;

	/* Enable DMA */
	REG_DMAC_DMACR(0) |= DMAC_DMACR_DMAE;
	REG_DMAC_DMACR(nand_dma_chan/HALF_DMA_NUM) |= DMAC_DMACR_DMAE;

	/* Enable BCH decoding */
	chip->ecc.hwctl(mtd, NAND_ECC_READ);

	dma_ack = 0;
	nand_status = NAND_READ;
	/* DMA doorbell set -- start nand DMA now ... */
	__dmac_channel_set_doorbell(nand_dma_chan);

	/* Setup DMA channel control/status register */
	REG_DMAC_DCCSR(nand_dma_chan) = DMAC_DCCSR_DES8 | DMAC_DCCSR_EN;

#define __nand_cmd(n)		(REG8(cmdport) = (n))
#define __nand_addr(n)		(REG8(addrport) = (n))

	__nand_cmd(NAND_CMD_READ0);

	__nand_addr(0);
	if (pagesize != 512)
		__nand_addr(0);

	__nand_addr(page & 0xff);
	__nand_addr((page >> 8) & 0xff);

	/* One more address cycle for the devices whose number of page address bits > 16  */
	if (((chip->chipsize >> chip->page_shift) >> 16) - 1 > 0)
		__nand_addr((page >> 16) & 0xff);

	if (pagesize != 512)
		__nand_cmd(NAND_CMD_READSTART);

#if USE_IRQ
	do {
		err = wait_event_interruptible_timeout(nand_read_wait_queue, dma_ack, 3 * HZ);
	}while(err == -ERESTARTSYS);
	nand_status = NAND_NONE;

	if (!err) {
		printk("*** NAND READ, Warning, wait event 3s timeout!\n");
		dump_jz_dma_channel(0);
		dump_jz_dma_channel(nand_dma_chan);
		printk("REG_BCH_CR=%x REG_BCH_CNT=0x%x REG_BCH_INTS=%x\n", REG_BCH_CR, REG_BCH_CNT, REG_BCH_INTS);
		printk("databuf[0]=%x\n", databuf[0]);
	}
	dprintk("timeout remain = %d\n", err);
#else
	int timeout;
	timeout = 100000;
	while ((!__dmac_channel_transmit_end_detected(bch_dma_chan)) && (timeout--));
	if (timeout <= 0) {
		printk("not use irq, NAND READ timeout!\n");
	}
#endif

	for (i = 0; i < eccsteps; i++) {
		int stat;
		struct buf_be_corrected *buf_correct = &buf_correct0;

		buf_correct->data = databuf + eccsize * i;
		buf_correct->oob = oobbuf + oob_per_eccsize * i;

		stat = chip->ecc.correct(mtd, (u8 *)buf_correct, (u8 *)&errs[i * ERRS_SIZE], NULL);
		if (stat < 0)
			mtd->ecc_stats.failed++;
		else
			mtd->ecc_stats.corrected += stat;
	}

#if defined(CONFIG_MTD_NAND_DMABUF)
	memcpy(buf, read_buf, pagesize);
	memcpy(chip->oob_poi, read_buf + pagesize, oobsize);
#endif
	return 0;
}

static int nand_read_page_hwecc_bch(struct mtd_info *mtd, struct nand_chip *chip, uint8_t * buf)
{
	u32 page = global_page;

	nand_read_page_hwecc_bch0(mtd, chip, buf, page);
	return 0;
}

static int nand_read_page_hwecc_bch_planes(struct mtd_info *mtd, struct nand_chip *chip, uint8_t * buf)
{
	u32 page;
	int pagesize = mtd->writesize >> 1;
	int ppb = mtd->erasesize / mtd->writesize;

	page = (global_page / ppb) * ppb + global_page; /* = global_page%ppb + (global_page/ppb)*ppb*2 */

        /* read 1st page */
	nand_read_page_hwecc_bch0(mtd, chip, buf, page);

        /* read 2nd page */
	nand_read_page_hwecc_bch0(mtd, chip, buf + pagesize, page + ppb);
	return 0;
}

#endif				/* CONFIG_MTD_NAND_DMA */

static int nand_read_page_hwecc_bch_cpu(struct mtd_info *mtd, struct nand_chip *chip, uint8_t * buf)
{
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps / chip->planenum;
	int ecc_pos = chip->ecc.layout->eccpos[0];
	int oob_per_eccsize = ecc_pos / eccsteps;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->ecc.layout->eccpos;
	int pagesize = mtd->writesize / chip->planenum;
	int oobsize = mtd->oobsize / chip->planenum;
	int ecctotal = chip->ecc.total / chip->planenum;
	static struct buf_be_corrected buf_correct0;

	chip->read_buf(mtd, buf, pagesize);
	chip->read_buf(mtd, chip->oob_poi, oobsize);

	for (i = 0; i < ecctotal; i++) {
		ecc_code[i] = chip->oob_poi[eccpos[i]];
	}

	for (i = 0; i < eccsteps; i++) {
		int stat;
		struct buf_be_corrected *buf_correct = &buf_correct0;

		buf_correct->data = buf + eccsize * i;
		buf_correct->oob = chip->oob_poi + oob_per_eccsize * i;

		chip->ecc.hwctl(mtd, NAND_ECC_READ);
		stat = chip->ecc.correct(mtd, (u8 *)buf_correct, &ecc_code[eccbytes*i], &ecc_calc[eccbytes*i]);
		if (stat < 0)
			mtd->ecc_stats.failed++;
		else
			mtd->ecc_stats.corrected += stat;
	}

	return 0;
}

static int nand_read_page_hwecc_bch_planes_cpu(struct mtd_info *mtd, struct nand_chip *chip, uint8_t * buf)
{
	int pagesize = mtd->writesize >> 1;
	int ppb = mtd->erasesize / mtd->writesize;
	uint32_t page;

	page = (global_page / ppb) * ppb + global_page; /* = global_page%ppb + (global_page/ppb)*ppb*2 */

	/* Read first page */
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);
	nand_read_page_hwecc_bch_cpu(mtd, chip, buf);

	/* Read 2nd page */
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page + ppb);
	nand_read_page_hwecc_bch_cpu(mtd, chip, buf+pagesize);
	return 0;
}

#endif				/* CONFIG_MTD_HW_BCH_ECC */

/* read oob using two-plane mode */
static int nand_read_oob_std_planes(struct mtd_info *mtd, struct nand_chip *chip,
				    int global_page, int sndcmd)
{
	int page;
	int oobsize = mtd->oobsize >> 1;
	int ppb = mtd->erasesize / mtd->writesize;

	page = (global_page / ppb) * ppb + global_page; /* = global_page%ppb + (global_page/ppb)*ppb*2 */

	/* Read first page OOB */
	if (sndcmd) {
		chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
	}
	chip->read_buf(mtd, chip->oob_poi, oobsize);
	nand_sw_bch_ops(mtd, chip->oob_poi, 0);
	/* Read second page OOB */
	page += ppb;
	if (sndcmd) {
		chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
		sndcmd = 0;
	}
	chip->read_buf(mtd, chip->oob_poi+oobsize, oobsize);
	nand_sw_bch_ops(mtd, chip->oob_poi + oobsize, 0);
	return 0;
}

/* write oob using two-plane mode */
static int nand_write_oob_std_planes(struct mtd_info *mtd, struct nand_chip *chip,
				     int global_page)
{
	int status = 0, page;
	uint8_t *buf = chip->oob_poi;
	int pagesize = mtd->writesize >> 1;
	int oobsize = mtd->oobsize >> 1;
	int ppb = mtd->erasesize / mtd->writesize;

	page = (global_page / ppb) * ppb + global_page; /* = global_page%ppb + (global_page/ppb)*ppb*2 */

        /* send cmd 0x80, the MSB should be valid if realplane is 4 */
	if (chip->realplanenum == 2)
	{
		if(global_mafid == 0x2c)
			chip->cmdfunc(mtd, 0x80, pagesize, page);
		else
			chip->cmdfunc(mtd, 0x80, pagesize, 0x00);
	}
	else
		chip->cmdfunc(mtd, 0x80, pagesize, page & (1 << (chip->chip_shift - chip->page_shift)));
	
	nand_sw_bch_ops(mtd, buf, 1);
	chip->write_buf(mtd, buf, oobsize);
	/* Send first command to program the OOB data */
	chip->cmdfunc(mtd, 0x11, -1, -1);
	ndelay(100);
	status = chip->waitfunc(mtd, chip);

	page += ppb;
	buf += oobsize;
	chip->cmdfunc(mtd, 0x81, pagesize, page);
	nand_sw_bch_ops(mtd, buf, 1);
	chip->write_buf(mtd, buf, oobsize);
	/* Send command to program the OOB data */
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	/* Wait long R/B */
	ndelay(100);
	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

/* nand erase using two-plane mode */
static void single_erase_cmd_planes(struct mtd_info *mtd, int global_page)
{
	struct nand_chip *chip = mtd->priv;
	int page, ppb = mtd->erasesize / mtd->writesize;

	page = (global_page / ppb) * ppb + global_page; /* = global_page%ppb + (global_page/ppb)*ppb*2 */

        /* send cmd 0x60, the MSB should be valid if realplane is 4 */
	if (chip->realplanenum == 2)
	{
		if(global_mafid == 0x2c)
			chip->cmdfunc(mtd, 0x60, -1, page);
		else
			chip->cmdfunc(mtd, 0x60, -1, 0x00);
	}
	else
		chip->cmdfunc(mtd, 0x60, -1, page & (1 << (chip->chip_shift - chip->page_shift)));

	page += ppb;
	chip->cmdfunc(mtd, 0x60, -1, page & (~(ppb-1))); /* send cmd 0x60 */

	chip->cmdfunc(mtd, NAND_CMD_ERASE2, -1, -1); /* send cmd 0xd0 */
	/* Do not need wait R/B or check status */
}

#if defined(CONFIG_MTD_NAND_DMA)

#if USE_IRQ
static irqreturn_t nand_dma_irq(int irq, void *dev_id)
{
	u8 dma_chan;
	volatile int wakeup = 0;

	dma_chan = irq - IRQ_DMA_0;

	dprintk("jz4750_dma_irq %d, channel %d\n", irq, dma_chan);

	if (__dmac_channel_transmit_halt_detected(dma_chan)) {
		__dmac_channel_clear_transmit_halt(dma_chan);
		wakeup = 1;
		printk("DMA HALT\n");
	}

	if (__dmac_channel_address_error_detected(dma_chan)) {

		REG_DMAC_DCCSR(dma_chan) &= ~DMAC_DCCSR_EN;	/* disable DMA */
		__dmac_channel_clear_address_error(dma_chan);

		REG_DMAC_DSAR(dma_chan) = 0;	/* reset source address register */
		REG_DMAC_DTAR(dma_chan) = 0;	/* reset destination address register */

		/* clear address error in DMACR */
		REG_DMAC_DMACR((dma_chan / HALF_DMA_NUM)) &= ~(1 << 2);
		wakeup = 1;
		printk("DMA address error!\n");
	}

	if (__dmac_channel_descriptor_invalid_detected(dma_chan)) {
		__dmac_channel_clear_descriptor_invalid(dma_chan);
		wakeup = 1;
		printk("DMA DESC INVALID\n");
	}
#if 1

	while (!__dmac_channel_transmit_end_detected(dma_chan));

	if (__dmac_channel_count_terminated_detected(dma_chan)) {
		dprintk("DMA CT\n");
		__dmac_channel_clear_count_terminated(dma_chan);
		wakeup = 0;
	}
#endif

	if (__dmac_channel_transmit_end_detected(dma_chan)) {
		dprintk("DMA TT\n");
		REG_DMAC_DCCSR(dma_chan) &= ~DMAC_DCCSR_EN;	/* disable DMA */
		__dmac_channel_clear_transmit_end(dma_chan);
		wakeup = 1;
	}

	if (wakeup) {
		dprintk("ack %d irq , wake up dma_chan %d nand_status %d\n", dma_ack, dma_chan, nand_status);
		/* wakeup wait event */
		if ((dma_chan == nand_dma_chan) && (nand_status == NAND_PROG)) {
			dprintk("nand prog dma irq, wake up----\n");
			dma_ack1 = 1;
			wake_up_interruptible(&nand_prog_wait_queue);
		}

		if ((dma_chan == bch_dma_chan) && (nand_status == NAND_READ)) {
			dprintk("nand read irq, wake up----\n");
			dma_ack = 1;
			wake_up_interruptible(&nand_read_wait_queue);
		}
		wakeup = 0;
	}

	return IRQ_HANDLED;
}
#endif				/* USE_IRQ */

static int jz4750_nand_dma_init(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	int eccsize = chip->ecc.size;
	int eccsteps = chip->ecc.steps / chip->planenum;
	int eccbytes = chip->ecc.bytes;
	int ecc_pos = chip->ecc.layout->eccpos[0];
	int oob_per_eccsize = ecc_pos / eccsteps;
	int pagesize = mtd->writesize / chip->planenum;
	int oobsize = mtd->oobsize / chip->planenum;
	int i, err;
	jz_dma_desc_8word *desc, *dma_desc_bch_ddr, *dma_desc_nand_ddr, *dma_desc_nand_cmd_pgprog;
	u32  *pval_nand_dcs, *pval_bch_ddr, *pval_bch_dcs, *dummy;
	u32 next;
#if defined(CONFIG_MTD_NAND_DMABUF)
	u8 *oobbuf;
#endif

#if USE_IRQ
	if ((nand_dma_chan = jz_request_dma(DMA_ID_NAND, "nand read or write", nand_dma_irq, IRQF_DISABLED, NULL)) < 0) {
		printk("can't reqeust DMA nand channel.\n");
		return 0;
	}
	dprintk("nand dma channel:%d----\n", nand_dma_chan);

	if ((err = request_irq(IRQ_DMA_0 + bch_dma_chan, nand_dma_irq, IRQF_DISABLED, "bch_dma", NULL))) {
		printk("bch_dma irq request err\n");
		return 0;
	}
#else
	if ((nand_dma_chan = jz_request_dma(DMA_ID_NAND, "nand read or write", NULL, IRQF_DISABLED, NULL)) < 0) {
		printk("can't reqeust DMA nand channel.\n");
		return 0;
	}
	dprintk("nand dma channel:%d----\n", nand_dma_chan);
#endif

	__dmac_channel_enable_clk(nand_dma_chan);
	__dmac_channel_enable_clk(bch_dma_chan);

#if defined(CONFIG_MTD_NAND_DMABUF)
	if (pagesize < 4096) {
		read_buf = prog_buf = (u8 *) __get_free_page(GFP_KERNEL);
	} else {
		read_buf = prog_buf = (u8 *) __get_free_pages(GFP_KERNEL, 1);
	}
	if (!read_buf)
		return -ENOMEM;
#endif
	/* space for the error reports of bch decoding((4 * 5 * eccsteps) bytes), and the space for the value
         * of ddr and dcs of channel 0 and channel nand_dma_chan (4 * (2 + 2) bytes), and the space for dummy
	 * and pval_nand_cmd_pgprog (4 * 2 bytes). */
	errs = (u32 *)kmalloc(4 * (2 + 2 + 2 + 5 * eccsteps), GFP_KERNEL);
	if (!errs)
		return -ENOMEM;

	pval_nand_ddr = errs + 5 * eccsteps;
	pval_nand_dcs = pval_nand_ddr + 1;
	pval_bch_ddr = pval_nand_dcs + 1;
	pval_bch_dcs = pval_bch_ddr + 1;
        /* space for nand prog waiting target, the content is useless */
	dummy = pval_bch_dcs + 1;
        /* space to store CMD_PGPROG(0x10) or 0x11 */
	pval_nand_cmd_pgprog = (u8 *)(dummy + 1);

	/* desc can't across 4KB boundary, as desc base address is fixed */
	/* space of descriptors for nand reading data and oob blocks */
	dma_desc_nand_read = (jz_dma_desc_8word *) __get_free_page(GFP_KERNEL);
	if (!dma_desc_nand_read)
		return -ENOMEM;

	/* space of descriptors for bch decoding */
	dma_desc_dec = dma_desc_nand_read + 2;
	dma_desc_dec1 = dma_desc_dec + eccsteps;
	dma_desc_dec2 = dma_desc_dec + eccsteps * 2;

	/* space of descriptors for notifying bch channel */
	dma_desc_bch_ddr = dma_desc_dec2 + eccsteps;

	/* space of descriptors for bch encoding */
	dma_desc_enc = dma_desc_bch_ddr + 2;
	dma_desc_enc1 = dma_desc_enc + eccsteps;

	/* space of descriptors for nand programing data and oob blocks */
	dma_desc_nand_prog = dma_desc_enc1 + eccsteps;

        /* space of descriptors for nand prog waiting, including pgprog and sync */
	dma_desc_nand_cmd_pgprog = dma_desc_nand_prog + 2;

	/* space of descriptors for notifying nand channel, including ddr and dcsr */
	dma_desc_nand_ddr = dma_desc_nand_cmd_pgprog + 2;

/*************************************
 * Setup of nand programing descriptors
 *************************************/
#if defined(CONFIG_MTD_NAND_DMABUF)
	oobbuf = prog_buf + pagesize;
#endif
	/* set descriptor for encoding data blocks */
	desc = dma_desc_enc;
	for (i = 0; i < eccsteps; i++) {
		next = (CPHYSADDR((u32)dma_desc_enc1) + i * (sizeof(jz_dma_desc_8word))) >> 4;

		desc->dcmd =
		    DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_8 |
		    DMAC_DCMD_DS_BCH | DMAC_DCMD_LINK;
#if defined(CONFIG_MTD_NAND_DMABUF)
		desc->dsadr = CPHYSADDR((u32)prog_buf) + i * eccsize;	/* DMA source address */
		desc->dtadr = CPHYSADDR((u32)oobbuf) + ecc_pos + i * eccbytes;	/* DMA target address */
#endif
		desc->ddadr = (next << 24) + eccsize / DIV_DS_BCH;	/* size: eccsize bytes */
		desc->dreqt = DMAC_DRSR_RS_BCH_ENC;
		dprintk("cmd:%x sadr:%x tadr:%x dadr:%x\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr);
		desc++;
	}

	/* set descriptor for encoding oob blocks */
	desc = dma_desc_enc1;
	for (i = 0; i < eccsteps; i++) {
		next = (CPHYSADDR((u32)dma_desc_enc) + (i + 1) * (sizeof(jz_dma_desc_8word))) >> 4;

		desc->dcmd =
		    DMAC_DCMD_BLAST | DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_8 |
		    DMAC_DCMD_DWDH_8 | DMAC_DCMD_DS_32BIT | DMAC_DCMD_LINK;
#if defined(CONFIG_MTD_NAND_DMABUF)
		desc->dsadr = CPHYSADDR((u32)oobbuf) + oob_per_eccsize * i;	/* DMA source address, 28/4 = 7bytes */
		desc->dtadr = CPHYSADDR((u32)oobbuf) + ecc_pos + i * eccbytes;	/* DMA target address */
#endif
		desc->ddadr = (next << 24) + (oob_per_eccsize + 3) / 4;	/* size: 7 bytes -> 2 words */
		desc->dreqt = DMAC_DRSR_RS_BCH_ENC;
		dprintk("cmd:%x sadr:%x tadr:%x dadr:%x\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr);
		desc++;
	}

	next = (CPHYSADDR((u32)dma_desc_nand_ddr)) >> 4;
	desc--;
	desc->ddadr = (next << 24) + (oob_per_eccsize + 3) / 4;

	/* set the descriptor to set door bell of nand_dma_chan for programing nand */
	desc = dma_desc_nand_ddr;
	*pval_nand_ddr = 1 << (nand_dma_chan - nand_dma_chan / HALF_DMA_NUM * HALF_DMA_NUM);
	next = (CPHYSADDR((u32)dma_desc_nand_ddr) + sizeof(jz_dma_desc_8word)) >> 4;
	desc->dcmd = DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BIT | DMAC_DCMD_LINK;
	desc->dsadr = CPHYSADDR((u32)pval_nand_ddr);	/* DMA source address */
	//desc->dtadr = CPHYSADDR(DMAC_DMADBSR(nand_dma_chan / HALF_DMA_NUM));	/* nand_dma_chan's descriptor addres register */
	desc->dtadr = CPHYSADDR(DMAC_DCCSR(nand_dma_chan / HALF_DMA_NUM));	/* nand_dma_chan's descriptor addres register */
	desc->ddadr = (next << 24) + 1;	/* size: 1 word */
	desc->dreqt = DMAC_DRSR_RS_AUTO;
	dprintk("*pval_nand_ddr=0x%x\n", *pval_nand_ddr);

	/* set the descriptor to write dccsr of nand_dma_chan for programing nand, dccsr should be set at last */
	desc++;
	*pval_nand_dcs = DMAC_DCCSR_DES8 | DMAC_DCCSR_EN;	/* set value for writing ddr to enable channel nand_dma_chan */
	desc->dcmd = DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BIT;
	desc->dsadr = CPHYSADDR((u32)pval_nand_dcs);	/* DMA source address */
	desc->dtadr = CPHYSADDR(DMAC_DCCSR(nand_dma_chan));	/* address of dma door bell set register */
	desc->ddadr = (0 << 24) + 1;	/* size: 1 word */
	desc->dreqt = DMAC_DRSR_RS_AUTO;
	dprintk("*pval_nand_dcs=0x%x\n", *pval_nand_dcs);

	/* set descriptor for nand programing data block */
	desc = dma_desc_nand_prog;
	next = (CPHYSADDR((u32)dma_desc_nand_prog) + sizeof(jz_dma_desc_8word)) >> 4;
	desc->dcmd =
	    DMAC_DCMD_SAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_8 |
	    DMAC_DCMD_DS_NAND | DMAC_DCMD_LINK;	//for unshare mode, no DMAC_DCMD_DAI
	if (is_share_mode()) {
		desc->dcmd |= DMAC_DCMD_DAI;
	}
#if defined(CONFIG_MTD_NAND_DMABUF)
	desc->dsadr = CPHYSADDR((u32)prog_buf);	/* DMA source address */
#endif
	desc->dtadr = CPHYSADDR((u32)(chip->IO_ADDR_W)); /* DMA target address */
	desc->ddadr = (next << 24) + pagesize / DIV_DS_NAND;	/* size: eccsize bytes */
	desc->dreqt = DMAC_DRSR_RS_AUTO;
	dprintk("cmd:%x sadr:%x tadr:%x dadr:%x\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr);

	/* set descriptor for nand programing oob block */
	desc++;
	next = (CPHYSADDR((u32)dma_desc_nand_cmd_pgprog)) >> 4;
	desc->dcmd =
	    DMAC_DCMD_SAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_8 |
	    DMAC_DCMD_DS_NAND | DMAC_DCMD_LINK;
	if (is_share_mode()) {
		desc->dcmd |= DMAC_DCMD_DAI;
	}
#if defined(CONFIG_MTD_NAND_DMABUF)
	desc->dsadr = CPHYSADDR((u32)oobbuf);                	/* DMA source address */
#endif
	desc->dtadr = CPHYSADDR((u32)(chip->IO_ADDR_W));	/* DMA target address: dataport */
	desc->ddadr = (next << 24) + oobsize / DIV_DS_NAND;            	/* size: eccsize bytes */
	desc->dreqt = DMAC_DRSR_RS_AUTO;
	dprintk("cmd:%x sadr:%x tadr:%x dadr:%x\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr);

	/* set descriptor for __nand_cmd(CMD_PGPROG) */
	desc = dma_desc_nand_cmd_pgprog;
	*pval_nand_cmd_pgprog = NAND_CMD_PAGEPROG;
	next = (CPHYSADDR((u32)dma_desc_nand_cmd_pgprog) + sizeof(jz_dma_desc_8word)) >> 4;
	desc->dcmd =
	    DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_8 | DMAC_DCMD_DWDH_8 | DMAC_DCMD_DS_8BIT | DMAC_DCMD_LINK;
	desc->dsadr = CPHYSADDR((u32)pval_nand_cmd_pgprog);	        /* DMA source address */
	desc->dtadr = CPHYSADDR((u32)chip->IO_ADDR_R | cmd_offset);	/* DMA target address: cmdport */
	desc->ddadr = (next << 24) + 1;	                        /* size: 1 byte */
	desc->dreqt = DMAC_DRSR_RS_AUTO;
	dprintk("cmd:%x sadr:%x tadr:%x dadr:%x\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr);

	/* set descriptor for __nand_sync() */
	desc++;
#if USE_IRQ
	desc->dcmd =
		DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BIT | DMAC_DCMD_TIE;
#else
	desc->dcmd =
		DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BIT;
#endif
	desc->dsadr = CPHYSADDR((u32)pval_nand_ddr);	/* DMA source address */
	desc->dtadr = CPHYSADDR((u32)dummy);        	/* DMA target address, the content is useless */
	desc->ddadr = (0 << 24) + 1;             	/* size: 1 word */
	desc->dreqt = DMAC_DRSR_RS_NAND;
	dprintk("1cmd:%x sadr:%x tadr:%x dadr:%x\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr);

        /* eccsteps*2 + 2 + 2 + 2:
	   dma_desc_enc + dma_desc_enc1 + dma_desc_nand_prog(oob) + dma_desc_nand_ddr(csr)
	   + dma_desc_nand_cmd_pgprog(sync) */
	dma_cache_wback_inv((u32)dma_desc_enc, (eccsteps * 2 + 2 + 2 + 2) * (sizeof(jz_dma_desc_8word)));
	/* 4*6: pval_nand_ddr, pval_nand_dcs, pval_bch_ddr, pval_bch_dcs, dummy, pval_nand_cmd_pgprog */
	dma_cache_wback_inv((u32)pval_nand_ddr, 4 * 8); /* 8 words, a cache line */

/*************************************
 * Setup of nand reading descriptors
 *************************************/
#if defined(CONFIG_MTD_NAND_DMABUF)
	oobbuf = read_buf + pagesize;
#endif
	/* set descriptor for nand reading data block */
	desc = dma_desc_nand_read;
	next = (CPHYSADDR((u32)dma_desc_nand_read) + sizeof(jz_dma_desc_8word)) >> 4;
	desc->dcmd =
	    DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_8 | DMAC_DCMD_DWDH_32 |
	    DMAC_DCMD_DS_NAND | DMAC_DCMD_LINK;		//for unshare mode, no DMAC_DCMD_SAI
	if (is_share_mode()) {
		desc->dcmd |= DMAC_DCMD_SAI;
	}
	desc->dsadr = CPHYSADDR((u32)(chip->IO_ADDR_R));	/* DMA source address */
#if defined(CONFIG_MTD_NAND_DMABUF)
	desc->dtadr = CPHYSADDR((u32)read_buf);	/* DMA target address */
#endif
	desc->ddadr = (next << 24) + pagesize / DIV_DS_NAND;	/* size: eccsize bytes */
	desc->dreqt = DMAC_DRSR_RS_NAND;
	dprintk("cmd:%x sadr:%x tadr:%x dadr:%x\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr);

	/* set descriptor for nand reading oob block */
	desc++;
	next = (CPHYSADDR((u32)dma_desc_bch_ddr)) >> 4;
	desc->dcmd =
	    DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_8 | DMAC_DCMD_DWDH_32 |
	    DMAC_DCMD_DS_NAND | DMAC_DCMD_LINK;
	if (is_share_mode()) {
		desc->dcmd |= DMAC_DCMD_SAI;
	}
	desc->dsadr = CPHYSADDR((u32)(chip->IO_ADDR_R));	/* DMA source address */
#if defined(CONFIG_MTD_NAND_DMABUF)
	desc->dtadr = CPHYSADDR((u32)oobbuf);	/* DMA target address */
#endif
	desc->ddadr = (next << 24) + oobsize / DIV_DS_NAND;	/* size: eccsize bytes */
	desc->dreqt = DMAC_DRSR_RS_AUTO;
	dprintk("cmd:%x sadr:%x tadr:%x dadr:%x\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr);

	/* set the descriptor to set door bell for bch */
	desc = dma_desc_bch_ddr;
	*pval_bch_ddr = DMAC_DMADBSR_DBS0;	// set value for writing ddr to enable channel 0
	next = (CPHYSADDR((u32)dma_desc_bch_ddr) + sizeof(jz_dma_desc_8word)) >> 4;
	desc->dcmd = DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BIT | DMAC_DCMD_LINK;
	desc->dsadr = CPHYSADDR((u32)pval_bch_ddr);	/* DMA source address */
	//desc->dtadr = CPHYSADDR(DMAC_DMADBSR(0));	/* channel 1's descriptor addres register */
	desc->dtadr = CPHYSADDR(DMAC_DCCSR(0));	/* channel 1's descriptor addres register */
	desc->ddadr = (next << 24) + 1;	/* size: 1 word */
	desc->dreqt = DMAC_DRSR_RS_AUTO;

	/* set descriptor for writing dcsr */
	desc++;
	*pval_bch_dcs = DMAC_DCCSR_DES8 | DMAC_DCCSR_EN;	// set value for writing ddr to enable channel 1
	desc->dcmd = DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BIT;
	desc->dsadr = CPHYSADDR((u32)pval_bch_dcs);	/* DMA source address */
	desc->dtadr = CPHYSADDR(DMAC_DCCSR(bch_dma_chan));	/* address of dma door bell set register */
	desc->ddadr = (0 << 24) + 1;	/* size: 1 word */
	desc->dreqt = DMAC_DRSR_RS_AUTO;

	/* descriptors for data to be written to bch */
	desc = dma_desc_dec;
	for (i = 0; i < eccsteps; i++) {
		next = CPHYSADDR((u32)dma_desc_dec1 + i * (sizeof(jz_dma_desc_8word))) >> 4;

		desc->dcmd =
		    DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 |
		    DMAC_DCMD_DS_BCH | DMAC_DCMD_LINK;
#if defined(CONFIG_MTD_NAND_DMABUF)
		desc->dsadr = CPHYSADDR((u32)read_buf) + i * eccsize;	/* DMA source address */
#endif
		desc->dtadr = CPHYSADDR((u32)errs) + i * 4 * ERRS_SIZE;	/* DMA target address */
		desc->ddadr = (next << 24) + eccsize / DIV_DS_BCH;	/* size: eccsize bytes */
		desc->dreqt = DMAC_DRSR_RS_BCH_DEC;
		dprintk("desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr);
		desc++;
	}

	/* descriptors for oob to be written to bch */
	desc = dma_desc_dec1;
	for (i = 0; i < eccsteps; i++) {
		next = CPHYSADDR((u32)dma_desc_dec2 + i * (sizeof(jz_dma_desc_8word))) >> 4;

		desc->dcmd =
		    DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_8 | DMAC_DCMD_DWDH_32 |
		    DMAC_DCMD_DS_8BIT | DMAC_DCMD_LINK;
#if defined(CONFIG_MTD_NAND_DMABUF)
		desc->dsadr = CPHYSADDR((u32)oobbuf) + oob_per_eccsize * i;	/* DMA source address */
#endif
		desc->dtadr = CPHYSADDR((u32)errs) + i * 4 * ERRS_SIZE;	/* DMA target address */
		desc->ddadr = (next << 24) + oob_per_eccsize;	/* size: 7 bytes */
		desc->dreqt = DMAC_DRSR_RS_BCH_DEC;
		dprintk("desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr);
		desc++;
	}

	/* descriptors for parities to be written to bch */
	desc = dma_desc_dec2;
	for (i = 0; i < eccsteps; i++) {
		next = (CPHYSADDR((u32)dma_desc_dec) + (i + 1) * (sizeof(jz_dma_desc_8word))) >> 4;

		desc->dcmd =
		    DMAC_DCMD_BLAST | DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_8 |
		    DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_BCH | DMAC_DCMD_LINK;
#if defined(CONFIG_MTD_NAND_DMABUF)
		desc->dsadr = CPHYSADDR((u32)oobbuf) + ecc_pos + i * eccbytes;	/* DMA source address */
#endif
		desc->dtadr = CPHYSADDR((u32)errs) + i * 4 * ERRS_SIZE;	/* DMA target address */
		desc->ddadr = (next << 24) + (eccbytes + DIV_DS_BCH - 1) / DIV_DS_BCH;	/* size: eccbytes bytes */
		desc->dreqt = DMAC_DRSR_RS_BCH_DEC;
		dprintk("desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr);
		desc++;
	}
	desc--;
	desc->dcmd &= ~DMAC_DCMD_LINK;
#if USE_IRQ
	desc->dcmd |= DMAC_DCMD_TIE;
#endif

	dma_cache_wback_inv((u32)dma_desc_nand_read, (2 + 2 + eccsteps * 3) * (sizeof(jz_dma_desc_8word)));
	dma_cache_wback_inv((u32)pval_bch_ddr, 4 * 2); /* two words */

	return 0;
}

#endif				/* CONFIG_MTD_NAND_DMA */

void copy_to_cpu_mode(struct mtd_info *mtd_cpu, struct mtd_info *mtd_dma, char use_planes)
{
	struct nand_chip *chip_cpu = (struct nand_chip *)(&mtd_cpu[1]);
	struct nand_chip *chip_dma = (struct nand_chip *)(&mtd_dma[1]);

#ifdef CONFIG_MTD_HW_BCH_ECC
	memcpy(mtd_cpu, mtd_dma, sizeof(struct mtd_info));
	mtd_cpu->priv = chip_cpu;
	memcpy(chip_cpu, chip_dma, sizeof(struct nand_chip));

	chip_cpu->ecc.correct = jzsoc_nand_bch_correct_data_cpu;

	if (use_planes) {
		chip_cpu->ecc.read_page = nand_read_page_hwecc_bch_planes_cpu;
		chip_cpu->ecc.write_page = nand_write_page_hwecc_bch_planes_cpu;
	} else {
		chip_cpu->ecc.read_page = nand_read_page_hwecc_bch_cpu;
		chip_cpu->ecc.write_page = nand_write_page_hwecc_bch_cpu;
	}
#else
	memcpy(mtd_cpu, mtd_dma, sizeof(struct mtd_info));
	mtd_cpu->priv = chip_cpu;
	memcpy(chip_cpu, chip_dma, sizeof(struct nand_chip));

#endif
	mtd_cpu->flags |= MTD_NAND_CPU_MODE;
}

/*
 * Main initialization routine
 */
int __init jznand_init(void)
{
	struct nand_chip *this;
	struct mtd_info *jz_mtd_cpu, *jz_mtd_cpu1; /* jz_mtd_cpu for 2 planes, jz_mtd_cpu1 for 1 plane */
	int ret, i;

	printk("JZ NAND init:");
#if defined(CONFIG_MTD_NAND_DMA)
#if defined(CONFIG_MTD_NAND_DMABUF)
	printk(" DMA mode, using DMA buffer in NAND driver, ");
#else
	printk(" DMA mode, using DMA buffer in upper layer, ");
#endif
#else
	printk(KERN_INFO " CPU mode, ");
#endif
#if defined(CONFIG_MTD_HW_BCH_8BIT)
	printk(" 8bit BCH.\n");
#else
	printk(" 4bit BCH.\n");
#endif

	/* Allocate memory for MTD device structure and private data */
	jz_mtd = kmalloc(sizeof(struct mtd_info) + sizeof(struct nand_chip), GFP_KERNEL);
	if (!jz_mtd) {
		printk("Unable to allocate JzSOC NAND MTD device structure for dma mode using multi-planes.\n");
		return -ENOMEM;
	}

	jz_mtd1 = kmalloc(sizeof(struct mtd_info) + sizeof (struct nand_chip), GFP_KERNEL);
	if (!jz_mtd1) {
		printk ("Unable to allocate JzSOC NAND MTD device structure for dma mode using 1 plane.\n");
		kfree(jz_mtd);
		return -ENOMEM;
	}

	jz_mtd_cpu = kmalloc(sizeof(struct mtd_info) + sizeof (struct nand_chip), GFP_KERNEL);
	if (!jz_mtd_cpu) {
		printk ("Unable to allocate JzSOC NAND MTD device structure for cpu mode using multi-planes.\n");
		kfree(jz_mtd);
		kfree(jz_mtd1);
		return -ENOMEM;
	}

	jz_mtd_cpu1 = kmalloc(sizeof(struct mtd_info) + sizeof (struct nand_chip), GFP_KERNEL);
	if (!jz_mtd_cpu) {
		printk ("Unable to allocate JzSOC NAND MTD device structure for cpu mode using 1 plane.\n");
		kfree(jz_mtd);
		kfree(jz_mtd1);
		kfree(jz_mtd_cpu);
		return -ENOMEM;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *)(&jz_mtd[1]);

	/* Initialize structures */
	memset((char *)jz_mtd, 0, sizeof(struct mtd_info));
	memset((char *)this, 0, sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	jz_mtd->priv = this;

	if (is_share_mode()) {
		addr_offset = NAND_ADDR_OFFSET0;
		cmd_offset = NAND_CMD_OFFSET0;
	} else {
		addr_offset = NAND_ADDR_OFFSET1;
		cmd_offset = NAND_CMD_OFFSET1;
	}

	/* Set & initialize NAND Flash controller */
	jz_device_setup();

	/* Set address of NAND IO lines to static bank1 by default */
	this->IO_ADDR_R = (void __iomem *)NAND_DATA_PORT1;
	this->IO_ADDR_W = (void __iomem *)NAND_DATA_PORT1;
	this->cmd_ctrl = jz_hwcontrol;
	this->dev_ready = jz_device_ready;

#ifdef CONFIG_MTD_HW_BCH_ECC
	this->ecc.calculate = jzsoc_nand_calculate_bch_ecc;
	this->ecc.hwctl = jzsoc_nand_enable_bch_hwecc;
	this->ecc.mode = NAND_ECC_HW;
	this->ecc.size = 512;
#if defined(CONFIG_MTD_NAND_DMA)
	this->ecc.correct = jzsoc_nand_bch_correct_data;
	this->ecc.read_page = nand_read_page_hwecc_bch;
	this->ecc.write_page = nand_write_page_hwecc_bch;
#else
	this->ecc.correct = jzsoc_nand_bch_correct_data_cpu;
	this->ecc.read_page = nand_read_page_hwecc_bch_cpu;
	this->ecc.write_page = nand_write_page_hwecc_bch_cpu;
#endif
#if defined(CONFIG_MTD_HW_BCH_8BIT)
	this->ecc.bytes = 13;
#else
	this->ecc.bytes = 7;
#endif
#endif

#ifdef  CONFIG_MTD_SW_HM_ECC
	this->ecc.mode = NAND_ECC_SOFT;
#endif
	/* 20 us command delay time */
	this->chip_delay = 20;
	/* Scan to find existance of the device */
	ret = nand_scan_ident(jz_mtd, nand_chips);

#ifdef CONFIG_MTD_HW_BCH_ECC
	if (!ret) {
		if (this->planenum == 2) {
			/* reset nand functions */
			this->erase_cmd = single_erase_cmd_planes;
#if defined(CONFIG_MTD_NAND_DMA)
			this->ecc.read_page = nand_read_page_hwecc_bch_planes;
			this->ecc.write_page = nand_write_page_hwecc_bch_planes;
#else
			this->ecc.read_page = nand_read_page_hwecc_bch_planes_cpu;
			this->ecc.write_page = nand_write_page_hwecc_bch_planes_cpu;
#endif
			this->ecc.read_oob = nand_read_oob_std_planes;
			this->ecc.write_oob = nand_write_oob_std_planes;

			printk(KERN_INFO "Nand using two-plane mode, "
			       "and resized to writesize:%d oobsize:%d blocksize:0x%x \n",
			       jz_mtd->writesize, jz_mtd->oobsize, jz_mtd->erasesize);
		}
	}
#endif

	/* Determine whether all the partitions will use multiple planes if supported */
	nr_partitions = sizeof(partition_info) / sizeof(struct mtd_partition);
	all_use_planes = 1;
	for (i = 0; i < nr_partitions; i++) {
		all_use_planes &= partition_info[i].use_planes;
	}

#if !defined(CONFIG_MTD_NAND_DMA)
	for (i = 0; i < nr_partitions; i++) {
		partition_info[i].cpu_mode = 1;
	}
#endif

	if (!ret)
		ret = nand_scan_tail(jz_mtd);

	if (ret){
		kfree (jz_mtd1);
		kfree (jz_mtd);
		return -ENXIO;
	}

#ifdef CONFIG_MTD_HW_BCH_ECC
#if defined(CONFIG_MTD_NAND_DMA)
	jz4750_nand_dma_init(jz_mtd);

 	((struct nand_chip *) (&jz_mtd1[1]))->ecc.read_page = nand_read_page_hwecc_bch;
 	((struct nand_chip *) (&jz_mtd1[1]))->ecc.write_page = nand_write_page_hwecc_bch;
#else
 	((struct nand_chip *) (&jz_mtd1[1]))->ecc.read_page = nand_read_page_hwecc_bch_cpu;
 	((struct nand_chip *) (&jz_mtd1[1]))->ecc.write_page = nand_write_page_hwecc_bch_cpu;
#endif
#endif

	/* Register the partitions */
	printk (KERN_NOTICE "Creating %d MTD partitions on \"%s\":\n", nr_partitions, jz_mtd->name);

	if ((this->planenum == 2) && !all_use_planes) {
		copy_to_cpu_mode(jz_mtd_cpu, jz_mtd, 1);
		copy_to_cpu_mode(jz_mtd_cpu1, jz_mtd1, 0);

		for (i = 0; i < nr_partitions; i++) {
			if (partition_info[i].use_planes) {
				if (partition_info[i].cpu_mode)
					add_mtd_partitions(jz_mtd_cpu, &partition_info[i], 1);
				else
					add_mtd_partitions(jz_mtd, &partition_info[i], 1);
			} else {
				if (partition_info[i].cpu_mode)
					add_mtd_partitions(jz_mtd_cpu1, &partition_info[i], 1);
				else
					add_mtd_partitions(jz_mtd1, &partition_info[i], 1);
			}
		}
	} else {
		kfree(jz_mtd1);
		kfree(jz_mtd_cpu1);
		copy_to_cpu_mode(jz_mtd_cpu, jz_mtd, 0);

		for (i = 0; i < nr_partitions; i++) {
			if (partition_info[i].cpu_mode)
				add_mtd_partitions(jz_mtd_cpu, &partition_info[i], 1);
			else
				add_mtd_partitions(jz_mtd, &partition_info[i], 1);
		}
	}
	return 0;
}

module_init(jznand_init);

/*
 * Clean up routine
 */
#ifdef MODULE

#if defined(CONFIG_MTD_NAND_DMA)
static int jz4750_nand_dma_exit(struct mtd_info *mtd)
{
	int pagesize = mtd->writesize / chip->planenum;

#if USE_IRQ
	free_irq(IRQ_DMA_0 + nand_dma_chan, NULL);
	free_irq(IRQ_DMA_0 + bch_dma_chan, NULL);
#endif

	/* space for the error reports of bch decoding((4 * 5 * eccsteps) bytes),
         * and the space for the value of ddr and dcs of channel 0 and channel
         * nand_dma_chan (4 * (2 + 2) bytes) */
	kfree(errs);

	/* space for dma_desc_nand_read contains dma_desc_nand_prog,
	 * dma_desc_enc and dma_desc_dec */
	free_page((u32)dma_desc_nand_read);

#if defined(CONFIG_MTD_NAND_DMABUF)
	if (pagesize < 4096) {
		free_page((u32)prog_buf);
	} else {
		free_pages((u32)prog_buf, 1);
	}
#endif

	return 0;
}
#endif

static void __exit jznand_cleanup(void)
{
#if defined(CONFIG_MTD_NAND_DMA)
	jz4750_nand_dma_exit(jz_mtd);
#endif

	/* Unregister partitions */
	del_mtd_partitions(jz_mtd);

	/* Unregister the device */
	del_mtd_device(jz_mtd);

	/* Free the MTD device structure */
	if ((this->planenum == 2) && !all_use_planes)
		kfree (jz_mtd1);
	kfree(jz_mtd);
}

module_exit(jznand_cleanup);
#endif
