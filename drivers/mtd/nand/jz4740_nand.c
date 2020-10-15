/*
 * linux/drivers/mtd/nand/jz4740_nand.c
 *
 * Copyright (c) 2005 - 2007 Ingenic Semiconductor Inc.
 *
 * Ingenic JZ4740 NAND driver
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
#include <linux/highmem.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include <asm/jzsoc.h>

#define NAND_DATA_PORT1	       0xB8000000	/* read-write area in static bank 1 */
#define NAND_DATA_PORT2	       0xB4000000	/* read-write area in static bank 2 */
#define NAND_DATA_PORT3	       0xAC000000	/* read-write area in static bank 3 */
#define NAND_DATA_PORT4	       0xA8000000	/* read-write area in static bank 4 */

#define PAR_SIZE 9

#define __nand_enable()	       (REG_EMC_NFCSR |= EMC_NFCSR_NFE1 | EMC_NFCSR_NFCE1)
#define __nand_disable()       (REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE1) 

#define __nand_ecc_enable()    (REG_EMC_NFECR = EMC_NFECR_ECCE | EMC_NFECR_ERST )
#define __nand_ecc_disable()   (REG_EMC_NFECR &= ~EMC_NFECR_ECCE)

#define __nand_select_hm_ecc() (REG_EMC_NFECR &= ~EMC_NFECR_RS )
#define __nand_select_rs_ecc() (REG_EMC_NFECR |= EMC_NFECR_RS)

#define __nand_read_hm_ecc()   (REG_EMC_NFECC & 0x00ffffff)

#define __nand_rs_ecc_encoding()	(REG_EMC_NFECR |= EMC_NFECR_RS_ENCODING)
#define __nand_rs_ecc_decoding()	(REG_EMC_NFECR &= ~EMC_NFECR_RS_ENCODING)
#define __nand_ecc_encode_sync() while (!(REG_EMC_NFINTS & EMC_NFINTS_ENCF))
#define __nand_ecc_decode_sync() while (!(REG_EMC_NFINTS & EMC_NFINTS_DECF))

/*
 * MTD structure for JzSOC board
 */
static struct mtd_info *jz_mtd = NULL;
extern struct mtd_info *jz_mtd1;
extern char all_use_planes;
extern int global_page; /* for two-plane operations */

int nr_partitions; /* Number of partitions */

/* 
 * Define partitions for flash devices
 */
#ifdef CONFIG_JZ4740_PAVO
struct mtd_partition partition_info[] = {
	{name:"NAND BOOT partition",
	 offset:0 * 0x100000,
	 size:4 * 0x100000,
	 use_planes: 0,
	 mtdblock_jz_invalid: 1},
	{name:"NAND KERNEL partition",
	 offset:4 * 0x100000,
	 size:4 * 0x100000,
	 use_planes: 0,
	 mtdblock_jz_invalid: 1},
	{name:"NAND ROOTFS partition",
	 offset:8 * 0x100000,
	 size:504 * 0x100000,
	 use_planes: 0,
	 mtdblock_jz_invalid: 1},
	{name:"NAND VFAT partition",
	 offset:512 * 0x100000,
	 size:512 * 0x100000,
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
					     2,		/* reserved blocks of mtd0 */
					     2,		/* reserved blocks of mtd1 */
					     10,	/* reserved blocks of mtd2 */
					     10,	/* reserved blocks of mtd3 */
					     20,	/* reserved blocks of mtd4 */
					     20};	/* reserved blocks of mtd5 */
#endif /* CONFIG_JZ4740_PAVO */

#ifdef CONFIG_JZ4740_LEO
struct mtd_partition partition_info[] = {
	{ name: "NAND BOOT partition",
	  offset:  0 * 0x100000,
	  size:    4 * 0x100000,
	  use_planes: 0,
	  mtdblock_jz_invalid: 1 },
	{ name: "NAND KERNEL partition",
	  offset:  4 * 0x100000,
	  size:    4 * 0x100000,
	  use_planes: 0,
	  mtdblock_jz_invalid: 1 },
	{ name: "NAND ROOTFS partition",
	  offset:  8 * 0x100000,
	  size:    56 * 0x100000,
	  use_planes: 0,
	  mtdblock_jz_invalid: 1 },
	{ name: "NAND VFAT partition",
	  offset:  64 * 0x100000,
	  size:    64 * 0x100000,
	  use_planes: 1,
	  mtdblock_jz_invalid: 0 },
};
static int partition_reserved_badblocks[] = {
					     2,		/* reserved blocks of mtd0 */
					     2,		/* reserved blocks of mtd1 */
					     10,	/* reserved blocks of mtd2 */
					     10};	/* reserved blocks of mtd3 */
#endif /* CONFIG_JZ4740_LEO */

#ifdef CONFIG_JZ4740_LYRA
struct mtd_partition partition_info[] = {
	{ name: "NAND BOOT partition",
	  offset:  0 * 0x100000,
	  size:    4 * 0x100000,
	  use_planes: 0,
	  mtdblock_jz_invalid: 1 },
	{ name: "NAND KERNEL partition",
	  offset:  4 * 0x100000,
	  size:    4 * 0x100000,
	  use_planes: 0,
	  mtdblock_jz_invalid: 1 },
	{ name: "NAND ROOTFS partition",
	  offset:  8 * 0x100000,
	  size:    120 * 0x100000,
	  use_planes: 0,
	  mtdblock_jz_invalid: 1 },
	{ name: "NAND DATA1 partition",
	  offset:  128 * 0x100000,
	  size:    128 * 0x100000,
	  use_planes: 0,
	  mtdblock_jz_invalid: 1 },
	{ name: "NAND DATA2 partition",
	  offset:  256 * 0x100000,
	  size:    256 * 0x100000,
	  use_planes: 0,
	  mtdblock_jz_invalid: 1 },
	{ name: "NAND VFAT partition",
	  offset:  512 * 0x100000,
	  size:    512 * 0x100000,
	  use_planes: 1,
	  mtdblock_jz_invalid: 0 },
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
					     2,		/* reserved blocks of mtd0 */
					     2,		/* reserved blocks of mtd1 */
					     10,	/* reserved blocks of mtd2 */
					     10,	/* reserved blocks of mtd3 */
					     20,	/* reserved blocks of mtd4 */
					     20};	/* reserved blocks of mtd5 */
#endif /* CONFIG_JZ4740_LYRA */

#ifdef CONFIG_JZ4725_DIPPER
struct mtd_partition partition_info[] = {
	{ name: "NAND BOOT partition",
	  offset:  0 * 0x100000,
	  size:    4 * 0x100000,
	  use_planes: 0,
	  mtdblock_jz_invalid: 1 },
	{ name: "NAND KERNEL partition",
	  offset:  4 * 0x100000,
	  size:    4 * 0x100000,
	  use_planes: 0,
	  mtdblock_jz_invalid: 1 },
	{ name: "NAND ROOTFS partition",
	  offset:  8 * 0x100000,
	  size:    56 * 0x100000,
	  use_planes: 0,
	  mtdblock_jz_invalid: 1 },
	{ name: "NAND VFAT partition",
	  offset:  64 * 0x100000,
	  size:    64 * 0x100000,
	  use_planes: 1,
	  mtdblock_jz_invalid: 0 },
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
					     2,		/* reserved blocks of mtd0 */
					     2,		/* reserved blocks of mtd1 */
					     10,	/* reserved blocks of mtd2 */
					     10};	/* reserved blocks of mtd3 */
#endif /* CONFIG_JZ4740_DIPPER */

#ifdef CONFIG_JZ4720_VIRGO
struct mtd_partition partition_info[] = {
	{ name: "NAND BOOT partition",
	  offset:  0 * 0x100000,
	  size:    4 * 0x100000,
	  use_planes: 0,
	  mtdblock_jz_invalid: 1 },
	{ name: "NAND KERNEL partition",
	  offset:  4 * 0x100000,
	  size:    4 * 0x100000,
	  use_planes: 0,
	  mtdblock_jz_invalid: 1 },
	{ name: "NAND ROOTFS partition",
	  offset:  8 * 0x100000,
	  size:    120 * 0x100000,
	  use_planes: 0,
	  mtdblock_jz_invalid: 1 },
	{ name: "NAND DATA1 partition",
	  offset:  128 * 0x100000,
	  size:    128 * 0x100000,
	  use_planes: 0,
	  mtdblock_jz_invalid: 1 },
	{ name: "NAND DATA2 partition",
	  offset:  256 * 0x100000,
	  size:    256 * 0x100000,
	  use_planes: 0,
	  mtdblock_jz_invalid: 1 },
	{ name: "NAND VFAT partition",
	  offset:  512 * 0x100000,
	  size:    512 * 0x100000,
	  use_planes: 1,
	  mtdblock_jz_invalid: 0 },
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
					     2,		/* reserved blocks of mtd0 */
					     2,		/* reserved blocks of mtd1 */
					     10,	/* reserved blocks of mtd2 */
					     10,	/* reserved blocks of mtd3 */
					     20,	/* reserved blocks of mtd4 */
					     20};	/* reserved blocks of mtd5 */
#endif /* CONFIG_JZ4720_VIRGO */
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
	return partition_reserved_badblocks;
}
EXPORT_SYMBOL(get_jz_badblock_table);

/*-------------------------------------------------------------------------*/

static void jz_hwcontrol(struct mtd_info *mtd, int dat, 
			 unsigned int ctrl)
{
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	unsigned int nandaddr = (unsigned int)this->IO_ADDR_W;
	extern u8 nand_nce;  /* in nand_base.c, indicates which chip select is used for current nand chip */

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

		if ( ctrl & NAND_ALE )
			nandaddr = (unsigned int)((unsigned long)(this->IO_ADDR_W) | 0x00010000);
		else
			nandaddr = (unsigned int)((unsigned long)(this->IO_ADDR_W) & ~0x00010000);

		if ( ctrl & NAND_CLE )
			nandaddr = nandaddr | 0x00008000;
		else
			nandaddr = nandaddr & ~0x00008000;
	}

	this->IO_ADDR_W = (void __iomem *)nandaddr;
	if (dat != NAND_CMD_NONE)
		writeb(dat, this->IO_ADDR_W);
}

static int jz_device_ready(struct mtd_info *mtd)
{
	int ready, wait = 10;
	while (wait--);
	ready = __gpio_get_pin(94);
	return ready;
}

/*
 * EMC setup
 */
static void jz_device_setup(void)
{
// PORT 0:
//  ...
// PORT 1:
// PIN/BIT N		FUNC0		FUNC1
//	25		CS1#		-
//	26		CS2#		-
//	27		CS3#		-
//	28		CS4#		-
#define GPIO_CS2_N (32+26)
#define GPIO_CS3_N (32+27)
#define GPIO_CS4_N (32+28)
#define SMCR_VAL   0x0d221200

	/* Set NFE bit */
	REG_EMC_NFCSR |= EMC_NFCSR_NFE1;
	/* Read/Write timings */
	REG_EMC_SMCR1 = SMCR_VAL;

#if defined(CONFIG_MTD_NAND_CS2)
        /* Set CS2# pin as function 0 */
	__gpio_as_func0(GPIO_CS2_N); 
	REG_EMC_NFCSR |= EMC_NFCSR_NFE2;
	REG_EMC_SMCR2 = SMCR_VAL;
#endif

#if defined(CONFIG_MTD_NAND_CS3)
	__gpio_as_func0(GPIO_CS3_N);
	REG_EMC_NFCSR |= EMC_NFCSR_NFE3;
	REG_EMC_SMCR3 = SMCR_VAL;
#endif

#if defined(CONFIG_MTD_NAND_CS4)
	__gpio_as_func0(GPIO_CS4_N);
	REG_EMC_NFCSR |= EMC_NFCSR_NFE4;
	REG_EMC_SMCR4 = SMCR_VAL;
#endif
}

#ifdef CONFIG_MTD_HW_HM_ECC

static int jzsoc_nand_calculate_hm_ecc(struct mtd_info* mtd, 
				       const u_char* dat, u_char* ecc_code)
{
	unsigned int calc_ecc;
	unsigned char *tmp;
	
	__nand_ecc_disable();

	calc_ecc = ~(__nand_read_hm_ecc()) | 0x00030000;
	
	tmp = (unsigned char *)&calc_ecc;
	//adjust eccbytes order for compatible with software ecc	
	ecc_code[0] = tmp[1];
	ecc_code[1] = tmp[0];
	ecc_code[2] = tmp[2];
	
	return 0;
}

static void jzsoc_nand_enable_hm_hwecc(struct mtd_info* mtd, int mode)
{
 	__nand_ecc_enable();
	__nand_select_hm_ecc();
}

static int jzsoc_nand_hm_correct_data(struct mtd_info *mtd, u_char *dat,
				     u_char *read_ecc, u_char *calc_ecc)
{
	u_char a, b, c, d1, d2, d3, add, bit, i;
		
	/* Do error detection */ 
	d1 = calc_ecc[0] ^ read_ecc[0];
	d2 = calc_ecc[1] ^ read_ecc[1];
	d3 = calc_ecc[2] ^ read_ecc[2];

	if ((d1 | d2 | d3) == 0) {
		/* No errors */
		return 0;
	}
	else {
		a = (d1 ^ (d1 >> 1)) & 0x55;
		b = (d2 ^ (d2 >> 1)) & 0x55;
		c = (d3 ^ (d3 >> 1)) & 0x54;
		
		/* Found and will correct single bit error in the data */
		if ((a == 0x55) && (b == 0x55) && (c == 0x54)) {
			c = 0x80;
			add = 0;
			a = 0x80;
			for (i=0; i<4; i++) {
				if (d1 & c)
					add |= a;
				c >>= 2;
				a >>= 1;
			}
			c = 0x80;
			for (i=0; i<4; i++) {
				if (d2 & c)
					add |= a;
				c >>= 2;
				a >>= 1;
			}
			bit = 0;
			b = 0x04;
			c = 0x80;
			for (i=0; i<3; i++) {
				if (d3 & c)
					bit |= b;
				c >>= 2;
				b >>= 1;
			}
			b = 0x01;
			a = dat[add];
			a ^= (b << bit);
			dat[add] = a;
			return 0;
		}
		else {
			i = 0;
			while (d1) {
				if (d1 & 0x01)
					++i;
				d1 >>= 1;
			}
			while (d2) {
				if (d2 & 0x01)
					++i;
				d2 >>= 1;
			}
			while (d3) {
				if (d3 & 0x01)
					++i;
				d3 >>= 1;
			}
			if (i == 1) {
				/* ECC Code Error Correction */
				read_ecc[0] = calc_ecc[0];
				read_ecc[1] = calc_ecc[1];
				read_ecc[2] = calc_ecc[2];
				return 0;
			}
			else {
				/* Uncorrectable Error */
				printk("NAND: uncorrectable ECC error\n");
				return -1;
			}
		}
	}
	
	/* Should never happen */
	return -1;
}

#endif /* CONFIG_MTD_HW_HM_ECC */

#ifdef CONFIG_MTD_HW_RS_ECC

static void jzsoc_nand_enable_rs_hwecc(struct mtd_info* mtd, int mode)
{
	REG_EMC_NFINTS = 0x0;
 	__nand_ecc_enable();
	__nand_select_rs_ecc();

	if (mode == NAND_ECC_READ)
		__nand_rs_ecc_decoding();

	if (mode == NAND_ECC_WRITE)
		__nand_rs_ecc_encoding();
}		

static void jzsoc_rs_correct(unsigned char *dat, int idx, int mask)
{
	int i;

	idx--;

	i = idx + (idx >> 3);
	if (i >= 512)
		return;

	mask <<= (idx & 0x7);

	dat[i] ^= mask & 0xff;
	if (i < 511)
		dat[i+1] ^= (mask >> 8) & 0xff;
}

/*
 * calc_ecc points to oob_buf for us
 */
static int jzsoc_nand_rs_correct_data(struct mtd_info *mtd, u_char *dat,
				 u_char *read_ecc, u_char *calc_ecc)
{
	volatile u8 *paraddr = (volatile u8 *)EMC_NFPAR0;
	short k;
	u32 stat;

	/* Set PAR values */
	for (k = 0; k < PAR_SIZE; k++) {
		*paraddr++ = read_ecc[k];
	}

	/* Set PRDY */
	REG_EMC_NFECR |= EMC_NFECR_PRDY;

	/* Wait for completion */
	__nand_ecc_decode_sync();
	__nand_ecc_disable();

	/* Check decoding */
	stat = REG_EMC_NFINTS;

	if (stat & EMC_NFINTS_ERR) {
		/* Error occurred */
		if (stat & EMC_NFINTS_UNCOR) {
			printk("NAND: Uncorrectable ECC error\n");
			return -1;
		} else {
			u32 errcnt = (stat & EMC_NFINTS_ERRCNT_MASK) >> EMC_NFINTS_ERRCNT_BIT;
			switch (errcnt) {
			case 4:
				jzsoc_rs_correct(dat, (REG_EMC_NFERR3 & EMC_NFERR_INDEX_MASK) >> EMC_NFERR_INDEX_BIT, (REG_EMC_NFERR3 & EMC_NFERR_MASK_MASK) >> EMC_NFERR_MASK_BIT);
				/* FALL-THROUGH */
			case 3:
				jzsoc_rs_correct(dat, (REG_EMC_NFERR2 & EMC_NFERR_INDEX_MASK) >> EMC_NFERR_INDEX_BIT, (REG_EMC_NFERR2 & EMC_NFERR_MASK_MASK) >> EMC_NFERR_MASK_BIT);
				/* FALL-THROUGH */
			case 2:
				jzsoc_rs_correct(dat, (REG_EMC_NFERR1 & EMC_NFERR_INDEX_MASK) >> EMC_NFERR_INDEX_BIT, (REG_EMC_NFERR1 & EMC_NFERR_MASK_MASK) >> EMC_NFERR_MASK_BIT);
				/* FALL-THROUGH */
			case 1:
				jzsoc_rs_correct(dat, (REG_EMC_NFERR0 & EMC_NFERR_INDEX_MASK) >> EMC_NFERR_INDEX_BIT, (REG_EMC_NFERR0 & EMC_NFERR_MASK_MASK) >> EMC_NFERR_MASK_BIT);
				return 0;
			default:
				break;
	   		}
		}
	}

	return 0;
}

static int jzsoc_nand_calculate_rs_ecc(struct mtd_info* mtd, const u_char* dat,
				u_char* ecc_code)
{
	volatile u8 *paraddr = (volatile u8 *)EMC_NFPAR0;
	short i;

	__nand_ecc_encode_sync(); 
	__nand_ecc_disable();

	for(i = 0; i < PAR_SIZE; i++) {
		ecc_code[i] = *paraddr++;			
	}

	return 0;
}

#endif /* CONFIG_MTD_HW_RS_ECC */

/* Nand optimized functions */
static int dma_chan;
static unsigned int dma_src_phys_addr, dma_dst_phys_addr;
extern int jz_request_dma(int dev_id, const char *dev_str, 
			  irqreturn_t (*irqhandler)(int, void *),
			  unsigned long irqflags, void *irq_dev_id);

static void dma_setup(void)
{
	/* Request DMA channel and setup irq handler */
	dma_chan = jz_request_dma(DMA_ID_AUTO, "auto", NULL, IRQF_DISABLED, NULL);
	if (dma_chan < 0) {
		printk("Setup irq for nand failed!\n");
		return;
	} else
		printk("Nand DMA request channel %d.\n",dma_chan);
}

static void jz4740_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int i;
	struct nand_chip *chip = mtd->priv;

	if ((len <= 32) || (len & 0xf) || ((u32)buf >= (u32)high_memory))
	{
		for (i = 0; i < len; i++)
			buf[i] = readb(chip->IO_ADDR_R);
	} else {
		REG_DMAC_DRSR(dma_chan) = DMAC_DRSR_RS_AUTO;
		dma_src_phys_addr = CPHYSADDR(chip->IO_ADDR_R);
		dma_dst_phys_addr = CPHYSADDR(buf);
		dma_cache_inv((u32)buf, len);
		REG_DMAC_DSAR(dma_chan) = dma_src_phys_addr;
		REG_DMAC_DTAR(dma_chan) = dma_dst_phys_addr;
		REG_DMAC_DTCR(dma_chan) = len / 16;
		REG_DMAC_DCMD(dma_chan) = DMAC_DCMD_DAI | DMAC_DCMD_SWDH_8 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_16BYTE;
		REG_DMAC_DCCSR(dma_chan) = DMAC_DCCSR_NDES | DMAC_DCCSR_EN;
		REG_DMAC_DMACR = DMAC_DMACR_DMAE; /* global DMA enable bit */
		
		while(!(REG_DMAC_DCCSR(dma_chan) & DMAC_DCCSR_TT));
		REG_DMAC_DCCSR(dma_chan) &= ~DMAC_DCCSR_EN;  /* disable DMA */
		__dmac_channel_clear_transmit_end(dma_chan);
	}
}

static void jz4740_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	int i;
	struct nand_chip *chip = mtd->priv;

	if ((len <= 32) || (len & 0xf) || ((u32)buf >= (u32)high_memory))
	{
		for (i = 0; i < len; i++)
			writeb(buf[i], chip->IO_ADDR_W);
	} else {
		REG_DMAC_DRSR(dma_chan) = DMAC_DRSR_RS_AUTO;
		dma_dst_phys_addr = CPHYSADDR(chip->IO_ADDR_R);
		dma_src_phys_addr = CPHYSADDR(buf);
		dma_cache_wback((unsigned long)buf, len);
		REG_DMAC_DSAR(dma_chan) = dma_src_phys_addr;
		REG_DMAC_DTAR(dma_chan) = dma_dst_phys_addr;
		REG_DMAC_DTCR(dma_chan) = len / 16;
		REG_DMAC_DCMD(dma_chan) = DMAC_DCMD_SAI | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_8 | DMAC_DCMD_DS_16BYTE ;
		REG_DMAC_DCCSR(dma_chan) = DMAC_DCCSR_NDES | DMAC_DCCSR_EN;
		REG_DMAC_DMACR = DMAC_DMACR_DMAE; /* global DMA enable bit */
		
		while(!(REG_DMAC_DCCSR(dma_chan) & DMAC_DCCSR_TT));
		REG_DMAC_DCCSR(dma_chan) &= ~DMAC_DCCSR_EN;  /* disable DMA */
		__dmac_channel_clear_transmit_end(dma_chan);
	}
}

static int nand_read_page_hwecc_rs_planes(struct mtd_info *mtd, struct nand_chip *chip,
					  uint8_t *buf)
{
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps >> 1;
	uint8_t *p;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->ecc.layout->eccpos;
	uint32_t page;
	uint8_t flag = 0;
	int oobsize = mtd->oobsize >> 1;
	int ppb = mtd->erasesize / mtd->writesize;
	int ecctotal = chip->ecc.total >> 1;

	page = (global_page / ppb) * ppb + global_page; /* = global_page%ppb + (global_page/ppb)*ppb*2 */

	/* Read first page */
	chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
	chip->read_buf(mtd, chip->oob_poi, oobsize);
	for (i = 0; i < ecctotal; i++) {
		ecc_code[i] = chip->oob_poi[eccpos[i]];
		if (ecc_code[i] != 0xff) flag = 1;
	}

	p = buf;
	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, 0x00, -1);
	for (i = 0 ; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		int stat;
		if (flag) {
			chip->ecc.hwctl(mtd, NAND_ECC_READ);
			chip->read_buf(mtd, p, eccsize);
			stat = chip->ecc.correct(mtd, p, &ecc_code[i], &ecc_calc[i]);
			if (stat < 0)
				mtd->ecc_stats.failed++;
			else
				mtd->ecc_stats.corrected += stat;
		}
		else {
			chip->ecc.hwctl(mtd, NAND_ECC_READ);
			chip->read_buf(mtd, p, eccsize);
		}
	}
	/* Read second page */
	page += ppb;
	flag = 0;
	chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
	chip->read_buf(mtd, chip->oob_poi + oobsize, oobsize);
	for (i = 0; i < ecctotal; i++) {
		ecc_code[i] = chip->oob_poi[oobsize + eccpos[i]];
		if (ecc_code[i] != 0xff) flag = 1;
	}

	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, 0x00, -1);
	eccsteps = chip->ecc.steps >> 1;
	for (i = 0 ; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		int stat;
		if (flag) {
			chip->ecc.hwctl(mtd, NAND_ECC_READ);
			chip->read_buf(mtd, p, eccsize);
			stat = chip->ecc.correct(mtd, p, &ecc_code[i], &ecc_calc[i]);
			if (stat < 0)
				mtd->ecc_stats.failed++;
			else
				mtd->ecc_stats.corrected += stat;
		}
		else {
			chip->ecc.hwctl(mtd, NAND_ECC_READ);
			chip->read_buf(mtd, p, eccsize);
		}
	}

	return 0;
}

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
	/* Read second page OOB */
	page += ppb;
	if (sndcmd) {
		chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
		sndcmd = 0;
	}
	chip->read_buf(mtd, chip->oob_poi+oobsize, oobsize);
	return 0;
}

static int nand_write_oob_std_planes(struct mtd_info *mtd, struct nand_chip *chip,
				     int global_page)
{
	int status = 0,page;
	int pagesize = mtd->writesize >> 1;
	int oobsize = mtd->oobsize >> 1;
	int ppb = mtd->erasesize / mtd->writesize;
	const uint8_t *buf = chip->oob_poi;

	page = (global_page / ppb) * ppb + global_page; /* = global_page%ppb + (global_page/ppb)*ppb*2 */

        /* send cmd 0x80, the MSB should be valid if realplane is 4 */
	if (chip->realplanenum == 2)
		chip->cmdfunc(mtd, 0x80, pagesize, 0x00);
	else
		chip->cmdfunc(mtd, 0x80, pagesize, page & (1 << (chip->chip_shift - chip->page_shift)));

	chip->write_buf(mtd, buf, oobsize);
	/* Send first command to program the OOB data */
	chip->cmdfunc(mtd, 0x11, -1, -1);
	ndelay(100);
	status = chip->waitfunc(mtd, chip);

	page += ppb;
	buf += oobsize;
	chip->cmdfunc(mtd, 0x81, pagesize, page);
	chip->write_buf(mtd, buf, oobsize);
	/* Send command to program the OOB data */
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	/* Wait long R/B */
	ndelay(100);
	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

static void nand_write_page_hwecc_planes(struct mtd_info *mtd, struct nand_chip *chip,
					 const uint8_t *buf)
{
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps >> 1;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint8_t *p = (uint8_t *)buf;
	uint32_t *eccpos = chip->ecc.layout->eccpos;
	int oobsize = mtd->oobsize >> 1;
	int ppb = mtd->erasesize / mtd->writesize;
	int ecctotal = chip->ecc.total >> 1;
	int page;

	page = (global_page / ppb) * ppb + global_page; /* = global_page%ppb + (global_page/ppb)*ppb*2 */

        /* send cmd 0x80, the MSB should be valid if realplane is 4 */
	if (chip->realplanenum == 2)
		chip->cmdfunc(mtd, 0x80, 0x00, 0x00);
	else
		chip->cmdfunc(mtd, 0x80, 0x00, page & (1 << (chip->chip_shift - chip->page_shift)));

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		chip->ecc.hwctl(mtd, NAND_ECC_WRITE);
		chip->write_buf(mtd, p, eccsize);
		chip->ecc.calculate(mtd, p, &ecc_calc[i]);
	}
	for (i = 0; i < ecctotal; i++)
		chip->oob_poi[eccpos[i]] = ecc_calc[i];

	chip->write_buf(mtd, chip->oob_poi, oobsize);

	chip->cmdfunc(mtd, 0x11, -1, -1); /* send cmd 0x11 */
	ndelay(100);
	while(!chip->dev_ready(mtd));

	page += ppb;
	chip->cmdfunc(mtd, 0x81, 0x00, page); /* send cmd 0x81 */
	eccsteps = chip->ecc.steps >> 1;
	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		chip->ecc.hwctl(mtd, NAND_ECC_WRITE);
		chip->write_buf(mtd, p, eccsize);
		chip->ecc.calculate(mtd, p, &ecc_calc[i]);
	}

	for (i = 0; i < ecctotal; i++)
		chip->oob_poi[eccpos[i]] = ecc_calc[i];

	chip->write_buf(mtd, chip->oob_poi, oobsize);
}

static void single_erase_cmd_planes(struct mtd_info *mtd, int global_page)
{
	struct nand_chip *chip = mtd->priv;

	/* Send commands to erase a block */
	int page;
	int ppb = mtd->erasesize / mtd->writesize;

	page = (global_page / ppb) * ppb + global_page; /* = global_page%ppb + (global_page/ppb)*ppb*2 */

        /* send cmd 0x60, the MSB should be valid if realplane is 4 */
	if (chip->realplanenum == 2)
		chip->cmdfunc(mtd, 0x60, -1, 0x00);
	else
		chip->cmdfunc(mtd, 0x60, -1, page & (1 << (chip->chip_shift - chip->page_shift)));

	page += ppb;
	chip->cmdfunc(mtd, 0x60, -1, page & (~(ppb-1))); /* send cmd 0x60 */

	chip->cmdfunc(mtd, NAND_CMD_ERASE2, -1, -1); /* send cmd 0xd0 */
	/* Do not need wait R/B or check status */
}

/*
 * Main initialization routine
 */
int __init jznand_init(void)
{
	struct nand_chip *this;
	int ret, i;

	/* Allocate memory for MTD device structure and private data */
	jz_mtd = kmalloc (sizeof(struct mtd_info) + sizeof (struct nand_chip),
				GFP_KERNEL);
	if (!jz_mtd) {
		printk ("Unable to allocate JzSOC NAND MTD device structure.\n");
		return -ENOMEM;
	}

	jz_mtd1 = kmalloc (sizeof(struct mtd_info) + sizeof (struct nand_chip),
				GFP_KERNEL);
	if (!jz_mtd1) {
		printk ("Unable to allocate JzSOC NAND MTD device structure 1.\n");
		kfree(jz_mtd);
		return -ENOMEM;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *) (&jz_mtd[1]);

	/* Initialize structures */
	memset((char *) jz_mtd, 0, sizeof(struct mtd_info));
	memset((char *) this, 0, sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	jz_mtd->priv = this;

	/* Set & initialize NAND Flash controller */
	jz_device_setup();

        /* Set address of NAND IO lines */
        this->IO_ADDR_R = (void __iomem *) NAND_DATA_PORT1;
        this->IO_ADDR_W = (void __iomem *) NAND_DATA_PORT1;
        this->cmd_ctrl = jz_hwcontrol;
        this->dev_ready = jz_device_ready;

#ifdef CONFIG_MTD_HW_HM_ECC
	this->ecc.calculate = jzsoc_nand_calculate_hm_ecc;
	this->ecc.correct   = jzsoc_nand_hm_correct_data;
	this->ecc.hwctl     = jzsoc_nand_enable_hm_hwecc;
	this->ecc.mode      = NAND_ECC_HW;
	this->ecc.size      = 256;
	this->ecc.bytes     = 3;

#endif

#ifdef CONFIG_MTD_HW_RS_ECC
	this->ecc.calculate = jzsoc_nand_calculate_rs_ecc;
	this->ecc.correct   = jzsoc_nand_rs_correct_data;
	this->ecc.hwctl     = jzsoc_nand_enable_rs_hwecc;
	this->ecc.mode      = NAND_ECC_HW;
	this->ecc.size      = 512;
	this->ecc.bytes     = 9;
#endif

#ifdef  CONFIG_MTD_SW_HM_ECC	
	this->ecc.mode      = NAND_ECC_SOFT;
#endif
        /* 20 us command delay time */
        this->chip_delay = 20;

#ifdef  CONFIG_MTD_NAND_DMA
	dma_setup();
#endif
	/* Scan to find existance of the device */
	ret = nand_scan_ident(jz_mtd, NAND_MAX_CHIPS);
	if (!ret) {
		if (this->planenum == 2) {
			/* reset nand functions */
			this->erase_cmd = single_erase_cmd_planes;
			this->ecc.read_page = nand_read_page_hwecc_rs_planes;   //Muti planes read 
			this->ecc.write_page = nand_write_page_hwecc_planes;
			this->ecc.read_oob = nand_read_oob_std_planes;
			this->ecc.write_oob = nand_write_oob_std_planes;
#ifdef  CONFIG_MTD_NAND_DMA
			this->write_buf = jz4740_nand_write_buf;
			this->read_buf = jz4740_nand_read_buf;
#endif
			printk(KERN_INFO "Nand using two-plane mode, "
			       "and resized to writesize:%d oobsize:%d blocksize:0x%x \n",
			       jz_mtd->writesize, jz_mtd->oobsize, jz_mtd->erasesize);
		}
	}

	/* Determine whether all the partitions will use multiple planes if supported */
	nr_partitions = sizeof(partition_info) / sizeof(struct mtd_partition);
	all_use_planes = 1;
	for (i = 0; i < nr_partitions; i++) {
		all_use_planes &= partition_info[i].use_planes;
	}

	if (!ret)
		ret = nand_scan_tail(jz_mtd);

	if (ret){
		kfree (jz_mtd1);
		kfree (jz_mtd);
		return -ENXIO;
	}

	/* Register the partitions */
	printk (KERN_NOTICE "Creating %d MTD partitions on \"%s\":\n", nr_partitions, jz_mtd->name);

	if ((this->planenum == 2) && !all_use_planes) {
		for (i = 0; i < nr_partitions; i++) {
			if (partition_info[i].use_planes)
				add_mtd_partitions(jz_mtd, &partition_info[i], 1);
			else
				add_mtd_partitions(jz_mtd1, &partition_info[i], 1);
		}
	} else {
		kfree(jz_mtd1);
		add_mtd_partitions(jz_mtd, partition_info, nr_partitions);
	}
	return 0;
}
module_init(jznand_init);

/*
 * Clean up routine
 */
#ifdef MODULE
static void __exit jznand_cleanup(void)
{
	struct nand_chip *this = (struct nand_chip *) &jz_mtd[1];

	/* Unregister partitions */
	del_mtd_partitions(jz_mtd);
	
	/* Unregister the device */
	del_mtd_device (jz_mtd);

	/* Free internal data buffers */
	kfree (this->data_buf);

	/* Free the MTD device structure */
	if ((this->planenum == 2) && !all_use_planes)
		kfree (jz_mtd1);
	kfree (jz_mtd);
}
module_exit(jznand_cleanup);
#endif
