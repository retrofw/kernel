/*
 * linux/drivers/mtd/nand/jz4730_nand.c
 *
 * Copyright (c) 2005 - 2007 Ingenic Semiconductor Inc.
 *
 * Ingenic JZ4730 NAND driver
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

#define NAND_DATA_PORT		0xB4000000  /* read-write area */
#define __nand_ecc()		(REG_EMC_NFECC & 0x00ffffff)
#define __nand_ecc_enable()	(REG_EMC_NFCSR |= EMC_NFCSR_ECCE | EMC_NFCSR_ERST)
#define __nand_ecc_disable()	(REG_EMC_NFCSR &= ~EMC_NFCSR_ECCE)

/*
 * MTD structure for JzSOC board
 */
static struct mtd_info *jz_mtd = NULL;

/*
 * Define partitions for flash devices
 */
#ifdef CONFIG_JZ4730_PMP
static struct mtd_partition partition_info[] = {
	{ name: "NAND BOOT partition",
	  offset:  0 * 0x100000,
	  size:    4 * 0x100000 },
	{ name: "NAND KERNEL partition",
	  offset:  4 * 0x100000,
	  size:    4 * 0x100000 },
	{ name: "NAND ROOTFS partition",
	  offset:  8 * 0x100000,
	  size:    56 * 0x100000 },
	{ name: "NAND SSFDC partition",
	  offset:  64 * 0x100000,
	  size:    64 * 0x100000 },
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
static int partition_reserved_badblocks[]= {
					    2,		/* reserved blocks of mtd0 */
					    2,		/* reserved blocks of mtd1 */
					    10,		/* reserved blocks of mtd2 */
					    10};	/* reserved blocks of mtd3 */
#elif CONFIG_JZ4730_PMPV1
static struct mtd_partition partition_info[] = {
	{ name: "NAND ROOTFS partition",
	  offset:  3 * 0x100000,
	  size:    (32-3) * 0x100000 },
	{ name: "NAND DATAFS partition",
	  offset:  32 * 0x100000,
	  size:    32 * 0x100000 },
};
static int partition_reserved_badblocks[]={
					   10,
					   10};
#else
static struct mtd_partition partition_info[] = {
	{ name: "NAND ROOTFS partition",
	  offset:  3 * 0x100000,
	  size:    (128-3) * 0x100000 },
};
static int partition_reserved_badblocks[]={
					   20};
#endif

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

	if (ctrl & NAND_CTRL_CHANGE) {
		if ( ctrl & NAND_ALE )
			nandaddr = (unsigned int)((unsigned long)(this->IO_ADDR_W) | 0x00080000);
		else
			nandaddr = (unsigned int)((unsigned long)(this->IO_ADDR_W) & ~0x00080000);

		if ( ctrl & NAND_CLE )
			nandaddr = nandaddr | 0x00040000;
		else
			nandaddr = nandaddr & ~0x00040000;
		if ( ctrl & NAND_NCE )
			REG_EMC_NFCSR |= EMC_NFCSR_FCE;
		else
			REG_EMC_NFCSR &= ~EMC_NFCSR_FCE;
	}

	this->IO_ADDR_W = (void __iomem *)nandaddr;
	if (dat != NAND_CMD_NONE)
		writeb(dat , this->IO_ADDR_W);

}

static int jz_device_ready(struct mtd_info *mtd)
{
	int ready;
	ready = (REG_EMC_NFCSR & EMC_NFCSR_RB) ? 1 : 0;
	return ready;
}

/*
 * EMC setup
 */
static void jz_device_setup(void)
{
	/* Set NFE bit */
	REG_EMC_NFCSR |= EMC_NFCSR_NFE;
}

static void jzsoc_nand_enable_hwecc(struct mtd_info* mtd, int mode)
{
 	__nand_ecc_enable();
}

static int jzsoc_nand_calculate_ecc(struct mtd_info* mtd, const u_char* dat,
				u_char* ecc_code)
{
	unsigned int calc_ecc;
	unsigned char *tmp;
	
	__nand_ecc_disable();

	calc_ecc = ~(__nand_ecc()) | 0x00030000;
	
	tmp = (unsigned char *)&calc_ecc;

	ecc_code[0] = tmp[1];
	ecc_code[1] = tmp[0];
	ecc_code[2] = tmp[2];

	return 0;
}

/* ECC handling functions */

static int jzsoc_nand_correct_data(struct mtd_info *mtd, u_char *dat,
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
				printk("uncorrectable ECC error\n");
				return -1;
			}
		}
	}
	
	/* Should never happen */
	return -1;
}


/*
 * Main initialization routine
 */
int __init jznand_init(void)
{
	struct nand_chip *this;
	int nr_partitions;
 
	/* Allocate memory for MTD device structure and private data */
	jz_mtd = kmalloc (sizeof(struct mtd_info) + sizeof (struct nand_chip),
				GFP_KERNEL);
	if (!jz_mtd) {
		printk ("Unable to allocate JzSOC NAND MTD device structure.\n");
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
        this->IO_ADDR_R = (void __iomem *) NAND_DATA_PORT;
        this->IO_ADDR_W = (void __iomem *) NAND_DATA_PORT;
        this->cmd_ctrl = jz_hwcontrol;
        this->dev_ready = jz_device_ready;

#ifdef CONFIG_MTD_HW_HM_ECC
	this->ecc.calculate = jzsoc_nand_calculate_ecc;
	this->ecc.correct   = jzsoc_nand_correct_data;
	this->ecc.hwctl     = jzsoc_nand_enable_hwecc;
	this->ecc.mode      = NAND_ECC_HW;
	this->ecc.size      = 256;
	this->ecc.bytes     = 3;
	
#endif

#ifdef CONFIG_MTD_SW_HM_ECC	
	this->eccmode	    = NAND_ECC_SOFT;
#endif

        /* 20 us command delay time */
        this->chip_delay = 20;

	/* Scan to find existance of the device */
	if (nand_scan(jz_mtd, 1)) {
		kfree (jz_mtd);
		return -ENXIO;
	}

	/* Register the partitions */
	nr_partitions = sizeof(partition_info) / sizeof(struct mtd_partition);
	printk (KERN_NOTICE "Creating %d MTD partitions on \"%s\":\n", nr_partitions, jz_mtd->name);
	add_mtd_partitions(jz_mtd, partition_info, nr_partitions);

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
	kfree (jz_mtd);
}
module_exit(jznand_cleanup);
#endif
