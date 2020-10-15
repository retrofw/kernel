/*
 * linux/drivers/mtd/nand/jz4760_nand.c
 *
 * JZ4760 NAND driver
 *
 * Copyright (c) 2005 - 2007 Ingenic Semiconductor Inc.
 * Author: <lhhuang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*
NOTE: The OOB size is align with 4 bytes now.
If your nand's OOB size not align with 4,and all of the data in OOB area is valid,
please FIXUP the value of desc->dcnt when init the write/read dma descs, in jz4760b_nand_dma_init func.
<hpyang@ingenic.cn> 2011/04/02
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

#define BDMAC_DCMD_DS_NAND		BDMAC_DCMD_DS_64BYTE
#define DIV_DS_NAND				64

#define BDMAC_DCMD_DS_BCH		BDMAC_DCMD_DS_64BYTE
#define DIV_DS_BCH           	64

#ifdef CONFIG_MTD_NAND_BUS_WIDTH_16
#define BDMAC_DCMD_DWDH_XX	BDMAC_DCMD_DWDH_16
#else
#define BDMAC_DCMD_DWDH_XX	BDMAC_DCMD_DWDH_8
#endif

#define USE_DIRECT	1
#define	USE_PN		0
#define	USE_COUNTER	0
#define	COUNT_0		0	/* 1:count the number of 1; 0:count the number of 0 */
#define PN_ENABLE			(1 | PN_RESET)
#define PN_DISABLE			0
#define PN_RESET			(1 << 1)
#define COUNTER_ENABLE  	((1 << 3) | COUNTER_RESET)
#define COUNTER_DISABLE 	(0 << 3)
#define COUNTER_RESET   	(1 << 5)
#define COUNT_FOR_1			(0 << 4)
#define COUNT_FOR_0			(1 << 4)

#define DMA_DESC_FLUSH_SIZE	2048		// Make sure all the dma desc buffer we used be flushed.

#define MISC_DEBUG   0
#define BCH_DEBUG    0
#define DMA_DEBUG    0

#if MISC_DEBUG
#define	dprintk(n,x...) printk(n,##x)
#else
#define	dprintk(n,x...)
#endif

#if BCH_DEBUG
#define	bch_printk(n,x...) printk(n,##x)
#else
#define	bch_printk(n,x...)
#endif

#if defined(CONFIG_MTD_HW_BCH_24BIT)
#define __ECC_ENCODING __ecc_encoding_24bit
#define __ECC_DECODING __ecc_decoding_24bit
#define ERRS_SIZE       13	/* 13 words */
#define PAR_SIZE       	78	/* 24-bit */
#elif defined(CONFIG_MTD_HW_BCH_20BIT)
#define __ECC_ENCODING __ecc_encoding_20bit
#define __ECC_DECODING __ecc_decoding_20bit
#define ERRS_SIZE       11	/* 11 words */
#define PAR_SIZE       	65	/* 20-bit */
#elif defined(CONFIG_MTD_HW_BCH_16BIT)
#define __ECC_ENCODING __ecc_encoding_16bit
#define __ECC_DECODING __ecc_decoding_16bit
#define ERRS_SIZE       9	/* 9 words */
#define PAR_SIZE       	52	/* 16-bit */
#elif defined(CONFIG_MTD_HW_BCH_12BIT)
#define __ECC_ENCODING __ecc_encoding_12bit
#define __ECC_DECODING __ecc_decoding_12bit
#define ERRS_SIZE       7	/* 7 words */
#define PAR_SIZE       	39	/* 12-bit */
#elif defined(CONFIG_MTD_HW_BCH_8BIT)
#define __ECC_ENCODING __ecc_encoding_8bit
#define __ECC_DECODING __ecc_decoding_8bit
#define ERRS_SIZE       5	/* 5 words */
#define PAR_SIZE       	26	/* 8-bit */
#else
#define __ECC_ENCODING __ecc_encoding_4bit
#define __ECC_DECODING __ecc_decoding_4bit
#define ERRS_SIZE       3	/* 3 words */
#define PAR_SIZE       	13	/* 4-bit */
#endif

#define NAND_DATA_PORT1	       0xBA000000	/* read-write area in static bank 1 */
#define NAND_DATA_PORT2	       0xB4000000	/* read-write area in static bank 2 */
#define NAND_DATA_PORT3	       0xAC000000	/* read-write area in static bank 3 */
#define NAND_DATA_PORT4	       0xA8000000	/* read-write area in static bank 4 */

#define NAND_ADDR_OFFSET       0x00800000      /* address port offset for unshare mode */
#define NAND_CMD_OFFSET        0x00400000      /* command port offset for unshare mode */

#if defined(CONFIG_MTD_NAND_DMA)
#define  USE_IRQ      1
enum {
	NAND_NONE,
	NAND_PROG,
	NAND_READ
};
enum {
	BCH_NONE,
	BCH_ENC,
	BCH_DEC
};
static volatile u8 nand_status;
static volatile u8 bch_status;
static volatile int dma_ack = 0;
static volatile int dma_ack1 = 0;
static volatile int bch_enc_ack = 0;
static volatile int bch_dec_ack = 0;
static char nand_dma_chan = 1;	/* fixed to channel 1 */
static char bch_dma_chan = 0;	/* fixed to channel 0 */
static u32 *errs;
static u32 oob_bch_errs[NAND_MAX_ERRSIZE];
static jz_bdma_desc_8word *dma_desc_enc, *dma_desc_enc1, *dma_desc_dec, *dma_desc_dec1,
			  *dma_desc_nand_prog, *dma_desc_nand_read;
#if USE_PN
static jz_bdma_desc_8word *dma_desc_pPN, *dma_desc_rPN;
#endif
static u32 *pval_nand_ddr;
static u32 *pval_nand_cmd_pgprog; /* for sending 0x11 or 0x10 when programing*/
#if defined(CONFIG_MTD_NAND_DMABUF)
u8 *prog_buf, *read_buf;
#endif
#if USE_PN
static u32 *pn_buf;
#endif
DECLARE_WAIT_QUEUE_HEAD(nand_prog_wait_queue);
DECLARE_WAIT_QUEUE_HEAD(nand_read_wait_queue);
#endif		/* CONFIG_MTD_NAND_DMA */

struct buf_be_corrected {
	u8 *data;
	u8 *oob;
	u8 eccsize;
};

static u32 addr_offset;
static u32 cmd_offset;

extern int global_page; /* for two-plane operations */
extern int global_mafid; /* ID of manufacture */

/*
 * MTD structure for JzSOC board
 */
static struct mtd_info *jz_mtd = NULL;
extern struct mtd_info *jz_mtd1;
extern char all_use_planes;

/*
 * Define partitions for flash devices
 */
#if defined(CONFIG_JZ4760_CYGNUS) || defined(CONFIG_JZ4760_LEPUS) || defined(CONFIG_JZ4760B_CYGNUS) || defined(CONFIG_JZ4760B_LEPUS) || defined(CONFIG_JZ4760_HTB80)
static struct mtd_partition partition_info[] = {
	{name:"NAND BOOT partition",
	  rl_offset:0 * 0x100000LL,
	  rl_size:4 * 0x100000LL,
	  use_planes: 0},
	{name:"NAND KERNEL partition",
	  rl_offset:4 * 0x100000LL,
	  rl_size:4 * 0x100000LL,
	  use_planes: 0},
	{name:"NAND ROOTFS partition",
	  rl_offset:8 * 0x100000LL,
	  rl_size:504 * 0x100000LL,
	  use_planes: 0},
	{name:"NAND DATA partition",
	  rl_offset:512 * 0x100000LL,
	  rl_size:512 * 0x100000LL,
	  use_planes: 1},
	{name:"NAND VFAT partition",
	  rl_offset:1024 * 0x100000LL,
	  rl_size:1024 * 0x100000LL,
	  use_planes: 1},
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
	20			/* reserved blocks of mtd4 */
};				/* reserved blocks of mtd5 */
#endif				/* CONFIG_JZ4760_CYGNUS || CONFIG_JZ4760_LEPUS */

#if defined(CONFIG_JZ4760_ALTAIR) || defined(CONFIG_JZ4760B_ALTAIR)

/* Reserve 32MB for bootloader, splash1, splash2 and radiofw */
#define MISC_OFFSET		(32  * 0x100000LL)

#define MISC_SIZE		(  1 * 0x100000LL)
#define RECOVERY_SIZE		(  5 * 0x100000LL)
#define BOOT_SIZE		(  4 * 0x100000LL)
#define SYSTEM_SIZE		(90 * 0x100000LL)
#define USERDATA_SIZE		(90 * 0x100000LL)
#define CACHE_SIZE		(32 * 0x100000LL)
#define STORAGE_SIZE		(MTDPART_SIZ_FULL)

static struct mtd_partition partition_info[] = {

	/* Android partitions:
	 *
	 * misc@mtd0 :      raw
	 * recovery@mtd1:   raw
	 * boot@mtd2:       raw
	 * system@mtd3:     yaffs2
	 * userdata@mtd4:   yaffs2
	 * cache@mtd5:      yaffs2
	 * storage@mtd6:    vfat
	 */
	{name:       "misc",
offset:     MISC_OFFSET,
	    real_size:       MISC_SIZE,
	    use_planes: 0},
	{name:       "recovery",
offset:     (MISC_OFFSET+MISC_SIZE),
	    real_size:       RECOVERY_SIZE,
	    use_planes: 0},
	{name:       "boot",
offset:     (MISC_OFFSET+MISC_SIZE+RECOVERY_SIZE),
	    real_size:       BOOT_SIZE,
	    use_planes: 0},
	{name:       "system",
offset:     (MISC_OFFSET+MISC_SIZE+RECOVERY_SIZE+BOOT_SIZE),
	    real_size:       SYSTEM_SIZE,
	    use_planes: 0},
	{name:       "userdata",
offset:     (MISC_OFFSET+MISC_SIZE+RECOVERY_SIZE+BOOT_SIZE+SYSTEM_SIZE),
	    real_size:       USERDATA_SIZE,
	    use_planes: 0},
	{name:       "cache",
offset:     (MISC_OFFSET+MISC_SIZE+RECOVERY_SIZE+BOOT_SIZE+SYSTEM_SIZE+USERDATA_SIZE),
	    real_size:       CACHE_SIZE,
	    use_planes: 0},
	{name:       "storage",
offset:     (MISC_OFFSET+MISC_SIZE+RECOVERY_SIZE+BOOT_SIZE+SYSTEM_SIZE+USERDATA_SIZE+CACHE_SIZE),
	    real_size:       STORAGE_SIZE,
	    use_planes: 0}
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
	10,			/* reserved blocks of mtd0 */
	10,			/* reserved blocks of mtd1 */
	10,			/* reserved blocks of mtd2 */
	10,			/* reserved blocks of mtd3 */
	10,			/* reserved blocks of mtd4 */
	10,			/* reserved blocks of mtd5 */
	12			/* reserved blocks of mtd6 */
};
#endif	                        /* CONFIG_JZ4760_ALTAIR */

/*-------------------------------------------------------------------------
 * Following three functions are exported and used by the mtdblock-jz.c
 * NAND FTL driver only.
 */

extern void buffer_dump(uint8_t *buffer, int length, const char *comment, char *file, char *function, int line);


static void dumpdata(u8 *databuf, u32 len)
{
	int i = 0;

	while(len --) {
		printk("0x%02x ", *(databuf++));
		i++;

		if (!(i % 16))
			printk("\n");
	}
	printk("\n");
}

static void calc_partition_size(struct mtd_info *mtd)
{
	int total_partitions,count;
	struct nand_chip *this = mtd->priv;
	total_partitions = sizeof(partition_info) / sizeof(struct mtd_partition);
	for(count = 0; count < total_partitions; count++){ //For the partition which accessed by driver must use -o mode
		partition_info[count].size = partition_info[count].rl_size
			- (partition_info[count].rl_size >> this->page_shift) * mtd->freesize;
		partition_info[count].offset = partition_info[count].rl_offset
			- (partition_info[count].rl_offset >> this->page_shift) * mtd->freesize;
		if(mtd_mod_by_eb(partition_info[count].size, mtd)){
			partition_info[count].size -= mtd_mod_by_eb(partition_info[count].size, mtd);
		}
	}
}

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
					REG_NEMC_NFCSR = NEMC_NFCSR_NFCE1 | NEMC_NFCSR_NFE1;
					break;
				case NAND_NCE2:
					this->IO_ADDR_W = this->IO_ADDR_R = (void __iomem *)NAND_DATA_PORT2;
					REG_NEMC_NFCSR = NEMC_NFCSR_NFCE2 | NEMC_NFCSR_NFE2;
					break;
				case NAND_NCE3:
					this->IO_ADDR_W = this->IO_ADDR_R = (void __iomem *)NAND_DATA_PORT3;
					REG_NEMC_NFCSR = NEMC_NFCSR_NFCE3 | NEMC_NFCSR_NFE3;
					break;
				case NAND_NCE4:
					this->IO_ADDR_W = this->IO_ADDR_R = (void __iomem *)NAND_DATA_PORT4;
					REG_NEMC_NFCSR = NEMC_NFCSR_NFCE4 | NEMC_NFCSR_NFE4;
					break;
				default:
					printk("error: no nand_nce 0x%x\n",nand_nce);
					break;
			}
		} else {

			REG_NEMC_NFCSR = 0;
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
	ready = ((REG_GPIO_PXPIN(0) & 0x00100000) ? 1 : 0);
	return ready;
}

/*
 * NEMC setup
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
	//	25		CS5#		-
	//	26		CS6#		-
#define GPIO_CS2_N (32*0+22)
#define GPIO_CS3_N (32*0+23)
#define GPIO_CS4_N (32*0+24)
#define GPIO_CS5_N (32*0+25)
#define GPIO_CS6_N (32*0+26)

#ifdef CONFIG_MTD_NAND_BUS_WIDTH_16
#define SMCR_VAL   0x11444440
	//#define SMCR_VAL   0x0fff7740  //slowest
	__gpio_as_nand_16bit(1);
#else
#define SMCR_VAL   0x11444400
	//#define SMCR_VAL   0x0fff7700  //slowest
	__gpio_as_nand_8bit(1);
#endif

	/* Read/Write timings */
	REG_NEMC_SMCR1 = SMCR_VAL;

#if defined(CONFIG_MTD_NAND_CS2)
	__gpio_as_func0(GPIO_CS2_N);

	/* Read/Write timings */
	REG_NEMC_SMCR2 = SMCR_VAL;
#endif

#if defined(CONFIG_MTD_NAND_CS3)
	__gpio_as_func0(GPIO_CS3_N);

	/* Read/Write timings */
	REG_NEMC_SMCR3 = SMCR_VAL;
#endif

#if defined(CONFIG_MTD_NAND_CS4)
	__gpio_as_func0(GPIO_CS4_N);

	/* Read/Write timings */
	REG_NEMC_SMCR4 = SMCR_VAL;
#endif

#if defined(CONFIG_MTD_NAND_CS5)
	__gpio_as_func0(GPIO_CS5_N);

	/* Read/Write timings */
	REG_NEMC_SMCR5 = SMCR_VAL;
#endif

#if defined(CONFIG_MTD_NAND_CS6)
	__gpio_as_func0(GPIO_CS6_N);

	/* Read/Write timings */
	REG_NEMC_SMCR6 = SMCR_VAL;
#endif
}

#ifdef CONFIG_MTD_HW_BCH_ECC

static void jzsoc_nand_enable_bch_hwecc(struct mtd_info *mtd, int mode)
{
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	int ecc_pos = this->eccpos;
	int eccsize = this->ecc.size;

	REG_BCH_INTS = 0xffffffff;
	if (mode == NAND_ECC_READ) {
		__ECC_DECODING();
		__ecc_cnt_dec(eccsize * 2 + PAR_SIZE);
#if defined(CONFIG_MTD_NAND_DMA)
		__ecc_dma_enable();
#endif
	}

	if (mode == NAND_ECC_WRITE) {
		__ECC_ENCODING();
		__ecc_cnt_enc(eccsize * 2);
#if defined(CONFIG_MTD_NAND_DMA)
		__ecc_dma_enable();
#endif
	}

	if (mode == NAND_READ_OOB) {
		__ECC_DECODING();
		__ecc_cnt_dec(ecc_pos * 2 + PAR_SIZE);
		REG_BCH_INTES = BCH_INTES_DECFES;
		REG_BCH_INTEC = ~BCH_INTEC_DECFEC;
	}

	if (mode == NAND_WRITE_OOB) {
		__ECC_ENCODING();
		__ecc_cnt_enc(ecc_pos * 2);
		REG_BCH_INTES = BCH_INTES_ENCFES;
		REG_BCH_INTEC = ~BCH_INTEC_ENCFEC;
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
	int ecc_pos = this->eccpos;
	//	int eccsize = this->ecc.size;
	int eccsize = ((struct buf_be_corrected *)dat)->eccsize;
	int i, bit;		/* the 'bit' of i byte is error */

	if (eccsize != ecc_pos)
		eccsize = this->ecc.size;

	i = (idx - 1) >> 3;
	bit = (idx - 1) & 0x7;

	dprintk("error:i=%d, bit=%d\n",i,bit);
	if (i < eccsize) {
		((struct buf_be_corrected *)dat)->data[i] ^= (1 << bit);
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
	u32 stat, i;
	u32 *errs = (u32 *)errs0;
	int ret = 0;

	if (REG_BDMAC_DCCSR(0) & BDMAC_DCCSR_BUERR) {
		printk("NAND: Uncorrectable ECC error**\n");
		return -1;
	}

	if (REG_BDMAC_DCCSR(0) & BDMAC_DCCSR_BERR) {
		stat = errs[0];
		dprintk("stat=%x err0:%x err1:%x \n", stat, errs[1], errs[2]);

		if (stat & BCH_INTS_ERR) {
			if (stat & BCH_INTS_UNCOR) {
				dprintk("NAND: Uncorrectable ECC error\n");
				return -1;
			} else {
				u32 errcnt = (stat & BCH_INTS_ERRC_MASK) >> BCH_INTS_ERRC_BIT;
				if(errcnt > 24)
					printk("NAND:err count[%d] is too big\n",errcnt);
				else {
					/*begin at the second DWORD*/
					errs = (u32 *)&errs0[4];
					for(i = 0; i < errcnt; i++) {
						/* errs[i>>1] get the error report regester value,
						 * i the error bit index.
						 * errs[i>>1] >> ((i % 2) << 4) means when error
						 * bit index is odd, errs[i>>1] >> 16*/
						bch_correct(mtd, dat, ((errs[i>>1] >> ((i % 2) << 4))) & BCH_ERR_INDEX_MASK);
					}
				}
			}
		}
	}

	return ret;
}

#else				/* cpu mode */

static int jzsoc_nand_bch_correct_data(struct mtd_info *mtd, u_char * dat, u_char * read_ecc, u_char * calc_ecc)
{
	return jzsoc_nand_bch_correct_data_misc(mtd, dat, read_ecc, calc_ecc);
}

#endif				/* CONFIG_MTD_NAND_DMA */

/**
 * jzsoc_nand_bch_correct_data_misc
 * @mtd:	mtd info structure
 * @dat:        data to be corrected
 * @read_ecc:   pointer to ecc buffer calculated when nand writing
 * @calc_ecc:   no used
 */
static int jzsoc_nand_bch_correct_data_misc(struct mtd_info *mtd, u_char * dat, u_char * read_ecc, u_char * calc_ecc)
{
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	int eccsize = ((struct buf_be_corrected *)dat)->eccsize;
	int eccbytes = this->ecc.bytes;
	int ecc_pos = this->eccpos;
	short k;
	u32 stat, i;
	int ret = 0;

	if (eccsize != ecc_pos)
		eccsize = this->ecc.size;

	/* Write data to REG_BCH_DR */
	for (k = 0; k < eccsize; k++) {
		REG_BCH_DR = ((struct buf_be_corrected *)dat)->data[k];
	}

	/* Write parities to REG_BCH_DR */
	for (k = 0; k < eccbytes; k++) {
		REG_BCH_DR = read_ecc[k];
	}

#if MISC_DEBUG

		buffer_dump(((struct buf_be_corrected *)dat)->data, eccsize, "data",	\
				 __FILE__, __func__, __LINE__);
		buffer_dump(read_ecc, eccbytes, "par", __FILE__, __func__, __LINE__);
#endif

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
			u32 *errs = (u32 *)BCH_ERR0;

			for(i = 0; i < errcnt; i++) {
				/* errs[i>>1] get the error report regester value,
				 * (i+1) the error bit index.
				 * errs[i>>1] >> (((i + 1) % 2) << 4) means when error
				 * bit index is even, errs[i>>1] >> 16*/
				bch_correct(mtd, dat, ((errs[i>>1] >> ((i % 2) << 4))) & BCH_ERR_INDEX_MASK);
			}
		}
	}

	return ret;
}

static int jzsoc_nand_calculate_bch_ecc(struct mtd_info *mtd, const u_char * dat, u_char * ecc_code)
{
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	int eccsize = this->ecc.size;
	int eccbytes = this->ecc.bytes;
	volatile u8 *paraddr = (volatile u8 *)BCH_PAR0;
	short i;

	/* Write data to REG_BCH_DR */
	for (i = 0; i < eccsize; i++) {
		REG_BCH_DR = ((struct buf_be_corrected *)dat)->data[i];
	}
	__ecc_encode_sync();
	__ecc_disable();

	for (i = 0; i < eccbytes; i++) {
		ecc_code[i] = *paraddr++;
	}

	return 0;
}

static int nand_oob_hwecc_bchenc(struct mtd_info *mtd, struct nand_chip *chip)
{
	int ecc_pos = chip->eccpos;
	volatile u8 *paraddr;
	u8 *oobbuf = chip->oob_poi;
	u8 *parbuf = oobbuf + ecc_pos;
	int eccbytes = chip->ecc.bytes;
	int count;

	paraddr = (volatile u8 *)BCH_PAR0;

	chip->ecc.hwctl(mtd, NAND_WRITE_OOB);

	for(count = 0; count < ecc_pos; count++)
		REG_BCH_DR = *oobbuf++;

	__ecc_encode_sync();

	__ecc_disable();

	for (count = 0; count < eccbytes; count++)
		*parbuf++ = *paraddr++;

	return 0;
}

#if defined(CONFIG_MTD_NAND_DMA)

#if DMA_DEBUG
static void save_dma_regs(int channel,u32 *val)
{
	int i = 0;
	val[i++]= REG_BDMAC_DSAR(channel);
	val[i++]= REG_BDMAC_DTAR(channel);
	val[i++]= REG_BDMAC_DTCR(channel);
	val[i++]= REG_BDMAC_DRSR(channel);
	val[i++]= REG_BDMAC_DCCSR(channel);
	val[i++]= REG_BDMAC_DCMD(channel);
	val[i++]= REG_BDMAC_DDA(channel);
	val[i++]= REG_BDMAC_DSD(channel);
	val[i++]= REG_BDMAC_DNT(channel);
	val[i++]= REG_BDMAC_DMACR;
	val[i++]= REG_BDMAC_DMAIPR;
	val[i++]= REG_BDMAC_DMADBR;
	val[i++]= REG_BDMAC_DMADBSR;
	val[i++]= REG_BDMAC_DMACKE;
}

static void dump_save_dma_regs(int channel,u32 *val)
{
	int i = 0;
	printk("\nsaved channel %d\n",channel);
	printk("DSA: %x\n",val[i++]);
	printk("DTA: %x\n",val[i++]);
	printk("DTC: %x\n",val[i++]);
	printk("DRT: %x\n",val[i++]);
	printk("DCS: %x\n",val[i++]);
	printk("DCM: %x\n",val[i++]);
	printk("DDA: %x\n",val[i++]);
	printk("DSD: %x\n",val[i++]);
	printk("DNT: %x\n",val[i++]);
	printk("DMAC: %x\n",val[i++]);
	printk("DIRQP: %x\n",val[i++]);
	printk("DDR: %x\n",val[i++]);
	printk("DDRS: %x\n",val[i++]);
	printk("DCKE: %x\n",val[i++]);
}

#endif

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
	int ecc_pos = chip->eccpos;
	int freesize = mtd->freesize / chip->planenum;
	int oobsize = mtd->oobsize / chip->planenum;
	int i, err, timeout;
	const u8 *databuf;
	u8 *oobbuf;
	jz_bdma_desc_8word *desc;

#if DMA_DEBUG
	u32 bch_dma_regs[14];
	u32 nand_dma_regs[14];
#endif

	nand_oob_hwecc_bchenc(mtd, chip);

	databuf = buf;
	oobbuf = chip->oob_poi;
	memset(oobbuf + oobsize, 0xff, mtd->freesize);

	/* descriptors for encoding data blocks */
	desc = dma_desc_enc1;
	for (i = 0; i < eccsteps; i++) {
		desc->dsadr = CPHYSADDR((u32)databuf) + i * eccsize;	/* DMA source address */
		desc->dtadr = CPHYSADDR((u32)oobbuf) + ecc_pos + (i + 1) * eccbytes;	/* DMA target address */
		desc++;
	}

	/* descriptor for nand programing data block */
	desc = dma_desc_nand_prog;
	desc->dsadr = CPHYSADDR((u32)databuf);	/* DMA source address */
	desc->dtadr = CPHYSADDR((u32)chip->IO_ADDR_W);  /* It will be changed when using multiply chip select */

	if (freesize != 0) {
		/* descriptor for nand programing free block */
		desc++;
		desc->dsadr = CPHYSADDR((u32)oobbuf) + oobsize;
		desc->dtadr = CPHYSADDR((u32)chip->IO_ADDR_W);

	}

	/* descriptor for nand programing oob block */
	desc++;
	desc->dsadr = CPHYSADDR((u32)oobbuf);	/* DMA source address */
	desc->dtadr = CPHYSADDR((u32)chip->IO_ADDR_W);  /* It will be changed when using multiply chip select */

	/* descriptor for __nand_cmd(CMD_PGPROG) */
	desc++;
	*pval_nand_cmd_pgprog = cmd_pgprog | 0x40000000;
	desc->dsadr = CPHYSADDR((u32)pval_nand_cmd_pgprog);
	desc->dtadr = CPHYSADDR((u32)chip->IO_ADDR_R);	/* DMA target address: cmdport */
	if (cmd_pgprog == 0x10)
		desc->dcmd |= BDMAC_DCMD_LINK;  /* __nand_sync() by a DMA descriptor */
	else if (cmd_pgprog == 0x11)
		desc->dcmd &= ~BDMAC_DCMD_LINK; /* __nand_sync() by polling */

	dma_cache_wback_inv((u32)dma_desc_enc, DMA_DESC_FLUSH_SIZE);

	dma_cache_wback_inv((u32)databuf, mtd->writesize);
	dma_cache_wback_inv((u32)oobbuf, oobsize + mtd->freesize);
	/* 4*6: pval_nand_ddr, pval_nand_dcs, pval_bch_ddr, pval_bch_dcs, dummy, pval_nand_cmd_pgprog */
	dma_cache_wback_inv((u32)pval_nand_ddr, 4 * 8); /* 8 words, a cache line */

	REG_BDMAC_DCCSR(bch_dma_chan) = 0;
	REG_BDMAC_DCCSR(nand_dma_chan) = 0;

	/* Setup DMA descriptor address */
	REG_BDMAC_DDA(bch_dma_chan) = CPHYSADDR((u32)dma_desc_enc1);
#if USE_PN
	REG_BDMAC_DDA(nand_dma_chan) = CPHYSADDR((u32)dma_desc_pPN);
#else
	REG_BDMAC_DDA(nand_dma_chan) = CPHYSADDR((u32)dma_desc_nand_prog);
#endif

	/* Setup request source */
	REG_BDMAC_DRSR(bch_dma_chan) = BDMAC_DRSR_RS_BCH_ENC;
	REG_BDMAC_DRSR(nand_dma_chan) = BDMAC_DRSR_RS_AUTO;

	/* Setup DMA channel control/status register */
	REG_BDMAC_DCCSR(bch_dma_chan) = BDMAC_DCCSR_DES8 | BDMAC_DCCSR_EN;	/* descriptor transfer, clear status, start channel */
	/* Enable DMA */
	REG_BDMAC_DMACR |= BDMAC_DMACR_DMAE;

	/* Enable BCH encoding */
	chip->ecc.hwctl(mtd, NAND_ECC_WRITE);

	dma_ack1 = 0;
	nand_status = NAND_PROG;

#if DMA_DEBUG
	save_dma_regs(bch_dma_chan,bch_dma_regs);
	save_dma_regs(nand_dma_chan,nand_dma_regs);
#endif

	/* DMA doorbell set -- start DMA now ... */
	__bdmac_channel_set_doorbell(bch_dma_chan);

#if USE_IRQ
	if (cmd_pgprog == 0x10) {
		dprintk("nand prog before wake up\n");
		do {
			dprintk("enter...\n");
			err = wait_event_interruptible_timeout(nand_prog_wait_queue, dma_ack1, 3 * HZ);
			dprintk("exit.\n");
		}while(err == -ERESTARTSYS);

		nand_status = NAND_NONE;
		dprintk("nand prog after wake up\n");
		if (!err) {
			printk("*** NAND WRITE, Warning, wait event 3s timeout!\n");
			dump_jz_bdma_channel(0);
			dump_jz_bdma_channel(nand_dma_chan);
			printk("REG_BCH_CR=%x REG_BCH_CNT=0x%x REG_BCH_INTS=%x\n", REG_BCH_CR, REG_BCH_CNT, REG_BCH_INTS);
#if DMA_DEBUG
			dump_save_dma_regs(bch_dma_chan,bch_dma_regs);
			dump_save_dma_regs(nand_dma_chan,nand_dma_regs);
#endif
		}
		dprintk("timeout remain = %d\n", err);
	} else if (cmd_pgprog == 0x11) {
		timeout = 100000;
		while ((!__bdmac_channel_transmit_end_detected(nand_dma_chan)) && (timeout--));
		if (timeout <= 0)
			printk("two-plane prog 0x11 timeout!\n");
	}
#else
	timeout = 100000;
	while ((!__bdmac_channel_transmit_end_detected(nand_dma_chan)) && (timeout--));
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
	int pagesize = mtd->rl_writesize >> 1;
	int ppb = mtd->rl_erasesize / mtd->rl_writesize;

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

static int nand_write_oob_hwecc_bch(struct mtd_info *mtd, struct nand_chip *chip, int page)
{
	int status = 0;
	int ecc_pos = chip->eccpos;
#if USE_IRQ
	int err;
#else
	volatile int timeout;
#endif

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, mtd->rl_writesize, page);

	nand_oob_hwecc_bchenc(mtd, chip);

	dma_cache_inv((u32)(chip->oob_poi), mtd->oobsize);

	__bdmac_channel_enable_clk(nand_dma_chan);

	REG_BDMAC_DCCSR(nand_dma_chan) = 0;

	REG_BDMAC_DRSR(nand_dma_chan) = BDMAC_DRSR_RS_AUTO;

	REG_BDMAC_DSAR(nand_dma_chan) = CPHYSADDR((u32)(chip->oob_poi));
	REG_BDMAC_DTAR(nand_dma_chan) = CPHYSADDR((u32)(chip->IO_ADDR_W));
	REG_BDMAC_DTCR(nand_dma_chan) = ecc_pos + chip->ecc.bytes;
#if USE_IRQ
	REG_BDMAC_DCMD(nand_dma_chan) = BDMAC_DCMD_SAI | BDMAC_DCMD_DAI | BDMAC_DCMD_SWDH_8 |
				BDMAC_DCMD_DWDH_8 | BDMAC_DCMD_DS_8BIT | BDMAC_DCMD_TIE;

	dma_ack1 = 0;
	nand_status = NAND_PROG;
#else
	REG_BDMAC_DCMD(nand_dma_chan) = BDMAC_DCMD_SAI | BDMAC_DCMD_DAI | BDMAC_DCMD_SWDH_8 |
				BDMAC_DCMD_DWDH_8 | BDMAC_DCMD_DS_8BIT;
#endif

	/* Enable DMA */
	REG_BDMAC_DMACR |= BDMAC_DMACR_DMAE;

	/* Setup DMA channel control/status register */
	REG_BDMAC_DCCSR(nand_dma_chan) |= BDMAC_DCCSR_NDES | BDMAC_DCCSR_EN;


#if USE_IRQ
	dprintk("nand prog before wake up\n");
	do {
		err = wait_event_interruptible_timeout(nand_prog_wait_queue, dma_ack1, 3 * HZ);
	}while(err == -ERESTARTSYS);
	nand_status = NAND_NONE;
	dprintk("nand prog after wake up\n");
	if (!err) {
		printk("*** NAND WRITE, Warning, wait event 3s timeout!\n");
		dump_jz_bdma_channel(0);
		dump_jz_bdma_channel(nand_dma_chan);
		printk("REG_BCH_CR=%x REG_BCH_CNT=0x%x REG_BCH_INTS=%x\n", REG_BCH_CR, REG_BCH_CNT, REG_BCH_INTS);
	}
	dprintk("timeout remain = %d\n", err);
#else
	timeout = 100000;
	while ((!__bdmac_channel_transmit_end_detected(nand_dma_chan)) && (timeout--));
	while(!chip->dev_ready(mtd));
	if (timeout <= 0)
		printk("not use irq, prog timeout!\n");
#endif

	/* Send command to program the OOB data */
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

#else	/* nand write in cpu mode */

static void nand_write_page_hwecc_bch(struct mtd_info *mtd, struct nand_chip *chip,
		const uint8_t *buf)
{
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps / chip->planenum;
	int ecc_pos = chip->eccpos;
	int oob_per_eccsize = ecc_pos / eccsteps;
	int oobsize = mtd->oobsize / chip->planenum;
	int ecctotal = chip->ecc.total / chip->planenum;
	uint8_t *p = (uint8_t *)buf;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint32_t *eccpos = chip->ecc.layout->eccpos;
	static struct buf_be_corrected buf_calc0;
	struct buf_be_corrected *buf_calc = &buf_calc0;

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

/* nand write using two-plane mode */
static void nand_write_page_hwecc_bch_planes(struct mtd_info *mtd, struct nand_chip *chip,
		const uint8_t *buf)
{
	int pagesize = mtd->rl_writesize >> 1;
	int ppb = mtd->rl_erasesize / mtd->rl_writesize;
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

	nand_write_page_hwecc_bch(mtd, chip, buf);

	chip->cmdfunc(mtd, 0x11, -1, -1); /* send cmd 0x11 */
	ndelay(100);
	while(!chip->dev_ready(mtd));

	chip->cmdfunc(mtd, 0x81, 0x00, page + ppb); /* send cmd 0x81 */
	nand_write_page_hwecc_bch(mtd, chip, buf + pagesize);
}
#endif				/* CONFIG_MTD_NAND_DMA */

static void bch_decode_oob(struct mtd_info *mtd, struct nand_chip *chip)
{
	int ecc_pos = chip->eccpos;
	struct buf_be_corrected buf_correct0;
	struct buf_be_corrected *buf_correct = &buf_correct0;
	int stat;
	u8 *oobbuf;

	oobbuf = chip->oob_poi;
	buf_correct->data = oobbuf;
	buf_correct->eccsize = ecc_pos;
	chip->ecc.hwctl(mtd, NAND_READ_OOB);
	stat = jzsoc_nand_bch_correct_data_misc(mtd, (u8 *)buf_correct, oobbuf + ecc_pos, NULL);
	if (stat < 0){
		printk("OOB:ecc Uncorrectable:global_page = %d,chip->planenum = %d\n",global_page,chip->planenum);
		mtd->ecc_stats.failed++;
	}else{
		mtd->ecc_stats.corrected += stat;
	}
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
	int ecc_pos = chip->eccpos;
	int pagesize = mtd->rl_writesize / chip->planenum;
	int freesize = mtd->freesize / chip->planenum;
	int oobsize = mtd->oobsize / chip->planenum;
	u8 *databuf, *oobbuf;
	jz_bdma_desc_8word *desc;
	int err;
	u32 addrport, cmdport;
	struct buf_be_corrected buf_correct0;
	struct buf_be_corrected *buf_correct = &buf_correct0;
	int stat;

	addrport = (u32)(chip->IO_ADDR_R) | addr_offset;
	cmdport = (u32)(chip->IO_ADDR_R) | cmd_offset;

	databuf = buf;
	oobbuf = chip->oob_poi;

	dprintk("page:%d eccbytes:%d\n",page,eccbytes);

	/* descriptor for nand reading data block */
	desc = dma_desc_nand_read;
	desc->dsadr = CPHYSADDR((u32)chip->IO_ADDR_R); /* It will be changed when using multiply chip select */
	desc->dtadr = CPHYSADDR((u32)databuf);	/* DMA target address */
	dprintk("desc_nand_read:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x dcnt:%d\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr,desc->dcnt);

	if (freesize != 0) {
		/* descriptor for nand reading FREE(use for oob) block */
		desc++;
		desc->dsadr = CPHYSADDR((u32)chip->IO_ADDR_R);
		desc->dtadr = CPHYSADDR((u32)oobbuf + oobsize);
		dprintk("desc_free_read:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x dcnt:%d\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
				desc->ddadr,desc->dcnt);
	}

	/* descriptor for nand reading oob block */
	desc++;
	desc->dsadr = CPHYSADDR((u32)chip->IO_ADDR_R); /* It will be changed when using multiply chip select */
	desc->dtadr = CPHYSADDR((u32)oobbuf);	/* DMA target address */
	dprintk("desc_oob_read:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x dcnt:%d\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr,desc->dcnt);

	/* descriptors for data to be written to bch */
	desc = dma_desc_dec;
	for (i = 0; i < eccsteps; i++) {
		desc->dsadr = CPHYSADDR((u32)databuf) + i * eccsize;	/* DMA source address */
		dprintk("dma_desc_dec:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x dcnt:%d\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
				desc->ddadr,desc->dcnt);
		desc++;
	}

	/* descriptors for parities to be written to bch */
	desc = dma_desc_dec1;
	for (i = 0; i < eccsteps; i++) {
		desc->dsadr = CPHYSADDR((u32)oobbuf) + ecc_pos + (i + 1) * eccbytes;	/* DMA source address */
		dprintk("dma_desc_dec1:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x dcnt:%d\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
				desc->ddadr,desc->dcnt);
		desc++;
	}

	dma_cache_wback_inv((u32)dma_desc_nand_read, DMA_DESC_FLUSH_SIZE);

	memset(errs, 0, eccsteps * ERRS_SIZE * 4);
	dma_cache_inv((u32)databuf, mtd->writesize);	// databuf should be invalidated.
	dma_cache_inv((u32)oobbuf, oobsize + freesize);	// oobbuf should be invalidated too
	dma_cache_wback_inv((u32)errs, eccsteps * ERRS_SIZE * 4);

	REG_BDMAC_DCCSR(bch_dma_chan) = 0;
	REG_BDMAC_DCCSR(nand_dma_chan) = 0;

	/* Setup DMA descriptor address */
#if USE_PN
	REG_BDMAC_DDA(nand_dma_chan) = CPHYSADDR((u32)dma_desc_rPN);
#else
	REG_BDMAC_DDA(nand_dma_chan) = CPHYSADDR((u32)dma_desc_nand_read);
#endif
	REG_BDMAC_DDA(bch_dma_chan) = CPHYSADDR((u32)dma_desc_dec);

	/* Setup request source */
#if USE_PN
	REG_BDMAC_DRSR(nand_dma_chan) = BDMAC_DRSR_RS_AUTO;
#else
	REG_BDMAC_DRSR(nand_dma_chan) = BDMAC_DRSR_RS_NAND0;
#endif
	REG_BDMAC_DRSR(bch_dma_chan) = BDMAC_DRSR_RS_BCH_DEC;

	/* Enable DMA */
	REG_BDMAC_DMACR |= BDMAC_DMACR_DMAE;

	/* Enable BCH decoding */
	chip->ecc.hwctl(mtd, NAND_ECC_READ);

	dma_ack = 0;
	nand_status = NAND_READ;
	/* DMA doorbell set -- start nand DMA now ... */
	__bdmac_channel_set_doorbell(nand_dma_chan);

	/* Setup DMA channel control/status register */
	REG_BDMAC_DCCSR(nand_dma_chan) = BDMAC_DCCSR_DES8 | BDMAC_DCCSR_EN;

#define __nand_cmd(n)		(REG8(cmdport) = (n))
#define __nand_addr(n)		(REG8(addrport) = (n))

	__nand_cmd(NAND_CMD_READ0);

	__nand_addr(0);
	if (pagesize != 512)
		__nand_addr(0);

	__nand_addr(page & 0xff);
	__nand_addr((page >> 8) & 0xff);

	/* One more address cycle for the devices whose number of page address bits > 16  */
	if (((chip->chipsize >> chip->page_shift) >> 16) > 0)
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
		dump_jz_bdma_channel(bch_dma_chan);
		dump_jz_bdma_channel(nand_dma_chan);
		printk("REG_BCH_CR=%x REG_BCH_CNT=0x%x REG_BCH_INTS=%x\n", REG_BCH_CR, REG_BCH_CNT, REG_BCH_INTS);
	}
	dprintk("timeout remain = %d\n", err);
#else
	int timeout;
	timeout = 100000;
	while ((!__bdmac_channel_transmit_end_detected(bch_dma_chan)) && (timeout--));
	if (timeout <= 0) {
		printk("not use irq, NAND READ timeout!\n");
	}
#endif
	for (i = 0; i < eccsteps; i++) {

		buf_correct->data = databuf + eccsize * i;

		stat = chip->ecc.correct(mtd, (u8 *)buf_correct, (u8 *)&errs[i * ERRS_SIZE], NULL);
		if (stat < 0)
		{
			printk("DATA:ecc Uncorrectable:global_page = %d,chip->planenum = %d\n",global_page,chip->planenum);
			mtd->ecc_stats.failed++;
		}
		else
			mtd->ecc_stats.corrected += stat;
	}

	bch_decode_oob(mtd, chip);
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
	int pagesize = mtd->rl_writesize >> 1;
	int ppb = mtd->rl_erasesize / mtd->rl_writesize;

	page = (global_page / ppb) * ppb + global_page; /* = global_page%ppb + (global_page/ppb)*ppb*2 */

	/* read 1st page */
	nand_read_page_hwecc_bch0(mtd, chip, buf, page);

	/* read 2nd page */
	nand_read_page_hwecc_bch0(mtd, chip, buf + pagesize, page + ppb);
	return 0;
}

static int nand_dma_read_data(struct nand_chip *chip, u8 *data, u32 len)
{
#if USE_IRQ
	int err;
#else
	volatile int timeout;
#endif

	__bdmac_channel_enable_clk(nand_dma_chan);

	REG_BDMAC_DCCSR(nand_dma_chan) = 0;

	REG_BDMAC_DRSR(nand_dma_chan) = BDMAC_DRSR_RS_NAND0;

	REG_BDMAC_DSAR(nand_dma_chan) = CPHYSADDR((u32)(chip->IO_ADDR_R));
	REG_BDMAC_DTAR(nand_dma_chan) = CPHYSADDR((u32)(data));
	REG_BDMAC_DTCR(nand_dma_chan) = len / 4;
#if USE_IRQ
	REG_BDMAC_DCMD(nand_dma_chan) = BDMAC_DCMD_SAI | BDMAC_DCMD_DAI | BDMAC_DCMD_SWDH_32 |
				BDMAC_DCMD_DWDH_32 | BDMAC_DCMD_DS_32BIT | BDMAC_DCMD_TIE;

	dma_ack = 0;
	nand_status = NAND_READ;
#else
	REG_BDMAC_DCMD(nand_dma_chan) = BDMAC_DCMD_SAI | BDMAC_DCMD_DAI | BDMAC_DCMD_SWDH_32 |
				BDMAC_DCMD_DWDH_32 | BDMAC_DCMD_DS_32BIT;
#endif

	/* Enable DMA */
	REG_BDMAC_DMACR |= BDMAC_DMACR_DMAE;

	/* Setup DMA channel control/status register */
	REG_BDMAC_DCCSR(nand_dma_chan) |= BDMAC_DCCSR_NDES | BDMAC_DCCSR_EN;

#if USE_IRQ
	do {
		err = wait_event_interruptible_timeout(nand_read_wait_queue, dma_ack, 3 * HZ);
	} while (err == -ERESTARTSYS);
	nand_status = NAND_NONE;

	if (!err) {
		printk("*** NAND READ, Warning, wait event 3s timeout!\n");
		dump_jz_bdma_channel(nand_dma_chan);
		return -1;
	}
#else
	timeout = 100000;
	while ((!__bdmac_channel_transmit_end_detected(nand_dma_chan)) && (timeout--));
	if (timeout <= 0) {
		printk("not use irq, NAND READ timeout!\n");
		return -1;
	}
#endif
	return 0;

}

static int bch_dma_decode(u8 *databuf, u8 *parbuf, u32 cnt)
{
#if USE_IRQ
	int err;
#else
	volatile int timeout;
#endif

	__bdmac_channel_enable_clk(bch_dma_chan);

	REG_BDMAC_DCCSR(bch_dma_chan) = 0;

	REG_BDMAC_DRSR(bch_dma_chan) = BDMAC_DRSR_RS_BCH_DEC;

	REG_BDMAC_DSAR(bch_dma_chan) = CPHYSADDR((u32)databuf);
	REG_BDMAC_DTAR(bch_dma_chan) = CPHYSADDR((u32)parbuf);
	REG_BDMAC_DTCR(bch_dma_chan) = cnt;
#if USE_IRQ
	REG_BDMAC_DCMD(bch_dma_chan) = BDMAC_DCMD_BLAST | BDMAC_DCMD_SAI | BDMAC_DCMD_DAI | BDMAC_DCMD_SWDH_8 |
				BDMAC_DCMD_DWDH_32 | BDMAC_DCMD_DS_8BIT | BDMAC_DCMD_TIE;

	bch_status = BCH_DEC;
	bch_dec_ack = 0;
#else
	REG_BDMAC_DCMD(bch_dma_chan) = BDMAC_DCMD_BLAST | BDMAC_DCMD_SAI | BDMAC_DCMD_DAI | BDMAC_DCMD_SWDH_8 |
				BDMAC_DCMD_DWDH_32 | BDMAC_DCMD_DS_8BIT;
#endif
	/* Enable DMA */
	REG_BDMAC_DMACR |= BDMAC_DMACR_DMAE;
	/* Setup DMA channel control/status register */
	REG_BDMAC_DCCSR(bch_dma_chan) |= BDMAC_DCCSR_NDES | BDMAC_DCCSR_EN;

#if USE_IRQ
	do {
		err = wait_event_interruptible_timeout(nand_read_wait_queue, bch_dec_ack, 3 * HZ);
	}while(err == -ERESTARTSYS);
	bch_status = BCH_NONE;

	if (!err) {
		printk("*** bch decode, Warning, wait event 3s timeout!\n");
		dump_jz_bdma_channel(bch_dma_chan);
		printk("REG_BCH_CR=%x REG_BCH_CNT=0x%x REG_BCH_INTS=%x\n", REG_BCH_CR, REG_BCH_CNT, REG_BCH_INTS);
		return -1;
	}
#else
	timeout = 100000;
	while ((!__bdmac_channel_transmit_end_detected(bch_dma_chan)) && (timeout--));
	if (timeout <= 0) {
		printk("not use irq, BCH decode timeout!\n");
		return -1;
	}
#endif

	return 0;
}

static int nand_read_oob_hwecc_bch(struct mtd_info *mtd, struct nand_chip *chip, int page, int sndcmd)
{
	int ecc_pos = chip->eccpos;
	int eccbytes = chip->ecc.bytes;
	int stat;
	struct buf_be_corrected buf_correct;

	memset(chip->oob_poi, 0x00, mtd->oobsize);
	memset(oob_bch_errs, 0, ERRS_SIZE * 4);
	dma_cache_wback_inv((u32)chip->oob_poi, mtd->oobsize);
	dma_cache_wback_inv((u32)oob_bch_errs, ERRS_SIZE * 4);

	if (sndcmd) {
		chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
		sndcmd = 0;
	}
	/*read oob data from NAND*/
	stat = nand_dma_read_data(chip,(u8 *)chip->oob_poi, mtd->oobsize);
	if (stat < 0)
		goto out;

	/* Enable BCH decoding */
	REG_BCH_INTS = 0xffffffff;
	__ECC_DECODING();
	__ecc_cnt_dec(ecc_pos * 2 + PAR_SIZE);
	__ecc_dma_enable();

	stat = bch_dma_decode((u8 *)chip->oob_poi, (u8 *)oob_bch_errs, (ecc_pos + eccbytes));
	if (stat < 0)
		goto out;

	buf_correct.data = chip->oob_poi;
	buf_correct.eccsize = ecc_pos;
	stat = chip->ecc.correct(mtd, (u8 *)&buf_correct, (u8 *)&oob_bch_errs[0], NULL);
 out:
	if (stat < 0) {
		printk("READ_OOB:ecc Uncorrectable:global_page = %d,chip->planenum = %d\n",page,chip->planenum);
		dumpdata((u8 *)chip->oob_poi, 32);
		mtd->ecc_stats.failed++;
	}
	else
		mtd->ecc_stats.corrected += stat;

	return sndcmd;
}

#else	/* nand read in cpu mode */

static int nand_read_page_hwecc_bch(struct mtd_info *mtd, struct nand_chip *chip, uint8_t * buf)
{
	int i,j, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps / chip->planenum;
	int ecc_pos = chip->eccpos;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->ecc.layout->eccpos;
	int pagesize = mtd->rl_writesize / chip->planenum;
	int oobsize = mtd->oobsize / chip->planenum;
	int ecctotal = chip->ecc.total / chip->planenum;
	struct buf_be_corrected buf_correct0;
	int data_per_page = mtd->writesize;

	dprintk("\nchip->planenum:%d eccsteps:%d eccsize:%d eccbytes:%d ecc_pos:%d pagesize:%d oobsize:%d \
			ecctotal:%d data_per_page:%d\n",chip->planenum,eccsteps,eccsize,eccbytes,ecc_pos,pagesize,oobsize, \
			ecctotal,data_per_page);

	chip->read_buf(mtd, buf, pagesize);
	chip->read_buf(mtd, chip->oob_poi, oobsize);

	if (ecctotal <= oobsize - ecc_pos) {
		for (i = 0; i < ecctotal; i++) {
			ecc_code[i] = chip->oob_poi[eccpos[i]];
		}
	} else {
		for (i = 0; i < oobsize - ecc_pos; i++) {
			ecc_code[i] = chip->oob_poi[ecc_pos + i];
		}
		for (j = 0; j < ecctotal - oobsize + ecc_pos; j++) {
			ecc_code[i + j] = buf[data_per_page + j];
		}
	}

	for (i = 0; i < eccsteps; i++) {
		int stat;
		struct buf_be_corrected *buf_correct = &buf_correct0;

		buf_correct->data = buf + eccsize * i;

		chip->ecc.hwctl(mtd, NAND_ECC_READ);
		stat = chip->ecc.correct(mtd, (u8 *)buf_correct, &ecc_code[eccbytes * i], &ecc_calc[eccbytes * i]);
		if (stat < 0)
		{
			printk("ecc Uncorrectable:global_page = %d,chip->planenum = %d\n",global_page,chip->planenum);
			mtd->ecc_stats.failed++;
		}
		else
			mtd->ecc_stats.corrected += stat;
	}

	return 0;
}

static int nand_read_page_hwecc_bch_planes(struct mtd_info *mtd, struct nand_chip *chip, uint8_t * buf)
{
	int pagesize = mtd->rl_writesize >> 1;
	int ppb = mtd->rl_erasesize / mtd->rl_writesize;
	uint32_t page;

	page = (global_page / ppb) * ppb + global_page; /* = global_page%ppb + (global_page/ppb)*ppb*2 */

	/* Read first page */
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);
	nand_read_page_hwecc_bch(mtd, chip, buf);

	/* Read 2nd page */
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page + ppb);
	nand_read_page_hwecc_bch(mtd, chip, buf+pagesize);
	return 0;
}
#endif				/* CONFIG_MTD_NAND_DMA */

#endif				/* CONFIG_MTD_HW_BCH_ECC */

/* read oob using two-plane mode */
static int nand_read_oob_std_planes(struct mtd_info *mtd, struct nand_chip *chip,
		int global_page, int sndcmd)
{
	int page;
	int oobsize = mtd->oobsize >> 1;
	int ppb = mtd->rl_erasesize / mtd->rl_writesize;

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

/* write oob using two-plane mode */
static int nand_write_oob_std_planes(struct mtd_info *mtd, struct nand_chip *chip,
		int global_page)
{
	int status = 0, page;
	const uint8_t *buf = chip->oob_poi;
	int pagesize = mtd->rl_writesize >> 1;
	int oobsize = mtd->oobsize >> 1;
	int ppb = mtd->rl_erasesize / mtd->rl_writesize;

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

/* nand erase using two-plane mode */
static void single_erase_cmd_planes(struct mtd_info *mtd, int global_page)
{
	struct nand_chip *chip = mtd->priv;
	int page, ppb = mtd->rl_erasesize / mtd->rl_writesize;

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

	dma_chan = irq - IRQ_BDMA_0;

	dprintk("jz4760_dma_irq %d, channel %d\n", irq, dma_chan);

	if (__bdmac_channel_transmit_halt_detected(dma_chan)) {
		__bdmac_channel_clear_transmit_halt(dma_chan);
		wakeup = 1;
		printk("DMA HALT\n");
	}

	if (__bdmac_channel_address_error_detected(dma_chan)) {

		REG_BDMAC_DCCSR(dma_chan) &= ~BDMAC_DCCSR_EN;	/* disable DMA */
		__bdmac_channel_clear_address_error(dma_chan);

		REG_BDMAC_DSAR(dma_chan) = 0;	/* reset source address register */
		REG_BDMAC_DTAR(dma_chan) = 0;	/* reset destination address register */

		/* clear address error in BDMACR */
		REG_BDMAC_DMACR &= ~(1 << 2);
		wakeup = 1;
		printk("DMA address error!\n");
	}

	if (__bdmac_channel_transmit_end_detected(dma_chan)) {
		dprintk("DMA TT\n");
		REG_BDMAC_DCCSR(dma_chan) &= ~BDMAC_DCCSR_EN;	/* disable DMA */
		__bdmac_channel_clear_transmit_end(dma_chan);
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
		if ((dma_chan == nand_dma_chan) && (nand_status == NAND_READ)) {
			dprintk("nand read oob data irq, wake up----\n");
			dma_ack = 1;
			wake_up_interruptible(&nand_read_wait_queue);
		}
		if ((dma_chan == bch_dma_chan) && (bch_status == BCH_DEC)) {
			dprintk("nand read oob bch decode irq, wake up----\n");
			bch_dec_ack = 1;
			wake_up_interruptible(&nand_read_wait_queue);
		}
		wakeup = 0;
	}

	return IRQ_HANDLED;
}
#endif				/* USE_IRQ */

int jz4760b_nand_dma_init(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	int eccsize = chip->ecc.size;
	int eccsteps = chip->ecc.steps / chip->planenum;
	int eccbytes = chip->ecc.bytes;
	int ecc_pos = chip->eccpos;
	int oobsize = mtd->oobsize / chip->planenum;
	int i, err;
	jz_bdma_desc_8word *desc, *dma_desc_bch_ddr, *dma_desc_nand_ddr, *dma_desc_nand_cmd_pgprog;
	u32  *pval_nand_dcs, *pval_bch_ddr, *pval_bch_dcs, *dummy;
	u32 next;

	dprintk("eccsize:%d eccsteps:%d eccbytes:%d ecc_pos:%d pagesize:%d oobsize:%d\n",eccsize,eccsteps,eccbytes,ecc_pos,pagesize,oobsize);

#if USE_IRQ
	if ((err = request_irq(IRQ_BDMA_0 + nand_dma_chan, nand_dma_irq, IRQF_DISABLED, "nand_dma", NULL))) {
		printk("can't reqeust DMA nand channel.\n");
		return 0;
	}

	if ((err = request_irq(IRQ_BDMA_0 + bch_dma_chan, nand_dma_irq, IRQF_DISABLED, "bch_dma", NULL))) {
		printk("bch_dma irq request err\n");
		return 0;
	}

#endif

	__bdmac_channel_enable_clk(nand_dma_chan);
	__bdmac_channel_enable_clk(bch_dma_chan);

	/* space for the error reports of bch decoding((4 * ERRS_SIZE * eccsteps) bytes), and the space for
	 * the value of ddr and dcs of channel 0 and channel 1.  (4 * (2 + 2) bytes), and the space for dummy
	 * and pval_nand_cmd_pgprog (4 * 2 bytes). */
	errs = (u32 *)kmalloc(4 * (2 + 2 + 2 + ERRS_SIZE * eccsteps), GFP_KERNEL);
	if (!errs)
		return -ENOMEM;

	pval_nand_ddr = errs + ERRS_SIZE * eccsteps;
	pval_nand_dcs = pval_nand_ddr + 1;
	pval_bch_ddr = pval_nand_dcs + 1;
	pval_bch_dcs = pval_bch_ddr + 1;
	/* space for nand prog waiting target, the content is useless */
	dummy = pval_bch_dcs + 1;
	/* space to store CMD_PGPROG(0x10) or 0x11 */
	pval_nand_cmd_pgprog = (u32 *)(dummy + 1);

	/* desc can't across 4KB boundary, as desc base address is fixed */
	/* space of descriptors for nand reading data and oob blocks */
	dma_desc_nand_read = (jz_bdma_desc_8word *) __get_free_page(GFP_KERNEL);
	if (!dma_desc_nand_read)
		return -ENOMEM;

	memset(dma_desc_nand_read, 0 ,4096);

	/* space of descriptors for bch decoding */
	dma_desc_dec = dma_desc_nand_read + 3;		// for data 512 bytes.
	dma_desc_dec1 = dma_desc_dec + eccsteps;	// for ecc 39 bytes of 24bits.

	/* space of descriptors for notifying bch channel */
	dma_desc_bch_ddr = dma_desc_dec1 + eccsteps;

	/* space of descriptors for bch encoding */
	dma_desc_enc = dma_desc_bch_ddr + 2;
	dma_desc_enc1 = dma_desc_enc + 1;

	/* space of descriptors for nand programing data oob and free blocks */
	dma_desc_nand_prog = dma_desc_enc1 + eccsteps;

	/* space of descriptors for nand prog waiting, including pgprog and sync */
	dma_desc_nand_cmd_pgprog = dma_desc_nand_prog + 3;

	/* space of descriptors for notifying nand channel, including ddr and dcsr */
	dma_desc_nand_ddr = dma_desc_nand_cmd_pgprog + 2;

	/*************************************
	 * Setup of nand programing descriptors
	 *************************************/

	/* set descriptor for encoding oob blocks */
	desc = dma_desc_enc;
	next = CPHYSADDR((u32)dma_desc_enc) + sizeof(jz_bdma_desc_8word);
	desc->dcmd =
		BDMAC_DCMD_BLAST | BDMAC_DCMD_SAI | BDMAC_DCMD_DAI | BDMAC_DCMD_SWDH_8 |
		BDMAC_DCMD_DWDH_8 | BDMAC_DCMD_DS_8BIT | BDMAC_DCMD_LINK;
	desc->dcnt = ecc_pos;	/* size: 7 bytes -> 2 words */
	desc->dreqt = BDMAC_DRSR_RS_BCH_ENC;
	desc->ddadr = next;
	dprintk("cmd:%x sadr:%x tadr:%x dadr:%x dcnt\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr,desc->dcnt);
	desc++;

	/* set descriptor for encoding data blocks */
	desc = dma_desc_enc1;
	for (i = 0; i < eccsteps; i++) {
		next = CPHYSADDR((u32)dma_desc_enc1) + (i + 1) * (sizeof(jz_bdma_desc_8word));
		desc->dcmd =
			BDMAC_DCMD_BLAST | BDMAC_DCMD_SAI | BDMAC_DCMD_DAI | BDMAC_DCMD_SWDH_32 | BDMAC_DCMD_DWDH_8 |
			BDMAC_DCMD_DS_BCH | BDMAC_DCMD_LINK;
		desc->dcnt = eccsize / DIV_DS_BCH;	/* size: eccsize bytes */
		desc->dreqt = BDMAC_DRSR_RS_BCH_ENC;
		desc->ddadr = next;
		dprintk("cmd:%x sadr:%x tadr:%x dadr:%x dcnt:%d\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr, desc->dcnt);
		desc++;
	}

	next = CPHYSADDR((u32)dma_desc_nand_ddr);
	desc--;
	desc->ddadr = next;

	/* set the descriptor to set door bell of nand_dma_chan for programing nand */
	desc = dma_desc_nand_ddr;
	*pval_nand_ddr = 1 << nand_dma_chan;
	next = CPHYSADDR((u32)dma_desc_nand_ddr) + sizeof(jz_bdma_desc_8word);
	desc->dcmd = BDMAC_DCMD_SWDH_32 | BDMAC_DCMD_DWDH_32 | BDMAC_DCMD_DS_32BIT | BDMAC_DCMD_LINK;
	desc->dsadr = CPHYSADDR((u32)pval_nand_ddr);	/* DMA source address */
	desc->dtadr = CPHYSADDR(BDMAC_DMADBSR);	/* nand_dma_chan's descriptor addres register */
	desc->dcnt = 1;	/* size: 1 word */
	desc->dreqt = BDMAC_DRSR_RS_AUTO;
	desc->ddadr = next;
	dprintk("*pval_nand_ddr=0x%x\n", *pval_nand_ddr);

	/* set the descriptor to write dccsr of nand_dma_chan for programing nand, dccsr should be set at last */
	desc++;
	*pval_nand_dcs = BDMAC_DCCSR_DES8 | BDMAC_DCCSR_EN;	/* set value for writing ddr to enable channel nand_dma_chan */
	desc->dcmd = BDMAC_DCMD_SWDH_32 | BDMAC_DCMD_DWDH_32 | BDMAC_DCMD_DS_32BIT;
	desc->dsadr = CPHYSADDR((u32)pval_nand_dcs);	/* DMA source address */
	desc->dtadr = CPHYSADDR(BDMAC_DCCSR(nand_dma_chan));	/* address of dma door bell set register */
	desc->dcnt = 1;	/* size: 1 word */
	desc->dreqt = BDMAC_DRSR_RS_AUTO;
	dprintk("*pval_nand_dcs=0x%x\n", *pval_nand_dcs);

	/* set descriptor for nand programing data block */
	desc = dma_desc_nand_prog;
	next = CPHYSADDR((u32)dma_desc_nand_prog) + sizeof(jz_bdma_desc_8word);
	desc->dcmd =
		BDMAC_DCMD_SAI | BDMAC_DCMD_DAI | BDMAC_DCMD_SWDH_32 | BDMAC_DCMD_DWDH_32 |
		BDMAC_DCMD_DS_NAND | BDMAC_DCMD_LINK;
#if USE_DIRECT
	desc->dcmd |= BDMAC_DCMD_NWR;
#endif
	desc->dtadr = CPHYSADDR((u32)(chip->IO_ADDR_W)); /* DMA target address */
	desc->dcnt = mtd->writesize / DIV_DS_NAND;	/* size: eccsize bytes */
	desc->dreqt = BDMAC_DRSR_RS_AUTO;
	desc->ddadr = next;
	dprintk("cmd:%x sadr:%x tadr:%x dadr:%x dcnt:%d\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr, desc->dcnt);

	if (mtd->freesize != 0) {
		/* set descriptor for nand programing free block */
		desc++;
		next = CPHYSADDR((u32)dma_desc_nand_prog) + 2 * sizeof(jz_bdma_desc_8word);
		desc->dcmd = BDMAC_DCMD_SAI | BDMAC_DCMD_DAI | BDMAC_DCMD_SWDH_32 | BDMAC_DCMD_DWDH_32 |
			BDMAC_DCMD_DS_NAND | BDMAC_DCMD_LINK;
#if USE_DIRECT
		desc->dcmd |= BDMAC_DCMD_NWR;
#endif
		desc->dtadr = CPHYSADDR((u32)(chip->IO_ADDR_W));
		desc->dcnt = mtd->freesize / DIV_DS_NAND;
		desc->dreqt = BDMAC_DRSR_RS_AUTO;
		desc->ddadr = next;
		dprintk("cmd:%x sadr:%x tadr:%x dadr:%x dcnt:%d\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr, desc->dcnt);
	}

	/* set descriptor for nand programing oob block */
	desc++;
	next = CPHYSADDR((u32)dma_desc_nand_cmd_pgprog);
	desc->dcmd =
		BDMAC_DCMD_SAI | BDMAC_DCMD_DAI | BDMAC_DCMD_SWDH_32 | BDMAC_DCMD_DWDH_32 |
		BDMAC_DCMD_DS_32BIT | BDMAC_DCMD_LINK;
#if USE_DIRECT
	desc->dcmd |= BDMAC_DCMD_NWR;
#endif
	desc->dtadr = CPHYSADDR((u32)(chip->IO_ADDR_W));	/* DMA target address: dataport */
	desc->dcnt = oobsize / 4;            	/* size: eccsize bytes */
	desc->dreqt = BDMAC_DRSR_RS_AUTO;
	desc->ddadr = next;
	dprintk("cmd:%x sadr:%x tadr:%x dadr:%x dcnt:%d\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr, desc->dcnt);

	/* set descriptor for __nand_cmd(CMD_PGPROG) */
	desc = dma_desc_nand_cmd_pgprog;
	*pval_nand_cmd_pgprog = NAND_CMD_PAGEPROG | 0x40000000;
	next = CPHYSADDR((u32)dma_desc_nand_cmd_pgprog) + sizeof(jz_bdma_desc_8word);
	desc->dcmd =
		BDMAC_DCMD_NAC | BDMAC_DCMD_SWDH_32 | BDMAC_DCMD_DWDH_XX | BDMAC_DCMD_DS_32BIT | BDMAC_DCMD_LINK;
	desc->dsadr = CPHYSADDR((u32)pval_nand_cmd_pgprog);	        /* DMA source address */
	desc->dtadr = CPHYSADDR((u32)chip->IO_ADDR_R);	/* DMA target address: cmdport */
	desc->dcnt = 1;	                        /* size: 1 byte */
	desc->dnt = 0;
	desc->dreqt = BDMAC_DRSR_RS_AUTO;
	desc->ddadr = next;
	dprintk("cmd:%x sadr:%x tadr:%x dadr:%x dcnt:%d\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr, desc->dcnt);

	/* set descriptor for __nand_sync() */
	desc++;
#if USE_IRQ
	desc->dcmd =
		BDMAC_DCMD_SWDH_32 | BDMAC_DCMD_DWDH_32 | BDMAC_DCMD_DS_32BIT | BDMAC_DCMD_TIE;
#else
	desc->dcmd =
		BDMAC_DCMD_SWDH_32 | BDMAC_DCMD_DWDH_32 | BDMAC_DCMD_DS_32BIT;
#endif
	desc->dsadr = CPHYSADDR((u32)pval_nand_ddr);	/* DMA source address */
	desc->dtadr = CPHYSADDR((u32)dummy);        	/* DMA target address, the content is useless */
	desc->dcnt = 1;             	/* size: 1 word */
	desc->dnt = 1;
	desc->dreqt = BDMAC_DRSR_RS_NAND0;
	dprintk("cmd:%x sadr:%x tadr:%x dadr:%x dcnt:%d\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr, desc->dcnt);

	/* 1 + eccsteps + 3 + 2 + 2:
	   dma_desc_enc + dma_desc_enc1 + dma_desc_nand_prog(oob/free) + dma_desc_nand_ddr(csr)
	   + dma_desc_nand_cmd_pgprog(sync) */

	dma_cache_wback_inv((u32)dma_desc_enc, DMA_DESC_FLUSH_SIZE);

	/* 4*6: pval_nand_ddr, pval_nand_dcs, pval_bch_ddr, pval_bch_dcs, dummy, pval_nand_cmd_pgprog */
	dma_cache_wback_inv((u32)pval_nand_ddr, 4 * 8); /* 8 words, a cache line */

	/*************************************
	 * Setup of nand reading descriptors
	 *************************************/


	/* set descriptor for nand reading data block */
	desc = dma_desc_nand_read;
	next = CPHYSADDR((u32)dma_desc_nand_read) + sizeof(jz_bdma_desc_8word);
	desc->dcmd =
		BDMAC_DCMD_SAI | BDMAC_DCMD_DAI | BDMAC_DCMD_SWDH_32 | BDMAC_DCMD_DWDH_32 |
		BDMAC_DCMD_DS_NAND | BDMAC_DCMD_LINK;
#if USE_DIRECT
	desc->dcmd |= BDMAC_DCMD_NRD;
#endif
	desc->dsadr = CPHYSADDR((u32)(chip->IO_ADDR_R));	/* DMA source address */
	desc->dcnt = mtd->writesize / DIV_DS_NAND;	/* size: eccsize bytes */
	desc->dreqt = BDMAC_DRSR_RS_NAND0;
	desc->ddadr = next;
	dprintk("cmd:%x sadr:%x tadr:%x dadr:%x\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr);

	if (mtd->freesize != 0) {
		/* set descriptor for nand reading FREE(use as oob) block */
		desc++;
		next = CPHYSADDR((u32)dma_desc_nand_read) + sizeof(jz_bdma_desc_8word) * 2;
		desc->dcmd =
			BDMAC_DCMD_SAI | BDMAC_DCMD_DAI | BDMAC_DCMD_SWDH_32 | BDMAC_DCMD_DWDH_32 |
			BDMAC_DCMD_DS_NAND | BDMAC_DCMD_LINK;
#if USE_DIRECT
		desc->dcmd |= BDMAC_DCMD_NRD;
#endif
		desc->dsadr = CPHYSADDR((u32)(chip->IO_ADDR_R));	/* DMA source address */
		desc->dcnt = mtd->freesize / DIV_DS_NAND;	/* size: eccsize bytes */
		desc->dreqt = BDMAC_DRSR_RS_AUTO;
		desc->ddadr = next;
		dprintk("cmd:%x sadr:%x tadr:%x dadr:%x\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr);
	}

	/* set descriptor for nand reading oob block */
	desc++;
	next = CPHYSADDR((u32)dma_desc_bch_ddr);
	desc->dcmd =
		BDMAC_DCMD_SAI | BDMAC_DCMD_DAI | BDMAC_DCMD_SWDH_32 | BDMAC_DCMD_DWDH_32 |
		BDMAC_DCMD_DS_32BIT | BDMAC_DCMD_LINK;
#if USE_DIRECT
	desc->dcmd |= BDMAC_DCMD_NRD;
#endif
	desc->dsadr = CPHYSADDR((u32)(chip->IO_ADDR_R));	/* DMA source address */
	//	desc->dcnt = oobsize / DIV_DS_NAND;	/* size: eccsize bytes */
	desc->dcnt = oobsize / 4;	/* size: eccsize bytes */
	desc->dreqt = BDMAC_DRSR_RS_AUTO;
	desc->ddadr = next;
	dprintk("cmd:%x sadr:%x tadr:%x dadr:%x\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr);

	/* set the descriptor to set door bell for bch */
	desc = dma_desc_bch_ddr;
	*pval_bch_ddr = BDMAC_DMADBSR_DBS0;	// set value for writing ddr to enable channel 0
	next = CPHYSADDR((u32)dma_desc_bch_ddr) + sizeof(jz_bdma_desc_8word);
	desc->dcmd = BDMAC_DCMD_SWDH_32 | BDMAC_DCMD_DWDH_32 | BDMAC_DCMD_DS_32BIT | BDMAC_DCMD_LINK;
	desc->dsadr = CPHYSADDR((u32)pval_bch_ddr);	/* DMA source address */
	desc->dtadr = CPHYSADDR(BDMAC_DMADBSR);	/* channel 0's descriptor addres register */
	desc->dcnt = 1;	/* size: 1 word */
	desc->dreqt = BDMAC_DRSR_RS_AUTO;
	desc->ddadr = next;

	/* set descriptor for writing dcsr */
	desc++;
	*pval_bch_dcs = BDMAC_DCCSR_DES8 | BDMAC_DCCSR_EN;	// set value for writing ddr to enable channel 0
	desc->dcmd = BDMAC_DCMD_SWDH_32 | BDMAC_DCMD_DWDH_32 | BDMAC_DCMD_DS_32BIT;
	desc->dsadr = CPHYSADDR((u32)pval_bch_dcs);	/* DMA source address */
	desc->dtadr = CPHYSADDR(BDMAC_DCCSR(bch_dma_chan));
	desc->dcnt = 1;	/* size: 1 word */
	desc->dreqt = BDMAC_DRSR_RS_AUTO;

	/* Ch0 ... */
	/* descriptors for data to be written to bch */
	desc = dma_desc_dec;
	for (i = 0; i < eccsteps; i++) {
		next = CPHYSADDR((u32)dma_desc_dec1 + i * (sizeof(jz_bdma_desc_8word)));

		desc->dcmd =
			BDMAC_DCMD_SAI | BDMAC_DCMD_DAI | BDMAC_DCMD_SWDH_32 | BDMAC_DCMD_DWDH_32 |
			BDMAC_DCMD_DS_BCH | BDMAC_DCMD_LINK;
		desc->dtadr = CPHYSADDR((u32)errs) + i * 4 * ERRS_SIZE;	/* DMA target address */
		desc->dcnt = eccsize / DIV_DS_BCH;	/* size: eccsize bytes */
		desc->dreqt = BDMAC_DRSR_RS_BCH_DEC;
		desc->ddadr = next;
		dprintk("desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
				desc->ddadr);
		desc++;
	}

	/* descriptors for parities to be written to bch */
	desc = dma_desc_dec1;
	for (i = 0; i < eccsteps; i++) {
		next = CPHYSADDR((u32)dma_desc_dec) + (i + 1) * (sizeof(jz_bdma_desc_8word));

		desc->dcmd =
			BDMAC_DCMD_BLAST | BDMAC_DCMD_SAI | BDMAC_DCMD_DAI | BDMAC_DCMD_SWDH_8 |
			BDMAC_DCMD_DWDH_8 | BDMAC_DCMD_DS_8BIT | BDMAC_DCMD_LINK;
		desc->dtadr = CPHYSADDR((u32)errs) + i * 4 * ERRS_SIZE;	/* DMA target address */
		desc->dcnt = eccbytes;	/* size: eccbytes bytes */
		desc->dreqt = BDMAC_DRSR_RS_BCH_DEC;
		desc->ddadr = next;
		dprintk("desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
				desc->ddadr);
		desc++;
	}
	desc--;
	desc->dcmd &= ~BDMAC_DCMD_LINK;
#if USE_IRQ
	desc->dcmd |= BDMAC_DCMD_TIE;
#endif

	dma_cache_wback_inv((u32)dma_desc_nand_read, DMA_DESC_FLUSH_SIZE);

	dma_cache_wback_inv((u32)pval_bch_ddr, 4 * 2); /* two words */

	return 0;
}

#endif				/* CONFIG_MTD_NAND_DMA */

/*
 * Main initialization routine
 */
int __init jznand_init(void)
{
	struct nand_chip *this;
	int nr_partitions, ret, i;

	printk(KERN_INFO "JZ NAND init");
#if defined(CONFIG_MTD_NAND_DMA)
#if defined(CONFIG_MTD_NAND_DMABUF)
	printk(KERN_INFO " DMA mode, using DMA buffer in NAND driver.\n");
#else
	printk(KERN_INFO " DMA mode, using DMA buffer in upper layer.\n");
#endif
#else
	printk(KERN_INFO " CPU mode.\n");
#endif

	cpm_start_clock(CGM_BDMA);

	/* Allocate memory for MTD device structure and private data */
	jz_mtd = kmalloc(sizeof(struct mtd_info) + sizeof(struct nand_chip), GFP_KERNEL);
	if (!jz_mtd) {
		printk("Unable to allocate JzSOC NAND MTD device structure.\n");
		return -ENOMEM;
	}

	/* Allocate memory for NAND when using only one plane */
	jz_mtd1 = kmalloc(sizeof(struct mtd_info) + sizeof (struct nand_chip), GFP_KERNEL);
	if (!jz_mtd1) {
		printk ("Unable to allocate JzSOC NAND MTD device structure 1.\n");
		kfree(jz_mtd);
		return -ENOMEM;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *)(&jz_mtd[1]);

	/* Initialize structures */
	memset((char *)jz_mtd, 0, sizeof(struct mtd_info));
	memset((char *)this, 0, sizeof(struct nand_chip));

#ifdef CONFIG_MTD_NAND_BUS_WIDTH_16
	this->options |= NAND_BUSWIDTH_16;
#endif

	/* Link the private data with the MTD structure */
	jz_mtd->priv = this;


	addr_offset = NAND_ADDR_OFFSET;
	cmd_offset = NAND_CMD_OFFSET;

	/* Set & initialize NAND Flash controller */
	jz_device_setup();

	/* Set address of NAND IO lines to static bank1 by default */
	this->IO_ADDR_R = (void __iomem *)NAND_DATA_PORT1;
	this->IO_ADDR_W = (void __iomem *)NAND_DATA_PORT1;
	this->cmd_ctrl = jz_hwcontrol;
	this->dev_ready = jz_device_ready;

#ifdef CONFIG_MTD_HW_BCH_ECC
	this->ecc.calculate = jzsoc_nand_calculate_bch_ecc;
	this->ecc.correct = jzsoc_nand_bch_correct_data;
	this->ecc.hwctl = jzsoc_nand_enable_bch_hwecc;
	this->ecc.mode = NAND_ECC_HW;
	this->ecc.size = 512;
	this->ecc.read_page = nand_read_page_hwecc_bch;
	this->ecc.write_page = nand_write_page_hwecc_bch;

	this->ecc.read_oob = nand_read_oob_hwecc_bch;
	this->ecc.write_oob = nand_write_oob_hwecc_bch;

#if defined(CONFIG_MTD_HW_BCH_24BIT)
	this->ecc.bytes = 39;
#elif defined(CONFIG_MTD_HW_BCH_20BIT)
	this->ecc.bytes = 33;
#elif defined(CONFIG_MTD_HW_BCH_16BIT)
	this->ecc.bytes = 26;
#elif defined(CONFIG_MTD_HW_BCH_12BIT)
	this->ecc.bytes = 20;
#elif defined(CONFIG_MTD_HW_BCH_8BIT)
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
	ret = nand_scan_ident(jz_mtd, NAND_MAX_CHIPS);

	if (!ret) {
		if (this->planenum == 2) {
			/* reset nand functions */
			this->erase_cmd = single_erase_cmd_planes;
			this->ecc.read_page = nand_read_page_hwecc_bch_planes;
			this->ecc.write_page = nand_write_page_hwecc_bch_planes;
			this->ecc.read_oob = nand_read_oob_std_planes;
			this->ecc.write_oob = nand_write_oob_std_planes;

			printk(KERN_INFO "Nand using two-plane mode, "
					"and resized to rl_writesize:%d oobsize:%d blocksize:0x%x \n",
					jz_mtd->rl_writesize, jz_mtd->oobsize, jz_mtd->rl_erasesize);
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

	((struct nand_chip *) (&jz_mtd1[1]))->ecc.read_page = nand_read_page_hwecc_bch;
	((struct nand_chip *) (&jz_mtd1[1]))->ecc.write_page = nand_write_page_hwecc_bch;

	/* Register the partitions */
	printk (KERN_NOTICE "Creating %d MTD partitions on \"%s\":\n", nr_partitions, jz_mtd->name);

	calc_partition_size(jz_mtd);

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

#if defined(CONFIG_MTD_NAND_DMA)
static int jz4760_nand_dma_exit(struct mtd_info *mtd)
{
	int pagesize = mtd->rl_writesize / chip->planenum;

#if USE_IRQ
	free_irq(IRQ_BDMA_0 + nand_dma_chan, NULL);
	free_irq(IRQ_BDMA_0 + bch_dma_chan, NULL);
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

#if USE_PN
	free_page((u32)dma_desc_pPN);
	free_page((u32)dma_desc_rPN);
	kfree(pn_buf);
#endif

	return 0;
}
#endif

static void __exit jznand_cleanup(void)
{
#if defined(CONFIG_MTD_NAND_DMA)
	jz4760_nand_dma_exit(jz_mtd);
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
