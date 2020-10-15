/*
 * Direct MTD block device access
 *
 * (C) Nancy <yrtan@ingenic.cn>
 *     Regen <lhhuang@ingenic.cn>
 *     Lucifer <yliu@ingenic.cn>
 *     Betty <xyzhang@ingenic.cn>
 */

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/hdreg.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/blktrans.h>
#include <linux/mutex.h>
#include <asm/jzsoc.h>

//#define MTDBLOCK_DEBUG
#ifdef MTDBLOCK_DEBUG
#define dprintk(a...)   printk(a)
#else
#define dprintk(a...)   while(0){}
#endif

#define MTDBLOCK_BIT_VALID_ENTRY          0x80000000
#define MTDBLOCK_BIT_BLOCK_ADDR           0x7FFFFFFF

#define MTDBLOCK_BIT_FREE_BLOCK           0x01
#define MTDBLOCK_BIT_BAD_BLOCK            0x02
#define MTDBLOCK_BIT_EMPTY_BLOCK          0x04
#define MTDBLOCK_NAND_BLK_STATUS_NORMAL   0xFF

#define WRITE_VERIFY_ENABLE   0
#define ECC_FAILD_RETRY       5
#define SECTOR_SIZE         512
#define SECTOR_SHIFT          9

#define Left(i)        ((i) << 1)
#define Right(i)       (((i) << 1) + 1)

struct mtdblk_block_info{
	unsigned int lifetime;
	unsigned char tag;
};

struct mtdblk_fake_fsbuf{
	unsigned int block_addr_field1;
	unsigned int block_addr_field2;
	unsigned int lifetime;
};

struct mtdblk_zone_t {
	int total_phys_block;
	int total_virt_block;
	int bad_block_num;

	int free_phys_block;
	int free_virt_block;
	unsigned int used_block;
	unsigned int bad_block;
	unsigned int *block_lookup; //index: virt_block value:block_info's index->phy_block
	struct mtdblk_block_info *block_info;
};

static struct mtdblk_dev {
	struct mtd_info *mtd;
	int virt_block;
	int new_phys_block;
	int old_phys_block;

	int count;
	struct mutex cache_mutex;
	/* block_cache */
	unsigned char *block_cache_data;
	unsigned char *page_state;
	unsigned char *page_offset_state;
	enum { STATE_UNUSED, STATE_USED } block_cache_state;
	/* page cache */
	unsigned char *page_cache_data;
	unsigned long page_num;
        /* temporary page buffer */
	unsigned char *g_page_buf;

	/* address mapping & wear-levelling used */
	struct mtdblk_zone_t *zone;
} *mtdblks[MAX_MTD_DEVICES];

extern unsigned char **jz_mtdblock_cache; /* defined in nand_base.c for allocating a block cache early */

static struct mtdblk_fake_fsbuf fsoobbuf;
static struct mtdblk_dev *g_udc_mtdblk;
static struct mtd_info *g_udc_mtd;

static int mtd_blk[8] = {0};  /* Get from cmdline. Specify which blocks work over mtdblock-jz. */
static int mtd_blkn[8] = {0}; /* Get from cmdline. Specify which blocks don't work over mtdblock-jz. */
static int mtdblklog = 0;     /* Get from cmdline. Specify whether some log is printed. */

/* address mapping & bad-block managment */
static int mtdblock_find_free_block (struct mtdblk_dev *mtdblk, int *free_phys_block);
static int mtdblock_block_info_map_bad_block (struct mtdblk_dev *mtdblk,int phys_block);
static int mtdblock_mark_bad_block_to_nand(struct mtdblk_dev *mtdblk, int phys_block);
static int mtdblock_block_lookup_map_entry (struct mtdblk_dev *mtdblk, int virt_block, int phys_block);
static int mtdblock_block_lookup_unmap_entry (struct mtdblk_dev *mtdblk, int virt_block);
static int mtdblock_address_translate (struct mtdblk_dev *mtdblk, int virt_block, int *phys_block);

/* block-cache operation */
static void inline mtdblock_init_block_cache (struct mtdblk_dev *mtdblk);
static void mtdblock_setup_block_cache (struct mtdblk_dev *mtdblk, int virt_block, int new_phys_block, int old_phys_block);
static int mtdblock_fill_block_cache(struct mtdblk_dev *mtdblk);
static int mtdblock_erase_block(struct mtdblk_dev *mtdblk, int phys_block);
static int mtdblock_program_block(struct mtdblk_dev *mtdblk, int phys_block);
static int mtdblock_move_to_another_block(struct mtdblk_dev *mtdblk, int old_phys_block);
static int erase_block (struct mtd_info *mtd, int phys_block);
static void erase_callback(struct erase_info *done);

/* init */
static int mtdblock_zone_init(struct mtdblk_dev *mtdblk, int dev);
static int mtdblock_getgeo(struct mtd_blktrans_dev *dev, struct hd_geometry *geo);

extern int *get_jz_badblock_table(void);
extern unsigned short get_mtdblock_oob_copies(void);
extern unsigned short get_mtdblock_write_verify_enable(void);

static int mtdblock_move_to_another_block(struct mtdblk_dev *mtdblk, int old_phys_block)
{
	struct mtd_info *mtd = mtdblk->mtd;
	struct nand_chip *this = (struct nand_chip *)mtd->priv;
	struct mtdblk_block_info *block_info = mtdblk->zone->block_info;
	struct mtd_oob_ops oobops;
	unsigned char *tmp_block_cache;
	unsigned long long pos;
	unsigned short ppb = this->ppb;
	int new_phys_block, phys_block, i , ret, readfail=0;

	tmp_block_cache = mtdblk->block_cache_data;

	if(!tmp_block_cache)
		return -ENOMEM;

	/* read a block from old block */
	pos = (unsigned long long)old_phys_block * mtd->erasesize;
	memset(&oobops, 0, sizeof(oobops));
	oobops.mode = MTD_OOB_AUTO;

	oobops.len = mtd->writesize;

	oobops.ooboffs = 2;
	oobops.ooblen = sizeof(fsoobbuf);
	oobops.oobbuf = (unsigned char *)&fsoobbuf;

	for(i=0; i<ppb; i++) {
		do {
			oobops.datbuf = &tmp_block_cache[mtd->writesize];
			ret = mtd->read_oob(mtd, pos, &oobops);
			if(ret){
				readfail ++;
				if (readfail < ECC_FAILD_RETRY)
					continue;
				else {
					printk("%s WARNING: uncorretable ecc or too many bit error cause bad block\n",__FILE__);
					readfail = 0;
					break;
				}
			} else {
				if (readfail != 0)
					printk("%s: uncorretable ecc ---> correctable ecc due to %d times read retry\n",__FILE__, readfail);
				readfail = 0;
				break;
			}
		} while(1);
		pos += mtd->writesize;
	}

	new_phys_block = mtdblock_mark_bad_block_to_nand(mtdblk, old_phys_block);
	/* write old block to a new free block */
	phys_block = new_phys_block;
 write_retry:
	do {
		ret = erase_block(mtd, phys_block);
		block_info[phys_block].lifetime++;

		/* if erase process error, tagged to be bad block,
		 * and find a new free phys_block to program
		 */
		if( ret < 0 ) {
			printk("%s: erase failed , mark to bad block: 0x%x \n",__FILE__, phys_block);
			phys_block = mtdblock_mark_bad_block_to_nand(mtdblk, phys_block);
		}

	}while(ret < 0);

	//go for write
	pos = (unsigned long long)phys_block * mtd->erasesize;
	for(i=0; i<ppb; i++){

		oobops.datbuf = &tmp_block_cache[mtd->writesize];
		ret =mtd->write_oob(mtd, pos, &oobops);
		if (ret ){
			printk("%s: write failed , mark to bad block: 0x%x \n",__FILE__, phys_block);
			phys_block = mtdblock_mark_bad_block_to_nand(mtdblk, phys_block);
			goto write_retry;
		}
		pos += mtd->writesize;
	}
	new_phys_block = phys_block;
	return new_phys_block;
}

static int mtdblock_find_free_block (struct mtdblk_dev *mtdblk, int *free_phys_block)
{
	struct mtdblk_zone_t *zone = mtdblk->zone;
	struct mtdblk_block_info *block_info_table;
	int i,phys_block;
	struct mtd_info *mtd = mtdblk->mtd;
	struct mtd_oob_ops oobops;
	struct nand_chip *this = mtdblk->mtd->priv;
	unsigned short ppb = this->ppb;
	unsigned long long pos;

	phys_block = -1;
	block_info_table = zone->block_info;
     	for (i = 0; i < zone->total_phys_block; i++)
		if ((block_info_table[i].tag & MTDBLOCK_BIT_FREE_BLOCK) &&
		    !(block_info_table[i].tag & MTDBLOCK_BIT_BAD_BLOCK)) {
			phys_block = i;
			break;
		}
	if (phys_block < 0)
		return -ENOMEM;
     	for (i = phys_block + 1; i < zone->total_phys_block; i++) {
		if ((block_info_table[i].tag & MTDBLOCK_BIT_FREE_BLOCK) &&
		    !(block_info_table[i].tag & MTDBLOCK_BIT_BAD_BLOCK) &&
		    (block_info_table[i].lifetime < block_info_table[phys_block].lifetime))
			phys_block = i;
	}

	*free_phys_block = phys_block;

	memset(&oobops, 0, sizeof(oobops));
	oobops.mode = MTD_OOB_AUTO;
	oobops.ooblen = sizeof(fsoobbuf);
	oobops.ooboffs = 2;
	oobops.oobbuf = (unsigned char *)&fsoobbuf;
	pos = ((unsigned long long)phys_block * mtd->erasesize) + ((ppb - 1) * mtd->writesize);
	mtd->read_oob(mtd, pos, &oobops);
	if (fsoobbuf.block_addr_field1 == 0xFFFFFFFF) {
		block_info_table[phys_block].tag |= MTDBLOCK_BIT_EMPTY_BLOCK;
	} else {
		block_info_table[phys_block].tag &= ~MTDBLOCK_BIT_EMPTY_BLOCK;
	}

	return 0;
}

static int mtdblock_block_info_map_bad_block (struct mtdblk_dev *mtdblk, int phys_block)
{
	struct mtdblk_zone_t *zone_ptr = mtdblk->zone;

	if(NULL == zone_ptr)
		printk("zone_ptr is null\n");
	if(NULL == zone_ptr->block_info)
		printk("zone_ptr->block_info is null\n");

	zone_ptr->block_info[phys_block].tag |= MTDBLOCK_BIT_BAD_BLOCK;
	zone_ptr->bad_block++;
	zone_ptr->free_phys_block--;
	zone_ptr->free_virt_block--;

	if (zone_ptr->bad_block > zone_ptr->bad_block_num ) {
		printk("%s Warning: too many bad blocks: %d, nand flash is un-useable\n", __FILE__, zone_ptr->bad_block);
		return -EIO;
	} else if (mtdblklog)
		printk("%s: total bad block num=%d, current bad phys_block=%d\n", __FILE__, zone_ptr->bad_block, phys_block);
	return 0;
}

static int mtdblock_block_lookup_map_entry (struct mtdblk_dev *mtdblk, int virt_block, int phys_block)
{
	struct mtdblk_zone_t *zone_ptr = mtdblk->zone;

	if(NULL == zone_ptr)
		printk("%s: zone_ptr is null\n",__FUNCTION__);

	if(virt_block >= zone_ptr->total_virt_block || virt_block < 0){
		dprintk("virt_block address Abnormal\n");
		return -EINVAL;
	}
	if (!(zone_ptr->block_lookup[virt_block] & MTDBLOCK_BIT_VALID_ENTRY)) {
		dprintk("map %d -> %d  free %d\n", virt_block, phys_block,zone_ptr->free_phys_block);
		zone_ptr->block_lookup[virt_block] = phys_block;
		zone_ptr->block_lookup[virt_block] |= MTDBLOCK_BIT_VALID_ENTRY;
		zone_ptr->block_info[phys_block].tag &= ~MTDBLOCK_BIT_FREE_BLOCK;
		zone_ptr->free_phys_block--;
		zone_ptr->free_virt_block--;
		zone_ptr->used_block++;
		return 0;
	} else {
		dprintk("Error: map block address 0x%x -> (new) 0x%x, (old) 0x%x\n",
			 virt_block, phys_block, zone_ptr->block_lookup[virt_block] & MTDBLOCK_BIT_BLOCK_ADDR);
		return -EINVAL;
	}
}

static int mtdblock_block_lookup_unmap_entry (struct mtdblk_dev *mtdblk, int virt_block)
{
	struct mtdblk_zone_t *zone_ptr = mtdblk->zone;
	int phys_block;

	if (zone_ptr->block_lookup[virt_block] & MTDBLOCK_BIT_VALID_ENTRY) {
		zone_ptr->block_lookup[virt_block] &= ~MTDBLOCK_BIT_VALID_ENTRY;
		phys_block = zone_ptr->block_lookup[virt_block] & MTDBLOCK_BIT_BLOCK_ADDR;
		zone_ptr->block_info[phys_block].tag |= MTDBLOCK_BIT_FREE_BLOCK;
		zone_ptr->free_phys_block++;
		zone_ptr->free_virt_block++;
		zone_ptr->used_block--;
		return 0;
	} else {
		printk("%s Error: unmap block address 0x%x -> NULL\n", __FILE__, virt_block);
		return -EINVAL;
	}
}

static int mtdblock_address_translate (struct mtdblk_dev *mtdblk, int virt_block, int *phys_block)
{
	struct mtdblk_zone_t *zone_ptr = mtdblk->zone;
	unsigned int entry;

	entry = zone_ptr->block_lookup[virt_block];
	if (!(entry & MTDBLOCK_BIT_VALID_ENTRY))
		return -EINVAL;

	*phys_block = entry & MTDBLOCK_BIT_BLOCK_ADDR;
	return 0;
}


static void mtdblock_init_block_cache(struct mtdblk_dev *mtdblk)
{
	struct nand_chip *this = mtdblk->mtd->priv;
	unsigned short ppb = this->ppb;
	unsigned short spp = mtdblk->mtd->writesize>> 9 ;  //spp : sectors per page

	mtdblk->block_cache_state = STATE_UNUSED;
	memset(mtdblk->page_state, 0, ppb);
	memset(mtdblk->page_offset_state, 0, ppb*spp);

	//must clear write buffer before using it.
	mtdblk->page_num = -1;
	memset(mtdblk->block_cache_data, 0xFF, mtdblk->mtd->erasesize);
}

static void mtdblock_setup_block_cache ( struct mtdblk_dev *mtdblk, int virt_block, int new_phys_block, int old_phys_block)
{
	struct nand_chip *this = mtdblk->mtd->priv;
	unsigned short ppb = this->ppb;
	unsigned short spp = mtdblk->mtd->writesize>> 9 ;  //spp : sectors per page

	mtdblk->old_phys_block = old_phys_block;
	mtdblk->new_phys_block = new_phys_block;
	mtdblk->virt_block = virt_block;
	mtdblk->page_num = -1;

	mtdblk->block_cache_state = STATE_USED;
	memset(mtdblk->page_state, 0, ppb);
	memset(mtdblk->page_offset_state, 0, ppb*spp);

	//must clear write buffer before using it.
	memset(mtdblk->block_cache_data, 0xFF, mtdblk->mtd->erasesize);
}

static int mtdblock_fill_block_cache(struct mtdblk_dev *mtdblk)
{
	struct mtd_info *mtd = mtdblk->mtd;
	struct nand_chip *this = mtd->priv;
	struct mtd_oob_ops oobops;
	int phys_block, ret;
	unsigned short page, sector;
	unsigned long phys_page;
	unsigned long long pos;
	unsigned short ppb = this->ppb;
	unsigned short sectors_per_page = mtd->writesize >> 9;

	unsigned char *page_buf = mtdblk->g_page_buf;
	static int fill_block1 = 0;
	static int fill_block2 = 0;

	if (mtdblk->old_phys_block == mtdblk->new_phys_block){
		return 0;
	}

	if (!page_buf)
		return -ENOMEM;

	memset(&oobops, 0, sizeof(oobops));
	oobops.mode = MTD_OOB_AUTO;
	oobops.len = mtd->writesize;

	phys_block = mtdblk->old_phys_block;
	for (page = 0; page < ppb; page++) {
		if ( ! mtdblk->page_state[page]) {
			phys_page = (phys_block * ppb) + page;
			pos = (unsigned long long)phys_page * mtd->writesize;
page_retry:
			oobops.datbuf = &mtdblk->block_cache_data[page * mtd->writesize];
			ret = mtd->read_oob(mtd, pos, &oobops);

			if(ret ){
				printk("%s: fill_block1 phys_block:%d,page:%d,retry:%d \n",__FILE__,
				       phys_block, page, fill_block1);

				fill_block1++;
				if(fill_block1 < ECC_FAILD_RETRY)
					goto page_retry;
				else
					printk("%s: reading error when modifying phys_block %d\n!",
					       __FILE__, phys_block);
			}
			fill_block1 = 0;
		}else{
			for(sector = 0; sector < sectors_per_page; sector++)
				if( !mtdblk->page_offset_state[(page*sectors_per_page)+sector] )
					break;

			if(sector != sectors_per_page){
				phys_page = (phys_block * ppb) + page;
				pos = (unsigned long long)phys_page * mtd->writesize;
page_offset_retry:
				oobops.datbuf = page_buf;
				ret = mtd->read_oob(mtd, pos, &oobops);

				if(ret ){
					printk("%s: fill_block2 phys_block:%d,page:%d,retry:%d \n",__FILE__,
					       phys_block, page, fill_block2);

					fill_block2++;
					if(fill_block2 < ECC_FAILD_RETRY)
						goto page_offset_retry;
					else
						printk("%s: reading error when modifying phys_block %d\n!",
						       __FILE__, phys_block);
				}
				fill_block2 = 0;
				for(; sector < sectors_per_page; sector++)
					if(!mtdblk->page_offset_state[(page*sectors_per_page) + sector])
						memcpy(&mtdblk->block_cache_data[(page * mtd->writesize)+(sector<<9)], &page_buf[sector<<9], 512);
			}
		}
	}
	return 0;
}

static int mtdblock_erase_block(struct mtdblk_dev *mtdblk, int phys_block)
{
	struct mtdblk_zone_t *zone_ptr = mtdblk->zone;
        struct mtdblk_block_info *block_info = zone_ptr->block_info;
	struct mtd_info *mtd = mtdblk->mtd;
	//struct nand_chip *this = mtd->priv;
	unsigned long long pos;
	int ret;

	struct mtd_oob_ops oobops;
	memset(&oobops, 0, sizeof(oobops));
	oobops.mode = MTD_OOB_AUTO;
	oobops.ooblen = sizeof(fsoobbuf);
	oobops.ooboffs = 2;
	oobops.oobbuf = (unsigned char *)&fsoobbuf;

	ret = erase_block(mtd, phys_block);
	if(ret)
		return ret;

        /* write info need to be changed Nancy mark! */
	memset(&fsoobbuf, 0xff, sizeof(fsoobbuf));

	block_info[phys_block].lifetime++;
	fsoobbuf.lifetime = block_info[phys_block].lifetime;

	pos = (unsigned long long)phys_block * mtd->erasesize;
	mtd->write_oob(mtd, pos, &oobops);

	return ret;
}

static int mtdblock_program_block(struct mtdblk_dev *mtdblk, int phys_block)
{
	struct mtd_info *mtd = mtdblk->mtd;
	struct nand_chip *this = mtd->priv;
	struct mtdblk_block_info *block_info = mtdblk->zone->block_info;
	struct mtd_oob_ops oobops;
	unsigned short ppb = this->ppb;
	unsigned long long pos;
	int ret,i;

	dprintk("W %d-%d\n", mtdblk->virt_block,phys_block);
	memset(&oobops, 0, sizeof(oobops));
	oobops.mode = MTD_OOB_AUTO;
	oobops.len = mtd->writesize;

	oobops.ooblen = sizeof(fsoobbuf);
	oobops.ooboffs = 2;
	oobops.oobbuf = (unsigned char *)&fsoobbuf;

	/* spare area mark  need to be changed Nancy mark */
	memset((unsigned char *)&fsoobbuf, 0xff, sizeof(fsoobbuf));
	fsoobbuf.block_addr_field1 = mtdblk->virt_block;
	fsoobbuf.block_addr_field2 = mtdblk->virt_block;
	fsoobbuf.lifetime = block_info[phys_block].lifetime;
	for(i=0; i<ppb; i++){
		pos = ((unsigned long long)phys_block * mtd->erasesize) + (i * mtd->writesize);
		oobops.datbuf = &mtdblk->block_cache_data[i * mtd->writesize];
		/* clear page cache if it is out of time! */
		if (mtdblk->page_num == (phys_block * ppb) + i)
			mtdblk->page_num = ~0;

		ret = mtd->write_oob(mtd, pos, &oobops);
		if (ret)
			return -1;
	}
	return 0;
}

int mtdblock_flush_cache (struct mtdblk_dev *mtdblk)
{
	struct mtd_info *mtd = mtdblk->mtd;
	struct nand_chip *this = mtd->priv;
	struct mtd_oob_ops oobops;
	struct mtdblk_block_info *block_info = mtdblk->zone->block_info;
	unsigned short ppb = this->ppb;
	unsigned long phys_page;
	unsigned long long pos;
	unsigned char *buf = mtdblk->g_page_buf;
	int phys_block, page;
	int ret = 0;

	if (STATE_UNUSED == mtdblk->block_cache_state)
		return 0;

	memset(&oobops, 0, sizeof(oobops));
	oobops.mode = MTD_OOB_AUTO;
	oobops.len = mtd->writesize;

	oobops.ooblen = sizeof(fsoobbuf);
	oobops.ooboffs = 2 ;
	oobops.oobbuf = (unsigned char *)&fsoobbuf;

	/* un-dirty data read from old block */
	mtdblock_fill_block_cache(mtdblk);

	/* erase a free block */
	phys_block = mtdblk->new_phys_block;

restart:
	do {
			ret = erase_block(mtd, phys_block);
			block_info[phys_block].lifetime++;

			/* if erase process error, tagged to be bad block,
			 * and find a new free phys_block to program
			 */
			if( ret < 0 ) {
				mtdblk->new_phys_block = mtdblock_mark_bad_block_to_nand(mtdblk, phys_block);
				printk("%s: phys_block 0x%x erasing failed, marked bad, and find new block 0x%x\n",
				       __FILE__, phys_block, mtdblk->new_phys_block);
				phys_block = mtdblk->new_phys_block;
			}
	} while(ret < 0);

	ret = mtdblock_program_block(mtdblk, phys_block);
	/* if write process error, tagged to be bad block,
	 * and find a new free phys_block to program
	 */
	if(ret < 0){
		mtdblk->new_phys_block = mtdblock_mark_bad_block_to_nand(mtdblk, phys_block);
		printk("%s: phys_block 0x%x programing failed, marked bad, and find new block 0x%x\n",
		       __FILE__, phys_block, mtdblk->new_phys_block);
		phys_block = mtdblk->new_phys_block;
		goto restart;
	}
	/* Now, program new block done correctly */

	/* if write_verify_enable, read the block back with ECC check,
	 * if Ecc check fail, do Re-programming( back to restart )
	 */
	if( get_mtdblock_write_verify_enable() ){
		for (page = 0; page < ppb; page++) {
			phys_page = (phys_block * ppb) + page;
			pos = (unsigned long long)phys_page * mtd->writesize;

			oobops.datbuf = buf;
			ret = mtd->read_oob(mtd, pos, &oobops);
			if (ret ){
				phys_block = mtdblock_mark_bad_block_to_nand(mtdblk, phys_block);
				mtdblk->new_phys_block = phys_block;
				goto restart;
			}
		}
	}
	if (mtdblk->old_phys_block != mtdblk->new_phys_block) {
			phys_block = mtdblk->old_phys_block;
			ret = mtdblock_erase_block(mtdblk, phys_block);
			if (ret)
			{
				mtdblock_block_info_map_bad_block(mtdblk, phys_block);
				pos = (unsigned long long)phys_block * mtd->erasesize;

				mtd->block_markbad(mtd, pos);
				printk("%s:erase old_phys_block %d faild,mark it bad\n",__FUNCTION__,phys_block);
			}
	}
	mtdblock_init_block_cache(mtdblk);
	return 0;
}

static int mtdblock_mark_bad_block_to_nand(struct mtdblk_dev *mtdblk, int phys_block)
{
	struct mtd_info *mtd = mtdblk->mtd;
    //struct nand_chip *this = mtd->priv;
	unsigned long long pos;
	int ret;

    /* TODO: when reading error, there is no need to unmap mtdblk->virt_block which
     will be written, unmapping is just needed when programming occurs error. */
	mtdblock_block_lookup_unmap_entry(mtdblk, mtdblk->virt_block);
	mtdblock_block_info_map_bad_block(mtdblk, phys_block);

	ret = erase_block(mtd, phys_block);
	if(ret)
		printk("%s erase block %d for marking bad failed\n", __FILE__, phys_block);

	pos = (unsigned long long)phys_block  * mtd->erasesize;

	mtd->block_markbad(mtd, pos);

	if (mtdblock_find_free_block(mtdblk, &phys_block))
		printk("%s %d ERROR: can't find_free_block!!\n", __FILE__, __LINE__);

	mtdblock_block_lookup_map_entry(mtdblk, mtdblk->virt_block, phys_block);

	return phys_block;
}

/*
 * Cache stuff...
 *
 * Since typical flash erasable sectors are much larger than what Linux's
 * buffer cache can handle, we must implement read-modify-write on flash
 * sectors for each block write requests.  To avoid over-erasing flash sectors
 * and to speed things up, we locally cache a whole flash sector while it is
 * being written to until a different sector is required.
 */

static void erase_callback(struct erase_info *done)
{
	wait_queue_head_t *wait_q = (wait_queue_head_t *)done->priv;
	wake_up(wait_q);
}

static int erase_block (struct mtd_info *mtd, int phys_block )
{
	//struct nand_chip *this = mtd->priv;
	struct erase_info erase;
	DECLARE_WAITQUEUE(wait, current);
	wait_queue_head_t wait_q;
	int ret;

	dprintk("E %d\n", phys_block);
	/*
	 * First, let's erase the flash block.
	 */
	init_waitqueue_head(&wait_q);
	erase.mtd = mtd;
	erase.callback = erase_callback;
	erase.addr = ((unsigned long long)phys_block * mtd->erasesize); //pos;
	erase.len = mtd->erasesize;  //len;
	erase.priv = (u_long)&wait_q;

	set_current_state(TASK_INTERRUPTIBLE);
	add_wait_queue(&wait_q, &wait);

	ret = mtd->erase(mtd, &erase);
	if (ret) {
		printk("%s: erase %d block failed\n", __FILE__, phys_block);
		set_current_state(TASK_RUNNING);
		remove_wait_queue(&wait_q, &wait);
		return ret;
	}
	schedule();  /* Wait for erase to finish. */
	remove_wait_queue(&wait_q, &wait);
	return 0;
}

static int do_cached_write (struct mtdblk_dev *mtdblk, unsigned long sector,
			    int len, const char *buf)
{
	struct mtd_info *mtd = mtdblk->mtd;
	struct nand_chip *this = mtd->priv;
	unsigned short ppb = this->ppb;
	unsigned long virt_page;
	int virt_block, old_phys_block, new_phys_block, page_offset;
	int page_num_in_block;
	unsigned short sectors_per_page = mtd->writesize>> 9;

	virt_page = sector / sectors_per_page;
	page_offset = sector % sectors_per_page;
	virt_block = virt_page / ppb;
	page_num_in_block = virt_page % ppb;

	if (mtdblock_address_translate(mtdblk, virt_block, &old_phys_block) < 0) {

		mutex_lock(&mtdblk->cache_mutex);
		mtdblock_flush_cache(mtdblk);
		mutex_unlock(&mtdblk->cache_mutex);

		if (mtdblock_find_free_block(mtdblk, &new_phys_block)) {
			printk("%s %d ERROR: can't find_free_block!!\n", __FILE__, __LINE__);
			return -1;
		}
		mtdblock_setup_block_cache(mtdblk, virt_block, new_phys_block, new_phys_block);
		mtdblock_block_lookup_map_entry(mtdblk, virt_block, new_phys_block);
	} else {
		if ( STATE_USED == mtdblk->block_cache_state ) {
			if ( mtdblk->virt_block!= virt_block) {
				// Commit before we start a new cache.
				mutex_lock(&mtdblk->cache_mutex);
				mtdblock_flush_cache(mtdblk);
				mutex_unlock(&mtdblk->cache_mutex);

				if (mtdblock_find_free_block(mtdblk, &new_phys_block)) {
					printk("%s %d ERROR: can't find_free_block!!\n", __FILE__, __LINE__);
					return -1;
				}
				mtdblock_block_lookup_unmap_entry(mtdblk, virt_block);
				mtdblock_setup_block_cache(mtdblk, virt_block, new_phys_block,
						  old_phys_block);
				mtdblock_block_lookup_map_entry(mtdblk, virt_block, new_phys_block);
			} else {
			}
		} else {

			if (mtdblock_find_free_block(mtdblk, &new_phys_block)) {
				printk("%s %d ERROR: can't find_free_block!!\n", __FILE__, __LINE__);
				return -1;
			}
			mtdblock_block_lookup_unmap_entry(mtdblk, virt_block);
			mtdblock_setup_block_cache(mtdblk, virt_block, new_phys_block,
					  old_phys_block);
			mtdblock_block_lookup_map_entry(mtdblk, virt_block,
						     new_phys_block);
		}
	}
	mtdblk->page_state[page_num_in_block] = 1;
	mtdblk->page_offset_state[(page_num_in_block*sectors_per_page) + page_offset] = 1;
	memcpy(&mtdblk->block_cache_data[(page_num_in_block * mtd->writesize) +(page_offset<<9)],
	       buf,512);
	return 0;
}



static int do_cached_read (struct mtdblk_dev *mtdblk, unsigned long sector,
			   int len, char *buf)
{
	struct mtd_info *mtd = mtdblk->mtd;
	struct nand_chip *this = mtd->priv;
	struct mtd_oob_ops oobops;
	unsigned short ppb = this->ppb;
	int ret, virt_block, phys_block, page_offset;
	unsigned long virt_page, phys_page, page_num_in_block;
	unsigned long long pos;
	unsigned short sectors_per_page = mtd->writesize >> 9;

	int readfail=0;

	virt_page = sector / sectors_per_page;
	page_offset = sector % sectors_per_page;
	virt_block = virt_page / ppb;

	if ((STATE_USED == mtdblk->block_cache_state) && (virt_block == mtdblk->virt_block)) {  /* if block already in cache */
		page_num_in_block = virt_page % ppb;
		if (mtdblk->page_offset_state[(page_num_in_block*sectors_per_page) + page_offset])
			memcpy(buf, &mtdblk->block_cache_data[(page_num_in_block * mtd->writesize) + (page_offset<<SECTOR_SHIFT)], len);
		else {
			memset(&oobops, 0, sizeof(oobops));
			oobops.mode = MTD_OOB_AUTO;
			oobops.len = mtd->writesize;
			oobops.datbuf = mtdblk->page_cache_data;

			phys_page = (mtdblk->old_phys_block * ppb) + (virt_page % ppb);
			if(phys_page != mtdblk->page_num) {
				pos = (unsigned long long)phys_page * mtd->writesize;
read_retry:
				ret = mtd->read_oob(mtd, pos, &oobops);
				if(ret){
					readfail ++;
					if (readfail < ECC_FAILD_RETRY)
						goto read_retry;
					else {
						printk("%s %s WARNING: page %d uncorretable ecc or too many bit error cause bad block,move this block\n",
						__FILE__,__FUNCTION__,(int)phys_page);
						mutex_lock(&mtdblk->cache_mutex);
						mtdblock_flush_cache(mtdblk); //It has completed "move to another block" function
						mutex_unlock(&mtdblk->cache_mutex);
					}
				}

				if (readfail != 0){
					printk("%s: page %d uncorretable ecc ---> correctable ecc due to %d times read retry,but still move this block\n",
					__FILE__,(int)phys_page,readfail);
					mutex_lock(&mtdblk->cache_mutex);
					mtdblock_flush_cache(mtdblk); //It has completed "move to another block" function
					mutex_unlock(&mtdblk->cache_mutex);
					readfail = 0;
				}

				mtdblk->page_num = phys_page;
			}
			memcpy(buf, &mtdblk->page_cache_data[page_offset<<SECTOR_SHIFT],len);
		}
	} else {
		do {
			if ( mtdblock_address_translate( mtdblk, virt_block, &phys_block) < 0) {
				// In a Flash Memory device, there might be a logical block that is
				// not allcated to a physical block due to the block not being used.
				// All data returned should be set to 0xFF when accessing this logical
				// block.
				memset(buf, 0xFF, SECTOR_SIZE);
				break;
			} else {
				memset(&oobops, 0, sizeof(oobops));
				oobops.mode = MTD_OOB_AUTO;
				oobops.len = mtd->writesize;
				oobops.datbuf = mtdblk->page_cache_data;

				phys_page = (phys_block * ppb) + (virt_page % ppb);
				if(phys_page != mtdblk->page_num ) {
					pos = (unsigned long long)phys_page * mtd->writesize;
					ret = mtd->read_oob(mtd, pos, &oobops);
					if(ret){
						readfail ++;
						if (readfail < ECC_FAILD_RETRY)
							goto read_retry;
						else {
							printk("%s WARNING: page %d uncorretable ecc or too many bit error cause bad block,move this block\n",
							__FILE__,(int)phys_page);
							mutex_lock(&mtdblk->cache_mutex);
							mtdblock_flush_cache(mtdblk);
							mutex_unlock(&mtdblk->cache_mutex);

							printk("--%s %s: move to another block\n",
							__FILE__, __FUNCTION__);
							mtdblock_move_to_another_block(mtdblk, phys_block);

						}
					}

					if (readfail != 0){
						printk("%s: page %d uncorretable ecc ---> correctable ecc due to %d times read retry,but still move this block\n",
						__FILE__,(int)phys_page, readfail);
						mutex_lock(&mtdblk->cache_mutex);
						mtdblock_flush_cache(mtdblk); //It has completed "move to another block" function
						mutex_unlock(&mtdblk->cache_mutex);
						printk("--%s %s: move to another block\n", __FILE__, __FUNCTION__);
						mtdblock_move_to_another_block(mtdblk, phys_block);
						readfail = 0;
					}

					mtdblk->page_num = phys_page;
				}
				memcpy(buf, &mtdblk->page_cache_data[page_offset<<SECTOR_SHIFT],len);
				break;
			}
		} while(1);
	}

	return 0;
}


static int mtdblock_readsect(struct mtd_blktrans_dev *dev,
			      unsigned long block, char *buf)
{
	struct mtdblk_dev *mtdblk = mtdblks[dev->devnum];
	if (unlikely(dev->mtd->flags & MTD_MTDBLOCK_JZ_INVALID)) {
		return 0;
	}
	return do_cached_read(mtdblk, block, SECTOR_SIZE, buf);
}

static int mtdblock_writesect(struct mtd_blktrans_dev *dev,
			      unsigned long block, char *buf)
{
	struct mtdblk_dev *mtdblk = mtdblks[dev->devnum];
	if (unlikely(dev->mtd->flags & MTD_MTDBLOCK_JZ_INVALID)) {
		return 0;
	}

	return do_cached_write(mtdblk, block, SECTOR_SIZE, buf);
}

#if 1//defined(CONFIG_UDC_USE_LB_CACHE)
int udc_mtdblock_readsect(struct mtdblk_dev *mtdblk,
			  unsigned long block, char *buf, int size)
{
	do_cached_read(mtdblk, block, SECTOR_SIZE, buf);
	return size;
}

int udc_mtdblock_writesect(struct mtdblk_dev *mtdblk,
			      unsigned long block, char *buf)
{
	do_cached_write(mtdblk, block, SECTOR_SIZE, buf);
	return SECTOR_SIZE;
}

struct mtdblk_dev *udc_get_mtdblk(void)
{
	return g_udc_mtdblk;
}

struct mtd_info *udc_get_mtd(void)
{
	return g_udc_mtd;
}

void udc_flush_cache(struct mtdblk_dev *mtdblk)
{
	if(mtdblk != NULL){
		mutex_lock(&mtdblk->cache_mutex);
		mtdblock_flush_cache(mtdblk);
		mutex_unlock(&mtdblk->cache_mutex);

		if (mtdblk->mtd->sync)
		mtdblk->mtd->sync(mtdblk->mtd);
	}
	else {
		printk("udc_flush_cache do nothing,mtdblk is NULL\n");
	}
}

EXPORT_SYMBOL_GPL(udc_mtdblock_readsect);
EXPORT_SYMBOL_GPL(udc_mtdblock_writesect);
EXPORT_SYMBOL_GPL(udc_get_mtdblk);
EXPORT_SYMBOL_GPL(udc_get_mtd);
EXPORT_SYMBOL_GPL(udc_flush_cache);
#endif

static int mtdblock_init_mtdblk(int dev, struct mtd_info *mtd)
{
	struct mtdblk_dev *mtdblk;
	struct nand_chip *this = mtd->priv;
	unsigned short ppb, spp;
	int ret;

	mtdblk = kzalloc(sizeof(struct mtdblk_dev), GFP_KERNEL);
	if (!mtdblk)
		return -ENOMEM;

	memset(mtdblk, 0, sizeof(*mtdblk));
	mtdblk->count = 1;
	mtdblk->mtd = mtd;
	mutex_init(&mtdblk->cache_mutex);
	ppb = this->ppb;
	spp = mtdblk->mtd->writesize >> 9 ;  //spp : sectors per page

	if (!jz_mtdblock_cache || !jz_mtdblock_cache[dev]) {
		if ((mtd->flags) & MTD_NAND_CPU_MODE) {
			mtdblk->block_cache_data = vmalloc(mtdblk->mtd->erasesize);
			printk(" vmalloc 0x%x bytes for jz_mtdblock%d.\n", mtd->erasesize, dev);
		} else {
			mtdblk->block_cache_data = kmalloc(mtdblk->mtd->erasesize, GFP_KERNEL);
			printk(" kmalloc 0x%x bytes for jz_mtdblock%d.\n", mtd->erasesize, dev);
		}
		if(!mtdblk->block_cache_data) {
			printk(" Allocating block cache in mtdblock-jz.c failed.\n");
		}
	} else {
		printk(" Use the block cache allocated early in nand_base.c.\n");
		mtdblk->block_cache_data = jz_mtdblock_cache[dev]; /* allocated in nand_base.c */
	}

	mtdblk->page_state = kmalloc(ppb, GFP_KERNEL);
	mtdblk->page_offset_state = kmalloc(ppb*spp, GFP_KERNEL);
	mtdblk->page_cache_data = kmalloc(mtdblk->mtd->writesize, GFP_KERNEL);
	mtdblk->g_page_buf = kmalloc(mtdblk->mtd->writesize, GFP_KERNEL);

	if(!mtdblk->page_state ||
	   !mtdblk->page_offset_state ||
	   !mtdblk->page_cache_data ||
	   !mtdblk->g_page_buf)
		return -ENOMEM;

	mtdblock_init_block_cache(mtdblk);

	/* alloc & init zone information */
	ret = mtdblock_zone_init(mtdblk, dev);
	if(ret)
		return -ENOMEM;

	mtdblk->virt_block = -1;
	mtdblks[dev] = mtdblk;
	g_udc_mtdblk = mtdblk;
	g_udc_mtd = mtdblk->mtd;
	return 0;
}

static int mtdblock_zone_init(struct mtdblk_dev *mtdblk, int dev)
{
	struct mtd_info *mtd = mtdblk->mtd;
	struct nand_chip *this = mtd->priv;
	struct mtdblk_zone_t *zone_ptr;
	struct mtd_oob_ops oobops;
	int *badblock_table = NULL;
	int i,virt_block, phys_block;
	unsigned long long pos;

	memset(&oobops, 0, sizeof(oobops));
	oobops.mode = MTD_OOB_AUTO;
	oobops.oobbuf = (unsigned char *)&fsoobbuf;
	oobops.ooboffs = 2;
	oobops.ooblen = sizeof(fsoobbuf);

	zone_ptr = kmalloc(sizeof(struct mtdblk_zone_t), GFP_KERNEL);
	if(!zone_ptr)
		return -ENOMEM;
	memset(zone_ptr, 0, sizeof(*zone_ptr));

	badblock_table = get_jz_badblock_table();
#if defined(CONFIG_SOC_JZ4760B)
	zone_ptr->total_phys_block = mtd->rl_size >> this->phys_erase_shift;
#else
	zone_ptr->total_phys_block = mtd->size >> this->phys_erase_shift;
#endif
	zone_ptr->bad_block_num = badblock_table[dev];
	zone_ptr->total_virt_block = zone_ptr->total_phys_block - zone_ptr->bad_block_num;

	printk(KERN_DEBUG "\ntotal_phys_block:%d\nbad_block_num:%d\ntotal_virt_block:%d\n\n",
		zone_ptr->total_phys_block,zone_ptr->bad_block_num,zone_ptr->total_virt_block);

	if( zone_ptr->total_virt_block <= 0 ){
		printk("ERROR 1: bad block allowed set error!!!\n");
		printk("current partiton totoal_phys_block: %d, bad block allowed set is %d \n",zone_ptr->total_phys_block, zone_ptr->bad_block_num);
		printk("NOTICE: If you are using Yaffs2 or Jffs2, you can ignore ERROR 1 \n\n");
		zone_ptr->total_virt_block = zone_ptr->total_phys_block - 1;
	}
	zone_ptr->free_phys_block = zone_ptr->total_phys_block;
	zone_ptr->free_virt_block = zone_ptr->total_virt_block;

	zone_ptr->block_lookup = kzalloc(sizeof(unsigned int) * zone_ptr->total_virt_block, GFP_KERNEL);
	zone_ptr->block_info = kmalloc(sizeof(struct mtdblk_block_info) * zone_ptr->total_phys_block, GFP_KERNEL);

	if(!zone_ptr->block_lookup ||
		!zone_ptr->block_info)
		return -ENOMEM;

	mtdblk->zone = zone_ptr;

	for (phys_block = 0; phys_block < zone_ptr->total_phys_block; phys_block++) {
		pos = (unsigned long long)phys_block * mtd->erasesize;
		/*bad block scan. notice: badblock mark is in the last page of eraseblock*/
		if(mtd->block_isbad(mtd, pos) ){
			mtdblock_block_info_map_bad_block(mtdblk, phys_block);
			dprintk("found bad block 0x%x -> 0x%x\n",
				virt_block, phys_block);
			continue;
		}

		i = 0;
		do {
			mtd->read_oob(mtd, pos, &oobops);
			pos +=  mtd->writesize;
			i++;
		} while(fsoobbuf.block_addr_field1 != fsoobbuf.block_addr_field2
		       && i < get_mtdblock_oob_copies());

		zone_ptr->block_info[phys_block].tag = MTDBLOCK_BIT_FREE_BLOCK;
		if( fsoobbuf.lifetime == 0xFFFFFFFF )
			zone_ptr->block_info[phys_block].lifetime = 0;
		else
			zone_ptr->block_info[phys_block].lifetime = fsoobbuf.lifetime;

		virt_block = fsoobbuf.block_addr_field1;
		
		mtdblock_block_lookup_map_entry(mtdblk, virt_block, phys_block);
	}

	return 0;
}

static int mtdblock_open(struct mtd_blktrans_dev *mbd)
{
	struct mtd_info *mtd = mbd->mtd;
	struct nand_chip *this = mtd->priv;
	int dev = mbd->devnum;
	int res, i;

	DEBUG(MTD_DEBUG_LEVEL1,"mtdblock_open\n");
	/* If the partition doesn't work over mtdblock-jz, just return! */
	for (i = 1; mtd_blkn[i]; i++) {
		if (dev == mtd_blkn[i])
			return 0;
	}
	if (mtd_blk[0]) { /* mtd_blk[0] is the num of blocks specified in cmdline */
		for (i = 1; mtd_blk[i]; i++) {
			if (dev == mtd_blk[i])
				break;
		}
		if (!mtd_blk[i])
			return 0;
	}

	if (mtdblks[dev]) {
		mtdblks[dev]->count++;
		if (mtdblklog)
			printk("%s: increase use count\n",__FUNCTION__);
		return 0;
	}

	/* OK, it's not open. Create cache info for it */
	if(this == NULL)
		printk("this is part mtd info\n");

	res = mtdblock_init_mtdblk(dev, mtd);

	return res;
}

static int mtdblock_release(struct mtd_blktrans_dev *mbd)
{
	struct mtd_info *mtd = mbd->mtd;
	int dev = mbd->devnum;
	struct mtdblk_dev *mtdblk;
       	struct mtdblk_zone_t *zone_ptr;
	int i;

	/* If the partition doesn't work over mtdblock-jz, just return! */
	for (i = 1; mtd_blkn[i]; i++) {
		if (dev == mtd_blkn[i])
			return 0;
	}
	if (mtd_blk[0]) {
		for (i = 1; mtd_blk[i]; i++) {
			if (dev == mtd_blk[i])
				break;
		}
		if (!mtd_blk[i])
			return 0;
	}

	if (mtd->flags & MTD_MTDBLOCK_JZ_INVALID) {
		return 0;
	}

	mtdblk = mtdblks[dev];
	zone_ptr = mtdblk->zone;

   	mutex_lock(&mtdblk->cache_mutex);
	mtdblock_flush_cache(mtdblk);
	mutex_unlock(&mtdblk->cache_mutex);

	if (!--mtdblk->count) {
		/* It was the last usage. Free the device */
		if (mtdblk->mtd->sync)
			mtdblk->mtd->sync(mtdblk->mtd);

		kfree(zone_ptr->block_info);
		zone_ptr->block_info = NULL;
		kfree(zone_ptr->block_lookup);
		zone_ptr->block_lookup = NULL;
		kfree(zone_ptr);
		zone_ptr = NULL;

		/* If it was allocated in mtdblock-jz itself, free it here. */
		if (!jz_mtdblock_cache || !jz_mtdblock_cache[dev]) {
			if ((mtd->flags) & MTD_NAND_CPU_MODE) {
				vfree(mtdblk->block_cache_data);
				mtdblk->block_cache_data = NULL;
				printk(" vfree 0x%x bytes for jz_mtdblock%d.\n", mtd->erasesize, dev);
			} else {
				kfree(mtdblk->block_cache_data);
				mtdblk->block_cache_data = NULL;
				printk(" kfree 0x%x bytes for jz_mtdblock%d.\n", mtd->erasesize, dev);
			}
		}
		kfree(mtdblk->page_state);
		mtdblk->page_state = NULL;
		kfree(mtdblk->page_offset_state);
		mtdblk->page_offset_state = NULL;
		kfree(mtdblk->page_cache_data);
		mtdblk->page_cache_data = NULL;
		kfree(mtdblk->g_page_buf);
		mtdblk->g_page_buf = NULL;
		kfree(mtdblk);
		mtdblk = NULL;
		mtdblks[dev] = NULL;
		g_udc_mtdblk = NULL;
		g_udc_mtd = NULL;

	} else if (mtdblklog) {
			printk("%s: decrease use count\n", __FUNCTION__);
	}

	return 0;
}

static int mtdblock_flush(struct mtd_blktrans_dev *dev)
{
	struct mtdblk_dev *mtdblk = mtdblks[dev->devnum];

	mutex_lock(&mtdblk->cache_mutex);
	mtdblock_flush_cache(mtdblk);
	mutex_unlock(&mtdblk->cache_mutex);

	if (mtdblk->mtd->sync)
		mtdblk->mtd->sync(mtdblk->mtd);
	return 0;
}

static void mtdblock_add_mtd(struct mtd_blktrans_ops *tr, struct mtd_info *mtd)
{
	struct mtd_blktrans_dev *dev = kmalloc(sizeof(*dev), GFP_KERNEL);
	//struct nand_chip *this = mtd->priv;
	int *badblock_table = NULL;
	int reserved_sectors;

	if (!dev)
		return;

	memset(dev, 0, sizeof(*dev));
	dev->mtd = mtd;
	dev->devnum = mtd->index;

	badblock_table = get_jz_badblock_table();
	reserved_sectors = (badblock_table[dev->devnum ] * mtd->erasesize) >> 9;//<< (this->phys_erase_shift - 9) ;
	dev->size = (mtd->size >> 9) - reserved_sectors;
	dev->tr = tr;

	if (!(mtd->flags & MTD_WRITEABLE))
		dev->readonly = 1;

	add_mtd_blktrans_dev(dev);
}

static int mtdblock_getgeo(struct mtd_blktrans_dev *dev, struct hd_geometry *geo)
{
	memset(geo, 0, sizeof(*geo));
	geo->heads     = 4;
	geo->sectors   = 16;
	geo->cylinders = dev->size/(4*16);
	return 0;
}

static void mtdblock_remove_dev(struct mtd_blktrans_dev *dev)
{
	del_mtd_blktrans_dev(dev);
	kfree(dev);
	dev = NULL;
}

static int __init mtdblk_setup(char *str)
{
	str = get_options(str, ARRAY_SIZE(mtd_blk), mtd_blk);
	if (!str || !*str)
		return 0;

	return 1;
}

__setup("mtdblk=", mtdblk_setup);

static int __init mtdblkn_setup(char *str)
{
	str = get_options(str, ARRAY_SIZE(mtd_blkn), mtd_blkn);
	if (!str || !*str)
		return 0;

	return 1;
}

__setup("mtdblkn=", mtdblkn_setup);

static int __init mtdblklog_setup(char *str)
{
	if (*str)
		return 0;
	mtdblklog = 1;
	return 1;
}

__setup("mtdblklog", mtdblklog_setup);


static struct mtd_blktrans_ops mtdblock_tr = {
	.name		  = "mtdblock",
	.major		  = 31,
	.part_bits	  = 0,
	.blksize          = 512,
	.open		  = mtdblock_open,
	.flush		  = mtdblock_flush,
	.getgeo           = mtdblock_getgeo,
	.release	  = mtdblock_release,
	.readsect	  = mtdblock_readsect,
	.writesect	  = mtdblock_writesect,
	.add_mtd	  = mtdblock_add_mtd,
	.remove_dev	  = mtdblock_remove_dev,
	.owner		  = THIS_MODULE,
};

static int __init init_mtdblock(void)
{
	return register_mtd_blktrans(&mtdblock_tr);
}

static void __exit cleanup_mtdblock(void)
{
	deregister_mtd_blktrans(&mtdblock_tr);
}

module_init(init_mtdblock);
module_exit(cleanup_mtdblock);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nancy <yrtan@ingenic.cn> et al.");
MODULE_DESCRIPTION("Caching read/erase/writeback block device emulation access to MTD devices");
