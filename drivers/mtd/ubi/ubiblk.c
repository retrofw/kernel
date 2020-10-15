/*
 * Direct UBI block device access
 *
 * (C) 2000-2003 Nicolas Pitre <nico@cam.org>
 * (C) 1999-2003 David Woodhouse <dwmw2@infradead.org>
 * (C) 2008 Yurong Tan <nancydreaming@gmail.com> :
 *        borrow mtdblock.c to work on top of UBI
 */

#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/blkpg.h>
#include <linux/freezer.h>
#include <asm/uaccess.h>

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/hdreg.h>
#include <linux/mutex.h>
#include "ubi.h"
#include "ubiblk.h"

#define UBIBLK_UNMAPPED 0
#define UBIBLK_SECTOR_SIZE 512

extern void ubi_open_blkdev(int ubi_num, int vol_id, int mode);
extern void ubi_close_blkdev(struct ubi_volume_desc *desc);
static void ubiblk_setup_writecache(struct ubiblk_dev *ubiblk, int virt_block);				   
static int ubiblk_flush_writecache(struct ubiblk_dev *ubiblk);
extern int ubiblk_leb_change(struct ubiblk_dev *ubiblk);

struct ubiblk_dev *ubiblks[UBI_MAX_VOLUMES];
static unsigned short subpage_shift; 

static struct ubi_blktrans_dev *g_udc_ubiblk;	//add by ylyuan for lb_cache

static int ubiblk_flush_writecache(struct ubiblk_dev *ubiblk)
{
	if (STATE_UNUSED == ubiblk->write_cache_state)
		return 0;
	ubiblk_leb_change(ubiblk);
	ubiblk->write_cache_state = STATE_UNUSED;

	return 0;
}

static void ubiblk_setup_writecache(struct ubiblk_dev *ubiblk, int virt_block)
{
	struct ubi_volume_desc *uv = ubiblk->uv;
	struct ubi_device *ubi = uv->vol->ubi;
	int ppb = ubi->leb_size / ubi->min_io_size;
	unsigned short spp = ubi->min_io_size >> subpage_shift;

	ubiblk->vbw = virt_block;
	ubiblk->write_cache_state = STATE_USED;
		
	memset(ubiblk->page_sts, 0, ppb);
	memset(ubiblk->subpage_sts, 0, ppb*spp);
}

static int do_cached_write (struct ubiblk_dev *ubiblk, unsigned long sector, 
			    int len, const char *buf)
{
	struct ubi_volume_desc *uv = ubiblk->uv;
	struct ubi_device *ubi = uv->vol->ubi;
	int ppb = ubi->leb_size / ubi->min_io_size;
	unsigned short sectors_per_page =  ubi->min_io_size / len;
	unsigned short sectors_in_page_shift = ffs(sectors_per_page) - 1;
	unsigned short page_shift =  ffs(ubi->min_io_size) - 1;
	unsigned short virt_block, page, subpage;
	unsigned long virt_page; 
	
	virt_page = sector / sectors_per_page;
	subpage = sector % sectors_per_page;
	virt_block = virt_page / ppb; 
	page = virt_page % ppb;
	
	if(ubi_is_mapped(uv, virt_block) == UBIBLK_UNMAPPED ){
		mutex_lock(&ubiblk->cache_mutex);
		ubiblk_flush_writecache(ubiblk);
		mutex_unlock(&ubiblk->cache_mutex);

		ubiblk_setup_writecache(ubiblk, virt_block);
	} else {
		if ( STATE_USED == ubiblk->write_cache_state ) {
			if ( ubiblk->vbw != virt_block) {
			// Commit before we start a new cache.
				mutex_lock(&ubiblk->cache_mutex);
				ubiblk_flush_writecache(ubiblk);
				mutex_unlock(&ubiblk->cache_mutex);
				
				ubiblk_setup_writecache(ubiblk, virt_block);
			} else {
				//printk("cache hit: 0x%x\n", virt_page);
			}
		} else {
//			printk("with existing mapping\n");
			ubiblk_setup_writecache(ubiblk, virt_block);
		}                        
	}		
	ubiblk->page_sts[page] = 1;
	ubiblk->subpage_sts[(page<<sectors_in_page_shift) + subpage] = 1;
	memcpy(&ubiblk->write_cache[(page<<page_shift) +(subpage<<subpage_shift)],
	       buf,len);
	return 0;
}

static int do_cached_read (struct ubiblk_dev *ubiblk, unsigned long sector, 
			   int len, char *buf)
{
	struct ubi_volume_desc *uv = ubiblk->uv;
	int ppb = uv->vol->ubi->leb_size / uv->vol->ubi->min_io_size;
	unsigned short sectors_per_page =  uv->vol->ubi->min_io_size >> 9;
	unsigned short page_shift =  ffs(uv->vol->ubi->min_io_size) - 1;
	unsigned short virt_block, page, page_offset; 	
	unsigned long virt_page; 
		
	virt_page = sector / sectors_per_page;
	page_offset = sector % sectors_per_page;
	virt_block = virt_page / ppb; 
	page = virt_page % ppb;

	if(ubiblk->vbw == virt_block){		
		mutex_lock(&ubiblk->cache_mutex);
		ubiblk_flush_writecache(ubiblk);
		mutex_unlock(&ubiblk->cache_mutex);
	}

	if ( ubi_is_mapped( uv, virt_block) == UBIBLK_UNMAPPED){
		/* In a Flash Memory device, there might be a logical block that is
		  * not allcated to a physical block due to the block not being used.
		  * All data returned should be set to 0xFF when accessing this logical 
		  * block.
		  */	
		  
		//printk("address translate fail\n");
		memset(buf, 0xFF, UBIBLK_SECTOR_SIZE);
	} else {

		if( ubiblk->vbr != virt_block ||ubiblk->read_cache_state == STATE_UNUSED ){
			ubiblk->vbr = virt_block;
			ubi_leb_read(uv, virt_block, ubiblk->read_cache, 0, uv->vol->usable_leb_size, 0);				
			ubiblk->read_cache_state = STATE_USED;
		}
		memcpy(buf, &ubiblk->read_cache[(page<<page_shift)+(page_offset<<9)], len);
	}
	return 0;
}

static int ubiblk_readsect(struct ubi_blktrans_dev *dev,
			      unsigned long block, char *buf)
{
	struct ubiblk_dev *ubiblk = ubiblks[dev->devnum];
	return do_cached_read(ubiblk, block, UBIBLK_SECTOR_SIZE, buf);
}

static int ubiblk_writesect(struct ubi_blktrans_dev *dev,
			      unsigned long block, char *buf)
{
	struct ubiblk_dev *ubiblk = ubiblks[dev->devnum];
	return do_cached_write(ubiblk, block, UBIBLK_SECTOR_SIZE, buf);
}

#if defined(CONFIG_UDC_USE_LB_CACHE)
#define SECTOR_SIZE	512
int udc_ubiblk_readsect(struct ubi_blktrans_dev *dev,
			  unsigned long block, char *buf, int size)
{
	struct ubiblk_dev *ubiblk = ubiblks[dev->devnum];
	do_cached_read(ubiblk, block, SECTOR_SIZE, buf);
	return size;
}

int udc_ubiblk_writesect(struct ubi_blktrans_dev *dev,
			      unsigned long block, char *buf)
{
	struct ubiblk_dev *ubiblk = ubiblks[dev->devnum];
	do_cached_write(ubiblk, block, SECTOR_SIZE, buf);
	return SECTOR_SIZE;
}

struct ubi_blktrans_dev *udc_get_ubiblk(void)
{
	return g_udc_ubiblk;
}

void udc_flush_writecache(struct ubi_blktrans_dev *dev)
{
	struct ubiblk_dev *ubiblk = ubiblks[dev->devnum];
	if(ubiblk != NULL){
		mutex_lock(&ubiblk->cache_mutex);
		ubiblk_flush_writecache(ubiblk);
		mutex_unlock(&ubiblk->cache_mutex);

		if (ubiblk->uv->vol->ubi->mtd->sync)
			ubiblk->uv->vol->ubi->mtd->sync(ubiblk->uv->vol->ubi->mtd);
	}
	else {
		printk("udc_flush_writecache do nothing, ubiblk is NULL\n");
	}
}

EXPORT_SYMBOL_GPL(udc_ubiblk_readsect);
EXPORT_SYMBOL_GPL(udc_ubiblk_writesect);
EXPORT_SYMBOL_GPL(udc_get_ubiblk);
EXPORT_SYMBOL_GPL(udc_flush_writecache);
#endif	/* CONFIG_UDC_USE_LB_CACHE */

//static int ubiblk_init_vol(int dev, struct ubi_volume_desc *uv)
static int ubiblk_init_vol(struct ubi_blktrans_dev *dev, struct ubi_volume_desc *uv)
{
	struct ubiblk_dev *ubiblk;
	struct ubi_device *ubi = uv->vol->ubi;
	int ppb = ubi->leb_size / ubi->min_io_size;
	unsigned short spp = ubi->min_io_size >> subpage_shift;
			
	ubiblk = kmalloc(sizeof(struct ubiblk_dev), GFP_KERNEL);
	if (!ubiblk)
		return -ENOMEM;

	memset(ubiblk, 0, sizeof(*ubiblk));

	ubiblk->count = 1;
	ubiblk->uv = uv;
	mutex_init (&ubiblk->cache_mutex);

#if defined(CONFIG_MTD_NAND_DMA) && !defined(CONFIG_MTD_NAND_DMABUF)
	ubiblk->write_cache = kmalloc(ubiblk->uv->vol->usable_leb_size, GFP_KERNEL); 
	ubiblk->read_cache = kmalloc(ubiblk->uv->vol->usable_leb_size, GFP_KERNEL);
	ubiblk->page_sts = kmalloc(ppb, GFP_KERNEL);
	ubiblk->subpage_sts = kmalloc(ppb*spp, GFP_KERNEL);
#else
	ubiblk->write_cache = vmalloc(ubiblk->uv->vol->usable_leb_size); 
	ubiblk->read_cache = vmalloc(ubiblk->uv->vol->usable_leb_size);
	ubiblk->page_sts = vmalloc(ppb);
	ubiblk->subpage_sts = vmalloc(ppb*spp);
#endif

	if(!ubiblk->write_cache || 
		!ubiblk->read_cache ||
		!ubiblk->page_sts ||
		!ubiblk->subpage_sts)
		return -ENOMEM;

	ubiblk->write_cache_state = STATE_UNUSED;
	ubiblk->read_cache_state = STATE_UNUSED;

	//ubiblks[dev] = ubiblk;
	ubiblks[dev->devnum] = ubiblk;
	g_udc_ubiblk = dev;	//add by ylyuan
	DEBUG(MTD_DEBUG_LEVEL1, "ok\n");
	return 0;
}

static int ubiblk_open(struct block_device *bdev, fmode_t fmode)
{
	struct ubi_volume_desc *desc;
	struct ubi_blktrans_dev *dev;
	int ubi_num = ubi_major2num(imajor(bdev->bd_inode));
	int vol_id = iminor(bdev->bd_inode);
	int mode;
	int ret = 0;

	if (fmode & FMODE_WRITE)
		mode = UBI_READWRITE;
	else
		mode = UBI_READONLY;

	dev = bdev->bd_disk->private_data;
	if (ubiblks[dev->devnum]) {
		ubiblks[dev->devnum]->count++;
		ubi_open_blkdev(ubi_num, vol_id, mode);
		printk("%s: increase use count\n",__FUNCTION__);
		return 0;
	}
	
	desc = ubi_open_volume(ubi_num, vol_id, mode);
	if (IS_ERR(desc))
		return PTR_ERR(desc);
	
	desc->vol->bdev_mode = mode;
	dev->uv = desc; 

	subpage_shift = ffs(UBIBLK_SECTOR_SIZE)-1; 
	//ret = ubiblk_init_vol(dev->devnum, desc);
	ret = ubiblk_init_vol(dev, desc);
	return ret;
}

static int ubiblk_release(struct ubi_blktrans_dev *ubd)
{
	int dev = ubd->devnum;
	struct ubiblk_dev *ubiblk = ubiblks[dev];
	struct ubi_device *ubi = ubiblk->uv->vol->ubi;

	mutex_lock(&ubiblk->cache_mutex);
	ubiblk_flush_writecache(ubiblk);
	mutex_unlock(&ubiblk->cache_mutex);

	ubiblk->count --;
	if (!ubiblk->count) {
		/* It was the last usage. Free the device */
		ubiblks[dev] = NULL;
		g_udc_ubiblk = NULL;	//add by ylyuan

		if (ubi->mtd->sync)
			ubi->mtd->sync(ubi->mtd);

#if defined(CONFIG_MTD_NAND_DMA) && !defined(CONFIG_MTD_NAND_DMABUF)
		kfree(ubiblk->write_cache);
		kfree(ubiblk->read_cache);
#else
		vfree(ubiblk->write_cache);
		vfree(ubiblk->read_cache);
#endif

		//modify by ylyuan
		ubi_close_volume(ubiblk->uv);
		kfree(ubiblk);

		return 0;
	}
	else{
		printk("%s: decrease use count\n",__FUNCTION__);
		ubi_close_blkdev(ubiblk->uv);
		return 0;
	}
	return 1;
}
static int ubiblk_flush(struct ubi_blktrans_dev *dev)
{
	struct ubiblk_dev *ubiblk = ubiblks[dev->devnum];
	struct ubi_device *ubi = ubiblk->uv->vol->ubi;
	
	mutex_lock(&ubiblk->cache_mutex);
	ubiblk_flush_writecache(ubiblk);
	mutex_unlock(&ubiblk->cache_mutex);

	if (ubi->mtd->sync)
		ubi->mtd->sync(ubi->mtd);
	return 0;
}

void ubiblk_add_vol_dev(struct ubi_blktrans_ops *tr, struct ubi_volume *vol)
{
	struct ubi_blktrans_dev *dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return;

	dev->devnum = vol->vol_id;
	dev->size = vol->used_bytes >> 9;
	dev->tr = tr;

	if (vol->bdev_mode == UBI_READONLY)
		dev->readonly = 1;

	vol->ubi->bdev_major = tr->major; 

	add_ubi_blktrans_dev(dev);
}

void ubiblk_remove_vol_dev(struct ubi_blktrans_dev *dev)
{
	del_ubi_blktrans_dev(dev);
	kfree(dev);
}

static int ubiblk_getgeo(struct ubi_blktrans_dev *dev, struct hd_geometry *geo)
{
	memset(geo, 0, sizeof(*geo));
	geo->heads     = 4;
	geo->sectors   = 16;
	geo->cylinders = dev->size/(4*16); 
	return 0;
}

static struct ubi_blktrans_ops ubiblk_tr = {
	.name		         = "ubiblock",
	.major                   = 0,
	.part_bits	         = 0,
	.blksize 	         = UBIBLK_SECTOR_SIZE,
	.open		         = ubiblk_open,
	.release	         = ubiblk_release,
	.readsect	         = ubiblk_readsect,
	.writesect	         = ubiblk_writesect,
	.getgeo                  = ubiblk_getgeo,
	.flush		         = ubiblk_flush,
	.add_vol	         = ubiblk_add_vol_dev,
	.remove_vol	         = ubiblk_remove_vol_dev,
	.owner		         = THIS_MODULE,
};

static int __init init_ubiblock(void)
{
	return register_ubi_blktrans(&ubiblk_tr);
}

static void __exit cleanup_ubiblock(void)
{
	deregister_ubi_blktrans(&ubiblk_tr);
}

module_init(init_ubiblock);
module_exit(cleanup_ubiblock);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nicolas Pitre <nico@cam.org> , Yurong Tan <nancydreaming@gmail.com>");
MODULE_DESCRIPTION("Caching read/erase/writeback block device emulation access to UBI volumes");
