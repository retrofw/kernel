/*
 * (C) 2003 David Woodhouse <dwmw2@infradead.org>
 *
 * Interface to Linux 2.5 block layer for UBI 'translation layers'.
 * 
 * 2008 Yurong Tan <nancydreaming@gmail.com>
 *
 * borrow from mtd_blkdevs.c for building block device layer on top of UBI  
 *
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/blkpg.h>
#include <linux/freezer.h>
#include <linux/spinlock.h>
#include <linux/hdreg.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <asm/uaccess.h>
#include "ubi.h"
#include "ubiblk.h"

static LIST_HEAD(blktrans_majors);
extern struct mutex vol_table_mutex;  
extern struct ubi_volume *vol_table[];      

extern void register_vol_user (struct vol_notifier *new);
extern int unregister_vol_user (struct vol_notifier *old);
extern int ubi_major2num(int major);

static int do_blktrans_request(struct ubi_blktrans_ops *tr,
			       struct ubi_blktrans_dev *dev,
			       struct request *req)
{
	unsigned long block, nsect;
	char *buf;

	block = blk_rq_pos(req) << 9 >> tr->blkshift;
	nsect = blk_rq_cur_bytes(req) >> tr->blkshift;

	buf = req->buffer;

	if (!blk_fs_request(req))
		return -EIO;

	if (blk_rq_pos(req) + blk_rq_cur_sectors(req) >
	    get_capacity(req->rq_disk))
		return -EIO;

	switch(rq_data_dir(req)) {
	case READ:
		for (; nsect > 0; nsect--, block++, buf += tr->blksize)
			if (tr->readsect(dev, block, buf))
				return -EIO;
		return 0;

	case WRITE:
		if (!tr->writesect)
			return -EIO;

		for (; nsect > 0; nsect--, block++, buf += tr->blksize)
			if (tr->writesect(dev, block, buf))
				return -EIO;
		return 0;

	default:
		printk(KERN_NOTICE "Unknown request %u\n", rq_data_dir(req));
		return -EIO;
	}
}

static int ubi_blktrans_thread(void *arg)
{
	struct ubi_blktrans_ops *tr = arg;
	struct request_queue *rq = tr->blkcore_priv->rq;
	struct request *req = NULL;

	/* we might get involved when memory gets low, so use PF_MEMALLOC */
	current->flags |= PF_MEMALLOC;

	spin_lock_irq(rq->queue_lock);

	while (!kthread_should_stop()) {
		struct ubi_blktrans_dev *dev;
		int res;

		if (!req && !(req = blk_fetch_request(rq))) {
			set_current_state(TASK_INTERRUPTIBLE);
			spin_unlock_irq(rq->queue_lock);
			schedule();
			spin_lock_irq(rq->queue_lock);
			continue;
		}

		dev = req->rq_disk->private_data;
		tr = dev->tr;

		spin_unlock_irq(rq->queue_lock);

		mutex_lock(&dev->lock);
		res = do_blktrans_request(tr, dev, req);
		mutex_unlock(&dev->lock);

		spin_lock_irq(rq->queue_lock);

		if (!__blk_end_request_cur(req, res))
			req = NULL;
	}

	if (req)
		__blk_end_request_all(req, -EIO);

	spin_unlock_irq(rq->queue_lock);

	return 0;
}

static void ubi_blktrans_request(struct request_queue *rq)
{
	struct ubi_blktrans_ops *tr = rq->queuedata;
	wake_up_process(tr->blkcore_priv->thread);
}

static int blktrans_open(struct block_device *bdev, fmode_t mode)
{
	struct ubi_blktrans_dev *dev = bdev->bd_disk->private_data;
	struct ubi_blktrans_ops *tr = dev->tr;
	int ret = 0;

	if (!try_module_get(tr->owner))
		goto out_tr;

	if(!tr->open) {
		return -1;
	}
	else {
		ret = tr->open(bdev, mode);
	}

	return ret;
out_tr:
	module_put(tr->owner);
	return -1;
}

static int blktrans_release(struct gendisk *disk, fmode_t mode)
{
	struct ubi_blktrans_dev *dev = disk->private_data;
	struct ubi_blktrans_ops *tr = dev->tr;
	struct ubi_volume_desc *desc;
	int ret = 0;

	desc = dev->uv;

	if (tr->release)
		ret = tr->release(dev);

	module_put(tr->owner);

	return ret;
}

static int blktrans_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	struct ubi_blktrans_dev *dev = bdev->bd_disk->private_data;

	if (dev->tr->getgeo)
		return dev->tr->getgeo(dev, geo);
	return -ENOTTY;
}

static int blktrans_ioctl(struct block_device *bdev, fmode_t mode,
			      unsigned int cmd, unsigned long arg)
{
	struct ubi_blktrans_dev *dev = bdev->bd_disk->private_data;
	struct ubi_blktrans_ops *tr = dev->tr;

	switch (cmd) {
	case BLKFLSBUF:
		if (tr->flush)
			return tr->flush(dev);
		/* The core code did the work, we had nothing to do. */
		return 0;
	default:
		return -ENOTTY;
	}
}

static struct block_device_operations ubi_blktrans_ops = {
	.owner		= THIS_MODULE,
	.open		= blktrans_open,
	.release	= blktrans_release,
	.locked_ioctl	= blktrans_ioctl,
	.getgeo		= blktrans_getgeo,
};

int add_ubi_blktrans_dev(struct ubi_blktrans_dev *new)
{
	struct ubi_blktrans_ops *tr = new->tr;
	struct ubi_blktrans_dev *d;
	int last_devnum = -1;
	struct gendisk *gd;

	if (mutex_trylock(&vol_table_mutex)) {
		mutex_unlock(&vol_table_mutex);
		BUG();
	}

	list_for_each_entry(d, &tr->devs, list) {
		if (new->devnum == -1) {
			/* Use first free number */
			if (d->devnum != last_devnum+1) {
				/* Found a free devnum. Plug it in here */
				new->devnum = last_devnum+1;
				list_add_tail(&new->list, &d->list);
				goto added;
			}
		} else if (d->devnum == new->devnum) {
			/* Required number taken */
			return -EBUSY;
		} else if (d->devnum > new->devnum) {
			/* Required number was free */
			list_add_tail(&new->list, &d->list);
			goto added;
		}
		last_devnum = d->devnum;
	}
	if (new->devnum == -1)
		new->devnum = last_devnum+1;

	if ((new->devnum << tr->part_bits) > 256) {
		return -EBUSY;
	}

	list_add_tail(&new->list, &tr->devs);
 added:
	mutex_init(&new->lock);
	if (!tr->writesect)
		new->readonly = 1;

	gd = alloc_disk(1 << tr->part_bits);
	if (!gd) {
		list_del(&new->list);
		return -ENOMEM;
	}
	gd->major = tr->major;
	gd->first_minor = (new->devnum) << tr->part_bits;
	gd->fops = &ubi_blktrans_ops;

	if (tr->part_bits)
		if (new->devnum < 26)
			snprintf(gd->disk_name, sizeof(gd->disk_name),
				 "%s%c", tr->name, 'a' + new->devnum);
		else
			snprintf(gd->disk_name, sizeof(gd->disk_name),
				 "%s%c%c", tr->name,
				 'a' - 1 + new->devnum / 26,
				 'a' + new->devnum % 26);
	else
		snprintf(gd->disk_name, sizeof(gd->disk_name),
			 "%s%d", tr->name, new->devnum);

	/* 2.5 has capacity in units of 512 bytes while still
	   having BLOCK_SIZE_BITS set to 10. Just to keep us amused. */
	set_capacity(gd, (new->size * tr->blksize) >> 9);

	gd->private_data = new;
	new->blkcore_priv = gd;
	gd->queue = tr->blkcore_priv->rq;

	if (new->readonly)
		set_disk_ro(gd, 1);

	add_disk(gd);
	return 0;
}

int del_ubi_blktrans_dev(struct ubi_blktrans_dev *old)
{
	if (mutex_trylock(&vol_table_mutex)) {
		mutex_unlock(&vol_table_mutex);
		BUG();
	}

	list_del(&old->list);

	del_gendisk(old->blkcore_priv);
	put_disk(old->blkcore_priv);

	return 0;
}
#if 0
static void blktrans_notify_remove(struct ubi_volume *vol)
{
	struct ubi_blktrans_ops *tr;
	struct ubi_blktrans_dev *dev, *next;

	list_for_each_entry(tr, &blktrans_majors, list)
		list_for_each_entry_safe(dev, next, &tr->devs, list)
			if (dev->uv->vol == vol)
				tr->remove_vol(dev);

}
#endif
static void blktrans_notify_remove(struct ubi_volume *vol)
{
	struct list_head *this, *this2, *next;

	list_for_each(this, &blktrans_majors) {
		struct ubi_blktrans_ops *tr = list_entry(this, struct ubi_blktrans_ops, list);

		list_for_each_safe(this2, next, &tr->devs) {
			struct ubi_blktrans_dev *dev = list_entry(this2, struct ubi_blktrans_dev, list);
			if (dev->uv->vol == vol)
				tr->remove_vol(dev);
		}
	}
}
#if 0
static void blktrans_notify_add(struct ubi_volume *vol)
{
	struct ubi_blktrans_ops *tr;

	list_for_each_entry(tr, &blktrans_majors, list)
		tr->add_vol(tr, vol);
}
#endif
static void blktrans_notify_add(struct ubi_volume *vol)
{
	struct list_head *this;

	list_for_each(this, &blktrans_majors) {
		struct ubi_blktrans_ops *tr = list_entry(this, struct ubi_blktrans_ops, list);
		tr->add_vol(tr,vol);
	}
}

static struct vol_notifier blktrans_notifier = {
	.add = blktrans_notify_add,
	.remove = blktrans_notify_remove,
};

int register_ubi_blktrans(struct ubi_blktrans_ops *tr)
{
	int i;

	/* Register the notifier if/when the first device type is
	   registered, to prevent the link/init ordering from fucking
	   us over. */
	if (!blktrans_notifier.list.next)
		register_vol_user(&blktrans_notifier);

	tr->blkcore_priv = kzalloc(sizeof(*tr->blkcore_priv), GFP_KERNEL);
	if (!tr->blkcore_priv)
		return -ENOMEM;

	mutex_lock(&vol_table_mutex);
	tr->major = register_blkdev(0, tr->name);

 	spin_lock_init(&tr->blkcore_priv->queue_lock);

	tr->blkcore_priv->rq = blk_init_queue(ubi_blktrans_request, &tr->blkcore_priv->queue_lock);
	if (!tr->blkcore_priv->rq) {
		unregister_blkdev(tr->major, tr->name);
		kfree(tr->blkcore_priv);
		mutex_unlock(&vol_table_mutex);
		return -ENOMEM;
	}

	tr->blkcore_priv->rq->queuedata = tr;
	blk_queue_logical_block_size(tr->blkcore_priv->rq, tr->blksize); //ylyuan
	tr->blkshift = ffs(tr->blksize) - 1;

	tr->blkcore_priv->thread = kthread_run(ubi_blktrans_thread, tr,
			"%sd", tr->name);

	if (IS_ERR(tr->blkcore_priv->thread)) {
		int ret = PTR_ERR(tr->blkcore_priv->thread);
		blk_cleanup_queue(tr->blkcore_priv->rq);
		unregister_blkdev(tr->major, tr->name);
		kfree(tr->blkcore_priv);
		mutex_unlock(&vol_table_mutex);
		return ret;
	}

	INIT_LIST_HEAD(&tr->devs);
	list_add(&tr->list, &blktrans_majors);

	for (i=0; i<UBI_MAX_DEVICES; i++) {
		if (vol_table[i])
			tr->add_vol(tr, vol_table[i]);
	}

	mutex_unlock(&vol_table_mutex);
	return 0;
}

int deregister_ubi_blktrans(struct ubi_blktrans_ops *tr)
{
	struct ubi_blktrans_dev *dev, *next;

	mutex_lock(&vol_table_mutex);

	/* Clean up the kernel thread */
	kthread_stop(tr->blkcore_priv->thread);

	/* Remove it from the list of active majors */
	list_del(&tr->list);

	list_for_each_entry_safe(dev, next, &tr->devs, list)
		tr->remove_vol(dev);

	blk_cleanup_queue(tr->blkcore_priv->rq);
	unregister_blkdev(tr->major, tr->name);

	mutex_unlock(&vol_table_mutex);

	kfree(tr->blkcore_priv);

	BUG_ON(!list_empty(&tr->devs));
	return 0;
}

static void __exit ubi_blktrans_exit(void)
{
	/* No race here -- if someone's currently in register_ubi_blktrans
	   we're screwed anyway. */
	if (blktrans_notifier.list.next)
		unregister_vol_user(&blktrans_notifier);
}

module_exit(ubi_blktrans_exit);

EXPORT_SYMBOL_GPL(register_ubi_blktrans);
EXPORT_SYMBOL_GPL(deregister_ubi_blktrans);
EXPORT_SYMBOL_GPL(add_ubi_blktrans_dev);
EXPORT_SYMBOL_GPL(del_ubi_blktrans_dev);

MODULE_AUTHOR("David Woodhouse <dwmw2@infradead.org>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Common interface to block layer for UBI 'translation layers'");
