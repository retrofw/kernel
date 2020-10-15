/*
 *  linux/drivers/char/jzchar/jzchars.c
 *
 *  JzSOC char device family common layer.
 */
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/serial.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>

#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>

#include <asm/jzsoc.h>
#include "jzchars.h"

LIST_HEAD(jz_char_devs);

int jz_register_chrdev(unsigned char minor, const char *name,
			 struct file_operations *fops, void *private)
{
	struct list_head *p;
	jz_char_dev_t *new;
	list_for_each(p, &jz_char_devs) {
		jz_char_dev_t *dev = (jz_char_dev_t *)p;
		if (minor == dev->dev_minor)
			return -EBUSY;
	}
	new = (jz_char_dev_t *)kmalloc(sizeof(jz_char_dev_t), GFP_KERNEL);
	new->dev_minor = minor;
	new->name = (char *)name;
	new->fops = fops;
	new->private = private;
	list_add_tail((struct list_head *)new, &jz_char_devs);
	return 0;
}

int jz_unregister_chrdev(unsigned char minor, const char *name)
{
	struct list_head *p;
	jz_char_dev_t *dev = NULL;
	list_for_each(p, &jz_char_devs) {
		jz_char_dev_t *one = (jz_char_dev_t *)p;
		if (minor == one->dev_minor) {
			dev = one;
			break;
		}
	}
	if (dev == NULL)
		return -EINVAL;
	list_del((struct list_head *)dev);
	kfree(dev);
	return 0;
}

static ssize_t jz_char_read(struct file *, char *, size_t, loff_t *);
static ssize_t jz_char_write(struct file *, const char *, size_t, loff_t *);
static ssize_t jz_char_mmap(struct file *filp,  struct vm_area_struct *vma);
static int jz_char_open(struct inode *, struct file *);
static int jz_char_release(struct inode *, struct file *);
static int jz_char_ioctl(struct inode *, struct file *,
			   unsigned int, unsigned long);

static struct file_operations jz_char_fops =
{
	read:    jz_char_read,
	write:   jz_char_write,
	mmap:    jz_char_mmap,
	ioctl:   jz_char_ioctl,
	open:    jz_char_open,
	release: jz_char_release
};

static int __init jz_char_family_init(void)
{
	printk(JZ_SOC_NAME": Char device core registered.\n");
	return register_chrdev(JZ_CHAR_MAJOR, "JzChar", &jz_char_fops);
}

static void __exit jz_char_family_exit(void)
{
	printk(JZ_SOC_NAME": Char device core registered.\n");
	unregister_chrdev(JZ_CHAR_MAJOR, "JzChar");
}

module_init(jz_char_family_init);
module_exit(jz_char_family_exit);

static int jz_char_open(struct inode *inode, struct file *filp)
{
	jz_char_dev_t *dev = NULL;
	unsigned int minor = iminor(inode);       //minor extend to 20bit!
	struct list_head *p;
	list_for_each(p, &jz_char_devs) {
		jz_char_dev_t *one = (jz_char_dev_t *)p;
		if (one->dev_minor == minor) {
			dev = one;
			filp->private_data = dev;
			return dev->fops->open(inode, filp);
		}
	}
	printk("JzChar: No such device\n");
	return -EINVAL;
}

static int jz_char_release(struct inode *inode, struct file *filp)
{
	jz_char_dev_t *dev = (jz_char_dev_t *)filp->private_data;
	if (dev->fops->release)
		return dev->fops->release(inode, filp);
	return 0;
}

static int jz_char_ioctl(struct inode *inode, struct file *filp,
			  unsigned int cmd, unsigned long arg)
{
	jz_char_dev_t *dev = (jz_char_dev_t *)filp->private_data;
	if (dev->fops->ioctl)
		return dev->fops->ioctl(inode, filp, cmd, arg);
	return 0;
}

static ssize_t jz_char_read(struct file *filp, char *buf,
			      size_t count, loff_t *ppos)
{
	jz_char_dev_t *dev = (jz_char_dev_t *)filp->private_data;
	if (dev->fops->read)
		return dev->fops->read(filp, buf, count, ppos);
	return 0;
}

static ssize_t jz_char_write(struct file *filp, const char *buf,
			      size_t count, loff_t *ppos)
{
	jz_char_dev_t *dev = (jz_char_dev_t *)filp->private_data;
	if (dev->fops->write)
		return dev->fops->write(filp, buf, count, ppos);
	return 0;
}

static ssize_t jz_char_mmap(struct file *filp,  struct vm_area_struct *vma)
{
	jz_char_dev_t *dev = (jz_char_dev_t *)filp->private_data;
	if (dev->fops->mmap)
		return dev->fops->mmap(filp, vma);
	return 0;
}
EXPORT_SYMBOL(jz_register_chrdev);
EXPORT_SYMBOL(jz_unregister_chrdev);
