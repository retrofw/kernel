/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * Control Driver.
 * 
 * Copyright (C) 2005-2010 Ingenic Semiconductor Inc.
 * Author: River Wang <zwang@ingenic.cn>
 *		
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

#define CTRL_DRV_POSTFIX "-ctrl"

static inline struct jz_fb_ot_info *minor_to_ot(int minor)
{
	struct jz_fb_ot_info *ot;

	unsigned int i;

	D("Called.");

	for (i = 0; i < sizeof(jz_fb_ots) / sizeof(struct jz_fb_ot_info *); i++) {
		ot = jz_fb_ots[i];
		if (ot->miscdev->minor == minor) {
			D("Output: %s is located.", ot->name);
			return ot;
		}
	}

	return NULL;
}

static int ctrl_drv_ioctl(struct inode *inode, 
		struct file *file, unsigned int cmd, unsigned long arg)
{
	struct jz_fb_ot_info *ot;

	int rv;

	D("Called.");	

	ot = minor_to_ot(iminor(inode));
	if (!ot) {
		E("Miscdevice (minor: %d) not found.", iminor(inode));
		return -ENODEV;
	}
	
	mutex_lock(&ot->ctrl_drv_lock);

	/* Dispatch control to specific output. */
	rv = ot->ops->control(ot, (void __user *)arg);

	mutex_unlock(&ot->ctrl_drv_lock);

	return rv;
}

static const struct file_operations ctrl_drv_misc_fops = {
	.ioctl = ctrl_drv_ioctl,
};

static int ctrl_drv_setup(struct jz_fb_ot_info *ot)
{
	struct miscdevice *dev;
	
	void *mem;
	
	size_t size;

	int rv;
	
	mutex_init(&ot->ctrl_drv_lock);

	size = sizeof(struct miscdevice) + sizeof(ot->name) + sizeof(CTRL_DRV_POSTFIX);

	mem = kzalloc(size, GFP_KERNEL);
	if (!mem) {
		return -ENOMEM;
	}
	
	dev = mem;
	dev->name = mem + sizeof(struct miscdevice);
	
	/* Control Path Name. */
	strcpy((char *)dev->name, ot->name);
	strcat((char *)dev->name, CTRL_DRV_POSTFIX);

	dev->minor = MISC_DYNAMIC_MINOR;
	dev->fops = &ctrl_drv_misc_fops;
	
	rv = misc_register(dev);
	if (rv) {
		kfree(dev);
		return rv;
	}

	ot->miscdev = dev;

	I("Create Control Path: %s, minor: %d.", dev->name, dev->minor);

	return 0;	
}

static int ctrl_drv_cleanup(struct jz_fb_ot_info *ot)
{
	misc_deregister(ot->miscdev);
	kfree(ot->miscdev);

	return 0;
}
