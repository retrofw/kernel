/*
 * linux/drivers/video/jz4810_lcd.c -- Ingenic Jz4810 LCD frame buffer device
 *
 * Copyright (C) 2005-2008, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * --------------------------------
 * NOTE:
 * This LCD driver support TFT16 TFT32 LCD, not support STN and Special TFT LCD
 * now.
 * 	It seems not necessory to support STN and Special TFT.
 * 	If it's necessary, update this driver in the future.
 * 	<Wolfgang Wang, Jun 10 2008>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/jzsoc.h>

#include "jz4810_aosd.h"

MODULE_DESCRIPTION("Jz4810 ALPHA OSD driver");
MODULE_AUTHOR("lltang, <lltang@ingenic.cn>");
MODULE_LICENSE("GPL");

#define D(fmt, args...) \
//	printk(KERN_ERR "%s(): "fmt"\n", __func__, ##args)

#define E(fmt, args...) \
	printk(KERN_ERR "%s(): "fmt"\n", __func__, ##args)

#define JZ_AOSD_DEBUG 1

#define AOSD_MAJOR 241
static int aosd_major = AOSD_MAJOR;
struct aosd_dev{
	struct cdev cdev;
	struct jz4810_aosd_info *aosd_info;
};

struct aosd_dev *aosd_devp;
int irq_flag = 0;
static int jz4810_alloc_aosd_info(void)
{
	aosd_devp->aosd_info = kmalloc(sizeof(struct jz4810_aosd_info) , GFP_KERNEL);

	if (!aosd_devp->aosd_info)
		return -1;

	memset(aosd_devp->aosd_info, 0, sizeof(struct jz4810_aosd_info) );
	return 0;
}

int aosd_open(struct inode *inode,struct file *filp)
{
//	filp->private_data = aosd_devp;
	return 0;
}

int aosd_release(struct inode *inode,struct file *filp)
{
	return 0;
}

/* The following routine is only for test */

#if JZ_AOSD_DEBUG
static void print_aosd_registers(void)	/* debug */
{
	/* LCD Controller Resgisters */
	printk("REG_AOSD_ADDR0:\t0x%08x\n", REG_AOSD_ADDR0);
	printk("REG_AOSD_ADDR1:\t0x%08x\n", REG_AOSD_ADDR1);
	printk("REG_AOSD_ADDR2:\t0x%08x\n", REG_AOSD_ADDR2);
	printk("REG_AOSD_ADDR3:\t0x%08x\n", REG_AOSD_ADDR3);
	printk("REG_AOSD_WADDR:\t0x%08x\n", REG_AOSD_WADDR);
	printk("REG_AOSD_ADDRLEN:\t0x%08x\n", REG_AOSD_ADDRLEN);
	printk("REG_AOSD_ALPHA_VALUE:\t0x%08x\n", REG_AOSD_ALPHA_VALUE);
	printk("REG_AOSD_CTRL:\t0x%08x\n",REG_AOSD_CTRL);
	printk("REG_AOSD_INT:\t0x%08x\n", REG_AOSD_INT);
	printk("REG_AOSD_CLK_GATE:\t0x%08x\n", REG_AOSD_CLK_GATE);	

	printk("REG_COMPRESS_OFFSIZE:\t0x%08x\n", REG_COMPRESS_OFFSIZE);
	printk("REG_COMPRESS_FRAME_SIZE:\t0x%08x\n", REG_COMPRESS_FRAME_SIZE);
	printk("REG_COMPRESS_CTRL:\t0x%08x\n", REG_COMPRESS_CTRL);
	printk("REG_COMPRESS_RATIO:\t0x%08x\n", REG_COMPRESS_RATIO);
	printk("REG_COMPRESS_OFFSET:\t0x%08x\n", REG_COMPRESS_OFFSET);
	printk("==================================\n");

}
#else
#define print_aosd_registers()
#endif

static unsigned char *addr0,*addr1,*addr2,*addr3;

static void jz4810_start_alpha_blending(void);
static void jz4810_aosd_set_mode(struct jz4810_aosd_info *jz4810_oa_info);
static void jz4810_compress_set_mode(struct jz4810_aosd_info *info);
static void jz4810_start_compress(void);

static int aosd_ioctl(struct inode *inodep, struct file *filep, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case ALPHA_OSD_PRINT:
		print_aosd_registers();
		break;


	case ALPHA_OSD_START:
		jz4810_start_alpha_blending();
		break;

	case ALPHA_OSD_GET_INFO:
		if (copy_to_user(argp, aosd_devp->aosd_info, sizeof(struct jz4810_aosd_info)))
			return -EFAULT;

		break;

	case ALPHA_OSD_SET_MODE:
		D("osd alpha set mode\n");

		if (copy_from_user(aosd_devp->aosd_info, argp, sizeof(struct jz4810_aosd_info)))
			return -EFAULT;

		jz4810_aosd_set_mode(aosd_devp->aosd_info);

		break;

	case COMPRESS_SET_MODE:
		D("compress set mode\n");

		if (copy_from_user(aosd_devp->aosd_info, argp, sizeof(struct jz4810_aosd_info)))
			return -EFAULT;

		jz4810_compress_set_mode(aosd_devp->aosd_info);

		break;

	case COMPRESS_START:
		jz4810_start_compress();
		break;


	default:
		printk("%s, unknown command(0x%x)", __FILE__, cmd);
		break;
	}

	return ret;
}

static int aosd_mmap(struct file *filep, struct vm_area_struct *vma)
{
	struct jz4810_aosd_info *info = aosd_devp->aosd_info;
	unsigned long start;
	unsigned long off;
	u32 len;

//	printk("%s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	off = vma->vm_pgoff << PAGE_SHIFT;
	//fb->fb_get_fix(&fix, PROC_CONSOLE(info), info);

	start = info->smem_start;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + info->addr_len);
	start &= PAGE_MASK;

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);	/* Uncacheable */

#if 1
 	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
 	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;		/* Uncacheable */
//	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;	/* Write-Back */
#endif

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}

/*
  Map 4 alpha osd memory ;added by lltang
*/
static int jz4810_map_osd_mem(struct jz4810_aosd_info *aosd_info)
{
	unsigned long page;
	unsigned int page_shift,needroom_osd, needroom;

	needroom = ((480*32) >> 3) * 272 ;
#if defined(CONFIG_JZ4810_AOSD)
	needroom_osd  = needroom * 4;
#else
	needroom_osd  = needroom * 2;
#endif
	page_shift = get_order(needroom_osd);
	printk("the PAGE_SIZE -> %x,the page_shift -> %d\n",PAGE_SIZE, page_shift);

	addr0 = (unsigned char *)__get_free_pages(GFP_KERNEL, page_shift);
	if(!addr0)
		return -ENOMEM;

	memset((void *)addr0, 0, PAGE_SIZE << page_shift); 
	addr1 = addr0 + needroom;

	REG_AOSD_ADDR0 = virt_to_phys((void *)addr0); 
	REG_AOSD_ADDR1 = virt_to_phys((void *)addr1); 
#if defined(CONFIG_JZ4810_AOSD)
	addr2 = addr0 + needroom * 2;
	addr3 = addr0 + needroom * 3;

	REG_AOSD_ADDR2 = virt_to_phys((void *)addr2); 
	REG_AOSD_ADDR3 = virt_to_phys((void *)addr3); 
#endif

	for (page = (unsigned long)addr0;
	     page < PAGE_ALIGN((unsigned long)addr0 + (PAGE_SIZE<<page_shift));
	     page += PAGE_SIZE) {
		SetPageReserved(virt_to_page((void*)page));
	}

	aosd_info->addr0 = virt_to_phys((void *)addr0);
	aosd_info->smem_start = virt_to_phys((void *)addr0);
//	aosd_info->addr_len = (PAGE_SIZE << page_shift); /* page_shift/2 ??? */
	aosd_info->addr_len = needroom_osd; /* page_shift/2 ??? */
	aosd_info->addr0_base =
		(unsigned char *)(((unsigned int)addr0&0x1fffffff) | 0xa0000000);

	if (!aosd_info->addr0_base) {
		printk("%s: unable to map aosd memory\n",__func__);
		return -ENOMEM;
	}

	printk("addr0 = %p addr1 = %p addr2 = %p, addr3 = %p\n", addr0, addr1, addr2, addr3);

	return 0;
}


static void jz4810_unmap_osd_mem(struct jz4810_aosd_info *aosd_info)
{

	struct page * map = NULL;
	unsigned char *tmp;
	unsigned int page_shift, needroom, needroom_osd;


	if (aosd_info && aosd_info->addr0_base) {
		iounmap(aosd_info->addr0_base);
		aosd_info->addr0_base = NULL;
		release_mem_region(aosd_info->addr0,aosd_info->addr_len);
	}

	needroom = ((480*32) >> 3) * 272; 
#if defined(CONFIG_JZ4810_AOSD)
	needroom_osd  = needroom * 4;
#else
	needroom_osd  = needroom*2 ;
#endif
	page_shift = get_order(needroom_osd);


	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom_osd)
			break;

	if (addr0) {
		for (tmp=(unsigned char *)addr0;
		     tmp < addr0 + (PAGE_SIZE << page_shift);
		     tmp += PAGE_SIZE) {
			map = virt_to_page(tmp);
			clear_bit(PG_reserved, &map->flags);
		}
		free_pages((int)addr0, page_shift);
	}

}

static void jz4810_start_alpha_blending(void)
{
//	print_aosd_registers();
	while(!(REG_AOSD_CTRL & AOSD_CTRL_FRM_END)) ;
	printk("all frames' alpha blending is finished\n");
	__osd_alpha_start();
}

static void jz4810_start_compress(void)
{
#if 0
	int i;
	int *test=(int*)addr0;
	for(i=0;i<480*32/32*272;i++){
		if(i%10 == 0)
			printk("\n");
		printk("%x\t",*(test+i));
	}
#endif

//	print_aosd_registers();

	while(!(REG_COMPRESS_CTRL & COMPRESS_CTRL_COMP_END));
	aosd_devp->aosd_info->compress_done = 0;
	printk("frame compress is finished and send to frame buffer\n");
	__compress_start();
}

static void jz4810_compress_set_mode(struct jz4810_aosd_info *info)
{
	int n;

/*SET SCR AND DES ADDR*/
	REG_AOSD_ADDR0 = info->addr0;//virt_to_phys((void *)addr0); //
        REG_COMPRESS_DES_ADDR = info->waddr;

/*SET DES OFFSIZE */
	REG_COMPRESS_OFFSIZE = (info->width+1) * 4; //des byte

/*SET SCR OFFSET*/
	REG_COMPRESS_OFFSET = info->width * 4; //scr byte

/*SET SIZE*/
	REG_COMPRESS_FRAME_SIZE = (info->width & 0xffff) | ((info->height & 0xffff) << 16);

/* SET CTRL*/
	if(info->without_alpha)
		REG_COMPRESS_CTRL = (COMPRESS_CTRL_WITHOUT_ALPHA | COMPRESS_CTRL_INT_MASK | COMPRESS_CTRL_COMP_ENABLE); 
	else
		REG_COMPRESS_CTRL = (COMPRESS_CTRL_WITH_ALPHA | COMPRESS_CTRL_INT_MASK | COMPRESS_CTRL_COMP_ENABLE); 		
}

static void jz4810_aosd_set_mode(struct jz4810_aosd_info *info)
{
#if 1
	printk("%s,info->waddr \t%x,info->width %d\t,info->height %d\t,info->alpha_value %x\t,\n",__func__,info->waddr,info->width,info->height,info->alpha_value);

#endif

	REG_AOSD_WADDR = info->waddr;
	REG_AOSD_ADDRLEN = info->width * info->height; 
	REG_AOSD_ALPHA_VALUE = info->alpha_value;

	printk("REG_AOSD_WADDR: %08x\n",REG_AOSD_WADDR);
	if(info->frmlv == 4){
		printk("info->frmlv == 4\n");
		REG_AOSD_CTRL = (AOSD_ALPHA_ENABLE | AOSD_CTRL_INT_MAKS |  AOSD_CTRL_FRMLV_4);
	}else if(info->frmlv == 3){
		printk("info->frmlv == 3\n");
		REG_AOSD_CTRL = (AOSD_ALPHA_ENABLE | AOSD_CTRL_INT_MAKS |  AOSD_CTRL_FRMLV_3);
	}else if(info->frmlv == 2){
		printk("info->frmlv == 2\n");
		REG_AOSD_CTRL = (AOSD_ALPHA_ENABLE | AOSD_CTRL_INT_MAKS |  AOSD_CTRL_FRMLV_2);
	}else{
		printk("frmlv default value is 0b01,now the frmlv < 2\n");
		return;
	}

	REG_AOSD_CTRL &= ~AOSD_CTRL_CHANNEL_LEVEL_MASK;
	REG_AOSD_CTRL |= (info->order << AOSD_CTRL_CHANNEL_LEVEL_BIT);

	REG_AOSD_CTRL &= ~AOSD_CTRL_ALPHA_MODE_MASK;
	REG_AOSD_CTRL |= (info->alpha_mode << AOSD_CTRL_ALPHA_MODE_BIT) ;

	REG_AOSD_CTRL &= ~AOSD_CTRL_FORMAT_MODE_MASK;
	if(info->bpp == 15)
		REG_AOSD_CTRL |= AOSD_CTRL_RGB555_FORMAT_MODE;
	else if(info->bpp == 16)
		REG_AOSD_CTRL |= AOSD_CTRL_RGB565_FORMAT_MODE;
	else if(info->bpp == 24)
		REG_AOSD_CTRL |= AOSD_CTRL_RGB8888_FORMAT_MODE;
	else
		printk("Sorry , we only support RGB555 RGB565 RGB8888!!!!!!\n");

}

static irqreturn_t jz4810_interrupt_handler(int irq, void *dev_id)
{
	unsigned int state;
	int cnt = 0;
	
	state = REG_AOSD_INT;
	D("In the lcd interrupt handler, state=0x%x\n", state);

	if (state & AOSD_INT_COMPRESS_END){
		printk("state & AOSD_INT_COMPRESS_END->OK!!!!!!!!!!!!!!!!!!!!\n");
//		print_aosd_registers();
		aosd_devp->aosd_info->compress_done = 1;
		REG_AOSD_ADDR0 = virt_to_phys((void *)addr0);; 
		REG_AOSD_INT = state & AOSD_INT_COMPRESS_END;
	}

	if (state & AOSD_INT_AOSD_END) {
		printk("state & AOSD_INT_AOSD_END\n");

		REG_AOSD_INT = state & AOSD_INT_AOSD_END;
		REG_AOSD_ADDR0 = virt_to_phys((void *)
addr0); 
		REG_AOSD_ADDR1 = virt_to_phys((void *)addr1); 
		REG_AOSD_ADDR2 = virt_to_phys((void *)addr2); 
		REG_AOSD_ADDR3 = virt_to_phys((void *)addr3); 
		aosd_devp->aosd_info->compress_done = 1;
	}

	return IRQ_HANDLED;
}

static const struct file_operations aosd_fops = {
	.owner           =  THIS_MODULE,
	.open            =  aosd_open,
	.release         =  aosd_release,
	.ioctl           =  aosd_ioctl,
	.mmap            =  aosd_mmap,
};

static void aosd_setup_cdev(struct aosd_dev *dev,int index)
{
	int err,devno = MKDEV(aosd_major,index);

	cdev_init(&dev->cdev, &aosd_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &aosd_fops;
	err = cdev_add(&dev->cdev,devno,1);
	if(err)
		printk(KERN_NOTICE "Error %d adding %d",err,index);
}

#ifdef CONFIG_PM
/*
 * Suspend the AOSD.
 */
static int jz4810_aosd_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

/*
 * Resume the AOSD.
 */
static int jz4810_aosd_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define jz4810_aosd_suspend      NULL
#define jz4810_aosd_resume       NULL
#endif /* CONFIG_PM */

static int __devinit jz4810_aosd_probe(struct platform_device *dev)
{
	struct jz4810_aosd_info *aosd_info;
	struct resource *r;
	int ret,irq;
	dev_t devno = MKDEV(aosd_major,0);

#if defined(CONFIG_JZ4810_AOSD) && defined(CONFIG_JZ4810_COMPRESS)
	printk("you can not use aosd and compress at the  same time!!!!\n");
	return -1;
#endif
	if(devno){
		ret = register_chrdev_region(devno,1,"aosd");
	}else{
		ret = alloc_chrdev_region(&devno,0 ,1,"aosd");
		aosd_major = MAJOR(devno);
	}
	if(ret < 0){
		printk("register_chrdev_region failed\n");
		return ret;
	}

	aosd_devp = kmalloc(sizeof(struct aosd_dev),GFP_KERNEL);
	if(!aosd_devp){
		printk("kmalloc aosd_dev failed\n");
		ret = -ENOMEM;
		goto failed1;
	}

	memset(aosd_devp,0,sizeof(struct aosd_dev));
	aosd_setup_cdev(aosd_devp,0);

	ret = jz4810_alloc_aosd_info();
	if(-1 == ret)
	{
		printk("alloc aosd info failed\n");
		ret = -ENOMEM;
		goto failed2;
	}

	r = platform_get_resource(dev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(dev, 0);
	if (!r || irq < 0){
		printk("platform get resource or irq  failed\n");
		ret = -ENXIO;
		goto failed4;
	}

	r = request_mem_region(r->start, r->end - r->start + 1, "oasd_compress");
	if (!r){
		printk("request_mem_region failed\n");
		ret = -EBUSY;
		goto failed4;
	}

	/* aosd and compress use the same IRQ number */
	if (request_irq(IRQ_AOSD, jz4810_interrupt_handler, IRQF_DISABLED,"oasd_compress", 0)) {
		printk("Faield to request ALPHA OSD IRQ.\n");
		ret = -EBUSY;
		goto failed4;
	}
#if defined(CONFIG_JZ4810_COMPRESS)
	REG_COMPRESS_CTRL = (COMPRESS_CTRL_INT_MASK | COMPRESS_CTRL_COMP_ENABLE); /* enable compress and enable finished interrupt */
#elif defined(CONFIG_JZ4810_AOSD)
	REG_AOSD_CTRL = (AOSD_ALPHA_ENABLE | AOSD_CTRL_INT_MAKS);
#endif
	REG_AOSD_CLK_GATE = AOSD_CLK_GATE_EN;
	ret = jz4810_map_osd_mem(aosd_devp->aosd_info);
	if (ret)
		goto failed5;
	printk("jz-aosd install!!! ######################################################\n");
	return 0;

failed5:
	free_irq(IRQ_AOSD,0);
failed4:

failed3:
	kfree(aosd_devp->aosd_info);
failed2:
	kfree(aosd_devp);
	cdev_del(&aosd_devp->cdev);
failed1:
	unregister_chrdev_region(devno,1);

	return ret;
}

static int __devexit jz4810_aosd_remove(struct platform_device *pdev)
{
	cdev_del(&aosd_devp->cdev);
	kfree(aosd_devp);
	kfree(aosd_devp->aosd_info);
	unregister_chrdev_region(MKDEV(aosd_major,0),1);
	return 0;
}

static struct platform_driver jz4810_aosd_driver = {
	.probe	= jz4810_aosd_probe,
	.remove = jz4810_aosd_remove,
	.suspend = jz4810_aosd_suspend,
	.resume = jz4810_aosd_resume,
	.driver = {
		.name = "jz-aosd",
		.owner = THIS_MODULE,
	},
};

static int __init jz4810_aosd_init(void)
{
	return platform_driver_register(&jz4810_aosd_driver);
}

static void __exit jz4810_aosd_cleanup(void)
{
	platform_driver_unregister(&jz4810_aosd_driver);
}

module_init(jz4810_aosd_init);
module_exit(jz4810_aosd_cleanup);
