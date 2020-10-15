/*
 * jz_tssi.c
 *
 * MPEG2-TS interface driver for the Ingenic JZ47XX.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/jzsoc.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>


#include "jzchars.h"
#include "jz4770_tssi.h"


MODULE_AUTHOR("Lucifer Liu <yliu@ingenic.cn>");
MODULE_DESCRIPTION("Ingenic MPEG2-TS interface Driver");
MODULE_LICENSE("GPL");

#define TSSI_NAME "tssi"
#define TSSI_MINOR 204         /* MAJOR: 10, MINOR: 16 */
#define TSSI_IRQ   IRQ_TSSI
#define PFX        TSSI_NAME

#define DEBUG 0

#ifdef JZ_TSSISI_DEBUG
#define dbg(format, arg...) printk(KERN_DEBUG PFX ": " format "\n" , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif
#define err(format, arg...) printk(KERN_ERR PFX ": " format "\n" , ## arg)
#define info(format, arg...) printk(KERN_INFO PFX ": " format "\n" , ## arg)
#define warn(format, arg...) printk(KERN_WARNING PFX ": " format "\n" , ## arg)


static int buf_cnt = 0;
static struct jz_tssi_desc_t *tssi_desc; 
static struct jz_tssi_t *jz_tssi_g;

static void dump_tssi_regs( void )
{
        printk("REG_TSSI_ENA   %8x \n", REG_TSSI_ENA);
        printk("REG_TSSI_NUM   %8x \n", REG_TSSI_NUM);
        printk("REG_TSSI_DTR   %8x \n", REG_TSSI_DTR);
        printk("REG_TSSI_CFG   %8x \n", REG_TSSI_CFG);
        printk("REG_TSSI_CTRL  %8x \n", REG_TSSI_CTRL);
        printk("REG_TSSI_STAT  %8x \n", REG_TSSI_STAT);
        printk("REG_TSSI_FIFO  %8x \n", REG_TSSI_FIFO);
        printk("REG_TSSI_PEN   %8x \n", REG_TSSI_PEN);
        printk("REG_TSSI_PID0  %8x \n", REG_TSSI_PID0);
        printk("REG_TSSI_PID1  %8x \n", REG_TSSI_PID1);
        printk("REG_TSSI_PID2  %8x \n", REG_TSSI_PID2);
        printk("REG_TSSI_PID3  %8x \n", REG_TSSI_PID3);
        printk("REG_TSSI_PID4  %8x \n", REG_TSSI_PID4);
        printk("REG_TSSI_PID5  %8x \n", REG_TSSI_PID5);
        printk("REG_TSSI_PID6  %8x \n", REG_TSSI_PID6);
        printk("REG_TSSI_PID7  %8x \n", REG_TSSI_PID7);
}


static void tssi_config_filting( void )
{
	__gpio_as_tssi();
	__tssi_disable_ctrl_irq();
	__tssi_dma_enable();
	__tssi_set_tigger_num(96);        //trig is 4 word!
	__tssi_filter_disable_pid0();
//	__tssi_filter_enable();
	__tssi_filter_disable();
	__tssi_set_wd_1();
	__tssi_set_data_pola_high();
	__tssi_select_paral_mode();
	__tssi_select_clk_fast();
//	__tssi_select_clk_slow(); 
	REG_TSSI_CTRL = 7;

#if 1
/* no add data 0 */
	REG_TSSI_CFG &= ~(1 << 10);
	REG_TSSI_CFG |= (2 << 10);
#endif
	__tssi_select_clk_posi_edge();
	__tssi_select_frm_act_high();
	__tssi_select_str_act_high();
	__tssi_select_fail_act_high();
//	__tssi_select_fail_act_low();
}

static void tssi_add_pid(int pid_num, int pid)
{
	unsigned int addr ;
	int n = pid_num / 2, hl = pid_num % 2;
	if ( hl )      //use high pid, pid1
	{ 
		addr = TSSI_PID0 + ( n * 4 );
		REG32( addr ) |= ( (pid & 0x1fff) << 16 );    //13bit
		REG_TSSI_PEN |= ( 1 << (16 + n) );
	}
	else           //use low  pid, pid0
	{
		addr = TSSI_PID0 + ( n * 4 );
		REG32( addr ) |= pid & 0x1fff;    //13bit
		REG_TSSI_PEN |= ( 1 << n  );
	}
}

static unsigned int tssi_get_offset(void)
{
	struct jz_tssi_t *tssi = jz_tssi_g;
	unsigned int offset = tssi->offset;
	spinlock_t lock = tssi->lock;
	unsigned long flags;
	unsigned int tmp_addr;
	unsigned int addr;


	wait_event_interruptible(tssi->wait, tssi->num == 1);

	spin_lock_irqsave(&lock, flags);
	tmp_addr = tssi_desc[tssi->did].dst_addr;
	tssi_desc[tssi->did].dst_addr = tssi_desc[SWAP_BUF+buf_cnt].dst_addr;
	tssi_desc[SWAP_BUF+buf_cnt].dst_addr = tmp_addr;
	dma_cache_wback((unsigned long)(&tssi_desc[tssi->did]), sizeof(struct jz_tssi_desc_t));

	tssi->num -= 1;
	spin_unlock_irqrestore(&lock, flags);

	addr = (unsigned int)(tssi_desc[SWAP_BUF+buf_cnt].dst_addr);

	buf_cnt++;
	if (buf_cnt >= GET_BUF)
		buf_cnt = 0;

	return addr;
}

static void jz_tssi_free(void)
{
	struct jz_tssi_t *tssi = jz_tssi_g;
	
        if(tssi->mem_base != 0) {
                int page_order;

                if(tssi->mem_size == 0) {
                        printk("Original memory is NULL\n");
                        return;
                }
                page_order = get_order(tssi->mem_size);
                free_pages((unsigned long)tssi->mem_base, page_order);
                tssi->mem_base = NULL;
                printk("tssi_fb_destory!\n");
        }
}



static int jz_open(struct inode * inode, struct file * filp)
{
	try_module_get(THIS_MODULE);

	__intc_mask_irq(TSSI_IRQ);
	tssi_config_filting();
	__tssi_soft_reset();
	__tssi_clear_state();

	return 0;
}

static int jz_release(struct inode * inode, struct file * filp)
{

	jz_tssi_free();
	__intc_mask_irq(TSSI_IRQ);
	module_put(THIS_MODULE);
	return 0;
}

static int jz_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	jz_char_dev_t *adev = (jz_char_dev_t *)file->private_data;
	struct jz_tssi_t* tssi = (struct jz_tssi_t*)adev->private;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case IOCTL_TSSI_ENABLE :
		__tssi_disable();
		__tssi_soft_reset();
		__tssi_clear_state();
//		dump_tssi_regs();
		__intc_ack_irq(TSSI_IRQ);
		__intc_unmask_irq(TSSI_IRQ);
		tssi->num = 0;
		__tssi_enable();

        	break;
	case IOCTL_TSSI_DISABLE :
		__tssi_disable();
		__tssi_soft_reset();
		__tssi_clear_state();

        	break;
	case IOCTL_TSSI_SOFTRESET :
		__tssi_soft_reset();

        	break;
	case IOCTL_TSSI_ENFILTER :
		__tssi_filter_enable();
        	break;
	case IOCTL_TSSI_DEFILTER :
		__tssi_filter_disable();
        	break;
	case IOCTL_TSSI_ADDPID :           //add one pid to filter
		if ( tssi->pid_num < 31 )
		{
			tssi_add_pid(tssi->pid_num, arg);
			tssi->pid_num ++ ;
		}
		break;

	case IOCTL_TSSI_FLUSHPID :               //set all filting pid to false
		REG_TSSI_PEN = 0x0;		
		REG_TSSI_PID0 = 0x0;
		REG_TSSI_PID1 = 0x0;
		REG_TSSI_PID2 = 0x0;
		REG_TSSI_PID3 = 0x0;
		REG_TSSI_PID4 = 0x0;
		REG_TSSI_PID5 = 0x0;
		REG_TSSI_PID6 = 0x0;
		REG_TSSI_PID7 = 0x0;
		break;

	case IOCTL_TSSI_INIT_DMA:
		break;
	case IOCTL_TSSI_DISABLE_DMA:
		break;
	case IOCTL_TSSI_GET_OFFSET:
		{
			unsigned int offset;

			offset = tssi_get_offset();
			offset -= (unsigned long) virt_to_phys(tssi->mem_base);

			return copy_to_user(argp, &offset, sizeof(offset)) ? -EFAULT : 0;
		}
	}

	return 0;
}

static int jz_tssi_alloc(struct jz_tssi_t *tssi)
{
	struct jz_tssi_t *tmp_tssi = tssi;
	int order;

	if (tmp_tssi->mem_base) {
		printk("mem_base not zero.\n");
		return 1;
	}

	order = get_order(tmp_tssi->mem_size);
	printk("order = %d\n", order);

	tmp_tssi->mem_base = (unsigned char *)__get_free_pages(GFP_KERNEL, order);

	if (tmp_tssi->mem_base == NULL) {
		printk("No mem: Alloc memory for tssi DMA failed.\n");
		return -ENOMEM;
	}

	return 0;
	
}

//Use mmap /dev/tssi can only get a non-cacheable Virtual Address.
static int jz_mmap(struct file *file, struct vm_area_struct *vma)
{
        unsigned long start;
        unsigned long off;
        u32 len;
	struct jz_tssi_t *tssi = jz_tssi_g;

        if( tssi->mem_base == 0 ) {
                if (jz_tssi_alloc(tssi)) {
                        printk("No mem: Alloc memory for tssi DMA\n");
                        return -ENOMEM;
                }
        }

        off = vma->vm_pgoff << PAGE_SHIFT;

        // frame buffer memory
        start = virt_to_phys(tssi->mem_base);
        len = PAGE_ALIGN((start & ~PAGE_MASK) + ((unsigned long)tssi->mem_size));

        start &= PAGE_MASK;

        if ((vma->vm_end - vma->vm_start + off) > len) {
                printk("Error: vma is larger than memory length\n");
                return -EINVAL;
        }
        off += start;

        vma->vm_pgoff = off >> PAGE_SHIFT;
        vma->vm_flags |= VM_IO;

        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);        // Uncacheable
//#if  defined(CONFIG_MIPS32)
#if 1
        pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
        pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;               /* Uncacheable */
#endif

	if (remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot)) {
		return -EAGAIN;
	}

        return 0;
}

static struct file_operations tssi_fops = {
	.owner	= THIS_MODULE,
	.poll	= NULL,
	.fasync	= NULL,
	.ioctl	= jz_ioctl,
	.open	= jz_open,
	.mmap	= jz_mmap,
	.release= jz_release,
};

static irqreturn_t tssi_interrupt(int irq, void * dev_id)
{
//	unsigned char *tmp;
	int num = REG_TSSI_NUM;
	int did = (REG_TSSI_DST & TSSI_DST_DID_MASK) >> TSSI_DST_DID_BIT;
	int j;
	

	struct jz_tssi_t *tssi = (struct jz_tssi_t *)dev_id;
	tssi->did = did;
	tssi->num++;
//	tmp = tssi->mem_base + did * MPEG2_PACKET_SIZE;

	__intc_mask_irq(TSSI_IRQ);

	__tssi_clear_desc_end_flag();

	if (REG_TSSI_STAT & TSSI_STAT_OVRN) {
		printk("tssi over run occur! %x, num = %d\n",REG8( TSSI_STAT ), num);
		__tssi_clear_state();
	}

#if 0
	dma_cache_inv(tssi->mem_base + did * MPEG2_PACKET_SIZE, MPEG2_PACKET_SIZE);
	for (j = 0; j < MPEG2_PACKET_SIZE; j++)
		if (j % 188 == 0 && tmp[j] != 0x47)
			printk("irq buf[%d] = %02x\n", j, tmp[j]);
#endif
	

	printk("num = %d, did = %d\n", tssi->num, did);

	if (tssi->num == 1)
		wake_up(&tssi->wait);

/* used for test */
#if DEBUG
/* it will be over run the buf */
	if (cur_buf->fu_num == 5) {
		__tssi_dma_enable();
		__tssi_disable();
		__intc_ack_irq(TSSI_IRQ);
		__intc_unmask_irq(TSSI_IRQ);
		return IRQ_HANDLED;
	}
#endif

	__intc_ack_irq(TSSI_IRQ);
	__intc_unmask_irq(TSSI_IRQ);

	return IRQ_HANDLED;
}



static void tssi_dma_desc_init(void)
{
	struct jz_tssi_t *tssi = jz_tssi_g;
	unsigned int base_addr;
	int i;

	tssi_desc = __get_free_pages(GFP_KERNEL, 0);
	if (!tssi_desc) {
		printk("tssi_desc malloc memory failed.\n");
		return -ENOMEM;
	}

	for (i = 0; i < NR_BUF; i++) {
		base_addr = virt_to_phys(tssi->mem_base) + i * MPEG2_PACKET_SIZE;
		tssi_desc[i].next_desc = (unsigned int)virt_to_phys((void*)&tssi_desc[i+1]);
		tssi_desc[i].dst_addr = base_addr;
		tssi_desc[i].did = i;
		tssi_desc[i].cmd = ((MPEG2_PACKET_SIZE/4) << TSSI_DCMD_TLEN_BIT) | TSSI_DCMD_TEFE |
					 TSSI_DCMD_TSZ_32 | TSSI_DCMD_TEIE | TSSI_DCMD_LINK;
	}

	tssi_desc[SWAP_BUF-1].next_desc = (unsigned int)virt_to_phys(&tssi_desc[0]);
	
	for(i = 0; i < GET_BUF; i++)
		tssi_desc[SWAP_BUF+i].next_desc  = virt_to_phys(NULL);

	for (i = 0; i < SWAP_BUF; i++) {
		dma_cache_wback((unsigned long)(&tssi_desc[i]), sizeof(struct jz_tssi_desc_t));
	}

	REG_TSSI_DDA = (unsigned int)virt_to_phys((void*)&tssi_desc[0]);
	
}

static int __init jztssi_init_module(void)
{
	int retval;
	struct jz_tssi_t *tssi;

	tssi  = kmalloc(sizeof(struct jz_tssi_t), GFP_KERNEL);
	if (!tssi) {
		printk("TSSI malloc memory failed.\n");
		return -ENOMEM;
	}

	jz_tssi_g = tssi;

	cpm_start_clock(CGM_TSSI);

	tssi->mem_base = NULL;
	tssi->mem_size = MPEG2_PACKET_SIZE * 5;
	tssi->offset = 0;
	tssi->num = 0;

	jz_tssi_alloc(tssi);
	tssi_dma_desc_init();

	spin_lock_init(&tssi->lock);
	init_waitqueue_head(&tssi->wait);

	retval = request_irq(TSSI_IRQ, tssi_interrupt, IRQF_DISABLED, TSSI_NAME, (void*)jz_tssi_g);
	if (retval) {
		printk("unable to get IRQ %d",TSSI_IRQ);
		return retval;
	}

	jz_register_chrdev(TSSI_MINOR, TSSI_NAME, &tssi_fops, (void *)jz_tssi_g);	

	printk(JZ_SOC_NAME":MPEG2-TS interface driver registered %x\n",(unsigned int)jz_tssi_g);
	return 0;
}

static void jztssi_cleanup_module(void)
{
	free_irq(TSSI_IRQ,0);
	jz_unregister_chrdev(TSSI_MINOR, TSSI_NAME);
}

module_init(jztssi_init_module);
module_exit(jztssi_cleanup_module);
