/*
 * linux/drivers/misc/jz_cim.c -- Ingenic CIM driver
 *
 * Copyright (C) 2005-2010, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/time.h>

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/major.h>
#include <linux/version.h>


#include <asm/cacheflush.h>
#include <asm/mipsregs.h>
#include <asm/mipsmtregs.h>
#include <asm/pgtable.h>
#include <asm/irq.h>
#include <asm/thread_info.h>
#include <asm/uaccess.h>
#include <asm/jzsoc.h>

#include "jz_cim_core.h"
#include "jz_sensor.h"

MODULE_AUTHOR("Lutts Cao<slcao@ingenic.cn>");
MODULE_DESCRIPTION("Ingenic Camera interface driver");
MODULE_LICENSE("GPL");

#define CIM_MINOR             234

//#define CONFIG_SOC_JZ4760B y
#ifdef CONFIG_JZ4810_F4810
#define CONFIG_SOC_JZ4760B y
#endif

//#define CIM_INTR_SOF_EN
#define CIM_INTR_EOF_EN
//#define CIM_INTR_EEOF_EN
//#define CIM_INTR_STOP_EN
//#define CIM_INTR_TRIG_EN
#define CIM_INTR_OF_EN

//#define CIM_SAFE_DISABLE

#if defined(CONFIG_SOC_JZ4750)
#undef CIM_INTR_EEOF_EN
#endif

#define CIM_DEBUG
#define CIM_I_DEBUG

#undef CIM_I_DEBUG
#undef CIM_DEBUG

#ifdef CIM_DEBUG
#define dprintk(x...)	do{printk("CIM---\t");printk(x);}while(0)
#else
#define dprintk(x...)
#endif

#ifdef CIM_I_DEBUG
#define iprintk(x...)	do{printk(x);}while(0)
#else
#define iprintk(x...)
#endif

#define CIM_NAME        "cim"

#define GET_BUF 2
#define SWAP_BUF 3
#define SWAP_NR (GET_BUF+SWAP_BUF)

/*
 * CIM DMA descriptor
 */
enum {TRUE=1,FALSE=0};

struct cim_desc {
	u32 nextdesc;   // Physical address of next desc
#if defined(CONFIG_SOC_JZ4760B) || defined(CONFIG_SOC_JZ4770)
	u32 frameid;    // Frame ID
	u32 framebuf;   // Physical address of frame buffer, when SEP=1, it's y framebuffer
#else
	u32 framebuf;   // Physical address of frame buffer, when SEP=1, it's y framebuffer
	u32 frameid;    // Frame ID
#endif
	u32 dmacmd;     // DMA command, when SEP=1, it's y cmd
	/* only used when SEP = 1 */
	u32 cb_frame;
	u32 cb_len;
	u32 cr_frame;
	u32 cr_len;
};

/*
 * CIM device structure
 */

struct cim_device
{
	cim_config_t cim_cfg;

	struct camera_buffer_info preview_binfo;
	struct camera_buffer_info capture_binfo;

	unsigned char *mem_base;
	unsigned int mem_size;

	wait_queue_head_t capture_wait_queue;
	wait_queue_head_t preview_wait_queue;

	struct list_head sensor_lists;
	struct global_info ginfo;

	int cim_started;
	int cim_transfer_started;
	int preview_timeout_state;

	int sensor_inited;
};


static unsigned int buf_offsize_app_drv;
static struct cim_device *jz_cim;

static int snapshot_flag __read_mostly = 0;

static unsigned int dmacmd_intr_flag = 0
#ifdef CIM_INTR_SOF_EN
	| CIM_CMD_SOFINT
#endif
#ifdef CIM_INTR_EOF_EN
	| CIM_CMD_EOFINT
#endif
#ifdef CIM_INTR_EEOF_EN
	| CIM_CMD_EEOFINT
#endif
	;


//-------------------------------------------------------------------------------------------
static spinlock_t fresh_lock = SPIN_LOCK_UNLOCKED;
static int fresh_id  __read_mostly = -1;
static int fresh_buf __read_mostly;
static int wait_count __read_mostly;

static struct cim_desc frame_desc[SWAP_NR] __attribute__ ((aligned (sizeof(struct cim_desc))));
static struct cim_desc capture_desc __attribute__ ((aligned (sizeof(struct cim_desc))));

/*====================================================================================
 * sensor muti-support
 *===================================================================================*/

static LIST_HEAD(sensor_list);
static DEFINE_MUTEX(sensor_lock);
static struct camera_sensor_desc *cur_desc __read_mostly = NULL;


/*==========================================================================
 * File operations
 *========================================================================*/

static int cim_open(struct inode *inode, struct file *filp);
static int cim_release(struct inode *inode, struct file *filp);
static ssize_t cim_read(struct file *filp, char *buf, size_t size, loff_t *l);
static ssize_t cim_write(struct file *filp, const char *buf, size_t size, loff_t *l);
static int cim_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int cim_mmap(struct file *file, struct vm_area_struct *vma);

static void cim_fb_free(void);
static int cim_set_function(int function,void *cookie);

static int cim_set_preview_resolution(struct resolution_info param);

static int cim_set_capture_resoultion(struct resolution_info param,int state);
static int cim_set_output_format(pixel_format_flag_t flag,int arg);

int camera_sensor_register(struct camera_sensor_desc *desc)
{
	if(desc==NULL)
		return -EINVAL;

	desc->sensor_id = 0xffff;

	desc->max_capture_size=
		desc->max_capture_parm.width
	       	*desc->max_capture_parm.height
		* 3; /* in order to support seperate YCbCr */
		//*desc->max_capture_parm.bpp >> 3;

	desc->max_preview_size=
		desc->max_preview_parm.width
	       	*desc->max_preview_parm.height
		* 3; /* in order to support seperate YCbCr */
		//*desc->max_preview_parm.bpp >> 3;

	if (desc->bus_width < 8)
		desc->bus_width = 8;

	mutex_lock(&sensor_lock);
	list_add_tail(&desc->list,&sensor_list);
	mutex_unlock(&sensor_lock);

	desc->ops->sensor_set_power(1);
	return 0;
}

void cim_scan_sensor(void)
{
	struct camera_sensor_desc *si;
	struct list_head *tmp;

	dprintk("Enter %s\n", __FUNCTION__);

	mutex_lock(&sensor_lock);
	list_for_each_entry(si, &sensor_list, list)
	{
		printk("scan sensor:%s\n",si->name);

		if(si->ops->camera_sensor_probe() != 0)
		{
			tmp=si->list.prev;
			list_del(&si->list);
			si=list_entry(tmp,struct camera_sensor_desc,list);
		}
	}

	mutex_unlock(&sensor_lock);
	dprintk("Leave %s\n", __FUNCTION__);
}

void sensors_make_default(void)
{
	struct camera_sensor_desc *si;
	printk("now we has:\n");
	mutex_lock(&sensor_lock);

	cur_desc = NULL;
	list_for_each_entry(si, &sensor_list, list)
	{
		si->sensor_id = jz_cim->ginfo.sensor_count;
		jz_cim->ginfo.sensor_count++;
		printk("sensor_name:%s\t\tid:%d\n",si->name,si->sensor_id);
		cur_desc = si;
		//cur_desc->wait_frames = 1000;
	}

	printk("default is %s\n",cur_desc->name);
	mutex_unlock(&sensor_lock);
	jz_cim->ginfo.preview_buf_nr = SWAP_NR;
}
/*==========================================================================
 * CIM print operations
 *========================================================================*/
static void cim_print_regs(void)
{
	printk("REG_CIM_CFG \t= \t0x%08x\n", REG_CIM_CFG);
	printk("REG_CIM_CTRL \t= \t0x%08x\n", REG_CIM_CTRL);
#if defined(CONFIG_SOC_JZ4760B) || defined(CONFIG_SOC_JZ4770)
	printk("REG_CIM_CTRL2 \t= \t0x%08x\n", REG_CIM_CTRL2);
#endif
	printk("REG_CIM_STATE \t= \t0x%08x\n", REG_CIM_STATE);
	printk("REG_CIM_IID \t= \t0x%08x\n", REG_CIM_IID);
	printk("REG_CIM_DA \t= \t0x%08x\n", REG_CIM_DA);
	printk("REG_CIM_FA \t= \t0x%08x\n", REG_CIM_FA);
	printk("REG_CIM_FID \t= \t0x%08x\n", REG_CIM_FID);
	printk("REG_CIM_CMD \t= \t0x%08x\n", REG_CIM_CMD);
	printk("REG_CIM_SIZE \t= \t0x%08x\n", REG_CIM_SIZE);
	printk("REG_CIM_OFFSET \t= \t0x%08x\n", REG_CIM_OFFSET);
#if defined(CONFIG_SOC_JZ4760B) || defined(CONFIG_SOC_JZ4770)
	printk("REG_CIM_YFA \t= \t%#08x\n", REG_CIM_YFA);
	printk("REG_CIM_YCMD \t= \t%#08x\n", REG_CIM_YCMD);
	printk("REG_CIM_CBFA \t= \t%#08x\n", REG_CIM_CBFA);
	printk("REG_CIM_CBCMD \t= \t%#08x\n", REG_CIM_CBCMD);
	printk("REG_CIM_CRFA \t= \t%#08x\n", REG_CIM_CRFA);
	printk("REG_CIM_CRCMD \t= \t%#08x\n", REG_CIM_CRCMD);
#endif
}

#if 1
static void cim_print_buffers(void)
{
	int i;
	//printk("cim_tran_buf_id%x\n",cim_tran_buf_id);
	//printk("data_ready_buf_id=0x%x\n",data_ready_buf_id);

	for(i = 0; i < SWAP_NR; i++)
	{
		printk("=============%p=============\n", &frame_desc[i]);
		printk("cim_nextdesc_desc[%d]=0x%x\n",i,frame_desc[i].nextdesc);
		printk("cim_framebuf_desc[%d]=0x%x\n",i,frame_desc[i].framebuf);
		printk("cim_frameid _desc[%d]=0x%x\n",i,frame_desc[i].frameid);
		printk("cim_dmacmd  _desc[%d]=0x%x\n",i,frame_desc[i].dmacmd);
		//if (!jz_cim->cim_cfg.packed)
		{
			printk("cim_cb_frame_desc[%d]=0x%x\n",i,frame_desc[i].cb_frame);
			printk("cim_cb_len_desc[%d]=0x%x\n",i,frame_desc[i].cb_len);
			printk("cim_cr_frame_desc[%d]=0x%x\n",i,frame_desc[i].cr_frame);
			printk("cim_cr_len_desc[%d]=0x%x\n",i,frame_desc[i].cr_len);
		}
		printk("========================================\n");
	}

	//printk("preview_working_buf=0x%x\n",preview_working_buf);
	//printk("recorder_prepare_buf=0x%x\n",recorder_prepare_buf);
	//printk("recorder_working_buf=0x%x\n\n",recorder_working_buf);
}
#endif

/*==========================================================================
 * Framebuffer allocation and destroy
 *========================================================================*/

static int cim_init_capture_desc(void)
{
	int imgsize = cur_desc->capture_parm.width * cur_desc->capture_parm.height;
	int capture_frmsize = ((imgsize * cur_desc->capture_parm.bpp) >> 3);

	dprintk("cap_size=0x%x\n",capture_frmsize);

	capture_desc.nextdesc = virt_to_phys(&capture_desc);
	capture_desc.frameid  = 0xff;
	capture_desc.framebuf = jz_cim->capture_binfo.base_paddr;
	printk("=======>cap fb = %#010x\n", capture_desc.framebuf);
	//capture_desc.dmacmd   = (capture_frmsize>>2) | CIM_CMD_EOFINT | CIM_CMD_STOP;
	if (jz_cim->cim_cfg.packed)
		capture_desc.dmacmd   = (capture_frmsize>>2) | dmacmd_intr_flag;
	else {
		printk("=======>%s:%d we are test sep!!!\n", __FUNCTION__, __LINE__);
		capture_desc.dmacmd   = (imgsize>>2) | dmacmd_intr_flag;
		/* FIXME: test YCbCr444 or YCbCr422 bellow */
		if (jz_cim->cim_cfg.fmt_422) {
			capture_desc.cb_frame = jz_cim->capture_binfo.base_paddr + imgsize;
			capture_desc.cb_len = (imgsize / 2) >> 2;
			capture_desc.cr_frame = jz_cim->capture_binfo.base_paddr + imgsize + imgsize / 2;
			capture_desc.cb_len = (imgsize / 2) >> 2;
		} else {
			capture_desc.cb_frame = jz_cim->capture_binfo.base_paddr + imgsize;
			capture_desc.cb_len = imgsize >> 2;
			capture_desc.cr_frame = jz_cim->capture_binfo.base_paddr + imgsize + imgsize;
			capture_desc.cb_len = imgsize >> 2;
		}
	}
#if !defined(CONFIG_SOC_JZ4750)
	capture_desc.dmacmd  |= CIM_CMD_OFRCV ;
#endif
	if(cur_desc->wait_frames == 0)
		capture_desc.dmacmd  |= CIM_CMD_STOP;

	dma_cache_wback_inv((unsigned long)&capture_desc, sizeof(struct cim_desc));
	return 0;
}

static int cim_init_preview_desc(void)
{

	int i;
	int imgsize = cur_desc->preview_parm.width * cur_desc->preview_parm.height;
	int preview_frmsize = ((imgsize * cur_desc->preview_parm.bpp) >> 3);
	unsigned int base_paddr = 0;

	dprintk("=================preview: width = %d height = %d bpp = %d\n",
		cur_desc->preview_parm.width , cur_desc->preview_parm.height, cur_desc->preview_parm.bpp);
	dprintk("pre_size=0x%x",preview_frmsize);

//--------------------------------------------------------------------------------------------

	for (i= 0; i< SWAP_NR; i++)
	{
		base_paddr = jz_cim->preview_binfo.base_paddr + jz_cim->ginfo.max_preview_size * i;

		frame_desc[i].nextdesc  = virt_to_phys(&frame_desc[i+1]);
		frame_desc[i].frameid  = i;
		frame_desc[i].framebuf = base_paddr;
		if (jz_cim->cim_cfg.packed)
			frame_desc[i].dmacmd   = (preview_frmsize>>2) | dmacmd_intr_flag;
		else {
			printk("=======>%s:%d we are test sep!!!\n", __FUNCTION__, __LINE__);
			frame_desc[i].dmacmd   = (imgsize>>2) | dmacmd_intr_flag;

			/* FIXME: test YCbCr444 and YCbCr422 here */
			if (jz_cim->cim_cfg.fmt_422) {
				frame_desc[i].cb_frame = base_paddr + imgsize;
				frame_desc[i].cb_len = (imgsize / 2) >> 2;
				frame_desc[i].cr_frame = base_paddr + imgsize + imgsize / 2;
				frame_desc[i].cr_len = (imgsize / 2) >> 2;
			} else {
				printk(">>>>>>>>>>>>>>in %s, we are test YUV444, img_size = %d\n", __FUNCTION__, imgsize);
				frame_desc[i].cb_frame = base_paddr + imgsize;
				frame_desc[i].cb_len = imgsize >> 2;
				frame_desc[i].cr_frame = base_paddr + imgsize * 2;
				frame_desc[i].cr_len = imgsize >> 2;
			}
		}
#if !defined(CONFIG_SOC_JZ4750)
		frame_desc[i].dmacmd  |= CIM_CMD_OFRCV ;
#endif
	}

	frame_desc[SWAP_BUF-1].nextdesc  = virt_to_phys(&frame_desc[0]);

	for(i = 0; i < GET_BUF; i++)
	{
		frame_desc[SWAP_BUF+i].nextdesc  = virt_to_phys(NULL);
	}

	for (i = 0; i < SWAP_BUF; i++)
		dma_cache_wback((unsigned long)(&frame_desc[i]), sizeof(struct cim_desc));


//----------------------------------------------------------------------------------------------
	return 0;
}

static unsigned int cim_init_fb_info(int swap_nr)
{
	unsigned int max_p_size=0,max_c_size=0;
	struct camera_sensor_desc *si;

	mutex_lock(&sensor_lock);
	list_for_each_entry(si, &sensor_list, list)
	{
		if(si->max_preview_size > max_p_size)
			max_p_size = si->max_preview_size;
		if(si->max_capture_size > max_c_size)
			max_c_size = si->max_capture_size;
	}
	mutex_unlock(&sensor_lock);

	dprintk("get_max_mem_size\n");
	dprintk("preview size = %dKB\n",max_p_size>>10);
	dprintk("capture size = %dKB\n",max_c_size>>10);

	jz_cim->ginfo.max_preview_size = max_p_size;
	jz_cim->ginfo.max_capture_size = max_c_size;

	jz_cim->preview_binfo.buffer_size = max_p_size * swap_nr;
	jz_cim->capture_binfo.buffer_size = max_c_size;

	return max_p_size * swap_nr + max_c_size;
}


static int cim_fb_alloc(void)
{
	unsigned int mem_size = 0, preview_mem_size = 0, capture_mem_size = 0;
	int order = 0;

	if (jz_cim->mem_base == NULL )
	{
		dprintk("get new page!\n");

		(void)cim_init_fb_info(SWAP_NR);
		preview_mem_size = jz_cim->ginfo.max_preview_size * SWAP_NR;
		capture_mem_size = jz_cim->ginfo.max_capture_size;
		mem_size = (preview_mem_size > capture_mem_size) ? preview_mem_size : capture_mem_size;
		order = get_order(mem_size);

		dprintk("mem_size=%dK\n", mem_size>>10);
		dprintk("order=%d\n", order);

		jz_cim->mem_base = (unsigned char *)__get_free_pages(GFP_KERNEL, order);

		if (jz_cim->mem_base == NULL)
		{
			printk("Preview: GET FREE PAGES FAILED!\n");
			return -ENOMEM;
		}

		jz_cim->preview_binfo.base_vaddr = (unsigned int)jz_cim->mem_base;
		jz_cim->preview_binfo.base_paddr = virt_to_phys(jz_cim->mem_base);

		jz_cim->capture_binfo.base_vaddr = (unsigned int)jz_cim->mem_base;
		jz_cim->capture_binfo.base_paddr = virt_to_phys(jz_cim->mem_base);

		memset(jz_cim->mem_base, 0, mem_size);

		jz_cim->mem_size = mem_size;
		jz_cim->ginfo.mmap_size = mem_size;
	}

	return 0;
}

static void cim_fb_free(void)
{
	if(jz_cim->mem_base != 0)
	{
		int page_order;
		__cim_disable();

		if(jz_cim->mem_size == 0)
	       	{
			dprintk("Original memory is NULL\n");
			return;
		}
		page_order = get_order(jz_cim->mem_size);
		free_pages((unsigned long)jz_cim->mem_base, page_order);
		jz_cim->mem_base = NULL;
		dprintk("cim_fb_destory!\n");
	}
}

/*==========================================================================
 * CIM Module operations
 *========================================================================*/

static void cim_config(cim_config_t *c)
{

	REG_CIM_CFG  = c->cfg;
	REG_CIM_CTRL = c->ctrl;
	REG_CIM_SIZE = c->size;
	REG_CIM_OFFSET = c->offs;

#if defined(CONFIG_SOC_JZ4760B) || defined(CONFIG_SOC_JZ4770)
		__cim_input_data_format_select_YUV422();
		//__cim_input_data_format_select_YUV444();
		__cim_set_input_data_stream_order(0); /* YCbCr or Y0CbY1Cr */
		//__cim_set_input_data_stream_order(1); /* YCrCb or Y0CrY1Cb */
		//__cim_set_input_data_stream_order(2); /* CbCrY or CbY0CrY1 */
		//__cim_set_input_data_stream_order(3); /* CrCbY or CrY0CbY1 */

		//__cim_set_data_packing_mode(0); /* 0x11 22 33 44 or 0x Y0 Cb Y1 Cr */
		//__cim_set_data_packing_mode(1); /* 0x 22 33 44 11 or 0x Cb Y1 Cr Y0 */
		//__cim_set_data_packing_mode(2); /* 0x 33 44 11 22 or 0x Y1 Cr Y0 Cb */
		//__cim_set_data_packing_mode(3); /* 0x 44 11 22 33 or 0x Cr Y0 Cb Y1 */
		//__cim_set_data_packing_mode(4); /* 0x 44 33 22 11 or 0x Cr Y1 Cb Y0 */
		//__cim_set_data_packing_mode(5); /* 0x 33 22 11 44 or 0x Y1 Cb Y0 Cr */
		//__cim_set_data_packing_mode(6); /* 0x 22 11 44 33 or 0x Cb Y0 Cr Y1 */
		//__cim_set_data_packing_mode(7); /* 0x 11 44 33 22 or 0x Y0 Cr Y1 Cb */

		__cim_enable_bypass_func();
		//__cim_disable_bypass_func();

		__cim_enable_auto_priority();
		//__cim_disable_auto_priority();

		__cim_enable_emergency();
		//__cim_disable_emergency();

		/* 0, 1, 2, 3
		 * 0: highest priority
		 * 3: lowest priority
		 * 1 maybe best for SEP=1
		 * 3 maybe best for SEP=0
		 */
		//__cim_set_opg(0);
		__cim_set_opg(1);
		//__cim_set_opg(2);
		//__cim_set_opg(3);

		__cim_enable_priority_control();
		//__cim_disable_priority_control();


	if (c->packed) {
		__cim_disable_sep();
	} else {
		__cim_enable_sep();
	}
#endif

	if(cur_desc->no_dma) //just for fake sensor test
		return;

	__cim_enable_dma();

#ifdef CIM_SAFE_DISABLE
	//__cim_enable_vdd_intr();
#else
	__cim_disable_vdd_intr();
#endif

#ifdef CIM_INTR_SOF_EN
	__cim_enable_sof_intr();
#else
	__cim_disable_sof_intr();
#endif

#ifdef CIM_INTR_EOF_EN
	__cim_enable_eof_intr();
#else
	__cim_disable_eof_intr();
#endif

#ifdef CIM_INTR_STOP_EN
	__cim_enable_stop_intr();
#else
	__cim_disable_stop_intr();
#endif

#ifdef CIM_INTR_TRIG_EN
	__cim_enable_trig_intr();
#else
	__cim_disable_trig_intr();
#endif

#ifdef CIM_INTR_OF_EN
	__cim_enable_rxfifo_overflow_intr();
#else
	__cim_disable_rxfifo_overflow_intr();
#endif

#ifdef CIM_INTR_EEOF_EN
	__cim_set_eeof_line(100);
	__cim_enable_eeof_intr();
#else
#if !defined(CONFIG_SOC_JZ4750)
	__cim_set_eeof_line(0);
	__cim_disable_eeof_intr();
#endif
#endif
}

static void cim_init_config(struct camera_sensor_desc *desc)
{
	if(desc == NULL)
		return;

	memset(&jz_cim->cim_cfg,0,sizeof(jz_cim->cim_cfg));

	jz_cim->cim_cfg.cfg = cur_desc->cfg_info.configure_register;
	jz_cim->cim_cfg.cfg &=~CIM_CFG_DMA_BURST_TYPE_MASK;
#if defined(CONFIG_SOC_JZ4760B) || defined(CONFIG_SOC_JZ4770)
	jz_cim->cim_cfg.cfg |= CIM_CFG_DMA_BURST_INCR32  | (0<<CIM_CFG_RXF_TRIG_BIT);// (n+1)*burst = 2*16 = 32 <64
	jz_cim->cim_cfg.ctrl =  CIM_CTRL_DMA_SYNC | CIM_CTRL_FRC_1;
#elif defined(CONFIG_SOC_JZ4760)
	jz_cim->cim_cfg.cfg |= CIM_CFG_DMA_BURST_INCR16;
	jz_cim->cim_cfg.ctrl =  CIM_CTRL_DMA_SYNC | CIM_CTRL_FRC_1 | (1<<CIM_CTRL_RXF_TRIG_BIT);// (n+1)*burst = 2*16 = 32 <64
#else
	jz_cim->cim_cfg.cfg |= CIM_CFG_DMA_BURST_INCR8;
	jz_cim->cim_cfg.ctrl = CIM_CTRL_FRC_1 | CIM_CTRL_RXF_TRIG_8 | CIM_CTRL_FAST_MODE; // 16 < 32
#endif

	if (desc->bus_width > 8) {
		jz_cim->cim_cfg.cfg |= CIM_CFG_EXPAND_MODE;
		jz_cim->cim_cfg.cfg &= ~CIM_CFG_BW_MASK;
		jz_cim->cim_cfg.cfg |= (desc->bus_width - 9) << CIM_CFG_BW_LSB;

		jz_cim->cim_cfg.packed = 1;
	} else {
		jz_cim->cim_cfg.cfg &= ~CIM_CFG_EXPAND_MODE;

		jz_cim->cim_cfg.packed = jz_cim->ginfo.packed;
		jz_cim->cim_cfg.fmt_422 = jz_cim->ginfo.fmt_422;
	}

	return;
}


static int cim_device_init(void)
{
	cim_init_config(cur_desc);
	cim_config(&jz_cim->cim_cfg);
	cim_print_regs();
	return 0;
}


inline void cim_start(void)
{
	if(jz_cim->cim_started == TRUE)
		return;

       	jz_cim->cim_started = TRUE;

	cim_power_on();
	cur_desc->ops->sensor_set_power(0);
}

static void cim_stop_preview(void)
{
	if(jz_cim->cim_transfer_started == 0)
		return;

	jz_cim->cim_transfer_started = 0;

	__cim_disable();
	__cim_clear_state();
	//cur_desc->ops->sensor_set_power(1);
}


static void cim_stop(void)
{
	if(cur_desc == NULL || jz_cim->cim_started == FALSE)
		return;

	cim_stop_preview();

	cur_desc->ops->sensor_set_power(1);
	cim_power_off();

	jz_cim->cim_started = FALSE;
	jz_cim->sensor_inited = FALSE;
}


void cim_start_preview(void)
{
	if(cur_desc == NULL || jz_cim->cim_transfer_started == 1 || jz_cim->cim_started == FALSE)
		return;

	__cim_disable();

	fresh_id = -1;
	fresh_buf = 0;

	jz_cim->cim_transfer_started = 1;
	cur_desc->ops->sensor_set_power(0);

	cim_device_init();

	if(jz_cim->sensor_inited == FALSE)
	{
		cur_desc->ops->sensor_init();

		cur_desc->ops->sensor_set_parameter(CPCMD_SET_BALANCE,
			cur_desc->parms.balance_flag,cur_desc->parms.balance_flag_arg);
		cur_desc->ops->sensor_set_parameter(CPCMD_SET_EFFECT,
			cur_desc->parms.effect_flag,cur_desc->parms.effect_flag_arg);
		cur_desc->ops->sensor_set_parameter(CPCMD_SET_ANTIBANDING,
			cur_desc->parms.antibanding_flag,cur_desc->parms.antibanding_flag_arg);
		cur_desc->ops->sensor_set_parameter(CPCMD_SET_FLASH_MODE,
			cur_desc->parms.flash_mode_flag,cur_desc->parms.flash_mode_flag_arg);
		cur_desc->ops->sensor_set_parameter(CPCMD_SET_SCENE_MODE,
			cur_desc->parms.scene_mode_flag,cur_desc->parms.scene_mode_flag_arg);
		cur_desc->ops->sensor_set_parameter(CPCMD_SET_FOCUS_MODE,
			cur_desc->parms.focus_mode_flag,cur_desc->parms.focus_mode_flag_arg);

		jz_cim->sensor_inited = TRUE;
	}

	cim_init_preview_desc();

	cim_set_function(0,NULL);
	cim_set_preview_resolution(cur_desc->preview_parm);

	if(cur_desc->no_dma) //just for fake sensor test
		return;

	__cim_set_da(virt_to_phys(&frame_desc[0]));
	__cim_clear_state();	// clear state register
	__cim_reset_rxfifo();	// resetting rxfifo
	__cim_unreset_rxfifo();
	__cim_enable_dma();
	__cim_enable();

}

#define OUTPUT_TIME()												\
	do{													\
		struct timeval s;										\
		do_gettimeofday(&s);										\
		printk("%s, %d --- %ld us\n", __FUNCTION__, __LINE__,s.tv_sec * 1000000LL + s.tv_usec);		\
	}while(0)

static int cim_snapshot(void)
{
	if(cur_desc == NULL) return -ENODEV;

	wait_count = 0;
	snapshot_flag = 1;

	__cim_disable();
	__cim_clear_state();	// clear state register
	__cim_set_da(virt_to_phys(&capture_desc));
	__cim_reset_rxfifo();	// resetting rxfifo
	__cim_unreset_rxfifo();

	dprintk("%s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);

	cim_device_init();

	if(jz_cim->cim_transfer_started != 1)
	{
		cur_desc->ops->sensor_set_power(0);

		if(jz_cim->sensor_inited == FALSE)
		{
			cur_desc->ops->sensor_init();

		  	cur_desc->ops->sensor_set_parameter(CPCMD_SET_BALANCE,
				cur_desc->parms.balance_flag,cur_desc->parms.balance_flag_arg);
			cur_desc->ops->sensor_set_parameter(CPCMD_SET_EFFECT,
				cur_desc->parms.effect_flag,cur_desc->parms.effect_flag_arg);
			cur_desc->ops->sensor_set_parameter(CPCMD_SET_ANTIBANDING,
				cur_desc->parms.antibanding_flag,cur_desc->parms.antibanding_flag_arg);
			cur_desc->ops->sensor_set_parameter(CPCMD_SET_FLASH_MODE,
				cur_desc->parms.flash_mode_flag,cur_desc->parms.flash_mode_flag_arg);
			cur_desc->ops->sensor_set_parameter(CPCMD_SET_SCENE_MODE,
				cur_desc->parms.scene_mode_flag,cur_desc->parms.scene_mode_flag_arg);
			cur_desc->ops->sensor_set_parameter(CPCMD_SET_FOCUS_MODE,
				cur_desc->parms.focus_mode_flag,cur_desc->parms.focus_mode_flag_arg);

			jz_cim->sensor_inited = TRUE;
		}

		jz_cim->cim_transfer_started = 1;
	}

	cim_init_capture_desc();

	dprintk("1: SNAPSHOT WRITE DMA PADDR:%#010x\n",capture_desc.framebuf);

	cim_set_function(1,NULL);
	cim_set_capture_resoultion(cur_desc->capture_parm,1);

	dprintk("2: SNAPSHOT WRITE DMA PADDR:%#010x\n",capture_desc.framebuf);

	//OUTPUT_TIME();
	dprintk("%s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	if(unlikely(cur_desc->no_dma)) //just for fake sensor test
	{
		if(cur_desc->ops->camera_fill_buffer != NULL)
			cur_desc->ops->camera_fill_buffer(
			jz_cim->capture_binfo.base_vaddr,
			cur_desc
			);//only for fake camera test
	}
	else
	{
		dprintk("%s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
		dprintk("SNAPSHOT START!\n");
		dprintk("SNAPSHOT WRITE DMA PADDR:%#010x\n",capture_desc.framebuf);

		__cim_enable();

		interruptible_sleep_on(&jz_cim->capture_wait_queue);
	}
	//OUTPUT_TIME();

	dprintk("%s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	cim_stop_preview();
	snapshot_flag = 0;

	return 0;
}
/*==========================================================================
 * Interrupt handler
 *========================================================================*/

static irqreturn_t cim_irq_handler(int irq, void *dev_id)
{
	u32 state = REG_CIM_STATE;
	u32 state_back = state;
	unsigned long flags;

	iprintk(">>>>>>>>>>>>>>>>>>>state = %#0x\n", state);

	if ( (state & CIM_STATE_DMA_SOF) ||
	     (state & CIM_STATE_DMA_STOP) ||
	     (state & CIM_STATE_VDD) ||
	     (state & CIM_STATE_RXF_TRIG)) {
		if (state & CIM_STATE_DMA_SOF) {
			state &= ~CIM_STATE_DMA_SOF;
			iprintk("sof intrrupt occured\n");
		}

		if (state & CIM_STATE_DMA_STOP) {
			state &= ~CIM_STATE_DMA_STOP;
			iprintk("stop intrrupt occured\n");
		}

		if (state & CIM_STATE_VDD) {
			state &= ~CIM_STATE_VDD;
			iprintk("cim disable done!\n");
		}

		if (state & CIM_STATE_RXF_TRIG) {
			state &= ~CIM_STATE_RXF_TRIG;
			iprintk("rx trig reached!\n");
		}
	}

#if !defined(CONFIG_SOC_JZ4750)
	if (state & CIM_STATE_DMA_EEOF) {
		state &= ~CIM_STATE_DMA_EEOF;
		iprintk("eeof intrrupt occured!\n");
	}
#endif

	if (state & CIM_STATE_DMA_EOF)
       	{
		if(likely(snapshot_flag != 1))
	       	{
			spin_lock_irqsave(&fresh_lock,flags);
			jz_cim->preview_timeout_state = 1;

			iprintk("__cim_get_iid() = %d\n", __cim_get_iid());
			//cim_print_regs();
			fresh_id =  __cim_get_iid() - 1;
			if(fresh_id == -1)
				fresh_id = SWAP_BUF-1;


			spin_unlock_irqrestore(&fresh_lock,flags);

			if(waitqueue_active(&jz_cim->preview_wait_queue))
				wake_up_interruptible(&jz_cim->preview_wait_queue);

			REG_CIM_STATE &= ~CIM_STATE_DMA_EOF;
			return IRQ_HANDLED;
		}
		else
	       	{
			dprintk("capture frame wait : %d\n",wait_count);

			++wait_count;
#if 0
			if(wait_count == cur_desc->wait_frames)
			{
				capture_desc.dmacmd |= CIM_CMD_STOP;
				dma_cache_wback((unsigned long)(&capture_desc), sizeof(struct cim_desc));
			}
#endif

			if(wait_count >= cur_desc->wait_frames + 1)
			{
				__cim_disable();
				wake_up_interruptible(&jz_cim->capture_wait_queue);
				__cim_clear_state();
				return IRQ_HANDLED;
			}

			REG_CIM_STATE &= ~CIM_STATE_DMA_EOF;
			return IRQ_HANDLED;
		}
	}

	state = state_back;
	if ( (state & CIM_STATE_RXF_OF)
#if defined(CONFIG_SOC_JZ4760B) || defined(CONFIG_SOC_JZ4770)
	     || (state & CIM_STATE_Y_RF_OF) ||
	     (state & CIM_STATE_CB_RF_OF) ||
	     (state & CIM_STATE_CR_RF_OF)
#endif
	     )
       	{
		if (state & CIM_STATE_RXF_OF)
			printk("OverFlow interrupt!\n");

#if defined(CONFIG_SOC_JZ4760B) || defined(CONFIG_SOC_JZ4770)
		if (state & CIM_STATE_Y_RF_OF)
			printk("Y overflow interrupt!!!\n");

		if (state & CIM_STATE_CB_RF_OF)
			printk("Cb overflow interrupt!!!\n");

		if (state & CIM_STATE_CR_RF_OF)
			printk("Cr overflow interrupt!!!\n");
#endif

		cim_print_regs();

		REG_CIM_STATE &= ~CIM_STATE_VDD;
		__cim_disable();
		__cim_reset_rxfifo();
		__cim_unreset_rxfifo();
		__cim_clear_state();	// clear state register
		__cim_enable();

		return IRQ_HANDLED;
	}

	__cim_clear_state();	// clear state register
 	return IRQ_HANDLED;
}

static struct file_operations cim_fops =
{
	open:			cim_open,
	release:		cim_release,
	read:			cim_read,
	write:			cim_write,
	ioctl:			cim_ioctl,
	mmap:			cim_mmap,

};

static atomic_t jzcim_opened = ATOMIC_INIT(1); /* initially not opened */

static int cim_open(struct inode *inode, struct file *filp)
{
	if (jz_cim->ginfo.sensor_count <= 0)
		return -ENODEV;

	if (! atomic_dec_and_test (&jzcim_opened)) {
		atomic_inc(&jzcim_opened);
		return -EBUSY; /* already open */
	}

	return cim_fb_alloc();
}

static int cim_release(struct inode *inode, struct file *filp)
{
	cim_fb_free();
	atomic_inc(&jzcim_opened); /* release the device */
	return 0;
}

unsigned int get_phy_addr(unsigned int vaddr)
{
	unsigned int addr=vaddr & (PAGE_SIZE-1);
	pgd_t   *pgdir;
#ifdef CONFIG_PGTABLE_4
	pud_t	*pudir;
#endif
     	pmd_t   *pmdir;
      	pte_t   *pte;

	dprintk("current task(%d)'s pgd is %p\n",current->pid,current->mm->pgd);

	pgdir=pgd_offset(current->mm,vaddr);
	if(pgd_none(*pgdir)||pgd_bad(*pgdir))
		return   -EINVAL;

#ifdef CONFIG_PGTABLE_4
	pudir=pud_offset(pgdir,vaddr);
	if(pud_none(*pudir)||pud_bad(*pudir))
		return   -EINVAL;

	pmdir=pmd_offset(pudir,vaddr);
	if(pmd_none(*pmdir)||pmd_bad(*pmdir))
		return   -EINVAL;
#else
	pmdir=pmd_offset((pud_t *)pgdir,vaddr);
	if(pmd_none(*pmdir)||pmd_bad(*pmdir))
		return   -EINVAL;
#endif

	pte=pte_offset(pmdir,vaddr);

	if(pte_present(*pte))
	{
		return   addr | (pte_pfn(*pte)<<PAGE_SHIFT);
		//pte_page(*pte);
	}

	return   -EINVAL;
}

static ssize_t cim_read(struct file *filp, char *buf, size_t size, loff_t *l)
{
	dprintk("user buf paddr = 0x%x\n",get_phy_addr((unsigned int)buf));
	return -1;
}

static ssize_t cim_write(struct file *filp, const char *buf, size_t size, loff_t *l)
{
	return -1;
}

static unsigned int cim_get_preview_buf(int is_pbuf)
{
	unsigned int buffer;
	unsigned long flags;
	u32 tmp_addr;

	if(unlikely(fresh_id == -1))
	{
		iprintk("w");
		//cim_print_regs();
		interruptible_sleep_on_timeout(&jz_cim->preview_wait_queue,10*HZ);
		//cim_print_regs();
	}

	if(unlikely(jz_cim->preview_timeout_state != 1)) {
		printk("========>preview timeout!!!\n");
		//cim_print_regs();
		return (unsigned int)(~0);
	}

	//cim_print_regs();

/******/spin_lock_irqsave(&fresh_lock,flags);/*********************************************************/

	iprintk("===>fresh_buf = %d fresh_id = %d\n", fresh_buf, fresh_id);
	tmp_addr=frame_desc[fresh_id].framebuf;
	frame_desc[fresh_id].framebuf=frame_desc[SWAP_BUF+fresh_buf].framebuf;
	frame_desc[SWAP_BUF+fresh_buf].framebuf=tmp_addr;
	if (!jz_cim->cim_cfg.packed) {
		iprintk("=======>%s:%d we are test sep!!!\n", __FUNCTION__, __LINE__);
		tmp_addr = frame_desc[fresh_id].cb_frame;
		frame_desc[fresh_id].cb_frame = frame_desc[SWAP_BUF+fresh_buf].cb_frame;
		frame_desc[SWAP_BUF+fresh_buf].cb_frame=tmp_addr;

		tmp_addr = frame_desc[fresh_id].cr_frame;
		frame_desc[fresh_id].cr_frame = frame_desc[SWAP_BUF+fresh_buf].cr_frame;
		frame_desc[SWAP_BUF+fresh_buf].cr_frame=tmp_addr;
	}
	dma_cache_wback((unsigned long)(&frame_desc[fresh_id]), sizeof(struct cim_desc));

	jz_cim->preview_timeout_state = 0;
	fresh_id = -1;

/******/spin_unlock_irqrestore(&fresh_lock,flags);/*******************************************************/


	if(likely(is_pbuf == 1))
		buffer=(unsigned int)(frame_desc[SWAP_BUF+fresh_buf].framebuf);
	else
		buffer=(unsigned int)(phys_to_virt(frame_desc[SWAP_BUF+fresh_buf].framebuf)-buf_offsize_app_drv);

	fresh_buf++;
	if(fresh_buf >= GET_BUF)
		fresh_buf =0;

	//iprintk("(%d)",__cim_get_fid());
	iprintk("(%x)transfer id %d\n",REG_CIM_FA,(REG_CIM_FA - jz_cim->preview_binfo.base_paddr)/jz_cim->ginfo.max_preview_size);
	iprintk("(%x)return buffer id %d\n",buffer,(buffer-jz_cim->preview_binfo.base_paddr)/jz_cim->ginfo.max_preview_size);
	return buffer;
}

static struct camera_sensor_desc *get_sensor_by_id(unsigned int id)
{
	struct camera_sensor_desc *si=NULL,*retval=NULL;
	mutex_lock(&sensor_lock);
	list_for_each_entry(si, &sensor_list, list)
	{
		if(si->sensor_id == id)
		{
			retval = si;
			break;
		}
	}
	mutex_unlock(&sensor_lock);

	return retval;
}

static int cim_get_support_size(void __user *arg)
{
	if(cur_desc == NULL)
		return -ENODEV;
	return copy_to_user(arg,cur_desc->resolution_table,
			(sizeof(struct resolution_info) * cur_desc->resolution_table_nr));
}

static void fill_info_by_desc(struct sensor_info *info,struct camera_sensor_desc *desc)
{
	strcpy(info->name,desc->name);
	info->sensor_id = desc->sensor_id;
	dprintk("GET SENSOR INFO (ID : %d NAME : %s )\n",info->sensor_id,info->name);
	info->resolution_table_nr = desc->resolution_table_nr;
	info->bus_width = desc->bus_width;

	memcpy(&info->flags,&desc->flags,sizeof(struct mode_flags));

	memcpy(&info->preview_parm,&desc->preview_parm,sizeof(struct resolution_info));
	memcpy(&info->capture_parm,&desc->capture_parm,sizeof(struct resolution_info));

	memcpy(&info->max_capture_parm,&desc->max_capture_parm,sizeof(struct resolution_info));
	memcpy(&info->max_preview_parm,&desc->max_preview_parm,sizeof(struct resolution_info));
}


static int cim_get_sensor_info(void __user *arg,struct sensor_info *info)
{
	if(cur_desc != NULL)
	{
		fill_info_by_desc(info,cur_desc);
		return copy_to_user(arg,info,sizeof(*info));
	}

	dprintk("CIM GET SENSOR INFO ERROR:CUR_DESC IS NULL\n");
	return -ENODEV;
}

#if !defined(CONFIG_SOC_JZ4750)
static int cim_enable_image_mode(int image_w,int image_h,int width,int height)
{
	int voffset,hoffset, half_words_per_line = 0;

	voffset = (height - image_h) / 2;
	voffset &= ~0x1;      /* must be even */

	/* hoffset / 2 * 4 = (width - image_w) / 2 * (bpp / 8)
	 * ---> hoffset = (width - image_w) / 2 * (bpp / 8) / 4 * 2
	 */
	if (jz_cim->cim_cfg.fmt_422) {
		hoffset = (width - image_w) / 2;
		hoffset &= ~0x1;      /* must be even */

		half_words_per_line = image_w;
	} else {
		hoffset = (width - image_w) / 2;
		hoffset = hoffset * 3 / 4 * 2;

		half_words_per_line = image_w * 3 / 2; /* image_w must be even */
	}

	__cim_set_line(image_h);
	__cim_set_pixel(half_words_per_line);

	__cim_set_v_offset(voffset);
	__cim_set_h_offset(hoffset);

	__cim_enable_size_func();

	dprintk("enable image mode (real size %d x %d) - %d x %d\n",width,height,image_w,image_h);
	return 0;
}


static int cim_disable_image_mode(void)
{
	__cim_disable_size_func();
	//REG_CIM_CTRL &= ~CIM_CTRL_WIN_EN;
	dprintk("disable image mode\n");
	return 0;
}
#else  /* !CONFIG_SOC_JZ4750 */
#define cim_enable_image_mode(iw, ih, w, h) do {  } while(0)
#define cim_disable_image_mode() do {  } while(0)
#endif

static void param_normalization(struct resolution_info *param)  {
	if((param->bpp != 8) && (param->bpp != 16) && (param->bpp != 24) && (param->bpp != 32) ) {
		if (jz_cim->cim_cfg.fmt_422)
			param->bpp = 16;
		else   /* yuv444 */
			param->bpp = 24;
	}
	if(param->format == (unsigned int)(-1))
		param->format= cur_desc->preview_parm.format;
}

static int cim_set_preview_resolution(struct resolution_info param)
{
	int i, max_preview_index, imgsize, framesize;

	dprintk("SET PREVIEW RESOULTION %d\tX\t%d\t%d\n",param.width,param.height,param.bpp);

	if(cur_desc == NULL)
		return -ENODEV;

	param_normalization(&param);

	imgsize = param.width * param.height;
	framesize = (imgsize * param.bpp + 7) >> 3;
	if (framesize > cur_desc->max_preview_size){
		dprintk("ERROR! Preview size is too large!\n");
		return -EINVAL;
	}

	cur_desc->preview_parm.width = param.width;
	cur_desc->preview_parm.height = param.height;
	cur_desc->preview_parm.bpp = param.bpp;

	if(jz_cim->cim_started != TRUE || jz_cim->cim_transfer_started != 1)
		return -1;

	if(cur_desc->ops->sensor_set_resolution== NULL)
		return -ENODEV;
//---------------------------------------------------------------------------
	for(max_preview_index=0;max_preview_index<cur_desc->resolution_table_nr;max_preview_index++)
	{
		if(cur_desc->resolution_table[max_preview_index].width == cur_desc->max_preview_parm.width
			&& cur_desc->resolution_table[max_preview_index].height == cur_desc->max_preview_parm.height)
			break;
	}

	for(i=max_preview_index;i<cur_desc->resolution_table_nr;i++)
	{

		dprintk("request preview size: %d x %d\t\tscan preview size: %d x %d\n",
				param.width,
				param.height,
				cur_desc->resolution_table[i].width,
				cur_desc->resolution_table[i].height);
		if(param.width == cur_desc->resolution_table[i].width && param.height == cur_desc->resolution_table[i].height)
		{
			dprintk("found preview size! now preview size is %d x %d\n",param.width,param.height);
			break;
		}
	}

	if(i >= cur_desc->resolution_table_nr)
	{
		dprintk("size not found! use image mode!\n");
		for(i=cur_desc->resolution_table_nr-1;i>=max_preview_index;i--)
		{
			if(
					cur_desc->resolution_table[i].width >= param.width
					&& cur_desc->resolution_table[i].height >= param.height)
				break;
		}

		cim_enable_image_mode(
				param.width,
				param.height,
				cur_desc->resolution_table[i].width,
				cur_desc->resolution_table[i].height);

		cur_desc->ops->sensor_set_resolution(
			cur_desc->resolution_table[i].width,
			cur_desc->resolution_table[i].height,
			cur_desc->preview_parm.bpp,
			cur_desc->preview_parm.format,
			CAMERA_MODE_PREVIEW);

		dprintk("PREVIEW SIZE:%d x %d\t\tPREVIEW IMAGE:%d x %d\n",
				cur_desc->resolution_table[i].width,
				cur_desc->resolution_table[i].height,
				param.width,
				param.height);
	}
	else
	{
		cim_disable_image_mode();
		cur_desc->ops->sensor_set_resolution(
			cur_desc->preview_parm.width,
			cur_desc->preview_parm.height,
			cur_desc->preview_parm.bpp,
			cur_desc->preview_parm.format,
			CAMERA_MODE_PREVIEW);
	}
//---------------------------------------------------------------------------
	for (i = 0; i < SWAP_NR - 1; i++) {
		if (jz_cim->cim_cfg.packed) {
			frame_desc[i].dmacmd &= ~CIM_CMD_LEN_MASK;
			frame_desc[i].dmacmd |= (framesize + 3) >> 2;
			dprintk("framesize = %d\n", framesize);
		} else {
			dprintk("=======>%s:%d we are test sep!!!\n", __FUNCTION__, __LINE__);
			frame_desc[i].dmacmd &= ~CIM_CMD_LEN_MASK;
			frame_desc[i].dmacmd |= (imgsize + 3) >> 2;

			/* FIXME: test YCbCr444 or YCbCr422 here */
			if (jz_cim->cim_cfg.fmt_422) {
				frame_desc[i].cb_frame = frame_desc[i].framebuf + imgsize;
				frame_desc[i].cb_len = ( (imgsize + 1) / 2 + 3) >> 2;
				frame_desc[i].cr_frame = frame_desc[i].cb_frame + imgsize / 2;
				frame_desc[i].cr_len = ( (imgsize + 1) / 2 + 3) >> 2;
			} else {
				printk("we are testing yuv444\n");
				frame_desc[i].cb_frame = frame_desc[i].framebuf + imgsize;
				frame_desc[i].cb_len = imgsize >> 2;
				frame_desc[i].cr_frame = frame_desc[i].cb_frame + imgsize;
				frame_desc[i].cr_len = imgsize >> 2;
			}
		}
		dma_cache_wback((unsigned long)(&frame_desc[i]), sizeof(struct cim_desc));
	}

	cim_print_buffers();
	cim_print_regs();
	printk("=========================\n");
	return 0;
}

static int cim_set_capture_resoultion(struct resolution_info param,int state)
{
	int i,imgsize, framesize, max_capture_index; // words per frame

	dprintk("SET CAPTURE RESOULTION %d\tX\t%d\t%d\n",param.width,param.height,param.bpp);

	if(cur_desc == NULL)
		return -ENODEV;

	param_normalization(&param);

	imgsize = param.width * param.height;
	framesize = (imgsize * param.bpp + 7) >> 3;
	if (framesize > cur_desc->max_capture_size){
		dprintk("ERROR! Capture size is too large!\n");
		return -EINVAL;
	}

	cur_desc->capture_parm.width = param.width;
	cur_desc->capture_parm.height = param.height;
	cur_desc->capture_parm.bpp = param.bpp;

	if(state == 0)
		return 0;

	if(jz_cim->cim_started != TRUE || jz_cim->cim_transfer_started != 1)
		return -1;

	if(cur_desc->ops->sensor_set_resolution== NULL)
		return -ENODEV;
//---------------------------------------------------------------------------------------------

	for(max_capture_index=0;max_capture_index<cur_desc->resolution_table_nr;max_capture_index++)
	{
		if(cur_desc->resolution_table[max_capture_index].width == cur_desc->max_capture_parm.width
			&& cur_desc->resolution_table[max_capture_index].height == cur_desc->max_capture_parm.height)
			break;
	}

	for(i=max_capture_index;i<cur_desc->resolution_table_nr;i++)
	{

		dprintk("request capture size: %d x %d\t\tscan capture size: %d x %d\n",
				param.width,
				param.height,
				cur_desc->resolution_table[i].width,
				cur_desc->resolution_table[i].height);
		if(param.width == cur_desc->resolution_table[i].width && param.height == cur_desc->resolution_table[i].height)
		{
			dprintk("found capture size! now capture size is %d x %d\n",param.width,param.height);
			break;
		}
	}

	if(i >= cur_desc->resolution_table_nr)
	{
		dprintk("size not found! use image mode!\n");
		for(i=cur_desc->resolution_table_nr-1;i>=max_capture_index;i--)
		{
			if(cur_desc->resolution_table[i].width > param.width && cur_desc->resolution_table[i].height > param.height)
				break;
		}

		cim_enable_image_mode(
				param.width,
				param.height,
				cur_desc->resolution_table[i].width,
				cur_desc->resolution_table[i].height);

		cur_desc->ops->sensor_set_resolution(
			cur_desc->resolution_table[i].width,
			cur_desc->resolution_table[i].height,
			cur_desc->capture_parm.bpp,
			cur_desc->capture_parm.format,
			CAMERA_MODE_CAPTURE);

		dprintk("CAPTURE SIZE:%d x %d\t\tCAPTURE IMAGE:%d x %d\n",
				cur_desc->resolution_table[i].width,
				cur_desc->resolution_table[i].height,
				param.width,
				param.height);
	}
	else
	{
		cim_disable_image_mode();
		cur_desc->ops->sensor_set_resolution(
		cur_desc->capture_parm.width,
		cur_desc->capture_parm.height,
		cur_desc->capture_parm.bpp,
		cur_desc->capture_parm.format,
		CAMERA_MODE_CAPTURE);
	}

	//---------------------------------------------------------------------------------------------
	if (jz_cim->cim_cfg.packed) {
		capture_desc.dmacmd &= ~CIM_CMD_LEN_MASK;
		capture_desc.dmacmd |= (framesize + 3) >>2;
	} else {
		dprintk("=======>%s:%d we are test sep!!!\n", __FUNCTION__, __LINE__);
		capture_desc.dmacmd &= ~CIM_CMD_LEN_MASK;
		capture_desc.dmacmd |= (imgsize + 3) >> 2;
		/* FIXME: test YCbCr444 or YCbCr422 */
		if (jz_cim->cim_cfg.fmt_422) {
			capture_desc.cb_frame = jz_cim->capture_binfo.base_paddr + imgsize;
			capture_desc.cb_len = ((imgsize + 1) / 2 + 3) >> 2;
			capture_desc.cb_frame = jz_cim->capture_binfo.base_paddr + imgsize + imgsize / 2;
			capture_desc.cr_len = ((imgsize + 1) / 2 + 3) >> 2;
		} else {
			capture_desc.cb_frame = jz_cim->capture_binfo.base_paddr + imgsize;
			capture_desc.cb_len = (imgsize + 3) >> 2;
			capture_desc.cb_frame = jz_cim->capture_binfo.base_paddr + imgsize + imgsize;
			capture_desc.cr_len = (imgsize + 3) >> 2;
		}
	}
	dma_cache_wback((unsigned long)(&capture_desc), sizeof(struct cim_desc));

	dprintk("SET CAPTURE RESOULTION %d\tX\t%d\t%d\n -- OK!",param.width,param.height,param.bpp);
	return 0;
}

static int cim_set_function(int function,void *cookie)
{
	if(cur_desc == NULL)
		return -ENODEV;
	if(cur_desc->ops->sensor_set_function == NULL)
		return -ENODEV;

	return cur_desc->ops->sensor_set_function(function,cookie);
}


static int cim_set_output_format(pixel_format_flag_t flag,int arg)
{
	return 0;//disabled interface

	dprintk("cim_set_output_format\n");
	if(cur_desc == NULL)
		return -ENODEV;
//	if(cur_desc->ops->camera_set_output_format == NULL)
//		return -ENODEV;

	cur_desc->parms.pixel_format_flag = flag;
	cur_desc->parms.pixel_format_flag_arg = arg;

	if(jz_cim->cim_transfer_started != 1)
		return 0;

	if(flag == PIXEL_FORMAT_YUV422I)
		__cim_input_data_format_select_YUV422();

	if(flag == PIXEL_FORMAT_RGB565)
		;//	__cim_input_data_format_select_RGB();

//	cur_desc->ops->camera_set_output_format(flag,arg);
	return 0;
}

static int cim_set_param(struct camera_param *cparam)
{
	if(cur_desc == NULL)
		return -ENODEV;
	if(cur_desc->ops->sensor_set_parameter == NULL)
		return -ENODEV;

	if(jz_cim->cim_transfer_started == 1 && !cur_desc->no_dma)
	{
		REG_CIM_STATE &= ~CIM_STATE_VDD;
		__cim_disable();
	}

	switch (cparam->cmd)
	{
	case CPCMD_SET_PREVIEW_RESOLUTION:
		cim_set_preview_resolution(cparam->preview_param);
		break;
	case CPCMD_SET_CAPTURE_RESOLUTION:
		cim_set_capture_resoultion(cparam->capture_param,0);
		break;
	case CPCMD_SET_OUTPUT_FORMAT:
		cim_set_output_format(cparam->flags.pixel_format_flag,cparam->mode_arg);
		break;
	case CPCMD_SET_BALANCE:
		cur_desc->parms.balance_flag = cparam->flags.balance_flag;
		cur_desc->parms.balance_flag_arg = cparam->mode_arg;
		if(jz_cim->cim_transfer_started == 1)
			cur_desc->ops->sensor_set_parameter(
					cparam->cmd,
					cur_desc->parms.balance_flag,
					cur_desc->parms.balance_flag_arg);
		break;
	case CPCMD_SET_EFFECT:
		cur_desc->parms.effect_flag = cparam->flags.effect_flag;
		cur_desc->parms.effect_flag_arg = cparam->mode_arg;
		if(jz_cim->cim_transfer_started == 1)
			cur_desc->ops->sensor_set_parameter(
					cparam->cmd,
					cur_desc->parms.effect_flag,
					cur_desc->parms.effect_flag_arg);
		break;
	case CPCMD_SET_ANTIBANDING:
		cur_desc->parms.antibanding_flag = cparam->flags.antibanding_flag;
		cur_desc->parms.antibanding_flag_arg = cparam->mode_arg;
		if(jz_cim->cim_transfer_started == 1)
			cur_desc->ops->sensor_set_parameter(
					cparam->cmd,
					cur_desc->parms.antibanding_flag,
					cur_desc->parms.antibanding_flag_arg);
		break;
	case CPCMD_SET_FLASH_MODE:
		cur_desc->parms.flash_mode_flag = cparam->flags.flash_mode_flag;
		cur_desc->parms.flash_mode_flag_arg = cparam->mode_arg;
		if(jz_cim->cim_transfer_started == 1)
			cur_desc->ops->sensor_set_parameter(
					cparam->cmd,
					cur_desc->parms.flash_mode_flag,
					cur_desc->parms.flash_mode_flag_arg);
		break;
	case CPCMD_SET_SCENE_MODE:
		cur_desc->parms.scene_mode_flag = cparam->flags.scene_mode_flag;
		cur_desc->parms.scene_mode_flag_arg = cparam->mode_arg;
		if(jz_cim->cim_transfer_started == 1)
			cur_desc->ops->sensor_set_parameter(
					cparam->cmd,
					cur_desc->parms.scene_mode_flag,
					cur_desc->parms.scene_mode_flag_arg);
		break;
	case CPCMD_SET_FOCUS_MODE:
		cur_desc->parms.focus_mode_flag = cparam->flags.focus_mode_flag;
		cur_desc->parms.focus_mode_flag_arg = cparam->mode_arg;
		if(jz_cim->cim_transfer_started == 1)
			cur_desc->ops->sensor_set_parameter(
					cparam->cmd,
					cur_desc->parms.focus_mode_flag,
					cur_desc->parms.focus_mode_flag_arg);
		break;
	case CPCMD_SET_PREVIEW_FPS:
		cur_desc->parms.preview_fps_arg = cparam->mode_arg;
		if(jz_cim->cim_transfer_started == 1)
			cur_desc->ops->sensor_set_parameter(
					cparam->cmd,
					-1,
					cur_desc->parms.preview_fps_arg);
		break;
	case CPCMD_SET_NIGHTSHOT_MODE:
		cur_desc->parms.nightshot_mode_arg = cparam->mode_arg;
		if(jz_cim->cim_transfer_started == 1)
			cur_desc->ops->sensor_set_parameter(
					cparam->cmd,
					-1,
					cur_desc->parms.nightshot_mode_arg);
		break;
	case CPCMD_SET_LUMA_ADAPTATION:
		cur_desc->parms.luma_adaptation_arg = cparam->mode_arg;
		if(jz_cim->cim_transfer_started == 1)
			cur_desc->ops->sensor_set_parameter(
					cparam->cmd,
					-1,
					cur_desc->parms.luma_adaptation_arg);
		break;

	default:
		dprintk("Not supported command: 0x%x\n", cparam->cmd);
		break;
	}

	if(jz_cim->cim_transfer_started == 1 && !cur_desc->no_dma)
	{
		__cim_clear_state();	// clear state register
		__cim_reset_rxfifo();	// resetting rxfifo
		__cim_unreset_rxfifo();
		__cim_enable();
	}

	return 0;
}


static unsigned int cim_get_global_info(void __user *argp)
{
	return copy_to_user(argp,&(jz_cim->ginfo),sizeof(struct global_info));
}


static int cim_select_sensor(unsigned int sensor_id)
{
	struct camera_sensor_desc *tmp;
	tmp = get_sensor_by_id(sensor_id);

	if(tmp == NULL)
		return -EFAULT;

	cim_stop_preview();
	cur_desc->ops->sensor_set_power(1);
	cur_desc = tmp;

	if(jz_cim->cim_started == TRUE)
		cur_desc->ops->sensor_set_power(0);

	return 0;
}

#if 0
static int cim_set_buffer(void __user *argp, struct camera_buffer_info *binfo)
{
	unsigned int mb, cap_mb;
	struct camera_buffer_info tmp;

	if (copy_from_user(&tmp, argp, sizeof(struct camera_buffer_info)))
		return -EFAULT;

	if (tmp.buffer_size < binfo->buffer_size)
	{
		dprintk("MEM IS TOO SMALL!\n");
		return -ENOMEM;
	}
	memcpy(binfo, &tmp, sizeof(struct camera_buffer_info));

	mb = (unsigned int)jz_cim->preview_mem_base;
	cap_mb = (unsigned int)jz_cim->cap_mem_base;
	if ((jz_cim->preview_binfo.base_vaddr != mb)
	 && (jz_cim->capture_binfo.base_vaddr != cap_mb))
	{
		cim_framebuffer_destroy();
		dprintk("CIM FRAME BUFFER DESTORY\n");
	}

	return 0;
}
#endif

/**************************
 *     IOCTL Handlers     *
 **************************/
static int cim_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval;
	void __user *argp = (void __user *)arg;
	switch (cmd)
       	{
	case IOCTL_CIM_START:
	{
		dprintk("IOCTL_START_CIM\n");
		cim_start();
		return 0;
	}
	case IOCTL_CIM_STOP:
	{
		dprintk("IOCTL_STOP_CIM\n");
		cim_stop();
		return 0;
	}
	case IOCTL_CIM_CHANGE_PACK_MODE:
		jz_cim->cim_cfg.packed = *((unsigned int *)argp);
		jz_cim->ginfo.packed = jz_cim->cim_cfg.packed;
		printk("=============>set packed to %d\n", jz_cim->ginfo.packed);
		return 0;
	case IOCTL_CIM_CHANGE_FMT:
		jz_cim->cim_cfg.fmt_422 = *((unsigned int *)argp);
		jz_cim->ginfo.fmt_422 = jz_cim->cim_cfg.fmt_422;
		printk("=============>set fmt_422 to %d\n", jz_cim->ginfo.fmt_422);
		return 0;
	case IOCTL_CIM_START_PREVIEW:
	{
		dprintk("IOCTL_CIM_START_PREVIEW\n");
		cim_start_preview();
		return 0;
	}
	case IOCTL_CIM_STOP_PREVIEW:
	{
		dprintk("IOCTL_CIM_STOP_PREVIEW\n");
		cim_stop_preview();
		return 0;
	}
	case IOCTL_CIM_SELECT_SENSOR:
	{
		return cim_select_sensor(arg);
	}
	case IOCTL_CIM_GET_PREVIEW_PARAM:
	{
		return copy_to_user(argp, &cur_desc->preview_parm, sizeof(struct resolution_info)) ? -EFAULT : 0;
	}
	case IOCTL_CIM_AF_INIT:
	{
		if(jz_cim->cim_started != TRUE || jz_cim->cim_transfer_started != 1)
			return -1;
		__cim_disable();
		__cim_reset_rxfifo();
		__cim_unreset_rxfifo();
		__cim_clear_state();

		retval = cim_set_function(4,(void*)arg);

		__cim_enable();

		return retval;
	}
	case IOCTL_CIM_GET_CAPTURE_PARAM:
	{
		return copy_to_user(argp, &cur_desc->capture_parm, sizeof(struct resolution_info)) ? -EFAULT : 0;
	}
	case IOCTL_CIM_GET_CAPTURE_BUF_INFO:
	{
		return copy_to_user(argp, &jz_cim->capture_binfo, sizeof(struct camera_buffer_info)) ? -EFAULT : 0;
	}
	case IOCTL_CIM_GET_PREVIEW_BUF_INFO:
	{
		return copy_to_user(argp, &jz_cim->preview_binfo, sizeof(struct camera_buffer_info)) ? -EFAULT : 0;
	}
#if 0
	case IOCTL_CIM_SET_PREVIEW_MEM:
	{
		dprintk("IOCTL_SET_PREVIEW_MEM\n");
		return cim_set_buffer(argp,&jz_cim->preview_binfo);
	}
	case IOCTL_CIM_SET_CAPTURE_MEM:
	{
		dprintk("IOCTL_SET_CAPTURE_MEM\n");
		return cim_set_buffer(argp,&jz_cim->capture_binfo);
	}
#endif
	case IOCTL_CIM_GET_PREVIEW_OFFSET:
	{
		unsigned int offset;

		if(cur_desc->no_dma)
		{
			if(cur_desc->ops->camera_fill_buffer != NULL)
				cur_desc->ops->camera_fill_buffer(
						jz_cim->preview_binfo.base_vaddr,
						cur_desc
						);//only for fake camera test
			offset = 0;
		}
		else
		{
			offset = cim_get_preview_buf(1);

			if(offset == (unsigned int)(~0))
				return -1;
			offset -= jz_cim->preview_binfo.base_paddr;
		}

		return copy_to_user(argp, &offset, sizeof(offset)) ? -EFAULT : 0;
	}
	case IOCTL_CIM_TAKE_SNAPSHOT:
	{
		dprintk("IOCTL_CIM_TAKE_SNAPSHOT\n");
		return cim_snapshot();
	}
	case IOCTL_CIM_GET_GLOBAL_INFO:
	{
		dprintk("IOCTL_GET_GLOBAL_INFO\n");
		return cim_get_global_info(argp);
	}
	case IOCTL_CIM_GET_SUPPORT_SIZE:
	{
		dprintk("IOCTL_GET_SUPPORT_SIZE\n");
		return cim_get_support_size(argp);
	}
	case IOCTL_CIM_GET_SENSOR_INFO:
	{
		struct sensor_info info;
		dprintk("IOCTL_GET_SENSOR_INFO\n");
		return cim_get_sensor_info(argp,&info);

	}
	case IOCTL_CIM_SET_PARAM:
	{
		struct camera_param param;
		if (copy_from_user(&param, (void *)arg, sizeof(param)))
			return -EFAULT;

		return	cim_set_param(&param);
	}
	case IOCTL_CIM_STOP_FOCUS:
	{
		return cim_set_function(5,NULL);
	}
	case IOCTL_CIM_DO_FOCUS:
	{
		return cim_set_function(3,NULL);
	}
	default:
		dprintk("Not supported command: 0x%x\n", cmd);
		return -EINVAL;
		break;
	}
	return 0;
}

//Use mmap /dev/fb can only get a non-cacheable Virtual Address.
static int cim_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long start;
	unsigned long off;
	u32 len;

	dprintk("%s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	if( jz_cim->mem_base== 0 )
	{
		if (cim_fb_alloc())
		{
			dprintk("No mem: Alloc memory for cim DMA\n");
			return -ENOMEM;
		}
	}

	off = vma->vm_pgoff << PAGE_SHIFT;

	// frame buffer memory
	start = virt_to_phys(jz_cim->mem_base);
	len = PAGE_ALIGN((start & ~PAGE_MASK) + ((unsigned long)jz_cim->mem_size));

	start &= PAGE_MASK;

	if ((vma->vm_end - vma->vm_start + off) > len) {
		dprintk("Error: vma is larger than memory length\n");
		return -EINVAL;
	}
	off += start;

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);	// Uncacheable
#if  defined(CONFIG_MIPS32)
 	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
 	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;		/* Uncacheable */
#endif

	if (remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
	vma->vm_end - vma->vm_start,
	vma->vm_page_prot)) {
	return -EAGAIN;
	}

	buf_offsize_app_drv =(unsigned int )(jz_cim->mem_base)-(unsigned int )(vma->vm_start);

#if 0
	dprintk("vma->vm_start: 0x%x\n",vma->vm_start);
	dprintk("vma->vm_end: 0x%x\n",vma->vm_end);
	dprintk("vma->vm_pgoff: 0x%x\n",vma->vm_pgoff);
	dprintk("jz_cim->mem_base(V): 0x%x\n",mem_base);
	dprintk("jz_cim->mem_base(P): 0x%x\n",virt_to_phys(mem_base));
	dprintk("jz_cim->mem_size: %dbytes\n",mem_size);
	dprintk("buf_offsize_app_drv = jz_cim->mem_base-vma->vm_start\n");
	dprintk("buf_offsize_app_drv = 0x%x\n",buf_offsize_app_drv);
	dprintk("get_phy_addr test addr = 0x%x\n",get_phy_addr(vma->vm_start));
#endif

	return 0;
}

static struct miscdevice cim_dev = {
	CIM_MINOR,
	"cim",
	&cim_fops
};


static int __init cim_init(void)
{
	int ret;

	__gpio_as_cim_10bit();

	jz_cim = kzalloc(sizeof(*jz_cim), GFP_KERNEL);
	if(jz_cim == NULL)
	{
		dprintk("no memory!\n");
		return -ENOMEM;
	}

#if !defined(CONFIG_SOC_JZ4750)
	jz_cim->ginfo.window_support = 1;
#else
	jz_cim->ginfo.window_support = 0;
#endif

	cim_power_on();

	cim_scan_sensor();

	cim_power_off();

	sensors_make_default();

	if(cur_desc == NULL)
	{
		dprintk("no sensor found!\nCIM exit!");
	}

	init_waitqueue_head(&jz_cim->capture_wait_queue);
	init_waitqueue_head(&jz_cim->preview_wait_queue);
	//cim_fb_alloc();

	if ((ret = request_irq(IRQ_CIM, cim_irq_handler, IRQF_DISABLED, CIM_NAME, jz_cim))) {
		dprintk(KERN_ERR "request_irq return error, ret=%d\n", ret);
		dprintk(KERN_ERR "CIM could not get IRQ\n");
		return ret;
	}

	ret = misc_register(&cim_dev);
	if (ret < 0) {
		return ret;
	}

	printk("Virtual Driver of JZ CIM registered\n");
	return 0;
}

static void __exit cim_exit(void)
{
	//cim_fb_free();
	kfree(jz_cim);
	misc_deregister(&cim_dev);
}


late_initcall(cim_init);// CIM driver should not be inited before PMU driver inited.
module_exit(cim_exit);
