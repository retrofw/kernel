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

#include <linux/videodev2.h>
#include <media/videobuf-core.h>
#include <media/videobuf-dma-contig.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#include <asm/cacheflush.h>
#include <asm/mipsregs.h>
#include <asm/mipsmtregs.h>
#include <asm/pgtable.h>
#include <asm/irq.h>
#include <asm/thread_info.h>
#include <asm/uaccess.h>
#include <asm/jzsoc.h>

#include "jz_cim_core_v4l2.h"
#include "jz_sensor.h"

MODULE_AUTHOR("Lutts Cao<slcao@ingenic.cn>");
MODULE_DESCRIPTION("Ingenic Camera interface driver");
MODULE_LICENSE("GPL");

/* module parameters */
static int video_nr = -1;       /* video device minor (-1 ==> auto assign) */

static int buf_cnt = -1;	/* frame buffer count */

static int dma_stop = 0;	/* stop dma or not */

#define CIM_MAJOR_VERSION 0
#define CIM_MINOR_VERSION 6
#define CIM_RELEASE 0
#define CIM_VERSION \
	KERNEL_VERSION(CIM_MAJOR_VERSION, CIM_MINOR_VERSION, CIM_RELEASE)

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

static unsigned debug = 0;	//enable debug mode

#define CIM_I_DEBUG

#undef CIM_I_DEBUG

#define dprintk(dev, level, fmt, arg...) \
	v4l2_dbg(level, debug, &dev->v4l2_dev, fmt, ## arg)

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

struct cim_fmt {
	char  *name;
	u32   fourcc;          /* v4l2 format id */
	int   depth;
};

static struct cim_fmt formats[] = {
	{
		.name     = "4:2:2, packed, YUYV",
		.fourcc   = V4L2_PIX_FMT_YUYV,
		.depth    = 16,
	},
	{
		.name     = "4:2:2, packed, UYVY",
		.fourcc   = V4L2_PIX_FMT_UYVY,
		.depth    = 16,
	},
	{
		.name     = "4:2:0, packed, YV12",
		.fourcc   = V4L2_PIX_FMT_YVU420,
		.depth    = 12,
	},
	{
		.name     = "RGB565 (LE)",
		.fourcc   = V4L2_PIX_FMT_RGB565, /* gggbbbbb rrrrrggg */
		.depth    = 16,
	},
	{
		.name     = "RGB565 (BE)",
		.fourcc   = V4L2_PIX_FMT_RGB565X, /* rrrrrggg gggbbbbb */
		.depth    = 16,
	},
	{
		.name     = "RGB555 (LE)",
		.fourcc   = V4L2_PIX_FMT_RGB555, /* gggbbbbb arrrrrgg */
		.depth    = 16,
	},
	{
		.name     = "RGB555 (BE)",
		.fourcc   = V4L2_PIX_FMT_RGB555X, /* arrrrrgg gggbbbbb */
		.depth    = 16,
	},
};

static struct cim_fmt *get_format(struct v4l2_format *f)
{
	struct cim_fmt *fmt;
	unsigned int k;

	for (k = 0; k < ARRAY_SIZE(formats); k++) {
		fmt = &formats[k];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			break;
	}

	if (k == ARRAY_SIZE(formats))
		return NULL;

	return &formats[k];
}

struct sg_to_addr {
	int pos;
	struct scatterlist *sg;
};

/* buffer for one video frame */
struct cim_buffer {
	/* common v4l buffer stuff -- must be first */
	struct videobuf_buffer vb;

	struct cim_fmt        *fmt;
};

struct cim_dmaqueue {
        struct list_head       active;

        /* thread for generating video stream*/
        struct task_struct         *kthread;
        wait_queue_head_t          wq;
        /* Counters to control fps rate */
        int                        frame;
        int                        ini_jiffies;
};

/* supported controls */
static struct v4l2_queryctrl cim_qctrl[] = {
        {
                .id            = V4L2_CID_AUDIO_VOLUME,
                .name          = "Volume",
                .minimum       = 0,
                .maximum       = 65535,
                .step          = 65535/100,
                .default_value = 65535,
                .flags         = V4L2_CTRL_FLAG_SLIDER,
                .type          = V4L2_CTRL_TYPE_INTEGER,
        }, {
                .id            = V4L2_CID_BRIGHTNESS,
                .type          = V4L2_CTRL_TYPE_INTEGER,
                .name          = "Brightness",
                .minimum       = 0,
                .maximum       = 255,
                .step          = 1,
                .default_value = 127,
                .flags         = V4L2_CTRL_FLAG_SLIDER,
        }
};

static LIST_HEAD(cim_devlist);

struct cim_dev {
	int			   sensor_cnt;
	struct cim_reg		   regs;

        struct list_head           cim_devlist;
        struct v4l2_device         v4l2_dev;

        spinlock_t                 slock;
        struct mutex               mutex;

        int                        users;

        /* various device info */
        struct video_device        *vfd;

        struct cim_dmaqueue       vidq;

        /* several counters */
        int                        h, m, s, ms;
        unsigned long              jiffies;
        char                       timestr[13];

        int                        mv_count;    /* controls bars movement */

        /* input number */
        int                        input;

        /* control 'registers' */
        int                        qctl_regs[ARRAY_SIZE(cim_qctrl)];

	void 			   *priv;	/* point to struct cim_fh -- by ylyuan */
};

struct cim_fh {
	struct cim_dev            *dev;

	/* video capture */
	struct cim_fmt            *fmt;
	unsigned int               width, height;
	struct videobuf_queue      vb_vidq;

	enum v4l2_buf_type         type;
	int			   input; 	/* Input Number on bars */
};

static struct cim_dev *jz_cim;

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

static struct cim_desc dma_desc[VIDEO_MAX_FRAME] __attribute__ ((aligned (VIDEO_MAX_FRAME * sizeof(struct cim_desc))));

#define is_sep()	( jz_cim->regs.cfg & CIM_CFG_SEP )
#define is_yuv422()	( (jz_cim->regs.cfg & CIM_CFG_DF_MASK) == CIM_CFG_DF_YUV422 )
/*====================================================================================
 * sensor muti-support
 *===================================================================================*/

static LIST_HEAD(sensor_list);
static DEFINE_MUTEX(sensor_lock);
static struct camera_sensor_desc *cur_desc __read_mostly = NULL;

/*==========================================================================
 * File operations
 *========================================================================*/

static int cim_open(struct file *filp);
static int cim_release(void);
static ssize_t cim_read(struct file *file, char __user *data, size_t count, loff_t *ppos);
static int cim_mmap(struct file *file, struct vm_area_struct *vma);

static int cim_set_function(int function,void *cookie);

static int cim_set_resolution(struct resolution_info param,int state);

int camera_sensor_register(struct camera_sensor_desc *desc)
{
	if (desc == NULL)
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
	list_add_tail(&desc->list, &sensor_list);
	mutex_unlock(&sensor_lock);

	desc->ops->sensor_set_power(1);

	return 0;
}

void cim_scan_sensor(void)
{
	struct camera_sensor_desc *si;
	struct list_head *tmp;

	mutex_lock(&sensor_lock);
	list_for_each_entry(si, &sensor_list, list) {
		printk("scan sensor: %s\n", si->name);

		if(si->ops->camera_sensor_probe() != 0) {
			tmp = si->list.prev;
			list_del(&si->list);
			si = list_entry(tmp, struct camera_sensor_desc,list);
			printk("fail to probe scan sensor %s\n", si->name);
		}
	}

	mutex_unlock(&sensor_lock);
}

void sensors_make_default(void)
{
	struct camera_sensor_desc *si;

	printk("now we has:\n");

	mutex_lock(&sensor_lock);

	cur_desc = NULL;
	list_for_each_entry(si, &sensor_list, list) {
		si->sensor_id = jz_cim->sensor_cnt;
		jz_cim->sensor_cnt++;
		printk("sensor_name:%s \t id:%d\n", si->name, si->sensor_id);
		cur_desc = si;
	}

	printk("default is %s\n", cur_desc->name);
	mutex_unlock(&sensor_lock);
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
	printk("REG_CIM_YFA \t= \t0x%08x\n", REG_CIM_YFA);
	printk("REG_CIM_YCMD \t= \t0x%08x\n", REG_CIM_YCMD);
	printk("REG_CIM_CBFA \t= \t0x%08x\n", REG_CIM_CBFA);
	printk("REG_CIM_CBCMD \t= \t0x%08x\n", REG_CIM_CBCMD);
	printk("REG_CIM_CRFA \t= \t0x%08x\n", REG_CIM_CRFA);
	printk("REG_CIM_CRCMD \t= \t0x%08x\n", REG_CIM_CRCMD);
#endif
}

#if 1
static void cim_print_desc(struct cim_desc *frame_desc)
{
	int i = 0;
	//printk("cim_tran_buf_id%x\n",cim_tran_buf_id);
	//printk("data_ready_buf_id=0x%x\n",data_ready_buf_id);

//	for(i = 0; i < SWAP_NR; i++)
//	{
		printk("=============%p=============\n", &frame_desc[i]);
		printk("cim_nextdesc	= 0x%08x\n", frame_desc[i].nextdesc);
		printk("cim_framebuf	= 0x%08x\n", frame_desc[i].framebuf);
		printk("cim_frameid	= 0x%08x\n", frame_desc[i].frameid);
		printk("cim_dmacmd	= 0x%08x\n", frame_desc[i].dmacmd);
		if (is_sep()) {
			printk("cim_cb_frame	= 0x%08x\n", frame_desc[i].cb_frame);
			printk("cim_cb_len	= 0x%08x\n", frame_desc[i].cb_len);
			printk("cim_cr_frame	= 0x%08x\n", frame_desc[i].cr_frame);
			printk("cim_cr_len	= 0x%08x\n", frame_desc[i].cr_len);
		}
		printk("========================================\n");
//	}

	//printk("preview_working_buf=0x%x\n",preview_working_buf);
	//printk("recorder_prepare_buf=0x%x\n",recorder_prepare_buf);
	//printk("recorder_working_buf=0x%x\n\n",recorder_working_buf);
}
#endif

static int cim_set_dma(struct videobuf_buffer *vbuf)
{
	struct cim_desc *desc = NULL;

	if (vbuf == NULL)
		return -EINVAL;

	dprintk(jz_cim, 1, "%s L%d: i=%d, vbuf=%p\n", __func__, __LINE__, vbuf->i, vbuf);

	desc = &dma_desc[vbuf->i];

	//init dma descriptor
	desc->nextdesc = virt_to_phys(&dma_desc[vbuf->i + 1]);
	desc->frameid  = vbuf->i;
	desc->framebuf = videobuf_to_dma_contig(vbuf);
	desc->dmacmd   = (vbuf->size>>2) | dmacmd_intr_flag | CIM_CMD_OFRCV;	//pack mode

	desc->dmacmd |= CIM_CMD_STOP;

//	cim_print_desc(desc);

	dma_cache_wback_inv((unsigned long)desc, sizeof(struct cim_desc));

	return 0;
}

static void cim_enable_dma(struct videobuf_buffer *vbuf)		//by ylyuan
{
	struct cim_desc *desc = NULL;
	int idx = 0;

//	for (int i = 0; i < buf_cnt; i++)
//		cim_print_desc(&dma_desc[i]);

	if (vbuf == NULL)
		idx = 0;
	else
		idx = vbuf->i;

	dprintk(jz_cim, 1, "==>%s L%d: buf[%d]\n", __func__, __LINE__, idx);

	desc = &dma_desc[idx];

	dma_cache_wback_inv((unsigned long)&dma_desc[0], buf_cnt * sizeof(struct cim_desc));

	//enable cim
	__cim_set_da(virt_to_phys(desc));
	__cim_clear_state();	// clear state register
	__cim_reset_rxfifo();	// resetting rxfifo
	__cim_unreset_rxfifo();
	__cim_enable_dma();
	__cim_enable();

//	cim_print_regs();
}

/*==========================================================================
 * CIM Module operations
 *========================================================================*/
#define V4L2_FRM_FMT_SEP        v4l2_fourcc('F', 'R', 'M', 'S')         /* separated frame format */
#define V4L2_FRM_FMT_MB         v4l2_fourcc('F', 'R', 'M', 'M')         /* macro block for YUV420 */
#define V4L2_FRM_FMT_PACK       v4l2_fourcc('F', 'R', 'M', 'P')         /* packaged frame format */

static void cim_set_fmt(struct cim_fmt *fmt, __u32 mode)
{
	printk("==>%s L%d, mode=%s\n", __func__, __LINE__, (char *)&mode);
	//set input format
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

	//pack mode or not
	switch (mode) {
	case V4L2_FRM_FMT_SEP:
	case V4L2_FRM_FMT_MB:
		__cim_enable_sep();
		break;
	case V4L2_FRM_FMT_PACK:
		__cim_disable_sep();
		break;
	default:
		break;
	}
#endif

	//set output format
	switch (fmt->fourcc) {
	case V4L2_PIX_FMT_YUYV:	//"YUYV" -> yuv422
		__cim_enable_bypass_func();
		//__cim_disable_bypass_func();
		break;
	case V4L2_PIX_FMT_YUV420:	//"YU12" -> yuv420
		break;
	default:
		printk(KERN_ERR "==>%s L%d: wrong format!\n", __func__, __LINE__);
		break;
	}
}

static void cim_config(struct cim_reg *c)
{
	REG_CIM_CFG  = c->cfg;
	REG_CIM_CTRL = c->ctrl;
	REG_CIM_SIZE = c->size;
	REG_CIM_OFFSET = c->offs;

//	cim_set_fmt(&formats[0], 0);

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

//	cim_print_regs();
}

static void cim_init_config(struct camera_sensor_desc *desc)
{
	if(desc == NULL)
		return;

	memset(&jz_cim->regs, 0, sizeof(jz_cim->regs));

	jz_cim->regs.cfg = cur_desc->cfg_info.configure_register;
	jz_cim->regs.cfg &= ~CIM_CFG_DMA_BURST_TYPE_MASK;
#if defined(CONFIG_SOC_JZ4760B) || defined(CONFIG_SOC_JZ4770)
	jz_cim->regs.cfg |= CIM_CFG_DMA_BURST_INCR32  | (0<<CIM_CFG_RXF_TRIG_BIT);// (n+1)*burst = 2*16 = 32 <64
	jz_cim->regs.ctrl =  CIM_CTRL_DMA_SYNC | CIM_CTRL_FRC_1;
#elif defined(CONFIG_SOC_JZ4760)
	jz_cim->regs.cfg |= CIM_CFG_DMA_BURST_INCR16;
	jz_cim->regs.ctrl =  CIM_CTRL_DMA_SYNC | CIM_CTRL_FRC_1 | (1<<CIM_CTRL_RXF_TRIG_BIT);// (n+1)*burst = 2*16 = 32 <64
#else
	jz_cim->regs.cfg |= CIM_CFG_DMA_BURST_INCR8;
	jz_cim->regs.ctrl = CIM_CTRL_FRC_1 | CIM_CTRL_RXF_TRIG_8 | CIM_CTRL_FAST_MODE; // 16 < 32
#endif

	if (desc->bus_width > 8) {
		jz_cim->regs.cfg |= CIM_CFG_EXPAND_MODE;
		jz_cim->regs.cfg &= ~CIM_CFG_BW_MASK;
		jz_cim->regs.cfg |= (desc->bus_width - 9) << CIM_CFG_BW_LSB;

		jz_cim->regs.cfg &= ~CIM_CFG_SEP;	//pack mode
	} else {
		jz_cim->regs.cfg &= ~CIM_CFG_EXPAND_MODE;

//		jz_cim->regs.cfg |= CIM_CFG_SEP;
	}
}

inline void cim_start(void)
{
	dma_stop = 0;

	cim_power_on();
	cur_desc->ops->sensor_set_power(0);
}

static void cim_stop(void)
{
	dma_stop = 1;

	__cim_disable();
	__cim_clear_state();

	cur_desc->ops->sensor_set_power(1);
	cim_power_off();
}

static int cim_device_init(void)
{
	cim_stop();

	cim_start();	//start cim clk

	cur_desc->ops->sensor_init();

	//set sensor working mode
	cim_set_function(0, NULL);	//preview mode

	cim_init_config(cur_desc);
	cim_config(&jz_cim->regs);

	return 0;
}

/*==========================================================================
 * Interrupt handler
 *========================================================================*/

static irqreturn_t cim_irq_handler2(int irq, void *data)
{
	struct cim_dev *dev		= (struct cim_dev *)data;
//	struct cim_dmaqueue *vidq	= &dev->vidq;
	struct cim_fh *fh		= (struct cim_fh *)dev->priv;
	struct videobuf_queue *q	= &fh->vb_vidq;
	struct videobuf_buffer *buf 	= NULL;
	u32 state, state_back;
	unsigned long flags;
	int iid = 0;

	state = state_back = REG_CIM_STATE;

	if (state & CIM_STATE_DMA_SOF) {
		state &= ~CIM_STATE_DMA_SOF;
		iprintk("sof intrrupt occured\n");
	}

	if (state & CIM_STATE_DMA_STOP) {
		__cim_disable();
		__cim_reset_rxfifo();
		__cim_unreset_rxfifo();
		__cim_clear_state();	// clear state register

		state &= ~CIM_STATE_DMA_STOP;
		iprintk("stop intrrupt occured\n");

	//	return IRQ_HANDLED;
	}

	if (state & CIM_STATE_VDD) {
		state &= ~CIM_STATE_VDD;
		iprintk("cim disable done!\n");
	}

	if (state & CIM_STATE_RXF_TRIG) {
		state &= ~CIM_STATE_RXF_TRIG;
		iprintk("rx trig reached!\n");
	}

#if !defined(CONFIG_SOC_JZ4750)
	if (state & CIM_STATE_DMA_EEOF) {
		state &= ~CIM_STATE_DMA_EEOF;
		iprintk("eeof intrrupt occured!\n");
	}
#endif

	if (state & CIM_STATE_DMA_EOF) {

/*		__cim_disable();
		__cim_reset_rxfifo();
		__cim_unreset_rxfifo();
		__cim_clear_state();	// clear state register
*/
		spin_lock_irqsave(&dev->slock, flags);

		iid = __cim_get_iid();

		state &= ~CIM_STATE_DMA_EOF;
		iprintk("eof intrrupt occured!\n");

		buf = q->bufs[iid];
		iprintk("==>%s L%d: buf[%d]=%p\n", __func__, __LINE__, iid, buf);
		if (buf != NULL) {
			buf->state = VIDEOBUF_DONE;
			wake_up(&buf->done);
		}
#if 0
		//setup future dma
		if (!list_empty(&q->stream)) {
			buf = list_entry(q->stream.next,
					 struct videobuf_buffer, stream);
			cim_set_dma(buf);
			if (!dma_stop)
				cim_enable_dma(buf);
		}
#endif
		spin_unlock_irqrestore(&dev->slock, flags);

		return IRQ_HANDLED;
	}

	state = state_back;
	if ( (state & CIM_STATE_RXF_OF)
#if defined(CONFIG_SOC_JZ4760B) || defined(CONFIG_SOC_JZ4770)
	     || (state & CIM_STATE_Y_RF_OF) ||
	     (state & CIM_STATE_CB_RF_OF) ||
	     (state & CIM_STATE_CR_RF_OF)
#endif
	     ) {
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

/* ------------------------------------------------------------------
	Videobuf operations
   ------------------------------------------------------------------*/

#define vid_limit 	16	//capture memory limit in megabytes
static int
buffer_setup(struct videobuf_queue *vq, unsigned int *count, unsigned int *size)
{
	struct cim_fh  *fh = vq->priv_data;
	struct cim_dev *dev  = fh->dev;

	*size = fh->width*fh->height*2;

	if (0 == *count)
		*count = 32;

	while (*size * *count > vid_limit * 1024 * 1024)
		(*count)--;

	dprintk(dev, 1, "%s, count=%d, size=%d\n", __func__,
		*count, *size);

	buf_cnt = *count;

	dprintk(dev, 1, "%s L%d: width=%d, height=%d, size=%d, count=%d\n", __func__, __LINE__, fh->width, fh->height, *size, *count);

	return 0;
}

static void free_buffer(struct videobuf_queue *vq, struct cim_buffer *buf)
{
	struct cim_fh  *fh = vq->priv_data;
	struct cim_dev *dev  = fh->dev;
	struct videobuf_buffer *vb = &buf->vb;

	dprintk(dev, 1, "%s, state: %i\n", __func__, buf->vb.state);

//	printk("==>%s L%d: vb=%p, state=%i\n", __func__, __LINE__, vb, buf->vb.state);

	if (in_interrupt())
		BUG();

        /* This waits until this buffer is out of danger, i.e., until it is no
         * longer in STATE_QUEUED or STATE_ACTIVE */
        videobuf_waiton(vb, 0, 0);
        videobuf_dma_contig_free(vq, vb);

	dprintk(dev, 1, "free_buffer: freed\n");
	buf->vb.state = VIDEOBUF_NEEDS_INIT;
}

#define norm_maxw() 1024
#define norm_maxh() 768
static int
buffer_prepare(struct videobuf_queue *vq, struct videobuf_buffer *vb,
						enum v4l2_field field)
{
	struct cim_fh     *fh  = vq->priv_data;
	struct cim_dev    *dev = fh->dev;
	struct cim_buffer *buf = container_of(vb, struct cim_buffer, vb);
	int rc;

	dprintk(dev, 1, "%s, field=%d\n", __func__, field);
	dprintk(dev, 1, "==>%s L%d: buf[%d]=%p\n", __func__, __LINE__, vb->i, vb);

	BUG_ON(NULL == fh->fmt);

	if (fh->width  < 48 || fh->width  > norm_maxw() ||
	    fh->height < 32 || fh->height > norm_maxh())
		return -EINVAL;

	buf->vb.size = fh->width*fh->height*2;
	if (0 != buf->vb.baddr  &&  buf->vb.bsize < buf->vb.size)
		return -EINVAL;

	/* These properties only change when queue is idle, see s_fmt */
	buf->fmt       = fh->fmt;
	buf->vb.width  = fh->width;
	buf->vb.height = fh->height;
	buf->vb.field  = field;

	if (VIDEOBUF_NEEDS_INIT == buf->vb.state) {
		rc = videobuf_iolock(vq, &buf->vb, NULL);
		if (rc < 0)
			goto fail;
	}

	buf->vb.state = VIDEOBUF_PREPARED;

	return 0;

fail:
	free_buffer(vq, buf);
	return rc;
}

static void
buffer_queue(struct videobuf_queue *vq, struct videobuf_buffer *vb)
{
	struct cim_buffer    *buf  = container_of(vb, struct cim_buffer, vb);
	struct cim_fh        *fh   = vq->priv_data;
	struct cim_dev       *dev  = fh->dev;
	struct cim_dmaqueue *vidq = &dev->vidq;

	dprintk(dev, 1, "%s\n", __func__);

	buf->vb.state = VIDEOBUF_QUEUED;
	list_add_tail(&buf->vb.queue, &vidq->active);

	cim_set_dma(vb);	//by ylyuan
}

static void buffer_release(struct videobuf_queue *vq,
			   struct videobuf_buffer *vb)
{
	struct cim_buffer   *buf  = container_of(vb, struct cim_buffer, vb);
	struct cim_fh       *fh   = vq->priv_data;
	struct cim_dev      *dev  = (struct cim_dev *)fh->dev;

	dprintk(dev, 1, "%s\n", __func__);

	free_buffer(vq, buf);
}

static struct videobuf_queue_ops cim_video_qops = {
	.buf_setup      = buffer_setup,
	.buf_prepare    = buffer_prepare,
	.buf_queue      = buffer_queue,
	.buf_release    = buffer_release,
};

/* ------------------------------------------------------------------
	IOCTL vidioc handling
   ------------------------------------------------------------------*/
static int vidioc_querycap(struct file *file, void  *priv,
					struct v4l2_capability *cap)
{
	struct cim_fh  *fh  = priv;
	struct cim_dev *dev = fh->dev;

	strcpy(cap->driver, "cim");
	strcpy(cap->card, "cim");
	strlcpy(cap->bus_info, dev->v4l2_dev.name, sizeof(cap->bus_info));
	cap->version = CIM_VERSION;
	cap->capabilities =	V4L2_CAP_VIDEO_CAPTURE |
				V4L2_CAP_STREAMING     |
				V4L2_CAP_READWRITE;
	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void  *priv,
					struct v4l2_fmtdesc *f)
{
	struct cim_fmt *fmt;

	if (f->index >= ARRAY_SIZE(formats))
		return -EINVAL;

	fmt = &formats[f->index];

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct cim_fh *fh = priv;

	f->fmt.pix.width        = fh->width;
	f->fmt.pix.height       = fh->height;
	f->fmt.pix.field        = fh->vb_vidq.field;
	f->fmt.pix.pixelformat  = fh->fmt->fourcc;
	f->fmt.pix.bytesperline =
		(f->fmt.pix.width * fh->fmt->depth) >> 3;
	f->fmt.pix.sizeimage =
		f->fmt.pix.height * f->fmt.pix.bytesperline;

	return (0);
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
			struct v4l2_format *f)
{
	struct cim_fh  *fh  = priv;
	struct cim_dev *dev = fh->dev;
	struct cim_fmt *fmt;
	enum v4l2_field field;
	unsigned int maxw, maxh;
	struct resolution_info res;

	fmt = get_format(f);
	if (!fmt) {
		dprintk(dev, 1, "Fourcc format (0x%08x) invalid.\n",
			f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	field = f->fmt.pix.field;

	if (field == V4L2_FIELD_ANY) {
		field = V4L2_FIELD_NONE;
	} else if (V4L2_FIELD_NONE != field) {
//		field = V4L2_FIELD_INTERLACED;
//	} else if (V4L2_FIELD_INTERLACED != field) {
		dprintk(dev, 1, "Field type invalid.\n");
		return -EINVAL;
	}

	maxw  = norm_maxw();
	maxh  = norm_maxh();

	f->fmt.pix.field = field;
	v4l_bound_align_image(&f->fmt.pix.width, 48, maxw, 2,
			      &f->fmt.pix.height, 32, maxh, 0, 0);

	cim_set_fmt(fmt, f->fmt.pix.priv);	//change format (yuv, rgb, raw...) by ylyuan
	res.width = f->fmt.pix.width;
	res.height = f->fmt.pix.height;
	res.bpp = fmt->depth;
	cim_set_resolution(res, 1);

	f->fmt.pix.bytesperline =
		(f->fmt.pix.width * fmt->depth) >> 3;
	f->fmt.pix.sizeimage =
		f->fmt.pix.height * f->fmt.pix.bytesperline;

	return 0;
}

/*FIXME: This seems to be generic enough to be at videodev2 */
static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct cim_fh *fh = priv;
	struct videobuf_queue *q = &fh->vb_vidq;

	int ret = vidioc_try_fmt_vid_cap(file, fh, f);
	if (ret < 0)
		return ret;

	mutex_lock(&q->vb_lock);

	if (videobuf_queue_is_busy(&fh->vb_vidq)) {
		dprintk(fh->dev, 1, "%s queue busy\n", __func__);
		ret = -EBUSY;
		goto out;
	}

	fh->fmt           = get_format(f);
	fh->width         = f->fmt.pix.width;
	fh->height        = f->fmt.pix.height;
	fh->vb_vidq.field = f->fmt.pix.field;
	fh->type          = f->type;

	ret = 0;
out:
	mutex_unlock(&q->vb_lock);

	return ret;
}

static int vidioc_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *p)
{
	struct cim_fh  *fh = priv;

	return (videobuf_reqbufs(&fh->vb_vidq, p));
}

static int vidioc_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct cim_fh  *fh = priv;

	return (videobuf_querybuf(&fh->vb_vidq, p));
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct cim_fh *fh = priv;

	return (videobuf_qbuf(&fh->vb_vidq, p));
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct cim_fh  *fh = priv;
	struct videobuf_queue *q = &fh->vb_vidq;
	struct videobuf_buffer *buf = NULL;
	int ret = 0;

	ret = videobuf_dqbuf(&fh->vb_vidq, p, file->f_flags & O_NONBLOCK);
	if (ret)
		return ret;

	//setup future dma
	if (!list_empty(&q->stream)) {
		buf = list_entry(q->stream.next,
				 struct videobuf_buffer, stream);
		cim_set_dma(buf);
		if (!dma_stop)
			cim_enable_dma(buf);
	}

	return ret;
}

#ifdef CONFIG_VIDEO_V4L1_COMPAT
static int vidiocgmbuf(struct file *file, void *priv, struct video_mbuf *mbuf)
{
	struct cim_fh  *fh = priv;

	return videobuf_cgmbuf(&fh->vb_vidq, mbuf, 8);
}
#endif

static int vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct cim_fh  *fh = priv;
	int ret  = 0;

	if (fh->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (i != fh->type)
		return -EINVAL;

	ret = videobuf_streamon(&fh->vb_vidq);

	if (ret == 0)
		cim_enable_dma(NULL);

	return ret;
}

static int vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct cim_fh  *fh = priv;

	if (fh->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (i != fh->type)
		return -EINVAL;

	cim_stop();

	return videobuf_streamoff(&fh->vb_vidq);
}

static int vidioc_s_std(struct file *file, void *priv, v4l2_std_id *i)
{
	return 0;
}

/* only one input in this sample driver */
static int vidioc_enum_input(struct file *file, void *priv,
				struct v4l2_input *inp)
{
	if (inp->index > 0)
		return -EINVAL;

	inp->type = V4L2_INPUT_TYPE_CAMERA;
//	inp->std = V4L2_STD_525_60;
	sprintf(inp->name, "Camera %u", inp->index);

	return (0);
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	struct cim_fh *fh = priv;
	struct cim_dev *dev = fh->dev;

	*i = dev->input;

	return (0);
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	struct cim_fh *fh = priv;
	struct cim_dev *dev = fh->dev;

	if (i > 0)
		return -EINVAL;

	dev->input = i;

	return (0);
}

static int vidioc_queryctrl(struct file *file, void *priv,
			    struct v4l2_queryctrl *qc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cim_qctrl); i++)
		if (qc->id && qc->id == cim_qctrl[i].id) {
			memcpy(qc, &(cim_qctrl[i]),
				sizeof(*qc));
			return (0);
		}

	return -EINVAL;
}

static int vidioc_g_ctrl(struct file *file, void *priv,
			 struct v4l2_control *ctrl)
{
	struct cim_fh *fh = priv;
	struct cim_dev *dev = fh->dev;
	int i;

	for (i = 0; i < ARRAY_SIZE(cim_qctrl); i++)
		if (ctrl->id == cim_qctrl[i].id) {
			ctrl->value = dev->qctl_regs[i];
			return 0;
		}

	return -EINVAL;
}
static int vidioc_s_ctrl(struct file *file, void *priv,
				struct v4l2_control *ctrl)
{
	struct cim_fh *fh = priv;
	struct cim_dev *dev = fh->dev;
	int i;

	for (i = 0; i < ARRAY_SIZE(cim_qctrl); i++)
		if (ctrl->id == cim_qctrl[i].id) {
			if (ctrl->value < cim_qctrl[i].minimum ||
			    ctrl->value > cim_qctrl[i].maximum) {
				return -ERANGE;
			}
			dev->qctl_regs[i] = ctrl->value;
			return 0;
		}
	return -EINVAL;
}

/* ------------------------------------------------------------------
	File operations for the device
   ------------------------------------------------------------------*/

static atomic_t jzcim_opened = ATOMIC_INIT(1); /* initially not opened */

static int cim_open(struct file *filp)
{
	struct cim_dev *dev = video_drvdata(filp);
	struct cim_fh *fh = NULL;
	int retval = 0;

	if (jz_cim->sensor_cnt <= 0)
		return -ENODEV;

	if (! atomic_dec_and_test (&jzcim_opened)) {
		atomic_inc(&jzcim_opened);
		return -EBUSY; /* already open */
	}

	cim_device_init();	//start cim clk

	mutex_lock(&jz_cim->mutex);
	jz_cim->users++;

	if (jz_cim->users > 1) {
		jz_cim->users--;
		mutex_unlock(&jz_cim->mutex);
		return -EBUSY;
	}

	dprintk(dev, 1, "open /dev/video%d type=%s users=%d\n", jz_cim->vfd->num,
		v4l2_type_names[V4L2_BUF_TYPE_VIDEO_CAPTURE], jz_cim->users);

	/* allocate + initialize per filehandle data */
	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (NULL == fh) {
		jz_cim->users--;
		retval = -ENOMEM;
	}
	mutex_unlock(&jz_cim->mutex);

	if (retval)
		return retval;

	filp->private_data = fh;
	fh->dev      = dev;
	dev->priv    = fh;	//by ylyuan

	fh->type     = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fh->fmt      = &formats[0];
	fh->width    = 320;
	fh->height   = 240;

	videobuf_queue_dma_contig_init(&fh->vb_vidq, &cim_video_qops,
			NULL, &jz_cim->slock, fh->type, V4L2_FIELD_NONE,
			sizeof(struct cim_buffer), fh);
	
	return 0;
}

static int cim_release(void)
{
	struct cim_dev *dev = jz_cim;

	v4l2_info(&dev->v4l2_dev, "unregistering /dev/video%d\n",
		dev->vfd->num);
	video_unregister_device(dev->vfd);
	v4l2_device_unregister(&dev->v4l2_dev);
	kfree(dev);

	atomic_inc(&jzcim_opened); /* release the device */
	return 0;
}

static ssize_t cim_read(struct file *file, char __user *data, size_t count, loff_t *ppos)
{
	struct cim_fh *fh = file->private_data;

	if (fh->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		return videobuf_read_stream(&fh->vb_vidq, data, count, ppos, 0,
					file->f_flags & O_NONBLOCK);
	}
	return 0;
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
	if (is_yuv422()) {
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

	printk("enable image mode (real size %d x %d) - %d x %d\n",width,height,image_w,image_h);
	return 0;
}

static int cim_disable_image_mode(void)
{
	__cim_disable_size_func();
	//REG_CIM_CTRL &= ~CIM_CTRL_WIN_EN;
	printk("disable image mode\n");
	return 0;
}
#else  /* !CONFIG_SOC_JZ4750 */
#define cim_enable_image_mode(iw, ih, w, h) do {  } while(0)
#define cim_disable_image_mode() do {  } while(0)
#endif

static void param_normalization(struct resolution_info *param)
{
	if((param->bpp != 8) && (param->bpp != 16) && (param->bpp != 24) && (param->bpp != 32) ) {
		if (is_yuv422())
			param->bpp = 16;
		else   /* yuv444 */
			param->bpp = 24;
	}
}

static int cim_set_resolution(struct resolution_info param, int state)
{
	int i,imgsize, framesize, max_capture_index; // words per frame

	if(cur_desc == NULL)
		return -ENODEV;

	param_normalization(&param);

	imgsize = param.width * param.height;
	framesize = (imgsize * param.bpp + 7) >> 3;

	if (framesize > cur_desc->max_capture_size){
		printk("ERROR! Capture size is too large!\n");
		return -EINVAL;
	}

	cur_desc->capture_parm.width = param.width;
	cur_desc->capture_parm.height = param.height;
	cur_desc->capture_parm.bpp = param.bpp;

	cur_desc->preview_parm.width = param.width;
	cur_desc->preview_parm.height = param.height;
	cur_desc->preview_parm.bpp = param.bpp;

	if (state == 0)
		return 0;

	if (cur_desc->ops->sensor_set_resolution== NULL)
		return -ENODEV;
//---------------------------------------------------------------------------------------------

	for(	max_capture_index = 0; 
		max_capture_index < cur_desc->resolution_table_nr;
		max_capture_index++ )
	{
	
		if (cur_desc->resolution_table[max_capture_index].width == cur_desc->max_capture_parm.width
		&& cur_desc->resolution_table[max_capture_index].height == cur_desc->max_capture_parm.height)
			break;
	}

	for(i=max_capture_index; i<cur_desc->resolution_table_nr; i++)
	{

	/*	printk("request capture size: %d x %d\t\tscan capture size: %d x %d\n",
				param.width,
				param.height,
				cur_desc->resolution_table[i].width,
				cur_desc->resolution_table[i].height);	*/
		if(param.width == cur_desc->resolution_table[i].width && param.height == cur_desc->resolution_table[i].height)
		{
		//	printk("found capture size! now capture size is %d x %d\n",param.width,param.height);
			break;
		}
	}

	if(i >= cur_desc->resolution_table_nr)
	{
		printk("size not found! use image mode!\n");
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
			param.bpp,
			0,
			CAMERA_MODE_CAPTURE);

		printk("CAPTURE SIZE:%d x %d\t\tCAPTURE IMAGE:%d x %d\n",
				cur_desc->resolution_table[i].width,
				cur_desc->resolution_table[i].height,
				param.width,
				param.height);
	}
	else
	{
		cim_disable_image_mode();
		cur_desc->ops->sensor_set_resolution (
		param.width,
		param.height,
		param.bpp,
		0,
		CAMERA_MODE_CAPTURE);
	}
#if 0
	//---------------------------------------------------------------------------------------------
	if (jz_cim->regs.packed) {
		capture_desc.dmacmd &= ~CIM_CMD_LEN_MASK;
		capture_desc.dmacmd |= (framesize + 3) >>2;
	} else {
		printk("=======>%s:%d we are test sep!!!\n", __FUNCTION__, __LINE__);
		capture_desc.dmacmd &= ~CIM_CMD_LEN_MASK;
		capture_desc.dmacmd |= (imgsize + 3) >> 2;
		/* FIXME: test YCbCr444 or YCbCr422 */
		if (is_yuv422()) {
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
#endif

	dprintk(jz_cim, 1, "%s L%d: w=%d, h=%d, bpp=%d\n", __func__, __LINE__, param.width, param.height, param.bpp);
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

static unsigned int
cim_poll(struct file *file, struct poll_table_struct *wait)
{
	struct cim_fh        *fh = file->private_data;
	struct cim_dev       *dev = fh->dev;
	struct videobuf_queue *q = &fh->vb_vidq;

	dprintk(dev, 1, "%s\n", __func__);

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != fh->type)
		return POLLERR;

	return videobuf_poll_stream(file, q, wait);
}

static int cim_close(struct file *file)
{
        struct cim_fh         *fh = file->private_data;
        struct cim_dev *dev       = fh->dev;
//	struct cim_dmaqueue *vidq = &dev->vidq;
	int minor = video_devdata(file)->minor;

        videobuf_stop(&fh->vb_vidq);
        videobuf_mmap_free(&fh->vb_vidq);

        kfree(fh);

        mutex_lock(&dev->mutex);
        dev->users--;
        mutex_unlock(&dev->mutex);

	atomic_inc(&jzcim_opened); /* release the device */

        dprintk(dev, 1, "close called (minor=%d, users=%d)\n",
                minor, dev->users);

        return 0;
}

static int cim_mmap(struct file *file, struct vm_area_struct *vma)
{
        struct cim_fh  *fh = file->private_data;
	struct cim_dev *dev = fh->dev;
        int ret;

        dprintk(dev, 1, "mmap called, vma=0x%08lx\n", (unsigned long)vma);

        ret = videobuf_mmap_mapper(&fh->vb_vidq, vma);

        dprintk(dev, 1, "vma start=0x%08lx, size=%ld, ret=%d\n",
                (unsigned long)vma->vm_start,
                (unsigned long)vma->vm_end-(unsigned long)vma->vm_start,
                ret);

        return ret;
}

static const struct v4l2_file_operations cim_fops = {
	.owner		= THIS_MODULE,
	.open           = cim_open,
	.release        = cim_close,
	.read           = cim_read,
	.poll		= cim_poll,
	.ioctl          = video_ioctl2, /* V4L2 ioctl handler */
	.mmap           = cim_mmap,
};

static const struct v4l2_ioctl_ops cim_ioctl_ops = {
	.vidioc_querycap      = vidioc_querycap,
	.vidioc_enum_fmt_vid_cap  = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap     = vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap   = vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap     = vidioc_s_fmt_vid_cap,
	.vidioc_reqbufs       = vidioc_reqbufs,
	.vidioc_querybuf      = vidioc_querybuf,
	.vidioc_qbuf          = vidioc_qbuf,
	.vidioc_dqbuf         = vidioc_dqbuf,
	.vidioc_s_std         = vidioc_s_std,
	.vidioc_enum_input    = vidioc_enum_input,
	.vidioc_g_input       = vidioc_g_input,
	.vidioc_s_input       = vidioc_s_input,
	.vidioc_queryctrl     = vidioc_queryctrl,
	.vidioc_g_ctrl        = vidioc_g_ctrl,
	.vidioc_s_ctrl        = vidioc_s_ctrl,
	.vidioc_streamon      = vidioc_streamon,
	.vidioc_streamoff     = vidioc_streamoff,
#ifdef CONFIG_VIDEO_V4L1_COMPAT
	.vidiocgmbuf          = vidiocgmbuf,
#endif
};

static struct video_device cim_template = {
	.name		= "cim",
	.fops           = &cim_fops,
	.ioctl_ops 	= &cim_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,

//	.tvnorms              = V4L2_STD_525_60,
//	.current_norm         = V4L2_STD_NTSC_M,
};

static int __init cim_init(void)
{
	struct video_device *vfd;
	int ret;

	__gpio_as_cim_10bit();

	jz_cim = kzalloc(sizeof(*jz_cim), GFP_KERNEL);
	if(jz_cim == NULL)
		return -ENOMEM;

	snprintf(jz_cim->v4l2_dev.name, sizeof(jz_cim->v4l2_dev.name),
			"%s-%03d", "jz_cim", 0);
	ret = v4l2_device_register(NULL, &jz_cim->v4l2_dev);
	if (ret)
		goto free_dev;

        /* init video dma queues */
        INIT_LIST_HEAD(&jz_cim->vidq.active);
        init_waitqueue_head(&jz_cim->vidq.wq);

	/* initialize locks */
	spin_lock_init(&jz_cim->slock);
	mutex_init(&jz_cim->mutex);

	/* scan sensor */
	cim_power_on();

	cim_scan_sensor();

	cim_power_off();

	sensors_make_default();

	if(cur_desc == NULL) {
		printk(KERN_ERR "CIM: No sensor found! Exit!\n");
		goto unreg_dev;
	}

	if ((ret = request_irq(IRQ_CIM, cim_irq_handler2, IRQF_DISABLED, CIM_NAME, jz_cim))) {
		printk(KERN_ERR "CIM: fail to request irq, ret=%d\n", ret);
		goto unreg_dev;
	}

	ret = -ENOMEM;
	vfd = video_device_alloc();
	if (!vfd)
		goto unreg_dev;

	*vfd = cim_template;
	vfd->debug = debug;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, video_nr);
	if (ret < 0)
		goto rel_vdev;

	video_set_drvdata(vfd, jz_cim);

	snprintf(vfd->name, sizeof(vfd->name), "%s (%i)",
			cim_template.name, vfd->num);

	if (video_nr >= 0)
		video_nr++;

	jz_cim->vfd = vfd;
	v4l2_info(&jz_cim->v4l2_dev, "V4L2 device registered as /dev/video%d\n",
			vfd->num);

	return 0;

rel_vdev:
	video_device_release(vfd);
unreg_dev:
	v4l2_device_unregister(&jz_cim->v4l2_dev);
free_dev:
	kfree(jz_cim);
	return ret;
}

static void __exit cim_exit(void)
{
	cim_release();
}

late_initcall(cim_init);// CIM driver should not be inited before PMU driver inited.
module_exit(cim_exit);
