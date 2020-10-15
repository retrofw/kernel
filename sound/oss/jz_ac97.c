/*
 *  linux/drivers/sound/jz_ac97.c
 *
 *  Jz On-Chip AC97 audio driver.
 *
 *  Copyright (C) 2005 - 2007, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Because the normal application of AUDIO devices are focused on Little_endian,
 * then we only perform the little endian data format in driver.
 *
 */

#define __NO_VERSION__

#include <linux/init.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sound.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/soundcard.h>
#include <linux/ac97_codec.h>
#include <asm/hardirq.h>
#include <asm/jzsoc.h>
//#include <asm/mach-jz4730/dma.h>
#include "sound_config.h"

#define DMA_ID_AC97_TX	DMA_ID_AIC_TX
#define DMA_ID_AC97_RX	DMA_ID_AIC_RX

/* maxinum number of AC97 codecs connected, AC97 2.0 defined 4 */
#define NR_AC97		2

#define STANDARD_SPEED  48000
#define MAX_RETRY       100

static unsigned int k_8000[] = {
	  0,  42,  85, 128, 170, 213, 
};

static unsigned int reload_8000[] = {
	1, 0, 0, 0, 0, 0, 
};

static unsigned int k_11025[] = {
	  0,  58, 117, 176, 234,  37,  96, 154, 
	213,  16,  74, 133, 192, 250,  53, 112, 
	170, 229,  32,  90, 149, 208,  10,  69, 
	128, 186, 245,  48, 106, 165, 224,  26, 
	 85, 144, 202,   5,  64, 122, 181, 240, 
	 42, 101, 160, 218,  21,  80, 138, 197, 
};

static unsigned int reload_11025[] = {
	1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 
	0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 
	0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 
};

static unsigned int k_16000[] = {
	  0,  85, 170, 
};

static unsigned int reload_16000[] = {
	1, 0, 0, 
};

static unsigned int k_22050[] = {
	  0, 117, 234,  96, 213,  74, 192,  53, 
	170,  32, 149,  10, 128, 245, 106, 224, 
	 85, 202,  64, 181,  42, 160,  21, 138, 
};

static unsigned int reload_22050[] = {
	1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 
	1, 0, 1, 0, 1, 0, 1, 0, 
};

static unsigned int k_24000[] = {
	  0, 128, 
};

static unsigned int reload_24000[] = {
	1, 0, 
};

static unsigned int k_32000[] = {
	  0, 170,  85, 
};

static unsigned int reload_32000[] = {
	1, 0, 1, 
};

static unsigned int k_44100[] = {
	  0, 234, 213, 192, 170, 149, 128, 106, 
	 85,  64,  42,  21, 
};

static unsigned int reload_44100[] = {
	1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
};

static unsigned int k_48000[] = {
	  0, 
};

static unsigned int reload_48000[] = {
	1, 
};


static unsigned int f_scale_counts[8] = {
	6, 48, 3, 24, 2, 3, 12, 1, 
};

static int		jz_audio_rate;
static char		jz_audio_format;
static char		jz_audio_channels;
static int              jz_audio_k;   /* rate expand multiple */
static int              jz_audio_q;   /* rate expand compensate */
static int		jz_audio_count;  /* total count of voice data */
static int		last_jz_audio_count;

static int              jz_audio_fragments;//unused fragment amount
static int              jz_audio_fragstotal;
static int              jz_audio_fragsize;
static int              jz_audio_dma_tran_count;//bytes count of one DMA transfer

static unsigned int	f_scale_count;
static unsigned int	*f_scale_array;
static unsigned int	*f_scale_reload;
static unsigned int	f_scale_idx;

static void (*old_mksound)(unsigned int hz, unsigned int ticks);
extern void (*kd_mksound)(unsigned int hz, unsigned int ticks);
extern void jz_set_dma_block_size(int dmanr, int nbyte);
extern void jz_set_dma_dest_width(int dmanr, int nbit);
extern void jz_set_dma_src_width(int dmanr, int nbit);

static void jz_update_filler(int bits, int channels);

static void Init_In_Out_queue(int fragstotal,int fragsize);
static void Free_In_Out_queue(int fragstotal,int fragsize);

static irqreturn_t
jz_ac97_replay_dma_irq(int irqnr, void *dev_id);
static irqreturn_t
jz_ac97_record_dma_irq(int irqnr, void *dev_id);

static void
(*replay_filler)(unsigned long src_start, int count, int id);
static int
(*record_filler)(unsigned long dst_start, int count, int id);

static struct file_operations jz_ac97_audio_fops;

DECLARE_WAIT_QUEUE_HEAD (rx_wait_queue);
DECLARE_WAIT_QUEUE_HEAD (tx_wait_queue);

struct jz_ac97_controller_info
{
	int io_base;
	int dma1; /* play */
	int dma2; /* record */

	char *name;
	
	int dev_audio;
	struct ac97_codec *ac97_codec[NR_AC97];

	unsigned short ac97_features;

	int opened1;
	int opened2;

        unsigned char *tmp1;  /* tmp buffer for sample conversions */
	unsigned char *tmp2;

	spinlock_t lock;
	spinlock_t ioctllock;
	wait_queue_head_t dac_wait;
	wait_queue_head_t adc_wait;

	int      nextIn;  // byte index to next-in to DMA buffer
	int      nextOut; // byte index to next-out from DMA buffer
	int             count;	// current byte count in DMA buffer
	int             finish; // current transfered byte count in DMA buffer 
	unsigned        total_bytes;	// total bytes written or read
	unsigned        blocks;
	unsigned        error;	// over/underrun

	/* We use two devices, because we can do simultaneous play and record.
	   This keeps track of which device is being used for what purpose;
	   these are the actual device numbers. */
	int dev_for_play;
	int dev_for_record;

	int playing;
	int recording;
	int patched;
	unsigned long rec_buf_size;
	unsigned long playback_buf_size;

#ifdef CONFIG_PM
	struct pm_dev		*pm;
#endif
};

static struct jz_ac97_controller_info *ac97_controller = NULL;

static int jz_readAC97Reg(struct ac97_codec *dev, u8 reg);
static int jz_writeAC97Reg(struct ac97_codec *dev, u8 reg, u16 data);
static u16 ac97_codec_read(struct ac97_codec *codec, u8 reg);
static void ac97_codec_write(struct ac97_codec *codec, u8 reg, u16 data);

#define QUEUE_MAX 2

typedef struct buffer_queue_s {
	int count;
	int *id;
	spinlock_t lock;
} buffer_queue_t;

static unsigned long *out_dma_buf = NULL;
static unsigned long *out_dma_pbuf = NULL;
static unsigned long *out_dma_buf_data_count = NULL;
static unsigned long *in_dma_buf = NULL;
static unsigned long *in_dma_pbuf = NULL;
static unsigned long *in_dma_buf_data_count = NULL;

static buffer_queue_t out_empty_queue;
static buffer_queue_t out_full_queue;
static buffer_queue_t out_busy_queue;

static buffer_queue_t in_empty_queue;
static buffer_queue_t in_full_queue;
static buffer_queue_t in_busy_queue;

static int first_record_call = 0;

static inline int get_buffer_id(struct buffer_queue_s *q)
{
	int r, i;
	unsigned long flags;
	spin_lock_irqsave(&q->lock, flags);
	if (q->count == 0) {
		spin_unlock_irqrestore(&q->lock, flags);
		return -1;
	}
	r = *(q->id + 0);
	for (i=0;i < q->count-1;i++)
		*(q->id + i) = *(q->id + (i+1));
	q->count --;
	spin_unlock_irqrestore(&q->lock, flags);
	return r;
}

static inline void put_buffer_id(struct buffer_queue_s *q, int id)
{
	unsigned long flags;
	spin_lock_irqsave(&q->lock, flags);
	*(q->id + q->count) = id;
	q->count ++;
	spin_unlock_irqrestore(&q->lock, flags);
}

static inline int elements_in_queue(struct buffer_queue_s *q)
{
	int r;
	unsigned long flags;
	spin_lock_irqsave(&q->lock, flags);
	r = q->count;
	spin_unlock_irqrestore(&q->lock, flags);
	return r;
}

/****************************************************************************
 * Architecture related routines
 ****************************************************************************/
static inline
void audio_start_dma(int chan, void *dev_id, unsigned long phyaddr,int count, int mode)
{
	unsigned long flags;
	struct jz_ac97_controller_info * controller =
		(struct jz_ac97_controller_info *) dev_id;
        //for DSP_GETOPTR
	spin_lock_irqsave(&controller->ioctllock, flags);
	jz_audio_dma_tran_count = count;
	spin_unlock_irqrestore(&controller->ioctllock, flags);

	flags = claim_dma_lock();
	disable_dma(chan);
	clear_dma_ff(chan);
	set_dma_mode(chan, mode);
	set_dma_addr(chan, phyaddr);
	if (count == 0) {
		count++;
		printk(KERN_DEBUG "%s: JzSOC DMA controller can't set dma count zero!\n",
		       __FUNCTION__);
	}
	set_dma_count(chan, count);
	enable_dma(chan);
	release_dma_lock(flags);
}

static irqreturn_t
jz_ac97_record_dma_irq (int irq, void *dev_id)
{
	struct jz_ac97_controller_info * controller =
		(struct jz_ac97_controller_info *) dev_id;
	int dma = controller->dma2;
	int id1, id2;
	unsigned long flags;

	disable_dma(dma);
	if (__dmac_channel_address_error_detected(dma)) {
		printk(KERN_DEBUG "%s: DMAC address error.\n", __FUNCTION__);
		__dmac_channel_clear_address_error(dma);
	}
	if (__dmac_channel_transmit_end_detected(dma)) {
		__dmac_channel_clear_transmit_end(dma);

                //for DSP_GETIPTR
		spin_lock_irqsave(&controller->ioctllock, flags);
		controller->total_bytes += jz_audio_dma_tran_count;
		controller->blocks ++;
		spin_unlock_irqrestore(&controller->ioctllock, flags);

		id1 = get_buffer_id(&in_busy_queue);
		put_buffer_id(&in_full_queue, id1);
		wake_up(&rx_wait_queue);
		wake_up(&controller->adc_wait);
		if ((id2 = get_buffer_id(&in_empty_queue)) >= 0) {
			put_buffer_id(&in_busy_queue, id2);

			*(in_dma_buf_data_count + id2) = *(in_dma_buf_data_count + id1);
			audio_start_dma(dma,dev_id,
					*(in_dma_pbuf + id2),
					*(in_dma_buf_data_count + id2),
					DMA_MODE_READ);
		} else {
			in_busy_queue.count = 0;
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t
jz_ac97_replay_dma_irq (int irq, void *dev_id)
{
	struct jz_ac97_controller_info * controller =
		(struct jz_ac97_controller_info *) dev_id;
	int dma = controller->dma1, id;
	unsigned long flags;

	disable_dma(dma);
	if (__dmac_channel_address_error_detected(dma)) {
		printk(KERN_DEBUG "%s: DMAC address error.\n", __FUNCTION__);
		__dmac_channel_clear_address_error(dma);
	}
	if (__dmac_channel_transmit_end_detected(dma)) {
		__dmac_channel_clear_transmit_end(dma);
                //for DSP_GETOPTR
		spin_lock_irqsave(&controller->ioctllock, flags);
		controller->total_bytes += jz_audio_dma_tran_count;
		controller->blocks ++;
		spin_unlock_irqrestore(&controller->ioctllock, flags);
		if ((id = get_buffer_id(&out_busy_queue)) < 0) {
			printk(KERN_DEBUG "Strange DMA finish interrupt for AC97 module\n");
		}

		put_buffer_id(&out_empty_queue, id);
		if ((id = get_buffer_id(&out_full_queue)) >= 0) {
			put_buffer_id(&out_busy_queue, id);
			if(*(out_dma_buf_data_count + id) > 0)	{
			audio_start_dma(dma, dev_id, *(out_dma_pbuf + id),
					*(out_dma_buf_data_count + id),
					DMA_MODE_WRITE);
			}
		} else {
			out_busy_queue.count = 0;
		}
		if (elements_in_queue(&out_empty_queue) > 0) {
			wake_up(&tx_wait_queue);
			wake_up(&controller->dac_wait);
		}
	}
	return IRQ_HANDLED;
}

/*
 * Initialize the onchip AC97 controller
 */
static void jz_ac97_initHw(struct jz_ac97_controller_info *controller)
{
	__ac97_disable();
	__ac97_reset();
	__ac97_enable();

	__ac97_cold_reset_codec();
	/* wait for a long time to let ac97 controller reset completely,
	 * otherwise, registers except ACFR will be clear by reset, can't be
	 * set correctly.
	 */
	udelay(160);

	__ac97_disable_record();
	__ac97_disable_replay();
	__ac97_disable_loopback();

	/* Check the trigger threshold reset value to detect version */
	if (((REG_AIC_FR & AIC_FR_TFTH_MASK) >> AIC_FR_TFTH_BIT) == 8) {
		printk("JzAC97: patched controller detected.\n");
		controller->patched = 1;
	} else {
		printk("JzAC97: standard controller detected.\n");
		controller->patched = 0;
	}

	/* Set FIFO data size. Which shows valid data bits.
	 *
	 */
	__ac97_set_oass(8);
	__ac97_set_iass(8);

	__ac97_set_xs_stereo();
	__ac97_set_rs_stereo();
}

/*
 * Initialize all of in(out)_empty_queue value
 */
static void Init_In_Out_queue(int fragstotal,int fragsize)
{
	int i;
	if(out_dma_buf || in_dma_buf)
		return;
	in_empty_queue.count = fragstotal;
	out_empty_queue.count = fragstotal;

	out_dma_buf = (unsigned long *)kmalloc(sizeof(unsigned long) * fragstotal, GFP_KERNEL);
	if (!out_dma_buf)
		goto all_mem_err;
        out_dma_pbuf = (unsigned long *)kmalloc(sizeof(unsigned long) * fragstotal, GFP_KERNEL);
	if (!out_dma_pbuf)
		goto all_mem_err;
	out_dma_buf_data_count = (unsigned long *)kmalloc(sizeof(unsigned long) * fragstotal, GFP_KERNEL);
	if (!out_dma_buf_data_count)
		goto all_mem_err;
	in_dma_buf = (unsigned long *)kmalloc(sizeof(unsigned long) * fragstotal, GFP_KERNEL);
	if (!in_dma_buf)
		goto all_mem_err;
        in_dma_pbuf = (unsigned long *)kmalloc(sizeof(unsigned long) * fragstotal, GFP_KERNEL);
	if (!in_dma_pbuf)
		goto all_mem_err;
	in_dma_buf_data_count = (unsigned long *)kmalloc(sizeof(unsigned long) * fragstotal, GFP_KERNEL);
	if (!in_dma_buf_data_count)
		goto all_mem_err;
	in_empty_queue.id = (int *)kmalloc(sizeof(int) * fragstotal, GFP_KERNEL);
	if (!in_empty_queue.id)
		goto all_mem_err;
        in_full_queue.id = (int *)kmalloc(sizeof(int) * fragstotal, GFP_KERNEL);
	if (!in_full_queue.id)
		goto all_mem_err;
	in_busy_queue.id = (int *)kmalloc(sizeof(int) * fragstotal, GFP_KERNEL);
	if (!in_busy_queue.id)
		goto all_mem_err;
	out_empty_queue.id = (int *)kmalloc(sizeof(int) * fragstotal, GFP_KERNEL);
	if (!out_empty_queue.id)
		goto all_mem_err;
	out_full_queue.id = (int *)kmalloc(sizeof(int) * fragstotal, GFP_KERNEL);
	if (!out_full_queue.id)
		goto all_mem_err;
	out_busy_queue.id = (int *)kmalloc(sizeof(int) * fragstotal, GFP_KERNEL);
	if (!out_busy_queue.id)
		goto all_mem_err;

	for (i=0;i < fragstotal;i++) {
		*(in_empty_queue.id + i) = i;
		*(out_empty_queue.id + i) = i;
	}

	in_full_queue.count = 0;
	in_busy_queue.count = 0;
	out_busy_queue.count = 0;
	out_full_queue.count = 0;
	/*alloc DMA buffer*/
	for (i = 0; i < jz_audio_fragstotal; i++) {
		*(out_dma_buf + i) = __get_free_pages(GFP_KERNEL | GFP_DMA, get_order(fragsize));
		if (*(out_dma_buf + i) == 0) {
			printk(" can't allocate required DMA(OUT) buffers.\n");
			goto mem_failed_out;
		}
		*(out_dma_pbuf + i) = virt_to_phys((void *)(*(out_dma_buf + i)));
	}
     
	for (i = 0; i < jz_audio_fragstotal; i++) {
		*(in_dma_buf + i) = __get_free_pages(GFP_KERNEL | GFP_DMA, get_order(fragsize));
		if (*(in_dma_buf + i) == 0) {
			printk(" can't allocate required DMA(IN) buffers.\n");
			goto mem_failed_in;
		}
                *(in_dma_pbuf + i) = virt_to_phys((void *)(*(in_dma_buf + i)));
		dma_cache_wback_inv(*(in_dma_buf + i), 4096*8);//fragsize
		*(in_dma_buf + i) = KSEG1ADDR(*(in_dma_buf + i));
	}
	return ;
       
        all_mem_err:
	printk("error:allocate memory occur error!\n");
	return ;

mem_failed_out:
	
	for (i = 0; i < jz_audio_fragstotal; i++) {
		if(*(out_dma_buf + i))
			free_pages(*(out_dma_buf + i), get_order(fragsize));
	}
	return ;

mem_failed_in:
	
	for (i = 0; i < jz_audio_fragstotal; i++) {
		if(*(in_dma_buf + i))
			free_pages(*(in_dma_buf + i), get_order(fragsize));
	}
	return ;

}
static void Free_In_Out_queue(int fragstotal,int fragsize)
{
	int i;        	
	if(out_dma_buf != NULL)
	{
                for (i = 0; i < jz_audio_fragstotal; i++)
                {                
                    if(*(out_dma_buf + i))
                        free_pages(*(out_dma_buf + i), get_order(fragsize));
		    *(out_dma_buf + i) = 0;
                }
		kfree(out_dma_buf);
		out_dma_buf = NULL;
	}
        if(out_dma_pbuf)
	{
		kfree(out_dma_pbuf);
		out_dma_pbuf = NULL;
	}
	if(out_dma_buf_data_count)
	{
		kfree(out_dma_buf_data_count);
		out_dma_buf_data_count = NULL;
	}
	if(in_dma_buf)
	{
                for (i = 0; i < jz_audio_fragstotal; i++)
                {                
                    if(*(in_dma_buf + i))
                        free_pages(*(in_dma_buf + i), get_order(fragsize));
		    *(in_dma_buf + i) = 0;
                }
		kfree(in_dma_buf);
		in_dma_buf = NULL;
	}
	if(in_dma_pbuf)
	{
		kfree(in_dma_pbuf);
		in_dma_pbuf = NULL;
	}
        if(in_dma_buf_data_count)
	{
		kfree(in_dma_buf_data_count);
		in_dma_buf_data_count = NULL;
	}
	if(in_empty_queue.id)
	{
		kfree(in_empty_queue.id);
		in_empty_queue.id = NULL;
	}
	if(in_full_queue.id)
	{
		kfree(in_full_queue.id);
		in_full_queue.id = NULL;
	}
	if(in_busy_queue.id)
	{
		kfree(in_busy_queue.id);
		in_busy_queue.id = NULL;
	}
	if(out_empty_queue.id)
	{
		kfree(out_empty_queue.id);
		out_empty_queue.id = NULL;
	}
	if(out_full_queue.id)
	{
		kfree(out_full_queue.id);
		out_full_queue.id = NULL;
	}
	if(out_busy_queue.id)
	{
		kfree(out_busy_queue.id);
		out_busy_queue.id = NULL;
	}

	in_empty_queue.count = fragstotal;
	out_empty_queue.count = fragstotal;
	in_full_queue.count = 0;
	in_busy_queue.count = 0;
	out_busy_queue.count = 0;
	out_full_queue.count = 0;
	return ;
}

/*
 * Reset everything
 */
static void
jz_ac97_full_reset(struct jz_ac97_controller_info *controller)
{
	jz_ac97_initHw(controller);
}


static void
jz_ac97_mksound(unsigned int hz, unsigned int ticks)
{
//	printk("BEEP - %d %d!\n", hz, ticks);
}

static int jz_audio_set_speed(int dev, int rate)
{
	/* 8000, 11025, 16000, 22050, 24000, 32000, 44100, 48000, 99999999 ? */
	u32 dacp;
	struct ac97_codec *codec=ac97_controller->ac97_codec[0];

	if (rate > 48000)
		rate = 48000;
	if (rate < 8000)
		rate = 8000;


	/* Power down the DAC */
	dacp=ac97_codec_read(codec, AC97_POWER_CONTROL);
	ac97_codec_write(codec, AC97_POWER_CONTROL, dacp|0x0200);
	/* Load the rate; only 48Khz playback available, read always zero */
	if ((ac97_controller->patched) && (ac97_controller->ac97_features & 1))
		ac97_codec_write(codec, AC97_PCM_FRONT_DAC_RATE, rate);
	else
		ac97_codec_write(codec, AC97_PCM_FRONT_DAC_RATE, 48000);

	/* Power it back up */
	ac97_codec_write(codec, AC97_POWER_CONTROL, dacp);

	jz_audio_rate = rate;
	jz_audio_k = STANDARD_SPEED / rate;
	if (rate * jz_audio_k != STANDARD_SPEED) 
		jz_audio_q = rate / ((STANDARD_SPEED / jz_audio_k) - rate );
	else
		jz_audio_q = 0x1fffffff; /* a very big value, don't compensate */

	switch (rate) {
	case 8000:
		f_scale_count	= f_scale_counts[0];
		f_scale_array	= k_8000;
		f_scale_reload	= reload_8000;
		break;
	case 11025:
		f_scale_count	= f_scale_counts[1];
		f_scale_array	= k_11025;
		f_scale_reload	= reload_11025;
		break;
	case 16000:
		f_scale_count	= f_scale_counts[2];
		f_scale_array	= k_16000;
		f_scale_reload	= reload_16000;
		break;
	case 22050:
		f_scale_count	= f_scale_counts[3];
		f_scale_array	= k_22050;
		f_scale_reload	= reload_22050;
		break;
	case 24000:
		f_scale_count	= f_scale_counts[4];
		f_scale_array	= k_24000;
		f_scale_reload	= reload_24000;
		break;
	case 32000:
		f_scale_count	= f_scale_counts[5];
		f_scale_array	= k_32000;
		f_scale_reload	= reload_32000;
		break;
	case 44100:
		f_scale_count	= f_scale_counts[6];
		f_scale_array	= k_44100;
		f_scale_reload	= reload_44100;
		break;
	case 48000:
		f_scale_count	= f_scale_counts[7];
		f_scale_array	= k_48000;
		f_scale_reload	= reload_48000;
		break;
	}
	f_scale_idx	= 0;

	return jz_audio_rate;
}

static int record_fill_1x8_u(unsigned long dst_start, int count, int id)
{
	int cnt = 0;
	unsigned char data;
	volatile unsigned char *s = (unsigned char *)(*(in_dma_buf + id));
	volatile unsigned char *dp = (unsigned char *)dst_start;

	while (count > 0) {
		count -= 2;		/* count in dword */
		if ((jz_audio_count++ % jz_audio_k) == 0) {
			cnt++;
			data = *(s++);
			*(dp ++) = data + 0x80;
			s++;		/* skip the other channel */
		} else {
			s += 2;		/* skip the redundancy */
		}
		if (jz_audio_count - last_jz_audio_count >= jz_audio_q) {
			jz_audio_count++;
			last_jz_audio_count = jz_audio_count;
			count -= 2;
			s += 2;
		}
	}
	return cnt;
}

static int record_fill_2x8_u(unsigned long dst_start, int count, int id)
{
	int cnt = 0;
	unsigned char d1, d2;
	volatile unsigned char *s = (unsigned char *)(*(in_dma_buf + id));
	volatile unsigned char *dp = (unsigned char *)dst_start;

	while (count > 0) {
		count -= 2;
		if ((jz_audio_count++ % jz_audio_k) == 0) {
			cnt += 2;
			d1 = *(s++);
			*(dp ++) = d1 + 0x80;
			d2 = *(s++);
			*(dp ++) = d2 + 0x80;
		} else {
			s += 2;		/* skip the redundancy */
		}
		if (jz_audio_count - last_jz_audio_count >= jz_audio_q * 2) {
			jz_audio_count += 2;
			last_jz_audio_count = jz_audio_count;
			count -= 2;
			s += 2;
		}
	}
	return cnt;
}

static int record_fill_1x16_s(unsigned long dst_start, int count, int id)
{
	int cnt = 0;
	unsigned short d1;
	unsigned short *s = (unsigned short *)(*(in_dma_buf + id));
	unsigned short *dp = (unsigned short *)dst_start;

	while (count > 0) {
		count -= 2;		/* count in dword */
		if ((jz_audio_count++ % jz_audio_k) == 0) {
			cnt += 2;	/* count in byte */
			d1 = *(s++);
			*(dp ++) = d1;
			s++;		/* skip the other channel */
		} else {
			s += 2;		/* skip the redundancy */
		}
		if (jz_audio_count - last_jz_audio_count >= jz_audio_q * 2) {
			jz_audio_count += 2;
			last_jz_audio_count = jz_audio_count;
			count -= 2;
			s += 2;
		}
	}
	return cnt;
}

static int record_fill_2x16_s(unsigned long dst_start, int count, int id)
{
	int cnt = 0;
	unsigned short d1, d2;
	unsigned short *s = (unsigned short *)(*(in_dma_buf + id));
	unsigned short *dp = (unsigned short *)dst_start;

	while (count > 0) {
		count -= 2;		/* count in dword */
		if ((jz_audio_count++ % jz_audio_k) == 0) {
			cnt += 4;	/* count in byte */
			d1 = *(s++);
			*(dp ++) = d1;
			d2 = *(s++);
			*(dp ++) = d2;
		} else
			s += 2;		/* skip the redundancy */

		if (jz_audio_count - last_jz_audio_count >= jz_audio_q * 4) {
			jz_audio_count += 4;
			last_jz_audio_count = jz_audio_count;
			count -= 2;
			s += 2;
		}
	}
	return cnt;
}


static void replay_fill_1x8_u(unsigned long src_start, int count, int id)
{
	int i, cnt = 0;
	unsigned char data;
	unsigned char *s = (unsigned char *)src_start;
	unsigned char *dp = (unsigned char *)(*(out_dma_buf + id));

	while (count > 0) {
		count--;
		jz_audio_count++;
		cnt += jz_audio_k;
		data = *(s++) - 0x80;
		for (i=0;i<jz_audio_k;i++) {
			*(dp ++) = data;
			*(dp ++) = data;
		}
	}
	cnt = cnt * 2;
	*(out_dma_buf_data_count + id) = cnt;
}

static void replay_fill_2x8_u(unsigned long src_start, int count, int id)
{
	int i, cnt = 0;
	unsigned char d1, d2;
	unsigned char *s = (unsigned char *)src_start;
	unsigned char *dp = (unsigned char*)(*(out_dma_buf + id));

	while (count > 0) {
		count -= 2;
		jz_audio_count += 2;
		cnt += 2 * jz_audio_k;
		d1 = *(s++) - 0x80;
		d2 = *(s++) - 0x80;
		for (i=0;i<jz_audio_k;i++) {
			*(dp ++) = d1;
			*(dp ++) = d2;
		}
		if (jz_audio_count - last_jz_audio_count >= jz_audio_q * 2) {
			cnt += 2 * jz_audio_k;
			last_jz_audio_count = jz_audio_count;
			for (i=0;i<jz_audio_k;i++) {
				*(dp ++) = d1;
				*(dp ++) = d2;
			}
		}
	}
	*(out_dma_buf_data_count + id) = cnt;
}

static void replay_fill_1x16_s(unsigned long src_start, int count, int id)
{
	int cnt = 0;
	static short d1, d2, d;
	short *s = (short *)src_start;
	short *dp = (short *)(*(out_dma_buf + id));

	d2 = *s++;
	count -= 2;
	while (count >= 0) {
		if (f_scale_reload[f_scale_idx]) {
			d1 = d2;
			d2 = *s++;
			if (!count)
				break;
			count -= 2;
		}
		d = d1 + (((d2 - d1) * f_scale_array[f_scale_idx]) >> 8);
		*dp++ = d;
		*dp++ = d;
		cnt += 4;
		f_scale_idx ++;
		if (f_scale_idx >= f_scale_count)
			f_scale_idx = 0;
	}
	*(out_dma_buf_data_count + id) = cnt;
}

static void replay_fill_2x16_s(unsigned long src_start, int count, int id)
{
	int cnt = 0;
	static short d11, d12, d21, d22, d1, d2;
	short *s = (short *)src_start;
	short *dp = (short *)(*(out_dma_buf + id));

	d12 = *s++;
	d22 = *s++;
	count -= 4;
	while (count >= 0) {
		register unsigned int kvalue;
		kvalue = f_scale_array[f_scale_idx];
		if (f_scale_reload[f_scale_idx]) {
			d11 = d12;
			d12 = *s++;
			d21 = d22;
			d22 = *s++;
			if (!count)
				break;
			count -= 4;
		}
		d1 = d11 + (((d12 - d11)*kvalue) >> 8);
		d2 = d21 + (((d22 - d21)*kvalue) >> 8);
		*dp++ = d1;
		*dp++ = d2;
		cnt += 4;
		f_scale_idx ++;
		if (f_scale_idx >= f_scale_count)
			f_scale_idx = 0;
	}
	*(out_dma_buf_data_count + id) = cnt;
}

static unsigned int jz_audio_set_format(int dev, unsigned int fmt)
{
	switch (fmt) {
	        case AFMT_U8:
		case AFMT_S16_LE:
			jz_audio_format = fmt;
			jz_update_filler(fmt, jz_audio_channels);
		case AFMT_QUERY:
			break;
	}
	return jz_audio_format;
}

static short jz_audio_set_channels(int dev, short channels)
{
	switch (channels) {
	case 1:
		__ac97_set_xs_stereo();	// always stereo when recording
		__ac97_set_rs_stereo();	// always stereo when recording
		jz_audio_channels = channels;
		jz_update_filler(jz_audio_format, jz_audio_channels);
		break;
	case 2:
		__ac97_set_xs_stereo();
		__ac97_set_rs_stereo();
		jz_audio_channels = channels;
		jz_update_filler(jz_audio_format, jz_audio_channels);
		break;
	case 0:
		break;
	}
	return jz_audio_channels;
}


static void jz_audio_reset(void)
{
	__ac97_disable_replay();
	__ac97_disable_receive_dma();
	__ac97_disable_record();
	__ac97_disable_transmit_dma();
}

static int jz_audio_release(struct inode *inode, struct file *file);
static int jz_audio_open(struct inode *inode, struct file *file);
static int jz_audio_ioctl(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg);
static unsigned int jz_audio_poll(struct file *file,
				  struct poll_table_struct *wait);
static ssize_t jz_audio_write(struct file *file, const char *buffer,
			      size_t count, loff_t *ppos);
static ssize_t jz_audio_read(struct file *file, char *buffer,
			     size_t count, loff_t *ppos);

/* static struct file_operations jz_ac97_audio_fops */
static struct file_operations jz_ac97_audio_fops =
{
	owner:              THIS_MODULE,
	open:               jz_audio_open,
	release:            jz_audio_release,
	write:              jz_audio_write,
	read:               jz_audio_read,
	poll:               jz_audio_poll,
	ioctl:              jz_audio_ioctl
};

/* Read / Write AC97 codec registers */
static inline int jz_out_command_ready(void)
{
	int t2 = 1000;
	int done = 0;

	while (! done && t2-- > 0) {
		if (REG32(AC97_ACSR) & AIC_ACSR_CADT) {
			REG32(AC97_ACSR) &= ~AIC_ACSR_CADT;
			done = 1;
		}
		else
			udelay (1);
	}
	return done;
}
	
static inline int jz_in_status_ready(void)
{
	int t2 = 1000;
	int done = 0;

	while (! done && t2-- > 0) {
		if (REG32(AC97_ACSR) & AIC_ACSR_SADR) {
			REG32(AC97_ACSR) &= ~AIC_ACSR_SADR;
			done = 1;
		}
		else {
			if (REG32(AC97_ACSR) & AIC_ACSR_RSTO) {
				REG32(AC97_ACSR) &= ~AIC_ACSR_RSTO;
				printk(KERN_DEBUG "%s: RSTO receive status timeout.\n",
				       __FUNCTION__);
				done = 0;
				break;
			}
			udelay (1);
		}
	}
	return done;
}

static int jz_readAC97Reg (struct ac97_codec *dev, u8 reg)
{
	u16 value;

	if (reg < 128) {
		u8  ret_reg;
		__ac97_out_rcmd_addr(reg);//output read addr
		if (jz_out_command_ready())//judge if send completely? 
			while (jz_in_status_ready()) {//judge if receive completely?
				ret_reg = __ac97_in_status_addr();//slot1:send addr
				value = __ac97_in_data();
				if (ret_reg == reg)
					return value;
				else {
//					printk(KERN_DEBUG "%s: index (0x%02x)->(0x%02x) 0x%x\n", __FUNCTION__, reg, ret_reg, value);
					return -EINVAL;
				}
			}
	}
	value = __ac97_in_data();
	printk (KERN_DEBUG "timeout while reading AC97 codec (0x%x)\n", reg);
	return -EINVAL;
}

static u16 ac97_codec_read(struct ac97_codec *codec, u8 reg)
{
	int res = jz_readAC97Reg(codec, reg);
	int count = 0;
	while (res == -EINVAL) {
		udelay(1000);
		__ac97_warm_reset_codec();
		udelay(1000);
		res = jz_readAC97Reg(codec, reg);
		count ++;
		if (count > MAX_RETRY){
			printk(KERN_WARNING"After try %d when read AC97 codec 0x%x, can't success, give up operate!!\n",
			       MAX_RETRY, reg);
			break;
		}
	}
	return (u16)res;
}

static int jz_writeAC97Reg (struct ac97_codec *dev, u8 reg, u16 value)
{
	//unsigned long flags;
	int done = 0;

	//save_and_cli(flags);

	__ac97_out_wcmd_addr(reg);
	__ac97_out_data(value);
	if (jz_out_command_ready())
		done = 1;
	else
		printk (KERN_DEBUG "Tiemout AC97 codec write (0x%X<==0x%X)\n", reg, value);
	//restore_flags(flags);
	return done;
}
static void ac97_codec_write(struct ac97_codec *codec, u8 reg, u16 data)
{
	int done = jz_writeAC97Reg(codec, reg, data);
	int count = 0;
	while (done == 0) {
		count ++;
		udelay (2000);
		__ac97_warm_reset_codec();
		udelay(2000);
		done = jz_writeAC97Reg(codec, reg, data);
		if ( count > MAX_RETRY ){
			printk (KERN_DEBUG " After try %d when write AC97 codec (0x%x), can't sucess, give up!! \n", 
				MAX_RETRY, reg);
			break;
		}
	}
}

/* OSS /dev/mixer file operation methods */

static int jz_ac97_open_mixdev(struct inode *inode, struct file *file)
{
	int i;
	int minor = MINOR(inode->i_rdev);
	struct jz_ac97_controller_info *controller = ac97_controller;

	for (i = 0; i < NR_AC97; i++)
		if (controller->ac97_codec[i] != NULL &&
		    controller->ac97_codec[i]->dev_mixer == minor)
			goto match;

	if (!controller)
		return -ENODEV;

 match:
	file->private_data = controller->ac97_codec[i];

	return 0;
}

static int jz_ac97_ioctl_mixdev(struct inode *inode, struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct ac97_codec *codec = (struct ac97_codec *)file->private_data;

	return codec->mixer_ioctl(codec, cmd, arg);
}

static loff_t jz_ac97_llseek(struct file *file, loff_t offset, int origin)
{
	return -ESPIPE;
}

static /*const*/ struct file_operations jz_ac97_mixer_fops = {
	owner:		THIS_MODULE,
	llseek:		jz_ac97_llseek,
	ioctl:		jz_ac97_ioctl_mixdev,
	open:		jz_ac97_open_mixdev,
};

/* AC97 codec initialisation. */
static int __init jz_ac97_codec_init(struct jz_ac97_controller_info *controller)
{
	int num_ac97 = 0;
	int ready_2nd = 0;
	struct ac97_codec *codec;
	unsigned short eid;
	int i = 0;

	if (__ac97_codec_is_low_power_mode()) {
		printk(KERN_DEBUG "AC97 codec is low power mode, warm reset ...\n");
		__ac97_warm_reset_codec();
		udelay(10);
	}
	i = 0;
	while (!__ac97_codec_is_ready()) {
		i++;
		if ( i > 100 ) {
			printk(KERN_WARNING "AC97 codec not ready, failed init ..\n");
			return -ENODEV;
		}
		udelay(10);
	}
	i = 0;

	/* Reset the mixer. */
	for (num_ac97 = 0; num_ac97 < NR_AC97; num_ac97++) {
		if ((codec = kmalloc(sizeof(struct ac97_codec),
				     GFP_KERNEL)) == NULL)
			return -ENOMEM;
		memset(codec, 0, sizeof(struct ac97_codec));

		/* initialize some basic codec information,
		   other fields will be filled
		   in ac97_probe_codec */
		codec->private_data = controller;
		codec->id = num_ac97;

		codec->codec_read = ac97_codec_read;
		codec->codec_write = ac97_codec_write;

		if (ac97_probe_codec(codec) == 0)
			break;

		eid = ac97_codec_read(codec, AC97_EXTENDED_ID);
		if (eid == 0xFFFF) {
			printk(KERN_WARNING "Jz AC97: no codec attached?\n");
			kfree(codec);
			break;
		}

		controller->ac97_features = eid;

		if (!(eid & 0x0001))
			printk(KERN_WARNING "AC97 codec: only 48Khz playback available.\n");
		else {
			printk(KERN_WARNING "AC97 codec: supports variable sample rate.\n");
			/* Enable HPEN: UCB1400 only */
			ac97_codec_write(codec, 0x6a,
					 ac97_codec_read(codec, 0x6a) | 0x40);

			/* Enable variable rate mode */
			ac97_codec_write(codec, AC97_EXTENDED_STATUS, 9);
			ac97_codec_write(codec,
				AC97_EXTENDED_STATUS,
				ac97_codec_read(codec, AC97_EXTENDED_STATUS) | 0xE800);
			/* power up everything, modify this
			   when implementing power saving */
			ac97_codec_write(codec,
				AC97_POWER_CONTROL,
				ac97_codec_read(codec, AC97_POWER_CONTROL) & ~0x7f00);
			/* wait for analog ready */
			for (i=10; i && ((ac97_codec_read(codec, AC97_POWER_CONTROL) & 0xf) != 0xf); i--) {
//				current->state = TASK_UNINTERRUPTIBLE;
//				schedule_timeout(HZ/20);
			}

			if (!(ac97_codec_read(codec, AC97_EXTENDED_STATUS) & 1)) {
				printk(KERN_WARNING "Jz AC97: Codec refused to allow VRA, using 48Khz only.\n");
				controller->ac97_features &= ~1;
			}
		}
   		
		if ((codec->dev_mixer =
		     register_sound_mixer(&jz_ac97_mixer_fops, -1)) < 0) {
			printk(KERN_ERR "Jz AC97: couldn't register mixer!\n");
			kfree(codec);
			break;
		}

		controller->ac97_codec[num_ac97] = codec;

		/* if there is no secondary codec at all, don't probe any more */
		if (!ready_2nd)
			return num_ac97+1;
	}
	return num_ac97;
}

static void jz_update_filler(int format, int channels)
{
#define TYPE(fmt,ch) (((fmt)<<3) | ((ch)&7))	/* up to 8 chans supported. */


	switch (TYPE(format, channels)) {
		default:
			
		case TYPE(AFMT_U8, 1):
			if ((ac97_controller->patched) &&
			    (ac97_controller->ac97_features & 1)) {
				__aic_enable_mono2stereo();
				__aic_enable_unsignadj();
				jz_set_dma_block_size(ac97_controller->dma1, 16);
				jz_set_dma_block_size(ac97_controller->dma2, 16);
			} else {
				__aic_disable_mono2stereo();
				__aic_disable_unsignadj();
				jz_set_dma_block_size(ac97_controller->dma1, 4);
				jz_set_dma_block_size(ac97_controller->dma2, 4);
			}
			replay_filler = replay_fill_1x8_u;
			record_filler = record_fill_1x8_u;
			__ac97_set_oass(8);
			__ac97_set_iass(8);
			jz_set_dma_dest_width(ac97_controller->dma1, 8);
			jz_set_dma_src_width(ac97_controller->dma2, 8);
			break;
		case TYPE(AFMT_U8, 2):
			if ((ac97_controller->patched) &&
			    (ac97_controller->ac97_features & 1)) {
				__aic_enable_mono2stereo();
				__aic_enable_unsignadj();
				jz_set_dma_block_size(ac97_controller->dma1, 16);
				jz_set_dma_block_size(ac97_controller->dma2, 16);
			} else {
				__aic_disable_mono2stereo();
				__aic_disable_unsignadj();
				jz_set_dma_block_size(ac97_controller->dma1, 4);
				jz_set_dma_block_size(ac97_controller->dma2, 4);
			}
			replay_filler = replay_fill_2x8_u;
			record_filler = record_fill_2x8_u;
			__ac97_set_oass(8);
			__ac97_set_iass(8);
			jz_set_dma_dest_width(ac97_controller->dma1, 8);
			jz_set_dma_src_width(ac97_controller->dma2, 8);
			break;
		case TYPE(AFMT_S16_LE, 1):
			if ((ac97_controller->patched) &&
			    (ac97_controller->ac97_features & 1)) {
				__aic_enable_mono2stereo();
				jz_set_dma_block_size(ac97_controller->dma1, 16);
				jz_set_dma_block_size(ac97_controller->dma2, 16);
			} else {
				__aic_disable_mono2stereo();
				jz_set_dma_block_size(ac97_controller->dma1, 4);
				jz_set_dma_block_size(ac97_controller->dma2, 4);
			}
			__aic_disable_unsignadj();
			replay_filler = replay_fill_1x16_s;
			record_filler = record_fill_1x16_s;
			__ac97_set_oass(16);
			__ac97_set_iass(16);

			jz_set_dma_dest_width(ac97_controller->dma1, 16);
			jz_set_dma_src_width(ac97_controller->dma2, 16);

			break;

		case TYPE(AFMT_S16_LE, 2):
			if ((ac97_controller->patched) &&
			    (ac97_controller->ac97_features & 1)) {
				jz_set_dma_block_size(ac97_controller->dma1, 16);
				jz_set_dma_block_size(ac97_controller->dma2, 16);
			} else {
				jz_set_dma_block_size(ac97_controller->dma1, 4);
				jz_set_dma_block_size(ac97_controller->dma2, 4);
			}
			__aic_disable_mono2stereo();
			__aic_disable_unsignadj();
			replay_filler = replay_fill_2x16_s;
			record_filler = record_fill_2x16_s;
			__ac97_set_oass(16);
			__ac97_set_iass(16);
			jz_set_dma_dest_width(ac97_controller->dma1, 16);
			jz_set_dma_src_width(ac97_controller->dma2, 16);
			break;
	}
}

#ifdef CONFIG_PROC_FS

extern struct proc_dir_entry *proc_jz_root;

static int jz_ac97_init_proc(struct jz_ac97_controller_info *controller)
{
	if (!create_proc_read_entry ("ac97", 0, proc_jz_root, 
				     ac97_read_proc, controller->ac97_codec[0]))
		return -EIO;

	return 0;
}

static void jz_ac97_cleanup_proc(struct jz_ac97_controller_info *controller)
{
}

#endif

static void __init attach_jz_ac97(struct jz_ac97_controller_info *controller)
{
	char *name;
	int adev;

	name = controller->name;
	jz_ac97_initHw(controller);

	/* register /dev/audio ? */
	adev = register_sound_dsp(&jz_ac97_audio_fops, -1);
	if (adev < 0)
		goto audio_failed;

	/* initialize AC97 codec and register /dev/mixer */
	if (jz_ac97_codec_init(controller) <= 0)
		goto mixer_failed;

#ifdef CONFIG_PROC_FS
	if (jz_ac97_init_proc(controller) < 0) {
		printk(KERN_ERR "%s: can't create AC97 proc filesystem.\n", name);
		goto proc_failed;
	}
#endif

	controller->tmp1 = (void *)__get_free_pages(GFP_KERNEL, 8);//4
	if (!controller->tmp1) {
		printk(KERN_ERR "%s: can't allocate tmp buffers.\n", controller->name);
		goto tmp1_failed;
	}

	controller->tmp2 = (void *)__get_free_pages(GFP_KERNEL, 8);//4
	if (!controller->tmp2) {
		printk(KERN_ERR "%s: can't allocate tmp buffers.\n", controller->name);
		goto tmp2_failed;
	}

	if ((controller->dma1 = jz_request_dma(DMA_ID_AC97_TX, "audio dac",
						 jz_ac97_replay_dma_irq,
						 IRQF_DISABLED, controller)) < 0) {
		printk(KERN_ERR "%s: can't reqeust DMA DAC channel.\n", name);
		goto dma1_failed;
	}
	if ((controller->dma2 = jz_request_dma(DMA_ID_AC97_RX, "audio adc",
						 jz_ac97_record_dma_irq,
						 IRQF_DISABLED, controller)) < 0) {
		printk(KERN_ERR "%s: can't reqeust DMA ADC channel.\n", name);
		goto dma2_failed;
	}

	printk("Jz On-Chip AC97 controller registered (DAC: DMA%d/IRQ%d, ADC: DMA%d/IRQ%d)\n",
	       controller->dma1, get_dma_done_irq(controller->dma1),
	       controller->dma2, get_dma_done_irq(controller->dma2));

	old_mksound = kd_mksound;   /* see vt.c */
	kd_mksound = jz_ac97_mksound;
	controller->dev_audio = adev;
	return;

 dma2_failed:
	jz_free_dma(controller->dma1);
 dma1_failed:
	free_pages((unsigned long)controller->tmp2, 8);//4
 tmp2_failed:
	free_pages((unsigned long)controller->tmp1, 8);//4
 tmp1_failed:
#ifdef CONFIG_PROC_FS
	jz_ac97_cleanup_proc(controller);
#endif
 proc_failed:
	/* unregister mixer dev */
 mixer_failed:
	unregister_sound_dsp(adev);
 audio_failed:
	return;
}
	
static int __init probe_jz_ac97(struct jz_ac97_controller_info **controller)
{
	if ((*controller = kmalloc(sizeof(struct jz_ac97_controller_info),
				   GFP_KERNEL)) == NULL) {
		printk(KERN_ERR "Jz AC97 Controller: out of memory.\n");
		return -ENOMEM;
	}
	memset( *controller, 0, sizeof(struct jz_ac97_controller_info) );
	(*controller)->name = "Jz AC97 controller";
	(*controller)->opened1 = 0;
	(*controller)->opened2 = 0;

	init_waitqueue_head(&(*controller)->adc_wait);
	init_waitqueue_head(&(*controller)->dac_wait);
	spin_lock_init(&(*controller)->lock);
	init_waitqueue_head(&rx_wait_queue);
	init_waitqueue_head(&tx_wait_queue);

	return 0;
}

static void __exit unload_jz_ac97(struct jz_ac97_controller_info *controller)
{
	int adev = controller->dev_audio;

	jz_ac97_full_reset (controller);

	controller->dev_audio = -1;

	if (old_mksound)
		kd_mksound = old_mksound;     /* Our driver support bell for kb, see vt.c */

#ifdef CONFIG_PROC_FS
	jz_ac97_cleanup_proc(controller);
#endif

	jz_free_dma(controller->dma1);
	jz_free_dma(controller->dma2);

	free_pages((unsigned long)controller->tmp1, 8);//4
	free_pages((unsigned long)controller->tmp2, 8);//4

	if (adev >= 0) {
		//unregister_sound_mixer(audio_devs[adev]->mixer_dev);
		unregister_sound_dsp(controller->dev_audio);
	}
}

#ifdef CONFIG_PM

static int reserve_mastervol, reserve_micvol;
static int reserve_power1, reserve_power2;
static int reserve_power3, reserve_power4;
static int reserve_power5, reserve_power6;

static int jz_ac97_suspend(struct jz_ac97_controller_info *controller, int state)
{       
	struct ac97_codec *codec = controller->ac97_codec[0];

	/* save codec states */
	reserve_mastervol = ac97_codec_read(codec, 0x0002);
	reserve_micvol = ac97_codec_read(codec, 0x000e);

	reserve_power1 = ac97_codec_read(codec, 0x0026);
	reserve_power2 = ac97_codec_read(codec, 0x006c);
	reserve_power3 = ac97_codec_read(codec, 0x005c);
	reserve_power4 = ac97_codec_read(codec, 0x0064);
	reserve_power5 = ac97_codec_read(codec, 0x0066);
	reserve_power6 = ac97_codec_read(codec, 0x006a);

	/* put codec into power-saving mode */
	ac97_codec_write(codec, 0x5c, 0xffff);
	ac97_codec_write(codec, 0x64, 0x0000);
	ac97_codec_write(codec, 0x66, 0x0000);
	ac97_codec_write(codec, 0x6a, 0x0000);

	ac97_codec_write(codec, 0x6c, 0x0030);
	ac97_codec_write(codec, 0x26, 0x3b00);

	ac97_save_state(codec);
	__ac97_disable();

	return 0;
}

static int jz_ac97_resume(struct jz_ac97_controller_info *controller)
{
	struct ac97_codec *codec = controller->ac97_codec[0];

	jz_ac97_full_reset(controller);
	ac97_probe_codec(codec);
	ac97_restore_state(codec);

	ac97_codec_write(codec, 0x0026, reserve_power1);
	ac97_codec_write(codec, 0x006c, reserve_power2);
	ac97_codec_write(codec, 0x005c, reserve_power3);
	ac97_codec_write(codec, 0x0064, reserve_power4);
	ac97_codec_write(codec, 0x0066, reserve_power5);
	ac97_codec_write(codec, 0x006a, reserve_power6);

	ac97_codec_write(codec, 0x0002, reserve_mastervol);
	ac97_codec_write(codec, 0x000e, reserve_micvol);

	/* Enable variable rate mode */
	ac97_codec_write(codec, AC97_EXTENDED_STATUS, 9);
	ac97_codec_write(codec,
			 AC97_EXTENDED_STATUS,
			 ac97_codec_read(codec, AC97_EXTENDED_STATUS) | 0xE800);

	return 0;
}

static int jz_ac97_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
{
	int ret;
	struct jz_ac97_controller_info *controller = pm_dev->data;

	if (!controller) return -EINVAL;

	switch (req) {
	case PM_SUSPEND:
		ret = jz_ac97_suspend(controller, (int)data);
		break;

	case PM_RESUME:
		ret = jz_ac97_resume(controller);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}
#endif /* CONFIG_PM */
	
static int __init init_jz_ac97(void)
{
	int errno;

	if ((errno = probe_jz_ac97(&ac97_controller)) < 0)
		return errno;
	attach_jz_ac97(ac97_controller);

	out_empty_queue.id = NULL;
	out_full_queue.id = NULL;
	out_busy_queue.id = NULL;

	in_empty_queue.id = NULL;
	in_full_queue.id = NULL;
	in_busy_queue.id = NULL;

#ifdef CONFIG_PM
	ac97_controller->pm = pm_register(PM_SYS_DEV, PM_SYS_UNKNOWN, 
					  jz_ac97_pm_callback);
	if (ac97_controller->pm)
		ac97_controller->pm->data = ac97_controller;
#endif

	return 0;
}

static void __exit cleanup_jz_ac97(void)
{
	unload_jz_ac97(ac97_controller);
}

module_init(init_jz_ac97);
module_exit(cleanup_jz_ac97);


static int drain_adc(struct jz_ac97_controller_info *ctrl, int nonblock)
{
	DECLARE_WAITQUEUE(wait, current);
	unsigned long flags;
	int count;
	unsigned tmo;

	add_wait_queue(&ctrl->adc_wait, &wait);
	for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&ctrl->lock, flags);
		count = get_dma_residue(ctrl->dma2);
		spin_unlock_irqrestore(&ctrl->lock, flags);
		if (count <= 0)
			break;

		if (signal_pending(current))
			break;
		if (nonblock) {
			remove_wait_queue(&ctrl->adc_wait, &wait);
			current->state = TASK_RUNNING;
			return -EBUSY;
		}
		tmo = (PAGE_SIZE * HZ) / STANDARD_SPEED;
		tmo *= jz_audio_k * (jz_audio_format == AFMT_S16_LE) ? 2 : 4;
		tmo /= jz_audio_channels;
	}
	remove_wait_queue(&ctrl->adc_wait, &wait);
	current->state = TASK_RUNNING;
	if (signal_pending(current))
		return -ERESTARTSYS;
	return 0;
}

static int drain_dac(struct jz_ac97_controller_info *ctrl, int nonblock)
{
	DECLARE_WAITQUEUE(wait, current);
	unsigned long flags;
	int count;
	unsigned tmo;

	add_wait_queue(&(ctrl->dac_wait), &wait);
	for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&ctrl->lock, flags);
		count = get_dma_residue(ctrl->dma1);
		spin_unlock_irqrestore(&ctrl->lock, flags);
		if (count <= 0 && elements_in_queue(&out_full_queue) <= 0)
			break;
		if (signal_pending(current))
			break;
		if (nonblock) {
			remove_wait_queue(&ctrl->dac_wait, &wait);
			current->state = TASK_RUNNING;
			return -EBUSY;
		}
		tmo = (PAGE_SIZE * HZ) / STANDARD_SPEED;
		tmo *= jz_audio_k * (jz_audio_format == AFMT_S16_LE) ? 2 : 4;
		tmo /= jz_audio_channels;
	}
	remove_wait_queue(&ctrl->dac_wait, &wait);
	current->state = TASK_RUNNING;
	if (signal_pending(current))
		return -ERESTARTSYS;
	return 0;
}

/*
 *  Audio operation routines implementation
 */
static int jz_audio_release(struct inode *inode, struct file *file)
{
	struct jz_ac97_controller_info *controller =
		(struct jz_ac97_controller_info *) file->private_data;
	unsigned long flags;

	if (file->f_mode & FMODE_WRITE) {
		controller->opened1 = 0;
		drain_dac(controller, file->f_flags & O_NONBLOCK);
		disable_dma(controller->dma1);
		set_dma_count(controller->dma1, 0);
		__ac97_disable_transmit_dma();
		__ac97_disable_replay();

		spin_lock_irqsave(&controller->ioctllock, flags);
		controller->total_bytes = 0;
		controller->count = 0;
		controller->finish = 0;
		jz_audio_dma_tran_count = 0;
		controller->blocks = 0;
		controller->nextOut = 0;
		spin_unlock_irqrestore(&controller->ioctllock, flags);

	}
	if (file->f_mode & FMODE_READ) {
		controller->opened2 = 0;
		first_record_call = 1;
		drain_adc(controller, file->f_flags & O_NONBLOCK);
		disable_dma(controller->dma2);
		set_dma_count(controller->dma2, 0);
		__ac97_disable_receive_dma();
		__ac97_disable_record();
		spin_lock_irqsave(&controller->ioctllock, flags);
		controller->total_bytes = 0;
		jz_audio_dma_tran_count = 0;
		controller->count = 0;
		controller->finish = 0;
		controller->blocks = 0;
		controller->nextIn = 0;
		spin_unlock_irqrestore(&controller->ioctllock, flags);

	}
	Free_In_Out_queue(jz_audio_fragstotal,jz_audio_fragsize);
	return 0;
}

static int jz_audio_open(struct inode *inode, struct file *file)
{
	struct jz_ac97_controller_info *controller= ac97_controller;

	if (controller == NULL)
		return -ENODEV;

	if (file->f_mode & FMODE_WRITE) {
		if (controller->opened1 == 1)
			return -EBUSY;
		controller->opened1 = 1;
                //for ioctl
		controller->total_bytes = 0;
		jz_audio_dma_tran_count = 0;
		controller->count = 0;
		controller->finish = 0;
		controller->blocks = 0;
		controller->nextOut = 0;
		jz_audio_set_channels(controller->dev_audio, 1);
		jz_audio_set_format(controller->dev_audio, AFMT_U8);
		jz_audio_set_speed(controller->dev_audio, 8000);
	}

	if (file->f_mode & FMODE_READ) {
		if (controller->opened2 == 1)
			return -EBUSY;
		controller->opened2 = 1;
		first_record_call = 1;
		printk(KERN_DEBUG "You'd better apply 48000Hz 16-bit Stereo for record.\n");
		//for ioctl
		controller->total_bytes = 0;
		jz_audio_dma_tran_count = 0;
		controller->count = 0;
		controller->finish = 0;
		controller->blocks = 0;
		controller->nextIn = 0;
		jz_audio_set_channels(controller->dev_audio, 2);
		jz_audio_set_format(controller->dev_audio, AFMT_S16_LE);
		jz_audio_set_speed(controller->dev_audio, 48000);
	}

	last_jz_audio_count = jz_audio_count = 0;
	file->private_data = controller;

	jz_audio_fragsize = 8 * PAGE_SIZE;
	jz_audio_fragstotal = 4;
	Free_In_Out_queue(jz_audio_fragstotal,jz_audio_fragsize);
	return 0;
}

static int jz_audio_ioctl(struct inode *inode, struct file *file,
			    unsigned int cmd, unsigned long arg)
{
	struct jz_ac97_controller_info *controller =
		(struct jz_ac97_controller_info *) file->private_data;
	int val=0,fullc,busyc,unfinish;
	unsigned int flags;
	count_info cinfo;

	switch (cmd) 
	{
	case OSS_GETVERSION:
		return put_user(SOUND_VERSION, (int *)arg);

	case SNDCTL_DSP_RESET:
		jz_audio_reset();
		return 0;

	case SNDCTL_DSP_SYNC:
		if (file->f_mode & FMODE_WRITE)
			return drain_dac(controller, file->f_flags & O_NONBLOCK);
		return 0;

	case SNDCTL_DSP_SPEED: /* set smaple rate */
	{
		if (get_user(val, (int *)arg))
			return -EFAULT;
		if (val >= 0)
			jz_audio_set_speed(controller->dev_audio, val);
		return put_user(val, (int *)arg);
	}

	case SNDCTL_DSP_STEREO: /* set stereo or mono channel */
		if (get_user(val, (int *)arg))
			return -EFAULT;
		jz_audio_set_channels(controller->dev_audio, val ? 2 : 1);
		return 0;

	case SNDCTL_DSP_GETBLKSIZE:
		//return put_user(4*PAGE_SIZE, (int *)arg);
		return put_user(jz_audio_fragsize , (int *)arg);
	
	case SNDCTL_DSP_GETFMTS: /* Returns a mask of supported sample format*/
		return put_user(AFMT_U8 | AFMT_S16_LE, (int *)arg);

	case SNDCTL_DSP_SETFMT: /* Select sample format */
		if (get_user(val, (int *)arg))
			return -EFAULT;
                
                if (val != AFMT_QUERY) {
	                jz_audio_set_format(controller->dev_audio,val);
                } else {
       			if (file->f_mode & FMODE_READ)
				val = (jz_audio_format == 16) ?
					AFMT_S16_LE : AFMT_U8;
			else
				val = (jz_audio_format == 16) ?
					AFMT_S16_LE : AFMT_U8;
	        }
		return put_user(val, (int *)arg);

	case SNDCTL_DSP_CHANNELS:
		if (get_user(val, (int *)arg))
			return -EFAULT;
		jz_audio_set_channels(controller->dev_audio, val);
		return put_user(val, (int *)arg);

	case SNDCTL_DSP_POST:
		/* FIXME: the same as RESET ?? */
		return 0;

	case SNDCTL_DSP_SUBDIVIDE:
		return 0;

	case SNDCTL_DSP_SETFRAGMENT:
		if(out_dma_buf || in_dma_buf)
			return -EBUSY;
		get_user(val, (long *) arg);
		jz_audio_fragsize = 1 << (val & 0xFFFF);//16 least bits
		
		if (jz_audio_fragsize < 4 * PAGE_SIZE)
		  	jz_audio_fragsize = 4 * PAGE_SIZE;
		if (jz_audio_fragsize > (16 * PAGE_SIZE))  //16 PAGE_SIZE
		  	jz_audio_fragsize = 16 * PAGE_SIZE;
		jz_audio_fragstotal = (val >> 16) & 0x7FFF;
		if (jz_audio_fragstotal < 2)
			jz_audio_fragstotal = 2;
		if (jz_audio_fragstotal > 32)
			jz_audio_fragstotal = 32;
      
	        Free_In_Out_queue(jz_audio_fragstotal,jz_audio_fragsize);	
		return 0;

	case SNDCTL_DSP_GETCAPS:
		return put_user(DSP_CAP_REALTIME|DSP_CAP_BATCH, (int *)arg);

	case SNDCTL_DSP_NONBLOCK:
		file->f_flags |= O_NONBLOCK;
		return 0;

	case SNDCTL_DSP_SETDUPLEX:
		return -EINVAL;

	case SNDCTL_DSP_GETOSPACE:
	{		
		audio_buf_info abinfo;
		int i, bytes = 0;

		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		//unused fragment amount
		spin_lock_irqsave(&controller->ioctllock, flags);
		jz_audio_fragments = elements_in_queue(&out_empty_queue);
		for (i = 0; i < jz_audio_fragments; i++)
			bytes += jz_audio_fragsize;
		bytes /= jz_audio_k;
		spin_unlock_irqrestore(&controller->ioctllock, flags);
		//bytes /= jz_audio_b;
		abinfo.fragments = jz_audio_fragments;
		abinfo.fragstotal = jz_audio_fragstotal;
		abinfo.fragsize = jz_audio_fragsize;
		abinfo.bytes = bytes;
		return copy_to_user((void *)arg, &abinfo, sizeof(abinfo)) ? -EFAULT : 0;
	}

	case SNDCTL_DSP_GETISPACE:
	{
		audio_buf_info abinfo;
		int i, bytes = 0;

		if (!(file->f_mode & FMODE_READ))
			return -EINVAL;			
		//unused fragment amount
		jz_audio_fragments = elements_in_queue(&in_empty_queue);
		for (i = 0; i < jz_audio_fragments; i++)
			bytes += jz_audio_fragsize;
	       
		abinfo.fragments = jz_audio_fragments;
		abinfo.fragstotal = jz_audio_fragstotal;
		abinfo.fragsize = jz_audio_fragsize;
		abinfo.bytes = bytes;
		return copy_to_user((void *)arg, &abinfo, sizeof(abinfo)) ? -EFAULT : 0;
	}
	case SNDCTL_DSP_GETTRIGGER:
		val = 0;
		if (file->f_mode & FMODE_READ && in_dma_buf) //record is at working
			val |= PCM_ENABLE_INPUT;
		if (file->f_mode & FMODE_WRITE && out_dma_buf) //playback is at working
			val |= PCM_ENABLE_OUTPUT;
		return put_user(val, (int *)arg);

	case SNDCTL_DSP_SETTRIGGER:
		if (get_user(val, (int *)arg))
			return -EFAULT;
		/*if (file->f_mode & FMODE_READ) {
			if (val & PCM_ENABLE_INPUT) {
				if (!dmabuf->ready && (ret = prog_dmabuf(state, 1)))
					return ret;
				start_adc(state);
			} else
				stop_adc(state);
		}
		if (file->f_mode & FMODE_WRITE) {
			if (val & PCM_ENABLE_OUTPUT) {
				if (!dmabuf->ready && (ret = prog_dmabuf(state, 0)))
					return ret;
				start_dac(state);
			} else
				stop_dac(state);
				}*/
		return 0;

	case SNDCTL_DSP_GETIPTR:
		if (!(file->f_mode & FMODE_READ))
			return -EINVAL;

		spin_lock_irqsave(&controller->ioctllock, flags);
		//controller->total_bytes += get_dma_residue(controller->dma2);
		cinfo.bytes = controller->total_bytes;
		cinfo.blocks = controller->blocks;
		cinfo.ptr = controller->nextIn;
		controller->blocks = 0;
		spin_unlock_irqrestore(&controller->ioctllock, flags);
		return copy_to_user((void *)arg, &cinfo, sizeof(cinfo));

	case SNDCTL_DSP_GETOPTR:
		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;

		spin_lock_irqsave(&controller->ioctllock, flags);
		//controller->total_bytes += get_dma_residue(controller->dma1);
		cinfo.bytes = controller->total_bytes;
		cinfo.blocks = controller->blocks;
		cinfo.ptr = controller->nextOut;
		controller->blocks = 0;
		spin_unlock_irqrestore(&controller->ioctllock, flags);
		return copy_to_user((void *)arg, &cinfo, sizeof(cinfo));

	case SNDCTL_DSP_GETODELAY:
	{
		int id, i;

		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		spin_lock_irqsave(&controller->ioctllock, flags);
		unfinish = 0;
		fullc = elements_in_queue(&out_full_queue);
		busyc = elements_in_queue(&out_busy_queue);
    
		for(i = 0;i < fullc ;i ++)
		{
			id = *(out_full_queue.id + i);
			unfinish += *(out_dma_buf_data_count + id); 
		}
		for(i = 0;i < busyc ;i ++)
		{
			id = *(out_busy_queue.id + i);
			unfinish += get_dma_residue(controller->dma1);
		}
		spin_unlock_irqrestore(&controller->ioctllock, flags);
		unfinish /= jz_audio_k;//jz_audio_k is jz_audio_b
		return put_user(val, (int *)arg);
	}
	case SOUND_PCM_READ_RATE:
		return put_user(jz_audio_rate, (int *)arg);
	case SOUND_PCM_READ_CHANNELS:
		return put_user(jz_audio_channels, (int *)arg);
	case SOUND_PCM_READ_BITS:
		return put_user((jz_audio_format & (AFMT_S8 | AFMT_U8)) ? 8 : 16, (int *)arg);
	case SNDCTL_DSP_MAPINBUF:
	case SNDCTL_DSP_MAPOUTBUF:
	case SNDCTL_DSP_SETSYNCRO:
	case SOUND_PCM_WRITE_FILTER:
	case SOUND_PCM_READ_FILTER:
		return -EINVAL;
	}
	return -EINVAL;
}

static unsigned int jz_audio_poll(struct file *file,
				    struct poll_table_struct *wait)
{
	struct jz_ac97_controller_info *controller =
		(struct jz_ac97_controller_info *) file->private_data;
	unsigned long flags;
	unsigned int mask = 0;

	if (file->f_mode & FMODE_WRITE) {
		if (elements_in_queue(&out_empty_queue) > 0)
			return POLLOUT | POLLWRNORM;
		poll_wait(file, &controller->dac_wait, wait);
	}
	if (file->f_mode & FMODE_READ) {
		if (elements_in_queue(&in_full_queue) > 0)
			return POLLIN | POLLRDNORM;
		poll_wait(file, &controller->adc_wait, wait);
	}
	spin_lock_irqsave(&controller->lock, flags);
	if (file->f_mode & FMODE_WRITE) {
		if (elements_in_queue(&out_empty_queue) > 0)
			mask |= POLLOUT | POLLWRNORM;
	}
	else if (file->f_mode & FMODE_READ) {
		if (elements_in_queue(&in_full_queue) > 0)
			mask |= POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&controller->lock, flags);
	return mask;
}

static ssize_t jz_audio_read(struct file *file, char *buffer,
			       size_t count, loff_t *ppos)
{
	struct jz_ac97_controller_info *controller =
		(struct jz_ac97_controller_info *) file->private_data;
	int id, ret = 0, left_count, copy_count, cnt = 0;
	unsigned long flags;

	if (count < 0)
		return -EINVAL;

	spin_lock_irqsave(&controller->ioctllock, flags);
	controller->nextIn = 0;
	spin_unlock_irqrestore(&controller->ioctllock, flags);
	Init_In_Out_queue(jz_audio_fragstotal,jz_audio_fragsize);

	if (count < 2*PAGE_SIZE / jz_audio_k) {
		copy_count = count * 16 / (jz_audio_channels * jz_audio_format);
	} else
		copy_count = ((2*PAGE_SIZE / jz_audio_k + 3) / 4) * 4;
	left_count = count;

	if (first_record_call) {
		first_record_call = 0;
		if ((id = get_buffer_id(&in_empty_queue)) >= 0) {
			put_buffer_id(&in_busy_queue, id);
			*(in_dma_buf_data_count + id) = copy_count * (jz_audio_format/8);
			__ac97_enable_receive_dma();
			__ac97_enable_record();
			audio_start_dma(controller->dma2,file->private_data,
					*(in_dma_pbuf + id),
					*(in_dma_buf_data_count + id),
					DMA_MODE_READ);
			interruptible_sleep_on(&rx_wait_queue);
		} else
			BUG();
	}

	while (left_count > 0) {
		if (elements_in_queue(&in_full_queue) <= 0) {
			if (file->f_flags & O_NONBLOCK)
				return ret ? ret : -EAGAIN;
			else
				interruptible_sleep_on(&rx_wait_queue);
		}
		if (signal_pending(current))
			return ret ? ret: -ERESTARTSYS;

		if ((id = get_buffer_id(&in_full_queue)) >= 0) {
			cnt = record_filler((unsigned long)controller->tmp2+ret, copy_count, id);
			put_buffer_id(&in_empty_queue, id);
		} else
			BUG();

		if (elements_in_queue(&in_busy_queue) == 0) {
			if ((id=get_buffer_id(&in_empty_queue)) >= 0) {
				put_buffer_id(&in_busy_queue, id);
				*(in_dma_buf_data_count + id) = copy_count * (jz_audio_format/8);
				__ac97_enable_receive_dma();
				__ac97_enable_record();
				audio_start_dma(controller->dma2,file->private_data,
						*(in_dma_pbuf + id),
						*(in_dma_buf_data_count + id),
						DMA_MODE_READ);
			}
		}

		if (ret + cnt > count)
			cnt = count - ret;

		if (copy_to_user(buffer+ret, controller->tmp2+ret, cnt))
			return ret ? ret : -EFAULT;

		ret += cnt;
		spin_lock_irqsave(&controller->ioctllock, flags);
		controller->nextIn += ret;
		spin_unlock_irqrestore(&controller->ioctllock, flags);
		left_count -= cnt;
	}
	if (ret != count)
		printk(KERN_DEBUG "%s: count=%d ret=%d jz_audio_count=%d\n",
		       __FUNCTION__, count, ret, jz_audio_count);
	return ret;
}

static ssize_t jz_audio_write(struct file *file, const char *buffer,
				size_t count, loff_t *ppos)
{
	struct jz_ac97_controller_info *controller =
		(struct jz_ac97_controller_info *) file->private_data;
	int id, ret = 0, left_count, copy_count;
	unsigned int flags;

	if (count <= 0)
		return -EINVAL;

	spin_lock_irqsave(&controller->ioctllock, flags);
	controller->nextOut = 0;
	spin_unlock_irqrestore(&controller->ioctllock, flags);
	Init_In_Out_queue(jz_audio_fragstotal,jz_audio_fragsize);

	/* The data buffer size of the user space is always a PAGE_SIZE
	 * scale, so the process can be simplified. 
	 */

	if ((ac97_controller->patched) && (ac97_controller->ac97_features & 1)) 
		 copy_count = count;
	else {
		if (count < 2*PAGE_SIZE / jz_audio_k)
			copy_count = count;
		else
			copy_count = ((2*PAGE_SIZE / jz_audio_k + 3) / 4) * 4;
	}
	left_count = count;

	if (!(ac97_controller->patched) || !(ac97_controller->ac97_features & 1)) {
		if (copy_from_user(controller->tmp1, buffer, count)) {
			printk(KERN_DEBUG "%s: copy_from_user failed.\n", __FUNCTION__);
			return ret ? ret : -EFAULT;
		}
	}

	while (left_count > 0) {
		if (elements_in_queue(&out_empty_queue) == 0) {
			if (file->f_flags & O_NONBLOCK)
				return ret;
			else
				interruptible_sleep_on(&tx_wait_queue);
		}

		if (signal_pending(current))
			return ret ? ret : -ERESTARTSYS;

		/* the end fragment size in this write */
		if (ret + copy_count > count)
			copy_count = count - ret;

		if ((id = get_buffer_id(&out_empty_queue)) >= 0) {
			if ((ac97_controller->patched) &&
			    (ac97_controller->ac97_features & 1)) {
				if (copy_from_user((char *)(*(out_dma_buf + id)), buffer+ret, copy_count)) {
					printk(KERN_DEBUG "%s: copy_from_user failed.\n", __FUNCTION__);
					return ret ? ret : -EFAULT;
				}
				*(out_dma_buf_data_count + id) = copy_count;
			} else
				replay_filler(
					(unsigned long)controller->tmp1 + ret,
					copy_count, id);
			//when 0,kernel will panic
			if(*(out_dma_buf_data_count + id) > 0) {
				put_buffer_id(&out_full_queue, id);
				dma_cache_wback_inv(*(out_dma_buf + id), *(out_dma_buf_data_count + id));
			} else {//when 0,i need refill in empty queue
				put_buffer_id(&out_empty_queue, id);
			}
		} else 
			BUG();
		left_count = left_count - copy_count;
		ret += copy_count;

		spin_lock_irqsave(&controller->ioctllock, flags);
		controller->nextOut += ret;
		spin_unlock_irqrestore(&controller->ioctllock, flags);

		if (elements_in_queue(&out_busy_queue) == 0) {
			if ((id=get_buffer_id(&out_full_queue)) >= 0) {
				put_buffer_id(&out_busy_queue, id);

				__ac97_enable_transmit_dma();
				__ac97_enable_replay();
			        if(*(out_dma_buf_data_count + id) > 0) {
				audio_start_dma(controller->dma1,file->private_data,
						*(out_dma_pbuf + id),
						*(out_dma_buf_data_count + id),
						DMA_MODE_WRITE);
				}
			}
		}
	}

	if (ret != count)
		printk(KERN_DEBUG "%s: count=%d ret=%d jz_audio_count=%d\n",
		       __FUNCTION__, count, ret, jz_audio_count);
	return ret;
}

/* this function for the other codec function */
struct ac97_codec * find_ac97_codec(void)
{
	if ( ac97_controller )
		return ac97_controller->ac97_codec[0];
	return 0;
		
}

