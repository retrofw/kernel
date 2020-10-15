/*
 *  linux/drivers/sound/jz_pcm_tlv320aic1106.c
 *
 *  JzSOC On-Chip PCM audio driver.
 *
 *  Copyright (C) 2005 by Ingenic Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Because the normal application of AUDIO devices are focused on Little_endian,
 * then we only perform the little endian data format in driver.
 *
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/delay.h>

#include <linux/sound.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <linux/proc_fs.h>
#include <linux/soundcard.h>
#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <asm/hardirq.h>
#include <asm/jzsoc.h>
#include "sound_config.h"

#define NR_PCM		2
#define MAXDELAY 50000
#define JZCODEC_RW_BUFFER_SIZE       4
#define JZCODEC_RW_BUFFER_TOTAL      3
#define JZCODEC_USER_BUFFER     6

#define MODE_is_8 0  // 1 is 8 bits, and 0 is 16 bits

static int		jz_audio_rate;
static char		jz_audio_format;
static char		jz_audio_volume;
static char		jz_audio_channels;
static int              jz_audio_fragments;//unused fragment amount
static int              jz_audio_fragstotal;
static int              jz_audio_fragsize;
static int              jz_audio_speed;
static int              jz_audio_dma_tran_count;//bytes count of one DMA transfer

static void jz_update_filler(int bits, int channels);
static int Init_In_Out_queue(int fragstotal,int fragsize);
static int Free_In_Out_queue(int fragstotal,int fragsize);
static irqreturn_t jz_pcm_irq(int irqnr, void *ref);
static irqreturn_t jz_pcm_replay_dma_irq(int irqnr, void *ref);
static irqreturn_t jz_pcm_record_dma_irq(int irqnr, void *ref);
static void (*replay_filler)(unsigned long src_start, int count, int id);
static int (*record_filler)(unsigned long dst_start, int count, int id);
static void dump_pcmc_reg(void);

static struct file_operations jz_pcm_audio_fops;
static DECLARE_WAIT_QUEUE_HEAD (rx_wait_queue);
static DECLARE_WAIT_QUEUE_HEAD (tx_wait_queue);

struct jz_pcm_controller_info {
	int io_base;
	int dma1; /* play */
	int dma2; /* record */
	char *name;
	int dev_audio;
	struct pcm_codec *pcm_codec[NR_PCM];
	int opened1;
	int opened2;
	unsigned char *tmp1;  /* tmp buffer for sample conversions */
	unsigned char *tmp2;
	spinlock_t lock;
	spinlock_t ioctllock;
	
	wait_queue_head_t dac_wait;
	wait_queue_head_t adc_wait;
	int nextIn;  // byte index to next-in to DMA buffer
	int nextOut; // byte index to next-out from DMA buffer
	int count;	// current byte count in DMA buffer
	int finish; // current transfered byte count in DMA buffer 
	unsigned long total_bytes;	// total bytes written or read
	unsigned long blocks;
	unsigned long error;	// over/underrun
};
static struct jz_pcm_controller_info *pcm_controller = NULL;

struct pcm_codec {
	/* PCM controller connected with */
	void *private_data;
	char *name;
	int id;
	int dev_mixer; 
	/* controller specific lower leverl pcm accessing routines */
	u16  (*codec_read)  (u8 reg);//the function accessing Codec REGs
	void (*codec_write) (u8 reg, u16 val);
	/* Wait for codec-ready.  Ok to sleep here.  */
	void  (*codec_wait)  (struct pcm_codec *codec);
	/* OSS mixer masks */
	int modcnt;
	int supported_mixers;
	int stereo_mixers;
	int record_sources;
	int bit_resolution;
	/* OSS mixer interface */
	int  (*read_mixer) (struct pcm_codec *codec, int oss_channel);
	void (*write_mixer)(struct pcm_codec *codec, int oss_channel,
			    unsigned int left, unsigned int right);
	int  (*recmask_io) (struct pcm_codec *codec, int rw, int mask);
	int  (*mixer_ioctl)(struct pcm_codec *codec, unsigned int cmd, unsigned long arg);
	/* saved OSS mixer states */
	unsigned int mixer_state[SOUND_MIXER_NRDEVICES];
};

typedef struct buffer_queue_s {
	int count;
	int *id;
	int lock;
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

static inline void audio_start_dma(int chan, void *dev_id, unsigned long phyaddr,int count, int mode)
{
	unsigned long flags;
	struct jz_pcm_controller_info * controller = (struct jz_pcm_controller_info *) dev_id;
	
	//for DSP_GETOPTR
	//spin_lock_irqsave(&controller->ioctllock, flags);
	spin_lock(&controller->ioctllock);
	jz_audio_dma_tran_count = count;
	//spin_unlock_irqrestore(&controller->ioctllock, flags);
	spin_unlock(&controller->ioctllock);
	flags = claim_dma_lock();
	__dmac_disable_module(0);//!!!!!!!!!
	disable_dma(chan);
	clear_dma_ff(chan);
	set_dma_mode(chan, mode);
#if MODE_is_8
	__dmac_channel_set_src_port_width(chan, 8);
	__dmac_channel_set_dest_port_width(chan, 8);
	__dmac_channel_set_transfer_unit_8bit(chan);
#else
	__dmac_channel_set_src_port_width(chan, 16);
	__dmac_channel_set_dest_port_width(chan, 16);
	__dmac_channel_set_transfer_unit_16bit(chan);
#endif

	set_dma_addr(chan, phyaddr);
	if (count == 0) {
		count++;
		printk("JzSOC DMA controller can't set dma 0 count!\n");
	}

	set_dma_count(chan, count);
	enable_dma(chan);
	__dmac_enable_module(0);
	release_dma_lock(flags);
#if 0
	//for DSP_GETOPTR on FPGA
	struct jz_dma_chan *tmpchan = &jz_dma_table[1];

	spin_lock(&controller->ioctllock);
	jz_audio_dma_tran_count = count;
	spin_unlock(&controller->ioctllock);
	flags = claim_dma_lock();
	disable_dma(chan);
	clear_dma_ff(chan);
	set_dma_mode(chan, mode);
#if MODE_is_8
	__dmac_channel_set_src_port_width(chan, 8);
	__dmac_channel_set_dest_port_width(chan, 8);
	__dmac_channel_set_transfer_unit_8bit(chan);
#else
	__dmac_channel_set_src_port_width(chan, 16);
	__dmac_channel_set_dest_port_width(chan, 16);
	__dmac_channel_set_transfer_unit_16bit(chan);
#endif
	set_dma_addr(chan, phyaddr);
	if (count == 0) {
		count++;
		printk("JzSOC DMA controller can't set dma 0 count!\n");
	}
	set_dma_count(chan, count);
	enable_dma(chan);
	release_dma_lock(flags);
#if 0	
	__pcm_enable_tfs_intr();
	__pcm_enable_tur_intr();
	__pcm_enable_rfs_intr();
	__pcm_enable_ror_intr();
#endif
#endif
}

static irqreturn_t jz_pcm_record_dma_irq(int irq, void *dev_id)
{
	int id1, id2;
	struct jz_pcm_controller_info * controller = (struct jz_pcm_controller_info *) dev_id;
	int dma = controller->dma2;

	disable_dma(dma);
	if (__dmac_channel_address_error_detected(dma)) {
		printk(KERN_DEBUG "%s: DMAC address error.\n", __FUNCTION__);
		__dmac_channel_clear_address_error(dma);
	}
	if (__dmac_channel_transmit_end_detected(dma)) {
		__dmac_channel_clear_transmit_end(dma);
		//for DSP_GETIPTR
		spin_lock(&controller->ioctllock);
		controller->total_bytes += jz_audio_dma_tran_count;
		controller->blocks ++;
		spin_unlock(&controller->ioctllock);
		id1 = get_buffer_id(&in_busy_queue);
		put_buffer_id(&in_full_queue, id1);
		
		wake_up(&rx_wait_queue);
		wake_up(&controller->adc_wait);
		if ((id2 = get_buffer_id(&in_empty_queue)) >= 0) {
			put_buffer_id(&in_busy_queue, id2);
			*(in_dma_buf_data_count + id2) = *(in_dma_buf_data_count + id1);
			
			dma_cache_wback_inv(*(in_dma_buf + id2), *(in_dma_buf_data_count + id2));
			audio_start_dma(dma,dev_id,
					*(in_dma_pbuf + id2),
					*(in_dma_buf_data_count + id2),
					DMA_MODE_READ);
		} else 
			in_busy_queue.count = 0;
	}
    
	return IRQ_HANDLED;
}

static irqreturn_t jz_pcm_irq(int irq, void *dev_id)
{
	struct jz_pcm_controller_info * controller = (struct jz_pcm_controller_info *) dev_id;
	printk("pcm interrupt REG_PCM_INTS : 0x%08x\n",REG_PCM_INTS);
      
	return IRQ_HANDLED;
}

static irqreturn_t jz_pcm_replay_dma_irq(int irq, void *dev_id)
{
	int id;
	unsigned long flags;
	struct jz_pcm_controller_info * controller = (struct jz_pcm_controller_info *) dev_id;
	int dma = controller->dma1;

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
		if ((id = get_buffer_id(&out_busy_queue)) < 0)
			printk(KERN_DEBUG "Strange DMA finish interrupt for PCM module\n");
		put_buffer_id(&out_empty_queue, id);
		if ((id = get_buffer_id(&out_full_queue)) >= 0) {
			put_buffer_id(&out_busy_queue, id); //very busy
			if(*(out_dma_buf_data_count + id) > 0) {
				audio_start_dma(dma, dev_id, *(out_dma_pbuf + id),
						*(out_dma_buf_data_count + id),
						DMA_MODE_WRITE);
			}
		} else
			out_busy_queue.count = 0;

		if (elements_in_queue(&out_empty_queue) > 0) {
			wake_up(&tx_wait_queue);
			wake_up(&controller->dac_wait);
		}
	}

	return IRQ_HANDLED;
}

static int Init_In_Out_queue(int fragstotal,int fragsize)
{
	int i;
	// recording
	in_empty_queue.count = fragstotal;
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

	for (i=0;i < fragstotal;i++)
		*(in_empty_queue.id + i) = i;
	in_full_queue.count = 0;
	in_busy_queue.count = 0;

	for (i = 0; i < fragstotal; i++) {
		*(in_dma_buf + i) = __get_free_pages(GFP_KERNEL | GFP_DMA, get_order(fragsize));
		if (*(in_dma_buf + i) == 0)
			goto mem_failed_in;
		*(in_dma_pbuf + i) = virt_to_phys((void *)(*(in_dma_buf + i)));
		dma_cache_wback_inv(*(in_dma_buf + i), fragsize);
	}

	//playing
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
        out_empty_queue.id = (int *)kmalloc(sizeof(int) * fragstotal, GFP_KERNEL);
	if (!out_empty_queue.id)
		goto all_mem_err;
	out_full_queue.id = (int *)kmalloc(sizeof(int) * fragstotal, GFP_KERNEL);
	if (!out_full_queue.id)
		goto all_mem_err;
	out_busy_queue.id = (int *)kmalloc(sizeof(int) * fragstotal, GFP_KERNEL);
	if (!out_busy_queue.id)
		goto all_mem_err;
	for (i=0;i < fragstotal;i++) 
		*(out_empty_queue.id + i) = i;

	out_busy_queue.count = 0;
	out_full_queue.count = 0;
	/*alloc DMA buffer*/
	for (i = 0; i < fragstotal; i++) {
		*(out_dma_buf + i) = __get_free_pages(GFP_KERNEL | GFP_DMA, get_order(fragsize));
		if (*(out_dma_buf + i) == 0) {
			printk(" can't allocate required DMA(OUT) buffers.\n");
			goto mem_failed_out;
		}
		*(out_dma_pbuf + i) = virt_to_phys((void *)(*(out_dma_buf + i)));
	}
	
	return 1;
all_mem_err:
	printk("error:allocate memory occur error 1!\n");
	return 0;
mem_failed_out:
	printk("error:allocate memory occur error 2!\n");
	for (i = 0; i < fragstotal; i++) {
		if(*(out_dma_buf + i))
			free_pages(*(out_dma_buf + i), get_order(fragsize));
	}

	return 0;
mem_failed_in:
	printk("error:allocate memory occur error 3!\n");
	for (i = 0; i < fragstotal; i++) {
		if(*(in_dma_buf + i))
			free_pages(*(in_dma_buf + i), get_order(fragsize));
	}
	return 0;
}

static int Free_In_Out_queue(int fragstotal,int fragsize)
{
	int i;
	//playing
	if(out_dma_buf != NULL) {
		for (i = 0; i < fragstotal; i++) {
			if(*(out_dma_buf + i))
				free_pages(*(out_dma_buf + i), get_order(fragsize));
			*(out_dma_buf + i) = 0; //release page error
		}
		kfree(out_dma_buf);
		out_dma_buf = NULL;
	}
	if(out_dma_pbuf) {
		kfree(out_dma_pbuf);
		out_dma_pbuf = NULL;
	}
	if(out_dma_buf_data_count) {
		kfree(out_dma_buf_data_count);
		out_dma_buf_data_count = NULL;
	}
	if(out_empty_queue.id) {
		kfree(out_empty_queue.id);
		out_empty_queue.id = NULL;
	}
	if(out_full_queue.id) {
		kfree(out_full_queue.id);
		out_full_queue.id = NULL;
	}
	if(out_busy_queue.id) {
		kfree(out_busy_queue.id);
		out_busy_queue.id = NULL;
	}
	out_empty_queue.count = fragstotal;
	out_busy_queue.count = 0;
	out_full_queue.count = 0;

	// recording
	if(in_dma_buf) {
		for (i = 0; i < fragstotal; i++) {
			if(*(in_dma_buf + i)) {
				dma_cache_wback_inv(*(in_dma_buf + i), fragsize);
				free_pages(*(in_dma_buf + i), get_order(fragsize));
			}
			*(in_dma_buf + i) = 0; //release page error 
		}
		kfree(in_dma_buf);
		in_dma_buf = NULL;
	}
	if(in_dma_pbuf) {
		kfree(in_dma_pbuf);
		in_dma_pbuf = NULL;
	}
	if(in_dma_buf_data_count) {
		kfree(in_dma_buf_data_count);
		in_dma_buf_data_count = NULL;
	}
	if(in_empty_queue.id) {
		kfree(in_empty_queue.id);
		in_empty_queue.id = NULL;
	}
	if(in_full_queue.id) {
		kfree(in_full_queue.id);
		in_full_queue.id = NULL;
	}
	if(in_busy_queue.id) {
		kfree(in_busy_queue.id);
		in_busy_queue.id = NULL;
	}

	in_empty_queue.count = fragstotal;
	in_full_queue.count = 0;
	in_busy_queue.count = 0;
 
	return 1;
}

static int jz_audio_set_speed(int dev, int rate)
{
	/* 8000, 11025, 16000, 22050, 24000, 32000, 44100, 48000 */
	long codec_speed;
	long speed = 0;
	
	jz_audio_speed = rate;
	if (rate > 48000)
		rate = 48000;
	if (rate < 8000)
		rate = 8000;
	jz_audio_rate = rate;
	
	/*switch (rate) {
	case 8000:
		speed = 0;
		break;
	case 11025:
		speed = 1;
		break;
	case 12000:
		speed = 2;
		break;
	case 16000:
		speed = 3;
		break;
	case 22050:
		speed = 4;
		break;
	case 24000:
		speed = 5;
		break;
	case 32000:
		speed = 6;
		break;
	case 44100:
		speed = 7;
		break;
	case 48000:
		speed = 8;
		break;
	default:
		break;
	}
	printk("set PCMIN or PCMOUT speed.\n");
	__pcm_set_clk_rate(speed);
	__pcm_set_sync_rate(256);*/

	return jz_audio_rate;
}


static int record_fill_1x8_u(unsigned long dst_start, int count, int id)
{
	int cnt = 0;
	volatile unsigned char *s = (unsigned char*)(*(in_dma_buf + id));
	volatile unsigned char *dp = (unsigned char*)dst_start;

	while (count > 0) {
		*dp = *s;
		dp ++;
		s++;
		count --;
		cnt ++;
	}

	return cnt;
}

static int record_fill_2x8_u(unsigned long dst_start, int count, int id)
{
	int cnt = 0;
	volatile unsigned char *s = (unsigned char*)(*(in_dma_buf + id));
	volatile unsigned char *dp = (unsigned char*)dst_start;

#if 1
	while (count > 0) {
		*dp = *s;
		s ++;
		dp ++;
		count --;
		cnt ++;
	}
#else
	while (count > 0) {
		*dp = *s;
		s += 2; //skip right sample
		dp ++;
		count -= 2;
		cnt ++;
	}
#endif
	return cnt;
}

static int record_fill_1x16_s(unsigned long dst_start, int count, int id)
{
	int cnt = 0;
	unsigned short d1;
	unsigned short *s = (unsigned short*)(*(in_dma_buf + id));
	unsigned short *dp = (unsigned short *)dst_start;

	while (count > 0) {
		*dp = *s;
		s ++;
		dp ++;
		count -= 2;
		cnt += 2;
	}

	return cnt;
}

static int record_fill_2x16_s(unsigned long dst_start, int count, int id)
{
	int cnt = 0;
	unsigned short *s = (unsigned short*)(*(in_dma_buf + id));
	unsigned short *dp = (unsigned short *)dst_start;

#if 1
	while (count > 0) {
		*dp = *s;
		s ++;
		dp ++;
		count -= 2;
		cnt += 2;
	}
#else
	while (count > 0) {
		*dp = *s;
		s += 2; //skip right sample
		dp ++;
		count -= 4;
		cnt += 2;/* count in byte */
	}
#endif
	
	return cnt;
}

static void replay_fill_1x8_u(unsigned long src_start, int count, int id)
{
#if 0
	volatile unsigned char *s = (unsigned char *)src_start;
	volatile unsigned char *dp = (unsigned char *)(*(out_dma_buf + id));

	*(out_dma_buf_data_count + id) = count;
	while (count > 0) {
	        *dp = *s;
		dp ++;
		s ++;
		count --;
	}
#else
	volatile u8 *s = (u8 *)src_start;
	volatile u8 *dp = (u8 *)(*(out_dma_buf + id));

	*(out_dma_buf_data_count + id) = count;
	while (count > 0) {
	        *dp = *s;
		dp ++;
		s ++;
		count --;
	}
#endif
}

static void replay_fill_2x8_u(unsigned long src_start, int count, int id)
{
	volatile unsigned char *s = (unsigned char *)src_start;
	volatile unsigned char *dp = (unsigned char *)(*(out_dma_buf + id));

#if 1
	*(out_dma_buf_data_count + id) = count;
	while (count > 0) {
	        *dp = *s;
		dp ++;
		s ++;
		count --;
	}
#else
	while (count > 0) {
		*dp = *s;
		s += 2; //skip right sample
		dp ++;
		count -= 2;
		cnt ++;
	}
	*(out_dma_buf_data_count + id) = cnt;
#endif
}


static void replay_fill_1x16_s(unsigned long src_start, int count, int id)
{
	int cnt = 0;
	unsigned short d1, l1;
	signed short sam_to;
	volatile unsigned short *s = (unsigned short *)src_start;
	volatile unsigned short *dp = (unsigned short*)(*(out_dma_buf + id));

	while (count > 0) {
		d1 = *s;

		sam_to = (signed short)d1;
		if (sam_to >= 0) {
			sam_to = 0xfff * sam_to / 0x7fff;
		} else {
			sam_to = 0 - sam_to;
			sam_to = 0xfff * sam_to / 0x7fff;
			sam_to = 0 - sam_to;
		}
		d1 = (unsigned short)sam_to;
		d1 = d1 << 3;
		l1 = d1 | jz_audio_volume;		

		*dp = l1;
		s ++;
		dp ++;
		count -= 2;
		cnt += 2 ;
	}
	*(out_dma_buf_data_count + id) = cnt;
}

static void replay_fill_2x16_s(unsigned long src_start, int count, int id)
{
	int cnt = 0;
	unsigned short d1, l1;
	volatile unsigned short *s = (unsigned short *)src_start;
	volatile unsigned short *dp = (unsigned short*)(*(out_dma_buf + id));
	
#if 1
	while (count > 0) {
		d1 = *s;
		l1 = (d1 & 0xfff8) | jz_audio_volume;		
		*dp = l1;
		s ++;
		dp ++;
		count -= 2;
		cnt += 2 ;
	}
	*(out_dma_buf_data_count + id) = cnt;
#else
	while (count > 0) {
		d1 = *s;
		l1 = (d1 & 0xfff8) | jz_audio_volume;
		*dp = l1;
		s += 2; //skip right sample
		dp ++;
		count -= 4;
		cnt += 2;
	}
	*(out_dma_buf_data_count + id) = cnt;
#endif
}

static unsigned int jz_audio_set_format(int dev, unsigned int fmt)
{
	switch (fmt) {
	case AFMT_U8:
	case AFMT_S16_LE:
		jz_audio_format = fmt;
		jz_update_filler(jz_audio_format, jz_audio_channels);
	case AFMT_QUERY:
		break;
	}

	return jz_audio_format;
}

static short jz_audio_set_channels(int dev, short channels)
{
	switch (channels) {
	case 1:
	case 2:
		jz_audio_channels = channels;
		jz_update_filler(jz_audio_format, jz_audio_channels);
		break;
	default:
		printk("channel number is wrong. %d\n",__LINE__);
	}

	return jz_audio_channels;
}

static void init_codec(void)
{
	printk("set PCM codec RESET pin on LOW, while MCLK occur.\n");
	printk("set LINSEL on LOW(8bits) or HIGH(13bits+3bits for PCMOUT gain).\n");
}

static void jz_audio_reset(void)
{
	__pcm_flush_fifo();
	__pcm_disable_txfifo();
	__pcm_disable_rxfifo();
	__pcm_set_transmit_trigger(7);
	__pcm_set_receive_trigger(7);
	__pcm_omsb_next_sync();
	__pcm_imsb_next_sync();
#if MODE_is_8
	__pcm_set_iss(8);//8bits decided by LINSEL
	__pcm_set_oss(8);//8bits decided by LINSEL
#else
	__pcm_set_iss(16);//16bits decided by LINSEL
	__pcm_set_oss(16);
#endif
	__pcm_disable_tfs_intr();
	__pcm_disable_tur_intr();
	__pcm_disable_rfs_intr();
	__pcm_disable_ror_intr();

	__pcm_disable_receive_dma();
	__pcm_disable_transmit_dma();

	//init_codec();
}

static int jz_audio_release(struct inode *inode, struct file *file);
static int jz_audio_open(struct inode *inode, struct file *file);
static int jz_audio_ioctl(struct inode *inode, struct file *file,unsigned int cmd, unsigned long arg);
static unsigned int jz_audio_poll(struct file *file,struct poll_table_struct *wait);
static ssize_t jz_audio_write(struct file *file, const char *buffer,size_t count, loff_t *ppos);
static ssize_t jz_audio_read(struct file *file, char *buffer,size_t count, loff_t *ppos);

/* static struct file_operations jz_pcm_audio_fops */
static struct file_operations jz_pcm_audio_fops =
{
	owner:              THIS_MODULE,
	open:               jz_audio_open,
	release:            jz_audio_release,
	write:              jz_audio_write,
	read:               jz_audio_read,
	poll:               jz_audio_poll,
	ioctl:              jz_audio_ioctl
};

static int jz_pcm_open_mixdev(struct inode *inode, struct file *file)
{
    int i;
    int minor = MINOR(inode->i_rdev);
    struct jz_pcm_controller_info *controller = pcm_controller;

    for (i = 0; i < NR_PCM; i++)
	    if (controller->pcm_codec[i] != NULL && controller->pcm_codec[i]->dev_mixer == minor)
		    goto match;

    if (!controller)
	    return -ENODEV;
match:
    file->private_data = controller->pcm_codec[i];

    return 0;
}

static int jz_pcm_ioctl_mixdev(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct pcm_codec *codec = (struct pcm_codec *)file->private_data;
	return codec->mixer_ioctl(codec, cmd, arg);
}

static loff_t jz_pcm_llseek(struct file *file, loff_t offset, int origin)
{
	return -ESPIPE;
}

static struct file_operations jz_pcm_mixer_fops = 
{
	owner:		THIS_MODULE,
	llseek:		jz_pcm_llseek,
	ioctl:		jz_pcm_ioctl_mixdev,
	open:		jz_pcm_open_mixdev,
};

static int pcm_mixer_ioctl(struct pcm_codec *codec, unsigned int cmd, unsigned long arg)
{
	int ret;
	long val = 0;
   
	switch (cmd) {
	case SOUND_MIXER_READ_STEREODEVS:
		return put_user(0, (long *) arg);
	case SOUND_MIXER_READ_CAPS:
		val = SOUND_CAP_EXCL_INPUT;
		return put_user(val, (long *) arg);
	case SOUND_MIXER_READ_DEVMASK:
		break;
	case SOUND_MIXER_READ_RECMASK:
		break;
	case SOUND_MIXER_READ_RECSRC:
		break;
	case SOUND_MIXER_WRITE_VOLUME:
		ret = get_user(val, (long *) arg);
		if (ret)
			return ret;
		val = val & 0xff;
		if(val < 0)
			val = 0;
		if(val > 100)
			val = 100;
		if (val == 100) {
			dump_pcmc_reg();
			return 100;
		}
		val = val / 10;
		switch (val) {
		case 0:
		case 1:
			jz_audio_volume = 7;
			break;
		case 2:
			jz_audio_volume = 6;
			break;
		case 3:
			jz_audio_volume = 5;
			break;
		case 4:
			jz_audio_volume = 4;
			break;
		case 5:
			jz_audio_volume = 3;
			break;
		case 6:
			jz_audio_volume = 2;
			break;
		case 7:
		case 8:
			jz_audio_volume = 1;
			break;
		case 9:
		case 10:
			jz_audio_volume = 0;
			break;
		}
		return 0;
	case SOUND_MIXER_READ_VOLUME:
		val = jz_audio_volume;
		ret = val << 8;
		val = val | ret;
		return put_user(val, (long *) arg);
	case SOUND_MIXER_WRITE_MIC:
		printk("Can not justify Mic gain to the PCM codec.!\n");
		return -ENOSYS;

	case SOUND_MIXER_READ_MIC:
		printk("Can not justify Mic gain to the PCM codec.!\n");
		return -ENOSYS;
	default:
		return -ENOSYS;
	}
	
	return 0;
}


int pcm_probe_codec(struct pcm_codec *codec)
{
	/* generic OSS to PCM wrapper */
	codec->mixer_ioctl = pcm_mixer_ioctl;
	return 1;
}

/* PCM codec initialisation. */
static int __init jz_pcm_codec_init(struct jz_pcm_controller_info *controller)
{
	int num_pcm = 0;
	struct pcm_codec *codec;
	
	for (num_pcm = 0; num_pcm < NR_PCM; num_pcm++) {
		if ((codec = kmalloc(sizeof(struct pcm_codec),GFP_KERNEL)) == NULL)
			return -ENOMEM;
		memset(codec, 0, sizeof(struct pcm_codec));
		codec->private_data = controller;
		codec->id = num_pcm;
		
		if (pcm_probe_codec(codec) == 0)
			break;
		if ((codec->dev_mixer = register_sound_mixer(&jz_pcm_mixer_fops, -1)) < 0) {
			printk(KERN_ERR "Jz PCM: couldn't register mixer!\n");
			kfree(codec);
			break;
		}
		controller->pcm_codec[num_pcm] = codec;
	}
	
	return num_pcm;
}

static void jz_update_filler(int format, int channels)
{
#define TYPE(fmt,ch) (((fmt)<<2) | ((ch)&3))

	switch (TYPE(format, channels)) {
	default:
	case TYPE(AFMT_U8, 1):
		replay_filler = replay_fill_1x8_u;
		record_filler = record_fill_1x8_u;
		break;
	case TYPE(AFMT_U8, 2):
		/*replay_filler = replay_fill_2x8_u;
		record_filler = record_fill_2x8_u;*/
		printk("channel is 2. Line:%d\n",__LINE__);
		break;
	case TYPE(AFMT_S16_LE, 1):
		replay_filler = replay_fill_1x16_s;
		record_filler = record_fill_1x16_s;
		break;
	case TYPE(AFMT_S16_LE, 2):
		/*replay_filler = replay_fill_2x16_s;
		record_filler = record_fill_2x16_s;*/
		printk("channel is 2. Line:%d\n",__LINE__);
		break;
	}
}

static void __init attach_jz_pcm(struct jz_pcm_controller_info *controller)
{
	char *name;
	int adev;//No of Audio device.
	int err;
	
	name = controller->name;
	/* register /dev/audio  */
	adev = register_sound_dsp(&jz_pcm_audio_fops, -1);
	if (adev < 0)
		goto audio_failed;
	/* initialize PCM codec and register /dev/mixer */
	if (jz_pcm_codec_init(controller) <= 0)
		goto mixer_failed;

	controller->tmp1 = (void *)__get_free_pages(GFP_KERNEL, JZCODEC_USER_BUFFER);
	if (!controller->tmp1) {
		printk(KERN_ERR "%s: can't allocate tmp buffers.\n", controller->name);
		goto tmp1_failed;
	}
	printk("DMA_ID_PCM_TX=0x%08x DMA_ID_PCM_RX=0x%08x\n",DMA_ID_PCM_TX ,DMA_ID_PCM_RX);
	if ((controller->dma1 = jz_request_dma(DMA_ID_PCM_TX, "audio dac", jz_pcm_replay_dma_irq, IRQF_DISABLED, controller)) < 0) {
		printk(KERN_ERR "%s: can't reqeust DMA DAC channel.\n", name);
		goto dma1_failed;
	}
	if ((controller->dma2 = jz_request_dma(DMA_ID_PCM_RX, "audio adc", jz_pcm_record_dma_irq, IRQF_DISABLED, controller)) < 0) {
		printk(KERN_ERR "%s: can't reqeust DMA ADC channel.\n", name);
		goto dma2_failed;
	}
	printk("JzSOC On-Chip PCM controller registered (DAC: DMA(play):%d/IRQ%d,\n ADC: DMA(record):%d/IRQ%d)\n", controller->dma1, get_dma_done_irq(controller->dma1), controller->dma2, get_dma_done_irq(controller->dma2));

	err = request_irq(IRQ_PCM, jz_pcm_irq, IRQF_DISABLED, "pcm irq", controller);
	if (err < 0) 
		printk("can't allocate pcm irq.\n");
	controller->dev_audio = adev;
	
	return;
dma2_failed:
	jz_free_dma(controller->dma2);
dma1_failed:
	jz_free_dma(controller->dma1);
tmp1_failed:
	free_pages((unsigned long)controller->tmp1, JZCODEC_USER_BUFFER);
	
mixer_failed:
	unregister_sound_dsp(adev);
audio_failed:
	return;
}


static int __init probe_jz_pcm(struct jz_pcm_controller_info **controller)
{
	if ((*controller = kmalloc(sizeof(struct jz_pcm_controller_info),
				   GFP_KERNEL)) == NULL) {
		printk(KERN_ERR "Jz PCM Controller: out of memory.\n");
		return -ENOMEM;
	}

	(*controller)->name = "Jz PCM controller";
	(*controller)->opened1 = 0;
	(*controller)->opened2 = 0;
	init_waitqueue_head(&(*controller)->adc_wait);
	init_waitqueue_head(&(*controller)->dac_wait);
	spin_lock_init(&(*controller)->lock);
	init_waitqueue_head(&rx_wait_queue);
	init_waitqueue_head(&tx_wait_queue);

	return 0;
}

static void __exit unload_jz_pcm(struct jz_pcm_controller_info *controller)
{
	int adev = controller->dev_audio;
	
	__pcm_reset();
	schedule_timeout(5);
	__pcm_disable();
	__pcm_clk_disable();
	__pcm_flush_fifo();
	
	controller->dev_audio = -1;
	jz_free_dma(controller->dma1);
	jz_free_dma(controller->dma2);
	free_pages((unsigned long)controller->tmp1, JZCODEC_USER_BUFFER);
	
	if (adev >= 0)
		unregister_sound_dsp(controller->dev_audio);
}

static int __init init_jz_pcm(void)
{
	int errno;
	/* 24M OSC ---> CPCCR.ECS ---> PCMCDR.PCMS ---> cpm_pcm_sysclk(X) */
	long X = 12000000; //in Hz /* 6.144 MHz <= X <= 264.192 MHz */
	
#if 0
	/* pcm_sys_clk is from PLL divsion */
	REG_CPM_PCMCDR = 0x8000001b;
	REG_CPM_CPCCR |= 0x00400000;
#endif
	/* pcm_sys_clk is from external clock */
	/* reset codec GPF4 */
	__gpio_as_output(32 * 5 + 4);
	__gpio_set_pin(32 * 5 + 4);
	mdelay(1);

	__pcm_reset();
	schedule_timeout(5);
	__pcm_clk_disable();

	/* set CPM to output cpm-pcm-sysclk ,assume cpm-pcm-sysclk is X Hz */
	/* PCMCLK must be 2048000 Hz, it is 256 mutil of PCMSYNC */
	//__pcm_set_clk_rate(X, 2048000);
	__pcm_set_clk_rate(X, 2000000);
	/* PCMSYNC must be 8000 Hz. 2048000 / 256 = 8000 */
	__pcm_set_sync_rate(2048000, 8000);
	//__pcm_set_sync_rate(2000000, 8000);
	__pcm_set_sync_len(0);
	
	__pcm_flush_fifo();
	__pcm_disable_txfifo();
	__pcm_disable_rxfifo();
	__pcm_set_transmit_trigger(7);
	__pcm_set_receive_trigger(7);
	__pcm_omsb_next_sync();
	__pcm_imsb_next_sync();

#if MODE_is_8
	__pcm_set_iss(8);//8bits decided by LINSEL
	__pcm_set_oss(8);
#else
	__pcm_set_iss(16);//16bits decided by LINSEL
	__pcm_set_oss(16);
#endif
	__pcm_set_valid_slot(1);

	__pcm_disable_tfs_intr();
	__pcm_disable_tur_intr();
	__pcm_disable_rfs_intr();
	__pcm_disable_ror_intr();

	__pcm_disable_receive_dma();
	__pcm_disable_transmit_dma();

	__pcm_last_sample();
	__pcm_as_master();

	__pcm_enable();
	__pcm_clk_enable();
	
	if ((errno = probe_jz_pcm(&pcm_controller)) < 0)
		return errno;
	attach_jz_pcm(pcm_controller);

	out_empty_queue.id = NULL;
	out_full_queue.id = NULL;
	out_busy_queue.id = NULL;
	in_empty_queue.id = NULL;
	in_full_queue.id = NULL;
	in_busy_queue.id = NULL;

	jz_audio_fragsize = JZCODEC_RW_BUFFER_SIZE * PAGE_SIZE;
	jz_audio_fragstotal = JZCODEC_RW_BUFFER_TOTAL ;
	Init_In_Out_queue(jz_audio_fragstotal,jz_audio_fragsize);
       
	__gpio_clear_pin(32 * 5 + 4);
	udelay(100);
      
	__gpio_set_pin(32 * 5 + 4);
	dump_pcmc_reg();

	return 0;
}


static void __exit cleanup_jz_pcm(void)
{
	unload_jz_pcm(pcm_controller);
	Free_In_Out_queue(jz_audio_fragstotal,jz_audio_fragsize); 
}

module_init(init_jz_pcm);
module_exit(cleanup_jz_pcm);

static int drain_adc(struct jz_pcm_controller_info *ctrl, int nonblock)
{
	unsigned long flags;
	int count, i=0;
	
	for (;;) {
		if ( i < MAXDELAY ) {
			udelay(10);
			i++;
		} else
			break; 
		
		spin_lock_irqsave(&ctrl->lock, flags);
		count = get_dma_residue(ctrl->dma2);
		spin_unlock_irqrestore(&ctrl->lock, flags);
		if (count <= 0)
			break;
		
		if (nonblock)
			return -EBUSY;
	}
	
	return 0;
}
static int drain_dac(struct jz_pcm_controller_info *ctrl, int nonblock)
{
	unsigned long flags;
	int count, ele, i=0;

	for (;;) {
		if(!nonblock) {//blocked
               		if ( i < MAXDELAY ) {
				udelay(10);
				i++;
	       		} else
		  		break; 
			
			ele = elements_in_queue(&out_full_queue);
			if(ele <= 0) {
				udelay(10);
				spin_lock_irqsave(&ctrl->lock, flags);
				count = get_dma_residue(ctrl->dma1);
				spin_unlock_irqrestore(&ctrl->lock, flags);
				if (count <= 0)
					break;
			}
		} else {//non-blocked
			mdelay(100);
			ele = elements_in_queue(&out_full_queue);
			
			if(ele <= 0) {
				mdelay(100); 
				
				spin_lock_irqsave(&ctrl->lock, flags);
				count = get_dma_residue(ctrl->dma1);
				spin_unlock_irqrestore(&ctrl->lock, flags);
				if (count <= 0)
					break;
			}
		}
	}

	return 0;
}

static int jz_audio_release(struct inode *inode, struct file *file)
{
	struct jz_pcm_controller_info *controller = (struct jz_pcm_controller_info *) file->private_data;
   
	if (controller == NULL)
		return -ENODEV;

	if ( controller->opened1 == 1  ) {
		__pcm_enable_transmit_dma();
		__pcm_enable_txfifo();
		drain_dac(controller, file->f_flags & O_NONBLOCK);
		disable_dma(controller->dma1);
		set_dma_count(controller->dma1, 0);
       
		__pcm_disable_transmit_dma();
		__pcm_disable_txfifo();

		spin_lock(&controller->ioctllock);
		controller->total_bytes = 0;
		controller->count = 0;
		controller->finish = 0;
		jz_audio_dma_tran_count = 0;
		controller->blocks = 0;
		controller->nextOut = 0;
		spin_unlock(&controller->ioctllock);
		//__pcm_disable();
		controller->opened1 = 0;
	}
	
	if ( controller->opened2 == 1  ) {
		first_record_call = 1;
		__pcm_enable_receive_dma();
		__pcm_enable_rxfifo();
		drain_adc(controller, file->f_flags & O_NONBLOCK);
		disable_dma(controller->dma2);
		set_dma_count(controller->dma2, 0);
		__pcm_disable_receive_dma();
		__pcm_disable_rxfifo();

		spin_lock(&controller->ioctllock);
		controller->total_bytes = 0;
		jz_audio_dma_tran_count = 0;
		controller->count = 0;
		controller->finish = 0;
		controller->blocks = 0;
		controller->nextIn = 0;
		spin_unlock(&controller->ioctllock);
		//__pcm_disable();
		controller->opened2 = 0;
	}
	__pcm_disable_tfs_intr();
	__pcm_disable_tur_intr();
	__pcm_disable_rfs_intr();
	__pcm_disable_ror_intr();
	
	return 0;
}

static int jz_audio_open(struct inode *inode, struct file *file)
{
	int i;
	struct jz_pcm_controller_info *controller = pcm_controller;

	if (controller == NULL)
		return -ENODEV;
	
	if (controller->opened1 == 1 || controller->opened2 == 1 ) {
		printk("\naudio is busy!\n");
		return -EBUSY;
	}
	REG_DMAC_DMACKE(0) = 0x3f;
	REG_DMAC_DMACKE(1) = 0x3f;
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

		out_empty_queue.count = jz_audio_fragstotal;	
		for (i=0;i < jz_audio_fragstotal;i++)
			*(out_empty_queue.id + i) = i;
		out_busy_queue.count = 0;
		out_full_queue.count = 0;
		/* set PCMOUT params */
	}

	if (file->f_mode & FMODE_READ) {
		if (controller->opened2 == 1)
			return -EBUSY;
		controller->opened2 = 1;
		first_record_call = 1;
		//for ioctl
		controller->total_bytes = 0;
		jz_audio_dma_tran_count = 0;
		controller->count = 0;
		controller->finish = 0;
		controller->blocks = 0;
		controller->nextIn = 0;
		in_empty_queue.count = jz_audio_fragstotal;
		for (i=0;i < jz_audio_fragstotal;i++)
			*(in_empty_queue.id + i) = i;
		in_full_queue.count = 0;
		in_busy_queue.count = 0;
		/* set PCMIN params */
	}

	file->private_data = controller;
	jz_audio_reset();
	__pcm_enable();

	return 0;
}

static int jz_audio_ioctl(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	int val,fullc,busyc,unfinish,newfragstotal,newfragsize;
	unsigned long flags;
        audio_buf_info abinfo;
        int i, bytes, id;         
	count_info cinfo;
	struct jz_pcm_controller_info *controller = (struct jz_pcm_controller_info *) file->private_data;
	
	val = 0;
	bytes = 0;
	switch (cmd) {
	case OSS_GETVERSION:
		return put_user(SOUND_VERSION, (int *)arg);
	case SNDCTL_DSP_RESET:
		return 0;
	case SNDCTL_DSP_SYNC:
		if (file->f_mode & FMODE_WRITE)
			drain_dac(controller, file->f_flags & O_NONBLOCK);
		return 0;
	case SNDCTL_DSP_SPEED:
	{
		if (get_user(val, (int *)arg))
			return -EFAULT;
		/* set smaple rate */
		if (val >= 0)
			jz_audio_set_speed(controller->dev_audio, val);
		return put_user(val, (int *)arg);
	}
	case SNDCTL_DSP_STEREO:
		/* set stereo or mono channel */
		if (get_user(val, (int *)arg))
			return -EFAULT;

		jz_audio_set_channels(controller->dev_audio, val ? 2 : 1);
		return 0;
		
	case SNDCTL_DSP_GETBLKSIZE:
		//return put_user(4*PAGE_SIZE, (int *)arg);
		return put_user(jz_audio_fragsize , (int *)arg);
	case SNDCTL_DSP_GETFMTS:
		/* Returns a mask of supported sample format*/
		return put_user(AFMT_U8 | AFMT_S16_LE, (int *)arg);
		
	case SNDCTL_DSP_SETFMT:
	{
		if (get_user(val, (int *)arg))
			return -EFAULT;
		/* Select sample format */
		if (val != AFMT_QUERY)
			jz_audio_set_format(controller->dev_audio,val);
		else
			if (file->f_mode & FMODE_READ)
				val = (jz_audio_format == 16) ?  AFMT_S16_LE : AFMT_U8;
			else
				val = (jz_audio_format == 16) ?  AFMT_S16_LE : AFMT_U8;
		return put_user(val, (int *)arg);
	}
	
	case SNDCTL_DSP_CHANNELS:
		if (get_user(val, (int *)arg))
			return -EFAULT;
		printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
		jz_audio_set_channels(controller->dev_audio, val);
		return put_user(val, (int *)arg);
		
	case SNDCTL_DSP_POST:
		/* FIXME: the same as RESET ?? */
		return 0;
		
	case SNDCTL_DSP_SUBDIVIDE:
		return 0;
		
	case SNDCTL_DSP_SETFRAGMENT:
		get_user(val, (long *) arg);
		newfragsize = 1 << (val & 0xFFFF);//16 least bits
		
		if (newfragsize < 4 * PAGE_SIZE)
			newfragsize = 4 * PAGE_SIZE;
		if (newfragsize > (16 * PAGE_SIZE))  //16 PAGE_SIZE
			newfragsize = 16 * PAGE_SIZE;
		
		newfragstotal = (val >> 16) & 0x7FFF;
		if (newfragstotal < 2)
			newfragstotal = 2;
		if (newfragstotal > 32)
			newfragstotal = 32;
		if((jz_audio_fragstotal == newfragstotal) && (jz_audio_fragsize == newfragsize))
			return 0;
		Free_In_Out_queue(jz_audio_fragstotal,jz_audio_fragsize);
		mdelay(500);
		jz_audio_fragstotal = newfragstotal;
		jz_audio_fragsize = newfragsize;
	       
		Init_In_Out_queue(jz_audio_fragstotal,jz_audio_fragsize);
		mdelay(10);

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
		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		//unused fragment amount
		spin_lock_irqsave(&controller->ioctllock, flags);
		jz_audio_fragments = elements_in_queue(&out_empty_queue);
		for (i = 0; i < jz_audio_fragments; i++)
			bytes += jz_audio_fragsize;

		spin_unlock_irqrestore(&controller->ioctllock, flags);
		abinfo.fragments = jz_audio_fragments;
		abinfo.fragstotal = jz_audio_fragstotal;
		abinfo.fragsize = jz_audio_fragsize;
		abinfo.bytes = bytes;
		return copy_to_user((void *)arg, &abinfo, sizeof(abinfo)) ? -EFAULT : 0;
	}
	case SNDCTL_DSP_GETISPACE:
		if (!(file->f_mode & FMODE_READ))
			return -EINVAL;
		bytes = 0;
		//unused fragment amount
		jz_audio_fragments = elements_in_queue(&in_empty_queue);
		for (i = 0; i < jz_audio_fragments; i++)
			bytes += jz_audio_fragsize;
		
		abinfo.fragments = jz_audio_fragments;
		abinfo.fragstotal = jz_audio_fragstotal;
		abinfo.fragsize = jz_audio_fragsize;
		abinfo.bytes = bytes;
		return copy_to_user((void *)arg, &abinfo, sizeof(abinfo)) ? -EFAULT : 0;
	case SNDCTL_DSP_GETTRIGGER:
		val = 0;
		if (file->f_mode & FMODE_READ && in_dma_buf)
			val |= PCM_ENABLE_INPUT;
		if (file->f_mode & FMODE_WRITE && out_dma_buf)
			val |= PCM_ENABLE_OUTPUT;
		return put_user(val, (int *)arg);
		
	case SNDCTL_DSP_SETTRIGGER:
		if (get_user(val, (int *)arg))
			return -EFAULT;
		return 0;
	case SNDCTL_DSP_GETIPTR:
		if (!(file->f_mode & FMODE_READ))
			return -EINVAL;

		spin_lock_irqsave(&controller->ioctllock, flags);
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
		cinfo.bytes = controller->total_bytes;
		cinfo.blocks = controller->blocks;
		cinfo.ptr = controller->nextOut;
		controller->blocks = 0;
		spin_unlock_irqrestore(&controller->ioctllock, flags);
		return copy_to_user((void *) arg, &cinfo, sizeof(cinfo));
	case SNDCTL_DSP_GETODELAY:
		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		spin_lock_irqsave(&controller->ioctllock, flags);
		unfinish = 0;
		fullc = elements_in_queue(&out_full_queue);
		busyc = elements_in_queue(&out_busy_queue);
		for(i = 0;i < fullc ;i ++) {
			id = *(out_full_queue.id + i);
			unfinish += *(out_dma_buf_data_count + id); 
		}
		for(i = 0;i < busyc ;i ++) {
			id = *(out_busy_queue.id + i);
			unfinish += get_dma_residue(controller->dma1);
		}
		spin_unlock_irqrestore(&controller->ioctllock, flags);
		return put_user(unfinish, (int *) arg);
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

static unsigned int jz_audio_poll(struct file *file,struct poll_table_struct *wait)
{
	unsigned long flags;
	unsigned int mask = 0;
	struct jz_pcm_controller_info *controller = (struct jz_pcm_controller_info *) file->private_data;
	
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
	} else if (file->f_mode & FMODE_READ) {
		if (elements_in_queue(&in_full_queue) > 0)
			mask |= POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&controller->lock, flags);

	return mask;
}

static ssize_t jz_audio_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	int id, ret = 0, left_count, copy_count, cnt = 0;
	struct jz_pcm_controller_info *controller = (struct jz_pcm_controller_info *) file->private_data;

	if (count < 0)
		return -EINVAL;

	__pcm_enable_receive_dma();
	__pcm_enable_rxfifo();
	
	spin_lock(&controller->ioctllock);
	controller->nextIn = 0;
	spin_unlock(&controller->ioctllock);

	spin_lock(&controller->lock);

#if 0
	if (count < 2 * PAGE_SIZE )
		copy_count = count * 16 / (jz_audio_channels * jz_audio_format);
	else
		copy_count = 2 * PAGE_SIZE ;
#else
	if (count <= jz_audio_fragsize)
		copy_count = count;
	else
		copy_count = jz_audio_fragsize;
#endif

	left_count = count;
	spin_unlock(&controller->lock);
    
	if (first_record_call) {
		first_record_call = 0;
audio_read_back_first:
		if ((id = get_buffer_id(&in_empty_queue)) >= 0) {
			put_buffer_id(&in_busy_queue, id);
			
			spin_lock(&controller->lock); 
			*(in_dma_buf_data_count + id) = copy_count;
			spin_unlock(&controller->lock);

			dma_cache_wback_inv(*(in_dma_buf + id), *(in_dma_buf_data_count + id));
			audio_start_dma(controller->dma2,file->private_data,
					*(in_dma_pbuf + id),
					*(in_dma_buf_data_count + id),
					DMA_MODE_READ);
			
			sleep_on(&rx_wait_queue);
		} else
			goto audio_read_back_first;
	}

	while (left_count > 0) {
audio_read_back_second:
		if (elements_in_queue(&in_full_queue) <= 0) {
			if (file->f_flags & O_NONBLOCK)
				return ret ? ret : -EAGAIN;
			else
				sleep_on(&rx_wait_queue);
		}

		if ((id = get_buffer_id(&in_full_queue)) >= 0) {
			spin_lock(&controller->lock);
			cnt = record_filler((unsigned long)controller->tmp1+ret, copy_count, id);
			spin_unlock(&controller->lock);
			put_buffer_id(&in_empty_queue, id);
		} else
			goto audio_read_back_second;
		
		if (elements_in_queue(&in_busy_queue) == 0) {
			if ((id=get_buffer_id(&in_empty_queue)) >= 0) {
				put_buffer_id(&in_busy_queue, id);
				spin_lock(&controller->lock);
				*(in_dma_buf_data_count + id) = copy_count;
				spin_unlock(&controller->lock);

				dma_cache_wback_inv(*(in_dma_buf + id), *(in_dma_buf_data_count + id));
				audio_start_dma(controller->dma2,file->private_data,
						*(in_dma_pbuf + id),
						*(in_dma_buf_data_count + id),
						DMA_MODE_READ);
			}
		}
		if (ret + cnt > count) {
			spin_lock(&controller->lock);
			cnt = count - ret;
			spin_unlock(&controller->lock);
		}
		if (copy_to_user(buffer+ret, controller->tmp1+ret, cnt))
			return ret ? ret : -EFAULT;
		spin_lock(&controller->lock);
		ret += cnt;
		spin_unlock(&controller->lock);

		spin_lock(&controller->ioctllock);
		controller->nextIn += ret;
		spin_unlock(&controller->ioctllock);

		spin_lock(&controller->lock);
		left_count -= cnt;
		spin_unlock(&controller->lock);
	}
 
	return ret;
}

static ssize_t jz_audio_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	int id, ret, left_count, copy_count;
	unsigned long flags;
	struct jz_pcm_controller_info *controller = (struct jz_pcm_controller_info *) file->private_data;
	
	if (count <= 0)
		return -EINVAL;
	
	__pcm_enable_transmit_dma();
	__pcm_enable_txfifo();
	
	spin_lock_irqsave(&controller->ioctllock, flags);
	controller->nextOut = 0;
	spin_unlock_irqrestore(&controller->ioctllock, flags);

#if 0
	if (count < 2 * PAGE_SIZE )
		copy_count = count;
	else
		copy_count = 2 * PAGE_SIZE;
#else
	if (count <= jz_audio_fragsize)
		copy_count = count;
	else
		copy_count = jz_audio_fragsize;	
#endif

	left_count = count;
	ret = 0;

	if (copy_from_user(controller->tmp1, buffer, count)) {
		printk("copy_from_user failed:%d",ret);
		return ret ? ret : -EFAULT;
	}
	
	while (left_count > 0) {
audio_write_back:
		if (elements_in_queue(&out_empty_queue) == 0) {
			// all are full
			if (file->f_flags & O_NONBLOCK)
				return ret;
			else
				sleep_on(&tx_wait_queue);
		}
		/* the end fragment size in this write */
		if (ret + copy_count > count)
			copy_count = count - ret;
		if ((id = get_buffer_id(&out_empty_queue)) >= 0) {
			replay_filler((unsigned long)controller->tmp1 + ret, copy_count, id);
			if(*(out_dma_buf_data_count + id) > 0) {
				put_buffer_id(&out_full_queue, id); //busy in
				dma_cache_wback_inv(*(out_dma_buf + id), *(out_dma_buf_data_count + id));
			} else
				put_buffer_id(&out_empty_queue, id); //spare
		} else
			goto audio_write_back;
		
		left_count = left_count - copy_count;
		ret += copy_count;//all is in byte
		
		spin_lock(&controller->ioctllock);
		controller->nextOut += ret;
		spin_unlock(&controller->ioctllock);

		if (elements_in_queue(&out_busy_queue) == 0) {
			if ((id=get_buffer_id(&out_full_queue)) >= 0) {
				put_buffer_id(&out_busy_queue, id);
				
				if(*(out_dma_buf_data_count + id) > 0) {
					audio_start_dma(controller->dma1,
							file->private_data,
							*(out_dma_pbuf + id),
							*(out_dma_buf_data_count + id),
							DMA_MODE_WRITE);
					
				}
			}
		}
	}

	return ret;
}

static void dump_pcmc_reg(void)
{
	printk("REG_DMAC_DMACKE(0) : 0x%08x\n",REG_DMAC_DMACKE(0));
	printk("REG_DMAC_DMACKE(1) : 0x%08x\n",REG_DMAC_DMACKE(1));
	printk("REG_CPM_CLKGR=0x%08x\n",REG_CPM_CLKGR);
	printk("REG_CPM_CPCCR : 0x%08x\n",REG_CPM_CPCCR);
	printk("REG_CPM_PCMCDR : 0x%08x\n",REG_CPM_PCMCDR);
	printk("REG_PCM_CLT : 0x%08x\n",REG_PCM_CTL);
	printk("REG_PCM_CFG : 0x%08x\n",REG_PCM_CFG);
	printk("REG_PCM_INTC : 0x%08x\n",REG_PCM_INTC);
	printk("REG_PCM_INTS : 0x%08x\n",REG_PCM_INTS);
	printk("REG_PCM_DIV : 0x%08x\n",REG_PCM_DIV);
}
