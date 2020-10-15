/*
 *  linux/drivers/sound/jz_spdif.c
 *
 *  JzSOC On-Chip SPDIF audio driver.
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

/*
 *  spdif kernel driver test for 24 bits transfer
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
//#include "jzhdmi_4760.h"

#define NR_SPDIF	2
#define MAXDELAY 50000
#define JZCODEC_RW_BUFFER_SIZE       11
#define JZCODEC_RW_BUFFER_TOTAL      4
#define JZCODEC_USER_BUFFER     10


static int		jz_audio_rate;
static char		jz_audio_format;
static char		jz_audio_volume;
static char		jz_audio_channels;
static int              jz_audio_fragments;//unused fragment amount
static int              jz_audio_fragstotal;
static int              jz_audio_fragsize;
static int              jz_audio_speed;
static int              jz_audio_dma_tran_count;//bytes count of one DMA transfer
static int              jz_print_count;
static int              jz_audio_print;
static int              pop_dma_flag;

static void jz_update_filler(int bits, int channels);
static int Init_In_Out_queue(int fragstotal,int fragsize);
static int Free_In_Out_queue(int fragstotal,int fragsize);
static void dump_spdif_reg(void);
static irqreturn_t jz_spdif_replay_dma_irq(int irqnr, void *ref);
static void (*replay_filler)(unsigned long src_start, int count, int id);

static struct file_operations jz_spdif_audio_fops;
static DECLARE_WAIT_QUEUE_HEAD (tx_wait_queue);
static DECLARE_WAIT_QUEUE_HEAD (pop_wait_queue);

struct jz_spdif_controller_info {
	int io_base;
	int dma1; /* play */
	char *name;
	int dev_audio;
	struct spdif_codec *spdif_codec[NR_SPDIF];
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
static struct jz_spdif_controller_info *spdif_controller = NULL;

struct spdif_codec {
	/* SPDIF controller connected with */
	void *private_data;
	char *name;
	int id;
	int dev_mixer;
	/* controller specific lower leverl spdif accessing routines */
	u16  (*codec_read)  (u8 reg);//the function accessing Codec REGs
	void (*codec_write) (u8 reg, u16 val);
	/* Wait for codec-ready.  Ok to sleep here.  */
	void  (*codec_wait)  (struct spdif_codec *codec);
	/* OSS mixer masks */
	int modcnt;
	int supported_mixers;
	int stereo_mixers;
	int record_sources;
	int bit_resolution;
	/* OSS mixer interface */
	int  (*read_mixer) (struct spdif_codec *codec, int oss_channel);
	void (*write_mixer)(struct spdif_codec *codec, int oss_channel,
			    unsigned int left, unsigned int right);
	int  (*recmask_io) (struct spdif_codec *codec, int rw, int mask);
	int  (*mixer_ioctl)(struct spdif_codec *codec, unsigned int cmd, unsigned long arg);
	/* saved OSS mixer states */
	unsigned int mixer_state[SOUND_MIXER_NRDEVICES];
};

typedef struct buffer_queue_s {
	int count;
	int *id;
	int lock;
} buffer_queue_t;

static unsigned long pop_turn_onoff_buf;
static unsigned long pop_turn_onoff_pbuf;

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
	struct jz_spdif_controller_info * controller = (struct jz_spdif_controller_info *) dev_id;

	//for DSP_GETOPTR
	//spin_lock_irqsave(&controller->ioctllock, flags);
	//printk("count=%d\n",count);
	spin_lock(&controller->ioctllock);
	jz_audio_dma_tran_count = count;
	//spin_unlock_irqrestore(&controller->ioctllock, flags);
	spin_unlock(&controller->ioctllock);
	flags = claim_dma_lock();
	disable_dma(chan);
	clear_dma_ff(chan);
	set_dma_mode(chan, mode);

#if 1
	/* for 16 bits */
	__dmac_channel_set_src_port_width(chan, 32/*32 good*/);
	__dmac_channel_set_dest_port_width(chan, 32);
	//__dmac_channel_set_transfer_unit_16bit(chan);
	//__dmac_channel_set_transfer_unit_32bit(chan);// ka
	//__dmac_channel_set_transfer_unit_16byte(chan);
	__dmac_channel_set_transfer_unit_32byte(chan);

#else
	/* for 24 bits */
	__dmac_channel_set_src_port_width(chan, 32);
	__dmac_channel_set_dest_port_width(chan, 32);
	//__dmac_channel_set_transfer_unit_16bit(chan);
	//__dmac_channel_set_transfer_unit_32bit(chan);
	//__dmac_channel_set_transfer_unit_16byte(chan);
	__dmac_channel_set_transfer_unit_32byte(chan);
#endif

	set_dma_addr(chan, phyaddr);
	if (count == 0) {
		count++;
		printk("JzSOC DMA controller can't set dma 0 count!\n");
	}

	set_dma_count(chan, count);
	dump_jz_dma_channel(chan);
	enable_dma(chan);
	__dmac_enable_module(0);
	release_dma_lock(flags);

}

static irqreturn_t jz_spdif_replay_dma_irq(int irq, void *dev_id)
{
	int id;
	unsigned long flags;
	struct jz_spdif_controller_info * controller = (struct jz_spdif_controller_info *) dev_id;
	int dma = controller->dma1;
	printk("--- dma irq ---\n");
	disable_dma(dma);
	if (__dmac_channel_address_error_detected(dma)) {
		printk(KERN_DEBUG "%s: DMAC address error.\n", __FUNCTION__);
		__dmac_channel_clear_address_error(dma);
	}
	if (__dmac_channel_transmit_end_detected(dma)) {
		__dmac_channel_clear_transmit_end(dma);

		if(pop_dma_flag == 1) {
			pop_dma_flag = 0;
			wake_up(&pop_wait_queue);
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
#if 0
static void set_dlv_speed(int rate)
{
	int speed = 0, val;
	speed = 0;

	switch (rate) {
	case 8000:
		speed = 10;
		break;
	case 9600:
		speed = 9;
		break;
	case 11025:
		speed = 8;
		break;
	case 12000:
		speed = 7;
		break;
	case 16000:
		speed = 6;
		break;
	case 22050:
		speed = 5;
		break;
	case 24000:
		speed = 4;
		break;
	case 32000:
		speed = 3;
		break;
	case 44100:

		speed = 2;
		break;
	case 48000:
		speed = 1;
		break;
	case 96000:
		speed = 0;
		break;
	default:
		break;
	}

#if 0
	val = read_codec_file(4);
	val = (speed << 4) | speed;

	//write_codec_file(4, val);
#endif
}
#endif

static void set_spdif_speed(int rate)
{
	int fs = 0, orgfs = 0;

	switch (rate) {
	case 8000:
		fs = 0x8;
		orgfs = 0x6;
		break;
	case 44100:
		fs = 0x0;
		orgfs = 0xf;
		break;
	case 48000:
		fs = 0x2;
		orgfs = 0xd;
		break;
	case 32000:
		fs = 0x3;
		orgfs = 0xc;
		break;
	case 96000:
		fs = 0xa;
		orgfs = 0x5;
		break;
	case 192000:
		fs = 0x6;
		orgfs = 0x1;
		break;
	default:
		break;
	}
	__spdif_set_fs(fs);
	__spdif_set_orgfrq(orgfs);
}

static int get_int(int a, int b)
{
	int c = a / b;
	int d = a % b;

	int addval = a - c * b;
	int musval = (c + 1) * b - a;

	if (d != 0) {
		if (addval > musval)
			c ++;
	}

	return c;
}

static void set_spdif_div(int rate)
{
	int sys_clk = 12 * 1000 * 1000;
	int song_clk = rate;
	int sync_clk = song_clk * 2;
	int bit_clk = sync_clk * 64;
	int dv = get_int(sys_clk, bit_clk) - 1;
	printk("sys_clk=%d  bit_clk=%d  dv=%d\n",sys_clk,bit_clk,dv);
	REG_AIC_I2SDIV = dv;

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
	printk("%s:%s:%d  rate=%d\n",__FILE__,__FUNCTION__,__LINE__,rate);


	//set_dlv_speed(rate);
	set_spdif_speed(rate);
	set_spdif_div(rate);

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
	volatile unsigned short *s = (unsigned short *)src_start;
	volatile unsigned short *dp = (unsigned short*)(*(out_dma_buf + id));

	while (count > 0) {
#if 0
		*dp = *s;
		s ++;
		dp ++;

		count -= 2;
		cnt += 2 ;
#else
		/////////// mono is good
		d1 = *s;
		*dp = d1;
		//printk("\n1 s=0x%08x  dp=0x%08x  *s=0x%08x  *dp=0x%08x\n",s,dp,*s,*dp);
		dp ++;
		//printk("2 dp=0x%08x  \n",dp);
		*dp = d1;
		s ++;
		//printk("3 s=0x%08x  *dp=0x%08x\n",s,*dp);
		dp ++;
		//printk("4 dp=0x%08x  \n",dp);
		count -= 2;
		cnt += 4 ;

		///////////
#endif
	}

	*(out_dma_buf_data_count + id) = cnt;
}

static void replay_fill_2x16_s(unsigned long src_start, int count, int id)
{
	int cnt = 0;
	unsigned short d1, l1;
	volatile unsigned short *s = (unsigned short *)src_start;
	volatile unsigned short *dp = (unsigned short*)(*(out_dma_buf + id));
	int aaa;
	int count_val = count;
	unsigned char *src_ch = (unsigned char*)src_start;
	unsigned char *dst_ch = (unsigned char*)(*(out_dma_buf + id));
#if 1
	while (count > 0) {
		d1 = *s;
		//l1 = (d1 & 0xfff8) | jz_audio_volume;
		l1 = d1;
		*dp = l1;
		s ++;
		dp ++;
		count -= 2;
		cnt += 2 ;
	}
	*(out_dma_buf_data_count + id) = cnt;

	/*jz_print_count++;
	printk("\n\n-------- 2x16   %d --------\n\n",jz_print_count);
	for (aaa = 0; aaa < 4; aaa++) {
		printk("src_ch[%d] = 0x%x\n",aaa,src_ch[aaa]);
	}
	for (aaa = count_val - 4; aaa < count_val; aaa++) {
		printk("src_ch[%d] = 0x%x\n",aaa,src_ch[aaa]);
	}
	for (aaa = 0; aaa < 4; aaa++) {
		printk("dst_ch[%d] = 0x%x\n",aaa,dst_ch[aaa]);
	}
	for (aaa = cnt - 4; aaa < cnt; aaa++) {
		printk("dst_ch[%d] = 0x%x\n",aaa,dst_ch[aaa]);
		}*/
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

#if 0
static void replay_fill_2x24_s(unsigned long src_start, int count, int id)
{
	int cnt = 0;
	signed short d1;
	signed long l1;
	volatile signed short *s = (signed short *)src_start;
	volatile signed long *dp = (signed long*)(*(out_dma_buf + id));
	int aaa;
	int count_val = count;
	unsigned char *src_ch = (unsigned char*)src_start;
	unsigned char *dst_ch = (unsigned char*)(*(out_dma_buf + id));

	while (count > 0) {
		count -= 2;
		cnt += 4;
		//printk("\n\ns=0x%08x  dp=0x%08x\n",s,dp);
		//d1 = *(s++);
		d1 = *s;
		s++;

		l1 = (signed long)d1;

		//*(dp ++) = l1;
		*dp = l1;
		dp ++;
		//printk("d1=0x%08x  l1=0x%08x\n",d1,l1);
	}

	*(out_dma_buf_data_count + id) = cnt;

	/*jz_print_count++;
	printk("\n\n-------- %d --------\n\n",jz_print_count);
	for (aaa = 0; aaa < 4; aaa++) {
		printk("src_ch[%d] = 0x%x\n",aaa,src_ch[aaa]);
	}
	for (aaa = count_val - 4; aaa < count_val; aaa++) {
		printk("src_ch[%d] = 0x%x\n",aaa,src_ch[aaa]);
	}
	for (aaa = 0; aaa < 8; aaa++) {
		printk("dst_ch[%d] = 0x%x\n",aaa,dst_ch[aaa]);
	}
	for (aaa = cnt - 8; aaa < cnt; aaa++) {
		printk("dst_ch[%d] = 0x%x\n",aaa,dst_ch[aaa]);
		}*/

}
#else
static void replay_fill_2x24_s(unsigned long src_start, int count, int id)
{
	int cnt = 0;
	//signed short d1;
	//signed long l1;
	unsigned char *s = (unsigned char *)src_start;
	unsigned char *dp = (unsigned char *)(*(out_dma_buf + id));

	/*while (count > 0) {
		count -= 2;
		cnt += 4;

		d1 = *s;
		s++;

		l1 = (signed long)d1;

		*dp = l1;
		dp ++;
		}*/
	memcpy(dp, s, count);

	*(out_dma_buf_data_count + id) = count;
}
#endif

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

static int jz_audio_release(struct inode *inode, struct file *file);
static int jz_audio_open(struct inode *inode, struct file *file);
static int jz_audio_ioctl(struct inode *inode, struct file *file,unsigned int cmd, unsigned long arg);
static unsigned int jz_audio_poll(struct file *file,struct poll_table_struct *wait);
static ssize_t jz_audio_write(struct file *file, const char *buffer,size_t count, loff_t *ppos);

static struct file_operations jz_spdif_audio_fops =
{
	owner:              THIS_MODULE,
	open:               jz_audio_open,
	release:            jz_audio_release,
	write:              jz_audio_write,
	poll:               jz_audio_poll,
	ioctl:              jz_audio_ioctl
};

static int jz_spdif_open_mixdev(struct inode *inode, struct file *file)
{
    int i;
    int minor = MINOR(inode->i_rdev);
    struct jz_spdif_controller_info *controller = spdif_controller;

    for (i = 0; i < NR_SPDIF; i++)
	    if (controller->spdif_codec[i] != NULL && controller->spdif_codec[i]->dev_mixer == minor)
		    goto match;

    if (!controller)
	    return -ENODEV;
match:
    file->private_data = controller->spdif_codec[i];

    return 0;
}

static int jz_spdif_ioctl_mixdev(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct spdif_codec *codec = (struct spdif_codec *)file->private_data;
	return codec->mixer_ioctl(codec, cmd, arg);
}

static loff_t jz_spdif_llseek(struct file *file, loff_t offset, int origin)
{
	return -ESPIPE;
}

static struct file_operations jz_spdif_mixer_fops =
{
	owner:		THIS_MODULE,
	llseek:		jz_spdif_llseek,
	ioctl:		jz_spdif_ioctl_mixdev,
	open:		jz_spdif_open_mixdev,
};

static int spdif_mixer_ioctl(struct spdif_codec *codec, unsigned int cmd, unsigned long arg)
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
		printk("Can not justify Mic gain to the SPDIF codec.!\n");
		return -ENOSYS;

	case SOUND_MIXER_READ_MIC:
		printk("Can not justify Mic gain to the SPDIF codec.!\n");
		return -ENOSYS;
	default:
		return -ENOSYS;
	}

	return 0;
}


int spdif_probe_codec(struct spdif_codec *codec)
{
	/* generic OSS to SPDIF wrapper */
	codec->mixer_ioctl = spdif_mixer_ioctl;
	return 1;
}

/* SPDIF codec initialisation. */
static int __init jz_spdif_codec_init(struct jz_spdif_controller_info *controller)
{
	int num_spdif = 0;
	struct spdif_codec *codec;

	for (num_spdif = 0; num_spdif < NR_SPDIF; num_spdif++) {
		if ((codec = kmalloc(sizeof(struct spdif_codec),GFP_KERNEL)) == NULL)
			return -ENOMEM;
		memset(codec, 0, sizeof(struct spdif_codec));
		codec->private_data = controller;
		codec->id = num_spdif;

		if (spdif_probe_codec(codec) == 0)
			break;
		if ((codec->dev_mixer = register_sound_mixer(&jz_spdif_mixer_fops, -1)) < 0) {
			printk(KERN_ERR "Jz SPDIF: couldn't register mixer!\n");
			kfree(codec);
			break;
		}
		controller->spdif_codec[num_spdif] = codec;
	}

	return num_spdif;
}

static void jz_update_filler(int format, int channels)
{
#define TYPE(fmt,ch) (((fmt)<<2) | ((ch)&3))

	switch (TYPE(format, channels)) {
	default:
	case TYPE(AFMT_U8, 1):
		printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
		replay_filler = replay_fill_1x8_u;
		break;
	case TYPE(AFMT_U8, 2):
		printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
		replay_filler = replay_fill_2x8_u;
		break;
	case TYPE(AFMT_S16_LE, 1):
		printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
		replay_filler = replay_fill_1x16_s;
		break;
	case TYPE(AFMT_S16_LE, 2):
		printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
#if 0
		replay_filler = replay_fill_2x16_s;
#else
		/* test 24bit sound sample */
		replay_filler = replay_fill_2x24_s;
#endif
		break;
	}
}

static void __init attach_jz_spdif(struct jz_spdif_controller_info *controller)
{
	char *name;
	int adev;//No of Audio device.
	int err;

	name = controller->name;
	/* register /dev/audio  */
	adev = register_sound_dsp(&jz_spdif_audio_fops, -1);
	if (adev < 0)
		goto audio_failed;
	/* initialize SPDIF codec and register /dev/mixer */
	if (jz_spdif_codec_init(controller) <= 0)
		goto mixer_failed;

	controller->tmp1 = (void *)__get_free_pages(GFP_KERNEL, JZCODEC_USER_BUFFER);
	if (!controller->tmp1) {
		printk(KERN_ERR "%s: can't allocate tmp buffers.\n", controller->name);
		goto tmp1_failed;
	}
	printk("DMA_ID_SPDIF_TX=0x%08x\n",DMA_ID_AIC_TX);
	if ((controller->dma1 = jz_request_dma(DMA_ID_AIC_TX, "audio dac", jz_spdif_replay_dma_irq, IRQF_DISABLED, controller)) < 0) {
		printk(KERN_ERR "%s: can't reqeust DMA DAC channel.\n", name);
		goto dma1_failed;
	}

	printk("JzSOC On-Chip SPDIF controller registered (DAC: DMA(play):%d/IRQ%d\n)", controller->dma1, get_dma_done_irq(controller->dma1));

	controller->dev_audio = adev;

	pop_turn_onoff_buf = __get_free_pages(GFP_KERNEL | GFP_DMA, JZCODEC_RW_BUFFER_SIZE);
	if(!pop_turn_onoff_buf)
		printk("pop_turn_onoff_buf alloc is wrong!\n");
	pop_turn_onoff_pbuf = virt_to_phys((void *)pop_turn_onoff_buf);


	return;
dma1_failed:
	jz_free_dma(controller->dma1);
tmp1_failed:
	free_pages((unsigned long)controller->tmp1, JZCODEC_USER_BUFFER);

mixer_failed:
	unregister_sound_dsp(adev);
audio_failed:
	return;
}


static int __init probe_jz_spdif(struct jz_spdif_controller_info **controller)
{
	if ((*controller = kmalloc(sizeof(struct jz_spdif_controller_info),
				   GFP_KERNEL)) == NULL) {
		printk(KERN_ERR "Jz SPDIF Controller: out of memory.\n");
		return -ENOMEM;
	}

	(*controller)->name = "Jz SPDIF controller";
	(*controller)->opened1 = 0;
	(*controller)->opened2 = 0;
	init_waitqueue_head(&(*controller)->adc_wait);
	init_waitqueue_head(&(*controller)->dac_wait);
	init_waitqueue_head(&pop_wait_queue);
	spin_lock_init(&(*controller)->lock);
	init_waitqueue_head(&tx_wait_queue);

	return 0;
}

static void __exit unload_jz_spdif(struct jz_spdif_controller_info *controller)
{
	int adev = controller->dev_audio;

	__spdif_enable_reset();
	while(REG_SPDIF_CTRL & SPDIF_CTRL_RST);
	__spdif_disable();
	while(REG_SPDIF_STATE & (1 << 7));

	controller->dev_audio = -1;
	jz_free_dma(controller->dma1);
	free_pages((unsigned long)controller->tmp1, JZCODEC_USER_BUFFER);
	free_pages((unsigned long)pop_turn_onoff_buf, JZCODEC_RW_BUFFER_SIZE);

	if (adev >= 0)
		unregister_sound_dsp(controller->dev_audio);
}

static int __init init_jz_spdif(void)
{
	int errno;

	cpm_start_clock(CGM_AIC);

	__gpio_as_aic();


	/* enable SYSCLK output */
	REG_AIC_I2SCR = 0x10;

	//write_codec_file(0, 0xf);
	//REG_AIC_I2SCR = 0x10;
#if 0
	__i2s_as_slave();
#else
	__i2s_as_master();
#endif
	__i2s_select_i2s();
	__aic_select_i2s();
	//__aic_reset();
	mdelay(10);
	REG_AIC_I2SCR = 0x10;
	mdelay(20);
#if 0
	__i2s_internal_codec();
#else
	__aic_external_codec();
#endif
	/* power on DLV */
	//write_codec_file(9, 0xff);
	//write_codec_file(8, 0x3f);
	mdelay(10);

	if ((errno = probe_jz_spdif(&spdif_controller)) < 0)
		return errno;

	attach_jz_spdif(spdif_controller);

	out_empty_queue.id = NULL;
	out_full_queue.id = NULL;
	out_busy_queue.id = NULL;
	in_empty_queue.id = NULL;
	in_full_queue.id = NULL;
	in_busy_queue.id = NULL;

	jz_audio_fragsize = JZCODEC_RW_BUFFER_SIZE * PAGE_SIZE;
	jz_audio_fragstotal = JZCODEC_RW_BUFFER_TOTAL ;
	Init_In_Out_queue(jz_audio_fragstotal,jz_audio_fragsize);

	//write_codec_file(0x05, 0x0f);
	//write_codec_file(0x06, 0);
	//REG_AIC_I2SCR = 0x10;
	printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
	dump_spdif_reg();

	return 0;
}


static void __exit cleanup_jz_spdif(void)
{
	unload_jz_spdif(spdif_controller);
	Free_In_Out_queue(jz_audio_fragstotal,jz_audio_fragsize);
}

module_init(init_jz_spdif);
module_exit(cleanup_jz_spdif);


static int drain_dac(struct jz_spdif_controller_info *ctrl, int nonblock)
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
	struct jz_spdif_controller_info *controller = (struct jz_spdif_controller_info *) file->private_data;

	jz_audio_print = 0;
	pop_dma_flag = 0;

	if (controller == NULL)
		return -ENODEV;

	if ( controller->opened1 == 1  ) {

		drain_dac(controller, file->f_flags & O_NONBLOCK);
		disable_dma(controller->dma1);
		set_dma_count(controller->dma1, 0);


		spin_lock(&controller->ioctllock);
		controller->total_bytes = 0;
		controller->count = 0;
		controller->finish = 0;
		jz_audio_dma_tran_count = 0;
		controller->blocks = 0;
		controller->nextOut = 0;
		spin_unlock(&controller->ioctllock);
		controller->opened1 = 0;
		__spdif_disable();
		while(REG_SPDIF_STATE & (1 << 7));
	}

	return 0;
}

static int jz_audio_open(struct inode *inode, struct file *file)
{
	int i;
	struct jz_spdif_controller_info *controller = spdif_controller;
	jz_print_count = 0;
	pop_dma_flag = 0;
	if (controller == NULL)
		return -ENODEV;
	jz_audio_print = 0;
	if (controller->opened1 == 1) {
		printk("\naudio is busy!\n");
		return -EBUSY;
	}
	//REG_DMAC_DMACKE(0) = 0x3f;
	//REG_DMAC_DMACKE(1) = 0x3f;
	printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
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
	}

	file->private_data = controller;
	printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
	if (file->f_mode & FMODE_WRITE) {
#if 0
		__spdif_disable_dtype();
#else
		__spdif_enable_dtype();
#endif
		/* cfg1 */
		__spdif_set_ch1num(0);
		__spdif_set_ch2num(1);
		__spdif_select_spdif();
		__spdif_set_srcnum(0);
		//__spdif_set_transmit_trigger(0);//4
		//__spdif_set_transmit_trigger(1);//8
		//__spdif_set_transmit_trigger(2);//16
		__spdif_set_transmit_trigger(3);//32
		__spdif_enable_zrovld_invald();
		__spdif_enable_initlvl_low();
		/* cfg2 */
		__spdif_enable_conpro();
		__spdif_enable_audion();
		__spdif_enable_copyn();
		__spdif_disable_pre();
		__spdif_set_chmode(0);
		__spdif_set_catcode(0);
		__spdif_set_clkacu(0);
#if 1
		__spdif_enable_samwl_20();
		__spdif_set_samwl(1);
#else
		__spdif_enable_samwl_24();
		__spdif_set_samwl(5);
#endif
		__spdif_set_orgfrq(0xf);
		__spdif_set_fs(0);
	}
	printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
	__spdif_enable_transmit_dma();
	__spdif_disable_sign();

	//jz_audio_reset();
	__spdif_enable_reset();
	printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);

	while(REG_SPDIF_CTRL & SPDIF_CTRL_RST);
	printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
	__spdif_enable_MTRIGmask();
	__spdif_enable_MFFURmask();
	__spdif_disable_invalid();
	__spdif_select_spdif();
	printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
	__spdif_enable();

	return 0;
}

static int jz_audio_ioctl(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	int val,fullc,busyc,unfinish,newfragstotal,newfragsize;
	unsigned int flags;
        audio_buf_info abinfo;
        int i, bytes, id;
	count_info cinfo;
	struct jz_spdif_controller_info *controller = (struct jz_spdif_controller_info *) file->private_data;

	val = 0;
	bytes = 0;
	switch (cmd) {
	case OSS_GETVERSION:
		return put_user(SOUND_VERSION, (int *)arg);
	case SNDCTL_DSP_RESET:
		return 0;
	case SNDCTL_DSP_SYNC:
		/*if (file->f_mode & FMODE_WRITE)
		  drain_dac(controller, file->f_flags & O_NONBLOCK);*/
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
		printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
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
	struct jz_spdif_controller_info *controller = (struct jz_spdif_controller_info *) file->private_data;

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


static ssize_t jz_audio_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	int id, ret, left_count, copy_count;
	unsigned int flags;
	struct jz_spdif_controller_info *controller = (struct jz_spdif_controller_info *) file->private_data;

	printk("count=%d\n",count);
	if (count <= 0)
		return -EINVAL;

	spin_lock_irqsave(&controller->ioctllock, flags);
	controller->nextOut = 0;
	spin_unlock_irqrestore(&controller->ioctllock, flags);

	left_count = count;
	ret = 0;

	if (copy_from_user(pop_turn_onoff_buf, buffer, count)) {
		printk("copy_from_user failed:%d",ret);
		return ret ? ret : -EFAULT;
	}
	dump_spdif_reg();
	dma_cache_wback_inv(pop_turn_onoff_buf, count);
	pop_dma_flag = 1;
	printk("--- play start ---\n");
	audio_start_dma(controller->dma1,controller,pop_turn_onoff_pbuf,count,DMA_MODE_WRITE);
	//sleep_on(&pop_wait_queue);
	mdelay(2500);
	printk("--- play end ---\n");
	pop_dma_flag = 0;


	/*audio_start_dma(controller->dma1,
			file->private_data,
			*(out_dma_pbuf + id),
			*(out_dma_buf_data_count + id),
			DMA_MODE_WRITE);*/


	return count;
}

static void dump_spdif_reg(void)
{
//	printk("REG_DMAC_DMACKE(0) : 0x%08x\n",REG_DMAC_DMACKE(0));
//	printk("REG_DMAC_DMACKE(1) : 0x%08x\n",REG_DMAC_DMACKE(1));

	printk("\n");
	printk("REG_AIC_FR : 0x%08x\n",REG_AIC_FR);
	printk("REG_AIC_CR : 0x%08x\n",REG_AIC_CR);
	printk("REG_AIC_I2SCR : 0x%08x\n",REG_AIC_I2SCR);
	printk("REG_AIC_I2SDIV : 0x%08x\n",REG_AIC_I2SDIV);

	printk("REG_SPDIF_ENA : 0x%08x\n",REG_SPDIF_ENA);
	printk("REG_SPDIF_CTRL : 0x%08x\n",REG_SPDIF_CTRL);
	printk("REG_SPDIF_STATE : 0x%08x\n",REG_SPDIF_STATE);
	printk("REG_SPDIF_CFG1: 0x%08x\n",REG_SPDIF_CFG1);
	printk("REG_SPDIF_CFG2 : 0x%08x\n",REG_SPDIF_CFG2);
	printk("\n");
}
