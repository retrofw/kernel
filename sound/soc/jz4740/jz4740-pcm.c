/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/io.h>
#include "jz4740-pcm.h"

static long sum_bytes = 0;
static int first_transfer = 0;
static int printk_flag = 0;
static int tran_bit = 0;
#ifdef CONFIG_SND_OSSEMUL
static int hw_params_cnt = 0;
#endif

static struct jz4740_dma_client jz4740_dma_client_out = {
	.name = "I2S PCM Stereo out"
};

static struct jz4740_dma_client jz4740_dma_client_in = {
	.name = "I2S PCM Stereo in"
};

static struct jz4740_pcm_dma_params jz4740_i2s_pcm_stereo_out = {
	.client		= &jz4740_dma_client_out,
	.channel	= DMA_ID_AIC_TX,
	.dma_addr	= AIC_DR,
	.dma_size	= 2,
};

static struct jz4740_pcm_dma_params jz4740_i2s_pcm_stereo_in = {
	.client		= &jz4740_dma_client_in,
	.channel	= DMA_ID_AIC_RX,
	.dma_addr	= AIC_DR,
	.dma_size	= 2,
};


struct jz4740_dma_buf_aic {
	struct jz4740_dma_buf_aic	*next;
	int			 size;		/* buffer size in bytes */
	dma_addr_t		 data;		/* start of DMA data */
	dma_addr_t		 ptr;		/* where the DMA got to [1] */
	void			*id;		/* client's id */
};

struct jz4740_runtime_data {
	spinlock_t lock;
	int state;
	int aic_dma_flag; /* start dma transfer or not */
	unsigned int dma_loaded;
	unsigned int dma_limit;
	unsigned int dma_period;
	dma_addr_t dma_start;
	dma_addr_t dma_pos;
	dma_addr_t dma_end;
	struct jz4740_pcm_dma_params *params;

	dma_addr_t user_cur_addr;         /* user current write buffer start address */
	unsigned int user_cur_len;        /* user current write buffer length */

	/* buffer list and information */
	struct jz4740_dma_buf_aic	*curr;		/* current dma buffer */
	struct jz4740_dma_buf_aic	*next;		/* next buffer to load */
	struct jz4740_dma_buf_aic	*end;		/* end of queue */

};

/* identify hardware playback capabilities */
static const struct snd_pcm_hardware jz4740_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
	                            SNDRV_PCM_INFO_MMAP_VALID |
				    SNDRV_PCM_INFO_INTERLEAVED |
	                            SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S8,
	.rates                  = SNDRV_PCM_RATE_8000_48000/*0x3fe*/,
	.rate_min               = 8000,
	.rate_min               = 48000,
	.channels_min		= 2,
	.channels_max		= 2,
	.buffer_bytes_max	= 128 * 1024,//16 * 1024
	.period_bytes_min	= PAGE_SIZE,
	.period_bytes_max	= PAGE_SIZE * 2,
	.periods_min		= 2,
	.periods_max		= 128,//16,
	.fifo_size		= 32,
};

/* jz4740__dma_buf_enqueue
 *
 * queue an given buffer for dma transfer.
 *
 * data       the physical address of the buffer data
 * size       the size of the buffer in bytes
 *
*/
static int jz4740_dma_buf_enqueue(struct jz4740_runtime_data *prtd, dma_addr_t data, int size)
{   
	struct jz4740_dma_buf_aic *aic_buf;

	aic_buf = kzalloc(sizeof(struct jz4740_dma_buf_aic), GFP_KERNEL);
	if (aic_buf == NULL) {
		printk("aic buffer allocate failed,no memory!\n");
		return -ENOMEM;
	}
	aic_buf->next = NULL;
	aic_buf->data = aic_buf->ptr = data;
	aic_buf->size = size;
	if( prtd->curr == NULL) {
		prtd->curr = aic_buf;
		prtd->end  = aic_buf;
		prtd->next = NULL;
	} else {
		if (prtd->end == NULL)
			printk("prtd->end is NULL\n");
			prtd->end->next = aic_buf;
			prtd->end = aic_buf;
	}

	/* if necessary, update the next buffer field */
	if (prtd->next == NULL)
		prtd->next = aic_buf;

	return 0;
}


void audio_start_dma(struct jz4740_runtime_data *prtd, int mode)
{
	unsigned long flags;
	struct jz4740_dma_buf_aic *aic_buf;
	int channel;

	switch (mode) {
	case DMA_MODE_WRITE:
		/* free cur aic_buf */
		if (first_transfer == 1) {
			first_transfer = 0;
		} else {
			aic_buf = prtd->curr;
			if (aic_buf != NULL) {
				prtd->curr = aic_buf->next;
				prtd->next = aic_buf->next;
				aic_buf->next  = NULL;
				kfree(aic_buf);
				aic_buf = NULL;
			}
		}

		aic_buf = prtd->next;		
		channel = prtd->params->channel;
		if (aic_buf) {			
			disable_dma(channel);
			jz_set_alsa_dma(channel, mode, tran_bit);
			set_dma_addr(channel, aic_buf->data);
			set_dma_count(channel, aic_buf->size);
			enable_dma(channel);
			prtd->aic_dma_flag |= AIC_START_DMA;
		} else {
			printk("next buffer is NULL for playback\n");
			prtd->aic_dma_flag &= ~AIC_START_DMA;
			return;
		}
		break;
	case DMA_MODE_READ:
                /* free cur aic_buf */
		if (first_transfer == 1) {
			first_transfer = 0;
		} else {
			aic_buf = prtd->curr;
			if (aic_buf != NULL) {
				prtd->curr = aic_buf->next;
				prtd->next = aic_buf->next;
				aic_buf->next  = NULL;
				kfree(aic_buf);
				aic_buf = NULL;
			}
		}

		aic_buf = prtd->next;
		channel = prtd->params->channel;

		if (aic_buf) {			
			disable_dma(channel);
                        jz_set_alsa_dma(channel, mode, tran_bit);
			set_dma_addr(channel, aic_buf->data);
			set_dma_count(channel, aic_buf->size);
			enable_dma(channel);
			prtd->aic_dma_flag |= AIC_START_DMA; 
		} else {
			printk("next buffer is NULL for capture\n");
			prtd->aic_dma_flag &= ~AIC_START_DMA;
			return;
		}
		break;
	}
	/* dump_jz_dma_channel(channel); */
}

/*
 * place a dma buffer onto the queue for the dma system to handle.
*/
static void jz4740_pcm_enqueue(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct jz4740_runtime_data *prtd = runtime->private_data;
	/*struct snd_dma_buffer *buf = &substream->dma_buffer;*/
	dma_addr_t pos = prtd->dma_pos;
	int ret;

	while (prtd->dma_loaded < prtd->dma_limit) {
		unsigned long len = prtd->dma_period;

		if ((pos + len) > prtd->dma_end) {
			len  = prtd->dma_end - pos;
		}
		ret = jz4740_dma_buf_enqueue(prtd, pos, len);
		if (ret == 0) {
			prtd->dma_loaded++;
			pos += prtd->dma_period;
			if (pos >= prtd->dma_end)
				pos = prtd->dma_start;
		} else 
			break;
	}

	prtd->dma_pos = pos;
}

/* 
 * call the function:jz4740_pcm_dma_irq() after DMA has transfered the current buffer  
 */
static irqreturn_t jz4740_pcm_dma_irq(int dma_ch, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct jz4740_runtime_data *prtd = runtime->private_data;
	/*struct jz4740_dma_buf_aic *aic_buf = prtd->curr;*/
	int channel = prtd->params->channel;
	unsigned long flags;

	disable_dma(channel);
	prtd->aic_dma_flag &= ~AIC_START_DMA;
	/* must clear TT bit in DCCSR to avoid interrupt again */
	if (__dmac_channel_transmit_end_detected(channel)) {
		__dmac_channel_clear_transmit_end(channel);
	}
	if (__dmac_channel_transmit_halt_detected(channel)) {
		__dmac_channel_clear_transmit_halt(channel);
	}

	if (__dmac_channel_address_error_detected(channel)) {
		__dmac_channel_clear_address_error(channel);
	}

	if (substream)
		snd_pcm_period_elapsed(substream);

	spin_lock(&prtd->lock);
	prtd->dma_loaded--;
	if (prtd->state & ST_RUNNING) {
		jz4740_pcm_enqueue(substream);
	}
	spin_unlock(&prtd->lock);

	local_irq_save(flags);
	if (prtd->state & ST_RUNNING) {
		if (prtd->dma_loaded) {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				audio_start_dma(prtd, DMA_MODE_WRITE);
			else
				audio_start_dma(prtd, DMA_MODE_READ);
		}
	}
	local_irq_restore(flags);
	return IRQ_HANDLED;
}

/* some parameter about DMA operation */
static int jz4740_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct jz4740_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct jz4740_pcm_dma_params *dma = &jz4740_i2s_pcm_stereo_out;
	size_t totbytes = params_buffer_bytes(params);
	int ret;

	if (!dma)
	 	return 0;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		tran_bit = 8;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		tran_bit = 16;
		break;
	}

	/* prepare DMA */
	prtd->params = dma;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ret = jz_request_dma(DMA_ID_AIC_TX, prtd->params->client->name, 
				     jz4740_pcm_dma_irq, IRQF_DISABLED, substream);
		if (ret < 0)
			return ret;
		prtd->params->channel = ret;
	} else {
		ret = jz_request_dma(DMA_ID_AIC_RX, prtd->params->client->name, 
				     jz4740_pcm_dma_irq, IRQF_DISABLED, substream);
		if (ret < 0)
			return ret;
		prtd->params->channel = ret;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totbytes;

	spin_lock_irq(&prtd->lock);
	prtd->dma_loaded = 0;
	prtd->aic_dma_flag = 0;
	prtd->dma_limit = runtime->hw.periods_min;
	prtd->dma_period = params_period_bytes(params); 
	prtd->dma_start = runtime->dma_addr;
	prtd->dma_pos = prtd->dma_start;
	prtd->dma_end = prtd->dma_start + totbytes;
	prtd->curr = NULL;
	prtd->next = NULL;
	prtd->end = NULL;
	sum_bytes = 0;
	first_transfer = 1;
	printk_flag = 0;

	__dmac_disable_descriptor(prtd->params->channel);
	__dmac_channel_disable_irq(prtd->params->channel);
	spin_unlock_irq(&prtd->lock);
	return ret;
}

static int jz4740_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct jz4740_runtime_data *prtd = substream->runtime->private_data;
	
	snd_pcm_set_runtime_buffer(substream, NULL);
	if (prtd->params) {
		jz_free_dma(prtd->params->channel);
		prtd->params = NULL;
	}

	return 0;
}

/* set some dma para for playback/capture */
static int jz4740_dma_ctrl(int channel)
{

	disable_dma(channel);

	/* must clear TT bit in DCCSR to avoid interrupt again */
	if (__dmac_channel_transmit_end_detected(channel)) {
		__dmac_channel_clear_transmit_end(channel);
	}
	if (__dmac_channel_transmit_halt_detected(channel)) {
		__dmac_channel_clear_transmit_halt(channel);
	}

	if (__dmac_channel_address_error_detected(channel)) {
		__dmac_channel_clear_address_error(channel);
	}

	return 0;
	
}

static int jz4740_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct jz4740_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;
	
	/* return if this is a bufferless transfer e.g */
	if (!prtd->params)
	 	return 0;

	/* flush the DMA channel and DMA channel bit check */
	jz4740_dma_ctrl(prtd->params->channel);
	prtd->dma_loaded = 0;
	prtd->dma_pos = prtd->dma_start;
       
	/* enqueue dma buffers */
	jz4740_pcm_enqueue(substream);

	return ret;

}

static int jz4740_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct jz4740_runtime_data *prtd = runtime->private_data;

	int ret = 0;
       
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		prtd->state |= ST_RUNNING;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			audio_start_dma(prtd, DMA_MODE_WRITE);
		} else {
			audio_start_dma(prtd, DMA_MODE_READ);
		}
		
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		prtd->state &= ~ST_RUNNING;
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
		printk(" RESUME \n");
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		printk(" RESTART \n");
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t
jz4740_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct jz4740_runtime_data *prtd = runtime->private_data;
	struct jz4740_dma_buf_aic *aic_buf = prtd->curr;
	long count,res;

	dma_addr_t ptr;
	snd_pcm_uframes_t x;
	int channel = prtd->params->channel;
	
	spin_lock(&prtd->lock);
#if 1

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		count = get_dma_residue(channel);
		count = aic_buf->size - count;
		ptr = aic_buf->data + count;
		res = ptr - prtd->dma_start;
	} else {
		count = get_dma_residue(channel);
		count = aic_buf->size - count;
		ptr = aic_buf->data + count;
		res = ptr - prtd->dma_start;       
	}

# else

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if ((prtd->aic_dma_flag & AIC_START_DMA) == 0) {
			count = get_dma_residue(channel);
			count = aic_buf->size - count;
			ptr = aic_buf->data + count;
			REG_DMAC_DSAR(channel) = ptr;
			res = ptr - prtd->dma_start;
		} else {
			ptr = REG_DMAC_DSAR(channel);
			if (ptr == 0x0)
				printk("\ndma address is 00000000 in running!\n");
			res = ptr - prtd->dma_start;
		}
	} else {
		if ((prtd->aic_dma_flag & AIC_START_DMA) == 0) {
			count = get_dma_residue(channel);
			count = aic_buf->size - count;
			ptr = aic_buf->data + count;
			REG_DMAC_DTAR(channel) = ptr;
			res = ptr - prtd->dma_start;
		} else {
			ptr = REG_DMAC_DTAR(channel);
			if (ptr == 0x0)
				printk("\ndma address is 00000000 in running!\n");
			res = ptr - prtd->dma_start;
		}       
	}
#endif
	spin_unlock(&prtd->lock);
	x = bytes_to_frames(runtime, res);
	if (x == runtime->buffer_size)
		x = 0;

	return x;
}

static int jz4740_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct jz4740_runtime_data *prtd;
	
#ifdef CONFIG_SND_OSSEMUL
	hw_params_cnt = 0;
#endif
	snd_soc_set_runtime_hwparams(substream, &jz4740_pcm_hardware);
	prtd = kzalloc(sizeof(struct jz4740_runtime_data), GFP_KERNEL);
	if (prtd == NULL)
		return -ENOMEM;

	spin_lock_init(&prtd->lock);

	runtime->private_data = prtd;
	REG_AIC_I2SCR = 0x10;
	return 0;
}

static int jz4740_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct jz4740_runtime_data *prtd = runtime->private_data;
	struct jz4740_dma_buf_aic *aic_buf = NULL;
       
#ifdef CONFIG_SND_OSSEMUL
	hw_params_cnt = 0;
#endif

	if (prtd) 
		aic_buf = prtd->curr;

	while (aic_buf != NULL) {
		prtd->curr = aic_buf->next;
		prtd->next = aic_buf->next;
		aic_buf->next  = NULL;
		kfree(aic_buf);
		aic_buf = NULL;
		aic_buf = prtd->curr;
	}
	
	if (prtd) {
		prtd->curr = NULL;
		prtd->next = NULL;
		prtd->end = NULL;
		kfree(prtd);
	}

	return 0;
}

static int jz4740_pcm_mmap(struct snd_pcm_substream *substream,
			   struct vm_area_struct *vma)//include/linux/mm.h
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long start;
	unsigned long off;
	u32 len;
    printk("%s:%s[%d]\n", __FILE__, __func__, __LINE__);
    return 0;
}

struct snd_pcm_ops jz4740_pcm_ops = {
	.open		= jz4740_pcm_open,
	.close		= jz4740_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= jz4740_pcm_hw_params,
	.hw_free	= jz4740_pcm_hw_free,
	.prepare	= jz4740_pcm_prepare,
	.trigger	= jz4740_pcm_trigger,
	.pointer	= jz4740_pcm_pointer,
	.mmap		= jz4740_pcm_mmap,
};

static int jz4740_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = jz4740_pcm_hardware.buffer_bytes_max;
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;

	/*buf->area = dma_alloc_coherent(pcm->card->dev, size,
	  &buf->addr, GFP_KERNEL);*/
	buf->area = dma_alloc_noncoherent(pcm->card->dev, size,
					  &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	return 0;
}

static void jz4740_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_noncoherent(pcm->card->dev, buf->bytes,
		  buf->area, buf->addr);
		buf->area = NULL;
	}
}

static u64 jz4740_pcm_dmamask = DMA_BIT_MASK(32);

int jz4740_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
	struct snd_pcm *pcm)
{
	int ret = 0;

    printk("pcm new\n");

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &jz4740_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (dai->playback.channels_min) {
		ret = jz4740_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->capture.channels_min) {
		ret = jz4740_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
 out:

	return ret;
}

struct snd_soc_platform jz4740_soc_platform = {
	.name		= "jz4740-audio",
	.pcm_ops 	= &jz4740_pcm_ops,
	.pcm_new	= jz4740_pcm_new,
	.pcm_free	= jz4740_pcm_free_dma_buffers,
};

EXPORT_SYMBOL_GPL(jz4740_soc_platform);

static int __init jz4740_soc_platform_init(void)
{
    return snd_soc_register_platform(&jz4740_soc_platform);
}
module_init(jz4740_soc_platform_init);

static void __exit jz4740_soc_platform_exit(void)
{
    snd_soc_unregister_platform(&jz4740_soc_platform);
}
module_exit(jz4740_soc_platform_exit);

MODULE_AUTHOR("Richard");
MODULE_DESCRIPTION("Ingenic Jz4740 PCM DMA module");
MODULE_LICENSE("GPL");
