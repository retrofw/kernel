/*
 *  linux/drivers/sound/Jz_i2s.c
 *
 *  JzSOC On-Chip I2S audio driver.
 *
 *  Copyright (C) 2005 by Junzheng Corp.
 *  Modified by cjfeng on Aug 9,2007,and not any bug on Jz4730 using
 *  dma channel 4&3,noah is tested.
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
#include <linux/pm_legacy.h>
#include <sound/driver.h>
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

#define DPRINTK(args...) printk(args)
#define DMA_ID_I2S_TX	DMA_ID_AIC_TX
#define DMA_ID_I2S_RX	DMA_ID_AIC_RX
#define NR_I2S		2
#define MAXDELAY 50000
#define JZCODEC_RW_BUFFER_SIZE       2
#define JZCODEC_RW_BUFFER_TOTAL      6

typedef struct hpvol_shift_s
{
	int hpvol;
	int shift;
} hpvol_shift_t;

mixer_info info;
_old_mixer_info old_info;
int codec_volue_shift;
hpvol_shift_t hpvol_shift_table[72];
int abnormal_data_count;
unsigned long i2s_clk;

void (*set_codec_mode)(void) = NULL;
void (*clear_codec_mode)(void) = NULL;
void (*set_codec_gpio_pin)(void) = NULL;
void (*each_time_init_codec)(void) = NULL;
int (*set_codec_startup_param)(void) = NULL;
void (*set_codec_volume_table)(void) = NULL;
void (*set_codec_record)(void) = NULL;
void (*set_codec_replay)(void) = NULL;
void (*set_codec_replay_record)(void);
void (*turn_on_codec)(void) = NULL;
void (*turn_off_codec)(void) = NULL;
void (*set_codec_speed)(int rate) = NULL;
void (*reset_codec)(void) = NULL;
void (*codec_mixer_old_info_id_name)(void) = NULL;
void (*codec_mixer_info_id_name)(void) = NULL;
void (*set_codec_bass)(int val) = NULL;
void (*set_codec_volume)(int val) = NULL;
void (*set_codec_mic)(int val) = NULL;
void (*i2s_resume_codec)(void) = NULL;
void (*i2s_suspend_codec)(int wr,int rd) = NULL;
void (*init_codec_pin)(void) = NULL;
void (*set_codec_some_func)(void) = NULL;
void (*clear_codec_record)(void) = NULL;
void (*clear_codec_replay)(void) = NULL;
void (*set_replay_hp_or_speaker)(void) = NULL;
void (*set_codec_direct_mode)(void) = NULL;
void (*clear_codec_direct_mode)(void) = NULL;

static int		jz_audio_rate;
static int		jz_audio_format;
static int		jz_audio_volume;
static int		jz_audio_channels;
static int		jz_audio_b;               /* bits expand multiple */
static int              jz_audio_fragments;       /* unused fragment amount */
static int              jz_audio_fragstotal;
static int              jz_audio_fragsize;
static int              jz_audio_speed;

static int              codec_bass_gain;
static int              audio_mix_modcnt;
static int              jz_audio_dma_tran_count;  /* bytes count of one DMA transfer */
static int              jz_mic_only = 1;
static int              jz_codec_config = 0;
static unsigned long    ramp_up_start;
static unsigned long    ramp_up_end;
static unsigned long    gain_up_start;
static unsigned long    gain_up_end;
static unsigned long    ramp_down_start;
static unsigned long    ramp_down_end;
static unsigned long    gain_down_start;
static unsigned long    gain_down_end;

static int              codec_mic_gain;
static int              pop_dma_flag;
static int              last_dma_buffer_id;
static int              drain_flag;

static void (*old_mksound)(unsigned int hz, unsigned int ticks);
extern void (*kd_mksound)(unsigned int hz, unsigned int ticks);
static void jz_update_filler(int bits, int channels);

static int Init_In_Out_queue(int fragstotal,int fragsize);
static int Free_In_Out_queue(int fragstotal,int fragsize);
static irqreturn_t jz_i2s_replay_dma_irq(int irqnr, void *ref);
static irqreturn_t jz_i2s_record_dma_irq(int irqnr, void *ref);
static void (*replay_filler)(signed long src_start, int count, int id);
static int (*record_filler)(unsigned long dst_start, int count, int id);
#if defined(CONFIG_I2S_ICODEC)
static void write_mute_to_dma_buffer(signed long l_sample, signed long r_sample);
#endif
static void jz_audio_reset(void);
static struct file_operations jz_i2s_audio_fops;

static DECLARE_WAIT_QUEUE_HEAD (rx_wait_queue);
static DECLARE_WAIT_QUEUE_HEAD (tx_wait_queue);
static DECLARE_WAIT_QUEUE_HEAD (drain_wait_queue);
static DECLARE_WAIT_QUEUE_HEAD (pop_wait_queue);

struct jz_i2s_controller_info
{
	int io_base;
	int dma1;  /* for play */
	int dma2;  /* for record */
	char *name;
	int dev_audio;
	struct i2s_codec *i2s_codec[NR_I2S];
	int opened1;
	int opened2;
	unsigned char *tmp1;  /* tmp buffer for sample conversions */
	unsigned char *tmp2;
	spinlock_t lock;
	spinlock_t ioctllock;
	
	wait_queue_head_t dac_wait;
	wait_queue_head_t adc_wait;
	int nextIn;  /* byte index to next-in to DMA buffer */
	int nextOut; /* byte index to next-out from DMA buffer */
	int count;	 /* current byte count in DMA buffer */
	int finish;  /* current transfered byte count in DMA buffer */ 
	unsigned        total_bytes;	/* total bytes written or read */
	unsigned        blocks;
	unsigned        error;	/* over/underrun */
#ifdef CONFIG_PM
	struct pm_dev		*pm;
#endif
};


static struct jz_i2s_controller_info *i2s_controller = NULL;
struct i2s_codec 
{
	/* I2S controller connected with */
	void *private_data;
	char *name;
	int id;
	int dev_mixer; 
	/* controller specific lower leverl i2s accessing routines */
	u16  (*codec_read)  (u8 reg); /* the function accessing Codec REGs */
	void (*codec_write) (u8 reg, u16 val);
	/* Wait for codec-ready */
	void  (*codec_wait)  (struct i2s_codec *codec);
	/* OSS mixer masks */
	int modcnt;
	int supported_mixers;
	int stereo_mixers;
	int record_sources;
	int bit_resolution;
	/* OSS mixer interface */
	int  (*read_mixer) (struct i2s_codec *codec, int oss_channel);
	void (*write_mixer)(struct i2s_codec *codec, int oss_channel,
			    unsigned int left, unsigned int right);
	int  (*recmask_io) (struct i2s_codec *codec, int rw, int mask);
	int  (*mixer_ioctl)(struct i2s_codec *codec, unsigned int cmd, unsigned long arg);
	/* saved OSS mixer states */
	unsigned int mixer_state[SOUND_MIXER_NRDEVICES];
};


typedef struct buffer_queue_s 
{
	int count;
	int *id;
	int lock;
} buffer_queue_t;

typedef struct left_right_sample_s
{
	signed long left;
	signed long right;
} left_right_sample_t;

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

static left_right_sample_t save_last_samples[64];
static int read_codec_file(int addr)
{
	while (__icdc_rgwr_ready());
	__icdc_set_addr(addr);
	mdelay(1);
	return(__icdc_get_value());
}

#if 0  /* mask warning */
static void printk_codec_files(void)
{
	int cnt;

	printk("\n");
	
	printk("REG_CPM_I2SCDR=0x%08x\n",REG_CPM_I2SCDR);
	printk("REG_CPM_CLKGR=0x%08x\n",REG_CPM_CLKGR);
	printk("REG_CPM_CPCCR=0x%08x\n",REG_CPM_CPCCR);
	printk("REG_AIC_FR=0x%08x\n",REG_AIC_FR);
	printk("REG_AIC_CR=0x%08x\n",REG_AIC_CR);		
	printk("REG_AIC_I2SCR=0x%08x\n",REG_AIC_I2SCR);
	printk("REG_AIC_SR=0x%08x\n",REG_AIC_SR);
	printk("REG_ICDC_RGDATA=0x%08x\n",REG_ICDC_RGDATA);

	for (cnt = 0; cnt <= 27 ; cnt++) {
		printk(" ( %d  :  0x%x ) ",cnt ,read_codec_file(cnt));
	}
	printk("\n");
}
#endif

static void write_codec_file(int addr, int val)
{
	while (__icdc_rgwr_ready());
	__icdc_set_addr(addr);
	__icdc_set_cmd(val); /* write */
	mdelay(1);
	__icdc_set_rgwr();
	mdelay(1);
}

static int write_codec_file_bit(int addr, int bitval, int mask_bit)
{
	int val;
	while (__icdc_rgwr_ready());
	__icdc_set_addr(addr);
	mdelay(1);
	val = __icdc_get_value(); /* read */

	while (__icdc_rgwr_ready());
	__icdc_set_addr(addr);
	val &= ~(1 << mask_bit);
	if (bitval == 1)
		val |= 1 << mask_bit;

	__icdc_set_cmd(val); /* write */
	mdelay(1);
	__icdc_set_rgwr();
	mdelay(1);

	while (__icdc_rgwr_ready());
	__icdc_set_addr(addr);
	val = __icdc_get_value(); /* read */	
	
	if (((val >> mask_bit) & bitval) == bitval)
		return 1;
	else 
		return 0;
}

#if 0 /* mask warning */
/* set Audio data replay */
static void set_audio_data_replay(void)
{
	/* DAC path */
	write_codec_file(9, 0xff);
	//write_codec_file(8, 0x30);
	write_codec_file(8, 0x20);
	mdelay(10);
	write_codec_file_bit(1, 0, 4);//CR1.HP_DIS->0
	write_codec_file_bit(5, 1, 3);//PMR1.SB_LIN->1
	write_codec_file_bit(5, 1, 0);//PMR1.SB_IND->1
	
	write_codec_file_bit(1, 0, 2);//CR1.BYPASS->0
	write_codec_file_bit(1, 1, 3);//CR1.DACSEL->1
	
	write_codec_file_bit(5, 0, 5);//PMR1.SB_MIX->0
	//mdelay(100);
	//write_codec_file_bit(5, 0, 6);//PMR1.SB_OUT->0
	write_codec_file_bit(1, 1, 7);//CR1.SB_MICBIAS->1
	//mdelay(300);
}
#endif

/* unset Audio data replay */
static void unset_audio_data_replay(void)
{
	//write_codec_file_bit(1, 1, 5);//DAC_MUTE->1
	//mdelay(800);
	//write_codec_file_bit(5, 1, 6);//SB_OUT->1
	//mdelay(800);
	write_codec_file_bit(5, 1, 7);//SB_DAC->1
	write_codec_file_bit(5, 1, 4);//SB_MIX->1
	write_codec_file_bit(6, 1, 0);//SB_SLEEP->1
	write_codec_file_bit(6, 1, 1);//SB->1
}

#if 0 /* mask warning */
/* set Record MIC input audio without playback */
static void set_record_mic_input_audio_without_playback(void)
{
	/* ADC path for MIC IN */
	jz_mic_only = 1;
	write_codec_file(9, 0xff);
	write_codec_file(8, 0x3f);
	write_codec_file_bit(23, 0, 7);//AGC1.AGC_EN->0
	mdelay(10);
	write_codec_file_bit(1, 1, 2);
	//write_codec_file_bit(1, 1, 6);//CR1.MONO->1
	
	write_codec_file(22, 0x40);//mic 1
	write_codec_file_bit(3, 1, 7);//CR1.HP_DIS->1
	write_codec_file_bit(5, 1, 3);//PMR1.SB_LIN->1
	write_codec_file_bit(5, 1, 0);//PMR1.SB_IND->1
	
	write_codec_file_bit(1, 0, 2);//CR1.BYPASS->0
	write_codec_file_bit(1, 0, 3);//CR1.DACSEL->0
	//write_codec_file_bit(6, 1, 3);// gain set
	
	write_codec_file_bit(5, 0, 5);//PMR1.SB_MIX->0
	mdelay(100);
	write_codec_file_bit(5, 0, 6);//PMR1.SB_OUT->0
	write_codec_file(1, 0x4);
}
#endif

#if 0 /* mask warning */
/* unset Record MIC input audio without playback */
static void unset_record_mic_input_audio_without_playback(void)
{
	/* ADC path for MIC IN */
	jz_mic_only = 0;
	write_codec_file_bit(5, 1, 4);//SB_ADC->1
	write_codec_file_bit(1, 1, 7);//CR1.SB_MICBIAS->1
	write_codec_file(22, 0xc0);//CR3.SB_MIC1
	write_codec_file_bit(5, 1, 6);//PMR1.SB_OUT->1
	write_codec_file_bit(1, 1, 5);//DAC_MUTE->1
	write_codec_file_bit(6, 1, 0);//SB_SLEEP->1
	write_codec_file_bit(6, 1, 1);//SB->1
}
#endif

#if 0 /* mask warning */
/* set Record LINE input audio without playback */
static void set_record_line_input_audio_without_playback(void)
{
	/* ADC path for LINE IN */	
	jz_mic_only = 1;
	write_codec_file(9, 0xff);
	write_codec_file(8, 0x3f);
	mdelay(10);
	write_codec_file(22, 0xf6);//line in 1
	write_codec_file_bit(23, 0, 7);//AGC1.AGC_EN->0
	write_codec_file_bit(3, 1, 7);//CR1.HP_DIS->1
	write_codec_file_bit(5, 0, 3);//PMR1.SB_LIN->0
	write_codec_file_bit(5, 1, 0);//PMR1.SB_IND->1
	
	write_codec_file_bit(1, 0, 2);//CR1.BYPASS->0
	write_codec_file_bit(1, 0, 3);//CR1.DACSEL->0
	mdelay(10);
	write_codec_file_bit(5, 0, 5);//PMR1.SB_MIX->0
	mdelay(100);
	write_codec_file_bit(5, 0, 6);//PMR1.SB_OUT->0
	write_codec_file(1, 0x4);
}
#endif

#if 0 /* mask warning */ 
/* unset Record LINE input audio without playback */
static void unset_record_line_input_audio_without_playback(void)
{
	/* ADC path for LINE IN */
	write_codec_file_bit(5, 1, 4);//SB_ADC->1
	write_codec_file_bit(5, 1, 3);//ONR1.SB_LIN->1
	
	write_codec_file(22, 0xc0);//CR3.SB_MIC1
	write_codec_file_bit(5, 1, 6);//PMR1.SB_OUT->1
	write_codec_file_bit(1, 1, 5);//DAC_MUTE->1
	write_codec_file_bit(6, 1, 0);//SB_SLEEP->1
	write_codec_file_bit(6, 1, 1);//SB->1
}
#endif

#if 0 /* mask warning */
/* set Playback LINE input audio direct only */
static void set_playback_line_input_audio_direct_only(void)
{
	jz_audio_reset();//or init_codec()
	REG_AIC_I2SCR = 0x10;
	write_codec_file(9, 0xff);
	write_codec_file(8, 0x3f);
	mdelay(10);
	write_codec_file(22, 0xf6);//line in 1
	write_codec_file_bit(23, 0, 7);//AGC1.AGC_EN->0
	mdelay(10);
	write_codec_file_bit(1, 1, 2);//CR1.HP_BYPASS->1
	write_codec_file_bit(1, 0, 4);//CR1.HP_DIS->0
	write_codec_file_bit(1, 0, 3);//CR1.DACSEL->0
	write_codec_file_bit(5, 1, 0);//PMR1.SB_IND->1
	write_codec_file_bit(5, 0, 3);//PMR1.SB_LIN->0

	write_codec_file_bit(5, 0, 5);//PMR1.SB_MIX->0
	mdelay(100);
	write_codec_file_bit(5, 0, 6);//PMR1.SB_OUT->0
	//write_codec_file_bit(5, 1, 7);//PMR1.SB_DAC->1
	//write_codec_file_bit(5, 1, 4);//PMR1.SB_ADC->1
}
#endif

#if 0 /* mask warning */
/* unset Playback LINE input audio direct only */
static void unset_playback_line_input_audio_direct_only(void)
{
	write_codec_file_bit(6, 0, 3);//GIM->0
	write_codec_file_bit(1, 0, 2);//PMR1.BYPASS->0
	write_codec_file_bit(5, 1, 3);//PMR1.SB_LINE->1
	write_codec_file_bit(5, 1, 6);//PMR1.SB_OUT->1
	mdelay(100);
	write_codec_file_bit(5, 1, 5);//PMR1.SB_MIX->1
	write_codec_file_bit(6, 1, 0);//SB_SLEEP->1
	write_codec_file_bit(6, 1, 1);//SB->1
}
#endif

#if 0 /* mask warning */
/* set Record MIC input audio with direct playback */
static void set_record_mic_input_audio_with_direct_playback(void)
{		
	write_codec_file_bit(23, 0, 7);//AGC1.AGC_EN->0
	jz_mic_only = 0;
	write_codec_file(9, 0xff);
	write_codec_file(8, 0x3f);
	mdelay(10);
	
	write_codec_file(22, 0x60);//mic 1
	write_codec_file_bit(23, 0, 7);//AGC1.AGC_EN->0
	write_codec_file_bit(5, 1, 3);//PMR1.SB_LIN->1
	write_codec_file_bit(5, 1, 0);//PMR1.SB_IND->1
	write_codec_file_bit(1, 0, 7);//CR1.SB_MICBIAS->0
	write_codec_file_bit(1, 0, 4);//CR1.HP_DIS->0
	
	write_codec_file_bit(1, 0, 2);//CR1.BYPASS->0
	write_codec_file_bit(1, 0, 3);//CR1.DACSEL->0
	write_codec_file_bit(6, 1, 3);// gain set
	
	write_codec_file_bit(5, 0, 5);//PMR1.SB_MIX->0
	mdelay(100);
	write_codec_file_bit(5, 0, 6);//PMR1.SB_OUT->0
	//write_codec_file(1, 0x4);
}
#endif

#if 0 /* mask warning */
/* unset Record MIC input audio with direct playback */
static void unset_record_mic_input_audio_with_direct_playback(void)
{
	/* ADC path for MIC IN */
	jz_mic_only = 0;
	write_codec_file_bit(5, 1, 4);//SB_ADC->1
	write_codec_file_bit(1, 1, 7);//CR1.SB_MICBIAS->1
	write_codec_file(22, 0xc0);//CR3.SB_MIC1
	write_codec_file_bit(5, 1, 6);//PMR1.SB_OUT->1
	write_codec_file_bit(1, 1, 5);//DAC_MUTE->1
	write_codec_file_bit(6, 1, 0);//SB_SLEEP->1
	write_codec_file_bit(6, 1, 1);//SB->1
}
#endif

#if 0 /* mask warning */
/* set Record playing audio mixed with MIC input audio */
static void set_record_playing_audio_mixed_with_mic_input_audio(void)
{
	write_codec_file_bit(23, 0, 7);//AGC1.AGC_EN->0
	write_codec_file(9, 0xff);
	//write_codec_file(8, 0x30);
	write_codec_file(8, 0x20);
	mdelay(10);
	
	write_codec_file(22, 0x63);//mic 1
	
	write_codec_file_bit(1, 0, 2);//CR1.BYPASS->0
	write_codec_file_bit(6, 1, 3);// gain set

	write_codec_file_bit(1, 0, 4);//CR1.HP_DIS->0	
	write_codec_file_bit(5, 1, 3);//PMR1.SB_LIN->1
	write_codec_file_bit(5, 1, 0);//PMR1.SB_IND->1
	write_codec_file_bit(1, 0, 7);//CR1.SB_MICBIAS->0
	write_codec_file_bit(22, 0, 7);//CR3.SB_MIC->0
	write_codec_file_bit(1, 1, 3);//CR1.DACSEL->1
	write_codec_file_bit(5, 0, 5);//PMR1.SB_MIX->0
	write_codec_file_bit(5, 0, 4);//PMR1.SB_MIX->0
}
#endif

#if 0 /* mask warning */
/* unset Record playing audio mixed with MIC input audio */
static void unset_record_playing_audio_mixed_with_mic_input_audio(void)
{
	/* ADC path */
	write_codec_file_bit(5, 1, 4);//SB_ADC->1
	write_codec_file_bit(1, 1, 7);//CR1.SB_MICBIAS->1
	//write_codec_file_bit(1, 1, 6);//CR1.MONO->1
	write_codec_file(22, 0xc0);//CR3.SB_MIC1->1
	//write_codec_file_bit(1, 1, 5);//DAC_MUTE->1
	//write_codec_file_bit(5, 1, 6);//SB_OUT->1
	write_codec_file_bit(5, 1, 7);//SB_DAC->1
	write_codec_file_bit(5, 1, 5);//SB_MIX->1
	write_codec_file_bit(6, 1, 0);//SB_SLEEP->1
	write_codec_file_bit(6, 1, 1);//SB->1
}
#endif

#if 0 /* mask warning */
/* set Record MIC input audio with Audio data replay (full duplex) */
static void set_record_mic_input_audio_with_audio_data_replay(void)
{		
	write_codec_file_bit(23, 0, 7);//AGC1.AGC_EN->0
	write_codec_file(9, 0xff);
	//write_codec_file(8, 0x30);
	write_codec_file(8, 0x20);
	write_codec_file_bit(1, 0, 4);//CR1.HP_DIS->0	
	write_codec_file_bit(5, 1, 3);//PMR1.SB_LIN->1
	write_codec_file_bit(5, 1, 0);//PMR1.SB_IND->1

	write_codec_file_bit(22, 0, 7);//CR3.SB_MIC->0
	write_codec_file_bit(1, 0, 7);//CR1.SB_MICBIAS->0

	write_codec_file_bit(1, 1, 3);//CR1.DACSEL->1
	write_codec_file_bit(5, 0, 5);//PMR1.SB_MIX->0
}
#endif

#if 0 /* mask warning */
/* unset Record MIC input audio with Audio data replay (full duplex) */
static void unset_record_mic_input_audio_with_audio_data_replay(void)
{
	/* ADC path */
	write_codec_file_bit(5, 1, 4);//SB_ADC->1
	write_codec_file_bit(1, 1, 7);//CR1.SB_MICBIAS->1
	//write_codec_file_bit(1, 1, 6);//CR1.MONO->1
	write_codec_file(22, 0xc0);//CR3.SB_MIC1->1
	write_codec_file_bit(5, 1, 7);//SB_DAC->1
	write_codec_file_bit(5, 1, 5);//SB_MIX->1
	write_codec_file_bit(6, 1, 0);//SB_SLEEP->1
	write_codec_file_bit(6, 1, 1);//SB->1
}
#endif

#if 0 /* mask warning */
/* set Record LINE input audio with Audio data replay (full duplex for linein) */
static void set_record_line_input_audio_with_audio_data_replay(void)
{		
	write_codec_file(9, 0xff);
	//write_codec_file(8, 0x30);
	write_codec_file(8, 0x20);
	write_codec_file_bit(1, 0, 4);//CR1.HP_DIS->0	
	write_codec_file_bit(5, 0, 3);//PMR1.SB_LIN->0
	write_codec_file_bit(5, 1, 0);//PMR1.SB_IND->1
	write_codec_file_bit(1, 1, 7);//CR1.SB_MICBIAS->1
	//write_codec_file_bit(22, 1, 7);//CR3.SB_MIC->1
	write_codec_file_bit(1, 1, 3);//CR1.DACSEL->1
	write_codec_file_bit(5, 0, 5);//PMR1.SB_MIX->0
 

	//jz_mic_only = 1;
	write_codec_file(22, 0xc6);//line in 1
	write_codec_file_bit(23, 0, 7);//AGC1.AGC_EN->0
	write_codec_file_bit(1, 0, 2);//CR1.BYPASS->0
	write_codec_file_bit(5, 0, 5);//PMR1.SB_MIX->0
}
#endif

#if 0 /* mask warning */
/* unset Record LINE input audio with Audio data replay (full duplex for linein) */
static void unset_record_line_input_audio_with_audio_data_replay(void)
{
	/* ADC path */
	write_codec_file_bit(5, 1, 4);//SB_ADC->1
	write_codec_file_bit(1, 1, 7);//CR1.SB_MICBIAS->1
	//write_codec_file_bit(1, 1, 6);//CR1.MONO->1
	write_codec_file(22, 0xc0);//CR3.SB_MIC1->1
	write_codec_file_bit(5, 1, 7);//SB_DAC->1
	write_codec_file_bit(5, 1, 5);//SB_MIX->1
	write_codec_file_bit(6, 1, 0);//SB_SLEEP->1
	write_codec_file_bit(6, 1, 1);//SB->1
}
#endif

static inline int get_buffer_id(struct buffer_queue_s *q)
{
	int r;
	unsigned long flags;
	int i;

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
	struct jz_i2s_controller_info * controller = (struct jz_i2s_controller_info *) dev_id;
	
	spin_lock_irqsave(&controller->ioctllock, flags);
	jz_audio_dma_tran_count = count / jz_audio_b;
	spin_unlock_irqrestore(&controller->ioctllock, flags);
	flags = claim_dma_lock();
	disable_dma(chan);
	clear_dma_ff(chan);
	//set_dma_mode(chan, mode);
	jz_set_oss_dma(chan, mode, jz_audio_format);
	set_dma_addr(chan, phyaddr);
	if (count == 0) {
		count++;
		printk("JzSOC DMA controller can't set dma 0 count!\n");
	}
	set_dma_count(chan, count);
	enable_dma(chan);
	release_dma_lock(flags);
}

static irqreturn_t jz_i2s_record_dma_irq (int irq, void *dev_id)
{
	int id1, id2;
	unsigned long flags;
	struct jz_i2s_controller_info * controller = (struct jz_i2s_controller_info *) dev_id;
	int dma = controller->dma2;
	
	disable_dma(dma);
	if (__dmac_channel_address_error_detected(dma)) {
		printk(KERN_DEBUG "%s: DMAC address error.\n", __FUNCTION__);
		__dmac_channel_clear_address_error(dma);
	}
	if (__dmac_channel_transmit_end_detected(dma)) {
		__dmac_channel_clear_transmit_end(dma);
		
		if(drain_flag == 1)
			wake_up(&drain_wait_queue);
		/* for DSP_GETIPTR */
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

static irqreturn_t jz_i2s_replay_dma_irq (int irq, void *dev_id)
{
	int id;
	unsigned long flags;
	struct jz_i2s_controller_info * controller = (struct jz_i2s_controller_info *) dev_id;
	int dma = controller->dma1;
   
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
		} else {
			if(drain_flag == 1) {
				/* Is replay dma buffer over ? */
				if(elements_in_queue(&out_full_queue) <= 0) {
					drain_flag = 0;
					wake_up(&drain_wait_queue);
				}
			}

			/* for DSP_GETOPTR */
			spin_lock_irqsave(&controller->ioctllock, flags);
			controller->total_bytes += jz_audio_dma_tran_count;
			controller->blocks ++;
			spin_unlock_irqrestore(&controller->ioctllock, flags);
			if ((id = get_buffer_id(&out_busy_queue)) < 0)
				printk(KERN_DEBUG "Strange DMA finish interrupt for I2S module\n");
			put_buffer_id(&out_empty_queue, id);
			if ((id = get_buffer_id(&out_full_queue)) >= 0) {
				put_buffer_id(&out_busy_queue, id);
				if(*(out_dma_buf_data_count + id) > 0) {
					audio_start_dma(dma, dev_id, *(out_dma_pbuf + id),
							*(out_dma_buf_data_count + id),
							DMA_MODE_WRITE);
					last_dma_buffer_id = id;
				}
			} else
				out_busy_queue.count = 0;

			if (elements_in_queue(&out_empty_queue) > 0) {
				wake_up(&tx_wait_queue);
				wake_up(&controller->dac_wait);
			}
		}
	}

	return IRQ_HANDLED;
}

static void jz_i2s_initHw(int set)
{
#if defined(CONFIG_MIPS_JZ_URANUS)
	i2s_clk = 48000000;
#else
	i2s_clk = __cpm_get_i2sclk();
#endif
	__i2s_disable();
	if(set)
		__i2s_reset();
	schedule_timeout(5);
	if(each_time_init_codec)
		each_time_init_codec();
	__i2s_disable_record();
	__i2s_disable_replay();
	__i2s_disable_loopback();
	__i2s_set_transmit_trigger(4);
	__i2s_set_receive_trigger(3);
}

static int Init_In_Out_queue(int fragstotal,int fragsize)
{
	int i;

	/* recording */
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

	/* playing */
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
	/* alloc DMA buffer */
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
	/* playing */
	if(out_dma_buf != NULL) {
		for (i = 0; i < fragstotal; i++) {
			if(*(out_dma_buf + i))
				free_pages(*(out_dma_buf + i), get_order(fragsize));
			*(out_dma_buf + i) = 0;
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

	/* recording */
	if(in_dma_buf) {
		for (i = 0; i < fragstotal; i++) {
			if(*(in_dma_buf + i)) {
				dma_cache_wback_inv(*(in_dma_buf + i), fragsize);
				free_pages(*(in_dma_buf + i), get_order(fragsize));
			}
			*(in_dma_buf + i) = 0; 
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

static void jz_i2s_full_reset(struct jz_i2s_controller_info *controller)
{
	jz_i2s_initHw(0);
}

static int jz_audio_set_speed(int dev, int rate)
{
    /* 8000, 11025, 16000, 22050, 24000, 32000, 44100, 48000, 99999999 ? */
	jz_audio_speed = rate;
	if (rate > 48000)
		rate = 48000;
	if (rate < 8000)
		rate = 8000;
	jz_audio_rate = rate;
	
	if(set_codec_speed)
		set_codec_speed(rate);

	return jz_audio_rate;
}


static int record_fill_1x8_u(unsigned long dst_start, int count, int id)
{
	int cnt = 0;
	unsigned long data;
	volatile unsigned long *s = (unsigned long*)(*(in_dma_buf + id));
	volatile unsigned char *dp = (unsigned char*)dst_start;

	while (count > 0) {
		count -= 2;   /* count in dword */
		cnt++;
		data = *(s++);
		*(dp ++) = ((data << 16) >> 24) + 0x80;
		s++;	      /* skip the other channel */
	}

	return cnt;
}


static int record_fill_2x8_u(unsigned long dst_start, int count, int id)
{
	int cnt = 0;
	unsigned long d1, d2;
	volatile unsigned long *s = (unsigned long*)(*(in_dma_buf + id));
	volatile unsigned char *dp = (unsigned char*)dst_start;
    
	while (count > 0) {
		count -= 2;
		cnt += 2;
		d1 = *(s++);
		*(dp ++) = ((d1 << 16) >> 24) + 0x80;
		d2 = *(s++);
		*(dp ++) = ((d2 << 16) >> 24) + 0x80;
	}
	
	return cnt;
}


static int record_fill_1x16_s(unsigned long dst_start, int count, int id)
{
    int cnt = 0;
    unsigned long d1;
    unsigned long *s = (unsigned long*)(*(in_dma_buf + id));
    unsigned short *dp = (unsigned short *)dst_start;
  
    while (count > 0) {
	    count -= 2;	   /* count in dword */
	    cnt += 2;	   /* count in byte */
	    d1 = *(s++);
	    *(dp ++) = (d1 << 16) >> 16;
	    s++;	   /* skip the other channel */
    }

    return cnt;
}


static int record_fill_2x16_s(unsigned long dst_start, int count, int id)
{
    int cnt = 0;
    unsigned long d1, d2;
    unsigned long *s = (unsigned long*)(*(in_dma_buf + id));
    unsigned short *dp = (unsigned short *)dst_start;
    while (count > 0) {
        count -= 2;	/* count in dword */
        cnt += 4;	/* count in byte */
        d1 = *(s++);
        d2 = *(s++);
	if(abnormal_data_count > 0) { 
		d1 = d2 = 0;
		abnormal_data_count --;
	}
	*(dp ++) = (d1 << 16) >> 16;
        *(dp ++) = (d2 << 16) >> 16;
    }

    return cnt;
}

static void replay_fill_1x8_u(signed long src_start, int count, int id)
{
	int cnt = 0;
	unsigned char data;
	unsigned long ddata;
	volatile unsigned char *s = (unsigned char *)src_start;
	volatile unsigned long *dp = (unsigned long*)(*(out_dma_buf + id));

	while (count > 0) {
		count--;
		cnt += 1;
		data = *(s++) - 0x80;
		ddata = (unsigned long) data << 8;
		*(dp ++) = ddata;
		*(dp ++) = ddata;

		/* save last left and right */
		if(count == 1) {
			save_last_samples[id].left = ddata;
			save_last_samples[id].right = ddata;
		}
	}
	cnt = cnt * 2 * jz_audio_b;
	*(out_dma_buf_data_count + id) = cnt;
}


static void replay_fill_2x8_u(signed long src_start, int count, int id)
{
	int cnt = 0;
	unsigned char d1;
	unsigned long dd1;
	volatile unsigned char *s = (unsigned char *)src_start;
	volatile unsigned long *dp = (unsigned long*)(*(out_dma_buf + id));

	while (count > 0) {
		count -= 1;
		cnt += 1 ;
		d1 = *(s++) - 0x80;
		dd1 = (unsigned long) d1 << 8;
		*(dp ++) = dd1;
		/* save last left */
		if(count == 2)
			save_last_samples[id].left = dd1;
		/* save last right */
		if(count == 1)
			save_last_samples[id].right = dd1;
	}
	cnt *= jz_audio_b;
	*(out_dma_buf_data_count + id) = cnt;
}


static void replay_fill_1x16_s(signed long src_start, int count, int id)
{
	int cnt = 0;
	signed short d1;
	signed long l1;
	volatile signed short *s = (signed short *)src_start;
	volatile signed long *dp = (signed long*)(*(out_dma_buf + id));
       
	while (count > 0) {
		count -= 2;
		cnt += 2 ;
		d1 = *(s++);
		l1 = (signed long)d1;
		*(dp ++) = l1;
		*(dp ++) = l1;

		/* save last left and right */
		if(count == 1) {
			save_last_samples[id].left = l1;
			save_last_samples[id].right = l1;
		}
	}
	cnt = cnt * 2 * jz_audio_b;
	*(out_dma_buf_data_count + id) = cnt;
}

#if 0
static void replay_fill_2x16_s(signed long src_start, int count, int id)
{
	int cnt = 0;
	signed short d1;
	signed long l1;
	int mute_cnt = 0;
	signed long tmp1,tmp2;
	volatile signed short *s = (signed short *)src_start;
	volatile signed long *dp = (signed long*)(*(out_dma_buf + id));
#if defined(CONFIG_I2S_ICDC)
	volatile signed long *before_dp;
	int sam_rate = jz_audio_rate / 20;

	tmp1 = tmp2 = 0;
	while (count > 0) {
		count -= 2;
		cnt += 2;
		d1 = *(s++);
		
		l1 = (signed long)d1;
		l1 >>= codec_volue_shift;

		if(l1 == 0) {
			mute_cnt ++;
			if(mute_cnt >= sam_rate) {
				before_dp = dp - 10;
				*(before_dp) = (signed long)1;
				before_dp = dp - 11;
				*(before_dp) = (signed long)1;
				mute_cnt = 0;
			}
		} else
			mute_cnt = 0;

		*(dp ++) = l1;
		
		tmp1 = tmp2;
		tmp2 = l1;
	}
   
	/* save last left */
	save_last_samples[id].left = tmp1;
	/* save last right */
	save_last_samples[id].right = tmp2;
#endif
#if defined(CONFIG_I2S_DLV)
	while (count > 0) {
		count -= 2;
		cnt += 2;
		d1 = *(s++);
		
		l1 = (signed long)d1;
		
		*(dp ++) = l1;
	}
#endif
	cnt *= jz_audio_b;
	*(out_dma_buf_data_count + id) = cnt;
}
#else
static void replay_fill_2x16_s(signed long src_start, int count, int id)
{
	int cnt = 0;
	signed short d1;
	signed long l1;

#if 0
	volatile signed short *s = (signed short *)src_start;
	volatile signed short *dp = (signed short*)(*(out_dma_buf + id));
	memcpy((char*)dp, (char*)s, count);
	*(out_dma_buf_data_count + id) = count;
#else	
	volatile signed short *s = (signed short *)src_start;
	volatile signed long *dp = (signed long*)(*(out_dma_buf + id));
	while (count > 0) {
		count -= 2;
		cnt += 2;
		d1 = *(s++);
		
		l1 = (signed long)d1;
		
		*(dp ++) = l1;
		}
	cnt *= jz_audio_b;
	*(out_dma_buf_data_count + id) = cnt;
#endif
}
#endif


static unsigned int jz_audio_set_format(int dev, unsigned int fmt)
{
	switch (fmt) {
	case AFMT_U8:
		__i2s_set_oss_sample_size(8);
		__i2s_set_iss_sample_size(8);
		jz_audio_format = fmt;
		jz_update_filler(jz_audio_format, jz_audio_channels);
		break;
	case AFMT_S16_LE:
#if defined(CONFIG_I2S_DLV)
		/* DAC path and ADC path */
		write_codec_file(2, 0x00);
		//write_codec_file(2, 0x60);
#endif
		jz_audio_format = fmt;
		jz_update_filler(jz_audio_format,jz_audio_channels);
		/* print all files */
		__i2s_set_oss_sample_size(16);
		__i2s_set_iss_sample_size(16);
		break;

	case AFMT_QUERY:
		break;
	}

	return jz_audio_format;
}


static short jz_audio_set_channels(int dev, short channels)
{
	switch (channels) {
	case 1:
		if(set_codec_some_func)
			set_codec_some_func();
		jz_audio_channels = channels;
		jz_update_filler(jz_audio_format, jz_audio_channels);
#if defined(CONFIG_I2S_DLV)
		write_codec_file_bit(1, 1, 6);//CR1.MONO->1 for Mono
#endif
		break;
	case 2:
		jz_audio_channels = channels;
		jz_update_filler(jz_audio_format, jz_audio_channels);
#if defined(CONFIG_I2S_DLV)
		write_codec_file_bit(1, 0, 6);//CR1.MONO->0 for Stereo
#endif
		break;
	case 0:
		break;
	}

	return jz_audio_channels;
}

static void init_codec(void)
{
	/* inititalize internal I2S codec */
	if(init_codec_pin)
		init_codec_pin();

#if defined(CONFIG_I2S_ICDC)
        /* initialize AIC but not reset it */
	jz_i2s_initHw(0);
#endif
	if(reset_codec)
		reset_codec();
}

static void jz_audio_reset(void)
{
	__i2s_disable_replay();
	__i2s_disable_receive_dma();
	__i2s_disable_record();
	__i2s_disable_transmit_dma();
#if defined(CONFIG_I2S_DLV)
	REG_AIC_I2SCR = 0x10;
#endif
	init_codec();
}

static int jz_audio_release(struct inode *inode, struct file *file);
static int jz_audio_open(struct inode *inode, struct file *file);
static int jz_audio_ioctl(struct inode *inode, struct file *file,unsigned int cmd, unsigned long arg);
static unsigned int jz_audio_poll(struct file *file,struct poll_table_struct *wait);
static ssize_t jz_audio_write(struct file *file, const char *buffer,size_t count, loff_t *ppos);
static ssize_t jz_audio_read(struct file *file, char *buffer,size_t count, loff_t *ppos);

/* static struct file_operations jz_i2s_audio_fops */
static struct file_operations jz_i2s_audio_fops =
{
	owner:              THIS_MODULE,
	open:               jz_audio_open,
	release:            jz_audio_release,
	write:              jz_audio_write,
	read:               jz_audio_read,
	poll:               jz_audio_poll,
	ioctl:              jz_audio_ioctl
};

static int jz_i2s_open_mixdev(struct inode *inode, struct file *file)
{
	int i;
	int minor = MINOR(inode->i_rdev);
	struct jz_i2s_controller_info *controller = i2s_controller;

	for (i = 0; i < NR_I2S; i++)
		if (controller->i2s_codec[i] != NULL && controller->i2s_codec[i]->dev_mixer == minor)
			goto match;
	
	if (!controller)
		return -ENODEV;
match:
	file->private_data = controller->i2s_codec[i];

	return 0;
}

static int jz_i2s_ioctl_mixdev(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2s_codec *codec = (struct i2s_codec *)file->private_data;
	return codec->mixer_ioctl(codec, cmd, arg);
}

static loff_t jz_i2s_llseek(struct file *file, loff_t offset, int origin)
{
	return -ESPIPE;
}

static struct file_operations jz_i2s_mixer_fops = 
{
	owner:		THIS_MODULE,
	llseek:		jz_i2s_llseek,
	ioctl:		jz_i2s_ioctl_mixdev,
	open:		jz_i2s_open_mixdev,
};

static int i2s_mixer_ioctl(struct i2s_codec *codec, unsigned int cmd, unsigned long arg)
{
	int ret;
	long val = 0;
	switch (cmd) {
	case SOUND_MIXER_INFO:

		if(codec_mixer_info_id_name)
			codec_mixer_info_id_name();
		info.modify_counter = audio_mix_modcnt;

		return copy_to_user((void *)arg, &info, sizeof(info));
	case SOUND_OLD_MIXER_INFO:

		if(codec_mixer_old_info_id_name)
			codec_mixer_old_info_id_name();

		return copy_to_user((void *)arg, &old_info, sizeof(info));
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
	case SOUND_MIXER_WRITE_SPEAKER:
		
		ret = get_user(val, (long *) arg);
		if (ret)
			return ret;
		val = val & 0xff;
		if(val < 0)
			val = 0;
		if(val > 100)
			val = 100;
		switch(val) {
		case 100:
			if(set_codec_direct_mode)
				set_codec_direct_mode();
			break;
		case 0:
			if(clear_codec_direct_mode)
				clear_codec_direct_mode();
			break;
		}
		break;
	case SOUND_MIXER_WRITE_BASS:

		ret = get_user(val, (long *) arg);
		if (ret)
			return ret;
		
		val = val & 0xff;
		if(val < 0)
			val = 0;
		if(val > 100)
			val = 100;
		codec_bass_gain = val;
		if(set_codec_bass)
			set_codec_bass(val);
		
		return 0;
	case SOUND_MIXER_READ_BASS:			
	
		val = codec_bass_gain;
		ret = val << 8;
		val = val | ret;			
		
		return put_user(val, (long *) arg);
	case SOUND_MIXER_WRITE_VOLUME:
		ret = get_user(val, (long *) arg);
		if (ret)
			return ret;
		val = val & 0xff;
		if(val < 0)
			val = 0;
		if(val > 100)
			val = 100;
		if (val > 31)
			val = 31;
		jz_audio_volume = val;

		if(set_codec_volume)
			set_codec_volume(val);
		return 0;
	case SOUND_MIXER_READ_VOLUME:
		
		val = jz_audio_volume;
		ret = val << 8;
		val = val | ret;
	   
		return put_user(val, (long *) arg);
    case SOUND_MIXER_WRITE_MIC:

	    ret = get_user(val, (long *) arg);
	    if (ret)
		    return ret;
	    
	    val = val & 0xff;
	    if(val < 0)
		    val = 0;
	    if(val > 100)
		    val = 100;
	    codec_mic_gain = val;
	    if(set_codec_mic)
		    set_codec_mic(val);

	return 0;
	case SOUND_MIXER_READ_MIC:				
		
		val = codec_mic_gain;
		ret = val << 8;
		val = val | ret;
		
		return put_user(val, (long *) arg);
	default:
		return -ENOSYS;
	}
	audio_mix_modcnt ++;
	return 0;
}


int i2s_probe_codec(struct i2s_codec *codec)
{
	/* generic OSS to I2S wrapper */
	codec->mixer_ioctl = i2s_mixer_ioctl;
	return 1;
}


/* I2S codec initialisation. */
static int __init jz_i2s_codec_init(struct jz_i2s_controller_info *controller)
{
	int num_i2s = 0;
	struct i2s_codec *codec;
    
	for (num_i2s = 0; num_i2s < NR_I2S; num_i2s++) {
		if ((codec = kmalloc(sizeof(struct i2s_codec),GFP_KERNEL)) == NULL)
			return -ENOMEM;
		memset(codec, 0, sizeof(struct i2s_codec));
		codec->private_data = controller;
		codec->id = num_i2s;
 
		if (i2s_probe_codec(codec) == 0)
			break;
		if ((codec->dev_mixer = register_sound_mixer(&jz_i2s_mixer_fops, -1)) < 0) {
			printk(KERN_ERR "Jz I2S: couldn't register mixer!\n");
			kfree(codec);
			break;
		}
		controller->i2s_codec[num_i2s] = codec;
	}
	return num_i2s;
}


static void jz_update_filler(int format, int channels)
{
#define TYPE(fmt,ch) (((fmt)<<2) | ((ch)&3))
	
	switch (TYPE(format, channels)) 
	{

	case TYPE(AFMT_U8, 1):
		jz_audio_b = 4; /* 4bytes * 8bits =32bits */
		replay_filler = replay_fill_1x8_u;
		record_filler = record_fill_1x8_u;
		break;
	case TYPE(AFMT_U8, 2):
		jz_audio_b = 4;
		replay_filler = replay_fill_2x8_u;
		record_filler = record_fill_2x8_u;
		break;
	case TYPE(AFMT_S16_LE, 1):
		jz_audio_b = 2; /* 2bytes * 16bits =32bits */
		replay_filler = replay_fill_1x16_s;
		record_filler = record_fill_1x16_s;
		break;
	case TYPE(AFMT_S16_LE, 2):
		jz_audio_b = 2;
		replay_filler = replay_fill_2x16_s;
		record_filler = record_fill_2x16_s;
		break;
	default:
		;
	}
}


#ifdef CONFIG_PROC_FS
extern struct proc_dir_entry *proc_jz_root;
int i2s_read_proc (char *page, char **start, off_t off, int count, int *eof, void *data)
{
	return 0;
}

static int jz_i2s_init_proc(struct jz_i2s_controller_info *controller)
{
	if (!create_proc_read_entry ("i2s", 0, proc_jz_root, i2s_read_proc, controller->i2s_codec[0]))
		return -EIO;
	return 0;
}

static void jz_i2s_cleanup_proc(struct jz_i2s_controller_info *controller)
{
}
#endif

static void __init attach_jz_i2s(struct jz_i2s_controller_info *controller)
{
	char *name;
	int adev; /* No of Audio device. */
  
	name = controller->name;
        /* initialize AIC controller and reset it */
	jz_i2s_initHw(1);
	adev = register_sound_dsp(&jz_i2s_audio_fops, -1);
	if (adev < 0)
		goto audio_failed;
	/* initialize I2S codec and register /dev/mixer */
	if (jz_i2s_codec_init(controller) <= 0)
		goto mixer_failed;

#ifdef CONFIG_PROC_FS
	if (jz_i2s_init_proc(controller) < 0) {
		printk(KERN_ERR "%s: can't create I2S proc filesystem.\n", name);
		goto proc_failed;
    }
#endif

	controller->tmp1 = (void *)__get_free_pages(GFP_KERNEL, 8);
	if (!controller->tmp1) {
		printk(KERN_ERR "%s: can't allocate tmp buffers.\n", controller->name);
		goto tmp1_failed;
	}
	controller->tmp2 = (void *)__get_free_pages(GFP_KERNEL, 8);
	if (!controller->tmp2) {
		printk(KERN_ERR "%s: can't allocate tmp buffers.\n", controller->name);
		goto tmp2_failed;
	}
	if ((controller->dma2 = jz_request_dma(DMA_ID_I2S_RX, "audio adc", jz_i2s_record_dma_irq, IRQF_DISABLED, controller)) < 0) {
		printk(KERN_ERR "%s: can't reqeust DMA ADC channel.\n", name);
		goto dma2_failed;
	}
	if ((controller->dma1 = jz_request_dma(DMA_ID_I2S_TX, "audio dac", jz_i2s_replay_dma_irq, IRQF_DISABLED, controller)) < 0) {
		printk(KERN_ERR "%s: can't reqeust DMA DAC channel.\n", name);
		goto dma1_failed;
	}
	printk("JzSOC On-Chip I2S controller registered (DAC: DMA(play):%d/IRQ%d,\n ADC: DMA(record):%d/IRQ%d)\n", controller->dma1, get_dma_done_irq(controller->dma1), controller->dma2, get_dma_done_irq(controller->dma2));

	controller->dev_audio = adev;
	pop_turn_onoff_buf = __get_free_pages(GFP_KERNEL | GFP_DMA, 8);
	if(!pop_turn_onoff_buf)
		printk("pop_turn_onoff_buf alloc is wrong!\n");
	pop_turn_onoff_pbuf = virt_to_phys((void *)pop_turn_onoff_buf);

	return;
dma2_failed:
	jz_free_dma(controller->dma1);
dma1_failed:
	free_pages((unsigned long)controller->tmp2, 8);
tmp2_failed:
	free_pages((unsigned long)controller->tmp1, 8);
tmp1_failed:

#ifdef CONFIG_PROC_FS
	jz_i2s_cleanup_proc(controller);
#endif
proc_failed:
	/* unregister mixer dev */
mixer_failed:
	unregister_sound_dsp(adev);
audio_failed:
	return;
}

static int __init probe_jz_i2s(struct jz_i2s_controller_info **controller)
{
	if ((*controller = kmalloc(sizeof(struct jz_i2s_controller_info),
				   GFP_KERNEL)) == NULL) {
		printk(KERN_ERR "Jz I2S Controller: out of memory.\n");
		return -ENOMEM;
	}
	(*controller)->name = "Jz I2S controller";
	(*controller)->opened1 = 0;
	(*controller)->opened2 = 0;
	init_waitqueue_head(&(*controller)->adc_wait);
	init_waitqueue_head(&(*controller)->dac_wait);
	spin_lock_init(&(*controller)->lock);
	init_waitqueue_head(&rx_wait_queue);
	init_waitqueue_head(&tx_wait_queue);
	init_waitqueue_head(&pop_wait_queue);
	init_waitqueue_head(&drain_wait_queue);

	return 0;
}

static void __exit unload_jz_i2s(struct jz_i2s_controller_info *controller)
{
	int adev = controller->dev_audio;

	jz_i2s_full_reset(controller);
	controller->dev_audio = -1;
	if (old_mksound)
		kd_mksound = old_mksound;/* Our driver support bell for kb, see vt.c */
	
#ifdef CONFIG_PROC_FS
	jz_i2s_cleanup_proc(controller);
#endif
 
	jz_free_dma(controller->dma1);
	jz_free_dma(controller->dma2);
	free_pages((unsigned long)controller->tmp1, 8);
	free_pages((unsigned long)controller->tmp2, 8);
	free_pages((unsigned long)pop_turn_onoff_buf, 8);
    
	if (adev >= 0) {
		/* unregister_sound_mixer(audio_devs[adev]->mixer_dev); */
		unregister_sound_dsp(controller->dev_audio);
	}
}

#ifdef CONFIG_PM
static int jz_i2s_suspend(struct jz_i2s_controller_info *controller, int state)
{       	
	if(i2s_suspend_codec)
		i2s_suspend_codec(controller->opened1,controller->opened2);
	printk("Aic and codec are suspended!\n");
	return 0;
}

static int jz_i2s_resume(struct jz_i2s_controller_info *controller)
{
	if(i2s_resume_codec)
		i2s_resume_codec();

#if defined(CONFIG_I2S_AK4642EN)
	jz_i2s_initHw(0);
            jz_audio_reset();
            __i2s_enable();
            jz_audio_set_speed(controller->dev_audio,jz_audio_speed);
	    /* playing */
            if(controller->opened1) {
		    if(set_codec_replay)
			    set_codec_replay();
		    int dma = controller->dma1;
		    int id;
		    unsigned long flags;
		    disable_dma(dma);
		    if(__dmac_channel_address_error_detected(dma)) {
			    printk(KERN_DEBUG "%s: DMAC address error.\n", __FUNCTION__);
			    __dmac_channel_clear_address_error(dma);
		    }
		    if(__dmac_channel_transmit_end_detected(dma)) 
			    __dmac_channel_clear_transmit_end(dma);

                    /* for DSP_GETOPTR */
                    spin_lock_irqsave(&controller->ioctllock, flags);
                    controller->total_bytes += jz_audio_dma_tran_count;
                    controller->blocks ++;
                    spin_unlock_irqrestore(&controller->ioctllock, flags);
                    while((id = get_buffer_id(&out_busy_queue)) >= 0)
			    put_buffer_id(&out_empty_queue, id);
          
                    out_busy_queue.count=0;
                    if((id = get_buffer_id(&out_full_queue)) >= 0) {
			    put_buffer_id(&out_empty_queue, id);
                    }
                    if (elements_in_queue(&out_empty_queue) > 0) {
			    wake_up(&tx_wait_queue);
			    wake_up(&controller->dac_wait);
                    } else
			    printk("pm out_empty_queue empty");
	    }

	    /* recording */
            if(controller->opened2) {
		    if(set_codec_record)
			    set_codec_record();
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
		    }
		    /* for DSP_GETIPTR */
		    spin_lock_irqsave(&controller->ioctllock, flags);
		    controller->total_bytes += jz_audio_dma_tran_count;
		    controller->blocks ++;
		    spin_unlock_irqrestore(&controller->ioctllock, flags);
		    id1 = get_buffer_id(&in_busy_queue);
		    put_buffer_id(&in_full_queue, id1);
		    wake_up(&rx_wait_queue);
		    wake_up(&controller->adc_wait);
		    if ((id2 = get_buffer_id(&in_empty_queue)) >= 0) {
			    put_buffer_id(&in_full_queue, id2);
		    } 
		    in_busy_queue.count = 0;
            }
#endif

	    return 0;
}

static int jz_i2s_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
{
	int ret;
	struct jz_i2s_controller_info *controller = pm_dev->data;

	if (!controller) return -EINVAL;

	switch (req) {
	case PM_SUSPEND:
		ret = jz_i2s_suspend(controller, (int)data);
		break;
	case PM_RESUME:
		ret = jz_i2s_resume(controller);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}
#endif /* CONFIG_PM */
static irqreturn_t aic_codec_irq(int irq, void *dev_id)
{
	u8 file_9 = read_codec_file(9);
	u8 file_8 = read_codec_file(8);
	
	//printk("--- 8:0x%x  9:0x%x ---\n",file_8,file_9);
	if ((file_9 & 0x1f) == 0x10) {

		write_codec_file(8, 0x3f);
		write_codec_file_bit(5, 1, 6);//SB_OUT->1
		mdelay(300);
		while ((read_codec_file(9) & 0x4) != 0x4);
		while ((read_codec_file(9) & 0x10) == 0x10) {
			write_codec_file(9, 0x10);
		}
		write_codec_file_bit(5, 0, 6);//SB_OUT->0
		mdelay(300);
		while ((read_codec_file(9) & 0x8) != 0x8);
		write_codec_file(9, file_9);
		write_codec_file(8, file_8);

		return IRQ_HANDLED;
	}

	if (file_9 & 0x8)
		ramp_up_end = jiffies;
	else if (file_9 & 0x4)
		ramp_down_end = jiffies;
	else if (file_9 & 0x2)
		gain_up_end = jiffies;
	else if (file_9 & 0x1)
		gain_down_end = jiffies;

	write_codec_file(9, file_9);
	if (file_9 & 0xf)
		wake_up(&pop_wait_queue);
	while (REG_ICDC_RGDATA & 0x100);
	
	return IRQ_HANDLED;
}

static int __init init_jz_i2s(void)
{
	int errno, retval;
#if defined(CONFIG_I2S_DLV)

	ramp_up_start = 0;
	ramp_up_end = 0;
	gain_up_start = 0;
	gain_up_end = 0;
	ramp_down_start = 0;
	ramp_down_end = 0;
	gain_down_start = 0;
	gain_down_end = 0;
#endif

	abnormal_data_count = 0;
	if(set_codec_mode)
		set_codec_mode();

	drain_flag = 0;
	if ((errno = probe_jz_i2s(&i2s_controller)) < 0)
		return errno;
	if(set_codec_gpio_pin)
		set_codec_gpio_pin();
	
	attach_jz_i2s(i2s_controller);
	if(set_codec_startup_param)
		set_codec_startup_param();
#if defined(CONFIG_I2S_DLV)
	jz_codec_config = 0;
	retval = request_irq(IRQ_AIC, aic_codec_irq, IRQF_DISABLED, "aic_codec_irq", NULL);
	if (retval) {
		printk("Could not get aic codec irq %d\n", IRQ_AIC);
		return retval;
	}
#endif
	if(set_codec_volume_table)
		set_codec_volume_table();

	out_empty_queue.id = NULL;
	out_full_queue.id = NULL;
	out_busy_queue.id = NULL;
	in_empty_queue.id = NULL;
	in_full_queue.id = NULL;
	in_busy_queue.id = NULL;

	jz_audio_fragsize = JZCODEC_RW_BUFFER_SIZE * PAGE_SIZE;
	jz_audio_fragstotal = JZCODEC_RW_BUFFER_TOTAL ;
	Init_In_Out_queue(jz_audio_fragstotal,jz_audio_fragsize);

#ifdef CONFIG_PM
	i2s_controller->pm = pm_register(PM_SYS_DEV, PM_SYS_UNKNOWN, 
					 jz_i2s_pm_callback);
	if (i2s_controller->pm)
		i2s_controller->pm->data = i2s_controller;
#endif

#if defined(CONFIG_I2S_DLV)
	__cpm_start_idct();
	__cpm_start_db();
	__cpm_start_me();
	__cpm_start_mc();
	__cpm_start_ipu();
#endif

	printk("JZ I2S OSS audio driver initialized\n");

	return 0;
}

static void __exit cleanup_jz_i2s(void)
{
#ifdef CONFIG_PM
	/* pm_unregister(i2s_controller->pm); */
#endif
#if defined(CONFIG_I2S_DLV)
	free_irq(IRQ_AIC, NULL);
#endif
	unload_jz_i2s(i2s_controller);
	Free_In_Out_queue(jz_audio_fragstotal,jz_audio_fragsize); 
	if(clear_codec_mode)
		clear_codec_mode();
}

module_init(init_jz_i2s);
module_exit(cleanup_jz_i2s);

#if defined(CONFIG_SOC_JZ4730)
static int drain_adc(struct jz_i2s_controller_info *ctrl, int nonblock)
{
	DECLARE_WAITQUEUE(wait, current);
	unsigned long flags;
	int count,con;
	
	if(elements_in_queue(&in_busy_queue) > 0) {
		if (nonblock)
			return -EBUSY;
		drain_flag = 1;
		sleep_on(&drain_wait_queue);
		drain_flag = 0;
	} else {
		add_wait_queue(&ctrl->adc_wait, &wait);
		for (con = 0; con < 1000; con ++) {
			udelay(1);
			set_current_state(TASK_INTERRUPTIBLE);
			spin_lock_irqsave(&ctrl->lock, flags);
			count = get_dma_residue(ctrl->dma2);
			spin_unlock_irqrestore(&ctrl->lock, flags);
			if (count <= 0)
				break;
			if (nonblock) {
				remove_wait_queue(&ctrl->adc_wait, &wait);
				current->state = TASK_RUNNING;
				return -EBUSY;
			}
		}
		remove_wait_queue(&ctrl->adc_wait, &wait);
		current->state = TASK_RUNNING;
	}
	return 0;
}

static int drain_dac(struct jz_i2s_controller_info *ctrl, int nonblock)
{
	DECLARE_WAITQUEUE(wait, current);
	unsigned long flags;
	int count;
	
	if(elements_in_queue(&out_full_queue) > 0) {
		if (nonblock)
			return -EBUSY;
		
		drain_flag = 1;
		sleep_on(&drain_wait_queue);
		drain_flag = 0;
	} else {
		add_wait_queue(&(ctrl->dac_wait), &wait);
		for (;;) {
			set_current_state(TASK_INTERRUPTIBLE);
			if(elements_in_queue(&out_full_queue) <= 0) {
				spin_lock_irqsave(&ctrl->lock, flags);
				count = get_dma_residue(ctrl->dma1);
				spin_unlock_irqrestore(&ctrl->lock, flags);
				if(count <= 0)
					break;
			}
			if (nonblock) {
				remove_wait_queue(&ctrl->dac_wait, &wait);
				current->state = TASK_RUNNING;
				return -EBUSY;
			}
		}
		remove_wait_queue(&ctrl->dac_wait, &wait);
		current->state = TASK_RUNNING;
	}
	
	return 0;
}
#endif

#if defined(CONFIG_SOC_JZ4750) || defined(CONFIG_SOC_JZ4750D)
static int drain_adc(struct jz_i2s_controller_info *ctrl, int nonblock)
{
	//DECLARE_WAITQUEUE(wait, current);
	unsigned long flags;
	int count,i=0;

	//add_wait_queue(&ctrl->adc_wait, &wait);
	for (;;) {
		if (i < MAXDELAY) {
			udelay(10);
			i++;
		} else
			break; 
		//set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&ctrl->lock, flags);
		//spin_lock(&ctrl->lock);
		count = get_dma_residue(ctrl->dma2);
		spin_unlock_irqrestore(&ctrl->lock, flags);
		//spin_unlock(&ctrl->lock);
		if (count <= 0)
			break;
		
		/*if (signal_pending(current))
		  break;*/
		if (nonblock) {
			//remove_wait_queue(&ctrl->adc_wait, &wait);
			//current->state = TASK_RUNNING;
			return -EBUSY;
		}
	}
	//remove_wait_queue(&ctrl->adc_wait, &wait);
	//current->state = TASK_RUNNING;
	/*if (signal_pending(current))
	  return -ERESTARTSYS;*/
	return 0;
}
static int drain_dac(struct jz_i2s_controller_info *ctrl, int nonblock)
{
	unsigned long flags;
	int count,ele,busyele,emptyele,i=0;

	for (;;) {
		if(!nonblock) {//blocked
               		if (i < MAXDELAY) {
				udelay(10);
				i++;
	       		} else
		  		break; 
			
			ele = elements_in_queue(&out_full_queue);
			if(ele <= 0) {
				udelay(200);
				
				busyele = elements_in_queue(&out_busy_queue);
				emptyele = elements_in_queue(&out_empty_queue);
				if (busyele <= 0 && emptyele >= jz_audio_fragstotal) {
					spin_lock_irqsave(&ctrl->lock, flags);
					count = get_dma_residue(ctrl->dma1);
					spin_unlock_irqrestore(&ctrl->lock, flags);
					if (count <= 0)
						break;
				}
			}
		} else {//non-blocked
			//mdelay(100);
			ele = elements_in_queue(&out_full_queue);
			
			if(ele <= 0) {
				//mdelay(100);
				busyele = elements_in_queue(&out_busy_queue);
				emptyele = elements_in_queue(&out_empty_queue);
				
				if (busyele <= 0 && emptyele >= jz_audio_fragstotal) {
					spin_lock_irqsave(&ctrl->lock, flags);
					count = get_dma_residue(ctrl->dma1);
					spin_unlock_irqrestore(&ctrl->lock, flags);
					if (count <= 0)
						break;
				}
			}
		}
	}
	
	return 0;
}
#endif

#if defined(CONFIG_SOC_JZ4740)
#define MAXDELAY 50000
static int drain_dac(struct jz_i2s_controller_info *ctrl, int nonblock)
{
	int count,ele,i=0;
	
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
				spin_lock(&ctrl->lock);
				count = get_dma_residue(ctrl->dma1);
				spin_unlock(&ctrl->lock);
				if (count <= 0)
					break;
			}
		} else {//non-blocked
			mdelay(100);
			ele = elements_in_queue(&out_full_queue);
			
			if(ele <= 0) {
				mdelay(100); 
				
				spin_lock(&ctrl->lock);
				count = get_dma_residue(ctrl->dma1);
				spin_unlock(&ctrl->lock);
				if (count <= 0)
					break;
			}
		}
	}
	
	return 0;
}

static int drain_adc(struct jz_i2s_controller_info *ctrl, int nonblock)
{
	int count,i=0;
	
	for (;;) {
		if ( i < MAXDELAY )
		{
			udelay(10);
			i++;
		}
		else
			break; 
		spin_lock(&ctrl->lock);
		count = get_dma_residue(ctrl->dma2);
		spin_unlock(&ctrl->lock);
		if (count <= 0)
			break;
		
		if (nonblock) {
			return -EBUSY;
		}
	}
	
	return 0;
}
#endif

static int jz_audio_release(struct inode *inode, struct file *file)
{
	unsigned long flags;
	struct jz_i2s_controller_info *controller = (struct jz_i2s_controller_info *) file->private_data;
	unsigned long tfl;
	
	if (controller == NULL)
		return -ENODEV;

	pop_dma_flag = 0;
	if (controller->opened1 == 1) {
		controller->opened1 = 0; 
		__i2s_enable_transmit_dma();
		__i2s_enable_replay();
		drain_dac(controller, file->f_flags & O_NONBLOCK);
		/* add some mute to anti-pop */
#if defined(CONFIG_I2S_DLV)
		/* wait for fifo empty */
		write_codec_file_bit(1, 1, 5);//DAC_MUTE->1
		gain_down_start = jiffies;
		sleep_on(&pop_wait_queue);
		//gain_down_end = jiffies;
		while (1) {
			tfl = REG_AIC_SR & 0x00003f00;
			if (tfl == 0) {
				udelay(500);
				break;
			}
			mdelay(2);
		}
#endif
		disable_dma(controller->dma1);
		set_dma_count(controller->dma1, 0);
		__i2s_disable_transmit_dma();
		__i2s_disable_replay();
		__aic_flush_fifo();
		if(clear_codec_replay)
			clear_codec_replay();
		__aic_flush_fifo();
  
		spin_lock_irqsave(&controller->ioctllock, flags);
		controller->total_bytes = 0;
		controller->count = 0;
		controller->finish = 0;
		jz_audio_dma_tran_count = 0;
		controller->blocks = 0;
		controller->nextOut = 0;
		spin_unlock_irqrestore(&controller->ioctllock, flags);

#if defined(CONFIG_I2S_DLV)
		write_codec_file_bit(5, 1, 6);//SB_OUT->1
		ramp_down_start = jiffies;
		sleep_on(&pop_wait_queue);
		//ramp_down_end = jiffies;
		unset_audio_data_replay();	
#endif
		__i2s_disable();
		if(turn_off_codec)
			turn_off_codec();
	}
	
	if (controller->opened2 == 1) {
		controller->opened2 = 0;
		first_record_call = 1;
		__i2s_enable_receive_dma();
		__i2s_enable_record();
		drain_adc(controller, file->f_flags & O_NONBLOCK);
		disable_dma(controller->dma2);
		set_dma_count(controller->dma2, 0);
		__i2s_disable_receive_dma();
		__i2s_disable_record();

		if(clear_codec_record)
			clear_codec_record();

		spin_lock_irqsave(&controller->ioctllock, flags);
		controller->total_bytes = 0;
		jz_audio_dma_tran_count = 0;
		controller->count = 0;
		controller->finish = 0;
		controller->blocks = 0;
		controller->nextIn = 0;
		spin_unlock_irqrestore(&controller->ioctllock, flags);
		__i2s_disable();
		if(turn_off_codec)
			turn_off_codec();
		abnormal_data_count = 0;
	}

#if defined(CONFIG_I2S_DLV)
	write_codec_file(9, 0xff);
	write_codec_file(8, 0x3f);
#endif
	return 0;
}

static int jz_audio_open(struct inode *inode, struct file *file)
{
	int i;
	struct jz_i2s_controller_info *controller = i2s_controller;
	
	if (controller == NULL)
		return -ENODEV;
	
	mdelay(2);
	REG_DMAC_DMACKE(0) = 0x3f;
	pop_dma_flag = 0;
	if (controller->opened1 == 1 || controller->opened2 == 1 ) {
		printk("\naudio is busy!\n");
		return -EBUSY;
	}
	jz_codec_config = 0;

	ramp_up_start = 0;
	ramp_up_end = 0;
	gain_up_start = 0;
	gain_up_end = 0;
	ramp_down_start = 0;
	ramp_down_end = 0;
	gain_down_start = 0;
	gain_down_end = 0;
	if (file->f_mode & FMODE_WRITE) {
		if (controller->opened1 == 1)
			return -EBUSY;
	
		controller->opened1 = 1;
		/* for ioctl */
		controller->total_bytes = 0;
		jz_audio_dma_tran_count = 0;
		controller->count = 0;
		controller->finish = 0;
		controller->blocks = 0;
		controller->nextOut = 0;

		for(i=0;i < 64;i++) {
			save_last_samples[i].left = 0;
			save_last_samples[i].right = 0;
		}

		out_empty_queue.count = jz_audio_fragstotal;	
		for (i=0;i < jz_audio_fragstotal;i++)
			*(out_empty_queue.id + i) = i;
		out_busy_queue.count = 0;
		out_full_queue.count = 0;
		last_dma_buffer_id = 0;
    }

	if (file->f_mode & FMODE_READ) {
		if (controller->opened2 == 1)
			return -EBUSY;

		controller->opened2 = 1;
		first_record_call = 1;
		/* for ioctl */
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
    }
    
	file->private_data = controller;
	jz_audio_reset();
	REG_AIC_FR |= (1 << 6);
	
	if (file->f_mode & FMODE_WRITE) {
		if(set_codec_replay)
			set_codec_replay();
	}

	if (file->f_mode & FMODE_READ) {
		abnormal_data_count = 0;
		if(set_codec_record)
			set_codec_record();
	}

#if defined(CONFIG_I2S_DLV)
	__aic_reset();
	
	mdelay(10);
	REG_AIC_I2SCR = 0x10;
	mdelay(20);
	__aic_flush_fifo();
#endif

	__i2s_enable();

#if defined(CONFIG_I2S_DLV)
	if (file->f_mode & FMODE_WRITE) {

		write_codec_file_bit(5, 0, 6);//PMR1.SB_OUT->0
		ramp_up_start = jiffies;
		/*while (!(REG_RTC_RCR & RTC_RCR_WRDY));
		REG_RTC_RCR = 0x1;
		while (!(REG_RTC_RCR & RTC_RCR_WRDY));
		REG_RTC_RGR = 1;*/
		sleep_on(&pop_wait_queue);
		//ramp_up_end = jiffies;
		write_codec_file_bit(5, 1, 4);//SB_ADC->1
	} else if (file->f_mode & FMODE_READ) {
		if (jz_mic_only)
			write_codec_file_bit(5, 1, 7);//SB_DAC->1
		else
			write_codec_file_bit(5, 0, 7);//SB_DAC->0
		mdelay(500);
	}	

#endif

	return 0;
}


static int jz_audio_ioctl(struct inode *inode, struct file *file,
unsigned int cmd, unsigned long arg)
{
	int val,fullc,busyc,unfinish,newfragstotal,newfragsize;
	unsigned int flags;
	struct jz_i2s_controller_info *controller = (struct jz_i2s_controller_info *) file->private_data;
	count_info cinfo;
	audio_buf_info abinfo;
	int id, i;

	val = 0;
	switch (cmd) {
	case OSS_GETVERSION:
		return put_user(SOUND_VERSION, (int *)arg);
	case SNDCTL_DSP_RESET:
#if 0
		jz_audio_reset();
		__i2s_disable_replay();
		__i2s_disable_receive_dma();
		__i2s_disable_record();
		__i2s_disable_transmit_dma();
#endif
		return 0;
	case SNDCTL_DSP_SYNC:
		if (file->f_mode & FMODE_WRITE)
			return drain_dac(controller, file->f_flags & O_NONBLOCK);
		return 0;
	case SNDCTL_DSP_SPEED:
		/* set smaple rate */
		if (get_user(val, (int *)arg))
			return -EFAULT;
		if (val >= 0)
			jz_audio_set_speed(controller->dev_audio, val);
		
		return put_user(val, (int *)arg);
	case SNDCTL_DSP_STEREO:
		/* set stereo or mono channel */
		if (get_user(val, (int *)arg))
		    return -EFAULT;
		jz_audio_set_channels(controller->dev_audio, val ? 2 : 1);
	    
		return 0;
	case SNDCTL_DSP_GETBLKSIZE:
		//return put_user(jz_audio_fragsize / jz_audio_b, (int *)arg);
		return put_user(jz_audio_fragsize, (int *)arg);
	case SNDCTL_DSP_GETFMTS:
		/* Returns a mask of supported sample format*/
		return put_user(AFMT_U8 | AFMT_S16_LE, (int *)arg);
	case SNDCTL_DSP_SETFMT:
		/* Select sample format */
		if (get_user(val, (int *)arg))
			return -EFAULT;
		if (val != AFMT_QUERY)
			jz_audio_set_format(controller->dev_audio,val);
		else {
			if (file->f_mode & FMODE_READ)
				val = (jz_audio_format == 16) ?  AFMT_S16_LE : AFMT_U8;
			else
				val = (jz_audio_format == 16) ?  AFMT_S16_LE : AFMT_U8;
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
		get_user(val, (long *) arg);
		newfragsize = 1 << (val & 0xFFFF);
		if (newfragsize < 4 * PAGE_SIZE)
			newfragsize = 4 * PAGE_SIZE;
		if (newfragsize > (16 * PAGE_SIZE))
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
		int i, bytes = 0;
		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		
		spin_lock_irqsave(&controller->ioctllock, flags);
		jz_audio_fragments = elements_in_queue(&out_empty_queue);
		for (i = 0; i < jz_audio_fragments; i++)
			bytes += jz_audio_fragsize;

		if (jz_audio_channels == 2)
			bytes /= jz_audio_b;
		else if  (jz_audio_channels == 1)
			bytes /= 4;
		else 
		printk("SNDCTL_DSP_GETOSPACE : channels is wrong 1!\n");
		
		
		spin_unlock_irqrestore(&controller->ioctllock, flags);
                /* unused fragment amount */
		abinfo.fragments = jz_audio_fragments; 
		/* amount of fragments */
		abinfo.fragstotal = jz_audio_fragstotal; 
                /* fragment size in bytes */
		if (jz_audio_channels == 2)
			abinfo.fragsize = jz_audio_fragsize / jz_audio_b;
		else if  (jz_audio_channels == 1)
			abinfo.fragsize = jz_audio_fragsize / 4;
		else 
			printk("SNDCTL_DSP_GETOSPACE : channels is wrong 2!\n");
 
		/* write size count without blocking in bytes */
		abinfo.bytes = bytes;

		return copy_to_user((void *)arg, &abinfo, 
				    sizeof(abinfo)) ? -EFAULT : 0;
	}
	case SNDCTL_DSP_GETISPACE:
	{
		int i, bytes = 0;
		if (!(file->f_mode & FMODE_READ))
			return -EINVAL;			
		jz_audio_fragments = elements_in_queue(&in_empty_queue);
		for (i = 0; i < jz_audio_fragments; i++)
			bytes += jz_audio_fragsize;

		if (jz_audio_channels == 2)
			bytes /= jz_audio_b;
		else if  (jz_audio_channels == 1)
			bytes /= 4;
		else 
			printk("SNDCTL_DSP_GETISPACE : channels is wrong 1!\n");
	    
		abinfo.fragments = jz_audio_fragments;
		abinfo.fragstotal = jz_audio_fragstotal;

		if (jz_audio_channels == 2)
			abinfo.fragsize = jz_audio_fragsize / jz_audio_b;
		else if  (jz_audio_channels == 1)
			abinfo.fragsize = jz_audio_fragsize / 4;
		else 
			printk("SNDCTL_DSP_GETISPACE : channels is wrong 2!\n");
		
		abinfo.bytes = bytes;
		
		return copy_to_user((void *)arg, &abinfo, 
				    sizeof(abinfo)) ? -EFAULT : 0;
	}
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
		
		if (jz_audio_channels == 2)
			unfinish /= jz_audio_b;
		else if  (jz_audio_channels == 1)
			unfinish /= 4;
		else 
		printk("SNDCTL_DSP_GETODELAY : channels is wrong !\n");
	    
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
	struct jz_i2s_controller_info *controller = (struct jz_i2s_controller_info *) file->private_data;
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
	} else if (file->f_mode & FMODE_READ) {
		if (elements_in_queue(&in_full_queue) > 0)
			mask |= POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&controller->lock, flags);

	return mask;
}

static ssize_t jz_audio_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	struct jz_i2s_controller_info *controller = (struct jz_i2s_controller_info *) file->private_data;
	int id, ret = 0, left_count, copy_count, cnt = 0;
	unsigned long flags;

	if (count < 0)
		return -EINVAL;
	
	__i2s_enable_receive_dma();
	__i2s_enable_record();

	spin_lock_irqsave(&controller->ioctllock, flags);
	controller->nextIn = 0;
	spin_unlock_irqrestore(&controller->ioctllock, flags);

	copy_count = jz_audio_fragsize / 4;

	left_count = count;    
	if (first_record_call) {
		first_record_call = 0;
	audio_read_back_first:
		if ((id = get_buffer_id(&in_empty_queue)) >= 0) {
			put_buffer_id(&in_busy_queue, id);
			spin_lock(&controller->lock); 
			*(in_dma_buf_data_count + id) = copy_count * 4;

			spin_unlock(&controller->lock);
			__i2s_enable_receive_dma();
			__i2s_enable_record();
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
			cnt = record_filler((unsigned long)controller->tmp2+ret, copy_count, id);
			spin_unlock(&controller->lock);
			put_buffer_id(&in_empty_queue, id);
		} else
			goto audio_read_back_second;
		
		if (elements_in_queue(&in_busy_queue) == 0) {
			if ((id=get_buffer_id(&in_empty_queue)) >= 0) {
				put_buffer_id(&in_busy_queue, id);
				spin_lock(&controller->lock);
				*(in_dma_buf_data_count + id) = copy_count * 4;
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
		if (copy_to_user(buffer+ret, controller->tmp2+ret, cnt))
			return ret ? ret : -EFAULT;

		spin_lock(&controller->lock);
		ret += cnt;
		spin_unlock(&controller->lock);

		spin_lock_irqsave(&controller->ioctllock, flags);
		controller->nextIn += ret;
		spin_unlock_irqrestore(&controller->ioctllock, flags);

		spin_lock(&controller->lock);
		left_count -= cnt;
		spin_unlock(&controller->lock);
	}
	return ret;
}

static ssize_t jz_audio_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	int id, ret = 0, left_count, copy_count = 0;
	unsigned int flags;
	struct jz_i2s_controller_info *controller = (struct jz_i2s_controller_info *) file->private_data;
	
	if (count <= 0)
		return -EINVAL;
	
	if(set_replay_hp_or_speaker)
		set_replay_hp_or_speaker();
	
	__i2s_enable_transmit_dma();
	__i2s_enable_replay();
	
	spin_lock_irqsave(&controller->ioctllock, flags);
	controller->nextOut = 0;
	spin_unlock_irqrestore(&controller->ioctllock, flags);
	if (jz_audio_channels == 2)
		copy_count = jz_audio_fragsize / jz_audio_b;
	else if(jz_audio_channels == 1)
		copy_count = jz_audio_fragsize / 4;
	left_count = count;
	if (copy_from_user(controller->tmp1, buffer, count)) {
		printk("copy_from_user failed:%d",ret);
		return ret ? ret : -EFAULT;
	}

	while (left_count > 0) {
	audio_write_back:
		/*if (file->f_flags & O_NONBLOCK)
		  udelay(2);*/
		if (elements_in_queue(&out_empty_queue) == 0) {
			if (file->f_flags & O_NONBLOCK)
				return ret;
			else
				sleep_on(&tx_wait_queue);
		}
		/* the end fragment size in this write */
		if (ret + copy_count > count)
			copy_count = count - ret;
		if ((id = get_buffer_id(&out_empty_queue)) >= 0) {
			replay_filler((signed long)controller->tmp1 + ret, copy_count, id);
			if(*(out_dma_buf_data_count + id) > 0) {
				put_buffer_id(&out_full_queue, id);
				dma_cache_wback_inv(*(out_dma_buf + id), 
						    *(out_dma_buf_data_count + id));
			} else
				put_buffer_id(&out_empty_queue, id);
		} else
			goto audio_write_back;
		
		left_count = left_count - copy_count;
		ret += copy_count;

		spin_lock_irqsave(&controller->ioctllock, flags);
		controller->nextOut += ret;
		spin_unlock_irqrestore(&controller->ioctllock, flags);

		if (elements_in_queue(&out_busy_queue) == 0) {
			if ((id=get_buffer_id(&out_full_queue)) >= 0) {
				put_buffer_id(&out_busy_queue, id);

				if(*(out_dma_buf_data_count + id) > 0) {  	
					audio_start_dma(controller->dma1,
							file->private_data,
							*(out_dma_pbuf + id),
							*(out_dma_buf_data_count + id),
							DMA_MODE_WRITE);
					last_dma_buffer_id = id;
					if (jz_codec_config == 0) {
						write_codec_file_bit(1, 0, 5);
						gain_up_start = jiffies;
						sleep_on(&pop_wait_queue);
						//gain_up_end = jiffies;
						jz_codec_config = 1;
						//SB_ADC->1
						//write_codec_file_bit(5, 1, 4);
						//while(1);
					}
				}
			}
		}
	}

	return ret;
}

#if defined(CONFIG_I2S_ICODEC)
static void write_mute_to_dma_buffer(signed long l_sample, signed long r_sample)
{
	int i,step_len;
	unsigned long *pop_buf = (unsigned long*)pop_turn_onoff_buf;
	unsigned int sample_oss = (REG_AIC_CR & 0x00380000) >> 19;
	unsigned long l_sample_count,r_sample_count,sample_count;
	struct jz_i2s_controller_info *controller = i2s_controller;
	signed int left_sam=0,right_sam=0,l_val,r_val;
	
	switch (sample_oss) {
	case 0x0:
		break;
	case 0x1:
		left_sam = (signed int)l_sample;
		right_sam = (signed int)r_sample;
		break;
	case 0x2:
		break;
	case 0x3:
		break;
	case 0x4:
		break;
	}

	if(left_sam == 0 && right_sam == 0)
		return;
    	
	switch (sample_oss) {
	case 0x0:			   
		break;
	case 0x1:
		step_len = jz_audio_speed / 10 * 3;
		step_len = step_len / 2;
		step_len = 0x7fff / step_len + 1;

		l_sample_count = 0;
		l_val = left_sam;

		while(1) {
			if(l_val > 0) {
				if(l_val >= step_len) {
					l_val -= step_len;
					l_sample_count ++;
				} else
					break;
			} 
			
			if(l_val < 0) {
				if(l_val <= -step_len) {
					l_val += step_len;
					l_sample_count ++;
				} else
					break;
			}
			
			if(l_val == 0) 
				break;
		}
		
		r_sample_count = 0;
		r_val = right_sam;
		while(1) {
			if(r_val > 0) {
				if(r_val >= step_len) {
					r_val -= step_len;
					r_sample_count ++;
				} else
					break;
			} 
			
			if(r_val < 0) {
				if(r_val <= -step_len) {
					r_val += step_len;
					r_sample_count ++;
				} else
					break;
			}
			
			if(r_val == 0)
				break;
		}
		/* fill up */
		if(l_sample_count > r_sample_count)
			sample_count = l_sample_count;
		else
			sample_count = r_sample_count;
		
		l_val = left_sam;
		r_val = right_sam;
		for(i=0;i <= sample_count;i++) {
			
			*pop_buf = (unsigned long)l_val;		
			pop_buf ++;
			
			if(l_val > step_len)
				l_val -= step_len;
			else if(l_val < -step_len)
				l_val += step_len;
			else if(l_val >= -step_len && l_val <= step_len)
				l_val = 0;

			*pop_buf = (unsigned long)r_val;
			pop_buf ++;
			if(r_val > step_len)
				r_val -= step_len;
			else if(r_val < -step_len)
				r_val += step_len;
			else if(r_val >= -step_len && r_val <= step_len)
				r_val = 0;
		}
		
		*pop_buf = 0;
		pop_buf ++;
		*pop_buf = 0;
	
		pop_buf ++;
		sample_count += 2;	
		dma_cache_wback_inv(pop_turn_onoff_buf, sample_count*8);

		pop_dma_flag = 1;
		audio_start_dma(controller->dma1,controller,pop_turn_onoff_pbuf,sample_count*8,DMA_MODE_WRITE);
		sleep_on(&pop_wait_queue);
		pop_dma_flag = 0;
		break;
	case 0x2:
		break;
	case 0x3:
		break;
	case 0x4:
		break;
	}
}
#endif
