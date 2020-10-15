/*
 *  linux/drivers/sound/jzcodec.c
 *
 *  JzSOC internal audio driver.
 *
 */
#include <linux/init.h>
#include <linux/interrupt.h>
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

#define USE_NONE 1
#define USE_MIC 2
#define USE_LINEIN 3

typedef struct hpvol_shift_s
{
	int hpvol;
	int shift;
} hpvol_shift_t;

extern mixer_info info;
extern _old_mixer_info old_info;
extern int codec_volue_shift;
extern hpvol_shift_t hpvol_shift_table[72];
extern int abnormal_data_count;

extern void (*set_codec_mode)(void);
extern void (*each_time_init_codec)(void);
extern int (*set_codec_startup_param)(void);
extern void (*set_codec_volume_table)(void);
extern void (*set_codec_record)(int mode);
extern void (*set_codec_replay)(void);
extern void (*set_codec_replay_record)(int mode);
extern void (*turn_on_codec)(void);
extern void (*turn_off_codec)(void);
extern void (*set_codec_speed)(int rate);
extern void (*reset_codec)(void);
extern void (*codec_mixer_old_info_id_name)(void);
extern void (*codec_mixer_info_id_name)(void);
extern void (*set_codec_bass)(int val);
extern void (*set_codec_volume)(int val);
extern void (*set_codec_mic)(int val);
extern void (*set_codec_line)(int val);
extern void (*i2s_resume_codec)(void);
extern void (*i2s_suspend_codec)(void);
extern void (*set_codec_direct_mode)(void);
extern void (*clear_codec_direct_mode)(void);

static int jzcodec_reg[2];

void set_jzcodec_mode(void);
void each_time_init_jzcodec(void);
int set_jzcodec_startup_param(void);
void set_jzcodec_volume_table(void);
void set_jzcodec_replay(void);
void set_jzcodec_record(int mode);
void turn_on_jzcodec(void);
void turn_off_jzcodec(void);
void set_jzcodec_speed(int rate);
void reset_jzcodec(void);
void jzcodec_mixer_old_info_id_name(void);
void jzcodec_mixer_info_id_name(void);
void set_jzcodec_bass(int val);
void set_jzcodec_volume(int val);
void set_jzcodec_mic(int val);
void set_jzcodec_line(int val);
void in_codec_app1(void);
void in_codec_app12(void);
void HP_turn_on(void);
void HP_turn_off(void);
void resume_jzcodec(void);
void suspend_jzcodec(void);
void set_jzcodec_replay_record(int mode);

void set_jzcodec_mode(void)
{
	__i2s_internal_codec();
	__i2s_as_slave();
	__i2s_select_i2s();
	__aic_select_i2s();      

	REG_ICDC_CDCCR1 = 0x001b2303;
	schedule_timeout(1);
	REG_ICDC_CDCCR1 = 0x001b2302;
}

void each_time_init_jzcodec(void)
{
        __i2s_disable();
	__i2s_as_slave();
	__i2s_set_oss_sample_size(16);
	__i2s_set_iss_sample_size(16);
}

int set_jzcodec_startup_param(void)
{
    	REG_ICDC_CDCCR1 = 0x001b2300;
	schedule_timeout(50);                 
	REG_ICDC_CDCCR1 = 0x00033300;
	schedule_timeout(5);
	REG_ICDC_CDCCR1 = 0x17026300;

	return 1;
}
void set_jzcodec_volume_table(void)
{
	int errno,hpvol_step,sample_shift;

	codec_volue_shift = 0;
	hpvol_step = 3;
	sample_shift = 0;
	for(errno = 0;errno < 72;errno++) { 
		hpvol_shift_table[errno].hpvol = hpvol_step;
		hpvol_shift_table[errno].shift = sample_shift;
		hpvol_step --;
		if(hpvol_step <= 0) {
			hpvol_step = 3;
			sample_shift ++;			
		}		
	}
}

void set_jzcodec_replay(void)
{
	in_codec_app1();
}

void set_jzcodec_record(int mode)
{
	switch (mode) {
	case USE_NONE:
	case USE_MIC:
		in_codec_app12();
		abnormal_data_count = 0;
		break;
	case USE_LINEIN:
		REG_ICDC_CDCCR1 = 0x27022000;//for linein
		mdelay(300);
		break;		
	}
}

void set_jzcodec_replay_record(int mode)
{
	long val = 0;
	REG_ICDC_CDCCR1 = 0x00037302;
	mdelay(2);
	REG_ICDC_CDCCR1 = 0x03006000;
	mdelay(2);

	switch (mode) {
	case USE_NONE:
	case USE_MIC:
		REG_ICDC_CDCCR1 = 0x17022000;//for mic
		break;
	case USE_LINEIN:
		REG_ICDC_CDCCR1 = 0x27022000;//for linein
		break;
	}
	
	val = REG_ICDC_CDCCR2;
	val &= 0x0000ff00;
	val |= 0x00170030;
	REG_ICDC_CDCCR2 = val;
}

void turn_on_jzcodec(void)
{
	HP_turn_on();
}

void set_jzcodec_direct_mode(void)
{
	long val = 0;
	REG_ICDC_CDCCR1 = 0x14000000;
	val &= 0x0000ff00;
	val |= 0x001f0033;
	REG_ICDC_CDCCR2 = val;
	mdelay(300);
}

void clear_jzcodec_direct_mode(void)
{
	HP_turn_off();
}

void turn_off_jzcodec(void)
{
	HP_turn_off();
}

void set_jzcodec_speed(int rate)
{
	long codec_speed,speed = 0;
    switch (rate) {
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

        codec_speed = REG_ICDC_CDCCR2;
        codec_speed |= 0x00000f00;

	speed = speed << 8;
	speed |= 0xfffff0ff;
	codec_speed &= speed;
	REG_ICDC_CDCCR2 = codec_speed;
}

void reset_jzcodec(void)
{
    REG_ICDC_CDCCR1 |= 1;
    mdelay(1);
    REG_ICDC_CDCCR1 &= 0xfffffffe;
}

void jzcodec_mixer_old_info_id_name(void)
{
	strncpy(info.id, "JZCODEC", sizeof(info.id));
	strncpy(info.name,"Jz internal codec", sizeof(info.name));
}

void jzcodec_mixer_info_id_name(void)
{
	strncpy(old_info.id, "JZCODEC", sizeof(old_info.id));
	strncpy(old_info.name,"Jz internal codec", sizeof(old_info.name));
}

void set_jzcodec_bass(int val)
{
	int bass_gain = 0;
	if(val < 25)
		bass_gain = 0;
	if(val >= 25 && val < 50)
		bass_gain = 1;
	if(val >= 50 && val < 75)
		bass_gain = 2;
	if(val >= 75 && val <= 100 )
		bass_gain = 3;

	REG_ICDC_CDCCR2 = ((REG_ICDC_CDCCR2 & ~(0x3 << 4)) | (bass_gain << 4));
}

void set_jzcodec_volume(int val)
{
	unsigned int sample_oss;
	int index,shift_max,vol_scale,vol_step,codec_volume;
	shift_max = 0;
	sample_oss = (REG_AIC_CR & 0x00380000) >> 19;
      
	switch(sample_oss)
	{
	case 0x0: 
		shift_max = 4; /* 8 bits */
	    break;
	case 0x1:
		shift_max = 10; /* 16 bits */
	    break;
	case 0x2: 
 		shift_max = 12; /* 18 bits */
		break;
	case 0x3: 
		shift_max = 15; /* 20 bits */
		break;
	case 0x4:
		shift_max = 19; /* 24 bits */
		break;
	}
	
	vol_scale = 3 * (shift_max + 1);
	vol_step = 100 / vol_scale;

	for(index = 0;index <= 100;index += vol_step)
		if( val <= index )
			break;

	if(index == 0)
		index = vol_step;
	index = index / vol_step;
	if(index > vol_scale)
		index = vol_scale;

	index = vol_scale - index;
	codec_volume = hpvol_shift_table[index].hpvol; 
	codec_volue_shift = hpvol_shift_table[index].shift;
	codec_volume = 3;
	REG_ICDC_CDCCR2 = ((REG_ICDC_CDCCR2 & ~(0x3)) | codec_volume);
}

void set_jzcodec_mic(int val)
{
	long mic_gain;

	mic_gain = 31 * val /100;
	REG_ICDC_CDCCR2 = (REG_ICDC_CDCCR2 | (0x3 << 4));
       	REG_ICDC_CDCCR2 = ((REG_ICDC_CDCCR2 & ~(0x1f << 16)) | (mic_gain << 16));
}

void set_jzcodec_line(int val)
{
	long line_gain;

	line_gain = 31 * val /100;
       	REG_ICDC_CDCCR2 = ((REG_ICDC_CDCCR2 & ~(0x1f << 16)) | (line_gain << 16));
}

void HP_turn_on(void)
{
	long val = 0;
	/* simple and slow anti-pop */
 	REG_ICDC_CDCCR1 = 0x00037302;
	mdelay(2);
	REG_ICDC_CDCCR1 = 0x03006000;
	mdelay(2);
	REG_ICDC_CDCCR1 = 0x03002000;

	val = REG_ICDC_CDCCR2;
	val &= 0x0000ff00;
	val |= 0x001f0033;
	REG_ICDC_CDCCR2 = val;
}

void HP_turn_off(void)
{
	mdelay(20);
	REG_ICDC_CDCCR1 = 0x00033300;
}

void in_codec_app1(void)
{
	/* test is OK */
	HP_turn_on();
}

void in_codec_app12(void)
{
	REG_ICDC_CDCCR1 = 0x14024300;
	mdelay(300);
}

void resume_jzcodec(void)
{
	REG_ICDC_CDCCR2 = 0x00170800;
	mdelay(2);
	REG_ICDC_CDCCR1 = 0x001f2102;
	mdelay(5);                 
	REG_ICDC_CDCCR1 = 0x00033302;
	mdelay(550);                 
	REG_ICDC_CDCCR1 = 0x00033300;
	REG_ICDC_CDCCR1 = jzcodec_reg[0];
	REG_ICDC_CDCCR2 = jzcodec_reg[1];
}

void suspend_jzcodec(void)
{

	jzcodec_reg[0] = REG_ICDC_CDCCR1;
	jzcodec_reg[1] = REG_ICDC_CDCCR2;

	REG_ICDC_CDCCR1 = 0x001b2302;
	mdelay(1);
	REG_ICDC_CDCCR1 = 0x001b2102;
}

static int __init init_jzcodec(void)
{
	set_codec_mode = set_jzcodec_mode;
	each_time_init_codec = each_time_init_jzcodec;
	
	set_codec_startup_param = set_jzcodec_startup_param;
	set_codec_volume_table = set_jzcodec_volume_table;
	set_codec_record = set_jzcodec_record;
	set_codec_replay = set_jzcodec_replay;
	set_codec_replay_record = set_jzcodec_replay_record;
	turn_on_codec = turn_on_jzcodec;
	turn_off_codec = turn_off_jzcodec;
	set_codec_speed = set_jzcodec_speed;
	reset_codec = reset_jzcodec;
	codec_mixer_old_info_id_name = jzcodec_mixer_old_info_id_name;
	codec_mixer_info_id_name = jzcodec_mixer_info_id_name;
	set_codec_bass = set_jzcodec_bass;
	set_codec_volume = set_jzcodec_volume;
	set_codec_mic = set_jzcodec_mic; 
	set_codec_line = set_jzcodec_line;
	i2s_resume_codec = resume_jzcodec;
	i2s_suspend_codec = suspend_jzcodec;
	set_codec_direct_mode = set_jzcodec_direct_mode;
	clear_codec_direct_mode = clear_jzcodec_direct_mode;

	return 0;
}


static void __exit cleanup_jzcodec(void)
{
        REG_ICDC_CDCCR1 = 0x001b2302;

}

module_init(init_jzcodec);
module_exit(cleanup_jzcodec);
