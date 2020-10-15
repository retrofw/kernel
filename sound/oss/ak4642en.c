/*
 * linux/sound/oss/ak4642en.c
 *
 *  AKM ak4642en codec chip driver to I2S interface
 *
 * Copyright (c) 2005-2007 Ingenic Semiconductor Inc.
 * Author: <cjfeng@ingenic.cn>
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Because the normal application of AUDIO devices are focused on Little_endian,
 * then we only perform the little endian data format in driver.
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

extern mixer_info info;
extern _old_mixer_info old_info;
extern int abnormal_data_count;

extern void (*clear_codec_mode)(void);
extern void (*set_codec_gpio_pin)(void);
extern void (*each_time_init_codec)(void);
extern void (*set_codec_record)(void);
extern void (*set_codec_replay)(void);
extern void (*clear_codec_record)(void);
extern void (*clear_codec_replay)(void);
extern void (*set_codec_speed)(int range);
extern void (*codec_mixer_old_info_id_name)(void);
extern void (*codec_mixer_info_id_name)(void);
extern void (*set_codec_volume)(int val);
extern void (*set_codec_mic)(int val);
extern void (*i2s_resume_codec)(void);
extern void (*i2s_suspend_codec)(int wr,int rd);
extern void (*set_replay_hp_or_speaker)(void);

#define I2S_PDN  68
#define JACK_PLUG_PIN  83
#define JACK_PLUG_IRQ  (IRQ_GPIO_0 + JACK_PLUG_PIN)

static int jack_plug_level, old_level;
static unsigned int i2c_addr = 0x26; //AK4642EN device address at I2C bus
static unsigned int i2c_clk = 100000;//AK4642EN 400kHz max,but 100kHz here
static unsigned int spk_hp = 0;
static int codec_volume;

void set_ak4642en_gpio_pin(void);
void each_time_init_ak4642en(void);
void set_ak4642en_replay(void);
void set_ak4642en_record(void);
void turn_on_ak4642en(void);
void turn_off_ak4642en(void);
void set_ak4642en_speed(int rate);
void reset_ak4642en(void);
void ak4642en_mixer_old_info_id_name(void);
void ak4642en_mixer_info_id_name(void);
void set_ak4642en_bass(int val);
void set_ak4642en_volume(int val);
void set_ak4642en_mic(int val);
void resume_ak4642en(void);
void suspend_ak4642en(int wr,int rd);

static void write_reg(u8 reg, u8 val)
{
	i2c_open();
	i2c_setclk(i2c_clk);
	i2c_write((i2c_addr >> 1), &val, reg, 1);
	i2c_close();
}

#if 0
static u8 read_reg(u8 reg)
{
	u8 val;
	i2c_open();
	i2c_setclk(i2c_clk);
	i2c_read((i2c_addr >> 1), &val, reg, 1);
	i2c_close();
	return val;
}

static u16 i2s_codec_read(u8 reg)
{
	u16 value;
	value = read_reg(reg);
	return value;
}
#endif

static void i2s_codec_write(u8 reg, u16 data)
{
	u8 val = data & 0xff;
	write_reg(reg, val);
}

void set_ak4642en_gpio_pin(void)
{
	//set AIC pin to I2S slave mode,only GPIO70,71,77,78
	__gpio_as_output(68);
	__gpio_clear_pin(68);
	__gpio_as_output(69);
	__gpio_clear_pin(69);
	__gpio_as_output(70);
	__gpio_clear_pin(70);
	__gpio_as_input(71);
	__gpio_clear_pin(71);
	__gpio_as_input(77);
	__gpio_clear_pin(77);
	__gpio_as_input(78);
	__gpio_clear_pin(78);
	REG_GPIO_GPALR(2) &= 0xC3FF0CFF;
	REG_GPIO_GPALR(2) |= 0x14005000;
	//set SCC clock initialization
	REG_SCC1_CR(SCC1_BASE) = 0x00000000;
	udelay(2);
	REG_SCC1_CR(SCC1_BASE) |= 1 << 31;
	udelay(2);

	__gpio_as_output(I2S_PDN);
	__gpio_set_pin(I2S_PDN);
	udelay(5);
	__gpio_clear_pin(I2S_PDN);
	ndelay(300);//>150ns
	__gpio_set_pin(I2S_PDN);
	mdelay(1);
	//set PLL Master mode
	i2s_codec_write(0x01, 0x0008);//master
	i2s_codec_write(0x04, 0x006b);//ref:12MHz;BITCLK:64fs;I2S compli
	i2s_codec_write(0x05, 0x000b);//sync:48KHz;
	i2s_codec_write(0x00, 0x0040);//PMVCM
	i2s_codec_write(0x01, 0x0009);//master,PLL enable
	mdelay(40);
	jack_plug_level = 10;
	old_level = 100;
	spk_hp = 0;
	__gpio_disable_pull(JACK_PLUG_PIN);
	udelay(10);
	__gpio_as_input(JACK_PLUG_PIN);
	jack_plug_level = __gpio_get_pin(JACK_PLUG_PIN);
	//i suppose jack_plug_lvel is 1 indicate with HPO
	if (jack_plug_level > 1 || jack_plug_level <0)
		printk("Audio ak4642en codec Jack plug level is wrong!\n");
	if (jack_plug_level)
		__gpio_as_irq_fall_edge(JACK_PLUG_PIN);
	else 
		__gpio_as_irq_rise_edge(JACK_PLUG_PIN);
}

void clear_ak4642en_mode(void)
{
	spk_hp = 0;
	i2s_codec_write(0x01, 0x0008);//master,PLL disable
	//free_irq(JACK_PLUG_IRQ, i2s_controller);
	__gpio_clear_pin(I2S_PDN);
	udelay(2);
	REG_SCC1_CR(SCC1_BASE) &= 0 << 31;
	udelay(2); 
}

void set_ak4642en_replay(void)
{
	//for poll
	/*jack_plug_level is H for SPK,is L for HP*/
	jack_plug_level = __gpio_get_pin(JACK_PLUG_PIN);
	if(old_level == jack_plug_level)
		return;
	old_level = jack_plug_level;
	if(spk_hp == 1) 
	{
		if(jack_plug_level == 1) 
		{
			//now HeadPhone output,so clear SPK
			i2s_codec_write(0x02, 0x0020);
			i2s_codec_write(0x02, 0x0000);
			i2s_codec_write(0x00, 0x0040); 
		} 
		else 
		{
			//now Speaker output,so clear HP
			i2s_codec_write(0x01, 0x0039);
			i2s_codec_write(0x01, 0x0009);
			i2s_codec_write(0x00, 0x0040);
			i2s_codec_write(0x0e, 0x0000);
			i2s_codec_write(0x0f, 0x0008);
		}
	}
	spk_hp = 1;
	if(jack_plug_level == 1) 
	{
		//for HeadPhone output
		i2s_codec_write(0x00, 0x0060); //
		i2s_codec_write(0x0f, 0x0009); //5-10

		i2s_codec_write(0x00, 0x0064); //
		i2s_codec_write(0x09, 0x0091);// volume control 0dB
		i2s_codec_write(0x0c, 0x0091);// 0dB(right)
		//eq off
		i2s_codec_write(0x11, 0x0000);//5-10 
		i2s_codec_write(0x01, 0x0039); //

		i2s_codec_write(0x01, 0x0079); //
	} 
	else 
	{
		//for Speaker output
		i2s_codec_write(0x00, 0x0040);
		i2s_codec_write(0x02, 0x0020);

		i2s_codec_write(0x03, 0x0018);//5-10
		i2s_codec_write(0x06, 0x003c);

		i2s_codec_write(0x08, 0x00A1);//5-10

		i2s_codec_write(0x0b, 0x0040); //5-10

		i2s_codec_write(0x07, 0x002d); //5-10
		i2s_codec_write(0x09, 0x0091);
		i2s_codec_write(0x0c, 0x0091);
		//HP volume output value

		i2s_codec_write(0x0a, codec_volume);//5-10
		i2s_codec_write(0x0d, codec_volume);//5-10

		i2s_codec_write(0x00, 0x0074);
		i2s_codec_write(0x02, 0x00a0);	
	}
}

void set_ak4642en_record(void)
{
	abnormal_data_count = 0;
	i2s_codec_write(0x02, 0x0004);
	i2s_codec_write(0x03, 0x0038);// recording volume add
	i2s_codec_write(0x06, 0x0000);//for ALC short waiting time
	i2s_codec_write(0x08, 0x00e1);
	i2s_codec_write(0x0b, 0x0000);
	i2s_codec_write(0x07, 0x0021); // ALC on

	i2s_codec_write(0x10, 0x0000);//0x0001
	//i2s_codec_write(0x10, 0x0001);//0x0001
	i2s_codec_write(0x01, 0x0039); //for open pop noise
	i2s_codec_write(0x01, 0x0079);
	i2s_codec_write(0x00, 0x0065);   
	mdelay(300);
}

void clear_ak4642en_replay(void)
{
	//for poll
	old_level = 100;
	spk_hp = 0;
	if(jack_plug_level == 1) 
	{
		//for HeadPhone output
		i2s_codec_write(0x01, 0x0039); // for close pop noise
		mdelay(300);
		i2s_codec_write(0x01, 0x0009); //PLL on I2S 
		i2s_codec_write(0x07, 0x0001);
		i2s_codec_write(0x11, 0x0000);
		i2s_codec_write(0x00, 0x0040);
		i2s_codec_write(0x0f, 0x0008); // for open pop noise
	} 
	else 
	{
		//for Speaker output
		i2s_codec_write(0x02, 0x0020);
		i2s_codec_write(0x07, 0x0001);
		i2s_codec_write(0x11, 0x0000);
		i2s_codec_write(0x02, 0x0000);
		i2s_codec_write(0x00, 0x0040); // for close pop noise
	}
}

void clear_ak4642en_record(void)
{
	//for Mic input(Stereo)
	i2s_codec_write(0x02, 0x0001);
	i2s_codec_write(0x07, 0x0001);
	i2s_codec_write(0x11, 0x0000);
}

void each_time_init_ak4642en(void)
{
	__i2s_disable();
	__i2s_as_slave();
	__i2s_set_sample_size(16);
}

void set_ak4642en_speed(int rate)
{
	//codec work at frequency
	unsigned short speed = 0;
	unsigned short val = 0;
	switch (rate) 
	{
	case 8000:
		speed = 0x00;
		if(jack_plug_level == 1) //speaker
		{ 
			i2s_codec_write(0x16, 0x0000);
			i2s_codec_write(0x17, 0x0000);
			i2s_codec_write(0x18, 0x0000);
			i2s_codec_write(0x19, 0x0000);
			i2s_codec_write(0x1A, 0x0000);
			i2s_codec_write(0x1B, 0x0000);
			i2s_codec_write(0x1C, 0x0027);//800hz
			i2s_codec_write(0x1D, 0x0018);
			i2s_codec_write(0x1E, 0x00b2);
			i2s_codec_write(0x1F, 0x002f);
			i2s_codec_write(0x11, 0x0010); //eq on
		}
		break;
	case 12000:
		speed = 0x01;
		if(jack_plug_level == 1)
		{
			i2s_codec_write(0x16, 0x0000);
			i2s_codec_write(0x17, 0x0000);
			i2s_codec_write(0x18, 0x0000);
			i2s_codec_write(0x19, 0x0000);
			i2s_codec_write(0x1A, 0x0000);
			i2s_codec_write(0x1B, 0x0000);
			i2s_codec_write(0x1C, 0x0064);
			i2s_codec_write(0x1D, 0x001a);
			i2s_codec_write(0x1E, 0x0038);
			i2s_codec_write(0x1F, 0x002b);
			i2s_codec_write(0x11, 0x0010); //eq on
		}
		break;
	case 16000:
		speed = 0x02;
		if(jack_plug_level == 1)
		{	
			i2s_codec_write(0x16, 0x00af);
			i2s_codec_write(0x17, 0x0020);
			i2s_codec_write(0x18, 0x0043);
			i2s_codec_write(0x19, 0x001a);
			i2s_codec_write(0x1A, 0x00af);
			i2s_codec_write(0x1B, 0x0020);
			i2s_codec_write(0x1C, 0x00a0);
			i2s_codec_write(0x1D, 0x001b);
			i2s_codec_write(0x1E, 0x00c0);
			i2s_codec_write(0x1F, 0x0028);
			i2s_codec_write(0x11, 0x0018); //eq on
		}
		break;
	case 24000:
		speed = 0x03;
		if(jack_plug_level == 1)
		{
			i2s_codec_write(0x16, 0x0086);
			i2s_codec_write(0x17, 0x0015);
			i2s_codec_write(0x18, 0x005d);
			i2s_codec_write(0x19, 0x0006);
			i2s_codec_write(0x1A, 0x0086);
			i2s_codec_write(0x1B, 0x0015);
			i2s_codec_write(0x1C, 0x00f5);
			i2s_codec_write(0x1D, 0x001c);
			i2s_codec_write(0x1E, 0x0016);
			i2s_codec_write(0x1F, 0x0026);
			i2s_codec_write(0x11, 0x0018); //eq on
		}
		break;
	case 7350:
		speed = 0x04;
		if(jack_plug_level == 1)
		{
			i2s_codec_write(0x16, 0x0000);
			i2s_codec_write(0x17, 0x0000);
			i2s_codec_write(0x18, 0x0000);
			i2s_codec_write(0x19, 0x0000);
			i2s_codec_write(0x1A, 0x0000);
			i2s_codec_write(0x1B, 0x0000);
			i2s_codec_write(0x1C, 0x0027);
			i2s_codec_write(0x1D, 0x0018);
			i2s_codec_write(0x1E, 0x00b2);
			i2s_codec_write(0x1F, 0x002f);
			i2s_codec_write(0x11, 0x0010); //eq on
		}
		break;
	case 11025:
		speed = 0x05;
		if(jack_plug_level == 1)
		{
			i2s_codec_write(0x16, 0x0059);
			i2s_codec_write(0x17, 0x000d);
			i2s_codec_write(0x18, 0x00cb);
			i2s_codec_write(0x19, 0x0037);
			i2s_codec_write(0x1A, 0x0059);
			i2s_codec_write(0x1B, 0x000d);
			i2s_codec_write(0x1C, 0x0046);
			i2s_codec_write(0x1D, 0x001e);
			i2s_codec_write(0x1E, 0x0074);
			i2s_codec_write(0x1F, 0x0023);
			i2s_codec_write(0x11, 0x0018); //eq on
		}
		break;
	case 14700:
		speed = 0x06;
		if(jack_plug_level == 1)
		{
			i2s_codec_write(0x16, 0x0000);
			i2s_codec_write(0x17, 0x0000);
			i2s_codec_write(0x18, 0x0000);
			i2s_codec_write(0x19, 0x0000);
			i2s_codec_write(0x1A, 0x0000);
			i2s_codec_write(0x1B, 0x0000);
			i2s_codec_write(0x1C, 0x004a);
			i2s_codec_write(0x1D, 0x001b);
			i2s_codec_write(0x1E, 0x006c);
			i2s_codec_write(0x1F, 0x0029);
			i2s_codec_write(0x11, 0x0010); //eq on
		}
		break;
	case 22050:
		speed = 0x07;
		if(jack_plug_level == 1)
		{
			i2s_codec_write(0x16, 0x002d);
			i2s_codec_write(0x17, 0x0017);
			i2s_codec_write(0x18, 0x0050);
			i2s_codec_write(0x19, 0x0009);
			i2s_codec_write(0x1A, 0x002d);
			i2s_codec_write(0x1B, 0x0017);
			i2s_codec_write(0x1C, 0x00d7);
			i2s_codec_write(0x1D, 0x001c);
			i2s_codec_write(0x1E, 0x0093);
			i2s_codec_write(0x1F, 0x0026);
			i2s_codec_write(0x11, 0x0018); //eq on
		}
		break;
	case 32000:
		speed = 0x0a;
		if(jack_plug_level == 1)
		{
			i2s_codec_write(0x16, 0x0012);
			i2s_codec_write(0x17, 0x0011);
			i2s_codec_write(0x18, 0x006e);
			i2s_codec_write(0x19, 0x003e);
			i2s_codec_write(0x1A, 0x0012);
			i2s_codec_write(0x1B, 0x0011);
			i2s_codec_write(0x1C, 0x00aa);
			i2s_codec_write(0x1D, 0x001d);
			i2s_codec_write(0x1E, 0x00ab);
			i2s_codec_write(0x1F, 0x0024);
			i2s_codec_write(0x11, 0x0018); //eq on
		}
		break;
	case 48000:
		speed = 0x0b;
		if(jack_plug_level == 1)
		{
			i2s_codec_write(0x16, 0x0082);
			i2s_codec_write(0x17, 0x000c);
			i2s_codec_write(0x18, 0x004b);
			i2s_codec_write(0x19, 0x0036);
			i2s_codec_write(0x1A, 0x0082);
			i2s_codec_write(0x1B, 0x000c);
			i2s_codec_write(0x1C, 0x0068);
			i2s_codec_write(0x1D, 0x001e);
			i2s_codec_write(0x1E, 0x0030);
			i2s_codec_write(0x1F, 0x0023);
			i2s_codec_write(0x11, 0x0018); //eq on
		}
		break;
	case 29400:
		speed = 0x0e;
		if(jack_plug_level == 1)
		{
			i2s_codec_write(0x16, 0x003d);
			i2s_codec_write(0x17, 0x0012);
			i2s_codec_write(0x18, 0x0083);
			i2s_codec_write(0x19, 0x0000);
			i2s_codec_write(0x1A, 0x003d);
			i2s_codec_write(0x1B, 0x0012);
			i2s_codec_write(0x1C, 0x0079);
			i2s_codec_write(0x1D, 0x001d);
			i2s_codec_write(0x1E, 0x000d);
			i2s_codec_write(0x1F, 0x0025);
			i2s_codec_write(0x11, 0x0018); //eq on
		}
		break;
	case 44100:
		speed = 0x0f;
		if(jack_plug_level == 1)
		{
			i2s_codec_write(0x16, 0x0059);
			i2s_codec_write(0x17, 0x000d);
			i2s_codec_write(0x18, 0x00cb);
			i2s_codec_write(0x19, 0x0037);
			i2s_codec_write(0x1A, 0x0059);
			i2s_codec_write(0x1B, 0x000d);
			i2s_codec_write(0x1C, 0x0046);
			i2s_codec_write(0x1D, 0x001e);
			i2s_codec_write(0x1E, 0x0074);
			i2s_codec_write(0x1F, 0x0023);
			i2s_codec_write(0x11, 0x0018); //eq on
		}
		break;			
	default:
		break;
	}
	val = speed & 0x08;
	val = val << 2;
	speed = speed & 0x07;
	val = val | speed;
	i2s_codec_write(0x05, val);
}

void ak4642en_mixer_old_info_id_name(void)
{
	strncpy(info.id, "AK4642EN", sizeof(info.id));
	strncpy(info.name,"AKM AK4642en codec", sizeof(info.name));
}

void ak4642en_mixer_info_id_name(void)
{
	strncpy(old_info.id, "AK4642EN", sizeof(old_info.id));
	strncpy(old_info.name,"AKM AK4642en codec", sizeof(old_info.name));
}

void set_ak4642en_volume(int val)
{
	if ( val == 0 )
        	codec_volume = 255;
	else if ( val > 1 && val <= 10)
        	codec_volume = 92;
	else if ( val > 10 && val <= 20 )
        	codec_volume = 67;
	else if ( val > 20 && val <= 30 )
        	codec_volume = 50;
	else if ( val > 30 && val <= 40 )
        	codec_volume = 40;
	else if ( val > 40 && val <= 50 )
        	codec_volume = 30;
	else if ( val > 50 && val <= 60 )
        	codec_volume = 22;
	else if ( val > 60&& val <= 70 )
        	codec_volume = 15;
	else if ( val > 70 && val <= 80 )
        	codec_volume = 8;
	else if ( val > 80 && val <= 90 )
        	codec_volume = 4;
	else if ( val > 90 && val <= 100 )
        	codec_volume = 2;

        i2s_codec_write(0x0a, codec_volume);
        i2s_codec_write(0x0d, codec_volume);
}

void set_ak4642en_mic(int val)
{
	int mic_gain;
	mic_gain = 241 * val /100;
	i2s_codec_write(0x09, mic_gain);
	i2s_codec_write(0x0c, mic_gain);
}

void resume_ak4642en(void)
{
	__gpio_as_output(17);
	__gpio_set_pin(17); //enable ak4642
	__gpio_as_output(68);
	__gpio_clear_pin(68);
	__gpio_as_output(69);
	__gpio_clear_pin(69);
	__gpio_as_output(70);
	__gpio_clear_pin(70);
	__gpio_as_input(71);
	__gpio_clear_pin(71);
	__gpio_as_input(77);
	__gpio_clear_pin(77);
	__gpio_as_input(78);
	__gpio_clear_pin(78);
	REG_GPIO_GPALR(2) &= 0xC3FF0CFF;
	REG_GPIO_GPALR(2) |= 0x14005000;
	//set SCC clock initialization
	REG_SCC1_CR(SCC1_BASE) = 0x00000000;
	udelay(2);
	REG_SCC1_CR(SCC1_BASE) |= 1 << 31;
	udelay(2);
	__gpio_as_output(I2S_PDN);
	__gpio_set_pin(I2S_PDN);
	udelay(5);
	__gpio_clear_pin(I2S_PDN);
	ndelay(300);//>150ns
	__gpio_set_pin(I2S_PDN);
	mdelay(1);
	//set PLL Master mode
	i2s_codec_write(0x01, 0x0008);//master
	i2s_codec_write(0x04, 0x006b);//ref:12MHz;BITCLK:64fs;I2S compli
	i2s_codec_write(0x05, 0x000b);//sync:48KHz;
	i2s_codec_write(0x00, 0x0040);//PMVCM
	i2s_codec_write(0x01, 0x0009);//master,PLL enable
	jack_plug_level = 10;
	old_level = 100;
	spk_hp = 0;
	__gpio_as_input(JACK_PLUG_PIN);
	jack_plug_level = __gpio_get_pin(JACK_PLUG_PIN);
	//i suppose jack_plug_lvel is 1 indicate with HPO
	if(jack_plug_level > 1 || jack_plug_level <0)
		printk("Audio ak4642en codec Jack plug level is wrong!\n");
	if(jack_plug_level)
            	__gpio_as_irq_fall_edge(JACK_PLUG_PIN);
	else 
            	__gpio_as_irq_rise_edge(JACK_PLUG_PIN);

	i2s_codec_write(0x00, 0x0065); //for resume power
	i2s_codec_write(0x01, 0x0039); //for open pop noise
	i2s_codec_write(0x01, 0x0079);	
	i2s_codec_write(0x0a, codec_volume);
	i2s_codec_write(0x0d, codec_volume);
}

void suspend_ak4642en(int wr,int rd)
{
	if(wr)    //playing
	{
    	    	if(jack_plug_level == 0) 
	    	{
			i2s_codec_write(0x01, 0x0039); // for close pop noise
			mdelay(500);
			i2s_codec_write(0x01, 0x0009); //PLL on I2S 
			i2s_codec_write(0x07, 0x0001);
			i2s_codec_write(0x11, 0x0000);
			i2s_codec_write(0x00, 0x0040);
			i2s_codec_write(0x0f, 0x0008); // for open pop noise
			
	    	}
	   	else
	    	{
			//for Speaker output
			i2s_codec_write(0x02, 0x0020);
			i2s_codec_write(0x07, 0x0001);
			i2s_codec_write(0x11, 0x0000);
			i2s_codec_write(0x02, 0x0000);
			i2s_codec_write(0x00, 0x0040); // for close pop noise
	     	}
	}
            
	if(rd)   // recording
	{
		i2s_codec_write(0x02, 0x0001); // 5-11 a1
    		i2s_codec_write(0x07, 0x0001);
    		i2s_codec_write(0x11, 0x0000);
		mdelay(300);
	}
	__gpio_as_output(17);
	__gpio_clear_pin(17);//disable ak4642
	__i2s_disable();
}

static int __init init_ak4642en(void)
{
	set_codec_gpio_pin = set_ak4642en_gpio_pin;
	each_time_init_codec = each_time_init_ak4642en;
	clear_codec_mode = clear_ak4642en_mode;

	set_codec_record = set_ak4642en_record;
	set_codec_replay = set_ak4642en_replay;
	set_replay_hp_or_speaker = set_ak4642en_replay;

	set_codec_speed = set_ak4642en_speed;
	clear_codec_record = clear_ak4642en_record;
	clear_codec_replay = clear_ak4642en_replay;

	codec_mixer_old_info_id_name = ak4642en_mixer_old_info_id_name;
	codec_mixer_info_id_name = ak4642en_mixer_info_id_name;

	set_codec_volume = set_ak4642en_volume;

	set_codec_mic = set_ak4642en_mic;

	i2s_resume_codec = resume_ak4642en;
	i2s_suspend_codec = suspend_ak4642en;
	printk("---> ak4642en initialization!\n");
	return 0;
}

static void __exit cleanup_ak4642en(void)
{
	spk_hp = 0;
	i2s_codec_write(0x01, 0x0008);//master,PLL disable
	//free_irq(JACK_PLUG_IRQ, i2s_controller);
	__gpio_clear_pin(I2S_PDN);
	udelay(2);
	REG_SCC1_CR(SCC1_BASE) &= 0 << 31;
	udelay(2); 
}

module_init(init_ak4642en);
module_exit(cleanup_ak4642en);
