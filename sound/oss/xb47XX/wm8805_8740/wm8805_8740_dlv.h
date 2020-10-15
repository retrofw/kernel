/*
 * Linux/sound/oss/jz4760_dlv.h
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#ifndef __WM8805_8740_DLV_H__
#define __WM8805_8740_DLV_H__

#include <linux/jz_audio.h>
//#include <linux/switch.h>

/* Enable headphone detection */
//#define HP_SENSE_DETECT		1

/* Power setting */
#define POWER_ON		0
#define POWER_OFF		1

/* Have not been used. */
#if 0

/* CODEC mode
 *
 * unsigned int:		I--- RL21 O--- -LSH
 */
#define CODEC_REPLAY		0x00008000
 #define REPLAY_HP_OUT		1 << 0
 #define REPLAY_SPEAKER		1 << 1
 #define REPLAY_LINE_OUT	1 << 2

#define CODEC_RECORD		0x80000000
 #define RECORD_MIC1_IN		1 << 8
 #define RECORD_MIC2_IN		1 << 9
 #define RECORD_LINE_IN_L	1 << 10
 #define RECORD_LINE_IN_R	1 << 11

#endif

/* File ops mode W & R */
#define REPLAY			1
#define RECORD			2

/* Channels */
#define LEFT_CHANNEL		1
#define RIGHT_CHANNEL		2

#define MAX_RATE_COUNT		10


/*
 * JZ4760 DLV CODEC registers
 */



/*
 * Ops
 */
/* misc ops*/

/*============================== ADC/DAC ==============================*/

/* adc/dac interface */



/*############################### misc start #################################*/

/*======================================================*/

#ifdef CONFIG_HP_SENSE_DETECT
/*
 * HP_SENSE switch
 */
typedef struct {
	struct switch_dev	sdev;
	const char		*name;
	const char		*name_on;
	const char		*name_off;
	const char		*state_on;
	const char		*state_off;
	
} jz_hp_switch_data_t ;

typedef struct {
	const char		*name;
	const char		*name_on;
	const char		*name_off;
	const char		*state_on;
	const char		*state_off;
	
} jz_hp_switch_platform_data_t ;

#endif //CONFIG_HP_SENSE_DETECT

/*======================================================*/

typedef struct {
	int dlv_sys_clk;
	int dlv_dmic_clk;
	int dlv_replay_volume_base;
	int dlv_record_volume_base;
	int dlv_record_digital_volume_base;
	int dlv_replay_digital_volume_base;
	int dlv_replay_hp_output_gain;
	unsigned int default_replay_route;
	unsigned int default_record_route;
	unsigned int default_call_record_route;
	int (*dlv_set_device)(struct snd_device_config *arg);
	int (*dlv_set_standby)(unsigned int sw);
        int (*dlv_set_gpio_before_set_route)(int route);
	int (*dlv_set_gpio_after_set_route)(int route);
	int (*dlv_init_part)(void);
	int (*dlv_turn_off_part)(int mode);
	int (*dlv_shutdown_part)(void);
	int (*dlv_reset_part)(void);
	int (*dlv_suspend_part)(void);
	int (*dlv_resume_part)(void);
	int (*dlv_anti_pop_part)(void);
	int (*dlv_board_mute)(int enable);
} jz_dlv_platform_data_t;

/*======================================================*/

int dlv_set_route(int route);
void dlv_sleep(int ms);

/*======================================================*/

/*############################### misc end ###################################*/

#endif // __WM8805_8740_DLV_H__

