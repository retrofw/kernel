/*
 * Linux/sound/oss/jz4760_route_conf.h
 *
 * DLV CODEC driver for Ingenic Jz4760 MIPS processor
 *
 * 2010-11-xx   jbbi <jbbi@ingenic.cn>
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#ifndef __ROUTE_CONF_H__
#define __ROUTE_CONF_H__
typedef struct __route_conf_base {
	/*--------route-----------*/
	int route_ready_mode;
	//record//
	int route_mic1_mode;
	int route_mic2_mode;
	int route_linein_mode;
	int route_record_mux_mode;
	int route_adc_mode;
	int route_record_mixer_mode;
	//replay
	int route_replay_mixer_mode;
	int route_replay_filter_mode;
	int route_dac_mode;
	int route_replay_mux_mode;
	int route_hp_mode;
	int route_lineout_mode;
	int route_btl_mode;
	/*--------attibute-------*/
	int attibute_agc_mode;

	/* gain note: use 32 instead of 0 */
	int attibute_mic1_gain;		//val: 32(0), +4, +8, +16, +20 (dB); 
	int attibute_mic2_gain;		//val: 32(0), +4, +8, +16, +20 (dB);
	int attibute_linein_l_gain;	//val: +6 ~ +1, 32(0), -1 ~ -25 (dB);
	int attibute_linein_r_gain;	//val: +6 ~ +1, 32(0), -1 ~ -25 (dB);
	int attibute_adc_l_gain;	//val: 32(0), +1 ~ +23 (dB);
	int attibute_adc_r_gain;	//val: 32(0), +1 ~ +23 (dB);
	int attibute_record_mixer_gain;	//val: 32(0), -1 ~ -31 (dB);
	int attibute_replay_mixer_gain;	//val: 32(0), -1 ~ -31 (dB);
	int attibute_dac_l_gain;	//val: 32(0), -1 ~ -31 (dB);
	int attibute_dac_r_gain;	//val: 32(0), -1 ~ -31 (dB);
	int attibute_hp_l_gain;		//val: +6 ~ +1, 32(0), -1 ~ -25 (dB);
	int attibute_hp_r_gain;		//val: +6 ~ +1, 32(0), -1 ~ -25 (dB);
} route_conf_base;

struct __dlv_route_info {
	unsigned int route_name;
	route_conf_base const *route_conf;
};

/*================ route conf ===========================*/

#define DISABLE						99

/*-------------- route part selection -------------------*/

#define ROUTE_READY_FOR_ADC				1
#define ROUTE_READY_FOR_DAC				2
#define ROUTE_READY_FOR_ADC_DAC				3

#define MIC1_DIFF_WITH_MICBIAS				1
#define MIC1_DIFF_WITHOUT_MICBIAS			2
#define MIC1_SING_WITH_MICBIAS				3
#define MIC1_SING_WITHOUT_MICBIAS			4
#define MIC1_DISABLE					DISABLE

#define MIC2_DIFF_WITH_MICBIAS				1
#define MIC2_DIFF_WITHOUT_MICBIAS			2
#define MIC2_SING_WITH_MICBIAS				3
#define MIC2_SING_WITHOUT_MICBIAS			4
#define MIC2_DISABLE					DISABLE
		
#define LINEIN_WITHOUT_BYPASS				1
#define LINEIN_WITH_BYPASS				2
#define LINEIN_DISABLE					DISABLE

#define AGC_ENABLE					1
#define AGC_DISABLE					DISABLE

#define RECORD_MUX_MIC1_TO_LR				1
#define RECORD_MUX_MIC2_TO_LR				2
#define RECORD_MUX_MIC1_TO_R_MIC2_TO_L			3
#define RECORD_MUX_MIC2_TO_R_MIC1_TO_L			4
#define RECORD_MUX_LINE_IN				5

#define ADC_STEREO					1
#define ADC_MONO					2
#define ADC_DISABLE					DISABLE

#define RECORD_MIXER_MIX1_INPUT_ONLY			1
#define RECORD_MIXER_MIX1_INPUT_AND_DAC			2

#define REPLAY_MIXER_PLAYBACK_DAC_ONLY			1
#define REPLAY_MIXER_PLAYBACK_DAC_AND_ADC		2

#define DAC_STEREO					1
#define DAC_MONO					2
#define DAC_DISABLE					DISABLE

#define REPLAY_FILTER_STEREO				1
#define REPLAY_FILTER_MONO				2

#define REPLAY_MUX_MIC1_TO_LR				1
#define REPLAY_MUX_MIC2_TO_LR				2
#define REPLAY_MUX_MIC1_TO_R_MIC2_TO_L			3
#define REPLAY_MUX_MIC2_TO_R_MIC1_TO_L			4
#define REPLAY_MUX_BYPASS_PATH				5
#define REPLAY_MUX_DAC_OUTPUT				6

#define HP_ENABLE					1
#define HP_DISABLE					DISABLE

#define LINEOUT_STEREO					1
#define LINEOUT_MONO					2
#define LINEOUT_DISABLE					DISABLE

#define BTL_ENABLE					1
#define BTL_DISABLE					DISABLE

/***************************************************************************************\
 *                                                                                     *
 *route table                                                                          *
 *                                                                                     *
\***************************************************************************************/

enum dlv_route_t {
	ROUTE_ALL_CLEAR,
	ROUTE_REPLAY_CLEAR,
	ROUTE_RECORD_CLEAR,
	RECORD_MIC1_MONO_DIFF_WITH_BIAS,
	RECORD_MIC1_MONO_DIFF_WITHOUT_BIAS,
	RECORD_MIC2_MONO_DIFF_WITH_BIAS,
	RECORD_MIC2_MONO_DIFF_WITHOUT_BIAS,
	REPLAY_OUT,
	REPLAY_HP_STEREO,
	REPLAY_LINEOUT_MONO,
	REPLAY_BTL,
	BYPASS_MIC1_DIFF_WITH_BIAS_TO_OUT,
	BYPASS_MIC1_DIFF_WITH_BIAS_TO_HP,
	BYPASS_MIC1_DIFF_WITH_BIAS_TO_LINEOUT_MONO,
	BYPASS_MIC1_DIFF_WITH_BIAS_TO_BTL,
	BYPASS_MIC2_DIFF_WITH_BIAS_TO_OUT,
	BYPASS_MIC2_DIFF_WITH_BIAS_TO_HP,
	BYPASS_MIC2_DIFF_WITH_BIAS_TO_LINEOUT_MONO,
	BYPASS_MIC2_DIFF_WITH_BIAS_TO_BTL,
	BYPASS_LINEIN_TO_OUT,
	BYPASS_LINEIN_TO_HP,
	BYPASS_LINEIN_TO_LINEOUT_MONO,
	BYPASS_LINEIN_TO_BTL,
	RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_OUT,
	RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_HP,
	RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_LINEOUT_MONO,
	RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_BTL,
	RECORD_MIC2_MONO_DIFF_WITHOUT_BIAS_REPLAY_LINEOUT_MONO,
	ROUTE_COUNT
};

extern struct __dlv_route_info dlv_route_info[];

#endif
