/* header file for JZ CODEC */
/* here is not very good, because some header file are same, you can change it  */
#if defined(CONFIG_SOC_JZ4760) || defined(CONFIG_SOC_JZ4760B)

#ifdef CONFIG_I2S_DLV_4760
#include "./xb4760/jz4760_dlv.h"
#include "./xb4760/jz4760_route_conf.h"

#else
#ifdef CONFIG_I2S_DLV_TAS5713
#include "./tas5713/tas5713_dlv.h"
#include "./tas5713/tas5713_route_conf.h"

#elif defined(CONFIG_I2S_DLV_TAS5707)
#include "./tas5707/tas5707_dlv.h"
#include "./tas5707/tas5707_route_conf.h"

#elif defined(CONFIG_I2S_DLV_NPCA110P)
#include "./npca110p/npca110p_dlv.h"
#include "./npca110p/npca110p_route_conf.h"

#elif defined(CONFIG_I2S_DLV_ES9023)
#include "./es9023/es9023_dlv.h"
#include "./es9023/es9023_route_conf.h"

#elif defined(CONFIG_I2S_DLV_WM8805_8740)
#include "./wm8805_8740/wm8805_8740_dlv.h"
#include "./wm8805_8740/wm8740_route_conf.h"
#endif
#endif

#elif defined(CONFIG_SOC_JZ4770)
#include "./xb4770/jz4770_dlv.h"
#include "./xb4770/jz4770_route_conf.h"
#endif

#ifndef _JZ_CODEC_H_
#define _JZ_CODEC_H_

enum codec_ioctl_cmd_t {
	CODEC_INIT,
	CODEC_TURN_OFF,
	CODEC_SHUTDOWN,
	CODEC_RESET,
	CODEC_SUSPEND,
	CODEC_RESUME,
	CODEC_ANTI_POP,
	CODEC_SET_ROUTE,
	CODEC_SET_DEVICE,
	CODEC_SET_STANDBY,
	CODEC_SET_RECORD_RATE,
	CODEC_SET_RECORD_DATA_WIDTH,
	CODEC_SET_MIC_VOLUME,
	CODEC_SET_RECORD_CHANNEL,
	CODEC_SET_REPLAY_RATE,
	CODEC_SET_REPLAY_DATA_WIDTH,
	CODEC_SET_REPLAY_VOLUME,
	CODEC_SET_REPLAY_CHANNEL,
	CODEC_SET_ADC_LRSWAP,
	CODEC_CLEAR_ADC_LRSWAP,
	CODEC_SET_DAC_LRSWAP,
	CODEC_CLEAR_DAC_LRSWAP,
	CODEC_DAC_MUTE,
	CODEC_BSP_MUTE,
	CODEC_DUMP_REGS,
	CODEC_DEBUG_ROUTINE
};

enum device_t {
	SND_DEVICE_DEFAULT = 0,
	SND_DEVICE_CURRENT,
	SND_DEVICE_HANDSET,
	SND_DEVICE_HEADSET,
	SND_DEVICE_SPEAKER,
	SND_DEVICE_BT,
	SND_DEVICE_BT_EC_OFF,
	SND_DEVICE_HEADSET_AND_SPEAKER,
	SND_DEVICE_TTY_FULL,
	SND_DEVICE_CARKIT,
	SND_DEVICE_FM_SPEAKER,
	SND_DEVICE_FM_HEADSET,
	SND_DEVICE_NO_MIC_HEADSET,
	SND_DEVICE_HDMI,
        SND_DEVICE_LINEIN,
	SND_DEVICE_COUNT
};
struct i2s_codec
{
        /* I2S controller connected with */
        void    *private_data;
        void    *codec_private;
        int     name;
        int     id;
        int     dev_mixer;

        int     use_mic_line_flag;
        int     audio_volume;
        int     mic_gain;
        int     bass_gain;

        unsigned int  record_audio_rate;
        unsigned int  replay_audio_rate;

        short   replay_codec_channel;
        short   record_codec_channel;

        short   replay_format;
        short   record_format;

        int     audiomute;
        int     user_need_mono;

        struct semaphore i2s_sem;
        int (*codecs_ioctrl)(void *context, unsigned int cmd, unsigned long arg);
};

void register_jz_codecs(void *func);
void register_jz_codecs_ex(void *func,void *codec);
void dump_dlv_regs(void);

#define SND_SET_STANDBY _IOW(SND_IOCTL_MAGIC, 4, unsigned int *)

// Deprecate !!!
#define USE_NONE		1
#define USE_MIC			2
#define USE_LINEIN		3

// For file ops
#define CODEC_WMODE		(1 << 0)
#define CODEC_RMODE		(1 << 1)
#define CODEC_WRMODE		(CODEC_WMODE | CODEC_RMODE)

#define SYS_CLK_12M             0
#define SYS_CLK_13M             1

#define DMIC_CLK_ON             1
#define DMIC_CLK_OFF            0

#ifdef CONFIG_I2S_DLV_NPCA110P
#define INIT_VOLUME             100
#else
#define INIT_VOLUME             20
#endif

#endif /* _JZ_CODEC_H_ */
