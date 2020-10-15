/* header file for JZ CODEC */
#ifndef _JZ_CODEC_H_
#define _JZ_CODEC_H_

//-------------------------------------------------
#define CODEC_INIT			0
#define CODEC_SET_MODE			1
#define CODEC_SET_STARTUP_PARAM		5 
#define CODEC_SET_VOLUME_TABLE		6

#define CODEC_SET_RECORD		7
#define CODEC_CLEAR_RECORD		24

#define CODEC_SET_REPLAY		8
#define CODEC_CLEAR_REPLAY		25

#define CODEC_SET_REPLAY_RECORD		9

#define CODEC_SET_REPLAY_RATE		12
#define CODEC_SET_RECORD_RATE		37

#define CODEC_SET_REPLAY_CHANNEL	33
#define CODEC_SET_REPLAY_DATA_WIDTH	34

#define CODEC_SET_RECORD_CHANNEL	35
#define CODEC_SET_RECORD_DATA_WIDTH	36

#define CODEC_SET_GPIO_PIN		3
#define CODEC_SET_BASS			16
#define CODEC_SET_REPLAY_VOLUME		17
#define CODEC_SET_MIC_VOLUME		18
#define CODEC_SET_LINE			19
#define CODEC_SET_SOME_FUNC		23

#define CODEC_CLEAR_MODE		2

#define CODEC_EACH_TIME_INIT		4
#define CODEC_TURN_ON			10
#define CODEC_TURN_OFF			11

#define CODEC_RESET			13
#define CODEC_GET_MIXER_OLD_INFO	14
#define CODEC_GET_MIXER_INFO		15
#define CODEC_I2S_RESUME		20
#define CODEC_I2S_SUSPEND		21
#define CODEC_PIN_INIT			22

#define CODEC_SET_REPLAY_HP_OR_SPKR	26
#define CODEC_SET_DIRECT_MODE		27
#define CODEC_CLEAR_DIRECT_MODE		28
#define CODEC_SET_LINEIN2HP		29
#define CODEC_CLEAR_LINEIN2HP		30
#define CODEC_SET_STANDBY		47

#define CODEC_SET_LINEIN2BTL		39
#define CODEC_CLEAR_LINEIN2BTL		40

#define CODEC_SET_DEVICE		41
#define CODEC_MUTE_DEVICE		42
#define CODEC_SET_REPLAY_SPEED          43
#define CODEC_SET_RECORD_SPEED          44
#define CODEC_SET_MIC                   45
#define CODEC_SET_VOLUME                46
//------------------------------------------------

#define CODEC_ANTI_POP			31
#define CODEC_TURN_REPLAY		32

#define CODEC_DAC_MUTE			38

#define CODEC_SET_STANDBY		47
#define CODEC_SET_REC_2_DAC		48
#define CODEC_DEBUG_ROUTINE		49
#define CODEC_SET_SPEAKER_POWER		50

#define CODEC_FIRST_OUTPUT		66

//-------------------------------------------------

//#define CODEC_DEBUG			100

//-------------------------------------------------

void register_jz_codecs(void *func);
void dump_dlv_regs(const char* str);
//void dlv_write_reg(int addr, int val);
//-------------------------------------------------

#define SND_SET_STANDBY _IOW(SND_IOCTL_MAGIC, 4, unsigned int *)

//-------------------------------------------------

// Deprecate !!!
#define USE_NONE		1
#define USE_MIC			2
#define USE_LINEIN		3

// For file ops
#define CODEC_WMODE		(1 << 0)
#define CODEC_RMODE		(1 << 1)
#define CODEC_WRMODE		(CODEC_WMODE | CODEC_RMODE)

#endif /* _JZ_CODEC_H_ */
