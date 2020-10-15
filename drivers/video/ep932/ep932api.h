#ifndef EP932_API_H
#define EP932_API_H

#include <linux/kernel.h>


#define DBG_PRINTK(x) printk x
//#define DBG_PRINTK(x) 

#ifndef min
#define min((a), (b))	((a) > (b)) ? (a) : (b)
#endif

typedef enum {
	// Master
	SMBUS_STATUS_SUCCESS = 0x00,
	SMBUS_STATUS_PENDING,//	SMBUS_STATUS_Abort,
	SMBUS_STATUS_NOACT = 0x02,
	SMBUS_STATUS_TIMEOUT,
	SMBUS_STATUS_ARBITRATIONLOSS = 0x04
}smbus_status;


typedef enum {
	AUD_SF_32000HZ = 1,
	AUD_SF_44100HZ,
	AUD_SF_48000HZ,
	AUD_SF_88200HZ,
	AUD_SF_96000HZ,
	AUD_SF_176400HZ,
	AUD_SF_192000HZ
} hdmi_audfreq;


typedef enum{
	AUDIOMUTE_DISABLE = 0,
	AUDIOMUTE_ENALBE = 1
} cstvout_audio_mode;
	

typedef enum {
	AUD_I2S = 0,
	AUD_SPDIF	
}hdmi_audfmt_t;

#define DSEL_DUAL_EDGE		0x08
#define BSEL_24BIT		0x04
#define EDGE_RISING		0x02
#define FMT_12			0x01



void ep_ep932m_reset(void);
void hdmi_main (void);
void ep_hdmi_setaudfmt(hdmi_audfmt_t  audfmt, hdmi_audfreq audfreq);
void ep_hdmi_set_video_timing(int timing);
unsigned char ep_hdmi_init(void); 

#endif
