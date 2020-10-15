#ifndef EP932_IF_H
#define EP932_IF_H

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/sched.h>

#include "ep932api.h"
#include "ep932regdef.h"


typedef enum {
	COLORSPACE_601 = 1,
	COLORSPACE_709
} colorspace_t;

typedef enum {
	SYNCMODE_HVDE = 0,
	SYNCMODE_HV,
	SYNCMODE_EMBEDED
} syncmode_t;

typedef enum {
	COLORFORMAT_RGB = 0,
	COLORFORMAT_YCC444,
	COLORFORMAT_YCC422
} colorformat_t;

typedef enum {
	AFAR_VIDEOCODE = 0,
	AFAR_4_3,
	AFAR_16_9,
	AFAR_14_9
} afar;

// video output congif params
typedef struct _vdo_params {
	unsigned char 			interface;			// DK[3:1], DKEN, DSEL, BSEL, EDGE, FMT12
	unsigned char 			videosettingindex;	// VIC
	unsigned char 			hvpol;				// x, x, x, x, VSO_POL, HSO_POL, x, x
	syncmode_t 		syncmode;			// 0 = HVDE, 1 = HV(DE Gen), 2 = Embedded Sync
	colorformat_t 	formatin;			// 0 = RGB, 1 = YCC444, 2 = YCC422
	colorformat_t 	formatout;			// 0 = RGB, 1 = YCC444, 2 = YCC422
// Which don't cause Timing Chage Reset
	colorspace_t 		colorspace;			// 0 = Auto, 1 = 601, 2 = 709
	afar 			afarate;			// 0 = Auto, 1 = 4:3, 2 = 16:9, 3 = 14:9
} vdo_params, *pvdo_params;

typedef enum {
	ADSFREQ_32000HZ = 0x03,
	ADSFREQ_44100HZ = 0x00,
	ADSFREQ_48000HZ = 0x02,
	ADSFREQ_88200HZ = 0x08,
	ADSFREQ_96000HZ = 0x0A,
	ADSFREQ_176400HZ = 0x0C,
	ADSFREQ_192000HZ = 0x0E
} adsfreq;

// Audio Output Congif Params
typedef struct _ado_params {
	unsigned char 		interface;				// x, x, x, x, IIS, WS_M, WS_POL, SCK_POL
	unsigned char 		videosettingindex;		// VIC
	unsigned char 		channelnumber;			// 1 = 2 ch, 2 = 3 ch, ... , 5 = 5.1 ch, 7 = 7.1 ch
	unsigned char 		adsrate;				// 1 = SF/2, 2 = SF/3, 3 = SF/4 (Down Sample)
	adsfreq			inputfrequency;			// ADSFREQ
	unsigned char       	vfs; 					// 0 = 59.94Hz, 1 = 60Hz (Vertical Frequency Shift of Video)
	unsigned char 		nocopyright;
} ado_params, *pado_params;

extern void ep932_if_initial(void);
extern void ep932_if_reset(void);



// Common
extern void hdmi_tx_power_down(void);
extern void hdmi_tx_power_up(void);
extern void hdmi_tx_hdmi(void);
extern void hdmi_tx_dvi(void);
extern unsigned char hdmi_tx_htplg(void);
extern unsigned char hdmi_tx_rsen(void);

// HDCP
extern void hdmi_tx_mute_enable(void);
extern void hdmi_tx_mute_disable(void);
extern void hdmi_tx_hdcp_enable(void);
extern void hdmi_tx_hdcp_disable(void);
extern void hdmi_tx_rptr_set(void);
extern void hdmi_tx_rptr_clear(void);
extern unsigned char hdmi_tx_ri_rdy(void);
extern void hdmi_tx_write_an(unsigned char *pan);
extern unsigned char hdmi_tx_aksv_rdy(void);
extern unsigned char hdmi_tx_read_aksv(unsigned char *paksv);
extern void hdmi_tx_write_bksv(unsigned char *pbksv);
extern unsigned char hdmi_tx_read_ri(unsigned char *pri);
extern void hdmi_tx_read_m0(unsigned char *pm0);
smbus_status hdmi_tx_get_key(unsigned char *key);

// special for EP932E
extern void hdmi_tx_amute_enable(void);
extern void hdmi_tx_amute_disable(void);
extern void hdmi_tx_vmute_enable(void);
extern void hdmi_tx_vmute_disable(void);
extern void hdmi_tx_video_config(pvdo_params params);
extern void hdmi_tx_audio_config(pado_params params);



// hardware interface

// EP932
smbus_status key_read(unsigned char byteaddr, void *data, unsigned int size);

smbus_status key_write(unsigned char byteaddr, void *data, unsigned int size);

smbus_status ep932_reg_read(unsigned char byteaddr, unsigned char *data, unsigned int size);
smbus_status ep932_reg_write(unsigned char byteaddr, unsigned char *data, unsigned int size);
smbus_status ep932_reg_set_bit(unsigned char byteaddr, unsigned char bitmask);
smbus_status ep932_reg_clear_bit(unsigned char byteaddr, unsigned char bitmask);


#endif // EP932_IF_H


