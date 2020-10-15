/******************************************************************************\

          (c) Copyright Explore Semiconductor, Inc. Limited 2005
                           ALL RIGHTS RESERVED 

--------------------------------------------------------------------------------

 Please review the terms of the license agreement before using this file.
 If you are not an authorized user, please destroy this source code file  
 and notify Explore Semiconductor Inc. immediately that you inadvertently 
 received an unauthorized copy.  

--------------------------------------------------------------------------------

  File        :  EP932SettingsData.h

  Description :  Head file of EP932SettingsData.

\******************************************************************************/

#ifndef EP932SETTINGDATA_H
#define EP932SETTINGDATA_H

// -----------------------------------------------------------------------------

#define EP932_VDO_SETTINGS_IT_START 76

// Definition of H/V Polarity
#define  VPOSHPOS               0x00
#define  VPOSHNEG               0x04
#define  VNEGHPOS               0x08
#define  VNEGHNEG               0x0C

// Pixel Freq Type
typedef enum {
	PIX_FREQ_25175KHZ = 0,
	PIX_FREQ_25200KHZ,

	PIX_FREQ_27000KHZ,
	PIX_FREQ_27027KHZ,

	PIX_FREQ_54000KHZ,
	PIX_FREQ_54054KHZ,

	PIX_FREQ_72000KHZ,

	PIX_FREQ_74176KHZ,
	PIX_FREQ_74250KHZ,

	PIX_FREQ_108000KHZ,
	PIX_FREQ_108108KHZ,

	PIX_FREQ_148352KHZ,
	PIX_FREQ_148500KHZ,

	PIX_FREQ_PC

} pix_freq_type;

//
// Index = [Video Code]
//
typedef struct _hvres_type {
   unsigned char  hvpol;
   unsigned short hres;
   unsigned short vres;
   unsigned short vprd;
} hvres_type_t, *phvres_type;

// DE Generation
typedef struct _de_gen_settings { // VideoCode to
	unsigned short de_dly;
	unsigned short de_cnt;
	unsigned char  de_top;
	unsigned short de_lin;
} de_gen_settings, *pde_gen_settings;

// Embeded Sybc
typedef struct _e_sync_settings { // VideoCode to
	unsigned char  ctl;
	unsigned short h_dly;
	unsigned short h_width;
	unsigned char  v_dly;
	unsigned char  v_width;
	unsigned short v_ofst;
} e_sync_settings, *pe_sync_settings;

// AVI Settings
typedef struct _vdo_settings { 
	unsigned char	videocode;
	hvres_type_t    hvres_type;
	de_gen_settings de_gen;
	e_sync_settings e_sync; // (HV_Gen)
	unsigned char   ar_pr;
	pix_freq_type   pix_freq_type;
} vdo_settings, *pvdo_settings;

extern vdo_settings ep932_vdo_settings[];
extern unsigned char ep932_vdo_settings_max; // = (sizeof(EP932_VDO_Settings)/sizeof(EP932_VDO_Settings[0]))

// -----------------------------------------------------------------------------

//
// Index = [Channel Number]
//
// Audio Channel and Allocation
typedef struct _ado_settings { // IIS ChannelNumber to
	unsigned char speakermapping;
	unsigned char flat;
} ado_settings, *pado_settings;

extern ado_settings ep932_ado_settings[];

// -----------------------------------------------------------------------------

//
// Index = [Pixel Freq Type]
//
// N and CTS
typedef struct _n_cts_setttings{ // IIS ChannelNumber to
	unsigned long n;
	unsigned long cts; // Use hardware to calculate the CTS
} n_cts_settings, *pn_cts_settings;

extern n_cts_settings n_cts_32k[];
extern n_cts_settings n_cts_44k1[];
extern n_cts_settings n_cts_48k[];


#endif
