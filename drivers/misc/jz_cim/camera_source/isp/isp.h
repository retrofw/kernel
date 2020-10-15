#ifndef _COREISP3_H_
#define _COREISP3_H_

#ifdef __cplusplus
//extern "C" {
#endif

typedef unsigned long long	ClUint_64;
typedef unsigned int		ClUint_32;
typedef unsigned short		ClUint_16;
typedef unsigned char		ClUint_8;
typedef long				ClSint_64;
typedef int					ClSint_32;
typedef short				ClSint_16;
typedef char				ClSint_8;
typedef char				Cl_Char;
typedef ClUint_32			Cl_Handle;
typedef ClUint_8			Cl_Bool;

#define Cl_Null      (0)
#define Cl_True      (1)
#define Cl_False     (0)

//#define ISP2_EVB_ENABLE
//#define ISP3_EVB_ENABLE

//#define ISP3_MCU_ARRAY  // Added by Jacky 
//
#define ISP3_MCU_LSC  // Jacky for LSC table in MCU 
//#define YUVDATA_DUMMY  // Jacky for test

//#define API_SCALE_PREVIEW  // Jacky for under 1.3M  preview

// Please select one sensor
#define MICRON_MI5130_SUNNY	// Sensor define	5M sensor
//#define MICRON_MI5130_MCNEX	// Sensor define       5M sensor
//#define SAMSUNG_3E2_MCNEX	// Sensor define  		5M sensor
//#define OV5630_TRULY	// Sensor define			5M sensor
//#define OV2M_TRULY	// Sensor define
//#define TRULY_2M_MODULE	// Sensor define
//#define HNT_5M_MODULE	// Sensor define  for SAMSUNG_3E2  HNT module
//#define SAMSUNG_3E2_LITE_ON	// Sensor define  		5M sensor
//#define HNT_2M_MODULE	// Sensor  SAMSUNG  HNT module

#ifdef SAMSUNG_3E2_LITE_ON
//#define EEPROM_ACCESS_ENABLE
#endif

#ifdef MICRON_MI5130_MCNEX
//#define ISP3_MCU_LSC  // Added by Jacky for test
//#define SENSOR_180_ROTATE  // Added by Jacky for test
//#define HISENSE_ZOOM_TEST
#define OPPO_FEATURES
#else
//#define MEIZU_FEATURES   
#endif

#define ISP3_COMMAND     0xe660

#define __ISP3_R__

#ifndef ISP3_EVB_ENABLE
#define DM_JPEG   1
#define  DebugMessage   // to your system print function
#define WaitTime_us  mdelay
#endif

#define IIC_BURST_MODE   // Jacky add this

#define ISP3_ZOOM_ENABLE
//#define DEFINE_FLOAT

#define	IICID_A		0x82
#define	IICID_B		0x80

#define	ISP_IICID_A		0x41<<1
#define	ISP_IICID_B		0x40<<1

#ifdef ISP3_ZOOM_ENABLE
#define ZOOMRATE_1X     1.0f
#define ZOOMRATE_2X     2.0f
#define ZOOMRATE_4X     4.0f
#define ZOOM_STEP_WIDTH 32      //  Minimum step size of Zoom Setting is width 32, height 24

typedef enum
{
//    enISP_ZOOM_RATE_0 = 0x00,      // 0
//    enISP_ZOOM_RATE_0_1,                     // 0.1
//    enISP_ZOOM_RATE_0_5,                     // 0.5
    enISP_ZOOM_RATE_1,                     // 1
    enISP_ZOOM_RATE_1_25,                    // 1.25
    enISP_ZOOM_RATE_1_5,                      // 1.5
    enISP_ZOOM_RATE_1_75,                     // 1.75
    enISP_ZOOM_RATE_2,                      // 2
    enISP_ZOOM_RATE_2_5                    // 2.5
}enIsp3ZoomRate;

typedef enum   // YuLong used code now
{
	CAM_ZOOM_100X=0,
	CAM_ZOOM_125X,
	CAM_ZOOM_150X,
	CAM_ZOOM_200X,
	CAM_ZOOM_250X,
	CAM_ZOOM_400X,
	CAM_ZOOM_CIF,    // 352x288   1.222
	CAM_ZOOM_QCIF, // 176x144
	CAM_ZOOM_480_360, // 480_360
	CAM_ZOOM_480_360X1, //
	CAM_ZOOM_480_360X2, //
	CAM_ZOOM_480_360X3, // 
	CAM_ZOOM_480_360X4, // 
	CAM_ZOOM_VGA,    // 640x480
	CAM_ZOOM_SVGA, // 800x600
	CAM_ZOOM_1_3M,  // 1280x960

//Following for OPPO 5:3 resolution
	CAM_ZOOM_800_480,
	CAM_ZOOM_1600_960,
	CAM_ZOOM_2048_1228,
	CAM_ZOOM_2560_1536,
//end 
	CAM_ZOOM_VP
}CAM_ZOOM_T;
#endif

// added by Jacky
typedef enum
{
	QSXGA						= 0x00,		// 2560x1920
	QXGA  						= 0x01,		// 2048x1536
	UXGA  						= 0x02,		// 1600x1200
	SXGA					,		// 1280x960
	XGA     					,		// 1024x768
	SVGA  					,		// 800x600
	VGA         				,		// 640x480
	QVGA   					,		// 320x240
	QQVGA  					,		// 160x120
	QSVGA  							// 400x300
	// Jacky add following code for test
	,HVGA_100  				
	,HVGA_125  						
	,HVGA_150  						
	,HVGA_175  						
	,HVGA_200  						
	//WSXGA				,		// 1280x1024
	,WQXGA						= 0x0A		// 2560x1536  		//	  5:3 ид  1280 * 960 ->800 * 480  ;  4:3 ид 1280*768->640 * 480 ;
} Resol_MODE;
typedef enum
{
    enISP_JPEG_QF_Fine = 18,   // 18
    enISP_JPEG_QF_High = 20,  // 20
    enISP_JPEG_QF_Middle = 25,// 25
    enISP_JPEG_QF_Low=28,   // 28
}enIsp3JPEGQF;

 // Added by Jacky for resolution, delete useless value
typedef enum
{
    enISP_RES_5MP_FULL = 0x00,      // 2560 x 1920
	enISP_RES_QXGA,						// 2048	x	1536
	enISP_RES_UXGA,						// 1600	x	1200
	enISP_RES_1_3MP,					// 1280	x	960
	enISP_RES_XGA,						// 1024	x	768
	enISP_RES_SVGA,						// 800	x	600
	enISP_RES_VGA,						// 640	x	480
	enISP_RES_QVGA,						// 320	x	240
	
	enISP_RES_SXGA,						// 1280	x	1024
	enISP_RES_QQVGA,						// 160	x	120
	enISP_RES_HQVGA,						// 240x160
 	enISP_RES_HVGA,						// 480x320
 	enISP_RES_480_360,						// 480x360
 	enISP_RES_QSVGA,						// 400x300
 	// For OPPO 5:3 resolution
	enISP_RES_800_480,
	enISP_RES_1600_960,
	enISP_RES_2048_1228,
	enISP_RES_2560_1536,
 	enISP_RES_480_320,						// 480x360
    enISP_RES_None
}enIsp3OutResolution;


typedef enum
{
    enISP_FUNC_AWB_OFF = 0x00,
    enISP_FUNC_AWB_ON 
}enIsp3FunctionsAWB;
typedef enum
{
    enISP_FUNC_AE_OFF = 0x00,
    enISP_FUNC_AE_ON 
}enIsp3FunctionsAE;


typedef enum
{
    enISP_FUNC_AF_OFF = 0x00,
    enISP_FUNC_AF_ON 
}enIsp3FunctionsAF;

typedef enum
{
    enISP_FUNC_ANR_LEVEL_0 = 0x00,
    enISP_FUNC_ANR_LEVEL_1,         
    enISP_FUNC_ANR_LEVEL_2,
    enISP_FUNC_ANR_LEVEL_3,
    enISP_FUNC_ANR_LEVEL_4
}enISP3FunctionsANRLevel;

typedef enum

{
    enISP_FUNC_WDR_LEVEL_0 = 0x00,
    enISP_FUNC_WDR_LEVEL_1,
    enISP_FUNC_WDR_LEVEL_2,
    enISP_FUNC_WDR_LEVEL_3,
    enISP_FUNC_WDR_LEVEL_4,
    enISP_FUNC_WDR_LEVEL_5,
    enISP_FUNC_WDR_LEVEL_6,
    enISP_FUNC_WDR_LEVEL_7
}enISP3FucntionsWDRLevel;

typedef struct
{
    ClUint_8 bWDREnable;
    enISP3FucntionsWDRLevel WDRlevel;
}_tISP_WDR_CTRL;

typedef struct
{
    ClUint_16 addr;
    ClUint_16   val;
}_tISP_IIC_CTRL;

typedef enum
{
    enISP_FUNC_FD_ROI_THICK_0 = 0x00,
    enISP_FUNC_FD_ROI_THICK_1,
    enISP_FUNC_FD_ROI_THICK_2,
    enISP_FUNC_FD_ROI_THICK_3,
    enISP_FUNC_FD_ROI_THICK_4,
    enISP_FUNC_FD_ROI_THICK_5,
    enISP_FUNC_FD_ROI_THICK_6,
    enISP_FUNC_FD_ROI_THICK_7,
    enISP_FUNC_FD_ROI_THICK_8,
    enISP_FUNC_FD_ROI_THICK_9,
    enISP_FUNC_FD_ROI_THICK_10,
    enISP_FUNC_FD_ROI_THICK_11,
    enISP_FUNC_FD_ROI_THICK_12,
    enISP_FUNC_FD_ROI_THICK_13,
    enISP_FUNC_FD_ROI_THICK_14,
    enISP_FUNC_FD_ROI_THICK_15
}enIsp3FaceROIThick;

typedef enum
{
    enISP_FUNC_FD_MAXCOUNT_1 = 0x00,
    enISP_FUNC_FD_MAXCOUNT_2,
    enISP_FUNC_FD_MAXCOUNT_3,
    enISP_FUNC_FD_MAXCOUNT_4,
    enISP_FUNC_FD_MAXCOUNT_5,    
    enISP_FUNC_FD_MAXCOUNT_6,
    enISP_FUNC_FD_MAXCOUNT_7,
    enISP_FUNC_FD_MAXCOUNT_8,    
    enISP_FUNC_FD_MAXCOUNT_9,    
    enISP_FUNC_FD_MAXCOUNT_10    
}enIsp3FaceMaxDetectionCount;

typedef struct
{
    ClUint_8 bFaceDetectionEnable;
    ClUint_8 bFaceTrackingEnable;
    ClUint_8 bFaceAE;
    ClUint_8 bFaceAF;
    ClUint_8 bFaceAWB;
    ClUint_8 bFaceRotation;
	ClUint_8 bFaceUserMode;
    enIsp3FaceROIThick Isp3FaceROIThick;
    enIsp3FaceMaxDetectionCount Isp3FaceMaxDetectionCount;
    enIsp3OutResolution OutputResolution;
}_tIspFaceDetectionCtrl;

typedef struct
{
	int sx;
	int sy;
	int ex;
	int ey;
	ClUint_8 first_pos_index;
	ClUint_8 second_pos_index;
}faceInfo;
extern faceInfo _faceInfo[10];

typedef enum
{
    enDownloadMode_StackedMem,
    enDownloadMode_SkipedMCUBin,
    enDownloadMode_CodeRAM,
    enDownloadMode_None
}enIsp3DownloadMode;

typedef enum
{
	CMD_Capture,
	CMD_Preview,
} CMD_Mode;   // update

typedef enum
{
	FLICKER_50HZ_OFF	= 0,
	FLICKER_50HZ_ON,
	FLICKER_60HZ_OFF,
	FLICKER_60HZ_ON,
	FLICKER_AUTO_OFF,
	FLICKER_AUTO_ON,
	
} FLICKER_TYPE;  // update

//auto
//sunny
//cloudy
//night
//flocsnet
//incandscent

typedef enum
{
	WB_AUTO					= 0x00,		//Auto White Balance mode
	WB_CLOUDY 				= 0x01,		//6000K
	WB_SUNNY					= 0x02,		//5200K
	WB_FLOCSNET				= 0x03,		//4600K    flocesnet
	WB_TUNGSTEN				= 0x04,		//3500K		
	WB_SUNSET				= 0x05,		//2000K~3000K
	WB_INCANDESCENT			= 0x06,		//2300K
	WB_CANDLE				= 0x07		//1500K~2000K
} enWB_MANUAL_TYPE;  // update

typedef enum
{
    enISP_Level_0    = 0x00,   	// -5
    enISP_Level_1,			// -4
    enISP_Level_2,			// -3
    enISP_Level_3,			// -2
    enISP_Level_4,			// -1
    enISP_Level_5,			// 0    
    enISP_Level_6,			// +1
    enISP_Level_7,			// +2
    enISP_Level_8,    			// +3
    enISP_Level_9,        		// +4
    enISP_Level_10,       		// +5
    enISP_Level_Max,        
    enISP_Level_Default,        
    enISP_Level_Off        		
}enIsp3Level_Value;			  // update by Jacky

typedef enum
{
    enISP_CrYCb    = 0x08,   	
    enISP_YCrCb    = 0x09,     	
    enISP_CbYCr    = 0x0a,   	
    enISP_YCbCr    = 0x0b,    	
}enIsp3YUV_Swap;			  // update by Jacky

typedef enum
{
	ISP3_ROTATION_NONE		= 0x00,
	ISP3_ROTATION_FLIP		= 0x01,
	ISP3_ROTATION_MIRROR	= 0x02,
	ISP3_ROTATION_MIRRORFLIP	= 0x03,
} enIsp3MIRROR_MODE;		  // update by Jacky

typedef enum
{
    enISP_FUNC_SS_LEVEL_NONE = 0x00,
    enISP_FUNC_SS_LEVEL_1,
    enISP_FUNC_SS_LEVEL_1_2,
    enISP_FUNC_SS_LEVEL_1_3,
    enISP_FUNC_SS_LEVEL_1_4,
    enISP_FUNC_SS_LEVEL_1_5,
    enISP_FUNC_SS_LEVEL_1_6
}enISP3FunctionsSSLevel;

typedef struct
{
    ClUint_8 bStillStabilizerEnable;
    enISP3FunctionsSSLevel StillStabilzerLevel;
}_tIspStillStabilizerCtrl;

typedef enum
{
    enISP_FUNC_IMAGE_NORMAL = 0x00,
    enISP_FUNC_IMAGE_EMBOSS,
    enISP_FUNC_IMAGE_SKETCH1,
    enISP_FUNC_IMAGE_SKETCH2,
    enISP_FUNC_IMAGE_BLACK_WHITE,
    enISP_FUNC_IMAGE_NORMAL_MOVIE,
    enISP_FUNC_IMAGE_OLD_MOVIE,
    enISP_FUNC_IMAGE_GRAY,
    enISP_FUNC_IMAGE_ACCENT,
    enISP_FUNC_IMAGE_SWAPING,
    enISP_FUNC_IMAGE_ACCENT_SWAPING,
    enISP_FUNC_IMAGE_WARM,
    enISP_FUNC_IMAGE_COOL,
    enISP_FUNC_IMAGE_FOG,
    enISP_FUNC_IMAGE_OPPOSITE_NEGATIVE,
    enISP_FUNC_IMAGE_OPPOSITE_AVERAGE
}enISPFunctionsImageEffect;

typedef enum
{
    enISP_FUNC_AEwithFACE_1 = 0,		//3 AE OFF
    enISP_FUNC_AEwithFACE_2,			//3 AE ON & Global
    enISP_FUNC_AEwithFACE_3			//3 AE ON & MULTI
}enISPFunctionsAEwithFaceApp;

typedef enum
{
    enISP_FUNC_AWBwithFD_Level_0    = 0x00,
    enISP_FUNC_AWBwithFD_Level_1,
    enISP_FUNC_AWBwithFD_Level_2,
    enISP_FUNC_AWBwithFD_Level_3,
    enISP_FUNC_AWBwithFD_Level_4,
    enISP_FUNC_AWBwithFD_Level_5,    
    enISP_FUNC_AWBwithFD_Level_6,
    enISP_FUNC_AWBwithFD_Level_7,
    enISP_FUNC_AWBwithFD_Level_8,    
    enISP_FUNC_AWBwithFD_Level_9,    
    enISP_FUNC_AWBwithFD_Level_10    
}enIsp3AWBwithFaceLevel;

typedef struct
{
	ClUint_32 sx;
	ClUint_32 sy;
	ClUint_32 ex;
	ClUint_32 ey;
	ClUint_8 first_pos_index;
	ClUint_8 second_pos_index;
	
}stFaceInfo;

typedef enum
{
    enISP_FUNC_FDUser_Level_0    = 0x00,
    enISP_FUNC_FDUser_Level_1,
    enISP_FUNC_FDUser_Level_2,
    enISP_FUNC_FDUser_Level_3,
    enISP_FUNC_FDUser_Level_4,
    enISP_FUNC_FDUser_Level_5,    
    enISP_FUNC_FDUser_Level_6,
    enISP_FUNC_FDUser_Level_7,
    enISP_FUNC_FDUser_Level_8,    
    enISP_FUNC_FDUser_Level_9    
}enIspFaceUserMode;

typedef enum
{
    ISP3_CMD_DEBUG_DEFAULT	= 0x01,
    ISP3_CMD_DEBUG_AE		= 0x02,
    ISP3_CMD_DEBUG_AF		= 0x03,
    ISP3_CMD_DEBUG_AWB		= 0x04,
    ISP3_CMD_DEBUG_FACE		= 0x05,
    ISP3_CMD_DEBUG_STILL	      = 0x06,
    ISP3_CMD_DEBUG_FLICKER	= 0x07,
    ISP3_CMD_DEBUG_ATC		= 0x08,
    ISP3_CMD_DEBUG_ANR		= 0x09,
    ISP3_CMD_DEBUG_EFFECT	= 0x0A,
    ISP3_CMD_DEBUG_BESTEXIF	= 0x0C,
    ISP3_CMD_DEBUG_FRMSG	= 0x0D,
    ISP3_CMD_BAUDRATE		= 0x0E,

    // MCU function        
    ISP3_CMD_ISR_ON		= 0x11,
    ISP3_CMD_ISR_OFF		= 0x12,
    ISP3_CMD_FRAME_COUNT_CLEAR	= 0x13,

    ISP3_CMD_GET_VERSION	= 0x14,
    // Camera secene mode       
    ISP3_CMD_MODE_AUTOMODE	= 0x20,
    ISP3_CMD_MODE_PORTRAIT	= 0x21,
    ISP3_CMD_MODE_LANDSCAPE	= 0x22,
    ISP3_CMD_MODE_INDOOR	= 0x23,
    ISP3_CMD_MODE_SPORTS	= 0x24,
    ISP3_CMD_MODE_NIGHT		= 0x25,
    ISP3_CMD_MODE_CANDLE	= 0x26,
    ISP3_CMD_MODE_FIREWORKS	= 0x27,
    ISP3_CMD_MODE_SNOW		= 0x28,
    ISP3_CMD_MODE_QUATREFOIL	= 0x29,

    // LSC         
    ISP3_CMD_LSC_DOWN		= 0x31,
    ISP3_CMD_LSC_DEBUG		= 0x32,

    // JPEG ZOOM        
    ISP3_CMD_JPEGZOOM_ON	= 0x41,
    ISP3_CMD_JPEGZOOM_OFF	= 0x42,

    //BEST EXIF        
    ISP3_CMD_BESTONE		= 0x59,
    ISP3_CMD_BESTTHR		= 0x5a,		
    ISP3_CMD_BESTFIV		= 0x5b,
    ISP3_CMD_BESTSEV		= 0x5c,
    ISP3_CMD_BESTNIN		= 0x5d,
    ISP3_CMD_BESTOFF 		= 0x5e,

    //Senser Reg R/W        
    ISP3_CMD_SenRegRead		= 0x62,
    ISP3_CMD_SenRegWrit		= 0x63,

    ISP3_CMD_MCUExifON		= 0x60,
    ISP3_CMD_MCUExifOFF		= 0x61,

    // IMAGE EFFECT        
    ISP3_CMD_EFFECT_DEFAULT	= 0x70,
    ISP3_CMD_EFFECT_SPLENDID	= 0x71,
    ISP3_CMD_EFFECT_CHARISMA	= 0x72,
    ISP3_CMD_EFFECT_DIGNITY	= 0x73,
    ISP3_CMD_EFFECT_HORROR	= 0x74,

    // ISO FLAG NUMBERING       
    ISP3_CMD_ASA100		= 0x80,
    ISP3_CMD_ASA200		= 0x81,
    ISP3_CMD_ASA400		= 0x82,
    ISP3_CMD_ASA800		= 0x83,
    ISP3_CMD_ASA1600		= 0x84,

    ISP3_CMD_OPTICAL_ZOOM_COMPENSATION	   = 0x90,
    ISP3_CMD_OPTICAL_ZOOMAF_COMPENSATION   = 0x91,
    ISP3_CMD_DIGITAL_ZOOM_ON		= 0x93,	// DIGITAL ZOOM CONTROL FUNCTION
    ISP3_CMD_DIGITAL_ZOOM_OFF		= 0x94,
    ISP3_CMD_DIGITAL_ZOOM_CAPTURE	= 0x95,
    ISP3_CMD_OPTICAL_ZOOM_MOVE	        = 0x96,
    ISP3_CMD_OPTICAL_ZOOMAF_MOVE	= 0x97,
    ISP3_CMD_ATC_ON			= 0x98,	// Adaptive Tone Curve
    ISP3_CMD_ATC_OFF			= 0x99,
    ISP3_CMD_ANR_ON			= 0x9c,	// Adaptive Noise Reduction
    ISP3_CMD_ANR_OFF			= 0x9d,

	// FLICKER DETECTION & SUPPRESSION
	ISP3_CMD_FLICKER_DETECTION_ON					= 0xa0,
	ISP3_CMD_FLICKER_DETECTION_OFF					= 0xa1,
	ISP3_CMD_FLICKER_SUPPRESSION_ON				= 0xa2,
	ISP3_CMD_FLICKER_SUPPRESSION_OFF				= 0xa3,
    // Sensor control        
    ISP3_CMD_SENSOR_INIT		= 0xb0,
    ISP3_CMD_SENSOR_READ	= 0xb1,
    ISP3_CMD_SENSOR_WRITE	= 0xb2,
    // Jacky add this for hisense
ISP3_CMD_FLASHLIGHT_ON							= 0xb4,
ISP3_CMD_FLASHLIGHT_OFF							= 0xb5,

    // AE FLAG NUMBERING        
    ISP3_CMD_AE_OFF			= 0xc0,
    ISP3_CMD_AE_ON			= 0xc1,
    ISP3_CMD_AE_MULTI		= 0xc2,
    ISP3_CMD_AE_GLOBAL		= 0xc3,
    ISP3_CMD_AE_SPOT			= 0xc4,
    ISP3_CMD_SET_SHUTTER_AGC	= 0xc5,	// cltuner sync need
    
    ISP3_CMD_AE_SET_ISO	= 0xc7,	// Jacky update
    //SET_AGC                           
    ISP3_CMD_AE_HALF_RELEASE		= 0xce,
    ISP3_CMD_AE_HALF_SHUTTER	= 0xcf,

    ISP3_CMD_STILL_OFF			= 0xc8,	// Still stabilizer
    ISP3_CMD_STILL_ON			= 0xc9,
    ISP3_CMD_SET_AE_CONFIG			= 0xcb,  // Jacky for EV
    // Image size flag
    ISP3_CMD_PREVIEW 			= 0xd0,		// Preview Size : Frame rate is High, but Image quality is low.
    ISP3_CMD_CAPTURE			= 0xdb,		// Capture Size : Frame rate is Low, but Image quality is High.
    ISP3_CMD_THUMBNAIL_ON		= 0xdc,		// Thumbnail Preview On
    ISP3_CMD_THUMBNAIL_OFF		= 0xdd,		// Thunbnail Preview Off

#ifdef TRULY_2M_MODULE   // Jacky add this for Truly 2M 
    ISP3_CMD_PREVIEW_SVGA		= 0xd1,		//SF SVGA sub-sampling preview
    ISP3_CMD_CAPTURE_5M 		= 0xd2,		//5M capture
    ISP3_CMD_CAPTURE_3M 		= 0xd3,		// 3M capture  add by Jacky
#else    
    ISP3_CMD_CAPTURE_5M 		= 0xd2,		//5M capture
#endif
    //MYShin_081002 MCU Fucntion Added..
/*    ISP3_CMD_SET_13MP_SIZE          = 0xd1,     //Preview Size : 1.3MP(1280 x 960)
    ISP3_CMD_SET_3MP_SIZE           = 0xd2,     //Preview Size : 3MP(2048 x 1536)
    ISP3_CMD_SET_5MP_FULL_SIZE    = 0xd3,     //Preview Size : 5MP(2560 x 1920)   
    ISP3_CMD_SET_SVGA_SIZE         = 0xd4,      //Preview Size : 800 x 600
*/
    ISP3_CMD_SET_VGA_SIZE         = 0xd5,      //Preview Size : 640 x 480
    ISP3_CMD_SET_HVGA_SIZE         = 0xb8,      //Preview Size : 480x320
    ISP3_CMD_DZOOM_SENSOR_VALUE_ON  = 0xd6,  // update
    ISP3_CMD_DZOOM_SENSOR_VALUE_OFF  = 0xd7,
    
    // AWB FLAG NUMBERING
    ISP3_CMD_AWB_SET_MAP1_DATA_TO_REG	= 0xe0,
    ISP3_CMD_AWB_SET_MAP2_DATA_TO_REG	= 0xe1,
    ISP3_CMD_AWB_SET_MAP1_REG_TO_DATA	= 0xe2,
    ISP3_CMD_AWB_SET_MAP2_REG_TO_DATA	= 0xe5,
    ISP3_CMD_AWB_ON			= 0xe3,
    ISP3_CMD_AWB_OFF       		= 0xe4,
    ISP3_CMD_AWB_DEFENCE_ONOFF		= 0xdf,
    ISP3_CMD_CUSTOM_WB			= 0xe6,
    ISP3_CMD_CUSTOM_WB_SET		= 0xe7,
    ISP3_CMD_WB_AUTO			= 0xe8,
    ISP3_CMD_WB_DAYLIGHT		= 0xeb,
    ISP3_CMD_WB_CLOUDY			= 0xec,
    ISP3_CMD_WB_SUNSET			= 0xed,
    ISP3_CMD_WB_FLOURESCENT		= 0xee,
    ISP3_CMD_WB_INCANDESCENT		= 0xef,

    // FACE DETECTION     
    ISP3_CMD_FACE_ROTATE_START						= 0x34,
    ISP3_CMD_FACE_ROTATE_STOP						= 0x35,
    ISP3_CMD_FACE_USERMODE_START					= 0x36,
    ISP3_CMD_FACE_USERMODE_STOP						= 0x37,

    ISP3_CMD_FACE_TRACKING_START						= 0x38,
    ISP3_CMD_FACE_TRACKING_STOP						= 0x39,
    ISP3_CMD_FACE_DETECTION_START		= 0xe6,
    ISP3_CMD_FACE_DETECTION_STOP		= 0xe7,
    ISP3_CMD_FLICKER_DETECTION_START		= 0xe8,
    ISP3_CMD_FLICKER_DETECTION_STOP		= 0xe9,
    ISP3_CMD_FLICKER_DETECTION_120_ENABLE	= 0xa0,
    ISP3_CMD_FLICKER_DETECTION_120_DISABLE	= 0xa1,
    ISP3_CMD_FACE_DETECTION_DEBUG		= 0xa2,
    ISP3_CMD_SET_FLIP_MIRROR                         =0xbe,
    ISP3_CMD_AE_50HZ				= 0xea,
    ISP3_CMD_AE_60HZ				= 0xeb,
    ISP3_CMD_FACE_AF_START			= 0xec,
    ISP3_CMD_FACE_AF_STOP			= 0xed,
    ISP3_CMD_FACE_AE_START			= 0xee,
    ISP3_CMD_FACE_AE_STOP			= 0xef,
    
    // AF  FLAG NUMBERING
    ISP3_CMD_AF_ON		= 0xf0,
    ISP3_CMD_AF_OFF		= 0xf1,
    ISP3_CMD_MICROAF		= 0xf2,   	// ISP3_CMD_CLOSEAF
    ISP3_CMD_FULLAF 		= 0xf3,
    ISP3_CMD_INTELAF		= 0xf4,
    ISP3_CMD_FORWARD 		= 0xf5,
    ISP3_CMD_BACKWARD		= 0xf6,
    ISP3_CMD_ENDNEAR		= 0xf7,
    ISP3_CMD_ENDFAR		= 0xf8,
    ISP3_CMD_AF_INITFAR		= 0xf9,
    ISP3_CMD_AF_MOVE_MOTOR	= 0xfa,
    ISP3_CMD_AF_FULL_SCAN	= 0xfb,   // Added by Jacky 
    ISP3_CMD_AF_MICRO_SCAN	= 0xfc,   // Added by Jacky 
	ISP3_CMD_NONE
} enISP3_CMD_TYPE;




/*------------------------------------------------------------------------------
                Image Processings
------------------------------------------------------------------------------*/
#define         DevID               0xE000
#define         IspFenA             0xE001
#define         IspFenB             0xE002
#define         IspFenC             0xE003
#define         IspFenD             0xE004
#define         IspFenE             0xE005
#define         IspFenF             0xE006
#define         IspFenG             0xE007

/*------------------------------------------------------------------------------
                Sensor Data    Interface
------------------------------------------------------------------------------*/
#define         Cis1IntA            0xE010
#define         Cis1IntB            0xE011
#define         Cis1IntC            0xE012
#define         Cis1IntD            0xE013
#define         Cis1IntE            0xE014
#define         Cis1IntF            0xE015

#define         Cis2IntA            0xE020
#define         Cis2IntB            0xE021
#define         Cis2IntC            0xE022
#define         Cis2IntD            0xE023

/*------------------------------------------------------------------------------
                Sensor Control Interface
------------------------------------------------------------------------------*/
#define         IicDev              0xE030
#define         IicMode1            0xE031
#define         IicMode2            0xE032
#define         IicAddrH            0xE033
#define         IicAddrL            0xE034
#define         IicWrDtH            0xE035
#define         IicWrDtL            0xE036
#define         S3Mode              0xE037
#define         S3SCKPrd            0xE038
#define         SIicFSM             0xE039
#define         IicSPrd             0xE03a
#define         SCtlMode            0xE03b
#define         CISSel              0xE03C    // Temporary hEF97, VISTA : hED97

/*------------------------------------------------------------------------------
                Host access flag
------------------------------------------------------------------------------*/
#define         HostAccess          0xE040

/*------------------------------------------------------------------------------
                Chip Test Gadget
------------------------------------------------------------------------------*/
#define         TstRamSel           0xE048

/*------------------------------------------------------------------------------
                Applocation Processor Interface
------------------------------------------------------------------------------*/
#define         OutFmt              0xE050
#define         OutFmt_EXT          0xE051
#define         CdEn                0xE052
#define         OutEnb              0xE058
#define         OutFmt_JPG          0xE059
#define         PrvMarker1          0xE05a
#define         PrvMarker2          0xE05b
#define         S4Mode              0xE05c
#define         S4SmplMode          0xE05d

/*------------------------------------------------------------------------------
                PLL Management
------------------------------------------------------------------------------*/
#define         PLLMODE             0xE060
#define         PLLMul              0xE061
#define         PLLPre              0xE062
#define         PLLPost             0xE063

/*------------------------------------------------------------------------------
                PAD Control
------------------------------------------------------------------------------*/
#define         PADCTRL0            0xE067
#define         PADCTRL1            0xE068
#define         PADCTRL2            0xE069
#define         PADCTRL3            0xE06a
#define         PADCTRL4            0xE06b

/*------------------------------------------------------------------------------
                MCU Control
------------------------------------------------------------------------------*/
#define         MCURST              0xE070
#define         MCUP0               0xE071
#define         MCUP1               0xE072
#define         MCUP2               0xE073
#define         MCUP3               0xE074
#define         MCUIntSel           0xE075

#define         GPIO0En             0xE080
#define         GPIO1En             0xE081
#define         GPIO2En             0xE082
#define         GPIO3En             0xE083
#define         GPIO0SMCEn          0xE084
#define         GPIO1SMCEn          0xE085
#define         GPIO2SMCEn          0xE086
#define         GPIO3SMCEn          0xE087
#define         GPIO0DirCon         0xE089
#define         GPIO1DirCon         0xE08a
#define         GPIO2DirCon         0xE08b
#define         GPIO3DirCon         0xE08c
#define         GPIO0CPU            0xE08d
#define         GPIO1CPU            0xE08e
#define         GPIO2CPU            0xE08f
#define         GPIO3CPU            0xE090
#define         GPIO0CPD            0xE091
#define         GPIO1CPD            0xE092
#define         GPIO2CPD            0xE093
#define         GPIO3CPD            0xE094


/*------------------------------------------------------------------------------
                Flash Memory Control
------------------------------------------------------------------------------*/
//efine         CdWrapper           0xE0a0
#define         FlashErase          0xE0a1
#define         FlashDEV            0xE0a2
#define         FlashAddrH          0xE0a3
#define         FlashAddrL          0xE0a4
#define         FlashData           0xE0a5
#define         FlashDSizeH         0xE0a6
#define         FlashDSizeL         0xE0a7
#define         UsrRomRdCfg         0xE0a8
#define         UsrRomWrCfg         0xE0a9
#define         SerialAddr1         0xE0aa
#define         SerialAddr2         0xE0ab
#define         SerialAddr3         0xE0ac

#define         FlashCMDAH          0xE0b0
#define         FlashCMDAL          0xE0b1
#define         FlashCMDBH          0xE0b2
#define         FlashCMDBL          0xE0b3
#define         FlashCMDCH          0xE0b4
#define         FlashCMDCL          0xE0b5
#define         FlashCMDDH          0xE0b6
#define         FlashCMDDL          0xE0b7
#define         FlashCMDEH          0xE0b8
#define         FlashCMDEL          0xE0b9
#define         FlashCMDFH          0xE0ba
#define         FlashCMDFL          0xE0bb

#define         FlashCMDDA          0xE0be
#define         FlashCMDDB          0xE0bf
#define         FlashPGMDC          0xE0c0
#define         FlashCMDDC          0xE0c1
#define         FlashCMDDD          0xE0c2
#define         FlashCMDDE          0xE0c3
#define         FlashCMDDF          0xE0c4

#define         FlashDown           0xE0c6

/*------------------------------------------------------------------------------
                Test Pattern
------------------------------------------------------------------------------*/
#define         PttrnSel            0xE0c8
#define         Pattern0            0xE0c9
#define         Pattern1            0xE0ca
#define         Pattern2            0xE0cb
#define         Pattern3            0xE0cc

/*------------------------------------------------------------------------------
                Sync Driving
------------------------------------------------------------------------------*/
#define         CDrvMode            0xE0d0
#define         CDrvXWidH           0xE0d1
#define         CDrvXWidL           0xE0d2
#define         CDrvYWidH           0xE0d3
#define         CDrvYWidL           0xE0d4
#define         CDrvHsWid           0xE0d5
#define         CDrvVsWid           0xE0d6

/*------------------------------------------------------------------------------
                Video Sync Repositioning
------------------------------------------------------------------------------*/
#define         VsyncStr            0xE0e0
#define         VsyncVldPrdH        0xE0e1
#define         VsyncVldPrdL        0xE0e2
#define         VSCntStrH           0xE0e3
#define         VSCntStrL           0xE0e4
#define         VSCntEndH           0xE0e5
#define         VSCntEndL           0xE0e6
#define         VsyncVldPrd_StrH    0xE0e7
#define         VsyncVldPrd_StrL    0xE0e8

/*------------------------------------------------------------------------------
                SPI
------------------------------------------------------------------------------*/
#define         SPICtrl             0xE0f0
#define         SPIClkCtrl          0xE0f1
#define         PADDRH              0xE0f2
#define         PADDRL              0xE0f3
#define         PWDATA_Reg_H        0xE0f4
#define         PWDATA_Reg_L        0xE0f5

/*------------------------------------------------------------------------------
                Optical Black Pixel Window Selection
------------------------------------------------------------------------------*/
#define         BlkWinAStrXH        0xE100
#define         BlkWinAStrXL        0xE101
#define         BlkWinAStrYH        0xE102
#define         BlkWinAStrYL        0xE103
#define         BlkWinAEndXH        0xE104
#define         BlkWinAEndXL        0xE105
#define         BlkWinAEndYH        0xE106
#define         BlkWinAEndYL        0xE107
#define         BlkWinBStrXH        0xE108
#define         BlkWinBStrXL        0xE109
#define         BlkWinBStrYH        0xE10a
#define         BlkWinBStrYL        0xE10b
#define         BlkWinBEndXH        0xE10c
#define         BlkWinBEndXL        0xE10d
#define         BlkWinBEndYH        0xE10e
#define         BlkWinBEndYL        0xE10f
#define         BlkWinCStrXH        0xE110
#define         BlkWinCStrXL        0xE111
#define         BlkWinCStrYH        0xE112
#define         BlkWinCStrYL        0xE113
#define         BlkWinCEndXH        0xE114
#define         BlkWinCEndXL        0xE115
#define         BlkWinCEndYH        0xE116
#define         BlkWinCEndYL        0xE117
#define         BlkWinDStrXH        0xE118
#define         BlkWinDStrXL        0xE119
#define         BlkWinDStrYH        0xE11a
#define         BlkWinDStrYL        0xE11b
#define         BlkWinDEndXH        0xE11c
#define         BlkWinDEndXL        0xE11d
#define         BlkWinDEndYH        0xE11e
#define         BlkWinDEndYL        0xE11f

/*------------------------------------------------------------------------------
                Active Pixel Window Selection (Start)
------------------------------------------------------------------------------*/
#define			ActWinStrXH			0xE120
#define			ActWinStrXL			0xE121
#define			ActWinStrYH			0xE122
#define			ActWinStrYL			0xE123
#define			ActWinEndXH			0xE124
#define			ActWinEndXL			0xE125
#define			ActWinEndYH			0xE126
#define			ActWinEndYL			0xE127
#define         ActWinWidthH        0xE128
#define         ActWinWidthL        0xE129

/*------------------------------------------------------------------------------
                Optical Black Mode & Average Value
------------------------------------------------------------------------------*/
#define         BlkMode             0xE1a0
#define         RgBlack             0xE1a1
#define         GrBlack             0xE1a2
#define         GbBlack             0xE1a3
#define         BgBlack             0xE1a4
#define         BlkScale            0xE1a5

/*------------------------------------------------------------------------------
                Black Offset Adjustment
------------------------------------------------------------------------------*/
#define         BlkLvlRg            0xE1a6
#define         BlkLvlGr            0xE1a7
#define         BlkLvlGb            0xE1a8
#define         BlkLvlBg            0xE1a9

/*------------------------------------------------------------------------------
                Offset adjustment control in Bayer digital gain stage
------------------------------------------------------------------------------*/
#define         RgOfsBgn            0xE1aa
#define         GrOfsBgn            0xE1ab
#define         GbOfsBgn            0xE1ac
#define         BgOfsBgn            0xE1ad

/*------------------------------------------------------------------------------
                R/G/B Channel Gain Control
------------------------------------------------------------------------------*/
#define         PRgGain             0xE1b0    //  Bayer Prefix Gain
#define         PGrGain             0xE1b1
#define         PGbGain             0xE1b2
#define         PBgGain             0xE1b3

#define         RGainH              0xE1b8    //  Bayer  Awb Gain Control
#define         RGainL              0xE1b9
#define         GGainH              0xE1ba
#define         GGainL              0xE1bb
#define         BGainH              0xE1bc
#define         BGainL              0xE1bd

#define         RGainMin            0xE1c8    //  R/G/B Gain Mix/Max Limit
#define         RGainMax            0xE1c9
#define         GGainMin            0xE1ca
#define         GGainMax            0xE1cb
#define         BGainMin            0xE1cc
#define         BGainMax            0xE1cd

#define         WhiteClip           0xE1d0

/*------------------------------------------------------------------------------
                DPC Internal Function Register
------------------------------------------------------------------------------*/
#define         HotRatioThrH_H      0xE1e2
#define         HotRatioThrH_L      0xE1e3
#define         HotRatioThrL_H      0xE1e4
#define         HotRatioThrL_L      0xE1e5

#define         CenterWeight        0xE1e6
                                    
#define         ClipRatioH_Max_H    0xE1e7
#define         ClipRatioH_Max_L    0xE1e8
#define         ClipRatioH_Min_H    0xE1e9
#define         ClipRatioH_Min_L    0xE1ea
#define         ClipRatioL_Max_H    0xE1eb
#define         ClipRatioL_Max_L    0xE1ec
#define         ClipRatioL_Min_H    0xE1ed
#define         ClipRatioL_Min_L    0xE1ee        
                                           
#define         LevelRange_Max_H    0xE1ef
#define         LevelRange_Max_L    0xE1f0
#define         LevelRange_Min_H    0xE1f1
#define         LevelRange_Min_L	0xE1f2

#define			HotThrNo			0xE1f3
#define         LevelThr            0xE1f4 // HotGNoTh, HotCNoTh

/*------------------------------------------------------------------------------
                DPC Pixel Location Address
------------------------------------------------------------------------------*/
#define         DPCPxlPntX0H        0xE200
#define         DPCPxlPntX0L        0xE201
#define         DPCPxlPntY0H        0xE202
#define         DPCPxlPntY0L        0xE203
#define         DPCPxlPntX1H        0xE204
#define         DPCPxlPntX1L        0xE205
#define         DPCPxlPntY1H        0xE206
#define         DPCPxlPntY1L        0xE207
#define         DPCPxlPntX2H        0xE208
#define         DPCPxlPntX2L        0xE209
#define         DPCPxlPntY2H        0xE20a
#define         DPCPxlPntY2L        0xE20b
#define         DPCPxlPntX3H        0xE20c
#define         DPCPxlPntX3L        0xE20d
#define         DPCPxlPntY3H        0xE20e
#define         DPCPxlPntY3L        0xE20f
#define         DPCPxlPntX4H        0xE210
#define         DPCPxlPntX4L        0xE211
#define         DPCPxlPntY4H        0xE212
#define         DPCPxlPntY4L        0xE213
#define         DPCPxlPntX5H        0xE214
#define         DPCPxlPntX5L        0xE215
#define         DPCPxlPntY5H        0xE216
#define         DPCPxlPntY5L        0xE217
#define         DPCPxlPntX6H        0xE218
#define         DPCPxlPntX6L        0xE219
#define         DPCPxlPntY6H        0xE21a
#define         DPCPxlPntY6L        0xE21b
#define         DPCPxlPntX7H        0xE21c
#define         DPCPxlPntX7L        0xE21d
#define         DPCPxlPntY7H        0xE21e
#define         DPCPxlPntY7L        0xE21f
#define         DPCPxlPntX8H        0xE220
#define         DPCPxlPntX8L        0xE221
#define         DPCPxlPntY8H        0xE222
#define         DPCPxlPntY8L        0xE223
#define         DPCPxlPntX9H        0xE224
#define         DPCPxlPntX9L        0xE225
#define         DPCPxlPntY9H        0xE226
#define         DPCPxlPntY9L        0xE227

#define         DPCPxlPntX10H       0xE228
#define         DPCPxlPntX10L       0xE229
#define         DPCPxlPntY10H       0xE22a
#define         DPCPxlPntY10L       0xE22b
#define         DPCPxlPntX11H       0xE22c
#define         DPCPxlPntX11L       0xE22d
#define         DPCPxlPntY11H       0xE22e
#define         DPCPxlPntY11L       0xE22f
#define         DPCPxlPntX12H       0xE230
#define         DPCPxlPntX12L       0xE231
#define         DPCPxlPntY12H       0xE232
#define         DPCPxlPntY12L       0xE233
#define         DPCPxlPntX13H       0xE234
#define         DPCPxlPntX13L       0xE235
#define         DPCPxlPntY13H       0xE236
#define         DPCPxlPntY13L       0xE237
#define         DPCPxlPntX14H       0xE238
#define         DPCPxlPntX14L       0xE239
#define         DPCPxlPntY14H       0xE23a
#define         DPCPxlPntY14L       0xE23b
#define         DPCPxlPntX15H       0xE23c
#define         DPCPxlPntX15L       0xE23d
#define         DPCPxlPntY15H       0xE23e
#define         DPCPxlPntY15L       0xE23f
#define         DPCPxlPntX16H       0xE240
#define         DPCPxlPntX16L       0xE241
#define         DPCPxlPntY16H       0xE242
#define         DPCPxlPntY16L       0xE243
#define         DPCPxlPntX17H       0xE244
#define         DPCPxlPntX17L       0xE245
#define         DPCPxlPntY17H       0xE246
#define         DPCPxlPntY17L       0xE247
#define         DPCPxlPntX18H       0xE248
#define         DPCPxlPntX18L       0xE249
#define         DPCPxlPntY18H       0xE24a
#define         DPCPxlPntY18L       0xE24b
#define         DPCPxlPntX19H       0xE24c
#define         DPCPxlPntX19L       0xE24d
#define         DPCPxlPntY19H       0xE24e
#define         DPCPxlPntY19L       0xE24f

//Flicker Detection
#define			Gap_Thr			    0xE250
#define         Percentage_all      0xE251
#define         Percentage_fk       0xE252
#define         Debug_line_Cnt_End1 0xE253
#define         Debug_line_Cnt_End2 0xE254
#define         Width_ratio         0xE255

#define         Flck_img            0xE256
#define         All_line            0xE257
#define         Dtd_All_line        0xE258
#define         All_fk_line         0xE259
#define         Dtd_All_fk_line     0xE25a
#define         Dtd_All_line_Thr    0xE25b
#define         M_LineCnt           0xE25c
#define         FkGap_Avg           0xE25d
#define         Inflection_Gap_Thr  0xE25e
#define         Row_Skip            0xE25f
#define         min_line            0xE260
#define         min_y               0xE261
#define         max_y               0xE262

#define         PxlDelay            0xE263

#define         RadiusY             0xE270
#define         RadiusCb            0xE271
#define         RadiusCr            0xE272

/*------------------------------------------------------------------------------
                Shading Correction - ISP1 legacy dead address
------------------------------------------------------------------------------*/
#define         ShdMode             0xE2a0
#define         ShdCenX             0xE2a1
#define         ShdCenY             0xE2a2
#define         ShdCnvRtoH          0xE2a3
#define         ShdCnvRtoL          0xE2a4
#define         ShdSclRg            0xE2a5
#define         ShdSclGr            0xE2a6
#define         ShdSclGb            0xE2a7
#define         ShdSclBg            0xE2a8

#define         ShdXpt1             0xE2b0
#define         ShdXpt2             0xE2b1
#define         ShdXpt3             0xE2b2
#define         ShdXpt4             0xE2b3
#define         ShdXpt5             0xE2b4
#define         ShdXpt6             0xE2b5
#define         ShdXpt7             0xE2b6
#define         ShdXpt8             0xE2b7
#define         ShdXpt9             0xE2b8

#define         ShdPtr0             0xE2c0
#define         ShdPtr1             0xE2c1
#define         ShdPtr2             0xE2c2
#define         ShdPtr3             0xE2c3
#define         ShdPtr4             0xE2c4
#define         ShdPtr5             0xE2c5
#define         ShdPtr6             0xE2c6
#define         ShdPtr7             0xE2c7
#define         ShdPtr8             0xE2c8
#define         ShdPtr9             0xE2c9

#define         ShdSlp0             0xE2d0
#define         ShdSlp1             0xE2d1
#define         ShdSlp2             0xE2d2
#define         ShdSlp3             0xE2d3
#define         ShdSlp4             0xE2d4
#define         ShdSlp5             0xE2d5
#define         ShdSlp6             0xE2d6
#define         ShdSlp7             0xE2d7
#define         ShdSlp8             0xE2d8
#define         ShdSlp9             0xE2d9
#define         ShdPtrGVal          0xE2b0

#define         ShdLMAddrH          0xE2e0
#define         ShdLMAddrL          0xE2e1
#define         ShdLMDsizeH         0xE2e2
#define         ShdLMDsizeL         0xE2e3
#define         ShdLMData           0xE2e4

#define         Shade_X_Ea          0xE2e5
#define         Shade_Y_Ea          0xE2e6

/*------------------------------------------------------------------------------
                EXIF data ram access port
------------------------------------------------------------------------------*/
#define         ExifData            0xE2f0
#define         ExifLMAddrH         0xE2f1         
#define         ExifLMAddrL         0xE2f2
#define         DataCnt_Dbg_H       0xE2f3
#define         DataCnt_Dbg_L       0xE2f4
#define         ExifState           0xE2f5
#define			FrameCnt_BestShot	0xE2f6

/*------------------------------------------------------------------------------
                Color Interpolation Block
------------------------------------------------------------------------------*/
#define         IntStrPxl           0xE300
#define         IntRTh              0xE301
#define         IntBTh              0xE302
#define         IntGTh              0xE303

#define			DeBugMode			0xE305
#define			DirectionThrL		0xE306
#define			DirectionThrH		0xE307

#define			DetailDirMul		0xE308
#define			DetailDirDiffMul	0xE309

#define			SyncGenStrH			0xE30a
#define			SyncGenStrL			0xE30b

#define			SyncGenPrd			0xE30c

/*------------------------------------------------------------------------------
                False Color Suppression
------------------------------------------------------------------------------*/
#define         FalseTh             0xE310

#define         FcsHiYHTh           0xE311
#define         FcsHiCHTh           0xE312
#define         FcsHiCLTh           0xE313
#define         FcsHiSlp            0xE314

#define         FcsLoYLTh           0xE315
#define         FcsLoCHTh           0xE316
#define         FcsLoCLTh           0xE317
#define         FcsLoSlp            0xE318

/*------------------------------------------------------------------------------
                Color Matrix
------------------------------------------------------------------------------*/
#define         CMAPO               0xE330
#define         CMA11H              0xE331
#define         CMA11L              0xE332
#define         CMA12H              0xE333
#define         CMA12L              0xE334
#define         CMA13H              0xE335
#define         CMA13L              0xE336
#define         CMA21H              0xE337
#define         CMA21L              0xE338
#define         CMA22H              0xE339
#define         CMA22L              0xE33a
#define         CMA23H              0xE33b
#define         CMA23L              0xE33c
#define         CMA31H              0xE33d
#define         CMA31L              0xE33e
#define         CMA32H              0xE33f
#define         CMA32L              0xE340
#define         CMA33H              0xE341
#define         CMA33L              0xE342
#define         RxOffH              0xE343
#define         RxOffL              0xE344
#define         GxOffH              0xE345
#define         GxOffL              0xE346
#define         BxOffH              0xE347
#define         BxOffL              0xE348

/*------------------------------------------------------------------------------
                Black Point Correction before Gamma
------------------------------------------------------------------------------*/
#define         R_Bpc               0xE350
#define         G_Bpc               0xE351
#define         B_Bpc               0xE352

/*------------------------------------------------------------------------------
                Separate Gamma for R, G, and B
------------------------------------------------------------------------------*/
#define         GmaYPtr00R          0xE400
#define         GmaYPtr01R          0xE401
#define         GmaYPtr02R          0xE402
#define         GmaYPtr03R          0xE403
#define         GmaYPtr04R          0xE404
#define         GmaYPtr05R          0xE405
#define         GmaYPtr06R          0xE406
#define         GmaYPtr07R          0xE407
#define         GmaYPtr08R          0xE408
#define         GmaYPtr09R          0xE409
#define         GmaYPtr0aR          0xE40a
#define         GmaYPtr0bR          0xE40b
#define         GmaYPtr0cR          0xE40c
#define         GmaYPtr0dR          0xE40d
#define         GmaYPtr0eR          0xE40e
#define         GmaYPtr0fR          0xE40f
#define         GmaYPtr10R          0xE410
#define         GmaYPtr11R          0xE411
#define         GmaYPtr12R          0xE412
#define         GmaYPtr13R          0xE413
#define         GmaYPtr14R          0xE414
#define         GmaYPtr15R          0xE415
#define         GmaYPtr16R          0xE416
#define         GmaYPtr17R          0xE417
#define         GmaYPtr18R          0xE418
#define         GmaYPtr19R          0xE419
#define         GmaYPtr1aR          0xE41a
#define         GmaYPtr1bR          0xE41b
#define         GmaYPtr1cR          0xE41c
#define         GmaYPtr1dR          0xE41d
#define         GmaYPtr1eR          0xE41e
#define         GmaYPtr1fR          0xE41f

#define         GmaYPtr00G          0xE420
#define         GmaYPtr01G          0xE421
#define         GmaYPtr02G          0xE422
#define         GmaYPtr03G          0xE423
#define         GmaYPtr04G          0xE424
#define         GmaYPtr05G          0xE425
#define         GmaYPtr06G          0xE426
#define         GmaYPtr07G          0xE427
#define         GmaYPtr08G          0xE428
#define         GmaYPtr09G          0xE429
#define         GmaYPtr0aG          0xE42a
#define         GmaYPtr0bG          0xE42b
#define         GmaYPtr0cG          0xE42c
#define         GmaYPtr0dG          0xE42d
#define         GmaYPtr0eG          0xE42e
#define         GmaYPtr0fG          0xE42f
#define         GmaYPtr10G          0xE430
#define         GmaYPtr11G          0xE431
#define         GmaYPtr12G          0xE432
#define         GmaYPtr13G          0xE433
#define         GmaYPtr14G          0xE434
#define         GmaYPtr15G          0xE435
#define         GmaYPtr16G          0xE436
#define         GmaYPtr17G          0xE437
#define         GmaYPtr18G          0xE438
#define         GmaYPtr19G          0xE439
#define         GmaYPtr1aG          0xE43a
#define         GmaYPtr1bG          0xE43b
#define         GmaYPtr1cG          0xE43c
#define         GmaYPtr1dG          0xE43d
#define         GmaYPtr1eG          0xE43e
#define         GmaYPtr1fG          0xE43f

#define         GmaYPtr00B          0xE440
#define         GmaYPtr01B          0xE441
#define         GmaYPtr02B          0xE442
#define         GmaYPtr03B          0xE443
#define         GmaYPtr04B          0xE444
#define         GmaYPtr05B          0xE445
#define         GmaYPtr06B          0xE446
#define         GmaYPtr07B          0xE447
#define         GmaYPtr08B          0xE448
#define         GmaYPtr09B          0xE449
#define         GmaYPtr0aB          0xE44a
#define         GmaYPtr0bB          0xE44b
#define         GmaYPtr0cB          0xE44c
#define         GmaYPtr0dB          0xE44d
#define         GmaYPtr0eB          0xE44e
#define         GmaYPtr0fB          0xE44f
#define         GmaYPtr10B          0xE450
#define         GmaYPtr11B          0xE451
#define         GmaYPtr12B          0xE452
#define         GmaYPtr13B          0xE453
#define         GmaYPtr14B          0xE454
#define         GmaYPtr15B          0xE455
#define         GmaYPtr16B          0xE456
#define         GmaYPtr17B          0xE457
#define         GmaYPtr18B          0xE458
#define         GmaYPtr19B          0xE459
#define         GmaYPtr1aB          0xE45a
#define         GmaYPtr1bB          0xE45b
#define         GmaYPtr1cB          0xE45c
#define         GmaYPtr1dB          0xE45d
#define         GmaYPtr1eB          0xE45e
#define         GmaYPtr1fB          0xE45f

#define         GmaXPtr00           0xE460
#define         GmaXPtr01           0xE461
#define         GmaXPtr02           0xE462
#define         GmaXPtr03           0xE463
#define         GmaXPtr04           0xE464
#define         GmaXPtr05           0xE465
#define         GmaXPtr06           0xE466
#define         GmaXPtr07           0xE467
#define         GmaXPtr08           0xE468
#define         GmaXPtr09           0xE469
#define         GmaXPtr0a           0xE46a
#define         GmaXPtr0b           0xE46b
#define         GmaXPtr0c           0xE46c
#define         GmaXPtr0d           0xE46d
#define         GmaXPtr0e           0xE46e
#define         GmaXPtr0f           0xE46f
#define         GmaXPtr10           0xE470
#define         GmaXPtr11           0xE471
#define         GmaXPtr12           0xE472
#define         GmaXPtr13           0xE473
#define         GmaXPtr14           0xE474
#define         GmaXPtr15           0xE475
#define         GmaXPtr16           0xE476
#define         GmaXPtr17           0xE477
#define         GmaXPtr18           0xE478
#define         GmaXPtr19           0xE479
#define         GmaXPtr1a           0xE47a
#define         GmaXPtr1b           0xE47b
#define         GmaXPtr1c           0xE47c
#define         GmaXPtr1d           0xE47d
#define         GmaXPtr1e           0xE47e
#define         GmaXPtr1f           0xE47f

/*------------------------------------------------------------------------------
                EFC1 Block Register
------------------------------------------------------------------------------*/
#define			ThrLO				0xE501
#define			ThrLM				0xE502
#define			ThrMH				0xE503
#define			ThrHI				0xE504
#define			Boost				0xE505
#define			boost_EMVSlp0H		0xE506
#define			boost_EMVSlp0M		0xE507
#define			boost_EMVSlp0L		0xE508
#define			boost_EMVSlp1H		0xE509
#define			boost_EMVSlp1M		0xE50a
#define			boost_EMVSlp1L		0xE50b
#define			boost_RangeMaxH		0xE50c
#define			boost_RangeMaxL		0xE50d
#define			boost_RangeMinH		0xE50e
#define			boost_RangeMinL		0xE50f
#define			boost_WeightSlp0H	0xE510
#define			boost_WeightSlp0L	0xE511
#define			boost_WeightSlp1H	0xE512
#define			boost_WeightSlp1L	0xE513
#define			NonZeroThr			0xE514

#define			DropLevelMaxH		0xE515
#define			DropLevelMaxL		0xE516
#define			DropLevelMinH		0xE517
#define			DropLevelMinL		0xE518
#define			DropValue			0xE519
#define			DropLUTSlp0H		0xE51a
#define			DropLUTSlp0L		0xE51b
#define			DropLUTSlp1H		0xE51c
#define			DropLUTSlp1L		0xE51d
#define			EMDThr				0xE51e
#define			GrayRatioThr		0xE51f

/*------------------------------------------------------------------------------
                EFC2 Block Register
------------------------------------------------------------------------------*/
#define			ThrLO2			    0xE521
#define			ThrLM2				0xE522
#define			ThrMH2				0xE523
#define			ThrHI2				0xE524
#define			Boost2				0xE525
#define			boost_EMVSlpH02		0xE526
#define			boost_EMVSlpM02		0xE527
#define			boost_EMVSlpL02		0xE528
#define			boost_EMVSlpH12		0xE529
#define			boost_EMVSlpM12		0xE52a
#define			boost_EMVSlpL12		0xE52b
#define			boost_RangeMax2	    0xE52c
#define			boost_RangeMin2 	0xE52d
#define			boost_WeightSlp0H2	0xE52e
#define			boost_WeightSlp0L2	0xE52f
#define			boost_WeightSlp1H2	0xE530
#define			boost_WeightSlp1L2	0xE531
#define			NonZeroThr2			0xE532

#define			DropLevelMax2		0xE533
#define			DropLevelMin2		0xE534
#define			DropValue2			0xE535
#define			DropLUTSlp0H2		0xE536
#define			DropLUTSlp0L2		0xE537
#define			DropLUTSlp1H2		0xE538
#define			DropLUTSlp1L2		0xE539
#define			EMDThr2				0xE53a
#define			GrayRatioThr2		0xE53b

#define			yy_sum_fr3			0xE53c
#define			yy_sum_fr2			0xE53d
#define			yy_sum_fr1			0xE53e
#define			yy_sum_fr0			0xE53f

#define			BestWinStrXH		0xE540
#define			BestWinStrXL		0xE541
#define			BestWinStrYH		0xE542
#define			BestWinStrYL		0xE543
#define			BestWinEndXH		0xE544
#define			BestWinEndXL		0xE545
#define			BestWinEndYH		0xE546
#define			BestWinEndYL		0xE547

/*------------------------------------------------------------------------------
                Histogram Stretching & Equalization
------------------------------------------------------------------------------*/
#define			Hst_PixelCntH		0xE550 
#define			Hst_PixelCntM       0xE551 
#define			Hst_PixelCntL       0xE552 
#define			Hst_Mean            0xE553 

#define			Region00            0xE554 
#define			Region01            0xE555 
#define			Region02            0xE556 
#define			Region03            0xE557 
#define			Region04            0xE558 
#define			Region05            0xE559 
#define			Region06            0xE55a 
#define			Region07            0xE55b 
#define			Region08            0xE55c 
#define			Region09            0xE55d 
#define			Region0A            0xE55e 
#define			Region0B            0xE55f 
#define			Region0C            0xE560 
#define			Region0D            0xE561 
#define			Region0E            0xE562 
#define			Region0F            0xE563 
#define			Region10            0xE564 
#define			Region11            0xE565 
#define			Region12            0xE566 
#define			Region13            0xE567 
#define			Region14            0xE568 
#define			Region15            0xE569 
#define			Region16            0xE56a 
#define			Region17            0xE56b 
#define			Region18            0xE56c 
#define			Region19            0xE56d 
#define			Region1A            0xE56e 
#define			Region1B            0xE56f 
#define			Region1C            0xE570 
#define			Region1D            0xE571 
#define			Region1E            0xE572 
#define			Region1F            0xE573 

#define			Thr00				0xE574
#define			Thr01				0xE575
#define			Thr02				0xE576
#define			Thr03				0xE577
#define			Thr04				0xE578
#define			Thr05				0xE579
#define			Thr06				0xE57A
#define			Thr07				0xE57B
#define			Thr08				0xE57C
#define			Thr09				0xE57D
#define			Thr0A				0xE57E
#define			Thr0B				0xE57F
#define			Thr0C				0xE580
#define			Thr0D				0xE581
#define			Thr0E				0xE582
#define			Thr0F				0xE583
#define			Thr10				0xE584
#define			Thr11				0xE585
#define			Thr12				0xE586
#define			Thr13				0xE587
#define			Thr14				0xE588
#define			Thr15				0xE589
#define			Thr16				0xE58A
#define			Thr17				0xE58B
#define			Thr18				0xE58C
#define			Thr19				0xE58D
#define			Thr1A				0xE58E
#define			Thr1B				0xE58F
#define			Thr1C				0xE590
#define			Thr1D				0xE591
#define			Thr1E				0xE592

#define			Line_Minus			0xE593

/*------------------------------------------------------------------------------
                Hue/saturation control
                Point color approximation control
------------------------------------------------------------------------------*/
#define         SatCtl              0xE5b0
#define         HueCtl              0xE5b1
#define         HueInpSel           0xE5b2
#define         HuePnt0Quad         0xE5b3
#define         HuePnt0TgtX         0xE5b4
#define         HuePnt0TgtY         0xE5b5
#define         HuePnt0SlpX         0xE5b6
#define         HuePnt0SlpY         0xE5b7
#define         HuePnt1Quad         0xE5b8
#define         HuePnt1TgtX         0xE5b9
#define         HuePnt1TgtY         0xE5ba
#define         HuePnt1SlpX         0xE5bb
#define         HuePnt1SlpY         0xE5bc
#define         HuePnt2Quad         0xE5bd
#define         HuePnt2TgtX         0xE5be
#define         HuePnt2TgtY         0xE5bf
#define         HuePnt2SlpX         0xE5c0
#define         HuePnt2SlpY         0xE5c1
#define         HuePnt3Quad         0xE5c2
#define         HuePnt3TgtX         0xE5c3
#define         HuePnt3TgtY         0xE5c4
#define         HuePnt3SlpX         0xE5c5
#define         HuePnt3SlpY         0xE5c6
#define         HuePnt4Quad         0xE5c7
#define         HuePnt4TgtX         0xE5c8
#define         HuePnt4TgtY         0xE5c9
#define         HuePnt4SlpX         0xE5ca
#define         HuePnt4SlpY         0xE5cb
#define         HuePnt5Quad         0xE5cc
#define         HuePnt5TgtX         0xE5cd
#define         HuePnt5TgtY         0xE5ce
#define         HuePnt5SlpX         0xE5cf
#define         HuePnt5SlpY         0xE5d0
#define         HuePnt6Quad         0xE5d1
#define         HuePnt6TgtX         0xE5d2
#define         HuePnt6TgtY         0xE5d3
#define         HuePnt6SlpX         0xE5d4
#define         HuePnt6SlpY         0xE5d5
#define         HuePnt7Quad         0xE5d6
#define         HuePnt7TgtX         0xE5d7
#define         HuePnt7TgtY         0xE5d8
#define         HuePnt7SlpX         0xE5d9
#define         HuePnt7SlpY         0xE5da

/*------------------------------------------------------------------------------
                Brightness/Contrast Control
------------------------------------------------------------------------------*/
#define         YCntrst             0xE5e0
#define         YBLvl               0xE5e1
#define         YCntrstRef          0xE5e2

/*------------------------------------------------------------------------------
                Auto Function Common Control
------------------------------------------------------------------------------*/
#define         AtoFrWait           0xE600
#define         AtoFrSkip           0xE601
#define         AtoChlSel           0xE603

/*------------------------------------------------------------------------------
                AE Window Register
------------------------------------------------------------------------------*/
#define         AeMode1             0xE610
#define         AeMode2             0xE611
#define         AeMode3             0xE612
#define         Y_Target            0xE613
#define         AeLockFineBnd       0xE614
#define         AeUnlockBnd         0xE615
#define         AeFrSkip            0xE616

#define         IntTimeH            0xE620
#define         IntTimeM            0xE621
#define         IntTimeL            0xE622
#define         AeIntMinLmtH        0xE623
#define         AeIntMinLmtL        0xE624
#define         AeIntStepH          0xE625
#define         AeIntStepM          0xE626
#define         AeIntStepL          0xE627
#define         AeBndOffLmtH        0xE628
#define         AeBndOffLmtM        0xE629
#define         AeBndOffLmtL        0xE62a
#define         AeIntMaxLmtH        0xE62b
#define         AeIntMaxLmtM        0xE62c
#define         AeIntMaxLmtL        0xE62d

#define         PgaNow              0xE640
#define         PgaMin              0xE641
#define         PgaNom              0xE642
#define         PgaMax              0xE643

#define         KlGain              0xE648
#define         KlBndMin            0xE649
#define         KlBndNom            0xE64a
#define         KlBndMax            0xE64b

#define         AeSclLmt            0xE650
#define         AeFrgBck            0xE651
#define         AeVblVal            0xE652

#define         AeFrXWidH           0xE658
#define         AeFrXWidL           0xE659
#define         AeFrYWidH           0xE65a
#define         AeFrYWidL           0xE65b

#define         AeHblAddr           0xE660    //  MCU CamMode
#define         AeHblMode           0xE661    //  MCU CamStatus
#define         AeHblUnit           0xE662    //  MCU CamFlag
#define         AeHblMin            0xE663
#define         AeHblMaxH           0xE664
#define         AeHblMaxL           0xE665

#define         AeCrseAddr1         0xE670    // shutter divide
#define         AeCrseAddr2         0xE671    // ANR
#define         AeCrseMode          0xE672

#define         AeFineAddr          0xE680    // MCU baud rate control
#define         AeFineMode          0xE681    // AWB even/odd map control
#define         AeFineUnit          0xE682    // AWB 1st rank user define
#define         AeFineMaxH          0xE683
#define         AeFineMaxL          0xE684

#define         AePgaAddr           0xE690
#define         AePgaMode           0xE691
#define         AePgaAdrStp         0xE692

#define         AeUpdAddr           0xE6a0
#define         AeUpdMode           0xE6a1

#define         AeDevSel            0xE6a8

#define         AeISO               0xE6a9
#define         AeShutter           0xE6aa

/*------------------------------------------------------------------------------
                AE Window Register
------------------------------------------------------------------------------*/
#define         WinAStrX            0xE6c0
#define         WinAStrY            0xE6c1
#define         WinAEndX            0xE6c2
#define         WinAEndY            0xE6c3
#define         WinBStrX            0xE6c4
#define         WinBStrY            0xE6c5
#define         WinBEndX            0xE6c6
#define         WinBEndY            0xE6c7
#define         WinCStrX            0xE6c8
#define         WinCStrY            0xE6c9
#define         WinCEndX            0xE6ca
#define         WinCEndY            0xE6cb
#define         WinDStrX            0xE6cc
#define         WinDStrY            0xE6cd
#define         WinDEndX            0xE6ce
#define         WinDEndY            0xE6cf
#define         WinEStrX            0xE6d0
#define         WinEStrY            0xE6d1
#define         WinEEndX            0xE6d2
#define         WinEEndY            0xE6d3
#define         WinFStrX            0xE6d4
#define         WinFStrY            0xE6d5
#define         WinFEndX            0xE6d6
#define         WinFEndY            0xE6d7
#define         WinGStrX            0xE6d8
#define         WinGStrY            0xE6d9
#define         WinGEndX            0xE6da
#define         WinGEndY            0xE6db
#define         WinHStrX            0xE6dc
#define         WinHStrY            0xE6dd
#define         WinHEndX            0xE6de
#define         WinHEndY            0xE6df
#define         WinIStrX            0xE6e0
#define         WinIStrY            0xE6e1
#define         WinIEndX            0xE6e2
#define         WinIEndY            0xE6e3
#define         WinJStrX            0xE6e4
#define         WinJStrY            0xE6e5
#define         WinJEndX            0xE6e6
#define         WinJEndY            0xE6e7
#define         WinKStrX            0xE6e8
#define         WinKStrY            0xE6e9
#define         WinKEndX            0xE6ea
#define         WinKEndY            0xE6eb
#define         WinLStrX            0xE6ec
#define         WinLStrY            0xE6ed
#define         WinLEndX            0xE6ee
#define         WinLEndY            0xE6ef

/*------------------------------------------------------------------------------
                AE Window Weighting
------------------------------------------------------------------------------*/
#define         WinABExpWght_f      0xE6f0
#define         WinCDExpWght_f      0xE6f1
#define         WinEFExpWght_f      0xE6f2
#define         WinGHExpWght_f      0xE6f3
#define         WinIJExpWght_f      0xE6f4
#define         WinKLExpWght_f      0xE6f5

#define         WinABExpWght_b      0xE6f6
#define         WinCDExpWght_b      0xE6f7
#define         WinEFExpWght_b      0xE6f8
#define         WinGHExpWght_b      0xE6f9
#define         WinIJExpWght_b      0xE6fa
#define         WinKLExpWght_b      0xE6fb

/*------------------------------------------------------------------------------
                AWB Mode Control
------------------------------------------------------------------------------*/
#define         AwbMode1            0xE800
#define         AwbMode2            0xE801
#define         CbTarget            0xE802
#define         CrTarget            0xE803
#define         AwbLockBnd          0xE804
#define         AwbUnlockBnd        0xE805
#define         AwbWhiteBnd         0xE806

#define         AwbWhite            0xE807
#define         AwbBlack            0xE808
#define         AwbNumbr            0xE809
#define         AwbExpLvl           0xE80a
#define         AwbBlkBnd           0xE80b
#define         AwbTuneRange        0xE80c
#define         AwbFrSkip           0xE80d
#define         AwbWpd              0xE80e

/*------------------------------------------------------------------------------
                Awb Light Source Polygon Map
--------------------------------------------------------------------------------
#define         WpdLn0Mode          0xE810
#define         WpdLn0CnstAH        0xE811
#define         WpdLn0CnstAL        0xE812
#define         WpdLn0CnstBH        0xE813
#define         WpdLn0CnstBL        0xE814

#define         WpdLn1Mode          0xE815
#define         WpdLn1CnstAH        0xE816
#define         WpdLn1CnstAL        0xE817
#define         WpdLn1CnstBH        0xE818
#define         WpdLn1CnstBL        0xE819

#define         WpdLn2Mode          0xE81a
#define         WpdLn2CnstAH        0xE81b
#define         WpdLn2CnstAL        0xE81c
#define         WpdLn2CnstBH        0xE81d
#define         WpdLn2CnstBL        0xE81e

#define         WpdLn3Mode          0xE820
#define         WpdLn3CnstAH        0xE821
#define         WpdLn3CnstAL        0xE822
#define         WpdLn3CnstBH        0xE823
#define         WpdLn3CnstBL        0xE824

#define         WpdLn4Mode          0xE825
#define         WpdLn4CnstAH        0xE826
#define         WpdLn4CnstAL        0xE827
#define         WpdLn4CnstBH        0xE828
#define         WpdLn4CnstBL        0xE829

#define         WpdLn5Mode          0xE82a
#define         WpdLn5CnstAH        0xE82b
#define         WpdLn5CnstAL        0xE82c
#define         WpdLn5CnstBH        0xE82d
#define         WpdLn5CnstBL        0xE82e

#define         WpdLn6Mode          0xE830
#define         WpdLn6CnstAH        0xE831
#define         WpdLn6CnstAL        0xE832
#define         WpdLn6CnstBH        0xE833
#define         WpdLn6CnstBL        0xE834

#define         WpdLn7Mode          0xE835
#define         WpdLn7CnstAH        0xE836
#define         WpdLn7CnstAL        0xE837
#define         WpdLn7CnstBH        0xE838
#define         WpdLn7CnstBL        0xE839

#define         WpdLn8Mode          0xE83a
#define         WpdLn8CnstAH        0xE83b
#define         WpdLn8CnstAL        0xE83c
#define         WpdLn8CnstBH        0xE83d
#define         WpdLn8CnstBL        0xE83e

#define         WpdLn9Mode          0xE840
#define         WpdLn9CnstAH        0xE841
#define         WpdLn9CnstAL        0xE842
#define         WpdLn9CnstBH        0xE843
#define         WpdLn9CnstBL        0xE844
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
                Awb Light Source Box Map 3
------------------------------------------------------------------------------*/
#define         WpdBox23ubts        0xe810
#define         WpdBox23ulxl        0xe811
#define         WpdBox23ulyl        0xe812
#define         WpdBox23lrxl        0xe813
#define         WpdBox23lryl        0xe814
                                            
#define         WpdBox22ubts        0xe815
#define         WpdBox22ulxl        0xe816
#define         WpdBox22ulyl        0xe817
#define         WpdBox22lrxl        0xe818
#define         WpdBox22lryl        0xe819
                                            
#define         WpdBox21ubts        0xe81a
#define         WpdBox21ulxl        0xe81b
#define         WpdBox21ulyl        0xe81c
#define         WpdBox21lrxl        0xe81d
#define         WpdBox21lryl        0xe81e
                                            
#define         WpdBox20ubts        0xe81f
#define         WpdBox20ulxl        0xe820
#define         WpdBox20ulyl        0xe821
#define         WpdBox20lrxl        0xe822
#define         WpdBox20lryl        0xe823
                                            
#define         WpdBox19ubts        0xe824
#define         WpdBox19ulxl        0xe825
#define         WpdBox19ulyl        0xe826
#define         WpdBox19lrxl        0xe827
#define         WpdBox19lryl        0xe828
                                            
#define         WpdBox18ubts        0xe829
#define         WpdBox18ulxl        0xe82a
#define         WpdBox18ulyl        0xe82b
#define         WpdBox18lrxl        0xe82c
#define         WpdBox18lryl        0xe82d
                                            
#define         WpdBox17ubts        0xe82e
#define         WpdBox17ulxl        0xe82f
#define         WpdBox17ulyl        0xe830
#define         WpdBox17lrxl        0xe831
#define         WpdBox17lryl        0xe832
                                           
#define         WpdBox16ubts        0xe833
#define         WpdBox16ulxl        0xe834
#define         WpdBox16ulyl        0xe835
#define         WpdBox16lrxl        0xe836
#define         WpdBox16lryl        0xe837

/*------------------------------------------------------------------------------
                Awb Light Source Box Map 2
------------------------------------------------------------------------------*/
#define         WpdBox15ubts        0xe838
#define         WpdBox15ulxl        0xe839
#define         WpdBox15ulyl        0xe83a
#define         WpdBox15lrxl        0xe83b
#define         WpdBox15lryl        0xe83c
                                            
#define         WpdBox14ubts        0xe83d
#define         WpdBox14ulxl        0xe83e
#define         WpdBox14ulyl        0xe83f
#define         WpdBox14lrxl        0xe840
#define         WpdBox14lryl        0xe841
                                            
#define         WpdBox13ubts        0xe842
#define         WpdBox13ulxl        0xe843
#define         WpdBox13ulyl        0xe844
#define         WpdBox13lrxl        0xe845
#define         WpdBox13lryl        0xe846
                                            
#define         WpdBox12ubts        0xe847
#define         WpdBox12ulxl        0xe848
#define         WpdBox12ulyl        0xe849
#define         WpdBox12lrxl        0xe84a
#define         WpdBox12lryl        0xe84b
                                            
#define         WpdBox11ubts        0xe84c
#define         WpdBox11ulxl        0xe84d
#define         WpdBox11ulyl        0xe84e
#define         WpdBox11lrxl        0xe84f
#define         WpdBox11lryl        0xe850
                                            
#define         WpdBox10ubts        0xe851
#define         WpdBox10ulxl        0xe852
#define         WpdBox10ulyl        0xe853
#define         WpdBox10lrxl        0xe854
#define         WpdBox10lryl        0xe855
                                            
#define         WpdBox9ubts         0xe856
#define         WpdBox9ulxl         0xe857
#define         WpdBox9ulyl         0xe858
#define         WpdBox9lrxl         0xe859
#define         WpdBox9lryl         0xe85a
                                            
#define         WpdBox8ubts         0xe85b
#define         WpdBox8ulxl         0xe85c
#define         WpdBox8ulyl         0xe85d
#define         WpdBox8lrxl         0xe85e
#define         WpdBox8lryl         0xe85f

/*------------------------------------------------------------------------------
                Awb Light Source Box Map 1
------------------------------------------------------------------------------*/
#define         WpdBox7ubts         0xE860
#define         WpdBox7ulxl         0xE861
#define         WpdBox7ulyl         0xE862
#define         WpdBox7lrxl         0xE863
#define         WpdBox7lryl         0xE864

#define         WpdBox6ubts         0xE865
#define         WpdBox6ulxl         0xE866
#define         WpdBox6ulyl         0xE867
#define         WpdBox6lrxl         0xE868
#define         WpdBox6lryl         0xE869

#define         WpdBox5ubts         0xE86a
#define         WpdBox5ulxl         0xE86b
#define         WpdBox5ulyl         0xE86c
#define         WpdBox5lrxl         0xE86d
#define         WpdBox5lryl         0xE86e

#define         WpdBox4ubts         0xE86f
#define         WpdBox4ulxl         0xE870
#define         WpdBox4ulyl         0xE871
#define         WpdBox4lrxl         0xE872
#define         WpdBox4lryl         0xE873

#define         WpdBox3ubts         0xE874
#define         WpdBox3ulxl         0xE875
#define         WpdBox3ulyl         0xE876
#define         WpdBox3lrxl         0xE877
#define         WpdBox3lryl         0xE878

#define         WpdBox2ubts         0xE879
#define         WpdBox2ulxl         0xE87a
#define         WpdBox2ulyl         0xE87b
#define         WpdBox2lrxl         0xE87c
#define         WpdBox2lryl         0xE87d

#define         WpdBox1ubts         0xE87e
#define         WpdBox1ulxl         0xE87f
#define         WpdBox1ulyl         0xE880
#define         WpdBox1lrxl         0xE881
#define         WpdBox1lryl         0xE882

#define         WpdBox0ubts         0xE883
#define         WpdBox0ulxl         0xE884
#define         WpdBox0ulyl         0xE885
#define         WpdBox0lrxl         0xE886
#define         WpdBox0lryl         0xE887

/*------------------------------------------------------------------------------
                AWB Window Weighting Register
------------------------------------------------------------------------------*/
#define         WinABAwbWght_f      0xE890
#define         WinCDAwbWght_f      0xE891
#define         WinEFAwbWght_f      0xE892
#define         WinABAwbWght_b      0xE893
#define         WinCDAwbWght_b      0xE894
#define         WinEFAwbWght_b      0xE895

/*------------------------------------------------------------------------------
                AWB window selection, 2005/12/03/Sat/11/00/11/14/junsc
------------------------------------------------------------------------------*/
#define         AWBWinStrX          0xE898
#define         AWBWinStrY          0xE899
#define         AWBWinEndX          0xE89a
#define         AWBWinEndY          0xE89b

/*------------------------------------------------------------------------------
                Awb Light Source Selection
------------------------------------------------------------------------------*/
#define         B_Y_BlkSel1         0xE8a0
#define         B_Y_BlkSel2         0xE8a1
#define         B_Y_Sel             0xE8a2
#define         Max1Blk             0xE8a3
#define         Max2Blk             0xE8a4
#define         Max3Blk             0xE8a5

/*------------------------------------------------------------------------------
                R/G, B/G ratio
------------------------------------------------------------------------------*/
#define         RGRatioH            0xE8b0
#define         RGRatioL            0xE8b1
#define         BGRatioH            0xE8b2
#define         BGRatioL            0xE8b3
#define         RGSft               0xE8b4
#define         BGSft               0xE8b5

/*------------------------------------------------------------------------------
                AF Mode Control
------------------------------------------------------------------------------*/
#define         AFMode1             0xEA00
#define         AFMode2             0xEA01
#define         AFWinFEVSel         0xEA02
#define         AFFEVTh             0xEA03

#define         af_clp              0xEA10
#define         af_pcpf1            0xEA11
#define         af_pcpf2            0xEA12
#define         af_pcpf3            0xEA13
#define         af_pcpf4            0xEA14

#define         af_scpf1            0xEA18
#define         af_scpf2            0xEA19
#define         af_scpf3            0xEA1a
#define         af_scpf4            0xEA1b

/*------------------------------------------------------------------------------
                AF Window Selection
------------------------------------------------------------------------------*/
#define         AFWinAStrX          0xEA20
#define         AFWinAStrY          0xEA21
#define         AFWinAEndX          0xEA22
#define         AFWinAEndY          0xEA23

#define         AFWinBStrX          0xEA24
#define         AFWinBStrY          0xEA25
#define         AFWinBEndX          0xEA26
#define         AFWinBEndY          0xEA27

#define         AFWinCStrX          0xEA28
#define         AFWinCStrY          0xEA29
#define         AFWinCEndX          0xEA2a
#define         AFWinCEndY          0xEA2b

#define         AFWinDStrX          0xEA2c
#define         AFWinDStrY          0xEA2d
#define         AFWinDEndX          0xEA2e
#define         AFWinDEndY          0xEA2f

#define         AFWinEStrX          0xEA30
#define         AFWinEStrY          0xEA31
#define         AFWinEEndX          0xEA32
#define         AFWinEEndY          0xEA33

#define         AFWinFStrX          0xEA34
#define         AFWinFStrY          0xEA35
#define         AFWinFEndX          0xEA36
#define         AFWinFEndY          0xEA37

/*------------------------------------------------------------------------------
                Actuator Control
------------------------------------------------------------------------------*/
#define         AFManTrgDir         0xEB30
#define         AFFtlDrv            0xEB31
#define         AFManDrv            0xEB32
#define         AFManDrvSum         0xEB33
#define         AFManDrvSumCCW      0xEB34

#define         SMCIOSel            0xEB35
#define         SMCLoop             0xEB36

/*------------------------------------------------------------------------------
                Actuator Drive PWM 1
------------------------------------------------------------------------------*/
#define         AFManIncH_A         0xEB40
#define         AFManIncL_A         0xEB41
#define         SMCCkPrdH_A         0xEB42
#define         SMCCkPrdL_A         0xEB43
#define         SMCTrPrd_A          0xEB44
#define         VCMDuty_A           0xEB45

#define         SMCTrPnt0_A         0xEB50
#define         SMCTrPnt1_A         0xEB51
#define         SMCTrPnt2_A         0xEB52
#define         SMCTrPnt3_A         0xEB53
#define         SMCTrPnt4_A         0xEB54
#define         SMCTrPnt5_A         0xEB55
#define         SMCTrPnt6_A         0xEB56
#define         SMCTrPnt7_A         0xEB57
#define         SMCTrPnt8_A         0xEB58
#define         SMCTrPnt9_A         0xEB59

#define         SMCPttA0_A          0xEB60
#define         SMCPttA1_A          0xEB61
#define         SMCPttA2_A          0xEB62
#define         SMCPttA3_A          0xEB63
#define         SMCPttA4_A          0xEB64
#define         SMCPttA5_A          0xEB65
#define         SMCPttA6_A          0xEB66
#define         SMCPttA7_A          0xEB67
#define         SMCPttA8_A          0xEB68
#define         SMCPttA9_A          0xEB69

#define         SMCPttB0_A          0xEB70
#define         SMCPttB1_A          0xEB71
#define         SMCPttB2_A          0xEB72
#define         SMCPttB3_A          0xEB73
#define         SMCPttB4_A          0xEB74
#define         SMCPttB5_A          0xEB75
#define         SMCPttB6_A          0xEB76
#define         SMCPttB7_A          0xEB77
#define         SMCPttB8_A          0xEB78
#define         SMCPttB9_A          0xEB79

/*------------------------------------------------------------------------------
                Actuator Drive PWM 2
------------------------------------------------------------------------------*/
#define         AFManIncH_B         0xEB80
#define         AFManIncL_B         0xEB81
#define         SMCCkPrdH_B         0xEB82
#define         SMCCkPrdL_B         0xEB83
#define         SMCTrPrd_B          0xEB84
#define         VCMDuty_B           0xEB85

#define         SMCTrPnt0_B         0xEB90
#define         SMCTrPnt1_B         0xEB91
#define         SMCTrPnt2_B         0xEB92
#define         SMCTrPnt3_B         0xEB93
#define         SMCTrPnt4_B         0xEB94
#define         SMCTrPnt5_B         0xEB95
#define         SMCTrPnt6_B         0xEB96
#define         SMCTrPnt7_B         0xEB97
#define         SMCTrPnt8_B         0xEB98
#define         SMCTrPnt9_B         0xEB99

#define         SMCPttA0_B          0xEBa0
#define         SMCPttA1_B          0xEBa1
#define         SMCPttA2_B          0xEBa2
#define         SMCPttA3_B          0xEBa3
#define         SMCPttA4_B          0xEBa4
#define         SMCPttA5_B          0xEBa5
#define         SMCPttA6_B          0xEBa6
#define         SMCPttA7_B          0xEBa7
#define         SMCPttA8_B          0xEBa8
#define         SMCPttA9_B          0xEBa9

#define         SMCPttB0_B          0xEBb0
#define         SMCPttB1_B          0xEBb1
#define         SMCPttB2_B          0xEBb2
#define         SMCPttB3_B          0xEBb3
#define         SMCPttB4_B          0xEBb4
#define         SMCPttB5_B          0xEBb5
#define         SMCPttB6_B          0xEBb6
#define         SMCPttB7_B          0xEBb7
#define         SMCPttB8_B          0xEBb8
#define         SMCPttB9_B          0xEBb9

/*------------------------------------------------------------------------------
                Actuator Drive PWM 3
------------------------------------------------------------------------------*/
#define         AFManIncH_C         0xEBc0
#define         AFManIncL_C         0xEBc1
#define         SMCCkPrdH_C         0xEBc2
#define         SMCCkPrdL_C         0xEBc3
#define         SMCTrPrd_C          0xEBc4
#define         VCMDuty_C           0xEBc5

#define         SMCTrPnt0_C         0xEBd0
#define         SMCTrPnt1_C         0xEBd1
#define         SMCTrPnt2_C         0xEBd2
#define         SMCTrPnt3_C         0xEBd3
#define         SMCTrPnt4_C         0xEBd4
#define         SMCTrPnt5_C         0xEBd5
#define         SMCTrPnt6_C         0xEBd6
#define         SMCTrPnt7_C         0xEBd7
#define         SMCTrPnt8_C         0xEBd8
#define         SMCTrPnt9_C         0xEBd9

#define         SMCPttA0_C          0xEBe0
#define         SMCPttA1_C          0xEBe1
#define         SMCPttA2_C          0xEBe2
#define         SMCPttA3_C          0xEBe3
#define         SMCPttA4_C          0xEBe4
#define         SMCPttA5_C          0xEBe5
#define         SMCPttA6_C          0xEBe6
#define         SMCPttA7_C          0xEBe7
#define         SMCPttA8_C          0xEBe8
#define         SMCPttA9_C          0xEBe9

#define         SMCPttB0_C          0xEBf0
#define         SMCPttB1_C          0xEBf1
#define         SMCPttB2_C          0xEBf2
#define         SMCPttB3_C          0xEBf3
#define         SMCPttB4_C          0xEBf4
#define         SMCPttB5_C          0xEBf5
#define         SMCPttB6_C          0xEBf6
#define         SMCPttB7_C          0xEBf7
#define         SMCPttB8_C          0xEBf8
#define         SMCPttB9_C          0xEBf9

/*------------------------------------------------------------------------------
                AE Statistics
------------------------------------------------------------------------------*/
#define         AeFSM               0xEC00
#define         Y_Avg4F             0xEC01
#define         Y_AvgAll            0xEC02
#define         WinAYAvg            0xEC03
#define         WinBYAvg            0xEC04
#define         WinCYAvg            0xEC05
#define         WinDYAvg            0xEC06
#define         WinEYAvg            0xEC07
#define         WinFYAvg            0xEC08
#define         WinGYAvg            0xEC09
#define         WinHYAvg            0xEC0a
#define         WinIYAvg            0xEC0b
#define         WinJYAvg            0xEC0c
#define         WinKYAvg            0xEC0d
#define         WinLYAvg            0xEC0e

/*------------------------------------------------------------------------------
                AWB Statistics
------------------------------------------------------------------------------*/
#define         AwbFSM              0xEC40
#define         CbAvg4F             0xEC41
#define         CrAvg4F             0xEC42

#define         CCntSumSel          0xEC45
#define         AwbRSumSel          0xEC46
#define         AwbGSumSel          0xEC47
#define         AwbBSumSel          0xEC48

#define         AwbRSum3            0xEC50
#define         AwbRSum2            0xEC51
#define         AwbRSum1            0xEC52
#define         AwbRSum0            0xEC53
#define         AwbGSum3            0xEC54
#define         AwbGSum2            0xEC55
#define         AwbGSum1            0xEC56
#define         AwbGSum0            0xEC57
#define         AwbBSum3            0xEC58
#define         AwbBSum2            0xEC59
#define         AwbBSum1            0xEC5a
#define         AwbBSum0            0xEC5b
#define         CCntSum2            0xEC5c
#define         CCntSum1            0xEC5d
#define         CCntSum0            0xEC5e

#define         RGainGuessH         0xEC60
#define         RGainGuessL         0xEC61
#define         BGainGuessH         0xEC62
#define         BGainGuessL         0xEC63
#define         BiasedGRH           0xEC64
#define         BiasedGRL           0xEC65
#define         BiasedGBH           0xEC66
#define         BiasedGBL           0xEC67

/*------------------------------------------------------------------------------
                AF Statistics
------------------------------------------------------------------------------*/
#define         af_hcp_rx_byte2     0xEC80
#define         af_hcp_rx_byte1     0xEC81
#define         af_hcp_rx_byte0     0xEC82

#define         af_pct1_r1_byte2    0xEC83
#define         af_pct1_r1_byte1    0xEC84
#define         af_pct1_r1_byte0    0xEC85
#define         af_pct2_r1_byte2    0xEC86
#define         af_pct2_r1_byte1    0xEC87
#define         af_pct2_r1_byte0    0xEC88
#define         af_pct3_r1_byte2    0xEC89
#define         af_pct3_r1_byte1    0xEC8a
#define         af_pct3_r1_byte0    0xEC8b
#define         af_pct4_r1_byte2    0xEC8c
#define         af_pct4_r1_byte1    0xEC8d
#define         af_pct4_r1_byte0    0xEC8e

#define         af_pct1_r2_byte2    0xEC8f
#define         af_pct1_r2_byte1    0xEC90
#define         af_pct1_r2_byte0    0xEC91
#define         af_pct2_r2_byte2    0xEC92
#define         af_pct2_r2_byte1    0xEC93
#define         af_pct2_r2_byte0    0xEC94
#define         af_pct3_r2_byte2    0xEC95
#define         af_pct3_r2_byte1    0xEC96
#define         af_pct3_r2_byte0    0xEC97
#define         af_pct4_r2_byte2    0xEC98
#define         af_pct4_r2_byte1    0xEC99
#define         af_pct4_r2_byte0    0xEC9a

#define         af_sum1_r1_byte3    0xEC9b
#define         af_sum1_r1_byte2    0xEC9c
#define         af_sum1_r1_byte1    0xEC9d
#define         af_sum1_r1_byte0    0xEC9e
#define         af_sum2_r1_byte3    0xEC9f
#define         af_sum2_r1_byte2    0xECa0
#define         af_sum2_r1_byte1    0xECa1
#define         af_sum2_r1_byte0    0xECa2
#define         af_sum3_r1_byte3    0xECa3
#define         af_sum3_r1_byte2    0xECa4
#define         af_sum3_r1_byte1    0xECa5
#define         af_sum3_r1_byte0    0xECa6
#define         af_sum4_r1_byte3    0xECa7
#define         af_sum4_r1_byte2    0xECa8
#define         af_sum4_r1_byte1    0xECa9
#define         af_sum4_r1_byte0    0xECaa

#define         af_sum1_r2_byte3    0xECab
#define         af_sum1_r2_byte2    0xECac
#define         af_sum1_r2_byte1    0xECad
#define         af_sum1_r2_byte0    0xECae
#define         af_sum2_r2_byte3    0xECaf
#define         af_sum2_r2_byte2    0xECb0
#define         af_sum2_r2_byte1    0xECb1
#define         af_sum2_r2_byte0    0xECb2
#define         af_sum3_r2_byte3    0xECb3
#define         af_sum3_r2_byte2    0xECb4
#define         af_sum3_r2_byte1    0xECb5
#define         af_sum3_r2_byte0    0xECb6
#define         af_sum4_r2_byte3    0xECb7
#define         af_sum4_r2_byte2    0xECb8
#define         af_sum4_r2_byte1    0xECb9
#define         af_sum4_r2_byte0    0xECba

#define         WinAFEV             0xECbb

#define         AFWinASumB3         0xECbc
#define         AFWinASumB2         0xECbd
#define         AFWinASumB1         0xECbe
#define         AFWinASumB0         0xECbf

#define         AFWinBSumB3         0xECc0
#define         AFWinBSumB2         0xECc1
#define         AFWinBSumB1         0xECc2
#define         AFWinBSumB0         0xECc3

#define         AFWinCSumB3         0xECc4
#define         AFWinCSumB2         0xECc5
#define         AFWinCSumB1         0xECc6
#define         AFWinCSumB0         0xECc7

#define         AFWinDSumB3         0xECc8
#define         AFWinDSumB2         0xECc9
#define         AFWinDSumB1         0xECca
#define         AFWinDSumB0         0xECcb

#define         AFWinESumB3         0xECcc
#define         AFWinESumB2         0xECcd
#define         AFWinESumB1         0xECce
#define         AFWinESumB0         0xECcf

#define         AFWinFSumB3         0xECd0
#define         AFWinFSumB2         0xECd1
#define         AFWinFSumB1         0xECd2
#define         AFWinFSumB0         0xECd3

/*------------------------------------------------------------------------------
               MIPI Control register
------------------------------------------------------------------------------*/
#define         MIPIMode            0xED00
#define         PLLCtlH             0xED01    //PLLCtl[19:0]
#define         PLLCtlM             0xED02
#define         PLLCtlL             0xED03
#define         M_Ctl               0xED04    //BandCtl[7:4],HSZeroCtl[3:0]
#define         LaneCtl             0xED05    //Enable : M0_Clk,M0_Data,S0_Clk,S0_Data,S1_Clk,S1_Data
#define         HSSettleCtl0        0xED06
#define         HSSettleCtl1        0xED07
#define         M_DPHYTestH         0xED08
#define         M_DPHYTestL         0xED09
#define         S0_DPHYTestH        0xED0a
#define         S0_DPHYTestL        0xED0b
#define         S1_DPHYTestH        0xED0c
#define         S1_DPHYTestL        0xED0d
#define         MIPI_ErrCode        0xED0e

/*------------------------------------------------------------------------------
				Image Size Register (Start)
------------------------------------------------------------------------------*/
// Scaler Control Selection (Start)
#define			CR_RatioXH				0xED10
#define			CR_RatioXL				0xED11
#define			CR_RatioYH				0xED12
#define			CR_RatioYL				0xED13
#define			CR_UpHBlankH			0xED14
#define			CR_UpHBlankL			0xED15
#define			CR_VRdRate_SclMode		0xED16
#define			CR_ClkMode_Bypss		0xED17

#define			Pre_RatioXH				0xED18
#define			Pre_RatioXL				0xED19
#define			Pre_RatioYH				0xED1A
#define			Pre_RatioYL				0xED1B
#define			Pre_UpHBlankH			0xED1C
#define			Pre_UpHBlankL			0xED1D
#define			Pre_VRdRate_SclMode		0xED1E
#define			Pre_ClkMode_Bypss		0xED1F

#define			Still_RatioX_SclH		0xED20
#define			Still_RatioX_SclL		0xED21
#define			Still_RatioY_SclH		0xED22
#define			Still_RatioY_SclL		0xED23
#define			Still_UpHBlankH			0xED24
#define			Still_UpHBlankL			0xED25
#define			Still_VRdRate_SclMode	0xED26
#define			Still_ClkMode_Bypss		0xED27

#define			Thu_RatioXH				0xED28
#define			Thu_RatioXL				0xED29
#define			Thu_RatioYH				0xED2A
#define			Thu_RatioYL				0xED2B
#define			Thu_UpHBlankH			0xED2C
#define			Thu_UpHBlankL			0xED2D
#define			Thu_VRdRate_SclMode		0xED2E
#define			Thu_ClkMode_Bypss		0xED2F
// Scaler Control Selection (End)

// Color Restoration Windows & Scaler Selection (Start)
#define			CR_WinXStrH				0xED30
#define			CR_WinXStrL				0xED31
#define			CR_WinYStrH				0xED32
#define			CR_WinYStrL				0xED33
#define			CR_WinWidthH			0xED34
#define			CR_WinWidthL			0xED35
#define			CR_WinHeightH			0xED36
#define			CR_WinHeightL			0xED37

#define			CR_SclWidthIH			0xED38
#define			CR_SclWidthIL			0xED39
#define			CR_SclHeightIH			0xED3A
#define			CR_SclHeightIL			0xED3B
#define			CR_SclWidthOH			0xED3C
#define			CR_SclWidthOL			0xED3D
#define			CR_SclHeightOH			0xED3E
#define			CR_SclHeightOL			0xED3F
// Color Restoration Windows & Scaler Selection (End)

// Preview Windows & Scaler Selection (Start)
#define			Pre_WinXStrH			0xED40
#define			Pre_WinXStrL			0xED41
#define			Pre_WinYStrH			0xED42
#define			Pre_WinYStrL			0xED43
#define			Pre_WinWidthH			0xED44
#define			Pre_WinWidthL			0xED45
#define			Pre_WinHeightH			0xED46
#define			Pre_WinHeightL			0xED47

#define			Pre_SclWidthIH			0xED48
#define			Pre_SclWidthIL			0xED49
#define			Pre_SclHeightIH			0xED4A
#define			Pre_SclHeightIL			0xED4B
#define			Pre_SclWidthOH			0xED4C
#define			Pre_SclWidthOL			0xED4D
#define			Pre_SclHeightOH			0xED4E
#define			Pre_SclHeightOL			0xED4F
// Preview Windows & Scaler Selection (End)

// Still Windows & Scaler Selection (Start)
#define			Still_WinXStrH			0xED50
#define			Still_WinXStrL			0xED51
#define			Still_WinYStrH			0xED52
#define			Still_WinYStrL			0xED53
#define			Still_WinWidthH			0xED54
#define			Still_WinWidthL			0xED55
#define			Still_WinHeightH		0xED56
#define			Still_WinHeightL		0xED57

#define			Still_SclWidthIH		0xED58
#define			Still_SclWidthIL		0xED59
#define			Still_SclHeightIH		0xED5A
#define			Still_SclHeightIL		0xED5B
#define			Still_SclWidthOH		0xED5C
#define			Still_SclWidthOL		0xED5D
#define			Still_SclHeightOH		0xED5E
#define			Still_SclHeightOL		0xED5F
// Still Windows & Scaler Selection (End)

// Thumbnail Windows & Scaler Selection (Start)
#define			Thu_WinXStrH			0xED60
#define			Thu_WinXStrL			0xED61
#define			Thu_WinYStrH			0xED62
#define			Thu_WinYStrL			0xED63
#define			Thu_WinWidthH			0xED64
#define			Thu_WinWidthL			0xED65
#define			Thu_WinHeightH			0xED66
#define			Thu_WinHeightL			0xED67

#define			Thu_SclWidthIH			0xED68
#define			Thu_SclWidthIL			0xED69
#define			Thu_SclHeightIH			0xED6A
#define			Thu_SclHeightIL			0xED6B
#define			Thu_SclWidthOH			0xED6C
#define			Thu_SclWidthOL			0xED6D
#define			Thu_SclHeightOH			0xED6E
#define			Thu_SclHeightOL			0xED6F
// Thumbnail Windows & Scaler Selection (End)

// Image Rotation Mode (Start)
#define			RotMode					0xED70
// Image Rotation Mode (End)

// Frame Skip Control (Start)
#define			FrmSkipNum				0xED71
// Frame Skip Control (End)

/*------------------------------------------------------------------------------
               Best shot register
------------------------------------------------------------------------------*/
#define         FrameSel1           0xEda0
#define         FrameSel2           0xEda1
#define         FrameSel3           0xEda2
#define         FrameLimitWr        0xEda3
#define         FrameLimitRd        0xEda4
#define         BestNumber          0xEda5
#define         FrameCntWr          0xEda6
#define         FrameSkip           0xEda7
#define         LastCnt             0xEda8

#define         ThumbSize0_H        0xEdc0
#define         ThumbSize0_L        0xEdc1
#define         ThumbSize1_H        0xEdc2
#define         ThumbSize1_L        0xEdc3
#define         ThumbSize2_H        0xEdc4
#define         ThumbSize2_L        0xEdc5
#define         ThumbSize3_H        0xEdc6
#define         ThumbSize3_L        0xEdc7
#define         ThumbSize4_H        0xEdc8
#define         ThumbSize4_L        0xEdc9
#define         ThumbSize5_H        0xEdca
#define         ThumbSize5_L        0xEdcb
#define         ThumbSize6_H        0xEdcc
#define         ThumbSize6_L        0xEdcd
#define         ThumbSize7_H        0xEdce
#define         ThumbSize7_L        0xEdcf
#define         ThumbSize8_H        0xEdd0
#define         ThumbSize8_L        0xEdd1

#define         SdramOffset0        0xEdd2
#define         SdramOffset1        0xEdd3
#define         SdramOffset2        0xEdd4


/*------------------------------------------------------------------------------
               SDRAM Control register
------------------------------------------------------------------------------*/
#define         SdramMode1          0xEde0
#define         SdramMode2          0xEde1
#define         Mem_Type            0xEde2
#define         Freq                0xEde3
#define         MRSH                0xEde4
#define         MRSL                0xEde5
#define         TRC                 0xEde6
#define         TRP                 0xEde7
#define         TRCD                0xEde8
#define         TRFC                0xEde9
                                           
#define         BISTMode1           0xEdea
#define         BISTMode2           0xEdeb
#define         RefTime             0xEdec
#define         SD_Arbi_FifoLimit_H 0xEded
#define         SD_Arbi_FifoLimit_L 0xEdee
#define         BlankCnt            0xEdef


/*------------------------------------------------------------------------------
               SMIA Control register
------------------------------------------------------------------------------*/
#define         SMIAMODE            0xEDF0
#define			SMIAOUTMODEH		0xEDF1
#define			SMIAOUTMODEL		0xEDF2
#define			SMIABypass		    0xEDF3

/*------------------------------------------------------------------------------
               TV_ENB register & read address
------------------------------------------------------------------------------*/
#define         ETC_EN              0xEDFE
#define			ETC_EN_RD			0xEDFF

/*------------------------------------------------------------------------------
                JPEG Encoder
------------------------------------------------------------------------------*/
#define         JSizeH              0xEE01
#define         JSizeM              0xEE02
#define         JSizeL              0xEE03
#define         VBlank              0xEE04
#define         YHblank             0xEE05
#define         JWinIWidH           0xEE06    //(Wr10)
#define         JWinIWidL           0xEE07    //(Wr11)
#define         JWinOWidH           0xEE08    //(Wr14)
#define         JWinOWidL           0xEE09    //(Wr15)
#define         RowCntH             0xEE0a
#define         RowCntL             0xEE0b
#define         FlipHold            0xEE0c
#define         RstVal              0xEE0d
#define         JpgRowCntH          0xEE0e
#define         JpgRowCntL          0xEE0f

#define         JpgWidthH           0xEE10
#define         JpgWidthL           0xEE11
#define         JpgHeightH          0xEE12
#define         JpgHeightL          0xEE13
#define         RSF                 0xEE15
#define         JpgOutMode          0xEE16
#define         Jpg_FifoLimit_H     0xEE17   
#define         Jpg_FifoLimit_L     0xEE18
#define         JHBlank             0xEE19
#define         PrvMode             0xEE1a
#define         JVsyncWidth         0xEE1b
#define         JVsyncDlyH          0xEE1c
#define         JVsyncDlyL          0xEE1d
#define         JpgClkDiv           0xEE1e
#define         ReqWidth            0xEE1f

/*------------------------------------------------------------------------------
//ImgEffect
------------------------------------------------------------------------------*/
#define         ImgEffectA          0xEE20
#define         ImgEffectB          0xEE21
#define         ImgEffectC          0xEE22
#define         ImgEffectD          0xEE23
#define         RChr                0xEE24    //(Wr6d)
#define         GChr                0xEE25    //(Wr6e)
#define         BChr                0xEE26    //(Wr6f)
#define         ImgEffectE          0xEE27
#define			SKETCH		    	0xEE28    // Color & Gray Sketch

#define			ReadEn_Blnk		  	0xEE2D    
#define         Addr_Cnst_H         0xEE2E    //jpeg WrAddr_H
#define         Addr_Cnst_L         0xEE2F    //jpeg WrAddr_L

#define         Acc_Cb_Min          0xEE30
#define         Acc_Cb_Max          0xEE31
#define         Acc_Cr_Min          0xEE32
#define         Acc_Cr_Max          0xEE33

#define         Swp_Cb_Min1         0xEE34
#define         Swp_Cb_Max1         0xEE35
#define         Swp_Cb_Min2         0xEE36
#define         Swp_Cb_Max2         0xEE37
#define         Swp_Cr_Min1         0xEE38
#define         Swp_Cr_Max1         0xEE39
#define         Swp_Cr_Min2         0xEE3a
#define         Swp_Cr_Max2         0xEE3b

#define         Swp_Cb1             0xEE3c
#define         Swp_Cb2             0xEE3d
#define         Swp_Cr1             0xEE3e
#define         Swp_Cr2             0xEE3f

#define         ASwp_Cb_Min1        0xEE40
#define         ASwp_Cb_Max1        0xEE41
#define         ASwp_Cb_Min2        0xEE42
#define         ASwp_Cb_Max2        0xEE43
#define         ASwp_Cr_Min1        0xEE44
#define         ASwp_Cr_Max1        0xEE45
#define         ASwp_Cr_Min2        0xEE46
#define         ASwp_Cr_Max2        0xEE47

#define         ASwp_Cb1            0xEE48
#define         ASwp_Cb2            0xEE49
#define         ASwp_Cr1            0xEE4a
#define         ASwp_Cr2            0xEE4b

/*------------------------------------------------------------------------------
// AdToneCu
------------------------------------------------------------------------------*/
#define			Mean_H				0xEE50
#define			Mean_L				0xEE60

#define			Thr0_H				0xEE51
#define			Thr1_H				0xEE52
#define			Thr2_H				0xEE53
#define			Thr3_H				0xEE54
#define			Thr4_H				0xEE55
#define			Thr5_H				0xEE56
#define			Thr6_H				0xEE57
#define			Thr7_H				0xEE58
#define			Thr8_H				0xEE59
#define			Thr9_H				0xEE5A
#define			ThrA_H				0xEE5B
#define			ThrB_H				0xEE5C
#define			ThrC_H				0xEE5D
#define			ThrD_H				0xEE5E
#define			ThrE_H				0xEE5F

#define			Thr0_L				0xEE61
#define			Thr1_L				0xEE62
#define			Thr2_L				0xEE63
#define			Thr3_L				0xEE64
#define			Thr4_L				0xEE65
#define			Thr5_L				0xEE66
#define			Thr6_L				0xEE67
#define			Thr7_L				0xEE68
#define			Thr8_L				0xEE69
#define			Thr9_L				0xEE6A
#define			ThrA_L				0xEE6B
#define			ThrB_L				0xEE6C
#define			ThrC_L				0xEE6D
#define			ThrD_L				0xEE6E
#define			ThrE_L				0xEE6F

#define			Region0				0xEE70
#define			Region1				0xEE71
#define			Region2				0xEE72
#define			Region3				0xEE73
#define			Region4				0xEE74
#define			Region5				0xEE75
#define			Region6				0xEE76
#define			Region7				0xEE77
#define			Region8				0xEE78
#define			Region9				0xEE79
#define			RegionA				0xEE7A
#define			RegionB				0xEE7B
#define			RegionC				0xEE7C
#define			RegionD				0xEE7D
#define			RegionE				0xEE7E
#define			RegionF				0xEE7F

#define			PixelCntH			0xEE80
#define			PixelCntM			0xEE81
#define			PixelCntL			0xEE82

/*------------------------------------------------------------------------------
                Legacy, not used for real
------------------------------------------------------------------------------*/
#define         VscStr              0xEE90
#define         VscEnd              0xEE91
#define         BlkStr              0xEE92
#define         BlkEnd              0xEE93
#define         HscStr              0xEE94
#define         HscEndH             0xEE95
#define         HscEndL             0xEE96
#define         AeVblMaxH           0xEE98
#define         AeVblMaxL           0xEE99
#define         AFFrFEVH            0xEE9a
#define         AFFrFEVL            0xEE9b
#define         VscVldPrd           0xEE9c

/*------------------------------------------------------------------------------
                YCAvg TEST
------------------------------------------------------------------------------*/
#define         YCAvgTest0          0xEEa0
#define         YCAvgTest1          0xEEa1
#define         YCAvgTest2          0xEEa2
#define         YCAvgTest3          0xEEa3

/*------------------------------------------------------------------------------
                Still Stabilizer
------------------------------------------------------------------------------*/
#define			Still_Control_Skip	0xF000
#define			Still_Margin		0xF001
#define			Still_In_FrameCnt	0xF002
#define			Still_Mean			0xF003

// CR Parameter (Start)
#define			Still_RatioX_H		0xF004
#define			Still_RatioX_M		0xF005
#define			Still_RatioX_L		0xF006
#define			Still_RatioY_H		0xF007
#define			Still_RatioY_M		0xF008
#define			Still_RatioY_L		0xF009

#define			Still_MaxThr_H		0xF00a
#define			Still_MaxThr_L		0xF00b
#define			Still_MinThr_H		0xF00c
#define			Still_MinThr_L		0xF00d

#define			Still_CR_Weight_1	0xF010
#define			Still_CR_Weight_2	0xF011
#define			Still_CR_Weight_3	0xF012

#define			Still_Y_Thr_L		0xF013
#define			Still_Thr_Min_Y_L	0xF014
#define			Still_Thr_Max_Y_L	0xF015

#define			Still_Y_Thr_H		0xF016
#define			Still_Thr_Min_Y_H	0xF017
#define			Still_Thr_Max_Y_H	0xF018

#define			Stretching_Max		0xF019
// CR Parameter (End)

// NR Parameter (Start)
#define			Still_Level_L		0xF020
#define			Still_Level_H		0xF021

#define			Still_Edge_Thr_Min	0xF022
#define			Still_Edge_Thr_Max	0xF023
#define			Still_Edge_Thr_Gain	0xF024

#define			Still_Flat_C_Weight	0xF025
#define			Still_Edge_C_Weight	0xF026
#define			Still_Edge_B_Gain	0xF027

#define			Still_Y_Thr1		0xF028
#define			Still_Y_Thr2		0xF029
#define			Still_C_Thr_Min		0xF02a
#define			Still_C_Thr_Max		0xF02b

#define			Still_C_Thr_Ext		0xF02c
// NR Parameter (End)

#define			Still_SyncGenStrH	0xF030
#define			Still_SyncGenStrL	0xF031

// LUT Parameter (Start)
#define			Y_000				0xF100
#define			Y_001				0xF101
#define			Y_002				0xF102
#define			Y_003				0xF103
#define			Y_004				0xF104
#define			Y_005				0xF105
#define			Y_006				0xF106
#define			Y_007				0xF107
#define			Y_008				0xF108
#define			Y_009				0xF109
#define			Y_010				0xF10a
#define			Y_011				0xF10b
#define			Y_012				0xF10c
#define			Y_013				0xF10d
#define			Y_014				0xF10e
#define			Y_015				0xF10f
#define			Y_016				0xF110
#define			Y_017				0xF111
#define			Y_018				0xF112
#define			Y_019				0xF113
#define			Y_020				0xF114
#define			Y_021				0xF115
#define			Y_022				0xF116
#define			Y_023				0xF117
#define			Y_024				0xF118
#define			Y_025				0xF119
#define			Y_026				0xF11a
#define			Y_027				0xF11b
#define			Y_028				0xF11c
#define			Y_029				0xF11d
#define			Y_030				0xF11e
#define			Y_031				0xF11f
#define			Y_032				0xF120
#define			Y_033				0xF121
#define			Y_034				0xF122
#define			Y_035				0xF123
#define			Y_036				0xF124
#define			Y_037				0xF125
#define			Y_038				0xF126
#define			Y_039				0xF127
#define			Y_040				0xF128
#define			Y_041				0xF129
#define			Y_042				0xF12a
#define			Y_043				0xF12b
#define			Y_044				0xF12c
#define			Y_045				0xF12d
#define			Y_046				0xF12e
#define			Y_047				0xF12f
#define			Y_048				0xF130
#define			Y_049				0xF131
#define			Y_050				0xF132
#define			Y_051				0xF133
#define			Y_052				0xF134
#define			Y_053				0xF135
#define			Y_054				0xF136
#define			Y_055				0xF137
#define			Y_056				0xF138
#define			Y_057				0xF139
#define			Y_058				0xF13a
#define			Y_059				0xF13b
#define			Y_060				0xF13c
#define			Y_061				0xF13d
#define			Y_062				0xF13e
#define			Y_063				0xF13f
#define			Y_064				0xF140
#define			Y_065				0xF141
#define			Y_066				0xF142
#define			Y_067				0xF143
#define			Y_068				0xF144
#define			Y_069				0xF145
#define			Y_070				0xF146
#define			Y_071				0xF147
#define			Y_072				0xF148
#define			Y_073				0xF149
#define			Y_074				0xF14a
#define			Y_075				0xF14b
#define			Y_076				0xF14c
#define			Y_077				0xF14d
#define			Y_078				0xF14e
#define			Y_079				0xF14f
#define			Y_080				0xF150
#define			Y_081				0xF151
#define			Y_082				0xF152
#define			Y_083				0xF153
#define			Y_084				0xF154
#define			Y_085				0xF155
#define			Y_086				0xF156
#define			Y_087				0xF157
#define			Y_088				0xF158
#define			Y_089				0xF159
#define			Y_090				0xF15a
#define			Y_091				0xF15b
#define			Y_092				0xF15c
#define			Y_093				0xF15d
#define			Y_094				0xF15e
#define			Y_095				0xF15f
#define			Y_096				0xF160
#define			Y_097				0xF161
#define			Y_098				0xF162
#define			Y_099				0xF163
#define			Y_100				0xF164
#define			Y_101				0xF165
#define			Y_102				0xF166
#define			Y_103				0xF167
#define			Y_104				0xF168
#define			Y_105				0xF169
#define			Y_106				0xF16a
#define			Y_107				0xF16b
#define			Y_108				0xF16c
#define			Y_109				0xF16d
#define			Y_110				0xF16e
#define			Y_111				0xF16f
#define			Y_112				0xF170
#define			Y_113				0xF171
#define			Y_114				0xF172
#define			Y_115				0xF173
#define			Y_116				0xF174
#define			Y_117				0xF175
#define			Y_118				0xF176
#define			Y_119				0xF177
#define			Y_120				0xF178
#define			Y_121				0xF179
#define			Y_122				0xF17a
#define			Y_123				0xF17b
#define			Y_124				0xF17c
#define			Y_125				0xF17d
#define			Y_126				0xF17e
#define			Y_127				0xF17f
#define			Y_128				0xF180
#define			Y_129				0xF181
#define			Y_130				0xF182
#define			Y_131				0xF183
#define			Y_132				0xF184
#define			Y_133				0xF185
#define			Y_134				0xF186
#define			Y_135				0xF187
#define			Y_136				0xF188
#define			Y_137				0xF189
#define			Y_138				0xF18a
#define			Y_139				0xF18b
#define			Y_140				0xF18c
#define			Y_141				0xF18d
#define			Y_142				0xF18e
#define			Y_143				0xF18f
#define			Y_144				0xF190
#define			Y_145				0xF191
#define			Y_146				0xF192
#define			Y_147				0xF193
#define			Y_148				0xF194
#define			Y_149				0xF195
#define			Y_150				0xF196
#define			Y_151				0xF197
#define			Y_152				0xF198
#define			Y_153				0xF199
#define			Y_154				0xF19a
#define			Y_155				0xF19b
#define			Y_156				0xF19c
#define			Y_157				0xF19d
#define			Y_158				0xF19e
#define			Y_159				0xF19f
#define			Y_160				0xF1a0
#define			Y_161				0xF1a1
#define			Y_162				0xF1a2
#define			Y_163				0xF1a3
#define			Y_164				0xF1a4
#define			Y_165				0xF1a5
#define			Y_166				0xF1a6
#define			Y_167				0xF1a7
#define			Y_168				0xF1a8
#define			Y_169				0xF1a9
#define			Y_170				0xF1aa
#define			Y_171				0xF1ab
#define			Y_172				0xF1ac
#define			Y_173				0xF1ad
#define			Y_174				0xF1ae
#define			Y_175				0xF1af
#define			Y_176				0xF1b0
#define			Y_177				0xF1b1
#define			Y_178				0xF1b2
#define			Y_179				0xF1b3
#define			Y_180				0xF1b4
#define			Y_181				0xF1b5
#define			Y_182				0xF1b6
#define			Y_183				0xF1b7
#define			Y_184				0xF1b8
#define			Y_185				0xF1b9
#define			Y_186				0xF1ba
#define			Y_187				0xF1bb
#define			Y_188				0xF1bc
#define			Y_189				0xF1bd
#define			Y_190				0xF1be
#define			Y_191				0xF1bf
#define			Y_192				0xF1c0
#define			Y_193				0xF1c1
#define			Y_194				0xF1c2
#define			Y_195				0xF1c3
#define			Y_196				0xF1c4
#define			Y_197				0xF1c5
#define			Y_198				0xF1c6
#define			Y_199				0xF1c7
#define			Y_200				0xF1c8
#define			Y_201				0xF1c9
#define			Y_202				0xF1ca
#define			Y_203				0xF1cb
#define			Y_204				0xF1cc
#define			Y_205				0xF1cd
#define			Y_206				0xF1ce
#define			Y_207				0xF1cf
#define			Y_208				0xF1d0
#define			Y_209				0xF1d1
#define			Y_210				0xF1d2
#define			Y_211				0xF1d3
#define			Y_212				0xF1d4
#define			Y_213				0xF1d5
#define			Y_214				0xF1d6
#define			Y_215				0xF1d7
#define			Y_216				0xF1d8
#define			Y_217				0xF1d9
#define			Y_218				0xF1da
#define			Y_219				0xF1db
#define			Y_220				0xF1dc
#define			Y_221				0xF1dd
#define			Y_222				0xF1de
#define			Y_223				0xF1df
#define			Y_224				0xF1e0
#define			Y_225				0xF1e1
#define			Y_226				0xF1e2
#define			Y_227				0xF1e3
#define			Y_228				0xF1e4
#define			Y_229				0xF1e5
#define			Y_230				0xF1e6
#define			Y_231				0xF1e7
#define			Y_232				0xF1e8
#define			Y_233				0xF1e9
#define			Y_234				0xF1ea
#define			Y_235				0xF1eb
#define			Y_236				0xF1ec
#define			Y_237				0xF1ed
#define			Y_238				0xF1ee
#define			Y_239				0xF1ef
#define			Y_240				0xF1f0
#define			Y_241				0xF1f1
#define			Y_242				0xF1f2
#define			Y_243				0xF1f3
#define			Y_244				0xF1f4
#define			Y_245				0xF1f5
#define			Y_246				0xF1f6
#define			Y_247				0xF1f7
#define			Y_248				0xF1f8
#define			Y_249				0xF1f9
#define			Y_250				0xF1fa
#define			Y_251				0xF1fb
#define			Y_252				0xF1fc
#define			Y_253				0xF1fd
#define			Y_254				0xF1fe
#define			Y_255				0xF1ff
// LUT Parameter (End)

/*------------------------------------------------------------------------------
                Face Tracking
------------------------------------------------------------------------------*/
#define     FaceAddrH         0xF200
#define     FaceAddrL         0xF201
#define     FaceWDataH        0xF202
#define     FaceWDataL        0xF203
#define     FaceRDataH        0xF204
#define     FaceRDataL        0xF205
#define     FaceRW            0xF206

#define     FaceModeReg       0xF207
#define     FaceFrmComReg     0xF208
#define     FaceFinishReg     0xF209
#define     ROIModeRegH       0xF20A
#define     ROIModeRegL       0xF20B
#define     ROIThick          0xF20C
#define     FaceAFPosH        0xF20F
#define     FaceAFPosL        0xF210
#define     AutoSelAFPos      0xF211
#define     FaceResultCnt     0xF212

#define     FaceAStrX_8051H   0xF3F1
#define     FaceAStrX_8051L   0xF3F2
#define     FaceAStrY_8051H   0xF3F3
#define     FaceAStrY_8051L   0xF3F4
#define     FaceAEndX_8051H   0xF3F5
#define     FaceAEndX_8051L   0xF3F6
#define     FaceAEndY_8051H   0xF3F7
#define     FaceAEndY_8051L   0xF3F8

#define     FaceBStrX_8051H   0xF3F9
#define     FaceBStrX_8051L   0xF3FA
#define     FaceBStrY_8051H   0xF3FB
#define     FaceBStrY_8051L   0xF3FC
#define     FaceBEndX_8051H   0xF3FD
#define     FaceBEndX_8051L   0xF3FE
#define     FaceBEndY_8051H   0xF3FF
#define     FaceBEndY_8051L   0xF400

#define     FaceCStrX_8051H   0xF401
#define     FaceCStrX_8051L   0xF402
#define     FaceCStrY_8051H   0xF403
#define     FaceCStrY_8051L   0xF404
#define     FaceCEndX_8051H   0xF405
#define     FaceCEndX_8051L   0xF406
#define     FaceCEndY_8051H   0xF407
#define     FaceCEndY_8051L   0xF408

#define     FaceDStrX_8051H   0xF409
#define     FaceDStrX_8051L   0xF40A
#define     FaceDStrY_8051H   0xF40B
#define     FaceDStrY_8051L   0xF40C
#define     FaceDEndX_8051H   0xF40D
#define     FaceDEndX_8051L   0xF40E
#define     FaceDEndY_8051H   0xF40F
#define     FaceDEndY_8051L   0xF410

#define     FaceEStrX_8051H   0xF411
#define     FaceEStrX_8051L   0xF412
#define     FaceEStrY_8051H   0xF413
#define     FaceEStrY_8051L   0xF414
#define     FaceEEndX_8051H   0xF415
#define     FaceEEndX_8051L   0xF416
#define     FaceEEndY_8051H   0xF417
#define     FaceEEndY_8051L   0xF418

#define     FaceFStrX_8051H   0xF419
#define     FaceFStrX_8051L   0xF41A
#define     FaceFStrY_8051H   0xF41B
#define     FaceFStrY_8051L   0xF41C
#define     FaceFEndX_8051H   0xF41D
#define     FaceFEndX_8051L   0xF41E
#define     FaceFEndY_8051H   0xF41F
#define     FaceFEndY_8051L   0xF420

#define     FaceGStrX_8051H   0xF421
#define     FaceGStrX_8051L   0xF422
#define     FaceGStrY_8051H   0xF423
#define     FaceGStrY_8051L   0xF424
#define     FaceGEndX_8051H   0xF425
#define     FaceGEndX_8051L   0xF426
#define     FaceGEndY_8051H   0xF427
#define     FaceGEndY_8051L   0xF428

#define     FaceHStrX_8051H   0xF429
#define     FaceHStrX_8051L   0xF42A
#define     FaceHStrY_8051H   0xF42B
#define     FaceHStrY_8051L   0xF42C
#define     FaceHEndX_8051H   0xF42D
#define     FaceHEndX_8051L   0xF42E
#define     FaceHEndY_8051H   0xF42F
#define     FaceHEndY_8051L   0xF430

#define     FaceIStrX_8051H   0xF431
#define     FaceIStrX_8051L   0xF432
#define     FaceIStrY_8051H   0xF433
#define     FaceIStrY_8051L   0xF434
#define     FaceIEndX_8051H   0xF435
#define     FaceIEndX_8051L   0xF436
#define     FaceIEndY_8051H   0xF437
#define     FaceIEndY_8051L   0xF438

#define     FaceJStrX_8051H   0xF439
#define     FaceJStrX_8051L   0xF43A
#define     FaceJStrY_8051H   0xF43B
#define     FaceJStrY_8051L   0xF43C
#define     FaceJEndX_8051H   0xF43D
#define     FaceJEndX_8051L   0xF43E
#define     FaceJEndY_8051H   0xF43F
#define     FaceJEndY_8051L   0xF440

#define     EnAWBFaceH        0xF467
#define     EnAWBFaceL        0xF468
#define     EnAEFaceH         0xF469
#define     EnAEFaceL         0xF46A
#define     EnAFFaceL         0xF46B

/*------------------------------------------------------------------------------
                Low Illiminance Image Data Clipping Test
------------------------------------------------------------------------------*/
#define			En_Debug_LowIll		0xF500

#define			Yx_Clip_High		0xF501
#define			Yx_Clip_Low			0xF502
#define			Yx_Target_Value		0xF503

#define			Cb_Clip_High		0xF504
#define			Cb_Clip_Low			0xF505
#define			Cb_Target_Value		0xF506

#define			Cr_Clip_High		0xF507
#define			Cr_Clip_Low			0xF508
#define			Cr_Target_Value		0xF509


typedef enum
{
    enISP_AE_ISO_AUTO   = 0x00,   
    enISP_AE_ISO_100,			
    enISP_AE_ISO_200,			
    enISP_AE_ISO_300,			
    enISP_AE_ISO_400,			
    enISP_AE_ISO_500,			
    enISP_AE_ISO_600,			
    enISP_AE_ISO_700,			
    enISP_AE_ISO_800			
}enIsp3AEISO;			  // update by Jacky

typedef enum
{
    enISP_Scene_Auto   = 0x00,   
    enISP_Scene_Portrait,			
    enISP_Scene_Landscape,			
    enISP_Scene_Indoor,			
    enISP_Scene_Sports,			
    enISP_Scene_Night,			
    enISP_Scene_Candle,			
    enISP_Scene_Fireworks,			
    enISP_Scene_Snow,			
    enISP_Scene_Sunset			
}enIsp3SceneMode;			  // update by Jacky

typedef struct
{
    ClUint_8 bOutPJpeg;
    ClUint_8 bThumbnail;
}_tIspOutputModeCtrl;

/* Functions */
void System_IICWrite( ClUint_8 id, ClUint_8 addr,  ClUint_8 data);
ClUint_16 System_IICRead(ClUint_16 addr);

ClUint_16 CoreISP3_I2C_Read( ClUint_16 addr );
int CoreISP3_I2C_Write( ClUint_16 addr, ClUint_8 data );
void CoreISP3_I2C_Write_Bulk( ClUint_16 addr, ClUint_8 data );

#ifdef IIC_BURST_MODE
void ISP3_FlashromBurstWrite(int writeStartAddress, ClUint_8* data, int length, void (*callbackFunc)(ClUint_8 *data, int n, int nCount));
#endif //4 IIC_BURST_MODE

void CoreISP3_I2C_Partial_Write( ClUint_16 Addr, ClUint_16 HighBit, ClUint_16 LowBit, ClUint_8 Data );
Cl_Bool CoreISP3_LSC_TableDownLoad( void );
ClUint_8 CoreISP3_MCU_CodeDownload( ClUint_8 * pInitialData, ClUint_32 InitialDataSize, enIsp3DownloadMode ModeParam );
void ISP3_FlashromFormat(void);
void ISP3_FlashromBlockErase(int eraseStartAddress);
//void ISP3_FlashromWrite(int writeStartAddress, ClUint_8* data, int length, void (*callbackFunc)(ClUint_8 *data, int n, int nCount) = Cl_Null);
//void ISP3_FlashromRead(int readStartAddress, ClUint_8 *data, int length, void (*callbackFunc)(ClUint_8 *data, int n, int nCount) = Cl_Null);
void CoreISP3_Send_Command( unsigned char  Cmd_Type );
void CoreISP3_OutpYCbCr( void );
void CoreISP3_RGB565_Out( void );
void CoreISP3_OutpJPEG_Resolution( Cl_Bool bThumbnail , enIsp3OutResolution OutResolution);
void CoreISP3_OutpJPEG( Cl_Bool bThumbnail );
void CoreISP3_SetJPEG_Quality_Factor(enIsp3JPEGQF value);
ClUint_32 CoreISP3_ReadJPEG_Size(void);
void CoreISP3_SetResolution( enIsp3OutResolution OutResolution , CMD_Mode Mode);
void CoreISP3_SetAutoExposure( enIsp3FunctionsAE AE_Param );
void CoreISP3_SetAutoWhiteBalance( enIsp3FunctionsAWB AWB_Param );
void CoreISP3_SetAutoFocus( enIsp3FunctionsAF AF_Param );
void CoreISP3_SetAutoFocus_FullScan(void);
ClUint_32 CoreISP3_GetShutterSpeed(void);
ClUint_32 CoreISP3_GetAutoFocusState(void);
ClUint_32 CoreISP3_GetISOGain(void);
void CoreISP3_SetANR( enISP3FunctionsANRLevel ANR_Param );
void CoreISP3_SetWDR( _tISP_WDR_CTRL *stIspWdrCtrl );
void CoreISP3_SetFaceTracking( Cl_Bool FaceTrackingParam );
void CoreISP3_SetFaceAE( Cl_Bool FaceAEParam );
void CoreISP3_SetFaceAF( Cl_Bool FaceAFParam );
void CoreISP3_SetFaceAWB( Cl_Bool FaceAWBParam );
void CoreISP3_SetFaceRotation( Cl_Bool FaceRotation );
void CoreISP3_SetFaceROIThick( enIsp3FaceROIThick FaceRoiThickParam );
void CoreISP3_SetFaceUserMode( enIspFaceUserMode FaceUserModeParam );
void CoreISP3_SetFaceMaxDetectionCount( enIsp3FaceMaxDetectionCount FaceMaxDetectionParam );
void CoreISP3_ControlFaceApplication( _tIspFaceDetectionCtrl *stIspFaceDetectionCtrlParam );
void CoreISP3_FaceTracking_On(void);
void CoreISP3_FaceTracking_Off(void);
void CoreISP3_StillStabilizerLevelSetup( ClUint_16 LevelParam );
void CoreISP3_SetStillStabilizer( _tIspStillStabilizerCtrl *stIspStillStabilizerCtrlParam );
ClUint_16 CoreISP3_GetZoomSize( ClUint_16 SensorOutput, ClUint_16 DigitalZoomRate );
//void CoreISP3_DigitalZoomScaleSetup( _tIspDigitalZoomCtrl *stIspDigitalZoomCtrlParam );
//void CoreISP3_SetDigitalZoom( _tIspDigitalZoomCtrl *stIspDigitalZoomCtrlParam );
void CoreISP3_SetOutputFormat( _tIspOutputModeCtrl *IspOutputModeCtrlParam );
void CoreISP3_SetImageEffect( enISPFunctionsImageEffect IspImageEffectParam );
void CoreISP3_SetClock( void );
Cl_Bool CoreISP3_Initialize( enIsp3DownloadMode param );

void ISP3_RegWrite_Partial( ClUint_16 addr, ClUint_16 data2,  ClUint_16 data1, ClUint_16 data0);
int ISP3_RegRead(ClUint_16 addr);
void CoreISP3_SetSensorInfo( enIsp3OutResolution Isp3OutResolution );
//void CoreISP3_GetSensorInfo( tSensorInfo *IspSensorInfo );
#ifdef ISP3_ZOOM_ENABLE
void ISP3_Set_StillWindowsStartX(ClUint_16 nData);
void ISP3_Set_StillWindowsStartY(ClUint_16 nData);
void ISP3_Set_StillWindowsWidth(ClUint_16 nData);
void ISP3_Set_StillWindowsHeight(ClUint_16 nData);
void ISP3_Set_ScaleInputWindowsWidth(ClUint_16 nData);
void ISP3_Set_ScaleInputWindowsHeight(ClUint_16 nData);
void ISP3_Set_ScaleOutputWindowsWidth(ClUint_16 nData);
void ISP3_Set_ScaleOutputWindowsHeight(ClUint_16 nData);
void ISP3_Set_JpegWindowsWidth(ClUint_16 nData);
void ISP3_Set_JpegWindowsHeight(ClUint_16 nData);
void ISP3_Set_ThumbnailScaleInputWidth(ClUint_16 nData);
void ISP3_Set_ThumbnailScaleInputHeight(ClUint_16 nData);
void ISP3_Set_ScalePreviewWindowsInputWidth(ClUint_16 nData);
void ISP3_Set_ScalePreviewWindowsInputHeight(ClUint_16 nData);
void ISP3_PreviewZoomScaleSet(unsigned int OutputWidth, unsigned int OutputHeight, unsigned int dwZoom);
void ISP3_PreviewZoomScaleSet_YuLong(unsigned int OutputWidth, unsigned int OutputHeight, unsigned int dwZoom);
void ISP3_ImageCrop(unsigned int sensorOutputWidth, unsigned int sensorOutputHeight,unsigned int CropedWidth,unsigned int CropedHeight, unsigned int OutputWidth,unsigned int OutputHeight);
float ISP3_SetDigitalZoomRate(float zoomRate);
float ISP3_SetDigitalZoomRate_Face(float zoomRate, int startPosDiffx, int startPosDiffy);
void ISP3_ZoomInit(void);
void ISP3_SetZoomRate(enIsp3ZoomRate rate);
void Yulong_TestZoom(void);
void ISP3_SetDigitalZoom(CAM_ZOOM_T rate);
#endif


void CoreISP3_PLLOn(Cl_Bool bOn);
void CoreISP3_FlickerSuppression(FLICKER_TYPE type);
void CoreISP3_AE_SetEV(int ev_Y/**< EV : -5~+5, 0:Default, -5: Dark, +5:Bright */);
void CoreISP3_AE_SetISO(enIsp3AEISO isoLevel/**< ISO Level : 1~8, 0: Auto 1: Low ISO, 8: High ISO*/);
//float
ClUint_32 CoreISP3_Get_Version(void);
void CoreISP3_SetAWBMode(enWB_MANUAL_TYPE type);		
void CoreISP3_Set_WDR_Off(void);
void CoreISP3_Set_StillStabilizer_On(void);
void CoreISP3_Set_StillStabilizer_Off(void);

void ISP3_SetSceneMode(enIsp3SceneMode mode);

void CoreISP3_SetBrightness(enIsp3Level_Value val);
void CoreISP3_SetContrast(enIsp3Level_Value val);
void CoreISP3_SetSaturation(enIsp3Level_Value val);
void CoreISP3_SetHue(enIsp3Level_Value val);

void CoreISP3_Brightness_OnOff(Cl_Bool Brightness);
void CoreISP3_Contrast_OnOff(Cl_Bool Contrast);
void CoreISP3_Saturation_OnOff(Cl_Bool Saturation);
void CoreISP3_Hue_OnOff(Cl_Bool Hue);

void CoreISP3_YUV_Swap(enIsp3YUV_Swap mode);

void CoreISP3_TestPatten(void);
void CoreISP3_PCLK_Inv(void);

void CoreISP3_Mirror_Flip(enIsp3MIRROR_MODE mode);

#ifdef EEPROM_ACCESS_ENABLE
#define DEV_ID	0xA0	// EEPROM I2C DeviceID
#define BYTE	ClUint_8

int ISP3_LiteOnEEPROM_Write(int startAddress, BYTE *data, int length, void (*callbackFunc)(BYTE *data, int n, int nCount));
int ISP3_LiteOnEEPROM_Read(int startAddress, BYTE *data, int length, void (*callbackFunc)(BYTE *data, int n, int nCount));
void EEPROM_Tes_Func(void);
#endif

#ifdef ISP2_EVB_ENABLE
#define _ROTATION_NORMAL_                   1
#define _ROTATION_HORIZONTAL_               2
#define _ROTATION_VERTICAL_                 3
#define _ROTATION_HORIZONTAL_VERTICAL_      4

#define _VGA_RES_                   0
#define _QVGA_RES_                   1
#define _HVGA_RES_               2
#define _3M_RES_                 3
#define _1M_RES_                 4  // 1024x768
#endif

#ifdef __cplusplus
//}
#endif


#endif //3 _COREISP3_H_


