/*
 * linux/drivers/misc/jz_cim.h -- Ingenic CIM driver
 *
 * Copyright (C) 2005-2010, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __JZ_CIM_H__
#define __JZ_CIM_H__

#include <linux/i2c.h>
//jz_cim_board_xxxx.c
extern void cim_power_on(void);
extern void cim_power_off(void);

/*
 * IOCTL_XXX commands
 */
#define IOCTL_CIM_START				0xf01 // start preview
#define IOCTL_CIM_STOP				0xf02 // stop preview

#define IOCTL_CIM_START_PREVIEW			0xf03
#define IOCTL_CIM_STOP_PREVIEW			0xf04

#define IOCTL_CIM_GET_PREVIEW_BUF_INFO		0xf05
#define IOCTL_CIM_GET_CAPTURE_BUF_INFO		0xf06

#define IOCTL_CIM_GET_PREVIEW_OFFSET		0xf07

#define IOCTL_CIM_GET_GLOBAL_INFO		0xf08
#define IOCTL_CIM_GET_SENSOR_INFO		0xf09

#define IOCTL_CIM_GET_CAPTURE_PARAM		0xf0a
#define IOCTL_CIM_GET_PREVIEW_PARAM		0xf0b	// get preview size and format

#define IOCTL_CIM_GET_SUPPORT_SIZE		0xf0c

#define IOCTL_CIM_SET_PARAM			0xf0d
//#define IOCTL_CIM_SET_PREVIEW_MEM		0xf0e 	// alloc mem for buffers by app
//#define IOCTL_CIM_SET_CAPTURE_MEM		0xf0f	// alloc mem for buffers by app


#define IOCTL_CIM_TAKE_SNAPSHOT			0xf10
#define IOCTL_CIM_SELECT_SENSOR			0xf11
#define IOCTL_CIM_DO_FOCUS			0xf12
#define IOCTL_CIM_AF_INIT			0xf13
#define IOCTL_CIM_STOP_FOCUS			0xf14

#define IOCTL_CIM_CHANGE_PACK_MODE		0xf15
#define IOCTL_CIM_CHANGE_FMT		0xf16

//-----------------------------------------------------------------------------------------------------------------------
//camera param cmd
#define CPCMD_SET_PREVIEW_RESOLUTION		0x01
#define CPCMD_SET_CAPTURE_RESOLUTION		0x02
#define CPCMD_SET_OUTPUT_FORMAT			0x03


#define CPCMD_SET_BALANCE			0x04
#define CPCMD_SET_EFFECT			0x05
#define CPCMD_SET_ANTIBANDING			0x06
#define CPCMD_SET_FLASH_MODE			0x07
#define CPCMD_SET_SCENE_MODE			0x08
#define CPCMD_SET_PIXEL_FORMAT			0x09
#define CPCMD_SET_FOCUS_MODE			0x0a
#define CPCMD_SET_PREVIEW_FPS			0x0b
#define CPCMD_SET_NIGHTSHOT_MODE		0x0c
#define CPCMD_SET_LUMA_ADAPTATION               0x0d
//-----------------------------------------------------------------------------------------------------------------------


// Values for white balance settings.
#define WHITE_BALANCE_AUTO			0x1<<0
#define WHITE_BALANCE_INCANDESCENT 		0x1<<1
#define WHITE_BALANCE_FLUORESCENT 		0x1<<2
#define WHITE_BALANCE_WARM_FLUORESCENT 		0x1<<3
#define WHITE_BALANCE_DAYLIGHT 			0x1<<4
#define WHITE_BALANCE_CLOUDY_DAYLIGHT 		0x1<<5
#define WHITE_BALANCE_TWILIGHT 			0x1<<6
#define WHITE_BALANCE_SHADE 			0x1<<7

// Values for effect settings.
#define EFFECT_NONE				0x1<<0
#define EFFECT_MONO 				0x1<<1
#define EFFECT_NEGATIVE 			0x1<<2
#define EFFECT_SOLARIZE 			0x1<<3
#define EFFECT_SEPIA 				0x1<<4
#define EFFECT_POSTERIZE 			0x1<<5
#define EFFECT_WHITEBOARD 			0x1<<6
#define EFFECT_BLACKBOARD			0x1<<7
#define EFFECT_AQUA 				0x1<<8
#define EFFECT_PASTEL				0x1<<9
#define EFFECT_MOSAIC				0x1<<10
#define EFFECT_RESIZE				0x1<<11

// Values for antibanding settings.
#define ANTIBANDING_AUTO 			0x1<<0
#define ANTIBANDING_50HZ 			0x1<<1
#define ANTIBANDING_60HZ 			0x1<<2
#define ANTIBANDING_OFF 			0x1<<3

// Values for flash mode settings.
#define FLASH_MODE_OFF				0x1<<0
#define FLASH_MODE_AUTO 			0x1<<1
#define FLASH_MODE_ON 				0x1<<2
#define FLASH_MODE_RED_EYE 			0x1<<3
#define FLASH_MODE_TORCH 		        0x1<<4

// Values for scene mode settings.
#define SCENE_MODE_AUTO 			0x1<<0
#define SCENE_MODE_ACTION 			0x1<<1
#define SCENE_MODE_PORTRAIT   			0x1<<2
#define SCENE_MODE_LANDSCAPE  			0x1<<3
#define SCENE_MODE_NIGHT     			0x1<<4
#define SCENE_MODE_NIGHT_PORTRAIT   		0x1<<5
#define SCENE_MODE_THEATRE  			0x1<<6
#define SCENE_MODE_BEACH   			0x1<<7
#define SCENE_MODE_SNOW    			0x1<<8
#define SCENE_MODE_SUNSET    			0x1<<9
#define SCENE_MODE_STEADYPHOTO   		0x1<<10
#define SCENE_MODE_FIREWORKS    		0x1<<11
#define SCENE_MODE_SPORTS    			0x1<<12
#define SCENE_MODE_PARTY   			0x1<<13
#define SCENE_MODE_CANDLELIGHT 			0x1<<14


//Formats for setPreviewFormat and setPictureFormat.
#define PIXEL_FORMAT_YUV422SP 			0x1<<0
#define PIXEL_FORMAT_YUV420SP 			0x1<<1
#define PIXEL_FORMAT_YUV422I 			0x1<<2
#define PIXEL_FORMAT_RGB565 			0x1<<3
#define PIXEL_FORMAT_JPEG 			0x1<<4

// Values for focus mode settings.
#define FOCUS_MODE_AUTO				0x1<<0
#define FOCUS_MODE_INFINITY 			0x1<<1
#define FOCUS_MODE_MACRO 			0x1<<2
#define FOCUS_MODE_FIXED 			0x1<<3


//-----------------------------------------------------------------------------------------------------------------------
typedef unsigned int balance_flag_t;
typedef unsigned int effect_flag_t;
typedef unsigned int antibanding_flag_t;
typedef unsigned int flash_mode_flag_t;
typedef unsigned int scene_mode_flag_t;
typedef unsigned int pixel_format_flag_t;
typedef unsigned int focus_mode_flag_t;

struct resolution_info
{
	unsigned int 		width;
	unsigned int 		height;
	unsigned int 		bpp;
	pixel_format_flag_t	format;
};


struct mode_flags
{

	balance_flag_t			balance_flag;
	effect_flag_t			effect_flag;
	antibanding_flag_t		antibanding_flag;
	flash_mode_flag_t		flash_mode_flag;
	scene_mode_flag_t 		scene_mode_flag;
	pixel_format_flag_t 		pixel_format_flag;
	focus_mode_flag_t 		focus_mode_flag;
};


struct camera_buffer_info
{
	unsigned int base_vaddr;
	unsigned int base_paddr;
	unsigned int buffer_size;
};

struct camera_param
{
	unsigned int 		cmd;
	struct resolution_info 	capture_param;
	struct resolution_info 	preview_param;

	struct mode_flags 	flags;
	int			mode_arg;
};

#define CIM_FB_PACKED	1
#define CIM_FB_PLANAR	0

struct global_info
{
	unsigned int sensor_count;
	unsigned int mmap_size;
	unsigned int max_preview_size;
	unsigned int preview_buf_nr;
	unsigned int max_capture_size;
	unsigned int packed:1,  /* framebuffer type: 1 -- packed, 0 -- planar */
		fmt_422:1,
		window_support:1,
		reserved:29;
};

struct sensor_info
{
	unsigned int 		sensor_id;
	char 			name[32];

       	unsigned int 		resolution_table_nr;
	struct mode_flags	flags;
	int			bus_width;


	struct resolution_info	max_preview_parm;
	struct resolution_info	max_capture_parm;

	struct resolution_info	preview_parm;
	struct resolution_info	capture_parm;
};

struct sensor_af_arg
{
	int len;
	unsigned char *buf;
};

//-----------------------------------------------------------------------

struct camera_parms
{
	balance_flag_t			balance_flag;
	effect_flag_t			effect_flag;
	antibanding_flag_t		antibanding_flag;
	flash_mode_flag_t		flash_mode_flag;
	scene_mode_flag_t 		scene_mode_flag;
	pixel_format_flag_t 		pixel_format_flag;
	focus_mode_flag_t 		focus_mode_flag;

	int				balance_flag_arg;
	int				effect_flag_arg;
	int				antibanding_flag_arg;
	int				flash_mode_flag_arg;
	int				scene_mode_flag_arg;
	int				pixel_format_flag_arg;
	int				focus_mode_flag_arg;

	int 				preview_fps_arg;
	int 				nightshot_mode_arg;
	int 				luma_adaptation_arg;
};

typedef struct{
	unsigned int cfg;
	unsigned int ctrl;
	unsigned int size;
	unsigned int offs;
	unsigned int packed:1,
		fmt_422:1,
		reserved:30;
} cim_config_t;

typedef enum{CAMERA_MODE_CAPTURE,CAMERA_MODE_PREVIEW} camera_mode_t;

struct camera_sensor_ops
{
	/*
	 *
	 */
	int (*sensor_init)(void);

	/* sensor_set_function use for init preview or capture.
	 * there may be some difference between preview and capture.
	 * so we divided it into two sequences.
	 * param: function indicated which function
	 * 0: preview
	 * 1: capture
	 * 2: recording
	 * 3: do focus
	 * 4: af init
	 * 5: stop focus
	 */
	int (*sensor_set_function)(int function,void *cookie);

	/*
	 *
	 */
	int (*sensor_set_resolution)(int width,int height,int bpp,pixel_format_flag_t fmt,camera_mode_t mode);

	/*
	 *
	 */
	int (*sensor_set_parameter)(int cmd, int mode, int arg);


	/* sensor_set_power use for change sensor power state
	 * state indicate which state is requested
	 * 0: power up
	 * 1: hardware power down
	 * 2: software power down
	 */
	int (*sensor_set_power)(int state);


	/* camera_sensor_probe use for probe weather the sensor present or not
	 * return 0 means success else means fail
	 */
	int (*camera_sensor_probe)(void);

	/* camera_fill_buffer only for fake sensor test
	 */
	int (*camera_fill_buffer)(unsigned int addr,void *desc);
};

struct sensor_cfg_info
{
	__u32 configure_register;
};

struct camera_sensor_desc
{
	unsigned int 		sensor_id;
	struct list_head	list;
	char 			name[32];
	struct i2c_client	*client;

	unsigned int 		camera_clock;
	unsigned int		wait_frames;
	int 			no_dma;

	struct resolution_info 	*resolution_table;
       	unsigned int 		resolution_table_nr;

	struct mode_flags	flags;

	struct resolution_info	max_preview_parm;
	struct resolution_info	max_capture_parm;

	struct resolution_info	preview_parm;
	struct resolution_info	capture_parm;
	struct camera_parms	parms;
	int			bus_width;

	struct sensor_cfg_info	 cfg_info;

	unsigned int max_preview_size;
	unsigned int max_capture_size;

	struct camera_sensor_ops *ops;
};


int camera_sensor_register(struct camera_sensor_desc *desc);

#endif // __JZ_CIM_H__

