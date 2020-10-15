/*
 * linux/drivers/misc/camera_source/ov2640/camera.h -- Ingenic CIM driver
 *
 * Copyright (C) 2005-2010, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define CAP_WAIT_FRAMES			0
// CLOCK SET--------------------

#define CAM_CLOCK						24000000

// MAX SIZE---------------------
#define MAX_PICTURE_WIDTH		1600
#define MAX_PICTURE_HEIGHT	1200

#define MAX_PREVIEW_WIDTH		640
#define MAX_PREVIEW_HEIGHT	480

#define MAX_PICTURE_BPP			16
#define MAX_PREVIEW_BPP			16

// DEFAULT SIZE-----------------
#define DEF_PICTURE_WIDTH		1600
#define DEF_PICTURE_HEIGHT	1200

#define DEF_PREVIEW_WIDTH		640
#define DEF_PREVIEW_HEIGHT	480

#define DEF_PICTURE_BPP			16
#define DEF_PREVIEW_BPP			16


// DATA SAMPLE-------------------

#define CAM_DATA_PACK_MODE	4
#define CAM_DATA_ORDER			0
#define CAM_DATA_FORMAT			2		//0--RGB  1--YUV444 2--YUV422  3--ITU656 YUV422

#define CAM_SAMPLE_MODE	    2		
#define CAM_DUMMY_ZERO			0		//JUST RGB888 CARE
#define CAM_EXTERNAL_VSYNC	0		//JUST ITU656 CARE
#define CAM_BYPASS					1		//JUST RGB to YUV transform CARE



#define CAM_VSP							1		//VSYNC polarity selection.
#define CAM_HSP							0		//HSYNC polarity selection.
#define CAM_PSP							0
#define CAM_DATA_INV				0


