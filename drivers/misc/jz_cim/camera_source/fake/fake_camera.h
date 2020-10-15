/*
 * linux/drivers/misc/camera_source/ov3640/camera.h -- Ingenic CIM driver
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
#define MAX_CAP_WIDTH	2048	
#define MAX_CAP_HEIGHT   1536	

#define MAX_PRE_WIDTH		1024
#define MAX_PRE_HEIGHT		768

#define MAX_CAP_BPP			16
#define MAX_PRE_BPP			16

// DEFAULT SIZE-----------------
#define DEF_CAP_WIDTH		640
#define DEF_CAP_HEIGHT	480

#define DEF_PRE_WIDTH		640
#define DEF_PRE_HEIGHT	480

#define DEF_CAP_BPP			16
#define DEF_PRE_BPP			16


// DATA SAMPLE------------------

#define CAM_DATA_PACK_MODE	4
#define CAM_DATA_ORDER			0
#define CAM_DATA_FORMAT			2		//0--RGB  1--YUV444 2--YUV422  3--ITU656 YUV422

#define CAM_SAMPLE_MODE	    2		
#define CAM_DUMMY_ZERO			0		//JUST RGB888 CARE
#define CAM_EXTERNAL_VSYNC	0		//JUST ITU656 CARE
#define CAM_BYPASS					1		//JUST RGB to YUV transform CARE



#define CAM_VSP							1		//VSYNC polarity selection.
#define CAM_HSP							0		//HSYNC polarity selection.
#define CAM_PSP							1
#define CAM_DATA_INV				0

//UNIT-----------------------------------------
#define UNIT_WIDTH		160
#define UNIT_HEIGHT		120

#define MAX_SENSOR_NUM          1 

#define COLOR_NUM		3
