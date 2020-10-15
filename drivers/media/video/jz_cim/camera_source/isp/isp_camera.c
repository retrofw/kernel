/*
 * linux/drivers/misc/camera_source/isp/camera.c -- Ingenic CIM driver
 *
 * Copyright (C) 2005-2010, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <asm/jzsoc.h>
#include "camera_ifc.h"
#include "isp.h"
#include "../../jz_cim_core.h"
#include "isp_camera.h"

#define CIM_DEBUG
//#undef CIM_DEBUG
#ifdef CIM_DEBUG
#define dprintk(x...)	printk(x)
#else
#define dprintk(x...)
#endif


//extern void camera_clk_init(void);

/* gpio init */
#if defined(CONFIG_JZ4750_APUS) || defined(CONFIG_JZ4750D_FUWA1) || defined(CONFIG_JZ4750_AQUILA)/* board APUS */
#define GPIO_CAMERA_RST         (32*4+8) /* CIM_MCLK as reset */
#elif defined(CONFIG_JZ4760_F4760) /* JZ4760 FPGA */
#define GPIO_CAMERA_RST         (32*1+9) /* CIM_MCLK as reset */
#elif defined(CONFIG_JZ4760_LEPUS)
#define GPIO_CAMERA_RST         (32*1 + 26) /* GPB26 */
#else
#error "isp/isp_camera.h , please define camera for your board."
#endif



void isp_reset(void)
{
#if  defined(CONFIG_JZ4750_AQUILA)
	__gpio_as_output(5*32+23);
	__gpio_clear_pin(5*32+23);

	__gpio_as_output(2*32+31);
	__gpio_set_pin(2*32+31);
	mdelay(5);
	__gpio_clear_pin(2*32+31);
	mdelay(5);
	__gpio_set_pin(2*32+31);
#else
	__gpio_as_output(GPIO_CAMERA_RST);
	__gpio_clear_pin(GPIO_CAMERA_RST);
	mdelay(50);
	__gpio_set_pin(GPIO_CAMERA_RST);
	mdelay(50);
#endif
}

void isp_power_init(void)
{
}

void isp_power_up(void)
{
#if defined(CONFIG_JZ4760_LEPUS)
	__gpio_as_output(32 * 1 + 27);
	__gpio_clear_pin(32 * 1 + 27);
#else
	__gpio_as_output(4*32+23);
	__gpio_clear_pin(4*32+23);
#endif
}
void isp_power_down(void)
{
#if defined(CONFIG_JZ4760_LEPUS)
	__gpio_as_output(32 * 1 + 27);
	__gpio_set_pin(32 * 1 + 27);
#else
	__gpio_as_output(4*32+23);
	__gpio_set_pin(4*32+23);
#endif
}

void isp_hw_init(void)
{
	//camera_reset();//mclk_pin used as reset_pin, so make the mclk_pin as gpio first

	//2.cim power init
	isp_power_init();

	//3.cim clock init
	//	camera_clk_init();


	//4.cim reset pin  control
	isp_reset();//reset

	//1.cim poewerdowen pin control
	isp_power_up();

}


void camera_wb(camera_wb_type id)
{
	switch(id)
	{
		case CAMERA_WB_MIN_MINUS_1:
			dprintk("CAMERA_WB_MIN_MINUS_1\n");
			break;
		case CAMERA_WB_AUTO :
			dprintk("CAMERA_WB_AUTO\n");
			break;
		case CAMERA_WB_CUSTOM:
			dprintk("CAMERA_WB_CUSTOM\n");
			break;
		case CAMERA_WB_INCANDESCENT:
			dprintk("CAMERA_WB_INCANDESCENT\n");
			break;
		case CAMERA_WB_FLUORESCENT:
			dprintk("CAMERA_WB_FLUORESCENT\n");
			break;
		case CAMERA_WB_DAYLIGHT:
			dprintk("CAMERA_WB_DAYLIGHT\n");
			break;
		case CAMERA_WB_CLOUDY_DAYLIGHT:
			dprintk("CAMERA_WB_CLOUDY_DAYLIGHT\n");
			break;
		case CAMERA_WB_TWILIGHT:
			dprintk("CAMERA_WB_TWILIGHT\n");
			break;
		case CAMERA_WB_SHADE:
			dprintk("CAMERA_WB_SHADE\n");
			break;
		case CAMERA_WB_MAX_PLUS_1:
			dprintk("CAMERA_WB_MAX_PLUS_1\n");
			break;
	}
}


void camera_antibanding(camera_antibanding_type id)
{
	switch(id){
		case CAMERA_ANTIBANDING_OFF:
			dprintk("\n");
			break;
		case CAMERA_ANTIBANDING_60HZ:
			dprintk("\n");
			break;
		case CAMERA_ANTIBANDING_50HZ:
			dprintk("\n");
			break;
		case CAMERA_ANTIBANDING_AUTO:
			dprintk("\n");
			break;
		case CAMERA_MAX_ANTIBANDING:
			dprintk("\n");
			break;
	}
}

void camera_preview_mode(camera_preview_mode_type id)
{
	switch(id){
		case CAMERA_PREVIEW_MODE_SNAPSHOT:
			dprintk("CAMERA_PREVIEW_MODE_SNAPSHOT\n");
			break;
		case CAMERA_PREVIEW_MODE_MOVIE:
			dprintk("CAMERA_PREVIEW_MODE_MOVIE\n");
			break;
		case CAMERA_MAX_PREVIEW_MODE:
			dprintk("CAMERA_MAX_PREVIEW_MODE\n");
			break;
	}
}


int isp_set_parm(unsigned int id,int32_t parm)
{
	int value=0;
	printk("id=0x%x,parm=0x%x\n",id,parm);

	switch(id){
		case CAMERA_PARM_STATE:
			value=CoreISP3_GetAutoFocusState();
			break;
		case CAMERA_PARM_BRIGHTNESS:
			CoreISP3_SetBrightness( parm );
			break;
		case CAMERA_PARM_CONTRAST:
			CoreISP3_SetContrast( parm);
			break;
		case CAMERA_PARM_HUE:
			CoreISP3_SetHue( parm );
			break;
		case CAMERA_PARM_SATURATION:
			CoreISP3_SetSaturation( parm );
			break;
		case CAMERA_PARM_WB:
			CoreISP3_SetAWBMode( parm );
			break;
		case CAMERA_PARM_EFFECT:
			CoreISP3_SetImageEffect( parm );
			break;
			//    case CAMERA_PARM_FOCUS_USER:
			//    	CoreISP3_UserArea_AFOn(int centerX, int centerY, int afWindowWidth, int afWindowHeight);
			//    	break;
		case CAMERA_PARM_AF_MODE:
			CoreISP3_SetAutoFocus( enISP_FUNC_AF_ON );
			break;
		case CAMERA_PARM_FACETRACKING:
			{
				if( parm )
					CoreISP3_FaceTracking_On();
				else
					CoreISP3_FaceTracking_Off();
			}
			break;
			//CoreISP3_Mirror_Flip(enIsp3MIRROR_MODE mode);
	}
	return value;

}
/*void camera_set_dimensions(uint16_t	picture_width,
  uint16_t	picture_height,
  uint16_t	display_width,
  uint16_t	display_height
  )
  {
  printf("set dimensions picture_width=%d,	picture_height=%d,display_width=%d,display_height=%d\n",
  picture_width,picture_height,display_width,display_height);
  }
  void camera_set_position(camera_position_type *position)
  {
  }
  void camera_set_thumbnail_properties(uint32_t width,uint32_t height,uint32_t quality)
  {
  }*/
int isp_set_preview(int width, int height, const char *format)
{

	//CoreISP3_Initialize( enDownloadMode_SkipedMCUBin );
	//ISP3_Set_ScaleOutputWindowsHeight(640);
	//ISP3_Set_ScaleOutputWindowsWidth(480);
	CoreISP3_SetResolution(enISP_RES_QVGA, CMD_Preview);

	return 0;
}
int isp_set_capture(int width, int height, const char *format)
{
	CoreISP3_SetResolution(enISP_RES_QXGA, CMD_Capture);
	return 0;
}

/*
 * Sensor Init Routine
 */


int isp_sensor_init(void)
{

	printk("System_IICRead(0xe060)=0x%x\n",System_IICRead(0xe060));
	CoreISP3_Initialize(
			//enDownloadMode_CodeRAM);
			//enDownloadMode_StackedMem);
		enDownloadMode_SkipedMCUBin);

	//CoreISP3_RGB565_Out();
	CoreISP3_OutpYCbCr();
	CoreISP3_FaceTracking_On();
	//ISP3_Set_ScaleOutputWindowsHeight(320);
	//ISP3_Set_ScaleOutputWindowsWidth(240);
	return 0;
}

/*
   CoreISP3_I2C_Write(0xe062,	0x06);//	#P  pll of XXCLK 48MHz HIVISION
   CoreISP3_I2C_Write(0xe061,	0x18);//	#M
   CoreISP3_I2C_Write(0xe063,	0x02);//	#S  GCLK 80Mhz when XCLK 48Mhz
   CoreISP3_I2C_Write(0xe060,	0x03);//	# PLL OFF
   CoreISP3_I2C_Write(0xe012,	0x01);//	# S1MCLK divide based on SCLK
   mdelay(100);//mdelay(	0x0400);//	# delay 100 msec
   CoreISP3_I2C_Write(0xe010,	0xf0);//	# S1_RST=1
   CoreISP3_I2C_Write(0xe010,	0xd0);//	# S1_RST=0
   mdelay(	0x0064);//	# delay 100 msec
   CoreISP3_I2C_Write(0xe010,	0xf0);//	# S1_RST=1
   CoreISP3_I2C_Write(0xe014,	0x12);//	# Cis1IntE 8051CLK, JCLK division   //0x__22 Jacky change to 0x12 test
   CoreISP3_I2C_Write(0xE6a8,  0x06);//  # resolution setting when initial
   CoreISP3_I2C_Write(0xe660,	0xb0);//  # sensor initial
   CoreISP3_I2C_Write(0xe070,	0x05);//	# MCU GPIO reset state to low
   mdelay(	0x0064);//	# delay 100 msec
   CoreISP3_I2C_Write(0xe070,	0x04);//	# MCU GPIO reset state to low
   mdelay(	0x0400);//mdelay(	0x0400);//	# delay 100 msec
   CoreISP3_I2C_Write(0xedf3,	0x00);//  #async I/F disable
   CoreISP3_I2C_Write(0xe060,	0x00);//	#PLL On
   CoreISP3_I2C_Write(0xe050,	0x1A);//  #rgb output
   CoreISP3_I2C_Write(0xe059,  0x82);//  #ycbcr
   CoreISP3_I2C_Write(0xe660,	0xe6);//	#face detect on
   mdelay(	0x0064);//	# delay 100 msec
   CoreISP3_I2C_Write(0xe660,	0x38);//	#face tracking on
   CoreISP3_I2C_Write(0xe660,	0x34);//	#face rotate
   CoreISP3_I2C_Write(0xf20a,	0x03);//	#face max count
   CoreISP3_I2C_Write(0xf20b,	0xff);//	#face max count 10
 */


int isp_sensor_probe(void)
{

#if  defined(CONFIG_JZ4750_AQUILA)
	return 0;
#endif
	return -1;
}


int isp_set_balance(balance_flag_t balance_flag,int arg)
{
	return 0;
}

int isp_set_effect(effect_flag_t effect_flag,int arg)
{
	return 0;
}

int isp_set_antibanding(antibanding_flag_t antibanding_flag,int arg)
{
	return 0;
}

int isp_set_flash_mode(flash_mode_flag_t flash_mode_flag,int arg)
{
	return 0;

}

int isp_set_scene_mode(scene_mode_flag_t scene_mode_flag,int arg)
{
	return 0;
}

int isp_set_focus_mode(focus_mode_flag_t flash_mode_flag,int arg)
{
	return 0;
}


int isp_set_output_format(pixel_format_flag_t pixel_format_flag,int arg)
{
	switch(pixel_format_flag)
	{
		case PIXEL_FORMAT_JPEG:
			printk("isp set output format to jepg");
			break;
		case PIXEL_FORMAT_YUV422SP:
			printk("isp set output format to yuv422sp");
			break;
		case PIXEL_FORMAT_YUV420SP:
			printk("isp set output format to yuv420sp");
			break;
		case PIXEL_FORMAT_YUV422I:
			CoreISP3_OutpYCbCr();
			printk("isp set output format to yuv422i");
			break;
		case PIXEL_FORMAT_RGB565:
			CoreISP3_RGB565_Out();
			printk("isp set output format to rgb565");
			break;
	}
	return 0;
}

int isp_set_resolution(int width,int height,int bpp,pixel_format_flag_t fmt)
{
	//not complete yet,see the isp_set_preview and isp_set_capture for details

	if(width == 2048 && height == 1536)
		CoreISP3_SetResolution(enISP_RES_QXGA, CMD_Capture);
	if(width == 640 && height == 480)
		CoreISP3_SetResolution(enISP_RES_QVGA, CMD_Preview);
	return 0;
}

struct camera_sensor_ops isp_sensor_ops = {
	.camera_power_down  = isp_power_down,
	.camera_power_up = isp_power_up,
	.camera_power_init = isp_power_init,
	.camera_reset = isp_reset,
	.camera_hw_init = isp_hw_init,
	.camera_sensor_init = isp_sensor_init,

	.camera_set_output_format = isp_set_output_format,
	.camera_set_balance=isp_set_balance,
	.camera_set_effect=isp_set_effect,
	.camera_set_antibanding=isp_set_antibanding,
	.camera_set_flash_mode=isp_set_flash_mode,
	.camera_set_scene_mode=isp_set_scene_mode,
	.camera_set_focus_mode=isp_set_focus_mode,
	.camera_set_resolution = isp_set_resolution,

	.camera_sensor_probe = isp_sensor_probe,
};

struct resolution_info isp_resolution_table[] = {
	{2048,1536,16,PIXEL_FORMAT_YUV422I},
	{640,480,16,PIXEL_FORMAT_YUV422I},
};

struct camera_sensor_desc isp_sensor_desc = {
	.name = "isp",
	.ops = &isp_sensor_ops,
	.address = 0x78,
	.camera_clock = CAM_CLOCK,
	.wait_frames =0,

	.resolution_table = isp_resolution_table,
	.resolution_table_nr=ARRAY_SIZE(isp_resolution_table),

	.flags={
		.pixel_format_flag = PIXEL_FORMAT_YUV422I,
	},

	.preview_parm = {DEF_PRE_WIDTH, DEF_PRE_HEIGHT, DEF_PRE_BPP,PIXEL_FORMAT_YUV422I},
	.capture_parm = {DEF_CAP_WIDTH, DEF_CAP_HEIGHT, DEF_CAP_BPP,PIXEL_FORMAT_YUV422I},
	.max_preview_parm = {MAX_PRE_WIDTH, MAX_PRE_HEIGHT, MAX_PRE_BPP,PIXEL_FORMAT_YUV422I},
	.max_capture_parm = {MAX_CAP_WIDTH, MAX_CAP_HEIGHT, MAX_CAP_BPP,PIXEL_FORMAT_YUV422I},

	.cfg_info={
		.cam_data_pack_mode	= CAM_DATA_PACK_MODE,
		.cam_data_order		= CAM_DATA_ORDER,
		.cam_data_format	= CAM_DATA_FORMAT,

		.cam_sample_mode	= CAM_SAMPLE_MODE,
		.cam_dummy_zero		= CAM_DUMMY_ZERO,
		.cam_external_vsync	= CAM_EXTERNAL_VSYNC,
		.cam_bypass		= CAM_BYPASS,

		.cam_vsp		= CAM_VSP,
		.cam_hsp		= CAM_HSP,
		.cam_psp		= CAM_PSP,
		.cam_data_inv		= CAM_DATA_INV,
	},
};

int isp_register(void)
{
	camera_sensor_register(&isp_sensor_desc);
	printk("isp sensor!");
	return 0;
}

early_initcall(isp_register);





