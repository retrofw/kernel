

#include <asm/jzsoc.h>
#include <linux/i2c.h>

#include "../../jz_cim_core.h"
#include "../../jz_sensor.h"

#include "ov3640_camera.h"
#include "ov3640_set.h"
#include "ov3640_set_mode.h"
#include "ov3640_focus.h"


//#define OV3640_DEBUG
//#undef DEBUG

#ifdef OV3640_DEBUG
#define dprintk(x...)   do{printk("OV3640---\t");printk(x);printk("\n");}while(0)
#else
#define dprintk(x...)
#endif

struct camera_sensor_desc ov3640_sensor_desc;

/* gpio init */
#if defined(CONFIG_JZ4750_APUS) || defined(CONFIG_JZ4750D_FUWA1) || defined(CONFIG_JZ4750_AQUILA)/* board APUS */
#define GPIO_CAMERA_RST         (32*4+8) /* MCLK as reset */
#elif defined(CONFIG_JZ4760_ALTAIR)
#define GPIO_CAMERA_RST         (32*4+13) /*GPE13*/
#elif defined(CONFIG_JZ4760_LEPUS) || defined(CONFIG_JZ4760B_LEPUS)
#define GPIO_CAMERA_RST         (32*1 + 26) /* GPB26 */
#elif defined(CONFIG_JZ4760_F4760) || defined(CONFIG_JZ4810_F4810)/* JZ4760 FPGA */
#define GPIO_CAMERA_RST         (32*1+9) /* CIM_MCLK as reset */
#elif defined(CONFIG_JZ4770_F4770)
#define GPIO_CAMERA_RST		GPB(9)
#elif defined(CONFIG_JZ4770_PISCES)
#define GPIO_CAMERA_RST		GPE(3)
#else
#error "ov3640/ov3640_camera.c , please define camera for your board."
#endif

void ov3640_power_down(void)
{
	dprintk("=======%s:%d\n", __FUNCTION__, __LINE__);
#if defined(CONFIG_JZ4750_AQUILA)
	__gpio_as_output(4*32+23);
	__gpio_set_pin(4*32+23);
#elif defined(CONFIG_JZ4760_ALTAIR)
	__gpio_as_output(0*32+27);                                    /* GPA27 */
	__gpio_set_pin(0*32+27);
#elif defined(CONFIG_JZ4760_LEPUS) || defined(CONFIG_JZ4760B_LEPUS)
	__gpio_as_output(32 * 1 + 27); /* GPB27 */
	__gpio_set_pin(32 * 1 + 27);
#elif defined(CONFIG_JZ4770_PISCES)
	__gpio_as_output1(GPE(4));
#endif
	mdelay(5);
}

void ov3640_power_up(void)
{
	dprintk("=======%s:%d\n", __FUNCTION__, __LINE__);
#if defined(CONFIG_JZ4750_AQUILA)
	__gpio_as_output(4*32+23);
	__gpio_clear_pin(4*32+23);
#elif defined(CONFIG_JZ4760_ALTAIR)
	__gpio_as_output(0*32+27);
	__gpio_clear_pin(0*32+27);
#elif defined(CONFIG_JZ4760_LEPUS) || defined(CONFIG_JZ4760B_LEPUS)
	__gpio_as_output(32 * 1 + 27); /* GPB27 */
	__gpio_clear_pin(32 * 1 + 27);
#elif defined(CONFIG_JZ4770_PISCES)
	__gpio_as_output0(GPE(4));
#endif
	mdelay(5);
}


int ov3640_set_mclk(unsigned int mclk)
{
	/* Set the master clock output */
	/* If use pll clock, enable it */
	// __cim_set_master_clk(__cpm_get_hclk(), c->mclk);
	return 0;//in this board use the extra osc - -20MHZ
}

void ov3640_reset(void)
{
#if  defined(CONFIG_JZ4750_AQUILA)
	__gpio_as_output(5*32+23); /* GPIO_IO_SWETCH_EN */
	__gpio_clear_pin(5*32+23);
	__gpio_as_output(2*32+31); /* GPIO_BOOTSEL1 for camera rest*/
	__gpio_set_pin(2*32+31);
	mdelay(5);
	__gpio_clear_pin(2*32+31);
	mdelay(5);
	__gpio_set_pin(2*32+31);
#else
	dprintk("=======%s:%d\n", __FUNCTION__, __LINE__);
#if defined(CONFIG_JZ4810_F4810) || defined(CONFIG_JZ4770_F4770) || defined(CONFIG_JZ4770_PISCES)
	//while(1)
	{
		__gpio_as_output0(GPIO_CAMERA_RST);
		mdelay(50);
		__gpio_as_output1(GPIO_CAMERA_RST);
		mdelay(50);
	}
#else
	__gpio_clear_pin(GPIO_CAMERA_RST);
	mdelay(50);
	__gpio_set_pin(GPIO_CAMERA_RST);
	mdelay(50);
#endif
#endif
}

int ov3640_set_balance(balance_flag_t balance_flag,int arg)
{
	dprintk("ov3640_set_balance");
	switch(balance_flag)
	{
		case WHITE_BALANCE_AUTO:
			ov3640_set_wb_auto_mode(ov3640_sensor_desc.client);
			dprintk("WHITE_BALANCE_AUTO");
			break;
		case WHITE_BALANCE_INCANDESCENT :
			dprintk("WHITE_BALANCE_INCANDESCENT ");
			break;
		case WHITE_BALANCE_FLUORESCENT ://ying guang
			dprintk("WHITE_BALANCE_FLUORESCENT ");
			break;
		case WHITE_BALANCE_WARM_FLUORESCENT :
			dprintk("WHITE_BALANCE_WARM_FLUORESCENT ");
			break;
		case WHITE_BALANCE_DAYLIGHT ://ri guang
			ov3640_set_wb_sunny_mode(ov3640_sensor_desc.client);
			dprintk("WHITE_BALANCE_DAYLIGHT ");
			break;
		case WHITE_BALANCE_CLOUDY_DAYLIGHT ://ying tian
			ov3640_set_wb_cloudy_mode(ov3640_sensor_desc.client);
			dprintk("WHITE_BALANCE_CLOUDY_DAYLIGHT ");
			break;
		case WHITE_BALANCE_TWILIGHT :
			dprintk("WHITE_BALANCE_TWILIGHT ");
			break;
		case WHITE_BALANCE_SHADE :
			dprintk("WHITE_BALANCE_SHADE ");
			break;
	}

	return 0;
}
int ov3640_set_antibanding(antibanding_flag_t antibanding_flag,int arg)
{
	dprintk("ov3640_set_antibanding");
	switch(antibanding_flag)
	{
		case ANTIBANDING_AUTO :
			ov3640_ab_auto(ov3640_sensor_desc.client);
			dprintk("ANTIBANDING_AUTO ");
			break;
		case ANTIBANDING_50HZ :
			ov3640_ab_50hz(ov3640_sensor_desc.client);
			dprintk("ANTIBANDING_50HZ ");
			break;
		case ANTIBANDING_60HZ :
			ov3640_ab_60hz(ov3640_sensor_desc.client);
			dprintk("ANTIBANDING_60HZ ");
			break;
		case ANTIBANDING_OFF :
			ov3640_ab_off(ov3640_sensor_desc.client);
			dprintk("ANTIBANDING_OFF ");
			break;
	}
	return 0;
}

int ov3640_set_flash_mode(flash_mode_flag_t flash_mode_flag,int arg)
{
	dprintk("ov3640_set_flash_mode");
	switch(flash_mode_flag)
	{
		case FLASH_MODE_OFF :
			dprintk("FLASH_MODE_OFF");
			break;
		case FLASH_MODE_AUTO :
			dprintk("FLASH_MODE_AUTO  ");
			break;
		case FLASH_MODE_ON :
			dprintk("FLASH_MODE_ON ");
			break;
		case FLASH_MODE_RED_EYE:
			dprintk("FLASH_MODE_RED_EYE ");
			break;
		case FLASH_MODE_TORCH:
			dprintk("FLASH_MODE_TORCH ");
			break;
	}
	return 0;

}

int ov3640_set_scene_mode(scene_mode_flag_t scene_mode_flag,int arg)
{
	dprintk("ov3640_set_scene_mode");
	switch(scene_mode_flag)
	{
		case SCENE_MODE_AUTO :
			dprintk("SCENE_MODE_AUTO ");
			break;
		case SCENE_MODE_ACTION :
			dprintk("SCENE_MODE_ACTION ");
			break;
		case SCENE_MODE_PORTRAIT   :
			dprintk("SCENE_MODE_PORTRAIT   ");
			break;
		case SCENE_MODE_LANDSCAPE  :
			dprintk("SCENE_MODE_LANDSCAPE  ");
			break;
		case SCENE_MODE_NIGHT     :
			dprintk("SCENE_MODE_NIGHT     ");
			break;
		case SCENE_MODE_NIGHT_PORTRAIT   :
			dprintk("SCENE_MODE_NIGHT_PORTRAIT   ");
			break;
		case SCENE_MODE_THEATRE  :
			dprintk("SCENE_MODE_THEATRE  ");
			break;
		case SCENE_MODE_BEACH   :
			dprintk("SCENE_MODE_BEACH   ");
			break;
		case SCENE_MODE_SNOW    :
			dprintk("SCENE_MODE_SNOW    ");
			break;
		case SCENE_MODE_SUNSET    :
			dprintk("SCENE_MODE_SUNSET    ");
			break;
		case SCENE_MODE_STEADYPHOTO   :
			dprintk("SCENE_MODE_STEADYPHOTO   ");
			break;
		case SCENE_MODE_FIREWORKS    :
			dprintk("SCENE_MODE_FIREWORKS    ");
			break;
		case SCENE_MODE_SPORTS    :
			dprintk("SCENE_MODE_SPORTS    ");
			break;
		case SCENE_MODE_PARTY   :
			dprintk("SCENE_MODE_PARTY   ");
			break;
		case SCENE_MODE_CANDLELIGHT    :
			dprintk("SCENE_MODE_CANDLELIGHT    ");
			break;
	}
	return 0;
	return 0;
}

int ov3640_set_focus_mode(focus_mode_flag_t flash_mode_flag,int arg)
{
	dprintk("ov3640_set_focus_mode");
	switch(flash_mode_flag)
	{
		case FOCUS_MODE_AUTO:
			dprintk("FOCUS_MODE_AUTO");
			break;
		case FOCUS_MODE_INFINITY:
			dprintk("FOCUS_MODE_INFINITY");
			break;
		case FOCUS_MODE_MACRO:
			dprintk("FOCUS_MODE_MACRO");
			break;
		case FOCUS_MODE_FIXED:
			dprintk("FOCUS_MODE_FIXED");
			break;
	}

	ov3640_set_af_mode(flash_mode_flag);
	return 0;
}

int ov3640_set_power(int state)
{
	dprintk("=======%s:%d\n", __FUNCTION__, __LINE__);
	switch (state)
	{
		case 0:
			/* hardware power up first */
			ov3640_power_up();
			/* software power up later if it implemented */
			break;
		case 1:
			ov3640_power_down();
			break;
		case 2:
			break;
		default:
			printk("%s : EINVAL! \n",__FUNCTION__);
	}
	return 0;
}

/* sensor_set_function use for init preview or capture.
 * there may be some difference between preview and capture.
 * so we divided it into two sequences.
 * param: function indicated which function
 * 0: preview
 * 1: capture
 * 2: recording
 */
int ov3640_set_effect(effect_flag_t effect_flag,int arg)
{
	dprintk("ov3640_set_effect");
	switch(effect_flag)
	{
		case EFFECT_NONE:
			ov3640_set_effect_normal(ov3640_sensor_desc.client);
			dprintk("EFFECT_NONE");
			break;
		case EFFECT_MONO :
			ov3640_set_effect_black_white(ov3640_sensor_desc.client);
			dprintk("EFFECT_MONO ");
			break;
		case EFFECT_NEGATIVE :
			ov3640_set_effect_negative(ov3640_sensor_desc.client);
			dprintk("EFFECT_NEGATIVE ");
			break;
		case EFFECT_SOLARIZE ://bao guang
			dprintk("EFFECT_SOLARIZE ");
			break;
		case EFFECT_SEPIA :
			ov3640_set_effect_sepia(ov3640_sensor_desc.client);
			dprintk("EFFECT_SEPIA ");
			break;
		case EFFECT_POSTERIZE ://se diao fen li
			dprintk("EFFECT_POSTERIZE ");
			break;
		case EFFECT_WHITEBOARD :
			dprintk("EFFECT_WHITEBOARD ");
			break;
		case EFFECT_BLACKBOARD :
			dprintk("EFFECT_BLACKBOARD ");
			break;
		case EFFECT_AQUA  ://qian lv se
			ov3640_set_effect_greenish(ov3640_sensor_desc.client);
			dprintk("EFFECT_AQUA  ");
			break;
		case EFFECT_PASTEL:
			dprintk("EFFECT_PASTEL");
			break;
		case EFFECT_MOSAIC:
			dprintk("EFFECT_MOSAIC");
			break;
		case EFFECT_RESIZE:
			dprintk("EFFECT_RESIZE");
			break;
	}

	return 0;
}


int ov3640_set_output_format(pixel_format_flag_t pixel_format_flag,int arg)
{
	dprintk("=======%s:%d\n", __FUNCTION__, __LINE__);
	switch(pixel_format_flag)
	{
		case PIXEL_FORMAT_JPEG:
			printk("ov3640 set output format to jepg");
			break;
		case PIXEL_FORMAT_YUV422SP:
			printk("ov3640 set output format to yuv422sp");
			break;
		case PIXEL_FORMAT_YUV420SP:
			printk("ov3640 set output format to yuv420sp");
			break;
		case PIXEL_FORMAT_YUV422I:
			printk("ov3640 set output format to yuv422i");
			break;
		case PIXEL_FORMAT_RGB565:
			printk("ov3640 set output format to rgb565");
			break;
	}
	return 0;
}

#if 0
int ov3640_af_init(struct sensor_af_arg *info)
{
	unsigned short *reg  = info->buf;
	unsigned char *value = info->buf+2;
	dprintk("=======%s:%d\n", __FUNCTION__, __LINE__);
	if(info->len % 3 != 0)
		return -1;
	while(value <= info->buf+info->len)
	{
		//printk("--------------------------------%4x %2x\n",*reg,*value);
		sensor_write_reg16(ov3640_sensor_desc.client,*reg,*value);
		reg =(unsigned short*)((unsigned char *)reg + 3);
		value += 3;
	}

//	sensor_write_reg16(ov3640_sensor_desc.client,0x3f00,0x01);//focus overlay
	return 0;
}
#endif

int ov3640_set_function(int function,void *cookie)
{
	dprintk("=======%s:%d\n", __FUNCTION__, __LINE__);
	switch (function)
	{
	case 0:
		preview_set(ov3640_sensor_desc.client);
		break;
	case 1:
		capture_set(ov3640_sensor_desc.client);
		break;
	case 2:
		break;
	case 3:
		dprintk("---- do focus ---");
		return ov3640_do_focus(ov3640_sensor_desc.client);
	case 4:
		ov3640_af_init(ov3640_sensor_desc.client,cookie);
		break;
	case 5:
		ov3640_stop_focus(ov3640_sensor_desc.client);
		break;
	}

	return 0;
}

int ov3640_set_fps(int fps)
{
	dprintk("=======%s:%d\n", __FUNCTION__, __LINE__);
	dprintk("set fps : %d",fps);
	return 0;
}

int ov3640_set_night_mode(int enable)
{
	dprintk("=======%s:%d\n", __FUNCTION__, __LINE__);
	if(enable)
		dprintk("nightshot_mode enable!");
	else
		dprintk("nightshot_mode disable!");

	__ov3640_set_night_mode(ov3640_sensor_desc.client,enable);

	return 0;
}

int ov3640_set_luma_adaptation(int arg)
{
	dprintk("luma_adaptation : %d",arg);
	return 0;
}

int ov3640_set_parameter(int cmd, int mode, int arg)
{
	dprintk("=======%s:%d\n", __FUNCTION__, __LINE__);
	switch(cmd)
	{
		case CPCMD_SET_BALANCE :
    			ov3640_set_balance(mode,arg);
			break;
		case CPCMD_SET_EFFECT :
    			ov3640_set_effect(mode,arg);
			break;
		case CPCMD_SET_ANTIBANDING :
    			ov3640_set_antibanding(mode,arg);
			break;
		case CPCMD_SET_FLASH_MODE :
    			ov3640_set_flash_mode(mode,arg);
			break;
		case CPCMD_SET_SCENE_MODE :
   			ov3640_set_scene_mode(mode,arg);
			break;
		case CPCMD_SET_PIXEL_FORMAT :
    			ov3640_set_output_format(mode,arg);
			break;
		case CPCMD_SET_FOCUS_MODE :
    			ov3640_set_focus_mode(mode,arg);
			break;
		case CPCMD_SET_PREVIEW_FPS:
    			ov3640_set_fps(arg);
			break;
		case CPCMD_SET_NIGHTSHOT_MODE:
			ov3640_set_night_mode(arg);
			break;
		case CPCMD_SET_LUMA_ADAPTATION:
			ov3640_set_luma_adaptation(arg);
			break;
	}
	return 0;
}

int ov3640_sensor_init(void)
{
	dprintk("=======%s:%d\n", __FUNCTION__, __LINE__);
	ov3640_set_mclk(0);
	ov3640_reset();

	init_set(ov3640_sensor_desc.client);
	return 0;
}


int ov3640_sensor_probe(void)
{
	int retval = 0;
	ov3640_power_up();
	ov3640_reset();
	mdelay(10);
	retval=sensor_read_reg16(ov3640_sensor_desc.client,0x300a);
	ov3640_power_down();

	if(retval == 0x36)//read id,ov3640 id is 0x36
		return 0;
	return -1;
}



int ov3640_set_resolution(int width,int height,int bpp,pixel_format_flag_t fmt,camera_mode_t mode)
{
	dprintk("ov3640_set_resolution %d x %d",width,height);
	size_switch(ov3640_sensor_desc.client,width,height,mode);
	return 0;
}


static int ov3640_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	ov3640_sensor_desc.client = client;
#ifndef CONFIG_JZ4810_F4810
	sensor_set_i2c_speed(client,400000);// set ov3640 i2c speed : 400khz
#else
	sensor_set_i2c_speed(client,5000);// F4760: set ov3640 i2c speed : 5khz
#endif
	camera_sensor_register(&ov3640_sensor_desc);

	return 0;
}

struct camera_sensor_ops ov3640_sensor_ops = {
	.sensor_init = ov3640_sensor_init,
	.sensor_set_function =  ov3640_set_function,
	.sensor_set_resolution = ov3640_set_resolution,
	.sensor_set_parameter = ov3640_set_parameter,
	.sensor_set_power = ov3640_set_power,

	.camera_sensor_probe = ov3640_sensor_probe,
};


struct resolution_info ov3640_resolution_table[] = {
	//{2048,1536,16,PIXEL_FORMAT_YUV422I},
	{1600,1200,16,PIXEL_FORMAT_YUV422I},
	{1280,1024,16,PIXEL_FORMAT_YUV422I},
	{1024,768,16,PIXEL_FORMAT_YUV422I},
	{800,600,16,PIXEL_FORMAT_YUV422I},
	{800,480,16,PIXEL_FORMAT_YUV422I},
	{640,480,16,PIXEL_FORMAT_YUV422I},
	{480,320,16,PIXEL_FORMAT_YUV422I},
	{352,288,16,PIXEL_FORMAT_YUV422I},
	{320,240,16,PIXEL_FORMAT_YUV422I},
	{176,144,16,PIXEL_FORMAT_YUV422I},
};


struct camera_sensor_desc ov3640_sensor_desc = {
	.name = "ov3640",
	.camera_clock = CAM_CLOCK,
	.wait_frames = 2,
	.client = NULL,

	.ops = &ov3640_sensor_ops,

	.resolution_table = ov3640_resolution_table,
	.resolution_table_nr=ARRAY_SIZE(ov3640_resolution_table),

#if 0
	.capture_parm = {2048, 1536, 16,PIXEL_FORMAT_YUV422I},
	.max_capture_parm = {2048, 1536, 16,PIXEL_FORMAT_YUV422I},

	.preview_parm = {1024,768, 16,PIXEL_FORMAT_YUV422I},
	.max_preview_parm = {1024,768, 16,PIXEL_FORMAT_YUV422I},
#else
#if 0
	.capture_parm = {1600, 1200, 16,PIXEL_FORMAT_YUV422I},
	.max_capture_parm = {1600, 1200, 16,PIXEL_FORMAT_YUV422I},

	.preview_parm = {1024, 768, 16,PIXEL_FORMAT_YUV422I},
	.max_preview_parm = {1024 , 768, 16,PIXEL_FORMAT_YUV422I},
#else
	.capture_parm = {800, 600, 16,PIXEL_FORMAT_YUV422I},
	.max_capture_parm = {800, 600, 16,PIXEL_FORMAT_YUV422I},

	.preview_parm = {640, 480, 16,PIXEL_FORMAT_YUV422I},
	.max_preview_parm = {640 , 480, 16,PIXEL_FORMAT_YUV422I},
#endif
#endif

	.cfg_info = {
		.configure_register= 0x0
			|CIM_CFG_PACK_2			/* pack mode : 3 2 1 4*/
			|CIM_CFG_BYPASS			/* Bypass Mode */
			//|CIM_CFG_VSP             	/* VSYNC Polarity:1-falling edge active */
			//|CIM_CFG_PCP             	/* PCLK working edge:1-falling */
			|CIM_CFG_DSM_GCM,		/* Gated Clock Mode */
	},

	.flags = {
		.focus_mode_flag = ~0x0,//FOCUS_MODE_INFINITY | FOCUS_MODE_AUTO,
		//.scene_mode_flag = ~0x0,
		.antibanding_flag = ~0x0,

		.effect_flag =		EFFECT_NONE
		       			|EFFECT_MONO
	       				|EFFECT_NEGATIVE
					|EFFECT_SEPIA
					|EFFECT_AQUA,

		.balance_flag = WHITE_BALANCE_AUTO | WHITE_BALANCE_CLOUDY_DAYLIGHT | WHITE_BALANCE_DAYLIGHT,
	},
};


static const struct i2c_device_id ov3640_id[] = {
	{ "ov3640", 0 },
	{ }	/* Terminating entry */
};
MODULE_DEVICE_TABLE(i2c, ov3640_id);

static struct i2c_driver ov3640_driver = {
	.probe		= ov3640_probe,
	.id_table	= ov3640_id,
	.driver	= {
		.name = "ov3640",
	},
};

static int __init ov3640_register(void)
{
	return i2c_add_driver(&ov3640_driver);
}

module_init(ov3640_register);




