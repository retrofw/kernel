#include <asm/jzsoc.h>
#include <linux/i2c.h>
#include "cm3511_camera.h"
#include "cm3511_set.h"

#define cm3511_DEBUG
//#undef DEBUG

#ifdef cm3511_DEBUG
#define dprintk(x...)   do{printk("cm3511---\t");printk(x);printk("\n");}while(0)
#else
#define dprintk(x...)
#endif

/* gpio init */
#if defined(CONFIG_JZ4750_APUS) || defined(CONFIG_JZ4750D_FUWA1) || defined(CONFIG_JZ4750_AQUILA)/* board APUS */
#define GPIO_CAMERA_RST         (32*4+8)
#elif defined(CONFIG_JZ4760_ALTAIR)
#define GPIO_CAMERA_RST         (32*4+13) /*GPE13*/
#elif defined(CONFIG_JZ4760_LEPUS)
#define GPIO_CAMERA_RST         (32*1 + 26) /* GPB26 */
#else
#error "cm3511/cm3511_camera.c , please define camera for your board."
#endif


struct cm3511_sensor cm3511;

void cm3511_power_down(void)
{
#if defined(CONFIG_JZ4750_AQUILA)
	__gpio_as_output(4*32+23);
	__gpio_clear_pin(4*32+23);
#elif defined(CONFIG_JZ4760_ALTAIR)
	__gpio_as_output(0*32+27);
	__gpio_clear_pin(0*32+27);
#elif defined(CONFIG_JZ4760_LEPUS)
	__gpio_as_output(32 * 1 + 27); /* GPB27 */
	__gpio_clear_pin(32 * 1 + 27);
#endif
	mdelay(5);
}

void cm3511_power_up(void)
{
#if defined(CONFIG_JZ4750_AQUILA)
	__gpio_as_output(4*32+23);
	__gpio_set_pin(4*32+23);
#elif defined(CONFIG_JZ4760_ALTAIR)
	__gpio_as_output(0*32+27);                                    /* GPA27 */
	__gpio_set_pin(0*32+27);
#elif defined(CONFIG_JZ4760_LEPUS)
	__gpio_as_output(32 * 1 + 27); /* GPB27 */
	__gpio_set_pin(32 * 1 + 27);
#endif
	mdelay(5);
}

int cm3511_set_mclk(unsigned int mclk)
{
	return 0;//in this board use the extra osc - -20MHZ
}

void cm3511_reset(void)
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
	__gpio_as_output(GPIO_CAMERA_RST);
	mdelay(50);
	__gpio_clear_pin(GPIO_CAMERA_RST);
	mdelay(50);
	__gpio_set_pin(GPIO_CAMERA_RST);
	mdelay(50);
#endif
}

int cm3511_set_balance(balance_flag_t balance_flag,int arg)
{
	dprintk("cm3511_set_balance");
	switch(balance_flag)
	{
		case WHITE_BALANCE_AUTO:
			cm3511_set_wb_auto(cm3511.client);
			dprintk("WHITE_BALANCE_AUTO");
			break;
		case WHITE_BALANCE_INCANDESCENT :
			cm3511_set_wb_incandescence(cm3511.client);
			dprintk("WHITE_BALANCE_INCANDESCENT ");
			break;
		case WHITE_BALANCE_FLUORESCENT ://ying guang
			cm3511_set_wb_fluorescent(cm3511.client);
			dprintk("WHITE_BALANCE_FLUORESCENT ");
			break;
		case WHITE_BALANCE_WARM_FLUORESCENT :
			dprintk("WHITE_BALANCE_WARM_FLUORESCENT ");
			break;
		case WHITE_BALANCE_DAYLIGHT ://ri guang
			cm3511_set_wb_daylight(cm3511.client);
			dprintk("WHITE_BALANCE_DAYLIGHT ");
			break;
		case WHITE_BALANCE_CLOUDY_DAYLIGHT ://ying tian
			cm3511_set_wb_cloud(cm3511.client);
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

int cm3511_set_antibanding(antibanding_flag_t antibanding_flag,int arg)
{
	dprintk("cm3511_set_antibanding");
	switch(antibanding_flag)
	{
		case ANTIBANDING_AUTO :
			dprintk("ANTIBANDING_AUTO ");
			break;
		case ANTIBANDING_50HZ :
			cm3511_set_ab_50hz(cm3511.client);
			dprintk("ANTIBANDING_50HZ ");
			break;
		case ANTIBANDING_60HZ :
			cm3511_set_ab_60hz(cm3511.client);
			dprintk("ANTIBANDING_60HZ ");
			break;
		case ANTIBANDING_OFF :
			dprintk("ANTIBANDING_OFF ");
			break;
	}
	return 0;
}

int cm3511_set_flash_mode(flash_mode_flag_t flash_mode_flag,int arg)
{
	dprintk("cm3511_set_flash_mode");
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

int cm3511_set_scene_mode(scene_mode_flag_t scene_mode_flag,int arg)
{
	dprintk("cm3511_set_scene_mode");
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
}

int cm3511_set_focus_mode(focus_mode_flag_t flash_mode_flag,int arg)
{
	dprintk("cm3511_set_focus_mode");
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

	return 0;
}

int cm3511_set_power(int state)
{
	switch (state)
	{
		case 0:
			/* hardware power up first */
			cm3511_power_up();
			/* software power up later if it implemented */
			break;
		case 1:
			cm3511_power_down();
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
int cm3511_set_effect(effect_flag_t effect_flag,int arg)
{
	dprintk("cm3511_set_effect");
	switch(effect_flag)
	{
		case EFFECT_NONE:
			cm3511_set_effect_normal(cm3511.client);
			dprintk("EFFECT_NONE");
			break;
		case EFFECT_MONO :
			cm3511_set_effect_grayscale(cm3511.client);
			dprintk("EFFECT_MONO ");
			break;
		case EFFECT_NEGATIVE :
			cm3511_set_effect_colorinv(cm3511.client);
			dprintk("EFFECT_NEGATIVE ");
			break;
		case EFFECT_SOLARIZE ://bao guang
			dprintk("EFFECT_SOLARIZE ");
			break;
		case EFFECT_SEPIA :
			cm3511_set_effect_sepia(cm3511.client);
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
			cm3511_set_effect_sepiagreen(cm3511.client);
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

int cm3511_set_output_format(pixel_format_flag_t pixel_format_flag,int arg)
{
	switch(pixel_format_flag)
	{
		case PIXEL_FORMAT_JPEG:
			printk("cm3511 set output format to jepg");
			break;
		case PIXEL_FORMAT_YUV422SP:
			printk("cm3511 set output format to yuv422sp");
			break;
		case PIXEL_FORMAT_YUV420SP:
			printk("cm3511 set output format to yuv420sp");
			break;
		case PIXEL_FORMAT_YUV422I:
			printk("cm3511 set output format to yuv422i");
			break;
		case PIXEL_FORMAT_RGB565:
			printk("cm3511 set output format to rgb565");
			break;
	}
	return 0;
}

int cm3511_set_function(int function)
{
	switch (function)
	{
		case 0:
			cm3511_set_preview_mode(cm3511.client);
			break;
		case 1:
			cm3511_set_capture_mode(cm3511.client);
			break;
		case 2:
			break;
		case 3:
			dprintk("---- do focus ---");
			break;
	}

	return 0;
}

int cm3511_set_fps(int fps)
{
	dprintk("set fps : %d",fps);
	return 0;
}

int cm3511_set_night_mode(int enable)
{
	if(enable)
		dprintk("nightshot_mode enable!");
	else
		dprintk("nightshot_mode disable!");

	cm3511_set_nightmode(cm3511.client,enable);
	return 0;
}

int cm3511_set_luma_adaptation(int arg)
{
	dprintk("luma_adaptation : %d",arg);
	return 0;
}

int cm3511_set_parameter(int cmd, int mode, int arg)
{
	switch(cmd)
	{
		case CPCMD_SET_BALANCE :
			cm3511_set_balance(mode,arg);
			break;
		case CPCMD_SET_EFFECT :
			cm3511_set_effect(mode,arg);
			break;
		case CPCMD_SET_ANTIBANDING :
			cm3511_set_antibanding(mode,arg);
			break;
		case CPCMD_SET_FLASH_MODE :
			cm3511_set_flash_mode(mode,arg);
			break;
		case CPCMD_SET_SCENE_MODE :
			cm3511_set_scene_mode(mode,arg);
			break;
		case CPCMD_SET_PIXEL_FORMAT :
			cm3511_set_output_format(mode,arg);
			break;
		case CPCMD_SET_FOCUS_MODE :
			cm3511_set_focus_mode(mode,arg);
			break;
		case CPCMD_SET_PREVIEW_FPS:
			cm3511_set_fps(arg);
			break;
		case CPCMD_SET_NIGHTSHOT_MODE:
			cm3511_set_night_mode(arg);
			break;
		case CPCMD_SET_LUMA_ADAPTATION:
			cm3511_set_luma_adaptation(arg);
			break;
	}
	return 0;
}

int cm3511_sensor_init(void)
{
	cm3511_set_mclk(0);
	cm3511_reset();

	sensor_write_reg(cm3511.client,0x03, 0x00);//
	sensor_write_reg(cm3511.client,0x01, 0xC1);
	sensor_write_reg(cm3511.client,0x01, 0xC3);//
	sensor_write_reg(cm3511.client,0x01, 0xC1);

	mdelay(50);
	sensor_write_reg(cm3511.client,0x03, 0x00);//

	cm3511_init_setting(cm3511.client);
	return 0;
}

int cm3511_sensor_probe(void)
{
	int sensor_id=0;
	cm3511_power_up();
	cm3511_reset();

	sensor_write_reg(cm3511.client,0x03, 0x00);//
	sensor_write_reg(cm3511.client,0x01, 0xC1);
	sensor_write_reg(cm3511.client,0x01, 0xC3);//
	sensor_write_reg(cm3511.client,0x01, 0xC1);

	mdelay(50);
	sensor_write_reg(cm3511.client,0x03, 0x00);//
	sensor_id = sensor_read_reg(cm3511.client,0x04);

	cm3511_power_down();
	if(sensor_id == 0x81)
		return 0;

	dprintk("sensor read is %d",sensor_id);
	return -1;
}

int cm3511_set_resolution(int width,int height,int bpp,pixel_format_flag_t fmt,camera_mode_t mode)
{
	return 0;
}

struct camera_sensor_ops cm3511_sensor_ops = {
	.sensor_init = 			cm3511_sensor_init,
	.camera_sensor_probe = 		cm3511_sensor_probe,
	.sensor_set_function =  	cm3511_set_function,
	.sensor_set_resolution = 	cm3511_set_resolution,
	.sensor_set_parameter = 	cm3511_set_parameter,
	.sensor_set_power = 		cm3511_set_power,
};

struct resolution_info cm3511_resolution_table[] = {
	{640,480,16,PIXEL_FORMAT_YUV422I},
//	{352,288,16,PIXEL_FORMAT_YUV422I},
};

struct cm3511_sensor cm3511 = {
	.desc = {
		.name = "cm3511",
		.wait_frames = 4,

		.ops = &cm3511_sensor_ops,

		.resolution_table = cm3511_resolution_table,
		.resolution_table_nr=ARRAY_SIZE(cm3511_resolution_table),

		.capture_parm = {640,480, 16,PIXEL_FORMAT_YUV422I},
		.max_capture_parm = {640,480, 16,PIXEL_FORMAT_YUV422I},

		.preview_parm = {640,480, 16,PIXEL_FORMAT_YUV422I},
		.max_preview_parm = {640,480, 16,PIXEL_FORMAT_YUV422I},

		.cfg_info = {
			.configure_register= 0x0
				|CIM_CFG_PACK_4			/* pack mode : 4 3 2 1 */
				|CIM_CFG_BYPASS			/* Bypass Mode */
			//	|CIM_CFG_VSP             	/* VSYNC Polarity:1-falling edge active */
			//	|CIM_CFG_PCP             	/* PCLK working edge:1-falling */
				|CIM_CFG_DSM_GCM,		/* Gated Clock Mode */
		},

		.flags = {
			.pixel_format_flag = PIXEL_FORMAT_YUV422SP,

			.scene_mode_flag = ~0,

			.effect_flag = 0
				|EFFECT_NONE
				|EFFECT_MONO
				|EFFECT_SEPIA
				|EFFECT_NEGATIVE
				|EFFECT_AQUA,

			.balance_flag = 0
				| WHITE_BALANCE_AUTO
				| WHITE_BALANCE_DAYLIGHT
				| WHITE_BALANCE_CLOUDY_DAYLIGHT
				| WHITE_BALANCE_INCANDESCENT
				| WHITE_BALANCE_FLUORESCENT,

			.antibanding_flag = 0
				|ANTIBANDING_50HZ
				|ANTIBANDING_60HZ,
		},

	},
};

static int cm3511_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	dprintk("cm3511_i2c_probe!\n");
	cm3511.client = client;
	return camera_sensor_register(&cm3511.desc);
}

static const struct i2c_device_id cm3511_id[] = {
	{ "cm3511", 0 },
	{ }	/* Terminating entry */
};
MODULE_DEVICE_TABLE(i2c, cm3511_id);

static struct i2c_driver cm3511_driver = {
	.probe		= cm3511_i2c_probe,
	.id_table	= cm3511_id,
	.driver	= {
		.name = "cm3511",
	},
};

static int __init cm3511_i2c_register(void)
{
	return i2c_add_driver(&cm3511_driver);
}

module_init(cm3511_i2c_register);


