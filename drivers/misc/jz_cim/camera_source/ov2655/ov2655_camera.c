#include <asm/jzsoc.h>
#include <linux/i2c.h>
#include "ov2655_camera.h"
#include "ov2655_set.h"
#include "ov2655_set_mode.h"
#include <asm/jzsoc.h>

#define ov2655_DEBUG
#ifdef ov2655_DEBUG
#define dprintk(x...)   do{printk("ov2655---\t");printk(x);printk("\n");}while(0)
#else
#define dprintk(x...)
#endif

/* gpio init */
#if defined(CONFIG_JZ4760_PT701)
#define GPIO_CAMERA_RST         (32*3+10) /*GPD10*/
#define GPIO_CAMERA_PD         (32*3+11) /*GPD11*/
#elif defined(CONFIG_JZ4760_LEPUS)
#define GPIO_CAMERA_RST         (32 * 1 + 26)   /* GPB26 */
#define GPIO_CAMERA_PD         (32 * 1 + 27) /* GPB27 */
#elif defined(CONFIG_JZ4770_PISCES)
#define GPIO_CAMERA_RST		GPE(3)
#define GPIO_CAMERA_PD		GPE(4)
#else
#error "ov2655/ov2655_camera.c , please define camera for your board."
#endif


struct ov2655_sensor ov2655;

void ov2655_power_down(void)
{
#if defined(CONFIG_SOC_JZ4770)
	__gpio_as_output1(GPIO_CAMERA_PD);
#else
	__gpio_as_output(GPIO_CAMERA_PD);
	__gpio_set_pin(GPIO_CAMERA_PD);
#endif
	mdelay(5);
}

void ov2655_power_up(void)
{
	//ov2655_power_on later
#if defined(CONFIG_SOC_JZ4770)
	__gpio_as_output0(GPIO_CAMERA_PD);
#else
	__gpio_as_output(GPIO_CAMERA_PD);
	__gpio_clear_pin(GPIO_CAMERA_PD);
#endif
	mdelay(5);
}

void ov2655_reset(void)
{
#if defined(CONFIG_SOC_JZ4770)
	__gpio_as_output0(GPIO_CAMERA_RST);
	mdelay(50);
	__gpio_as_output1(GPIO_CAMERA_RST);
#else
	__gpio_as_output(GPIO_CAMERA_RST);
	mdelay(50);
	__gpio_clear_pin(GPIO_CAMERA_RST);
	mdelay(50);
	__gpio_set_pin(GPIO_CAMERA_RST);
#endif
	mdelay(50);
}


int ov2655_set_balance(balance_flag_t balance_flag,int arg)
{
	dprintk("ov2655_set_balance");
	switch(balance_flag)
	{
		case WHITE_BALANCE_AUTO:
			ov2655_set_wb_auto_mode(ov2655.client);
			dprintk("wb_auto ");
			break;
		case WHITE_BALANCE_DAYLIGHT ://ri guang
			ov2655_set_wb_sunny_mode(ov2655.client);
			dprintk("wb_daylight ");
			break;
		case WHITE_BALANCE_CLOUDY_DAYLIGHT ://ying tian
			ov2655_set_wb_cloudy_mode(ov2655.client);
			dprintk("wb_cloudy daylight ");
			break;
		case WHITE_BALANCE_INCANDESCENT :
			ov2655_set_wb_office_mode(ov2655.client);
			dprintk("wb_incandenscent ");
			break;
	}
	return 0;
}

int ov2655_set_effect(effect_flag_t effect_flag,int arg)
{
	dprintk("ov2655_set_effect");
	switch(effect_flag)
	{
		case EFFECT_NONE:
			ov2655_set_effect_normal(ov2655.client);
			dprintk("effect_none");
			break;
		case EFFECT_MONO :
			ov2655_set_effect_blackwhite(ov2655.client);
			dprintk("effect_mono ");
			break;
		case EFFECT_NEGATIVE :
			ov2655_set_effect_negative(ov2655.client);
			dprintk("effect_negative ");
			break;
		case EFFECT_SOLARIZE ://bao guang
			dprintk("effect_solarize ");
			break;
		case EFFECT_SEPIA :
			ov2655_set_effect_sepia(ov2655.client);
			dprintk("effect_sepia ");
			break;
		case EFFECT_POSTERIZE ://se diao fen li
			dprintk("effect_posterize ");
			break;
		case EFFECT_WHITEBOARD :
			dprintk("effect_whiteboard ");
			break;
		case EFFECT_BLACKBOARD :
			dprintk("effect_blackboard ");
			break;
		case EFFECT_AQUA ://qian lv se
			ov2655_set_effect_greenish(ov2655.client);
			dprintk("effect_aqua  ");
			break;
		case EFFECT_PASTEL:
			dprintk("effect_pastel");
			break;
		case EFFECT_MOSAIC:
			dprintk("effect_mosaic");
			break;
		case EFFECT_RESIZE:
			dprintk("effect_resize");
			break;
	}
	return 0;
}

int ov2655_set_antibanding(antibanding_flag_t antibanding_flag,int arg)
{
	dprintk("ov2655_set_antibanding");
	switch(antibanding_flag)
	{
		case ANTIBANDING_AUTO :
			ov2655_ab_auto(ov2655.client);
			dprintk("ANTIBANDING_AUTO ");
			break;
		case ANTIBANDING_50HZ :
			ov2655_ab_50hz(ov2655.client);
			dprintk("ANTIBANDING_50HZ ");
			break;
		case ANTIBANDING_60HZ :
			ov2655_ab_60hz(ov2655.client);
			dprintk("ANTIBANDING_60HZ ");
			break;
		case ANTIBANDING_OFF :
			ov2655_ab_off(ov2655.client);
			dprintk("ANTIBANDING_OFF ");
			break;
	}
	return 0;
}


int ov2655_set_flash_mode(flash_mode_flag_t flash_mode_flag,int arg)
{
	return 0;
}

int ov2655_set_scene_mode(scene_mode_flag_t scene_mode_flag,int arg)
{
	return 0;
}

int ov2655_set_focus_mode(focus_mode_flag_t flash_mode_flag,int arg)
{
	return 0;
}


int ov2655_set_fps(int fps)
{
	dprintk("set fps : %d",fps);
	return 0;
}

int ov2655_set_luma_adaptation(int arg)
{
	dprintk("luma_adaptation : %d",arg);
	return 0;
}

int ov2655_set_parameter(int cmd, int mode, int arg)
{
	switch(cmd)
	{
		case CPCMD_SET_BALANCE :
			ov2655_set_balance(mode,arg);
			break;
		case CPCMD_SET_EFFECT :
			ov2655_set_effect(mode,arg);
			break;
		case CPCMD_SET_ANTIBANDING :
			ov2655_set_antibanding(mode,arg);
			break;
		case CPCMD_SET_FLASH_MODE :
			ov2655_set_flash_mode(mode,arg);
			break;
		case CPCMD_SET_SCENE_MODE :
			ov2655_set_scene_mode(mode,arg);
			break;
		case CPCMD_SET_PIXEL_FORMAT :
			break;
		case CPCMD_SET_FOCUS_MODE :
			ov2655_set_focus_mode(mode,arg);
			break;
		case CPCMD_SET_PREVIEW_FPS:
			ov2655_set_fps(arg);
			break;
		case CPCMD_SET_NIGHTSHOT_MODE:
			break;
		case CPCMD_SET_LUMA_ADAPTATION:
			ov2655_set_luma_adaptation(arg);
			break;
	}
	return 0;
}

int ov2655_set_power(int state)
{
	switch (state)
	{
		case 0:
			/* hardware power up first */
			ov2655_power_up();
			/* software power up later if it implemented */
			break;
		case 1:
			ov2655_power_down();
			break;
		case 2:
			break;
		default:
			printk("%s : EINVAL! \n",__FUNCTION__);
	}
	return 0;
}

int ov2655_sensor_init(void)
{
	//ov2655_reset();
	ov2655_init_setting(ov2655.client);
	return 0;
}

int ov2655_sensor_probe(void)
{
	ov2655_power_up();
	ov2655_reset();

	int sensor_id = sensor_read_reg16(ov2655.client,0x300a);
	ov2655_power_down();

	if(sensor_id == 0x26)
		return 0;
	printk("ov2655 probe error : id = %x",sensor_id);
	return -1;
}

/* sensor_set_function use for init preview or capture.there may be some difference between preview and capture.
 * so we divided it into two sequences.param: function indicated which function
 * 0: preview
 * 1: capture
 * 2: recording
 */
int ov2655_set_function(int function)
{
	switch (function)
	{
		case 0:
			preview_set(ov2655.client);
			break;
		case 1:
			capture_set(ov2655.client);
			break;
		case 2:
			break;
	}
	return 0;
}

int ov2655_set_resolution(int width,int height,int bpp,pixel_format_flag_t fmt,camera_mode_t mode)
{
	size_switch(ov2655.client,width,height,mode);
	return 0;
}

static int ov2655_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	ov2655.client = client;
	return camera_sensor_register(&ov2655.desc);
}

struct camera_sensor_ops ov2655_sensor_ops = {
	.sensor_init = ov2655_sensor_init,
	.camera_sensor_probe = ov2655_sensor_probe,
	.sensor_set_function = ov2655_set_function,
	.sensor_set_resolution = ov2655_set_resolution,
	.sensor_set_parameter = ov2655_set_parameter,
	.sensor_set_power = ov2655_set_power,
};

struct resolution_info ov2655_resolution_table[] = {
	//{1600,1200,16,PIXEL_FORMAT_YUV422I},
	//{1280,1024,16,PIXEL_FORMAT_YUV422I},
	{1024,768,16,PIXEL_FORMAT_YUV422I},
	{800,600,16,PIXEL_FORMAT_YUV422I},
        {640,480,16,PIXEL_FORMAT_YUV422I},
	{352,288,16,PIXEL_FORMAT_YUV422I},  //must be included
	{176,144,16,PIXEL_FORMAT_YUV422I},  //must be included in order to recording

};

struct ov2655_sensor ov2655 = {
	.desc = {
		.name = "ov2655",
		.wait_frames = 2,

		.ops = &ov2655_sensor_ops,

		.resolution_table = ov2655_resolution_table,
		.resolution_table_nr=ARRAY_SIZE(ov2655_resolution_table),

		.capture_parm = {640,480, 16,PIXEL_FORMAT_YUV422I},
		.max_capture_parm = {1600,1200, 16,PIXEL_FORMAT_YUV422I},

		.preview_parm = {800,600, 16,PIXEL_FORMAT_YUV422I},
		.max_preview_parm = {800,600, 16,PIXEL_FORMAT_YUV422I},
		.bus_width = 10, /* this also means 10bit per pixel, from the buffer's view, it's 16bpp(upper 6bit are not used) */

		.cfg_info = {
			.configure_register= 0x0
				|CIM_CFG_PACK_3			/* pack mode : 4 3 2 1 */
				|CIM_CFG_BYPASS			/* Bypass Mode */
				|CIM_CFG_VSP             	/* VSYNC Polarity:1-falling edge active */
                	//      |CIM_CFG_HSP
				|CIM_CFG_PCP             	/* PCLK working edge:1-falling */
				|CIM_CFG_DSM_GCM,		/* Gated Clock Mode */
		},

		.flags = {
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
			.antibanding_flag = ~0x0,
			.flash_mode_flag = 0,
			.scene_mode_flag = 0,
			.pixel_format_flag = 0,
			.focus_mode_flag = 0,
		},

	},
};

static const struct i2c_device_id ov2655_id[] = {
	{ "ov2655", 0 },
	{ }	/* Terminating entry */
};
MODULE_DEVICE_TABLE(i2c, ov2655_id);

static struct i2c_driver ov2655_driver = {
	.probe		= ov2655_i2c_probe,
	.id_table	= ov2655_id,
	.driver	= {
		.name = "ov2655",
	},
};

static int __init ov2655_i2c_register(void)
{
	return i2c_add_driver(&ov2655_driver);
}

module_init(ov2655_i2c_register);
