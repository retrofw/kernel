
#include <asm/jzsoc.h>

#include "ov7690_set.h"
#include "ov7690_camera.h"
#include <linux/i2c.h>
#include "../../jz_cim_core.h"
#include "../../jz_sensor.h"

#define OV7690_DEBUG
//#undef DEBUG

#ifdef OV7690_DEBUG
#define dprintk(x...)   do{printk("OV7690---\t");printk(x);printk("\n");}while(0)
#else
#define dprintk(x...)
#endif

/* define reset and powerdown pin */
#if defined(CONFIG_JZ4750_AQUILA)
#define OV7690_RESET_PIN
#define OV7690_PD_PIN       (4*32+8)                  //GPE8
#elif defined(CONFIG_JZ4760_ALTAIR)
#define OV7690_RESET_PIN
#define OV7690_PD_PIN 		(1*32+9)		//GPB9
#elif defined(CONFIG_JZ4760_LEPUS)
#define OV7690_RESET_PIN	(32 * 1 + 26)   /* GPB26 */
#define OV7690_PD_PIN 		(32 * 1 + 27) /* GPB27 */
#elif defined(CONFIG_JZ4750_APUS)
#define OV7690_RESET_PIN	(32 * 4 + 16)   /* GPe16 */
#define OV7690_PD_PIN 		(32 * 4 + 17) /* GPe17 */
#endif


struct camera_sensor_desc ov7690_sensor_desc;
struct resolution_info ov7690_resolution_table[];
struct camera_sensor_ops ov7690_sensor_ops;

/* sensor pravite functions */
static inline void ov7690_hardware_power_down(void)
{
	__gpio_as_output(OV7690_PD_PIN);
	__gpio_set_pin(OV7690_PD_PIN);
}

static inline void ov7690_hardware_power_up(void)
{
	__gpio_as_output(OV7690_PD_PIN);
	__gpio_clear_pin(OV7690_PD_PIN);
	/* short wait for power stable */
	mdelay(3);
}

static inline void ov7690_software_power_down(void)
{
	/* wait for implement later if we need */
}

static inline void ov7690_software_power_up(void)
{
	/* wait for implement later if we need */
}



/* new sensor function interface */

/* sensor_set_power use for change sensor power state
 * state indicate which state is requested
 * 0: power up
 * 1: hardware power down
 * 2: software power down
 */
static int sensor_set_power(int state)
{
	switch (state)
	{
		case 0:
			/* hardware power up first */
			ov7690_hardware_power_up();
			/* software power up later if it implemented */
			ov7690_software_power_up();
			break;
		case 1:
			ov7690_hardware_power_down();
			break;
		case 2:
			ov7690_software_power_down();
			break;
		default:
			printk("%s : EINVAL! \n",__FUNCTION__);
	}
	return 0;
}

/* sensor_early_init use for init sensor just after power up.
 * it do these basic initializations but not related preview or capture.
 * sensor_early_init should be called once after power up.
 */
static int sensor_early_init(void)
{
	/* enable power supply first*/
	ov7690_hardware_power_up();

	/* hardware reset */
	// no used in ov7690

	/* ov7690 internal registers reset */
	sensor_write_reg(ov7690_sensor_desc.client, 0x12, 0x80);
	mdelay(3);

	/* set mclk accroding to sensor need */
	// ov7690 use external crystal

	/* ov7690 basic register initializations */
	/* ov7690 GPIO init*/
	sensor_write_reg(ov7690_sensor_desc.client, 0x0c, 0x16);

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
static int sensor_set_function(int function,void *cookie)
{
	switch (function)
	{
		/* ov7690 has the same initializations sequence */
		case 0:
		case 1:
		case 2:
			sensor_write_reg(ov7690_sensor_desc.client, 0x41, 0x43);
			sensor_write_reg(ov7690_sensor_desc.client, 0x81, 0xff);
			sensor_write_reg(ov7690_sensor_desc.client, 0x21, 0x44);
			sensor_write_reg(ov7690_sensor_desc.client, 0x16, 0x03);
			sensor_write_reg(ov7690_sensor_desc.client, 0x39, 0x80);

			//		ov7690_set_FPS15(ov7690_sensor_desc.client);
			ov7690_set_FPS25(ov7690_sensor_desc.client);
			ov7690_lens_corrention_set(ov7690_sensor_desc.client);
			ov7690_color_matrix_set(ov7690_sensor_desc.client);
			ov7690_edge_donoise_set(ov7690_sensor_desc.client);
			ov7690_uvajust_set(ov7690_sensor_desc.client);
			ov7690_aec_agc_target_set(ov7690_sensor_desc.client);
			ov7690_gama_set(ov7690_sensor_desc.client);
			ov7690_general_control_set(ov7690_sensor_desc.client);

			break;
	}
	return 0;
}

/* sensor_probe use for probe weather the sensor present or not
 * return 0 means success else means fail
 */
static int sensor_probe(void)
{
	int ok;
	unsigned char idh,idl;
	/* power up and reset sensor first */
	sensor_early_init();

	idh = sensor_read_reg(ov7690_sensor_desc.client, 0x0a);
	idl = sensor_read_reg(ov7690_sensor_desc.client, 0x0b);

	/* then probe ,just read the sensor ID */
	if (idh == 0x76 &&
			(idl == 0x90 || idl == 0x91)) {
		printk("ov7690 probe suceess!");
		ok = 0;
	} else {
		printk("ov7690 probe fail !");
		ok = -1;
	}

	/* hardware power down before we leave */
	sensor_set_power(1);
	return ok;
}


static int sensor_set_resolution(int width, int height, int bpp,
		pixel_format_flag_t fmt, camera_mode_t mode)
{
	/* preview and capture has the same setting */
	/* so we do not care about mode */

	/* we support YUV422I only now */

	if ((width == 640) && (height == 480))
	{
		dprintk("640x480");
		ov7690_640_480(ov7690_sensor_desc.client);
	}
	else if ((width == 480) && (height == 320))
	{
		dprintk("480x320");
		ov7690_480_320(ov7690_sensor_desc.client);
	}
	else if ((width == 352) && (height == 288))
	{
		dprintk("352x288");
		ov7690_352_288(ov7690_sensor_desc.client);
	}
	else if ((width == 320) && (height == 240))
	{
		dprintk("320x240");
		ov7690_320_240(ov7690_sensor_desc.client);
	}
	else if ((width == 176) && (height == 144))
	{
		dprintk("176x144");
		ov7690_176_144(ov7690_sensor_desc.client);
	}
	else
	{
		dprintk("ov7690 do not support this resolution %dx%d!",width, height);
		return -1;
	}

	return 0;
}

static int ov7690_set_balance(balance_flag_t balance_flag, int arg)
{
	dprintk("ov7690_set_balance");
	switch(balance_flag)
	{
		case WHITE_BALANCE_AUTO:
			ov7690_set_wb_auto(ov7690_sensor_desc.client);
			dprintk("WHITE_BALANCE_AUTO");
			break;
		case WHITE_BALANCE_DAYLIGHT :
			ov7690_set_wb_daylight(ov7690_sensor_desc.client);
			dprintk("WHITE_BALANCE_DAYLIGHT ");
			break;
		case WHITE_BALANCE_CLOUDY_DAYLIGHT :
			ov7690_set_wb_cloudy(ov7690_sensor_desc.client);
			dprintk("WHITE_BALANCE_CLOUDY_DAYLIGHT ");
			break;
		default:
			dprintk("ov7690 do not support this WB mode %d!",balance_flag);
			return -1;
	}

	return 0;
}

static int ov7690_set_effect(effect_flag_t effect_flag,int arg)
{
	dprintk("ov7690_set_effect");
	switch(effect_flag)
	{
		case EFFECT_NONE:
			ov7690_set_effect_none(ov7690_sensor_desc.client);
			dprintk("EFFECT_NONE");
			break;
		case EFFECT_NEGATIVE :
			ov7690_set_effect_negative(ov7690_sensor_desc.client);
			dprintk("EFFECT_NEGATIVE ");
			break;
		case EFFECT_MONO :
			ov7690_set_effect_wb(ov7690_sensor_desc.client);
			dprintk("EFFECT_MONO ");
			break;
		case EFFECT_AQUA  ://qian lv se
			ov7690_set_effect_green(ov7690_sensor_desc.client);
			dprintk("EFFECT_AQUA  ");
			break;
		default:
			dprintk("ov7690 do not support this effect mode %d ",effect_flag);
			return -1;
	}

	return 0;
}

static int ov7690_set_antibanding(antibanding_flag_t antibanding_flag,int arg)
{
	dprintk("ov7690_set_antibanding");
	switch(antibanding_flag)
	{
		case ANTIBANDING_AUTO :
			ov7690_set_antibanding_auto(ov7690_sensor_desc.client);
			dprintk("ANTIBANDING_AUTO ");
			break;
		default:
			dprintk("ov7690 support ANTIBANDING_AUTO only!");
			return -1;
	}
	return 0;
}

static int ov7690_set_flash_mode(flash_mode_flag_t flash_mode_flag,int arg)
{
	dprintk("ov7690_set_flash_mode");
	switch(flash_mode_flag)
	{
		case FLASH_MODE_OFF :
			dprintk("FLASH_MODE_OFF");
			break;
		default:
			dprintk("ov7690 support FLASH_MODE_OFF only!");
			return -1;
	}
	return 0;

}

static int ov7690_set_scene_mode(scene_mode_flag_t scene_mode_flag,int arg)
{
	dprintk("ov7690_set_scene_mode");
	switch(scene_mode_flag)
	{
		case SCENE_MODE_AUTO :
			ov7690_set_scene_auto(ov7690_sensor_desc.client);
			dprintk("SCENE_MODE_AUTO ");
			break;
		case SCENE_MODE_NIGHT :
			ov7690_set_scene_night(ov7690_sensor_desc.client);
			dprintk("SCENE_MODE_NIGHT");
			break;
		default:
			dprintk("ov7690 do not support this scene mode %d !",scene_mode_flag);
			return -1;
	}
	return 0;
}

static int ov7690_set_focus_mode(focus_mode_flag_t flash_mode_flag,int arg)
{
	dprintk("ov7690_set_focus_mode");
	switch(flash_mode_flag)
	{
		case FOCUS_MODE_AUTO:
			dprintk("FOCUS_MODE_AUTO");
			break;
		default:
			dprintk("ov7690 support FOCUS_MODE_AUTO only!");
			return -1;
	}

	return 0;
}

int ov7690_set_fps(int arg)
{
	dprintk("set fps - %d",arg);
	return 0;
}

int ov7690_set_night_mode(int enable)
{
	if(enable)
		dprintk("nightshot_mode enable!");
	else
		dprintk("nightshot_mode disable!");
	return 0;
}

int ov7690_set_luma_adaptation(int arg)
{
	dprintk("luma_adaptation : %d",arg);
	return 0;
}

static int sensor_set_parameter(int flag1, int flag2, int arg)
{
	/* call ov7690_set_scene_mode
	 *      ov7690_set_balance
	 *      ov7690_set_effect
	 *      ........ and so on
	 */
	switch (flag1)
	{
		case CPCMD_SET_BALANCE:
			return ov7690_set_balance(flag2, arg);
		case CPCMD_SET_EFFECT:
			return ov7690_set_effect(flag2, arg);
		case CPCMD_SET_ANTIBANDING:
			return ov7690_set_antibanding(flag2, arg);
		case CPCMD_SET_FLASH_MODE:
			return ov7690_set_flash_mode(flag2, arg);
		case CPCMD_SET_SCENE_MODE:
			return ov7690_set_scene_mode(flag2, arg);
		case CPCMD_SET_FOCUS_MODE:
			return ov7690_set_focus_mode(flag2, arg);
		case CPCMD_SET_PREVIEW_FPS:
			ov7690_set_fps(arg);
			break;
		case CPCMD_SET_NIGHTSHOT_MODE:
			ov7690_set_night_mode(arg);
			break;
		case CPCMD_SET_LUMA_ADAPTATION:
			ov7690_set_luma_adaptation(arg);
			break;
		default:
			printk("Not supported parameter setting in ov7690! %d ", flag1);
	}


	//	ov7690_set_output_format(PIXEL_FORMAT_YUV422I,0);
	//	sensor_write_reg(ov7690_sensor_desc.client, 0x28, 0x40);

	return 0;

}

struct camera_sensor_ops ov7690_sensor_ops = {
	.sensor_init		= sensor_early_init,
	.sensor_set_power	= sensor_set_power,
	.sensor_set_function 	= sensor_set_function,
	.sensor_set_resolution 	= sensor_set_resolution,
	.sensor_set_parameter 	= sensor_set_parameter,
	.camera_sensor_probe	= sensor_probe,
};

struct resolution_info ov7690_resolution_table[] = {
	{640,480,16,PIXEL_FORMAT_YUV422I},
	//{480,320,16,PIXEL_FORMAT_YUV422I},
	{352,288,16,PIXEL_FORMAT_YUV422I},
	{320,240,16,PIXEL_FORMAT_YUV422I},
	{176,144,16,PIXEL_FORMAT_YUV422I},
};

struct camera_sensor_desc ov7690_sensor_desc = {
	.name = "ov7690",
	.camera_clock = CAM_CLOCK,
	.wait_frames = 0,
	.client = NULL,

	.ops = &ov7690_sensor_ops,

	.resolution_table = ov7690_resolution_table,
	.resolution_table_nr = ARRAY_SIZE(ov7690_resolution_table),

	.capture_parm = {640,480 , DEF_CAP_BPP,PIXEL_FORMAT_YUV422I},
	.max_capture_parm = {640,480 , MAX_CAP_BPP,PIXEL_FORMAT_YUV422I},

	.preview_parm = {640,480, 16,PIXEL_FORMAT_YUV422I},
	.max_preview_parm = {640,480, 16,PIXEL_FORMAT_YUV422I},

	.cfg_info = {
		.configure_register= 0x0
			|CIM_CFG_PACK_2			/* pack mode : ???? */
			|CIM_CFG_BYPASS			/* Bypass Mode */
			|CIM_CFG_VSP             	/* VSYNC Polarity:1-falling edge active */
			//		|CIM_CFG_PCP             	/* PCLK working edge:1-falling */
			|CIM_CFG_DSM_GCM,		/* Gated Clock Mode */
	},

	.flags = {
		//.focus_mode_flag = FOCUS_MODE_AUTO,
		.effect_flag = EFFECT_NONE | EFFECT_NEGATIVE | EFFECT_AQUA | EFFECT_MONO,
		.antibanding_flag = ANTIBANDING_AUTO,
		.balance_flag = WHITE_BALANCE_AUTO | WHITE_BALANCE_DAYLIGHT | WHITE_BALANCE_CLOUDY_DAYLIGHT,
		.scene_mode_flag = SCENE_MODE_AUTO | SCENE_MODE_NIGHT,
		//.flash_mode_flag = FLASH_MODE_OFF,

	},

};

static int ov7690_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	ov7690_sensor_desc.client = client;
	sensor_set_i2c_speed(client,200000);//set i2c speed : 200khz
	camera_sensor_register(&ov7690_sensor_desc);

	return 0;
}

static const struct i2c_device_id ov7690_id[] = {
	/* name, private data to driver */
	{ "ov7690", 0},
	{ }	/* Terminating entry */
};

MODULE_DEVICE_TABLE(i2c, ov7690_id);

static struct i2c_driver ov7690_driver = {
	.probe		= ov7690_probe,
	.id_table	= ov7690_id,
	.driver	= {
		.name = "ov7690",
	},
};

static int __init ov7690_register(void)
{
	return i2c_add_driver(&ov7690_driver);
}
module_init(ov7690_register);
