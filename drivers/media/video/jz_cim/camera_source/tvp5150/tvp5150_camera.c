#include <asm/jzsoc.h>
#include <linux/i2c.h>
#include "tvp5150_camera.h"
#include "tvp5150_reg.h"
#include "tvp5150_set.h"
#include <asm/jzsoc.h>

#define tvp5150_DEBUG
#ifdef tvp5150_DEBUG
#define dprintk(x...)   do{printk("tvp5150---\t");printk(x);printk("\n");}while(0)
#else
#define dprintk(x...)
#endif

/* gpio init */
#if defined(CONFIG_JZ4750_APUS)
#define GPIO_CAMERA_RST         (32 * 4 + 16)
#define GPIO_CAMERA_PD          (32 * 4 + 17)
#elif defined(CONFIG_JZ4760_PT701)
#define GPIO_CAMERA_RST         (32*3+10) /*GPD10*/
#define GPIO_CAMERA_PD         (32*3+11) /*GPD11*/
#elif defined(CONFIG_JZ4760_LEPUS)
#define GPIO_CAMERA_RST         (32 * 1 + 26)   /* GPB26 */
#define GPIO_CAMERA_PD         (32 * 1 + 27) /* GPB27 */
#else
#error "tvp5150/tvp5150_camera.c , please define camera for your board."
#endif


struct camera_sensor_desc tvp5150_sensor_desc;

void tvp5150_power_down(void)
{
	printk("===>power down tvp5150\n");
	__gpio_as_output(GPIO_CAMERA_PD);
	__gpio_clear_pin(GPIO_CAMERA_PD);

	mdelay(10);

	__gpio_as_output(GPIO_CAMERA_RST);
	__gpio_set_pin(GPIO_CAMERA_RST);

	mdelay(10);
}

void tvp5150_power_up(void)
{

	printk("===>power up tvp5150\n");
	__gpio_as_output(GPIO_CAMERA_PD);
	__gpio_set_pin(GPIO_CAMERA_PD);

	mdelay(20);
}

void tvp5150_reset(void)
{
	printk("===>reset tvp5150\n");
	__gpio_as_output(GPIO_CAMERA_RST);
	__gpio_clear_pin(GPIO_CAMERA_RST);
	mdelay(1);
	__gpio_set_pin(GPIO_CAMERA_RST);
	mdelay(1);
}


int tvp5150_set_balance(balance_flag_t balance_flag,int arg)
{
	return 0;
}

int tvp5150_set_effect(effect_flag_t effect_flag,int arg)
{
	return 0;
}

int tvp5150_set_antibanding(antibanding_flag_t antibanding_flag,int arg)
{
	return 0;
}


int tvp5150_set_flash_mode(flash_mode_flag_t flash_mode_flag,int arg)
{
	return 0;
}

int tvp5150_set_scene_mode(scene_mode_flag_t scene_mode_flag,int arg)
{
	return 0;
}

int tvp5150_set_focus_mode(focus_mode_flag_t flash_mode_flag,int arg)
{
	return 0;
}


int tvp5150_set_fps(int fps)
{
	dprintk("set fps : %d",fps);
	return 0;
}

int tvp5150_set_luma_adaptation(int arg)
{
	dprintk("luma_adaptation : %d",arg);
	return 0;
}

int tvp5150_set_parameter(int cmd, int mode, int arg)
{
	switch(cmd)
	{
		case CPCMD_SET_BALANCE :
			tvp5150_set_balance(mode,arg);
			break;
		case CPCMD_SET_EFFECT :
			tvp5150_set_effect(mode,arg);
			break;
		case CPCMD_SET_ANTIBANDING :
			tvp5150_set_antibanding(mode,arg);
			break;
		case CPCMD_SET_FLASH_MODE :
			tvp5150_set_flash_mode(mode,arg);
			break;
		case CPCMD_SET_SCENE_MODE :
			tvp5150_set_scene_mode(mode,arg);
			break;
		case CPCMD_SET_PIXEL_FORMAT :
			break;
		case CPCMD_SET_FOCUS_MODE :
			tvp5150_set_focus_mode(mode,arg);
			break;
		case CPCMD_SET_PREVIEW_FPS:
			tvp5150_set_fps(arg);
			break;
		case CPCMD_SET_NIGHTSHOT_MODE:
			break;
		case CPCMD_SET_LUMA_ADAPTATION:
			tvp5150_set_luma_adaptation(arg);
			break;
	}
	return 0;
}

int tvp5150_set_power(int state)
{

	switch (state)
	{
		case 0:
			/* hardware power up first */
			tvp5150_power_up();
			/* software power up later if it implemented */
			break;
		case 1:
			tvp5150_power_down();
			break;
		case 2:
			break;
		default:
			printk("%s : EINVAL! \n",__FUNCTION__);
	}
	return 0;
}

int tvp5150_sensor_init(void)
{
	printk("===>init TVP5150AM1\n");

	tvp5150_power_up();
	tvp5150_reset();

	//tvp5150_probe(tvp5150_sensor_desc.client);
	//tvp5150_sensor_reset(tvp5150_sensor_desc.client);
	tvp5150_write(tvp5150_sensor_desc.client, TVP5150_AUTOSW_MSK, 0);
	tvp5150_write(tvp5150_sensor_desc.client, TVP5150_MISC_CTL, 0x0d);
	tvp5150_write(tvp5150_sensor_desc.client, TVP5150_DATA_RATE_SEL, 0x40);

	return 0;
}

int tvp5150_sensor_probe(void)
{
	u8 msb_rom, lsb_rom;

	tvp5150_power_up();
	tvp5150_reset();

	msb_rom = tvp5150_read(tvp5150_sensor_desc.client, TVP5150_ROM_MAJOR_VER);
	lsb_rom = tvp5150_read(tvp5150_sensor_desc.client, TVP5150_ROM_MINOR_VER);

#if 0
	/* just for test */
	if (msb_rom == 4 && lsb_rom == 0) { /* Is TVP5150AM1 */
		printk("===>init TVP5150AM1\n");
		tvp5150_probe(tvp5150_sensor_desc.client);
		tvp5150_sensor_reset(tvp5150_sensor_desc.client);
		return 0;
	}
#else
	tvp5150_power_down();

	if (msb_rom == 4 && lsb_rom == 0) { /* Is TVP5150AM1 */
		return 0;
	}
#endif

	return -1;
}

/* sensor_set_function use for init preview or capture.there may be some difference between preview and capture.
 * so we divided it into two sequences.param: function indicated which function
 * 0: preview
 * 1: capture
 * 2: recording
 */
int tvp5150_set_function(int function, void *cookie)
{
	switch (function)
	{
		case 0:
			preview_set(tvp5150_sensor_desc.client);
			break;
		case 1:
			capture_set(tvp5150_sensor_desc.client);
			break;
		case 2:
			break;
	}
	return 0;
}

int tvp5150_set_resolution(int width,int height,int bpp,pixel_format_flag_t fmt,camera_mode_t mode)
{
	size_switch(tvp5150_sensor_desc.client, width, height, mode);
	return 0;
}

static int tvp5150_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	tvp5150_sensor_desc.client = client;
	sensor_set_i2c_speed(client,400000);// set ov3640 i2c speed : 400khz
	ret = camera_sensor_register(&tvp5150_sensor_desc);
	dump_stack();

	return ret;
}

struct camera_sensor_ops tvp5150_sensor_ops = {
	.sensor_init = tvp5150_sensor_init,
	.camera_sensor_probe = tvp5150_sensor_probe,
	.sensor_set_function = tvp5150_set_function,
	.sensor_set_resolution = tvp5150_set_resolution,
	.sensor_set_parameter = tvp5150_set_parameter,
	.sensor_set_power = tvp5150_set_power,
};

struct resolution_info tvp5150_resolution_table[] = {
	{720,486,PIXEL_FORMAT_YUV422I},
};

struct camera_sensor_desc tvp5150_sensor_desc = {
	.name = "tvp5150",
	.camera_clock = CAM_CLOCK,
	.wait_frames = 2,
	.client = NULL,

	.ops = &tvp5150_sensor_ops,

	.resolution_table = tvp5150_resolution_table,
	.resolution_table_nr=ARRAY_SIZE(tvp5150_resolution_table),

	.capture_parm = {720,486, 16,PIXEL_FORMAT_YUV422I},
	.max_capture_parm = {720, 486, 16,PIXEL_FORMAT_YUV422I},

	.preview_parm = {720, 486, 16,PIXEL_FORMAT_YUV422I},
	.max_preview_parm = {720 , 486, 16,PIXEL_FORMAT_YUV422I},

	.cfg_info = {
		/* src: cb y cr y, dst: cb y cr y */
		.configure_register= 0x0
		| CIM_CFG_DF_ITU656
		| CIM_CFG_ORDER_2
		|CIM_CFG_PACK_4			/* pack mode :0x 4 3 2 1*/
		|CIM_CFG_BYPASS			/* Bypass Mode */
		//|CIM_CFG_PCP             	/* PCLK working edge:1-falling */
		//|CIM_CFG_DSM_GCM,		/* Gated Clock Mode */
		| CIM_CFG_DSM_CIM, /* CCIR656 Interlace Mode */
	},
};

static const struct i2c_device_id tvp5150_id[] = {
	{ "tvp5150", 0 },
};
MODULE_DEVICE_TABLE(i2c, tvp5150_id);

static struct i2c_driver tvp5150_driver = {
	.probe		= tvp5150_i2c_probe,
	.id_table	= tvp5150_id,
	.driver	= {
		.name = "tvp5150",
	},
};

static int __init tvp5150_register(void)
{
	return i2c_add_driver(&tvp5150_driver);
}

module_init(tvp5150_register);
