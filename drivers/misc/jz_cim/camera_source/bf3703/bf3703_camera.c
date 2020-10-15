#include <asm/jzsoc.h>
#include <linux/i2c.h>
#include "bf3703_camera.h"
#include "bf3703_set.h"
#include "bf3703_set_mode.h"
#include <asm/jzsoc.h>

#define bf3703_DEBUG
#ifdef bf3703_DEBUG
#define dprintk(x...)   do{printk("bf3703---\t");printk(x);printk("\n");}while(0)
#else
#define dprintk(x...)
#endif

/* gpio init */
#if defined(CONFIG_JZ4760_LYNX) || defined(CONFIG_JZ4760B_LYNX) || defined(CONFIG_JZ4760B_LEPUS)
#define GPIO_CAMERA_RST         	(32*4+18) /*GPE18*/ 
#define GPIO_CAMERA_PDN   		(32*4+19) /*GPE19*/
#elif defined(CONFIG_JZ4770_MAPLE)
#define GPIO_CAMERA_RST         	(32*4+3) /*GPE18*/ 
#define GPIO_CAMERA_PDN   		(32*4+4) /*GPE19*/
#else
#error "bf3703/bf3703_camera.c , please define camera gpio for your board."
#endif

struct bf3703_sensor bf3703;

void bf3703_power_down(void)
{
	__gpio_as_output(GPIO_CAMERA_PDN);
	__gpio_set_pin(GPIO_CAMERA_PDN);
	mdelay(500);
}

void bf3703_power_up(void)
{ 
	//bf3703_power_on later
	__gpio_as_output(GPIO_CAMERA_PDN);
	__gpio_clear_pin(GPIO_CAMERA_PDN);
	mdelay(50);
}

void bf3703_reset(void)
{
	__gpio_as_output(GPIO_CAMERA_RST);
	mdelay(50);
	__gpio_clear_pin(GPIO_CAMERA_RST);
	mdelay(50);
	__gpio_set_pin(GPIO_CAMERA_RST);
	mdelay(50);
}


int bf3703_set_balance(balance_flag_t balance_flag,int arg)
{
	dprintk("bf3703_set_balance");
	switch(balance_flag)
	{
		case WHITE_BALANCE_AUTO:
			bf3703_set_wb_auto_mode(bf3703.client);
			dprintk("wb_auto ");
			break;
		case WHITE_BALANCE_DAYLIGHT ://ri guang
			bf3703_set_wb_sunny_mode(bf3703.client);
			dprintk("wb_daylight ");
			break;
		case WHITE_BALANCE_CLOUDY_DAYLIGHT ://ying tian
			bf3703_set_wb_cloudy_mode(bf3703.client);
			dprintk("wb_cloudy daylight ");
			break;
		case WHITE_BALANCE_INCANDESCENT :
			bf3703_set_wb_office_mode(bf3703.client);
			dprintk("wb_incandenscent ");
			break;
	}
	return 0;
}

void bf3703_set_effect_whiteboard(struct i2c_client *client);
int bf3703_set_effect(effect_flag_t effect_flag,int arg)
{
	dprintk("bf3703_set_effect");
	switch(effect_flag)
	{
		case EFFECT_NONE:
			bf3703_set_effect_normal(bf3703.client);
			dprintk("effect_none");
			break;
		case EFFECT_MONO :
			bf3703_set_effect_blackwhite(bf3703.client);  
			dprintk("effect_mono ");
			break;
		case EFFECT_NEGATIVE :
			bf3703_set_effect_negative(bf3703.client);
			dprintk("effect_negative ");
			break;
		case EFFECT_SOLARIZE ://bao guang
			dprintk("effect_solarize ");
			break;
		case EFFECT_SEPIA :
			bf3703_set_effect_sepia(bf3703.client);
			dprintk("effect_sepia ");
			break;
		case EFFECT_POSTERIZE ://se diao fen li
			dprintk("effect_posterize ");
			break;
		case EFFECT_WHITEBOARD :
			dprintk("effect_whiteboard ");
			break;
		case EFFECT_BLACKBOARD :
			bf3703_set_effect_whiteboard(bf3703.client);
			dprintk("effect_blackboard ");
			break;
		case EFFECT_AQUA ://qian lv se
			bf3703_set_effect_greenish(bf3703.client);
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

int bf3703_set_antibanding(antibanding_flag_t antibanding_flag,int arg)
{
	dprintk("bf3703_set_antibanding");
	switch(antibanding_flag)
	{
		case ANTIBANDING_AUTO :
			bf3703_ab_auto(bf3703.client);
			dprintk("ANTIBANDING_AUTO ");
			break;
		case ANTIBANDING_50HZ :
			bf3703_ab_50hz(bf3703.client);
			dprintk("ANTIBANDING_50HZ ");
			break;
		case ANTIBANDING_60HZ :
			bf3703_ab_60hz(bf3703.client);
			dprintk("ANTIBANDING_60HZ ");
			break;
		case ANTIBANDING_OFF :
			bf3703_ab_off(bf3703.client);
			dprintk("ANTIBANDING_OFF ");
			break;
	}
	return 0;
}


int bf3703_set_flash_mode(flash_mode_flag_t flash_mode_flag,int arg)
{
	return 0;
}

int bf3703_set_scene_mode(scene_mode_flag_t scene_mode_flag,int arg)
{
	return 0;
}

int bf3703_set_focus_mode(focus_mode_flag_t flash_mode_flag,int arg)
{
	return 0;
}


int bf3703_set_fps(int fps)
{
	dprintk("set fps : %d",fps);
	return 0;
}

int bf3703_set_luma_adaptation(int arg)
{
	dprintk("luma_adaptation : %d",arg);
	return 0;
}

int bf3703_set_parameter(int cmd, int mode, int arg)
{
	switch(cmd)
	{
		case CPCMD_SET_BALANCE :
			bf3703_set_balance(mode,arg);
			break;
		case CPCMD_SET_EFFECT :
			bf3703_set_effect(mode,arg);
			break;
		case CPCMD_SET_ANTIBANDING :
			bf3703_set_antibanding(mode,arg);
			break;
		case CPCMD_SET_FLASH_MODE :
			bf3703_set_flash_mode(mode,arg);
			break;
		case CPCMD_SET_SCENE_MODE :
			bf3703_set_scene_mode(mode,arg);
			break;
		case CPCMD_SET_PIXEL_FORMAT :
			break;
		case CPCMD_SET_FOCUS_MODE :
			bf3703_set_focus_mode(mode,arg);
			break;
		case CPCMD_SET_PREVIEW_FPS:
			bf3703_set_fps(arg);
			break;
		case CPCMD_SET_NIGHTSHOT_MODE:
			break;
		case CPCMD_SET_LUMA_ADAPTATION:
			bf3703_set_luma_adaptation(arg);
			break;
	}
	return 0;
}

int bf3703_set_power(int state)
{
	switch (state)
	{
		case 0:           
			/* hardware power up first */
			bf3703_power_up();
			/* software power up later if it implemented */
			break;
		case 1:
			bf3703_power_down();
			break;
		case 2:
			break;
		default:
			printk("%s : EINVAL! \n",__FUNCTION__);
	}
	return 0;
}

int bf3703_sensor_init(void)
{
	//bf3703_reset();
	bf3703_init_setting(bf3703.client);
	return 0;
}

int bf3703_sensor_probe(void)
{
	int sensor_id = 0;

	bf3703_power_up();
	bf3703_reset();
	sensor_id = (sensor_read_reg(bf3703.client,0xFC)<<8)|sensor_read_reg(bf3703.client,0xFD);	
	bf3703_power_down();

	if(sensor_id == 0x3703)
		return 0; 
	return -1;
}

/* sensor_set_function use for init preview or capture.there may be some difference between preview and capture.
 * so we divided it into two sequences.param: function indicated which function
 * 0: preview
 * 1: capture
 * 2: recording
 */
int bf3703_set_function(int function)
{
	switch (function)
	{
		case 0:
			preview_set(bf3703.client);
			break;
		case 1:
			capture_set(bf3703.client);
			break;
		case 2:		
			break;
	}
	return 0;
}

int bf3703_set_resolution(int width,int height,int bpp,pixel_format_flag_t fmt,camera_mode_t mode)
{
	size_switch(bf3703.client,width,height,mode);
	return 0;         
}

static ssize_t bf3703_write_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	//uint32_t reg,val;
	//sscanf(buf,"%x,%x",&reg,&val);
	//printk("write reg : %x | value : %x\n",reg&0xffff,val&0xff);
	//sensor_write_reg16(bf3703.client,reg&0xffff,val&0xff);

	return count;
}

static ssize_t bf3703_read_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	//uint32_t reg;
	//sscanf(buf,"%x",&reg);
	//printk("read reg 0x%x = 0x%x\n",reg&0xffff,sensor_read_reg16(bf3703.client,reg&0xffff));

	return count;
}


static DEVICE_ATTR(write, 0664, NULL, bf3703_write_store);
static DEVICE_ATTR(read, 0664, NULL, bf3703_read_store);

//static struct attribute *bf3703_attributes[] = {
	//&dev_attr_write.attr,
	//&dev_attr_read.attr,
	//NULL
//};

//static const struct attribute_group bf3703_attr_group = {
//	.attrs = bf3703_attributes,
//};

static int bf3703_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	bf3703.client = client;

	//sysfs_create_group(&client->dev.kobj, &bf3703_attr_group);
	sensor_set_i2c_speed(client,400000);
	return camera_sensor_register(&bf3703.desc);
}

struct camera_sensor_ops bf3703_sensor_ops = {
	.sensor_init = bf3703_sensor_init,
	.camera_sensor_probe = bf3703_sensor_probe,
	.sensor_set_function = bf3703_set_function,
	.sensor_set_resolution = bf3703_set_resolution, 
	.sensor_set_parameter = bf3703_set_parameter, 
	.sensor_set_power = bf3703_set_power,
};

struct resolution_info bf3703_resolution_table[] = {	
	{640,480,16,PIXEL_FORMAT_YUV422I},	//
	{352,288,16,PIXEL_FORMAT_YUV422I},
	{320,240,16,PIXEL_FORMAT_YUV422I},	//
	{176,144,16,PIXEL_FORMAT_YUV422I},
};

struct bf3703_sensor bf3703 = {
	.desc = {
		.name = "bf3703",
		.wait_frames = 0,

		.ops = &bf3703_sensor_ops,

		.resolution_table = bf3703_resolution_table,
		.resolution_table_nr=ARRAY_SIZE(bf3703_resolution_table),

		.capture_parm = {640,480, 16,PIXEL_FORMAT_YUV422I},
		.max_capture_parm = {640,480, 16,PIXEL_FORMAT_YUV422I},

		.preview_parm = {640,480, 16,PIXEL_FORMAT_YUV422I}, 
		.max_preview_parm = {640,480, 16,PIXEL_FORMAT_YUV422I},

		.cfg_info = {
			.configure_register= 0x0
				|CIM_CFG_PACK_3			/* pack mode : 4 3 2 1 */
				|CIM_CFG_BYPASS			/* Bypass Mode */	
				|CIM_CFG_VSP             	/* VSYNC Polarity:1-falling edge active */
                	//      |CIM_CFG_HSP 
		//		|CIM_CFG_PCP             	/* PCLK working edge:1-falling */
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
				| WHITE_BALANCE_INCANDESCENT,
			.antibanding_flag = ~0x0,
			.flash_mode_flag = 0,
			.scene_mode_flag = 0,
			.pixel_format_flag = 0,
			.focus_mode_flag = 0,
		},
	},
};

static const struct i2c_device_id bf3703_id[] = {
	{ "bf3703", 0 },
	{ }	/* Terminating entry */
};
MODULE_DEVICE_TABLE(i2c, bf3703_id);

static struct i2c_driver bf3703_driver = {
	.probe		= bf3703_i2c_probe,
	.id_table	= bf3703_id,
	.driver	= {
		.name = "bf3703",
	},
};

static int __init bf3703_i2c_register(void)
{
	return i2c_add_driver(&bf3703_driver);
}

module_init(bf3703_i2c_register);
