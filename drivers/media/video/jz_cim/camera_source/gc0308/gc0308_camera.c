#include <linux/i2c.h>
#include <asm/jzsoc.h>
#include "../../jz_sensor.h"
#include "../../jz_cim_core.h"
#include "gc0308.h"

struct camera_sensor_desc gc0308_sensor_desc;

void gc0308_power_down(void)
{
}

void gc0308_power_up(void)
{
}

int gc0308_set_parameter(int cmd, int mode, int arg)
{
	return 0;
}

int gc0308_set_power(int state)
{
	switch (state)
	{
	case 0:
		gc0308_power_up();
		break;
	case 1:
		gc0308_power_down();
		break;
	case 2:
		break;
	default:
		printk("%s : EINVAL! \n",__FUNCTION__);
	}
	return 0;
}

int gc0308_sensor_init(void)
{
	printk("===>init GC0308\n");
	return 0;
}

int gc0308_sensor_probe(void)
{
	unsigned char chipid;

	printk("===>enter %s:%d\n", __func__, __LINE__);

	/* read the chipid to confirm that the sensor is GC0308 */
	select_page_gc308(0);
	chipid = sensor_read_reg_nostop(gc0308_sensor_desc.client, 0x00);

	printk("===>enter %s:%d, chipid = 0x%02x\n", __func__, __LINE__, chipid);

	return (chipid == 0x9b) ? 0 : -1;
}

/* sensor_set_function use for init preview or capture.there may be some difference between preview and capture.
 * so we divided it into two sequences.param: function indicated which function
 * 0: preview
 * 1: capture
 * 2: recording
 */
int gc0308_set_function(int function, void *cookie)
{
	switch (function)
	{
	case 0:
		//preview_set(gc0308_sensor_desc.client);
		break;
	case 1:
		//capture_set(gc0308_sensor_desc.client);
		break;
	case 2:
		break;
	}
	return 0;
}

int gc0308_set_resolution(int width,int height,int bpp,pixel_format_flag_t fmt,camera_mode_t mode)
{
	return 0;
}

static int gc0308_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	printk("===>enter %s:%d\n", __func__, __LINE__);
	gc0308_sensor_desc.client = client;
	set_gc0308_i2c_client(client);
	sensor_set_i2c_speed(client,100000);// max speed : 400khz
	return camera_sensor_register(&gc0308_sensor_desc);
}

struct camera_sensor_ops gc0308_sensor_ops = {
	.sensor_init = gc0308_sensor_init,
	.camera_sensor_probe = gc0308_sensor_probe,
	.sensor_set_function = gc0308_set_function,
	.sensor_set_resolution = gc0308_set_resolution,
	.sensor_set_parameter = gc0308_set_parameter,
	.sensor_set_power = gc0308_set_power,
};

struct resolution_info gc0308_resolution_table[] = {
	{640,480,16,PIXEL_FORMAT_YUV422I},
	{480,320,16,PIXEL_FORMAT_YUV422I},
	{352,288,16,PIXEL_FORMAT_YUV422I},
	{320,240,16,PIXEL_FORMAT_YUV422I},
	{176,144,16,PIXEL_FORMAT_YUV422I},
};

struct camera_sensor_desc gc0308_sensor_desc = {
	.name = "gc0308",
	.camera_clock = 24000000,
	.wait_frames = 2,
	.client = NULL,

	.ops = &gc0308_sensor_ops,

	.resolution_table = gc0308_resolution_table,
	.resolution_table_nr=ARRAY_SIZE(gc0308_resolution_table),

	.capture_parm = {640,480, 16,PIXEL_FORMAT_YUV422I},
	.max_capture_parm = {640, 480, 16,PIXEL_FORMAT_YUV422I},

	.preview_parm = {640, 480, 16,PIXEL_FORMAT_YUV422I},
	.max_preview_parm = {640 , 480, 16,PIXEL_FORMAT_YUV422I},

	.cfg_info = {
		.configure_register= 0x0
		|CIM_CFG_PACK_4			/* pack mode : 4 3 2 1, aka keep the sequence of the orignal data*/
		|CIM_CFG_BYPASS			/* Bypass Mode */
		//|CIM_CFG_VSP             	/* VSYNC Polarity:1-falling edge active */
		//|CIM_CFG_PCP             	/* PCLK working edge:1-falling */
		|CIM_CFG_DSM_GCM,		/* Gated Clock Mode */
	},
};

static const struct i2c_device_id gc0308_id[] = {
	{ "gc030_7_8", 0 },
};
MODULE_DEVICE_TABLE(i2c, gc0308_id);

static struct i2c_driver gc0308_driver = {
	.probe		= gc0308_i2c_probe,
	.id_table	= gc0308_id,
	.driver	= {
		.name = "gc030_7_8",
	},
};

static int __init gc0308_register(void)
{
	printk("===>enter %s:%d\n", __func__, __LINE__);
	return i2c_add_driver(&gc0308_driver);
}

module_init(gc0308_register);
