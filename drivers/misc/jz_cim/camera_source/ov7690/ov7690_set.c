#include <asm/jzsoc.h>

#include <linux/i2c.h>
#include "../../jz_cim_core.h"
#include "../../jz_sensor.h"

/************************FPS Set******************************/
int ov7690_set_FPS15(struct i2c_client *client)
{
	sensor_write_reg(client, 0x11, 0x01);
	sensor_write_reg(client, 0x29, 0x50);
	sensor_write_reg(client, 0x2a, 0x30);
	sensor_write_reg(client, 0x2b, 0x08);
	sensor_write_reg(client, 0x2c, 0x00);
	sensor_write_reg(client, 0x15, 0x00);
	sensor_write_reg(client, 0x2d, 0x00);
	sensor_write_reg(client, 0x2e, 0x00);
	return 0;
}

int ov7690_set_FPS25(struct i2c_client *client)
{
	sensor_write_reg(client, 0x11, 0x00);
	sensor_write_reg(client, 0x29, 0x50);
	sensor_write_reg(client, 0x2a, 0x30);
	sensor_write_reg(client, 0x2b, 0x08);
	sensor_write_reg(client, 0x2c, 0x67);
	sensor_write_reg(client, 0x15, 0x00);
	sensor_write_reg(client, 0x2d, 0x00);
	sensor_write_reg(client, 0x2e, 0x00);

	return 0;
}

/************************Format Set******************************/
int ov7690_format_yuv422(struct i2c_client *client)
{
	sensor_write_reg(client, 0x12, 0x00);
	sensor_write_reg(client, 0x82, 0x03);
	sensor_write_reg(client, 0xd0, 0x48);
	sensor_write_reg(client, 0x80, 0x7f);
	sensor_write_reg(client, 0x3e, 0x30);
	sensor_write_reg(client, 0x22, 0x00);
	return 0;
}
	

int ov7690_format_rgb565(struct i2c_client *client)
{
	return 0;
}

/*************************Resolution Set*************************/
int ov7690_640_480(struct i2c_client *client)
{
	sensor_write_reg(client,0x16,0x03);
	sensor_write_reg(client,0x17,0x69);
	sensor_write_reg(client,0x18,0xa4);
	sensor_write_reg(client,0x19,0x0c);
	sensor_write_reg(client,0x1a,0xf6);
	sensor_write_reg(client,0x22,0x00);
	sensor_write_reg(client,0xc8,0x02);
	sensor_write_reg(client,0xc9,0x80);
	sensor_write_reg(client,0xca,0x01);
	sensor_write_reg(client,0xcb,0xe0);
	sensor_write_reg(client,0xcc,0x02);
	sensor_write_reg(client,0xcd,0x80);
	sensor_write_reg(client,0xce,0x01);
	sensor_write_reg(client,0xcf,0xe0);
	return 0;
}

int ov7690_480_320(struct i2c_client *client)
{
	sensor_write_reg(client,0xcc,0x01);
	sensor_write_reg(client,0xcd,0xe0);
	sensor_write_reg(client,0xce,0x01);
	sensor_write_reg(client,0xcf,0x40);
	return 0;
}


int ov7690_352_288(struct i2c_client *client)
{
	sensor_write_reg(client,0x16,0x03);
	sensor_write_reg(client,0x17,0x83);
	sensor_write_reg(client,0x18,0x97);
	sensor_write_reg(client,0x19,0x08);
	sensor_write_reg(client,0x1a,0xf6);
	sensor_write_reg(client,0x22,0x00);
	sensor_write_reg(client,0xc8,0x02);
	sensor_write_reg(client,0xc9,0x4c);
	sensor_write_reg(client,0xca,0x01);
	sensor_write_reg(client,0xcb,0xe2);
	sensor_write_reg(client,0xcc,0x01);
	sensor_write_reg(client,0xcd,0x60);
	sensor_write_reg(client,0xce,0x01);
	sensor_write_reg(client,0xcf,0x20);
	return 0;
}

int ov7690_320_240(struct i2c_client *client)
{
	sensor_write_reg(client,0x16,0x03);
	sensor_write_reg(client,0x17,0x69);
	sensor_write_reg(client,0x18,0xa4);
	sensor_write_reg(client,0x19,0x08);
	sensor_write_reg(client,0x1a,0xf6);
	sensor_write_reg(client,0x22,0x10);
	sensor_write_reg(client,0xc8,0x02);
	sensor_write_reg(client,0xc9,0x80);
	sensor_write_reg(client,0xca,0x01);
	sensor_write_reg(client,0xcb,0xe2);

	sensor_write_reg(client,0xcc,0x01);
	sensor_write_reg(client,0xcd,0x40);
	sensor_write_reg(client,0xce,0x00);
	sensor_write_reg(client,0xcf,0xf0);
	return 0;
}

int ov7690_176_144(struct i2c_client *client)
{

	sensor_write_reg(client,0x16,0x03);
	sensor_write_reg(client,0x17,0x83);
	sensor_write_reg(client,0x18,0x97);
	sensor_write_reg(client,0x19,0x08);
	sensor_write_reg(client,0x1a,0xf6);
	sensor_write_reg(client,0x22,0x10);
	sensor_write_reg(client,0xc8,0x02);
	sensor_write_reg(client,0xc9,0x4c);
	sensor_write_reg(client,0xca,0x01);
	sensor_write_reg(client,0xcb,0xe2);
	sensor_write_reg(client,0xcc,0x00);
	sensor_write_reg(client,0xcd,0xb0);
	sensor_write_reg(client,0xce,0x00);
	sensor_write_reg(client,0xcf,0x90);
	return 0;
}

/****************lens corrention**********************/
int ov7690_lens_corrention_set(struct i2c_client *client)
{
	sensor_write_reg(client, 0x85, 0x90);
	sensor_write_reg(client, 0x86, 0x00);
	sensor_write_reg(client, 0x87, 0x00);
	sensor_write_reg(client, 0x88, 0x10);
	sensor_write_reg(client, 0x89, 0x30);
	sensor_write_reg(client, 0x8a, 0x29);
	sensor_write_reg(client, 0x8b, 0x26);
	return 0;
}

int ov7690_color_matrix_set(struct i2c_client *client)
{
	sensor_write_reg(client, 0xbb, 0x80);
	sensor_write_reg(client, 0xbc, 0x62);
	sensor_write_reg(client, 0xbd, 0x1e);
	sensor_write_reg(client, 0xbe, 0x26);
	sensor_write_reg(client, 0xbf, 0x7b);
	sensor_write_reg(client, 0xc0, 0xac);
	sensor_write_reg(client, 0xc1, 0x1e);
	return 0;
}

int ov7690_edge_donoise_set(struct i2c_client *client)
{
	sensor_write_reg(client, 0xb7, 0x0c);
	sensor_write_reg(client, 0xb8, 0x04);
	sensor_write_reg(client, 0xb9, 0x00);
	sensor_write_reg(client, 0xba, 0x04);
	return 0;
}

int ov7690_uvajust_set(struct i2c_client *client)
{
	sensor_write_reg(client, 0x5a, 0x14);
	sensor_write_reg(client, 0x5b, 0xa2);
	sensor_write_reg(client, 0x5c, 0x70);
	sensor_write_reg(client, 0x5d, 0x20);
	return 0;
}

int ov7690_aec_agc_target_set(struct i2c_client *client)
{
	sensor_write_reg(client, 0x24, 0x78);
	sensor_write_reg(client, 0x25, 0x68);
	sensor_write_reg(client, 0x26, 0xb3);
	return 0;
}

int ov7690_gama_set(struct i2c_client *client)
{
	sensor_write_reg(client, 0xa3, 0x08);
	sensor_write_reg(client, 0xa4, 0x15);
	sensor_write_reg(client, 0xa5, 0x24);
	sensor_write_reg(client, 0xa6, 0x45);
	sensor_write_reg(client, 0xa7, 0x55);
	sensor_write_reg(client, 0xa8, 0x6a);
	sensor_write_reg(client, 0xa9, 0x78);
	sensor_write_reg(client, 0xaa, 0x87);
	sensor_write_reg(client, 0xab, 0x96);
	sensor_write_reg(client, 0xac, 0xa3);
	sensor_write_reg(client, 0xad, 0xb4);
	sensor_write_reg(client, 0xae, 0xc3);
	sensor_write_reg(client, 0xaf, 0xd6);
	sensor_write_reg(client, 0xb0, 0xe6);
	sensor_write_reg(client, 0xb1, 0xf2);
	sensor_write_reg(client, 0xb2, 0x12);
	return 0;
}

int ov7690_general_control_set(struct i2c_client *client)
{
	sensor_write_reg(client, 0x14, 0x20);
	sensor_write_reg(client, 0x13, 0xf7);
	return 0;
}

int ov7690_set_wb_auto(struct i2c_client *client)
{
	sensor_write_reg(client, 0x8c, 0x5c);
	sensor_write_reg(client, 0x8d, 0x11);
	sensor_write_reg(client, 0x8e, 0x12);
	sensor_write_reg(client, 0x8f, 0x19);
	sensor_write_reg(client, 0x90, 0x50);
	sensor_write_reg(client, 0x91, 0x20);
	sensor_write_reg(client, 0x92, 0x99);
	sensor_write_reg(client, 0x93, 0x91);
	sensor_write_reg(client, 0x94, 0x0f);
	sensor_write_reg(client, 0x95, 0x13);
	sensor_write_reg(client, 0x96, 0xff);
	sensor_write_reg(client, 0x97, 0x00);
	sensor_write_reg(client, 0x98, 0x38);
	sensor_write_reg(client, 0x99, 0x33);
	sensor_write_reg(client, 0x9a, 0x4f);
	sensor_write_reg(client, 0x9b, 0x43);
	sensor_write_reg(client, 0x9c, 0xf0);
	sensor_write_reg(client, 0x9d, 0xf0);
	sensor_write_reg(client, 0x9e, 0xf0);
	sensor_write_reg(client, 0x9f, 0xff);
	sensor_write_reg(client, 0xa0, 0x60);
	sensor_write_reg(client, 0xa1, 0x5a);
	sensor_write_reg(client, 0xa2, 0x10);
	return 0;
}	

int ov7690_set_wb_daylight(struct i2c_client *client)
{
	sensor_write_reg(client, 0x13, 0xf5);
	sensor_write_reg(client, 0x01, 0x5a);
	sensor_write_reg(client, 0x02, 0x5c);
	sensor_write_reg(client, 0x15, 0x00);
	return 0;
}

int ov7690_set_wb_cloudy(struct i2c_client *client)
{
	sensor_write_reg(client, 0x13, 0xf5); 
	sensor_write_reg(client, 0x01, 0x58);
	sensor_write_reg(client, 0x02, 0x60);
	sensor_write_reg(client, 0x15, 0x00);
	return 0;
}

int ov7690_set_effect_none(struct i2c_client *client)
{
	unsigned char temp;
	temp = 	sensor_read_reg(client, 0x81);
	temp &= 0xdf;
	sensor_write_reg(client, 0x81, temp);
	sensor_write_reg(client, 0x28, 0x00);
	sensor_write_reg(client, 0xd2, 0x00);
	return 0;
}

int ov7690_set_effect_negative(struct i2c_client *client)
{
	unsigned char temp;
	temp = 	sensor_read_reg(client, 0x81);
	temp |= 0x20;
	sensor_write_reg(client, 0x81, temp);
	sensor_write_reg(client, 0x28, 0x80);
	sensor_write_reg(client, 0xd2, 0x00);
	return 0;
}

int ov7690_set_effect_green(struct i2c_client *client)
{
	unsigned char temp;
	temp = 	sensor_read_reg(client, 0x81);
	temp |= 0x20;
	sensor_write_reg(client, 0x81, temp);
	sensor_write_reg(client, 0x28, 0x00);
	sensor_write_reg(client, 0xd2, 0x18);
	sensor_write_reg(client, 0xda, 0x60);
	sensor_write_reg(client, 0xdb, 0x60);
	return 0;
}


int ov7690_set_effect_wb(struct i2c_client *client)
{
	unsigned char temp;
	temp = 	sensor_read_reg(client, 0x81);
	temp |= 0x20;
	sensor_write_reg(client, 0x81, temp);
	sensor_write_reg(client, 0x28, 0x00);
	sensor_write_reg(client, 0xd2, 0x18);
	sensor_write_reg(client, 0xda, 0x80);
	sensor_write_reg(client, 0xdb, 0x80);
	return 0;
}


int ov7690_set_antibanding_auto(struct i2c_client *client)
{
	return 0;
}

int ov7690_set_scene_auto(struct i2c_client *client)
{
	sensor_write_reg(client, 0x13, 0xf7); 
	sensor_write_reg(client, 0x15, 0x00);
	return 0;
}

int ov7690_set_scene_night(struct i2c_client *client)
{
	sensor_write_reg(client, 0x13, 0xf7); 
	sensor_write_reg(client, 0x15, 0xb8);
	return 0;
}
