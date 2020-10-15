/*
 * OV3640 CMOS Camera Sensor Initialization
 */


#include <asm/jzsoc.h>
#include "../../jz_sensor.h"
#include <linux/i2c.h>

static char save_reg[10];
static int has_saved = -1;


void night_mode(struct i2c_client *client)
{

	if(has_saved != -1)
		return;

	save_reg[0]=sensor_read_reg16(client,0x300e);
	save_reg[1]=sensor_read_reg16(client,0x3011);
	save_reg[2]=sensor_read_reg16(client,0x3010);
	save_reg[3]=sensor_read_reg16(client,0x302a);
	save_reg[4]=sensor_read_reg16(client,0x302b);
	save_reg[5]=sensor_read_reg16(client,0x302c);
	save_reg[6]=sensor_read_reg16(client,0x3014);
	save_reg[7]=sensor_read_reg16(client,0x302e);
	save_reg[8]=sensor_read_reg16(client,0x302d);
	save_reg[9]=sensor_read_reg16(client,0x3015);
	sensor_write_reg16(client,0x300e, 0x32);
	sensor_write_reg16(client,0x3011, 0x00);
	sensor_write_reg16(client,0x3010, 0x20);
	sensor_write_reg16(client,0x302a, 0x03);
	sensor_write_reg16(client,0x302b, 0x92);//add 130 line
	sensor_write_reg16(client,0x302c, 0x00);
	sensor_write_reg16(client,0x3014, 0x0c);
	sensor_write_reg16(client,0x302e, 0x00);
	sensor_write_reg16(client,0x302d, 0x00);
	sensor_write_reg16(client,0x3015, 0x42);

	has_saved = 1;
}

void de_night_mode(struct i2c_client *client)
{
	if(has_saved != 1)
		return;

	sensor_write_reg16(client,0x300e, save_reg[0]);
	sensor_write_reg16(client,0x3011, save_reg[1]);
	sensor_write_reg16(client,0x3010, save_reg[2]);
	sensor_write_reg16(client,0x302a, save_reg[3]);
	sensor_write_reg16(client,0x302b, save_reg[4]);
	sensor_write_reg16(client,0x302c, save_reg[5]);
	sensor_write_reg16(client,0x3014, save_reg[6]);
	sensor_write_reg16(client,0x302e, save_reg[7]);
	sensor_write_reg16(client,0x302d, save_reg[8]);
	sensor_write_reg16(client,0x3015, save_reg[9]);

	has_saved = -1;
}


void __ov3640_set_night_mode(struct i2c_client *client,int enable)
{
	if(enable)
		night_mode(client);
	else
		de_night_mode(client);
}


void ov3640_set_wb_auto_mode(struct i2c_client *client)
{
	sensor_write_reg16(client,0x332b, 0x00);//AWB auto, bit[3]:0,auto
}

void ov3640_set_wb_sunny_mode(struct i2c_client *client)
{
	sensor_write_reg16(client,0x332b, 0x08); //AWB off
	sensor_write_reg16(client,0x33a7, 0x5e);
	sensor_write_reg16(client,0x33a8, 0x40);
	sensor_write_reg16(client,0x33a9, 0x46);
}

void ov3640_set_wb_cloudy_mode(struct i2c_client *client)
{
	sensor_write_reg16(client,0x332b, 0x08);
	sensor_write_reg16(client,0x33a7, 0x68);
	sensor_write_reg16(client,0x33a8, 0x40);
	sensor_write_reg16(client,0x33a9, 0x4e);
}

void ov3640_set_wb_office_mode(struct i2c_client *client)
{
	sensor_write_reg16(client,0x332b, 0x08);
	sensor_write_reg16(client,0x33a7, 0x52);
	sensor_write_reg16(client,0x33a8, 0x40);
	sensor_write_reg16(client,0x33a9, 0x58);
}

void ov3640_set_wb_home_mode(struct i2c_client *client)
{
	sensor_write_reg16(client,0x332b, 0x08);
	sensor_write_reg16(client,0x33a7, 0x44);
	sensor_write_reg16(client,0x33a8, 0x40);
	sensor_write_reg16(client,0x33a9, 0x70);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void ov3640_set_effect_antique(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3302, 0xef);
	sensor_write_reg16(client,0x3355, 0x18);
	sensor_write_reg16(client,0x335a, 0x40);
	sensor_write_reg16(client,0x335b, 0xa6);
}

void ov3640_set_effect_bluish(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3302, 0xef);
	sensor_write_reg16(client,0x3355, 0x18);
	sensor_write_reg16(client,0x335a, 0xa0);
	sensor_write_reg16(client,0x335b, 0x40);
}
void ov3640_set_effect_reddish(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3302, 0xef);
	sensor_write_reg16(client,0x3355, 0x18);
	sensor_write_reg16(client,0x335a, 0x80);
	sensor_write_reg16(client,0x335b, 0xc0);
}

void ov3640_set_effect_yellowish(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3302, 0xef);
	sensor_write_reg16(client,0x3355, 0x18);
	sensor_write_reg16(client,0x335a, 0x30);
	sensor_write_reg16(client,0x335b, 0x90);
}


void ov3640_set_effect_greenish(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3302, 0xef);
	sensor_write_reg16(client,0x3355, 0x18);
	sensor_write_reg16(client,0x335a, 0x60);
	sensor_write_reg16(client,0x335b, 0x60);
}


void ov3640_set_effect_sepia(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3302, 0xef);
	sensor_write_reg16(client,0x3355, 0x18);
	sensor_write_reg16(client,0x335a, 0x40);
	sensor_write_reg16(client,0x335b, 0xa6);
}


void ov3640_set_effect_black_white(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3302, 0xef);
	sensor_write_reg16(client,0x3355, 0x18);//bit[4]fix u enable, bit[3]fix v enable
	sensor_write_reg16(client,0x335a, 0x80);
	sensor_write_reg16(client,0x335b, 0x80);
}

void ov3640_set_effect_negative(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3302, 0xef);
	sensor_write_reg16(client,0x3355, 0x40);//bit[6] negative
}

void ov3640_set_effect_normal(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3302, 0xef);
	sensor_write_reg16(client,0x3355, 0x00);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Adjust brightness [+1 ,+7] */
void ov3640_set_adjust_bright (struct i2c_client *client, unsigned  int level)
{
	char val;
	val = level-4;
	val *= 16;
	printk(" ==============  val: %d\n", val );
	if (val>=0)
	{	
		sensor_write_reg16(client,0x3302, 0xef);
		sensor_write_reg16(client,0x3355, 0x04|sensor_read_reg16(client,0x3355)); //bit[2] enable
		sensor_write_reg16(client,0x3354, 0x01); //bit[3] sign of brightness
		sensor_write_reg16(client,0x335e, val);
	}
	else
	{
		val =-val;
		sensor_write_reg16(client,0x3302, 0xef);
		sensor_write_reg16(client,0x3355, 0x04|sensor_read_reg16(client,0x3355)); //bit[2] enable
		sensor_write_reg16(client,0x3354, 0x09); //bit[3] sign of brightness
		sensor_write_reg16(client,0x335e, val);
	}
}



void ov3640_ab_auto(struct i2c_client *client)
{
	unsigned char reg3013,reg3014;
	reg3013 = sensor_read_reg16(client,0x3013);
	reg3014 = sensor_read_reg16(client,0x3014);

	reg3013 = (reg3013 & ~(0x20)) | 0x20;
	sensor_write_reg16(client,0x3013, reg3013);
	reg3014 = (reg3014 & ~(0xc0)) | 0x40;
	sensor_write_reg16(client,0x3014, reg3014);
}

void ov3640_ab_50hz(struct i2c_client *client)
{
	unsigned char reg3013,reg3014;
	reg3013 = sensor_read_reg16(client,0x3013);
	reg3014 = sensor_read_reg16(client,0x3014);

	reg3013 = (reg3013 & ~(0x20)) | 0x20;
	sensor_write_reg16(client,0x3013, reg3013);
	reg3014 = (reg3014 & ~(0xc0)) | 0x80;
	sensor_write_reg16(client,0x3014, reg3014);
}


void ov3640_ab_60hz(struct i2c_client *client)
{
	unsigned char reg3013,reg3014;
	reg3013 = sensor_read_reg16(client,0x3013);
	reg3014 = sensor_read_reg16(client,0x3014);

	reg3013 = (reg3013 & ~(0x20)) | 0x20;
	sensor_write_reg16(client,0x3013, reg3013);
	reg3014 = (reg3014 & ~(0xc0)) | 0x00;
	sensor_write_reg16(client,0x3014, reg3014);
}


void ov3640_ab_off(struct i2c_client *client)
{
	unsigned char reg3013;
	reg3013 = sensor_read_reg16(client,0x3013);

	reg3013 = (reg3013 & ~(0x20)) | 0x00;
	sensor_write_reg16(client,0x3013, reg3013);
}






