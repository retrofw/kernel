#include <asm/jzsoc.h>
#include "../../jz_sensor.h"
#include <linux/i2c.h>

#define ov2655_DEBUG
#ifdef ov2655_DEBUG
#define dprintk(x...)   do{printk("cm3511---\t");printk(x);printk("\n");}while(0)
#else
#define dprintk(x...)
#endif


void night_enable(struct i2c_client *client)
{
	sensor_write_reg16(client,0x300e, 0x34);
	sensor_write_reg16(client,0x3011, 0x00);
	sensor_write_reg16(client,0x302c, 0x00);
	sensor_write_reg16(client,0x3071, 0x00);
	sensor_write_reg16(client,0x3070, 0xb9);
	sensor_write_reg16(client,0x301c, 0x02);
	sensor_write_reg16(client,0x3073, 0x00);
	sensor_write_reg16(client,0x3072, 0x9a);
	sensor_write_reg16(client,0x301d, 0x03);
	sensor_write_reg16(client,0x3014, 0x0c);
	sensor_write_reg16(client,0x3015, 0x50);
	sensor_write_reg16(client,0x302e, 0x00);
	sensor_write_reg16(client,0x302d, 0x00);
}

void de_night_disable(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3014, 0x04);
	sensor_write_reg16(client,0x3015, 0x00);
	sensor_write_reg16(client,0x302e, 0x00);
	sensor_write_reg16(client,0x302d, 0x00);
}

void ov2655_night_select(struct i2c_client *client,int enable)
{
}
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/






/*----------------------------------set white balance------------------------------------------------*/

void ov2655_set_wb_auto_mode(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3306,0x00);   // select Auto WB
}

void ov2655_set_wb_sunny_mode(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3306, 0x02);  // Disable Auto WB, select Manual WB
	sensor_write_reg16(client,0x3337, 0x5e);
	sensor_write_reg16(client,0x3338, 0x40);
	sensor_write_reg16(client,0x3339, 0x46);
}

void ov2655_set_wb_cloudy_mode(struct i2c_client *client)
{ 
	sensor_write_reg16(client,0x3306, 0x82);  // Disable Auto WB, select Manual WB
	sensor_write_reg16(client,0x3337, 0x68); //manual R G B
	sensor_write_reg16(client,0x3338, 0x40);
	sensor_write_reg16(client,0x3339, 0x4e);
}

void ov2655_set_wb_office_mode(struct i2c_client *client)
{ 
	sensor_write_reg16(client,0x3306, 0x02);  // Disable Auto WB, select Manual WB
	sensor_write_reg16(client,0x3337, 0x52);
	sensor_write_reg16(client,0x3338, 0x40);
	sensor_write_reg16(client,0x3339, 0x58);
}

void ov2655_set_wb_home_mode(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3320,0x9a);

	sensor_write_reg16(client,0x3306, 0x02);  // Disable Auto WB, select Manual WB
	sensor_write_reg16(client,0x3337, 0x44);
	sensor_write_reg16(client,0x3338, 0x40);
	sensor_write_reg16(client,0x3339, 0x70);
}

/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/





/*---------------------------------------set banding------------------------------------------------*/

void ov2655_ab_auto(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3014, 0x44);    /* enable banding and 50 Hz */
}

void ov2655_ab_50hz(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3014, 0x84);    /* enable banding and 50 Hz */
}


void ov2655_ab_60hz(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3014, 0x04);    /* enable banding and 60 Hz */
}


void ov2655_ab_off(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3014, 0x44);    /* enable banding and 50 Hz */
}
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/





/*---------------------------------------set effect------------------------------------------------*/

void ov2655_set_effect_normal(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3391, 0x00);
}

void ov2655_set_effect_sepia(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3391, 0x18);
	sensor_write_reg16(client,0x3396, 0x40);
	sensor_write_reg16(client,0x3397, 0xa6);
}

void ov2655_set_effect_bluish(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3391, 0x18);
	sensor_write_reg16(client,0x3396, 0xa0);
	sensor_write_reg16(client,0x3397, 0x40);
}

void ov2655_set_effect_greenish(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3391, 0x18);
	sensor_write_reg16(client,0x3396, 0x60);
	sensor_write_reg16(client,0x3397, 0x60);
}

void ov2655_set_effect_reddish(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3391, 0x18);
	sensor_write_reg16(client,0x3396, 0x80);
	sensor_write_reg16(client,0x3397, 0xc0);
}

void ov2655_set_effect_yellowish(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3391, 0x18);
	sensor_write_reg16(client,0x3396, 0x30);
	sensor_write_reg16(client,0x3397, 0x90);
}

void ov2655_set_effect_blackwhite(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3391, 0x20);
}

void ov2655_set_effect_negative(struct i2c_client *client)
{
	sensor_write_reg16(client,0x3391, 0x40);  //b[6] is negative
}

/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/





