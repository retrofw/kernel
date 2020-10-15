#include <asm/jzsoc.h>
#include "../../jz_sensor.h"
#include <linux/i2c.h>

#define bf3703_DEBUG
#ifdef bf3703_DEBUG
#define dprintk(x...)   do{printk("cm3511---\t");printk(x);printk("\n");}while(0)
#else
#define dprintk(x...)
#endif


void night_enable(struct i2c_client *client)
{
	
}

void de_night_disable(struct i2c_client *client)
{
	
}

void bf3703_night_select(struct i2c_client *client,int enable)
{
}
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/






/*----------------------------------set white balance------------------------------------------------*/

void bf3703_set_wb_auto_mode(struct i2c_client *client)
{
	sensor_write_reg(client,0x01,0x15);
	sensor_write_reg(client,0x02,0x24);
	sensor_write_reg(client,0x13,0x07);   // Enable AWB
}

void bf3703_set_wb_sunny_mode(struct i2c_client *client)
{
	sensor_write_reg(client,0x13,0x05);   // Disable AWB
	sensor_write_reg(client,0x01,0x13);
	sensor_write_reg(client,0x02,0x26);	
}

void bf3703_set_wb_cloudy_mode(struct i2c_client *client)
{ 
	sensor_write_reg(client,0x13,0x05);   // Disable AWB
	sensor_write_reg(client,0x01,0x10);
	sensor_write_reg(client,0x02,0x28);
}

void bf3703_set_wb_office_mode(struct i2c_client *client)
{ 
	sensor_write_reg(client,0x13,0x05);   // Disable AWB
	sensor_write_reg(client,0x01,0x1f);
	sensor_write_reg(client,0x02,0x15);
}

void bf3703_set_wb_home_mode(struct i2c_client *client)
{
	
}

/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/





/*---------------------------------------set banding------------------------------------------------*/

void bf3703_ab_auto(struct i2c_client *client)
{
}

void bf3703_ab_50hz(struct i2c_client *client)
{
	sensor_write_reg(client,0x9d,0x99); 

}


void bf3703_ab_60hz(struct i2c_client *client)
{
	sensor_write_reg(client,0x9e,0x80); 

}


void bf3703_ab_off(struct i2c_client *client)
{
}
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/





/*---------------------------------------set effect------------------------------------------------*/

void bf3703_set_effect_normal(struct i2c_client *client)
{
	sensor_write_reg(client,0x80,0x45);		
	sensor_write_reg(client,0x76,0x00);
	sensor_write_reg(client,0x69,0x00);		
	sensor_write_reg(client,0x67,0x80);		
	sensor_write_reg(client,0x68,0x80);
}

void bf3703_set_effect_sepia(struct i2c_client *client)
{
	sensor_write_reg(client,0x80,0x45);
	sensor_write_reg(client,0x76,0x00);
	sensor_write_reg(client,0x69,0x20);		
	sensor_write_reg(client,0x67,0x60);		
	sensor_write_reg(client,0x68,0x98);
}

void bf3703_set_effect_whiteboard(struct i2c_client *client)
{
 	sensor_write_reg(client,0x69,0x00);
	sensor_write_reg(client,0x80,0xc5);		
	sensor_write_reg(client,0x76,0xf0);			
	sensor_write_reg(client,0x67,0x80);		
	sensor_write_reg(client,0x68,0x80);
}


void bf3703_set_effect_bluish(struct i2c_client *client)
{
 	
}

void bf3703_set_effect_greenish(struct i2c_client *client)
{
	sensor_write_reg(client,0x80,0x45);
	sensor_write_reg(client,0x76,0x00);
	sensor_write_reg(client,0x69,0x20);		
	sensor_write_reg(client,0x67,0x40);		
	sensor_write_reg(client,0x68,0x40);
}

void bf3703_set_effect_reddish(struct i2c_client *client)
{
}

void bf3703_set_effect_yellowish(struct i2c_client *client)
{
}

void bf3703_set_effect_blackwhite(struct i2c_client *client)
{
	sensor_write_reg(client,0x80,0x45);		
	sensor_write_reg(client,0x76,0x00);
	sensor_write_reg(client,0x69,0x20);		
	sensor_write_reg(client,0x67,0x80);		
	sensor_write_reg(client,0x68,0x80);
}

void bf3703_set_effect_negative(struct i2c_client *client)
{
	sensor_write_reg(client,0x80,0x45);		
	sensor_write_reg(client,0x76,0x00);
	sensor_write_reg(client,0x69,0x40);		
	sensor_write_reg(client,0x67,0x80);		
	sensor_write_reg(client,0x68,0x80);

}

/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/





