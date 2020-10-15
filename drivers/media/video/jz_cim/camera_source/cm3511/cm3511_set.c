
#include <asm/jzsoc.h>
#include "../../jz_sensor.h"
#include <linux/i2c.h>

#define cm3511_DEBUG
//#undef DEBUG

#ifdef cm3511_DEBUG
#define dprintk(x...)   do{printk("cm3511---\t");printk(x);printk("\n");}while(0)
#else
#define dprintk(x...)
#endif



void cm3511_set_preview_mode(struct i2c_client *client)
{
}

void cm3511_set_capture_mode(struct i2c_client *client)
{
}

void cm3511_set_nightmode(struct i2c_client *client,int enable)
{
	if (!enable)
	{

		sensor_write_reg(client,0x01, 0xf1);
		sensor_write_reg(client,0x03, 0x20);
		sensor_write_reg(client,0x10, 0x1c);
		sensor_write_reg(client,0x18, 0x38);
		sensor_write_reg(client,0x83, 0x01);//20fps
		sensor_write_reg(client,0x84, 0x24);
		sensor_write_reg(client,0x85, 0xf8);
		sensor_write_reg(client,0x88, 0x02);//10fps
		sensor_write_reg(client,0x89, 0x49);
		sensor_write_reg(client,0x8a, 0xf0);
		sensor_write_reg(client,0x70, 0x42);
		sensor_write_reg(client,0xb2, 0xd0);//AGMAX
		sensor_write_reg(client,0x01, 0xf0);
		sensor_write_reg(client,0x03, 0x20);
		sensor_write_reg(client,0x10, 0x9c);
		sensor_write_reg(client,0x18, 0x30);
	}
	else
	{

		sensor_write_reg(client,0x01, 0xf1);
		sensor_write_reg(client,0x03, 0x20);
		sensor_write_reg(client,0x10, 0x1c);
		sensor_write_reg(client,0x18, 0x38);
		sensor_write_reg(client,0x83, 0x01);//20fps
		sensor_write_reg(client,0x84, 0x24);
		sensor_write_reg(client,0x85, 0xf8);
		sensor_write_reg(client,0x88, 0x06); // 3.333fps
		sensor_write_reg(client,0x89, 0xdd);
		sensor_write_reg(client,0x8a, 0xd0);
		sensor_write_reg(client,0x70, 0x48);
		sensor_write_reg(client,0xb2, 0xd0);//AGMAX
		sensor_write_reg(client,0x01, 0xf0);
		sensor_write_reg(client,0x03, 0x20);
		sensor_write_reg(client,0x10, 0x9c);
		sensor_write_reg(client,0x18, 0x30);
	}
}   /* CM3511_NightMode */

void cm3511_set_ab_50hz(struct i2c_client *client)
{
	sensor_write_reg(client,0x03, 0x20);
	sensor_write_reg(client,0x10, 0x9c);
}

void cm3511_set_ab_60hz(struct i2c_client *client)
{
	sensor_write_reg(client,0x03, 0x20);
	sensor_write_reg(client,0x10, 0x8c);
}

void cm3511_set_effect_normal(struct i2c_client *client)
{
	sensor_write_reg(client,0x03, 0x10);
	sensor_write_reg(client,0x11, 0x03);
	sensor_write_reg(client,0x12, 0x30);
	sensor_write_reg(client,0x13, 0x00);
	sensor_write_reg(client,0x44, 0x80);
	sensor_write_reg(client,0x45, 0x80);
	sensor_write_reg(client,0x47, 0x7f);
	sensor_write_reg(client,0x20, 0x07);
	sensor_write_reg(client,0x21, 0x03);
}

void cm3511_set_effect_grayscale(struct i2c_client *client)
{
	sensor_write_reg(client,0x03, 0x10);
	sensor_write_reg(client,0x11, 0x03);
	sensor_write_reg(client,0x12, 0x23);
	sensor_write_reg(client,0x13, 0x00);
	sensor_write_reg(client,0x44, 0x80);
	sensor_write_reg(client,0x45, 0x80);
	sensor_write_reg(client,0x47, 0x7f);
	sensor_write_reg(client,0x20, 0x07);
	sensor_write_reg(client,0x21, 0x03);
}

void cm3511_set_effect_sepia(struct i2c_client *client)
{
	sensor_write_reg(client,0x03, 0x10);
	sensor_write_reg(client,0x11, 0x03);
	sensor_write_reg(client,0x12, 0x23);
	sensor_write_reg(client,0x13, 0x00);
	sensor_write_reg(client,0x44, 0x70);
	sensor_write_reg(client,0x45, 0x98);
	sensor_write_reg(client,0x47, 0x7f);
	sensor_write_reg(client,0x20, 0x07);
	sensor_write_reg(client,0x21, 0x03);
}

void cm3511_set_effect_colorinv(struct i2c_client *client)
{
	sensor_write_reg(client,0x03, 0x10);
	sensor_write_reg(client,0x11, 0x03);
	sensor_write_reg(client,0x12, 0x28);
	sensor_write_reg(client,0x13, 0x00);
	sensor_write_reg(client,0x44, 0x80);
	sensor_write_reg(client,0x45, 0x80);
	sensor_write_reg(client,0x47, 0x7f);
	sensor_write_reg(client,0x20, 0x07);
	sensor_write_reg(client,0x21, 0x03);
}

void cm3511_set_effect_sepiagreen(struct i2c_client *client)
{
	sensor_write_reg(client,0x03, 0x10);
	sensor_write_reg(client,0x11, 0x03);
	sensor_write_reg(client,0x12, 0x33);
	sensor_write_reg(client,0x13, 0x00);
	sensor_write_reg(client,0x44, 0x30);
	sensor_write_reg(client,0x45, 0x50);
	sensor_write_reg(client,0x47, 0x7f);
	sensor_write_reg(client,0x20, 0x07);
	sensor_write_reg(client,0x21, 0x03);
}

void cm3511_set_effect_sepiablue(struct i2c_client *client)
{
	sensor_write_reg(client,0x03, 0x10);
	sensor_write_reg(client,0x11, 0x03);
	sensor_write_reg(client,0x12, 0x33);
	sensor_write_reg(client,0x13, 0x00);
	sensor_write_reg(client,0x44, 0xb0);
	sensor_write_reg(client,0x45, 0x40);
	sensor_write_reg(client,0x47, 0x7f);
	sensor_write_reg(client,0x20, 0x07);
	sensor_write_reg(client,0x21, 0x03);
}

void cm3511_set_effect_grayinv(struct i2c_client *client)
{
	sensor_write_reg(client,0x03, 0x10);
	sensor_write_reg(client,0x11, 0x03);
	sensor_write_reg(client,0x12, 0x28);
	sensor_write_reg(client,0x13, 0x00);
	sensor_write_reg(client,0x44, 0x80);
	sensor_write_reg(client,0x45, 0x80);
	sensor_write_reg(client,0x47, 0x7f);
	sensor_write_reg(client,0x20, 0x07);
	sensor_write_reg(client,0x21, 0x03);
}

void cm3511_set_effect_embossment(struct i2c_client *client)
{
	sensor_write_reg(client,0x03, 0x10);
	sensor_write_reg(client,0x11, 0x23);
	sensor_write_reg(client,0x12, 0x33);
	sensor_write_reg(client,0x13, 0x02);
	sensor_write_reg(client,0x44, 0x80);
	sensor_write_reg(client,0x45, 0x80);
	sensor_write_reg(client,0x47, 0x7f);
	sensor_write_reg(client,0x20, 0x07);
	sensor_write_reg(client,0x21, 0x07);
}

void cm3511_set_effect_sketch(struct i2c_client *client)
{
	sensor_write_reg(client,0x03, 0x10);
	sensor_write_reg(client,0x11, 0x13);
	sensor_write_reg(client,0x12, 0x38);
	sensor_write_reg(client,0x13, 0x02);
	sensor_write_reg(client,0x44, 0x80);
	sensor_write_reg(client,0x45, 0x80);
	sensor_write_reg(client,0x47, 0x7f);
	sensor_write_reg(client,0x20, 0x07);
	sensor_write_reg(client,0x21, 0x07);
}


void cm3511_set_wb_auto(struct i2c_client *client)
{
	sensor_write_reg(client,0x03,0x22);  // AUTO 3000K~7000K                                     
	sensor_write_reg(client,0x10,0x6a);                                                          
	//sensor_write_reg(client,0x80,0x3a);                                                          
	//sensor_write_reg(client,0x81,0x20);		                                                       
	//sensor_write_reg(client,0x82,0x32);		                                                       
	sensor_write_reg(client,0x83,0x65);		                                                       
	sensor_write_reg(client,0x84,0x1a);		                                                       
	sensor_write_reg(client,0x85,0x58);		                                                       
	sensor_write_reg(client,0x86,0x22);                                                          
	sensor_write_reg(client,0x03,0x22);                                                          
	sensor_write_reg(client,0x10,0xea);        
}

void cm3511_set_wb_cloud(struct i2c_client *client)
{
	// NOON130PC20_reg_WB_cloudy   ÒõÌì
	sensor_write_reg(client,0x03,0x22);   //7000K                                     
	sensor_write_reg(client,0x10,0x6a);                                                          
	sensor_write_reg(client,0x80,0x50);                                                          
	sensor_write_reg(client,0x81,0x20);		                                                       
	sensor_write_reg(client,0x82,0x24);		                                                       
	sensor_write_reg(client,0x83,0x6d);		                                                       
	sensor_write_reg(client,0x84,0x65);		                                                       
	sensor_write_reg(client,0x85,0x24);		                                                       
	sensor_write_reg(client,0x86,0x1c);                                                          
	//sensor_write_reg(client,0x03,0x22);                                                          
	//sensor_write_reg(client,0x10,0xea);                                                          
}


void cm3511_set_wb_daylight(struct i2c_client *client)
{
	// NOON130PC20_reg_WB_daylight  °×Ìì 
	sensor_write_reg(client,0x03,0x22);  //6500K                                     
	sensor_write_reg(client,0x10,0x6a);                                                          
	sensor_write_reg(client,0x80,0x40);                                                          
	sensor_write_reg(client,0x81,0x20);		                                                       
	sensor_write_reg(client,0x82,0x28);		                                                       
	sensor_write_reg(client,0x83,0x45);		                                                       
	sensor_write_reg(client,0x84,0x35);		                                                       
	sensor_write_reg(client,0x85,0x2d);		                                                       
	sensor_write_reg(client,0x86,0x1c);                                                          
}


void cm3511_set_wb_incandescence(struct i2c_client *client)
{
	sensor_write_reg(client,0x03,0x22);  //2800K~3000K                                     
	sensor_write_reg(client,0x10,0x6a);                                                          
	sensor_write_reg(client,0x80,0x25);                                                          
	sensor_write_reg(client,0x81,0x20);		                                                       
	sensor_write_reg(client,0x82,0x44);		                                                       
	sensor_write_reg(client,0x83,0x24);		                                                       
	sensor_write_reg(client,0x84,0x1e);		                                                       
	sensor_write_reg(client,0x85,0x50);		                                                       
	sensor_write_reg(client,0x86,0x45);                                                          
}


void cm3511_set_wb_fluorescent(struct i2c_client *client)
{
	sensor_write_reg(client,0x03,0x22);  //4200K~5000K                                     
	sensor_write_reg(client,0x10,0x6a);                                                          
	sensor_write_reg(client,0x80,0x35);                                                          
	sensor_write_reg(client,0x81,0x20);		                                                       
	sensor_write_reg(client,0x82,0x32);		                                                       
	sensor_write_reg(client,0x83,0x3c);		                                                       
	sensor_write_reg(client,0x84,0x2c);		                                                       
	sensor_write_reg(client,0x85,0x45);		                                                       
	sensor_write_reg(client,0x86,0x35);                                                          
}


void cm3511_set_wb_tungsten(struct i2c_client *client)
{
	sensor_write_reg(client,0x03,0x22);  //4000K                                   
	sensor_write_reg(client,0x10,0x6a);                                                          
	sensor_write_reg(client,0x80,0x33);                                                          
	sensor_write_reg(client,0x81,0x20);		                                                       
	sensor_write_reg(client,0x82,0x3d);		                                                       
	sensor_write_reg(client,0x83,0x2e);		                                                       
	sensor_write_reg(client,0x84,0x24);		                                                       
	sensor_write_reg(client,0x85,0x43);		                                                       
	sensor_write_reg(client,0x86,0x3d);                                                          
}


void cm3511_init_setting(struct i2c_client *client)
{	
	dprintk("-------------sensor init!");
	sensor_write_reg(client,0x01, 0xf1);
	sensor_write_reg(client,0x01, 0xf3);
	sensor_write_reg(client,0x01, 0xf1);

	sensor_write_reg(client,0x03, 0x20);
	sensor_write_reg(client,0x10, 0x1c);//AE off
	sensor_write_reg(client,0x03, 0x22);
	sensor_write_reg(client,0x10, 0x6a);//AWB off

	sensor_write_reg(client,0x03, 0x00);//Page 0
	sensor_write_reg(client,0x10, 0x00);
	sensor_write_reg(client,0x11, 0x90);
	sensor_write_reg(client,0x12, 0x04);//Pclk inversion
	sensor_write_reg(client,0x20, 0x00);
	sensor_write_reg(client,0x21, 0x05);
	sensor_write_reg(client,0x22, 0x00);
	sensor_write_reg(client,0x23, 0x07);
	sensor_write_reg(client,0x24, 0x01);
	sensor_write_reg(client,0x25, 0xe0);
	sensor_write_reg(client,0x26, 0x02);
	sensor_write_reg(client,0x27, 0x80);

	sensor_write_reg(client,0x40, 0x01);
	sensor_write_reg(client,0x41, 0x50);
	sensor_write_reg(client,0x42, 0x00);
	sensor_write_reg(client,0x43, 0x14);

	sensor_write_reg(client,0x80, 0x0e);
	sensor_write_reg(client,0x90, 0x0a);
	sensor_write_reg(client,0x91, 0x0a);
	sensor_write_reg(client,0x92, 0x60);

	sensor_write_reg(client,0xa0, 0x10);
	sensor_write_reg(client,0xa1, 0x10);
	sensor_write_reg(client,0xa2, 0x10);
	sensor_write_reg(client,0xa3, 0x10);
	sensor_write_reg(client,0xa4, 0x10);
	sensor_write_reg(client,0xa5, 0x10);
	sensor_write_reg(client,0xa6, 0x10);
	sensor_write_reg(client,0xa7, 0x10);

	sensor_write_reg(client,0xa8, 0x48);
	sensor_write_reg(client,0xa9, 0x48);
	sensor_write_reg(client,0xaa, 0x47);
	sensor_write_reg(client,0xab, 0x47);
	sensor_write_reg(client,0xac, 0x47);
	sensor_write_reg(client,0xad, 0x47);
	sensor_write_reg(client,0xae, 0x46);
	sensor_write_reg(client,0xaf, 0x46);

	sensor_write_reg(client,0x03, 0x02 );//Page 2
	sensor_write_reg(client,0x1a, 0x31 );
	sensor_write_reg(client,0x1c, 0x00 );
	sensor_write_reg(client,0x1d, 0x03 );

	sensor_write_reg(client,0x20, 0x33 );
	sensor_write_reg(client,0x21, 0x77 );
	sensor_write_reg(client,0x22, 0xad );
	sensor_write_reg(client,0x34, 0xff );
	sensor_write_reg(client,0x54, 0x30 );

	sensor_write_reg(client,0x62, 0x78 );
	sensor_write_reg(client,0x63, 0x7a );
	sensor_write_reg(client,0x64, 0x7d );
	sensor_write_reg(client,0x65, 0x88 );

	sensor_write_reg(client,0x72, 0x78 );
	sensor_write_reg(client,0x73, 0x8b );
	sensor_write_reg(client,0x74, 0x78 );
	sensor_write_reg(client,0x75, 0x8b );

	sensor_write_reg(client,0xa0, 0x03 );
	sensor_write_reg(client,0xa8, 0x03 );
	sensor_write_reg(client,0xaa, 0x03 );

	sensor_write_reg(client,0x03, 0x10 );//Page 10
	sensor_write_reg(client,0x10, 0x01);//ISPCTL1(Control the format of image data) : YUV order for Lot51 by NHJ
	sensor_write_reg(client,0x11, 0x03);
	sensor_write_reg(client,0x12, 0x30);

	//sensor_write_reg(client,0x40, 0x18);
	sensor_write_reg(client,0x41, 0x05);
	sensor_write_reg(client,0x50, 0x50);

	sensor_write_reg(client,0x60, 0x1f);
	sensor_write_reg(client,0x61, 0xa8);
	sensor_write_reg(client,0x62, 0xaa);
	sensor_write_reg(client,0x63, 0x50);
	sensor_write_reg(client,0x64, 0x60);
	sensor_write_reg(client,0x65, 0x90);

	sensor_write_reg(client,0x03, 0x11);//Page 11
	sensor_write_reg(client,0x10, 0x1d);
	sensor_write_reg(client,0x11, 0x0a); //open LPF in dark, 0x0a close
	sensor_write_reg(client,0x21, 0x28);
	sensor_write_reg(client,0x60, 0x12);
	sensor_write_reg(client,0x61, 0x83);
	sensor_write_reg(client,0x62, 0x43);
	sensor_write_reg(client,0x63, 0x53);

	sensor_write_reg(client,0x03, 0x12);//Page 12
	sensor_write_reg(client,0x40, 0x21);
	sensor_write_reg(client,0x41, 0x07);
	sensor_write_reg(client,0x42, 0x0f); //add
	sensor_write_reg(client,0x50, 0x0d);
	sensor_write_reg(client,0x70, 0x1d);
	sensor_write_reg(client,0x74, 0x04);
	sensor_write_reg(client,0x75, 0x06);
	sensor_write_reg(client,0x90, 0x5d);
	sensor_write_reg(client,0x91, 0x10);
	sensor_write_reg(client,0xb0, 0xc9);

	sensor_write_reg(client,0x03, 0x13);//Page 13
	sensor_write_reg(client,0x10, 0x19);
	sensor_write_reg(client,0x11, 0x07);
	sensor_write_reg(client,0x12, 0x01);
	sensor_write_reg(client,0x13, 0x02);
	sensor_write_reg(client,0x20, 0x06);
	sensor_write_reg(client,0x21, 0x05);
	sensor_write_reg(client,0x23, 0x18);
	sensor_write_reg(client,0x24, 0x03);

	sensor_write_reg(client,0x80, 0x0d);
	sensor_write_reg(client,0x81, 0x01);
	sensor_write_reg(client,0x83, 0x5d);

	sensor_write_reg(client,0x90, 0x02);
	sensor_write_reg(client,0x91, 0x02);
	sensor_write_reg(client,0x93, 0x19);
	sensor_write_reg(client,0x94, 0x03);
	sensor_write_reg(client,0x95, 0x00);

	sensor_write_reg(client,0x03, 0x14);//Page 14
	sensor_write_reg(client,0x10, 0x07);
	sensor_write_reg(client,0x20, 0x80);
	sensor_write_reg(client,0x21, 0x80);
	sensor_write_reg(client,0x22, 0x80);
	sensor_write_reg(client,0x23, 0x64);
	sensor_write_reg(client,0x24, 0x52);
	sensor_write_reg(client,0x25, 0x78);
	sensor_write_reg(client,0x26, 0x70);

	sensor_write_reg(client,0x03, 0x15);//Page 15
	sensor_write_reg(client,0x10, 0x0f);
	sensor_write_reg(client,0x14, 0x36);
	sensor_write_reg(client,0x16, 0x28);
	sensor_write_reg(client,0x17, 0x2f);

	sensor_write_reg(client,0x30, 0x5b);
	sensor_write_reg(client,0x31, 0x26);
	sensor_write_reg(client,0x32, 0x0a);
	sensor_write_reg(client,0x33, 0x11);
	sensor_write_reg(client,0x34, 0x65);
	sensor_write_reg(client,0x35, 0x14);
	sensor_write_reg(client,0x36, 0x01);
	sensor_write_reg(client,0x37, 0x33);
	sensor_write_reg(client,0x38, 0x74);

	sensor_write_reg(client,0x40, 0x00);
	sensor_write_reg(client,0x41, 0x00);
	sensor_write_reg(client,0x42, 0x00);
	sensor_write_reg(client,0x43, 0x8b);
	sensor_write_reg(client,0x44, 0x07);
	sensor_write_reg(client,0x45, 0x04);
	sensor_write_reg(client,0x46, 0x84);
	sensor_write_reg(client,0x47, 0xa1);
	sensor_write_reg(client,0x48, 0x25);

	sensor_write_reg(client,0x03, 0x16);//Page 16
	sensor_write_reg(client,0x10, 0x01);
	sensor_write_reg(client,0x30, 0x00);// original
	sensor_write_reg(client,0x31, 0x0d);
	sensor_write_reg(client,0x32, 0x16);
	sensor_write_reg(client,0x33, 0x2a);
	sensor_write_reg(client,0x34, 0x44);
	sensor_write_reg(client,0x35, 0x62);
	sensor_write_reg(client,0x36, 0x7d);
	sensor_write_reg(client,0x37, 0x97);
	sensor_write_reg(client,0x38, 0xa9);
	sensor_write_reg(client,0x39, 0xba);
	sensor_write_reg(client,0x3a, 0xc8);
	sensor_write_reg(client,0x3b, 0xdd);
	sensor_write_reg(client,0x3c, 0xec);
	sensor_write_reg(client,0x3d, 0xf8);
	sensor_write_reg(client,0x3e, 0xff);

	sensor_write_reg(client,0x03, 0x17);
	sensor_write_reg(client,0xc0, 0x03);
	sensor_write_reg(client,0xc4, 0x23);
	sensor_write_reg(client,0xc5, 0x1d);
	sensor_write_reg(client,0xc6, 0x02);
	sensor_write_reg(client,0xc7, 0x20);

	sensor_write_reg(client,0x03, 0x20);//page 20
	sensor_write_reg(client,0x10, 0x1c);
	sensor_write_reg(client,0x11, 0x00);
	sensor_write_reg(client,0x20, 0x00);
	sensor_write_reg(client,0x28, 0x0b);
	sensor_write_reg(client,0x29, 0xaf);
	sensor_write_reg(client,0x2a, 0xf0);
	sensor_write_reg(client,0x2b, 0x34);
	sensor_write_reg(client,0x30, 0xf8); //new add by peter

	sensor_write_reg(client,0x60, 0x00);
	//sensor_write_reg(client,0x70, 0x42);
	sensor_write_reg(client,0x7a, 0x34);


	sensor_write_reg(client,0x83, 0x01); //;;ExpNormal 20Fps
	sensor_write_reg(client,0x84, 0x24);
	sensor_write_reg(client,0x85, 0xf8);

	sensor_write_reg(client,0x86, 0x01);
	sensor_write_reg(client,0x87, 0xf4);

	//sensor_write_reg(client,0x88, 0x06);//ExpMin 3.3333Fps
	//sensor_write_reg(client,0x89, 0xdd);
	//sensor_write_reg(client,0x8a, 0xd0);

	//sensor_write_reg(client,0x88, 0x02);//ExpMin 10Fps
	//sensor_write_reg(client,0x89, 0x49);
	//sensor_write_reg(client,0x8a, 0xf0);

	sensor_write_reg(client,0x8b, 0x3a);//EXP100
	sensor_write_reg(client,0x8c, 0x98);
	sensor_write_reg(client,0x8d, 0x30);//EXP120
	sensor_write_reg(client,0x8e, 0xd4);

	sensor_write_reg(client,0x8f, 0xc4);
	sensor_write_reg(client,0x90, 0x68);

	sensor_write_reg(client,0x9c, 0x04);//new add by peter
	sensor_write_reg(client,0x9d, 0x65);
	sensor_write_reg(client,0x9e, 0x00);
	sensor_write_reg(client,0x9f, 0xfa);

	sensor_write_reg(client,0xb0, 0x1a);//AG
	sensor_write_reg(client,0xb1, 0x1a);//AGMIN
	//sensor_write_reg(client,0xb2, 0x80);//AGMAX
	sensor_write_reg(client,0xb3, 0x1a);//AGLVL
	sensor_write_reg(client,0xb4, 0x1a);//AGTH1
	sensor_write_reg(client,0xb5, 0x45);//AGTH2
	sensor_write_reg(client,0xb6, 0x2f);//AGBTH1
	sensor_write_reg(client,0xb7, 0x28);//AGBTH2
	sensor_write_reg(client,0xb8, 0x24);//AGBTH3
	sensor_write_reg(client,0xb9, 0x22);//AGBTH4
	sensor_write_reg(client,0xba, 0x21);//AGBTH5
	sensor_write_reg(client,0xbb, 0x20);//AGBTH6
	sensor_write_reg(client,0xbc, 0x1f);//AGBTH7
	sensor_write_reg(client,0xbd, 0x1e);//AGBTH8
	sensor_write_reg(client,0xc0, 0x14);//PGA_Sky_gain
	sensor_write_reg(client,0xc3, 0x60);
	sensor_write_reg(client,0xc4, 0x58);
	sensor_write_reg(client,0xc8, 0x78);

	sensor_write_reg(client,0x03, 0x22);//page 22
	//sensor_write_reg(client,0x10, 0x6a);
	sensor_write_reg(client,0x11, 0x2c);
	sensor_write_reg(client,0x21, 0x01);
	sensor_write_reg(client,0x38, 0x12);
	sensor_write_reg(client,0x40, 0xe3);
	sensor_write_reg(client,0x41, 0x88);
	sensor_write_reg(client,0x42, 0x44);
	sensor_write_reg(client,0x46, 0x0a);

	sensor_write_reg(client,0x80, 0x38);//R Gain
	sensor_write_reg(client,0x81, 0x20);//G Gain
	sensor_write_reg(client,0x82, 0x30);//B Gain

	sensor_write_reg(client,0x83, 0x65);//RMAX
	sensor_write_reg(client,0x84, 0x1a);//RMIN
	sensor_write_reg(client,0x85, 0x58);//BMAX
	sensor_write_reg(client,0x86, 0x22);//BMIN

	sensor_write_reg(client,0x87, 0x50);//RMAXB
	sensor_write_reg(client,0x88, 0x20);//RMINB
	sensor_write_reg(client,0x89, 0x50);//BMAXB
	sensor_write_reg(client,0x8a, 0x20);//BMINB
	sensor_write_reg(client,0x8b, 0x08);
	sensor_write_reg(client,0x8d, 0x14);
	sensor_write_reg(client,0x8e, 0x61);

	sensor_write_reg(client,0x8f, 0x58);
	sensor_write_reg(client,0x90, 0x54);
	sensor_write_reg(client,0x91, 0x50);
	sensor_write_reg(client,0x92, 0x4b);
	sensor_write_reg(client,0x93, 0x46);
	sensor_write_reg(client,0x94, 0x43);
	sensor_write_reg(client,0x95, 0x40);
	sensor_write_reg(client,0x96, 0x3c);
	sensor_write_reg(client,0x97, 0x38);
	sensor_write_reg(client,0x98, 0x34);
	sensor_write_reg(client,0x99, 0x2e);
	sensor_write_reg(client,0x9a, 0x26);
	sensor_write_reg(client,0x9b, 0x07);

	sensor_write_reg(client,0x10, 0xea);//AWB ON
	//sensor_write_reg(client,0x03, 0x20);//page 20
	//sensor_write_reg(client,0x10, 0x9c);//AE ON

	sensor_write_reg(client,0x01, 0xf0);


}   /* CM3511_Write_Sensor_Initial_Setting */

