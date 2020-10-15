#include "../../jz_cim_core.h"
#include "../../jz_sensor.h"
#include <linux/i2c.h>
#include <asm/jzsoc.h>

#define ov2655_DEBUG
#ifdef ov2655_DEBUG
#define dprintk(x...)   do{printk("ov2655-\t");printk(x);printk("\n");}while(0)
#else
#define dprintk(x...)
#endif


#define Default_SVGA_Line_Width (1940)
#define Default_UXGA_Line_Width (1940)
uint8_t Default_Reg0x3028 = 7;
uint8_t Default_Reg0x3029 = 147;
uint8_t Default_Reg0x302a = 4;
uint8_t Default_Reg0x302b = 212;

#define Default_UXGA_maximum_shutter (1236)
//#define Preview_dummy_pixel  0
#define Capture_dummy_pixel  (0)
#define Capture_PCLK_Frequency (36)//Unit is MHz
#define Preview_PCLK_Frequency (36)//Unit is MHz

uint8_t Preview_Reg0x3000,
	Preview_Reg0x3002,
	Preview_Reg0x3003,
	Preview_Reg0x3013,
	Reg0x3013,
	Reg0x3000,
	Reg0x3002,
	Reg0x3003;


uint16_t Preview_dummy_pixel,
	 Extra_lines,
	 Capture_Banding_Filter,
	 Capture_Gain16,
	 Preview_Gain16,
	 Capture_dummy_line = 63;

uint32_t Preview_Exposure,
	 Shutter,
	 Capture_Max_Gain16,
	 Preview_Line_Width,
	 Capture_Line_Width,
	 Capture_Exposure,
	 Gain_Exposure,
	 Capture_Maximum_Shutter;


void preview_set(struct i2c_client *client)
{
	dprintk("-preview_set UXGA>SVGA");
	// transfer from UXGA to SVGA
	sensor_write_reg16(client,0x300e,0x34);
	sensor_write_reg16(client,0x3011,0x00);
	sensor_write_reg16(client,0x3012,0x10);
	sensor_write_reg16(client,0x302A,0x02);
	sensor_write_reg16(client,0x302B,0x6a);
	sensor_write_reg16(client,0x306f,0x14);

	sensor_write_reg16(client,0x3020,0x01);
	sensor_write_reg16(client,0x3021,0x18);
	sensor_write_reg16(client,0x3022,0x00);
	sensor_write_reg16(client,0x3023,0x06);
	sensor_write_reg16(client,0x3024,0x06);
	sensor_write_reg16(client,0x3025,0x58);
	sensor_write_reg16(client,0x3026,0x02);
	sensor_write_reg16(client,0x3027,0x61);
	sensor_write_reg16(client,0x3088,0x03);
	sensor_write_reg16(client,0x3089,0x20);
	sensor_write_reg16(client,0x308a,0x02);
	sensor_write_reg16(client,0x308b,0x58);
	sensor_write_reg16(client,0x3316,0x64);
	sensor_write_reg16(client,0x3317,0x25);
	sensor_write_reg16(client,0x3318,0x80);
	sensor_write_reg16(client,0x3319,0x08);
	sensor_write_reg16(client,0x331a,0x64);
	sensor_write_reg16(client,0x331b,0x4b);
	sensor_write_reg16(client,0x331c,0x00);
	sensor_write_reg16(client,0x331d,0x38);
	sensor_write_reg16(client,0x3302,0x11);
	sensor_write_reg16(client,0x3013,0xf7);

	sensor_write_reg16(client,0x3070,0xb9);
	sensor_write_reg16(client,0x3071,0x00);
	sensor_write_reg16(client,0x3072,0x9a);
	sensor_write_reg16(client,0x3073,0x00);
	sensor_write_reg16(client,0x301c,0x02);
	sensor_write_reg16(client,0x301d,0x03);

	/* color bar */
	{
		unsigned char val;

		/* color bar enable */
		val = sensor_read_reg16(client, 0x3308);
		val |= 0x1;   /* set bit0 */
		sensor_write_reg16(client, 0x3308, val);
	}
}    //===preview setting end===


void capture_reg_set(struct i2c_client *client)
{
	dprintk("capture_reg_set");
	// ?? -> 1600x1200
	sensor_write_reg16(client,0x300e,0x34);
	sensor_write_reg16(client,0x3011,0x01);
	sensor_write_reg16(client,0x3012,0x00);
	sensor_write_reg16(client,0x302A,0x04);
	sensor_write_reg16(client,0x302B,0xd4);
	sensor_write_reg16(client,0x306f,0x54);
	sensor_write_reg16(client,0x3020,0x01);
	sensor_write_reg16(client,0x3021,0x18);
	sensor_write_reg16(client,0x3022,0x00);
	sensor_write_reg16(client,0x3023,0x0a);
	sensor_write_reg16(client,0x3024,0x06);
	sensor_write_reg16(client,0x3025,0x58);
	sensor_write_reg16(client,0x3026,0x04);
	sensor_write_reg16(client,0x3027,0xbc);
	sensor_write_reg16(client,0x3088,0x06);
	sensor_write_reg16(client,0x3089,0x40);
	sensor_write_reg16(client,0x308a,0x04);
	sensor_write_reg16(client,0x308b,0xb0);
	sensor_write_reg16(client,0x3316,0x64);
	sensor_write_reg16(client,0x3317,0x4b);
	sensor_write_reg16(client,0x3318,0x00);
	sensor_write_reg16(client,0x3319,0x2c);
	sensor_write_reg16(client,0x331a,0x64);
	sensor_write_reg16(client,0x331b,0x4b);
	sensor_write_reg16(client,0x331c,0x00);
	sensor_write_reg16(client,0x331d,0x4c);
	sensor_write_reg16(client,0x3302,0x01);
}

void p_reg(struct i2c_client *client)
{
	printk("\n\n\n=========================================================\n\n\n");
	printk("0x3000 = 0x%2x\n",sensor_read_reg16(client,0x3000));
	printk("0x3002 = 0x%2x\n",sensor_read_reg16(client,0x3002));
	printk("0x3003 = 0x%2x\n",sensor_read_reg16(client,0x3003));
	printk("\n\n\n=========================================================\n\n\n");
}

void capture_set(struct i2c_client *client)
{
	dprintk("capture_set");

	//Stop AE/AG
	Reg0x3013 = sensor_read_reg16(client,0x3013);
	Preview_Reg0x3013 = Reg0x3013;
	Reg0x3013 = Reg0x3013 & 0xfa;
	dprintk("-yes-Reg0x3013=%d\n",Reg0x3013);

	sensor_write_reg16(client,0x3013,Reg0x3013);

	Reg0x3002 = sensor_read_reg16(client,0x3002);
	dprintk("Reg0x3002=%d\n",Reg0x3002);
	Reg0x3003 = sensor_read_reg16(client,0x3003);
	dprintk("Reg0x3003=%d\n",Reg0x3003);
	Shutter = (Reg0x3002<<8) + Reg0x3003;
	dprintk("Shutter=%d\n",Shutter);

	Preview_Exposure = Shutter ;
	//Read Back Gain for preview
	Reg0x3000 = sensor_read_reg16(client,0x3000);
	dprintk("Reg0x3000=%d\n",Reg0x3000);
	Preview_Reg0x3000 = Reg0x3000;
	Preview_Gain16 = (((Reg0x3000 & 0xf0)>>4) + 1) * (16 + (Reg0x3000 & 0x0f));
	dprintk("Preview_Gain16=%d\n",Preview_Gain16);

	//Maximum gain limitation for capture, Capture_max_gain16 = capture_maximum_gain * 16
	Capture_Max_Gain16 = 32;
	dprintk("Capture_Max_Gain16=%d\n",Capture_Max_Gain16);
	Preview_Line_Width = Default_SVGA_Line_Width ;
	Capture_Line_Width = Default_UXGA_Line_Width ;
	Capture_Maximum_Shutter = Default_UXGA_maximum_shutter;

	Capture_Exposure =
		Preview_Exposure
		*Capture_PCLK_Frequency
		/Preview_PCLK_Frequency
		*Preview_Line_Width
		/Capture_Line_Width
		/2;

	dprintk("Capture_Exposure=%d\n",Capture_Exposure);

	Capture_Banding_Filter = Capture_PCLK_Frequency*1000000/ 100/(2*Capture_Line_Width);
	dprintk("Capture_Banding_Filter=%d\n",Capture_Banding_Filter);
	Gain_Exposure = Preview_Gain16 * Capture_Exposure*100/85;
	dprintk("Gain_Exposure=%d\n",Gain_Exposure);
	if (Gain_Exposure < Capture_Banding_Filter * 16) {//1200
		// Exposure < 1/100
		Capture_Exposure = Gain_Exposure /16;
		Capture_Gain16 = (Gain_Exposure*2 + 1)/Capture_Exposure/2;
		dprintk("Capture_Gain16=%d\n",Capture_Gain16);
	}
	else
	{
		if (Gain_Exposure > Capture_Maximum_Shutter * 16) //16720
		{
			// Exposure > Capture_Maximum_Shutter
			dprintk(" 1");
			Capture_Exposure = Capture_Maximum_Shutter;
			Capture_Gain16 = (Gain_Exposure*2 + 1)/Capture_Maximum_Shutter/2;
			dprintk("-Gain_Exposure > Capture_Maximum_Shutter * 16Capture_Gain16=%d\n",Capture_Gain16);
			if (Capture_Gain16 > Capture_Max_Gain16)
			{
				dprintk("2");
				// gain reach maximum, insert extra line
				Capture_Exposure = Gain_Exposure*1.1/Capture_Max_Gain16;
				// Exposure = n/100
				Capture_Exposure = Gain_Exposure/16/Capture_Banding_Filter;
				Capture_Exposure = Capture_Exposure * Capture_Banding_Filter;
				Capture_Gain16 = (Gain_Exposure *2+1)/ Capture_Exposure/2;
				dprintk("-Capture_Gain16 > Capture_Max_Gain16Capture_Gain16=%d\n",Capture_Gain16);
			}
		}
		else
		{
			// 1/100 < Exposure < Capture_Maximum_Shutter, Exposure = n/100
			dprintk("3");
			Capture_Exposure = Gain_Exposure /16;
			Capture_Exposure = Capture_Exposure/Capture_Banding_Filter;
			dprintk("4");
			Capture_Exposure = Capture_Exposure * Capture_Banding_Filter;
			printk("Capture_Exposure = %d\n",Capture_Exposure);
			printk("Gain_Exposure = %d\n",Gain_Exposure);
			Capture_Gain16 = (Gain_Exposure*2 +1) / Capture_Exposure/2;
			dprintk("else-Capture_Gain16=%d\n",Capture_Gain16);
		}
	}


	//Write Exposure
	//       if (Capture_Exposure > Capture_Maximum_Shutter) {
	Shutter = Capture_Exposure;
	dprintk("Write ExposureShutter=%d\n",Shutter);
	//	}

	capture_reg_set(client);

	Reg0x3003 = Shutter & 0x00ff;
	Reg0x3002 = (Shutter >>8) & 0x00ff;
	sensor_write_reg16(client,0x3003, Reg0x3003);
	sensor_write_reg16(client,0x3002, Reg0x3002);

	// Write Gain
	uint8_t Gain = 0;
	if (Capture_Gain16 > 16) {
		Capture_Gain16 = Capture_Gain16 /2;
		Gain = 0x10;
	}
	if (Capture_Gain16 > 16) {
		Capture_Gain16 = Capture_Gain16 /2;
		Gain = Gain | 0x20;
	}
	if (Capture_Gain16 > 16) {
		Capture_Gain16 = Capture_Gain16 /2;
		Gain = Gain | 0x40;
	}
	if (Capture_Gain16 > 16) {
		Capture_Gain16 = Capture_Gain16 /2;
		Gain = Gain | 0x80;
	}
	if (Capture_Gain16 > 16) {
		Gain = Gain | (Capture_Gain16 -16);
	}
	dprintk("Gain=%d\n",Gain);
	sensor_write_reg16(client,0x3000, Gain);

	p_reg(client);
}



void set_size_1600x1200(struct i2c_client *client,int mode)
{
	if(mode == CAMERA_MODE_CAPTURE)
	{
		//doing nothing
	}
}

void set_size_1280x1024(struct i2c_client *client,int mode)
{
	if(mode == CAMERA_MODE_CAPTURE)
	{
		sensor_write_reg16(client,0x3302,0x11);
		sensor_write_reg16(client,0x3088,0x05);
		sensor_write_reg16(client,0x3089,0x00);
		sensor_write_reg16(client,0x308a,0x04);
		sensor_write_reg16(client,0x308b,0x00);
		sensor_write_reg16(client,0x331a,0x50);
		sensor_write_reg16(client,0x331b,0x40);
		sensor_write_reg16(client,0x331c,0x00);
	}

	dprintk("UXGA-> SXGA;1280x1024");
}

void set_size_1024x768(struct i2c_client *client,int mode)
{
	if(mode == CAMERA_MODE_CAPTURE)
	{
		sensor_write_reg16(client,0x3011,0x01);
		sensor_write_reg16(client,0x3088,0x04);
		sensor_write_reg16(client,0x3089,0x00);
		sensor_write_reg16(client,0x308a,0x03);
		sensor_write_reg16(client,0x308b,0x00);
		sensor_write_reg16(client,0x331a,0x50);
		sensor_write_reg16(client,0x331b,0x40);
		sensor_write_reg16(client,0x3302,0x11);
	}

}

void set_size_800x600(struct i2c_client *client,int mode)
{
	if(mode == CAMERA_MODE_PREVIEW)
	{
		sensor_write_reg16(client,0x3302,0x11);
		sensor_write_reg16(client,0x3088,0x03);
		sensor_write_reg16(client,0x3089,0x20);
		sensor_write_reg16(client,0x308a,0x02);
		sensor_write_reg16(client,0x308b,0x58);
		sensor_write_reg16(client,0x331a,0x64);
		sensor_write_reg16(client,0x331b,0x4b);
		sensor_write_reg16(client,0x331c,0x00);
	}
	else if(mode == CAMERA_MODE_CAPTURE)
	{
		sensor_write_reg16(client,0x3302,0x11);
		sensor_write_reg16(client,0x3088,0x03);
		sensor_write_reg16(client,0x3089,0x20);
		sensor_write_reg16(client,0x308a,0x02);
		sensor_write_reg16(client,0x308b,0x58);
		sensor_write_reg16(client,0x331a,0x32);
		sensor_write_reg16(client,0x331b,0x25);
		sensor_write_reg16(client,0x331c,0x80);
	}

	dprintk(" UXGA>SVGA 800x600");
}

void set_size_640x480(struct i2c_client *client,int mode)
{
	if(mode == CAMERA_MODE_PREVIEW)
	{
		sensor_write_reg16(client,0x3302,0x11);
		sensor_write_reg16(client,0x3088,0x02);
		sensor_write_reg16(client,0x3089,0x80);
		sensor_write_reg16(client,0x308a,0x01);
		sensor_write_reg16(client,0x308b,0xe0);
		sensor_write_reg16(client,0x331a,0x28);
		sensor_write_reg16(client,0x331b,0x1e);
		sensor_write_reg16(client,0x331c,0x00);
		sensor_write_reg16(client,0x3302,0x11);
	}
	else if(mode == CAMERA_MODE_CAPTURE)
	{
		sensor_write_reg16(client,0x3302,0x11);
		sensor_write_reg16(client,0x3088,0x02);
		sensor_write_reg16(client,0x3089,0x80);
		sensor_write_reg16(client,0x308a,0x01);
		sensor_write_reg16(client,0x308b,0xe0);
		sensor_write_reg16(client,0x331a,0x28);
		sensor_write_reg16(client,0x331b,0x1e);
		sensor_write_reg16(client,0x331c,0x00);
	}

	dprintk(" UXGA->VGA 640x480");
}

void set_size_352x288(struct i2c_client *client,int mode)
{
	if(mode == CAMERA_MODE_PREVIEW)
	{
		sensor_write_reg16(client,0x3302,0x11);
		sensor_write_reg16(client,0x3088,0x01);
		sensor_write_reg16(client,0x3089,0x60);
		sensor_write_reg16(client,0x308a,0x01);
		sensor_write_reg16(client,0x308b,0x20);
		sensor_write_reg16(client,0x331a,0x16);
		sensor_write_reg16(client,0x331b,0x12);
		sensor_write_reg16(client,0x331c,0x00);

	}

	dprintk(" UXGA->CIF 352x288");
}

void set_size_176x144(struct i2c_client *client,int mode)
{
	if(mode == CAMERA_MODE_PREVIEW)
	{
		sensor_write_reg16(client,0x3302,0x11);
		sensor_write_reg16(client,0x3088,0x00);
		sensor_write_reg16(client,0x3089,0xb0);
		sensor_write_reg16(client,0x308a,0x00);
		sensor_write_reg16(client,0x308b,0x90);
		sensor_write_reg16(client,0x331a,0x0b);
		sensor_write_reg16(client,0x331b,0x09);
		sensor_write_reg16(client,0x331c,0x00);
	}

	dprintk(" UXGA->QCIF 176x144");
}

void size_switch(struct i2c_client *client,int width,int height,int setmode)
{
	dprintk("%dx%d - mode(%d)",width,height,setmode);

	if(width == 1600 && height == 1200)
	{
		set_size_1600x1200(client,setmode);
	}
	else if(width == 1280 && height == 1024)
	{
		set_size_1280x1024(client,setmode);
	}
	else if(width == 1024 && height == 768)
	{
		set_size_1024x768(client,setmode);
	}
	else if(width == 800 && height == 600)
	{
		set_size_800x600(client,setmode);
	}
	else if(width == 640 && height == 480)
	{
		set_size_640x480(client,setmode);
	}
	else if(width == 352 && height == 288)
	{
		set_size_352x288(client,setmode);
	}
	else if(width == 176 && height == 144)
	{
		set_size_176x144(client,setmode);
	}
	else
		return;
//	mdelay(500);
}


void ov2655_init_setting(struct i2c_client *client)
{
	//fish add for debug on 2010.6.3
	dprintk("-sensor ov2655 is initial 1");

	sensor_write_reg16(client,0x3012,0x80);

	mdelay(10);

	//IO & Clock & Analog Setup
	sensor_write_reg16(client,0x308c,0x80);
	sensor_write_reg16(client,0x308d,0x0e);
	sensor_write_reg16(client,0x360b,0x00); /* 10bit mode */
	sensor_write_reg16(client,0x30b0,0xff);
	sensor_write_reg16(client,0x30b1,0xff);   //fish mark that B[3] 1 is that pclk pin is output
	sensor_write_reg16(client,0x30b2,0x24);   //fish

	sensor_write_reg16(client,0x300e,0x34);
	sensor_write_reg16(client,0x300f,0xa6);
	sensor_write_reg16(client,0x3010,0x81);
	sensor_write_reg16(client,0x3082,0x01);
	sensor_write_reg16(client,0x30f4,0x01);
	sensor_write_reg16(client,0x3090,0x33);

	sensor_write_reg16(client,0x3091,0xc0);

	sensor_write_reg16(client,0x30ac,0x42);

	sensor_write_reg16(client,0x30d1,0x08);
	sensor_write_reg16(client,0x30a8,0x56);
	sensor_write_reg16(client,0x3015,0x01);
	sensor_write_reg16(client,0x3093,0x00);
	sensor_write_reg16(client,0x307e,0xe5);
	sensor_write_reg16(client,0x3079,0x00);
	sensor_write_reg16(client,0x30aa,0x42);
	sensor_write_reg16(client,0x3017,0x40);
	sensor_write_reg16(client,0x30f3,0x82);
	sensor_write_reg16(client,0x306a,0x0c);
	sensor_write_reg16(client,0x306d,0x00);
	sensor_write_reg16(client,0x336a,0x3c);
	sensor_write_reg16(client,0x3076,0x6a);
	sensor_write_reg16(client,0x30d9,0x95);
	sensor_write_reg16(client,0x3016,0x82);
	sensor_write_reg16(client,0x3601,0x30);
	sensor_write_reg16(client,0x304e,0x88);
	sensor_write_reg16(client,0x30f1,0x82);
	sensor_write_reg16(client,0x306f,0x14);
	sensor_write_reg16(client,0x302a,0x02);
	sensor_write_reg16(client,0x302b,0x6a);


	sensor_write_reg16(client,0x3012,0x10);
	sensor_write_reg16(client,0x3011,0x01);  //fish mark B[6] 0 is that master mode,sensor prov2655ides pclk
	//	sensor_write_reg16(client,0x302a,0x02);
	//	sensor_write_reg16(client,0x302b,0xe6);
	//	sensor_write_reg16(client,0x3028,0x07);
	//	sensor_write_reg16(client,0x3029,0x93);
	dprintk("-sensor ov2655 is initial 22222222222");

	//	sensor_write_reg16(client,0x3391,0x06);
	//	sensor_write_reg16(client,0x3394,0x38);
	//	sensor_write_reg16(client,0x3395,0x38);

	//AEC/AGC
	sensor_write_reg16(client,0x3013,0xf7);
	//	sensor_write_reg16(client,0x3018,0x78);
	//	sensor_write_reg16(client,0x3019,0x68);
	//	sensor_write_reg16(client,0x301a,0xd4);
	sensor_write_reg16(client,0x301c,0x13);
	sensor_write_reg16(client,0x301d,0x17);
	sensor_write_reg16(client,0x3070,0x5d);
	sensor_write_reg16(client,0x3072,0x4d);

	//D5060
	sensor_write_reg16(client,0x30af,0x00);
	sensor_write_reg16(client,0x3048,0x1f);
	sensor_write_reg16(client,0x3049,0x4e);
	sensor_write_reg16(client,0x304a,0x20);
	sensor_write_reg16(client,0x304f,0x20);
	sensor_write_reg16(client,0x304b,0x02);
	sensor_write_reg16(client,0x304c,0x00);
	sensor_write_reg16(client,0x304d,0x02);
	sensor_write_reg16(client,0x304f,0x20);
	sensor_write_reg16(client,0x30a3,0x10);
	sensor_write_reg16(client,0x3013,0xf7);
	sensor_write_reg16(client,0x3014,0x44); //84
	sensor_write_reg16(client,0x3071,0x00);
	sensor_write_reg16(client,0x3070,0x5d);
	sensor_write_reg16(client,0x3073,0x00);
	sensor_write_reg16(client,0x3072,0x4d);
	sensor_write_reg16(client,0x301c,0x05); //07
	sensor_write_reg16(client,0x301d,0x06); //08
	sensor_write_reg16(client,0x304d,0x42);
	sensor_write_reg16(client,0x304a,0x40);
	sensor_write_reg16(client,0x304f,0x40);
	sensor_write_reg16(client,0x3095,0x07);
	sensor_write_reg16(client,0x3096,0x16);
	sensor_write_reg16(client,0x3097,0x1d);

	//Window Setup
	sensor_write_reg16(client,0x300e,0x38);

	sensor_write_reg16(client,0x3020,0x01);
	sensor_write_reg16(client,0x3021,0x18);
	sensor_write_reg16(client,0x3022,0x00);
	sensor_write_reg16(client,0x3023,0x06);
	sensor_write_reg16(client,0x3024,0x06);
	dprintk("-sensor ov2655 is initial 33333333333");
	sensor_write_reg16(client,0x3025,0x58);
	sensor_write_reg16(client,0x3026,0x02);
	sensor_write_reg16(client,0x3027,0x61); //5e
	//800x600
	sensor_write_reg16(client,0x3088,0x03);
	sensor_write_reg16(client,0x3089,0x20);   //high
	sensor_write_reg16(client,0x308a,0x02);
	sensor_write_reg16(client,0x308b,0x58);   //width

	sensor_write_reg16(client,0x3316,0x64);
	sensor_write_reg16(client,0x3317,0x25);
	sensor_write_reg16(client,0x3318,0x80);
	sensor_write_reg16(client,0x3319,0x08);
	sensor_write_reg16(client,0x331a,0x64);
	sensor_write_reg16(client,0x331b,0x4b);
	sensor_write_reg16(client,0x331c,0x00);
	sensor_write_reg16(client,0x331d,0x38);
	sensor_write_reg16(client,0x3100,0x00);

	//AWB
	sensor_write_reg16(client,0x3320,0xfa);
	sensor_write_reg16(client,0x3321,0x11);
	sensor_write_reg16(client,0x3322,0x92);
	sensor_write_reg16(client,0x3323,0x01);
	sensor_write_reg16(client,0x3324,0x97);
	sensor_write_reg16(client,0x3325,0x02);
	sensor_write_reg16(client,0x3326,0xff);
	sensor_write_reg16(client,0x3327,0x0c);
	sensor_write_reg16(client,0x3328,0x10);
	sensor_write_reg16(client,0x3329,0x10);
	sensor_write_reg16(client,0x332a,0x58);
	sensor_write_reg16(client,0x332b,0x50); //56
	sensor_write_reg16(client,0x332c,0xbe);
	sensor_write_reg16(client,0x332d,0xe1);
	sensor_write_reg16(client,0x332e,0x43); //3a
	sensor_write_reg16(client,0x332f,0x36); //38
	sensor_write_reg16(client,0x3330,0x4d);
	sensor_write_reg16(client,0x3331,0x44);
	sensor_write_reg16(client,0x3332,0xf8);
	sensor_write_reg16(client,0x3333,0x0a);
	sensor_write_reg16(client,0x3334,0xf0);
	sensor_write_reg16(client,0x3335,0xf0);
	sensor_write_reg16(client,0x3336,0xf0);
	sensor_write_reg16(client,0x3337,0x40);
	sensor_write_reg16(client,0x3338,0x40);
	sensor_write_reg16(client,0x3339,0x40);
	sensor_write_reg16(client,0x333a,0x00);
	sensor_write_reg16(client,0x333b,0x00);

	//Color Matrix
	dprintk("-sensor ov2655 is initial 4");
	sensor_write_reg16(client,0x3380,0x28);
	sensor_write_reg16(client,0x3381,0x48);
	sensor_write_reg16(client,0x3382,0x10);
	sensor_write_reg16(client,0x3383,0x23); //22
	sensor_write_reg16(client,0x3384,0xc0);
	sensor_write_reg16(client,0x3385,0xe5); //e2
	sensor_write_reg16(client,0x3386,0xc2); //e2
	sensor_write_reg16(client,0x3387,0xb3); //f2
	sensor_write_reg16(client,0x3388,0x0e); //10
	sensor_write_reg16(client,0x3389,0x98);
	sensor_write_reg16(client,0x338a,0x01);	//00

	//Gamma
	sensor_write_reg16(client,0x3340,0x0e); //04
	sensor_write_reg16(client,0x3341,0x1a); //07
	sensor_write_reg16(client,0x3342,0x31); //19
	sensor_write_reg16(client,0x3343,0x45); //34
	sensor_write_reg16(client,0x3344,0x5a); //4a
	sensor_write_reg16(client,0x3345,0x69); //5a
	sensor_write_reg16(client,0x3346,0x75); //67
	sensor_write_reg16(client,0x3347,0x7e); //71
	sensor_write_reg16(client,0x3348,0x88); //7c
	sensor_write_reg16(client,0x3349,0x96); //8c
	sensor_write_reg16(client,0x334a,0xa3); //9b
	sensor_write_reg16(client,0x334b,0xaf); //a9
	sensor_write_reg16(client,0x334c,0xc4); //c0
	sensor_write_reg16(client,0x334d,0xd7); //d5
	sensor_write_reg16(client,0x334e,0xe8);
	sensor_write_reg16(client,0x334f,0x20);

	//Lens correction(largon 9310)
	//	sensor_write_reg16(client,0x3090,0x03);
	//	sensor_write_reg16(client,0x307c,0x10);  //fish mark 0x307c is used to reverse
	//R
	sensor_write_reg16(client,0x3350,0x32); //33
	sensor_write_reg16(client,0x3351,0x25); //28
	sensor_write_reg16(client,0x3352,0x80); //00
	sensor_write_reg16(client,0x3353,0x19); //14
	sensor_write_reg16(client,0x3354,0x00);
	sensor_write_reg16(client,0x3355,0x85);
	//G
	sensor_write_reg16(client,0x3356,0x32); //35
	sensor_write_reg16(client,0x3357,0x25); //28
	sensor_write_reg16(client,0x3358,0x80); //00
	sensor_write_reg16(client,0x3359,0x1b); //13
	sensor_write_reg16(client,0x335a,0x00);
	sensor_write_reg16(client,0x335b,0x85);
	//B
	sensor_write_reg16(client,0x335c,0x32);
	dprintk("-sensor ov2655 is initial 5");
	sensor_write_reg16(client,0x335d,0x25);
	sensor_write_reg16(client,0x335e,0x80);
	sensor_write_reg16(client,0x335f,0x1b);
	sensor_write_reg16(client,0x3360,0x00);
	sensor_write_reg16(client,0x3361,0x85);

	sensor_write_reg16(client,0x3363,0x70);
	sensor_write_reg16(client,0x3364,0x7f);
	sensor_write_reg16(client,0x3365,0x00);
	sensor_write_reg16(client,0x3366,0x00);
	//	sensor_write_reg16(client,0x3362,0x90);

	//UVadjust
	sensor_write_reg16(client,0x3301,0xff);
	sensor_write_reg16(client,0x338B,0x11);
	sensor_write_reg16(client,0x338c,0x10);
	sensor_write_reg16(client,0x338d,0x40);

	//Sharpness/De-noise
	sensor_write_reg16(client,0x3370,0xd0);
	dprintk("-sensor ov2655 is initial 6");
	sensor_write_reg16(client,0x3371,0x00);
	sensor_write_reg16(client,0x3372,0x00);
	sensor_write_reg16(client,0x3373,0x40);
	sensor_write_reg16(client,0x3374,0x10);
	sensor_write_reg16(client,0x3375,0x10);
	sensor_write_reg16(client,0x3376,0x04);
	sensor_write_reg16(client,0x3377,0x00);
	sensor_write_reg16(client,0x3378,0x04);
	sensor_write_reg16(client,0x3379,0x80);

	//BLC
	sensor_write_reg16(client,0x3069,0x84);
	//	sensor_write_reg16(client,0x3087,0x02);
	sensor_write_reg16(client,0x307c,0x10);
	sensor_write_reg16(client,0x3087,0x02);

	//Other functions
	sensor_write_reg16(client,0x3300,0xfc);
	sensor_write_reg16(client,0x3302,0x11);

	/* yuyuyuyu.../yuyvyuyv... */
#define OV2655_FMT_YUV422	0x00

	/* yyyyyyyy.../yuyuyuyv */
#define OV2655_FMT_YUV420	0x10

	/*  */
#define OV2655_FMT_Y8		0x20
#define OV2655_FMT_YUV444	0x30
#define OV2655_FMT_RGB565	0x40
#define OV2655_FMT_RGB555	0x50
#define OV2655_FMT_RGB444_1	0x60
#define OV2655_FMT_RGB444_2	0x70
#define OV2655_FMT_RGB444_3	0x80
#define OV2655_FMT_RAW		0x90
#define OV2655_FMT_YUV420_2	0xa0
#define OV2655_FMT_RGB555	0xb0

	//sensor_write_reg16(client,0x3400,0x03);    //fish mark that use YUV422
	sensor_write_reg16(client,0x3400,0x90);    //fish mark that use YUV422
	sensor_write_reg16(client,0x3606,0x20);
	sensor_write_reg16(client,0x3601,0x30);
	//	sensor_write_reg16(client,0x30f3,0x83);
	sensor_write_reg16(client,0x300e,0x34);
	sensor_write_reg16(client,0x3011,0x00);
	sensor_write_reg16(client,0x30f3,0x83);
	sensor_write_reg16(client,0x304e,0x88);

	//fish add from spec 1.54
	//    sensor_write_reg16(client,0x3090,0x03);
	//    sensor_write_reg16(client,0x30aa,0x32);
	//    sensor_write_reg16(client,0x30a3,0x80);
	//	sensor_write_reg16(client,0x30a1,0x41);
	//    sensor_write_reg16(client,0x363b,0x01);
	//    sensor_write_reg16(client,0x363c,0xf2);

	sensor_write_reg16(client,0x3086,0x0f);
	sensor_write_reg16(client,0x3086,0x00);


	dprintk("-sensor ov2655 is initial 7");

	sensor_write_reg16(client,0x3600,0x84);

}   /* ov2655_Write_Sensor_Initial_Setting */

