#include <linux/i2c.h>
#include "../../jz_sensor.h"
#include "../../jz_cim_core.h"
#include "gc0308.h"

static struct i2c_client *m_sensor_fd = NULL;
static int m_sensitivity = 0;
static int m_sensor_opt = 0;

static int m_image_reverse = 0;
static int m_image_symmetry = 0;

#ifdef GC0308_DEBUG
#define gc0308_dbg(x...)			\
	do{					\
		printk("GC0308---\t");		\
		printk(x);			\
		printk("\n");			\
	} while(0)
#else
#define gc0308_dbg(x...)
#endif

/* integration time */
//static unsigned int integration_time = 35; /* unit: ms */

/* master clock and video clock */
//static unsigned int mclk_hz = 25000000;    /* 25 MHz */
//static unsigned int vclk_div = 2;          /* VCLK = MCLK/vclk_div: 2,4,8,16,32 */

void set_ExposAdjust_GC308(void)
{
	int exposH = 0x0250;

	//无膜指纹头
	if(m_image_reverse==1)
	{
		switch(m_sensitivity)
		{
		case 0://low	//适合干手指
			//针对0308小指纹头
			if(m_image_symmetry==1)
				exposH = 0x07D0;
			else
				exposH = 0x08D0;
			break;
		case 1://middle	//干湿手指并用的情况
			//针对0308小指纹头
			if(m_image_symmetry==1)
				exposH = 0x0150;
			else
				exposH = 0x0250;
			break;
		case 2://high	//适合湿手指
			if(m_image_symmetry==1)
				exposH = 0x0100;
			else
				exposH = 0x01C0;
			break;
		default:
			if(m_image_symmetry==1)
				exposH = 0x0150;
			else
				exposH = 0x0250;

			gc0308_dbg(GC0308_DBG_MODE1, "%s() Error: Default m_sensitivity=%d\n", __FUNCTION__, m_sensitivity);
			break;
		}
	}
	//有膜指纹头，暂时没有用20100727
	else if(m_image_reverse==0)
	{
		exposH = 0x0250;
	}

	gc0308_dbg(GC0308_DBG_MODE1, "%s() exposH=0x%x\n", __FUNCTION__, exposH);
	set_EXPOS_GC308(exposH);
}

void set_BackgroundRGB_GC308(int RBlackLevel, int BBlackLevel, int GBlackLevel, int RBlackFactor, int GBlackFactor, int BBlackFactor)
{
	sensor_write_reg(m_sensor_fd, 0x85, RBlackLevel);
	sensor_write_reg(m_sensor_fd, 0x86, BBlackLevel);
	sensor_write_reg(m_sensor_fd, 0x87, GBlackLevel);
	sensor_write_reg(m_sensor_fd, 0x88, RBlackFactor);
	sensor_write_reg(m_sensor_fd, 0x89, GBlackFactor);
	sensor_write_reg(m_sensor_fd, 0x8a, BBlackFactor);
}

void set_Contrast_GC308(int contrast)
{
	sensor_write_reg(m_sensor_fd, 0xb3, contrast);
	sensor_write_reg(m_sensor_fd, 0xb4, 0x80);
}

void set_RGBGain_GCC308(int GGain, int G1Gain, int RGain, int BGain, int G2Gain)
{
	if(GGain > 0)
		sensor_write_reg(m_sensor_fd, 0x50, GGain);//Global_Gain
	if(G1Gain > 0)
		sensor_write_reg(m_sensor_fd, 0x53, G1Gain);//Gain_G1
	if(RGain > 0)
		sensor_write_reg(m_sensor_fd, 0x54, RGain);//Gain_R
	if(BGain > 0)
		sensor_write_reg(m_sensor_fd, 0x55, BGain);//Gain_B
	if(G2Gain > 0)
		sensor_write_reg(m_sensor_fd, 0x56, G2Gain);//Gain_G2
}

void _initgc308(void)
{
	//printf("%s() m_sensor_fd=%d\n", __func__,m_sensor_fd);
	sensor_write_reg(m_sensor_fd,0xfe,0x00);//[7:4]VB_High_4bit,[3:0]HB_high_bit
	sensor_write_reg(m_sensor_fd,0x01,0x6a);//hb[7:0]
	sensor_write_reg(m_sensor_fd,0x02,0x70);//vb[7:0]

	sensor_write_reg(m_sensor_fd,0xe2,0x00);
	sensor_write_reg(m_sensor_fd,0xe3,0x96);//anti-flicker step[7:0]

	sensor_write_reg(m_sensor_fd,0xe4,0x02);//exp 1
	sensor_write_reg(m_sensor_fd,0xe5,0x58);
	sensor_write_reg(m_sensor_fd,0xe6,0x03);//exp 2
	sensor_write_reg(m_sensor_fd,0xe7,0x84);
	sensor_write_reg(m_sensor_fd,0xe8,0x07);//exp 3
	sensor_write_reg(m_sensor_fd,0xe9,0x08);
	sensor_write_reg(m_sensor_fd,0xea,0x0d);//exp 4
	sensor_write_reg(m_sensor_fd,0xeb,0x7a);

	sensor_write_reg(m_sensor_fd,0xec,0x20);//[5:4]exp_level [3:0]minimum esposure high 4 bint

	sensor_write_reg(m_sensor_fd,0x03,0x00);//Expos-high 4
	sensor_write_reg(m_sensor_fd,0x04,0x96);//Expos-low 8
	sensor_write_reg(m_sensor_fd,0x05,0x00);//rpw_start_high
	sensor_write_reg(m_sensor_fd,0x06,0x00);//row_start_low
	sensor_write_reg(m_sensor_fd,0x07,0x00);//col_start_high
	sensor_write_reg(m_sensor_fd,0x08,0x00);//cpl_start_low
	sensor_write_reg(m_sensor_fd,0x09,0x01);//[8]cis_win_height 488
	sensor_write_reg(m_sensor_fd,0x0a,0xe8);//[7:0]cis_win_height
	sensor_write_reg(m_sensor_fd,0x0b,0x02);//[9:8]cis_win_width 648
	sensor_write_reg(m_sensor_fd,0x0c,0x88);//[7:0]cis_win_width
	sensor_write_reg(m_sensor_fd,0x0d,0x02);//vs_st
	sensor_write_reg(m_sensor_fd,0x0e,0x02);//vs_et

	sensor_write_reg(m_sensor_fd,0x10,0x26);//[7:4]restg_with,[3:0]sh_width
	sensor_write_reg(m_sensor_fd,0x11,0x0d);//fd [7:4]tx_width [3:0]space wdth,*2
	sensor_write_reg(m_sensor_fd,0x12,0x2a);//sh_delay
	sensor_write_reg(m_sensor_fd,0x13,0x00);//[3:0] row_tail_width
	sensor_write_reg(m_sensor_fd,0x14,0x10);//[7]hsync_always[6]NA,[5:4]CFA sequence [3:2]NA,[1]upside_down,[0]mirrir
	sensor_write_reg(m_sensor_fd,0x15,0x0a);//[7:6]output_mode,[5:4]restg_mode[3:2]sdark_mode,[1]new exposure
	sensor_write_reg(m_sensor_fd,0x16,0x05);//[7:5]NA,[4]capture_ad_edge,[3:0]Number of A/D pipe stages
	sensor_write_reg(m_sensor_fd,0x17,0x01);//[7:6]analog_opa_r [5]coltest_en[4]ad_test_enalble,[3]tz_allow[2]balck sun correction[1:0]black su control reg //xsen test 01
	sensor_write_reg(m_sensor_fd,0x18,0x44);//[7]NA[6:4]column gain ee [3]NA[2:0]column gain eo
	sensor_write_reg(m_sensor_fd,0x19,0x44);//[7]NA[6:4]column gain oe [3]NA[2:0]column gain oo
	sensor_write_reg(m_sensor_fd,0x1a,0x2a);
	sensor_write_reg(m_sensor_fd,0x1b,0x00);
	sensor_write_reg(m_sensor_fd,0x1c,0x49);
	sensor_write_reg(m_sensor_fd,0x1d,0x9a);
	sensor_write_reg(m_sensor_fd,0x1e,0x61);
	sensor_write_reg(m_sensor_fd,0x1f,0x16);

	sensor_write_reg(m_sensor_fd,0x20,0x4a);//xsen test, ff
	sensor_write_reg(m_sensor_fd,0x21,0xfa);
	sensor_write_reg(m_sensor_fd,0x22,0x57);
	sensor_write_reg(m_sensor_fd,0x24,0xa2);
	sensor_write_reg(m_sensor_fd,0x25,0x0f);
	/*output sync_mode*/
#ifdef ZEM800
	sensor_write_reg(m_sensor_fd,0x26,0x02);
#else
	sensor_write_reg(m_sensor_fd,0x26,0x3f);
#endif
	sensor_write_reg(m_sensor_fd,0x2f,0x01);
	/*grab*/
	sensor_write_reg(m_sensor_fd,0x30,0xf7);
	sensor_write_reg(m_sensor_fd,0x31,0x50);
	sensor_write_reg(m_sensor_fd,0x32,0x00);
	sensor_write_reg(m_sensor_fd,0x39,0x04);
	sensor_write_reg(m_sensor_fd,0x3a,0x20);
	sensor_write_reg(m_sensor_fd,0x3b,0x20);
	sensor_write_reg(m_sensor_fd,0x3c,0x00);
	sensor_write_reg(m_sensor_fd,0x3d,0x00);
	sensor_write_reg(m_sensor_fd,0x3e,0x00);
	sensor_write_reg(m_sensor_fd,0x3f,0x00);
	/*gain*/
	sensor_write_reg(m_sensor_fd,0x50,0x14);

	sensor_write_reg(m_sensor_fd,0x53,0x80);
	sensor_write_reg(m_sensor_fd,0x54,0x80);
	sensor_write_reg(m_sensor_fd,0x55,0x88);
	sensor_write_reg(m_sensor_fd,0x56,0x80);
	/*LSC_t*/
	sensor_write_reg(m_sensor_fd,0x8b,0x10);
	sensor_write_reg(m_sensor_fd,0x8c,0x10);
	sensor_write_reg(m_sensor_fd,0x8d,0x10);
	sensor_write_reg(m_sensor_fd,0x8e,0x10);
	sensor_write_reg(m_sensor_fd,0x8f,0x10);
	sensor_write_reg(m_sensor_fd,0x90,0x10);
	sensor_write_reg(m_sensor_fd,0x91,0x3c);
	sensor_write_reg(m_sensor_fd,0x92,0x50);
	sensor_write_reg(m_sensor_fd,0x5d,0x12);
	sensor_write_reg(m_sensor_fd,0x5e,0x1a);
	sensor_write_reg(m_sensor_fd,0x5f,0x24);
	/*DNDD_t*/
	sensor_write_reg(m_sensor_fd,0x60,0x07);
	sensor_write_reg(m_sensor_fd,0x61,0x15);
	sensor_write_reg(m_sensor_fd,0x62,0x08);
	sensor_write_reg(m_sensor_fd,0x64,0x03);
	sensor_write_reg(m_sensor_fd,0x66,0xe8);
	sensor_write_reg(m_sensor_fd,0x67,0x86);
	sensor_write_reg(m_sensor_fd,0x68,0xa2);
	/*asde_t*/
	sensor_write_reg(m_sensor_fd,0x69,0x18);//18,xsen test
	sensor_write_reg(m_sensor_fd,0x6a,0x0f);
	sensor_write_reg(m_sensor_fd,0x6b,0x00);
	sensor_write_reg(m_sensor_fd,0x6c,0x5f);
	sensor_write_reg(m_sensor_fd,0x6d,0x8f);
	sensor_write_reg(m_sensor_fd,0x6e,0x55);
	sensor_write_reg(m_sensor_fd,0x6f,0x38);
	sensor_write_reg(m_sensor_fd,0x70,0x15);
	sensor_write_reg(m_sensor_fd,0x71,0x33);
	/*eeintp_t*/
	sensor_write_reg(m_sensor_fd,0x72,0xdc);
	sensor_write_reg(m_sensor_fd,0x73,0x80);

	/*for high resolution in light scene*/
	sensor_write_reg(m_sensor_fd,0x74,0x02);
	sensor_write_reg(m_sensor_fd,0x75,0x3f);
	sensor_write_reg(m_sensor_fd,0x76,0x02);
	sensor_write_reg(m_sensor_fd,0x77,0x47);
	sensor_write_reg(m_sensor_fd,0x78,0x88);
	sensor_write_reg(m_sensor_fd,0x79,0x81);
	sensor_write_reg(m_sensor_fd,0x7a,0x81);
	sensor_write_reg(m_sensor_fd,0x7b,0x22);
	sensor_write_reg(m_sensor_fd,0x7c,0xff);
	/*cct*/
	sensor_write_reg(m_sensor_fd,0x93,0x40);//46
	sensor_write_reg(m_sensor_fd,0x94,0x00);//00
	sensor_write_reg(m_sensor_fd,0x95,0x03);//03
	sensor_write_reg(m_sensor_fd,0x96,0xd0);//d0
	sensor_write_reg(m_sensor_fd,0x97,0x40);//40
	sensor_write_reg(m_sensor_fd,0x98,0xf0);//f0
	/*YCPT*/
	sensor_write_reg(m_sensor_fd,0xb1,0x40);
	sensor_write_reg(m_sensor_fd,0xb2,0x40);
	sensor_write_reg(m_sensor_fd,0xb3,0x40);
	sensor_write_reg(m_sensor_fd,0xb6,0xe0);
	sensor_write_reg(m_sensor_fd,0xbd,0x3c);
	sensor_write_reg(m_sensor_fd,0xbe,0x36);

	/*AECT*/
	sensor_write_reg(m_sensor_fd,0xd0,0xc9);
	sensor_write_reg(m_sensor_fd,0xd1,0x10);
	sensor_write_reg(m_sensor_fd,0xd2,0x10);//xsen test 90, 10:AEC disable
	sensor_write_reg(m_sensor_fd,0xd3,0x88);
	sensor_write_reg(m_sensor_fd,0xd5,0xf2);
	sensor_write_reg(m_sensor_fd,0xd6,0x10);
	sensor_write_reg(m_sensor_fd,0xdb,0x92);
	sensor_write_reg(m_sensor_fd,0xdc,0xa5);
	sensor_write_reg(m_sensor_fd,0xdf,0x23);


	sensor_write_reg(m_sensor_fd,0xd9,0x00);
	sensor_write_reg(m_sensor_fd,0xda,0x00);
	sensor_write_reg(m_sensor_fd,0xe0,0x09);

	sensor_write_reg(m_sensor_fd,0xed,0x04);
	sensor_write_reg(m_sensor_fd,0xee,0xa0);
	sensor_write_reg(m_sensor_fd,0xef,0x40);
	/*ABBT*/
	sensor_write_reg(m_sensor_fd,0x80,0x03);
	/*RGB_gamma_m5*/
	sensor_write_reg(m_sensor_fd,0x9f,0x0e);
	sensor_write_reg(m_sensor_fd,0xa0,0x1c);
	sensor_write_reg(m_sensor_fd,0xa1,0x34);
	sensor_write_reg(m_sensor_fd,0xa2,0x48);
	sensor_write_reg(m_sensor_fd,0xa3,0x5a);
	sensor_write_reg(m_sensor_fd,0xa4,0x6b);
	sensor_write_reg(m_sensor_fd,0xa5,0x7b);
	sensor_write_reg(m_sensor_fd,0xa6,0x95);
	sensor_write_reg(m_sensor_fd,0xa7,0xab);
	sensor_write_reg(m_sensor_fd,0xa8,0xbf);
	sensor_write_reg(m_sensor_fd,0xa9,0xce);
	sensor_write_reg(m_sensor_fd,0xaa,0xd9);
	sensor_write_reg(m_sensor_fd,0xab,0xe4);
	sensor_write_reg(m_sensor_fd,0xac,0xec);
	sensor_write_reg(m_sensor_fd,0xad,0xf7);
	sensor_write_reg(m_sensor_fd,0xae,0xfd);
	sensor_write_reg(m_sensor_fd,0xaf,0xff);

	/*
 	*wint Y_gamma
 	*/
	sensor_write_reg(m_sensor_fd,0xc0,0x00);
	sensor_write_reg(m_sensor_fd,0xc1,0x14);
	sensor_write_reg(m_sensor_fd,0xc2,0x21);
	sensor_write_reg(m_sensor_fd,0xc3,0x36);
	sensor_write_reg(m_sensor_fd,0xc4,0x49);
	sensor_write_reg(m_sensor_fd,0xc5,0x5b);
	sensor_write_reg(m_sensor_fd,0xc6,0x6b);
	sensor_write_reg(m_sensor_fd,0xc7,0x7b);
	sensor_write_reg(m_sensor_fd,0xc8,0x98);
	sensor_write_reg(m_sensor_fd,0xc9,0xb4);
	sensor_write_reg(m_sensor_fd,0xca,0xce);
	sensor_write_reg(m_sensor_fd,0xcb,0xe8);
	sensor_write_reg(m_sensor_fd,0xcc,0xff);
	/*ABS*/
	sensor_write_reg(m_sensor_fd,0xf0,0x02);
	sensor_write_reg(m_sensor_fd,0xf1,0x01);
	sensor_write_reg(m_sensor_fd,0xf2,0x04);
	sensor_write_reg(m_sensor_fd,0xf3,0x30);
	sensor_write_reg(m_sensor_fd,0xf9,0x9f);
	sensor_write_reg(m_sensor_fd,0xfa,0x78);
	/*AWBp*/
	sensor_write_reg(m_sensor_fd,0xfe,0x01);
	sensor_write_reg(m_sensor_fd,0x00,0xf5);
	sensor_write_reg(m_sensor_fd,0x02,0x20);
	sensor_write_reg(m_sensor_fd,0x04,0x10);
	sensor_write_reg(m_sensor_fd,0x05,0x10);
	sensor_write_reg(m_sensor_fd,0x06,0x20);
	sensor_write_reg(m_sensor_fd,0x08,0x15);
	sensor_write_reg(m_sensor_fd,0x0a,0xa0);
	sensor_write_reg(m_sensor_fd,0x0b,0x64);
	sensor_write_reg(m_sensor_fd,0x0c,0x08);
	sensor_write_reg(m_sensor_fd,0x0e,0x4c);
	sensor_write_reg(m_sensor_fd,0x0f,0x39);
	sensor_write_reg(m_sensor_fd,0x10,0x41);
	sensor_write_reg(m_sensor_fd,0x11,0x37);
	sensor_write_reg(m_sensor_fd,0x12,0x24);
	sensor_write_reg(m_sensor_fd,0x13,0x39);
	sensor_write_reg(m_sensor_fd,0x14,0x42);
	sensor_write_reg(m_sensor_fd,0x15,0x42);
	sensor_write_reg(m_sensor_fd,0x16,0xc2);
	sensor_write_reg(m_sensor_fd,0x17,0xa8);
	sensor_write_reg(m_sensor_fd,0x18,0x18);
	sensor_write_reg(m_sensor_fd,0x19,0x55);
	sensor_write_reg(m_sensor_fd,0x1a,0xd8);
	sensor_write_reg(m_sensor_fd,0x1b,0xf5);
	sensor_write_reg(m_sensor_fd,0x70,0x40);
	sensor_write_reg(m_sensor_fd,0x71,0x58);
	sensor_write_reg(m_sensor_fd,0x72,0x30);
	sensor_write_reg(m_sensor_fd,0x73,0x48);
	sensor_write_reg(m_sensor_fd,0x74,0x20);
	sensor_write_reg(m_sensor_fd,0x75,0x60);
	sensor_write_reg(m_sensor_fd,0x77,0x20);
	sensor_write_reg(m_sensor_fd,0x78,0x32);
	/*HSP*/
	/*page1*/
	sensor_write_reg(m_sensor_fd,0x30,0x03);
	sensor_write_reg(m_sensor_fd,0x31,0x40);
	sensor_write_reg(m_sensor_fd,0x32,0x10);
	sensor_write_reg(m_sensor_fd,0x33,0xe0);
	sensor_write_reg(m_sensor_fd,0x34,0xe0);
	sensor_write_reg(m_sensor_fd,0x35,0x00);
	sensor_write_reg(m_sensor_fd,0x36,0x80);
	sensor_write_reg(m_sensor_fd,0x37,0x00);
	sensor_write_reg(m_sensor_fd,0x38,0x04);
	sensor_write_reg(m_sensor_fd,0x39,0x09);
	sensor_write_reg(m_sensor_fd,0x3a,0x12);
	sensor_write_reg(m_sensor_fd,0x3b,0x1c);
	sensor_write_reg(m_sensor_fd,0x3c,0x28);
	sensor_write_reg(m_sensor_fd,0x3d,0x31);
	sensor_write_reg(m_sensor_fd,0x3e,0x44);
	sensor_write_reg(m_sensor_fd,0x3f,0x57);
	sensor_write_reg(m_sensor_fd,0x40,0x6c);
	sensor_write_reg(m_sensor_fd,0x41,0x81);
	sensor_write_reg(m_sensor_fd,0x42,0x94);
	sensor_write_reg(m_sensor_fd,0x43,0xa7);
	sensor_write_reg(m_sensor_fd,0x44,0xb8);
	sensor_write_reg(m_sensor_fd,0x45,0xd6);
	sensor_write_reg(m_sensor_fd,0x46,0xee);
	sensor_write_reg(m_sensor_fd,0x47,0x0d);
	/*OUT*/
	sensor_write_reg(m_sensor_fd,0xfe,0x00);
}

void select_page_gc308(int page)
{
	sensor_write_reg(m_sensor_fd, 0xFE, page);
}

static void set_window_GC308(int l, int t, int w, int h)
{
	/* Set the row start address */
	sensor_write_reg(m_sensor_fd,GC0308_ROW_H, (t >> 8) & 0xff);
	sensor_write_reg(m_sensor_fd,GC0308_ROW_L, t & 0xff);

	/* Set the column start address */
	sensor_write_reg(m_sensor_fd,GC0308_COL_H, (l >> 8) & 0xff);
	sensor_write_reg(m_sensor_fd,GC0308_COL_L, l & 0xff);

	/* Set the image window width*/
	sensor_write_reg(m_sensor_fd,GC0308_WINW_H, ((w+8)>> 8) & 0xff);
	sensor_write_reg(m_sensor_fd,GC0308_WINW_L, (w+8) & 0xff);

	/* Set the image window height*/
	sensor_write_reg(m_sensor_fd,GC0308_WINH_H, ((h+8) >> 8) & 0xff);
	sensor_write_reg(m_sensor_fd,GC0308_WINH_L, (h+8) & 0xff);
}

/* VCLK = MCLK/div */
static void set_sensor_clock_gc308(int div)
{
	/* ABLC enable */
	int FrequencyNumber=div&0x0F;
	int FrequencyDiv=(div>>4)&0x0F;
	switch (FrequencyNumber+FrequencyDiv) {
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
		sensor_write_reg(m_sensor_fd,GC0308_SCTRA, (FrequencyNumber+FrequencyDiv-1)<<4 | FrequencyDiv);       // DCF=MCLK
		break;
	default:
		sensor_write_reg(m_sensor_fd,GC0308_SCTRA, 0x00);       // DCF=MCLK
		break;
	}
}

void set_EXPOS_GC308(int exposH)
{
	//int value=sensor_read_reg_nostop(m_sensor_fd, 0xd0);
	//sensor_write_reg(m_sensor_fd, 0xd0, value | 0x20);
	sensor_write_reg(m_sensor_fd, GC0308_EXP_H, (exposH>>8)&0xff);
	sensor_write_reg(m_sensor_fd, GC0308_EXP_L, exposH&0xff);
}

void set_vertical_GC308(int vertical)
{
	int value=0;
	value=sensor_read_reg_nostop(m_sensor_fd, 0x14);
	if(vertical)
		sensor_write_reg(m_sensor_fd, 0x14, value | 0x02);
	else
		sensor_write_reg(m_sensor_fd, 0x14, value & (~0x02));
}

//设置图像是否需要左右镜像处理
void set_symmetry_GC308(int symmetry)
{
	int value=sensor_read_reg_nostop(m_sensor_fd, 0x14);
	if(symmetry==1)
		sensor_write_reg(m_sensor_fd, 0x14, value | 0x01);//图像需要垂直翻转
	else
		sensor_write_reg(m_sensor_fd, 0x14, value & (~0x01));//图像不需要垂直翻转
}

void sensor_power_on_GC308(void)
{
	unsigned char value;
	value = sensor_read_reg_nostop(m_sensor_fd, 0x24);
#ifdef ZEM800
	sensor_write_reg(m_sensor_fd, 0x24, value & 0xe0);
#else
	sensor_write_reg(m_sensor_fd, 0x24, 0xF1);
#endif
	value = sensor_read_reg_nostop(m_sensor_fd, 0x25);
	sensor_write_reg(m_sensor_fd, 0x25, value | 0x0F);
}

void sensor_power_down_GC308(void)
{
	unsigned char value;
	value = sensor_read_reg_nostop(m_sensor_fd, 0x25);
	sensor_write_reg(m_sensor_fd, 0x25, value & (~0x0F));
}

//****************************************************************
//对外接口部分
void sensor_init_GC308(int left, int top, int width, int height)
{
	gc0308_dbg("%s() int left=%d, int top=%d, int width=%d, int height=%d\n", __FUNCTION__, left, top, width, height);

	_initgc308();

	select_page_gc308(0);

	set_window_GC308(left, top, width, height);

	set_sensor_clock_gc308(0x00);//xsen test

	//set_BackgroundRGB_GC308(0x08, 0x09, 0x0a, 0x09, 0x0b, 0x0d);

	set_EXPOS_GC308(0x280);

	//set_Contrast_GC308(0x50);

	set_vertical_GC308(1);

	set_RGBGain_GCC308(0x50, 0x55, 0x60, 0x60, 0x55);

	//打开指纹头的电源
	sensor_power_on_GC308();

	//设置曝光度
	//set_sensitivity_GC307();
}

void set_gc0308_i2c_client(struct i2c_client *client) {
	m_sensor_fd = client;
}
