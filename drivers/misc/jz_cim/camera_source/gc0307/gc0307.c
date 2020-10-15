#include <linux/i2c.h>
#include "../../jz_sensor.h"
#include "../../jz_cim_core.h"
#include "gc0307.h"

static struct i2c_client *m_sensor_fd = NULL;
static int m_sensitivity = 0;
static int m_sensor_opt = 0;

static int m_image_reverse = 0;
static int m_image_symmetry = 0;

#ifdef GC0308_DEBUG
#define gc0307_dbg(x...)			\
	do{					\
		printk("GC0307---\t");		\
		printk(x);			\
		printk("\n");			\
	} while(0)
#else
#define gc0307_dbg(x...)
#endif

/* integration time */
//static unsigned int integration_time = 35; /* unit: ms */

/* master clock and video clock */
//static unsigned int mclk_hz = 25000000;    /* 25 MHz */
//static unsigned int vclk_div = 2;          /* VCLK = MCLK/vclk_div: 2,4,8,16,32 */

void sensor_power_down_GC307(void)
{
	unsigned char v;
	sensor_write_reg(m_sensor_fd,	0x43  ,0x00);
	v = sensor_read_reg_nostop(m_sensor_fd,0x44);
	/* clear output_enable */
	sensor_write_reg(m_sensor_fd,	0x44  ,v&~(0x40));
}


void set_sensitivity_GC307()
{
	int exposH = 0x0250;
#ifdef ZEM800
	exposH = 0x0400;
#endif

#if 0			      /* slcao */
	if(m_sensor_opt!=OPT_FLAG_GC0307)
		zkprintf(ZKPRINTF_MODE1, "%s() doesn't support 0307 m_sensor_opt=%d\n", __FUNCTION__, m_sensor_opt);
#endif

	//无膜指纹头
	if(m_image_reverse==1)
	{
		switch(m_sensitivity)
		{
		case 0://low	//适合干手指
			//针对0307小指纹头
#ifdef ZEM800
			exposH = 0x300;
#else
			if(m_image_symmetry==1)
				exposH = 0x07D0;
			else
				exposH = 0x08D0;
#endif
			break;
		case 1://middle	//干湿手指并用的情况
			//针对0307小指纹头
#ifdef ZEM800
			exposH = 0x400;
#else
			if(m_image_symmetry==1)
				exposH = 0x0150;
			else
				exposH = 0x0250;
#endif
			break;
		case 2://high	//适合湿手指
#ifdef ZEM800
			exposH = 0x500;
#else
			if(m_image_symmetry==1)
				exposH = 0x0100;
			else
				exposH = 0x01C0;
#endif
			break;
		default:
#ifdef ZEM800
			exposH = 0x400;
#else
			if(m_image_symmetry==1)
				exposH = 0x0150;
			else
				exposH = 0x0250;
#endif

			gc0307_dbg("%s() Error: Default m_sensitivity=%d\n", __FUNCTION__, m_sensitivity);
			break;
		}
	}
	//有膜指纹头，暂时没有用20100727
	else if(m_image_reverse==0)
	{
#ifdef ZEM800
		exposH = 0x0400;
#else
		exposH = 0x0250;
#endif
	}

	gc0307_dbg("%s() exposH=0x%x\n", __FUNCTION__, exposH);
	set_EXPOS_GC307(exposH);
}

//////////	end
//****************************************************************

//设置图像是否需要左右镜像处理
void set_symmetry_GC307(int symmetry)
{
	int value = sensor_read_reg_nostop(m_sensor_fd, 0x0F);
	if(symmetry==1)
		sensor_write_reg(m_sensor_fd, 0x0F, value | 0x30);//图像需要垂直翻转
	else
		sensor_write_reg(m_sensor_fd, 0x0F, value & (~0x10));//图像不需要垂直翻转
}

void sensor_power_on_GC307(void)
{
	unsigned char v;
	sensor_write_reg(m_sensor_fd,	0x43  ,0x40);
	v = sensor_read_reg_nostop(m_sensor_fd, 0x44);
	sensor_write_reg(m_sensor_fd,	0x44  ,v|0x40);
}

static void set_window_GC307(int l, int t, int w, int h)
{
	int tp=0;

#ifdef ZEM800
	l = (640 - w)/2*2;
	t = (480 - h)/2*2;
#else
 	if((w+l)>640)
  	{
  		tp = w+l-640;
  		l -= tp;
  	}

  	if(l<0)
  		l = 0;

  	if( (t+h)>480 )
  	{
  		tp = h+t-480;
  		t -= tp;
  	}
 	if(t<0)
  		t = 0;
#endif

	/* Set the row start address */
	sensor_write_reg(m_sensor_fd,GC0307_ROW_H, (t >> 8) & 0xff);
	sensor_write_reg(m_sensor_fd,GC0307_ROW_L, t & 0xff);

	/* Set the column start address */
	sensor_write_reg(m_sensor_fd,GC0307_COL_H, (l >> 8) & 0xff);
	sensor_write_reg(m_sensor_fd,GC0307_COL_L, l & 0xff);

	/* Set the image window width*/
	sensor_write_reg(m_sensor_fd,GC0307_WINW_H, (w >> 8) & 0xff);
	sensor_write_reg(m_sensor_fd,GC0307_WINW_L, w & 0xff);

	/* Set the image window height*/
#ifdef ZEM800
	sensor_write_reg(m_sensor_fd,GC0307_WINH_H, ((h+8) >> 8) & 0xff);
	sensor_write_reg(m_sensor_fd,GC0307_WINH_L, (h+8) & 0xff);
#else
	sensor_write_reg(m_sensor_fd,GC0307_WINH_H, ((h + 8) >> 8) & 0xff);
	sensor_write_reg(m_sensor_fd,GC0307_WINH_L, (h + 8) & 0xff);
#endif
}

/* VCLK = MCLK/div */
static void set_sensor_clock_gc307(int div)
{
	#define ABLC_EN (0xC0)
	/* ABLC enable */
	switch (div) {
	case 2:
		sensor_write_reg(m_sensor_fd,GC0307_SCTRA, ABLC_EN | 0x00);       // DCF=MCLK
		break;
	case 4:
		sensor_write_reg(m_sensor_fd,GC0307_SCTRA, ABLC_EN | 0x04);       // DCF=MCLK/2
		break;
	case 8:
		sensor_write_reg(m_sensor_fd,GC0307_SCTRA, ABLC_EN | 0x08);       // DCF=MCLK/4
		break;
	case 16:
		sensor_write_reg(m_sensor_fd,GC0307_SCTRA, ABLC_EN | 0x0C);       // DCF=MCLK/8
		break;
	default:
		gc0307_dbg("%s() Error: Default div=%d\n", __FUNCTION__, div);
		break;
	}
}

void set_EXPOS_GC307(int exposH)
{
	sensor_write_reg(m_sensor_fd,GC0307_EXP_H, (exposH>>8)&0xff);
	sensor_write_reg(m_sensor_fd,GC0307_EXP_L, exposH&0xff);
}


void select_page_gc307(int page)
{
	sensor_write_reg(m_sensor_fd,0xF0,page);
}

void _initgc307(void)
{
   	sensor_write_reg(m_sensor_fd,0xf0  ,0x00);
   	sensor_write_reg(m_sensor_fd,0x43  ,0x00);
	sensor_write_reg(m_sensor_fd,0x44  ,0xa2);

   	sensor_write_reg(m_sensor_fd,0x03  ,0x00);
	sensor_write_reg(m_sensor_fd,0x04  ,0x90);
	//========= close some functions
	// open them after configure their parmameters
	sensor_write_reg(m_sensor_fd,0x40  ,0x10);
	sensor_write_reg(m_sensor_fd,0x41  ,0x00);
	sensor_write_reg(m_sensor_fd,0x42  ,0x10);
	sensor_write_reg(m_sensor_fd,0x47  ,0x00);//mode1,
	//sensor_write_reg(m_sensor_fd,0x48  ,0xc3);//0xc3);//mode2,
	sensor_write_reg(m_sensor_fd,0x49  ,0x00);//dither_mode
	sensor_write_reg(m_sensor_fd,0x4a  ,0x00);//clock_gating_en
	sensor_write_reg(m_sensor_fd,0x4b  ,0x00);//mode_reg3
#ifdef ZEM800
	sensor_write_reg(m_sensor_fd,0x4E  ,0x23);//sync mode for zem800
#else
	sensor_write_reg(m_sensor_fd,0x4E  ,0x22);//sync mode for zem600
#endif
	//sensor_write_reg(m_sensor_fd,0x4E  ,0x1e);//sync mode
	sensor_write_reg(m_sensor_fd,0x4F  ,0x01);//AWB, AEC, every N frame

	//========= frame timing
	sensor_write_reg(m_sensor_fd,0x01  ,0x6a);//HB
#ifdef ZEM800
	sensor_write_reg(m_sensor_fd,0x02  ,0x25);//VB
#else
	sensor_write_reg(m_sensor_fd,0x02  ,0x0c);//VB
#endif
	sensor_write_reg(m_sensor_fd,0x1C  ,0x00);//Vs_st
	sensor_write_reg(m_sensor_fd,0x1D  ,0x00);//Vs_et
	sensor_write_reg(m_sensor_fd,0x10  ,0x00);//high 4 bits of VB, HB
	sensor_write_reg(m_sensor_fd,0x11  ,0x05);//row_tail,  AD_pipe_number


	//========= windowing
	sensor_write_reg(m_sensor_fd,0x05  ,0x00);//row_start
	sensor_write_reg(m_sensor_fd,0x06  ,0x00);
	sensor_write_reg(m_sensor_fd,0x07  ,0x00);//col start
	sensor_write_reg(m_sensor_fd,0x08  ,0x00);
	sensor_write_reg(m_sensor_fd,0x09  ,0x01);//win height
	sensor_write_reg(m_sensor_fd,0x0A  ,0xE8);
	sensor_write_reg(m_sensor_fd,0x0B  ,0x02);//win width, pixel array only 640
	sensor_write_reg(m_sensor_fd,0x0C  ,0x80);

	//========= analog
	sensor_write_reg(m_sensor_fd,0x0D  ,0x22);//rsh_width
#ifdef ZEM800
	sensor_write_reg(m_sensor_fd,0x0E  ,0x02);//CISCTL mode2,
#else
	sensor_write_reg(m_sensor_fd,0x0E  ,0x42);//CISCTL mode2,
#endif


	sensor_write_reg(m_sensor_fd,0x12  ,0x70);//7 hrst, 6_4 darsg,
	sensor_write_reg(m_sensor_fd,0x13  ,0x00);//7 CISCTL_restart, 0 apwd
	sensor_write_reg(m_sensor_fd,0x14  ,0x00);//NA
	sensor_write_reg(m_sensor_fd,0x15  ,0xba);//7_4 vref
	sensor_write_reg(m_sensor_fd,0x16  ,0x13);//5to4 _coln_r,  __1to0__da18
	sensor_write_reg(m_sensor_fd,0x17  ,0x52);//opa_r, ref_r, sRef_r
	sensor_write_reg(m_sensor_fd,0x18  ,0xc0);//analog_mode, best case for left band.

	sensor_write_reg(m_sensor_fd,0x1E  ,0x0d);//tsp_width
	sensor_write_reg(m_sensor_fd,0x1F  ,0x32);//sh_delay

	//========= offset
	sensor_write_reg(m_sensor_fd,0x47  ,0x00); //7__test_image, __6__fixed_pga, __5__auto_DN, __4__CbCr_fix,

	sensor_write_reg(m_sensor_fd,0x19  ,0x06); //pga_o
	sensor_write_reg(m_sensor_fd,0x1a  ,0x06); //pga_e

	sensor_write_reg(m_sensor_fd,0x31  ,0x00); //4	//pga_oFFset ,	 high 8bits of 11bits
	sensor_write_reg(m_sensor_fd,0x3B  ,0x00); //global_oFFset, low 8bits of 11bits

	sensor_write_reg(m_sensor_fd,0x59  ,0x0f); //offset_mode
	sensor_write_reg(m_sensor_fd,0x58  ,0x88); //DARK_VALUE_RATIO_G,  DARK_VALUE_RATIO_RB
	sensor_write_reg(m_sensor_fd,0x57  ,0x08); //DARK_CURRENT_RATE
	sensor_write_reg(m_sensor_fd,0x56  ,0x77); //PGA_OFFSET_EVEN_RATIO, PGA_OFFSET_ODD_RATIO

	//========= blk
	sensor_write_reg(m_sensor_fd,0x35  ,0xd8); //blk_mode

	sensor_write_reg(m_sensor_fd,0x36  ,0x40);

	sensor_write_reg(m_sensor_fd,0x3C  ,0x00);
	sensor_write_reg(m_sensor_fd,0x3D  ,0x00);
	sensor_write_reg(m_sensor_fd,0x3E  ,0x00);
	sensor_write_reg(m_sensor_fd,0x3F  ,0x00);

	sensor_write_reg(m_sensor_fd,0xb5  ,0x70);
	sensor_write_reg(m_sensor_fd,0xb6  ,0x40);
	sensor_write_reg(m_sensor_fd,0xb7  ,0x00);
	sensor_write_reg(m_sensor_fd,0xb8  ,0x38);
	sensor_write_reg(m_sensor_fd,0xb9  ,0xc3);
	sensor_write_reg(m_sensor_fd,0xba  ,0x0f);

	sensor_write_reg(m_sensor_fd,0x7e  ,0x45);
	sensor_write_reg(m_sensor_fd,0x7f  ,0x66);

	sensor_write_reg(m_sensor_fd,0x5c  ,0x48);//78
	sensor_write_reg(m_sensor_fd,0x5d  ,0x58);//88


	//========= manual_gain
	sensor_write_reg(m_sensor_fd,0x61  ,0x80);//manual_gain_g1
#ifdef ZEM800
	sensor_write_reg(m_sensor_fd,0x63  ,0x80);//0x80);//manual_gain_r
	sensor_write_reg(m_sensor_fd,0x65  ,0x98);//98);//0x98);//manual_gai_b, 0xa0=1.25, 0x98=1.1875
#else
	sensor_write_reg(m_sensor_fd,0x63  ,0x9B);//0x80);//manual_gain_r
	sensor_write_reg(m_sensor_fd,0x65  ,0xAC);//98);//0x98);//manual_gai_b, 0xa0=1.25, 0x98=1.1875
#endif
	sensor_write_reg(m_sensor_fd,0x67  ,0x80);//manual_gain_g2
	sensor_write_reg(m_sensor_fd,0x68  ,0x18);//global_manual_gain	 2.4bits

	//=========CC _R
	sensor_write_reg(m_sensor_fd,0x69  ,0x58); //54
	sensor_write_reg(m_sensor_fd,0x6A  ,0xf6); //ff
	sensor_write_reg(m_sensor_fd,0x6B  ,0xfb); //fe
	sensor_write_reg(m_sensor_fd,0x6C  ,0xf4); //ff
	sensor_write_reg(m_sensor_fd,0x6D  ,0x5a); //5f
	sensor_write_reg(m_sensor_fd,0x6E  ,0xe6); //e1

	sensor_write_reg(m_sensor_fd,0x6f  ,0x00);

	//=========lsc
	sensor_write_reg(m_sensor_fd,0x70  ,0x14);
	sensor_write_reg(m_sensor_fd,0x71  ,0x1c);
	sensor_write_reg(m_sensor_fd,0x72  ,0x20);

	sensor_write_reg(m_sensor_fd,0x73  ,0x10);
	sensor_write_reg(m_sensor_fd,0x74  ,0x3c);
	sensor_write_reg(m_sensor_fd,0x75  ,0x52);

	//=========dn
	sensor_write_reg(m_sensor_fd,0x7d  ,0x2f); //dn_mode
	sensor_write_reg(m_sensor_fd,0x80  ,0x0c);//when auto_dn, check 7e,7f
	sensor_write_reg(m_sensor_fd,0x81  ,0x0c);
	sensor_write_reg(m_sensor_fd,0x82  ,0x44);

	//dd
	sensor_write_reg(m_sensor_fd,0x83  ,0x18); //DD_TH1
	sensor_write_reg(m_sensor_fd,0x84  ,0x18); //DD_TH2
	sensor_write_reg(m_sensor_fd,0x85  ,0x04); //DD_TH3
	sensor_write_reg(m_sensor_fd,0x87  ,0x34); //32 b DNDD_low_range X16,  DNDD_low_range_C_weight_center


	//=========intp-ee
	sensor_write_reg(m_sensor_fd,0x88  ,0x04);
	sensor_write_reg(m_sensor_fd,0x89  ,0x01);
	sensor_write_reg(m_sensor_fd,0x8a  ,0x50);//60
	sensor_write_reg(m_sensor_fd,0x8b  ,0x50);//60
	sensor_write_reg(m_sensor_fd,0x8c  ,0x07);

	sensor_write_reg(m_sensor_fd,0x50  ,0x0c);
	sensor_write_reg(m_sensor_fd,0x5f  ,0x3c);

	sensor_write_reg(m_sensor_fd,0x8e  ,0x02);
	sensor_write_reg(m_sensor_fd,0x86  ,0x02);

	sensor_write_reg(m_sensor_fd,0x51  ,0x20);
	sensor_write_reg(m_sensor_fd,0x52  ,0x08);
	sensor_write_reg(m_sensor_fd,0x53  ,0x00);


	//========= YCP
	//contrast_center
	sensor_write_reg(m_sensor_fd,0x77  ,0x80);//contrast_center
	sensor_write_reg(m_sensor_fd,0x78  ,0x00);//fixed_Cb
	sensor_write_reg(m_sensor_fd,0x79  ,0x00);//fixed_Cr
	sensor_write_reg(m_sensor_fd,0x7a  ,0x00);//luma_offset
	sensor_write_reg(m_sensor_fd,0x7b  ,0x40);//hue_cos
	sensor_write_reg(m_sensor_fd,0x7c  ,0x00);//hue_sin

	//saturation
	sensor_write_reg(m_sensor_fd,0xa0  ,0x40);//global_saturation
#ifdef ZEM800
	sensor_write_reg(m_sensor_fd,0xa1  ,0x40);//luma_contrast
#else
	sensor_write_reg(m_sensor_fd,0xa1  ,0x60);//luma_contrast
#endif
	sensor_write_reg(m_sensor_fd,0xa2  ,0x34);//saturation_Cb
	sensor_write_reg(m_sensor_fd,0xa3  ,0x34);//saturation_Cr

	sensor_write_reg(m_sensor_fd,0xa4  ,0xc8);
	sensor_write_reg(m_sensor_fd,0xa5  ,0x02);
	sensor_write_reg(m_sensor_fd,0xa6  ,0x28);
	sensor_write_reg(m_sensor_fd,0xa7  ,0x02);

	//skin
	sensor_write_reg(m_sensor_fd,0xa8  ,0xee);
	sensor_write_reg(m_sensor_fd,0xa9  ,0x12);
	sensor_write_reg(m_sensor_fd,0xaa  ,0x01);
	sensor_write_reg(m_sensor_fd,0xab  ,0x20);
	sensor_write_reg(m_sensor_fd,0xac  ,0xf0);
	sensor_write_reg(m_sensor_fd,0xad  ,0x10);

	//========= ABS
	sensor_write_reg(m_sensor_fd,0xae  ,0x18);
	sensor_write_reg(m_sensor_fd,0xaf  ,0x74);
	sensor_write_reg(m_sensor_fd,0xb0  ,0xe0);
	sensor_write_reg(m_sensor_fd,0xb1  ,0x20);
	sensor_write_reg(m_sensor_fd,0xb2  ,0x6c);
#ifdef ZEM800
	sensor_write_reg(m_sensor_fd,0xb3  ,0xc0);
#else
	sensor_write_reg(m_sensor_fd,0xb3  ,0x40);
#endif
	sensor_write_reg(m_sensor_fd,0xb4  ,0x04);

	//========= AWB
	sensor_write_reg(m_sensor_fd,0xbb  ,0x42);
	sensor_write_reg(m_sensor_fd,0xbc  ,0x60);
	sensor_write_reg(m_sensor_fd,0xbd  ,0x50);
	sensor_write_reg(m_sensor_fd,0xbe  ,0x50);

	sensor_write_reg(m_sensor_fd,0xbf  ,0x0c);
	sensor_write_reg(m_sensor_fd,0xc0  ,0x06);
	sensor_write_reg(m_sensor_fd,0xc1  ,0x60);
	sensor_write_reg(m_sensor_fd,0xc2  ,0xf1); //f1
	sensor_write_reg(m_sensor_fd,0xc3  ,0x40);
	sensor_write_reg(m_sensor_fd,0xc4  ,0x1c);//18//20
	sensor_write_reg(m_sensor_fd,0xc5  ,0x56); //33
	sensor_write_reg(m_sensor_fd,0xc6  ,0x1d);

#ifdef ZEM800
	sensor_write_reg(m_sensor_fd,0xca  ,0x56);
	sensor_write_reg(m_sensor_fd,0xcb  ,0x52);
	sensor_write_reg(m_sensor_fd,0xcc  ,0x66);
#else
	sensor_write_reg(m_sensor_fd,0xca  ,0x70);
	sensor_write_reg(m_sensor_fd,0xcb  ,0x70);
	sensor_write_reg(m_sensor_fd,0xcc  ,0x78);
#endif

	sensor_write_reg(m_sensor_fd,0xcd  ,0x80);//R_ratio
	sensor_write_reg(m_sensor_fd,0xce  ,0x80);//G_ratio  , cold_white white
	sensor_write_reg(m_sensor_fd,0xcf  ,0x80);//B_ratio

	//=========  aecT
	sensor_write_reg(m_sensor_fd,0x20  ,0x06);//0x02
	sensor_write_reg(m_sensor_fd,0x21  ,0xc0);
	sensor_write_reg(m_sensor_fd,0x22  ,0x60);
	sensor_write_reg(m_sensor_fd,0x23  ,0x88);
	sensor_write_reg(m_sensor_fd,0x24  ,0x96);
	sensor_write_reg(m_sensor_fd,0x25  ,0x30);
	sensor_write_reg(m_sensor_fd,0x26  ,0xd0);
	sensor_write_reg(m_sensor_fd,0x27  ,0x00);

#ifdef ZEM800
	sensor_write_reg(m_sensor_fd,0x28  ,0x02);//AEC_exp_level_1bit11to8
	sensor_write_reg(m_sensor_fd,0x29  ,0x58);//AEC_exp_level_1bit7to0
	sensor_write_reg(m_sensor_fd,0x2a  ,0x02);//AEC_exp_level_2bit11to8
	sensor_write_reg(m_sensor_fd,0x2b  ,0x58);//AEC_exp_level_2bit7to0
	sensor_write_reg(m_sensor_fd,0x2c  ,0x02);//AEC_exp_level_3bit11to8   659 - 8FPS,  8ca - 6FPS  //
	sensor_write_reg(m_sensor_fd,0x2d  ,0x58);//AEC_exp_level_3bit7to0
	sensor_write_reg(m_sensor_fd,0x2e  ,0x05);//AEC_exp_level_4bit11to8   4FPS
	sensor_write_reg(m_sensor_fd,0x2f  ,0xdc);//AEC_exp_level_4bit7to0

#else
	sensor_write_reg(m_sensor_fd,0x28  ,0x01);//AEC_exp_level_1bit11to8
	sensor_write_reg(m_sensor_fd,0x29  ,0xf4);//AEC_exp_level_1bit7to0
	sensor_write_reg(m_sensor_fd,0x2a  ,0x02);//AEC_exp_level_2bit11to8
	sensor_write_reg(m_sensor_fd,0x2b  ,0xbc);//AEC_exp_level_2bit7to0
	sensor_write_reg(m_sensor_fd,0x2c  ,0x03);//AEC_exp_level_3bit11to8   659 - 8FPS,  8ca - 6FPS  //
	sensor_write_reg(m_sensor_fd,0x2d  ,0xe8);//AEC_exp_level_3bit7to0
	sensor_write_reg(m_sensor_fd,0x2e  ,0x09);//AEC_exp_level_4bit11to8   4FPS
	sensor_write_reg(m_sensor_fd,0x2f  ,0xc4);//AEC_exp_level_4bit7to0
#endif

	sensor_write_reg(m_sensor_fd,0x30  ,0x20);
	sensor_write_reg(m_sensor_fd,0x31  ,0x00);
	sensor_write_reg(m_sensor_fd,0x32  ,0x1c);
	sensor_write_reg(m_sensor_fd,0x33  ,0x90);
	sensor_write_reg(m_sensor_fd,0x34  ,0x10);

	sensor_write_reg(m_sensor_fd,0xd0  ,0x34);

	sensor_write_reg(m_sensor_fd,0xd1  ,0x50);//AEC_target_Y
	sensor_write_reg(m_sensor_fd,0xd2  ,0x61);//0xf2
	sensor_write_reg(m_sensor_fd,0xd4  ,0x96);
	sensor_write_reg(m_sensor_fd,0xd5  ,0x01);// william 0318
	sensor_write_reg(m_sensor_fd,0xd6  ,0x96);//antiflicker_step
	sensor_write_reg(m_sensor_fd,0xd7  ,0x03);//AEC_exp_time_min ,william 20090312
	sensor_write_reg(m_sensor_fd,0xd8  ,0x02);

	sensor_write_reg(m_sensor_fd,0xdd  ,0x22);//0x12

	//========= measure window
	sensor_write_reg(m_sensor_fd,0xe0  ,0x03);
	sensor_write_reg(m_sensor_fd,0xe1  ,0x02);
	sensor_write_reg(m_sensor_fd,0xe2  ,0x27);
	sensor_write_reg(m_sensor_fd,0xe3  ,0x1e);
	sensor_write_reg(m_sensor_fd,0xe8  ,0x3b);
	sensor_write_reg(m_sensor_fd,0xe9  ,0x6e);
	sensor_write_reg(m_sensor_fd,0xea  ,0x2c);
	sensor_write_reg(m_sensor_fd,0xeb  ,0x50);
	sensor_write_reg(m_sensor_fd,0xec  ,0x73);

	//========= close_frame
	sensor_write_reg(m_sensor_fd,0xed  ,0x00);//close_frame_num1 ,can be use to reduce FPS
	sensor_write_reg(m_sensor_fd,0xee  ,0x00);//close_frame_num2
	sensor_write_reg(m_sensor_fd,0xef  ,0x00);//close_frame_num

	// page1
	sensor_write_reg(m_sensor_fd,0xf0  ,0x01);//select page1

	sensor_write_reg(m_sensor_fd,0x00  ,0x20);
	sensor_write_reg(m_sensor_fd,0x01  ,0x20);
	sensor_write_reg(m_sensor_fd,0x02  ,0x20);
	sensor_write_reg(m_sensor_fd,0x03  ,0x20);
	sensor_write_reg(m_sensor_fd,0x04  ,0x78);
	sensor_write_reg(m_sensor_fd,0x05  ,0x78);
	sensor_write_reg(m_sensor_fd,0x06  ,0x78);
	sensor_write_reg(m_sensor_fd,0x07  ,0x78);



	sensor_write_reg(m_sensor_fd,0x10  ,0x04);
	sensor_write_reg(m_sensor_fd,0x11  ,0x04);
	sensor_write_reg(m_sensor_fd,0x12  ,0x04);
	sensor_write_reg(m_sensor_fd,0x13  ,0x04);
	sensor_write_reg(m_sensor_fd,0x14  ,0x01);
	sensor_write_reg(m_sensor_fd,0x15  ,0x01);
	sensor_write_reg(m_sensor_fd,0x16  ,0x01);
	sensor_write_reg(m_sensor_fd,0x17  ,0x01);


	sensor_write_reg(m_sensor_fd,0x20  ,0x00);
	sensor_write_reg(m_sensor_fd,0x21  ,0x00);
	sensor_write_reg(m_sensor_fd,0x22  ,0x00);
	sensor_write_reg(m_sensor_fd,0x23  ,0x00);
	sensor_write_reg(m_sensor_fd,0x24  ,0x00);
	sensor_write_reg(m_sensor_fd,0x25  ,0x00);
	sensor_write_reg(m_sensor_fd,0x26  ,0x00);
	sensor_write_reg(m_sensor_fd,0x27  ,0x00);

	sensor_write_reg(m_sensor_fd,0x40  ,0x11);

	//=============================lscP
	sensor_write_reg(m_sensor_fd,0x45  ,0x06);
	sensor_write_reg(m_sensor_fd,0x46  ,0x06);
	sensor_write_reg(m_sensor_fd,0x47  ,0x05);

	sensor_write_reg(m_sensor_fd,0x48  ,0x04);
	sensor_write_reg(m_sensor_fd,0x49  ,0x03);
	sensor_write_reg(m_sensor_fd,0x4a  ,0x03);


	sensor_write_reg(m_sensor_fd,0x62  ,0xd8);
	sensor_write_reg(m_sensor_fd,0x63  ,0x24);
	sensor_write_reg(m_sensor_fd,0x64  ,0x24);
	sensor_write_reg(m_sensor_fd,0x65  ,0x24);
	sensor_write_reg(m_sensor_fd,0x66  ,0xd8);
	sensor_write_reg(m_sensor_fd,0x67  ,0x24);

	sensor_write_reg(m_sensor_fd,0x5a  ,0x00);
	sensor_write_reg(m_sensor_fd,0x5b  ,0x00);
	sensor_write_reg(m_sensor_fd,0x5c  ,0x00);
	sensor_write_reg(m_sensor_fd,0x5d  ,0x00);
	sensor_write_reg(m_sensor_fd,0x5e  ,0x00);
	sensor_write_reg(m_sensor_fd,0x5f  ,0x00);


	//============================= ccP

	sensor_write_reg(m_sensor_fd,0x69  ,0x03);//cc_mode

	//CC_G
	sensor_write_reg(m_sensor_fd,0x70  ,0x5d);
	sensor_write_reg(m_sensor_fd,0x71  ,0xed);
	sensor_write_reg(m_sensor_fd,0x72  ,0xff);
	sensor_write_reg(m_sensor_fd,0x73  ,0xe5);
	sensor_write_reg(m_sensor_fd,0x74  ,0x5f);
	sensor_write_reg(m_sensor_fd,0x75  ,0xe6);

	sensor_write_reg(m_sensor_fd,0x76  ,0x41);
	sensor_write_reg(m_sensor_fd,0x77  ,0xef);
	sensor_write_reg(m_sensor_fd,0x78  ,0xff);
	sensor_write_reg(m_sensor_fd,0x79  ,0xff);
	sensor_write_reg(m_sensor_fd,0x7a  ,0x5f);
	sensor_write_reg(m_sensor_fd,0x7b  ,0xfa);


	//============================= AGP

	sensor_write_reg(m_sensor_fd,0x7e  ,0x00);
	sensor_write_reg(m_sensor_fd,0x7f  ,0x00);
	sensor_write_reg(m_sensor_fd,0x80  ,0xc8);
	sensor_write_reg(m_sensor_fd,0x81  ,0x06);
	sensor_write_reg(m_sensor_fd,0x82  ,0x08);

	sensor_write_reg(m_sensor_fd,0x83  ,0x23);
	sensor_write_reg(m_sensor_fd,0x84  ,0x38);
	sensor_write_reg(m_sensor_fd,0x85  ,0x4F);
	sensor_write_reg(m_sensor_fd,0x86  ,0x61);
	sensor_write_reg(m_sensor_fd,0x87  ,0x72);
	sensor_write_reg(m_sensor_fd,0x88  ,0x80);
	sensor_write_reg(m_sensor_fd,0x89  ,0x8D);
	sensor_write_reg(m_sensor_fd,0x8a  ,0xA2);
	sensor_write_reg(m_sensor_fd,0x8b  ,0xB2);
	sensor_write_reg(m_sensor_fd,0x8c  ,0xC0);
	sensor_write_reg(m_sensor_fd,0x8d  ,0xCA);
	sensor_write_reg(m_sensor_fd,0x8e  ,0xD3);
	sensor_write_reg(m_sensor_fd,0x8f  ,0xDB);
	sensor_write_reg(m_sensor_fd,0x90  ,0xE2);
	sensor_write_reg(m_sensor_fd,0x91  ,0xED);
	sensor_write_reg(m_sensor_fd,0x92  ,0xF6);
	sensor_write_reg(m_sensor_fd,0x93  ,0xFD);

	//about gamma1 is hex r oct
	sensor_write_reg(m_sensor_fd,0x94  ,0x04);
	sensor_write_reg(m_sensor_fd,0x95  ,0x0E);
	sensor_write_reg(m_sensor_fd,0x96  ,0x1B);
	sensor_write_reg(m_sensor_fd,0x97  ,0x28);
	sensor_write_reg(m_sensor_fd,0x98  ,0x35);
	sensor_write_reg(m_sensor_fd,0x99  ,0x41);
	sensor_write_reg(m_sensor_fd,0x9a  ,0x4E);
	sensor_write_reg(m_sensor_fd,0x9b  ,0x67);
	sensor_write_reg(m_sensor_fd,0x9c  ,0x7E);
	sensor_write_reg(m_sensor_fd,0x9d  ,0x94);
	sensor_write_reg(m_sensor_fd,0x9e  ,0xA7);
	sensor_write_reg(m_sensor_fd,0x9f  ,0xBA);
	sensor_write_reg(m_sensor_fd,0xa0  ,0xC8);
	sensor_write_reg(m_sensor_fd,0xa1  ,0xD4);
	sensor_write_reg(m_sensor_fd,0xa2  ,0xE7);
	sensor_write_reg(m_sensor_fd,0xa3  ,0xF4);
	sensor_write_reg(m_sensor_fd,0xa4  ,0xFA);

	//========= open functions
	sensor_write_reg(m_sensor_fd,0xf0  ,0x00);//set back to page0
#ifdef ZEM800
	sensor_write_reg(m_sensor_fd,0x40  ,0x18);//1c
	sensor_write_reg(m_sensor_fd,0x41  ,0x24);//0x00);

	sensor_write_reg(m_sensor_fd,0x0F  ,0x22);//图像需要垂直翻转 zsliu
	sensor_write_reg(m_sensor_fd,0x45  ,0x26);
	sensor_write_reg(m_sensor_fd,0x47  ,0x28);

	sensor_write_reg(m_sensor_fd,0x43  ,0x40);	//zem800
	sensor_write_reg(m_sensor_fd,0x44  ,0xe0);

#else
	sensor_write_reg(m_sensor_fd,0x40  ,0x1c);//1c
	sensor_write_reg(m_sensor_fd,0x41  ,0x00);//0x00);

	//sensor_write_reg(m_sensor_fd,0x0F  ,0x02);//0xb2);//CISCTL mode1
	sensor_write_reg(m_sensor_fd,0x0F  ,0x20);//图像需要垂直翻转 zsliu
	sensor_write_reg(m_sensor_fd,0x45  ,0x27);
	//sensor_write_reg(m_sensor_fd,0x47  ,0x2c | (1 << 7) | (1 << 0));
	//sensor_write_reg(m_sensor_fd,0x47  ,0x2c | ( 1 << 7));
	//sensor_write_reg(m_sensor_fd,0x47  ,0x2c | ( 1 << 0));
	sensor_write_reg(m_sensor_fd,0x47  ,0x2c);

	//=========open output
//	sensor_write_reg(m_sensor_fd,0x43  ,0x40);
	//sensor_write_reg(m_sensor_fd,0x44  ,0xd1);
#define GC0307_OUT_CbYCrY	0x00
#define GC0307_OUT_CrYCbY	0x01
#define GC0307_OUT_YCbYCr	0x02
#define GC0307_OUT_YCrYCb	0x03
#define GC0307_OUT_RGB565	0x06
#define GC0307_OUT_RGBx555	0x07
#define GC0307_OUT_RGB555x	0x08
#define GC0307_OUT_RGBx444	0x09
#define GC0307_OUT_RGB444x	0x0a
#define GC0307_OUT_BGRG		0x0b
#define GC0307_OUT_RGBG		0x0c
#define GC0307_OUT_GBGR		0x0d
#define GC0307_OUT_GRGB		0x0e
#define GC0307_OUT_ONLY_Y	0x11
#define GC0307_OUT_ONLY_Cb	0x12
#define GC0307_OUT_ONLY_Cr	0x13
#define GC0307_OUT_ONLY_R	0x14
#define GC0307_OUT_ONLY_G	0x15
#define GC0307_OUT_ONLY_B	0x16

#define GC0307_OUT_FMT		GC0307_OUT_ONLY_Y
	sensor_write_reg(m_sensor_fd,0x44  ,0xa0 | GC0307_OUT_FMT);
#endif
}

//****************************************************************
//对外接口部分
void sensor_init_GC307(int left, int top, int width, int height)
{
	gc0307_dbg("%s() int left=%d, int top=%d, int width=%d, int height=%d\n", __FUNCTION__, left, top, width, height);
	_initgc307();

	select_page_gc307(0);

	set_window_GC307(left, top, width, height);

#ifdef ZEM800
	set_sensor_clock_gc307(2);
#else
	set_sensor_clock_gc307(4);//解决0307指纹头图像有偏移问题，经过测试确认，硬件和库文件都存在问题，
#endif

	//set_EXPOS_GC307(0x90);

	//打开指纹头的电源
	sensor_power_on_GC307();

	//设置曝光度
	set_sensitivity_GC307();

	gc0307_dbg("%s() GC307 init finished!!!\n", __FUNCTION__);
}

void set_gc0307_i2c_client(struct i2c_client *client) {
	m_sensor_fd = client;
}
