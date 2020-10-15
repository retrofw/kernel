/*
 * Hv7131 CMOS camera sensor ops
 */

#include <linux/i2c.h>
#include "../../jz_sensor.h"
#include "../../jz_cim_core.h"
#include "hv7131.h"

// IOCTL commands
#define IOCTL_READ_REG   	0
#define IOCTL_WRITE_REG  	1

#define IOCTL_READ_EEPROM 	2
#define IOCTL_WRITE_EEPROM 	3
#define IOCTL_SET_ADDR            4 /* set i2c address */
#define IOCTL_SET_CLK             5 /* set i2c clock */

#define OPT_FLAG_GC303	0x20

typedef struct
{
	unsigned char reg;
	unsigned char val;
} S_CFG;

static struct i2c_client *m_sensor_fd = NULL;

static int m_sensor_opt = 0;
static int m_sensitivity = 0;
static int CMOSGain=0x30;

/* master clock and video clock */
static unsigned int mclk_hz = 25000000;    /* 25 MHz */
static unsigned int vclk_div = 2;          /* VCLK = MCLK/vclk_div: 2,4,8,16,32 */

static unsigned char read_reg(unsigned char reg)
{

	return (unsigned char)sensor_read_reg_nostop(m_sensor_fd, reg);
}

static int write_reg(unsigned char reg, unsigned char val)
{
	return sensor_write_reg(m_sensor_fd, reg, val);
}

/* VCLK = MCLK/div */
static void set_sensor_clock(int div)
{
#define ABLC_EN (1 << 3)
	/* ABLC enable */
	switch (div) {
	case 2:
		write_reg(SCTRA, ABLC_EN | 0x01);       // DCF=MCLK
		break;
	case 4:
		write_reg(SCTRA, ABLC_EN | 0x11);       // DCF=MCLK/2
		break;
	case 8:
		write_reg(SCTRA, ABLC_EN | 0x21);       // DCF=MCLK/4
		break;
	case 16:
		write_reg(SCTRA, ABLC_EN | 0x31);       // DCF=MCLK/8
		break;
	case 32:
		write_reg(SCTRA, ABLC_EN | 0x41);       // DCF=MCLK/16
		break;
	default:
		break;
	}
}

//303/305 support

#define GC_EXP_H	0x83
#define GC_EXP_L	0x84

#define GC_ROW_H	0x92
#define GC_ROW_L	0x93
#define GC_COL_H	0x94
#define GC_COL_L	0x95
#define GC_WINH_H	0x96
#define GC_WINH_L	0x97
#define GC_WINW_H	0x98
#define GC_WINW_L	0x99

static void set_window_GC303(int l, int t, int w, int h)
{
	l=(640 - w - l)/2*2;
	t=(480 - h - t)/2*2;
	/* Set the row start address */
	write_reg(GC_ROW_H, (t >> 8) & 0xff);
	write_reg(GC_ROW_L, t & 0xff);

	/* Set the column start address */
	write_reg(GC_COL_H, (l >> 8) & 0xff);
	write_reg(GC_COL_L, l & 0xff);

	/* Set the image window width*/
	write_reg(GC_WINW_H, (w >> 8) & 0xff);
	write_reg(GC_WINW_L, w & 0xff);

	/* Set the image window height*/
	write_reg(GC_WINH_H, (h >> 8) & 0xff);
	write_reg(GC_WINH_L, h & 0xff);
}

void set_BLANK_GC303(int d3,int d4)
{
	unsigned char val;
	write_reg(0x85, d3);
	write_reg(0x9a, d4);
	val = read_reg(0x9a);
	printk("0x9a:%x\n",val);
}

void set_BRGB_GC303(int d1,int d2,int d3,int d4)
{
        write_reg(0x86, d1);
        write_reg(0x87, d2);
        write_reg(0x88, d3);
        write_reg(0x89, d4);
}

void set_PRGB_GC303(int d1,int d2,int d3,int d4,int d5)
{
	unsigned char val;

        write_reg(0x8a, d1/256);
        write_reg(0x8b, d1%256);
        write_reg(0x8c, d2/256);
        write_reg(0x8d, d2%256);
        write_reg(0x8e, d3/256);
        write_reg(0x8f, d3%256);
        write_reg(0x90, d4/256);
        write_reg(0x91, d4%256);

        val = read_reg(0x9c);
        printk("0x9c:%x\n",val);

}

void set_EXPOS_GC303(int exposH)
{
       	write_reg(GC_EXP_H, exposH);
	write_reg(GC_EXP_L, 0x90);
}

void sensor_power_down_GC303(void)
{
	write_reg(0x9b, 0x24);
}

void sensor_power_on_GC303(void)
{
	write_reg(0x9b, 0x20);
}

static void set_sensitivity_GC303(void)
{
	int exposH=0;
	//expose
	if(m_sensor_opt & 0x01)
	{
		switch(m_sensitivity)
		{
		case 0://low
			exposH = 0x02;
			break;
		case 2://high
			exposH = 0x12;
			break;
		case 1://middle
		default:
			exposH = 0x06;
			break;
		}
		set_EXPOS_GC303(exposH);
	}else
	{
		set_EXPOS_GC303(0x02);
	}
}

static void sensor_init_GC303(int left, int top, int width, int height)
{
	unsigned char val;

        /* set clock */
        set_sensor_clock(vclk_div);

        val = read_reg(0x80);
	printk("Sensor version:%x\n",val);
	set_BLANK_GC303(0x00,00);

	set_BRGB_GC303(0,0,0,0);
	set_PRGB_GC303(0x180,0x180,0x180,0x130,0x25);

        write_reg(0x9c, 0x17);
        write_reg(0x9d, 0x80);	//0xa0
        write_reg(0x9e, 0x08);
        write_reg(0x9f, 0x80 | (1 << 3));	//mirror image
        write_reg(0xa0, 0x00);
	val = read_reg(0xa1);
        printk("AD Bias current control:%x\n",val);
        val = read_reg(0x80);
        printk("Sensor version1:%x\n",val);

	set_window_GC303(left, top, width, height);

	if (val==0x11)
        {
		printk("GC0303 CMOS\n");
	}
	else if (val==0x29)
	{
		printk("GC0305 CMOS\n");
	}
	sensor_power_on_GC303();
	printk("GC303 init finished\n");
}

void sensor_init(int left, int top, int width, int height)
{
	int ret = 0;
	set_sensor_clock(vclk_div);
	sensor_init_GC303(left, top ,width, height);
}

int SetCMOSGain(int Gain)
{
	return write_reg(PAG, Gain);//Pre-amp Gain
}

int IncCMOSGain(void)
{
        if(CMOSGain>4)
                CMOSGain=(CMOSGain*5+2)/4;
        else
                CMOSGain=10;
        if(CMOSGain>255) CMOSGain=255;
        return SetCMOSGain(CMOSGain);
}

int DecCMOSGain(void)
{
        CMOSGain=CMOSGain*2/3;
        if(CMOSGain==0) CMOSGain=1;
        return SetCMOSGain(CMOSGain);
}

void set_gc0303_i2c_client(struct i2c_client *client) {
	m_sensor_fd = client;
}
