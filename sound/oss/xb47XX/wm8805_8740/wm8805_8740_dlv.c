/*
 * Linux/sound/oss/jz_dlv.c
 *
 * DLV CODEC driver for Ingenic Jz4750 MIPS processor
 *
 * 2009-12-xx	Steven <dsqiu@ingenic.cn>
 * 2010-01-xx	Jason <xwang@ingenic.cn>
 * 2010-11-xx   jbbi <jbbi@ingenic.cn>
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/sound.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <linux/proc_fs.h>
#include <linux/soundcard.h>
#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>

#include <linux/i2c.h>

#include <asm/hardirq.h>
#include <asm/jzsoc.h>
//#include <mach/chip-aic.h>
//#include <linux/jz_audio.h>

#include "../jz47XX_codec.h"


#define WM8805_RST	32+16	//32+16 DRA


#define DUMP_FUNC() printk("WM8805_8740_DLV:%s\tline:%d\n", __func__, __LINE__)

#define DEFAULT_SAMPLE_RATE	48000


#define WM8805_CHIPADDR			0x74
#define WM8805_RST_DEVID1			0x00
#define WM8805_DEVID2				0x01
#define WM8805_DEVREV				0x02
#define WM8805_PLL1				0x03
#define WM8805_PLL2				0x04
#define WM8805_PLL3				0x05
#define WM8805_PLL4				0x06
#define WM8805_PLL5				0x07
#define WM8805_PLL6				0x08
#define WM8805_SPDMODE				0x09
#define WM8805_INTMASK				0x0A
#define WM8805_INTSTAT				0x0B
#define WM8805_SPDSTAT				0x0C
#define WM8805_RXCHAN1				0x0D
#define WM8805_RXCHAN2				0x0E
#define WM8805_RXCHAN3				0x0F
#define WM8805_RXCHAN4				0x10
#define WM8805_RXCHAN5				0x11
#define WM8805_SPDTX1				0x12
#define WM8805_SPDTX2				0x13
#define WM8805_SPDTX3				0x14
#define WM8805_SPDTX4				0x15
#define WM8805_SPDTX5				0x16
#define WM8805_GPO0				0x17
#define WM8805_GPO1				0x18
#define WM8805_GPO2				0x1A
#define WM8805_AIFTX				0x1B
#define WM8805_AIFRX				0x1C
#define WM8805_SPDRX1				0x1D
#define WM8805_PWRDN				0x1E

#define WM8805_REGISTER_COUNT			30
#define WM8805_MAX_REGISTER			0x1E

#define WM8805_TX_CLKSRC_MCLK			1
#define WM8805_TX_CLKSRC_PLL			2

#define WM8805_CLKOUT_SRC_CLK1			3
#define WM8805_CLKOUT_SRC_OSCCLK		4

#define WM8805_CLKOUT_DIV			1

#define WM8805_NUM_SUPPLIES 2

#define	WM8805_SRC_CLK	12000000
#define	WM8805_N_FS	256
static u8 wm8805_reg_defs[] = {
	0x05,     /* R0  - RST/DEVID1 */
	0x88,     /* R1  - DEVID2 */
	0x04,     /* R2  - DEVREV */
	0x21,     /* R3  - PLL1 */
	0xFD,     /* R4  - PLL2 */
	0x36,     /* R5  - PLL3 */
	0x07,     /* R6  - PLL4 */
	0x16,     /* R7  - PLL5 */
	0x20, //0x19,//    /* R8  - PLL6 */
	0xFF,     /* R9  - SPDMODE */
	0x00,     /* R10 - INTMASK */
	0x2D,//0x08,//0x00,     /* R11 - INTSTAT */
	0x04,//0x00,     /* R12 - SPDSTAT */
	0x04,//0x00,     /* R13 - RXCHAN1 */
	0x00,     /* R14 - RXCHAN2 */
	0x00,     /* R15 - RXCHAN3 */
	0x00,     /* R16 - RXCHAN4 */
	0x00,     /* R17 - RXCHAN5 */
	0x00,     /* R18 - SPDTX1 */
	0x00,     /* R19 - SPDTX2 */
	0x00,     /* R20 - SPDTX3 */
	0x71,     /* R21 - SPDTX4 */
	0x0B,     /* R22 - SPDTX5 */
	0x70,//0xD0,//0x70,     /* R23 - GPO0 */
	0x57,     /* R24 - GPO1 */
	0x00,     /* R25 */
	0x70,//0xD2,     /* R26 - GPO2 */
	0x06,     /* R27 - AIFTX */
	0x4a,     /* R28 - AIFRX */
	0xa0,     /* R29 - SPDRX1 */
	0x03,//0x07     /* R30 - PWRDN */
};


#define WM8741_CHIPADDR_L	0x36
#define WM8741_CHIPADDR_R	0x34
/*
 * Register values.
 */
#define WM8741_DACLLSB_ATTENUATION              0x00
#define WM8741_DACLMSB_ATTENUATION              0x01
#define WM8741_DACRLSB_ATTENUATION              0x02
#define WM8741_DACRMSB_ATTENUATION              0x03
#define WM8741_VOLUME_CONTROL                   0x04
#define WM8741_FORMAT_CONTROL                   0x05
#define WM8741_FILTER_CONTROL                   0x06
#define WM8741_MODE_CONTROL_1                   0x07
#define WM8741_MODE_CONTROL_2                   0x08
#define WM8741_RESET                            0x09
#define WM8741_ADDITIONAL_CONTROL_1             0x20

#define WM8741_REGISTER_COUNT                   11
#define WM8741_MAX_REGISTER                     0x20

/*
 * Field Definitions.
 */

/*
 * R0 (0x00) - DACLLSB_ATTENUATION
 */
#define WM8741_UPDATELL                         0x0020  /* UPDATELL */
#define WM8741_UPDATELL_MASK                    0x0020  /* UPDATELL */
#define WM8741_UPDATELL_SHIFT                        5  /* UPDATELL */
#define WM8741_UPDATELL_WIDTH                        1  /* UPDATELL */
#define WM8741_LAT_4_0_MASK                     0x001F  /* LAT[4:0] - [4:0] */
#define WM8741_LAT_4_0_SHIFT                         0  /* LAT[4:0] - [4:0] */
#define WM8741_LAT_4_0_WIDTH                         5  /* LAT[4:0] - [4:0] */

/*
 * R1 (0x01) - DACLMSB_ATTENUATION
 */
#define WM8741_UPDATELM                         0x0020  /* UPDATELM */
#define WM8741_UPDATELM_MASK                    0x0020  /* UPDATELM */
#define WM8741_UPDATELM_SHIFT                        5  /* UPDATELM */
#define WM8741_UPDATELM_WIDTH                        1  /* UPDATELM */
#define WM8741_LAT_9_5_0_MASK                   0x001F  /* LAT[9:5] - [4:0] */
#define WM8741_LAT_9_5_0_SHIFT                       0  /* LAT[9:5] - [4:0] */
#define WM8741_LAT_9_5_0_WIDTH                       5  /* LAT[9:5] - [4:0] */

/*
 * R2 (0x02) - DACRLSB_ATTENUATION
 */
#define WM8741_UPDATERL                         0x0020  /* UPDATERL */
#define WM8741_UPDATERL_MASK                    0x0020  /* UPDATERL */
#define WM8741_UPDATERL_SHIFT                        5  /* UPDATERL */
#define WM8741_UPDATERL_WIDTH                        1  /* UPDATERL */
#define WM8741_RAT_4_0_MASK                     0x001F  /* RAT[4:0] - [4:0] */
#define WM8741_RAT_4_0_SHIFT                         0  /* RAT[4:0] - [4:0] */
#define WM8741_RAT_4_0_WIDTH                         5  /* RAT[4:0] - [4:0] */

/*
 * R3 (0x03) - DACRMSB_ATTENUATION
 */
#define WM8741_UPDATERM                         0x0020  /* UPDATERM */
#define WM8741_UPDATERM_MASK                    0x0020  /* UPDATERM */
#define WM8741_UPDATERM_SHIFT                        5  /* UPDATERM */
#define WM8741_UPDATERM_WIDTH                        1  /* UPDATERM */
#define WM8741_RAT_9_5_0_MASK                   0x001F  /* RAT[9:5] - [4:0] */
#define WM8741_RAT_9_5_0_SHIFT                       0  /* RAT[9:5] - [4:0] */
#define WM8741_RAT_9_5_0_WIDTH                       5  /* RAT[9:5] - [4:0] */

/*
 * R4 (0x04) - VOLUME_CONTROL
 */
#define WM8741_AMUTE                            0x0080  /* AMUTE */
#define WM8741_AMUTE_MASK                       0x0080  /* AMUTE */
#define WM8741_AMUTE_SHIFT                           7  /* AMUTE */
#define WM8741_AMUTE_WIDTH                           1  /* AMUTE */
#define WM8741_ZFLAG_MASK                       0x0060  /* ZFLAG - [6:5] */
#define WM8741_ZFLAG_SHIFT                           5  /* ZFLAG - [6:5] */
#define WM8741_ZFLAG_WIDTH                           2  /* ZFLAG - [6:5] */
#define WM8741_IZD                              0x0010  /* IZD */
#define WM8741_IZD_MASK                         0x0010  /* IZD */
#define WM8741_IZD_SHIFT                             4  /* IZD */
#define WM8741_IZD_WIDTH                             1  /* IZD */
#define WM8741_SOFT                             0x0008  /* SOFT MUTE */
#define WM8741_SOFT_MASK                        0x0008  /* SOFT MUTE */
#define WM8741_SOFT_SHIFT                            3  /* SOFT MUTE */
#define WM8741_SOFT_WIDTH                            1  /* SOFT MUTE */
#define WM8741_ATC                              0x0004  /* ATC */
#define WM8741_ATC_MASK                         0x0004  /* ATC */
#define WM8741_ATC_SHIFT                             2  /* ATC */
#define WM8741_ATC_WIDTH                             1  /* ATC */
#define WM8741_ATT2DB                           0x0002  /* ATT2DB */
#define WM8741_ATT2DB_MASK                      0x0002  /* ATT2DB */
#define WM8741_ATT2DB_SHIFT                          1  /* ATT2DB */
#define WM8741_ATT2DB_WIDTH                          1  /* ATT2DB */
#define WM8741_VOL_RAMP                         0x0001  /* VOL_RAMP */
#define WM8741_VOL_RAMP_MASK                    0x0001  /* VOL_RAMP */
#define WM8741_VOL_RAMP_SHIFT                        0  /* VOL_RAMP */
#define WM8741_VOL_RAMP_WIDTH                        1  /* VOL_RAMP */

/*
 * R5 (0x05) - FORMAT_CONTROL
 */
#define WM8741_PWDN                             0x0080  /* PWDN */
#define WM8741_PWDN_MASK                        0x0080  /* PWDN */
#define WM8741_PWDN_SHIFT                            7  /* PWDN */
#define WM8741_PWDN_WIDTH                            1  /* PWDN */
#define WM8741_REV                              0x0040  /* REV */
#define WM8741_REV_MASK                         0x0040  /* REV */
#define WM8741_REV_SHIFT                             6  /* REV */
#define WM8741_REV_WIDTH                             1  /* REV */
#define WM8741_BCP                              0x0020  /* BCP */
#define WM8741_BCP_MASK                         0x0020  /* BCP */
#define WM8741_BCP_SHIFT                             5  /* BCP */
#define WM8741_BCP_WIDTH                             1  /* BCP */
#define WM8741_LRP                              0x0010  /* LRP */
#define WM8741_LRP_MASK                         0x0010  /* LRP */
#define WM8741_LRP_SHIFT                             4  /* LRP */
#define WM8741_LRP_WIDTH                             1  /* LRP */
#define WM8741_FMT_MASK                         0x000C  /* FMT - [3:2] */
#define WM8741_FMT_SHIFT                             2  /* FMT - [3:2] */
#define WM8741_FMT_WIDTH                             2  /* FMT - [3:2] */
#define WM8741_IWL_MASK                         0x0003  /* IWL - [1:0] */
#define WM8741_IWL_SHIFT                             0  /* IWL - [1:0] */
#define WM8741_IWL_WIDTH                             2  /* IWL - [1:0] */

/*
 * R6 (0x06) - FILTER_CONTROL
 */
#define WM8741_ZFLAG_HI                         0x0080  /* ZFLAG_HI */
#define WM8741_ZFLAG_HI_MASK                    0x0080  /* ZFLAG_HI */
#define WM8741_ZFLAG_HI_SHIFT                        7  /* ZFLAG_HI */
#define WM8741_ZFLAG_HI_WIDTH                        1  /* ZFLAG_HI */
#define WM8741_DEEMPH_MASK                      0x0060  /* DEEMPH - [6:5] */
#define WM8741_DEEMPH_SHIFT                          5  /* DEEMPH - [6:5] */
#define WM8741_DEEMPH_WIDTH                          2  /* DEEMPH - [6:5] */
#define WM8741_DSDFILT_MASK                     0x0018  /* DSDFILT - [4:3] */
#define WM8741_DSDFILT_SHIFT                         3  /* DSDFILT - [4:3] */
#define WM8741_DSDFILT_WIDTH                         2  /* DSDFILT - [4:3] */
#define WM8741_FIRSEL_MASK                      0x0007  /* FIRSEL - [2:0] */
#define WM8741_FIRSEL_SHIFT                          0  /* FIRSEL - [2:0] */
#define WM8741_FIRSEL_WIDTH                          3  /* FIRSEL - [2:0] */

/*
 * R7 (0x07) - MODE_CONTROL_1
 */
#define WM8741_MODE8X                           0x0080  /* MODE8X */
#define WM8741_MODE8X_MASK                      0x0080  /* MODE8X */
#define WM8741_MODE8X_SHIFT                          7  /* MODE8X */
#define WM8741_MODE8X_WIDTH                          1  /* MODE8X */
#define WM8741_OSR_MASK                         0x0060  /* OSR - [6:5] */
#define WM8741_OSR_SHIFT                             5  /* OSR - [6:5] */
#define WM8741_OSR_WIDTH                             2  /* OSR - [6:5] */
#define WM8741_SR_MASK                          0x001C  /* SR - [4:2] */
#define WM8741_SR_SHIFT                              2  /* SR - [4:2] */
#define WM8741_SR_WIDTH                              3  /* SR - [4:2] */
#define WM8741_MODESEL_MASK                     0x0003  /* MODESEL - [1:0] */
#define WM8741_MODESEL_SHIFT                         0  /* MODESEL - [1:0] */
#define WM8741_MODESEL_WIDTH                         2  /* MODESEL - [1:0] */

/*
 * R8 (0x08) - MODE_CONTROL_2
 */
#define WM8741_DSD_GAIN                         0x0040  /* DSD_GAIN */
#define WM8741_DSD_GAIN_MASK                    0x0040  /* DSD_GAIN */
#define WM8741_DSD_GAIN_SHIFT                        6  /* DSD_GAIN */
#define WM8741_DSD_GAIN_WIDTH                        1  /* DSD_GAIN */
#define WM8741_SDOUT                            0x0020  /* SDOUT */
#define WM8741_SDOUT_MASK                       0x0020  /* SDOUT */
#define WM8741_SDOUT_SHIFT                           5  /* SDOUT */
#define WM8741_SDOUT_WIDTH                           1  /* SDOUT */
#define WM8741_DOUT                             0x0010  /* DOUT */
#define WM8741_DOUT_MASK                        0x0010  /* DOUT */
#define WM8741_DOUT_SHIFT                            4  /* DOUT */
#define WM8741_DOUT_WIDTH                            1  /* DOUT */
#define WM8741_DIFF_MASK                        0x000C  /* DIFF - [3:2] */
#define WM8741_DIFF_SHIFT                            2  /* DIFF - [3:2] */
#define WM8741_DIFF_WIDTH                            2  /* DIFF - [3:2] */
#define WM8741_DITHER_MASK                      0x0003  /* DITHER - [1:0] */
#define WM8741_DITHER_SHIFT                          0  /* DITHER - [1:0] */
#define WM8741_DITHER_WIDTH                          2  /* DITHER - [1:0] */

/*
 * R32 (0x20) - ADDITONAL_CONTROL_1
 */
#define WM8741_DSD_LEVEL                        0x0002  /* DSD_LEVEL */
#define WM8741_DSD_LEVEL_MASK                   0x0002  /* DSD_LEVEL */
#define WM8741_DSD_LEVEL_SHIFT                       1  /* DSD_LEVEL */
#define WM8741_DSD_LEVEL_WIDTH                       1  /* DSD_LEVEL */
#define WM8741_DSD_NO_NOTCH                     0x0001  /* DSD_NO_NOTCH */
#define WM8741_DSD_NO_NOTCH_MASK                0x0001  /* DSD_NO_NOTCH */
#define WM8741_DSD_NO_NOTCH_SHIFT                    0  /* DSD_NO_NOTCH */
#define WM8741_DSD_NO_NOTCH_WIDTH                    1  /* DSD_NO_NOTCH */

#define  WM8741_SYSCLK 0
static u8 wm8741_reg_defs[WM8741_REGISTER_COUNT] = {
	#if 1
	#if 0
	0x20,     /* R0  - DACLLSB Attenuation */
	0x20,     /* R1  - DACLMSB Attenuation */
	0x20,     /* R2  - DACRLSB Attenuation */
	0x20,     /* R3  - DACRMSB Attenuation */
	0x00,     /* R4  - Volume Control */
	0x0a,     /* R5  - Format Control */
	0x00,     /* R6  - Filter Control */
	0x44,     /* R7  - Mode Control 1 */
	0x02,     /* R8  - Mode Control 2 */
	0x00,	    /* R9  - Reset */
	0x02,     /* R32 - ADDITONAL_CONTROL_1 */
	#endif
	#if 1
	0x3F,     /* R0  - DACLLSB Attenuation */
	0x3F,     /* R1  - DACLMSB Attenuation */
	0x3F,     /* R2  - DACRLSB Attenuation */
	0x3F,     /* R3  - DACRMSB Attenuation */
	/*0xc0*/0x61,     /* R4  - Volume Control */
	0x0a,     /* R5  - Format Control */
	0x02,//0x00,     /* R6  - Filter Control */
	0x40,//0x00,     /* R7  - Mode Control 1 */
	0x06,    //0x0e /* R8  - Mode Control 2 */
	0x00,	    /* R9  - Reset */
	0x02,     /* R32 - ADDITONAL_CONTROL_1 */
	#endif
	#else
	0x00,     /* R0  - DACLLSB Attenuation */
	0x00,     /* R1  - DACLMSB Attenuation */
	0x00,     /* R2  - DACRLSB Attenuation */
	0x00,     /* R3  - DACRMSB Attenuation */
	0x00,     /* R4  - Volume Control */
	0x2A,     /* R5  - Format Control */
	0x00,     /* R6  - Filter Control */
	0x44,     /* R7  - Mode Control 1 */
	0x02,     /* R8  - Mode Control 2 */
	0x00,	    /* R9  - Reset */
	0x02,     /* R32 - ADDITONAL_CONTROL_1 */
	#endif
};


struct wm8741_private {
	unsigned int mclk; /* Input frequency of the MCLK pin */
	unsigned int mode; /* The mode (I2S or left-justified) */
	unsigned int slave_mode;
	unsigned int manual_mute;
	unsigned int enable_l;
	unsigned int enable_r;
	struct i2c_client *client_l;
	struct i2c_client *client_r;
};

struct pll_div {
	u32 prescale:1;
	u32 mclkdiv:1;
	u32 freqmode:2;
	u32 n:4;
	u32 k:22;
};

/* PLL rate to output rate divisions */
static struct {
	unsigned int div;
	unsigned int freqmode;
	unsigned int mclkdiv;
} post_table[] = {
	{  2,  0, 1 },
	{  4,  1, 0 },
	{  4,  1, 0 },
	{  8,  2, 0 },
	{  8,  2, 0 },
	{ 16,  2, 1 },
	{ 12,  3, 0 },
	{ 24,  3, 1 }
};

#define FIXED_PLL_SIZE ((1ULL << 22) * 10)

#define WM8805_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE)

#define WM8805_RATES (SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
		      SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_64000 | \
		      SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 | \
		      SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000)


struct wm8805_private {
	unsigned int chip_id;
	unsigned int mclk; /* Input frequency of the MCLK pin */
	unsigned int mode; /* The mode (I2S or left-justified) */
	unsigned int slave_mode;
	unsigned int manual_mute;
	struct i2c_client *client;
};

static unsigned int g_buf_cnt = 0;

typedef struct {
	int (*dlv_init)(void *);
	int (*dlv_shutdown)(void *);
	int (*dlv_reset)(void *);
	int (*dlv_dump_regs)(void *);
	int (*dlv_set_record_volume)(void *,int);
	int (*dlv_set_replay_volume)(void *,int);
	int (*dlv_set_record_rate)(void *,int);
	int (*dlv_set_replay_rate)(void *,int);
	int (*dlv_set_record_data_width)(void *,int);
	int (*dlv_set_replay_data_width)(void *,int);
	int (*dlv_suspend)(void *);
	int (*dlv_resume)(void *);
} ex_codec_ctrl_t;


struct codec_data {
void * clk_unit;
void * dlv_unit;
ex_codec_ctrl_t func;
};

static struct codec_data *g_codec_data = NULL;

/****************** wm8805 api  ****************************************/

static int wm8805_i2c_read(struct wm8805_private *dev,unsigned char reg)
{

	u_int8_t value;
	int ret;
	
	/* Set Index 0 */
	value = 0;
	ret = i2c_master_send(dev->client, &reg, 1);
	if (ret != 1) {
	        dev_err(&dev->client->dev, "Unable to write i2c index (ret=%d)\n",ret);
	        return -1;
	}   

	/* Now do Page Read */
	ret = i2c_master_recv(dev->client, &value, 1);
	if (ret != 1) {
	        dev_err(&dev->client->dev, "Unable to read i2c page (ret=%d)\n",ret);
	        return -1;
	}

	return value;
}
static int wm8805_i2c_write(struct wm8805_private *dev,unsigned char reg,unsigned char value)
{
	int ret;
	u_int8_t buf[2] = {reg,value};

	ret = i2c_master_send(dev->client, buf, 2);
	if (ret != 2) {
	        dev_err(&dev->client->dev, "Unable to write i2c index (ret=%d)\n",ret);
	        return -1;
	}
	return 0;
}


int wm8805_i2c_update_bits(struct wm8805_private *dev, unsigned short reg,
		unsigned short mask, unsigned short value)
{
	int change;
	unsigned short old, new;

	old = wm8805_reg_defs[reg];
	new = (old & ~mask) | value;
	change = (old != new);
	
	if (change){
		wm8805_i2c_write(dev, reg, new);
		wm8805_reg_defs[reg] = new;
		//printk("@@reg=0x%x  old value=0x%x,new value=0x%x\n",reg,old,new);
	}
	
	return change;
}

static int wm8805_rx_pll(struct wm8805_private *dev,unsigned char enable)
{
	//wm8805_i2c_update_bits(dev, WM8805_PWRDN, 0x1, 0x1);
	if(enable)
		{
			wm8805_i2c_update_bits(dev, WM8805_PLL1, 0xff ,0xba);
			wm8805_i2c_update_bits(dev, WM8805_PLL2, 0xff ,0x49);
			wm8805_i2c_update_bits(dev, WM8805_PLL3, 0xff ,0x0c);
			wm8805_i2c_update_bits(dev, WM8805_PLL4, 0x0f ,0x08);
			wm8805_i2c_update_bits(dev, WM8805_SPDRX1, 0x80 ,0x80);
			wm8805_i2c_update_bits(dev, WM8805_PLL5,  0x8, 1 << 3);
		}
	else
		{
			wm8805_i2c_update_bits(dev, WM8805_PLL1, 0xff ,0x21);
			wm8805_i2c_update_bits(dev, WM8805_PLL2, 0xff ,0xfd);
			wm8805_i2c_update_bits(dev, WM8805_PLL3, 0xff ,0x36);
			wm8805_i2c_update_bits(dev, WM8805_PLL4, 0x0f ,0x07);
			wm8805_i2c_update_bits(dev, WM8805_SPDRX1, 0x80 ,0x00);
			wm8805_i2c_update_bits(dev, WM8805_PLL5,  0x8, 0 << 3);
		}
	//wm8805_i2c_update_bits(dev, WM8805_PWRDN, 0x1, 0x0);
	return 0;
}
static int wm8805_reset(struct wm8805_private *dev)
{
	int ret = 0;
	ret =  wm8805_i2c_write(dev, WM8805_RST_DEVID1, 0x0);
	if(ret < 0){
		printk("wm8805 reset fail !\n");
	}
	return ret;
}
static int wm8805_chip(struct wm8805_private *dev)
{
	int ret;
	unsigned int id1, id2, chip_id;

	ret = wm8805_i2c_read(dev, WM8805_RST_DEVID1);
	if(ret < 0) return ret;
	id1 = ret;

	ret = wm8805_i2c_read(dev, WM8805_DEVID2);
	if(ret < 0) return ret;
	id2 = ret;

	id2 = (id2 << 8) | id1;

	chip_id = (wm8805_reg_defs[WM8805_DEVID2] << 8)| wm8805_reg_defs[WM8805_RST_DEVID1];
	
	if(id2 != chip_id){
		printk("@@@@wm8805 chipID = 0x%x, correct chipID = 0x%x ",id2,chip_id);
		return -1;
	}
	dev->chip_id = chip_id;
	printk("wm8805 chip id: 0x%x ",chip_id);

	wm8805_i2c_read(dev, WM8805_DEVREV);

	printk( "(revision %c)\n", ret + 'A');

	return 0;
}

static int wm8805_set_rxch(struct wm8805_private *dev, unsigned char ch)
{
	if(ch>=8){
		printk("wm8805 only have 8 channe\n");
		return -EINVAL;
	}
	wm8805_i2c_update_bits(dev, WM8805_PLL6, 0x07 ,ch);
	return 0;
}
/*128,256fs.....*/
static int wm8805_set_clkdiv(struct wm8805_private *dev, int div_id, int div)
{

	switch (div_id) {
	case WM8805_CLKOUT_DIV:
		wm8805_i2c_update_bits(dev, WM8805_PLL5, 0x30,
				    (div & 0x3) << 4);
		break;
	default:
		printk("Unknown clock divider: %d\n", div_id);
		return -EINVAL;
	}
	return 0;
}

static int wm8805_set_sysclk(struct wm8805_private *dev, int clk_id, unsigned int freq, int dir)
{
	switch (clk_id) {
	case WM8805_TX_CLKSRC_MCLK:
		if ((freq >= 10000000 && freq <= 14400000)
				|| (freq >= 16280000 && freq <= 27000000))
			wm8805_i2c_update_bits(dev, WM8805_PLL6, 0x80, 0x80);
		else {
			printk( "OSCCLOCK is not within the "
				"recommended range: %uHz\n", freq);
			return -EINVAL;
		}
		break;
	case WM8805_TX_CLKSRC_PLL://yes
		wm8805_i2c_update_bits(dev, WM8805_PLL6, 0x80, 0);
		break;
	case WM8805_CLKOUT_SRC_CLK1:
		wm8805_i2c_update_bits(dev, WM8805_PLL6, 0x8, 0);
		break;
	case WM8805_CLKOUT_SRC_OSCCLK:
		wm8805_i2c_update_bits(dev, WM8805_PLL6, 0x8, 0x8);
		break;
	default:
		printk("Unknown clock source: %d\n", clk_id);
		return -EINVAL;
	}

	return 0;
}

static int pll_factors(struct pll_div *pll_div, unsigned int target,
		       unsigned int source)
{
	u64 Kpart;
	unsigned long int K, Ndiv, Nmod, tmp;
	int i;

	/*
	 * Scale the output frequency up; the PLL should run in the
	 * region of 90-100MHz.
	 */
	for (i = 0; i < ARRAY_SIZE(post_table); i++) {
		tmp = target * post_table[i].div;
		if (tmp >= 90000000 && tmp <= 100000000) {
			pll_div->freqmode = post_table[i].freqmode;
			pll_div->mclkdiv = post_table[i].mclkdiv;
			target *= post_table[i].div;
			break;
		}
	}

	if (i == ARRAY_SIZE(post_table)) {
		printk("%s: Unable to scale output frequency: %uHz\n",
		       __func__, target);
		return -EINVAL;
	}

	pll_div->prescale = 0;
	Ndiv = target / source;
	if (Ndiv < 5) {
		source >>= 1;
		pll_div->prescale = 1;
		Ndiv = target / source;
	}

	if (Ndiv < 5 || Ndiv > 13) {
		printk("%s: WM8805 N value is not within the recommended range: %lu\n",
		       __func__, Ndiv);
		return -EINVAL;
	}
	pll_div->n = Ndiv;

	Nmod = target % source;
	Kpart = FIXED_PLL_SIZE * (u64)Nmod;

	do_div(Kpart, source);

	K = Kpart & 0xffffffff;
	if ((K % 10) >= 5)
		K += 5;
	K /= 10;
	pll_div->k = K;

	return 0;
}

static int wm8805_set_pll(struct wm8805_private *dev, int pll_id,int source, unsigned int freq_in,unsigned int freq_out)
{

	if (!freq_in || !freq_out) {
		/* disable the PLL */
		wm8805_i2c_update_bits(dev, WM8805_PWRDN, 0x1, 0x1);
		return 0;
	} else {
		int ret;
		struct pll_div pll_div;

		ret = pll_factors(&pll_div, freq_out, freq_in);
		if (ret)
			return ret;

		/* power down the PLL before reprogramming it */
		wm8805_i2c_update_bits(dev, WM8805_PWRDN, 0x1, 0x1);

		if (!freq_in || !freq_out)
			return 0;

		wm8805_i2c_update_bits(dev, WM8805_PLL1, 0xFF, pll_div.k & 0xff);
		wm8805_i2c_update_bits(dev, WM8805_PLL2, 0xFF, (pll_div.k >> 8) & 0xff);
		wm8805_i2c_update_bits(dev, WM8805_PLL3, 0xFF, pll_div.k >> 16);
		/* set PLLN and PRESCALE */
		wm8805_i2c_update_bits(dev, WM8805_PLL4, 0xf | 0x10,
				    pll_div.n | (pll_div.prescale << 4));
		/* set mclkdiv and freqmode */
		wm8805_i2c_update_bits(dev, WM8805_PLL5, 0x3 | 0x8,
				    pll_div.freqmode | (pll_div.mclkdiv << 3));
		/* power up the PLL */
		wm8805_i2c_update_bits(dev, WM8805_PWRDN, 0x1, 0);
	}

	return 0;
}

static int wm8805_set_fmt(struct wm8805_private *dev, unsigned int fmt)
{
	u16 format, master, bcp, lrp;

	#if 0
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		format = 0x2;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		format = 0x0;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		format = 0x1;
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		format = 0x3;
		break;
	default:
		printk( "Unknown dai format\n");
		return -EINVAL;
	}
	#endif
	format = 0x2;
	/* set data format */
	wm8805_i2c_update_bits(dev, WM8805_AIFTX, 0x3, format);
	wm8805_i2c_update_bits(dev, WM8805_AIFRX, 0x3, format);

	#if 0
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		master = 1;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		master = 0;
		break;
	default:
		printk( "Unknown master/slave configuration\n");
		return -EINVAL;
	}
	#endif
	master = 1;
	/* set master/slave mode */
	wm8805_i2c_update_bits(dev, WM8805_AIFRX, 0x40, master << 6);

	bcp = lrp = 0;
	#if 0
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		bcp = lrp = 1;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		bcp = 1;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		lrp = 1;
		break;
	default:
		printk( "Unknown polarity configuration\n");
		return -EINVAL;
	}
	#endif
	/* set frame inversion */
	wm8805_i2c_update_bits(dev, WM8805_AIFTX, 0x10 | 0x20,
			    (bcp << 4) | (lrp << 5));
	wm8805_i2c_update_bits(dev, WM8805_AIFRX, 0x10 | 0x20,
			    (bcp << 4) | (lrp << 5));
	return 0;
}

static int wm8805_hw_params(struct wm8805_private *dev)
{
	unsigned int blen = 0x2;
	#if 0
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		blen = 0x0;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		blen = 0x1;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		blen = 0x2;
		break;
	default:
		dev_err(dai->dev, "Unsupported word length: %u\n",
			params_format(params));
		return -EINVAL;
	}
	#endif
	/* set word length */
	wm8805_i2c_update_bits(dev, WM8805_AIFTX, 0xc, blen << 2);
	wm8805_i2c_update_bits(dev, WM8805_AIFRX, 0xc, blen << 2);

	return 0;
}

static int txsrc_put(struct wm8805_private *dev, unsigned int src)
{
	//unsigned int txpwr;

	/* save the current power state of the transmitter */
	//txpwr = wm8805_reg_defs[WM8805_PWRDN]& 0x4;//wm8805_i2c_read(dev, WM8805_PWRDN) & 0x4;
	/* set the tx source */
	/* power down the transmitter */
	wm8805_i2c_update_bits(dev, WM8805_PWRDN, 0x4, 0x4);
	wm8805_i2c_update_bits(dev, WM8805_SPDTX4, 0x40,
			    src << 6);

	if (src) {
		/* power down the receiver */
		wm8805_i2c_update_bits(dev, WM8805_PWRDN, 0x2, 0x2);
		/* power up the AIF */
		wm8805_i2c_update_bits(dev, WM8805_PWRDN, 0x10, 0);
					/* power up the transmitter */
		wm8805_i2c_update_bits(dev, WM8805_PWRDN, 0x4, 0x0);
	} else {
		/* don't power down the AIF -- may be used as an output */
			/* power down the transmitter */
		wm8805_i2c_update_bits(dev, WM8805_PWRDN, 0x4, 0x4);
		/* power up the receiver */
		wm8805_i2c_update_bits(dev, WM8805_PWRDN, 0x2, 0);
	}

	/* restore the transmitter's configuration */
	//wm8805_i2c_update_bits(dev, WM8805_PWRDN, 0x4, txpwr);
	/* power up the receiver */
		//wm8805_i2c_update_bits(dev, WM8805_PWRDN, 0x2, 0x0);
	return 0;
}

static void wm8805_dump_all(struct wm8805_private *dev)
{
	unsigned char i;
	printk("wm8805 dump all regs:\n");
	for(i=0;i<31;i++)
		printk("reg =0x%x,value=0x%x\n",i,wm8805_i2c_read(dev, i));	
}

static void wm8805_write_all(struct wm8805_private *dev)
{
	unsigned char i,reg,value;

	for( i=1; i<31; i++ ){
		reg = i;
		value = wm8805_reg_defs[i];
		wm8805_i2c_write(dev, reg ,value);
	}
}

/****************** wm8740 api ****************************************/

static int wm8741_i2c_read(struct wm8741_private *dev,unsigned char reg)
{

	u_int8_t value_l=0, value_r=0;
	int ret;
	
	/* Set Index 0 */
	if(dev->enable_l){
		value_l = 0;
		ret = i2c_master_send(dev->client_l, &reg, 1);
		if (ret != 1) {
		        dev_err(&dev->client_l->dev, "Unable to write i2c index (ret=%d)\n",ret);
		        return -1;
		}   

		/* Now do Page Read */
		ret = i2c_master_recv(dev->client_l, &value_l, 1);
		if (ret != 1) {
		        dev_err(&dev->client_l->dev, "Unable to read i2c page (ret=%d)\n",ret);
		        return -1;
		}
	}

	if(dev->enable_r){
		/* Set Index 0 */
		value_r = 0;
		ret = i2c_master_send(dev->client_r, &reg, 1);
		if (ret != 1) {
		        dev_err(&dev->client_r->dev, "Unable to write i2c index (ret=%d)\n",ret);
		        return -1;
		}   

		/* Now do Page Read */
		ret = i2c_master_recv(dev->client_r, &value_r, 1);
		if (ret != 1) {
		        dev_err(&dev->client_r->dev, "Unable to read i2c page (ret=%d)\n",ret);
		        return -1;
		}
	}

	return value_l;
}
static int wm8741_i2c_write(struct wm8741_private *dev,unsigned char reg,unsigned char value)
{
	int ret;
	u_int8_t buf[2] = {reg,value};

	if(dev->enable_l){
		ret = i2c_master_send(dev->client_l, buf, 2);
		if (ret != 2) {
		        dev_err(&dev->client_l->dev, "Unable to write i2c index (ret=%d)\n",ret);
		        return -1;
		}
	}
	if(dev->enable_r){
		ret = i2c_master_send(dev->client_r, buf, 2);
		if (ret != 2) {
		        dev_err(&dev->client_r->dev, "Unable to write i2c index (ret=%d)\n",ret);
		        return -1;
		}
	}
	return 0;
}
void wm8741_codec_detect(struct wm8741_private *dev)
{
	unsigned char reg=0x02;
	
	int ret = -1, enable_l = 0, enable_r = 0;
	
	dev->enable_l = 1;
	dev->enable_r = 0;
	ret = wm8741_i2c_write(dev, reg , 0);
	if(ret)
		enable_l = 0;
	else
		enable_l = 1;

	dev->enable_l = 0;
	dev->enable_r = 1;
	ret = wm8741_i2c_write(dev, reg , 0);
	if(ret)
		enable_r = 0;
	else
		enable_r = 1;

	dev->enable_l = enable_l;
	dev->enable_r = enable_r;

	printk("audio codec detect result: enable L =%d, enable R=%d\n",enable_l,enable_r);
}


int wm8741_i2c_update_bits(struct wm8741_private *dev, unsigned short reg,
		unsigned short mask, unsigned short value)
{
	int change;
	unsigned short old, new;

	old = wm8741_reg_defs[reg];
	new = (old & ~mask) | value;
	change = (old != new);
	
	if (change){
		wm8805_i2c_write(dev, reg, new);
		wm8741_reg_defs[reg] = new;
		//printk("@@reg=0x%x  old value=0x%x,new value=0x%x\n",reg,old,new);
	}
	
	return change;
}


static int wm8741_reset(struct wm8741_private *dev)
{
	return wm8741_i2c_write(dev, WM8741_RESET, 0);
}

static int wm8741_init(struct wm8741_private *dev)
{
	#if 0
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct wm8741_priv *wm8741 = snd_soc_codec_get_drvdata(codec);
	u16 iface = snd_soc_read(codec, WM8741_FORMAT_CONTROL) & 0x1FC;
	int i;

	/* Find a supported LRCLK ratio */
	for (i = 0; i < ARRAY_SIZE(lrclk_ratios); i++) {
		if (wm8741->sysclk / params_rate(params) ==
		    lrclk_ratios[i].ratio)
			break;
	}

	/* Should never happen, should be handled by constraints */
	if (i == ARRAY_SIZE(lrclk_ratios)) {
		dev_err(codec->dev, "MCLK/fs ratio %d unsupported\n",
			wm8741->sysclk / params_rate(params));
		return -EINVAL;
	}

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0001;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0002;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		iface |= 0x0003;
		break;
	default:
		dev_dbg(codec->dev, "wm8741_hw_params:    Unsupported bit size param = %d",
			params_format(params));
		return -EINVAL;
	}

	dev_dbg(codec->dev, "wm8741_hw_params:    bit size param = %d",
		params_format(params));

	snd_soc_write(codec, WM8741_FORMAT_CONTROL, iface);
	return 0;
	#endif
	wm8741_i2c_update_bits(dev, WM8741_FORMAT_CONTROL, 0x3, 0x2);
}

static void wm8741_write_all(struct wm8741_private *dev)
{
	unsigned char i;
	unsigned char reg,value;

	for (i = 0; i < WM8741_REGISTER_COUNT; i++) 
	{
		reg = i <<1;
		value = wm8741_reg_defs[i];
		if (WM8741_RESET == i)
			continue;

		if(i == 10)
		{
			reg = 0x20 <<1;
			value = wm8741_reg_defs[i];
		}
		wm8741_i2c_write(dev, reg, value);
	}

	return 0;
}

void wm8741_dump_reg(struct wm8741_private *dev)
{
	unsigned value,i;
	for(i=0;i<WM8741_REGISTER_COUNT;i++)
		{
			value = wm8741_reg_defs[i];
//			value = wm8741_i2c_read(dev,i);
			printk("dump reg 0x%x=0x%x\n",i,value);
		}
}
static int wm8741_mclk_adapt(struct wm8741_private *dev, int systemclk ,unsigned char filter)
{
	unsigned char reg;
	int i,ret=0;

	if(filter < 0 || filter >4)
		filter = 0;
	if((systemclk/WM8805_N_FS)>=100000)
		{
			wm8741_i2c_update_bits(dev, WM8741_MODE_CONTROL_1, WM8741_OSR_MASK, 0x2 << WM8741_OSR_SHIFT);
			wm8741_i2c_update_bits(dev, WM8741_FILTER_CONTROL, WM8741_FIRSEL_MASK, filter << WM8741_FIRSEL_SHIFT);
		}
	else if((systemclk/WM8805_N_FS)>=50000)
		{
			wm8741_i2c_update_bits(dev, WM8741_MODE_CONTROL_1, WM8741_OSR_MASK, 0x1 << WM8741_OSR_SHIFT);
			wm8741_i2c_update_bits(dev, WM8741_FILTER_CONTROL, WM8741_FIRSEL_MASK, filter << WM8741_FIRSEL_SHIFT);
		}
	else
		{
			/* mark improve 44.1K fs ability*///wm8741_i2c_update_bits(dev, WM8741_MODE_CONTROL_1, WM8741_OSR_MASK, 0x0<<WM8741_OSR_SHIFT);
			wm8741_i2c_update_bits(dev, WM8741_MODE_CONTROL_1, WM8741_OSR_MASK, 0x0 << WM8741_OSR_SHIFT);
			wm8741_i2c_update_bits(dev, WM8741_FILTER_CONTROL, WM8741_FIRSEL_MASK, filter << WM8741_FIRSEL_SHIFT);

		}

	return ret;
}

static int wm8741_data_width_adapt(struct wm8741_private *dev, int width)
{
	unsigned char reg;
	int i,ret=0;

	switch(width)
		{
			case 16:
				wm8741_i2c_update_bits(dev, WM8741_FORMAT_CONTROL, WM8741_FMT_WIDTH, 0x0);
			break;
			case 20:
				wm8741_i2c_update_bits(dev, WM8741_FORMAT_CONTROL, WM8741_FMT_WIDTH, 0x01);
			break;
			case 24:
				wm8741_i2c_update_bits(dev, WM8741_FORMAT_CONTROL, WM8741_FMT_WIDTH, 0x02);
			break;
			case 32:
				wm8741_i2c_update_bits(dev, WM8741_FORMAT_CONTROL, WM8741_FMT_WIDTH, 0x03);
			break;		
			default:
				break;
		}

	return ret;
}

static int wm8741_set_volume(struct wm8741_private *dev,int level)
{
	#if 0
	wm8805_i2c_update_bits(dev, WM8805_GPO0, 0xF0 ,0x70);
	wm8805_i2c_update_bits(dev, WM8805_GPO2, 0xF0 ,0x70);
	udelay(50);
	wm8741_i2c_update_bits(dev, WM8741_DACLLSB_ATTENUATION, 0x1F, level&0x1F);
	wm8741_i2c_update_bits(dev, WM8741_DACLLSB_ATTENUATION, 0x20, 0x20);
	wm8741_i2c_update_bits(dev, WM8741_DACLMSB_ATTENUATION, 0x1F, (level&0xE0)>>5);
	wm8741_i2c_update_bits(dev, WM8741_DACLMSB_ATTENUATION, 0x20, 0x20);

	wm8741_i2c_update_bits(dev, WM8741_DACRLSB_ATTENUATION, 0x1F, level&0x1F);
	wm8741_i2c_update_bits(dev, WM8741_DACRLSB_ATTENUATION, 0x20, 0x20);
	wm8741_i2c_update_bits(dev, WM8741_DACRMSB_ATTENUATION, 0x1F, (level&0xE0)>>5);
	wm8741_i2c_update_bits(dev, WM8741_DACRMSB_ATTENUATION, 0x20, 0x20);
	udelay(50);
	wm8805_i2c_update_bits(dev, WM8805_GPO0, 0xF0 ,0xD0);
	wm8805_i2c_update_bits(dev, WM8805_GPO2, 0xF0 ,0xD0);
	#endif
	wm8741_i2c_update_bits(dev, WM8741_DACLMSB_ATTENUATION, 0x3F, ((level&0x3E0)>>5)|0x20);
	//wm8741_i2c_update_bits(dev, WM8741_DACLMSB_ATTENUATION, 0x20, 0x20);
	udelay(20);
	wm8741_i2c_update_bits(dev, WM8741_DACLLSB_ATTENUATION, 0x3F, (level&0x1F)|0x20);
	//wm8741_i2c_update_bits(dev, WM8741_DACLLSB_ATTENUATION, 0x20, 0x20);
	udelay(20);
	wm8741_i2c_update_bits(dev, WM8741_DACRMSB_ATTENUATION, 0x3F, ((level&0x3E0)>>5)|0x20);
	//wm8741_i2c_update_bits(dev, WM8741_DACRMSB_ATTENUATION, 0x20, 0x20);
	udelay(20);
	wm8741_i2c_update_bits(dev, WM8741_DACRLSB_ATTENUATION, 0x3F, (level&0x1F)|0x20);
	//wm8741_i2c_update_bits(dev, WM8741_DACRLSB_ATTENUATION, 0x20, 0x20);
	udelay(20);

	return 0;
}

static int wm8741_set_de_emph(struct wm8741_private *dev,int fs)
{
	switch(fs)
		{
			case 32000 * WM8805_N_FS:
				wm8741_i2c_update_bits(dev, WM8741_FILTER_CONTROL, WM8741_DEEMPH_MASK, 0x01<<WM8741_DEEMPH_SHIFT);
				break;
			case 44100 * WM8805_N_FS:
				wm8741_i2c_update_bits(dev, WM8741_FILTER_CONTROL, WM8741_DEEMPH_MASK, 0x02<<WM8741_DEEMPH_SHIFT);
				break;
			case 48000 * WM8805_N_FS:
				wm8741_i2c_update_bits(dev, WM8741_FILTER_CONTROL, WM8741_DEEMPH_MASK, 0x03<<WM8741_DEEMPH_SHIFT);
				break;
			default:
				wm8741_i2c_update_bits(dev, WM8741_FILTER_CONTROL, WM8741_DEEMPH_MASK, 0x00<<WM8741_DEEMPH_SHIFT);
				break;
		}

	return 0;
}

static int wm8741_shutdown(struct wm8741_private *dev)
{
	printk("wm8741_shutdown NOT implement!\n");
	return -1;
}


static int wm8741_8805_mute(struct wm8805_private *dev1, struct wm8741_private *dev2, int mute)
{
	if(mute)
		{
			//wm8805_i2c_update_bits(dev, WM8805_PWRDN, 0x04, 0x04);// power down tx0
			#ifdef EXT_OPT
			__gpio_set_pin(GPIO_DATA_CHN);
			#endif
			if(!((dev2->enable_l == 0) && (dev2->enable_r == 0)))
				{
					wm8805_i2c_update_bits(dev1, WM8805_GPO0, 0xF0 ,0x70);
					//mdelay(15);
					wm8805_i2c_update_bits(dev1, WM8805_GPO2, 0xF0 ,0x70);
				}
			udelay(50);
			wm8741_i2c_update_bits(dev2, WM8741_VOLUME_CONTROL, WM8741_AMUTE_MASK, (mute&0x01)<<WM8741_AMUTE_SHIFT);
			wm8741_i2c_update_bits(dev2, WM8741_VOLUME_CONTROL, WM8741_SOFT_MASK, (mute&0x01)<<WM8741_SOFT_SHIFT);
		}
	else
		{
			//wm8805_i2c_update_bits(dev, WM8805_PWRDN, 0x04, 0x0);// power up tx0
			wm8741_i2c_update_bits(dev2, WM8741_VOLUME_CONTROL, WM8741_AMUTE_MASK, (mute&0x01)<<WM8741_AMUTE_SHIFT);
			wm8741_i2c_update_bits(dev2, WM8741_VOLUME_CONTROL, WM8741_SOFT_MASK, (mute&0x01)<<WM8741_SOFT_SHIFT);
			udelay(50);
			if(!((dev2->enable_l == 0) && (dev2->enable_r == 0)))
				{
					wm8805_i2c_update_bits(dev1, WM8805_GPO0, 0xF0 ,0xD0);
					//mdelay(15);
					wm8805_i2c_update_bits(dev1, WM8805_GPO2, 0xF0 ,0xD0);
				}
			#ifdef EXT_OPT
			__gpio_clear_pin(GPIO_DATA_CHN);
			#endif
			//msleep(2000);
		}
	return 0;
}

static int wm8741_8805_mute2(struct wm8805_private *dev1, struct wm8741_private *dev2, int mute)
{
	if(mute){
		wm8805_i2c_update_bits(dev1, WM8805_GPO0, 0xF0 ,0x70);
		wm8805_i2c_update_bits(dev1, WM8805_GPO2, 0xF0 ,0x70);
		//udelay(50);
		wm8741_i2c_update_bits(dev2, WM8741_VOLUME_CONTROL, WM8741_AMUTE_MASK, (mute&0x01)<<WM8741_AMUTE_SHIFT);
		wm8741_i2c_update_bits(dev2, WM8741_VOLUME_CONTROL, WM8741_SOFT_MASK, (mute&0x01)<<WM8741_SOFT_SHIFT);
	}else{
		wm8741_i2c_update_bits(dev2, WM8741_VOLUME_CONTROL, WM8741_AMUTE_MASK, (mute&0x01)<<WM8741_AMUTE_SHIFT);
		wm8741_i2c_update_bits(dev2, WM8741_VOLUME_CONTROL, WM8741_SOFT_MASK, (mute&0x01)<<WM8741_SOFT_SHIFT);
		//udelay(50);
		wm8805_i2c_update_bits(dev1, WM8805_GPO0, 0xF0 ,0xD0);
		wm8805_i2c_update_bits(dev1, WM8805_GPO2, 0xF0 ,0xD0);
	}

	return 0;
}

static int codec_set_speed_ex(int rate)
{
	int speed = 0, val;

	int mrate[MAX_RATE_COUNT] = {
		//192000, /*176400,*/ 96000, 48000, 44100, 32000
		32000, 44100, 48000, 88200, 96000, 176400, 192000/*176400,*/   
	};

	for (val = 0; val < MAX_RATE_COUNT; val++) {
		//if (rate >= mrate[val]) {
		if (rate <= mrate[val]) {
			//printk("@@@rate=%d , mrate[val]l=%d,val=%d\n",rate,mrate[val],val);
			speed = val;
			break;
		}
	}
	//if (rate < mrate[MAX_RATE_COUNT - 1]) {
	if (rate > mrate[MAX_RATE_COUNT - 1]) {
		speed = MAX_RATE_COUNT - 1;
	}

	return mrate[speed];
}




/***************************************************************************************\
 *                                                                                     *
 *global variable and structure interface                                              *
 *                                                                                     *
\***************************************************************************************/

static unsigned int cur_route = -1;
unsigned int keep_old_route = -1;

//static struct workqueue_struct *dlv_work_queue;
static struct work_struct dlv_irq_work;
static spinlock_t dlv_irq_lock;

unsigned int g_current_out_dev;

#ifdef CONFIG_HP_SENSE_DETECT
static jz_hp_switch_data_t *g_switch_data = NULL;
#endif 

/*---------------------*/
static jz_dlv_platform_data_t dlv_platform_data_init_val = {
	.dlv_replay_volume_base = 0,
	.dlv_record_volume_base = 0,
	.default_replay_route = ROUTE_COUNT,
	.default_record_route = ROUTE_COUNT,
	.default_call_record_route = ROUTE_COUNT,
	.dlv_set_device = NULL,
	.dlv_set_standby = NULL,
        .dlv_set_gpio_before_set_route = NULL,
	.dlv_set_gpio_after_set_route = NULL,
	.dlv_init_part = NULL,
	.dlv_turn_off_part = NULL,
	.dlv_shutdown_part = NULL,
	.dlv_reset_part = NULL,
	.dlv_suspend_part = NULL,
	.dlv_resume_part = NULL,
	.dlv_anti_pop_part = NULL,
};

static jz_dlv_platform_data_t *dlv_platform_data = &dlv_platform_data_init_val;

static int dlv_mute(void *codec_prv, int val);

/*=================== lock ============================*/
static struct semaphore *g_dlv_sem = 0;

#define DLV_DEBUG_SEM(x,y...) //printk(x,##y);

#define DLV_LOCK()							\
	do{								\
		if(g_dlv_sem)						\
			down(g_dlv_sem);				\
		DLV_DEBUG_SEM("dlvsemlock lock\n");			\
	}while(0)

#define DLV_UNLOCK()							\
	do{								\
		if(g_dlv_sem)						\
			up(g_dlv_sem);					\
		DLV_DEBUG_SEM("dlvsemlock unlock\n");			\
	}while(0)

#define DLV_LOCKINIT()							\
	do{								\
		if(g_dlv_sem == NULL)					\
			g_dlv_sem = (struct semaphore *)vmalloc(sizeof(struct semaphore)); \
		if(g_dlv_sem)						\
			init_MUTEX_LOCKED(g_dlv_sem);			\
		DLV_DEBUG_SEM("dlvsemlock init\n");			\
	}while(0)

#define DLV_LOCKDEINIT()						\
	do{								\
		if(g_dlv_sem)						\
			vfree(g_dlv_sem);				\
		g_dlv_sem = NULL;					\
		DLV_DEBUG_SEM("dlvsemlock deinit\n");			\
	}while(0)


/*==============================================================*/
/**
 * dlv_sleep
 *
 *if use in suspend and resume, should use delay
 */
static int g_dlv_sleep_mode = 1;
void dlv_sleep(int ms)
{
	if(g_dlv_sleep_mode)
		msleep(ms);
	else
		mdelay(ms);
}

/***************************************************************************************\
 *                                                                                     *
 *debug part                                                                           *
 *                                                                                     *
\***************************************************************************************/
/*###############################################*/

#define DLV_DUMP_IOC_CMD		0
#define DLV_DUMP_ROUTE_REGS		0
#define DLV_DUMP_ROUTE_PART_REGS	0
#define DLV_DUMP_GAIN_PART_REGS		0
#define DLV_DUMP_ROUTE_NAME		1

/*##############################################*/

#if DLV_DUMP_IOC_CMD 
static void dlv_print_ioc_cmd(int cmd)
{
	int i;

	int cmd_arr[] = {
		CODEC_INIT,			CODEC_TURN_OFF,			
		CODEC_SHUTDOWN,			CODEC_RESET,
		CODEC_SUSPEND,			CODEC_RESUME,
		CODEC_ANTI_POP, 		CODEC_SET_ROUTE,
 		CODEC_SET_DEVICE,		CODEC_SET_RECORD_RATE,
 		CODEC_SET_RECORD_DATA_WIDTH, 	CODEC_SET_MIC_VOLUME,
		CODEC_SET_RECORD_CHANNEL, 	CODEC_SET_REPLAY_RATE,
 		CODEC_SET_REPLAY_DATA_WIDTH,   	CODEC_SET_REPLAY_VOLUME,
		CODEC_SET_REPLAY_CHANNEL, 	CODEC_DAC_MUTE,
		CODEC_DEBUG_ROUTINE,		CODEC_SET_STANDBY,
		CODEC_DUMP_REGS
	};

	char *cmd_str[] = {
		"CODEC_INIT", 			"CODEC_TURN_OFF",
		"CODEC_SHUTDOWN", 		"CODEC_RESET",
		"CODEC_SUSPEND",		"CODEC_RESUME",
		"CODEC_ANTI_POP", 		"CODEC_SET_ROUTE",
		"CODEC_SET_DEVICE",		"CODEC_SET_RECORD_RATE",
		"CODEC_SET_RECORD_DATA_WIDTH", 	"CODEC_SET_MIC_VOLUME",
		"CODEC_SET_RECORD_CHANNEL", 	"CODEC_SET_REPLAY_RATE",
		"CODEC_SET_REPLAY_DATA_WIDTH", 	"CODEC_SET_REPLAY_VOLUME",
		"CODEC_SET_REPLAY_CHANNEL", 	"CODEC_DAC_MUTE",
		"CODEC_DEBUG_ROUTINE",		"CODEC_SET_STANDBY",
		"CODEC_DUMP_REGS"
	};

	for ( i = 0; i < sizeof(cmd_arr) / sizeof(int); i++) {
		if (cmd_arr[i] == cmd) {
			printk("CODEC IOC: Command name : %s\n", cmd_str[i]);
			return;
		}
	}

	if (i == sizeof(cmd_arr) / sizeof(int)) {
		printk("CODEC IOC: command is not under control\n");
	}
}
#endif //DLV_DUMP_IOC_CMD 

#if DLV_DUMP_ROUTE_NAME
static void dlv_print_route_name(int route)
{
	int i;

	int route_arr[] = {
		ROUTE_ALL_CLEAR,
		ROUTE_REPLAY_CLEAR,
		ROUTE_RECORD_CLEAR,
		RECORD_MIC1_MONO_DIFF_WITH_BIAS,
		RECORD_MIC1_MONO_DIFF_WITHOUT_BIAS,
		RECORD_MIC2_MONO_DIFF_WITH_BIAS,
		RECORD_MIC2_MONO_DIFF_WITHOUT_BIAS,
		REPLAY_OUT,
		REPLAY_HP_STEREO,
		REPLAY_LINEOUT_MONO,
		REPLAY_BTL,
		BYPASS_MIC1_DIFF_WITH_BIAS_TO_OUT,
		BYPASS_MIC1_DIFF_WITH_BIAS_TO_HP,
		BYPASS_MIC1_DIFF_WITH_BIAS_TO_LINEOUT_MONO,
		BYPASS_MIC1_DIFF_WITH_BIAS_TO_BTL,
		BYPASS_MIC2_DIFF_WITH_BIAS_TO_OUT,
		BYPASS_MIC2_DIFF_WITH_BIAS_TO_HP,
		BYPASS_MIC2_DIFF_WITH_BIAS_TO_LINEOUT_MONO,
		BYPASS_MIC2_DIFF_WITH_BIAS_TO_BTL,
		BYPASS_LINEIN_TO_OUT,
		BYPASS_LINEIN_TO_HP,
		BYPASS_LINEIN_TO_LINEOUT_MONO,
		BYPASS_LINEIN_TO_BTL,
		RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_OUT,
		RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_HP,
		RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_LINEOUT_MONO,
		RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_BTL,
		RECORD_MIC2_MONO_DIFF_WITHOUT_BIAS_REPLAY_LINEOUT_MONO
	};

	char *route_str[] = {
		"ROUTE_ALL_CLEAR",
		"ROUTE_REPLAY_CLEAR",
		"ROUTE_RECORD_CLEAR",
		"RECORD_MIC1_MONO_DIFF_WITH_BIAS",
		"RECORD_MIC1_MONO_DIFF_WITHOUT_BIAS",
		"RECORD_MIC2_MONO_DIFF_WITH_BIAS",
		"RECORD_MIC2_MONO_DIFF_WITHOUT_BIAS",
		"REPLAY_OUT",
		"REPLAY_HP_STEREO",
		"REPLAY_LINEOUT_MONO",
		"REPLAY_BTL",
		"BYPASS_MIC1_DIFF_WITH_BIAS_TO_OUT",
		"BYPASS_MIC1_DIFF_WITH_BIAS_TO_HP",
		"BYPASS_MIC1_DIFF_WITH_BIAS_TO_LINEOUT_MONO",
		"BYPASS_MIC1_DIFF_WITH_BIAS_TO_BTL",
		"BYPASS_MIC2_DIFF_WITH_BIAS_TO_OUT",
		"BYPASS_MIC2_DIFF_WITH_BIAS_TO_HP",
		"BYPASS_MIC2_DIFF_WITH_BIAS_TO_LINEOUT_MONO",
		"BYPASS_MIC2_DIFF_WITH_BIAS_TO_BTL",
		"BYPASS_LINEIN_TO_OUT",
		"BYPASS_LINEIN_TO_HP",
		"BYPASS_LINEIN_TO_LINEOUT_MONO",
		"BYPASS_LINEIN_TO_BTL",
		"RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_OUT",
		"RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_HP",
		"RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_LINEOUT_MONO",
		"RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_BTL",
		"RECORD_MIC2_MONO_DIFF_WITHOUT_BIAS_REPLAY_LINEOUT_MONO",
	};

	for ( i = 0; i < sizeof(route_arr) / sizeof(unsigned int); i++) {
		if (route_arr[i] == route) {
			printk("\nCODEC SET ROUTE: Route name : %s\n", route_str[i]);
			return;
		}
	}

	if (i == sizeof(route_arr) / sizeof(unsigned int)) {
		printk("\nCODEC SET ROUTE: Route is not configed yet!\n");
	}
}
#endif //DLV_DUMP_ROUTE_NAME

void dump_dlv_regs(void){
}

static int dlv_dump_regs(void *codec_prv)
{
	struct codec_data * codec = (struct codec_data *)codec_prv;
	struct wm8805_private* wm8805_data = (struct wm8805_private*) codec->clk_unit;
	struct wm8741_private* wm8741_data = (struct wm8741_private*) codec->dlv_unit;

	if(wm8805_data)
		wm8805_dump_all(wm8805_data);
	
	return 0;
}

#if 0

void dump_dlv_route_regs(void)
{
	unsigned int i;
	unsigned char data;
	for (i = 0x2; i < 0xA; i++) {
		data = dlv_read_reg(i);
		printk("address = 0x%02x, data = 0x%02x\n", i, data);
	}
}

void dump_dlv_gain_regs(void)
{
	unsigned int i;
	unsigned char data;
	for (i = 0xC; i < 0x15; i++) {
		data = dlv_read_reg(i);
		printk("address = 0x%02x, data = 0x%02x\n", i, data);
	}
}
#endif
/*=========================================================*/

#if DLV_DUMP_ROUTE_NAME
#define DUMP_ROUTE_NAME(route) dlv_print_route_name(route)
#else //DLV_DUMP_ROUTE_NAME
#define DUMP_ROUTE_NAME(route)
#endif //DLV_DUMP_ROUTE_NAME

/*-------------------*/
#if DLV_DUMP_IOC_CMD 
#define DUMP_IOC_CMD()								\
	do {									\
		printk("[dlv IOCTL]++++++++++++++++++++++++++++\n");		\
		printk("%s  cmd = %d, arg = %lu\n", __func__, cmd, arg); 	\
		dlv_print_ioc_cmd(cmd);						\
		printk("[dlv IOCTL]----------------------------\n");		\
										\
	} while (0)
#else //DLV_DUMP_IOC_CMD
#define DUMP_IOC_CMD()	
#endif //DLV_DUMP_IOC_CMD

#if DLV_DUMP_ROUTE_REGS
#define DUMP_ROUTE_REGS(value)							\
	do {									\
		printk("codec register dump,%s\tline:%d-----%s:\n",		\
		       __func__, __LINE__, value);				\
		dump_dlv_regs();						\
										\
	} while (0)
#else //DLV_DUMP_ROUTE_REGS
#define DUMP_ROUTE_REGS(value)
#endif //DLV_DUMP_ROUTE_REGS

#if DLV_DUMP_ROUTE_PART_REGS
#define DUMP_ROUTE_PART_REGS(value)						\
	do {									\
		if (mode != DISABLE) {						\
			printk("codec register dump,%s\tline:%d-----%s:\n", 	\
			       __func__, __LINE__, value);			\
			dump_dlv_route_regs();					\
		}								\
										\
	} while (0)
#else //DLV_DUMP_ROUTE_PART_REGS
#define DUMP_ROUTE_PART_REGS(value)
#endif //DLV_DUMP_ROUTE_PART_REGS

#if DLV_DUMP_GAIN_PART_REGS
#define DUMP_GAIN_PART_REGS(value)						\
	do {									\
		printk("codec register dump,%s\tline:%d-----%s:\n", 		\
		       __func__, __LINE__, value);				\
		dump_dlv_gain_regs();						\
										\
	} while (0)
#else //DLV_DUMP_GAIN_PART_REGS
#define DUMP_GAIN_PART_REGS(value)
#endif //DLV_DUMP_GAIN_PART_REGS


/***************************************************************************************\
 *                                                                                     *
 *dlv route                                                                            *
 *                                                                                     *
\***************************************************************************************/

static void dlv_set_route_base(const void *arg)
{
	route_conf_base *conf = (route_conf_base *)arg;

}

/***************************************************************************************\
 *                                                                                     *
 *ioctl support function                                                               *
 *                                                                                     *
\***************************************************************************************/

/*------------------sub fun-------------------*/

/**
 * CODEC set gpio before set route and dlv set gpio after set route
 *
 * these two function below is just demo frames, they should be realized 
 * depend on difficent boards, they should not be modifiy here
 *
 **/

static int dlv_set_gpio_before_set_route(int route)
{
	printk("WM8741_DLV: waring, %s() is a default function\n", __func__);

	switch(route){

	case ROUTE_ALL_CLEAR:
		break;

	case RECORD_MIC1_MONO_DIFF_WITH_BIAS:
		break;

	case REPLAY_HP_STEREO:
		break;

	/* and so on */

	default:
		printk("%s: dlv set route gpio error!, undecleard route\n", __func__);
	}
	
	return 0;
}

static int dlv_set_gpio_after_set_route(int route)
{
	printk("WM8741_DLV: waring, %s() is a default function\n", __func__);

	switch(route){

	case ROUTE_ALL_CLEAR:
		break;

	case RECORD_MIC1_MONO_DIFF_WITH_BIAS:
		break;

	case REPLAY_HP_STEREO:
		break;
		
	/* and so on */

	default:
		printk("%s: dlv set route gpio error!, undecleard route\n", __func__);
	}

	return 0;
}

/*-----------------main fun-------------------*/

int dlv_set_route(int route)
{
	int i = 0;

	DUMP_ROUTE_REGS("enter");

	/* set gpio befor set route */
	if(dlv_platform_data->dlv_set_gpio_before_set_route)
	{
		if(dlv_platform_data->dlv_set_gpio_before_set_route(route))
		{
			dlv_set_gpio_before_set_route(route);
		}
	} else
		dlv_set_gpio_before_set_route(route);

	/* set route */
	DUMP_ROUTE_NAME(route);

	if(cur_route != route)
	{
		for (i = 0; i < ROUTE_COUNT; i ++)
		{
			if (route == dlv_route_info[i].route_name)
			{
				/* set route */
				dlv_set_route_base(dlv_route_info[i].route_conf);
				/* keep_old_route is used in resume part */
				keep_old_route = cur_route;
				/* change cur_route */
				cur_route = route;
				break;
			}
		}
		if (i == ROUTE_COUNT)
			printk("SET_ROUTE: dlv set route error!, undecleard route, route = %d\n", route);
	} else 
		printk("SET_ROUTE: need not to set!, current route is route now!\n");
	
	/* set gpio after set route */
	if(dlv_platform_data->dlv_set_gpio_after_set_route)
	{
		if(dlv_platform_data->dlv_set_gpio_after_set_route(route))
		{
			dlv_set_gpio_after_set_route(route);
		}
	} else
		dlv_set_gpio_after_set_route(route);

	DUMP_ROUTE_REGS("leave");

	return cur_route;
}

/*----------------------------------------*/
/****** dlv_init ********/
/**
 * CODEC dlv init part
 *
 * it will do the initialization as default, it can be recode 
 * depend on difficent boards if necessary
 *
 **/

static int dlv_init_part(void)
{
	printk("WM8741_DLV: waring, %s() is a default function\n", __func__);

	return 0;
}

static int dlv_init(void *codec)
{
	int ret;
	struct codec_data * codec_prv = (struct codec_data *)codec;
	
	DLV_LOCKINIT();

	/* set default route */
	if(dlv_platform_data->default_replay_route && (dlv_platform_data->default_replay_route != ROUTE_COUNT))
		DEFAULT_REPLAY_ROUTE = dlv_platform_data->default_replay_route;

	if(dlv_platform_data->default_record_route && (dlv_platform_data->default_record_route != ROUTE_COUNT))
		DEFAULT_RECORD_ROUTE = dlv_platform_data->default_record_route;

	if(dlv_platform_data->default_call_record_route && (dlv_platform_data->default_call_record_route != ROUTE_COUNT))
		DEFAULT_CALL_RECORD_ROUTE = dlv_platform_data->default_call_record_route;

	g_current_out_dev = DEFAULT_REPLAY_ROUTE;

	/* dlv init */
	if(dlv_platform_data->dlv_init_part)
	{
		ret = dlv_platform_data->dlv_init_part();
		if(ret)
		{
			ret = dlv_init_part();
		}
	} else
		ret = dlv_init_part();

	return ret;
}

/****** dlv_turn_off ********/
/**
 * CODEC dlv turn off part
 *
 * it will turn off the codec by modes as default, it can be recode 
 * depend on difficent boards if necessary
 *
 **/
static int dlv_turn_off_part(int mode)
{
	int ret;
	int route = keep_old_route;

	printk("WM8741_DLV: waring, %s() is a default function\n", __func__);


	return 0;
}

static int dlv_turn_off(int mode)
{
	int ret;

	if(dlv_platform_data->dlv_turn_off_part)
	{
		ret = dlv_platform_data->dlv_turn_off_part(mode);
		if(ret)
		{
			ret = dlv_turn_off_part(mode);
		}
	} else
		ret = dlv_turn_off_part(mode);

	DLV_LOCKDEINIT();  

	return ret;
}

/****** dlv_shutdown *******/
/**
 * CODEC dlv shutdown part
 *
 * it will shutdown the gpio when system shutdown,
 * it can be recode depend on difficent boards if necessary
 *
 **/

static int dlv_shutdown_part(void)
{
	printk("WM8741_DLV: waring, %s() is a default function\n", __func__);

	return 0;
}

static int dlv_shutdown(void)
{
	int ret;
		
	if(dlv_platform_data->dlv_shutdown_part)
	{
		ret = dlv_platform_data->dlv_shutdown_part();
		if(ret)
		{
			ret = dlv_shutdown_part();
		}
	} else
		ret = dlv_shutdown_part();

	return ret;
}

/****** dlv_reset **********/
/**
 * CODEC dlv reset part
 *
 * it will run to set the codec when codec power on as default,
 * it can be recode depend on difficent boards if necessary
 *
 **/
static int dlv_reset_part(void)
{
	printk("WM8741_DLV: waring, %s() is a default function\n", __func__);

	/* select serial interface and work mode of adc and dac */

	return 0;
}

static int dlv_reset(void)
{
	int ret;

	if(dlv_platform_data->dlv_reset_part)
	{
		ret = dlv_platform_data->dlv_reset_part();
		if(ret)
		{
			ret = dlv_reset_part();
		}
	} else
		ret = dlv_reset_part();

	return ret;
}

/******** dlv_suspend ************/
/**
 * CODEC dlv suspend part
 *
 * it will do the suspend as default, it can be recode 
 * depend on difficent boards if necessary
 *
 **/
static int dlv_suspend_part(void)
{
	int ret;

	printk("WM8741_DLV: waring, %s() is a default function\n", __func__);

	ret = dlv_set_route(ROUTE_ALL_CLEAR);
	if(ret != ROUTE_ALL_CLEAR)
	{
		printk("WM8741 CODEC: dlv_suspend_part error!\n");
		return -1;
	}

	return 0;
}

static int dlv_suspend(void)
{
	int ret;
	g_dlv_sleep_mode = 0;

	if(dlv_platform_data->dlv_suspend_part)
	{
		ret = dlv_platform_data->dlv_suspend_part();
		if(ret)
		{
			ret = dlv_suspend_part();
		}
	} else
		ret = dlv_suspend_part();

	return ret;
}

/********* dlv_resume ***********/
/**
 * CODEC dlv resume part
 *
 * it will do the resume as default, it can be recode 
 * depend on difficent boards if necessary
 *
 **/

static int dlv_resume_part(void)
{
	int ret;
	int route = keep_old_route;

	printk("WM8741_DLV: waring, %s() is a default function\n", __func__);

	/*default, the resume will restore the route before suspend*/
	ret = dlv_set_route(route);

	if(ret != route)
	{
		printk("WM8741 CODEC: dlv_resume_part error!\n");
		return -1;
	}

	return 0;
}

static int dlv_resume(void* ex_codec)
{
	int ret;
	if(dlv_platform_data->dlv_resume_part)
	{
		ret = dlv_platform_data->dlv_resume_part();
		if(ret)
		{
			ret = dlv_resume_part();
		}
	} else
		ret = dlv_resume_part();

	g_dlv_sleep_mode = 1;

	return ret;
}

/*---------------------------------------*/

/**
 * CODEC set device
 *
 * this is just a demo function, and it will be use as default 
 * if it is not realized depend on difficent boards 
 *
 */
static int dlv_set_device(struct snd_device_config *snd_dev_cfg)
{
	int ret;
	int iserror = 0;

	printk("WM8741_DLV: waring, %s() is a default function\n", __func__);

	switch (snd_dev_cfg->device) {

	case SND_DEVICE_HEADSET:
		ret = dlv_set_route(REPLAY_HP_STEREO);
		if(ret != REPLAY_HP_STEREO)
		{
			printk("WM8741 CODEC: set device SND_DEVICE_HEADSET error!\n");
			return -1;
		}
		break;

	case SND_DEVICE_HANDSET:
		ret = dlv_set_route(REPLAY_LINEOUT_MONO);
		if(ret != REPLAY_LINEOUT_MONO)
		{
			printk("WM8741 CODEC: set device SND_DEVICE_HANDSET error!\n");
			return -1;
		}
		break;

	case SND_DEVICE_SPEAKER:
		ret = dlv_set_route(REPLAY_BTL);
		if(ret != REPLAY_BTL)
		{
			printk("WM8741 CODEC: set device SND_DEVICE_SPEAKER error!\n");
			return -1;
		}
		break;

	case SND_DEVICE_HEADSET_AND_SPEAKER:
		ret = dlv_set_route(REPLAY_BTL);
		if(ret != REPLAY_BTL)
		{
			printk("WM8741 CODEC: set device SND_DEVICE_HEADSET_AND_SPEAKER error!\n");
			return -1;
		}			
		break;

	default:
		iserror = 1;
		printk("WM8741 DLV: Unkown ioctl argument in SND_SET_DEVICE\n");
	};

	if (!iserror)
		g_current_out_dev = snd_dev_cfg->device;

	return 0;
}

/*---------------------------------------*/

/**
 * CODEC set standby
 *
 * this is just a demo function, and it will be use as default 
 * if it is not realized depend on difficent boards 
 *
 */

static int dlv_set_standby(unsigned int sw)
{
	printk("WM8741_DLV: waring, %s() is a default function\n", __func__);

	switch(g_current_out_dev) {

	case SND_DEVICE_HEADSET:
		if (sw == POWER_ON) {
			/* set the relevant route */
		} else {
			/* clean the relevant route */
		}
		break;

	case SND_DEVICE_HANDSET:
		if (sw == POWER_ON) {
			/* set the relevant route */
		} else {
			/* clean the relevant route */
		}
		break;

	case SND_DEVICE_SPEAKER:
		if (sw == POWER_ON) {
			/* set the relevant route */
		} else {
			/* clean the relevant route */
		}
		break;

	case SND_DEVICE_HEADSET_AND_SPEAKER:
		if (sw == POWER_ON) {
			/* set the relevant route */
		} else {
			/* clean the relevant route */
		}
		break;

	default:
		printk("WM8741 DLV: Unkown ioctl argument in SND_SET_STANDBY\n");

	}

	return 0;
}

/*---------------------------------------*/
/**
 * CODEC set record rate & data width & volume & channel  
 *
 */

static int dlv_set_record_rate(int rate)
{
	int speed = 0, val;
	int mrate[MAX_RATE_COUNT] = {
		96000, 48000, 44100, 32000, 24000,
		22050, 16000, 12000, 11025, 8000
	};

	for (val = 0; val < MAX_RATE_COUNT; val++) {
		if (rate >= mrate[val]) {
			speed = val;
			break;
		}
	}
	if (rate < mrate[MAX_RATE_COUNT - 1]) {
		speed = MAX_RATE_COUNT - 1;
	}

//	__dlv_select_adc_samp_rate(speed);

	return mrate[speed];
}

static int dlv_set_record_data_width(int width)
{
	int supported_width[4] = {16, 18, 20, 24};
	int fix_width;

	for(fix_width = 0; fix_width < 3; fix_width ++)
	{
		if (width <= supported_width[fix_width])
			break;
	}

//	__dlv_select_dac_word_length(fix_width);

	return width;
}

static int dlv_set_record_volume(int val)
{
	/*just set analog gm1 and gm2*/
	int fixed_vol;
	int volume_base;

	if(dlv_platform_data->dlv_record_volume_base)
	{
		volume_base = dlv_platform_data->dlv_record_volume_base;

		fixed_vol = (volume_base >> 2) + 
			     ((5 - (volume_base >> 2)) * val / 100);		
	}
	else 
		fixed_vol = (5 * val / 100);

	printk("%s: Fix Me !!! (line:%d)\n",__func__,__LINE__);
	
	return val;
}

static int dlv_set_record_channel(int channel)
{
	return channel;
}

/*---------------------------------------*/
/**
 * CODEC set replay rate & data width & volume & channel  
 *
 */

static int dlv_set_replay_rate(void *codec_prv, int rate)
{
	struct codec_data * codec = (struct codec_data *)codec_prv;
	struct wm8805_private* wm8805_data = (struct wm8805_private*) codec->clk_unit;
	struct wm8741_private* wm8741_data = (struct wm8741_private*) codec->dlv_unit;

	int speed = 0, val;
	int mrate[MAX_RATE_COUNT] = { 
		96000, 48000, 44100, 32000, 24000,
		22050, 16000, 12000, 11025, 8000
	};

	for (val = 0; val < MAX_RATE_COUNT; val++) {
		if (rate >= mrate[val]) {
			speed = val;
			break;
		}
	}
	if (rate < mrate[MAX_RATE_COUNT - 1]) {
		speed = MAX_RATE_COUNT - 1;
	}

#if 0
	if( mrate[speed] == 48000 ){
		cpm_set_clock(CGU_I2SCLK, JZ_EXTAL);
	} else if( mrate[speed] == 44100 ){
		cpm_set_clock(CGU_I2SCLK, 11289600);
	}
#endif

	wm8805_set_clkdiv(wm8805_data,WM8805_CLKOUT_DIV,0x02);//128fs
	mdelay(100);
	wm8805_set_fmt(wm8805_data,0);
	//      wm8805_set_pll(wm8805_data,0,0,12000000,24576000);
	//      wm8805_set_pll(wm8805_data,0,0,WM8805_SRC_CLK,0 * WM8805_N_FS/*44100*128*/);
	wm8805_set_pll(wm8805_data,0,0,WM8805_SRC_CLK,DEFAULT_SAMPLE_RATE * WM8805_N_FS/*44100*128*/);
	//wm8805_pll_test();
	wm8805_set_sysclk(wm8805_data,WM8805_TX_CLKSRC_PLL,12000000,0);

	return mrate[speed];
}

static int dlv_set_replay_data_width(void *codec_prv, int width)
{
	int supported_width[4] = {16, 18, 20, 24};
	int fix_width;

	for(fix_width = 0; fix_width < 3; fix_width ++)
	{
		if (width <= supported_width[fix_width])
			break;
	}

//	__dlv_select_dac_word_length(fix_width);

	return width;
}

static int dlv_set_replay_volume(void *codec_prv, int val)
{
	struct codec_data * codec = (struct codec_data *)codec_prv;
	struct wm8805_private* wm8805_data = (struct wm8805_private*) codec->clk_unit;
	struct wm8741_private* wm8741_data = (struct wm8741_private*) codec->dlv_unit;
	
	val = 255 * (100 - val) / 100;
	val &= 0xff;

	if(wm8741_data)
		wm8741_set_volume(wm8741_data, val);

	return val;
}

static int dlv_set_replay_channel(void *codec_prv, int channel)
{
	channel = (channel >= 2) + 1;

	switch (channel) {
	case 1:
		// MONO->1 for Mono

		break;
	case 2:
		// MONO->0 for Stereo

		break;
	}

	return channel;
}

/*---------------------------------------*/
/**
 * CODEC set mute
 *  
 * set dac mute used for anti pop
 *
 */

static int dlv_mute(void *codec_prv, int val)
{
	struct codec_data * codec = (struct codec_data *)codec_prv;
	struct wm8805_private* wm8805_data = (struct wm8805_private*) codec->clk_unit;
	struct wm8741_private* wm8741_data = (struct wm8741_private*) codec->dlv_unit;

	wm8741_8805_mute(wm8805_data, wm8741_data, val);

	return 0;
}

static int dlv_bsp_mute(int val)
{
	int ret = -1;

	if(dlv_platform_data->dlv_board_mute){
		if(val == 0)
			ret = dlv_platform_data->dlv_board_mute(0);
		else
			ret = dlv_platform_data->dlv_board_mute(1);
	}
	
	if(ret) printk("dlv_bsp_mute: enable=%d fail!\n",val);
	
	return ret;
}
/*---------------------------------------*/

static int dlv_debug_routine(void *arg)
{
	return 0;
}

/***************************************************************************************\
 *                                                                                     *
 *control interface                                                                    *
 *                                                                                     *
\***************************************************************************************/
/**
 * CODEC ioctl (simulated) routine
 *
 * Provide control interface for i2s driver
 */
static int jzdlv_ioctl(void *context, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct codec_data * codec_prv = ((struct i2s_codec*)context)->codec_private;

	DUMP_IOC_CMD();

	DLV_LOCK();
	{
		switch (cmd) {

		case CODEC_INIT:
	//		ret = dlv_init();
			if(codec_prv->func.dlv_init)
				ret = codec_prv->func.dlv_init(codec_prv);
			break;

		case CODEC_TURN_OFF:
	//		ret = dlv_turn_off(arg);
			break;

		case CODEC_SHUTDOWN:
	//		ret = dlv_shutdown();
			break;

		case CODEC_RESET:
	//		ret = dlv_reset();
			break;

		case CODEC_SUSPEND:
	//		ret = dlv_suspend();
			break;

		case CODEC_RESUME:
	//		ret = dlv_resume();
			break;

		case CODEC_ANTI_POP:
	//		ret = dlv_anti_pop((int)arg);
			break;

		case CODEC_SET_ROUTE:
	//		ret = dlv_set_route((int)arg);
			break;

		case CODEC_SET_DEVICE:
		#if 0
			if (dlv_platform_data->dlv_set_device)
			{
				ret = dlv_platform_data->dlv_set_device((struct snd_device_config *)arg);
				if (ret)
				{
					ret = dlv_set_device((struct snd_device_config *)arg);
				}
			} else
				ret = dlv_set_device((struct snd_device_config *)arg);
		#endif
			break;

		case CODEC_SET_STANDBY:
		#if 0
			if (dlv_platform_data->dlv_set_standby)
			{
				ret = dlv_platform_data->dlv_set_standby((unsigned int)arg);
				if (ret)
				{
					ret = dlv_set_standby((unsigned int)arg);
				}
			} else
				ret = dlv_set_standby((unsigned int)arg);
		#endif
			break;

		case CODEC_SET_RECORD_RATE:
	//		ret = dlv_set_record_rate((int)arg);
			break;

		case CODEC_SET_RECORD_DATA_WIDTH:
	//		ret = dlv_set_record_data_width((int)arg);
			break;

		case CODEC_SET_MIC_VOLUME:
	//		ret = dlv_set_record_volume((int)arg);
			break;

		case CODEC_SET_RECORD_CHANNEL:
	//		ret = dlv_set_record_channel((int)arg);
			break;

		case CODEC_SET_REPLAY_RATE:
	//		ret = dlv_set_replay_rate((int)arg);
			if(codec_prv->func.dlv_set_replay_rate)
				ret = codec_prv->func.dlv_set_replay_rate(codec_prv,(int)arg);
			break;

		case CODEC_SET_REPLAY_DATA_WIDTH:
	//		ret = dlv_set_replay_data_width((int)arg);
			break;

		case CODEC_SET_REPLAY_VOLUME:
	//		ret = dlv_set_replay_volume((int)arg);
			if(codec_prv->func.dlv_set_replay_volume)
				ret = codec_prv->func.dlv_set_replay_volume(codec_prv,(int)arg);
	
			break;

		case CODEC_SET_REPLAY_CHANNEL:
	//		ret = dlv_set_replay_channel((int)arg);
			break;

		case CODEC_DAC_MUTE:
	//		ret = dlv_mute((int)arg);
			break;

		case CODEC_BSP_MUTE:
			ret = dlv_bsp_mute((int)arg);
			break;
		case CODEC_DEBUG_ROUTINE:
	//		ret = dlv_debug_routine((void *)arg);
			break;

		case CODEC_DUMP_REGS:
			if(codec_prv->func.dlv_dump_regs)
				ret = codec_prv->func.dlv_dump_regs(codec_prv);
			break;

		default:
			printk("WM8741 DLV:%s:%d: Unkown IOC commond\n", __FUNCTION__, __LINE__);
			ret = -1;
		}
	}
	DLV_UNLOCK();

	return ret;
}




#ifdef CONFIG_HP_SENSE_DETECT

/*
 * Headphone sense switch registration & initialization
 */
static ssize_t jz_hp_switch_print_state(struct switch_dev *sdev, char *buf)
{
	jz_hp_switch_data_t	*switch_data =
		container_of(sdev, jz_hp_switch_data_t, sdev);
	const char *state;

	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

static int jz_hp_switch_probe(struct platform_device *pdev)
{
	jz_hp_switch_data_t *switch_data;
	jz_hp_switch_platform_data_t *pdata = pdev->dev.platform_data;
	int ret = 0;

	switch_data = kzalloc(sizeof(jz_hp_switch_data_t), GFP_KERNEL);
	if (!switch_data) {
		printk("WM8741 HP Switch kzalloc failed (%s:%d)\n", __FUNCTION__, __LINE__);
		return -ENOMEM;
	}
	g_switch_data = switch_data;

	switch_data->sdev.name		= pdata->name;
	switch_data->name_on		= pdata->name_on;
	switch_data->name_off		= pdata->name_off;
	switch_data->state_on		= pdata->state_on;
	switch_data->state_off		= pdata->state_off;
	switch_data->sdev.print_state	= jz_hp_switch_print_state;

	if ((ret = switch_dev_register(&switch_data->sdev))) {
		printk("WM8741 HP Switch: Could net register switch device\n");
		return ret;
	}

	ret = __dlv_get_irq_flag();

	switch_set_state(&switch_data->sdev, dlv_read_reg(DLV_REG_IFR) & (1 << IFR_JACK));

	return 0;
}

static int __devexit jz_hp_switch_remove(struct platform_device *pdev)
{
	switch_dev_unregister(&g_switch_data->sdev);
	kfree(g_switch_data);

	return 0;
}

static struct platform_driver jz_hp_switch_driver = {
	.probe		= jz_hp_switch_probe,
	.remove		= __devexit_p(jz_hp_switch_remove),
	.driver		= {
		.name	= "hp-switch",
		.owner	= THIS_MODULE,
	},
};
#endif  // ifdef CONFIG_HP_SENSE_DETECT


static int set_clk_unit(void* dev)
{
	g_codec_data->clk_unit = dev;
	return 0;
}
static int set_dlv_unit(void* dev)
{
	g_codec_data->dlv_unit = dev;
	return 0;
}


/******************************* wm8805 i2c ********************************************/

static int wm8805_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct wm8805_private *wm8805_pdata;
    int err = 0;
	
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
            err = -ENODEV;
            goto exit_check_functionality_failed;
    }
	
	wm8805_pdata = kzalloc(sizeof(struct wm8805_private), GFP_KERNEL);
    if (!wm8805_pdata) {
            err = -ENOMEM;
            goto exit_alloc_data_failed;
    }

	set_clk_unit((void *)wm8805_pdata);
	
	wm8805_pdata->client = client;
    i2c_set_clientdata(client, wm8805_pdata);
	mdelay(200);

	/* delay for i2c bus ready ... */
	mdelay(200);

	err = wm8805_chip(wm8805_pdata);
	if(err < 0){
		printk("Read CHIP ID Error!!!\n");
		goto chip_err;
	}
	
	wm8805_reset(wm8805_pdata);
	wm8805_write_all(wm8805_pdata);
	wm8805_set_clkdiv(wm8805_pdata,WM8805_CLKOUT_DIV,0x02);//128fs
	mdelay(100);
	wm8805_set_fmt(wm8805_pdata,0);
//	wm8805_set_pll(wm8805_pdata,0,0,12000000,24576000);
//	wm8805_set_pll(wm8805_pdata,0,0,WM8805_SRC_CLK,0 * WM8805_N_FS/*44100*128*/);
	wm8805_set_pll(wm8805_pdata,0,0,WM8805_SRC_CLK,DEFAULT_SAMPLE_RATE * WM8805_N_FS/*44100*128*/);
	wm8805_set_sysclk(wm8805_pdata,WM8805_TX_CLKSRC_PLL,12000000,0);
	wm8805_hw_params(wm8805_pdata);

	printk("wm8805 init done!\n");
	
	return 0;

chip_err:
	kfree(wm8805_pdata);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;

}

static int wm8805_i2c_remove(struct i2c_client *client)
{
        struct wm8805_private *priv = dev_get_drvdata(&client->dev);

        kfree(priv);

        dev_set_drvdata(&client->dev, NULL);

        return 0;
}

static const struct i2c_device_id wm8805_i2c_id[] = { 
        { "wm8805", 0 },
};
MODULE_DEVICE_TABLE(i2c, wm8805_i2c_id);

static struct i2c_driver wm8805_i2c_driver = { 
        .probe          = wm8805_i2c_probe,
        .remove         = __devexit_p(wm8805_i2c_remove),
        .id_table       = wm8805_i2c_id,
        .driver = { 
                .name   = "wm8805",
        },  
};

/*------------------------------------------*/

static int jz_dlv_probe(struct platform_device *pdev)
{
	int err = 0;
	struct codec_data *codec_prv;
	
	printk("WM8805 & WM8741 CODEC probe!\n");

	dlv_platform_data = pdev->dev.platform_data;

	dlv_reset();

	codec_prv = kzalloc(sizeof(struct codec_data), GFP_KERNEL);
    if (!codec_prv) {
            err = -ENOMEM;
            goto init_fail;
    }
	
	g_codec_data = codec_prv;
	
	codec_prv->clk_unit = NULL;
	codec_prv->dlv_unit = NULL;

	codec_prv->func.dlv_init = dlv_init;
	codec_prv->func.dlv_dump_regs = dlv_dump_regs;
	codec_prv->func.dlv_set_replay_rate = dlv_set_replay_rate;

	register_jz_codecs_ex((void *)jzdlv_ioctl, codec_prv);

	i2c_add_driver(&wm8805_i2c_driver);
	
	printk("WM8805 & WM8741 CODEC probe done!\n");
	return 0;

init_fail:
		printk("WM8805 & WM8741 CODEC probe fail!\n");
	return err;
}

static int __devexit jz_dlv_remove(struct platform_device *pdev)
{
	dlv_platform_data = NULL;
	return 0;
}

static struct platform_driver jz_dlv_driver = {
	.probe		= jz_dlv_probe,
	.remove		= __devexit_p(jz_dlv_remove),
	.driver		= {
		.name	= "wm8805_8741_dlv",
		.owner	= THIS_MODULE,
	},
};


/***************************************************************************************\
 *                                                                                     *
 *module init                                                                          *
 *                                                                                     *
\***************************************************************************************/

/**
 * Module init
 */
static int __init init_dlv(void)
{
	int retval;

	cpm_start_clock(CGM_AIC);

#if 0
	spin_lock_init(&dlv_irq_lock);

	INIT_WORK(&dlv_irq_work, dlv_irq_work_handler);

	dlv_work_queue = create_singlethread_workqueue("dlv_irq_wq");

	if (!dlv_work_queue) {
		// this can not happen, if happen, we die!
		BUG();
	}
#endif


#if 0
	retval = request_irq(IRQ_AIC, dlv_codec_irq, IRQF_DISABLED, "dlv_codec_irq", NULL);
	if (retval) {
		printk("WM8741 DLV: Could not get AIC CODEC irq %d\n", IRQ_AIC);
		return retval;
	}
#endif


#ifdef CONFIG_HP_SENSE_DETECT
	retval = platform_driver_register(&jz_hp_switch_driver);
	if (retval) {
		printk("WM8741 HP Switch: Could net register headphone sense switch\n");
		return retval;
	}
#endif

	retval = platform_driver_register(&jz_dlv_driver);
	if (retval) {
		printk("WM8741 CODEC: Could net register jz_dlv_driver\n");
		return retval;
	}

	return 0;

fail:
	printk("WM8805 & WM8741 CODEC register fail!\n\n");
	
	return -1;
}

/**
 * Module exit
 */
static void __exit cleanup_dlv(void)
{

	free_irq(IRQ_AIC, NULL);
#ifdef CONFIG_HP_SENSE_DETECT
	platform_driver_unregister(&jz_hp_switch_driver);
#endif
	platform_driver_unregister(&jz_dlv_driver);
}

module_init(init_dlv);
module_exit(cleanup_dlv);
