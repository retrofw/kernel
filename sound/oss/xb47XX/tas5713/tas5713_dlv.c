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

#include <asm/hardirq.h>
#include <asm/jzsoc.h>
//#include <mach/chip-aic.h>
//#include <linux/jz_audio.h>

#include "../jz47XX_codec.h"

#include <linux/i2c.h>

#define DUMP_FUNC() printk("TAS5713_DLV:%s\tline:%d\n", __func__, __LINE__)

#define GPIO_TAS5713_RESET  		(32*1 + 16)
#define GPIO_TAS5713_PDN		(32*4 + 4)
//#define GPIO_TAS5713_INPUT_SELECT	(32*5 + 3)

struct tas5713_private {
	unsigned int mclk; /* Input frequency of the MCLK pin */
	unsigned int mode; /* The mode (I2S or left-justified) */
	unsigned int slave_mode;
	unsigned int manual_mute;
	int i2c_handle;
//	struct miscdevice dev;
};

static struct tas5713_private tas5713_data;

static int tas5713_dac_flag = 0;

//static unsigned char tas5713_reg_defs[128];
#define TAS5713_CHIPADDR	0x36

#define REG_CLOCK_CONTROL	0x00
#define REG_DEVICE_ID		0x01
#define REG_ERROR_STATUS	0x02
#define REG_SYSTEM_CONTROL1	0x03
#define REG_I2S_MODE	0x04
#define REG_SYSTEM_CONTROL2	0x05
#define REG_SOFT_MUTE	0x06
#define REG_VOLUME_MASTER	0x07
#define REG_VOLUME_CHAN1	0x08
#define REG_VOLUME_CHAN2	0x09
#define REG_VOLUME_HP	0x0A
#define REG_VOLUME_CONFIG	0x0E
#define REG_MODULATION_LIMIT	0x10
#define REG_INTERCHANNEL1_DELAY	0x011
#define REG_INTERCHANNEL2_DELAY	0x012
#define REG_INTERCHANNEL3_DELAY	0x013
#define REG_INTERCHANNEL4_DELAY	0x014
#define REG_PWM_SHUTDOWN	0x19
#define REG_START_PERIOD	0x1A
#define REG_OSCILLATOR_TRIM	0X1B
#define REG_BKND_ERR	0x1C
#define REG_INPUT_MULTIPLEXER	0x20
#define REG_CHANNEL_SELECT		0x21
#define REG_PWM_MUX	0x25
#define REG_DRC_CONTROL	0x46
#define REG_BANK_AND_EQ	0x50


/****************** i2c bus ****************************************/
#define GPIO_I2C_SDA 32*4+30
#define GPIO_I2C_SCL 32*4+31

#define USE_OLD_I2C 1

#if USE_OLD_I2C
struct gp_i2c_handle
{
	unsigned int slaveAddr;			/*!< @brief slave device address*/
	unsigned int clkRate;				/*!< @brief i2c bus clock rate */
};

static struct gp_i2c_handle  *g_i2c_handle = NULL;
static spinlock_t glock;


static void waste(void)
{
	int delaytime=3;
	
	if((500000/(g_i2c_handle->clkRate))<5)
		delaytime=5;
	else
		delaytime=500000/(g_i2c_handle->clkRate);
	udelay(delaytime);
	return;
}

static void I2cSdIOModech_GPIO(int inputEnable)
{
	//gp_gpio_set_value(gp_i2c.handlesd,0);
	__gpio_clear_pin(GPIO_I2C_SDA);
	if(inputEnable)
		{
			//gp_gpio_set_input(gp_i2c.handlesd,1);
			__gpio_as_input(GPIO_I2C_SDA);
		}
	else
		{
			//gp_gpio_set_output(gp_i2c.handlesd,0,20);
			__gpio_as_output(GPIO_I2C_SDA);
		}
}
static unsigned char I2cSdIOStatus_GPIO(void)
{
	unsigned int value=0;
	//gp_gpio_get_value(gp_i2c.handlesd,&value);
	value=__gpio_get_pin(GPIO_I2C_SDA);
	return (unsigned char)value;
}

static void setI2cSck_GPIO(int value)
{
	//gp_gpio_set_value(gp_i2c.handlesck,value);
	if(value)
		__gpio_set_pin(GPIO_I2C_SCL);
	else
		__gpio_clear_pin(GPIO_I2C_SCL);
}
static void setI2cSd_GPIO(int value)
{
	//gp_gpio_set_value(gp_i2c.handlesd,value);
	if(value)
		__gpio_set_pin(GPIO_I2C_SDA);
	else
		__gpio_clear_pin(GPIO_I2C_SDA);
}

static void i2c_init(void)
{
	//setI2cSck_GPIO(0);
	__gpio_as_output(GPIO_I2C_SDA);
	__gpio_as_output(GPIO_I2C_SCL);

}
static void i2c_start(void)
{
	i2c_init();

	setI2cSck_GPIO(1);
	setI2cSd_GPIO(1);
	waste();
	setI2cSck_GPIO(1);
	waste();
	setI2cSd_GPIO(0);
	waste();
	setI2cSck_GPIO(0);
}

static unsigned char i2c_sendbyte(int data)
{
	int i = 0;
	unsigned char ret=0;
	for(i=0;i<8;i++)
		{
			setI2cSck_GPIO(0);
			if((data<<i)&0x80)
				setI2cSd_GPIO(1);
			else
				setI2cSd_GPIO(0);
			waste();
			setI2cSck_GPIO(1);
			waste();
			setI2cSck_GPIO(0);
		}
	waste();
	I2cSdIOModech_GPIO(1);
	waste();
	setI2cSck_GPIO(1);
	waste();
	ret=I2cSdIOStatus_GPIO();
	setI2cSck_GPIO(0);
	I2cSdIOModech_GPIO(0);
	return ret;
}

static void i2c_revbyte(unsigned char* data)

{
	int i;
	*data=0;
	setI2cSck_GPIO(0);
	waste();
	I2cSdIOModech_GPIO(1);//input
	for(i=0;i<8;i++){
		waste();
		setI2cSck_GPIO(0);
		waste();
		setI2cSck_GPIO(1);
		waste();
		*data<<=1;
		if(I2cSdIOStatus_GPIO())
			*data+=1;
	}
	setI2cSck_GPIO(0);
	I2cSdIOModech_GPIO(0);//output
	return ;
}

static void i2c_mack(unsigned char aCk)
{
	setI2cSck_GPIO(0);
	if(aCk)
		setI2cSd_GPIO(0);
	else
		setI2cSd_GPIO(1);
	waste();
	setI2cSck_GPIO(1);
	waste();
	setI2cSck_GPIO(0);
}

static void i2c_stop(void)
{
	setI2cSck_GPIO(0);
	waste();
	setI2cSd_GPIO(0);
	waste();
	setI2cSck_GPIO(1);
	waste();
	setI2cSd_GPIO(1);
	//waste();//lifengxiao
	//setI2cSck_GPIO(0);//lifengxiao
}

static int i2c_request(int slaveAddr, int clkRate)
{
	struct gp_i2c_handle *i2c_handle = NULL;
	/*init i2c bus register*/
	i2c_handle = (struct gp_i2c_handle *)kmalloc(sizeof(struct gp_i2c_handle), GFP_KERNEL);
	if(!i2c_handle){
		goto __errKmalloc;
	}
	//memset(i2c_handle, 0, sizeof(gp_i2c_handle));

	i2c_handle -> slaveAddr = slaveAddr;
	i2c_handle -> clkRate = clkRate;

	return ((int)i2c_handle);
	__errKmalloc:
		kfree(i2c_handle);
		i2c_handle = NULL;
	return -ENOMEM;
}

static int sky_i2c_write_reg(int handle, unsigned char reg, unsigned char* data, unsigned int len)
{
	int i,ret =-1;
	int retry = 0;
	struct gp_i2c_handle  *i2c_handle = NULL;
	if(!data || len <= 0){
		printk(KERN_ALERT "err argument: sky_i2c_write\n");
		return ret;
	}
	i2c_handle = (struct gp_i2c_handle *)handle;
	g_i2c_handle = (struct gp_i2c_handle *)handle;
//	printk("enter sky_i2c_write_reg reg 0x%02x, len %d\n", reg, len);
	spin_lock_irq(&glock);

retry_start:
	//start
	i2c_start();
	//send aid
	waste();
	ret=i2c_sendbyte(i2c_handle->slaveAddr);
	if(ret)
		{
			i2c_stop();
			printk("sky_i2c_write_reg send device addr 0x%02x err\n", i2c_handle->slaveAddr);
			retry ++;
			if (retry < 3)
				goto retry_start;
			else
				goto errexit;

		}
	//send addr&adata
	waste();
	ret=i2c_sendbyte(reg);
	if(ret)
		{
			printk("sky_i2c_write_reg send reg addr 0x%02x err\n", reg);
				goto errexit;
		}
	//send addr&adata
	waste();
	for(i=0; i<len; i++){
			ret=i2c_sendbyte(data[i] & 0xff);
			if(ret)
				{
					printk("write without receive ack err\n");
					goto errexit;
				}
			waste();
		}
	//stop
	i2c_stop();

	errexit:
		spin_unlock_irq(&glock);
	return ret;
}

static int sky_i2c_read_reg (int handle, unsigned char reg, unsigned char* data, unsigned int len)
{
	int i,ret = -1;
	int retry = 0;
	struct gp_i2c_handle *i2c_handle = (struct gp_i2c_handle *)handle;
	g_i2c_handle = (struct gp_i2c_handle *)handle;
	if(!data || len <= 0){
		printk("err argument: sky_i2c_read\n");
		return ret;
	}
//	printk("enter sky_i2c_read_reg reg 0x%02x, len %d\n", reg, len);
	spin_lock_irq(&glock);
retry_start:
	//start
	i2c_start();
	//send read addr
	waste();
	ret=i2c_sendbyte(i2c_handle->slaveAddr);
	if(ret)
		{
			i2c_stop();
			retry ++;
			printk("i2c read send device addr 0x%02x err\n", i2c_handle->slaveAddr);
			if (retry < 3)
				goto retry_start;
			else
				goto errexit;
		}
	//read
	waste();
	ret=i2c_sendbyte(reg);
	if(ret)
		{
			printk("i2c read send reg addr 0x%02x err\n", reg);
				goto errexit;
		}
	//read
	waste();
	i2c_start();
	ret=i2c_sendbyte(i2c_handle->slaveAddr|0x01);
	if(ret)
		{
			printk("i2c read resend device addr 0x%02x err \n", i2c_handle->slaveAddr|01);
				goto errexit;
		}
	//read
	waste();	
	for(i=0; i<len-1; i++){
		i2c_revbyte(&data[i]);
		waste();
		//master ack
		i2c_mack(1);
		waste();
	}
	i2c_revbyte(&data[i]);
	waste();
	//master nack
	i2c_mack(0);
	waste();
	//stop
	i2c_stop();
	
	errexit:
	spin_unlock_irq(&glock);

	return ret;
}

#else

/****  I2C bus interface function ****/
extern void i2c_jz_setclk(struct i2c_client *client,unsigned long i2cclk);
static struct i2c_client *tas5713_client = NULL;

static int sky_i2c_write_reg(unsigned char reg, unsigned char* data, unsigned int len)
{
	int i,ret =-1;
	int retry = 0;
	unsigned char buf[10] = {0};
        struct i2c_client *client = tas5713_client;

	if(!data || len <= 0){
		printk("err argument: sky_i2c_write_reg\n");
		return ret;
	}
	
	buf[0] = reg;
	for(i=0; i < len;i++){
		buf[i+1] = *data;
		data++;
	}

//	printk("enter sky_i2c_write_reg reg 0x%02x, len %d\n", reg, len);
	ret = i2c_master_send(client, buf, len+1);
        if (ret < 0) 
	        printk("sky_i2c_write_reg 0x%02x err\n", reg);
	return ret;
}

static int sky_i2c_read_reg (unsigned char reg, unsigned char* data, unsigned int len)
{
	int i,ret = -1;
	int retry = 0;
        struct i2c_client *client = tas5713_client;
	
	if(!data || len <= 0){
		printk("err argument: sky_i2c_read_reg\n");
		return ret;
	}
//	printk("enter sky_i2c_read_reg reg 0x%02x, len %d\n", reg, len);
	ret = i2c_master_send(client, &reg, 1);
        if (ret < 0) 
	{
		printk("sky_i2c_read_reg send reg addr 0x%02x err\n", reg);
		return ret;
	}

	ret = i2c_master_recv(client, data, len);
        if (ret < 0) 
	{
		printk("sky_i2c_read_reg 0x%02x err\n", reg);
		return ret;
	}
	
	return ret;
}

static void tas5713_reset(void);
static int __devinit tas5713_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err;
	char buf[2]= {0};

	printk("\n%s %d>>>>>  ok\n\n",__FUNCTION__,__LINE__);
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		return err;
	}
	
	tas5713_client = client;
	//printk("tas5713_client address %d\n",tas5713_client->addr);
	//i2c_jz_setclk(client, 100*1000);
	
	buf[0] = 0x0e;
	buf[1] = 0x90;
	
        err = i2c_master_send(client, buf, 2);
        if (err < 0) {
                dev_err(&client->dev, "send one byte failed !\n");
                return err;
        }

	return 0;
}
static int __devexit tas5713_i2c_remove(struct i2c_client *client)
{
	tas5713_client = NULL;
        return 0;
}

static const struct i2c_device_id tas5713_id[] = {
        { "tas5713", 0 },
        { }
};      
MODULE_DEVICE_TABLE(i2c, tas5713_id);

static struct i2c_driver tas5713_i2c_driver = {
        .driver.name = "tas5713",
        .probe       = tas5713_i2c_probe,
        .remove      = __devexit_p(tas5713_i2c_remove),
        .id_table    = tas5713_id,
};

/************ end I2C controller ***********/
#endif


/****************** tas5713 api ****************************************/

void tas5713_read_reg(unsigned char reg, unsigned char len)
{
	unsigned char buf[32] = {0};
	int i = 0;
#if USE_OLD_I2C
	sky_i2c_read_reg(tas5713_data.i2c_handle, reg, buf, len);
#else
	sky_i2c_read_reg(reg, buf, len);
#endif
	printk("{0x%02x, ", reg);
	for (i = 0; i< len; i++) printk("0x%02x, ", buf[i]);
	printk("},\n\n");
}

static void tas5713_reset(void)
{
	__gpio_as_output(GPIO_TAS5713_RESET);
	__gpio_as_output(GPIO_TAS5713_PDN);

//       __gpio_as_output(GPIO_TAS5713_INPUT_SELECT);
//       __gpio_clear_pin(GPIO_TAS5713_INPUT_SELECT);	 //select cpu's i2s output
       
	__gpio_set_pin(GPIO_TAS5713_PDN);

	__gpio_set_pin(GPIO_TAS5713_RESET);
	mdelay(10);
	__gpio_clear_pin(GPIO_TAS5713_RESET);
	mdelay(10);
	__gpio_set_pin(GPIO_TAS5713_RESET);
	mdelay(20);	

	return ;
}

static int tas5713_check_err_status(void)
{
	unsigned char value = 0;

#if USE_OLD_I2C
	sky_i2c_read_reg(tas5713_data.i2c_handle, REG_ERROR_STATUS, &value, 1);
#else
	sky_i2c_read_reg(REG_ERROR_STATUS, &value, 1);
#endif

	if (value) {
		printk("tas5713_check_err_status 0x%02x\n", value);
		value = 0;
#if USE_OLD_I2C
		sky_i2c_write_reg(tas5713_data.i2c_handle, REG_ERROR_STATUS, &value, 1); // clear the status
#else
		sky_i2c_write_reg(REG_ERROR_STATUS, &value, 1); // clear the status
#endif
		return -1;
	}

	return 0;
}

static void tas5713_shutdown(void)
{
//	unsigned char value = 0x40;

	printk("tas5713_shutdown\n");

//	sky_i2c_write_reg(tas5713_data.i2c_handle, REG_SYSTEM_CONTROL2, &value,1);

	return ;
}

static void tas5713_wakeup(void)
{
//	unsigned char value = 0x00;

	printk("tas5713_wakeup\n");

//	sky_i2c_write_reg(tas5713_data.i2c_handle, REG_SYSTEM_CONTROL2, &value,1);

	return ;
}


static int tas5713_set_volume(unsigned char vol)
{
	int ret;
	printk("\n tas5713_set_volume vol=%d\n", vol);
#if USE_OLD_I2C
	sky_i2c_write_reg(tas5713_data.i2c_handle, REG_VOLUME_MASTER, &vol, 1);
#else
	sky_i2c_write_reg(REG_VOLUME_MASTER, &vol, 1);
#endif

	tas5713_check_err_status();

	return 0;
}


static int tas5713_set_mute(int mute)
{
	unsigned char value = 0x00;
	
	if (mute) 
		value = 0x07;
	else  
		value = 0x00;
#if USE_OLD_I2C
	sky_i2c_write_reg(tas5713_data.i2c_handle, REG_SOFT_MUTE, &value,1);
#else
	sky_i2c_write_reg(REG_SOFT_MUTE, &value,1);
#endif
	
	return 0;
}
/*
int tas5713_input_select(int input)
{
	printk(KERN_DEBUG "tas5713_input_select %s !\n", input?"adc line in":"local play");
	__gpio_as_output(GPIO_TAS5713_INPUT_SELECT);
	
	if (!input) {
		tas5713_dac_flag = 0;
		__gpio_clear_pin(GPIO_TAS5713_INPUT_SELECT);  //select cpu  i2s output
		tas5713_set_mute(1); // mute the tas5713 output if tvout
		
	} else {
		tas5713_dac_flag = 1;
		__gpio_set_pin(GPIO_TAS5713_INPUT_SELECT);  //select pcm1808 adc  i2s output
		tas5713_set_mute(0); // unmute
	}
	return 0;
}
*/


static unsigned char init_regs1[][2] = {
{0x1B, 0x00},
//{0x04, 0x03},	// ... 16bit
{0x03, 0x80},
{0x11, 0xAC},
{0x12, 0x54},
{0x13, 0xAC},
{0x14, 0x54},
{0x1C, 0x07},
{0x0A, 0x30},
{0x0E, 0xF1},
{0x00, 0x6C},
{0x05, 0x00},
{0x07, 0x60},
{0x08, 0x30},
{0x09, 0x30},
};

static unsigned char init_regs4[][5] = {
{0x50, 0x00, 0x00, 0x00, 0x10},
{0x25, 0x01, 0x02, 0x13, 0x45},
{0x70, 0x00, 0x80, 0x00, 0x00},	
{0x71, 0x00, 0x00, 0x00, 0x00},
{0x74, 0x00, 0x80, 0x00, 0x00},
{0x75, 0x00, 0x00, 0x00, 0x00},
{0x46, 0x00, 0x02, 0x00, 0x20},
{0x20, 0x00, 0x01, 0x77, 0x72}, 
{0x73, 0x00, 0x80, 0x00, 0x00}, 
{0x72, 0x00, 0x00, 0x00, 0x00},
{0x77, 0x00, 0x80, 0x00, 0x00}, 
{0x76, 0x00, 0x00, 0x00, 0x00},
{0x56, 0x00, 0x40, 0x00, 0x00},
{0x57, 0x00, 0x02, 0x00, 0x00},
{0x50, 0x00, 0x00, 0x00, 0x10},
};

static unsigned char init_regs8[][9] = {
{0x40, 0x09, 0x0A, 0x00, 0x00, 0x09, 0x09, 0xFF, 0xFF},
{0x3B, 0x00, 0x20, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00},	
{0x3c, 0x00, 0x00, 0x10, 0x00, 0xff, 0xff, 0xff, 0xfd},
{0x43, 0x04, 0xf0, 0x00, 0x00, 0x04, 0xef, 0xff, 0xff},
{0x3e, 0x00, 0x20, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00},
{0x3f, 0x00, 0x08, 0x00, 0x00, 0xff, 0xf8, 0x00, 0x00},
{0x51, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0x52, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
};

static unsigned char init_regs20[][21] = {
{0x29, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0x2a, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0x2b, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0x2c, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0x2d, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0x2e, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0x2f, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0x58, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0x59, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0x5a, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
{0x5b, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
};


static void tas5713_init_reg(void)
{
//	unsigned char value = 0;
	int i = 0;

	int size = sizeof(init_regs1)/sizeof(init_regs1[0]);
	for (i=0; i< size; i++) {
#if USE_OLD_I2C
		sky_i2c_write_reg(tas5713_data.i2c_handle, init_regs1[i][0], &init_regs1[i][1], 1);
#else
		sky_i2c_write_reg(init_regs1[i][0], &init_regs1[i][1], 1);
#endif
	}

	size = sizeof(init_regs4)/sizeof(init_regs4[0]);
	for (i=0; i< size; i++) {
#if USE_OLD_I2C
		sky_i2c_write_reg(tas5713_data.i2c_handle, init_regs4[i][0], &init_regs4[i][1], 4);
#else
		sky_i2c_write_reg(init_regs4[i][0], &init_regs4[i][1], 4);
#endif
	}

	size = sizeof(init_regs8)/sizeof(init_regs8[0]);
	for (i=0; i< size; i++) {
#if USE_OLD_I2C
		sky_i2c_write_reg(tas5713_data.i2c_handle, init_regs8[i][0], &init_regs8[i][1], 8);
#else
		sky_i2c_write_reg(init_regs8[i][0], &init_regs8[i][1], 8);
#endif
	}

	size = sizeof(init_regs20)/sizeof(init_regs20[0]);
	for (i=0; i< size; i++) {
#if USE_OLD_I2C
		sky_i2c_write_reg(tas5713_data.i2c_handle, init_regs20[i][0], &init_regs20[i][1], 8);
#else
		sky_i2c_write_reg(init_regs20[i][0], &init_regs20[i][1], 8);
#endif
	}
}

void tas5713_dump_all(void)
{
//	unsigned int i = 0;
//	unsigned char tmp[32]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

#if 0
	sky_i2c_read_reg(tas5713_data.i2c_handle, 0, tmp,32);

	for(i=0;i<32;i++) {
		printk("reg = 0x%02x,value = 0x%02x\n",i,tmp[i]);
	}

#endif

#if 1

	tas5713_read_reg(0x50, 4);
	tas5713_read_reg(0x25, 4);
	tas5713_read_reg(0x70, 4);
	tas5713_read_reg(0x71, 4);

	tas5713_read_reg(0x74, 4);
	tas5713_read_reg(0x75, 4);
	tas5713_read_reg(0x46, 4);
	tas5713_read_reg(0x20, 4);

	tas5713_read_reg(0x73, 4);
	tas5713_read_reg(0x72, 4);
	tas5713_read_reg(0x77, 4);
	tas5713_read_reg(0x76, 4);

	tas5713_read_reg(0x56, 4);
	tas5713_read_reg(0x57, 4);


	tas5713_read_reg(0x29, 20);
	tas5713_read_reg(0x2A, 20);
	tas5713_read_reg(0x2B, 20);
	tas5713_read_reg(0x2C, 20);
	tas5713_read_reg(0x2D, 20);
	tas5713_read_reg(0x2E, 20);
	tas5713_read_reg(0x2F, 20);
#endif
#if 1

	tas5713_read_reg(0x40, 8);
	tas5713_read_reg(0x3B, 8);
	tas5713_read_reg(0x3C, 8);
	tas5713_read_reg(0x43, 8);
	tas5713_read_reg(0x3E, 8);
	tas5713_read_reg(0x3F, 8);
	tas5713_read_reg(0x51, 8);
	tas5713_read_reg(0x52, 8);

	tas5713_read_reg(0x58, 20);
	tas5713_read_reg(0x59, 20);
	tas5713_read_reg(0x5A, 20);
	tas5713_read_reg(0x5B, 20);
#endif

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

static int dlv_mute(int val);

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

/**
 * CODEC read register
 *
 * addr:	address of register
 * return:	value of register
 */
static inline int dlv_read_reg(int addr)
{
	volatile int reg;
	while (__icdc_rgwr_ready()) {
		;//nothing...
	}
	__icdc_set_addr(addr);
	reg = __icdc_get_value();
	reg = __icdc_get_value();
	reg = __icdc_get_value();
	reg = __icdc_get_value();
	reg = __icdc_get_value();
	return __icdc_get_value();
}

/**
 * CODEC write register
 *
 * addr:	address of register
 * val:		value to set
 */
void dlv_write_reg(int addr, int val)
{
	volatile int reg;
	while (__icdc_rgwr_ready()) {
		;//nothing...
	}
	REG_ICDC_RGADW = ((addr << ICDC_RGADW_RGADDR_LSB) | val);
	__icdc_set_rgwr();
	reg = __icdc_rgwr_ready();
	reg = __icdc_rgwr_ready();
	reg = __icdc_rgwr_ready();
	reg = __icdc_rgwr_ready();
	reg = __icdc_rgwr_ready();
	reg = __icdc_rgwr_ready();
	while (__icdc_rgwr_ready()) {
		;//nothing...
	}
}

/**
 * CODEC write a bit of a register
 *
 * addr:	address of register
 * bitval:	bit value to modifiy
 * mask_bit:	indicate which bit will be modifiy
 */
static int dlv_write_reg_bit(int addr, int bitval, int mask_bit)
{
	int val = dlv_read_reg(addr);

	if (bitval)
		val |= (1 << mask_bit);
	else
		val &= ~(1 << mask_bit);
	dlv_write_reg(addr, val);

	return 1;
}

static inline void dlv_sleep_wait_bitset(int reg, unsigned bit, int stime, int line)
{
	int count = 0;
	while(!(dlv_read_reg(reg) & (1 << bit))) {
		//printk("DLV waiting reg(%2x) bit(%2x) set %d \n",reg, bit, line);
		dlv_sleep(stime);
		count++;
		if(count > 10){
			printk("%s %d timeout\n",__FILE__,__LINE__);
			break;
		}
	}
}

static inline void dlv_sleep_wait_bitclear(int reg, unsigned bit, int stime)
{
	int count = 0;

	while((dlv_read_reg(reg) & (1 << bit)))
	{
	      dlv_sleep(stime);
		  count++;
		  if(count > 10){
			  printk("%s %d timeout\n",__FILE__,__LINE__);
			  break;
		  }
	}
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
		CODEC_DEBUG_ROUTINE,		CODEC_SET_STANDBY
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
		"CODEC_DEBUG_ROUTINE",		"CODEC_SET_STANDBY"
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

#if 0
void dump_dlv_regs(void)
{
	unsigned int i;
	unsigned char data;
	for (i = 0; i < 32; i++) {
		data = dlv_read_reg(i);
		printk("address = 0x%02x, data = 0x%02x\n", i, data);
	}
}

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
 *route part and attibute                                                              *
 *                                                                                     *
\***************************************************************************************/
/*=========================power on==========================*/
static void dlv_set_route_ready(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");


	DUMP_ROUTE_PART_REGS("leave");
}

/*=================route part functions======================*/

static void dlv_set_mic1(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case MIC1_DIFF_WITH_MICBIAS:

		break;

	case MIC1_DIFF_WITHOUT_MICBIAS:

		break;

	case MIC1_SING_WITH_MICBIAS:

		break;

	case MIC1_SING_WITHOUT_MICBIAS:

		break;

	case MIC1_DISABLE:
	
		break;

	default:
		printk("TAS5713_DLV: line: %d, mic1 mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_mic2(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case MIC2_DIFF_WITH_MICBIAS:
		
		break;

	case MIC2_DIFF_WITHOUT_MICBIAS:
	
		break;

	case MIC2_SING_WITH_MICBIAS:
	
		break;

	case MIC2_SING_WITHOUT_MICBIAS:
	
		break;

	case MIC2_DISABLE:
	
		break;

	default:
		printk("TAS5713_DLV: line: %d, mic2 mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_linein(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){
		
	case LINEIN_WITHOUT_BYPASS:

		break;

	case LINEIN_WITH_BYPASS:
		
		break;

	case LINEIN_DISABLE:
		
		break;
		
	default:
		printk("TAS5713_DLV: line: %d, linein mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_agc(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case AGC_ENABLE:

		break;
		
	case AGC_DISABLE:

		break;

	default:
		printk("TAS5713_DLV: line: %d, agc mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_record_mux(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case RECORD_MUX_MIC1_TO_LR:

		break;

	case RECORD_MUX_MIC2_TO_LR:
		break;

	case RECORD_MUX_MIC1_TO_R_MIC2_TO_L:

		break;

	case RECORD_MUX_MIC2_TO_R_MIC1_TO_L:

		break;

	case RECORD_MUX_LINE_IN:
		break;

	default:
		printk("TAS5713_DLV: line: %d, record mux mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_adc(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case ADC_STEREO:

		break;

	case ADC_MONO:
		break;

	case ADC_DISABLE:

		break;

	default:
		printk("TAS5713_DLV: line: %d, adc mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_record_mixer(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case RECORD_MIXER_MIX1_INPUT_ONLY:
		break;

	case RECORD_MIXER_MIX1_INPUT_AND_DAC:
		break;

	default:
		printk("TAS5713_DLV: line: %d, record mixer mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_replay_mixer(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case REPLAY_MIXER_PLAYBACK_DAC_ONLY:
		break;

	case REPLAY_MIXER_PLAYBACK_DAC_AND_ADC:
		break;

	default:
		printk("TAS5713_DLV: line: %d, replay mixer mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_dac(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case DAC_STEREO:
		break;

	case DAC_MONO:
		break;

	case DAC_DISABLE:
		break;

	default:
		printk("TAS5713_DLV: line: %d, dac mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_replay_filter(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case REPLAY_FILTER_STEREO:
		break;

	case REPLAY_FILTER_MONO:
		break;

	default:
		printk("TAS5713_DLV: line: %d, replay filter mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_replay_mux(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case REPLAY_MUX_MIC1_TO_LR:
		break;

	case REPLAY_MUX_MIC2_TO_LR:
		break;

	case REPLAY_MUX_MIC1_TO_R_MIC2_TO_L:
		break;

	case REPLAY_MUX_MIC2_TO_R_MIC1_TO_L:
		break;

	case REPLAY_MUX_BYPASS_PATH:
		break;

	case REPLAY_MUX_DAC_OUTPUT:
		break;

	default:
		printk("TAS5713_DLV: line: %d, replay mux mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_hp(int mode)
{
//	int load_flag = 0;

	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case HP_ENABLE:

		break;

	case HP_DISABLE:

		break;

	default:
		printk("TAS5713_DLV: line: %d, hp mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_lineout(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case LINEOUT_STEREO:
		break;

	case LINEOUT_MONO:
		break;

	case LINEOUT_DISABLE:
		break;

	default:
		printk("TAS5713_DLV: line: %d, lineout mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

static void dlv_set_btl(int mode)
{
	DUMP_ROUTE_PART_REGS("enter");

	switch(mode){

	case BTL_ENABLE:
		break;

	case BTL_DISABLE:
		break;

	default:
		printk("TAS5713_DLV: line: %d, btl mode error!\n", __LINE__);
	}

	DUMP_ROUTE_PART_REGS("leave");
}

/*=================route attibute(gain) functions======================*/

static void dlv_set_gain_mic1(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_mic2(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_linein_left(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_linein_right(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");


	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_adc_left(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_adc_right(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");


	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_record_mixer(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");


	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_replay_mixer(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_dac_left(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");


	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_dac_right(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_hp_left(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");

	DUMP_GAIN_PART_REGS("leave");
}
static void dlv_set_gain_hp_right(int gain)
{
//	int val;

	DUMP_GAIN_PART_REGS("enter");


	DUMP_GAIN_PART_REGS("leave");
}
/***************************************************************************************\
 *                                                                                     *
 *dlv route                                                                            *
 *                                                                                     *
\***************************************************************************************/

static void dlv_set_route_base(const void *arg)
{
	route_conf_base *conf = (route_conf_base *)arg;

	/*codec turn on sb and sb_sleep*/
	if (conf->route_ready_mode)
		dlv_set_route_ready(conf->route_ready_mode);

	/*--------------route---------------*/
	/* record path */
	if (conf->route_mic1_mode)
		dlv_set_mic1(conf->route_mic1_mode);

	if (conf->route_mic2_mode)
		dlv_set_mic2(conf->route_mic2_mode);

	if (conf->route_linein_mode)
		dlv_set_linein(conf->route_linein_mode);

	if (conf->route_record_mux_mode)
		dlv_set_record_mux(conf->route_record_mux_mode);

	if (conf->route_adc_mode)
		dlv_set_adc(conf->route_adc_mode);

	if (conf->route_record_mixer_mode)
		dlv_set_record_mixer(conf->route_record_mixer_mode);

	/* replay path */
	if (conf->route_replay_mixer_mode)
		dlv_set_replay_mixer(conf->route_replay_mixer_mode);

	if (conf->route_replay_filter_mode)
		dlv_set_replay_filter(conf->route_replay_filter_mode);

	if (conf->route_dac_mode)
		dlv_set_dac(conf->route_dac_mode);

	if (conf->route_replay_mux_mode)
		dlv_set_replay_mux(conf->route_replay_mux_mode);

	if (conf->route_hp_mode)
		dlv_set_hp(conf->route_hp_mode);

	if (conf->route_lineout_mode)
		dlv_set_lineout(conf->route_lineout_mode);

	if (conf->route_btl_mode)
		dlv_set_btl(conf->route_btl_mode);

	/*----------------attibute-------------*/
	/* auto gain */
	if (conf->attibute_agc_mode)
		dlv_set_agc(conf->attibute_agc_mode);

	/* gain , use 32 instead of 0 */
	if (conf->attibute_mic1_gain) {
		if (conf->attibute_mic1_gain == 32)
			dlv_set_gain_mic1(0);
		else 
			dlv_set_gain_mic1(conf->attibute_mic1_gain);
	}

	if (conf->attibute_mic2_gain) {
		if (conf->attibute_mic2_gain == 32)
			dlv_set_gain_mic2(0);
		else
			dlv_set_gain_mic2(conf->attibute_mic2_gain);
	}

	if (conf->attibute_linein_l_gain) {
		if (conf->attibute_linein_l_gain == 32)
			dlv_set_gain_linein_left(0);
		else
			dlv_set_gain_linein_left(conf->attibute_linein_l_gain);
	}

	if (conf->attibute_linein_r_gain) {
		if (conf->attibute_linein_r_gain == 32)
			dlv_set_gain_linein_right(0);
		else 
			dlv_set_gain_linein_right(conf->attibute_linein_r_gain);
	}

	if (conf->attibute_adc_l_gain) {
		if (conf->attibute_adc_l_gain == 32)
			dlv_set_gain_adc_left(0);
		else
			dlv_set_gain_adc_left(conf->attibute_adc_l_gain);
	}

	if (conf->attibute_adc_r_gain) {
		if (conf->attibute_adc_r_gain == 32)
			dlv_set_gain_adc_right(0);
		else 
			dlv_set_gain_adc_right(conf->attibute_adc_r_gain);
	}

	if (conf->attibute_record_mixer_gain) {
		if (conf->attibute_record_mixer_gain == 32)
			dlv_set_gain_record_mixer(0);
		else 
			dlv_set_gain_record_mixer(conf->attibute_record_mixer_gain);
	}

	if (conf->attibute_replay_mixer_gain) {
		if (conf->attibute_replay_mixer_gain == 32)
			dlv_set_gain_replay_mixer(0);
		else 
			dlv_set_gain_replay_mixer(conf->attibute_replay_mixer_gain);
	}

	if (conf->attibute_dac_l_gain) {
		if (conf->attibute_dac_l_gain == 32)
			dlv_set_gain_dac_left(0);
		else
			dlv_set_gain_dac_left(conf->attibute_dac_l_gain);
	}
	
	if (conf->attibute_dac_r_gain) {
		if (conf->attibute_dac_r_gain == 32)
			dlv_set_gain_dac_right(0);
		else
			dlv_set_gain_dac_right(conf->attibute_dac_r_gain);
	}

	if (conf->attibute_hp_l_gain) {
		if (conf->attibute_hp_l_gain == 32)
			dlv_set_gain_hp_left(0);
		else
			dlv_set_gain_hp_left(conf->attibute_hp_l_gain);
	}

	if (conf->attibute_hp_r_gain) {
		if (conf->attibute_hp_r_gain == 32)
			dlv_set_gain_hp_right(0);
		else
			dlv_set_gain_hp_right(conf->attibute_hp_r_gain);
	}
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
	printk("TAS5713_DLV: waring, %s() is a default function\n", __func__);

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
	printk("TAS5713_DLV: waring, %s() is a default function\n", __func__);

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
	printk("TAS5713_DLV: waring, %s() is a default function\n", __func__);

	__dlv_set_int_form(ICR_INT_HIGH_8CYCLES);

	__dlv_set_irq_mask(ICR_COMMON_MASK);
	__dlv_set_irq_flag(0x3f);

	__dlv_set_12m_crystal();

	return 0;
}

static int dlv_init(void)
{
	int ret;

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

	printk("TAS5713_DLV: waring, %s() is a default function\n", __func__);

	if ((mode & REPLAY) && (mode & RECORD)) {
		printk("TAS5713 DLV: Close REPLAY & RECORD\n");
		tas5713_shutdown();
		/* set 16ohm load to keep away from the bug can not waited RDO */
		if(__dlv_get_load() == LOAD_10KOHM)
			__dlv_set_16ohm_load();

		ret = dlv_set_route(ROUTE_ALL_CLEAR);
		if(ret != ROUTE_ALL_CLEAR)
		{
			printk("TAS5713 CODEC: dlv_turn_off_part replay & record mode error!\n");
			return -1;
		}
	} else if (mode & REPLAY) {
		printk("TAS5713 DLV: Close REPLAY\n");
		tas5713_shutdown();
		/* set 16ohm load to keep away from the bug can not waited RDO */
/*
		if(__dlv_get_load() == LOAD_10KOHM)
			__dlv_set_16ohm_load();

		ret = dlv_set_route(ROUTE_REPLAY_CLEAR);
		if(ret != ROUTE_REPLAY_CLEAR)
		{
			printk("TAS5713 CODEC: dlv_turn_off_part replay mode error!\n");
			return -1;
		}
*/
	} else if (mode & RECORD) {
		printk("TAS5713 DLV: Close RECORD\n");
		tas5713_shutdown();
		ret = dlv_set_route(ROUTE_RECORD_CLEAR);
		if(ret != ROUTE_RECORD_CLEAR)
		{
			printk("TAS5713 CODEC: dlv_turn_off_part record mode error!\n");
			return -1;
		}
		
		ret = dlv_set_route(route);
		if(ret != route)
		{
			printk("TAS5713 CODEC: %s record mode error!\n", __func__);
			return -1;
		}
	}

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
	printk("TAS5713_DLV: waring, %s() is a default function\n", __func__);

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
	printk("TAS5713_DLV: waring, %s() is a default function\n", __func__);

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

/******** dlv_anti_pop ********/
/**
 * CODEC dlv anti pop part
 *
 * it will be used as default, it can be recode 
 * depend on difficent boards if necessary
 *
 **/
static int dlv_anti_pop_part(void)
{
//	printk("TAS5713_DLV: waring, %s() is a default function\n", __func__);

	return 0;
}

static int dlv_anti_pop(int mode)
{
	int ret = 0;

	switch(mode) {
	case CODEC_WRMODE:
		break;
	case CODEC_RMODE:
		break;
	case CODEC_WMODE:
		if(dlv_platform_data->dlv_anti_pop_part)
		{
			ret = dlv_platform_data->dlv_anti_pop_part();
			if(ret)
			{
				ret = dlv_anti_pop_part();
			}
		} else
			ret = dlv_anti_pop_part();
		break;
	}

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

	printk("TAS5713_DLV: waring, %s() is a default function\n", __func__);

	ret = dlv_set_route(ROUTE_ALL_CLEAR);
	if(ret != ROUTE_ALL_CLEAR)
	{
		printk("TAS5713 CODEC: dlv_suspend_part error!\n");
		return -1;
	}

	return 0;
}

static int dlv_suspend(void)
{
	int ret;
	g_dlv_sleep_mode = 0;
	
	/* set 16ohm load to keep away from the bug can not waited RDO */
	if(__dlv_get_load() == LOAD_10KOHM)
		__dlv_set_16ohm_load();

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

	printk("TAS5713_DLV: waring, %s() is a default function\n", __func__);

	/*default, the resume will restore the route before suspend*/
	ret = dlv_set_route(route);

	if(ret != route)
	{
		printk("TAS5713 CODEC: dlv_resume_part error!\n");
		return -1;
	}

	return 0;
}

static int dlv_resume(void)
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

	printk("TAS5713_DLV: waring, %s() is a default function\n", __func__);

	switch (snd_dev_cfg->device) {

	case SND_DEVICE_HEADSET:
		ret = dlv_set_route(REPLAY_HP_STEREO);
		if(ret != REPLAY_HP_STEREO)
		{
			printk("TAS5713 CODEC: set device SND_DEVICE_HEADSET error!\n");
			return -1;
		}
		break;

	case SND_DEVICE_HANDSET:
		ret = dlv_set_route(REPLAY_LINEOUT_MONO);
		if(ret != REPLAY_LINEOUT_MONO)
		{
			printk("TAS5713 CODEC: set device SND_DEVICE_HANDSET error!\n");
			return -1;
		}
		break;

	case SND_DEVICE_SPEAKER:
		ret = dlv_set_route(REPLAY_BTL);
		if(ret != REPLAY_BTL)
		{
			printk("TAS5713 CODEC: set device SND_DEVICE_SPEAKER error!\n");
			return -1;
		}
		break;

	case SND_DEVICE_HEADSET_AND_SPEAKER:
		ret = dlv_set_route(REPLAY_BTL);
		if(ret != REPLAY_BTL)
		{
			printk("TAS5713 CODEC: set device SND_DEVICE_HEADSET_AND_SPEAKER error!\n");
			return -1;
		}			
		break;

	default:
		iserror = 1;
		printk("TAS5713 DLV: Unkown ioctl argument in SND_SET_DEVICE\n");
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
	printk("TAS5713_DLV: waring, %s() is a default function\n", __func__);

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
		printk("TAS5713 DLV: Unkown ioctl argument in SND_SET_STANDBY\n");

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

//	__dlv_set_gm1(fixed_vol);
//	__dlv_set_gm2(fixed_vol);

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

static int dlv_set_replay_rate(int rate)
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

	if( mrate[speed] == 48000 ){
		cpm_set_clock(CGU_I2SCLK, JZ_EXTAL);
	} else if( mrate[speed] == 44100 ){
		cpm_set_clock(CGU_I2SCLK, 11289600);
	}

//	__dlv_select_dac_samp_rate(speed);

	return mrate[speed];
}

static int dlv_set_replay_data_width(int width)
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

static int dlv_set_replay_volume(int val)
{
	/*just set analog gol and gor*/
//	unsigned long fixed_vol;
//	int volume_base;
#if 0
	if(dlv_platform_data->dlv_replay_volume_base)
	{
		volume_base = dlv_platform_data->dlv_replay_volume_base;


		fixed_vol = (6 - volume_base) + 
			((25 + volume_base) * (100 - val) / 100);			
	}
	else 
		fixed_vol = (6 + (25 * (100 - val) / 100));

	__dlv_set_hp_volume(fixed_vol);

	if (val == 0) {
//		__dlv_set_godr(0x1f | 0x80);
		dlv_mute(1);
	} else {
//		__dlv_set_godr(0x0 | 0x80);
		dlv_mute(0);
	}
#else
	val = 255 * (100 - val) / 100;
	tas5713_set_volume(val & 0xff);
#endif

	return val;
}

static int dlv_set_replay_channel(int channel)
{
	channel = (channel >= 2) + 1;

	switch (channel) {
	case 1:
		// MONO->1 for Mono
		dlv_set_replay_filter(REPLAY_FILTER_MONO);
		break;
	case 2:
		// MONO->0 for Stereo
		dlv_set_replay_filter(REPLAY_FILTER_STEREO);
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

static int dlv_mute(int val)
{
	tas5713_set_mute(val);

	return 0;
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

	DUMP_IOC_CMD();

	DLV_LOCK();
	{
		switch (cmd) {

		case CODEC_INIT:
			ret = dlv_init();
			break;

		case CODEC_TURN_OFF:
			ret = dlv_turn_off(arg);
			break;

		case CODEC_SHUTDOWN:
			ret = dlv_shutdown();
			break;

		case CODEC_RESET:
			ret = dlv_reset();
			break;

		case CODEC_SUSPEND:
			ret = dlv_suspend();
			break;

		case CODEC_RESUME:
			ret = dlv_resume();
			break;

		case CODEC_ANTI_POP:
			ret = dlv_anti_pop((int)arg);
			break;

		case CODEC_SET_ROUTE:
			ret = dlv_set_route((int)arg);
			break;

		case CODEC_SET_DEVICE:
			if (dlv_platform_data->dlv_set_device)
			{
				ret = dlv_platform_data->dlv_set_device((struct snd_device_config *)arg);
				if (ret)
				{
					ret = dlv_set_device((struct snd_device_config *)arg);
				}
			} else
				ret = dlv_set_device((struct snd_device_config *)arg);
			break;

		case CODEC_SET_STANDBY:
			if (dlv_platform_data->dlv_set_standby)
			{
				ret = dlv_platform_data->dlv_set_standby((unsigned int)arg);
				if (ret)
				{
					ret = dlv_set_standby((unsigned int)arg);
				}
			} else
				ret = dlv_set_standby((unsigned int)arg);
			break;

		case CODEC_SET_RECORD_RATE:
			ret = dlv_set_record_rate((int)arg);
			break;

		case CODEC_SET_RECORD_DATA_WIDTH:
			ret = dlv_set_record_data_width((int)arg);
			break;

		case CODEC_SET_MIC_VOLUME:
			ret = dlv_set_record_volume((int)arg);
			break;

		case CODEC_SET_RECORD_CHANNEL:
			ret = dlv_set_record_channel((int)arg);
			break;

		case CODEC_SET_REPLAY_RATE:
			ret = dlv_set_replay_rate((int)arg);
			break;

		case CODEC_SET_REPLAY_DATA_WIDTH:
			ret = dlv_set_replay_data_width((int)arg);
			break;

		case CODEC_SET_REPLAY_VOLUME:
			ret = dlv_set_replay_volume((int)arg);
	
			break;

		case CODEC_SET_REPLAY_CHANNEL:
			ret = dlv_set_replay_channel((int)arg);
			break;

		case CODEC_DAC_MUTE:
			ret = dlv_mute((int)arg);
			break;

		case CODEC_DEBUG_ROUTINE:
			ret = dlv_debug_routine((void *)arg);
			break;

		default:
			printk("TAS5713 DLV:%s:%d: Unkown IOC commond\n", __FUNCTION__, __LINE__);
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
		printk("TAS5713 HP Switch kzalloc failed (%s:%d)\n", __FUNCTION__, __LINE__);
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
		printk("TAS5713 HP Switch: Could net register switch device\n");
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

/*------------------------------------------*/

static int jz_dlv_probe(struct platform_device *pdev)
{
	dlv_platform_data = pdev->dev.platform_data;

	return 0;
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
		.name	= "jz_dlv",
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
	printk("===================================================================================================\n");
	register_jz_codecs((void *)jzdlv_ioctl);

	dlv_reset_part();
#if 0
	retval = request_irq(IRQ_AIC, dlv_codec_irq, IRQF_DISABLED, "dlv_codec_irq", NULL);
	if (retval) {
		printk("TAS5713 DLV: Could not get AIC CODEC irq %d\n", IRQ_AIC);
		return retval;
	}
#endif

	tas5713_reset();

#if USE_OLD_I2C
	tas5713_data.i2c_handle = i2c_request(TAS5713_CHIPADDR,100000);
#else
	i2c_add_driver(&tas5713_i2c_driver);
#endif	

	tas5713_init_reg();

	tas5713_dump_all();

#ifdef CONFIG_HP_SENSE_DETECT
	retval = platform_driver_register(&jz_hp_switch_driver);
	if (retval) {
		printk("TAS5713 HP Switch: Could net register headphone sense switch\n");
		return retval;
	}
#endif

	retval = platform_driver_register(&jz_dlv_driver);
	if (retval) {
		printk("TAS5713 CODEC: Could net register jz_dlv_driver\n");
		return retval;
	}

	return 0;
}

/**
 * Module exit
 */
static void __exit cleanup_dlv(void)
{

	free_irq(IRQ_AIC, NULL);

#if USE_OLD_I2C
#else
	i2c_del_driver(&tas5713_i2c_driver);
#endif

#ifdef CONFIG_HP_SENSE_DETECT
	platform_driver_unregister(&jz_hp_switch_driver);
#endif
	platform_driver_unregister(&jz_dlv_driver);
}

module_init(init_dlv);
module_exit(cleanup_dlv);
