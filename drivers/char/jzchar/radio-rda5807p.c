#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/jzsoc.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/i2c.h>
/* Medive add */
#include "fm_i2c.h"

extern int jz_audio_open_for_fm(void);
extern int jz_audio_close_for_fm(void);
extern void dlv_set_replay_volume(int val);
//extern void dlv_enable_line_in_bypass_hp(void);

MODULE_AUTHOR("caijicheng<caijicheng2006@163.com>");
MODULE_DESCRIPTION("RDA5807P radio Driver");
MODULE_LICENSE("GPL");

//#define AMPLIFIER_EN (32*3+28)
//#define AMPLIFIER   (32*1+17)
//#define AMPLIFIER_IRQ (AMPLIFIER + IRQ_GPIO_0)

#define FM_DEV_NAME "fm_rda5807p"
#define FM_MINOR 0x42

#define FM_I2C_ADDR 0x11

#define AUTO_SEEK 0x0
#define AUTO_SEEK_JAPAN 0x01
#define SET_FREQ 0x02
#define SET_VOLUME 0x03
#define READ_VOLUME 0x04
#define SET_STEREO 0x05
//#define READ_STEREO 0x06
#define FM_POWER_ON 0x06
#define FM_POWER_OFF 0x07
#define SET_MUTE     0x08
#define SET_THRESHOLD     0x09
#define SET_AREA     0x0a
//unsigned char write_data[8] = {0xc0, 0x01, 0x00, 0x10, 0x40, 0x00, 0x88, 0xaf};

unsigned char reset_chip[3] = {0x00,0x00, 0x02};
unsigned char power_on_chip[3] = {0x00,0xc0, 0x01};
//unsigned char read_data[8] = {0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char read_data[5] = {0x00, 0x00, 0x00, 0x00,0x00};
unsigned char set_tune[3] = {0x00,0x00, 0x10};
//unsigned char set_threshold[2]={0x14, 0xa8};
unsigned char set_threshold[3]={0x00,0x08, 0xa8};
//unsigned char stereo_indicator[2] = {0x00,0x00};

//unsigned int pll;
//unsigned int frequency = 103900;
//unsigned int save_reg[2];
unsigned int freq[220];

static void radio_set_volume(int val)
{
	dlv_set_replay_volume(val);
}

/****    codec set input line  end   ****/

/* NOTE: lower 4bits of reg03 is set in fm_ioctl, and can NOT change in other place */
u16 rda5807p_freq_to_chan(u16 frequency)
{
	u8 channel_spacing=0;
	u16 bottom_band=0;
	u16 channel=0;

	if ((set_tune[2] & 0x0c) == 0x00) /* US/Europe */
		bottom_band = 870;
	else if ((set_tune[2] & 0x0c) == 0x04) /* Japan */
		bottom_band = 760;
	else if ((set_tune[2] & 0x0c) == 0x08) /* Japan wide */
		bottom_band = 760;


	/* because we can not use float, so channel spacing is 2x */
	switch(set_tune[2] & 0x03) {
	case 0x0:
		channel_spacing = 200;
		break;
	case 0x1:
		channel_spacing = 400;
		break;
	case 0x2:
		channel_spacing = 100;
		break;
	case 0x3:
		channel_spacing = 25;

	}

	channel = ((frequency - bottom_band) * 2 * 100) / channel_spacing;

#if 0
	printk("bottom_band = %d, 2xchannel_spacing = %d, channel = %d\n",
	       bottom_band, channel_spacing, channel);
#endif

	return (channel);
}

/* cur_freq 870~1080 */
int rda5807p_set_freq(u16 cur_freq, int wait_true)
{
	int i = 0;
	int wait_time;
	u16 cur_chan;

	cur_chan = rda5807p_freq_to_chan(cur_freq);
	set_tune[0] = 0x03;
	set_tune[1] = cur_chan >> 2;

	/* NOTE: lower 4bits of reg03 is set in fm_ioctl, and can NOT change in other place */
	set_tune[2] |= 0x10;  /* tune enable */
	set_tune[2] |= ((cur_chan & 0x3) << 6);

	//fm5807_i2c_txdata(set_tune,3);
	fm_i2c_write_device(0x03,2,&set_tune[1]);

	if (wait_true)
		wait_time = 25; /* 25ms */
	else
		wait_time = 15; /* 15ms */

	for (i = 0; i < wait_time; i++) {
		memset(read_data, 0, 4);
		//read_data[0] = 0x0a;
		//fm5807_i2c_rxdata(read_data, 4);
		fm_i2c_read_device(0x0a,4,read_data);
#if 0
		printk("read_data = %x, %x, %x, %x\n",
		       read_data[0], read_data[1],
		       read_data[2], read_data[3]);
#endif

#define FM_STC		(1 << 6) /* 0x0a, bit14 */

#define FM_TRUE		(1 << 0) /* 0x0b, bit8 */
#define FM_READY	(1 << 7) /* 0x0b, bit7 */

		if ((read_data[0]) & FM_STC) /* tune complete */
		{
			if (!wait_true){
				//printk("yes0 a!\n");
				return 1;
			}

			if ((read_data[2] & FM_TRUE) &&
					(read_data[3] & FM_READY)) {
				//printk("yes1 a!\n");
				return 1;
			}
		}

		/* continue */
		msleep(1);
	}

	return 0;
}

static int fm_open(struct inode *inode, struct file *filp)
{

	//__gpio_as_i2c(0);
	//udelay(300);
	//cpm_start_clock(CGM_I2C0);
	//__i2c_disable(0);
	//__i2c_enable(0);

	//radio_reset();
	//set_playback_line_input_audio();

	//__i2c_enable(1);

	//cpm_start_clock(CGM_I2C1);

#if defined(CONFIG_JZ4770_PISCES)
	act8600_output_enable(ACT8600_OUT6,ACT8600_OUT_ON);
#endif
	jz_audio_open_for_fm();
	mdelay(10);

	/* reset chip */
	//i2c_write(FM_I2C_ADDR, reset_chip, 0x02, 2);
	printk("fm open  start  ,please check it !\n");

	/* Medive change i2c write function*/
	//reset_chip[0] = 0x02;
	//fm5807_i2c_txdata(reset_chip,3);
	fm_i2c_write_device(0x02,2,&reset_chip[1]);
	


	/* power on chip */
	//i2c_write(FM_I2C_ADDR, power_on_chip, 0x02, 2);
	//power_on_chip[0] = 0x02;
	//fm5807_i2c_txdata(power_on_chip,3);
	fm_i2c_write_device(0x02,2,&power_on_chip[1]);

	//i2c_write(FM_I2C_ADDR, set_threshold, 0x05, 2);
	//set_threshold[0] = 0x05;
	//fm5807_i2c_txdata(set_threshold,3);
	fm_i2c_write_device(0x05,2,&set_threshold[1]);
	return 0;
}

static ssize_t fm_read(struct file *filp, char __user *buf, size_t count, loff_t *f_ops)
{
	return 4;
}

static ssize_t fm_write(struct file *filp, char __user *buf, size_t count, loff_t *f_ops)
{
	return 4;
}

static int fm_release(struct inode *inode, struct file *filp)
{
	printk("fm_release!\n");

	//radio_reset();
	jz_audio_close_for_fm();
	//shutdown fm power 
#if defined(CONFIG_JZ4770_PISCES)
	act8600_output_enable(ACT8600_OUT6,ACT8600_OUT_OFF);
#endif

	//__i2c_disable(1);

	//cpm_stop_clock(CGM_I2C1);
	//udelay(300);
	/* Medive change for disable i2c gpio function and i2c clock */
	//__i2c_disable(0);
	//cpm_stop_clock(CGM_I2C0);
	return 0;
}

static int area_flag = 0;    //1: other  2 : japen
static int fm_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	int threshold = 2;
	int seek=0;
	int stereo=2;
	int volume=0;
	int set_freq = 870;
	int mute = 2;
	switch (cmd)
	{
	case SET_AREA:
		get_user(area_flag, (int*)arg);
		printk("cur area is %d\n",area_flag);
		break;
	case AUTO_SEEK:
		get_user(seek, (int*)arg);
		set_tune[1] = 0x0;
		set_tune[2] = 0x10; /* band: 00, US/Europe, spacing: 00, 100KHZ */
		printk("rda5807p set freq = %d\n", seek);

		return rda5807p_set_freq(seek, 1);


#if 0
		/* Medive change for i2c read function */
		//i2c_read(FM_I2C_ADDR, read_data, 0x0a, 4);
		read_data[0] = 0x0a;
		fm5807_i2c_rxdata(read_data,4);
		printk("read_data = %x, %x, %x, %x\n", read_data[0], read_data[1], read_data[2], read_data[3]);
		mdelay(50);
		if (((read_data[0] >> 6) & 0x01) && (read_data[2] & 0x01))
		{
			printk("++++++china get tune! freq = %d+++++++\n", seek);
//			put_user(seek, (int*)arg);
			return 1;
		}
		return 0;
#endif
		break;
	case AUTO_SEEK_JAPAN:
		get_user(seek, (int*)arg);
		set_tune[1] = 0x0;
		set_tune[2] = 0x14; /* band: 01, japan, spacing: 00, 100KHZ */

		return rda5807p_set_freq(seek, 1);

#if 0
		//i2c_read(FM_I2C_ADDR, read_data, 0x0a, 4);
		read_data[0] = 0x0a;
		fm5807_i2c_rxdata(read_data,4);

		if (((read_data[0] >> 6) & 0x01) && (read_data[2] & 0x01))
		{
			printk("++++++japan get tune! freq = %d+++++++\n", seek);
//			put_user(seek, (int*)arg);
			return 1;
		}
		return 0;
#endif
		break;
	case SET_FREQ:
		get_user(set_freq, (int*)arg);

		//medive change
		
		if (area_flag == 1){
			set_tune[1] = 0x0;
			set_tune[2] = 0x10; /* band: 00, US/Europe, spacing: 00, 100KHZ */

		}else if (area_flag == 2){
			set_tune[1] = 0x0;
			set_tune[2] = 0x14; /* band: 01, japan, spacing: 00, 100KHZ */
		}else{
			set_tune[1] = 0x0;
			set_tune[2] = 0x10; /* band: 00, US/Europe, spacing: 00, 100KHZ */
		}


		//end

		rda5807p_set_freq(set_freq, 0);
		break;
	case SET_VOLUME:
		get_user(volume, (int*)arg); // volume : 0 ~ 31

		if (volume < 0)
			volume = 0;
		if (volume > 31)
			volume = 31;

		radio_set_volume(volume*3);

//		REG_ICDC_CDCCR2 = ((volume)<<16 | 0x0003);
		mdelay(2);
		break;
	case READ_VOLUME:
//		volume = (REG_ICDC_CDCCR2>>16);
		mdelay(2);
		put_user(volume, (int*)arg);
		break;
	case SET_MUTE:
		get_user(mute, (int*)arg);
		if (mute == 0)
		{
			power_on_chip[1] &= ~(1<<7);
			printk("set mute = %d\n", mute);
		}else{
			power_on_chip[1] |= (1<<7);
			printk("set normal = %d\n", mute);
		}
		//i2c_write(FM_I2C_ADDR, power_on_chip, 0x02, 2);
		//power_on_chip[0] = 0x02;
		//fm5807_i2c_txdata(power_on_chip,3);
		fm_i2c_write_device(0x02,2,&power_on_chip[1]);
		mdelay(50);
		break;
	case SET_STEREO:
		get_user(stereo, (int*)arg);
		if (stereo == 0) // 0:set stereo  1: set mono
		{
			power_on_chip[1] &= ~(1<<6);
			printk("set stereo = %d\n", stereo);
		}else{
			power_on_chip[1] |= (1<<6);
			printk("set mono = %d\n", stereo);
		}
		//i2c_write(FM_I2C_ADDR, power_on_chip, 0x02, 2);
		//power_on_chip[0] = 0x02;
		//fm5807_i2c_txdata(power_on_chip,3);
		fm_i2c_write_device(0x02,2,&power_on_chip[1]);
		mdelay(50);
		break;
/*	case READ_STEREO:
	//	stereo_indicator[1] |= (1<<4);
	//	i2c_write(FM_I2C_ADDR, stereo_indicator, 0x04, 2);
//		mdelay(20);
		i2c_read(FM_I2C_ADDR, read_data, 0x0a, 5);
		mdelay(50);
		put_user((((read_data[0]>>6)&0x01) == 0x0)?0:1, (int*)arg);// 1: stereo, 0: mono
		break;
*/
	case FM_POWER_ON:
		power_on_chip[1] =0xc0;
		power_on_chip[2] =0x01;
		//power_on_chip[1] =0xd2;
		//power_on_chip[2] =0x81;
		set_threshold[1] =0x14;
		set_threshold[2] =0xa8;

		printk("fm power on  !\n");
#if 0
		__gpio_as_i2c(0);
		udelay(300);
		cpm_start_clock(CGM_I2C0);
		__i2c_disable(0);
		__i2c_enable(0);
#endif

		//aic_init();

		//i2c_write(FM_I2C_ADDR, reset_chip, 0x02, 2);
		//reset_chip[0] = 0x02;
		//fm5807_i2c_txdata(reset_chip,3);
		fm_i2c_write_device(0x02,2,&reset_chip[1]);
		//i2c_write(FM_I2C_ADDR, power_on_chip, 0x02, 2);
		//power_on_chip[0] = 0x02;
		//fm5807_i2c_txdata(power_on_chip,3);
		fm_i2c_write_device(0x02,2,&power_on_chip[1]);
		//i2c_write(FM_I2C_ADDR, set_threshold, 0x05, 2);
		//set_threshold[0] = 0x05;
		//fm5807_i2c_txdata(set_threshold,3);
		fm_i2c_write_device(0x05,2,&set_threshold[1]);

		break;
	case FM_POWER_OFF:
		power_on_chip[2] &= (~0x01);
		//i2c_write(FM_I2C_ADDR, power_on_chip, 0x02, 2);
		//power_on_chip[0] = 0x02;
		//fm5807_i2c_txdata(power_on_chip,3);
		fm_i2c_write_device(0x02,2,&power_on_chip[1]);
		mdelay(20);
		printk("FM_POWER_OFF\n");
		break;
	case SET_THRESHOLD:
		get_user(threshold, (int*)arg);
		if (threshold == 1){ // 1: use auto seek, 0:use manual seek
			set_threshold[1] = 0x0d;
			//set_threshold[0] = 0x08;
			set_threshold[2] = 0xa8;
		}else if (threshold == 2){
			set_threshold[1] = 0x0b;
			set_threshold[2] = 0xa8;
		}
        else if (threshold == 3){
          set_threshold[1] = 0x08;
          set_threshold[2] = 0xa8;
        }

		//i2c_write(FM_I2C_ADDR, set_threshold, 0x05, 2);
		//set_threshold[0] = 0x05;
		//fm5807_i2c_txdata(set_threshold,3);
		fm_i2c_write_device(0x05,2,&set_threshold[1]);
		break;
	default:
		printk("can not support fm ioctl\n");
		return -EINVAL;
		break;
	}
	return 0;
}

/*
static irqreturn_t amplifier_handle(int irq, void *dev_id)
{
	if (__gpio_get_pin(AMPLIFIER) == 0)
	{
		__gpio_clear_pin(AMPLIFIER_EN);
		__gpio_as_irq_rise_edge(AMPLIFIER);
	}
	else
	{
		__gpio_set_pin(AMPLIFIER_EN);
		printk("set AMPLIFIER_EN !\n");
		__gpio_as_irq_fall_edge(AMPLIFIER);
	}

	
	return IRQ_HANDLED;
}
*/

static struct file_operations fm_fops = {
	.owner   = THIS_MODULE,
	.open    = fm_open,
	.read    = fm_read,
	.write   = fm_write,
	.ioctl   = fm_ioctl,
	.release = fm_release,
};

static struct miscdevice fm_device = {
	.minor = FM_MINOR,
	.name  = FM_DEV_NAME,
	.fops  = &fm_fops,
};

static int __init rda5807p_init(void)
{
	int ret = 0;
	printk("init rda5807p \n");
//	int j = 0;
//	char flag = 0;
//	static unsigned short freq[64];
//	unsigned int i = 0;
//	unsigned short tune = 0x0;

/*
	//test
	__gpio_as_i2c();
	udelay(200);
	__cpm_start_i2c();	
	i2c_close();
	i2c_open();

	while (1)
	{
		// reset chip
		i2c_write(FM_I2C_ADDR, reset_chip, 0x02, 2);
		mdelay(50);
		// power on chip
		i2c_write(FM_I2C_ADDR, power_on_chip, 0x02, 2);	
		mdelay(500);
		// read chip id
		i2c_read(FM_I2C_ADDR, read_data, 0x0d, 2);
		printk("read id = %x, %x\n", read_data[0], read_data[1]);
	
		rda5807p_set_freq(1071);
		// init chip
//		i2c_write(FM_I2C_ADDR, fm_rda5807p_init, 0x02, 64);
		mdelay(60000);
//		i2c_read(FM_I2C_ADDR, read_data, 0x0a, 8);
//		printk("read = %x,%x,%x,%x,%x,%x,%x,%x\n", read_data[0],read_data[1],read_data[2],
//				read_data[3],read_data[4],read_data[5],read_data[6],read_data[7]);
	}
*/

/*	printk("init rda5807p!\n");
	if (aic_init() != 0)
	{
		printk("init aic err!\n");
		return -1;
	}

	// reset chip
	i2c_write(FM_I2C_ADDR, reset_chip, 0x02, 2);
	mdelay(50);
	// power on chip
	i2c_write(FM_I2C_ADDR, power_on_chip, 0x02, 2);	
	mdelay(50);
	printk("---------init----------\n"); 
//	i2c_write(FM_I2C_ADDR, set_tune, 0x03, 2);
//	mdelay(50);
*/
/*	i = 870;
	while(1)
	{
		rda5807p_set_freq(i);
		printk("rda5087p_set_freq\n");
		i2c_read(FM_I2C_ADDR, read_data, 0x0a, 4);
		mdelay(100);
		if (((read_data[0] >> 6) & 0x01))
		{
			if (read_data[2] & 0x01)
			{
				freq[j++] = i;
				printk("/n ++++++++++++++++++get tune! freq[%d] = %d++++++++++++++\n", j, i);
				mdelay(10000);
			}
			if (flag == 1)
			{
				for (i = 0 ; i<64; i++)
					printk("all tune[%d] = %d\n", i, freq[i]);
				//while(1);
				break;
			}
		}

		if (i >= 1080)
			flag = 1;
		i++;
	}
*/
	ret = misc_register(&fm_device);
	if (ret < 0)
	{
		printk("misc register fm err!\n");
		return ret;
	}

#if 0
	__gpio_disable_pull(AMPLIFIER_EN);
	__gpio_as_output(AMPLIFIER_EN);

	if (__gpio_get_pin(AMPLIFIER) == 1)
	{
		__gpio_set_pin(AMPLIFIER_EN);
		__gpio_as_irq_fall_edge(AMPLIFIER);
	}
	else
	{
		__gpio_clear_pin(AMPLIFIER_EN);
		__gpio_as_irq_rise_edge(AMPLIFIER);
	}	

	ret = request_irq(AMPLIFIER_IRQ, amplifier_handle, IRQF_DISABLED, "amplifier", NULL);
	if (ret < 0)
	{
		printk("amplifier request_irq err!\n");
		return ret;
	}
#endif

//	save_reg[0] = REG_ICDC_CDCCR1;
//	save_reg[1] = REG_ICDC_CDCCR2;
	return 0;	
}

static void __exit rda5807p_exit(void)
{
	printk("exit rda5807p!\n");
	misc_deregister(&fm_device);
	//free_irq(AMPLIFIER_IRQ, NULL);
}

module_init(rda5807p_init);
module_exit(rda5807p_exit);
