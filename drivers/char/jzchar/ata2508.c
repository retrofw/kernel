
#include <linux/autoconf.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/spinlock.h>

#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/jzsoc.h>

#define MP4_KEY_RST	(32*3+3)
#define MP4_KEY_TINT	(32*3+2)
#define MP4_KEY_SCL	(32*3+1)
#define MP4_KEY_SDA	(32*3+0)
#define MP4_TINT_IRQ       (IRQ_GPIO_0 + MP4_KEY_TINT)

#define ADDR_WARM_RESET		0xFF
#define ATA2508_SENSOR_MASK 	0x1F

const unsigned char init_data_burst[] = {//Address:0x0D-0x3E
	0x04, // BETA 
	0x27, // AIC_WAIT 
	//0x32, // REF_DELAY
	0x16, // REF_DELAY
	0x02, // HYSTERESIS01 
	0x02, // HYSTERESIS1
	0x02, // HYSTERESIS2
	0x02, // HYSTERESIS3 
	0x02, // HYSTERESIS4 
	0x02, // HYSTERESIS51
	0x02, // HYSTERESIS61 
	0x02, // HYSTERESIS7 
	0x02, // HYSTERESIS8 
	0x02, // HYSTERESIS9 
	0x02, // HYSTERESIS10 
	0x02, // HYSTERESIS11 
	0x64, // STRENGTH_THRESHOLD0
	0x64, // STRENGTH_THRESHOLD1
	0x64, // STRENGTH_THRESHOLD2
	0x64, // STRENGTH_THRESHOLD3
	0x64, // STRENGTH_THRESHOLD4
	0x64, // STRENGTH_THRESHOLD5
	0x64, // STRENGTH_THRESHOLD6
	0x64, // STRENGTH_THRESHOLD7
	0x64, // STRENGTH_THRESHOLD8
	0x64, // STRENGTH_THRESHOLD9
	0x64, // STRENGTH_THRESHOLD10
	0x64, // STRENGTH_THRESHOLD11
	0x0f, // Sampling Interval
	0xC8, // INTEGRATION TIME
	0x0f, // IDLE TIME
	0x00, // SIF_SETUP(RESERVED)
	0x01, // MODE 
	0x00, // GPIO_REG_L 
	0x00, // GPIO_REG_H 
	0x00, // GPIO_CONFIGURATION_L
	0x00, // GPIO_CONFIGURATION_H
	0x00, // GPIO_DIR_L 
	0x00, // GPIO_DIR_H 
	0x0c, // CONTROL 
	0x38, // INT_MASK 
	0x00, // INT_CLEAR 
	0xFF, // INT_edge 
	0x02, // CONTROL_2 
	0xAF, // BEEP_TIME 
	0x7F, // BEEP_FREQUENCY 
	0x30, // CALIBRATION INTERVAL 
	0x00, // EINT_ENABLE 
	0x00, // EINT_POL 
	0x00, // FILTER_PERIOD 
	0x00, // FILTER_THRESHOLD 
};
const unsigned char init_data_alpha[] = {//Address:0x00-0x0C
	0x02, // APIS 
	0x08, // ALPHA0 
	0x08, // ALPHA1 
	0x08, // ALPHA2 
	0x08, // ALPHA3 
	0x08, // ALPHA4 
	0x28, // ALPHA5 
	0x28, // ALPHA6 
	0x28, // ALPHA7
	0x28, // ALPHA8 
	0x28, // ALPHA9 
	0x28, // ALPHA10 
	0x28, // ALPHA11 
};
static unsigned int i2c_addr = 0x58;
static unsigned int i2c_clk = 100000;

static void write_reg(u8 reg, u8 val)
{
	int ret;
	i2c_open();
	i2c_setclk(i2c_clk);
	ret = i2c_write(i2c_addr, &val, reg, 1);
	i2c_close();
}

static u8 read_reg(u8 reg)
{
	u8 val;

	i2c_open();
	i2c_setclk(i2c_clk);
	i2c_read(i2c_addr, &val, reg, 1);
	i2c_close();
	return val;
}

/*
 * Interrupt handler
 */
static irqreturn_t mp4_tint_irq(int irq, void *dev_id)
{
	int key_num = 0;
	u8 value0, value1;
	
	__gpio_ack_irq(MP4_KEY_TINT);
	value0 = read_reg(0x75);
	value1 = read_reg(0x76);
	value0 &= ATA2508_SENSOR_MASK;
	if (value0 == 0) {
		printk("\nRelease key!\n");
		return IRQ_HANDLED;
	}
	while(value0 >> 1){
		value0 >>= 1;
		key_num++;
	}
	
	printk("\nPress key %d!\n", key_num);
	return IRQ_HANDLED;
}

static int __init init_ata2508(void)
{
	int i;
	unsigned char data1;
	int retval;

	__gpio_as_output(MP4_KEY_RST);
	__gpio_set_pin(MP4_KEY_RST);
	mdelay(100);
	__gpio_clear_pin(MP4_KEY_RST);
	mdelay(800);
	__gpio_set_pin(MP4_KEY_RST);
	__gpio_mask_irq(MP4_KEY_TINT);

	/*write registers*/
	for(i=0; i<13; i++)
	{
		data1 = init_data_alpha[i];
		write_reg(i, data1);
	}
	
	for(i=13; i<63; i++)
	{
		data1 = init_data_burst[i-13];
		write_reg(i, data1);
	}
#if 0
	for (i = 0; i < 63; i++)
	{
		data1 = read_reg(i);
		printk("REG0x%02x = 0x%02x\n", i, data1);
	}
#endif
	
	/* wait for 1 ms*/
	mdelay(1);
#if 0
	while(1)
	{
		data1 = read_reg(0x68);
		printk("REG0x68 = %d\n", data1);
		data1 = read_reg(0x75);
		printk("REG0x75 = 0x%02x\n", data1);
		data1 = read_reg(0x76);
		printk("REG0x76 = 0x%02x\n", data1);
		mdelay(2000);
	}
#endif
	data1 = read_reg(0x68);
	printk("REG0x68 = %d\n", data1);

	/* to activate all the new settings, give a WARM RESET.*/
	write_reg(ADDR_WARM_RESET, 0x00);	//ADDR_WARM_RESET=0xFF

	//printk("REG0x68 = %d\n", data1);
	
	/* wait for 1 ~ 10 ms.*/
	mdelay(10);
	data1 = read_reg(0x68);

	/* Enable INT that connected to ATA2508's TINT.*/
	__gpio_as_irq_rise_edge(MP4_KEY_TINT);

	retval = request_irq(MP4_TINT_IRQ, mp4_tint_irq,
			     IRQF_DISABLED, "mp4_key_tint", NULL);
	if (retval) {
		printk("Could not get mp4 key irq %d\n", MP4_TINT_IRQ);
		return retval;
	}

	printk(JZ_SOC_NAME ": MP4 touch panel register.\n");

	return 0;
}

static void __exit exit_ata2508(void)
{
	free_irq(MP4_TINT_IRQ, NULL);	
}

module_init(init_ata2508);
module_exit(exit_ata2508);
