/****************************************** ************************************
 *
 *  Author:  <xfli@ingenic.cn>
 *
 *
 *******************************************************************************
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/jzsoc.h>


#include "simulate_i2c.h"

#ifndef abs
#define abs(a)              (((a) < 0) ? (-(a)) : (a))
#endif

/*Use simulated i2c macro switch */
#define VIRTUAL_IIC	 1

#if  VIRTUAL_IIC
/*Controll the timeing of clock keep low*/
#define TIMING_1   2
/*Controll the timeing of clock keep high*/
#define TIMING_2   2
/* delay timing before sending setting info, > 500ms*/
#define TIMING_3   10

#define GPIO_SDA		(32*4+30)	//GPE30
#define GPIO_SCL		(32*4+31)	//GPE31

#define I2C_LOW		(0)
#define I2C_HIGH		(1)

#define FM_DEV_ADDR            0x11
#define GSENSOR_DEV_ADDR       0x15

/*
  *********************************************************************************
  */
 
static void delay_x_us(unsigned int time)
{
	 udelay(time);
}
#if 0
static void delay_x_ms(unsigned int time)
{
	  mdelay(time);
}
#endif
static void delay_30_us(void)
{	
	delay_x_us(3);							//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}

static unsigned char inline read_sda(void)
{
	if(__gpio_get_pin(GPIO_SDA))
		return 1;
	else
		return 0;	
}

static void inline sda_out_low(void)
{
	__gpio_as_output(GPIO_SDA);
	__gpio_clear_pin(GPIO_SDA);
	
}

static void inline scl_out_low(void)
{	
	__gpio_as_output(GPIO_SCL);
	__gpio_clear_pin(GPIO_SCL);
}

static void inline scl_in(void)
{
	__gpio_as_input(GPIO_SCL);
	//__gpio_as_output(GPIO_SCL);
	//__gpio_set_pin(GPIO_SCL);
}

static void inline sda_in(void)
{
	__gpio_as_input(GPIO_SDA);
	//__gpio_as_output(GPIO_SDA);
	//__gpio_set_pin(GPIO_SDA);
}

/*
  *********************************************************************************
  */
static void I2C_start(void)
{
	scl_in();
	sda_in();              				//bus to free status
	delay_x_us(TIMING_1);
	sda_out_low();
	delay_x_us(TIMING_2);
}

static void I2C_stop(void)
{
	scl_out_low();
	sda_out_low();
	delay_x_us(TIMING_1);
	scl_in();            				// clock line to high
	delay_x_us(TIMING_2);
	sda_in();            				//data line to high
}

static unsigned char  I2C_read_ack(void)
{
	scl_out_low();		
	sda_in();           				//set to input after making  low, in order to preventing cause stop singal
	delay_x_us(TIMING_1);
	scl_in();
	delay_x_us(TIMING_2);
	
	if(read_sda()) {
		scl_out_low();
		return 1;
	} else {
		scl_out_low();
		return 0;
	}
}
  
static void I2C_send_ack(unsigned char  ACK_S)
{
	scl_out_low();
	
	if(ACK_S)
		sda_in();
	else
		sda_out_low();
	
	delay_x_us(TIMING_1);
	scl_in();
	delay_x_us(TIMING_2);
	scl_out_low();
	sda_in();           				//must free bus after sending ACK singal
}
  

static void send_one_byte(unsigned char  byte)
{
	unsigned char  var1;
	unsigned char  i;
	unsigned char  j;
	
	var1=byte;
	scl_out_low();
	
	for(i=0; i<8; var1=var1<<1, i++)
	{
		j=1;
		scl_out_low();
		while(j--);
		
		if(var1 & 0x80)
			sda_in();
		else 
			sda_out_low(); 		//data ready when clock is low.¡¡

		delay_x_us(TIMING_1);
		scl_in();
		delay_x_us(TIMING_2);
		scl_out_low();
	}  
}
 
   
static unsigned char  read_one_byte(void)
{
	unsigned char  data=0;
	unsigned char  i;
	
	scl_out_low();
	sda_in();
	
	for(i=0; i<8; i++)
	{
		scl_out_low();
		delay_x_us(TIMING_1);
		scl_in();
		data = data << 1;					
		delay_x_us(TIMING_2);
		
		if(read_sda())
			data |= 1;	
		
		scl_out_low();
	}
	return data;
}


/*
  *********************************************************************************
  */
static int simulate_i2c_read(unsigned char device, unsigned char *buf,unsigned char offset, int count)
{	
	unsigned char *ptr;
	int cnt;
	int timeout = 5;
	//printk("gpio read!\n");
	
restarts:
	ptr = buf;
	cnt = count;
	
	//__i2c_disable();
	if (timeout < 0)
		goto I2CERROR;
	
	I2C_start();							
	delay_30_us();	
	send_one_byte((device << 1) | I2C_WRITE);	//send device address and write flag
	if(I2C_read_ack())							//read ack
		goto i2c_stop;

	delay_30_us();
	send_one_byte(offset); 						//set operated register
	if( I2C_read_ack() )
		goto i2c_stop;

	I2C_start();								//re-start communicate
	delay_30_us();
	send_one_byte((device << 1) | I2C_READ);		//send device address and read flag
	I2C_send_ack(I2C_LOW);					//send low ack
	
	while (cnt) 
	{		
		if (cnt == 1) {
			delay_30_us();
			*ptr = read_one_byte();
			I2C_send_ack(I2C_HIGH);			//send high ack
		} else {
			delay_30_us();
			*ptr = read_one_byte();
			I2C_send_ack(I2C_LOW);			
		}
		
		cnt--;
		ptr++;
	}

	delay_30_us();
	I2C_stop();
	udelay(300);
	return count - cnt;
	
i2c_stop:
	timeout --;
	delay_30_us();
	I2C_stop();
	printk("ERROR: I2C READ REPEAT!\n");
	udelay(300);
	goto restarts;
	
I2CERROR:
	printk("ERROR: I2C READ TIMEOUT!\n");
	udelay(300);
	return -1;
}

static int simulate_i2c_write(unsigned char device, unsigned char *buf, unsigned char offset, int count)
{
	int cnt = count;
	int cnt_in_pg;
	int timeout = 5;
	unsigned char *tmpbuf;
	unsigned char tmpaddr;

	//printk("gpio write!\n\n");
	
restarts:
	//__i2c_disable();
	if (timeout < 0)
		goto I2CERROR;	
	
	cnt = count;
	tmpbuf = (unsigned char *)buf;
	tmpaddr = offset;

start_write_page:
	cnt_in_pg = 0;
 	I2C_start();
	delay_30_us();
	send_one_byte((device << 1) | I2C_WRITE);	//send device address and write flag
	if(I2C_read_ack())							//read ack
		goto i2c_stop;
	
	delay_30_us();
	send_one_byte(tmpaddr); 					//set operated register
	if(I2C_read_ack())
		goto i2c_stop;
		
	while (cnt) 
	{ 
		if (++cnt_in_pg > 8) {
			I2C_stop();
			mdelay(1);
			tmpaddr += 8;
			goto start_write_page;
		}
		
		delay_30_us();
		send_one_byte(*tmpbuf);
		if(I2C_read_ack())
			goto i2c_stop;
		
		cnt--;
		tmpbuf++;
	}

	delay_30_us();
	I2C_stop();
	udelay(300);
	return count - cnt;
	
i2c_stop:
	timeout --;
	delay_30_us();
	I2C_stop();
	printk("ERROR: I2C READ REPEAT!\n");
	udelay(300);
	goto restarts;	
	
I2CERROR:
	printk("ERROR: I2C READ TIMEOUT!\n");
	udelay(300);
	return -1;
}

int fm_i2c_read_device(unsigned char reg, int bytes, void *dest)
{
	int ret;
	
	ret = simulate_i2c_read((unsigned char)FM_DEV_ADDR, (unsigned char *)dest, reg, bytes);
	if (ret < 0)
		return ret;
	if (ret < bytes)
		return -2;
	
	return 0;
}

/* 	Currently we allocate the write buffer on the stack; this is OK for
  * 	small writes - if we need to do large writes this will need to be
  * 	revised.
  */
int fm_i2c_write_device(unsigned char reg, int bytes, void *src)
{
	int ret;

	ret = simulate_i2c_write((unsigned char)FM_DEV_ADDR, (unsigned char *)src, reg, bytes);
	if (ret < 0)
		return ret;
	if (ret < bytes)
		return -2;

	return 0;
}

int gsensor_i2c_read_device(unsigned char reg, int bytes, void *dest)
{
	int ret;
	
	ret = simulate_i2c_read((unsigned char)GSENSOR_DEV_ADDR, (unsigned char *)dest, reg, bytes);
	if (ret < 0)
		return ret;
	if (ret < bytes)
		return -2;
	
	return 0;
}

#endif

