/*
 * linux/drivers/char/jzchar/sadc.c
 *
 * SAR-ADC driver for JZ4740.
 *
 * Copyright (C) 2006  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/spinlock.h>

#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/jzsoc.h>

MODULE_AUTHOR("Benson.Yang<hpyang@ingenic.cn>");
MODULE_DESCRIPTION("Jz4770 TCU driver");
MODULE_LICENSE("GPL");

#define TCU_NAME        "tcu_driver"

#define ENTRY(n)   (REG_TCU_TFIFOSR(n)>>1)&0x1F

#define FIFO_CHANNEL(n)	(((0 == (n)) || (3 == (n)) || (4 == (n)) || (5 == (n))) ? 1 : 0)

struct jz_fifo_mode_data
{
	unsigned int  sum_count0;
	unsigned int  using_count0;
	unsigned int  sum_count1;
	unsigned int  using_count1;
	unsigned char buf1_used;
	unsigned char buf2_used;
	unsigned char buf_using;
	unsigned char reserved;
	unsigned int  fifo_mode;
	unsigned char *buf1;
	unsigned char *buf2;
};

struct tcu_device
{
	unsigned char channel_num;
	struct timer_list key_timer;
	struct miscdevice * tcu_dev;
	struct jz_fifo_mode_data tcu0;
	struct jz_fifo_mode_data tcu3;
	struct jz_fifo_mode_data tcu4;
	struct jz_fifo_mode_data tcu5;
};


struct tcu_device *jz_tcu_dev;

/*
 * File operations
 */

static int select_div(unsigned char divsor)
{
	unsigned char i,n;
	n = jz_tcu_dev->channel_num;
	i = divsor;
	switch (i)
	{
		case 0:
			__tcu_select_clk_div1(n);
			break;
		case 1:
			__tcu_select_clk_div4(n);
			break;
		case 2:
			__tcu_select_clk_div16(n);
			break;
		case 3:
			__tcu_select_clk_div64(n);
			break;
		case 4:
			__tcu_select_clk_div256(n);
			break;
		case 5:
			__tcu_select_clk_div1024(n);
			break;
		default:
			__tcu_select_clk_div1(n);
			break;
	}

	return 0;
}

static int select_clk(unsigned char clk)
{

	unsigned char channel;
	channel=jz_tcu_dev->channel_num;
	switch (clk)
	{
		case 0:	__tcu_select_extalclk(channel);
			break;
		case 1:	__tcu_select_rtcclk(channel);
			break;
		case 2:	__tcu_select_pclk(channel);
			break;
		default:__tcu_select_pclk(channel);
			break;
	}	
	return 0;
}

static void set_gpio_as_pwm(unsigned char channel)
{
	switch (channel)
	{
		case 0: __gpio_as_pwm (0);
			break;
		case 1: __gpio_as_pwm (1);
			break;
		case 2: __gpio_as_pwm (2);
			break;
		case 3: __gpio_as_pwm (3);
			break;
		case 4: __gpio_as_pwm (4);
			break;
		case 5: __gpio_as_pwm (5);
			break;
		case 6: __gpio_as_pwm (6);
			break;
		case 7: __gpio_as_pwm (7);
			break;
		default: __gpio_as_pwm(0);
			break;
	}
}

static int start_pwm_output(void)
{
	unsigned char channel;
	channel=jz_tcu_dev->channel_num;

	set_gpio_as_pwm(channel);

	__tcu_stop_counter(channel);
	__tcu_set_pwm_output_shutdown_graceful(channel);

	__tcu_select_extalclk(channel);
	__tcu_select_clk_div64(channel);
	__tcu_init_pwm_output_high(channel);

	__tcu_enable_pwm_output(channel);

	__tcu_start_counter(channel);
	if(jz_tcu_dev->tcu4.fifo_mode == 0x05)
		REG_TCU_TMCR=(0x01<<26);

	return 0;
}

static void start_pwm_bypass(void)
{
	unsigned char channel;
	channel=jz_tcu_dev->channel_num;

	set_gpio_as_pwm(channel);

	__tcu_stop_counter(channel);
	__tcu_set_pwm_output_shutdown_graceful(channel);
	__tcu_select_extalclk(channel);
	__tcu_select_clk_div4(channel);
	__tcu_init_pwm_output_high(channel);
	__tcu_enable_pwm_output(channel);
	REG_TCU_TCSR(channel) |= (0x1<<11);
}

static int start_pwm_input(void)
{
	unsigned char channel;
	channel=jz_tcu_dev->channel_num;
	__tcu_stop_counter(channel);
	__tcu_set_pwm_output_shutdown_graceful(channel);
	__tcu_select_clk_div1(channel);
	__tcu_set_half_data(channel,0xffff);
	__tcu_set_full_data(channel,0xffff);
	__tcu_set_count(channel,0x0);
	__tcu_disable_all_clk(channel);

	__tcu_start_counter(channel);

	return 0;
}

static void stop_pwm_input(void)
{
	unsigned char channel;
	channel=jz_tcu_dev->channel_num;
	__tcu_stop_counter(channel);
}

static int stop_pwm_output(void)
{
	unsigned char channel;
	struct jz_fifo_mode_data *fifo_data;

	channel=jz_tcu_dev->channel_num;

	switch(channel)
	{
		case 0:
			fifo_data = &jz_tcu_dev->tcu0;
			break;

		case 3:
			fifo_data = &jz_tcu_dev->tcu3;
			break;

		case 4:
			fifo_data = &jz_tcu_dev->tcu4;
			break;

		case 5:
			fifo_data = &jz_tcu_dev->tcu5;
			break;

		default:
			fifo_data = &jz_tcu_dev->tcu0;
			break;
	}

	REG_TCU_MOD(channel) = 0x0;

	__tcu_stop_counter(channel);

	if(fifo_data->fifo_mode)
	{
		REG_TCU_MOD(channel) = 0x0;
		memset(fifo_data,0,sizeof(struct jz_fifo_mode_data)-8);
	}
	__tcu_disable_pwm_output(channel);

	return 0;
}

static inline void pwm_set_pulse_width(unsigned int pulse_width)
{
	unsigned char channel;
	channel=jz_tcu_dev->channel_num;
	__tcu_set_half_data(channel, pulse_width);
	printk("Halfval:%d\n",__tcu_get_half_data(channel));
}

static inline void pwm_set_frequency(unsigned int max_counter)
{
	unsigned char channel;
	channel=jz_tcu_dev->channel_num;	
	__tcu_set_full_data(channel, max_counter + 1);
	printk("Fullval:%d\n",__tcu_get_full_data(channel));
}

static inline void pwm_select_mode(unsigned char mode)
{
	unsigned char channel;
	channel=jz_tcu_dev->channel_num;
	if(mode==1)
	{
		REG_TCU_MOD(channel) = 0x1;	
	}
	else
	{
		REG_TCU_MOD(channel) = 0x0;	
	}
}


static int tcu_open(struct inode *inode, struct file *filp);
static int tcu_release(struct inode *inode, struct file *filp);
static ssize_t tcu_read(struct file *filp, char *buf, size_t size, loff_t *l);
static ssize_t tcu_write(struct file *filp, const char *buf, size_t size, loff_t *l);
static int tcu_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

static struct file_operations tcu_fops = 
{
	.owner		= THIS_MODULE,
	.open 		= tcu_open,
	.release	= tcu_release,
	.read		= tcu_read,
	.write		= tcu_write,
	.ioctl		= tcu_ioctl,
};

static int tcu_open(struct inode *inode, struct file *filp)
{
#if 0
		printk("\nTMR.1:0x%x\n" ,REG_TCU_TMR);
		REG_TCU_TMSR=(0xffffffff);
		printk("\nTMR.2:0x%x\n" ,REG_TCU_TMR);
		printk("\nINTCM.1:0x%x\n",REG32(0xB0001004));
		REG32(0xB0001008) |= (0x7 << 25);
		printk("\nINTCM.2:0x%x\n",REG32(0xB0001004));
#endif
	printk("\ntcu_open ok!!!\n");
	return 0;
}



static int tcu_release(struct inode *inode, struct file *filp)
{
	printk("\ntcu_release ok!!!\n");
	return 0;
}

static ssize_t tcu_read(struct file *filp, char *buf, size_t size, loff_t *l)
{
	printk("\ntcu_read ok!!!\n");

	return size;
}

static ssize_t tcu_write(struct file *filp, const char *buf, size_t size, loff_t *l)
{
	unsigned int i,j;
	unsigned char *p0=NULL;
	unsigned int  *p1=NULL;
	unsigned char channel;
	struct jz_fifo_mode_data fifo_data;

	channel= jz_tcu_dev->channel_num;

	switch(channel)
	{
		case 0:
			fifo_data = jz_tcu_dev->tcu0;
			break;

		case 3:
			fifo_data = jz_tcu_dev->tcu3;
			break;

		case 4:
			fifo_data = jz_tcu_dev->tcu4;
			break;

		case 5:
			fifo_data = jz_tcu_dev->tcu5;
			break;

		default:
			fifo_data = jz_tcu_dev->tcu0;
			break;
	}

	i=size;
	p0=(void  *)kmalloc(4096,GFP_KERNEL);
	if(p0==NULL)
	{
		printk("\nmalloc failure!\n");
		return 0;
	}
	if(i<4096)
	{	
		if(copy_from_user(p0,buf,i))
			return -EFAULT;
	}
	else
		return -EINVAL;

	printk("\ntcu_write !!!!!!!!!\n");

	p1 = (unsigned int *)p0;

	if(((fifo_data.fifo_mode)&0x8) == 0x8)
	{
		j=((fifo_data.fifo_mode >> 4)&0x1f);
		REG_TCU_MOD(channel) |= (1<<1);
		printk("REG_TCU_ENTRY%d:0x%x",channel,ENTRY(channel));
		REG_TCU_MOD(channel) &= ~(1<<1);
		while(j--)
		{	while(((REG_TCU_TFIFOSR(channel) >>7)&0x1) == 0x1);
			printk("\nwrite data 0x%x\n",*p1);
			REG_TCU_TFWD(channel) = *(p1++);
		}
		printk("REG_TCU_TFWD%d:0x%x\n",channel,REG_TCU_TFWD(channel));
		REG_TCU_MOD(channel) = fifo_data.fifo_mode;
		printk("REG_TCU_ENTRY%d:0x%x\n",channel,ENTRY(channel));
		printk("REG_TCU_MOD%d:0x%x\n",channel,REG_TCU_MOD(channel));
	}
	else
	{	
		REG_TCU_MOD(channel) |= (1<<1);
		REG_TCU_MOD(channel) &= ~(1<<1);
		if(((REG_TCU_TFIFOSR(channel) >> 1)&(0x1f)) == 0x0)
		{

			if(i>16*4)
			{
				j=16;
				i=i-16*4;
			}
			else
			{
				j=i/4;
				i=0;
			}
			while(j--)
			{	while(((REG_TCU_TFIFOSR(channel))&0x1) == 0x1);
				printk("\nwrite data 0x%x\n",*p1);
				REG_TCU_TFWD(channel) = *(p1++);
			}
			REG_TCU_TFCR = 0xFFFFFFFF;
			REG_TCU_TMCR = (1 << (24 + (channel ? (channel - 2) : 0)));
			printk("REG_TCU_TFWD%d:0x%x\n",channel,REG_TCU_TFWD(channel));
			printk("REG_TCU_ENTRY%d:0x%x\n",channel,ENTRY(channel));
			printk("REG_TCU_MOD%d:0x%x\n",channel,REG_TCU_MOD(channel));
			p0 = (unsigned char *)p1;
		}
		if((fifo_data.buf1_used == 0)&&i)
		{
			memcpy(fifo_data.buf1,p0,i);
			fifo_data.buf1_used = 1;
			fifo_data.sum_count0=i;
			fifo_data.using_count0=0;
			if(fifo_data.buf_using != 2)
				fifo_data.buf_using = 1;
			printk("\nwrite buf1: %d\n",i);
		}
		else if((fifo_data.buf2_used == 0)&&i)
		{
			memcpy(fifo_data.buf2,p0,i);
			fifo_data.buf2_used = 1;
			fifo_data.sum_count1=i;
			fifo_data.using_count1=0;
			printk("\nwrite buf2\n");
		}
		else
		{
			printk("\nwrite buf wrong\n");
			return -EINVAL;
		}
	}

	printk("\ntcu write data ok\n");
	kfree(p0);
	return size;
}

static void set_pwm_fifo_mode(unsigned int  mode)
{
	unsigned char channel;
	struct jz_fifo_mode_data *fifo_data;

	channel= jz_tcu_dev->channel_num;

	switch(channel)
	{
		case 0:
			fifo_data = &jz_tcu_dev->tcu0;
			break;

		case 3:
			fifo_data = &jz_tcu_dev->tcu3;
			break;

		case 4:
			fifo_data = &jz_tcu_dev->tcu4;
			break;

		case 5:
			fifo_data = &jz_tcu_dev->tcu5;
			break;

		default:
			fifo_data = &jz_tcu_dev->tcu0;
			break;
	}

	REG_TCU_MOD(channel) = mode;
	fifo_data->fifo_mode = mode;
}

static void print_reg(void)
{
	printk("\nTCSR4:0X%x\n",REG_TCU_TCSR4);
	printk("\nTCNT4:0X%x\n",REG_TCU_TCNT4);
	printk("\nTER:0X%x\n" ,REG_TCU_TER);
	printk("\nTFR:0X%x\n" ,REG_TCU_TFR);
	printk("\nTMR:0X%x\n" ,REG_TCU_TMR);
	printk("\nTSR:0X%x\n" ,REG_TCU_TSR);
	printk("\nTSTR:0X%x\n",REG_TCU_TSTR);
	printk("\nTFIFOSR4:0x%x\n",REG32(0X0B0002128));

}

static void get_count(unsigned int total)
{
	unsigned char channel = jz_tcu_dev->channel_num;
	unsigned int value;
	unsigned int loop;

	for(loop = 0; loop < total; loop++){
		do{
			printk("Reading...No.%d\n",loop);	
			value = __tcu_get_count(channel);
		}while(__tcu_read_real_value(channel) != 1);
		printk("T.V:%d\n",value);	
	}
}

static void tcu2_start(void)
{
	unsigned char channel = jz_tcu_dev->channel_num;

	/* Setup GPIO port */
	set_gpio_as_pwm(channel);

	/* Set Oscillator and Power Control Register (OPCR) */
	printk("OPCR.1:0x%x\n",REG32(0xB0000024));
	REG32(0xB0000024) |= 0x4;
	printk("OPCR.2:0x%x\n",REG32(0xB0000024));

	/* Initial state */
	__tcu_select_clk_div1(channel);
	__tcu_disable_pwm_output(channel);
	__tcu_stop_counter(channel);

	/* Reset */
	__tcu_select_pclk(channel);
	__tcu_clear_counter_to_zero(channel);
	__tcu_disable_all_clk(channel);

	/*  Initial the configuration */
	__tcu_set_half_data(channel,0x0800);
	__tcu_set_full_data(channel,0x0f00);
	__tcu_init_pwm_output_low(channel);
	__tcu_select_clk_div64(channel);
	__tcu_select_rtcclk(channel);
	__tcu_enable_pwm_output(channel);

	/* Start the counter */
	__tcu_start_counter(channel);
}

static void tcu2_stop(void)
{
	unsigned char channel = jz_tcu_dev->channel_num;
	__tcu_disable_pwm_output(channel);
}

static int tcu_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned int tmp=arg;
	switch (cmd) {
		case 0 : start_pwm_output();
			 printk("\nstart the pwm\n");
			 break;
		case 1: stop_pwm_output();
			printk("\nstop the pwm\n");
			break;
		case 2: pwm_set_pulse_width(tmp);
			break;
		case 3: pwm_set_frequency(tmp);
			break;
		case 4: if(tmp<8)
			{
				jz_tcu_dev->channel_num=tmp;

				printk("\nset the channel :%d\n",tmp);
			}	       
			break;
		case 5: set_pwm_fifo_mode(tmp);
			printk("\nset the pwm fifo mode\n");
			break;
		case 6: pwm_select_mode(tmp);
			break;
		case 7: print_reg();
			break;
		case 8: start_pwm_input();
			break;
		case 9: stop_pwm_input();
			break;
		case 10: start_pwm_bypass();
			break;
		case 11: select_div(tmp);
			break;
		case 12: select_clk(tmp);
			break;
		case 13: get_count(tmp);
			break;
		case 14: tcu2_start();
			break;
		case 15: tcu2_stop();

		default:
			 printk("Not supported command: 0x%x\n", cmd);
			 return -EINVAL;
			 break;
	}
	return 0;
}

#define dev_minor 131

static struct miscdevice tcu_dev1 =
{
	dev_minor,
	"TcuTest",
	&tcu_fops,
};

static irqreturn_t tcu_interrupt(int irq, void * dev_id)
{
	unsigned int i_flags,i_mask;
	unsigned char channel;
	struct jz_fifo_mode_data *fifo_data;

	channel=jz_tcu_dev->channel_num;

	switch(channel)
	{
		case 0:
			fifo_data = &jz_tcu_dev->tcu0;
			break;

		case 3:
			fifo_data = &jz_tcu_dev->tcu3;
			break;

		case 4:
			fifo_data = &jz_tcu_dev->tcu4;
			break;

		case 5:
			fifo_data = &jz_tcu_dev->tcu5;
			break;

		default:
			fifo_data = &jz_tcu_dev->tcu0;
			break;
	}

	i_flags = REG_TCU_TFR;	
	i_mask  = REG_TCU_TMR;
	i_flags = (i_flags & (~i_mask));
	REG_TCU_TMSR=(1 << (24 + (channel ? (channel - 2) : 0)));

	if(FIFO_CHANNEL(channel) && ((REG_TCU_MOD(channel) & 0x1) == 0x1) && (((i_flags>>(8 + (channel ? (channel - 2) : 0)))&0x1)==0x1))
	{
		REG_TCU_TFCR = (0x1<<(8 + (channel ? (channel - 2) : 0)));
		printk("\nInterrupt No.%d bit\n",(8 + (channel ? (channel - 2) : 0)));
		printk("\nTFWD%d:0x%x\n",channel,REG_TCU_TFWD(channel));

	}
	else if ((!FIFO_CHANNEL(channel)) && (((i_flags>>channel)&0x1)==0x1))
	{
		REG_TCU_TFCR = (0x1<<channel);
		printk("\nInterrupt %d\n",channel);	

	}
	else if(FIFO_CHANNEL(channel) && ((REG_TCU_MOD(channel) & 0x1) == 0x1) && ((i_flags >> (24 + (channel ? (channel - 2) : 0)) & 0x1) == 0x1))
	{
		printk("\ninterrupt bit26\n");
		REG_TCU_TFCR = (0x1<<(24 + (channel ? (channel - 2) : 0)));
		while(((REG_TCU_TFIFOSR(channel) >> 1)&0x1f) < 0x10)
		{
			if((fifo_data->buf_using == 1)&&(fifo_data->buf1_used == 1))
			{
				if((fifo_data->using_count0) < (fifo_data->sum_count0))
				{
					while(((REG_TCU_TFIFOSR(channel))&0x1) == 0x1);
					REG_TCU_TFWD(channel) = *(unsigned int *)(fifo_data->buf1 + fifo_data->using_count0);
					fifo_data->using_count0 +=4;
					printk("\n Here.01\n");
				}
				else
				{
					printk("\nbuf1 is  null \n");
					fifo_data->sum_count0 = 0;
					fifo_data->using_count0 = 0;
					fifo_data->buf1_used = 0;
					if(fifo_data->buf2_used == 1)
						fifo_data->buf_using =2;
					else
						fifo_data->buf_using =0;
					printk("\n Here.02\n");
				}
			}
			else if((fifo_data->buf_using == 2)&&(fifo_data->buf2_used == 1))
			{
				if(fifo_data->using_count1 < fifo_data->sum_count1)
				{
					while(((REG_TCU_TFIFOSR(channel)) & 0x1) == 0x1);
					REG_TCU_TFWD(channel) =* (unsigned int *)(fifo_data->buf2 + fifo_data->using_count1);
					fifo_data->using_count1 +=4;
				}
				else
				{
					fifo_data->sum_count1 = 0;
					fifo_data->using_count1 = 0;
					fifo_data->buf2_used = 0;
					if(fifo_data->buf1_used == 1)
						fifo_data->buf_using =1;
					else
						fifo_data->buf_using =0;
				}
			}
			else
			{
				printk("\nData write to fifo over!!!!\n");
				if((fifo_data->buf1_used == 0)&&(fifo_data->buf2_used == 0))
				{
					printk("\nBreak out!!!\n");
					goto write_data_over;
				}

				goto flags1;
			}
		}
	}
	else
	{
		printk("\nNot interrupt:0x %x\n",i_flags);
	}

	printk("\nTCU interrupt \n");

flags1:
	REG_TCU_TMCR=(1 << (24 + (channel ? (channel - 2) : 0)));
write_data_over:
	return IRQ_HANDLED;
}

/* NOTE: We should save current regs.This is just demo route. */
static int jz_tcu_suspend(struct platform_device *pdev,pm_message_t state)
{
	printk("\nTCU Sleep ...\n");
	printk("Set Ch1 Output RTC Clock.\n");
	jz_tcu_dev->channel_num = 1;
	start_pwm_bypass();	
	select_clk(1);
	printk("Set Ch2 Output RTC Clock.\n");
	jz_tcu_dev->channel_num = 2;
	start_pwm_bypass();	
	select_clk(1);
	printk("Done.\n\n");

	return 0;
}

/* NOTE: We should restore saved regs.This is just demo route. */
static int jz_tcu_resume(struct platform_device *pdev)
{
	printk("\nTCU Wakeup ...\n");
	printk("Done.\n\n");
	return 0;
}

static struct platform_driver jz_tcu_driver = {
	.suspend = jz_tcu_suspend,
	.resume = jz_tcu_resume,
	.driver = {
		.owner = THIS_MODULE,
		.name = "JZ-TCU",
	},
};

static int __init tcu_init(void)
{
	int error;
	unsigned char *p0 = NULL;

	jz_tcu_dev = kmalloc(sizeof(struct tcu_device), GFP_KERNEL);
	if(jz_tcu_dev == NULL)
	{
		printk("Alloc tcu_device failure!!\n");
		goto err_free_irq;
	}
	memset(&(jz_tcu_dev->tcu4),0,sizeof(struct jz_fifo_mode_data));

	misc_register(&tcu_dev1);
	jz_tcu_dev ->tcu_dev=&tcu_dev1;
	error = request_irq(IRQ_TCU2, tcu_interrupt, IRQF_DISABLED, TCU_NAME, NULL);
	if (error) 
	{
		printk("unable to get PenDown IRQ %d\n", IRQ_TCU2);
		goto err_free_irq;
	}

	p0 = (unsigned char *)__get_free_pages(GFP_KERNEL,3);
	if(p0 == NULL)
	{
		goto err_free_mem;
	}
	memset(p0,0,PAGE_SIZE*8);

	jz_tcu_dev->tcu0.buf1 = (p0+0);
	jz_tcu_dev->tcu0.buf2 = (p0+PAGE_SIZE);
	jz_tcu_dev->tcu3.buf1 = (p0+(PAGE_SIZE*2));
	jz_tcu_dev->tcu3.buf2 = (p0+(PAGE_SIZE*3));
	jz_tcu_dev->tcu4.buf1 = (p0+(PAGE_SIZE*4));
	jz_tcu_dev->tcu4.buf2 = (p0+(PAGE_SIZE*5));
	jz_tcu_dev->tcu5.buf1 = (p0+(PAGE_SIZE*6));
	jz_tcu_dev->tcu5.buf2 = (p0+(PAGE_SIZE*7));

	platform_driver_register(&jz_tcu_driver);
	
	printk("\n\n\n\nJZ_SOC_NAME: JZ4770 TCU driver registered.\n\n\n");
	return 0;

err_free_irq:
	free_irq(IRQ_TCU2,NULL);

	return 0;

err_free_mem:
	kfree(p0);
	return 0;
}

static void __exit tcu_exit(void)
{
	platform_driver_unregister(&jz_tcu_driver);
	misc_deregister(&tcu_dev1);	
}

module_init(tcu_init);
module_exit(tcu_exit);
