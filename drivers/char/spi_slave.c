#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/completion.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/jzsoc.h>
#include "spi_slave.h"

#define  DRIVER_NAME    "jz_spi_slave"

static int MAJOR_NR = 0;               /*  Driver Major Number */
static int MINOR_NR = 0;               //次设备起始号

struct class *ssi_class;

static struct semaphore readable;      //定义信号量结构体,用于进程间读的互斥同步
static struct semaphore writeable;     //定义信号量结构体,用于进程间写的互斥同步

unsigned char *buffer_read;
unsigned char *buffer_write;

static int exit_flag = 0;
static int irq_num,count_write,count_read;
static int write_count,read_count;     //用于接收读写数据的大小

/* GPIO 口的定义*/
#define REQ_SEND	(32 * 4 + 18)
#define RCV_REQ		(32 * 1 + 29)
#define CS		(32 * 1 + 26)

#define SDO		(32 * 1 + 27)
#define SCLK		(32 * 1 + 28)
#define SDI		(32 * 1 + 30)
#define RESP_SEND	(32 * 1 + 31)
#define TP3		(32 * 5 + 12)

#define IRQ_SCLK IRQ_GPIO_0 + SCLK

//=============================================================
//语法格式：    SSI_Init(void)
//实现功能：    SSI初始化，配置io口、寄存器
//参数：        无
//返回值：      无
//=============================================================
static void ssi_io_init(void)                                                                                                  
{
	/* 配置各个端口的输入输出 */
	__gpio_as_output(REQ_SEND);         // 配置为输出端口，用于通知TM89有数据要发送
	__gpio_as_input(RCV_REQ);
	
	__gpio_as_input(CS);
	__gpio_disable_pull(CS);
	__gpio_as_input(SDO);               // 配置为输入端口，用于接收TM89的数据
	
	__gpio_as_irq_rise_edge(SCLK);
	
	__gpio_as_output(SDI);              // 配置为输出端口，同于向TM89发送数据
	__gpio_as_input(RESP_SEND);         // 配置为输入端口，用于接收TM89的响应

	__gpio_clear_pin(REQ_SEND);
}
void Tcu_Init(void)
{
	__tcu_select_extalclk(3);       /* channel 4: ext clock */
	__tcu_select_clk_div16(3);      /* set clk division  , channel 4 clock is 1.5M */
	__tcu_set_half_data(3,0x384);   /* half data max: 0x12c */ 

	REG_TCU_TCSR(3) |= (1 << 9);    /* Abrupt shutdown */
	__tcu_clear_half_match_flag(3);
}
void Tcu_Start(void)
{
	__tcu_set_count(3,0) ;			/* clear counter */
	REG_TCU_TESR |= (1 << 3);       // TCU 开始计时 
}
void Tcu_Stop(void)
{
	__tcu_stop_counter(3);
	__tcu_clear_half_match_flag(3);
}
int mystrlen(unsigned char *s){
	int i = 0,j = 0;
	while(s[read_count - j] == '\0'){
		j++;
	}
	i = read_count - j + 1;
	return i;
}
/* 
 *从设备使能
 *enable : 为1时，使能信号有效，CS低电平
 *为0时，使能信号无效，CS为高电平
 * */
#if 0
void cs_enable(int enable)
{
	if(enable)
		__gpio_clear_pin(CS);      //cs低电平，从设备使能有效
	else
		__gpio_set_pin(CS);        //cs高电平，从设备使能无效
}
#endif

/* spi 字节写 */
void spi_write_byte(unsigned char data)
{
	int i;
	unsigned long timeout = jiffies + 2 * HZ/10000;
	
	for(i = 7; i >= 0; i--){
		// timeout, 5K, 200us
		while((!(__gpio_get_pin(SCLK))) && (!time_after(jiffies,timeout)));
#if 0
		Tcu_Start();
		while((!(__gpio_get_pin(SCLK))) && (!(__tcu_half_match_flag(3))));
		udelay(10);
		while((!(__gpio_get_pin(SCLK))) && (!(__tcu_half_match_flag(3))));
		if(__tcu_half_match_flag(3)){
			printk("Tcu Match Flag write error1\n");
			printk("count write is %d\n",count_write);
			Tcu_Stop();
			return;
		}
		Tcu_Stop();
#endif
		if(data & (1 << i)){						//从高位7到低位0串行写入     
			__gpio_set_pin(SDI);
		}
		else{
			__gpio_clear_pin(SDI);
		}
		while(__gpio_get_pin(SCLK) && (!time_after(jiffies,timeout)));

#if 0
		Tcu_Start();
		while(__gpio_get_pin(SCLK) && (!(__tcu_half_match_flag(3))));
		udelay(10);
		while(__gpio_get_pin(SCLK) && (!(__tcu_half_match_flag(3))));
		if(__tcu_half_match_flag(3)){
			printk("Tcu Match Flag write error2\n");
			printk("count write is %d\n",count_write);
			Tcu_Stop();
			return;
		}
		Tcu_Stop();
#endif
	}
	count_write++;
}

/* spi 字节读 */
static unsigned char spi_read_byte(void)
{
	//printk("%s, %d\n",__func__,__LINE__);
	int i;
	unsigned char r = 0;
	//unsigned long timeout = jiffies + 2 * HZ/10000;

	for(i = 0; i < 8; i++){
#if 0
		while((!(__gpio_get_pin(SCLK))) && (!time_after(jiffies,timeout)));
		//udelay(10);
		while(__gpio_get_pin(SCLK) && (!time_after(jiffies,timeout)));
		//udelay(10);
		//while(__gpio_get_pin(SCLK) && (!time_after(jiffies,timeout)));
#endif
#if 1
		Tcu_Start();
		while((!(__gpio_get_pin(SCLK))) && (!(__tcu_half_match_flag(3))));
		udelay(10);
		while((!(__gpio_get_pin(SCLK))) && (!(__tcu_half_match_flag(3))));
		if(__tcu_half_match_flag(3)){
			printk("Tcu Match Flag read error1\n");
			printk("count read is %d\n",count_read);
			Tcu_Stop();
			exit_flag = 1;
			return 0;
		}
		Tcu_Stop();
#endif
#if 1
		Tcu_Start();
		while(__gpio_get_pin(SCLK) && (!(__tcu_half_match_flag(3))));
		udelay(10);
		while(__gpio_get_pin(SCLK) && (!(__tcu_half_match_flag(3))));
		if(__tcu_half_match_flag(3)){
			printk("Tcu Match Flag read error2\n");
			printk("count read is %d\n",count_read);
			Tcu_Stop();
			exit_flag = 1;
			break;
		}
		Tcu_Stop();
#endif
		r = (r << 1) | __gpio_get_pin(SDO);  //从高位7到低位0进行串行读
	}
	count_read++;

	return r;
}

/*
 *spi  写操作
 *buf：写缓冲区
 *count：写入字节的长度
 * */
static ssize_t jz_ssi_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	static int ret;
	ssize_t write_size = count;
	//unsigned long timeout = jiffies + 100 * HZ;
	//printk("Wait for Master Device RECV....\n");
	printk("Waiting for Device to Transmit Data....\n");
	buffer_write = (unsigned char *)kzalloc(write_count * sizeof(unsigned char),GFP_KERNEL);
	if(!buffer_write){
		return -ENOMEM;
	}
	//__gpio_set_pin(REQ_SEND);			//通知TM89，有数据要发送
	/*开始写入数据*/
	ret=copy_from_user(buffer_write, buf, write_count);
	if(ret != 0){
		printk("copy from user_write buffer error\n");
		kfree(buffer_write);
		return 0;
	}
	__gpio_set_pin(REQ_SEND);			//通知TM89，有数据要发送
	//if(down_interruptible(&writeable)!=0){      //等待触发中断
	ret = down_timeout(&writeable,5 * HZ);
	if(ret == -ETIME){      //等待触发中断
    	printk("Time Out is happen\n");
		return 0;
	}
	kfree(buffer_write);
	//printk("Slave Device Send Finish!\n");
	return write_size;
}

/*
 *spi读操作
 *buf：读缓冲区
 *count：读入字节的长度
 * */
ssize_t jz_ssi_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	//printk("%s, %d\n",__func__,__LINE__);
	int ret;
	ssize_t read_size = count;
	//unsigned long timeout = jiffies + 100 * HZ;
	//printk("Wait for Master Device Send....\n");
	buffer_read = (unsigned char *)kzalloc(read_count * sizeof(unsigned char),GFP_KERNEL);
	if(!buffer_read){
		return -ENOMEM;
	}
	/*开始读入数据*/
	//if(down_interruptible(&readable)!=0){      //等待触发中断
	ret = down_timeout(&readable,5 * HZ);
	if(ret == -ETIME){      //等待触发中断
    	printk("Time Out is happen\n");
		return 0;
	}
	ret=copy_to_user(buf, buffer_read, read_count);
	if(ret != 0){
		printk("copy to user_read buffer error\n");
		kfree(buffer_read);
		return 0;
	}
	kfree(buffer_read);
	//printk("Slave Device Recv Finish!\n");
	printk("Complete the data Transmission equipment!!\n");
	if(ret == 0)
		return read_size;
	else
		return ret;
}

/*  中断服务程序 */
static irqreturn_t ssi_interrupt(int irq,void *dev_id)
{
	static int i = 0, j = 0;

	disable_irq_nosync(IRQ_SCLK);
	/* 开始写入数据*/
	if(__gpio_get_pin(RCV_REQ) && (!(__gpio_get_pin(CS)))){
		switch(irq_num)	
		{
			case 1:
				for(i = 0; i < write_count; i++){
					spi_write_byte(buffer_write[i]);
				}
				__gpio_clear_pin(REQ_SEND);
				up(&writeable);
				break;
			case 2:
				spi_write_byte(buffer_write[i]);
				i++;
				if(i == write_count){
					__gpio_clear_pin(REQ_SEND);
					up(&writeable);
					i = 0;
				}
				break;
			default:
				return -ENOTTY;
		} 
	}
	else if((__gpio_get_pin(REQ_SEND)) && (!(__gpio_get_pin(RCV_REQ)))){
			printk("===>RCV_REQ is Low--No Ack<===\n");
			__gpio_clear_pin(REQ_SEND);
			return 0;
	}
	else if((__gpio_get_pin(CS)) && (!(__gpio_get_pin(RESP_SEND)))){
		printk("===>CS is High<===\n");
		return 0;
	}
	/* 开始读入数据*/
	else if((!(__gpio_get_pin(CS))) && (__gpio_get_pin(RESP_SEND))){
#if 1
		switch(irq_num)
		{
			case 1:
				for(j = 0; j < 12; j++ ){
					buffer_read[j] = spi_read_byte();
					if(j == 2)
						read_count = buffer_read[1] + 2;
					if((__gpio_get_pin(CS)) && (!(__gpio_get_pin(RESP_SEND)))){
						exit_flag = 0;
						break;
					}
					if(exit_flag == 1){
						exit_flag = 0;
						break;
					}
				}
				up(&readable);
				break;
			case 2:
				buffer_read[j] = spi_read_byte();
				j++;
				if(j == 2)
					read_count = buffer_read[1] + 2;
				if((__gpio_get_pin(CS)) && (!(__gpio_get_pin(RESP_SEND)))){
					exit_flag = 0;
					up(&readable);
					j = 0;
				}   
				if(exit_flag == 1){
					exit_flag = 0;
					up(&readable);
					j = 0;
				}
				break;
			default:
				return -ENOTTY;
		}
#endif
#if 0
		switch(irq_num)
		{
			case 1:
				for(j = 0; j < read_count; j++ ){
					buffer_read[j] = spi_read_byte();
					if(j == 2)
						read_count = buffer_read[1] + 2;
				}
				up(&readable);
				break;
			case 2:
				buffer_read[j] = spi_read_byte();
				j++;
				if(j == 2)
					read_count = buffer_read[1] + 2;
				if(j == read_count){
					up(&readable);
					j = 0;
				}   
				break;
			default:
				return -ENOTTY;
		}
#endif
	}
	else {
		printk("====> ??! <====\n");
		BUG_ON(1);
	}
	REG_GPIO_PXFLGC(SCLK / 32) = 1 << (SCLK  % 32);
	enable_irq(IRQ_SCLK);
	return IRQ_HANDLED;
}

static int jz_ssi_open(struct inode *inode,struct file *filp)                                                         
{
	int ret;
	//printk("%s, %d\n",__func__,__LINE__);
	//ssi_io_init();
	/*设置外部中断*/
	ret = set_irq_type(IRQ_SCLK,IRQ_TYPE_EDGE_RISING);  //设置使能，上升沿触发，有数据要读写
	if(ret != 0){
		printk("irq request failed!\n");
		return ret;
	}

	ret = request_irq(IRQ_SCLK,&ssi_interrupt,IRQF_DISABLED,"gpio_sim_ssi", (void *)1);    //申请中断
	if(ret != 0){
		printk("\nirq request failed %d\n",ret);
		return ret;
	}
	Tcu_Init();
	
	return 0;
}

static int jz_ssi_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret,len;
	switch(cmd)
	{   
		case JZ_SPI_READ:
			__gpio_as_input(CS);
			__gpio_as_input(RESP_SEND);
			__gpio_as_input(SDO);
			break;
		case JZ_SPI_WRITE:
			__gpio_clear_pin(REQ_SEND);
			__gpio_clear_pin(SDI);
			//__gpio_set_pin(REQ_SEND);
			break;
		case RET_BUFFER_LEN:
			len = mystrlen(buffer_read);
			ret = copy_to_user((unsigned int *)arg, &len, 4);
			count_read = 0;
			break;
		case CLR_WRITE_BUFFER:
			memset(buffer_write, 0, sizeof(buffer_write));	
			break;
		case CLR_READ_BUFFER:
			memset(buffer_read, 0, sizeof(buffer_read));	
			break;
		case WRITE_BUFFER_LEN:
			ret = copy_from_user(&write_count, (void __user *)arg, 4);
			break;
		case READ_BUFFER_LEN:
			ret = copy_from_user(&read_count, (unsigned int *)arg, 4);
			break;
		case IRQ_NUM:
			ret = copy_from_user(&irq_num, (unsigned int *)arg, 4);
			break;
		default:
			return -ENOTTY;
	}

	return 0;
}

static int jz_ssi_release(struct inode *inode,struct file *filp)
{
	free_irq(IRQ_SCLK,(void *)1);     //释放外部中断
	return 0;
}

/*  Driver Operation structure */
static const struct file_operations SSI_fops = {
	.owner = THIS_MODULE,
	.read = jz_ssi_read,
	.write = jz_ssi_write,
	.open = jz_ssi_open,
    .ioctl = jz_ssi_ioctl,
	.release = jz_ssi_release,
};

static int __init ssi_init(void)
{
	//printk("GPIO_SIM_SPI_Driver_init\n");
	
	sema_init(&readable,0);
	sema_init(&writeable,0);

	MAJOR_NR = register_chrdev(MAJOR_NR, DRIVER_NAME, &SSI_fops);
	
	if(MAJOR_NR < 0)     
	{  
		printk("register char device fail!\n");
		return MAJOR_NR; 
	}
	
	ssi_class = class_create(THIS_MODULE,"ssi_slave_driver");
	device_create(ssi_class,NULL, MKDEV(MAJOR_NR, MINOR_NR), NULL,DRIVER_NAME);
			       
	ssi_io_init();
	printk("SPI MyDriver OK! Major = %d\n", MAJOR_NR);
	
	return 0;
}

static void __exit ssi_exit(void)
{
	/*  Module exit code */
	printk("exit in ssi_driver\n");
	
	disable_irq(IRQ_SCLK);
	free_irq(IRQ_SCLK,(void *)1);
	
	/*  Driver unregister */
	if(MAJOR_NR > 0)
	{
		unregister_chrdev(MAJOR_NR, DRIVER_NAME);
		device_destroy(ssi_class,MKDEV(MAJOR_NR, MINOR_NR));
		class_destroy(ssi_class);
		printk("ssi_Module_exit ok\n");
	}
	
	return;
}

MODULE_AUTHOR("lsun@ingenic.cn");
MODULE_LICENSE("Dual BSD/GPL");

module_init(ssi_init);
module_exit(ssi_exit);
