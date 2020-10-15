/* linux/drivers/char/jzchar/jz_spi/jz_spi_sys.c
 *
 * SPI test device driver for Jz47xx SSI controller;
 *
 * Copyright (c) 2006 Ingenic
 * Shumb <sbhuang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/jzsoc.h> 

#include <linux/list.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/spi/spidev.h>

#include <linux/dma-mapping.h>

#include "spi_jz47xx.h"
#include "jz_spidev_test.h"

static SPI_STATE_STRUCT spi_state;

#if defined(CONFIG_SOC_JZ4750)
#define INTC_SSI_UNMASK(n)	((n) ?__intc_unmask_irq(IRQ_SSI1):__intc_unmask_irq(IRQ_SSI0))
#define INTC_SSI_MASK(n)	((n) ?__intc_mask_irq(IRQ_SSI1):__intc_mask_irq(IRQ_SSI0))
#define CPM_SSI_START(n) 	((n) ? __cpm_start_ssi(1):__cpm_start_ssi(0))
#define CPM_SSI_STOP(n)		((n) ? __cpm_stop_ssi(1):__cpm_stop_ssi(0))
#define __REG_INTC_IMR()		REG_INTC_IMR

#elif defined(CONFIG_SOC_JZ4750D)
#define INTC_SSI_UNMASK(n)	(__intc_unmask_irq(IRQ_SSI))
#define INTC_SSI_MASK(n)	(__intc_mask_irq(IRQ_SSI))
#define CPM_SSI_START(n) 	((n) ? :__cpm_start_ssi(0))
#define CPM_SSI_STOP(n)		((n) ? :__cpm_stop_ssi(0))
#define __REG_INTC_IMR()		REG_INTC_IMR

#else
#define INTC_SSI_UNMASK(n)	((n) ?__intc_unmask_irq(IRQ_SSI1):__intc_unmask_irq(IRQ_SSI0))
#define INTC_SSI_MASK(n)	((n) ?__intc_mask_irq(IRQ_SSI1):__intc_mask_irq(IRQ_SSI0))
#define CPM_SSI_START(n) 	((n) ?cpm_start_clock(CGM_SSI1):cpm_start_clock(CGM_SSI0))
#define CPM_SSI_STOP(n)		((n) ?cpm_stop_clock(CGM_SSI1):cpm_stop_clock(CGM_SSI0))
extern unsigned int cpm_get_clock(cgu_clock);
#define __cpm_get_pllout() 	cpm_get_clock(CGU_CCLK)
#define __cpm_get_cclk() 	cpm_get_clock(CGU_CCLK)
#define __cpm_get_hclk() 	cpm_get_clock(CGU_HCLK)
#define __cpm_get_pclk() 	cpm_get_clock(CGU_PCLK)
#define __cpm_get_mclk() 	cpm_get_clock(CGU_MCLK)
#define REG_CPM_CPPCR		REG_CPM_CPPCR0
#define __REG_INTC_IMR()		REG_INTC_IMR(0)

#endif

struct spidev_data {
	struct device		dev;
	struct spi_device	*spi;
	struct list_head	device_entry;

	struct mutex		buf_lock;
	unsigned		users;
	u8			*buffer;
	u8			*name;
};
static struct class *spitest_class;

extern int jz_spi_pinconfig(struct jz47xx_spi *hw);

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

struct spidev_data *pspidev;

#define __ssi0_reset()           \
do{				     \
	REG_SSI_CR0(0) = 0x0000;	\
	REG_SSI_CR1(0) = 0x00007960;	\
	REG_SSI_SR(0) = 0x00000098;	\
	REG_SSI_ITR(0) = 0x0000;	\
	REG_SSI_ICR(0) = 0x00;	\
	REG_SSI_GR(0) = 0x0000;	\
	REG_SSI_CR0(0) |= SSI_CR0_TFLUSH;		\
	REG_SSI_CR0(0) |= SSI_CR0_RFLUSH;		\
}while(0)
 void uspi_base_init(void )
{
	/* set GPIO */
	/* If use wait mode, REG_SSI_GR must not be 0 */
	/* Extal Clock */
        __ssi0_reset();
        __gpio_as_ssi0();
	__cpm_set_ssidiv(1);	/* ssi source clk = 360 /(n+1) = 180 MHz */
	REG_SSI_GR(0) = 25;         //ÉèÖÃssiµÄ Ê±ÖÓ
	//CPM_SSI_START(0);
	CPM_SSI_START(0);
	
        __ssi_disable(0);
	__ssi_disable_tx_intr(0);
	__ssi_disable_rx_intr(0);
	__ssi_enable_receive(0);
	
	__ssi_flush_fifo(0);
 	__ssi_clear_errors(0);
	__ssi_set_spi_clock_phase(0,0);
	__ssi_set_spi_clock_polarity(0,0);  
	__ssi_set_msb(0);
	
	__ssi_set_frame_length(0,8);
	__ssi_disable_loopback(0);   
	//__ssi_select_ce(0);
        REG_SSI_CR0(0) &= ~SSI_CR0_FSEL;
	__ssi_set_tx_trigger(0,112);
	__ssi_set_rx_trigger(0,16);  
	
	__ssi_spi_format(0);
	__ssi_enable(0);
//	__cpm_start_dmac();

}

void spi_write_bits(unsigned char chnl,unsigned int data)
{
//	while(__ssi_is_busy(chnl))printk("=");
	__ssi_transmit_data(chnl, data);
	__ssi_clear_errors(chnl);
//	while(!(__ssi_transfer_end(chnl)));
}
unsigned int spi_read_bits(unsigned char chnl)
{
//	__ssi_clear_overrun(chnl);
//	while(__ssi_is_busy(chnl));
	return __ssi_receive_data(chnl);
}
int gpio_config(unsigned int pin,unsigned char level)
{
	__gpio_as_output(pin);
	if(level)
		__gpio_set_pin(pin);	
	else
		__gpio_clear_pin(pin);

	return __gpio_get_pin(pin);	
}
int pin_config(unsigned char chnl,unsigned char pin,unsigned int val)
{
	unsigned int n;
	
	if(val == 0)
	{
		__gpio_as_ssi0();
	}
	else
	{
		if(chnl == 0)
		{
			switch(pin)
			{
				case PIN_SSI_CLK:
					n = (32*1+26);
					break;
				case PIN_SSI_CE:
					n = (32*1+29);
					break;
				case PIN_SSI_CE2:
					n = (32*1+31);
					break;
				case PIN_SSI_GPC:
					n = (32*1+30);
					break;
				case PIN_SSI_DT:
					n = (32*1+27);
					break;
				case PIN_SSI_DR:
					n = (32*1+28);
					break;
				default:
					n = (32*1+29);
					break;
			}
		}else if(chnl == 1){
			switch(pin)
			{
				case PIN_SSI_CLK:
					n = (32*3+26);
					break;
				case PIN_SSI_CE:
					n = (32*3+29);
					break;
				case PIN_SSI_CE2:
					n = (32*3+30);
					break;
				case PIN_SSI_GPC:
					n = (32*3+30);
					break;
				case PIN_SSI_DT:
					n = (32*3+27);
					break;
				case PIN_SSI_DR:
					n = (32*3+28);
					break;
				default:
					n = (32*3+29);
					break;
			}
		
		}else{
			printk("channel %d don't support\n",chnl);
			return -1;
		}
		
		if(val & 0x1)
			__gpio_set_pin(n);
		else
			__gpio_clear_pin(n);
		
		__gpio_as_output(n);
	}
	return 0;
}
int read_via_spisystem(struct spi_device *spi, u8 *buf, size_t len)
{
	return spi_read(spi,buf,len);

}
int write_via_spisystem(struct spi_device *spi, u8 *buf, size_t len)
{
	return	spi_write(spi,buf,len);
}
void spi_complete_loop(void *arg)
{
	printk("spi_transfer done !!!\n");
}
int write_via_spisystem_loop(struct spi_device *spi, const u8 *w_buf,u8 *r_buf, size_t len)
{
	u32 status;
	struct spi_transfer	t = {
			.tx_buf		= w_buf,
			.rx_buf		= r_buf,
			.len		= len,
//			.cs_change  = 1,
		};

	struct spi_message	m;

//	m.complete = spi_complete_loop;
	
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	printk("tx_buf:0x%08X rx_buf:0x%08X\ntx_dma:0x%08X rx_dma:0x%08X\n",
		(u32)t.tx_buf,(u32)t.rx_buf,t.tx_dma,t.rx_dma);
	printk("m.is_dma_mapped = %d\n\n",m.is_dma_mapped);
	printk("------------------come to SSI driver\n");
	status=spi_sync(spi, &m);
	printk("------------------SPI done.\n");
	printk("status = %d\n",status);
//	return spi_async(spi, &m);
	return 0;
}
int write_via_dma_loop(struct spi_device *spi, dma_addr_t dma_tx,dma_addr_t dma_rx, size_t len)
{
	u32 status;
	struct spi_transfer	t = {
	//		.tx_buf		= 0,
	//		.rx_buf		= 0,
			.tx_dma		= dma_tx,
			.rx_dma		= dma_rx,
			.len		= len,
//			.cs_change  = 1,
		};

	struct spi_message	m;

	t.tx_buf 		= phys_to_virt(dma_tx);
	t.rx_buf 		= phys_to_virt(dma_rx);

//	m.complete = spi_complete_loop;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	m.is_dma_mapped = 1;

//	printk("tx_dma = %s\nrx_dma = %s\n",t.tx_dma,t.rx_dma);

//	dma_alloc_coherent();
//	dma_cache_wback_inv((unsigned long)w_buf,len);
//	dma_cache_wback_inv((unsigned long)r_buf,len);

	printk("------------------come to SSI driver\n");
	status=spi_sync(spi, &m);

	printk("------------------SPI done.\n");
//	printk("tx_dma = %s\nrx_dma = %s\n",t.tx_dma,t.rx_dma);

	printk("status = %d\n",status);

	printk("\ntx_buf:0x%08X rx_buf:0x%08X\ntx_dma:0x%08X rx_dma:0x%08X\n",
		(u32)t.tx_buf,(u32)t.rx_buf,t.tx_dma,t.rx_dma);
	printk("m.is_dma_mapped = %d\n\n",m.is_dma_mapped);
	
//	w_buf = phys_to_virt(t.tx_dma);
//	r_buf = phys_to_virt(t.rx_dma);
	
//	dma_free_coherent(NULL, len, t.tx_buf, t.tx_dma);
//	dma_free_coherent(NULL, len, t.rx_buf, t.rx_dma);
	
//	return spi_async(spi, &m);
	return 0;
}

int spi_transfer_file(struct spi_device *spi, u32 tx, u32 rx, size_t len, int flag)
{
	u32 status;
	struct spi_transfer	 t;
	struct spi_message	m;
	
	memset(&t,0,sizeof t);
	
	spi_message_init(&m);

	t.len = len;
	
	if(flag == SPI_DMA_MODE){
		t.tx_dma = (dma_addr_t)tx;
		t.rx_dma = (dma_addr_t)rx;
		t.tx_buf = phys_to_virt(t.tx_dma);
		t.rx_buf = phys_to_virt(t.rx_dma);
		m.is_dma_mapped = 1;
		printk("SPI DMA Mode!\n");
	}else{
		t.tx_buf = (u8 *)tx;
		t.rx_buf = (u8 *)rx;
		m.is_dma_mapped = 0;
		printk("SPI CPU Mode!\n");
	}

	spi_message_add_tail(&t, &m);

	printk("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	
	status=spi_sync(spi, &m);
	
	printk("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	
	printk("\ntx_buf:0x%08X rx_buf:0x%08X\ntx_dma:0x%08X rx_dma:0x%08X\n",
		(u32)t.tx_buf,(u32)t.rx_buf,t.tx_dma,t.rx_dma);
	printk("m.is_dma_mapped = %d\n\n",m.is_dma_mapped);
	
	printk("status = %d\n",status);
	
	return 0;
}

static int spi_transfer_bytes(struct spi_device *spi,const u8 *spisrc,size_t len,u8 send_mode)
{
	u8 *src,*dest;
	dma_addr_t dma_tx,dma_rx;
	struct spi_transfer	 t;
	struct spi_message	m;
	u8 status;
	
	if(send_mode == SPI_DMA_MODE){
		src = dma_alloc_coherent(NULL,((len+15)/16)*16,&dma_tx,GFP_KERNEL);
		dest= dma_alloc_coherent(NULL,((len+15)/16)*16,&dma_rx,GFP_KERNEL);
	}else{
		src = kzalloc(((len+1)/2)*2, GFP_KERNEL);
		dest= kzalloc(((len+1)/2)*2, GFP_KERNEL);
	}
	if(src == 0 || dest == 0){
		printk("kernel xalloc memery fail\n");
		return -1;
	}
	memcpy(src,spisrc,len);
	
	/* setup spi_message and spi_transfer */

	memset(&t,0,sizeof t);
	
	spi_message_init(&m);

	t.len = len;
	
	if(send_mode == SPI_DMA_MODE){
		t.tx_dma = dma_tx;
		t.rx_dma = dma_rx;
		t.tx_buf = phys_to_virt(t.tx_dma);
		t.rx_buf = phys_to_virt(t.rx_dma);
		m.is_dma_mapped = 1;
		printk("SPI DMA Mode!\n");
	}else{
		t.tx_buf = src;
		t.rx_buf = dest;
		m.is_dma_mapped = 0;
		printk("SPI CPU Mode!\n");
	}

	spi_message_add_tail(&t, &m);

	printk("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	
	status=spi_sync(spi, &m);
	
	printk("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

	printk("status = %d\n",status);
	
//		printk("src  =%s\n",src);
//		printk("dest =%s\n",dest);
	if(send_mode == SPI_DMA_MODE){
		dma_free_coherent(NULL, ((len+15)/16)*16, src, dma_tx);
		dma_free_coherent(NULL, ((len+15)/16)*16, dest,dma_rx);
	}else{
		kfree(src);
		kfree(dest);
	}
	return status;
}
int spi_standard_ops(struct spi_device *spi,int n)
{
	u8 *src,*dest;
	u8 cmd,result;
	
	src = kzalloc(10, GFP_KERNEL);
	dest = kzalloc(10, GFP_KERNEL);
	
	if(n == 0){
		printk("ops=spi_read\n");

		spi_read(spi,dest,1);
		printk("dest[0] = %d \n",dest[0]);
	}else if(n == 1){
		printk("ops=spi_write\n");

		*src = 63;
		spi_write(spi,src,1);
		printk("src[0] = %d\n",src[0]);
	}else if(n == 2){
		printk("ops=spi_w8r8\n");

		cmd = 63;
		result = spi_w8r8(spi,cmd);
		printk("spi_w8r8 = %d\n",result);
	}
	
	
	
	kfree(src);
	kfree(dest);
	return 0;
}

static int  jz_spi_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	T_SPI_PARAM_STRUCT tparam;   
	unsigned char i,*dest,*src;
	dma_addr_t dma_tx,dma_rx;
	
    void __user *argp = (void __user *)arg;

	struct spidev_data	*spidev;
	struct spi_device	*spi;
	struct jz47xx_spi *hw;
	
	spidev = file->private_data;
	spi = spidev->spi;
	hw = spi_master_get_devdata(spi->master);
	
    switch(cmd)
    {

/* add by shumb */
	case SPI_REGS_PRINT:
		copy_from_user(&tparam, argp, sizeof(tparam));
		tparam.reg[0] = REG_SSI_DR(tparam.chnl);
		tparam.reg[1] = REG_SSI_CR0(tparam.chnl);
		tparam.reg[2] = REG_SSI_CR1(tparam.chnl);
		tparam.reg[3] = REG_SSI_SR(tparam.chnl);
		tparam.reg[4] = REG_SSI_ITR(tparam.chnl);	 
		tparam.reg[5] = REG_SSI_ICR(tparam.chnl);
		tparam.reg[6] = REG_SSI_GR(tparam.chnl);

		if (copy_to_user((void*)arg, &tparam, sizeof(tparam))) return -EFAULT;
		break;
		
	case TEST_SPI_READ:
		copy_from_user(&tparam, argp, sizeof(tparam));
		tparam.val=spi_read_bits(tparam.chnl);
		if (copy_to_user((void*)arg, &tparam, sizeof(tparam))) {
			printk("Read fail\n");
			return -EFAULT;
			}
		break;
		
	case TEST_SPI_WRITE:
		copy_from_user(&tparam, argp, sizeof(tparam));
		printk("\nSPI WRITE:   0x%X(%d)  Channel %d\n",tparam.val,tparam.val,tparam.chnl);
		spi_write_bits(tparam.chnl,tparam.val);
		break;

	case TEST_SPI_INIT:
//		copy_from_user(&tparam, argp, sizeof(tparam));
		uspi_base_init();
		break;
		
	case TEST_SPI_REGS_SET:
		copy_from_user(&tparam, argp, sizeof(tparam));
		for(i=0;i<7;i++)
			printk("tparam.reg[%d]=0x%X\n",i,tparam.reg[i]);
		REG_SSI_DR(tparam.chnl)	= tparam.reg[0];
		REG_SSI_CR0(tparam.chnl)= tparam.reg[1];
		REG_SSI_CR1(tparam.chnl)= tparam.reg[2];
		REG_SSI_SR(tparam.chnl)	= tparam.reg[3];
		REG_SSI_ITR(tparam.chnl)= tparam.reg[4];	 
		REG_SSI_ICR(tparam.chnl)= tparam.reg[5];
		REG_SSI_GR(tparam.chnl)	= tparam.reg[6];
		break;
		
	case TEST_SPI_STATUS:
		copy_from_user(&tparam, argp, sizeof(tparam));
	//	__ssi_flush_fifo(tparam.chnl);
//		tparam.reg[0] = REG_SSI_DR(tparam.chnl);
		tparam.reg[1] = REG_SSI_CR0(tparam.chnl);
		tparam.reg[2] = REG_SSI_CR1(tparam.chnl);
		tparam.reg[3] = REG_SSI_SR(tparam.chnl);
		tparam.reg[4] = REG_SSI_ITR(tparam.chnl);
		tparam.reg[5] = REG_SSI_ICR(tparam.chnl);
		tparam.reg[6] = REG_SSI_GR(tparam.chnl);
		if (copy_to_user((void*)arg, &tparam, sizeof(tparam))) return -EFAULT;
		break;
	case TEST_SPI_PIN_CFG:
		copy_from_user(&tparam, argp, sizeof(tparam));
		if(tparam.io == 0)
			pin_config(tparam.chnl,tparam.ptr,tparam.val);
		else if(tparam.io == 1){
			tparam.reg[1] = gpio_config(tparam.reg[0],tparam.reg[1]);
			if (copy_to_user((void*)arg, &tparam, sizeof(tparam))) return -EFAULT;
		}
		break;
	case TEST_SPI_EN:
		copy_from_user(&tparam, argp, sizeof(tparam));
	//	REG_SSI_CR0(tparam.chnl) &= ~(1<<15);
	//	REG_SSI_CR0(tparam.chnl) |= tparam.reg[1];
		REG_SSI_CR0(tparam.chnl) = tparam.reg[1];
		break;
	case TEST_SPI_CLOCK:
		copy_from_user(&tparam, argp, sizeof(tparam));
		if(tparam.val == 0)
		{
		tparam.reg[0] = __cpm_get_pllout();
		tparam.reg[1] = __cpm_get_cclk();
		tparam.reg[2] = __cpm_get_hclk();
		tparam.reg[3] = __cpm_get_mclk();
		tparam.reg[4] = __cpm_get_pclk();
		tparam.reg[5] = __cpm_get_extalclk0();
		}else if(tparam.val == 1){
		tparam.reg[0] = REG_CPM_CPCCR;
		tparam.reg[1] = REG_CPM_CPPCR;
		tparam.reg[2] = REG_CPM_CPPSR;
		tparam.reg[3] = REG_CPM_SSICDR;
		tparam.reg[4] = REG_SSI_GR(tparam.chnl);
		
		}else if(tparam.val == 2){
		printk("CPCCR(x) = 0x%x\n",tparam.reg[0]);
		printk("CPPCR(x) = 0x%x\n",tparam.reg[1]);
		printk("CPPSR(x) = 0x%x\n",tparam.reg[2]);
		printk("SSICDR(x)= 0x%x\n",tparam.reg[3]);
		printk("SSI_GR(x)= 0x%x\n",tparam.reg[4]);
		REG_CPM_CPCCR = tparam.reg[0];
		REG_CPM_CPPCR = tparam.reg[1];
		REG_CPM_CPPSR = tparam.reg[2];
		REG_CPM_SSICDR = tparam.reg[3];
		REG_SSI_GR(tparam.chnl) = tparam.reg[4];
	//	udelay(10);
		}
		
		if (copy_to_user((void*)arg, &tparam, sizeof(tparam))) {
			printk("Clock Read fail\n");
			return -EFAULT;
		}
		break;
		
	case TEST_SPI_THRESHOLD_INT:
		copy_from_user(&tparam, argp, sizeof(tparam));
		if(tparam.val == 0){   //read regs except SSI_DR
//			tparam.reg[0] = REG_SSI_DR(tparam.chnl);
			tparam.reg[1] = REG_SSI_CR0(tparam.chnl);
			tparam.reg[2] = REG_SSI_CR1(tparam.chnl);
			tparam.reg[3] = REG_SSI_SR(tparam.chnl);
			tparam.reg[4] = REG_SSI_ITR(tparam.chnl);	 
			tparam.reg[5] = REG_SSI_ICR(tparam.chnl);
			tparam.reg[6] = REG_SSI_GR(tparam.chnl);
		}else if(tparam.val == 1){
//			REG_SSI_DR(tparam.chnl)	= tparam.reg[0];
			REG_SSI_CR0(tparam.chnl)= tparam.reg[1];
			REG_SSI_CR1(tparam.chnl)= tparam.reg[2];
			REG_SSI_SR(tparam.chnl)	= tparam.reg[3];
			REG_SSI_ITR(tparam.chnl)= tparam.reg[4];	 
			REG_SSI_ICR(tparam.chnl)= tparam.reg[5];
			REG_SSI_GR(tparam.chnl)	= tparam.reg[6];
		}else if(tparam.val == 2){
//			__ssi_disable_tx_intr(tparam.chnl);
//			__ssi_disable_rx_intr(tparam.chnl);

			__ssi_clear_errors(tparam.chnl);
			
			if(tparam.reg[0] == 0)
				INTC_SSI_UNMASK(tparam.chnl);
			else
				INTC_SSI_MASK(tparam.chnl);
			tparam.reg[0] = __REG_INTC_IMR();
		}

		if (copy_to_user((void*)arg, &tparam, sizeof(tparam))) {
			printk("Interrupt fail\n");
			return -EFAULT;
		}
		break;
		
	case TEST_SPI_READ_SUBSYSTEM:
/*		copy_from_user(&tparam, argp, sizeof(tparam));
		printk("Kernel  src = %s\n",(char *)tparam.src);
		
		
//		memcpy(tparam.dest,ps,strlen(ps)); 
		strcpy(tparam.dest,ps); 

		if (copy_to_user((void*)arg, &tparam, sizeof(tparam))) {
			printk("Interrupt fail\n");
			return -EFAULT;
		}*/
		break;
		
	case TEST_SPI_WRITE_SUBSYSTEM:
		copy_from_user(&tparam, argp, sizeof(tparam));
		printk("\n---------------------------------------\n");
		printk("\n@bytes size: ");
		
		printk("0x%X\n",tparam.val);
		if(tparam.val > 0x100000)
		{
			tparam.val = 0x100000;
			printk("limited to %d",tparam.val);
		}
		src = kzalloc(tparam.val+1, GFP_KERNEL);
		dest= kzalloc(tparam.val+1, GFP_KERNEL);
		
		if(src == 0 || dest == 0){
			printk("kzalloc fail\n");
			return -1;
		}
		strcpy(src,tparam.src); 
		printk("\nKernel  src = %s\n\n",src);
		
		printk("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
		
		write_via_spisystem_loop(spi,src,dest,tparam.val);
		
		printk("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
		
		printk("come back to spitest\n");
		
		strcpy(tparam.dest,dest);
//		printk("\nKernel dest = %s\n\n",dest);
		kfree(src);
		kfree(dest);

		if (copy_to_user((void*)arg, &tparam, sizeof(tparam))) {
			printk("Interrupt fail\n");
			return -EFAULT;
		}
		printk("\n---------------------------------------\n");
		break;
	case TEST_SPI_MALLOC:
		copy_from_user(&tparam, argp, sizeof(tparam));
		
		if (copy_to_user((void*)arg,&tparam, sizeof(tparam))) {
			printk("Interrupt fail\n");
			return -EFAULT;
		}
		
		break;
	case TEST_SPI_DEVICE_CONFIG:
		copy_from_user(&tparam, argp, sizeof(tparam));
		if(tparam.val == 0)
		{
			tparam.reg[0] = spi->bits_per_word;
			tparam.reg[1] = spi->mode;
			tparam.reg[2] = spi->max_speed_hz;

		}else if(tparam.val == 1){
			spi->bits_per_word = tparam.reg[0];
			spi->mode = tparam.reg[1];
			spi->max_speed_hz = tparam.reg[2];

			spi_setup(spi);
		}
		
		if (copy_to_user((void*)arg, &tparam, sizeof(tparam))) {
			printk("Interrupt fail\n");
			return -EFAULT;
		}
		break;
		
	case TEST_SPI_DMA_WRITE:
		copy_from_user(&tparam, argp, sizeof(tparam));
		printk("\n---------------------------------------\n");
		printk("\n@bytes size: ");

		printk("0x%X\n",tparam.val);
		if(tparam.val > 0x100000)
		{
			tparam.val = 0x100000;
			printk("limited to %d",tparam.val);
		}
		
	//	src = kzalloc(tparam.val+1, GFP_KERNEL);
	//	dest= kzalloc(tparam.val+1, GFP_KERNEL);
		src = dma_alloc_coherent(NULL,tparam.val,&dma_tx,GFP_KERNEL);
		dest= dma_alloc_coherent(NULL,tparam.val,&dma_rx,GFP_KERNEL);
		
		if(src == 0 || dest == 0){
			printk("dma_alloc_coherent fail\n");
			return -1;
		}
		
//		strcpy(src,tparam.src); 
		memcpy(src,tparam.src,tparam.val);
		
		printk("dma_tx:0x%08X\ndma_rx:0x%08X\n",dma_tx,dma_rx);
		
		printk("\nKernel  src = %s\n\n",src);
		
		
		printk("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
		
		write_via_dma_loop(spi,dma_tx,dma_rx,tparam.val);
		
		printk("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
		
		printk("come back to spitest\n");
		
//		strcpy(tparam.dest,dest);
		memcpy(tparam.dest,dest,tparam.val);

		dma_free_coherent(NULL, tparam.val, src, dma_tx);
		dma_free_coherent(NULL, tparam.val, dest,dma_rx);
		
//		kfree(src);
//		kfree(dest);

		if (copy_to_user((void*)arg, &tparam, sizeof(tparam))) {
			printk("Interrupt fail\n");
			return -EFAULT;
		}
		printk("\n---------------------------------------\n");
		break;
		
	case TEST_SPI_FILE_WRITE:
		copy_from_user(&tparam, argp, sizeof(tparam));
		if(tparam.io == SPI_DMA_MODE){
			src = dma_alloc_coherent(NULL,((tparam.val+15)/16)*16,&dma_tx,GFP_KERNEL);
			dest= dma_alloc_coherent(NULL,((tparam.val+15)/16)*16,&dma_rx,GFP_KERNEL);
		}else{
			src = kzalloc(((tparam.val+1)/2)*2, GFP_KERNEL);
			dest= kzalloc(((tparam.val+1)/2)*2, GFP_KERNEL);
		}
		if(src == 0 || dest == 0){
			printk("kernel xalloc memery fail\n");
			return -1;
		}
		
		memcpy(src,tparam.src,tparam.val);
		
		if(tparam.io == SPI_DMA_MODE)
			spi_transfer_file(spi,(u32)dma_tx,(u32)dma_rx,tparam.val,SPI_DMA_MODE);
		else
			spi_transfer_file(spi,(u32)src,(u32)dest,tparam.val,SPI_CPU_MODE);

		memcpy(tparam.dest,dest,tparam.val);

//		printk("src  =%s\n",src);
//		printk("dest =%s\n",dest);
		if(tparam.io == SPI_DMA_MODE){
			dma_free_coherent(NULL, tparam.val, src, dma_tx);
			dma_free_coherent(NULL, tparam.val, dest,dma_rx);
		}else{
			kfree(src);
			kfree(dest);
		}
		break;
	case TEST_SPI_CHIPSELECT:
		copy_from_user(&tparam, argp, sizeof(tparam));
		if(tparam.val == 0)
		{
			tparam.reg[0] = hw->pdata->board_size;
			tparam.reg[1] = hw->pdata->board_info[0].chip_select;
			tparam.reg[2] = hw->pdata->board_info[1].chip_select;

		}else if(tparam.val == 1){
	//		tparam.reg[0] = hw->pdata->board_size;
			hw->pdata->board_info[0].chip_select = tparam.reg[1];
			hw->pdata->board_info[0].chip_select = tparam.reg[2];

			jz_spi_pinconfig(hw);
		}
		
		if (copy_to_user((void*)arg, &tparam, sizeof(tparam))) {
			printk("Interrupt fail\n");
			return -EFAULT;
		}
		break;
	case TEST_SPI_SEND_BYTES:
		copy_from_user(&tparam, argp, sizeof(tparam));
		if(tparam.io == SPI_DMA_MODE)
			spi_transfer_bytes(spi,tparam.src,tparam.val,SPI_DMA_MODE);
		else
			spi_transfer_bytes(spi,tparam.src,tparam.val,SPI_CPU_MODE);
		
		break;
	case TEST_SPI_STANDARD_OPS:
		copy_from_user(&tparam, argp, sizeof(tparam));
		spi_standard_ops(spi,tparam.val);
		break;
    default:
        return -EINVAL;
        break;
    }
    return 0;
} 



static int jz_spi_open(struct inode *inode,struct file *filp )   
{ 
	

    try_module_get(THIS_MODULE);
    memset(&spi_state,0,sizeof(spi_state));
    
    filp->private_data = pspidev;
	
    printk("spi test open !\n");
	printk("spidev name = %s\n",pspidev->name);
    return 0;   
}   
  
static int jz_spi_release(struct inode *inode , struct file *filp)   
{   
    module_put(THIS_MODULE); 
    return 0;   
}   
static int spitest_probe(struct spi_device *spi)
{
	struct spidev_data	*spidev;
	dev_t			devt;

	
	printk("in %s\n",__FUNCTION__);
	
	/* Allocate driver data */
	spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
	if (!spidev)
		return -ENOMEM;

	pspidev = spidev;
	
	/* Initialize the driver data */
	spidev->spi = spi;
	spidev->name = "@spitest@";

	spidev->dev.parent = &spi->dev;

	dev_set_drvdata(&spi->dev, spidev);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */

	devt = MKDEV(JZSPI_MAJOR, 0);
	device_create(spitest_class, &spi->dev, devt,
			    spidev, "spitest");

#define KB (1024)
#define MB (1024*(KB))

//	spi->mode |= SPI_LOOP;
	spi->bits_per_word = 8;
	spi->max_speed_hz = 5*KB;

	spi_setup(spi);
	
	printk("spi_device name:%s,mode:0x%02X,bit:%d,max_speed_hz:%d\n",
		spi->modalias,spi->mode,spi->bits_per_word,spi->max_speed_hz);
	
	printk("out %s\n",__FUNCTION__);
	return 0;
}
static int spitest_remove(struct spi_device *spi)
{

	dev_set_drvdata(&spi->dev, NULL);
//	device_unregister(&spitest->dev);
	
	return 0;
}
static struct file_operations  jz_spi_fops =   
{   
    .owner  = THIS_MODULE,   
    .open   = jz_spi_open,   
    .ioctl  = jz_spi_ioctl,  
    //.poll   = jz_spi_select, 
    .release= jz_spi_release,   
};   

static struct spi_driver spitest_spi = {
	.driver = {
		.name =		"spitest",
		.owner =	THIS_MODULE,
	},
	.probe =	spitest_probe,
	.remove =	__devexit_p(spitest_remove),
};

static int __init jz_spi_init(void)   
{   
    int ret,status;   	

    //init_waitqueue_head(&gpio_wait);

    ret = register_chrdev(JZSPI_MAJOR,JZSPI_NAME,&jz_spi_fops);   
    if(ret < 0)   
    {   
        printk("SPITEST device driver can't register :%d\n",ret);   
        return -EAGAIN;   
    }   
    
    printk("SPITEST device driver registered\n"); 
	
     __gpio_as_ssi0();  
     CPM_SSI_START(0);
     __ssi_select_ce2(0);

	spitest_class = class_create(THIS_MODULE, "spitest");
	if (IS_ERR(spitest_class)) {
		unregister_chrdev(JZSPI_MAJOR, spitest_spi.driver.name);
		return PTR_ERR(spitest_class);
	}

	status = spi_register_driver(&spitest_spi);
	if (status < 0) {
		printk("spi_driver registered fail\n"); 
		return status;	
	}
	
	printk("spi_driver registered (status = %d)\n",status);
    return 0;   
}   
 
static void __exit jz_spi_exit(void)   
{    
	printk("jz_spi_exit\n");
	spi_unregister_driver(&spitest_spi);
    unregister_chrdev(JZSPI_MAJOR,JZSPI_NAME);   
}   
   
module_init(jz_spi_init);   
module_exit(jz_spi_exit);   
MODULE_LICENSE("GPL"); 

