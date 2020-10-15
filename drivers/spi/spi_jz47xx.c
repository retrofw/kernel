/* linux/drivers/spi/spi_jz47xx.c
 *
 * SSI controller for SPI protocol,use FIFO and DMA;
 * base-to: linux/drivers/spi/spi_bitbang.c
 *
 * Copyright (c) 2010 Ingenic
 * Author:Shumb <sbhuang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/jzsoc.h> 

#include "spi_jz47xx.h"

//#define SSI_DEGUG
#ifdef SSI_DEGUG
#define SSI_KEY_DEGUG
#define SSI_MSG
#define  print_dbg(format,arg...)			\
	printk(format,## arg)
#else
#define  print_dbg(format,arg...)	
#endif

//#define SSI_KEY_DEGUG
#ifdef SSI_KEY_DEGUG
#define print_kdbg(format,arg...)			\
	printk(format,## arg)
#else
#define print_kdbg(format,arg...)	
#endif

//#define SSI_MSG
#ifdef SSI_MSG
#define print_msg(format,arg...)			\
	printk(format,## arg)
#else
#define print_msg(format,arg...)	
#endif

static int jz_spi_dma_init(struct jz47xx_spi *hw,int flag);

static inline struct jz47xx_spi *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static void jz47xx_spi_cs(struct jz47xx_spi_info *spi, u8 cs, unsigned int pol)
{
	u32 pin_value = cs;

	__gpio_as_output(pin_value);
	if(!pol)
		__gpio_clear_pin(pin_value);
	else
		__gpio_set_pin(pin_value);
	
	print_msg("GPIO_PIN:0x%04X  LEVEL: %d:%d\n",
		pin_value,pol,__gpio_get_pin(pin_value));

}
static void jz47xx_spi_chipsel(struct spi_device *spi, int value)
{
	struct jz47xx_spi *hw = to_hw(spi);
	unsigned int cspol = spi->mode & SPI_CS_HIGH ? 1 : 0;

	switch (value) {
	case BITBANG_CS_INACTIVE:
		/* chip disable selected */
		if(hw->set_cs)
			hw->set_cs(hw->pdata,spi->chip_select, cspol^1);
		break;

	case BITBANG_CS_ACTIVE:

		if (spi->mode & SPI_CPHA)
			__ssi_set_spi_clock_phase(hw->chnl,1);
		else
			__ssi_set_spi_clock_phase(hw->chnl,0);
		
		if (spi->mode & SPI_CPOL)
			__ssi_set_spi_clock_polarity(hw->chnl,1);
		else
			__ssi_set_spi_clock_polarity(hw->chnl,0);

		if (!(spi->mode & SPI_LSB_FIRST))
			__ssi_set_msb(hw->chnl);
		else
			__ssi_set_lsb(hw->chnl);
		
		if(spi->mode & SPI_LOOP)
			__ssi_enable_loopback(hw->chnl);
		else
			__ssi_disable_loopback(hw->chnl);
		
		/* chip enable selected */
		if(hw->set_cs)
			hw->set_cs(hw->pdata,spi->chip_select, cspol);

		break;
	default:
		break;
	}
}

u32 jz47xx_spi_rx_buf_u32(struct jz47xx_spi *hw)
{				
	u32 data;
	u32 data1  = (u32)__ssi_receive_data(hw->chnl);
	u32 data2  = (u32)__ssi_receive_data(hw->chnl);
	u32 * rx = (u32 *)hw->rx;	
	if(hw->spi_mode & SPI_LSB_FIRST )
		data = (data1 | data2 << 16);
	else
		data = (data1 << 16 | data2);
	*rx++ = (u32)(data);			  		
	hw->rx = (u8 *)rx;	
	return (u32)data;				
}

u32 jz47xx_spi_tx_buf_u32(struct jz47xx_spi *hw)	
{										
	u32 data;
	u16 data1,data2;
	const u32 * tx = (u32 *)hw->tx;	
	data = *tx++;						
	hw->tx = (u8 *)tx;	
	if(hw->spi_mode & SPI_LSB_FIRST ){
		data1 = (u16)(data & 0xFFFF);
		data2 = (u16)(data >> 16);
	}else{
		data1 = (u16)(data >> 16);
		data2 = (u16)(data & 0xFFFF);
	}
	__ssi_transmit_data(hw->chnl,data1);
	__ssi_transmit_data(hw->chnl,data2);
	return data;						
}

#define JZ47XX_SPI_RX_BUF(type) 			\
u32 jz47xx_spi_rx_buf_##type(struct jz47xx_spi *hw) \
{									  		\
	u16 data  = __ssi_receive_data(hw->chnl);\
	type * rx = (type *)hw->rx;				\
	*rx++ = (type)(data);			  		\
	hw->rx = (u8 *)rx;						\
	return (u32)data;						\
}

#define JZ47XX_SPI_TX_BUF(type)				\
u32 jz47xx_spi_tx_buf_##type(struct jz47xx_spi *hw)	\
{										\
	u16 data;							\
	const type * tx = (type *)hw->tx;	\
	data = *tx++;						\
	hw->tx = (u8 *)tx;					\
	__ssi_transmit_data(hw->chnl,data);	\
	return (u32)data;					\
}

JZ47XX_SPI_RX_BUF(u8)
JZ47XX_SPI_TX_BUF(u8)

JZ47XX_SPI_RX_BUF(u16)
JZ47XX_SPI_TX_BUF(u16)

static unsigned int jz_spi_get_clk(struct spi_device *spi)
{
	struct jz47xx_spi *hw = to_hw(spi);
	int ssicdr,cgv;
	unsigned long clk;
	
	ssicdr 	= __cpm_get_ssidiv();
	cgv 	= __ssi_get_grdiv(hw->chnl);
#ifdef JZ_NEW_CODE_TYPE
	if(!hw->pdata->is_pllclk)
		ssicdr = 0;
#endif
	clk = hw->src_clk/(2*(cgv+1)*(ssicdr+1));
	hw->spi_clk = clk;
	return clk;
}

static unsigned int jz_spi_set_clk(struct spi_device *spi,u32 hz)
{
	struct jz47xx_spi *hw = to_hw(spi);
	int div,ssicdr=0,cgv=0;
	u32 ssiclk;

	ssiclk = hw->src_clk;
	
	if(hw->src_clk < hz){
		dev_info(&spi->dev,"Warning:invalid clock(%d Hz) be set to source clk(%d Hz)!\n",
		hz,(uint)hw->src_clk);
		hz = hw->src_clk;
	}

	div = ssiclk/ hz;

	/* clk = (exclk or pllclk)/( (SSICDR +1) * (2*(CGV+1)) )    */
	/* 4750: SSICDR (0-15)   CGV (0-255)   16*(2*256)     		*/
	/* 4760: SSICDR (0-63)   CGV (0-255)   64*(2*256)	  		*/

	if(hw->pdata->is_pllclk){  		/* source clock is PLLCLK */
		if(div <= 2*(MAX_SSICDR+1)){
			cgv    = 0;
			ssicdr = div/2 -1;
		}else if(div <= 2*(MAX_CGV+1)*(MAX_SSICDR+1)){	
			ssicdr = MAX_SSICDR;
			cgv    = (div/(MAX_SSICDR+1))/2 - 1;

		}else{
			ssicdr = MAX_SSICDR;
			cgv	   = MAX_CGV;
		}	
	}else{  						/* source clock is EXCLK */
		if(div <= 2*(MAX_CGV+1)){	
			cgv    = div/2- 1;
		}else
			cgv	   = MAX_CGV;
	}

	if (cgv < 0)
		cgv = 0;
	if (ssicdr < 0)
		ssicdr = 0;
	
	dev_dbg(&spi->dev,"SSICLK:%d setting pre-scaler to %d (hz %d) SSICDR:%d  CGV:%d\n",
		ssiclk,div, hz,ssicdr,cgv);

	__ssi_disable(0);
	__ssi_disable(1);
	__cpm_set_ssidiv(ssicdr);
	__ssi_set_grdiv(hw->chnl,cgv);

	return 0;
}

/* the spi->mode bits understood by this driver: */
#define MODEBITS (SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST | SPI_LOOP)
#define SPI_BITS_SUPPORT  (SPI_BITS_8 | SPI_BITS_16 | SPI_BITS_32)

/* every spi_transfer could call this routine to setup itself */
static int jz47xx_spi_setupxfer(struct spi_device *spi,struct spi_transfer *t)
{
	struct jz47xx_spi *hw = to_hw(spi);
	u8  bpw,fifo_width;
	u32 hz;
	
	bpw = t ? t->bits_per_word : spi->bits_per_word;
	hz  = t ? t->speed_hz : spi->max_speed_hz;

	if(t){
		if(!bpw)
			bpw = spi->bits_per_word;
		if(!hz) 
			hz= spi->max_speed_hz;
	}
	
	if (bpw < 2 || bpw > 32) {
		dev_err(&spi->dev, "invalid bits-per-word (%d)\n", bpw);
		return -EINVAL;
	}

	hw->bits_per_word = bpw;
	if(bpw <= 8 ){
		hw->transfer_unit_size = SPI_8BITS;
		hw->get_rx = jz47xx_spi_rx_buf_u8;
		hw->get_tx = jz47xx_spi_tx_buf_u8;
		fifo_width = FIFO_W8;
	}else if(bpw <= 16){
		hw->transfer_unit_size = SPI_16BITS;
		hw->get_rx = jz47xx_spi_rx_buf_u16;
		hw->get_tx = jz47xx_spi_tx_buf_u16;
		fifo_width = FIFO_W16;
	}else{
		hw->transfer_unit_size = SPI_32BITS;
		hw->get_rx = jz47xx_spi_rx_buf_u32;
		hw->get_tx = jz47xx_spi_tx_buf_u32;
		fifo_width = FIFO_W16;
	}
	hw->txfifo_width = fifo_width;
	hw->rxfifo_width = fifo_width;
	__ssi_set_frame_length(hw->chnl,fifo_width);
	
	jz_spi_set_clk(spi,hz);

	dev_dbg(&spi->dev,"The real SPI CLK is %d Hz\n",jz_spi_get_clk(spi));
	
	spin_lock(&hw->bitbang.lock);
	if (!hw->bitbang.busy) {
		hw->bitbang.chipselect(spi, BITBANG_CS_INACTIVE);
		/* need to ndelay for 0.5 clocktick ? */
	}
	spin_unlock(&hw->bitbang.lock);

	return 0;
}

static int jz47xx_spi_setup(struct spi_device *spi)
{
	int ret;
	struct jz47xx_spi *hw = to_hw(spi);
	
	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	
	if (spi->mode & ~MODEBITS) {
		dev_info(&spi->dev, "Warning: unsupported mode bits %x\n",
			spi->mode & ~MODEBITS);
		return -EINVAL;
	}
	hw->spi_mode = spi->mode;

	if(spi->bits_per_word & ~SPI_BITS_SUPPORT){
		dev_info(&spi->dev, "Warning: unsupported bits_per_word: %d\n",
			spi->bits_per_word);
		return -EINVAL;
	}
	
	if (!spi->max_speed_hz)
		return -EINVAL;
	
	if(hw->src_clk < spi->max_speed_hz){
		dev_info(&spi->dev,"Warning:invalid clock(%d Hz) be set to source clk(%d Hz)!\n",
		spi->max_speed_hz,(uint)hw->src_clk);
		spi->max_speed_hz = hw->src_clk;
	}
	ret = jz47xx_spi_setupxfer(spi, NULL);
	if (ret < 0) {
		dev_err(&spi->dev, "Warning:setupxfer returned %d\n", ret);
		return ret;
	}

	return 0;
}

static inline u32 cpu_write_txfifo(struct jz47xx_spi *hw,u32 entries)
{
	u32 i,cnt,count;
	u32 dat;
	u8 unit_size = hw->transfer_unit_size;
		
	if((!entries )|| (!(hw->rw_mode & RW_MODE))){
		return -1;
	}
	/* because SSI FIFO width is 16(17) bits */
	cnt = (unit_size == SPI_32BITS) ? (entries >> 1) : entries;
	count = cnt * unit_size;
	if(hw->rw_mode & W_MODE){                           
		for(i=0;i<cnt;i++)
		{
			dat = (u32)(hw->get_tx(hw));
			print_dbg("+%x,",dat);
		}	
	}else{		 /* read, fill txfifo with 0 */
		
		for(i=0;i<cnt;i++)
			__ssi_transmit_data(hw->chnl,0);
		print_dbg("0x0...,");
	}
	
	hw->count += count;
	
	return count;	
}
static inline int spi_start_dma(struct jz47xx_spi *hw,unsigned int count, int mode)
{
	unsigned char dma_unit,src_bit,dest_bit;
	unsigned char bpw = hw->transfer_unit_size * 8;
	int chan;
	unsigned int phyaddr;
	unsigned long flags;
//	unsigned char fifo_width = FIFO_W8;
//	unsigned char fifo_width = (bpw < FIFO_W16 ) ? FIFO_W8 : FIFO_W16;
	
	mode &= DMA_MODE_MASK;
	if( mode == DMA_MODE_READ ){	/* dma read rxfifo */
		chan = hw->dma_rx_chnl;
		src_bit  = hw->rxfifo_width;
		dest_bit = bpw;
		phyaddr  = hw->rx_dma;	
		dma_unit = hw->dma_rx_unit;
		
		print_dbg("SPI DMA Rx fifo_width = %d bits\n",hw->rxfifo_width);
	}else if( mode == DMA_MODE_WRITE ){  /* dma write txfifo */
		chan = hw->dma_tx_chnl;
		src_bit  = bpw;
		dest_bit = hw->txfifo_width;
		phyaddr  = hw->tx_dma;
		dma_unit = hw->dma_tx_unit;
		
		print_dbg("SPI DMA Tx fifo_width = %d bits\n",hw->txfifo_width);
	}else{
		dev_err(hw->dev,"SPI Start DMA Fail(Mode Error) !!!\n");
		return SPI_DMA_ERROR;
	}

	if(chan < 0)
		return chan;

	flags = claim_dma_lock();
	disable_dma(chan);
	clear_dma_ff(chan);	
	jz_set_dma_src_width(chan, src_bit);
	jz_set_dma_dest_width(chan, dest_bit);

	jz_set_dma_block_size(chan, dma_unit);	 /* n byte burst */
	set_dma_mode(chan, mode);
	set_dma_addr(chan, phyaddr);
	set_dma_count(chan, count); /* ensure dma count align */
	enable_dma(chan);
	release_dma_lock(flags);

	return 0;
}

static inline int spi_dma_setup(struct jz47xx_spi *hw,unsigned int len)
{
	int status = 0;
		
	/* Start Rx to read if hw->rx_dma is not NULL */
	if(hw->rx_dma){
		status = spi_start_dma(hw,len,DMA_MODE_READ);
		if(status < 0 && hw->rw_mode & R_DMA)
			return status;
	}

	/* Start Tx for dummy write */
	if(!(hw->tx_dma)){
		dev_info(hw->dev,"hw->tx_dma = NULL !!!\n");
		hw->tx_dma = hw->rx_dma;
	}
	
	status = spi_start_dma(hw,len,DMA_MODE_WRITE);
	if(status < 0)
		return status;
	
	print_kdbg("DMA transfer Start...\n");
	
	return status;
}

int jz_spi_dma_transfer(struct jz47xx_spi *hw)
{
	/* change SSI trigger for DMA transfer */
	/* Important!!! it probable die in waitting for DMA_RX_END intr
		if configure improper. */
	int status,tmp;
	int i,dma_ds[] = {32,16, 4, 2, 1};//{4, 1, 2, 16, 32};
//	u8 unit_size = hw->transfer_unit_size;
	
	/* request DMA irq dynamic */
	if(hw->chnl != SSI_DMA_FASTNESS_CHNL){
		status = jz_spi_dma_init(hw,1);
		if(status){
			dev_err(hw->dev,"DMA dynamic request fail!\n");
			return SPI_DMA_ERROR;
		}
		
		if(hw->rw_mode & R_DMA){
			if(hw->dma_rx_chnl < 0){
				dev_err(hw->dev,"Error:DMA Rx channel invalid !!!\n");
				return SPI_DMA_ERROR;
			}
		}
	}

	for(i = 0; i< ARRAY_SIZE(dma_ds); i++){
		if(!(hw->len % dma_ds[i]))
			break;
	}
	if(i < ARRAY_SIZE(dma_ds)){
		/* Is it DMA Hardware error ??? I don't known. So to avoid it as below. */
		if(hw->transfer_unit_size == SPI_8BITS &&  dma_ds[i]== 32)
			tmp = 16;
		else
			tmp = dma_ds[i];
		
		hw->dma_tx_unit = tmp;
		hw->dma_rx_unit = tmp;
		
	}else{
		print_msg("DMA block_size force to defaut set!!!");
		hw->dma_tx_unit = JZ_SSI_DMA_BURST_LENGTH;
		hw->dma_rx_unit = JZ_SSI_DMA_BURST_LENGTH;

	}
	hw->tx_trigger = hw->dma_tx_unit/(hw->txfifo_width>>3);
	hw->rx_trigger = hw->dma_rx_unit/(hw->rxfifo_width>>3);	

	if(hw->rx_trigger < 8)
		hw->rx_trigger = 8;

	print_msg("dma_tx_unit=%d,dma_rx_unit=%d,tx_trigger=%d,rx_trigger=%d,\n",
		hw->dma_tx_unit,hw->dma_rx_unit,hw->tx_trigger,hw->rx_trigger);	
	
	__ssi_set_tx_trigger(hw->chnl,hw->tx_trigger);
	__ssi_set_rx_trigger(hw->chnl,hw->rx_trigger);
	
	status = spi_dma_setup(hw,hw->len);
	
	if(status < 0){
		dev_info(hw->dev,"DMA setup fail!(rw_mode = %d,status = %d)\n",hw->rw_mode,status);
		return status;
	}
	
	hw->count = hw->len;
	hw->use_dma = 1;
	return 0;
}
int jz_spi_cpu_transfer(struct jz47xx_spi *hw,long length)
{
	unsigned char int_flag = 0,last_flag = 0;
	unsigned int unit_size,trigger,send_entries,entries=0,cnt;
	long leave_len_bytes;
	
	/* calculate the left entries */
	leave_len_bytes = hw->len - hw->count;
	
	if(hw->len < hw->count){
		dev_err(hw->dev,"Fill data len error!!!(len < count)\n");
		return -1;
	}
	if(leave_len_bytes == 0)							/* --- End or ??? --- */
	{
		print_dbg("leave_len_bytes = 0\n");
		return 0;
	}
	
	unit_size = hw->transfer_unit_size;
	if( unit_size == SPI_8BITS )
		entries = leave_len_bytes;
	else if(unit_size == SPI_32BITS )
	{
		/* because SSI FIFO width is 16(17) bits */
		entries = leave_len_bytes >> 1;
	}else if(unit_size == SPI_16BITS )
	{
		entries = leave_len_bytes >> 1;
	}else{
		dev_err(hw->dev,"transfer_unit_size error!\n");
		return -1;
	}

	trigger = JZ_SSI_MAX_FIFO_ENTRIES - hw->tx_trigger;
	
	/* calculate the entries which will be sent currently  */
	if(hw->is_first){  /* distinguish between the first and interrupt */

		/* CPU Mode should reset SSI triggers at first */
		hw->tx_trigger = SSI_TX_FIFO_THRESHOLD*8;
		hw->rx_trigger = (SSI_RX_FIFO_THRESHOLD - SSI_SAFE_THRESHOLD)*8;
		__ssi_set_tx_trigger(hw->chnl,hw->tx_trigger);
		__ssi_set_rx_trigger(hw->chnl,hw->rx_trigger);
		
		if(entries <= JZ_SSI_MAX_FIFO_ENTRIES)	
			send_entries = entries;
		else{ 			
		/* need enable half_intr, left entries will be sent in SSI interrupt 
			and receive the datas*/
			send_entries = JZ_SSI_MAX_FIFO_ENTRIES;
			int_flag = 1;
		}
		hw->is_first = 0;
	}else{	/* happen in interrupts */
		if(entries <= trigger){
			send_entries = entries;
			last_flag = 1;	/* the last part of data shouldn't disable RXI_intr at once !!! */
		}
		else{			
		/* need enable half_intr, left entries will be sent in SSI interrupt 
		and receive the datas*/
			send_entries = CPU_ONCE_BLOCK_ENTRIES;
			int_flag = 1;
		}
	}
	
	if(length > 0){
		length = length/hw->transfer_unit_size;
		if(length < send_entries)
			send_entries = length;
	}
	/* fill the txfifo with CPU Mode */
	if((cnt = cpu_write_txfifo(hw,send_entries)) < 0){
		dev_info(hw->dev,"cpu_write_txfifo error!\n");
		return -1;
	}
	print_kdbg("+:(%d)\n",cnt);
	/* every time should control the SSI half_intrs */
	if(int_flag)
	{	
		__ssi_enable_txfifo_half_empty_intr(hw->chnl);
		__ssi_enable_rxfifo_half_full_intr(hw->chnl);
	}else
	{
		__ssi_disable_txfifo_half_empty_intr(hw->chnl);
		__ssi_disable_rxfifo_half_full_intr(hw->chnl);
	}
	
	/* to avoid RxFIFO overflow when CPU Mode at last time to fill */
	if(last_flag)
	{				
		last_flag = 0;
		__ssi_enable_rxfifo_half_full_intr(hw->chnl);
	}
	
	return 0;
}

/* Fill SSI TxFIFO according to the data count,and decide whether DMA is used */
static inline int jz_spi_start_transfer(struct jz47xx_spi *hw)
{
	int status = 0;
	int err_len;
	hw->use_dma = 0;
	hw->is_first = 1;

	if( (err_len =hw->len % hw->transfer_unit_size) != 0){
		dev_info(hw->dev,"Warning:The spi_transfer len didn't aligned!!! try to align!\n");
		hw->len -= err_len;
		print_msg("spi_transfer modified len=%d ,bits_per_word =%d bits\n",hw->len,hw->bits_per_word);	
		if(hw->len < hw->transfer_unit_size){
			dev_info(hw->dev,"ERROR:The spi_transfer modified len=%d,bits_per_word =%d bits\n",
				hw->len,hw->bits_per_word);
			return -1;
		}
			
	}

	print_kdbg("START:");
	if( hw->rw_mode & RW_DMA ) 							/* --- DMA transfer --- */	
	{
		status = jz_spi_dma_transfer(hw);
		if(status){
			if(status == SPI_DMA_ERROR){
				if(hw->dma_tx_chnl)
					jz_free_dma(hw->dma_tx_chnl);
				if(hw->dma_rx_chnl)
					jz_free_dma(hw->dma_rx_chnl);

				if(!(hw->tx) && !(hw->rx)){
					dev_err(hw->dev,"DMA mode transfer fail!\n");
					return status;
				}
				hw->rw_mode &= ~RW_DMA;
				hw->use_dma = 0;
				dev_info(hw->dev,"Try CPU mode instead!\n");
				status = jz_spi_cpu_transfer(hw,0);
		    }else
		    	return status;
		}
	}
	else			 									/* --- CPU transfer --- */	
		status = jz_spi_cpu_transfer(hw,0);
	
	if(status < 0)
	{
		__ssi_disable_tx_intr(hw->chnl);
		__ssi_disable_tx_intr(hw->chnl);
		dev_err(hw->dev,"CPU mode transfer fail!\n");
		return SPI_CPU_ERROR;
	}
	
	if(hw->use_dma){
//		__ssi_disable_tx_error_intr(hw->chnl);
//		__ssi_disable_rx_error_intr(hw->chnl);
		__ssi_disable_tx_intr(hw->chnl);
		__ssi_disable_tx_intr(hw->chnl);

	}else{
		__ssi_enable_tx_error_intr(hw->chnl);
		__ssi_enable_rx_error_intr(hw->chnl);
	}

	/* start SSI transfer, and start DMA transfer when DMA Mode */
	__ssi_enable(hw->chnl);
	
	return 0;
}
void print_ssi_regs(u8 n)
{
	print_msg("\nSSI%d\n",n);
//	print_msg("REG_SSI_DR ========0x%x  \n",REG_SSI_DR(n));
	print_msg("REG_SSI_CR0========0x%x  \n",REG_SSI_CR0(n));
	print_msg("REG_SSI_CR1========0x%x  \n",REG_SSI_CR1(n));
	print_msg("REG_SSI_SR ========0x%x  \n",REG_SSI_SR(n));
	print_msg("REG_SSI_ITR========0x%x  \n",REG_SSI_ITR(n));
	print_msg("REG_SSI_ICR========0x%x  \n",REG_SSI_ICR(n));
	print_msg("REG_SSI_GR ========0x%x  \n",REG_SSI_GR(n));

}

void save_ssi_regs(u32 *regs,u8 n)
{
	regs[0]=REG_SSI_CR0(n);
	regs[1]=REG_SSI_CR1(n);
	regs[2]=REG_SSI_SR(n);
	regs[3]=REG_SSI_ITR(n);
	regs[4]=REG_SSI_ICR(n);
	regs[5]=REG_SSI_GR(n);
}

void show_ssi_regs(u32 *regs)
{
	
//	print_msg("REG_SSI_DR ========0x%x  \n",REG_SSI_DR(n));
	print_msg("\nREG_SSI_CR0=====0x%x  \n",regs[0]);
	print_msg("REG_SSI_CR1=====0x%x  \n",regs[1]);
	print_msg("REG_SSI_SR =====0x%x  \n",regs[2]);
	print_msg("REG_SSI_ITR=====0x%x  \n",regs[3]);
	print_msg("REG_SSI_ICR=====0x%x  \n",regs[4]);
	print_msg("REG_SSI_GR =====0x%x  \n",regs[5]);
}

void ssi_intr_stat(struct jz_intr_cnt *jz_intr)
{
	print_msg("\nSSI interrupt cnts = %d\n",jz_intr->ssi_intr_cnt);
	
	print_msg("TXI:%d  RXI:%d\nunderrun:%d  overrun:%d\n\n",
		jz_intr->ssi_txi,jz_intr->ssi_rxi,jz_intr->ssi_eti,jz_intr->ssi_eri);

	print_msg("DMA TX interrupt cnts = %d\nDMA RX interrupt cnts = %d\n",
		jz_intr->dma_tx_cnt,jz_intr->dma_rx_cnt);

	print_msg("dma_tx_err:%d  dma_tx_end:%d\ndma_rx_err:%d  dma_rx_end:%d\n\n",
		jz_intr->dma_tx_err,jz_intr->dma_tx_end,jz_intr->dma_rx_err,jz_intr->dma_rx_end);
}

#ifdef SSI_DEGUG
void ssi_dump_jz_ddma_channel(unsigned int dmanr)
{
	printk("\n----------chan%d----------------\n",dmanr);
	printk("  DSAR   = 0x%08x\n", REG_DMAC_DSAR(dmanr));
	printk("  DTAR   = 0x%08x\n", REG_DMAC_DTAR(dmanr));
	printk("  DTCR   = 0x%08x\n", REG_DMAC_DTCR(dmanr));
	printk("  DRSR   = 0x%08x\n", REG_DMAC_DRSR(dmanr));
	printk("  DCCSR  = 0x%08x\n", REG_DMAC_DCCSR(dmanr));
	printk("  DCMD  = 0x%08x\n", REG_DMAC_DCMD(dmanr));
	printk("  DDA  = 0x%08x\n", REG_DMAC_DDA(dmanr));
	printk("-------------------------\n");
}
#endif

static int jz47xx_spi_txrx(struct spi_device * spi, struct spi_transfer *t)
{
	int status;
	struct jz47xx_spi * hw = to_hw(spi);
	struct jz_intr_cnt *g_jz_intr = hw->g_jz_intr;

	
	print_dbg("in %s\n",__FUNCTION__);
	
	print_kdbg("mname: %s  txrx: tx %p, rx %p, len %d ,tx_dma 0x%08X ,rx_dma 0x%08X\n",
		spi->modalias,t->tx_buf, t->rx_buf, t->len, t->tx_dma ,t->rx_dma);	

	if(t->len == 0)
		return 0;
	
	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;
	hw->tx_dma = t->tx_dma;
	hw->rx_dma = t->rx_dma;
	hw->len = t->len;
	hw->count = 0;
	hw->rlen = 0;
	hw->dma_flag &= ~SPI_DMA_ACK;
	g_jz_intr->ssi_intr_cnt = 0;

	hw->rw_mode = 0;
	if(hw->tx)
		hw->rw_mode |= W_MODE; 
	if(hw->rx)
		hw->rw_mode |= R_MODE; 

	if(hw->tx_dma)
		hw->rw_mode |= W_DMA;
	if(hw->rx_dma)
		hw->rw_mode |= R_DMA;
	
#ifdef SSI_DEGUG
	if( REG_SSI_CR0(hw->chnl) & (1<<10) )
		print_dbg("Loop Mode\n");
	else
		print_dbg("Normal Mode\n");

	print_kdbg("hw->rw_mode = 0x%X\n",hw->rw_mode);
#endif
	
	__ssi_disable_tx_intr(hw->chnl);
	__ssi_disable_rx_intr(hw->chnl);

	__ssi_wait_transmit(hw->chnl);
	/* flush TX FIFO and fill FIFO */
	__ssi_flush_fifo(hw->chnl);
	
	__ssi_enable_receive(hw->chnl);
	__ssi_clear_errors(hw->chnl);
	
	memset(g_jz_intr, 0, sizeof *g_jz_intr);
	
	status = jz_spi_start_transfer(hw);
	if(status){
		dev_err(hw->dev,"ERROR:spi_transfer error(%d)!\n",status);
		return status;
	}

	/* wait the interrupt finish the transfer( one spi_transfer be sent ) */
	wait_for_completion(&hw->done);
	
	__ssi_finish_transmit(hw->chnl);
	__ssi_clear_errors(hw->chnl);
	if(__ssi_rxfifo_empty(hw->chnl))
		__ssi_disable(hw->chnl);
	
	/* ------- for debug --------- */
#ifdef SSI_MSG
	//	print_msg("\ninterrupt Enable:\nTIE:%d RIE:%d \nTEIE:%d REIE:%d\n",
	//		(regs[0]&1<<14)>>14,(regs[0]&1<<13)>>13,(regs[0]&1<<12)>>12,(regs[0]&1<<11)>>11);
	ssi_intr_stat(g_jz_intr);
#if 0
	print_msg("--- SSI DMA Rx ---\n");
	ssi_dump_jz_ddma_channel(hw->dma_rx_chnl);
	print_msg("check:hw->tx_trigger=%d,hw->tx_trigger=%d\n",hw->tx_trigger,hw->tx_trigger);
#endif
#endif
	/* ---------------------------- */

	if(hw->use_dma){
		hw->use_dma = 0;
		if(hw->dma_flag == SPI_DMA_ACK)
			hw->rlen += hw->count;
		else
			dev_info(hw->dev,"DMA data transfer ERROR!\n");
		
		if(hw->chnl != SSI_DMA_FASTNESS_CHNL){
			if(hw->dma_tx_chnl >= 0)
				jz_free_dma(hw->dma_tx_chnl);
			if(hw->dma_rx_chnl >= 0)
				jz_free_dma(hw->dma_rx_chnl);
		}
	}	

	if(hw->rlen != t->len){
		dev_info(hw->dev,"Length error:hw->rlen=%d  t->len=%d\n",hw->rlen,t->len);
		if(hw->rlen > hw->len)
			hw->rlen = hw->len;
	}

	return hw->rlen;
}
static inline u32 cpu_read_rxfifo(struct jz47xx_spi *hw)
{
	u32 cnt,dat;
	u8 unit_size = hw->transfer_unit_size;
	
	if(__ssi_get_rxfifo_count(hw->chnl) < 1){
		print_msg("The count of TxFIFO is %d \n",__ssi_get_rxfifo_count(hw->chnl));
		return 0;
	}
		
	cnt = hw->rlen;
	if((hw->rw_mode & RW_MODE) == W_MODE){
		print_dbg("W_MODE\n");
		if(unit_size == SPI_32BITS)
			hw->rlen += unit_size*(__ssi_get_rxfifo_count(hw->chnl)/2);
		else
			hw->rlen += unit_size*__ssi_get_rxfifo_count(hw->chnl);
		
		__ssi_flush_rxfifo(hw->chnl);
		return (hw->rlen - cnt);
	}
	while(!__ssi_rxfifo_empty(hw->chnl))
	{
		dat = hw->get_rx(hw);
		hw->rlen += unit_size;
		print_dbg("-%x,",dat);
	}
	return (hw->rlen - cnt);
}
static irqreturn_t jz47xx_spi_irq(int irq, void *dev)
{
	struct jz47xx_spi *hw = dev;
	long left_count= hw->len - hw->count;
	u8 flag = 0;
	u32 cnt;
	int status;
	struct jz_intr_cnt *g_jz_intr = hw->g_jz_intr;

	g_jz_intr->ssi_intr_cnt++;
	/* to avoid die in interrupt if some error occur */
	if(g_jz_intr->ssi_intr_cnt >MAX_SSI_INTR)
	{
		__ssi_disable_tx_intr(hw->chnl);
		__ssi_disable_rx_intr(hw->chnl);
		dev_err(hw->dev,"\nERROR:SSI interrupts too many count(%d)!\n",
			g_jz_intr->ssi_intr_cnt);
			
		g_jz_intr->ssi_intr_cnt = 0;
		complete(&hw->done);

		goto irq_done;
	}
	
	if( __ssi_underrun(hw->chnl) &&
		__ssi_tx_error_intr(hw->chnl)){
		print_kdbg("UNDR:");
		g_jz_intr->ssi_eti++;
		__ssi_disable_tx_error_intr(hw->chnl);
		/*  spi transfer complete at DMA mode */
		if(hw->use_dma && (hw->rw_mode & RW_DMA)){
			print_kdbg("DMA mode SPI transfer complete for aligness!\n");
			__ssi_clear_errors(hw->chnl);
			if(hw->rw_mode & R_DMA){
				hw->rx_trigger = 0;
				__ssi_set_rx_trigger(hw->chnl,hw->rx_trigger);
			}else{
				/* need delay for data transfer complete in SPI BUS.
				so need to fill spi_transfer.delay_usecs */ 
				if(!(hw->dma_flag & SPI_DMA_ACK)){
					hw->dma_flag |= SPI_DMA_ACK;
					__ssi_disable_tx_intr(hw->chnl);
					complete(&hw->done);
				}			
			}	
			goto irq_done;
		}

		if( left_count == 0){
			cnt = cpu_read_rxfifo(hw);
			print_kdbg("-:(%d)\n",cnt);
			
			__ssi_disable_tx_intr(hw->chnl);
			__ssi_disable_rx_intr(hw->chnl);

			complete(&hw->done);
		}
		else 
			__ssi_enable_tx_error_intr(hw->chnl);
		
		flag++;

	}
	
	if( __ssi_overrun(hw->chnl) &&
			__ssi_rx_error_intr(hw->chnl)){

			print_kdbg(" overrun:");
			g_jz_intr->ssi_eri++;
	
			cnt = cpu_read_rxfifo(hw);
			print_kdbg("-:(%d)\n",cnt);
				
			flag++;
	}

	if ( __ssi_rxfifo_half_full(hw->chnl) &&
		__ssi_rxfifo_half_full_intr(hw->chnl)) {

		print_kdbg("RXI:");
		g_jz_intr->ssi_rxi++;
		
		cnt = cpu_read_rxfifo(hw);
		print_kdbg("-:(%d)\n",cnt);
		
		flag++;
	}

	if ( __ssi_txfifo_half_empty_intr(hw->chnl) &&
		 __ssi_txfifo_half_empty(hw->chnl)) {

		print_kdbg("TXI:");
		g_jz_intr->ssi_txi++;
		
		status = jz_spi_cpu_transfer(hw,0);
		if(status < 0)
		{
			__ssi_disable(hw->chnl);
			__ssi_disable_tx_intr(hw->chnl);
			__ssi_disable_rx_intr(hw->chnl);	
			dev_err(hw->dev,"jz_spi_cpu_transfer error!!!!!\n");
			complete(&hw->done);
			
			goto irq_done;
		}
		flag++;
	}
	
	if(!flag)
	{
		dev_info(hw->dev,"\nERROR:SSI interrupt Type error\n");
		complete(&hw->done);
	}
	
 irq_done:
 	__ssi_clear_errors(hw->chnl);
	return IRQ_HANDLED;
}
static irqreturn_t spi_dma_tx_irq(int irq, void *dev)
{
	struct jz47xx_spi *hw = dev;
	struct jz_intr_cnt *g_jz_intr = hw->g_jz_intr;
	int chan = hw->dma_tx_chnl;

	g_jz_intr->dma_tx_cnt++;
	disable_dma(chan);
	if (__dmac_channel_address_error_detected(chan)) {
		printk(KERN_DEBUG "%s: DMAC address error.\n", __FUNCTION__);
		__dmac_channel_clear_address_error(chan);
		g_jz_intr->dma_tx_err++;
		
		dev_err(hw->dev,"DMA addr error\n");
		complete(&hw->done);
		
		return IRQ_HANDLED;
	}
	if (__dmac_channel_transmit_end_detected(chan)) {
		__dmac_channel_clear_transmit_end(chan);
		__ssi_disable_tx_error_intr(hw->chnl); /* disable underrun irq here ??? */
		g_jz_intr->dma_tx_end++;

		print_msg("DMA Write End\n");
		print_msg("DMA Tx FIFO = %d\n",__ssi_get_txfifo_count(hw->chnl));
		/* sometimes it don't reach the SSI rx trigger so it wouldn't send dma rx request 
		for receiving last part of data from RxFIFO when the length was not alignd */
		if((hw->dma_rx_unit/(hw->rxfifo_width>>3)) < 8 || hw->dma_rx_chnl < 0 ){		
			__ssi_clear_errors(hw->chnl);
			__ssi_enable_tx_error_intr(hw->chnl);
			return IRQ_HANDLED;
		}
		
		return IRQ_HANDLED;
	}
	
	dev_err(hw->dev,"DMA others tx int error\n");
	complete(&hw->done);
	return IRQ_HANDLED;
}
static irqreturn_t spi_dma_rx_irq(int irq, void *dev)
{
	struct jz47xx_spi *hw = dev;
	struct jz_intr_cnt *g_jz_intr = hw->g_jz_intr;
	int chan = hw->dma_rx_chnl;
	
	g_jz_intr->dma_rx_cnt++;
	disable_dma(chan);
	if (__dmac_channel_address_error_detected(chan)) {
		dev_err(hw->dev,"%s: DMAC address error.\n", __FUNCTION__);
		g_jz_intr->dma_rx_err++;
		__dmac_channel_clear_address_error(chan);
		complete(&hw->done);
		
		return IRQ_HANDLED;
	}
	if (__dmac_channel_transmit_end_detected(chan)) {
		g_jz_intr->dma_rx_end++;
		__dmac_channel_clear_transmit_end(chan);

		print_msg("DMA Read End\n");
		
		print_msg("DMA Rx FIFO = %d\n",__ssi_get_rxfifo_count(hw->chnl));

		if(!(hw->dma_flag & SPI_DMA_ACK)){
				hw->dma_flag |= SPI_DMA_ACK;
				complete(&hw->done);
		}

		return IRQ_HANDLED;
	}
	
	dev_err(hw->dev,"DMA others rx int error\n");
	complete(&hw->done);
	return IRQ_HANDLED;
}
static int jz_spi_dma_init(struct jz47xx_spi *hw,int flag)
{
	int tx_dma_id,rx_dma_id;
	print_dbg("SPI Master %d request DMA!\n",hw->chnl);
	if(hw->chnl == 0){
		tx_dma_id = DMA_ID_SSI0_TX;
		rx_dma_id = DMA_ID_SSI0_RX;
	}else if(hw->chnl == 1){
		tx_dma_id = DMA_ID_SSI1_TX;
		rx_dma_id = DMA_ID_SSI1_RX;
	}else{
		printk(KERN_ERR "SSI %d DMA request failed (channel error)!\n",hw->chnl);
		return -EINVAL;
	}

	if ((hw->dma_tx_chnl= jz_request_dma(tx_dma_id, "SSI Tx DMA", 
		spi_dma_tx_irq,IRQ_DISABLED, hw)) < 0 ) {

		print_msg("SSI %d Tx DMA request failed(ENO=%d)!\n",hw->chnl,hw->dma_tx_chnl);
		return -EINVAL;
	}
	if(flag == 0 || (hw->rw_mode & R_DMA)){
		if ((hw->dma_rx_chnl = jz_request_dma(rx_dma_id, "SSI Rx DMA", 
			spi_dma_rx_irq,IRQ_DISABLED, hw)) < 0 ) {

			print_msg("SSI %d Rx DMA request failed(ENO=%d)!\n",hw->chnl,hw->dma_rx_chnl);
			return -EINVAL;
		}
	}else{
		print_msg("SSI %d didn't request Rx DMA !!!\n",hw->chnl);
	}
	
	return 0;
}
int jz_spi_pinconfig(struct jz47xx_spi *hw)
{
	u8 f_gpiocs=0,f_spics0=0,f_spics1=0;
	u16 i,*p_chipselect;
	struct jz47xx_spi_info 	*pdata = hw->pdata;
	struct spi_board_info	*board_info = pdata->board_info;

	if(pdata->board_size > MAX_SPI_DEVICES)
	{
		pdata->board_size = MAX_SPI_DEVICES;
		dev_info(hw->dev,"SPI devices exceed defined max_num!!!\n");
	}

	if(board_info){
		board_info[0].chip_select = 77;
		for(i = 0; i< pdata->board_size; i++){
			p_chipselect = &(board_info[i].chip_select);
		
#ifdef CONFIG_JZ_SPI_BOARD_INFO_REGISTER
			if(*p_chipselect < 32)
				dev_info(hw->dev,"Warnning:chip select PIN is 32*%d+%d , it's right?\n",
					(*p_chipselect)/32,(*p_chipselect)%32);
			
			if(*p_chipselect == PIN_SSI_CE0){ 
				*p_chipselect = (!hw->chnl) ? SSI0_CE0_PIN : SSI1_CE0_PIN;
				f_spics0 = 1;
			}else if(*p_chipselect == PIN_SSI_CE1){ 	
				*p_chipselect = (!hw->chnl) ? SSI0_CE1_PIN : SSI1_CE1_PIN;
				f_spics1 = 1;
			}else									
				f_gpiocs = 1;
#else
			if(*p_chipselect < 32){
				dev_info(hw->dev,"Warnning:chip select PIN is 32*%d+%d , it's right?\n",
					(*p_chipselect)/32,(*p_chipselect)%32);
				if(*p_chipselect < 2)
					return -EINVAL;
					
			}
			f_gpiocs = 1;
#endif
			__gpio_as_output(*p_chipselect);
			
			print_msg("chip select PIN:0x%04X\n",*p_chipselect);
		}
		
	}
	else
		f_gpiocs = 1;

	print_msg("f_spics0=%d\nf_spics1=%d\nf_gpiocs=%d\n",f_spics0,f_spics1,f_gpiocs);

	
	if(pdata->pins_config){			/* if user want to configure by themself */
		pdata->pins_config();
		return 0;
	}
	
	if(f_spics0|| f_spics1){		/* spi chipselect for ssi controller internal */

		/* one of two controllers in SOC 
		 *
		 * ??? pin_output function  instead of controller internal_chipselect, because ...
		 *
		 */
		if(f_spics1)
			__ssi_select_ce2(hw->chnl);
		if(f_spics0)
			__ssi_select_ce(hw->chnl);
		
		GPIO_AS_SSI(hw->chnl);
	}
	
	if(f_gpiocs){					/* config SPI_PINs for spi function except for CE0 and CE1 
	 *
	 * SPI_PINs: SSI0_CLK, SSI0_DT, SSI0_DR
	 */
		
		GPIO_AS_SSI_EX(hw->chnl);
	}
	
	return 0;
}
EXPORT_SYMBOL_GPL(jz_spi_pinconfig);
	
static int jz47xx_spi_init_setup(struct jz47xx_spi *hw)
{
		
	/* for the moment,open the SSI clock gate */

	/* disable the SSI controller */
	__ssi_disable(hw->chnl);
	
	CPM_SSI_START(hw->chnl);
	
	/* set default half_intr trigger */
	hw->tx_trigger = SSI_TX_FIFO_THRESHOLD*8;
	hw->rx_trigger = SSI_RX_FIFO_THRESHOLD*8;
	__ssi_set_tx_trigger(hw->chnl,hw->tx_trigger);
	__ssi_set_rx_trigger(hw->chnl,hw->rx_trigger);

	/* First,mask the interrupt, while verify the status ? */
	__ssi_disable_tx_intr(hw->chnl);
	__ssi_disable_rx_intr(hw->chnl);

	__ssi_disable_receive(hw->chnl);

	__ssi_set_spi_clock_phase(hw->chnl,0);
	__ssi_set_spi_clock_polarity(hw->chnl,0);  
	__ssi_set_msb(hw->chnl);
	__ssi_spi_format(hw->chnl);
	__ssi_set_frame_length(hw->chnl,8);
	__ssi_disable_loopback(hw->chnl);   
	__ssi_flush_fifo(hw->chnl);

	__ssi_underrun_auto_clear(hw->chnl);
 	__ssi_clear_errors(hw->chnl);
		
	return 0;
}


static int jz_ssi_clk_setup(struct jz47xx_spi *hw)
{
#ifdef JZ_NEW_CODE_TYPE
	if(hw->pdata->is_pllclk){
		__ssi_select_pllclk();
	}else{
		__ssi_select_exclk();
	}
	__cpm_set_ssidiv(0);
	hw->src_clk = cpm_get_clock(CGU_SSICLK);
#else
	hw->src_clk = __cpm_get_pllout();
	hw->pdata->is_pllclk = 1;
#endif
	
	return 0;
}

static int __init jz47xx_spi_probe(struct platform_device *pdev)
{
	struct jz47xx_spi *hw;
	struct spi_master *master;
	
	struct resource *res;
	int err = 0;
	
#ifdef CONFIG_JZ_SPI_BOARD_INFO_REGISTER
	int i;
	struct spi_board_info *bi;
#endif

	print_msg("in %s\n",__FUNCTION__);
	master = spi_alloc_master(&pdev->dev, sizeof(struct jz47xx_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}
	
	hw = spi_master_get_devdata(master);
	memset(hw, 0, sizeof(struct jz47xx_spi));

	hw->g_jz_intr = kzalloc(sizeof(struct jz_intr_cnt),GFP_KERNEL);

	if(hw->g_jz_intr == NULL)
	{
		dev_err(&pdev->dev, "No memory for jz_intr_cnt\n");
		err = -ENOMEM;
		goto err_nomem;
	}
	
	hw->master = spi_master_get(master);
	hw->dev = &pdev->dev;
	
	hw->pdata = pdev->dev.platform_data;
	if (hw->pdata == NULL) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		err = -ENOENT;
		goto err_no_pdata;
	}
	hw->chnl= hw->pdata->chnl;
	
	if(hw->chnl != 0 && hw->chnl != 1){
		dev_err(&pdev->dev, "No this channel\n");
		err = -ENOENT;
		goto err_no_pdata;
	}
	
	platform_set_drvdata(pdev, hw);
	init_completion(&hw->done);

	/* setup the state for the bitbang driver */

	hw->bitbang.master         = hw->master;
	hw->bitbang.setup_transfer = jz47xx_spi_setupxfer;
	hw->bitbang.chipselect     = jz47xx_spi_chipsel;
	hw->bitbang.txrx_bufs      = jz47xx_spi_txrx;
	hw->bitbang.master->setup  = jz47xx_spi_setup;

	dev_dbg(hw->dev, "bitbang at %p\n", &hw->bitbang);

	/* find and map our resources */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_no_iores;
	}
	hw->ioarea = request_mem_region(res->start, (res->end - res->start)+1,
					pdev->name);

	if (hw->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve region\n");
		err = -ENXIO;
		goto err_no_iores;
	}
	hw->regs = ioremap(res->start, (res->end - res->start)+1);
	if (hw->regs == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_no_iomap;
	}
	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_no_irq;
	}

	hw->dma_tx_chnl = DMA_INVALID;
	hw->dma_rx_chnl = DMA_INVALID;
	if(hw->chnl == SSI_DMA_FASTNESS_CHNL){
		err = jz_spi_dma_init(hw,0);
		if (err) {
			dev_err(&pdev->dev, "Cannot claim DMA IRQ\n");
			/* release DMA channel */
			if (hw->dma_rx_chnl >= 0)
				jz_free_dma(hw->dma_rx_chnl);
			
			if (hw->dma_tx_chnl >= 0)
				jz_free_dma(hw->dma_tx_chnl);
			
			goto err_no_irq;
		}
	}
	/* request SSI irq */
	err = request_irq(hw->irq, jz47xx_spi_irq, 0, pdev->name, hw);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_no_irq;
	}
	
	/* get controller associated params */
	master->bus_num = hw->pdata->bus_num;
	master->num_chipselect = hw->pdata->num_chipselect;


	/* setup chipselect */
	if (hw->pdata->set_cs)
		hw->set_cs = hw->pdata->set_cs; 
	else
		hw->set_cs = jz47xx_spi_cs;

	/* SSI controller clock configure */
	jz_ssi_clk_setup(hw);
	/* SSI controller initializations for SPI */
	jz47xx_spi_init_setup(hw);
	/* SPI_PINs and chipselect configure */
	err = jz_spi_pinconfig(hw);
	if(err){
		dev_err(&pdev->dev, "PINs configure error\n");
		goto err_register;
	}
		
	/* register our spi controller */
	err = spi_bitbang_start(&hw->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master ERR_NO:%d\n",err);
		goto err_register;
	}

#ifdef CONFIG_JZ_SPI_BOARD_INFO_REGISTER	
	/* register all the devices associated */
	bi = &hw->pdata->board_info[0];
	if(bi){
		for (i = 0; i < hw->pdata->board_size; i++, bi++) {
			dev_info(hw->dev, "registering %s\n", bi->modalias);

			bi->controller_data = hw;
			spi_new_device(master, bi);
		}
	}
#endif

	printk(KERN_INFO
	       "JZ47xx SSI Controller for SPI channel %d driver register\n",hw->chnl);
	
	return 0;

 err_register:
	CPM_SSI_STOP(hw->chnl);

	free_irq(hw->irq, hw);

 err_no_irq:
	iounmap(hw->regs);

 err_no_iomap:
	release_resource(hw->ioarea);
	kfree(hw->ioarea);

 err_no_iores:
 err_no_pdata:
	spi_master_put(hw->master);;

 err_nomem:
	return err;
}

static int __exit jz47xx_spi_remove(struct platform_device *dev)
{
	struct jz47xx_spi *hw = platform_get_drvdata(dev);

	CPM_SSI_STOP(hw->chnl);
	__ssi_disable_tx_intr(hw->chnl);
	__ssi_disable_rx_intr(hw->chnl);
	__ssi_disable(hw->chnl);

	spi_master_put(hw->master);
	spi_bitbang_stop(&hw->bitbang);

	platform_set_drvdata(dev, NULL);
	
	free_irq(hw->irq, hw);
	iounmap(hw->regs);

	release_resource(hw->ioarea);
	kfree(hw->ioarea);

	/* release DMA channel */
	if (hw->dma_rx_chnl >= 0) {
		jz_free_dma(hw->dma_rx_chnl);
		printk("dma_rx_chnl release\n");
	}
	if (hw->dma_tx_chnl >= 0) {
		jz_free_dma(hw->dma_tx_chnl);
		printk("dma_tx_chnl release\n");
	}

	kfree(hw->g_jz_intr);
	kfree(hw);
	printk(KERN_INFO
	       "JZ47xx SSI Controller for SPI channel %d driver removed\n",hw->chnl);
	
	return 0;
}
#ifdef CONFIG_PM

static int jz47xx_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct jz47xx_spi *hw = platform_get_drvdata(pdev);

	CPM_SSI_STOP(hw->chnl);

	return 0;
}

static int jz47xx_spi_resume(struct platform_device *pdev)
{
	struct jz47xx_spi *hw = platform_get_drvdata(pdev);

	CPM_SSI_START(hw->chnl);

	return 0;
}

#else
#define jz47xx_spi_suspend NULL
#define jz47xx_spi_resume  NULL
#endif

MODULE_ALIAS("jz47xx_spi");			/* for platform bus hotplug */
static struct platform_driver jz47xx_spidrv = {
	.remove		= __exit_p(jz47xx_spi_remove),
	.suspend	= jz47xx_spi_suspend,
	.resume		= jz47xx_spi_resume,
	.driver		= {
		.name	= "jz47xx-spi",
		.owner	= THIS_MODULE,
	},
};
static int __init jz47xx_spi_init(void)
{
        return platform_driver_probe(&jz47xx_spidrv, jz47xx_spi_probe);
}

static void __exit jz47xx_spi_exit(void)
{
        platform_driver_unregister(&jz47xx_spidrv);
		printk(KERN_INFO "JZ47xx SSI Controller Module EXIT\n");

}
module_init(jz47xx_spi_init);
module_exit(jz47xx_spi_exit);

MODULE_DESCRIPTION("JZ47XX SPI Driver");
MODULE_LICENSE("GPL");

