/*
 * jz_tssi.c
 *
 * MPEG2-TS interface driver for the Ingenic JZ47XX.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/jzsoc.h>

#include "jzchars.h"
#include "jz_tssi.h"


MODULE_AUTHOR("Lucifer Liu <yliu@ingenic.cn>");
MODULE_DESCRIPTION("Ingenic MPEG2-TS interface Driver");
MODULE_LICENSE("GPL");

#define TSSI_NAME "tssi"
#define TSSI_MINOR 204         /* MAJOR: 10, MINOR: 16 */
#define TSSI_IRQ   IRQ_TSSI
#define PFX        TSSI_NAME
#define RING_BUF_NUM  100

#define USE_DMA
#define TRIG_PIN    ( 32 * 2 + 15 )
#define DMA_ID_TSSI 1
//#define JZ_TSSI_DEBUG

#ifdef JZ_TSSISI_DEBUG
#define dbg(format, arg...) printk(KERN_DEBUG PFX ": " format "\n" , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif
#define err(format, arg...) printk(KERN_ERR PFX ": " format "\n" , ## arg)
#define info(format, arg...) printk(KERN_INFO PFX ": " format "\n" , ## arg)
#define warn(format, arg...) printk(KERN_WARNING PFX ": " format "\n" , ## arg)

static struct jz_tssi_t jz_tssi_g;
static struct jz_tssi_buf_ring_t jz_tssi_ring_g;
static int tssi_dma_reinit(int dma_chan, unsigned char *dma_buf, int size);

static void dump_tssi_regs( void )
{
	printk("REG_TSSI_ENA   %8x \n ",    REG8( TSSI_ENA ));
	printk("REG_TSSI_CFG   %8x \n ",    REG16( TSSI_CFG ));
	printk("REG_TSSI_CTRL  %8x \n ",    REG8( TSSI_CTRL ));
	printk("REG_TSSI_STAT  %8x \n ",    REG8( TSSI_STAT ));
	printk("REG_TSSI_FIFO  %8x \n ",    REG32( TSSI_FIFO ));
	printk("REG_TSSI_PEN   %8x \n ",    REG32( TSSI_PEN ));
	printk("REG_TSSI_PID0  %8x \n ",    REG32( TSSI_PID0 ));
	printk("REG_TSSI_PID1  %8x \n ",    REG32( TSSI_PID1 ));
	printk("REG_TSSI_PID2  %8x \n ",    REG32( TSSI_PID2 ));
	printk("REG_TSSI_PID3  %8x \n ",    REG32( TSSI_PID3 ));
	printk("REG_TSSI_PID4  %8x \n ",    REG32( TSSI_PID4 ));
	printk("REG_TSSI_PID5  %8x \n ",    REG32( TSSI_PID5 ));
	printk("REG_TSSI_PID6  %8x \n ",    REG32( TSSI_PID6 ));
	printk("REG_TSSI_PID7  %8x \n ",    REG32( TSSI_PID7 ));
}

void dump_dma_channel(unsigned int dmanr)
{
	printk("DMA%d Registers:\n", dmanr);
	printk("  DMACR  = 0x%8x\n", REG_DMAC_DMACR(0));
	printk("  DSAR   = 0x%8x\n", REG_DMAC_DSAR(dmanr));
	printk("  DTAR   = 0x%8x\n", REG_DMAC_DTAR(dmanr));
	printk("  DTCR   = 0x%8x\n", REG_DMAC_DTCR(dmanr));
	printk("  DRSR   = 0x%8x\n", REG_DMAC_DRSR(dmanr));
	printk("  DCCSR  = 0x%8x\n", REG_DMAC_DCCSR(dmanr));
	printk("  DCMD  = 0x%8x\n", REG_DMAC_DCMD(dmanr));
	printk("  DDA  = 0x%8x\n", REG_DMAC_DDA(dmanr));
	printk("  DMADBR = 0x%8x\n", REG_DMAC_DMADBR(1));
}

static int tssi_buf_init( struct jz_tssi_buf_ring_t * ring ) 	
{
	int i;
	struct jz_tssi_buf * bp,* ap, *cp;

	ap = cp = bp = (struct jz_tssi_buf *)kmalloc( sizeof( struct jz_tssi_buf ) ,GFP_KERNEL );  //the first
	if ( !bp ) { 
		printk("Can not malloc buffer! \n");
		return -1;
	}

	for ( i = 0; i < RING_BUF_NUM; i ++ ) {
		bp = ap;
		bp->buf = (unsigned int *) kmalloc(MPEG2_TS_PACHAGE_SIZE / 4 * sizeof(unsigned int) ,GFP_KERNEL);
		if ( !bp->buf ) {        
			printk("Can not malloc buffer! \n");
			return -1;
		}
		memset(bp->buf, 0, MPEG2_TS_PACHAGE_SIZE);
		bp->index = i;
		bp->pos = 0;
		ap = (struct jz_tssi_buf *)kmalloc( sizeof( struct jz_tssi_buf ) ,GFP_KERNEL );
		if ( !ap ) { 
			printk("Can not malloc buffer! \n");
			return -1;
		}

		bp->next = ap;      //point to next !
	}

	bp->next = cp;                  //point loop to first!
	ring->front = cp;
	ring->rear  = cp;
	ring->fu_num = 0;
	kfree(ap);
	return 0;
}

static void tssi_free_buf( struct jz_tssi_buf_ring_t * ring )
{
	int i;
	struct jz_tssi_buf * ap;
	for ( i = 0; i < RING_BUF_NUM; i ++ ) 
	{
		ap = ring->front;
		ring->front = ring->front->next;
		kfree( ap );
	}
}

#if 0
static void tssi_read_fifo(void *dev_id)
{
	struct jz_tssi_t* tssi = ( struct jz_tssi_t* )dev_id;
	struct jz_tssi_buf_ring_t * ring = tssi->cur_buf;
	struct jz_tssi_buf *buf = ring->rear;
	int i;
#if 0
	if ( ring->fu_num > RING_BUF_NUM )
	{
		printk("Ring buffer full ! %d \n",ring->fu_num);
		return;
	}
#endif
	
	for ( i = 0; i < 8 ; i ++ )
	{
		ring->front->buf[ring->front->pos++] = REG_TSSI_FIFO;
	}

	if ( ring->front->pos >= MPEG2_TS_PACHAGE_SIZE ) 
	{
		ring->fu_num ++;
		ring->front = ring->front->next;
		ring->front->pos = 0;
	}
}
#endif

static void tssi_config_filting( void )
{
	__gpio_as_tssi_1();
	__tssi_disable_ovrn_irq();         //use dma ,no need irq
	__tssi_disable_trig_irq();
	__tssi_set_tigger_num( 4 );        //trig is 4 word!
//	__tssi_filter_enable();
//	__tssi_set_data0_mode(1);	/* add data 0 after stream */
	__tssi_filter_disable();
	__tssi_state_clear_overrun();
//	__tssi_clear_trig_irq_flag();
	__tssi_dma_enable();

//	__tssi_enable_ovrn_irq();
//	__tssi_enable_trig_irq();

	//set config
//	__tssi_set_bt_1();
	__tssi_set_wd_1();
//	__tssi_set_data_use_data7();
	__tssi_set_data_pola_high();
//	__tssi_select_serail_mode();
	__tssi_select_paral_mode();
	__tssi_select_clk_fast();
	__tssi_select_clk_posi_edge();
	__tssi_select_frm_act_high();
	__tssi_select_str_act_high();
	__tssi_select_fail_act_high();
//	__tssi_select_fail_act_low();
//	__tssi_disable_filte_pid0();     //we disable pid0 filter for ever!
	REG_TSSI_ENA &= ~(1 << 3);
	REG_TSSI_CTRL = 5;
}

static void tssi_add_pid(int pid_num, int pid)
{
	unsigned int addr ;
	int n = pid_num / 2, hl = pid_num % 2;
	if ( hl )      //use high pid, pid1
	{ 
		addr = TSSI_PID0 + ( n * 4 );
		REG32( addr ) |= ( (pid & 0x1fff) << 16 );    //13bit
		REG_TSSI_PEN |= ( 1 << (16 + n) );
	}
	else           //use low  pid, pid0
	{
		addr = TSSI_PID0 + ( n * 4 );
		REG32( addr ) |= pid & 0x1fff;    //13bit
		REG_TSSI_PEN |= ( 1 << n  );
	}
}

static irqreturn_t tssi_dma_irq(int irq, void * dev_id)
{
	struct jz_tssi_t *tssi = (struct jz_tssi_t *)dev_id;
	struct jz_tssi_buf_ring_t *buf = tssi->cur_buf;

#if 1
	if ( REG_TSSI_STAT & TSSI_STAT_OVRN )
	{
		printk("tssi over run occur! stat = %x\n",REG8( TSSI_STAT ));
		__tssi_clear_state();
	while(1);
	}
#endif
	REG_DMAC_DCCSR(tssi->dma_chan) &= ~DMAC_DCCSR_EN;  /* disable DMA */

	if (__dmac_channel_transmit_end_detected(tssi->dma_chan)) {
		__dmac_channel_clear_transmit_end(tssi->dma_chan);
		if ( buf->fu_num < RING_BUF_NUM )
		{
			buf->front = buf->front->next;
			REG_DMAC_DSAR(tssi->dma_chan) = CPHYSADDR(TSSI_FIFO);
			REG_DMAC_DTAR(tssi->dma_chan) = CPHYSADDR((unsigned int)buf->front->buf);
			REG_DMAC_DTCR(tssi->dma_chan) = MPEG2_TS_PACHAGE_SIZE / 16;
			REG_DMAC_DCCSR(tssi->dma_chan) = DMAC_DCCSR_NDES | DMAC_DCCSR_EN;
			buf->fu_num ++;
			if (buf->fu_num == 1)
				wake_up(&tssi->wait);
		} else {
			printk("cpu get slow buf over run.\n");
//			while(1);
		}
		__tssi_clear_state();
	}

	if (__dmac_channel_transmit_halt_detected(tssi->dma_chan)) {
		printk("DMA HALT\n");
		__dmac_channel_clear_transmit_halt(tssi->dma_chan);
	}

	if (__dmac_channel_address_error_detected(tssi->dma_chan)) {
		printk("DMA ADDR ERROR\n");
		__dmac_channel_clear_address_error(tssi->dma_chan);
	}

#if 0
	if (__dmac_channel_descriptor_invalid_detected(tssi->dma_chan)) {
		printk("DMA DESC INVALID\n");
		__dmac_channel_clear_descriptor_invalid(tssi->dma_chan);
	}

	if (__dmac_channel_count_terminated_detected(tssi->dma_chan)) {
		printk("DMA CT\n");
		__dmac_channel_clear_count_terminated(tssi->dma_chan);
	}
#endif

	return IRQ_HANDLED;
}

static irqreturn_t tssi_interrupt(int irq, void * dev_id)
{
	__intc_mask_irq(TSSI_IRQ);
#if 1
	if ( REG_TSSI_STAT & TSSI_STAT_OVRN )
	{
		printk("tssi over run occur! %x\n",REG8( TSSI_STAT ));
		while(1);
		__tssi_clear_state();
//		printk("clear ! %x\n",REG8( TSSI_STAT ));
	}
#endif
	if ( REG_TSSI_STAT & TSSI_STAT_TRIG )
	{
		printk("tssi trig irq occur! \n");
//		tssi_read_fifo( dev_id );
	}

	__intc_ack_irq(TSSI_IRQ);
	__intc_unmask_irq(TSSI_IRQ);
	return IRQ_HANDLED;
}

static ssize_t jz_read(struct file * filp, char * buffer, size_t count, loff_t * ppos)
{
	jz_char_dev_t *adev = (jz_char_dev_t *)filp->private_data;
	struct jz_tssi_t* tssi = (struct jz_tssi_t*)adev->private;
	struct jz_tssi_buf_ring_t* ring = tssi->cur_buf;
	int rv;
	int i;

	rv = wait_event_interruptible(tssi->wait, (ring->fu_num != 0));
	if (rv) {
		printk("wait event interrupt by user space.\n");
		return -ERESTARTSYS;
	}

	count /= MPEG2_TS_PACHAGE_SIZE;

	mutex_lock(&tssi->lock);
	if ( count > ring->fu_num )
		count = ring->fu_num;
	mutex_unlock(&tssi->lock);

	for ( i = 0; i < count; i ++ )
	{
		copy_to_user( buffer + ( i * MPEG2_TS_PACHAGE_SIZE),
			ring->rear->buf, MPEG2_TS_PACHAGE_SIZE );
		ring->rear->pos = 0;
		ring->rear = ring->rear->next;
	}
	mutex_lock(&tssi->lock);
	ring->fu_num -= count;
	mutex_unlock(&tssi->lock);
	return count * MPEG2_TS_PACHAGE_SIZE;
}

static int tssi_dma_reinit(int dma_chan, unsigned char *dma_buf, int size)
{
	static unsigned int dma_src_phys_addr, dma_dst_phys_addr;
	REG_DMAC_DMACKE(0) = 0xff;
	dma_src_phys_addr = CPHYSADDR(TSSI_FIFO);
	dma_dst_phys_addr = CPHYSADDR((unsigned int)dma_buf);
	REG_DMAC_DMACR(dma_chan/HALF_DMA_NUM) = 0;
	REG_DMAC_DCCSR(dma_chan) = 0;
	REG_DMAC_DRSR(dma_chan) = DMAC_DRSR_RS_TSSIIN;
	REG_DMAC_DSAR(dma_chan) = dma_src_phys_addr;
	REG_DMAC_DTAR(dma_chan) = dma_dst_phys_addr;
//	REG_DMAC_DTCR(dma_chan) = size / 32;                 
	REG_DMAC_DTCR(dma_chan) = size / 16;                 
	REG_DMAC_DCMD(dma_chan) = DMAC_DCMD_DAI | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_16BYTE | DMAC_DCMD_TIE;
	REG_DMAC_DCCSR(dma_chan) = DMAC_DCCSR_NDES | DMAC_DCCSR_EN;
	REG_DMAC_DMACR(dma_chan/HALF_DMA_NUM) = DMAC_DMACR_DMAE; /* global DMA enable bit */
	return 0;
}

static int jz_open(struct inode * inode, struct file * filp)
{
	try_module_get(THIS_MODULE);

	__intc_mask_irq(TSSI_IRQ);
	tssi_config_filting();
	__tssi_soft_reset();
	__tssi_clear_state();

	return 0;
}

static int jz_release(struct inode * inode, struct file * filp)
{
	__intc_mask_irq(TSSI_IRQ);
 	module_put(THIS_MODULE);
	return 0;
}

static int jz_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	jz_char_dev_t *adev = (jz_char_dev_t *)file->private_data;
	struct jz_tssi_t* tssi = (struct jz_tssi_t*)adev->private;

	switch (cmd)
	{
	case IOCTL_TSSI_ENABLE :
		__tssi_disable();
		__tssi_soft_reset();
		__tssi_clear_state();
		dump_tssi_regs();
//		__intc_ack_irq(TSSI_IRQ);
//		__intc_unmask_irq(TSSI_IRQ);
		__tssi_enable();
        	break;
	case IOCTL_TSSI_DISABLE :
		__tssi_disable();
		__tssi_soft_reset();
		__tssi_clear_state();

        	break;
	case IOCTL_TSSI_SOFTRESET :
		__tssi_soft_reset();

        	break;
	case IOCTL_TSSI_ENFILTER :
		__tssi_filter_enable();
        	break;
	case IOCTL_TSSI_DEFILTER :
		__tssi_filter_disable();
        	break;
	case IOCTL_TSSI_ADDPID :           //add one pid to filter
		if ( tssi->pid_num < 15 )
		{
			tssi_add_pid(tssi->pid_num, arg);
			tssi->pid_num ++ ;
		}
		break;

	case IOCTL_TSSI_FLUSHPID :               //set all filting pid to false
		REG_TSSI_PEN = 0x0;		
		REG_TSSI_PID0 = 0x0;
		REG_TSSI_PID1 = 0x0;
		REG_TSSI_PID2 = 0x0;
		REG_TSSI_PID3 = 0x0;
		REG_TSSI_PID4 = 0x0;
		REG_TSSI_PID5 = 0x0;
		REG_TSSI_PID6 = 0x0;
		REG_TSSI_PID7 = 0x0;
		break;

	case IOCTL_TSSI_INIT_DMA:
  		tssi_dma_reinit(tssi->dma_chan, tssi->cur_buf->front->buf, MPEG2_TS_PACHAGE_SIZE);
		break;
	case IOCTL_TSSI_DISABLE_DMA:
		REG_DMAC_DCCSR(tssi->dma_chan) &= ~DMAC_DCCSR_EN;  /* disable DMA */
		break;
	}

	return 0;
}

static struct file_operations tssi_fops = {
	.owner		=	THIS_MODULE,
	.read		=       jz_read,
	.ioctl		=       jz_ioctl,
	.open		=	jz_open,
	.release	=	jz_release,
};

static int __init jztssi_init_module(void)
{
	int retval;
	struct jz_tssi_t *tssi = &jz_tssi_g;

	cpm_start_clock(CGM_TSSI);
	cpm_start_clock(CGM_DMAC);

	mutex_init(&tssi->lock);
	init_waitqueue_head(&tssi->wait);

	
	tssi_buf_init( &jz_tssi_ring_g );
	tssi->cur_buf = &jz_tssi_ring_g;
	tssi->pid_num = 0;
	retval = request_irq(TSSI_IRQ, tssi_interrupt, IRQF_DISABLED, TSSI_NAME, &jz_tssi_g);

	if (retval) {
		printk("unable to get IRQ %d",TSSI_IRQ);
		return retval;
	}

	tssi->dma_chan = jz_request_dma(DMA_ID_TSSI, "tssi", tssi_dma_irq,
				  IRQF_DISABLED, &jz_tssi_g);
	if ( tssi->dma_chan < 0 )
	{
		printk("MPEG2-TS request irq fail! \n");
		return -1;
	}

	jz_register_chrdev(TSSI_MINOR, TSSI_NAME, &tssi_fops, &jz_tssi_g);	

	printk(JZ_SOC_NAME": MPEG2-TS interface driver registered %x %d\n",&jz_tssi_g,tssi->dma_chan);
	return 0;
}

static void __exit jztssi_cleanup_module(void)
{
	free_irq(TSSI_IRQ,0);
	jz_free_dma(jz_tssi_g.dma_chan);
	tssi_free_buf( &jz_tssi_ring_g );
	jz_unregister_chrdev(TSSI_MINOR, TSSI_NAME);
}

module_init(jztssi_init_module);
module_exit(jztssi_cleanup_module);
