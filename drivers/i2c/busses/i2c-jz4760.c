
/*
 * I2C adapter for the INGENIC I2C bus access.
 *
 * Copyright (C) 2006 - 2009 Ingenic Semiconductor Inc.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <linux/module.h>
#include <asm/addrspace.h>

#include <asm/jzsoc.h>
#include "i2c-jz4760.h"

/* I2C protocol */
#define I2C_READ	1
#define I2C_WRITE	0
#define I2C_CLIENT_NUM  20
#define TIMEOUT         0xffff

//#undef DEBUG
#define DEBUG 
#ifdef DEBUG
#define dprintk(x...)	printk(x)
#else
#define dprintk(x...) do{}while(0)
#endif

#define __para_printk()\
do {\
dprintk("cmd_flag=%d,cmd_cnt=%d,r_cnt=%d length=%d",cmd_flag[I2C_ID],cmd_cnt[I2C_ID],r_cnt[I2C_ID],length);\
} while(0)

#define __reg_printk()							\
do {								        \
dprintk("  REG_I2C_STA(%d)=0x%x",I2C_ID,REG_I2C_STA(I2C_ID));		\
dprintk("  REG_I2C_TXTL(%d)=0x%x",I2C_ID,REG_I2C_TXTL(I2C_ID));		\
dprintk("  REG_I2C_INTST(%d)=0x%x",I2C_ID,REG_I2C_INTST(I2C_ID));	\
dprintk("  REG_I2C_TXABRT(%d)=0x%x\n",I2C_ID,REG_I2C_TXABRT(I2C_ID));	\
dprintk("------------------%s:%d\n",__FUNCTION__,__LINE__);				\
} while(0)

struct i2c_speed {
	unsigned int speed;
	unsigned char slave_addr;
};
static  struct i2c_speed jz4760_i2c_speed[I2C_CLIENT_NUM];
static unsigned char current_device[2]={0x0,0x0};
static int client_cnt = 0;
static int i2c_ctrl_rest[2]  = {0,0};
struct jz_i2c {
	spinlock_t		lock;
	wait_queue_head_t	wait;
	int                     id;
	unsigned int            irq;
	struct i2c_msg		*msg;
	unsigned int		msg_num;
	unsigned int		slave_addr;
	struct i2c_adapter	adap;
	struct clk		*clk;
};

void i2c_jz_setclk(struct i2c_client *client,unsigned long i2cclk)
{
	if (i2cclk > 0 && i2cclk <= 400000) {
		jz4760_i2c_speed[client_cnt].slave_addr = client->addr;
		jz4760_i2c_speed[client_cnt].speed      = i2cclk/1000;
	} else if (i2cclk <= 0) {
		jz4760_i2c_speed[client_cnt].slave_addr = client->addr;
		jz4760_i2c_speed[client_cnt].speed      = 100;
	} else {
		jz4760_i2c_speed[client_cnt].slave_addr = client->addr;
		jz4760_i2c_speed[client_cnt].speed      = 400;
	}

	printk("Device 0x%2x with i2c speed:%dK\n",jz4760_i2c_speed[client_cnt].slave_addr,
	       jz4760_i2c_speed[client_cnt].speed );

	client_cnt++;
}
EXPORT_SYMBOL_GPL(i2c_jz_setclk);

/* 
 *jz_i2c_irq
*/
static unsigned char *msg_buf0,*msg_buf1;
static int cmd_cnt[2];
static volatile int cmd_flag[2];
static int r_cnt[2];

static irqreturn_t jz_i2c_irq(int irqno, void *dev_id)
{
	struct jz_i2c *i2c = dev_id;
	int I2C_ID = i2c->id;
	int flags = i2c->msg->flags;
	int timeout = TIMEOUT;
	
	if (__i2c_abrt(I2C_ID) || __i2c_abrt_intr(I2C_ID)) {
		cmd_flag[I2C_ID] = -1;
		//printk("*****jz4760 i2c abort.\n");
		//__reg_printk();
		//__i2c_clear_interrupts(ret,I2C_ID);
		REG_I2C_INTM(I2C_ID) = 0x0;
		return IRQ_HANDLED;
	}
	/* first byte,when length > 1 */
	if (cmd_flag[I2C_ID] == 0 && cmd_cnt[I2C_ID] > 1) { 
		cmd_flag[I2C_ID] = 1;
		if (flags & I2C_M_RD) {			
			REG_I2C_DC(I2C_ID) = I2C_READ << 8;
		} else {
			if (I2C_ID == 0) {
				REG_I2C_DC(I2C_ID) = (I2C_WRITE << 8) | *msg_buf0++;
			} else {
				REG_I2C_DC(I2C_ID) = (I2C_WRITE << 8) | *msg_buf1++;
			}
		}
		cmd_cnt[I2C_ID]--;
	}

	if (flags & I2C_M_RD) {
		if (REG_I2C_STA(I2C_ID) & I2C_STA_RFNE) {
			if (I2C_ID == 0) {
				*msg_buf0++ = REG_I2C_DC(I2C_ID) & 0xff;
			} else {
				*msg_buf1++ = REG_I2C_DC(I2C_ID) & 0xff;
			}
			r_cnt[I2C_ID]--;
		}

		REG_I2C_DC(I2C_ID) = I2C_READ << 8;
	} else {
		if (I2C_ID == 0) {
			REG_I2C_DC(I2C_ID) = (I2C_WRITE << 8) | *msg_buf0++;
		} else {
			REG_I2C_DC(I2C_ID) = (I2C_WRITE << 8) | *msg_buf1++;
		}
	}

	cmd_cnt[I2C_ID]--;
	if (!(cmd_cnt[I2C_ID])) {
		REG_I2C_INTM(I2C_ID) = 0x0;
		cmd_flag[I2C_ID] = 2;
		if (flags & I2C_M_RD){
			while (r_cnt[I2C_ID] > 2) {
				if ((REG_I2C_STA(I2C_ID) & I2C_STA_RFNE) && timeout) {
					if (I2C_ID == 0) {
						*msg_buf0++ = REG_I2C_DC(I2C_ID) & 0xff;
					} else {
						*msg_buf1++ = REG_I2C_DC(I2C_ID) & 0xff;
					}
					r_cnt[I2C_ID]--;
				}
				if (!(timeout--)) {
					cmd_flag[I2C_ID] = -1;
					return IRQ_HANDLED;
				}
			}
		}
	}
	
	return IRQ_HANDLED;
}

static int i2c_disable(int I2C_ID)
{
	int timeout = TIMEOUT;
	
	__i2c_disable(I2C_ID);
	while(__i2c_is_enable(I2C_ID) && (timeout > 0)) {
		udelay(1);
		timeout--;
	}
	if(timeout)
		return 0;
	else
		return 1;
}
#if 0
static int i2c_enable(int I2C_ID)
{
	int timeout = TIMEOUT;
	
	__i2c_enable(I2C_ID);
	while(__i2c_is_disable(I2C_ID) && (timeout > 0)) {
		mdelay(1);
		timeout--;
	}
	if(timeout)
		return 0;
	else
		return 1;
}
#endif

static int i2c_set_clk(int i2c_clk, int I2C_ID)
{
	int dev_clk_khz = cpm_get_clock(CGU_PCLK) / 1000;
	int cnt_high = 0;	/* HIGH period count of the SCL clock */
	int cnt_low = 0;	/* LOW period count of the SCL clock */
	int cnt_period = 0;	/* period count of the SCL clock */
	
	if (i2c_clk <= 0 || i2c_clk > 400)
		goto Set_clk_err;

	cnt_period = dev_clk_khz / i2c_clk;
	if (i2c_clk <= 100) {
		/* i2c standard mode, the min LOW and HIGH period are 4700 ns and 4000 ns */
		cnt_high = (cnt_period * 4000) / (4700 + 4000);
	} else {
		/* i2c fast mode, the min LOW and HIGH period are 1300 ns and 600 ns */
		cnt_high = (cnt_period * 600) / (1300 + 600);
	}

	cnt_low = cnt_period - cnt_high;

	if (i2c_clk <= 100) {
		REG_I2C_CTRL(I2C_ID) = 0x43 | i2c_ctrl_rest[I2C_ID];      /* standard speed mode*/
		REG_I2C_SHCNT(I2C_ID) = I2CSHCNT_ADJUST(cnt_high);
		REG_I2C_SLCNT(I2C_ID) = I2CSLCNT_ADJUST(cnt_low);
	} else {
		REG_I2C_CTRL(I2C_ID) = 0x45 | i2c_ctrl_rest[I2C_ID];       /* high speed mode*/
		REG_I2C_FHCNT(I2C_ID) = I2CFLCNT_ADJUST(cnt_high);
		REG_I2C_FLCNT(I2C_ID) = I2CFLCNT_ADJUST(cnt_low);
	}

	return 0;

Set_clk_err:
	
	printk("i2c set sclk faild,i2c_clk=%d KHz,dev_clk=%dKHz.\n", i2c_clk, dev_clk_khz);
	return -1;
}

static void i2c_set_target(unsigned char address,int I2C_ID)
{	
	while (!__i2c_txfifo_is_empty(I2C_ID) || __i2c_master_active(I2C_ID));
	REG_I2C_TAR(I2C_ID) = address;  /* slave id needed write only once */
}

static void i2c_init_as_master(int I2C_ID,unsigned char device)
{
	int i;
	if(i2c_disable(I2C_ID))
		printk("i2c not disable\n");

	for (i = 0; i < I2C_CLIENT_NUM; i++) {
		if(device == jz4760_i2c_speed[i].slave_addr) {
			i2c_set_clk(jz4760_i2c_speed[i].speed,I2C_ID);
			break;
		}
	}
	if (i == I2C_CLIENT_NUM) {
		i2c_set_clk(100,I2C_ID);
	}
	REG_I2C_INTM(I2C_ID) = 0x0; /*mask all interrupt*/
	REG_I2C_TXTL(I2C_ID) = 0x1;
	REG_I2C_ENB(I2C_ID) = 1;   /*enable i2c*/
}

static DEFINE_SPINLOCK(i2c_read_lock);
static int xfer_read_nostart(struct i2c_msg *msg1, struct i2c_msg *msg2, struct jz_i2c *i2c)
{
	unsigned char *subaddr = msg1->buf;
	unsigned char device = msg2->addr;
	//unsigned char *buf   = msg2->buf;
	//int length           = msg2->len;
	int timeout,timeout_1,ret = 0;
	int I2C_ID = i2c->id;
	unsigned long irqflag;
	
	/*add for 0v2655 virtual I2C ID on 2010.6.5 and for hi704 on 2010.8.3*/
	if (device == 0xaa || device == 0xbb)
		device =0x30;
	if(device == 0xcc)
		device =0x21;

#if defined(CONFIG_TOUCHSCREEN_JZ_MT4D)
	if ((device == 0x40) && __gpio_get_pin(GPIO_ATTN)) {
		return -87;
	}
#endif
	i2c_set_target(device,I2C_ID);
	cmd_flag[I2C_ID] = 0;
#if 1
	REG_I2C_INTM(I2C_ID) = 0x40;
	if (msg1->len == 1) {
		REG_I2C_DC(I2C_ID) = (I2C_WRITE << 8) | *subaddr;
	} else if (msg1->len == 2) {
		spin_lock_irqsave(&i2c_read_lock, irqflag);
		REG_I2C_DC(I2C_ID) = (I2C_WRITE << 8) | *subaddr++;
		REG_I2C_DC(I2C_ID) = (I2C_WRITE << 8) | *subaddr;
		spin_unlock_irqrestore(&i2c_read_lock, irqflag);
	} else {
		printk("your device subaddres is not support.\n");
		return -7;
	}
#endif

	REG_I2C_INTM(I2C_ID) = 0x50;
	timeout = TIMEOUT;
	while (cmd_flag[I2C_ID] != 2 && --timeout) {
		if (cmd_flag[I2C_ID] == -1) {
			ret = 1; /*device nack.*/
			goto R_dev_err;
		}
		udelay(1);
	}
	if (!timeout) {
		ret = 2; /*wait interrupt timeout.*/
	      	goto R_timeout;
	}

	timeout_1 = TIMEOUT;
	while (r_cnt[I2C_ID] && --timeout_1) {
		timeout = TIMEOUT;
		while ((!(REG_I2C_STA(I2C_ID) & I2C_STA_RFNE)) && (--timeout)) {
			if ((cmd_flag[I2C_ID] == -1) || 
			    (REG_I2C_INTST(I2C_ID) & I2C_INTST_TXABT) ||
			    REG_I2C_TXABRT(I2C_ID)) {
				ret = 3; /*i2c abort or nack.*/
				goto R_dev_err;
			}
		}
		if (!timeout){
			ret = 4;
			goto R_timeout;
		}

		if (I2C_ID == 0) {
			*msg_buf0++ = REG_I2C_DC(I2C_ID) & 0xff;
		} else {
			*msg_buf1++ = REG_I2C_DC(I2C_ID) & 0xff;
		}
		r_cnt[I2C_ID]--;
	}
	if (!timeout_1) {
		ret = 6; /*can not run here.*/
		goto R_timeout;
	}

	timeout = TIMEOUT;
	while ((REG_I2C_STA(I2C_ID) & I2C_STA_MSTACT) && --timeout)
		udelay(10);
	if (!timeout){
      		ret = 5; /*wait master inactive timeout.*/
	      	goto R_timeout;
      	}

	return 0;

R_dev_err:
R_timeout:

	//__reg_printk();
	__i2c_clear_interrupts(ret,I2C_ID);
	i2c_init_as_master(I2C_ID,device);
	//dprintk("Read i2c device:0x%2x failed, ret = %d.",device, ret);
	return -ret;
}

static int xfer_read(unsigned char device, unsigned char *buf, 
		     int length, struct jz_i2c *i2c)
{
	int timeout,timeout_1,ret = 0;
	int I2C_ID = i2c->id;
	
	/*add for 0v2655 virtual I2C ID on 2010.6.5 and for hi704 on 2010.8.3*/
	if (device == 0xaa || device == 0xbb)
		device =0x30;
	if(device == 0xcc)
	  device =0x21;

#if defined(CONFIG_TOUCHSCREEN_JZ_MT4D)
	if ((device == 0x40) && __gpio_get_pin(GPIO_ATTN)) {
		return -87;
	}
#endif
	i2c_set_target(device,I2C_ID);
	cmd_flag[I2C_ID] = 0;
	REG_I2C_INTM(I2C_ID) = 0x50;
	timeout = TIMEOUT;
	while (cmd_flag[I2C_ID] != 2 && --timeout) {
		if (cmd_flag[I2C_ID] == -1) {
			ret = 1; /*device nack.*/
			goto R_dev_err;
		}
		udelay(1);
	}
	if (!timeout) {
		ret = 2; /*wait interrupt timeout.*/
	      	goto R_timeout;
	}

	timeout_1 = TIMEOUT;
	while (r_cnt[I2C_ID] && --timeout_1) {
		timeout = TIMEOUT;
		while ((!(REG_I2C_STA(I2C_ID) & I2C_STA_RFNE)) && (--timeout)) {
			if ((cmd_flag[I2C_ID] == -1) || 
			    (REG_I2C_INTST(I2C_ID) & I2C_INTST_TXABT) ||
			    REG_I2C_TXABRT(I2C_ID)) {
				ret = 3; /*i2c abort or nack.*/
				goto R_dev_err;
			}
		}
		if (!timeout){
			ret = 4;
			goto R_timeout;
		}

		if (I2C_ID == 0) {
			*msg_buf0++ = REG_I2C_DC(I2C_ID) & 0xff;
		} else {
			*msg_buf1++ = REG_I2C_DC(I2C_ID) & 0xff;
		}
		r_cnt[I2C_ID]--;
	}
	if (!timeout_1) {
		ret = 6; /*can not run here.*/
		goto R_timeout;
	}

	timeout = TIMEOUT;
	while ((REG_I2C_STA(I2C_ID) & I2C_STA_MSTACT) && --timeout)
		udelay(10);
	if (!timeout){
      		ret = 5; /*wait master inactive timeout.*/
	      	goto R_timeout;
      	}

	return 0;

R_dev_err:
R_timeout:

	//__reg_printk();
	__i2c_clear_interrupts(ret,I2C_ID);
	i2c_init_as_master(I2C_ID,device);
	//dprintk("Read i2c device:0x%2x failed, ret = %d.",device, ret);
	return -ret;
}

static int xfer_write(unsigned char device, unsigned char *buf,
		      int length, struct jz_i2c *i2c)
{
	int timeout,ret = 0;
	int I2C_ID = i2c->id;

	/*add for 0v2655 virtual I2C ID on 2010.6.5 and for hi704 on 2010.8.3*/
	if (device == 0xaa||device == 0xbb)
		device =0x30;
	if(device == 0xcc)
	  device = 0x21;

	i2c_set_target(device,I2C_ID);
	cmd_flag[I2C_ID] = 0;
	REG_I2C_INTM(I2C_ID) = 0x50;

	timeout = TIMEOUT;
	while ((cmd_flag[I2C_ID] != 2) && (--timeout)) {
		if (cmd_flag[I2C_ID] == -1){
			ret = 51; /*device no ack*/
			goto W_dev_err;
		}
		udelay(1);
	}

       	if (!timeout){
       		ret = 55; /*wait interrupt timeout.*/
       		goto W_timeout;
       	}

	timeout = TIMEOUT;
	while((!(REG_I2C_STA(I2C_ID) & I2C_STA_TFE)) && --timeout){
		udelay(10);
	}
       	if (!timeout){
       		ret = 52; /*wait TF buf empty timeout.*/
       		goto W_timeout;
       	}

	timeout = TIMEOUT;
	while (__i2c_master_active(I2C_ID) && --timeout);
	if (!timeout){
		ret = 53; /*wait master incative timeout.*/
		goto W_timeout;
	}

	if (((cmd_flag[I2C_ID] == -1) || 
	    (REG_I2C_INTST(I2C_ID) & I2C_INTST_TXABT) ||
	     REG_I2C_TXABRT(I2C_ID))) {
		int ret;
		ret = 54; /*device nack or Transmite abort.*/
		goto W_dev_err;
	}
	return 0;
	
W_dev_err:
W_timeout:
	//__reg_printk();
	__i2c_clear_interrupts(ret,I2C_ID);
	i2c_init_as_master(I2C_ID,device);
	//dprintk("Write i2c device:0x%2x failed, ret = %d.",device, ret);

	return -ret;
}

static int i2c_jz_xfer(struct i2c_adapter *adap, struct i2c_msg *spmsg, int num)
{
	int ret = 0, i = 0;
	int retry = adap->retries;
	struct i2c_msg *pmsg = spmsg;
	struct jz_i2c *i2c = adap->algo_data;

	if (spmsg->addr != current_device[i2c->id]) {
		current_device[i2c->id] = spmsg->addr;
		if(spmsg->flags & I2C_M_NOSTART)
			i2c_ctrl_rest[i2c->id] = I2C_CTRL_REST;
		else
			i2c_ctrl_rest[i2c->id] = 0x0;
		i2c_init_as_master(i2c->id,spmsg->addr);
	}

    if ((num > 2) || (num < 1)) {
        printk("error massage num,num need is 1 or 2.\n");
        return -100;
    }

	if ((spmsg->flags & I2C_M_NOSTART) && (num == 2)) {

		while (retry--) {
			pmsg = spmsg;
			pmsg++;	
			if (pmsg->len && pmsg->buf) {	/* sanity check */
				i2c->msg = pmsg;
				if (i2c->id == 0) {
					msg_buf0 = pmsg->buf;
				} else {
					msg_buf1 = pmsg->buf;
				}
				cmd_cnt[i2c->id] = pmsg->len;
				r_cnt[i2c->id] = pmsg->len;

				ret = xfer_read_nostart(spmsg, pmsg, i2c);
				if (ret) {
					udelay(100);
					continue;
				} else {
					return num;
				}
			}
		}

		if (ret) {
			if ((ret == -1) || (ret == -2 ) || (ret == -3)){
				printk("Nostart read i2c device 0x%2x, nack or abort, ret=%d\n",\
						current_device[i2c->id],ret);
			} else if (ret == -4) {
				printk("Nostart read i2c device 0x%2x, wait device date timeout,ret=%d\n",
						current_device[i2c->id],ret);
			} else if (ret == -5){
				printk("Nostart read i2c device 0x%2x, wait mst inactive timeout,ret=%d\n",
						current_device[i2c->id],ret);
			} else if (ret == -6) {
				printk("Nostart read i2c device 0x%2x, can not get here,check your driver,ret=%d\n",
						current_device[i2c->id],ret);
			}
			return ret;
		}
	}

	while (retry--) {
		pmsg = spmsg;
		for (i = 0; i < num; i++) {
			if (pmsg->len && pmsg->buf) {	/* sanity check */
				i2c->msg = pmsg;
				if (i2c->id == 0) {
					msg_buf0 = pmsg->buf;
				} else {
					msg_buf1 = pmsg->buf;
				}
				cmd_cnt[i2c->id] = pmsg->len;
				r_cnt[i2c->id] = pmsg->len;

				if (pmsg->flags & I2C_M_RD){
					ret = xfer_read(pmsg->addr, pmsg->buf, pmsg->len,i2c);
				} else {
					ret = xfer_write(pmsg->addr, pmsg->buf, pmsg->len,i2c);
				}
				if (ret)
					break;
			}
			pmsg++;		/* next message */
		}

		if (!ret && (i == num))
			return i;
		else
			udelay(100);
	}

	if (ret) {
		if ((ret == -1) || (ret == -2 ) || (ret == -3)){
			printk("Read i2c device 0x%2x, nack or abort, ret=%d\n",\
					current_device[i2c->id],ret);
		} else if((ret == -51) || (ret == -54) || (ret == -55)){
			printk("Write i2c device 0x%2x, nack or abort, ret=%d\n",\
					current_device[i2c->id],ret);
		} else if (ret == -4) {
			printk("Read i2c device 0x%2x, wait device date timeout,ret=%d\n",
					current_device[i2c->id],ret);
		} else if (ret == -5){
			printk("Read i2c device 0x%2x, wait mst inactive timeout,ret=%d\n",
					current_device[i2c->id],ret);
		} else if (ret == -53) {
			printk("Write i2c device 0x%2x, wait mst inactive timeout,ret=%d\n",
					current_device[i2c->id],ret);
		} else if (ret == -6) {
			printk("Write i2c device 0x%2x, can not get here,check your driver,ret=%d\n",
					current_device[i2c->id],ret);
		} else{
			printk("Write i2c device 0x%2x, wait TF empty timeout,ret=%d\n",
					current_device[i2c->id],ret);
		}
		return ret;

	} else {
		return i;
	}
}

static u32 i2c_jz_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm i2c_jz_algorithm = {
	.master_xfer	= i2c_jz_xfer,
	.functionality	= i2c_jz_functionality,
};

static int i2c_jz_probe(struct platform_device *pdev)
{
	struct jz_i2c *i2c;
	struct i2c_jz_platform_data *plat = pdev->dev.platform_data;
	int ret;

	i2c = kzalloc(sizeof(struct jz_i2c), GFP_KERNEL);
	if (!i2c) {
		printk("There is no enough memory\n");
		ret = -ENOMEM;
		goto emalloc;
	}

	i2c->id            = pdev->id;
	i2c->adap.owner   = THIS_MODULE;
	i2c->adap.algo    = &i2c_jz_algorithm;
	i2c->adap.retries = 6;
	spin_lock_init(&i2c->lock);
	init_waitqueue_head(&i2c->wait);
	sprintf(i2c->adap.name, "jz_i2c-i2c.%u", pdev->id);
	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &pdev->dev;
	if (i2c->id == 0)
		cpm_start_clock(CGM_I2C0);
	else
		cpm_start_clock(CGM_I2C1);

	__gpio_as_i2c(i2c->id);

	i2c_init_as_master(i2c->id,0xff);

	if (plat) {
		i2c->adap.class = plat->class;
	}

	i2c->irq = platform_get_irq(pdev, 0);
	ret = request_irq(i2c->irq, jz_i2c_irq, IRQF_DISABLED,
			  dev_name(&pdev->dev), i2c);

	if (ret != 0) {
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", i2c->irq);
		goto emalloc;
	}
	
	/*
	 * If "dev->id" is negative we consider it as zero.
	 * The reason to do so is to avoid sysfs names that only make
	 * sense when there are multiple adapters.
	 */
	i2c->adap.nr = pdev->id != -1 ? pdev->id : 0;
	/* ret = i2c_add_adapter(&i2c->adap); */
	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		printk(KERN_INFO "I2C: Failed to add bus\n");
		goto eadapt;
	}

	platform_set_drvdata(pdev, i2c);
	dev_info(&pdev->dev, "JZ4760 i2c bus driver.\n");
	return 0;

eadapt:
emalloc:
	kfree(i2c);
	return ret;
}

static int i2c_jz_remove(struct platform_device *pdev)
{
	struct i2c_adapter *adapter = platform_get_drvdata(pdev);
	int rc;

	rc = i2c_del_adapter(adapter);
	platform_set_drvdata(pdev, NULL);
	return rc;
}

#ifdef  CONFIG_I2C0_JZ4760
static struct platform_driver i2c_0_jz_driver = {
	.probe		= i2c_jz_probe,
	.remove		= i2c_jz_remove,
	.driver		= {
		.name	= "jz_i2c0",
	},
};
#endif

#ifdef  CONFIG_I2C1_JZ4760
static struct platform_driver i2c_1_jz_driver = {
	.probe		= i2c_jz_probe,
	.remove		= i2c_jz_remove,
	.driver		= {
		.name	= "jz_i2c1",
	},
};
#endif

static int __init i2c_adap_jz_init(void)
{
	int ret = 0;

#ifdef  CONFIG_I2C0_JZ4760
	ret = platform_driver_register(&i2c_0_jz_driver);
#endif

#ifdef  CONFIG_I2C1_JZ4760
	ret = platform_driver_register(&i2c_1_jz_driver);
#endif
	return ret;
}

static void __exit i2c_adap_jz_exit(void)
{
#ifdef  CONFIG_I2C0_JZ4760
	platform_driver_unregister(&i2c_0_jz_driver);
#endif

#ifdef  CONFIG_I2C1_JZ4760
	platform_driver_unregister(&i2c_1_jz_driver);
#endif
}

MODULE_LICENSE("GPL");
subsys_initcall(i2c_adap_jz_init);
module_exit(i2c_adap_jz_exit);
