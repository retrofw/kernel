/*
 *  linux/drivers/net/jz_eth.c
 *
 *  Jz4730/Jz5730 On-Chip ethernet driver.
 *
 *  Copyright (C) 2005 - 2007  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/skbuff.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/pm.h>

#include <asm/io.h>
#include <asm/addrspace.h>
#include <asm/uaccess.h>
#include <asm/cacheflush.h>
#include <asm/cacheops.h>
#include <asm/jzsoc.h>

#include "jz_eth.h"

#define P2ADDR(a)	(((unsigned long)(a) & 0x1fffffff) | 0xa0000000)
#define P1ADDR(a)	(((unsigned long)(a) & 0x1fffffff) | 0x80000000)

//#define DEBUG
#ifdef DEBUG
#     define DBPRINTK(fmt,args...) printk(KERN_DEBUG fmt,##args)
#else
#     define DBPRINTK(fmt,args...) do {} while(0)
#endif

#define errprintk(fmt,args...)  printk(KERN_ERR fmt,##args);
#define infoprintk(fmt,args...) printk(KERN_INFO fmt,##args);

#define DRV_NAME	"jz_eth"
#define DRV_VERSION	"1.2"
#define DRV_AUTHOR	"Peter Wei <jlwei@ingenic.cn>"
#define DRV_DESC	"JzSOC On-chip Ethernet driver"

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");

/*
 * Local variables
 */
static struct net_device *netdev;
static char * hwaddr = NULL;
static int debug = -1;
static struct mii_if_info mii_info;

MODULE_PARM_DESC(debug, "i");
MODULE_PARM_DESC(hwaddr,"s");

/*
 * Local routines
 */
static irqreturn_t jz_eth_interrupt(int irq, void *dev_id);

static int link_check_thread (void *data); 

/*
 * Get MAC address
 */

#define I2C_DEVICE  0x57
#define MAC_OFFSET  64

extern void i2c_open(void);
extern void i2c_close(void);
extern int i2c_read(unsigned char device, unsigned char *buf,
		    unsigned char address, int count);

static inline unsigned char str2hexnum(unsigned char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return 0; /* foo */
}

static inline void str2eaddr(unsigned char *ea, unsigned char *str)
{
	int i;

	for (i = 0; i < 6; i++) {
		unsigned char num;

		if((*str == '.') || (*str == ':'))
			str++;
		num = str2hexnum(*str++) << 4;
		num |= (str2hexnum(*str++));
		ea[i] = num;
	}
}

static int ethaddr_cmd = 0;
static unsigned char ethaddr_hex[6];

static int __init ethernet_addr_setup(char *str)
{
	if (!str) {
	        printk("ethaddr not set in command line\n");
		return -1;
	}
	ethaddr_cmd = 1;
	str2eaddr(ethaddr_hex, str);

	return 0;
}

__setup("ethaddr=", ethernet_addr_setup);

static int get_mac_address(struct net_device *dev)
{
	int i;
	unsigned char flag0=0;
	unsigned char flag1=0xff;
	
	dev->dev_addr[0] = 0xff;
	if (hwaddr != NULL) {
		/* insmod jz-ethc.o hwaddr=00:ef:a3:c1:00:10 */
		str2eaddr(dev->dev_addr, hwaddr);
	} else if (ethaddr_cmd) {
		/* linux command line: ethaddr=00:ef:a3:c1:00:10 */
		for (i=0; i<6; i++)
			dev->dev_addr[i] = ethaddr_hex[i];
	} else {
#if 0
		/* mac address in eeprom:  byte 0x40-0x45 */
		i2c_open();
		i2c_read(I2C_DEVICE, dev->dev_addr, MAC_OFFSET, 6);
		i2c_close();
#endif
	}

	/* check whether valid MAC address */
	for (i=0; i<6; i++) {
		flag0 |= dev->dev_addr[i];
		flag1 &= dev->dev_addr[i];
	}
	if ((dev->dev_addr[0] & 0xC0) || (flag0 == 0) || (flag1 == 0xff)) {
		printk("WARNING: There is not MAC address, use default ..\n");
		dev->dev_addr[0] = 0x00;
		dev->dev_addr[1] = 0xef;
		dev->dev_addr[2] = 0xa3;
		dev->dev_addr[3] = 0xc1;
		dev->dev_addr[4] = 0x00;
		dev->dev_addr[5] = 0x10;
		dev->dev_addr[5] = 0x03;
	}
	return 0;
}

/*---------------------------------------------------------------------*/

static u32 jz_eth_curr_mode(struct net_device *dev);

/*
 * Ethernet START/STOP routines
 */
#define START_ETH {			\
    s32 val;				\
    val = readl(DMA_OMR);		\
    val |= OMR_ST | OMR_SR;		\
    writel(val, DMA_OMR); 		\
}

#define STOP_ETH {			\
    s32 val;				\
    val = readl(DMA_OMR);		\
    val &= ~(OMR_ST|OMR_SR);		\
    writel(val, DMA_OMR);  		\
}

/*
 * Link check routines
 */
static void start_check(struct net_device *dev)
{
	struct jz_eth_private *np = (struct jz_eth_private *)netdev_priv(dev);

	np->thread_die = 0;
	init_waitqueue_head(&np->thr_wait);
	init_completion (&np->thr_exited);
	np->thr_pid = kernel_thread (link_check_thread,(void *)dev, 
				     CLONE_FS | CLONE_FILES);
	if (np->thr_pid < 0)
		errprintk("%s: unable to start kernel thread\n",dev->name);
}

static int close_check(struct net_device *dev)
{
	struct jz_eth_private *np = (struct jz_eth_private *)netdev_priv(dev);

	int ret = 0;

	if (np->thr_pid >= 0) {
		np->thread_die = 1;
		wmb();
		ret = kill_proc (np->thr_pid, SIGTERM, 1);
		if (ret) {
			errprintk("%s: unable to signal thread\n", dev->name);
			return 1;
		}
		wait_for_completion (&np->thr_exited);
	}
	return 0;
}

static int link_check_thread(void *data)
{
	struct net_device *dev=(struct net_device *)data;
	struct jz_eth_private *np = (struct jz_eth_private *)netdev_priv(netdev);

	unsigned char current_link;
	unsigned long timeout;

	daemonize("%s", dev->name);
	spin_lock_irq(&current->sighand->siglock);
	sigemptyset(&current->blocked);
	recalc_sigpending();
	spin_unlock_irq(&current->sighand->siglock);

	strncpy (current->comm, dev->name, sizeof(current->comm) - 1);
	current->comm[sizeof(current->comm) - 1] = '\0';

	while (1) {
		timeout = 3*HZ;
		do {
			timeout = interruptible_sleep_on_timeout (&np->thr_wait, timeout);
			/* make swsusp happy with our thread */
//			if (current->flags & PF_FREEZE)
//				refrigerator(PF_FREEZE);
		} while (!signal_pending (current) && (timeout > 0));

		if (signal_pending (current)) {
			spin_lock_irq(&current->sighand->siglock);
			flush_signals(current);
			spin_unlock_irq(&current->sighand->siglock);
		}

		if (np->thread_die)
			break;
		
		current_link=mii_link_ok(&mii_info);
		if (np->link_state!=current_link) {
			if (current_link) {
				infoprintk("%s: Ethernet Link OK!\n",dev->name);
				jz_eth_curr_mode(dev);
				netif_carrier_on(dev);
			}
			else {
				errprintk("%s: Ethernet Link offline!\n",dev->name);
				netif_carrier_off(dev);
			}
		}
		np->link_state=current_link;

	}
	complete_and_exit (&np->thr_exited, 0);	
}

#ifdef DEBUG
/*
 * Display ethernet packet header
 * This routine is used for test function
 */
static void eth_dbg_rx(struct sk_buff *skb, int len) 
{

  	int i, j; 
    
  	printk("R: %02x:%02x:%02x:%02x:%02x:%02x <- %02x:%02x:%02x:%02x:%02x:%02x len/SAP:%02x%02x [%d]\n",
  	       (u8)skb->data[0], 
  	       (u8)skb->data[1], 
  	       (u8)skb->data[2],
  	       (u8)skb->data[3], 
  	       (u8)skb->data[4], 
  	       (u8)skb->data[5], 
  	       (u8)skb->data[6], 
  	       (u8)skb->data[7], 
  	       (u8)skb->data[8], 
  	       (u8)skb->data[9],
  	       (u8)skb->data[10], 
  	       (u8)skb->data[11], 
  	       (u8)skb->data[12], 
  	       (u8)skb->data[13], 
  	       len); 
  	for (j=0; len>0; j+=16, len-=16) { 
  		printk("    %03x: ",j); 
  		for (i=0; i<16 && i<len; i++) { 
  			printk("%02x ",(u8)skb->data[i+j]); 
  		} 
  		printk("\n"); 
  	} 
  	return; 
  } 
#endif 

/*
 * Reset ethernet device
 */
static inline void jz_eth_reset(void)
{				
	u32 i;					
	i = readl(DMA_BMR);
	writel(i | BMR_SWR, DMA_BMR);			
	for(i = 0; i < 1000; i++) {			
		if(!(readl(DMA_BMR) & BMR_SWR)) break;	
		mdelay(1);			
	}						
}

/*
 * MII operation routines 
 */
static inline void mii_wait(void)
{
	int i;
	for(i = 0; i < 10000; i++) {
		if(!(readl(MAC_MIIA) & 0x1)) 
			break;
		mdelay(1);
	}
	if (i >= 10000)
		printk("MII wait timeout : %d.\n", i);
}

static int mdio_read(struct net_device *dev,int phy_id, int location)
{
	u32 mii_cmd = (phy_id << 11) | (location << 6) | 1;
	int retval = 0;
	
	writel(mii_cmd, MAC_MIIA);
	mii_wait();
	retval = readl(MAC_MIID) & 0x0000ffff;
	
	return retval;
	
}

static void mdio_write(struct net_device *dev,int phy_id, int location, int data)
{
	u32 mii_cmd = (phy_id << 11) | (location << 6) | 0x2 | 1;
	
	writel(mii_cmd, MAC_MIIA);
	writel(data & 0x0000ffff, MAC_MIID);
	mii_wait();
}


/*
 * Search MII phy
 */
static int jz_search_mii_phy(struct net_device *dev)
{
	
	struct jz_eth_private *np = (struct jz_eth_private *)netdev_priv(dev);

	int phy, phy_idx = 0;

	np->valid_phy = 0xff;
	for (phy = 0; phy < 32; phy++) {
		int mii_status = mdio_read(dev,phy, 1);
		if (mii_status != 0xffff  &&  mii_status != 0x0000) {
			np->phys[phy_idx] = phy;
			np->ecmds[phy_idx].speed=SPEED_100;
			np->ecmds[phy_idx].duplex=DUPLEX_FULL;
			np->ecmds[phy_idx].port=PORT_MII;
			np->ecmds[phy_idx].transceiver=XCVR_INTERNAL;
			np->ecmds[phy_idx].phy_address=np->phys[phy_idx];
			np->ecmds[phy_idx].autoneg=AUTONEG_ENABLE;
			np->ecmds[phy_idx].advertising=(ADVERTISED_10baseT_Half |
							ADVERTISED_10baseT_Full |
							ADVERTISED_100baseT_Half |
							ADVERTISED_100baseT_Full);
			phy_idx++;
		}
	}
	if (phy_idx == 1) {
		np->valid_phy = np->phys[0];
		np->phy_type = 0;
	}
	if (phy_idx != 0) {
		phy = np->valid_phy;
		np->advertising = mdio_read(dev,phy, 4);
	}
	return phy_idx;	
}

/*
 * CRC calc for Destination Address for gets hashtable index
 */

#define POLYNOMIAL 0x04c11db7UL
static u16 jz_hashtable_index(u8 *addr)
{
#if 1
	u32 crc = 0xffffffff, msb;
	int  i, j;
	u32  byte;
	for (i = 0; i < 6; i++) {
		byte = *addr++;
		for (j = 0; j < 8; j++) {
			msb = crc >> 31;
			crc <<= 1;
			if (msb ^ (byte & 1)) crc ^= POLYNOMIAL;
			byte >>= 1;
		}
	}
	return ((int)(crc >> 26));
#endif
#if 0
	int crc = -1;
	int length=6;
	int bit;
	unsigned char current_octet;
	while (--length >= 0) {
		current_octet = *addr++;
		for (bit = 0; bit < 8; bit++, current_octet >>= 1)
			crc = (crc << 1) ^ ((crc < 0) ^ (current_octet & 1) ?
			     POLYNOMIAL : 0);
	}
	return ((int)(crc >> 26));
#endif
}

/*
 * Multicast filter and config multicast hash table
 */
#define MULTICAST_FILTER_LIMIT 64

static void jz_set_multicast_list(struct net_device *dev)
{
	int i, hash_index;
	u32 mcr, hash_h, hash_l, hash_bit;
	
	mcr = readl(MAC_MCR);
	mcr &= ~(MCR_PR | MCR_PM | MCR_HP);
	
	if (dev->flags & IFF_PROMISC) {
		/* Accept any kinds of packets */
		mcr |= MCR_PR;
		hash_h = 0xffffffff;
		hash_l = 0xffffffff;
		DBPRINTK("%s: enter promisc mode!\n",dev->name);
	}
	else  if ((dev->flags & IFF_ALLMULTI) || (dev->mc_count > MULTICAST_FILTER_LIMIT)){
		/* Accept all multicast packets */
		mcr |= MCR_PM;
		hash_h = 0xffffffff;
		hash_l = 0xffffffff;
		DBPRINTK("%s: enter allmulticast mode!   %d \n",dev->name,dev->mc_count);
	}
	else if (dev->flags & IFF_MULTICAST)
	{
		/* Update multicast hash table */
		struct dev_mc_list *mclist;
		hash_h = readl(MAC_HTH);
		hash_l = readl(MAC_HTL);
		for (i = 0, mclist = dev->mc_list; mclist && i < dev->mc_count;
		     i++, mclist = mclist->next)
		{
			hash_index = jz_hashtable_index(mclist->dmi_addr);
			hash_bit=0x00000001;
			hash_bit <<= (hash_index & 0x1f);
			if (hash_index > 0x1f) 
				hash_h |= hash_bit;
			else
				hash_l |= hash_bit;
			DBPRINTK("----------------------------\n");
#ifdef DEBUG
			int j;
			for (j=0;j<mclist->dmi_addrlen;j++)
				printk("%2.2x:",mclist->dmi_addr[j]);
			printk("\n");
#endif
			DBPRINTK("dmi.addrlen => %d\n",mclist->dmi_addrlen);
			DBPRINTK("dmi.users   => %d\n",mclist->dmi_users);
			DBPRINTK("dmi.gusers  => %d\n",mclist->dmi_users);
		}
		writel(hash_h,MAC_HTH);
		writel(hash_l,MAC_HTL);
		mcr |= MCR_HP;
		DBPRINTK("This is multicast hash table high bits [%4.4x]\n",readl(MAC_HTH));
		DBPRINTK("This is multicast hash table low  bits [%4.4x]\n",readl(MAC_HTL));
		DBPRINTK("%s: enter multicast mode!\n",dev->name);
	}
	writel(mcr,MAC_MCR);
}

static inline int jz_phy_reset(struct net_device *dev)
{
	struct jz_eth_private *np = (struct jz_eth_private *)netdev_priv(dev);

	unsigned int mii_reg0;
	unsigned int count;
	
	mii_reg0 = mdio_read(dev,np->valid_phy,MII_BMCR);
	mii_reg0 |=MII_CR_RST;   
	mdio_write(dev,np->valid_phy,MII_BMCR,mii_reg0);  //reset phy
	for ( count = 0; count < 1000; count++) {
		mdelay(1);
		mii_reg0 = mdio_read(dev,np->valid_phy,MII_BMCR);
		if (!(mii_reg0 & MII_CR_RST)) break;  //reset completed
	}
	if (count>=100) 
		return 1;     //phy error
	else
		return 0;
}

/*
 * Show all mii registers  -  this routine is used for test
 */
#ifdef DEBUG
static void mii_db_out(struct net_device *dev) 
{
	struct jz_eth_private *np = (struct jz_eth_private *)netdev_priv(dev);

	unsigned int mii_test;

	mii_test = mdio_read(dev,np->valid_phy,MII_BMCR);
	DBPRINTK("BMCR ====> 0x%4.4x \n",mii_test);
	
	mii_test = mdio_read(dev,np->valid_phy,MII_BMSR);
	DBPRINTK("BMSR ====> 0x%4.4x \n",mii_test);
	
	mii_test = mdio_read(dev,np->valid_phy,MII_ANAR);
	DBPRINTK("ANAR ====> 0x%4.4x \n",mii_test);
	
	mii_test = mdio_read(dev,np->valid_phy,MII_ANLPAR);
	DBPRINTK("ANLPAR ====> 0x%4.4x \n",mii_test);
	
	mii_test = mdio_read(dev,np->valid_phy,16);
	DBPRINTK("REG16 ====> 0x%4.4x \n",mii_test);
	
	mii_test = mdio_read(dev,np->valid_phy,17);
	DBPRINTK("REG17 ====> 0x%4.4x \n",mii_test);
}
#endif

/*
 * Start Auto-Negotiation function for PHY 
 */
static int jz_autonet_complete(struct net_device *dev)
{
	struct jz_eth_private *np = (struct jz_eth_private *)netdev_priv(dev);
	int count;
	u32 mii_reg1, timeout = 3000;

	for (count = 0; count < timeout; count++) {
		mdelay(1);
		mii_reg1 = mdio_read(dev,np->valid_phy,MII_BMSR);
		if (mii_reg1 & 0x0020) break;
	}
	//mii_db_out(dev);  //for debug to display all register of MII
	if (count >= timeout) 
		return 1;     //auto negotiation  error
	else
		return 0;
}  

/*
 * Get current mode of eth phy
 */
static u32 jz_eth_curr_mode(struct net_device *dev)
{
	struct jz_eth_private *np = (struct jz_eth_private *)netdev_priv(dev);
	unsigned int mii_reg17;
	u32 flag = 0;

	mii_reg17 = mdio_read(dev,np->valid_phy,MII_DSCSR); 
	np->media = mii_reg17>>12;
	if (np->media==8) {
		infoprintk("%s: Current Operation Mode is [100M Full Duplex]",dev->name);
		flag = 0;
		np->full_duplex=1;
	}
	if (np->media==4) {
		infoprintk("%s: Current Operation Mode is [100M Half Duplex]",dev->name);
		flag = 0;
		np->full_duplex=0;
	}
	if (np->media==2) {
		infoprintk("%s: Current Operation Mode is [10M Full Duplex]",dev->name);
		flag = OMR_TTM;
		np->full_duplex=1;
	}
	if (np->media==1) {
		infoprintk("%s: Current Operation Mode is [10M Half Duplex]",dev->name);
		flag = OMR_TTM;
		np->full_duplex=0;
	}
	printk("\n");
	return flag;
}

/*
 * Ethernet device hardware init
 * This routine initializes the ethernet device hardware and PHY
 */
static int jz_init_hw(struct net_device *dev)
{
	struct jz_eth_private *np = (struct jz_eth_private *)netdev_priv(dev);
	struct ethtool_cmd ecmd;
	u32 mcr, omr;
	u32 sts, flag = 0;
	int i;

	jz_eth_reset();
	STOP_ETH;
#if 0
	/* mii operation */
	if (jz_phy_reset(dev)) {
		errprintk("PHY device do not reset!\n");
		return -EPERM;          // return operation not permitted 
	}
#endif
	/* Set MAC address */
	writel(le32_to_cpu(*(unsigned long *)&dev->dev_addr[0]), MAC_MAL);
	writel(le32_to_cpu(*(unsigned long *)&dev->dev_addr[4]), MAC_MAH);
	printk("%s: JZ On-Chip ethernet (MAC ", dev->name);
	for (i = 0; i < 5; i++) {
		printk("%2.2x:", dev->dev_addr[i]);
	}
	printk("%2.2x, IRQ %d)\n", dev->dev_addr[i], dev->irq);

	np->mii_phy_cnt = jz_search_mii_phy(dev);
	printk("%s: Found %d PHY on JZ MAC\n", dev->name, np->mii_phy_cnt);

	mii_info.phy_id = np->valid_phy;
	mii_info.dev = dev;
	mii_info.mdio_read = &mdio_read;
	mii_info.mdio_write = &mdio_write;

	ecmd.speed = SPEED_100;
	ecmd.duplex = DUPLEX_FULL;
	ecmd.port = PORT_MII;
	ecmd.transceiver = XCVR_INTERNAL;
	ecmd.phy_address = np->valid_phy;
	ecmd.autoneg = AUTONEG_ENABLE;
        
	mii_ethtool_sset(&mii_info,&ecmd);
	if (jz_autonet_complete(dev)) 
		errprintk("%s: Ethernet Module AutoNegotiation failed\n",dev->name);
	mii_ethtool_gset(&mii_info,&ecmd);
	
	infoprintk("%s: Provide Modes: ",dev->name);
	for (i = 0; i < 5;i++) 
		if (ecmd.advertising & (1<<i))
			printk("(%d)%s", i+1, media_types[i]);
	printk("\n");  

	flag = jz_eth_curr_mode(dev);

	/* Config OMR register */
	omr = readl(DMA_OMR) & ~OMR_TTM;
	omr |= flag;
	//omr |= OMR_OSF;
	omr |= OMR_SF;
	writel(omr, DMA_OMR);

	readl(DMA_MFC); //through read operation to clear the register for 0x0000000
	/* Set the programmable burst length (value 1 or 4 is validate)*/
#if 0 /* __BIG_ENDIAN__ */
	writel(PBL_4 | DSL_0 | 0x100080, DMA_BMR);  /* DSL_0: see DESC_SKIP_LEN and DESC_ALIGN */
#else /* __LITTLE_ENDIAN__ */
	writel(PBL_4 | DSL_0, DMA_BMR);  /* DSL_0: see DESC_SKIP_LEN and DESC_ALIGN */
#endif
	/* Config MCR register*/
	mcr = (readl(MAC_MCR) & ~(MCR_PS | MCR_HBD | MCR_FDX));   
	if(np->full_duplex)
		mcr |= MCR_FDX;
	mcr |= MCR_BFD | MCR_TE | MCR_RE | MCR_OWD|MCR_HBD;
	writel(mcr, MAC_MCR);
//	mcr &= (readl(MAC_MCR) & ~(MCR_PM | MCR_PR | MCR_IF | MCR_HO | MCR_HP));
//	mcr &= 0xffdf;
//	mcr |= 0x0020;
//	writel(mcr, MAC_MCR);

	/* Set base address of TX and RX descriptors */
	writel(np->dma_rx_ring, DMA_RRBA);
	writel(np->dma_tx_ring, DMA_TRBA);

	START_ETH;

	/* set interrupt mask */
	writel(IMR_DEFAULT | IMR_ENABLE, DMA_IMR);

	/* Reset any pending (stale) interrupts */
	sts = readl(DMA_STS);
	writel(sts, DMA_STS);

	return 0;
}

static int jz_eth_open(struct net_device *dev)
{
	struct jz_eth_private *np = (struct jz_eth_private *)netdev_priv(dev);
	int retval, i;

	retval = request_irq(dev->irq, jz_eth_interrupt, 0, dev->name, dev);
	if (retval) {
		errprintk("%s: unable to get IRQ %d .\n", dev->name, dev->irq);
		return -EAGAIN;
	}

	for (i = 0; i < NUM_RX_DESCS; i++) {
		np->rx_ring[i].status = cpu_to_le32(R_OWN);
		np->rx_ring[i].desc1 = cpu_to_le32(RX_BUF_SIZE | RD_RCH);
		np->rx_ring[i].buf1_addr = cpu_to_le32(np->dma_rx_buf + i*RX_BUF_SIZE);
		np->rx_ring[i].next_addr = cpu_to_le32(np->dma_rx_ring + (i+1) * sizeof (jz_desc_t));
	}
	np->rx_ring[NUM_RX_DESCS - 1].next_addr = cpu_to_le32(np->dma_rx_ring);

	for (i = 0; i < NUM_TX_DESCS; i++) {
		np->tx_ring[i].status = cpu_to_le32(0);
		np->tx_ring[i].desc1  = cpu_to_le32(TD_TCH);
		np->tx_ring[i].buf1_addr = 0;
		np->tx_ring[i].next_addr = cpu_to_le32(np->dma_tx_ring + (i+1) * sizeof (jz_desc_t));
	}
	np->tx_ring[NUM_TX_DESCS - 1].next_addr = cpu_to_le32(np->dma_tx_ring);

	np->rx_head = 0;
	np->tx_head = np->tx_tail = 0;

	jz_init_hw(dev);

	dev->trans_start = jiffies;
	netif_start_queue(dev);
	start_check(dev);

	return 0;
}

static int jz_eth_close(struct net_device *dev)
{
	netif_stop_queue(dev);
	close_check(dev);
	STOP_ETH;
	free_irq(dev->irq, dev);
	return 0;
}

/*
 * Get the current statistics.
 * This may be called with the device open or closed.
 */
static struct net_device_stats * jz_eth_get_stats(struct net_device *dev)
{
	struct jz_eth_private *np = (struct jz_eth_private *)netdev_priv(dev);
	int tmp;
	
	tmp = readl(DMA_MFC); // After read clear to zero
	np->stats.rx_missed_errors += (tmp & MFC_CNT2) + ((tmp & MFC_CNT1) >> 16);
	
	return &np->stats;
}

/*
 * ethtool routines
 */
static int jz_ethtool_ioctl(struct net_device *dev, void *useraddr)
{
	struct jz_eth_private *np = (struct jz_eth_private *)netdev_priv(dev);
	u32 ethcmd;

	/* dev_ioctl() in ../../net/core/dev.c has already checked
	   capable(CAP_NET_ADMIN), so don't bother with that here.  */

	if (get_user(ethcmd, (u32 *)useraddr))
		return -EFAULT;

	switch (ethcmd) {

	case ETHTOOL_GDRVINFO: {
		struct ethtool_drvinfo info = { ETHTOOL_GDRVINFO };
		strcpy (info.driver, DRV_NAME);
		strcpy (info.version, DRV_VERSION);
		strcpy (info.bus_info, "OCS");
		if (copy_to_user (useraddr, &info, sizeof (info)))
			return -EFAULT;
		return 0;
	}

	/* get settings */
	case ETHTOOL_GSET: {
		struct ethtool_cmd ecmd = { ETHTOOL_GSET };
		spin_lock_irq(&np->lock);
		mii_ethtool_gset(&mii_info, &ecmd);
		spin_unlock_irq(&np->lock);
		if (copy_to_user(useraddr, &ecmd, sizeof(ecmd)))
			return -EFAULT;
		return 0;
	}
	/* set settings */
	case ETHTOOL_SSET: {
		int r;
		struct ethtool_cmd ecmd;
		if (copy_from_user(&ecmd, useraddr, sizeof(ecmd)))
			return -EFAULT;
		spin_lock_irq(&np->lock);
		r = mii_ethtool_sset(&mii_info, &ecmd);
		spin_unlock_irq(&np->lock);
		return r;
	}
	/* restart autonegotiation */
	case ETHTOOL_NWAY_RST: {
		return mii_nway_restart(&mii_info);
	}
	/* get link status */
	case ETHTOOL_GLINK: {
		struct ethtool_value edata = {ETHTOOL_GLINK};
		edata.data = mii_link_ok(&mii_info);
		if (copy_to_user(useraddr, &edata, sizeof(edata)))
			return -EFAULT;
		return 0;
	}

	/* get message-level */
	case ETHTOOL_GMSGLVL: {
		struct ethtool_value edata = {ETHTOOL_GMSGLVL};
		edata.data = debug;
		if (copy_to_user(useraddr, &edata, sizeof(edata)))
			return -EFAULT;
		return 0;
	}
	/* set message-level */
	case ETHTOOL_SMSGLVL: {
		struct ethtool_value edata;
		if (copy_from_user(&edata, useraddr, sizeof(edata)))
			return -EFAULT;
		debug = edata.data;
		return 0;
	}


	default:
		break;
	}

	return -EOPNOTSUPP;

}

/*
 * Config device
 */
static int jz_eth_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct jz_eth_private *np =(struct jz_eth_private *)netdev_priv(dev);
	struct mii_ioctl_data *data, rdata;

	switch (cmd) {
	case SIOCETHTOOL:
		return jz_ethtool_ioctl(dev, (void *) rq->ifr_data);
	case SIOCGMIIPHY:
	case SIOCDEVPRIVATE:
		data = (struct mii_ioctl_data *)&rq->ifr_data;
		data->phy_id = np->valid_phy;
	case SIOCGMIIREG:
	case SIOCDEVPRIVATE+1:
		data = (struct mii_ioctl_data *)&rq->ifr_data;
		data->val_out = mdio_read(dev,np->valid_phy, data->reg_num & 0x1f);
		return 0;
	case SIOCSMIIREG:
	case SIOCDEVPRIVATE+2:
		data = (struct mii_ioctl_data *)&rq->ifr_data;
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		mdio_write(dev,np->valid_phy, data->reg_num & 0x1f, data->val_in);
		return 0;
	case READ_COMMAND:	
		data = (struct mii_ioctl_data *)rq->ifr_data;
		if (copy_from_user(&rdata,data,sizeof(rdata)))
			return -EFAULT;
		rdata.val_out = mdio_read(dev,rdata.phy_id, rdata.reg_num & 0x1f);
		if (copy_to_user(data,&rdata,sizeof(rdata)))
			return -EFAULT;
		return 0;
	case WRITE_COMMAND:
		if (np->phy_type==1) {
			data = (struct mii_ioctl_data *)rq->ifr_data;
			if (!capable(CAP_NET_ADMIN))
				return -EPERM;
			if (copy_from_user(&rdata,data,sizeof(rdata)))
				return -EFAULT;
			mdio_write(dev,rdata.phy_id, rdata.reg_num & 0x1f, rdata.val_in);
		}
		return 0;
	case GETDRIVERINFO:
		if (np->phy_type==1) {
			data = (struct mii_ioctl_data *)rq->ifr_data;
			if (copy_from_user(&rdata,data,sizeof(rdata)))
				return -EFAULT;
			rdata.val_in = 0x1;
			rdata.val_out = 0x00d0;
			if (copy_to_user(data,&rdata,sizeof(rdata)))
				return -EFAULT;
		}
		return 0;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

/*
 * Received one packet
 */
static void eth_rxready(struct net_device *dev)
{
	struct jz_eth_private *np = (struct jz_eth_private*)netdev_priv(dev);
	struct sk_buff *skb;
	unsigned char *pkt_ptr;
	u32 pkt_len;
	u32 status;

	status = le32_to_cpu(np->rx_ring[np->rx_head].status);
	while (!(status & R_OWN)) {               /* owner bit = 0 */
		if (status & RD_ES) {              /* error summary */
			np->stats.rx_errors++;    /* Update the error stats. */
			if (status & (RD_RF | RD_TL))
				np->stats.rx_frame_errors++;
			if (status & RD_CE)
				np->stats.rx_crc_errors++;
			if (status & RD_TL)
				np->stats.rx_length_errors++;
		} else {
			pkt_ptr = bus_to_virt(le32_to_cpu(np->rx_ring[np->rx_head].buf1_addr));
			pkt_len = ((status & RD_FL) >> 16) - 4;

			skb = dev_alloc_skb(pkt_len + 2);
			if (skb == NULL) {
				printk("%s: Memory squeeze, dropping.\n",
				       dev->name);
				np->stats.rx_dropped++;
				break;
			}
			skb->dev = dev;
			skb_reserve(skb, 2); /* 16 byte align */

			//pkt_ptr = P1ADDR(pkt_ptr);
			//dma_cache_inv(pkt_ptr, pkt_len);
			memcpy(skb->data, pkt_ptr, pkt_len);
			skb_put(skb, pkt_len);

			//eth_dbg_rx(skb, pkt_len);
			skb->protocol = eth_type_trans(skb,dev);
			netif_rx(skb);	/* pass the packet to upper layers */
			dev->last_rx = jiffies;
			np->stats.rx_packets++;
			np->stats.rx_bytes += pkt_len;
		}
		np->rx_ring[np->rx_head].status = cpu_to_le32(R_OWN);

		np->rx_head ++;
		if (np->rx_head >= NUM_RX_DESCS)
			np->rx_head = 0;
		status = le32_to_cpu(np->rx_ring[np->rx_head].status);
	}
}

/*
 * Tx timeout routine 
 */
static void jz_eth_tx_timeout(struct net_device *dev)
{
	struct jz_eth_private *np = (struct jz_eth_private *)netdev_priv(dev);

	jz_init_hw(dev);
	np->stats.tx_errors ++;
	netif_wake_queue(dev);
}

/*
 * One packet was transmitted
 */
static void eth_txdone(struct net_device *dev)
{
	struct jz_eth_private *np = (struct jz_eth_private*)netdev_priv(dev);
	int tx_tail = np->tx_tail;

	while (tx_tail != np->tx_head) {
		int entry = tx_tail % NUM_TX_DESCS;
		s32 status = le32_to_cpu(np->tx_ring[entry].status);
		if(status < 0) break;
		if (status & TD_ES ) {       /* Error summary */
			np->stats.tx_errors++;
			if (status & TD_NC) np->stats.tx_carrier_errors++;
			if (status & TD_LC) np->stats.tx_window_errors++;
			if (status & TD_UF) np->stats.tx_fifo_errors++;
			if (status & TD_DE) np->stats.tx_aborted_errors++;
			if (np->tx_head != np->tx_tail)
				writel(1, DMA_TPD);  /* Restart a stalled TX */
		} else
			np->stats.tx_packets++;
		/* Update the collision counter */
		np->stats.collisions += ((status & TD_EC) ? 16 : ((status & TD_CC) >> 3));
		/* Free the original skb */
		if (np->tx_skb[entry]) {
			dev_kfree_skb_irq(np->tx_skb[entry]);
			np->tx_skb[entry] = 0;
		}
		tx_tail++;
	}
	if (np->tx_full && (tx_tail + NUM_TX_DESCS > np->tx_head + 1)) {
		/* The ring is no longer full */
		np->tx_full = 0;
		netif_start_queue(dev);
	}
	np->tx_tail = tx_tail;
}

/*
 * Update the tx descriptor
 */
static void load_tx_packet(struct net_device *dev, char *buf, u32 flags, struct sk_buff *skb)
{
	struct jz_eth_private *np = (struct jz_eth_private *)netdev_priv(dev);
	int entry = np->tx_head % NUM_TX_DESCS;
	
	np->tx_ring[entry].buf1_addr = cpu_to_le32(virt_to_bus(buf));
	np->tx_ring[entry].desc1 &= cpu_to_le32((TD_TER | TD_TCH));
	np->tx_ring[entry].desc1 |= cpu_to_le32(flags);
	np->tx_ring[entry].status = cpu_to_le32(T_OWN);
	np->tx_skb[entry] = skb;
}

/*
 * Transmit one packet
 */
static int jz_eth_send_packet(struct sk_buff *skb, struct net_device *dev)
{
	struct jz_eth_private *np = (struct jz_eth_private *)netdev_priv(dev);
	u32 length;

	if (np->tx_full) {
		return 0;
	}
#ifdef CONFIG_FPGA
	mdelay(10);
#else
	udelay(500);	/* FIXME: can we remove this delay ? */
#endif
	length = (skb->len < ETH_ZLEN) ? ETH_ZLEN : skb->len;
	dma_cache_wback((unsigned long)skb->data, length);
	load_tx_packet(dev, (char *)skb->data, TD_IC | TD_LS | TD_FS | length, skb);
	spin_lock_irq(&np->lock);
	np->tx_head ++;
	np->stats.tx_bytes += length;
	writel(1, DMA_TPD);		/* Start the TX */
	dev->trans_start = jiffies;	/* for timeout */
	if (np->tx_tail + NUM_TX_DESCS > np->tx_head + 1) {
		np->tx_full = 0;
	}
	else {
		np->tx_full = 1;
		netif_stop_queue(dev);
	}
	spin_unlock_irq(&np->lock);

	return 0;
}

/*
 * Interrupt service routine
 */
static irqreturn_t jz_eth_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct jz_eth_private *np = netdev_priv(dev);
	u32 sts;
	int i;

	spin_lock(&np->lock);

	writel((readl(DMA_IMR) & ~IMR_ENABLE), DMA_IMR); /* Disable interrupt */

	for (i = 0; i < 100; i++) {
		sts = readl(DMA_STS);
		writel(sts, DMA_STS);	/* clear status */

		if (!(sts & IMR_DEFAULT)) break;

		if (sts & (DMA_INT_RI | DMA_INT_RU)) /* Rx IRQ */
			eth_rxready(dev);
		if (sts & (DMA_INT_TI | DMA_INT_TU)) /* Tx IRQ */
			eth_txdone(dev); 

		/* check error conditions */
		if (sts & DMA_INT_FB){      /* fatal bus error */
			STOP_ETH;
			errprintk("%s: Fatal bus error occurred, sts=%#8x, device stopped.\n",dev->name, sts);
			break;
		}

		if (sts & DMA_INT_UN) {     /* Transmit underrun */
			u32 omr;
			omr = readl(DMA_OMR);
			if (!(omr & OMR_SF)) {
				omr &= ~(OMR_ST | OMR_SR);
				writel(omr, DMA_OMR);
				while (readl(DMA_STS) & STS_TS);  /* wait for stop */
				if ((omr & OMR_TR) < OMR_TR) {  /* ? */
					omr += TR_24;
				} else {
					omr |= OMR_SF;
				}
				writel(omr | OMR_ST | OMR_SR, DMA_OMR);
			}
		}	
	}

	writel(readl(DMA_IMR) | IMR_ENABLE, DMA_IMR); /* enable interrupt */

	spin_unlock(&np->lock);

	return IRQ_HANDLED;
}

#if 0 //def CONFIG_PM
/*
 * Suspend the ETH interface.
 */
static int jz_eth_suspend(struct net_device *dev, int state)
{
	struct jz_eth_private *jep = (struct jz_eth_private *)netdev_priv(dev);
	unsigned long flags, tmp;

	printk("ETH suspend.\n");

	if (!netif_running(dev)) {
		return 0;
	}

	netif_device_detach(dev);

	spin_lock_irqsave(&jep->lock, flags);

	/* Disable interrupts, stop Tx and Rx. */
	REG32(DMA_IMR) = 0;
	STOP_ETH;

	/* Update the error counts. */
	tmp = REG32(DMA_MFC);
	jep->stats.rx_missed_errors += (tmp & 0x1ffff);
	jep->stats.rx_fifo_errors += ((tmp >> 17) & 0x7ff);

	spin_unlock_irqrestore(&jep->lock, flags);

	return 0;
}

/*
 * Resume the ETH interface.
 */
static int jz_eth_resume(struct net_device *dev)
{
	printk("ETH resume.\n");

	if (!netif_running(dev))
		return 0;

	jz_init_hw(dev);

	netif_device_attach(dev);
	jz_eth_tx_timeout(dev);
	netif_wake_queue(dev);

	return 0;
}

static int jz_eth_pm_callback(struct pm_dev *dev, pm_request_t rqst, void *data)
{
	int ret;

	if (!dev->data)
		return -EINVAL;

	switch (rqst) {
	case PM_SUSPEND:
		ret = jz_eth_suspend((struct net_device *)dev->data,
				     (int)data);
		break;

	case PM_RESUME:
		ret = jz_eth_resume((struct net_device *)dev->data);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

#endif /* CONFIG_PM */

static const struct net_device_ops jz_eth_netdev_ops = {
	.ndo_open		= jz_eth_open,
	.ndo_stop		= jz_eth_close,
	.ndo_get_stats		= jz_eth_get_stats,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_do_ioctl		= jz_eth_do_ioctl,
	.ndo_start_xmit		= jz_eth_send_packet,
	.ndo_set_multicast_list	= jz_set_multicast_list,
	.ndo_tx_timeout		= jz_eth_tx_timeout,
};

static int __init jz_eth_init(void)
{
	struct net_device *dev;
	struct jz_eth_private *np;
	int err;

	dev = alloc_etherdev(sizeof(struct jz_eth_private));
	if (!dev) {
		printk(KERN_ERR "%s: alloc_etherdev failed\n", DRV_NAME);
		return -ENOMEM;
	}
	
	netdev = dev;
	np = netdev_priv(dev);
	memset(np, 0, sizeof(struct jz_eth_private));

	np->vaddr_rx_buf = (u32)dma_alloc_noncoherent(NULL, NUM_RX_DESCS*RX_BUF_SIZE, 
						      &np->dma_rx_buf, 0);

	if (!np->vaddr_rx_buf) {
		printk(KERN_ERR "%s: Cannot alloc dma buffers\n", DRV_NAME);
		unregister_netdev(dev);
		free_netdev(dev);
		return -ENOMEM;
	}

	np->dma_rx_ring = virt_to_bus(np->rx_ring);
	np->dma_tx_ring = virt_to_bus(np->tx_ring);
	np->full_duplex = 1;
	np->link_state = 1;

	spin_lock_init(&np->lock);

	dev->irq = IRQ_ETH;
	dev->watchdog_timeo = ETH_TX_TIMEOUT;
	dev->netdev_ops = &jz_eth_netdev_ops;

	/* configure MAC address */
	get_mac_address(dev);

	if ((err = register_netdev(dev)) != 0) {
		printk(KERN_ERR "%s: Cannot register net device, error %d\n",
				DRV_NAME, err);
		free_netdev(dev);
		return -ENOMEM;
	}

//#ifdef 0 //CONFIG_PM
//	np->pmdev = pm_register(PM_SYS_DEV, PM_SYS_UNKNOWN, jz_eth_pm_callback);
//	if (np->pmdev)
//		np->pmdev->data = dev;
//#endif

	return 0;
}

static void __exit jz_eth_exit(void)
{
	struct net_device *dev = netdev;
	struct jz_eth_private *np = netdev_priv(dev);

	unregister_netdev(dev);
	dma_free_noncoherent(NULL, NUM_RX_DESCS * RX_BUF_SIZE,
			     (void *)np->vaddr_rx_buf, np->dma_rx_buf);
	free_netdev(dev);
}	

module_init(jz_eth_init);
module_exit(jz_eth_exit);
