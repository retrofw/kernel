
/*
 * linux/drivers/net/jzcs8900a.c
 *
 * Author: Lucifer  <yliu@ingenic>
 *
 * A Cirrus Logic CS8900A driver for Linux
 * based on the cs89x0 driver written by Russell Nelson,
 * Donald Becker, and others.
 *
 * This source code is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

/*
 * At the moment the driver does not support memory mode operation.
 * It is trivial to implement this, but not worth the effort.
 */

/*
 * TODO:
 *
 *   1. If !ready in send_start(), queue buffer and send it in interrupt handler
 *      when we receive a BufEvent with Rdy4Tx, send it again. dangerous!
 *   2. how do we prevent interrupt handler destroying integrity of get_stats()?
 *   3. Change reset code to check status.
 *   4. Implement set_mac_address and remove fake mac address
 *   5. Link status detection stuff
 *   6. Write utility to write EEPROM, do self testing, etc.
 *   7. Implement DMA routines (I need a board w/ DMA support for that)
 *   8. Power management
 *   9. Add support for multiple ethernet chips
 *  10. Add support for other cs89xx chips (need hardware for that)
 */
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/delay.h>

#include <asm/system.h>
#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/jzsoc.h>

#include "jzcs8900a.h"

#define FULL_DUPLEX
#define INT_PIN                 0

#ifdef CONFIG_SOC_JZ4740
#define CIRRUS_DEFAULT_IO	0xa8000000
#define CIRRUS_DEFAULT_IRQ	107

#elif defined(CONFIG_SOC_JZ4750)
#define CIRRUS_DEFAULT_IO	0xac000000

#ifdef CONFIG_JZ4750_FUWA
#define CIRRUS_DEFAULT_IRQ	(32*4+20+48)
#else
#define CIRRUS_DEFAULT_IRQ	(32*2 +6+48)
#endif

#elif defined(CONFIG_SOC_JZ4760) || defined(CONFIG_SOC_JZ4760B)
#if defined(CONFIG_JZ4760_LEPUS) ||  defined(CONFIG_JZ4760B_LEPUS) ||  defined(CONFIG_JZ4760B_LYNX)
#define CIRRUS_DEFAULT_IO	0xb4000000
#elif defined(CONFIG_JZ4760_CYGNUS) || defined(CONFIG_JZ4760B_CYGNUS)
#define CIRRUS_DEFAULT_IO	0xb5000000
#endif
#define CIRRUS_DEFAULT_IRQ	(GPIO_NET_INT + IRQ_GPIO_0)
#endif

typedef struct {
	struct net_device_stats stats;
	u16 txlen;
} cirrus_t;

static int ethaddr_cmd = 0;
static unsigned char ethaddr_hex[6];
static struct net_device *dev;

/*
 * I/O routines
 */
static void gpio_init_cs8900(void)
{
#ifdef CONFIG_SOC_JZ4740
	__gpio_as_func0(60);             //CS4#
	__gpio_as_func0(61);             //RD#
	__gpio_as_func0(62);             //WR#
	__gpio_as_irq_high_level(59);    //irq
	__gpio_disable_pull(59);         //disable pull
	REG_EMC_SMCR4 |= (1 << 6);       //16bit
#elif defined(CONFIG_SOC_JZ4750)
	__gpio_as_func0(32*2+23);             //CS3#
	__gpio_as_func0(32*2+25);             //RD#
	__gpio_as_func0(32*2+26);             //WR#

#ifdef CONFIG_JZ4750_FUWA
	__gpio_as_irq_high_level(32*4+20);    //irq
	__gpio_disable_pull(32*4+20);         //disable pull
#else
	__gpio_as_irq_high_level(32*2 +6);    //irq
	__gpio_disable_pull(32*2 +6);         //disable pull
#endif

	REG_EMC_SMCR3 |= (1 << 6);            //16bit

#elif defined(CONFIG_SOC_JZ4760) || defined(CONFIG_SOC_JZ4760B)

#define RD_N_PIN (32*0 +16)  //gpa16
#define WE_N_PIN (32*0 +17)  //gpa17

#if defined(CONFIG_JZ4760_LEPUS) ||  defined(CONFIG_JZ4760B_LEPUS) ||  defined(CONFIG_JZ4760B_LYNX)
#define WAIT_N (32*0 + 27)   //WAIT_N--->gpa27
#define CS_PIN (32*0 + 26)   //CS6--->gpa26

#elif defined(CONFIG_JZ4760_CYGNUS) || defined(CONFIG_JZ4760B_CYGNUS)
#define CS_PIN (32*0 + 25)   //CS5--->gpa25
#define WAIT_N (32*0 + 27)   //WAIT_N--->gpa27
#define CS8900_RESET_PIN (32 * 1 +23)  //gpb23
#endif
	__gpio_as_func0(CS_PIN);
	__gpio_as_func0(RD_N_PIN);
	__gpio_as_func0(WE_N_PIN);

	__gpio_as_func0(32 * 1 + 2);
	__gpio_as_func0(32 * 1 + 3);


#if defined(CONFIG_JZ4760_LEPUS) ||  defined(CONFIG_JZ4760B_LEPUS) ||  defined(CONFIG_JZ4760B_LYNX)
	REG_GPIO_PXFUNS(0) = 0x0000ffff;
	REG_GPIO_PXTRGC(0) = 0x0000ffff;
	REG_GPIO_PXSELC(0) = 0x0000ffff;

	__gpio_as_func0(WAIT_N);

	REG_NEMC_SMCR6 &= ~EMC_SMCR_BW_MASK;
	REG_NEMC_SMCR6 |= EMC_SMCR_BW_16BIT;

#elif defined(CONFIG_JZ4760_CYGNUS)
	__gpio_as_output(CS8900_RESET_PIN);
	__gpio_set_pin(CS8900_RESET_PIN);
	udelay(10000);
	__gpio_clear_pin(CS8900_RESET_PIN);

	__gpio_as_func0(WAIT_N);

	REG_NEMC_SMCR5 &= ~EMC_SMCR_BW_MASK;
	REG_NEMC_SMCR5 |= EMC_SMCR_BW_16BIT;
#endif

	__gpio_as_irq_high_level(GPIO_NET_INT);	/* irq */
	__gpio_disable_pull(GPIO_NET_INT);	/* disable pull */
#endif
	udelay(1);
}

static inline u16 cirrus_read (struct net_device *dev,u16 reg)
{
	outw (reg,dev->base_addr + PP_Address);
	return (inw (dev->base_addr + PP_Data));
}

static inline void cirrus_write (struct net_device *dev,u16 reg,u16 value)
{
	outw (reg,dev->base_addr + PP_Address);
	outw (value,dev->base_addr + PP_Data);
}

static inline void cirrus_set (struct net_device *dev,u16 reg,u16 value)
{
	cirrus_write (dev,reg,cirrus_read (dev,reg) | value);
}

static inline void cirrus_clear (struct net_device *dev,u16 reg,u16 value)
{
	cirrus_write (dev,reg,cirrus_read (dev,reg) & ~value);
}

static inline void cirrus_frame_read (struct net_device *dev,struct sk_buff *skb,u16 length)
{
	insw (dev->base_addr,skb_put (skb,length),(length + 1) / 2);
}

static inline void cirrus_frame_write (struct net_device *dev,struct sk_buff *skb)
{
	outsw (dev->base_addr,skb->data,(skb->len + 1) / 2);
}

/*
 * Debugging functions
 */

#ifdef DEBUG
static inline int printable (int c)
{
	return ((c >= 32 && c <= 126) ||
			(c >= 174 && c <= 223) ||
			(c >= 242 && c <= 243) ||
			(c >= 252 && c <= 253));
}

static void dump16 (struct net_device *dev,const u8 *s,size_t len)
{
	int i;
	char str[128];

	if (!len) return;

	*str = '\0';

	for (i = 0; i < len; i++) {
		if (i && !(i % 4)) strcat (str," ");
		sprintf (str,"%s%.2x ",str,s[i]);
	}

	for ( ; i < 16; i++) {
		if (i && !(i % 4)) strcat (str," ");
		strcat (str,"   ");
	}

	strcat (str," ");
	for (i = 0; i < len; i++) sprintf (str,"%s%c",str,printable (s[i]) ? s[i] : '.');

	printk (KERN_DEBUG "%s:     %s\n",dev->name,str);
}

static void hexdump (struct net_device *dev,const void *ptr,size_t size)
{
	const u8 *s = (u8 *) ptr;
	int i;
	for (i = 0; i < size / 16; i++, s += 16) dump16 (dev,s,16);
	dump16 (dev,s,size % 16);
}

static void dump_packet (struct net_device *dev,struct sk_buff *skb,const char *type)
{
	printk (KERN_INFO "%s: %s %d byte frame %.2x:%.2x:%.2x:%.2x:%.2x:%.2x to %.2x:%.2x:%.2x:%.2x:%.2x:%.2x type %.4x\n",
			dev->name,
			type,
			skb->len,
			skb->data[0],skb->data[1],skb->data[2],skb->data[3],skb->data[4],skb->data[5],
			skb->data[6],skb->data[7],skb->data[8],skb->data[9],skb->data[10],skb->data[11],
			(skb->data[12] << 8) | skb->data[13]);
	if (skb->len < 0x100) hexdump (dev,skb->data,skb->len);
}
#endif	/* #ifdef DEBUG */

/*
 * Driver functions
 */

static void cirrus_receive (struct net_device *dev)
{
	cirrus_t *priv = netdev_priv(dev);
	struct sk_buff *skb;
	u16 status,length;

	status = cirrus_read (dev,PP_RxStatus);
	length = cirrus_read (dev,PP_RxLength);

	if (!(status & RxOK)) {
		priv->stats.rx_errors++;
		if ((status & (Runt | Extradata))) priv->stats.rx_length_errors++;
		if ((status & CRCerror)) priv->stats.rx_crc_errors++;
		return;
	}

	if ((skb = dev_alloc_skb (length + 4)) == NULL) {
		priv->stats.rx_dropped++;
		return;
	}

	skb->dev = dev;
	skb_reserve (skb,2);

	cirrus_frame_read (dev,skb,length);
	skb->protocol = eth_type_trans (skb,dev);

	netif_rx (skb);
	dev->last_rx = jiffies;

	priv->stats.rx_packets++;
	priv->stats.rx_bytes += length;
}

static int cirrus_send_start (struct sk_buff *skb,struct net_device *dev)
{
	cirrus_t *priv = netdev_priv(dev);
	u16 status;

	mdelay(10);
	netif_stop_queue (dev);

	cirrus_write (dev,PP_TxCMD,TxStart (After5));
	cirrus_write (dev,PP_TxLength,skb->len);

	status = cirrus_read (dev,PP_BusST);

	if ((status & TxBidErr)) {
		printk (KERN_WARNING "%s: Invalid frame size %d!\n",dev->name,skb->len);
		priv->stats.tx_errors++;
		priv->stats.tx_aborted_errors++;
		priv->txlen = 0;
		return (1);
	}

	if (!(status & Rdy4TxNOW)) {
		//printk (KERN_WARNING "%s: Transmit buffer not free!\n",dev->name);
		priv->stats.tx_errors++;
		priv->txlen = 0;
		/* FIXME: store skb and send it in interrupt handler */
		return (1);
	}

	cirrus_frame_write (dev,skb);
	dev->trans_start = jiffies;

	dev_kfree_skb (skb);

	priv->txlen = skb->len;

	return (0);
}

static irqreturn_t cirrus_interrupt(int irq, void *id)
{
	struct net_device *dev = (struct net_device *) id;
	cirrus_t *priv;
	u16 status;

	priv = (cirrus_t *) netdev_priv(dev);

	while ((status = cirrus_read (dev,PP_ISQ))) {
		switch (RegNum (status)) {
		case RxEvent:
			cirrus_receive (dev);
			break;

		case TxEvent:
			priv->stats.collisions += ColCount (cirrus_read (dev,PP_TxCOL));
			if (!(RegContent (status) & TxOK)) {
				priv->stats.tx_errors++;
				if ((RegContent (status) & Out_of_window)) priv->stats.tx_window_errors++;
				if ((RegContent (status) & Jabber)) priv->stats.tx_aborted_errors++;
				break;
			} else if (priv->txlen) {
				priv->stats.tx_packets++;
				priv->stats.tx_bytes += priv->txlen;
			}
			priv->txlen = 0;
			netif_wake_queue (dev);
			break;

		case BufEvent:
			if ((RegContent (status) & RxMiss)) {
				u16 missed = MissCount (cirrus_read (dev,PP_RxMISS));
				priv->stats.rx_errors += missed;
				priv->stats.rx_missed_errors += missed;
			}
			if ((RegContent (status) & TxUnderrun)) {
				priv->stats.tx_errors++;
				priv->stats.tx_fifo_errors++;
			}
			/* FIXME: if Rdy4Tx, transmit last sent packet (if any) */
			priv->txlen = 0;
			netif_wake_queue (dev);
			break;

		case TxCOL:
			priv->stats.collisions += ColCount (cirrus_read (dev,PP_TxCOL));
			break;

		case RxMISS:
			status = MissCount (cirrus_read (dev,PP_RxMISS));
			priv->stats.rx_errors += status;
			priv->stats.rx_missed_errors += status;
			break;
		default:
			return IRQ_HANDLED;
		}
	}

	return IRQ_HANDLED;
}

static void cirrus_transmit_timeout (struct net_device *dev)
{
	cirrus_t *priv = netdev_priv(dev);
	priv->stats.tx_errors++;
	priv->stats.tx_heartbeat_errors++;
	priv->txlen = 0;
	netif_wake_queue (dev);
}

static int cirrus_start (struct net_device *dev)
{
	int result;

	/* valid ethernet address? */
	if (!is_valid_ether_addr(dev->dev_addr)) {
		printk(KERN_ERR "%s: invalid ethernet MAC address\n",dev->name);
		return (-EINVAL);
	}

	/* install interrupt handler */
	if ((result = request_irq (dev->irq, &cirrus_interrupt, IRQF_DISABLED, dev->name, dev)) < 0) {
		printk (KERN_ERR "%s: could not register interrupt %d\n",dev->name,dev->irq);
		return (result);
	}

	/* enable the ethernet controller */
	cirrus_set (dev,PP_RxCFG,RxOKiE | BufferCRC | CRCerroriE | RuntiE | ExtradataiE);
	cirrus_set (dev,PP_RxCTL,RxOKA | IndividualA | BroadcastA);
	cirrus_set (dev,PP_TxCFG,TxOKiE | Out_of_windowiE | JabberiE);
	cirrus_set (dev,PP_BufCFG,Rdy4TxiE | RxMissiE | TxUnderruniE | TxColOvfiE | MissOvfloiE);
	cirrus_set (dev,PP_LineCTL,SerRxON | SerTxON);
	cirrus_set (dev,PP_BusCTL,EnableRQ);

#ifdef FULL_DUPLEX
	cirrus_set (dev,PP_TestCTL,FDX);
#endif	/* #ifdef FULL_DUPLEX */

	/* start the queue */
	netif_start_queue (dev);
	__gpio_unmask_irq(59);

	//MOD_INC_USE_COUNT;
	return (0);
}

static int cirrus_stop (struct net_device *dev)
{
	/* disable ethernet controller */
	cirrus_write (dev,PP_BusCTL,0);
	cirrus_write (dev,PP_TestCTL,0);
	cirrus_write (dev,PP_SelfCTL,0);
	cirrus_write (dev,PP_LineCTL,0);
	cirrus_write (dev,PP_BufCFG,0);
	cirrus_write (dev,PP_TxCFG,0);
	cirrus_write (dev,PP_RxCTL,0);
	cirrus_write (dev,PP_RxCFG,0);

	/* uninstall interrupt handler */
	free_irq (dev->irq,dev);

	/* stop the queue */
	netif_stop_queue (dev);

	//MOD_DEC_USE_COUNT;

	return (0);
}

static int cirrus_set_mac_address (struct net_device *dev, void *p)
{
	struct sockaddr *addr = (struct sockaddr *)p;
	int i;

	if (netif_running(dev))
		return -EBUSY;

	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

	/* configure MAC address */
	for (i = 0; i < ETH_ALEN; i += 2)
		cirrus_write (dev,PP_IA + i,dev->dev_addr[i] | (dev->dev_addr[i + 1] << 8));

	return 0;
}

static struct net_device_stats *cirrus_get_stats (struct net_device *dev)
{
	cirrus_t *priv = netdev_priv(dev);
	return (&priv->stats);
}

static void cirrus_set_receive_mode (struct net_device *dev)
{
	if ((dev->flags & IFF_PROMISC))
		cirrus_set (dev,PP_RxCTL,PromiscuousA);
	else
		cirrus_clear (dev,PP_RxCTL,PromiscuousA);

	if ((dev->flags & IFF_ALLMULTI) && dev->mc_list)
		cirrus_set (dev,PP_RxCTL,MulticastA);
	else
		cirrus_clear (dev,PP_RxCTL,MulticastA);
}

/*
 * Architecture dependant code
 */

static const struct net_device_ops cirrus_netdev_ops = {
	.ndo_open		= cirrus_start,
	.ndo_stop		= cirrus_stop,
	.ndo_get_stats		= cirrus_get_stats,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address 	= cirrus_set_mac_address,
	.ndo_start_xmit		= cirrus_send_start,
	.ndo_set_multicast_list	= cirrus_set_receive_mode,
	.ndo_tx_timeout		= cirrus_transmit_timeout,
};

/*
 * Driver initialization routines
 */

int __init cirrus_probe(void)
{
	int i;
	u16 value;

	printk (JZ_SOC_NAME": CS8900A driver for Linux (V0.02)\n");

	/* Init hardware for PAVO board */
	gpio_init_cs8900();

	/* Allocate ethernet device */
	dev = alloc_etherdev(sizeof(cirrus_t));
	if (dev == NULL) {
		printk("Unable to alloc new net device.\n");
		return -ENOMEM;
	}

	dev->if_port   = IF_PORT_10BASET;
	dev->base_addr = CIRRUS_DEFAULT_IO;
	dev->irq = CIRRUS_DEFAULT_IRQ;
	dev->watchdog_timeo     = HZ;
	dev->netdev_ops         = &cirrus_netdev_ops;

        if (ethaddr_cmd==1)
	{
		dev->dev_addr[0] = ethaddr_hex[0];
		dev->dev_addr[1] = ethaddr_hex[1];
		dev->dev_addr[2] = ethaddr_hex[2];
		dev->dev_addr[3] = ethaddr_hex[3];
		dev->dev_addr[4] = ethaddr_hex[4];
		dev->dev_addr[5] = ethaddr_hex[5];
	}
	else         //default mac address 00:2a:cc:2a:af:fe
	{
		dev->dev_addr[0] = 0x00;
		dev->dev_addr[1] = 0x62;
		dev->dev_addr[2] = 0xde;
		dev->dev_addr[3] = 0xad;
		dev->dev_addr[4] = 0xbe;
		dev->dev_addr[5] = 0xef;
	}
	/* module parameters override everything */
	if (!dev->base_addr) {
		printk (KERN_ERR
				"%s: No default I/O base address defined. Use io=... or\n"
				"%s: define CIRRUS_DEFAULT_IO for your platform\n",
				dev->name,dev->name);
		return (-EINVAL);
	}

	if (!dev->irq) {
		printk (KERN_ERR
				"%s: No default IRQ number defined. Use irq=... or\n"
				"%s: define CIRRUS_DEFAULT_IRQ for your platform\n",
				dev->name,dev->name);
		return (-EINVAL);
	}
#if 0
	if ((result = check_region (dev->base_addr,16))) {
		printk (KERN_ERR "%s: can't get I/O port address 0x%lx\n",dev->name,dev->base_addr);
		return (result);
	}
#endif
	if (!request_region (dev->base_addr,16,dev->name))
		return -EBUSY;
#if 0
	/* verify EISA registration number for Cirrus Logic */
	if ((value = cirrus_read (dev,PP_ProductID)) != EISA_REG_CODE) {
		printk (KERN_ERR "%s: incorrect signature 0x%.4x\n",dev->name,value);
		return (-ENXIO);
	}
#endif

	/* verify chip version */
	value = cirrus_read (dev,PP_ProductID + 2);
	if (VERSION (value) != CS8900A) {
		printk (KERN_ERR "%s: unknown chip version 0x%.8x\n",dev->name,VERSION (value));
		return (-ENXIO);
	}
	printk (KERN_INFO "%s: CS8900A rev %c detected\n",dev->name,'B' + REVISION (value) - REV_B);

	/* setup interrupt number */
	cirrus_write (dev,PP_IntNum,INT_PIN);

	/* configure MAC address */
	for (i = 0; i < ETH_ALEN; i += 2)
	{
		//printk(" %x",dev->dev_addr[i] | (dev->dev_addr[i + 1] << 8));
		cirrus_write (dev,PP_IA + i,dev->dev_addr[i] | (dev->dev_addr[i + 1] << 8));
	}

	if (register_netdev(dev) != 0) {
		printk(KERN_ERR " Cannot register net device\n");
		free_netdev(dev);
		return -ENOMEM;
	}

	return (0);
}

static inline unsigned char str2hexnum(unsigned char c)
{
	if(c >= '0' && c <= '9')
		return c - '0';
	if(c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if(c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return 0; /* foo */
}

static inline void str2eaddr(unsigned char *ea, unsigned char *str)
{
	int i;

	for(i = 0; i < 6; i++) {
		unsigned char num;

		if((*str == '.') || (*str == ':'))
			str++;
		num = str2hexnum(*str++) << 4;
		num |= (str2hexnum(*str++));
		ea[i] = num;
	}
}

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

//EXPORT_NO_SYMBOLS;

MODULE_AUTHOR ("Lucifer <yliu@ingenic.com>");
MODULE_DESCRIPTION ("Jz CS8900A driver for Linux (V0.02)");
MODULE_LICENSE ("GPL");

//#ifdef MODULE


#if 0
static int io = 0;
static int irq = 0;

module_param(io, int, 0);
MODULE_PARM_DESC (io,"I/O Base Address");
//MODULE_PARM (io,"i");

module_param(irq, int, 0);
MODULE_PARM_DESC (irq,"IRQ Number");
//MODULE_PARM (irq,"i");
#endif

static int __init jzcs8900_init(void)
{
	if (cirrus_probe()) {
                printk(KERN_WARNING "jzcs8900: No cs8900a found\n");
	}

	return 0;
}

static void __exit jzcs8900_exit(void)
{
	release_region(dev->base_addr,16);
	unregister_netdev(dev);
	free_netdev(dev);
}

module_init(jzcs8900_init);
module_exit(jzcs8900_exit);
