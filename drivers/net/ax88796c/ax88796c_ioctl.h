
 /*============================================================================
 * Module Name: driver.h
 * Purpose:
 * Author:
 * Date:
 * Notes:
 * $Log: driver.h,v $
 * no message
 *
 *
 *=============================================================================
 */

#ifndef _IOCTL_H
#define _IOCTL_H

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ethtool.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/mii.h>

#include <linux/in.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include "command.h"

extern struct ethtool_ops ax88796c_ethtool_ops;

u8 ax88796c_check_power_state (struct net_device *ndev);
void ax88796c_set_power_saving (struct net_device *ndev, u8 ps_level);
int ax88796c_mdio_read_phy (struct net_device *ndev, int phy_id, int loc);
int ax88796c_mdio_read(struct net_device *ndev, int phy_id, int loc);
void ax88796c_mdio_write_phy (struct net_device *ndev,
			      int phy_id, int loc, int val);
void ax88796c_mdio_write(struct net_device *ndev, int phy_id, int loc, int val);
int ax88796c_ioctl(struct net_device *ndev, struct ifreq *ifr, int cmd);
void ax88796c_set_csums (struct net_device *ndev);
#endif

