/*
 * linux/drivers/power/jz47xx_battery
 *
 * Battery measurement code for Ingenic JZ SOC.
 *
 * based on tosa_battery.c
 *
 * Copyright (C) 2011 ztyan <ztyan@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <asm/jzsoc.h>

#include <linux/suspend.h>
#include <linux/jz47xx_battery.h>
#include <linux/syscalls.h>

int fake_get_ad_online_state(void *fake_interface)
{
	return 1;
}

int fake_get_usb_online_state(void *fake_interface)
{
	return 1;
}

int fake_get_charging_state(void *fake_interface)
{
	return POWER_SUPPLY_STATUS_CHARGING;
}

void *jz47xx_battery_register_interface(void *pdata,battery_charged_callback_t cb,void *cookie)
{
	struct jz47xx_battery_device *dev = cookie;
	dev->get_ad_online_state = fake_get_ad_online_state;
	dev->get_usb_online_state = fake_get_usb_online_state;
	dev->get_charging_state = fake_get_charging_state;

	return "fake_interface";
}


