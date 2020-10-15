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
#include <linux/act8600_power.h>
#include <linux/syscalls.h>

struct act8600_interface {
	int irq;
	int ad;
	int usb;
	int state;
	void *cookie;
	struct delayed_work irq_work;
	battery_charged_callback_t cb;
};


int get_state(struct act8600_interface *interface);

int act8600_get_ad_online_state(struct act8600_interface *interface)
{
	return interface->ad;
}

int act8600_get_usb_online_state(struct act8600_interface *interface)
{
	return interface->usb;
}

int act8600_get_charging_state(struct act8600_interface *interface)
{
#ifdef	CONFIG_ACT8600_HAS_CHARGE_LED
	return	get_state(interface);
#else
	return interface->state;
#endif	
}

int get_state(struct act8600_interface *interface)
{
	char chgst,intr1,otg_con;

	act8600_read_reg(ACT8600_APCH_INTR1,&intr1);
	act8600_read_reg(ACT8600_OTG_CON,&otg_con);

	interface->ad = ((intr1 & ACT8600_APCH_INTR1_INDAT) != 0);	/* if CHGIN valid */
	interface->usb = (((otg_con & ACT8600_OTG_CON_VBUSDAT) != 0)
			  && ((otg_con & ACT8600_OTG_CON_Q1) == 0));	/* if USB charge */

#if CONFIG_ACTBAT_DC
	if (interface->ad) {
		/* enable CHGIN disconnet interrupt */
		act8600_write_reg(ACT8600_APCH_INTR1,ACT8600_APCH_INTR1_INSTAT);
		act8600_write_reg(ACT8600_APCH_INTR2,ACT8600_APCH_INTR2_INDIS);
	} else {
		/* enable CHGIN connet interrupt */
		act8600_write_reg(ACT8600_APCH_INTR1,ACT8600_APCH_INTR1_INSTAT);
		act8600_write_reg(ACT8600_APCH_INTR2,ACT8600_APCH_INTR2_INCON);
	}
#endif

#if CONFIG_ACTBAT_USB
	if(otg_con & ACT8600_OTG_CON_VBUSDAT) {
		/* enable falling edge interrupt of VBUS */
		act8600_write_reg(ACT8600_OTG_INTR,ACT8600_OTG_INTR_INVBUSF);
	} else {
		/* enable rising edge interrupt of VBUS */
		act8600_write_reg(ACT8600_OTG_INTR,ACT8600_OTG_INTR_INVBUSR);
	}
#endif
	act8600_read_reg(ACT8600_APCH_STAT,&chgst);
	switch(chgst & ACT8600_APCH_STAT_STATE_MASK)
	{
		case ACT8600_APCH_STAT_STATE_EOC:
			return POWER_SUPPLY_STATUS_FULL;
		case ACT8600_APCH_STAT_STATE_PRE:
		case ACT8600_APCH_STAT_STATE_CHAGE:
			return POWER_SUPPLY_STATUS_CHARGING;
		/* CHGIN not valid */
		case ACT8600_APCH_STAT_STATE_SUSPEND:
			if(interface->usb) {
				act8600_set_q3(1);
				return POWER_SUPPLY_STATUS_CHARGING;
			}

			return POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static void irq_work_func(struct work_struct *work)
{
	struct delayed_work *tmp = container_of(work,struct delayed_work,work);
	struct act8600_interface *interface = container_of(tmp,struct act8600_interface,irq_work);

	interface->state = get_state(interface);
	interface->cb(interface->cookie);

	enable_irq(interface->irq);
}

static irqreturn_t battery_interrupt(int irq, void *dev)
{
	struct act8600_interface *interface = dev;
	disable_irq_nosync(irq);
	cancel_delayed_work(&interface->irq_work);
	schedule_delayed_work(&interface->irq_work,HZ);
	return IRQ_HANDLED; 
}

void *jz47xx_battery_register_interface(void *pdata,battery_charged_callback_t cb,void *cookie)
{
	struct act8600_interface *interface = kzalloc(sizeof(*interface), GFP_KERNEL);
	struct act8600_interface_platform_data *data = pdata;
	struct jz47xx_battery_device *dev = cookie;

	dev->get_ad_online_state = act8600_get_ad_online_state;
	dev->get_usb_online_state = act8600_get_usb_online_state;
	dev->get_charging_state = act8600_get_charging_state;
	
	interface->cb = cb;
	interface->cookie = cookie;
	interface->irq = data->irq;
	interface->state = get_state(interface);

	INIT_DELAYED_WORK(&interface->irq_work, irq_work_func);
    	request_irq(interface->irq,battery_interrupt,IRQF_DISABLED,"jz47xx_battery_act8600",interface);
	enable_irq_wake(interface->irq);

	__gpio_as_irq_fall_edge(data->irq - IRQ_GPIO_0);

	return interface;
}


