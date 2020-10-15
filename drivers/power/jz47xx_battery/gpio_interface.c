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
#include <linux/gpio_interface.h>
#include <linux/syscalls.h>


struct gpio_interface {
	int irq;
	int ad;
	int usb;
	int state;              
	void *cookie;
	struct delayed_work irq_work;
	battery_charged_callback_t cb;
};
struct gpio_interface_platform_data *data;

void set_dete_pin(struct gpio_detect_pin *dete)
{
	int retval = __gpio_get_pin(dete->pin);
	if(retval)
		__gpio_as_irq_fall_edge(dete->pin);
	else
		__gpio_as_irq_rise_edge(dete->pin);
}

int read_dete_pin(struct gpio_detect_pin *dete) //read and reset irq 
{
	int retval = __gpio_get_pin(dete->pin);    
	if(retval)
		__gpio_as_irq_fall_edge(dete->pin);
	else
		__gpio_as_irq_rise_edge(dete->pin);

	return dete->low_active?!retval:retval;
}

int gpio_get_ad_online_state(void *dev)
{
    struct gpio_interface *interface = dev;
	return interface->ad;
}

int gpio_get_usb_online_state(void *dev)
{
    struct gpio_interface *interface = dev;
	return interface->usb;
}

int gpio_get_charging_state(void *dev)
{
    struct gpio_interface *interface = dev;
	return interface->state;
}


int get_state(struct gpio_interface *interface)
{
	if(data->gpio_dc_dete.pin) {    
		if(read_dete_pin(&data->gpio_dc_dete)) {
 			interface->ad = 1;
		} else {
			interface->ad = 0;
		}
	}

	if (data->gpio_usb_dete.pin) {
		if(read_dete_pin(&data->gpio_usb_dete))
			interface->usb = 1;
		else
			interface->usb = 0;
	}

	if(data->gpio_charg_stat.pin) {
		if(read_dete_pin(&data->gpio_charg_stat)){
            return POWER_SUPPLY_STATUS_CHARGING;
		} else {
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	}
	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static void irq_work_func(struct work_struct *work)
{
	struct delayed_work *tmp = container_of(work,struct delayed_work,work);
	struct gpio_interface *interface = container_of(tmp,struct gpio_interface,irq_work);

	interface->state = get_state(interface);
	interface->cb(interface->cookie);

    if (data->gpio_dc_dete.pin)
		enable_irq((data->gpio_dc_dete.pin + IRQ_GPIO_0));
    if(data->gpio_charg_stat.pin)
        enable_irq((data->gpio_charg_stat.pin + IRQ_GPIO_0));
}

static irqreturn_t battery_interrupt(int irq, void *dev)
{
	struct gpio_interface *interface = dev;

    if(data->gpio_charg_stat.pin)
        disable_irq_nosync((data->gpio_charg_stat.pin + IRQ_GPIO_0));
	if (data->gpio_dc_dete.pin)
		disable_irq_nosync((data->gpio_dc_dete.pin + IRQ_GPIO_0));
	cancel_delayed_work(&interface->irq_work);
	schedule_delayed_work(&interface->irq_work,HZ);
	return IRQ_HANDLED; 
}

void *jz47xx_battery_register_interface(void *pdata,battery_charged_callback_t cb,void *cookie)
{
	struct gpio_interface *interface = kzalloc(sizeof(*interface), GFP_KERNEL);
	struct jz47xx_battery_device *dev = cookie;
    int retval;
    
    data = pdata;
	dev->get_ad_online_state = gpio_get_ad_online_state;
	dev->get_usb_online_state = gpio_get_usb_online_state;
	dev->get_charging_state = gpio_get_charging_state;
	
	interface->cb = cb;
	interface->cookie = cookie;
	interface->state = get_state(interface);

	INIT_DELAYED_WORK(&interface->irq_work, irq_work_func);
    
    if(data->gpio_charg_stat.pin){
        retval = request_irq((data->gpio_charg_stat.pin + IRQ_GPIO_0),
                             battery_interrupt,IRQF_DISABLED,"jz-battery-charg",interface);
        set_dete_pin(&data->gpio_charg_stat);
    }
    
	if(data->gpio_dc_dete.pin){
	    retval = request_irq((data->gpio_dc_dete.pin + IRQ_GPIO_0),
                             battery_interrupt,IRQF_DISABLED,"jz-battery-dc",interface);
        set_dete_pin(&data->gpio_dc_dete);
    }

    
	return interface;
}
