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

#undef DEBUG
//#define DEBUG 1
#ifdef DEBUG
#define dprintk(msg...) printk("jz-act8600-battery: " msg)
#else
#define dprintk(msg...)
#endif

int charg_state_irq_enable_flag = 0;


extern int early_suspend_state;
extern int connect_to_adapter;
extern int connect_to_pc;
extern void act8600_enable_charging(void);
extern void act8600_disable_charging(void);

struct act8600_interface {
	int irq;
	int charg_state;
	int ad;
	int usb;
	int state;
	void *cookie;
	struct delayed_work irq_work;
	struct delayed_work charg_state_irq_work;
	struct delayed_work usb_only_connect_judge_work;
	battery_charged_callback_t cb;
};

static int read_gpio_state(int gpio)
{	
	int gpio_state;
	__gpio_as_input(gpio);
	msleep(1);
	gpio_state =  __gpio_get_pin(gpio);
	if(gpio_state){
		__gpio_as_irq_fall_edge(gpio);
	}else{
		__gpio_as_irq_rise_edge(gpio);
	}
	return gpio_state;
}

/*******************************************
if charge led is blink battery is not charge
*******************************************/
static int charg_led_is_blink(struct act8600_interface *interface)
{
	int i;
	int gpio_state = 0;

	__gpio_as_input(interface->charg_state);	
	
	for(i = 10;i > 0; i --){
		msleep(10);
		if(!(__gpio_get_pin(interface->charg_state)))
			gpio_state += 1;
	}
	dprintk("%s:gpio_state = %d\n",__func__,gpio_state);
	read_gpio_state(interface->charg_state);
	if(gpio_state == 10) { //charg led not blink and charg
		return 0;
	} else if(gpio_state == 0){ //charg led not blink and not charg
		return 1;
	} else {
		return 2; //charg led blink	
	}
}

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

void act8600_charging_or_discharg(int usb, int ac, int suspend)
{
	if(ac){
		dprintk("enable charging...\n");
		act8600_enable_charging();
	} else {
		if(suspend) {
			dprintk("enable charging...\n");	
			act8600_enable_charging();
		} else {  
			dprintk("disable charging...\n");
			act8600_disable_charging();
		}
	}
}
int get_state(struct act8600_interface *interface)
{
	char chgst,intr1,otg_con;

	act8600_read_reg(ACT8600_APCH_INTR1,&intr1);
	act8600_read_reg(ACT8600_OTG_CON,&otg_con);
	act8600_read_reg(ACT8600_APCH_STAT,&chgst);

	interface->ad = ((intr1 & ACT8600_APCH_INTR1_INDAT) != 0);	/* if CHGIN valid */
	interface->usb = (((otg_con & ACT8600_OTG_CON_VBUSDAT) != 0)
			  && ((otg_con & ACT8600_OTG_CON_Q1) == 0));	/* if USB charge */
	
	/*if Usb only we judge again, because Usb may be connected to PC or Adapter,PMU can not do this*/
#if 0
	if(interface->ad == 0 && interface->usb ==1) {
		interface->ad = connect_to_adapter;
		interface->usb = connect_to_pc;		
		if(interface->ad == 0 && interface->usb == 0){
			interface->ad = 1;	
		}
		dprintk("judge again : connect_to_adapter : %d connect_to_pc : %d\n",connect_to_adapter,connect_to_pc);	
	}
#endif

	dprintk("usb : %d   dc : %d early_suspend_state : %d\n",interface->usb,interface->ad,early_suspend_state);
	act8600_charging_or_discharg(interface->usb,interface->ad,early_suspend_state);
	if(!(intr1 & ACT8600_APCH_INTR1_TEMPDAT)){
		printk("!!!!!!! Warning battery temperature is outside of valid range !!!!!!!!\n");
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	switch(chgst & ACT8600_APCH_STAT_STATE_MASK)
	{
		case ACT8600_APCH_STAT_STATE_EOC:
			dprintk("################POWER_SUPPLY_STATUS_FULL\n");
			return POWER_SUPPLY_STATUS_FULL;
		case ACT8600_APCH_STAT_STATE_PRE:
		case ACT8600_APCH_STAT_STATE_CHAGE:
			if(charg_led_is_blink(interface)) { 	//if charg led blink battery is not charge
				dprintk("################POWER_SUPPLY_STATUS_NOT_CHARGING LDE BLINK\n");
				return POWER_SUPPLY_STATUS_NOT_CHARGING;
			} else {
				dprintk("################POWER_SUPPLY_STATUS_CHARGING\n");
				return POWER_SUPPLY_STATUS_CHARGING;
			}
		/* CHGIN not valid */
		case ACT8600_APCH_STAT_STATE_SUSPEND:
			if(interface->usb) {
				act8600_set_q3(1);
				if(charg_led_is_blink(interface)) {  //if charg led blink battery is not charge
					dprintk("################POWER_SUPPLY_STATUS_NOT_CHARGING LDE BLINK\n");
					return POWER_SUPPLY_STATUS_NOT_CHARGING;
				} else {
					dprintk("################POWER_SUPPLY_STATUS_CHARGING\n");
					return POWER_SUPPLY_STATUS_CHARGING;
				}
			}
			dprintk("################POWER_SUPPLY_STATUS_NOT_CHARGING\n");
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

/*this function depends on usb driver callback*/
static void usb_only_connect_judge_work_func(struct work_struct *work)
{
	struct delayed_work *tmp = container_of(work,struct delayed_work,work);
	struct act8600_interface *interface = container_of(tmp,struct act8600_interface,usb_only_connect_judge_work);
	interface->ad = connect_to_adapter;
	interface->usb = connect_to_pc;
	dprintk("%s ## dc : %d ## usb :%d \n",__func__,interface->ad,interface->usb);
	interface->cb(interface->cookie);
}

static void charg_state_irq_work_func(struct work_struct *work)
{
	struct delayed_work *tmp = container_of(work,struct delayed_work,work);
	struct act8600_interface *interface = container_of(tmp,struct act8600_interface,charg_state_irq_work);

	if(2 != charg_led_is_blink(interface)) {  //charg led not blink
		enable_irq(interface->charg_state + IRQ_GPIO_0);
	} else {
		cancel_delayed_work(&interface->charg_state_irq_work);
		schedule_delayed_work(&interface->charg_state_irq_work,3*HZ);
	}
	interface->state = get_state(interface);
	interface->cb(interface->cookie);
}

static void irq_work_func(struct work_struct *work)
{
	struct delayed_work *tmp = container_of(work,struct delayed_work,work);
	struct act8600_interface *interface = container_of(tmp,struct act8600_interface,irq_work);

	interface->state = get_state(interface);
	/*usb only,delay 3s to judge connect pc or adapter. note: this function depends on usb driver callback*/
	if(interface->usb && (!interface->ad)){ 
		cancel_delayed_work(&interface->usb_only_connect_judge_work);
		schedule_delayed_work(&interface->usb_only_connect_judge_work,3*HZ);
	}
	interface->cb(interface->cookie);

	enable_irq(interface->irq);
}

static irqreturn_t battery_interrupt(int irq, void *dev)
{
	struct act8600_interface *interface = dev;
	disable_irq_nosync(interface->irq);

	cancel_delayed_work(&interface->irq_work);
	schedule_delayed_work(&interface->irq_work,HZ);
	return IRQ_HANDLED; 
}

static irqreturn_t battery_interrupt2(int irq, void *dev)
{
	struct act8600_interface *interface = dev;
	disable_irq_nosync(interface->charg_state + IRQ_GPIO_0);

	cancel_delayed_work(&interface->charg_state_irq_work);
	schedule_delayed_work(&interface->charg_state_irq_work,HZ);
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
	dev->charging_or_discharg = act8600_charging_or_discharg;
	
	interface->cb = cb;
	interface->cookie = cookie;
	interface->irq = data->irq;
	interface->charg_state = data->charg_state;
	interface->state = get_state(interface);
	INIT_DELAYED_WORK(&interface->irq_work, irq_work_func);
	INIT_DELAYED_WORK(&interface->charg_state_irq_work, charg_state_irq_work_func);
	INIT_DELAYED_WORK(&interface->usb_only_connect_judge_work, usb_only_connect_judge_work_func);
    request_irq(interface->irq,battery_interrupt,IRQF_DISABLED,"jz47xx_battery_act8600",(void *)interface);
    request_irq((interface->charg_state + IRQ_GPIO_0),battery_interrupt2,IRQF_DISABLED,"jz47xx_battery_act8600",(void *)interface);
	enable_irq_wake(interface->irq);
	enable_irq_wake(interface->charg_state + IRQ_GPIO_0);
	read_gpio_state(interface->charg_state);
	__gpio_as_irq_fall_edge(data->irq - IRQ_GPIO_0);

	return interface;
}


