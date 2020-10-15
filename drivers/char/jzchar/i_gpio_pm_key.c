/*
 * linux/drivers/char/jzchar/i_gpio_pm_key.c
 *
 * GPIO PM Key Handling.
 *
 * Author: River Wang <zwang@ingenic.cn>
 *
 * Copyright (C) 2005-2010  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * 2009-03-07 River Wang:
 * 		Code clean #1 for iRiver D8. 
 *		Timer rountines has been re-written.
 *		Now only 1 timer is used to count down.
 *		GPIO pin reading routines has been improved.
 *
 * 2010-03-29 River Wang:
 * 		Code clean & refactor #2 for Noah.
 * 		Board definations may need some more works.
 */

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/major.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/pm.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/jzsoc.h>

#define D(fmt, args...) \
	printk(KERN_ERR "%s(): LINE: %d "fmt"\n", __func__, __LINE__, ##args)

/* ------------ CUSTOM: Modify these as your need. ----------------- */
#define	GPIO_PM_DETECT_PERIOD		(HZ / 2)    /* unit: jiffies.*/
#define GPIO_PM_INT_REACTIVE_DELAY	2	    /* unit: period. */
#define GPIO_PM_TOTAL_COUNT		4	    /* unit: period. */
#define GPIO_PM_SHORT_PRESS_COUNT	4	    /* unit: period. */
/* ------------------------------------------------------------------*/

enum {
	GPIO_PM_SHORT_PRESSED = KOBJ_REMOVE,
	GPIO_PM_LONG_PRESSED = KOBJ_ADD,
};

/* ---------------- CUSTOM: Board Definations. ----------- */
#if defined (CONFIG_SOC_JZ4750) || (CONFIG_SOC_JZ4750D)
#define GPIO_PW_I	GPIO_WAKEUP
#endif

#define GPIO_PM_PIN	GPIO_PW_I
#define GPIO_PM_IRQ	(IRQ_GPIO_0 + GPIO_PM_PIN)

#define gpio_pm_pin_as_irq()	__gpio_as_irq_fall_edge(GPIO_PM_PIN)

#define GPIO_PM_PIN_DOWN	0
/* ------------------------------------------------------- */

#define DRV_NAME "gpio-pm"

MODULE_AUTHOR("River Wang <zwang@ingenic.cn>");
MODULE_DESCRIPTION("GPIO PM Key Handling.");
MODULE_LICENSE("GPL");

struct drv_data {
	struct timer_list suspend_timer;
	struct timer_list int_reactive_timer;

	unsigned int suspend_timer_count;
	unsigned int int_reactive_timer_count;

	unsigned int suspend_active;

	int action;
	struct work_struct work;

	struct platform_device *pdev;
};

static struct drv_data g_drv_data;

static unsigned int read_gpio_pin(void)
{
	unsigned int try_loop = 1000;

	unsigned int t, v;
	unsigned int i;
	
	i = try_loop;

	v = t = 0;

	while (i--) {
		t = __gpio_get_pin(GPIO_PM_PIN);
		if (v != t)
			i = try_loop;
		
		v = t;
	}
	
	return v;
}

static void int_reactive_timer_routine(unsigned long data)
{
	struct drv_data *drv = (struct drv_data *)data;

	if (drv->int_reactive_timer_count) {
		drv->int_reactive_timer_count --;
		mod_timer(&drv->int_reactive_timer, jiffies + GPIO_PM_DETECT_PERIOD);
	}else{
		D("Timeout.");

		drv->suspend_active = 0;
	}

	return;
}

static void start_int_reactive(struct drv_data *drv)
{
	gpio_pm_pin_as_irq();

	drv->int_reactive_timer_count = GPIO_PM_INT_REACTIVE_DELAY;

	mod_timer(&drv->int_reactive_timer, jiffies + 1);	

	return;
}

static void suspend_timer_routine(unsigned long data)
{
	struct drv_data *drv = (struct drv_data *)data;

	if (read_gpio_pin() == GPIO_PM_PIN_DOWN) {
		if (drv->suspend_timer_count == 0) {
			D("Long Pressed.");

			drv->action = GPIO_PM_LONG_PRESSED;

			schedule_work(&drv->work);
		}else{
			drv->suspend_timer_count --;
			mod_timer(&drv->suspend_timer, jiffies + GPIO_PM_DETECT_PERIOD);
		}
	}else{
		if (drv->suspend_timer_count <= GPIO_PM_SHORT_PRESS_COUNT) {
			D("Short Pressed.");

			drv->action = GPIO_PM_SHORT_PRESSED;

			schedule_work(&drv->work);
		}else{
			D("Dummy Key.");

			start_int_reactive(drv);
		}
	}

	return;
}

static void start_suspend(struct drv_data *drv)
{
	__gpio_as_input(GPIO_PM_PIN);

	del_timer(&drv->int_reactive_timer);

	drv->suspend_timer_count = GPIO_PM_TOTAL_COUNT;
	drv->suspend_active = 1;

	mod_timer(&drv->suspend_timer, jiffies + 1);	

	return;
}

static void gpio_pm_work(struct work_struct *work)
{
	struct drv_data *drv = (struct drv_data*)container_of(work, struct drv_data, work);

	D("Action: %d.", drv->action);

	kobject_uevent(&drv->pdev->dev.kobj, drv->action);
	
	return;
}

static irqreturn_t gpio_pm_irq(int irq, void *dev_id)
{
	struct drv_data *drv = (struct drv_data *)dev_id;

	D("Called.");	

	if (!drv->suspend_active)
		start_suspend(drv);

	return IRQ_HANDLED;
}

static int gpio_pm_suspend(struct platform_device *pdev, pm_message_t state)
{
	D("Called.");
	
	return 0;
}

static int gpio_pm_resume(struct platform_device *pdev)
{
	struct drv_data *drv = (struct drv_data *)platform_get_drvdata(pdev);
	
	D("Called.");

	start_int_reactive(drv);

	return 0;
}

static int gpio_pm_probe(struct platform_device *pdev)
{
	struct drv_data *drv = &g_drv_data;

	int rv;

	setup_timer(&drv->suspend_timer, 
			suspend_timer_routine, (unsigned long)drv);	

	setup_timer(&drv->int_reactive_timer, 
			int_reactive_timer_routine, (unsigned long)drv);
	
	INIT_WORK(&drv->work, gpio_pm_work);

	platform_set_drvdata(pdev, drv);

	rv = request_irq(GPIO_PM_IRQ, gpio_pm_irq, IRQF_DISABLED, DRV_NAME, drv);
	if (rv) {
		printk("Could not get IRQ %d\n", GPIO_PM_IRQ);

		return rv;
	}
	
	drv->pdev = pdev;

	gpio_pm_pin_as_irq();
		
	printk(KERN_INFO JZ_SOC_NAME": GPIO PM Key Driver Registered.");

	return rv;
}

static int gpio_pm_remove(struct platform_device *pdev)
{
	struct drv_data *drv = (struct drv_data *)platform_get_drvdata(pdev);

	free_irq(GPIO_PM_IRQ, drv);
	
	del_timer(&drv->suspend_timer);	
	del_timer(&drv->int_reactive_timer);
	
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_device gpio_pm_platform_device = {
	.name		= (char *) DRV_NAME,
};

static struct platform_driver gpio_pm_platform_driver = {
	.probe = gpio_pm_probe,
	.remove = gpio_pm_remove,
	.suspend = gpio_pm_suspend,
	.resume = gpio_pm_resume,
	.driver = {
		.name = DRV_NAME,
	},
};

static int __init gpio_pm_setup(void)
{
        platform_driver_register(&gpio_pm_platform_driver);

	return platform_device_register(&gpio_pm_platform_device);
}

static void __exit gpio_pm_cleanup(void)
{
	platform_driver_unregister(&gpio_pm_platform_driver);
	platform_device_unregister(&gpio_pm_platform_device);

	return;
}

module_init(gpio_pm_setup);
module_exit(gpio_pm_cleanup);
