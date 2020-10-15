/*
 * linux/drivers/input/keyboard/jz_keypad.c
 *
 * JZ Keypad Driver
 *
 * Copyright (c) 2005 - 2008  Ingenic Semiconductor Inc.
 *
 * Author: Richard <cjfeng@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/version.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <asm/gpio.h>

#include <asm/jzsoc.h>

#define KB_ROWS         3
#define KB_COLS         3

#define SCAN_INTERVAL       (10)

static unsigned short col[KB_COLS] = {85,87,91};
static unsigned short row[KB_ROWS] = {60,61,62};
static unsigned short s0[KB_COLS];
static unsigned short s1[KB_COLS]={7,7,7};
static unsigned short precol,prerow;

static const unsigned int jz_kbd_keycode[KB_COLS * KB_ROWS] = {
	KEY_1, KEY_4, KEY_7,
	KEY_2, KEY_5, 0,
	KEY_3, KEY_6, 0,
};

struct jz_kbd {
	unsigned int keycode[ARRAY_SIZE(jz_kbd_keycode)];
	struct input_dev *input;
	char phys[32];
	
	spinlock_t lock;
        struct timer_list timer;
	
	unsigned int suspended;
	unsigned long suspend_jiffies;
};

static struct jz_kbd g_jz_kbd;

static inline void jz_scan_kbd(unsigned short *s)
{
	int i;

	if (!s)
		return;

	for (i = 0; i < KB_COLS; i++) {

		__gpio_as_input(85); /* row */
		__gpio_as_input(87); /* row */
		__gpio_as_input(91); /* row */
		
		__gpio_as_input(60); /* col */
		__gpio_as_input(61); /* col */
		__gpio_as_input(62); /* col */

		__gpio_clear_pin(col[i]);
		__gpio_as_output(col[i]);

		udelay(1000);
		s[i]=(__gpio_get_pin(60) << 0) | (__gpio_get_pin(61) << 1) |
			(__gpio_get_pin(62) << 2);
	}
}

static void jz_kbd_scankeyboard(struct jz_kbd *kbd_data)
{
	unsigned int row,col;
	unsigned long flags;
	unsigned int num_pressed;

	if (kbd_data->suspended)
		return;

	spin_lock_irqsave(&kbd_data->lock, flags);

	num_pressed = 0;
	jz_scan_kbd(s0);

	/* look for key if pressed down on not, col & row */
	if (s0[0] == 7 && s0[1] == 7 && s0[2] == 7) {
		if (s1[0] != 7 || s1[1] != 7 || s1[2] != 7) {
			/* up */
			input_report_key(kbd_data->input, kbd_data->keycode[prerow * KB_COLS + precol], 0);
			input_sync(kbd_data->input);
		}
		precol = prerow = -1;
		s1[0] = s1[1] = s1[2] = 7;
		spin_unlock_irqrestore(&kbd_data->lock, flags);
		return;
	}

	if (s0[0] == 6 && s0[1] == 7 && s0[2] == 7) {
		row = 0;//K7
		col = 2;
		goto find_row_col;
	}
	if (s0[0] == 7 && s0[1] == 3 && s0[2] == 7) {
		row = 2;//k6
		col = 1;
		goto find_row_col;
	} 
	if (s0[0] == 7 && s0[1] == 5 && s0[2] == 7) {
		row = 1;//k5
		col = 1;
		goto find_row_col;
	}
	if (s0[0] == 7 && s0[1] == 6 && s0[2] == 7) {
		row = 0;//k4
		col = 1;
		goto find_row_col;
	}
	if (s0[0] == 7 && s0[1] == 7 && s0[2] == 3) {
		row = 2;//k3
		col = 0;
		goto find_row_col;
	}
	if (s0[0] == 7 && s0[1] == 7 && s0[2] == 5) {
		row = 1;//k2
		col = 0;
		goto find_row_col;
	}
	if (s0[0] == 7 && s0[1] == 7 && s0[2] == 6) {
		row = 0;//k1
		col = 0;
		goto find_row_col;
	}
	/* 2 or 3 buttons are pressed */
	s0[0] = s0[1] = s0[2] = 7;
	s1[0] = s1[1] = s1[2] = 7;
	prerow = precol = -1;
	spin_unlock_irqrestore(&kbd_data->lock, flags);
	return;
find_row_col:
	if (s1[0] == 7 && s1[1] == 7 && s1[2] == 7) {
		/* down */
		input_report_key(kbd_data->input, kbd_data->keycode[row * KB_COLS + col], 1);
		input_sync(kbd_data->input);
		s1[0] = s0[0];
		s1[1] = s0[1];
		s1[2] = s0[2];

		precol = col;
		prerow = row;
		spin_unlock_irqrestore(&kbd_data->lock, flags);
		return;
	}
	if (s1[0] != 7 || s1[1] != 7 || s1[2] != 7) {
		/* is the same as the preview key */
		if (s0[0] == s1[0] && s0[1] == s1[1] && s0[2] == s1[2]) {
			input_report_key(kbd_data->input, kbd_data->keycode[row * KB_COLS + col], 1);
			input_sync(kbd_data->input);
			s1[0] = s0[0];
			s1[1] = s0[1];
			s1[2] = s0[2];

			precol = col;
			prerow = row;
			spin_unlock_irqrestore(&kbd_data->lock, flags);
			return;
		} else {
			/* the preview key is up and other key is down */
			if (s0[0] != s1[0] || s0[1] != s1[1] || s0[2] != s1[2]) {
				input_report_key(kbd_data->input, kbd_data->keycode[prerow * KB_COLS + precol], 0);
				input_sync(kbd_data->input);
				input_report_key(kbd_data->input, kbd_data->keycode[row * KB_COLS + col], 1);
				input_sync(kbd_data->input);
				s1[0] = s0[0];
				s1[1] = s0[1];
				s1[2] = s0[2];
				precol = col;
				prerow = row;
				spin_unlock_irqrestore(&kbd_data->lock, flags);
				return;
			}
		}
	}
}

static void jz_kbd_timer_callback(unsigned long data)
{
	jz_kbd_scankeyboard(&g_jz_kbd);
	mod_timer(&g_jz_kbd.timer, jiffies + SCAN_INTERVAL);
}

#ifdef CONFIG_PM
static int jz_kbd_suspend(struct platform_device *dev, pm_message_t state)
{
	struct jz_kbd *jz_kbd = platform_get_drvdata(dev);

	printk("%s(): called.\n", __func__);

	jz_kbd->suspended = 1;
	
	return 0;
}

static int jz_kbd_resume(struct platform_device *dev)
{
	struct jz_kbd *jz_kbd = platform_get_drvdata(dev);
	
	printk("%s(): called.\n", __func__);

	jz_kbd->suspend_jiffies = jiffies;
	jz_kbd->suspended = 0;
	
	return 0;
}
#else
#define jz_kbd_suspend NULL
#define jz_kbd_resume      NULL
#endif

static int __init jz_kbd_probe(struct platform_device *dev)
{
	struct input_dev *input_dev;
	int i, error;

	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;
	
	platform_set_drvdata(dev, &g_jz_kbd);
    
	strcpy(g_jz_kbd.phys, "input/kbd0");
	
	spin_lock_init(&g_jz_kbd.lock);
	
	g_jz_kbd.suspend_jiffies = jiffies;
	g_jz_kbd.input = input_dev;
	
	input_dev->private = &g_jz_kbd;
	input_dev->name = "JZ Keypad";
	input_dev->phys = g_jz_kbd.phys;
	input_dev->cdev.dev = &dev->dev;

	input_dev->id.bustype = BUS_PARPORT;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	
	input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_REP) | BIT(EV_SYN);
	input_dev->keycode = g_jz_kbd.keycode; /* keycode array address */
	input_dev->keycodesize = sizeof(unsigned int);
	input_dev->keycodemax = ARRAY_SIZE(jz_kbd_keycode);

	memcpy(g_jz_kbd.keycode, jz_kbd_keycode, sizeof(g_jz_kbd.keycode));

	for (i = 0; i < ARRAY_SIZE(jz_kbd_keycode); i++)
		set_bit(g_jz_kbd.keycode[i], input_dev->keybit);
	
	//clear_bit(0, input_dev->keybit);
	
	__gpio_as_input(85);
	__gpio_as_input(87);
	__gpio_as_input(91);

#if 0
	__gpio_as_input(60);
	__gpio_as_input(61);
	__gpio_as_input(62);
#endif

	/* Init Keyboard rescan timer */
	init_timer(&g_jz_kbd.timer);
	g_jz_kbd.timer.function = jz_kbd_timer_callback;
	g_jz_kbd.timer.data = (unsigned long)&g_jz_kbd;
	mod_timer(&g_jz_kbd.timer, jiffies + SCAN_INTERVAL);

	error = input_register_device(input_dev);
	if (error) {
		pr_err("gpio-keys: Unable to register input device, "
			"error: %d\n", error);
	}
	printk("input: %s Keypad Registered.\n", JZ_SOC_NAME);
	
	return 0;
}

static int jz_kbd_remove(struct platform_device *dev)
{
	struct jz_kbd *jz_kbd = platform_get_drvdata(dev);
	 
	del_timer_sync(&jz_kbd->timer);
	
	__gpio_as_input(85);
	__gpio_as_input(87);
	__gpio_as_input(91);

	/* These pins is conficting with cs8900a's CS RD WE pins on JZ4740-PAVO board */
	__gpio_as_input(60);
	__gpio_as_input(61);
	__gpio_as_input(62);	 

	input_unregister_device(jz_kbd->input);

	return 0;
}

static struct platform_driver jz_kbd_driver = {
	.probe      = jz_kbd_probe,
	.remove     = jz_kbd_remove,
	.suspend    = jz_kbd_suspend,
	.resume     = jz_kbd_resume,
	.driver     = {
		.name   = "jz-keypad",
	},
};

/*
 * Jz Keyboard Device
 */
static struct platform_device jzkbd_device = {
	.name		= "jz-keypad",
	.id		= -1,
};

static int __init jz_kbd_init(void)
{
	platform_device_register(&jzkbd_device);
	return platform_driver_register(&jz_kbd_driver);
}

static void __exit jz_kbd_exit(void)
{
	platform_device_unregister(&jzkbd_device);
	platform_driver_unregister(&jz_kbd_driver);
}

module_init(jz_kbd_init);
module_exit(jz_kbd_exit);

MODULE_AUTHOR("Richard");
MODULE_DESCRIPTION("JZ keypad driver");
MODULE_LICENSE("GPL");
