/*
 * linux/drivers/char/jzchar/backlight.c
 *
 * Backlight handling.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 */
#include <linux/kthread.h>
#include <linux/module.h>
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
#include <linux/proc_fs.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/jzsoc.h>

#include "jzchars.h"


MODULE_AUTHOR("Tony Jih <tonyjih@gmail.com>");
MODULE_DESCRIPTION("Backlight handling");
MODULE_LICENSE("GPL");

typedef enum
{
	BTN_NONE = 0,
	BTN_PRESSED,
	BTN_HELD,
	BTN_RELEASED,
}buttonStatus;

int backlightButton = BTN_NONE;
// extern unsigned backlight_value;
unsigned backlight_value = 80;
bool enable = true;

static void process_button(int* button,uint gpio_pin, bool onStatus)
{
			//LED 0是按下,1是沒按
		if (__gpio_get_pin(gpio_pin) == onStatus)
		{
			switch(backlightButton)
			{
				case BTN_NONE:
				case BTN_RELEASED:
					backlightButton = BTN_PRESSED;
					break;
				case BTN_PRESSED:
					backlightButton = BTN_HELD;
					break;
				case BTN_HELD:
					break;
			}
		}
		else
		{
			switch(backlightButton)
			{
				case BTN_NONE:
				case BTN_RELEASED:
					backlightButton = BTN_NONE;
					break;
				case BTN_PRESSED:
				case BTN_HELD:
					backlightButton = BTN_RELEASED;
					break;
			}
		}

}

static int backlight_control_thread(void *unused)
{
	while(1)
	{
		if (enable)
		{
			process_button(&backlightButton,UMIDO_KEY_LED, 0);
			if (backlightButton == BTN_PRESSED)
			{
				backlight_value += 20;
				if (backlight_value >= 120)
					backlight_value = 5;
				else if (backlight_value > 100)
					backlight_value = 100;
				__lcd_set_backlight_level(backlight_value);
			}
		}
		msleep(50);
	}
	return 0;
}

static int proc_backlight_control_read_proc(
			char *page, char **start, off_t off,
			int count, int *eof, void *data)
{
	return sprintf(page, "%u\n", enable);
}

static int proc_backlight_control_write_proc(
		struct file *file, const char *buffer,
		unsigned long count, void *data)
{
	enable = !!simple_strtol(buffer, 0, 10);
	return count;
}
static struct task_struct *backlight_control_task;

static int __init backlight_control_init(void)
{
	struct proc_dir_entry *res;

	backlight_control_task = kthread_run(backlight_control_thread, NULL, "backlight_control");
	if (IS_ERR(backlight_control_task)) {
		printk(KERN_ERR "jz_backlight_control: FAIL.\n");
		return PTR_ERR(backlight_control_task);
	}
	res = create_proc_entry("jz/backlight_control", 0, NULL);
	if(res)
	{
		res->read_proc = proc_backlight_control_read_proc;
		res->write_proc = proc_backlight_control_write_proc;
		res->data = NULL;
	}

	return 0;
}

static void __exit backlight_control_exit(void)
{
}

module_init(backlight_control_init);
module_exit(backlight_control_exit);

