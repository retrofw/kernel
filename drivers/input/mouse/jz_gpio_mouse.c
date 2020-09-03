/*
 * Driver for simulating a mouse on GPIO lines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/input-polldev.h>
#include <linux/gpio_mouse.h>
#include <asm/jzsoc.h>
#include <linux/proc_fs.h>

struct jz_mouse {
	struct input_polled_dev *poll_dev;
};

struct jz_mouse jz_gpio_mouse;
int mouse_step;

/*
 * Timer function which is run every scan_ms ms when the device is opened.
 * The dev input variable is set to the the input_dev pointer.
 */
static void gpio_mouse_scan(struct input_polled_dev *dev)
{
	if (!mouse_step) return;
	struct gpio_mouse_platform_data *gpio = dev->private;
	struct input_dev *input_dev = dev->input;

	input_report_key(input_dev, BTN_LEFT,   __gpio_get_pin(UMIDO_KEY_A) || __gpio_get_pin(UMIDO_KEY_L));
	input_report_key(input_dev, BTN_RIGHT,  __gpio_get_pin(UMIDO_KEY_B) || __gpio_get_pin(UMIDO_KEY_R) || __gpio_get_pin(UMIDO_KEY_Y));
	input_report_key(input_dev, BTN_MIDDLE, __gpio_get_pin(UMIDO_KEY_X));
	input_report_rel(input_dev, REL_X, mouse_step * (__gpio_get_pin(UMIDO_KEY_LEFT) - __gpio_get_pin(UMIDO_KEY_RIGHT)));
	input_report_rel(input_dev, REL_Y, mouse_step * (__gpio_get_pin(UMIDO_KEY_UP)   - __gpio_get_pin(UMIDO_KEY_DOWN)));
	input_sync(input_dev);
}

static int jz_gpio_mouse_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	return sprintf(page, "%u\n", mouse_step);
}

static int jz_gpio_mouse_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	mouse_step = simple_strtol(buffer, 0, 10);
	return count;
}

static int __init gpio_mouse_init(void)
{
	struct input_polled_dev *poll_dev;
	struct input_dev *input_dev;
	int error;

	mouse_step = 0; // start disabled

	poll_dev = input_allocate_polled_device();
	if (!poll_dev) {
		printk("not enough memory for input device\n");
		error = -ENOMEM;
		goto fail;
	}

	jz_gpio_mouse.poll_dev = poll_dev;

	poll_dev->private = &jz_gpio_mouse;
	poll_dev->poll = gpio_mouse_scan;
	poll_dev->poll_interval = 20;

	input_dev = poll_dev->input;
	input_dev->name = "JZ GPIO mouse";
	input_dev->phys = "jz-gpio-mouse/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0002;
	input_dev->id.version = 0x0100;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REL);
	input_dev->relbit[0] = BIT_MASK(REL_X) | BIT_MASK(REL_Y);
	input_dev->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_LEFT) | BIT_MASK(BTN_MIDDLE) | BIT_MASK(BTN_RIGHT);

	error = input_register_polled_device(jz_gpio_mouse.poll_dev);
	if (error) {
		printk("could not register input device\n");
		goto fail;
	}

	struct proc_dir_entry *res;
	res = create_proc_entry("jz/mouse", 0, NULL);
	if (res) {
		res->read_proc = jz_gpio_mouse_read_proc;
		res->write_proc = jz_gpio_mouse_write_proc;
	}

	return 0;

 fail:
	input_free_polled_device(poll_dev);
	return error;
}
module_init(gpio_mouse_init);

static void __exit gpio_mouse_exit(void)
{
	remove_proc_entry("jz/mouse", 0);
	input_unregister_polled_device(jz_gpio_mouse.poll_dev);
	input_free_polled_device(jz_gpio_mouse.poll_dev);
}
module_exit(gpio_mouse_exit);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("0xdc JZ GPIO mouse driver");
MODULE_AUTHOR("<pingflood@retrofw.github.io>");
MODULE_ALIAS("platform:jz_gpio_mouse");
