/*
 * linux/drivers/mfd/jz_gpio.c -- 4760 gpio support for RetroFW
 *
 * Copyright (C) 2020, PingFlood (pingflood@retrofw.github.io)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/proc_fs.h>

#include <asm/gpio.h>
#include <asm/jzsoc.h>

#define TS_NAME "jz-gpio"

static int jz_gpio_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	static char buf[1024];
	int x, y;

	sprintf(buf, "");

#if defined(CONFIG_LCD_TM370LN4309)
	sprintf(buf + strlen(buf), "480x272 TM370LN4309\n");
#elif defined(CONFIG_LCD_A030JTN01)
	sprintf(buf + strlen(buf), "320X480 A030JTN01\n");
#elif defined(CONFIG_LCD_EJ030NA09B)
	sprintf(buf + strlen(buf), "320X480 EJ030NA09B\n");
#elif defined(CONFIG_LCD_LKWY030A01)
	sprintf(buf + strlen(buf), "320X480 LKWY030A01\n");
#elif defined(CONFIG_LCD_Y030XX067A)
	sprintf(buf + strlen(buf), "320X480 Y030XX067A\n");
#else
	sprintf(buf + strlen(buf), "UNKNOWN LCD PANEL\n");
#endif

	sprintf(buf + strlen(buf), "\n%16s", " ");
	for (x = 3; x >= 0; x--) {
		sprintf(buf + strlen(buf), "%-20d", x);
	}
	sprintf(buf + strlen(buf), "\n%13s", " ");

	for (x = 31; x >= 0; x--) {
		sprintf(buf + strlen(buf), " %d", x % 10);
	}
	sprintf(buf + strlen(buf), "\n");

	for (y = 0; y <= 5; y++) {
		sprintf(buf + strlen(buf), "%X: 0x%08x ", y + 10, __gpio_get_port(y));

		for (x = 31; x >= 0; x--) {
			sprintf(buf + strlen(buf), "%d ", __gpio_get_pin((32 * y + x)));
		}
		sprintf(buf + strlen(buf), "\n");
	}

	return sprintf(page, "%s\n", buf);
}

static int jz_gpio_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char *array[10];
	int i = 0, *ptr;

	if ((ptr = strchr(buffer, '\n')) != NULL) *ptr = '\0';
	if ((ptr = strchr(buffer, '\r')) != NULL) *ptr = '\0';

	while ((array[i] = strsep(&buffer, " ")) != NULL) i++;

	if (i < 2) return count;

	unsigned int port = simple_strtoul(array[0], 0, 16);
	unsigned int bit = simple_strtoul(array[1], 0, 10);
	unsigned int val = simple_strtoul(array[2], 0, 10);

	if (port < 10 || port > 15 || bit > 31) return count;

	unsigned int gpio = __GPIO(port, bit);

	__gpio_as_output(gpio);

	if (val)
		__gpio_set_pin(gpio);
	else
		__gpio_clear_pin(gpio);

	return count;
}

static int __init jz_gpio_init(void)
{
	struct proc_dir_entry *res;

	if (res = create_proc_entry("jz/gpio", 0, NULL)) {
		res->read_proc = jz_gpio_read_proc;
		res->write_proc = jz_gpio_write_proc;
	}

	printk("0xdc: JZ4760 GPIO driver registered\n");
	return 0;
}

static void __exit jz_gpio_exit(void)
{
	remove_proc_entry("jz/gpio", NULL);
}

module_init(jz_gpio_init);
module_exit(jz_gpio_exit);

MODULE_LICENSE("Proprietary");
MODULE_DESCRIPTION("0xdc JZ GPIO Driver");
MODULE_AUTHOR("<pingflood@retrofw.github.io>");
