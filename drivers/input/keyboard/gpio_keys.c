/*
 *  Keyboard driver for the IngeniC JZ SoC
 *
 *  Copyright (c) 2009 Ignacio Garcia Perez <iggarpe@gmail.com>
 *
 *  Mod: <maddrone@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  TODO(IGP):
 *  - On power slider long press, use 'o' instead of 'b' to power off instead of reboot.
 *  - Button wake up (when idle and low power stuff works).
 *
 */

#include <linux/init.h>
#include <linux/input-polldev.h>
#include <linux/module.h>
#include <linux/sysrq.h>
#include <linux/sched.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/jzsoc.h>
#include <linux/proc_fs.h>

#define SCAN_INTERVAL		(20) /* (ms) */
#define HOLD_COUNT			(300 / SCAN_INTERVAL)
#define REBOOT_COUNT		(4000 / SCAN_INTERVAL)
#define SLEEP_COUNT			(1000 / SCAN_INTERVAL)
#define CONFIG_JZ4750D_L009

/*
 * NOTE: maximum 32 GPIO, since we use bits in an unsigned long to store states
 */

static const struct {
	unsigned int gpio;
	unsigned int actlow; /* Active low */
	unsigned int ncode; /* Normal keycode */
	unsigned int acode; /* Alt keycode */
	unsigned int scode; /* Special keycode */
	unsigned int sysrq; /* SYSRQ code */
	unsigned int wakeup;
} jz_button[] = {
	{ .gpio = UMIDO_KEY_UP,		.actlow = 1,	.ncode = KEY_UP,		.scode = KEY_W,		.sysrq = 's' 	}, /* D-pad up */
	{ .gpio = UMIDO_KEY_DOWN,	.actlow = 1, 	.ncode = KEY_DOWN,		.scode = KEY_S,		.sysrq = 'u' 	}, /* D-pad down */
	{ .gpio = UMIDO_KEY_LEFT,	.actlow = 1,	.ncode = KEY_LEFT,		.scode = KEY_A,		.sysrq = 'e' 	}, /* D-pad left */
	{ .gpio = UMIDO_KEY_RIGHT,	.actlow = 1,	.ncode = KEY_RIGHT,		.scode = KEY_D,		.sysrq = 'i' 	}, /* D-pad right */
	{ .gpio = UMIDO_KEY_A,		.actlow = 1,	.ncode = KEY_LEFTCTRL,	.scode = KEY_1,		.acode = KEY_LEFTALT	}, /* A button */
	{ .gpio = UMIDO_KEY_B,		.actlow = 1,	.ncode = KEY_LEFTALT,	.scode = KEY_2,		.acode = KEY_LEFTCTRL	}, /* B button */
	{ .gpio = UMIDO_KEY_X,		.actlow = 1,	.ncode = KEY_SPACE,		.scode = KEY_4,						}, /* X button */
	{ .gpio = UMIDO_KEY_Y,		.actlow = 1,	.ncode = KEY_LEFTSHIFT,	.scode = KEY_3,						}, /* Y button */
	{ .gpio = UMIDO_KEY_START,	.actlow = 0,	.ncode = KEY_ENTER,		.scode = KEY_HOME,	.sysrq = 'b'	}, /* START button(SYSRQ) */
	{ .gpio = UMIDO_KEY_SELECT,	.actlow = 0,	.ncode = KEY_ESC,											}, /* SELECT button */
	{ .gpio = UMIDO_KEY_L,		.actlow = 1,	.ncode = KEY_TAB,		.scode = KEY_PAGEUP,				}, /* Left shoulder button */
	{ .gpio = UMIDO_KEY_R,		.actlow = 1,	.ncode = KEY_BACKSPACE,	.scode = KEY_PAGEDOWN,				}, /* Right shoulder button */
	{ .gpio = UMIDO_KEY_LED,	.actlow = 1,	.ncode = KEY_LEFTBRACE, .scode = KEY_RIGHTBRACE,			},
	{ .gpio = GPIO_POWER_ON,	.actlow = 1,	.ncode = KEY_END, 		.scode = KEY_POWER,					},
//	{ .gpio = UMIDO_KEY_VOL_UP,	.actlow = 1,	.ncode = KEY_1,							}, /* START button(SYSRQ) */
//	{ .gpio = UMIDO_KEY_VOL_DOWN,	.actlow = 1,	.ncode = KEY_2,						}, /* START button(SYSRQ) */
};

// #define GPIO_POWER		(125)	/* Power slider */
// #define GPIO_POWER_ACTLOW	1
#define GPIO_POWER		13	/* power button (index in jz_button table) */
#define GPIO_SPECIAL	9	/* Alternate sysrq button (index in jz_button table) */
// #define SYSRQ_ALT		10	/* Alternate sysrq button (index in jz_button table) */
#define GPIO_BACKLIGHT	12	/* backlight button (index in jz_button table) */

struct jz_kbd {
	unsigned int			keycode [2 * ARRAY_SIZE(jz_button)];
	struct input_polled_dev *poll_dev;
	unsigned long			normal_state;	/* Normal key state */
	unsigned long			special_state;	/* Special key state */
	unsigned long			sysrq_state;	/* SYSRQ key state */
	unsigned long			backlight_state;/* backlight key state */
	unsigned long			actlow;			/* Active low mask */
	unsigned int			power_state;	/* Power slider state */
	unsigned int			hold_count;		/* Power slider active count */
	unsigned int			special_count;	/* special key active count */
	unsigned int			power_count;	/* power key active count */
};

struct jz_kbd jz_gpio_kbd;

#ifdef GPIO_BACKLIGHT
extern bool backlight_control;
extern int jz4760fb_get_backlight_level(void);
extern void jz4760fb_set_backlight_level(int n);
#endif
int backlight_value = -1;
int alt_keys = 0;

static void jz_kbd_poll(struct input_polled_dev *dev)
{
	struct jz_kbd *kbd = dev->private;
	struct input_dev *input = kbd->poll_dev->input;
	unsigned int i; //, p, sync = 0;
	unsigned long s, m, x;

	/* TODO: lock */

	/* Scan raw key states */
	for (s = 0, m = 1, i = 0; i < ARRAY_SIZE(jz_button); i++, m <<= 1) {
		if (__gpio_get_pin(jz_button[i].gpio)) s |= m;
	}

	/* Invert active low buttons */
	s ^= kbd->actlow;

	/* Read power slider state */
// #ifdef GPIO_POWER_ON
// #ifdef GPIO_POWER_ACTLOW
// 	p = !__gpio_get_pin(GPIO_POWER_ON);
// #else
// 	p = __gpio_get_pin(GPIO_POWER_ON);
// #endif
// #else
// 	p = 0;
// #endif

#ifdef GPIO_BACKLIGHT
	if (s & (1 << GPIO_BACKLIGHT) && !kbd->backlight_state && backlight_control) {
		backlight_value = jz4760fb_get_backlight_level() + 20;
		if (backlight_value >= 120)
			backlight_value = 5;
		else if (backlight_value > 100)
			backlight_value = 100;
		jz4760fb_set_backlight_level(backlight_value);

	}
	kbd->backlight_state = s; /* Update current backlight button state */
#endif

#ifdef SYSRQ_ALT
	if (s & (1 << SYSRQ_ALT)) {

		/* Calculate changed button state for system requests */
		x = s ^ kbd->sysrq_state;

		/* Generate system requests (only on keypress) */
		for (i = 0, m = 1; i < ARRAY_SIZE(jz_button); i++, m <<= 1) {
			if ((x & s & m) && jz_button[i].sysrq) {
				handle_sysrq(jz_button[i].sysrq, NULL);
			}
		}

		kbd->hold_count = 0; /* Stop sysrq pressed counter */
		kbd->sysrq_state = s; /* Update current sysrq button state */
	} else
#endif

#ifdef GPIO_SPECIAL
	if ((s & (1 << GPIO_SPECIAL)) && (s & (1 << GPIO_POWER))) {
		handle_sysrq('k', NULL); /* Kill all */
		while(!__gpio_get_pin(GPIO_POWER_ON));
	}

	if (alt_keys == 2 && (s & (1 << GPIO_SPECIAL))) {
		/* Calculate changed button state for special keycodes */
		x = s ^ kbd->special_state;

		/* If special button just pressed, start counter, otherwise increase it */
		if (!kbd->special_state) {
			kbd->special_count = 1;
		} else {
			if (kbd->special_count < HOLD_COUNT) {
				for (i = 0, m = 1; i < ARRAY_SIZE(jz_button); i++, m <<= 1) {
					if ((x & m) && jz_button[i].scode) {
						input_report_key(input, jz_button[i].scode, s & m);
						kbd->special_count = 0;
					}
				}
			} else {
				/* Calculate changed button state for normal keycodes */
				x = s ^ kbd->normal_state;

				/* Generate normal keycodes for changed keys */
				for (i = 0, m = 1; i < ARRAY_SIZE(jz_button); i++, m <<= 1) {
					if (/*(x & m) &&*/ jz_button[i].ncode) {
						input_report_key(input, jz_button[i].ncode, s & m);
					}
				}
			}

			if (kbd->special_count > 0) {
				kbd->special_count++;
				if (kbd->special_count == HOLD_COUNT) {
					input_report_key(input, jz_button[GPIO_SPECIAL].ncode, 1);
				}
			}
		}

		/* Update current special button state */
		kbd->special_state = s;
	} else
#endif

#ifdef GPIO_POWER
	/* If power button is pressed... */
	if (s & (1 << GPIO_POWER)) {
		for (i = 0, m = 1; i < ARRAY_SIZE(jz_button); i++, m <<= 1) {
			if ((m != (1 << GPIO_POWER)) && (s & m)) {
				goto normal; // I hate this but it's the best way to prevent code duplication
			}
		}
		kbd->power_count++;

		if (kbd->power_count > REBOOT_COUNT) {
			__lcd_set_backlight_level(20);
			handle_sysrq('s', NULL); /* Force sync */
			handle_sysrq('u', NULL); /* Force read-only remount */
			// handle_sysrq('b', NULL); /* Immediate reboot */
			mdelay(200);
			__lcd_set_backlight_level(0);
			while(!__gpio_get_pin(GPIO_POWER_ON));
			handle_sysrq('o', NULL); /* Immediate poweroff */
		}
	#ifdef CONFIG_PM
		else if (kbd->power_count > SLEEP_COUNT && backlight_value > 0) {
			while (backlight_value-- > 0) {
				__lcd_set_backlight_level(backlight_value);
				mdelay(3);
			} // loop ends with backlight_value = -1;
		}
	#endif // CONFIG_PM
	} else
#endif // GPIO_POWER

	{
		normal:
		/* If normal key... */
		for (i = 0, m = 1; i < ARRAY_SIZE(jz_button); i++, m <<= 1) {
			if (jz_button[i].scode) {
				input_report_key(input, jz_button[i].scode, 0);
			}
		}

		/* Calculate changed button state for normal keycodes */
		x = s ^ kbd->normal_state;

		/* Generate normal keycodes for changed keys */
		for (i = 0, m = 1; i < ARRAY_SIZE(jz_button); i++, m <<= 1) {
			if (alt_keys == 1 && jz_button[i].acode) {
				input_report_key(input, jz_button[i].acode, s & m);
			}
			else if (/*(x & m) && */jz_button[i].ncode) {
				input_report_key(input, jz_button[i].ncode, s & m);
			}
		}

		if (kbd->special_count > 0 && kbd->special_count < HOLD_COUNT) {
			input_report_key(input, jz_button[GPIO_SPECIAL].ncode, 1);
			mdelay(40);
		}

	#ifdef CONFIG_PM
		if (kbd->power_count > SLEEP_COUNT && kbd->power_count < REBOOT_COUNT) {
			jz4760fb_set_backlight_level(-2);
		} else if (kbd->power_count > 0) {
			input_report_key(input, jz_button[GPIO_POWER].ncode, 1);
			mdelay(40);
		}
		if (backlight_value < 0) {
			backlight_value = jz4760fb_get_backlight_level();
		}
	#endif // CONFIG_PM

		kbd->power_count = kbd->special_count = kbd->hold_count = 0; /* Stop hold pressed counter */
		kbd->normal_state = s; /* Update current normal button state */
		kbd->power_state = s & (1 << GPIO_POWER); /* Update power slider state */
		kbd->special_state = s & (1 << GPIO_SPECIAL); /* Update special state */
	}

	/* Synchronize input if any keycodes sent */
	// if (sync) input_sync(input);
	/* TODO: unlock */
}

static int jz_alt_keys_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	return sprintf(page, "%u\n", alt_keys);
}

static int jz_alt_keys_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	alt_keys = simple_strtoul(buffer, 0, 10);
	return count;
}

static int __init jz_kbd_init(void)
{
	struct input_polled_dev *poll_dev;
	struct input_dev *input_dev;
	int i, j, error;

	printk("jz-gpio-keys: scan interval %ums\n", SCAN_INTERVAL);

	for (i = 0; i < ARRAY_SIZE(jz_button); i++) {
		#if 0
			if (i == 8 || i == 9) {   //select start
				__gpio_as_func0(jz_button[i].gpio);
				__gpio_as_input(jz_button[i].gpio);
				__gpio_disable_pull(jz_button[i].gpio);
			} else {
				__gpio_as_func0(jz_button[i].gpio);
				__gpio_as_input(jz_button[i].gpio);
				__gpio_enable_pull(jz_button[i].gpio);
			}
		#else
			__gpio_as_func0(jz_button[i].gpio);
			__gpio_as_input(jz_button[i].gpio);
			__gpio_enable_pull(jz_button[i].gpio);
		#endif
	}

	poll_dev = input_allocate_polled_device();
	if (!poll_dev) {
		error = -ENOMEM;
		goto fail;
	}

	jz_gpio_kbd.poll_dev = poll_dev;

	poll_dev->private = &jz_gpio_kbd;
	poll_dev->poll = jz_kbd_poll;
	poll_dev->poll_interval = SCAN_INTERVAL;

	input_dev = poll_dev->input;
	input_dev->name = "JZ GPIO keys";
	input_dev->phys = "jz-gpio-keys/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;

	/* Prepare active low mask and keycode array/bits */
	for (i = j = 0; i < ARRAY_SIZE(jz_button); i++) {
		if (jz_button[i].actlow)
			jz_gpio_kbd.actlow |= 1 << i;

		if (jz_button[i].ncode) {
			jz_gpio_kbd.keycode[j++] = jz_button[i].ncode;
			__set_bit(jz_button[i].ncode, input_dev->keybit);
		}

		if (jz_button[i].scode) {
			jz_gpio_kbd.keycode[j++] = jz_button[i].scode;
			__set_bit(jz_button[i].scode, input_dev->keybit);
		}
	}

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP) | BIT_MASK(EV_SYN);
	input_dev->keycode = jz_gpio_kbd.keycode;
	input_dev->keycodesize = sizeof(jz_gpio_kbd.keycode[0]);
	input_dev->keycodemax = j;

	error = input_register_polled_device(jz_gpio_kbd.poll_dev);
	if (error) goto fail;

	struct proc_dir_entry *res;
	res = create_proc_entry("jz/alt", 0, NULL);
	if (res) {
		res->read_proc = jz_alt_keys_read_proc;
		res->write_proc = jz_alt_keys_write_proc;
	}

	return 0;

 fail:
	input_free_polled_device(poll_dev);
	return error;
}

static void __exit jz_kbd_exit(void)
{
	input_unregister_polled_device(jz_gpio_kbd.poll_dev);
	input_free_polled_device(jz_gpio_kbd.poll_dev);
}

module_init(jz_kbd_init);
module_exit(jz_kbd_exit);

MODULE_AUTHOR("Ignacio Garcia Perez <iggarpe@gmail.com>");
MODULE_DESCRIPTION("JZ GPIO keys driver");
MODULE_LICENSE("GPLv2");
