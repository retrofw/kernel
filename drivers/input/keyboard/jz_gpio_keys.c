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

#include <asm/jzsoc.h>
#include <linux/proc_fs.h>

#define SCAN_INTERVAL		(20)	/* (ms) */
#define POWER_INTERVAL		(2000)	/* (ms) */
#define POFF_INTERVAL		(5000)	/* (ms) */
#define POWER_COUNT		(POWER_INTERVAL / SCAN_INTERVAL)
#define POFF_COUNT		(POFF_INTERVAL / SCAN_INTERVAL)

#define CONFIG_H350_KEYPAD



/*
 * NOTE: maximum 32 GPIO, since we use bits in an unsigned long to store states
 */

static const struct {
	unsigned int gpio;
	unsigned int actlow;	/* Active low */
	unsigned int ncode;	/* Normal keycode */
	unsigned int scode;	/* Special keycode */
	unsigned int sysrq;	/* SYSRQ code */
	unsigned int wakeup;
} jz_button[] = {

#ifdef CONFIG_JZ4740_A320
	{ .gpio = 102,	.actlow = 1,	.ncode = KEY_UP,	.scode = KEY_VOLUMEUP,		.sysrq = 's'	}, /* D-pad up */
	{ .gpio = 123,	.actlow = 1, 	.ncode = KEY_DOWN,	.scode = KEY_VOLUMEDOWN,	.sysrq = 'u'	}, /* D-pad down */
	{ .gpio = 101,	.actlow = 1,	.ncode = KEY_LEFT,	.scode = KEY_BRIGHTNESSDOWN,	.sysrq = 'e'	}, /* D-pad left */
	{ .gpio = 114,	.actlow = 1,	.ncode = KEY_RIGHT,	.scode = KEY_BRIGHTNESSUP,	.sysrq = 'i'	}, /* D-pad right */
	{ .gpio = 96,	.actlow = 1,	.ncode = KEY_LEFTCTRL,							}, /* A button */
	{ .gpio = 97,	.actlow = 1,	.ncode = KEY_LEFTALT,							}, /* B button */
	{ .gpio = 115,	.actlow = 1,	.ncode = KEY_SPACE,							}, /* X button */
	{ .gpio = 98,	.actlow = 1,	.ncode = KEY_LEFTSHIFT,							}, /* Y button */
	{ .gpio = 110,	.actlow = 1,	.ncode = KEY_TAB,	.scode = KEY_EXIT				}, /* Left shoulder button */
	{ .gpio = 111,	.actlow = 1,	.ncode = KEY_BACKSPACE,							}, /* Right shoulder button */
	{ .gpio = 81,	.actlow = 1,	.ncode = KEY_ENTER,							}, /* START button(SYSRQ) */
	{ .gpio = 113,	.actlow = 1,	.ncode = KEY_ESC,	.scode = KEY_MENU,		.sysrq = 'b'	}, /* SELECT button */
	{ .gpio = 118,	.actlow = 1,	.ncode = KEY_PAUSE,							}, /* HOLD */

#define GPIO_POWER		GPIO_WAKEUP	/* Power slider */
#define GPIO_POWER_ACTLOW	1
#define SYSRQ_ALT		10	/* Alternate sysrq button (index in jz_button table) */
#endif

#ifdef CONFIG_JZ4740_PAVO
	{ .gpio = 96,	.actlow = 1,	.ncode = KEY_1 },
	{ .gpio = 97,	.actlow = 1,	.ncode = KEY_2 },
	{ .gpio = 98,	.actlow = 1,	.ncode = KEY_3 },
	{ .gpio = 99,	.actlow = 1,	.ncode = KEY_4 },
#endif

#ifdef CONFIG_H350_KEYPAD
	{ .gpio = (UMIDO_KEY_UP    )/*GPE0*/,	.actlow = UMIDO_KEY_UP_ACTIVELOW    ,	.ncode = KEY_UP,	.scode = KEY_VOLUMEUP,		.sysrq = 's'	}, /* D-pad up */
	{ .gpio = (UMIDO_KEY_DOWN  )/*GPE1*/,	.actlow = UMIDO_KEY_DOWN_ACTIVELOW  , 	.ncode = KEY_DOWN,	.scode = KEY_VOLUMEDOWN,	.sysrq = 'u'	}, /* D-pad down */
	{ .gpio = (UMIDO_KEY_LEFT  )/*GPE2*/,	.actlow = UMIDO_KEY_LEFT_ACTIVELOW  ,	.ncode = KEY_LEFT,	.scode = KEY_BRIGHTNESSDOWN,	.sysrq = 'e'	}, /* D-pad left */
	{ .gpio = (UMIDO_KEY_RIGHT )/*GPE3*/,	.actlow = UMIDO_KEY_RIGHT_ACTIVELOW ,	.ncode = KEY_RIGHT,	.scode = KEY_BRIGHTNESSUP,	.sysrq = 'i'	}, /* D-pad right */
	{ .gpio = (UMIDO_KEY_A     )/*GPC31*/,	.actlow = UMIDO_KEY_A_ACTIVELOW     ,	.ncode = KEY_LEFTCTRL,							}, /* A button */
	{ .gpio = (UMIDO_KEY_B     )/*GPE11*/,	.actlow = UMIDO_KEY_B_ACTIVELOW     ,	.ncode = KEY_LEFTALT,							}, /* B button */
	{ .gpio = (UMIDO_KEY_X     )/*GPD16*/,	.actlow = UMIDO_KEY_X_ACTIVELOW     ,	.ncode = KEY_SPACE,							}, /* X button */
	{ .gpio = (UMIDO_KEY_Y     )/*GPD17*/,	.actlow = UMIDO_KEY_Y_ACTIVELOW     ,	.ncode = KEY_LEFTSHIFT,							}, /* Y button */
	{ .gpio = (UMIDO_KEY_UP_L1 )/*GPE7*/,	.actlow = UMIDO_KEY_UP_L1_ACTIVELOW  ,	.ncode = KEY_TAB,	.scode = KEY_EXIT				}, /* Left shoulder button */
	{ .gpio = (UMIDO_KEY_UP_L2 )/*GPE10*/,	.actlow = UMIDO_KEY_UP_L2_ACTIVELOW ,	.ncode = KEY_BACKSPACE,							}, /* Right shoulder button */
	{ .gpio = (UMIDO_KEY_UP_R1 )/*GPE7*/,	.actlow = UMIDO_KEY_UP_R1_ACTIVELOW  ,	.ncode = KEY_TAB,	.scode = KEY_EXIT				}, /* Left shoulder button */
	{ .gpio = (UMIDO_KEY_UP_R2 )/*GPE10*/,	.actlow = UMIDO_KEY_UP_R2_ACTIVELOW ,	.ncode = KEY_BACKSPACE,							}, /* Right shoulder button */
	{ .gpio = (UMIDO_KEY_DOWN_L1 )/*GPE7*/,	.actlow = UMIDO_KEY_DOWN_L1_ACTIVELOW  ,	.ncode = KEY_TAB,	.scode = KEY_EXIT				}, /* Left shoulder button */
	{ .gpio = (UMIDO_KEY_DOWN_L2 )/*GPE10*/,	.actlow = UMIDO_KEY_DOWN_L2_ACTIVELOW ,	.ncode = KEY_BACKSPACE,							}, /* Right shoulder button */
	{ .gpio = (UMIDO_KEY_DOWN_R1 )/*GPE7*/,	.actlow = UMIDO_KEY_DOWN_R1_ACTIVELOW  ,	.ncode = KEY_TAB,	.scode = KEY_EXIT				}, /* Left shoulder button */
	{ .gpio = (UMIDO_KEY_DOWN_R2 )/*GPE10*/,	.actlow = UMIDO_KEY_DOWN_R2_ACTIVELOW ,	.ncode = KEY_BACKSPACE,							}, /* Right shoulder button */
	{ .gpio = (UMIDO_KEY_START )/*GPD21*/,	.actlow = UMIDO_KEY_START_ACTIVELOW ,	.ncode = KEY_ENTER,							}, /* START button(SYSRQ) */
	{ .gpio = (UMIDO_KEY_SELECT)/*GPE8*/,	.actlow = UMIDO_KEY_SELECT_ACTIVELOW,	.ncode = KEY_ESC,	.scode = KEY_MENU,		.sysrq = 'b'	}, /* SELECT button */

#endif


};




struct jz_kbd {
	unsigned int			keycode [2 * ARRAY_SIZE(jz_button)];
	struct input_polled_dev *	poll_dev;
	unsigned long			normal_state;	/* Normal key state */
	unsigned long			special_state;	/* Special key state */
	unsigned long			sysrq_state;	/* SYSRQ key state */
	unsigned long			actlow;		/* Active low mask */
	unsigned int			power_state;	/* Power slider state */
	unsigned int			power_count;	/* Power slider active count */
};

static unsigned int sdl_key_enable = 1;
extern unsigned int start_mode;
static int proc_alt_read_proc(
			char *page, char **start, off_t off,
			int count, int *eof, void *data)
{
	return sprintf(page, "%lu\n", start_mode);
}

static int proc_alt_write_proc(
			struct file *file, const char *buffer,
			unsigned long count, void *data)
{
	start_mode =  simple_strtoul(buffer, 0, 10);
	return count;
}


static void send_sig_to_init( int sig )
{
	struct task_struct *p;

	for_each_process(p)
	{
		if (is_global_init(p))
			force_sig(sig, p);
	}
}


static void send_sig_all( int sig )
{
#if 0
	struct task_struct *p;

	for_each_process(p)
	{
		if (p->mm && !is_global_init(p)) /* without important stuff */
			force_sig(sig, p);
	}
#endif
}

static void prepare_for_restart(void)
{
#ifdef CONFIG_JZ4740_A320
	/* TODO : rewrite more smart code */
	__gpio_as_output(GPIO_LCD_BACKLIGHT);
	__gpio_clear_pin(GPIO_LCD_BACKLIGHT);
#endif

}

extern unsigned int jz_read_ts_data();




static void jz_kbd_poll (struct input_polled_dev *dev)
{
	if(sdl_key_enable)
	{
		struct jz_kbd *kbd = dev->private;
		struct input_dev *input = kbd->poll_dev->input;
		unsigned int i, p, sync = 0;
		unsigned long s, m, x;
		unsigned int val;

		/* TODO: lock */

		//maddrone: Add wireless gamepad here

		/* Scan raw key states */
		for (s = 0, m = 1, i = 0; i < ARRAY_SIZE(jz_button); i++, m <<= 1)
		{
			if (__gpio_get_pin(jz_button[i].gpio)){
				s |= m;
			}
		}

#if 0
		if(l009_gsensor_flag)
		{
			val = l009_gsensor_read();
			s &= (~val);
		}
#endif

#if 0
		val = jz_read_ts_data();
		s &= (~val);
#endif


		/* Invert active low buttons */
		s ^= kbd->actlow;

#ifdef H350_HOLD_PIN

		if(s != 0)
		{
			if(!__gpio_get_pin(H350_HOLD_PIN))
			{
				return;
			}
		}
#endif
		/* Read power slider state */

		p = 0;

		if (p) {

#if 0
			/* If power slider and SYSRQ_ALT button are pressed... */
#ifdef SYSRQ_ALT
			if (s & (1 << SYSRQ_ALT)) {

				/* Calculate changed button state for system requests */
				x = s ^ kbd->sysrq_state;

				/* Generate system requests (only on keypress) */
				for (i = 0, m = 1; i < ARRAY_SIZE(jz_button); i++, m <<= 1) {
					if ((x & s & m) && jz_button[i].sysrq)
					{
						/* added safety code for reboot */
						if (jz_button[i].sysrq == 'b')
						{
							prepare_for_restart();
							send_sig_to_init(SIGINT);
						}
						else
						{
							handle_sysrq(jz_button[i].sysrq, NULL);
						}
					}
				}

				kbd->power_count = 0;	/* Stop power slider pressed counter */
				kbd->sysrq_state = s;	/* Update current sysrq button state */
			}

			else
#endif
				/* If power slider is pressed... */
			{

				/* Calculate changed button state for special keycodes */
				x = s ^ kbd->special_state;

				/* Generate special keycodes for changed keys */
				for (i = 0, m = 1; i < ARRAY_SIZE(jz_button); i++, m <<= 1) {
					if ((x & m) && jz_button[i].scode) {
						input_report_key(input, jz_button[i].scode, s & m);
						sync++;
					}
				}

				/* If power state just pressed, start counter, otherwise increase it */
				if (!kbd->power_state) kbd->power_count = 1;
				else {
					if (kbd->power_count > 0) {
						if (kbd->power_count == POWER_COUNT) {
							input_report_key(input, KEY_POWER, 1);
							input_report_key(input, KEY_POWER, 0);
							sync++;
						}
						if (kbd->power_count == POFF_COUNT)
						{
							prepare_for_restart();
							send_sig_to_init(SIGUSR2);
						}
						kbd->power_count++;
					}
				}

				if (s) kbd->power_count = 0;	/* If any button pressed, stop counter */
				kbd->special_state = s;		/* Update current special button state */
				kbd->power_state = p;		/* Update power slider state */
			}
#endif
		}

		/* If power slider is NOT pressed... */
		else {

			/* Calculate changed button state for normal keycodes */
			x = s ^ kbd->normal_state;

			/* Generate normal keycodes for changed keys */
			for (i = 0, m = 1; i < ARRAY_SIZE(jz_button); i++, m <<= 1) {
				if ((x & m) && jz_button[i].ncode) {
					input_report_key(input, jz_button[i].ncode, s & m);
					sync++;
				}
			}

			kbd->power_count = 0;	/* Stop power slider pressed counter */
			kbd->normal_state = s;	/* Update current normal button state */
			kbd->power_state = p;	/* Update power slider state */
		}

		/* Synchronize input if any keycodes sent */
		if (sync) input_sync(input);

		/* TODO: unlock */
	}
}

static struct jz_kbd jz_kbd;

static int proc_otg_state_read_proc(
			char *page, char **start, off_t off,
			int count, int *eof, void *data)
{

	//printk("otg value  is %x\n",value);
	int value = 0;

	if (!__gpio_get_pin(GPIO_OTG_ID_PIN)){
		value |= 0x80000000;
	}

	return sprintf(page, "%lu\n", value);
}

static int proc_otg_state_write_proc(
			struct file *file, const char *buffer,
			unsigned long count, void *data)
{
	return count;
}

static int __init jz_kbd_init(void)
{
	struct input_polled_dev *poll_dev;
	struct input_dev *input_dev;
	int i, j, error;

	printk("jz-gpio-keys: scan interval %ums\n", SCAN_INTERVAL);



	//Maddrone: add gpio init
	for(i=0; i<ARRAY_SIZE(jz_button); i++)
	{
#if H350_V0
		if(i != 8 && i != 10)
		{
			__gpio_as_input(jz_button[i].gpio);
			__gpio_enable_pull(jz_button[i].gpio);
		}else{
			__gpio_as_input(jz_button[i].gpio);
			__gpio_disable_pull(jz_button[i].gpio);
		}
#else
		if(i != 16)
		{
			__gpio_as_input(jz_button[i].gpio);
			__gpio_enable_pull(jz_button[i].gpio);
		}else{
			__gpio_as_input(jz_button[i].gpio);
			__gpio_disable_pull(jz_button[i].gpio);
		}

#endif
	}


	poll_dev = input_allocate_polled_device();
	if (!poll_dev) {
		error = -ENOMEM;
		goto fail;
	}

	jz_kbd.poll_dev = poll_dev;

	poll_dev->private = &jz_kbd;
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
		if (jz_button[i].actlow) jz_kbd.actlow |= 1 << i;
		if (jz_button[i].ncode) {
			jz_kbd.keycode[j++] = jz_button[i].ncode;
			__set_bit(jz_button[i].ncode, input_dev->keybit);
		}
		if (jz_button[i].scode) {
			jz_kbd.keycode[j++] = jz_button[i].scode;
			__set_bit(jz_button[i].scode, input_dev->keybit);
		}
	}

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP) | BIT_MASK(EV_SYN);
	input_dev->keycode = jz_kbd.keycode;
	input_dev->keycodesize = sizeof(jz_kbd.keycode[0]);
	input_dev->keycodemax = j;

	error = input_register_polled_device(jz_kbd.poll_dev);
	if (error) goto fail;

#if 1

//maddrone add
	struct proc_dir_entry *res;
	res = create_proc_entry("jz/alt", 0, NULL);
	if(res)
	{
		res->read_proc = proc_alt_read_proc;
		res->write_proc = proc_alt_write_proc;
		res->data = NULL;
	}
#endif


        struct proc_dir_entry *res_otg_status;
        res_otg_status = create_proc_entry("jz/otg_state", 0, NULL);
        if(res_otg_status)
        {
            res_otg_status->read_proc = proc_otg_state_read_proc;
            res_otg_status->write_proc = proc_otg_state_write_proc;
        }


        return 0;

 fail:	input_free_polled_device(poll_dev);
	return error;
}

static void __exit jz_kbd_exit(void)
{
	input_unregister_polled_device(jz_kbd.poll_dev);
	input_free_polled_device(jz_kbd.poll_dev);
}

module_init(jz_kbd_init);
module_exit(jz_kbd_exit);

MODULE_AUTHOR("Ignacio Garcia Perez <iggarpe@gmail.com>");
MODULE_DESCRIPTION("JZ GPIO keys driver");
MODULE_LICENSE("GPLv2");
