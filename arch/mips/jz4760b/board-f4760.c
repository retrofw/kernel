/*
 * linux/arch/mips/jz4760b/board-f4760b.c
 *
 * JZ4760B F4760b board setup routines.
 *
 * Copyright (c) 2006-2008  Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <asm/jzsoc.h>

/*
#include <linux/mfd/wm831x/core.h>
#include <linux/mfd/wm831x/auxadc.h>
#include <linux/mfd/wm831x/pdata.h>
#include <linux/mfd/wm831x/irq.h>
#include <linux/mfd/wm831x/watchdog.h>
#include <linux/mfd/wm831x/status.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
*/

#define WM831X_LDO_MAX_NAME 6

extern void (*jz_timer_callback)(void);

static void dancing(void)
{
	static unsigned char slash[] = "\\|/-";
//	static volatile unsigned char *p = (unsigned char *)0xb6000058;
	static volatile unsigned char *p = (unsigned char *)0xb6000016;
	static unsigned int count = 0;
	*p = slash[count++];
	count &= 3;
}

static void f4760b_timer_callback(void)
{
	static unsigned long count = 0;

	if ((++count) % 50 == 0) {
		dancing();
		count = 0;
	}
}

static void __init board_cpm_setup(void)
{
	/* Stop unused module clocks here.
	 * We have started all module clocks at arch/mips/jz4760b/setup.c.
	 */
}

static void __init board_gpio_setup(void)
{
	/*
	 * Initialize SDRAM pins
	 */
}

void __init jz_board_setup(void)
{
	printk("JZ4760B F4760b board setup\n");
//	jz_restart(NULL);
	board_cpm_setup();
	board_gpio_setup();

	jz_timer_callback = f4760b_timer_callback;
}

/**
 * Called by arch/mips/kernel/proc.c when 'cat /proc/cpuinfo'.
 * Android requires the 'Hardware:' field in cpuinfo to setup the init.%hardware%.rc.
 */
const char *get_board_type(void)
{
	return "f4760b";
}

/*****
 * Wm831x init
 *****/
#if 0
struct wm831x_ldo {
	char name[WM831X_LDO_MAX_NAME];
	struct regulator_desc desc;
	int base;
	struct wm831x *wm831x;
	struct regulator_dev *regulator;
};

static int wm8310_pre_init(struct wm831x *wm831x){

	int ret;

        /* close all wm831x regulators . */
	ret = wm831x_set_bits(wm831x,WM831X_LDO_ENABLE,0x7ff,0);
	if (ret != 0)
		dev_err(wm831x->dev, "Failed to close all ldo: %d\n",ret);
	
	return 0;
}

static int wm8310_post_init(struct wm831x *wm831x){
	
	int ret;

	ret = wm831x_reg_unlock(wm831x);
	if (ret != 0) {
		dev_err(wm831x->dev, "Failed to unlock registers: %d\n", ret);
		return -1;
	}
	
	// close wm831x watchdog
	ret = wm831x_set_bits(wm831x,WM831X_WATCHDOG,WM831X_WDOG_ENA,0);
	if (ret != 0)
		dev_err(wm831x->dev, "Failed to close watchdog: %d\n",ret);
	
	// set ON pin timeout period and set secondary action as irq

	ret = wm831x_set_bits(wm831x,WM831X_ON_PIN_CONTROL,
			      WM831X_ON_PIN_SECACT_MASK  | 
			      WM831X_ON_PIN_PRIMACT_MASK |
			      WM831X_ON_PIN_TO_MASK,
			      WM831X_ON_PIN_AS_IRQ );
	if (ret != 0)
		dev_err(wm831x->dev, "Failed to set ON pin timeout period: %d\n",ret);

	wm831x_reg_lock(wm831x);
	//set wm831x LED1 as a flag of charge status
	ret = wm831x_set_bits(wm831x,WM831X_STATUS_LED_1,WM831X_LED1_MASK,
			      WM831X_LED1_CHARGE_STATE);
	if (ret != 0)
		dev_err(wm831x->dev, "Failed to set status of charge on led1: %d\n",ret);

	return 0;
}

struct wm831x_backlight_pdata wm8310_backlight = {

	.isink     = 1,
	.max_uA   = 3000,
};
static struct wm831x_battery_pdata wm8310_battery_data={

	.enable        = 1,
	.fast_enable   = 1,
	.off_mask      = 0,
	.trickle_ilim  = 50,         /** Trickle charge current limit, in mA */
	.vsel          = 4200,       /** Target voltage, in mV */
	.eoc_iterm     = 50,         /** End of trickle charge current, in mA */
	.fast_ilim     = 400,        /** Fast charge current limit, in mA */
	.timeout       = 300,        /** Charge cycle timeout, in minutes */
};

struct wm831x_status_pdata wm8310_led_status[] = {
	{
		.default_src = WM831X_STATUS_CHARGER,
		.name        = "wm831x-status",
	},
};

static struct regulator_init_data wm8310_dcdc[]={
	{
		.constraints = {
			.name    = "wm831x-dcdc1",
//			.boot_on = 0,
		},
	},
	{
		.constraints = {
			.name    = "wm831x-dcdc2",
//			.boot_on = 1,
		},
	 
	},
       	{
		.constraints = {
			.name    = "wm831x-dcdc3",
//			.boot_on = 1,
		},
	},
	{
		.constraints = {
			.name    = "wm831x-dcdc4",
//			.boot_on = 1,
		},
	},
};

static int wm8310_ldo1_init( void *driver_data )
{	
	return 0;
}
struct wm831x_ldo ldo1_driver_data = {
	.base = WM831X_LDO1_CONTROL,
};

static struct regulator_init_data wm8310_ldo[]={
	{
		.constraints = {
			.name     = "wm831x-ldo1",
			.apply_uV = 1,
			.min_uV   = 3300000,
			.max_uV   = 3300000,
			.boot_on  = 0,
		},
		.regulator_init  = &wm8310_ldo1_init,
	},
	{
		.constraints = {
			.name = "wm831x-ldo2",
			.apply_uV = 1,
			.min_uV   = 1200000,
			.max_uV   = 1200000,
			.boot_on = 0,
		},
	 
	},
       	{
		.constraints = {
			.name    = "wm831x-ldo3",
			.apply_uV = 1,
			.min_uV   = 3300000,
			.max_uV   = 3300000,
			.boot_on = 0,
		},
	},
	{
		.constraints = {
			.name = "wm831x-ldo4",
			.apply_uV = 1,
			.min_uV   = 3300000,
			.max_uV   = 3300000,
			.boot_on = 0,
		},
	 
	},
	{
		.constraints = {
			.name    = "wm831x-ldo5",
			.apply_uV = 1,
			.min_uV   = 3300000,
			.max_uV   = 3300000,
			.boot_on = 0,
		},
	},
	{
		.constraints = {
			.name = "wm831x-ldo6",
			.apply_uV = 1,
			.min_uV   = 3300000,
			.max_uV   = 3300000,
			.boot_on = 0,
		},
	 
	},
       	{
		.constraints = {
			.name    = "wm831x-ldo7",
			.apply_uV = 1,
			.min_uV   = 1200000,
			.max_uV   = 1200000,
			.boot_on = 0,
		},
	},
	{
		.constraints = {
			.name = "wm831x-ldo8",
			.apply_uV = 1,
			.min_uV   = 1800000,
			.max_uV   = 1800000,
			.boot_on = 0,
		},
	 
	},
		{
		.constraints = {
			.name    = "wm831x-ldo9",
			.apply_uV = 1,
			.min_uV   = 1800000,
			.max_uV   = 1800000,
			.boot_on = 0,
		},
	},
	{
		.constraints = {
			.name = "wm831x-ldo10",
			.apply_uV = 1,
			.min_uV   = 2800000,
			.max_uV   = 2800000,
			.boot_on = 0,
		},
	 
	},
	{
		.constraints = {
			.name = "wm831x-ldo11",
			.boot_on = 0,
		},
	 
	},

};

static struct wm831x_pdata wm831x_platform_data = {
	.pre_init  = &wm8310_pre_init,
	.post_init = &wm8310_post_init,
	.backlight = &wm8310_backlight,
	.battery   = &wm8310_battery_data,
	.dcdc      = { wm8310_dcdc, wm8310_dcdc+1, wm8310_dcdc+2, wm8310_dcdc+3 },
	.ldo       = { 
			wm8310_ldo,  wm8310_ldo+1,wm8310_ldo+2,wm8310_ldo+3,
			wm8310_ldo+4,wm8310_ldo+5,wm8310_ldo+6,wm8310_ldo+7,
			wm8310_ldo+8,wm8310_ldo+9,wm8310_ldo+10
	},
};

static struct i2c_board_info wm831x_i2c_devs[] __initdata = {
	{
		I2C_BOARD_INFO("wm8310", 0x34),
//		.irq = GPIO_WM831x_IRQ,
		.platform_data = &wm831x_platform_data, 
	}, 
};

static int __init wm831x_platform_init( void )
{
	i2c_register_board_info(0,wm831x_i2c_devs,ARRAY_SIZE(wm831x_i2c_devs));
//	platform_add_devices(aquila_platform_devices, ARRAY_SIZE(aquila_platform_devices));
	return 0;
}

arch_initcall(wm831x_platform_init);
#endif
