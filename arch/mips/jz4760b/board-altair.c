/*
 * linux/arch/mips/jz4760b/board-altair.c
 *
 * JZ4760B Altair board setup routines.
 *
 * Copyright (c) 2006-2010  Ingenic Semiconductor Inc.
 *
 * Author: Jason<xwang@ingenic>
 *		Based on board-cygnus.c
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
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

/*
#include <linux/mfd/wm831x/core.h>
#include <linux/mfd/wm831x/auxadc.h>
#include <linux/mfd/wm831x/pdata.h>
#include <linux/mfd/wm831x/irq.h>
#include <linux/mfd/wm831x/watchdog.h>
#include <linux/mfd/wm831x/status.h>
*/
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
//#include <linux/timed_gpio.h>

#include <asm/jzsoc.h>
//#include <asm/jzmmc/jz_mmc_platform_data.h>

#define WM831X_LDO_MAX_NAME 6

void __init board_msc_init(void);

//extern int jz_add_msc_devices(unsigned int controller, struct jz_mmc_platform_data *plat);
extern void (*jz_timer_callback)(void);

#if 0
static void dancing(void)
{
	static unsigned char slash[] = "\\|/-";
//	static volatile unsigned char *p = (unsigned char *)0xb6000058;
	static volatile unsigned char *p = (unsigned char *)0xb6000016;
	static unsigned int count = 0;
	*p = slash[count++];
	count &= 3;
}
#endif

/* MSC SETUP */
/*
static void altair_sdio_gpio_init(struct device *dev)
{
	__gpio_as_msc0_4bit();

	// BT/WLAN reset
	__gpio_as_output(17); //GPA17
	__gpio_clear_pin(17);
	mdelay(10);
	__gpio_as_input(17);
}

static void altair_sdio_power_on(struct device *dev)
{
	__msc0_enable_power();
}

static void altair_sdio_power_off(struct device *dev)
{
	__msc0_disable_power();
}

static unsigned int altair_sdio_status(struct device *dev)
{
	unsigned int status;

	// WIFI virtual 'card detect' status
	status = 1;
	return (status);
}

static struct jz_mmc_platform_data altair_sdio_data = {
	.support_sdio   = 1,
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.init           = altair_sdio_gpio_init,
	.power_on       = altair_sdio_power_on,
	.power_off      = altair_sdio_power_off,
	.status		= altair_sdio_status,
	.max_bus_width  = MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED | MMC_CAP_8_BIT_DATA,
#ifdef CONFIG_JZ_MSC0_BUS_1
	.bus_width      = 1,
#elif defined  CONFIG_JZ_MSC0_BUS_4
	.bus_width      = 4,
#else
	.bus_width      = 8,
#endif
};

static void altair_tf_gpio_init(struct device *dev)
{
	__gpio_as_msc1_4bit();
	__gpio_as_output(GPIO_SD1_VCC_EN_N);
}

static void altair_tf_power_on(struct device *dev)
{
	__msc1_enable_power();
}

static void altair_tf_power_off(struct device *dev)
{
	__msc1_disable_power();
}

static unsigned int altair_tf_status(struct device *dev)
{
	unsigned int status;

	status = (unsigned int) __gpio_get_pin(GPIO_SD1_CD_N);
	return (status);
}

static void altair_tf_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_low_level(MSC1_HOTPLUG_PIN);
	else
		__gpio_as_irq_high_level(MSC1_HOTPLUG_PIN);
}

static struct jz_mmc_platform_data altair_tf_data = {
#ifndef CONFIG_JZ_MSC1_SDIO_SUPPORT
	.support_sdio   = 0,
#else
	.support_sdio   = 1,
#endif
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.status_irq	= MSC1_HOTPLUG_IRQ,
	.detect_pin     = GPIO_SD1_CD_N,
	.init           = altair_tf_gpio_init,
	.power_on       = altair_tf_power_on,
	.power_off      = altair_tf_power_off,
	.status		= altair_tf_status,
	.plug_change	= altair_tf_plug_change,
	.max_bus_width  = MMC_CAP_SD_HIGHSPEED | MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_JZ_MSC1_BUS_1
	.bus_width      = 1,
#else
	.bus_width      = 4,
#endif
};

void __init board_msc_init(void)
{
#ifdef CONFIG_JZ_MSC0
	jz_add_msc_devices(0, &altair_sdio_data);
#endif

#ifdef CONFIG_JZ_MSC1
	jz_add_msc_devices(1, &altair_tf_data);
#endif
}

*/

static void f4760b_timer_callback(void)
{
	static unsigned long count = 0;

	if ((++count) % 50 == 0) {
//		dancing();
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
        __jtag_as_uart3(); /* for GSM modem IW368 */
}

void __init jz_board_setup(void)
{
	printk("JZ4760B Altair board setup\n");
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
	return "Altair";
}

/*****
 * Wm831x init
 *****/

/*
struct wm831x_ldo {
	char name[WM831X_LDO_MAX_NAME];
	struct regulator_desc desc;
	int base;
	struct wm831x *wm831x;
	struct regulator_dev *regulator;
};

static int wm8310_pre_init(struct wm831x *wm831x){
	//close all wm831x regulator .
	int ret;

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
	.trickle_ilim  = 50,
	.vsel          = 4200,
	.eoc_iterm     = 50,
	.fast_ilim     = 400,
	.timeout       = 300,
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
			.min_uV   = 1500000,
			.max_uV   = 1500000,
			.boot_on = 0,
		},

	},
       	{
		.constraints = {
			.name    = "wm831x-ldo3",
			.apply_uV = 1,
			.min_uV   = 3300000,
			.max_uV   = 3300000,
			.boot_on = 1,
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
			.min_uV   = 2800000,
			.max_uV   = 2800000,
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
		.irq = GPIO_WM831x_DETECT + IRQ_GPIO_0,
		.platform_data = &wm831x_platform_data,
	},
};
*/

static struct i2c_board_info altair_i2c0_devs[] __initdata = {
	{
		I2C_BOARD_INFO("ov3640", 0x3c),
	},
	{
	},
};

static struct i2c_board_info altair_i2c1_devs[] __initdata = {
	{
		I2C_BOARD_INFO("jz_mt4d_ts", 0x40),
		.irq = LCD_INT_IRQ,
	},
	{
	},
};

static struct i2c_board_info altair_gpio_i2c_devs[] __initdata = {
#if 0
	{
		I2C_BOARD_INFO("jz_mt4d_ts", 0x40),
		.irq = LCD_INT_IRQ,
	},
#endif
	{
	},
};

static struct i2c_gpio_platform_data altair_i2c_gpio_data = {
	.sda_pin	= CIM_I2C_SDA,
	.scl_pin	= CIM_I2C_SCK,
};

static struct platform_device altair_i2c_gpio_device = {
	.name	= "i2c-gpio",
	.id	= 2,
	.dev	= {
		.platform_data = &altair_i2c_gpio_data,
	},
};

/*
struct timed_gpio vibrator_timed_gpio = {
		.name         = "vibrator",
//		.gpio         = GPIO_VIBRATOR_EN_N,
		.active_low   = 1,
		.max_timeout  = 15000,
};

static struct timed_gpio_platform_data vibrator_platform_data = {
	.num_gpios    = 1,
	.gpios        = &vibrator_timed_gpio,
};

static struct platform_device altair_timed_gpio_device = {
	.name = TIMED_GPIO_NAME,
	.id = 0,
	.dev = {
		.platform_data          = &vibrator_platform_data,
	},

};

static struct platform_device *altair_platform_devices[] __initdata = {
	&altair_timed_gpio_device,
	&altair_i2c_gpio_device,
};
*/

static int __init altair_board_init( void )
{
//	i2c_register_board_info(1, wm831x_i2c_devs, ARRAY_SIZE(wm831x_i2c_devs));
	i2c_register_board_info(0, altair_i2c0_devs, ARRAY_SIZE(altair_i2c0_devs));
	i2c_register_board_info(1, altair_i2c1_devs, ARRAY_SIZE(altair_i2c1_devs));
	i2c_register_board_info(2, altair_gpio_i2c_devs, ARRAY_SIZE(altair_gpio_i2c_devs));
//	platform_add_devices(altair_platform_devices, ARRAY_SIZE(altair_platform_devices));

	/*
	 * for gpio-based bitbanging i2c bus
	 */
	__gpio_as_output(IOSWITCH_EN);
	__gpio_clear_pin(IOSWITCH_EN);

	return 0;
}

arch_initcall(altair_board_init);
