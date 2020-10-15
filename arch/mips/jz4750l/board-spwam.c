/*
 * linux/arch/mips/jz4750l/board-spwam.c
 *
 * JZ4750L SPWAM board setup routines.
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
#include <linux/mmc/host.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input/sadc_keys.h>
#include <linux/mfd/jz4750l-sadc.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <asm/jzsoc.h>

extern int __init jz_add_msc_devices(unsigned int controller, struct jz_mmc_platform_data *plat);
extern int __init jz_add_spi_devices(struct spi_board_info *board_info, int board_array);
extern int __init jz_device_register(struct platform_device *pdev,void *pdata);

extern void (*jz_timer_callback)(void);
#ifdef CONFIG_MFD_JZ4750L_SADC
extern struct platform_device jz_adc_device;
extern struct jz_adc_platform_data jz_adc_pdata;
#endif

static void dancing(void)
{

}

static void spwam_timer_callback(void)
{
	static unsigned long count = 0;

	if ((++count) % 50 == 0) {
		dancing();
		count = 0;
	}
}

static void spwam_sd_gpio_init(struct device *dev)
{
	__gpio_as_msc0_4bit();
	__gpio_as_input(GPIO_SD0_CD_N);
}

static void spwam_sd_power_on(struct device *dev)
{
}

static void spwam_sd_power_off(struct device *dev)
{
}

static void spwam_sd_cpm_start(struct device *dev)
{
	__cpm_start_msc(0);
}

static unsigned int spwam_sd_status(struct device *dev)
{
#if defined(CONFIG_JZ_SYSTEM_AT_CARD)
	return 1;
#endif
	unsigned int status;

	status = (unsigned int)__gpio_get_pin(GPIO_SD0_CD_N);
	return (!status);
}

#if 0
static void spwam_sd_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_high_level(MSC0_HOTPLUG_PIN); /* wait remove */
	else
		__gpio_as_irq_low_level(MSC0_HOTPLUG_PIN); /* wait insert */
}
#else
static void spwam_sd_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_rise_edge(MSC0_HOTPLUG_PIN);
	else
		__gpio_as_irq_fall_edge(MSC0_HOTPLUG_PIN);
}
#endif

static unsigned int spwam_sd_get_wp(struct device *dev)
{
#if defined(CONFIG_JZ_SYSTEM_AT_CARD)
	return 0;
#endif
#if 0
	unsigned int status;

	status = (unsigned int) __gpio_get_pin(GPIO_SD0_WP);
	return (status);
#else
	return 0;
#endif
}

struct jz_mmc_platform_data spwam_sd_data = {
#ifndef CONFIG_JZ_MSC0_SDIO_SUPPORT
	.support_sdio   = 0,
#else
	.support_sdio   = 1,
#endif
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
#if defined(CONFIG_SYSTEM_AT_CARD)
	.status_irq	= 0,
	.detect_pin     = 0,
#else
	.status_irq	= MSC0_HOTPLUG_IRQ,
	.detect_pin     = GPIO_SD0_CD_N,
#endif
	.init           = spwam_sd_gpio_init,
	.power_on       = spwam_sd_power_on,
	.power_off      = spwam_sd_power_off,
	.cpm_start	= spwam_sd_cpm_start,
	.status		= spwam_sd_status,
	.plug_change	= spwam_sd_plug_change,
	.write_protect  = spwam_sd_get_wp,
	.max_bus_width  = MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED
#ifdef CONFIG_JZ_MSC0_BUS_4
	| MMC_CAP_4_BIT_DATA
#endif
#ifdef CONFIG_JZ_MSC0_BUS_8
	| MMC_CAP_8_BIT_DATA
#endif
	,
#ifdef CONFIG_JZ_MSC0_BUS_1
	.bus_width      = 1,
#elif defined  CONFIG_JZ_MSC0_BUS_4
	.bus_width      = 4,
#else
	.bus_width      = 8,
#endif
};

static void spwam_tf_gpio_init(struct device *dev)
{
	__gpio_as_msc1_4bit();
	__gpio_as_input(GPIO_SD1_CD_N);
}

static void spwam_tf_power_on(struct device *dev)
{
}

static void spwam_tf_power_off(struct device *dev)
{
}

static void spwam_tf_cpm_start(struct device *dev)
{
	__cpm_start_msc(1);
}

static unsigned int spwam_tf_status(struct device *dev)
{
	unsigned int status;

	status = (unsigned int) __gpio_get_pin(GPIO_SD1_CD_N);
	return (status);
}

#if 0
static void spwam_tf_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_low_level(MSC1_HOTPLUG_PIN);
	else
		__gpio_as_irq_high_level(MSC1_HOTPLUG_PIN);
}
#else
static void spwam_tf_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_fall_edge(MSC1_HOTPLUG_PIN);
	else
		__gpio_as_irq_rise_edge(MSC1_HOTPLUG_PIN);
}
#endif

struct jz_mmc_platform_data spwam_tf_data = {
#ifndef CONFIG_JZ_MSC1_SDIO_SUPPORT
	.support_sdio   = 0,
#else
	.support_sdio   = 1,
#endif
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.status_irq	= MSC1_HOTPLUG_IRQ,
	.detect_pin     = GPIO_SD1_CD_N,
	.init           = spwam_tf_gpio_init,
	.power_on       = spwam_tf_power_on,
	.power_off      = spwam_tf_power_off,
	.cpm_start	= spwam_tf_cpm_start,
	.status		= spwam_tf_status,
	.plug_change	= spwam_tf_plug_change,
	.max_bus_width  = MMC_CAP_SD_HIGHSPEED
#ifdef CONFIG_JZ_MSC1_BUS_4
	| MMC_CAP_4_BIT_DATA
#endif
	,
#ifdef CONFIG_JZ_MSC1_BUS_1
	.bus_width      = 1,
#else
	.bus_width      = 4,
#endif
};

/* SPI NOR */
#ifdef CONFIG_JZ_SPI_NOR
//#define GIGA_SPI_NOR_Q64
#ifdef GIGA_SPI_NOR_Q64
static struct spi_nor_block_info flash_block_info[] = {
	{
		.blocksize	= 64 * 1024,
		.cmd_blockerase	= 0xD8,
		.be_maxbusy	= 1200  /* 1.2s */
	},

	{
		.blocksize	= 32 * 1024,
		.cmd_blockerase	= 0x52,
		.be_maxbusy	= 1000  /* 1s */
	},
};

static struct spi_nor_platform_data spi_nor_pdata = {
	.pagesize	= 256,
	.sectorsize	= 4 * 1024,
	.chipsize	= 8192 * 1024,

	.block_info	= flash_block_info,
	.num_block_info	= ARRAY_SIZE(flash_block_info),

	.addrsize	= 3,
	.pp_maxbusy	= 3,            /* 3ms */
	.se_maxbusy	= 300,          /* 300ms */
	.ce_maxbusy	= 60 * 1000,    /* 60s */

	.st_regnum	= 3,
};
#else /* GIGA_SPI_NOR_Q128 */
static struct spi_nor_block_info flash_block_info[] = {
	{
		.blocksize	= 64 * 1024,
		.cmd_blockerase	= 0xD8,
		.be_maxbusy	= 1200  /* 1.2s */
	},

	{
		.blocksize	= 32 * 1024,
		.cmd_blockerase	= 0x52,
		.be_maxbusy	= 1000  /* 1s */
	},
};

static struct spi_nor_platform_data spi_nor_pdata = {
	.pagesize	= 256,
	.sectorsize	= 4 * 1024,
	.chipsize	= 16384 * 1024,

	.block_info	= flash_block_info,
	.num_block_info	= ARRAY_SIZE(flash_block_info),

	.addrsize	= 3,
	.pp_maxbusy	= 3,            /* 3ms */
	.se_maxbusy	= 400,          /* 400ms */
	.ce_maxbusy	= 80 * 1000,    /* 80s */

	.st_regnum	= 3,
};
#endif /* GIGA_SPI_NOR_Q64 */
#undef GIGA_SPI_NOR_Q64
#endif

/* SPI Devices */
static struct spi_board_info jz_spi_board_info[] = {
#ifdef CONFIG_JZ_SPI_NOR
	/* SPI NOR */
	[0] = {
		.modalias		= "jz_nor",
		.platform_data		= &spi_nor_pdata,
		.controller_data	= (void *)GPIO_PC(19), /* cs for spi gpio */
		.max_speed_hz		= 48000000, /* 48MHz */
#ifdef CONFIG_SPI_GPIO
		.bus_num		= 3,
#else
		.bus_num		= 0,
#endif
		.chip_select		= 0,
		.mode			= SPI_MODE_3,
	},
#endif
};

#ifdef CONFIG_KEYBOARD_SADCIN
static struct sadc_keys_info sadc_keys_info[] = {
	{
		.code    = KEY_MENU,
		.voltage = 55,
		.fuzz    = 20,
	},
	{
		.code    = KEY_UP,
		.voltage = 335,
		.fuzz    = 20,
	},
	{
		.code    = KEY_DOWN,
		.voltage = 675,
		.fuzz    = 30,
	},
	{
		.code    = KEY_LEFT,
		.voltage = 1060,
		.fuzz    = 30,
	},
	{
		.code    = KEY_ENTER,
		.voltage = 1420,
		.fuzz    = 40,
	},
	{
		.code    = KEY_RIGHT,
		.voltage = 1830,
		.fuzz    = 40,
	},
};

static struct sadc_keys_gpio sadc_keys_gpio = {
	.num = GPIO_PC(0),
	.enable_level = LOW_ENABLE,
	.debounce_interval = 30,	/* ms */
};

static struct sadc_keys_devdata sadc_keys_devdata = {
	.repeat = 1,
	.gpio = &sadc_keys_gpio,
	.info = sadc_keys_info,
	.info_num = ARRAY_SIZE(sadc_keys_info),
};

static struct  adc_cell_platform_data sadcin_pdata = {
	.name = "sadcin",
	.dev_name = "sadc_keys",
	.dev_data = &sadc_keys_devdata,
};
#endif

static void __init board_cpm_setup(void)
{
	/* Stop unused module clocks here.
	 * We have started all module clocks at arch/mips/jz4750l/setup.c.
	 */
}

static void __init board_gpio_setup(void)
{
	/*
	 * Initialize SDRAM pins
	 */
}

void __init board_devices_init(void)
{
#ifdef CONFIG_JZ_MSC0
	jz_add_msc_devices(0, &spwam_sd_data);
#endif

#ifdef CONFIG_JZ_MSC1
	jz_add_msc_devices(1, &spwam_tf_data);
#endif

#if defined(CONFIG_SPI_JZ47XX) || defined(CONFIG_SPI_GPIO)
	jz_add_spi_devices(jz_spi_board_info, ARRAY_SIZE(jz_spi_board_info));
#endif

#ifdef CONFIG_KEYBOARD_SADCIN
	jz_adc_pdata.cell_pdata[0] = &sadcin_pdata;
#endif
#ifdef CONFIG_MFD_JZ4750L_SADC
	jz_device_register(&jz_adc_device, &jz_adc_pdata);
#endif
}

void __init jz_board_setup(void)
{
	printk("JZ4750L SPWAM board setup\n");

	board_cpm_setup();
	board_gpio_setup();

	jz_timer_callback = spwam_timer_callback;
}
