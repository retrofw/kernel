/*
 * linux/arch/mips/jz4750l/board-taurus.c
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

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <asm/jzsoc.h>

extern int __init jz_add_msc_devices(unsigned int controller, struct jz_mmc_platform_data *plat);
extern int __init jz_add_spi_devices(struct spi_board_info *board_info, int board_array);

extern void (*jz_timer_callback)(void);

static void dancing(void)
{

}

static void taurus_timer_callback(void)
{
	static unsigned long count = 0;

	if ((++count) % 50 == 0) {
		dancing();
		count = 0;
	}
}

static void taurus_sd_gpio_init(struct device *dev)
{
	__gpio_as_msc0_4bit();
	__gpio_as_input(GPIO_SD0_CD_N);
	__gpio_as_output(GPIO_SD0_VCC_EN_N);
}

static void taurus_sd_power_on(struct device *dev)
{
	__msc0_enable_power();
}

static void taurus_sd_power_off(struct device *dev)
{
	__msc0_disable_power();
}

static void taurus_sd_cpm_start(struct device *dev)
{
	__cpm_start_msc(0);
}

static unsigned int taurus_sd_status(struct device *dev)
{
#if defined(CONFIG_JZ_SYSTEM_AT_CARD)
	return 1;
#endif
	unsigned int status;

	status = (unsigned int)__gpio_get_pin(GPIO_SD0_CD_N);
	return (!status);
}

#if 0
static void taurus_sd_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_high_level(MSC0_HOTPLUG_PIN); /* wait remove */
	else
		__gpio_as_irq_low_level(MSC0_HOTPLUG_PIN); /* wait insert */
}
#else
static void taurus_sd_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_rise_edge(MSC0_HOTPLUG_PIN);
		//__gpio_as_irq_fall_edge(MSC0_HOTPLUG_PIN);
	else
		//__gpio_as_irq_rise_edge(MSC0_HOTPLUG_PIN);
		__gpio_as_irq_fall_edge(MSC0_HOTPLUG_PIN);
}
#endif

static unsigned int taurus_sd_get_wp(struct device *dev)
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

struct jz_mmc_platform_data taurus_sd_data = {
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
	.init           = taurus_sd_gpio_init,
	.power_on       = taurus_sd_power_on,
	.power_off      = taurus_sd_power_off,
	.cpm_start	= taurus_sd_cpm_start,
	.status		= taurus_sd_status,
	.plug_change	= taurus_sd_plug_change,
	.write_protect  = taurus_sd_get_wp,
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

static void taurus_tf_gpio_init(struct device *dev)
{
	__gpio_as_msc1_4bit();
	__gpio_as_input(GPIO_SD1_CD_N);
}

static void taurus_tf_power_on(struct device *dev)
{
	//__msc1_enable_power();
}

static void taurus_tf_power_off(struct device *dev)
{
	//__msc1_disable_power();
}

static void taurus_tf_cpm_start(struct device *dev)
{
	__cpm_start_msc(1);
}

static unsigned int taurus_tf_status(struct device *dev)
{
	unsigned int status;

	status = (unsigned int) __gpio_get_pin(GPIO_SD1_CD_N);
	return (!status);
}

#if 0
static void taurus_tf_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_low_level(MSC1_HOTPLUG_PIN);
	else
		__gpio_as_irq_high_level(MSC1_HOTPLUG_PIN);
}
#else
static void taurus_tf_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_fall_edge(MSC1_HOTPLUG_PIN);
	else
		__gpio_as_irq_rise_edge(MSC1_HOTPLUG_PIN);
}
#endif

struct jz_mmc_platform_data taurus_tf_data = {
#ifndef CONFIG_JZ_MSC1_SDIO_SUPPORT
	.support_sdio   = 0,
#else
	.support_sdio   = 1,
#endif
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.status_irq	= MSC1_HOTPLUG_IRQ,
	.detect_pin     = GPIO_SD1_CD_N,
	.init           = taurus_tf_gpio_init,
	.power_on       = taurus_tf_power_on,
	.power_off      = taurus_tf_power_off,
	.cpm_start	= taurus_tf_cpm_start,
	.status		= taurus_tf_status,
	.plug_change	= taurus_tf_plug_change,
	.max_bus_width  = MMC_CAP_SD_HIGHSPEED | MMC_CAP_MMC_HIGHSPEED
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
	jz_add_msc_devices(0, &taurus_sd_data);
#endif

#ifdef CONFIG_JZ_MSC1
	jz_add_msc_devices(1, &taurus_tf_data);
#endif

#if defined(CONFIG_SPI_JZ47XX) || defined(CONFIG_SPI_GPIO)
	jz_add_spi_devices(jz_spi_board_info, ARRAY_SIZE(jz_spi_board_info));
#endif
}

void __init jz_board_setup(void)
{
	printk("JZ4750L TAURUS board setup\n");

	board_cpm_setup();
	board_gpio_setup();

	jz_timer_callback = taurus_timer_callback;
}
