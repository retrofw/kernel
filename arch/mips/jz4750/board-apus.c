/*
 * linux/arch/mips/jz4750/board-apus.c
 *
 * JZ4750 APUS board setup routines.
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

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <asm/jzsoc.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>

void __init board_msc_init(void);

extern int jz_add_msc_devices(unsigned int controller, struct jz_mmc_platform_data *plat);
extern int __init jz_add_spi_devices(unsigned int host_id, struct spi_board_info *board_info,int board_num);

extern void (*jz_timer_callback)(void);

static void dancing(void)
{
}

static void apus_timer_callback(void)
{
	static unsigned long count = 0;

	if ((++count) % 50 == 0) {
		dancing();
		count = 0;
	}
}

static void apus_sd_gpio_init(struct device *dev)
{
	__gpio_as_msc0_8bit();
	__gpio_as_output(GPIO_SD0_VCC_EN_N);
	__gpio_as_input(GPIO_SD0_CD_N);
}

static void apus_sd_power_on(struct device *dev)
{
	__gpio_clear_pin(GPIO_SD0_VCC_EN_N);
}

static void apus_sd_power_off(struct device *dev)
{
	__gpio_set_pin(GPIO_SD0_VCC_EN_N);
}

static void apus_sd_cpm_start(struct device *dev)
{
	__cpm_start_msc(0);
}

static unsigned int apus_sd_status(struct device *dev)
{
#if defined(CONFIG_JZ_SYSTEM_AT_CARD)
	return 1;
#endif
	unsigned int status;

	status = (unsigned int) __gpio_get_pin(GPIO_SD0_CD_N);
	return (!status);
}

#if 0
static void apus_sd_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_high_level(MSC0_HOTPLUG_PIN); /* wait remove */
	else
		__gpio_as_irq_low_level(MSC0_HOTPLUG_PIN); /* wait insert */
}
#else
static void apus_sd_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_rise_edge(MSC0_HOTPLUG_PIN);
	else
		__gpio_as_irq_fall_edge(MSC0_HOTPLUG_PIN);
}
#endif

static unsigned int apus_sd_get_wp(struct device *dev)
{
#if defined(CONFIG_JZ_SYSTEM_AT_CARD)
	return 0;
#endif

	unsigned int status;

	status = (unsigned int) __gpio_get_pin(GPIO_SD0_WP);
	return (status);
}

struct jz_mmc_platform_data apus_sd_data = {
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
	.init           = apus_sd_gpio_init,
	.power_on       = apus_sd_power_on,
	.power_off      = apus_sd_power_off,
	.cpm_start	= apus_sd_cpm_start,
	.status		= apus_sd_status,
	.plug_change	= apus_sd_plug_change,
	.write_protect  = apus_sd_get_wp,
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

static void apus_tf_gpio_init(struct device *dev)
{
	__gpio_as_msc1_4bit();
	__gpio_as_output(GPIO_SD1_VCC_EN_N);
	__gpio_as_input(GPIO_SD1_CD_N);
}

static void apus_tf_power_on(struct device *dev)
{
	__msc1_enable_power();
}

static void apus_tf_power_off(struct device *dev)
{
	__msc1_disable_power();
}

static void apus_tf_cpm_start(struct device *dev)
{
	__cpm_start_msc(1);
}

static unsigned int apus_tf_status(struct device *dev)
{
	unsigned int status;

	status = (unsigned int) __gpio_get_pin(GPIO_SD1_CD_N);
	return (status);
}

#if 0
static void apus_tf_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_low_level(MSC1_HOTPLUG_PIN);
	else
		__gpio_as_irq_high_level(MSC1_HOTPLUG_PIN);
}
#else
static void apus_tf_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_fall_edge(MSC1_HOTPLUG_PIN);
	else
		__gpio_as_irq_rise_edge(MSC1_HOTPLUG_PIN);
}
#endif

struct jz_mmc_platform_data apus_tf_data = {
#ifndef CONFIG_JZ_MSC1_SDIO_SUPPORT
	.support_sdio   = 0,
#else
	.support_sdio   = 1,
#endif
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.status_irq	= MSC1_HOTPLUG_IRQ,
	.detect_pin     = GPIO_SD1_CD_N,
	.init           = apus_tf_gpio_init,
	.power_on       = apus_tf_power_on,
	.power_off      = apus_tf_power_off,
	.cpm_start	= apus_tf_cpm_start,
	.status		= apus_tf_status,
	.plug_change	= apus_tf_plug_change,
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

static struct i2c_board_info apus_i2c0_devs[] __initdata = {
	{
		I2C_BOARD_INFO("cm3511", 0x30),
	},
	{
		I2C_BOARD_INFO("ov3640", 0x3c),
	},
	{
		I2C_BOARD_INFO("tvp5150", 0x5d),
	},
	{
		I2C_BOARD_INFO("ov7690", 0x21),
	},
	{
	},
};

void __init board_i2c_init(void) {
	i2c_register_board_info(0, apus_i2c0_devs, ARRAY_SIZE(apus_i2c0_devs));
}

/* SPI devices */
struct spi_board_info apus_spi0_board_info[]  = {
	[0] = {
		.modalias       = "spidev0",
		.bus_num        = SPI0_BUS,
		.chip_select    = SPI_CHIPSELECT_NUM_A,
		.max_speed_hz   = 120000,
	},
	[1] = {
		.modalias       = "spitest",
		.bus_num        = SPI0_BUS,
		.chip_select    = SPI_CHIPSELECT_NUM_B,
		.max_speed_hz   = 2500000,
	},
};
struct spi_board_info apus_spi1_board_info[]  = {
	[0] = {
		.modalias       = "spidev1",
		.bus_num        = SPI1_BUS,
		.chip_select    = SPI_CHIPSELECT_NUM_C,
		.max_speed_hz   = 12000000,
	},
};
void __init board_spi_init(void){
	jz_add_spi_devices(0,apus_spi0_board_info,ARRAY_SIZE(apus_spi0_board_info));
	jz_add_spi_devices(1,apus_spi1_board_info,ARRAY_SIZE(apus_spi1_board_info));

}
static void __init board_cpm_setup(void)
{
	/* Stop unused module clocks here.
	 * We have started all module clocks at arch/mips/jz4750/setup.c.
	 */
}

static void __init board_gpio_setup(void)
{
	__gpio_as_pcm();
}

void __init board_msc_init(void)
{
#ifdef CONFIG_JZ_MSC0
	jz_add_msc_devices(0, &apus_sd_data);
#endif

#ifdef CONFIG_JZ_MSC1
	jz_add_msc_devices(1, &apus_tf_data);
#endif
}

void __init jz_board_setup(void)
{
	printk("JZ4750 APUS board setup\n");

	board_cpm_setup();
	board_gpio_setup();

	jz_timer_callback = apus_timer_callback;
}
