/*
 * linux/arch/mips/jz4750d/board-cetus.c
 *
 * JZ4750D CETUS board setup routines.
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
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/platform_device.h>

#include <asm/jzsoc.h>
#include <linux/spi/spi.h>
#include <linux/gwtc9xxxb_ts.h>
#include <media/jz_cim.h> 

void __init board_msc_init(void);

extern int jz_add_msc_devices(unsigned int controller, struct jz_mmc_platform_data *plat);
extern int __init jz_add_spi_devices(unsigned int host_id, struct spi_board_info *board_info,int board_num);

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

static void cetus_timer_callback(void)
{
	static unsigned long count = 0;

	if ((++count) % 50 == 0) {
		dancing();
		count = 0;
	}
}

static void cetus_sd_gpio_init(struct device *dev)
{
	__gpio_as_msc0_4bit();
	__gpio_as_output(GPIO_SD0_VCC_EN_N);
	__gpio_as_input(GPIO_SD0_CD_N);
}

static void cetus_sd_power_on(struct device *dev)
{
	__gpio_clear_pin(GPIO_SD0_VCC_EN_N);
}

static void cetus_sd_power_off(struct device *dev)
{
	__gpio_set_pin(GPIO_SD0_VCC_EN_N);
}

static void cetus_sd_cpm_start(struct device *dev)
{
	__cpm_start_msc(0);
}

static unsigned int cetus_sd_status(struct device *dev)
{
#if defined(CONFIG_JZ_SYSTEM_AT_CARD)
	return 1;
#endif
	unsigned int status;

	status = (unsigned int) __gpio_get_pin(GPIO_SD0_CD_N);
	return (!status);
}

#if 0
static void cetus_sd_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_high_level(MSC0_HOTPLUG_PIN); /* wait remove */
	else
		__gpio_as_irq_low_level(MSC0_HOTPLUG_PIN); /* wait insert */
}
#else
static void cetus_sd_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_rise_edge(MSC0_HOTPLUG_PIN);
	else
		__gpio_as_irq_fall_edge(MSC0_HOTPLUG_PIN);
}
#endif

static unsigned int cetus_sd_get_wp(struct device *dev)
{
#if defined(CONFIG_JZ_SYSTEM_AT_CARD)
	return 0;
#endif
	unsigned int status;

	status = (unsigned int) __gpio_get_pin(GPIO_SD0_WP);
	return (status);
}

struct jz_mmc_platform_data cetus_sd_data = {
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
	.init           = cetus_sd_gpio_init,
	.power_on       = cetus_sd_power_on,
	.power_off      = cetus_sd_power_off,
	.cpm_start	= cetus_sd_cpm_start,
	.status		= cetus_sd_status,
	.plug_change	= cetus_sd_plug_change,
	.write_protect  = cetus_sd_get_wp,
	.max_bus_width  = MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED | MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_JZ_MSC0_BUS_1
	.bus_width      = 1,
#elif defined  CONFIG_JZ_MSC0_BUS_4
	.bus_width      = 4,
#else
	.bus_width      = 8,
#endif
};

static void cetus_tf_gpio_init(struct device *dev)
{
	__gpio_as_msc1_4bit();
	__gpio_as_output(GPIO_SD1_VCC_EN_N);
	__gpio_as_input(GPIO_SD1_CD_N);
}

static void cetus_tf_power_on(struct device *dev)
{
	__msc1_enable_power();
}

static void cetus_tf_power_off(struct device *dev)
{
	__msc1_disable_power();
}

static void cetus_tf_cpm_start(struct device *dev)
{
	__cpm_start_msc(1);
}

static unsigned int cetus_tf_status(struct device *dev)
{
	unsigned int status;

	status = (unsigned int) __gpio_get_pin(GPIO_SD1_CD_N);
	return (!status);
}

#if 0
static void cetus_tf_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_low_level(MSC1_HOTPLUG_PIN);
	else
		__gpio_as_irq_high_level(MSC1_HOTPLUG_PIN);
}
#else
static void cetus_tf_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_fall_edge(MSC1_HOTPLUG_PIN);
	else
		__gpio_as_irq_rise_edge(MSC1_HOTPLUG_PIN);
}
#endif

struct jz_mmc_platform_data cetus_tf_data = {
#ifndef CONFIG_JZ_MSC1_SDIO_SUPPORT
	.support_sdio   = 0,
#else
	.support_sdio   = 1,
#endif
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.status_irq	= MSC1_HOTPLUG_IRQ,
	.detect_pin     = GPIO_SD1_CD_N,
	.init           = cetus_tf_gpio_init,
	.power_on       = cetus_tf_power_on,
	.power_off      = cetus_tf_power_off,
	.cpm_start	= cetus_tf_cpm_start,
	.status		= cetus_tf_status,
	.plug_change	= cetus_tf_plug_change,
	.max_bus_width  = MMC_CAP_SD_HIGHSPEED | MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_JZ_MSC1_BUS_1
	.bus_width      = 1,
#else
	.bus_width      = 4,
#endif
};

#ifdef CONFIG_VIDEO_OV5640               
struct jz_sensor_platform_data ov5640_pdata = {
	.cim_id = 0,
	.rst_pin = GPIO_OV5640_RST,
	.pd_pin = GPIO_OV5640_PWDN,
};
#endif

#ifdef CONFIG_VIDEO_GC2035
struct jz_sensor_platform_data gc2035_pdata = {
	.cim_id = 0,
	.rst_pin = GPIO_GC2035_RST,
	.pd_pin = GPIO_GC2035_PWDN,
};
#endif

static struct gwtc9xxxb_ts_platform_data gwtc9xxxb_ts_pdata = {                                           
	.intr = GPIO_GWTC9XXXB_INT,                                                                       
};                            
#if defined(CONFIG_I2C_GPIO)
static struct i2c_board_info pyxis_gpio_i2c_devs[] __initdata = {
#ifdef CONFIG_VIDEO_OV5640                                                                                
        {                                                                                           
                 I2C_BOARD_INFO("ov5640", 0x3c),                                                    
                 .platform_data  = &ov5640_pdata,                                              
        },
#endif
#ifdef CONFIG_VIDEO_GC2035
	{
		I2C_BOARD_INFO("gc2035", 0x3c),
		.platform_data  = &gc2035_pdata,
	}
#endif
#ifdef CONFIG_TOUCHSCREEN_GWTC9XXXB	
	{                                                                                                 
                I2C_BOARD_INFO(GWTC9XXXB_NAME, 0x05),                                                     
                .irq = GPIO_GWTC9XXXB_IRQ,                                                                
                .platform_data = &gwtc9xxxb_ts_pdata,                                                     
        },                                                                                        
#endif                                                                                               
};
static struct i2c_gpio_platform_data pyxis_i2c_gpio_data = {
        .sda_pin        = (32*4+12),
        .scl_pin        = (32*4+13),
};
static struct platform_device pyxis_i2c_gpio_device = {
        .name   = "i2c-gpio",
        .id     = 0,
        .dev    = {
                .platform_data = &pyxis_i2c_gpio_data,
        },
};
static struct platform_device *pyxis_platform_devices[] __initdata = {
        &pyxis_i2c_gpio_device,
};
#endif
/*i2c devices*/                                                                                           
static struct i2c_board_info cetus_i2c0_devs[] __initdata = {                                             
        {                                                                                                 
                I2C_BOARD_INFO("ov3640", 0x3c),                                                           
        },                                                                                                
};                                                                                                        
void __init board_i2c_init(void) {                                                                        
	//i2c_register_board_info(0, cetus_i2c0_devs, ARRAY_SIZE(cetus_i2c0_devs));                         
#if defined(CONFIG_I2C_GPIO)
	i2c_register_board_info(0, pyxis_gpio_i2c_devs, ARRAY_SIZE(pyxis_gpio_i2c_devs));
	platform_add_devices(pyxis_platform_devices, ARRAY_SIZE(pyxis_platform_devices));
#endif
}             
/* SPI devices */
struct spi_board_info cetus_spi0_board_info[]  = {
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
void __init board_spi_init(void){
	jz_add_spi_devices(0,cetus_spi0_board_info,ARRAY_SIZE(cetus_spi0_board_info));
}
static void __init board_cpm_setup(void)
{
	/* Stop unused module clocks here.
	 * We have started all module clocks at arch/mips/jz4750d/setup.c.
	 */
}

static void __init board_gpio_setup(void)
{
	/*
	 * Initialize SDRAM pins
	 */
}

void __init board_msc_init(void)
{
#ifdef CONFIG_JZ_MSC0
	jz_add_msc_devices(0, &cetus_sd_data);
#endif

#ifdef CONFIG_JZ_MSC1
	jz_add_msc_devices(1, &cetus_tf_data);
#endif
}

void __init jz_board_setup(void)
{
	printk("JZ4750D CETUS board setup\n");

	board_cpm_setup();
	board_gpio_setup();

	jz_timer_callback = cetus_timer_callback;
}
