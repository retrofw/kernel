/*
 * linux/arch/mips/jz4760/board-cygnus.c
 *
 * JZ4760 Cygnus board setup routines.
 *
 * Copyright (c) 2006-2010  Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/console.h>
#include <linux/delay.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <asm/jzsoc.h>
#include <linux/i2c.h>
#include <linux/mmc/host.h>
#include <linux/ft5x0x_ts.h>
#include <linux/spi/spi.h>
extern void (*jz_timer_callback)(void);
extern int __init jz_add_msc_devices(unsigned int controller, struct jz_mmc_platform_data *plat);

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define dprintk(x...)	printk(x)
#else
#define dprintk(x...)
#endif

/*
 *  config_gpio_on_sleep config all gpio pins when sleep.
 */
struct gpio_sleep_status {
	unsigned int input;
	unsigned int input_pull;
	unsigned int input_no_pull;
	unsigned int output;
	unsigned int output_high;
	unsigned int output_low;
	unsigned int no_operation;
};

void config_gpio_on_sleep(void)
{
	int i = 0;
	struct gpio_sleep_status gpio_sleep_st[] = {
		/* GPA */
		{
			.input_pull = BIT31 | BIT27 |
			BITS_H2L(29,28) | /* NC pin */
			BITS_H2L(19,0), /* NAND: SD0~SD15 */

			.output_high = BIT21 | BIT22 | BIT23 | BIT24 | BIT25 | BIT26, /* NAND: CS1~CS6 */
			.output_low = 0x0,
			.no_operation = 0x0,
		},

		/* GPB */
		{
			.input_pull = BIT30 | BIT27 | BIT26 | BIT25 | BITS_H2L(24,22) | BIT20 |
			BITS_H2L(19,0), /* SA0~SA5 */

			.output_high = BIT29,
			.output_low = BIT31 | BIT28 | BIT21,
			.no_operation = 0x0,
		},

		/* GPC */
		{
			.input_pull = BITS_H2L(31,28),
			.output_high = 0x0,
			.output_low = BITS_H2L(27,0),
			.no_operation = 0x0,
		},

		/* GPD */
		{
			.input_pull = BITS_H2L(29,26) | BITS_H2L(19,14) | BITS_H2L(13,12) || BITS_H2L(10,0) | BIT11,  // bit11 temporary input_pull
			.output_high = 0x0,
			.output_low =  BITS_H2L(25,20), // | BIT11,
			.no_operation = 0x0,
		},

		/* GPE */
		{
			.input_pull = BITS_H2L(18,11) | BITS_H2L(8,3) | BIT0,
			.output_high = BIT9,
			.output_low = BITS_H2L(29,20) | BIT10 | BIT1 | BIT2,
			.no_operation = 0x0,
		},

		/* GPF */
		{
			.input_pull = BIT11 | BITS_H2L(8,4) | BITS_H2L(2,0),
			.output_high = BIT9,
			.output_low = BIT3,
			.no_operation = 0x0,
		},
	};

	for (i = 0; i < 6; i++) {
		gpio_sleep_st[i].input_pull &= ~gpio_sleep_st[i].no_operation;
		gpio_sleep_st[i].output_high &= ~gpio_sleep_st[i].no_operation;
		gpio_sleep_st[i].output_low &= ~gpio_sleep_st[i].no_operation;
		gpio_sleep_st[i].input_no_pull = 0xffffffff &
			~(gpio_sleep_st[i].input_pull |
			  gpio_sleep_st[i].output_high |
			  gpio_sleep_st[i].output_low) &
			~gpio_sleep_st[i].no_operation;

		gpio_sleep_st[i].input = gpio_sleep_st[i].input_pull | gpio_sleep_st[i].input_no_pull;
		gpio_sleep_st[i].output = gpio_sleep_st[i].output_high | gpio_sleep_st[i].output_low;

		/* all as gpio, except interrupt pins(see @wakeup_key_setup()) */
		REG_GPIO_PXFUNC(i) =  0xffffffff;
		REG_GPIO_PXSELC(i) =  0xffffffff;
		/* input */
		REG_GPIO_PXDIRC(i) =  gpio_sleep_st[i].input;
		/* pull */
		REG_GPIO_PXPEC(i) = gpio_sleep_st[i].input_pull;
		/* no_pull */
		REG_GPIO_PXPES(i) =  gpio_sleep_st[i].input_no_pull;

		/* output */
		REG_GPIO_PXDIRS(i) =  gpio_sleep_st[i].output;
		REG_GPIO_PXPES(i)  = gpio_sleep_st[i].output; /* disable pull */
		/* high */
		REG_GPIO_PXDATS(i) = gpio_sleep_st[i].output_high;
		/* low */
		REG_GPIO_PXDATC(i) = gpio_sleep_st[i].output_low;
	}
}

struct wakeup_key_s {
	int gpio;       /* gpio pin number */
	int active_low; /* the key interrupt pin is low voltage
                           or fall edge acitve */
};

/* add wakeup keys here */
static struct wakeup_key_s wakeup_key[] = {
	{
		.gpio = GPIO_POWER_ON,
		.active_low = ACTIVE_LOW_WAKE_UP,
	},
	{
		.gpio = MSC1_HOTPLUG_PIN,
		.active_low = ACTIVE_LOW_MSC1_CD,
	},
};

static void wakeup_key_setup(void)
{
	int i;
	int num = sizeof(wakeup_key) / sizeof(wakeup_key[0]);

	for(i = 0; i < num; i++) {
		if(wakeup_key[i].active_low)
			__gpio_as_irq_fall_edge(wakeup_key[i].gpio);
		else
			__gpio_as_irq_rise_edge(wakeup_key[i].gpio);

		__gpio_ack_irq(wakeup_key[i].gpio);
		__gpio_unmask_irq(wakeup_key[i].gpio);
		__intc_unmask_irq(IRQ_GPIO0 - (wakeup_key[i].gpio/32));  /* unmask IRQ_GPIOn */
	}
}


/* NOTES:
 * 1: Pins that are floated (NC) should be set as input and pull-enable.
 * 2: Pins that are pull-up or pull-down by outside should be set as input
 *    and pull-disable.
 * 3: Pins that are connected to a chip except sdram and nand flash
 *    should be set as input and pull-disable, too.
 */
void jz_board_do_sleep(unsigned long *ptr)
{
	unsigned char i;

        /* Print messages of GPIO registers for debug */
	for(i=0;i<GPIO_PORT_NUM;i++) {
		dprintk("run dat:%x pin:%x fun:%x sel:%x dir:%x pull:%x msk:%x trg:%x\n",        \
			REG_GPIO_PXDAT(i),REG_GPIO_PXPIN(i),REG_GPIO_PXFUN(i),REG_GPIO_PXSEL(i), \
			REG_GPIO_PXDIR(i),REG_GPIO_PXPE(i),REG_GPIO_PXIM(i),REG_GPIO_PXTRG(i));
	}

        /* Save GPIO registers */
	for(i = 1; i < GPIO_PORT_NUM; i++) {
		*ptr++ = REG_GPIO_PXFUN(i);
		*ptr++ = REG_GPIO_PXSEL(i);
		*ptr++ = REG_GPIO_PXDIR(i);
		*ptr++ = REG_GPIO_PXPE(i);
		*ptr++ = REG_GPIO_PXIM(i);
		*ptr++ = REG_GPIO_PXDAT(i);
		*ptr++ = REG_GPIO_PXTRG(i);
	}

        /*
         * Set all pins to pull-disable, and set all pins as input except
         * sdram and the pins which can be used as CS1_N to CS4_N for chip select.
         */
	config_gpio_on_sleep();

        /*
	 * Set proper status for GPC21 to GPC24 which can be used as CS1_N to CS4_N.
	 * Keep the pins' function used for chip select(CS) here according to your
         * system to avoid chip select crashing with sdram when resuming from sleep mode.
         */

	/*
         * If you must set some GPIOs as output to high level or low level,
         * you can set them here, using:
         * __gpio_as_output(n);
         * __gpio_set_pin(n); or  __gpio_clear_pin(n);
	 */

#if 0
        /* Keep uart function for printing debug message */
	__gpio_as_uart0();
	__gpio_as_uart1();
	__gpio_as_uart2();
	__gpio_as_uart3();

        /* Print messages of GPIO registers for debug */
	for(i=0;i<GPIO_PORT_NUM;i++) {
		printk("GP%d: data:0x%08x pin:0x%08x fun:0x%08x sel:0x%08x dir:0x%08x pull:0x%08x msk:0x%08x trg:0x%08x\n",
		       i, REG_GPIO_PXDAT(i),REG_GPIO_PXPIN(i),REG_GPIO_PXFUN(i),REG_GPIO_PXSEL(i),
		       REG_GPIO_PXDIR(i),REG_GPIO_PXPE(i),REG_GPIO_PXIM(i),REG_GPIO_PXTRG(i));
	}
#endif
	wakeup_key_setup();
}

void jz_board_do_resume(unsigned long *ptr)
{
	unsigned char i;

	/* Restore GPIO registers */
	for(i = 1; i < GPIO_PORT_NUM; i++) {
		 REG_GPIO_PXFUNS(i) = *ptr;
		 REG_GPIO_PXFUNC(i) = ~(*ptr++);

		 REG_GPIO_PXSELS(i) = *ptr;
		 REG_GPIO_PXSELC(i) = ~(*ptr++);

		 REG_GPIO_PXDIRS(i) = *ptr;
		 REG_GPIO_PXDIRC(i) = ~(*ptr++);

		 REG_GPIO_PXPES(i) = *ptr;
		 REG_GPIO_PXPEC(i) = ~(*ptr++);

		 REG_GPIO_PXIMS(i)=*ptr;
		 REG_GPIO_PXIMC(i)=~(*ptr++);

		 REG_GPIO_PXDATS(i)=*ptr;
		 REG_GPIO_PXDATC(i)=~(*ptr++);

		 REG_GPIO_PXTRGS(i)=*ptr;
		 REG_GPIO_PXTRGC(i)=~(*ptr++);
	}

        /* Print messages of GPIO registers for debug */
	for(i=0;i<GPIO_PORT_NUM;i++) {
		dprintk("resume dat:%x pin:%x fun:%x sel:%x dir:%x pull:%x msk:%x trg:%x\n",     \
			REG_GPIO_PXDAT(i),REG_GPIO_PXPIN(i),REG_GPIO_PXFUN(i),REG_GPIO_PXSEL(i), \
			REG_GPIO_PXDIR(i),REG_GPIO_PXPE(i),REG_GPIO_PXIM(i),REG_GPIO_PXTRG(i));
	}
}

static void htb80_tf_gpio_init(struct device *dev)
{
	__gpio_as_msc1_4bit();
	__gpio_as_output(GPIO_SD1_VCC_EN_N);
	__gpio_set_pin(GPIO_SD1_VCC_EN_N); /* poweroff */
	__gpio_as_input(GPIO_SD1_CD_N);
}

static void htb80_tf_power_on(struct device *dev)
{
	__gpio_clear_pin(GPIO_SD1_VCC_EN_N);
}

static void htb80_tf_power_off(struct device *dev)
{
	__gpio_set_pin(GPIO_SD1_VCC_EN_N);
}

static void htb80_tf_cpm_start(struct device *dev)
{
	cpm_start_clock(CGM_MSC1);
}

static unsigned int htb80_tf_status(struct device *dev)
{
	unsigned int status = 0;
	status = (unsigned int) __gpio_get_pin(GPIO_SD1_CD_N);
#if ACTIVE_LOW_MSC1_CD
	return !status;
#else
	return status;
#endif
}

static void htb80_tf_plug_change(int state)
{
	if(state == CARD_INSERTED) /* wait for remove */
#if ACTIVE_LOW_MSC1_CD
		__gpio_as_irq_high_level(MSC1_HOTPLUG_PIN);
#else
		__gpio_as_irq_low_level(MSC1_HOTPLUG_PIN);
#endif
	else		      /* wait for insert */
#if ACTIVE_LOW_MSC1_CD
		__gpio_as_irq_low_level(MSC1_HOTPLUG_PIN);
#else
		__gpio_as_irq_high_level(MSC1_HOTPLUG_PIN);
#endif
}

struct jz_mmc_platform_data htb80_tf_data = {
#ifndef CONFIG_JZ_MSC1_SDIO_SUPPORT
	.support_sdio   = 0,
#else
	.support_sdio   = 1,
#endif
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.status_irq	= IRQ_GPIO_0 + GPIO_SD1_CD_N,
	.detect_pin     = GPIO_SD1_CD_N,
	.init           = htb80_tf_gpio_init,
	.power_on       = htb80_tf_power_on,
	.power_off      = htb80_tf_power_off,
	.cpm_start	= htb80_tf_cpm_start,
	.status		= htb80_tf_status,
	.plug_change	= htb80_tf_plug_change,
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

static void htb80_wifi_gpio_init(struct device *dev)
{
	__gpio_as_msc2_4bit();
	__gpio_as_output(GPIO_SD2_VCC_EN_N);
	__gpio_set_pin(GPIO_SD2_VCC_EN_N);
	__gpio_as_input(GPIO_SD2_CD_N);

	__gpio_as_output(GPIO_WIFI_PDn);
	__gpio_clear_pin(GPIO_WIFI_PDn); /* default to poweroff the wifi */
}

static void htb80_wifi_power_on(struct device *dev)
{
	__gpio_clear_pin(GPIO_SD2_VCC_EN_N);
	msleep(10);
	__gpio_set_pin(GPIO_WIFI_PDn);
}

static void htb80_wifi_power_off(struct device *dev)
{
	__gpio_clear_pin(GPIO_WIFI_PDn);
	msleep(10);
	__gpio_set_pin(GPIO_SD2_VCC_EN_N);
}

static void htb80_wifi_cpm_start(struct device *dev)
{
	cpm_start_clock(CGM_MSC2);
}

static unsigned int htb80_wifi_status(struct device *dev)
{
	unsigned int status = 0;
	status = (unsigned int) __gpio_get_pin(GPIO_SD2_CD_N);
#if ACTIVE_LOW_MSC2_CD
	return !status;
#else
	return status;
#endif
}

static void htb80_wifi_plug_change(int state)
{
	if(state == CARD_INSERTED) /* wait for remove */
#if ACTIVE_LOW_MSC2_CD
		__gpio_as_irq_high_level(MSC2_HOTPLUG_PIN);
#else
	__gpio_as_irq_low_level(MSC2_HOTPLUG_PIN);
#endif
	else		      /* wait for insert */
#if ACTIVE_LOW_MSC2_CD
		__gpio_as_irq_low_level(MSC2_HOTPLUG_PIN);
#else
	__gpio_as_irq_high_level(MSC2_HOTPLUG_PIN);
#endif
}

struct jz_mmc_platform_data htb80_wifi_data = {
#ifndef CONFIG_JZ_MSC2_SDIO_SUPPORT
	.support_sdio   = 0,
#else
	.support_sdio   = 1,
#endif
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.status_irq	= IRQ_GPIO_0 + GPIO_SD2_CD_N,
	.detect_pin     = GPIO_SD2_CD_N,
	.init           = htb80_wifi_gpio_init,
	.power_on       = htb80_wifi_power_on,
	.power_off      = htb80_wifi_power_off,
	.cpm_start	= htb80_wifi_cpm_start,
	.status		= htb80_wifi_status,
	.plug_change	= htb80_wifi_plug_change,
	.max_bus_width  = MMC_CAP_SD_HIGHSPEED | MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_JZ_MSC2_BUS_1
	.bus_width      = 1,
#else
	.bus_width      = 4,
#endif
};

void __init board_msc_init(void)
{
	jz_add_msc_devices(1, &htb80_tf_data);
	jz_add_msc_devices(2, &htb80_wifi_data);
}

static void f4760_timer_callback(void)
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
	 * We have started all module clocks at arch/mips/jz4760/setup.c.
	 */
}

static void __init board_gpio_setup(void)
{
	/*
	 * Initialize SDRAM pins
	 */
}

static struct ft5x0x_ts_platform_data ft5x0x_ts_pdata = {
	        .intr = GPIO_TS_I2C_INT,
};
static struct i2c_board_info htb80_i2c0_devs[] __initdata = {
	{
		I2C_BOARD_INFO("cm3511", 0x30),
	},
	{
		I2C_BOARD_INFO("ov3640", 0x3c),
	},
	{
		I2C_BOARD_INFO("ov7690", 0x21),
	},
	{
	        I2C_BOARD_INFO(FT5X0X_NAME, 0x38),
	       .irq = GPIO_TS_I2C_IRQ,
	       .platform_data = &ft5x0x_ts_pdata,
	},
	{
	},
};
/* SPI devices */
struct spi_board_info jz4760_spi0_board_info[]  = {
	[0] = {
		.modalias       = "spidev0",
		.bus_num        = 0,
		.chip_select    = 0,
		.max_speed_hz   = 120000,
/*		.platform_data	= &spitest,*/
	},
	[1] = {
		.modalias       = "spitest",
		.bus_num        = 0,
		.chip_select    = 1,
		.max_speed_hz   = 2500000,
/*		.platform_data	= */
	},
};
struct spi_board_info jz4760_spi1_board_info[]  = {
	[0] = {
		.modalias       = "spidev1",
		.bus_num        = 1,
		.chip_select    = 0,
		.max_speed_hz   = 12000000,
	},
};

void __init board_i2c_init(void) {
	i2c_register_board_info(0, htb80_i2c0_devs, ARRAY_SIZE(htb80_i2c0_devs));
}
void __init board_spi_init(void){
	spi_register_board_info(jz4760_spi0_board_info,ARRAY_SIZE(jz4760_spi0_board_info));
	spi_register_board_info(jz4760_spi1_board_info,ARRAY_SIZE(jz4760_spi1_board_info));
}
void __init jz_board_setup(void)
{
	printk("JZ4760 HTB80 board setup\n");
	board_cpm_setup();
	board_gpio_setup();

	jz_timer_callback = f4760_timer_callback;
}

/**
 * Called by arch/mips/kernel/proc.c when 'cat /proc/cpuinfo'.
  */
const char *get_board_type(void)
{
	return "HTB80";
}
