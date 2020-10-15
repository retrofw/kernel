/*
 * linux/arch/mips/jz4760/board-f4760.c
 *
 * JZ4770 Pisces board setup routines.
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
#include <linux/i2c-gpio.h>
#include <linux/platform_device.h>


#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <asm/jzsoc.h>
#include <linux/mmc/host.h>
#include <linux/act8600_power.h>
#include <linux/ft5x0x_ts.h>
#include <linux/jz_hdmi.h>


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
#ifndef CONFIG_JZ_SYSTEM_AT_CARD
			.no_operation = 0x0,
#else
			.no_operation = BITS_H2L(23, 18),
#endif
		},

		/* GPB */
		{
			.input_pull = BIT30 | BIT27 | BIT26 | BIT25 | BITS_H2L(24,22) | BIT20 |
			BITS_H2L(19,0), /* SA0~SA5 */

			.output_high = BIT29,
			.output_low = BIT31 | BIT28 | BIT21,
#ifndef CONFIG_JZ_SYSTEM_AT_CARD
			.no_operation = 0x0,
#else
			.no_operation = BIT0,
#endif
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
		REG_GPIO_PXINTC(i) =  (0xffffffff & ~gpio_sleep_st[i].no_operation);
		REG_GPIO_PXMASKS(i) =  (0xffffffff & ~gpio_sleep_st[i].no_operation);
		/* input */
		REG_GPIO_PXPAT1S(i) =  gpio_sleep_st[i].input;
		/* pull */
		REG_GPIO_PXPENC(i) = gpio_sleep_st[i].input_pull;
		/* no_pull */
		REG_GPIO_PXPENS(i) =  gpio_sleep_st[i].input_no_pull;

		/* output */
		REG_GPIO_PXPAT1C(i) =  gpio_sleep_st[i].output;
		REG_GPIO_PXPENS(i)  = gpio_sleep_st[i].output; /* disable pull */
		/* high */
		REG_GPIO_PXPAT0S(i) = gpio_sleep_st[i].output_high;
		/* low */
		REG_GPIO_PXPAT0C(i) = gpio_sleep_st[i].output_low;
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
/*
#ifndef CONFIG_JZ_SYSTEM_AT_CARD
	{
		.gpio = MSC0_HOTPLUG_PIN,
		.active_low = ACTIVE_LOW_MSC0_CD,
	},
#endif
	{
		.gpio = MSC1_HOTPLUG_PIN,
		.active_low = ACTIVE_LOW_MSC1_CD,
	},
*/
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
		dprintk("run int:%x mask:%x pat1:%x pat0:%x pen:%x flg:%x\n",        \
			REG_GPIO_PXINT(i),REG_GPIO_PXMASK(i),REG_GPIO_PXPAT1(i),REG_GPIO_PXPAT0(i), \
			REG_GPIO_PXPEN(i),REG_GPIO_PXFLG(i));
	}

        /* Save GPIO registers */
	for(i = 1; i < GPIO_PORT_NUM; i++) {
		*ptr++ = REG_GPIO_PXINT(i);
		*ptr++ = REG_GPIO_PXMASK(i);
		*ptr++ = REG_GPIO_PXPAT0(i);
		*ptr++ = REG_GPIO_PXPAT1(i);
		*ptr++ = REG_GPIO_PXFLG(i);
		*ptr++ = REG_GPIO_PXPEN(i);
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

	if (!console_suspend_enabled)
		__gpio_as_uart2();

#if 0
        /* Keep uart function for printing debug message */
	__gpio_as_uart0();
	__gpio_as_uart1();
	__gpio_as_uart2();
//	__gpio_as_uart3();

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
		 REG_GPIO_PXINTS(i) = *ptr;
		 REG_GPIO_PXINTC(i) = ~(*ptr++);

		 REG_GPIO_PXMASKS(i) = *ptr;
		 REG_GPIO_PXMASKC(i) = ~(*ptr++);

		 REG_GPIO_PXPAT0S(i) = *ptr;
		 REG_GPIO_PXPAT0C(i) = ~(*ptr++);

		 REG_GPIO_PXPAT1S(i) = *ptr;
		 REG_GPIO_PXPAT1C(i) = ~(*ptr++);

//		 REG_GPIO_PXFLGS(i)=*ptr;
		 REG_GPIO_PXFLGC(i)=~(*ptr++);

		 REG_GPIO_PXPENS(i)=*ptr;
		 REG_GPIO_PXPENC(i)=~(*ptr++);
	}

        /* Print messages of GPIO registers for debug */
	for(i=0;i<GPIO_PORT_NUM;i++) {
		dprintk("run int:%x mask:%x pat1:%x pat0:%x pen:%x flg:%x\n",        \
			REG_GPIO_PXINT(i),REG_GPIO_PXMASK(i),REG_GPIO_PXPAT1(i),REG_GPIO_PXPAT0(i), \
			REG_GPIO_PXPEN(i),REG_GPIO_PXFLG(i));
	}
}

#define WM831X_LDO_MAX_NAME 6

extern void (*jz_timer_callback)(void);
extern int __init jz_add_msc_devices(unsigned int controller, struct jz_mmc_platform_data *plat);

static void dancing(void)
{
	static unsigned char slash[] = "\\|/-";
//	static volatile unsigned char *p = (unsigned char *)0xb6000058;
	static volatile unsigned char *p = (unsigned char *)0xb6000016;
	static unsigned int count = 0;
	*p = slash[count++];
	count &= 3;
}

static void pisces_sd_gpio_init(struct device *dev)
{
#if defined (CONFIG_JZ_SYSTEM_AT_CARD)
	__gpio_as_msc0_boot();
#else
	__gpio_as_msc0_8bit();
#endif

#ifdef GPIO_SD0_VCC_EN_N
	__gpio_as_output1(GPIO_SD0_VCC_EN_N); /* poweroff */
	__gpio_as_input(GPIO_SD0_CD_N);
#endif
}

static void pisces_sd_power_on(struct device *dev)
{

#ifdef GPIO_SD0_VCC_EN_N
	__gpio_as_output0(GPIO_SD0_VCC_EN_N);
#endif
}

static void pisces_sd_power_off(struct device *dev)
{

#ifdef GPIO_SD0_VCC_EN_N
	__gpio_as_output1(GPIO_SD0_VCC_EN_N);
#endif
}

static void pisces_sd_cpm_start(struct device *dev)
{
	cpm_start_clock(CGM_MSC0);
}

static unsigned int pisces_sd_status(struct device *dev)
{
#if defined(CONFIG_JZ_SYSTEM_AT_CARD)
	return 1;
#else
	unsigned int status;

	status = (unsigned int) __gpio_get_pin(GPIO_SD0_CD_N);
  #if ACTIVE_LOW_MSC0_CD
	return !status;
  #else
	return status;
  #endif
#endif
}

static void pisces_sd_plug_change(int state)
{
	if(state == CARD_INSERTED) /* wait for remove */
#if ACTIVE_LOW_MSC0_CD
		__gpio_as_irq_high_level(MSC0_HOTPLUG_PIN);
#else
	__gpio_as_irq_low_level(MSC0_HOTPLUG_PIN);
#endif
	else		      /* wait for insert */
#if ACTIVE_LOW_MSC0_CD
		__gpio_as_irq_low_level(MSC0_HOTPLUG_PIN);
#else
	__gpio_as_irq_high_level(MSC0_HOTPLUG_PIN);
#endif
}

static unsigned int pisces_sd_get_wp(struct device *dev)
{
#if defined(CONFIG_JZ_SYSTEM_AT_CARD)
	return 0;
#else
	unsigned int status;

	status = (unsigned int) __gpio_get_pin(MSC0_WP_PIN);
	return (status);
#endif
}
#if defined(CONFIG_JZ_SYSTEM_AT_CARD)
struct mmc_partition_info pisces_partitions[] = {
	[0] = {"mbr",0,512,0},//0 - 512
	[1] = {"uboot",512,3*1024*1024-512,0}, // 512 - 2.5MB
	[2] = {"misc",0x300000,0x100000,0},//3MB - 1MB
	[3] = {"kernel",0x400000,0x400000,0},//4MB - 4MB
	[4] = {"recovery",0x800000,0x400000,0},//8MB -4MB

	[5] = {"rootfs",12*1024*1024,80*1024*1024,1}, //root
	[6] = {"data",92*1024*1024,50*1024*1024,1},   //APP
	[7] = {"cache",142*1024*1024,18*1024*1024,1},  // CFG
	//[8] = {"test_0",0x0,0xffffffff,0},
	[8] = {"test_0",160*1024*1024,1683*1024*1024UL,0},//2G is 1843
};
#endif

struct jz_mmc_platform_data pisces_sd_data = {
#ifndef CONFIG_JZ_MSC0_SDIO_SUPPORT
	.support_sdio   = 0,
#else
	.support_sdio   = 1,
#endif
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
#if defined(CONFIG_JZ_SYSTEM_AT_CARD)
	.status_irq	= 0,
	.detect_pin     = 0,
#else
	.status_irq	= MSC0_HOTPLUG_IRQ,
	.detect_pin     = GPIO_SD0_CD_N,
#endif
	.init           = pisces_sd_gpio_init,
	.power_on       = pisces_sd_power_on,
	.power_off      = pisces_sd_power_off,
	.cpm_start	= pisces_sd_cpm_start,
	.status		= pisces_sd_status,
	.plug_change	= pisces_sd_plug_change,
	.write_protect  = pisces_sd_get_wp,
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
#if defined(CONFIG_JZ_SYSTEM_AT_CARD)
	.partitions = pisces_partitions,
	.num_partitions = ARRAY_SIZE(pisces_partitions),
	.permission = MMC_BOOT_AREA_PROTECTED,
#endif
};

static void pisces_tf_gpio_init(struct device *dev)
{
	__gpio_as_msc1_4bit();

#ifdef GPIO_SD1_VCC_EN_N
	__gpio_as_output1(GPIO_SD1_VCC_EN_N); /* poweroff */
#endif
	__gpio_as_input(GPIO_SD1_CD_N);
}

static void pisces_tf_power_on(struct device *dev)
{
#ifdef GPIO_SD1_VCC_EN_N
	__gpio_as_output0(GPIO_SD1_VCC_EN_N);
#endif
}

static void pisces_tf_power_off(struct device *dev)
{
#ifdef GPIO_SD1_VCC_EN_N
	__gpio_as_output1(GPIO_SD1_VCC_EN_N);
#endif
}

static void pisces_tf_cpm_start(struct device *dev)
{
	printk("===>start MSC1 clock!\n");
	cpm_start_clock(CGM_MSC1);
	printk("===>REG_CPM_CLKGR0 = 0x%08x\n", REG_CPM_CLKGR0);
}

static unsigned int pisces_tf_status(struct device *dev)
{
	unsigned int status = 0;
	status = (unsigned int) __gpio_get_pin(GPIO_SD1_CD_N);
#if ACTIVE_LOW_MSC1_CD
	return !status;
#else
	return status;
#endif
}

static void pisces_tf_plug_change(int state)
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

struct jz_mmc_platform_data pisces_tf_data = {
#ifndef CONFIG_JZ_MSC1_SDIO_SUPPORT
	.support_sdio   = 0,
#else
	.support_sdio   = 1,
#endif
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.status_irq	= MSC1_HOTPLUG_IRQ,
	.detect_pin     = GPIO_SD1_CD_N,
	.init           = pisces_tf_gpio_init,
	.power_on       = pisces_tf_power_on,
	.power_off      = pisces_tf_power_off,
	.cpm_start	= pisces_tf_cpm_start,
	.status		= pisces_tf_status,
	.plug_change	= pisces_tf_plug_change,
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

static void pisces_msc2_gpio_init(struct device *dev)
{
	__gpio_as_msc2_8bit();
	
#ifdef GPIO_SD2_VCC_EN_N
	__gpio_as_output1(GPIO_SD2_VCC_EN_N); /* poweroff */
	__gpio_as_input(GPIO_SD2_CD_N);
#endif

	return;
}

static void pisces_msc2_power_on(struct device *dev)
{
	
#ifdef GPIO_SD2_VCC_EN_N
	__gpio_as_output0(GPIO_SD2_VCC_EN_N);
#endif

	return;
}

static void pisces_msc2_power_off(struct device *dev)
{

#ifdef GPIO_SD2_VCC_EN_N
	__gpio_as_output1(GPIO_SD2_VCC_EN_N);
#endif

	return;
}

static void pisces_msc2_cpm_start(struct device *dev)
{
	cpm_start_clock(CGM_MSC2);
}

static unsigned int pisces_msc2_status(struct device *dev)
{
	unsigned int status;

	status = (unsigned int) __gpio_get_pin(GPIO_SD2_CD_N);
#if ACTIVE_LOW_MSC2_CD
	return !status;
#else
	return status;
#endif
}

static void pisces_msc2_plug_change(int state)
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

	return;
}

struct jz_mmc_platform_data pisces_msc2_data = {
#ifndef CONFIG_JZ_MSC2_SDIO_SUPPORT
	.support_sdio   = 0,
#else
	.support_sdio   = 1,
#endif
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.status_irq	= MSC2_HOTPLUG_IRQ,
	.detect_pin     = GPIO_SD2_CD_N,
	.init           = pisces_msc2_gpio_init,
	.power_on       = pisces_msc2_power_on,
	.power_off      = pisces_msc2_power_off,
	.cpm_start	= pisces_msc2_cpm_start,
	.status		= pisces_msc2_status,
	.plug_change	= pisces_msc2_plug_change,
	.max_bus_width  = MMC_CAP_SD_HIGHSPEED
#ifdef CONFIG_JZ_MSC2_BUS_4
	| MMC_CAP_4_BIT_DATA
#endif
	,
#ifdef CONFIG_JZ_MSC2_BUS_1
	.bus_width      = 1,
#else
	.bus_width      = 4,
#endif
};

void __init board_msc_init(void)
{
	jz_add_msc_devices(0, &pisces_sd_data);
	//jz_add_msc_devices(1, &pisces_tf_data);
	//jz_add_msc_devices(2, &pisces_msc2_data); //allen del
}

static void pisces_timer_callback(void)
{
	static unsigned long count = 0;

	if ((++count) % 50 == 0) {
		dancing();
		count = 0;
	}
}

static void __init board_cpm_setup(void)
{
	//allen add
#ifdef HAVE_OTHER_UART
	__gpio_as_uart0();
	__cpm_start_uart0();
	
	//__cpm_start_uart1(); //ethernel have use
	
	#if JZ_EARLY_UART_BASE != UART2_BASE
		//__cpm_start_uart2(); //default is uart2,so needn't add
		//__gpio_as_uart2();
	#endif

	 __gpio_as_uart3();
	__cpm_start_uart3();
	
#endif
}

static void __init board_gpio_setup(void)
{
	/*
	 * Initialize SDRAM pins
	 */
}

#ifdef CONFIG_PMU_ACT8600_SUPPORT
static struct act8600_outputs_t act8600_outputs[] = {
#if 0
        {1,0x30,1},//out1 1.2V - 0b110000
        {5,0x31,1},//out5 2.5  - 0b110001
        {6,0x39,0},//out6 LCD3.3V  - 0b111001
        {7,0x18,0},//out7 1.2V CON - 0b011000
        {8,0x24,0},//out8 1.8V CON - 0b100100
#else
    	{4,0x54,1},//out4 5.0V -
      //	{4,0x39,1},//out4 2.5V - 0b110001
        {5,0x31,1},//out5 2.5  - 0b110001
        {6,0x39,0},//out6 LCD3.3V  - 0b111001
        {7,0x39,1},//out7 3.3V CON - 0b011000
        {8,0x24,1},//out8 1.8V CON - 0b100100
#endif
};

static struct act8600_platform_pdata_t act8600_platform_pdata = {
        .outputs = act8600_outputs,
        .nr_outputs = ARRAY_SIZE(act8600_outputs),
};
#endif


static struct i2c_board_info pisces_i2c1_devs[] __initdata = {
        {
		    I2C_BOARD_INFO("nmi", 0x60),

	},
#ifdef CONFIG_PMU_ACT8600_SUPPORT
	{
		I2C_BOARD_INFO(ACT8600_NAME, 0x5a),
		.platform_data = &act8600_platform_pdata,
	},
#endif

};
#ifdef CONFIG_TOUCHSCREEN_FT5X0X
static struct ft5x0x_ts_platform_data ft5x0x_ts_pdata = {
	.intr = GPIO_TS_I2C_INT,
};
#endif

static struct i2c_board_info pisces_i2c0_devs[] __initdata = {
        {
                I2C_BOARD_INFO("ov3640", 0x3c),
        },
	{
                I2C_BOARD_INFO("ov2655", 0x30),
        },
        {
                I2C_BOARD_INFO("ov7690", 0x21),
        },
#if 0
        {
                I2C_BOARD_INFO("jz_ep932", 0x38),
	},
#endif
        {
                I2C_BOARD_INFO("fm5807_i2c", 0x11),  //0x11 
	},
#if 0
	{
	        I2C_BOARD_INFO(FT5X0X_NAME, 0x38),
		.irq = GPIO_TS_I2C_IRQ,
		.platform_data = &ft5x0x_ts_pdata,
	},
#endif
        {
        },
};

#if defined(CONFIG_I2C_GPIO)
static struct i2c_board_info pisces_gpio_i2c_devs[] __initdata = {
        {
                I2C_BOARD_INFO("jz_edid", 0xa0 >> 1),
        },
        {
                I2C_BOARD_INFO("jz_hdcp_rx", 0x74 >> 1),
        },
        {
                I2C_BOARD_INFO("jz_hey", 0xa8 >> 1),
        },
#if 0
        {
                I2C_BOARD_INFO("jz_edid_segment_ptr", 0x60 >> 1),
        },
        {
                I2C_BOARD_INFO("jz_hdcp_rx_bksv", 0x00 >> 1),
        },
        {
                I2C_BOARD_INFO("jz_hdcp_rx_ri", 0x08 >> 1),
        },
        {
                I2C_BOARD_INFO("jz_hdcp_rx_aksv", 0x10 >> 1),
        },
        {
                I2C_BOARD_INFO("jz_hdcp_rx_ainfo", 0x15 >> 1),
        },
        {
                I2C_BOARD_INFO("jz_hdcp_rx_an", 0x18 >> 1),
        },
        {
                I2C_BOARD_INFO("jz_hdcp_rx_sha1_hash", 0x20 >> 1),
        },
        {
                I2C_BOARD_INFO("jz_hdcp_rx_bcaps", 0x40 >> 1),
        },
        {
                I2C_BOARD_INFO("jz_hdcp_rx_bstatus", 0x41 >> 1),
        },
        {
                I2C_BOARD_INFO("jz_hdcp_rx_ksv_fifo", 0x43 >> 1),
        },
#endif
};

static struct i2c_gpio_platform_data pisces_i2c_gpio_data = {
        .sda_pin        = GPIO_I2C1_SDA,
        .scl_pin        = GPIO_I2C1_SCK,
};

static struct platform_device pisces_i2c_gpio_device = {
        .name   = "i2c-gpio",
        .id     = 3,
        .dev    = {
                .platform_data = &pisces_i2c_gpio_data,
        },
};


static struct platform_device *pisces_platform_devices[] __initdata = {
//#if defined(CONFIG_I2C_GPIO)
        &pisces_i2c_gpio_device,
//#endif
};
#endif


void __init board_i2c_init(void) {
        i2c_register_board_info(0, pisces_i2c0_devs, ARRAY_SIZE(pisces_i2c0_devs));
        i2c_register_board_info(1, pisces_i2c1_devs, ARRAY_SIZE(pisces_i2c1_devs));
#if defined(CONFIG_I2C_GPIO)
        i2c_register_board_info(3, pisces_gpio_i2c_devs, ARRAY_SIZE(pisces_gpio_i2c_devs));
        platform_add_devices(pisces_platform_devices, ARRAY_SIZE(pisces_platform_devices));
#endif
}

void __init jz_board_setup(void)
{

	printk("JZ4770 Reference Board (PISCES) setup\n");
//	jz_restart(NULL);
	board_cpm_setup();
	board_gpio_setup();

	jz_timer_callback = pisces_timer_callback;
}

/**
 * Called by arch/mips/kernel/proc.c when 'cat /proc/cpuinfo'.
 * Android requires the 'Hardware:' field in cpuinfo to setup the init.%hardware%.rc.
 */
const char *get_board_type(void)
{
	return "PISCES";
}
