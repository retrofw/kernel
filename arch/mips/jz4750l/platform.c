/*
 * Platform device support for Jz4740 SoC.
 *
 * Copyright 2007, <yliu@ingenic.cn>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/resource.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>

#include <asm/jzsoc.h>
#include <../sound/oss/jz_audio.h>
#include <linux/mfd/jz4750l-sadc.h>

int __init jz_device_register(struct platform_device *pdev,void *pdata)
{
	pdev->dev.platform_data = pdata;

	return platform_device_register(pdev);
}

#if 0
/* OHCI (USB full speed host controller) */
static struct resource jz_usb_ohci_resources[] = {
	[0] = {
		.start		= CPHYSADDR(UHC_BASE), // phys addr for ioremap
		.end		= CPHYSADDR(UHC_BASE) + 0x10000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_UHC,
		.end		= IRQ_UHC,
		.flags		= IORESOURCE_IRQ,
	},
};

/* The dmamask must be set for OHCI to work */
static u64 ohci_dmamask = ~(u32)0;

static struct platform_device jz_usb_ohci_device = {
	.name		= "jz-ohci",
	.id		= 0,
	.dev = {
		.dma_mask		= &ohci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(jz_usb_ohci_resources),
	.resource	= jz_usb_ohci_resources,
};
#endif
/*** LCD controller ***/
static struct resource jz_lcd_resources[] = {
	[0] = {
		.start          = CPHYSADDR(LCD_BASE),
		.end            = CPHYSADDR(LCD_BASE) + 0x10000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_LCD,
		.end            = IRQ_LCD,
		.flags          = IORESOURCE_IRQ,
	}
};

static u64 jz_lcd_dmamask = ~(u32)0;

static struct platform_device jz_lcd_device = {
	.name           = "jz-lcd",
	.id             = 0,
	.dev = {
		.dma_mask               = &jz_lcd_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_lcd_resources),
	.resource       = jz_lcd_resources,
};

/* UDC (USB gadget controller) */
static struct resource jz_usb_gdt_resources[] = {
	[0] = {
		.start		= CPHYSADDR(UDC_BASE),
		.end		= CPHYSADDR(UDC_BASE) + 0x10000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_UDC,
		.end		= IRQ_UDC,
		.flags		= IORESOURCE_IRQ,
	},
};

static u64 udc_dmamask = ~(u32)0;

static struct platform_device jz_usb_gdt_device = {
	.name		= "jz-udc",
	.id		= 0,
	.dev = {
		.dma_mask		= &udc_dmamask,
		.coherent_dma_mask	= 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(jz_usb_gdt_resources),
	.resource	= jz_usb_gdt_resources,
};

/** MMC/SD controller **/
#if 0
static struct resource jz_mmc_resources[] = {
	[0] = {
		.start          = CPHYSADDR(MSC_BASE),
		.end            = CPHYSADDR(MSC_BASE) + 0x10000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_MSC0,
		.end            = IRQ_MSC0,
		.flags          = IORESOURCE_IRQ,
	}
};

static u64 jz_mmc_dmamask =  ~(u32)0;

static struct platform_device jz_mmc_device = {
	.name = "jz-mmc",
	.id = 0,
	.dev = {
		.dma_mask               = &jz_mmc_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_mmc_resources),
	.resource       = jz_mmc_resources,
};
#else
/** MMC/SD controller MSC0**/
static struct resource jz_msc0_resources[] = {
	{
		.start          = CPHYSADDR(MSC_BASE),
		.end            = CPHYSADDR(MSC_BASE) + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = IRQ_MSC0,
		.end            = IRQ_MSC0,
		.flags          = IORESOURCE_IRQ,
	},
	{
		.start          = DMA_ID_MSC0_RX,
		.end            = DMA_ID_MSC0_TX,
		.flags          = IORESOURCE_DMA,
	},
};

static u64 jz_msc0_dmamask =  ~(u32)0;

static struct platform_device jz_msc0_device = {
	.name = "jz-msc",
	.id = 0,
	.dev = {
		.dma_mask               = &jz_msc0_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_msc0_resources),
	.resource       = jz_msc0_resources,
};

/** MMC/SD controller MSC1**/
static struct resource jz_msc1_resources[] = {
	{
		.start          = CPHYSADDR(MSC_BASE) + 0x1000,
		.end            = CPHYSADDR(MSC_BASE) + 0x10000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = IRQ_MSC1,
		.end            = IRQ_MSC1,
		.flags          = IORESOURCE_IRQ,
	},
	{
		.start          = DMA_ID_MSC1_RX,
		.end            = DMA_ID_MSC1_TX,
		.flags          = IORESOURCE_DMA,
	},

};

static u64 jz_msc1_dmamask =  ~(u32)0;

static struct platform_device jz_msc1_device = {
	.name = "jz-msc",
	.id = 1,
	.dev = {
		.dma_mask               = &jz_msc1_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_msc1_resources),
	.resource       = jz_msc1_resources,
};

static struct platform_device *jz_msc_devices[] __initdata = {
	&jz_msc0_device,
	&jz_msc1_device,
};

int __init jz_add_msc_devices(unsigned int controller, struct jz_mmc_platform_data *plat)
{
	struct platform_device	*pdev;

	if (controller < 0 || controller > 1)
		return -EINVAL;

	pdev = jz_msc_devices[controller];

	pdev->dev.platform_data = plat;

	printk("%s, %d\n", __func__, __LINE__);

	return platform_device_register(pdev);
}
#endif

/* Syncronous Serial Interface */
#ifdef CONFIG_JZ_SPI0
static u32 spi_chip_select[] = {GPIO_PC(19)};
static struct jz47xx_spi_info spi_info_cfg = {
	.chnl		= 0,
	.bus_num	= 0,
	.is_pllclk	= 1,
	.num_chipselect = ARRAY_SIZE(spi_chip_select),
	.chipselect	= spi_chip_select,
#ifdef CONFIG_SPI0_USE_DMA
	.use_dma	= 1,
#endif
};

static struct resource jz_spi_resource[] = {
	[0] = {
		.start	= CPHYSADDR(SSI_BASE),
		.end	= CPHYSADDR(SSI_BASE) + 0x1000 - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_SSI,
		.end	= IRQ_SSI,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 jz_spi_dmamask =  ~(u32)0;
static struct platform_device jz_spi_device = {
	.name		= "jz47xx-spi",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(jz_spi_resource),
	.resource	= jz_spi_resource,
        .dev	= {
                .dma_mask = &jz_spi_dmamask,
                .coherent_dma_mask = 0xffffffffUL,
                .platform_data = &spi_info_cfg,
        },
};
#endif

/* SPI By GPIO */
#ifdef CONFIG_SPI_GPIO
static struct spi_gpio_platform_data jz4750l_spi_gpio_data = {
	.sck		= GPIO_PC(16),
	.mosi		= GPIO_PC(17),
	.miso		= GPIO_PC(18),
	.num_chipselect	= 1,
};

static struct platform_device jz4750l_spi_gpio_device = {
	.name	= "spi_gpio",
	.id	= 3,
	.dev	= {
		.platform_data = &jz4750l_spi_gpio_data,
	},
};
#endif /* CONFIG_SPI_GPIO */

int __init jz_add_spi_devices(struct spi_board_info *board_info, int board_array)
{
	if (!board_array)
		return -EINVAL;

#ifndef CONFIG_JZ_SPI_BOARD_INFO_REGISTER
        spi_register_board_info(board_info, board_array);
#endif

	return 0;
}

////////////////////////////////
#define SND(num, desc) { .name = desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
 SND(0, "HANDSET"),
 SND(1, "SPEAKER"),
 SND(2, "HEADSET"),
};
#undef SND

static struct jz_snd_endpoints jz4750_snd_endpoints = {
      .endpoints = snd_endpoints_list,
      .num = ARRAY_SIZE(snd_endpoints_list),
};

static struct platform_device jz_snd_device = {
    .name = "mixer",
    .id = -1,
    .dev = {
      .platform_data = &jz4750_snd_endpoints,
    },
};

#ifdef CONFIG_MFD_JZ4750L_SADC
static struct resource jz_adc_resources[] = {
	{
		.start	= CPHYSADDR(SADC_BASE),
		.end	= CPHYSADDR(SADC_BASE) + 0x2c - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_SADC,
		.end	= IRQ_SADC,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start  = IRQ_SADC_0,	/* IRQ SADC BASE NUM */
		.end    = IRQ_SADC_0,
		.flags  = IORESOURCE_IRQ,
	},
};

struct platform_device jz_adc_device = {
	.name = "jz4750l-adc",
	.id = -1,
	.num_resources	= ARRAY_SIZE(jz_adc_resources),
	.resource	= jz_adc_resources,
};

struct jz_adc_platform_data jz_adc_pdata;
#endif
////////////////////////////////////////

/* All */
static struct platform_device *jz_platform_devices[] __initdata = {
//	&jz_usb_ohci_device,
	&jz_lcd_device,
	&jz_usb_gdt_device,
	&jz_snd_device,
	//&jz_mmc_device,
#ifdef CONFIG_JZ_SPI0
	&jz_spi_device,
#endif
#ifdef CONFIG_SPI_GPIO
	&jz4750l_spi_gpio_device,
#endif
};

extern void __init board_devices_init(void);
static int __init jz_platform_init(void)
{
	board_devices_init();

	return platform_add_devices(jz_platform_devices, ARRAY_SIZE(jz_platform_devices));
}

arch_initcall(jz_platform_init);
