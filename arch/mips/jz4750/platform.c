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

#include <asm/jzsoc.h>
#include <../sound/oss/jz_audio.h>
#include <linux/spi/spi.h>

extern void __init board_msc_init(void);

int __init jz_add_msc_devices(unsigned int controller, struct jz_mmc_platform_data *plat);

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
	.name = "jz-msc",
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

	return platform_device_register(pdev);
}
#endif


/** I2C controller **/
static struct resource jz_i2c_resources[] = {
	[0] = {
		.start          = CPHYSADDR(I2C_BASE),
		.end            = CPHYSADDR(I2C_BASE) + 0x10000 - 1,
		.flags          = IORESOURCE_MEM,
	},
#if 0
	[1] = {
		.start          = IRQ_I2C,
		.end            = IRQ_I2C,
		.flags          = IORESOURCE_IRQ,
	}
#endif
};

static u64 jz_i2c_dmamask =  ~(u32)0;
static struct platform_device jz_i2c_device = {
	.name = "jz_i2c",
	.id = 0,
	.dev = {
		.dma_mask               = &jz_i2c_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_i2c_resources),
	.resource       = jz_i2c_resources,
};


/** AX88796C controller **/
static struct resource ax88796c_resources[] = {
	[0] = {
		.start          = CPHYSADDR(0xac000000),
		.end            = CPHYSADDR(0xac000000) + 0x6800 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = 118,
		.end            = 118,
		.flags          = IORESOURCE_IRQ,
	}
};

static u64 ax88796c_dmamask =  ~(u32)0;

static struct platform_device ax88796c_dev = {
	.name = "ax88796c",
	.id = 0,
	.dev = {
		.dma_mask               = &ax88796c_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(ax88796c_resources),
	.resource       = ax88796c_resources,
};

//////////////////////////////////////////////////////////
#define SND(num, desc) { .name = desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
 SND(0, "HANDSET"),
 SND(1, "SPEAKER"),
 SND(2, "HEADSET"),

};
#undef SND

static struct jz_snd_endpoints vogue_snd_endpoints = {
      .endpoints = snd_endpoints_list,
      .num = ARRAY_SIZE(snd_endpoints_list),
};

static struct platform_device vogue_snd_device = {
    .name = "mixer",
    .id = -1,
    .dev = {
      .platform_data = &vogue_snd_endpoints,
    },
};
///////////////////////////////////////////////////////////

#define __BUILD_JZ_SPI_PLATFORM_DEV(ssi_id) 	\
											\
struct jz47xx_spi_info spi##ssi_id##_info_cfg = {		\
	.chnl = ssi_id,										\
	.bus_num = ssi_id,									\
	.is_pllclk = 1,										\
	.num_chipselect = MAX_SPI_CHIPSELECT_NUM,		\
};					\
static struct resource jz_spi##ssi_id##_resource[] = {	\
	[0] = {												\
		.start          = CPHYSADDR(SSI_BASE) + 0x1000*(ssi_id) ,			\
		.end            = CPHYSADDR(SSI_BASE) + 0x1000*((ssi_id)+1) - 1,	\
		.flags 			= IORESOURCE_MEM,				\
	},									\
	[1] = {								\
		.start = IRQ_SSI##ssi_id,		\
		.end   = IRQ_SSI##ssi_id,		\
		.flags = IORESOURCE_IRQ,		\
	},									\
};	\
static u64 jz_spi##ssi_id##_dmamask =  ~(u32)0;			\
struct platform_device jz_spi##ssi_id##_device = {		\
	.name		  = "jz47xx-spi",						\
	.id		  = ssi_id,									\
	.num_resources	  = ARRAY_SIZE(jz_spi##ssi_id##_resource),			\
	.resource	  = jz_spi##ssi_id##_resource,							\
        .dev              = {											\
                .dma_mask = &jz_spi##ssi_id##_dmamask,					\
                .coherent_dma_mask = 0xffffffffUL,						\
                .platform_data = & spi##ssi_id##_info_cfg,				\
        },																\
};


#ifdef CONFIG_JZ_SPI0
__BUILD_JZ_SPI_PLATFORM_DEV(0)
#endif
#ifdef CONFIG_JZ_SPI1
__BUILD_JZ_SPI_PLATFORM_DEV(1)
#endif


static struct platform_device *jz_spi_devices[] __initdata = {
#ifdef CONFIG_JZ_SPI0
	&jz_spi0_device,
#else
	NULL,
#endif
#ifdef CONFIG_JZ_SPI1
	&jz_spi1_device,
#else
	NULL,
#endif
};

int __init jz_add_spi_devices(unsigned int host_id, struct spi_board_info *board_info,int board_num)
{

	struct platform_device	*pdev;
	struct jz47xx_spi_info	*host_info;
	
	if (JZ_SPI_ID_INVALID(host_id))
		return -EINVAL;
	pdev = jz_spi_devices[host_id];
	if (NULL == pdev)
		return -EINVAL;

	host_info = (struct jz47xx_spi_info *)pdev->dev.platform_data;
	host_info->board_info = board_info;
	host_info->board_size = board_num;
#ifndef CONFIG_JZ_SPI_BOARD_INFO_REGISTER
	spi_register_board_info(board_info,board_num);	
#endif

	return platform_device_register(pdev);
}


/* All */
static struct platform_device *jz_platform_devices[] __initdata = {
	&jz_usb_ohci_device,
	&jz_lcd_device,
	&jz_usb_gdt_device,
	//	&jz_mmc_device,
	&jz_i2c_device,
	&vogue_snd_device,
	&ax88796c_dev,

};

extern void __init board_i2c_init(void);
extern void __init board_spi_init(void);
static int __init jz_platform_init(void)
{
	board_i2c_init();
	board_msc_init();

	board_spi_init();


	return platform_add_devices(jz_platform_devices, ARRAY_SIZE(jz_platform_devices));
}

arch_initcall(jz_platform_init);
