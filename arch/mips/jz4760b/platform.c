/*
 * Platform device support for Jz4760b SoC.
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

#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif

#include <asm/jzsoc.h>
#include <linux/usb/musb.h>
#include <../sound/oss/jz_audio.h>
#include <linux/spi/spi.h>

extern void __init board_msc_init(void);

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

/* USB OTG Controller */
static struct platform_device jz_usb_otg_xceiv_device = {
	.name	= "nop_usb_xceiv",
	.id	= 0,
};

static struct musb_hdrc_config jz_usb_otg_config = {
	.multipoint	= 1,
	.dyn_fifo	= 0,
	.soft_con	= 1,
	.dma		= 1,
/* Max EPs scanned. Driver will decide which EP can be used automatically. */
	.num_eps	= 6,
};

static struct musb_hdrc_platform_data jz_usb_otg_platform_data = {
#if defined(CONFIG_USB_MUSB_OTG)
	.mode           = MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode           = MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode           = MUSB_PERIPHERAL,
#endif
	.config		= &jz_usb_otg_config,
};

static struct resource jz_usb_otg_resources[] = {
	[0] = {
		.start		= CPHYSADDR(UDC_BASE),
		.end		= CPHYSADDR(UDC_BASE) + 0x10000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_OTG,
		.end		= IRQ_OTG,
		.flags		= IORESOURCE_IRQ,
	},
};

static u64  usb_otg_dmamask = ~(u32)0;

static struct platform_device jz_usb_otg_device = {
	.name	= "musb_hdrc",
	.id	= 0,
	.dev = {
		.dma_mask		= &usb_otg_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &jz_usb_otg_platform_data,
	},
	.num_resources	= ARRAY_SIZE(jz_usb_otg_resources),
	.resource	= jz_usb_otg_resources,
};

/** MMC/SD/SDIO controllers**/
#define __BUILD_JZ_MSC_PLATFORM_DEV(msc_id)				\
	static struct resource jz_msc##msc_id##_resources[] = {		\
		{							\
			.start          = CPHYSADDR(MSC##msc_id##_BASE), \
			.end            = CPHYSADDR(MSC##msc_id##_BASE) + 0x1000 - 1, \
			.flags          = IORESOURCE_MEM,		\
		},							\
		{							\
			.start          = IRQ_MSC##msc_id,		\
			.end            = IRQ_MSC##msc_id,		\
			.flags          = IORESOURCE_IRQ,		\
		},							\
		{							\
			.start          = DMA_ID_MSC##msc_id,	\
			.end            = DMA_ID_MSC##msc_id,	\
			.flags          = IORESOURCE_DMA,		\
		},							\
	};								\
									\
	static u64 jz_msc##msc_id##_dmamask =  ~(u32)0;			\
									\
	static struct platform_device jz_msc##msc_id##_device = {	\
		.name = "jz-msc",					\
		.id = msc_id,						\
		.dev = {						\
			.dma_mask               = &jz_msc##msc_id##_dmamask, \
			.coherent_dma_mask      = 0xffffffff,		\
		},							\
		.num_resources  = ARRAY_SIZE(jz_msc##msc_id##_resources), \
		.resource       = jz_msc##msc_id##_resources,		\
	};

#ifdef CONFIG_JZ_MSC0
__BUILD_JZ_MSC_PLATFORM_DEV(0)
#endif
#ifdef CONFIG_JZ_MSC1
__BUILD_JZ_MSC_PLATFORM_DEV(1)
#endif
#ifdef CONFIG_JZ_MSC2
__BUILD_JZ_MSC_PLATFORM_DEV(2)
#endif

static struct platform_device *jz_msc_devices[] __initdata = {
#ifdef CONFIG_JZ_MSC0
	&jz_msc0_device,
#else
	NULL,
#endif
#ifdef CONFIG_JZ_MSC1
	&jz_msc1_device,
#else
	NULL,
#endif
#ifdef CONFIG_JZ_MSC2
	&jz_msc2_device,
#else
	NULL,
#endif
};

int __init jz_add_msc_devices(unsigned int id, struct jz_mmc_platform_data *plat)
{
	struct platform_device	*pdev;

	if (JZ_MSC_ID_INVALID(id))
		return -EINVAL;

	pdev = jz_msc_devices[id];
	if (NULL == pdev) {
		return -EINVAL;
	}

	pdev->dev.platform_data = plat;

	return platform_device_register(pdev);
}

/* + Sound device */
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
		.platform_data = &vogue_snd_device,
	},
};

static struct resource jz_i2c0_resources[] = {
	[0] = {
		.start          = CPHYSADDR(I2C0_BASE),
		.end            = CPHYSADDR(I2C0_BASE) + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_I2C0,
		.end            = IRQ_I2C0,
		.flags          = IORESOURCE_IRQ,
	},
};

static struct resource jz_i2c1_resources[] = {
	[0] = {
		.start          = CPHYSADDR(I2C1_BASE),
		.end            = CPHYSADDR(I2C1_BASE) + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_I2C1,
		.end            = IRQ_I2C1,
		.flags          = IORESOURCE_IRQ,
	},
};

static u64 jz_i2c_dmamask =  ~(u32)0;

static struct platform_device jz_i2c0_device = {
	.name = "jz_i2c0",
	.id = 0,
	.dev = {
		.dma_mask               = &jz_i2c_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_i2c0_resources),
	.resource       = jz_i2c0_resources,
};

static struct platform_device jz_i2c1_device = {
	.name = "jz_i2c1",
	.id = 1,
	.dev = {
		.dma_mask               = &jz_i2c_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_i2c1_resources),
	.resource       = jz_i2c1_resources,
};

static struct platform_device rtc_device = {
	.name		= "jz4760b-rtc",
	.id		= -1,
};

#ifdef CONFIG_JZ_AX88796C
/** AX88796C controller **/
static struct resource ax88796c_resources[] = {
	[0] = {
		.start          = CPHYSADDR(0xb4000000),
		.end            = CPHYSADDR(0xb4000000) + 0x6800 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = GPIO_NET_INT + IRQ_GPIO_0,
		.end            = GPIO_NET_INT + IRQ_GPIO_0,
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
#endif /* CONFIG_JZ_AX88796C */

///////////////////////////////////

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
		.start          = CPHYSADDR(SSI##ssi_id##_BASE) ,			\
		.end            = CPHYSADDR(SSI##ssi_id##_BASE) + 0x1000 - 1,	\
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
	&jz_usb_otg_xceiv_device,
	&jz_usb_otg_device,
	&jz_lcd_device,
	&vogue_snd_device,
	&jz_i2c0_device,
	&jz_i2c1_device,
	// &jz_msc0_device,
	// &jz_msc1_device,
	&rtc_device,
#ifdef CONFIG_JZ_AX88796C
	&ax88796c_dev,
#endif
};

extern void __init board_i2c_init(void);
extern void __init board_spi_init(void);
static int __init jz_platform_init(void)
{
	int ret = 0;

	board_i2c_init();
	
	board_spi_init();

	ret = platform_add_devices(jz_platform_devices, ARRAY_SIZE(jz_platform_devices));
#ifdef CONFIG_ANDROID_PMEM
	platform_pmem_device_setup();
#endif

	printk("jz_platform_init\n");
	board_msc_init();
	return ret;
}

arch_initcall(jz_platform_init);

