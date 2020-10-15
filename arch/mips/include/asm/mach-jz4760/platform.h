#ifndef __JZ4760_PLATFORM_H__
#define __JZ4760_PLATFORM_H__

/* msc */
#define CARD_INSERTED 1
#define CARD_REMOVED 0

#ifdef CONFIG_JZ_SYSTEM_AT_CARD
struct mmc_partition_info {
	char name[32];
	unsigned int saddr;
	unsigned int len;
	int type;
};
#endif

struct jz_mmc_platform_data {
	unsigned int ocr_mask;			/* available voltages */
	unsigned long detect_delay;		/* delay in jiffies before detecting cards after interrupt */
	unsigned char status_irq;
	unsigned char support_sdio;
	unsigned char bus_width;
	unsigned int max_bus_width;
	unsigned int detect_pin;

	unsigned char msc_irq;
	unsigned char dma_rxid;
	unsigned char dma_txid;

	void *driver_data;

	void (*init) (struct device *);
	void (*power_on) (struct device *);
	void (*power_off) (struct device *);
	void (*cpm_start) (struct device *);
	unsigned int (*status) (struct device *);
	unsigned int (*write_protect) (struct device *);
	void (*plug_change) (int);
#ifdef CONFIG_JZ_SYSTEM_AT_CARD
	struct mmc_partition_info *partitions;
	int num_partitions;
	
	unsigned int permission;
	
#define MMC_BOOT_AREA_PROTECTED	(0x1234)	/* Can not modified the area protected */
#define MMC_BOOT_AREA_OPENED	(0x4321)	/* Can modified the area protected */
#endif
};

#endif /* __JZ4760_PLATFORM_H__ */
