#ifndef __JZ4750_PLATFORM_H__
#define __JZ4750_PLATFORM_H__

/* msc */
#define CARD_INSERTED 1
#define CARD_REMOVED 0

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
};

/* SPI NOR Flash */
struct spi_nor_block_info {
	u32 blocksize;
	u8 cmd_blockerase;
	/* MAX Busytime for block erase, unit: ms */
	u32 be_maxbusy;
};

struct spi_nor_platform_data {
	u32 pagesize;
	u32 sectorsize;
	u32 chipsize;

	/* Some NOR flash has different blocksize and block erase command,
	 * One command with One blocksize. */
	struct spi_nor_block_info *block_info;
	int num_block_info;

	/* Flash Address size, unit: Bytes */
	int addrsize;

	/* MAX Busytime for page program, unit: ms */
	u32 pp_maxbusy;
	/* MAX Busytime for sector erase, unit: ms */
	u32 se_maxbusy;
	/* MAX Busytime for chip erase, unit: ms */
	u32 ce_maxbusy;

	/* Flash status register num, Max support 3 register */
	int st_regnum;
};

#endif /* __JZ4750_PLATFORM_H__ */
