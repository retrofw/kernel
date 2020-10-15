#ifndef __LINUX_SPI_JZ47XX_H
#define __LINUX_SPI_JZ47XX_H

#define R_MODE		0x1
#define W_MODE		0x2
#define RW_MODE	(R_MODE | W_MODE)

#define R_DMA		0x4
#define W_DMA		0x8
#define RW_DMA		(R_DMA |W_DMA)

#define SPI_DMA_ACK			0x1

#define SPI_DMA_ERROR  		-3
#define SPI_CPU_ERROR		-4

#define SPI_COMPLETE		5


#define JZ_SSI_MAX_FIFO_ENTRIES 	128
#define JZ_SSI_DMA_BURST_LENGTH 	16

#define FIFO_W8			8
#define FIFO_W16		16
#define FIFO_W32		32

#define SPI_BITS_8		8
#define SPI_BITS_16		16
#define SPI_BITS_32		32


#define SPI_8BITS		1
#define SPI_16BITS		2
#define SPI_32BITS		4




/* tx rx threshold from 0x0 to 0xF */
#define SSI_FULL_THRESHOLD			0xF
#define SSI_TX_FIFO_THRESHOLD		0x1
#define SSI_RX_FIFO_THRESHOLD		(SSI_FULL_THRESHOLD - SSI_TX_FIFO_THRESHOLD)
#define SSI_SAFE_THRESHOLD			0x1

#define CPU_ONCE_BLOCK_ENTRIES 		((SSI_FULL_THRESHOLD-SSI_TX_FIFO_THRESHOLD)*8)

#define MAX_SSI_INTR		10000

#if defined(CONFIG_SOC_JZ4750) || defined(CONFIG_SOC_JZ4750D)
#define MAX_SSICDR			15
#define MAX_CGV				255
#else
#define MAX_SSICDR			63
#define MAX_CGV				255
#endif

#define SSI_DMA_FASTNESS_CHNL 	 0   // SSI controller [n] FASTNESS when probe();

#define DMA_INVALID 			-1

#if defined(CONFIG_SOC_JZ4750D) /* jz4755 */
#define GPIO_AS_SSI(n)	 	\
    __gpio_as_ssi()		
#define GPIO_AS_SSI_EX(n)	 	\
	__gpio_as_ssi_x()

#else
#define GPIO_AS_SSI(n)	 	\
do{								\
	if(n) __gpio_as_ssi1();		\
	else __gpio_as_ssi0();		\
}while(0)
#define GPIO_AS_SSI_EX(n)	 	\
do{								\
	if(n) __gpio_as_ssi1_x();		\
	else __gpio_as_ssi0_x();		\
}while(0)
#endif

#if defined(CONFIG_SOC_JZ4750)	 /* jz4750 */
#define CPM_SSI_START(n) 	((n) ? __cpm_start_ssi(1):__cpm_start_ssi(0))
#define CPM_SSI_STOP(n)		((n) ? __cpm_stop_ssi(1):__cpm_stop_ssi(0))

#define SSI0_CE0_PIN	(32*1+29)
#define SSI0_CE1_PIN	(32*1+31)
#define SSI0_GPC_PIN	(32*1+30)
#define SSI1_CE0_PIN	(32*3+29)
#define SSI1_CE1_PIN	(32*3+30)

#elif defined(CONFIG_SOC_JZ4750D) /* jz4755 */
#define CPM_SSI_START(n) 	((n) ? :__cpm_start_ssi(0))
#define CPM_SSI_STOP(n)		((n) ? :__cpm_stop_ssi(0))

#define SSI0_CE0_PIN	(32*1+29)
#define SSI0_CE1_PIN	(32*1+31)
#define SSI0_GPC_PIN	(32*1+30)
#define SSI1_CE0_PIN	(32*1+29)		/* same as SSI0, for avoiding compilation error and ... */
#define SSI1_CE1_PIN	(32*1+31)

#else /* CONFIG_SOC_JZ4760 || CONFIG_SOC_JZ4760B */
extern unsigned int cpm_get_clock(cgu_clock); 	
#define CPM_SSI_START(n) 	((n) ?cpm_start_clock(CGM_SSI1):cpm_start_clock(CGM_SSI0))
#define CPM_SSI_STOP(n)		((n) ?cpm_stop_clock(CGM_SSI1):cpm_stop_clock(CGM_SSI0))

#define SSI0_CE0_PIN	(32*1+29)
#define SSI0_CE1_PIN	(32*1+31)
#define SSI0_GPC_PIN	(32*1+30)
#define SSI1_CE0_PIN	(32*3+29)
#define SSI1_CE1_PIN	(32*3+30)

#endif

#if defined(CONFIG_SOC_JZ4750) || defined(CONFIG_SOC_JZ4750D)
#undef JZ_NEW_CODE_TYPE
#else
#define JZ_NEW_CODE_TYPE
#endif

struct jz47xx_spi {
	/* bitbang has to be first */
	struct spi_bitbang	 bitbang;
	struct completion	 done;
	
	u8			 chnl;
	u8			 rw_mode;
	u8			 spi_mode;
	u8			 use_dma;
	u8			 is_first;

	u8			 bits_per_word;		/*8 or 16 (or 32)*/
	u8			 transfer_unit_size;	/* 1 or 2 (or 4) */
	u8			 tx_trigger;					/* 0-128 */
	u8			 rx_trigger;					/* 0-128 */
	u8			 dma_tx_unit;			/* 1 or 2 or 4 or 16 or 32*/
	u8			 dma_rx_unit;			/* 1 or 2 or 4 or 16 or 32*/
	u8			 txfifo_width;
	u8			 rxfifo_width;

	/* data buffers */
	const u8	*tx;
	u8			*rx;
	
	void __iomem		*regs;
	int			 irq;
	u32			 len;
	u32			 rlen;	  /* receive len */
	u32			 count;   /* sent count */
	u32			 dma_flag;

	void			(*set_cs)(struct jz47xx_spi_info *spi, u8 cs, unsigned int pol);
	
	/* functions to deal with different size buffers */
	u32 (*get_rx) (struct jz47xx_spi *);
	u32 (*get_tx) (struct jz47xx_spi *);
	


	int dma_tx_chnl;                 /* dma tx channel                  */
	int dma_rx_chnl;                 /* dma rx channel                  */
	
	dma_addr_t	tx_dma;
	dma_addr_t	rx_dma;
	
	unsigned long src_clk;
	unsigned long spi_clk;
	
	struct jz_intr_cnt *g_jz_intr;

	struct resource		*ioarea;
	struct spi_master	*master;
	struct device		*dev;
	struct jz47xx_spi_info *pdata;
};


struct jz_intr_cnt{
	int dma_tx_cnt;
	int dma_rx_cnt;
	int ssi_intr_cnt;
	int ssi_txi;
	int ssi_rxi;
	int ssi_eti;
	int ssi_eri;
	int ssi_rlen;
	int dma_tx_err;
	int dma_tx_end;
	int dma_rx_err;
	int dma_rx_end;
};

#endif /* __LINUX_SPI_JZ47XX_H */
