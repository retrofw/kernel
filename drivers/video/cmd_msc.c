/*
*
*	Umido add for suport read mmc
*	<kongjianhua@umidotech.com>
*/
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <asm/gpio.h>
#include <asm/jzsoc.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/system.h>

static int highcap = 0;
static int rca;

/*
 * GPIO definition
 */
#define MMC_IRQ_MASK()				\
do {						\
	REG_MSC_IMASK(0) = 0xffff;			\
	REG_MSC_IREG(0) = 0xffff;			\
} while (0)

/* Stop the MMC clock and wait while it happens */
static inline int jz_mmc_stop_clock(void)
{
	int timeout = 1000;

	REG_MSC_STRPCL(0) = MSC_STRPCL_CLOCK_CONTROL_STOP;

	while (timeout && (REG_MSC_STAT(0) & MSC_STAT_CLK_EN)) {
		timeout--;
		if (timeout == 0) {
			return -1;
		}
		udelay(1);
	}
	return 0;
}

/* Start the MMC clock and operation */
static inline int jz_mmc_start_clock(void)
{
	REG_MSC_STRPCL(0) = MSC_STRPCL_CLOCK_CONTROL_START | MSC_STRPCL_START_OP;
	return 0;
}

static u8 * mmc_cmd(u16 cmd, unsigned int arg, unsigned int cmdat, u16 rtype)
{
	static u8 resp[20];
	u32 timeout = 0x3fffff;
	int words, i;

	jz_mmc_stop_clock();
	REG_MSC_CMD(0) = cmd;
	REG_MSC_ARG(0) = arg;
	REG_MSC_CMDAT(0) = cmdat;

	REG_MSC_IMASK(0) = ~MSC_IMASK_END_CMD_RES;
	jz_mmc_start_clock();

	while (timeout-- && !(REG_MSC_STAT(0) & MSC_STAT_END_CMD_RES))
		;
	REG_MSC_IREG(0) = MSC_IREG_END_CMD_RES;

	switch (rtype) {
		case MSC_CMDAT_RESPONSE_R1:
		case MSC_CMDAT_RESPONSE_R3:
			words = 3;
			break;

		case MSC_CMDAT_RESPONSE_R2:
			words = 8;
			break;

		default:
			return 0;
	}
	
	for (i = words-1; i >= 0; i--) {
		u16 res_fifo = REG_MSC_RES(0);
		int offset = i << 1;

		resp[offset] = ((u8 *)&res_fifo)[0];
		resp[offset+1] = ((u8 *)&res_fifo)[1];
	}
	return resp;
}

#if 0
static int __mmc_block_writem(u32 src, u32 num, u8 *dst)
{
	u8 *resp;
	u32 stat, timeout, cnt, nob, sorm;
	u32 *wbuf = (u32 *)dst;

	resp = mmc_cmd(16, 0x200, 0x401, MSC_CMDAT_RESPONSE_R1);
	REG_MSC_BLKLEN(0) = 0x200;
	REG_MSC_NOB(0) = num / 512;
	nob  = num / 512;

	if (nob == 1) {
		if (highcap)
			resp = mmc_cmd(24, src, 0x419, MSC_CMDAT_RESPONSE_R1);
		else
			resp = mmc_cmd(24, src * 512, 0x419, MSC_CMDAT_RESPONSE_R1);

		sorm = 0;
	} else {
		if (highcap)
			resp = mmc_cmd(25, src, 0x419, MSC_CMDAT_RESPONSE_R1); // for sdhc card
		else
			resp = mmc_cmd(25, src * 512, 0x419, MSC_CMDAT_RESPONSE_R1);

		sorm = 1;
	}

	for(nob; nob >= 1; nob--) {

		timeout = 0x3FFFFFF;

		while (timeout) {
			timeout--;
			stat = REG_MSC_STAT(0);
			if (stat & (MSC_STAT_CRC_WRITE_ERROR | MSC_STAT_CRC_WRITE_ERROR_NOSTS)) {
				printk("\n MSC_STAT_CRC_WRITE_ERROR\n\n");
				return -1;
			}
			else if (!(stat & MSC_STAT_DATA_FIFO_FULL)) {
				/* Ready to write data */
				break;
			}

			udelay(1);
		}

		if (!timeout)
			return -1;

		/* Write data to TXFIFO */
		cnt = 128;
		while (cnt) {
			while (REG_MSC_STAT(0) & MSC_STAT_DATA_FIFO_FULL)
				;
			REG_MSC_TXFIFO(0) = *wbuf++;
			cnt--;
		}
	}

	if (sorm)
		resp = mmc_cmd(12, 0, 0x41, MSC_CMDAT_RESPONSE_R1);

	while (!(REG_MSC_STAT(0) & MSC_STAT_DATA_TRAN_DONE))
		;

	while (!(REG_MSC_STAT(0) & MSC_STAT_PRG_DONE))
		;
	jz_mmc_stop_clock();
	return 0;
}
#endif

static int __mmc_block_readm(u32 src, u32 num, u8 *dst)
{
	u8 *resp;
	u32 stat, timeout, data, cnt, nob, sorm;

	resp = mmc_cmd(16, 0x200, 0x401, MSC_CMDAT_RESPONSE_R1);
	REG_MSC_BLKLEN(0) = 0x200;
	REG_MSC_NOB(0) = num / 512;
	nob  = num / 512;

	if (nob == 1) {
		if (highcap)
			resp = mmc_cmd(17, src, 0x409, MSC_CMDAT_RESPONSE_R1);
		else
			resp = mmc_cmd(17, src * 512, 0x409, MSC_CMDAT_RESPONSE_R1);
			
		sorm = 0;
	} else {
		if (highcap)
			resp = mmc_cmd(18, src, 0x409, MSC_CMDAT_RESPONSE_R1);
		else
			resp = mmc_cmd(18, src * 512, 0x409, MSC_CMDAT_RESPONSE_R1);

		sorm = 1;
	}

	for (nob; nob >= 1; nob--) {

		timeout = 0x3ffffff;

		while (timeout) {
			timeout--;
			stat = REG_MSC_STAT(0);
			if (stat & MSC_STAT_TIME_OUT_READ) {
				printk("\n MSC_STAT_TIME_OUT_READ\n\n");
				return -1;
			}
			else if (stat & MSC_STAT_CRC_READ_ERROR) {
				printk("\n MSC_STAT_CRC_READ_ERROR\n\n");
				return -1;
			}
			else if (!(stat & MSC_STAT_DATA_FIFO_EMPTY)) {
				/* Ready to read data */
				break;
			}
			udelay(1);
		}
		if (!timeout) {
			printk("\n mmc/sd read timeout\n");
			return -1;
		}

		/* Read data from RXFIFO. It could be FULL or PARTIAL FULL */
		cnt = 128;
		while (cnt) {
			while (cnt && (REG_MSC_STAT(0) & MSC_STAT_DATA_FIFO_EMPTY))
				;
			cnt --;

			data = REG_MSC_RXFIFO(0);
			{
				*dst++ = (u8)(data >> 0);
				*dst++ = (u8)(data >> 8);
				*dst++ = (u8)(data >> 16);
				*dst++ = (u8)(data >> 24);
			}
		}
	}

	if (sorm)
		resp = mmc_cmd(12, 0, 0x41, MSC_CMDAT_RESPONSE_R1);

	jz_mmc_stop_clock();
	return 0;
}

#if 1
static void sd_init(void)
{
	int retries;
	u8 *resp;
	unsigned int cardaddr;

	resp = mmc_cmd(41, 0x40ff8000, 0x3, MSC_CMDAT_RESPONSE_R3);
	retries = 500;
	while (retries-- && resp && !(resp[4] & 0x80)) {
		resp = mmc_cmd(55, 0, 0x1, MSC_CMDAT_RESPONSE_R1);
		resp = mmc_cmd(41, 0x40ff8000, 0x3, MSC_CMDAT_RESPONSE_R3);
		udelay(1000);
		udelay(1000);
	}

	if (resp[4] & 0x80) 
		printk("SD init ok\n");
	else 
		printk("SD init fail\n");

	/* try to get card id */
	resp = mmc_cmd(2, 0, 0x2, MSC_CMDAT_RESPONSE_R2);
	resp = mmc_cmd(3, 0, 0x6, MSC_CMDAT_RESPONSE_R1);
	cardaddr = (resp[4] << 8) | resp[3]; 
	rca = cardaddr << 16;
	resp = mmc_cmd(9, rca, 0x2, MSC_CMDAT_RESPONSE_R2);
	highcap = (resp[14] & 0xc0) >> 6;
	REG_MSC_CLKRT(0) = 2;    //2 is ok ,dont't grater
	resp = mmc_cmd(7, rca, 0x41, MSC_CMDAT_RESPONSE_R1);
	resp = mmc_cmd(55, rca, 0x1, MSC_CMDAT_RESPONSE_R1);
	resp = mmc_cmd(6, 0x2, 0x401, MSC_CMDAT_RESPONSE_R1);
}

/* init mmc/sd card we assume that the card is in the slot */
static int  mmc_init(void)
{
	int retries;
	u8 *resp;
#if 0
	__gpio_as_msc();
	__msc_reset(0);
	MMC_IRQ_MASK();	
	__cpm_select_msc_clk(0,1);
	REG_MSC_CLKRT(0) = 7;    //250k
	REG_MSC_RDTO(0) = 0xffffffff;
#endif
	resp = mmc_cmd(12, 0, 0x41, MSC_CMDAT_RESPONSE_R1);
	/* reset */
	resp = mmc_cmd(0, 0, 0x80, 0);
	resp = mmc_cmd(8, 0x1aa, 0x1, MSC_CMDAT_RESPONSE_R1);
	resp = mmc_cmd(55, 0, 0x1, MSC_CMDAT_RESPONSE_R1);
	if(!(resp[0] & 0x20) && (resp[5] != 0x37)) { 
		resp = mmc_cmd(1, 0x40ff8000, 0x3, MSC_CMDAT_RESPONSE_R3);
		retries = 500;
		while (retries-- && resp && !(resp[4] & 0x80)) {
			resp = mmc_cmd(1, 0x40300000, 0x3, MSC_CMDAT_RESPONSE_R3);
			udelay(1000);
			udelay(1000);
		}

		if ((resp[4] & 0x80 ) == 0x80) 
			printk("MMC init ok\n");
		else 
			printk("MMC init fail\n");

		if((resp[4] & 0x60) == 0x40)
			highcap = 1;
		else
			highcap = 0;
	
		/* try to get card id */
		resp = mmc_cmd(2, 0, 0x2, MSC_CMDAT_RESPONSE_R2);
		resp = mmc_cmd(3, 0x10, 0x1, MSC_CMDAT_RESPONSE_R1);

		REG_MSC_CLKRT(0) = 2;	/* 16/1 MHz */
		resp = mmc_cmd(7, 0x10, 0x1, MSC_CMDAT_RESPONSE_R1);
		resp = mmc_cmd(6, 0x3b70101, 0x441, MSC_CMDAT_RESPONSE_R1);
	}
	else
		sd_init();
	return 0;
}
#endif

/*
 *The API for Load data from MMC/SD into RAM
 */
int msc_read(unsigned long start_byte, u8 *dst, size_t len)
{
	int start_sect;

	mmc_init();
	start_sect = start_byte / 512;
	__mmc_block_readm(start_sect, len, dst);

	return 0;
}

EXPORT_SYMBOL(msc_read);
#if 0
static int msc_write(unsigned long start_byte, u8 *dst, size_t len)
{
	int start_sect;

	start_sect = start_byte / 512;
	__mmc_block_writem(start_sect, len, dst);

	return 0;
}
#endif
