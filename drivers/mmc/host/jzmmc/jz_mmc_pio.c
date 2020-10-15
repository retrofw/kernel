#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/scatterlist.h>
#include <linux/kthread.h>
#include <asm/jzsoc.h>
#include "include/jz_mmc_dma.h"
#include "include/jz_mmc_host.h"

//extern unsigned long loops_per_jiffy; /* 10ms */
//#define WAIT_FIFO_MAX_TIMEOUT	(loops_per_jiffy * 10)
#define WAIT_FIFO_MAX_TIMEOUT	100

static int jz_mmc_pio_read(struct jz_mmc_host *host, unsigned char *buf, size_t len) {
	unsigned int *buf_v = (unsigned int *)buf;
	size_t len_v = (len >> 2);

	unsigned char *buf_o = buf + (len & (~0x3));
	size_t len_o = len & 0x3;
	size_t i = 0;
	unsigned int timeout;

	for (i = 0; i < len_v; i++) {
		timeout = WAIT_FIFO_MAX_TIMEOUT;
		while ( (timeout > 1) && ((REG_MSC_STAT(host->pdev_id) & (MSC_STAT_DATA_FIFO_EMPTY | MSC_STAT_TIME_OUT_READ)) &&
					  !host->eject)) {
			msleep(1);
			timeout --;
		}
		if (timeout == 1) {
			printk("wait fifo not empty timedout!\n");
			return -ETIMEDOUT;
		}

		if (host->eject) {
			printk("card ejected.\n");
			return -ENOMEDIUM;
		}

		if (REG_MSC_STAT(host->pdev_id) & MSC_STAT_TIME_OUT_READ) {
			printk("read timedout!\n");
			return -ETIMEDOUT;
		}

		*buf_v = REG_MSC_RXFIFO(host->pdev_id);
		buf_v++;
	}

	if (len_o) {
		unsigned int temp;
		unsigned char *c = (unsigned char *)&temp;

		timeout = WAIT_FIFO_MAX_TIMEOUT;
		while ( (timeout > 1) && ((REG_MSC_STAT(host->pdev_id) & (MSC_STAT_DATA_FIFO_EMPTY | MSC_STAT_TIME_OUT_READ)) &&
					  !host->eject)) {
			msleep(1);
			timeout --;
		}
		if (timeout == 1) {
			printk("wait fifo not empty timedout!\n");
			return -ETIMEDOUT;
		}

		if (host->eject) {
			printk("card ejected.\n");
			return -ENOMEDIUM;
		}

		if (REG_MSC_STAT(host->pdev_id) & MSC_STAT_TIME_OUT_READ) {
			printk("read timedout!\n");
			return -ETIMEDOUT;
		}

		temp = REG_MSC_RXFIFO(host->pdev_id);

		for (i = 0; i < len_o; i++)
			buf_o[i] = c[i];
	}

	return 0;
}

static int jz_mmc_pio_write(struct jz_mmc_host *host, unsigned char *buf, size_t len) {
	unsigned int *buf_v = (unsigned int *)buf;
	size_t len_v = (len >> 2);

	unsigned char *buf_o = buf + (len & (~0x3));
	size_t len_o = (len & 0x3);

	size_t i = 0;
	unsigned int timeout;

	for (i = 0; i < len_v; i++) {
		timeout = WAIT_FIFO_MAX_TIMEOUT;
		while ( (timeout > 1) && ((REG_MSC_STAT(host->pdev_id) & MSC_STAT_DATA_FIFO_FULL) && !host->eject)) {
			msleep(1);
			timeout --;
		}

		if (timeout == 1) {
			printk("wait fifo not full timedout.\n");
			return -ETIMEDOUT;
		}

		if (host->eject) {
			printk("card ejected.\n");
			return -ENOMEDIUM;
		}

		REG_MSC_TXFIFO(host->pdev_id) = *buf_v;
		buf_v++;
	}

	if (len_o) {
		unsigned int temp;
		unsigned char *c = (unsigned char *)&temp;

		for (i = 0; i < len_o; i++)
			c[i] = buf_o[i];

		timeout = WAIT_FIFO_MAX_TIMEOUT;
		while ( (timeout > 1) && ((REG_MSC_STAT(host->pdev_id) & MSC_STAT_DATA_FIFO_FULL) && !host->eject))
			msleep(1);
			timeout --;

		if (timeout == 1) {
			printk("wait fifo not full timedout.\n");
			return -ETIMEDOUT;
		}

		if (host->eject) {
			printk("card ejected.\n");
			return -ENOMEDIUM;
		}

		REG_MSC_TXFIFO(host->pdev_id) = temp;
	}

	return 0;
}

void jz_mmc_stop_pio(struct jz_mmc_host *host) {
	while(!host->transfer_end) {
		printk("===>transfer not end!\n");
	}
}

#if 0
static int jz_mmc_data_transfer(void *arg) {
	struct jz_mmc_host *host = (struct jz_mmc_host *)arg;
	struct mmc_data *data = host->curr_mrq->data;
	int is_write = data->flags & MMC_DATA_WRITE;
	int i = 0;
	int ret = 0;
	struct scatterlist *sgentry = NULL;
	unsigned char *buf = NULL;
	int len = 0;

	for_each_sg(data->sg, sgentry, data->sg_len, i) {
		buf = sg_virt(sgentry);
		len = sg_dma_len(sgentry);

		if (is_write)
			ret = jz_mmc_pio_write(host, buf, len);
		else
			ret = jz_mmc_pio_read(host, buf, len);
		if (ret) {
			data->error = ret;
			break;
		}
	}

	if (is_write) {
		if (ret) {
			host->data_ack = 0;
			wake_up_interruptible(&host->data_wait_queue);
		}
		/* else, DATA_TRANS_DONE interrupt will raise */
	} else {
		if (!ret)
			host->data_ack = 1;
		else
			host->data_ack = 0;
		wake_up_interruptible(&host->data_wait_queue);
	}

	host->transfer_end = 1;

	return 0;
}

#else
static int jz_mmc_data_transfer(void *arg) {
	struct jz_mmc_host *host = (struct jz_mmc_host *)arg;
	struct mmc_data *data = host->curr_mrq->data;
	int is_write = data->flags & MMC_DATA_WRITE;

	struct sg_mapping_iter	sg_miter;	/* SG state for PIO */
	int sg_flags = 0;

	int ret = 0;

	if (!is_write)	      /* read */
		sg_flags |= SG_MITER_TO_SG;
	else
		sg_flags |= SG_MITER_FROM_SG;

	sg_miter_start(&sg_miter, data->sg, data->sg_len, sg_flags);

	while (sg_miter_next(&sg_miter)) {
		unsigned char *buf = sg_miter.addr;
		size_t len = sg_miter.length;

		//printk("===>buf = %p, phyaddr = 0x%08x len = %d\n", buf, page_to_phys(sg_miter.page), len);

		if (is_write) {
			ret = jz_mmc_pio_write(host, buf, len);
		} else {
			ret = jz_mmc_pio_read(host, buf, len);
		}

		if (ret) {
			data->error = ret;
			break;
		}
	}

	sg_miter_stop(&sg_miter);

	if (is_write) {
		if (ret) {
			host->data_ack = 0;
			wake_up_interruptible(&host->data_wait_queue);
		}
		/* else, DATA_TRANS_DONE interrupt will raise */
	} else {
		if (!ret)
			host->data_ack = 1;
		else
			host->data_ack = 0;
		wake_up_interruptible(&host->data_wait_queue);
	}

	host->transfer_end = 1;

	return 0;
}
#endif

void jz_mmc_start_pio(struct jz_mmc_host *host) {
	REG_MSC_CMDAT(host->pdev_id) &= ~MSC_CMDAT_DMA_EN;
	host->transfer_end = 0;
	kthread_run(jz_mmc_data_transfer, (void *)host, "msc pio transfer");
}
