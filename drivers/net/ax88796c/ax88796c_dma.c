#include <linux/version.h>
#include <asm/jzsoc.h>
#include "ax88796c.h"

struct ax_dma_channel {
	int		chan;
	void		*priv;
	dma_addr_t	dma_addr;
	void (*dma_complete)(void *data);
};

static struct ax_dma_channel rx_chan;
static struct ax_dma_channel tx_chan;

static irqreturn_t ax88796c_dma_callback(int irq, void *devid)
{
	struct ax_dma_channel *dma_chan = (struct ax_dma_channel *)devid;

	disable_dma(dma_chan->chan);
	if (__dmac_channel_address_error_detected(dma_chan->chan)) {
		printk("%s: DMAC address error.\n",
		       __FUNCTION__);
		__dmac_channel_clear_address_error(dma_chan->chan);
	}
	if (__dmac_channel_transmit_end_detected(dma_chan->chan)) {
		__dmac_channel_clear_transmit_end(dma_chan->chan);
		dma_chan->dma_complete(dma_chan->priv);
	}

	return IRQ_HANDLED;
}

static void jz_start_normal_dma(int chan, unsigned long mem_addr, unsigned long dev_addr,
			     int count, int tx)
{
	unsigned long flags;
	u32 dma_cmd = 0;
	u32 src_addr = 0;
	u32 dst_addr = 0;
	u32 ds = 4;

	BUG_ON(src_addr % 4);

	if (count % 32 == 0)
		ds = 32; /* 32 byte */
	else if (count % 16 == 0)
		ds = 16; /* 16 byte */
	else
		ds = 4; /* default to 4 byte */

	flags = claim_dma_lock();
	dma_cmd = DMAC_DCMD_RDIL_IGN | DMAC_DCMD_TIE;
	if (tx)
		dma_cmd |= DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_16;
	else
		dma_cmd |= DMAC_DCMD_SWDH_16 | DMAC_DCMD_DWDH_32;
	switch (ds) {
	case 32:
		dma_cmd |= DMAC_DCMD_DS_32BYTE;
		break;

	case 16:
		dma_cmd |= DMAC_DCMD_DS_16BYTE;
		break;

	case 4:
		dma_cmd |= DMAC_DCMD_DS_32BIT;
		break;

	default:
		;
	}
	if (tx) {
		dma_cmd |= DMAC_DCMD_SAI;
		src_addr = (unsigned int)mem_addr;      /* DMA source address */
		dst_addr = dev_addr;      /* DMA target address */
	} else {
		dma_cmd |= DMAC_DCMD_DAI;
		src_addr = dev_addr;
		dst_addr = mem_addr;
	}
//	printk("dma_cmd =%08x\n",dma_cmd); //jarvis debug
//	printk("source addr = %08x \n",src_addr);
//	printk("target  addr = %08x \n",dst_addr);

	REG_DMAC_DCCSR(chan) |= DMAC_DCCSR_NDES; /* No-descriptor transfer */
	REG_DMAC_DSAR(chan) = src_addr;
	REG_DMAC_DTAR(chan) = dst_addr;
	REG_DMAC_DTCR(chan) = (count + ds - 1) / ds;
	REG_DMAC_DCMD(chan) = dma_cmd;
	REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_AUTO;

	REG_DMAC_DMACR(chan / HALF_DMA_NUM) |= DMAC_DMACR_DMAE;
	REG_DMAC_DCCSR(chan) |= DMAC_DCCSR_EN;
	release_dma_lock(flags);
}

void dma_start (dma_addr_t mem_addr, int len, u8 tx) {
	if (tx) {
		jz_start_normal_dma(tx_chan.chan, mem_addr, tx_chan.dma_addr, len * 2, tx);
	} else {
		jz_start_normal_dma(rx_chan.chan, mem_addr, rx_chan.dma_addr, len * 2, 0);
	}
}

int ax88796c_platform_dma_init (unsigned long base_addr,
				void (*tx_dma_complete)(void *data),
				void (*rx_dma_complete)(void *data),
				void *priv)
{
	if (rx_dma_complete) {
		rx_chan.dma_complete = rx_dma_complete;
		rx_chan.priv = priv;
		rx_chan.dma_addr = CPHYSADDR(base_addr + DATA_PORT_ADDR);
		rx_chan.chan = jz_request_dma(DMA_ID_AX88796C_RX,
					       "ax88796c_rx",
					       ax88796c_dma_callback,
					       0, &rx_chan);
	}

	if (tx_dma_complete) {
		tx_chan.dma_complete = tx_dma_complete;
		tx_chan.priv = priv;
		tx_chan.dma_addr = CPHYSADDR(base_addr + DATA_PORT_ADDR);
		tx_chan.chan = jz_request_dma(DMA_ID_AX88796C_TX,
					       "ax88796c_tx",
					       ax88796c_dma_callback,
					       0, &tx_chan);
	}

	return 0;
}
