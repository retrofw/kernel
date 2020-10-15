#include <linux/list.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/vmalloc.h>
#include <linux/gfp.h>
#include <linux/module.h>

#include "jz47xx_dma.h"
#include "jz47xx_data_cvt.h"

#include <asm/jzsoc.h>

static int jz_audio_dma_debug = 0;
module_param(jz_audio_dma_debug, int, 0644);
#define AUDIO_DMA_DEBUG_MSG(msg...)			\
	do {					\
		if (jz_audio_dma_debug)		\
			printk("AUDIO DMA: " msg);	\
	} while(0)


extern void dump_aic_regs(const char *str);
extern void dump_dlv_regs(const char * str);

struct jz_audio_node {
	unsigned char *data;
	int offset;
	int data_len;
	int index;	      /* for debug */
};

struct jz_audio_data_ring {
	struct jz_audio_node *data_nodes;
	dma_addr_t dma_addr_s;
	unsigned int count;
	unsigned int max_node_size;

	/*
	 * for replay: the user is writer, it will put data to next_to_use
	 *	       dma is reader, it will get data from next_to_clean
	 *
	 * for record: the dma is writer, it will put data to next_to_use
	 *	       the user is reader, it will get data from next_to_clean
	 */
	unsigned int next_to_use;
	unsigned int next_to_clean;

	u8 *data_buffer;

	int dma_chan;
	wait_queue_head_t wait_queue;
    struct work_struct dma_work;

	int data_width;
	int audio_chan;
	int (*convert_data)(void *buff, int data_len);

	int is_write;
	int should_stop;
	int someone_waiting;
	atomic_t tokens;
};

#define JZ_AUDIO_NODE_UNUSED(R)						\
	((((R)->next_to_clean > (R)->next_to_use)			\
	  ? 0 : (R)->count) + (R)->next_to_clean - (R)->next_to_use - 1)

#define JZ_AUDIO_NODE_USED(R)  (((R)->count - 1) - JZ_AUDIO_NODE_UNUSED(R))

#define JZ_AUDIO_RING_ACTIVE(R)					\
	((REG_DMAC_DCCSR((R)->dma_chan) & DMAC_DCCSR_EN))

static struct jz_audio_data_ring replay_ring;
static struct jz_audio_data_ring record_ring;

static void jz_audio_start_dma(struct jz_audio_data_ring *ring);
static void audio_dma_work_handler(struct work_struct *work);
static int __jz_audio_init_ring(struct jz_audio_data_ring *ring, u32 nr_frag, u32 frag_size) {
	int i = 0;

	BUG_ON(ring->data_nodes != NULL);
	BUG_ON(ring->data_buffer != NULL);

	ring->data_nodes = vmalloc(nr_frag * sizeof(struct jz_audio_node));
	if (!ring->data_nodes) {
		printk("init ring nodes failed, no enough memory.\n");
		return -ENOMEM;
	}

	ring->data_buffer = (u8 *)__get_free_pages(GFP_KERNEL, get_order(nr_frag * frag_size));
	if (!ring->data_buffer) {
		printk("init ring data failed, no enough memory.\n");
		vfree(ring->data_nodes);
		return -ENOMEM;
	}

	for (i = 0; i < nr_frag; i++) {
		ring->data_nodes[i].data = ring->data_buffer + i * frag_size;
		ring->data_nodes[i].offset = 0;
		ring->data_nodes[i].data_len = 0;
		ring->data_nodes[i].index = i;
	}

	ring->count = nr_frag;
	ring->max_node_size = frag_size;
	ring->next_to_use = ring->next_to_clean = 0;

	atomic_set(&ring->tokens, 1);

    init_waitqueue_head(&ring->wait_queue);
    INIT_WORK(&ring->dma_work, audio_dma_work_handler);

	return 0;
}

static int jz_audio_init_ring(struct jz_audio_data_ring *ring, u32 nr_frag, u32 frag_size) {
	return __jz_audio_init_ring(ring, nr_frag, frag_size);
}

static void jz_audio_deinit_ring(struct jz_audio_data_ring *ring) {
	BUG_ON(JZ_AUDIO_RING_ACTIVE(ring));

	vfree(ring->data_nodes);
	ring->data_nodes = NULL;

	free_pages(ring->data_buffer, get_order(ring->count * ring->max_node_size));
	ring->data_buffer = NULL;
}

static int jz_audio_reinit_ring(struct jz_audio_data_ring *ring, u32 nr_frag, u32 frag_size) {
	jz_audio_deinit_ring(ring);
	return __jz_audio_init_ring(ring, nr_frag, frag_size);
}

static void jz_audio_reset_ring(struct jz_audio_data_ring *ring) {
	ring->next_to_clean = ring->next_to_use = 0;
}

static inline struct jz_audio_node *jz_audio_begin_pull_data(struct jz_audio_data_ring *ring) {
	if (JZ_AUDIO_NODE_USED(ring))
		return ring->data_nodes + ring->next_to_clean;

	return NULL;
}

static inline void jz_audio_end_pull_data(struct jz_audio_data_ring *ring,
					struct jz_audio_node *node) {
	if (node->data_len == 0) {
		ring->next_to_clean ++;
		if (ring->next_to_clean == ring->count)
			ring->next_to_clean = 0;
	}
}

static inline struct jz_audio_node *jz_audio_begin_push_data(struct jz_audio_data_ring *ring) {
	if (JZ_AUDIO_NODE_UNUSED(ring))
		return ring->data_nodes + ring->next_to_use;

	return NULL;
}

static inline void jz_audio_end_push_data(struct jz_audio_data_ring *ring,
				   struct jz_audio_node *node) {
	ring->next_to_use ++;
	if (ring->next_to_use == ring->count)
		ring->next_to_use = 0;
}

static void audio_dma_work_handler(struct work_struct *work)
{
    struct jz_audio_data_ring *ring = container_of(work, struct jz_audio_data_ring, dma_work);
	jz_audio_start_dma(ring);
}

/* dma operations start */

static irqreturn_t jz_audio_dma_callback(int irq, void *devid)
{
	struct jz_audio_data_ring *ring = (struct jz_audio_data_ring *)devid;
	struct jz_audio_node *node;
	int chan = ring->dma_chan;

	disable_dma(chan);
	if (__dmac_channel_address_error_detected(chan)) {
		printk("%s: DMAC address error.\n",
		       __FUNCTION__);
		__dmac_channel_clear_address_error(chan);
	}
	if (__dmac_channel_transmit_end_detected(chan)) {
		__dmac_channel_clear_transmit_end(chan);
	}

	if (ring->is_write) {
		node = jz_audio_begin_pull_data(ring);
		AUDIO_DMA_DEBUG_MSG("node%d finish replay\n", node->index);
		node->data_len = 0;
		jz_audio_end_pull_data(ring, node);
	} else {
		node = jz_audio_begin_push_data(ring);
		node->data_len = ring->max_node_size;
		node->offset = 0;
		jz_audio_end_push_data(ring, node);

		if (ring->someone_waiting) {
			ring->someone_waiting = 0;
			wmb();
			wake_up_interruptible(&ring->wait_queue);
		}
	}

    if(!work_pending(&ring->dma_work))
        schedule_work(&ring->dma_work);

	return IRQ_HANDLED;
}

static void __jz_audio_start_dma(int chan, u8 *data, int data_len,
				 int data_width, int is_write) {
	unsigned long start_time;
	u32 dma_dcmd = 0;
	int ds = 16;

	AUDIO_DMA_DEBUG_MSG("start dma on chan%d\n", chan);

	start_time = jiffies;
	while (REG_DMAC_DMACR(chan / HALF_DMA_NUM) & (DMAC_DMACR_HLT | DMAC_DMACR_AR)) {
		if (jiffies - start_time > 10) { /* 100ms */
			printk("DMAC unavailable! REG_DMAC_DMACR(%d) = 0x%08x\n",
			       chan / HALF_DMA_NUM, REG_DMAC_DMACR(chan / HALF_DMA_NUM));
			jz_stop_dma(chan);
			break;
		}
	}

	start_time = jiffies;
	while (REG_DMAC_DCCSR(chan) & (DMAC_DCCSR_HLT | DMAC_DCCSR_TT | DMAC_DCCSR_AR)) {
		if (jiffies - start_time > 10) { /* 100ms */
			printk("DMA channel %d unavailable! REG_DMAC_DCCSR(%d) = 0x%08x\n",
			       chan, chan, REG_DMAC_DCCSR(chan));
			jz_stop_dma(chan);
			break;
		}
	}

    if (is_write) {
        dma_cache_wback((unsigned long)data,(unsigned long)data_len);
    } else {
        dma_cache_wback_inv((unsigned long)data,(unsigned long)data_len);
    }

	disable_dma(chan);
	REG_DMAC_DCCSR(chan) |= DMAC_DCCSR_NDES; /* No-descriptor transfer */

	/* FIXME: determine burst size by data_len */
	if (is_write) {
		REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_AICOUT;
		REG_DMAC_DSAR(chan) = CPHYSADDR(data);
		REG_DMAC_DTAR(chan) = CPHYSADDR(AIC_DR);

		dma_dcmd = DMAC_DCMD_SAI | DMAC_DCMD_TIE;
		switch(data_width) {
		case 8:
			dma_dcmd = (dma_dcmd | DMAC_DCMD_SWDH_8 | DMAC_DCMD_DWDH_8 | DMAC_DCMD_DS_16BYTE);
			ds = 16;
			break;
		case 16:
			dma_dcmd = (dma_dcmd | DMAC_DCMD_SWDH_16 | DMAC_DCMD_DWDH_16 | DMAC_DCMD_DS_16BYTE);
			ds = 16;
			break;
		case 17 ... 32:
			dma_dcmd = (dma_dcmd | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BYTE);
			ds = 32;
			break;
		default:
			printk("%s: Unkown DMA mode(sound data width) %d\n", __FUNCTION__, data_width);
			break;
		}
	} else {
		REG_DMAC_DRSR(chan) = DMAC_DRSR_RS_AICIN;
		REG_DMAC_DSAR(chan) = CPHYSADDR(AIC_DR);
		REG_DMAC_DTAR(chan) = CPHYSADDR(data);

		dma_dcmd = DMAC_DCMD_DAI | DMAC_DCMD_TIE;

		switch(data_width) {
		case 8:
			dma_dcmd = (dma_dcmd | DMAC_DCMD_SWDH_8 | DMAC_DCMD_DWDH_8 | DMAC_DCMD_DS_16BYTE);
			ds = 16;
			break;
		case 16:
			dma_dcmd = (dma_dcmd | DMAC_DCMD_SWDH_16 | DMAC_DCMD_DWDH_16 | DMAC_DCMD_DS_16BYTE);
			ds = 16;
			break;
		case 17 ... 32:
			dma_dcmd = (dma_dcmd | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BYTE);
			ds = 32;
			break;
		default:
			printk("%s: Unkown DMA mode(sound data width) %d\n", __FUNCTION__, data_width);
			break;
		}
	}

	REG_DMAC_DTCR(chan) = (data_len + ds - 1) / ds;

	REG_DMAC_DCMD(chan) = dma_dcmd;
	REG_DMAC_DMACR(chan / HALF_DMA_NUM) |= DMAC_DMACR_DMAE;

	REG_DMAC_DCCSR(chan) |= DMAC_DCCSR_EN;
}

static void jz_audio_start_dma(struct jz_audio_data_ring *ring) {
	struct jz_audio_node *node;

	AUDIO_DMA_DEBUG_MSG("try start dma on chan%d, token = %d\n",
			    ring->dma_chan, atomic_read(&ring->tokens));

	if (ring->should_stop) {
		AUDIO_DMA_DEBUG_MSG("should stop!\n");
		return;
	}

	if ((REG_DMAC_DCCSR(ring->dma_chan) & DMAC_DCCSR_EN)) {
		AUDIO_DMA_DEBUG_MSG("already running!\n");
		return;
	}

	/* queue on the token ring */
	while(!atomic_dec_and_test(&ring->tokens)) {
                atomic_inc(&ring->tokens);
        }

	/* re-check the dma controller status */
	if ((REG_DMAC_DCCSR(ring->dma_chan) & DMAC_DCCSR_EN)) {
		return;
	}

	if (ring->is_write) {
		node = jz_audio_begin_pull_data(ring);
	} else {
		node = jz_audio_begin_push_data(ring);
	}

	if (!node) {
		atomic_inc(&ring->tokens);
		return;
	}

	__jz_audio_start_dma(ring->dma_chan,
			     node->data, 
                 ring->is_write ? node->data_len : ring->max_node_size,
			     ring->data_width, ring->is_write);

	atomic_inc(&ring->tokens);
}

/* dma operations end */

int jz_audio_dma_resize_buffer(int mode, int nr_frag, int frag_size) {
	if (mode & FMODE_WRITE) {
		jz_audio_reinit_ring(&replay_ring, nr_frag, frag_size);
	}

	if (mode & FMODE_READ) {
		jz_audio_reinit_ring(&record_ring, nr_frag, frag_size);
	}

	return 0;
}

int jz_audio_dma_push(const u8 *data /* user */, int len, int block /* not used */) {
	struct jz_audio_node *node;
	int offset = 0;
	int copy_len;

	AUDIO_DMA_DEBUG_MSG("data = %p, len = %d\n", data, len);

	do {
		node = jz_audio_begin_push_data(&replay_ring);
		/* FIXME: add block stuffs */
		if (!node) {
			AUDIO_DMA_DEBUG_MSG("no enough node\n");
			break;
		}

		AUDIO_DMA_DEBUG_MSG("node index = %d, ring: next_to_use = %d, next_to_clean = %d\n",
		       node->index, replay_ring.next_to_use, replay_ring.next_to_clean);

		copy_len = (len > replay_ring.max_node_size)
			? replay_ring.max_node_size
			: len;

		if (copy_from_user(node->data, data + offset, copy_len)) {
			printk("copy from user failed, data offset = %d, copy_len = %d\n",
			       offset, copy_len);
			return -EFAULT;
		}

		node->offset = 0;
		node->data_len = copy_len;

		jz_audio_end_push_data(&replay_ring, node);

		replay_ring.should_stop = 0;
		jz_audio_start_dma(&replay_ring);

		offset += copy_len;
		len -= copy_len;
	} while (len > 0);

	return offset;
}

int jz_audio_dma_pull(u8 *data /* user */, int len, int block) {

	int ret;
	int offset = 0;
	int copy_len;
	int real_copy_len;
	struct jz_audio_node *node;

	do {
		record_ring.should_stop = 0;
		jz_audio_start_dma(&record_ring);

		node = jz_audio_begin_pull_data(&record_ring);

		if (!node) {
			if (!block)
				break;
			else {
				record_ring.someone_waiting = 1;
				/* the best time is (frag_size / sample_rate) + 1s
				   here, we hope the user do not set a very large frag_size
				*/
				ret = wait_event_interruptible_timeout(record_ring.wait_queue,
								       (record_ring.someone_waiting == 0),
								       3 * HZ);
				if (ret <= 0) { /* failed or time elapsed */
					break;
				}

				continue; /* retry pull node */
			}
		}

		AUDIO_DMA_DEBUG_MSG("node index = %d, ring: next_to_use = %d, next_to_clean = %d\n",
		       node->index, record_ring.next_to_use, record_ring.next_to_clean);

        if(record_ring.audio_chan == 2){
            real_copy_len = copy_len =
                (len > node->data_len) ? node->data_len : len;
        }else if(record_ring.audio_chan == 1){
            real_copy_len = copy_len =
                ((len*2) > node->data_len) ? node->data_len : (len*2);
        }else{
            return -EFAULT;
        }

		if (record_ring.convert_data) {
			real_copy_len = record_ring.convert_data(node->data + node->offset, copy_len);
		}

        AUDIO_DMA_DEBUG_MSG("%s : copy_len = %d, real_copy_len = %d, node->data_len = %d, len = %d\n", 
                            __FUNCTION__, copy_len, real_copy_len, node->data_len, len);

		if (copy_to_user(data + offset, node->data + node->offset, real_copy_len))
			return -EFAULT;

		node->offset += copy_len;
		node->data_len -= copy_len;

		jz_audio_end_pull_data(&record_ring, node);

		offset += real_copy_len;
		len -= real_copy_len;
        AUDIO_DMA_DEBUG_MSG("%s : offset = %d\n", 
                            __FUNCTION__, offset);
	} while (len > 0);

	return offset;
}

static void jz_audio_set_hw_params(int mode, int width, int chan) {
	if (mode & FMODE_WRITE) {
		if (width)
			replay_ring.data_width = width;
		if (chan)
			replay_ring.audio_chan = chan;
	}
	if (mode & FMODE_READ) {
		if (width)
			record_ring.data_width = width;

		if (chan)
			record_ring.audio_chan = chan;

		if (record_ring.audio_chan == 1) {
			if (record_ring.data_width == 8) {
				record_ring.convert_data = convert_8bits_stereo2mono;
			} else if (record_ring.data_width == 16) {
				record_ring.convert_data = convert_16bits_stereo2mono;
			} else if(record_ring.data_width == 24){
                record_ring.convert_data = convert_32bits_stereo2mono;
            }else{
				record_ring.convert_data = NULL;
			}
		} else {
			record_ring.convert_data = NULL;
		}
	}
}

void jz_audio_dma_set_width(int mode, int width) {
	jz_audio_set_hw_params(mode, width, 0);
}

void jz_audio_dma_set_channels(int mode, int chan) {
	jz_audio_set_hw_params(mode, 0, chan);
}

int jz_audio_dma_stop(int mode) {
	if (mode & FMODE_WRITE) {
		replay_ring.should_stop = 1;
		while(JZ_AUDIO_RING_ACTIVE(&replay_ring));
	}

	if (mode & FMODE_READ) {
		record_ring.should_stop = 1;
		while(JZ_AUDIO_RING_ACTIVE(&record_ring));
	}

	return 0;
}

int jz_audio_dma_start(int mode) {
	if (mode & FMODE_WRITE) {
		replay_ring.should_stop = 0;
		jz_audio_start_dma(&replay_ring);
	}

	if (mode & FMODE_READ) {
		record_ring.should_stop = 0;
		jz_audio_start_dma(&record_ring);
	}

	return 0;
}

void jz_audio_dma_sync(int mode) {
	if (mode & FMODE_WRITE)
		while(JZ_AUDIO_RING_ACTIVE(&replay_ring));

	if (mode & FMODE_READ)
		while(JZ_AUDIO_RING_ACTIVE(&record_ring));
}

int jz_audio_dma_flush(int mode) {
	if (mode & FMODE_WRITE) {
		replay_ring.should_stop = 1;
		while(JZ_AUDIO_RING_ACTIVE(&replay_ring));

		jz_audio_reset_ring(&replay_ring);
	}

	if (mode & FMODE_READ) {
		record_ring.should_stop = 1;
		while(JZ_AUDIO_RING_ACTIVE(&record_ring));

		jz_audio_reset_ring(&record_ring);
	}

	return 0;
}

int jz_audio_dma_get_ospace(audio_buf_info *abinfo) {
    AUDIO_DMA_DEBUG_MSG("%s  %s[%d]:",__FILE__, __FUNCTION__, __LINE__);
    abinfo->fragments = JZ_AUDIO_NODE_UNUSED(&replay_ring);
    abinfo->fragstotal = replay_ring.count;
    abinfo->fragsize = replay_ring.max_node_size;
    abinfo->bytes = abinfo->fragments * replay_ring.max_node_size;
	return 0;
}

int jz_audio_dma_get_ispace(audio_buf_info *abinfo) {
    AUDIO_DMA_DEBUG_MSG("%s  %s[%d]:",__FILE__, __FUNCTION__, __LINE__);
    abinfo->fragments = JZ_AUDIO_NODE_UNUSED(&record_ring);
    abinfo->fragstotal = record_ring.count;
    abinfo->fragsize = record_ring.max_node_size;
    abinfo->bytes = abinfo->fragments * record_ring.max_node_size;
	return 0;
}

int jz_audio_dma_get_odelay(int *unfinish) {
    int count, next;
    AUDIO_DMA_DEBUG_MSG("%s  %s[%d]:",__FILE__, __FUNCTION__, __LINE__);

    count = 0;
    next = replay_ring.next_to_clean;
    while(next != replay_ring.next_to_use){
        count += (replay_ring.data_nodes + next)->data_len;
        ++next;
        if(next == replay_ring.count){
            next = 0;
        }
    }

    *unfinish = count;
	return 0;
}

int jz_audio_dma_init(void) {
	int ret = 0;

	memset(&replay_ring, 0, sizeof(struct jz_audio_data_ring));
	replay_ring.is_write = 1;
	replay_ring.dma_chan = jz_request_dma(DMA_ID_AIC_TX, "audio write",
					  jz_audio_dma_callback, IRQF_DISABLED, &replay_ring);
	if (replay_ring.dma_chan < 0) {
		printk("AUDIO WRITE: request dma failed\n");
		return -ENODEV;
	}

	memset(&record_ring, 0, sizeof(struct jz_audio_data_ring));
	record_ring.is_write = 0;
	record_ring.dma_chan = jz_request_dma(DMA_ID_AIC_RX, "audio read",
					  jz_audio_dma_callback, IRQF_DISABLED, &record_ring);
	if (record_ring.dma_chan < 0) {
		printk("AUDIO READ: request dma failed\n");
		jz_free_dma(replay_ring.dma_chan);
		return -ENODEV;
	}

	ret = jz_audio_init_ring(&replay_ring, DEFAULT_NR_FRAG, DEFAULT_FRAG_SIZE);
	if (ret < 0) {
		goto error;
	}

	ret = jz_audio_init_ring(&record_ring, DEFAULT_NR_FRAG, DEFAULT_FRAG_SIZE);
	if (ret < 0) {
		goto error;
	}

	return 0;

error:
	jz_audio_deinit_ring(&replay_ring);
	jz_audio_deinit_ring(&record_ring);

	jz_free_dma(replay_ring.dma_chan);
	jz_free_dma(record_ring.dma_chan);
	return ret;
}

int jz_audio_dma_deinit(void) {
	replay_ring.should_stop = 1;
	record_ring.should_stop = 1;

	while(JZ_AUDIO_RING_ACTIVE(&replay_ring));
	while(JZ_AUDIO_RING_ACTIVE(&record_ring));

	jz_free_dma(replay_ring.dma_chan);
	jz_free_dma(record_ring.dma_chan);

	jz_audio_deinit_ring(&replay_ring);
	jz_audio_deinit_ring(&record_ring);

	return 0;
}
