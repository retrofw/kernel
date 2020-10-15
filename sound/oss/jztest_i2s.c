/*
 * Linux/sound/oss/jz_i2s.c
 *
 * Sound driver for Ingenic Jz4750 MIPS processor
 *
 * 2009-12-xx	Steven <dsqiu@ingenic.cn>
 * 2010-01-xx	Jason <xwang@ingenic.cn>
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/sound.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <linux/proc_fs.h>
#include <linux/soundcard.h>
#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <asm/hardirq.h>
#include <asm/jzsoc.h>
#include "sound_config.h"

#include <linux/miscdevice.h>
#include <linux/platform_device.h>
//#include <linux/msm_audio.h>
#include "jz_codec.h"
#include "jz_i2s_dbg.h"

//#if defined CONFIG_PM
//#undef CONFIG_PM
//#endif

#define DMA_ID_I2S_TX			DMA_ID_AIC_TX
#define DMA_ID_I2S_RX			DMA_ID_AIC_RX

// reference to dma.c
#define DMA_TX_CHAN			6
#define DMA_RX_CHAN			7

#define NR_I2S				2

#define JZCODEC_RW_BUFFER_SIZE		1
#define JZCODEC_RW_BUFFER_TOTAL		4

#define AUDIOBUF_STATE_FREE		0

#define NOMAL_STOP			0
#define FORCE_STOP			1
#define PIPE_TRANS			1

#define AUDIO_LOCK(lock, flags)		spin_lock_irqsave(&lock, flags)
#define AUDIO_UNLOCK(lock, flags)	spin_unlock_irqrestore(&lock, flags)

#define THIS_AUDIO_NODE(p)		list_entry(p, audio_node, list)
#define ALIGN_PAGE_SIZE(x)		(((x) + PAGE_SIZE) / PAGE_SIZE * PAGE_SIZE)

typedef struct {
	struct list_head list;
	unsigned int pBuf;
#ifdef Q_DEBUG
	unsigned int pBufID;
#endif
	unsigned int start;
	unsigned int end;
	unsigned int phyaddr;
} audio_node;

typedef struct {
	unsigned int fact;
	unsigned int datasize;
	unsigned int listsize;
	struct list_head free;
	struct list_head use;
} audio_head;

typedef struct
{
	int ch;
	int onetrans_bit;
	int rw;
	unsigned int *trans_addr;
	unsigned int *trans_count;
	unsigned int *trans_mode;
	unsigned int *data_addr;
} audio_dma_type;

typedef struct __audio_pipe
{
	spinlock_t	lock;
	audio_dma_type	dma;
	unsigned int	*mem;
	audio_node	*savenode;

	int	fragsize;
	int 	fragstotal;
	int	is_non_block;
	volatile int	trans_state;

	wait_queue_head_t	q_full;
	int			avialable_couter;

#ifdef WORK_QUEUE_MODE
	struct work_struct	work;
#endif
	void (*handle)(struct __audio_pipe *endpoint);
	int (*filter)(void *buff, int cnt);
} audio_pipe;

struct i2s_codec
{
	/* I2S controller connected with */
	void	*private_data;
	char	*name;
	int	id;
	int	dev_mixer;

	int	use_mic_line_flag;
	int	audio_volume;
	int	mic_gain;
	int	bass_gain;

	unsigned short	record_audio_rate;
	unsigned short	replay_audio_rate;

	short	replay_codec_channel;
	short	record_codec_channel;

	short	replay_format;
	short	record_format;

	int	audiomute;
	int	user_need_mono;

	struct semaphore i2s_sem;
	int (*codecs_ioctrl)(void *context, unsigned int cmd, unsigned long arg);
};

struct jz_i2s_controller_info
{
	char		*name;
	audio_pipe	*pout_endpoint;
	audio_pipe	*pin_endpoint;
	int		dev_audio;
	unsigned int	error;	/* over / underrun */

	struct i2s_codec *i2s_codec;

#ifdef CONFIG_PM
	struct pm_dev	*pm;
#endif
};


/*
 * Global variates
 */
static audio_pipe out_endpoint = {
	.mem		= 0,
	.savenode	= 0,
	.fragsize	= 0,
	.fragstotal	= 0,
	.trans_state	= 0,
};

static audio_pipe in_endpoint= {
	.mem		= 0,
	.savenode	= 0,
	.fragsize	= 0,
	.fragstotal	= 0,
	.trans_state	= 0,
};

static struct i2s_codec the_codecs[NR_I2S];
static struct jz_i2s_controller_info *the_i2s_controller = NULL;
static int audio_mix_modcnt = 0;
static audio_node *last_read_node = NULL;
static int g_play_first = 0;

#ifdef CONFIG_JZ_EBOOK_HARD
int audio_device_open = 0;
volatile int audio_device_pm_state = 0;
#endif

/*
 * Debug functions
 */
#ifdef DMA_DEBUG
void dump_dma(unsigned int dmanr, const char *str)
{
	printk("DMA%d Registers, %s:\n", dmanr, str);
	printk("\tDMACR	= 0x%08x\n", REG_DMAC_DMACR(dmanr/HALF_DMA_NUM));
	printk("\tDSAR	= 0x%08x\n", REG_DMAC_DSAR(dmanr));
	printk("\tDTAR	= 0x%08x\n", REG_DMAC_DTAR(dmanr));
	printk("\tDTCR	= 0x%08x\n", REG_DMAC_DTCR(dmanr));

	*(unsigned int *)0xb342010c = 0x18;

	printk("\tDRSR	= 0x%08x, addr = 0x%08x\n", REG_DMAC_DRSR(dmanr), DMAC_DRSR(dmanr));
	printk("\tDCCSR	= 0x%08x\n", REG_DMAC_DCCSR(dmanr));
	printk("\tDCMD	= 0x%08x\n", REG_DMAC_DCMD(dmanr));
	printk("\tDDA	= 0x%08x\n", REG_DMAC_DDA(dmanr));
	printk("\tDMADBR= 0x%08x\n", REG_DMAC_DMADBR(dmanr/HALF_DMA_NUM));
	printk("\tCPCCR = 0x%08x\n", REG_CPM_CPCCR);
	printk("\tCPPCR = 0x%08x\n", REG_CPM_CPPCR0);
	printk("\tREG_CPM_CLKGR0 = 0x%08x\n", REG_CPM_CLKGR0);
	printk("\tREG_CPM_CLKGR1 = 0x%08x\n", REG_CPM_CLKGR1);
}
#endif

#ifdef IOC_DEBUG
void dsp_print_ioc_cmd(int cmd)
{
	int i;
	int cmd_arr[] = {
		OSS_GETVERSION,		SNDCTL_DSP_RESET,	SNDCTL_DSP_SYNC,
		SNDCTL_DSP_SPEED,	SNDCTL_DSP_STEREO,	SNDCTL_DSP_GETBLKSIZE,
		SNDCTL_DSP_GETFMTS,	SNDCTL_DSP_SETFMT,	SNDCTL_DSP_CHANNELS,
		SNDCTL_DSP_POST,	SNDCTL_DSP_SUBDIVIDE,	SNDCTL_DSP_SETFRAGMENT,
		SNDCTL_DSP_GETCAPS,	SNDCTL_DSP_NONBLOCK,	SNDCTL_DSP_SETDUPLEX,
		SNDCTL_DSP_GETOSPACE,	SNDCTL_DSP_GETISPACE,	SNDCTL_DSP_GETTRIGGER,
		SNDCTL_DSP_SETTRIGGER,	SNDCTL_DSP_GETIPTR,	SNDCTL_DSP_GETOPTR,
		SNDCTL_DSP_GETODELAY,	SOUND_PCM_READ_RATE,	SOUND_PCM_READ_CHANNELS,
		SOUND_PCM_READ_BITS,	SNDCTL_DSP_MAPINBUF,	SNDCTL_DSP_MAPOUTBUF,
		SNDCTL_DSP_SETSYNCRO,	SOUND_PCM_READ_FILTER,	SOUND_PCM_WRITE_FILTER,
//		AUDIO_GET_CONFIG,	AUDIO_SET_CONFIG
	};
	char *cmd_str[] = {
		"OSS_GETVERSION",	"SNDCTL_DSP_RESET",	"SNDCTL_DSP_SYNC",
		"SNDCTL_DSP_SPEED",	"SNDCTL_DSP_STEREO",	"SNDCTL_DSP_GETBLKSIZE",
		"SNDCTL_DSP_GETFMTS",	"SNDCTL_DSP_SETFMT",	"SNDCTL_DSP_CHANNELS",
		"SNDCTL_DSP_POST",	"SNDCTL_DSP_SUBDIVIDE",	"SNDCTL_DSP_SETFRAGMENT",
		"SNDCTL_DSP_GETCAPS",	"SNDCTL_DSP_NONBLOCK",	"SNDCTL_DSP_SETDUPLEX",
		"SNDCTL_DSP_GETOSPACE",	"SNDCTL_DSP_GETISPACE",	"SNDCTL_DSP_GETTRIGGER",
		"SNDCTL_DSP_SETTRIGGER","SNDCTL_DSP_GETIPTR",	"SNDCTL_DSP_GETOPTR",
		"SNDCTL_DSP_GETODELAY",	"SOUND_PCM_READ_RATE",	"SOUND_PCM_READ_CHANNELS",
		"SOUND_PCM_READ_BITS",	"SNDCTL_DSP_MAPINBUF",	"SNDCTL_DSP_MAPOUTBUF",
		"SNDCTL_DSP_SETSYNCRO",	"SOUND_PCM_READ_FILTER","SOUND_PCM_WRITE_FILTER",
//		"AUDIO_GET_CONFIG",	"AUDIO_SET_CONFIG"
	};

	for ( i = 0; i < sizeof(cmd_arr) / sizeof(int); i++) {
		if (cmd_arr[i] == cmd) {
			printk("Command name : %s\n", cmd_str[i]);
			return;
		}
	}

	if (i == sizeof(cmd_arr) / sizeof(int)) {
		printk("Unknown command\n");
	}
}

void mixer_print_ioc_cmd(int cmd)
{
	int i;
	int cmd_arr[] = {
		SOUND_MIXER_INFO,	SOUND_OLD_MIXER_INFO,		SOUND_MIXER_READ_STEREODEVS,
		SOUND_MIXER_READ_CAPS,	SOUND_MIXER_READ_DEVMASK,	SOUND_MIXER_READ_RECMASK,
		SOUND_MIXER_READ_RECSRC,SOUND_MIXER_WRITE_SPEAKER,	SOUND_MIXER_WRITE_BASS,
		SOUND_MIXER_READ_BASS,	SOUND_MIXER_WRITE_VOLUME,	SOUND_MIXER_READ_VOLUME,
		SOUND_MIXER_WRITE_MIC,	SOUND_MIXER_READ_MIC,		SOUND_MIXER_WRITE_LINE,
		SOUND_MIXER_READ_LINE,	SOUND_MIXER_WRITE_MUTE,		SOUND_MIXER_READ_MUTE,
//		SND_SET_DEVICE,		SND_SET_VOLUME,
//		SND_GET_NUM_ENDPOINTS,	SND_GET_ENDPOINT
	};

	char *cmd_str[] = {
		"SOUND_MIXER_INFO",		"SOUND_OLD_MIXER_INFO",		"SOUND_MIXER_READ_STEREODEVS",
		"SOUND_MIXER_READ_CAPS",	"SOUND_MIXER_READ_DEVMASK",	"SOUND_MIXER_READ_RECMASK",
		"SOUND_MIXER_READ_RECSRC",	"SOUND_MIXER_WRITE_SPEAKER",	"SOUND_MIXER_WRITE_BASS",
		"SOUND_MIXER_READ_BASS",	"SOUND_MIXER_WRITE_VOLUME",	"SOUND_MIXER_READ_VOLUME",
		"SOUND_MIXER_WRITE_MIC",	"SOUND_MIXER_READ_MIC",		"SOUND_MIXER_WRITE_LINE",
		"SOUND_MIXER_READ_LINE",	"SOUND_MIXER_WRITE_MUTE",	"SOUND_MIXER_READ_MUTE",
//		"SND_SET_DEVICE",		"SND_SET_VOLUME",
//		"SND_GET_NUM_ENDPOINTS",	"SND_GET_ENDPOINT"
	};

	for (i = 0; i < sizeof(cmd_arr) / sizeof(int); i++) {
		if (cmd_arr[i] == cmd) {
			printk("Command name : %s\n", cmd_str[i]);
			return;
		}
	}

	printk("Unknown command\n");
}
#endif

//#ifdef REG_DEBUG
void dump_aic_regs(const char *str)
{
	char *regname[] = {"aicfr","aiccr","aiccr1","aiccr2","i2scr","aicsr","acsr","i2ssr",
			   "accar", "accdr", "acsar", "acsdr", "i2sdiv", "aicdr"};
	int i;
	unsigned int addr;

	printk("AIC regs dump, %s\n", str);
	for (i = 0; i <= 0x34; i += 4) {
		addr = AIC_BASE + i;
		printk("%s\t0x%08x -> 0x%08x\n", regname[i/4], addr, *(unsigned int *)addr);
	}
}
//#endif

#ifdef BUF_DEBUG
void dump_buf(char *buf, int dump_len, int bytes_in_line)
{
	int i;
	printk("Buffer 0x%p:\n", buf);
	for (i = 0; i < dump_len; i++) {
		printk("%02x ", (unsigned char)buf[i]);
		if ((i+1) % bytes_in_line == 0) {
			printk("\n");
		}
	}
	printk("\n");
}
#endif

#ifdef Q_DEBUG
void dump_node(audio_node *node, const char *str)
{
	if (!node || !str) {
		printk("DUMP_NODE: detected argument is NULL\n");
		return;
	}

	printk("%s: addr(0x%08x) id=%d, pBuf=0x%08x, start=0x%08x, end=0x%08x, phyaddr=0x%08x\n",
	       str, (unsigned int)node, node->pBufID, node->pBuf, node->start, node->end, node->phyaddr);
}

void dump_list(audio_head *head)
{
	audio_node *tmp;
	struct list_head *p, *n;

	BUG_ON(!head);

	printk("--------\nAudio head info: fact = %d, datasize = %d, listsize = %d\n",
	       head->fact, head->datasize, head->listsize);

	printk("free q:\n");
	list_for_each_safe(p, n, &head->free) {
		tmp = list_entry(p, audio_node, list);
		DUMP_NODE(tmp, "fQ");
	}
	printk("use q:\n");
	list_for_each_safe(p, n, &head->use) {
		tmp = list_entry(p, audio_node, list);
		DUMP_NODE(tmp, "uQ");
	}
	printk("--------\n");
}
#endif

//----------------------------------------------------------------
// audio node operater
// int init_audio_node(unsigned int **memory, unsigned int pagesize, unsigned int count)
// void deinit_audio_node(unsigned int **memory)
// static inline audio_node *get_audio_freenode(unsigned int *mem)
// static inline void put_audio_usenode(unsigned int *mem, audio_node *node)
// static inline audio_node *get_audio_usenode(unsigned int *mem)
// static inline void put_audio_freenode(unsigned int *mem, audio_node *node)
// static inline int get_audio_freenodecount(unsigned int *mem)
//
//----------------------------------------------------------------

void deinit_audio_node(unsigned int **memory)
{
	audio_head	*phead;
	unsigned int	fact;

	phead = (audio_head *)*memory;
	fact = phead->fact;
	free_pages((unsigned long)*memory, fact);
	*memory = NULL;
}

int init_audio_node(unsigned int **memory, unsigned int pagesize, unsigned int count)
{
	unsigned int	fact;
	audio_node	*pbuff;
	audio_head	*phead;
	unsigned int	*mem;
	struct list_head *audio_wfree;
	struct list_head *audio_wuse;
	int	memsize;
	int	datasize;
	int	headlistsize;
	int	i;

	ENTER();

	// Alloc memory first, to avail fail
	datasize	= ALIGN_PAGE_SIZE(pagesize * count);
	headlistsize	= ALIGN_PAGE_SIZE(count * sizeof(audio_node) + sizeof(audio_head));
	memsize		= headlistsize + datasize;
	fact		= get_order(memsize);

	mem = (unsigned int *)__get_free_pages(GFP_KERNEL | GFP_DMA, fact);
	if (mem == NULL) {
		printk("JZ I2S: Memory allocation failed in function init_audio_node!\n");
		return 0;
	}

	DPRINT("Mem alloc finish! memsize = %x, fact = %d, mem = 0x%08x\n",
	       memsize, fact, (unsigned int)mem);

	// Free old buffer
	if (*memory) {
		phead	= (audio_head *)*memory;
		fact	= phead->fact;
		free_pages((unsigned long)*memory, fact);
		*memory	= NULL;
	}
	*memory = mem;

/*
	datasize	= ALIGN_PAGE_SIZE(pagesize * count);
	headlistsize	= ALIGN_PAGE_SIZE(count * sizeof(audio_node) + sizeof(audio_head)); //8byte is save head data
	memsize		= headlistsize + datasize;

	fact = get_order(memsize);
*/

	// Update list head
	phead		= (audio_head *)*memory;
	phead->fact	= fact;
	phead->listsize	= headlistsize;
	phead->datasize	= datasize;

	audio_wuse	= &(phead->use);
	audio_wfree	= &(phead->free);
	INIT_LIST_HEAD(audio_wuse);
	INIT_LIST_HEAD(audio_wfree);

	pbuff = (audio_node *)((unsigned int)*memory + sizeof(audio_head));
	for (i = 0; i < count; i++) {
		pbuff->pBuf	= (unsigned int)*memory + headlistsize + pagesize * i;
		pbuff->phyaddr	= (unsigned int)virt_to_phys((void *)pbuff->pBuf);
		pbuff->start	= 0;
		pbuff->end	= 0;
#ifdef Q_DEBUG
		pbuff->pBufID	= i;
#endif
		DPRINT_Q("audio_note buffer[%d] = %x\n", i, (unsigned int)pbuff->pBuf);
		list_add(&pbuff->list, audio_wfree);
		pbuff++;
	}

	DUMP_LIST(phead);

	LEAVE();
	return 1;
}

#define is_null_free_audio_node(mem)				\
({								\
	audio_head *phead = (audio_head *)(mem);		\
	struct list_head *pfree = &(phead->pfree);		\
	(pfree->next == pfree);					\
})

#define is_null_use_audio_node(mem)				\
({								\
	audio_head *phead = (audio_head *)mem;			\
	struct list_head *puse = &(phead->use);			\
	(puse->next == puse);					\
})

//static unsigned int putid = 0, getid = 0;

static inline audio_node *get_audio_freenode(unsigned int *mem)
{
	audio_head	 *phead;
	audio_node	 *node = NULL;
	struct list_head *pfree;
	struct list_head *curnode;

	phead	= (audio_head *)mem;
	pfree	= &(phead->free);
	curnode	= pfree->next;

	if (curnode != pfree) {
		node = THIS_AUDIO_NODE(curnode);
		node->start = 0;
		node->end = 0;
		list_del(curnode);
	}
	return node;
}

static inline void put_audio_usenode(unsigned int *mem, audio_node *node)
{
	audio_head *phead = (audio_head *)mem;
	struct list_head *puse = &(phead->use);
	struct list_head *curnode = &(node->list);

	list_add_tail(curnode, puse);
}

static inline audio_node *get_audio_usenode(unsigned int *mem)
{
	audio_head	 *phead;
	audio_node	 *node = NULL;
	struct list_head *curnode;
	struct list_head *puse;

	phead	= (audio_head *)mem;
	puse	=  &(phead->use);
	curnode	= puse->next;

	if (curnode != puse) {
		node = THIS_AUDIO_NODE(curnode);
		list_del(curnode);
	}
	return node;
}

static inline void put_audio_freenode(unsigned int *mem, audio_node *node)
{
	audio_head *phead = (audio_head *)mem;
	struct list_head *pfree = &(phead->free);
	struct list_head *curnode = &(node->list);

	list_add_tail(curnode, pfree);
}

static inline int get_audio_freenodecount(unsigned int *mem)
{
	struct list_head *pfree;
	struct list_head *plist;
	audio_head *phead;
	int count = 0;

	phead = (audio_head *)mem;
	pfree =  &(phead->free);
	plist = pfree;
	while (plist->next != pfree) {
		count++;
		plist = plist->next;
	}
	return count;
}

#if 1
static struct timer_list debug_timer;
static int dbg_timer_should_stop = 0;

static void debug_func(unsigned long arg) {
	dump_dlv_regs(__func__);	
	dump_aic_regs(__func__);
	dump_jz_dma_channel(6);
	dump_jz_dma_channel(6);

	if (!dbg_timer_should_stop) {
		debug_timer.expires = jiffies + HZ;
		add_timer(&debug_timer);
	}
}

static void start_debug_timer() {
	static audio_timer_inited = 0;

	if (!audio_timer_inited) {
		init_timer(&debug_timer);		
		debug_timer.function = debug_func;
		debug_timer.data = 0x0;
	}

	debug_timer.expires = jiffies + HZ;

	dbg_timer_should_stop = 0;
	add_timer(&debug_timer);
}

static void stop_debug_timer() {
	dbg_timer_should_stop = 1;
	del_timer_sync(&debug_timer);
}
#endif

//--------------------------------------------------------------------
// end audio node operater
//--------------------------------------------------------------------

//--------------------------------------------------------------------
// static irqreturn_t jz_i2s_dma_irq (int irq, void *dev_id)
// int init_audio_recorddma(audio_pipe *endpoint)
// int init_audio_replaydma(audio_pipe *endpoint)
// int init_audio_audiodma(audio_pipe *endpoint, int mode)
// void config_dma_trans_mode(spinlock_t lock, audio_dma_type* dma, int mode)
// static inline int audio_trystart_dma_node(audio_dma_type* dma, audio_node *node)
// static inline int audio_trystart_dma_node(audio_dma_type* dma, audio_node *node)
// static inline void audio_stop_dma_node(audio_dma_type* dma)

static irqreturn_t jz_i2s_dma_irq (int irq, void *dev_id)
{
	audio_pipe * endpoint = (audio_pipe *) dev_id;
	int dma_chan = endpoint->dma.ch;
	int dma_state = REG_DMAC_DCCSR(dma_chan);
	int err = 0;

	ENTER();

	//printk("===>enter %s:%d\n", __func__, __LINE__);
	stop_debug_timer();

	REG_DMAC_DCCSR(dma_chan) = 0;

	DPRINT_IRQ("!!!! endpoint direct = %s \n",(endpoint == &out_endpoint) ? "out" : "in");

	if (dma_state & DMAC_DCCSR_HLT) {
		err = 0;
		DPRINT_IRQ("!!!! DMA HALT\n");
	}
	if (dma_state & DMAC_DCCSR_AR) {
		err = 1;
		DPRINT_IRQ("!!!! DMA ADDR ERROR\n");
	}
	if (dma_state & DMAC_DCCSR_CT) {
		DPRINT_IRQ("!!!! DMA descriptor finish\n");
	}
	/*
	if (dma_state & DMA_DCCSR_TT) {

	}
	*/
	if (err == 0) {
		//printk("schedule_work++++ %x %x\n", endpoint,&(endpoint->work));
		//schedule_work(&(endpoint->work));
		//printk("schedule_work----\n");
		endpoint->handle(endpoint);
	} else {
		DPRINT_IRQ("!!!! ??? unknown !!!\n");
	}

	LEAVE();

	return IRQ_HANDLED;
}

static int jz_request_aic_dma(int dev_id, const char *dev_str,
			      irqreturn_t (*irqhandler)(int, void *),
			      unsigned long irqflags, void *irq_dev_id)
{
	struct jz_dma_chan *chan;
	int i, ret;

	if (dev_id == DMA_ID_AIC_TX) {
		i = DMA_TX_CHAN;
		if (jz_dma_table[i].dev_id != DMA_ID_AIC_TX) {
			BUG_ON(1);
		}
	} else if (dev_id == DMA_ID_AIC_RX) {
		i = DMA_RX_CHAN;
		if (jz_dma_table[i].dev_id != DMA_ID_AIC_RX) {
			BUG_ON(1);
		}
	} else {
		BUG_ON(1);
	}

	/* we got channel */
	chan = &jz_dma_table[i];

	if (irqhandler) {
		chan->irq = IRQ_DMA_0 + i;
		chan->irq_dev = irq_dev_id;
		if ((ret = request_irq(chan->irq, irqhandler, irqflags,
				       dev_str, chan->irq_dev))) {
			chan->irq = -1;
			chan->irq_dev = NULL;
			return ret;
		}
	} else {
		chan->irq = -1;
		chan->irq_dev = NULL;
	}
/*
	printk("\n@@@@ %s:%d chan index = %d, chan.irq = %d\n\n",
	       __FUNCTION__, __LINE__, i, chan->irq);
*/
	chan->io	= i;
	chan->dev_id	= dev_id;
	chan->dev_str	= dev_str;
	chan->fifo_addr	= CPHYSADDR(AIC_DR);

	switch (dev_id) {
	case DMA_ID_AIC_TX:
		chan->mode	= DMA_AIC_TX_CMD_UNPACK | DMA_MODE_WRITE;
		chan->source	= DMAC_DRSR_RS_AICOUT;
		break;
	case DMA_ID_AIC_RX:
		chan->mode	= DMA_32BIT_RX_CMD | DMA_MODE_READ;
		chan->source	= DMAC_DRSR_RS_AICIN;
		break;
	default:
		printk("JZ AIC: %s:%d, need fix !!!\n", __FUNCTION__, __LINE__);
		BUG_ON(1);
	}

	// Open AIC_TX and AIC_RX
#ifdef CONFIG_SOC_JZ4760B
	REG_DMAC_DMACKS(1) = 1 << (DMA_RX_CHAN - HALF_DMA_NUM) | 1 << (DMA_TX_CHAN - HALF_DMA_NUM);
#else
	REG_DMAC_DMACKE(1) = 1 << (DMA_RX_CHAN - HALF_DMA_NUM) | 1 << (DMA_TX_CHAN - HALF_DMA_NUM);
#endif
	return i;
}

static int init_audio_recorddma(audio_pipe *endpoint)
{
	int ch = 0;

	ENTER();
	if ((ch = jz_request_aic_dma(DMA_ID_I2S_RX, "audio adc", jz_i2s_dma_irq, IRQF_DISABLED, endpoint)) < 0) {
		printk(KERN_ERR "%s: can't reqeust DMA DAC channel.\n", __FUNCTION__);
		return -1;
	}
	REG_DMAC_DMACR(ch / HALF_DMA_NUM) |= 1;
	REG_DMAC_DCMD(ch) =  DMAC_DCMD_DAI | DMAC_DCMD_SWDH_32 | DMAC_DCMD_TIE;
	REG_DMAC_DRSR(ch) =  DMAC_DRSR_RS_AICIN;
	REG_DMAC_DSAR(ch) =  (unsigned int)CPHYSADDR(AIC_DR);

	endpoint->dma.ch = ch;
	endpoint->dma.trans_addr = (unsigned int *)DMAC_DTAR(ch);
	endpoint->dma.trans_count = (unsigned int *)DMAC_DTCR(ch);
	endpoint->dma.trans_mode = (unsigned int *)DMAC_DCMD(ch);
	endpoint->dma.data_addr = (unsigned int *)DMAC_DSAR(ch);

	endpoint->dma.rw = 0;

	LEAVE();
	return ch;
}

static int init_audio_replaydma(audio_pipe *endpoint)
{
	int ch = 0;
	if ((ch = jz_request_aic_dma(DMA_ID_I2S_TX,"audio dac", jz_i2s_dma_irq, IRQF_DISABLED, endpoint)) < 0) {
		printk(KERN_ERR "%s: can't reqeust DMA DAC channel.\n", __FUNCTION__);
		return -1;
	}

	REG_DMAC_DMACR(ch / HALF_DMA_NUM) |= 1;
	REG_DMAC_DCMD(ch) =  DMAC_DCMD_SAI | DMAC_DCMD_DWDH_32 | DMAC_DCMD_TIE;

	//printk("$$$$ before set --- REG_DMAC_DRSR(ch) = 0x%08x\n", REG_DMAC_DRSR(ch));
	REG_DMAC_DRSR(ch) =  DMAC_DRSR_RS_AICOUT;

	//printk("$$$$ ch = %d, DMAC_DRSR = 0x%08x, set 0x%08x, after set -- 0x%08x\n",
	//       ch, DMAC_DRSR(ch), DMAC_DRSR_RS_AICOUT, REG_DMAC_DRSR(ch));

	*(unsigned int *)0xb342010c = 0x18;

	//printk("$$$$ after force set --- REG_DMAC_DRSR(ch) = 0x%08x\n", REG_DMAC_DRSR(ch));

	REG_DMAC_DTAR(ch) =  (unsigned int)CPHYSADDR(AIC_DR);

	endpoint->dma.ch = ch;
	endpoint->dma.trans_addr = (unsigned int *)DMAC_DSAR(ch);
	endpoint->dma.trans_count = (unsigned int *)DMAC_DTCR(ch);
	endpoint->dma.trans_mode = (unsigned int *)DMAC_DCMD(ch);
	endpoint->dma.data_addr = (unsigned int *)DMAC_DTAR(ch);
	endpoint->dma.rw = 1;
	return ch;
}

static int init_audio_audiodma(audio_pipe *endpoint, int mode)
{
	if (mode == CODEC_RMODE) {
		return init_audio_recorddma(endpoint);
	}

	if (mode == CODEC_WMODE) {
		return init_audio_replaydma(endpoint);
	}

	return -1;
}

static void config_dma_trans_mode(spinlock_t lock, audio_dma_type* dma, int sound_data_width)
{
	unsigned int	curmode;
	unsigned long	flags;

	ENTER();
	AUDIO_LOCK(lock, flags);
	curmode = *dma->trans_mode;

	if (dma->rw) {
		curmode &= ~(DMAC_DCMD_DWDH_MASK | DMAC_DCMD_DS_MASK);
		switch(sound_data_width) {
		case 8:
			*dma->trans_mode = (curmode | DMAC_DCMD_DWDH_8 | DMAC_DCMD_DS_16BYTE);
			dma->onetrans_bit = 16 * 8;
			break;
		case 16:
			*dma->trans_mode = (curmode | DMAC_DCMD_DWDH_16 | DMAC_DCMD_DS_16BYTE);
			dma->onetrans_bit = 16 * 8;
			break;
		case 17 ... 32:
			*dma->trans_mode = (curmode | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BYTE);
			dma->onetrans_bit = 32 * 8;
			break;
		default:
			printk("JZ I2S: Unkown DMA mode(sound data width) %d\n", sound_data_width);
			break;
		}
	} else {
		curmode &= ~(DMAC_DCMD_SWDH_MASK | DMAC_DCMD_DS_MASK);
		switch(sound_data_width) {
		case 8:
			*dma->trans_mode = (curmode | DMAC_DCMD_SWDH_8 | DMAC_DCMD_DS_16BYTE);
			dma->onetrans_bit = 16 * 8;
			break;
		case 16:
			*dma->trans_mode = (curmode | DMAC_DCMD_SWDH_16 | DMAC_DCMD_DS_16BYTE);
			dma->onetrans_bit = 16 * 8;
			break;
		case 17 ... 32:
			*dma->trans_mode = (curmode | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DS_32BYTE);
			dma->onetrans_bit = 32 * 8;
			break;
		default:
			printk("JZ I2S: Unkown DMA mode(sound data width) %d\n", sound_data_width);
			break;
		}
	}

	AUDIO_UNLOCK(lock, flags);
	DUMP_DMA(dma->ch, __FUNCTION__);
	DPRINT_DMA("dma_trans = %d\n", dma->onetrans_bit);
	LEAVE();
}

#define aic_enable_transmit()					\
do {								\
	int dat = REG_AIC_CR;					\
	dat |= (AIC_CR_TDMS | AIC_CR_ERPL);			\
	REG_AIC_CR = dat;					\
} while (0)

#define aic_disable_transmit()					\
do {								\
	int dat = REG_AIC_CR;					\
	dat &= ~(AIC_CR_TDMS | AIC_CR_ERPL);			\
	REG_AIC_CR = dat;					\
} while (0)

static inline int audio_trystart_dma_node(audio_dma_type* dma, audio_node *node)
{
	int start = 0;

	ENTER();
	//printk("===>enter %s:%d\n", __func__, __LINE__);

	if ((REG_DMAC_DCCSR(dma->ch) & DMAC_DCCSR_EN) == 0) {
		int count = node->end - node->start;
		*(dma->trans_addr) = node->phyaddr;
		*(dma->data_addr) =  (unsigned int)CPHYSADDR(AIC_DR);
		*(dma->trans_count) = count * 8 / dma->onetrans_bit;
		REG_DMAC_DCCSR(dma->ch) = DMAC_DCCSR_NDES | DMAC_DCCSR_EN;
		//printk("virt = 0x%08x phy = 0x%08x, dma->onetrans_bit = 0x%x\n",
			   //node->pBuf, node->phyaddr, dma->onetrans_bit);

		DUMP_CODEC_REGS(__FUNCTION__);
		DUMP_AIC_REGS(__FUNCTION__);
		start = 1;
		start_debug_timer();
	}

	DUMP_DMA(dma->ch, "audio_trystart_dma_node -----------");

	LEAVE();
	return start;
}

static inline void audio_stop_dma_node(audio_dma_type* dma)
{
	REG_DMAC_DCCSR(dma->ch) = 0;
}

/* Never be used, fix me ???
static inline int recalculate_fifowidth(short channels, short fmt)
{
	int bit = 16;

	if (fmt <= 8) {
		bit = 8;
	} else if (fmt > 16) {
		bit = 32;
	} else {
		bit = 16;
	}

	return 	bit *= channels;
}
*/
#define I2S_FIFO_DEPTH 32

static inline void set_controller_triger(struct jz_i2s_controller_info *controller,
					 audio_pipe *endpoint, short channels, short format)
{
	int sound_data_width = 0;

	ENTER();

//	printk("%%%% format = %d\n", format);

	switch (format) {
	case AFMT_U8:
	case AFMT_S8:
		sound_data_width = 8;
		break;
	case AFMT_S16_LE:
	case AFMT_S16_BE:
		sound_data_width = 16;
		break;
	default:
		printk("JZ I2S: Unkown sound format %d\n", format);
		return ;
	}

	config_dma_trans_mode(endpoint->lock,&(endpoint->dma), sound_data_width);
	if (endpoint == &out_endpoint) {
		if ((I2S_FIFO_DEPTH - endpoint->dma.onetrans_bit / sound_data_width) >= 30) {
			__i2s_set_transmit_trigger(14);
		} else {
			__i2s_set_transmit_trigger((I2S_FIFO_DEPTH - endpoint->dma.onetrans_bit / sound_data_width) / 2);
		}
	}
	if (endpoint == &in_endpoint) {
		__i2s_set_receive_trigger((endpoint->dma.onetrans_bit / sound_data_width) / 2);
	}

	LEAVE();
}

//-------------------------------------------------------------------
/*
  int trystart_endpoint_out(audio_pipe *endpoint, audio_node *node);
  int trystart_endpoint_in(audio_pipe *endpoint, audio_node *node);
  note: this two function isn't protected;
 */
static inline int trystart_endpoint_out(struct jz_i2s_controller_info *controller, audio_node *node)
{
	audio_pipe *endpoint = controller->pout_endpoint;
	int start = 0;

	ENTER();

	start = audio_trystart_dma_node(&(endpoint->dma), node);
	if (start) {
		endpoint->trans_state |= PIPE_TRANS;
		endpoint->savenode = node;
		aic_enable_transmit();
		DUMP_AIC_REGS(__FUNCTION__);
		DUMP_CODEC_REGS(__FUNCTION__);
	}

	LEAVE();
	return start;
}

static inline int trystart_endpoint_in(struct jz_i2s_controller_info *controller, audio_node *node)
{
	audio_pipe *endpoint = controller->pin_endpoint;
	int start = 0;

	ENTER();
	dma_cache_wback_inv((unsigned long)node->pBuf, endpoint->fragsize);
	start = audio_trystart_dma_node(&(endpoint->dma), node);
	if (start) {
		endpoint->trans_state |= PIPE_TRANS;
		endpoint->savenode = node;
		__i2s_enable_receive_dma();
		__i2s_enable_record();
		DUMP_AIC_REGS(__FUNCTION__);
		DUMP_CODEC_REGS(__FUNCTION__);
	}
	LEAVE();
	return start;
}

int audio_get_endpoint_freesize(audio_pipe *endpoint, audio_buf_info *info)
{
	int count;
	unsigned long flags;

	AUDIO_LOCK(endpoint->lock, flags);
	count = get_audio_freenodecount(endpoint->mem);
	AUDIO_UNLOCK(endpoint->lock, flags);
	info->fragments = count;
	info->fragstotal = endpoint->fragstotal;
	info->fragsize = endpoint->fragsize;
	info->bytes = count * endpoint->fragsize;
	return info->bytes;
}

void audio_clear_endpoint(audio_pipe *endpoint)
{
	audio_node *pusenode;
	unsigned long flags;

	ENTER();
	AUDIO_LOCK(endpoint->lock, flags);
	while (!is_null_use_audio_node(endpoint->mem)) {
		pusenode = get_audio_usenode(endpoint->mem);
		if (pusenode) {
			put_audio_freenode(endpoint->mem, pusenode);
		}
	}
	AUDIO_UNLOCK(endpoint->lock, flags);
	LEAVE();
}

void audio_sync_endpoint(audio_pipe *endpoint)
{
	int isnull = 1;
	unsigned long flags;

	ENTER();

	do {
		AUDIO_LOCK(endpoint->lock, flags);
		isnull = is_null_use_audio_node(endpoint->mem);
		AUDIO_UNLOCK(endpoint->lock, flags);
		if (!isnull) {
			//printk("&&&& audio_sync_endpoint\n");
			schedule_timeout(1);
		}
	} while (!isnull);

	LEAVE();
}

void audio_close_endpoint(audio_pipe *endpoint, int mode)
{
	int is_use_list_null = 1, trans = 0;
	unsigned long flags;

	ENTER();

	AUDIO_LOCK(endpoint->lock, flags);
	is_use_list_null = is_null_use_audio_node(endpoint->mem);
	trans = endpoint->trans_state & PIPE_TRANS;
	AUDIO_UNLOCK(endpoint->lock, flags);

	if (is_use_list_null) {
		// Wait savenode trans complete
		while (trans) {
			AUDIO_LOCK(endpoint->lock, flags);
			trans = endpoint->trans_state & PIPE_TRANS;
			AUDIO_UNLOCK(endpoint->lock, flags);
			DPRINT("waiting savenode\n");
			if (trans) {
				schedule_timeout(10);
			}
		}

		/* In replay mode, savenode must been put into free list after trans completed,
		 * so we don't care it in this condition.
		 * But in record mode, savenode must been put into use list after trans completed,
		 * so we have to ignore the incomming data and move it to free list forcely.
		 */
		if (endpoint == &out_endpoint) {
			goto _L_AUDIO_CLOSE_EP_RET;
		}
	}

	// NOMAL_STOP routine of replay mode
	if (mode == NOMAL_STOP) {
		BUG_ON(endpoint != &out_endpoint);

		// Wait use list free
		audio_sync_endpoint(endpoint);
		// wait savenode trans finish
		while (trans) {
			AUDIO_LOCK(endpoint->lock, flags);
			trans = endpoint->trans_state & PIPE_TRANS;
			AUDIO_UNLOCK(endpoint->lock, flags);
			//printk("waiting savenode\n");
			if (trans) {
				schedule_timeout(10);
			}
		}

		AUDIO_LOCK(endpoint->lock, flags);
		DUMP_LIST((audio_head *)endpoint->mem);
		DUMP_NODE(endpoint->savenode, "SN");
		AUDIO_UNLOCK(endpoint->lock, flags);
	} else {
		// FORCE_STOP routine, both replay and record mode could run
		audio_node *pusenode;

		// Shutdown DMA immediately and clear lists forcely.
		AUDIO_LOCK(endpoint->lock, flags);

		endpoint->trans_state &= ~PIPE_TRANS;
		audio_stop_dma_node(&endpoint->dma);

		DUMP_LIST((audio_head *)endpoint->mem);
		DUMP_NODE(endpoint->savenode, "SN");
		DPRINT_Q("---------------------------------\n");

		while (!is_null_use_audio_node(endpoint->mem)) {
			pusenode = get_audio_usenode(endpoint->mem);
			if (pusenode) {
				put_audio_freenode(endpoint->mem, pusenode);
			}
		}

		DUMP_LIST((audio_head *)endpoint->mem);
		DUMP_NODE(endpoint->savenode, "SN");
		DPRINT_Q("---------------------------------\n");

		if (endpoint->savenode) {
			DPRINT_Q("handle savenode : 0x%08x\n", (unsigned int)endpoint->savenode);
			DUMP_NODE(endpoint->savenode, "SN");
			put_audio_freenode(endpoint->mem, endpoint->savenode);

			DPRINT_Q("savenode->list->next = 0x%08x, savenode->list->prev = 0x%08x\n",
			       (unsigned int)endpoint->savenode->list.next,
			       (unsigned int)endpoint->savenode->list.prev);

			endpoint->savenode = NULL;
		}

		DUMP_LIST((audio_head *)endpoint->mem);
		DUMP_NODE(endpoint->savenode, "SN");

		AUDIO_UNLOCK(endpoint->lock, flags);
	}

_L_AUDIO_CLOSE_EP_RET:
	LEAVE();
}

int audio_resizemem_endpoint(audio_pipe *endpoint, unsigned int pagesize, unsigned int count)
{
	int ret;
	if((endpoint->fragsize == pagesize)&&(endpoint->fragstotal == count))
	return 1;//debug by wll
	ret = init_audio_node(&endpoint->mem, pagesize, count);
	if (ret) {
		endpoint->fragsize = pagesize;
		endpoint->fragstotal = count;
	}
	return ret;
}

static void handle_in_endpoint_work(audio_pipe *endpoint)
{
	audio_node	*node;
	unsigned long	flags;

	ENTER();

	AUDIO_LOCK(endpoint->lock, flags);
	if (endpoint->savenode) {
		DPRINT_Q("\nIIII RRRR QQQQ >>>>\n");
		DUMP_LIST((audio_head *)endpoint->mem);
		DUMP_NODE(endpoint->savenode, "IRQSN");
		DPRINT_Q("IIII RRRR QQQQ <<<<\n\n");

		DPRINT_IRQ("%s endpoint->savenode = 0x%p\n", __FUNCTION__, endpoint->savenode);
		put_audio_usenode(endpoint->mem, endpoint->savenode);

		endpoint->savenode = NULL;
		DUMP_BUF((char *)(endpoint->savenode->pBuf + endpoint->savenode->start), 64, 32);

		if (!(endpoint->is_non_block)) {
			endpoint->avialable_couter++;
			wake_up_interruptible(&endpoint->q_full);
		}
	}

	node = get_audio_freenode(endpoint->mem);
	if (node) {
		int start;
		node->end = endpoint->fragsize;
		dma_cache_wback_inv((unsigned long)node->pBuf, endpoint->fragsize);
		start = audio_trystart_dma_node(&(endpoint->dma), node);
		if (start == 0) {
			put_audio_freenode(endpoint->mem, node);
		} else {
			endpoint->savenode = node;
		}
	} else {
		endpoint->trans_state &= ~PIPE_TRANS;
		__i2s_disable_receive_dma();
		__i2s_disable_record();
		DPRINT_IRQ("!!!! Stop AIC record !\n");
	}

	DPRINT_Q("\nIIII RRRR QQQQ >>>>\n");
	DUMP_LIST((audio_head *)endpoint->mem);
	DUMP_NODE(endpoint->savenode, "SN");
	DPRINT_Q("IIII RRRR QQQQ <<<<\n\n");

	AUDIO_UNLOCK(endpoint->lock, flags);

	LEAVE();
}

/*
static void audio_in_endpoint_work(struct work_struct *work)
{
	audio_pipe *endpoint = &in_endpoint;
	handle_in_endpoint_work(endpoint);
}
*/

static void handle_out_endpoint_work(audio_pipe *endpoint)
{
	audio_node *node;
	unsigned long flags;

	ENTER();

	AUDIO_LOCK(endpoint->lock, flags);
	DPRINT_IRQ("%s endpoint->savenode = 0x%08x\n", __FUNCTION__, (unsigned int)endpoint->savenode);

	if (endpoint->savenode) {
		put_audio_freenode(endpoint->mem, endpoint->savenode);
		DPRINT_IRQ("put_audio_freenode\n");
		endpoint->savenode = NULL;

		if (!(endpoint->is_non_block)) {
			wake_up_interruptible(&endpoint->q_full);
			endpoint->avialable_couter++;
		}
	}

	node = get_audio_usenode(endpoint->mem);
	if (node) {
		int start;
		start = audio_trystart_dma_node(&(endpoint->dma), node);
		if (start == 0) {
			printk("audio_out_endpoint_work audio_trystart_dma_node error!\n");
		} else {
			endpoint->savenode = node;
			DPRINT_DMA("restart dma!\n");
		}
	} else {
		endpoint->trans_state &= ~PIPE_TRANS;
		aic_disable_transmit();
		DPRINT_IRQ("!!!! Stop AIC !\n");
	}

	AUDIO_UNLOCK(endpoint->lock, flags);
	LEAVE();
}

/*
static void audio_out_endpoint_work(struct work_struct *work)
{
	audio_pipe *endpoint = &out_endpoint;
	handle_out_endpoint_work(endpoint);
}
*/

void audio_init_endpoint(audio_pipe *endpoint, unsigned int pagesize, unsigned int count)
{
	audio_resizemem_endpoint(endpoint, pagesize, count);
	spin_lock_init(&endpoint->lock);
	init_waitqueue_head(&endpoint->q_full);
	endpoint->avialable_couter = 0;
	endpoint->filter = NULL;

	if (endpoint == &in_endpoint) {
		init_audio_audiodma(endpoint, CODEC_RMODE);
		// INIT_WORK(&endpoint->work, audio_in_endpoint_work);
		endpoint->handle = handle_in_endpoint_work;
	}
	if (endpoint == &out_endpoint) {
		init_audio_audiodma(endpoint, CODEC_WMODE);
		// INIT_WORK(&endpoint->work, audio_out_endpoint_work);
		endpoint->handle = handle_out_endpoint_work;
	}
}

void audio_deinit_endpoint(audio_pipe *endpoint)
{
	audio_close_endpoint(endpoint, FORCE_STOP);
	deinit_audio_node(&endpoint->mem);
}

void register_jz_codecs(void *func)
{
	int i;

	ENTER();

	for (i = 0; i < NR_I2S; i++) {
		if (the_codecs[i].codecs_ioctrl == 0) {
			printk("register codec %x\n",(unsigned int)func);
			the_codecs[i].id = i;
			the_codecs[i].codecs_ioctrl = func;
			init_MUTEX(&(the_codecs[i].i2s_sem));
			break;
		}
	}

	LEAVE();
}

#define codec_ioctrl(codec, cmd, args) ({			\
	int result;						\
	down(&(codec)->i2s_sem);				\
	result = (codec)->codecs_ioctrl((codec), (cmd), (args));\
	up(&(codec)->i2s_sem);					\
	result;							\
})

static int jz_i2s_open_mixdev(struct inode *inode, struct file *file)
{
	int i;
	int minor = MINOR(inode->i_rdev);

	ENTER();

	for (i = 0; i < NR_I2S; i++) {
		if (the_codecs[i].dev_mixer == minor) {
			goto match;
		}
	}
match:
	file->private_data = &the_codecs[i];

	LEAVE();
	return 0;
}

/*
 * Debug entry for Android
 */
static int jz_i2s_write_mixdev(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct i2s_codec *codec = (struct i2s_codec *)file->private_data;
	char	buf_byte = 0;
	char	argument[16];
	int	val;

	if (copy_from_user((void *)&buf_byte, buffer, 1)) {
		printk("JZ MIX: copy_from_user failed !\n");
		return -EFAULT;
	}

	switch (buf_byte) {
	case '1':
		dump_dlv_regs("jz_i2s_write_mixdev --- debug routine");
		dump_aic_regs("");
		break;
	case '2':
		printk("dlv_set_replay\n");
		codec_ioctrl(codec, CODEC_SET_REPLAY, 0);
		break;
	case '3':
		printk("dlv_set_record\n");
		codec_ioctrl(codec, CODEC_SET_RECORD, 0);
		break;
	case '4':
		if (codec_ioctrl(codec, CODEC_SET_RECORD_DATA_WIDTH, 16) >= 0) {
			printk("Set data width : 16\n");
		} else {
			printk("Could not set data width\n");
		}
		break;
	case '5':
		if (copy_from_user((void *)&argument, buffer + 1, 3)) {
			printk("JZ MIX: copy_from_user failed !\n");
			return -EFAULT;
		}
		if (argument[0] >= '0' && argument[0] <= '9'
		    && argument [1] >= '0' && argument[1] <= '9'
		    && argument [2] >= '0' && argument[2] <= '9') {

			val = (argument[0] - '0') * 100 + (argument[1] - '0') * 10 + argument[2] - '0';

			printk("JZ MIX: set volume (%d)\n", val);
			codec_ioctrl(codec, CODEC_SET_VOLUME, val);
		} else {
			printk("JZ MIX: invalid argument for set volume\n");
		}
		break;
	}

	return count;
}

/*
 * Handle IOCTL request on /dev/mixer
 *
 * Support OSS IOCTL interfaces for /dev/mixer
 * Support IOCTL interfaces for /dev/mixer defined in include/msm_audio.h
 */
static int jz_i2s_ioctl_mixdev(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2s_codec *codec = (struct i2s_codec *)file->private_data;
	long	val = 0;
	int	ret, rc = 0;

#ifdef CONFIG_JZ_EBOOK_HARD
//	printk("DEBUG: %s, %d\n", __FUNCTION__, __LINE__);
	/* add by qinbh, control the aic clock */
	int reg = REG_CPM_CLKGR;
//	__cpm_start_aic1();
#endif


	ENTER();

	DPRINT_IOC("[mixer IOCTL]~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	DPRINT_IOC(" mixer IOCTL %s cmd = 0x%08x, arg = %lu\n", __FUNCTION__, cmd, arg);
	DPRINT_MIXER_IOC_CMD(cmd);
	DPRINT_IOC("[mixer IOCTL]~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

	// struct jz_i2s_controller_info *controller = (struct jz_i2s_controller_info *) file->private_data;

	switch (cmd) {

	/*
	 * OSS IOCTL commands for /dev/mixer
	 */
	case SOUND_MIXER_INFO:
	{
		mixer_info info;
		codec_ioctrl(codec, CODEC_GET_MIXER_INFO, (unsigned int)&info);
		info.modify_counter = audio_mix_modcnt;
		return copy_to_user((void *)arg, &info, sizeof(info));
	}
	case SOUND_OLD_MIXER_INFO:
	{
		_old_mixer_info info;
		codec_ioctrl(codec, CODEC_GET_MIXER_OLD_INFO, (unsigned int)&info);
		return copy_to_user((void *)arg, &info, sizeof(info));
	}

	case SOUND_MIXER_READ_STEREODEVS:
		return put_user(0, (long *) arg);
	case SOUND_MIXER_READ_CAPS:
		return put_user(SOUND_CAP_EXCL_INPUT, (long *) arg);

	case SOUND_MIXER_READ_DEVMASK:
		break;
	case SOUND_MIXER_READ_RECMASK:
		break;
	case SOUND_MIXER_READ_RECSRC:
		break;

	case SOUND_MIXER_WRITE_SPEAKER:
		ret = get_user(val, (long *) arg);
		if ((val &= 0xff) >= 100) {
			val = 100;
		}
		codec_ioctrl(codec, CODEC_SET_DIRECT_MODE, val);
		break;

	case SOUND_MIXER_WRITE_BASS:
		ret = get_user(val, (long *) arg);
		if ((val &= 0xff) >= 100) {
			val = 100;
		}
		codec->bass_gain = val;
		codec_ioctrl(codec, CODEC_SET_BASS, val);
		return 0;

	case SOUND_MIXER_READ_BASS:
		val = codec->bass_gain;
		ret = val << 8;
		val = val | ret;
		return put_user(val, (long *) arg);

	case SOUND_MIXER_WRITE_VOLUME:
		ret = get_user(val, (long *) arg);
		if ((val &= 0xff) >= 100) {
			val = 100;
		}

		DPRINT_IOC("SOUND_MIXER_WRITE_VOLUME <- %lu\n", val);

		codec->audio_volume = val;
		codec_ioctrl(codec, CODEC_SET_VOLUME, val);
		return 0;

	case SOUND_MIXER_READ_VOLUME:
		val = codec->audio_volume;
		ret = val << 8;
		val = val | ret;
		return put_user(val, (long *) arg);

	case SOUND_MIXER_WRITE_MIC:
		ret = get_user(val, (long *) arg);
		if ((val &= 0xff) >= 100) {
			val = 100;
		}
		codec->mic_gain = val;
		codec->use_mic_line_flag = USE_MIC;
		codec_ioctrl(codec, CODEC_SET_MIC, val);
		return 0;

	case SOUND_MIXER_READ_MIC:
		val = codec->mic_gain;
		ret = val << 8;
		val = val | ret;
		return put_user(val, (long *) arg);

	case SOUND_MIXER_WRITE_LINE:
		ret = get_user(val, (long *) arg);
		if (ret) {
			return ret;
		}
		if ((val &= 0xff) >= 100) {
			val = 100;
		}
		codec->use_mic_line_flag = USE_LINEIN;
		codec->mic_gain = val;
		codec_ioctrl(codec, CODEC_SET_LINE, val);
		return 0;

	case SOUND_MIXER_READ_LINE:
		val = codec->mic_gain;
		ret = val << 8;
		val = val | ret;
		return put_user(val, (long *) arg);

	case SOUND_MIXER_WRITE_MUTE:
		get_user(codec->audiomute, (long *)arg);
		//codec_ioctrl(codec, CODEC_DAC_MUTE, codec->audiomute);
		break;

	case SOUND_MIXER_READ_MUTE:
		put_user(codec->audiomute, (long *) arg);
		break;

#if 0
	/*
	 * MSM IOCTL commands for /dev/mixer
	 */
	case SND_SET_DEVICE:
	{
		struct snd_device_config dev;
		if (copy_from_user(&dev, (void *) arg, sizeof(dev))) {
			rc = -EFAULT;
			break;
		}
		break;
	}

	case SND_SET_VOLUME:
	{
		struct snd_volume_config vol;
		if (copy_from_user(&vol, (void *) arg, sizeof(vol))) {
			return -EFAULT;
		}
		val =  vol.volume;
		if ((val &= 0xff) >= 100) {
			val = 100;
		}
		DPRINT_IOC("snd_set_volume %d %d %d\n", vol.device, vol.method, vol.volume);
		codec->audio_volume = val;
		codec_ioctrl(codec, CODEC_SET_MIC, (unsigned int)&val); ///??????????????????????????
		//error
		break;
	}

	case SND_GET_NUM_ENDPOINTS:
		if (copy_to_user((void __user*) arg, &snd->snd_epts->num, sizeof(unsigned))) {
			printk("%s: error get endpoint\n",__FUNCTION__);
			rc = -EFAULT;
		}
		val = 2;
		if (copy_to_user((void __user*) arg, &val, sizeof(unsigned))) {
			printk("%s: error get endpoint\n",__FUNCTION__);
			rc = -EFAULT;
		}

		break;
	case SND_GET_ENDPOINT:
		//rc = get_endpoint(snd, arg);
		break;
#endif

	default:
		printk("Mixer IOCTL error: %s:%d: known command: 0x%08x\n", __FUNCTION__, __LINE__, cmd);
		return -ENOSYS;
	}
	audio_mix_modcnt++;

#ifdef CONFIG_JZ_EBOOK_HARD
//	printk("DEBUG: %s, %d\n", __FUNCTION__, __LINE__);
	/* add by qinbh */
	REG_CPM_CLKGR = reg;
#endif

	LEAVE();
	return rc;
}

static struct file_operations jz_i2s_mixer_fops =
{
	owner:		THIS_MODULE,
	ioctl:		jz_i2s_ioctl_mixdev,
	open:		jz_i2s_open_mixdev,
	write:		jz_i2s_write_mixdev,
};

int i2s_probe_codec(struct i2s_codec *codec)
{
	/* generic OSS to I2S wrapper */
	return (codec->codecs_ioctrl) ? 1 : 0;
}

/* I2S codec initialisation. */
static int __init jz_i2s_codec_init(struct jz_i2s_controller_info *controller)
{
	int i;

	ENTER();

	for (i = 0; i < NR_I2S; i++) {
		the_codecs[i].private_data = controller;
		if (i2s_probe_codec(&the_codecs[i]) == 0) {
			break;
		}
		if ((the_codecs[i].dev_mixer = register_sound_mixer(&jz_i2s_mixer_fops, the_codecs[i].id)) < 0) {
			printk(KERN_ERR "JZ I2S: couldn't register mixer!\n");
			break;
		}

	}
	controller->i2s_codec = &the_codecs[0];

	LEAVE();
	return i;
}

static void jz_i2s_reinit_hw(struct i2s_codec *codec, int mode)
{
	ENTER();

   	__i2s_disable();
	schedule_timeout(5);
	codec_ioctrl(codec, CODEC_EACH_TIME_INIT, 0);
	__i2s_disable_record();
	__i2s_disable_replay();
	__i2s_disable_loopback();
	__i2s_set_transmit_trigger(4);
	__i2s_set_receive_trigger(3);
	__i2s_send_rfirst();

	LEAVE();
}

static int jz_codec_set_speed(struct i2s_codec *codec, int rate, int mode)
{
	ENTER();

	/* 8000, 11025, 16000, 22050, 24000, 32000, 44100, 48000, 99999999 ? */
	if (mode & CODEC_RMODE) {
		rate = codec_ioctrl(codec, CODEC_SET_RECORD_SPEED, rate);
		if (rate > 0) {
			codec->record_audio_rate = rate;
		} else {
			rate = codec->record_audio_rate;
		}
	}
	if (mode & CODEC_WMODE) {
		rate = codec_ioctrl(codec, CODEC_SET_REPLAY_SPEED, rate);
		if (rate > 0) {
			codec->replay_audio_rate = rate;
		} else {
			rate = codec->replay_audio_rate;
		}
	}

	LEAVE();
	return rate;
}

static short jz_codec_set_channels(struct i2s_codec *codec, short channels, int mode)
{
	ENTER();

	DPRINT_IOC("%s mode = %x channels = %d\n", __FUNCTION__, mode, channels);
	DPRINT_IOC("mode & CODEC_RMODE == %x", mode & CODEC_RMODE);

	if (mode & CODEC_RMODE) {
		channels = codec_ioctrl(codec, CODEC_SET_RECORD_CHANNEL, channels);
		codec->record_codec_channel = channels;
	}
	if (mode & CODEC_WMODE) {
		channels = codec_ioctrl(codec, CODEC_SET_REPLAY_CHANNEL, channels);
		codec->replay_codec_channel = channels;
		if (channels == 1) {
			__aic_enable_mono2stereo();
			__aic_out_channel_select(0);
		} else {
			__aic_disable_mono2stereo();
			__aic_out_channel_select(1);
		}
	}

	LEAVE();

	return channels;
}

static void jz_codec_select_mode(struct i2s_codec *codec, int mode)
{
	ENTER();

	switch (mode) {
	case CODEC_WRMODE:
		if (codec->use_mic_line_flag == USE_NONE) {
			codec->use_mic_line_flag = USE_MIC;
		}
		codec_ioctrl(codec, CODEC_SET_REPLAY_RECORD, codec->use_mic_line_flag);
		break;
	case CODEC_RMODE:
		if (codec->use_mic_line_flag == USE_NONE) {
			codec->use_mic_line_flag = USE_MIC;
		}
		codec_ioctrl(codec, CODEC_SET_RECORD, codec->use_mic_line_flag);
		break;
	case CODEC_WMODE:
		printk("===>wmode!!!\n");
		codec_ioctrl(codec, CODEC_SET_REPLAY, mode);
		break;
	}

	LEAVE();
}

void jz_codec_anti_pop(struct i2s_codec *codec, int mode)
{
	ENTER();
	codec_ioctrl(codec, CODEC_ANTI_POP, mode);
	LEAVE();
}

void jz_codec_close(struct i2s_codec *codec, int mode)
{
	ENTER();
	down(&codec->i2s_sem);
	codec->codecs_ioctrl(codec, CODEC_TURN_OFF, mode);
	up(&codec->i2s_sem);
	LEAVE();
}

/***************************************************************
			filter functions
 ***************************************************************/

/*
 * Convert signed byte to unsiged byte
 *
 * Mapping:
 * 	signed		unsigned
 *	0x00 (0)	0x80 (128)
 *	0x01 (1)	0x81 (129)
 *	......		......
 *	0x7f (127)	0xff (255)
 *	0x80 (-128)	0x00 (0)
 *	0x81 (-127)	0x01 (1)
 *	......		......
 *	0xff (-1)	0x7f (127)
 */
static int convert_8bits_signed2unsigned(void *buffer, int counter)
{
	int i;
	int counter_8align	= counter & ~0x7;
	unsigned char *ucsrc	= buffer;
	unsigned char *ucdst	= buffer;

	ENTER();

	for (i = 0; i < counter_8align; i+=8) {
		*(ucdst + i + 0) = *(ucsrc + i + 0) + 0x80;
		*(ucdst + i + 1) = *(ucsrc + i + 1) + 0x80;
		*(ucdst + i + 2) = *(ucsrc + i + 2) + 0x80;
		*(ucdst + i + 3) = *(ucsrc + i + 3) + 0x80;
		*(ucdst + i + 4) = *(ucsrc + i + 4) + 0x80;
		*(ucdst + i + 5) = *(ucsrc + i + 5) + 0x80;
		*(ucdst + i + 6) = *(ucsrc + i + 6) + 0x80;
		*(ucdst + i + 7) = *(ucsrc + i + 7) + 0x80;
		//printk("csrc + %d + 7 = %d,  ucdst + %d + 7 = %d\n",
		//       i, *(csrc + i + 7), i, *(ucdst + i + 7));
	}

	BUG_ON(i != counter_8align);

	for (i = counter_8align; i < counter; i++) {
		*(ucdst + i) = *(ucsrc + i) + 0x80;
	}

	//printk("[dbg] src = 0x%02x (%d) --- dst = 0x%02x (%d), cnt = %d, cnt8a = %d\n",
	//       *csrc, *csrc, *ucdst, *ucdst, counter, counter_8align);
	LEAVE();
	return counter;
}

/*
 * Convert stereo data to mono data, data width: 8 bits/channel
 *
 * buff:	buffer address
 * data_len:	data length in kernel space, the length of stereo data
 *		calculated by "node->end - node->start"
 */
int convert_8bits_stereo2mono(void *buff, int data_len)
{
	/* stride = 16 bytes = 2 channels * 1 byte * 8 pipelines */
	int data_len_16aligned = data_len & ~0xf;
	int mono_cur, stereo_cur;
	unsigned char *uc_buff = buff;

	/* copy 8 times each loop */
	for (stereo_cur = mono_cur = 0;
	     stereo_cur < data_len_16aligned;
	     stereo_cur += 16, mono_cur += 8) {

		uc_buff[mono_cur + 0] = uc_buff[stereo_cur + 0];
		uc_buff[mono_cur + 1] = uc_buff[stereo_cur + 2];
		uc_buff[mono_cur + 2] = uc_buff[stereo_cur + 4];
		uc_buff[mono_cur + 3] = uc_buff[stereo_cur + 6];
		uc_buff[mono_cur + 4] = uc_buff[stereo_cur + 8];
		uc_buff[mono_cur + 5] = uc_buff[stereo_cur + 10];
		uc_buff[mono_cur + 6] = uc_buff[stereo_cur + 12];
		uc_buff[mono_cur + 7] = uc_buff[stereo_cur + 14];
	}

	BUG_ON(stereo_cur != data_len_16aligned);

	/* remaining data */
	for (; stereo_cur < data_len; stereo_cur += 2, mono_cur++) {
		uc_buff[mono_cur] = uc_buff[stereo_cur];
	}

	LEAVE();
	return (data_len / 2);
}

/*
 * Convert stereo data to mono data, and convert signed byte to unsigned byte.
 *
 * data width: 8 bits/channel
 *
 * buff:	buffer address
 * data_len:	data length in kernel space, the length of stereo data
 *		calculated by "node->end - node->start"
 */
int convert_8bits_stereo2mono_signed2unsigned(void *buff, int data_len)
{
	/* stride = 16 bytes = 2 channels * 1 byte * 8 pipelines */
	int data_len_16aligned = data_len & ~0xf;
	int mono_cur, stereo_cur;
	unsigned char *uc_buff = buff;

	/* copy 8 times each loop */
	for (stereo_cur = mono_cur = 0;
	     stereo_cur < data_len_16aligned;
	     stereo_cur += 16, mono_cur += 8) {

		uc_buff[mono_cur + 0] = uc_buff[stereo_cur + 0] + 0x80;
		uc_buff[mono_cur + 1] = uc_buff[stereo_cur + 2] + 0x80;
		uc_buff[mono_cur + 2] = uc_buff[stereo_cur + 4] + 0x80;
		uc_buff[mono_cur + 3] = uc_buff[stereo_cur + 6] + 0x80;
		uc_buff[mono_cur + 4] = uc_buff[stereo_cur + 8] + 0x80;
		uc_buff[mono_cur + 5] = uc_buff[stereo_cur + 10] + 0x80;
		uc_buff[mono_cur + 6] = uc_buff[stereo_cur + 12] + 0x80;
		uc_buff[mono_cur + 7] = uc_buff[stereo_cur + 14] + 0x80;
	}

	BUG_ON(stereo_cur != data_len_16aligned);

	/* remaining data */
	for (; stereo_cur < data_len; stereo_cur += 2, mono_cur++) {
		uc_buff[mono_cur] = uc_buff[stereo_cur] + 0x80;
	}

	LEAVE();
	return (data_len / 2);
}

/*
 * Convert stereo data to mono data, data width: 16 bits/channel
 *
 * buff:	buffer address
 * data_len:	data length in kernel space, the length of stereo data
 *		calculated by "node->end - node->start"
 */
int convert_16bits_stereo2mono(void *buff, int data_len)
{
	/* stride = 32 bytes = 2 channels * 2 byte * 8 pipelines */
	int data_len_32aligned = data_len & ~0x1f;
	int data_cnt_ushort = data_len_32aligned / 2;
	int mono_cur, stereo_cur;
	unsigned short *ushort_buff = (unsigned short *)buff;

	/* copy 8 times each loop */
	for (stereo_cur = mono_cur = 0;
	     stereo_cur < data_cnt_ushort;
	     stereo_cur += 16, mono_cur += 8) {

		ushort_buff[mono_cur + 0] = ushort_buff[stereo_cur + 0];
		ushort_buff[mono_cur + 1] = ushort_buff[stereo_cur + 2];
		ushort_buff[mono_cur + 2] = ushort_buff[stereo_cur + 4];
		ushort_buff[mono_cur + 3] = ushort_buff[stereo_cur + 6];
		ushort_buff[mono_cur + 4] = ushort_buff[stereo_cur + 8];
		ushort_buff[mono_cur + 5] = ushort_buff[stereo_cur + 10];
		ushort_buff[mono_cur + 6] = ushort_buff[stereo_cur + 12];
		ushort_buff[mono_cur + 7] = ushort_buff[stereo_cur + 14];
	}

	BUG_ON(stereo_cur != data_cnt_ushort);

	/* remaining data */
	for (; stereo_cur < data_cnt_ushort; stereo_cur += 2, mono_cur++) {
		ushort_buff[mono_cur] = ushort_buff[stereo_cur];
	}

	LEAVE();
	return (data_len / 2);
}

/*
 * Set convert function for audio_pipe
 *
 * In AIC, we just use signed data for all ops as it is shared by
 * replay and record. So, converting data for every non-compatible
 * format is neccessary.
 */
static inline int endpoint_set_filter(audio_pipe *endpoint, int format, int channels)
{
	ENTER();

	DPRINT("%s %d, endpoint = 0x%08x, format = %d, channels = %d\n",
	       __FUNCTION__, __LINE__, (unsigned int)endpoint, format, channels);

	endpoint->filter = NULL;

	switch (format) {
	case AFMT_U8:
		if (endpoint == &in_endpoint) {
			if (channels == 2) {
				endpoint->filter = convert_8bits_stereo2mono_signed2unsigned;
				DPRINT("$$$$ set pin_endpoint->filter = convert_8bits_stereo_2_mono\n");
			} else {
				endpoint->filter = convert_8bits_signed2unsigned;
				DPRINT("$$$$ set pin_endpoint->filter = convert_8bits_signed2unsigned\n");
			}
		}
		break;
	case AFMT_S16_LE:
		if (endpoint == &in_endpoint) {
			if (channels == 1) {
				endpoint->filter = convert_16bits_stereo2mono;
				DPRINT("$$$$ set pin_endpoint->filter = convert_16bits_stereo2mono\n");
			} else {
				endpoint->filter = NULL;
				DPRINT("$$$$ set pin_endpoint->filter = NULL\n");
			}
		}
		break;
	default:
		printk("JZ I2S endpoint_set_filter: unknown format\n");
		endpoint->filter = NULL;
	}

	LEAVE();
	return 0;
}

/*
 * The "format" contains data width, signed/unsigned and LE/BE
 *
 * The AIC registers will not be modified !
 *
 * For CODEC	set	data_width
 */
static int jz_codec_set_format(struct i2s_codec *codec, unsigned int format, int mode)
{
	/* The value of format reference to soundcard.h:
	 *
	 * AFMT_MU_LAW		0x00000001
	 * AFMT_A_LAW		0x00000002
	 * AFMT_IMA_ADPCM	0x00000004
	 * AFMT_U8		0x00000008
	 * AFMT_S16_LE		0x00000010
	 * AFMT_S16_BE		0x00000020
	 * AFMT_S8		0x00000040
	 */
	int data_width = 0;

	ENTER();

	DPRINT("$$$$ %s %d, format = %u, mode = %d\n", __FUNCTION__, __LINE__, format, mode);

	down(&codec->i2s_sem);

	/*
	 * It is dangerous to modify settings about signed bit, endian and M2S
	 * as record and replay shared the settings.
	 *
	 * Now we don't support unsigned format (AFMT_U8) and BE format (AFMT_S16_BE)
	 * To support such format, corresponding filter function must be implemented.
	 */
	switch (format) {
	case AFMT_U8:
		data_width = 8;
		if (mode & CODEC_RMODE) {
			__i2s_set_iss_sample_size(8);
		}
		if (mode & CODEC_WMODE) {
			__i2s_set_oss_sample_size(8);
		}
		break;
	case AFMT_S8:
		data_width = 8;
		if (mode & CODEC_RMODE) {
			__i2s_set_iss_sample_size(8);
		}
		if (mode & CODEC_WMODE) {
			__i2s_set_oss_sample_size(8);
		}
		break;
	case AFMT_S16_LE:
		data_width = 16;
		if (mode & CODEC_RMODE) {
			__i2s_set_iss_sample_size(16);
		}
		if (mode & CODEC_WMODE) {
			__i2s_set_oss_sample_size(16);
		}
		break;
	case AFMT_S16_BE:
		data_width = 16;
		if (mode & CODEC_RMODE) {
			__i2s_set_iss_sample_size(16);
		}
		if (mode & CODEC_WMODE) {
			__i2s_set_oss_sample_size(16);
		}
		break;
	default:
		printk("JZ I2S: Unkown sound format %d\n", format);
		goto _ERROR_SET_FORMAT;
	}

	if (mode & CODEC_RMODE) {
		if (codec->codecs_ioctrl(codec, CODEC_SET_RECORD_DATA_WIDTH, data_width) < 0) {
			printk("JZ I2S: CODEC ioctl error, command: CODEC_SET_RECORD_FORMAT");
			goto _ERROR_SET_FORMAT;
		}
		codec->record_format = format;
	}

	if (mode & CODEC_WMODE) {
		if (codec->codecs_ioctrl(codec, CODEC_SET_REPLAY_DATA_WIDTH, data_width) < 0) {
			printk("JZ I2S: CODEC ioctl error, command: CODEC_SET_REPLAY_FORMAT");
			goto _ERROR_SET_FORMAT;
		}
		codec->replay_format = format;
	}

	up(&codec->i2s_sem);
	LEAVE();
	return format;

_ERROR_SET_FORMAT:
	up(&codec->i2s_sem);
	LEAVE();
	return -1;
}

static int jz_audio_release(struct inode *inode, struct file *file)
{
	struct jz_i2s_controller_info *controller = (struct jz_i2s_controller_info *) file->private_data;
	int mode = 0;
	int codec_closed = 0;


	ENTER();

	if (controller == NULL) {
		printk("\nAudio device not ready!\n");
		return -ENODEV;
	}
	if ((controller->pin_endpoint == NULL) && (controller->pout_endpoint == NULL) ) {
		printk("\nAudio endpoint not open!\n");
		return -ENODEV;
	}
	if ((file->f_mode & FMODE_READ) && controller->pin_endpoint) {
//		printk("Read mode, %s\n", __FUNCTION__);
		mode |= CODEC_RMODE;
		audio_close_endpoint(controller->pin_endpoint, FORCE_STOP);
		controller->pin_endpoint = NULL;

		__i2s_disable_receive_dma();
		jz_codec_close(controller->i2s_codec, mode);
		__i2s_disable_record();
	}

	if ((file->f_mode & FMODE_WRITE) && controller->pout_endpoint) {
//		printk("Write mode, %s\n", __FUNCTION__);
		mode |= CODEC_WMODE;
		audio_close_endpoint(controller->pout_endpoint, NOMAL_STOP);
		controller->pout_endpoint = NULL;

		__i2s_disable_transmit_dma();

		jz_codec_close(controller->i2s_codec, mode);
		__i2s_enable_replay();
		msleep(1);

		__i2s_disable_replay();
		codec_closed = 1;


#ifdef CONFIG_JZ_EBOOK_HARD
//		printk("DEBUG: %s, %d\n", __FUNCTION__, __LINE__);
		__gpio_clear_pin(GPIO_SPK_SHUD);
#endif
	}


	if ((controller->pin_endpoint == NULL) && (controller->pout_endpoint == NULL) ) {
		__i2s_disable();
	}

	last_read_node = NULL;

//	jz_codec_close(controller->i2s_codec, mode);

#ifdef CONFIG_JZ_EBOOK_HARD
//	printk("DEBUG: %s, %d\n", __FUNCTION__, __LINE__);
//	__cpm_stop_aic1();
	audio_device_open = 0;
#endif

	LEAVE();
	return 0;
}

static int jz_audio_open(struct inode *inode, struct file *file)
{
	struct jz_i2s_controller_info *controller = the_i2s_controller;
	struct i2s_codec *codec = controller->i2s_codec;
	int mode = 0;
	int reset = 1;

	ENTER();

	printk("===>enter %s:%d\n", __func__, __LINE__);

	if (controller == NULL) {
		return -ENODEV;
	}
#ifdef CONFIG_JZ_EBOOK_HARD
//	printk("DEBUG: %s, %d\n", __FUNCTION__, __LINE__);
//	__cpm_start_aic1();
#endif

	if (controller->pin_endpoint || controller->pout_endpoint) {
		reset = 0;
	}

	if ((file->f_mode & FMODE_READ) && (controller->pin_endpoint)) {
		printk("\nAudio read device is busy!\n");
		return -EBUSY;
	}
	if ((file->f_mode & FMODE_WRITE) && (controller->pout_endpoint)) {
		printk("\nAudio write device is busy!\n");
		return -EBUSY;
	}

	if (file->f_mode & FMODE_WRITE) {
		controller->pout_endpoint = &out_endpoint;
		controller->pout_endpoint->is_non_block = file->f_flags & O_NONBLOCK;
		mode |= CODEC_WMODE;
	}
	if (file->f_mode & FMODE_READ) {
		controller->pin_endpoint = &in_endpoint;
		controller->pin_endpoint->is_non_block = file->f_flags & O_NONBLOCK;
		mode |= CODEC_RMODE;
	}
   	file->private_data = controller;

	/* we should turn codec and anti-pop first */
	//jz_codec_anti_pop(controller->i2s_codec, mode);
	//printk("===>check point1\n");
	//mdelay(5000);

	if (mode & CODEC_RMODE){
/*
		jz_codec_set_channels(codec, 2, CODEC_RMODE);
		jz_codec_set_format(codec, 8, CODEC_RMODE);
		jz_codec_set_speed(codec, 8000, CODEC_RMODE);
*/
		jz_codec_set_channels(codec, 2, CODEC_RMODE);
		jz_codec_set_format(codec, 16, CODEC_RMODE);
		jz_codec_set_speed(codec, 44100, CODEC_RMODE);
		codec->user_need_mono = 0;

		set_controller_triger(controller, &in_endpoint, codec->record_codec_channel, codec->record_format);


	}
	if (mode & CODEC_WMODE) {
		jz_codec_set_channels(codec, 2, CODEC_WMODE);
		jz_codec_set_format(codec, 16, CODEC_WMODE);
		jz_codec_set_speed(codec, 44100, CODEC_WMODE);
		set_controller_triger(controller, &out_endpoint, codec->replay_codec_channel, codec->replay_format);
#ifdef CONFIG_JZ_EBOOK_HARD
//		printk("DEBUG: %s, %d\n", __FUNCTION__, __LINE__);
		while (audio_device_pm_state == 1) schedule();
		audio_device_open = 1;
		if ((__gpio_get_pin(GPIO_HPONE_PLUG))) /* opposite logic with D21 */
		{
			__gpio_set_pin(GPIO_SPK_SHUD);
		}
#endif
	}

	//printk("===>check point2\n");
	//mdelay(5000);

	DPRINT_IOC("============ default_codec record ===============\n"
		   "format	= %d\n"
		   "channels	= %d\n"
		   "rate	= %d\n"
		   "dma one tran bit = %d\n",
		   codec->record_format, codec->record_codec_channel,
		   codec->record_audio_rate, in_endpoint.dma.onetrans_bit);

	DPRINT_IOC("============ default_codec replay ===============\n"
		   "format	= %d\n"
		   "channels	= %d\n"
		   "rate	= %d\n"
		   "dma one tran bit = %d\n",
		   codec->replay_format, codec->replay_codec_channel,
		   codec->replay_audio_rate, out_endpoint.dma.onetrans_bit);

	jz_codec_select_mode(controller->i2s_codec, mode);

	//printk("===>check point3\n");
	//mdelay(5000);

	/* note: reset AIC  protected REG_AIC_I2SCR.ECCLK is setting */
	if (reset) {
		down(&controller->i2s_codec->i2s_sem);
		//__i2s_enable_transmit_dma();
		//__i2s_enable_receive_dma();
		//__i2s_enable_replay();
		__i2s_enable();
		up(&controller->i2s_codec->i2s_sem);
	}
	//reinit codec option

	//DUMP_AIC_REGS();
	DPRINT_TRC(".... jz_audio_open\n");

	g_play_first = 0;
	jz_codec_anti_pop(controller->i2s_codec, mode);

	//printk("===>check point4\n");
	//mdelay(5000);

	LEAVE();
	return 0;
}

static int jz_audio_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	long	rc = -EINVAL;
	int	val = 0;
	int	mode = 0;

	struct jz_i2s_controller_info *controller = (struct jz_i2s_controller_info *) file->private_data;
	struct i2s_codec *codec = controller->i2s_codec;
	audio_pipe	*pin_endpoint = controller->pin_endpoint;
	audio_pipe	*pout_endpoint = controller->pout_endpoint;

	ENTER();

	DPRINT_IOC("[dsp IOCTL] --------------------------------\n");
	DPRINT_IOC(" dsp IOCTL %s cmd = (0x%08x), arg = %lu\n", __FUNCTION__, cmd, arg);
	DPRINT_DSP_IOC_CMD(cmd);
	DPRINT_IOC("[dsp IOCTL] --------------------------------\n");

	if (file->f_mode & FMODE_READ) {
		mode |= CODEC_RMODE;
	}
	if (file->f_mode & FMODE_WRITE) {
		mode |= CODEC_WMODE;
	}

	switch (cmd) {

	case OSS_GETVERSION:
		rc = put_user(SOUND_VERSION, (int *)arg);
		break;
	case SNDCTL_DSP_RESET:
		break;

	case SNDCTL_DSP_SYNC:
		if (mode & CODEC_WMODE) {
			if (pout_endpoint) {
				audio_sync_endpoint(pout_endpoint);
			}
		}
		rc = 1;
		break;

	case SNDCTL_DSP_SPEED:
		/* set smaple rate */
		if (get_user(val, (int *)arg)) {
			rc = -EFAULT;
		}
		//printk("SNDCTL_DSP_SPEED ... set to %d\n", val);
		val = jz_codec_set_speed(codec, val, mode);
		rc = put_user(val, (int *)arg);
		break;

	case SNDCTL_DSP_STEREO:
		/* set stereo or mono channel */
		if (get_user(val, (int *)arg)) {
		    rc = -EFAULT;
		}

		jz_codec_set_channels(controller->i2s_codec, val ? 2 : 1, mode);

		if (mode & CODEC_RMODE) {
			set_controller_triger(controller, pin_endpoint,
					      codec->record_codec_channel, codec->record_format);
		}

		if (mode & CODEC_WMODE) {
			set_controller_triger(controller, pout_endpoint,
					      codec->replay_codec_channel, codec->replay_format);
		}

		rc = 1;
		break;

	case SNDCTL_DSP_GETBLKSIZE:
	{
		// It seems that device could only be open with one mode (R or W)
		int fragsize = 0;
		if (mode & CODEC_RMODE) {
			fragsize = pin_endpoint->fragsize;
		}
		if (mode & CODEC_WMODE) {
			fragsize = pout_endpoint->fragsize;
		}
		rc = put_user(fragsize, (int *)arg);
		break;
	}

	case SNDCTL_DSP_GETFMTS:
		/* Returns a mask of supported sample format*/
		rc = put_user(AFMT_U8 | AFMT_S16_LE, (int *)arg);
		break;

	case SNDCTL_DSP_SETFMT:
		/* Select sample format */
		if (get_user(val, (int *)arg)) {
			rc = -EFAULT;
		}

//		printk("\nSNDCTL_DSP_SETFMT ... set to %d\n", val);

		if (val == AFMT_QUERY) {
			if (mode & CODEC_RMODE) {
				val = codec->record_format;
			} else {
				val = codec->replay_format;
			}
		} else {
			val = jz_codec_set_format(codec, val, mode);
			if (mode & CODEC_RMODE) {
				if (codec->user_need_mono) {
					endpoint_set_filter(pin_endpoint, val, 1);
				} else {
					endpoint_set_filter(pin_endpoint, val, 2);
				}

				set_controller_triger(controller, pin_endpoint,
						      codec->record_codec_channel, codec->record_format);
			}
			if (mode & CODEC_WMODE) {
				set_controller_triger(controller, pout_endpoint,
						      codec->replay_codec_channel, codec->replay_format);
			}
		}

		rc = put_user(val, (int *)arg);
		break;

	case SNDCTL_DSP_CHANNELS:
		if (get_user(val, (int *)arg)) {
			rc = -EFAULT;
		}
		//printk("\nSNDCTL_DSP_CHANNELS ... set to %d\n", val);

		/* if mono, change to 2, and set 1 to codec->user_need_mono */
		if (mode & CODEC_RMODE) {
			if (val == 1) {
				val = 2;
				codec->user_need_mono = 1;

			} else {
				codec->user_need_mono = 0;
			}
		}

		/* Following lines could be marked as nothing will be changed */
		jz_codec_set_channels(codec, val, mode);

		if (mode & CODEC_RMODE) {
			/* Set filter according to channel count */
			if (codec->user_need_mono) {
				endpoint_set_filter(pin_endpoint, codec->record_format, 1);
			} else {
				endpoint_set_filter(pin_endpoint, codec->record_format, 2);
			}

			set_controller_triger(controller, pin_endpoint,
					      codec->record_codec_channel, codec->record_format);
		}
		if (mode & CODEC_WMODE) {
			set_controller_triger(controller, pout_endpoint,
					      codec->replay_codec_channel, codec->replay_format);
		}

		/* Restore for return value */
		if (codec->user_need_mono) {
			val = 1;
		}

		rc = put_user(val, (int *)arg);
		break;

	case SNDCTL_DSP_POST:
		/* FIXME: the same as RESET ?? */
		break;

	case SNDCTL_DSP_SUBDIVIDE:
		break;

	case SNDCTL_DSP_SETFRAGMENT:
		rc = get_user(val, (long *) arg);
		if (rc != -EINVAL) {
			int newfragsize, newfragstotal;
			newfragsize = 1 << (val & 0xFFFF);
			if (newfragsize < 4 * PAGE_SIZE) {
				newfragsize = 4 * PAGE_SIZE;
			}
			if (newfragsize > (16 * PAGE_SIZE)) {
				newfragsize = 16 * PAGE_SIZE;
			}

			newfragstotal = (val >> 16) & 0x7FFF;
			if (newfragstotal < 2) {
				newfragstotal = 2;
			}
			if (newfragstotal > 32) {
				newfragstotal = 32;
			}

			if (mode & CODEC_RMODE) {
				rc = audio_resizemem_endpoint(controller->pin_endpoint, newfragsize, newfragstotal);
				if (!rc) {
					rc = -EINVAL;
				}
			}
			if (mode & CODEC_WMODE) {
				rc = audio_resizemem_endpoint(controller->pout_endpoint, newfragsize, newfragstotal);
				if (!rc) {
					rc = -EINVAL;
				}
			}
		}
		break;

	case SNDCTL_DSP_GETCAPS:
		rc = put_user(DSP_CAP_REALTIME | DSP_CAP_BATCH, (int *)arg);
		break;

	case SNDCTL_DSP_NONBLOCK:
		file->f_flags |= O_NONBLOCK;
		rc = 0;
		break;

	case SNDCTL_DSP_SETDUPLEX:
		rc = -EINVAL;
		break;

	case SNDCTL_DSP_GETOSPACE:
	{
		audio_buf_info abinfo;
		if (!(mode & CODEC_WMODE)) {
			return -EINVAL;
		}
		audio_get_endpoint_freesize(pout_endpoint, &abinfo);
		rc = copy_to_user((void *)arg, &abinfo, sizeof(abinfo)) ? -EFAULT : 0;
		break;
	}

	case SNDCTL_DSP_GETISPACE:
	{
		audio_buf_info abinfo;
		if (!(mode & CODEC_RMODE)) {
			return -EINVAL;
		}
		audio_get_endpoint_freesize(controller->pin_endpoint, &abinfo);
		rc = copy_to_user((void *)arg, &abinfo, sizeof(abinfo)) ? -EFAULT : 0;
		break;
	}

	case SNDCTL_DSP_GETTRIGGER:
		val = 0;
		if ((mode & CODEC_RMODE) && controller->pin_endpoint) {
			val |= PCM_ENABLE_INPUT;
		}
		if ((mode & CODEC_WMODE) && controller->pout_endpoint) {
			val |= PCM_ENABLE_OUTPUT;
		}
		rc = put_user(val, (int *)arg);

		break;

	case SNDCTL_DSP_SETTRIGGER:
		if (get_user(val, (int *)arg)) {
			rc = -EFAULT;
		}
		break;

	case SNDCTL_DSP_GETIPTR:
	{
		count_info cinfo;
		if (!(mode & CODEC_RMODE)) {
			rc = -EINVAL;
		}
		rc = copy_to_user((void *)arg, &cinfo, sizeof(cinfo));
		break;
	}

	case SNDCTL_DSP_GETOPTR:
	{
		count_info cinfo;
		if (!(mode & CODEC_WMODE)) {
			rc = -EINVAL;
		}
		rc = copy_to_user((void *) arg, &cinfo, sizeof(cinfo));
		break;
	}

	case SNDCTL_DSP_GETODELAY:
	{
		// fix me !!!
		int unfinish = 0;
		if (!(mode & CODEC_WMODE)) {
			rc = -EINVAL;
		}
		rc = put_user(unfinish, (int *) arg);
		break;
	}

	case SOUND_PCM_READ_RATE:
		if (mode  & CODEC_RMODE) {
			//printk("\nSOUND_PCM_READ_RATE = %d\n", codec->record_audio_rate);
			rc = put_user(codec->record_audio_rate, (int *)arg);
		}
		if (mode & CODEC_WMODE) {
			//printk("\nSOUND_PCM_READ_RATE = %d\n", codec->replay_audio_rate);
			rc = put_user(codec->replay_audio_rate, (int *)arg);
		}
		break;

	case SOUND_PCM_READ_CHANNELS:
		if (mode & CODEC_RMODE) {
			//printk("\nSOUND_PCM_READ_RATE = %d\n", codec->record_codec_channel);
			rc = put_user(codec->record_codec_channel, (int *)arg);
		}
		if (mode & CODEC_WMODE) {
			//printk("\nSOUND_PCM_READ_RATE = %d\n", codec->replay_codec_channel);
			rc = put_user(codec->replay_codec_channel, (int *)arg);
		}
		break;

	case SOUND_PCM_READ_BITS:
		if (mode & CODEC_RMODE) {
			rc = put_user((codec->record_format & (AFMT_S8 | AFMT_U8)) ? 8 : 16, (int *)arg);
		}
		if (mode & CODEC_WMODE) {
			rc = put_user((codec->record_format & (AFMT_S8 | AFMT_U8)) ? 8 : 16, (int *)arg);
		}
		break;

	case SNDCTL_DSP_MAPINBUF:
	case SNDCTL_DSP_MAPOUTBUF:
	case SNDCTL_DSP_SETSYNCRO:
	case SOUND_PCM_WRITE_FILTER:
	case SOUND_PCM_READ_FILTER:
		rc = -EINVAL;
		break;
#if 0
	/* may be for msm only */
	case AUDIO_GET_CONFIG:
 		break;

	case AUDIO_SET_CONFIG:
		break;
#endif
	default:
		printk("%s[%s]:%d---no cmd\n",__FILE__,__FUNCTION__,__LINE__);
		break;
	}

	LEAVE();

	return rc;
}

static inline int endpoint_put_userdata(audio_pipe *endpoint, const char __user *buffer, size_t count)
{
	unsigned long	flags;
	audio_node	*node;

	ENTER();
	DPRINT("<<<< put_userdata\n");

	AUDIO_LOCK(endpoint->lock, flags);
	node = get_audio_freenode(endpoint->mem);
	AUDIO_UNLOCK(endpoint->lock, flags);

	// For non-block mode
	if (endpoint->is_non_block && !node) {
		LEAVE();
		return 0;
	}

	// For block mode, wait free node
	while (!node) {
		DPRINT("wait ----------\n");

		AUDIO_LOCK(endpoint->lock, flags);
		DUMP_LIST((audio_head *)endpoint->mem);
		DUMP_NODE(endpoint->savenode, "SN");
		AUDIO_UNLOCK(endpoint->lock, flags);

		// wait available node
		wait_event_interruptible(endpoint->q_full, (endpoint->avialable_couter >= 1));

		AUDIO_LOCK(endpoint->lock, flags);
		node = get_audio_freenode(endpoint->mem);
		endpoint->avialable_couter = 0;
		AUDIO_UNLOCK(endpoint->lock, flags);
	}

	if (copy_from_user((void *)node->pBuf, buffer, count)) {
		printk("JZ I2S: copy_from_user failed !\n");
		return -EFAULT;
	}
	dma_cache_wback_inv((unsigned long)node->pBuf,(unsigned long)count);
	node->start = 0;
	node->end = count;
	AUDIO_LOCK(endpoint->lock, flags);
	put_audio_usenode(endpoint->mem, node);
	AUDIO_UNLOCK(endpoint->lock, flags);

	LEAVE();

	return count;
}

static ssize_t jz_audio_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct jz_i2s_controller_info *controller = (struct jz_i2s_controller_info *)file->private_data;
	audio_pipe *pout_endpoint = controller->pout_endpoint;
	struct i2s_codec *codec = (struct i2s_codec *)controller->i2s_codec;
	size_t	usecount = 0;
	int	bat_cnt = -1;
	int	rem_cnt = 0;

	if (!g_play_first) {
		// first play, trun on dac mute
		//codec_ioctrl(codec, CODEC_FIRST_OUTPUT, 0);
		g_play_first = 1;
	}

	//printk("===>enter %s:%d\n", __func__, __LINE__);

	ENTER();

	//dump_dlv_regs(__FUNCTION__);
	//dump_aic_regs(__FUNCTION__);

	// wll@20101020
//	printk("===>enter %s: \n", __FUNCTION__);
//	printk("write data count = %d\n", count);

	while (count >= pout_endpoint->fragsize) {

		bat_cnt = endpoint_put_userdata(pout_endpoint,
						&(buffer[usecount]),
						pout_endpoint->fragsize);
		// Prepare data success.
		if (bat_cnt > 0) {
			usecount += bat_cnt;
			count -= bat_cnt;
			DPRINT("bat_cnt = %d\n", bat_cnt);
		}
		// Perhaps non node is avialable.
		else if (bat_cnt == 0) {
			DPRINT("bat_cnt == 0\n");
			break;
		}
		// Error occured.
		else {
			// break and handle prepared data.
			if (usecount > 0) {
				DPRINT("bat_cnt < 0, usecount > 0\n");
				break;
			}
			// Has not prepared any data and return error when prepared data.
			else {
				DPRINT("bat_cnt < 0, usecount == 0\n");
				return bat_cnt;
			}
		}
	}

	DPRINT("count = %d\n", count);

	// Prepare few data or remain data after below code.
	if (bat_cnt != 0 && count >= 32) {
		DPRINT("check point 2 ... count = %d\n", count);
		rem_cnt = endpoint_put_userdata(pout_endpoint, &buffer[usecount], count);
		if (rem_cnt > 0) {
			usecount += rem_cnt;
			count -= rem_cnt;
			DPRINT("check point 3 ... rem_cnt = %d\n", rem_cnt);
		} else if (rem_cnt <= 0) {
			// Not success... return Error.
			if (usecount == 0) {
				DPRINT("rem_cnt <= 0, usecount == 0\n");
				return rem_cnt;
			}
			// Go on handle prepared data, ignore the error.
			else {
				DPRINT("rem_cnt <= 0, usecount != 0, usecount = %d\n", usecount);
			}
		}
	}

	// Handle prepared data.
	if (usecount > 0) {
		unsigned long	flags;
		audio_node	*node;
		AUDIO_LOCK(pout_endpoint->lock, flags);
		if ((pout_endpoint->trans_state & PIPE_TRANS) == 0) {
			node = get_audio_usenode(pout_endpoint->mem);
			if (node) {
				unsigned int start;
				start = trystart_endpoint_out(controller, node);
				if (start == 0) {
					printk("JZ I2S: trystart_endpoint_out error\n");
				}
			}
		}
		AUDIO_UNLOCK(pout_endpoint->lock, flags);
	}

	DPRINT("----write data usecount = %d, count = %d\n", usecount, count);
	BUG_ON(count < 0);
	LEAVE();

	return usecount + (count < 32 ? count : 0);
}

/**
 *  Copy recorded sound data from 'use' link list to userspace
 */
static inline int endpoint_get_userdata(audio_pipe *endpoint, const char __user *buffer, size_t count)
{
	unsigned long	flags;
	audio_node	*node = last_read_node;
	int	ret;

	/* counter for node buffer, raw data */
	int	node_buff_cnt = 0;

	ENTER();

	if (!node) {
		AUDIO_LOCK(endpoint->lock, flags);
		node = get_audio_usenode(endpoint->mem);
		AUDIO_UNLOCK(endpoint->lock, flags);

		if (node && endpoint->filter) {
			node_buff_cnt = node->end - node->start;
			node_buff_cnt = endpoint->filter((void *)(node->pBuf + node->start), node_buff_cnt);
			node->end = node->start + node_buff_cnt;
		}
	}

	DPRINT(">>>> %s mode\n", endpoint->is_non_block ? "non block" : "block");

	// For non-block mode
	if (endpoint->is_non_block && !node) {
		return 0;
	}

	// For block mode, wait node which full filled data
	while (!node) {
		if ((endpoint->trans_state & PIPE_TRANS) == 0 ) {
			DPRINT("DMA trans has not been started !\n");
			return -1;
		}

		AUDIO_LOCK(endpoint->lock, flags);
		DUMP_LIST((audio_head *)endpoint->mem);
		DUMP_NODE(endpoint->savenode, "SN");
		AUDIO_UNLOCK(endpoint->lock, flags);

		DPRINT("record stereo ... wait pipe_sem ----------\n");

		// wait available node
//		interruptible_sleep_on(&endpoint->q_full);
		wait_event_interruptible(endpoint->q_full, endpoint->avialable_couter >= 1);

		AUDIO_LOCK(endpoint->lock, flags);
		node = get_audio_usenode(endpoint->mem);
		endpoint->avialable_couter = 0;
		AUDIO_UNLOCK(endpoint->lock, flags);

		if (node && endpoint->filter) {
			node_buff_cnt = node->end - node->start;
			node_buff_cnt = endpoint->filter((void *)(node->pBuf + node->start), node_buff_cnt);
			node->end = node->start + node_buff_cnt;
		}
	}

	if (node && (node_buff_cnt = node->end - node->start)) {
		DPRINT("node_buff_cnt = %d, count = %d\n", node_buff_cnt, count);

		if (count >= (size_t)node_buff_cnt) {
			DPRINT(">>>> count >= fixed_buff_cnt, copy_to_user count = %d\n", node_buff_cnt);
			ret = copy_to_user((void *)buffer, (void *)(node->pBuf + node->start), node_buff_cnt);
			if (ret) {
				printk("JZ I2S: copy_to_user failed, return %d\n", ret);
				return -EFAULT;
			}
			put_audio_freenode(endpoint->mem, node);
			last_read_node = NULL;
		} else {
			DPRINT(">>>> count < fixed_buff_cnt, copy_to_user count = %d\n", count);
			ret = copy_to_user((void *)buffer,(void *)(node->pBuf + node->start), count);
			if (ret) {
				printk("JZ I2S: copy_to_user failed, return %d\n", ret);
				return -EFAULT;
			}
			node->start += count;
			last_read_node = node;
		}
	}

	LEAVE();
	return (node_buff_cnt < count ? node_buff_cnt : count);
}

static ssize_t jz_audio_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	struct jz_i2s_controller_info *controller = (struct jz_i2s_controller_info *)file->private_data;
	audio_pipe	*pin_endpoint = controller->pin_endpoint;
	audio_node	*node;
	unsigned long	flags;
	int		mcount, usecount = 0;

	ENTER();

//	dump_dlv_regs(__FUNCTION__);
//	dump_aic_regs(__FUNCTION__);

	if (count == 0) {
		DPRINT("@@@@ jz_audio_read count == 0\n");
		return 0;
	}

	AUDIO_LOCK(pin_endpoint->lock, flags);

	DUMP_LIST((audio_head *)pin_endpoint->mem);
	DUMP_NODE(pin_endpoint->savenode, "SN");

	DPRINT("@@@@ jz_audio_read, pin_endpoint->trans_state = 0x%08x\n",
	       pin_endpoint->trans_state);

	if ((pin_endpoint->trans_state & PIPE_TRANS) == 0) {
		DPRINT("@@@@ jz_audio_read, PIPE_TRANS\n");
		node = get_audio_freenode(pin_endpoint->mem);
		if (node) {
			unsigned int start;
			DPRINT("@@@@ jz_audio_read, trystart_endpoint_in\n");
//			pin_endpoint->fragsize = count;
			node->end = pin_endpoint->fragsize;

			start = trystart_endpoint_in(controller, node);
			if (start == 0) {
				DPRINT("@@@@ Error ! jz_audio_read, start == 0\n");
				put_audio_freenode(pin_endpoint->mem, node);
			}
		}
	}
	AUDIO_UNLOCK(pin_endpoint->lock, flags);

	DUMP_AIC_REGS(__FUNCTION__);
	DUMP_CODEC_REGS(__FUNCTION__);
	//dump_dlv_regs(__FUNCTION__);
	DPRINT("@@@@ count = %d\n", count);

	do{
		mcount = endpoint_get_userdata(pin_endpoint, &buffer[usecount], count);

		DPRINT("@@@@ jz_audio_read, mcount = %d, usecount = %d\n", mcount, usecount);

		if (mcount < 0) {
			DPRINT("@@@@ jz_audio_read, mcount < 0, %d\n", mcount);
			if (usecount > 0) {
				break;
			} else {
				return mcount;
			}
		} else if (mcount == 0) {
			DPRINT("@@@@ jz_audio_read, mcount == 0\n");
			break;
		} else {
			usecount += mcount;
			count -= mcount;
			DPRINT("@@@@ jz_audio_read, mcount > 0, %d\n", mcount);
		}
	} while (count > 0);

	DPRINT("@@@@ jz_audio_read, usecount = %d\n", usecount);

	LEAVE();
	return usecount;
}

/* static struct file_operations jz_i2s_audio_fops */
static struct file_operations jz_i2s_audio_fops = {
	owner:		THIS_MODULE,
	open:		jz_audio_open,
	release:	jz_audio_release,
	write:		jz_audio_write,
	read:		jz_audio_read,
	ioctl:		jz_audio_ioctl
};

static void __init attach_jz_i2s(struct jz_i2s_controller_info *controller)
{
	char	*name = NULL;
	int	adev = 0; /* No of Audio device. */

	ENTER();

	name = controller->name;

	/* Initialize I2S CODEC and register /dev/mixer. */
	if (jz_i2s_codec_init(controller) <= 0) {
		goto mixer_failed;
	}

	/* Initialize AIC controller and reset it. */
	jz_i2s_reinit_hw(controller->i2s_codec,1);
	adev = register_sound_dsp(&jz_i2s_audio_fops, -1);
	if (adev < 0) {
		goto audio_failed;
	}

	controller->dev_audio = adev;
	jz_codec_anti_pop(controller->i2s_codec, 1);    //CODEC_WMODE

	LEAVE();

	return;
mixer_failed:

audio_failed:
	unregister_sound_dsp(adev);

	LEAVE();
	return;
}

static void __exit unload_jz_i2s(struct jz_i2s_controller_info *controller)
{
	jz_i2s_reinit_hw(controller->i2s_codec,0);
}

//--------------------------------------------------------------------
#ifdef CONFIG_PM
static int jz_i2s_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i;
	struct i2s_codec *codec;
	//audio_sync_endpoint(&out_endpoint);
	//msleep(30);
	for(i = 0;i < NR_I2S; i++){
		codec = &the_codecs[i];
		if (codec && codec->codecs_ioctrl) {
			codec->codecs_ioctrl(codec, CODEC_I2S_SUSPEND, 0);
		}
	}

//	printk("Aic and codec are suspended!\n");
	return 0;
}

static int jz_i2s_resume(struct platform_device *pdev)
{
	int i;
	struct i2s_codec *codec;
	for(i = 0;i < NR_I2S; i++){
		codec = &the_codecs[i];
		if (codec && codec->codecs_ioctrl) {
			codec->codecs_ioctrl(codec, CODEC_I2S_RESUME, 0);
		}
	}
	return 0;
}
#endif /* CONFIG_PM */

static int __init probe_jz_i2s(struct jz_i2s_controller_info **controller)
{
	struct jz_i2s_controller_info *ctrl;

	ENTER();
	ctrl = kmalloc(sizeof(struct jz_i2s_controller_info), GFP_KERNEL);
	if (ctrl == NULL) {
		printk(KERN_ERR "Jz I2S Controller: out of memory.\n");
		return -ENOMEM;
	}
	ctrl->name = "Jz I2S controller";
	ctrl->pout_endpoint = 0;
	ctrl->pin_endpoint = 0;
	ctrl->error = 0;
	//ctrl->i2s_codec->use_mic_line_flag = USE_NONE;

	*controller = ctrl;

	LEAVE();

	return 0;
}

void i2s_controller_init(void)
{
	unsigned int aicfr;
	unsigned int aiccr;
	//init cpm clock, use ext clock;

	ENTER();

	/* Select exclk as i2s clock */
	cpm_set_clock(CGU_I2SCLK, JZ_EXTAL);

	aicfr = (8 << 12) | (8 << 8) | (AIC_FR_ICDC | AIC_FR_LSMP | AIC_FR_AUSEL);
	REG_AIC_FR = aicfr;

	aiccr = REG_AIC_CR;
	aiccr &= (~(AIC_CR_EREC | AIC_CR_ERPL | AIC_CR_TDMS | AIC_CR_RDMS));
	REG_AIC_CR = aiccr;

	LEAVE();
}

static int __init init_jz_i2s(struct platform_device *pdev)
{
	struct i2s_codec *default_codec = &(the_codecs[0]);
	int errno;
	int fragsize;
	int fragstotal;

	printk("===>enter %s\n", __func__);

#ifdef CONFIG_JZ_EBOOK_HARD
//	printk("DEBUG: %s, %d\n", __FUNCTION__, __LINE__);
//	__cpm_start_aic1();
#endif

	cpm_start_clock(CGM_AIC);

	REG_AIC_I2SCR |= AIC_I2SCR_ESCLK;
	REG_AIC_I2SCR = 0x5a5a5a5a;
	printk("===>REG_AIC_I2SCR = 0x%0x\n", REG_AIC_I2SCR);

	i2s_controller_init();
	if (default_codec->codecs_ioctrl == NULL) {
		printk("default_codec: not ready!");
		return -1;
	}

	default_codec->codecs_ioctrl(default_codec, CODEC_SET_MODE, 0);
	//default_codec->codecs_ioctrl(default_codec, CODEC_INIT, 0);

	if ((errno = probe_jz_i2s(&the_i2s_controller)) < 0) {
		return errno;
	}

	//printk("===>enter %s:%d\n", __func__, __LINE__);
	//mdelay(5000);

	/* May be external CODEC need it ...
	 * default_codec->codecs_ioctrl(default_codec, CODEC_SET_GPIO_PIN, 0);
	 */
	attach_jz_i2s(the_i2s_controller);
	//printk("===>enter %s:%d\n", __func__, __LINE__);
	//mdelay(5000);

	/* Actually, the handler function of the command do nothing ...
	 * default_codec->codecs_ioctrl(default_codec, CODEC_SET_STARTUP_PARAM, 0);
	 * default_codec->codecs_ioctrl(default_codec, CODEC_SET_STARTUP_PARAM, 0);
	 */

	/* Now the command is not supported by DLV CODEC ...
	 * default_codec->codecs_ioctrl(default_codec, CODEC_SET_VOLUME_TABLE, 0);
	 */
	fragsize = JZCODEC_RW_BUFFER_SIZE * PAGE_SIZE;
	fragstotal = JZCODEC_RW_BUFFER_TOTAL;

	audio_init_endpoint(&out_endpoint, fragsize, fragstotal);
	audio_init_endpoint(&in_endpoint, fragsize, fragstotal);

#ifdef CONFIG_JZ_EBOOK_HARD
//	printk("DEBUG: %s, %d\n", __FUNCTION__, __LINE__);
//	__cpm_stop_aic1();
	audio_device_open = 0;
#endif

	printk("JZ I2S OSS audio driver initialized\n");

	LEAVE();

	return 0;
}

static void __exit cleanup_jz_i2s(void)
{
#ifdef CONFIG_PM
	/* pm_unregister(i2s_controller->pm); */
#endif
	struct i2s_codec *default_codec = &the_codecs[0];
	unload_jz_i2s(the_i2s_controller);
	the_i2s_controller = NULL;
	audio_deinit_endpoint(&out_endpoint);
	audio_deinit_endpoint(&in_endpoint);
	default_codec->codecs_ioctrl(default_codec, CODEC_CLEAR_MODE, 0);
}

static struct platform_driver snd_plat_driver = {
	.probe	=  init_jz_i2s,
	.driver	= {
		.name	= "mixer",
		.owner	= THIS_MODULE,
	},
	.suspend	= jz_i2s_suspend,
	.resume		= jz_i2s_resume,
};

static int __init snd_init(void)
{
	return platform_driver_register(&snd_plat_driver);
}

module_init(snd_init);
module_exit(cleanup_jz_i2s);
