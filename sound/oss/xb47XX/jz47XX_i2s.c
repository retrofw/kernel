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
#include <linux/soundcard.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <linux/proc_fs.h>
#include <linux/soundcard.h>
#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <asm/hardirq.h>

#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>

#ifdef CONFIG_ANDROID
#include <linux/earlysuspend.h>
#endif

#include "jz47XX_codec.h"
#include "jz47XX_dbg.h"
#include "jz47XX_bt_call.h"


#include <asm/jzsoc.h>

#ifdef CONFIG_AK5358
#include <linux/ak5358.h>
#endif

#ifdef CONFIG_I2S_DLV_NPCA110P
extern struct semaphore linein_is_playing;
extern int npca110p_register_state_notifier(struct notifier_block *nb);
extern void npca110p_unregister_state_notifier(struct notifier_block *nb);
extern int replay_last;
#endif

#define DEBUG_RECORD			0

#ifdef CONFIG_SOC_JZ4770
 #define CALL_RECORD_PLAYBACK
#endif

#define DMA_ID_I2S_TX			DMA_ID_AIC_TX
#define DMA_ID_I2S_RX			DMA_ID_AIC_RX

// reference to dma.c
#define DMA_TX_CHAN			6
#define DMA_RX_CHAN			7

#define NR_I2S				2

#define JZCODEC_RW_BUFFER_SIZE		1
#define JZCODEC_RW_BUFFER_TOTAL		8

#define AUDIOBUF_STATE_FREE		0

#define NOMAL_STOP			0
#define FORCE_STOP			1
#define PIPE_TRANS			1

#define AUDIO_LOCK(lock, flags)		spin_lock_irqsave(&lock, flags)
#define AUDIO_UNLOCK(lock, flags)	spin_unlock_irqrestore(&lock, flags)

#define THIS_AUDIO_NODE(p)		list_entry(p, audio_node, list)
#define ALIGN_PAGE_SIZE(x)		(((x) + PAGE_SIZE) / PAGE_SIZE * PAGE_SIZE)

unsigned int SPEAKER_FLAG = 0;

unsigned int DEFAULT_REPLAY_ROUTE = REPLAY_HP_STEREO;
unsigned int DEFAULT_RECORD_ROUTE = RECORD_MIC1_MONO_DIFF_WITH_BIAS;
unsigned int DEFAULT_CALL_RECORD_ROUTE = RECORD_STEREO_MIC_DIFF_WITH_BIAS_BYPASS_MIXER_MIC2_TO_OUT;

static struct i2s_codec *inter_codec = NULL;
static struct i2s_codec *exter_codec = NULL;
static int is_dsp_open = 0;
static struct semaphore hp_sem;

#ifdef CONFIG_HP_INSERT_GPIO_DETECT
#define GPIO_HP_SENSE   (32*5 + 0)          //tas5707 board use
static struct workqueue_struct *hp_work_queue;
static struct work_struct hp_gpio_irq_work;
#endif

static struct workqueue_struct *workqueue;
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
	unsigned int memsize;
	audio_node	*savenode;

	audio_node	*save_usernode;

    int *fragmem_start;

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

#if 0
struct i2s_codec
{
	/* I2S controller connected with */
	void	*private_data;
	void	*codec_private;
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
#endif

struct jz_mute{
	int bsp_mute;
	int bsp_mute_status;
	int codec_mute;
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
#ifdef CONFIG_AK5358
        struct ak5358   *ak5358;
#endif

	struct workqueue_struct *workqueue;
	struct delayed_work     mute_work;
	struct jz_mute 			mute;
};

enum aic_link_mode_t {
	LINK_EXTERNEL_AC97_CODEC,
	LINK_EXTERNEL_I2S_CODEC_AS_MASTER,
	LINK_EXTERNEL_I2S_CODEC_AS_SLAVE,
	LINK_EXTERNEL_HDMI_OR_MUILTY_CHANNEL_CODEC_VIA_I2S,
	LINK_EXTERNEL_HDMI_VIA_SPDIF,
	LINK_INTERNEL_I2S_CODEC_AS_MASTER
};

static int jz_i2s_suspend(struct platform_device *, pm_message_t state);
static int jz_i2s_resume(struct platform_device *);
static void jz_i2s_shutdown(struct platform_device *);
static int do_jz_mute(int enable, unsigned long delay);

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

/* For route selection, indicate that the current virtual device No. */
static unsigned int g_current_device = 0;
/* indicate if it is in-call */
static unsigned int g_in_call = 0;
unsigned int g_in_call_record;

/* BT call global definitions */
int bt_call_down_stream_thread(void *data);
int bt_call_up_stream_thread(void *data);
void bt_call_exit(void);
void bt_call_enter(void);

extern struct bt_call_pcm_info gbt_call_pcm;
static int bt_call_exit_flag = 1;

#define JZ_INTERNAL_CODEC  		0x1000
#define JZ_EXTERNAL_CODEC  		0x2000
#define JZ_I2S_EXTERNAL_CODEC  		0x2001
#define JZ_SPDIF_EXTERNAL_CODEC  	0x2002

#define DMA_RUN_ALLOW		0
#define DMA_REQUEST_DONE	0
#define DMA_STOP_REQUEST	1
struct dma_ctrl_info{
	unsigned int cmd;
	unsigned int ack;
	audio_pipe *ctrled_endpoint;
	audio_node *ctrled_node;
} dma_ctrl_info;
static struct dma_ctrl_info  g_dma_ctrl = {
	.cmd = DMA_RUN_ALLOW,
	.ack = 0,
	.ctrled_endpoint = NULL,
	.ctrled_node = NULL,
};
#ifdef CONFIG_SPDIF_DIGITAL_OUTPUT_DEFAULT
bool g_audio_spdif_fmt = true;
#else
bool g_audio_spdif_fmt = false;
#endif
static bool is_g_spdif_mode = false;

static void set_i2s_internal_codec(void);
static void set_i2s_external_codec(void);
static void set_spdif_external_codec(void);
static int set_internal_codec(struct jz_i2s_controller_info *controller);
static int set_external_codec(struct jz_i2s_controller_info *controller,bool is_spdif);
static int set_audio_codec(struct jz_i2s_controller_info *controller,unsigned int codec);
static unsigned int get_current_audio_codec(void);
static void dump_spdif_reg(void);
#ifdef CONFIG_ANDROID
static int is_i2s_suspended = 0;

static void jz_i2s_late_resume(struct early_suspend *h)
{
	int i;
	struct i2s_codec *codec;

	if (is_i2s_suspended)
		for(i = 0;i < NR_I2S; i++){
			codec = &the_codecs[i];
			if (codec && codec->codecs_ioctrl) {
				codec->codecs_ioctrl(codec, CODEC_RESUME, 0);
			}
		}

	is_i2s_suspended = 0;
}

static struct early_suspend jz_i2c_early_suspend = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .resume = jz_i2s_late_resume,
};
#endif

#if DEBUG_RECORD
struct file *f_test = NULL;
static unsigned long long f_test_offset = 0;
static mm_segment_t old_fs;
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

//	*(unsigned int *)0xb342010c = 0x18;

	printk("\tDRSR	= 0x%08x, addr = 0x%08x\n", REG_DMAC_DRSR(dmanr), DMAC_DRSR(dmanr));
	printk("\tDCCSR	= 0x%08x\n", REG_DMAC_DCCSR(dmanr));
	printk("\tDCMD	= 0x%08x\n", REG_DMAC_DCMD(dmanr));
	printk("\tDDA	= 0x%08x\n", REG_DMAC_DDA(dmanr));
	printk("\tDMADBR= 0x%08x\n", REG_DMAC_DMADBR(dmanr/HALF_DMA_NUM));
	printk("\tCPCCR = 0x%08x\n", REG_CPM_CPCCR);
//	printk("\tCPPCR = 0x%08x\n", REG_CPM_CPPCR);
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
		AUDIO_GET_CONFIG,	AUDIO_SET_CONFIG
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
		"AUDIO_GET_CONFIG",	"AUDIO_SET_CONFIG"
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
		SND_SET_DEVICE,		SND_SET_VOLUME,			SND_SET_STANDBY,
		SND_GET_NUM_ENDPOINTS,	SND_GET_ENDPOINT
	};

	char *cmd_str[] = {
		"SOUND_MIXER_INFO",		"SOUND_OLD_MIXER_INFO",		"SOUND_MIXER_READ_STEREODEVS",
		"SOUND_MIXER_READ_CAPS",	"SOUND_MIXER_READ_DEVMASK",	"SOUND_MIXER_READ_RECMASK",
		"SOUND_MIXER_READ_RECSRC",	"SOUND_MIXER_WRITE_SPEAKER",	"SOUND_MIXER_WRITE_BASS",
		"SOUND_MIXER_READ_BASS",	"SOUND_MIXER_WRITE_VOLUME",	"SOUND_MIXER_READ_VOLUME",
		"SOUND_MIXER_WRITE_MIC",	"SOUND_MIXER_READ_MIC",		"SOUND_MIXER_WRITE_LINE",
		"SOUND_MIXER_READ_LINE",	"SOUND_MIXER_WRITE_MUTE",	"SOUND_MIXER_READ_MUTE",
		"SND_SET_DEVICE",		"SND_SET_VOLUME",		"SND_SET_STANDBY",
		"SND_GET_NUM_ENDPOINTS",	"SND_GET_ENDPOINT"
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
	char *regname[] = {"aicfr","aiccr","aiccr1","aiccr2","i2scr","aicsr","acsr","i2ssr"};
	int i;
	unsigned int addr;

	printk("AIC regs dump, %s\n", str);
	for (i = 0; i < 0x1c; i += 4) {
		addr = 0xb0020000 + i;
		printk("%s\t0x%08x -> 0x%08x\n", regname[i/4], addr, *(unsigned int *)addr);
	}
}
//#endif
void write_aic_regs(int index, unsigned int val)
{
	char *regname[] = {"aicfr","aiccr","aiccr1","aiccr2","i2scr","aicsr","acsr","i2ssr",
			 "SPDIF_ENA","SPDIF_CTRL","SPDIF_STATE","SPDIF_CFG1","SPDIF_CFG2","SPDIF_FIFO"};
	int i;
	unsigned int addr;

	printk("AIC regs change before:\n");
	for (i = 0; i < 0x20; i += 4) {
		addr = 0xb0020000 + i;
		printk("[%d]%s\t0x%08x -> 0x%08x\n", (i/4),regname[i/4], addr, *(unsigned int *)addr);
	}
	for (i = 0; i < 0x18; i += 4) {
		addr = 0xb0020080 + i;
		printk("[%d]%s\t0x%08x -> 0x%08x\n",(8+i/4), regname[8+i/4], addr, *(unsigned int *)addr);
	}

	// change the regs
	if(index < 0 || index > 14) {
		printk("Permission denied !!! check it!\n");
		return;
	}

	if(index < 8){
		addr = 0xb0020000 + index*4;
	}else{
		addr = 0xb0020080 + (index - 8)*4;
	}

	printk("Before change: Reg[%s]\t0x%08x = 0x%08x\n", regname[index], addr, *(unsigned int *)addr);
	*(unsigned int *)addr = val;
	printk("After change : Reg[%s]\t0x%08x = 0x%08x\n\n", regname[index], addr, *(unsigned int *)addr);
}
#ifdef BUF_DEBUG
static void dump_buf(char *buf, int dump_len, int bytes_in_line)
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

int init_audio_node(unsigned int **memory, unsigned int pagesize, unsigned int count,int *fragmem_start)
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

//	dump_stack(); // shumb

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

	printk("JZ I2S: Mem alloc finish! memsize = 0x%x, fact = %d, mem = 0x%08x\n",
	       memsize, fact, (unsigned int)mem);

	// Free old buffer
	if (*memory) {
		audio_head *phead = (audio_head *)*memory;
		free_pages((unsigned long)*memory, phead->fact);
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
	*fragmem_start = (int)((unsigned int)*memory + headlistsize);
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
	return ALIGN_PAGE_SIZE(memsize);
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

static inline int get_audio_usenodecount(unsigned int *mem)
{
	struct list_head *puse;
	struct list_head *plist;
	audio_head *phead;
	int count = 0;

	phead = (audio_head *)mem;
	puse =  &(phead->use);
	plist = puse;
	while (plist->next != puse) {
		count++;
		plist = plist->next;
	}
	return count;
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
#if (!defined(CONFIG_SOC_JZ4760B)) && (!defined(CONFIG_SOC_JZ4770))
	if (dma_state & DMAC_DCCSR_INV) {
		err = 1;
		DPRINT_IRQ("!!!! DMA descriptor invalid\n");
	}
	if (dma_state & DMAC_DCCSR_CT) {
		DPRINT_IRQ("!!!! DMA descriptor finish\n");
	}
#endif
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
	__dmac_channel_enable_clk(6);
	__dmac_channel_enable_clk(7);

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
	//      ch, DMAC_DRSR(ch), DMAC_DRSR_RS_AICOUT, REG_DMAC_DRSR(ch));

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

static void config_dma_trans_mode(spinlock_t lock, audio_dma_type* dma, int sound_data_width,int burst_len)
{
	unsigned int	curmode;
	unsigned long	flags;
	unsigned int burstlen;
	ENTER();
	AUDIO_LOCK(lock, flags);
	curmode = *dma->trans_mode;
	switch(burst_len)
	{
	case 16:
		burstlen = DMAC_DCMD_DS_16BYTE;
		break;
	case 32:
		burstlen = DMAC_DCMD_DS_32BYTE;
		break;
	case 64:
		burstlen = DMAC_DCMD_DS_64BYTE;
		break;
	default:
		burstlen = DMAC_DCMD_DS_16BYTE;
	}
	dma->onetrans_bit = 8 * burst_len;
	if (dma->rw) {
		curmode &= ~(DMAC_DCMD_DWDH_MASK | DMAC_DCMD_DS_MASK);

		switch(sound_data_width) {
		case 8:
			*dma->trans_mode = (curmode | DMAC_DCMD_DWDH_8 | burstlen);

			break;
		case 16:
			*dma->trans_mode = (curmode | DMAC_DCMD_DWDH_16 | burstlen);
			break;
		case 17 ... 32:
			*dma->trans_mode = (curmode | DMAC_DCMD_DWDH_32 | burstlen);
			break;
		default:
			printk("JZ I2S: Unkown DMA mode(sound data width) %d\n", sound_data_width);
			break;
		}
	} else {
		curmode &= ~(DMAC_DCMD_SWDH_MASK | DMAC_DCMD_DS_MASK);
		switch(sound_data_width) {
		case 8:
			*dma->trans_mode = (curmode | DMAC_DCMD_SWDH_8 | burstlen);
			break;
		case 16:
			*dma->trans_mode = (curmode | DMAC_DCMD_SWDH_16 | burstlen);
			break;
		case 17 ... 32:
			*dma->trans_mode = (curmode | DMAC_DCMD_SWDH_32 | burstlen);
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

	if ((REG_DMAC_DCCSR(dma->ch) & DMAC_DCCSR_EN) == 0) {
		int count = node->end - node->start;
		*(dma->trans_addr) = node->phyaddr;
		if(!is_g_spdif_mode){
			*(dma->data_addr) =  (unsigned int)CPHYSADDR(AIC_DR);
		}else{
			*(dma->data_addr) =  (unsigned int)CPHYSADDR(SPDIF_FIFO);
		}
		*(dma->trans_count) = count * 8 / dma->onetrans_bit;
		REG_DMAC_DCCSR(dma->ch) = DMAC_DCCSR_NDES | DMAC_DCCSR_EN;
		DPRINT_DMA("virt = 0x%08x phy = 0x%08x, dma->onetrans_bit = 0x%x\n",
			   node->pBuf, node->phyaddr, dma->onetrans_bit);

		DUMP_CODEC_REGS(__FUNCTION__);
		DUMP_AIC_REGS(__FUNCTION__);
		start = 1;
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
#define I2S_TX_FIFO_DEPTH 64
#define I2S_RX_FIFO_DEPTH 32

static inline void set_controller_triger(struct jz_i2s_controller_info *controller,
					 audio_pipe *endpoint, short channels, short format)
{
	int sound_data_width = 0;
	int burst_len_rx = 32;
	int burst_len_tx = 64;
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
	case AFMT_S24_LE:
	case AFMT_S24_BE:
		sound_data_width = 24;
		break;
	default:
		printk("JZ I2S: Unkown sound format %d\n", format);
		return ;
	}


	if (endpoint == &out_endpoint) {
		config_dma_trans_mode(endpoint->lock,&(endpoint->dma), sound_data_width,burst_len_tx);
		if ((I2S_TX_FIFO_DEPTH - endpoint->dma.onetrans_bit / sound_data_width) >= 60) {
			__i2s_set_transmit_trigger(30);
		} else {
			__i2s_set_transmit_trigger(((I2S_TX_FIFO_DEPTH - endpoint->dma.onetrans_bit / sound_data_width) >> 1) - 1);
		}
	}
	if (endpoint == &in_endpoint) {
		config_dma_trans_mode(endpoint->lock,&(endpoint->dma), sound_data_width,burst_len_rx);
		__i2s_set_receive_trigger((endpoint->dma.onetrans_bit / sound_data_width) >> 1);
	}
	//printk("trigger = %d\n",(I2S_TX_FIFO_DEPTH - endpoint->dma.onetrans_bit / sound_data_width) / 2);
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

	dma_cache_wback((unsigned long)node->pBuf, endpoint->fragsize);
	start = audio_trystart_dma_node(&(endpoint->dma), node);
	if (start) {
		endpoint->trans_state |= PIPE_TRANS;
		endpoint->savenode = node;
		if(!is_g_spdif_mode){
			aic_enable_transmit();
		//	REG_AIC_CR |= AIC_CR_ETUR;
		}else{
			__spdif_enable();
		}
		do_jz_mute(0, 10);
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
		{
			__aic_flush_rfifo();
			__i2s_enable_record();
		}
		{
			/* read the first sample and ignore it */
			__i2s_read_rfifo();
			__i2s_enable_receive_dma();
		}

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

	if((endpoint->fragsize == pagesize)&&(endpoint->fragstotal == count)) {
		printk("Same pagesize && count !\n");
		return 1;
	}

	ret = init_audio_node(&endpoint->mem, pagesize, count,(int *)&endpoint->fragmem_start);
	if (ret) {
		endpoint->fragsize = pagesize;
		endpoint->fragstotal = count;
		endpoint->memsize = ret;
	}
	return ret;
}
static inline audio_node *endpoint_get_outnode(audio_pipe *endpoint)
{
	unsigned long	flags;
	audio_node	*node;
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

		//AUDIO_LOCK(endpoint->lock, flags);
		//DUMP_LIST((audio_head *)endpoint->mem);
		//DUMP_NODE(endpoint->savenode, "SN");
		//AUDIO_UNLOCK(endpoint->lock, flags);

		// wait available node
		wait_event_interruptible(endpoint->q_full, (endpoint->avialable_couter >= 1));

		AUDIO_LOCK(endpoint->lock, flags);
		node = get_audio_freenode(endpoint->mem);
		endpoint->avialable_couter = 0;
		AUDIO_UNLOCK(endpoint->lock, flags);
	}
	return node;
}
static inline int endpoint_post_outnode(audio_pipe *endpoint,audio_node	*node,int count)
{
	unsigned long	flags;
	dma_cache_wback((unsigned long)node->pBuf,(unsigned long)count);
	node->start = 0;
	node->end = count;
	AUDIO_LOCK(endpoint->lock, flags);
	put_audio_usenode(endpoint->mem, node);
	AUDIO_UNLOCK(endpoint->lock, flags);
	return count;
}
static inline void endpoint_start_outdma(struct jz_i2s_controller_info *controller,audio_pipe *endpoint){
	unsigned long	flags;
	audio_node	*node;
	AUDIO_LOCK(endpoint->lock, flags);
	if ((endpoint->trans_state & PIPE_TRANS) == 0) {
		node = get_audio_usenode(endpoint->mem);
		if (node) {
			unsigned int start;
			start = trystart_endpoint_out(controller, node);
			if (start == 0) {
				printk("JZ I2S: trystart_endpoint_out error\n");
			}
		}
	}
	AUDIO_UNLOCK(endpoint->lock, flags);
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
			endpoint->avialable_couter = 1;
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

static int keep_zero_playing = 0;
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

	node = get_audio_usenode(endpoint->mem);
#if 0
	if(the_i2s_controller->mute.bsp_mute_status == 0 && node == NULL){
		keep_zero_playing = 1;
	}
	if(keep_zero_playing && the_i2s_controller->mute.bsp_mute_status == 1){
		keep_zero_playing = 0;
		printk("release\n");
	}
#endif
	if (endpoint->savenode) {
		if(keep_zero_playing == 0){
		put_audio_freenode(endpoint->mem, endpoint->savenode);
		DPRINT_IRQ("put_audio_freenode\n");
		endpoint->savenode = NULL;
		} else{
			printk("Keep savenode!\n");
			node = endpoint->savenode;
			do_jz_mute(1, 0);
			node->start = 0;
			node->end = endpoint->fragsize; // fix me!
			memset((void *)node->pBuf,0,(node->end - node->start));
		}

		if (!(endpoint->is_non_block)) {
			endpoint->avialable_couter = 1;
			wake_up_interruptible(&endpoint->q_full);
		}
	}

	if (node) {
		int start;
		if(g_dma_ctrl.cmd == DMA_STOP_REQUEST){
			printk("DMA_STOP_REQUEST coming !\n");
			g_dma_ctrl.cmd = DMA_REQUEST_DONE;
			g_dma_ctrl.ack = 1;
			g_dma_ctrl.ctrled_endpoint = endpoint;
			g_dma_ctrl.ctrled_node = node;
		}else{
			start = audio_trystart_dma_node(&(endpoint->dma), node);
			if (start == 0) {
			printk("audio_out_endpoint_work audio_trystart_dma_node error!\n");
		} else {
			endpoint->savenode = node;
				DPRINT_DMA("restart dma!\n");
			}
		}
	} else {
		endpoint->trans_state &= ~PIPE_TRANS;
		if(!is_g_spdif_mode){
		//	aic_disable_transmit();
			int dat = REG_AIC_CR;
	//		dat &= ~(AIC_CR_TDMS | AIC_CR_ERPL);
			dat &= ~(AIC_CR_TDMS );
			REG_AIC_CR = dat;
		}else{
			__spdif_disable();
		}
//		REG_AIC_FR &= ~AIC_FR_LSMP;
//		if(__aic_transmit_underrun()){
//			printk("Underrun,replay stop!\n");
//		}
		printk("Stop AIC!\n");
		REG_AIC_CR &= ~AIC_CR_ERPL;
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

void register_jz_codecs_ex(void *func,void *codec_private)
{
	int i;

	ENTER();

	for (i = 0; i < NR_I2S; i++) {
		if (the_codecs[i].codecs_ioctrl == 0) {
			printk("register codec %x\n",(unsigned int)func);
			the_codecs[i].id = i;
			the_codecs[i].name = JZ_I2S_EXTERNAL_CODEC;
			the_codecs[i].audio_volume = INIT_VOLUME;
			the_codecs[i].codec_private = codec_private;
			the_codecs[i].codecs_ioctrl = func;
			sema_init(&(the_codecs[i].i2s_sem),1);
			break;
		}
	}

	LEAVE();
}
void register_jz_codecs(void *func)
{
	int i;

	ENTER();

	for (i = 0; i < NR_I2S; i++) {
		if (the_codecs[i].codecs_ioctrl == 0) {
			printk("register codec %x\n",(unsigned int)func);
			the_codecs[i].id = i;
			the_codecs[i].name = JZ_INTERNAL_CODEC;
			the_codecs[i].audio_volume = INIT_VOLUME;
			the_codecs[i].codecs_ioctrl = func;
			sema_init(&(the_codecs[i].i2s_sem),1);
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
#define nonblk_codec_ioctrl(codec, cmd, args) ({			\
	int result;						\
	result = (codec)->codecs_ioctrl((codec), (cmd), (args));\
	result;							\
})

static int jz_i2s_open_mixdev(struct inode *inode, struct file *file)
{
	return 0;
}

int read_reg(unsigned int reg)
{
	int val;
	val = REG32(reg);
	printk("read  reg(0x%08x)=0x%08x\n",reg,val);
	return val;
}
int write_reg(unsigned int reg, unsigned int val)
{
	REG32(reg) = val;
	printk("write reg(0x%08x)=0x%08x\n",reg,val);
	return 0;
}
void test_start(void)
{
	int i=0;
	printk("Test start:\n");

	printk("gpio regs:\n");
	for(i=0;i<6;i++){
		printk("GPIO_PXDAT(%d)=0x%08x\n",i,REG_GPIO_PXDAT(i));
	}
	printk("\ncpm clk:\n");
	printk("CPM_CLKGR0=0x%08x\n",REG_CPM_CLKGR0);
	printk("CPM_CLKGR1=0x%08x\n",REG_CPM_CLKGR1);
}
static bool g_is_spdif_fmt = false;
/*
 * Debug entry for Android
 */

#ifdef CONFIG_I2S_DLV_TAS5707
extern int tas5707_check_err_status(void);
extern void tas5707_dump_all(void);
#endif
#ifdef CONFIG_I2S_DLV_4760
extern void dump_dlv_regs(void);
#endif

static int jz_i2s_write_mixdev(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct jz_i2s_controller_info *controller = the_i2s_controller;
	struct i2s_codec *codec = (struct i2s_codec *)controller->i2s_codec;
	char	buf_byte = 0;

	char    str_arg[23];
	char	*pstr,*tstr,str_val[11];
	unsigned int Reg=0,Val=0,cur_codec=0;

	str_val[10] = 0;
	pstr = str_arg;
	tstr = pstr;

	if (copy_from_user((void *)&buf_byte, buffer, 1)) {
		printk("JZ MIX: copy_from_user failed !\n");
		return -EFAULT;
	}
	if (copy_from_user((void *)&str_arg, buffer, 23)) {
                printk("JZ MIX arg: copy_from_user failed !\n");
        }


	switch (buf_byte) {
	case '0':
		//test_start();
		break;
	case '1':
		dump_aic_regs("");
		dump_spdif_reg();
#ifdef DMA_DEBUG
		dump_dma(controller->pout_endpoint->dma.ch,"");
#endif
		cur_codec = get_current_audio_codec();
		if(cur_codec == JZ_INTERNAL_CODEC){
			printk("Using internal audio codec!\n");
		}else if(cur_codec == JZ_I2S_EXTERNAL_CODEC){
			printk("Using external I2S audio codec!\n");
		}else if(cur_codec == JZ_SPDIF_EXTERNAL_CODEC){
			printk("Using external SPDIF audio codec!\n");
		}else{
			printk("Using unknown audio codec(0x%08x)!\n",cur_codec);
		}
#ifdef CONFIG_I2S_DLV_4760
		dump_dlv_regs();
#endif
		break;
	case '2':
#ifdef CONFIG_I2S_DLV_TAS5707
		printk("dump tas5707 registers\n");
		tas5707_dump_all();
#endif
		break;
	case '3':
#ifdef CONFIG_I2S_DLV_TAS5707
		printk("clear TAS5707 0x02 ERR register\n");
		tas5707_check_err_status();
#endif
		break;
	case '4':
		break;
	case 'w':
		// example: "w 0xb1000012=0x00000001"
		if(*(pstr+1) != ' ' && *(pstr+11) != '='){
			printk("Not Reg write operation!\n");
			return -1;
		}
		memcpy(str_val,pstr+2,10);
		Reg = (unsigned int)simple_strtoul(str_val,&tstr,0);
		memcpy(str_val,pstr+13,10);
		Val = (unsigned int)simple_strtoul(str_val,&tstr,0);
		if( Reg == 0 ){
			printk("write:Error Reg(0x0) = 0x%08x\n",Val);
			return -1;
		}
		write_reg(Reg,Val);
		read_reg(Reg);
		break;
	case 's':
		if(str_arg[1] == '+'){
			printk("SND switch to external codec!\n");
			printk("Codec --- %s !!!\n", g_is_spdif_fmt?"SPDIF":"I2S");
			set_external_codec(controller,g_is_spdif_fmt);
			g_is_spdif_fmt = g_is_spdif_fmt?false:true;
		}else if(str_arg[1] == '-'){
			printk("SND switch to internal codec!\n");
			set_internal_codec(controller);
		}else{
			printk("Usage:\t's[+|-][0|1]'\n");
		}
	}

	return count;
}

#ifndef CONFIG_JZ_EXTERNAL_CODEC_I2S_CONFIGURE
#define __gpio_as_ex_i2s()	\
do{				\
	printk("audio external i2s codec gpio NEED configured!!!\n");\
}while(0)
#endif

static void dump_spdif_reg(void)
{
//      printk("REG_DMAC_DMACKE(0) : 0x%08x\n",REG_DMAC_DMACKE(0));
//      printk("REG_DMAC_DMACKE(1) : 0x%08x\n",REG_DMAC_DMACKE(1));

        printk("\n");
        printk("REG_AIC_FR : 0x%08x\n",REG_AIC_FR);
        printk("REG_AIC_CR : 0x%08x\n",REG_AIC_CR);
        printk("REG_AIC_I2SCR : 0x%08x\n",REG_AIC_I2SCR);
        printk("REG_AIC_I2SDIV : 0x%08x\n",REG_AIC_I2SDIV);

        printk("REG_SPDIF_ENA : 0x%08x\n",REG_SPDIF_ENA);
        printk("REG_SPDIF_CTRL : 0x%08x\n",REG_SPDIF_CTRL);
        printk("REG_SPDIF_STATE : 0x%08x\n",REG_SPDIF_STATE);
        printk("REG_SPDIF_CFG1: 0x%08x\n",REG_SPDIF_CFG1);
        printk("REG_SPDIF_CFG2 : 0x%08x\n",REG_SPDIF_CFG2);
        printk("\n");
}

static void set_spdif_output(void)
{
	int i;
	//__aic_disable_transmit_dma();

	__spdif_enable_conpro();
	__spdif_enable_audion();
	__spdif_disable_sign();

	__spdif_enable_initlvl_low();
	__spdif_set_ch1num(1);
	__spdif_set_ch2num(2);
	__spdif_set_srcnum(2);


	/* sample rate 44.1 khz*/
	__spdif_set_fs(0x0);
	__spdif_set_orgfrq(0xf);

	/* sample size 16bit */
	__spdif_enable_samwl_20();
	__spdif_set_samwl(1);


	__spdif_set_transmit_trigger(32);
	__spdif_enable_transmit_dma();
	__spdif_enable_reset();
	for(i = 0; i < 10; i++ ){
		if(!(REG_SPDIF_CTRL & SPDIF_CTRL_RST)) break;
		mdelay(50);
	}
	if(REG_SPDIF_CTRL & SPDIF_CTRL_RST){
		printk("Warning: spdif reset fifo timeout !!!\n");
		REG_SPDIF_CTRL &= ~SPDIF_CTRL_RST;
	}
	DPRINT("Spdif FIFO rested !\n");
	__spdif_enable_MTRIGmask();
	__spdif_enable_MFFURmask();

	__spdif_disable_invalid();
	__spdif_select_spdif();
	__spdif_enable();
	//dump_spdif_reg();
}

static int request_dma_stop(struct jz_i2s_controller_info *controller)
{
	int transmiting = controller->pout_endpoint->trans_state & PIPE_TRANS;
	if( transmiting ){
		int i;

		DPRINT("Out endpoint is transmiting, and it will be stop!\n");
		g_dma_ctrl.cmd = DMA_STOP_REQUEST;

		i = 10;
		while(i--){
			if(g_dma_ctrl.ack == 1){
				break;
			}
			mdelay(100);
		}

		if(g_dma_ctrl.ack == 1){
			DPRINT("DMA ACK Done!\n");
			return 1;
		}else{
			DPRINT("Warning: [audio] request dma stop fail!!!\n");
		}
	}else{
		DPRINT("Out endpoint transmit end!\n");

	}
	return 0;
}
static int request_dma_restart(struct jz_i2s_controller_info *controller)
{
	int stat = 0;
	int transmiting = controller->pout_endpoint->trans_state & PIPE_TRANS;
	if( transmiting ){
		int start;
		audio_pipe *endpoint = g_dma_ctrl.ctrled_endpoint;
		audio_node *node = g_dma_ctrl.ctrled_node;

		if(g_dma_ctrl.ack == 1){
			g_dma_ctrl.ack = 0;
			if(endpoint != NULL && node != NULL){
				DPRINT("request_dma_restart: audio_trystart_dma_node()\n");
				start = audio_trystart_dma_node(&(endpoint->dma), node);
				if (start == 0) {
					printk("audio_out_endpoint_work audio_trystart_dma_node error!\n");
					stat = -1;
				} else {
					endpoint->savenode = node;
					printk("Restart dma Done!\n");
				}
			}else{
				printk("some problem occured!!! (endpoint = %p ,node = %p)\n",endpoint,node);
			}
		}
	}

	if(g_dma_ctrl.cmd == DMA_STOP_REQUEST){
		g_dma_ctrl.cmd = DMA_REQUEST_DONE;
	}

	return stat;
}
static int g_jz_i2s_codec = -1;
static void set_i2s_external_codec()
{
	g_jz_i2s_codec = JZ_I2S_EXTERNAL_CODEC;

	__spdif_disable_transmit_dma();
	__spdif_select_i2s();
	__spdif_disable();
	__i2s_disable();

	//cpm_set_clock(CGU_I2SCLK, 11289600);  // 11289600 Hz need by external i2s codec for FS = 44.1KHz
	cpm_set_clock(CGU_I2SCLK, 12288000);  // 12288000 Hz need by external i2s codec for FS = 48 KHz
	//cpm_set_clock(CGU_I2SCLK, JZ_EXTAL);

	__gpio_as_ex_i2s();

	__i2s_external_codec();

	__i2s_send_lfirst();       //send left channel first, it's some external CODEC requirement.

#ifdef CONFIG_JZ_I2S_AS_SLAVE
	__i2s_as_slave();
#else
	__i2s_as_master();
#endif

	__i2s_enable();

}
static void set_spdif_external_codec()
{

	g_jz_i2s_codec = JZ_SPDIF_EXTERNAL_CODEC;

	__i2s_disable();
	cpm_set_clock(CGU_I2SCLK, 2*11289600 );  // 11289600*2 Hz need by external spdif codec for FS = 44.1KHz

	__gpio_as_ex_i2s();

	__i2s_external_codec();

	__i2s_send_lfirst();       //send left channel first, it's some external CODEC requirement.

#ifdef CONFIG_JZ_I2S_AS_SLAVE
	__i2s_as_slave();
#else
	__i2s_as_master();
#endif

	set_spdif_output();
	__spdif_enable_transmit_dma();

}
static int  set_external_codec(struct jz_i2s_controller_info *controller, bool is_spdif)
{
	int dma_stat = 0;
	unsigned int new_codec = is_spdif ? JZ_SPDIF_EXTERNAL_CODEC:JZ_I2S_EXTERNAL_CODEC;

	printk("SND use external %s codec!\n",is_spdif?"SPDIF":"I2S");
	if(get_current_audio_codec() == new_codec){
		printk("Same with the current audio codec(0x%x)!\n",new_codec);
		return 0;
	}

	dma_stat = request_dma_stop(controller);

	if(is_spdif){
		set_spdif_external_codec();
		is_g_spdif_mode = true;
	}else{
		set_i2s_external_codec();
		is_g_spdif_mode = false;
	}
	if(dma_stat == 1){
		request_dma_restart(controller);
		if(!is_spdif){
			__aic_enable_transmit_dma();
			__aic_enable_replay();
		}
	}
	return 0;
}
static void set_i2s_internal_codec(void)
{

	g_jz_i2s_codec = JZ_INTERNAL_CODEC;

	__spdif_disable_transmit_dma();
	__spdif_select_i2s();
	__spdif_disable();

	__i2s_disable();
	cpm_set_clock(CGU_I2SCLK, JZ_EXTAL); //  12000000 Hz need by internal CODEC

	__i2s_send_rfirst();            //send right channel first, it's internal CODEC requirement.

	__i2s_as_slave();
	__i2s_internal_codec();
	__i2s_enable();
}
static int set_internal_codec(struct jz_i2s_controller_info *controller)
{
	int dma_stat = 0;

	printk("SND use internal codec!\n");
	if(get_current_audio_codec() == JZ_INTERNAL_CODEC)
		return 0;

	dma_stat = request_dma_stop(controller);

	g_jz_i2s_codec = JZ_INTERNAL_CODEC;

	is_g_spdif_mode = false;


	set_i2s_internal_codec();

	if(dma_stat == 1){
		request_dma_restart(controller);
		__aic_enable_transmit_dma();
		__aic_enable_replay();

	}

	return 0;
}

static int set_audio_codec(struct jz_i2s_controller_info *controller,unsigned int codec)
{
	int err = 0;
	switch(codec){
	case JZ_INTERNAL_CODEC:
		set_internal_codec(controller);
		break;
	case JZ_EXTERNAL_CODEC:
		if(g_audio_spdif_fmt)
			set_external_codec(controller,true);
		else
			set_external_codec(controller,false);
		break;
	case JZ_I2S_EXTERNAL_CODEC:
		set_external_codec(controller,false);
		break;
	case JZ_SPDIF_EXTERNAL_CODEC:
		set_external_codec(controller,true);
		break;
	default:
		printk("The audio codec(0x%08x) is unavailable!!!\n",codec);
		err = -1;
		break;

	}
	return err;
}
static unsigned int get_current_audio_codec(void)
{
	return g_jz_i2s_codec;
}
/*
 * Handle IOCTL request on /dev/mixer
 *
 * Support OSS IOCTL interfaces for /dev/mixer
 * Support IOCTL interfaces for /dev/mixer defined in include/msm_audio.h
 */

static int jz_aic_link_format_config(int format);
void jz_codec_dac_mute_enable(struct i2s_codec *codec);
void jz_codec_dac_mute_disable(struct i2s_codec *codec);
static int g_in_hdmi = 0;

//static int jz_i2s_ioctl_mixdev(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
static long jz_i2s_ioctl_mixdev(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct jz_i2s_controller_info *controller = the_i2s_controller;
	struct i2s_codec *codec = (struct i2s_codec *)controller->i2s_codec;
	long	val = 0;
	int	i,ret, rc = 0;

	ENTER();
	DPRINT_IOC(" mixer IOCTL %s cmd = 0x%08x, arg = %lu\n", __FUNCTION__, cmd, arg);
	DPRINT_MIXER_IOC_CMD(cmd);

	switch (cmd) {
	/*
	 * OSS IOCTL commands for /dev/mixer
	 */
	case SOUND_MIXER_INFO:
		break;
	case SOUND_OLD_MIXER_INFO:
		break;
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
		break;
	case SOUND_MIXER_WRITE_BASS:
		break;
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
		codec_ioctrl(codec, CODEC_SET_REPLAY_VOLUME, val);
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
		codec_ioctrl(codec, CODEC_SET_MIC_VOLUME, val);
		return 0;

	case SOUND_MIXER_READ_MIC:
		val = codec->mic_gain;
		ret = val << 8;
		val = val | ret;
		return put_user(val, (long *) arg);

	case SOUND_MIXER_WRITE_LINE:
		break;

	case SOUND_MIXER_READ_LINE:
		val = codec->mic_gain;
		ret = val << 8;
		val = val | ret;
		return put_user(val, (long *) arg);

	case SOUND_MIXER_WRITE_MUTE:
		get_user(codec->audiomute, (long *)arg);
		codec_ioctrl(codec, CODEC_DAC_MUTE, codec->audiomute);
		break;

	case SOUND_MIXER_READ_MUTE:
		put_user(codec->audiomute, (long *) arg);
		break;

	/*
	 * MSM IOCTL commands for /dev/mixer
	 */
	case SND_SET_DEVICE:
	{
		static unsigned long flags;
		struct snd_device_config dev;
		if (copy_from_user(&dev, (void *)arg, sizeof(dev))) {
			rc = -EFAULT;
			break;
		}

		if (dev.device == SND_DEVICE_HDMI) {
			//codec_ioctrl(codec, CODEC_DAC_MUTE, 1);
			jz_codec_dac_mute_enable(controller->i2s_codec);
			jz_aic_link_format_config(LINK_EXTERNEL_HDMI_OR_MUILTY_CHANNEL_CODEC_VIA_I2S);
			g_in_hdmi = 1;
			break;
		} else {
			if (g_in_hdmi) {
				jz_aic_link_format_config(LINK_INTERNEL_I2S_CODEC_AS_MASTER);
				//codec_ioctrl(codec, CODEC_DAC_MUTE, 0);
				jz_codec_dac_mute_disable(controller->i2s_codec);
				g_in_hdmi = 0;
			}
		}

		/* to avoid exchange R&L channel while replay */
		if(dev.device == SND_DEVICE_HEADSET){
			//printk("T resident:%d  R count:%d\n",__aic_get_transmit_resident(),__aic_get_receive_count());
			//dump_aic_regs("");
			__aic_disable_transmit_dma();
			//aic_disable_transmit();
			/* delay 30ms at most to wait for TXFIFO empty */
			for(i=0;i<30;i++)
			{
				if(__aic_get_transmit_resident() == 0) break;
				mdelay(1);
			}
			if(__aic_get_transmit_resident() != 0){
				printk("wait %d ms for TXFIFO empty\n",i);
				printk("Tx resident:%d  Rx count:%d\n",__aic_get_transmit_resident(),__aic_get_receive_count());
				printk("Warnning:it's likely to exchange R&L channel!!!\n");

				__aic_flush_tfifo();
			}

			__aic_disable_replay();
			//dump_aic_regs("");
		}

		if (dev.device == SND_DEVICE_BT_EC_OFF)
			dev.device = SND_DEVICE_BT;

		if (g_current_device == SND_DEVICE_BT && !bt_call_exit_flag) {
			if (dev.device != SND_DEVICE_BT) {
				/* we should exit bt call mode here */
				bt_call_exit_flag = 1;
		//		bt_call_exit();
			}
		}

		AUDIO_LOCK(the_i2s_controller->pout_endpoint->lock, flags);

		g_current_device = dev.device;
		g_in_call = !dev.ear_mute;

		AUDIO_UNLOCK(the_i2s_controller->pout_endpoint->lock, flags);

		codec_ioctrl(codec, CODEC_SET_DEVICE, (unsigned long)&dev);

		if(dev.device == SND_DEVICE_HEADSET){
			__aic_enable_transmit_dma();
			__aic_enable_replay();
			//aic_enable_transmit();
		}

		if (dev.device == SND_DEVICE_BT && bt_call_exit_flag){
			/* we should enter bt call mode here */
			bt_call_exit_flag = 0;
	//		bt_call_enter();
		}

		printk("\n\n########### SND_SET_DEVICE ###########\n");
		//dump_route_device(dev.device);
		printk(", device = %d, ear_mute = %d, mic_mute = %d\n"
		       "########### SND_SET_DEVICE ###########\n\n",
		       dev.device, dev.ear_mute, dev.mic_mute);
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
		// mic volume ??? need fix
		codec_ioctrl(codec, CODEC_SET_MIC_VOLUME, (unsigned int)&val);
		break;
	}

	case SND_SET_STANDBY:
	{
		unsigned long flags;
		ret = get_user(val, (unsigned int *)arg);
		if (ret) {
			printk("Mixer IOCTL argument error, command = SND_SET_STANDBY\n");
		}

		AUDIO_LOCK(the_i2s_controller->pout_endpoint->lock, flags);
		if (g_in_call) {
			AUDIO_UNLOCK(the_i2s_controller->pout_endpoint->lock, flags);
			break;
		}
		AUDIO_UNLOCK(the_i2s_controller->pout_endpoint->lock, flags);

		codec_ioctrl(codec, CODEC_SET_STANDBY, (unsigned int)val);

		break;
	}
	case SND_SELECT_CODEC:
	{
		ret = get_user(val, (unsigned int *)arg);
		if(ret)	printk("Mixer IOCTL argument error, command = SND_SET_CODEC\n");

		#if 0
		if(val == JZ_INTERNAL_CODEC){
			set_internal_codec(controller);
			printk("SND use internal codec!\n");
		}else if(val == JZ_EXTERNAL_CODEC){
			set_external_codec(controller);
			printk("SND use external codec!\n");
		}else{
			printk(KERN_ERR "set codec fail (codec=0x%x)!!!\n",(unsigned int)val);
			val = 0;
			put_user(val, (long *) arg);
			return -EFAULT;
		}
		#else

		if(val == get_current_audio_codec())
			printk("Same audio codec(0x%08x)!\n",(unsigned int)val);
		else
			set_audio_codec(controller,val);

		#endif

		put_user(get_current_audio_codec(), (long *) arg);
		break;
	}
	case SND_CODEC_SET_MUTE:
	{
		ret = get_user(val, (unsigned int *)arg);
		printk("SND_CODEC_SET_MUTE:val=%d\n",(unsigned int)val);
		if(val == 0)
			do_jz_mute(0, 0);
		else
			do_jz_mute(1, 0);
		break;
	}
#if 0
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

	LEAVE();
	return rc;
}

static struct file_operations jz_i2s_mixer_fops =
{
	owner:		THIS_MODULE,
	unlocked_ioctl:	jz_i2s_ioctl_mixdev,
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
	int mixer_num;

	ENTER();

	if((mixer_num = register_sound_mixer(&jz_i2s_mixer_fops, 0)) < 0){
		printk(KERN_ERR "JZ I2S: couldn't register mixer!\n");
		LEAVE();
		return -1;
	}

	for (i = 0; i < NR_I2S; i++) {
		if (i2s_probe_codec(&the_codecs[i]) == 0) {
			break;
		}
		the_codecs[i].private_data = controller;
		the_codecs[i].dev_mixer = mixer_num;
	}

	LEAVE();
	return i;
}

static void jz_i2s_reinit_hw(int mode)
{
	ENTER();

	__i2s_disable_record();
	__i2s_disable_replay();
	__i2s_disable_loopback();
	__i2s_set_transmit_trigger(4);
	__i2s_set_receive_trigger(3);

#ifdef	CONFIG_JZ4760_HDMI_DISPLAY
	__gpio_as_func1(3*32 + 12); //blck
	__gpio_as_func0(3*32 + 13); //sync
	__gpio_as_func0(4*32 + 7);  //sd0
#endif
	LEAVE();
}

static int jz_codec_set_speed(struct i2s_codec *codec, int rate, int mode)
{
	ENTER();

	/* 8000, 11025, 16000, 22050, 24000, 32000, 44100, 48000, 99999999 ? */
	if (mode & CODEC_RMODE) {
		rate = codec_ioctrl(codec, CODEC_SET_RECORD_RATE, rate);
		if (rate > 0) {
			codec->record_audio_rate = rate;
		} else {
			rate = codec->record_audio_rate;
		}
	}
	if (mode & CODEC_WMODE) {
		rate = codec_ioctrl(codec, CODEC_SET_REPLAY_RATE, rate);
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
			__aic_out_channel_select(0);
			__aic_enable_mono2stereo();
		} else {
			__aic_out_channel_select(1);
			__aic_disable_mono2stereo();
		}
	}

	LEAVE();

	return channels;
}

static void jz_codec_select_mode(struct i2s_codec *codec, int mode)
{
	ENTER();

	g_in_call_record = 0;

	switch (mode) {
	case CODEC_WRMODE:
		if (codec->use_mic_line_flag == USE_NONE) {
			codec->use_mic_line_flag = USE_MIC;
		}
		codec_ioctrl(codec, CODEC_SET_ROUTE, DEFAULT_REPLAY_ROUTE);
		break;
	case CODEC_RMODE:
		if (codec->use_mic_line_flag == USE_NONE) {
			codec->use_mic_line_flag = USE_MIC;
		}
		if(g_in_call){
			codec_ioctrl(codec, CODEC_SET_ROUTE, DEFAULT_CALL_RECORD_ROUTE);
			g_in_call_record = 1;
		}
		else
			codec_ioctrl(codec, CODEC_SET_ROUTE, DEFAULT_RECORD_ROUTE);
		break;
	case CODEC_WMODE:
		codec_ioctrl(codec, CODEC_SET_ROUTE, DEFAULT_REPLAY_ROUTE);
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

void jz_codec_dac_mute_enable(struct i2s_codec *codec)
{
	ENTER();
	codec_ioctrl(codec, CODEC_DAC_MUTE, 1);
	LEAVE();
}

void jz_codec_dac_mute_disable(struct i2s_codec *codec)
{
	ENTER();
	codec_ioctrl(codec, CODEC_DAC_MUTE, 0);
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

static int jz_aic_link_format_config(int format)
{
	switch (format) {
	case LINK_EXTERNEL_AC97_CODEC:
		break;

	case LINK_EXTERNEL_I2S_CODEC_AS_MASTER:
		break;

	case LINK_EXTERNEL_I2S_CODEC_AS_SLAVE:
		break;

	case LINK_EXTERNEL_HDMI_OR_MUILTY_CHANNEL_CODEC_VIA_I2S:
	{
		__aic_external_codec();
		__aic_select_i2s();
		{
			__i2s_stop_bitclk();
			__i2s_as_master();
			__i2s_start_bitclk();
		}
		__aic_flush_tfifo();
		__aic_unflush_rfifo();
		break;
	}

	case LINK_EXTERNEL_HDMI_VIA_SPDIF:
		break;

	case LINK_INTERNEL_I2S_CODEC_AS_MASTER:
	default:
		__aic_internal_codec();
		__aic_select_i2s();
		__i2s_select_i2s();
		{
			__i2s_stop_bitclk();
			__i2s_as_slave();
			__i2s_start_bitclk();
		}
		__aic_flush_tfifo();
		__aic_unflush_rfifo();
	}

	return 0;
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
	return (data_len >> 1);
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
	return (data_len >> 1);
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
	int data_cnt_ushort = data_len_32aligned >> 1;
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
	return (data_len >> 1);
}

/*
 * convert normal 16bit stereo data to mono data
 *
 * buff:	buffer address
 * data_len:	data length in kernel space, the length of stereo data
 *
 */
int convert_16bits_stereomix2mono(void *buff, int data_len)
{
	/* stride = 32 bytes = 2 channels * 2 byte * 8 pipelines */
	int data_len_32aligned = data_len & ~0x1f;
	int data_cnt_ushort = data_len_32aligned >> 1;
	int left_cur, right_cur, mono_cur;
	short *ushort_buff = (short *)buff;
	/*init*/
	left_cur = 0;
	right_cur = left_cur + 1;
	mono_cur = 0;
	/*because the buff's size is always 4096 bytes,so it will not lost data*/
	while(right_cur < data_cnt_ushort)
	{
		ushort_buff[mono_cur + 0] = ((ushort_buff[left_cur + 0]) + (ushort_buff[right_cur + 0]));
		ushort_buff[mono_cur + 1] = ((ushort_buff[left_cur + 2]) + (ushort_buff[right_cur + 2]));
		ushort_buff[mono_cur + 2] = ((ushort_buff[left_cur + 4]) + (ushort_buff[right_cur + 4]));
		ushort_buff[mono_cur + 3] = ((ushort_buff[left_cur + 6]) + (ushort_buff[right_cur + 6]));
		ushort_buff[mono_cur + 4] = ((ushort_buff[left_cur + 8]) + (ushort_buff[right_cur + 8]));
		ushort_buff[mono_cur + 5] = ((ushort_buff[left_cur + 10]) + (ushort_buff[right_cur + 10]));
		ushort_buff[mono_cur + 6] = ((ushort_buff[left_cur + 12]) + (ushort_buff[right_cur + 12]));
		ushort_buff[mono_cur + 7] = ((ushort_buff[left_cur + 14]) + (ushort_buff[right_cur + 14]));

		left_cur += 16;
		right_cur = left_cur + 1;
		mono_cur += 8;
	}

	LEAVE();
	return (data_len >> 1);
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
				if(g_in_call)
					endpoint->filter = convert_16bits_stereomix2mono;
				else
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
	__aic_disable_unsignadj();
	__aic_disable_byteswap();

	switch (format) {
	case AFMT_U8:
		data_width = 8;
		if (mode & CODEC_RMODE) {
			__i2s_set_iss_sample_size(8);
		}
		if (mode & CODEC_WMODE) {
			__i2s_set_oss_sample_size(8);
		}
		__aic_enable_unsignadj();
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
		__aic_enable_byteswap();
		break;
	case AFMT_S24_LE:
                data_width = 24;
                if (mode & CODEC_RMODE) {
                        __i2s_set_iss_sample_size(24);
                }
                if (mode & CODEC_WMODE) {
                        __i2s_set_oss_sample_size(24);
                }
                break;
        case AFMT_S24_BE:
                data_width = 24;
                if (mode & CODEC_RMODE) {
                        __i2s_set_iss_sample_size(24);
                }
                if (mode & CODEC_WMODE) {
                        __i2s_set_oss_sample_size(24);
                }
		__aic_enable_byteswap();
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
	struct jz_i2s_controller_info *controller = the_i2s_controller;
	int mode = 0;

	ENTER();

	down(&(hp_sem));

	is_dsp_open = 0;

	if (controller == NULL) {
		printk("\nAudio device not ready!\n");
		return -ENODEV;
	}
	if((file->f_mode & FMODE_READ) && (file->f_mode & FMODE_WRITE))
		return 0;
	if ((controller->pin_endpoint == NULL) && (controller->pout_endpoint == NULL) ) {
		printk("\nAudio endpoint not open!\n");
		return -ENODEV;
	}

	do_jz_mute(1, 0);

	if ((file->f_mode & FMODE_READ) && controller->pin_endpoint) {
		printk("Read mode, %s\n", __FUNCTION__);

#if DEBUG_RECORD
		printk("DEBUG:----close /data/record_test.pcm-%s\tline:%d\n",__func__,__LINE__);
		filp_close(f_test, NULL);
#endif

		mode |= CODEC_RMODE;
		audio_close_endpoint(controller->pin_endpoint, FORCE_STOP);
		controller->pin_endpoint = NULL;

		__i2s_disable_receive_dma();
		__i2s_disable_record();

		if (g_in_call_record) {
#ifdef CALL_RECORD_PLAYBACK
			struct jz_i2s_controller_info *controller = the_i2s_controller;
			struct i2s_codec *codec = controller->i2s_codec;
			jz_codec_set_speed(codec, 44100, CODEC_WMODE);
#endif
			g_in_call_record = 0;
		}
	}

	if ((file->f_mode & FMODE_WRITE) && controller->pout_endpoint) {
		printk("Write mode, %s\n", __FUNCTION__);
		mode |= CODEC_WMODE;
		audio_close_endpoint(controller->pout_endpoint, NOMAL_STOP);
		controller->pout_endpoint = NULL;

		__i2s_disable_transmit_dma();
		mdelay(10);
		__i2s_disable_replay();
	}

	jz_codec_close(controller->i2s_codec, mode);

	if ((controller->pin_endpoint == NULL) && (controller->pout_endpoint == NULL) ) {
		__i2s_disable();
	}

	up(&(hp_sem));
	LEAVE();
	return 0;
}

#ifdef CONFIG_HP_INSERT_GPIO_DETECT
/* You can change the HP detect GPIO according to your board */
static int get_hp_station(void)
{
     if(__gpio_get_pin(GPIO_HP_SENSE))
		 return 1;
	 else
		 return 0;
}

/* When HP station change, irq will call the function */
int hp_insert_event(void)
{
	int val;
	struct i2s_codec *codec;
	struct i2s_codec *old_codec;

	down(&(hp_sem));
	if(is_dsp_open)
	{
		if((val = get_hp_station()) == 1)
		{
			if((g_jz_i2s_codec != JZ_INTERNAL_CODEC)&&(inter_codec != NULL)){
				old_codec = the_i2s_controller->i2s_codec;
				the_i2s_controller->i2s_codec = inter_codec;
				codec = the_i2s_controller->i2s_codec;

				/* store replay volume */
				codec->audio_volume = old_codec->audio_volume;
				codec_ioctrl(old_codec, CODEC_SET_REPLAY_VOLUME, 0x0);
				/* 1: stop old codec I2S clks */
				__i2s_disable_transmit_dma();
				mdelay(5);
				__i2s_disable_replay();
				/* 2: close old codec */
				jz_codec_close(old_codec, CODEC_WMODE);

				/* 3: support current codec I2S clks */
				set_i2s_internal_codec();

				/* 4: reset current codec */
				jz_codec_anti_pop(codec, CODEC_WMODE);

				jz_codec_set_channels(codec, old_codec->replay_codec_channel, CODEC_WMODE);
				jz_codec_set_format(codec, old_codec->replay_format, CODEC_WMODE);
				jz_codec_set_speed(codec, old_codec->replay_audio_rate, CODEC_WMODE);

				jz_codec_select_mode(codec, CODEC_WMODE);

				/* 5: start replay, dac mute just for anti_pop sound */
				jz_codec_dac_mute_enable(codec);
				__i2s_enable_transmit_dma();
				__i2s_enable_replay();
				jz_codec_dac_mute_disable(codec);
				//mdelay(5);
				/* restore replay volume */
				codec_ioctrl(codec, CODEC_SET_REPLAY_VOLUME, codec->audio_volume);
			}
		}else{

			if((g_jz_i2s_codec != JZ_I2S_EXTERNAL_CODEC)&&(exter_codec != NULL)){
				old_codec = the_i2s_controller->i2s_codec;
				the_i2s_controller->i2s_codec = exter_codec;
				codec = the_i2s_controller->i2s_codec;

				/* store replay volume */
				codec->audio_volume = old_codec->audio_volume;
				codec_ioctrl(old_codec, CODEC_SET_REPLAY_VOLUME, 0x0);
				/* 1: stop old codec I2S clks */
				__i2s_disable_transmit_dma();
				mdelay(5);
				__i2s_disable_replay();
				/* 2: close old codec */
				jz_codec_close(old_codec, CODEC_WMODE);

				/* 3: support current codec I2S clks */
				set_i2s_external_codec();

				/* 4: reset current codec */
				jz_codec_anti_pop(codec, CODEC_WMODE);

				jz_codec_set_channels(codec, old_codec->replay_codec_channel, CODEC_WMODE);
				jz_codec_set_format(codec, old_codec->replay_format, CODEC_WMODE);

				jz_codec_set_speed(codec, old_codec->replay_audio_rate, CODEC_WMODE);

				jz_codec_select_mode(codec, CODEC_WMODE);

				/* 5: start replay, dac mute just for anti_pop sound */
				jz_codec_dac_mute_enable(codec);
				__i2s_enable_transmit_dma();
				__i2s_enable_replay();
				jz_codec_dac_mute_disable(codec);
				//mdelay(5);
				/* restore replay volume */
				codec_ioctrl(codec, CODEC_SET_REPLAY_VOLUME, codec->audio_volume);
			}
		}
	}else
		printk("hp change station\n");

	up(&(hp_sem));

	return 0;
}
#endif

static int jz_audio_open(struct inode *inode, struct file *file)
{
	struct jz_i2s_controller_info *controller = the_i2s_controller;
	struct i2s_codec *codec = controller->i2s_codec;
	int mode = 0;
	int val = 0;
	int reset = 1;
	audio_node *node;

	ENTER();

	if (controller == NULL) {
		return -ENODEV;
	}

	if((file->f_mode & FMODE_READ) && (file->f_mode & FMODE_WRITE)){
		if(controller->pout_endpoint)
		{
			return 0;
		}else
			return -ENODEV;
	}
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
#ifdef CONFIG_AK5358
        if (controller->ak5358) {
                int ret = 0;
                ret = down_interruptible(&controller->ak5358->is_playing);
                if (ret < 0)
                        return ret;
        }
#endif
#ifdef CONFIG_I2S_DLV_NPCA110P
        {
                int ret = 0;
                ret = down_interruptible(&linein_is_playing);
                if (ret < 0)
                        return ret;
        }
#endif

	do_jz_mute(1, 0);

	if (file->f_mode & FMODE_WRITE) {
		controller->pout_endpoint = &out_endpoint;
		controller->pout_endpoint->is_non_block = file->f_flags & O_NONBLOCK;
		mode |= CODEC_WMODE;
	}
	if (file->f_mode & FMODE_READ) {
		controller->pin_endpoint = &in_endpoint;
		controller->pin_endpoint->is_non_block = file->f_flags & O_NONBLOCK;
		mode |= CODEC_RMODE;
#ifdef CALL_RECORD_PLAYBACK
		if (g_in_call){
			audio_close_endpoint(&out_endpoint, NOMAL_STOP);

			/* disable replay */
			__i2s_disable_replay();
			mdelay(1);
			__aic_flush_tfifo();
			mdelay(1);

			/* enable replay */
			__i2s_enable_replay();
		}
#endif
	}

	down(&(hp_sem));
#ifdef CONFIG_HP_INSERT_GPIO_DETECT
	/* when /dev/dsp open, ensure that wether HP insert, and do codec switch */
	if((val = get_hp_station()) == 1)
	{
		if((g_jz_i2s_codec != JZ_INTERNAL_CODEC)&&(inter_codec != NULL)){
			printk("set inter codec clk >>>\n");
			controller->i2s_codec = inter_codec;
			codec = controller->i2s_codec;
			set_i2s_internal_codec();
		}
	}else{

		if((g_jz_i2s_codec != JZ_I2S_EXTERNAL_CODEC)&&(exter_codec != NULL)){
			printk("set exter codec clk >>>\n");
			controller->i2s_codec = exter_codec;
			codec = controller->i2s_codec;
			set_i2s_external_codec();
		}
	}
#endif
	is_dsp_open = 1;

	/* we should turn codec and anti-pop first */
	jz_codec_anti_pop(controller->i2s_codec, mode);

	if (mode & CODEC_RMODE){
#if DEBUG_RECORD
		printk("DEBUG:----open /data/record_test.pcm-%s\tline:%d\n",__func__,__LINE__);
		f_test = filp_open("/data/record_test.pcm", O_RDWR | O_CREAT | O_APPEND, S_IRUSR | S_IWUSR);
		f_test_offset = 0;
#endif

#ifdef CALL_RECORD_PLAYBACK
		if (g_in_call) {
			jz_codec_set_speed(codec, 8000, CODEC_WMODE);
		}
#endif

		jz_codec_set_channels(codec, 2, CODEC_RMODE);
		jz_codec_set_format(codec, 16, CODEC_RMODE);
		jz_codec_set_speed(codec, 44100, CODEC_RMODE);
		codec->user_need_mono = 0;

		set_controller_triger(controller, &in_endpoint, codec->record_codec_channel, codec->record_format);
	}

	if (mode & CODEC_WMODE) {
		jz_codec_set_channels(codec, 2, CODEC_WMODE);
		jz_codec_set_format(codec, 16, CODEC_WMODE);
		jz_codec_set_speed(codec, 48000, CODEC_WMODE);
		set_controller_triger(controller, &out_endpoint, codec->replay_codec_channel, codec->replay_format);
	}
	/* set codec replay & record route */
	jz_codec_select_mode(codec, mode);

	/* anti pop sound method */
	if (reset) {
		/* play a half node 0x00 to let the pop noise go away */
		if (file->f_mode & FMODE_WRITE) {
			node = get_audio_freenode(out_endpoint.mem);
			node->start = 0;
			node->end = out_endpoint.fragsize >> 1;
			memset((void *)node->pBuf, 0x00, out_endpoint.fragsize >> 1);
			trystart_endpoint_out(controller, node);
		}

		/* enable codec dac mute to avoid pop noise */
		jz_codec_dac_mute_enable(controller->i2s_codec);
		
		/* open i2s */
		down(&controller->i2s_codec->i2s_sem);
		__i2s_enable();
		up(&controller->i2s_codec->i2s_sem);

		/* disable dac mute */
		jz_codec_dac_mute_disable(controller->i2s_codec);
	}

/*########################################################################################################*/
//	printk("SPEAKER_FLAG = %d\n", SPEAKER_FLAG);
	if ( SPEAKER_FLAG )                          //This is pretest test code,please not delete.
	{
		struct snd_device_config dev;
		dev.device = SND_DEVICE_SPEAKER;
		dev.ear_mute = 1;
		codec_ioctrl(codec, CODEC_SET_DEVICE, (unsigned long)&dev);

		SPEAKER_FLAG = 0;
	}
	do_jz_mute(0, 0);
	up(&(hp_sem));
#ifdef CONFIG_AK5358
        if (controller->ak5358)
                up(&controller->ak5358->is_playing);
#endif
#ifdef CONFIG_I2S_DLV_NPCA110P
        up(&linein_is_playing);
#endif
	DPRINT_TRC(".... jz_audio_open\n");
	LEAVE();
	return 0;
}

//static int jz_audio_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
static long jz_audio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long rc = -EINVAL;
	int	val = 0;
	int	mode = 0;

	struct jz_i2s_controller_info *controller = the_i2s_controller;
	struct i2s_codec *codec = controller->i2s_codec;
	audio_pipe	*pin_endpoint = controller->pin_endpoint;
	audio_pipe	*pout_endpoint = controller->pout_endpoint;

	ENTER();

	DPRINT_IOC("[dsp IOCTL] --------------------------------\n");
	DPRINT_IOC(" dsp IOCTL %s cmd = (0x%08x), arg = %lu\n", __FUNCTION__, cmd, arg);
	DPRINT_DSP_IOC_CMD(cmd);
	DPRINT_IOC("[dsp IOCTL] --------------------------------\n");

#ifdef CONFIG_AK5358
        if (controller->ak5358) {
                int ret = 0;
                ret = down_interruptible(&controller->ak5358->is_playing);
                if (ret < 0)
                        return ret;
        }
#endif
#ifdef CONFIG_I2S_DLV_NPCA110P
        {
                int ret = 0;
                ret = down_interruptible(&linein_is_playing);
                if (ret < 0)
                        return ret;
        }
#endif

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
	case SNDCTL_DSP_GETBLKSIZE:{
		// It seems that device could only be open with one mode (R or W)
		int fragsize = 0;
		if (mode & CODEC_RMODE) {
			fragsize = pin_endpoint->fragsize * pin_endpoint->fragstotal;
		}
		if (mode & CODEC_WMODE) {
			fragsize = pout_endpoint->fragsize * pout_endpoint->fragstotal;
		}
		rc = put_user(fragsize, (int *)arg);
		break;
	}
	case SNDCTL_EXT_DIRECT_GETNODE:{
		if (mode & CODEC_WMODE) {
			direct_info info;
			audio_node *node = endpoint_get_outnode(pout_endpoint);
			if(node)
			{
				pout_endpoint->save_usernode = node;
				info.bytes = pout_endpoint->fragsize;
				info.offset = (int)((unsigned int)node->pBuf - (unsigned int)pout_endpoint->fragmem_start);
				//printk("pBuf = %p\n",node->pBuf);
			}else{
				info.bytes = 0;
				info.offset = -1;
			}
			//printk("info.offset = %d\n",info.offset);
			rc = copy_to_user((void *)arg, &info, sizeof(info)) ? -EFAULT : 0;
		}
		break;
	}
	case SNDCTL_EXT_DIRECT_SETNODE:{
		//printk("SNDCTL_EXT_DIRECT_SETNODE!\n");
		if (mode & CODEC_WMODE) {
			direct_info info;

			rc = copy_from_user((void *)&info, (void *)arg, sizeof(info)) ? -EFAULT : 0;
			if(!rc)
			{
				//printk("%02x %02x %02x %02x\n",buf[0],buf[1],buf[2],buf[3]);
				endpoint_post_outnode(pout_endpoint,pout_endpoint->save_usernode,info.bytes);
				endpoint_start_outdma(controller,pout_endpoint);
			}
			break;
		}
	}
	case SNDCTL_DSP_GETFMTS:
		/* Returns a mask of supported sample format*/
		rc = put_user(AFMT_U8 | AFMT_S16_LE | AFMT_S24_LE, (int *)arg);
		break;

	case SNDCTL_DSP_SETFMT:
		/* Select sample format */
		if (get_user(val, (int *)arg)) {
			rc = -EFAULT;
		}

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
                        rc = -EINVAL;
                        goto error;
		}
		audio_get_endpoint_freesize(pout_endpoint, &abinfo);
		rc = copy_to_user((void *)arg, &abinfo, sizeof(abinfo)) ? -EFAULT : 0;
		break;
	}

	case SNDCTL_DSP_GETISPACE:
	{
		audio_buf_info abinfo;
		if (!(mode & CODEC_RMODE)) {
                        rc = -EINVAL;
                        goto error;
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
			rc = put_user(codec->record_codec_channel, (int *)arg);
		}
		if (mode & CODEC_WMODE) {
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

	case AUDIO_GET_CONFIG:
 		break;

	case AUDIO_SET_CONFIG:
		break;

	default:
		printk("%s[%s]:%d---no cmd\n",__FILE__,__FUNCTION__,__LINE__);
		break;
	}

	LEAVE();
error:
#ifdef CONFIG_AK5358
        if (controller->ak5358)
                up(&controller->ak5358->is_playing);
#endif
#ifdef CONFIG_I2S_DLV_NPCA110P
        up(&linein_is_playing);
#endif
	return rc;
}

static inline int endpoint_put_userdata(audio_pipe *endpoint, const char __user *buffer, size_t count)
{
	audio_node	*node;

	ENTER();
	DPRINT("<<<< put_userdata\n");
	node = endpoint_get_outnode(endpoint);
	if(!node)
		return 0;
	if (copy_from_user((void *)node->pBuf, buffer, count)) {
		printk("JZ I2S: copy_from_user failed !\n");
		return -EFAULT;
	}
	LEAVE();
	return endpoint_post_outnode(endpoint,node,count);
}
static ssize_t jz_audio_write_data(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	/* we have to use global data to aviod crash when BT call come in*/
	struct jz_i2s_controller_info *controller = the_i2s_controller;
	audio_pipe	*pout_endpoint = controller->pout_endpoint;
	size_t	usecount = 0;
	int	bat_cnt = -1;
	int	rem_cnt = 0;
	unsigned long	flags;

	ENTER();

	/* Ignore any write operations while calling. */
	AUDIO_LOCK(pout_endpoint->lock, flags);
	if (g_in_call) {
		AUDIO_UNLOCK(pout_endpoint->lock, flags);

		//printk("IN-CALL mode: ignore any write operations to audio device ...\n");
		return count;
	}
	AUDIO_UNLOCK(pout_endpoint->lock, flags);

	DPRINT("write data count = %d\n", count);
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
		endpoint_start_outdma(controller,pout_endpoint);
		if(controller->mute.bsp_mute_status != 0){
		//	printk("bsp_mute_status=%d , should disable mute!\n",controller->mute.bsp_mute_status);
		}
	}

	DPRINT("----write data usecount = %d, count = %d\n", usecount, count);
	BUG_ON(count < 0);
	LEAVE();

	return usecount + (count < 32 ? count : 0);
}

static ssize_t jz_audio_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
#ifdef CONFIG_ANDROID
	if (is_i2s_suspended) {
		/* if device is suspended,
		   write it will do nothing,
		   but do not return -EIO
		*/
		return 0;
	}
#endif

	if((file->f_mode & FMODE_WRITE) && (file->f_mode & FMODE_READ)) {
		direct_info info;
		int rc;
		struct jz_i2s_controller_info *controller = the_i2s_controller;
		audio_pipe	*pout_endpoint = controller->pout_endpoint;

		rc = copy_from_user((void *)&info, (void *)buffer, sizeof(info)) ? -EFAULT : 0;
		if(!rc)
		{
			//unsigned char * buf = (unsigned char *)pout_endpoint->save_usernode->pBuf;

			//printk("%02x %02x %02x %02x\n",buf[0],buf[1],buf[2],buf[3]);
			endpoint_post_outnode(pout_endpoint,pout_endpoint->save_usernode,info.bytes);
			endpoint_start_outdma(controller,pout_endpoint);
			pout_endpoint->save_usernode = NULL;
		}else
			return -EIO;
		return sizeof(info);
	}else if(file->f_mode & FMODE_WRITE)
		return jz_audio_write_data(file, buffer, count, ppos);
	return -EIO;

}
/**
 *  Copy recorded sound data from 'use' link list to userspace
 */
static inline int endpoint_get_userdata(audio_pipe *endpoint, const char __user *buffer, size_t count)
{
#ifdef CALL_RECORD_PLAYBACK
	struct jz_i2s_controller_info *controller = the_i2s_controller;
	audio_node	*temp_out_node;
#endif
	unsigned long	flags;
	audio_node	*node;
	int	ret;

	/* counter for node buffer, raw data */
	int	node_buff_cnt = 0;
	/* counter for node buffer after filte, fixed data */
	int	fixed_buff_cnt = 0;

	ENTER();

	AUDIO_LOCK(endpoint->lock, flags);
	node = get_audio_usenode(endpoint->mem);
	AUDIO_UNLOCK(endpoint->lock, flags);

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
	}

	if (node && (node_buff_cnt = node->end - node->start)) {
		DPRINT("node_buff_cnt = %d, count = %d\n", node_buff_cnt, count);

#if DEBUG_RECORD
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		vfs_write(f_test, (void *)(node->pBuf + node->start),
			  node->end - node->start, &f_test_offset);

		set_fs(old_fs);
#endif

#ifdef CALL_RECORD_PLAYBACK
		/* here playback the node, and in codec, use dac left only, in such way, the mic2 data (in left channel)
		 can be playback */
		if (g_in_call) {
			/* get an free out node */
			wait_event_interruptible(out_endpoint.q_full, out_endpoint.avialable_couter >= 1);

			AUDIO_LOCK(out_endpoint.lock, flags);
			temp_out_node = get_audio_freenode(out_endpoint.mem);
			AUDIO_UNLOCK(out_endpoint.lock, flags);

			/* the out node clone the in node */
			if (temp_out_node) {
				memcpy((void *)temp_out_node->pBuf, (void *)node->pBuf, node_buff_cnt);
				temp_out_node->start = node->start;
				temp_out_node->end = node->end;

				if((out_endpoint.trans_state & PIPE_TRANS) == 0) {
					/* clear out fifo */
					printk("START AIC OUT\n");
					if (trystart_endpoint_out(controller, temp_out_node) == 0) {
						printk("JZ I2S: trystart_endpoint_out error\n");
					}
				} else {
					printk("put audio node\n");
					AUDIO_LOCK(out_endpoint.lock, flags);
					put_audio_usenode(out_endpoint.mem, temp_out_node);
					AUDIO_UNLOCK(out_endpoint.lock, flags);
				}
			}
		}
#endif

		if (endpoint->filter) {
			/* ret indicate that final data length when copy_to_user
			 * (node->end - node->start) may not equals to ret !
			 */
			fixed_buff_cnt = endpoint->filter((void *)(node->pBuf + node->start), node_buff_cnt);
		} else {
			fixed_buff_cnt = node_buff_cnt;
		}

		if (count >= (size_t)fixed_buff_cnt) {
			DPRINT(">>>> count >= fixed_buff_cnt, copy_to_user, fixed_buff_cnt = %d\n", fixed_buff_cnt);
			ret = copy_to_user((void *)buffer, (void *)(node->pBuf + node->start), fixed_buff_cnt);
			if (ret) {
				printk("JZ I2S: copy_to_user failed, return %d\n", ret);
				return -EFAULT;
			}
			put_audio_freenode(endpoint->mem, node);
		} else {
			DPRINT(">>>> count < fixed_buff_cnt, copy_to_user, fixed_buff_cnt = %d\n", fixed_buff_cnt);
			ret = copy_to_user((void *)buffer,(void *)(node->pBuf + node->start), count);
			if (ret) {
				printk("JZ I2S: copy_to_user failed, return %d\n", ret);
				return -EFAULT;
			}
			node->start += node_buff_cnt;
		}
	}

	LEAVE();
	return (fixed_buff_cnt < count ? fixed_buff_cnt : count);
}

static ssize_t jz_audio_read_data(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	/* we have to use global data to aviod crash when BT call come in*/
	struct jz_i2s_controller_info *controller = the_i2s_controller;
	audio_pipe	*pin_endpoint = controller->pin_endpoint;
	audio_node	*node;
	unsigned long	flags;
	int		mcount, usecount = 0;

	ENTER();
#ifdef CONFIG_ANDROID
	if (is_i2s_suspended) {
		/* if device is suspended,
		   read data frome it will
		   return -EIO,
		*/
		return -EIO;
	}
#endif

	if (count == 0) {
		return 0;
	}

	AUDIO_LOCK(pin_endpoint->lock, flags);

	DUMP_LIST((audio_head *)pin_endpoint->mem);
	DUMP_NODE(pin_endpoint->savenode, "SN");

	if ((pin_endpoint->trans_state & PIPE_TRANS) == 0) {
		node = get_audio_freenode(pin_endpoint->mem);
		if (node) {
			unsigned int start;
			node->end = pin_endpoint->fragsize;

			start = trystart_endpoint_in(controller, node);
			if (start == 0) {
				put_audio_freenode(pin_endpoint->mem, node);
			}
		}
	}
	AUDIO_UNLOCK(pin_endpoint->lock, flags);

	do{
		mcount = endpoint_get_userdata(pin_endpoint, &buffer[usecount], count);

		if (mcount < 0) {
			if (usecount > 0) {
				break;
			} else {
				return mcount;
			}
		} else if (mcount == 0) {
			break;
		} else {
			usecount += mcount;
			count -= mcount;
		}
	} while (count > 0);

	LEAVE();
	return usecount;
}
static ssize_t jz_audio_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	if((file->f_mode & FMODE_WRITE) && (file->f_mode & FMODE_READ)) {
		struct jz_i2s_controller_info *controller = the_i2s_controller;
		audio_pipe	*pout_endpoint = controller->pout_endpoint;

		direct_info info;
		int rc;
		audio_node *node = endpoint_get_outnode(pout_endpoint);

		if(pout_endpoint->save_usernode) {
			put_audio_freenode(pout_endpoint->mem, pout_endpoint->save_usernode);
			pout_endpoint->save_usernode = NULL;
		}

		if(node)
		{
			pout_endpoint->save_usernode = node;
			info.bytes = pout_endpoint->fragsize;
			info.offset = (int)((unsigned int)node->pBuf - (unsigned int)pout_endpoint->fragmem_start);
		}else{
				info.bytes = 0;
				info.offset = -1;
		}
		//printk("info.offset = %d\n",info.offset);
		rc = copy_to_user((void *)buffer, &info, count) ? -EFAULT : 0;
		return sizeof(info);
	}else if(file->f_mode & FMODE_READ)
		return jz_audio_read_data(file, buffer, count, ppos);
	return -EIO;

}
static int jz_audio_mmap(struct file *file, struct vm_area_struct *vma){
	struct jz_i2s_controller_info *controller = the_i2s_controller;
	unsigned long start;
	unsigned long off;
	u32 len;
	if (!(file->f_mode & FMODE_WRITE)){
		printk("no write device not map!");
	}
	if(!controller->pout_endpoint){
		printk("\nAudio write device isn't ready!\n");
		return -EBUSY;
	}
	off = vma->vm_pgoff << PAGE_SHIFT;
	start = (unsigned long)virt_to_phys((void *)controller->pout_endpoint->fragmem_start);
	len = PAGE_ALIGN(controller->pout_endpoint->fragstotal * controller->pout_endpoint->fragsize);
	start &= PAGE_MASK;
	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;

 	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot)) {

		return -EAGAIN;
	}
	return 0;
}

/* static struct file_operations jz_i2s_audio_fops */
static struct file_operations jz_i2s_audio_fops = {
	owner:		THIS_MODULE,
	open:		jz_audio_open,
	release:	jz_audio_release,
	write:		jz_audio_write,
	read:		jz_audio_read,
	unlocked_ioctl:	jz_audio_ioctl,
	mmap:       jz_audio_mmap
};

static void __init attach_jz_i2s(struct jz_i2s_controller_info *controller)
{
	char* name = NULL;
	int	adev = 0; /* No of Audio device. */

	ENTER();

	name = controller->name;
	/* Initialize I2S CODEC and register /dev/mixer. */
	if (jz_i2s_codec_init(controller) <= 0) {
		goto mixer_failed;
	}

	/* Initialize AIC controller and reset it. */
	jz_i2s_reinit_hw(1);
	adev = register_sound_dsp(&jz_i2s_audio_fops, -1);
	if (adev < 0) {
		goto audio_failed;
	}
	controller->dev_audio = adev;

	LEAVE();
	return;
audio_failed:
	unregister_sound_dsp(adev);
mixer_failed:
	LEAVE();
	return;
}

static void __exit unload_jz_i2s(struct jz_i2s_controller_info *controller)
{
	jz_i2s_reinit_hw(0);
}

//--------------------------------------------------------------------

static int jz_i2s_suspend(struct platform_device *pd, pm_message_t state)
{
	int i;
	struct i2s_codec *codec;

	for(i = 0;i < NR_I2S; i++){
		codec = &the_codecs[i];
		if (codec && codec->codecs_ioctrl) {
			codec->codecs_ioctrl(codec, CODEC_SUSPEND, 0);
		}
	}

#ifdef CONFIG_ANDROID
	is_i2s_suspended = 1;
#endif

	printk("Aic and codec are suspended!\n");
	return 0;
}

static int jz_i2s_resume(struct platform_device *pd)
{
#ifndef CONFIG_ANDROID
	int i;
        struct i2s_codec *codec;
	for(i = 0;i < NR_I2S; i++){
		codec = &the_codecs[i];
		if (codec && codec->codecs_ioctrl) {
			codec->codecs_ioctrl(codec, CODEC_RESUME, 0);
		}
	}
#endif
	return 0;
}

static void jz_i2s_shutdown(struct platform_device *pd)
{
	int i;
	struct i2s_codec *codec;
	for(i = 0;i < NR_I2S; i++){
		codec = &the_codecs[i];
		if (codec && codec->codecs_ioctrl) {
			codec->codecs_ioctrl(codec, CODEC_SHUTDOWN, 0);
			codec->codecs_ioctrl(codec, CODEC_TURN_OFF, CODEC_WMODE | CODEC_RMODE);
		}
	}
}

static int __init probe_jz_i2s(struct jz_i2s_controller_info **controller)
{
	struct jz_i2s_controller_info *ctrl;

	ENTER();
	ctrl = kzalloc(sizeof(struct jz_i2s_controller_info), GFP_KERNEL);
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

void i2s_controller_init(int codec_id)
{
	unsigned int aicfr;
	unsigned int aiccr;
	//init cpm clock, use ext clock;
	//unsigned int sys_clk = JZ_EXTAL; // 12000000 Hz need by internal CODEC

	ENTER();
	//cpm_set_clock(CGU_I2SCLK, sys_clk);
	aicfr = (8 << 12) | (8 << 8) | (AIC_FR_ICDC | AIC_FR_LSMP | AIC_FR_AUSEL);
	REG_AIC_FR = aicfr;

	aiccr = REG_AIC_CR;
	aiccr &= (~(AIC_CR_EREC | AIC_CR_ERPL | AIC_CR_TDMS | AIC_CR_RDMS));
	REG_AIC_CR = aiccr;

	/* Select exclk as i2s clock */
	if(codec_id == JZ_I2S_EXTERNAL_CODEC){
		is_g_spdif_mode = false;
		set_i2s_external_codec();
	}
	else if(codec_id == JZ_SPDIF_EXTERNAL_CODEC)
	{
		//if(g_audio_spdif_fmt){
		is_g_spdif_mode = true;
		set_spdif_external_codec();
		//}
	}
	else if(codec_id == JZ_INTERNAL_CODEC){
		is_g_spdif_mode = false;
		set_i2s_internal_codec();
	}

	LEAVE();
}

static int do_jz_mute(int enable, unsigned long delay)
{
	int ret = -1;
	struct i2s_codec *codec = the_i2s_controller->i2s_codec;
	if(the_i2s_controller->mute.bsp_mute_status == enable){
		return 0;
	}
	the_i2s_controller->mute.bsp_mute = enable;
	if(0 == delay){
		ret = nonblk_codec_ioctrl(codec, CODEC_BSP_MUTE, enable);
		the_i2s_controller->mute.bsp_mute_status = enable;
	}else{
		ret = queue_delayed_work(the_i2s_controller->workqueue, &(the_i2s_controller->mute_work), delay);
	}
	return ret;
}



void i2s_mute_work_handler(struct work_struct *work)
{
	struct jz_i2s_controller_info *controller = container_of(work, struct jz_i2s_controller_info, mute_work.work);
	struct i2s_codec *codec = controller->i2s_codec;
	int enable,ret = -1;

	enable = controller->mute.bsp_mute;
	ret = nonblk_codec_ioctrl(codec, CODEC_BSP_MUTE, enable);
	//ret = codec_ioctrl(codec, CODEC_BSP_MUTE, enable);
	controller->mute.bsp_mute_status = enable;
}

#ifdef CONFIG_HP_INSERT_GPIO_DETECT
static void hp_insert_irq_work_handler(struct work_struct *work)
{
	hp_insert_event();
	return;
}

static irqreturn_t hp_insert_gpio_irq(int irq, void *dev_id)
{
	int state;
	state = __gpio_get_pin(GPIO_HP_SENSE);
	mdelay(5);
	state = __gpio_get_pin(GPIO_HP_SENSE);

	if(state)
		__gpio_as_irq_fall_edge(GPIO_HP_SENSE);
	else
		__gpio_as_irq_rise_edge(GPIO_HP_SENSE);

	queue_work(hp_work_queue, &hp_gpio_irq_work);
	return IRQ_HANDLED;
}

static void enable_hp_sense_irq(void)
{
	int state;
	state = __gpio_get_pin(GPIO_HP_SENSE);
	if(state)
		__gpio_as_irq_fall_edge(GPIO_HP_SENSE);
	else
		__gpio_as_irq_rise_edge(GPIO_HP_SENSE);

	return;
}
#endif

#ifdef CONFIG_AK5358
static int linein_dete_notifier(struct notifier_block *nb,
                                unsigned long val, void *data)
{
        struct ak5358 *ak5358 = (struct ak5358 *)data;
        struct jz_i2s_controller_info *controller = the_i2s_controller;
        struct i2s_codec *codec = controller->i2s_codec;

        if (controller->ak5358 == NULL)
                controller->ak5358 = ak5358;

        if (val) {
                down(&ak5358->is_playing);
                ak5358->replay_last = REG_AIC_CR & AIC_CR_ERPL;

                if (ak5358->replay_last) {
                        __aic_disable_replay();
                        __i2s_stop_bitclk();
                        ak5358->fs_last = codec->replay_audio_rate;
                        ak5358->channel_last = codec->replay_codec_channel;
                        ak5358->format_last = codec->replay_format;
                }

                jz_codec_anti_pop(codec, CODEC_WMODE);
                jz_codec_set_speed(codec, ak5358->fs, CODEC_WMODE);
                jz_codec_set_channels(codec, 2, CODEC_RMODE);
                jz_codec_set_format(codec, AFMT_S24_LE, CODEC_WMODE);
                jz_codec_select_mode(codec, CODEC_WMODE);
                jz_codec_dac_mute_disable(codec);

                __gpio_as_input(BIT_CLK);
                __gpio_as_input(LR_CLK);
                __gpio_as_input(I2S_OUT);
                __gpio_enable_pull(BIT_CLK);
                __gpio_enable_pull(LR_CLK);
                __gpio_enable_pull(I2S_OUT);
        } else {
                if (ak5358->replay_last) {
                        jz_codec_set_speed(codec, ak5358->fs_last, CODEC_WMODE);
                        jz_codec_set_channels(codec, ak5358->channel_last, CODEC_RMODE);
                        jz_codec_set_format(codec, ak5358->format_last, CODEC_WMODE);
                        ak5358->replay_last = 0;
                        ak5358->fs_last = 0;
                        ak5358->channel_last = 0;
                        ak5358->format_last = 0;
                        __i2s_start_bitclk();
                        __aic_enable_replay();
                } else
                        jz_codec_close(codec, CODEC_WMODE);

                __gpio_as_func1(BIT_CLK);
                __gpio_as_func0(LR_CLK);
                __gpio_as_func0(I2S_OUT);
                up(&ak5358->is_playing);
        }

        return 0;
}

static struct notifier_block linein_dete_notifier_block = {
        .notifier_call = linein_dete_notifier,
};
#endif

#ifdef CONFIG_I2S_DLV_NPCA110P
static int linein_dete_notifier(struct notifier_block *nb,
                                unsigned long val, void *data)
{
        struct jz_i2s_controller_info *controller = the_i2s_controller;
	struct i2s_codec *codec = (struct i2s_codec *)controller->i2s_codec;
        struct snd_device_config dev;

        if (val) {
                down(&linein_is_playing);
                do_jz_mute(1, 0);
                replay_last = REG_AIC_CR & AIC_CR_ERPL;

                if (replay_last) {
                        __aic_disable_replay();
                }
                dev.device = SND_DEVICE_LINEIN;
                codec_ioctrl(codec, CODEC_SET_DEVICE, (unsigned long)&dev);
                codec_ioctrl(codec, CODEC_SET_REPLAY_VOLUME, codec->audio_volume);
                do_jz_mute(0, 0);
        } else {
                do_jz_mute(1, 0);
                dev.device = SND_DEVICE_SPEAKER;
                codec_ioctrl(codec, CODEC_SET_DEVICE, (unsigned long)&dev);
                if (replay_last) {
                        replay_last = 0;
                        __i2s_enable_replay();
                        codec_ioctrl(codec, CODEC_SET_REPLAY_VOLUME, codec->audio_volume);
                        do_jz_mute(0, 0);
                }

                up(&linein_is_playing);
        }

        return 0;
}

static struct notifier_block linein_dete_notifier_block = {
        .notifier_call = linein_dete_notifier,
};
#endif

static int __init init_jz_i2s(struct platform_device *pdev)
{
	struct i2s_codec *codec;
	int errno;
	int fragsize;
	int fragstotal;
	unsigned char i;

#ifdef CONFIG_AK5358
        errno = ak5358_register_state_notifier(&linein_dete_notifier_block);
        if (errno)
                dev_warn(&pdev->dev,"ak5358 notifier_block regist error\n");
#endif
#ifdef CONFIG_I2S_DLV_NPCA110P
        errno = npca110p_register_state_notifier(&linein_dete_notifier_block);
        if (errno)
                dev_warn(&pdev->dev,"npca110p notifier_block regist error\n");
#endif

	/* build global i2s controller struct */
	if ((errno = probe_jz_i2s(&the_i2s_controller)) < 0) {
		return errno;
	}

	cpm_start_clock(CGM_AIC);
	/* start I2S clks output to codec */
	REG_AIC_I2SCR |= AIC_I2SCR_ESCLK;

	for(i = 0;i < NR_I2S; i++){
		codec = &the_codecs[i];
		if (codec && codec->codecs_ioctrl) {
			if(codec->name == JZ_INTERNAL_CODEC){
				i2s_controller_init(JZ_INTERNAL_CODEC);
				inter_codec = codec;
			}
			else if(codec->name == JZ_I2S_EXTERNAL_CODEC){
				i2s_controller_init(JZ_I2S_EXTERNAL_CODEC);
				exter_codec = codec;
			}
			else if(codec->name == JZ_SPDIF_EXTERNAL_CODEC){
				i2s_controller_init(JZ_SPDIF_EXTERNAL_CODEC);
				exter_codec = codec;
			}
			/* codec init */
			codec->codecs_ioctrl(codec, CODEC_INIT, 0);
			the_i2s_controller->i2s_codec = codec;
		}
	}

	/* build HP station sem */
	sema_init(&(hp_sem),1);

	/* May be external CODEC need it ...
	 * default_codec->codecs_ioctrl(default_codec, CODEC_SET_GPIO_PIN, 0);
	 */
	attach_jz_i2s(the_i2s_controller);

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

	workqueue = create_singlethread_workqueue("kjzi2s");
	if (!workqueue){
		printk("JZ I2S: create_singlethread_workqueue fail!\n");
		return -ENOMEM;
	}
	the_i2s_controller->workqueue = workqueue;
	INIT_DELAYED_WORK(&the_i2s_controller->mute_work, i2s_mute_work_handler);
	flush_workqueue(the_i2s_controller->workqueue);
	the_i2s_controller->mute.bsp_mute = -1;
	the_i2s_controller->mute.codec_mute = -1;

#ifdef CONFIG_HP_INSERT_GPIO_DETECT
	/* request HP insert GPIO irq and work_queue */
	INIT_WORK(&hp_gpio_irq_work, hp_insert_irq_work_handler);
	hp_work_queue = create_singlethread_workqueue("hp_insert_irq_wq");
	if (!hp_work_queue) {
		printk("hp_work_queue creat error\n");
		BUG();
	}

	errno = request_irq((IRQ_GPIO_0 + GPIO_HP_SENSE), hp_insert_gpio_irq, IRQF_DISABLED, "hp_insert_gpio_irq", NULL);
	if (errno) {
		printk("JZ DLV: Could not request HP insert gpio irq %d\n",(IRQ_GPIO_0 + GPIO_HP_SENSE));
#if defined(CONFIG_I2S_DLV_NPCA110P) || defined(CONFIG_AK5358)
                goto finish;
#endif
		return errno;
	}
	/* enable hp GPIO irq */
	enable_hp_sense_irq();
#if defined(CONFIG_I2S_DLV_NPCA110P) || defined(CONFIG_AK5358)
finish:
#endif
#endif
	printk("JZ I2S OSS audio initialized\n");
	LEAVE();
	return 0;
}

static void __exit cleanup_jz_i2s(void)
{
#ifdef CONFIG_PM
	/* pm_unregister(i2s_controller->pm); */
#endif
#ifdef CONFIG_HP_INSERT_GPIO_DETECT
	free_irq((IRQ_GPIO_0 + GPIO_HP_SENSE), NULL);
#endif
	unload_jz_i2s(the_i2s_controller);
	the_i2s_controller = NULL;
	audio_deinit_endpoint(&out_endpoint);
	audio_deinit_endpoint(&in_endpoint);
	destroy_workqueue(workqueue);
}

__refdata static struct platform_driver snd_plat_driver = {
	.probe		=  init_jz_i2s,
	.driver		= {
		.name	= "mixer",
		.owner	= THIS_MODULE,
	},
	.suspend	= jz_i2s_suspend,
	.resume		= jz_i2s_resume,
	.shutdown   = jz_i2s_shutdown,
};

static int __init snd_init(void)
{

#ifdef CONFIG_ANDROID
	register_early_suspend(&jz_i2c_early_suspend);
#endif

	return platform_driver_register(&snd_plat_driver);
}

module_init(snd_init);
module_exit(cleanup_jz_i2s);

/*--------------------------------------------------------------------------*/
/* BT call thread */

#if 0
static void bt_call_restore_setting(void);
extern void bt_call_pcm_init(void);
extern void bt_call_pcm_start_dma(unsigned char *buf, int size, int path);
extern void bt_call_pcm_stop(void);

struct timer_list restart_down_pcm_dma_timer;
struct timer_list restart_up_pcm_dma_timer;

audio_node *cur_in_node;
audio_node *cur_out_node;

static void bt_call_restore_setting(void)
{
	struct jz_i2s_controller_info *controller = the_i2s_controller;
	struct i2s_codec *codec = controller->i2s_codec;
	audio_pipe	*pin_endpoint = controller->pin_endpoint;
	audio_pipe	*pout_endpoint = controller->pout_endpoint;
	audio_node * node;

	jz_codec_set_channels(codec, 2, CODEC_RMODE);
	jz_codec_set_format(codec, 16, CODEC_RMODE);
	jz_codec_set_speed(codec, 44100, CODEC_RMODE);
	codec->user_need_mono = 0;
	set_controller_triger(controller, &in_endpoint, codec->record_codec_channel, codec->record_format);

	jz_codec_set_channels(codec, 2, CODEC_WMODE);
	jz_codec_set_format(codec, 16, CODEC_WMODE);
	jz_codec_set_speed(codec, 44100, CODEC_WMODE);
	set_controller_triger(controller, &out_endpoint, codec->replay_codec_channel, codec->replay_format);

	controller->pin_endpoint = gbt_call_pcm.save_in_endpoint;
	controller->pout_endpoint = gbt_call_pcm.save_out_endpoint;

	/* release nodes if we holded */
	if (cur_in_node) {
		put_audio_freenode(in_endpoint.mem, cur_in_node);
		cur_in_node = NULL;
	}

	if (cur_out_node) {
		cur_out_node->start = 0;
		cur_out_node->end = out_endpoint.fragsize;

		put_audio_usenode(out_endpoint.mem, cur_out_node);
		cur_out_node = NULL;
	}

	/* free all in node */
	while (!is_null_use_audio_node(pin_endpoint->mem)) {
		node = get_audio_usenode(pin_endpoint->mem);
		if (node) {
			put_audio_freenode(pin_endpoint->mem, node);
		}
	}

	/* free all out node */
	while (!is_null_use_audio_node(pout_endpoint->mem)) {
		node = get_audio_usenode(pout_endpoint->mem);
		if (node) {
			put_audio_freenode(pout_endpoint->mem, node);
		}
	}
}

void bt_call_down_stream_handle_buffer(void)
{
	struct jz_i2s_controller_info *controller = the_i2s_controller;
	unsigned long flags;

	if (cur_in_node) {
		/* if DMA have not started ,so start it */
		if (!(in_endpoint.trans_state & PIPE_TRANS)) {
			if (trystart_endpoint_in(controller, cur_in_node) == 0) {
				printk("JZ I2S: trystart_endpoint_in error\n");
			}
		} else {
			AUDIO_LOCK(in_endpoint.lock, flags);
			put_audio_freenode(in_endpoint.mem, cur_in_node);
			AUDIO_UNLOCK(in_endpoint.lock, flags);
		}

		cur_in_node = NULL;
	}

	/* exit flag */
	if (bt_call_exit_flag) return;

	AUDIO_LOCK(in_endpoint.lock, flags);
	cur_in_node = get_audio_usenode(in_endpoint.mem);
	AUDIO_UNLOCK(in_endpoint.lock, flags);

	if (cur_in_node) {
		bt_call_pcm_start_dma((unsigned char *)cur_in_node->pBuf,
				     cur_in_node->end - cur_in_node->start, 0);
		//printk("jz_bt_call_start_dma downstream size=%d \n",
		//	cur_in_node->end - cur_in_node->start);
	}
	else {
		printk("bt_call_down_stream can not get AIC buffer!\n");

		//close pcm replay
		__pcm_disable_transmit_dma(CUR_PCM);
		__pcm_disable_txfifo(CUR_PCM);

		mod_timer(&restart_down_pcm_dma_timer, jiffies + 5);
	}
}

void bt_call_up_stream_handle_buffer(void)
{
	struct jz_i2s_controller_info *controller = the_i2s_controller;
	unsigned long flags;

	if (cur_out_node) {
		cur_out_node->start = 0;
		cur_out_node->end = out_endpoint.fragsize;

		if (!(out_endpoint.trans_state & PIPE_TRANS)) {
			if (trystart_endpoint_out(controller, cur_out_node) == 0) {
				printk("JZ I2S: trystart_endpoint_out error\n");
			}
		} else {
			AUDIO_LOCK(out_endpoint.lock, flags);
			put_audio_usenode(out_endpoint.mem, cur_out_node);
			AUDIO_UNLOCK(out_endpoint.lock, flags);
		}

		cur_out_node = NULL;
	}

	/* exit flag */
	if (bt_call_exit_flag) return;

	AUDIO_LOCK(out_endpoint.lock, flags);
	cur_out_node = get_audio_freenode(out_endpoint.mem);
	AUDIO_UNLOCK(out_endpoint.lock, flags);

	if (cur_out_node) {
		bt_call_pcm_start_dma((unsigned char *)cur_out_node->pBuf,
				     out_endpoint.fragsize, 1);

		//printk("jz_bt_call_start_dma upstream size=%d %x\n",
		       //out_endpoint.fragsize, time3);
	}
	else {
		printk("bt_call_up_stream can not get AIC buffer!\n");

		//close pcm record
		__pcm_disable_receive_dma(CUR_PCM);
		__pcm_disable_rxfifo(CUR_PCM);

		mod_timer(&restart_up_pcm_dma_timer, jiffies + 5);
	}
}

int bt_call_down_stream_thread(void *v)
{
	struct jz_i2s_controller_info *controller = the_i2s_controller;
	struct i2s_codec *codec = controller->i2s_codec;
	audio_pipe	*pin_endpoint ;
	audio_node *node;
	int buffer_delay;
	unsigned int start;
	unsigned long flags;

	/* save the old in endpoint */
	gbt_call_pcm.save_in_endpoint = controller->pin_endpoint;

	controller->pin_endpoint = &in_endpoint;
	controller->pin_endpoint->is_non_block = 0;
	pin_endpoint = controller->pin_endpoint;

	/* init i2s*/
	jz_codec_set_channels(codec, 1, CODEC_RMODE);
	jz_codec_set_format(codec, 16, CODEC_RMODE);
	jz_codec_set_speed(codec, 8000, CODEC_RMODE);
	/* we do not need filter now */

	/* init codec */
	set_controller_triger(controller, &in_endpoint, codec->record_codec_channel, codec->record_format);

	/* make sure all node in list */
	audio_close_endpoint(pin_endpoint, FORCE_STOP);

	/* flush all old record node */
	AUDIO_LOCK(pin_endpoint->lock, flags);
	while (!is_null_use_audio_node(pin_endpoint->mem)) {
		node = get_audio_usenode(pin_endpoint->mem);
		if (node) {
			put_audio_freenode(pin_endpoint->mem, node);
		}
	}
	AUDIO_UNLOCK(pin_endpoint->lock, flags);
	printk("free count in in endpoint =%d \n", get_audio_freenodecount(pin_endpoint->mem));

	AUDIO_LOCK(pin_endpoint->lock, flags);
	node = get_audio_freenode(pin_endpoint->mem);
	AUDIO_UNLOCK(pin_endpoint->lock, flags);

	/* calculate the AIC buffer delay, we delay half buffer size */
	buffer_delay = pin_endpoint->fragsize / 8 / 2 / 2 / 2;
	printk("bt call down stream going to delay %d ms\n",buffer_delay);

	node->end = pin_endpoint->fragsize;

	/* start record DMA */
	start = trystart_endpoint_in(controller, node);
	if (start == 0) {
		DPRINT("@@@@ Error ! BT call PCM jz_audio_read, start == 0\n");
		AUDIO_LOCK(pin_endpoint->lock, flags);
		put_audio_freenode(pin_endpoint->mem, node);
		AUDIO_UNLOCK(pin_endpoint->lock, flags);
	}

	pin_endpoint->avialable_couter = 0;
	/* wait till one node available */
	wait_event_interruptible(pin_endpoint->q_full, pin_endpoint->avialable_couter >= 1);

	pin_endpoint->avialable_couter = 0;
	cur_in_node = NULL;

	msleep(buffer_delay);

	/* start PCM DMA */
	bt_call_down_stream_handle_buffer();

	/* now the buffer run has began, this thread can exit! */
	do_exit(0);

	return 0;
}

int bt_call_up_stream_thread(void *v)
{
	struct jz_i2s_controller_info *controller = the_i2s_controller;
	struct i2s_codec *codec = controller->i2s_codec;
	audio_pipe	*pout_endpoint ;
	audio_node *node;
	unsigned long flags;

	/* save the old out endpoint */
	gbt_call_pcm.save_out_endpoint = controller->pout_endpoint;

	controller->pout_endpoint = &out_endpoint;
	controller->pout_endpoint->is_non_block = 0;
	pout_endpoint = &out_endpoint;

	jz_codec_set_channels(codec, 1, CODEC_WMODE);
	jz_codec_set_format(codec, 16, CODEC_WMODE);
	jz_codec_set_speed(codec, 8000, CODEC_WMODE);
	set_controller_triger(controller, &out_endpoint, codec->replay_codec_channel, codec->replay_format);

	/* make sure all node in list */
	audio_close_endpoint(pout_endpoint, FORCE_STOP);

	/* flush all old replay node */
	AUDIO_LOCK(pout_endpoint->lock, flags);
	while (!is_null_use_audio_node(pout_endpoint->mem)) {
		node = get_audio_usenode(pout_endpoint->mem);
		if (node) {
			put_audio_freenode(pout_endpoint->mem, node);
		}
	}
	AUDIO_UNLOCK(pout_endpoint->lock, flags);
	printk("free count in out endpoint =%d \n", get_audio_freenodecount(pout_endpoint->mem));

	/* get two free node ,and memset to mute */
	/* play the two node to make buffer delay */
	AUDIO_LOCK(pout_endpoint->lock, flags);
	node = get_audio_freenode(out_endpoint.mem);
	AUDIO_UNLOCK(pout_endpoint->lock, flags);

	node->start = 0;
	node->end = out_endpoint.fragsize;
	memset((void *)node->pBuf, 0x00, out_endpoint.fragsize);

	printk("start AIC replay DMA \n");
	if (trystart_endpoint_out(controller, node) == 0) {
		printk("JZ I2S: trystart_endpoint_out error\n");
	}

	/* second node just fill half data to make half buffer delay */
	AUDIO_LOCK(pout_endpoint->lock, flags);
	node = get_audio_freenode(out_endpoint.mem);
	AUDIO_UNLOCK(pout_endpoint->lock, flags);

	node->start = 0;
	node->end = out_endpoint.fragsize / 2;
	memset((void *)node->pBuf, 0x00, out_endpoint.fragsize / 2);

	AUDIO_LOCK(pout_endpoint->lock, flags);
	put_audio_usenode(out_endpoint.mem, node);
	AUDIO_UNLOCK(pout_endpoint->lock, flags);

	/* get one free node and start PCM DMA */
	cur_out_node = NULL;
	bt_call_up_stream_handle_buffer();

	/* just exit */
	do_exit(0);

	return 0;
}

static void bt_call_down_stream_handle_buffer_t(unsigned long unused)
{
	bt_call_down_stream_handle_buffer();
}

static void bt_call_up_stream_handle_buffer_t(unsigned long unused)
{
	bt_call_up_stream_handle_buffer();
}

void bt_call_enter(void)
{
	bt_call_pcm_init();

	//init timer
	setup_timer(&restart_down_pcm_dma_timer, bt_call_down_stream_handle_buffer_t, 0L);
	setup_timer(&restart_up_pcm_dma_timer, bt_call_up_stream_handle_buffer_t, 0L);

	gbt_call_pcm.kthread_out = kthread_run(bt_call_down_stream_thread, NULL, "kbtcalldownstream");
	gbt_call_pcm.kthread_in = kthread_run(bt_call_up_stream_thread, NULL, "kbtcallupstream");
}

void bt_call_exit(void)
{
	bt_call_pcm_stop();

	//delete timer
	del_timer(&restart_down_pcm_dma_timer);
	del_timer(&restart_up_pcm_dma_timer);

	bt_call_restore_setting();
}
#endif
