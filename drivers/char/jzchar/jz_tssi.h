#ifndef __JZ_TSSI_H__
#define __JZ_TSSI_H__


/* 
 * IOCTL commands
 */
#define IOCTL_TSSI_ENABLE               0x01
#define IOCTL_TSSI_DISABLE              0x02
#define IOCTL_TSSI_SOFTRESET            0x03
#define IOCTL_TSSI_ENFILTER             0x04
#define IOCTL_TSSI_DEFILTER             0x05
#define IOCTL_TSSI_ADDPID               0x06
#define IOCTL_TSSI_FLUSHPID             0x07
#define IOCTL_TSSI_INIT_DMA             0x08
#define IOCTL_TSSI_DISABLE_DMA          0x09

#if 0
#define IOCTL_TSSI_SET_CFG              0x06
#define IOCTL_TSSI_GET_CFG              0x07
#define IOCTL_TSSI_ENIRQ_TRIG           0x08
#define IOCTL_TSSI_DEIRQ_TRIG           0x09
#define IOCTL_TSSI_ENIRQ_OVRN           0x0a
#define IOCTL_TSSI_DEIRQ_OVRN           0x0b 
#define IOCTL_TSSI_ENPID0               0x0c
#define IOCTL_TSSI_DEPID0               0x0d 
#define IOCTL_TSSI_ENPIDN               0x0e
#define IOCTL_TSSI_DEPIDN               0x0f
#define IOCTL_TSSI_SETPIDN              0x10
#define IOCTL_TSSI_SET_TRIG             0x11
#endif

#define MAX_PID_NUM                     15
#define MPEG2_TS_PACHAGE_SIZE           19200

struct jz_tssi_cfg_t
{
	unsigned char wordorder;
	unsigned char byteorder;
	unsigned char dataploa;
	unsigned char use0;
	unsigned char clkch;
	unsigned char mode;
	unsigned char clkpola;
	unsigned char frmpola;
	unsigned char strpola;
	unsigned char failpola;
	unsigned char trignum;

	unsigned short pid;
	unsigned char pid_index;          //0 to 15
};

struct jz_tssi_buf
{
	unsigned int *buf;
	unsigned int   pos;
	unsigned int   index;
	struct jz_tssi_buf *next;
};

struct jz_tssi_buf_ring_t
{
	struct jz_tssi_buf	*front;
	struct jz_tssi_buf	*rear;
	unsigned int  fu_num;
};

struct jz_tssi_t
{
	struct jz_tssi_cfg_t cur_config;
	struct jz_tssi_buf_ring_t *cur_buf;
	struct mutex lock;
	wait_queue_head_t wait;
	int dma_chan, pid_num;
};

#endif /* __JZ_TSSI_H__ */
