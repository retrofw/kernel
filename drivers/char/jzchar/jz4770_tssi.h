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
#define IOCTL_TSSI_GET_OFFSET		0x0a

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
//#define MPEG2_PACKET_SIZE          (1024 * 960) /* bytes */
#define MPEG2_PACKET_SIZE          (1024 * 188 * 10) /* bytes */

#define GET_BUF         2
#define SWAP_BUF        3
#define NR_BUF          (GET_BUF + SWAP_BUF)

struct jz_tssi_desc_t
{
	unsigned int next_desc;
	unsigned int dst_addr;
	unsigned int did;
	unsigned int cmd;
};


struct jz_tssi_t
{
	struct jz_tssi_desc_t tssi_desc[NR_BUF];
	spinlock_t lock;
	wait_queue_head_t wait;
	unsigned int offset;
	int did;
	int num;
	int pid_num;
	unsigned char *mem_base;
	unsigned int mem_size;
};

#endif /* __JZ_TSSI_H__ */
