/*
 * Linux/sound/oss/jz4760_dlv.h
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 * Author :   yliu@ingenic.cn
 */

#ifndef __JZ4760_BT_CALL_H__
#define __JZ4760_BT_CALL_H__

/* the global infomation about BT call */
/* to keep some crytical data */
struct bt_call_pcm_info {
	int dma_in_ch;
	int dma_out_ch;
	int in_thread_up;
	int out_thread_up;
	wait_queue_head_t  out_wait_queue;
	wait_queue_head_t  in_wait_queue;
	struct task_struct *kthread_out;
	struct task_struct *kthread_in;
	void *save_in_endpoint;
	void *save_out_endpoint;
};

/* PCM configuration infomation */
struct bt_call_pcm_cfg{
	unsigned char omsb_pos,imsb_pos;
	unsigned char iss_bit, oss_bit;
	unsigned char time_slot;
	unsigned char mode;
};

#define PCM_SSBIT_8      	(8)
#define PCM_SSBIT_16      	(16)
#define PCM_MSB_POS_SAME      	(0)
#define PCM_MSB_POS_NEXT      	(1)
#define PCM_MODE_MASTER  	(0)
#define PCM_MODE_SLAVE    	(1)

#define BT_CALL_SAMPLE_SIZE  	16
#define BT_CALL_SAMPLE_RATE  	8192
#define BT_CALL_BUFFER_RATIO 	(1/16)
#define BT_CALL_BUFFER_SIZE  	(BT_CALL_SAMPLE_SIZE * BT_CALL_SAMPLE_RATE * BT_CALL_SAMPLE_RATIO)
#define BT_CALL_BUFFER_NUM   	2

/* down stream data path: baseband->mic2->pcmout->BT->earphone  */
/* up stream data path: earmic->BT->pcmin->lineout->baseband  */
#define BT_CALL_DOWN_STREAM  	0
#define BT_CALL_UP_STREAM  	1

#ifdef CONFIG_SOC_JZ4770
#define IRQ_PCM       	IRQ_PCM0
#endif

#define CUR_PCM		PCM0 

#endif
