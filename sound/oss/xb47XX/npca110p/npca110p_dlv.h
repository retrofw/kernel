/*
 * Linux/sound/oss/npca110p_dlv.h
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#ifndef __NPCA110P_DLV_H__
#define __NPCA110P_DLV_H__

#include <linux/jz_audio.h>

// DEBUG switch
#define DLV_DUMP_IOC_CMD        0

/* Power setting */
#define POWER_ON		0
#define POWER_OFF		1

/* File ops mode W & R */
#define REPLAY			1
#define RECORD			2

#define GPIO_NPCA110P_RESET     (32 * 1 + 16)
#define GPIO_NPCA110P_POWER     (32 * 1 + 8)
#define GPIO_SPEAKER_POWER      (32 * 4 + 1)

#define GPIO_LINEIN_DETECT      (32 * 5 + 0)
#define GPIO_LINEIN_IRQ         (GPIO_LINEIN_DETECT + 64)
#define GPIO_LINEIN_ACTIVE      0

#if DLV_DUMP_IOC_CMD
static void dlv_print_ioc_cmd(int cmd)
{
	char *dlv_ioc_cmd[] = {
		"CODEC_INIT", "CODEC_TRUN_OFF",
                "CODEC_SET_SHUTDOWN", "CODEC_RESET",
                "CODEC_SUSPEND", "CODEC_RESUME",
                "CODEC_ANTI_POP", "CODEC_SET_ROUTE",
                "CODEC_SET_DEVICE", "CODEC_SET_STANDBY",
                "CODEC_SET_RECORD_RATE", "CODEC_SET_RECORD_DATA_WIDTH",
                "CODEC_SET_MIC_VOLUME", "CODEC_SET_RECORD_CHANNEL",
                "CODEC_SET_REPLAY_RATE", "CODEC_SET_REPLAY_DATA_WIDTH",
                "CODEC_SET_REPLAY_VOLUME", "CODEC_SET_REPLAY_CHANNEL",
                "CODEC_SET_ADC_LRSWAP", "CODEC_CLEAR_ADC_LRSWAP",
                "CODEC_SET_DAC_LRSWAP", "CODEC_CLEAR_DAC_LRSWAP",
                "CODEC_DAC_MUTE", "CODEC_BSP_MUTE",
                "CODEC_DUMP_REGS", "CODEC_DEBUG_ROUTINE",
	};

	if (cmd >= (sizeof(dlv_ioc_cmd) / sizeof(dlv_ioc_cmd[0]))) {
		printk("%s: Unkown command !\n", __FUNCTION__);
	} else {
		printk("IOC CMD NAME = %s\n", dlv_ioc_cmd[cmd]);
	}
}

#define DUMP_IOC_CMD()								\
	do {									\
		printk("[dlv IOCTL]++++++++++++++++++++++++++++\n");		\
		printk("%s  cmd = %d, arg = %lu\n", __func__, cmd, arg); 	\
		dlv_print_ioc_cmd(cmd);						\
		printk("[dlv IOCTL]----------------------------\n");		\
										\
	} while (0)
#else
#define DUMP_IOC_CMD()
#endif

#endif

