/* sound/oss/jz_audio.h
 *
 * Copyright (C) 2010 Ingenic, Inc.
 *
 * 2010-03-30 Copy from include/linux/msm_audio.h which is write by Google.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_JZ_AUDIO_H
#define __LINUX_JZ_AUDIO_H

#include <linux/types.h>
#include <linux/ioctl.h>
//#include <asm/sizes.h>

/* PCM Audio */

#define AUDIO_IOCTL_MAGIC 'a'

#define AUDIO_START          _IOW(AUDIO_IOCTL_MAGIC, 0, unsigned)
#define AUDIO_STOP           _IOW(AUDIO_IOCTL_MAGIC, 1, unsigned)
#define AUDIO_FLUSH          _IOW(AUDIO_IOCTL_MAGIC, 2, unsigned)
#define AUDIO_GET_CONFIG     _IOR(AUDIO_IOCTL_MAGIC, 3, unsigned)
#define AUDIO_SET_CONFIG     _IOW(AUDIO_IOCTL_MAGIC, 4, unsigned)
#define AUDIO_GET_STATS      _IOR(AUDIO_IOCTL_MAGIC, 5, unsigned)
#define AUDIO_ENABLE_AUDPP   _IOW(AUDIO_IOCTL_MAGIC, 6, unsigned)
#define AUDIO_SET_ADRC       _IOW(AUDIO_IOCTL_MAGIC, 7, unsigned)
#define AUDIO_SET_EQ         _IOW(AUDIO_IOCTL_MAGIC, 8, unsigned)
#define AUDIO_SET_RX_IIR     _IOW(AUDIO_IOCTL_MAGIC, 9, unsigned)
#define AUDIO_SET_VOLUME     _IOW(AUDIO_IOCTL_MAGIC, 10, unsigned)
#define AUDIO_ENABLE_AUDPRE  _IOW(AUDIO_IOCTL_MAGIC, 11, unsigned)
#define AUDIO_SET_AGC        _IOW(AUDIO_IOCTL_MAGIC, 12, unsigned)
#define AUDIO_SET_NS         _IOW(AUDIO_IOCTL_MAGIC, 13, unsigned)
#define AUDIO_SET_TX_IIR     _IOW(AUDIO_IOCTL_MAGIC, 14, unsigned)
#define AUDIO_SET_AAC_CONFIG _IOW(AUDIO_IOCTL_MAGIC, 15, unsigned)
#define AUDIO_WAIT_ADSP_DONE _IOR(AUDIO_IOCTL_MAGIC, 16, unsigned)
#define AUDIO_ADSP_PAUSE     _IOR(AUDIO_IOCTL_MAGIC, 17, unsigned)
#define AUDIO_ADSP_RESUME    _IOR(AUDIO_IOCTL_MAGIC, 18, unsigned)
#define AUDIOIN_SET_CONFIG   _IOR(AUDIO_IOCTL_MAGIC, 19, unsigned)

struct jz_audio_config {
	uint32_t buffer_size;
	uint32_t buffer_count;
	uint32_t channel_count;
	uint32_t sample_rate;
	uint32_t type;
        uint32_t codec_format;
	uint32_t unused[3];
};

struct jz_audio_stats {
	uint32_t byte_count;
	uint32_t sample_count;
	uint32_t unused[2];
};

struct jz_audio_aac_config {
	signed short format;
	unsigned short audio_object;
	unsigned short ep_config;
	unsigned short aac_section_data_resilience_flag;
	unsigned short aac_scalefactor_data_resilience_flag;
	unsigned short aac_spectral_data_resilience_flag;
	unsigned short sbr_on_flag;
	unsigned short sbr_ps_on_flag;
	unsigned short dual_mono_mode;
	unsigned short channel_configuration;
};


////////////////////////////////////////////////////////////////////////

#define SND_IOCTL_MAGIC 's'

#define SND_MUTE_UNMUTED 0
#define SND_MUTE_MUTED 1

struct snd_device_config {
	uint32_t device;
	uint32_t ear_mute;
	uint32_t mic_mute;
};


#define SND_SET_DEVICE _IOW(SND_IOCTL_MAGIC, 2, struct msm_device_config *)

#define SND_METHOD_VOICE 0

struct snd_volume_config {
	uint32_t device;
	uint32_t method;
	uint32_t volume;
};

#define SND_SET_VOLUME _IOW(SND_IOCTL_MAGIC, 3, struct msm_snd_volume_config *)



struct snd_endpoint {
	int id;
	char name[64];
};

struct jz_snd_endpoints {
	struct snd_endpoint *endpoints;
	int num;
};


#if 0
struct snd_ctxt {
	struct mutex lock;
	int opened;

	struct jz_rpc_endpoint *ept;
	struct task_struct *task;
	int inited;
};
#endif
#define SND_GET_NUM_ENDPOINTS	_IOR(SND_IOCTL_MAGIC, 4, unsigned *)
#define SND_GET_ENDPOINT	_IOWR(SND_IOCTL_MAGIC, 5, struct jz_snd_endpoint *)
#define SND_SELECT_CODEC 	_IOWR(SND_IOCTL_MAGIC, 6, unsigned *)

//#define SND_GET_NUM_ENDPOINTS 101
//#define SND_GET_ENDPOINT 102

#endif
