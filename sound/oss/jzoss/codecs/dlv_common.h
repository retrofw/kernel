#ifndef __JZDLV_COMMON_H__
#define __JZDLV_COMMON_H__

#include "dlv_control.h"

#define CODEC_MODE_REPLAY		FMODE_WRITE
#define CODEC_MODE_RECORD		FMODE_READ
#define CODEC_MODE_REPLAY_RECORD	(CODEC_MODE_REPLAY | CODEC_MODE_RECORD)


struct codec_ops {
    int (*dump_regs)(const char *str);
	int (*set_width)(int mode, int width);
	int (*set_rate)(int mode, int rate);
	int (*set_channels)(int mode, int channels);

	int (*set_replay_volume)(int vol);
	int (*set_record_volume)(int vol);
	int (*set_gcr)(struct codec_gcr_ctrl *ctrl);

    int (*set_mode)(int mode);

	int (*triggle)(int event);
};

#define CODEC_EVENT_INIT	0x1
#define CODEC_EVENT_DEINIT	0x2
#define CODEC_EVENT_SUSPEND	0x3
#define CODEC_EVENT_RESUME	0x4
#define CODEC_EVENT_REPLAY_START	0x5
#define CODEC_EVENT_REPLAY_STOP	    0x6
#define CODEC_EVENT_RECORD_START	0x7
#define CODEC_EVENT_RECORD_STOP	    0x8


#endif /* __JZDLV_COMMON_H__ */
