#ifndef __JZ_SND_H__
#define __JZ_SND_H__
#include "codecs/dlv_common.h"

#define MODE_REPLAY CODEC_MODE_REPLAY
#define MODE_RECORD CODEC_MODE_RECORD
#define MODE_REPLAY_RECORD CODEC_MODE_REPLAY_RECORD

#define DEFAULT_CHANNELS 2
#define DEFAULT_RATE     44100
#define DEFAULT_FORMAT    AFMT_S16_LE
#define INIT_DEFAULT_REPLAY_VOLUME 80
#define INIT_DEFAULT_RECORD_VOLUME 80

extern int register_codec(struct codec_ops *cops);
extern int unregister_codec(struct codec_ops *cops);

struct jz_snd_codec_info
{
	int	replay_volume;
	int	record_volume;

	unsigned short	record_rate;
	unsigned short	replay_rate;

	short	replay_channels;
	short	record_channels;

	short	replay_format;
	short	record_format;

	struct codec_ops *codec_ops;

    struct semaphore sem;
};

struct jz_snd_controller
{
    int is_snd_ready;

    short is_replaying;
    short is_recording;

    short is_block_replay;
    short is_block_record;

    short is_first_replay;
    short is_first_record;

    struct semaphore sem;

    //FIXME: fragsize(record and replay)
     int fragsize_replay;
     int fragsize_record;

};

#endif  /* __JZ_SND_H__ */
