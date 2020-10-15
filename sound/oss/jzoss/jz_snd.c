/*
 * Linux/sound/oss/jzoss/jz_snd.c
 *
 * Sound driver for Ingenic Jz MIPS processor
 *
 * 2011-11-xx	liulu <lliu@ingenic.cn>
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */
#include <linux/soundcard.h>
#include <linux/sound.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>

#include "codecs/dlv_control.h"
#include "codecs/dlv_common.h"
#include "jz47xx_i2s.h"
#include "jz47xx_dma.h"
#include "jz_snd.h"

static int jz_snd_debug = 0;
module_param(jz_snd_debug, int, 0644);
#define JZ_SND_DEBUG_MSG(msg...)			\
	do {					\
		if (jz_snd_debug)		\
			printk("jz_snd: " msg);	\
	} while(0)

static struct jz_snd_codec_info codec_info;
static struct jz_snd_controller controller =
{
    .is_first_replay = 1,
    .is_first_record = 1,
    .fragsize_replay = DEFAULT_FRAG_SIZE,
    .fragsize_record = DEFAULT_FRAG_SIZE,
};
static int g_mixer_modcnt = 0;

int register_codec(struct codec_ops *cops)
{
	JZ_SND_DEBUG_MSG("enter %s\n", __func__);
    if(cops == NULL){
        return -1;
    }

    codec_info.codec_ops = cops;
    return 0;
}

int unregister_codec(struct codec_ops *cops)
{
	JZ_SND_DEBUG_MSG("enter %s\n", __func__);
    if(cops == NULL || cops != codec_info.codec_ops){
        return -1;
    }

    codec_info.codec_ops = NULL;
    return 0;
}

static int jz_snd_set_format(int mode, int format)
{
    int data_width;

	JZ_SND_DEBUG_MSG("enter %s, mode = %d, format = %d\n", __func__, mode, format);

    switch (format) {
    case AFMT_U8:
    case AFMT_S8:
        data_width = 8;
        break;
    case AFMT_S16_LE:
    case AFMT_S16_BE:
        data_width = 16;
        break;
    case AFMT_S24_LE:
    case AFMT_S24_BE:
    default:
        data_width = 24;
        break;
    }

    codec_info.codec_ops->set_width(mode, data_width);
    jz47xx_i2s_set_width(mode, data_width);
    jz_audio_dma_set_width(mode, data_width);

    if(mode & MODE_RECORD){
        codec_info.record_format = format;
    }
    if(mode & MODE_REPLAY){
        codec_info.replay_format = format;
    }

	JZ_SND_DEBUG_MSG("enter %s, mode = %d, data_width = %d\n", __func__, mode, data_width);
    return format;
}

static int jz_snd_set_rate(int mode, int rate)
{
    int s_rate;

	JZ_SND_DEBUG_MSG("enter %s, mode = %d, rate = %d\n", __func__, mode, rate);

    s_rate = codec_info.codec_ops->set_rate(mode, rate);
    jz47xx_i2s_set_rate(mode, s_rate);

    if(mode & MODE_RECORD){
        codec_info.record_rate = s_rate;
    }
    if(mode & MODE_REPLAY){
        codec_info.replay_rate = s_rate;
    }

    return s_rate;
}

static int jz_snd_set_channels(int mode, int channels)
{
	JZ_SND_DEBUG_MSG("enter %s, mode = %d, channels = %d\n", __func__, mode, channels);

    codec_info.codec_ops->set_channels(mode, channels);
    jz47xx_i2s_set_channels(mode, channels);
    jz_audio_dma_set_channels(mode, channels);

    if(mode & MODE_RECORD){
        codec_info.record_channels = channels;
    }
    if(mode & MODE_REPLAY){
        codec_info.replay_channels = channels;
    }

    return channels;
}

static int jz_snd_dsp_open(struct inode *inode, struct file *file)
{
	int mode = 0, ret = 0;

	JZ_SND_DEBUG_MSG("enter %s\n", __func__);

    if (down_interruptible(&controller.sem)){
        return -ERESTARTSYS;
    }
    if (!controller.is_snd_ready){
		printk("\nAudio device not ready!\n");
        ret = -ENODEV;
        goto dsp_open_out;
    }
	if ((file->f_mode & FMODE_READ) && controller.is_recording) {
		printk("\nAudio read device is busy!\n");
		ret = -EBUSY;
        goto dsp_open_out;
	}
	if ((file->f_mode & FMODE_WRITE) && controller.is_replaying) {
		printk("\nAudio write device is busy!\n");
		ret = -EBUSY;
        goto dsp_open_out;
	}
    up(&controller.sem);

	if (file->f_mode & FMODE_WRITE) {
		controller.is_replaying = 1;
		controller.is_block_replay = !(file->f_flags & O_NONBLOCK);
        controller.is_first_replay = 1;
		mode |= MODE_REPLAY;
	}
	if (file->f_mode & FMODE_READ) {
		controller.is_recording = 1;
		controller.is_block_record = !(file->f_flags & O_NONBLOCK);
        controller.is_first_record = 1;
		mode |= MODE_RECORD;
	}

    jz_snd_set_channels(mode, DEFAULT_CHANNELS);
    jz_snd_set_rate(mode, DEFAULT_RATE);
    jz_snd_set_format(mode, DEFAULT_FORMAT);

    return ret;

 dsp_open_out:
    up(&controller.sem);
    return ret;
}

static int jz_snd_dsp_release(struct inode *inode, struct file *file)
{
	JZ_SND_DEBUG_MSG("enter %s\n", __func__);

    if(down_interruptible(&controller.sem)){
        return -ERESTARTSYS;
    }
    if(!controller.is_snd_ready){
		printk("\nAudio device not ready!\n");
		return -ENODEV;
    }

    if((file->f_mode & FMODE_READ) && controller.is_recording){
        jz_audio_dma_sync(MODE_RECORD);
        jz_audio_dma_stop(MODE_RECORD);
        jz_audio_dma_flush(MODE_RECORD);
        codec_info.codec_ops->triggle(CODEC_EVENT_RECORD_STOP);
        jz47xx_i2s_rx_ctrl(0);
        controller.is_first_record = 1;
        controller.is_recording = 0;
    }
    if((file->f_mode & FMODE_WRITE) && controller.is_replaying){
        jz_audio_dma_sync(MODE_REPLAY);
        jz_audio_dma_stop(MODE_REPLAY);
        jz_audio_dma_flush(MODE_RECORD);
        codec_info.codec_ops->triggle(CODEC_EVENT_REPLAY_STOP);
        jz47xx_i2s_tx_ctrl(0);
        controller.is_first_replay = 1;
        controller.is_replaying = 0;
    }
    up(&controller.sem);
    return 0;
}

static ssize_t jz_snd_dsp_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
    int s_count;

	JZ_SND_DEBUG_MSG("enter %s\n", __func__);

    if(down_interruptible(&controller.sem)){
        return -ERESTARTSYS;
    }
    if(controller.is_first_record){
        jz47xx_i2s_rx_ctrl(1);
        codec_info.codec_ops->triggle(CODEC_EVENT_RECORD_START);
        controller.is_first_record = 0;
    }
    up(&controller.sem);

    s_count = jz_audio_dma_pull(buffer, count, controller.is_block_record);

    JZ_SND_DEBUG_MSG("%s: s_count = %d\n", __FUNCTION__, s_count);
    return s_count;
}

static ssize_t jz_snd_dsp_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int s_count;

	JZ_SND_DEBUG_MSG("enter %s\n", __func__);

    if(down_interruptible(&controller.sem)){
        return -ERESTARTSYS;
    }
    if(controller.is_first_replay){
        jz47xx_i2s_tx_ctrl(1);
        codec_info.codec_ops->triggle(CODEC_EVENT_REPLAY_START);
        controller.is_first_replay = 0;
    }
    up(&controller.sem);

    s_count = jz_audio_dma_push((const u8 *)buffer, count, controller.is_block_replay);

    return s_count;
}

static int jz_snd_dsp_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	long	ret = -EINVAL;
	int	val = 0;
	int	mode = 0;
	int 	format;

	JZ_SND_DEBUG_MSG("enter %s\n", __func__);

	if (file->f_mode & FMODE_READ) {
		mode |= MODE_RECORD;
	}
	if (file->f_mode & FMODE_WRITE) {
		mode |= MODE_REPLAY;
	}

	switch (cmd) {

	case OSS_GETVERSION:
		ret = put_user(SOUND_VERSION, (int *)arg);
		break;
	case SNDCTL_DSP_RESET:
		break;

	case SNDCTL_DSP_SYNC:
        jz_audio_dma_sync(mode);
        ret = 0;
		break;

	case SNDCTL_DSP_SPEED:
		/* set smaple rate */
		if (get_user(val, (int *)arg)) {
			ret = -EFAULT;
		}
		val = jz_snd_set_rate(mode, val);
		ret = put_user(val, (int *)arg);
		break;

	case SNDCTL_DSP_STEREO:
		/* set stereo or mono channel */
		if (get_user(val, (int *)arg)) {
		    ret = -EFAULT;
		}
		jz_snd_set_channels(mode, val ? 2 : 1);
		ret = 1;
		break;

	case SNDCTL_DSP_GETBLKSIZE:
        {
            int fragsize = 0;
            if (mode & MODE_RECORD) {
                fragsize = controller.fragsize_record;
            }
            if (mode & MODE_REPLAY) {
                fragsize = controller.fragsize_replay;
            }
            ret = put_user(fragsize, (int *)arg);
            break;
        }

	case SNDCTL_DSP_GETFMTS:
		/* Returns a mask of supported sample format*/
		ret = put_user(AFMT_U8 | AFMT_S16_LE | AFMT_S24_LE, (int *)arg);
		break;

	case SNDCTL_DSP_SETFMT:
		/* Select sample format */
		if (get_user(val, (int *)arg)) {
			ret = -EFAULT;
		}

		if (val == AFMT_QUERY) {
			if (mode & MODE_RECORD) {
				val = codec_info.record_format;
			} else {
				val = codec_info.replay_format;
			}
		} else {
            val = jz_snd_set_format(mode, val);
		}

		ret = put_user(val, (int *)arg);
		break;

	case SNDCTL_DSP_CHANNELS:
		if (get_user(val, (int *)arg)) {
			ret = -EFAULT;
		}
        val = jz_snd_set_channels(mode, val);
		ret = put_user(val, (int *)arg);
		break;

	case SNDCTL_DSP_POST:
		break;

	case SNDCTL_DSP_SUBDIVIDE:
		break;

	case SNDCTL_DSP_SETFRAGMENT:
		ret = get_user(val, (long *) arg);
		if (ret != -EINVAL) {
			int newfragsize, newfragstotal;
			newfragsize = 1 << (val & 0xFFFF);
			if (newfragsize < PAGE_SIZE) {
				newfragsize = PAGE_SIZE;
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

            if (mode & MODE_RECORD) {
                controller.fragsize_record = newfragsize;
            }
            if (mode & MODE_REPLAY) {
                controller.fragsize_replay = newfragsize;
            }
            ret = jz_audio_dma_resize_buffer(mode, newfragstotal, newfragsize);
		}
		break;

	case SNDCTL_DSP_GETCAPS:
		ret = put_user(DSP_CAP_REALTIME | DSP_CAP_BATCH, (int *)arg);
		break;

	case SNDCTL_DSP_NONBLOCK:
		file->f_flags |= O_NONBLOCK;
        if(mode & MODE_REPLAY){
            controller.is_block_replay = !(file->f_flags & O_NONBLOCK);
        }
        if(mode & MODE_RECORD){
            controller.is_block_record = !(file->f_flags & O_NONBLOCK);
        }
		ret = 0;
		break;

	case SNDCTL_DSP_SETDUPLEX:
		ret = -EINVAL;
		break;

	case SNDCTL_DSP_GETOSPACE:
        {
            audio_buf_info abinfo;
            if (!(mode & MODE_REPLAY)) {
                return -EINVAL;
            }
            jz_audio_dma_get_ospace(&abinfo);
            ret = copy_to_user((void *)arg, &abinfo, sizeof(abinfo)) ? -EFAULT : 0;
            break;
        }

	case SNDCTL_DSP_GETISPACE:
        {
            audio_buf_info abinfo;
            if (!(mode & MODE_RECORD)) {
                return -EINVAL;
            }
            jz_audio_dma_get_ispace(&abinfo);
            ret = copy_to_user((void *)arg, &abinfo, sizeof(abinfo)) ? -EFAULT : 0;
            break;
        }

	case SNDCTL_DSP_GETTRIGGER:
		val = 0;
		if ((mode & MODE_RECORD) && controller.is_recording) {
			val |= PCM_ENABLE_INPUT;
		}
		if ((mode & MODE_REPLAY) && controller.is_replaying) {
			val |= PCM_ENABLE_OUTPUT;
		}
		ret = put_user(val, (int *)arg);

		break;

	case SNDCTL_DSP_SETTRIGGER:
		/* if (get_user(val, (int *)arg)) { */
		/* 	ret = -EFAULT; */
		/* } */
		break;

	case SNDCTL_DSP_GETIPTR:
        {
            /* count_info cinfo; */
            /* if (!(mode & MODE_RECORD)) { */
            /*     ret = -EINVAL; */
            /* } */
        
            /* ret = copy_to_user((void *)arg, &cinfo, sizeof(cinfo)); */
            break;
        }

	case SNDCTL_DSP_GETOPTR:
        {
            /* count_info cinfo; */
            /* if (!(mode & MODE_REPLAY)) { */
            /*     ret = -EINVAL; */
            /* } */
        
            /* ret = copy_to_user((void *) arg, &cinfo, sizeof(cinfo)); */
            break;
        }

	case SNDCTL_DSP_GETODELAY:
        {
            int unfinish = 0;
            if (!(mode & MODE_REPLAY)) {
                ret = -EINVAL;
            }
            jz_audio_dma_get_odelay(&unfinish);
            ret = put_user(unfinish, (int *) arg);
            break;
        }

	case SOUND_PCM_READ_RATE:
		if (mode  & MODE_RECORD) {
			ret = put_user(codec_info.record_rate, (int *)arg);
		}
		if (mode & MODE_REPLAY) {
			ret = put_user(codec_info.replay_rate, (int *)arg);
		}
		break;

	case SOUND_PCM_READ_CHANNELS:
		if (mode & MODE_RECORD) {
			ret = put_user(codec_info.record_channels, (int *)arg);
		}
		if (mode & MODE_REPLAY) {
			ret = put_user(codec_info.replay_channels, (int *)arg);
		}
		break;

	case SOUND_PCM_READ_BITS:
        //sigle mode
		if (mode & MODE_RECORD) {
			if(codec_info.record_format & (AFMT_S8 | AFMT_U8)){
                format = 8;
			}
			else if(codec_info.record_format & (AFMT_S16_LE | AFMT_S16_BE)){
                format = 16;
			}
			else if(codec_info.record_format & (AFMT_S24_LE  | AFMT_S24_BE )){
                format = 24;
			}else{
                format = 24;
			}
			ret = put_user(format , (int *)arg);
		}
		if (mode & MODE_REPLAY) {
			if(codec_info.replay_format & (AFMT_S8 | AFMT_U8)){
                format = 8;
			}
			else if(codec_info.replay_format & (AFMT_S16_LE | AFMT_S16_BE)){
                format = 16;
			}
			else if(codec_info.replay_format & (AFMT_S24_LE  | AFMT_S24_BE )){
                format = 24;
			}else{
                format = 24;
			}
			ret = put_user(format, (int *)arg);
		}
		break;

	case SNDCTL_DSP_MAPINBUF:
	case SNDCTL_DSP_MAPOUTBUF:
	case SNDCTL_DSP_SETSYNCRO:
	case SOUND_PCM_WRITE_FILTER:
	case SOUND_PCM_READ_FILTER:
		ret = -EINVAL;
		break;
	default:
		printk("%s[%s]:%d---no cmd\n",__FILE__,__FUNCTION__,__LINE__);
		break;
	}

    return ret;
}

static int jz_snd_mixer_open(struct inode *inode, struct file *file)
{
	JZ_SND_DEBUG_MSG("enter %s\n", __func__);

    return 0;
}

static int jz_snd_mixer_release(struct inode *inode, struct file *file)
{
	JZ_SND_DEBUG_MSG("enter %s\n", __func__);

    return 0;
}

static int jz_snd_mixer_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char	buf_byte = 0;

	JZ_SND_DEBUG_MSG("enter %s\n", __func__);

	if (copy_from_user((void *)&buf_byte, buffer, 1)) {
		printk("JZ MIX: copy_from_user failed !\n");
		return -EFAULT;
	}
	switch (buf_byte) {
	case '1':
		codec_info.codec_ops->dump_regs("jz_snd_mixer_write --- debug routine");
		jz47xx_i2s_dump_regs("");
		break;
	}

	return count;
}

static int jz_snd_mixer_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	long	val = 0;
	int	ret = -EINVAL, rc = 0;

	JZ_SND_DEBUG_MSG("enter %s\n", __func__);

    if(down_interruptible(&codec_info.sem)){
        return -ERESTARTSYS;
    }
	switch (cmd) {
	case SOUND_MIXER_INFO:
        {
            /* mixer_info info; */
            /* codec_ioctrl(codec, CODEC_GET_MIXER_INFO, (unsigned int)&info); */
            /* info.modify_counter = g_mixer_modcnt; */
            /* ret = copy_to_user((void *)arg, &info, sizeof(info)); */
            break;
        }
	case SOUND_OLD_MIXER_INFO:
        {
            /* _old_mixer_info info; */
            /* codec_ioctrl(codec, CODEC_GET_MIXER_OLD_INFO, (unsigned int)&info); */
            /* ret = copy_to_user((void *)arg, &info, sizeof(info)); */
            break;
        }

	case SOUND_MIXER_READ_STEREODEVS:
		/* ret = put_user(0, (long *) arg); */
        break;

	case SOUND_MIXER_READ_CAPS:
		/* ret = put_user(SOUND_CAP_EXCL_INPUT, (long *) arg); */
        break;

	case SOUND_MIXER_READ_DEVMASK:
        ret = 0;
		break;
	case SOUND_MIXER_READ_RECMASK:
        ret = 0;
		break;
	case SOUND_MIXER_READ_RECSRC:
        ret = 0;
		break;

	case SOUND_MIXER_WRITE_SPEAKER:
		/* ret = get_user(val, (long *) arg); */
		/* if ((val &= 0xff) >= 100) { */
		/* 	val = 100; */
		/* } */
		/* codec_ioctrl(codec, CODEC_SET_DIRECT_MODE, val); */
		break;

	case SOUND_MIXER_WRITE_BASS:
		/* ret = get_user(val, (long *) arg); */
		/* if ((val &= 0xff) >= 100) { */
		/* 	val = 100; */
		/* } */
		/* codec->bass_gain = val; */
		/* codec_ioctrl(codec, CODEC_SET_BASS, val); */
        break;

	case SOUND_MIXER_READ_BASS:
		/* val = codec->bass_gain; */
		/* rc = val << 8; */
		/* val = val | rc; */
		/* ret = put_user(val, (long *) arg); */
        break;

	case SOUND_MIXER_WRITE_VOLUME:
		ret = get_user(val, (long *) arg);
        val &= 0xff;
        if(ret == 0){
            val = codec_info.codec_ops->set_replay_volume(val);
            codec_info.replay_volume = val;
        }
        break;

	case SOUND_MIXER_READ_VOLUME:
		val = codec_info.replay_volume;
		rc = val << 8;
		val = val | rc;
		ret = put_user(val, (long *) arg);
        break;

	case SOUND_MIXER_WRITE_MIC:
		ret = get_user(val, (long *) arg);
        val &= 0xff;
        if(ret == 0){
            val = codec_info.codec_ops->set_record_volume(val);
            codec_info.record_volume = val;
        }
        break;

	case SOUND_MIXER_READ_MIC:
		val = codec_info.record_volume;
		rc = val << 8;
		val = val | rc;
		ret = put_user(val, (long *) arg);
        break;

	case CODEC_SET_MODE:{
        long md;
		ret = get_user(md, (long *) arg);
        if(ret == 0){
            ret = codec_info.codec_ops->set_mode(md);
        }
        break;
    }

	case CODEC_SET_GCR:{
        struct codec_gcr_ctrl ctrl;
        ret = copy_from_user((void *)&ctrl, (void *)arg, sizeof(struct codec_gcr_ctrl));
        if(ret == 0){
            ret = codec_info.codec_ops->set_gcr(&ctrl);
        }
        break;
    }

	case SOUND_MIXER_WRITE_LINE:
		/* codec->use_mic_line_flag = USE_LINEIN; */
		/* codec->mic_gain = val; */
		//codec_ioctrl(codec, CODEC_SET_LINE, val);
        break;

	case SOUND_MIXER_READ_LINE:
		/* val = codec->mic_gain; */
		/* rc = val << 8; */
		/* val = val | rc; */
		/* ret = put_user(val, (long *) arg); */
        break;

	case SOUND_MIXER_WRITE_MUTE:
		/* ret = get_user(codec->audiomute, (long *)arg); */
		//codec_ioctrl(codec, CODEC_DAC_MUTE, codec->audiomute);
		break;

	case SOUND_MIXER_READ_MUTE:
		/* ret = put_user(codec->audiomute, (long *) arg); */
		break;

	default:
		printk("Mixer IOCTL error: %s:%d: known command: 0x%08x\n", __FUNCTION__, __LINE__, cmd);
		ret =  -ENOSYS;
        break;
	}
	g_mixer_modcnt++;
    up(&codec_info.sem);

	return ret;
}

/* static struct file_operations jz_i2s_audio_fops */
static struct file_operations jz_i2s_audio_fops = {
	owner:		THIS_MODULE,
	open:		jz_snd_dsp_open,
	release:	jz_snd_dsp_release,
	read:		jz_snd_dsp_read,
	write:		jz_snd_dsp_write,
	ioctl:		jz_snd_dsp_ioctl
};

static struct file_operations jz_i2s_mixer_fops =
{
	owner:		THIS_MODULE,
	open:		jz_snd_mixer_open,
	release:	jz_snd_mixer_release,
	write:		jz_snd_mixer_write,
	ioctl:		jz_snd_mixer_ioctl,
};

int dsp_dev, mixer_dev;

static int __init jz_snd_probe(struct platform_device *pdev)
{
	JZ_SND_DEBUG_MSG("enter %s\n", __func__);
    //----------------------------------------------------------------
    // init i2s
    //----------------------------------------------------------------
    if(jz47xx_i2s_init() != 0){
        goto i2s_init_failed;
    }

    //----------------------------------------------------------------
    // init codec
    //----------------------------------------------------------------
    if(codec_info.codec_ops != NULL){
        if(codec_info.codec_ops->triggle(CODEC_EVENT_INIT) != 0){
            goto codec_init_failed;
        }
    }else{
        goto codec_init_failed;
    }

    //----------------------------------------------------------------
    // init dma
    //----------------------------------------------------------------
    if(jz_audio_dma_init() != 0){
        goto dma_init_failed;
    }

    //----------------------------------------------------------------
    // init semaphore
    //----------------------------------------------------------------
    init_MUTEX(&controller.sem);
    init_MUTEX(&codec_info.sem);

    //----------------------------------------------------------------
    // regist /dev/dsp and /dev/mixer
    //----------------------------------------------------------------
    if((mixer_dev = register_sound_mixer(&jz_i2s_mixer_fops, -1)) < 0){
        goto regist_mixer_failed;
    }
	if((dsp_dev = register_sound_dsp(&jz_i2s_audio_fops, -1)) < 0){
        goto regist_dsp_failed;
    }

    //----------------------------------------------------------------
    // set default volume
    //----------------------------------------------------------------
    if(down_interruptible(&controller.sem)){
        return -ERESTARTSYS;
    }
    codec_info.replay_volume = INIT_DEFAULT_REPLAY_VOLUME;
    codec_info.record_volume = INIT_DEFAULT_RECORD_VOLUME;
    codec_info.codec_ops->set_replay_volume(codec_info.replay_volume);
    codec_info.codec_ops->set_record_volume(codec_info.record_volume);
    up(&controller.sem);
    
    //----------------------------------------------------------------
    // snd is ready
    //----------------------------------------------------------------
    controller.is_snd_ready = 1;

    return 0;

 regist_dsp_failed:
    unregister_sound_dsp(dsp_dev);
 regist_mixer_failed:
    unregister_sound_mixer(mixer_dev);
 dma_init_failed:
    jz_audio_dma_deinit();
 codec_init_failed:
    if(codec_info.codec_ops != NULL){
        codec_info.codec_ops->triggle(CODEC_EVENT_DEINIT);
    }
 i2s_init_failed:
    jz47xx_i2s_init();

    return -1;
}

static int jz_snd_remove(struct platform_device *pdev)
{
	JZ_SND_DEBUG_MSG("enter %s\n", __func__);
    controller.is_snd_ready = 0;

    //----------------------------------------------------------------
    // unregist /dev/dsp and /dev/mixer
    //----------------------------------------------------------------
    unregister_sound_dsp(dsp_dev);
    unregister_sound_mixer(mixer_dev);

    //----------------------------------------------------------------
    // deinit dma
    //----------------------------------------------------------------
    jz_audio_dma_deinit();

    //----------------------------------------------------------------
    // deinit codec
    //----------------------------------------------------------------
    if(codec_info.codec_ops != NULL){
        codec_info.codec_ops->triggle(CODEC_EVENT_DEINIT);
    }

    //----------------------------------------------------------------
    // deinit i2s
    //----------------------------------------------------------------
    jz47xx_i2s_init();

    return 0;
}

static int jz_snd_suspend(struct platform_device *pdev, pm_message_t state)
{
	JZ_SND_DEBUG_MSG("enter %s\n", __func__);
    if(codec_info.codec_ops != NULL){
        codec_info.codec_ops->triggle(CODEC_EVENT_SUSPEND);
    }

	return 0;
}

static int jz_snd_resume(struct platform_device *pdev)
{
	JZ_SND_DEBUG_MSG("enter %s\n", __func__);
    if(codec_info.codec_ops != NULL){
        codec_info.codec_ops->triggle(CODEC_EVENT_RESUME);
    }

	return 0;
}

static struct platform_driver jz_platform_driver = {
	.probe	    =  jz_snd_probe,
	.remove     =  jz_snd_remove,
	.suspend	=  jz_snd_suspend,
	.resume		=  jz_snd_resume,
	.driver	= {
		.name	= "mixer",
		.owner	= THIS_MODULE,
	},
};

/**
 * Module init
 */
static int __init jz_snd_init(void)
{
	JZ_SND_DEBUG_MSG("enter %s\n", __func__);
    return platform_driver_register(&jz_platform_driver);
}

module_init(jz_snd_init);

/**
 * Module exit
 */
static void __exit jz_snd_exit(void)
{
	JZ_SND_DEBUG_MSG("enter %s\n", __func__);
    platform_driver_unregister(&jz_platform_driver);
}
module_exit(jz_snd_exit);

MODULE_AUTHOR("liulu <lliu@ingenic.cn>");
MODULE_DESCRIPTION("jz sound driver core");
MODULE_LICENSE("GPL");
