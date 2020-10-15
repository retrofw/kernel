#ifndef __JZ47XX_DMA_H__
#define __JZ47XX_DMA_H__

#include <linux/soundcard.h>

#define DEFAULT_NR_FRAG		5
#define DEFAULT_FRAG_SIZE	PAGE_SIZE

#define DEFAULT_DATA_WIDTH	16
#define DEFAULT_SND_CHANNELS	2

extern int jz_audio_dma_init(void);
extern int jz_audio_dma_deinit(void);

extern int jz_audio_dma_resize_buffer(int mode, int nr_frag, int frag_size);

extern int jz_audio_dma_push(const u8 *data /* user */, int len, int block);
extern int jz_audio_dma_pull(u8 *data /* user */, int len, int block);

extern void jz_audio_dma_set_width(int mode, int width);
extern void jz_audio_dma_set_channels(int mode, int width);

extern int jz_audio_dma_stop(int mode);
extern int jz_audio_dma_start(int mode);
extern void jz_audio_dma_sync(int mode);
extern int jz_audio_dma_flush(int mode);

extern int jz_audio_dma_get_ospace(audio_buf_info *abinfo);
extern int jz_audio_dma_get_ispace(audio_buf_info *abinfo);
extern int jz_audio_dma_get_odelay(int *unfinish);

#endif /* __JZ47XX_DMA_H__ */
