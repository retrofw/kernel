/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _ICODEC_H
#define _ICODEC_H

/* jzcodec register space */
#define ICODEC_1_LOW   0x00  /* bit0 -- bit15 in CDCCR1 */
#define ICODEC_1_HIGH  0x01  /* bit16 -- bit31 in CDCCR1 */
#define ICODEC_2_LOW   0x02  /* bit0 -- bit16 in CDCCR2 */
#define ICODEC_2_HIGH  0x03  /* bit16 -- bit31 in CDCCR2 */

#define JZCODEC_CACHEREGNUM  4
#define JZCODEC_SYSCLK	0

extern struct snd_soc_dai jzcodec_dai;
extern struct snd_soc_codec_device soc_codec_dev_jzcodec;

#endif
