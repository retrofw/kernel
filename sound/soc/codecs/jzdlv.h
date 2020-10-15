/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DLV_H
#define _DLV_H

/* jzdlv register space */

#define DLV_AICR    0x00
#define DLV_CR1     0x01
#define DLV_CR2     0x02
#define DLV_CCR1    0x03
#define DLV_CCR2    0x04
#define DLV_PMR1    0x05
#define DLV_PMR2    0x06
#define DLV_CRR     0x07
#define DLV_ICR     0x08
#define DLV_IFR     0x09
#define DLV_CGR1    0x0a
#define DLV_CGR2    0x0b
#define DLV_CGR3    0x0c
#define DLV_CGR4    0x0d
#define DLV_CGR5    0x0e
#define DLV_CGR6    0x0f
#define DLV_CGR7    0x10
#define DLV_CGR8    0x11
#define DLV_CGR9    0x12
#define DLV_CGR10   0x13
#define DLV_TR1     0x14
#define DLV_TR2     0x15
#define DLV_CR3     0x16
#define DLV_AGC1    0x17
#define DLV_AGC2    0x18
#define DLV_AGC3    0x19
#define DLV_AGC4    0x1a
#define DLV_AGC5    0x1b

#define JZDLV_CACHEREGNUM  (DLV_AGC5+1)
#define JZDLV_SYSCLK	0

int read_codec_file(int addr);
int write_codec_file_bit(int addr, int bitval, int mask_bit);
void write_codec_file(int addr, int val);

extern struct snd_soc_dai jzdlv_dai;
extern struct snd_soc_codec_device soc_codec_dev_jzdlv;

#endif
