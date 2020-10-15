/*
 * linux/sound/soc/jz4750/jz4750-ac97.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _JZ4750_AC97_H
#define _JZ4750_AC97_H

#define JZ4750_DAI_AC97_HIFI	0
#define JZ4750_DAI_AC97_AUX		1
#define JZ4750_DAI_AC97_MIC		2

extern struct snd_soc_cpu_dai jz4750_ac97_dai[3];

/* platform data */
extern struct snd_ac97_bus_ops jz4750_ac97_ops;

#endif
