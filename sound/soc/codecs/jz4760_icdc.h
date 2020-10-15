/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __JZ47XX_ICDC_H__
#define __JZ47XX_ICDC_H__

#include <asm/jzsoc.h>

/* JZ internal register space */
enum {
	JZ_ICDC_SR	= 0x00,
	JZ_ICDC_AICR	= 0x01,
#define ICDC_AICR_ADC_I2S	0
#define ICDC_AICR_DAC_I2S	1
#define ICDC_AICR_ADC_SERIAL	2
#define ICDC_AICR_DAC_SERIAL	3
#define ICDC_AICR_ADC_ADWL_LSB  4
#define ICDC_AICR_ADC_ADWL_MASK	0x3
#define ICDC_AICR_DAC_ADWL_LSB  6
#define ICDC_AICR_DAC_ADWL_MASK	0x3

	JZ_ICDC_CR1	= 0x02,
#define ICDC_CR1_OUTSEL_LSB	0
#define ICDC_CR1_OUTSEL_MSB	1
#define ICDC_CR1_BTL_MUTE	3
#define ICDC_CR1_LOUT_MUTE	4
#define ICDC_CR1_HP_MUTE	5
#define ICDC_CR1_LOAD		7

	JZ_ICDC_CR2	= 0x03,
#define ICDC_CR2_DAC_RIGHT_ONLY	0
#define ICDC_CR2_NOMAD		1
#define ICDC_CR2_DAC_MUTE	5
#define ICDC_CR2_MONO		7

	JZ_ICDC_CR3	= 0x04,
#define ICDC_CR3_MICDIFF	0
#define ICDC_CR3_MICSTEREO	1
#define ICDC_CR3_INSEL_LSB	2
#define ICDC_CR3_INSEL_MASK	0x3

	JZ_ICDC_CR4	= 0x05,
#define ICDC_CR4_ADC_RIGHT_ONLY	0
#define ICDC_CR4_ADC_HPF	7

	JZ_ICDC_CCR1	= 0x06,
#define ICDC_CCR1_CRYSTAL_LSB	0
#define ICDC_CCR1_CRYSTAL_MASK	0xf
#define ICDC_CRYSTAL_12M	0x0
#define ICDC_CRYSTAL_13M	0x1

	JZ_ICDC_CCR2	= 0x07,
#define ICDC_CCR2_ADC_FREQ_LSB	0
#define ICDC_CCR2_ADC_FREQ_MASK	0xf
#define ICDC_CCR2_DAC_FREQ_LSB	4
#define ICDC_CCR2_DAC_FREQ_MASK	0xf

	JZ_ICDC_PMR1	= 0x08,
#define ICDC_PMR1_SB_BIAS	0
#define ICDC_PMR1_SB_BYPASS	1
#define ICDC_PMR1_SB_MIC2	2
#define ICDC_PMR1_SB_MIC1	3
#define ICDC_PMR1_SB_LINE	4
#define ICDC_PMR1_SB_AIP	5
#define ICDC_PMR1_SB_SLEEP	6
#define ICDC_PMR1_SB		7

	JZ_ICDC_PMR2	= 0x09,
#define ICDC_PMR2_SB_DAC	0
#define ICDC_PMR2_SB_LOUT	1
#define ICDC_PMR2_SB_BTL	2
#define ICDC_PMR2_SB_HP		3
#define ICDC_PMR2_SB_ADC	4

	JZ_ICDC_ICR	= 0x0a,
#define ICDC_ICR_GDO_MASK	0
#define ICDC_ICR_GUP_MASK	1
#define ICDC_ICR_RDO_MASK	2
#define ICDC_ICR_RUP_MASK	3
#define ICDC_ICR_SCMC_MASK	4
#define ICDC_ICR_JACK_MASK	5
#define ICDC_ICR_INT_FORM_LSB	6
#define ICDC_ICR_INT_FORM_MASK	0x3
#define ICDC_INT_FORM_HIGH	0x0
#define ICDC_INT_FORM_LOW	0x1
#define ICDC_INT_FORM_HIGH8	0x2
#define ICDC_INT_FORM_LOW8	0x3

	JZ_ICDC_IFR	= 0x0b,
#define ICDC_IFR_GDO	0
#define ICDC_IFR_GUP	1
#define ICDC_IFR_RDO	2
#define ICDC_IFR_RUP	3
#define ICDC_IFR_SCMC	4
#define ICDC_IFR_JACK_EVENT	5
#define ICDC_IFR_JACK	6

	JZ_ICDC_CGR1	= 0x0c,
	JZ_ICDC_CGR2	= 0x0d,
	JZ_ICDC_CGR3	= 0x0e,
	JZ_ICDC_CGR4	= 0x0f,
	JZ_ICDC_CGR5	= 0x10,
	JZ_ICDC_CGR6	= 0x11,
	JZ_ICDC_CGR7	= 0x12,
	JZ_ICDC_CGR8	= 0x13,
	JZ_ICDC_CGR9	= 0x14,
	JZ_ICDC_AGC1	= 0x15,
	JZ_ICDC_AGC2	= 0x16,
	JZ_ICDC_AGC3	= 0x17,
	JZ_ICDC_AGC4	= 0x18,
	JZ_ICDC_AGC5	= 0x19,
	JZ_ICDC_MIX1	= 0x1a,
	JZ_ICDC_MIX2	= 0x1b,
	JZ_ICDC_MAX_REGNUM,

	/* virtual registers */
	JZ_ICDC_LOUTSEL  = 0x1c,
	JZ_ICDC_ROUTSEL  = 0x1d,

	JZ_ICDC_LINSEL	= 0x1e,
	JZ_ICDC_RINSEL	= 0x1f,

	JZ_ICDC_MAX_NUM
};

extern struct snd_soc_dai jz_icdc_dai;
extern struct snd_soc_codec_device jz_icdc_soc_codec_dev;

#endif	/* __JZ47XX_ICDC_H__ */
