/*
 * Linux/sound/oss/jz4760_dlv.h
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZ4760_DLV_H__
#define __JZ4760_DLV_H__

/* Enable headphone detection */
//#define HP_SENSE_DETECT		1

/* Power setting */
#define POWER_ON		0
#define POWER_OFF		1

/* File ops mode W & R */
#define REPLAY			1
#define RECORD			2

/* Channels */
#define LEFT_CHANNEL		1
#define RIGHT_CHANNEL		2

#define MAX_RATE_COUNT		10


/*
 * JZ4760 DLV CODEC registers
 */

#define DLV_REG_SR		0

#define DLV_REG_AICR		1
 #define AICR_DAC_SERIAL	3
 #define AICR_ADC_SERIAL	2
 #define AICR_DAC_I2S		1
 #define AICR_ADC_I2S		0

#define DLV_REG_CR1		2
 #define CR1_LOAD		7
 #define CR1_HP_MUTE		5
 #define CR1_LINEOUT_MUTE	4
 #define CR1_BTL_MUTE		3
 #define CR1_OUTSEL1		1
 #define CR1_OUTSEL0		0
 #define CR1_OUTSEL_MASK	0x3

#define DLV_REG_CR2		3
 #define CR2_MONO		7
 #define CR2_DAC_MUTE		5
 #define CR2_NOMAD		1
 #define CR2_DAC_RIGHT_ONLY	0

#define DLV_REG_CR3		4
 #define CR3_INSEL1		3
 #define CR3_INSEL0		2
 #define CR3_MICSTEREO		1
 #define CR3_MICDIFF		0

#define DLV_REG_CR4		5
 #define CR4_ADC_HPF		7
 #define CR4_ADC_RIGHT_ONLY	0

#define DLV_REG_CCR1		6
 #define CCR1_CRYSTAL_BIT	0
 #define CCR1_CRYSTAL_MASK	0xf

#define DLV_REG_CCR2		7

#define DLV_REG_PMR1		8
 #define PMR1_SB		7
 #define PMR1_SB_SLEEP		6
 #define PMR1_SB_AIP		5
 #define PMR1_SB_LINE		4
 #define PMR1_SB_MIC1		3
 #define PMR1_SB_MIC2		2
 #define PMR1_SB_BYPASS		1
 #define PMR1_SB_MICBIAS	0

#define DLV_REG_PMR2		9
 #define PMR2_SB_ADC		4
 #define PMR2_SB_HP		3
 #define PMR2_SB_BTL		2
 #define PMR2_SB_LOUT		1
 #define PMR2_SB_DAC		0

#define DLV_REG_ICR		10
 #define ICR_INT_FORM_MASK	0xc0
 #define ICR_INT_FORM_BIT	6
 #define ICR_INT_HIGH		0x00
 #define ICR_INT_LOW		0x01
 #define ICR_INT_HIGH_8CYCLES	0x10
 #define ICR_JACK_MASK		(1 << 5)
 #define ICR_SCMC_MASK		(1 << 4)
 #define ICR_RUP_MASK		(1 << 3)
 #define ICR_RDO_MASK		(1 << 2)
 #define ICR_GUP_MASK		(1 << 1)
 #define ICR_GDO_MASK		(1 << 0)
 #define ICR_ALL_MASK							\
	(ICR_GDO_MASK | ICR_GUP_MASK | ICR_RDO_MASK | ICR_RUP_MASK | ICR_SCMC_MASK | ICR_JACK_MASK)

//#define ICR_COMMON_MASK	(ICR_GDO_MASK | ICR_GUP_MASK | ICR_RDO_MASK | ICR_RUP_MASK)
 #define ICR_COMMON_MASK	(ICR_GDO_MASK | ICR_GUP_MASK | ICR_RDO_MASK | ICR_RUP_MASK | ICR_JACK_MASK)

#define DLV_REG_IFR		11
 #define IFR_JACK		6
 #define IFR_JACKE		5
 #define IFR_SCMC		4
 #define IFR_RUP		3
 #define IFR_RDO		2
 #define IFR_GUP		1
 #define IFR_GDO		0

#define DLV_REG_GCR1		12
#define DLV_REG_GCR2		13
#define DLV_REG_GCR3		14
 #define GCR3_RLGI		7
 #define GCR3_GIR_BIT		0
 #define GCR3_GIR_MASK		0x1f

#define DLV_REG_GCR4		15
 #define GCR4_GIL_BIT		0
 #define GCR3_GIR_MASK		0x1f

#define DLV_REG_GCR5		16
#define DLV_REG_GCR6		17
#define DLV_REG_GCR7		18

#define DLV_REG_GCR8		19
 #define GCR8_GIDR_MASK		0x1f
 #define GCR8_GIDR_BIT		0

#define DLV_REG_GCR9		20
 #define GCR9_GIDL_MASK		0x1f
 #define GCR9_GIDL_BIT		0

#define DLV_REG_AGC1		21
 #define AGC1_AGCEN		7

#define DLV_REG_AGC2		22
#define DLV_REG_AGC3		23
#define DLV_REG_AGC4		24
#define DLV_REG_AGC5		25
#define DLV_REG_MIX1		26
 #define MIX1_REC_BIT		6
 #define MIX1_REC_MASK		0x3
 #define MIX1_GIMIX_BIT		0
 #define MIX1_GIMIX_MASK	0x1f

#define DLV_REG_MIX2		27

//#define HP_SENSE_DETECT

/*
 * Ops
 */

/* SB switch */

#define __dlv_get_sb_aip()		((dlv_read_reg(DLV_REG_PMR1) & (1 << PMR1_SB_AIP)) ? POWER_OFF : POWER_ON)

#define __dlv_switch_sb_aip(pwrstat)					\
do {									\
	if (__dlv_get_sb_aip() != pwrstat) {				\
		dlv_write_reg_bit(DLV_REG_PMR1, pwrstat, PMR1_SB_AIP);	\
	}								\
									\
} while (0)


#define __dlv_get_sb()			((dlv_read_reg(DLV_REG_PMR1) & (1 << PMR1_SB)) ? POWER_OFF : POWER_ON)

#define __dlv_switch_sb(pwrstat)					\
do {									\
	if (__dlv_get_sb() != pwrstat) {				\
		dlv_write_reg_bit(DLV_REG_PMR1, pwrstat, PMR1_SB);	\
	}								\
									\
} while (0)

#define __dlv_get_sb_sleep()		((dlv_read_reg(DLV_REG_PMR1) & (1 << PMR1_SB_SLEEP)) ? POWER_OFF : POWER_ON)

#define __dlv_switch_sb_sleep(pwrstat)					\
do {									\
	if (__dlv_get_sb_sleep() != pwrstat) {				\
		dlv_write_reg_bit(DLV_REG_PMR1, pwrstat, PMR1_SB_SLEEP);\
	}								\
									\
} while (0)

#define __dlv_get_sb_dac()		((dlv_read_reg(DLV_REG_PMR2) & (1 << PMR2_SB_DAC)) ? POWER_OFF : POWER_ON)

#define __dlv_switch_sb_dac(pwrstat)					\
do {									\
	if (__dlv_get_sb_dac() != pwrstat) {				\
		dlv_write_reg_bit(DLV_REG_PMR2, pwrstat, PMR2_SB_DAC);	\
	}								\
									\
} while (0)

#define __dlv_get_sb_line_out()		((dlv_read_reg(DLV_REG_PMR2) & (1 << PMR2_SB_LOUT)) ? POWER_OFF : POWER_ON)

#define __dlv_switch_sb_line_out(pwrstat)				\
do {									\
	if (__dlv_get_sb_line_out() != pwrstat) {			\
		dlv_write_reg_bit(DLV_REG_PMR2, pwrstat, PMR2_SB_LOUT);	\
	}								\
									\
} while (0)

#define __dlv_get_sb_hp()		((dlv_read_reg(DLV_REG_PMR2) & (1 << PMR2_SB_HP)) ? POWER_OFF : POWER_ON)

#define __dlv_switch_sb_hp(pwrstat)					\
do {									\
	if (__dlv_get_sb_hp() != pwrstat) {				\
		dlv_write_reg_bit(DLV_REG_PMR2, pwrstat, PMR2_SB_HP);	\
	}								\
									\
} while (0)

#define __dlv_get_sb_btl()		((dlv_read_reg(DLV_REG_PMR2) & (1 << PMR2_SB_BTL)) ? POWER_OFF : POWER_ON)

#define __dlv_switch_sb_btl(pwrstat)					\
do {									\
	if (__dlv_get_sb_btl() != pwrstat) {				\
		dlv_write_reg_bit(DLV_REG_PMR2, pwrstat, PMR2_SB_BTL);	\
	}								\
									\
} while (0)

#define __dlv_switch_sb_out(pwrstat)					\
do {									\
	/*__dlv_switch_sb_btl(pwrstat);*/				\
	__dlv_switch_sb_hp(pwrstat);					\
	/*__dlv_switch_sb_line_out(pwrstat);*/				\
} while (0)

#define __dlv_get_sb_adc()		((dlv_read_reg(DLV_REG_PMR2) & (1 << PMR2_SB_ADC)) ? POWER_OFF : POWER_ON)

#define __dlv_switch_sb_adc(pwrstat)					\
do {									\
	if (__dlv_get_sb_adc() != pwrstat) {				\
		dlv_write_reg_bit(DLV_REG_PMR2, pwrstat, PMR2_SB_ADC);	\
	}								\
									\
} while (0)

#define __dlv_get_sb_mic1()		((dlv_read_reg(DLV_REG_PMR1) & (1 << PMR1_SB_MIC1)) ? POWER_OFF : POWER_ON)

#define __dlv_switch_sb_mic1(pwrstat)					\
do {									\
	if (__dlv_get_sb_mic1() != pwrstat) {				\
		dlv_write_reg_bit(DLV_REG_PMR1, pwrstat, PMR1_SB_MIC1);	\
	}								\
									\
} while (0)

#define __dlv_get_sb_mic2()		((dlv_read_reg(DLV_REG_PMR1) & (1 << PMR1_SB_MIC2)) ? POWER_OFF : POWER_ON)

#define __dlv_switch_sb_mic2(pwrstat)					\
do {									\
	if (__dlv_get_sb_mic2() != pwrstat) {				\
		dlv_write_reg_bit(DLV_REG_PMR1, pwrstat, PMR1_SB_MIC2);	\
	}								\
									\
} while (0)

#define __dlv_get_sb_micbias()		((dlv_read_reg(DLV_REG_PMR1) & (1 << PMR1_SB_MICBIAS)) ? POWER_OFF : POWER_ON)

#define __dlv_switch_sb_micbias(pwrstat)				\
do {									\
	if (__dlv_get_sb_micbias() != pwrstat) {			\
		dlv_write_reg_bit(DLV_REG_PMR1, pwrstat, PMR1_SB_MICBIAS);\
	}								\
									\
} while (0)

#define __dlv_get_sb_line_in()		((dlv_read_reg(DLV_REG_PMR1) & (1 << PMR1_SB_LINE)) ? POWER_OFF : POWER_ON)

#define __dlv_switch_sb_line_in(pwrstat)				\
do {									\
	if (__dlv_get_sb_line_in() != pwrstat) {			\
		dlv_write_reg_bit(DLV_REG_PMR1, pwrstat, PMR1_SB_LINE);	\
	}								\
									\
} while (0)

#define __dlv_get_sb_bypass()		((dlv_read_reg(DLV_REG_PMR1) & (1 << PMR1_SB_BYPASS)) ? POWER_OFF : POWER_ON)

#define __dlv_switch_sb_bypass(pwrstat)					\
do {									\
	if (__dlv_get_sb_bypass() != pwrstat) {				\
		dlv_write_reg_bit(DLV_REG_PMR1, pwrstat, PMR1_SB_BYPASS);\
	}								\
									\
} while (0)

#define __dlv_set_int_form(v)						\
do {									\
	dlv_write_reg_bit(DLV_REG_ICR, ((v) >> 1) & 1, 7);		\
	dlv_write_reg_bit(DLV_REG_ICR, (v) & 1, 6);			\
} while (0)

#define __dlv_set_irq_mask(mask)					\
do {									\
	dlv_write_reg(DLV_REG_ICR, mask);				\
									\
} while (0)

#define __dlv_set_irq_flag(flag)					\
do {									\
	dlv_write_reg(DLV_REG_IFR, flag);				\
									\
} while (0)

#define __dlv_get_irq_flag()		(dlv_read_reg(DLV_REG_IFR))
#define __dlv_get_irq_mask()		(dlv_read_reg(DLV_REG_ICR) & 0x3f)

#define __dlv_set_16ohm_load()						\
do {									\
	dlv_write_reg_bit(DLV_REG_CR1, 0, CR1_LOAD);			\
									\
} while (0)

#define __dlv_set_10kohm_load()						\
do {									\
	dlv_write_reg_bit(DLV_REG_CR1, 1, CR1_LOAD);			\
									\
} while (0)

#define __dlv_enable_hp_mute()						\
do {									\
	dlv_write_reg_bit(DLV_REG_CR1, 1, CR1_HP_MUTE);			\
									\
} while (0)

#define __dlv_disable_hp_mute()						\
do {									\
	dlv_write_reg_bit(DLV_REG_CR1, 0, CR1_HP_MUTE);			\
									\
} while (0)

#define __dlv_enable_lineout_mute()					\
do {									\
	dlv_write_reg_bit(DLV_REG_CR1, 1, CR1_LINEOUT_MUTE);		\
									\
} while (0)

#define __dlv_disable_lineout_mute()					\
do {									\
	dlv_write_reg_bit(DLV_REG_CR1, 0, CR1_LINEOUT_MUTE);		\
									\
} while (0)

#define __dlv_enable_btl_mute()						\
do {									\
	dlv_write_reg_bit(DLV_REG_CR1, 1, CR1_BTL_MUTE);		\
									\
} while (0)

#define __dlv_disable_btl_mute()					\
do {									\
	dlv_write_reg_bit(DLV_REG_CR1, 0, CR1_BTL_MUTE);		\
									\
} while (0)

#define MIC1_TO_LR		0 //00b
#define MIC2_TO_LR		1 //01b
#define MIC1_TO_R_MIC2_TO_L	0 //00b
#define MIC2_TO_R_MIC1_TO_L	1 //01b
#define BYPASS_PATH		2 //10b
#define DAC_OUTPUT		3 //11b

#define __dlv_set_outsel(opt)						\
do {									\
	dlv_write_reg_bit(DLV_REG_CR1, (opt) & 0x1, CR1_OUTSEL0);	\
	dlv_write_reg_bit(DLV_REG_CR1, (opt >> 1) & 0x1, CR1_OUTSEL1);	\
} while (0)

#define __dlv_enable_dac_m2s()						\
do {									\
	dlv_write_reg_bit(DLV_REG_CR2, 1, CR2_MONO);			\
									\
} while (0)

#define __dlv_disable_dac_m2s()						\
do {									\
	dlv_write_reg_bit(DLV_REG_CR2, 0, CR2_MONO);			\
									\
} while (0)

#define __dlv_get_dac_mute()        ((dlv_read_reg(DLV_REG_CR2) & (1 << CR2_DAC_MUTE)) ? 1 : 0)

#define __dlv_enable_dac_mute()						\
do {									\
	dlv_write_reg_bit(DLV_REG_CR2, 1, CR2_DAC_MUTE);		\
									\
} while (0)

#define __dlv_disable_dac_mute()					\
do {									\
	dlv_write_reg_bit(DLV_REG_CR2, 0, CR2_DAC_MUTE);		\
									\
} while (0)

#define __dlv_enable_nomad()						\
do {									\
	dlv_write_reg_bit(DLV_REG_CR2, 1, CR2_NOMAD);			\
									\
} while (0)

#define __dlv_disable_nomad()						\
do {									\
	dlv_write_reg_bit(DLV_REG_CR2, 0, CR2_NOMAD);			\
									\
} while (0)


#define __dlv_enable_dac_right_only()					\
do {									\
	dlv_write_reg_bit(DLV_REG_CR2, 1, CR2_DAC_RIGHT_ONLY);		\
									\
} while (0)

#define __dlv_disable_dac_right_only()					\
do {									\
	dlv_write_reg_bit(DLV_REG_CR2, 0, CR2_DAC_RIGHT_ONLY);		\
									\
} while (0)

/* Following macros are the arguments for __dlv_set_insel.
 * And they have already defined.
 *
#define MIC1_TO_LR		0 //00b
#define MIC2_TO_LR		1 //01b
#define MIC1_TO_R_MIC2_TO_L	0 //00b
#define MIC2_TO_R_MIC1_TO_L	1 //01b
 *
 */
#define LINE_INPUT		2 //10b

#define __dlv_set_insel(opt)						\
do {									\
	dlv_write_reg_bit(DLV_REG_CR3, (opt) & 1, CR3_INSEL0);		\
	dlv_write_reg_bit(DLV_REG_CR3, (opt >> 1) & 1, CR3_INSEL1);	\
} while (0)

#define __dlv_set_mic_stereo()						\
do {									\
	dlv_write_reg_bit(DLV_REG_CR3, 1, CR3_MICSTEREO);		\
									\
} while (0)

#define __dlv_set_mic_mono()						\
do {									\
	dlv_write_reg_bit(DLV_REG_CR3, 0, CR3_MICSTEREO);		\
									\
} while (0)

#define __dlv_enable_mic_diff()					\
do {								\
	dlv_write_reg_bit(DLV_REG_CR3, 1, CR3_MICDIFF);		\
								\
} while (0)

#define __dlv_disable_mic_diff()				\
do {								\
	dlv_write_reg_bit(DLV_REG_CR3, 0, CR3_MICDIFF);		\
								\
} while (0)

#define __dlv_enable_adc_high_pass()				\
do {								\
	dlv_write_reg_bit(DLV_REG_CR4, 1, CR4_ADC_HPF);		\
								\
} while (0)

#define __dlv_disable_adc_high_pass()				\
do {								\
	dlv_write_reg_bit(DLV_REG_CR4, 0, CR4_ADC_HPF);		\
								\
} while (0)

#define __dlv_enable_adc_right_only()				\
do {								\
	dlv_write_reg_bit(DLV_REG_CR4, 1, CR4_ADC_RIGHT_ONLY);	\
								\
} while (0)

#define __dlv_disable_adc_right_only()				\
do {								\
	dlv_write_reg_bit(DLV_REG_CR4, 0, CR4_ADC_RIGHT_ONLY);	\
								\
} while (0)

#define __dlv_set_hp_volume(vol)				\
do {								\
	dlv_write_reg(DLV_REG_GCR1, vol |= 0x80);		\
								\
} while (0)

#define __dlv_set_line_in_bypass_volume(vol)			\
do {								\
	vol = (vol & GCR3_GIR_MASK) | (1 << GCR3_RLGI);		\
	dlv_write_reg(DLV_REG_GCR3, vol);			\
								\
} while (0)

#define __dlv_set_line_in_bypass_volume_rl(lvol, rvol)		\
do {								\
	rvol = rvol & GCR3_GIR_MASK;				\
	dlv_write_reg(DLV_REG_GCR3, rvol);			\
	dlv_write_reg(DLV_REG_GCR4, lvol);			\
								\
} while (0)

#define __dlv_set_mic_1_volume(vol)				\
do {								\
	dlv_write_reg(DLV_REG_GCR8, vol | 0x80);		\
								\
} while (0)

#define __dlv_set_gim(value)					\
do {								\
	dlv_write_reg(DLV_REG_GCR7, (value));			\
								\
} while (0)

#define __dlv_set_gidr(value)					\
do {								\
	unsigned int r = dlv_read_reg(DLV_REG_GCR8);		\
	r = (r & ~GCR8_GIDR_MASK) | ((value) << GCR8_GIDR_BIT);	\
	dlv_write_reg(DLV_REG_GCR8, r << GCR8_GIDR_BIT);	\
								\
} while (0)

#define __dlv_set_gidl(value)					\
do {								\
	unsigned int r = dlv_read_reg(DLV_REG_GCR9);		\
	r = (r & ~GCR9_GIDL_MASK) | ((value) << GCR9_GIDL_BIT);	\
	dlv_write_reg(DLV_REG_GCR9, r << GCR9_GIDL_BIT);	\
								\
} while (0)

#define __dlv_set_godr(value)					\
do {								\
	dlv_write_reg(DLV_REG_GCR5, value);	                \
	                                                        \
} while (0)

#define __dlv_set_godl(value)					\
do {								\
	dlv_write_reg(DLV_REG_GCR6, value);			\
								\
} while (0)

#define __dlv_enable_agc()					\
do {								\
	dlv_write_reg_bit(DLV_REG_AGC1, 1, AGC1_AGCEN);		\
								\
} while (0)

#define __dlv_disable_agc()					\
do {								\
	dlv_write_reg_bit(DLV_REG_AGC1, 0, AGC1_AGCEN);		\
								\
} while (0)

#define __dlv_set_12m_crystal()					\
do {								\
	dlv_write_reg_bit(DLV_REG_CCR1, 0, 1);			\
	dlv_write_reg_bit(DLV_REG_CCR1, 0, 0);			\
								\
} while (0)

#define __dlv_set_mix_rec_2_dac()				\
do {								\
	dlv_write_reg_bit(DLV_REG_MIX1, 1, 6);			\
	dlv_write_reg_bit(DLV_REG_CCR1, 0, 7);			\
								\
} while (0)

#define __dlv_set_mix_rec_only()				\
do {								\
	dlv_write_reg_bit(DLV_REG_MIX1, 0, 6);			\
	dlv_write_reg_bit(DLV_REG_CCR1, 0, 7);			\
								\
} while (0)

#define __dlv_reset_rup()                                       \
do {                                                            \
	dlv_write_reg_bit(DLV_REG_IFR, 1, IFR_RUP);             \
	                                                        \
} while (0)

#define __dlv_reset_gup()                                       \
do {                                                            \
	dlv_write_reg_bit(DLV_REG_IFR, 1, IFR_GUP);             \
                                                                \
} while (0)

#define __dlv_reset_gdo()                                       \
do {                                                            \
	dlv_write_reg_bit(DLV_REG_IFR, 1, IFR_GDO);             \
                                                                \
} while (0)

#define __dlv_reset_rdo()                                       \
do {                                                            \
	dlv_write_reg_bit(DLV_REG_IFR, 1, IFR_RDO);             \
                                                                \
} while (0)

#ifdef CONFIG_HP_SENSE_DETECT
/*
 * HP_SENSE switch
 */
typedef struct {
	struct switch_dev	sdev;
	const char		*name;
	const char		*name_on;
	const char		*name_off;
	const char		*state_on;
	const char		*state_off;

} jz_hp_switch_data_t ;

typedef struct {
	const char		*name;
	const char		*name_on;
	const char		*name_off;
	const char		*state_on;
	const char		*state_off;

} jz_hp_switch_platform_data_t ;

#endif

#endif // __JZ4760_DLV_H__
