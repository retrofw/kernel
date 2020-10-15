#ifndef __UCB1400_H__
#define __UCB1400_H__

/* ucb1400 aclink register mappings */

#define UCB_IO_DATA	0x5a
#define UCB_IO_DIR	0x5c
#define UCB_IE_RIS	0x5e
#define UCB_IE_FAL	0x60
#define UCB_IE_STATUS	0x62
#define UCB_IE_CLEAR	0x62
#define UCB_TS_CR	0x64
#define UCB_ADC_CR	0x66
#define UCB_ADC_DATA	0x68
#define UCB_ID		0x7e /* 7c is mfr id, 7e part id (from aclink spec) */

#define UCB_ADC_DAT(x)		((x) & 0x3ff)

/* register bits */

#define UCB_IO_0		(1 << 0)
#define UCB_IO_1		(1 << 1)
#define UCB_IO_2		(1 << 2)
#define UCB_IO_3		(1 << 3)
#define UCB_IO_4		(1 << 4)
#define UCB_IO_5		(1 << 5)
#define UCB_IO_6		(1 << 6)
#define UCB_IO_7		(1 << 7)
#define UCB_IO_8		(1 << 8)
#define UCB_IO_9		(1 << 9)

#define UCB_IE_ADC		(1 << 11)
#define UCB_IE_TSPX		(1 << 12)
#define UCB_IE_TSMX		(1 << 13)
#define UCB_IE_TCLIP		(1 << 14)
#define UCB_IE_ACLIP		(1 << 15)

#define UCB_IRQ_TSPX		12

#define UCB_TC_A_LOOP		(1 << 7)	/* UCB1200 */
#define UCB_TC_A_AMPL		(1 << 7)	/* UCB1300 */

#define UCB_TC_B_VOICE_ENA	(1 << 3)
#define UCB_TC_B_CLIP		(1 << 4)
#define UCB_TC_B_ATT		(1 << 6)
#define UCB_TC_B_SIDE_ENA	(1 << 11)
#define UCB_TC_B_MUTE		(1 << 13)
#define UCB_TC_B_IN_ENA		(1 << 14)
#define UCB_TC_B_OUT_ENA	(1 << 15)

#define UCB_AC_B_LOOP		(1 << 8)
#define UCB_AC_B_MUTE		(1 << 13)
#define UCB_AC_B_IN_ENA		(1 << 14)
#define UCB_AC_B_OUT_ENA	(1 << 15)

#define UCB_TS_CR_TSMX_POW	(1 << 0)
#define UCB_TS_CR_TSPX_POW	(1 << 1)
#define UCB_TS_CR_TSMY_POW	(1 << 2)
#define UCB_TS_CR_TSPY_POW	(1 << 3)
#define UCB_TS_CR_TSMX_GND	(1 << 4)
#define UCB_TS_CR_TSPX_GND	(1 << 5)
#define UCB_TS_CR_TSMY_GND	(1 << 6)
#define UCB_TS_CR_TSPY_GND	(1 << 7)
#define UCB_TS_CR_MODE_INT	(0 << 8)
#define UCB_TS_CR_MODE_PRES	(1 << 8)
#define UCB_TS_CR_MODE_POS	(2 << 8)
#define UCB_TS_CR_BIAS_ENA	(1 << 11)
#define UCB_TS_CR_TSPX_LOW	(1 << 12)
#define UCB_TS_CR_TSMX_LOW	(1 << 13)

#define UCB_ADC_SYNC_ENA	(1 << 0)
#define UCB_ADC_VREFBYP_CON	(1 << 1)
#define UCB_ADC_INP_TSPX	(0 << 2)
#define UCB_ADC_INP_TSMX	(1 << 2)
#define UCB_ADC_INP_TSPY	(2 << 2)
#define UCB_ADC_INP_TSMY	(3 << 2)
#define UCB_ADC_INP_AD0		(4 << 2)
#define UCB_ADC_INP_AD1		(5 << 2)
#define UCB_ADC_INP_AD2		(6 << 2)
#define UCB_ADC_INP_AD3		(7 << 2)
#define UCB_ADC_EXT_REF		(1 << 5)
#define UCB_ADC_START		(1 << 7)
#define UCB_ADC_ENA		(1 << 15)

#define UCB_ADC_DAT_VAL		(1 << 15)

#define UCB_ID_1200		0x1004
#define UCB_ID_1300		0x1005
#define UCB_ID_1400		0x4304
#define UCB_ID_1400_BUGGY	0x4303	/* fake ID */

#define UCB_MODE_DYN_VFLAG_ENA	(1 << 12)
#define UCB_MODE_AUD_OFF_CAN	(1 << 13)

/*
 * Which edges of the IRQ do you want to control today?
 */
#define UCB_RISING	(1 << 0)
#define UCB_FALLING	(1 << 1)

/* Device data structure */

struct ucb1400 {
	spinlock_t		lock;
	struct pm_dev		*pmdev;
	struct semaphore	adc_sem;
	u16			adc_cr;
	u16			irq_fal_enbl;
	u16			irq_ris_enbl;
	int			irq_enabled;
};

#endif /* __UCB1400_H__ */
