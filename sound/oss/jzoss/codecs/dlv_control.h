#ifndef __DLV_CONTROL_H__
#define __DLV_CONTROL_H__

#define CODEC_GCR_CTRL_NAME_LEN	16

//mode
#define CODEC_MODE_OFF         (0x1U << 0)
#define CODEC_MODE_MIC1_2_ADC         (0x1U << 1)//adc
#define CODEC_MODE_MIC2_2_ADC         (0x1U << 2)
#define CODEC_MODE_MIC1_MIC2_2_ADC    (0x1U << 3)
#define CODEC_MODE_MIC2_MIC1_2_ADC    (0x1U << 4)
#define CODEC_MODE_LINE_IN_2_ADC      (0x1U << 5)
#define CODEC_MODE_MIC1_2_HP       (0x1U << 6)//hp
#define CODEC_MODE_MIC2_2_HP       (0x1U << 7)
#define CODEC_MODE_MIC1_MIC2_2_HP  (0x1U << 8)
#define CODEC_MODE_MIC2_MIC1_2_HP  (0x1U << 9)
#define CODEC_MODE_LINE_IN_2_HP    (0x1U << 10)
#define CODEC_MODE_DAC_2_HP        (0x1U << 11)
#define CODEC_MODE_MIC1_2_LO            (0x1U << 12)//line out
#define CODEC_MODE_MIC2_2_LO            (0x1U << 13)
#define CODEC_MODE_MIC1_MIC2_2_LO       (0x1U << 14)
#define CODEC_MODE_LINE_IN_2_LO         (0x1U << 15)
#define CODEC_MODE_DAC_2_LO             (0x1U << 16)
#define CODEC_MODE_MIC1_2_JD       (0x1U << 17)//jack detect
#define CODEC_MODE_MIC2_2_JD       (0x1U << 18)
#define CODEC_MODE_MIC1_MIC2_2_JD  (0x1U << 19)
#define CODEC_MODE_MIC2_MIC1_2_JD  (0x1U << 20)
#define CODEC_MODE_LINE_IN_2_JD    (0x1U << 21)
#define CODEC_MODE_DAC_2_JD        (0x1U << 22)


/**
 * Please see spec. Name is the name of gcr, and gain will set to gcr.
 */
struct codec_gcr_ctrl {
	char name[CODEC_GCR_CTRL_NAME_LEN];
    int gain1;
    int gain2;
};

/**
 * arg: int
 * arg direction: IN
 * return: 0
 */
#define CODEC_SET_MODE		SOUND_MIXER_PRIVATE1

/**
 * arg: struct codec_gcr_ctrl*
 * arg direction: IN/OUT
 * return: 0
 */
#define CODEC_SET_GCR		SOUND_MIXER_PRIVATE2

#endif /* __DLV_CONTROL_H__ */
