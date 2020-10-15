#ifndef __JZ47XX_DATA_CONVERT_H__
#define __JZ47XX_DATA_CONVERT_H__

extern int convert_8bits_stereo2mono(void *buff, int data_len);
extern int convert_16bits_stereo2mono(void *buff, int data_len);
extern int convert_32bits_stereo2mono(void *buff, int data_len);

#endif /* __JZ47XX_DATA_CONVERT_H__ */
