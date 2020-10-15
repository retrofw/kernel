#include <linux/types.h>
#include "jz47xx_data_cvt.h"
#include <asm/bug.h>

int convert_8bits_stereo2mono(void *buff, int data_len)
{
	int data_len_16aligned = data_len & ~0xf;
	int mono_cur, stereo_cur;
	u8 *uc_buff = buff;

	/* copy 8 times each loop */
	for (stereo_cur = mono_cur = 0;
	     stereo_cur < data_len_16aligned;
	     stereo_cur += 16, mono_cur += 8) {

		uc_buff[mono_cur + 0] = uc_buff[stereo_cur + 0];
		uc_buff[mono_cur + 1] = uc_buff[stereo_cur + 2];
		uc_buff[mono_cur + 2] = uc_buff[stereo_cur + 4];
		uc_buff[mono_cur + 3] = uc_buff[stereo_cur + 6];
		uc_buff[mono_cur + 4] = uc_buff[stereo_cur + 8];
		uc_buff[mono_cur + 5] = uc_buff[stereo_cur + 10];
		uc_buff[mono_cur + 6] = uc_buff[stereo_cur + 12];
		uc_buff[mono_cur + 7] = uc_buff[stereo_cur + 14];
	}

	BUG_ON(stereo_cur != data_len_16aligned);

	/* remaining data */
	for (; stereo_cur < data_len; stereo_cur += 2, mono_cur++) {
		uc_buff[mono_cur] = uc_buff[stereo_cur];
	}

	return (data_len / 2);
}

int convert_16bits_stereo2mono(void *buff, int data_len)
{
	int data_len_32aligned = data_len & ~0x1f;
	int data_cnt_ushort = data_len_32aligned / 2;
	int mono_cur, stereo_cur;
	u16 *ushort_buff = buff;

	/* copy 8 times each loop */
	for (stereo_cur = mono_cur = 0;
	     stereo_cur < data_cnt_ushort;
	     stereo_cur += 16, mono_cur += 8) {

		ushort_buff[mono_cur + 0] = ushort_buff[stereo_cur + 0];
		ushort_buff[mono_cur + 1] = ushort_buff[stereo_cur + 2];
		ushort_buff[mono_cur + 2] = ushort_buff[stereo_cur + 4];
		ushort_buff[mono_cur + 3] = ushort_buff[stereo_cur + 6];
		ushort_buff[mono_cur + 4] = ushort_buff[stereo_cur + 8];
		ushort_buff[mono_cur + 5] = ushort_buff[stereo_cur + 10];
		ushort_buff[mono_cur + 6] = ushort_buff[stereo_cur + 12];
		ushort_buff[mono_cur + 7] = ushort_buff[stereo_cur + 14];
	}

	BUG_ON(stereo_cur != data_cnt_ushort);

	/* remaining data */
	for (; stereo_cur < data_cnt_ushort; stereo_cur += 2, mono_cur++) {
		ushort_buff[mono_cur] = ushort_buff[stereo_cur];
	}

	return (data_len / 2);
}

int convert_32bits_stereo2mono(void *buff, int data_len)
{
	int data_len_32aligned = data_len & ~0x3f;
	int data_cnt_uint = data_len_32aligned / 4;
	int mono_cur, stereo_cur;
	u32 *uint_buff = buff;

	/* copy 8 times each loop */
	for (stereo_cur = mono_cur = 0;
	     stereo_cur < data_cnt_uint;
	     stereo_cur += 16, mono_cur += 8) {

		uint_buff[mono_cur + 0] = uint_buff[stereo_cur + 0];
		uint_buff[mono_cur + 1] = uint_buff[stereo_cur + 2];
		uint_buff[mono_cur + 2] = uint_buff[stereo_cur + 4];
		uint_buff[mono_cur + 3] = uint_buff[stereo_cur + 6];
		uint_buff[mono_cur + 4] = uint_buff[stereo_cur + 8];
		uint_buff[mono_cur + 5] = uint_buff[stereo_cur + 10];
		uint_buff[mono_cur + 6] = uint_buff[stereo_cur + 12];
		uint_buff[mono_cur + 7] = uint_buff[stereo_cur + 14];
	}

	BUG_ON(stereo_cur != data_cnt_uint);

	/* remaining data */
	for (; stereo_cur < data_cnt_uint; stereo_cur += 2, mono_cur++) {
		uint_buff[mono_cur] = uint_buff[stereo_cur];
	}

	return (data_len / 2);
}

