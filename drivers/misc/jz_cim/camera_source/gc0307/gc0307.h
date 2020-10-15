#ifndef __GC0307_H__
#define __GC0307_H__

#define GC0307_CHIPID	0x00

/*exp reg */
#ifdef ZEM800
#define GC0307_EXP_H        0x03
#define GC0307_EXP_L        0x04

#define GC0307_ROW_H        0x92
#define GC0307_ROW_L        0x93
#define GC0307_COL_H        0x94
#define GC0307_COL_L        0x95
#define GC0307_WINH_H       0x96
#define GC0307_WINH_L       0x97
#define GC0307_WINW_H       0x98
#define GC0307_WINW_L       0x99
#else
#define GC0307_EXP_H        0x03
#define GC0307_EXP_L        0x04

/* windows regs*/
#define GC0307_ROW_H        0x05
#define GC0307_ROW_L        0x06
#define GC0307_COL_H        0x07
#define GC0307_COL_L        0x08
#define GC0307_WINH_H       0x09
#define GC0307_WINH_L       0x0A
#define GC0307_WINW_H       0x0B
#define GC0307_WINW_L       0x0C
#endif

#define GC0307_CISCTL_MODE2 0x0E
#define GC0307_CISCTL_MODE1 0x0F

#define GC0307_SCTRA 0x48

void set_gc0307_i2c_client(struct i2c_client *client);

//***************************************************
/////////	对外的接口

void sensor_init_GC307(int left, int top, int width, int height);

void set_sensitivity_GC307();

//设置图像是否需要左右镜像处理
void set_symmetry_GC307(int symmetry);

void sensor_power_down_GC307(void);

////////	对外借口部分已经全部结束
//***************************************************



//***************************************************
/////////	对内部的借口
void sensor_power_on_GC307(void);
/* VCLK = MCLK/div */
void set_EXPOS_GC307(int exposH);

void _initgc307(void);
void select_page_gc307(int page);
////////	对内借口已经结束
//***************************************************
















#endif
