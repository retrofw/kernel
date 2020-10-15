#ifndef __GC0308_H__
#define __GC0308_H__

#define GC0308_CHIPID	0x00

/*exp reg */
#define GC0308_EXP_H        0x03
#define GC0308_EXP_L        0x04

/* windows regs*/
#define GC0308_ROW_H        0x05
#define GC0308_ROW_L        0x06
#define GC0308_COL_H        0x08
#define GC0308_COL_L        0x07
#define GC0308_WINH_H       0x09
#define GC0308_WINH_L       0x0A
#define GC0308_WINW_H       0x0B
#define GC0308_WINW_L       0x0C

#define GC0308_CISCTL_MODE2 0x0E
#define GC0308_CISCTL_MODE1 0x0F

#define GC0308_SCTRA 0x28

void set_gc0308_i2c_client(struct i2c_client *client);


//***************************************************
/////////	对外的接口

void sensor_init_GC308(int left, int top, int width, int height);

void set_sensitivity_GC308(void);

//设置图像是否需要左右镜像处理
void set_symmetry_GC308(int symmetry);

void sensor_power_down_GC308(void);

////////	对外借口部分已经全部结束
//***************************************************



//***************************************************
/////////	对内部的借口
void sensor_power_on_GC308(void);
/* VCLK = MCLK/div */
void set_EXPOS_GC308(int exposH);

void _initgc308(void);
void select_page_gc308(int page);
////////	对内借口已经结束
//***************************************************

#endif
