//
// Copyright (c) Ingenic Semiconductor Co., Ltd. 2008.
//

#ifndef __OV3640SET_H__
#define __OV3640SET_H__

#include <linux/i2c.h>

void init_set(struct i2c_client *client);
void preview_set(struct i2c_client *client);
void capture_set(struct i2c_client *client);
void size_switch(struct i2c_client *client,int width,int height,int setmode);


#endif //__OV3640SET_H__
