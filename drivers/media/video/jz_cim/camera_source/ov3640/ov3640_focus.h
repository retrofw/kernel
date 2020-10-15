
#ifndef __OV3640_FOUCS_H__
#define __OV3640_FOCUS_H__

#include<linux/i2c.h>
#include"../../jz_cim_core.h"


#define S_STARTUP		0xff
#define S_DRIVER_ERR 		0xee
#define S_IDLE			0x00
#define S_STEP			0x20

#define STEP_STATE_NO 		0x80
#define MASK_MODECHANGE 	0x40
#define MASK_CAPTURECMD 	0x20
#define RESERVED_INVALID 	0x10
#define MODE_IDLE 		0x00
#define MODE_SINGLE 		0x04
#define MODE_CONTINUE 		0x08
#define MODE_STEP 		0x0c
#define MODE_STEP_INSTRUCTION 	0x00
#define MODE_STEP_FOCUSING 	0x01
#define MODE_STEP_FOCUSED 	0x02
#define MODE_STEP_CAPTURE 	0x03

#define STATE_INF 		(MODE_IDLE|MODE_STEP_INSTRUCTION)
#define STATE_SINGLE 		(MODE_SINGLE|MODE_STEP_FOCUSING|MASK_MODECHANGE|MASK_CAPTURECMD)

#define STATE_SUCCESS_S 	(MODE_SINGLE|MODE_STEP_FOCUSED|MASK_MODECHANGE)
#define STATE_FAIL_S 		(MODE_SINGLE|MODE_STEP_FOCUSED|MASK_MODECHANGE|STEP_STATE_NO)
#define STATE_CAPTURE_S 	(MODE_SINGLE|MODE_STEP_CAPTURE|MASK_MODECHANGE)

#define REG_CMD		0x3f00
#define REG_TAG 	0x3f01
#define REG_PARA0 	0x3f05
#define REG_STATE 	0x3f07

int ov3640_do_focus(struct i2c_client *client);
int ov3640_stop_focus(struct i2c_client *client);
int ov3640_af_init(struct i2c_client *client,struct sensor_af_arg *info);
int ov3640_set_af_mode(int mode);


int ov3640_do_auto_focus(struct i2c_client *client);
int ov3640_do_fixed_focus(struct i2c_client *client);
int ov3640_do_infinity_focus(struct i2c_client *client);
int ov3640_do_macro_focus(struct i2c_client *client);

#endif
