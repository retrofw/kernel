#ifndef OV7690_INIT_H
#define OV7690_INIT_H
#include <linux/i2c.h>


int ov7690_format_yuv422(struct i2c_client *client);
int ov7690_format_gbr422(struct i2c_client *client);
int ov7690_format_rgb565(struct i2c_client *client);
int ov7690_format_wbcraw(struct i2c_client *client);
int ov7690_format_cipraw(struct i2c_client *client);

int ov7690_640_480(struct i2c_client *client);
int ov7690_480_320(struct i2c_client *client);
int ov7690_352_288(struct i2c_client *client);
int ov7690_320_240(struct i2c_client *client);
int ov7690_176_144(struct i2c_client *client);

int ov7690_lens_corrention_set(struct i2c_client *client);
int ov7690_color_matrix_set(struct i2c_client *client);
int ov7690_edge_donoise_set(struct i2c_client *client);
int ov7690_uvajust_set(struct i2c_client *client);
int ov7690_aec_agc_target_set(struct i2c_client *client);
int ov7690_gama_set(struct i2c_client *client);
int ov7690_general_control_set(struct i2c_client *client);

int ov7690_set_wb_auto(struct i2c_client *client);
int ov7690_set_wb_daylight(struct i2c_client *client);
int ov7690_set_wb_cloudy(struct i2c_client *client);

int ov7690_set_effect_none(struct i2c_client *client);
int ov7690_set_effect_negative(struct i2c_client *client);
int ov7690_set_effect_green(struct i2c_client *client);
int ov7690_set_effect_wb(struct i2c_client *client);

int ov7690_set_antibanding_auto(struct i2c_client *client);

int ov7690_set_scene_auto(struct i2c_client *client);
int ov7690_set_scene_night(struct i2c_client *client);

int ov7690_set_FPS15(struct i2c_client *client);
int ov7690_set_FPS25(struct i2c_client *client);

#endif
