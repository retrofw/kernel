#ifndef INCLUDE_FILE_OV3640_SET_MODE_H
#define INCLUDE_FILE_OV3640_SET_MODE_H

void __ov3640_set_night_mode(struct i2c_client *client,int enable);
void ov3640_set_adjust_bright (struct i2c_client *client, unsigned  int level);


void ov3640_set_wb_office_mode(struct i2c_client *client);
void ov3640_set_wb_home_mode(struct i2c_client *client);
void ov3640_set_wb_auto_mode(struct i2c_client *client);
void ov3640_set_wb_sunny_mode(struct i2c_client *client);
void ov3640_set_wb_cloudy_mode(struct i2c_client *client);

void ov3640_set_effect_normal(struct i2c_client *client);
void ov3640_set_effect_negative(struct i2c_client *client);
void ov3640_set_effect_black_white(struct i2c_client *client);
void ov3640_set_effect_greenish(struct i2c_client *client);
void ov3640_set_effect_sepia(struct i2c_client *client);

void ov3640_set_effect_antique(struct i2c_client *client);
void ov3640_set_effect_bluish(struct i2c_client *client);
void ov3640_set_effect_reddish(struct i2c_client *client);
void ov3640_set_effect_yellowish(struct i2c_client *client);


void ov3640_ab_auto(struct i2c_client *client);
void ov3640_ab_50hz(struct i2c_client *client);
void ov3640_ab_60hz(struct i2c_client *client);
void ov3640_ab_off(struct i2c_client *client);

#endif

