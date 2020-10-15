
#ifndef CM3511_SET_H
#define CM3511_SET_H


void cm3511_set_preview_mode(struct i2c_client *client);
void cm3511_set_capture_mode(struct i2c_client *client);

void cm3511_init_setting(struct i2c_client *client);
void cm3511_set_nightmode(struct i2c_client *client,int enable);

void cm3511_set_ab_50hz(struct i2c_client *client);
void cm3511_set_ab_60hz(struct i2c_client *client);

void cm3511_set_wb_auto(struct i2c_client *client);
void cm3511_set_wb_cloud(struct i2c_client *client);
void cm3511_set_wb_daylight(struct i2c_client *client);
void cm3511_set_wb_incandescence(struct i2c_client *client);
void cm3511_set_wb_fluorescent(struct i2c_client *client);
void cm3511_set_wb_tungsten(struct i2c_client *client);

void cm3511_set_effect_normal(struct i2c_client *client);
void cm3511_set_effect_grayscale(struct i2c_client *client);
void cm3511_set_effect_sepia(struct i2c_client *client);
void cm3511_set_effect_colorinv(struct i2c_client *client);
void cm3511_set_effect_sepiagreen(struct i2c_client *client);
void cm3511_set_effect_sepiablue(struct i2c_client *client);
void cm3511_set_effect_grayinv(struct i2c_client *client);
void cm3511_set_effect_embossment(struct i2c_client *client);
void cm3511_set_effect_sketch(struct i2c_client *client);

#endif
