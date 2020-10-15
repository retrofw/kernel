#ifndef bf3703_SET_MODE_H
#define bf3703_SET_MODE_H

#include<linux/i2c.h>

void bf3703_night_select(struct i2c_client *client,int enable);

void bf3703_set_wb_auto_mode(struct i2c_client *client);
void bf3703_set_wb_sunny_mode(struct i2c_client *client);
void bf3703_set_wb_cloudy_mode(struct i2c_client *client);
void bf3703_set_wb_office_mode(struct i2c_client *client);
void bf3703_set_wb_home_mode(struct i2c_client *client);
void bf3703_set_wb_fluorescent_mode(struct i2c_client *client);


void bf3703_ab_auto(struct i2c_client *client);
void bf3703_ab_50hz(struct i2c_client *client);
void bf3703_ab_60hz(struct i2c_client *client);
void bf3703_ab_off(struct i2c_client *client);


void bf3703_set_effect_normal(struct i2c_client *client);
void bf3703_set_effect_sepia(struct i2c_client *client);
void bf3703_set_effect_bluish(struct i2c_client *client);
void bf3703_set_effect_greenish(struct i2c_client *client);
void bf3703_set_effect_reddish(struct i2c_client *client);
void bf3703_set_effect_yellowish(struct i2c_client *client);
void bf3703_set_effect_blackwhite(struct i2c_client *client);
void bf3703_set_effect_negative(struct i2c_client *client);

#endif

