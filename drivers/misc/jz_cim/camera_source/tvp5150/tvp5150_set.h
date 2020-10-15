#ifndef tvp5150_SET_H
#define tvp5150_SET_H

#include<linux/i2c.h>

#define TVP5150_CTRL_BRIGHTNESS	0x0
#define TVP5150_CTRL_CONTRAST	0x1
#define TVP5150_CTRL_SATURATION	0x2
#define TVP5150_CTRL_HUE	0x3

/* supported controls */
struct tvp5150_ctrl {
	__u32		     id;
	__s32		     minimum;	/* Note signedness */
	__s32		     maximum;
};

struct tvp5150_control {
	__u32		     id;
	__s32		     value;
};

void preview_set(struct i2c_client *client);
void capture_set(struct i2c_client *client);
void size_switch(struct i2c_client *client,int width,int height,int setmode);

int tvp5150_probe(struct i2c_client *c);
int tvp5150_remove(struct i2c_client *c);
int tvp5150_sensor_reset(struct i2c_client *c);

int tvp5150_read(struct i2c_client *c, unsigned char addr);
void tvp5150_write(struct i2c_client *c, unsigned char addr,
		   unsigned char value);

#endif

