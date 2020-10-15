#include "ov3640_focus.h"
#include <asm/jzsoc.h>
#include "../../jz_sensor.h"
#include "../../jz_cim_core.h"
#include<linux/delay.h>


#define REG_CMD		0x3f00
#define REG_STATE 	0x3f07

static int focus_mode = FOCUS_MODE_AUTO;

int ov3640_set_af_mode(int mode)
{
	focus_mode = mode;
	return 0;
}


int ov3640_stop_focus(struct i2c_client *client)
{
	sensor_write_reg16(client,REG_CMD,0x08);// ?? -> idle
	sensor_write_reg16(client,REG_CMD,0x02);// disable overlay
	return 0;
}


int ov3640_do_focus(struct i2c_client *client)
{
	switch(focus_mode)
	{
		case FOCUS_MODE_AUTO:
			return ov3640_do_auto_focus(client);
		case FOCUS_MODE_MACRO:
			return ov3640_do_macro_focus(client);
		case FOCUS_MODE_FIXED:
			return ov3640_do_fixed_focus(client);
		case FOCUS_MODE_INFINITY:
			return ov3640_do_infinity_focus(client);
	}
	return 0;
}


int ov3640_do_fixed_focus(struct i2c_client *client)
{
	return ov3640_stop_focus(client);
}

int ov3640_do_infinity_focus(struct i2c_client *client)
{
	return ov3640_stop_focus(client);//ov3640 default is infinity focus
}

int ov3640_do_macro_focus(struct i2c_client *client)
{
	int count = 20;
	unsigned char regSTATE = sensor_read_reg16(client,REG_STATE);
	
	if(regSTATE == S_STARTUP)
	{
		printk("S_STARTUP\n");
		return -1;
	}
	if(regSTATE == S_DRIVER_ERR)
	{
		printk("S_DRIVER_ERR\n");
		return -1;
	}

	ov3640_stop_focus(client);
	while(sensor_read_reg16(client,REG_STATE) != S_IDLE && count-- > 0)
		msleep(5);

	if(count <= 0)
	{
		printk("S_IDLE\n");
		return -1;
	}

	sensor_write_reg16(client,REG_CMD,0x01);// enable overlay
#if 1
#define CHECK_TAG() 									\
	do{										\
		int to = 10;								\
		while(sensor_read_reg16(client,REG_TAG) != 0 && to-- > 0)		\
			msleep(10);							\
		if(to <= 0)								\
		{									\
			ov3640_stop_focus(client);					\
			return -1;							\
		}									\
	}while(0)

	sensor_write_reg16(client,REG_TAG,0x04);// step to nearest
	sensor_write_reg16(client,REG_CMD,0x05);// idle -> step
	CHECK_TAG();

	sensor_write_reg16(client,REG_TAG,0x01);// step 1
	sensor_write_reg16(client,REG_CMD,0x05);// step
	CHECK_TAG();

	sensor_write_reg16(client,REG_TAG,0x01);// step 1
	sensor_write_reg16(client,REG_CMD,0x05);// step
	CHECK_TAG();

	sensor_write_reg16(client,REG_TAG,0x01);// step 1
	sensor_write_reg16(client,REG_CMD,0x05);// step
	CHECK_TAG();

	sensor_write_reg16(client,REG_TAG,0x04);
	sensor_write_reg16(client,REG_PARA0,0xff);
	sensor_write_reg16(client,REG_CMD,0x05);
	CHECK_TAG();

	sensor_write_reg16(client,REG_TAG,0x04);
	sensor_write_reg16(client,REG_PARA0,0x80);
	sensor_write_reg16(client,REG_CMD,0x05);
	CHECK_TAG();

#undef CHECK_TAG
#endif
	sensor_write_reg16(client,REG_CMD,0x02);// disable overlay
	return 0;
}

int ov3640_do_auto_focus(struct i2c_client *client)
{
	int count = 20;
	int successed = -1;
	unsigned char regSTATE = sensor_read_reg16(client,REG_STATE);
	
	if(regSTATE == S_STARTUP)
	{
		printk("S_STARTUP\n");
		return -1;
	}
	if(regSTATE == S_DRIVER_ERR)
	{
		printk("S_DRIVER_ERR\n");
		return -1;
	}

	ov3640_stop_focus(client);

	while(sensor_read_reg16(client,REG_STATE) != S_IDLE && count-- > 0)
		msleep(5);

	if(count <= 0)
	{
		printk("S_IDLE\n");
		return -1;
	}

	sensor_write_reg16(client,REG_CMD,0x01);// enable overlay
#if 1
	sensor_write_reg16(client,REG_CMD,0x03);// idle -> signal mode
#endif

	count = 12;
	while(count--)
	{
		printk("---------------count %d --- ",count);
		msleep(80);
		printk(" -00- ");
		regSTATE = sensor_read_reg16(client,REG_STATE);
		printk("%2x\n",regSTATE);
		if(regSTATE & MODE_STEP_FOCUSED)
		{
			successed = 0;
			break;
		}
	}

	sensor_write_reg16(client,REG_CMD,0x02);// disable overlay
	return successed;
}

int ov3640_af_init(struct i2c_client *client,struct sensor_af_arg *info)
{
	unsigned char *tmp_buf = info->buf;
	unsigned short reg;
	unsigned char *value = info->buf+2;

	if(info->len % 3 != 0)
		return -1;
	while(value <= info->buf+info->len)
	{
		reg = (tmp_buf[1] << 8) + tmp_buf[0];
		sensor_write_reg16(client,reg,*value);
		tmp_buf += 3;
		value += 3;
	}

	return 0;
}


