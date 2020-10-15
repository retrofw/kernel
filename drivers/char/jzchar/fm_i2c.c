
/* Medive add for fm i2c driver  */

#include <asm/jzsoc.h>
#include <linux/i2c.h>

struct i2c_client *fm5807_client;

extern void i2c_jz_setclk(struct i2c_client *client,unsigned long i2cclk);


int fm5807_i2c_rxdata (char *rxdata,int len)
{
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr = fm5807_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxdata,
		},
		{
			.addr = fm5807_client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = rxdata,
		},
	};
	ret = i2c_transfer(fm5807_client->adapter,msgs,2);
	if (ret < 0){
		printk(" \n Medive printk: fm5807 read i2c failed!\n");
	}
	return ret;
		      
}

int fm5807_i2c_txdata(char *txdata,int len)
{
	int ret;
	struct i2c_msg msg[] = {
		{
			.addr = fm5807_client->addr,
			.flags = 0,
			.len = len,
			.buf = txdata,
		},
	};
	ret = 	i2c_transfer(fm5807_client->adapter,msg,1);
	if (ret < 0){
		printk(" \n Medive printk: fm5807 write i2c failed!\n");
	}
}

static inline int sensor_i2c_master_send(struct i2c_client *client,const char *buf ,int count)
{
	int ret;
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (char *)buf;
	ret = i2c_transfer(adap, &msg, 1);

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	   transmitted, else error code. */
	return (ret == 1) ? count : ret;
}

static inline int sensor_i2c_master_recv(struct i2c_client *client, char *buf ,int count)
{
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;
	int ret;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = buf;
	ret = i2c_transfer(adap, &msg, 1);

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	   transmitted, else error code. */
	return (ret == 1) ? count : ret;
}

int fm_write_reg(struct i2c_client *client,unsigned char reg, unsigned char val)
{
	unsigned char msg[2];  
	int ret;

	memcpy(&msg[0], &reg, 1);      
	memcpy(&msg[1], &val, 1);      

	ret = sensor_i2c_master_send(client, msg, 2);

	if (ret < 0)
	{
		printk("RET<0\n");
		return ret;
	}
	if (ret < 2)
	{
		printk("RET<2\n");
		return -EIO;
	}

	return 0;
}

int fm_read_reg(unsigned char reg,unsigned char * retval,int len)
{
	int ret;
	//unsigned char retval;

	struct i2c_client *client = fm5807_client;

	ret = sensor_i2c_master_send(client,&reg,1);

	if (ret < 0)
		return ret;
	if (ret != 1)
		return -EIO;

	ret = sensor_i2c_master_recv(client, retval, len);
	if (ret < 0)
		return ret;
	if (ret != 1)
		return -EIO;
	return ret;
}



void fm_sensor_set_i2c_speed(struct i2c_client *client,unsigned long speed)
{
	i2c_jz_setclk(client,speed);
}

static int fm_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	fm5807_client = client;
	fm_sensor_set_i2c_speed(client,10000);  //10000
	return 0;
}

static const struct i2c_device_id fm_i2c_id[] = {
	{"fm5807_i2c",0},
	{ }
};

MODULE_DEVICE_TABLE(i2c,fm_i2c_id);

static struct i2c_driver i2c_fm_driver = {
	.probe		= fm_i2c_probe,
	.id_table 	= fm_i2c_id,
	.driver = {
		.name = "fm5807_i2c",
	},
};
static int __init fm_i2c_driver(void)
{
	return i2c_add_driver(&i2c_fm_driver);
}

module_init(fm_i2c_driver);

