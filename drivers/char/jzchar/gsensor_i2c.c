
/* Medive add for gsensor i2c driver  */

#include <asm/jzsoc.h>
#include <linux/i2c.h>

struct i2c_client *gsensor_client;

extern void i2c_jz_setclk(struct i2c_client *client,unsigned long i2cclk);


int gsensor_i2c_rxdata (char *rxdata,int len)
{
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr = gsensor_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxdata,
		},
		{
			.addr = gsensor_client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = rxdata,
		},
	};
	ret = i2c_transfer(gsensor_client->adapter,msgs,2);
	if (ret < 0){
		printk(" \n Medive printk: gsensor read i2c failed!\n");
	}
	return ret;
		      
}

int gsensor_i2c_txdata(char *txdata,int len)
{
	int ret;
	struct i2c_msg msg[] = {
		{
			.addr = gsensor_client->addr,
			.flags = 0,
			.len = len,
			.buf = txdata,
		},
	};
	ret = 	i2c_transfer(gsensor_client->adapter,msg,1);
	if (ret < 0){
		printk(" \n Medive printk: gsensor write i2c failed!\n");
	}
}


static void sensor_set_i2c_speed(struct i2c_client *client,unsigned long speed)
{
	i2c_jz_setclk(client,speed);
}

static int gsensor_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	gsensor_client = client;
	sensor_set_i2c_speed(client,10000);
	return 0;
}

static const struct i2c_device_id gsensor_i2c_id[] = {
	{"gsensor_i2c",0},
	{ }
};

MODULE_DEVICE_TABLE(i2c,gsensor_i2c_id);

static struct i2c_driver i2c_gsensor_driver = {
	.probe		= gsensor_i2c_probe,
	.id_table 	= gsensor_i2c_id,
	.driver = {
		.name = "gsensor_i2c",
	},
};
static int __init gsensor_i2c_driver(void)
{
	return i2c_add_driver(&i2c_gsensor_driver);
}

module_init(gsensor_i2c_driver);

