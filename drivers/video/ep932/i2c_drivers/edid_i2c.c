#include <linux/kernel.h>
#include <linux/i2c.h>
#include <asm/jzsoc.h>

static struct i2c_client *this_client;

#define EDID_SPEED		100000
extern void i2c_jz_setclk(struct i2c_client *client,unsigned long i2cclk);
static int jz_edid_write(unsigned char *txData, int length)
{
        int retry;
        struct i2c_msg msg[] = {
                {
                        .addr = this_client->addr,
                        .flags = 0,
                        .len = length,
                        .buf = txData,
                },
        };

        for (retry = 0; retry <= 100; retry++) {
                if(i2c_transfer(this_client->adapter, msg, 1) > 0)
                        break;
                else
                        mdelay(10);
        }
        if (retry > 100) {
                printk(KERN_ERR "%s: retry over 100\n", __func__);
                return -EIO;
        }

        return 0;
}



static int jz_edid_read(unsigned char *rxData, int length)
{
        int retry;

        struct i2c_msg msgs[] = {
                {
                        .addr = this_client->addr,
                        .flags = 0,
                        .len = 1,
                        .buf = rxData,
                },
                {
                        .addr = this_client->addr,
                        .flags = I2C_M_RD,
                        .len = length,
                        .buf = rxData,
                },
        };

        for (retry = 0; retry <= 100; retry++) {
                if (i2c_transfer(this_client->adapter, msgs, 2) > 0)
                        break;
                else
                        mdelay(10);
        }
        if (retry > 100) {
                printk(KERN_ERR "%s: retry over 100\n",__func__);
                return -EIO;
        }

        return 0;
}

int jz_i2c_edid_write(unsigned char adress, unsigned char *txData, int length)
{
	int i;
	
	for (i = length; i >= 1; i--) {
		txData[i] = txData[i-1];
	}

	txData[0] = adress;
	return jz_edid_write(txData, length+1);
}

int jz_i2c_edid_read(unsigned char adress, unsigned char *txData, int length)
{
	txData[0] = adress;
	return jz_edid_read(txData, length);
}
static int
jz_edid_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		return err;
	}
	
	this_client = client;
	i2c_jz_setclk(client,EDID_SPEED);
	return 0;
}


static int jz_edid_remove(struct i2c_client *client)
{
	return 0;
}
static const struct i2c_device_id jz_edid_id[] = {
        { "jz_edid", 0 },
        { }     /* Terminating entry */
};
MODULE_DEVICE_TABLE(i2c, jz_edid_id);


static struct i2c_driver jz_edid_driver = {
        .probe          = jz_edid_probe,
        .remove         = jz_edid_remove,
        .id_table       = jz_edid_id,
        .driver = {
                .name   = "jz_edid",
        },
};
static int __init jz_edid_init(void)
{
        return i2c_add_driver(&jz_edid_driver);
}

static void __exit jz_edid_exit(void)
{
        i2c_del_driver(&jz_edid_driver);
        printk(KERN_INFO "JZ_EP932 driver: exit\n");
}

module_init(jz_edid_init);
module_exit(jz_edid_exit);

EXPORT_SYMBOL(jz_i2c_edid_read);
EXPORT_SYMBOL(jz_i2c_edid_write);

MODULE_LICENSE("GPL");
