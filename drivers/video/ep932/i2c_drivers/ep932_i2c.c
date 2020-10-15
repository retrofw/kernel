#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/kthread.h>

#include <asm/jzsoc.h>

extern void hdmi_test(void);
extern void hdmi_start();
extern void hdmi_stop();


struct task_struct *hdmi_kthread;
static struct i2c_client *this_client;

#define HDMI_INIT		1
#define HDMI_EXIT		2
#define HDMI_POWER_ON		3
#define HDMI_POWER_OFF		4



#define EP932_RESET_PIN		(32*4 + 11)
#define EP932_SPEED		100000
extern void i2c_jz_setclk(struct i2c_client *client,unsigned long i2cclk);

static int jz_ep932_write(unsigned char *txData, int length)
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



static int jz_ep932_read(unsigned char *rxData, int length)
{
        int retry;

        struct i2c_msg msgs[] = {
                {
                        .addr = this_client->addr,
                        .flags = I2C_M_NOSTART,
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

int jz_i2c_ep932_write(unsigned char adress, unsigned char *txData, int length)
{
	int i;
	
	for (i = length; i >= 1; i--) {
		txData[i] = txData[i-1];
	}

	txData[0] = adress;
	return jz_ep932_write(txData, length+1);
}

int jz_i2c_ep932_read(unsigned char adress, unsigned char *txData, int length)
{
	txData[0] = adress;
	return jz_ep932_read(txData, length);
}

/************************************************************************/

static int hdmi_thread(void *data)
{
	printk("called hdmi_test.\n");
	hdmi_test();

	return 0;
}

static int ep932_open(struct inode *inode, struct file *file)
{
        return 0;
}

static int ep932_release(struct inode *inode, struct file *file)
{
        return 0;
}

static int ep932_ioctl(struct inode *inode, struct file *file,
                         unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int hdmi_type;
	switch (cmd) {
	case HDMI_INIT:
		hdmi_kthread = kthread_run(hdmi_thread, NULL, "hdmi_run");
		if (IS_ERR(hdmi_kthread)) {
			printk(": Failed to create HDMI monitor thread.\n");
			PTR_ERR(hdmi_kthread);
		}
		break;
	case HDMI_EXIT:
		if(hdmi_kthread)
			kthread_stop(hdmi_kthread);

		break;
	case HDMI_POWER_ON:
		if (copy_from_user(&hdmi_type, argp, sizeof(int)))
			return -EFAULT;
		printk(": HDMI type:%d\n",hdmi_type);
		hdmi_start(hdmi_type);
		break;
	case HDMI_POWER_OFF:
		printk(": HDMI stop:\n");
		hdmi_stop();
	break;
	}
	
	return 0;

}

static struct file_operations ep932_fops = {
        .owner  = THIS_MODULE,
        .open   = ep932_open,
        .ioctl  = ep932_ioctl,
        .release = ep932_release,
};

static struct miscdevice ep932_device = {
        .minor  = MISC_DYNAMIC_MINOR,
        .name   = "hdmi_ep932",
        .fops   = &ep932_fops,
};


static int jz_ep932_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		return err;
	}
	
	this_client = client;
	i2c_jz_setclk(client,EP932_SPEED);

	err = misc_register(&ep932_device);
        if (err) {
                dev_err(&client->dev,
                       "%s: ep932 device register failed\n", __func__);
		return err;
        }
#if 0
		hdmi_start(6);
#if 0
		hdmi_kthread = kthread_run(hdmi_thread, NULL, "hdmi_run");
		if (IS_ERR(hdmi_kthread)) {
			printk(": Failed to create HDMI monitor thread.\n");
			PTR_ERR(hdmi_kthread);
		}
#endif
#endif
	return 0;
}


static int jz_ep932_remove(struct i2c_client *client)
{
	return 0;
}
static const struct i2c_device_id jz_ep932_id[] = {
        { "jz_ep932", 0 },
        { }     /* Terminating entry */
};
MODULE_DEVICE_TABLE(i2c, jz_ep932_id);


static struct i2c_driver jz_ep932_driver = {
        .probe          = jz_ep932_probe,
        .remove         = jz_ep932_remove,
        .id_table       = jz_ep932_id,
        .driver = {
                .name   = "jz_ep932",
        },
};
static int __init jz_ep932_init(void)
{
        return i2c_add_driver(&jz_ep932_driver);
}

static void __exit jz_ep932_exit(void)
{
	kthread_stop(hdmi_kthread);
        i2c_del_driver(&jz_ep932_driver);
        printk(KERN_INFO "JZ_EP932 driver: exit\n");
}
late_initcall(jz_ep932_init);
//module_init(jz_ep932_init);
module_exit(jz_ep932_exit);

EXPORT_SYMBOL(jz_i2c_ep932_read);
EXPORT_SYMBOL(jz_i2c_ep932_write);

MODULE_LICENSE("GPL");
