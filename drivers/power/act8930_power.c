/*
 * PMU driver for act8930 PMU
 *
 * Copyright 2010 Ingenic Semiconductor LTD.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/mod_devicetable.h>
#include <linux/i2c.h>
#include <asm/jzsoc.h>
#include <linux/act8930_power.h>

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define dprintk(x...) printk(x)
#else
#define dprintk(x...)
#endif

extern void i2c_jz_setclk(struct i2c_client *client,unsigned long i2cclk);

struct act8930 *act8930;

static int act8930_reg_locked(struct act8930 *act8930, unsigned char reg)
{
	if (!act8930->locked)
		return 0;

	switch (reg) 
	{
		default:		
			return 0;
	}
}

static int act8930_i2c_read_device(struct act8930 *act8930, unsigned char reg,
				  int bytes, void *dest)
{
	struct i2c_client *i2c = act8930->control_data;
	int ret;
	u8 r = reg;

	ret = i2c_master_send(i2c, (unsigned char *)&r, 1);
	
	if (ret < 0)
		return ret;
	if (ret != 1)
		return -EIO;

	ret = i2c_master_recv(i2c, dest, bytes);
	if (ret < 0)
		return ret;
	if (ret != bytes)
		return -EIO;
	return 0;
}

/* 	Currently we allocate the write buffer on the stack; this is OK for
  * 	small writes - if we need to do large writes this will need to be
  * 	revised.
  */
static int act8930_i2c_write_device(struct act8930 *act8930, unsigned char reg,
				   int bytes, void *src)
{
	struct i2c_client *i2c = act8930->control_data;
	unsigned char msg[bytes + 1];
	int ret;

	memcpy(&msg[0], &reg, 1);
	memcpy(&msg[1], src, bytes);

	ret = i2c_master_send(i2c, msg, bytes + 1);
	if (ret < 0)
		return ret;
	if (ret < bytes + 1)
		return -EIO;

	return 0;
}
static int act8930_read(struct act8930 *act8930, unsigned char reg,
		       int bytes, void *dest)
{
	int ret, i;
	u8 *buf = dest;

	BUG_ON(bytes <= 0);

	ret = act8930->read_dev(act8930, reg, bytes, dest);
	if (ret < 0)
		return ret;

	for (i = 0; i < bytes; i++) {
		dev_vdbg(act8930->dev, "Read %04x from R%d(0x%x)\n",
			 buf[i], reg + i, reg + i);
	}

	return 0;
}

/*
  *   act8930_reg_write: Write a single ACT8930 register.
  *
  *   @act8930: 	Device to write to.
  *   @reg: 		Register to write to.
  *   @bytes:		byte counts to write
  *   @*src: 		Value to write.
  */
 
static int act8930_write(struct act8930 *act8930, unsigned char reg,
			int bytes, void *src)
{
	u8 *buf = src;
	int i;

	BUG_ON(bytes <= 0);

	for (i = 0; i < bytes; i++) {
		if (act8930_reg_locked(act8930, reg))
			return -EPERM;

		dev_vdbg(act8930->dev, "Write %04x to R%d(0x%x)\n",
			 buf[i], reg + i, reg + i);
	}

	return act8930->write_dev(act8930, reg, bytes, src);
}
 
/**
 * act8930_reg_read: Read a single ACT8930 register.
 *
 * @act8930: Device to read from.
 * @reg: Register to read.
 */
unsigned char act8930_reg_read(struct act8930 *act8930, unsigned char reg)
{
	unsigned char val;
	unsigned char ret;

	mutex_lock(&act8930->io_lock);

	ret = act8930_read(act8930, reg, 1, &val);

	mutex_unlock(&act8930->io_lock);

	if (ret < 0)
		return ret;
	else
		return val;
}
EXPORT_SYMBOL_GPL(act8930_reg_read);

unsigned char act8930_reg_write(struct act8930 *act8930, unsigned char reg,
		     unsigned char val)
{
	unsigned char ret;

	mutex_lock(&act8930->io_lock);

	ret = act8930_write(act8930, reg, 1, &val);

	mutex_unlock(&act8930->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(act8930_reg_write);

int dc_power_detect(void)
{	
	unsigned char ret;	
	ret = act8930_reg_read(act8930,  ACT8930_APCH3);
	ret &= ACT8930_APCH3_ACINSTAT_MASK;
	if(2 == ret)
		return 1;
	return 0;
}

EXPORT_SYMBOL_GPL(dc_power_detect);

int act8930_ldo_enable(int voltage_set_reg, int voltage)
{	
	mutex_lock(&act8930->power_lock);
	if(voltage_set_reg > ACT8930_LDO3_VOLTAGE_SET){
		act8930_reg_write(act8930, voltage_set_reg, voltage);
		act8930_reg_write(act8930,(voltage_set_reg + 0x01), ACT8930_POWER_ON);
		if(act8930_reg_read(act8930, (voltage_set_reg + 0x01)) != 0x81)
			return 1;
		}
	else{
		act8930_reg_write(act8930, voltage_set_reg, voltage);
		act8930_reg_write(act8930,(voltage_set_reg + 0x02), ACT8930_POWER_ON);
		if(act8930_reg_read(act8930, (voltage_set_reg + 0x02)) != 0x81 )
			return 1;
		}
	mutex_unlock(&act8930->power_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(act8930_ldo_enable);

int act8930_ldo_disable(int voltage_set_reg)
{	
	mutex_unlock(&act8930->power_lock);
	if(voltage_set_reg > ACT8930_LDO3_VOLTAGE_SET){
		act8930_reg_write(act8930,(voltage_set_reg + 0x01), ACT8930_POWER_OFF);
		if(act8930_reg_read(act8930, (voltage_set_reg +0x01)) !=0x1) 
			return 1;	
	}
	else{
		act8930_reg_write(act8930,(voltage_set_reg + 0x02), ACT8930_POWER_OFF);
		if(act8930_reg_read(act8930, (voltage_set_reg +0x02)) !=0x1) 
			return 1;	
	}
	mutex_unlock(&act8930->power_lock);
	
	return 0;
}
EXPORT_SYMBOL_GPL(act8930_ldo_disable);


static int act8930_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
    int i;
	int ret;
	struct act8930_platform_pdata_t *pdata = i2c->dev.platform_data;

	act8930 = kzalloc(sizeof(struct act8930), GFP_KERNEL);
	if (act8930 == NULL) {
		kfree(i2c);
		return -ENOMEM;
	}

	i2c_set_clientdata(i2c, act8930);
	act8930->dev = &i2c->dev;
	act8930->control_data = i2c;
	act8930->read_dev = act8930_i2c_read_device;
	act8930->write_dev = act8930_i2c_write_device;
	i2c_jz_setclk(i2c,10 * 1000);

	mutex_init(&act8930->io_lock);
	mutex_init(&act8930->key_lock);
	mutex_init(&act8930->power_lock);
	dev_set_drvdata(act8930->dev, act8930);


	for (i = 0; i < pdata->numldos; i++)
	{
		struct act8930_ldos *pldo = &pdata->ldos[i];
		dprintk("act8930_power:ldos[%d]->ldo = 0x%x\n",i,pldo->ldo);
		dprintk("act8930_power:ldos[%d]->voltage = 0x%x\n",i,pldo->voltage);
		dprintk("act8930_power:ldos[%d]->active_off = %d\n",i,pldo->active_off);
		if(pldo->active_off){
			ret = act8930_ldo_disable(pldo->ldo);
			if(ret)
				printk("act8930 disable ldo[%d] error\n",i);
			}
		if(!pldo->active_off){
			ret = act8930_ldo_enable(pldo->ldo,pldo->voltage);
			if(ret)
				printk("act8930 enable ldo[%d] error\n",i);
			}
	}
	return 0;
}

static int act8930_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id act8930_pmu_id[] = {
	{ "act8930", 0 },
	{ }
};

static struct i2c_driver act8930_pmu_driver = {
	.probe		= act8930_i2c_probe,
	.remove		= act8930_i2c_remove,
	.id_table		= act8930_pmu_id,
	.driver = {
		.name	= "act8930",
		.owner	= THIS_MODULE,
	},
};

static int __devinit act8930_pmu_init(void)
{	
	return i2c_add_driver(&act8930_pmu_driver);
}

static void __exit act8930_pmu_exit(void)
{
	i2c_del_driver(&act8930_pmu_driver);
}


subsys_initcall(act8930_pmu_init);
module_exit(act8930_pmu_exit);

MODULE_DESCRIPTION("ACT8930 PMU Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("qyang@ingenic.cn");


