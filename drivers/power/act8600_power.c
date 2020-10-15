/*
 * PMU driver for act8600 PMU
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
#include <linux/act8600_power.h>

#define DEBUG
#ifdef DEBUG
#define dprintk(x...) printk(x)
#else
#define dprintk(x...)
#endif

//static int core = 0;
//module_param(core, int, 0644);

extern void i2c_jz_setclk(struct i2c_client *client,unsigned long i2cclk);


struct act8600_device {
	struct i2c_client *client;
};

struct act8600_device *act8600;

int act8600_write_reg(char reg,char val)
{
	char msg[2];  

	memcpy(&msg[0], &reg, 1);      
	memcpy(&msg[1], &val, 1);      

	return i2c_master_send(act8600->client, msg, 2);
}

int act8600_read_reg(char reg,char *val)
{
	i2c_master_send(act8600->client,&reg,1);
	return i2c_master_recv(act8600->client, val, 1);
}

int act8600_output_set(int outnum,int regvalue)
{
	char reg;
	switch(outnum) {
		case ACT8600_OUT1:reg = ACT8600_REG1_VSET;break;
		case ACT8600_OUT2:reg = ACT8600_REG2_VSET;break;
		case ACT8600_OUT3:reg = ACT8600_REG3_VSET;break;
		case ACT8600_OUT4:reg = ACT8600_REG4_VSET;break;
		case ACT8600_OUT5:reg = ACT8600_REG5_VSET;break;
		case ACT8600_OUT6:reg = ACT8600_REG6_VSET;break;
		case ACT8600_OUT7:reg = ACT8600_REG7_VSET;break;
		case ACT8600_OUT8:reg = ACT8600_REG8_VSET;break;
		default:return -1;
	}
	
	return act8600_write_reg(reg,regvalue);
}

int act8600_output_enable(int outnum,int enable)
{
	char reg,value;
	switch(outnum) {
		case ACT8600_OUT1:reg = ACT8600_REG1_VCON;break;
		case ACT8600_OUT2:reg = ACT8600_REG2_VCON;break;
		case ACT8600_OUT3:reg = ACT8600_REG3_VCON;break;
		case ACT8600_OUT4:reg = ACT8600_REG4_VCON;break;
		case ACT8600_OUT5:reg = ACT8600_REG5_VCON;break;
		case ACT8600_OUT6:reg = ACT8600_REG6_VCON;break;
		case ACT8600_OUT7:reg = ACT8600_REG7_VCON;break;
		case ACT8600_OUT8:reg = ACT8600_REG8_VCON;break;
		default:return -1;
	}

	act8600_read_reg(reg,&value);
	value = (~(0x1 << 7) & value) | ((enable & 0x1)<<7);
	return act8600_write_reg(reg,value);
}
EXPORT_SYMBOL_GPL(act8600_output_enable);

static int act8600_probe(struct i2c_client *i2c,const struct i2c_device_id *id)
{
    	int i;
	struct act8600_platform_pdata_t *pdata = i2c->dev.platform_data;

	act8600 = kzalloc(sizeof(struct act8600_device), GFP_KERNEL);
	if (act8600 == NULL) {
		return -ENOMEM;
	}

	act8600->client = i2c;
	i2c_set_clientdata(i2c, act8600);
	i2c_jz_setclk(i2c,100000);


	dprintk("act8600_power:\n");
#if 0
	if (core) {
		act8600_output_set(1, core);
		act8600_output_enable(1, 1);
		dprintk("%d\t\t%d\t\t%d\n", 1, core, 1);
	}
#endif
	for (i = 0; i < pdata->nr_outputs; i++) {
		struct act8600_outputs_t *p = &pdata->outputs[i];
		act8600_output_set(p->no,p->value);
		act8600_output_enable(p->no,p->active_on);

		dprintk("%d\t\t%d\t\t%d\n",p->no,p->value,p->active_on);
	}
#if 1
	act8600_write_reg(0x91, 0xd0);	
	act8600_write_reg(0x90, 0x17);	
	act8600_write_reg(0x91, 0xc0);	
#endif
	return 0;
}

static int act8600_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id act8600_id[] = {
	{ ACT8600_NAME, 0 },
	{ }
};

static struct i2c_driver act8600_pmu_driver = {
	.probe		= act8600_probe,
	.remove		= act8600_remove,
	.id_table	= act8600_id,
	.driver = {
		.name	= ACT8600_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __devinit act8600_pmu_init(void)
{	
	return i2c_add_driver(&act8600_pmu_driver);
}

static void __exit act8600_pmu_exit(void)
{
	i2c_del_driver(&act8600_pmu_driver);
}

arch_initcall(act8600_pmu_init);
module_exit(act8600_pmu_exit);

MODULE_DESCRIPTION("ACT8600 PMU Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("ztyan@ingenic.cn");

