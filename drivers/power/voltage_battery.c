/*
 * linux/drivers/power/jz_battery
 *
 * Battery measurement code for Ingenic JZ SOC.
 *
 * based on tosa_battery.c
 *
 * Copyright (C) 2008 Marek Vasut <marek.vasut@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/init.h>
#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/syscalls.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>

#include <linux/power_supply.h>
#include <linux/jz_battery.h>
#include <linux/jz47xx_battery.h>
#include <linux/jz_sadc.h>

#include <linux/init.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#include <asm/irq.h>
#include <asm/gpio.h>
#include <asm/jzsoc.h>

#define BAT_MINOR 231
#define BAT_VOLTAGE_FILE  "/data/battery_voltage"
#define MAX_SAMPLE_NUM 10

static u16 battery_value = 0;
static int enabled = 0;

#define EFUSE_DATA1             0xb34100e4      
#define EFUSE_DATA2             0xb34100e8      
#define EFUSE_DATA3             0xb34100ec      

#define REG_EFUSE_DATA1         REG32(EFUSE_DATA1)
#define REG_EFUSE_DATA2         REG32(EFUSE_DATA2)
#define REG_EFUSE_DATA3         REG32(EFUSE_DATA3)

int efuse_data_read(signed char *slope,signed char *cut)
{
    if(!(REG_EFUSE_DATA1||REG_EFUSE_DATA2||REG_EFUSE_DATA3)){
        return -1;                                            /*the cpu chip has not been adjusted*/
    }
	(*slope) = (REG_EFUSE_DATA2 >> 24) & 0xff;
	(*cut) = (REG_EFUSE_DATA2 >> 16) & 0xff;    
    //    printk("\n slope value = %d\n, cut value = %d\n",*slope, *cut);
    return 0;
}
unsigned int jz_adjusted_voltage( unsigned int (*func)(void) )
{
	unsigned int i, x[100];
	int y = 0, final_value = 0;
	int max, min;
	int lose = 0;
    int real_voltage = 0;
    signed char slope, cut;
    
	udelay(10000);

    func();

    y = max = min = func();
	x[0] = y;
	for (i = 1; i < 50; i++)
	{
		x[i] = func();        
		y +=x[i];
		if (max < x[i])
			max = x[i];
		if (min > x[i])
			min = x[i];
	}
	final_value = y / 50;
    for(i = 0; i < 50; i++){
		if(x[i] > final_value){
			if(((x[i] - final_value) > 4)){
				lose++;
				y -= x[i];
			}
		}
		else{
			if(((final_value - x[i]) > 4)){
				lose++;
				y -= x[i];
			}
		}
    }


    if(lose < 50) final_value = y / (50 - lose);

#ifdef JZ_VBAT_ADJ_VAL
    if((final_value + JZ_VBAT_ADJ_VAL) > 0) final_value += JZ_VBAT_ADJ_VAL;
#endif

    if(efuse_data_read(&slope,&cut)){
#if defined(CONFIG_SOC_JZ4770)
        final_value = final_value * 1200 / 4096 * 4;
#endif
#if defined(CONFIG_SOC_JZ4760) || defined(CONFIG_SOC_JZ4760B)
        final_value = final_value * 2500 / 4096 * 4;
#endif
        printk("voltage :%d\n",final_value);
        return final_value;
    }
    real_voltage = (slope*final_value*6 + cut*485 + 15*4850*3)/(4850*6) + (2916*final_value)/10000;
    printk("voltage : %d \n",real_voltage * 4);
    return real_voltage*4;
}

static inline void sadc_start_pbat(void)
{
	SETREG8(SADC_ADENA, ADENA_VBATEN);      /* Enable pbat adc */
}

unsigned int jz_read_sadc(void)
{
	u16 pbat = battery_value;
	if(enabled) {
		CLRREG8(SADC_ADCTRL, ADCTRL_VRDYM);
		sadc_start_pbat();
	}
	return pbat;
}

unsigned int jz_read_sadc_p0(void)
{
	unsigned int timeout = 0x3ffff;
	u16 pbat;
	u8 state;
    
	state = INREG8(SADC_ADENA);
	if(state & (ADENA_POWER)){              /*sadc power down*/
		CLRREG8(SADC_ADENA, ADENA_POWER);           /*sadc power on*/
		msleep(2);                  /*wait 2 ms to Enable sadc*/
	}
	SETREG8(SADC_ADCTRL, ADCTRL_VRDYM);
	SETREG8(SADC_ADENA, ADENA_VBATEN);      /* Enable pbat adc */
	//udelay(300);
	while(!(INREG8(SADC_ADSTATE) & ADSTATE_VRDY) && --timeout);

	if (!timeout)
		printk(KERN_ERR "Reading battery timeout!");

	pbat = INREG16(SADC_ADVDAT) & ADVDAT_VDATA_MASK;

	OUTREG8(SADC_ADSTATE, ADSTATE_VRDY);
	CLRREG8(SADC_ADENA, ADENA_VBATEN); /* hardware may not shut down really */

	OUTREG8(SADC_ADENA, state);       
	return pbat;

}

unsigned int jz_read_battery(void)                   /*get readable voltage value*/
{
    return jz_adjusted_voltage(jz_read_sadc);
}
unsigned int jz_read_battery_p0(void)                 /*get readable voltage value*/
{
    return jz_adjusted_voltage(jz_read_sadc_p0);
}

static irqreturn_t battery_data_ready_interrupt(int irq, void * dev_id)
{
	u16 pbat;
    static int battery_count = 0;

	OUTREG8(SADC_ADSTATE, ADSTATE_VRDY);
	battery_count++;
	pbat = INREG16(SADC_ADVDAT) & ADVDAT_VDATA_MASK;

	if(battery_count == 2){
		battery_value = pbat;
		sadc_start_pbat();
	} else if(battery_count == 3){
		battery_value += pbat;
		battery_value >>= 1;

		SETREG8(SADC_ADCTRL, ADCTRL_VRDYM);
		CLRREG8(SADC_ADENA, ADENA_VBATEN); /* hardware may not shut down really */
		battery_count = 0;
	}
	else
		sadc_start_pbat();

	return IRQ_HANDLED;
}

static int jz_bat_voltage_open(struct inode *inode,struct file *filp)
{
	return 0;
}
static ssize_t jz_bat_voltage_write(struct file *filp,const char __user *buf,size_t count,loff_t *f_pos)
{   
	int value =  0;    
	int i = 0;
	char tmp[16]="";
	int n =0;

	mm_segment_t old_fs;
	int fd;

	old_fs = get_fs();	
	set_fs(KERNEL_DS);

	fd = sys_open(BAT_VOLTAGE_FILE, O_RDWR | O_CREAT|O_TRUNC, 0666);
	if (fd < 0) {
		printk("%s: Can not open %s\n",__func__, BAT_VOLTAGE_FILE);
		return -ENOENT;
	}

	//transform char* buf to int  
	if((*buf)>='0'&&(*buf)<='9'){
		n = (*buf)+(1-'1');
		buf++;
	}

	for(;(*buf)>='0'&&(*buf)<='9';buf++){
		n=n*10+(*buf)+(1-'1');
	}
	n = n>0?n:(-1)*n;
	n %=MAX_SAMPLE_NUM;

	//read voltage and write it to file
	for( i = 0;i<n;++i){    
		value = jz_read_battery_p0();
		sprintf(tmp,"cur_voltage : %d\n",value);
		sys_write(fd,tmp,strlen(tmp));
		memset(tmp,0,sizeof(tmp));
	}
	printk("write voltage to %s\n",BAT_VOLTAGE_FILE);
    
	sys_close(fd);
	set_fs(old_fs);

	return count;

}
static struct file_operations jz_bat_voltage_fops = 
{
	open:		jz_bat_voltage_open,
	write:		jz_bat_voltage_write,
};

static struct miscdevice jz_bat_voltage_dev = {
	BAT_MINOR,
	"battery",
	&jz_bat_voltage_fops
};

static int __init jz_bat_voltage_init(void)
{
	int ret;
	int error = 0;
	ret = misc_register(&jz_bat_voltage_dev);
	if (ret < 0) {
		return ret;
	}
#ifdef CONFIG_USE_VAT_ER
	REG_SADC_ADCFG &= ~(1 << 30);
#endif
	error = sadc_request_irq(BAT_DATA_READY_IRQ, battery_data_ready_interrupt, NULL);
	if (error) {
		goto err_free_irq;
	}

	enabled = 1;
	return 0;

err_free_irq:
	misc_deregister(&jz_bat_voltage_dev);	

	return 0;
}

static void __exit jz_bat_voltage_exit(void)
{
	misc_deregister(&jz_bat_voltage_dev);	
}

module_init(jz_bat_voltage_init);
module_exit(jz_bat_voltage_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marek Vasut <marek.vasut@gmail.com>");
MODULE_DESCRIPTION("Palm T|X battery driver");
