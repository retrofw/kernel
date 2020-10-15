/*
 * drivers/power/act8930_power.h -- Core interface for ACT8930
 *
 * Copyright 2010 Ingenic Semiconductor LTD.
 *
 * Author: Dragon Lu <ghlu@ingenic.cn>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __ACT8930_POWER_H__
#define __ACT8930_POWER_H__

struct act8930 {
	struct mutex io_lock;

	struct device *dev;
	int (*read_dev)(struct act8930 *act8930, unsigned char reg,
			int bytes, void *dest);
	int (*write_dev)(struct act8930 *act8930, unsigned char reg,
			 int bytes, void *src);

	void *control_data;

	int irq;  /* Our chip IRQ */
	struct mutex irq_lock;
	struct workqueue_struct *irq_wq;
	struct work_struct irq_work;
	unsigned int irq_base;
	int irq_masks[5];
	int lowbat_dete;
	int interrupt_state;
	int num_gpio;

	struct mutex power_lock;

	
	/* The ACT8930 has a security key blocking access to certain
	 * registers.  The mutex is taken by the accessors for locking
	 * and unlocking the security key, locked is used to fail
	 * writes if the lock is held.
	 */
	struct mutex key_lock;
	unsigned int locked:1;
};

struct act8930_ldos{
	int ldo;
	int voltage;
	int active_off;
};
  
struct act8930_platform_pdata_t{
	struct act8930_ldos *ldos;
	int numldos;
};


/*for act8930 Battery Charge Management */
enum  charging_state{
	PRECONDITION_STATE = 0, 	/* CSTATE[1][0] = 00*/
	FAST_CHARGE_STATE,		/* CSTATE[1][0] = 01*/
	TOP_OFF_STATE,				/* CSTATE[1][0] = 10*/
	END_OF_CHARGE_STATE,		/* CSTATE[1][0] = 11*/
};


/*
 * Register values.
 */
#define ACT8930_LDO1_VOLTAGE_SET	0X20
#define ACT8930_LDO2_VOLTAGE_SET	0X30
#define ACT8930_LDO3_VOLTAGE_SET    0X40
#define ACT8930_LDO4_VOLTAGE_SET    0X50
#define ACT8930_LDO5_VOLTAGE_SET    0X54
#define ACT8930_LDO6_VOLTAGE_SET    0X60
#define ACT8930_LDO7_VOLTAGE_SET    0X64

#define ACT8930_LDO1_VOLTAGE_CONTROL    0X22
#define ACT8930_LDO2_VOLTAGE_CONTROL    0X32
#define ACT8930_LDO3_VOLTAGE_CONTROL    0X42
#define ACT8930_LDO4_VOLTAGE_CONTROL    0X51
#define ACT8930_LDO5_VOLTAGE_CONTROL    0X55
#define ACT8930_LDO6_VOLTAGE_CONTROL    0X61
#define ACT8930_LDO7_VOLTAGE_CONTROL    0X65
/*
  * voltage configure
  */
#define ACT8930_0_6     0x00	/*0.6v*/
#define ACT8930_0_6_5   0x02	/*0.65v*/
#define ACT8930_0_7     0x04
#define ACT8930_0_7_5   0x06
#define ACT8930_0_8     0x08
#define ACT8930_0_8_5   0x0a
#define ACT8930_0_9     0x0c
#define ACT8930_0_9_5   0x0e
#define ACT8930_1_0     0x10	/*1.0v*/
#define ACT8930_1_0_5   0x12	/*1.05v*/
#define ACT8930_1_1     0x14
#define ACT8930_1_1_5   0x16
#define ACT8930_1_2		0x18
#define ACT8930_1_2_5   0x19
#define ACT8930_1_3     0x1a
#define ACT8930_1_3_5   0x1b
#define ACT8930_1_4     0x1c
#define ACT8930_1_4_5   0x1d
#define ACT8930_1_5		0x1e
#define ACT8930_1_5_5   0x1f
#define ACT8930_1_6     0x20	
#define ACT8930_1_6_5   0x21
#define ACT8930_1_7     0x22
#define ACT8930_1_7_5   0x23
#define ACT8930_1_8		0x24
#define ACT8930_1_8_5   0x25
#define ACT8930_1_9     0x26
#define ACT8930_1_9_5   0x27
#define ACT8930_2_0		0x28	/*2.0v*/
#define ACT8930_2_0_5   0x29	/*2.05v*/
#define ACT8930_2_1     0x2a
#define ACT8930_2_1_5   0x2b
#define ACT8930_2_2		0x2c
#define ACT8930_2_2_5	0x2d
#define ACT8930_2_3		0x2e
#define ACT8930_2_3_5	0x2f
#define ACT8930_2_4     0x30	
#define ACT8930_2_5     0x31
#define ACT8930_2_6     0x32
#define ACT8930_2_7     0x33
#define ACT8930_2_8     0x34
#define ACT8930_2_9     0x35
#define ACT8930_3_0     0x36	/*3.0v*/
#define ACT8930_3_1     0x37	/*3.1v*/
#define ACT8930_3_2     0x38
#define ACT8930_3_3		0x39
#define ACT8930_3_4		0x3a
#define ACT8930_3_5		0x3b
#define ACT8930_3_6		0x3c
#define ACT8930_3_7		0x3d
#define ACT8930_3_8		0x3e
#define ACT8930_3_9     0x3f
/*
  * voltage control
  */
#define ACT8930_POWER_ON		(0x1 << 7)
#define ACT8930_POWER_OFF          0x0

#define ACT8930_APCH1	0x71
#define ACT8930_APCH2	0x78
#define ACT8930_APCH3	0x7A

/*ACIN Status. Indicates the state of the ACIN input, 
typically inorder to identify the type of input supply 
connected. Value is1 when ACIN is above the 1.2V 
precision threshold, value is0 when ACIN is below this 
threshold.*/
#define ACT8930_APCH3_ACINSTAT_MASK                             0x02 /*BIT[1]*/
#define ACT8930_CHARGE_STATE_MASK                                 0x30 /*BIT[4:5]*/
#endif  /* __ACT8930_POWER_H__ */
