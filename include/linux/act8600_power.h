/*
 * drivers/power/act8600_power.h -- Core interface for ACT8600
 *
 * Copyright 2010 Ingenic Semiconductor LTD.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __ACT8600_POWER_H__
#define __ACT8600_POWER_H__

/*
 * Register values.
 */
#define ACT8600_OUT1		1
#define ACT8600_OUT2		2
#define ACT8600_OUT3		3	
#define ACT8600_OUT4		4
#define ACT8600_OUT5		5
#define ACT8600_OUT6		6
#define ACT8600_OUT7		7
#define ACT8600_OUT8		8

#define ACT8600_OUT_ON		1
#define ACT8600_OUT_OFF		0

#define ACT8600_REG1_VSET    	0x10
#define ACT8600_REG2_VSET    	0x20
#define ACT8600_REG3_VSET    	0x30
#define ACT8600_REG4_VSET    	0x40
#define ACT8600_REG5_VSET    	0x50
#define ACT8600_REG6_VSET    	0x60
#define ACT8600_REG7_VSET    	0x70
#define ACT8600_REG8_VSET    	0x80

#define ACT8600_REG1_VCON	0x12
#define ACT8600_REG2_VCON	0x22
#define ACT8600_REG3_VCON	0x32
#define ACT8600_REG4_VCON	0x41
#define ACT8600_REG5_VCON	0x51
#define ACT8600_REG6_VCON	0x61
#define ACT8600_REG7_VCON	0x71
#define ACT8600_REG8_VCON	0x81
	#define ACT8600_REG_VCON_OK		(1 << 0)
	#define ACT8600_REG_VCON_DIS		(1 << 2)
	#define ACT8600_REG_VCON_ON		(1 << 7)

#define ACT8600_APCH_INTR0	0xa1
	#define ACT8600_APCH_INTR0_SUS		(1 << 7)
#define ACT8600_APCH_INTR1	0xa8
	#define ACT8600_APCH_INTR1_INSTAT 	(1 << 5)
	#define ACT8600_APCH_INTR1_CHGSTAT 	(1 << 4)
	#define ACT8600_APCH_INTR1_INDAT 	(1 << 1)
	#define ACT8600_APCH_INTR1_CHGDAT 	(1 << 0)
#define ACT8600_APCH_INTR2	0xa9
	#define ACT8600_APCH_INTR2_INCON 	(1 << 5)
	#define ACT8600_APCH_INTR2_CHGEOCIN 	(1 << 4)
	#define ACT8600_APCH_INTR2_INDIS 	(1 << 1)
	#define ACT8600_APCH_INTR2_CHGEOCOUT 	(1 << 0)
#define ACT8600_APCH_STAT	0xaa
	#define ACT8600_APCH_STAT_STATE_MASK	(0x30)
	#define ACT8600_APCH_STAT_STATE_PRE	(0x30)
	#define ACT8600_APCH_STAT_STATE_CHAGE	(0x20)
	#define ACT8600_APCH_STAT_STATE_EOC	(0x10)
	#define ACT8600_APCH_STAT_STATE_SUSPEND	(0x00)

#define ACT8600_OTG_CON		0xb0
	#define ACT8600_OTG_CON_Q1		(1 << 7)
	#define ACT8600_OTG_CON_Q2		(1 << 6)
	#define ACT8600_OTG_CON_Q3		(1 << 5)
	#define ACT8600_OTG_CON_DBLIMITQ3	(1 << 1)
	#define ACT8600_OTG_CON_VBUSDAT		(1 << 0)
#define ACT8600_OTG_INTR	0xb2
	#define ACT8600_OTG_INTR_INVBUSR	((1 << 7) | 0x3)
	#define ACT8600_OTG_INTR_INVBUSF	((1 << 6) | 0x3)

#define ACT8600_SYS0		0x00
#define ACT8600_SYS1		0x01

#define ACT8600_NAME		"act8600"

struct act8600_outputs_t{
	int no;
	int value;
	int active_on;
};
  
struct act8600_platform_pdata_t{
	struct act8600_outputs_t *outputs;
	int nr_outputs;
};

//act8600_output_enable(ACT8600_OUT8,ACT8600_OUT_ON);
int act8600_output_enable(int outnum,int enable);
int act8600_write_reg(char reg,char val);
int act8600_read_reg(char reg,char *val);
int act8600_set_q1(int enable);
int act8600_set_q3(int enable);
int act8600_set_double_q3(int enable);
void act8600_start_recharging(void);
	
/*
 * voltage control
 */

#endif  /* __ACT8600_POWER_H__ */
