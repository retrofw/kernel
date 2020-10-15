
/*
 * This is a interface file for jz soc ,if you ues jz soc ,this file is need . 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#if defined(CONFIG_JZ4760_LYNX) || defined(CONFIG_JZ4760B_LYNX)
#define PMU_HDMI_1_8              0x54
#define PMU_HDMI_3_3   			  0x50
#else
#define PMU_MSC0    			       0x50
#define PMU_LCD                        0x54
#endif
#define PMU_WIFI                       0x60
#define PMU_COMPASS                    0x64

#define PMU_V3_3		0x39
#define PMU_V1_8		0x24

extern int act8930_ldo_disable(int voltage_set_reg);
extern int act8930_ldo_enable(int voltage_set_reg, int voltage);
extern int dc_power_detect(void);
	
#if defined(CONFIG_JZ4760_LYNX) || defined(CONFIG_JZ4760B_LYNX)
//-------------------------------------------------------------------
//for hdmi 
#define __hdmi_power_on() 						         \
	do {											\
		while(act8930_ldo_enable(PMU_HDMI_3_3,PMU_V3_3))		\
			printk("act8930 : enable lcd power err.\n");		\
		while(act8930_ldo_enable(PMU_HDMI_1_8,PMU_V1_8))		\
			printk("act8930 : enable lcd power err.\n");		\
	} while (0)

#define __hdmi_power_off() 						\
	do {					        		                   \
		while(act8930_ldo_disable(PMU_HDMI_1_8))		\
			printk("act8930 : disable lcd power err.\n");	\
		while(act8930_ldo_disable(PMU_HDMI_3_3))		\
			printk("act8930 : disable lcd power err.\n");	\
	} while (0)
//------------------------------------------------------------------
//for tf card
#define __msc0_power_on()			\
	do {					        \
		__gpio_clear_pin(GPIO_SD1_VCC_EN_N);	\
	} while (0)

#define __msc0_power_off()			\
	do {					        \
		__gpio_set_pin(GPIO_SD1_VCC_EN_N);	\
	} while (0)
//-------------------------------------------------------------------
//for lcd 
#define __lcd_power_on() 						         \
	do {					        \
		__gpio_set_pin(GPIO_LCD_VCC_EN);	\
	} while (0)

#define __lcd_power_off() 						\
	do {					        \
		__gpio_clear_pin(GPIO_LCD_VCC_EN);	\
	} while (0)
//------------------------------------------------------------------
#else
//-------------------------------------------------------------------
//for lcd 
#define __lcd_power_on() 						         \
	do {											\
		while(act8930_ldo_enable(PMU_LCD,PMU_V3_3))		\
			printk("act8930 : enable lcd power err.\n");		\
	} while (0)

#define __lcd_power_off() 						\
	do {					        		                   \
		while(act8930_ldo_disable(PMU_LCD))		\
			printk("act8930 : disable lcd power err.\n");	\
	} while (0)
//------------------------------------------------------------------
//for tf card
#define __msc0_power_on()			\
	do {					        \
		while(act8930_ldo_enable(PMU_MSC0, PMU_V3_3))			\
			printk("act8930 : disable msc0 power err.\n");		\
	} while (0)

#define __msc0_power_off()			\
	do {						\
		while(act8930_ldo_disable(PMU_MSC0))			\
			printk("act8930 : disable msc0 power err.\n");		\
	} while (0)
//-------------------------------------------------------------------
#endif
//for msc0
#define __msc0_enable_power()			\
	do {					        \
		__gpio_clear_pin(GPIO_SD0_VCC_EN_N);	\
	} while (0)

#define __msc0_disable_power()			\
	do {						\
		__gpio_set_pin(GPIO_SD0_VCC_EN_N);	\
	} while (0)
//-------------------------------------------------------------------
//for compass sensor
#define __compass_power_on() 						\
	do {					        		\
		while(act8930_ldo_enable(PMU_COMPASS, PMU_V3_3))		\
			printk("act8930 : enable compass power err.\n");		\
	} while (0)

#define __compass_power_off() 						\
	do {					        		\
		while(act8930_ldo_disable(PMU_COMPASS))		\
			printk("act8930 : disable compass power err.\n");	\
	} while (0)


//-------------------------------------------------------------------
//for wifi
#define __wifi_power_on() 						\
	do {				        		\
		while(act8930_ldo_enable(PMU_WIFI, PMU_V3_3))			\
			printk("act8930 : enable wifi power err.\n");		\
	} while (0)

#define __wifi_power_off() 						\
	do {				        		\
		while(act8930_ldo_disable(PMU_WIFI))			\
			printk("act8930 : disable wifi power err.\n");		\
	} while (0)
//-------------------------------------------------------------------

//four ldo power off
#define __pmu_power_off()			\
	do {						\
		while(act8930_ldo_disable( 0x50))			\
			printk("act8930 : disable lcd power err.\n");		\
		while(act8930_ldo_disable( 0x60))			\
			printk("act8930 : disable wifii power err.\n");		\
		while(act8930_ldo_disable( 0x64))			\
			printk("act8930 : disable compass power err.\n");		\
		while(act8930_ldo_disable( 0x54))			\
			printk("act8930 : disable msc0 power err.\n");		\
	} while (0)
//-------------------------------------------------------------------

