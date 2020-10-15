/*
 * This is a interface file for jz soc ,if you ues jz soc ,this file is need . 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define AXP192_COMPASS              	2      //LDO2
#define AXP192_HDMI			3      //LDO3

#define __lcd_power_on()    do{}while(0)
#define __lcd_power_off()   do{}while(0)

int axp192_ldo_disable(int ldo_num);

//-------------------------------------------------------------------
//for compass sensor
#define __compass_power_on() 						\
	do {					        		\
		while(axp192_ldo_enable(AXP192_COMPASS))		\
		printk("axp192 : enable compass power err.\n");		\
	} while (0)

#define __compass_power_off() 						\
	do {					        		\
		while(axp192_ldo_disable(AXP192_COMPASS))		\
		printk("axp192 : disable compass power err.\n");	\
	} while (0)


//-------------------------------------------------------------------
//for compass sensor
#define __hdmi_power_on() 						\
	do {					        		\
		while(axp192_ldo_enable(AXP192_HDMI))			\
		printk("axp192 : enable hdmi power err.\n");		\
	} while (0)

#define __hdmi_power_off() 						\
	do {					        		\
		while(axp192_ldo_disable(AXP192_HDMI))			\
		printk("axp192 : disable hdmi power err.\n");		\
	} while (0)

//-------------------------------------------------------------------
//for wifi
#define __msc0_enable_power()			\
	do {					        \
		__gpio_clear_pin(GPIO_SD0_VCC_EN_N);	\
	} while (0)


//-------------------------------------------------------------------
//for wifi
#define __msc0_disable_power()			\
	do {						\
		__gpio_set_pin(GPIO_SD0_VCC_EN_N);	\
	} while (0)


//-------------------------------------------------------------------
//when system power on ,this power is enabled.
#define __camera_power_on()			\
	do{					        \
	}while(0)
#define __camera_power_off()			\
	do{					        \
	}while(0)

#define __gps_power_on()   do{}while(0)
#define __gps_power_off()  do{}while(0)

#define __msc1_enable_power()			\
	do {\
		__gpio_clear_pin(GPIO_SD1_VCC_EN_N);	\
	} while (0)

#define __msc1_disable_power()\
	do {\
		__gpio_set_pin(GPIO_SD1_VCC_EN_N);\
	} while (0)
