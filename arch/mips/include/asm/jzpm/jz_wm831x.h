
/*
 * This is a interface file for jz-soc & wm8310 model, this file is need . 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define WM8310_BT_WIFI_VPA           0      //LDO1
#define WM8310_CIM_VCORE             1      //LDO2
#define WM8310_GPS_VDD               2      //LDO3
#define WM8310_LCD_VDD               3      //LDO4
#define WM8310_LCD_VDDA              4      //LDO5
#define WM8310_VCC_MSCI              5      //LDO6
#define WM8310_BT_WIFI_VCORE         6      //LDO7
#define WM8310_BT_WIFI_VCC           7      //LDO8
#define WM8310_CIM_AVDD              8      //LDO9
#define WM8310_CIM_DVDD              9      //LDO10

#define __lcd_power_on()			         \
do {						         \
	while (wm8310_ldo_enable(WM8310_LCD_VDD))	 \
	printk("lcd power1: enable lcd power err.\n");  \
	while (wm8310_ldo_enable(WM8310_LCD_VDDA))	 \
	printk("lcd power2: enable lcd power err.\n");  \
} while (0)

#define __lcd_power_off()			         \
do {						         \
	while (wm8310_ldo_disable(WM8310_LCD_VDD))	 \
	printk("lcd power1: disable lcd power err.\n");  \
	while (wm8310_ldo_disable(WM8310_LCD_VDDA))	 \
	printk("lcd power2: disable lcd power err.\n");  \
} while (0)

//for wifi
#define __msc0_enable_power()			        \
do {	 					        \
	while (wm8310_ldo_enable(WM8310_BT_WIFI_VPA))	\
	printk("wifi power1: enable wifi power err.\n");\
	while (wm8310_ldo_enable(WM8310_BT_WIFI_VCC))	\
	printk("wifi power2: enable wifi power err.\n");\
	while (wm8310_ldo_enable(WM8310_BT_WIFI_VCORE))	\
	printk("wifi power3: enable wifi power err.\n");\
} while (0)

//for wifi
#define __msc0_disable_power()			         \
do {						         \
	while (wm8310_ldo_disable(WM8310_BT_WIFI_VCORE))	 \
	printk("wifi power1: disable wifi power err.\n");\
	while (wm8310_ldo_disable(WM8310_BT_WIFI_VCC)) \
	printk("wifi power2: disable wifi power err.\n");\
	while (wm8310_ldo_disable(WM8310_BT_WIFI_VPA))	 \
	printk("wifi power3: disable wifi power err.\n");\
} while (0)

#define __camera_power_on()			            \
do {						            \
	printk("----------------------camera_power_on\n");	\
	while (wm8310_ldo_enable(WM8310_CIM_DVDD))	    \
	printk("camera power2: enable camera power err.\n");\
	while (wm8310_ldo_enable(WM8310_CIM_VCORE))	    \
	printk("camera power3: enable camera power err.\n");\
	while (wm8310_ldo_enable(WM8310_CIM_AVDD))	    \
	printk("camera power1: enable camera power err.\n");\
} while (0)

/*
#define __camera_power_on()			            \
do {						            \
	while (wm8310_ldo_enable(WM8310_CIM_AVDD))	    \
	printk("camera power1: enable camera power err.\n");\
	msleep(1);					    \
	while (wm8310_ldo_enable(WM8310_CIM_DVDD))	    \
	printk("camera power2: enable camera power err.\n");\
	msleep(1);					    \
	while (wm8310_ldo_enable(WM8310_CIM_VCORE))	    \
	printk("camera power3: enable camera power err.\n");\
} while (0)
*/
#define __camera_power_off()			             \
do {						             \
	printk("----------------------camera_power_off\n");  \
	while (wm8310_ldo_disable(WM8310_CIM_AVDD))	     \
	printk("camera power1: disable camera power err.\n");\
	while (wm8310_ldo_disable(WM8310_CIM_DVDD))	     \
	printk("camera power2: disable camera power err.\n");\
	while (wm8310_ldo_disable(WM8310_CIM_VCORE))	     \
	printk("camera power3: disable camera power err.\n");\
} while (0)

#define __compass_power_on() __gps_power_on()
#define __gps_power_on()                                     \
do {  		                                             \
	while (wm8310_ldo_enable(WM8310_GPS_VDD))	     \
	printk("gps power1: enable gps power err.\n");       \
} while (0)

#define __compass_power_off() __gps_power_off()
#define __gps_power_off()                                    \
do {  		                                             \
	while (wm8310_ldo_disable(WM8310_GPS_VDD))	     \
	printk("gps power1: disable gps power err.\n");      \
} while (0)

#define __msc1_enable_power()			            \
do {  		                                            \
	while (wm8310_ldo_enable(WM8310_VCC_MSCI))	    \
	printk("msc1 power1: enable msc1 power err.\n");    \
} while (0)

#define __msc1_disable_power()			           \
do {  		                                           \
	while (wm8310_ldo_disable(WM8310_VCC_MSCI))	                   \
	printk("msc1 power1: disable msc1 power err.\n");  \
} while (0)
