#ifndef JZ47XX_BATTERY_H
#define JZ47XX_BATTERY_H

#include "gpio_interface.h"
#include "act8600_interface.h"

//#include <linux/wakelock.h>
#include <linux/power_supply.h>
//#include <linux/earlysuspend.h>

struct battery_info {
	int max_vol;
	int min_vol;
	int capacity;
	int dc_chg_max_vol;
	int dc_chg_min_vol;
	int usb_chg_max_vol;
	int usb_chg_min_vol;
	int battery_mah;
	int dc_charg_ma;
	int usb_charg_ma;
	__kernel_time_t update_time;
	int windage;
};

struct jz47xx_battery_device {
	int state;
	int enabled;
	int next_scan_time;
	int wait_stfull_timeout;
	struct power_supply ac;
	struct power_supply usb;
	struct power_supply battery;
//	struct wake_lock wake_lock;
//	struct early_suspend early_suspend;
	int suspend_ac;
	int suspend_usb;
	int suspend_state;
	__kernel_time_t resume_time;
	__kernel_time_t suspend_time;
	struct delayed_work update_work;
	struct delayed_work resume_work;
	struct battery_info info;
	int (*get_ad_online_state)(void *);
	int (*get_usb_online_state)(void *);
	int (*get_charging_state)(void *);
	int (*charging_or_discharg)(int usb, int ac, int suspend);
	int (*read_battery)(void);
	void (*enable)(void *data);
	int (*curve)[2];
	int dc_charg_time;
	int usb_charg_time;
	void *interface;
};

struct jz47xx_battery_platform_data {
	void *interface_pdata;
	struct battery_info *info;
};

typedef int (*battery_charged_callback_t)(void *);

extern void *jz47xx_battery_register_interface(void *pdata,battery_charged_callback_t cb,void *cookie);

extern int g_jz_battery_min_voltage;
#endif
