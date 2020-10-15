/*  include/linux/jz_battery.h
 *
 *  Copyright (C) 2010 Ingenic Semiconductor, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _LINUX_JZ_BATTERY_H_
#define _LINUX_JZ_BATTERY_H_

struct jz_battery_info
{
	int voltage;
	int status;
};

struct jz_battery_platform_data
{
	int max_voltage;
	int min_voltage;
	unsigned int gpio_usb_dete;
	unsigned int gpio_bat_dete;
	unsigned int gpio_dc_dete_n;
	unsigned int gpio_charg_stat_n;
	int extra;
	int modulus;
	int windage;
	int (*table_usb)[3];
	int (*table_dc)[3];
	int (*table_misc)[3];
};

#endif /* _LINUX_JZ_BATTERY_H_ */

