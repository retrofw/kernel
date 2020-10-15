/*
 * linux/drivers/misc/camera_source/cm3511/camera.h -- Ingenic CIM driver
 *
 * Copyright (C) 2005-2010, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef CM3511_CAMERA_H
#define CM3511_CAMERA_H


#include "../../jz_cim_core.h"
#include "../../jz_sensor.h"


struct cm3511_sensor
{
	struct i2c_client      		 *client;
	struct camera_sensor_desc	 desc;
};

#endif
