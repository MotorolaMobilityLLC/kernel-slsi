/*
 * s2mu106_pmeter.h - Header of S2MU106 Powermeter Driver
 *
 * Copyright (C) 2016 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef S2MU106_PMETER_H
#define S2MU106_PMETER_H
#include <linux/mfd/samsung/s2mu106.h>
#include <linux/power/s2mu00x_battery.h>

struct s2mu106_pmeter_data {
	struct i2c_client       *i2c;
	struct device *dev;
	struct s2mu106_platform_data *s2mu106_pdata;
};

int s2mu106_powermeter_get_vchg_voltage(void);
int s2mu106_powermeter_get_vchg_current(void);

#endif /*S2MU106_PMETER_H*/
