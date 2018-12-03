/*
 * Samsung Exynos5 SoC series Sensor driver
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_EEPROM_H
#define FIMC_IS_EEPROM_H

void fimc_is_eeprom_cal_data_set(char *data, char *name,
		u32 addr, u32 size, u32 value);
int fimc_is_eeprom_file_write(const char *file_name, const void *data,
		unsigned long size);
int fimc_is_eeprom_file_read(const char *file_name, const void *data,
		unsigned long size);
int fimc_is_eeprom_module_read(struct i2c_client *client, u32 addr,
		char *data, unsigned long size);
int fimc_is_sensor_eeprom_check_crc(char *data, size_t size);
int fimc_is_sensor_eeprom_check_awb_ratio(char *unit, char *golden, char *limit);

#endif

