/*
 * Samsung Exynos5 SoC series EEPROM driver
 *
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_EEPROM_12A10FF_H
#define FIMC_IS_EEPROM_12A10FF_H

#define EEPROM_DATA_PATH		"/data/vendor/camera/dump_12a10ff_eeprom_data.bin"

/* Total Cal data size */
#define EEPROM_DATA_SIZE		SZ_8K

/* Big endian Checksum */
/* Address */
#define EEPROM_ADD_CRC_FST		0x00
#define EEPROM_ADD_CRC_SEC		0x01
#define EEPROM_ADD_CRC_CHK_START	0x04
#define EEPROM_ADD_CRC_CHK_SIZE		0x48
#define EEPROM_ADD_CAL_SIZE		0x4C

/* Information Cal */
#define EEPROM_INFO_CRC_FST		0x60
#define EEPROM_INFO_CRC_SEC		0x61
#define EEPROM_INFO_CRC_CHK_START	0x64
#define EEPROM_INFO_CRC_CHK_SIZE	0x36
#define EEPROM_INFO_SERIAL_NUM_START	0x89
#define EEPROM_INFO_SERIAL_NUM_SIZE	0x10
#define EEPROM_INFO_CAL_SIZE		0x3A

/* AWB Cal */
#define EEPROM_AWB_CRC_FST		0xB0
#define EEPROM_AWB_CRC_SEC		0xB1
#define EEPROM_AWB_CRC_CHK_START	0xB4
#define EEPROM_AWB_CRC_CHK_SIZE		0x50
#define EEPROM_AWB_CAL_SIZE		0x54
#define EEPROM_AWB_LIMIT_OFFSET		0xD8
#define EEPROM_AWB_GOLDEN_OFFSET	0xDC
#define EEPROM_AWB_UNIT_OFFSET		0xE2

/* LSC Cal */
#define EEPROM_LSC_CRC_FST		0x120
#define EEPROM_LSC_CRC_SEC		0x121
#define EEPROM_LSC_CRC_CHK_START	0x124
#define EEPROM_LSC_CRC_CHK_SIZE		0x1374
#define EEPROM_LSC_CAL_SIZE		0x1378

/* SFR Cal */
#define EEPROM_SFR_CRC_FST		0x1500
#define EEPROM_SFR_CRC_SEC		0x1501
#define EEPROM_SFR_CRC_CHK_START	0x1504
#define EEPROM_SFR_CRC_CHK_SIZE		0x64
#define EEPROM_SFR_CAL_SIZE		0x68

#endif
