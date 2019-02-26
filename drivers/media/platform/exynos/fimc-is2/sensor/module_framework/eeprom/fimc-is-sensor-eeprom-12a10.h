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

#ifndef FIMC_IS_EEPROM_12A10_H
#define FIMC_IS_EEPROM_12A10_H

#define EEPROM_DATA_PATH		"/data/vendor/camera/dump_12a10_eeprom_data.bin"
#define EEPROM_DUAL_DATA_PATH		"/data/vendor/camera/dual_cal_dump.bin"
#define EEPROM_SERIAL_NUM_DATA_PATH	"/data/vendor/camera/serial_number.bin"

/* Total Cal data size */
#define EEPROM_DATA_SIZE		SZ_16K

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

/* AF Cal */
#define EEPROM_AF_CRC_FST		0x120
#define EEPROM_AF_CRC_SEC		0x121
#define EEPROM_AF_CRC_CHK_START		0x124
#define EEPROM_AF_CRC_CHK_SIZE		0x1C
#define EEPROM_AF_CAL_SIZE		0x20

/* LSC Cal */
#define EEPROM_LSC_CRC_FST		0x150
#define EEPROM_LSC_CRC_SEC		0x151
#define EEPROM_LSC_CRC_CHK_START	0x154
#define EEPROM_LSC_CRC_CHK_SIZE		0x16D0
#define EEPROM_LSC_CAL_SIZE		0x16D4

/* PDAF Cal */
#define EEPROM_PDAF_CRC_FST		0x1900
#define EEPROM_PDAF_CRC_SEC		0x1901
#define EEPROM_PDAF_CRC_CHK_START	0x1904
#define EEPROM_PDAF_CRC_CHK_SIZE	0x5A0
#define EEPROM_PDAF_CAL_SIZE		0x5A4

/* AE Sync Cal */
#define EEPROM_AE_CRC_FST		0x2000
#define EEPROM_AE_CRC_SEC		0x2001
#define EEPROM_AE_CRC_CHK_START		0x2004
#define EEPROM_AE_CRC_CHK_SIZE		0x4
#define EEPROM_AE_CAL_SIZE		0x8

/* Dual camera Cal */
#define EEPROM_DUAL_CRC_FST		0x2100
#define EEPROM_DUAL_CRC_SEC		0x2101
#define EEPROM_DUAL_CRC_CHK_START	0x2104
#define EEPROM_DUAL_CRC_CHK_SIZE	0x800
#define EEPROM_DUAL_CAL_SIZE		0x804

/* SFR Cal */
#define EEPROM_SFR_CRC_FST		0x3000
#define EEPROM_SFR_CRC_SEC		0x3001
#define EEPROM_SFR_CRC_CHK_START	0x3004
#define EEPROM_SFR_CRC_CHK_SIZE		0x1CC
#define EEPROM_SFR_CAL_SIZE		0x1D0

#endif
