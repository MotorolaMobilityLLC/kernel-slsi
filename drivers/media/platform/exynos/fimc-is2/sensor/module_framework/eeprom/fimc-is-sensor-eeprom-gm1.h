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

#ifndef FIMC_IS_EEPROM_GM1_H
#define FIMC_IS_EEPROM_GM1_H

#define EEPROM_DATA_PATH		"/data/camera/gm1_eeprom_data.bin"
/* Total Cal data size */
#define EEPROM_DATA_SIZE		10 * 1024

/* Information Cal */
#define EEPROM_INFO_CRC_START		0x50
#define EEPROM_INFO_CRC_END		0x51
#define EEPROM_INFO_CRC_SIZE		0x31

/* AWB Cal */
#define EEPROM_AWB_CRC_START		0x90
#define EEPROM_AWB_CRC_END		0xC5
#define EEPROM_AWB_CRC_SIZE		0x35

/* AF Cal */
#define EEPROM_AF_CRC_START		0xD0
#define EEPROM_AF_CRC_END		0xEF
#define EEPROM_AF_CRC_SIZE		0x1F

/* LSC Cal */
#define EEPROM_LSC_CRC_START		0x100
#define EEPROM_LSC_CRC_END		0x17D3
#define EEPROM_LSC_CRC_SIZE		0x16D3

/* PDAF Cal */
#define EEPROM_PDAF_CRC_START		0x1800
#define EEPROM_PDAF_CRC_END		0x1DA3
#define EEPROM_PDAF_CRC_SIZE		0x5A3

/* OIS Cal */
#define EEPROM_OIS_CRC_START		0x1DB0
#define EEPROM_OIS_CRC_END		0x1DF7
#define EEPROM_OIS_CRC_SIZE		0x47

/* AE Sync Cal */
#define EEPROM_AE_CRC_START		0x2000
#define EEPROM_AE_CRC_END		0x2013
#define EEPROM_AE_CRC_SIZE		0x13

/* Dual camera Cal */
#define EEPROM_DUAL_CRC_START		0x2100
#define EEPROM_DUAL_CRC_END		0x2903
#define EEPROM_DUAL_CRC_SIZE		0x803

#endif
