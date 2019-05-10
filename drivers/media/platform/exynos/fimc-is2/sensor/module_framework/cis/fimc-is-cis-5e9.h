/*
 * Samsung Exynos5 SoC series Sensor driver
 *
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_CIS_5E9_H
#define FIMC_IS_CIS_5E9_H

#include "fimc-is-cis.h"

#define EXT_CLK_Mhz (26)

#define SENSOR_5E9_MAX_WIDTH		(2592 + 0)
#define SENSOR_5E9_MAX_HEIGHT		(1944 + 0)

/* TODO: Check below values are valid */
#define SENSOR_5E9_FINE_INTEGRATION_TIME_MIN                0x64
#define SENSOR_5E9_FINE_INTEGRATION_TIME_MAX                0x64
#define SENSOR_5E9_COARSE_INTEGRATION_TIME_MIN              0x2
#define SENSOR_5E9_COARSE_INTEGRATION_TIME_MAX_MARGIN       0x5

#define USE_GROUP_PARAM_HOLD	(1)
#if defined(CONFIG_CAMERA_OTPROM_SUPPORT_REAR)
#define USE_OTP_AWB_CAL_DATA	(1)
#else
#define USE_OTP_AWB_CAL_DATA	(0)
#endif

/* OTP valies */
#define OTP_DATA_PATH			"/data/vendor/camera/"
#define OTP_PAGE_CTRL			0x0A00
#define OTP_PAGE_ERRCHK			0x0A01
#define OTP_PAGE_SELECT			0x0A02
#define OTP_PAGE_BASE			0x0A04
#define OTP_PAGE_START			0
#define OTP_PAGE_END			63
#define OTP_PAGE_SIZE			64

#define OTP_GRP_FLAG			(17 * OTP_PAGE_SIZE)

#define OTP_GRP_ADDR_CHKSUM		(17 * OTP_PAGE_SIZE + 0x4)
#define OTP_GRP_ADDR_CRC_START		(17 * OTP_PAGE_SIZE + 0x8)
#define OTP_GRP_ADDR_CRC_SIZE		(72)
#define OTP_GRP_INFO_CHKSUM		(18 * OTP_PAGE_SIZE + 0x20)
#define OTP_GRP_INFO_CRC_START		(18 * OTP_PAGE_SIZE + 0x24)
#define OTP_GRP_INFO_CRC_SIZE		(54)
#define OTP_GRP_AWB_CHKSUM		(19 * OTP_PAGE_SIZE + 0x30)
#define OTP_GRP_AWB_CRC_START		(19 * OTP_PAGE_SIZE + 0x34)
#define OTP_GRP_AWB_CRC_SIZE		(80)
#define OTP_GRP_LSC_XTC_CHKSUM		(21 * OTP_PAGE_SIZE + 0x20)
#define OTP_GRP_LSC_XTC_CRC_START	(21 * OTP_PAGE_SIZE + 0x24)
#define OTP_GRP_LSC_XTC_CRC_SIZE	(888)
#define OTP_GRP_AE_SYNC_CHKSUM		(35 * OTP_PAGE_SIZE + 0x30)
#define OTP_GRP_AE_SYNC_CRC_START	(35 * OTP_PAGE_SIZE + 0x34)
#define OTP_GRP_AE_SYNC_CRC_SIZE	(4)

#define OTP_GRP2_OFFSET			(20)

/* AWB ratio check */
#define RG_MIN_LIMIT_OFFSET		0x28
#define RG_MAX_LIMIT_OFFSET		0x29
#define BG_MIN_LIMIT_OFFSET		0x2A
#define BG_MAX_LIMIT_OFFSET		0x2B

#define MASTER_RG_RATIO_OFFSET		0x2C
#define MASTER_BG_RATIO_OFFSET		0x2E
#define MASTER_GR_GB_RATIO_OFFSET	0x30
#define CURRENT_RG_RATIO_OFFSET		0x32
#define CURRENT_BG_RATIO_OFFSET		0x34
#define CURRENT_GR_GB_RATIO_OFFSET	0x36

#define OTP_SERIAL_PAGE_START		19
#define OTP_SERIAL_PAGE_END		19
#define OTP_SERIAL_NUMBER_SIZE		16

#define OTP_SERIAL_PAGE_MAX		2
#define OTP_SERIAL_INDEX_START		9
#define OTP_SERIAL_INDEX_END		24

enum otp_group {
	OTP_GROUP_ONE = 0x1,
	OTP_GROUP_TWO = 0x2,
	OTP_GROUP_MAX,
};

enum valid_check {
	OTP_DATA_EMPTY = 0x0,
	OTP_DATA_VALID = 0x40,
	OTP_DATA_INVALID = 0xC0,
};

#endif

