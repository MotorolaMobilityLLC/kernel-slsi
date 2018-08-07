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

#ifndef FIMC_IS_CIS_2X5SP_H
#define FIMC_IS_CIS_2X5SP_H

#include "fimc-is-cis.h"

#define EXT_CLK_Mhz (26)

#define SENSOR_2X5SP_MAX_WIDTH		(5760)
#define SENSOR_2X5SP_MAX_HEIGHT		(4320)

/* TODO: Check below values are valid */
#define SENSOR_2X5SP_FINE_INTEGRATION_TIME_MIN                0x0
#define SENSOR_2X5SP_FINE_INTEGRATION_TIME_MAX                0x0 /* Not used */
#define SENSOR_2X5SP_COARSE_INTEGRATION_TIME_MIN              0x4
#define SENSOR_2X5SP_COARSE_INTEGRATION_TIME_MAX_MARGIN       0x4

#define USE_GROUP_PARAM_HOLD		(0)

/* OTP valies */
#define OTP_DATA_PATH			"/data/camera/2x5_otp_cal_data.bin"
#define OTP_PAGE_CTRL			0x0A00
#define OTP_PAGE_SELECT			0x0A02
#define OTP_PAGE_BASE			0x0A04
#define OTP_PAGE_START			0
#define OTP_PAGE_END			255
#define OTP_PAGE_SIZE			64

#define OTP_AWB_LIMIT_OFFSET	28
#define OTP_AWB_GOLDEN_OFFSET	40
#define OTP_AWB_UNIT_OFFSET	54

#define OTP_GRP1_AWB_CRC_START		(254 * OTP_PAGE_SIZE)
#define OTP_GRP1_AWB_CRC_SIZE		(60)
#define OTP_GRP1_LSC_XTC_CRC_START	(163 * OTP_PAGE_SIZE + 8)
#define OTP_GRP1_LSC_XTC_CRC_SIZE	(2896)

#define OTP_GRP2_AWB_CRC_START		(255 * OTP_PAGE_SIZE)
#define OTP_GRP2_AWB_CRC_SIZE		(60)
#define OTP_GRP2_LSC_XTC_CRC_START	(208 * OTP_PAGE_SIZE + 24)
#define OTP_GRP2_LSC_XTC_CRC_SIZE	(2896)

/* [shift value setting for 2X5SP]
 * 1s: 2, 2s: 3, 4s: 4, 8s: 5, 16s: 6, 32s: 7
 */
#define GET_2X5SP_LTE_SHIFT_CNT(val)	\
	({u8 shift_count = 1, shifter;		\
	if (val >= 1000000) {			\
		shifter = val / 1000000;	\
		while (shifter) {		\
			shifter /= 2;		\
			shift_count++;		\
		}				\
	} else {				\
		shift_count = 0;		\
	}; shift_count;})

enum otp_group {
	OTP_GROUP_ONE = 0x1,
	OTP_GROUP_TWO = 0x2,
	OTP_GROUP_MAX,
};

enum valid_check {
	OTP_DATA_EMPTY = 0x00,
	OTP_DATA_VALID = 0x40,
	OTP_DATA_INVALID = 0xC0,
};

enum sensor_2x5sp_mode_enum {
	SENSOR_2X5SP_2880X2160_30FPS = 0,
	SENSOR_2X5SP_2880X2160_30FPS_3DHDR,
	SENSOR_2X5SP_5760X4320_24FPS,
	SENSOR_2X5SP_1920X1080_120FPS,
	SENSOR_2X5SP_1280X720_240FPS,
};

#endif

