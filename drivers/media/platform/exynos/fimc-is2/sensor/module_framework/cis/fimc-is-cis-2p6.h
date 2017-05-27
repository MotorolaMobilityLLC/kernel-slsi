/*
 * Samsung Exynos5 SoC series Sensor driver
 *
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_CIS_2P6_H
#define FIMC_IS_CIS_2P6_H

#include "fimc-is-cis.h"

#define EXT_CLK_Mhz (26)

#define SENSOR_2P6_MAX_WIDTH		(4608 + 16)
#define SENSOR_2P6_MAX_HEIGHT		(3456 + 12)

/* TODO: Check below values are valid */
#define SENSOR_2P6_FINE_INTEGRATION_TIME_MIN                0x0618
#define SENSOR_2P6_FINE_INTEGRATION_TIME_MAX                0x0618
#define SENSOR_2P6_COARSE_INTEGRATION_TIME_MIN              0x07
#define SENSOR_2P6_COARSE_INTEGRATION_TIME_MAX_MARGIN       0x08

#define USE_GROUP_PARAM_HOLD	(0)

#define S5K2P6_USE_COMPACT_PLL_INFO

#endif

