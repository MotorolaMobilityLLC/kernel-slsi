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

#ifndef FIMC_IS_CIS_16885C_H
#define FIMC_IS_CIS_16885C_H

#include "fimc-is-cis.h"

#define EXT_CLK_Mhz (26)

#define SENSOR_16885C_MAX_WIDTH		(4096)
#define SENSOR_16885C_MAX_HEIGHT		(3072)

/* TODO: Check below values are valid */
#define SENSOR_16885C_FINE_INTEGRATION_TIME_MIN                0x0
#define SENSOR_16885C_FINE_INTEGRATION_TIME_MAX                0x0 /* Not used */
#define SENSOR_16885C_COARSE_INTEGRATION_TIME_MIN              0x8 /* TODO */
#define SENSOR_16885C_COARSE_INTEGRATION_TIME_MAX_MARGIN       0xC /* TODO */

#define USE_GROUP_PARAM_HOLD	(0)

#endif

