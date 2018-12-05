/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __VIPX_TIME_H__
#define __VIPX_TIME_H__

#include <linux/time.h>

enum vipx_time_measure_point {
	VIPX_TIME_QUEUE,
	VIPX_TIME_REQUEST,
	VIPX_TIME_RESOURCE,
	VIPX_TIME_PROCESS,
	VIPX_TIME_DONE,
	VIPX_TIME_COUNT
};

struct vipx_time {
	struct timeval		time;
};

void vipx_get_timestamp(struct vipx_time *time);

#define VIPX_TIME_IN_US(v)	((v).time.tv_sec * 1000000 + (v).time.tv_usec)

#endif
