/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>

enum vipx_time_measure_point {
	VIPX_TMP_QUEUE,
	VIPX_TMP_REQUEST,
	VIPX_TMP_RESOURCE,
	VIPX_TMP_PROCESS,
	VIPX_TMP_DONE,
	VIPX_TMP_COUNT
};

struct vipx_time {
	struct timeval		time;
};

void vipx_get_timestamp(struct vipx_time *time);

#define VIPX_TIME_IN_US(v)	((v).time.tv_sec * 1000000 + (v).time.tv_usec)
