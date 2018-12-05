/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/ktime.h>

#include "vipx-log.h"
#include "vipx-time.h"

void vipx_get_timestamp(struct vipx_time *time)
{
	do_gettimeofday(&time->time);
}
