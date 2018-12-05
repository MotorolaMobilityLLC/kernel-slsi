/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __VIPX_DEVICE_H__
#define __VIPX_DEVICE_H__

#include <linux/device.h>

#include "vipx-system.h"
#include "vipx-core.h"
#include "vipx-debug.h"

struct vipx_device;

enum vipx_device_state {
	VIPX_DEVICE_STATE_OPEN,
	VIPX_DEVICE_STATE_START
};

struct vipx_device {
	struct device		*dev;
	unsigned long		state;

	struct vipx_system	system;
	struct vipx_core	core;
	struct vipx_debug	debug;
};

int vipx_device_open(struct vipx_device *device);
int vipx_device_close(struct vipx_device *device);
int vipx_device_start(struct vipx_device *device);
int vipx_device_stop(struct vipx_device *device);

#endif
