/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef VIPX_DEVICE_H_
#define VIPX_DEVICE_H_

#include "vipx-graphmgr.h"
#include "vipx-system.h"
#include "vipx-vertex.h"
#include "vipx-debug.h"

enum vipx_device_state {
	VIPX_DEVICE_STATE_OPEN,
	VIPX_DEVICE_STATE_START
};

enum vipx_device_mode {
	VIPX_DEVICE_MODE_NORMAL,
	VIPX_DEVICE_MODE_TEST
};

struct vipx_device {
	struct device			*dev;
	unsigned long			state;
	u32				mode;

	struct vipx_graphmgr		graphmgr;
	struct vipx_system		system;
	struct vipx_vertex		vertex;
	struct vipx_debug		debug;
};

int vipx_device_open(struct vipx_device *device);
int vipx_device_close(struct vipx_device *device);
int vipx_device_start(struct vipx_device *device);
int vipx_device_stop(struct vipx_device *device);

#endif
