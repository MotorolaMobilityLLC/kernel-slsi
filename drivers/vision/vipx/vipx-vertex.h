/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef VIPX_VERTEX_H_
#define VIPX_VERTEX_H_

#define VIPX_VERTEX_NAME		"vertex"
#define VIPX_VERTEX_MINOR		0

#include "vision-dev.h"
#include "vision-ioctl.h"
#include "vipx-queue.h"

struct vipx_vertex;

enum vipx_vertex_state {
	VIPX_VERTEX_OPEN,
	VIPX_VERTEX_GRAPH,
	VIPX_VERTEX_FORMAT,
	VIPX_VERTEX_START,
	VIPX_VERTEX_STOP
};

struct vipx_vertex_refcount {
	atomic_t			refcount;
	struct vipx_vertex		*vertex;
	int				(*first)(struct vipx_vertex *vertex);
	int				(*final)(struct vipx_vertex *vertex);
};

struct vipx_vertex {
	struct mutex			lock;
	struct vision_device		vd;
	struct vipx_vertex_refcount	open_cnt;
	struct vipx_vertex_refcount	start_cnt;
};

struct vipx_vertex_ctx {
	u32				state;
	u32				idx;
	struct mutex			lock;
	struct vipx_queue		queue;
	struct vipx_vertex		*vertex;
};

int vipx_vertex_probe(struct vipx_vertex *vertex, struct device *parent);

#endif
