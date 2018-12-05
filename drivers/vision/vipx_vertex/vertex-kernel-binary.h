/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __VERTEX_KERNEL_BINARY_H__
#define __VERTEX_KERNEL_BINARY_H__

#include "vertex-context.h"
#include "vertex-memory.h"

struct vertex_kernel_binary {
	unsigned int		global_id;
	struct vertex_buffer	buffer;
	struct list_head	list;

	struct vertex_context	*vctx;
};

int vertex_kernel_binary_add(struct vertex_context *vctx, unsigned int id,
		int fd, unsigned int size);
void vertex_kernel_binary_remove(struct vertex_kernel_binary *kbin);
void vertex_kernel_binary_all_remove(struct vertex_context *vctx);

#endif
