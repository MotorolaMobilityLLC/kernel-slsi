/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __VERTEX_SLAB_H__
#define __VERTEX_SLAB_H__

#include <linux/slab.h>

struct vertex_slab_allocator {
	size_t			cache_size;
	struct kmem_cache	*cache;
};

int vertex_slab_alloc(struct vertex_slab_allocator *allocator, void **target,
		size_t size);
void vertex_slab_free(struct vertex_slab_allocator *allocator, void *target);

int vertex_slab_init(struct vertex_slab_allocator *allocator);
void vertex_slab_deinit(struct vertex_slab_allocator *allocator);

#endif
