/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "vertex-log.h"
#include "vertex-message.h"
#include "vertex-slab.h"

int vertex_slab_alloc(struct vertex_slab_allocator *allocator, void **target,
		size_t size)
{
	int ret;

	vertex_enter();
	if (size <= allocator->cache_size) {
		ret = -EINVAL;
		vertex_err("size is too big tahn slab size (%zu, %zu)\n",
				size, allocator->cache_size);
		goto p_err;
	}

	*target = kmem_cache_alloc(allocator->cache, GFP_KERNEL);
	if (!(*target)) {
		ret = -ENOMEM;
		vertex_err("kmem_cache_alloc is fail\n");
		goto p_err;
	}

	vertex_leave();
	return 0;
p_err:
	return ret;
}

void vertex_slab_free(struct vertex_slab_allocator *allocator, void *target)
{
	vertex_enter();
	kmem_cache_free(allocator->cache, target);
	vertex_leave();
}

int vertex_slab_init(struct vertex_slab_allocator *allocator)
{
	int ret;
	char name[100];
	size_t size;

	vertex_enter();
	size = sizeof(struct vertex_message);
	allocator->cache_size = size;

	snprintf(name, sizeof(name), "vertex-slab-%zu", size);

	allocator->cache = kmem_cache_create(name, size,
			ARCH_KMALLOC_MINALIGN, SLAB_POISON | SLAB_PANIC, NULL);
	if (!allocator->cache) {
		ret = -ENOMEM;
		vertex_err("kmem_cache_create(%zu) is fail\n", size);
		goto p_err;
	}

	vertex_leave();
	return 0;
p_err:
	return ret;
}

void vertex_slab_deinit(struct vertex_slab_allocator *allocator)
{
	kmem_cache_destroy(allocator->cache);
}
