/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "vipx-log.h"
#include "vipx-message.h"
#include "vipx-slab.h"

int vipx_slab_alloc(struct vipx_slab_allocator *allocator, void **target,
		size_t size)
{
	int ret;

	vipx_enter();
	if (size <= allocator->cache_size) {
		ret = -EINVAL;
		vipx_err("size is too big tahn slab size (%zu, %zu)\n",
				size, allocator->cache_size);
		goto p_err;
	}

	*target = kmem_cache_alloc(allocator->cache, GFP_KERNEL);
	if (!(*target)) {
		ret = -ENOMEM;
		vipx_err("kmem_cache_alloc is fail\n");
		goto p_err;
	}

	vipx_leave();
	return 0;
p_err:
	return ret;
}

void vipx_slab_free(struct vipx_slab_allocator *allocator, void *target)
{
	vipx_enter();
	kmem_cache_free(allocator->cache, target);
	vipx_leave();
}

int vipx_slab_init(struct vipx_slab_allocator *allocator)
{
#ifdef TODO_CHECK
	int ret;
	char name[100];
	size_t size;

	vipx_enter();
	size = sizeof(struct vipx_message);
	allocator->cache_size = size;

	snprintf(name, sizeof(name), "vipx-slab-%zu", size);

	allocator->cache = kmem_cache_create(name, size,
			ARCH_KMALLOC_MINALIGN, SLAB_POISON | SLAB_PANIC, NULL);
	if (!allocator->cache) {
		ret = -ENOMEM;
		vipx_err("kmem_cache_create(%zu) is fail\n", size);
		goto p_err;
	}

	vipx_leave();
	return 0;
p_err:
	return ret;
#else
	return 0;
#endif
}

void vipx_slab_deinit(struct vipx_slab_allocator *allocator)
{
	kmem_cache_destroy(allocator->cache);
}
