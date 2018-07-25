/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/random.h>
#include <linux/slab.h>

#include "vipx-config.h"
#include "vipx-slab.h"

int vipx_slab_init(struct vipx_slab_allocator *allocator)
{
	int ret = 0, i;
	char name[100];
	size_t size;

	allocator->cache_size[0] = sizeof(vipx_msg_t);

	for (i = 0; i < VIPX_SLAB_MAX_LEVEL; ++i) {
		size = allocator->cache_size[i];
		snprintf(name, sizeof(name), "vipx=size-%zd", size);

		allocator->cache[i] = kmem_cache_create(name, size,
			ARCH_KMALLOC_MINALIGN, SLAB_POISON | SLAB_PANIC, NULL);
		if (!allocator->cache[i]) {
			probe_err("kmem_cache_create(%zd) is fail\n", size);
			ret = -ENOMEM;
			goto p_err;
		}
	}

p_err:
	return ret;
}

int vipx_slab_alloc(struct vipx_slab_allocator *allocator, void **target, size_t size)
{
	int ret = 0, i;

	for (i = 0; i < VIPX_SLAB_MAX_LEVEL; ++i) {
		if (size <= allocator->cache_size[i])
			break;
	}

	if (i >= VIPX_SLAB_MAX_LEVEL) {
		vipx_err("alloc size is invalid(%zd)\n", size);
		ret= -EINVAL;
		goto p_err;
	}

	*target = kmem_cache_alloc(allocator->cache[i], GFP_KERNEL);
	if (!(*target)) {
		vipx_err("kmem_cache_alloc is fail\n");
		ret = -ENOMEM;
		goto p_err;
	}

p_err:
	return ret;
}

int vipx_slab_free(struct vipx_slab_allocator *allocator, void *target, size_t size)
{
	int ret = 0, i;

	for (i = 0; i < VIPX_SLAB_MAX_LEVEL; ++i) {
		if (size <= allocator->cache_size[i])
			break;
	}

	if (i >= VIPX_SLAB_MAX_LEVEL) {
		vipx_err("alloc size is invalid(%zd)\n", size);
		BUG();
	}

	kmem_cache_free(allocator->cache[i], target);

	return ret;
}
