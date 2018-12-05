/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __VIPX_SLAB_H__
#define __VIPX_SLAB_H__

#include <linux/slab.h>

struct vipx_slab_allocator {
	size_t			cache_size;
	struct kmem_cache	*cache;
};

int vipx_slab_alloc(struct vipx_slab_allocator *allocator, void **target,
		size_t size);
void vipx_slab_free(struct vipx_slab_allocator *allocator, void *target);

int vipx_slab_init(struct vipx_slab_allocator *allocator);
void vipx_slab_deinit(struct vipx_slab_allocator *allocator);

#endif
