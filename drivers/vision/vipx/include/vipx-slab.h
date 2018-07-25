/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef VIPX_SLAB_H_
#define VIPX_SLAB_H_

#include <interface/ap_vip_if.h>

#define VIPX_SLAB_MAX_LEVEL		1

struct vipx_slab_allocator {
	u32				cache_size[VIPX_SLAB_MAX_LEVEL];
	struct kmem_cache		*cache[VIPX_SLAB_MAX_LEVEL];
};

int vipx_slab_init(struct vipx_slab_allocator *allocator);
int vipx_slab_alloc(struct vipx_slab_allocator *allocator, void **target, size_t size);
int vipx_slab_free(struct vipx_slab_allocator *allocator, void *target, size_t size);

#endif
