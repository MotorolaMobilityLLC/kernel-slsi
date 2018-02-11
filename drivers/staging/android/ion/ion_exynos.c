/*
 * drivers/staging/android/ion/ion_exynos.c
 *
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "ion.h"
#include "ion_exynos.h"

struct dma_buf *ion_alloc_dmabuf(const char *heap_name,
				 size_t len, unsigned int flags)
{
	struct ion_heap *heap = ion_get_heap_by_name(heap_name);

	if (!heap) {
		pr_err("%s: heap '%s' is not found\n", __func__, heap_name);
		return ERR_PTR(-EINVAL);
	}

	return __ion_alloc(len, 1 << heap->id, flags);
}
