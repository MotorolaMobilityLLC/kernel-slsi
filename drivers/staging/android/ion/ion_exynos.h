/*
 * drivers/staging/android/ion/ion_exynos.h
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

#ifndef _ION_EXYNOS_H_
#define _ION_EXYNOS_H_

struct cma;
struct ion_heap;
struct ion_platform_heap;

#ifdef CONFIG_ION_CARVEOUT_HEAP
extern struct ion_heap *ion_carveout_heap_create(struct ion_platform_heap *);
#else
#define ion_carveout_heap_create(p) ERR_PTR(-ENODEV)
#endif

#if defined(CONFIG_ION_CMA_HEAP) && defined(CONFIG_ION_EXYNOS)
extern struct ion_heap *ion_cma_heap_create(struct cma *cma,
					    struct ion_platform_heap *pheap);
#else
#define ion_cma_heap_create(cma, p) ERR_PTR(-ENODEV)
#endif

#endif /* _ION_EXYNOS_H_ */
