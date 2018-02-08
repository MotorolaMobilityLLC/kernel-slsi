/*
 * drivers/staging/android/ion/ion_fdt_exynos.c
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

#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>
#include <linux/cma.h>

#include "ion.h"
#include "ion_exynos.h"

unsigned int reserved_mem_count __initdata;
struct ion_reserved_mem_struct {
	char		*heapname;
	struct cma	*cma;
	phys_addr_t	base;
	phys_addr_t	size;
	unsigned int	alloc_align;
	bool		untouchable;
} ion_reserved_mem[ION_NUM_HEAP_IDS - 1] __initdata;

static int __init exynos_ion_reserved_mem_setup(struct reserved_mem *rmem)
{
	bool untch, reusable;
	size_t alloc_align = PAGE_SIZE;
	char *heapname;
	const __be32 *prop;
	int len;

	reusable = !!of_get_flat_dt_prop(rmem->fdt_node, "ion,reusable", NULL);
	untch = !!of_get_flat_dt_prop(rmem->fdt_node, "ion,untouchable", NULL);

	prop = of_get_flat_dt_prop(rmem->fdt_node, "ion,alignment", &len);
	if (prop && (be32_to_cpu(prop[0]) >= PAGE_SIZE)) {
		alloc_align = be32_to_cpu(prop[0]);
		if ((alloc_align & (alloc_align - 1)) != 0)
			alloc_align = 1 << (get_order(alloc_align) + PAGE_SHIFT);
	}

	prop = of_get_flat_dt_prop(rmem->fdt_node, "ion,heapname", &len);
	if (!prop) {
		pr_err("%s: 'ion,heapname' is missing in '%s' node\n",
		       __func__, rmem->name);
		return -EINVAL;
	}
	heapname = (char *)prop;

	if (reserved_mem_count == ARRAY_SIZE(ion_reserved_mem)) {
		pr_err("%s: Not enough reserved_mem slot for %s\n",
		       __func__, rmem->name);
		return -ENOMEM;
	}

	if (untch && reusable) {
		pr_err("%s: 'reusable', 'untouchable' should not be together\n",
		       __func__);
		return -EINVAL;
	}

	if (reusable) {
		struct cma *cma;
		int ret;

		ret = cma_init_reserved_mem(rmem->base, rmem->size, 0,
					    heapname, &cma);
		if (ret < 0) {
			pr_err("%s: failed to init cma for '%s'\n",
			       __func__, heapname);
			return ret;
		}

		ion_reserved_mem[reserved_mem_count].cma = cma;
	}

	ion_reserved_mem[reserved_mem_count].base = rmem->base;
	ion_reserved_mem[reserved_mem_count].size = rmem->size;
	ion_reserved_mem[reserved_mem_count].heapname = heapname;
	ion_reserved_mem[reserved_mem_count].alloc_align = alloc_align;
	ion_reserved_mem[reserved_mem_count].untouchable = untch;
	reserved_mem_count++;

	return 0;
}

RESERVEDMEM_OF_DECLARE(ion, "exynos9820-ion", exynos_ion_reserved_mem_setup);

static int __init exynos_ion_register_heaps(void)
{
	unsigned int i;

	for (i = 0; i < reserved_mem_count; i++) {
		struct ion_platform_heap pheap;
		struct ion_heap *heap;

		pheap.name	  = ion_reserved_mem[i].heapname;
		pheap.base	  = ion_reserved_mem[i].base;
		pheap.size	  = ion_reserved_mem[i].size;
		pheap.align	  = ion_reserved_mem[i].alloc_align;
		pheap.untouchable = ion_reserved_mem[i].untouchable;

		if (ion_reserved_mem[i].cma) {
			pheap.type = ION_HEAP_TYPE_DMA;
			heap = ion_cma_heap_create(ion_reserved_mem[i].cma,
						   &pheap);
		} else {
			pheap.type = ION_HEAP_TYPE_CARVEOUT;
			heap = ion_carveout_heap_create(&pheap);
		}

		if (IS_ERR(heap)) {
			pr_err("%s: failed to register '%s' heap\n",
			       __func__, pheap.name);
			continue;
		}

		ion_device_add_heap(heap);
		pr_info("ION: registered '%s' heap\n", pheap.name);
	}

	return 0;
}
device_initcall(exynos_ion_register_heaps);
