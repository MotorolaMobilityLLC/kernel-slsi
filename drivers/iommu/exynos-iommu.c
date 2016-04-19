/* linux/drivers/iommu/exynos_iommu.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_iommu.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include <asm/cacheflush.h>
#include <asm/pgtable.h>

#include "exynos-iommu.h"

static int __init exynos_sysmmu_probe(struct platform_device *pdev)
{
	/* Dummy */

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int exynos_sysmmu_suspend(struct device *dev)
{
	return 0;
}

static int exynos_sysmmu_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops sysmmu_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(exynos_sysmmu_suspend, exynos_sysmmu_resume)
};

static const struct of_device_id sysmmu_of_match[] __initconst = {
	{ .compatible	= "samsung,exynos-sysmmu", },
	{ },
};

static struct platform_driver exynos_sysmmu_driver __refdata = {
	.probe	= exynos_sysmmu_probe,
	.driver	= {
		.name		= "exynos-sysmmu",
		.of_match_table	= sysmmu_of_match,
		.pm		= &sysmmu_pm_ops,
	}
};

static struct iommu_domain *exynos_iommu_domain_alloc(unsigned type)
{
	return NULL;
}

static void exynos_iommu_domain_free(struct iommu_domain *iommu_domain)
{
}

static int exynos_iommu_attach_device(struct iommu_domain *iommu_domain,
				   struct device *master)
{
	return 0;
}

static void exynos_iommu_detach_device(struct iommu_domain *iommu_domain,
				    struct device *master)
{
}

static int exynos_iommu_map(struct iommu_domain *iommu_domain,
			    unsigned long l_iova, phys_addr_t paddr, size_t size,
			    int prot)
{
	return 0;
}

static size_t exynos_iommu_unmap(struct iommu_domain *iommu_domain,
				 unsigned long l_iova, size_t size)
{
	return 0;
}

static phys_addr_t exynos_iommu_iova_to_phys(struct iommu_domain *iommu_domain,
					  dma_addr_t d_iova)
{
	return 0;
}

static int exynos_iommu_of_xlate(struct device *master,
				 struct of_phandle_args *spec)
{
	return 0;
}

static struct iommu_ops exynos_iommu_ops = {
	.domain_alloc = exynos_iommu_domain_alloc,
	.domain_free = exynos_iommu_domain_free,
	.attach_dev = exynos_iommu_attach_device,
	.detach_dev = exynos_iommu_detach_device,
	.map = exynos_iommu_map,
	.unmap = exynos_iommu_unmap,
	.map_sg = default_iommu_map_sg,
	.iova_to_phys = exynos_iommu_iova_to_phys,
	.pgsize_bitmap = SECT_SIZE | LPAGE_SIZE | SPAGE_SIZE,
	.of_xlate = exynos_iommu_of_xlate,
};

IOMMU_OF_DECLARE(exynos_iommu_of, "samsung,exynos-sysmmu", NULL);
