/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Data structure definition for Exynos IOMMU driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _EXYNOS_IOMMU_H_
#define _EXYNOS_IOMMU_H_

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/genalloc.h>
#include <linux/iommu.h>
#include <linux/irq.h>
#include <linux/clk.h>

typedef u32 sysmmu_pte_t;

#define SECT_ORDER 20
#define LPAGE_ORDER 16
#define SPAGE_ORDER 12

#define SECT_SIZE (1 << SECT_ORDER)
#define LPAGE_SIZE (1 << LPAGE_ORDER)
#define SPAGE_SIZE (1 << SPAGE_ORDER)

#define SECT_MASK (~(SECT_SIZE - 1))
#define LPAGE_MASK (~(LPAGE_SIZE - 1))
#define SPAGE_MASK (~(SPAGE_SIZE - 1))

#define NUM_LV2ENTRIES (SECT_SIZE / SPAGE_SIZE)
#define LV2TABLE_SIZE (NUM_LV2ENTRIES * sizeof(sysmmu_pte_t))

/*
 * This structure exynos specific generalization of struct iommu_domain.
 * It contains list of all master devices represented by owner, which has
 * been attached to this domain and page tables of IO address space defined by
 * it. It is usually referenced by 'domain' pointer.
 */
struct exynos_iommu_domain {
	struct iommu_domain domain;	/* generic domain data structure */
	sysmmu_pte_t *pgtable;		/* lv1 page table, 16KB */
	spinlock_t pgtablelock;		/* lock for modifying page table */
	struct list_head clients_list;	/* list of exynos_iommu_owner.client */
	atomic_t *lv2entcnt;	/* free lv2 entry counter for each section */
	spinlock_t lock;		/* lock for modifying clients_list */
};

/*
 * This structure is attached to dev.archdata.iommu of the master device
 * on device add, contains a list of SYSMMU controllers defined by device tree,
 * which are bound to given master device. It is usually referenced by 'owner'
 * pointer.
*/
struct exynos_iommu_owner {
	struct exynos_iommu_owner *next;
	struct list_head sysmmu_list;	/* list of sysmmu_drvdata */
	spinlock_t lock;		/* lock for modifying sysmmu_list */
	struct iommu_domain *domain;	/* domain of owner */
	struct device *master;		/* master device */
	struct list_head client;	/* node for owner clients_list */
};

/*
 * This structure hold all data of a single SYSMMU controller, this includes
 * hw resources like registers and clocks, pointers and list nodes to connect
 * it to all other structures, internal state and parameters read from device
 * tree. It is usually referenced by 'data' pointer.
 */
struct sysmmu_drvdata {
	struct sysmmu_drvdata *next;
	struct device *sysmmu;		/* SYSMMU controller device */
	void __iomem *sfrbase;		/* our registers */
	struct clk *clk;		/* SYSMMU's clock */
	int activations;		/* number of calls to sysmmu_enable */
	int runtime_active;	/* Runtime PM activated count from master */
	spinlock_t lock;		/* lock for modyfying state */
	phys_addr_t pgtable;		/* assigned page table structure */
	unsigned int version;		/* our version */
};
#endif /* _EXYNOS_IOMMU_H_ */
