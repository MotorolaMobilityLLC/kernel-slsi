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

#define IOVM_NUM_PAGES(vmsize) (vmsize / PAGE_SIZE)
#define IOVM_BITMAP_SIZE(vmsize) \
		((IOVM_NUM_PAGES(vmsize) + BITS_PER_BYTE) / BITS_PER_BYTE)

#define SECT_ORDER 20
#define LPAGE_ORDER 16
#define SPAGE_ORDER 12

#define SECT_SIZE (1 << SECT_ORDER)
#define LPAGE_SIZE (1 << LPAGE_ORDER)
#define SPAGE_SIZE (1 << SPAGE_ORDER)

#define SECT_MASK (~(SECT_SIZE - 1))
#define LPAGE_MASK (~(LPAGE_SIZE - 1))
#define SPAGE_MASK (~(SPAGE_SIZE - 1))

#define NUM_LV1ENTRIES	4096
#define NUM_LV2ENTRIES (SECT_SIZE / SPAGE_SIZE)
#define LV2TABLE_SIZE (NUM_LV2ENTRIES * sizeof(sysmmu_pte_t))

#define PG_ENT_SHIFT	4
#define lv1ent_fault(sent)	((*(sent) & 7) == 0)
#define lv1ent_page(sent)	((*(sent) & 7) == 1)

#define lv2table_base(sent)	((phys_addr_t)(*(sent) & ~0x3F) << PG_ENT_SHIFT)

#define REG_MMU_CTRL		0x000
#define REG_MMU_CFG		0x004
#define REG_MMU_STATUS		0x008
#define REG_MMU_VERSION		0x034

#define CTRL_ENABLE	0x5
#define CTRL_BLOCK	0x7
#define CTRL_DISABLE	0x0
#define CTRL_BLOCK_DISABLE 0x3

#define CFG_MASK	0x01101FBC /* Selecting bit 24, 20, 12-7, 5-2 */
#define CFG_ACGEN	(1 << 24)
#define CFG_FLPDCACHE	(1 << 20)

#define REG_PT_BASE_PPN		0x00C
#define REG_MMU_FLUSH		0x010
#define REG_MMU_FLUSH_ENTRY	0x014
#define REG_MMU_FLUSH_RANGE	0x018
#define REG_FLUSH_RANGE_START	0x020
#define REG_FLUSH_RANGE_END	0x024
#define REG_MMU_CAPA		0x030
#define REG_MMU_CAPA_1		0x038
#define REG_INT_STATUS		0x060
#define REG_INT_CLEAR		0x064

#define REG_L2TLB_CFG		0x200

#define MMU_HAVE_PB(reg)	(!!((reg >> 20) & 0xF))
#define MMU_IS_TLB_CONFIGURABLE(reg)	(!!((reg >> 16) & 0xFF))

#define MMU_MASK_LINE_SIZE	0x7
#define MMU_DEFAULT_LINE_SIZE	(0x2 << 4)

#define MMU_MAJ_VER(val)	((val) >> 11)
#define MMU_MIN_VER(val)	((val >> 4) & 0x7F)
#define MMU_REV_VER(val)	((val) & 0xF)
#define MMU_RAW_VER(reg)	(((reg) >> 17) & 0x7FFF) /* upper 15 bits */

#define MAKE_MMU_VER(maj, min)	((((maj) & 0xF) << 11) | \
					(((min) & 0x7F) << 4))
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
	struct exynos_iovmm *vmm_data;
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
	struct iommu_device iommu;	/* IOMMU core handle */
	int activations;		/* number of calls to sysmmu_enable */
	int runtime_active;	/* Runtime PM activated count from master */
	spinlock_t lock;		/* lock for modyfying state */
	phys_addr_t pgtable;		/* assigned page table structure */
	int version;			/* our version */
};

struct exynos_vm_region {
	struct list_head node;
	u32 start;
	u32 size;
	u32 section_off;
	u32 dummy_size;
};

struct exynos_iovmm {
	struct iommu_domain *domain;	/* iommu domain for this iovmm */
	size_t iovm_size;		/* iovm bitmap size per plane */
	u32 iova_start;			/* iovm start address per plane */
	unsigned long *vm_map;		/* iovm biatmap per plane */
	struct list_head regions_list;	/* list of exynos_vm_region */
	spinlock_t vmlist_lock;		/* lock for updating regions_list */
	spinlock_t bitmap_lock;		/* lock for manipulating bitmaps */
	struct device *dev;	/* peripheral device that has this iovmm */
	size_t allocated_size;
	int num_areas;
	unsigned int num_map;
	unsigned int num_unmap;
	const char *domain_name;
};

void exynos_sysmmu_tlb_invalidate(struct iommu_domain *domain, dma_addr_t start,
				  size_t size);
int exynos_iommu_map_userptr(struct iommu_domain *dom, unsigned long addr,
			      dma_addr_t iova, size_t size, int prot);
void exynos_iommu_unmap_userptr(struct iommu_domain *dom,
				dma_addr_t iova, size_t size);

static inline bool get_sysmmu_runtime_active(struct sysmmu_drvdata *data)
{
	return ++data->runtime_active == 1;
}

static inline bool put_sysmmu_runtime_active(struct sysmmu_drvdata *data)
{
	BUG_ON(data->runtime_active < 1);
	return --data->runtime_active == 0;
}

static inline bool is_sysmmu_runtime_active(struct sysmmu_drvdata *data)
{
	return data->runtime_active > 0;
}

static inline bool set_sysmmu_active(struct sysmmu_drvdata *data)
{
	/* return true if the System MMU was not active previously
	   and it needs to be initialized */
	return ++data->activations == 1;
}

static inline bool set_sysmmu_inactive(struct sysmmu_drvdata *data)
{
	/* return true if the System MMU is needed to be disabled */
	BUG_ON(data->activations < 1);
	return --data->activations == 0;
}

static inline bool is_sysmmu_active(struct sysmmu_drvdata *data)
{
	return data->activations > 0;
}

#if defined(CONFIG_EXYNOS_IOVMM)
static inline struct exynos_iovmm *exynos_get_iovmm(struct device *dev)
{
	if (!dev->archdata.iommu) {
		dev_err(dev, "%s: System MMU is not configured\n", __func__);
		return NULL;
	}

	return ((struct exynos_iommu_owner *)dev->archdata.iommu)->vmm_data;
}

struct exynos_vm_region *find_iovm_region(struct exynos_iovmm *vmm,
						dma_addr_t iova);

struct exynos_iovmm *exynos_create_single_iovmm(const char *name);
#else
static inline struct exynos_iovmm *exynos_get_iovmm(struct device *dev)
{
	return NULL;
}

struct exynos_vm_region *find_iovm_region(struct exynos_iovmm *vmm,
						dma_addr_t iova)
{
	return NULL;
}

static inline struct exynos_iovmm *exynos_create_single_iovmm(const char *name)
{
	return NULL;
}
#endif /* CONFIG_EXYNOS_IOVMM */

#endif /* _EXYNOS_IOMMU_H_ */
