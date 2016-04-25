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

static struct kmem_cache *lv2table_kmem_cache;

static struct sysmmu_drvdata *sysmmu_drvdata_list;
static struct exynos_iommu_owner *sysmmu_owner_list;

static struct exynos_iommu_domain *to_exynos_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct exynos_iommu_domain, domain);
}

struct sysmmu_list_data {
	struct device *sysmmu;
	struct list_head node;
};

struct exynos_client {
	struct list_head list;
	struct device_node *master_np;
	struct exynos_iovmm *vmm_data;
};
static LIST_HEAD(exynos_client_list);
static DEFINE_SPINLOCK(exynos_client_lock);

int exynos_client_add(struct device_node *np, struct exynos_iovmm *vmm_data)
{
	struct exynos_client *client = kzalloc(sizeof(*client), GFP_KERNEL);

	if (!client)
		return -ENOMEM;

	INIT_LIST_HEAD(&client->list);
	client->master_np = np;
	client->vmm_data = vmm_data;
	spin_lock(&exynos_client_lock);
	list_add_tail(&client->list, &exynos_client_list);
	spin_unlock(&exynos_client_lock);

	return 0;
}

#define has_sysmmu(dev)		((dev)->archdata.iommu != NULL)

/* For ARM64 only */
static inline void pgtable_flush(void *vastart, void *vaend)
{
	__dma_flush_area(vastart, vaend - vastart);
}

static bool has_sysmmu_capable_pbuf(void __iomem *sfrbase)
{
	unsigned long cfg = readl_relaxed(sfrbase + REG_MMU_CAPA);

	return MMU_HAVE_PB(cfg);
}

static bool has_sysmmu_set_associative_tlb(void __iomem *sfrbase)
{
	u32 cfg = readl_relaxed(sfrbase + REG_MMU_CAPA_1);

	return MMU_IS_TLB_CONFIGURABLE(cfg);
}

static void __sysmmu_set_tlb_line_size(void __iomem *sfrbase)
{
	u32 cfg =  readl_relaxed(sfrbase + REG_L2TLB_CFG);
	cfg &= ~MMU_MASK_LINE_SIZE;
	cfg |= MMU_DEFAULT_LINE_SIZE;
	writel_relaxed(cfg, sfrbase + REG_L2TLB_CFG);
}

static void __exynos_sysmmu_set_prefbuf_axi_id(struct sysmmu_drvdata *drvdata)
{
	if (!has_sysmmu_capable_pbuf(drvdata->sfrbase))
		return;

	/* TODO: implement later */
}

static void __sysmmu_tlb_invalidate_all(void __iomem *sfrbase)
{
	writel(0x1, sfrbase + REG_MMU_FLUSH);
}

static void __sysmmu_set_ptbase(void __iomem *sfrbase, phys_addr_t pfn_pgtable)
{
	writel_relaxed(pfn_pgtable, sfrbase + REG_PT_BASE_PPN);

	__sysmmu_tlb_invalidate_all(sfrbase);
}

void exynos_sysmmu_tlb_invalidate(struct iommu_domain *iommu_domain,
					dma_addr_t d_start, size_t size)
{
	return;
}

int exynos_iommu_map_userptr(struct iommu_domain *dom, unsigned long addr,
			      dma_addr_t d_iova, size_t size, int prot)
{
	return 0;
}

void exynos_iommu_unmap_userptr(struct iommu_domain *dom,
				dma_addr_t d_iova, size_t size)
{
	return;
}

static irqreturn_t exynos_sysmmu_irq(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static int get_hw_version(struct device *dev, void __iomem *sfrbase)
{
	int ret;

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "Failed to runtime pm get(%d)\n", ret);
		return ret;
	}
	ret = MMU_RAW_VER(__raw_readl(sfrbase + REG_MMU_VERSION));
	pm_runtime_put(dev);

	return ret;
}

static struct iommu_ops exynos_iommu_ops;
static int __init exynos_sysmmu_probe(struct platform_device *pdev)
{
	int irq, ret;
	struct device *dev = &pdev->dev;
	struct sysmmu_drvdata *data;
	struct resource *res;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Failed to get resource info\n");
		return -ENOENT;
	}

	data->sfrbase = devm_ioremap_resource(dev, res);
	if (IS_ERR(data->sfrbase))
		return PTR_ERR(data->sfrbase);

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(dev, "Unable to find IRQ resource\n");
		return irq;
	}

	ret = devm_request_irq(dev, irq, exynos_sysmmu_irq, 0,
				dev_name(dev), data);
	if (ret) {
		dev_err(dev, "Unabled to register handler of irq %d\n", irq);
		return ret;
	}

	data->clk = devm_clk_get(dev, "aclk");
	if (IS_ERR(data->clk)) {
		dev_err(dev, "Failed to get clock!\n");
		return PTR_ERR(data->clk);
	} else  {
		ret = clk_prepare(data->clk);
		if (ret) {
			dev_err(dev, "Failed to prepare clk\n");
			return ret;
		}
	}

	data->sysmmu = dev;
	spin_lock_init(&data->lock);
	if (!sysmmu_drvdata_list) {
		sysmmu_drvdata_list = data;
	} else {
		data->next = sysmmu_drvdata_list->next;
		sysmmu_drvdata_list->next = data;
	}

	platform_set_drvdata(pdev, data);

	pm_runtime_enable(dev);

	data->version = get_hw_version(dev, data->sfrbase);

	/* TODO: Parsing Device Tree for properties */

	iommu_device_set_ops(&data->iommu, &exynos_iommu_ops);
	iommu_device_set_fwnode(&data->iommu, &dev->of_node->fwnode);

	ret = iommu_device_register(&data->iommu);
	if (ret) {
		dev_err(dev, "Failed to register device\n");
		return ret;
	}

	dev_info(data->sysmmu, "is probed. Version %d.%d.%d\n",
			MMU_MAJ_VER(data->version),
			MMU_MIN_VER(data->version),
			MMU_REV_VER(data->version));

	return 0;
}

static void __sysmmu_disable_nocount(struct sysmmu_drvdata *drvdata)
{
	writel_relaxed(0, drvdata->sfrbase + REG_MMU_CFG);
	writel_relaxed(CTRL_BLOCK_DISABLE, drvdata->sfrbase + REG_MMU_CTRL);
	BUG_ON(readl_relaxed(drvdata->sfrbase + REG_MMU_CTRL) != CTRL_BLOCK_DISABLE);

	clk_disable(drvdata->clk);
}

static bool __sysmmu_disable(struct sysmmu_drvdata *drvdata)
{
	bool disabled;
	unsigned long flags;

	spin_lock_irqsave(&drvdata->lock, flags);

	disabled = set_sysmmu_inactive(drvdata);

	if (disabled) {
		drvdata->pgtable = 0;

		if (is_sysmmu_runtime_active(drvdata))
			__sysmmu_disable_nocount(drvdata);

		dev_dbg(drvdata->sysmmu, "Disabled\n");
	} else  {
		dev_dbg(drvdata->sysmmu, "%d times left to disable\n",
					drvdata->activations);
	}

	spin_unlock_irqrestore(&drvdata->lock, flags);

	return disabled;
}

static void sysmmu_disable_from_master(struct device *master)
{
	unsigned long flags;
	struct exynos_iommu_owner *owner = master->archdata.iommu;
	struct sysmmu_list_data *list;
	struct sysmmu_drvdata *drvdata;

	BUG_ON(!has_sysmmu(master));

	spin_lock_irqsave(&owner->lock, flags);
	list_for_each_entry(list, &owner->sysmmu_list, node) {
		drvdata = dev_get_drvdata(list->sysmmu);
		__sysmmu_disable(drvdata);
	}
	spin_unlock_irqrestore(&owner->lock, flags);
}

static void __sysmmu_init_config(struct sysmmu_drvdata *drvdata)
{
	unsigned long cfg;

	writel_relaxed(CTRL_BLOCK, drvdata->sfrbase + REG_MMU_CTRL);

	if (MMU_MAJ_VER(drvdata->version) >= 7) {
		/* TODO: implement later */
	} else {
		__exynos_sysmmu_set_prefbuf_axi_id(drvdata);
		if (has_sysmmu_set_associative_tlb(drvdata->sfrbase))
			__sysmmu_set_tlb_line_size(drvdata->sfrbase);
	}

	cfg = CFG_FLPDCACHE | CFG_ACGEN;
	cfg |= __raw_readl(drvdata->sfrbase + REG_MMU_CFG) & ~CFG_MASK;
	writel_relaxed(cfg, drvdata->sfrbase + REG_MMU_CFG);
}

static void __sysmmu_enable_nocount(struct sysmmu_drvdata *drvdata)
{
	clk_enable(drvdata->clk);

	__sysmmu_init_config(drvdata);

	__sysmmu_set_ptbase(drvdata->sfrbase, drvdata->pgtable / PAGE_SIZE);

	writel(CTRL_ENABLE, drvdata->sfrbase + REG_MMU_CTRL);
}

static int __sysmmu_enable(struct sysmmu_drvdata *drvdata, phys_addr_t pgtable)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&drvdata->lock, flags);
	if (set_sysmmu_active(drvdata)) {
		drvdata->pgtable = pgtable;

		if (is_sysmmu_runtime_active(drvdata))
			__sysmmu_enable_nocount(drvdata);

		dev_dbg(drvdata->sysmmu, "Enabled\n");
	} else {
		ret = (pgtable == drvdata->pgtable) ? 1 : -EBUSY;

		dev_dbg(drvdata->sysmmu, "already enabled\n");
	}

	if (WARN_ON(ret < 0))
		set_sysmmu_inactive(drvdata); /* decrement count */

	spin_unlock_irqrestore(&drvdata->lock, flags);

	return ret;
}

static int sysmmu_enable_from_master(struct device *master,
				struct exynos_iommu_domain *domain)
{
	int ret = 0;
	unsigned long flags;
	struct exynos_iommu_owner *owner = master->archdata.iommu;
	struct sysmmu_list_data *list;
	struct sysmmu_drvdata *drvdata;
	phys_addr_t pgtable = virt_to_phys(domain->pgtable);

	spin_lock_irqsave(&owner->lock, flags);
	list_for_each_entry(list, &owner->sysmmu_list, node) {
		drvdata = dev_get_drvdata(list->sysmmu);
		ret = __sysmmu_enable(drvdata, pgtable);

		/* rollback if enable is failed */
		if (ret < 0) {
			list_for_each_entry_continue_reverse(list,
						&owner->sysmmu_list, node) {
				drvdata = dev_get_drvdata(list->sysmmu);
				__sysmmu_disable(drvdata);
			}
			break;
		}
	}
	spin_unlock_irqrestore(&owner->lock, flags);

	return ret;
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
	struct exynos_iommu_domain *domain;

	if (type != IOMMU_DOMAIN_UNMANAGED)
		return NULL;

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);
	if (!domain)
		return NULL;

	domain->pgtable = (sysmmu_pte_t *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, 2);
	if (!domain->pgtable)
		goto err_pgtable;

	domain->lv2entcnt = (atomic_t *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, 2);
	if (!domain->lv2entcnt)
		goto err_counter;

	pgtable_flush(domain->pgtable, domain->pgtable + NUM_LV1ENTRIES);

	spin_lock_init(&domain->lock);
	spin_lock_init(&domain->pgtablelock);
	INIT_LIST_HEAD(&domain->clients_list);

	/* TODO: get geometry from device tree */
	domain->domain.geometry.aperture_start = 0;
	domain->domain.geometry.aperture_end   = ~0UL;
	domain->domain.geometry.force_aperture = true;

	return &domain->domain;

err_counter:
	free_pages((unsigned long)domain->pgtable, 2);
err_pgtable:
	kfree(domain);
	return NULL;
}

static void exynos_iommu_domain_free(struct iommu_domain *iommu_domain)
{
	struct exynos_iommu_domain *domain = to_exynos_domain(iommu_domain);
	struct exynos_iommu_owner *owner;
	unsigned long flags;
	int i;

	WARN_ON(!list_empty(&domain->clients_list));

	spin_lock_irqsave(&domain->lock, flags);

	list_for_each_entry(owner, &domain->clients_list, client)
		sysmmu_disable_from_master(owner->master);

	while (!list_empty(&domain->clients_list))
		list_del_init(domain->clients_list.next);

	spin_unlock_irqrestore(&domain->lock, flags);

	for (i = 0; i < NUM_LV1ENTRIES; i++)
		if (lv1ent_page(domain->pgtable + i))
			kmem_cache_free(lv2table_kmem_cache,
				phys_to_virt(lv2table_base(domain->pgtable + i)));

	free_pages((unsigned long)domain->pgtable, 2);
	free_pages((unsigned long)domain->lv2entcnt, 2);
	kfree(domain);
}

static int exynos_iommu_attach_device(struct iommu_domain *iommu_domain,
				   struct device *master)
{
	struct exynos_iommu_domain *domain = to_exynos_domain(iommu_domain);
	struct exynos_iommu_owner *owner;
	phys_addr_t pagetable = virt_to_phys(domain->pgtable);
	unsigned long flags;
	int ret = -ENODEV;

	if (!has_sysmmu(master)) {
		dev_err(master, "has no sysmmu device.\n");
		return -ENODEV;
	}

	spin_lock_irqsave(&domain->lock, flags);
	list_for_each_entry(owner, &domain->clients_list, client) {
		if (owner->master == master) {
			dev_err(master, "is already attached!\n");
			spin_unlock_irqrestore(&domain->lock, flags);
			return -EEXIST;
		}
	}
	/* owner is under domain. */
	owner = master->archdata.iommu;
	list_add_tail(&owner->client, &domain->clients_list);
	spin_unlock_irqrestore(&domain->lock, flags);

	ret = sysmmu_enable_from_master(master, domain);
	if (ret < 0) {
		dev_err(master, "%s: Failed to attach IOMMU with pgtable %pa\n",
					__func__, &pagetable);
		return ret;
	}

	dev_dbg(master, "%s: Attached IOMMU with pgtable %pa %s\n",
		__func__, &pagetable, (ret == 0) ? "" : ", again");

	return ret;
}

static void exynos_iommu_detach_device(struct iommu_domain *iommu_domain,
				    struct device *master)
{
	struct exynos_iommu_domain *domain = to_exynos_domain(iommu_domain);
	phys_addr_t pagetable = virt_to_phys(domain->pgtable);
	struct exynos_iommu_owner *owner, *tmp_owner;
	unsigned long flags;
	bool found = false;

	if (!has_sysmmu(master))
		return;

	spin_lock_irqsave(&domain->lock, flags);
	list_for_each_entry_safe(owner, tmp_owner,
				&domain->clients_list, client) {
		if (owner->master == master) {
			sysmmu_disable_from_master(master);
			list_del_init(&owner->client);
			found = true;
		}
	}
	spin_unlock_irqrestore(&domain->lock, flags);

	if (found)
		dev_dbg(master, "%s: Detached IOMMU with pgtable %pa\n",
					__func__, &pagetable);
	else
		dev_err(master, "%s: No IOMMU is attached\n", __func__);
}

static sysmmu_pte_t *alloc_lv2entry(struct exynos_iommu_domain *domain,
		sysmmu_pte_t *sent, sysmmu_iova_t iova, atomic_t *pgcounter)
{
	if (lv1ent_section(sent)) {
		WARN(1, "Trying mapping on %#08x mapped with 1MiB page", iova);
		return ERR_PTR(-EADDRINUSE);
	}

	if (lv1ent_fault(sent)) {
		unsigned long flags;
		spin_lock_irqsave(&domain->pgtablelock, flags);
		if (lv1ent_fault(sent)) {
			sysmmu_pte_t *pent;

			pent = kmem_cache_zalloc(lv2table_kmem_cache, GFP_ATOMIC);
			BUG_ON((unsigned long)pent & (LV2TABLE_SIZE - 1));
			if (!pent) {
				spin_unlock_irqrestore(&domain->pgtablelock, flags);
				return ERR_PTR(-ENOMEM);
			}

			*sent = mk_lv1ent_page(virt_to_phys(pent));
			kmemleak_ignore(pent);
			atomic_set(pgcounter, NUM_LV2ENTRIES);
			pgtable_flush(pent, pent + NUM_LV2ENTRIES);
			pgtable_flush(sent, sent + 1);
		}
		spin_unlock_irqrestore(&domain->pgtablelock, flags);
	}

	return page_entry(sent, iova);
}
static void clear_lv2_page_table(sysmmu_pte_t *ent, int n)
{
	if (n > 0)
		memset(ent, 0, sizeof(*ent) * n);
}

static int lv1set_section(struct exynos_iommu_domain *domain,
			  sysmmu_pte_t *sent, sysmmu_iova_t iova,
			  phys_addr_t paddr, atomic_t *pgcnt)
{
	if (lv1ent_section(sent)) {
		WARN(1, "Trying mapping on 1MiB@%#08x that is mapped",
			iova);
		return -EADDRINUSE;
	}

	if (lv1ent_page(sent)) {
		if (WARN_ON(atomic_read(pgcnt) != NUM_LV2ENTRIES)) {
			WARN(1, "Trying mapping on 1MiB@%#08x that is mapped",
				iova);
			return -EADDRINUSE;
		}
		/* TODO: for v7, free lv2 page table */
	}

	*sent = mk_lv1ent_sect(paddr);
	pgtable_flush(sent, sent + 1);

	return 0;
}

static int lv2set_page(sysmmu_pte_t *pent, phys_addr_t paddr, size_t size,
								atomic_t *pgcnt)
{
	if (size == SPAGE_SIZE) {
		if (WARN_ON(!lv2ent_fault(pent)))
			return -EADDRINUSE;

		*pent = mk_lv2ent_spage(paddr);
		pgtable_flush(pent, pent + 1);
		atomic_dec(pgcnt);
	} else { /* size == LPAGE_SIZE */
		int i;

		for (i = 0; i < SPAGES_PER_LPAGE; i++, pent++) {
			if (WARN_ON(!lv2ent_fault(pent))) {
				clear_lv2_page_table(pent - i, i);
				return -EADDRINUSE;
			}

			*pent = mk_lv2ent_lpage(paddr);
		}
		pgtable_flush(pent - SPAGES_PER_LPAGE, pent);
		atomic_sub(SPAGES_PER_LPAGE, pgcnt);
	}

	return 0;
}
static int exynos_iommu_map(struct iommu_domain *iommu_domain,
			    unsigned long l_iova, phys_addr_t paddr, size_t size,
			    int prot)
{
	struct exynos_iommu_domain *domain = to_exynos_domain(iommu_domain);
	sysmmu_pte_t *entry;
	sysmmu_iova_t iova = (sysmmu_iova_t)l_iova;
	int ret = -ENOMEM;

	BUG_ON(domain->pgtable == NULL);

	entry = section_entry(domain->pgtable, iova);

	if (size == SECT_SIZE) {
		ret = lv1set_section(domain, entry, iova, paddr,
				     &domain->lv2entcnt[lv1ent_offset(iova)]);
	} else {
		sysmmu_pte_t *pent;

		pent = alloc_lv2entry(domain, entry, iova,
				      &domain->lv2entcnt[lv1ent_offset(iova)]);

		if (IS_ERR(pent))
			ret = PTR_ERR(pent);
		else
			ret = lv2set_page(pent, paddr, size,
				       &domain->lv2entcnt[lv1ent_offset(iova)]);
	}

	if (ret)
		pr_err("%s: Failed(%d) to map %#zx bytes @ %#x\n",
			__func__, ret, size, iova);

	return ret;
}

static size_t exynos_iommu_unmap(struct iommu_domain *iommu_domain,
				 unsigned long l_iova, size_t size)
{
	struct exynos_iommu_domain *domain = to_exynos_domain(iommu_domain);
	sysmmu_iova_t iova = (sysmmu_iova_t)l_iova;
	sysmmu_pte_t *sent, *pent;
	size_t err_pgsize;
	atomic_t *lv2entcnt = &domain->lv2entcnt[lv1ent_offset(iova)];

	BUG_ON(domain->pgtable == NULL);

	sent = section_entry(domain->pgtable, iova);

	if (lv1ent_section(sent)) {
		if (WARN_ON(size < SECT_SIZE)) {
			err_pgsize = SECT_SIZE;
			goto err;
		}

		*sent = 0;
		pgtable_flush(sent, sent + 1);
		size = SECT_SIZE;
		goto done;
	}

	if (unlikely(lv1ent_fault(sent))) {
		if (size > SECT_SIZE)
			size = SECT_SIZE;
		goto done;
	}

	/* lv1ent_page(sent) == true here */

	pent = page_entry(sent, iova);

	if (unlikely(lv2ent_fault(pent))) {
		size = SPAGE_SIZE;
		goto done;
	}

	if (lv2ent_small(pent)) {
		*pent = 0;
		size = SPAGE_SIZE;
		pgtable_flush(pent, pent + 1);
		atomic_inc(lv2entcnt);
		goto unmap_flpd;
	}

	/* lv1ent_large(pent) == true here */
	if (WARN_ON(size < LPAGE_SIZE)) {
		err_pgsize = LPAGE_SIZE;
		goto err;
	}

	clear_lv2_page_table(pent, SPAGES_PER_LPAGE);
	pgtable_flush(pent, pent + SPAGES_PER_LPAGE);
	size = LPAGE_SIZE;
	atomic_add(SPAGES_PER_LPAGE, lv2entcnt);

unmap_flpd:
	/* TODO: for v7, remove all */
	if (atomic_read(lv2entcnt) == NUM_LV2ENTRIES) {
		unsigned long flags;
		spin_lock_irqsave(&domain->pgtablelock, flags);
		if (atomic_read(lv2entcnt) == NUM_LV2ENTRIES) {
			kmem_cache_free(lv2table_kmem_cache,
					page_entry(sent, 0));
			atomic_set(lv2entcnt, 0);
			*sent = 0;
		}
		spin_unlock_irqrestore(&domain->pgtablelock, flags);
	}

done:
	/* TLB invalidation is performed by IOVMM */

	return size;
err:
	pr_err("%s: Failed: size(%#zx) @ %#x is smaller than page size %#zx\n",
		__func__, size, iova, err_pgsize);

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
	struct exynos_iommu_owner *owner = master->archdata.iommu;
	struct platform_device *sysmmu_pdev = of_find_device_by_node(spec->np);
	struct sysmmu_drvdata *data;
	struct device *sysmmu;
	struct exynos_client *client, *buf_client;
	struct sysmmu_list_data *list_data;

	if (!sysmmu_pdev)
		return -ENODEV;

	data = platform_get_drvdata(sysmmu_pdev);
	if (!data)
		return -ENODEV;

	sysmmu = data->sysmmu;
	if (!owner) {
		owner = kzalloc(sizeof(*owner), GFP_KERNEL);
		if (!owner)
			return -ENOMEM;

		INIT_LIST_HEAD(&owner->sysmmu_list);
		INIT_LIST_HEAD(&owner->client);
		master->archdata.iommu = owner;
		owner->master = master;
		spin_lock_init(&owner->lock);

		list_for_each_entry_safe(client, buf_client,
					&exynos_client_list, list) {
			if (client->master_np == master->of_node) {
				owner->domain = client->vmm_data->domain;
				owner->vmm_data = client->vmm_data;
				list_del(&client->list);
				kfree(client);
			}
		}

		if (!sysmmu_owner_list) {
			sysmmu_owner_list = owner;
		} else {
			owner->next = sysmmu_owner_list->next;
			sysmmu_owner_list->next = owner;
		}
	}

	list_for_each_entry(list_data, &owner->sysmmu_list, node)
		if (list_data->sysmmu == sysmmu)
			return 0;

	list_data = devm_kzalloc(sysmmu, sizeof(*list_data), GFP_KERNEL);
	if (!list_data)
		return -ENOMEM;

	INIT_LIST_HEAD(&list_data->node);
	list_data->sysmmu = sysmmu;

	/*
	 * Use device link to make relationship between SysMMU and master.
	 * SysMMU device is supplier, and master device is consumer.
	 * This relationship guarantees that supplier is enabled before
	 * consumer, and it is disabled after consumer.
	 */
	device_link_add(master, sysmmu, DL_FLAG_PM_RUNTIME);

	/*
	 * System MMUs are attached in the order of the presence
	 * in device tree
	 */
	list_add_tail(&list_data->node, &owner->sysmmu_list);
	dev_info(master, "is owner of %s\n", dev_name(sysmmu));

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

static int __init exynos_iommu_create_domain(void)
{
	struct device_node *domain_np;
	int ret;

	for_each_compatible_node(domain_np, NULL, "samsung,exynos-iommu-bus") {
		struct device_node *np;
		struct exynos_iovmm *vmm = NULL;
		int i = 0;

		while ((np = of_parse_phandle(domain_np, "domain-clients", i++))) {
			if (!vmm) {
				vmm = exynos_create_single_iovmm(np->name);
				if (IS_ERR(vmm)) {
					pr_err("%s: Failed to create IOVM space\
							of %s\n",
							__func__, np->name);
					of_node_put(np);
					of_node_put(domain_np);
					return -ENOMEM;
				}
			}
			/* Relationship between domain and client is added. */
			ret = exynos_client_add(np, vmm);
			if (ret) {
				pr_err("Failed to adding client[%s] to domain %s\n",
						np->name, domain_np->name);
				of_node_put(np);
				of_node_put(domain_np);
				return -ENOMEM;
			} else {
				pr_info("Added client.%d[%s] into domain %s\n",
						i, np->name, domain_np->name);
			}
			of_node_put(np);
		}
		of_node_put(domain_np);
	}

	return 0;
}

static int __init exynos_iommu_init(void)
{
	int ret;

	lv2table_kmem_cache = kmem_cache_create("exynos-iommu-lv2table",
				LV2TABLE_SIZE, LV2TABLE_SIZE, 0, NULL);
	if (!lv2table_kmem_cache) {
		pr_err("%s: Failed to create kmem cache\n", __func__);
		return -ENOMEM;
	}

	ret = platform_driver_register(&exynos_sysmmu_driver);
	if (ret) {
		pr_err("%s: Failed to register driver\n", __func__);
		goto err_reg_driver;
	}

	ret = bus_set_iommu(&platform_bus_type, &exynos_iommu_ops);
	if (ret) {
		pr_err("%s: Failed to register exynos-iommu driver.\n",
								__func__);
		goto err_set_iommu;
	}
	ret = exynos_iommu_create_domain();
	if (ret) {
		pr_err("%s: Failed to create domain\n", __func__);
		goto err_create_domain;
	}

	return 0;

err_create_domain:
	bus_set_iommu(&platform_bus_type, NULL);
err_set_iommu:
	platform_driver_unregister(&exynos_sysmmu_driver);
err_reg_driver:
	kmem_cache_destroy(lv2table_kmem_cache);
	return ret;
}
core_initcall(exynos_iommu_init);

IOMMU_OF_DECLARE(exynos_iommu_of, "samsung,exynos-sysmmu", NULL);
