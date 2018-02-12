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

#include <linux/slab.h>
#include <linux/dma-buf.h>
#include <linux/exynos_iovmm.h>

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

bool ion_cached_dmabuf(struct dma_buf *dmabuf)
{
	return ion_buffer_cached(dmabuf->priv);
}

bool ion_hwrender_dmabuf(struct dma_buf *dmabuf)
{
	struct ion_buffer *buffer = dmabuf->priv;

	return !!(buffer->flags & ION_FLAG_MAY_HWRENDER);
}

struct ion_iovm_map {
	struct list_head list;
	struct device *dev;
	struct iommu_domain *domain;
	dma_addr_t iova;
	atomic_t mapcnt;
	int prop;
};

static struct ion_iovm_map *ion_buffer_iova_create(struct ion_buffer *buffer,
						   struct device *dev,
						   enum dma_data_direction dir,
						   int prop)
{
	struct ion_iovm_map *iovm_map;

	iovm_map = kzalloc(sizeof(*iovm_map), GFP_KERNEL);
	if (!iovm_map)
		return ERR_PTR(-ENOMEM);

	iovm_map->iova = iovmm_map(dev, buffer->sg_table->sgl,
				   0, buffer->size, dir, prop);
	if (IS_ERR_VALUE(iovm_map->iova)) {
		int ret = (int)iovm_map->iova;

		kfree(iovm_map);
		dev_err(dev, "%s: failed to allocate iova (err %d)\n",
			__func__, ret);
		return ERR_PTR(ret);
	}

	iovm_map->dev = dev;
	iovm_map->domain = get_domain_from_dev(dev);
	iovm_map->prop= prop;

	atomic_inc(&iovm_map->mapcnt);

	return iovm_map;
}

dma_addr_t ion_iovmm_map(struct dma_buf_attachment *attachment,
			 off_t offset, size_t size,
			 enum dma_data_direction direction, int prop)
{
	struct ion_buffer *buffer = attachment->dmabuf->priv;
	struct ion_iovm_map *iovm_map;
	struct iommu_domain *domain;

	BUG_ON(attachment->dmabuf->ops != &ion_dma_buf_ops);

	if (IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION) &&
				(buffer->flags & ION_FLAG_PROTECTED)) {
		struct ion_buffer_prot_info *prot = buffer->priv_virt;

		return prot->dma_addr;
	}

	domain = get_domain_from_dev(attachment->dev);
	if (!domain) {
		dev_err(attachment->dev, "%s: no iommu domain\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&buffer->lock);

	if (!ion_buffer_cached(buffer))
		prop &= ~IOMMU_CACHE;

	list_for_each_entry(iovm_map, &buffer->iovas, list) {
		if ((domain == iovm_map->domain) && (prop == iovm_map->prop)) {
			mutex_unlock(&buffer->lock);
			atomic_inc(&iovm_map->mapcnt);
			return iovm_map->iova;
		}
	}

	iovm_map = ion_buffer_iova_create(buffer, attachment->dev,
					  direction, prop);
	if (IS_ERR(iovm_map)) {
		mutex_unlock(&buffer->lock);
		return PTR_ERR(iovm_map);
	}

	list_add_tail(&iovm_map->list, &buffer->iovas);

	mutex_unlock(&buffer->lock);

	return iovm_map->iova;
}

/* unmapping is deferred until buffer is freed for performance */
void ion_iovmm_unmap(struct dma_buf_attachment *attachment, dma_addr_t iova)
{
	struct ion_buffer *buffer = attachment->dmabuf->priv;
	struct ion_iovm_map *iovm_map;
	struct iommu_domain *domain;

	domain = get_domain_from_dev(attachment->dev);
	if (!domain) {
		dev_err(attachment->dev, "%s: no iommu domain\n", __func__);
		return;
	}

	mutex_lock(&buffer->lock);
	list_for_each_entry(iovm_map, &buffer->iovas, list) {
		if ((domain == iovm_map->domain) && (iova == iovm_map->iova)) {
			mutex_unlock(&buffer->lock);
			atomic_dec(&iovm_map->mapcnt);
			return;
		}
	}

	WARN(1, "iova %pad found for %s\n", &iova, dev_name(attachment->dev));
}
