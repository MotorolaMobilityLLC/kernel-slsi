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
#include <linux/exynos_iovmm.h>
#include <asm/cacheflush.h>
#include <linux/idr.h>

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
	else if (device_get_dma_attr(attachment->dev) == DEV_DMA_COHERENT)
		prop |= IOMMU_CACHE;

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

	if (IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION) &&
	    (buffer->flags & ION_FLAG_PROTECTED)) {
		struct ion_buffer_prot_info *prot = buffer->priv_virt;

		if (prot->dma_addr != iova)
			WARN(1, "unmap invalid secure iova %pad for %#x\n",
			     &iova, (int)prot->dma_addr);

		return;
	}

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

	WARN(1, "iova %pad not found for %s\n",
	     &iova, dev_name(attachment->dev));
}

#define MAX_BUFFER_IDS 2048
static DEFINE_IDA(ion_buffer_ida);
static int last_buffer_id;

/*
 * exynos_ion_fixup - do something to ion_device for the Exynos extensions
 */
void exynos_ion_fixup(struct ion_device *idev)
{
	struct device *dev = idev->dev.this_device;

	/*
	 * dma-mapping API only works on dma supported device. dma_map_sg() and
	 * the similarities allocates swiotlb buffers if the dma mask of the
	 * given device is not capable of full access to physical address space.
	 * This forces dma-mapping of ARM64 works as if the given device is full
	 * memory access devices.
	 * See ion_buffer_create() and ion_buffer_destroy().
	 */
	arch_setup_dma_ops(dev, 0x0ULL, 1ULL << 36, NULL, false);
	dev->dma_mask = &dev->coherent_dma_mask;
	dma_set_mask(dev, DMA_BIT_MASK(36));
}

int exynos_ion_alloc_fixup(struct ion_device *idev, struct ion_buffer *buffer)
{
	struct sg_table *table = buffer->sg_table;
	int nents, id;

	id = ida_simple_get(&ion_buffer_ida, last_buffer_id,
			    MAX_BUFFER_IDS, GFP_KERNEL);
	if (id < 0)
		id = ida_simple_get(&ion_buffer_ida, 0, MAX_BUFFER_IDS,
				    GFP_KERNEL);

	if (id < 0) {
		id = MAX_BUFFER_IDS;
		last_buffer_id = 0;
	} else {
		last_buffer_id = id;
	}

	buffer->id = id;

	/* assign dma_addresses to scatter-gather list */
	nents = dma_map_sg_attrs(idev->dev.this_device, table->sgl,
				 table->orig_nents, DMA_TO_DEVICE,
				 DMA_ATTR_SKIP_CPU_SYNC);
	if (nents < table->orig_nents) {
		pr_err("%s: failed dma_map_sg(nents %d)=nents %d\n",
		       __func__, table->orig_nents, nents);
		if (id < MAX_BUFFER_IDS)
			ida_simple_remove(&ion_buffer_ida, id);
		return -ENOMEM;
	}

	return 0;
}

void exynos_ion_free_fixup(struct ion_buffer *buffer)
{
	struct sg_table *table = buffer->sg_table;
	struct ion_iovm_map *iovm_map, *tmp;

	dma_unmap_sg_attrs(buffer->dev->dev.this_device, table->sgl,
			   table->orig_nents, DMA_TO_DEVICE,
			   DMA_ATTR_SKIP_CPU_SYNC);

	list_for_each_entry_safe(iovm_map, tmp, &buffer->iovas, list) {
		iovmm_unmap(iovm_map->dev, iovm_map->iova);
		list_del(&iovm_map->list);
		kfree(iovm_map);
	}

	if (buffer->id < MAX_BUFFER_IDS)
		ida_simple_remove(&ion_buffer_ida, buffer->id);
}

struct sg_table *ion_exynos_map_dma_buf(struct dma_buf_attachment *attachment,
					enum dma_data_direction direction)
{
	struct ion_buffer *buffer = attachment->dmabuf->priv;

	if (ion_buffer_cached(buffer) && direction != DMA_NONE)
		dma_sync_sg_for_device(attachment->dev, buffer->sg_table->sgl,
				       buffer->sg_table->nents, direction);

	return buffer->sg_table;
}

void ion_exynos_unmap_dma_buf(struct dma_buf_attachment *attachment,
			      struct sg_table *table,
			      enum dma_data_direction direction)
{
	struct ion_buffer *buffer = attachment->dmabuf->priv;

	if (ion_buffer_cached(buffer) && direction != DMA_NONE)
		dma_sync_sg_for_cpu(attachment->dev, table->sgl,
				    table->nents, direction);
}

static void exynos_flush_sg(struct device *dev,
			    struct scatterlist *sgl, int nelems)
{
	struct scatterlist *sg;
	int i;
	void *virt;

	for_each_sg(sgl, sg, nelems, i) {
		virt = phys_to_virt(dma_to_phys(dev, sg->dma_address));

		__dma_flush_area(virt, nelems);
	}
}

int ion_exynos_dma_buf_begin_cpu_access(struct dma_buf *dmabuf,
					enum dma_data_direction direction)
{
	struct ion_buffer *buffer = dmabuf->priv;
	void *vaddr;

	if (buffer->heap->ops->map_kernel) {
		mutex_lock(&buffer->lock);
		vaddr = ion_buffer_kmap_get(buffer);
		mutex_unlock(&buffer->lock);
	}

	if (!ion_buffer_cached(buffer))
		return 0;

	mutex_lock(&buffer->lock);
	if (direction == DMA_BIDIRECTIONAL) {
		exynos_flush_sg(buffer->dev->dev.this_device,
				buffer->sg_table->sgl,
				buffer->sg_table->orig_nents);
	} else {
		dma_sync_sg_for_cpu(buffer->dev->dev.this_device,
				    buffer->sg_table->sgl,
				    buffer->sg_table->orig_nents,
				    direction);
	}
	mutex_unlock(&buffer->lock);

	return 0;
}

int ion_exynos_dma_buf_end_cpu_access(struct dma_buf *dmabuf,
				      enum dma_data_direction direction)
{
	struct ion_buffer *buffer = dmabuf->priv;

	if (buffer->heap->ops->map_kernel) {
		mutex_lock(&buffer->lock);
		ion_buffer_kmap_put(buffer);
		mutex_unlock(&buffer->lock);
	}

	if (!ion_buffer_cached(buffer))
		return 0;

	mutex_lock(&buffer->lock);
	if (direction == DMA_BIDIRECTIONAL) {
		exynos_flush_sg(buffer->dev->dev.this_device,
				buffer->sg_table->sgl,
				buffer->sg_table->orig_nents);
	} else {
		dma_sync_sg_for_device(buffer->dev->dev.this_device,
				       buffer->sg_table->sgl,
				       buffer->sg_table->orig_nents,
				       direction);
	}
	mutex_unlock(&buffer->lock);

	return 0;
}
