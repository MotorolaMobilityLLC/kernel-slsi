/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <asm/cacheflush.h>
#include <linux/ion_exynos.h>
#include <linux/exynos_iovmm.h>
#include <linux/of_reserved_mem.h>

#include "vertex-log.h"
#include "vertex-mailbox.h"
#include "vertex-system.h"
#include "vertex-memory.h"

/* TODO check */
void vertex_memory_sync_for_device(struct vertex_priv_mem *pmem,
		off_t offset, size_t size, enum dma_data_direction dir)
{
	vertex_enter();
	if (pmem->kvaddr) {
		WARN_ON((offset < 0) || (offset > pmem->size));
		WARN_ON((offset + size) < size);
		WARN_ON((size > pmem->size) || ((offset + size) > pmem->size));

		__dma_map_area(pmem->kvaddr + offset, size, dir);
	}
	vertex_leave();
}

void vertex_memory_sync_for_cpu(struct vertex_priv_mem *pmem,
		off_t offset, size_t size, enum dma_data_direction dir)
{
	vertex_enter();
	if (pmem->kvaddr) {
		WARN_ON((offset < 0) || (offset > pmem->size));
		WARN_ON((offset + size) < size);
		WARN_ON((size > pmem->size) || ((offset + size) > pmem->size));

		__dma_unmap_area(pmem->kvaddr + offset, size, dir);
	}
	vertex_leave();
}

static int vertex_memory_map_dmabuf(struct vertex_memory *mem,
		struct vertex_buffer *buf)
{
	int ret;
	struct dma_buf *dbuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	dma_addr_t dvaddr;
	void *kvaddr;

	vertex_enter();
	if (buf->m.fd <= 0) {
		ret = -EINVAL;
		vertex_err("fd(%d) is invalid\n", buf->m.fd);
		goto p_err;
	}

	dbuf = dma_buf_get(buf->m.fd);
	if (IS_ERR(dbuf)) {
		ret = PTR_ERR(dbuf);
		vertex_err("dma_buf is invalid (%d/%d)\n", buf->m.fd, ret);
		goto p_err;
	}
	buf->dbuf = dbuf;

	buf->aligned_size = PAGE_ALIGN(buf->size);
	if (buf->aligned_size > dbuf->size) {
		ret = -EINVAL;
		vertex_err("size is invalid (%zu/%zu/%zu)\n",
				buf->size, buf->aligned_size, dbuf->size);
		goto p_err_size;
	}

	attachment = dma_buf_attach(dbuf, mem->dev);
	if (IS_ERR(attachment)) {
		ret = PTR_ERR(attachment);
		vertex_err("failed to attach dma-buf (%d)\n", ret);
		goto p_err_attach;
	}
	buf->attachment = attachment;

	sgt = dma_buf_map_attachment(attachment, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		vertex_err("failed to map attachment (%d)\n", ret);
		goto p_err_map_attach;
	}
	buf->sgt = sgt;

	dvaddr = ion_iovmm_map(attachment, 0, buf->aligned_size,
			DMA_BIDIRECTIONAL, 0);
	if (IS_ERR_VALUE(dvaddr)) {
		ret = (int)dvaddr;
		vertex_err("failed to map iova (%d)\n", ret);
		goto p_err_iova;
	}
	buf->dvaddr = dvaddr;

	/* TODO: check sync */
	kvaddr = dma_buf_vmap(dbuf);
	if (IS_ERR(kvaddr)) {
		ret = PTR_ERR(kvaddr);
		vertex_err("failed to map kvaddr (%d)\n", ret);
		goto p_err_kva;
	}
	buf->kvaddr = kvaddr;

	vertex_leave();
	return 0;
p_err_kva:
	ion_iovmm_unmap(attachment, dvaddr);
p_err_iova:
	dma_buf_unmap_attachment(attachment, sgt, DMA_BIDIRECTIONAL);
p_err_map_attach:
	dma_buf_detach(dbuf, attachment);
p_err_attach:
p_err_size:
	dma_buf_put(dbuf);
p_err:
	return ret;
}

static int vertex_memory_unmap_dmabuf(struct vertex_memory *mem,
		struct vertex_buffer *buf)
{
	vertex_enter();
	dma_buf_vunmap(buf->dbuf, buf->kvaddr);
	ion_iovmm_unmap(buf->attachment, buf->dvaddr);
	dma_buf_unmap_attachment(buf->attachment, buf->sgt, DMA_BIDIRECTIONAL);
	dma_buf_detach(buf->dbuf, buf->attachment);
	dma_buf_put(buf->dbuf);
	vertex_leave();
	return 0;
}

static int vertex_memory_map_userptr(struct vertex_memory *mem,
		struct vertex_buffer *buf)
{
	vertex_enter();
	vertex_leave();
	return -EINVAL;
}

static int vertex_memory_unmap_userptr(struct vertex_memory *mem,
		struct vertex_buffer *buf)
{
	vertex_enter();
	vertex_leave();
	return -EINVAL;
}

const struct vertex_memory_ops vertex_memory_ops = {
	.map_dmabuf	= vertex_memory_map_dmabuf,
	.unmap_dmabuf	= vertex_memory_unmap_dmabuf,
	.map_userptr	= vertex_memory_map_userptr,
	.unmap_userptr	= vertex_memory_unmap_userptr
};

static int __vertex_memory_alloc(struct vertex_memory *mem,
		struct vertex_priv_mem *pmem)
{
	int ret;
	const char *heap_name = "ion_system_heap";
	struct dma_buf *dbuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	dma_addr_t dvaddr;
	void *kvaddr;

	vertex_enter();
	dbuf = ion_alloc_dmabuf(heap_name, pmem->size, pmem->flags);
	if (IS_ERR(dbuf)) {
		ret = PTR_ERR(dbuf);
		vertex_err("Failed to allocate dma_buf (%d) [%s]\n",
				ret, pmem->name);
		goto p_err_alloc;
	}
	pmem->dbuf = dbuf;

	attachment = dma_buf_attach(dbuf, mem->dev);
	if (IS_ERR(attachment)) {
		ret = PTR_ERR(attachment);
		vertex_err("Failed to attach dma_buf (%d) [%s]\n",
				ret, pmem->name);
		goto p_err_attach;
	}
	pmem->attachment = attachment;

	sgt = dma_buf_map_attachment(attachment, pmem->direction);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		vertex_err("Failed to map attachment (%d) [%s]\n",
				ret, pmem->name);
		goto p_err_map_attachment;
	}
	pmem->sgt = sgt;

	dvaddr = ion_iovmm_map(attachment, 0, pmem->size, pmem->direction, 0);
	if (IS_ERR_VALUE(dvaddr)) {
		ret = (int)dvaddr;
		vertex_err("Failed to map dvaddr (%d) [%s]\n", ret, pmem->name);
		goto p_err_map_dva;
	}
	pmem->dvaddr = dvaddr;

	if (pmem->kmap) {
		kvaddr = dma_buf_vmap(dbuf);
		if (IS_ERR(kvaddr)) {
			ret = PTR_ERR(kvaddr);
			vertex_err("Failed to map kvaddr (%d) [%s]\n",
					ret, pmem->name);
			goto p_err_kmap;
		}
		pmem->kvaddr = kvaddr;
	}

	vertex_leave();
	return 0;
p_err_kmap:
	ion_iovmm_unmap(attachment, dvaddr);
p_err_map_dva:
	dma_buf_unmap_attachment(attachment, sgt, pmem->direction);
p_err_map_attachment:
	dma_buf_detach(dbuf, attachment);
p_err_attach:
	dma_buf_put(dbuf);
p_err_alloc:
	return ret;
}

static void __vertex_memory_free(struct vertex_memory *mem,
		struct vertex_priv_mem *pmem)
{
	vertex_enter();
	if (pmem->kmap)
		dma_buf_vunmap(pmem->dbuf, pmem->kvaddr);

	ion_iovmm_unmap(pmem->attachment, pmem->dvaddr);
	dma_buf_unmap_attachment(pmem->attachment, pmem->sgt, pmem->direction);
	dma_buf_detach(pmem->dbuf, pmem->attachment);
	dma_buf_put(pmem->dbuf);
	dma_buf_put(pmem->dbuf);
	vertex_leave();
}

dma_addr_t vertex_memory_allocate_heap(struct vertex_memory *mem,
		int id, size_t size)
{
	int ret;
	struct vertex_priv_mem *heap;

	vertex_enter();
	heap = &mem->heap;

	snprintf(heap->name, VERTEX_PRIV_MEM_NAME_LEN, "vertex_heap_%u", id);
	heap->size = PAGE_ALIGN(size);
	heap->flags = 0;
	heap->direction = DMA_BIDIRECTIONAL;
	heap->kmap = false;

	ret = __vertex_memory_alloc(mem, heap);
	if (ret)
		goto p_err;

	vertex_leave();
	return heap->dvaddr;
p_err:
	return ret;
}

void vertex_memory_free_heap(struct vertex_memory *mem)
{
	vertex_enter();
	__vertex_memory_free(mem, &mem->heap);
	vertex_leave();
}

int vertex_memory_open(struct vertex_memory *mem)
{
	int ret;
	struct vertex_priv_mem *fw;
	struct vertex_priv_mem *mbox;
	struct vertex_priv_mem *debug;

	vertex_enter();
	fw = &mem->fw;
	mbox = &mem->mbox;
	debug = &mem->debug;

	ret = iommu_map(mem->domain, fw->dvaddr, fw->paddr, fw->size, 0);
	if (ret) {
		vertex_err("iommu mapping is failed (0x%x/0x%x/%zu)(%d)\n",
				(int)fw->dvaddr, (int)fw->paddr, fw->size, ret);
		goto p_err_map;
	}

	/* TODO check */
	//fw->kvaddr = dmam_alloc_coherent(mem->dev, fw->size, &fw->paddr,
	//		GFP_KERNEL);
	//if (!fw->kvaddr) {
	//	ret = -ENOMEM;
	//	vertex_err("Failed to allocate coherent memory for CM7 DRAM\n");
	//	goto p_err_cm7;
	//}
	//
	//iommu_map(mem->domain, fw->dvaddr, fw->paddr, fw->size, 0);

	ret = __vertex_memory_alloc(mem, mbox);
	if (ret)
		goto p_err_mbox;

	ret = __vertex_memory_alloc(mem, debug);
	if (ret)
		goto p_err_debug;

	vertex_leave();
	return 0;
p_err_debug:
	__vertex_memory_free(mem, mbox);
p_err_mbox:
	iommu_unmap(mem->domain, fw->dvaddr, fw->size);
p_err_map:
	return ret;
}

int vertex_memory_close(struct vertex_memory *mem)
{
	struct vertex_priv_mem *fw;
	struct vertex_priv_mem *mbox;
	struct vertex_priv_mem *debug;

	vertex_enter();
	fw = &mem->fw;
	mbox = &mem->mbox;
	debug = &mem->debug;

	__vertex_memory_free(mem, debug);
	__vertex_memory_free(mem, mbox);

	iommu_unmap(mem->domain, fw->dvaddr, fw->size);
	/* TODO check */
	//dmam_free_coherent(mem->dev, fw->size, fw->kvaddr, fw->paddr);

	vertex_leave();
	return 0;
}

extern struct reserved_mem *vipx_cm7_rmem;

int vertex_memory_probe(struct vertex_system *sys)
{
	struct device *dev;
	struct vertex_memory *mem;
	struct vertex_priv_mem *fw;
	struct vertex_priv_mem *mbox;
	struct vertex_priv_mem *debug;

	vertex_enter();
	dev = sys->dev;
	dma_set_mask(dev, DMA_BIT_MASK(36));

	mem = &sys->memory;
	mem->dev = dev;
	mem->domain = get_domain_from_dev(dev);
	mem->mops = &vertex_memory_ops;

	fw = &mem->fw;
	mbox = &mem->mbox;
	debug = &mem->debug;

	snprintf(fw->name, VERTEX_PRIV_MEM_NAME_LEN, "vertex_cm7_dram_bin");
	fw->size = vipx_cm7_rmem->size;
	fw->kvaddr = phys_to_virt(vipx_cm7_rmem->base);
	fw->paddr = vipx_cm7_rmem->base;
	fw->dvaddr = VERTEX_CM7_DRAM_BIN_DVADDR;

	snprintf(mbox->name, VERTEX_PRIV_MEM_NAME_LEN, "vertex_mbox");
	mbox->size = PAGE_ALIGN(sizeof(struct vertex_mailbox_ctrl));
	mbox->flags = 0;
	mbox->direction = DMA_BIDIRECTIONAL;
	mbox->kmap = true;

	snprintf(debug->name, VERTEX_PRIV_MEM_NAME_LEN, "vertex_debug");
	debug->size = PAGE_ALIGN(VERTEX_DEBUG_SIZE);
	debug->flags = 0;
	debug->direction = DMA_BIDIRECTIONAL;
	/* TODO remove */
	debug->kmap = true;

	vertex_leave();
	return 0;
}

void vertex_memory_remove(struct vertex_memory *mem)
{
	vertex_enter();
	vertex_leave();
}
