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

#include "vipx-log.h"
#include "vipx-mailbox.h"
#include "vipx-system.h"
#include "vipx-memory.h"

/* TODO check */
void vipx_memory_sync_for_device(struct vipx_priv_mem *pmem,
		off_t offset, size_t size, enum dma_data_direction dir)
{
	vipx_enter();
	if (pmem->kvaddr) {
		WARN_ON((offset < 0) || (offset > pmem->size));
		WARN_ON((offset + size) < size);
		WARN_ON((size > pmem->size) || ((offset + size) > pmem->size));

		__dma_map_area(pmem->kvaddr + offset, size, dir);
	}
	vipx_leave();
}

void vipx_memory_sync_for_cpu(struct vipx_priv_mem *pmem,
		off_t offset, size_t size, enum dma_data_direction dir)
{
	vipx_enter();
	if (pmem->kvaddr) {
		WARN_ON((offset < 0) || (offset > pmem->size));
		WARN_ON((offset + size) < size);
		WARN_ON((size > pmem->size) || ((offset + size) > pmem->size));

		__dma_unmap_area(pmem->kvaddr + offset, size, dir);
	}
	vipx_leave();
}

static int vipx_memory_map_dmabuf(struct vipx_memory *mem,
		struct vipx_buffer *buf)
{
	int ret;
	struct dma_buf *dbuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	dma_addr_t dvaddr;
	void *kvaddr;

	vipx_enter();
	if (buf->m.fd <= 0) {
		ret = -EINVAL;
		vipx_err("fd(%d) is invalid\n", buf->m.fd);
		goto p_err;
	}

	dbuf = dma_buf_get(buf->m.fd);
	if (IS_ERR(dbuf)) {
		ret = PTR_ERR(dbuf);
		vipx_err("dma_buf is invalid (%d/%d)\n", buf->m.fd, ret);
		goto p_err;
	}
	buf->dbuf = dbuf;

	buf->aligned_size = PAGE_ALIGN(buf->size);
	if (buf->aligned_size > dbuf->size) {
		ret = -EINVAL;
		vipx_err("size is invalid (%zu/%zu/%zu)\n",
				buf->size, buf->aligned_size, dbuf->size);
		goto p_err_size;
	}

	attachment = dma_buf_attach(dbuf, mem->dev);
	if (IS_ERR(attachment)) {
		ret = PTR_ERR(attachment);
		vipx_err("failed to attach dma-buf (%d)\n", ret);
		goto p_err_attach;
	}
	buf->attachment = attachment;

	sgt = dma_buf_map_attachment(attachment, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		vipx_err("failed to map attachment (%d)\n", ret);
		goto p_err_map_attach;
	}
	buf->sgt = sgt;

	dvaddr = ion_iovmm_map(attachment, 0, buf->aligned_size,
			DMA_BIDIRECTIONAL, 0);
	if (IS_ERR_VALUE(dvaddr)) {
		ret = (int)dvaddr;
		vipx_err("failed to map iova (%d)\n", ret);
		goto p_err_iova;
	}
	buf->dvaddr = dvaddr;

	/* TODO: check sync */
	kvaddr = dma_buf_vmap(dbuf);
	if (IS_ERR(kvaddr)) {
		ret = PTR_ERR(kvaddr);
		vipx_err("failed to map kvaddr (%d)\n", ret);
		goto p_err_kva;
	}
	buf->kvaddr = kvaddr;

	vipx_leave();
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

static int vipx_memory_unmap_dmabuf(struct vipx_memory *mem,
		struct vipx_buffer *buf)
{
	vipx_enter();
	dma_buf_vunmap(buf->dbuf, buf->kvaddr);
	ion_iovmm_unmap(buf->attachment, buf->dvaddr);
	dma_buf_unmap_attachment(buf->attachment, buf->sgt, DMA_BIDIRECTIONAL);
	dma_buf_detach(buf->dbuf, buf->attachment);
	dma_buf_put(buf->dbuf);
	vipx_leave();
	return 0;
}

static int vipx_memory_map_userptr(struct vipx_memory *mem,
		struct vipx_buffer *buf)
{
	vipx_enter();
	vipx_leave();
	return -EINVAL;
}

static int vipx_memory_unmap_userptr(struct vipx_memory *mem,
		struct vipx_buffer *buf)
{
	vipx_enter();
	vipx_leave();
	return -EINVAL;
}

const struct vipx_memory_ops vipx_memory_ops = {
	.map_dmabuf	= vipx_memory_map_dmabuf,
	.unmap_dmabuf	= vipx_memory_unmap_dmabuf,
	.map_userptr	= vipx_memory_map_userptr,
	.unmap_userptr	= vipx_memory_unmap_userptr
};

static int __vipx_memory_alloc(struct vipx_memory *mem,
		struct vipx_priv_mem *pmem)
{
	int ret;
	const char *heap_name = "ion_system_heap";
	struct dma_buf *dbuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	dma_addr_t dvaddr;
	void *kvaddr;

	vipx_enter();
	dbuf = ion_alloc_dmabuf(heap_name, pmem->size, pmem->flags);
	if (IS_ERR(dbuf)) {
		ret = PTR_ERR(dbuf);
		vipx_err("Failed to allocate dma_buf (%d) [%s]\n",
				ret, pmem->name);
		goto p_err_alloc;
	}
	pmem->dbuf = dbuf;

	attachment = dma_buf_attach(dbuf, mem->dev);
	if (IS_ERR(attachment)) {
		ret = PTR_ERR(attachment);
		vipx_err("Failed to attach dma_buf (%d) [%s]\n",
				ret, pmem->name);
		goto p_err_attach;
	}
	pmem->attachment = attachment;

	sgt = dma_buf_map_attachment(attachment, pmem->direction);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		vipx_err("Failed to map attachment (%d) [%s]\n",
				ret, pmem->name);
		goto p_err_map_attachment;
	}
	pmem->sgt = sgt;

	dvaddr = ion_iovmm_map(attachment, 0, pmem->size, pmem->direction, 0);
	if (IS_ERR_VALUE(dvaddr)) {
		ret = (int)dvaddr;
		vipx_err("Failed to map dvaddr (%d) [%s]\n", ret, pmem->name);
		goto p_err_map_dva;
	}
	pmem->dvaddr = dvaddr;

	if (pmem->kmap) {
		kvaddr = dma_buf_vmap(dbuf);
		if (IS_ERR(kvaddr)) {
			ret = PTR_ERR(kvaddr);
			vipx_err("Failed to map kvaddr (%d) [%s]\n",
					ret, pmem->name);
			goto p_err_kmap;
		}
		pmem->kvaddr = kvaddr;
	}

	vipx_leave();
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

static void __vipx_memory_free(struct vipx_memory *mem,
		struct vipx_priv_mem *pmem)
{
	vipx_enter();
	if (pmem->kmap)
		dma_buf_vunmap(pmem->dbuf, pmem->kvaddr);

	ion_iovmm_unmap(pmem->attachment, pmem->dvaddr);
	dma_buf_unmap_attachment(pmem->attachment, pmem->sgt, pmem->direction);
	dma_buf_detach(pmem->dbuf, pmem->attachment);
	dma_buf_put(pmem->dbuf);
	dma_buf_put(pmem->dbuf);
	vipx_leave();
}

dma_addr_t vipx_memory_allocate_heap(struct vipx_memory *mem,
		int id, size_t size)
{
	int ret;
	struct vipx_priv_mem *heap;

	vipx_enter();
	heap = &mem->heap;

	snprintf(heap->name, VIPX_PRIV_MEM_NAME_LEN, "vipx_heap_%u", id);
	heap->size = PAGE_ALIGN(size);
	heap->flags = 0;
	heap->direction = DMA_BIDIRECTIONAL;
	heap->kmap = false;

	ret = __vipx_memory_alloc(mem, heap);
	if (ret)
		goto p_err;

	vipx_leave();
	return heap->dvaddr;
p_err:
	return ret;
}

void vipx_memory_free_heap(struct vipx_memory *mem)
{
	vipx_enter();
	__vipx_memory_free(mem, &mem->heap);
	vipx_leave();
}

int vipx_memory_open(struct vipx_memory *mem)
{
	int ret;
	struct vipx_priv_mem *fw;
	struct vipx_priv_mem *mbox;
	struct vipx_priv_mem *heap;
	struct vipx_priv_mem *debug;

	vipx_enter();
	fw = &mem->fw;
	mbox = &mem->mbox;
	heap = &mem->heap;
	debug = &mem->debug;

	ret = iommu_map(mem->domain, fw->dvaddr, fw->paddr, fw->size, 0);
	if (ret) {
		vipx_err("iommu mapping is failed (0x%x/0x%x/%zu)(%d)\n",
				(int)fw->dvaddr, (int)fw->paddr, fw->size, ret);
		goto p_err_map;
	}

	/* TODO check */
	//fw->kvaddr = dmam_alloc_coherent(mem->dev, fw->size, &fw->paddr,
	//		GFP_KERNEL);
	//if (!fw->kvaddr) {
	//	ret = -ENOMEM;
	//	vipx_err("Failed to allocate coherent memory for CM7 DRAM\n");
	//	goto p_err_cm7;
	//}
	//
	//iommu_map(mem->domain, fw->dvaddr, fw->paddr, fw->size, 0);

	ret = __vipx_memory_alloc(mem, mbox);
	if (ret)
		goto p_err_mbox;

	ret = __vipx_memory_alloc(mem, heap);
	if (ret)
		goto p_err_heap;

	ret = __vipx_memory_alloc(mem, debug);
	if (ret)
		goto p_err_debug;

	vipx_leave();
	return 0;
p_err_debug:
	__vipx_memory_free(mem, heap);
p_err_heap:
	__vipx_memory_free(mem, mbox);
p_err_mbox:
	iommu_unmap(mem->domain, fw->dvaddr, fw->size);
p_err_map:
	return ret;
}

int vipx_memory_close(struct vipx_memory *mem)
{
	struct vipx_priv_mem *fw;
	struct vipx_priv_mem *mbox;
	struct vipx_priv_mem *heap;
	struct vipx_priv_mem *debug;

	vipx_enter();
	fw = &mem->fw;
	mbox = &mem->mbox;
	heap = &mem->heap;
	debug = &mem->debug;

	__vipx_memory_free(mem, debug);
	__vipx_memory_free(mem, heap);
	__vipx_memory_free(mem, mbox);

	iommu_unmap(mem->domain, fw->dvaddr, fw->size);
	/* TODO check */
	//dmam_free_coherent(mem->dev, fw->size, fw->kvaddr, fw->paddr);

	vipx_leave();
	return 0;
}

struct reserved_mem *vipx_cm7_rmem;

static int __init vipx_cm7_rmem_setup(struct reserved_mem *rmem)
{
	vipx_enter();
	vipx_info("base=%pa, size=%pa\n", &rmem->base, &rmem->size);
	vipx_cm7_rmem = rmem;
	vipx_leave();
	return 0;
}

RESERVEDMEM_OF_DECLARE(vipx_cm7_rmem, "exynos,vipx_fw_code_rmem",
		vipx_cm7_rmem_setup);

int vipx_memory_probe(struct vipx_system *sys)
{
	struct device *dev;
	struct vipx_memory *mem;
	struct vipx_priv_mem *fw;
	struct vipx_priv_mem *mbox;
	struct vipx_priv_mem *heap;
	struct vipx_priv_mem *debug;

	vipx_enter();
	dev = sys->dev;
	dma_set_mask(dev, DMA_BIT_MASK(36));

	mem = &sys->memory;
	mem->dev = dev;
	mem->domain = get_domain_from_dev(dev);
	mem->mops = &vipx_memory_ops;

	fw = &mem->fw;
	mbox = &mem->mbox;
	heap = &mem->heap;
	debug = &mem->debug;

	snprintf(fw->name, VIPX_PRIV_MEM_NAME_LEN, "vipx_cm7_dram_bin");
	fw->size = vipx_cm7_rmem->size;
	fw->kvaddr = phys_to_virt(vipx_cm7_rmem->base);
	fw->paddr = vipx_cm7_rmem->base;
	fw->dvaddr = VIPX_CM7_DRAM_BIN_DVADDR;

	snprintf(mbox->name, VIPX_PRIV_MEM_NAME_LEN, "vipx_mbox");
	mbox->size = PAGE_ALIGN(sizeof(struct vipx_mailbox_ctrl));
	mbox->flags = 0;
	mbox->direction = DMA_BIDIRECTIONAL;
	mbox->kmap = true;

	snprintf(heap->name, VIPX_PRIV_MEM_NAME_LEN, "vipx_heap");
	heap->size = PAGE_ALIGN(VIPX_HEAP_SIZE);
	heap->flags = 0;
	heap->direction = DMA_BIDIRECTIONAL;

	snprintf(debug->name, VIPX_PRIV_MEM_NAME_LEN, "vipx_debug");
	debug->size = PAGE_ALIGN(VIPX_DEBUG_SIZE);
	debug->flags = 0;
	debug->direction = DMA_BIDIRECTIONAL;
	/* TODO remove */
	debug->kmap = true;

	vipx_leave();
	return 0;
}

void vipx_memory_remove(struct vipx_memory *mem)
{
	vipx_enter();
	vipx_leave();
}
