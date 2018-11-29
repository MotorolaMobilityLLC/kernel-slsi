/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <video/videonode.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/videodev2.h>
#include <linux/videodev2_exynos_camera.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/dma-buf.h>
#include <linux/ion_exynos.h>
#include <asm/cacheflush.h>


#include "vipx-config.h"
#include "vipx-memory.h"

#include <linux/exynos_iovmm.h>



extern void *vipx_fw_code_rmem_base;

#if defined(CONFIG_VIDEOBUF2_DMA_SG)
/* vipx vb2 buffer operations */
static inline ulong vipx_vb2_ion_plane_kvaddr(
		struct vipx_vb2_buf *vbuf, u32 plane)

{
	return (ulong)vb2_plane_vaddr(&vbuf->vb.vb2_buf, plane);
}

static inline ulong vipx_vb2_ion_plane_cookie(
		struct vipx_vb2_buf *vbuf, u32 plane)
{
	return (ulong)vb2_plane_cookie(&vbuf->vb.vb2_buf, plane);
}

static dma_addr_t vipx_vb2_ion_plane_dvaddr(
		struct vipx_vb2_buf *vbuf, u32 plane)

{
	return (ulong)vb2_dma_sg_plane_dma_addr(&vbuf->vb.vb2_buf, plane);
}
#if 0
static void vipx_vb2_ion_plane_prepare(struct vipx_vb2_buf *vbuf,
		u32 plane, bool exact)
{
	struct vb2_buffer *vb = &vbuf->vb.vb2_buf;
	enum dma_data_direction dir;
	unsigned long size;
	u32 spare;

	/* skip meta plane */
	spare = vb->num_planes - 1;
	if (plane == spare)
		return;

	dir = V4L2_TYPE_IS_OUTPUT(vb->type) ?
		DMA_TO_DEVICE : DMA_FROM_DEVICE;

	size = exact ?
		vb2_get_plane_payload(vb, plane) : vb2_plane_size(vb, plane);

	vb2_ion_sync_for_device((void *)vb2_plane_cookie(vb, plane), 0,
			size, dir);
}

static void vipx_vb2_ion_plane_finish(struct vipx_vb2_buf *vbuf,
		u32 plane, bool exact)
{
	struct vb2_buffer *vb = &vbuf->vb.vb2_buf;
	enum dma_data_direction dir;
	unsigned long size;
	u32 spare;

	/* skip meta plane */
	spare = vb->num_planes - 1;
	if (plane == spare)
		return;

	dir = V4L2_TYPE_IS_OUTPUT(vb->type) ?
		DMA_TO_DEVICE : DMA_FROM_DEVICE;

	size = exact ?
		vb2_get_plane_payload(vb, plane) : vb2_plane_size(vb, plane);

	vb2_ion_sync_for_cpu((void *)vb2_plane_cookie(vb, plane), 0,
			size, dir);
}

static void vipx_vb2_ion_buf_prepare(struct vipx_vb2_buf *vbuf, bool exact)
{
	struct vb2_buffer *vb = &vbuf->vb.vb2_buf;
	enum dma_data_direction dir;
	unsigned long size;
	u32 plane;
	u32 spare;

	dir = V4L2_TYPE_IS_OUTPUT(vb->type) ?
		DMA_TO_DEVICE : DMA_FROM_DEVICE;

	/* skip meta plane */
	spare = vb->num_planes - 1;

	for (plane = 0; plane < spare; plane++) {
		size = exact ?
			vb2_get_plane_payload(vb, plane) : vb2_plane_size(vb, plane);

		vb2_ion_sync_for_device((void *)vb2_plane_cookie(vb, plane), 0,
				size, dir);
	}
}

static void vipx_vb2_ion_buf_finish(struct vipx_vb2_buf *vbuf, bool exact)
{
	struct vb2_buffer *vb = &vbuf->vb.vb2_buf;
	enum dma_data_direction dir;
	unsigned long size;
	u32 plane;
	u32 spare;

	dir = V4L2_TYPE_IS_OUTPUT(vb->type) ?
		DMA_TO_DEVICE : DMA_FROM_DEVICE;

	/* skip meta plane */
	spare = vb->num_planes - 1;

	for (plane = 0; plane < spare; plane++) {
		size = exact ?
			vb2_get_plane_payload(vb, plane) : vb2_plane_size(vb, plane);

		vb2_ion_sync_for_cpu((void *)vb2_plane_cookie(vb, plane), 0,
				size, dir);
	}

	clear_bit(VIPX_VBUF_CACHE_INVALIDATE, &vbuf->cache_state);
}
#endif

const struct vipx_vb2_buf_ops vipx_vb2_buf_ops_ion = {
	.plane_kvaddr	= vipx_vb2_ion_plane_kvaddr,
	.plane_cookie	= vipx_vb2_ion_plane_cookie,
	.plane_dvaddr	= vipx_vb2_ion_plane_dvaddr,
//	.plane_prepare	= vipx_vb2_ion_plane_prepare,
//	.plane_finish	= vipx_vb2_ion_plane_finish,
//	.buf_prepare	= vipx_vb2_ion_buf_prepare,
//	.buf_finish	= vipx_vb2_ion_buf_finish,
};

/* vipx private buffer operations */
static void vipx_ion_free(struct vipx_priv_buf *pbuf)
{
	struct vipx_ion_ctx *alloc_ctx;

	alloc_ctx = pbuf->ctx;
	mutex_lock(&alloc_ctx->lock);
	if (pbuf->iova)
		ion_iovmm_unmap(pbuf->attachment, pbuf->iova);
    if (pbuf->kva)
        dma_buf_vunmap(pbuf->dma_buf, pbuf->kva);
	mutex_unlock(&alloc_ctx->lock);

	dma_buf_unmap_attachment(pbuf->attachment, pbuf->sgt,
				DMA_BIDIRECTIONAL);
	dma_buf_detach(pbuf->dma_buf, pbuf->attachment);
	dma_buf_put(pbuf->dma_buf);

	vfree(pbuf);
}

static void *vipx_ion_kvaddr(struct vipx_priv_buf *pbuf)
{
	if (!pbuf)
		return 0;

	if (!pbuf->kva)
		pbuf->kva = dma_buf_vmap(pbuf->dma_buf);

	return pbuf->kva;

}

static dma_addr_t vipx_ion_dvaddr(struct vipx_priv_buf *pbuf)
{
	dma_addr_t dva = 0;

	if (!pbuf)
		return -EINVAL;

	if (pbuf->iova == 0)
		return -EINVAL;

	dva = pbuf->iova;

	return (ulong)dva;
}

static phys_addr_t vipx_ion_phaddr(struct vipx_priv_buf *pbuf)
{
	/* PHCONTIG option is not supported in ION */
	return 0;
}

static void vipx_ion_sync_for_device(struct vipx_priv_buf *pbuf,
		off_t offset, size_t size, enum dma_data_direction dir)
{
//	struct vipx_ion_ctx *alloc_ctx = pbuf->ctx;

	if (pbuf->kva) {
		BUG_ON((offset < 0) || (offset > pbuf->size));
		BUG_ON((offset + size) < size);
		BUG_ON((size > pbuf->size) || ((offset + size) > pbuf->size));

		__dma_map_area(pbuf->kva + offset, size, dir);
	}
}

static void vipx_ion_sync_for_cpu(struct vipx_priv_buf *pbuf,
		off_t offset, size_t size, enum dma_data_direction dir)
{
//	struct vipx_ion_ctx *alloc_ctx = pbuf->ctx;

	if (pbuf->kva) {
		BUG_ON((offset < 0) || (offset > pbuf->size));
		BUG_ON((offset + size) < size);
		BUG_ON((size > pbuf->size) || ((offset + size) > pbuf->size));

		__dma_unmap_area(pbuf->kva + offset, size, dir);
	}
}

const struct vipx_priv_buf_ops vipx_priv_buf_ops_ion = {
	.free			= vipx_ion_free,
	.kvaddr			= vipx_ion_kvaddr,
	.dvaddr			= vipx_ion_dvaddr,
	.phaddr			= vipx_ion_phaddr,
	.sync_for_device	= vipx_ion_sync_for_device,
	.sync_for_cpu		= vipx_ion_sync_for_cpu,
};

/* vipx memory operations */
static void *vipx_ion_init(struct device *dev)
{
	struct vipx_ion_ctx *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	ctx->dev = dev;
    ctx->alignment = SZ_4K;
    ctx->flags = ION_FLAG_CACHED;
    mutex_init(&ctx->lock);

	return ctx;

}

static void vipx_ion_deinit(void *ctx)
{
	struct vipx_ion_ctx *alloc_ctx = ctx;

	mutex_destroy(&alloc_ctx->lock);
	kfree(alloc_ctx);
}

static struct vipx_priv_buf *vipx_ion_alloc(void *ctx,
		size_t size, size_t align)
{
	struct vipx_ion_ctx *alloc_ctx = ctx;
	struct vipx_priv_buf *buf;
    const char *heapname = "ion_system_heap";
	int ret = 0;

	buf = vzalloc(sizeof(*buf));
	if (!buf)
		return ERR_PTR(-ENOMEM);

	size = PAGE_ALIGN(size);

	buf->dma_buf = ion_alloc_dmabuf(heapname, size, ION_FLAG_NON_CACHED);
	//buf->dma_buf = ion_alloc_dmabuf(heapname, size, alloc_ctx->flags);
	if (IS_ERR(buf->dma_buf)) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	buf->attachment = dma_buf_attach(buf->dma_buf, alloc_ctx->dev);
	if (IS_ERR(buf->attachment)) {
		ret = PTR_ERR(buf->attachment);
		goto err_attach;
	}

	buf->sgt = dma_buf_map_attachment(
				buf->attachment, DMA_BIDIRECTIONAL);
	if (IS_ERR(buf->sgt)) {
		ret = PTR_ERR(buf->sgt);
		goto err_map_dmabuf;
	}


	buf->ctx = alloc_ctx;
	buf->size = size;
	buf->direction = DMA_BIDIRECTIONAL;
	buf->ops = &vipx_priv_buf_ops_ion;

	mutex_lock(&alloc_ctx->lock);
	buf->iova = ion_iovmm_map(buf->attachment, 0,
				   buf->size, buf->direction, 0);
	if (IS_ERR_VALUE(buf->iova)) {
		ret = (int)buf->iova;
		mutex_unlock(&alloc_ctx->lock);
		goto err_ion_map_io;
	}
	mutex_unlock(&alloc_ctx->lock);

	return buf;

err_ion_map_io:
	dma_buf_unmap_attachment(buf->attachment, buf->sgt,
				DMA_BIDIRECTIONAL);
err_map_dmabuf:
	dma_buf_detach(buf->dma_buf, buf->attachment);
err_attach:
	dma_buf_put(buf->dma_buf);
err_alloc:
	vfree(buf);


	pr_err("%s: Error occured while allocating\n", __func__);
	return ERR_PTR(ret);
}

static int vipx_ion_resume(void *ctx)
{
	struct vipx_ion_ctx *alloc_ctx = ctx;
	int ret = 0;

	if (!alloc_ctx)
		return -ENOENT;

	mutex_lock(&alloc_ctx->lock);
	if (alloc_ctx->iommu_active_cnt == 0)
		ret = iovmm_activate(alloc_ctx->dev);
	if (!ret)
		alloc_ctx->iommu_active_cnt++;
	mutex_unlock(&alloc_ctx->lock);

	return ret;
}

static void vipx_ion_suspend(void *ctx)
{
	struct vipx_ion_ctx *alloc_ctx = ctx;

	if (!alloc_ctx)
		return;

	mutex_lock(&alloc_ctx->lock);
	BUG_ON(alloc_ctx->iommu_active_cnt == 0);

	if (--alloc_ctx->iommu_active_cnt == 0)
		iovmm_deactivate(alloc_ctx->dev);
	mutex_unlock(&alloc_ctx->lock);
}

const struct vipx_mem_ops vipx_mem_ops_ion = {
	.init			= vipx_ion_init,
	.cleanup		= vipx_ion_deinit,
	.resume			= vipx_ion_resume,
	.suspend		= vipx_ion_suspend,
	.alloc			= vipx_ion_alloc,
};
#endif

/* vipx private buffer operations */
static void vipx_km_free(struct vipx_priv_buf *pbuf)
{
	kfree(pbuf->kvaddr);
	kfree(pbuf);
}

static void *vipx_km_kvaddr(struct vipx_priv_buf *pbuf)
{
	if (!pbuf)
		return 0;

	return pbuf->kvaddr;
}

static phys_addr_t vipx_km_phaddr(struct vipx_priv_buf *pbuf)
{
	phys_addr_t pa = 0;

	if (!pbuf)
		return 0;

	pa = virt_to_phys(pbuf->kvaddr);

	return pa;
}
const struct vipx_priv_buf_ops vipx_priv_buf_ops_km = {
	.free			= vipx_km_free,
	.kvaddr			= vipx_km_kvaddr,
	.phaddr			= vipx_km_phaddr,
};

static struct vipx_priv_buf *vipx_kmalloc(size_t size, size_t align)
{
	struct vipx_priv_buf *buf = NULL;
	int ret = 0;

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	buf->kvaddr = kzalloc(size, GFP_KERNEL);
	if (!buf->kvaddr) {
		ret = -ENOMEM;
		goto err_priv_alloc;
	}

	buf->size = size;
	buf->align = align;
	buf->ops = &vipx_priv_buf_ops_km;

	return buf;

err_priv_alloc:
	kfree(buf);

	return ERR_PTR(ret);
}

int vipx_memory_probe(struct vipx_memory *mem, struct device *dev)
{
	u32 ret = 0;

#if defined(CONFIG_VIDEOBUF2_DMA_SG)
	mem->vipx_mem_ops = &vipx_mem_ops_ion;
	mem->vb2_mem_ops = &vb2_dma_sg_memops;
	mem->vipx_vb2_buf_ops = &vipx_vb2_buf_ops_ion;
	mem->kmalloc = &vipx_kmalloc;
#endif

	mem->dev = dev;
	mem->iommu_domain = get_domain_from_dev(dev);

	mem->default_ctx = CALL_PTR_MEMOP(mem, init, dev);
	if (IS_ERR_OR_NULL(mem->default_ctx)) {
		if (IS_ERR(mem->default_ctx))
			ret = PTR_ERR(mem->default_ctx);
		else
			ret = -EINVAL;
		goto p_err;
	}

p_err:
	probe_info("%s():%d\n", __func__, ret);
	return 0;
}

dma_addr_t vipx_allocate_heap(struct vipx_memory *mem, u32 size)
{
	int ret = 0;
	struct vipx_minfo *minfo;

	BUG_ON(!mem);

	minfo = &mem->info;

	if (size != 0) {
		minfo->pb_heap = vipx_ion_alloc(mem->default_ctx, size, SZ_4K);
		if (IS_ERR_OR_NULL(minfo->pb_heap)) {
			vipx_err("failed to allocate buffer for VIPX_CM7_HEAP_SIZE");
			ret = -ENOMEM;
		}
#ifdef DUMP_DEBUG_LOG_REGION
		minfo->kvaddr_heap = vipx_ion_kvaddr(minfo->pb_heap);
#endif
		minfo->dvaddr_heap = vipx_ion_dvaddr(minfo->pb_heap);
		vipx_info("heap kva(0x%p), dva(0x%x), size(%ld)\n",
			minfo->kvaddr_heap, (u32)minfo->dvaddr_heap, minfo->pb_heap->size);
	}
	return minfo->dvaddr_heap;

}

void vipx_free_heap(struct vipx_memory *mem, struct vipx_binary *binary, u32 heap_size)
{
	struct vipx_minfo *minfo;
#ifdef DUMP_DEBUG_LOG_REGION
	int ret;
#endif
	BUG_ON(!mem);

	minfo = &mem->info;
#ifdef DUMP_DEBUG_LOG_REGION

	vipx_info("heap kva %p\n", minfo->kvaddr_heap);

	if (minfo->pb_heap) {
		if (!IS_ERR_OR_NULL(minfo->kvaddr_heap)) {
			ret = vipx_binary_write(binary, VIPX_FW_PATH1, "vipx_heap.bin", minfo->kvaddr_heap, heap_size);
			if (ret)
				vipx_err("vipx_binary_write is fail(%d)\n", ret);
		}
	}
#endif

	vipx_info("Free heap\n");
	if (minfo->pb_heap) {
		vipx_ion_free(minfo->pb_heap);
		minfo->pb_heap = 0;
		minfo->kvaddr_heap = 0;
		minfo->dvaddr_heap = 0;
	}
}

int vipx_memory_open(struct vipx_memory *mem)
{
	int ret = 0;
	struct vipx_minfo *minfo;

	BUG_ON(!mem);

	minfo = &mem->info;

	/* TODO */
	if (VIPX_CM7_DRAM_BIN_SIZE != 0) {
#if 1
		minfo->paddr_fw = VIPX_IOVA_DRAM_FIRMWARE;
		minfo->kvaddr_fw = vipx_fw_code_rmem_base;
		minfo->dvaddr_fw = VIPX_IOVA_DRAM_FIRMWARE;
		iommu_map(mem->iommu_domain, VIPX_IOVA_DRAM_FIRMWARE, minfo->paddr_fw,
				VIPX_CM7_DRAM_BIN_SIZE, 0);
		//memset(minfo->kvaddr_fw, 0x0, VIPX_CM7_DRAM_BIN_SIZE);
#else
		minfo->kvaddr_fw = dmam_alloc_coherent(mem->dev, VIPX_CM7_DRAM_BIN_SIZE,
				&minfo->paddr_fw, GFP_KERNEL);
		if (IS_ERR_OR_NULL(minfo->kvaddr_fw)) {
			vipx_err("Failed to allocate coherent memory: %ld\n",
					PTR_ERR(minfo->kvaddr_fw));
			ret = PTR_ERR(minfo->kvaddr_fw);
			goto p_err;
		}
		vipx_info("%s(%pa) is mapped on %p with size of %d\n",
				"vipx_firmware", &minfo->paddr_fw, minfo->kvaddr_fw,
				VIPX_CM7_DRAM_BIN_SIZE);

		minfo->dvaddr_fw = VIPX_IOVA_DRAM_FIRMWARE;

		memset(minfo->kvaddr_fw, 0x0, VIPX_CM7_DRAM_BIN_SIZE);
		iommu_map(mem->iommu_domain, VIPX_IOVA_DRAM_FIRMWARE, minfo->paddr_fw,
				VIPX_CM7_DRAM_BIN_SIZE, 0);
#endif
	}

	if (VIPX_MBOX_SIZE != 0) {
		minfo->kvaddr_mbox = dmam_alloc_coherent(mem->dev, VIPX_MBOX_SIZE,
				&minfo->paddr_mbox, GFP_KERNEL);
		if (IS_ERR_OR_NULL(minfo->kvaddr_mbox)) {
			vipx_err("Failed to allocate coherent memory: %ld\n",
					PTR_ERR(minfo->kvaddr_mbox));
			ret = PTR_ERR(minfo->kvaddr_mbox);
			goto p_err;
		}
		vipx_info("%s(%pa) is mapped on %p with size of %d\n",
				"vipx_mbox", &minfo->paddr_mbox, minfo->kvaddr_mbox,
				VIPX_MBOX_SIZE);

		minfo->dvaddr_mbox = VIPX_IOVA_DRAM_MBOX;

		//memset(minfo->kvaddr_mbox, 0x0, VIPX_MBOX_SIZE);
		iommu_map(mem->iommu_domain, VIPX_IOVA_DRAM_MBOX, minfo->paddr_mbox,
				VIPX_MBOX_SIZE, 0);
	}

	if (VIPX_DEBUG_SIZE != 0) {
		minfo->pb_debug = vipx_ion_alloc(mem->default_ctx, VIPX_DEBUG_SIZE, SZ_4K);
		if (IS_ERR_OR_NULL(minfo->pb_debug)) {
			vipx_err("failed to allocate buffer for VIPX_DEBUG_SIZE");
			ret = -ENOMEM;
		}
		minfo->kvaddr_debug = vipx_ion_kvaddr(minfo->pb_debug);
		minfo->dvaddr_debug = vipx_ion_dvaddr(minfo->pb_debug);
		vipx_info("debug kva(0x%p), dva(0x%x), size(%ld)\n", minfo->kvaddr_debug, (u32)minfo->dvaddr_debug, minfo->pb_debug->size);
	}

p_err:
	return ret;
}

int vipx_memory_close(struct vipx_memory *mem)
{
	int ret = 0;
	struct vipx_minfo *minfo;

	BUG_ON(!mem);

	minfo = &mem->info;

	iommu_unmap(mem->iommu_domain, VIPX_IOVA_DRAM_FIRMWARE, VIPX_CM7_DRAM_BIN_SIZE);

#if 0
	if (!IS_ERR_OR_NULL(minfo->kvaddr_fw)) {
		dmam_free_coherent(mem->dev, VIPX_CM7_DRAM_BIN_SIZE, minfo->kvaddr_fw,
				minfo->paddr_fw);
	}
#endif

	iommu_unmap(mem->iommu_domain, VIPX_IOVA_DRAM_MBOX, VIPX_MBOX_SIZE);

	if (!IS_ERR_OR_NULL(minfo->kvaddr_mbox)) {
		dmam_free_coherent(mem->dev, VIPX_MBOX_SIZE, minfo->kvaddr_mbox,
				minfo->paddr_mbox);
	}

	if (minfo->pb_debug) {
		vipx_ion_free(minfo->pb_debug);
		minfo->pb_debug = 0;
		minfo->kvaddr_debug = 0;
		minfo->dvaddr_debug = 0;
	}

	if (minfo->pb_heap) {
		vipx_ion_free(minfo->pb_heap);
		minfo->pb_heap = 0;
		minfo->kvaddr_heap = 0;
		minfo->dvaddr_heap = 0;
	}

	return ret;
}
