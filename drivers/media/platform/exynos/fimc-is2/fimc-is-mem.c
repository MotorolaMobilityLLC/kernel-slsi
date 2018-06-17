/*
 * Samsung Exynos5 SoC series FIMC-IS driver
 *
 * exynos5 fimc-is core functions
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd
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

#include "fimc-is-core.h"
#include "fimc-is-cmd.h"
#include "fimc-is-regs.h"
#include "fimc-is-err.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0))
#include <linux/exynos_iovmm.h>
#else
#include <plat/iovmm.h>
#endif

#if defined(CONFIG_VIDEOBUF2_DMA_SG)
/* fimc-is vb2 buffer operations */
static inline ulong fimc_is_vb2_ion_plane_kvaddr(
		struct fimc_is_vb2_buf *vbuf, u32 plane)

{
	return (ulong)vb2_plane_vaddr(&vbuf->vb.vb2_buf, plane);
}

static dma_addr_t fimc_is_vb2_ion_plane_dvaddr(
		struct fimc_is_vb2_buf *vbuf, u32 plane)

{
	return (ulong)vb2_dma_sg_plane_dma_addr(&vbuf->vb.vb2_buf, plane);
}

static dma_addr_t fimc_is_bufcon_map(struct fimc_is_vb2_buf *vbuf,
	struct device *dev,
	int idx,
	struct dma_buf *dmabuf)
{
	int ret = 0;

	vbuf->bufs[idx] = dmabuf;

	vbuf->attachment[idx] = dma_buf_attach(dmabuf, dev);
	if (IS_ERR(vbuf->attachment[idx])) {
		ret = PTR_ERR(vbuf->attachment[idx]);
		pr_err("%s: Failed to dma_buf_attach %d\n", __func__, idx);
		goto err_attach;
	}

	vbuf->sgt[idx] = dma_buf_map_attachment(vbuf->attachment[idx], DMA_BIDIRECTIONAL);
	if (IS_ERR(vbuf->sgt[idx])) {
		ret = PTR_ERR(vbuf->sgt[idx]);
		pr_err("%s: Failed to dma_buf_map_attachment %d\n", __func__, idx);
		goto err_map_dmabuf;
	}

	vbuf->dva[idx] = ion_iovmm_map(vbuf->attachment[idx], 0, 0, DMA_BIDIRECTIONAL, 0);
	if (IS_ERR_VALUE(vbuf->dva[idx])) {
		pr_err("%s: Failed to ion_iovmm_map %d\n", __func__, idx);
		ret = -ENOMEM;
		goto err_ion_map_io;
	}

	return vbuf->dva[idx];

err_ion_map_io:
	dma_buf_unmap_attachment(vbuf->attachment[idx], vbuf->sgt[idx], DMA_BIDIRECTIONAL);
err_map_dmabuf:
	dma_buf_detach(dmabuf, vbuf->attachment[idx]);
err_attach:
	vbuf->bufs[idx] = NULL;

	return 0;
}

static void fimc_is_bufcon_unmap(struct fimc_is_vb2_buf *vbuf,
	int idx)
{
	if (!vbuf->bufs[idx])
		return;

	ion_iovmm_unmap(vbuf->attachment[idx], vbuf->dva[idx]);
	vbuf->dva[idx] = 0;

	dma_buf_unmap_attachment(vbuf->attachment[idx], vbuf->sgt[idx], DMA_BIDIRECTIONAL);
	dma_buf_detach(vbuf->bufs[idx], vbuf->attachment[idx]);

	vbuf->bufs[idx] = NULL;
}

const struct fimc_is_vb2_buf_ops fimc_is_vb2_buf_ops_ion = {
	.plane_kvaddr	= fimc_is_vb2_ion_plane_kvaddr,
	.plane_dvaddr	= fimc_is_vb2_ion_plane_dvaddr,
	.bufcon_map	= fimc_is_bufcon_map,
	.bufcon_unmap	= fimc_is_bufcon_unmap,
};

/* fimc-is private buffer operations */
static void fimc_is_ion_free(struct fimc_is_priv_buf *pbuf)
{
	struct fimc_is_ion_ctx *alloc_ctx;

	alloc_ctx = pbuf->ctx;
	mutex_lock(&alloc_ctx->lock);
	if (pbuf->iova)
		ion_iovmm_unmap(pbuf->attachment, pbuf->iova);
	mutex_unlock(&alloc_ctx->lock);

	if (pbuf->kva)
		dma_buf_vunmap(pbuf->dma_buf, pbuf->kva);

	dma_buf_unmap_attachment(pbuf->attachment, pbuf->sgt,
				DMA_BIDIRECTIONAL);
	dma_buf_detach(pbuf->dma_buf, pbuf->attachment);
	dma_buf_put(pbuf->dma_buf);

	vfree(pbuf);
}

static ulong fimc_is_ion_kvaddr(struct fimc_is_priv_buf *pbuf)
{
	if (!pbuf)
		return 0;

	if (!pbuf->kva)
		pbuf->kva = dma_buf_vmap(pbuf->dma_buf);

	return (ulong)pbuf->kva;
}

static dma_addr_t fimc_is_ion_dvaddr(struct fimc_is_priv_buf *pbuf)
{
	dma_addr_t dva = 0;

	if (!pbuf)
		return -EINVAL;

	if (pbuf->iova == 0)
		return -EINVAL;

	dva = pbuf->iova;

	return (ulong)dva;
}

static phys_addr_t fimc_is_ion_phaddr(struct fimc_is_priv_buf *pbuf)
{
	/* PHCONTIG option is not supported in ION */
	return 0;
}

static void fimc_is_ion_sync_for_device(struct fimc_is_priv_buf *pbuf,
		off_t offset, size_t size, enum dma_data_direction dir)
{
	if (pbuf->kva) {
		FIMC_BUG_VOID((offset < 0) || (offset > pbuf->size));
		FIMC_BUG_VOID((offset + size) < size);
		FIMC_BUG_VOID((size > pbuf->size) || ((offset + size) > pbuf->size));

		__dma_map_area(pbuf->kva + offset, size, dir);
	}
}

static void fimc_is_ion_sync_for_cpu(struct fimc_is_priv_buf *pbuf,
		off_t offset, size_t size, enum dma_data_direction dir)
{
	if (pbuf->kva) {
		FIMC_BUG_VOID((offset < 0) || (offset > pbuf->size));
		FIMC_BUG_VOID((offset + size) < size);
		FIMC_BUG_VOID((size > pbuf->size) || ((offset + size) > pbuf->size));

		__dma_unmap_area(pbuf->kva + offset, size, dir);
	}
}

const struct fimc_is_priv_buf_ops fimc_is_priv_buf_ops_ion = {
	.free			= fimc_is_ion_free,
	.kvaddr			= fimc_is_ion_kvaddr,
	.dvaddr			= fimc_is_ion_dvaddr,
	.phaddr			= fimc_is_ion_phaddr,
	.sync_for_device	= fimc_is_ion_sync_for_device,
	.sync_for_cpu		= fimc_is_ion_sync_for_cpu,
};

/* fimc-is memory operations */
static void *fimc_is_ion_init(struct platform_device *pdev)
{
	struct fimc_is_ion_ctx *ctx;

	ctx = vzalloc(sizeof(*ctx));
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	ctx->dev = &pdev->dev;
	ctx->alignment = SZ_4K;
	ctx->flags = ION_FLAG_CACHED;
	mutex_init(&ctx->lock);

	return ctx;

}

static void fimc_is_ion_deinit(void *ctx)
{
	struct fimc_is_ion_ctx *alloc_ctx = ctx;

	mutex_destroy(&alloc_ctx->lock);
	vfree(alloc_ctx);
}

static struct fimc_is_priv_buf *fimc_is_ion_alloc(void *ctx,
		size_t size, size_t align)
{
	struct fimc_is_ion_ctx *alloc_ctx = ctx;
	struct fimc_is_priv_buf *buf;
	const char *heapname = "ion_system_heap";
	int ret = 0;

	buf = vzalloc(sizeof(*buf));
	if (!buf)
		return ERR_PTR(-ENOMEM);

	size = PAGE_ALIGN(size);

	buf->dma_buf = ion_alloc_dmabuf(heapname, size, alloc_ctx->flags);
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
	buf->ops = &fimc_is_priv_buf_ops_ion;

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

static int fimc_is_ion_resume(void *ctx)
{
	struct fimc_is_ion_ctx *alloc_ctx = ctx;
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

static void fimc_is_ion_suspend(void *ctx)
{
	struct fimc_is_ion_ctx *alloc_ctx = ctx;

	if (!alloc_ctx)
		return;

	mutex_lock(&alloc_ctx->lock);
	FIMC_BUG_VOID(alloc_ctx->iommu_active_cnt == 0);

	if (--alloc_ctx->iommu_active_cnt == 0)
		iovmm_deactivate(alloc_ctx->dev);
	mutex_unlock(&alloc_ctx->lock);
}

const struct fimc_is_mem_ops fimc_is_mem_ops_ion = {
	.init			= fimc_is_ion_init,
	.cleanup		= fimc_is_ion_deinit,
	.resume			= fimc_is_ion_resume,
	.suspend		= fimc_is_ion_suspend,
	.alloc			= fimc_is_ion_alloc,
};
#endif

/* fimc-is private buffer operations */
static void fimc_is_km_free(struct fimc_is_priv_buf *pbuf)
{
	kfree(pbuf->kvaddr);
	kfree(pbuf);
}

static ulong fimc_is_km_kvaddr(struct fimc_is_priv_buf *pbuf)
{
	if (!pbuf)
		return 0;

	return (ulong)pbuf->kvaddr;
}

static phys_addr_t fimc_is_km_phaddr(struct fimc_is_priv_buf *pbuf)
{
	phys_addr_t pa = 0;

	if (!pbuf)
		return 0;

	pa = virt_to_phys(pbuf->kvaddr);

	return pa;
}
const struct fimc_is_priv_buf_ops fimc_is_priv_buf_ops_km = {
	.free			= fimc_is_km_free,
	.kvaddr			= fimc_is_km_kvaddr,
	.phaddr			= fimc_is_km_phaddr,
};

static struct fimc_is_priv_buf *fimc_is_kmalloc(size_t size, size_t align)
{
	struct fimc_is_priv_buf *buf = NULL;
	int ret = 0;

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	buf->kvaddr = kzalloc(DEBUG_REGION_SIZE + 0x10, GFP_KERNEL);
	if (!buf->kvaddr) {
		ret = -ENOMEM;
		goto err_priv_alloc;
	}

	buf->size = size;
	buf->align = align;
	buf->ops = &fimc_is_priv_buf_ops_km;

	return buf;

err_priv_alloc:
	kfree(buf);

	return ERR_PTR(ret);
}

int fimc_is_mem_init(struct fimc_is_mem *mem, struct platform_device *pdev)
{
#if defined(CONFIG_VIDEOBUF2_DMA_SG)
	mem->fimc_is_mem_ops = &fimc_is_mem_ops_ion;
	mem->vb2_mem_ops = &vb2_dma_sg_memops;
	mem->fimc_is_vb2_buf_ops = &fimc_is_vb2_buf_ops_ion;
	mem->kmalloc = &fimc_is_kmalloc;
#endif

	mem->default_ctx = CALL_PTR_MEMOP(mem, init, pdev);
	if (IS_ERR_OR_NULL(mem->default_ctx)) {
		if (IS_ERR(mem->default_ctx))
			return PTR_ERR(mem->default_ctx);
		else
			return -EINVAL;
	}

	return 0;
}
