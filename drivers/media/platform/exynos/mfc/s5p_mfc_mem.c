/*
 * drivers/media/platform/exynos/mfc/s5p_mfc_mem.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "s5p_mfc_mem.h"

struct vb2_mem_ops *s5p_mfc_mem_ops(void)
{
	return (struct vb2_mem_ops *)&vb2_dma_sg_memops;
}

void s5p_mfc_mem_clean(struct s5p_mfc_dev *dev,
			struct s5p_mfc_special_buf *special_buf,
			off_t offset, size_t size)
{
	__dma_map_area(special_buf->vaddr + offset, size, DMA_TO_DEVICE);
	return;
}

void s5p_mfc_mem_invalidate(struct s5p_mfc_dev *dev,
			struct s5p_mfc_special_buf *special_buf,
			off_t offset, size_t size)
{
	__dma_map_area(special_buf->vaddr + offset, size, DMA_FROM_DEVICE);
	return;
}

int s5p_mfc_mem_get_user_shared_handle(struct s5p_mfc_ctx *ctx,
	struct mfc_user_shared_handle *handle)
{
	int ret = 0;

	handle->dma_buf = dma_buf_get(handle->fd);
	if (IS_ERR(handle->dma_buf)) {
		mfc_err_ctx("Failed to import fd\n");
		ret = PTR_ERR(handle->dma_buf);
		goto import_dma_fail;
	}

	handle->vaddr = dma_buf_vmap(handle->dma_buf);
	if (handle->vaddr == NULL) {
		mfc_err_ctx("Failed to get kernel virtual address\n");
		ret = -EINVAL;
		goto map_kernel_fail;
	}

	mfc_debug(2, "User Handle: fd = %d, virtual addr = 0x%p\n",
				handle->fd, handle->vaddr);

	return 0;

map_kernel_fail:
	handle->vaddr = NULL;
	dma_buf_put(handle->dma_buf);

import_dma_fail:
	handle->dma_buf = NULL;
	handle->fd = -1;
	return ret;
}

void s5p_mfc_mem_cleanup_user_shared_handle(struct s5p_mfc_ctx *ctx,
		struct mfc_user_shared_handle *handle)
{
	if (handle->vaddr)
		dma_buf_vunmap(handle->dma_buf, handle->vaddr);
	if (handle->dma_buf)
		dma_buf_put(handle->dma_buf);

	handle->dma_buf = NULL;
	handle->vaddr = NULL;
	handle->fd = -1;
}

int s5p_mfc_mem_ion_alloc(struct s5p_mfc_dev *dev,
		struct s5p_mfc_special_buf *special_buf)
{
	struct s5p_mfc_ctx *ctx = dev->ctx[dev->curr_ctx];
	int flag;
	const char *heapname;

	switch (special_buf->buftype) {
	case MFCBUF_NORMAL:
		heapname = "ion_system_heap";
		flag = 0;
		break;
	case MFCBUF_NORMAL_FW:
		heapname = "vnfw_heap";
		flag = 0;
		break;
	case MFCBUF_DRM:
		heapname = "vframe_heap";
		flag = ION_FLAG_PROTECTED;
		break;
	case MFCBUF_DRM_FW:
		heapname = "vfw_heap";
		flag = ION_FLAG_PROTECTED;
		break;
	default:
		mfc_err_ctx("not supported mfc mem type: %d, heapname: %s\n",
				special_buf->buftype, heapname);
		return -EINVAL;
	}
	special_buf->dma_buf =
			ion_alloc_dmabuf(heapname, special_buf->size, flag);
	if (IS_ERR(special_buf->dma_buf)) {
		mfc_err_ctx("Failed to allocate buffer (err %ld)\n",
				PTR_ERR(special_buf->dma_buf));
		goto err_ion_alloc;
	}

	special_buf->attachment = dma_buf_attach(special_buf->dma_buf, dev->device);
	if (IS_ERR(special_buf->attachment)) {
		mfc_err_ctx("Failed to get dma_buf_attach (err %ld)\n",
				PTR_ERR(special_buf->attachment));
		goto err_attach;
	}

	special_buf->sgt = dma_buf_map_attachment(special_buf->attachment,
			DMA_BIDIRECTIONAL);
	if (IS_ERR(special_buf->sgt)) {
		mfc_err_ctx("Failed to get sgt (err %ld)\n",
				PTR_ERR(special_buf->sgt));
		goto err_map;
	}

	special_buf->daddr = ion_iovmm_map(special_buf->attachment, 0,
			special_buf->size, DMA_BIDIRECTIONAL, 0);
	if (IS_ERR_VALUE(special_buf->daddr)) {
		mfc_err_ctx("Failed to allocate iova (err 0x%p)\n",
				&special_buf->daddr);
		goto err_iovmm;
	}

	special_buf->vaddr = dma_buf_vmap(special_buf->dma_buf);
	if (IS_ERR(special_buf->vaddr)) {
		mfc_err_ctx("Failed to get vaddr (err 0x%p)\n",
				&special_buf->vaddr);
		goto err_vaddr;
	}

	return 0;
err_vaddr:
	special_buf->vaddr = NULL;
	ion_iovmm_unmap(special_buf->attachment, special_buf->daddr);

err_iovmm:
	special_buf->daddr = 0;
	dma_buf_unmap_attachment(special_buf->attachment, special_buf->sgt,
				 DMA_BIDIRECTIONAL);
err_map:
	special_buf->sgt = NULL;
	dma_buf_detach(special_buf->dma_buf, special_buf->attachment);
err_attach:
	special_buf->attachment = NULL;
	dma_buf_put(special_buf->dma_buf);
err_ion_alloc:
	special_buf->dma_buf = NULL;
	return -ENOMEM;
}

void s5p_mfc_mem_ion_free(struct s5p_mfc_dev *dev,
		struct s5p_mfc_special_buf *special_buf)
{
	if (special_buf->vaddr)
		dma_buf_vunmap(special_buf->dma_buf, special_buf->vaddr);
	if (special_buf->daddr)
		ion_iovmm_unmap(special_buf->attachment, special_buf->daddr);
	if (special_buf->sgt)
		dma_buf_unmap_attachment(special_buf->attachment,
					 special_buf->sgt, DMA_BIDIRECTIONAL);
	if (special_buf->attachment)
		dma_buf_detach(special_buf->dma_buf, special_buf->attachment);
	if (special_buf->dma_buf)
		dma_buf_put(special_buf->dma_buf);

	special_buf->dma_buf = NULL;
	special_buf->attachment = NULL;
	special_buf->sgt = NULL;
	special_buf->daddr = 0;
	special_buf->vaddr = NULL;
}
