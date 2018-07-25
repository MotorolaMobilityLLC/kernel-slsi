/*
 * Samsung Exynos5 SoC series VIPx driver
 *
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef VIPX_MEM_H
#define VIPX_MEM_H

#include <linux/platform_device.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-sg.h>
#include <linux/dma-buf.h>
#include <linux/ion_exynos.h>


#include "vipx-binary.h"

/*
 * TEMP for test
 */
#define VIPX_CM7_DRAM_BIN_SIZE (1024 * 1024 * 3)
#define VIPX_MBOX_SIZE (1024 * 24)
#define VIPX_DEBUG_SIZE (1024 * 1024 * 16)
#define VIPX_CM7_HEAP_SIZE_PREVIEW (1024 * 1024 * 16)
#define VIPX_CM7_HEAP_SIZE_ENF (1024 * 1024 * 64)
#define VIPX_CM7_HEAP_SIZE_CAPTURE (1024 * 1024 * 128)

#define VIPX_IOVA_DRAM_FIRMWARE	0xB8000000
#define VIPX_IOVA_DRAM_MBOX	0xA9000000

struct vipx_vb2_buf;
struct vipx_vb2_buf_ops {
	ulong (*plane_kvaddr)(struct vipx_vb2_buf *vbuf, u32 plane);
	ulong (*plane_cookie)(struct vipx_vb2_buf *vbuf, u32 plane);
	dma_addr_t (*plane_dvaddr)(struct vipx_vb2_buf *vbuf, u32 plane);
	void (*plane_prepare)(struct vipx_vb2_buf *vbuf, u32 plane,
						bool exact);
	void (*plane_finish)(struct vipx_vb2_buf *vbuf, u32 plane,
						bool exact);
	void (*buf_prepare)(struct vipx_vb2_buf *vbuf, bool exact);
	void (*buf_finish)(struct vipx_vb2_buf *vbuf, bool exact);
};

enum vipx_vbuf_cache_state {
	VIPX_VBUF_CACHE_INVALIDATE,
};

struct vipx_vb2_buf {
	struct vb2_v4l2_buffer		vb;
	ulong				kva[VIDEO_MAX_PLANES];
	dma_addr_t			dva[VIDEO_MAX_PLANES];

	/* for cache operation */
	unsigned long			cache_state;
	struct list_head		cache_flush_list;

	const struct vipx_vb2_buf_ops *ops;
};

struct vipx_priv_buf;
struct vipx_priv_buf_ops {
	void (*free)(struct vipx_priv_buf *pbuf);
	void *(*kvaddr)(struct vipx_priv_buf *pbuf);
	dma_addr_t (*dvaddr)(struct vipx_priv_buf *pbuf);
	phys_addr_t (*phaddr)(struct vipx_priv_buf *pbuf);
	void (*sync_for_device)(struct vipx_priv_buf *pbuf,
							off_t offset, size_t size,
							enum dma_data_direction dir);
	void (*sync_for_cpu)(struct vipx_priv_buf *pbuf,
							off_t offset, size_t size,
							enum dma_data_direction dir);
};

struct vipx_priv_buf {
	size_t					size;
	size_t					align;
	void					*ctx;   
	void					*kvaddr;

	const struct vipx_priv_buf_ops	*ops;
	void					*priv;
	struct dma_buf				*dma_buf;
	struct dma_buf_attachment		*attachment;
	enum dma_data_direction			direction;
	void					*kva;
    dma_addr_t              iova;
    struct sg_table        *sgt;
};

#define vb_to_vipx_vb2_buf(x)				\
	container_of(x, struct vipx_vb2_buf, vb)

#define CALL_BUFOP(buf, op, args...)			\
	((buf)->ops->op ? (buf)->ops->op(args) : 0)

#define CALL_PTR_BUFOP(buf, op, args...)		\
	((buf)->ops->op ? (buf)->ops->op(args) : NULL)

#define CALL_VOID_BUFOP(buf, op, args...)	\
	do {									\
		if ((buf)->ops->op)					\
			(buf)->ops->op(args);			\
	} while (0)

#define call_buf_op(buf, op, args...)			\
	((buf)->ops->op ? (buf)->ops->op((buf), args) : 0)

struct vipx_mem_ops {
	void *(*init)(struct device *dev);
	void (*cleanup)(void *ctx);
	int (*resume)(void *ctx);
	void (*suspend)(void *ctx);
	void (*set_cached)(void *ctx, bool cacheable);
	int (*set_alignment)(void *ctx, size_t alignment);
	struct vipx_priv_buf *(*alloc)(void *ctx, size_t size, size_t align);
};

struct vipx_ion_ctx {
	struct device		*dev;
	unsigned long		alignment;
	long			flags;

	/* protects iommu_active_cnt and protected */
	struct mutex		lock;
	int			iommu_active_cnt;
};

struct vipx_minfo {
	struct vipx_priv_buf *pb_debug;
	struct vipx_priv_buf *pb_heap;

	ulong		kvaddr_debug_cnt;

	dma_addr_t	paddr_fw;
	dma_addr_t	dvaddr_fw;
	void		*kvaddr_fw;

	dma_addr_t	paddr_mbox;
	dma_addr_t	dvaddr_mbox;
	void		*kvaddr_mbox;

	dma_addr_t	paddr_debug;
	dma_addr_t	dvaddr_debug;
	void		*kvaddr_debug;

	dma_addr_t	paddr_heap;
	dma_addr_t	dvaddr_heap;
	void		*kvaddr_heap;
};

struct vipx_memory {
	struct device				*dev;
	struct vipx_ion_ctx			*default_ctx;
	struct vipx_ion_ctx			*phcontig_ctx;
	const struct vipx_mem_ops		*vipx_mem_ops;
	const struct vb2_mem_ops		*vb2_mem_ops;
	const struct vipx_vb2_buf_ops		*vipx_vb2_buf_ops;
	struct iommu_domain *iommu_domain;

	struct vipx_minfo			info;
	void					*priv;
	struct vipx_priv_buf *(*kmalloc)(size_t size, size_t align);
};

#define CALL_MEMOP(mem, op, args...)			\
	((mem)->vipx_mem_ops->op ?				\
		(mem)->vipx_mem_ops->op(args) : 0)

#define CALL_PTR_MEMOP(mem, op, args...)		\
	((mem)->vipx_mem_ops->op ?				\
		(mem)->vipx_mem_ops->op(args) : NULL)

#define CALL_VOID_MEMOP(mem, op, args...)		\
	do {										\
		if ((mem)->vipx_mem_ops->op)			\
			(mem)->vipx_mem_ops->op(args);	\
	} while (0)

int vipx_memory_probe(struct vipx_memory *mem, struct device *dev);
int vipx_memory_open(struct vipx_memory *mem);
int vipx_memory_close(struct vipx_memory *mem);
dma_addr_t vipx_allocate_heap(struct vipx_memory *mem, u32 size);
void vipx_free_heap(struct vipx_memory *mem, struct vipx_binary *binary, u32 heap_size);

#endif
