/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "vertex-log.h"
#include "vertex-kernel-binary.h"

int vertex_kernel_binary_add(struct vertex_context *vctx, unsigned int id,
		int fd, unsigned int size)
{
	int ret;
	struct vertex_kernel_binary *kbin;
	struct vertex_memory *mem;
	unsigned long flags;

	vertex_enter();
	kbin = kzalloc(sizeof(*kbin), GFP_KERNEL);
	if (!kbin) {
		ret = -ENOMEM;
		vertex_err("Failed to alloc kernel binary\n");
		goto p_err;
	}
	kbin->global_id = id;
	kbin->buffer.m.fd = fd;
	kbin->buffer.size = size;

	mem = &vctx->core->system->memory;
	ret = mem->mops->map_dmabuf(mem, &kbin->buffer);
	if (ret)
		goto p_err_map;

	spin_lock_irqsave(&vctx->binary_slock, flags);
	vctx->binary_count++;
	list_add_tail(&kbin->list, &vctx->binary_list);
	spin_unlock_irqrestore(&vctx->binary_slock, flags);

	kbin->vctx = vctx;
	vertex_leave();
	return 0;
p_err_map:
	kfree(kbin);
p_err:
	return ret;
}

void vertex_kernel_binary_remove(struct vertex_kernel_binary *kbin)
{
	struct vertex_context *vctx;
	unsigned long flags;
	struct vertex_memory *mem;

	vertex_enter();
	vctx = kbin->vctx;

	spin_lock_irqsave(&vctx->binary_slock, flags);
	list_del(&kbin->list);
	vctx->binary_count--;
	spin_unlock_irqrestore(&vctx->binary_slock, flags);

	mem = &vctx->core->system->memory;
	mem->mops->unmap_dmabuf(mem, &kbin->buffer);

	kfree(kbin);

	vertex_leave();
}

void vertex_kernel_binary_all_remove(struct vertex_context *vctx)
{
	struct vertex_kernel_binary *kbin, *temp;

	vertex_enter();
	list_for_each_entry_safe(kbin, temp, &vctx->binary_list, list) {
		vertex_kernel_binary_remove(kbin);
	}
	vertex_leave();
}
