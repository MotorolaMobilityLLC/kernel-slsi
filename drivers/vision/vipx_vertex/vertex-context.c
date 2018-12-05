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
#include "vertex-util.h"
#include "vertex-kernel-binary.h"
#include "vertex-context.h"

#define KERNEL_BINARY_DEBUG
#define LOAD_GRAPH_INFO_DEBUG
#define UNLOAD_GRAPH_INFO_DEBUG
#define EXECUTE_DEBUG

static int vertex_context_load_kernel_binary(struct vertex_context *vctx,
		struct vertex_ioc_load_kernel_binary *kernel_bin)
{
	vertex_enter();
	vertex_leave();
	return 0;
}

static int vertex_context_load_graph_info(struct vertex_context *vctx,
		struct vertex_ioc_load_graph_info *ginfo)
{
	vertex_enter();
	vertex_leave();
	return 0;
}

static int vertex_context_unload_graph_info(struct vertex_context *vctx,
		struct vertex_ioc_unload_graph_info *ginfo)
{
	vertex_enter();
	vertex_leave();
	return 0;
}

static int vertex_context_execute_submodel(struct vertex_context *vctx,
		struct vertex_ioc_execute_submodel *execute)
{
	vertex_enter();
	vertex_leave();
	return 0;
}

static const struct vertex_context_ops vertex_context_ops = {
	.load_kernel_binary	= vertex_context_load_kernel_binary,
	.load_graph_info	= vertex_context_load_graph_info,
	.unload_graph_info	= vertex_context_unload_graph_info,
	.execute_submodel	= vertex_context_execute_submodel
};

struct vertex_context *vertex_context_create(struct vertex_core *core)
{
	int ret;
	struct vertex_context *vctx;

	vertex_enter();
	vctx = kzalloc(sizeof(*vctx), GFP_KERNEL);
	if (!vctx) {
		ret = -ENOMEM;
		vertex_err("Failed to alloc context\n");
		goto p_err;
	}
	vctx->core = core;

	vctx->idx = vertex_util_bitmap_get_zero_bit(core->vctx_map,
			VERTEX_MAX_CONTEXT);
	if (vctx->idx == VERTEX_MAX_CONTEXT) {
		ret = -ENOMEM;
		vertex_err("Failed to get idx of context\n");
		goto p_err_bitmap;
	}

	core->vctx_count++;
	list_add_tail(&vctx->list, &core->vctx_list);

	mutex_init(&vctx->lock);

	vctx->vops = &vertex_context_ops;
	vctx->binary_count = 0;
	INIT_LIST_HEAD(&vctx->binary_list);
	spin_lock_init(&vctx->binary_slock);

	vertex_queue_init(vctx);

	vctx->state = BIT(VERTEX_CONTEXT_OPEN);
	vertex_leave();
	return vctx;
p_err_bitmap:
	kfree(vctx);
p_err:
	return ERR_PTR(ret);
}

void vertex_context_destroy(struct vertex_context *vctx)
{
	struct vertex_core *core;

	vertex_enter();
	core = vctx->core;

	if (vctx->binary_count)
		vertex_kernel_binary_all_remove(vctx);

	mutex_destroy(&vctx->lock);
	list_del(&vctx->list);
	core->vctx_count--;
	vertex_util_bitmap_clear_bit(core->vctx_map, vctx->idx);
	kfree(vctx);
	vertex_leave();
}
