/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef VIPX_GRAPH_H_
#define VIPX_GRAPH_H_

#include <linux/types.h>

#include "vs4l.h"
#include "vipx-taskmgr.h"
#include "vipx-vertex.h"
#include "vipx-interface.h"

#define VIPX_GRAPH_MAX_VERTEX		20
#define VIPX_GRAPH_MAX_PRIORITY		20
#define VIPX_GRAPH_MAX_INTERMEDIATE	64
#define VIPX_GRAPH_MAX_LEVEL		20
#define VIPX_GRAPH_STOP_TIMEOUT		(3 * HZ)

struct vipx_graph;

enum vipx_graph_state {
	VIPX_GRAPH_STATE_CONFIG,
	VIPX_GRAPH_STATE_HENROLL,
	VIPX_GRAPH_STATE_HMAPPED,
	VIPX_GRAPH_STATE_MMAPPED,
	VIPX_GRAPH_STATE_START,
};

enum vipx_graph_flag {
	VIPX_GRAPH_FLAG_UPDATE_PARAM = VS4L_GRAPH_FLAG_END
};

struct vipx_format {
	u32			format;
	u32			plane;
	u32			width;
	u32			height;
};

struct vipx_format_list {
	u32			count;
	struct vipx_format	*formats;
};

struct vipx_graph_ops {
	int (*control)(struct vipx_graph *graph, struct vipx_task *task);
	int (*request)(struct vipx_graph *graph, struct vipx_task *task);
	int (*process)(struct vipx_graph *graph, struct vipx_task *task);
	int (*cancel)(struct vipx_graph *graph, struct vipx_task *task);
	int (*done)(struct vipx_graph *graph, struct vipx_task *task);
	int (*update_param)(struct vipx_graph *graph, struct vipx_task *task);
};

struct vipx_graph_intermediate {
	u32				buffer_index;
	u32				*buffer;
	void				*handle;
	int				fd;
};

struct vipx_graph {
	u32				idx;
	u32				uid;
	unsigned long			state;
	unsigned long			flags;
	u32				priority;
	struct mutex			local_lock;
	struct mutex			*global_lock;

	/* for debugging */
	u32				input_cnt;
	u32				cancel_cnt;
	u32				done_cnt;
	u32				recent;

	const struct vipx_graph_ops	*gops;

	void				*cookie;
	void				*memory;
	struct vipx_pipe			*pipe;
	struct vipx_vertex_ctx 		vctx;

	struct vipx_format_list		informat_list;
	struct vipx_format_list		otformat_list;

	u32				inhash[VIPX_MAX_TASK];
	u32				othash[VIPX_MAX_TASK];
	struct vipx_taskmgr		taskmgr;
	struct vipx_task		control;
	wait_queue_head_t		control_wq;
};

void vipx_graph_print(struct vipx_graph *graph);
int vipx_graph_create(struct vipx_graph **graph, void *cookie, void *memory);
int vipx_graph_destroy(struct vipx_graph *graph);
int vipx_graph_config(struct vipx_graph *graph, struct vs4l_graph *info);
int vipx_graph_param(struct vipx_graph *graph, struct vs4l_param_list *plist);

#define CALL_GOPS(g, op, ...)	(((g)->gops->op) ? ((g)->gops->op(g, ##__VA_ARGS__)) : 0)

#endif
