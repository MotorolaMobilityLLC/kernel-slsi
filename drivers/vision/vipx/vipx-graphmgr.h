/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef VIPX_GRAPHMGR_H_
#define VIPX_GRAPHMGR_H_

#include <linux/kthread.h>
#include <linux/types.h>

#include "vipx-taskmgr.h"
#include "vipx-graph.h"
#include "vipx-interface.h"

#define VIPX_MAX_TASKDESC			(VIPX_MAX_GRAPH * 2)

enum vipx_taskdesc_state {
	VIPX_TASKDESC_STATE_INVALID,
	VIPX_TASKDESC_STATE_FREE,
	VIPX_TASKDESC_STATE_READY,
	VIPX_TASKDESC_STATE_REQUEST,
	VIPX_TASKDESC_STATE_ALLOC,
	VIPX_TASKDESC_STATE_PROCESS,
	VIPX_TASKDESC_STATE_COMPLETE
};

struct vipx_taskdesc {
	struct list_head		list;
	u32				index;
	u32				priority;
	struct vipx_graph		*graph;
	struct vipx_task		*task;
	u32				state;
};

enum vipx_graphmgr_state {
	VIPX_GRAPHMGR_OPEN,
	VIPX_GRAPHMGR_ENUM
};

enum vipx_graphmgr_client {
	VIPX_GRAPHMGR_CLIENT_GRAPH = 1,
	VIPX_GRAPHMGR_CLIENT_INTERFACE
};

struct vipx_graphmgr {
	struct vipx_graph		*graph[VIPX_MAX_GRAPH];
	atomic_t			active_cnt;
	unsigned long			state;
	struct mutex			mlock;

	u32				tick_cnt;
	u32				tick_pos;
	u32				sched_cnt;
	u32				sched_pos;
	struct kthread_worker		worker;
	struct task_struct		*task_graph;
	struct task_struct		*task_timer;

	struct vipx_interface		*interface;

	struct mutex			tdlock;
	struct vipx_taskdesc		taskdesc[VIPX_MAX_TASKDESC];
	struct list_head		tdfre_list;
	struct list_head		tdrdy_list;
	struct list_head		tdreq_list;
	struct list_head		tdalc_list;
	struct list_head		tdpro_list;
	struct list_head		tdcom_list;
	u32				tdfre_cnt;
	u32				tdrdy_cnt;
	u32				tdreq_cnt;
	u32				tdalc_cnt;
	u32				tdpro_cnt;
	u32				tdcom_cnt;
};

void vipx_taskdesc_print(struct vipx_graphmgr *graphmgr);
int vipx_graphmgr_probe(struct vipx_graphmgr *graphmgr);
int vipx_graphmgr_open(struct vipx_graphmgr *graphmgr);
int vipx_graphmgr_close(struct vipx_graphmgr *graphmgr);
int vipx_graphmgr_grp_register(struct vipx_graphmgr *graphmgr, struct vipx_graph *graph);
int vipx_graphmgr_grp_unregister(struct vipx_graphmgr *graphmgr, struct vipx_graph *graph);
int vipx_graphmgr_grp_start(struct vipx_graphmgr *graphmgr, struct vipx_graph *graph);
int vipx_graphmgr_grp_stop(struct vipx_graphmgr *graphmgr, struct vipx_graph *graph);
int vipx_graphmgr_itf_register(struct vipx_graphmgr *graphmgr, struct vipx_interface *interface);
int vipx_graphmgr_itf_unregister(struct vipx_graphmgr *graphmgr, struct vipx_interface *interface);
void vipx_graphmgr_queue(struct vipx_graphmgr *graphmgr, struct vipx_task *task);
#endif
