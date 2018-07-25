/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/slab.h>

#include "vipx-graphmgr.h"
#include "vipx-graph.h"
#include "vipx-debug.h"
#include "vs4l.h"

const struct vipx_graph_ops vipx_graph_ops;

static int __vipx_graph_start(struct vipx_graph *graph)
{
	int ret = 0;
	BUG_ON(!graph);

	if (test_bit(VIPX_GRAPH_STATE_START, &graph->state))
		return 0;

	ret = vipx_graphmgr_grp_start(graph->cookie, graph);
	if (ret) {
		vipx_ierr("vipx_graphmgr_grp_start is fail(%d)\n", graph, ret);
		goto p_err;
	}

	set_bit(VIPX_GRAPH_STATE_START, &graph->state);

p_err:
	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

static int __vipx_graph_stop(struct vipx_graph *graph)
{
	int ret = 0, errcnt = 0;
	u32 retry, timeout;
	struct vipx_taskmgr *taskmgr;
	struct vipx_task *control;

	if (!test_bit(VIPX_GRAPH_STATE_START, &graph->state))
		return 0;

	taskmgr = &graph->taskmgr;
	if (taskmgr->req_cnt + taskmgr->pre_cnt) {
		control = &graph->control;
		control->message = VIPX_CTRL_STOP;
		vipx_graphmgr_queue(graph->cookie, control);
		timeout = wait_event_timeout(graph->control_wq,
			control->message == VIPX_CTRL_STOP_DONE, VIPX_GRAPH_STOP_TIMEOUT);
		if (!timeout) {
			vipx_ierr("wait_event_timeout is expired\n", graph);
			errcnt++;
		}
	}

	retry = VIPX_STOP_WAIT_COUNT;
	while (--retry && taskmgr->req_cnt) {
		vipx_iwarn("waiting %d request cancel...(%d)\n", graph, taskmgr->req_cnt, retry);
		msleep(10);
	}

	if (!retry) {
		vipx_ierr("request cancel is fail\n", graph);
		errcnt++;
	}

	retry = VIPX_STOP_WAIT_COUNT;
	while (--retry && taskmgr->pre_cnt) {
		vipx_iwarn("waiting %d prepare cancel...(%d)\n", graph, taskmgr->pre_cnt, retry);
		msleep(10);
	}

	if (!retry) {
		vipx_ierr("prepare cancel is fail\n", graph);
		errcnt++;
	}

	retry = VIPX_STOP_WAIT_COUNT;
	while (--retry && taskmgr->pro_cnt) {
		vipx_iwarn("waiting %d process done...(%d)\n", graph, taskmgr->pro_cnt, retry);
		msleep(10);
	}

	if (!retry) {
		vipx_ierr("process done is fail\n", graph);
		errcnt++;
	}

	ret = vipx_graphmgr_grp_stop(graph->cookie, graph);
	if (ret) {
		vipx_ierr("vipx_graphmgr_grp_stop is fail(%d)\n", graph, ret);
		errcnt++;
	}

	vipx_task_flush(taskmgr);
	clear_bit(VIPX_GRAPH_STATE_START, &graph->state);

	vipx_info("%s:%d\n", __func__, ret);
	return errcnt;
}

static int __vipx_graph_unmap(struct vipx_graph *graph)
{
	int ret = 0;

	clear_bit(VIPX_GRAPH_STATE_MMAPPED, &graph->state);

	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

static int __vipx_graph_map(struct vipx_graph *graph)
{
	int ret = 0;

	set_bit(VIPX_GRAPH_STATE_MMAPPED, &graph->state);

	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

static int __vipx_graph_alloc(struct vipx_graph *graph)
{
	int ret = 0;

	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

static int __vipx_graph_free(struct vipx_graph *graph)
{
	int ret = 0;

	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

static int __vipx_graph_parse(struct vipx_graph *graph)
{
	int ret = 0;

	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

void vipx_graph_task_print(struct vipx_graph *graph)
{
	vipx_info("%s:\n", __func__);
}

void vipx_graph_print(struct vipx_graph *graph)
{

	vipx_info("%s:\n", __func__);
}

int vipx_graph_create(struct vipx_graph **graph, void *cookie, void *memory)
{
	int ret = 0;
	u32 i;
	struct vipx_taskmgr *taskmgr;

	BUG_ON(!cookie);

	*graph = kzalloc(sizeof(struct vipx_graph), GFP_KERNEL);
	if (*graph == NULL) {
		vipx_err("kzalloc is fail");
		ret = -ENOMEM;
		goto p_err;
	}

	ret = vipx_graphmgr_grp_register(cookie, *graph);
	if (ret) {
		vipx_err("vipx_graphmgr_grp_register is fail(%d)\n", ret);
		kfree(*graph);
		goto p_err;
	}

	(*graph)->control.message = VIPX_CTRL_NONE;
	(*graph)->cookie = cookie;
	(*graph)->memory = memory;
	(*graph)->gops = &vipx_graph_ops;
	mutex_init(&(*graph)->local_lock);

	/* task manager init */
	taskmgr = &(*graph)->taskmgr;
	taskmgr->id = (*graph)->idx;
	taskmgr->sindex = 0;
	spin_lock_init(&taskmgr->slock);

	for (i = 0; i < VIPX_MAX_TASK; ++i) {
		(*graph)->inhash[i] = VIPX_MAX_TASK;
		(*graph)->othash[i] = VIPX_MAX_TASK;
	}

	(*graph)->control.owner = *graph;
	init_waitqueue_head(&(*graph)->control_wq);
	ret = vipx_task_init(taskmgr, *graph);
	if (ret) {
		vipx_err("vipx_task_init is fail(%d)\n", ret);
		kfree(*graph);
		goto p_err;
	}

	(*graph)->informat_list.count = 0;
	(*graph)->informat_list.formats = 0;
	(*graph)->otformat_list.count = 0;
	(*graph)->otformat_list.formats = 0;

p_err:
	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

int vipx_graph_destroy(struct vipx_graph *graph)
{
	int ret = 0;

	BUG_ON(!graph);

	ret = __vipx_graph_stop(graph);
	if (ret)
		vipx_ierr("__vipx_graph_stop is fail(%d)\n", graph, ret);

	ret = vipx_graphmgr_grp_unregister(graph->cookie, graph);
	if (ret)
		vipx_ierr("vipx_graphmgr_grp_unregister is fail(%d)\n", graph, ret);

	ret = __vipx_graph_unmap(graph);
	if (ret)
		vipx_ierr("__vipx_graph_unmap is fail(%d)\n", graph, ret);

	ret = __vipx_graph_free(graph);
	if (ret)
		vipx_ierr("__vipx_graph_free is fail(%d)\n", graph, ret);

	if (graph->informat_list.formats)
		kfree(graph->informat_list.formats);
	graph->informat_list.formats = 0;
	graph->informat_list.count = 0;

	if (graph->otformat_list.formats)
		kfree(graph->otformat_list.formats);
	graph->otformat_list.count = 0;

	kfree(graph);

	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

int vipx_graph_config(struct vipx_graph *graph, struct vs4l_graph *info)
{
	int ret = 0;

	BUG_ON(!graph);
	BUG_ON(!graph->cookie);
	BUG_ON(!info);

	if (test_bit(VIPX_GRAPH_STATE_CONFIG, &graph->state)) {
		vipx_ierr("graph is already configured\n", graph);
		ret = -EINVAL;
		goto p_err;
	}

	if (info->priority > VIPX_GRAPH_MAX_PRIORITY) {
		vipx_iwarn("graph priority is over(%d)\n", graph, info->priority);
		info->priority = VIPX_GRAPH_MAX_PRIORITY;
	}

	graph->uid = info->id;
	graph->flags = info->flags;
	graph->priority = info->priority;

	/* 2. graph allocation */
	ret = __vipx_graph_alloc(graph);
	if (ret) {
		vipx_ierr("__vipx_graph_alloc is fail(%d)\n", graph, ret);
		goto p_err;
	}

	/* 3. parsing */
	ret = __vipx_graph_parse(graph);
	if (ret) {
		vipx_ierr("__vipx_graph_parse is fail(%d)\n", graph, ret);
		goto p_err;
	}

	/* 4. Buffer Mapping */
	ret = __vipx_graph_map(graph);
	if (ret) {
		vipx_ierr("__vipx_graph_map is fail(%d)\n", graph, ret);
		goto p_err;
	}

	set_bit(VIPX_GRAPH_STATE_CONFIG, &graph->state);

p_err:
	vipx_iinfo("%s(%d, %d, %X):%d\n", graph, __func__, info->id, info->priority, info->flags, ret);
	return ret;
}

int vipx_graph_param(struct vipx_graph *graph, struct vs4l_param_list *plist)
{
	int ret = 0;

	set_bit(VIPX_GRAPH_FLAG_UPDATE_PARAM, &graph->flags);

	vipx_iinfo("%s:%d\n", graph, __func__, ret);
	return ret;
}

static int vipx_graph_prepare(struct vb_queue *q, struct vb_container_list *clist)
{
	int ret = 0;

	vipx_dbg("%s:%d\n", __func__, ret);
	return ret;
}

static int vipx_graph_unprepare(struct vb_queue *q, struct vb_container_list *clist)
{
	int ret = 0;

	vipx_dbg("%s:%d\n", __func__, ret);
	return ret;
}

const struct vb_ops vb_ops = {
	.buf_prepare = vipx_graph_prepare,
	.buf_unprepare = vipx_graph_unprepare
};

int vipx_graph_start(struct vipx_queue *queue)
{
	int ret = 0;
	struct vipx_vertex_ctx *vctx;
	struct vipx_graph *graph;

	BUG_ON(!queue);

	vctx = container_of(queue, struct vipx_vertex_ctx, queue);
	graph = container_of(vctx, struct vipx_graph, vctx);

	ret = __vipx_graph_start(graph);
	if (ret)
		vipx_ierr("__vipx_graph_start is fail(%d)\n", graph, ret);

	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

int vipx_graph_stop(struct vipx_queue *queue)
{
	int ret = 0;
	struct vipx_graph *graph;
	struct vipx_vertex_ctx *vctx;

	BUG_ON(!queue);

	vctx = container_of(queue, struct vipx_vertex_ctx, queue);
	graph = container_of(vctx, struct vipx_graph, vctx);

	ret = __vipx_graph_stop(graph);
	if (ret)
		vipx_ierr("__vipx_graph_stop is fail(%d)\n", graph, ret);

	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

/* Description of flist elements
 * tartget : We done need this field. Ignore.
 * format : Fourcc name. Converting to char pointer.
 * plane : Num of planes.
 * width, height
 */
int vipx_graph_format(struct vipx_queue *queue, struct vs4l_format_list *flist)
{
	int ret = 0;
	struct vipx_graph *graph;
	struct vipx_vertex_ctx *vctx;
	struct vipx_format_list *in_list;
	struct vipx_format_list *ot_list;
	int cnt = 0;

	BUG_ON(!queue);
	BUG_ON(!flist);

	vctx = container_of(queue, struct vipx_vertex_ctx, queue);
	graph = container_of(vctx, struct vipx_graph, vctx);
	in_list = &graph->informat_list;
	ot_list = &graph->otformat_list;

	if (flist->direction == VS4L_DIRECTION_IN) {
		if (in_list->count != flist->count) {
			if (in_list->formats)
				kfree(in_list->formats);

			in_list->count = flist->count;
			in_list->formats = kzalloc(in_list->count * sizeof(struct vipx_format), GFP_KERNEL);
			if (!in_list->formats) {
				ret = -ENOMEM;
				goto p_err;
			}

			for (cnt = 0; cnt < in_list->count; cnt++) {
				vipx_info("[%d] size (%dx%d), format(%d), plane(%d)\n", cnt,
						flist->formats[cnt].width,
						flist->formats[cnt].height,
						flist->formats[cnt].format,
						flist->formats[cnt].plane);

				in_list->formats[cnt].width = flist->formats[cnt].width;
				in_list->formats[cnt].height = flist->formats[cnt].height;
				in_list->formats[cnt].format = flist->formats[cnt].format;
				in_list->formats[cnt].plane = flist->formats[cnt].plane;
			}
		}
	} else if (flist->direction == VS4L_DIRECTION_OT) {
		if (ot_list->count != flist->count) {
			if (ot_list->formats)
				kfree(ot_list->formats);

			ot_list->count = flist->count;
			ot_list->formats = kzalloc(ot_list->count * sizeof(struct vipx_format), GFP_KERNEL);
			if (!ot_list->formats) {
				ret = -ENOMEM;
				goto p_err;
			}

			for (cnt = 0; cnt < ot_list->count; cnt++) {
				vipx_info("[%d] size (%dx%d), format(%d), plane(%d)\n", cnt,
						flist->formats[cnt].width,
						flist->formats[cnt].height,
						flist->formats[cnt].format,
						flist->formats[cnt].plane);

				ot_list->formats[cnt].width = flist->formats[cnt].width;
				ot_list->formats[cnt].height = flist->formats[cnt].height;
				ot_list->formats[cnt].format = flist->formats[cnt].format;
				ot_list->formats[cnt].plane = flist->formats[cnt].plane;
			}
		}
	} else {
		vipx_err("invalid direction(%d)\n", flist->direction);
		ret = -EINVAL;
		goto p_err;
	}

	vipx_info("%s:%d, count in/out (%d/%d)\n", __func__, ret,
			graph->informat_list.count,
			graph->otformat_list.count);
	return ret;

p_err:
	if (in_list->formats)
		kfree(in_list->formats);
	in_list->formats = 0;

	if (ot_list->formats)
		kfree(ot_list->formats);
	ot_list->formats = 0;

	return ret;
}

static int vipx_graph_queue(struct vipx_queue *queue, struct vb_container_list *incl, struct vb_container_list *otcl)
{
	int ret = 0;
	unsigned long flag;
	struct vipx_graph *graph;
	struct vipx_vertex_ctx *vctx;
	struct vipx_taskmgr *taskmgr;
	struct vipx_task *task;
	int i = 0, j = 0;

	BUG_ON(!queue);
	BUG_ON(!incl);
	//BUG_ON(incl->index < VIPX_MAX_TASK);
	BUG_ON(!otcl);
	//BUG_ON(otcl->index < VIPX_MAX_TASK);

	vctx = container_of(queue, struct vipx_vertex_ctx, queue);
	graph = container_of(vctx, struct vipx_graph, vctx);
	taskmgr = &graph->taskmgr;

	if (!test_bit(VIPX_GRAPH_STATE_START, &graph->state)) {
		vipx_ierr("graph is NOT start\n", graph);
		ret = -EINVAL;
		goto p_err;
	}

	if (incl->id != otcl->id) {
		vipx_warn("buffer id is incoincidence(%d, %d)\n", incl->id, otcl->id);
		otcl->id = incl->id;
	}

	taskmgr_e_barrier_irqs(taskmgr, 0, flag);
	vipx_task_pick_fre_to_req(taskmgr, &task);
	taskmgr_x_barrier_irqr(taskmgr, 0, flag);

	if (!task) {
		vipx_ierr("task is lack\n", graph);
		vipx_task_print_all(taskmgr);
		ret = -ENOMEM;
		goto p_err;
	}

	graph->inhash[incl->index] = task->index;
	graph->othash[otcl->index] = task->index;
	graph->input_cnt++;

	vipx_dbg("in-container list: dir(%d), id(%d), index(%d), flags(%lx), count(%d)\n",
			incl->direction, incl->id, incl->index, incl->flags, incl->count);
	for (i = 0; i < incl->count; i++) {
		vipx_dbg("in-containers[%d] type(%d), target(%d), memory(%d), count(%d)\n", i,
				incl->containers[i].type, incl->containers[i].target,
				incl->containers[i].memory, incl->containers[i].count);

		for (j = 0; j < incl->containers[i].count; j++) {
			vipx_dbg("in-buffer[%d] fd(%d), kvaddr(%p), dvaddr(%p)\n", j,
					incl->containers[i].buffers[j].m.fd,
					incl->containers[i].buffers[j].kvaddr,
					(void *)incl->containers[i].buffers[j].dvaddr);
		}
	}

	vipx_dbg("out-container list: dir(%d), id(%d), index(%d), flags(%lx), count(%d)\n",
			otcl->direction, otcl->id, otcl->index, otcl->flags, otcl->count);
	for (i = 0; i < otcl->count; i++) {
		vipx_dbg("out-containers[%d] type(%d), target(%d), memory(%d), count(%d)\n", i,
				otcl->containers[i].type, otcl->containers[i].target,
				otcl->containers[i].memory, otcl->containers[i].count);

		for (j = 0; j < otcl->containers[i].count; j++) {
			vipx_dbg("out-buffer[%d] fd(%d), kvaddr(%p), dvaddr(%p)\n", j,
					otcl->containers[i].buffers[j].m.fd,
					otcl->containers[i].buffers[j].kvaddr,
					(void *)otcl->containers[i].buffers[j].dvaddr);
		}
	}

	task->id = incl->id;
	task->incl = incl;
	task->otcl = otcl;
	task->message = VIPX_TASK_REQUEST;
	task->param0 = 0;
	task->param1 = 0;
	task->param2 = 0;
	task->param3 = 0;
	clear_bit(VS4L_CL_FLAG_TIMESTAMP, &task->flags);

	if ((incl->flags & (1 << VS4L_CL_FLAG_TIMESTAMP)) ||
		(otcl->flags & (1 << VS4L_CL_FLAG_TIMESTAMP))) {
		set_bit(VS4L_CL_FLAG_TIMESTAMP, &task->flags);
		vipx_get_timestamp(&task->time[VIPX_TMP_QUEUE]);
	}

	vipx_graphmgr_queue(graph->cookie, task);

p_err:
	vipx_dbg("%s:%d\n", __func__, ret);
	return ret;
}

static int vipx_graph_deque(struct vipx_queue *queue, struct vb_container_list *clist)
{
	int ret = 0;
	u32 findex;
	unsigned long flags;
	struct vipx_graph *graph;
	struct vipx_vertex_ctx *vctx;
	struct vipx_taskmgr *taskmgr;
	struct vipx_task *task;

	BUG_ON(!queue);
	BUG_ON(!clist);
	//BUG_ON(clist->index < VIPX_MAX_TASK);

	vctx = container_of(queue, struct vipx_vertex_ctx, queue);
	graph = container_of(vctx, struct vipx_graph, vctx);
	taskmgr = &graph->taskmgr;

	if (!test_bit(VIPX_GRAPH_STATE_START, &graph->state)) {
		vipx_ierr("graph is NOT start\n", graph);
		ret = -EINVAL;
		goto p_err;
	}

	if (clist->direction == VS4L_DIRECTION_IN)
		findex = graph->inhash[clist->index];
	else
		findex = graph->othash[clist->index];

	if (findex >= VIPX_MAX_TASK) {
		vipx_ierr("task index(%d) invalid\n", graph, findex);
		BUG();
	}

	task = &taskmgr->task[findex];
	if (task->state != VIPX_TASK_STATE_COMPLETE) {
		vipx_ierr("task state(%d) is invalid\n", graph, task->state);
		BUG();
	}

	if (clist->direction == VS4L_DIRECTION_IN) {
		if (task->incl != clist) {
			vipx_ierr("incl ptr is invalid(%p != %p)\n", graph, task->incl, clist);
			BUG();
		}

		graph->inhash[clist->index] = VIPX_MAX_TASK;
		task->incl = NULL;
	} else {
		if (task->otcl != clist) {
			vipx_ierr("otcl ptr is invalid(%p != %p)\n", graph, task->otcl, clist);
			BUG();
		}

		graph->othash[clist->index] = VIPX_MAX_TASK;
		task->otcl = NULL;
	}

	if (task->incl || task->otcl)
		goto p_err;

	taskmgr_e_barrier_irqs(taskmgr, 0, flags);
	vipx_task_trans_com_to_fre(taskmgr, task);
	taskmgr_x_barrier_irqr(taskmgr, 0, flags);

p_err:
	vipx_dbg("%s:%d\n", __func__, ret);
	return ret;
}

const struct vipx_queue_ops vipx_queue_ops = {
	.start		= vipx_graph_start,
	.stop		= vipx_graph_stop,
	.format 	= vipx_graph_format,
	.queue		= vipx_graph_queue,
	.deque		= vipx_graph_deque
};

static int vipx_graph_control(struct vipx_graph *graph, struct vipx_task *task)
{
	int ret = 0;
	struct vipx_taskmgr *taskmgr;

	BUG_ON(!graph);
	BUG_ON(!task);

	taskmgr = &graph->taskmgr;

	if (&graph->control != task) {
		vipx_ierr("control task is invalid(%p == %p)\n", graph, &graph->control, task);
		BUG();
	}

	switch (task->message) {
	case VIPX_CTRL_STOP:
		graph->control.message = VIPX_CTRL_STOP_DONE;
		wake_up(&graph->control_wq);
		break;
	default:
		vipx_ierr("unresolved message(%d)\n", graph, task->message);
		vipx_task_print_all(taskmgr);
		BUG();
		break;
	}

	vipx_iinfo("%s:%d\n", graph, __func__, ret);
	return ret;
}

static int vipx_graph_request(struct vipx_graph *graph, struct vipx_task *task)
{
	int ret = 0;
	unsigned long flags;
	struct vipx_taskmgr *taskmgr;

	BUG_ON(!graph);
	BUG_ON(!task);

	taskmgr = &graph->taskmgr;

	if (task->state != VIPX_TASK_STATE_REQUEST) {
		vipx_ierr("task state(%d) is invalid\n", graph, task->state);
		BUG();
	}

	taskmgr_e_barrier_irqs(taskmgr, 0, flags);
	vipx_task_trans_req_to_pre(taskmgr, task);
	taskmgr_x_barrier_irqr(taskmgr, 0, flags);

	if (test_bit(VS4L_CL_FLAG_TIMESTAMP, &task->flags))
		vipx_get_timestamp(&task->time[VIPX_TMP_REQUEST]);

	vipx_idbg("%s:%d\n", graph, __func__, ret);
	return ret;
}

static int vipx_graph_process(struct vipx_graph *graph, struct vipx_task *task)
{
	int ret = 0;
	unsigned long flags;
	struct vipx_taskmgr *taskmgr;

	BUG_ON(!graph);
	BUG_ON(!task);

	taskmgr = &graph->taskmgr;

	if (task->state != VIPX_TASK_STATE_PREPARE) {
		vipx_ierr("task state(%d) is invalid\n", graph, task->state);
		BUG();
	}

	taskmgr_e_barrier_irqs(taskmgr, TASKMGR_IDX_0, flags);
	vipx_task_trans_pre_to_pro(taskmgr, task);
	taskmgr_x_barrier_irqr(taskmgr, TASKMGR_IDX_0, flags);

#ifdef DBG_STREAMING
	vipx_iinfo("PROCESS(%d, %d)\n", graph, task->index, task->id);
#endif

	if (test_bit(VS4L_CL_FLAG_TIMESTAMP, &task->flags))
		vipx_get_timestamp(&task->time[VIPX_TMP_PROCESS]);

	vipx_idbg("%s:%d\n", graph, __func__, ret);
	return ret;
}

static int vipx_graph_cancel(struct vipx_graph *graph, struct vipx_task *task)
{
	int ret = 0;
	unsigned long flags;
	unsigned long result;
	struct vipx_taskmgr *taskmgr;
	struct vipx_queue *queue;
	struct vb_container_list *incl, *otcl;

	BUG_ON(!graph);
	BUG_ON(!task);

	taskmgr = &graph->taskmgr;
	queue = &graph->vctx.queue;
	incl = task->incl;
	otcl = task->otcl;
	result = 0;

	if (!test_bit(VIPX_GRAPH_STATE_START, &graph->state)) {
		vipx_ierr("graph is NOT start\n", graph);
		BUG();
	}

	if (task->state != VIPX_TASK_STATE_PROCESS) {
		vipx_ierr("task state(%d) is invalid\n", graph, task->state);
		BUG();
	}

	if (test_bit(VS4L_CL_FLAG_TIMESTAMP, &task->flags)) {
		vipx_get_timestamp(&task->time[VIPX_TMP_DONE]);

		if (incl->flags & (1 << VS4L_CL_FLAG_TIMESTAMP))
			memcpy(incl->timestamp, task->time, sizeof(task->time));

		if (otcl->flags & (1 << VS4L_CL_FLAG_TIMESTAMP))
			memcpy(otcl->timestamp, task->time, sizeof(task->time));

#ifdef DBG_TIMEMEASURE
		vipx_irinfo("[TM] G%d : QR(%ld), RR(%ld), RP(%ld), PD(%ld)\n", graph, task, graph->uid,
			VIPX_TIME_IN_US(task->time[VIPX_TMP_REQUEST]) -
			VIPX_TIME_IN_US(task->time[VIPX_TMP_QUEUE]),
			VIPX_TIME_IN_US(task->time[VIPX_TMP_RESOURCE]) -
			VIPX_TIME_IN_US(task->time[VIPX_TMP_REQUEST]),
			VIPX_TIME_IN_US(task->time[VIPX_TMP_PROCESS]) -
			VIPX_TIME_IN_US(task->time[VIPX_TMP_RESOURCE]),
			VIPX_TIME_IN_US(task->time[VIPX_TMP_DONE]) -
			VIPX_TIME_IN_US(task->time[VIPX_TMP_PROCESS]));
#endif
	}

#ifdef DBG_STREAMING
	vipx_iinfo("NDONE(%d, %d)\n", graph, task->index, task->id);
#endif
	set_bit(VS4L_CL_FLAG_DONE, &result);
	set_bit(VS4L_CL_FLAG_INVALID, &result);

	taskmgr_e_barrier_irqs(taskmgr, TASKMGR_IDX_0, flags);
	vipx_task_trans_pro_to_com(taskmgr, task);
	taskmgr_x_barrier_irqr(taskmgr, TASKMGR_IDX_0, flags);

	graph->recent = task->id;
	graph->done_cnt++;
	vipx_queue_done(queue, incl, otcl, result);

	vipx_iinfo("%s:%d\n", graph, __func__, ret);
	return ret;
}

static int vipx_graph_done(struct vipx_graph *graph, struct vipx_task *task)
{
	int ret = 0;
	unsigned long flags;
	unsigned long result;
	struct vipx_taskmgr *taskmgr;
	struct vipx_queue *queue;
	struct vb_container_list *incl, *otcl;

	BUG_ON(!graph);
	BUG_ON(!task);

	taskmgr = &graph->taskmgr;
	queue = &graph->vctx.queue;
	incl = task->incl;
	otcl = task->otcl;
	result = 0;

	if (!test_bit(VIPX_GRAPH_STATE_START, &graph->state)) {
		vipx_ierr("graph is NOT start\n", graph);
		BUG();
	}

	if (task->state != VIPX_TASK_STATE_PROCESS) {
		vipx_ierr("task state(%d) is invalid\n", graph, task->state);
		BUG();
	}

	if (test_bit(VS4L_CL_FLAG_TIMESTAMP, &task->flags)) {
		vipx_get_timestamp(&task->time[VIPX_TMP_DONE]);

		if (incl->flags & (1 << VS4L_CL_FLAG_TIMESTAMP))
			memcpy(incl->timestamp, task->time, sizeof(task->time));

		if (otcl->flags & (1 << VS4L_CL_FLAG_TIMESTAMP))
			memcpy(otcl->timestamp, task->time, sizeof(task->time));

#ifdef DBG_TIMEMEASURE
		vipx_irinfo("[TM] G%d : QR(%ld), RR(%ld), RP(%ld), PD(%ld)\n", graph, task, graph->uid,
			VIPX_TIME_IN_US(task->time[VIPX_TMP_REQUEST]) -
			VIPX_TIME_IN_US(task->time[VIPX_TMP_QUEUE]),
			VIPX_TIME_IN_US(task->time[VIPX_TMP_RESOURCE]) -
			VIPX_TIME_IN_US(task->time[VIPX_TMP_REQUEST]),
			VIPX_TIME_IN_US(task->time[VIPX_TMP_PROCESS]) -
			VIPX_TIME_IN_US(task->time[VIPX_TMP_RESOURCE]),
			VIPX_TIME_IN_US(task->time[VIPX_TMP_DONE]) -
			VIPX_TIME_IN_US(task->time[VIPX_TMP_PROCESS]));
#endif
	}

	if (task->param0) {
#ifdef DBG_STREAMING
		vipx_iinfo("NDONE(%d, %d)\n", graph, task->index, task->id);
#endif
		set_bit(VS4L_CL_FLAG_DONE, &result);
		set_bit(VS4L_CL_FLAG_INVALID, &result);
	} else {
#ifdef DBG_STREAMING
		vipx_iinfo("DONE(%d, %d)\n", graph, task->index, task->id);
#endif
		set_bit(VS4L_CL_FLAG_DONE, &result);
	}

	taskmgr_e_barrier_irqs(taskmgr, TASKMGR_IDX_0, flags);
	vipx_task_trans_pro_to_com(taskmgr, task);
	taskmgr_x_barrier_irqr(taskmgr, TASKMGR_IDX_0, flags);

	graph->recent = task->id;
	graph->done_cnt++;
	vipx_queue_done(queue, incl, otcl, result);

	vipx_idbg("%s:%d\n", graph, __func__, ret);
	return ret;
}

static int vipx_graph_update_param(struct vipx_graph *graph, struct vipx_task *task)
{
	int ret = 0;

	vipx_iinfo("%s:%d\n", graph, __func__, ret);
	return ret;
}

const struct vipx_graph_ops vipx_graph_ops = {
	.control	= vipx_graph_control,
	.request	= vipx_graph_request,
	.process	= vipx_graph_process,
	.cancel 	= vipx_graph_cancel,
	.done		= vipx_graph_done,
	.update_param	= vipx_graph_update_param
};
