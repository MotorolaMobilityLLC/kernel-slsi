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
#include <linux/kthread.h>
#include <linux/sched/types.h>
#include <linux/sched/rt.h>
#include <linux/bug.h>
#include <linux/scatterlist.h>


#include "vipx-taskmgr.h"
#include "vipx-device.h"
#include "vipx-graphmgr.h"

/*
 * task trans
 */

static void __vipx_taskdesc_s_free(struct vipx_graphmgr *graphmgr, struct vipx_taskdesc *taskdesc)
{
	BUG_ON(!graphmgr);
	BUG_ON(!taskdesc);

	taskdesc->state = VIPX_TASKDESC_STATE_FREE;

	list_add_tail(&taskdesc->list, &graphmgr->tdfre_list);
	graphmgr->tdfre_cnt++;
}

static void __vipx_taskdesc_free_head(struct vipx_graphmgr *graphmgr, struct vipx_taskdesc **taskdesc)
{
	BUG_ON(!graphmgr);
	BUG_ON(!taskdesc);

	if (graphmgr->tdfre_cnt)
		*taskdesc = container_of(graphmgr->tdfre_list.next, struct vipx_taskdesc, list);
	else
		*taskdesc = NULL;
}

static void __vipx_taskdesc_s_ready(struct vipx_graphmgr *graphmgr, struct vipx_taskdesc *taskdesc)
{
	BUG_ON(!graphmgr);
	BUG_ON(!taskdesc);

	taskdesc->state = VIPX_TASKDESC_STATE_FREE;

	list_add_tail(&taskdesc->list, &graphmgr->tdrdy_list);
	graphmgr->tdrdy_cnt++;
}

static void __vipx_taskdesc_g_ready(struct vipx_graphmgr *graphmgr,
	struct vipx_taskdesc **taskdesc)
{
	if (graphmgr->tdrdy_cnt &&
		(*taskdesc = container_of(graphmgr->tdrdy_list.next, struct vipx_taskdesc, list))) {
		list_del(&(*taskdesc)->list);
		graphmgr->tdrdy_cnt--;
		(*taskdesc)->state = VIPX_TASK_STATE_INVALID;
	} else {
		*taskdesc = NULL;
	}
}

static void __vipx_taskdesc_s_request(struct vipx_graphmgr *graphmgr,
	struct vipx_taskdesc *prev,
	struct vipx_taskdesc *taskdesc,
	struct vipx_taskdesc *next)
{
	taskdesc->state = VIPX_TASKDESC_STATE_REQUEST;

	if (prev && next) {
		next->list.prev = &taskdesc->list;
	        taskdesc->list.next = &next->list;
	        taskdesc->list.prev = &prev->list;
	        prev->list.next = &taskdesc->list;
	} else if (prev) {
		list_add_tail(&taskdesc->list, &graphmgr->tdreq_list);
	} else {
		list_add(&taskdesc->list, &graphmgr->tdreq_list);
	}

	graphmgr->tdreq_cnt++;
}

static void __vipx_taskdesc_s_alloc(struct vipx_graphmgr *graphmgr, struct vipx_taskdesc *taskdesc)
{
	taskdesc->state = VIPX_TASKDESC_STATE_ALLOC;
	list_add_tail(&taskdesc->list, &graphmgr->tdalc_list);
	graphmgr->tdalc_cnt++;
}

void __vipx_taskdesc_s_process(struct vipx_graphmgr *graphmgr, struct vipx_taskdesc *taskdesc)
{
	BUG_ON(!graphmgr);
	BUG_ON(!taskdesc);

	taskdesc->state = VIPX_TASKDESC_STATE_PROCESS;
	list_add_tail(&taskdesc->list, &graphmgr->tdpro_list);
	graphmgr->tdpro_cnt++;
}

static void __vipx_taskdesc_s_complete(struct vipx_graphmgr *graphmgr, struct vipx_taskdesc *taskdesc)
{
	BUG_ON(!graphmgr);
	BUG_ON(!taskdesc);

	taskdesc->state = VIPX_TASKDESC_STATE_COMPLETE;
	list_add_tail(&taskdesc->list, &graphmgr->tdcom_list);
	graphmgr->tdcom_cnt++;
}

void __vipx_taskdesc_trans_fre_to_rdy(struct vipx_graphmgr *graphmgr, struct vipx_taskdesc *taskdesc)
{
	BUG_ON(!graphmgr);
	BUG_ON(!taskdesc);
	BUG_ON(!graphmgr->tdfre_cnt);
	BUG_ON(taskdesc->state != VIPX_TASKDESC_STATE_FREE);

	list_del(&taskdesc->list);
	graphmgr->tdfre_cnt--;
	__vipx_taskdesc_s_ready(graphmgr, taskdesc);
}

void __vipx_taskdesc_trans_rdy_to_fre(struct vipx_graphmgr *graphmgr, struct vipx_taskdesc *taskdesc)
{
	BUG_ON(!graphmgr);
	BUG_ON(!taskdesc);
	BUG_ON(!graphmgr->tdrdy_cnt);
	BUG_ON(taskdesc->state != VIPX_TASKDESC_STATE_READY);

	list_del(&taskdesc->list);
	graphmgr->tdrdy_cnt--;
	__vipx_taskdesc_s_free(graphmgr, taskdesc);
}

void __vipx_taskdesc_trans_req_to_alc(struct vipx_graphmgr *graphmgr, struct vipx_taskdesc *taskdesc)
{
	BUG_ON(!graphmgr);
	BUG_ON(!taskdesc);
	BUG_ON(!graphmgr->tdreq_cnt);
	BUG_ON(taskdesc->state != VIPX_TASKDESC_STATE_REQUEST);

	list_del(&taskdesc->list);
	graphmgr->tdreq_cnt--;
	__vipx_taskdesc_s_alloc(graphmgr, taskdesc);
}

void __vipx_taskdesc_trans_req_to_pro(struct vipx_graphmgr *graphmgr, struct vipx_taskdesc *taskdesc)
{
	BUG_ON(!graphmgr);
	BUG_ON(!taskdesc);
	BUG_ON(!graphmgr->tdreq_cnt);
	BUG_ON(taskdesc->state != VIPX_TASKDESC_STATE_REQUEST);

	list_del(&taskdesc->list);
	graphmgr->tdreq_cnt--;
	__vipx_taskdesc_s_process(graphmgr, taskdesc);
}

void __vipx_taskdesc_trans_req_to_fre(struct vipx_graphmgr *graphmgr, struct vipx_taskdesc *taskdesc)
{
	BUG_ON(!graphmgr);
	BUG_ON(!taskdesc);
	BUG_ON(!graphmgr->tdreq_cnt);
	BUG_ON(taskdesc->state != VIPX_TASKDESC_STATE_REQUEST);

	list_del(&taskdesc->list);
	graphmgr->tdreq_cnt--;
	__vipx_taskdesc_s_free(graphmgr, taskdesc);
}

void __vipx_taskdesc_trans_alc_to_pro(struct vipx_graphmgr *graphmgr, struct vipx_taskdesc *taskdesc)
{
	BUG_ON(!graphmgr);
	BUG_ON(!taskdesc);
	BUG_ON(!graphmgr->tdalc_cnt);
	BUG_ON(taskdesc->state != VIPX_TASKDESC_STATE_ALLOC);

	list_del(&taskdesc->list);
	graphmgr->tdalc_cnt--;
	__vipx_taskdesc_s_process(graphmgr, taskdesc);
}

void __vipx_taskdesc_trans_alc_to_fre(struct vipx_graphmgr *graphmgr, struct vipx_taskdesc *taskdesc)
{
	BUG_ON(!graphmgr);
	BUG_ON(!taskdesc);
	BUG_ON(!graphmgr->tdalc_cnt);
	BUG_ON(taskdesc->state != VIPX_TASKDESC_STATE_ALLOC);

	list_del(&taskdesc->list);
	graphmgr->tdalc_cnt--;
	__vipx_taskdesc_s_free(graphmgr, taskdesc);
}

static void __vipx_taskdesc_trans_pro_to_com(struct vipx_graphmgr *graphmgr, struct vipx_taskdesc *taskdesc)
{
	BUG_ON(!graphmgr);
	BUG_ON(!taskdesc);
	BUG_ON(!graphmgr->tdpro_cnt);
	BUG_ON(taskdesc->state != VIPX_TASKDESC_STATE_PROCESS);

	list_del(&taskdesc->list);
	graphmgr->tdpro_cnt--;
	__vipx_taskdesc_s_complete(graphmgr, taskdesc);
}

void __vipx_taskdesc_trans_pro_to_fre(struct vipx_graphmgr *graphmgr, struct vipx_taskdesc *taskdesc)
{
	BUG_ON(!graphmgr);
	BUG_ON(!taskdesc);
	BUG_ON(!graphmgr->tdpro_cnt);
	BUG_ON(taskdesc->state != VIPX_TASKDESC_STATE_PROCESS);

	list_del(&taskdesc->list);
	graphmgr->tdpro_cnt--;
	__vipx_taskdesc_s_free(graphmgr, taskdesc);
}

static void __vipx_taskdesc_trans_com_to_fre(struct vipx_graphmgr *graphmgr, struct vipx_taskdesc *taskdesc)
{
	BUG_ON(!graphmgr);
	BUG_ON(!taskdesc);
	BUG_ON(!graphmgr->tdcom_cnt);
	BUG_ON(taskdesc->state != VIPX_TASKDESC_STATE_COMPLETE);

	list_del(&taskdesc->list);
	graphmgr->tdcom_cnt--;
	__vipx_taskdesc_s_free(graphmgr, taskdesc);
}

/*
 * task trans END
 */

void vipx_taskdesc_print(struct vipx_graphmgr *graphmgr)
{
}

#ifdef USE_TASK_TIMER
static int vipx_time_thread(void *data)
{
	int ret = 0;

	return ret;
}
#endif

static int __vipx_itf_init(struct vipx_interface *interface,
	struct vipx_graph *graph)
{
	int ret = 0;
	unsigned long flags;
	struct vipx_taskmgr *itaskmgr;
	struct vipx_task *itask;

	BUG_ON(!interface);
	BUG_ON(!graph);

	itaskmgr = &interface->taskmgr;

	ret = vipx_hw_enum(interface);
	if (ret) {
		vipx_err("vipx_hw_enum is fail(%d)\n", ret);
		goto p_err;
	}

	taskmgr_e_barrier_irqs(itaskmgr, 0, flags);
	vipx_task_pick_fre_to_req(itaskmgr, &itask);
	taskmgr_x_barrier_irqr(itaskmgr, 0, flags);

	if (!itask) {
		vipx_err("itask is NULL\n");
		ret = -ENOMEM;
		goto p_err;
	}

	itask->id = 0;
	itask->lock = &graph->local_lock;
	itask->findex = VIPX_MAX_TASK;
	itask->tdindex = VIPX_MAX_TASKDESC;
	itask->message = VIPX_TASK_INIT;
	itask->param0 = graph->uid;
	itask->param1 = graph->idx;
	itask->param2 = 0;
	itask->param3 = 0;

	ret = vipx_hw_init(interface, itask);
	if (ret) {
		vipx_err("vipx_hw_init is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	vipx_info("%s:\n", __func__);
	return ret;
}

static int __vipx_itf_deinit(struct vipx_interface *interface,
	struct vipx_graph *graph)
{
	int ret = 0;
	unsigned long flags;
	struct vipx_taskmgr *itaskmgr;
	struct vipx_task *itask;

	BUG_ON(!interface);
	BUG_ON(!graph);

	itaskmgr = &interface->taskmgr;

	taskmgr_e_barrier_irqs(itaskmgr, 0, flags);
	vipx_task_pick_fre_to_req(itaskmgr, &itask);
	taskmgr_x_barrier_irqr(itaskmgr, 0, flags);

	if (!itask) {
		vipx_err("itask is NULL\n");
		ret = -ENOMEM;
		goto p_err;
	}

	itask->id = 0;
	itask->lock = &graph->local_lock;
	itask->findex = VIPX_MAX_TASK;
	itask->tdindex = VIPX_MAX_TASKDESC;
	itask->message = VIPX_TASK_DEINIT;
	itask->param0 = graph->uid;
	itask->param1 = graph->idx;
	itask->param2 = 0;
	itask->param3 = 0;

	ret = vipx_hw_deinit(interface, itask);
	if (ret) {
		vipx_err("vipx_hw_deinit is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	vipx_info("%s:\n", __func__);
	return ret;
}

int __vipx_itf_create(struct vipx_interface *interface,
	struct vipx_graph *graph)
{
	int ret = 0;
	unsigned long flags;
	struct vipx_taskmgr *itaskmgr;
	struct vipx_task *itask;

	BUG_ON(!interface);
	BUG_ON(!graph);

	itaskmgr = &interface->taskmgr;

	taskmgr_e_barrier_irqs(itaskmgr, 0, flags);
	vipx_task_pick_fre_to_req(itaskmgr, &itask);
	taskmgr_x_barrier_irqr(itaskmgr, 0, flags);

	if (!itask) {
		vipx_err("itask is NULL\n");
		ret = -ENOMEM;
		goto p_err;
	}

#ifdef DBG_TIMEMEASURE
	vipx_get_timestamp(&itask->time[VIPX_TMP_REQUEST]);
#endif

	itask->id = 0;
	itask->lock = &graph->local_lock;
	itask->findex = VIPX_MAX_TASK;
	itask->tdindex = VIPX_MAX_TASKDESC;
	itask->message = VIPX_TASK_CREATE;
	itask->param0 = graph->uid;
	itask->param1 = graph->idx;
	itask->param2 = (ulong)0;
	itask->param3 = (ulong)0;

	ret = vipx_hw_create(interface, itask);
	if (ret) {
		vipx_err("vipx_hw_create is fail(%d)\n", ret);
		goto p_err;
	}

#ifdef DBG_TIMEMEASURE
	vipx_iinfo("[TM] C : %ldus, %ldus\n", graph,
		VIPX_TIME_IN_US(itask->time[VIPX_TMP_PROCESS]) -
		VIPX_TIME_IN_US(itask->time[VIPX_TMP_REQUEST]),
		VIPX_TIME_IN_US(itask->time[VIPX_TMP_DONE]) -
		VIPX_TIME_IN_US(itask->time[VIPX_TMP_PROCESS]));
#endif

p_err:
	vipx_info("%s:\n", __func__);
	return ret;
}

static int __vipx_itf_destroy(struct vipx_interface *interface,
	struct vipx_graph *graph)
{
	int ret = 0;
	unsigned long flags;
	struct vipx_taskmgr *itaskmgr;
	struct vipx_task *itask;

	BUG_ON(!interface);
	BUG_ON(!graph);

	itaskmgr = &interface->taskmgr;

	taskmgr_e_barrier_irqs(itaskmgr, 0, flags);
	vipx_task_pick_fre_to_req(itaskmgr, &itask);
	taskmgr_x_barrier_irqr(itaskmgr, 0, flags);

	if (!itask) {
		vipx_err("itask is NULL\n");
		ret = -ENOMEM;
		goto p_err;
	}

#ifdef DBG_TIMEMEASURE
	vipx_get_timestamp(&itask->time[VIPX_TMP_REQUEST]);
#endif

	itask->id = 0;
	itask->lock = &graph->local_lock;
	itask->findex = VIPX_MAX_TASK;
	itask->tdindex = VIPX_MAX_TASKDESC;
	itask->message = VIPX_TASK_DESTROY;
	itask->param0 = graph->uid;
	itask->param1 = graph->idx;
	itask->param2 = 0;
	itask->param3 = 0;

	ret = vipx_hw_destroy(interface, itask);
	if (ret) {
		vipx_err("vipx_hw_destory is fail(%d)\n", ret);
		goto p_err;
	}

#ifdef DBG_TIMEMEASURE
	vipx_iinfo("[TM] D : %ldus, %ldus\n", graph,
		VIPX_TIME_IN_US(itask->time[VIPX_TMP_PROCESS]) -
		VIPX_TIME_IN_US(itask->time[VIPX_TMP_REQUEST]),
		VIPX_TIME_IN_US(itask->time[VIPX_TMP_DONE]) -
		VIPX_TIME_IN_US(itask->time[VIPX_TMP_PROCESS]));
#endif

p_err:
	vipx_info("%s:\n", __func__);
	return ret;
}

static int __vipx_itf_config(struct vipx_interface *interface,
	struct vipx_graph *graph)
{
	int ret = 0;
	unsigned long flags;
	struct vipx_taskmgr *itaskmgr;
	struct vipx_task *itask;

	BUG_ON(!interface);
	BUG_ON(!graph);

	itaskmgr = &interface->taskmgr;

	taskmgr_e_barrier_irqs(itaskmgr, 0, flags);
	vipx_task_pick_fre_to_req(itaskmgr, &itask);
	taskmgr_x_barrier_irqr(itaskmgr, 0, flags);

	if (!itask) {
		vipx_err("itask is NULL\n");
		ret = -ENOMEM;
		goto p_err;
	}

#ifdef DBG_TIMEMEASURE
	vipx_get_timestamp(&itask->time[VIPX_TMP_REQUEST]);
#endif

	itask->id = 0;
	itask->lock = &graph->local_lock;
	itask->findex = VIPX_MAX_TASK;
	itask->tdindex = VIPX_MAX_TASKDESC;
	itask->message = VIPX_TASK_ALLOCATE;
	itask->param0 = graph->uid;
	itask->param1 = graph->idx;
	itask->param2 = (ulong)&graph->informat_list;
	itask->param3 = (ulong)&graph->otformat_list;

	ret = vipx_hw_config(interface, itask);
	if (ret) {
		vipx_err("vipx_hw_config is fail(%d)\n", ret);
		goto p_err;
	}

#ifdef DBG_TIMEMEASURE
	vipx_iinfo("[TM] A : %ldus, %ldus\n", graph,
		VIPX_TIME_IN_US(itask->time[VIPX_TMP_PROCESS]) -
		VIPX_TIME_IN_US(itask->time[VIPX_TMP_REQUEST]),
		VIPX_TIME_IN_US(itask->time[VIPX_TMP_DONE]) -
		VIPX_TIME_IN_US(itask->time[VIPX_TMP_PROCESS]));
#endif

p_err:
	vipx_info("%s:\n", __func__);
	return ret;
}

static int __vipx_itf_process(struct vipx_interface *interface,
	struct vipx_graph *graph,
	struct vipx_task *task)
{
	int ret = 0;
	unsigned long flags;
	struct vipx_taskmgr *itaskmgr;
	struct vipx_task *itask;

	BUG_ON(!interface);
	BUG_ON(!graph);
	BUG_ON(!task);

	itaskmgr = &interface->taskmgr;

	taskmgr_e_barrier_irqs(itaskmgr, 0, flags);
	vipx_task_pick_fre_to_req(itaskmgr, &itask);
	taskmgr_x_barrier_irqr(itaskmgr, 0, flags);

	if (!itask) {
		vipx_err("itask is NULL\n");
		ret = -ENOMEM;
		goto p_err;
	}

	if (test_bit(VIPX_GRAPH_FLAG_UPDATE_PARAM, &graph->flags)) {
		ret = CALL_GOPS(graph, update_param, task);
		if (ret) {
			vipx_err("GOPS(update_param) is fail(%d)\n", ret);
			goto p_err;
		}

		clear_bit(VIPX_GRAPH_FLAG_UPDATE_PARAM, &graph->flags);
	}

	itask->id = task->id;
	itask->lock = &graph->local_lock;
	itask->findex = task->index;
	itask->tdindex = task->tdindex;
	itask->message = VIPX_TASK_PROCESS;
	itask->param0 = graph->uid;
	itask->param1 = graph->idx;
	itask->param2 = (ulong)0; /* return : DONE or NDONE */
	itask->param3 = 0; /* return : error code if param2 is NDONE */
	itask->flags = task->flags;
	itask->incl = task->incl;
	itask->otcl = task->otcl;

	ret = CALL_GOPS(graph, process, task);
	if (ret) {
		vipx_err("GOPS(process) is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vipx_hw_process(interface, itask);
	if (ret) {
		vipx_err("vipx_hw_process is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	vipx_dbg("%s:\n", __func__);
	return ret;
}

static void __vipx_graphmgr_sched(struct vipx_graphmgr *graphmgr)
{
	int ret = 0;
	struct vipx_graph *graph;
	struct vipx_task *task;
	struct vipx_taskdesc *ready, *request, *process, *prev, *next, *temp;
	struct vipx_interface *interface;

	interface = graphmgr->interface;
	ready = NULL;
	request = NULL;
	prev = NULL;
	next = NULL;

	mutex_lock(&graphmgr->tdlock);

	/* 1. priority order */
	while (1) {
		__vipx_taskdesc_g_ready(graphmgr, &ready);
		if (!ready)
			break;

		list_for_each_entry_safe(next, temp, &graphmgr->tdreq_list, list) {
			if (ready->priority > next->priority)
				break;

			prev = next;
			next = NULL;
		}

		__vipx_taskdesc_s_request(graphmgr, prev, ready, next);
	}

	/* 2. */
	list_for_each_entry_safe(request, temp, &graphmgr->tdreq_list, list) {
		graph = request->graph;
		task = request->task;

		__vipx_taskdesc_trans_req_to_pro(graphmgr, request);
	}

	mutex_unlock(&graphmgr->tdlock);

	/* 3. process graph */
	list_for_each_entry_safe(process, temp, &graphmgr->tdpro_list, list) {
		graph = process->graph;
		task = process->task;

		ret = __vipx_itf_process(interface, graph, task);
		if (ret) {
			vipx_err("__vipx_itf_process is fail(%d)\n", ret);

			ret = CALL_GOPS(graph, cancel, task);
			if (ret) {
				vipx_err("CALL_GOPS(cancel) is fail(%d)\n", ret);
				BUG();
			}

			process->graph = NULL;
			process->task = NULL;
			__vipx_taskdesc_trans_pro_to_fre(graphmgr, process);
			continue;
		}

		__vipx_taskdesc_trans_pro_to_com(graphmgr, process);
	}

	graphmgr->sched_cnt++;
	vipx_dbg("[%s:%d]\n", __func__, __LINE__);
}

static void vipx_graph_thread(struct kthread_work *work)
{
	int ret = 0;
	struct vipx_graphmgr *graphmgr;
	struct vipx_graph *graph;
	struct vipx_task *task;
	struct vipx_taskdesc *taskdesc, * temp;

	BUG_ON(!work);

	task = container_of(work, struct vipx_task, work);
	graph = task->owner;
	graphmgr = graph->cookie;

	switch (task->message) {
	case VIPX_TASK_REQUEST:
		ret = CALL_GOPS(graph, request, task);
		if (ret) {
			vipx_err("CALL_GOPS(request) is fail(%d)\n", ret);
			BUG();
		}

		__vipx_taskdesc_free_head(graphmgr, &taskdesc);
		if (!taskdesc) {
			vipx_err("taskdesc is NULL\n");
			BUG();
		}

		task->tdindex = taskdesc->index;
		taskdesc->graph = graph;
		taskdesc->task = task;
		taskdesc->priority = graph->priority;
		__vipx_taskdesc_trans_fre_to_rdy(graphmgr, taskdesc);
		break;
	case VIPX_CTRL_STOP:
		list_for_each_entry_safe(taskdesc, temp, &graphmgr->tdrdy_list, list) {
			if (taskdesc->graph->idx != graph->idx)
				continue;

			ret = CALL_GOPS(graph, cancel, taskdesc->task);
			if (ret) {
				vipx_err("CALL_GOPS(cancel) is fail(%d)\n", ret);
				BUG();
			}

			taskdesc->graph = NULL;
			taskdesc->task = NULL;
			__vipx_taskdesc_trans_rdy_to_fre(graphmgr, taskdesc);
		}

		mutex_lock(&graphmgr->tdlock);

		list_for_each_entry_safe(taskdesc, temp, &graphmgr->tdreq_list, list) {
			if (taskdesc->graph->idx != graph->idx)
				continue;

			ret = CALL_GOPS(graph, cancel, taskdesc->task);
			if (ret) {
				vipx_err("CALL_GOPS(cancel) is fail(%d)\n", ret);
				BUG();
			}

			taskdesc->graph = NULL;
			taskdesc->task = NULL;
			__vipx_taskdesc_trans_req_to_fre(graphmgr, taskdesc);
		}

		mutex_unlock(&graphmgr->tdlock);

		ret = CALL_GOPS(graph, control, task);
		if (ret) {
			vipx_err("CALL_GOPS(control) is fail(%d)\n", ret);
			BUG();
		}
		return;
	default:
		BUG();
		break;
	}

	__vipx_graphmgr_sched(graphmgr);
	vipx_dbg("[%s:%d]\n", __func__, __LINE__);
}

static void vipx_interface_thread(struct kthread_work *work)
{
	int ret = 0;
	u32 task_index;
	u32 taskdesc_index;
	unsigned long flag;
	struct vipx_graphmgr *graphmgr;
	struct vipx_graph *graph;
	struct vipx_taskmgr *itaskmgr;
	struct vipx_task *task, *itask;
	struct vipx_taskdesc *taskdesc;
	struct vipx_interface *interface;

	BUG_ON(!work);

	itask = container_of(work, struct vipx_task, work);
	interface = itask->owner;
	itaskmgr = &interface->taskmgr;
	graphmgr = interface->cookie;

	switch (itask->message) {
	case VIPX_TASK_ALLOCATE:
		task_index = itask->findex;
		taskdesc_index = itask->tdindex;

		if (taskdesc_index >= VIPX_MAX_TASKDESC) {
			vipx_err("taskdesc index(%d) is invalid\n", taskdesc_index);
			BUG();
		}

		if (task_index >= VIPX_MAX_TASK) {
			vipx_err("task index(%d) is invalid\n", task_index);
			BUG();
		}

		taskdesc = &graphmgr->taskdesc[taskdesc_index];
		if (taskdesc->state != VIPX_TASKDESC_STATE_ALLOC) {
			vipx_err("taskdesc state is invalid(%d)\n", taskdesc->state);
			vipx_taskdesc_print(graphmgr);
			BUG();
		}

		graph = taskdesc->graph;
		if (!graph) {
			vipx_err("graph is NULL(%d)\n", taskdesc_index);
			BUG();
		}

		task = taskdesc->task;
		if (!task) {
			vipx_err("task is NULL(%d)\n", taskdesc_index);
			BUG();
		}

		task->message = itask->message;
		task->param0 = itask->param2;
		task->param1 = itask->param3;

		/* return status check */
		if (task->param0) {
			vipx_err("allocation is fail(%ld, %ld)\n", task->param0, task->param1);

			ret = CALL_GOPS(graph, cancel, task);
			if (ret) {
				vipx_err("CALL_GOPS(cancel) is fail(%d)\n", ret);
				BUG();
			}

			/* taskdesc cleanup */
			mutex_lock(&graphmgr->tdlock);
			taskdesc->graph = NULL;
			taskdesc->task = NULL;
			__vipx_taskdesc_trans_alc_to_fre(graphmgr, taskdesc);
			mutex_unlock(&graphmgr->tdlock);
		} else {
			/* taskdesc transition */
			mutex_lock(&graphmgr->tdlock);
			__vipx_taskdesc_trans_alc_to_pro(graphmgr, taskdesc);
			mutex_unlock(&graphmgr->tdlock);
		}

		/* itask cleanup */
		taskmgr_e_barrier_irqs(itaskmgr, 0, flag);
		vipx_task_trans_com_to_fre(itaskmgr, itask);
		taskmgr_x_barrier_irqr(itaskmgr, 0, flag);
		break;
	case VIPX_TASK_PROCESS:
		task_index = itask->findex;
		taskdesc_index = itask->tdindex;

		if (taskdesc_index >= VIPX_MAX_TASKDESC) {
			vipx_err("taskdesc index(%d) is invalid\n", taskdesc_index);
			BUG();
		}

		if (task_index >= VIPX_MAX_TASK) {
			vipx_err("task index(%d) is invalid\n", task_index);
			BUG();
		}

		taskdesc = &graphmgr->taskdesc[taskdesc_index];
		if (taskdesc->state == VIPX_TASKDESC_STATE_FREE) {
			vipx_err("taskdesc state is FREE(%d)\n", taskdesc->state);
			vipx_taskdesc_print(graphmgr);

			/* itask cleanup */
			taskmgr_e_barrier_irqs(itaskmgr, 0, flag);
			vipx_task_trans_com_to_fre(itaskmgr, itask);
			taskmgr_x_barrier_irqr(itaskmgr, 0, flag);
			break;
		}

		if (taskdesc->state != VIPX_TASKDESC_STATE_COMPLETE) {
			vipx_err("taskdesc state is invalid(%d)\n", taskdesc->state);
			vipx_taskdesc_print(graphmgr);
			BUG();
		}

		graph = taskdesc->graph;
		if (!graph) {
			vipx_err("graph is NULL(%d)\n", taskdesc_index);
			BUG();
		}

		task = taskdesc->task;
		if (!task) {
			vipx_err("task is NULL(%d)\n", taskdesc_index);
			BUG();
		}

		task->message = itask->message;
		task->tdindex = VIPX_MAX_TASKDESC;
		task->param0 = itask->param2;
		task->param1 = itask->param3;

		ret = CALL_GOPS(graph, done, task);
		if (ret) {
			vipx_err("CALL_GOPS(done) is fail(%d)\n", ret);
			BUG();
		}

		/* taskdesc cleanup */
		taskdesc->graph = NULL;
		taskdesc->task = NULL;
		__vipx_taskdesc_trans_com_to_fre(graphmgr, taskdesc);

		/* itask cleanup */
		taskmgr_e_barrier_irqs(itaskmgr, 0, flag);
		vipx_task_trans_com_to_fre(itaskmgr, itask);
		taskmgr_x_barrier_irqr(itaskmgr, 0, flag);
		break;
	default:
		BUG();
		break;
	}

	__vipx_graphmgr_sched(graphmgr);
	vipx_dbg("[%s:%d]\n", __func__, __LINE__);
}

int vipx_graphmgr_grp_register(struct vipx_graphmgr *graphmgr, struct vipx_graph *graph)
{
	int ret = 0;
	u32 index;

	BUG_ON(!graphmgr);
	BUG_ON(!graph);

	mutex_lock(&graphmgr->mlock);
	for (index = 0; index < VIPX_MAX_GRAPH; index++) {
		if (!graphmgr->graph[index]) {
			graphmgr->graph[index] = graph;
			graph->idx = index;
			break;
		}
	}
	mutex_unlock(&graphmgr->mlock);

	if (index >= VIPX_MAX_GRAPH) {
		vipx_err("graph slot is lack\n");
		ret = -EINVAL;
		goto p_err;
	}

	kthread_init_work(&graph->control.work, vipx_graph_thread);
	for (index = 0; index < VIPX_MAX_TASK; ++index)
		kthread_init_work(&graph->taskmgr.task[index].work, vipx_graph_thread);

	graph->global_lock = &graphmgr->mlock;
	atomic_inc(&graphmgr->active_cnt);

p_err:
	vipx_info("[%s:%d]\n", __func__, __LINE__);
	return ret;
}

int vipx_graphmgr_grp_unregister(struct vipx_graphmgr *graphmgr, struct vipx_graph *graph)
{
	int ret = 0;

	BUG_ON(!graphmgr);
	BUG_ON(!graph);

	mutex_lock(&graphmgr->mlock);
	graphmgr->graph[graph->idx] = NULL;
	mutex_unlock(&graphmgr->mlock);

	atomic_dec(&graphmgr->active_cnt);

	vipx_info("[%s:%d]\n", __func__, __LINE__);
	return ret;
}

int vipx_graphmgr_grp_start(struct vipx_graphmgr *graphmgr, struct vipx_graph *graph)
{
	int ret = 0;

	mutex_lock(&graphmgr->mlock);
	if (test_bit(VIPX_GRAPHMGR_ENUM, &graphmgr->state)) {
		mutex_unlock(&graphmgr->mlock);
		goto p_skip_hw_init;
	}

	ret = __vipx_itf_init(graphmgr->interface, graph);
	if (ret) {
		mutex_unlock(&graphmgr->mlock);
		vipx_err("__vipx_itf_init is fail(%d)\n", ret);
		goto p_err;
	}

	set_bit(VIPX_GRAPHMGR_ENUM, &graphmgr->state);
	mutex_unlock(&graphmgr->mlock);

p_skip_hw_init:
#if 0
	ret = __vipx_itf_create(graphmgr->interface, graph);
	if (ret) {
		vipx_err("__vipx_itf_create is fail(%d)\n", ret);
		goto p_err;
	}
#endif
	ret = __vipx_itf_config(graphmgr->interface, graph);
	if (ret) {
		vipx_err("__vipx_itf_config is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	vipx_info("[%s:%d]\n", __func__, __LINE__);
	return ret;
}

int vipx_graphmgr_grp_stop(struct vipx_graphmgr *graphmgr, struct vipx_graph *graph)
{
	int ret = 0;

	ret = __vipx_itf_destroy(graphmgr->interface, graph);
	if (ret) {
		vipx_err("__vipx_itf_destroy is fail(%d)\n", ret);
		goto p_err;
	}

	mutex_lock(&graphmgr->mlock);
	if (test_bit(VIPX_GRAPHMGR_ENUM, &graphmgr->state) &&
			atomic_read(&graphmgr->active_cnt) == 1) {
		ret = __vipx_itf_deinit(graphmgr->interface, graph);
		if (ret) {
			mutex_unlock(&graphmgr->mlock);
			vipx_err("__vipx_itf_deinit is fail(%d)\n", ret);
			goto p_err;
		}

		clear_bit(VIPX_GRAPHMGR_ENUM, &graphmgr->state);
	}
	mutex_unlock(&graphmgr->mlock);

p_err:
	vipx_info("[%s:%d]\n", __func__, __LINE__);
	return ret;
}

int vipx_graphmgr_itf_register(struct vipx_graphmgr *graphmgr, struct vipx_interface *interface)
{
	int ret = 0;
	u32 index;

	BUG_ON(!graphmgr);
	BUG_ON(!interface);

	graphmgr->interface = interface;
	for (index = 0; index < VIPX_MAX_TASK; ++index)
		kthread_init_work(&interface->taskmgr.task[index].work, vipx_interface_thread);

	vipx_info("[%s:%d]\n", __func__, __LINE__);
	return ret;
}

int vipx_graphmgr_itf_unregister(struct vipx_graphmgr *graphmgr, struct vipx_interface *interface)
{
	int ret = 0;
	BUG_ON(!graphmgr);
	BUG_ON(!interface);

	graphmgr->interface = NULL;

	vipx_info("[%s:%d]\n", __func__, __LINE__);
	return ret;
}

void vipx_graphmgr_queue(struct vipx_graphmgr *graphmgr, struct vipx_task *task)
{
	BUG_ON(!graphmgr);
	BUG_ON(!task);

	kthread_queue_work(&graphmgr->worker, &task->work);
	vipx_dbg("[%s:%d]\n", __func__, __LINE__);
}

int vipx_graphmgr_probe(struct vipx_graphmgr *graphmgr)
{
	int ret = 0;
	u32 index;
	struct vipx_device *device = container_of(graphmgr, struct vipx_device, graphmgr);

	BUG_ON(!graphmgr);
	BUG_ON(!device);

	graphmgr->tick_cnt = 0;
	graphmgr->tick_pos = 0;
	graphmgr->sched_cnt = 0;
	graphmgr->sched_pos = 0;
	graphmgr->task_graph = NULL;
#ifdef USE_TASK_TIMER
	graphmgr->task_timer = NULL;
#endif
	atomic_set(&graphmgr->active_cnt, 0);
	mutex_init(&graphmgr->mlock);
	mutex_init(&graphmgr->tdlock);
	clear_bit(VIPX_GRAPHMGR_OPEN, &graphmgr->state);
	clear_bit(VIPX_GRAPHMGR_ENUM, &graphmgr->state);

	for (index = 0; index < VIPX_MAX_GRAPH; ++index)
		graphmgr->graph[index] = NULL;

	INIT_LIST_HEAD(&graphmgr->tdfre_list);
	INIT_LIST_HEAD(&graphmgr->tdrdy_list);
	INIT_LIST_HEAD(&graphmgr->tdreq_list);
	INIT_LIST_HEAD(&graphmgr->tdalc_list);
	INIT_LIST_HEAD(&graphmgr->tdpro_list);
	INIT_LIST_HEAD(&graphmgr->tdcom_list);

	graphmgr->tdfre_cnt = 0;
	graphmgr->tdrdy_cnt = 0;
	graphmgr->tdreq_cnt = 0;
	graphmgr->tdalc_cnt = 0;
	graphmgr->tdpro_cnt = 0;
	graphmgr->tdcom_cnt = 0;

	for (index = 0; index < VIPX_MAX_TASKDESC; ++index) {
		graphmgr->taskdesc[index].index = index;
		graphmgr->taskdesc[index].graph = NULL;
		graphmgr->taskdesc[index].task = NULL;
		graphmgr->taskdesc[index].state = VIPX_TASKDESC_STATE_INVALID;
		__vipx_taskdesc_s_free(graphmgr, &graphmgr->taskdesc[index]);
	}

	vipx_info("[%s:%d]\n", __func__, __LINE__);
	return ret;
}

int vipx_graphmgr_open(struct vipx_graphmgr *graphmgr)
{
	int ret = 0;
	char name[30];
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };

	BUG_ON(!graphmgr);

	kthread_init_worker(&graphmgr->worker);
	snprintf(name, sizeof(name), "vipx_graph");
	graphmgr->task_graph = kthread_run(kthread_worker_fn, &graphmgr->worker, name);
	if (IS_ERR_OR_NULL(graphmgr->task_graph)) {
		vipx_err("kthread_run is fail\n");
		ret = -EINVAL;
		goto p_err;
	}

	ret = sched_setscheduler_nocheck(graphmgr->task_graph, SCHED_FIFO, &param);
	if (ret) {
		vipx_err("sched_setscheduler_nocheck is fail(%d)\n", ret);
		goto p_err;
	}

#ifdef USE_TASK_TIMER
	snprintf(name, sizeof(name), "vipx_timer");
	graphmgr->task_timer = kthread_run(vipx_time_thread, graphmgr, name);
	if (IS_ERR_OR_NULL(graphmgr->task_timer)) {
		vipx_err("kthread_run is fail\n");
		ret = -EINVAL;
		goto p_err;
	}

	ret = sched_setscheduler_nocheck(graphmgr->task_timer, SCHED_FIFO, &param);
	if (ret) {
		vipx_err("sched_setscheduler_nocheck is fail(%d)\n", ret);
		goto p_err;
	}
#endif

	set_bit(VIPX_GRAPHMGR_OPEN, &graphmgr->state);

p_err:
	vipx_info("[%s:%d]\n", __func__, __LINE__);
	return ret;
}

int vipx_graphmgr_close(struct vipx_graphmgr *graphmgr)
{
	int ret = 0;

#ifdef USE_TASK_TIMER
	kthread_stop(graphmgr->task_timer);
#endif
	kthread_stop(graphmgr->task_graph);

	clear_bit(VIPX_GRAPHMGR_OPEN, &graphmgr->state);
	clear_bit(VIPX_GRAPHMGR_ENUM, &graphmgr->state);

	vipx_info("[%s:%d]\n", __func__, __LINE__);
	return ret;
}
