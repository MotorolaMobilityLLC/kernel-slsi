/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "vipx-debug.h"
#include "vipx-taskmgr.h"
#include "vipx-graphmgr.h"

#if 0
#define taskmgr_debug vipx_info
#else
#define taskmgr_debug(fmt, args...)
#endif

void vipx_task_s_free(struct vipx_taskmgr *taskmgr, struct vipx_task *task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);

	task->state = VIPX_TASK_STATE_FREE;

	list_add_tail(&task->list, &taskmgr->fre_list);
	taskmgr->fre_cnt++;
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_g_free(struct vipx_taskmgr *taskmgr, struct vipx_task **task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);

	if (taskmgr->fre_cnt &&
		(*task = container_of(taskmgr->fre_list.next, struct vipx_task, list))) {
		list_del(&(*task)->list);
		taskmgr->fre_cnt--;
		(*task)->state = VIPX_TASK_STATE_INVALID;
	} else {
		*task = NULL;
	}
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_free_head(struct vipx_taskmgr *taskmgr, struct vipx_task **task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);

	if (taskmgr->fre_cnt)
		*task = container_of(taskmgr->fre_list.next, struct vipx_task, list);
	else
		*task = NULL;
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_free_tail(struct vipx_taskmgr *taskmgr, struct vipx_task **task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);

	if (taskmgr->fre_cnt)
		*task = container_of(taskmgr->fre_list.prev, struct vipx_task, list);
	else
		*task = NULL;
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_print_free_list(struct vipx_taskmgr *taskmgr)
{
	DLOG_INIT();
	struct vipx_task *task, *temp;

	DLOG("[FRM] fre(%d, %d) :", taskmgr->id, taskmgr->fre_cnt);
	list_for_each_entry_safe(task, temp, &taskmgr->fre_list, list) {
		DLOG("%d->", task->index);
	}
	DLOG("X");

	vipx_info("%s\n", DLOG_OUT());
}

void vipx_task_s_request(struct vipx_taskmgr *taskmgr, struct vipx_task *task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);

	task->state = VIPX_TASK_STATE_REQUEST;

	list_add_tail(&task->list, &taskmgr->req_list);
	taskmgr->req_cnt++;
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_g_request(struct vipx_taskmgr *taskmgr, struct vipx_task **task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);

	if (taskmgr->req_cnt &&
		(*task = container_of(taskmgr->req_list.next, struct vipx_task, list))) {
		list_del(&(*task)->list);
		taskmgr->req_cnt--;
		(*task)->state = VIPX_TASK_STATE_INVALID;
	} else {
		*task = NULL;
	}
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_request_head(struct vipx_taskmgr *taskmgr, struct vipx_task **task)
{
	if (taskmgr->req_cnt)
		*task = container_of(taskmgr->req_list.next, struct vipx_task, list);
	else
		*task = NULL;
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_request_tail(struct vipx_taskmgr *taskmgr, struct vipx_task **task)
{
	if (taskmgr->req_cnt)
		*task = container_of(taskmgr->req_list.prev, struct vipx_task, list);
	else
		*task = NULL;
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_print_request_list(struct vipx_taskmgr *taskmgr)
{
	DLOG_INIT();
	struct vipx_task *task, *temp;

	DLOG("[FRM] req(%d, %d) :", taskmgr->id, taskmgr->req_cnt);
	list_for_each_entry_safe(task, temp, &taskmgr->req_list, list) {
		DLOG("%d->", task->index);
	}
	DLOG("X");

	vipx_info("%s\n", DLOG_OUT());
}

void vipx_task_s_prepare(struct vipx_taskmgr *taskmgr, struct vipx_task *task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);

	task->state = VIPX_TASK_STATE_PREPARE;

	list_add_tail(&task->list, &taskmgr->pre_list);
	taskmgr->pre_cnt++;
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_g_prepare(struct vipx_taskmgr *taskmgr, struct vipx_task **task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);

	if (taskmgr->pre_cnt &&
		(*task = container_of(taskmgr->pre_list.next, struct vipx_task, list))) {
		list_del(&(*task)->list);
		taskmgr->pre_cnt--;
		(*task)->state = VIPX_TASK_STATE_INVALID;
	} else {
		*task = NULL;
	}
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_prepare_head(struct vipx_taskmgr *taskmgr, struct vipx_task **task)
{
	if (taskmgr->pre_cnt)
		*task = container_of(taskmgr->pre_list.next, struct vipx_task, list);
	else
		*task = NULL;
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_prepare_tail(struct vipx_taskmgr *taskmgr, struct vipx_task **task)
{
	if (taskmgr->pre_cnt)
		*task = container_of(taskmgr->pre_list.prev, struct vipx_task, list);
	else
		*task = NULL;
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_print_prepare_list(struct vipx_taskmgr *taskmgr)
{
	DLOG_INIT();
	struct vipx_task *task, *temp;

	DLOG("[FRM] pre(%d, %d) :", taskmgr->id, taskmgr->pre_cnt);
	list_for_each_entry_safe(task, temp, &taskmgr->pre_list, list) {
		DLOG("%d->", task->index);
	}
	DLOG("X");

	vipx_info("%s\n", DLOG_OUT());
}

void vipx_task_s_process(struct vipx_taskmgr *taskmgr, struct vipx_task *task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);

	task->state = VIPX_TASK_STATE_PROCESS;

	list_add_tail(&task->list, &taskmgr->pro_list);
	taskmgr->pro_cnt++;
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_g_process(struct vipx_taskmgr *taskmgr, struct vipx_task **task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);

	if (taskmgr->pro_cnt &&
		(*task = container_of(taskmgr->pro_list.next, struct vipx_task, list))) {
		list_del(&(*task)->list);
		taskmgr->pro_cnt--;
		(*task)->state = VIPX_TASK_STATE_INVALID;
	} else {
		*task = NULL;
	}
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_process_head(struct vipx_taskmgr *taskmgr, struct vipx_task **task)
{
	if (taskmgr->pro_cnt)
		*task = container_of(taskmgr->pro_list.next, struct vipx_task, list);
	else
		*task = NULL;
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_process_tail(struct vipx_taskmgr *taskmgr, struct vipx_task **task)
{
	if (taskmgr->pro_cnt)
		*task = container_of(taskmgr->pro_list.prev, struct vipx_task, list);
	else
		*task = NULL;
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_print_process_list(struct vipx_taskmgr *taskmgr)
{
	DLOG_INIT();
	struct vipx_task *task, *temp;

	DLOG("[FRM] pro(%d, %d) :", taskmgr->id, taskmgr->pro_cnt);
	list_for_each_entry_safe(task, temp, &taskmgr->pro_list, list) {
		DLOG("%d->", task->index);
	}
	DLOG("X");

	vipx_info("%s\n", DLOG_OUT());
}

void vipx_task_s_complete(struct vipx_taskmgr *taskmgr, struct vipx_task *task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);

	task->state = VIPX_TASK_STATE_COMPLETE;

	list_add_tail(&task->list, &taskmgr->com_list);
	taskmgr->com_cnt++;
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_g_complete(struct vipx_taskmgr *taskmgr, struct vipx_task **task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);

	if (taskmgr->com_cnt &&
		(*task = container_of(taskmgr->com_list.next, struct vipx_task, list))) {
		list_del(&(*task)->list);
		taskmgr->com_cnt--;
		(*task)->state = VIPX_TASK_STATE_INVALID;
	} else {
		*task = NULL;
	}
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_complete_head(struct vipx_taskmgr *taskmgr, struct vipx_task **task)
{
	if (taskmgr->com_cnt)
		*task = container_of(taskmgr->com_list.next, struct vipx_task, list);
	else
		*task = NULL;
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_complete_tail(struct vipx_taskmgr *taskmgr, struct vipx_task **task)
{
	if (taskmgr->com_cnt)
		*task = container_of(taskmgr->com_list.prev, struct vipx_task, list);
	else
		*task = NULL;
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_print_complete_list(struct vipx_taskmgr *taskmgr)
{
	DLOG_INIT();
	struct vipx_task *task, *temp;

	DLOG("[FRM] com(%d, %d) :", taskmgr->id, taskmgr->com_cnt);
	list_for_each_entry_safe(task, temp, &taskmgr->com_list, list) {
		DLOG("%d->", task->index);
	}
	DLOG("X");

	vipx_info("%s\n", DLOG_OUT());
}

void vipx_task_trans_fre_to_req(struct vipx_taskmgr *taskmgr, struct vipx_task *task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);
	BUG_ON(!taskmgr->fre_cnt);
	BUG_ON(task->state != VIPX_TASK_STATE_FREE);

	list_del(&task->list);
	taskmgr->fre_cnt--;
	vipx_task_s_request(taskmgr, task);
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_trans_req_to_pre(struct vipx_taskmgr *taskmgr, struct vipx_task *task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);
	BUG_ON(!taskmgr->req_cnt);
	BUG_ON(task->state != VIPX_TASK_STATE_REQUEST);

	list_del(&task->list);
	taskmgr->req_cnt--;
	vipx_task_s_prepare(taskmgr, task);
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_trans_req_to_pro(struct vipx_taskmgr *taskmgr, struct vipx_task *task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);
	BUG_ON(!taskmgr->req_cnt);
	BUG_ON(task->state != VIPX_TASK_STATE_REQUEST);

	list_del(&task->list);
	taskmgr->req_cnt--;
	vipx_task_s_process(taskmgr, task);
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_trans_req_to_com(struct vipx_taskmgr *taskmgr, struct vipx_task *task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);
	BUG_ON(!taskmgr->req_cnt);
	BUG_ON(task->state != VIPX_TASK_STATE_REQUEST);

	list_del(&task->list);
	taskmgr->req_cnt--;
	vipx_task_s_complete(taskmgr, task);
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_trans_req_to_fre(struct vipx_taskmgr *taskmgr, struct vipx_task *task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);
	BUG_ON(!taskmgr->req_cnt);
	BUG_ON(task->state != VIPX_TASK_STATE_REQUEST);

	list_del(&task->list);
	taskmgr->req_cnt--;
	vipx_task_s_free(taskmgr, task);
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_trans_pre_to_pro(struct vipx_taskmgr *taskmgr, struct vipx_task *task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);
	BUG_ON(!taskmgr->pre_cnt);
	BUG_ON(task->state != VIPX_TASK_STATE_PREPARE);

	list_del(&task->list);
	taskmgr->pre_cnt--;
	vipx_task_s_process(taskmgr, task);
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_trans_pre_to_com(struct vipx_taskmgr *taskmgr, struct vipx_task *task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);
	BUG_ON(!taskmgr->pre_cnt);
	BUG_ON(task->state != VIPX_TASK_STATE_PREPARE);

	list_del(&task->list);
	taskmgr->pre_cnt--;
	vipx_task_s_complete(taskmgr, task);
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_trans_pre_to_fre(struct vipx_taskmgr *taskmgr, struct vipx_task *task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);
	BUG_ON(!taskmgr->pre_cnt);
	BUG_ON(task->state != VIPX_TASK_STATE_PREPARE);

	list_del(&task->list);
	taskmgr->pre_cnt--;
	vipx_task_s_free(taskmgr, task);
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_trans_pro_to_com(struct vipx_taskmgr *taskmgr, struct vipx_task *task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);
	BUG_ON(!taskmgr->pro_cnt);
	BUG_ON(task->state != VIPX_TASK_STATE_PROCESS);

	list_del(&task->list);
	taskmgr->pro_cnt--;
	vipx_task_s_complete(taskmgr, task);
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_trans_pro_to_fre(struct vipx_taskmgr *taskmgr, struct vipx_task *task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);
	BUG_ON(!taskmgr->pro_cnt);
	BUG_ON(task->state != VIPX_TASK_STATE_PROCESS);

	list_del(&task->list);
	taskmgr->pro_cnt--;
	vipx_task_s_free(taskmgr, task);
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_trans_com_to_fre(struct vipx_taskmgr *taskmgr, struct vipx_task *task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);
	BUG_ON(!taskmgr->com_cnt);
	BUG_ON(task->state != VIPX_TASK_STATE_COMPLETE);

	list_del(&task->list);
	taskmgr->com_cnt--;
	vipx_task_s_free(taskmgr, task);
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_trans_any_to_fre(struct vipx_taskmgr *taskmgr, struct vipx_task *task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);

	list_del(&task->list);
	switch (task->state) {
	case VIPX_TASK_STATE_REQUEST:
		taskmgr->req_cnt--;
		break;
	case VIPX_TASK_STATE_PREPARE:
		taskmgr->pre_cnt--;
		break;
	case VIPX_TASK_STATE_PROCESS:
		taskmgr->pro_cnt--;
		break;
	case VIPX_TASK_STATE_COMPLETE:
		taskmgr->com_cnt--;
		break;
	default:
		BUG();
		break;
	}

	vipx_task_s_free(taskmgr, task);
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_pick_fre_to_req(struct vipx_taskmgr *taskmgr, struct vipx_task **task)
{
	BUG_ON(!taskmgr);
	BUG_ON(!task);

	vipx_task_free_head(taskmgr, task);
	if (*task)
		vipx_task_trans_fre_to_req(taskmgr, *task);
	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
}

void vipx_task_print_all(struct vipx_taskmgr *taskmgr)
{
	BUG_ON(!taskmgr);

	vipx_task_print_request_list(taskmgr);
	vipx_task_print_prepare_list(taskmgr);
	vipx_task_print_process_list(taskmgr);
	vipx_task_print_complete_list(taskmgr);
}

int vipx_task_init(struct vipx_taskmgr *taskmgr, void *owner)
{
	int ret = 0;
	unsigned long flags;
	u32 index;

	spin_lock_irqsave(&taskmgr->slock, flags);

	INIT_LIST_HEAD(&taskmgr->fre_list);
	INIT_LIST_HEAD(&taskmgr->req_list);
	INIT_LIST_HEAD(&taskmgr->pre_list);
	INIT_LIST_HEAD(&taskmgr->pro_list);
	INIT_LIST_HEAD(&taskmgr->com_list);

	taskmgr->tot_cnt = VIPX_MAX_TASK;
	taskmgr->fre_cnt = 0;
	taskmgr->req_cnt = 0;
	taskmgr->pre_cnt = 0;
	taskmgr->pro_cnt = 0;
	taskmgr->com_cnt = 0;

	/* task index 0 means invalid because firmware can't accept 0 invocation id */
	for (index = 1; index < taskmgr->tot_cnt; ++index) {
		taskmgr->task[index].index = index;
		taskmgr->task[index].owner = owner;
		taskmgr->task[index].findex = VIPX_MAX_TASK;
		taskmgr->task[index].tdindex = VIPX_MAX_TASKDESC;
		vipx_task_s_free(taskmgr, &taskmgr->task[index]);
	}

	spin_unlock_irqrestore(&taskmgr->slock, flags);

	return ret;
}

int vipx_task_deinit(struct vipx_taskmgr *taskmgr)
{
	int ret = 0;
	unsigned long flags;
	u32 index;

	spin_lock_irqsave(&taskmgr->slock, flags);

	for (index = 0; index < taskmgr->tot_cnt; ++index)
		vipx_task_s_free(taskmgr, &taskmgr->task[index]);

	spin_unlock_irqrestore(&taskmgr->slock, flags);

	taskmgr_debug("[TASKMGR] %s[%d]\n", __func__, __LINE__);
	return ret;
}

void vipx_task_flush(struct vipx_taskmgr *taskmgr)
{
	unsigned long flag;
	struct vipx_task *task, *temp;

	BUG_ON(!taskmgr);

	spin_lock_irqsave(&taskmgr->slock, flag);

	list_for_each_entry_safe(task, temp, &taskmgr->req_list, list) {
		vipx_task_trans_req_to_fre(taskmgr, task);
		vipx_warn("req task%d is flushed(count : %d)", task->id, taskmgr->req_cnt);
	}

	list_for_each_entry_safe(task, temp, &taskmgr->pre_list, list) {
		vipx_task_trans_pre_to_fre(taskmgr, task);
		vipx_warn("pre task%d is flushed(count : %d)", task->id, taskmgr->pre_cnt);
	}

	list_for_each_entry_safe(task, temp, &taskmgr->pro_list, list) {
		vipx_task_trans_pro_to_fre(taskmgr, task);
		vipx_warn("pro task%d is flushed(count : %d)", task->id, taskmgr->pro_cnt);
	}

	list_for_each_entry_safe(task, temp, &taskmgr->com_list, list) {
		vipx_task_trans_com_to_fre(taskmgr, task);
		vipx_warn("com task%d is flushed(count : %d)", task->id, taskmgr->com_cnt);
	}

	spin_unlock_irqrestore(&taskmgr->slock, flag);

	BUG_ON(taskmgr->fre_cnt != (taskmgr->tot_cnt - 1));
}
