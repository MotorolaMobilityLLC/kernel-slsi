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
#include <linux/random.h>
#include <linux/slab.h>

#include "vipx-config.h"
#include "vipx-interface.h"
#include "vipx-memory.h"
#include "vipx-mailbox.h"
#include "vipx-io.h"
#include "../vipx-graphmgr.h"
#include "../vipx-device.h"

#include <interface/ap_vip_if.h>

#define TURN_AROUND_TIME (HZ/20)

#define enter_request_barrier(task) mutex_lock(task->lock);
#define exit_request_barrier(task) mutex_unlock(task->lock);
#define init_process_barrier(itf) spin_lock_init(&itf->process_barrier);
#define enter_process_barrier(itf) spin_lock_irq(&itf->process_barrier);
#define exit_process_barrier(itf) spin_unlock_irq(&itf->process_barrier);

#ifdef DBG_INTERFACE_ISR
#define DEBUG_ISR vipx_info
#else
#define DEBUG_ISR vipx_dbg
#endif

/* #define NON_BLOCK_INVOKE */

atomic_t checker;

int __get_size_from(struct vipx_format *format, unsigned int plane, int *width, int *height)
{
	int ret = 0;

	switch (format->format) {
		case VS4L_DF_IMAGE_U8:
		case VS4L_DF_IMAGE_U16:
		case VS4L_DF_IMAGE_RGB:
		case VS4L_DF_IMAGE_U32:
			*width = format->width;
			*height = format->height;
			break;
		case VS4L_DF_IMAGE_NV21:
			if (plane == 0) {
				*width = format->width;
				*height = format->height;
			} else if (plane == 1) {
				*width = format->width;
				*height = format->height / 2;
			} else {
				vipx_err("Invalid plane(%d) for NV21 format\n", plane);
				ret = -1;
			}
			break;
		case VS4L_DF_IMAGE_YV12:
		case VS4L_DF_IMAGE_I420:
			if (plane == 0) {
				*width = format->width;
				*height = format->height;
			} else if (plane == 1) {
				*width = format->width / 2;
				*height = format->height / 2;
			} else if (plane == 2) {
				*width = format->width / 2;
				*height = format->height / 2;
			} else {
				vipx_err("Invalid plane(%d) for YV12/I420 format\n", plane);
				ret = -1;
			}
			break;
		case VS4L_DF_IMAGE_I422:
			if (plane == 0) {
				*width = format->width;
				*height = format->height;
			} else if (plane == 1) {
				*width = format->width / 2;
				*height = format->height;
			} else if (plane == 2) {
				*width = format->width / 2;
				*height = format->height;
			} else {
				vipx_err("Invalid plane(%d) for YV12/I422 format\n", plane);
				ret = -1;
			}
			break;
		default:
			vipx_err("invalid format(%d)\n", format->format);
			ret = -1;
			break;
	}

	return ret;
}

static void WORK_TO_FREE(struct vipx_work_list *work_list, struct vipx_work *work)
{
	unsigned long flags;

	BUG_ON(!work_list);
	BUG_ON(!work);

	spin_lock_irqsave(&work_list->slock, flags);
	list_add_tail(&work->list, &work_list->free_head);
	work_list->free_cnt++;
	spin_unlock_irqrestore(&work_list->slock, flags);

	vipx_dbg("%s:\n", __func__);
}

static void WORK_FR_FREE(struct vipx_work_list *work_list, struct vipx_work **work)
{
	unsigned long flags;

	BUG_ON(!work_list);
	BUG_ON(!work);

	spin_lock_irqsave(&work_list->slock, flags);
	if (work_list->free_cnt) {
		*work = container_of(work_list->free_head.next, struct vipx_work, list);
		list_del(&(*work)->list);
		work_list->free_cnt--;
	} else {
		*work = NULL;
	}
	spin_unlock_irqrestore(&work_list->slock, flags);

	vipx_dbg("%s:\n", __func__);
}

static void WORK_TO_REPLY(struct vipx_work_list *work_list, struct vipx_work *work)
{
	unsigned long flags;

	BUG_ON(!work_list);
	BUG_ON(!work);

	spin_lock_irqsave(&work_list->slock, flags);
	list_add_tail(&work->list, &work_list->reply_head);
	work_list->reply_cnt++;
	spin_unlock_irqrestore(&work_list->slock, flags);

	vipx_dbg("%s:\n", __func__);
}

static void WORK_FR_REPLY(struct vipx_work_list *work_list, struct vipx_work **work)
{
	unsigned long flags;

	BUG_ON(!work_list);
	BUG_ON(!work);

	spin_lock_irqsave(&work_list->slock, flags);
	if (work_list->reply_cnt) {
		*work = container_of(work_list->reply_head.next, struct vipx_work, list);
		list_del(&(*work)->list);
		work_list->reply_cnt--;
	} else {
		*work = NULL;
	}
	spin_unlock_irqrestore(&work_list->slock, flags);

	vipx_dbg("%s:\n", __func__);
}

static void INIT_WORK_LIST(struct vipx_work_list *work_list, u32 count)
{
	u32 i;

	work_list->free_cnt = 0;
	work_list->reply_cnt = 0;
	INIT_LIST_HEAD(&work_list->free_head);
	INIT_LIST_HEAD(&work_list->reply_head);
	spin_lock_init(&work_list->slock);
	init_waitqueue_head(&work_list->wait_queue);
	for (i = 0; i < count; ++i)
		WORK_TO_FREE(work_list, &work_list->work[i]);

	vipx_dbg("%s:\n", __func__);
}

static inline void __set_reply(struct vipx_interface *interface, u32 graph_idx)
{
	BUG_ON(graph_idx >= VIPX_MAX_GRAPH);
	interface->reply[graph_idx].valid = 1;
	wake_up(&interface->reply_queue);

	vipx_dbg("%s:\n", __func__);
}

static inline void __clr_reply(struct vipx_interface *interface, u32 graph_idx)
{
	BUG_ON(graph_idx >= VIPX_MAX_GRAPH);
	interface->reply[graph_idx].valid = 0;

	vipx_dbg("%s:\n", __func__);
}

static int __wait_reply(struct vipx_interface *interface, u32 graph_idx)
{
	int ret = 0;
	int remain_time;
	u32 cmd, type;

	BUG_ON(!interface);
	BUG_ON(graph_idx >= VIPX_MAX_GRAPH);
	BUG_ON(!interface->request[graph_idx]);

	cmd = interface->request[graph_idx]->message;
	type = interface->request[graph_idx]->param3;

#ifdef NON_BLOCK_INVOKE
	remain_time = wait_event_timeout(interface->reply_queue,
		interface->reply[graph_idx].valid, VIPX_COMMAND_TIMEOUT);
#else
	remain_time = wait_event_timeout(interface->reply_queue,
		interface->reply[graph_idx].valid, VIPX_COMMAND_TIMEOUT * 2);
#endif
	if (!remain_time) {
		vipx_err("[GID:%d] %d command[type: %d] : reply is timeout\n", graph_idx, cmd, type);
		ret = -ETIME;
		goto p_err;
	}

p_err:

	vipx_dbg("%s:%d\n", __func__, ret);
	return ret;
}

static void __send_interrupt(struct vipx_interface *interface)
{
#ifndef CONFIG_EXYNOS_EMUL_MBOX
	u32 try_count;
	u32 type, offset, val;

	BUG_ON(!interface);
	BUG_ON(!interface->process);

	type = interface->process->param3;
	offset = (type == VIPX_MTYPE_H2F_NORMAL) ? 0x10 : 0x0C;


	/* Check interrupt clear */
	try_count = 100;
	val = readl(interface->regs + offset);
	while (--try_count && val) {
		vipx_warn("waiting interrupt clear(%d)...(%d)\n", val, try_count);
		val = readl(interface->regs + offset);
	}

	DEBUG_ISR("[DEBUG ISR] interrupt generate (%x)\n", offset);

	/* Raise interrupt */
	writel(0x100, interface->regs + offset);
#endif

	vipx_dbg("%s:\n", __func__);
}

static int __vipx_set_cmd(struct vipx_interface *interface,
	struct vipx_task *itask)
{
	int ret = 0;
	struct vipx_taskmgr *itaskmgr;
	struct vipx_mailbox_ctrl *mctrl;
	void *payload;
	u32 cmd, gidx, cid, size, type;
	ulong flag;

	BUG_ON(!interface);
	BUG_ON(!itask);
	BUG_ON(!itask->lock);
	BUG_ON(itask->param1 >= VIPX_MAX_GRAPH);

	itaskmgr = &interface->taskmgr;
	mctrl = interface->private_data;
	cmd = itask->message;
	payload = (void *)itask->param0;
	gidx = itask->param1;
	size = itask->param2;
	type = itask->param3;
	cid = itask->index;

	enter_request_barrier(itask);
	interface->request[gidx] = itask;

	enter_process_barrier(interface);
	interface->process = itask;

	ret = vipx_mbox_ready(mctrl, size, type);
	if (ret) {
		interface->process = NULL;
		exit_process_barrier(interface);
		interface->request[gidx] = NULL;
		exit_request_barrier(itask);
		vipx_err("vipx_mbox_ready is fail(%d)", ret);
		goto p_err;
	}

	taskmgr_e_barrier_irqs(itaskmgr, 0, flag);
	vipx_task_trans_req_to_pro(itaskmgr, itask);
	taskmgr_x_barrier_irqr(itaskmgr, 0, flag);

	ret = vipx_mbox_write(mctrl, payload, size, type, gidx, cmd, cid);
	if (ret) {
		interface->process = NULL;
		exit_process_barrier(interface);
		interface->request[gidx] = NULL;
		exit_request_barrier(itask);
		pr_err("vipx_mbox_write_request fail (%d)\n", ret);
		goto p_err;
	}

	atomic_inc(&checker);

	__send_interrupt(interface);

	DEBUG_ISR("[I%ld][MBOX] CMD : %d, ID : %d\n", itask->param1, cmd, cid);

	interface->process = NULL;
	exit_process_barrier(interface);

	ret = __wait_reply(interface, gidx);
	if (ret) {
		interface->request[gidx] = NULL;
		exit_request_barrier(itask);
		vipx_err("%d command is timeout", cmd);
		ret = -ETIME;
		goto p_err;
	}

	if (interface->reply[gidx].work_param1) {
		interface->request[gidx] = NULL;
		exit_request_barrier(itask);
		vipx_err("%d command is error(%d)\n", cmd, interface->reply[gidx].work_param1);
		ret = interface->reply[gidx].work_param1;
		goto p_err;
	}

	__clr_reply(interface, gidx);

	interface->request[gidx] = NULL;
	exit_request_barrier(itask);

p_err:

	vipx_dbg("%s:%d\n", __func__, ret);
	return ret;
}

#ifdef NON_BLOCK_INVOKE
static int __vipx_set_cmd_nblk(struct vipx_interface *interface,
	struct vipx_task *itask)
{
	int ret = 0;
	struct vipx_taskmgr *itaskmgr;
	struct vipx_mailbox_ctrl *mctrl;
	void *payload;
	u32 cmd, gidx, cid, size, type;
	ulong flag;

	BUG_ON(!interface);
	BUG_ON(!itask);

	itaskmgr = &interface->taskmgr;
	mctrl = interface->private_data;
	cmd = itask->message;
	payload = (void *)itask->param0;
	gidx = itask->param1;
	size = itask->param2;
	type = itask->param3;
	cid = itask->index;

	enter_process_barrier(interface);
	interface->process = itask;

	ret = vipx_mbox_ready(mctrl, size, type);
	if (ret) {
		interface->process = NULL;
		exit_process_barrier(interface);
		vipx_err("vipx_mbox_ready is fail(%d)", ret);
		goto p_err;
	}

	taskmgr_e_barrier_irqs(itaskmgr, 0, flag);
	vipx_task_trans_req_to_pro(itaskmgr, itask);
	taskmgr_x_barrier_irqr(itaskmgr, 0, flag);

	ret = vipx_mbox_write(mctrl, payload, size, type, gidx, cmd, cid);
	if (ret) {
		interface->process = NULL;
		exit_process_barrier(interface);
		pr_err("vipx_mbox_write_request fail (%d)\n", ret);
		goto p_err;
	}

	atomic_inc(&checker);

	__send_interrupt(interface);

	DEBUG_ISR("[I%ld][MBOX] CMD : %d, ID : %d\n", itask->param1, cmd, cid);

	interface->process = NULL;
	exit_process_barrier(interface);

p_err:

	vipx_dbg("%s:%d\n", __func__, ret);
	return ret;
}
#endif

static int __vipx_interface_cleanup(struct vipx_interface *interface)
{
	int ret = 0;
	struct vipx_taskmgr *taskmgr;
	struct vipx_task *task;
	ulong flags;
	u32 i;

	taskmgr = &interface->taskmgr;

	if (taskmgr->req_cnt) {
		vipx_err("[ITF] request count is NOT zero(%d)\n", taskmgr->req_cnt);
		ret += taskmgr->req_cnt;
	}

	if (taskmgr->pro_cnt) {
		vipx_err("[ITF] process count is NOT zero(%d)\n", taskmgr->pro_cnt);
		ret += taskmgr->pro_cnt;
	}

	if (taskmgr->com_cnt) {
		vipx_err("[ITF] complete count is NOT zero(%d)\n", taskmgr->com_cnt);
		ret += taskmgr->com_cnt;
	}

	if (ret) {
		taskmgr_e_barrier_irqs(taskmgr, 0, flags);

		for (i = 1; i < taskmgr->tot_cnt; ++i) {
			task = &taskmgr->task[i];
			if (task->state == VIPX_TASK_STATE_FREE)
				continue;

			vipx_task_trans_any_to_fre(taskmgr, task);
			ret--;
		}

		taskmgr_x_barrier_irqr(taskmgr, 0, flags);
	}

	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

void vipx_interface_print(struct vipx_interface *interface)
{
	DLOG_INIT();
	struct vipx_taskmgr *itaskmgr;
	struct vipx_task *itask, *itemp;

	BUG_ON(!interface);

	itaskmgr = &interface->taskmgr;

	DLOG("REQUEST LIST(%d) :", itaskmgr->req_cnt);
	list_for_each_entry_safe(itask, itemp, &itaskmgr->req_list, list) {
		DLOG(" %d(%d, %d)", itask->index, itask->param1, itask->message);
	}
	vipx_info("%s\n", DLOG_OUT());

	DLOG("PROCESS LIST(%d) :", itaskmgr->pro_cnt);
	list_for_each_entry_safe(itask, itemp, &itaskmgr->pro_list, list) {
		DLOG(" %d(%d, %d)", itask->index, itask->param1, itask->message);
	}
	vipx_info("%s\n", DLOG_OUT());

	DLOG("COMPLETE LIST(%d) :", itaskmgr->com_cnt);
	list_for_each_entry_safe(itask, itemp, &itaskmgr->com_list, list) {
		DLOG(" %d(%d, %d)", itask->index, itask->param1, itask->message);
	}
	vipx_info("%s\n", DLOG_OUT());
}

static irqreturn_t interface_isr(int irq, void *data)
{
	int ret = 0;
	struct vipx_interface *interface;
	struct vipx_mailbox_ctrl *mctrl;
	struct vipx_mailbox_f2h *mbox;
	struct work_struct *work_queue;
	struct vipx_work_list *work_list;
	struct vipx_work *work;
	vipx_msg_t msg_rsp;

	interface = (struct vipx_interface *)data;
	mctrl = interface->private_data;
	work_queue = &interface->work_queue;
	work_list = &interface->work_list;

	mbox = &mctrl->urgent_stack->f2h;
	DEBUG_ISR("[DEBUG_ISR] status of urgent mbox w(%d) r(%d)\n", mbox->wmsg_idx, mbox->rmsg_idx);
	while (mbox->wmsg_idx != mbox->rmsg_idx) {
		ret = vipx_mbox_read(mctrl, &msg_rsp, VIPX_MTYPE_H2F_URGENT, data);
		if (ret) {
			vipx_err("vipx_mbox_read is fail(%d)\n", ret);
			break;
		}
		DEBUG_ISR("[DEBUG_ISR] read urgent mbox transid %d\n", msg_rsp.transId);

		if (msg_rsp.type == BOOTUP_RSP) {
			if (msg_rsp.msg_u.bootup_rsp.error == 0) {
				vipx_info("CM7 bootup complete %d\n", msg_rsp.transId);
				set_bit(VIPX_ITF_STATE_BOOTUP, &interface->state);
			}
			break;
		}

		atomic_dec(&checker);

		WORK_FR_FREE(work_list, &work);
		if (work) {
			/* TODO */
			work->id = msg_rsp.transId;
			work->message = 0;
			work->work_param0 = 0;
			work->work_param1 = 0;
			work->work_param2 = 0;
			work->work_param3 = 0;

			WORK_TO_REPLY(work_list, work);
			if (!work_pending(work_queue))
				schedule_work(work_queue);
		} else {
			vipx_err("free work is empty\n");
			break;
		}
		break;
	}

	mbox = &mctrl->stack->f2h;
	DEBUG_ISR("[DEBUG_ISR] status of normal mbox w(%d) r(%d)\n", mbox->wmsg_idx, mbox->rmsg_idx);
	while (mbox->wmsg_idx != mbox->rmsg_idx) {
		ret = vipx_mbox_read(mctrl, &msg_rsp, VIPX_MTYPE_H2F_NORMAL, data);
		if (ret) {
			vipx_err("vipx_mbox_read is fail(%d)\n", ret);
			break;
		}
		DEBUG_ISR("[DEBUG_ISR] read normal mbox transid %d\n", msg_rsp.transId);

		atomic_dec(&checker);

		WORK_FR_FREE(work_list, &work);
		if (work) {
			/* TODO */
			work->id = msg_rsp.transId;
			work->message = 0;
			work->work_param0 = 0;
			work->work_param1 = 0;
			work->work_param2 = 0;
			work->work_param3 = 0;

			WORK_TO_REPLY(work_list, work);
			if (!work_pending(work_queue))
				schedule_work(work_queue);
		} else {
			vipx_err("free work is empty\n");
			break;
		}
		break;
	}

	return IRQ_HANDLED;
}

static irqreturn_t interface_isr0(int irq, void *data)
{
	struct vipx_interface *interface = (struct vipx_interface *)data;
	u32 val;

	val = readl(interface->regs + 0x8);
	if (val & 0x1) {
		val &= ~(0x1);
		writel(val, interface->regs + 0x8);
		interface_isr(irq, data);
	}

	return IRQ_HANDLED;
}

static irqreturn_t interface_isr1(int irq, void *data)
{
	struct vipx_interface *interface = (struct vipx_interface *)data;
	u32 val;

	val = readl(interface->regs + 0x8);
	if (val & (0x1 << 0x1)) {
		val &= ~(0x1 << 0x1);
		writel(val, interface->regs + 0x8);
		interface_isr(irq, data);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_EXYNOS_EMUL_MBOX
static void interface_timer(unsigned long data)
{
	struct vipx_interface *interface = (struct vipx_interface *)data;
	struct vipx_mailbox_ctrl *mctrl;
	int ret = 0;
	u32 random;

	mctrl = interface->private_data;

	if (!test_bit(VIPX_ITF_STATE_START, &interface->state))
		goto exit;

	ret = emul_mbox_handler(mctrl);
	if (ret)
	{
		if (atomic_read(&checker) != 0) {
			vipx_info("nothing to handle, checker %d\n", atomic_read(&checker));
		}
		goto exit;
	}

	interface_isr(0, (void *)data);

exit:
	get_random_bytes(&random, sizeof(random));
	random = random % TURN_AROUND_TIME;
	mod_timer(&interface->timer, jiffies + random);
}
#endif

static void vipx_wq_func(struct work_struct *data)
{
	struct vipx_interface *interface;
	struct vipx_taskmgr *itaskmgr;
	struct vipx_task *itask;
	struct vipx_work_list *work_list;
	struct vipx_work *work;
	u32 graph_idx;
	ulong flags;

	interface = container_of(data, struct vipx_interface, work_queue);
	itaskmgr = &interface->taskmgr;
	work_list = &interface->work_list;

	WORK_FR_REPLY(work_list, &work);
	while (work) {
		if (work->id >= VIPX_MAX_TASK) {
			vipx_err("work id is invalid(%d)\n", work->id);
			break;
		}
		itask = &itaskmgr->task[work->id];

		if (itask->state != VIPX_TASK_STATE_PROCESS) {
			vipx_err("task(%d, %d, %ld) state is invalid(%d), work(%d, %d, %d)\n",
					itask->message, itask->index, itask->param1, itask->state,
					work->id, work->work_param0, work->work_param1);
			vipx_interface_print(interface);
			BUG();
		}

		switch (itask->message) {
		case VIPX_TASK_INIT:
		case VIPX_TASK_DEINIT:
		case VIPX_TASK_CREATE:
		case VIPX_TASK_DESTROY:
		case VIPX_TASK_ALLOCATE:
		case VIPX_TASK_REQUEST:
			graph_idx = itask->param1;
			interface->reply[graph_idx] = *work;
			DEBUG_ISR("[DEBUG_ISR] set reply graph idx(%d)\n", graph_idx);
			__set_reply(interface, graph_idx);
			break;
		case VIPX_TASK_PROCESS:
			DEBUG_ISR("[DEBUG_ISR] TASK PROCESS\n");

#ifdef NON_BLOCK_INVOKE
#else
			graph_idx = itask->param1;
			interface->reply[graph_idx] = *work;
			__set_reply(interface, graph_idx);
#endif

			taskmgr_e_barrier_irqs(itaskmgr, 0, flags);
			vipx_task_trans_pro_to_com(itaskmgr, itask);
			taskmgr_x_barrier_irqr(itaskmgr, 0, flags);

			itask->param2 = 0;
			itask->param3 = 0;
			vipx_graphmgr_queue(interface->cookie, itask);
			interface->done_cnt++;
			break;
		default:
			vipx_err("unresolved message(%d) have arrived\n", work->message);
			break;
		}

		WORK_TO_FREE(work_list, work);
		WORK_FR_REPLY(work_list, &work);
	}
}

int vipx_interface_probe(struct vipx_interface *interface,
	struct device *dev,
	void __iomem *regs,
	resource_size_t regs_size,
	u32 irq0, u32 irq1)
{
	int ret = 0;
	struct vipx_device *device;
	struct vipx_system *system;
	struct vipx_taskmgr *taskmgr;

	BUG_ON(!interface);
	BUG_ON(!dev);
	BUG_ON(!regs);

	system = container_of(interface, struct vipx_system, interface);
	device = container_of(system, struct vipx_device, system);

	init_process_barrier(interface);
	init_waitqueue_head(&interface->reply_queue);

	interface->regs = regs;
	interface->regs_size = regs_size;
	interface->cookie = (void *)&device->graphmgr;
	clear_bit(VIPX_ITF_STATE_OPEN, &interface->state);
	clear_bit(VIPX_ITF_STATE_BOOTUP, &interface->state);
	clear_bit(VIPX_ITF_STATE_ENUM, &interface->state);
	clear_bit(VIPX_ITF_STATE_START, &interface->state);
	interface->private_data = kmalloc(sizeof(struct vipx_mailbox_ctrl), GFP_KERNEL);
	if (!interface->private_data) {
		probe_err("kmalloc is fail\n");
		ret = -ENOMEM;
		goto p_err;
	}

	ret = devm_request_irq(dev, irq0, interface_isr0, 0, dev_name(dev), interface);
	if (ret) {
		probe_err("devm_request_irq(0) is fail(%d)\n", ret);
		goto p_err;
	}

	ret = devm_request_irq(dev, irq1, interface_isr1, 0, dev_name(dev), interface);
	if (ret) {
		probe_err("devm_request_irq(1) is fail(%d)\n", ret);
		goto p_err;
	}

	taskmgr = &interface->taskmgr;
	taskmgr->id = VIPX_MAX_GRAPH;
	taskmgr->sindex = 0;
	spin_lock_init(&taskmgr->slock);

	ret = vipx_task_init(taskmgr, interface);
	if (ret) {
		probe_err("vipx_task_init is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vipx_slab_init(&interface->slab);
	if (ret) {
		probe_err("vipx_slab_init is fail(%d)\n", ret);
		goto p_err;
	}

	INIT_WORK(&interface->work_queue, vipx_wq_func);
	INIT_WORK_LIST(&interface->work_list, VIPX_WORK_MAX_COUNT);

p_err:

	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

int vipx_interface_open(struct vipx_interface *interface,
	void *mbox, size_t mbox_size)
{
	struct vipx_mailbox_ctrl *mctrl;
	int ret = 0;
	u32 i;

	BUG_ON(!interface);
	BUG_ON(!mbox);

	interface->mbox = mbox;
	interface->mbox_size = mbox_size;
	memset(interface->mbox, 0x0, interface->mbox_size);

	mctrl = interface->private_data;
	mctrl->stack = vipx_mbox_g_stack(mbox, mbox_size);
	mctrl->urgent_stack = vipx_mbox_g_urgent_stack(mbox, mbox_size);

	ret = vipx_graphmgr_itf_register(interface->cookie, interface);
	if (ret) {
		vipx_err("vipx_graphmgr_itf_register is fail(%d)\n", ret);
		goto p_err;
	}

	atomic_set(&checker, 0);

	interface->process = NULL;
	for (i = 0; i < VIPX_MAX_GRAPH; ++i) {
		interface->request[i] = NULL;
		interface->reply[i].valid = 0;
	}

	interface->done_cnt = 0;
	set_bit(VIPX_ITF_STATE_OPEN, &interface->state);
	clear_bit(VIPX_ITF_STATE_BOOTUP, &interface->state);
	clear_bit(VIPX_ITF_STATE_ENUM, &interface->state);
	clear_bit(VIPX_ITF_STATE_START, &interface->state);

p_err:

	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

int vipx_interface_close(struct vipx_interface *interface)
{
	int ret = 0;

	BUG_ON(!interface);

	ret = __vipx_interface_cleanup(interface);
	if (ret)
		vipx_err("__vipx_interface_cleanup is fail(%d)\n", ret);

	ret = vipx_graphmgr_itf_unregister(interface->cookie, interface);
	if (ret)
		vipx_err("vipx_graphmgr_itf_unregister is fail(%d)\n", ret);

	interface->mbox = 0;
	interface->mbox_size = 0;

	clear_bit(VIPX_ITF_STATE_OPEN, &interface->state);
	clear_bit(VIPX_ITF_STATE_BOOTUP, &interface->state);

	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

int vipx_interface_start(struct vipx_interface *interface)
{
	int ret = 0;

	set_bit(VIPX_ITF_STATE_START, &interface->state);

#ifdef CONFIG_EXYNOS_EMUL_MBOX
	init_timer(&interface->timer);
	interface->timer.expires = jiffies + TURN_AROUND_TIME;
	interface->timer.data = (unsigned long)interface;
	interface->timer.function = interface_timer;
	add_timer(&interface->timer);
#endif

	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

int vipx_interface_stop(struct vipx_interface *interface)
{
	int errcnt = 0;
	struct vipx_taskmgr *itaskmgr;
	u32 retry;

	itaskmgr = &interface->taskmgr;

	retry = VIPX_STOP_WAIT_COUNT;
	while (--retry && itaskmgr->req_cnt) {
		vipx_warn("waiting %d request completion...(%d)\n", itaskmgr->req_cnt, retry);
		msleep(1);
	}

	if (!retry) {
		vipx_err("request completion is fail\n");
		vipx_interface_print(interface);
		errcnt++;
	}

	retry = VIPX_STOP_WAIT_COUNT;
	while (--retry && itaskmgr->pro_cnt) {
		vipx_warn("waiting %d process completion...(%d)\n", itaskmgr->pro_cnt, retry);
		msleep(1);
	}

	if (!retry) {
		vipx_err("process completion is fail\n");
		vipx_interface_print(interface);
		errcnt++;
	}

	retry = VIPX_STOP_WAIT_COUNT;
	while (--retry && itaskmgr->com_cnt) {
		vipx_warn("waiting %d complete completion...(%d)\n", itaskmgr->com_cnt, retry);
		msleep(1);
	}

	if (!retry) {
		vipx_err("complete completion is fail\n");
		vipx_interface_print(interface);
		errcnt++;
	}

#ifdef CONFIG_EXYNOS_EMUL_MBOX
	del_timer(&interface->timer);
#endif
	clear_bit(VIPX_ITF_STATE_START, &interface->state);

	vipx_info("%s:\n", __func__);
	return 0;
}

int vipx_hw_wait_bootup(struct vipx_interface *interface)
{
	int ret = 0;
	struct vipx_system *system = container_of(interface, struct vipx_system, interface);

	BUG_ON(!interface);

#ifdef DUMP_DEBUG_LOG_REGION
        int try_cnt = 10;
        struct vipx_binary *binary = &system->binary;
	while (try_cnt && !test_bit(VIPX_ITF_STATE_BOOTUP, &interface->state)) {
		msleep(1);
		try_cnt--;
		vipx_info("%s(): wait CM7 bootup\n",__func__);
	}
#endif

    vipx_info("debug kva(0x%p), dva(0x%x), size(%ld)\n", system->memory.info.kvaddr_debug, (u32)system->memory.info.dvaddr_debug, system->memory.info.pb_debug->size);

#ifdef DUMP_DEBUG_LOG_REGION
	if (try_cnt == 0)
	{
		if (!IS_ERR_OR_NULL(system->memory.info.kvaddr_debug)) {
			ret = vipx_binary_write(binary, VIPX_FW_PATH1, "vipx_log.bin",
				system->memory.info.kvaddr_debug, VIPX_DEBUG_SIZE);
			if (ret)
				vipx_err("vipx_binary_write is fail(%d)\n", ret);
		}
		ret = -EINVAL;
	}
#endif
	vipx_info("%s():%d\n", __func__, ret);
	return ret;
}

int vipx_hw_enum(struct vipx_interface *interface)
{
	int ret = 0;

	BUG_ON(!interface);

	set_bit(VIPX_ITF_STATE_ENUM, &interface->state);

	vipx_info("%s():%d\n", __func__, ret);
	return ret;
}

/* Init Req */
int vipx_hw_init(struct vipx_interface *interface, struct vipx_task *itask)
{
	int ret = 0;
	struct vipx_taskmgr *itaskmgr;
	ulong flag;
	vipx_msg_t payload;

	BUG_ON(!interface);
	BUG_ON(!itask);

	itaskmgr = &interface->taskmgr;

	payload.transId = itask->index;
	payload.type = INIT_REQ;

	/* TODO */
	/* fill init_req */
#if 0
	payload.msg_u.init_req.p_vip_core_bin = 0;
	payload.msg_u.init_req.sz_vip_core_bin = 0;
	payload.msg_u.init_req.p_cc_log = 0;
	payload.msg_u.init_req.sz_cc_log = 0;
#endif

	payload.msg_u.init_req.p_cc_heap = 0;
	payload.msg_u.init_req.sz_cc_heap = 0;

	itask->param0 = (ulong)&payload;
	/* Do not update param1. param1 is graph_idx */
	/* itask->param1 = 0; */
	itask->param2 = sizeof(vipx_msg_t);
	itask->param3 = VIPX_MTYPE_H2F_URGENT;

	ret = __vipx_set_cmd(interface, itask);
	if (ret) {
		vipx_err("__vipx_set_cmd is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	taskmgr_e_barrier_irqs(itaskmgr, 0, flag);
	vipx_task_trans_any_to_fre(itaskmgr, itask);
	taskmgr_x_barrier_irqr(itaskmgr, 0, flag);

	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

/* Power doen Req */
int vipx_hw_deinit(struct vipx_interface *interface, struct vipx_task *itask)
{
	int ret = 0;
	struct vipx_taskmgr *itaskmgr;
	ulong flag;
	vipx_msg_t payload;

	BUG_ON(!interface);
	BUG_ON(!itask);

	itaskmgr = &interface->taskmgr;

	/* TODO */
	payload.transId = itask->index;
	payload.type = POWER_DOWN_REQ;

	/* fill powerdown_graph_req */
	payload.msg_u.powerdown_req.valid = 1;

	itask->param0 = (ulong)&payload;
	/* Do not update param1. param1 is graph_idx */
	/* itask->param1 = 0; */
	itask->param2 = sizeof(vipx_msg_t);
	itask->param3 = VIPX_MTYPE_H2F_URGENT;

	ret = __vipx_set_cmd(interface, itask);
	if (ret) {
		vipx_err("__vipx_set_cmd is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	taskmgr_e_barrier_irqs(itaskmgr, 0, flag);
	vipx_task_trans_any_to_fre(itaskmgr, itask);
	taskmgr_x_barrier_irqr(itaskmgr, 0, flag);

	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

/* Create graph */
int vipx_hw_create(struct vipx_interface *interface, struct vipx_task *itask)
{
	int ret = 0;

	struct vipx_taskmgr *itaskmgr;
	ulong flag;
	vipx_msg_t payload;

	BUG_ON(!interface);
	BUG_ON(!itask);

	itaskmgr = &interface->taskmgr;

	/* TODO */
	payload.transId = itask->index;
	payload.type = CREATE_GRAPH_REQ;

	/* fill create_graph_req */
	payload.msg_u.create_graph_req.p_graph = 0;
	payload.msg_u.create_graph_req.sz_graph = 0;

	itask->param0 = (ulong)&payload;
	/* Do not update param1. param1 is graph_idx */
	/* itask->param1 = 0; */
	itask->param2 = sizeof(vipx_msg_t);
	itask->param3 = VIPX_MTYPE_H2F_NORMAL;

	ret = __vipx_set_cmd(interface, itask);
	if (ret) {
		vipx_err("__vipx_set_cmd is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	taskmgr_e_barrier_irqs(itaskmgr, 0, flag);
	vipx_task_trans_any_to_fre(itaskmgr, itask);
	taskmgr_x_barrier_irqr(itaskmgr, 0, flag);

	vipx_info("%s:%d\n", __func__, ret);

	return ret;
}

/* Destroy graph */
int vipx_hw_destroy(struct vipx_interface *interface, struct vipx_task *itask)
{
	int ret = 0;
	struct vipx_taskmgr *itaskmgr;
	struct vipx_system *system = container_of(interface, struct vipx_system, interface);
	ulong flag;
	vipx_msg_t payload;
	u32 heap_size;

	BUG_ON(!interface);
	BUG_ON(!itask);

	itaskmgr = &interface->taskmgr;

	/* TODO */
	payload.transId = itask->index;
	payload.type = DESTROY_GRAPH_REQ;

	/* fill destroy_graph_req */
	payload.msg_u.destroy_graph_req.graph_id = itask->param0;
	vipx_dbg("[%s] graph_id(%d)\n", __func__, payload.msg_u.destroy_graph_req.graph_id);

	itask->param0 = (ulong)&payload;
	/* Do not update param1. param1 is graph_idx */
	/* itask->param1 = 0; */
	itask->param2 = sizeof(vipx_msg_t);
	itask->param3 = VIPX_MTYPE_H2F_NORMAL;

	ret = __vipx_set_cmd(interface, itask);
	if (ret) {
		vipx_err("__vipx_set_cmd is fail(%d)\n", ret);
		goto p_err;
	}

	if (payload.msg_u.destroy_graph_req.graph_id < SCENARIO_DE_CAPTURE)
		heap_size = VIPX_CM7_HEAP_SIZE_PREVIEW;
	else if ((payload.msg_u.destroy_graph_req.graph_id == SCENARIO_ENF) ||
		 (payload.msg_u.destroy_graph_req.graph_id == SCENARIO_ENF_YUV))
		heap_size = VIPX_CM7_HEAP_SIZE_ENF;
	else
		heap_size = VIPX_CM7_HEAP_SIZE_CAPTURE;

	vipx_free_heap(&system->memory, &system->binary, heap_size);

	vipx_info("checker %d\n", atomic_read(&checker));

p_err:
	taskmgr_e_barrier_irqs(itaskmgr, 0, flag);
	vipx_task_trans_any_to_fre(itaskmgr, itask);
	taskmgr_x_barrier_irqr(itaskmgr, 0, flag);

	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

int vipx_hw_config(struct vipx_interface *interface, struct vipx_task *itask)
{
	int ret = 0;
	struct vipx_taskmgr *itaskmgr;
	struct vipx_system *system = container_of(interface, struct vipx_system, interface);
	ulong flag;
	vipx_msg_t payload;
	int i = 0, j = 0;
	int num_inputs = 0;
	int num_outputs = 0;
	int cur_width = 0;
	int cur_height = 0;
	dma_addr_t dvaddr_heap = 0;
	u32 heap_size;

	struct vipx_format_list *in_list;
	struct vipx_format_list *out_list;

	BUG_ON(!interface);
	BUG_ON(!itask);

	itaskmgr = &interface->taskmgr;
	in_list = (void *)itask->param2;
	out_list = (void *)itask->param3;

	payload.transId = itask->index;
	payload.type = SET_GRAPH_REQ;

	/* fill set_graph_req */
	payload.msg_u.set_graph_req.graph_id = itask->param0;
	vipx_dbg("[%s] graph_id(%d)\n", __func__, payload.msg_u.set_graph_req.graph_id);

	/* TODO : How to update depth field */
	for (i = 0; i < in_list->count; i++) {
		vipx_info("in-buf[%d], fmt %d, plane %d, size: %dx%d\n",
				i, in_list->formats[i].format,
				in_list->formats[i].plane,
				in_list->formats[i].width,
				in_list->formats[i].height);
		for (j = 0; j < in_list->formats[i].plane; j++) {
			ret = __get_size_from(&in_list->formats[i], j, &cur_width, &cur_height);
			if (ret) {
				vipx_err("__get_size_from fail (%d)\n", ret);
				goto p_err;
			}
			payload.msg_u.set_graph_req.input_width[num_inputs] = cur_width;
			payload.msg_u.set_graph_req.input_height[num_inputs] = cur_height;
			payload.msg_u.set_graph_req.input_depth[num_inputs] = 0;
			vipx_info("input[%d] WxH(%dx%d), depth(%d)\n", num_inputs,
					payload.msg_u.set_graph_req.input_width[num_inputs],
					payload.msg_u.set_graph_req.input_height[num_inputs],
					payload.msg_u.set_graph_req.input_depth[num_inputs]
				 );
			num_inputs++;
		}
	}

	for (i = 0; i < out_list->count; i++) {
		vipx_info("out-buf[%d], fmt %d, plane %d, size: %dx%d\n",
				i, out_list->formats[i].format,
				out_list->formats[i].plane,
				out_list->formats[i].width,
				out_list->formats[i].height);
		for (j = 0; j < out_list->formats[i].plane; j++) {
			ret = __get_size_from(&out_list->formats[i], j, &cur_width, &cur_height);
			if (ret) {
				vipx_err("__get_size_from fail (%d)\n", ret);
				goto p_err;
			}
			payload.msg_u.set_graph_req.output_width[num_outputs] = cur_width;
			payload.msg_u.set_graph_req.output_height[num_outputs] = cur_height;
			payload.msg_u.set_graph_req.output_depth[num_outputs] = 0;
			vipx_info("output[%d] WxH(%dx%d), depth(%d)\n", num_outputs,
					payload.msg_u.set_graph_req.output_width[num_outputs],
					payload.msg_u.set_graph_req.output_height[num_outputs],
					payload.msg_u.set_graph_req.output_depth[num_outputs]
				 );
			num_outputs++;
		}
	}
	// Allocate and assign heap
	if (payload.msg_u.set_graph_req.graph_id < SCENARIO_DE_CAPTURE)
		heap_size = VIPX_CM7_HEAP_SIZE_PREVIEW;
	else if ((payload.msg_u.destroy_graph_req.graph_id == SCENARIO_ENF_UV) || 
	    (payload.msg_u.destroy_graph_req.graph_id == SCENARIO_ENF) ||
	    (payload.msg_u.destroy_graph_req.graph_id == SCENARIO_ENF_YUV))
		heap_size = VIPX_CM7_HEAP_SIZE_ENF;
	else
		heap_size = VIPX_CM7_HEAP_SIZE_CAPTURE;
	dvaddr_heap = vipx_allocate_heap(&system->memory, heap_size);

	payload.msg_u.set_graph_req.p_temp = dvaddr_heap;
	payload.msg_u.set_graph_req.sz_temp = heap_size;
	vipx_info("CC heap dva(0x%x), size(%d)\n", (u32)dvaddr_heap, heap_size);

	payload.msg_u.set_graph_req.num_inputs = num_inputs;
	payload.msg_u.set_graph_req.num_outputs = num_outputs;
	vipx_info("input cnt(%d), output cnt(%d)\n", num_inputs, num_outputs);

	itask->param0 = (ulong)&payload;
	/* Do not update param1. param1 is graph_idx */
	/* itask->param1 = 0; */
	itask->param2 = sizeof(vipx_msg_t);
	itask->param3 = VIPX_MTYPE_H2F_NORMAL;

	ret = __vipx_set_cmd(interface, itask);
	if (ret) {
		vipx_err("__vipx_set_cmd is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	taskmgr_e_barrier_irqs(itaskmgr, 0, flag);
	vipx_task_trans_any_to_fre(itaskmgr, itask);
	taskmgr_x_barrier_irqr(itaskmgr, 0, flag);

	vipx_info("%s:%d\n", __func__, ret);
	return ret;
}

int vipx_hw_process(struct vipx_interface *interface, struct vipx_task *itask)
{
	int ret = 0;
	struct vipx_taskmgr *itaskmgr;
	ulong flag;
	vipx_msg_t payload;
	int i = 0, j = 0;
	int num_inputs = 0;
	int num_outputs = 0;
	int num_user_params = 0;

	struct vb_container_list *incl;
	struct vb_container_list *otcl;

	BUG_ON(!interface);
	BUG_ON(!itask);

	itaskmgr = &interface->taskmgr;
	incl = itask->incl;
	otcl = itask->otcl;

	payload.transId = itask->index;
	payload.type = INVOKE_GRAPH_REQ;

	/* use itask->incl and itask->otcl */
	payload.msg_u.invoke_graph_req.graph_id = itask->param0;
	vipx_dbg("[%s] graph_id(%d)\n", __func__, payload.msg_u.invoke_graph_req.graph_id);

	/* TODO : We need to consider to support multi plain buffer */
	for (i = 0; i < incl->count; i++) {
		for (j = 0; j < incl->containers[i].count; j++) {
			payload.msg_u.invoke_graph_req.p_input[num_inputs] = incl->containers[i].buffers[j].dvaddr;
			vipx_dbg("input[%d] buffer dvaddr(%x)\n", num_inputs,
					payload.msg_u.invoke_graph_req.p_input[num_inputs]);
			num_inputs++;
		}
	}
	for (i = 0; i < otcl->count; i++) {
		for (j = 0; j < otcl->containers[i].count; j++) {
			payload.msg_u.invoke_graph_req.p_output[num_outputs] = otcl->containers[i].buffers[j].dvaddr;
			vipx_dbg("output[%d] buffer dvaddr(%x)\n", num_outputs,
					payload.msg_u.invoke_graph_req.p_output[num_outputs]);
			num_outputs++;
		}
	}
	for (num_user_params = 0; num_user_params < MAX_NUM_OF_USER_PARAMS; num_user_params++) {
		payload.msg_u.invoke_graph_req.user_params[num_user_params] = incl->user_params[num_user_params];
		vipx_dbg("user_params[%d] = (%d)\n", num_user_params,
				payload.msg_u.invoke_graph_req.user_params[num_user_params]);
	}

	payload.msg_u.invoke_graph_req.num_inputs = num_inputs;
	payload.msg_u.invoke_graph_req.num_outputs = num_outputs;
	vipx_dbg("input cnt(%d), output cnt(%d)\n", num_inputs, num_outputs);

	itask->param0 = (ulong)&payload;
	/* Do not update param1. param1 is graph_idx */
	/* itask->param1 = 0; */
	itask->param2 = sizeof(vipx_msg_t);
	itask->param3 = VIPX_MTYPE_H2F_NORMAL;

#ifdef NON_BLOCK_INVOKE
	ret = __vipx_set_cmd_nblk(interface, itask);
	if (ret) {
		vipx_err("__vipx_set_cmd_nblk is fail(%d)\n", ret);
		goto p_err;
	}
#else
	ret = __vipx_set_cmd(interface, itask);
	if (ret) {
		vipx_err("__vipx_set_cmd is fail(%d)\n", ret);
		goto p_err;
	}
#endif

	vipx_dbg("%s:%d\n", __func__, ret);
	return 0;

p_err:
	taskmgr_e_barrier_irqs(itaskmgr, 0, flag);
	vipx_task_trans_any_to_fre(itaskmgr, itask);
	taskmgr_x_barrier_irqr(itaskmgr, 0, flag);

	return ret;
}
