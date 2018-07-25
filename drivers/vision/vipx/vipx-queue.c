/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "vipx-queue.h"
#include "vipx-config.h"
#include "vipx-taskmgr.h"
#include "vipx-graph.h"

extern const struct vb_ops vb_ops;
extern const struct vipx_queue_ops vipx_queue_ops;

int vipx_queue_init(struct vipx_queue *queue,
	struct vipx_memory *memory,
	struct mutex *lock)
{
	int ret = 0;
	struct vb_queue *inq, *otq;

	queue->qops = &vipx_queue_ops;
	inq = &queue->inqueue;
	otq = &queue->otqueue;
	inq->private_data = queue;
	otq->private_data = queue;

	ret = vb_queue_init(inq, memory->dev, memory->vb2_mem_ops, &vb_ops, lock, VS4L_DIRECTION_IN);
	if (ret) {
		vipx_err("vb_queue_init is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vb_queue_init(otq, memory->dev, memory->vb2_mem_ops, &vb_ops, lock, VS4L_DIRECTION_OT);
	if (ret) {
		vipx_err("vb_queue_init is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

int vipx_queue_s_format(struct vipx_queue *queue, struct vs4l_format_list *f)
{
	int ret = 0;
	struct vb_queue *q, *inq, *otq;

	BUG_ON(!queue);
	BUG_ON(!f);

	inq = &queue->inqueue;
	otq = &queue->otqueue;

	if (f->direction == VS4L_DIRECTION_IN)
		q = inq;
	else
		q = otq;

	ret = CALL_QOPS(queue, format, f);
	if (ret) {
		vipx_err("CALL_QOPS(format) is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vb_queue_s_format(q, f);
	if (ret) {
		vipx_err("vb_queue_s_format is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

int vipx_queue_start(struct vipx_queue *queue)
{
	int ret = 0;
	struct vb_queue *inq, *otq;

	inq = &queue->inqueue;
	otq = &queue->otqueue;

	ret = vb_queue_start(inq);
	if (ret) {
		vipx_err("vb_queue_init is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vb_queue_start(otq);
	if (ret) {
		vipx_err("vb_queue_init is fail(%d)\n", ret);
		goto p_err;
	}

	ret = CALL_QOPS(queue, start);
	if (ret) {
		vipx_err("CALL_QOPS(start) is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

int vipx_queue_stop(struct vipx_queue *queue)
{
	int ret = 0;
	struct vb_queue *inq, *otq;

	inq = &queue->inqueue;
	otq = &queue->otqueue;

	ret = CALL_QOPS(queue, stop);
	if (ret) {
		vipx_err("CALL_QOPS(stop) is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vb_queue_stop(inq);
	if (ret) {
		vipx_err("vb_queue_init is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vb_queue_stop(otq);
	if (ret) {
		vipx_err("vb_queue_init is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

int vipx_queue_poll(struct vipx_queue *queue, struct file *file, poll_table *poll)
{
	int ret = 0;
	struct vb_queue *inq, *otq;
	unsigned long events;

	BUG_ON(!queue);
	BUG_ON(!file);
	BUG_ON(!poll);

	events = poll_requested_events(poll);

	inq = &queue->inqueue;
	otq = &queue->otqueue;

	if (events & POLLIN) {
		if (list_empty(&inq->done_list))
			poll_wait(file, &inq->done_wq, poll);

		if (list_empty(&inq->done_list))
			ret |= POLLIN | POLLWRNORM;
	}

	if (events & POLLOUT) {
		if (list_empty(&otq->done_list))
			poll_wait(file, &otq->done_wq, poll);

		if (list_empty(&otq->done_list))
			ret |= POLLOUT | POLLWRNORM;
	}

	return ret;
}

int vipx_queue_qbuf(struct vipx_queue *queue, struct vs4l_container_list *c)
{
	int ret = 0;
	struct vb_queue *q, *inq, *otq;
	struct vb_bundle *invb, *otvb;

	inq = &queue->inqueue;
	otq = &queue->otqueue;

	if (c->direction == VS4L_DIRECTION_IN)
		q = inq;
	else
		q = otq;

	ret = vb_queue_qbuf(q, c);
	if (ret) {
		vipx_err("vb_queue_qbuf is fail(%d)\n", ret);
		goto p_err;
	}

	if (list_empty(&inq->queued_list))
		goto p_err;

	if (list_empty(&otq->queued_list))
		goto p_err;

	invb = list_first_entry(&inq->queued_list, struct vb_bundle, queued_entry);
	otvb = list_first_entry(&otq->queued_list, struct vb_bundle, queued_entry);

	vb_queue_process(inq, invb);
	vb_queue_process(otq, otvb);

	ret = CALL_QOPS(queue, queue, &invb->clist, &otvb->clist);
	if (ret) {
		vipx_err("CALL_QOPS(queue) is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

int vipx_queue_dqbuf(struct vipx_queue *queue, struct vs4l_container_list *c, bool nonblocking)
{
	int ret = 0;
	struct vb_queue *q;
	struct vb_bundle *bundle;

	BUG_ON(!queue);

	if (c->direction == VS4L_DIRECTION_IN)
		q = &queue->inqueue;
	else
		q = &queue->otqueue;

	ret = vb_queue_dqbuf(q, c, nonblocking);
	if (ret) {
		vipx_err("vb_queue_dqbuf is fail(%d)\n", ret);
		goto p_err;
	}

	if (c->index >= VIPX_MAX_BUFFER) {
		vipx_err("container index(%d) is invalid\n", c->index);
		ret = -EINVAL;
		goto p_err;
	}

	bundle = q->bufs[c->index];
	if (!bundle) {
		vipx_err("bundle(%d) is NULL\n", c->index);
		ret = -EINVAL;
		goto p_err;
	}

	if (bundle->clist.index != c->index) {
		vipx_err("index is NOT matched(%d != %d)\n", bundle->clist.index, c->index);
		ret = -EINVAL;
		goto p_err;
	}

	ret = CALL_QOPS(queue, deque, &bundle->clist);
	if (ret) {
		vipx_err("CALL_QOPS(deque) is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

void vipx_queue_done(struct vipx_queue *queue,
	struct vb_container_list *incl,
	struct vb_container_list *otcl,
	unsigned long flags)
{
	struct vb_queue *inq, *otq;
	struct vb_bundle *invb, *otvb;

	BUG_ON(!queue);
	BUG_ON(!incl);
	BUG_ON(!otcl);

	inq = &queue->inqueue;
	otq = &queue->otqueue;

	if (list_empty(&inq->process_list)) {
		vipx_err("inqueue is empty\n");
		BUG();
	}

	if (list_empty(&otq->process_list)) {
		vipx_err("otqueue is empty\n");
		BUG();
	}

	invb = container_of(incl, struct vb_bundle, clist);
	otvb = container_of(otcl, struct vb_bundle, clist);

	if (invb->state != VB_BUF_STATE_PROCESS) {
		vipx_err("invb state(%d) is invalid\n", invb->state);
		BUG();
	}

	if (otvb->state != VB_BUF_STATE_PROCESS) {
		vipx_err("otvb state(%d) is invalid\n", otvb->state);
		BUG();
	}

	otvb->flags |= flags;
	vb_queue_done(otq, otvb);

	invb->flags |= flags;
	vb_queue_done(inq, invb);
}
