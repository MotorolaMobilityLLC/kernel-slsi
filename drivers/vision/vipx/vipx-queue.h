/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef VIPX_QUEUE_H_
#define VIPX_QUEUE_H_

#include <linux/types.h>
#include <linux/mutex.h>

#include "vision-buffer.h"
#include "vipx-taskmgr.h"
#include "vipx-memory.h"

struct vipx_queue;

struct vipx_queue_ops {
	int (*start)(struct vipx_queue *queue);
	int (*stop)(struct vipx_queue *queue);
	int (*format)(struct vipx_queue *queue, struct vs4l_format_list *f);
	int (*queue)(struct vipx_queue *queue, struct vb_container_list *incl, struct vb_container_list *otcl);
	int (*deque)(struct vipx_queue *queue, struct vb_container_list *clist);
};

struct vipx_queue {
	struct vb_queue			inqueue;
	struct vb_queue			otqueue;
	const struct vipx_queue_ops	*qops;
};

int vipx_queue_init(struct vipx_queue *queue, struct vipx_memory *memory, struct mutex *lock);
int vipx_queue_s_format(struct vipx_queue *queue, struct vs4l_format_list *f);
int vipx_queue_start(struct vipx_queue *queue);
int vipx_queue_stop(struct vipx_queue *queue);
int vipx_queue_poll(struct vipx_queue *queue, struct file *file, poll_table *poll);
int vipx_queue_qbuf(struct vipx_queue *queue, struct vs4l_container_list *c);
int vipx_queue_dqbuf(struct vipx_queue *queue, struct vs4l_container_list *c, bool nonblocking);
void vipx_queue_done(struct vipx_queue *queue, struct vb_container_list *incl, struct vb_container_list *otcl, unsigned long flags);

#define CALL_QOPS(q, op, ...)	(((q)->qops->op) ? ((q)->qops->op(q, ##__VA_ARGS__)) : 0)

#endif
