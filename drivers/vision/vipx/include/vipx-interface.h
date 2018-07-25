/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef VIPX_INTERFACE_H_
#define VIPX_INTERFACE_H_

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/timer.h>

#include "vipx-taskmgr.h"
#include "vipx-slab.h"

#define VIPX_WORK_MAX_COUNT		20
#define VIPX_WORK_MAX_DATA		24
#define VIPX_COMMAND_TIMEOUT		(3 * HZ)

typedef enum {
	VIPX_WORK_MTYPE_BLK,
	VIPX_WORK_MTYPE_NBLK,
} vipx_work_msg_t;

struct vipx_work {
	struct list_head		list;
	u32				valid:1;
	u32				id;
	vipx_work_msg_t			message;
	u32				work_param0;
	u32				work_param1;
	u32				work_param2;
	u32				work_param3;
	u8				data[VIPX_WORK_MAX_DATA];
};

struct vipx_work_list {
	struct vipx_work			work[VIPX_WORK_MAX_COUNT];
	spinlock_t			slock;
	struct list_head		free_head;
	u32				free_cnt;
	struct list_head		reply_head;
	u32				reply_cnt;
	wait_queue_head_t		wait_queue;
};

enum vipx_interface_state {
	VIPX_ITF_STATE_OPEN,
	VIPX_ITF_STATE_BOOTUP,
	VIPX_ITF_STATE_ENUM,
	VIPX_ITF_STATE_START
};

struct vipx_interface {
	void				*mbox;
	size_t				mbox_size;
	void __iomem			*regs;
	resource_size_t			regs_size;
	unsigned long			state;

	struct vipx_slab_allocator	slab;
	struct vipx_taskmgr		taskmgr;
	void				*cookie;
	u32				done_cnt;

	struct vipx_task		*request[VIPX_MAX_GRAPH];
	struct mutex			request_barrier;
	struct vipx_work		reply[VIPX_MAX_GRAPH];
	wait_queue_head_t		reply_queue;
	struct vipx_task		*process;
	spinlock_t			process_barrier;

	struct vipx_work_list		work_list;
	struct work_struct		work_queue;

#if defined(CONFIG_EXYNOS_EMUL_MBOX)
	struct timer_list		timer;
#endif
	void				*private_data;
};

int vipx_interface_probe(struct vipx_interface *interface,
	struct device *dev,
	void __iomem *regs,
	resource_size_t regs_size,
	u32 irq0, u32 irq1);
int vipx_interface_open(struct vipx_interface *interface,
	void *mbox, size_t mbox_size);
int vipx_interface_close(struct vipx_interface *interface);
int vipx_interface_start(struct vipx_interface *interface);
int vipx_interface_stop(struct vipx_interface *interface);
void vipx_interface_print(struct vipx_interface *interface);

int vipx_hw_wait_bootup(struct vipx_interface *interface);

int vipx_hw_enum(struct vipx_interface *interface);
int vipx_hw_init(struct vipx_interface *interface, struct vipx_task *itask);
int vipx_hw_deinit(struct vipx_interface *interface, struct vipx_task *itask);
int vipx_hw_create(struct vipx_interface *interface, struct vipx_task *itask);
int vipx_hw_destroy(struct vipx_interface *interface, struct vipx_task *itask);
int vipx_hw_config(struct vipx_interface *interface, struct vipx_task *itask);
int vipx_hw_process(struct vipx_interface *interface, struct vipx_task *itask);

#endif
