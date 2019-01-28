/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __VIPX_TASKMGR_H__
#define __VIPX_TASKMGR_H__

#include <linux/kthread.h>

#include "vipx-config.h"
#include "vipx-time.h"

#define TASKMGR_IDX_0		(1 << 0) /* graphmgr thread */
#define TASKMGR_IDX_1		(1 << 1)
#define TASKMGR_IDX_2		(1 << 2)
#define TASKMGR_IDX_3		(1 << 3)
#define TASKMGR_IDX_4		(1 << 4)
#define TASKMGR_IDX_5		(1 << 5)
#define TASKMGR_IDX_6		(1 << 6)
#define TASKMGR_IDX_7		(1 << 7)
#define TASKMGR_IDX_8		(1 << 8)
#define TASKMGR_IDX_9		(1 << 9)
#define TASKMGR_IDX_10		(1 << 10)
#define TASKMGR_IDX_11		(1 << 11)
#define TASKMGR_IDX_12		(1 << 12)
#define TASKMGR_IDX_13		(1 << 13)
#define TASKMGR_IDX_14		(1 << 14)
#define TASKMGR_IDX_15		(1 << 15)
#define TASKMGR_IDX_16		(1 << 16)
#define TASKMGR_IDX_17		(1 << 17)
#define TASKMGR_IDX_18		(1 << 18)
#define TASKMGR_IDX_19		(1 << 19)
#define TASKMGR_IDX_20		(1 << 20)
#define TASKMGR_IDX_21		(1 << 21)
#define TASKMGR_IDX_22		(1 << 22)
#define TASKMGR_IDX_23		(1 << 23)
#define TASKMGR_IDX_24		(1 << 24)
#define TASKMGR_IDX_25		(1 << 25)
#define TASKMGR_IDX_26		(1 << 26)
#define TASKMGR_IDX_27		(1 << 27)
#define TASKMGR_IDX_28		(1 << 28)
#define TASKMGR_IDX_29		(1 << 29)
#define TASKMGR_IDX_30		(1 << 30)
#define TASKMGR_IDX_31		(1 << 31)

#define taskmgr_e_barrier_irqs(taskmgr, index, flag)		\
	do {							\
		taskmgr->sindex |= index;			\
		spin_lock_irqsave(&taskmgr->slock, flag);	\
	} while (0)

#define taskmgr_x_barrier_irqr(taskmgr, index, flag)		\
	do {							\
		spin_unlock_irqrestore(&taskmgr->slock, flag);	\
		taskmgr->sindex &= ~index;			\
	} while (0)

#define taskmgr_e_barrier_irq(taskmgr, index)			\
	do {							\
		taskmgr->sindex |= index;			\
		spin_lock_irq(&taskmgr->slock);			\
	} while (0)

#define taskmgr_x_barrier_irq(taskmgr, index)			\
	do {							\
		spin_unlock_irq(&taskmgr->slock);		\
		taskmgr->sindex &= ~index;			\
	} while (0)

#define taskmgr_e_barrier(taskmgr, index)			\
	do {							\
		taskmgr->sindex |= index;			\
		spin_lock(&taskmgr->slock);			\
	} while (0)

#define taskmgr_x_barrier(taskmgr, index)			\
	do {							\
		spin_unlock(&taskmgr->slock);			\
		taskmgr->sindex &= ~index;			\
	} while (0)

enum vipx_task_state {
	VIPX_TASK_STATE_FREE = 1,
	VIPX_TASK_STATE_REQUEST,
	VIPX_TASK_STATE_PREPARE,
	VIPX_TASK_STATE_PROCESS,
	VIPX_TASK_STATE_COMPLETE,
	VIPX_TASK_STATE_INVALID
};

enum vipx_task_flag {
	VIPX_TASK_FLAG_IOCPY = 16
};

enum vipx_task_message {
	VIPX_TASK_INIT = 1,
	VIPX_TASK_DEINIT,
	VIPX_TASK_CREATE,
	VIPX_TASK_DESTROY,
	VIPX_TASK_ALLOCATE,
	VIPX_TASK_PROCESS,
	VIPX_TASK_REQUEST = 10,
	VIPX_TASK_DONE,
	VIPX_TASK_NDONE
};

enum vipx_control_message {
	VIPX_CTRL_NONE = 100,
	VIPX_CTRL_STOP,
	VIPX_CTRL_STOP_DONE
};

struct vipx_task {
	struct list_head		list;
	struct kthread_work		work;
	unsigned int			state;

	unsigned int			message;
	unsigned long			param0;
	unsigned long			param1;
	unsigned long			param2;
	unsigned long			param3;

	struct vipx_container_list	*incl;
	struct vipx_container_list	*otcl;
	unsigned long			flags;

	unsigned int			id;
	struct mutex			*lock;
	unsigned int			index;
	unsigned int			findex;
	unsigned int			tdindex;
	void				*owner;

	struct vipx_time		time;
};

struct vipx_taskmgr {
	unsigned int			id;
	unsigned int			sindex;
	spinlock_t			slock;
	struct vipx_task		task[VIPX_MAX_TASK];

	struct list_head		fre_list;
	struct list_head		req_list;
	struct list_head		pre_list;
	struct list_head		pro_list;
	struct list_head		com_list;

	unsigned int			tot_cnt;
	unsigned int			fre_cnt;
	unsigned int			req_cnt;
	unsigned int			pre_cnt;
	unsigned int			pro_cnt;
	unsigned int			com_cnt;
};

void vipx_task_print_free_list(struct vipx_taskmgr *tmgr);
void vipx_task_print_request_list(struct vipx_taskmgr *tmgr);
void vipx_task_print_prepare_list(struct vipx_taskmgr *tmgr);
void vipx_task_print_process_list(struct vipx_taskmgr *tmgr);
void vipx_task_print_complete_list(struct vipx_taskmgr *tmgr);
void vipx_task_print_all(struct vipx_taskmgr *tmgr);

void vipx_task_trans_fre_to_req(struct vipx_taskmgr *tmgr,
		struct vipx_task *task);
void vipx_task_trans_req_to_pre(struct vipx_taskmgr *tmgr,
		struct vipx_task *task);
void vipx_task_trans_req_to_pro(struct vipx_taskmgr *tmgr,
		struct vipx_task *task);
void vipx_task_trans_req_to_com(struct vipx_taskmgr *tmgr,
		struct vipx_task *task);
void vipx_task_trans_req_to_fre(struct vipx_taskmgr *tmgr,
		struct vipx_task *task);
void vipx_task_trans_pre_to_pro(struct vipx_taskmgr *tmgr,
		struct vipx_task *task);
void vipx_task_trans_pre_to_com(struct vipx_taskmgr *tmgr,
		struct vipx_task *task);
void vipx_task_trans_pre_to_fre(struct vipx_taskmgr *tmgr,
		struct vipx_task *task);
void vipx_task_trans_pro_to_com(struct vipx_taskmgr *tmgr,
		struct vipx_task *task);
void vipx_task_trans_pro_to_fre(struct vipx_taskmgr *tmgr,
		struct vipx_task *task);
void vipx_task_trans_com_to_fre(struct vipx_taskmgr *tmgr,
		struct vipx_task *task);
void vipx_task_trans_any_to_fre(struct vipx_taskmgr *tmgr,
		struct vipx_task *task);

struct vipx_task *vipx_task_pick_fre_to_req(struct vipx_taskmgr *tmgr);

int vipx_task_init(struct vipx_taskmgr *tmgr, unsigned int id, void *owner);
int vipx_task_deinit(struct vipx_taskmgr *tmgr);
void vipx_task_flush(struct vipx_taskmgr *tmgr);

#endif
