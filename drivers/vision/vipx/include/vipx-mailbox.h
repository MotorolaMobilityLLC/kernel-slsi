/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef VIPX_MAILBOX_H_
#define VIPX_MAILBOX_H_

#include <interface/ap_vip_if.h>

#define VIPX_MAILBOX_SIGNATURE1		0xCAFE
#define VIPX_MAILBOX_SIGNATURE2		0xEFAC
#define VIPX_MAILBOX_BASEOFFSET		16

#define VIPX_CMD_SIGNATURE		0x1234
#define VIPX_DUM_SIGNATURE		0x5678

#define MAX_MESSAGE_CNT 8

#define DMESG() printk("[%s:%d]\n", __func__, __LINE__)

enum {
	VIPX_MTYPE_H2F_NORMAL = 0,
	VIPX_MTYPE_H2F_URGENT = 1,
	VIPX_MTYPE_F2H_NORMAL = 0,
	VIPX_MTYPE_F2H_URGENT = 1,
	VIPX_MTYPE_MAX,
};

struct vipx_mailbox_h2f {
	volatile u32			wmsg_idx; /* Host permission is RW */
	volatile u32			rmsg_idx; /* Host permission is R_ONLY */
	vipx_msg_t		msg[MAX_MESSAGE_CNT];
};

struct vipx_mailbox_f2h {
	volatile u32			wmsg_idx; /* Host permission is R_ONLY */
	volatile u32			rmsg_idx; /* Host permission is RW */
	vipx_msg_t		msg[MAX_MESSAGE_CNT];
};

struct vipx_mailbox_stack {
	struct vipx_mailbox_h2f h2f;
	struct vipx_mailbox_f2h f2h;
};

struct vipx_mailbox_ctrl {
	struct vipx_mailbox_stack	*stack;
	struct vipx_mailbox_stack	*urgent_stack;
};

struct vipx_mailbox_stack * vipx_mbox_g_stack(void *mbox, u32 mbox_size);
struct vipx_mailbox_stack * vipx_mbox_g_urgent_stack(void *mbox, u32 mbox_size);
int vipx_mbox_ready(struct vipx_mailbox_ctrl *mctrl, size_t size, u32 type);
int vipx_mbox_wait_reply(struct vipx_mailbox_ctrl *mctrl, u32 cmd, u32 type);
int vipx_mbox_write(struct vipx_mailbox_ctrl *mctrl,
	void *payload, size_t size, u32 type, u32 gid, u32 cmd, u32 cid);
int vipx_mbox_read(struct vipx_mailbox_ctrl *mctrl,
	void *payload, u32 type, void *debug_data);

int emul_mbox_handler(struct vipx_mailbox_ctrl *mctrl);

#endif
