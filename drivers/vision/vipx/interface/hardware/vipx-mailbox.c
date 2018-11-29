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
#include <linux/kernel.h>

#include "vipx-config.h"
#include "vipx-io.h"
#include "vipx-mailbox.h"
#include "vipx-interface.h"

#include <interface/ap_vip_if.h>

#define INDEX(x) (x % MAX_MESSAGE_CNT)

struct vipx_mailbox_stack * vipx_mbox_g_stack(void *mbox, u32 mbox_size)
{
	vipx_info("sizeof mbox stack (%lx) address of normal mbox (%p)\n",
			sizeof(struct vipx_mailbox_stack), mbox);
	return (struct vipx_mailbox_stack *)mbox;
}

struct vipx_mailbox_stack * vipx_mbox_g_urgent_stack(void *mbox, u32 mbox_size)
{
	vipx_info("address of urgent mbox (%p)\n", ((char *)mbox + sizeof(struct vipx_mailbox_stack)));
	return (struct vipx_mailbox_stack *)((char *)mbox + sizeof(struct vipx_mailbox_stack));
}

static u32 __vipx_mbox_g_freesize(struct vipx_mailbox_h2f *mbox)
{
	u32 wmsg_idx = 0;
	u32 rmsg_idx = 0;
	u32 free_size = 0;

	BUG_ON(!mbox);

	wmsg_idx = mbox->wmsg_idx;
	rmsg_idx = mbox->rmsg_idx;

	free_size = MAX_MESSAGE_CNT - (wmsg_idx - rmsg_idx);

	BUG_ON(free_size < 0);

	return free_size;
}

int vipx_mbox_ready(struct vipx_mailbox_ctrl *mctrl,
	size_t size, u32 type)
{
	u32 try_count;
	struct vipx_mailbox_h2f *mbox;
	u16 free_size16;

	BUG_ON(!mctrl);
	BUG_ON(!IS_ALIGNED(size, 2));

	if (type == VIPX_MTYPE_H2F_NORMAL) {
		mbox = &mctrl->stack->h2f;
	} else if (type == VIPX_MTYPE_H2F_URGENT) {
		mbox = &mctrl->urgent_stack->h2f;
	} else {
		vipx_err("invalid type(%d)\n", type);
		return -EINVAL;
	}
	try_count = 1000;

	free_size16 = __vipx_mbox_g_freesize(mbox);
	while (--try_count && (free_size16 == 0)) {
		vipx_warn("mbox is not ready(freesize %d)...(%d)\n",
			free_size16, try_count);
		udelay(10);
		free_size16 = __vipx_mbox_g_freesize(mbox);
	}

	if (try_count)
		return 0;
	else
		return -EBUSY;
}

int vipx_mbox_wait_reply(struct vipx_mailbox_ctrl *mctrl,
	u32 cmd, u32 type)
{
	int ret = 0;
	struct vipx_mailbox_f2h *mbox;
	int try_count;

	if (type == VIPX_MTYPE_F2H_NORMAL) {
		mbox = &mctrl->stack->f2h;
	} else if (type == VIPX_MTYPE_F2H_URGENT) {
		mbox = &mctrl->urgent_stack->f2h;
	} else {
		vipx_err("cmd(%d) has invalid type(%d)\n", cmd, type);
		ret = -EINVAL;
		goto p_err;
	}

	try_count = 1000;
	while (--try_count && (mbox->wmsg_idx == mbox->rmsg_idx)) {
		vipx_warn("waiting vipx reply(%d, %d)...(%d)\n", mbox->wmsg_idx, mbox->rmsg_idx, try_count);
		msleep(1);
	}

	if (try_count <= 0) {
		vipx_err("waiting vipx reply is timeout\n");
		ret = -EINVAL;
		goto p_err;
	}

p_err:
	return ret;
}

int vipx_mbox_write(struct vipx_mailbox_ctrl *mctrl,
	void *payload, size_t size, u32 type, u32 gid, u32 cmd, u32 cid)
{
	int ret = 0;
	struct vipx_mailbox_h2f *mbox;
	u32 *wptr;

	BUG_ON(!mctrl);
	BUG_ON(!payload);
	BUG_ON(!IS_ALIGNED(size, 2));

	if (type == VIPX_MTYPE_H2F_NORMAL) {
		mbox = &mctrl->stack->h2f;
	} else if (type == VIPX_MTYPE_H2F_URGENT) {
		mbox = &mctrl->urgent_stack->h2f;
	} else {
		vipx_err("invalid type(%d)\n", type);
		ret = -EINVAL;
		goto p_err;
	}
	wptr = (void *)&mbox->msg[mbox->wmsg_idx % MAX_MESSAGE_CNT];

	mem2iocpy(wptr, payload, size);

	/* increment actual write pointer */
	mbox->wmsg_idx++;

#ifdef DBG_INTERFACE_ISR
	vipx_info("[I%d][MBOX][H2F] %d command(type: %d, cid: %d, wmsg_idx: %d, rmsg_idx: %d)\n", gid,
		cmd, type, cid, mbox->wmsg_idx, mbox->rmsg_idx);
#endif

p_err:
	return ret;
}

int vipx_mbox_read(struct vipx_mailbox_ctrl *mctrl,
	void *payload, u32 type, void *debug_data)
{
	int ret = 0;
	struct vipx_mailbox_f2h *mbox;
	u32 *rptr;

	BUG_ON(!mctrl);
	BUG_ON(!payload);

	if (type == VIPX_MTYPE_H2F_NORMAL) {
		mbox = &mctrl->stack->f2h;
	} else if (type == VIPX_MTYPE_H2F_URGENT) {
		mbox = &mctrl->urgent_stack->f2h;
	} else {
		vipx_err("invalid type(%d)\n", type);
		ret = -EINVAL;
		goto p_err;
	}
	rptr = (void *)&mbox->msg[mbox->rmsg_idx % MAX_MESSAGE_CNT];

	io2memcpy(payload, rptr, sizeof(vipx_msg_t));

	/* increment actual read pointer */
	mbox->rmsg_idx++;

p_err:
	return ret;
}
