/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef VIPX_DEBUG_H_
#define VIPX_DEBUG_H_

#include <linux/types.h>
#include <linux/timer.h>

#include "vipx-config.h"

#define DEBUG_SENTENCE_MAX		300
#define DEBUG_MONITORING_PERIOD 	(HZ * 5)

struct vipx_debug_imgdump {
	struct dentry			*file;
	u32				target_graph;
	u32				target_chain;
	u32				target_pu;
	u32				target_index;
	void				*kvaddr;
	void				*cookie;
	size_t				length;
	size_t				offset;
};

struct vipx_debug_monitor {
	struct timer_list		timer;
	u32				time_cnt;
	u32				tick_cnt;
	u32				sched_cnt;
	u32				done_cnt;
};

enum vipx_debug_state {
	VIPX_DEBUG_STATE_START
};

struct vipx_debug {
	unsigned long			state;

	struct dentry			*root;
	struct dentry			*logfile;
	struct dentry			*grpfile;
	struct dentry			*buffile;

	/* graph */
	void				*graphmgr_data;
	void				*system_data;

	struct vipx_debug_imgdump	imgdump;
	struct vipx_debug_monitor	monitor;
};

struct vipx_debug_log {
	size_t			dsentence_pos;
	char			dsentence[DEBUG_SENTENCE_MAX];
};

s32 atoi(const char *psz_buf);
int bitmap_scnprintf(char *buf, unsigned int buflen,
        const unsigned long *maskp, int nmaskbits);

int vipx_debug_probe(struct vipx_debug *debug, void *graphmgr_data, void *interface_data);
int vipx_debug_open(struct vipx_debug *debug);
int vipx_debug_close(struct vipx_debug *debug);
int vipx_debug_start(struct vipx_debug *debug);
int vipx_debug_stop(struct vipx_debug *debug);

void vipx_dmsg_concate(struct vipx_debug_log *log, const char *fmt, ...);
char * vipx_dmsg_print(struct vipx_debug_log *log);
int vipx_debug_memdump8(u8 *start, u8 *end);
int vipx_debug_memdump16(u16 *start, u16 *end);
int vipx_debug_memdump32(u32 *start, u32 *end);

#ifdef DBG_HISTORY
#define DLOG_INIT()		struct vipx_debug_log vipx_debug_log = { .dsentence_pos = 0 }
#define DLOG(fmt, ...)		vipx_dmsg_concate(&vipx_debug_log, fmt, ##__VA_ARGS__)
#define DLOG_OUT()		vipx_dmsg_print(&vipx_debug_log)
#else
#define DLOG_INIT()
#define DLOG(fmt, ...)
#define DLOG_OUT()		"FORBIDDEN HISTORY"
#endif

#endif
