/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __VIPX_DEBUG_H__
#define __VIPX_DEBUG_H__

#include <linux/timer.h>

#include "vipx-config.h"
#include "vipx-system.h"

#ifdef DEBUG_LOG_CONCATE_ENABLE
#define DLOG_INIT()		struct vipx_debug_log dlog = \
					{ .dsentence_pos = 0 }
#define DLOG(fmt, ...)		\
	vipx_debug_log_concate(&dlog, fmt, ##__VA_ARGS__)
#define DLOG_OUT()		vipx_debug_log_print(&dlog)
#else
#define DLOG_INIT()
#define DLOG(fmt, ...)
#define DLOG_OUT()		"FORBIDDEN HISTORY"
#endif
#define DEBUG_SENTENCE_MAX		(300)

struct vipx_device;

struct vipx_debug_imgdump {
	struct dentry			*file;
	unsigned int			target_graph;
	unsigned int			target_chain;
	unsigned int			target_pu;
	unsigned int			target_index;
	void				*kvaddr;
	void				*cookie;
	size_t				length;
	size_t				offset;
};

struct vipx_debug_monitor {
	unsigned int			time_cnt;
	unsigned int			tick_cnt;
	unsigned int			sched_cnt;
	unsigned int			done_cnt;
	struct timer_list		timer;
};

enum vipx_debug_state {
	VIPX_DEBUG_STATE_START,
};

struct vipx_debug {
	unsigned long			state;
	struct vipx_system		*system;

	struct dentry			*root;
	struct dentry			*logfile;
	struct dentry			*grpfile;
	struct dentry			*buffile;

	struct vipx_debug_imgdump	imgdump;
	struct vipx_debug_monitor	monitor;
};

struct vipx_debug_log {
	size_t				dsentence_pos;
	char				dsentence[DEBUG_SENTENCE_MAX];
};

int vipx_debug_write_log_binary(void);
int vipx_debug_dump_debug_regs(void);

int vipx_debug_atoi(const char *psz_buf);
int vipx_debug_bitmap_scnprintf(char *buf, unsigned int buflen,
		const unsigned long *maskp, int nmaskbits);

void vipx_debug_log_concate(struct vipx_debug_log *log, const char *fmt, ...);
char *vipx_debug_log_print(struct vipx_debug_log *log);

void vipx_debug_memdump(const char *prefix, void *start, size_t size);

int vipx_debug_start(struct vipx_debug *debug);
int vipx_debug_stop(struct vipx_debug *debug);
int vipx_debug_open(struct vipx_debug *debug);
int vipx_debug_close(struct vipx_debug *debug);

int vipx_debug_probe(struct vipx_device *device);
void vipx_debug_remove(struct vipx_debug *debug);

#endif
