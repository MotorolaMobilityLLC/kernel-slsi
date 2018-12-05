/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/ctype.h>
#include <linux/uaccess.h>

#include "vipx-log.h"
#include "vipx-device.h"
#include "vipx-graphmgr.h"
#include "vipx-graph.h"
#include "vipx-debug.h"

#define DEBUG_FS_ROOT_NAME		"vipx"
#define DEBUG_FS_LOGFILE_NAME		"fw-msg"
#define DEBUG_FS_IMGFILE_NAME		"dump-img"
#define DEBUG_FS_GRPFILE_NAME		"graph"
#define DEBUG_FS_BUFFILE_NAME		"buffer"

#define DEBUG_MONITORING_PERIOD		(HZ * 5)
#define CHUNK_SIZE			(32)

static struct vipx_device *debug_device;

int vipx_debug_write_log_binary(void)
{
	int ret;
	struct vipx_system *sys;
	struct vipx_binary *bin;

	vipx_enter();
	sys = &debug_device->system;
	bin = &sys->binary;

	if (!sys->memory.debug.kvaddr)
		return -ENOMEM;

	if (!current->fs) {
		vipx_warn("Failed to write debug log as fs is invalid\n");
		return -ESRCH;
	}

	ret = vipx_binary_write(bin, VIPX_DEBUG_BIN_PATH,
			"vipx_log.bin",
			sys->memory.debug.kvaddr,
			sys->memory.debug.size);
	if (!ret)
		vipx_info("%s/vipx_log.bin was created for debugging\n",
				VIPX_DEBUG_BIN_PATH);

	vipx_leave();
	return ret;
}

int vipx_debug_dump_debug_regs(void)
{
	struct vipx_system *sys;

	vipx_enter();
	sys = &debug_device->system;

	sys->ctrl_ops->debug_dump(sys);
	vipx_leave();
	return 0;
}

int vipx_debug_atoi(const char *psz_buf)
{
	const char *pch = psz_buf;
	int base;

	while (isspace(*pch))
		pch++;

	if (*pch == '-' || *pch == '+') {
		base = 10;
		pch++;
	} else if (*pch && tolower(pch[strlen(pch) - 1]) == 'h') {
		base = 16;
	} else {
		base = 0;
	}

	return kstrtoul(pch, base, NULL);
}

int vipx_debug_bitmap_scnprintf(char *buf, unsigned int buflen,
		const unsigned long *maskp, int nmaskbits)
{
	int chunksz;
	int idx;
	unsigned int chunkmask;
	int word, bit, len;
	unsigned long val;
	const char *sep = "";

	chunksz = nmaskbits & (CHUNK_SIZE - 1);
	if (chunksz == 0)
		chunksz = CHUNK_SIZE;

	for (idx = ALIGN(nmaskbits, CHUNK_SIZE) - CHUNK_SIZE, len = 0; idx >= 0;
			idx -= CHUNK_SIZE) {
		chunkmask = ((1ULL << chunksz) - 1);
		word = idx / BITS_PER_LONG;
		bit = idx % BITS_PER_LONG;
		val = (maskp[word] >> bit) & chunkmask;
		len += scnprintf(buf + len, buflen - len, "%s%0*lx", sep,
				(chunksz + 3) / 4, val);
		chunksz = CHUNK_SIZE;
		sep = ",";
	}

	return len;
}

void vipx_dmsg_concate(struct vipx_debug_log *log, const char *fmt, ...)
{
	va_list ap;
	char term[50];
	size_t size;

	va_start(ap, fmt);
	vsnprintf(term, sizeof(term), fmt, ap);
	va_end(ap);

	if (log->dsentence_pos >= DEBUG_SENTENCE_MAX) {
		vipx_err("debug message(%zu) over max\n", log->dsentence_pos);
		return;
	}

	size = min((DEBUG_SENTENCE_MAX - log->dsentence_pos - 1),
			strlen(term));
	strncpy(log->dsentence + log->dsentence_pos, term, size);
	log->dsentence_pos += size;
	log->dsentence[log->dsentence_pos] = 0;
}

char *vipx_dmsg_print(struct vipx_debug_log *log)
{
	log->dsentence_pos = 0;
	return log->dsentence;
}

void vipx_debug_memdump(const char *prefix, void *start, size_t size)
{
	char log[30];

	snprintf(log, sizeof(log), "[VIPx]%s ", prefix);
	print_hex_dump(KERN_DEBUG, log, DUMP_PREFIX_OFFSET, 32, 4, start,
			size, false);
}

static int vipx_debug_log_open(struct inode *inode, struct file *file)
{
	vipx_enter();
	if (inode->i_private)
		file->private_data = inode->i_private;
	vipx_leave();
	return 0;
}

static ssize_t vipx_debug_log_read(struct file *file, char __user *user_buf,
		size_t buf_len, loff_t *ppos)
{
	int ret;
	size_t size;
	struct vipx_debug *debug;
	struct vipx_system *system;

	vipx_enter();
	debug = file->private_data;
	if (!debug) {
		vipx_err("Cannot find vipx_debug\n");
		return 0;
	}

	system = debug->system;
	if (!system) {
		vipx_err("Cannot find vipx_system\n");
		return 0;
	}

	if (buf_len > VIPX_DEBUG_SIZE)
		size = VIPX_DEBUG_SIZE;
	else
		size = buf_len;

	if (!system->memory.debug.kvaddr) {
		vipx_err("Cannot find debug region for vipx\n");
		return 0;
	}

	ret = copy_to_user(user_buf, system->memory.debug.kvaddr, size);
	if (ret)
		memcpy(user_buf, system->memory.debug.kvaddr, size);

	vipx_leave();
	return size;
}

static int __vipx_debug_grp_show(struct seq_file *s, void *unused)
{
	unsigned int idx;
	struct vipx_debug *debug;
	struct vipx_graphmgr *gmgr;
	struct vipx_graph *graph;

	vipx_enter();
	debug = s->private;

	seq_printf(s, "%7.s %7.s %7.s %7.s %7.s %7.s %7.s\n", "graph", "prio",
			"period", "input", "done", "cancel", "recent");

	gmgr = &debug->system->graphmgr;
	mutex_lock(&gmgr->mlock);
	for (idx = 0; idx < VIPX_MAX_GRAPH; ++idx) {
		graph = gmgr->graph[idx];

		if (!graph)
			continue;

		seq_printf(s, "%2d(%3d) %7d %7d %7d %7d %7d\n",
				graph->idx, graph->uid, graph->priority,
				graph->input_cnt, graph->done_cnt,
				graph->cancel_cnt, graph->recent);
	}
	mutex_unlock(&gmgr->mlock);

	vipx_leave();
	return 0;
}

static int vipx_debug_grp_open(struct inode *inode, struct file *file)
{
	vipx_check();
	return single_open(file, __vipx_debug_grp_show, inode->i_private);
}

static int __vipx_debug_buf_show(struct seq_file *s, void *unused)
{
	vipx_enter();
	vipx_leave();
	return 0;
}

static int vipx_debug_buf_open(struct inode *inode, struct file *file)
{
	vipx_enter();
	vipx_leave();
	return single_open(file, __vipx_debug_buf_show, inode->i_private);
}

static int vipx_debug_img_open(struct inode *inode, struct file *file)
{
	vipx_enter();
	if (inode->i_private)
		file->private_data = inode->i_private;
	vipx_leave();
	return 0;
}

static ssize_t vipx_debug_img_read(struct file *file, char __user *user_buf,
		size_t len, loff_t *ppos)
{
	int ret;
	size_t size;
	struct vipx_debug *debug;
	struct vipx_debug_imgdump *imgdump;

	vipx_enter();
	debug = file->private_data;
	imgdump = &debug->imgdump;

	if (!imgdump->kvaddr) {
		vipx_err("kvaddr of imgdump for debugging is NULL\n");
		return 0;
	}

	/* TODO check */
	//if (!imgdump->cookie) {
	//	vipx_err("cookie is NULL\n");
	//	return 0;
	//}

	if (len <= imgdump->length)
		size = len;
	else
		size = imgdump->length;

	if (!size) {
		imgdump->cookie = NULL;
		imgdump->kvaddr = NULL;
		imgdump->length = 0;
		imgdump->offset = 0;
		return 0;
	}

	/* HACK for test */
	memset(imgdump->kvaddr, 0x88, size / 2);
	/* TODO check */
	//vb2_ion_sync_for_device(imgdump->cookie, imgdump->offset,
	//		size, DMA_FROM_DEVICE);
	ret = copy_to_user(user_buf, imgdump->kvaddr, size);
	if (ret)
		memcpy(user_buf, imgdump->kvaddr, size);

	imgdump->offset += size;
	imgdump->length -= size;
	imgdump->kvaddr = (char *)imgdump->kvaddr + size;

	vipx_leave();
	return size;
}

static ssize_t vipx_debug_img_write(struct file *file,
		const char __user *user_buf, size_t len, loff_t *ppos)
{
	vipx_enter();
	vipx_leave();
	return len;
}

static const struct file_operations vipx_debug_log_fops = {
	.open	= vipx_debug_log_open,
	.read	= vipx_debug_log_read,
	.llseek	= default_llseek
};

static const struct file_operations vipx_debug_grp_fops = {
	.open	= vipx_debug_grp_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release
};

static const struct file_operations vipx_debug_buf_fops = {
	.open	= vipx_debug_buf_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release
};

static const struct file_operations vipx_debug_img_fops = {
	.open	= vipx_debug_img_open,
	.read	= vipx_debug_img_read,
	.write	= vipx_debug_img_write,
	.llseek	= default_llseek
};

static void __vipx_debug_monitor_fn(unsigned long data)
{
	struct vipx_debug *debug;
	struct vipx_debug_monitor *monitor;
	struct vipx_graphmgr *gmgr;
	struct vipx_system *system;
	struct vipx_interface *itf;

	vipx_enter();
	debug = (struct vipx_debug *)data;
	monitor = &debug->monitor;
	system = debug->system;
	gmgr = &system->graphmgr;
	itf = &system->interface;

	if (!test_bit(VIPX_DEBUG_STATE_START, &debug->state))
		return;

	if (monitor->tick_cnt == gmgr->tick_cnt)
		vipx_err("timer thread is stuck (%u,%u)\n",
				monitor->tick_cnt, gmgr->tick_pos);

	if (monitor->sched_cnt == gmgr->sched_cnt) {
		struct vipx_graph *graph;
		unsigned int idx;

		vipx_dbg("[DEBUGFS] GRAPH\n");
		for (idx = 0; idx < VIPX_MAX_GRAPH; idx++) {
			graph = gmgr->graph[idx];
			if (!graph)
				continue;

			vipx_graph_print(graph);
		}

		vipx_dbg("[DEBUGFS] GRAPH-MGR\n");
		vipx_taskdesc_print(gmgr);

		vipx_dbg("[DEBUGFS] INTERFACE-MGR\n");
		vipx_task_print_all(&itf->taskmgr);
	}

	monitor->tick_cnt = gmgr->tick_cnt;
	monitor->sched_cnt = gmgr->sched_cnt;
	monitor->done_cnt = itf->done_cnt;

	monitor->time_cnt++;
	mod_timer(&monitor->timer, jiffies + DEBUG_MONITORING_PERIOD);
	vipx_leave();
}

static int __vipx_debug_stop(struct vipx_debug *debug)
{
	struct vipx_debug_monitor *monitor;

	vipx_enter();
	monitor = &debug->monitor;

	if (!test_bit(VIPX_DEBUG_STATE_START, &debug->state))
		goto p_end;

	/* TODO check debug */
	//del_timer_sync(&monitor->timer);
	clear_bit(VIPX_DEBUG_STATE_START, &debug->state);

	vipx_leave();
p_end:
	return 0;
}

int vipx_debug_start(struct vipx_debug *debug)
{
	struct vipx_system *system = debug->system;
	struct vipx_graphmgr *gmgr = &system->graphmgr;
	struct vipx_debug_monitor *monitor = &debug->monitor;
	struct vipx_interface *itf = &system->interface;

	vipx_enter();
	monitor->time_cnt = 0;
	monitor->tick_cnt = gmgr->tick_cnt;
	monitor->sched_cnt = gmgr->sched_cnt;
	monitor->done_cnt = itf->done_cnt;

	init_timer(&monitor->timer);
	monitor->timer.expires = jiffies + DEBUG_MONITORING_PERIOD;
	monitor->timer.data = (unsigned long)debug;
	monitor->timer.function = __vipx_debug_monitor_fn;
	/* TODO check debug */
	//add_timer(&monitor->timer);

	set_bit(VIPX_DEBUG_STATE_START, &debug->state);

	vipx_leave();
	return 0;
}

int vipx_debug_stop(struct vipx_debug *debug)
{
	vipx_check();
	return __vipx_debug_stop(debug);
}

int vipx_debug_open(struct vipx_debug *debug)
{
	vipx_enter();
	vipx_leave();
	return 0;
}

int vipx_debug_close(struct vipx_debug *debug)
{
	vipx_check();
	return  __vipx_debug_stop(debug);
}

int vipx_debug_probe(struct vipx_device *device)
{
	struct vipx_debug *debug;

	vipx_enter();
	debug_device = device;
	debug = &device->debug;
	debug->system = &device->system;
	debug->state = 0;

	debug->root = debugfs_create_dir(DEBUG_FS_ROOT_NAME, NULL);
	if (!debug->root) {
		vipx_err("Filed to create debug file[%s]\n",
				DEBUG_FS_ROOT_NAME);
		goto p_end;
	}

	debug->logfile = debugfs_create_file(DEBUG_FS_LOGFILE_NAME, 0400,
			debug->root, debug, &vipx_debug_log_fops);
	if (!debug->logfile)
		vipx_err("Filed to create debug file[%s]\n",
				DEBUG_FS_LOGFILE_NAME);

	debug->grpfile = debugfs_create_file(DEBUG_FS_GRPFILE_NAME, 0400,
			debug->root, debug, &vipx_debug_grp_fops);
	if (!debug->grpfile)
		vipx_err("Filed to create debug file[%s]\n",
				DEBUG_FS_GRPFILE_NAME);

	debug->buffile = debugfs_create_file(DEBUG_FS_BUFFILE_NAME, 0400,
			debug->root, debug, &vipx_debug_buf_fops);
	if (!debug->buffile)
		vipx_err("Filed to create debug file[%s]\n",
				DEBUG_FS_BUFFILE_NAME);

	debug->imgdump.file = debugfs_create_file(DEBUG_FS_IMGFILE_NAME, 0400,
			debug->root, debug, &vipx_debug_img_fops);
	if (!debug->imgdump.file)
		vipx_err("Filed to create debug file[%s]\n",
				DEBUG_FS_IMGFILE_NAME);

	vipx_leave();
p_end:
	return 0;
}

void vipx_debug_remove(struct vipx_debug *debug)
{
	debugfs_remove_recursive(debug->root);
}
