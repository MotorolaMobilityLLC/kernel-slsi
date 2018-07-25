/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <media/videobuf2-dma-sg.h>
#include <linux/dma-buf.h>
#include <linux/ion_exynos.h>

#include "vipx-config.h"
#include "vipx-debug.h"
#include "vipx-graphmgr.h"
#include "vipx-graph.h"
#include "vipx-system.h"

#define DEBUG_FS_ROOT_NAME	"vipx"
#define DEBUG_FS_LOGFILE_NAME	"fw-msg"
#define DEBUG_FS_IMGFILE_NAME	"dump-img"
#define DEBUG_FS_GRPFILE_NAME	"graph"
#define DEBUG_FS_BUFFILE_NAME	"buffer"

#define CHUNKSZ			32

s32 atoi(const char *psz_buf)
{
        const char *pch = psz_buf;
        s32 base = 0;

        while (isspace(*pch))
                pch++;

        if (*pch == '-' || *pch == '+') {
                base = 10;
                pch++;
        } else if (*pch && tolower(pch[strlen(pch) - 1]) == 'h') {
                base = 16;
        }

        return simple_strtoul(pch, NULL, base);
}

int bitmap_scnprintf(char *buf, unsigned int buflen,
        const unsigned long *maskp, int nmaskbits)
{
        int i, word, bit, len = 0;
        unsigned long val;
        const char *sep = "";
        int chunksz;
        u32 chunkmask;

        chunksz = nmaskbits & (CHUNKSZ - 1);
        if (chunksz == 0)
                chunksz = CHUNKSZ;

        i = ALIGN(nmaskbits, CHUNKSZ) - CHUNKSZ;
        for (; i >= 0; i -= CHUNKSZ) {
                chunkmask = ((1ULL << chunksz) - 1);
                word = i / BITS_PER_LONG;
                bit = i % BITS_PER_LONG;
                val = (maskp[word] >> bit) & chunkmask;
                len += scnprintf(buf+len, buflen-len, "%s%0*lx", sep,
                        (chunksz+3)/4, val);
                chunksz = CHUNKSZ;
                sep = ",";
        }
        return len;
}

void vipx_dmsg_concate(struct vipx_debug_log *log, const char *fmt, ...)
{
	va_list ap;
	char term[50];
	u32 copy_len;

	va_start(ap, fmt);
	vsnprintf(term, sizeof(term), fmt, ap);
	va_end(ap);

	if (log->dsentence_pos >= DEBUG_SENTENCE_MAX) {
		vipx_err("debug message(%zd) over max\n", log->dsentence_pos);
		return;
	}

	copy_len = min((DEBUG_SENTENCE_MAX - log->dsentence_pos - 1), strlen(term));
	strncpy(log->dsentence + log->dsentence_pos, term, copy_len);
	log->dsentence_pos += copy_len;
	log->dsentence[log->dsentence_pos] = 0;
}

char * vipx_dmsg_print(struct vipx_debug_log *log)
{
	log->dsentence_pos = 0;
	return log->dsentence;
}

int vipx_debug_memdump8(u8 *start, u8 *end)
{
	int ret = 0;
	u8 *cur;
	u32 items, offset;
	char term[50], sentence[250];

	cur = start;
	items = 0;
	offset = 0;

	memset(sentence, 0, sizeof(sentence));
	snprintf(sentence, sizeof(sentence), "[V] Memory Dump8(%p ~ %p)", start, end);

	while (cur < end) {
		if ((items % 16) == 0) {
#ifdef DEBUG_LOG_MEMORY
			printk(KERN_DEBUG "%s\n", sentence);
#else
			printk(KERN_INFO "%s\n", sentence);
#endif
			offset = 0;
			snprintf(term, sizeof(term), "[V] %p:      ", cur);
			snprintf(&sentence[offset], sizeof(sentence) - offset, "%s", term);
			offset += strlen(term);
			items = 0;
		}

		snprintf(term, sizeof(term), "%02X ", *cur);
		snprintf(&sentence[offset], sizeof(sentence) - offset, "%s", term);
		offset += strlen(term);
		cur++;
		items++;
	}

	if (items) {
#ifdef DEBUG_LOG_MEMORY
		printk(KERN_DEBUG "%s\n", sentence);
#else
		printk(KERN_INFO "%s\n", sentence);
#endif
	}

	ret = cur - end;

	return ret;
}


int vipx_debug_memdump16(u16 *start, u16 *end)
{
	int ret = 0;
	u16 *cur;
	u32 items, offset;
	char term[50], sentence[250];

	cur = start;
	items = 0;
	offset = 0;

	memset(sentence, 0, sizeof(sentence));
	snprintf(sentence, sizeof(sentence), "[V] Memory Dump16(%p ~ %p)", start, end);

	while (cur < end) {
		if ((items % 16) == 0) {
#ifdef DEBUG_LOG_MEMORY
			printk(KERN_DEBUG "%s\n", sentence);
#else
			printk(KERN_INFO "%s\n", sentence);
#endif
			offset = 0;
			snprintf(term, sizeof(term), "[V] %p:      ", cur);
			snprintf(&sentence[offset], sizeof(sentence) - offset, "%s", term);
			offset += strlen(term);
			items = 0;
		}

		snprintf(term, sizeof(term), "0x%04X ", *cur);
		snprintf(&sentence[offset], sizeof(sentence) - offset, "%s", term);
		offset += strlen(term);
		cur++;
		items++;
	}

	if (items) {
#ifdef DEBUG_LOG_MEMORY
		printk(KERN_DEBUG "%s\n", sentence);
#else
		printk(KERN_INFO "%s\n", sentence);
#endif
	}

	ret = cur - end;

	return ret;
}

int vipx_debug_memdump32(u32 *start, u32 *end)
{
	int ret = 0;
	u32 *cur;
	u32 items, offset;
	char term[50], sentence[250];

	cur = start;
	items = 0;
	offset = 0;

	memset(sentence, 0, sizeof(sentence));
	snprintf(sentence, sizeof(sentence), "[V] Memory Dump32(%p ~ %p)", start, end);

	while (cur < end) {
		if ((items % 8) == 0) {
#ifdef DEBUG_LOG_MEMORY
			printk(KERN_DEBUG "%s\n", sentence);
#else
			printk(KERN_INFO "%s\n", sentence);
#endif
			offset = 0;
			snprintf(term, sizeof(term), "[V] %p:      ", cur);
			snprintf(&sentence[offset], sizeof(sentence) - offset, "%s", term);
			offset += strlen(term);
			items = 0;
		}

		snprintf(term, sizeof(term), "0x%08X ", *cur);
		snprintf(&sentence[offset], sizeof(sentence) - offset, "%s", term);
		offset += strlen(term);
		cur++;
		items++;
	}

	if (items) {
#ifdef DEBUG_LOG_MEMORY
		printk(KERN_DEBUG "%s\n", sentence);
#else
		printk(KERN_INFO "%s\n", sentence);
#endif
	}

	ret = cur - end;

	return ret;
}

static int vipx_debug_log_open(struct inode *inode, struct file *file)
{
	if (inode->i_private)
		file->private_data = inode->i_private;

	return 0;
}

static ssize_t vipx_debug_log_read(struct file *file, char __user *user_buf,
	size_t buf_len, loff_t *ppos)
{
	int ret = 0;
	int size = 0;

	struct vipx_debug *debug;
	struct vipx_system *system;

	debug = file->private_data;
	if (debug == NULL) {
		vipx_err("Cannot find private data\n");
		return 0;
	}

	system = debug->system_data;
	if (system == NULL) {
		vipx_err("Cannot find system data\n");
		return 0;
	}

	if (buf_len > VIPX_DEBUG_SIZE)
		size = VIPX_DEBUG_SIZE;
	else
		size = buf_len;

	if (system->memory.info.kvaddr_debug == 0) {
		vipx_err("Cannot find debug region\n");
		return 0;
	}

	ret = copy_to_user(user_buf, system->memory.info.kvaddr_debug, size);
	if (ret) {
		vipx_err("copy_from_user is fail(%d)\n", ret);
		memcpy(user_buf, system->memory.info.kvaddr_debug, size);
		ret = 0;
		//	return 0;
	}

	return size;
}

static int vipx_debug_img_open(struct inode *inode, struct file *file)
{
	if (inode->i_private)
		file->private_data = inode->i_private;

	return 0;
}

static ssize_t vipx_debug_img_read(struct file *file, char __user *user_buf,
	size_t len, loff_t *ppos)
{
	size_t size = 0;
	struct vipx_debug *debug;
	struct vipx_debug_imgdump *imgdump;
	int ret = 0;

	debug = file->private_data;
	imgdump = &debug->imgdump;

	if (!imgdump->kvaddr) {
		vipx_err("kvaddr is NULL\n");
		return 0;
	}

	if (!imgdump->cookie) {
		vipx_err("cookie is NULL\n");
		return 0;
	}

	if (len <= imgdump->length)
		size = len;
	else
		size = imgdump->length;

	if (!size) {
		imgdump->cookie = NULL;
		imgdump->kvaddr = NULL;
		imgdump->length = 0;
		imgdump->offset = 0;
		goto p_err;
	}

	/* HACK for test */
	memset(imgdump->kvaddr, 0x88, size / 2);
	//vb2_ion_sync_for_device(imgdump->cookie, imgdump->offset, size, DMA_FROM_DEVICE);
	ret = copy_to_user(user_buf, imgdump->kvaddr, size);
	if (ret) {
		vipx_err("copy_from_user is fail(%d)\n", ret);
		memcpy(user_buf, imgdump->kvaddr, size);
		ret = 0;
		//	return 0;

	}

	vipx_info("DUMP : %p, SIZE : %zd\n", imgdump->kvaddr, size);

	imgdump->offset += size;
	imgdump->length -= size;
	imgdump->kvaddr = (char *)imgdump->kvaddr + size;

p_err:
	return size;
}

static ssize_t vipx_debug_img_write(struct file *file, const char __user *user_buf,
	size_t len, loff_t *ppos)
{
	return len;
}

static int vipx_debug_grp_show(struct seq_file *s, void *unused)
{
	u32 i;
	struct vipx_debug *debug = s->private;
	struct vipx_graphmgr *graphmgr = debug->graphmgr_data;
	struct vipx_graph *graph;

	seq_printf(s, "------------------------------------------"
			"----------------------------------------"
			"--------------------------------------\n");
	seq_printf(s, "%7.s %7.s %7.s %7.s %7.s %7.s %7.s\n",
			"graph", "prio", "period", "input", "done", "cancel", "recent");
	seq_printf(s, "------------------------------------------"
			"----------------------------------------"
			"--------------------------------------\n");

	mutex_lock(&graphmgr->mlock);
	for (i = 0; i < VIPX_MAX_GRAPH; ++i) {
		graph = graphmgr->graph[i];

		if (!graph)
			continue;

		seq_printf(s, "%2d(%3d) %7d %7d %7d %7d %7d\n",
			graph->idx, graph->uid, graph->priority,
			graph->input_cnt, graph->done_cnt, graph->cancel_cnt, graph->recent);
	}
	mutex_unlock(&graphmgr->mlock);

	seq_printf(s, "------------------------------------------"
			"----------------------------------------"
			"--------------------------------------\n");
	return 0;
}

static int vipx_debug_grp_open(struct inode *inode, struct file *file)
{
	return single_open(file, vipx_debug_grp_show, inode->i_private);
}

static int vipx_debug_buf_show(struct seq_file *s, void *unused)
{
	return 0;
}

static int vipx_debug_buf_open(struct inode *inode, struct file *file)
{
	return single_open(file, vipx_debug_buf_show, inode->i_private);
}

static const struct file_operations vipx_debug_log_fops = {
	.open	= vipx_debug_log_open,
	.read	= vipx_debug_log_read,
	.llseek	= default_llseek
};

static const struct file_operations vipx_debug_img_fops = {
	.open	= vipx_debug_img_open,
	.read	= vipx_debug_img_read,
	.write	= vipx_debug_img_write,
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

static void vipx_debug_monitor_fn(unsigned long data)
{
	struct vipx_debug_monitor *monitor = (struct vipx_debug_monitor *)data;
	struct vipx_debug *debug = container_of(monitor, struct vipx_debug, monitor);
	struct vipx_graphmgr *graphmgr = debug->graphmgr_data;
	struct vipx_system *system = debug->system_data;
	struct vipx_interface *interface = &system->interface;

	if (!test_bit(VIPX_DEBUG_STATE_START, &debug->state))
		return;

	if (monitor->tick_cnt == graphmgr->tick_cnt)
		vipx_err("timer thread is stuck(%d, %d)\n", monitor->tick_cnt, graphmgr->tick_pos);

	if (monitor->sched_cnt == graphmgr->sched_cnt) {
		struct vipx_graph *graph;
		u32 i;

		vipx_info("GRAPH--------------------------------------------------------------\n");
		for (i = 0; i < VIPX_MAX_GRAPH; i++) {
			graph = graphmgr->graph[i];
			if (!graph)
				continue;

			vipx_graph_print(graph);
		}

		vipx_info("GRAPH-MGR----------------------------------------------------------\n");
		vipx_taskdesc_print(graphmgr);

		vipx_info("INTERFACE-MGR------------------------------------------------------\n");
		vipx_interface_print(interface);

		vipx_info("-------------------------------------------------------------------\n");
		vipx_err("graph thread is stuck(%d, %d)\n", monitor->sched_cnt, graphmgr->sched_pos);
	}

	monitor->tick_cnt = graphmgr->tick_cnt;
	monitor->sched_cnt = graphmgr->sched_cnt;
	monitor->done_cnt = interface->done_cnt;
	vipx_info("TIME %d(%d, %d, %d)\n", monitor->time_cnt, monitor->sched_cnt, monitor->tick_cnt, monitor->done_cnt);

	monitor->time_cnt++;
	mod_timer(&monitor->timer, jiffies + DEBUG_MONITORING_PERIOD);
}

int __vipx_debug_stop(struct vipx_debug *debug)
{
	int ret = 0;
	struct vipx_debug_monitor *monitor = &debug->monitor;

	if (!test_bit(VIPX_DEBUG_STATE_START, &debug->state))
		goto p_err;

	del_timer(&monitor->timer);
	clear_bit(VIPX_DEBUG_STATE_START, &debug->state);

p_err:
	return ret;
}

int vipx_debug_probe(struct vipx_debug *debug, void *graphmgr_data, void *system_data)
{
	debug->graphmgr_data = graphmgr_data;
	debug->system_data = system_data;

	debug->root = debugfs_create_dir(DEBUG_FS_ROOT_NAME, NULL);
	if (debug->root)
		probe_info("%s is created\n", DEBUG_FS_ROOT_NAME);

	debug->logfile = debugfs_create_file(DEBUG_FS_LOGFILE_NAME, S_IRUSR,
		debug->root, debug, &vipx_debug_log_fops);
	if (debug->logfile)
		probe_info("%s is created\n", DEBUG_FS_LOGFILE_NAME);

	debug->imgdump.file = debugfs_create_file(DEBUG_FS_IMGFILE_NAME, S_IRUSR,
		debug->root, debug, &vipx_debug_img_fops);
	if (debug->imgdump.file)
		probe_info("%s is created\n", DEBUG_FS_IMGFILE_NAME);

	debug->grpfile = debugfs_create_file(DEBUG_FS_GRPFILE_NAME, S_IRUSR,
		debug->root, debug, &vipx_debug_grp_fops);
	if (debug->grpfile)
		probe_info("%s is created\n", DEBUG_FS_GRPFILE_NAME);

	debug->grpfile = debugfs_create_file(DEBUG_FS_BUFFILE_NAME, S_IRUSR,
		debug->root, debug, &vipx_debug_buf_fops);
	if (debug->buffile)
		probe_info("%s is created\n", DEBUG_FS_BUFFILE_NAME);

	clear_bit(VIPX_DEBUG_STATE_START, &debug->state);

	return 0;
}

int vipx_debug_open(struct vipx_debug *debug)
{
	return 0;
}

int vipx_debug_close(struct vipx_debug *debug)
{
	int ret = 0;

	ret = __vipx_debug_stop(debug);
	if (ret)
		vipx_err("__vipx_debug_stop is fail(%d)\n", ret);

	return ret;
}

int vipx_debug_start(struct vipx_debug *debug)
{
	int ret = 0;
	struct vipx_debug_monitor *monitor = &debug->monitor;
	struct vipx_graphmgr *graphmgr = debug->graphmgr_data;
	struct vipx_system *system = debug->system_data;
	struct vipx_interface *interface = &system->interface;

	monitor->tick_cnt = graphmgr->tick_cnt;
	monitor->sched_cnt = graphmgr->sched_cnt;
	monitor->done_cnt = interface->done_cnt;
	monitor->time_cnt = 0;

	set_bit(VIPX_DEBUG_STATE_START, &debug->state);

	init_timer(&monitor->timer);
	monitor->timer.expires = jiffies + DEBUG_MONITORING_PERIOD;
	monitor->timer.data = (unsigned long)monitor;
	monitor->timer.function = vipx_debug_monitor_fn;
	add_timer(&monitor->timer);

	return ret;
}

int vipx_debug_stop(struct vipx_debug *debug)
{
	int ret = 0;

	ret = __vipx_debug_stop(debug);
	if (ret)
		vipx_err("__vipx_debug_stop is fail(%d)\n", ret);

	return ret;
}
