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

#include "vipx-log.h"
#include "vipx-device.h"
#include "vipx-pm.h"
#include "vipx-debug.h"

static struct vipx_device *debug_device;
int vipx_debug_log_enable;

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

static int vipx_debug_dvfs_show(struct seq_file *file, void *unused)
{
#if defined(CONFIG_PM_DEVFREQ)
	struct vipx_debug *debug;
	struct vipx_pm *pm;
	int idx;

	vipx_enter();
	debug = file->private;
	pm = &debug->system->pm;

	mutex_lock(&pm->lock);
	seq_printf(file, "available level count is [L0 - L%d]\n",
			pm->qos_count - 1);
	for (idx = 0; idx < pm->qos_count; ++idx)
		seq_printf(file, "[L%02d] %d\n", idx, pm->qos_table[idx]);

	if (pm->default_qos < 0)
		seq_puts(file, "default: not set\n");
	else
		seq_printf(file, "default: L%d\n", pm->default_qos);

	if (pm->resume_qos < 0)
		seq_puts(file, "resume : not set\n");
	else
		seq_printf(file, "resume : L%d\n", pm->resume_qos);

	if (pm->current_qos < 0)
		seq_puts(file, "current: off\n");
	else
		seq_printf(file, "current: L%d\n", pm->current_qos);

	mutex_unlock(&pm->lock);

	vipx_leave();
	return 0;
#else
	seq_puts(file, "devfreq is not supported\n");
	return 0;
#endif
}

static int vipx_debug_dvfs_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, vipx_debug_dvfs_show, inode->i_private);
}

static ssize_t vipx_debug_dvfs_write(struct file *filp,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct seq_file *file;
	struct vipx_debug *debug;
	struct vipx_pm *pm;
	char buf[30];
	int ret, qos;
	ssize_t len;

	vipx_enter();
	file = filp->private_data;
	debug = file->private;
	pm = &debug->system->pm;

	if (count > sizeof(buf)) {
		vipx_err("[debugfs] writing size(%zd) is larger than buffer\n",
				count);
		goto out;
	}

	len = simple_write_to_buffer(buf, sizeof(buf), ppos, user_buf, count);
	if (len <= 0) {
		vipx_err("[debugfs] Failed to get user buf(%d)\n", len);
		goto out;
	}

	buf[len] = '\0';

	ret = sscanf(buf, "%d\n", &qos);
	if (ret != 1) {
		vipx_err("[debugfs] Failed to get qos value(%d)\n", ret);
		goto out;
	}

	ret = vipx_pm_qos_set_default(pm, qos);
	if (ret) {
		vipx_err("[debugfs] Failed to set default qos(%d)\n", ret);
		goto out;
	} else {
		vipx_info("[debugfs] default qos setting\n");
	}

	vipx_leave();
out:
	return count;
}

static const struct file_operations vipx_debug_dvfs_fops = {
	.open		= vipx_debug_dvfs_open,
	.read		= seq_read,
	.write		= vipx_debug_dvfs_write,
	.llseek		= seq_lseek,
	.release	= single_release
};

static int vipx_debug_clk_show(struct seq_file *file, void *unused)
{
	struct vipx_debug *debug;
	struct vipx_system *sys;
	struct vipx_pm *pm;
	const struct vipx_clk_ops *ops;
	int count, idx;
	unsigned long freq;
	const char *name;

	vipx_enter();
	debug = file->private;
	sys = debug->system;
	pm = &sys->pm;
	ops = sys->clk_ops;

	mutex_lock(&pm->lock);
	if (vipx_pm_qos_active(pm)) {
		count = ops->get_count(sys);
		for (idx = 0; idx < count; ++idx) {
			freq = ops->get_freq(sys, idx);
			name = ops->get_name(sys, idx);
			seq_printf(file, "%30s(%d) : %3lu.%06lu MHz\n",
					name, idx,
					freq / 1000000, freq % 1000000);
		}
	} else {
		seq_puts(file, "power off\n");
	}
	mutex_unlock(&pm->lock);

	vipx_leave();
	return 0;
}

static int vipx_debug_clk_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, vipx_debug_clk_show, inode->i_private);
}

static const struct file_operations vipx_debug_clk_fops = {
	.open		= vipx_debug_clk_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release
};

static int vipx_debug_wait_time_show(struct seq_file *file, void *unused)
{
	struct vipx_debug *debug;
	struct vipx_interface *itf;

	vipx_enter();
	debug = file->private;
	itf = &debug->system->interface;

	seq_printf(file, "response wait time %u ms\n", itf->wait_time);

	vipx_leave();
	return 0;
}

static int vipx_debug_wait_time_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, vipx_debug_wait_time_show, inode->i_private);
}

static ssize_t vipx_debug_wait_time_write(struct file *filp,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct seq_file *file;
	struct vipx_debug *debug;
	struct vipx_interface *itf;
	char buf[30];
	int ret, time;
	ssize_t len;

	vipx_enter();
	file = filp->private_data;
	debug = file->private;
	itf = &debug->system->interface;

	if (count > sizeof(buf)) {
		vipx_err("[debugfs] writing size(%zd) is larger than buffer\n",
				count);
		goto out;
	}

	len = simple_write_to_buffer(buf, sizeof(buf), ppos, user_buf, count);
	if (len <= 0) {
		vipx_err("[debugfs] Failed to get user buf(%d)\n", len);
		goto out;
	}

	buf[len] = '\0';

	ret = sscanf(buf, "%d\n", &time);
	if (ret != 1) {
		vipx_err("[debugfs] Failed to get time value(%d)\n", ret);
		goto out;
	}

	vipx_info("[debugfs] wait time is changed form %d ms to %d ms\n",
			itf->wait_time, time);
	itf->wait_time = time;

	vipx_leave();
out:
	return count;
}

static const struct file_operations vipx_debug_wait_time_fops = {
	.open		= vipx_debug_wait_time_open,
	.read		= seq_read,
	.write		= vipx_debug_wait_time_write,
	.llseek		= seq_lseek,
	.release	= single_release
};

int vipx_debug_start(struct vipx_debug *debug)
{
	vipx_enter();
	set_bit(VIPX_DEBUG_STATE_START, &debug->state);
	vipx_leave();
	return 0;
}

int vipx_debug_stop(struct vipx_debug *debug)
{
	vipx_enter();
	clear_bit(VIPX_DEBUG_STATE_START, &debug->state);
	vipx_leave();
	return 0;
}

int vipx_debug_open(struct vipx_debug *debug)
{
	vipx_enter();
	vipx_leave();
	return 0;
}

int vipx_debug_close(struct vipx_debug *debug)
{
	vipx_enter();
	if (debug->log_bin_enable)
		vipx_debug_write_log_binary();
	vipx_leave();
	return 0;
}

int vipx_debug_probe(struct vipx_device *device)
{
	struct vipx_debug *debug;

	vipx_enter();
	debug_device = device;
	debug = &device->debug;
	debug->system = &device->system;
	debug->state = 0;

	debug->root = debugfs_create_dir("vipx", NULL);
	if (!debug->root) {
		vipx_err("Failed to create debug root file\n");
		goto p_end;
	}

	debug->log = debugfs_create_u32("log", 0640, debug->root,
			&vipx_debug_log_enable);
	if (!debug->log)
		vipx_err("Failed to create log debugfs file\n");

	debug->log_bin = debugfs_create_u32("log_bin", 0640, debug->root,
			&debug->log_bin_enable);
	if (!debug->log_bin)
		vipx_err("Failed to create log_bin debugfs file\n");

	debug->dvfs = debugfs_create_file("dvfs", 0640, debug->root, debug,
			&vipx_debug_dvfs_fops);
	if (!debug->dvfs)
		vipx_err("Filed to create dvfs debugfs file\n");

	debug->clk = debugfs_create_file("clk", 0640, debug->root, debug,
			&vipx_debug_clk_fops);
	if (!debug->clk)
		vipx_err("Filed to create clk debugfs file\n");

	debug->wait_time = debugfs_create_file("wait_time", 0640, debug->root,
			debug, &vipx_debug_wait_time_fops);
	if (!debug->wait_time)
		vipx_err("Filed to create wait_time debugfs file\n");

	vipx_leave();
p_end:
	return 0;
}

void vipx_debug_remove(struct vipx_debug *debug)
{
	debugfs_remove_recursive(debug->root);
}
