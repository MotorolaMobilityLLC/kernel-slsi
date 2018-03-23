/*
 * drivers/staging/android/ion/ion_debug.c
 *
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/list.h>
#include <linux/rbtree.h>
#include <linux/debugfs.h>
#include <linux/oom.h>

#include "ion.h"
#include "ion_exynos.h"

#define ION_MAX_LOGBUF 128

static size_t ion_print_buffer(struct ion_buffer *buffer, bool alive,
			       char *logbuf, int buflen)
{
	struct ion_iovm_map *iovm;
	int count;

	count = scnprintf(logbuf, buflen, "[%4d] %15s %#5lx %8zu : ",
			  buffer->id, buffer->heap->name, buffer->flags,
			  buffer->size / SZ_1K);
	buflen = max(0, buflen - count);
	/*
	 * It is okay even if count is larger than ION_MAX_LOGBUF
	 * because buflen then becomes zero and scnprintf never writes to
	 * the buffer if the size is zero.
	 */
	logbuf += count;

	if (alive) {
		list_for_each_entry(iovm, &buffer->iovas, list) {
			count = scnprintf(logbuf, buflen, "%s(%d) ",
					  dev_name(iovm->dev),
					  atomic_read(&iovm->mapcnt));
			buflen = max(0, buflen - count);
			logbuf += count;
		}
		scnprintf(logbuf, buflen, "\n");
	} else {
		scnprintf(logbuf, buflen, "(freed)\n");
	}

	return buffer->size;
}

static int ion_debug_buffers_show(struct seq_file *s, void *unused)
{
	struct ion_device *idev = s->private;
	struct rb_node *n;
	struct ion_buffer *buffer;
	struct ion_heap *heap;
	char logbuf[ION_MAX_LOGBUF];
	size_t total = 0;

	seq_printf(s, "[  id] %15s %5s %8s : %s\n",
		   "heap", "flags", "size(kb)", "iommu_mapped...");

	mutex_lock(&idev->buffer_lock);
	for (n = rb_first(&idev->buffers); n; n = rb_next(n)) {
		buffer = rb_entry(n, struct ion_buffer, node);
		mutex_lock(&buffer->lock);

		total += ion_print_buffer(buffer, true, logbuf, ION_MAX_LOGBUF);
		seq_puts(s, logbuf);

		mutex_unlock(&buffer->lock);
	}
	mutex_unlock(&idev->buffer_lock);

	down_read(&idev->lock);
	plist_for_each_entry(heap, &idev->heaps, node) {
		if (!(heap->flags & ION_HEAP_FLAG_DEFER_FREE))
			continue;

		spin_lock(&heap->free_lock);
		/* buffer lock is not required because the buffer is freed */
		list_for_each_entry(buffer, &heap->free_list, list) {
			ion_print_buffer(buffer, false, logbuf, ION_MAX_LOGBUF);
			seq_puts(s, logbuf);
		}
		spin_unlock(&heap->free_lock);
	}
	up_read(&idev->lock);

	seq_printf(s, "TOTAL: %zu kb\n\n", total / SZ_1K);

	down_read(&idev->lock);
	plist_for_each_entry(heap, &idev->heaps, node)
		if (heap->debug_show) {
			seq_printf(s, "Page pools of %s:\n", heap->name);
			heap->debug_show(heap, s, unused);
		}
	up_read(&idev->lock);

	return 0;
}

static int ion_debug_buffers_open(struct inode *inode, struct file *file)
{
	return single_open(file, ion_debug_buffers_show, inode->i_private);
}

static const struct file_operations debug_buffers_fops = {
	.open = ion_debug_buffers_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

struct ion_oom_notifier_struct {
	struct notifier_block nb;
	struct ion_device *idev;
};

static int ion_oom_notifier_fn(struct notifier_block *nb,
			       unsigned long action, void *data)
{
	struct ion_device *idev =
		container_of(nb, struct ion_oom_notifier_struct, nb)->idev;
	struct rb_node *n;
	struct ion_buffer *buffer;
	struct ion_heap *heap;
	char logbuf[ION_MAX_LOGBUF];
	size_t total = 0;

	pr_info("[  id] %15s %5s %8s : %s\n",
		"heap", "flags", "size(kb)", "iommu_mapped...");

	mutex_lock(&idev->buffer_lock);
	for (n = rb_first(&idev->buffers); n; n = rb_next(n)) {
		buffer = rb_entry(n, struct ion_buffer, node);
		mutex_lock(&buffer->lock);

		total += ion_print_buffer(buffer, true, logbuf, ION_MAX_LOGBUF);
		pr_info("%s", logbuf);

		mutex_unlock(&buffer->lock);
	}
	mutex_unlock(&idev->buffer_lock);

	down_read(&idev->lock);
	plist_for_each_entry(heap, &idev->heaps, node) {
		if (!(heap->flags & ION_HEAP_FLAG_DEFER_FREE))
			continue;

		spin_lock(&heap->free_lock);
		/* buffer lock is not required because the buffer is freed */
		list_for_each_entry(buffer, &heap->free_list, list) {
			ion_print_buffer(buffer, false, logbuf, ION_MAX_LOGBUF);
			pr_info("%s", logbuf);
		}
		spin_unlock(&heap->free_lock);
	}
	up_read(&idev->lock);

	pr_info("TOTAL: %zu kb\n", total / SZ_1K);

	return 0;
}

static struct ion_oom_notifier_struct ion_oom_notifier = {
	.nb = { .notifier_call = ion_oom_notifier_fn}
};

void ion_debug_initialize(struct ion_device *idev)
{
	struct dentry *buffer_file;

	buffer_file = debugfs_create_file("buffers", 0444, idev->debug_root,
					  idev, &debug_buffers_fops);
	if (!buffer_file)
		pr_err("%s: failed to create debugfs/ion/buffers\n", __func__);

	ion_oom_notifier.idev = idev;
	register_oom_notifier(&ion_oom_notifier.nb);
}
