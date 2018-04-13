/*
 * Copyright(C) 2018 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include <linux/dma-buf-container.h>

#include "dma-buf-container.h"

#define MAX_BUFCON_BUFS 32
#define MAX_BUFCON_SRC_BUFS (MAX_BUFCON_BUFS - 1)

static int dmabuf_container_get_user_data(unsigned int cmd, void __user *arg,
					  int fds[])
{
	int __user *ufds;
	int count, ret;

#ifdef CONFIG_COMPAT
	if (cmd == DMA_BUF_COMPAT_IOCTL_MERGE) {
		struct compat_dma_buf_merge __user *udata = arg;
		compat_uptr_t compat_ufds;

		ret = get_user(compat_ufds, &udata->dma_bufs);
		ret |= get_user(count, &udata->count);
		ufds = compat_ptr(compat_ufds);
	} else
#endif
	{
		struct dma_buf_merge __user *udata = arg;

		ret = get_user(ufds, &udata->dma_bufs);
		ret |= get_user(count, &udata->count);
	}

	if (ret) {
		pr_err("%s: failed to read data from user\n", __func__);
		return -EFAULT;
	}

	if ((count < 1) || (count > MAX_BUFCON_SRC_BUFS)) {
		pr_err("%s: invalid buffer count %u\n", __func__, count);
		return -EINVAL;
	}

	if (copy_from_user(fds, ufds, sizeof(fds[0]) * count)) {
		pr_err("%s: failed to read %u dma_bufs from user\n",
		       __func__, count);
		return -EFAULT;
	}

	return count;
}

static int dmabuf_container_put_user_data(unsigned int cmd, void __user *arg,
					  struct dma_buf *merged)
{
	int fd = get_unused_fd_flags(O_CLOEXEC);
	int ret;

	if (fd < 0) {
		pr_err("%s: failed to get new fd\n", __func__);
		return fd;
	}

#ifdef CONFIG_COMPAT
	if (cmd == DMA_BUF_COMPAT_IOCTL_MERGE) {
		struct compat_dma_buf_merge __user *udata = arg;

		ret = put_user(fd, &udata->dmabuf_container);
	} else
#endif
	{
		struct dma_buf_merge __user *udata = arg;

		ret = put_user(fd, &udata->dmabuf_container);
	}

	if (ret) {
		pr_err("%s: failed to store dmabuf_container fd to user\n",
		       __func__);

		put_unused_fd(fd);
		return ret;
	}

	fd_install(fd, merged->file);

	return 0;
}

/*
 * struct dma_buf_container - container description
 * @table:	dummy sg_table for container
 * @count:	the number of the buffers
 * @dmabufs:	dmabuf array representing each buffers
 */
struct dma_buf_container {
	struct sg_table	table;
	int		count;
	struct dma_buf	*dmabufs[0];
};

static void dmabuf_container_put_dmabuf(struct dma_buf_container *container)
{
	int i;

	for (i = 0; i < container->count; i++)
		dma_buf_put(container->dmabufs[i]);
}

static void dmabuf_container_dma_buf_release(struct dma_buf *dmabuf)
{
	dmabuf_container_put_dmabuf(dmabuf->priv);

	kfree(dmabuf->priv);
}

static struct sg_table *dmabuf_container_map_dma_buf(
				    struct dma_buf_attachment *attachment,
				    enum dma_data_direction direction)
{
	struct dma_buf_container *container = attachment->dmabuf->priv;

	return &container->table;
}

static void dmabuf_container_unmap_dma_buf(struct dma_buf_attachment *attach,
					   struct sg_table *table,
					   enum dma_data_direction direction)
{
}

static void *dmabuf_container_dma_buf_kmap(struct dma_buf *dmabuf,
					   unsigned long offset)
{
	return NULL;
}

static int dmabuf_container_mmap(struct dma_buf *dmabuf,
				 struct vm_area_struct *vma)
{
	pr_err("%s: dmabuf container does not support mmap\n", __func__);

	return -EACCES;
}

static struct dma_buf_ops dmabuf_container_dma_buf_ops = {
	.map_dma_buf = dmabuf_container_map_dma_buf,
	.unmap_dma_buf = dmabuf_container_unmap_dma_buf,
	.release = dmabuf_container_dma_buf_release,
	.map_atomic = dmabuf_container_dma_buf_kmap,
	.map = dmabuf_container_dma_buf_kmap,
	.mmap = dmabuf_container_mmap,
};

static bool is_dmabuf_container(struct dma_buf *dmabuf)
{
	return dmabuf->ops == &dmabuf_container_dma_buf_ops;
}

static struct dma_buf_container *get_container(struct dma_buf *dmabuf)
{
	return dmabuf->priv;
}

static int get_dma_buf_count(struct dma_buf *dmabuf)
{
	return is_dmabuf_container(dmabuf) ? get_container(dmabuf)->count : 1;
}

static struct dma_buf *__dmabuf_container_get_buffer(struct dma_buf *dmabuf,
						     int index)
{
	struct dma_buf *out = is_dmabuf_container(dmabuf)
			      ? get_container(dmabuf)->dmabufs[index] : dmabuf;

	get_dma_buf(out);

	return out;
}

static struct dma_buf *dmabuf_container_export(struct dma_buf_container *bufcon)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	unsigned long size = 0;
	int i;

	for (i = 0; i < bufcon->count; i++)
		size += bufcon->dmabufs[i]->size;

	exp_info.ops = &dmabuf_container_dma_buf_ops;
	exp_info.size = size;
	exp_info.flags = O_RDWR;
	exp_info.priv = bufcon;

	return dma_buf_export(&exp_info);
}

static struct dma_buf *create_dmabuf_container(struct dma_buf *base,
					       struct dma_buf *src[], int count)
{
	struct dma_buf_container *container;
	struct dma_buf *merged;
	int total = 0;
	int i, isrc;
	int nelem;

	for (i = 0; i < count; i++)
		total += get_dma_buf_count(src[i]);
	total += get_dma_buf_count(base);

	if (total > MAX_BUFCON_BUFS) {
		pr_err("%s: too many (%u) dmabuf merge request\n",
		       __func__, total);
		return ERR_PTR(-EINVAL);
	}

	container = kzalloc(sizeof(*container) +
			    sizeof(container->dmabufs[0]) * total, GFP_KERNEL);
	if (!container)
		return ERR_PTR(-ENOMEM);

	nelem = get_dma_buf_count(base);
	for (i = 0; i < nelem; i++)
		container->dmabufs[i] = __dmabuf_container_get_buffer(base, i);

	for (isrc = 0; isrc < count; isrc++)
		for (i = 0; i < get_dma_buf_count(src[isrc]); i++)
			container->dmabufs[nelem++] =
				__dmabuf_container_get_buffer(src[isrc], i);

	container->count = nelem;

	merged = dmabuf_container_export(container);
	if (IS_ERR(merged)) {
		pr_err("%s: failed to export dmabuf container.\n", __func__);
		dmabuf_container_put_dmabuf(container);
		kfree(container);
	}

	return merged;
}

static struct dma_buf *dmabuf_container_create(struct dma_buf *dmabuf,
					       int fds[], int count)
{
	struct dma_buf *src[MAX_BUFCON_SRC_BUFS];
	struct dma_buf *merged;
	int i;

	for (i = 0; i < count; i++) {
		src[i] = dma_buf_get(fds[i]);
		if (IS_ERR(src[i])) {
			merged = src[i];
			pr_err("%s: failed to get dmabuf of fd %d @ %u/%u\n",
			       __func__, fds[i], i, count);

			goto err_get;
		}
	}

	merged = create_dmabuf_container(dmabuf, src, count);
	/*
	 * reference count of dma_bufs (file->f_count) in src[] are increased
	 * again in create_dmabuf_container(). So they should be decremented
	 * before return.
	 */
err_get:
	while (i-- > 0)
		dma_buf_put(src[i]);

	return merged;
}

long dma_buf_merge_ioctl(struct dma_buf *dmabuf,
			 unsigned int cmd, unsigned long arg)
{
	int fds[MAX_BUFCON_SRC_BUFS];
	int count;
	struct dma_buf *merged;
	long ret;

	count = dmabuf_container_get_user_data(cmd, (void __user *)arg, fds);
	if (count < 0)
		return count;

	merged = dmabuf_container_create(dmabuf, fds, count);
	if (IS_ERR(merged))
		return PTR_ERR(merged);

	ret = dmabuf_container_put_user_data(cmd, (void __user *)arg, merged);
	if (ret) {
		dma_buf_put(merged);
		return ret;
	}

	return 0;
}

int dmabuf_container_get_count(struct dma_buf *dmabuf)
{
	if (!is_dmabuf_container(dmabuf))
		return -EINVAL;

	return get_container(dmabuf)->count;
}
EXPORT_SYMBOL_GPL(dmabuf_container_get_count);

struct dma_buf *dmabuf_container_get_buffer(struct dma_buf *dmabuf, int index)
{
	struct dma_buf_container *container = get_container(dmabuf);

	if (!is_dmabuf_container(dmabuf))
		return NULL;

	if (WARN_ON(index >= container->count))
		return NULL;

	get_dma_buf(container->dmabufs[index]);

	return container->dmabufs[index];
}
EXPORT_SYMBOL_GPL(dmabuf_container_get_buffer);

struct dma_buf *dma_buf_get_any(int fd)
{
	struct dma_buf *dmabuf = dma_buf_get(fd);
	struct dma_buf *anybuf = __dmabuf_container_get_buffer(dmabuf, 0);

	dma_buf_put(dmabuf);

	return anybuf;
}
EXPORT_SYMBOL_GPL(dma_buf_get_any);
