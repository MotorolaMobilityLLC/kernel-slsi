/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <video/videonode.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/bug.h>

#include "vipx-config.h"
#include "vision-ioctl.h"
#include "vipx-vertex.h"
#include "vision-dev.h"
#include "vipx-device.h"
#include "vipx-graphmgr.h"
#include "vipx-graph.h"
#include "vipx-queue.h"
#include "vipx-control.h"

const struct vision_file_ops vipx_vertex_fops;
const const struct vertex_ioctl_ops vipx_vertex_ioctl_ops;

static int __vref_open(struct vipx_vertex *vertex)
{
	struct vipx_device *device = container_of(vertex, struct vipx_device, vertex);
	atomic_set(&vertex->start_cnt.refcount, 0);
	return vipx_device_open(device);
}

static int __vref_close(struct vipx_vertex *vertex)
{
	struct vipx_device *device = container_of(vertex, struct vipx_device, vertex);
	return vipx_device_close(device);
}

static int __vref_start(struct vipx_vertex *vertex)
{
	struct vipx_device *device = container_of(vertex, struct vipx_device, vertex);
	return vipx_device_start(device);
}

static int __vref_stop(struct vipx_vertex *vertex)
{
	struct vipx_device *device = container_of(vertex, struct vipx_device, vertex);
	return vipx_device_stop(device);
}

static inline void __vref_init(struct vipx_vertex_refcount *vref,
	struct vipx_vertex *vertex, int (*first)(struct vipx_vertex *vertex), int (*final)(struct vipx_vertex *vertex))
{
	vref->vertex = vertex;
	vref->first = first;
	vref->final = final;
	atomic_set(&vref->refcount, 0);
}

static inline int __vref_get(struct vipx_vertex_refcount *vref)
{
	return (atomic_inc_return(&vref->refcount) == 1) ? vref->first(vref->vertex) : 0;
}

static inline int __vref_put(struct vipx_vertex_refcount *vref)
{
	return (atomic_dec_return(&vref->refcount) == 0) ? vref->final(vref->vertex) : 0;
}

int vipx_vertex_probe(struct vipx_vertex *vertex, struct device *parent)
{
	int ret = 0;
	struct vision_device *vdev;

	BUG_ON(!vertex);
	BUG_ON(!parent);

	get_device(parent);
	mutex_init(&vertex->lock);
	__vref_init(&vertex->open_cnt, vertex, __vref_open, __vref_close);
	__vref_init(&vertex->start_cnt, vertex, __vref_start, __vref_stop);

	vdev = &vertex->vd;
	snprintf(vdev->name, sizeof(vdev->name), "%s", VIPX_VERTEX_NAME);
	vdev->fops		= &vipx_vertex_fops;
	vdev->ioctl_ops		= &vipx_vertex_ioctl_ops;
	vdev->release		= NULL;
	vdev->lock		= NULL;
	vdev->parent		= parent;
	vdev->type		= VISION_DEVICE_TYPE_VERTEX;
	dev_set_drvdata(&vdev->dev, vertex);

	ret = vision_register_device(vdev, VIPX_VERTEX_MINOR, vipx_vertex_fops.owner);
	if (ret) {
		probe_err("vision_register_device is fail(%d)\n", ret);
		goto p_err;
	}

	probe_info("%s():%d\n", __func__, ret);

p_err:
	return ret;
}

/*
 * =============================================================================
 * Video File Opertation
 * =============================================================================
 */

static int vipx_vertex_open(struct file *file)
{
	int ret = 0;
	struct vipx_vertex *vertex = dev_get_drvdata(&vision_devdata(file)->dev);
	struct vipx_device *device = container_of(vertex, struct vipx_device, vertex);
	struct mutex *lock = &vertex->lock;
	struct vipx_vertex_ctx *vctx = NULL;
	struct vipx_graph *graph = NULL;

	if (mutex_lock_interruptible(lock)) {
		vipx_err("mutex_lock_interruptible is fail\n");
		return -ERESTARTSYS;
	}

	ret = __vref_get(&vertex->open_cnt);
	if (ret) {
		vipx_err("vref_get is fail(%d)", ret);
		goto p_err;
	}

	ret = vipx_graph_create(&graph,
		&device->graphmgr,
		&device->system.memory);
	if (ret) {
		vipx_err("vipx_graph_create is fail(%d)", ret);
		goto p_err;
	}

	vctx			= &graph->vctx;
	vctx->idx		= graph->idx;
	vctx->vertex		= vertex;
	mutex_init(&vctx->lock);

	ret = vipx_queue_init(&vctx->queue, &device->system.memory, &vctx->lock);
	if (ret) {
		vipx_err("vipx_queue_init is fail(%d)", ret);
		goto p_err;
	}

	file->private_data = vctx;
	vctx->state = BIT(VIPX_VERTEX_OPEN);

p_err:
	vipx_info("%s():%d\n", __func__, ret);
	mutex_unlock(lock);
	return ret;
}

static int vipx_vertex_close(struct file *file)
{
	int ret = 0;
	struct vipx_vertex_ctx *vctx = file->private_data;
	struct vipx_graph *graph = container_of(vctx, struct vipx_graph, vctx);
	struct vipx_vertex *vertex = vctx->vertex;
	struct mutex *lock = &vertex->lock;
#if !DISABLE_VIPX_LOG
    int idx = vctx->idx;
#endif

	if (mutex_lock_interruptible(lock)) {
		vipx_ierr("mutex_lock_interruptible is fail\n", vctx);
		return -ERESTARTSYS;
	}

	ret = vipx_graph_destroy(graph);
	if (ret) {
		vipx_err("vipx_graph_destroy is fail(%d)", ret);
		goto p_err;
	}

	ret = __vref_put(&vertex->open_cnt);
	if (ret) {
		vipx_err("vref_put is fail(%d)", ret);
		goto p_err;
	}

p_err:
	vipx_info("[I%d]%s():%d\n", idx, __func__, ret);
	mutex_unlock(lock);
	return ret;
}

static unsigned int vipx_vertex_poll(struct file *file,
	poll_table *poll)
{
	int ret = 0;
	struct vipx_vertex_ctx *vctx = file->private_data;
	struct vipx_queue *queue = &vctx->queue;

	if (!(vctx->state & BIT(VIPX_VERTEX_START))) {
		vipx_ierr("invalid state(%X)", vctx, vctx->state);
		ret |= POLLERR;
		goto p_err;
	}

	ret = vipx_queue_poll(queue, file, poll);

p_err:
	return ret;
}

const struct vision_file_ops vipx_vertex_fops = {
	.owner		= THIS_MODULE,
	.open		= vipx_vertex_open,
	.release	= vipx_vertex_close,
	.poll		= vipx_vertex_poll,
	.ioctl		= vertex_ioctl,
	.compat_ioctl	= vertex_compat_ioctl32
};

/*
 * =============================================================================
 * Video Ioctl Opertation
 * =============================================================================
 */

static int vipx_vertex_s_graph(struct file *file, struct vs4l_graph *ginfo)
{
	int ret = 0;
	struct vipx_vertex_ctx *vctx = file->private_data;
	struct vipx_graph *graph = container_of(vctx, struct vipx_graph, vctx);
	struct mutex *lock = &vctx->lock;

	if (mutex_lock_interruptible(lock)) {
		vipx_ierr("mutex_lock_interruptible is fail\n", vctx);
		return -ERESTARTSYS;
	}

	if (!(vctx->state & BIT(VIPX_VERTEX_OPEN))) {
		vipx_ierr("invalid state(%X)\n", vctx, vctx->state);
		ret = -EINVAL;
		goto p_err;
	}

	ret = vipx_graph_config(graph, ginfo);
	if (ret) {
		vipx_err("vipx_graph_config is fail(%d)\n", ret);
		goto p_err;
	}

	vctx->state = BIT(VIPX_VERTEX_GRAPH);

p_err:
	vipx_iinfo("%s():%d\n", vctx, __func__, ret);
	mutex_unlock(lock);
	return ret;
}

static int vipx_vertex_s_format(struct file *file, struct vs4l_format_list *flist)
{
	int ret = 0;
    int bioctl32 = 0;
	struct vipx_vertex_ctx *vctx = file->private_data;
	struct vipx_queue *queue = &vctx->queue;
	struct mutex *lock = &vctx->lock;
	struct vs4l_format *tmp_buf = 0;

	if (mutex_lock_interruptible(lock)) {
		vipx_ierr("mutex_lock_interruptible is fail\n", vctx);
		return -ERESTARTSYS;
	}

	if (!(vctx->state & (BIT(VIPX_VERTEX_GRAPH) | BIT(VIPX_VERTEX_FORMAT)))) {
		vipx_ierr("invalid state(%X)\n", vctx, vctx->state);
		ret = -EINVAL;
		goto p_err;
	}

	tmp_buf = kzalloc(flist->count * sizeof(struct vs4l_format), GFP_KERNEL);
	if (!tmp_buf) {
		ret = -ENOMEM;
		goto p_err;
	}

	ret = copy_from_user(tmp_buf, (void __user *)flist->formats, flist->count * sizeof(struct vs4l_format));
	if (ret) {
		vipx_err("copy_from_user is fail(%d)\n", ret);
		//	kfree(tmp_buf);
		memcpy(tmp_buf, (void __user *)flist->formats, flist->count * sizeof(struct vs4l_format));
		ret = 0;
		//	goto p_err;
		bioctl32 = 1;

	}

	flist->formats = tmp_buf;

	ret = vipx_queue_s_format(queue, flist);
	if (ret) {
		vipx_ierr("vipx_queue_s_format is fail(%d)\n", vctx, ret);
		kfree(tmp_buf);
		goto p_err;
	}

    if (!bioctl32)
    {
        kfree(tmp_buf);
    }

	vctx->state = BIT(VIPX_VERTEX_FORMAT);

p_err:
	vipx_iinfo("%s():%d\n", vctx, __func__, ret);
	mutex_unlock(lock);
	return ret;
}

static int vipx_vertex_s_param(struct file *file, struct vs4l_param_list *plist)
{
	int ret = 0;
	struct vipx_vertex_ctx *vctx = file->private_data;
	struct vipx_graph *graph = container_of(vctx, struct vipx_graph, vctx);
	struct mutex *lock = &vctx->lock;

	if (mutex_lock_interruptible(lock)) {
		vipx_ierr("mutex_lock_interruptible is fail\n", vctx);
		return -ERESTARTSYS;
	}

	if (!(vctx->state & BIT(VIPX_VERTEX_START))) {
		vipx_ierr("invalid state(%X)\n", vctx, vctx->state);
		ret = -EINVAL;
		goto p_err;
	}

	ret = vipx_graph_param(graph, plist);
	if (ret) {
		vipx_err("vipx_graph_param is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	vipx_iinfo("%s():%d\n", vctx, __func__, ret);
	mutex_unlock(lock);
	return ret;
}

static int vipx_vertex_s_ctrl(struct file *file, struct vs4l_ctrl *ctrl)
{
	int ret = 0;
	struct vipx_vertex_ctx *vctx = file->private_data;
	struct vipx_vertex *vertex = dev_get_drvdata(&vision_devdata(file)->dev);
	struct vipx_device *device = container_of(vertex, struct vipx_device, vertex);
	struct vipx_graph *graph = container_of(vctx, struct vipx_graph, vctx);
	struct mutex *lock = &vctx->lock;

	if (mutex_lock_interruptible(lock)) {
		vipx_ierr("mutex_lock_interruptible is fail\n", vctx);
		return -ERESTARTSYS;
	}

	switch (ctrl->ctrl) {
	case VIPX_CTRL_DUMP:
		vipx_graph_print(graph);
		break;
	case VIPX_CTRL_MODE:
		device->mode = ctrl->value;
		break;
	case VIPX_CTRL_TEST:
		break;
	default:
		vipx_ierr("request control is invalid(%d)\n", vctx, ctrl->ctrl);
		ret = -EINVAL;
		break;
	}

	vipx_iinfo("%s(%d):%d\n", vctx, __func__, ctrl->ctrl, ret);
	mutex_unlock(lock);
	return ret;
}

static int vipx_vertex_qbuf(struct file *file, struct vs4l_container_list *clist)
{
	int ret = 0;
	struct vipx_vertex_ctx *vctx = file->private_data;
	struct vipx_queue *queue = &vctx->queue;
	struct mutex *lock = &vctx->lock;

	if (mutex_lock_interruptible(lock)) {
		vipx_ierr("mutex_lock_interruptible is fail\n", vctx);
		return -ERESTARTSYS;
	}

	if (!(vctx->state & BIT(VIPX_VERTEX_START))) {
		vipx_ierr("(%d) invalid state(%X)\n", vctx, clist->direction, vctx->state);
		ret = -EINVAL;
		goto p_err;
	}

	ret = vipx_queue_qbuf(queue, clist);
	if (ret) {
		vipx_ierr("(%d) vipx_queue_qbuf is fail(%d)\n", vctx, clist->direction, ret);
		goto p_err;
	}

p_err:
	mutex_unlock(lock);
	return ret;
}

static int vipx_vertex_dqbuf(struct file *file, struct vs4l_container_list *clist)
{
	int ret = 0;
	struct vipx_vertex_ctx *vctx = file->private_data;
	struct vipx_queue *queue = &vctx->queue;
	struct mutex *lock = &vctx->lock;
	bool nonblocking = file->f_flags & O_NONBLOCK;

	if (mutex_lock_interruptible(lock)) {
		vipx_ierr("mutex_lock_interruptible is fail\n", vctx);
		return -ERESTARTSYS;
	}

	if (!(vctx->state & BIT(VIPX_VERTEX_START))) {
		vipx_ierr("(%d) invalid state(%X)\n", vctx, clist->direction, vctx->state);
		ret = -EINVAL;
		goto p_err;
	}

	ret = vipx_queue_dqbuf(queue, clist, nonblocking);
	if (ret) {
		vipx_ierr("(%d) vipx_queue_dqbuf is fail(%d)\n", vctx, clist->direction, ret);
		goto p_err;
	}

p_err:
	mutex_unlock(lock);
	return ret;
}

static int vipx_vertex_streamon(struct file *file)
{
	int ret = 0;
	struct vipx_vertex_ctx *vctx = file->private_data;
	struct vipx_vertex *vertex = vctx->vertex;
	struct vipx_queue *queue = &vctx->queue;
	struct mutex *lock = &vctx->lock;

	if (mutex_lock_interruptible(lock)) {
		vipx_ierr("mutex_lock_interruptible is fail\n", vctx);
		return -ERESTARTSYS;
	}

	if (!(vctx->state & (BIT(VIPX_VERTEX_FORMAT) | BIT(VIPX_VERTEX_STOP)))) {
		vipx_ierr("invalid state(%X)\n", vctx, vctx->state);
		ret = -EINVAL;
		goto p_err;
	}

	ret = __vref_get(&vertex->start_cnt);
	if (ret) {
		vipx_err("vref_get is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vipx_queue_start(queue);
	if (ret) {
		vipx_ierr("vipx_queue_start is fail(%d)\n", vctx, ret);
		goto p_err;
	}

	vctx->state = BIT(VIPX_VERTEX_START);

p_err:
	vipx_iinfo("%s():%d\n", vctx, __func__, ret);
	mutex_unlock(lock);
	return ret;
}

static int vipx_vertex_streamoff(struct file *file)
{
	int ret = 0;
	struct vipx_vertex_ctx *vctx = file->private_data;
	struct vipx_vertex *vertex = vctx->vertex;
	struct vipx_queue *queue = &vctx->queue;
	struct mutex *lock = &vctx->lock;

	if (mutex_lock_interruptible(lock)) {
		vipx_ierr("mutex_lock_interruptible is fail\n", vctx);
		return -ERESTARTSYS;
	}

	if (!(vctx->state & BIT(VIPX_VERTEX_START))) {
		vipx_ierr("invalid state(%X)\n", vctx, vctx->state);
		ret = -EINVAL;
		goto p_err;
	}

	ret = vipx_queue_stop(queue);
	if (ret) {
		vipx_ierr("vipx_queue_stop is fail(%d)\n", vctx, ret);
		goto p_err;
	}

	ret = __vref_put(&vertex->start_cnt);
	if (ret) {
		vipx_err("vref_put is fail(%d)\n", ret);
		goto p_err;
	}

	vctx->state = BIT(VIPX_VERTEX_STOP);

p_err:
	vipx_iinfo("%s():%d\n", vctx, __func__, ret);
	mutex_unlock(lock);
	return ret;
}

const struct vertex_ioctl_ops vipx_vertex_ioctl_ops = {
	.vertexioc_s_graph	= vipx_vertex_s_graph,
	.vertexioc_s_format	= vipx_vertex_s_format,
	.vertexioc_s_param	= vipx_vertex_s_param,
	.vertexioc_s_ctrl	= vipx_vertex_s_ctrl,
	.vertexioc_qbuf		= vipx_vertex_qbuf,
	.vertexioc_dqbuf	= vipx_vertex_dqbuf,
	.vertexioc_streamon	= vipx_vertex_streamon,
	.vertexioc_streamoff	= vipx_vertex_streamoff
};
