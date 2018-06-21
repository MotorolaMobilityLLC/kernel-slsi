/*
 * drivers/media/platform/exynos/mfc/s5p_mfc_enc_vb2_ops.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "s5p_mfc_common.h"

#include "s5p_mfc_hwlock.h"
#include "s5p_mfc_nal_q.h"
#include "s5p_mfc_opr.h"
#include "s5p_mfc_sync.h"

#include "s5p_mfc_qos.h"
#include "s5p_mfc_queue.h"
#include "s5p_mfc_utils.h"
#include "s5p_mfc_buf.h"
#include "s5p_mfc_mem.h"

static int s5p_mfc_enc_queue_setup(struct vb2_queue *vq,
				unsigned int *buf_count, unsigned int *plane_count,
				unsigned int psize[], struct device *alloc_devs[])
{
	struct s5p_mfc_dev *dev;
	struct s5p_mfc_ctx *ctx = vq->drv_priv;
	struct s5p_mfc_enc *enc = ctx->enc_priv;
	struct s5p_mfc_raw_info *raw;
	int i;

	mfc_debug_enter();

	dev = ctx->dev;
	if (!dev) {
		mfc_err_dev("no mfc device to run\n");
		return -EINVAL;
	}

	if (ctx->state != MFCINST_GOT_INST &&
	    vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		mfc_err_ctx("invalid state: %d\n", ctx->state);
		return -EINVAL;
	}
	if (ctx->state >= MFCINST_FINISHING &&
	    vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_err_ctx("invalid state: %d\n", ctx->state);
		return -EINVAL;
	}

	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		mfc_debug(4, "enc dst\n");
		if (ctx->dst_fmt)
			*plane_count = ctx->dst_fmt->mem_planes;
		else
			*plane_count = MFC_ENC_CAP_PLANE_COUNT;

		if (*buf_count < 1)
			*buf_count = 1;
		if (*buf_count > MFC_MAX_BUFFERS)
			*buf_count = MFC_MAX_BUFFERS;

		psize[0] = enc->dst_buf_size;
		alloc_devs[0] = dev->device;
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_debug(4, "enc src\n");
		raw = &ctx->raw_buf;

		if (ctx->src_fmt)
			*plane_count = ctx->src_fmt->mem_planes;
		else
			*plane_count = MFC_ENC_OUT_PLANE_COUNT;

		if (*buf_count < 1)
			*buf_count = 1;
		if (*buf_count > MFC_MAX_BUFFERS)
			*buf_count = MFC_MAX_BUFFERS;

		if (*plane_count == 1) {
			psize[0] = raw->total_plane_size;
			alloc_devs[0] = dev->device;
		} else {
			for (i = 0; i < *plane_count; i++) {
				psize[i] = raw->plane_size[i];
				alloc_devs[i] = dev->device;
			}
		}
	} else {
		mfc_err_ctx("invalid queue type: %d\n", vq->type);
		return -EINVAL;
	}

	mfc_debug(2, "buf_count: %d, plane_count: %d, type: %#x\n",
			*buf_count, *plane_count, vq->type);
	for (i = 0; i < *plane_count; i++)
		mfc_debug(2, "plane[%d] size: %d\n", i, psize[i]);

	mfc_debug_leave();

	return 0;
}

static void s5p_mfc_enc_unlock(struct vb2_queue *q)
{
	struct s5p_mfc_ctx *ctx = q->drv_priv;
	struct s5p_mfc_dev *dev = ctx->dev;

	mutex_unlock(&dev->mfc_mutex);
}

static void s5p_mfc_enc_lock(struct vb2_queue *q)
{
	struct s5p_mfc_ctx *ctx = q->drv_priv;
	struct s5p_mfc_dev *dev = ctx->dev;

	mutex_lock(&dev->mfc_mutex);
}

static int s5p_mfc_enc_buf_init(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct s5p_mfc_ctx *ctx = vq->drv_priv;
	int ret;

	mfc_debug_enter();

	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = s5p_mfc_check_vb_with_fmt(ctx->dst_fmt, vb);
		if (ret < 0)
			return ret;

		if (call_cop(ctx, init_buf_ctrls, ctx, MFC_CTRL_TYPE_DST,
					vb->index) < 0)
			mfc_err_ctx("failed in init_buf_ctrls\n");

	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ret = s5p_mfc_check_vb_with_fmt(ctx->src_fmt, vb);
		if (ret < 0)
			return ret;

		if (call_cop(ctx, init_buf_ctrls, ctx, MFC_CTRL_TYPE_SRC,
					vb->index) < 0)
			mfc_err_ctx("failed in init_buf_ctrls\n");
	} else {
		mfc_err_ctx("inavlid queue type: %d\n", vq->type);
		return -EINVAL;
	}

	mfc_debug_leave();

	return 0;
}

static int s5p_mfc_enc_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct s5p_mfc_ctx *ctx = vq->drv_priv;
	struct s5p_mfc_enc *enc = ctx->enc_priv;
	struct s5p_mfc_raw_info *raw;
	unsigned int index = vb->index;
	struct s5p_mfc_buf *buf = vb_to_mfc_buf(vb);
	struct dma_buf *dmabuf[MFC_MAX_PLANES];
	int i, mem_get_count = 0;
	size_t buf_size;

	mfc_debug_enter();

	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		buf_size = vb2_plane_size(vb, 0);
		mfc_debug(2, "[STREAM] vb size: %lu, calc size: %u\n",
			buf_size, enc->dst_buf_size);

		if (buf_size < enc->dst_buf_size) {
			mfc_err_ctx("[STREAM] size(%d) is smaller than (%d)\n",
					buf_size, enc->dst_buf_size);
			return -EINVAL;
		}

		buf->addr[0][0] = s5p_mfc_mem_get_daddr_vb(vb, 0);
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		raw = &ctx->raw_buf;
		if (ctx->src_fmt->mem_planes == 1) {
			buf_size = vb2_plane_size(vb, 0);
			mfc_debug(2, "[FRAME] single plane vb size: %lu, calc size: %d\n",
					buf_size, raw->total_plane_size);
			if (buf_size < raw->total_plane_size) {
				mfc_err_ctx("[FRAME] single plane size(%d) is smaller than (%d)\n",
						buf_size, raw->total_plane_size);
				return -EINVAL;
			}
		} else {
			for (i = 0; i < ctx->src_fmt->mem_planes; i++) {
				buf_size = vb2_plane_size(vb, i);
				mfc_debug(2, "[FRAME] plane[%d] vb size: %lu, calc size: %d\n",
						i, buf_size, raw->plane_size[i]);
				if (buf_size < raw->plane_size[i]) {
					mfc_err_ctx("[FRAME] plane[%d] size(%d) is smaller than (%d)\n",
							i, buf_size, raw->plane_size[i]);
					return -EINVAL;
				}
			}
		}

		for (i = 0; i < ctx->src_fmt->mem_planes; i++) {
			dmabuf[i] = dma_buf_get(vb->planes[i].m.fd);
			if (IS_ERR(dmabuf[i])) {
				mfc_err_ctx("failed to get bufcon dmabuf\n");
				goto err_mem_put;
			}

			mem_get_count++;
			buf->num_bufs_in_vb = s5p_mfc_bufcon_get_buf_count(dmabuf[i]);
			mfc_debug(3, "bufcon count:%d\n", buf->num_bufs_in_vb);
			if (buf->num_bufs_in_vb == 0) {
				mfc_err_ctx("bufcon count couldn't be zero\n");
				goto err_mem_put;
			}

			if (buf->num_bufs_in_vb < 0)
				buf->num_bufs_in_vb = 0;

			if (!ctx->batch_mode && buf->num_bufs_in_vb > 0) {
				ctx->batch_mode = 1;
				mfc_debug(3, "buffer batch mode enabled\n");
			}

			if (buf->num_bufs_in_vb > 0) {
				int count = 0;

				ctx->framerate = buf->num_bufs_in_vb * ENC_DEFAULT_CAM_CAPTURE_FPS;
				mfc_debug(3, "framerate: %ld\n", ctx->framerate);

				count = s5p_mfc_bufcon_get_daddr(ctx, buf, dmabuf[i], i);
				if (count != buf->num_bufs_in_vb) {
					mfc_err_ctx("invalid buffer count %d != num_bufs_in_vb %d\n",
							count, buf->num_bufs_in_vb);
					goto err_mem_put;
				}

				dma_buf_put(dmabuf[i]);
			} else {
				dma_addr_t start_raw;

				dma_buf_put(dmabuf[i]);
				start_raw = s5p_mfc_mem_get_daddr_vb(vb, 0);
				if (start_raw == 0) {
					mfc_err_ctx("Plane mem not allocated\n");
					return -ENOMEM;
				}
				if (ctx->src_fmt->fourcc == V4L2_PIX_FMT_NV12N) {
					buf->addr[0][0] = start_raw;
					buf->addr[0][1] = NV12N_CBCR_BASE(start_raw,
							ctx->img_width,
							ctx->img_height);
				} else if (ctx->src_fmt->fourcc == V4L2_PIX_FMT_YUV420N) {
					buf->addr[0][0] = start_raw;
					buf->addr[0][1] = YUV420N_CB_BASE(start_raw,
							ctx->img_width,
							ctx->img_height);
					buf->addr[0][2] = YUV420N_CR_BASE(start_raw,
							ctx->img_width,
							ctx->img_height);
				} else {
					buf->addr[0][i] = s5p_mfc_mem_get_daddr_vb(vb, i);
				}
			}
		}

		if (call_cop(ctx, to_buf_ctrls, ctx, &ctx->src_ctrls[index]) < 0)
			mfc_err_ctx("failed in to_buf_ctrls\n");
	} else {
		mfc_err_ctx("inavlid queue type: %d\n", vq->type);
		return -EINVAL;
	}

	mfc_debug_leave();
	return 0;

err_mem_put:
	for (i = 0; i < mem_get_count; i++)
		dma_buf_put(dmabuf[i]);

	return -ENOMEM;
}

static void s5p_mfc_enc_buf_finish(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct s5p_mfc_ctx *ctx = vq->drv_priv;
	unsigned int index = vb->index;


	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (call_cop(ctx, to_ctx_ctrls, ctx, &ctx->dst_ctrls[index]) < 0)
			mfc_err_ctx("failed in to_ctx_ctrls\n");
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (call_cop(ctx, to_ctx_ctrls, ctx, &ctx->src_ctrls[index]) < 0)
			mfc_err_ctx("failed in to_ctx_ctrls\n");
	}
}

static void s5p_mfc_enc_buf_cleanup(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct s5p_mfc_ctx *ctx = vq->drv_priv;
	unsigned int index = vb->index;

	mfc_debug_enter();

	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (call_cop(ctx, cleanup_buf_ctrls, ctx,
					MFC_CTRL_TYPE_DST, index) < 0)
			mfc_err_ctx("failed in cleanup_buf_ctrls\n");
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (call_cop(ctx, cleanup_buf_ctrls, ctx,
					MFC_CTRL_TYPE_SRC, index) < 0)
			mfc_err_ctx("failed in cleanup_buf_ctrls\n");
	} else {
		mfc_err_ctx("s5p_mfc_enc_buf_cleanup: unknown queue type\n");
	}

	mfc_debug_leave();
}

static int s5p_mfc_enc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct s5p_mfc_ctx *ctx = q->drv_priv;
	struct s5p_mfc_dev *dev = ctx->dev;

	/* If context is ready then dev = work->data;schedule it to run */
	if (s5p_mfc_enc_ctx_ready(ctx)) {
		s5p_mfc_set_bit(ctx->num, &dev->work_bits);
	}

	s5p_mfc_try_run(dev);

	return 0;
}

static void s5p_mfc_enc_stop_streaming(struct vb2_queue *q)
{
	struct s5p_mfc_ctx *ctx = q->drv_priv;
	struct s5p_mfc_dev *dev = ctx->dev;
	int index = 0;
	int aborted = 0;
	int ret = 0;

	mfc_info_ctx("enc stop_streaming is called, hwlock : %d, type : %d\n",
				test_bit(ctx->num, &dev->hwlock.bits), q->type);
	MFC_TRACE_CTX("** ENC streamoff(type:%d)\n", q->type);

	/* If a H/W operation is in progress, wait for it complete */
	if (need_to_wait_nal_abort(ctx)) {
		if (s5p_mfc_wait_for_done_ctx(ctx, S5P_FIMV_R2H_CMD_NAL_ABORT_RET)) {
			mfc_err_ctx("time out during nal abort\n");
			s5p_mfc_cleanup_work_bit_and_try_run(ctx);
		}
		aborted = 1;
	}
	MFC_TRACE_CTX_HWLOCK("**ENC streamoff(type:%d)\n", q->type);
	ret = s5p_mfc_get_hwlock_ctx(ctx);
	if (ret < 0) {
		mfc_err_ctx("Failed to get hwlock\n");
		return;
	}

	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		s5p_mfc_cleanup_enc_dst_queue(ctx);

		while (index < MFC_MAX_BUFFERS) {
			index = find_next_bit(&ctx->dst_ctrls_avail,
					MFC_MAX_BUFFERS, index);
			if (index < MFC_MAX_BUFFERS)
				call_cop(ctx, reset_buf_ctrls, &ctx->dst_ctrls[index]);
			index++;
		}
	} else if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (ctx->state == MFCINST_RUNNING) {
			s5p_mfc_change_state(ctx, MFCINST_FINISHING);
			s5p_mfc_set_bit(ctx->num, &dev->work_bits);

			while (s5p_mfc_get_buf(&ctx->buf_queue_lock, &ctx->dst_buf_queue, MFC_BUF_NO_TOUCH_USED)) {
				ret = s5p_mfc_just_run(dev, ctx->num);
				if (ret) {
					mfc_err_ctx("Failed to run MFC\n");
					break;
				}
				if (s5p_mfc_wait_for_done_ctx(ctx, S5P_FIMV_R2H_CMD_FRAME_DONE_RET)) {
					mfc_err_ctx("Waiting for LAST_SEQ timed out\n");
					break;
				}
				if (ctx->state == MFCINST_RUNNING) {
					mfc_debug(2, "all encoded buffers out\n");
					break;
				}
			}
		}

		s5p_mfc_move_all_bufs(&ctx->buf_queue_lock, &ctx->src_buf_queue,
				&ctx->ref_buf_queue, MFC_QUEUE_ADD_BOTTOM);
		s5p_mfc_cleanup_enc_src_queue(ctx);

		while (index < MFC_MAX_BUFFERS) {
			index = find_next_bit(&ctx->src_ctrls_avail,
					MFC_MAX_BUFFERS, index);
			if (index < MFC_MAX_BUFFERS)
				call_cop(ctx, reset_buf_ctrls, &ctx->src_ctrls[index]);
			index++;
		}
	}

	if (aborted || ctx->state == MFCINST_FINISHING)
		s5p_mfc_change_state(ctx, MFCINST_RUNNING);

	mfc_debug(2, "buffer cleanup is done in stop_streaming, type : %d\n", q->type);

	s5p_mfc_clear_bit(ctx->num, &dev->work_bits);
	s5p_mfc_release_hwlock_ctx(ctx);

	if (s5p_mfc_is_work_to_do(dev))
		queue_work(dev->butler_wq, &dev->butler_work);
}

static void s5p_mfc_enc_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct s5p_mfc_ctx *ctx = vq->drv_priv;
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_buf *buf = vb_to_mfc_buf(vb);
	int i;

	mfc_debug_enter();

	buf->next_index = 0;
	buf->done_index = 0;

	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		mfc_debug(2, "[BUFINFO] ctx[%d] add dst index: %d, addr: 0x%08llx\n",
				ctx->num, vb->index, buf->addr[0][0]);

		/* Mark destination as available for use by MFC */
		s5p_mfc_add_tail_buf(&ctx->buf_queue_lock, &ctx->dst_buf_queue, buf);
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		for (i = 0; i < ctx->src_fmt->mem_planes; i++)
			mfc_debug(2, "[BUFINFO] ctx[%d] add src index: %d, addr[%d]: 0x%08llx\n",
					ctx->num, vb->index, i, buf->addr[0][i]);
		s5p_mfc_add_tail_buf(&ctx->buf_queue_lock, &ctx->src_buf_queue, buf);

		if (debug_ts == 1)
			mfc_info_ctx("[TS] framerate: %ld, timestamp: %lld\n",
					ctx->framerate, buf->vb.vb2_buf.timestamp);

		s5p_mfc_qos_update_last_framerate(ctx, buf->vb.vb2_buf.timestamp);
		s5p_mfc_qos_update_framerate(ctx);
	} else {
		mfc_err_ctx("unsupported buffer type (%d)\n", vq->type);
	}

	if (s5p_mfc_enc_ctx_ready(ctx)) {
		s5p_mfc_set_bit(ctx->num, &dev->work_bits);
	}
	s5p_mfc_try_run(dev);

	mfc_debug_leave();
}

struct vb2_ops s5p_mfc_enc_qops = {
	.queue_setup		= s5p_mfc_enc_queue_setup,
	.wait_prepare		= s5p_mfc_enc_unlock,
	.wait_finish		= s5p_mfc_enc_lock,
	.buf_init		= s5p_mfc_enc_buf_init,
	.buf_prepare		= s5p_mfc_enc_buf_prepare,
	.buf_finish		= s5p_mfc_enc_buf_finish,
	.buf_cleanup		= s5p_mfc_enc_buf_cleanup,
	.start_streaming	= s5p_mfc_enc_start_streaming,
	.stop_streaming		= s5p_mfc_enc_stop_streaming,
	.buf_queue		= s5p_mfc_enc_buf_queue,
};
