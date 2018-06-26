/*
 * drivers/media/platform/exynos/mfc/mfc_cmd.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <trace/events/mfc.h>

#include "mfc_cmd.h"

#include "mfc_reg_api.h"
#include "mfc_hw_reg_api.h"
#include "mfc_mmcache.h"

#include "mfc_utils.h"
#include "mfc_buf.h"

int mfc_cmd_sys_init(struct mfc_dev *dev,
					enum mfc_buf_usage_type buf_type)
{
	struct mfc_ctx_buf_size *buf_size;
	struct mfc_special_buf *ctx_buf;

	mfc_debug_enter();

	if (!dev) {
		mfc_err_dev("no mfc device to run\n");
		return -EINVAL;
	}

	mfc_clean_dev_int_flags(dev);

	buf_size = dev->variant->buf_size->ctx_buf;
	ctx_buf = &dev->common_ctx_buf;
#ifdef CONFIG_EXYNOS_CONTENT_PATH_PROTECTION
	if (buf_type == MFCBUF_DRM)
		ctx_buf = &dev->drm_common_ctx_buf;
#endif
	MFC_WRITEL(ctx_buf->daddr, MFC_REG_CONTEXT_MEM_ADDR);
	MFC_WRITEL(buf_size->dev_ctx, MFC_REG_CONTEXT_MEM_SIZE);

	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_SYS_INIT);

	mfc_debug_leave();

	return 0;
}

void mfc_cmd_sleep(struct mfc_dev *dev)
{
	mfc_debug_enter();

	mfc_clean_dev_int_flags(dev);
	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_SLEEP);

	mfc_debug_leave();
}

void mfc_cmd_wakeup(struct mfc_dev *dev)
{
	mfc_debug_enter();

	mfc_clean_dev_int_flags(dev);
	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_WAKEUP);

	mfc_debug_leave();
}

/* Open a new instance and get its number */
int mfc_cmd_open_inst(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev;

	mfc_debug_enter();

	if (!ctx) {
		mfc_err_dev("no mfc context to run\n");
		return -EINVAL;
	}
	dev = ctx->dev;
	mfc_debug(2, "Requested codec mode: %d\n", ctx->codec_mode);

	MFC_WRITEL(ctx->codec_mode, MFC_REG_CODEC_TYPE);
	MFC_WRITEL(ctx->instance_ctx_buf.daddr, MFC_REG_CONTEXT_MEM_ADDR);
	MFC_WRITEL(ctx->instance_ctx_buf.size, MFC_REG_CONTEXT_MEM_SIZE);
	if (ctx->type == MFCINST_DECODER)
		MFC_WRITEL(ctx->dec_priv->crc_enable, MFC_REG_D_CRC_CTRL);

	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_OPEN_INSTANCE);

	mfc_debug_leave();

	return 0;
}

/* Close instance */
int mfc_cmd_close_inst(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev;

	mfc_debug_enter();

	if (!ctx) {
		mfc_err_dev("no mfc context to run\n");
		return -EINVAL;
	}
	dev = ctx->dev;

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);

	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_CLOSE_INSTANCE);

	mfc_debug_leave();

	return 0;
}

int mfc_cmd_dpb_flush(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;

	if (ON_RES_CHANGE(ctx))
		mfc_err_ctx("dpb flush on res change(state:%d)\n", ctx->state);

	mfc_clean_ctx_int_flags(ctx);

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);
	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_DPB_FLUSH);

	return 0;
}

int mfc_cmd_cache_flush(struct mfc_dev *dev)
{
	if (!dev) {
		mfc_err_dev("no mfc device to run\n");
		return -EINVAL;
	}

	mfc_clean_dev_int_flags(dev);
	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_CACHE_FLUSH);

	return 0;
}

int mfc_cmd_dec_init_buffers(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev;
	struct mfc_dec *dec;
	unsigned int reg = 0, pix_val;
	int ret;

	if (!ctx) {
		mfc_err_dev("no mfc context to run\n");
		return -EINVAL;
	}
	dec = ctx->dec_priv;
	dev = ctx->dev;
	if (!dev) {
		mfc_err_ctx("no mfc device to run\n");
		return -EINVAL;
	}

	switch (ctx->dst_fmt->fourcc) {
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV12N:
	case V4L2_PIX_FMT_NV12MT_16X16:
	case V4L2_PIX_FMT_NV16M:
	case V4L2_PIX_FMT_NV12N_10B:
	case V4L2_PIX_FMT_NV12M_S10B:
	case V4L2_PIX_FMT_NV16M_S10B:
	case V4L2_PIX_FMT_NV12M_P010:
	case V4L2_PIX_FMT_NV16M_P210:
		pix_val = 0;
		break;
	case V4L2_PIX_FMT_NV21M:
	case V4L2_PIX_FMT_NV61M:
	case V4L2_PIX_FMT_NV21M_S10B:
	case V4L2_PIX_FMT_NV61M_S10B:
	case V4L2_PIX_FMT_NV21M_P010:
	case V4L2_PIX_FMT_NV61M_P210:
		pix_val = 1;
		break;
	case V4L2_PIX_FMT_YVU420M:
		pix_val = 2;
		break;
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YUV420N:
		pix_val = 3;
		break;
	default:
		pix_val = 0;
		break;
	}
	reg = MFC_READL(MFC_REG_PIXEL_FORMAT);
	reg &= ~(0xF);
	reg |= pix_val & 0xF;
	MFC_WRITEL(reg, MFC_REG_PIXEL_FORMAT);
	mfc_debug(2, "[FRAME] pixel format: %d, mem_type_10bit should be fixed on SEQ_START(reg: %#x)\n",
			pix_val, reg);

	mfc_clean_ctx_int_flags(ctx);
	ret = mfc_set_dec_codec_buffers(ctx);
	if (ret) {
		mfc_info_ctx("isn't enough codec buffer size, re-alloc!\n");

		if (dev->has_mmcache && dev->mmcache.is_on_status)
			mfc_invalidate_mmcache(dev);

		mfc_release_codec_buffers(ctx);
		ret = mfc_alloc_codec_buffers(ctx);
		if (ret) {
			mfc_err_ctx("Failed to allocate decoding buffers\n");
			return ret;
		}
		ret = mfc_set_dec_codec_buffers(ctx);
		if (ret) {
			mfc_err_ctx("Failed to alloc frame mem\n");
			return ret;
		}
	}

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);

	if (sfr_dump & MFC_DUMP_DEC_INIT_BUFS)
		call_dop(dev, dump_regs, dev);

	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_INIT_BUFFERS);

	return ret;
}

int mfc_cmd_enc_init_buffers(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev;
	int ret;

	dev = ctx->dev;
	if (!dev) {
		mfc_err_dev("no mfc device to run\n");
		return -EINVAL;
	}

	/*
	 * Header was generated now starting processing
	 * First set the reference frame buffers
	 */
	if (!ctx->codec_buffer_allocated) {
		mfc_info_ctx("there isn't codec buffer, re-alloc!\n");
		ret = mfc_alloc_codec_buffers(ctx);
		if (ret) {
			mfc_err_ctx("Failed to allocate encoding buffers\n");
			return ret;
		}
	}

	mfc_clean_ctx_int_flags(ctx);
	ret = mfc_set_enc_codec_buffers(ctx);
	if (ret) {
		mfc_info_ctx("isn't enough codec buffer size, re-alloc!\n");

		if (dev->has_mmcache && dev->mmcache.is_on_status)
			mfc_invalidate_mmcache(dev);

		mfc_release_codec_buffers(ctx);
		ret = mfc_alloc_codec_buffers(ctx);
		if (ret) {
			mfc_err_ctx("Failed to allocate encoding buffers\n");
			return ret;
		}
		ret = mfc_set_enc_codec_buffers(ctx);
		if (ret) {
			mfc_err_ctx("Failed to set enc codec buffers\n");
			return ret;
		}
	}

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);

	if (sfr_dump & MFC_DUMP_ENC_INIT_BUFS)
		call_dop(dev, dump_regs, dev);

	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_INIT_BUFFERS);

	return ret;
}
