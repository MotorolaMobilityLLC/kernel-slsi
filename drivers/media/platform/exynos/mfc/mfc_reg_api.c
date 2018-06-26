/*
 * drivers/media/platform/exynos/mfc/mfc_reg.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/delay.h>

#include "mfc_reg_api.h"

void mfc_dbg_enable(struct mfc_dev *dev)
{
	mfc_debug(2, "MFC debug info enable\n");
	MFC_WRITEL(0x1, MFC_REG_DBG_INFO_ENABLE);
}

void mfc_dbg_disable(struct mfc_dev *dev)
{
	mfc_debug(2, "MFC debug info disable\n");
	MFC_WRITEL(0x0, MFC_REG_DBG_INFO_ENABLE);
}

void mfc_dbg_set_addr(struct mfc_dev *dev)
{
	struct mfc_ctx_buf_size *buf_size = dev->variant->buf_size->ctx_buf;

	memset((void *)dev->dbg_info_buf.vaddr, 0, buf_size->dbg_info_buf);

	MFC_WRITEL(dev->dbg_info_buf.daddr, MFC_REG_DBG_BUFFER_ADDR);
	MFC_WRITEL(buf_size->dbg_info_buf, MFC_REG_DBG_BUFFER_SIZE);
}

void mfc_otf_set_frame_addr(struct mfc_ctx *ctx, int num_planes)
{
	struct mfc_dev *dev = ctx->dev;
	struct _otf_handle *handle = ctx->otf_handle;
	struct _otf_buf_addr *buf_addr = &handle->otf_buf_addr;
	int index = handle->otf_buf_index;
	int i;

	for (i = 0; i < num_planes; i++) {
		mfc_debug(2, "[OTF][FRAME] set frame buffer[%d], 0x%08llx)\n",
				i, buf_addr->otf_daddr[index][i]);
		MFC_WRITEL(buf_addr->otf_daddr[index][i],
				MFC_REG_E_SOURCE_FIRST_ADDR + (i * 4));
	}
}

void mfc_otf_set_stream_size(struct mfc_ctx *ctx, unsigned int size)
{
	struct mfc_dev *dev = ctx->dev;
	struct _otf_handle *handle = ctx->otf_handle;
	struct _otf_debug *debug = &handle->otf_debug;
	struct mfc_special_buf *buf;

	mfc_debug(2, "[OTF] set stream buffer full size, %u\n", size);
	MFC_WRITEL(size, MFC_REG_E_STREAM_BUFFER_SIZE);

	if (otf_dump && !ctx->is_drm) {
		buf = &debug->stream_buf[debug->frame_cnt];
		mfc_debug(2, "[OTF] set stream addr for debugging\n");
		mfc_debug(2, "[OTF][STREAM] buf[%d] daddr: 0x%08llx\n",
				debug->frame_cnt, buf->daddr);
		MFC_WRITEL(buf->daddr, MFC_REG_E_STREAM_BUFFER_ADDR);
	}
}

void mfc_otf_set_hwfc_index(struct mfc_ctx *ctx, int job_id)
{
	struct mfc_dev *dev = ctx->dev;

	mfc_debug(2, "[OTF] set hwfc index, %d\n", job_id);
	HWFC_WRITEL(job_id, HWFC_ENCODING_IDX);
}

/* Set decoding frame buffer */
int mfc_set_dec_codec_buffers(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev;
	struct mfc_dec *dec;
	unsigned int i;
	size_t frame_size_mv;
	dma_addr_t buf_addr;
	int buf_size;
	int align_gap;
	struct mfc_raw_info *raw;
	unsigned int reg = 0;

	if (!ctx) {
		mfc_err_dev("no mfc context to run\n");
		return -EINVAL;
	}

	dev = ctx->dev;
	if (!dev) {
		mfc_err_dev("no mfc device to run\n");
		return -EINVAL;
	}

	dec = ctx->dec_priv;
	if (!dec) {
		mfc_err_dev("no mfc decoder to run\n");
		return -EINVAL;
	}

	raw = &ctx->raw_buf;
	buf_addr = ctx->codec_buf.daddr;
	buf_size = ctx->codec_buf.size;

	mfc_debug(2, "[MEMINFO] codec buf 0x%llx size: %d\n", buf_addr, buf_size);
	mfc_debug(2, "Total DPB COUNT: %d, display delay: %d\n",
			dec->total_dpb_count, dec->display_delay);

	/* set decoder DPB size, stride */
	MFC_WRITEL(dec->total_dpb_count, MFC_REG_D_NUM_DPB);
	for (i = 0; i < raw->num_planes; i++) {
		mfc_debug(2, "[FRAME] buf[%d] size: %d, stride: %d\n",
				i, raw->plane_size[i], raw->stride[i]);
		MFC_WRITEL(raw->plane_size[i], MFC_REG_D_FIRST_PLANE_DPB_SIZE + (i * 4));
		MFC_WRITEL(ctx->raw_buf.stride[i],
				MFC_REG_D_FIRST_PLANE_DPB_STRIDE_SIZE + (i * 4));
		if (ctx->is_10bit) {
			MFC_WRITEL(raw->stride_2bits[i], MFC_REG_D_FIRST_PLANE_2BIT_DPB_STRIDE_SIZE + (i * 4));
			MFC_WRITEL(raw->plane_size_2bits[i], MFC_REG_D_FIRST_PLANE_2BIT_DPB_SIZE + (i * 4));
			mfc_debug(2, "[FRAME][10BIT] 2bits buf[%d] size: %d, stride: %d\n",
					i, raw->plane_size_2bits[i], raw->stride_2bits[i]);
		}
	}

	/* set codec buffers */
	MFC_WRITEL(buf_addr, MFC_REG_D_SCRATCH_BUFFER_ADDR);
	MFC_WRITEL(ctx->scratch_buf_size, MFC_REG_D_SCRATCH_BUFFER_SIZE);
	buf_addr += ctx->scratch_buf_size;
	buf_size -= ctx->scratch_buf_size;

	if (IS_H264_DEC(ctx) || IS_H264_MVC_DEC(ctx) || IS_HEVC_DEC(ctx) || IS_BPG_DEC(ctx))
		MFC_WRITEL(ctx->mv_size, MFC_REG_D_MV_BUFFER_SIZE);

	if (IS_VP9_DEC(ctx)){
		MFC_WRITEL(buf_addr, MFC_REG_D_STATIC_BUFFER_ADDR);
		MFC_WRITEL(DEC_STATIC_BUFFER_SIZE, MFC_REG_D_STATIC_BUFFER_SIZE);
		buf_addr += DEC_STATIC_BUFFER_SIZE;
		buf_size -= DEC_STATIC_BUFFER_SIZE;
	}

	if (IS_MPEG4_DEC(ctx) && dec->loop_filter_mpeg4) {
		mfc_debug(2, "Add DPB for loop filter of MPEG4\n");
		for (i = 0; i < NUM_MPEG4_LF_BUF; i++) {
			MFC_WRITEL(buf_addr, MFC_REG_D_POST_FILTER_LUMA_DPB0 + (4 * i));
			buf_addr += ctx->loopfilter_luma_size;
			buf_size -= ctx->loopfilter_luma_size;

			MFC_WRITEL(buf_addr, MFC_REG_D_POST_FILTER_CHROMA_DPB0 + (4 * i));
			buf_addr += ctx->loopfilter_chroma_size;
			buf_size -= ctx->loopfilter_chroma_size;
		}
		reg |= ((dec->loop_filter_mpeg4 & MFC_REG_D_INIT_BUF_OPT_LF_CTRL_MASK)
				<< MFC_REG_D_INIT_BUF_OPT_LF_CTRL_SHIFT);
	}

	reg |= (0x1 << MFC_REG_D_INIT_BUF_OPT_DYNAMIC_DPB_SET_SHIFT);

	if (CODEC_NOT_CODED(ctx)) {
		reg |= (0x1 << MFC_REG_D_INIT_BUF_OPT_COPY_NOT_CODED_SHIFT);
		mfc_debug(2, "Notcoded frame copy mode start\n");
	}
	/* Enable 10bit Dithering */
	if (ctx->is_10bit) {
		reg |= (0x1 << MFC_REG_D_INIT_BUF_OPT_DITHERING_EN_SHIFT);
		/* 64byte align, It is vaid only for VP9 */
		reg |= (0x1 << MFC_REG_D_INIT_BUF_OPT_STRIDE_SIZE_ALIGN);
	} else {
		/* 16byte align, It is vaid only for VP9 */
		reg &= ~(0x1 << MFC_REG_D_INIT_BUF_OPT_STRIDE_SIZE_ALIGN);
	}

	MFC_WRITEL(reg, MFC_REG_D_INIT_BUFFER_OPTIONS);

	frame_size_mv = ctx->mv_size;
	MFC_WRITEL(dec->mv_count, MFC_REG_D_NUM_MV);
	if (IS_H264_DEC(ctx) || IS_H264_MVC_DEC(ctx) || IS_HEVC_DEC(ctx) || IS_BPG_DEC(ctx)) {
		for (i = 0; i < dec->mv_count; i++) {
			/* To test alignment */
			align_gap = buf_addr;
			buf_addr = ALIGN(buf_addr, 16);
			align_gap = buf_addr - align_gap;
			buf_size -= align_gap;

			MFC_WRITEL(buf_addr, MFC_REG_D_MV_BUFFER0 + i * 4);
			buf_addr += frame_size_mv;
			buf_size -= frame_size_mv;
		}
	}

	mfc_debug(2, "[MEMINFO] codec buf 0x%llx, remained size: %d\n", buf_addr, buf_size);
	if (buf_size < 0) {
		mfc_debug(2, "[MEMINFO] Not enough memory has been allocated\n");
		return -ENOMEM;
	}

	return 0;
}

/* Set encoding ref & codec buffer */
int mfc_set_enc_codec_buffers(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_enc *enc = ctx->enc_priv;
	dma_addr_t buf_addr;
	int buf_size;
	int i;

	mfc_debug_enter();

	buf_addr = ctx->codec_buf.daddr;
	buf_size = ctx->codec_buf.size;

	mfc_debug(2, "[MEMINFO] codec buf 0x%llx, size: %d\n", buf_addr, buf_size);
	mfc_debug(2, "DPB COUNT: %d\n", ctx->dpb_count);

	MFC_WRITEL(buf_addr, MFC_REG_E_SCRATCH_BUFFER_ADDR);
	MFC_WRITEL(ctx->scratch_buf_size, MFC_REG_E_SCRATCH_BUFFER_SIZE);
	buf_addr += ctx->scratch_buf_size;
	buf_size -= ctx->scratch_buf_size;

	/* start address of per buffer is aligned */
	for (i = 0; i < ctx->dpb_count; i++) {
		MFC_WRITEL(buf_addr, MFC_REG_E_LUMA_DPB + (4 * i));
		buf_addr += enc->luma_dpb_size;
		buf_size -= enc->luma_dpb_size;
	}
	for (i = 0; i < ctx->dpb_count; i++) {
		MFC_WRITEL(buf_addr, MFC_REG_E_CHROMA_DPB + (4 * i));
		buf_addr += enc->chroma_dpb_size;
		buf_size -= enc->chroma_dpb_size;
	}
	for (i = 0; i < ctx->dpb_count; i++) {
		MFC_WRITEL(buf_addr, MFC_REG_E_ME_BUFFER + (4 * i));
		buf_addr += enc->me_buffer_size;
		buf_size -= enc->me_buffer_size;
	}

	MFC_WRITEL(buf_addr, MFC_REG_E_TMV_BUFFER0);
	buf_addr += enc->tmv_buffer_size >> 1;
	MFC_WRITEL(buf_addr, MFC_REG_E_TMV_BUFFER1);
	buf_addr += enc->tmv_buffer_size >> 1;
	buf_size -= enc->tmv_buffer_size;

	mfc_debug(2, "[MEMINFO] codec buf 0x%llx, remained size: %d\n", buf_addr, buf_size);
	if (buf_size < 0) {
		mfc_debug(2, "[MEMINFO] Not enough memory has been allocated\n");
		return -ENOMEM;
	}

	mfc_debug_leave();

	return 0;
}

/* Set registers for decoding stream buffer */
int mfc_set_dec_stream_buffer(struct mfc_ctx *ctx, struct mfc_buf *mfc_buf,
		  unsigned int start_num_byte, unsigned int strm_size)
{
	struct mfc_dev *dev;
	struct mfc_dec *dec;
	unsigned int cpb_buf_size;
	dma_addr_t addr;
	int index = -1;

	mfc_debug_enter();
	if (!ctx) {
		mfc_err_dev("no mfc context to run\n");
		return -EINVAL;
	}
	dev = ctx->dev;
	if (!dev) {
		mfc_err_dev("no mfc device to run\n");
		return -EINVAL;
	}
	dec = ctx->dec_priv;
	if (!dec) {
		mfc_err_dev("no mfc decoder to run\n");
		return -EINVAL;
	}

	cpb_buf_size = ALIGN(dec->src_buf_size, STREAM_BUF_ALIGN);

	if (mfc_buf) {
		index = mfc_buf->vb.vb2_buf.index;
		addr = mfc_buf->addr[0][0];
		if (strm_size > set_strm_size_max(cpb_buf_size)) {
			mfc_info_ctx("Decrease strm_size because of %d align: %u -> %u\n",
				STREAM_BUF_ALIGN, strm_size, set_strm_size_max(cpb_buf_size));
			strm_size = set_strm_size_max(cpb_buf_size);
			mfc_buf->vb.vb2_buf.planes[0].bytesused = strm_size;
		}
	} else {
		addr = 0;
	}

	mfc_debug(2, "[BUFINFO] ctx[%d] set src index: %d, addr: 0x%08llx\n",
			ctx->num, index, addr);
	mfc_debug(2, "[STREAM] strm_size: %#lx(%d), buf_size: %u, offset: %u\n",
			strm_size, strm_size, cpb_buf_size, start_num_byte);

	if (strm_size == 0)
		mfc_info_ctx("stream size is 0\n");

	MFC_WRITEL(strm_size, MFC_REG_D_STREAM_DATA_SIZE);
	MFC_WRITEL(addr, MFC_REG_D_CPB_BUFFER_ADDR);
	MFC_WRITEL(cpb_buf_size, MFC_REG_D_CPB_BUFFER_SIZE);
	MFC_WRITEL(start_num_byte, MFC_REG_D_CPB_BUFFER_OFFSET);

	if (mfc_buf)
		MFC_TRACE_CTX("Set src[%d] fd: %d, %#llx\n",
				index, mfc_buf->vb.vb2_buf.planes[0].m.fd, addr);

	mfc_debug_leave();
	return 0;
}

void mfc_set_enc_frame_buffer(struct mfc_ctx *ctx,
		struct mfc_buf *mfc_buf, int num_planes)
{
	struct mfc_dev *dev = ctx->dev;
	dma_addr_t addr[3] = { 0, 0, 0 };
	dma_addr_t addr_2bit[2] = { 0, 0 };
	int index, i;

	if (!mfc_buf) {
		mfc_debug(3, "enc zero buffer set\n");
		goto buffer_set;
	}

	index = mfc_buf->vb.vb2_buf.index;
	if (mfc_buf->num_valid_bufs > 0) {
		for (i = 0; i < num_planes; i++) {
			addr[i] = mfc_buf->addr[mfc_buf->next_index][i];
			mfc_debug(2, "[BUFCON][BUFINFO] ctx[%d] set src index:%d, batch[%d], addr[%d]: 0x%08llx\n",
					ctx->num, index, mfc_buf->next_index, i, addr[i]);
		}
		mfc_buf->next_index++;
	} else {
		for (i = 0; i < num_planes; i++) {
			addr[i] = mfc_buf->addr[0][i];
			mfc_debug(2, "[BUFINFO] ctx[%d] set src index:%d, addr[%d]: 0x%08llx\n",
					ctx->num, index, i, addr[i]);
		}
	}

buffer_set:
	for (i = 0; i < num_planes; i++)
		MFC_WRITEL(addr[i], MFC_REG_E_SOURCE_FIRST_ADDR + (i * 4));

	if (ctx->src_fmt->fourcc == V4L2_PIX_FMT_NV12M_S10B ||
		ctx->src_fmt->fourcc == V4L2_PIX_FMT_NV21M_S10B) {
		addr_2bit[0] = addr[0] + NV12N_10B_Y_8B_SIZE(ctx->img_width, ctx->img_height);
		addr_2bit[1] = addr[1] + NV12N_10B_CBCR_8B_SIZE(ctx->img_width, ctx->img_height);

		for (i = 0; i < num_planes; i++) {
			MFC_WRITEL(addr_2bit[i], MFC_REG_E_SOURCE_FIRST_2BIT_ADDR + (i * 4));
			mfc_debug(2, "[BUFINFO][10BIT] ctx[%d] set src 2bit addr[%d]: 0x%08llx\n",
					ctx->num, i, addr_2bit[i]);
		}
	} else if (ctx->src_fmt->fourcc == V4L2_PIX_FMT_NV16M_S10B ||
		ctx->src_fmt->fourcc == V4L2_PIX_FMT_NV61M_S10B) {
		addr_2bit[0] = addr[0] + NV16M_Y_SIZE(ctx->img_width, ctx->img_height);
		addr_2bit[1] = addr[1] + NV16M_CBCR_SIZE(ctx->img_width, ctx->img_height);

		for (i = 0; i < num_planes; i++) {
			MFC_WRITEL(addr_2bit[i], MFC_REG_E_SOURCE_FIRST_2BIT_ADDR + (i * 4));
			mfc_debug(2, "[BUFINFO][10BIT] ctx[%d] set src 2bit addr[%d]: 0x%08llx\n",
					ctx->num, i, addr_2bit[i]);
		}
	}
}

/* Set registers for encoding stream buffer */
int mfc_set_enc_stream_buffer(struct mfc_ctx *ctx,
		struct mfc_buf *mfc_buf)
{
	struct mfc_dev *dev = ctx->dev;
	dma_addr_t addr;
	unsigned int size, offset, index;

	index = mfc_buf->vb.vb2_buf.index;
	addr = mfc_buf->addr[0][0];
	offset = mfc_buf->vb.vb2_buf.planes[0].data_offset;
	size = (unsigned int)vb2_plane_size(&mfc_buf->vb.vb2_buf, 0);
	size = ALIGN(size, 512);

	MFC_WRITEL(addr, MFC_REG_E_STREAM_BUFFER_ADDR); /* 16B align */
	MFC_WRITEL(size, MFC_REG_E_STREAM_BUFFER_SIZE);
	MFC_WRITEL(offset, MFC_REG_E_STREAM_BUFFER_OFFSET);

	mfc_debug(2, "[BUFINFO] ctx[%d] set dst index: %d, addr: 0x%08llx\n",
			ctx->num, index, addr);
	mfc_debug(2, "[STREAM] buf_size: %u, offset: %d\n", size, offset);

	return 0;
}

void mfc_get_enc_frame_buffer(struct mfc_ctx *ctx,
		dma_addr_t addr[], int num_planes)
{
	struct mfc_dev *dev = ctx->dev;
	unsigned long enc_recon_y_addr, enc_recon_c_addr;
	int i, addr_offset;

	addr_offset = MFC_REG_E_ENCODED_SOURCE_FIRST_ADDR;

	for (i = 0; i < num_planes; i++)
		addr[i] = MFC_READL(addr_offset + (i * 4));

	enc_recon_y_addr = MFC_READL(MFC_REG_E_RECON_LUMA_DPB_ADDR);
	enc_recon_c_addr = MFC_READL(MFC_REG_E_RECON_CHROMA_DPB_ADDR);

	mfc_debug(2, "[MEMINFO] recon y: 0x%08lx c: 0x%08lx\n",
			enc_recon_y_addr, enc_recon_c_addr);
}

void mfc_set_enc_stride(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	int i;

	for (i = 0; i < ctx->raw_buf.num_planes; i++) {
		MFC_WRITEL(ctx->raw_buf.stride[i],
				MFC_REG_E_SOURCE_FIRST_STRIDE + (i * 4));
		mfc_debug(2, "[FRAME] enc src plane[%d] stride: %d\n",
				i, ctx->raw_buf.stride[i]);
	}
}

int mfc_set_dynamic_dpb(struct mfc_ctx *ctx, struct mfc_buf *dst_mb)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_raw_info *raw = &ctx->raw_buf;
	int dst_index;
	int i;

	dst_index = dst_mb->vb.vb2_buf.index;
	set_bit(dst_index, &dec->available_dpb);
	dec->dynamic_set = 1 << dst_index;
	mfc_debug(2, "[DPB] ADDING Flag after: 0x%lx\n", dec->available_dpb);

	/* for debugging about black bar detection */
	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->black_bar) && dec->detect_black_bar) {
		for (i = 0; i < raw->num_planes; i++) {
			dec->frame_vaddr[i][dec->frame_cnt] = vb2_plane_vaddr(&dst_mb->vb.vb2_buf, i);
			dec->frame_daddr[i][dec->frame_cnt] = dst_mb->addr[0][i];
			dec->frame_size[i][dec->frame_cnt] = raw->plane_size[i];
			dec->index[i][dec->frame_cnt] = dst_index;
			dec->fd[i][dec->frame_cnt] = dst_mb->vb.vb2_buf.planes[0].m.fd;
		}
		dec->frame_cnt++;
		if (dec->frame_cnt >= 30)
			dec->frame_cnt = 0;
	}

	/* decoder dst buffer CFW PROT */
	mfc_protect_dpb(ctx, dst_mb);

	for (i = 0; i < raw->num_planes; i++) {
		MFC_WRITEL(raw->plane_size[i],
				MFC_REG_D_FIRST_PLANE_DPB_SIZE + i * 4);
		MFC_WRITEL(dst_mb->addr[0][i],
				MFC_REG_D_FIRST_PLANE_DPB0 + (i * 0x100 + dst_index * 4));
		if (ctx->is_10bit)
			MFC_WRITEL(raw->plane_size_2bits[i],
					MFC_REG_D_FIRST_PLANE_2BIT_DPB_SIZE + (i * 4));
		mfc_debug(2, "[BUFINFO][DPB] ctx[%d] set dst index: %d, addr[%d]: 0x%08llx\n",
				ctx->num, dst_index, i, dst_mb->addr[0][i]);
	}

	MFC_TRACE_CTX("Set dst[%d] fd: %d, %#llx / avail %#lx used %#x\n",
			dst_index, dst_mb->vb.vb2_buf.planes[0].m.fd, dst_mb->addr[0][0],
			dec->available_dpb, dec->dynamic_used);

	return 0;
}

void mfc_set_pixel_format(struct mfc_dev *dev, unsigned int format)
{
	unsigned int reg = 0;
	unsigned int pix_val, mem_type_10bit = 0;

	if (dev->pdata->P010_decoding)
		mem_type_10bit = 1;

	switch (format) {
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV12N:
	case V4L2_PIX_FMT_NV12MT_16X16:
	case V4L2_PIX_FMT_NV16M:
		pix_val = 0;
		break;
	case V4L2_PIX_FMT_NV21M:
	case V4L2_PIX_FMT_NV61M:
		pix_val = 1;
		break;
	case V4L2_PIX_FMT_YVU420M:
		pix_val = 2;
		break;
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YUV420N:
		pix_val = 3;
		break;
	/* For 10bit direct set */
	case V4L2_PIX_FMT_NV12N_10B:
	case V4L2_PIX_FMT_NV12M_S10B:
	case V4L2_PIX_FMT_NV16M_S10B:
		mem_type_10bit = 0;
		pix_val = 0;
		break;
	case V4L2_PIX_FMT_NV12M_P010:
	case V4L2_PIX_FMT_NV16M_P210:
		mem_type_10bit = 1;
		pix_val = 0;
		break;
	case V4L2_PIX_FMT_NV21M_S10B:
	case V4L2_PIX_FMT_NV61M_S10B:
		mem_type_10bit = 0;
		pix_val = 1;
		break;
	case V4L2_PIX_FMT_NV21M_P010:
	case V4L2_PIX_FMT_NV61M_P210:
		mem_type_10bit = 1;
		pix_val = 1;
		break;
	default:
		pix_val = 0;
		break;
	}
	reg |= pix_val;
	reg |= (mem_type_10bit << 4);
	MFC_WRITEL(reg, MFC_REG_PIXEL_FORMAT);
	mfc_debug(2, "[FRAME] pixel format: %d, mem_type_10bit for 10bit: %d (reg: %#x)\n",
			pix_val, mem_type_10bit, reg);
}
