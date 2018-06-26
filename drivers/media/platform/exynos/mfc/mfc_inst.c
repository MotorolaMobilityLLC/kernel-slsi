/*
 * drivers/media/platform/exynos/mfc/mfc_inst.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "mfc_inst.h"

#include "mfc_cmd.h"
#include "mfc_enc_param.h"
#include "mfc_perf_measure.h"
#include "mfc_reg_api.h"
#include "mfc_hw_reg_api.h"

#include "mfc_utils.h"

int mfc_open_inst(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	unsigned int reg;
	int ret;

	/* Preparing decoding - getting instance number */
	mfc_debug(2, "Getting instance number\n");
	mfc_clean_ctx_int_flags(ctx);

	reg = MFC_READL(MFC_REG_CODEC_CONTROL);
	/* Clear OTF_CONTROL[2:1] & OTF_DEBUG[3] */
	reg &= ~(0x7 << 1);
	if (ctx->otf_handle) {
		/* Set OTF_CONTROL[2:1], 0: Non-OTF, 1: OTF+HWFC, 2: OTF only */
		reg |= (0x1 << 1);
		mfc_info_ctx("HWFC + OTF enabled\n");
		if (otf_dump && !ctx->is_drm) {
			/* Set OTF_DEBUG[3] for OTF path dump */
			reg |= (0x1 << 3);
			mfc_info_ctx("Debugging mode enabled\n");
		}
	}
	MFC_WRITEL(reg, MFC_REG_CODEC_CONTROL);


	ret = mfc_cmd_open_inst(ctx);
	if (ret) {
		mfc_err_ctx("Failed to create a new instance\n");
		mfc_change_state(ctx, MFCINST_ERROR);
	}

	return ret;
}

int mfc_close_inst(struct mfc_ctx *ctx)
{
	int ret = -EINVAL;

	/* Closing decoding instance  */
	mfc_debug(2, "Returning instance number\n");
	mfc_clean_ctx_int_flags(ctx);
	if (ctx->state == MFCINST_FREE) {
		mfc_err_ctx("ctx already free status\n");
		return ret;
	}

	ret = mfc_cmd_close_inst(ctx);
	if (ret) {
		mfc_err_ctx("Failed to return an instance\n");
		mfc_change_state(ctx, MFCINST_ERROR);
	}

	return ret;
}

int mfc_abort_inst(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev;

	if (!ctx) {
		mfc_err_dev("no mfc context to run\n");
		return -EINVAL;
	}

	dev = ctx->dev;
	if (!dev) {
		mfc_err_dev("no mfc device to run\n");
		return -EINVAL;
	}

	mfc_clean_ctx_int_flags(ctx);

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);
	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_NAL_ABORT);

	return 0;
}

/* Initialize decoding */
int mfc_init_decode(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev;
	struct mfc_dec *dec;
	unsigned int reg = 0;
	int fmo_aso_ctrl = 0;

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
	mfc_debug(2, "InstNo: %d/%d\n", ctx->inst_no, MFC_REG_H2R_CMD_SEQ_HEADER);
	mfc_debug(2, "BUFs: %08x\n", MFC_READL(MFC_REG_D_CPB_BUFFER_ADDR));

	/* When user sets desplay_delay to 0,
	 * It works as "display_delay enable" and delay set to 0.
	 * If user wants display_delay disable, It should be
	 * set to negative value. */
	if (dec->display_delay >= 0) {
		reg |= (0x1 << MFC_REG_D_DEC_OPT_DISPLAY_DELAY_EN_SHIFT);
		MFC_WRITEL(dec->display_delay, MFC_REG_D_DISPLAY_DELAY);
	}

	/* FMO_ASO_CTRL - 0: Enable, 1: Disable */
	reg |= ((fmo_aso_ctrl & MFC_REG_D_DEC_OPT_FMO_ASO_CTRL_MASK)
			<< MFC_REG_D_DEC_OPT_FMO_ASO_CTRL_SHIFT);

	reg |= ((dec->idr_decoding & MFC_REG_D_DEC_OPT_IDR_DECODING_MASK)
			<< MFC_REG_D_DEC_OPT_IDR_DECODING_SHIFT);

	/* VC1 RCV: Discard to parse additional header as default */
	if (IS_VC1_RCV_DEC(ctx))
		reg |= (0x1 << MFC_REG_D_DEC_OPT_DISCARD_RCV_HEADER_SHIFT);

	/* conceal control to specific color */
	reg |= (0x4 << MFC_REG_D_DEC_OPT_CONCEAL_CONTROL_SHIFT);

	/* Disable parallel processing if nal_q_parallel_disable was set */
	if (nal_q_parallel_disable)
		reg |= (0x2 << MFC_REG_D_DEC_OPT_PARALLEL_DISABLE_SHIFT);

	/* Realloc buffer for resolution decrease case in NAL QUEUE mode */
	reg |= (0x1 << MFC_REG_D_DEC_OPT_REALLOC_CONTROL_SHIFT);

	/* Parsing all including PPS */
	reg |= (0x1 << MFC_REG_D_DEC_OPT_SPECIAL_PARSING_SHIFT);

	MFC_WRITEL(reg, MFC_REG_D_DEC_OPTIONS);

	MFC_WRITEL(MFC_CONCEAL_COLOR, MFC_REG_D_FORCE_PIXEL_VAL);

	if (IS_FIMV1_DEC(ctx)) {
		mfc_debug(2, "Setting FIMV1 resolution to %dx%d\n",
					ctx->img_width, ctx->img_height);
		MFC_WRITEL(ctx->img_width, MFC_REG_D_SET_FRAME_WIDTH);
		MFC_WRITEL(ctx->img_height, MFC_REG_D_SET_FRAME_HEIGHT);
	}

	mfc_set_pixel_format(dev, ctx->dst_fmt->fourcc);

	reg = 0;
	/* Enable realloc interface if SEI is enabled */
	if (dec->sei_parse)
		reg |= (0x1 << MFC_REG_D_SEI_ENABLE_NEED_INIT_BUFFER_SHIFT);
	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->static_info_dec)) {
		reg |= (0x1 << MFC_REG_D_SEI_ENABLE_CONTENT_LIGHT_SHIFT);
		reg |= (0x1 << MFC_REG_D_SEI_ENABLE_MASTERING_DISPLAY_SHIFT);
	}
	reg |= (0x1 << MFC_REG_D_SEI_ENABLE_RECOVERY_PARSING_SHIFT);

	MFC_WRITEL(reg, MFC_REG_D_SEI_ENABLE);
	mfc_debug(2, "SEI enable was set, 0x%x\n", MFC_READL(MFC_REG_D_SEI_ENABLE));

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);

	if (sfr_dump & MFC_DUMP_DEC_SEQ_START)
		call_dop(dev, dump_regs, dev);

	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_SEQ_HEADER);

	mfc_debug_leave();
	return 0;
}

/* Decode a single frame */
int mfc_decode_one_frame(struct mfc_ctx *ctx, int last_frame)
{
	struct mfc_dev *dev;
	struct mfc_dec *dec;
	u32 reg = 0;

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

	mfc_debug(2, "Dynamic:0x%08x, Available:0x%lx\n",
			dec->dynamic_set, dec->available_dpb);

	reg = MFC_READL(MFC_REG_D_NAL_START_OPTIONS);
	reg &= ~(0x1 << MFC_REG_D_NAL_START_OPT_BLACK_BAR_SHIFT);
	reg |= ((dec->detect_black_bar & 0x1) << MFC_REG_D_NAL_START_OPT_BLACK_BAR_SHIFT);
	MFC_WRITEL(reg, MFC_REG_D_NAL_START_OPTIONS);
	mfc_debug(3, "[BLACKBAR] black bar detect set: %#x\n", reg);

	MFC_WRITEL(dec->dynamic_set, MFC_REG_D_DYNAMIC_DPB_FLAG_LOWER);
	MFC_WRITEL(0x0, MFC_REG_D_DYNAMIC_DPB_FLAG_UPPER);
	MFC_WRITEL(dec->available_dpb, MFC_REG_D_AVAILABLE_DPB_FLAG_LOWER);
	MFC_WRITEL(0x0, MFC_REG_D_AVAILABLE_DPB_FLAG_UPPER);
	MFC_WRITEL(dec->slice_enable, MFC_REG_D_SLICE_IF_ENABLE);
	MFC_WRITEL(MFC_TIMEOUT_VALUE, MFC_REG_DEC_TIMEOUT_VALUE);

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);

	if ((sfr_dump & MFC_DUMP_DEC_NAL_START) && !ctx->check_dump) {
		call_dop(dev, dump_regs, dev);
		ctx->check_dump = 1;
	}

	/* Issue different commands to instance basing on whether it
	 * is the last frame or not. */
	switch (last_frame) {
	case 0:
		mfc_perf_measure_on(dev);

		mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_NAL_START);
		break;
	case 1:
		mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_LAST_FRAME);
		break;
	}

	mfc_debug(2, "Decoding a usual frame\n");
	return 0;
}

int mfc_init_encode(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;

	mfc_debug(2, "++\n");

	if (IS_H264_ENC(ctx))
		mfc_set_enc_params_h264(ctx);
	else if (IS_MPEG4_ENC(ctx))
		mfc_set_enc_params_mpeg4(ctx);
	else if (IS_H263_ENC(ctx))
		mfc_set_enc_params_h263(ctx);
	else if (IS_VP8_ENC(ctx))
		mfc_set_enc_params_vp8(ctx);
	else if (IS_VP9_ENC(ctx))
		mfc_set_enc_params_vp9(ctx);
	else if (IS_HEVC_ENC(ctx))
		mfc_set_enc_params_hevc(ctx);
	else if (IS_BPG_ENC(ctx))
		mfc_set_enc_params_bpg(ctx);
	else {
		mfc_err_ctx("Unknown codec for encoding (%x)\n",
			ctx->codec_mode);
		return -EINVAL;
	}

	mfc_debug(5, "RC) Bitrate: %d / framerate: %#x / config %#x / mode %#x\n",
			MFC_READL(MFC_REG_E_RC_BIT_RATE),
			MFC_READL(MFC_REG_E_RC_FRAME_RATE),
			MFC_READL(MFC_REG_E_RC_CONFIG),
			MFC_READL(MFC_REG_E_RC_MODE));

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);

	if (sfr_dump & MFC_DUMP_ENC_SEQ_START)
		call_dop(dev, dump_regs, dev);

	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_SEQ_HEADER);

	mfc_debug(2, "--\n");

	return 0;
}

static int mfc_h264_set_aso_slice_order(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_enc *enc = ctx->enc_priv;
	struct mfc_enc_params *p = &enc->params;
	struct mfc_h264_enc_params *p_264 = &p->codec.h264;
	int i;

	if (p_264->aso_enable) {
		for (i = 0; i < 8; i++)
			MFC_WRITEL(p_264->aso_slice_order[i],
				MFC_REG_E_H264_ASO_SLICE_ORDER_0 + i * 4);
	}
	return 0;
}

/* Encode a single frame */
int mfc_encode_one_frame(struct mfc_ctx *ctx, int last_frame)
{
	struct mfc_dev *dev = ctx->dev;

	mfc_debug(2, "++\n");

	if (IS_H264_ENC(ctx))
		mfc_h264_set_aso_slice_order(ctx);

	mfc_set_slice_mode(ctx);

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);

	if ((sfr_dump & MFC_DUMP_ENC_NAL_START) && !ctx->check_dump) {
		call_dop(dev, dump_regs, dev);
		ctx->check_dump = 1;
	}

	/* Issue different commands to instance basing on whether it
	 * is the last frame or not. */
	switch (last_frame) {
	case 0:
		mfc_perf_measure_on(dev);

		mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_NAL_START);
		break;
	case 1:
		mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_LAST_FRAME);
		break;
	}

	mfc_debug(2, "--\n");

	return 0;
}
