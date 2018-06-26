/*
 * drivers/media/platform/exynos/mfc/mfc_enc_param.h
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_ENC_PARAM_H
#define __MFC_ENC_PARAM_H __FILE__

#include "mfc_common.h"

void mfc_set_slice_mode(struct mfc_ctx *ctx);
void mfc_set_enc_params_h264(struct mfc_ctx *ctx);
void mfc_set_enc_params_mpeg4(struct mfc_ctx *ctx);
void mfc_set_enc_params_h263(struct mfc_ctx *ctx);
void mfc_set_enc_params_vp8(struct mfc_ctx *ctx);
void mfc_set_enc_params_vp9(struct mfc_ctx *ctx);
void mfc_set_enc_params_hevc(struct mfc_ctx *ctx);
void mfc_set_enc_params_bpg(struct mfc_ctx *ctx);

#endif /* __MFC_ENC_PARAM_H */
