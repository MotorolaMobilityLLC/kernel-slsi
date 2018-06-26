/*
 * drivers/media/platform/exynos/mfc/mfc_inst.h
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_INST_H
#define __MFC_INST_H __FILE__

#include "mfc_common.h"

int mfc_open_inst(struct mfc_ctx *ctx);
int mfc_close_inst(struct mfc_ctx *ctx);
int mfc_abort_inst(struct mfc_ctx *ctx);

int mfc_init_decode(struct mfc_ctx *ctx);
int mfc_decode_one_frame(struct mfc_ctx *ctx, int last_frame);

int mfc_init_encode(struct mfc_ctx *ctx);
int mfc_encode_one_frame(struct mfc_ctx *ctx, int last_frame);

#endif /* __MFC_INST_H  */
