/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "../decon.h"
#include "../dqe.h"
#include "regs-dqe.h"

/* DQE_CON register set */
void dqe_reg_set_gamma_on(u32 on)
{
	dqe_write_mask(DQECON, ~0, DQE_GAMMA_ON_MASK);
}

u32 dqe_reg_get_gamma_on(void)
{
	return dqe_read_mask(DQECON, DQE_GAMMA_ON_MASK);
}

void dqe_reg_set_cgc_on(u32 on)
{
	dqe_write_mask(DQECON, ~0, DQE_CGC_ON_MASK);
}

u32 dqe_reg_get_cgc_on(void)
{
	return dqe_read_mask(DQECON, DQE_CGC_ON_MASK);
}

void dqe_reg_set_hsc_on(u32 on)
{
	dqe_write_mask(DQECON, ~0, DQE_HSC_ON_MASK);
}

u32 dqe_reg_get_hsc_on(void)
{
	return dqe_read_mask(DQECON, DQE_HSC_ON_MASK);
}

void dqe_reg_hsc_sw_reset(u32 id)
{
	u32 cnt = 5000; /* 3 frame */
	u32 state;

	dqe_write_mask(DQECON, ~0, DQE_HSC_SW_RESET_MASK);
	decon_reg_update_req_dqe(id);

	do {
		state = dqe_read_mask(DQECON, DQE_HSC_SW_RESET_MASK);
		cnt--;
		udelay(10);
	} while (state && cnt);

	if (!cnt)
		dqe_err("%s is timeout.\n", __func__);

	dqe_dbg("dqe hsc_sw_reset:%d cnt:%d\n",
		DQE_HSC_SW_RESET_GET(dqe_read(DQECON)), cnt);
}

void dqe_reg_set_hsc_pphc_on(u32 on)
{
	dqe_write_mask(DQEHSC_CONTROL, ~0, HSC_PPHC_ON_MASK);
}

void dqe_reg_set_hsc_ppsc_on(u32 on)
{
	dqe_write_mask(DQEHSC_CONTROL, ~0, HSC_PPSC_ON_MASK);
}

void dqe_reg_set_hsc_control(u32 val)
{
	dqe_write_mask(DQEHSC_CONTROL, val, HSC_ALL_MASK);
}

void dqe_reg_set_hsc_control_all_reset(void)
{
	dqe_write_mask(DQEHSC_CONTROL, 0, HSC_ALL_MASK);
}

u32 dqe_reg_get_hsc_control(void)
{
	return dqe_read_mask(DQEHSC_CONTROL, HSC_ALL_MASK);
}

void dqe_reg_set_hsc_full_pxl_num(struct decon_lcd *lcd_info)
{
	u32 val, mask;

	val = (u32)(lcd_info->xres * lcd_info->yres);
	mask = DQEHSC_FULL_PXL_NUM_MASK;
	dqe_write_mask(DQEHSC_FULL_PXL_NUM, val, mask);
}

u32 dqe_reg_get_hsc_full_pxl_num(void)
{
	return dqe_read_mask(DQEHSC_FULL_PXL_NUM, DQEHSC_FULL_PXL_NUM_MASK);
}

void dqe_reg_set_img_size(u32 id, struct decon_lcd *lcd_info)
{
	u32 width, val, mask;

	if (id != 0)
		return;

	width = lcd_info->xres;

	val = DQEIMG_VSIZE_F(lcd_info->yres) | DQEIMG_HSIZE_F(width);
	mask = DQEIMG_VSIZE_MASK | DQEIMG_HSIZE_MASK;
	dqe_write_mask(DQEIMG_SIZESET, val, mask);
}

void dqe_reg_set_data_path_enable(u32 id, u32 en)
{
	u32 val;
	enum decon_data_path d_path = DPATH_DSCENC0_OUTFIFO0_DSIMIF0;
	enum decon_scaler_path s_path = SCALERPATH_OFF;
	enum decon_enhance_path e_path = ENHANCEPATH_ENHANCE_ALL_OFF;

	if (id != 0)
		return;

	val = en ? ENHANCEPATH_DQE_ON : ENHANCEPATH_ENHANCE_ALL_OFF;
	decon_reg_get_data_path(id, &d_path, &s_path, &e_path);
	decon_reg_set_data_path(id, d_path, s_path, val);
}

void dqe_reg_start(u32 id, struct decon_lcd *lcd_info)
{
	dqe_reg_set_data_path_enable(id, 1);
	dqe_reg_set_img_size(id, lcd_info);
}

void dqe_reg_stop(u32 id)
{
	dqe_reg_set_data_path_enable(id, 0);
}

