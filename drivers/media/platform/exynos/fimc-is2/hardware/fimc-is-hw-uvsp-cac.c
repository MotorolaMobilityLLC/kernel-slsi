/*
 * Samsung EXYNOS FIMC-IS (Imaging Subsystem) driver
 *
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "fimc-is-hw-mcscaler-v2.h"
#include "api/fimc-is-hw-api-mcscaler-v2.h"
#include "../interface/fimc-is-interface-ischain.h"
#include "fimc-is-param.h"
#include "fimc-is-err.h"

/* NI from DDK needs to adjust scale factor (by multipling 10) */
#define MULTIPLIED_10(value)		(10 * (value))
#define INTRPL_SHFT_VAL			(12)
#define SUB(a, b)		(u32)(((a) - (b)) > 0 ? ((a) - (b)) : ((b) - (a)))
#define LSHFT(a)		((int)((a) << INTRPL_SHFT_VAL))
#define RSHFT(a)		((int)((a) >> INTRPL_SHFT_VAL))
#define NUMERATOR(Y1, Y2, DXn)			(((Y2) - (Y1)) * (DXn))
#define CALC_LNR_INTRPL(Y1, Y2, X1, X2, X)	(LSHFT(NUMERATOR(Y1, Y2, SUB(X, X1))) / SUB(X2, X1) + LSHFT(Y1))
#define GET_LNR_INTRPL(Y1, Y2, X1, X2, X)	RSHFT(CALC_LNR_INTRPL(Y1, Y2, X1, X2, X))

struct ref_ni {
	u32 min;
	u32 max;
};

struct ref_ni hw_mcsc_find_ni_idx_for_cac(struct fimc_is_hw_ip *hw_ip,
	struct cac_setfile_contents *cac, u32 cur_ni)
{
	struct ref_ni ret_idx = {0, 0};
	struct ref_ni ni_idx_range;
	u32 ni_idx;

	for (ni_idx = 0; ni_idx < cac->ni_max - 1; ni_idx++) {
		ni_idx_range.min = MULTIPLIED_10(cac->ni_vals[ni_idx]);
		ni_idx_range.max = MULTIPLIED_10(cac->ni_vals[ni_idx + 1]);

		if (ni_idx_range.min < cur_ni && cur_ni < ni_idx_range.max) {
			ret_idx.min = ni_idx;
			ret_idx.max = ni_idx + 1;
			break;
		} else if (cur_ni == ni_idx_range.min) {
			ret_idx.min = ni_idx;
			ret_idx.max = ni_idx;
			break;
		} else if (cur_ni == ni_idx_range.max) {
			ret_idx.min = ni_idx + 1;
			ret_idx.max = ni_idx + 1;
			break;
		}
	}

	sdbg_hw(10, "TEST: get_lnr_intprl(11, 20, 1, 10, 3) = %d\n",
			hw_ip, GET_LNR_INTRPL(11, 20, 1, 10, 3));

	sdbg_hw(2, "[CAC] find_ni_idx: cur_ni(%d), ni[%d, %d], idx[%d, %d]\n", hw_ip,
		cac->ni_vals[ret_idx.min], cac->ni_vals[ret_idx.max],
		ret_idx.min, ret_idx.max);

	return ret_idx;
}

void hw_mcsc_calc_cac_map_thr(struct cac_cfg_by_ni *cac_cfg,
	struct cac_cfg_by_ni *cfg_min, struct cac_cfg_by_ni *cfg_max,
	struct ref_ni *ni_idx, u32 cur_ni)
{
	struct cac_map_thr_cfg *min, *max;

	min = &cfg_min->map_thr_cfg;
	max = &cfg_max->map_thr_cfg;

	cac_cfg->map_thr_cfg.map_spot_thr_l = GET_LNR_INTRPL(
			min->map_spot_thr_l, max->map_spot_thr_l,
			ni_idx->min, ni_idx->max, cur_ni);
	cac_cfg->map_thr_cfg.map_spot_thr_h = GET_LNR_INTRPL(
			min->map_spot_thr_h, max->map_spot_thr_h,
			ni_idx->min, ni_idx->max, cur_ni);
	cac_cfg->map_thr_cfg.map_spot_thr = GET_LNR_INTRPL(
			min->map_spot_thr, max->map_spot_thr,
			ni_idx->min, ni_idx->max, cur_ni);
	cac_cfg->map_thr_cfg.map_spot_nr_strength = GET_LNR_INTRPL(
			min->map_spot_nr_strength, max->map_spot_nr_strength,
			ni_idx->min, ni_idx->max, cur_ni);

}

void hw_mcsc_calc_cac_crt_thr(struct cac_cfg_by_ni *cac_cfg,
	struct cac_cfg_by_ni *cfg_min, struct cac_cfg_by_ni *cfg_max,
	struct ref_ni *ni_idx, u32 cur_ni)
{
	struct cac_crt_thr_cfg *min, *max;

	min = &cfg_min->crt_thr_cfg;
	max = &cfg_max->crt_thr_cfg;

	cac_cfg->crt_thr_cfg.crt_color_thr_l_dot = GET_LNR_INTRPL(
			min->crt_color_thr_l_dot, max->crt_color_thr_l_dot,
			ni_idx->min, ni_idx->max, cur_ni);
	cac_cfg->crt_thr_cfg.crt_color_thr_l_line = GET_LNR_INTRPL(
			min->crt_color_thr_l_line, max->crt_color_thr_l_line,
			ni_idx->min, ni_idx->max, cur_ni);
	cac_cfg->crt_thr_cfg.crt_color_thr_h = GET_LNR_INTRPL(
			min->crt_color_thr_h, max->crt_color_thr_h,
			ni_idx->min, ni_idx->max, cur_ni);
}

void hw_mcsc_calc_cac_param_by_ni(struct fimc_is_hw_ip *hw_ip,
	struct cac_setfile_contents *cac, u32 cur_ni)
{
	struct ref_ni ni_range, ni_idx;
	struct cac_cfg_by_ni cac_cfg, cfg_max, cfg_min;

	ni_range.min = MULTIPLIED_10(cac->ni_vals[0]);
	ni_range.max = MULTIPLIED_10(cac->ni_vals[cac->ni_max - 1]);

	if (cur_ni <= ni_range.min) {
		sdbg_hw(2, "[CAC] cur_ni(%d) <= ni_range.min(%d)\n", hw_ip, cur_ni, ni_range.min);
		ni_idx.min = 0;
		ni_idx.max = 0;
	} else if (cur_ni >= ni_range.max) {
		sdbg_hw(2, "[CAC] cur_ni(%d) >= ni_range.max(%d)\n", hw_ip, cur_ni, ni_range.max);
		ni_idx.min = cac->ni_max - 1;
		ni_idx.max = cac->ni_max - 1;
	} else {
		ni_idx = hw_mcsc_find_ni_idx_for_cac(hw_ip, cac, cur_ni);
	}

	cfg_min = cac->cfgs[ni_idx.min];
	cfg_max = cac->cfgs[ni_idx.max];
	hw_mcsc_calc_cac_map_thr(&cac_cfg, &cfg_min, &cfg_max, &ni_idx, cur_ni);
	hw_mcsc_calc_cac_crt_thr(&cac_cfg, &cfg_min, &cfg_max, &ni_idx, cur_ni);

	fimc_is_scaler_set_cac_map_crt_thr(hw_ip->regs, &cac_cfg);
	fimc_is_scaler_set_cac_enable(hw_ip->regs, 1);
}

int fimc_is_hw_mcsc_update_cac_register(struct fimc_is_hw_ip *hw_ip,
	struct fimc_is_frame *frame, u32 instance)
{
	int ret = 0;
	struct fimc_is_hw_mcsc *hw_mcsc;
	struct fimc_is_hw_mcsc_cap *cap;
	struct hw_mcsc_setfile *setfile;
	struct cac_setfile_contents *cac;
	enum exynos_sensor_position sensor_position;
	u32 ni, backup_in;

	BUG_ON(!hw_ip);
	BUG_ON(!hw_ip->priv_info);

	hw_mcsc = (struct fimc_is_hw_mcsc *)hw_ip->priv_info;
	cap = GET_MCSC_HW_CAP(hw_ip);

	if (cap->cac != MCSC_CAP_SUPPORT)
		return ret;

#ifdef LHM_ENABLE_EVT0
	return ret;
#endif

	backup_in = hw_mcsc->cac_in;
	if (hw_ip->hardware->video_mode)
		hw_mcsc->cac_in = DEV_HW_MCSC0;
	else
		hw_mcsc->cac_in = DEV_HW_MCSC1;

	if (backup_in != hw_mcsc->cac_in)
		sdbg_hw(0, "cac input_source changed %d-> %d\n", hw_ip,
			backup_in - DEV_HW_MCSC0, hw_mcsc->cac_in - DEV_HW_MCSC0);

	fimc_is_scaler_set_cac_input_source(hw_ip->regs, (hw_mcsc->cac_in - DEV_HW_MCSC0));

	if (hw_mcsc->cac_in != hw_ip->id)
		return ret;

	sensor_position = hw_ip->hardware->sensor_position[instance];
	setfile = hw_mcsc->cur_setfile[sensor_position];

	/* calculate cac parameters */
#ifdef FIXED_TDNR_NOISE_INDEX
	ni = FIXED_TDNR_NOISE_INDEX_VALUE;
#else
	ni = frame->noise_idx;
#endif
	if (hw_mcsc->cur_ni == ni)
		goto exit;

#if defined(USE_UVSP_CAC)
	cac = &setfile->cac;
	hw_mcsc_calc_cac_param_by_ni(hw_ip, cac, ni);
#endif

	sdbg_hw(2, "[CAC][F:%d]: ni(%d)\n",
		hw_ip, __func__, frame->fcount, ni);
exit:
	hw_mcsc->cur_ni = ni;

	return 0;
}

int fimc_is_hw_mcsc_recovery_cac_register(struct fimc_is_hw_ip *hw_ip,
		struct is_param_region *param, u32 instance)
{
	int ret = 0;
	/* TODO */

	return ret;
}
