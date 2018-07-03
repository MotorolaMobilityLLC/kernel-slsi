/*
 * Samsung EXYNOS FIMC-IS (Imaging Subsystem) driver
 *
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_HW_UVSP_CAC_H
#define FIMC_IS_HW_UVSP_CAC_H

#define CAC_MAX_NI_DEPENDED_CFG     (9)
struct cac_map_thr_cfg {
	u32	map_spot_thr_l;
	u32	map_spot_thr_h;
	u32	map_spot_thr;
	u32	map_spot_nr_strength;
};

struct cac_crt_thr_cfg {
	u32	crt_color_thr_l_dot;
	u32	crt_color_thr_l_line;
	u32	crt_color_thr_h;
};

struct cac_cfg_by_ni {
	struct cac_map_thr_cfg	map_thr_cfg;
	struct cac_crt_thr_cfg	crt_thr_cfg;
};

struct cac_setfile_contents {
	u32	ni_max;
	u32	ni_vals[CAC_MAX_NI_DEPENDED_CFG];
	struct cac_cfg_by_ni	cfgs[CAC_MAX_NI_DEPENDED_CFG];
};

#define UVSP_MAX_NI_DEPENDED_CONFIGS     (6)
struct uvsp_radial_cfg {
	u32	radial_biquad_shift;

	bool	radial_random_en;
	u32	radial_random_power;
	bool	radial_refine_en;
	u32	radial_refine_luma_min;
	u32	radial_refine_denom;

	bool	radial_alpha_gain_add_en;
	bool	radial_alpha_green_en;
	u32	radial_alpha_r;
	u32	radial_alpha_g;
	u32	radial_alpha_b;
};

struct uvsp_pedestal_cfg {
	u32	pedestal_r;
	u32	pedestal_g;
	u32	pedestal_b;
};

struct uvsp_offset_cfg {
	u32	ofs_r;
	u32	ofs_g;
	u32	ofs_b;
};

struct uvsp_desat_cfg {
	bool	desat_ctrl_en;
	bool	desat_ctrl_singleSide;
	u32	desat_ctrl_luma_offset;
	u32	desat_ctrl_gain_offset;

	u32	desat_y_shift;
	u32	desat_y_luma_max;
	u32	desat_u_low;
	u32	desat_u_high;
	u32	desat_v_low;
	u32	desat_v_high;
};

struct uvsp_rgb2yuv_coef_cfg {
	u32	r2y_coef_00;
	u32	r2y_coef_01;
	u32	r2y_coef_02;
	u32	r2y_coef_10;
	u32	r2y_coef_11;
	u32	r2y_coef_12;
	u32	r2y_coef_20;
	u32	r2y_coef_21;
	u32	r2y_coef_22;
	u32	r2y_coef_shift;
};

struct uvsp_cfg_by_ni {
	struct uvsp_radial_cfg		radial_cfg;
	struct uvsp_pedestal_cfg	pedestal_cfg;
	struct uvsp_offset_cfg		offset_cfg;
	struct uvsp_desat_cfg		desat_cfg;
	struct uvsp_rgb2yuv_coef_cfg	rgb2yuv_coef_cfg;
};

struct uvsp_setfile_contents {
	u32	ni_max;
	u32	ni_vals[UVSP_MAX_NI_DEPENDED_CONFIGS];
	struct uvsp_cfg_by_ni	cfgs[UVSP_MAX_NI_DEPENDED_CONFIGS];
};
#endif
