/*
 * Samsung EXYNOS FIMC-IS (Imaging Subsystem) driver
 *
 * Copyright (C) 2014 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_HW_MCSC_H
#define FIMC_IS_HW_MCSC_H

#include "fimc-is-hw-control.h"
#include "fimc-is-hw-tdnr.h"
#include "fimc-is-hw-djag.h"
#include "fimc-is-interface-library.h"
#include "fimc-is-param.h"

#define MCSC_ROUND_UP(x, d) \
	((d) * (((x) + ((d) - 1)) / (d)))

#define GET_MCSC_HW_CAP(hwip) \
	((hwip->priv_info) ? &((struct fimc_is_hw_mcsc *)hw_ip->priv_info)->cap : NULL)
#define GET_ENTRY_FROM_OUTPUT_ID(output_id) \
	(output_id + ENTRY_M0P)
#define GET_DJAG_ZOOM_RATIO(in, out) (u32)(((in * 1000 / out) << MCSC_PRECISION) / 1000)

#define INTERPOLATE_SHIFT			(12)
#define INTERPOLATE_NUMERATOR(Y1, Y2, diff_x_x1) \
	((((Y2) - (Y1)) * (diff_x_x1)) << INTERPOLATE_SHIFT)
#define GET_LINEAR_INTERPOLATE_VALUE(Y1, Y2, diff_x2_x1, diff_x_x1)		\
	(((INTERPOLATE_NUMERATOR((int)Y1, (int)Y2, diff_x_x1)) / (diff_x2_x1)) + \
					(((int)(Y1) << INTERPOLATE_SHIFT)))
#define RESTORE_SHIFT_VALUE(value) ((int)(value) >> INTERPOLATE_SHIFT)

enum mcsc_img_format {
	MCSC_YUV422_1P_YUYV = 0,
	MCSC_YUV422_1P_YVYU,
	MCSC_YUV422_1P_UYVY,
	MCSC_YUV422_1P_VYUY,
	MCSC_YUV422_2P_UFIRST,
	MCSC_YUV422_2P_VFIRST,
	MCSC_YUV422_3P,
	MCSC_YUV420_2P_UFIRST,
	MCSC_YUV420_2P_VFIRST,
	MCSC_YUV420_3P,
	MCSC_RGB_ARGB8888,
	MCSC_RGB_BGRA8888,
	MCSC_RGB_RGBA8888,
	MCSC_RGB_ABGR8888,
	MCSC_MONO_Y8,
};

enum mcsc_io_type {
	HW_MCSC_OTF_INPUT,
	HW_MCSC_OTF_OUTPUT,
	HW_MCSC_DMA_INPUT,
	HW_MCSC_DMA_OUTPUT,
};

enum mcsc_cap_enum {
	MCSC_CAP_NOT_SUPPORT = 0,
	MCSC_CAP_SUPPORT,
};

enum mcsc_shadow_ctrl {
	SHADOW_WRITE_START = 0,
	SHADOW_WRITE_FINISH,
};

enum yic_mode {
	TDNR_YIC_ENABLE = 0,
	TDNR_YIC_DISABLE,
};

enum mcsc_block_set_ctrl {
	TDNR_SET_DONE = 0,
	DJAG_SET_DONE,
	YSUM_SET_DONE,
	DSVRA_SET_DONE,
	ALL_BLOCK_SET_DONE = 0xFF,
};

struct scaler_setfile_contents {
	/* Brightness/Contrast control param */
	u32 y_offset;
	u32 y_gain;

	/* Hue/Saturation control param */
	u32 c_gain00;
	u32 c_gain01;
	u32 c_gain10;
	u32 c_gain11;
};

struct hw_api_scaler_setfile {
	u32 setfile_version;

	/* contents for Full/Narrow mode
	 * 0 : SCALER_OUTPUT_YUV_RANGE_FULL
	 * 1 : SCALER_OUTPUT_YUV_RANGE_NARROW
	 */
	struct scaler_setfile_contents contents[2];
#ifdef MCSC_DNR_USE_TUNING
	struct tdnr_setfile_contents tdnr_contents;
#endif
#ifdef MCSC_USE_DEJAG_TUNING_PARAM
	/* Setfile tuning parameters for DJAG (Lhotse)
	 * 0 : Scaling ratio = x1.0
	 * 1 : Scaling ratio = x1.1~x1.4
	 * 2 : Scaling ratio = x1.5~x2.0
	 * 3 : Scaling ratio = x2.1~
	 */
	struct djag_setfile_contents djag_contents[MAX_SCALINGRATIOINDEX_DEPENDED_CONFIGS];
#endif
};

/**
 * struct fimc_is_fw_mcsc_cap - capability of mcsc
 *  This Structure specified the spec of mcsc.
 * @hw_ver: type is hexa. eg. 1.22.0 -> 0b0001_0022_0000_0000
 * @max_output: the number of output port to support
 * <below fields has the value in enum mcsc_cap_enum>
 * @in_otf: capability of input otf
 * @in_dma: capability of input dma
 * @hw_fc: capability of hardware flow control
 * @out_otf: capability of output otf
 * @out_dma: capability of output dma
 * @out_hwfc: capability of output dma (each output)
 * @tdnr: capability of 3DNR feature
 */
struct fimc_is_hw_mcsc_cap {
	u32			hw_ver;
	u32			max_output;
	enum mcsc_cap_enum	in_otf;
	enum mcsc_cap_enum	in_dma;
	enum mcsc_cap_enum	hwfc;
	enum mcsc_cap_enum	out_otf[MCSC_OUTPUT_MAX];
	enum mcsc_cap_enum	out_dma[MCSC_OUTPUT_MAX];
	enum mcsc_cap_enum	out_hwfc[MCSC_OUTPUT_MAX];
	bool 			enable_shared_output;
	enum mcsc_cap_enum	tdnr;
	enum mcsc_cap_enum	djag;
	enum mcsc_cap_enum	ysum;
	enum mcsc_cap_enum	ds_vra;
};

struct fimc_is_hw_mcsc {
	struct	hw_api_scaler_setfile setfile[SENSOR_POSITION_END][FIMC_IS_MAX_SETFILE];
	struct	hw_api_scaler_setfile *applied_setfile[SENSOR_POSITION_END];
	struct	fimc_is_hw_mcsc_cap cap;

	u32	in_img_format;
	u32	out_img_format[MCSC_OUTPUT_MAX];
	bool	conv420_en[MCSC_OUTPUT_MAX];
	bool	rep_flag[FIMC_IS_STREAM_COUNT];
	int	yuv_range;
	u32	instance;
	ulong	out_en;		/* This flag save whether the capture video node of MCSC is opened or not. */
	ulong	blk_set_ctrl[FIMC_IS_STREAM_COUNT];
	u32	prev_hwfc_output_ids;

	/* for tdnr use */
	enum mcsc_output_index	tdnr_output;
	bool			tdnr_first;
	bool			tdnr_internal_buf;
	dma_addr_t		dvaddr_tdnr[2];
	enum tdnr_mode		cur_tdnr_mode;
	enum yic_mode		yic_en;
	u32			cur_noise_index;
	struct tdnr_configs	tdnr_cfgs;

	/* for Djag */
	u32			djag_input_source;
	struct djag_setfile_contents	djag_tunecfg;

	/* for full otf overflow recovery */
	struct is_param_region	*back_param;
	u32			back_lindex;
	u32			back_hindex;
};

int fimc_is_hw_mcsc_probe(struct fimc_is_hw_ip *hw_ip, struct fimc_is_interface *itf,
	struct fimc_is_interface_ischain *itfc, int id, const char *name);

int fimc_is_hw_mcsc_update_param(struct fimc_is_hw_ip *hw_ip,
	struct mcs_param *param, u32 lindex, u32 hindex, u32 instance);
void fimc_is_hw_mcsc_frame_done(struct fimc_is_hw_ip *hw_ip, struct fimc_is_frame *frame,
	int done_type);
int fimc_is_hw_mcsc_reset(struct fimc_is_hw_ip *hw_ip);
int fimc_is_hw_mcsc_clear_interrupt(struct fimc_is_hw_ip *hw_ip);

int fimc_is_hw_mcsc_otf_input(struct fimc_is_hw_ip *hw_ip, struct param_mcs_input *input,
	u32 instance);
int fimc_is_hw_mcsc_dma_input(struct fimc_is_hw_ip *hw_ip, struct param_mcs_input *input,
	u32 instance);
int fimc_is_hw_mcsc_poly_phase(struct fimc_is_hw_ip *hw_ip, struct param_mcs_input *input,
	struct param_mcs_output *output, u32 output_id, u32 instance);
int fimc_is_hw_mcsc_post_chain(struct fimc_is_hw_ip *hw_ip, struct param_mcs_input *input,
	struct param_mcs_output *output, u32 output_id, u32 instance);
int fimc_is_hw_mcsc_flip(struct fimc_is_hw_ip *hw_ip, struct param_mcs_output *output,
	u32 output_id, u32 instance);
int fimc_is_hw_mcsc_otf_output(struct fimc_is_hw_ip *hw_ip, struct param_mcs_output *output,
	u32 output_id, u32 instance);
int fimc_is_hw_mcsc_dma_output(struct fimc_is_hw_ip *hw_ip, struct param_mcs_output *output,
	u32 output_id, u32 instance);
int fimc_is_hw_mcsc_output_yuvrange(struct fimc_is_hw_ip *hw_ip, struct param_mcs_output *output,
	u32 output_id, u32 instance);
int fimc_is_hw_mcsc_hwfc_mode(struct fimc_is_hw_ip *hw_ip, struct param_mcs_input *input,
	u32 hwfc_output_ids, u32 dma_output_ids, u32 instance);
int fimc_is_hw_mcsc_hwfc_output(struct fimc_is_hw_ip *hw_ip, struct param_mcs_output *output,
	u32 output_id, u32 instance);

int fimc_is_hw_mcsc_adjust_input_img_fmt(u32 format, u32 plane, u32 order, u32 *img_format);
int fimc_is_hw_mcsc_adjust_output_img_fmt(u32 format, u32 plane, u32 order, u32 *img_format,
	bool *conv420_flag);
int fimc_is_hw_mcsc_check_format(enum mcsc_io_type type, u32 format, u32 bit_width,
	u32 width, u32 height);
u32 fimc_is_scaler_get_idle_status(void __iomem *base_addr, u32 hw_id);

void fimc_is_hw_mcsc_tdnr_init(struct fimc_is_hw_ip *hw_ip,
	struct mcs_param *mcs_param, u32 instance);
int fimc_is_hw_mcsc_update_tdnr_register(struct fimc_is_hw_ip *hw_ip,
	struct fimc_is_frame *frame,
	struct is_param_region *param,
	bool start_flag);
int fimc_is_hw_mcsc_recovery_tdnr_register(struct fimc_is_hw_ip *hw_ip,
		struct is_param_region *param, u32 instance);


void fimc_is_hw_mcsc_adjust_size_with_djag(struct fimc_is_hw_ip *hw_ip, struct param_mcs_input *input,
	struct fimc_is_hw_mcsc_cap *cap, u32 *x, u32 *y, u32 *width, u32 *height);
int fimc_is_hw_mcsc_update_djag_register(struct fimc_is_hw_ip *hw_ip,
		struct mcs_param *param,
		u32 instance);
int fimc_is_hw_mcsc_update_ysum_register(struct fimc_is_hw_ip *hw_ip,
	struct fimc_is_group *head, struct mcs_param *mcs_param,
	u32 instance, struct camera2_shot *shot);

int fimc_is_hw_mcsc_update_dsvra_register(struct fimc_is_hw_ip *hw_ip,
	struct fimc_is_group *head, struct mcs_param *mcs_param,
	u32 instance, struct camera2_shot *shot);
void fimc_is_scaler_set_lfro_mode_enable(void __iomem *base_addr, u32 lfro_enable, u32 lfro_total_fnum);
u32 fimc_is_scaler_get_lfro_mode_status(void __iomem *base_addr);

#ifdef DEBUG_HW_SIZE
#define hw_mcsc_check_size(hw_ip, param, instance, output_id) \
	fimc_is_hw_mcsc_check_size(hw_ip, param, instance, output_id)
#else
#define hw_mcsc_check_size(hw_ip, param, instance, output_id)
#endif
#endif
