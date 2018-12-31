/* linux/drivers/video/fbdev/exynos/dpu/dqe_common.h
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *      http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SAMSUNG_DQE_H__
#define __SAMSUNG_DQE_H__

#include "decon.h"
#if defined(CONFIG_SOC_EXYNOS9610)
#include "./cal_9610/regs-dqe.h"
#endif

#define dqe_err(fmt, ...)							\
	do {									\
		if (dqe_log_level >= 3) {					\
			pr_err(pr_fmt(fmt), ##__VA_ARGS__);			\
		}								\
	} while (0)

#define dqe_warn(fmt, ...)							\
	do {									\
		if (dqe_log_level >= 4) {					\
			pr_warn(pr_fmt(fmt), ##__VA_ARGS__);			\
		}								\
	} while (0)

#define dqe_info(fmt, ...)							\
	do {									\
		if (dqe_log_level >= 6)					\
			pr_info(pr_fmt(fmt), ##__VA_ARGS__);			\
	} while (0)

#define dqe_dbg(fmt, ...)							\
	do {									\
		if (dqe_log_level >= 7)					\
			pr_info(pr_fmt(fmt), ##__VA_ARGS__);			\
	} while (0)

static inline u32 dqe_read(u32 reg_id)
{
	struct decon_device *decon = get_decon_drvdata(0);

	return readl(decon->res.regs + DQE_BASE + reg_id);
}

static inline u32 dqe_read_mask(u32 reg_id, u32 mask)
{
	u32 val = dqe_read(reg_id);

	val &= (mask);
	return val;
}

static inline void dqe_write(u32 reg_id, u32 val)
{
	struct decon_device *decon = get_decon_drvdata(0);

	writel(val, decon->res.regs + DQE_BASE + reg_id);
}

static inline void dqe_write_mask(u32 reg_id, u32 val, u32 mask)
{
	struct decon_device *decon = get_decon_drvdata(0);
	u32 old = dqe_read(reg_id);

	val = (val & mask) | (old & ~mask);
	writel(val, decon->res.regs + DQE_BASE + reg_id);
}

struct dqe_reg_dump {
	u32 addr;
	u32 val;
};

struct dqe_ctx {
	struct dqe_reg_dump cgc[DQECGC1LUT_MAX + DQECGC2LUT_MAX];
	struct dqe_reg_dump gamma[DQEGAMMALUT_MAX];
	struct dqe_reg_dump hsc[DQEHSCLUT_MAX];
	u32 cgc_on;
	u32 gamma_on;
	u32 hsc_on;
	u32 hsc_control;
	bool need_udpate;
	u32 color_mode;
	u32 night_light_on;
};

struct dqe_device {
	struct device *dev;
	struct decon_device *decon;
	struct mutex lock;
	struct dqe_ctx ctx;
};

/* CAL APIs list */
void dqe_reg_module_on_off(bool en_she, bool en_cgc, bool en_gamma,
		bool en_hsc, bool en_aps);
void dqe_reg_module_reset(bool en_hsc, bool en_aps, bool en_rst);
void dqe_reg_start(u32 id, struct decon_lcd *lcd_info);
void dqe_reg_stop(u32 id);

void dqe_reg_set_she_on(u32 on);
void dqe_reg_set_cgc_on(u32 on);
u32 dqe_reg_get_cgc_on(void);
void dqe_reg_set_gamma_on(u32 on);
u32 dqe_reg_get_gamma_on(void);
void dqe_reg_set_hsc_on(u32 on);
u32 dqe_reg_get_hsc_on(void);
void dqe_reg_set_hsc_pphc_on(u32 on);
void dqe_reg_set_hsc_ppsc_on(u32 on);
void dqe_reg_set_hsc_control(u32 val);
void dqe_reg_set_hsc_control_all_reset(void);
u32 dqe_reg_get_hsc_control(void);
void dqe_reg_set_hsc_full_pxl_num(struct decon_lcd *lcd_info);
u32 dqe_reg_get_hsc_full_pxl_num(void);
void dqe_reg_set_aps_on(u32 on);
void dqe_reg_hsc_sw_reset(u32 en);
void dqe_reg_aps_sw_reset(u32 en);
void dqe_reg_reset(u32 en);
void dqe_reg_set_gammagray_on(u32 on);
void dqe_reg_lpd_mode_exit(u32 en);

void dqe_reg_module_on_off(bool en_she, bool en_cgc, bool en_gamma,
		bool en_hsc, bool en_aps);
void dqe_reg_module_reset(bool en_hsc, bool en_aps, bool en_rst);

void dqe_reg_set_img_size0(u32 width, u32 height);
void dqe_reg_set_img_size1(u32 width, u32 height);
void dqe_reg_set_img_size2(u32 width, u32 height);

/* DQE_HSC register set */
void dqe_reg_set_hsc_ppsc_on(u32 en);
void dqe_reg_set_hsc_ycomp_on(u32 en);
void dqe_reg_set_hsc_tsc_on(u32 en);
void dqe_reg_set_hsc_dither_on(u32 en);
void dqe_reg_set_hsc_pphc_on(u32 en);
void dqe_reg_set_hsc_skin_on(u32 en);
void dqe_reg_set_hsc_ppscgain_rgb(u32 r, u32 g, u32 b);
void dqe_reg_set_hsc_ppsc_gain_cmy(u32 c, u32 m, u32 y);
void dqe_reg_set_hsc_alphascale_shift(u32 alpha_shift1, u32 alpha_shift2,
		u32 alpha_scale);
void dqe_reg_set_hsc_poly_curve0(u32 curve1, u32 curve2, u32 curve3, u32 curve4);
void dqe_reg_set_hsc_poly_curve1(u32 curve5, u32 curve6, u32 curve7, u32 curve8);
void dqe_reg_set_hsc_skin(u32 skin_h1, u32 skin_h2, u32 skin_s1, u32 skin_s2);
void dqe_reg_set_hsc_pphcgain_rgb(u32 r, u32 g, u32 b);
void dqe_reg_set_hsc_pphcgain_cmy(u32 c, u32 m, u32 y);
void dqe_reg_set_hsc_tsc_ycomp(u32 ratio, u32 gain);

void decon_dqe_enable(struct decon_device *decon);
void decon_dqe_disable(struct decon_device *decon);
int decon_dqe_create_interface(struct decon_device *decon);

int decon_dqe_set_color_mode(struct decon_color_mode_with_render_intent_info *color_mode);
int decon_dqe_set_color_transform(struct decon_color_transform_info *transform);

#endif
