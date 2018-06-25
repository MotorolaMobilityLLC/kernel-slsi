/*
 * drivers/media/platform/exynos/mfc/s5p_mfc_perf_measure.h
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_PERF_MEASURE_H
#define __MFC_PERF_MEASURE_H __FILE__

#include <linux/clk.h>

#include "mfc_reg.h"

void s5p_mfc_perf_register(struct s5p_mfc_dev *dev);
void mfc_measure_init(void);
void mfc_measure_on(struct s5p_mfc_dev *dev);
void mfc_measure_off(struct s5p_mfc_dev *dev);
void mfc_measure_store(struct s5p_mfc_dev *dev, int diff);
void s5p_mfc_perf_print(void);

//#define PERF_MEASURE

#ifndef PERF_MEASURE

static inline void s5p_mfc_perf_init(struct s5p_mfc_dev *dev) {}
static inline void s5p_mfc_perf_cancel_drv_margin(struct s5p_mfc_dev *dev) {}
static inline void s5p_mfc_perf_measure_on(struct s5p_mfc_dev *dev) {}
static inline void s5p_mfc_perf_measure_off(struct s5p_mfc_dev *dev) {}

#else

extern unsigned int perf_measure_option;

static inline void s5p_mfc_perf_init(struct s5p_mfc_dev *dev)
{
	dev->perf.new_start = 0;
	dev->perf.count = 0;
	dev->perf.drv_margin = 0;

	mfc_measure_init();

	mfc_info_dev("MFC frequency : %ld\n", clk_get_rate(dev->pm.clock));
}

static inline void s5p_mfc_perf_cancel_drv_margin(struct s5p_mfc_dev *dev)
{
	dev->perf.drv_margin = 0;
}

static inline void s5p_mfc_perf_measure_on(struct s5p_mfc_dev *dev)
{
	int diff;

	if (dev->perf.drv_margin) {
		do_gettimeofday(&dev->perf.end);

		diff = (dev->perf.end.tv_sec * 1000000 + dev->perf.end.tv_usec)
			- (dev->perf.begin.tv_sec * 1000000 + dev->perf.begin.tv_usec);

		mfc_info_dev("IRQ -> NAL_START time(ms) = %03d.%03d\n", diff / 1000, diff % 1000);

		dev->perf.drv_margin = 0;
	}

	do_gettimeofday(&dev->perf.begin);

	mfc_measure_on(dev);

	dev->perf.new_start = 1;
	dev->perf.count++;
}

static inline void s5p_mfc_perf_measure_off(struct s5p_mfc_dev *dev)
{
	unsigned int diff;

	if ((dev->perf.new_start) && (dev->perf.count > 0)) {
		mfc_measure_off(dev);

		do_gettimeofday(&dev->perf.end);

		diff = (dev->perf.end.tv_sec * 1000000 + dev->perf.end.tv_usec)
			- (dev->perf.begin.tv_sec * 1000000 + dev->perf.begin.tv_usec);

		mfc_measure_store(dev, diff);

		mfc_debug(3, "uDECtype :%d, uENCtype :%d, codectype :%d\n",
			s5p_mfc_get_dec_frame_type(), s5p_mfc_get_enc_slice_type(), MFC_READL(S5P_FIMV_CODEC_TYPE));

		dev->perf.drv_margin = 1;

		do_gettimeofday(&dev->perf.begin);
	}

	dev->perf.new_start = 0;
}

#endif

#endif /* __MFC_PERF_MEASURE_H */
