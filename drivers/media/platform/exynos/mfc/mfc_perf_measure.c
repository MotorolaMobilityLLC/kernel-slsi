/*
 * drivers/media/platform/exynos/mfc/s5p_mfc_perf_measure.c
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "mfc_perf_measure.h"

#ifndef PERF_MEASURE

void s5p_mfc_perf_register(struct s5p_mfc_dev *dev) {}
void mfc_measure_init(void) {}
void mfc_measure_on(struct s5p_mfc_dev *dev) {}
void mfc_measure_off(struct s5p_mfc_dev *dev) {}
void mfc_measure_store(struct s5p_mfc_dev *dev, int diff) {}
void s5p_mfc_perf_print(void) {}

#else

#endif
