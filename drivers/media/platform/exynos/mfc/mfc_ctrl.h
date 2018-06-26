/*
 * drivers/media/platform/exynos/mfc/mfc_ctrl.h
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_CTRL_H
#define __MFC_CTRL_H __FILE__

#include "mfc_common.h"

int mfc_init_hw(struct mfc_dev *dev);
void mfc_deinit_hw(struct mfc_dev *dev);

int mfc_sleep(struct mfc_dev *dev);
int mfc_wakeup(struct mfc_dev *dev);

#endif /* __MFC_CTRL_H */
