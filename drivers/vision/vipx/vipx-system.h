/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef VIPX_SYSTEM_H_
#define VIPX_SYSTEM_H_

#include <linux/platform_device.h>

#include "vipx-interface.h"
#include "vipx-memory.h"
#include "vipx-exynos.h"
#include "vipx-binary.h"

struct vipx_system {
	struct platform_device			*pdev;
	void __iomem				*reg_ss[VIPX_REG_MAX_CNT];
	void __iomem				*itcm;
	void __iomem				*dtcm;
	resource_size_t 			reg_ss_size[VIPX_REG_MAX_CNT];
	resource_size_t 			itcm_size;
	resource_size_t 			dtcm_size;
	int					irq0;
	int					irq1;

	u32					cam_qos;
	u32					mif_qos;

	struct vipx_interface 			interface;
	struct vipx_memory			memory;
	struct vipx_exynos			exynos;
	struct vipx_binary			binary;
};

int vipx_system_probe(struct vipx_system *system, struct platform_device *pdev);
int vipx_system_open(struct vipx_system *system);
int vipx_system_close(struct vipx_system *system);
int vipx_system_resume(struct vipx_system *system, u32 mode);
int vipx_system_suspend(struct vipx_system *system);
int vipx_system_start(struct vipx_system *system);
int vipx_system_stop(struct vipx_system *system);
int vipx_system_fw_bootup(struct vipx_system *system);

#endif
