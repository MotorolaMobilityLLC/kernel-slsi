/*
 * Samsung Exynos SoC series VIPX driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef VIPX_EXYNOS9610_H_
#define VIPX_EXYNOS9610_H_

enum vipx_reg_cpu_ss1_name {
	VIPX_CPU_SS1_VERSION_ID,
	VIPX_CPU_SS1_QCHANNEL,
	VIPX_CPU_SS1_IRQ_VIP_TO_HOST,
	VIPX_CPU_SS1_IRQ0_HOST_TO_VIP,
	VIPX_CPU_SS1_IRQ1_HOST_TO_VIP,
	VIPX_CPU_SS1_GLOBAL_CTRL,
	VIPX_CPU_SS1_CORTEX_CONTROL,
	VIPX_CPU_SS1_CPU_CTRL,
};

enum vipx_reg_cpu_ss2_name {
	VIPX_CPU_SS2_QCHANNEL,
	VIPX_CPU_SS2_GLOBAL_CTRL,
};

struct vipx_reg vipx_regs_cpu_ss1[] = {
	{0x00000, "VERSION_ID"},
	{0x00004, "SS1_QCHANNEL"},
	{0x00008, "SS1_IRQ_VIP_TO_HOST"},
	{0x0000C, "SS1_IRQ0_HOST_TO_VIP"},
	{0x00010, "SS1_IRQ1_HOST_TO_VIP"},
	{0x00014, "SS1_GLOBAL_CTRL"},
	{0x00018, "SS1_CORTEX_CONTROL"},
	{0x0001C, "SS1_CPU_CTRL"},
};

struct vipx_reg vipx_regs_cpu_ss2[] = {
	{0x00000, "SS2_QCHANNEL"},
	{0x00004, "SS2_GLOBAL_CTRL"},
};

#endif
