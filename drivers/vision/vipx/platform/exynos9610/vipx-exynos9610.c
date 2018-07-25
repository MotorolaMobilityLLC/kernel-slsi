/*
 * Samsung Exynos SoC series VIPX driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/delay.h>

#include "vipx-config.h"
#include "vipx-exynos.h"
#include "vipx-exynos9610.h"
#include "vipx-debug.h"

#define CLK_INDEX(name) VIPX_##name
#define REGISTER_CLK(name) [CLK_INDEX(name)] = {#name, NULL}

extern int vipx_clk_set_rate(struct device *dev, u32 index, ulong frequency);
extern ulong vipx_clk_get_rate(struct device *dev, u32 index);
extern int  vipx_clk_enable(struct device *dev, u32 index);
extern int vipx_clk_disable(struct device *dev, u32 index);

enum vipx_clk_index {
	CLK_INDEX(UMUX_CLKCMU_VIPX1_BUS),
	CLK_INDEX(GATE_VIPX1_QCH),
	CLK_INDEX(UMUX_CLKCMU_VIPX2_BUS),
	CLK_INDEX(GATE_VIPX2_QCH),
	CLK_INDEX(GATE_VIPX2_QCH_LOCAL)
};

struct vipx_clk vipx_clk_array[] = {
	REGISTER_CLK(UMUX_CLKCMU_VIPX1_BUS),
	REGISTER_CLK(GATE_VIPX1_QCH),
	REGISTER_CLK(UMUX_CLKCMU_VIPX2_BUS),
	REGISTER_CLK(GATE_VIPX2_QCH),
	REGISTER_CLK(GATE_VIPX2_QCH_LOCAL)
};

const u32 vipx_clk_array_size = ARRAY_SIZE(vipx_clk_array);

int vipx_exynos_clk_cfg(struct vipx_exynos *exynos)
{
	return 0;
}

int vipx_exynos_clk_on(struct vipx_exynos *exynos)
{
	vipx_clk_enable(exynos->dev, CLK_INDEX(UMUX_CLKCMU_VIPX1_BUS));
	vipx_clk_get_rate(exynos->dev, CLK_INDEX(UMUX_CLKCMU_VIPX1_BUS));

	vipx_clk_enable(exynos->dev, CLK_INDEX(GATE_VIPX1_QCH));
	vipx_clk_get_rate(exynos->dev, CLK_INDEX(GATE_VIPX1_QCH));

	vipx_clk_enable(exynos->dev, CLK_INDEX(UMUX_CLKCMU_VIPX2_BUS));
	vipx_clk_get_rate(exynos->dev, CLK_INDEX(UMUX_CLKCMU_VIPX2_BUS));

	vipx_clk_enable(exynos->dev, CLK_INDEX(GATE_VIPX2_QCH));
	vipx_clk_get_rate(exynos->dev, CLK_INDEX(GATE_VIPX2_QCH));

	vipx_clk_enable(exynos->dev, CLK_INDEX(GATE_VIPX2_QCH_LOCAL));
	vipx_clk_get_rate(exynos->dev, CLK_INDEX(GATE_VIPX2_QCH_LOCAL));

	vipx_info("[%s]\n", __func__);
	return 0;
}

int vipx_exynos_clk_off(struct vipx_exynos *exynos)
{
	vipx_clk_disable(exynos->dev, CLK_INDEX(UMUX_CLKCMU_VIPX1_BUS));
	vipx_clk_disable(exynos->dev, CLK_INDEX(GATE_VIPX1_QCH));
	vipx_clk_disable(exynos->dev, CLK_INDEX(UMUX_CLKCMU_VIPX2_BUS));
	vipx_clk_disable(exynos->dev, CLK_INDEX(GATE_VIPX2_QCH));
	vipx_clk_disable(exynos->dev, CLK_INDEX(GATE_VIPX2_QCH_LOCAL));

	vipx_info("[%s]\n", __func__);
	return 0;
}

int vipx_exynos_clk_dump(struct vipx_exynos *exynos)
{
	int ret = 0;

	return ret;
}

int dummy_vipx_exynos_clk_cfg(struct vipx_exynos *exynos) { return 0; }
int dummy_vipx_exynos_clk_on(struct vipx_exynos *exynos) { return 0; }
int dummy_vipx_exynos_clk_off(struct vipx_exynos *exynos) { return 0; }
int dummy_vipx_exynos_clk_dump(struct vipx_exynos *exynos) { return 0; }

const struct vipx_clk_ops vipx_clk_ops = {
#ifdef CONFIG_EXYNOS_VIPX_HARDWARE
	.clk_cfg	= vipx_exynos_clk_cfg,
	.clk_on		= vipx_exynos_clk_on,
	.clk_off	= vipx_exynos_clk_off,
	.clk_dump	= vipx_exynos_clk_dump
#else
	.clk_cfg	= dummy_vipx_exynos_clk_cfg,
	.clk_on		= dummy_vipx_exynos_clk_on,
	.clk_off	= dummy_vipx_exynos_clk_off,
	.clk_dump	= dummy_vipx_exynos_clk_dump
#endif
};

int vipx_exynos_ctl_reset(struct vipx_exynos *exynos, bool hold)
{
	u32 val;

	vipx_readl(exynos->regbase[VIPX_REG_CPU_SS1], &vipx_regs_cpu_ss1[VIPX_CPU_SS1_GLOBAL_CTRL], &val);
	val = val | 0xF1;
	vipx_writel(exynos->regbase[VIPX_REG_CPU_SS1], &vipx_regs_cpu_ss1[VIPX_CPU_SS1_GLOBAL_CTRL], val);
	mdelay(1);

	vipx_readl(exynos->regbase[VIPX_REG_CPU_SS2], &vipx_regs_cpu_ss2[VIPX_CPU_SS2_GLOBAL_CTRL], &val);
	val = val | 0xF1;
	vipx_writel(exynos->regbase[VIPX_REG_CPU_SS2], &vipx_regs_cpu_ss2[VIPX_CPU_SS2_GLOBAL_CTRL], val);
	mdelay(1);

	vipx_readl(exynos->regbase[VIPX_REG_CPU_SS1], &vipx_regs_cpu_ss1[VIPX_CPU_SS1_CPU_CTRL], &val);
	val = val | 0x1;
	vipx_writel(exynos->regbase[VIPX_REG_CPU_SS1], &vipx_regs_cpu_ss1[VIPX_CPU_SS1_CPU_CTRL], val);
	mdelay(1);

	vipx_readl(exynos->regbase[VIPX_REG_CPU_SS1], &vipx_regs_cpu_ss1[VIPX_CPU_SS1_CORTEX_CONTROL], &val);
	val = val | 0x1;
	vipx_writel(exynos->regbase[VIPX_REG_CPU_SS1], &vipx_regs_cpu_ss1[VIPX_CPU_SS1_CORTEX_CONTROL], val);
	mdelay(1);

	vipx_writel(exynos->regbase[VIPX_REG_CPU_SS1], &vipx_regs_cpu_ss1[VIPX_CPU_SS1_CPU_CTRL], 0x0);
	mdelay(1);

	/* QACTIVE */
	vipx_readl(exynos->regbase[VIPX_REG_CPU_SS1], &vipx_regs_cpu_ss1[VIPX_CPU_SS1_QCHANNEL], &val);
	val = val | 0x1;
	vipx_writel(exynos->regbase[VIPX_REG_CPU_SS1], &vipx_regs_cpu_ss1[VIPX_CPU_SS1_QCHANNEL], val);
	mdelay(1);

	vipx_readl(exynos->regbase[VIPX_REG_CPU_SS2], &vipx_regs_cpu_ss2[VIPX_CPU_SS2_QCHANNEL], &val);
	val = val | 0x1;
	vipx_writel(exynos->regbase[VIPX_REG_CPU_SS2], &vipx_regs_cpu_ss2[VIPX_CPU_SS2_QCHANNEL], val);
	mdelay(1);

	vipx_info("hold(%d)\n", hold);
	return 0;
}

int vipx_exynos_ctl_dump(struct vipx_exynos *exynos, u32 instance)
{

	vipx_info("instance(%d)\n", instance);
	return 0;
}

void *vipx_exynos_ctl_remap(struct vipx_exynos *exynos, u32 instance)
{
	BUG_ON(!exynos);
	BUG_ON(instance >= VIPX_REG_MAX_CNT);

	vipx_info("instance(%d)\n", instance);
	return exynos->regbase[instance];
}

int vipx_exynos_ctl_unmap(struct vipx_exynos *exynos, u32 instance, void *base)
{
	BUG_ON(!exynos);
	BUG_ON(!base);
	BUG_ON(instance >= VIPX_REG_MAX_CNT);

	vipx_info("instance(%d)\n", instance);
	return 0;
}

int vipx_exynos_ctl_trigger(struct vipx_exynos *exynos, u32 chain_id)
{

	vipx_info("chain_id(%d)\n", chain_id);
	return 0;
}

int vipx_exynos_ctl_start(struct vipx_exynos *exynos, u32 chain_id)
{
	vipx_writel(exynos->regbase[VIPX_REG_CPU_SS1], &vipx_regs_cpu_ss1[VIPX_CPU_SS1_CORTEX_CONTROL], 0x0);

	vipx_info("chain_id(%d) %x %x\n", chain_id,exynos->regbase[VIPX_REG_CPU_SS1],exynos->regbase[VIPX_REG_CPU_SS2]);
	return 0;
}

int dummy_vipx_exynos_ctl_reset(struct vipx_exynos *exynos, bool hold) { return 0; }
int dummy_vipx_exynos_ctl_dump(struct vipx_exynos *exynos, u32 instance) { return 0; }
void *dummy_vipx_exynos_ctl_remap(struct vipx_exynos *exynos, u32 instance) { return NULL; }
int dummy_vipx_exynos_ctl_unmap(struct vipx_exynos *exynos, u32 instance, void *base) { return 0; }
int dummy_vipx_exynos_ctl_trigger(struct vipx_exynos *exynos, u32 chain_id) { return 0; }
int dummy_vipx_exynos_ctl_start(struct vipx_exynos *exynos, u32 chain_id) { return 0; }

const struct vipx_ctl_ops vipx_ctl_ops = {
#ifdef CONFIG_EXYNOS_VIPX_HARDWARE
	.ctl_reset	= vipx_exynos_ctl_reset,
	.ctl_dump	= vipx_exynos_ctl_dump,
	.ctl_remap	= vipx_exynos_ctl_remap,
	.ctl_unmap	= vipx_exynos_ctl_unmap,
	.ctl_trigger	= vipx_exynos_ctl_trigger,
	.ctl_start	= vipx_exynos_ctl_start,
#else
	.ctl_reset	= dummy_vipx_exynos_ctl_reset,
	.ctl_dump	= dummy_vipx_exynos_ctl_dump,
	.ctl_remap	= dummy_vipx_exynos_ctl_remap,
	.ctl_unmap	= dummy_vipx_exynos_ctl_unmap,
	.ctl_trigger	= dummy_vipx_exynos_ctl_trigger,
	.ctl_start	= dummy_vipx_exynos_ctl_start,
#endif
};
